//! The x86 lifter against the x86 interpreter, over a generated corpus.
//!
//! CLAUDE.md, "CPU cores": an IR frontend *is differentially tested against the
//! interpreter forever*, and the interpreter is the oracle. The comparison
//! itself lives in [`cpu::x86::differential`], because
//! `fuzz_targets/x86_lift.rs` drives the same functions; this file is the half
//! of it that runs in a plain `cargo test`, offline, with no fuzzer and no
//! downloaded corpus.
//!
//! The programs are generated rather than written, because the bugs a
//! hand-written suite finds are the ones its author thought of. Generation is a
//! 64-bit LCG with a fixed seed, so the corpus is the same on every machine and
//! in every run (`ROADMAP.md` §0) — a failure here is reproducible from the
//! seed printed beside it, and a new failure is a real regression rather than a
//! different draw.
//!
//! # Six frontends, not one — in three worlds
//!
//! Every case runs under all three [`Shape`]s crossed with both [`Flags`]
//! policies, because each of those emits **different IR from the same bytes**:
//!
//! * a `Shape::Trace` conditional jump is a `brcond` and a side exit where a
//!   `Shape::BasicBlock` one is a `setcond`/`movcond` pair;
//! * `Flags::Elide` leaves a flag out of a boundary's live map when the next
//!   instruction overwrites it and cannot fault, and dead-code elimination then
//!   deletes the arithmetic behind it, where `Flags::Eager` keeps all six
//!   everywhere.
//!
//! All six must agree with the one interpreter on every column, so a
//! disagreement between two of them is a frontend bug wherever it shows up.
//! [`Smc`] is the third axis, exercised the same way by the self-modifying-code
//! cases below.
//!
//! The whole cross product runs in each of the three **worlds** `World::of`
//! accepts — 32-bit protected mode, the same with `CR0.PG` set, and long
//! mode — and a world is not a seventh policy: it changes the *bytes*. `40`-`4f`
//! are `INC` and `DEC` in two of them and the `REX` prefix in the third, the
//! register file above seven exists only in the third, and three addressing
//! modes have no spelling outside it. So the long-mode sweeps generate from
//! [`synthesize64`] rather than from [`synthesize`]; a sweep that reused the
//! 32-bit encodings would run and would be measuring the wrong instruction set.

#![cfg(all(feature = "cpu-x86-lift", feature = "jit"))]

use rsemu::cpu::x86::differential::{
    Case, Verdict, compare, compare_cached, measure_cached, synthesize, synthesize64,
};
use rsemu::cpu::x86::lift::{Flags, Shape, Smc};

/// A 64-bit linear congruential generator — Knuth's MMIX multiplier and
/// increment.
///
/// A named, fixed generator rather than anything from the host: the corpus has
/// to be identical everywhere, and a hash of the run index would give a
/// different sequence the day the hasher changes.
struct Lcg(u64);

impl Lcg {
    fn next(&mut self) -> u64 {
        self.0 = self
            .0
            .wrapping_mul(6_364_136_223_846_793_005)
            .wrapping_add(1_442_695_040_888_963_407);
        self.0
    }
}

/// One generated program of up to `len` instructions, terminated by a byte the
/// lifter refuses so a run cannot fall off the end into the data window.
fn generate(rng: &mut Lcg, len: usize) -> Vec<u8> {
    generate_in(rng, len, World::Protected)
}

/// Which world a generated program is written for — this file's own name for
/// the choice, not `lift::World`.
///
/// Not a policy: the two produce **different bytes**, because `40`-`4f` are
/// `INC` and `DEC` in one and the `REX` prefix in the other, and three
/// addressing modes exist in only one of them. Paging is *not* one of these,
/// because it changes no encoding: a paged sweep runs the same bytes.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
enum World {
    /// 32-bit protected mode, with or without paging.
    Protected,
    /// Long mode, which is paged by construction.
    Long,
}

fn generate_in(rng: &mut Lcg, len: usize, world: World) -> Vec<u8> {
    let mut out = Vec::new();
    for _ in 0..len {
        let bits = rng.next();
        let (form, fields) = ((bits >> 40) as u32, bits as u32);
        out.extend_from_slice(&match world {
            World::Protected => synthesize(form, fields),
            World::Long => synthesize64(form, fields),
        });
    }
    // `HLT`: outside the subset, so the block ends cleanly rather than lifting
    // whatever the data window happens to hold.
    out.push(0xf4);
    out
}

/// What a sweep covered. A [`Verdict::Trapped`] is a real result — both engines
/// stopped at the same instruction in the same state — and a
/// [`Verdict::Nothing`] means the first instruction was outside the subset, so
/// counting them is how the test can assert it is exercising the lifter rather
/// than measuring an empty block a thousand times.
#[derive(Default, Debug)]
struct Coverage {
    agreed: usize,
    trapped: usize,
    nothing: usize,
    insns: usize,
}

fn sweep(seed: u64, count: usize, shape: Shape, flags: Flags) -> Coverage {
    sweep_in(seed, count, shape, flags, false)
}

/// The same sweep, in a chosen world.
///
/// `paged` puts `CR0.PG` on: the guest's linear addresses are unchanged and
/// every one of them names a different physical page, the entry translation is
/// charged on the way in, and each access walks the tables through the
/// interpreter's own memory path. Every column stays the same, which is the
/// point — a walk's ticks and the accessed and dirty bits it writes are both
/// compared.
fn sweep_in(seed: u64, count: usize, shape: Shape, flags: Flags, paged: bool) -> Coverage {
    let mut rng = Lcg(seed);
    let mut cov = Coverage::default();
    for n in 0..count {
        let len = 1 + (rng.next() % 12) as usize;
        let case = Case::seeded(generate(&mut rng, len))
            .with_shape(shape)
            .with_flags(flags);
        let case = if paged { case.paged() } else { case };
        match compare(&case) {
            Ok(Verdict::Agreed { insns, .. }) => {
                cov.agreed += 1;
                cov.insns += insns;
            }
            Ok(Verdict::Trapped { insns }) => {
                cov.trapped += 1;
                cov.insns += insns;
            }
            Ok(Verdict::Nothing) => cov.nothing += 1,
            Err(e) => panic!(
                "case {n} of seed {seed:#x} diverged under {shape:?}/{flags:?}{}:\n{e}",
                if paged { "/paged" } else { "" }
            ),
        }
    }
    cov
}

/// The same sweep in **long mode**, over a corpus written for it.
///
/// The corpus has to change with the world or the coverage claim is empty:
/// `synthesize`'s bytes decode in long mode, but `40`-`4f` become a `REX`
/// prefix on the instruction after them and the register file above seven is
/// never named. [`synthesize64`] writes the encodings this world actually
/// has — `REX` on nearly everything, `R8`-`R15`, a 64-bit operand size beside
/// a 32-bit one, and `RIP`-relative addressing.
fn sweep_long(seed: u64, count: usize, shape: Shape, flags: Flags) -> Coverage {
    let mut rng = Lcg(seed);
    let mut cov = Coverage::default();
    for n in 0..count {
        let len = 1 + (rng.next() % 12) as usize;
        let case = Case::seeded(generate_in(&mut rng, len, World::Long))
            .with_shape(shape)
            .with_flags(flags)
            .long();
        match compare(&case) {
            Ok(Verdict::Agreed { insns, .. }) => {
                cov.agreed += 1;
                cov.insns += insns;
            }
            Ok(Verdict::Trapped { insns }) => {
                cov.trapped += 1;
                cov.insns += insns;
            }
            Ok(Verdict::Nothing) => cov.nothing += 1,
            Err(e) => {
                panic!("long case {n} of seed {seed:#x} diverged under {shape:?}/{flags:?}:\n{e}")
            }
        }
    }
    cov
}

#[test]
fn a_generated_corpus_agrees_with_the_interpreter_under_every_shape_and_flag_policy() {
    // The cross product is the point: the same bytes lift to different IR under
    // each of the six, and every one of them is compared against the one
    // interpreter.
    let mut total = Coverage::default();
    for (n, shape) in [Shape::BasicBlock, Shape::Extended, Shape::Trace]
        .into_iter()
        .enumerate()
    {
        for (m, flags) in [Flags::Eager, Flags::Elide].into_iter().enumerate() {
            let seed = 0x5eed_0000 + (n as u64) * 16 + m as u64;
            let cov = sweep(seed, 600, shape, flags);
            assert!(
                cov.agreed > 300,
                "{shape:?}/{flags:?}: only {} of 600 cases ran to completion ({} trapped, {} \
                 lifted nothing) — the generator has stopped producing programs in the subset",
                cov.agreed,
                cov.trapped,
                cov.nothing
            );
            total.agreed += cov.agreed;
            total.trapped += cov.trapped;
            total.nothing += cov.nothing;
            total.insns += cov.insns;
        }
    }
    assert!(
        total.trapped > 0,
        "no case reached a fault, so the precise-state column was never tested"
    );
    assert!(
        total.insns > 5_000,
        "only {} guest instructions retired across the whole corpus",
        total.insns
    );
}

/// The corpus again, with `CR0.PG` set.
///
/// **A fourth world rather than a seventh policy**, and the reason it is here
/// at all: `World::of` refused paging outright until this round, so every
/// claim the harness made about the lifter was a claim about unpaged code.
/// Widening the lifter's world without widening the corpus would have left
/// the coverage claim empty.
///
/// What is different about a paged case, column by column:
///
/// * **the memory path is the interpreter's own** — `Exec::read_mem` and
///   `Exec::write_mem`, the functions `Exec::step` calls — rather than this
///   harness's segment check plus one bus cycle, because a walk's tick cost
///   and its accessed-bit write-back are not things a second implementation
///   should be trusted to reproduce;
/// * **the entry fetch translation is charged on the way in**, on every block,
///   which is the contract that otherwise looks like a working JIT with a
///   clock short by one walk per entry;
/// * **RAM is compared over the page tables too**, so the accessed and dirty
///   bits both engines wrote are a compared column and not an assumption;
/// * **the block is keyed on the physical page its entry resolved to**, and
///   linear and physical are deliberately different numbers in this machine.
///
/// `Smc::EndBlock` is forced by `Case::paged`, because the in-block guard
/// compares linear pages and two of them may alias one physical page — the
/// lifter refuses the other combination.
#[test]
fn the_same_corpus_agrees_with_paging_on() {
    let mut total = Coverage::default();
    for (n, shape) in [Shape::BasicBlock, Shape::Extended, Shape::Trace]
        .into_iter()
        .enumerate()
    {
        for (m, flags) in [Flags::Eager, Flags::Elide].into_iter().enumerate() {
            let seed = 0x9a6e_0000 + (n as u64) * 16 + m as u64;
            let cov = sweep_in(seed, 400, shape, flags, true);
            assert!(
                cov.agreed > 200,
                "{shape:?}/{flags:?} paged: only {} of 400 cases ran to completion ({} trapped, \
                 {} lifted nothing)",
                cov.agreed,
                cov.trapped,
                cov.nothing
            );
            total.agreed += cov.agreed;
            total.trapped += cov.trapped;
            total.nothing += cov.nothing;
            total.insns += cov.insns;
        }
    }
    assert!(
        total.trapped > 0,
        "no paged case reached a fault, so the precise-state column was never tested under paging"
    );
    assert!(
        total.insns > 3_000,
        "only {} guest instructions retired across the paged corpus",
        total.insns
    );
}

/// The same program, once with paging off and once with it on, must retire the
/// same instructions and leave the same registers — and must **not** charge the
/// same ticks, because a walk costs bus cycles.
///
/// The second half is what makes the first half worth asserting. A paged run
/// that cost exactly what an unpaged one cost would mean no walk had happened,
/// which is the shape of a JIT that skipped the entry translation.
#[test]
fn paging_changes_the_clock_and_nothing_else() {
    // mov eax, [ebx] ; add eax, ecx ; mov [ebx+4], eax ; hlt
    let program = vec![0x8b, 0x03, 0x01, 0xc8, 0x89, 0x43, 0x04, 0xf4];
    let flat = Case::seeded(program.clone());
    let paged = Case::seeded(program).paged();
    let (a, b) = match (compare(&flat), compare(&paged)) {
        (Ok(a), Ok(b)) => (a, b),
        (a, b) => panic!("{a:?}\n{b:?}"),
    };
    let (
        Verdict::Agreed {
            insns: ia,
            ticks: ta,
        },
        Verdict::Agreed {
            insns: ib,
            ticks: tb,
        },
    ) = (a, b)
    else {
        panic!("both worlds run this program to completion");
    };
    assert_eq!(ia, ib, "the same program retires the same instructions");
    assert!(
        tb > ta,
        "paging cost nothing ({ta} ticks unpaged, {tb} paged), so no page-table walk happened"
    );
}

/// The corpus in **long mode**, under all six shape-and-flag frontends.
///
/// The third of the three worlds, and the one paging was a prerequisite for
/// rather than an alternative to: `EFER.LMA` is set only when `CR0.PG` goes on with
/// `EFER.LME`, and IA-32e paging requires `CR4.PAE` (*Intel SDM* volume 3
/// §9.8.5), so a processor in long mode is a processor with paging on. The
/// walk here is therefore four levels of eight-byte entries rather than two of
/// four, and every column the paged sweep compares is compared again over it.
///
/// What is new in this world, beyond the tables:
///
/// * **sixteen registers**, all sixty-four bits wide and all compared —
///   `R8`-`R15` are unreachable without a `REX` prefix, so a 32-bit corpus
///   cannot touch them;
/// * **a 64-bit operand size**, at which `ADD`'s carry has no bit above it to
///   be read from, a shift count masks to six bits rather than five, and
///   `MUL`/`IMUL` need the double-width product `Opcode::MULU2`/`MULS2`
///   carries — the two opcodes a 32-bit subset never reached;
/// * **the doubleword write's zero-extension**, which is the opposite of what
///   a byte or word write does to the bits above it, and which half the
///   generated forms drop `REX.W` to exercise;
/// * **`RIP`-relative addressing**, the one mode whose effective address
///   depends on the instruction's own length.
///
/// What is *not* in this world is a computed near transfer: `RET`, `JMP r/m`
/// and `CALL r/m` fault on a non-canonical target and a block cannot deliver
/// that exception, so `lift` refuses them here and the interpreter takes them.
#[test]
fn the_same_corpus_agrees_in_long_mode() {
    let mut total = Coverage::default();
    for (n, shape) in [Shape::BasicBlock, Shape::Extended, Shape::Trace]
        .into_iter()
        .enumerate()
    {
        for (m, flags) in [Flags::Eager, Flags::Elide].into_iter().enumerate() {
            let seed = 0x6400_0000 + (n as u64) * 16 + m as u64;
            let cov = sweep_long(seed, 400, shape, flags);
            assert!(
                cov.agreed > 200,
                "{shape:?}/{flags:?} long: only {} of 400 cases ran to completion ({} trapped, \
                 {} lifted nothing)",
                cov.agreed,
                cov.trapped,
                cov.nothing
            );
            total.agreed += cov.agreed;
            total.trapped += cov.trapped;
            total.nothing += cov.nothing;
            total.insns += cov.insns;
        }
    }
    assert!(
        total.trapped > 0,
        "no long-mode case reached a fault, so the precise-state column was never tested there"
    );
    assert!(
        total.insns > 3_000,
        "only {} guest instructions retired across the long-mode corpus",
        total.insns
    );
}

/// The long-mode corpus through the translation runtime rather than one block
/// at a time.
///
/// The entry translation is a **four-level** walk here and it happens on every
/// execution, including a block served from the cache and one reached by
/// following a patched exit — so a block that skipped its walk the second time
/// round is a tick divergence here and nowhere else.
#[test]
fn the_long_mode_corpus_agrees_through_the_cached_and_chained_runtime() {
    let mut rng = Lcg(0x6400_beef);
    let (mut agreed, mut trapped, mut nothing) = (0usize, 0usize, 0usize);
    for n in 0..400 {
        let len = 1 + (rng.next() % 12) as usize;
        let case = Case::seeded(generate_in(&mut rng, len, World::Long)).long();
        match compare_cached(&case, 32) {
            Ok(Verdict::Agreed { .. }) => agreed += 1,
            Ok(Verdict::Trapped { .. }) => trapped += 1,
            Ok(Verdict::Nothing) => nothing += 1,
            Err(e) => panic!("cached long case {n} diverged:\n{e}"),
        }
    }
    assert!(
        agreed > 200,
        "only {agreed} of 400 cached long cases ran to completion ({trapped} trapped, {nothing} \
         lifted nothing)"
    );
    assert!(trapped > 0, "no cached long case reached a fault");
}

/// A store into a running block's own page, in long mode.
///
/// The same mechanism the paged case proves, with the store's linear page and
/// the block's physical page still different numbers and the registers now
/// sixty-four bits wide.
#[test]
fn a_long_mode_store_into_a_running_blocks_own_page_is_honoured() {
    for shape in [Shape::BasicBlock, Shape::Extended, Shape::Trace] {
        // `mov [rax], bl` with RAX pointing at the instruction two ahead, then
        // that instruction. `48 ff c6` is `inc rsi`, and `0xc7` turns it into
        // `inc rdi`.
        let program = vec![
            0x88, 0x18, // mov [rax], bl
            0x90, 0x90, // nop, nop
            0x48, 0xff, 0xc6, // inc rsi — the byte at 6 is overwritten
            0xf4,
        ];
        let case = Case::new(program)
            .with_reg(0, rsemu::cpu::x86::differential::BASE + 6)
            .with_reg(3, 0xc7)
            .with_shape(shape)
            .long();
        let run = measure_cached(&case, 32).unwrap_or_else(|e| panic!("{shape:?} long: {e}"));
        assert!(
            matches!(run.verdict, Verdict::Agreed { .. }),
            "{shape:?} long: {:?}",
            run.verdict
        );
        assert!(
            run.smc > 0,
            "{shape:?} long: nothing was invalidated, so the store was never matched against the \
             block's physical page"
        );
    }
}

#[test]
fn the_same_corpus_agrees_through_the_cached_and_chained_runtime() {
    // The second harness: many blocks, served from a cache, exits patched, the
    // instruction bytes read out of guest RAM, and every access through the
    // software TLB. It covers what one freshly lifted block cannot.
    let mut rng = Lcg(0x5eed_beef);
    let (mut agreed, mut trapped, mut nothing) = (0usize, 0usize, 0usize);
    for n in 0..600 {
        let len = 1 + (rng.next() % 12) as usize;
        let case = Case::seeded(generate(&mut rng, len));
        match compare_cached(&case, 32) {
            Ok(Verdict::Agreed { .. }) => agreed += 1,
            Ok(Verdict::Trapped { .. }) => trapped += 1,
            Ok(Verdict::Nothing) => nothing += 1,
            Err(e) => panic!("cached case {n} diverged:\n{e}"),
        }
    }
    assert!(
        agreed > 300,
        "only {agreed} of 600 cached cases ran to completion ({trapped} trapped, {nothing} lifted \
         nothing)"
    );
    assert!(trapped > 0, "no cached case reached a fault");
}

/// The paged corpus through the translation runtime rather than one block at a
/// time.
///
/// What this covers that [`the_same_corpus_agrees_with_paging_on`] cannot: the
/// entry translation happens on **every** execution, including a block served
/// from the cache and a block reached by following a patched exit, so a block
/// that skipped its walk the second time round is a tick divergence here and
/// nowhere else. The block is looked up under the key the entry translation
/// just produced, which is the only place that key is exercised as a key.
#[test]
fn the_paged_corpus_agrees_through_the_cached_and_chained_runtime() {
    let mut rng = Lcg(0x9a6e_beef);
    let (mut agreed, mut trapped, mut nothing) = (0usize, 0usize, 0usize);
    for n in 0..400 {
        let len = 1 + (rng.next() % 12) as usize;
        let case = Case::seeded(generate(&mut rng, len)).paged();
        match compare_cached(&case, 32) {
            Ok(Verdict::Agreed { .. }) => agreed += 1,
            Ok(Verdict::Trapped { .. }) => trapped += 1,
            Ok(Verdict::Nothing) => nothing += 1,
            Err(e) => panic!("cached paged case {n} diverged:\n{e}"),
        }
    }
    assert!(
        agreed > 200,
        "only {agreed} of 400 cached paged cases ran to completion ({trapped} trapped, {nothing} \
         lifted nothing)"
    );
    assert!(trapped > 0, "no cached paged case reached a fault");
}

/// A store into a running block's own page, with paging on — where the store's
/// **linear** page and the block's **physical** page are different numbers.
///
/// This is the case the in-block guard could not have handled and the reason
/// `lift` refuses it under paging: the guard compares linear pages, and both
/// ends of the real mechanism are physical. `Smc::EndBlock` puts the boundary
/// where the store is, the host notes the physical page its store reached, and
/// the cache invalidates the translation whose bytes came from that page.
///
/// The assertion that matters is the last one: without invalidation the block
/// would run the instruction it was lifted from and the interpreter would run
/// the one the store wrote, and the two engines would disagree about `ESI` and
/// `EDI` — which is what this harness compares.
#[test]
fn a_paged_store_into_a_running_blocks_own_page_is_honoured() {
    for shape in [Shape::BasicBlock, Shape::Extended, Shape::Trace] {
        let case = Case::new(self_modifying())
            .with_reg(0, 4)
            .with_reg(3, 0x47)
            .with_shape(shape)
            .paged();
        let run = measure_cached(&case, 32).unwrap_or_else(|e| panic!("{shape:?} paged: {e}"));
        assert!(
            matches!(run.verdict, Verdict::Agreed { .. }),
            "{shape:?} paged: {:?}",
            run.verdict
        );
        assert!(
            run.smc > 0,
            "{shape:?} paged: nothing was invalidated, so the store was never matched against the \
             block's physical page"
        );
    }
}

/// The same corpus again, executed as **host code**.
///
/// `ROADMAP.md` §9's x86-64 backend, held to the standard CLAUDE.md sets for
/// everything below it: the interpreter is the oracle, forever. Every column
/// `compare_cached` compares is compared here — the eight general registers,
/// `EIP`, the flags word, the cycle counter, guest memory, and the
/// architectural state at a fault — with the only difference being which engine
/// executed the block.
///
/// x86 is the frontend that exercises the lowerings RISC-V never reaches:
/// `popcount` on the parity flag, `extract` on the auxiliary carry, both
/// widening multiplies, the rotates through carry, `bswap`, `clz` and `ctz`.
#[cfg(all(feature = "jit-x86", target_os = "linux", target_arch = "x86_64"))]
#[test]
fn the_same_corpus_agrees_when_it_is_compiled_to_host_code() {
    use rsemu::cpu::x86::differential::measure_compiled;

    let mut rng = Lcg(0x5eed_beef);
    let (mut agreed, mut trapped, mut nothing) = (0usize, 0usize, 0usize);
    let (mut compiled, mut blocks) = (0u64, 0u64);
    for n in 0..600 {
        let len = 1 + (rng.next() % 12) as usize;
        let case = Case::seeded(generate(&mut rng, len));
        match measure_compiled(&case, 32) {
            Ok(run) => {
                compiled += run.compiled;
                blocks += run.blocks as u64;
                match run.verdict {
                    Verdict::Agreed { .. } => agreed += 1,
                    Verdict::Trapped { .. } => trapped += 1,
                    Verdict::Nothing => nothing += 1,
                }
            }
            Err(e) => panic!("compiled case {n} diverged:\n{e}"),
        }
    }
    assert!(
        agreed > 300,
        "only {agreed} of 600 compiled cases ran to completion ({trapped} trapped, {nothing} \
         lifted nothing)"
    );
    assert!(trapped > 0, "no compiled case reached a fault");
    // The assertion without which all of the above would pass on a backend
    // that had quietly stopped taking a single block. It is a *fraction*
    // rather than a count, because the interesting number is coverage: a
    // refused block is correct and slow, and the two engines are mixed inside
    // one run.
    assert!(
        compiled * 2 > blocks,
        "only {compiled} of {blocks} blocks were executed as host code"
    );
}

/// A store that reaches a running block's own page **through the other linear
/// mapping of it**.
///
/// The case the in-block store guard could never have handled, made real: a
/// long-mode machine maps the four guest pages twice — once at `BASE`, where
/// the code runs, and once at `HIGH`, where the corpus points its data — so a
/// store through the high window into the low window's code page names one
/// physical frame under two linear addresses two and a half tebibytes apart.
/// A guard comparing linear pages would see the store miss the block's page
/// and carry on executing bytes that no longer exist.
///
/// What makes it work is that both ends of the mechanism are physical:
/// `Lifted::page` is the page the entry translation resolved to, and the host
/// notes the physical address its store reached. The assertion that matters is
/// the last one — without invalidation the block would run the instruction it
/// was lifted from and the interpreter would run the one the store wrote.
#[test]
fn a_store_through_the_other_mapping_of_the_code_page_is_honoured() {
    use rsemu::cpu::x86::differential::HIGH;

    for shape in [Shape::BasicBlock, Shape::Extended, Shape::Trace] {
        let program = vec![
            0x88, 0x18, // mov [rax], bl
            0x90, 0x90, // nop, nop
            0x48, 0xff, 0xc6, // inc rsi — the byte at 6 is overwritten
            0xf4,
        ];
        let case = Case::new(program)
            // The *high* alias of the code page, not the address the block
            // was entered at.
            .with_reg(0, HIGH + 6)
            .with_reg(3, 0xc7) // `48 ff c7` is `inc rdi`
            .with_shape(shape)
            .long();
        let run = measure_cached(&case, 32).unwrap_or_else(|e| panic!("{shape:?} alias: {e}"));
        assert!(
            matches!(run.verdict, Verdict::Agreed { .. }),
            "{shape:?} alias: {:?}",
            run.verdict
        );
        assert!(
            run.smc > 0,
            "{shape:?} alias: the store through the other mapping never invalidated a \
             translation, so the case is not testing what it says it is"
        );
    }
}

/// The **long-mode** corpus, executed as host code.
///
/// The only harness in the tree that reaches
/// [`Opcode::MULU2`](rsemu::ir::Opcode::MULU2) and
/// [`Opcode::MULS2`](rsemu::ir::Opcode::MULS2) from a real guest: a 64-bit
/// `MUL` needs a double-width product, no 32-bit encoding produces one, and
/// `jit::x86`'s `widening_multiply` had never been driven by anything but its
/// own unit tests. It also puts a 64-bit operand through every lowering the
/// 32-bit corpus only ever exercised at thirty-two — the popcount behind `PF`,
/// the extract behind `AF`, both rotates through carry, `bswap`, `clz`, `ctz`
/// and the shifts, at the width where an off-by-one in a mask is invisible in
/// the narrower case.
#[cfg(all(feature = "jit-x86", target_os = "linux", target_arch = "x86_64"))]
#[test]
fn the_long_mode_corpus_agrees_when_it_is_compiled_to_host_code() {
    use rsemu::cpu::x86::differential::measure_compiled;

    let mut rng = Lcg(0x6400_beef);
    let (mut agreed, mut trapped, mut nothing) = (0usize, 0usize, 0usize);
    let (mut compiled, mut blocks) = (0u64, 0u64);
    for n in 0..400 {
        let len = 1 + (rng.next() % 12) as usize;
        let case = Case::seeded(generate_in(&mut rng, len, World::Long)).long();
        match measure_compiled(&case, 32) {
            Ok(run) => {
                compiled += run.compiled;
                blocks += run.blocks as u64;
                match run.verdict {
                    Verdict::Agreed { .. } => agreed += 1,
                    Verdict::Trapped { .. } => trapped += 1,
                    Verdict::Nothing => nothing += 1,
                }
            }
            Err(e) => panic!("compiled long case {n} diverged:\n{e}"),
        }
    }
    assert!(
        agreed > 200,
        "only {agreed} of 400 compiled long cases ran to completion ({trapped} trapped, \
         {nothing} lifted nothing)"
    );
    assert!(trapped > 0, "no compiled long case reached a fault");
    assert!(
        compiled * 2 > blocks,
        "only {compiled} of {blocks} long-mode blocks were executed as host code"
    );
}

/// A tight loop: `dec ecx` / `jnz` back to it, entered with a small count.
///
/// Four bytes, wholly inside one page, and the shape that shows what merging
/// buys — a trace unrolls the back edge until the instruction limit, where a
/// basic block dispatches once per iteration.
fn counted_loop(count: u32) -> Case {
    let program = vec![
        // 0: dec ecx
        0x49, // 1: jnz -3
        0x75, 0xfd, // 3: hlt
        0xf4,
    ];
    Case::new(program).with_reg(1, u64::from(count))
}

#[test]
fn a_backward_branch_unrolls_under_a_trace_and_does_not_under_a_basic_block() {
    let plain = measure_cached(&counted_loop(40).with_shape(Shape::BasicBlock), 200)
        .expect("the basic-block shape agrees");
    let trace = measure_cached(&counted_loop(40).with_shape(Shape::Trace), 200)
        .expect("the trace shape agrees");
    assert!(matches!(plain.verdict, Verdict::Agreed { .. }));
    assert!(matches!(trace.verdict, Verdict::Agreed { .. }));
    // The same guest work, in far fewer blocks: that is the whole of
    // `ROADMAP.md` §9's fourth mechanism, measured rather than asserted.
    assert!(
        trace.blocks * 4 < plain.blocks,
        "a trace took {} blocks where a basic block took {}",
        trace.blocks,
        plain.blocks
    );
    // The basic-block run goes round the loop forty times through the same two
    // translations, so nearly every edge is a patched exit rather than a
    // lookup. The trace does not chain, and that is the point: it has swallowed
    // the edges that would have been chained.
    assert!(
        plain.chained > 30,
        "only {} of {} blocks were reached by a patched exit",
        plain.chained,
        plain.blocks
    );
    assert!(trace.translated <= plain.translated);
}

#[test]
fn a_generated_corpus_agrees_on_a_486() {
    // A different `Op::clocks` table is not what changes here — the tick model
    // is the same — but the variant is in the cache key and in `World`, so it
    // is a second frontend to run rather than a second run of the first.
    let mut rng = Lcg(0x486_0001);
    for n in 0..400 {
        let len = 1 + (rng.next() % 10) as usize;
        let mut case = Case::seeded(generate(&mut rng, len));
        case.variant = rsemu::cpu::x86::Variant::I80486;
        if let Err(e) = compare(&case) {
            panic!("486 case {n} diverged:\n{e}");
        }
    }
}

/// A program that writes a byte into its own page and then executes what it
/// wrote.
///
/// `mov [eax], bl` with `EAX` pointing at the instruction two ahead of it, then
/// that instruction. Under the interpreter the store is visible immediately,
/// because the interpreter re-fetches; under a translated block it is visible
/// only if the block gives the dispatcher a boundary to invalidate at, which is
/// exactly what the two [`Smc`] policies do in their two different ways.
fn self_modifying() -> Vec<u8> {
    vec![
        // 0: mov [eax], bl        — EAX is set to 4 by the caller
        0x88, 0x18, // 2: nop
        0x90, // 3: nop
        0x90, // 4: inc esi        — overwritten by the store
        0x46, // 5: hlt
        0xf4,
    ]
}

#[test]
fn a_store_into_a_running_block_is_honoured_under_both_policies() {
    for smc in [Smc::EndBlock, Smc::Guard] {
        for shape in [Shape::BasicBlock, Shape::Extended, Shape::Trace] {
            // EAX points at offset 4, BL holds `0x47` — `inc edi` — so the
            // instruction the block was lifted from changes under it.
            let case = Case::new(self_modifying())
                .with_reg(0, 4)
                .with_reg(3, 0x47)
                .with_shape(shape)
                .with_smc(smc);
            let run =
                measure_cached(&case, 32).unwrap_or_else(|e| panic!("{smc:?}/{shape:?}: {e}"));
            assert!(
                matches!(run.verdict, Verdict::Agreed { .. }),
                "{smc:?}/{shape:?}: {:?}",
                run.verdict
            );
            assert!(
                run.smc > 0,
                "{smc:?}/{shape:?}: the store never invalidated a translation, so the case is \
                 not testing what it says it is"
            );
        }
    }
}

#[test]
fn a_store_that_misses_the_code_page_invalidates_nothing() {
    // The other half of the guard: the overwhelmingly common case is a store
    // that has nothing to do with code, and it must cost one not-taken branch
    // rather than a block boundary.
    let program = vec![
        // mov [eax], bl ; inc esi ; hlt — EAX points into the data window.
        0x88, 0x18, 0x46, 0xf4,
    ];
    let case = Case::seeded(program).with_smc(Smc::Guard);
    let run = measure_cached(&case, 32).expect("agrees");
    assert!(matches!(run.verdict, Verdict::Agreed { .. }));
    assert_eq!(run.smc, 0, "a data store must not invalidate a translation");
    assert_eq!(
        run.insns_retired, 2,
        "the guard must not cut the block short: {run:?}"
    );
}

#[test]
fn an_access_past_the_segment_limit_faults_in_the_same_state_in_both_engines() {
    // The fault column, driven deliberately rather than waited for. `EAX` is
    // one byte inside the limit, so a doubleword read straddles it — and the
    // architectural state at the fault is everything the two adds before it
    // produced, flags included.
    let program = vec![
        // 0: add ecx, edx
        0x01, 0xd1, // 2: sub ebx, 1
        0x83, 0xeb, 0x01, // 5: mov esi, [eax]
        0x8b, 0x30, // 7: inc edi   — never reached
        0x47, 0xf4,
    ];
    for shape in [Shape::BasicBlock, Shape::Extended, Shape::Trace] {
        for flags in [Flags::Eager, Flags::Elide] {
            let case = Case::seeded(program.clone())
                .with_reg(0, rsemu::cpu::x86::differential::RAM_SIZE - 1)
                .with_shape(shape)
                .with_flags(flags);
            let verdict = compare(&case).unwrap_or_else(|e| panic!("{shape:?}/{flags:?}: {e}"));
            assert!(
                matches!(verdict, Verdict::Trapped { insns: 2 }),
                "{shape:?}/{flags:?}: {verdict:?}"
            );
        }
    }
}

#[test]
fn a_program_of_pure_flag_arithmetic_agrees_bit_for_bit() {
    // Every flag-producing family in one program, with a `LAHF` at the end so
    // the packed low byte is observed rather than only compared.
    let program = vec![
        0x83, 0xc0, 0x7f, // add eax, 127
        0x11, 0xd8, // adc eax, ebx
        0x19, 0xc8, // sbb eax, ecx
        0x21, 0xd0, // and eax, edx
        0xf7, 0xd8, // neg eax
        0xd1, 0xe0, // shl eax, 1
        0xc1, 0xf8, 0x03, // sar eax, 3
        0xd1, 0xd0, // rcl eax, 1
        0xc1, 0xc8, 0x07, // ror eax, 7
        0xf7, 0xe3, // mul ebx
        0x0f, 0xaf, 0xc1, // imul eax, ecx
        0x9f, // lahf
        0xf4,
    ];
    for start in [0u32, 0x8000_0000, 0xffff_ffff, 1, 0x7fff_ffff] {
        for shape in [Shape::BasicBlock, Shape::Extended, Shape::Trace] {
            for flags in [Flags::Eager, Flags::Elide] {
                let case = Case::seeded(program.clone())
                    .with_reg(0, u64::from(start))
                    .with_reg(3, u64::from(start.rotate_left(13)))
                    .with_shape(shape)
                    .with_flags(flags);
                compare(&case).unwrap_or_else(|e| panic!("{start:#x}/{shape:?}/{flags:?}: {e}"));
            }
        }
    }
}

#[test]
fn a_call_and_a_return_agree_through_the_stack() {
    let program = vec![
        0xe8, 0x03, 0x00, 0x00, 0x00, // call +3  -> offset 8
        0x46, // inc esi
        0xeb, 0x03, // jmp +3 -> offset 11
        0x47, // inc edi
        0xc3, // ret
        0x90, // nop (padding so the jump lands here)
        0xf4, // hlt
    ];
    for shape in [Shape::BasicBlock, Shape::Extended, Shape::Trace] {
        for smc in [Smc::EndBlock, Smc::Guard] {
            let case = Case::seeded(program.clone())
                .with_shape(shape)
                .with_smc(smc);
            let verdict =
                compare_cached(&case, 16).unwrap_or_else(|e| panic!("{shape:?}/{smc:?}: {e}"));
            assert!(
                matches!(verdict, Verdict::Agreed { .. }),
                "{shape:?}/{smc:?}: {verdict:?}"
            );
        }
    }
}

#[test]
fn a_shift_by_cl_agrees_at_every_count_including_zero() {
    // The one instruction in the subset whose *whole effect* is conditional: a
    // count of zero writes no flag and no operand, which the lifter expresses
    // as a select rather than a branch.
    for op in [0xe0u8, 0xe8, 0xf8, 0xc0, 0xc8, 0xd0, 0xd8] {
        for count in [0u32, 1, 7, 31, 32, 33, 255] {
            let program = vec![0xd3, op, 0xf4];
            let case = Case::seeded(program)
                .with_reg(0, 0x8123_4567)
                .with_reg(1, u64::from(count))
                .with_eflags(0x0002 | 0x0001);
            compare(&case).unwrap_or_else(|e| panic!("d3 /{op:#x} by {count}: {e}"));
        }
    }
}

/// A `CALL` whose pushed return address lands on the instruction it is about
/// to jump to.
///
/// The one instruction in the subset whose store is **not** its last effect:
/// `CALL` pushes and then transfers, so a self-modifying-code exit taken at the
/// push has to resume at the call's *target* and not after the call. That is
/// the whole reason the lifter carries a `Resume` rather than assuming the next
/// instruction, and nothing in the generated corpus reaches it — the corpus
/// keeps its stack in the data window, where a push can never be code.
fn call_that_overwrites_its_own_target() -> Vec<u8> {
    let mut program = vec![0xe8, 0x17, 0x00, 0x00, 0x00]; // call +0x17 -> 0x1c
    while program.len() < 0x1c {
        program.push(0x90); // nop
    }
    program.push(0x46); // 0x1c: inc esi — the byte the push overwrites
    while program.len() < 0x21 {
        program.push(0xf4); // hlt, so the untouched path stops here
    }
    program.push(0xf4); // 0x21: hlt, where the *rewritten* instruction ends
    program
}

#[test]
fn a_call_that_rewrites_its_own_target_resumes_at_the_target() {
    // ESP is four bytes past the target, so the pushed return address lands
    // exactly on it. Under `Smc::Guard` the trace has already merged across the
    // call, so the store's page guard is the only thing between the push and
    // executing bytes that no longer exist.
    let case = Case::new(call_that_overwrites_its_own_target())
        .with_reg(4, 0x20)
        .with_shape(Shape::Trace)
        .with_smc(Smc::Guard);
    let mut case = case;
    case.keep_esp = true;
    let run = measure_cached(&case, 32).expect("the two engines agree");
    assert!(
        matches!(run.verdict, Verdict::Agreed { .. }),
        "{:?}",
        run.verdict
    );
    assert!(
        run.smc > 0,
        "the push never invalidated a translation, so the case is not testing what it says"
    );
}

#[test]
fn a_compare_against_memory_still_makes_its_bus_cycle() {
    // The shape `MemOp::volatile` exists for. `cmp eax, [ebx]` loads a
    // doubleword whose *value* nothing keeps: it feeds the six flags and
    // nothing else. The `xor` after it overwrites all six and cannot fault, so
    // under `Flags::Elide` the boundary drops them and dead-code elimination is
    // free to take the whole chain — the popcount, the comparisons, and the
    // load with them. Only the load's `volatile` stops it, and without that the
    // block would be two ticks short and one bus cycle quieter than the
    // interpreter.
    let program = vec![
        0x3b, 0x03, // cmp eax, [ebx]
        0x31, 0xc9, // xor ecx, ecx
        0xf4,
    ];
    for flags in [Flags::Eager, Flags::Elide] {
        let case = Case::seeded(program.clone()).with_flags(flags);
        let verdict = compare(&case).unwrap_or_else(|e| panic!("{flags:?}: {e}"));
        assert!(
            matches!(verdict, Verdict::Agreed { insns: 2, .. }),
            "{flags:?}: {verdict:?}"
        );
    }
}

#[test]
fn a_pop_into_a_stack_relative_address_uses_the_stack_pointer_it_ends_with() {
    // The one instruction in the subset that rebinds a register and *then*
    // reaches memory, and the one place the architecture asks for the address
    // to be computed late. *Intel SDM* volume 2, `POP`: "If the ESP register is
    // used as a base register for addressing a destination operand in memory,
    // the POP instruction computes the effective address of the operand after
    // it increments the ESP register." So `pop [esp+4]` stores four bytes above
    // where the address taken at the start of the instruction points, and both
    // engines have to agree about which.
    //
    // Written by hand rather than generated, because the corpus keeps `ESP`
    // out of its operands on purpose: a random stack pointer makes every push
    // a fault.
    // The value has to be one the wrong address would not already hold, or the
    // two answers write the same zero four bytes apart and nothing notices —
    // which is exactly what the first draft of this test did.
    let program = vec![
        0xc7, 0x04, 0x24, 0x44, 0x33, 0x22, 0x11, // mov dword [esp], 0x11223344
        0x8f, 0x44, 0x24, 0x04, // pop dword [esp+4]
        0x40, // inc eax
        0xf4,
    ];
    for shape in [Shape::BasicBlock, Shape::Extended, Shape::Trace] {
        for smc in [Smc::EndBlock, Smc::Guard] {
            let case = Case::seeded(program.clone())
                .with_shape(shape)
                .with_smc(smc);
            let verdict = compare(&case).unwrap_or_else(|e| panic!("{shape:?}/{smc:?}: {e}"));
            assert!(
                matches!(verdict, Verdict::Agreed { .. } | Verdict::Trapped { .. }),
                "{shape:?}/{smc:?}: {verdict:?}"
            );
        }
    }
    // and the same through the cached path, where the store also has to be
    // matched against the block cache at the right address.
    let case = Case::seeded(program);
    compare_cached(&case, 8).expect("the cached path agrees");
}

#[test]
fn a_push_of_a_stack_relative_operand_reads_before_the_pointer_moves() {
    // The mirror image, and the one the interpreter gets right by ordering
    // rather than by caching: `push [esp]` reads its operand and *then* moves
    // the pointer.
    let program = vec![
        0xff, 0x34, 0x24, // push dword [esp]
        0x40, // inc eax
        0xf4,
    ];
    let case = Case::seeded(program);
    let verdict = compare(&case).expect("agrees");
    assert!(matches!(verdict, Verdict::Agreed { .. }), "{verdict:?}");
}
