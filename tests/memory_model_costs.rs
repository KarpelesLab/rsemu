//! The numbers behind the memory-model boundaries `core::space::store` and
//! `core::sync` record: what single-copy atomicity would cost, what the store
//! path it would be charged to costs, how long the host keeps a store
//! invisible, and what the fence that covers that window costs.
//!
//! All of them are `#[ignore]`d — they are measurements, not gates, and a gate
//! that depends on a host's store buffer is a flake. They live here rather than
//! in a scratch file because they are quoted in module documentation as
//! decisions, and a quoted number nobody can re-derive is a number that rots.
//! `tests/memory_model_litmus.rs` is the gate these numbers explain.
//!
//! Run with `cargo test --release --test memory_model_costs -- --ignored
//! --nocapture --test-threads=1`. Release matters: the first measurement is a
//! handful of instructions and a debug build measures the bounds checks. One
//! test thread matters too, because the second measurement is about what two
//! threads see of each other and a third one competing for the same cores
//! moves the answer.
//!
//! # Why there is `unsafe` in this file and nowhere near it in the crate
//!
//! `space::store`'s cost table has **three** rows and this file used to build
//! two of them. The missing one — a wide aligned atomic through a cast pointer —
//! is the shape whose price decides whether the defect is worth a design review
//! at all, and it was the one number in that table nobody could re-derive.
//!
//! So it is built here, in a test binary, behind a scoped
//! `#[allow(unsafe_code)]` on the two model methods that need it. That is
//! **not** an eighth sanctioned subsystem and not a step towards one:
//! `CLAUDE.md`'s ceiling is about the crate, `src/` is untouched, and nothing
//! here is linked into anything a board runs. It is a bench and two litmus
//! programs, and its whole purpose is to let whoever holds the design review
//! read the number and the machine code instead of taking a previous agent's
//! word for both. Deleting this section costs exactly one row of a table and no
//! shipped behaviour.

#![cfg(feature = "std")]

use std::hint::black_box;
use std::sync::atomic::{AtomicU8, AtomicU16, AtomicU32, AtomicU64, AtomicUsize, Ordering};
use std::sync::{Arc, Barrier};
use std::time::Instant;

// ---------------------------------------------------------------------------
// What single-copy atomicity would cost the RAM store
// ---------------------------------------------------------------------------

/// Bytes in the model stores. Small enough to stay in L1, so this measures the
/// access shape rather than the memory system.
const LEN: usize = 1 << 16;
/// Accesses per timed run.
const REPS: usize = 4_000_000;

/// What `RamStore` does today: a byte loop over `Vec<AtomicU8>`.
///
/// Per-byte atomicity and no `unsafe` at all — and, because the bytes are
/// independent, no single-copy atomicity for anything wider than one of them.
struct Bytes(Vec<AtomicU8>);

impl Bytes {
    fn new() -> Bytes {
        Bytes((0..LEN).map(|_| AtomicU8::new(0)).collect())
    }

    #[inline]
    fn write(&self, off: usize, src: &[u8]) {
        for (i, b) in src.iter().enumerate() {
            self.0[off + i].store(*b, Ordering::Relaxed);
        }
    }

    #[inline]
    fn read(&self, off: usize, dst: &mut [u8]) {
        for (i, b) in dst.iter_mut().enumerate() {
            *b = self.0[off + i].load(Ordering::Relaxed);
        }
    }
}

/// The alternative that keeps every rule the crate has: `Vec<AtomicU64>`, so a
/// whole word is one atomic access, with sub-word writes done by
/// compare-exchange because there is no other way to change part of a word
/// without letting a wide reader see the halves separately.
///
/// This is the shape whose cost decided the question, and the compare-exchange
/// is where the cost is.
struct Words(Vec<AtomicU64>);

impl Words {
    fn new() -> Words {
        Words((0..LEN / 8).map(|_| AtomicU64::new(0)).collect())
    }

    #[inline]
    fn write(&self, off: usize, src: &[u8]) {
        let n = src.len();
        let (word, lo) = (off / 8, off % 8);
        assert!(lo + n <= 8, "the model only does within-word accesses");
        let mut value = 0u64;
        for (i, b) in src.iter().enumerate() {
            value |= u64::from(*b) << ((lo + i) * 8);
        }
        if n == 8 {
            self.0[word].store(value, Ordering::Relaxed);
            return;
        }
        let mask = ((1u64 << (n * 8)) - 1) << (lo * 8);
        let cell = &self.0[word];
        let mut cur = cell.load(Ordering::Relaxed);
        loop {
            match cell.compare_exchange_weak(
                cur,
                (cur & !mask) | value,
                Ordering::Relaxed,
                Ordering::Relaxed,
            ) {
                Ok(_) => return,
                Err(now) => cur = now,
            }
        }
    }

    #[inline]
    fn read(&self, off: usize, dst: &mut [u8]) {
        let (word, lo) = (off / 8, off % 8);
        assert!(lo + dst.len() <= 8);
        let value = self.0[word].load(Ordering::Relaxed);
        for (i, b) in dst.iter_mut().enumerate() {
            *b = (value >> ((lo + i) * 8)) as u8;
        }
    }
}

/// The third shape: `RamStore`'s own `Vec<AtomicU8>`, but a naturally aligned
/// access of a power-of-two width becomes **one** atomic access of that width
/// through a cast pointer.
///
/// This is the shape that is cheap and that needs `unsafe`, and it models
/// `RamStore` faithfully in the one respect that decides whether it is sound at
/// all: the allocation carries `HOST_PAGE - 1` bytes of slack and `base` is
/// chosen so that guest byte zero is 4 KiB aligned. A `Vec<AtomicU8>` from the
/// global allocator has layout alignment **1**, so without that slack a
/// "naturally aligned" guest offset would say nothing whatever about the host
/// address. The alignment the store already carries for KVM's memory slots is
/// exactly the alignment a wide access needs; that is a coincidence worth
/// knowing about before anyone reasons from the guest offset alone.
struct Wide {
    cells: Vec<AtomicU8>,
    base: usize,
}

impl Wide {
    fn new() -> Wide {
        let page = 4096usize;
        let mut cells = Vec::new();
        cells.resize_with(LEN + page - 1, || AtomicU8::new(0));
        let base = (cells.as_ptr() as usize).wrapping_neg() % page;
        Wide { cells, base }
    }

    /// Whether `off` is a natural boundary for an `n`-byte access, and `n` is a
    /// width a host has an atomic for.
    #[inline]
    fn wide(off: usize, n: usize) -> bool {
        matches!(n, 1 | 2 | 4 | 8) && off.is_multiple_of(n)
    }

    #[inline]
    #[allow(unsafe_code)]
    fn write(&self, off: usize, src: &[u8]) {
        let n = src.len();
        if !Wide::wide(off, n) {
            for (i, b) in src.iter().enumerate() {
                self.cells[self.base + off + i].store(*b, Ordering::Relaxed);
            }
            return;
        }
        // Whole-allocation provenance, not one cell's: `Vec::as_ptr` covers
        // every byte of the allocation, where `&self.cells[i]` would cover one
        // and reading eight through it is what makes a cast of an *indexed*
        // reference wrong rather than merely unusual.
        let at = self.cells.as_ptr();
        // SAFETY (model, not crate code): `base + off + n <= cells.len()` by
        // the caller's construction, `base` makes `at.add(base)` 4 KiB aligned
        // and `off % n == 0` therefore makes `at.add(base + off)` `n`-aligned,
        // and no Rust reference to the bytes is formed. What this does *not*
        // discharge is the mixed-size question: eight `AtomicU8` objects and
        // one `AtomicU64` overlapping them are not the same object in the
        // Rust/C++ model, and that is the point at issue, not a detail this
        // comment can wave away.
        unsafe {
            let p = at.add(self.base + off);
            match n {
                1 => (*p).store(src[0], Ordering::Relaxed),
                2 => (*p.cast::<AtomicU16>()).store(
                    u16::from_ne_bytes(src.try_into().expect("two bytes")),
                    Ordering::Relaxed,
                ),
                4 => (*p.cast::<AtomicU32>()).store(
                    u32::from_ne_bytes(src.try_into().expect("four bytes")),
                    Ordering::Relaxed,
                ),
                _ => (*p.cast::<AtomicU64>()).store(
                    u64::from_ne_bytes(src.try_into().expect("eight bytes")),
                    Ordering::Relaxed,
                ),
            }
        }
    }

    #[inline]
    #[allow(unsafe_code)]
    fn read(&self, off: usize, dst: &mut [u8]) {
        let n = dst.len();
        if !Wide::wide(off, n) {
            for (i, b) in dst.iter_mut().enumerate() {
                *b = self.cells[self.base + off + i].load(Ordering::Relaxed);
            }
            return;
        }
        let at = self.cells.as_ptr();
        // SAFETY: as `Wide::write`, and with the same unresolved half.
        unsafe {
            let p = at.add(self.base + off);
            match n {
                1 => dst[0] = (*p).load(Ordering::Relaxed),
                2 => dst.copy_from_slice(
                    &(*p.cast::<AtomicU16>())
                        .load(Ordering::Relaxed)
                        .to_ne_bytes(),
                ),
                4 => dst.copy_from_slice(
                    &(*p.cast::<AtomicU32>())
                        .load(Ordering::Relaxed)
                        .to_ne_bytes(),
                ),
                _ => dst.copy_from_slice(
                    &(*p.cast::<AtomicU64>())
                        .load(Ordering::Relaxed)
                        .to_ne_bytes(),
                ),
            }
        }
    }
}

/// Best of five, after three warm-up runs. Best rather than mean because the
/// question is what the access costs, not what the machine's other tenants do.
fn timed(name: &str, f: impl Fn()) {
    for _ in 0..3 {
        f();
    }
    let mut best = f64::MAX;
    for _ in 0..5 {
        let start = Instant::now();
        f();
        best = best.min(start.elapsed().as_secs_f64() * 1e9 / REPS as f64);
    }
    println!("{name}: {best:.2} ns/op");
}

/// `space::store`'s "What removing it would cost, measured" — all three rows.
///
/// Recorded there, on the author's host: today 0.74/0.89/1.25/1.99 ns to store
/// 1/2/4/8 bytes, against 3.86/4.05/4.24/1.55 for the word-and-CAS shape and
/// 1.34/1.34/1.40/1.40 for the wide-aligned-atomic one. Read the third row
/// against the first per width rather than as an average: the wide shape is
/// *dearer* than today at one and two bytes and **cheaper** at four and eight,
/// which is not what "roughly cost-neutral" suggests and matters because a
/// 64-bit guest's stores are mostly four and eight.
#[test]
#[ignore = "a measurement, not a gate"]
fn what_single_copy_atomicity_would_cost() {
    let bytes = Bytes::new();
    let words = Words::new();
    let wide = Wide::new();
    for width in [1usize, 2, 4, 8] {
        // The offset walks a cache line at a time and the width is opaque to
        // the optimiser, because in the real store both are dynamic.
        let offsets = |i: usize| ((i * 64) % (LEN - 8)) & !7;
        timed(&format!("AtomicU8  store {width}B"), || {
            let src = vec![0xa5u8; width];
            for i in 0..REPS {
                bytes.write(offsets(i), black_box(&src));
            }
        });
        timed(&format!("AtomicU64 store {width}B"), || {
            let src = vec![0xa5u8; width];
            for i in 0..REPS {
                words.write(offsets(i), black_box(&src));
            }
        });
        timed(&format!("wide      store {width}B"), || {
            let src = vec![0xa5u8; width];
            for i in 0..REPS {
                wide.write(offsets(i), black_box(&src));
            }
        });
        timed(&format!("AtomicU8  load  {width}B"), || {
            let mut dst = vec![0u8; width];
            for i in 0..REPS {
                bytes.read(offsets(i), black_box(&mut dst));
            }
            black_box(&dst);
        });
        timed(&format!("AtomicU64 load  {width}B"), || {
            let mut dst = vec![0u8; width];
            for i in 0..REPS {
                words.read(offsets(i), black_box(&mut dst));
            }
            black_box(&dst);
        });
        timed(&format!("wide      load  {width}B"), || {
            let mut dst = vec![0u8; width];
            for i in 0..REPS {
                wide.read(offsets(i), black_box(&mut dst));
            }
            black_box(&dst);
        });
    }
}

/// The **denominator**, which was quoted and never measured: what one whole
/// store through `SpaceView::write_span` costs.
///
/// "+3 ns is +12% of the hottest path in the emulator" is a claim about a ratio,
/// and only its numerator was re-derivable. This is the other half — a
/// `Region::ram` mapped at zero, written one aligned word at a time through the
/// same public entry point every core, DMA engine and ROM loader uses, so it
/// carries the topology try-lock, the flat-view lookup, the permission check,
/// the width constraint check, `ExclusiveMonitor::note_store` and the dirty
/// bitmap as well as the bytes.
///
/// Recorded on the author's host: ~20 ns for four bytes, down from ~25 ns
/// before `mark_dirty` stopped paying for a locked instruction on every store
/// and `SpaceView::write` started carrying the value into the leaf instead of
/// a stack buffer. Compare a candidate's *delta* against this, not against the
/// bare access — the byte loop is a few per cent of what a guest store actually
/// costs, which is the number that decides whether the price is affordable.
#[test]
#[ignore = "a measurement, not a gate"]
fn what_a_whole_store_through_the_space_costs() {
    use rsemu::core::space::{AddressSpace, MemAttrs, RamStore, Region};
    use rsemu::core::value::Width;

    let space = AddressSpace::new("bench", 32);
    space
        .topology()
        .map(Region::ram("ram", Arc::new(RamStore::new(LEN as u64))), 0)
        .expect("the map fits");
    for (width, n) in [
        (Width::U8, 1u64),
        (Width::U16, 2),
        (Width::U32, 4),
        (Width::U64, 8),
    ] {
        timed(&format!("AddressSpace::write {n}B"), || {
            for i in 0..REPS {
                let addr = ((i as u64 * 64) % (LEN as u64 - 8)) & !7;
                space
                    .write(
                        addr,
                        width,
                        black_box(0xa5a5_a5a5_a5a5_a5a5),
                        MemAttrs::DEFAULT,
                    )
                    .expect("the store lands");
            }
        });
        timed(&format!("AddressSpace::read  {n}B"), || {
            for i in 0..REPS {
                let addr = ((i as u64 * 64) % (LEN as u64 - 8)) & !7;
                black_box(
                    space
                        .read(addr, width, MemAttrs::DEFAULT)
                        .expect("the load lands"),
                );
            }
        });
    }
}

/// One 64-instruction loop of `body`, stepped through the x86 interpreter, in
/// nanoseconds per guest instruction.
///
/// `bx` holds `0x2000`, so a body of `mov [bx], …` stores to a fixed aligned
/// word without needing an address computation in the loop.
///
/// **The branch back is `E9 cw`, not `EB cb`, and that is not a detail.** A
/// sixty-four-instruction body is well past the 128 bytes an `EB` displacement
/// reaches; an earlier draft of this measurement used one, the truncated
/// displacement sent the core off into unmapped memory, and it produced a
/// perfectly stable, perfectly meaningless 24 ns an instruction. Hence the two
/// checks at the end: the loop must still be inside itself, and a store body
/// must have left its value behind. A benchmark that silently measures a
/// different program is worse than no benchmark.
#[cfg(feature = "cpu-x86")]
fn interpreted(name: &str, body: &[u8], expect: Option<u32>) {
    use rsemu::core::space::{AddressSpace, MemAttrs, RamStore, Region};
    use rsemu::core::value::Width;
    use rsemu::cpu::x86::{Config, Variant, X86};

    const AT: u64 = 0x100;
    const STEPS: u64 = 2_000_000;

    let space = Arc::new(AddressSpace::new("bench", 32));
    space
        .topology()
        .map(Region::ram("ram", Arc::new(RamStore::new(0x4_0000))), 0)
        .expect("the map fits");
    let mut code = Vec::new();
    for _ in 0..64 {
        code.extend_from_slice(body);
    }
    let back = -((code.len() + 3) as i64) as i16;
    code.push(0xe9);
    code.extend_from_slice(&back.to_le_bytes());
    let end = AT + code.len() as u64;
    space
        .write_bytes(AT, &code, MemAttrs::DEFAULT)
        .expect("the program lands");

    let cpu = Arc::new(X86::new(Config::default().with_variant(Variant::I80486)));
    cpu.attach_space(Arc::clone(&space));
    // The first step is the power-on reset and discards the register file, so
    // it is spent before the core is placed — as `smp_single_copy_atomicity`
    // explains at greater length.
    cpu.step();
    let mut regs = cpu.regs();
    regs.cs = 0;
    regs.ds = 0;
    regs.es = 0;
    regs.ss = 0;
    regs.rip = AT;
    regs.rbx = 0x2000;
    regs.rax = u64::from(expect.unwrap_or(0));
    cpu.set_regs(regs);

    for _ in 0..200_000 {
        assert!(cpu.step() > 0, "{name}: the core stopped");
    }
    let mut best = f64::MAX;
    for _ in 0..5 {
        let start = Instant::now();
        for _ in 0..STEPS {
            cpu.step();
        }
        best = best.min(start.elapsed().as_secs_f64() * 1e9 / STEPS as f64);
    }
    let rip = cpu.regs().rip;
    assert!(
        (AT..=end).contains(&rip),
        "{name}: the loop left itself and ran at {rip:#x}, so this measured \
         some other program"
    );
    if let Some(v) = expect {
        assert_eq!(
            space.read(0x2000, Width::U32, MemAttrs::DEFAULT),
            Ok(u64::from(v)),
            "{name}: the store body never stored anything"
        );
    }
    println!("{name}: {best:.1} ns/guest instruction");
}

/// The **other** denominator, and the one that decides the question: what a
/// whole guest store *instruction* costs the engine that can actually tear.
///
/// A store the JIT inlines is one host instruction of the guest's width at a
/// naturally aligned address (`jit::x86::compile`'s `probe` refuses anything
/// else), so it is **already** single-copy atomic and no change to `RamStore`
/// is for its benefit. The interpreter is the engine the defect belongs to, so
/// the interpreter is what a candidate fix has to be priced against.
///
/// Recorded on the author's host: a `nop` costs ~55 ns, `mov [bx], al` ~104,
/// `mov [bx], eax` ~120, and the nine-byte immediate form the SMP test's
/// writer uses ~203 — instruction fetch is a bus access a byte at a time, so
/// most of the spread is the encoding's length. `AddressSpace::write` itself
/// measures ~25 of those nanoseconds and the byte loop inside it ~1.3.
///
/// So "+3 ns is +12% of the hottest path in the emulator" has the right
/// numerator and the wrong denominator. `SpaceView::write_span` is not a path a
/// guest executes; it is a fifth of one. Against the instruction that contains
/// it, +3 ns is **+2 to +3%**, and that is the number the decision should be
/// made on.
#[cfg(feature = "cpu-x86")]
#[test]
#[ignore = "a measurement, not a gate"]
fn what_a_guest_store_instruction_costs_the_interpreter() {
    interpreted("nop          ", &[0x90], None);
    interpreted("mov [bx], al ", &[0x88, 0x07], None);
    interpreted("mov [bx], eax", &[0x66, 0x89, 0x07], Some(0xdead_beef));
    let mut imm = vec![0x66, 0xc7, 0x06, 0x00, 0x20];
    imm.extend_from_slice(&0xdead_beefu32.to_le_bytes());
    interpreted("mov dword [0x2000], imm32", &imm, Some(0xdead_beef));
}

// ---------------------------------------------------------------------------
// Does the wide shape actually do what it is proposed for
// ---------------------------------------------------------------------------

/// The wide shape, put under the race `tests/smp_single_copy_atomicity.rs` runs
/// against the real store: a four-byte aligned store alternating two values,
/// against a four-byte aligned load.
///
/// `Bytes` — today's shape — tears here in the hundreds. `Wide` must tear zero
/// times, and the witness count is printed so that a run in which the two
/// threads never overlapped is not mistaken for a run that proved something.
/// This is the "does it work" half of the case; the "is it sound" half is not a
/// thing any test can answer, and the next test says exactly how far a test can
/// get.
#[test]
#[ignore = "a measurement, not a gate: how often it tears is the host's business"]
fn the_wide_shape_does_not_tear_where_the_byte_loop_does() {
    const ROUNDS: usize = 5_000_000;

    fn race(store: Arc<impl Send + Sync + 'static + Access>) -> (usize, usize) {
        let torn = Arc::new(AtomicUsize::new(0));
        let saw = Arc::new(AtomicUsize::new(0));
        let gate = Arc::new(Barrier::new(2));
        let writer = Arc::clone(&store);
        let start = Arc::clone(&gate);
        std::thread::scope(|s| {
            s.spawn(move || {
                start.wait();
                for i in 0..ROUNDS {
                    let v: u32 = if i % 2 == 0 { 0 } else { u32::MAX };
                    writer.put(0x40, &v.to_ne_bytes());
                }
            });
            gate.wait();
            let mut buf = [0u8; 4];
            for _ in 0..ROUNDS {
                store.get(0x40, &mut buf);
                match u32::from_ne_bytes(buf) {
                    0 => {
                        saw.fetch_add(1, Ordering::Relaxed);
                    }
                    u32::MAX => {}
                    _ => {
                        torn.fetch_add(1, Ordering::Relaxed);
                    }
                }
            }
        });
        (torn.load(Ordering::Relaxed), saw.load(Ordering::Relaxed))
    }

    let (torn, saw) = race(Arc::new(Bytes::new()));
    println!("byte loop: {torn} torn of {ROUNDS}, {saw} saw the other value");
    let (torn, saw) = race(Arc::new(Wide::new()));
    println!("wide     : {torn} torn of {ROUNDS}, {saw} saw the other value");
    assert!(
        saw > 0,
        "the two threads never overlapped, so this proves nothing"
    );
    assert_eq!(
        torn, 0,
        "a wide aligned atomic store cannot be seen in halves"
    );
}

/// The mixed-size case itself, which is the thing the design review is actually
/// about — and the honest limit of what running it can tell anyone.
///
/// A four-byte wide atomic store to `[0x40]` races a **one-byte** atomic store
/// to `[0x44]` while an eight-byte wide atomic load reads both. Three widths,
/// two of them overlapping the third, on one address — the shape the Rust and
/// C++ memory models do not define, because `AtomicU8` and `AtomicU64` covering
/// the same bytes are different *memory locations* that happen to overlap and
/// the model's data-race rule is stated per location.
///
/// Every hardware architecture in the tree defines it (the Cambridge group's
/// mixed-size work, `docs/techniques/memory-models.md`), and LLVM lowers each
/// relaxed atomic access to exactly one machine access of exactly that width —
/// it will not split one, merge two, or forward across them. Both of those are
/// checkable, and this checks the second one's *effect*.
///
/// **A pass here is not a soundness argument.** Undefined behaviour that this
/// host, this compiler and this optimisation level decline to exercise is still
/// undefined behaviour; what a green run buys is evidence that the failure is
/// not sitting in plain sight, which is worth having and is not the same thing.
#[test]
#[ignore = "a measurement, not a gate"]
fn a_mixed_size_race_produces_no_value_that_was_never_written() {
    const ROUNDS: usize = 5_000_000;
    let store = Arc::new(Wide::new());
    let illegal = Arc::new(AtomicUsize::new(0));
    let gate = Arc::new(Barrier::new(3));

    std::thread::scope(|s| {
        let (wide, narrow) = (Arc::clone(&store), Arc::clone(&store));
        let (a, b) = (Arc::clone(&gate), Arc::clone(&gate));
        s.spawn(move || {
            a.wait();
            for i in 0..ROUNDS {
                let v: u32 = if i % 2 == 0 { 0x1111_1111 } else { 0x2222_2222 };
                wide.write(0x40, &v.to_ne_bytes());
            }
        });
        s.spawn(move || {
            b.wait();
            for i in 0..ROUNDS {
                narrow.write(0x44, &[if i % 2 == 0 { 0xaa } else { 0xbb }]);
            }
        });
        gate.wait();
        let mut buf = [0u8; 8];
        for _ in 0..ROUNDS {
            store.read(0x40, &mut buf);
            let low = u32::from_ne_bytes(buf[..4].try_into().expect("four bytes"));
            let byte = buf[4];
            let low_ok = matches!(low, 0 | 0x1111_1111 | 0x2222_2222);
            let byte_ok = matches!(byte, 0 | 0xaa | 0xbb);
            if !(low_ok && byte_ok) {
                illegal.fetch_add(1, Ordering::Relaxed);
            }
        }
    });

    let n = illegal.load(Ordering::Relaxed);
    println!("mixed size: {n} illegal of {ROUNDS} eight-byte loads");
    assert_eq!(
        n, 0,
        "a narrow store and a wide one must not corrupt each other"
    );
}

/// What the two model stores have in common, so the race above can be run
/// against either without a second copy of it.
trait Access {
    fn put(&self, off: usize, src: &[u8]);
    fn get(&self, off: usize, dst: &mut [u8]);
}

impl Access for Bytes {
    fn put(&self, off: usize, src: &[u8]) {
        self.write(off, src);
    }
    fn get(&self, off: usize, dst: &mut [u8]) {
        self.read(off, dst);
    }
}

impl Access for Wide {
    fn put(&self, off: usize, src: &[u8]) {
        self.write(off, src);
    }
    fn get(&self, off: usize, dst: &mut [u8]) {
        self.read(off, dst);
    }
}

// ---------------------------------------------------------------------------
// How long the host keeps a store invisible
// ---------------------------------------------------------------------------

/// One store-buffer litmus round over the primitive `RamStore` is built from.
///
/// Two threads, each storing to its own byte and then loading the other's, with
/// `pad` units of work in between standing in for the emulator's own cost per
/// guest instruction. Both loads returning zero is the outcome a guest's
/// `MFENCE` — or `DMB`, or `FENCE` — exists to forbid, and which every core in
/// the tree now emits a host fence to prevent.
fn store_buffer_rounds(pad: u64, rounds: usize) -> usize {
    let x = Arc::new(AtomicU8::new(0));
    let y = Arc::new(AtomicU8::new(0));
    let seen = [Arc::new(AtomicU8::new(9)), Arc::new(AtomicU8::new(9))];
    let both_zero = Arc::new(AtomicUsize::new(0));
    // Three: the two litmus threads and the referee that reads the result and
    // resets the round. Without a referee the two threads have to agree about
    // when a round ended, which needs the ordering the test is measuring.
    let gate = Arc::new(Barrier::new(3));

    std::thread::scope(|s| {
        for (who, slot) in seen.iter().enumerate() {
            let (x, y) = (Arc::clone(&x), Arc::clone(&y));
            let out = Arc::clone(slot);
            let gate = Arc::clone(&gate);
            s.spawn(move || {
                for _ in 0..rounds {
                    gate.wait();
                    let (mine, theirs) = if who == 0 { (&x, &y) } else { (&y, &x) };
                    mine.store(1, Ordering::Relaxed);
                    for i in 0..pad {
                        black_box(i);
                    }
                    out.store(theirs.load(Ordering::Relaxed), Ordering::Relaxed);
                    gate.wait();
                }
            });
        }
        for _ in 0..rounds {
            gate.wait();
            gate.wait();
            if seen.iter().all(|r| r.load(Ordering::Relaxed) == 0) {
                both_zero.fetch_add(1, Ordering::Relaxed);
            }
            x.store(0, Ordering::Relaxed);
            y.store(0, Ordering::Relaxed);
            for r in &seen {
                r.store(9, Ordering::Relaxed);
            }
        }
    });
    both_zero.load(Ordering::Relaxed)
}

/// `core::sync`'s "The ladder is about deadlock, not about the guest's memory
/// model".
///
/// Recorded there, on the author's x86-64 host: tens to hundreds of forbidden
/// outcomes in 200 000 rounds with little or nothing between the store and the
/// load, and none at all once about forty nanoseconds separate them. The count
/// itself is scheduler noise; where it reaches zero is the number that matters,
/// because that is the window a guest barrier would have to cover.
#[test]
#[ignore = "a measurement, not a gate"]
fn how_long_the_host_keeps_a_store_invisible() {
    const ROUNDS: usize = 200_000;
    for pad in [0u64, 1, 2, 4, 8, 16, 32, 64, 128] {
        let n = store_buffer_rounds(pad, ROUNDS);
        println!("pad={pad:>4}: forbidden outcome {n} / {ROUNDS}");
    }
}

// ---------------------------------------------------------------------------
// What the fence itself costs
// ---------------------------------------------------------------------------

/// What a guest barrier costs now that the three interpreters execute one.
///
/// The measurement the decision needed: the exclusive monitor's shape turned on
/// 1.6 ns per store and the bus lock on ~13 ns per locked instruction, so a
/// barrier had to be priced in the same currency before it could be argued
/// about. It is measured against the same store the litmus uses, because a
/// fence in isolation is not a thing a guest ever executes and because an empty
/// loop with a fence in it measures the loop.
///
/// Recorded on the author's x86-64 host: a relaxed byte store alone, the same
/// store followed by a `SeqCst` fence, the same store followed by the
/// unconditional relaxed `fetch_or` `RamStore::mark_dirty` used to do after
/// every write, and the same store followed by the test-before-set it does
/// now.
///
/// | | ns |
/// | --- | --- |
/// | store | 0.20 |
/// | store + `fence(SeqCst)` | 3.96 |
/// | store + unconditional `fetch_or` | 3.97 |
/// | store + test-before-set | 0.20 |
///
/// The third row was the one that mattered and the fourth is why it does not
/// any more. A locked read-modify-write is itself a full barrier on this
/// architecture and costs what the fence costs, to within noise — which is why
/// `tests/memory_model_litmus.rs` used to find the guest's `MFENCE` changing
/// nothing *here* while it would change everything on a weakly ordered host.
/// Testing the bit first removes the locked instruction in the steady state
/// and the cost goes with it, back to the price of the bare store; that file's
/// table is the same statement in forbidden outcomes rather than nanoseconds.
#[test]
#[ignore = "a measurement, not a gate"]
fn what_a_host_fence_costs() {
    let cell = AtomicU8::new(0);
    let dirty = AtomicU64::new(0);
    timed("store", || {
        for i in 0..REPS {
            cell.store(black_box(i) as u8, Ordering::Relaxed);
        }
    });
    timed("store + fence(SeqCst)", || {
        for i in 0..REPS {
            cell.store(black_box(i) as u8, Ordering::Relaxed);
            std::sync::atomic::fence(Ordering::SeqCst);
        }
    });
    timed("store + unconditional fetch_or", || {
        for i in 0..REPS {
            cell.store(black_box(i) as u8, Ordering::Relaxed);
            dirty.fetch_or(1, Ordering::Relaxed);
        }
    });
    // What `mark_dirty` does now. The bit is set by the first iteration and
    // the branch is not taken again, which is the steady state a guest store
    // path is in for every page after its first write.
    timed("store + mark_dirty (test-before-set)", || {
        for i in 0..REPS {
            cell.store(black_box(i) as u8, Ordering::Relaxed);
            if dirty.load(Ordering::Relaxed) & 1 == 0 {
                dirty.fetch_or(1, Ordering::Relaxed);
            }
        }
    });
}
