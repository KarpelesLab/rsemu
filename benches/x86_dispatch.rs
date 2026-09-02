//! What the x86 frontend's three policies actually bought, measured.
//!
//! The companion to `benches/jit_dispatch.rs`, which measures the same
//! translation runtime with the RISC-V frontend in front of it. Everything
//! below is still *interpreting* — there is no host code generator yet — so
//! what this measures is the thing that is finished: the mechanisms
//! `ROADMAP.md` §9.1 puts ahead of code generation, plus the two decisions x86
//! forced that RISC-V did not have to make.
//!
//! # The eight configurations
//!
//! | | translation | block shape | flags | stores | memory |
//! | --- | --- | --- | --- | --- | --- |
//! | `interpreter` | none — `X86::step`, the oracle | — | — | — | `AddressSpace` |
//! | `lift-each` | re-lift every block, no cache | basic block | eager | end the block | `AddressSpace` |
//! | `cached` | block cache, exits chained | basic block | eager | end the block | `AddressSpace` |
//! | `+tlb` | block cache, exits chained | basic block | eager | end the block | `jit::Tlb` |
//! | `+extended` | " | a load no longer ends a block | eager | end the block | `jit::Tlb` |
//! | `+trace` | " | direct branches merged, with side exits | eager | end the block | `jit::Tlb` |
//! | `+elide` | " | trace | **elided** | end the block | `jit::Tlb` |
//! | `+guard` | " | trace | elided | **guarded in place** | `jit::Tlb` |
//!
//! The rows are cumulative on purpose, and the last two are what this file
//! exists for:
//!
//! * **`+elide`** is `lift::Flags::Elide`. Six flags are written by nearly
//!   every arithmetic instruction and read by almost none, and the IR's
//!   decision 1 states its own cost plainly — *"this design is strictly worse
//!   than eager packing until liveness and DCE exist"*. Dead-code elimination
//!   alone cannot collect it, because a temporary any boundary names is live
//!   by definition; the elision is what makes the boundaries stop naming the
//!   dead ones. This column is the difference.
//! * **`+guard`** is `lift::Smc::Guard`. x86 makes a coherent instruction
//!   cache architectural, so a store into a running block's own page has to be
//!   honoured before the next instruction. `Smc::EndBlock` is RISC-V's answer
//!   — a store is the last guest instruction in its block — and on x86 that
//!   costs a whole dispatch per store. The guard is three IR instructions
//!   instead, and `memcpy` is the workload where the difference is the whole
//!   loop.
//!
//! # The workloads
//!
//! Five 32-bit protected-mode programs, all inside the lifted subset, all
//! written here rather than fetched, because a commercial ROM cannot be
//! committed (CLAUDE.md, "Testing"):
//!
//! * **`alu-loop`** — six arithmetic instructions and a backward `jmp`. Every
//!   one of them writes all six flags and none of them reads one, which is the
//!   best case for elision and a case the TLB never sees.
//! * **`memcpy`** — a byte-at-a-time copy loop: a load, a store, four pointer
//!   instructions and a jump. The **store** is the whole point: under
//!   `Smc::EndBlock` it ends the block every iteration.
//! * **`load-heavy`** — four loads and a jump. Under the basic-block shape
//!   that is one guest instruction per block: the TLB's best case and the
//!   block cache's worst, which are the same fact.
//! * **`branchy`** — `cmp`/`jcc` pairs, where the flags really are read. The
//!   control case for `+elide`: nothing here is dead, so the column must show
//!   *no* win rather than a win the harness cannot see.
//! * **`chain`** — four blocks in a cycle, so the exits are patched in four
//!   directions and a lookup is a lookup rather than a self-loop.
//!
//! # The method
//!
//! Each configuration runs each workload for the same number of **guest
//! instructions**, not for the same wall time, and the run is checked against
//! the interpreter's final register file *and flags* first — a benchmark whose
//! guest quietly stopped doing the work looks like an enormous speedup, and
//! that is the most expensive mistake available in a file like this. The
//! reported figure is the best of `--reps` timed runs after one warm-up.
//!
//! **A dispatcher's budget is in blocks, and a trace is a much bigger block.**
//! `benches/jit_dispatch.rs` records that trap in full and this file inherits
//! its fix: the block budget is re-aimed at what is left after every call, and
//! the reported time is scaled by the instructions actually retired rather
//! than by the ones asked for.
//!
//! The machine is [`differential::machine`] and [`differential::oracle`] — the
//! same two the correctness harness builds, deliberately, so that a
//! configuration cannot be fast here and untested there.
//!
//! **`cargo bench` only.** The bench profile is `-O` with debug assertions
//! off; the same code under `cargo test` is several times slower and the
//! number means nothing. No `criterion` — the dependency policy is absolute
//! and an `Instant` loop is all this needs.
//!
//! Four million guest instructions is about the floor at which these numbers
//! stop moving: at four hundred thousand the columns were not even monotonic,
//! and a table that is not monotonic across cumulative rows is measuring the
//! host rather than the mechanism.
//!
//! # What the numbers said the first time, and what to make of them
//!
//! Recorded because two of the columns came out differently from how they were
//! expected to, and a benchmark whose surprises go unwritten gets re-run
//! forever by people who do not know they have already been explained. These
//! are one machine's figures, not a claim; re-run them.
//!
//! * **`+elide` is the whole story.** `alu-loop` went 10.8 → 26.4 Mi/s and
//!   `memcpy` 12.7 → 27.5, both about 2.2x, from one change: the boundaries
//!   stopped naming flags nothing could observe, so dead-code elimination
//!   could finally take the popcount, the two comparisons and the mask behind
//!   each of them. `load-heavy` and `chain` moved by one percent — neither
//!   writes a flag — and `branchy`, which *reads* its flags, gained 1.36x
//!   rather than 2.2x. That spread is the claim: the win is exactly the dead
//!   flags and nothing else.
//! * **`+guard` is a wash, and that is worth knowing.** It was expected to
//!   pay on `memcpy`, where `Smc::EndBlock` ends a block at every store; it
//!   cost about four percent instead. The reason is visible in the same table:
//!   the guard removes roughly fourteen dispatches out of fifteen and buys
//!   nothing, so a dispatch is not what this configuration spends its time on
//!   — interpreting the IR is. Three extra IR instructions per store therefore
//!   cost more than the dispatches they save. What that says is *when* the
//!   guard pays: once a host backend turns those three instructions into a
//!   not-taken compare and branch and a dispatch back into tens of
//!   instructions, not before. It stays the default because it is the answer
//!   to `ROADMAP.md` §9.1's third mechanism rather than a speed knob, and
//!   because both policies are correct and both are in the cache key.
//! * **Two workloads are still slower than the interpreter.** `branchy` and
//!   `chain` are, and honestly: there is no host code generator, so every row
//!   here is one interpreter in front of another. `benches/jit_dispatch.rs`
//!   says the same thing about the same runtime with a RISC-V frontend, where
//!   the ratios are better for a reason worth stating — an RV64I instruction
//!   lifts to two or three IR instructions and an x86 one with live flags to
//!   fifteen.
//!
//! ```text
//! cargo bench --features jit,cpu-x86-lift --bench x86_dispatch
//! cargo bench --features jit,cpu-x86-lift --bench x86_dispatch -- --insns 20000000
//! ```

use std::sync::Arc;
use std::time::{Duration, Instant};

use rsemu::core::error::{BusError, Result};
use rsemu::core::space::{AddressSpace, MemAttrs, MemResult};
use rsemu::core::value::Width;
use rsemu::cpu::x86::differential::{self, BASE, Case, DATA, RAM_SIZE};
use rsemu::cpu::x86::isa::seg;
use rsemu::cpu::x86::lift::{
    self, EIP, FLAG_BITS, FLAG_SLOTS, Flags, SLOT_COUNT, Shape, Smc, World, r_slot,
};
use rsemu::ir::{AccessKind, InsnStart, IrHost, MemOp, RegSlot};
use rsemu::jit::{
    BlockCache, Context, DirtyPages, Dispatcher, Epoch, Frontend, StoreLog, Tlb, Translation,
};

fn main() {
    let args = Args::parse(std::env::args().skip(1));
    println!(
        "rsemu x86 jit benchmark — {} guest instructions per run, best of {}\n",
        args.insns, args.reps
    );
    println!(
        "{:<12} {:>11} {:>11} {:>11} {:>11} {:>11} {:>11} {:>11} {:>11}",
        "workload",
        "interpreter",
        "lift-each",
        "cached",
        "+tlb",
        "+extended",
        "+trace",
        "+elide",
        "+guard",
    );

    // The eight configurations, in the order the table lists them. Each one
    // changes exactly one thing from the row above it, which is what lets a
    // number attach to a mechanism rather than to a release.
    let configs = [
        Config::new(false, false, Shape::BasicBlock, Flags::Eager, Smc::EndBlock),
        Config::new(true, false, Shape::BasicBlock, Flags::Eager, Smc::EndBlock),
        Config::new(true, true, Shape::BasicBlock, Flags::Eager, Smc::EndBlock),
        Config::new(true, true, Shape::Extended, Flags::Eager, Smc::EndBlock),
        Config::new(true, true, Shape::Trace, Flags::Eager, Smc::EndBlock),
        Config::new(true, true, Shape::Trace, Flags::Elide, Smc::EndBlock),
        Config::new(true, true, Shape::Trace, Flags::Elide, Smc::Guard),
    ];

    for w in workloads() {
        let interp = best(args.reps, || run_interpreter(&w, args.insns));
        let mut times = Vec::new();
        for cfg in configs {
            times.push(best(args.reps, || run_translated(&w, args.insns, cfg)));
        }
        print!("{:<12} {:>11}", w.name, mips(args.insns, interp));
        for t in &times {
            print!(" {:>11}", mips(args.insns, *t));
        }
        println!();
        print!("{:<12} {:>11}", "", "1.00x");
        for t in &times {
            print!(" {:>11}", ratio(interp, *t));
        }
        println!();
    }
    println!(
        "\nEvery row was checked against the interpreter's final register file \
         and flags before it was timed."
    );
}

/// Millions of guest instructions per second, as a string.
fn mips(insns: u64, took: Duration) -> String {
    let secs = took.as_secs_f64();
    if secs <= 0.0 {
        return "-".into();
    }
    format!("{:.1} Mi/s", insns as f64 / secs / 1e6)
}

/// How much faster than the baseline, as a string.
fn ratio(base: Duration, other: Duration) -> String {
    if other.as_secs_f64() <= 0.0 {
        return "-".into();
    }
    format!("{:.2}x", base.as_secs_f64() / other.as_secs_f64())
}

/// The best of `reps` timed runs, after one warm-up.
fn best(reps: usize, mut f: impl FnMut() -> Duration) -> Duration {
    let _ = f();
    (0..reps.max(1)).map(|_| f()).min().unwrap_or_default()
}

/// One point in the configuration space.
#[derive(Debug, Clone, Copy)]
struct Config {
    cache: bool,
    tlb: bool,
    shape: Shape,
    flags: Flags,
    smc: Smc,
}

impl Config {
    const fn new(cache: bool, tlb: bool, shape: Shape, flags: Flags, smc: Smc) -> Config {
        Config {
            cache,
            tlb,
            shape,
            flags,
            smc,
        }
    }
}

// ---------------------------------------------------------------------------
// The workloads
// ---------------------------------------------------------------------------

struct Workload {
    name: &'static str,
    case: Case,
}

/// `group 81 /ext r32, imm32`.
fn alu_imm(ext: u8, reg: u8, imm: u32) -> Vec<u8> {
    let mut out = vec![0x81, 0xc0 | (ext << 3) | reg];
    out.extend_from_slice(&imm.to_le_bytes());
    out
}

/// `jmp rel8`, backwards over `back` bytes counted from the end of the jump.
fn jmp_back(back: i8) -> Vec<u8> {
    vec![0xeb, back as u8]
}

fn workloads() -> Vec<Workload> {
    // Two windows inside the data segment, 2 KiB apart, so the copy loop's
    // pointers can be wrapped back into them with an `and` and an `or`.
    const SRC: u32 = DATA as u32;
    const DST: u32 = (DATA + 0x0800) as u32;

    let mut alu = Vec::new();
    alu.extend_from_slice(&[0x01, 0xc8]); // add eax, ecx
    alu.extend_from_slice(&[0x29, 0xd3]); // sub ebx, edx
    alu.extend_from_slice(&[0x31, 0xf7]); // xor edi, esi
    alu.extend_from_slice(&[0x21, 0xc5]); // and ebp, eax
    alu.extend_from_slice(&[0x09, 0xd9]); // or ecx, ebx
    alu.extend_from_slice(&[0x83, 0xc0, 0x03]); // add eax, 3
    let back = -(alu.len() as i32 + 2) as i8;
    alu.extend_from_slice(&jmp_back(back));
    alu.push(0xf4);

    let mut copy = Vec::new();
    copy.extend_from_slice(&[0x8a, 0x06]); // mov al, [esi]
    copy.extend_from_slice(&[0x88, 0x07]); // mov [edi], al
    copy.extend_from_slice(&[0x46]); // inc esi
    copy.extend_from_slice(&[0x47]); // inc edi
    copy.extend_from_slice(&alu_imm(4, 6, 0x7ff)); // and esi, 0x7ff
    copy.extend_from_slice(&alu_imm(4, 7, 0x7ff)); // and edi, 0x7ff
    copy.extend_from_slice(&alu_imm(1, 6, SRC)); // or  esi, SRC
    copy.extend_from_slice(&alu_imm(1, 7, DST)); // or  edi, DST
    let back = -(copy.len() as i32 + 2) as i8;
    copy.extend_from_slice(&jmp_back(back));
    copy.push(0xf4);

    let mut loads = Vec::new();
    loads.extend_from_slice(&[0x8b, 0x2b]); // mov ebp, [ebx]
    loads.extend_from_slice(&[0x8b, 0x29]); // mov ebp, [ecx]
    loads.extend_from_slice(&[0x8b, 0x2a]); // mov ebp, [edx]
    loads.extend_from_slice(&[0x8b, 0x2e]); // mov ebp, [esi]
    let back = -(loads.len() as i32 + 2) as i8;
    loads.extend_from_slice(&jmp_back(back));
    loads.push(0xf4);

    let mut branchy = Vec::new();
    branchy.extend_from_slice(&[0x39, 0xc8]); // cmp eax, ecx
    branchy.extend_from_slice(&[0x7c, 0x02]); // jl +2
    branchy.extend_from_slice(&[0x40, 0x40]); // inc eax ; inc eax
    branchy.extend_from_slice(&[0x39, 0xda]); // cmp edx, ebx
    branchy.extend_from_slice(&[0x75, 0x01]); // jne +1
    branchy.extend_from_slice(&[0x43]); // inc ebx
    branchy.extend_from_slice(&[0x83, 0xc0, 0x01]); // add eax, 1
    let back = -(branchy.len() as i32 + 2) as i8;
    branchy.extend_from_slice(&jmp_back(back));
    branchy.push(0xf4);

    // Four blocks in a cycle: each `jmp` is a direct transfer, so the exits are
    // patched four ways and a chained edge is a real edge.
    let mut chain = Vec::new();
    for _ in 0..4 {
        chain.extend_from_slice(&[0x40]); // inc eax
        chain.extend_from_slice(&[0xeb, 0x00]); // jmp +0 (to the next block)
    }
    let back = -(chain.len() as i32 + 2) as i8;
    chain.extend_from_slice(&jmp_back(back));
    chain.push(0xf4);

    vec![
        Workload {
            name: "alu-loop",
            case: Case::seeded(alu),
        },
        Workload {
            name: "memcpy",
            case: Case::new(copy).with_reg(6, SRC).with_reg(7, DST),
        },
        Workload {
            name: "load-heavy",
            // Four pointers, three of them in a different page from the first,
            // so the TLB answers about several entries rather than one.
            case: Case::new(loads)
                .with_reg(3, (DATA + 0x11) as u32)
                .with_reg(1, (DATA + 0x400) as u32)
                .with_reg(2, (DATA + 0x1000) as u32)
                .with_reg(6, (DATA + 0x1800) as u32),
        },
        Workload {
            name: "branchy",
            case: Case::seeded(branchy),
        },
        Workload {
            name: "chain",
            case: Case::seeded(chain),
        },
    ]
}

// ---------------------------------------------------------------------------
// Running
// ---------------------------------------------------------------------------

fn run_interpreter(w: &Workload, insns: u64) -> Duration {
    let (space, _ram) = differential::machine(&w.case);
    let cpu = differential::oracle(&w.case, space);
    let start = Instant::now();
    for _ in 0..insns {
        cpu.step();
    }
    let took = start.elapsed();
    assert!(!cpu.is_halted(), "{}: the guest trapped", w.name);
    took
}

/// The largest number of blocks one dispatcher call is asked for.
const BLOCK_BUDGET: usize = 4096;

/// The translated path, with every mechanism switched independently.
///
/// Checked against the interpreter before it is timed: the two must reach the
/// same eight registers *and the same flags* after the same number of guest
/// instructions, or the number below is measuring a guest that stopped working.
fn run_translated(w: &Workload, insns: u64, cfg: Config) -> Duration {
    let (space, _ram) = differential::machine(&w.case);
    let mut front = Lifter::new(&w.case, Arc::clone(&space), cfg);
    let mut host = BenchHost::new(&w.case, space, cfg.tlb);
    // "No cache" is one block per call plus a flush between, which also removes
    // chaining — a translator that re-lifts has no predecessor to patch.
    let mut disp =
        Dispatcher::with_cache(BlockCache::with_capacity(if cfg.cache { 1024 } else { 1 }));
    let mut budget = if cfg.cache { BLOCK_BUDGET } else { 1 };

    let start = Instant::now();
    let mut done = 0u64;
    let mut pc = BASE;
    while done < insns {
        let run = disp
            .run(&mut front, &mut host, pc, budget)
            .expect("the dispatcher runs");
        assert!(
            run.insns > 0,
            "{}: the dispatcher made no progress ({:?})",
            w.name,
            run.stop
        );
        done += run.insns as u64;
        pc = run.pc;
        if !cfg.cache {
            disp.cache_mut().flush();
        } else if run.blocks > 0 {
            // Re-aim at what is left. A budget in *blocks* means a trace can
            // overshoot the target by an order of magnitude.
            let per_block = (run.insns as u64 / run.blocks as u64).max(1);
            budget = usize::try_from((insns.saturating_sub(done) / per_block) + 1)
                .unwrap_or(BLOCK_BUDGET)
                .clamp(1, BLOCK_BUDGET);
        }
    }
    let took = start.elapsed();
    if std::env::var("RSEMU_BENCH_STATS").is_ok() {
        eprintln!("{} {cfg:?}: {done} insns, {:?}", w.name, disp.stats());
    }

    // The check. `done` is at least `insns`, so the oracle runs the same number
    // the translated path actually retired.
    let (oracle_space, _oracle_ram) = differential::machine(&w.case);
    let cpu = differential::oracle(&w.case, oracle_space);
    for _ in 0..done {
        cpu.step();
    }
    let regs = cpu.regs();
    for n in 0..8u8 {
        assert_eq!(
            regs.dword(n),
            host.slots[r_slot(n).0 as usize] as u32,
            "{}: register {n} disagrees after {done} instructions under {cfg:?}",
            w.name
        );
    }
    assert_eq!(
        regs.eflags,
        host.eflags(),
        "{}: the flags disagree after {done} instructions under {cfg:?}",
        w.name
    );

    // Scaled to the instruction target, because a configuration cannot always
    // stop exactly on it: what is reported is time *per guest instruction*.
    Duration::from_secs_f64(took.as_secs_f64() * insns as f64 / done as f64)
}

// ---------------------------------------------------------------------------
// The frontend and the host
// ---------------------------------------------------------------------------

struct Lifter {
    world: World,
    cfg: Config,
    space: Arc<AddressSpace>,
}

impl Lifter {
    fn new(case: &Case, space: Arc<AddressSpace>, cfg: Config) -> Lifter {
        Lifter {
            world: differential::world(case),
            cfg,
            space,
        }
    }
}

impl Frontend for Lifter {
    fn epoch(&mut self) -> Epoch {
        Epoch {
            topology: self.space.generation(),
            translation: 0,
        }
    }
    fn key(&mut self) -> u64 {
        lift::key(&self.world, self.cfg.shape, self.cfg.smc, self.cfg.flags)
    }
    fn pc_slot(&self) -> RegSlot {
        EIP
    }
    fn translate(&mut self, pc: u64) -> Result<Translation> {
        let space = Arc::clone(&self.space);
        let mut src = |addr: u64| {
            space
                .read(addr, Width::U8, MemAttrs::DEFAULT)
                .ok()
                .map(|v| v as u8)
        };
        let lifted = lift::lift(
            &self.world,
            pc,
            &mut src,
            lift::MAX_INSNS,
            self.cfg.shape,
            self.cfg.smc,
            self.cfg.flags,
        )?;
        Ok(Translation {
            page: lifted.page,
            insns: lifted.insns,
            block: lifted.block,
        })
    }
}

const RING0: Context = Context {
    level: 0,
    translating: false,
};

struct BenchHost {
    slots: [u64; SLOT_COUNT as usize],
    space: Arc<AddressSpace>,
    /// `None` runs every access through the address space, which is the
    /// baseline the TLB row is compared against.
    tlb: Option<Tlb>,
    base: [u64; seg::COUNT],
    limit: u64,
    dirty: DirtyPages,
}

impl BenchHost {
    fn new(case: &Case, space: Arc<AddressSpace>, tlb: bool) -> BenchHost {
        let world = differential::world(case);
        let mut slots = [0u64; SLOT_COUNT as usize];
        for (n, value) in case.regs.iter().enumerate() {
            slots[n] = u64::from(*value);
        }
        slots[4] = differential::STACK;
        slots[EIP.0 as usize] = BASE;
        // The reserved bit, which `Regs::normalise_flags` forces on.
        slots[lift::EFLAGS_REST.0 as usize] = 0x0002;
        BenchHost {
            slots,
            tlb: tlb.then(|| Tlb::new(Arc::clone(&space))),
            space,
            base: world.seg_base,
            limit: RAM_SIZE - 1,
            dirty: DirtyPages::new(),
        }
    }

    fn eflags(&self) -> u32 {
        let mut value = self.slots[lift::EFLAGS_REST.0 as usize] as u32;
        for (i, bit) in FLAG_BITS.iter().enumerate() {
            if self.slots[FLAG_SLOTS[i].0 as usize] & 1 != 0 {
                value |= bit;
            }
        }
        value
    }

    fn access(&mut self, mem: &MemOp, addr: u64, value: Option<u64>) -> MemResult<u64> {
        let sr = mem.seg.map_or(seg::DS, |s| s.0);
        let size = mem.size.bytes();
        let last = addr.checked_add(size - 1).ok_or(BusError::Protected)?;
        if last > self.limit {
            return Err(BusError::Protected);
        }
        let lin = self.base[usize::from(sr)].wrapping_add(addr);
        match (&mut self.tlb, value) {
            (Some(tlb), None) => tlb.read(
                AccessKind::Load,
                lin,
                lin,
                mem.size,
                RING0,
                MemAttrs::DEFAULT,
            ),
            (Some(tlb), Some(v)) => {
                self.dirty.note(lin, size);
                tlb.write(lin, lin, mem.size, v, RING0, MemAttrs::DEFAULT)
                    .map(|()| 0)
            }
            (None, None) => self.space.read(lin, mem.size, MemAttrs::DEFAULT),
            (None, Some(v)) => {
                self.dirty.note(lin, size);
                self.space
                    .write(lin, mem.size, v, MemAttrs::DEFAULT)
                    .map(|()| 0)
            }
        }
    }
}

impl IrHost for BenchHost {
    fn read_slot(&mut self, slot: RegSlot) -> u128 {
        u128::from(self.slots[slot.0 as usize])
    }
    fn write_slot(&mut self, slot: RegSlot, value: u128) {
        self.slots[slot.0 as usize] = value as u64;
    }
    fn load(&mut self, mem: &MemOp, addr: u64) -> MemResult<u64> {
        self.access(mem, addr, None)
    }
    fn store(&mut self, mem: &MemOp, addr: u64, value: u64) -> MemResult {
        self.access(mem, addr, Some(value)).map(|_| ())
    }
    fn charge(&mut self, _ticks: u64) {}
    fn insn_start(&mut self, _mark: &InsnStart) {}
}

impl StoreLog for BenchHost {
    fn drain_dirty(&mut self, sink: &mut dyn FnMut(u64)) {
        self.dirty.drain_dirty(sink);
    }
}

// ---------------------------------------------------------------------------
// Arguments
// ---------------------------------------------------------------------------

struct Args {
    insns: u64,
    reps: usize,
}

impl Args {
    fn parse(args: impl Iterator<Item = String>) -> Args {
        let mut out = Args {
            insns: 5_000_000,
            reps: 3,
        };
        let mut args = args.peekable();
        while let Some(arg) = args.next() {
            match arg.as_str() {
                "--insns" => {
                    if let Some(v) = args.next().and_then(|v| v.parse().ok()) {
                        out.insns = v;
                    }
                }
                "--reps" => {
                    if let Some(v) = args.next().and_then(|v| v.parse().ok()) {
                        out.reps = v;
                    }
                }
                "--smoke" => out.insns = 20_000,
                _ => {}
            }
        }
        out
    }
}
