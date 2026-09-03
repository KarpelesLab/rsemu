//! What the software TLB and the block cache actually bought, measured.
//!
//! `ROADMAP.md` §13's phase-8 gate is a wall-clock number. What this measures
//! is each of §9.1's mechanisms, switched on one at a time so the number
//! attaches to a mechanism rather than to a release — and, in the last column,
//! the one they were all waiting for: the **x86-64 host code generator**.
//!
//! # The seven configurations
//!
//! | | translation | block shape | memory | engine |
//! | --- | --- | --- | --- | --- |
//! | `interpreter` | none — `Hart::step`, the oracle | — | `AddressSpace` | — |
//! | `lift-each-time` | re-lift every block, no cache | basic block | `AddressSpace` | `ir::Interp` |
//! | `cached` | block cache, exits chained | basic block | `AddressSpace` | `ir::Interp` |
//! | `cached+tlb` | block cache, exits chained | basic block | `jit::Tlb` | `ir::Interp` |
//! | `+extended` | block cache, exits chained | a load no longer ends a block | `jit::Tlb` | `ir::Interp` |
//! | `+superblock` | block cache, exits chained | direct branches merged, with side exits | `jit::Tlb` | `ir::Interp` |
//! | `+compiled` | " | " | `jit::Tlb`, **inlined** | `jit::x86`, temporaries in a frame |
//! | `+allocated` | " | " | " | `jit::x86`, **linear scan** |
//!
//! The last two columns are `ROADMAP.md` §9's first backend, and `+compiled`
//! is also where the TLB finally does what §9.1 asks of it — *"the fast path
//! is inlined into generated code: mask, compare, add, load"*. Every earlier
//! row reaches the TLB through a call.
//!
//! **`+allocated` is §9's pipeline finished** — *"register allocation (linear
//! scan) → host backend"*. It is a separate column rather than a replacement
//! because `jit::x86::Regs::Frame` is still a supported policy and is what the
//! backend's differential runs as its control, so the two are measurable
//! against each other rather than against a number in a commit message.
//!
//! It is only a column where there is a backend to run it: `jit-x86` is
//! `cfg`-gated to an x86-64 Linux host, and elsewhere it repeats
//! `+superblock`. Build with `--features jit-x86,cpu-riscv-lift` to get it.
//!
//! The rows are cumulative on purpose: `lift-each-time` is the honest
//! starting point for a translator, because a translator that re-lifts is
//! slower than the interpreter it replaces, and the cache is what turns that
//! around. Reporting the cache's win against the interpreter instead would
//! credit it with the lifter's speed as well.
//!
//! The last two rows exist because a claim about superblocks needs a baseline
//! that is still runnable rather than a number from a previous commit. They
//! are [`lift::Shape`]'s three values, and the corpus in
//! `tests/riscv_lift_differential.rs` runs all three against the interpreter,
//! so a shape that got faster by getting wrong would fail a test rather than
//! win a column.
//!
//! # The workloads
//!
//! Four RISC-V programs, all inside the lifted RV64I subset, all written here
//! rather than fetched, because a commercial ROM cannot be committed
//! (CLAUDE.md, "Testing"):
//!
//! * **`alu-loop`** — a tight arithmetic loop, one block, chained to itself.
//!   The best case for the block cache and a case the TLB never sees, since it
//!   touches no memory. Its back edge is a direct `JAL`, so a trace unrolls it.
//! * **`memcpy`** — a byte-at-a-time copy loop: a load, a store, six
//!   arithmetic instructions and a branch. Every iteration crosses the memory
//!   path twice, and the **store** ends the block under every shape
//!   (`cpu::riscv::lift`, "A store still ends the block"), so this is the
//!   workload that shows what that rule costs.
//! * **`load-heavy`** — four loads and a jump. Under the basic-block shape the
//!   lifter ended a block at every access, so this was one guest instruction
//!   per block and the highest ratio of memory work to everything else the
//!   subset can express: the TLB's best case *and* the block cache's worst,
//!   which are the same fact. It is the row the `+extended` column was written
//!   for.
//! * **`chain`** — four blocks in a cycle, so the exits are patched in four
//!   directions and a lookup is a lookup rather than a self-loop. Every edge
//!   is a direct `JAL`, so a trace swallows the whole cycle.
//!
//! # The method
//!
//! Each configuration runs each workload for the same number of **guest
//! instructions**, not for the same wall time, and the run is checked against
//! the interpreter's final register file first — a benchmark whose guest
//! quietly stopped doing the work looks like an enormous speedup, and that is
//! the most expensive mistake available in a file like this. The reported
//! figure is the best of `--reps` timed runs after one warm-up, because the
//! minimum is the sample least polluted by the host's other work.
//!
//! **A dispatcher's budget is in blocks, and a trace is a much bigger block,**
//! which is a trap this file fell into and is worth writing down. The timed
//! loop asks for blocks until the instruction target is met, so a fixed block
//! budget overshoots by up to one budget's worth of blocks: 0.6% of a five
//! million instruction run under basic blocks, and **thirteen times the whole
//! run** under a trace, which read as the superblock column being four times
//! *slower* than no translation at all. Two things fix it, and both are here
//! because either alone would leave a number that is nearly right for the
//! wrong reason: the block budget is re-aimed at what is left after every
//! call, and the reported time is scaled by the instructions actually retired
//! rather than by the ones asked for.
//!
//! **`cargo bench` only.** The bench profile is `-O` with debug assertions
//! off; the same code under `cargo test` is several times slower and the
//! number means nothing. No `criterion` — the dependency policy is absolute
//! and an `Instant` loop is all this needs.
//!
//! ```text
//! cargo bench --features jit-x86,cpu-riscv-lift --bench jit_dispatch
//! cargo bench --features jit-x86,cpu-riscv-lift --bench jit_dispatch -- --insns 20000000
//! ```

use std::sync::Arc;
use std::time::{Duration, Instant};

use rsemu::core::error::Result;
use rsemu::core::space::{AddressSpace, MemAttrs, MemResult, RamStore, Region, UnassignedPolicy};
use rsemu::core::value::Width;
use rsemu::cpu::riscv::lift::{self, Origin, PC, SLOT_COUNT, Shape, x_slot};
use rsemu::cpu::riscv::{Config, Hart};
use rsemu::ir::{AccessKind, Align, InsnStart, IrHost, MemOp, RegSlot};
use rsemu::jit::{
    BlockCache, Context, DirtyPages, Dispatcher, Epoch, FastMem, Frontend, LoadPlan, StoreLog, Tlb,
    Translation,
};

/// Where the program is loaded and RAM is mapped.
const BASE: u64 = 0x8000_0000;
/// Sixteen pages: one of code, the rest for the copy loop to work in.
const RAM_SIZE: u64 = 16 * 4096;
/// Where the data window starts, as an offset from [`BASE`].
const DATA: u64 = 4096;

fn main() {
    let args = Args::parse(std::env::args().skip(1));
    println!(
        "rsemu jit benchmark — {} guest instructions per run, best of {}\n",
        args.insns, args.reps
    );
    println!(
        "{:<12} {:>11} {:>11} {:>11} {:>11} {:>11} {:>11} {:>11} {:>11}",
        "workload",
        "interpreter",
        "lift-each",
        "cached",
        "cached+tlb",
        "+extended",
        "+superblock",
        "+compiled",
        "+allocated",
    );
    for w in workloads() {
        let basic = Shape::BasicBlock;
        let interp = best(args.reps, || run_interpreter(&w, args.insns));
        let cold = best(args.reps, || {
            run_translated(&w, args.insns, false, false, basic, Engine::Interp)
        });
        let cached = best(args.reps, || {
            run_translated(&w, args.insns, true, false, basic, Engine::Interp)
        });
        let tlb = best(args.reps, || {
            run_translated(&w, args.insns, true, true, basic, Engine::Interp)
        });
        let extended = best(args.reps, || {
            run_translated(&w, args.insns, true, true, Shape::Extended, Engine::Interp)
        });
        let trace = best(args.reps, || {
            run_translated(&w, args.insns, true, true, Shape::Trace, Engine::Interp)
        });
        let jit = best(args.reps, || {
            run_translated(&w, args.insns, true, true, Shape::Trace, Engine::Frame)
        });
        let scan = best(args.reps, || {
            run_translated(&w, args.insns, true, true, Shape::Trace, Engine::Scan)
        });
        println!(
            "{:<12} {:>11} {:>11} {:>11} {:>11} {:>11} {:>11} {:>11} {:>11}",
            w.name,
            mips(args.insns, interp),
            mips(args.insns, cold),
            mips(args.insns, cached),
            mips(args.insns, tlb),
            mips(args.insns, extended),
            mips(args.insns, trace),
            mips(args.insns, jit),
            mips(args.insns, scan),
        );
        println!(
            "{:<12} {:>11} {:>11} {:>11} {:>11} {:>11} {:>11} {:>11} {:>11}",
            "",
            "1.00x",
            ratio(interp, cold),
            ratio(interp, cached),
            ratio(interp, tlb),
            ratio(interp, extended),
            ratio(interp, trace),
            ratio(interp, jit),
            ratio(interp, scan),
        );
        println!(
            "{:<12} {:>83} {:>11}",
            "",
            "the allocator against the frame:",
            ratio(jit, scan),
        );
    }
    println!(
        "\nEvery row was checked against the interpreter's final register file \
         before it was timed."
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

// ---------------------------------------------------------------------------
// The workloads
// ---------------------------------------------------------------------------

struct Workload {
    name: &'static str,
    program: Vec<u32>,
    regs: [u64; 32],
}

fn workloads() -> Vec<Workload> {
    let mut memcpy_regs = [0u64; 32];
    // x1 = source, x2 = destination.
    memcpy_regs[1] = BASE + DATA;
    memcpy_regs[2] = BASE + DATA + 4096;
    // Four pointers, three of them in a different page from the first, so the
    // TLB is answering about several entries rather than one.
    let mut load_regs = [0u64; 32];
    load_regs[1] = BASE + DATA + 0x11;
    load_regs[2] = BASE + DATA + 0x1000;
    load_regs[3] = BASE + DATA + 0x2000;
    load_regs[4] = BASE + DATA + 0x3000;
    vec![
        Workload {
            // Six adds and a backward branch: one block, chained to itself.
            name: "alu-loop",
            program: vec![
                addi(10, 10, 1),
                addi(11, 10, 3),
                add(12, 11, 10),
                slli(13, 12, 1),
                xor(14, 13, 11),
                sub(15, 14, 10),
                jal(0, -24),
            ],
            regs: [0; 32],
        },
        Workload {
            // A byte copy that wraps its pointers back inside the window, so
            // it runs forever without leaving RAM. Two memory accesses and a
            // branch per iteration.
            name: "memcpy",
            program: vec![
                lbu(5, 1, 0),      // x5 = [x1]
                sb(2, 5, 0),       // [x2] = x5
                addi(1, 1, 1),     // x1++
                addi(2, 2, 1),     // x2++
                andi(1, 1, 0x7ff), // wrap into a 2 KiB window — 0x7ff is the
                andi(2, 2, 0x7ff), // widest mask a signed 12-bit immediate holds
                add(1, 1, 20),     // x20 = BASE + DATA
                add(2, 2, 21),     // x21 = BASE + DATA + 4096
                jal(0, -32),
            ],
            regs: memcpy_regs,
        },
        Workload {
            // Four loads and a jump: the lifter ends a block at every access,
            // so this is one guest instruction per block and the highest ratio
            // of memory work to everything else that the subset can express.
            // If the software TLB does not show here, it does not show.
            name: "load-heavy",
            program: vec![
                lbu(5, 1, 0),
                lbu(6, 2, 0),
                lbu(7, 3, 0),
                lbu(8, 4, 0),
                jal(0, -16),
            ],
            regs: load_regs,
        },
        Workload {
            // Four blocks in a cycle: each `jal` ends a block, so the exits are
            // patched four ways and a chained edge is a real edge.
            name: "chain",
            program: vec![
                addi(10, 10, 1),
                jal(0, 8),
                addi(11, 11, 1), // never reached from above
                addi(12, 12, 1),
                jal(0, 8),
                addi(13, 13, 1), // never reached
                addi(14, 14, 1),
                jal(0, -24),
            ],
            regs: [0; 32],
        },
    ]
}

// The RV64I encodings the workloads use, from *The RISC-V Instruction Set
// Manual, Volume I* (CC-BY-4.0), chapter 2's format tables.
const fn i_type(opcode: u32, funct3: u32, rd: u32, rs1: u32, imm: i32) -> u32 {
    ((imm as u32 & 0xfff) << 20) | (rs1 << 15) | (funct3 << 12) | (rd << 7) | opcode
}
const fn r_type(funct3: u32, funct7: u32, rd: u32, rs1: u32, rs2: u32) -> u32 {
    (funct7 << 25) | (rs2 << 20) | (rs1 << 15) | (funct3 << 12) | (rd << 7) | 0x33
}
const fn addi(rd: u32, rs1: u32, imm: i32) -> u32 {
    i_type(0x13, 0, rd, rs1, imm)
}
const fn andi(rd: u32, rs1: u32, imm: i32) -> u32 {
    i_type(0x13, 7, rd, rs1, imm)
}
const fn slli(rd: u32, rs1: u32, sh: u32) -> u32 {
    i_type(0x13, 1, rd, rs1, sh as i32)
}
const fn add(rd: u32, rs1: u32, rs2: u32) -> u32 {
    r_type(0, 0, rd, rs1, rs2)
}
const fn sub(rd: u32, rs1: u32, rs2: u32) -> u32 {
    r_type(0, 0x20, rd, rs1, rs2)
}
const fn xor(rd: u32, rs1: u32, rs2: u32) -> u32 {
    r_type(4, 0, rd, rs1, rs2)
}
const fn lbu(rd: u32, rs1: u32, imm: i32) -> u32 {
    i_type(0x03, 4, rd, rs1, imm)
}
const fn sb(rs1: u32, rs2: u32, imm: i32) -> u32 {
    let imm = imm as u32;
    (((imm >> 5) & 0x7f) << 25) | (rs2 << 20) | (rs1 << 15) | ((imm & 0x1f) << 7) | 0x23
}
const fn jal(rd: u32, imm: i32) -> u32 {
    let imm = imm as u32;
    0x6f | (rd << 7)
        | (((imm >> 12) & 0xff) << 12)
        | (((imm >> 11) & 1) << 20)
        | (((imm >> 1) & 0x3ff) << 21)
        | (((imm >> 20) & 1) << 31)
}

// ---------------------------------------------------------------------------
// The machines
// ---------------------------------------------------------------------------

fn machine(w: &Workload) -> (Arc<AddressSpace>, Arc<RamStore>) {
    let ram = Arc::new(RamStore::new(RAM_SIZE));
    for (n, word) in w.program.iter().enumerate() {
        for (k, byte) in word.to_le_bytes().iter().enumerate() {
            ram.write_u8(n as u64 * 4 + k as u64, *byte).expect("fits");
        }
    }
    // Fill the source window with something, so the copy loop moves real bytes.
    for i in 0..4096u64 {
        ram.write_u8(DATA + i, (i * 31) as u8).expect("fits");
    }
    let space = AddressSpace::new("mem", 64).with_unassigned(UnassignedPolicy::FAULT);
    space
        .topology()
        .map(Region::ram("ram", Arc::clone(&ram)), BASE)
        .expect("one region maps");
    (Arc::new(space), ram)
}

fn seeded_hart(w: &Workload, space: Arc<AddressSpace>) -> Hart {
    let hart = Hart::new(Config::rv64i().with_reset_vector(BASE));
    hart.attach_space(space);
    for (n, value) in w.regs.iter().enumerate().skip(1) {
        hart.set_x(n as u32, *value);
    }
    // The two constants the copy loop adds back after masking.
    hart.set_x(20, BASE + DATA);
    hart.set_x(21, BASE + DATA + 4096);
    hart
}

fn run_interpreter(w: &Workload, insns: u64) -> Duration {
    let (space, _ram) = machine(w);
    let hart = seeded_hart(w, space);
    let start = Instant::now();
    for _ in 0..insns {
        hart.step();
    }
    let took = start.elapsed();
    assert_eq!(hart.csrs().mcause, 0, "{}: the guest trapped", w.name);
    took
}

/// Which engine executes a block, and under which register policy.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Engine {
    /// `ir::Interp`.
    Interp,
    /// The host code generator with every temporary in a frame slot: the
    /// backend before `ROADMAP.md` §9's register allocator.
    Frame,
    /// The host code generator with linear-scan register allocation.
    Scan,
}

/// The translated path, with the cache and the TLB switched independently.
///
/// Checked against the interpreter before it is timed: the two must reach the
/// same register file after the same number of guest instructions, or the
/// number below is measuring a guest that stopped working.
fn run_translated(
    w: &Workload,
    insns: u64,
    cache: bool,
    tlb: bool,
    shape: Shape,
    engine: Engine,
) -> Duration {
    let (space, _ram) = machine(w);
    let mut front = Lifter::new(Arc::clone(&space), shape);
    let mut host = BenchHost::new(w, space, tlb);
    // "No cache" is one block per call plus a flush between, which also
    // removes chaining — a translator that re-lifts has no predecessor to
    // patch. Running many blocks per call and flushing afterwards would leave
    // the cache live inside the call and measure something else. Its cache is
    // sized to one block so that the flush itself is not what is being timed.
    let mut disp = Dispatcher::with_cache(BlockCache::with_capacity(if cache { 1024 } else { 1 }));
    if engine != Engine::Interp {
        disp = with_backend(disp, engine == Engine::Scan);
    }
    let mut budget = if cache { BLOCK_BUDGET } else { 1 };

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
        if !cache {
            disp.cache_mut().flush();
        } else if run.blocks > 0 {
            // Re-aim at what is left. A budget in *blocks* means a trace can
            // overshoot the target by an order of magnitude, and a run that
            // did thirteen times the work in the same time is not a slow run
            // (see "The method").
            let per_block = (run.insns as u64 / run.blocks as u64).max(1);
            budget = usize::try_from((insns.saturating_sub(done) / per_block) + 1)
                .unwrap_or(BLOCK_BUDGET)
                .clamp(1, BLOCK_BUDGET);
        }
    }
    let took = start.elapsed();
    // A diagnostic rather than decoration: nearly every wrong number this file
    // has produced was a dispatcher that stopped caching or a run that did the
    // wrong amount of work, and both are visible here in one line.
    if std::env::var("RSEMU_BENCH_STATS").is_ok() {
        eprintln!("{} {shape:?}: {done} insns, {:?}", w.name, disp.stats());
    }

    // The check. `done` is at least `insns`, so the oracle runs the same
    // number the translated path actually retired.
    let (oracle_space, _oracle_ram) = machine(w);
    let hart = seeded_hart(w, oracle_space);
    for _ in 0..done {
        hart.step();
    }
    for n in 1..32u32 {
        assert_eq!(
            hart.x(n),
            host.slots[x_slot(n).0 as usize],
            "{}: x{n} disagrees after {done} instructions under {shape:?}",
            w.name
        );
    }
    // Scaled to the instruction target, because a configuration cannot always
    // stop exactly on it: what is being reported is time *per guest
    // instruction*, and dividing a run's time by the count it was asked for
    // rather than the count it retired credits an overshoot as slowness.
    Duration::from_secs_f64(took.as_secs_f64() * insns as f64 / done as f64)
}

/// Attach the host code generator, where this build has one.
///
/// `ROADMAP.md` §9's x86-64 backend is behind `jit-x86` and `cfg`-gated to an
/// x86-64 Linux host, so this file builds and runs everywhere and the compiled
/// column simply repeats the `+superblock` one where there is no backend.
#[cfg(all(feature = "jit-x86", target_os = "linux", target_arch = "x86_64"))]
fn with_backend(disp: Dispatcher, allocate: bool) -> Dispatcher {
    let mut engine = rsemu::jit::x86::Engine::new().expect("a W^X code buffer");
    engine.set_regs(if allocate {
        rsemu::jit::x86::Regs::Scan
    } else {
        rsemu::jit::x86::Regs::Frame
    });
    disp.with_backend(engine)
}

#[cfg(not(all(feature = "jit-x86", target_os = "linux", target_arch = "x86_64")))]
fn with_backend(disp: Dispatcher, _allocate: bool) -> Dispatcher {
    disp
}

/// The largest number of blocks one dispatcher call is asked for.
///
/// Also the largest overshoot in blocks, which is why the loop above re-aims
/// it rather than leaving it at the maximum.
const BLOCK_BUDGET: usize = 4096;

// ---------------------------------------------------------------------------
// The frontend and the host
// ---------------------------------------------------------------------------

struct Lifter {
    cfg: Config,
    shape: Shape,
    space: Arc<AddressSpace>,
}

impl Lifter {
    fn new(space: Arc<AddressSpace>, shape: Shape) -> Lifter {
        Lifter {
            cfg: Config::rv64i(),
            shape,
            space,
        }
    }
}

impl<H: ?Sized> Frontend<H> for Lifter {
    fn epoch(&mut self) -> Epoch {
        Epoch {
            topology: self.space.generation(),
            translation: 0,
        }
    }
    fn key(&mut self) -> u64 {
        lift::key(&self.cfg, Origin::Bare, self.shape)
    }
    fn pc_slot(&self) -> RegSlot {
        PC
    }
    fn translate(&mut self, pc: u64) -> Result<Translation> {
        let space = Arc::clone(&self.space);
        let mut src = |addr: u64| {
            space
                .read(addr, Width::U16, MemAttrs::DEFAULT)
                .ok()
                .map(|v| v as u16)
        };
        let lifted = lift::lift(
            &self.cfg,
            Origin::Bare,
            pc,
            &mut src,
            lift::MAX_INSNS,
            self.shape,
        )?;
        Ok(Translation {
            page: pc & !rsemu::jit::PAGE_MASK,
            insns: lifted.insns,
            block: lifted.block,
        })
    }
}

const MACHINE: Context = Context {
    level: 3,
    translating: false,
};

struct BenchHost {
    slots: [u64; SLOT_COUNT as usize],
    space: Arc<AddressSpace>,
    /// `None` runs every access through the address space, which is the
    /// baseline the TLB row is compared against.
    tlb: Option<Tlb>,
    dirty: DirtyPages,
}

impl BenchHost {
    fn new(w: &Workload, space: Arc<AddressSpace>, tlb: bool) -> BenchHost {
        let mut slots = [0u64; SLOT_COUNT as usize];
        for (n, value) in w.regs.iter().enumerate().skip(1) {
            slots[n] = *value;
        }
        slots[20] = BASE + DATA;
        slots[21] = BASE + DATA + 4096;
        BenchHost {
            slots,
            tlb: tlb.then(|| Tlb::new(Arc::clone(&space))),
            space,
            dirty: DirtyPages::new(),
        }
    }

    fn once(&mut self, addr: u64, width: Width, value: Option<u64>) -> MemResult<u64> {
        match (&mut self.tlb, value) {
            (Some(tlb), None) => tlb.read(
                AccessKind::Load,
                addr,
                addr,
                width,
                MACHINE,
                MemAttrs::DEFAULT,
            ),
            (Some(tlb), Some(v)) => tlb
                .write(addr, addr, width, v, MACHINE, MemAttrs::DEFAULT)
                .map(|()| 0),
            (None, None) => self.space.read(addr, width, MemAttrs::DEFAULT),
            (None, Some(v)) => self
                .space
                .write(addr, width, v, MemAttrs::DEFAULT)
                .map(|()| 0),
        }
    }

    fn access(&mut self, mem: &MemOp, addr: u64, value: Option<u64>) -> MemResult<u64> {
        let bytes = mem.size.bytes();
        if addr.is_multiple_of(bytes) {
            if value.is_some() {
                self.dirty.note(addr, bytes);
            }
            return self.once(addr, mem.size, value);
        }
        if mem.align == Align::Fault {
            return Err(rsemu::core::error::BusError::BadAccess);
        }
        match value {
            None => {
                let mut got = 0u64;
                for i in 0..bytes {
                    let byte = self.once(addr.wrapping_add(i), Width::U8, None)?;
                    got |= (byte & 0xff) << (8 * i);
                }
                Ok(got)
            }
            Some(v) => {
                self.dirty.note(addr, bytes);
                for i in 0..bytes {
                    self.once(addr.wrapping_add(i), Width::U8, Some(v >> (8 * i)))?;
                }
                Ok(0)
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

/// The seam the compiled column measures: the backend inlines an aligned load
/// into a mask, a compare, an add and a `mov`, out of *this* host's own TLB.
///
/// `note_fast_load` charges nothing because `BenchHost::once` charges nothing —
/// this file measures wall time, and `charge` is a no-op throughout. The
/// correctness of the tick accounting is `cpu::riscv::differential`'s job and
/// is checked there, on the same code path.
impl FastMem for BenchHost {
    fn load_plan(&mut self) -> Option<LoadPlan> {
        self.tlb
            .as_ref()
            .map(|tlb| tlb.plan(AccessKind::Load, MACHINE))
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
