//! What the AArch64 frontend's mechanisms bought, measured one at a time.
//!
//! The third of these, after [`jit_dispatch`](../jit_dispatch/index.html) for
//! RISC-V and `x86_dispatch` for x86, and deliberately the same shape: a
//! cumulative ladder of `ROADMAP.md` §9.1's speed mechanisms, each workload
//! checked against the interpreter before it is timed, `harness = false`
//! because libtest's bench harness is nightly-only and this project is stable,
//! and no `criterion` because the dependency policy is absolute.
//!
//! # Two tables, and the second one is why this file was written
//!
//! **The ladder** is what the other two cores measure — a dispatcher, a
//! frontend and a host, driven directly, with the cache, the software TLB, the
//! block shape and the code generator switched on in turn.
//!
//! **The quantum** is A64's own, and it exists because the largest number in
//! this whole subsystem turned out to live there rather than in the ladder. A
//! real core does not run a dispatcher, it runs `Cpu::run_budget`, and a
//! translated block may only be admitted if its worst case fits what is left
//! of the quantum (`cpu::arm::a64::engine`'s `admit`). That guard used to
//! answer for a PC it knew nothing about with the *cold* worst case — 64
//! instructions of a split, walked pair access, 5 188 ticks — against an
//! `arm64-virt` quantum of 10 000. Over half of every quantum was therefore
//! interpreted an instruction at a time, and it never recovered inside a
//! quantum, because the PC after an interpreted instruction is in the middle
//! of a block and so is cold too.
//!
//! The second table sweeps the quantum so that the cliff is visible rather
//! than described: throughput against budget, per engine. It is the row that
//! would have caught the defect, which is the argument for it being here.
//!
//! # The ladder's configurations
//!
//! | | translation | block shape | memory | engine |
//! | --- | --- | --- | --- | --- |
//! | `interpreter` | none — `Cpu::step`, the oracle | — | `AddressSpace` | — |
//! | `lift-each` | re-lift every block, no cache | basic block | `AddressSpace` | `ir::Interp` |
//! | `cached` | block cache, exits chained | basic block | `AddressSpace` | `ir::Interp` |
//! | `cached+tlb` | block cache, exits chained | basic block | `jit::Tlb` | `ir::Interp` |
//! | `+extended` | " | a load no longer ends a block | `jit::Tlb` | `ir::Interp` |
//! | `+superblock` | " | direct branches merged, with side exits | `jit::Tlb` | `ir::Interp` |
//! | `+compiled` | " | " | `jit::Tlb`, **inlined** | `jit::x86` |
//!
//! `lift-each` is the honest starting point rather than the interpreter: a
//! translator that re-lifts every block is *slower* than the interpreter it
//! replaces, and the cache is what turns that around. Reporting the cache's
//! win against the interpreter would credit it with the lifter's speed too.
//!
//! `+compiled` only exists where the build has a backend — `jit-x86` is
//! `cfg`-gated to an x86-64 Linux host — and elsewhere it repeats
//! `+superblock`.
//!
//! # The workloads
//!
//! Four A64 programs, written here rather than fetched, because a commercial
//! ROM cannot be committed (CLAUDE.md, "Testing"). Every encoding below was
//! assembled with `llvm-mc -triple=aarch64` and is named in the comment beside
//! it.
//!
//! * **`alu-loop`** — six arithmetic instructions and a backward `B`. One
//!   block, chained to itself, touching no memory: the block cache's best case
//!   and a case the TLB never sees. The back edge is direct, so a trace
//!   unrolls it to the instruction limit.
//! * **`memcpy`** — a byte-at-a-time copy. Two accesses per iteration, and the
//!   **store ends the block under every shape** (`cpu::arm::a64::lift`, "A
//!   store ends the block"), so this is the workload that prices that rule.
//! * **`load-heavy`** — four loads and a branch, the pointers in four
//!   different pages. Under the basic-block shape that is one guest
//!   instruction per block: the TLB's best case and the cache's worst, which
//!   are the same fact, and the row `+extended` was written for.
//! * **`chain`** — four blocks in a cycle, so exits are patched in four
//!   directions and a chained edge is a real edge rather than a self-loop.
//!
//! ```text
//! cargo bench --features jit-x86,cpu-arm-a64-lift --bench a64_dispatch
//! cargo bench --features jit-x86,cpu-arm-a64-lift --bench a64_dispatch -- --insns 20000000
//! cargo bench --features jit-x86,cpu-arm-a64-lift --bench a64_dispatch -- --smoke
//! ```

use std::sync::Arc;
use std::time::{Duration, Instant};

use rsemu::core::error::Result;
use rsemu::core::space::{AddressSpace, MemAttrs, MemResult, RamStore, Region, UnassignedPolicy};
use rsemu::core::value::Width;
use rsemu::cpu::arm::a64::lift::{self, Origin, PC, SLOT_COUNT, Shape, World, x_slot};
use rsemu::cpu::arm::a64::{Config, Cpu};
use rsemu::ir::{AccessKind, Align, InsnStart, IrHost, MemOp, RegSlot};
use rsemu::jit::{
    BlockCache, Context, DirtyPages, Dispatcher, Epoch, FastMem, Frontend, MemPlan, StoreLog, Tlb,
    Translation,
};

/// Where the program is loaded and RAM is mapped. Zero, because the ladder
/// runs bare — `Origin::Bare` makes the guest PC the physical address, and a
/// block's own PC arithmetic then means what it says.
const BASE: u64 = 0;
/// Sixteen pages: one of code, the rest for the copy loop to work in.
const RAM_SIZE: u64 = 16 * 4096;
/// Where the data window starts.
const DATA: u64 = 4096;

fn main() {
    let args = Args::parse(std::env::args().skip(1));
    println!(
        "rsemu a64 dispatch benchmark — {} guest instructions per run, best of {}\n",
        args.insns, args.reps
    );
    ladder(&args);
    println!();
    quantum(&args);
}

// ---------------------------------------------------------------------------
// Table one: the mechanism ladder
// ---------------------------------------------------------------------------

fn ladder(args: &Args) {
    println!(
        "{:<12} {:>11} {:>11} {:>11} {:>11} {:>11} {:>11} {:>11}",
        "workload",
        "interpreter",
        "lift-each",
        "cached",
        "cached+tlb",
        "+extended",
        "+superblock",
        "+compiled",
    );
    for w in workloads() {
        let basic = Shape::BasicBlock;
        let interp = best(args.reps, || run_interpreter(&w, args.insns));
        let cold = best(args.reps, || {
            run_translated(&w, args.insns, false, false, basic, false)
        });
        let cached = best(args.reps, || {
            run_translated(&w, args.insns, true, false, basic, false)
        });
        let tlb = best(args.reps, || {
            run_translated(&w, args.insns, true, true, basic, false)
        });
        let extended = best(args.reps, || {
            run_translated(&w, args.insns, true, true, Shape::Extended, false)
        });
        let trace = best(args.reps, || {
            run_translated(&w, args.insns, true, true, Shape::Trace, false)
        });
        let compiled = best(args.reps, || {
            run_translated(&w, args.insns, true, true, Shape::Trace, true)
        });
        println!(
            "{:<12} {:>11} {:>11} {:>11} {:>11} {:>11} {:>11} {:>11}",
            w.name,
            mips(args.insns, interp),
            mips(args.insns, cold),
            mips(args.insns, cached),
            mips(args.insns, tlb),
            mips(args.insns, extended),
            mips(args.insns, trace),
            mips(args.insns, compiled),
        );
        println!(
            "{:<12} {:>11} {:>11} {:>11} {:>11} {:>11} {:>11} {:>11}",
            "",
            "1.00x",
            ratio(interp, cold),
            ratio(interp, cached),
            ratio(interp, tlb),
            ratio(interp, extended),
            ratio(interp, trace),
            ratio(interp, compiled),
        );
    }
    println!(
        "\nEvery row was checked against the interpreter's final register file \
         before it was timed."
    );
}

// ---------------------------------------------------------------------------
// Table two: the quantum
// ---------------------------------------------------------------------------

/// The scheduler budgets swept, in bus accesses.
///
/// `machines/arm64-virt.machine` runs on **10 000**, which is the column that
/// matters; the rest bracket it so that the shape of the curve is visible.
/// Below `worst_bound`'s 1 088 ticks in bare mode no block could be admitted
/// at all before the guard learned to lift, and the run collapsed to the
/// interpreter — which is what this table exists to make visible.
const BUDGETS: [u64; 6] = [64, 256, 1_024, 4_096, 10_000, 65_536];

fn quantum(args: &Args) {
    println!("the quantum: throughput against `Cpu::run_budget`'s budget\n");
    println!(
        "Millions of *bus accesses* per second — the currency the scheduler and \
         the guard\nboth speak, one per instruction plus one per memory access. \
         Read across a row.\n"
    );
    print!("{:<12} {:>10}", "engine", "workload");
    for budget in BUDGETS {
        print!(" {budget:>11}");
    }
    println!();
    for (name, engine) in [
        ("interp", rsemu::cpu::arm::a64::Engine::Interp),
        ("jit", rsemu::cpu::arm::a64::Engine::Jit),
        ("jit-host", rsemu::cpu::arm::a64::Engine::JitHost),
    ] {
        for w in workloads() {
            print!("{name:<12} {:>10}", w.name);
            for budget in BUDGETS {
                let took = best(args.reps, || run_core(&w, args.insns, engine, budget));
                print!(" {:>11}", mts(args.insns, took));
            }
            println!();
        }
    }
    println!(
        "\nA budget below a block's worst case is interpreted, so the left-hand \
         columns\nare where `admit`'s guard decides the run rather than the \
         code generator."
    );
}

/// Run one workload through a whole core, in quanta of `budget` ticks.
///
/// Ticks rather than instructions, because that is the currency the scheduler
/// and the guard both speak; the loop stops on the instruction count so that
/// every cell of the table does the same guest work.
fn run_core(
    w: &Workload,
    insns: u64,
    engine: rsemu::cpu::arm::a64::Engine,
    budget: u64,
) -> Duration {
    let (space, _ram) = machine(w);
    let cpu = seeded_cpu(w, space, engine);
    // Exactly `insns` ticks, however they are cut into quanta — `run_budget`
    // consumes precisely what it is given and carries any overrun as debt, so
    // every cell of a row does the same guest work and the only variable is
    // the quantum. A loop that ran `insns / budget` *whole* quanta would give
    // the widest budget three times the work and read as three times slower.
    let start = Instant::now();
    let mut left = insns;
    while left > 0 {
        let ticks = budget.min(left);
        cpu.run_budget(ticks);
        left -= ticks;
    }
    let took = start.elapsed();
    assert_eq!(
        cpu.sysregs().esr_el1,
        0,
        "{}: the guest trapped under {engine:?}",
        w.name
    );
    took
}

// ---------------------------------------------------------------------------
// Reporting
// ---------------------------------------------------------------------------

/// Millions of guest instructions per second, as a string.
fn mips(insns: u64, took: Duration) -> String {
    let secs = took.as_secs_f64();
    if secs <= 0.0 {
        return "-".into();
    }
    format!("{:.1} Mi/s", insns as f64 / secs / 1e6)
}

/// Millions of guest bus accesses per second, as a string.
fn mts(ticks: u64, took: Duration) -> String {
    let secs = took.as_secs_f64();
    if secs <= 0.0 {
        return "-".into();
    }
    format!("{:.1} Mt/s", ticks as f64 / secs / 1e6)
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
    regs: [u64; 31],
}

fn workloads() -> Vec<Workload> {
    // x1 = source, x2 = destination, x20/x21 the constants the loop adds back.
    let mut memcpy_regs = [0u64; 31];
    memcpy_regs[1] = BASE + DATA;
    memcpy_regs[2] = BASE + DATA + 2048;
    memcpy_regs[20] = BASE + DATA;
    memcpy_regs[21] = BASE + DATA + 2048;
    // Four pointers in four different pages, so the TLB answers about several
    // entries rather than one.
    let mut load_regs = [0u64; 31];
    load_regs[1] = BASE + DATA + 0x11;
    load_regs[2] = BASE + DATA + 0x1000;
    load_regs[3] = BASE + DATA + 0x2000;
    load_regs[4] = BASE + DATA + 0x3000;
    vec![
        Workload {
            name: "alu-loop",
            program: vec![
                0x9100_054a, // add  x10, x10, #1
                0x9100_0d4b, // add  x11, x10, #3
                0x8b0a_016c, // add  x12, x11, x10
                0xd37f_f98d, // lsl  x13, x12, #1
                0xca0b_01ae, // eor  x14, x13, x11
                0xcb0a_01cf, // sub  x15, x14, x10
                0x17ff_fffa, // b    .-24
            ],
            regs: [0; 31],
        },
        Workload {
            // A byte copy that wraps its pointers back inside the window, so
            // it runs forever without leaving RAM.
            name: "memcpy",
            program: vec![
                0x3940_0025, // ldrb w5, [x1]
                0x3900_0045, // strb w5, [x2]
                0x9100_0421, // add  x1, x1, #1
                0x9100_0442, // add  x2, x2, #1
                0x9200_1c21, // and  x1, x1, #0x7ff
                0x9200_1c42, // and  x2, x2, #0x7ff
                0x8b14_0021, // add  x1, x1, x20
                0x8b15_0042, // add  x2, x2, x21
                0x17ff_fff8, // b    .-32
            ],
            regs: memcpy_regs,
        },
        Workload {
            name: "load-heavy",
            program: vec![
                0x3940_0025, // ldrb w5, [x1]
                0x3940_0046, // ldrb w6, [x2]
                0x3940_0067, // ldrb w7, [x3]
                0x3940_0088, // ldrb w8, [x4]
                0x17ff_fffc, // b    .-16
            ],
            regs: load_regs,
        },
        Workload {
            // Four blocks in a cycle: each `B` ends a block under the two
            // non-merging shapes, so the exits are patched four ways.
            name: "chain",
            program: vec![
                0x9100_054a, // add  x10, x10, #1
                0x1400_0002, // b    .+8
                0x9100_056b, // add  x11, x11, #1   ; never reached
                0x9100_058c, // add  x12, x12, #1
                0x1400_0002, // b    .+8
                0x9100_05ad, // add  x13, x13, #1   ; never reached
                0x9100_05ce, // add  x14, x14, #1
                0x17ff_fffa, // b    .-24
            ],
            regs: [0; 31],
        },
    ]
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
    // Something for the copy loop to move.
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

fn config() -> Config {
    Config::cortex_a53().with_reset_vector(BASE)
}

fn seeded_cpu(w: &Workload, space: Arc<AddressSpace>, engine: rsemu::cpu::arm::a64::Engine) -> Cpu {
    let cpu = Cpu::new(config()).with_engine(engine);
    cpu.attach_space(space);
    for (n, value) in w.regs.iter().enumerate().skip(1) {
        cpu.set_x(n as u32, *value);
    }
    cpu
}

fn run_interpreter(w: &Workload, insns: u64) -> Duration {
    let (space, _ram) = machine(w);
    let cpu = seeded_cpu(w, space, rsemu::cpu::arm::a64::Engine::Interp);
    let start = Instant::now();
    for _ in 0..insns {
        cpu.step();
    }
    let took = start.elapsed();
    assert_eq!(cpu.sysregs().esr_el1, 0, "{}: the guest trapped", w.name);
    took
}

/// The translated path, with the cache, the TLB, the shape and the code
/// generator switched independently.
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
    compiled: bool,
) -> Duration {
    let (space, _ram) = machine(w);
    let mut front = Lifter::new(Arc::clone(&space), shape);
    let mut host = BenchHost::new(w, space, tlb);
    // "No cache" is one block per call plus a flush between, which also
    // removes chaining — a translator that re-lifts has no predecessor to
    // patch. Its cache is sized to one block so the flush is not what is timed.
    let mut disp = Dispatcher::with_cache(BlockCache::with_capacity(if cache { 1024 } else { 1 }));
    if compiled {
        disp = with_backend(disp);
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
            // did thirteen times the work in the same time is not a slow run.
            let per_block = (run.insns as u64 / run.blocks as u64).max(1);
            budget = usize::try_from((insns.saturating_sub(done) / per_block) + 1)
                .unwrap_or(BLOCK_BUDGET)
                .clamp(1, BLOCK_BUDGET);
        }
    }
    let took = start.elapsed();
    if std::env::var("RSEMU_BENCH_STATS").is_ok() {
        eprintln!("{} {shape:?}: {done} insns, {:?}", w.name, disp.stats());
    }

    // The check. `done` is at least `insns`, so the oracle runs the same
    // number the translated path actually retired.
    let (oracle_space, _oracle_ram) = machine(w);
    let cpu = seeded_cpu(w, oracle_space, rsemu::cpu::arm::a64::Engine::Interp);
    for _ in 0..done {
        cpu.step();
    }
    for n in 0..31u32 {
        assert_eq!(
            cpu.x(n),
            host.slots[x_slot(n).0 as usize],
            "{}: x{n} disagrees after {done} instructions under {shape:?}",
            w.name
        );
    }
    // Scaled to the instruction target, because a configuration cannot always
    // stop exactly on it: what is reported is time *per guest instruction*.
    Duration::from_secs_f64(took.as_secs_f64() * insns as f64 / done as f64)
}

/// Attach the host code generator, where this build has one.
#[cfg(all(feature = "jit-x86", target_os = "linux", target_arch = "x86_64"))]
fn with_backend(disp: Dispatcher) -> Dispatcher {
    match rsemu::jit::x86::Engine::new() {
        Some(engine) => disp.with_backend(engine),
        None => disp,
    }
}

#[cfg(not(all(feature = "jit-x86", target_os = "linux", target_arch = "x86_64")))]
fn with_backend(disp: Dispatcher) -> Dispatcher {
    disp
}

/// The largest number of blocks one dispatcher call is asked for.
const BLOCK_BUDGET: usize = 4096;

// ---------------------------------------------------------------------------
// The frontend and the host
// ---------------------------------------------------------------------------

struct Lifter {
    world: World,
    shape: Shape,
    space: Arc<AddressSpace>,
}

impl Lifter {
    fn new(space: Arc<AddressSpace>, shape: Shape) -> Lifter {
        Lifter {
            world: World {
                features: config().features,
                origin: Origin::Bare,
                strict_align: false,
            },
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
        lift::key(&self.world, self.shape)
    }
    fn pc_slot(&self) -> RegSlot {
        PC
    }
    fn translate(&mut self, pc: u64) -> Result<Translation> {
        let space = Arc::clone(&self.space);
        let mut src = |addr: u64| {
            space
                .read(addr, Width::U32, MemAttrs::DEFAULT)
                .ok()
                .map(|v| v as u32)
        };
        let lifted = lift::lift(&self.world, pc, &mut src, lift::MAX_INSNS, self.shape)?;
        Ok(Translation {
            page: pc & !rsemu::jit::PAGE_MASK,
            insns: lifted.insns,
            block: lifted.block,
        })
    }
}

/// EL1 with translation off, which is the world the ladder lifts in.
const MACHINE: Context = Context {
    level: 1,
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
            slots[x_slot(n as u32).0 as usize] = *value;
        }
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

/// The seam the compiled column measures: the backend inlines an aligned
/// access into a mask, a compare, an add and a `mov`, out of *this* host's own
/// TLB.
///
/// `note_fast_store` is not free in the same way a charge would be: the dirty
/// log is what the dispatcher drains to invalidate a rewritten page, so a
/// benchmark that skipped it would be measuring a machine that cannot run
/// self-modifying code — and on A64 that is not hypothetical, because the
/// `memcpy` workload's store ends its own block.
impl FastMem for BenchHost {
    fn load_plan(&mut self) -> Option<MemPlan> {
        self.tlb
            .as_ref()
            .map(|tlb| tlb.plan(AccessKind::Load, MACHINE))
    }

    fn store_plan(&mut self) -> Option<MemPlan> {
        self.tlb
            .as_ref()
            .map(|tlb| tlb.plan(AccessKind::Store, MACHINE))
    }

    fn note_fast_store(&mut self, addr: u64, bytes: u64) {
        if let Some(tlb) = self.tlb.as_mut() {
            tlb.note_fast_store(addr, bytes);
        }
        self.dirty.note(addr, bytes);
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
