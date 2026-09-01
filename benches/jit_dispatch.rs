//! What the software TLB and the block cache actually bought, measured.
//!
//! `ROADMAP.md` §13's phase-8 gate is a wall-clock number, and this is nowhere
//! near it — there is no host code generator yet, so every configuration below
//! is still interpreting. What this measures is the thing that *is* finished:
//! the three mechanisms §9.1 puts ahead of code generation, each switched on
//! one at a time so the number attaches to a mechanism rather than to a
//! release.
//!
//! # The four configurations
//!
//! | | translation | memory |
//! | --- | --- | --- |
//! | `interpreter` | none — `Hart::step`, the oracle | `AddressSpace` |
//! | `lift-every-time` | re-lift every block, no cache | `AddressSpace` |
//! | `cached` | block cache, exits chained | `AddressSpace` |
//! | `cached+tlb` | block cache, exits chained | `jit::Tlb` |
//!
//! The rows are cumulative on purpose: `lift-every-time` is the honest
//! starting point for a translator, because a translator that re-lifts is
//! slower than the interpreter it replaces, and the cache is what turns that
//! around. Reporting the cache's win against the interpreter instead would
//! credit it with the lifter's speed as well.
//!
//! # The workloads
//!
//! Three RISC-V programs, all inside the lifted RV64I subset, all written here
//! rather than fetched, because a commercial ROM cannot be committed
//! (CLAUDE.md, "Testing"):
//!
//! * **`alu-loop`** — a tight arithmetic loop, one block, chained to itself.
//!   The best case for the block cache and a case the TLB never sees, since it
//!   touches no memory.
//! * **`memcpy`** — a byte-at-a-time copy loop: a load, a store, six
//!   arithmetic instructions and a branch. Every iteration crosses the memory
//!   path twice.
//! * **`load-heavy`** — four loads and a jump. The lifter ends a block at
//!   every access (`cpu::riscv::lift`, "Ticks"), so this is one guest
//!   instruction per block and the highest ratio of memory work to everything
//!   else the subset can express. It is the TLB's best case *and* the block
//!   cache's worst, and the two facts are the same fact.
//! * **`chain`** — four blocks in a cycle, so the exits are patched in four
//!   directions and a lookup is a lookup rather than a self-loop.
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
//! **`cargo bench` only.** The bench profile is `-O` with debug assertions
//! off; the same code under `cargo test` is several times slower and the
//! number means nothing. No `criterion` — the dependency policy is absolute
//! and an `Instant` loop is all this needs.
//!
//! ```text
//! cargo bench --features jit,cpu-riscv-lift --bench jit_dispatch
//! cargo bench --features jit,cpu-riscv-lift --bench jit_dispatch -- --insns 20000000
//! ```

use std::sync::Arc;
use std::time::{Duration, Instant};

use rsemu::core::error::Result;
use rsemu::core::space::{AddressSpace, MemAttrs, MemResult, RamStore, Region, UnassignedPolicy};
use rsemu::core::value::Width;
use rsemu::cpu::riscv::lift::{self, Origin, PC, SLOT_COUNT, x_slot};
use rsemu::cpu::riscv::{Config, Hart};
use rsemu::ir::{AccessKind, Align, InsnStart, IrHost, MemOp, RegSlot};
use rsemu::jit::{
    BlockCache, Context, DirtyPages, Dispatcher, Epoch, Frontend, StoreLog, Tlb, Translation,
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
        "{:<12} {:>14} {:>14} {:>14} {:>14}",
        "workload", "interpreter", "lift-each-time", "cached", "cached+tlb"
    );
    for w in workloads() {
        let interp = best(args.reps, || run_interpreter(&w, args.insns));
        let cold = best(args.reps, || run_translated(&w, args.insns, false, false));
        let cached = best(args.reps, || run_translated(&w, args.insns, true, false));
        let tlb = best(args.reps, || run_translated(&w, args.insns, true, true));
        println!(
            "{:<12} {:>14} {:>14} {:>14} {:>14}",
            w.name,
            mips(args.insns, interp),
            mips(args.insns, cold),
            mips(args.insns, cached),
            mips(args.insns, tlb),
        );
        println!(
            "{:<12} {:>14} {:>14} {:>14} {:>14}",
            "",
            "1.00x",
            ratio(interp, cold),
            ratio(interp, cached),
            ratio(interp, tlb),
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

/// The translated path, with the cache and the TLB switched independently.
///
/// Checked against the interpreter before it is timed: the two must reach the
/// same register file after the same number of guest instructions, or the
/// number below is measuring a guest that stopped working.
fn run_translated(w: &Workload, insns: u64, cache: bool, tlb: bool) -> Duration {
    let (space, _ram) = machine(w);
    let mut front = Lifter::new(Arc::clone(&space));
    let mut host = BenchHost::new(w, space, tlb);
    // "No cache" is one block per call plus a flush between, which also
    // removes chaining — a translator that re-lifts has no predecessor to
    // patch. Running many blocks per call and flushing afterwards would leave
    // the cache live inside the call and measure something else. Its cache is
    // sized to one block so that the flush itself is not what is being timed.
    let mut disp = Dispatcher::with_cache(BlockCache::with_capacity(if cache { 1024 } else { 1 }));
    let budget = if cache { 4096 } else { 1 };

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
        }
    }
    let took = start.elapsed();

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
            "{}: x{n} disagrees after {done} instructions",
            w.name
        );
    }
    took
}

// ---------------------------------------------------------------------------
// The frontend and the host
// ---------------------------------------------------------------------------

struct Lifter {
    cfg: Config,
    space: Arc<AddressSpace>,
}

impl Lifter {
    fn new(space: Arc<AddressSpace>) -> Lifter {
        Lifter {
            cfg: Config::rv64i(),
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
        lift::key(&self.cfg, Origin::Bare)
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
        let lifted = lift::lift(&self.cfg, Origin::Bare, pc, &mut src, lift::MAX_INSNS)?;
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
