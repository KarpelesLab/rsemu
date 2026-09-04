//! The debugger against a **translating** execution engine.
//!
//! `engine = "jit"` and `engine = "jit-host"` are selectable from the command
//! line on four boards now, and both put something between the guest's memory
//! and what actually executes: a cache of blocks lifted from guest code, and a
//! run loop that goes round several instructions at a time. Neither is visible
//! to `tests/gdb_session.rs` or `tests/gdb_real_client.rs`, which debug an
//! interpreted 6502 and an interpreted 8086. So the two questions a debugger
//! has to answer about a JIT are asked here, on the board that has had one the
//! longest.
//!
//! # Does a breakpoint inside a compiled block fire?
//!
//! Yes, and by construction rather than by luck. With a breakpoint armed
//! `MachineTarget::resume` advances **one clock tick at a time**, and a tick
//! is a budget of one bus access; a core's `advance` refuses to run a block
//! whose worst case does not fit what is left of the budget and interprets the
//! instruction instead. So an armed breakpoint degrades a translating core to
//! one interpreted instruction per tick for as long as it is armed, and the
//! program counter is compared after every one. The first test below holds
//! that for a breakpoint on a block's entry, in its middle and on its last
//! instruction, on all three engines.
//!
//! That is also why it is not free: a breakpoint-checking slice costs about
//! the same on every engine, because while it is checking there is no engine.
//! `src/host/gdb/target.rs` says so in as many words; this file is where the
//! claim is measured.
//!
//! # Does a debugger's write into guest code take effect?
//!
//! It did not, and that was a real defect — `gdb`'s `restore`, `set
//! *(int *) $pc = …` and every other way of patching a guest were being run
//! past by stale compiled blocks. A guest store into a page a block was lifted
//! from invalidates it; a *debugger* store goes straight at the address space
//! and the core never sees it. Measured on the loop below, one patched
//! instruction was executed forty times in twenty-five thousand iterations —
//! the interpreter's share of the run — and ignored the rest.
//!
//! `MachineTarget::invalidate_translations` is the fix and its docs are the
//! long form. The second test is the measurement, and it is quantitative on
//! purpose: the obvious observable, "did the guest eventually notice", passes
//! even when the stale block runs thousands of times, because the run
//! eventually drops into the interpreter and converges. Counting is what tells
//! the two apart.
//!
//! **`cpu.arm.a64` is not fixed by this**, and that is why it is not tested
//! here: its `Device::load` flushes its TLB but not its block cache, so the
//! seam this file's fix goes through does not reach it. `cpu.riscv` and
//! `cpu.x86` both flush. See `MachineTarget::invalidate_translations`.

#![cfg(all(
    feature = "gdb",
    feature = "machine-riscv-virt",
    feature = "cpu-riscv-lift",
    feature = "jit"
))]

use rsemu::host::gdb::{DebugTarget, MachineTarget, StopKind};
use rsemu::machine::{Machine, catalog};

/// An RV64I loop with a back edge, loaded at `0x8000_0000`.
///
/// ```text
///   0x80000000  auipc t5, 0          t5 = 0x80000000
///   0x80000004  addi  t5, t5, 20     t5 = loop
///   0x80000008  addi  t2, x0, 0
///   0x8000000c  addi  t0, x0, 0
///   0x80000010  addi  t1, x0, 1
///   0x80000014  loop: add t0, t0, t1
///   0x80000018  slli  t2, t0, 3
///   0x8000001c  srli  t2, t2, 3
///   0x80000020  add   t0, t0, t2
///   0x80000024  jalr  x0, 0(t5)
/// ```
///
/// Every instruction is inside the lifted subset except the closing `jalr`,
/// which is what `tests/riscv_virt_engines.rs` boots this board with and for
/// the same reason: the JIT really does translate, cache, chain and re-serve
/// blocks here. There is no store, so nothing the *guest* does can invalidate
/// a block — which is what makes the second test about the debugger and
/// nothing else.
///
/// Source: *The RISC-V Instruction Set Manual, Volume I*, chapter 2.
const PROGRAM: [u32; 10] = [
    0x0000_0f17, // auipc t5, 0
    0x014f_0f13, // addi  t5, t5, 20
    0x0000_0393, // addi  t2, x0, 0
    0x0000_0293, // addi  t0, x0, 0
    0x0010_0313, // addi  t1, x0, 1
    0x0062_82b3, // loop: add t0, t0, t1
    0x0033_9393, // slli  t2, t0, 3
    0x0033_d393, // srli  t2, t2, 3
    0x0072_82b3, // add   t0, t0, t2
    0x000f_0067, // jalr  x0, 0(t5)
];

/// Where the loop's first instruction is: a block entry, and the branch
/// target.
const LOOP_TOP: u64 = 0x8000_0014;
/// The `slli` — one instruction into the block, so a stub that only ever looks
/// at block boundaries cannot see it.
const MID_BLOCK: u64 = 0x8000_0018;
/// The `add` that closes the arithmetic, one before the back edge.
const LAST_INSN: u64 = 0x8000_0020;

/// `x5`, the loop counter: it goes up by one per iteration.
const T0: usize = 5;
/// `x28`, which the loop never touches. [`COUNTER`] is what makes it move.
const T3: usize = 28;
/// `addi t3, t3, 1` — the instruction the debugger patches in.
const COUNTER: u32 = 0x001e_0e13;

/// Build `riscv-virt` on `engine`, with [`PROGRAM`] as its firmware.
///
/// 16 MiB of DRAM rather than the board's 128, and a console and power signal
/// named after the run, because several of these live in one process.
fn board(engine: &str, tag: &str) -> Machine {
    let firmware: Vec<u8> = PROGRAM.iter().flat_map(|w| w.to_le_bytes()).collect();
    let entry = catalog::machine("riscv-virt").expect("this build ships riscv-virt");
    let mut options = catalog::build_options().expect("the catalog agrees with itself");
    options
        .realize
        .media
        .insert("firmware", firmware.as_slice());
    for slot in ["flash0", "flash1", "disk", "initrd"] {
        options.realize.media.insert(slot, &[][..]);
    }
    for (name, value) in [
        (String::from("ram"), String::from("16M")),
        (String::from("engine"), String::from(engine)),
        (String::from("console"), format!("gdb.engines.{tag}")),
        (String::from("power"), format!("gdb.engines.{tag}")),
    ] {
        options.resolve.params.push((name, value));
    }
    let registry = catalog::registry().expect("a registry");
    rsemu::machine::build(entry.name, entry.source, &registry, &options)
        .unwrap_or_else(|e| panic!("riscv-virt does not build with engine={engine}: {e}"))
}

/// One `x` register, out of the `g` packet the debugger would send.
///
/// Read through the debugger rather than through the core, because the
/// debugger's own view is the thing under test.
fn reg(target: &MachineTarget<'_>, x: usize) -> u64 {
    let regs = target.read_registers(0).expect("the register file");
    u64::from_le_bytes(
        <[u8; 8]>::try_from(&regs[x * 8..x * 8 + 8]).expect("an eight-byte register"),
    )
}

/// The program counter, which is register 32 in this core's map.
fn pc(target: &MachineTarget<'_>) -> u64 {
    let regs = target.read_registers(0).expect("the register file");
    u64::from_le_bytes(<[u8; 8]>::try_from(&regs[256..264]).expect("eight bytes"))
}

#[test]
fn a_breakpoint_inside_a_compiled_block_is_not_missed() {
    for engine in ["interp", "jit", "jit-host"] {
        for addr in [LOOP_TOP, MID_BLOCK, LAST_INSN] {
            let mut machine = board(engine, &format!("bp.{engine}.{addr:x}"));
            let mut target = MachineTarget::new(&mut machine);
            target.add_breakpoint(addr, false).expect("Z0");
            target.begin_resume();
            let mut stop = None;
            for _ in 0..64 {
                if let Some(hit) = target.resume().expect("the machine advances") {
                    stop = Some(hit);
                    break;
                }
            }
            let stop = stop.unwrap_or_else(|| {
                panic!(
                    "engine={engine}: a breakpoint at {addr:#x} was never reported, and the \
                     guest is a loop that runs through it — the compiled block ran past it"
                )
            });
            assert_eq!(
                stop.kind,
                StopKind::Breakpoint { hardware: false },
                "engine={engine}, breakpoint at {addr:#x}"
            );
            assert_eq!(
                pc(&target),
                addr,
                "engine={engine}: stopped, but not on the breakpoint's own instruction"
            );
        }
    }
}

#[test]
fn a_debugger_patch_over_a_compiled_block_is_executed() {
    for engine in ["interp", "jit", "jit-host"] {
        let mut machine = board(engine, &format!("smc.{engine}"));
        let mut target = MachineTarget::new(&mut machine);
        // Warm the cache: nothing is armed, so this runs flat out on whichever
        // engine the board was built with.
        for _ in 0..2 {
            target.resume().expect("the machine advances");
        }
        assert!(
            reg(&target, T0) > 0,
            "engine={engine}: the guest never went round the loop"
        );

        target
            .write_memory(0, MID_BLOCK, &COUNTER.to_le_bytes())
            .expect("the debugger writes code");
        let mut back = [0u8; 4];
        target
            .read_memory(0, MID_BLOCK, &mut back)
            .expect("and reads it back");
        assert_eq!(
            u32::from_le_bytes(back),
            COUNTER,
            "engine={engine}: the write did not even reach memory"
        );

        let t0 = reg(&target, T0);
        let t3 = reg(&target, T3);
        for _ in 0..2 {
            target.resume().expect("the machine advances");
        }
        let iterations = reg(&target, T0) - t0;
        let executions = reg(&target, T3) - t3;
        assert!(iterations > 1000, "engine={engine}: the guest barely moved");
        // One apart at most: the run can stop mid-block, having gone round the
        // loop once more than it has reached the patched instruction.
        assert!(
            iterations.abs_diff(executions) <= 1,
            "engine={engine}: the guest went round the loop {iterations} times but executed \
             the instruction the debugger wrote {executions} times. A compiled block lifted \
             before the write is still being served — see \
             `MachineTarget::invalidate_translations`"
        );
    }
}
