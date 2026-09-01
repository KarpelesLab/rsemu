//! The gdbstub on a machine with **two CPUs**, in both threading modes.
//!
//! `tests/gdb_session.rs` and `tests/gdb_real_client.rs` both debug a machine
//! with exactly one core, which is the case where every "which CPU?" question
//! has the same answer whatever you do. This file uses
//! `machines/tests/heterogeneous.machine` — a RISC-V hart and a 6502, two
//! address spaces, different widths and different endianness, one shared RAM
//! region — because on that board getting the CPU wrong is *visible*: the same
//! byte has two addresses, and an address that means something on one side
//! means nothing on the other.
//!
//! What it asserts:
//!
//! * the two cores are two GDB threads, each with its own register map, its own
//!   program counter and its own address space;
//! * a **watchpoint is polled through the CPU that set it**, not through CPU 0.
//!   That was a defect: `Z2` on a 6502 address was read through the hart's
//!   space, where nothing is mapped, so the shadow was open-bus forever and the
//!   watchpoint never fired — silently, which is the worst way for a debugger
//!   to be wrong;
//! * and the whole thing works under [`ThreadingMode::Parallel`], where the two
//!   cores really do run on host threads of their own. The stub never has to
//!   stop the world by hand, and this test is why that claim is checked rather
//!   than asserted: virtual time only advances inside `resume`/`step`, and the
//!   round those call joins every worker before it returns (`ROADMAP.md` §4.7).
//!
//! [`ThreadingMode::Parallel`]: rsemu::core::sched::ThreadingMode::Parallel

#![cfg(all(feature = "gdb", feature = "cpu-riscv", feature = "cpu-mos6502"))]

use rsemu::core::sched::ThreadingMode;
use rsemu::host::gdb::{DebugTarget, MachineTarget, StopKind};
use rsemu::machine::{Machine, catalog};

const HETEROGENEOUS: &str = include_str!("../machines/tests/heterogeneous.machine");

/// The hart's program: bump a counter in the shared region for ever.
///
/// Assembled by hand from *The RISC-V Instruction Set Manual, Volume I:
/// Unprivileged ISA*, chapter 2.
///
/// ```text
///   1000  lui  t0, 0x100     # t0 = 0x00100000, the shared region
///   1004  addi t1, t1, 1     # loop:
///   1008  sb   t1, 0x200(t0)
///   100c  j    loop
/// ```
const RV_CODE: &[u32] = &[
    0x0010_02b7, // lui  t0, 0x100
    0x0013_0313, // addi t1, t1, 1
    0x2062_8023, // sb   t1, 0x200(t0)
    0xff9f_f06f, // j    -8
];

/// The 6502's program: bump a byte in *its* window on the same region.
///
/// Assembled by hand from the *MCS6500 Family Programming Manual*'s opcode
/// table. `$4000` is shared byte 0; the hart's counter is at shared byte
/// `0x200`, so the two never collide.
///
/// ```text
///   e000  inc $4000        ; loop:
///   e003  jmp loop
/// ```
const MOS_CODE: &[u8] = &[0xee, 0x00, 0x40, 0x4c, 0x00, 0xe0];

/// Where the 6502 fetches its reset vector, as an offset into its 8 KiB ROM.
const RESET_VECTOR: usize = 0x1ffc;

/// The 6502 address of shared byte 0, and the hart address of the same byte.
const MOS_SHARED: u64 = 0x4000;
const RV_SHARED: u64 = 0x0010_0000;

fn mos_rom() -> Vec<u8> {
    let mut rom = vec![0u8; 8 * 1024];
    rom[..MOS_CODE.len()].copy_from_slice(MOS_CODE);
    rom[RESET_VECTOR] = 0x00;
    rom[RESET_VECTOR + 1] = 0xe0;
    rom
}

fn rv_rom() -> Vec<u8> {
    let mut rom = vec![0u8; 4 * 1024];
    for (i, word) in RV_CODE.iter().enumerate() {
        rom[i * 4..i * 4 + 4].copy_from_slice(&word.to_le_bytes());
    }
    rom
}

/// Build the fixture in `mode`.
fn board(mode: ThreadingMode) -> Machine {
    let mut options = catalog::build_options().expect("the catalog agrees with itself");
    options.realize.scheduler.mode = mode;
    options.realize.scheduler.workers = 2;
    options.realize.media.insert("rvcode", rv_rom());
    options.realize.media.insert("moscode", mos_rom());
    let registry = catalog::registry().expect("a registry");
    match rsemu::machine::build("heterogeneous.machine", HETEROGENEOUS, &registry, &options) {
        Ok(m) => m,
        Err(e) => panic!("the fixture does not realize: {e}"),
    }
}

/// Which of the target's CPUs is the 6502, by class name.
fn cpu_named(target: &MachineTarget<'_>, class: &str) -> usize {
    (0..target.cpu_count())
        .find(|i| target.arch(*i).expect("an arch").class.name == class)
        .unwrap_or_else(|| panic!("no {class} among the target's CPUs"))
}

#[test]
fn two_cores_are_two_threads_with_their_own_spaces() {
    let mut m = board(ThreadingMode::Deterministic);
    let mut target = MachineTarget::new(&mut m);
    assert_eq!(target.cpu_count(), 2, "a hart and a 6502");

    let mos = cpu_named(&target, "cpu.mos6502");
    let hart = cpu_named(&target, "cpu.riscv");
    assert_ne!(mos, hart);

    // Each thread has its own register file, and they are not the same shape.
    let mos_regs = target.read_registers(mos).expect("the 6502's registers");
    let hart_regs = target.read_registers(hart).expect("the hart's registers");
    assert_eq!(mos_regs.len(), 7, "a x y sp p pc");
    assert_ne!(mos_regs.len(), hart_regs.len());

    // The shared byte has two addresses. Writing it through one CPU's space and
    // reading it through the other's is the whole point of this fixture, and it
    // only works if each CPU's accesses go through its *own* space.
    target
        .write_memory(mos, MOS_SHARED, &[0xa5])
        .expect("the 6502 window is writable");
    let mut seen = [0u8; 1];
    target
        .read_memory(hart, RV_SHARED, &mut seen)
        .expect("the hart sees the same region");
    assert_eq!(seen[0], 0xa5, "the two CPUs did not reach the same byte");

    // And the 6502's address means nothing in the hart's space: `0x4000` is
    // unmapped there. `open-bus` rather than a fault, which is exactly why
    // reading a watchpoint through the wrong CPU used to fail silently.
    let mut elsewhere = [0u8; 1];
    target
        .read_memory(hart, MOS_SHARED, &mut elsewhere)
        .expect("open-bus, not a fault");
    assert_ne!(
        elsewhere[0], 0xa5,
        "the 6502's address must not resolve in the hart's space"
    );
}

#[test]
fn a_watchpoint_is_polled_through_the_cpu_that_set_it() {
    let mut m = board(ThreadingMode::Deterministic);
    let mut target = MachineTarget::new(&mut m);
    let mos = cpu_named(&target, "cpu.mos6502");

    // A watchpoint on an address that only exists in the 6502's space. Before
    // this was per-CPU it was read through CPU 0 — the hart — where `0x4000` is
    // open-bus, so the shadow never changed and the watchpoint never fired.
    target
        .add_watchpoint(mos, MOS_SHARED, 1)
        .expect("a write watchpoint on the 6502's window");
    target.begin_resume();

    let mut hit = None;
    for _ in 0..200 {
        if let Some(stop) = target.resume().expect("the machine runs") {
            hit = Some(stop);
            break;
        }
    }
    let stop = hit.expect("the 6502's `inc $4000` must trip the watchpoint");
    assert_eq!(
        stop.kind,
        StopKind::Watchpoint { addr: MOS_SHARED },
        "the stop reply names the watched address"
    );
    assert_eq!(
        stop.cpu, mos,
        "the stop reply must name the CPU whose watchpoint it is, not CPU 0"
    );
}

#[test]
fn a_debugger_drives_a_parallel_machine() {
    // `parallel` is the mode where the two cores are on host threads of their
    // own. The stub does not stop the world by hand and does not need to: time
    // only moves inside `resume`/`step`, and the scheduling round they drive
    // joins every worker before it returns (`ROADMAP.md` §4.7).
    let mut m = board(ThreadingMode::Parallel);
    assert_eq!(m.threading_mode(), ThreadingMode::Parallel);
    let mut target = MachineTarget::new(&mut m);
    let mos = cpu_named(&target, "cpu.mos6502");
    let hart = cpu_named(&target, "cpu.riscv");

    // Registers are readable with the world stopped, on both threads.
    let before = target.read_registers(hart).expect("the hart's registers");
    assert!(!before.is_empty());

    // One step really is one instruction, on a machine whose other core is
    // running on another thread.
    let first = target.step(mos).expect("a step");
    assert_eq!(first.cpu, mos);
    for _ in 0..8 {
        target.step(mos).expect("a step");
    }

    // A breakpoint on the 6502's loop, hit while the hart runs beside it.
    target.add_breakpoint(0xe003, false).expect("arm");
    target.begin_resume();
    let mut hit = None;
    for _ in 0..200 {
        if let Some(stop) = target.resume().expect("the machine runs") {
            hit = Some(stop);
            break;
        }
    }
    let stop = hit.expect("the 6502 reaches its `jmp` under the parallel scheduler");
    assert_eq!(stop.kind, StopKind::Breakpoint { hardware: false });
    assert_eq!(stop.cpu, mos);

    // And the hart has been running all along: its own program counter moved.
    let after = target.read_registers(hart).expect("the hart's registers");
    assert_ne!(before, after, "the other core never ran");
}

#[test]
fn monitor_commands_answer_for_the_thread_they_were_typed_on() {
    // `monitor x` reads guest memory *as that CPU sees it*, which on this board
    // is two different address spaces. A monitor that answered for CPU 0
    // whatever thread the user had selected would be showing them somebody
    // else's memory and saying nothing about it.
    let mut m = board(ThreadingMode::Deterministic);
    let mut target = MachineTarget::new(&mut m);
    let mos = cpu_named(&target, "cpu.mos6502");
    let hart = cpu_named(&target, "cpu.riscv");

    target
        .write_memory(mos, MOS_SHARED, &[0xde, 0xad])
        .expect("the 6502 window is writable");

    let seen = target
        .monitor(mos, &format!("x {MOS_SHARED:x} 2"))
        .expect("`x` is a command");
    assert!(seen.contains("de ad"), "{seen}");

    // The same number, on the other thread, is a different place — and it is
    // not the same bytes.
    let elsewhere = target
        .monitor(hart, &format!("x {MOS_SHARED:x} 2"))
        .expect("`x` is a command");
    assert!(!elsewhere.contains("de ad"), "{elsewhere}");

    // The hart's own address for that byte finds it again.
    let through_hart = target
        .monitor(hart, &format!("x {RV_SHARED:x} 2"))
        .expect("`x` is a command");
    assert!(through_hart.contains("de ad"), "{through_hart}");

    // `xp` skips translation; with no MMU turned on here it agrees with `x`,
    // which is the case a user needs to be able to see rather than assume.
    let physical = target
        .monitor(hart, &format!("xp {RV_SHARED:x} 2"))
        .expect("`xp` is a command");
    assert!(physical.contains("de ad"), "{physical}");
    assert!(
        target
            .monitor(hart, &format!("translate {RV_SHARED:x}"))
            .expect("`translate` is a command")
            .contains("identity"),
        "an RV64 hart with satp at zero translates nothing"
    );

    // The map is the CPU's own space unless a name says otherwise.
    let map = target.monitor(mos, "map").expect("`map` is a command");
    assert!(map.starts_with("big "), "{map}");
    let named = target.monitor(mos, "map little").expect("`map` by name");
    assert!(named.starts_with("little "), "{named}");
    assert!(
        target
            .monitor(mos, "map nosuchspace")
            .expect("an answer, not silence")
            .contains("no address space named")
    );

    // An unknown command is `None`, which is the protocol's "no such command"
    // and is what lets GDB fall back rather than print an empty line.
    assert!(target.monitor(mos, "frobnicate").is_none());
    assert!(target.monitor(mos, "help").expect("help").contains("xp "));
}
