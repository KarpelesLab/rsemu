//! Debugging an M-profile core, on the three things that are M-profile.
//!
//! `tests/gdb_cores.rs` holds the register *map* against the core — every
//! offset, every name, every width. What it cannot hold is behaviour, and an
//! ARMv7-M has three behaviours no other core in this tree has, each of which
//! breaks a debugger in its own way:
//!
//! * **Instructions are two or four bytes.** A breakpoint is an address
//!   compared against the program counter, so a wide Thumb-2 instruction is
//!   the case where an emulator that stepped by halfwords, or a stepper that
//!   stopped after the first halfword of an instruction, would look right and
//!   be wrong.
//! * **`IT`.** Up to four instructions after an `IT` are conditional, and the
//!   condition lives in `xPSR` rather than in them. A single step has to move
//!   one instruction and one `ITSTATE` slot; stepping the block as a unit, or
//!   losing the state between steps, both leave the guest somewhere it never
//!   was.
//! * **Exceptions are a stack frame and a magic return address.** Stepping an
//!   `SVC` lands in the handler with `EXC_RETURN` in `lr` — and on whichever
//!   stack `CONTROL.SPSEL` selected, which is exactly why a debugger needs
//!   `msp` and `psp` separately rather than the one `sp` the core feature has.
//!
//! Every assertion here reads through [`DebugTarget`] — the packets' own path
//! — and compares against the core's own accessors, so a map that agrees with
//! itself and not with the machine cannot pass.
//!
//! Machine code below is assembled by hand from ARM DDI 0403E.b's encoding
//! diagrams, with the section given at each one. No assembler runs in
//! `cargo test`, and no emulator of any licence was consulted.

#![cfg(all(feature = "gdb", feature = "cpu-arm-v7m"))]

use std::sync::Arc;

use rsemu::cpu::arm::v7m::{ArmV7m, xpsr};
use rsemu::host::gdb::{DebugTarget, MachineTarget, MemKind, StopKind};
use rsemu::machine::{Machine, catalog};

// ---------------------------------------------------------------------------
// The board
// ---------------------------------------------------------------------------

/// A Cortex-M4 with RAM from zero, so the vector table, the code and the stack
/// are all writable from the debugger.
///
/// The `rom` object is mapped and never executed: it is there so the memory
/// map has something to call read-only, which is the distinction
/// `qXfer:memory-map:read` exists to draw.
const BOARD: &str = r#"
machine "gdb-v7m-behaviour" {
  osc hclk = 168000000 Hz
  space mem { width = 32 }
  object cpu "cpu.arm.v7m" {
    clock  = hclk
    space  = mem
    part   = "cortex-m4"
  }
  object dram "ram" { size = 64K }
  object code "rom" { size = 4K }
  map mem 0x00000000 size 64K = dram
  map mem 0x08000000 size 4K  = code
}
"#;

/// Where the test programs live, and where the stack starts.
const CODE: u64 = 0x1000;
/// The handler the `SVC` vector points at.
const HANDLER: u64 = 0x2000;
/// The initial `MSP`, well clear of both.
const STACK: u32 = 0x8000;

/// Build the board, keeping the core so every assertion has an oracle.
fn board(name: &str) -> (Machine, Arc<ArmV7m>) {
    use rsemu::core::Captured;

    let cores: Arc<Captured<ArmV7m>> = Arc::new(Captured::new());
    let kept = Arc::clone(&cores);
    let mut bindings = catalog::bindings().expect("this build's bindings");
    bindings.replace("cpu.arm.v7m", move |props| {
        let cpu = Arc::new(ArmV7m::from_props(props)?);
        kept.push(&cpu);
        Ok(cpu)
    });
    let options = rsemu::machine::BuildOptions::new()
        .with_classes(catalog::classes())
        .with_bindings(bindings);
    let registry = catalog::registry().expect("a registry");
    let machine = rsemu::machine::build(name, BOARD, &registry, &options)
        .unwrap_or_else(|e| panic!("the v7m fixture does not realize: {e}"));
    let cpu = cores.last().expect("the binding captured the core");
    (machine, cpu)
}

/// Boot the core to `entry` the way the silicon does: a vector table at
/// `VTOR`'s reset value of zero, whose first word is the initial `MSP` and
/// whose second is the reset address.
///
/// Setting the registers directly would not do. A freshly built machine has a
/// reset pending, and the core spends its first step on the reset sequence —
/// which reads those two words and overwrites whatever a test had put in
/// `pc` and `sp` (DDI 0403E.b B1.5.5). Booting properly is also the honest
/// fixture: everything after it is a core that got where it is by running.
fn boot(target: &mut MachineTarget<'_>, entry: u64) {
    let mut table = Vec::new();
    table.extend_from_slice(&STACK.to_le_bytes());
    table.extend_from_slice(&(entry as u32 | 1).to_le_bytes());
    target
        .write_memory(0, 0, &table)
        .expect("the vector table is in RAM");
    // The reset sequence is a step of its own and retires no instruction, so
    // one step lands on the entry point with nothing executed.
    for _ in 0..4 {
        target.step(0).expect("a step");
        if u64::from(reg(target, r::PC)) == entry {
            return;
        }
    }
    panic!(
        "the core did not reset to {entry:#x}; it is at {:#x}",
        reg(target, r::PC)
    );
}

/// Write halfwords at `addr`, little-endian, through the debugger's own path.
fn assemble(target: &mut MachineTarget<'_>, addr: u64, code: &[u16]) {
    let mut bytes = Vec::with_capacity(code.len() * 2);
    for half in code {
        bytes.extend_from_slice(&half.to_le_bytes());
    }
    target
        .write_memory(0, addr, &bytes)
        .expect("the fixture's RAM is writable");
}

/// One register, as the `p` packet would read it.
fn reg(target: &MachineTarget<'_>, index: usize) -> u32 {
    let bytes = target
        .read_register(0, index)
        .unwrap_or_else(|e| panic!("register {index}: {e}"));
    u32::from_le_bytes(<[u8; 4]>::try_from(&bytes[..]).expect("a four-byte register"))
}

/// Write one register, as `P` would.
fn set_reg(target: &mut MachineTarget<'_>, index: usize, value: u32) {
    target
        .write_register(0, index, &value.to_le_bytes())
        .unwrap_or_else(|e| panic!("register {index}: {e}"));
}

/// gdb's numbering for this core's map, which is what `p`/`P` speak.
#[allow(unreachable_pub)]
mod r {
    pub const SP: usize = 13;
    pub const LR: usize = 14;
    pub const PC: usize = 15;
    pub const XPSR: usize = 16;
    pub const MSP: usize = 17;
    pub const PSP: usize = 18;
    pub const PRIMASK: usize = 19;
    pub const BASEPRI: usize = 20;
    pub const FAULTMASK: usize = 21;
    pub const CONTROL: usize = 22;
}

// ---------------------------------------------------------------------------
// org.gnu.gdb.arm.m-system
// ---------------------------------------------------------------------------

/// The six system registers, read and written through the packets' own path
/// and checked against the core's accessors.
///
/// The check that matters is that each one is the *core's* — a stub that
/// simply read back what was written would pass a round trip and show a user
/// numbers the machine never had. So every read is compared against
/// `ArmV7m::regs`, and every write is checked there too.
#[test]
fn the_m_system_registers_are_the_core_s_own() {
    let (mut m, cpu) = board("m-system");
    let mut target = MachineTarget::new(&mut m);

    let arch = target.arch(0).expect("a register map");
    for (index, name) in [
        (r::MSP, "msp"),
        (r::PSP, "psp"),
        (r::PRIMASK, "primask"),
        (r::BASEPRI, "basepri"),
        (r::FAULTMASK, "faultmask"),
        (r::CONTROL, "control"),
    ] {
        assert_eq!(arch.regs[index].name, name);
        assert_eq!(
            arch.feature_of(index),
            Some("org.gnu.gdb.arm.m-system"),
            "`{name}` is declared in the wrong feature, so GDB will not find it"
        );
    }
    assert_eq!(arch.feature_of(r::XPSR), Some("org.gnu.gdb.arm.m-profile"));

    // Distinct values everywhere, so a table that is off by one entry cannot
    // pass.
    let mut regs = cpu.regs();
    regs.msp = 0x2000_1000;
    regs.psp = 0x2000_2000;
    regs.primask = true;
    regs.basepri = 0x40;
    regs.faultmask = true;
    regs.control = 0; // Thread mode on the main stack.
    regs.xpsr = xpsr::T;
    cpu.set_regs(regs);

    let regs = cpu.regs();
    assert_eq!(reg(&target, r::MSP), regs.msp);
    assert_eq!(reg(&target, r::PSP), regs.psp);
    assert_eq!(reg(&target, r::PRIMASK), 1);
    assert_eq!(reg(&target, r::BASEPRI), u32::from(regs.basepri));
    assert_eq!(reg(&target, r::FAULTMASK), 1);
    assert_eq!(reg(&target, r::CONTROL), regs.control);
    // `sp` is the selected bank, and `CONTROL.SPSEL` is clear, so it is `MSP`.
    assert_eq!(reg(&target, r::SP), regs.msp);

    // And the writes reach the core rather than a shadow copy.
    set_reg(&mut target, r::MSP, 0x2000_3000);
    set_reg(&mut target, r::PSP, 0x2000_4000);
    set_reg(&mut target, r::PRIMASK, 0);
    set_reg(&mut target, r::BASEPRI, 0x20);
    set_reg(&mut target, r::FAULTMASK, 0);
    let regs = cpu.regs();
    assert_eq!(regs.msp, 0x2000_3000);
    assert_eq!(regs.psp, 0x2000_4000);
    assert!(!regs.primask);
    assert_eq!(regs.basepri, 0x20);
    assert!(!regs.faultmask);
    // Writing `MSP` moved the selected stack pointer with it, because `MSP` is
    // the selected one.
    assert_eq!(regs.r[13], 0x2000_3000);
}

/// `PRIMASK` and `FAULTMASK` are one bit each, and the chunk holds them as a
/// `bool`: anything non-zero has to arrive as a set bit, not as the byte the
/// user typed.
#[test]
fn a_wide_write_to_a_one_bit_mask_register_sets_the_bit() {
    let (mut m, cpu) = board("masks");
    let mut target = MachineTarget::new(&mut m);
    set_reg(&mut target, r::PRIMASK, 0xffff_ffff);
    set_reg(&mut target, r::FAULTMASK, 0x0000_0002);
    assert!(cpu.regs().primask);
    assert!(cpu.regs().faultmask);
    // Read back as the one bit it is, not as what was written.
    assert_eq!(reg(&target, r::PRIMASK), 1);
    assert_eq!(reg(&target, r::FAULTMASK), 1);
    // `BASEPRI` is a whole byte; the bits above it are RES0 and are dropped.
    set_reg(&mut target, r::BASEPRI, 0xdead_beef);
    assert_eq!(cpu.regs().basepri, 0xef);
    assert_eq!(reg(&target, r::BASEPRI), 0xef);
}

/// `CONTROL.SPSEL` decides which bank `sp` is, so writing it has to move the
/// banks — the core's own `sync_stack` (DDI 0403 B1.4.1) does, and a write
/// through the snapshot chunk would not unless it is made to.
#[test]
fn writing_control_moves_sp_to_the_bank_it_selects() {
    let (mut m, cpu) = board("spsel");
    let mut target = MachineTarget::new(&mut m);
    let mut regs = cpu.regs();
    regs.xpsr = xpsr::T; // Thread mode: `SPSEL` means something here.
    regs.msp = 0x2000_1000;
    regs.psp = 0x2000_2000;
    regs.control = 0;
    cpu.set_regs(regs);
    assert_eq!(reg(&target, r::SP), 0x2000_1000, "the main stack, to start");

    set_reg(&mut target, r::CONTROL, 2); // `SPSEL`
    let regs = cpu.regs();
    assert_eq!(regs.r[13], 0x2000_2000, "`sp` is now the process stack");
    assert_eq!(regs.msp, 0x2000_1000, "and neither bank lost its value");
    assert_eq!(regs.psp, 0x2000_2000);
    assert_eq!(reg(&target, r::SP), 0x2000_2000);
    assert_eq!(reg(&target, r::MSP), 0x2000_1000);
    assert_eq!(reg(&target, r::PSP), 0x2000_2000);

    // And back, which is the direction that would silently swap the two if the
    // hook re-derived nothing.
    set_reg(&mut target, r::CONTROL, 0);
    let regs = cpu.regs();
    assert_eq!(regs.r[13], 0x2000_1000);
    assert_eq!(regs.msp, 0x2000_1000);
    assert_eq!(regs.psp, 0x2000_2000);
}

/// A whole-register-file write is the packet that would get the banking wrong:
/// `G` hands over `sp`, `msp`, `psp` and `control` in the description's order,
/// and `control` — which decides where the other three go — is last.
#[test]
fn a_whole_file_write_keeps_the_two_stacks_apart() {
    let (mut m, cpu) = board("gpacket");
    let mut target = MachineTarget::new(&mut m);
    let mut regs = cpu.regs();
    regs.xpsr = xpsr::T;
    regs.msp = 0x2000_1000;
    regs.psp = 0x2000_2000;
    regs.control = 2; // Start on the process stack.
    cpu.set_regs(regs);

    // Read the file out, change `CONTROL` back to the main stack in the bytes,
    // and write the whole thing back — which is what `set $control = 0` does
    // in a GDB that has no `P` packet.
    let mut file = target.read_registers(0).expect("the register file");
    assert_eq!(
        file.len(),
        23 * 4,
        "seventeen core registers and six system"
    );
    let at = r::CONTROL * 4;
    file[at..at + 4].copy_from_slice(&0u32.to_le_bytes());
    target.write_registers(0, &file).expect("G");

    let regs = cpu.regs();
    assert_eq!(regs.control, 0);
    assert_eq!(regs.msp, 0x2000_1000, "the main stack pointer survived");
    assert_eq!(regs.psp, 0x2000_2000, "and so did the process one");
    assert_eq!(regs.r[13], 0x2000_1000, "`sp` followed `SPSEL`");
}

// ---------------------------------------------------------------------------
// A breakpoint on a wide instruction
// ---------------------------------------------------------------------------

/// `MOV.W Rd, #const` — encoding T2, DDI 0403E.b A7.7.76.
///
/// `11110 i 0 0010 S Rn(1111) : 0 imm3 Rd imm8`, with `S` clear, `i` and
/// `imm3` zero: a four-byte instruction whose first halfword alone decodes as
/// nothing at all.
const fn mov_w(rd: u16, imm8: u16) -> [u16; 2] {
    [0xf04f, (rd << 8) | imm8]
}

/// `B .` — an unconditional branch to itself, encoding T2 (A7.7.12), so a run
/// that overshoots the program stops rather than executing whatever is next.
const SPIN: u16 = 0xe7fe;

/// A `Z0` on the first halfword of a four-byte instruction must stop *before*
/// it, not inside it.
///
/// The observable is the destination register: the breakpoint fires with the
/// program counter on the instruction and the register still holding its
/// sentinel, and one step later the register holds what the instruction puts
/// there and the program counter is four bytes on — not two.
#[test]
fn a_breakpoint_on_a_wide_instruction_stops_before_it_executes() {
    let (mut m, _cpu) = board("wide-bp");
    let mut target = MachineTarget::new(&mut m);
    let wide = mov_w(0, 0x11);
    assemble(
        &mut target,
        CODE,
        &[0xbf00, wide[0], wide[1], SPIN], // nop; mov.w r0, #0x11; b .
    );
    boot(&mut target, CODE);
    // The sentinel the instruction overwrites, written the way GDB would.
    set_reg(&mut target, 0, 0xdead_beef);

    let wide_at = CODE + 2;
    target.add_breakpoint(wide_at, false).expect("Z0");
    target.begin_resume();
    let mut stop = None;
    for _ in 0..64 {
        if let Some(hit) = target.resume().expect("the machine advances") {
            stop = Some(hit);
            break;
        }
    }
    let stop = stop.expect("the breakpoint on the wide instruction never fired");
    assert_eq!(stop.kind, StopKind::Breakpoint { hardware: false });
    assert_eq!(u64::from(reg(&target, r::PC)), wide_at);
    assert_eq!(
        reg(&target, 0),
        0xdead_beef,
        "the instruction under the breakpoint had already run"
    );

    // One step is the whole four bytes.
    target.step(0).expect("a step");
    assert_eq!(reg(&target, 0), 0x11);
    assert_eq!(
        u64::from(reg(&target, r::PC)),
        wide_at + 4,
        "a step over a wide instruction landed inside it"
    );
}

// ---------------------------------------------------------------------------
// Stepping an IT block
// ---------------------------------------------------------------------------

/// `ITE EQ` — DDI 0403E.b A7.7.38. `1011 1111 firstcond mask`, with
/// `firstcond` = `EQ` (`0000`) and a mask of `1100`: two instructions, the
/// first on the condition and the second on its inverse.
const ITE_EQ: u16 = 0xbf0c;
/// `MOVS Rd, #imm8` — encoding T1 (A7.7.76). Inside an `IT` block it does not
/// set the flags, which is why it may appear there at all.
const fn mov_t1(rd: u16, imm8: u16) -> u16 {
    0x2000 | (rd << 8) | imm8
}
/// `CMP Rn, #imm8` — encoding T1 (A7.7.27).
const fn cmp_t1(rn: u16, imm8: u16) -> u16 {
    0x2800 | (rn << 8) | imm8
}

/// A single step inside an `IT` block moves one instruction and one `ITSTATE`
/// slot, including over the instruction the condition skips.
///
/// The skipped instruction is the interesting one: it retires, the program
/// counter moves past it, and nothing it names changes. A stepper that treated
/// a skipped instruction as "no instruction retired" would run to the end of
/// the block on one step, and a debugger that lost `ITSTATE` between steps
/// would execute it.
#[test]
fn stepping_an_it_block_takes_one_instruction_at_a_time() {
    let (mut m, _cpu) = board("it-block");
    let mut target = MachineTarget::new(&mut m);
    assemble(
        &mut target,
        CODE,
        &[
            mov_t1(0, 0),    // movs r0, #0        — sets Z
            cmp_t1(0, 0),    // cmp  r0, #0        — Z stays set, so EQ holds
            ITE_EQ,          // ite  eq
            mov_t1(1, 0x11), // moveq r1, #0x11    — taken
            mov_t1(2, 0x22), // movne r2, #0x22    — skipped
            SPIN,
        ],
    );
    boot(&mut target, CODE);
    set_reg(&mut target, 1, 0);
    set_reg(&mut target, 2, 0);

    // Two instructions to set the flags, then the `IT` itself.
    for expected in [CODE + 2, CODE + 4, CODE + 6] {
        target.step(0).expect("a step");
        assert_eq!(u64::from(reg(&target, r::PC)), expected);
    }
    // `ITSTATE` is visible to the debugger, in `xPSR` where the architecture
    // puts it: `IT[7:2]` in bits 15-10 and `IT[1:0]` in 26-25.
    let it = reg(&target, r::XPSR) & xpsr::IT_MASK;
    assert_ne!(
        it, 0,
        "the `IT` block's state is not in the `xpsr` GDB reads"
    );
    assert_eq!(
        ((reg(&target, r::XPSR) >> 10) & 0x3f) << 2 | ((reg(&target, r::XPSR) >> 25) & 3),
        0x0c,
        "`ITSTATE` is `firstcond:mask`, which for `ITE EQ` is 0b0000_1100"
    );

    // The conditional instruction that runs.
    target.step(0).expect("a step");
    assert_eq!(u64::from(reg(&target, r::PC)), CODE + 8);
    assert_eq!(reg(&target, 1), 0x11, "the taken instruction did not run");
    assert_eq!(reg(&target, 2), 0, "the skipped one ran early");

    // And the one the condition skips, which still costs exactly one step.
    target.step(0).expect("a step");
    assert_eq!(
        u64::from(reg(&target, r::PC)),
        CODE + 10,
        "the skipped instruction was not stepped over"
    );
    assert_eq!(reg(&target, 2), 0, "the skipped instruction ran");
    assert_eq!(
        reg(&target, r::XPSR) & xpsr::IT_MASK,
        0,
        "`ITSTATE` did not clear at the end of the block"
    );
}

// ---------------------------------------------------------------------------
// Stepping into an exception
// ---------------------------------------------------------------------------

/// `SVC #0` — DDI 0403E.b A7.7.175, encoding T1: `1101 1111 imm8`.
const SVC_0: u16 = 0xdf00;
/// SVCall is exception 11, so its vector is the twelfth word of the table
/// (DDI 0403E.b B1.5.2, Table B1-4).
const SVC_VECTOR: u64 = 11 * 4;

/// A step over `SVC` lands on the handler's first instruction, with the frame
/// stacked and `EXC_RETURN` in `lr`.
///
/// This is the step that has no equivalent on any other core here: the
/// program counter after it is not the next instruction and is not a branch
/// target either — it comes out of the vector table, and eight words of state
/// went onto a stack on the way. A debugger that reported the address after
/// the `SVC` would be showing a line of source the core is not on.
#[test]
fn stepping_into_an_exception_lands_on_the_handler_s_first_instruction() {
    let (mut m, _cpu) = board("exception");
    let mut target = MachineTarget::new(&mut m);
    assemble(&mut target, CODE, &[SVC_0, SPIN]);
    assemble(&mut target, HANDLER, &[mov_t1(3, 0x33), SPIN]);
    // The vector table, at `VTOR`'s reset value of zero: the handler's address
    // with bit 0 set, because every M-profile vector is a Thumb address.
    target
        .write_memory(0, SVC_VECTOR, &(HANDLER as u32 | 1).to_le_bytes())
        .expect("the vector table is in RAM");
    boot(&mut target, CODE);

    let sp_before = reg(&target, r::SP);
    assert_eq!(sp_before, STACK);
    assert_eq!(reg(&target, r::XPSR) & xpsr::EXCEPTION, 0, "Thread mode");

    // One step for the `SVC` itself, and at most one more for the entry
    // sequence — whether taking the exception retires with the instruction or
    // just after it is the core's business, not the protocol's.
    let mut landed = false;
    for _ in 0..2 {
        target.step(0).expect("a step");
        if u64::from(reg(&target, r::PC)) == HANDLER {
            landed = true;
            break;
        }
    }
    assert!(
        landed,
        "after stepping `SVC` the program counter is {:#x}, not the handler at {HANDLER:#x}",
        reg(&target, r::PC)
    );

    assert_eq!(
        reg(&target, r::XPSR) & xpsr::EXCEPTION,
        11,
        "the core is not in SVCall's handler mode"
    );
    // `EXC_RETURN`: the top 27 bits are set, and the low bits say which stack
    // and which mode to return to (DDI 0403E.b B1.5.8).
    assert_eq!(
        reg(&target, r::LR) & 0xffff_fff0,
        0xffff_fff0,
        "`lr` does not hold an `EXC_RETURN` value"
    );
    // Eight words of exception frame, on the main stack because that is what
    // Thread mode was using.
    assert_eq!(reg(&target, r::SP), sp_before - 32);
    assert_eq!(reg(&target, r::MSP), sp_before - 32);
    // Handler mode always runs on the main stack, whatever `SPSEL` says, so
    // the process stack pointer is untouched.
    assert_eq!(reg(&target, r::PSP), 0);
    // And the stacked return address is the instruction after the `SVC`.
    let mut stacked = [0u8; 4];
    target
        .read_memory(0, u64::from(reg(&target, r::SP)) + 24, &mut stacked)
        .expect("the frame is readable");
    assert_eq!(u64::from(u32::from_le_bytes(stacked)), CODE + 2);

    // The handler runs from there, one step at a time.
    target.step(0).expect("a step");
    assert_eq!(reg(&target, 3), 0x33);
}

// ---------------------------------------------------------------------------
// qXfer:memory-map:read
// ---------------------------------------------------------------------------

/// The memory map is the machine's, not a string.
///
/// Two mappings, one writable and one not, at the addresses the fixture puts
/// them at — and nothing anywhere else, because an unmapped address is one GDB
/// should be told is unmapped rather than one it writes into a hole.
#[test]
fn the_memory_map_is_read_off_the_machine_s_own_address_space() {
    use rsemu::host::gdb::memory_map_xml;

    let (mut m, _cpu) = board("memory-map");
    let target = MachineTarget::new(&mut m);
    let map = target
        .memory_map(0)
        .expect("this machine can describe itself");

    let ram = map
        .iter()
        .find(|r| r.start == 0)
        .expect("the RAM at zero is described");
    assert_eq!(ram.kind, MemKind::Ram);
    assert_eq!(ram.length, 64 * 1024);

    let rom = map
        .iter()
        .find(|r| r.start == 0x0800_0000)
        .expect("the ROM is described");
    assert_eq!(
        rom.kind,
        MemKind::Rom,
        "a ROM store reported as writable memory is a `load` that silently \
         does nothing"
    );
    assert_eq!(rom.length, 4 * 1024);

    // Nothing else: the core's own system block at 0xe000e000 is inside the
    // CPU rather than on the bus, and the rest of the space is a hole.
    assert_eq!(map.len(), 2, "{map:?}");

    let xml = memory_map_xml(&map);
    assert!(xml.starts_with("<?xml"), "{xml}");
    assert!(
        xml.contains("<memory type=\"ram\" start=\"0x0\" length=\"0x10000\"/>"),
        "{xml}"
    );
    assert!(
        xml.contains("<memory type=\"rom\" start=\"0x8000000\" length=\"0x1000\"/>"),
        "{xml}"
    );
    assert!(xml.trim_end().ends_with("</memory-map>"), "{xml}");
}
