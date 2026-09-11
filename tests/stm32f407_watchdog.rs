//! The `stm32f407` board's watchdogs, end to end.
//!
//! `src/dev/stm32/iwdg.rs` and `wwdg.rs` prove their own counters expire and
//! pulse a `reset` pin; `src/dev/stm32/rcc/tests.rs` proves that a level on
//! `rcc.iwdgrst` sets `RCC_CSR.IWDGRSTF`. Neither proves the thing a user
//! cares about, because in both of them the test is the one driving the wire.
//!
//! This file joins the three ends up. **A watchdog actually times out, the
//! core actually restarts, and the firmware reads back the flag that says
//! which of the two did it.** Nothing here pokes a pin: the only writes are
//! the ones the hand-assembled program makes, and everything else is the
//! machine file's wiring doing its job. A wrong `wire` line — `iwdg.reset`
//! going to `rcc.wwdgrst`, or to nothing at all — passes every unit test in
//! the tree and fails here.
//!
//! The other half is `DBGMCU`. A watchdog you cannot freeze is a board you
//! cannot debug: set a breakpoint in the loop that kicks the `IWDG`, look away
//! for half a second, and the part resets under you. So the last test halts
//! the core the way a debugger does, with `DBG_IWDG_STOP` set, and asserts
//! that four timeouts' worth of virtual time go by without the reset that
//! would otherwise have landed four times over.
//!
//! Everything here needs a machine, so the whole file is gated on
//! `machine-stm32f407`.

#![cfg(feature = "machine-stm32f407")]

use rsemu::core::clock::GlobalTime;
use rsemu::core::space::MemAttrs;
use rsemu::core::value::Width;
use rsemu::machine::{Machine, catalog};

/// `RCC`'s base (RM0090 Table 1: AHB1 + 0x3800).
const RCC: u64 = 0x4002_3800;
/// `RCC_CSR`'s offset (RM0090 §7.3.21).
const CSR: u64 = 0x74;
/// `RCC_CSR.IWDGRSTF`, bit 29.
const IWDGRSTF: u64 = 1 << 29;
/// `RCC_CSR.WWDGRSTF`, bit 30.
const WWDGRSTF: u64 = 1 << 30;

/// `IWDG`'s base (APB1 + 0x3000).
const IWDG: u64 = 0x4000_3000;
/// `WWDG`'s base (APB1 + 0x2C00).
const WWDG: u64 = 0x4000_2c00;

/// `DBGMCU`'s base — not on any APB, but in the vendor window of the private
/// peripheral bus (RM0090 §38.16.1, ARM DDI 0403 B3.1).
const DBGMCU: u64 = 0xe004_2000;
/// `DBGMCU_APB1_FZ`'s offset.
const APB1_FZ: u64 = 0x08;
/// `DBGMCU_APB1_FZ.DBG_IWDG_STOP`, bit 12 (RM0090 §38.16.3).
const DBG_IWDG_STOP: u64 = 1 << 12;
/// What an F407's `DBGMCU_IDCODE` reads as: revision A of device `0x413`.
const IDCODE: u64 = 0x1000_0413;

/// Where SRAM1 starts. The program leaves its evidence at the bottom of it,
/// and SRAM survives the reset because `cpu.reset` resets the **core** and not
/// the peripherals — which is exactly how firmware finds a reset reason.
const SRAM1: u64 = 0x2000_0000;

/// The initial stack pointer: the top of SRAM2.
const STACK: u32 = 0x2002_0000;
/// Where the program's entry point is.
const ENTRY: u32 = 0x100;

/// The `IWDG`'s timeout with `PR = 0` and `RLR = 0xFFF`: 4 × 4096 = 16384 LSI
/// cycles, and the machine file's LSI is 32 kHz, so 512 ms exactly (RM0090
/// Table 96).
const IWDG_TIMEOUT_NS: u64 = 512_000_000;

/// A reload of `0xFF` instead: 4 × 256 = 1024 LSI cycles, 32 ms.
///
/// The tests that are about something other than the timeout itself use this
/// one, because the core spins at 168 MHz between the arming and the reset and
/// half a second of that is thirty million instructions of nothing.
const SHORT_RLR: u16 = 0x00ff;
/// What [`SHORT_RLR`] times out in.
const SHORT_TIMEOUT_NS: u64 = 32_000_000;

/// The `WWDG`'s timeout from `T = 0x7F` with `WDGTB = 0`: 64 decrements of
/// 4096 PCLK1 cycles each, and the board's PCLK1 is HSE × 21/4 = 42 MHz, so
/// 262144 / 42e6 s ≈ 6.241 ms (RM0090 §20.3).
const WWDG_TIMEOUT_NS: u64 = 6_241_523;

// ---------------------------------------------------------------------------
// The same very small Thumb-2 assembler the other two board files use
// ---------------------------------------------------------------------------
//
// Copied rather than shared because a `tests/` file is its own crate. Each
// encoding is checkable by hand against the ARMv7-M ARM, DDI 0403 A7.7.

/// `MOVW Rd, #imm16` — encoding T3, DDI 0403 A7.7.76.
fn movw(d: u16, imm16: u16) -> [u16; 2] {
    let i = (imm16 >> 11) & 1;
    let imm4 = (imm16 >> 12) & 0xf;
    let imm3 = (imm16 >> 8) & 7;
    let imm8 = imm16 & 0xff;
    [0xf240 | (i << 10) | imm4, (imm3 << 12) | (d << 8) | imm8]
}

/// `MOVT Rd, #imm16` — encoding T1, A7.7.79. The same field layout.
fn movt(d: u16, imm16: u16) -> [u16; 2] {
    let [a, b] = movw(d, imm16);
    [a | 0x0080, b]
}

/// `STR Rt, [Rn, #imm5*4]` — encoding T1, A7.7.158.
fn str_imm(t: u16, n: u16, off: u16) -> u16 {
    assert!(
        off.is_multiple_of(4) && off / 4 < 32,
        "STR T1 cannot reach {off:#x}"
    );
    0x6000 | ((off / 4) << 6) | (n << 3) | t
}

/// `LDR Rt, [Rn, #imm5*4]` — encoding T1, A7.7.42. The same shape as `STR`
/// with bit 11 set, which is the whole of the difference.
fn ldr_imm(t: u16, n: u16, off: u16) -> u16 {
    assert!(
        off.is_multiple_of(4) && off / 4 < 32,
        "LDR T1 cannot reach {off:#x}"
    );
    0x6800 | ((off / 4) << 6) | (n << 3) | t
}

/// `MOVS Rd, #imm8` — encoding T1, A7.7.76.
fn movs(d: u16, imm8: u16) -> u16 {
    0x2000 | (d << 8) | imm8
}

/// `B .` — encoding T2, A7.7.12, with a displacement of −4: branch to self.
fn spin() -> u16 {
    0xe7fe
}

/// Load a 32-bit constant into a low register, which is `MOVW` then `MOVT`.
fn load32(d: u16, value: u32) -> Vec<u16> {
    let mut out = Vec::new();
    out.extend(movw(d, value as u16));
    out.extend(movt(d, (value >> 16) as u16));
    out
}

// ---------------------------------------------------------------------------
// The firmware
// ---------------------------------------------------------------------------

/// A program that records `RCC_CSR` at `SRAM1[0]`, runs `arm`, and then hangs.
///
/// ```text
///   0x000: .word 0x20020000       ; initial SP
///   0x004: .word entry|1          ; reset vector
///
///   entry: r1 = RCC_CSR           ; *every* boot, before anything else
///          SRAM1[0] = r1          ;   so the second boot's copy has the flag
///          <arm>                  ; start the watchdog under test
///          b .                    ; and never kick it
/// ```
///
/// **Unconditionally**, on every boot, with no branch anywhere: the program
/// arms the watchdog again after the reset it caused, so the board reboots
/// forever. That is what the part does with a watchdog nobody kicks, and it
/// keeps the image down to encodings that can be read off the manual — the
/// alternative is a compare and a conditional branch whose displacement is the
/// one thing a hand assembler gets wrong.
fn firmware(arm: &[u16]) -> Vec<u8> {
    let mut main: Vec<u16> = Vec::new();
    main.extend(load32(0, (RCC + CSR) as u32));
    main.push(ldr_imm(1, 0, 0x00));
    main.extend(load32(2, SRAM1 as u32));
    main.push(str_imm(1, 2, 0x00));
    main.extend_from_slice(arm);
    main.push(spin());

    let mut image = vec![0u8; 0x200];
    image[0x00..0x04].copy_from_slice(&STACK.to_le_bytes());
    image[0x04..0x08].copy_from_slice(&(ENTRY | 1).to_le_bytes());
    let at = ENTRY as usize;
    assert!(
        at + main.len() * 2 <= image.len(),
        "the program is too long"
    );
    for (i, half) in main.iter().enumerate() {
        image[at + i * 2..at + i * 2 + 2].copy_from_slice(&half.to_le_bytes());
    }
    image
}

/// Unlock, set `PR = 0` (divide by four) and `RLR = rlr`, then start it.
///
/// `0x5555` is the only key that opens `PR` and `RLR`, and `0xCCCC` closes
/// them again as it starts the counter (RM0090 §21.4.1).
fn arm_iwdg(rlr: u16) -> Vec<u16> {
    let mut out = load32(0, IWDG as u32);
    out.extend(movw(1, 0x5555));
    out.push(str_imm(1, 0, 0x00)); // KR: unlock
    out.push(movs(1, 0));
    out.push(str_imm(1, 0, 0x04)); // PR: /4
    out.extend(movw(1, rlr));
    out.push(str_imm(1, 0, 0x08)); // RLR
    out.extend(movw(1, 0xcccc));
    out.push(str_imm(1, 0, 0x00)); // KR: start
    out
}

/// `CR = WDGA | T[6:0] = 0x7F`: activate from the top of the count.
///
/// One write, because `WDGA` and the counter are the same register and
/// activating is setting bit 7 (RM0090 §20.4.1).
fn arm_wwdg() -> Vec<u16> {
    let mut out = load32(0, WWDG as u32);
    out.push(movs(1, 0xff)); // WDGA | T = 0x7F
    out.push(str_imm(1, 0, 0x00)); // CR
    out
}

/// Read `DBGMCU_IDCODE` and leave it at `SRAM1[4]`.
///
/// The only way to prove the core reaches `0xE0042000` at all: the private
/// peripheral bus is answered *inside* the processor, and everything but the
/// vendor window from `0xE0042000` up never reaches the address space. A read
/// through the machine's `mem` space would pass with the routing broken.
fn read_idcode() -> Vec<u16> {
    let mut out = load32(0, DBGMCU as u32);
    out.push(ldr_imm(1, 0, 0x00));
    out.extend(load32(2, SRAM1 as u32));
    out.push(str_imm(1, 2, 0x04));
    out
}

// ---------------------------------------------------------------------------
// The harness
// ---------------------------------------------------------------------------

/// Build the board out of the catalog with `image` in its `firmware` slot.
fn boot_with(image: Vec<u8>) -> Machine {
    let entry = catalog::machine("stm32f407").expect("this build ships stm32f407");
    let mut options = catalog::build_options().expect("the catalog agrees with itself");
    options.realize.media.insert("firmware", image);
    let registry = catalog::registry().expect("a registry");
    match rsemu::machine::build(entry.name, entry.source, &registry, &options) {
        Ok(m) => m,
        Err(e) => panic!("the board does not realize: {e}"),
    }
}

/// Read one word without side effects.
fn peek(m: &Machine, addr: u64) -> u64 {
    m.space("mem")
        .expect("the memory space")
        .read(addr, Width::U32, MemAttrs::DEBUG)
        .expect("a mapped word")
}

/// Write one word the way the guest would.
fn store(m: &Machine, addr: u64, value: u64) {
    m.space("mem")
        .expect("the memory space")
        .write(addr, Width::U32, value, MemAttrs::DEFAULT)
        .expect("a mapped word");
}

/// Run for `ns` of virtual time.
fn run(m: &mut Machine, ns: u64) {
    m.run_for(GlobalTime::from_nanos(ns))
        .expect("the machine runs");
}

// ---------------------------------------------------------------------------
// The tests
// ---------------------------------------------------------------------------

#[test]
fn an_unkicked_iwdg_resets_the_core_and_firmware_reads_iwdgrstf() {
    let mut m = boot_with(firmware(&arm_iwdg(0x0fff)));

    // A power-on leaves BOR, pin and POR set and nothing else (RM0090
    // §7.3.21), so the two watchdog flags start clear.
    run(&mut m, 1_000_000);
    assert_eq!(peek(&m, RCC + CSR) & (IWDGRSTF | WWDGRSTF), 0);
    assert_ne!(
        peek(&m, SRAM1),
        0,
        "the program ran and recorded its first boot"
    );
    let first = peek(&m, SRAM1);
    assert_eq!(first & (IWDGRSTF | WWDGRSTF), 0, "nothing has reset yet");

    // Ten milliseconds short of the timeout, still nothing.
    run(&mut m, IWDG_TIMEOUT_NS - 11_000_000);
    assert_eq!(
        peek(&m, RCC + CSR) & IWDGRSTF,
        0,
        "the watchdog fired early"
    );

    // And past it, the whole path runs: the counter expires, `iwdg.reset`
    // pulses, RCC latches the cause, the core restarts through the vector
    // table, and the program's first act is to read the flag back.
    run(&mut m, 20_000_000);
    assert_ne!(
        peek(&m, RCC + CSR) & IWDGRSTF,
        0,
        "RCC_CSR.IWDGRSTF was never set"
    );
    let second = peek(&m, SRAM1);
    assert_ne!(
        second & IWDGRSTF,
        0,
        "the firmware re-ran but did not see IWDGRSTF; \
         the core did not actually reset"
    );
    assert_eq!(
        second & WWDGRSTF,
        0,
        "the independent watchdog set the *window* watchdog's flag — \
         `iwdg.reset` is wired to the wrong cause pin"
    );
}

#[test]
fn an_unkicked_wwdg_resets_the_core_and_firmware_reads_wwdgrstf() {
    let mut m = boot_with(firmware(&arm_wwdg()));

    // Well short of 6.24 ms.
    run(&mut m, 4_000_000);
    assert_eq!(
        peek(&m, RCC + CSR) & WWDGRSTF,
        0,
        "the window watchdog fired early"
    );
    assert_eq!(peek(&m, SRAM1) & (IWDGRSTF | WWDGRSTF), 0);

    run(&mut m, WWDG_TIMEOUT_NS);
    assert_ne!(
        peek(&m, RCC + CSR) & WWDGRSTF,
        0,
        "RCC_CSR.WWDGRSTF was never set"
    );
    let after = peek(&m, SRAM1);
    assert_ne!(
        after & WWDGRSTF,
        0,
        "the firmware re-ran but did not see WWDGRSTF"
    );
    assert_eq!(
        after & IWDGRSTF,
        0,
        "the window watchdog set the *independent* one's flag"
    );
}

#[test]
fn the_core_reaches_dbgmcu_in_the_vendor_window_of_the_ppb() {
    // `0xE0042000` is inside `0xE0000000`–`0xE00FFFFF`, which an ARMv7-M core
    // answers itself — except for the window DDI 0403 B3.1 leaves
    // implementation-defined, which is the one ST put `DBGMCU` in. If that
    // exception is missing the load below reads zero and nothing else fails.
    let mut m = boot_with(firmware(&[read_idcode(), arm_iwdg(SHORT_RLR)].concat()));
    run(&mut m, 1_000_000);
    assert_eq!(
        peek(&m, SRAM1 + 4),
        IDCODE,
        "the core did not reach DBGMCU_IDCODE"
    );
}

#[test]
fn a_halted_core_with_dbg_iwdg_stop_freezes_the_counter() {
    let mut m = boot_with(firmware(&arm_iwdg(SHORT_RLR)));
    run(&mut m, 1_000_000);
    assert_eq!(peek(&m, RCC + CSR) & IWDGRSTF, 0);

    // What a debug script does before it sets a breakpoint.
    store(&m, DBGMCU + APB1_FZ, DBG_IWDG_STOP);
    assert_eq!(peek(&m, DBGMCU + APB1_FZ), DBG_IWDG_STOP);

    // The bit on its own freezes nothing: the core is still running, and the
    // hardware's wording is "stopped when core is halted".
    run(&mut m, SHORT_TIMEOUT_NS + 4_000_000);
    assert_ne!(
        peek(&m, RCC + CSR) & IWDGRSTF,
        0,
        "DBG_IWDG_STOP froze a watchdog on a *running* core"
    );

    // Now do it properly: a fresh board, the bit set, and the machine told
    // that a debugger has it stopped — which is what `Machine::set_debug_halted`
    // carries and what the gdb session drives.
    let mut m = boot_with(firmware(&arm_iwdg(SHORT_RLR)));
    run(&mut m, 1_000_000);
    store(&m, DBGMCU + APB1_FZ, DBG_IWDG_STOP);
    m.set_debug_halted(true);
    assert!(m.debug_halted());

    // Four timeouts' worth of virtual time at a breakpoint.
    run(&mut m, 4 * SHORT_TIMEOUT_NS);
    assert_eq!(
        peek(&m, RCC + CSR) & IWDGRSTF,
        0,
        "the watchdog reset the board while a debugger had the core halted"
    );

    // Let it go and the counter picks up where it was rather than where the
    // wall clock is: it had nearly the whole timeout left, so it does not fire
    // immediately and it does fire soon after.
    m.set_debug_halted(false);
    assert!(!m.debug_halted());
    run(&mut m, SHORT_TIMEOUT_NS - 4_000_000);
    assert_eq!(
        peek(&m, RCC + CSR) & IWDGRSTF,
        0,
        "the frozen time was counted after all"
    );
    run(&mut m, 8_000_000);
    assert_ne!(
        peek(&m, RCC + CSR) & IWDGRSTF,
        0,
        "the watchdog never restarted once the debugger let go"
    );
}
