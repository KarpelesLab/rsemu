//! The STM32L4 Firewall on a board, with the two wires that are only a board.
//!
//! `src/dev/stm32/firewall/tests.rs` proves the state machine. This proves the
//! machine layer carries it: that clearing `SYSCFG_CFGR1.FWDIS` reaches the
//! firewall over a wire and switches it on, that an illegal access pulses a
//! reset line the board drew to the core *and* to `rcc.fwrst`, and that
//! `RCC_CSR.FWRSTF` therefore says what rebooted the part.
//!
//! Neither of those can fail in a unit test and neither can fail loudly: a
//! board that forgot `wire syscfg.fwdis -> fw.fwdis` has a firewall that
//! protects nothing, and one that forgot `wire fw.reset -> rcc.fwrst` has a
//! firmware that cannot tell a firewall reset from a power-on.
//!
//! The accesses are made over the bus rather than by running a program, which
//! is the same choice `tests/stm32f407_wiring.rs` makes for the peripherals it
//! drives: an instruction fetch is an ordinary read with
//! [`AccessPurpose::FETCH`] on it, and hand-assembling a protected routine
//! would test the assembler.

#![cfg(all(
    feature = "cpu-arm-v7m",
    feature = "dev-stm32-firewall",
    feature = "dev-stm32-exti",
    feature = "dev-stm32-rcc"
))]

use rsemu::core::error::BusError;
use rsemu::core::space::{AccessPurpose, MemAttrs};
use rsemu::core::value::Width;
use rsemu::machine::{Machine, catalog};

/// The board, which is not in the catalog and is included by path.
const BOARD: &str = include_str!("../machines/tests/stm32l4-firewall.machine");

/// `SYSCFG`'s base (RM0351 Table 1).
const SYSCFG: u64 = 0x4001_0000;
/// `SYSCFG_CFGR1`, whose bit 0 is `FWDIS`.
const CFGR1: u64 = SYSCFG + 0x04;
/// `FWDIS`.
const FWDIS: u64 = 1 << 0;

/// The firewall's base (RM0351 §4.4).
const FW: u64 = 0x4001_1c00;
/// `FW_CSSA`, which is the block's first register.
const FW_CSSA: u64 = FW;
/// `RCC`'s base.
const RCC: u64 = 0x4002_1000;
/// `RCC_CSR` (RM0351 §6.4.29).
const CSR: u64 = RCC + 0x94;
/// `RCC_CSR.FWRSTF`, the firewall reset flag.
const FWRSTF: u64 = 1 << 24;
/// `RCC_CSR.RMVF`, which clears the reset flags. Bit 23 on an L4.
const RMVF: u64 = 1 << 23;

/// The code segment: 0x08001000, 0x400 bytes.
const CSSA: u64 = 0x0000_1000;
/// Its length.
const CSL: u64 = 0x0000_0400;
/// The non-volatile data segment: 0x08002000, 0x200 bytes.
const NVDSSA: u64 = 0x0000_2000;
/// Its length.
const NVDSL: u64 = 0x0000_0200;

/// The call gate: the first word of the code segment.
const GATE: u64 = 0x0800_0000 + CSSA;
/// The middle of the protected routine, which is not a legal entry point.
const INSIDE_CODE: u64 = GATE + 8;
/// Unprotected flash, where the caller lives.
const OUTSIDE: u64 = 0x0800_0000 + 0x100;
/// The first word of the protected data.
const NVDS: u64 = 0x0800_0000 + NVDSSA;

fn boot() -> Machine {
    let mut options = catalog::build_options().expect("the catalog agrees with itself");
    options.realize.media.insert("firmware", vec![0u8; 0x100]);
    let registry = catalog::registry().expect("a registry");
    match rsemu::machine::build("stm32l4-firewall", BOARD, &registry, &options) {
        Ok(m) => m,
        Err(e) => panic!("the board does not realize: {e}"),
    }
}

/// A guest load, through the firewall.
fn load(m: &Machine, addr: u64) -> Result<u64, BusError> {
    m.space("cpubus")
        .expect("the core's space")
        .read(addr, Width::U32, MemAttrs::DEFAULT)
}

/// A guest store, through the firewall.
fn store(m: &Machine, addr: u64, value: u64) -> Result<(), BusError> {
    m.space("cpubus")
        .expect("the core's space")
        .write(addr, Width::U32, value, MemAttrs::DEFAULT)
}

/// An instruction fetch, through the firewall.
fn fetch(m: &Machine, addr: u64) -> Result<u64, BusError> {
    m.space("cpubus").expect("the core's space").read(
        addr,
        Width::U32,
        MemAttrs::DEFAULT.with_purpose(AccessPurpose::FETCH),
    )
}

/// A side-effect-free read, which the firewall lets through unjudged.
fn peek(m: &Machine, addr: u64) -> u64 {
    m.space("mem")
        .expect("the memory space")
        .read(addr, Width::U32, MemAttrs::DEBUG)
        .expect("a mapped word")
}

/// Program the two flash segments and enable the firewall.
fn arm(m: &Machine) {
    store(m, FW_CSSA, CSSA).expect("CSSA");
    store(m, FW + 0x04, CSL).expect("CSL");
    store(m, FW + 0x08, NVDSSA).expect("NVDSSA");
    store(m, FW + 0x0c, NVDSL).expect("NVDSL");
    store(m, CFGR1, 0).expect("FWDIS");
}

#[test]
fn the_board_realizes_with_the_firewall_in_front_of_the_core() {
    // `cpubus` is the core's space and holds nothing but the filter; `mem` is
    // the real map. A board that gave the core `mem` directly would work for
    // every test that does not involve a fetch, which is most of them.
    let m = boot();
    assert!(m.space("cpubus").is_some());
    assert!(m.space("mem").is_some());
    // The flash alias at zero answers through the filter, which is what the
    // core's first fetch of `SP` and `PC` goes through.
    assert!(fetch(&m, 0).is_ok());
    assert_eq!(load(&m, FW + 0x20).ok(), Some(0), "FW_CR reads back");
}

#[test]
fn clearing_fwdis_over_the_wire_is_what_switches_the_firewall_on() {
    // The seam: `SYSCFG_CFGR1.FWDIS` had no reader at all before this board.
    let m = boot();
    assert_eq!(load(&m, CFGR1).ok(), Some(FWDIS), "FWDIS resets high");

    store(&m, FW_CSSA, CSSA).expect("CSSA");
    store(&m, FW + 0x04, CSL).expect("CSL");
    assert!(
        fetch(&m, INSIDE_CODE).is_ok(),
        "the segments are programmed but the firewall is still idle"
    );

    store(&m, CFGR1, 0).expect("the one write a trusted bootloader makes");
    assert_eq!(load(&m, CFGR1).ok(), Some(0));
    assert_eq!(
        fetch(&m, INSIDE_CODE),
        Err(BusError::Protected),
        "FWDIS was cleared and the fence did not come up — check `wire \
         syscfg.fwdis -> fw.fwdis`"
    );
}

#[test]
fn an_illegal_entry_resets_the_machine_and_rcc_says_it_was_the_firewall() {
    // The other wire, and the reason it is two wires and not one: the core is
    // reset, and `RCC_CSR` remembers why, because the flag is RCC's state.
    let m = boot();
    // Clear whatever the power-on left in `CSR`, so `FWRSTF` is unambiguous.
    store(&m, CSR, RMVF).expect("RMVF");
    assert_eq!(peek(&m, CSR) & FWRSTF, 0, "no firewall reset yet");

    arm(&m);
    assert_eq!(fetch(&m, INSIDE_CODE), Err(BusError::Protected));
    assert_ne!(
        peek(&m, CSR) & FWRSTF,
        0,
        "the firewall reset the machine and `RCC_CSR.FWRSTF` did not latch — \
         check `wire fw.reset -> rcc.fwrst`"
    );

    // A second illegal access latches it again, which is what a firmware that
    // cleared the flag and then tripped the fence twice would see.
    store(&m, CSR, RMVF).expect("RMVF");
    assert_eq!(peek(&m, CSR) & FWRSTF, 0);
    assert_eq!(load(&m, NVDS), Err(BusError::Protected));
    assert_ne!(peek(&m, CSR) & FWRSTF, 0);
}

#[test]
fn the_protected_data_is_reachable_only_through_the_call_gate() {
    let m = boot();
    store(&m, NVDS, 0xc0ff_ee00).expect("seed the protected data");
    arm(&m);

    assert_eq!(
        load(&m, NVDS),
        Err(BusError::Protected),
        "untrusted code read the protected data"
    );
    assert!(fetch(&m, GATE).is_ok(), "the call gate");
    assert_eq!(load(&m, NVDS).ok(), Some(0xc0ff_ee00));

    // `FW_CR.FPA` is the exit protocol: set it, then leave. Without it the
    // fetch outside the code segment is a reset.
    store(&m, FW + 0x20, 1).expect("FPA");
    assert!(fetch(&m, OUTSIDE).is_ok());
    assert_eq!(
        load(&m, FW + 0x20).ok(),
        Some(0),
        "the hardware cleared FPA"
    );
    assert_eq!(load(&m, NVDS), Err(BusError::Protected), "closed again");
}

#[test]
fn the_board_snapshots_and_restores_with_the_firewall_open() {
    // The open/closed bit has no register, so a snapshot that dropped it would
    // restore a machine whose protected code is suddenly unreachable — or,
    // worse, reachable.
    let m = boot();
    arm(&m);
    assert!(fetch(&m, GATE).is_ok());

    let bytes = m.save().expect("the machine snapshots");
    let before = m.state_hash().expect("a hash");

    let mut other = boot();
    other.load(&bytes).expect("the snapshot loads");
    assert_eq!(
        other.state_hash().expect("a hash"),
        before,
        "a save/load round trip changed the machine's state hash"
    );
    assert!(load(&other, NVDS).is_ok(), "still open after the restore");
}
