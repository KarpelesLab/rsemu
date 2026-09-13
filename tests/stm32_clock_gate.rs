//! `RCC_xxENR` reaching a peripheral, on a board.
//!
//! Each device's own tests drive its `enable` pin directly, which proves the
//! gate and proves nothing about the wire. This proves the machine layer
//! carries it: that `wire rcc.ahb1en3 -> gpiod.enable` is the line that makes
//! `RCC_AHB1ENR` bit 3 mean what RM0090 §7.3.12 says it means, and that the
//! `xxRSTR` companion resets the block it names.
//!
//! The failure it exists for is the quiet one. A board that draws the wrong
//! bit number, or draws none at all, builds and runs: **an `enable` nobody
//! wires leaves a peripheral clocked**, which is the rule that lets every
//! board written before the gate keep working and is therefore also the rule
//! that hides a missing line. Only a board that expects a peripheral to be
//! *dead* until firmware enables it can tell the two apart.
//!
//! `machines/tests/stm32f4-gated.machine` is that board and is deliberately not the
//! shipped F407 — see `docs/platforms/stm32f407.md` for what wiring that one
//! costs. The accesses here are made over the bus rather than by running a
//! program, the same choice `tests/stm32f407_wiring.rs` makes: what is under
//! test is a wire, not an assembler.

#![cfg(all(
    feature = "cpu-arm-v7m",
    feature = "dev-stm32",
    feature = "dev-stm32-rcc",
    feature = "dev-stm32-tim"
))]

use std::sync::Arc;

use rsemu::core::clock::GlobalTime;
use rsemu::core::space::MemAttrs;
use rsemu::core::value::Width;
use rsemu::host::chardev::{CharPort, ports};
use rsemu::machine::{Machine, catalog};

/// The board, which is not in the catalog and is included by path.
const BOARD: &str = include_str!("../machines/tests/stm32f4-gated.machine");

/// `RCC`'s base (RM0090 Table 1: AHB1 + 0x3800).
const RCC: u64 = 0x4002_3800;
/// `RCC_AHB1ENR` (§7.3.12).
const AHB1ENR: u64 = RCC + 0x30;
/// `RCC_AHB1RSTR` (§7.3.5 of the reset half, Table 1's 0x10).
const AHB1RSTR: u64 = RCC + 0x10;
/// `RCC_APB1ENR` (§7.3.13).
const APB1ENR: u64 = RCC + 0x40;
/// `RCC_APB1RSTR`.
const APB1RSTR: u64 = RCC + 0x20;

/// `AHB1ENR.GPIODEN`, bit 3.
const GPIODEN: u64 = 1 << 3;
/// `APB1ENR.USART2EN`, bit 17.
const USART2EN: u64 = 1 << 17;
/// `APB1ENR.TIM2EN`, bit 0.
const TIM2EN: u64 = 1 << 0;

/// `GPIOD`'s base.
const GPIOD: u64 = 0x4002_0c00;
/// `GPIOx_MODER`, the first register of a port.
const MODER: u64 = 0x00;

/// `USART2`'s base.
const USART2: u64 = 0x4000_4400;
/// `USART_SR` on an F4 layout.
const SR: u64 = 0x00;
/// `USART_DR`.
const DR: u64 = 0x04;
/// `USART_CR1`.
const CR1: u64 = 0x0c;
/// `CR1`'s `UE | TE`.
const UE_TE: u64 = (1 << 13) | (1 << 3);
/// `SR.TXE`, which is set out of reset.
const TXE: u64 = 1 << 7;

/// `TIM2`'s base, which is the bottom of APB1.
const TIM2: u64 = 0x4000_0000;
/// `TIMx_CR1`, whose bit 0 is `CEN`.
const TIM_CR1: u64 = 0x00;
/// `TIMx_CNT`.
const CNT: u64 = 0x24;
/// `TIMx_ARR`. It resets to zero, and "the counter is blocked while the
/// auto-reload value is null" (RM0090 §18.4.8), so a timer that is to count at
/// all needs this written first.
const ARR: u64 = 0x2c;

/// How long a test that needs the character seam to move runs for.
///
/// A USART byte crosses on a `Device::run` call, and the scheduler hands those
/// out a round at a time, so a byte written after a round has been planned
/// leaves on the next one. Four milliseconds is several rounds of this board's
/// 1 MHz character clock and is not otherwise load-bearing.
const SPAN: GlobalTime = GlobalTime::from_nanos(4_000_000);

/// A firmware image that parks the core: a vector table and `b .`.
///
/// The tests drive the peripherals over the bus, but three of them let virtual
/// time pass, and a core fetching out of an all-zero image would spend that
/// time faulting rather than idling. The stack pointer is the top of the
/// board's SRAM and the reset vector is `ENTRY | 1`, Thumb as an ARMv7-M
/// requires.
fn parked() -> Vec<u8> {
    const ENTRY: u32 = 0x100;
    let mut image = vec![0u8; 0x104];
    image[0x00..0x04].copy_from_slice(&0x2001_0000u32.to_le_bytes());
    image[0x04..0x08].copy_from_slice(&(ENTRY | 1).to_le_bytes());
    // `B .` — encoding T2, DDI 0403 A7.7.12, branching to itself.
    image[0x100..0x102].copy_from_slice(&0xe7feu16.to_le_bytes());
    image
}

fn boot() -> (Machine, Arc<CharPort>) {
    let mut options = catalog::build_options().expect("the catalog agrees with itself");
    options.realize.media.insert("firmware", parked());
    let registry = catalog::registry().expect("a registry");
    let machine = match rsemu::machine::build("stm32f4-gated", BOARD, &registry, &options) {
        Ok(m) => m,
        Err(e) => panic!("the board does not realize: {e}"),
    };
    let port = ports::open(&options.realize.hosts, "console").expect("USART2 opened it");
    (machine, port)
}

/// A guest load.
fn load(m: &Machine, addr: u64) -> u64 {
    m.space("mem")
        .expect("the memory space")
        .read(addr, Width::U32, MemAttrs::DEFAULT)
        .expect("a mapped word")
}

/// A guest store.
fn store(m: &Machine, addr: u64, value: u64) {
    m.space("mem")
        .expect("the memory space")
        .write(addr, Width::U32, value, MemAttrs::DEFAULT)
        .expect("a mapped word");
}

#[test]
fn a_peripheral_whose_clock_is_disabled_reads_as_zero() {
    // GitHub issue #14's own test, on a board. Out of reset `AHB1ENR` has
    // nothing but the CCM bit set, so GPIOD has no clock: its registers are
    // not readable and the value that comes back is zero, whatever the guest
    // writes (RM0090's note under §7.3.12).
    let (m, _port) = boot();
    assert_eq!(load(&m, AHB1ENR) & GPIODEN, 0, "gated out of reset");

    store(&m, GPIOD + MODER, 0x5500_0000);
    assert_eq!(
        load(&m, GPIOD + MODER),
        0,
        "a gated port answered, so the `enable` wire is not carrying"
    );

    store(&m, AHB1ENR, load(&m, AHB1ENR) | GPIODEN);
    assert_eq!(
        load(&m, GPIOD + MODER),
        0,
        "the write that was dropped stays dropped"
    );
    store(&m, GPIOD + MODER, 0x5500_0000);
    assert_eq!(
        load(&m, GPIOD + MODER),
        0x5500_0000,
        "and now it configures"
    );
}

#[test]
fn a_gated_usart_moves_no_character() {
    // The other half of the issue's test: with `APB1ENR.USART2EN` clear,
    // `USART2_SR` reads zero and a `DR` write reaches no chardev.
    let (mut m, port) = boot();
    assert_eq!(load(&m, USART2 + SR), 0, "no clock, no status");

    store(&m, USART2 + CR1, UE_TE);
    store(&m, USART2 + DR, u64::from(b'x'));
    m.run_for(SPAN).expect("it runs");
    assert!(port.drain().is_empty(), "a gated USART transmitted");

    store(&m, APB1ENR, load(&m, APB1ENR) | USART2EN);
    assert_eq!(load(&m, USART2 + SR) & TXE, TXE, "and the clock is back");
    // `CR1` and `DR` were written while the block was deaf, so both writes are
    // gone and the guest has to make them again — which is the whole symptom.
    store(&m, USART2 + CR1, UE_TE);
    store(&m, USART2 + DR, u64::from(b'x'));
    m.run_for(SPAN).expect("it runs");
    assert_eq!(port.drain(), b"x");
}

#[test]
fn a_reset_bit_puts_its_peripheral_back() {
    // `AHB1RSTR.GPIODRST` is the *other* wire, and the one that distinguishes
    // "no clock" from "held in reset": a clock coming back leaves the register
    // file as it was, and a reset does not.
    let (m, _port) = boot();
    store(&m, AHB1ENR, load(&m, AHB1ENR) | GPIODEN);
    store(&m, GPIOD + MODER, 0x5500_0000);
    assert_eq!(load(&m, GPIOD + MODER), 0x5500_0000);

    store(&m, AHB1RSTR, GPIODEN);
    assert_eq!(load(&m, GPIOD + MODER), 0, "deaf while the line is pulled");
    store(&m, AHB1RSTR, 0);
    assert_eq!(load(&m, GPIOD + MODER), 0, "and back at its reset value");

    // Where merely gating it keeps what was written.
    store(&m, GPIOD + MODER, 0x5500_0000);
    store(&m, AHB1ENR, load(&m, AHB1ENR) & !GPIODEN);
    store(&m, AHB1ENR, load(&m, AHB1ENR) | GPIODEN);
    assert_eq!(
        load(&m, GPIOD + MODER),
        0x5500_0000,
        "removing a clock is not a reset"
    );

    // And the same for the other bank, because a bank's base offset is exactly
    // the kind of number a board file gets wrong: `APB1RSTR` bit 17 is
    // USART2RST, sixteen registers below `APB1ENR`'s bit 17.
    store(&m, APB1ENR, load(&m, APB1ENR) | USART2EN);
    store(&m, USART2 + CR1, UE_TE);
    assert_eq!(load(&m, USART2 + CR1), UE_TE);
    store(&m, APB1RSTR, USART2EN);
    store(&m, APB1RSTR, 0);
    assert_eq!(
        load(&m, USART2 + CR1),
        0,
        "`UE` and `TE` went with the reset"
    );
}

#[test]
fn a_gated_timer_does_not_count() {
    // The half a register model alone would get wrong: `TIM2EN` is not a mask
    // over the registers, it is the clock the counter counts. A gated timer
    // stands still and goes on from where it stood.
    let (mut m, _port) = boot();
    store(&m, APB1ENR, load(&m, APB1ENR) | TIM2EN);
    store(&m, TIM2 + ARR, 0xffff_ffff);
    store(&m, TIM2 + TIM_CR1, 1);
    m.run_for(SPAN).expect("it runs");
    let counted = load(&m, TIM2 + CNT);
    assert!(counted > 0, "an enabled timer counts");

    store(&m, APB1ENR, load(&m, APB1ENR) & !TIM2EN);
    m.run_for(SPAN).expect("it runs");
    store(&m, APB1ENR, load(&m, APB1ENR) | TIM2EN);
    let after = load(&m, TIM2 + CNT);
    assert_eq!(
        after, counted,
        "a timer with no clock counted, or caught up when it came back"
    );

    m.run_for(SPAN).expect("it runs");
    assert!(load(&m, TIM2 + CNT) > after, "and it goes on from there");
}

#[test]
fn a_snapshot_restores_the_gates_without_carrying_them() {
    // The design claim in `src/dev/stm32/gate.rs`: a gate's level is a level
    // RCC drives, and it is deliberately in **no** device's chunk. What makes
    // that safe is that `Rcc::load` republishes every gate pin and no
    // peripheral's `load` touches its gate, so the two cannot disagree
    // whichever order the chunks come back in. A second board that never
    // enabled anything is where a missing republish would show.
    let (a, _port_a) = boot();
    store(&a, AHB1ENR, load(&a, AHB1ENR) | GPIODEN);
    store(&a, GPIOD + MODER, 0x5500_0000);
    let saved = a.save().expect("it snapshots");

    let (mut b, _port_b) = boot();
    assert_eq!(load(&b, GPIOD + MODER), 0, "the fresh board is gated");
    b.load(&saved).expect("a fresh board takes the snapshot");
    assert_eq!(
        load(&b, GPIOD + MODER),
        0x5500_0000,
        "the restored board's GPIOD is deaf, so the gate did not come back"
    );
    assert_eq!(
        a.save().expect("a saves"),
        b.save().expect("b saves"),
        "and the two boards do not agree"
    );
}

#[test]
fn the_board_draws_the_rccs_own_interrupt() {
    // `wire rcc.irq -> cpu.irq5` — RM0090 Table 62 position 5. What the level
    // does is `src/dev/stm32/rcc/tests.rs`'s business; what this asserts is
    // that the pin exists under the name a board writes, which is the half a
    // unit test cannot fail on.
    let (m, _port) = boot();
    assert!(m.device("rcc").is_some());
    assert!(m.device("cpu").is_some());
}
