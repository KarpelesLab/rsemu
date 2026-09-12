//! The `stm32f407` board's peripheral wiring, end to end.
//!
//! `tests/stm32f407_board.rs` proves the board runs a program and that one
//! USART interrupt arrives. This file proves the rest of the machine file is
//! load-bearing rather than decorative, and it exists for one failure in
//! particular: **a wrong NVIC vector number does not fail.** A peripheral whose
//! `irq` pin is wired to the wrong `cpu.irq{n}` still pends an interrupt, the
//! core still takes an exception, and the only symptom is that somebody else's
//! handler runs. No unit test of the device catches that, and
//! `every_shipped_machine_realizes` does not either.
//!
//! So the test that matters here hand-assembles firmware with a **real vector
//! table**, arms two peripherals whose positions in RM0090 Table 62 are
//! different kinds of fact — TIM2's own vector at 28, and EXTI line 5's shared
//! `EXTI9_5` vector at 23 — and has each handler record the exception number it
//! is actually running as, out of its own `IPSR`. External interrupt *n* is
//! exception *n + 16*, so 28 and 23 have to come back as 44 and 39 and nothing
//! else will do.
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
/// `TIM2`'s base, which is the bottom of APB1.
const TIM2: u64 = 0x4000_0000;
/// `EXTI`'s base (APB2 + 0x3c00).
const EXTI: u64 = 0x4001_3c00;
/// `SYSCFG`'s base (APB2 + 0x3800).
const SYSCFG: u64 = 0x4001_3800;
/// `PWR`'s base (APB1 + 0x7000).
const PWR: u64 = 0x4000_7000;
/// The CRC unit's base (AHB1 + 0x3000).
const CRC: u64 = 0x4002_3000;
/// `DMA1`'s base (AHB1 + 0x6000).
const DMA1: u64 = 0x4002_6000;
/// Where CCM starts — reachable by the core and by nothing else.
const CCM: u64 = 0x1000_0000;
/// Where SRAM1 starts.
const SRAM1: u64 = 0x2000_0000;

/// `RCC_CR.HSEON`, and the `HSERDY` the hardware answers with.
const HSEON: u64 = 1 << 16;
/// `RCC_CR.HSERDY`.
const HSERDY: u64 = 1 << 17;
/// `RCC_CR.PLLON`.
const PLLON: u64 = 1 << 24;
/// `RCC_CR.PLLRDY`.
const PLLRDY: u64 = 1 << 25;

/// The initial stack pointer: the top of SRAM2.
const STACK: u32 = 0x2002_0000;
/// Where the program's entry point is.
const ENTRY: u32 = 0x100;
/// Where the `EXTI9_5` handler is.
const EXTI_HANDLER: u32 = 0x200;
/// Where the `TIM2` handler is.
const TIM_HANDLER: u32 = 0x280;

/// `EXTI9_5` is position 23 in RM0090 Table 62, so it is exception 23 + 16.
const EXTI9_5_EXCEPTION: u64 = 39;
/// `TIM2` is position 28, so it is exception 28 + 16.
const TIM2_EXCEPTION: u64 = 44;

// ---------------------------------------------------------------------------
// The same very small Thumb-2 assembler `stm32f407_board.rs` uses
// ---------------------------------------------------------------------------
//
// Copied rather than shared because a `tests/` file is its own crate and the
// alternative is a `tests/common/` module for six encodings. Each one is
// checkable by hand against the ARMv7-M ARM, DDI 0403 A7.7.

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

/// `MOVS Rd, #imm8` — encoding T1, A7.7.76.
fn movs(d: u16, imm8: u16) -> u16 {
    0x2000 | (d << 8) | imm8
}

/// `MRS Rd, IPSR` — encoding T1, A7.7.82.
///
/// `SYSm` 5 is `IPSR` (DDI 0403 B5.2.3), which reads the exception number the
/// processor is *currently executing*: zero in Thread mode, and otherwise the
/// vector-table index it came in through. That is the whole point of this file.
fn mrs_ipsr(d: u16) -> [u16; 2] {
    [0xf3ef, 0x8005 | (d << 8)]
}

/// `BX LR` — encoding T1, A7.7.20. In a handler this is an exception return:
/// `LR` holds `EXC_RETURN` rather than an address.
fn bx_lr() -> u16 {
    0x4770
}

/// Load a 32-bit constant into a low register, which is `MOVW` then `MOVT`.
fn load32(d: u16, value: u32) -> Vec<u16> {
    let mut out = Vec::new();
    out.extend(movw(d, value as u16));
    out.extend(movt(d, (value >> 16) as u16));
    out
}

/// The firmware image: a vector table with two real handlers in it, a program
/// that arms both sources, and the handlers themselves.
///
/// ```text
///   0x000: .word 0x20020000       ; initial SP
///   0x004: .word entry|1          ; reset vector
///   0x09c: .word exti_handler|1   ; exception 39 = external interrupt 23
///   0x0b0: .word tim_handler|1    ; exception 44 = external interrupt 28
///
///   entry: NVIC_ISER0 = (1<<23)|(1<<28)   ; enable both
///          EXTI_IMR   = 1<<5              ; unmask line 5
///          EXTI_SWIER = 1<<5              ; and raise it from software
///          TIM2_PSC   = 99                ; a slow enough counter that the
///          TIM2_ARR   = 9                 ;   update is not instantaneous
///          TIM2_DIER  = UIE
///          TIM2_CR1   = CEN
///          b .
///
///   exti_handler: EXTI_PR = 1<<5          ; write one to clear
///                 SRAM1[0] = IPSR
///                 bx lr
///
///   tim_handler:  TIM2_DIER = 0           ; drop the request
///                 TIM2_SR   = 0           ; and clear UIF with it
///                 SRAM1[4]  = IPSR
///                 bx lr
/// ```
fn firmware() -> Vec<u8> {
    let mut main: Vec<u16> = Vec::new();
    // Both interrupts enabled in one write to `NVIC_ISER0`.
    main.extend(load32(0, 0xe000_e100));
    main.extend(load32(1, (1 << 23) | (1 << 28)));
    main.push(str_imm(1, 0, 0x00));
    // EXTI: unmask line 5, then raise it from software.
    main.extend(load32(0, EXTI as u32));
    main.push(movs(1, 1 << 5));
    main.push(str_imm(1, 0, 0x00)); // IMR
    main.push(str_imm(1, 0, 0x10)); // SWIER
    // TIM2: prescale, reload, enable the update interrupt, start it.
    main.extend(load32(0, TIM2 as u32));
    main.extend(movw(1, 99));
    main.push(str_imm(1, 0, 0x28)); // PSC
    main.push(movs(1, 9));
    main.push(str_imm(1, 0, 0x2c)); // ARR
    main.push(movs(1, 1));
    main.push(str_imm(1, 0, 0x0c)); // DIER.UIE
    main.push(movs(1, 1));
    main.push(str_imm(1, 0, 0x00)); // CR1.CEN
    main.push(0xe7fe); // b .

    let mut exti_handler: Vec<u16> = Vec::new();
    exti_handler.extend(load32(0, EXTI as u32));
    exti_handler.push(movs(1, 1 << 5));
    exti_handler.push(str_imm(1, 0, 0x14)); // PR: write one to clear
    exti_handler.extend(mrs_ipsr(3));
    exti_handler.extend(load32(2, SRAM1 as u32));
    exti_handler.push(str_imm(3, 2, 0x00));
    exti_handler.push(bx_lr());

    let mut tim_handler: Vec<u16> = Vec::new();
    tim_handler.extend(load32(0, TIM2 as u32));
    tim_handler.push(movs(1, 0));
    tim_handler.push(str_imm(1, 0, 0x0c)); // DIER = 0: the request drops
    tim_handler.push(str_imm(1, 0, 0x10)); // SR = 0: and UIF with it
    tim_handler.extend(mrs_ipsr(3));
    tim_handler.extend(load32(2, SRAM1 as u32));
    tim_handler.push(str_imm(3, 2, 0x04));
    tim_handler.push(bx_lr());

    let mut image = vec![0u8; 0x300];
    fn word(image: &mut [u8], at: usize, value: u32) {
        image[at..at + 4].copy_from_slice(&value.to_le_bytes());
    }
    word(&mut image, 0x00, STACK);
    word(&mut image, 0x04, ENTRY | 1);
    word(&mut image, 4 * EXTI9_5_EXCEPTION as usize, EXTI_HANDLER | 1);
    word(&mut image, 4 * TIM2_EXCEPTION as usize, TIM_HANDLER | 1);

    for (at, code) in [
        (ENTRY, &main),
        (EXTI_HANDLER, &exti_handler),
        (TIM_HANDLER, &tim_handler),
    ] {
        let at = at as usize;
        assert!(
            at + code.len() * 2 <= image.len(),
            "a block ran off the end"
        );
        for (i, half) in code.iter().enumerate() {
            image[at + i * 2..at + i * 2 + 2].copy_from_slice(&half.to_le_bytes());
        }
    }
    image
}

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

/// A board with nothing in its firmware slot, for the tests that drive the
/// peripherals over the bus rather than from a program.
fn boot() -> Machine {
    boot_with(vec![0u8; 0x100])
}

/// Read one word without side effects.
fn peek(m: &Machine, addr: u64) -> u64 {
    m.space("mem")
        .expect("the memory space")
        .read(addr, Width::U32, MemAttrs::DEBUG)
        .expect("a mapped word")
}

/// Read one word the way the guest would, side effects and all.
fn load(m: &Machine, addr: u64) -> u64 {
    m.space("mem")
        .expect("the memory space")
        .read(addr, Width::U32, MemAttrs::DEFAULT)
        .expect("a mapped word")
}

/// Write one word the way the guest would.
fn store(m: &Machine, addr: u64, value: u64) {
    m.space("mem")
        .expect("the memory space")
        .write(addr, Width::U32, value, MemAttrs::DEFAULT)
        .expect("a mapped word");
}

#[test]
fn every_peripheral_the_part_has_is_on_the_board() {
    let m = boot();
    for path in [
        "rcc", "pwr", "crc", "rng", "iwdg", "wwdg", "exti", "syscfg", "dma1", "dma2", "tim1",
        "tim2", "tim3", "tim4", "tim5", "tim6", "tim7", "tim8", "tim9", "tim10", "tim11", "tim12",
        "tim13", "tim14",
    ] {
        assert!(
            m.device(path).is_some(),
            "the machine has no instance called `{path}`"
        );
    }
}

#[test]
fn the_register_blocks_decode_where_rm0090_table_1_puts_them() {
    // A read of each base, and a read a quarter of a kilobyte in, which is the
    // hole above a peripheral's registers. Getting a base wrong by 0x400 is the
    // mistake this catches, and it would otherwise look like a working board
    // until firmware wrote to the wrong chip.
    let m = boot();
    for (name, base) in [
        ("rcc", RCC),
        ("pwr", PWR),
        ("crc", CRC),
        ("exti", EXTI),
        ("syscfg", SYSCFG),
        ("dma1", DMA1),
        ("tim2", TIM2),
        ("tim1", 0x4001_0000),
        ("tim9", 0x4001_4000),
        ("iwdg", 0x4000_3000),
        ("wwdg", 0x4000_2c00),
        ("rng", 0x5006_0800),
    ] {
        assert!(
            m.space("mem")
                .expect("the memory space")
                .read(base, Width::U32, MemAttrs::DEBUG)
                .is_ok(),
            "nothing decodes `{name}`'s base at {base:#x}"
        );
    }
    // An STM32 peripheral is allotted a kilobyte and decodes the low bytes of
    // it; the rest is a hole, which is what makes a wrong offset a fault.
    assert!(
        m.space("mem")
            .expect("the memory space")
            .read(CRC + 0x100, Width::U32, MemAttrs::DEBUG)
            .is_err(),
        "the CRC unit answered above its register window"
    );
}

#[test]
fn an_hserdy_poll_terminates_now_that_rcc_is_a_device() {
    // The whole reason `st.rcc` replaced a scratch RAM window. `HSERDY` and
    // `PLLRDY` are hardware-set bits: RAM reads back what was written, so the
    // spin every vendor `SystemInit` opens with never exited.
    let mut m = boot();
    assert_eq!(load(&m, RCC) & HSERDY, 0, "HSERDY before HSEON");
    store(&m, RCC, load(&m, RCC) | HSEON);
    m.run_for(GlobalTime::from_nanos(1_000_000))
        .expect("it runs");
    assert_eq!(
        load(&m, RCC) & HSERDY,
        HSERDY,
        "HSEON never produced HSERDY, so a `SystemInit` spin would not exit"
    );

    store(&m, RCC, load(&m, RCC) | PLLON);
    m.run_for(GlobalTime::from_nanos(1_000_000))
        .expect("it runs");
    assert_eq!(
        load(&m, RCC) & PLLRDY,
        PLLRDY,
        "PLLON never produced PLLRDY"
    );

    // And dropping the bit drops the ready with it, which is the other half of
    // the contract: a ready bit is a property of the oscillator, not a latch.
    store(&m, RCC, load(&m, RCC) & !HSEON);
    assert_eq!(load(&m, RCC) & HSERDY, 0, "HSERDY outlived HSEON");
}

#[test]
fn ccm_is_in_the_cores_view_of_memory_and_not_in_a_dma_masters() {
    // The board file's comment used to say "CCM: no DMA reaches it" on a
    // single-space board, where it was a claim rather than a mechanism. It is
    // a mechanism now: `dma1` and `dma2` master `dmabus`, and CCM is not
    // mapped there, so a stream programmed at 0x10000000 takes a bus error.
    let m = boot();
    let mem = m.space("mem").expect("the core's space");
    let dma = m.space("dmabus").expect("the bus matrix's space");

    assert!(
        mem.write(CCM, Width::U32, 0x1234_5678, MemAttrs::DEFAULT)
            .is_ok(),
        "the core cannot reach CCM"
    );
    assert_eq!(
        mem.read(CCM, Width::U32, MemAttrs::DEBUG).ok(),
        Some(0x1234_5678)
    );
    assert!(
        dma.read(CCM, Width::U32, MemAttrs::DEBUG).is_err(),
        "a bus master reached CCM, which no DMA controller on an F4 can do"
    );

    // SRAM and the peripherals are in both, or the controllers could not
    // transfer at all.
    assert!(dma.read(SRAM1, Width::U32, MemAttrs::DEBUG).is_ok());
    assert!(dma.read(TIM2, Width::U32, MemAttrs::DEBUG).is_ok());
    // The boot alias is the core's first fetch and not a DMA window, and
    // neither is a DMA controller's own register block.
    assert!(dma.read(0, Width::U32, MemAttrs::DEBUG).is_err());
    assert!(dma.read(DMA1, Width::U32, MemAttrs::DEBUG).is_err());
}

#[test]
fn a_timer_and_an_exti_line_reach_the_handlers_table_62_gives_them() {
    // The test this file exists for. Each handler records its own `IPSR`, which
    // is the exception number the core actually vectored through — so a wire to
    // the wrong `cpu.irq{n}` cannot pass by pending *an* interrupt. External
    // interrupt n is exception n + 16 (DDI 0403 B1.5.2).
    let mut m = boot_with(firmware());
    m.run_for(GlobalTime::from_nanos(20_000_000))
        .expect("it runs");

    assert_eq!(
        peek(&m, SRAM1),
        EXTI9_5_EXCEPTION,
        "EXTI line 5 did not vector through `EXTI9_5`, which RM0090 Table 62 \
         puts at position 23 — `wire exti.irq5 -> cpu.irq23`"
    );
    assert_eq!(
        peek(&m, SRAM1 + 4),
        TIM2_EXCEPTION,
        "TIM2's update event did not vector through `TIM2`, which RM0090 \
         Table 62 puts at position 28 — `wire tim2.irq -> cpu.irq28`"
    );

    // The handlers ran to completion and returned: `PR` is clear, so the EXTI
    // request went away, and the timer is still counting with its interrupt
    // disabled rather than re-entering forever.
    assert_eq!(
        peek(&m, EXTI + 0x14) & (1 << 5),
        0,
        "EXTI PR5 still pending"
    );
    assert_eq!(peek(&m, TIM2 + 0x0c), 0, "TIM2 DIER was not cleared");
}

#[test]
fn the_board_still_snapshots_and_restores_with_every_peripheral_on_it() {
    // Twenty-three more stateful devices than the board had, each with its own
    // `save`/`load`. A round trip that changed the state hash would mean one of
    // them dropped something, and the machine-level hash is what notices.
    let mut m = boot_with(firmware());
    m.run_for(GlobalTime::from_nanos(20_000_000))
        .expect("it runs");

    let bytes = m.save().expect("the machine snapshots");
    let before = m.state_hash().expect("a hash");

    let mut other = boot_with(firmware());
    other.load(&bytes).expect("the snapshot loads");
    assert_eq!(
        other.state_hash().expect("a hash"),
        before,
        "a save/load round trip changed the machine's state hash"
    );
    assert_eq!(peek(&other, SRAM1 + 4), TIM2_EXCEPTION);
}

#[test]
fn a_dma_stream_faults_on_ccm_and_copies_out_of_sram() {
    // The other half of the CCM story, at the device rather than at the space:
    // `dma1` actually masters `dmabus`, so a memory-to-memory stream pointed at
    // CCM takes a bus error and sets `TEIF`, which is what firmware sees.
    // RM0090 §10.5.1: `LISR` bit 3 is `TEIF0` and bit 5 `TCIF0`.
    //
    // Stream 0's registers are at DMA1 + 0x10: `S0CR`, `S0NDTR`, `S0PAR`,
    // `S0M0AR`. `DIR = 10` is memory-to-memory, which is the one mode that
    // needs no request line — and nothing on this board drives one.
    const S0CR: u64 = DMA1 + 0x10;
    const S0NDTR: u64 = DMA1 + 0x14;
    const S0PAR: u64 = DMA1 + 0x18;
    const S0M0AR: u64 = DMA1 + 0x1c;
    // `DIR = 10`, `PINC`, `MINC`, byte-wide on both sides, then `EN`.
    const MEM2MEM: u64 = (0b10 << 6) | (1 << 9) | (1 << 10) | 1;
    const TEIF0: u64 = 1 << 3;
    const TCIF0: u64 = 1 << 5;

    let mut m = boot();

    // A control first: SRAM to SRAM, which is in `dmabus` and works.
    store(&m, SRAM1 + 0x100, 0x1234_5678);
    store(&m, S0PAR, SRAM1 + 0x100);
    store(&m, S0M0AR, SRAM1 + 0x200);
    store(&m, S0NDTR, 4);
    store(&m, S0CR, MEM2MEM);
    m.run_for(GlobalTime::from_nanos(1_000_000))
        .expect("it runs");
    assert_eq!(
        peek(&m, DMA1) & TCIF0,
        TCIF0,
        "the SRAM copy did not finish"
    );
    assert_eq!(
        peek(&m, SRAM1 + 0x200),
        0x1234_5678,
        "the bytes did not move"
    );

    // Then the same transfer sourced from CCM, on a fresh machine rather than by
    // re-arming the stream above. A stream re-armed between two `run_for` calls
    // moves its first beat a quantum later than one armed before the first, so
    // reusing the machine would make this assertion partly about when the
    // scheduler next reaches the controller. It is about the address map.
    let mut m = boot();
    store(&m, S0PAR, CCM);
    store(&m, S0M0AR, SRAM1 + 0x300);
    store(&m, S0NDTR, 4);
    store(&m, S0CR, MEM2MEM);
    m.run_for(GlobalTime::from_nanos(1_000_000))
        .expect("it runs");
    assert_eq!(
        peek(&m, DMA1) & TEIF0,
        TEIF0,
        "a DMA read of CCM succeeded; `dmabus` is not what the controller masters"
    );
    assert_eq!(
        peek(&m, SRAM1 + 0x300),
        0,
        "a byte came out of CCM over the bus matrix"
    );
}

#[test]
fn the_same_edge_on_pb0_is_ignored_until_exticr1_selects_port_b() {
    // The seam this board has three objects for: a port drives a pin, SYSCFG's
    // `EXTICR` chooses which port's pin 0 is EXTI line 0, and only then does an
    // edge there reach `PR`. A mux that ignored `EXTICR` would look identical
    // on port A, and that is the failure this catches.
    //
    // `GPIOB`'s base is AHB1 + 0x400; `MODER` is at 0x00 and `BSRR` at 0x18
    // (RM0090 §8.4). PB0 in output mode lets the port drive its own pin, which
    // is the only source of an edge on this board.
    const GPIOB: u64 = 0x4002_0400;
    const MODER: u64 = 0x00;
    const BSRR: u64 = 0x18;
    const IMR: u64 = 0x00;
    const RTSR: u64 = 0x08;
    const PR: u64 = 0x14;
    const EXTICR1: u64 = 0x08;

    let m = boot();
    store(&m, GPIOB + MODER, 0b01); // PB0 general-purpose output
    store(&m, EXTI + IMR, 1); // line 0 unmasked
    store(&m, EXTI + RTSR, 1); // rising edge

    // EXTICR1 comes out of reset naming port A, and PA0 is not moving.
    assert_eq!(peek(&m, SYSCFG + EXTICR1) & 0xf, 0, "reset is port A");
    store(&m, GPIOB + BSRR, 1); // PB0 -> high
    assert_eq!(
        peek(&m, EXTI + PR) & 1,
        0,
        "an edge on PB0 reached EXTI line 0 while EXTICR1 still named port A"
    );

    // Point the line at port B. PB0 is already high, so the mux republishes a
    // low-to-high transition on the line and `PR0` sets on the spot — which is
    // the spurious interrupt a real `HAL_GPIO_Init` produces.
    store(&m, SYSCFG + EXTICR1, 1);
    assert_eq!(
        peek(&m, EXTI + PR) & 1,
        1,
        "EXTICR1 selected port B and the pin sitting high did not reach line 0"
    );

    // And it is genuinely the mux rather than an accident of ordering: clear
    // the pending bit, take PB0 low and back up, and it sets again.
    store(&m, EXTI + PR, 1);
    assert_eq!(peek(&m, EXTI + PR) & 1, 0);
    store(&m, GPIOB + BSRR, 1 << 16); // PB0 -> low
    assert_eq!(peek(&m, EXTI + PR) & 1, 0, "a falling edge, and no FTSR");
    store(&m, GPIOB + BSRR, 1);
    assert_eq!(peek(&m, EXTI + PR) & 1, 1);
}

#[test]
fn memrmp_switches_the_boot_alias_to_sram() {
    // `SYSCFG_MEMRMP.MEM_MODE` is what the BOOT pins set and what firmware
    // moves afterwards, and moving it has to move the memory: the reason a
    // bootloader writes this register is to run a vector table out of SRAM.
    //
    // The board comes up at `MEM_MODE = 00`, so zero is the flash.
    let m = boot();
    let flash_word = peek(&m, 0x0800_0000);
    assert_eq!(
        peek(&m, 0),
        flash_word,
        "the alias is the flash out of reset"
    );

    store(&m, SRAM1 + 0x40, 0xcafe_f00d);
    assert_ne!(peek(&m, 0x40), 0xcafe_f00d, "SRAM is not at zero yet");

    // `MEM_MODE = 11`: embedded SRAM at 0x00000000 (RM0090 §9.2.1).
    store(&m, SYSCFG, 0b11);
    assert_eq!(peek(&m, SYSCFG) & 0b11, 0b11);
    assert_eq!(
        peek(&m, 0x40),
        0xcafe_f00d,
        "MEMRMP selected SRAM and 0x00000000 still reads the flash"
    );

    // The same memory, not a copy: a store through the alias lands in SRAM1.
    store(&m, 0x44, 0x1234_5678);
    assert_eq!(peek(&m, SRAM1 + 0x44), 0x1234_5678);

    // And back. `MEM_MODE = 01` is the system bootloader, which the part has
    // and this board does not model, so it faults rather than aliasing the
    // flash a second time.
    store(&m, SYSCFG, 0b00);
    assert_eq!(peek(&m, 0), flash_word);
    store(&m, SYSCFG, 0b01);
    assert!(
        m.space("mem")
            .expect("the memory space")
            .read(0, Width::U32, MemAttrs::DEBUG)
            .is_err(),
        "the system-bootloader encoding aliased something this board does not have"
    );
}

/// A firmware image that does nothing but stay alive: a vector table whose
/// reset vector points at `B .`.
///
/// [`boot`]'s empty image is fine for a test that pokes a peripheral and reads
/// it straight back, because the machine only has to get through realize. It is
/// *not* fine for one that runs the scheduler: a core that resets to `SP = 0`
/// and `PC = 0` faults into lockup and stops asking for time, and a peripheral
/// starved of quanta looks exactly like a peripheral that was never wired.
fn spinning_firmware() -> Vec<u8> {
    let mut image = vec![0u8; 0x400];
    image[0x00..0x04].copy_from_slice(&STACK.to_le_bytes());
    image[0x04..0x08].copy_from_slice(&(ENTRY | 1).to_le_bytes());
    // `B .` — encoding T2, DDI 0403 A7.7.12, with an `imm11` of -2 halfwords.
    let at = ENTRY as usize;
    image[at..at + 2].copy_from_slice(&0xe7feu16.to_le_bytes());
    image
}

#[test]
fn tim2_reaches_stream_7_only_when_chsel_names_its_channel() {
    // RM0090 Table 43 puts `TIM2_UP` on DMA1 **stream 1 channel 3** and on
    // **stream 7 channel 3**. This board wires both, which it could not do
    // before `CHSEL` gated the request: a guest using stream 7 for one of the
    // seven other requests that share it would otherwise be handed TIM2's
    // update events as well. That is the failure this test exists for, and it
    // is invisible to any unit test of either device.
    //
    // Stream 7's registers are at DMA1 + 0x10 + 0x18 * 7 (RM0090 §10.5).
    const S7CR: u64 = DMA1 + 0x10 + 0x18 * 7;
    const S7NDTR: u64 = S7CR + 4;
    const S7PAR: u64 = S7CR + 8;
    const S7M0AR: u64 = S7CR + 0x0c;
    /// `HISR` is DMA1 + 0x04, and stream 7's flags sit at bit 22 (§10.5.2).
    const HISR: u64 = DMA1 + 0x04;
    const TCIF7: u64 = 1 << (22 + 5);
    /// `TIM2_DIER.UDE`, RM0090 §17.4.4 bit 8: an update raises the DMA request.
    const DIER_UDE: u64 = 1 << 8;

    /// `CHSEL` is `SxCR` bits 27:25.
    fn chsel(n: u64) -> u64 {
        n << 25
    }

    /// Arm stream 7 peripheral-to-memory on channel `sel` and let TIM2's
    /// update event drive it, returning what `NDTR` reached.
    ///
    /// It polls `NDTR` between quanta the way a driver waiting on a transfer
    /// does, and that is not incidental: `st.tim` catches its counter up on
    /// access, so a run with no reads in it is a timer that produces no
    /// updates. Eight rounds is several times what the transfer needs.
    fn remaining_after_eight_rounds(sel: u64) -> (u64, u64) {
        let mut m = boot_with(spinning_firmware());
        store(&m, S7PAR, SRAM1 + 0x100);
        store(&m, S7M0AR, SRAM1 + 0x200);
        store(&m, S7NDTR, 4);
        // Peripheral-to-memory, byte on both sides, `MINC`, then `EN`.
        store(&m, S7CR, chsel(sel) | (1 << 10) | 1);
        // `PSC` = 99, `ARR` = 9: an update every thousand timer ticks.
        store(&m, TIM2 + 0x28, 99);
        store(&m, TIM2 + 0x2c, 9);
        store(&m, TIM2 + 0x0c, DIER_UDE);
        store(&m, TIM2, 1); // `CR1.CEN`
        let mut left = 4;
        for _ in 0..8 {
            m.run_for(GlobalTime::from_nanos(1_000_000))
                .expect("it runs");
            left = peek(&m, S7NDTR);
        }
        (left, peek(&m, HISR) & TCIF7)
    }

    // Channel 5 is a channel TIM2 is not on — Table 43's stream 7 channel 5 is
    // `TIM3_CH3` — so the update events must not be heard.
    assert_eq!(
        remaining_after_eight_rounds(5),
        (4, 0),
        "stream 7 is listening to channel 5 and heard TIM2 on channel 3"
    );

    // The same stream and the same wiring with `CHSEL = 3`: now it is TIM2's.
    assert_eq!(
        remaining_after_eight_rounds(3),
        (0, TCIF7),
        "TIM2_UP is wired to `dma1.req7c3` and CHSEL says 3, so it should move"
    );
}

#[test]
fn a_stream_hears_only_the_request_wired_to_it_and_not_its_neighbours() {
    // The finer half of the same story, and the one that is a machine-layer
    // fact rather than a device one.
    //
    // Table 43's DMA1 channel 3 column is all TIM2: stream 1 is
    // `TIM2_UP`/`TIM2_CH3`, stream 5 is `TIM2_CH1`, **stream 6 is
    // `TIM2_CH2`/`TIM2_CH4`**, stream 7 is `TIM2_UP`/`TIM2_CH4`. Writing that
    // column down puts `tim2.dma-up` into two pins and `tim2.dma-ch4` into two
    // more, and a net is a *connected component* of the wire statements — so
    // streams 1, 6 and 7 end up in one component with five drivers in it.
    //
    // A sink handed the component's drivers rather than its own would have
    // stream 6 served by TIM2's **update** event, which is exactly the spurious
    // beat `CHSEL` gating exists to prevent, arriving by a different road.
    // `realize::drivers_of` is what stops it.
    //
    // So: `UDE` on and every `CCxDE` off. Stream 1 is wired to `TIM2_UP` and
    // must run; stream 6 is not and must not.
    const S1CR: u64 = DMA1 + 0x10 + 0x18;
    const S6CR: u64 = DMA1 + 0x10 + 0x18 * 6;
    /// `LISR` is DMA1 + 0x00 and stream 1's flags sit at bit 6 (§10.5.1).
    const TCIF1: u64 = 1 << (6 + 5);
    /// `TIM2_DIER.UDE`, RM0090 §17.4.4 bit 8.
    const DIER_UDE: u64 = 1 << 8;

    let mut m = boot_with(spinning_firmware());
    for (cr, dst) in [(S1CR, SRAM1 + 0x200), (S6CR, SRAM1 + 0x300)] {
        store(&m, cr + 8, SRAM1 + 0x100); // `SxPAR`
        store(&m, cr + 0x0c, dst); // `SxM0AR`
        store(&m, cr + 4, 4); // `SxNDTR`
        // Peripheral-to-memory, byte on both sides, `MINC`, `CHSEL = 3`, `EN`.
        store(&m, cr, (3 << 25) | (1 << 10) | 1);
    }
    store(&m, TIM2 + 0x28, 99); // `PSC`
    store(&m, TIM2 + 0x2c, 9); // `ARR`
    store(&m, TIM2 + 0x0c, DIER_UDE); // the update event only
    store(&m, TIM2, 1); // `CR1.CEN`

    let mut stream1 = 4;
    let mut stream6 = 4;
    for _ in 0..8 {
        m.run_for(GlobalTime::from_nanos(1_000_000))
            .expect("it runs");
        stream1 = peek(&m, S1CR + 4);
        stream6 = peek(&m, S6CR + 4);
    }

    assert_eq!(stream1, 0, "stream 1 is `TIM2_UP`'s and should have run");
    assert_eq!(peek(&m, DMA1) & TCIF1, TCIF1);
    assert_eq!(
        stream6, 4,
        "stream 6's channel-3 cell is `TIM2_CH2`/`TIM2_CH4`, and neither is \
         enabled: it heard a request no `wire` statement gave it"
    );
}
