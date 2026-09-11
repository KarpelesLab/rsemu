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
        "rcc", "pwr", "crc", "iwdg", "wwdg", "exti", "syscfg", "dma1", "dma2", "tim1", "tim2",
        "tim3", "tim4", "tim5", "tim6", "tim7", "tim8", "tim9", "tim10", "tim11", "tim12", "tim13",
        "tim14",
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
