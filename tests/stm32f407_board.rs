//! The `stm32f407` board, end to end.
//!
//! A unit test can say "the USART's `MemOps` moved a byte". This says something
//! stronger: an ARMv7E-M core **named in a `.machine` file** is handed an
//! address space by the machine layer, resets out of the boot alias at zero,
//! runs a hand-assembled Thumb-2 program out of flash, drives a GPIO pin
//! through `BSRR`, pushes four characters out of USART2 onto the host's
//! terminal one scheduler tick at a time, and then *sees the USART's interrupt
//! arrive in its own NVIC* — through a wire the machine file drew from
//! `usart2.irq` to `cpu.irq38`.
//!
//! That last hop is the one this board exists to prove. An M-profile core's
//! NVIC is inside the core, so there is no controller object between the
//! peripheral and the CPU and nothing else in the tree wires an interrupt this
//! way.
//!
//! Everything here needs a machine, so the whole file is gated on
//! `machine-stm32f407`.

#![cfg(feature = "machine-stm32f407")]

use rsemu::core::clock::GlobalTime;
use rsemu::core::space::MemAttrs;
use rsemu::core::value::Width;
use rsemu::host::chardev::ports;
use rsemu::machine::{Machine, catalog};

/// `GPIOD`'s base, per the machine file (RM0090 §2.3, Table 1).
const GPIOD: u64 = 0x4002_0c00;
/// `USART2`'s base.
const USART2: u64 = 0x4000_4400;
/// Where SRAM1 starts.
const SRAM1: u64 = 0x2000_0000;

/// `GPIOx_IDR`'s offset — what the pins are at.
const IDR: u64 = 0x10;
/// `GPIOx_ODR`'s offset.
const ODR: u64 = 0x14;

/// What the program sends.
const MESSAGE: &[u8] = b"Hi!\n";

/// Where the message sits in flash, which is also where it sits in the boot
/// alias the core fetches through.
const MSG_ADDR: u32 = 0x180;

/// Where the program's entry point is.
const ENTRY: u32 = 0x100;

/// The initial stack pointer: the top of SRAM2, which is contiguous with
/// SRAM1 (0x2000_0000 + 112 KiB + 16 KiB).
const STACK: u32 = 0x2002_0000;

// ---------------------------------------------------------------------------
// A very small Thumb-2 assembler
// ---------------------------------------------------------------------------
//
// The crate has no assembler and this is the fifth board file to want one, but
// four encodings and two branch forms is not an assembler — it is a table,
// checkable by hand against the ARMv7-M ARM (DDI 0403, A7.7). What is computed
// rather than written down is the two branch displacements, because those are
// the part a human gets wrong.

/// `MOVW Rd, #imm16` — encoding T3, DDI 0403 A7.7.76.
///
/// `imm16` is `imm4:i:imm3:imm8`, which is why this is worth a function.
fn movw(d: u16, imm16: u16) -> [u16; 2] {
    let i = (imm16 >> 11) & 1;
    let imm4 = (imm16 >> 12) & 0xf;
    let imm3 = (imm16 >> 8) & 7;
    let imm8 = imm16 & 0xff;
    [0xf240 | (i << 10) | imm4, (imm3 << 12) | (d << 8) | imm8]
}

/// `MOVT Rd, #imm16` — encoding T1, DDI 0403 A7.7.79. The same field layout.
fn movt(d: u16, imm16: u16) -> [u16; 2] {
    let [a, b] = movw(d, imm16);
    [a | 0x0080, b]
}

/// `STR Rt, [Rn, #imm5*4]` — encoding T1, A7.7.158.
fn str_imm(t: u16, n: u16, off: u16) -> u16 {
    0x6000 | ((off / 4) << 6) | (n << 3) | t
}

/// `LDR Rt, [Rn, #imm5*4]` — encoding T1, A7.7.42.
fn ldr_imm(t: u16, n: u16, off: u16) -> u16 {
    0x6800 | ((off / 4) << 6) | (n << 3) | t
}

/// `LDRB Rt, [Rn, #imm5]` — encoding T1, A7.7.45.
fn ldrb_imm(t: u16, n: u16, off: u16) -> u16 {
    0x7800 | (off << 6) | (n << 3) | t
}

/// `MOVS Rd, #imm8` — encoding T1, A7.7.76.
fn movs(d: u16, imm8: u16) -> u16 {
    0x2000 | (d << 8) | imm8
}

/// `CMP Rn, #imm8` — encoding T1, A7.7.27.
fn cmp(n: u16, imm8: u16) -> u16 {
    0x2800 | (n << 8) | imm8
}

/// `ANDS Rdn, Rm` — encoding T1, A7.7.9.
fn ands(dn: u16, m: u16) -> u16 {
    0x4000 | (m << 3) | dn
}

/// `ADDS Rd, Rn, #imm3` — encoding T1, A7.7.3.
fn adds(d: u16, n: u16, imm3: u16) -> u16 {
    0x1c00 | (imm3 << 6) | (n << 3) | d
}

/// A conditional branch's displacement: `imm8` is signed, doubled, and taken
/// from `PC`, which in Thumb is the instruction's address plus four.
fn beq(at: usize, target: usize) -> u16 {
    let delta = (target as isize - (at as isize + 2)) as i16;
    0xd000 | (delta as u16 & 0xff)
}

/// The same for an unconditional `B`, which has eleven bits of it.
fn b(at: usize, target: usize) -> u16 {
    let delta = (target as isize - (at as isize + 2)) as i16;
    0xe000 | (delta as u16 & 0x7ff)
}

/// The firmware image: a vector table, the program, and the message.
///
/// ```text
///   0x000: .word 0x20020000            ; initial SP, the top of SRAM2
///   0x004: .word 0x00000101            ; reset vector, entry|1
///
///   0x100: movw r0, #0x0c00            ; r0 = GPIOD
///          movt r0, #0x4002
///          movw r1, #0x0000            ; r1 = 0x05000000: PD12 and PD13
///          movt r1, #0x0500            ;   are general-purpose outputs
///          str  r1, [r0, #0x00]        ; MODER
///          movw r1, #0x3000            ; BS12 | BS13
///          str  r1, [r0, #0x18]        ; BSRR -> both pins high, atomically
///
///          movw r4, #0x4400            ; r4 = USART2
///          movt r4, #0x4000
///          movw r1, #0x202c            ; UE | TE | RE | RXNEIE
///          str  r1, [r4, #0x0c]        ; CR1
///          movw r2, #0x0180            ; r2 = the message
///   tx:    ldrb r3, [r2, #0]
///          cmp  r3, #0
///          beq  done
///   wait:  ldr  r5, [r4, #0x00]        ; SR
///          movs r6, #0x80              ; TXE
///          ands r5, r6
///          cmp  r5, #0
///          beq  wait                   ; back pressure: spin until the byte went
///          str  r3, [r4, #0x04]        ; DR
///          adds r2, r2, #1
///          b    tx
///
///   done:  movw r1, #0x0000            ; r1 = 0x20000000: BR13
///          movt r1, #0x2000
///          str  r1, [r0, #0x18]        ; BSRR -> PD13 low, PD12 untouched
///
///          movw r5, #0xe204            ; r5 = NVIC_ISPR1, the core's own block
///          movt r5, #0xe000
///   poll:  ldr  r6, [r5, #0x00]
///          cmp  r6, #0
///          beq  poll                   ; wait for USART2's interrupt to arrive
///          movw r7, #0x0000            ; r7 = SRAM1
///          movt r7, #0x2000
///          str  r6, [r7, #0x00]        ; stash what the NVIC says
///          b    .
///
///   0x180: "Hi!\0"
/// ```
fn firmware() -> Vec<u8> {
    let mut code: Vec<u16> = Vec::new();
    code.extend(movw(0, 0x0c00));
    code.extend(movt(0, 0x4002));
    code.extend(movw(1, 0x0000));
    code.extend(movt(1, 0x0500));
    code.push(str_imm(1, 0, 0x00));
    code.extend(movw(1, 0x3000));
    code.push(str_imm(1, 0, 0x18));
    code.extend(movw(4, 0x4400));
    code.extend(movt(4, 0x4000));
    code.extend(movw(1, 0x202c));
    code.push(str_imm(1, 4, 0x0c));
    code.extend(movw(2, MSG_ADDR as u16));

    let tx = code.len();
    code.push(ldrb_imm(3, 2, 0));
    code.push(cmp(3, 0));
    let beq_done = code.len();
    code.push(0); // patched below
    let wait = code.len();
    code.push(ldr_imm(5, 4, 0x00));
    code.push(movs(6, 0x80));
    code.push(ands(5, 6));
    code.push(cmp(5, 0));
    let beq_wait = code.len();
    code.push(beq(beq_wait, wait));
    code.push(str_imm(3, 4, 0x04));
    code.push(adds(2, 2, 1));
    let b_tx = code.len();
    code.push(b(b_tx, tx));

    let done = code.len();
    code[beq_done] = beq(beq_done, done);
    code.extend(movw(1, 0x0000));
    code.extend(movt(1, 0x2000));
    code.push(str_imm(1, 0, 0x18));
    code.extend(movw(5, 0xe204));
    code.extend(movt(5, 0xe000));
    let poll = code.len();
    code.push(ldr_imm(6, 5, 0x00));
    code.push(cmp(6, 0));
    let beq_poll = code.len();
    code.push(beq(beq_poll, poll));
    code.extend(movw(7, 0x0000));
    code.extend(movt(7, 0x2000));
    code.push(str_imm(6, 7, 0x00));
    code.push(0xe7fe); // b .

    let mut image = vec![0u8; MSG_ADDR as usize + MESSAGE.len() + 1];
    image[0..4].copy_from_slice(&STACK.to_le_bytes());
    image[4..8].copy_from_slice(&(ENTRY | 1).to_le_bytes());
    let entry = ENTRY as usize;
    for (i, half) in code.iter().enumerate() {
        image[entry + i * 2..entry + i * 2 + 2].copy_from_slice(&half.to_le_bytes());
    }
    assert!(
        entry + code.len() * 2 <= MSG_ADDR as usize,
        "the program grew into its own message"
    );
    let msg = MSG_ADDR as usize;
    image[msg..msg + MESSAGE.len()].copy_from_slice(MESSAGE);
    image
}

/// Build the board out of the catalog, with the firmware in its `firmware`
/// slot and the far end of its console in hand.
fn boot() -> (Machine, std::sync::Arc<rsemu::host::chardev::CharPort>) {
    let entry = catalog::machine("stm32f407").expect("this build ships stm32f407");
    let mut options = catalog::build_options().expect("the catalog agrees with itself");
    options.realize.media.insert("firmware", firmware());
    let registry = catalog::registry().expect("a registry");
    let machine = match rsemu::machine::build(entry.name, entry.source, &registry, &options) {
        Ok(m) => m,
        Err(e) => panic!("the board does not realize: {e}"),
    };
    // The same rendezvous the machine file's `port = console` names.
    let port = ports::open(&options.realize.hosts, "console").expect("USART2 opened it");
    (machine, port)
}

/// Read one word of the guest's memory space, without side effects.
fn peek(m: &Machine, addr: u64) -> u64 {
    m.space("mem")
        .expect("the memory space")
        .read(addr, Width::U32, MemAttrs::DEBUG)
        .expect("a mapped word")
}

#[test]
fn the_board_realizes_with_the_core_bound_and_the_usart_wired_to_it() {
    let (m, _port) = boot();
    assert_eq!(m.name(), "stm32f407");
    for path in [
        "cpu", "rom", "ram1", "ram2", "ccmram", "rcc", "gpioa", "gpiob", "gpioc", "gpiod", "gpioe",
        "gpioh", "usart2",
    ] {
        assert!(
            m.device(path).is_some(),
            "the machine has no instance called `{path}`"
        );
    }
    // Flash is mapped twice: at 0x08000000, where it lives, and at zero, which
    // is the boot alias the core fetches `SP` and `PC` out of.
    assert_eq!(peek(&m, 0x0000_0000), u64::from(STACK));
    assert_eq!(peek(&m, 0x0800_0000), u64::from(STACK));
    assert_eq!(peek(&m, 0x0000_0004), u64::from(ENTRY | 1));

    // `GPIOA` comes up with its debug pins already in alternate function, and
    // `GPIOD` comes up blank. Two instances of one class, different reset
    // values, both out of the machine file.
    assert_eq!(peek(&m, 0x4002_0000), 0xa800_0000, "GPIOA MODER");
    assert_eq!(peek(&m, GPIOD), 0, "GPIOD MODER");
}

#[test]
fn flash_is_not_writable_through_the_bus() {
    // `perms = "r-x"` on both mappings of the flash. A stray store into a
    // program is a fault rather than a silently modified program, and that is
    // a property of the decode in front of the chip rather than of the chip.
    let (m, _port) = boot();
    let space = m.space("mem").expect("the memory space");
    assert!(
        space
            .write(0x0800_0000, Width::U32, 0, MemAttrs::DEFAULT)
            .is_err(),
        "the flash mapping accepted a write"
    );
    assert!(
        space
            .write(0x0000_0000, Width::U32, 0, MemAttrs::DEFAULT)
            .is_err(),
        "the boot alias accepted a write"
    );
    // SRAM, at the same instant, does not object.
    assert!(
        space
            .write(SRAM1, Width::U32, 0x1234, MemAttrs::DEFAULT)
            .is_ok()
    );
}

#[test]
fn the_program_toggles_a_gpio_pin_and_writes_a_string_out_of_the_usart() {
    let (mut m, port) = boot();

    // Twenty milliseconds of virtual time, which is far more than four
    // characters need. The rate is not the USART's 11520 Hz domain: the guest
    // has to *write* each byte between the ticks that move them, so the string
    // comes out at one byte per scheduler quantum — and it only comes out at
    // all if the scheduler is genuinely interleaving a 168 MHz core with a
    // peripheral four orders of magnitude slower than it, with the guest
    // spinning on `TXE` in between.
    m.run_for(GlobalTime::from_nanos(20_000_000))
        .expect("it runs");

    assert_eq!(
        port.drain(),
        MESSAGE,
        "the message did not come out of USART2"
    );

    // `BSRR` set both pins and then reset one of them, and `IDR` is what a
    // scope on the pin would read.
    assert_eq!(
        peek(&m, GPIOD + IDR) & 0x3000,
        0x1000,
        "PD12 should be high and PD13 low"
    );
    assert_eq!(peek(&m, GPIOD + ODR) & 0x3000, 0x1000);
}

#[test]
fn a_usart_interrupt_reaches_the_cores_own_nvic() {
    // The wiring decision this board exists to demonstrate. There is no
    // interrupt controller between the peripheral and the core: the machine
    // file draws one line, `wire usart2.irq -> cpu.irq38`, and 38 is USART2's
    // position in RM0090 Table 62. What the guest reads back is bit 6 of
    // `NVIC_ISPR1` — external interrupt 32 + 6 — out of the core's own system
    // block at 0xE000E000, which no address space maps.
    let (mut m, port) = boot();

    // A byte from the host. The program has already set `RXNEIE`, so `RXNE`
    // going high drives the pin.
    port.feed(b"?");

    m.run_for(GlobalTime::from_nanos(20_000_000))
        .expect("it runs");

    assert_eq!(port.drain(), MESSAGE);
    assert_eq!(
        peek(&m, SRAM1),
        1 << 6,
        "the guest did not see USART2's interrupt pending in NVIC_ISPR1"
    );
    // Pending, and deliberately not taken: nothing wrote `NVIC_ISER1`, so the
    // interrupt is disabled and the core stays in Thread mode. A pending bit
    // for a disabled interrupt is exactly what the architecture says happens
    // (DDI 0403 B3.4.4).
    assert_eq!(
        peek(&m, USART2) & 0x20,
        0x20,
        "and RXNE is still set, because nothing serviced it"
    );
}

#[test]
fn the_board_snapshots_and_restores_to_an_identical_state_hash() {
    let (mut m, _port) = boot();
    m.run_for(GlobalTime::from_nanos(20_000_000))
        .expect("it runs");

    let bytes = m.save().expect("the machine snapshots");
    let before = m.state_hash().expect("a hash");

    let (mut other, _other_port) = boot();
    other.load(&bytes).expect("the snapshot loads");
    assert_eq!(
        other.state_hash().expect("a hash"),
        before,
        "a save/load round trip changed the machine's state hash"
    );

    // And the restored machine's pins and registers came across, rather than
    // the reset it would otherwise have taken on the way in.
    assert_eq!(peek(&other, GPIOD + ODR) & 0x3000, 0x1000);
}
