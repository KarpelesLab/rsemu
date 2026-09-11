//! The `stm32f407` board's RNG: where it decodes, which vector it takes, and —
//! the substance — that it is reproducible.
//!
//! `src/dev/stm32/rng.rs` proves the register face one device at a time. This
//! file proves the three claims that are only true of a *machine*:
//!
//! 1. **Two runs of one board deal the same words and hash the same.** That is
//!    `ROADMAP.md` §0's rule, applied to the one device on this board whose
//!    whole job is to look random. A device test asserting `Stream::new(7)`
//!    twice does not check it, because the interesting failure is a stream
//!    seeded from something the board did not write down.
//! 2. **The seed is a knob.** `-p seed=…` deals a different board, and that is
//!    the machine-level control the design chose over a global.
//! 3. **`wire rng.irq -> cpu.irq80` is the right number.** A wrong vector does
//!    not fail: the core takes an exception and somebody else's handler runs.
//!    The only way to catch it is to ask the handler what it is executing as,
//!    which is what the firmware below does — `tests/stm32f407_wiring.rs`
//!    exists for that failure and this is the same test for one more
//!    peripheral.
//!
//! Sources: ST **RM0090** rev 21 §24 for the RNG and Table 62 for the vector.

#![cfg(feature = "machine-stm32f407")]

use rsemu::core::clock::GlobalTime;
use rsemu::core::space::MemAttrs;
use rsemu::core::value::Width;
use rsemu::machine::{Machine, catalog};

/// The RNG's base: AHB2 + 0x60800 (RM0090 Table 1).
const RNG: u64 = 0x5006_0800;
/// `RNG_CR`.
const CR: u64 = RNG;
/// `RNG_SR`.
const SR: u64 = RNG + 4;
/// `RNG_DR`.
const DR: u64 = RNG + 8;

/// `CR.RNGEN`.
const RNGEN: u64 = 1 << 2;
/// `CR.IE`.
const IE: u64 = 1 << 3;
/// `SR.DRDY`.
const DRDY: u64 = 1 << 0;

/// Where SRAM1 starts.
const SRAM1: u64 = 0x2000_0000;
/// The initial stack pointer: the top of SRAM2.
const STACK: u32 = 0x2002_0000;
/// Where the program's entry point is.
const ENTRY: u32 = 0x100;
/// Where the `HASH_RNG` handler is.
const HANDLER: u32 = 0x200;

/// `HASH_RNG` is position **80** in RM0090 Table 62 — "Hash and Rng global
/// interrupt", vector address 0x0000_0180. External interrupt *n* is exception
/// *n + 16*, so the handler runs as exception 96 and its vector sits at
/// 4 × 96 = 0x180, which is what the table prints.
const HASH_RNG_EXCEPTION: u64 = 96;

// ---------------------------------------------------------------------------
// The same very small Thumb-2 assembler `stm32f407_wiring.rs` uses
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

/// `LDR Rt, [Rn, #imm5*4]` — encoding T1, A7.7.42.
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

/// `MRS Rd, IPSR` — encoding T1, A7.7.82.
///
/// `SYSm` 5 is `IPSR` (DDI 0403 B5.2.3), which reads the exception number the
/// processor is *currently executing*. That is the whole point of the vector
/// test below.
fn mrs_ipsr(d: u16) -> [u16; 2] {
    [0xf3ef, 0x8005 | (d << 8)]
}

/// `BX LR` — encoding T1, A7.7.20. In a handler this is an exception return.
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

/// A vector table, a program that arms the RNG's interrupt, and a handler that
/// records what it is executing as and takes the word.
///
/// ```text
/// main:      NVIC_ISER2 = 1 << 16      ; interrupt 80 is ISER2 bit 16
///            RNG_CR     = RNGEN | IE
///            b .
///
/// handler:   NVIC_ICER2 = 1 << 16      ; see below: once is what is wanted
///            SRAM1[0]   = IPSR         ; the number this test is about
///            SRAM1[8]   = RNG_SR       ; which flag brought us here
///            SRAM1[4]   = RNG_DR       ; and the word, which clears DRDY
///            RNG_CR     = 0
///            bx lr
/// ```
///
/// The handler disables its own NVIC line first, and that is not tidiness. The
/// RNG's request is a **level**: it is still high when the core enters the
/// handler, because nothing has read `DR` yet, so the NVIC re-pends it on
/// entry and the handler runs a second time after it returns. That second run
/// reads `DR` with `DRDY` already clear and gets zero — which is exactly what
/// the silicon does, and which would overwrite the word this test wants to
/// look at. Real firmware does not care, because it takes the word the first
/// time.
fn firmware() -> Vec<u8> {
    let mut main: Vec<u16> = Vec::new();
    // Interrupt 80 is bit 16 of `NVIC_ISER2`, at 0xE000E108.
    main.extend(load32(0, 0xe000_e108));
    main.extend(load32(1, 1 << 16));
    main.push(str_imm(1, 0, 0x00));
    main.extend(load32(0, CR as u32));
    main.push(movs(1, (RNGEN | IE) as u16));
    main.push(str_imm(1, 0, 0x00));
    main.push(0xe7fe); // b .

    let mut handler: Vec<u16> = Vec::new();
    // `NVIC_ICER2` is at 0xE000E188.
    handler.extend(load32(0, 0xe000_e188));
    handler.extend(load32(1, 1 << 16));
    handler.push(str_imm(1, 0, 0x00));
    handler.extend(mrs_ipsr(3));
    handler.extend(load32(2, SRAM1 as u32));
    handler.push(str_imm(3, 2, 0x00));
    handler.extend(load32(0, RNG as u32));
    handler.push(ldr_imm(1, 0, 0x04)); // SR, before anything is taken
    handler.push(str_imm(1, 2, 0x08));
    handler.push(ldr_imm(1, 0, 0x08)); // DR, which is what drops the request
    handler.push(str_imm(1, 2, 0x04));
    handler.push(movs(1, 0));
    handler.push(str_imm(1, 0, 0x00)); // CR = 0: one word is enough
    handler.push(bx_lr());

    let mut image = vec![0u8; 0x300];
    fn word(image: &mut [u8], at: usize, value: u32) {
        image[at..at + 4].copy_from_slice(&value.to_le_bytes());
    }
    word(&mut image, 0x00, STACK);
    word(&mut image, 0x04, ENTRY | 1);
    word(&mut image, 4 * HASH_RNG_EXCEPTION as usize, HANDLER | 1);

    for (at, code) in [(ENTRY, &main), (HANDLER, &handler)] {
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

/// Build the board with `image` in its `firmware` slot and `params` overridden.
fn boot_with(image: Vec<u8>, params: &[(&str, &str)]) -> Machine {
    let entry = catalog::machine("stm32f407").expect("this build ships stm32f407");
    let mut options = catalog::build_options().expect("the catalog agrees with itself");
    options.realize.media.insert("firmware", image);
    options.resolve.params = params
        .iter()
        .map(|(k, v)| ((*k).to_string(), (*v).to_string()))
        .collect();
    let registry = catalog::registry().expect("a registry");
    match rsemu::machine::build(entry.name, entry.source, &registry, &options) {
        Ok(m) => m,
        Err(e) => panic!("the board does not realize: {e}"),
    }
}

/// A board with nothing in its firmware slot, driven over the bus.
fn boot(params: &[(&str, &str)]) -> Machine {
    boot_with(vec![0u8; 0x100], params)
}

fn load(m: &Machine, addr: u64) -> u64 {
    m.space("mem")
        .expect("the memory space")
        .read(addr, Width::U32, MemAttrs::DEFAULT)
        .expect("a mapped word")
}

fn peek(m: &Machine, addr: u64) -> u64 {
    m.space("mem")
        .expect("the memory space")
        .read(addr, Width::U32, MemAttrs::DEBUG)
        .expect("a mapped word")
}

fn store(m: &Machine, addr: u64, value: u64) {
    m.space("mem")
        .expect("the memory space")
        .write(addr, Width::U32, value, MemAttrs::DEFAULT)
        .expect("a mapped word");
}

/// Enable the block and take `n` words the way a polling driver does, letting
/// the machine run between polls so the scheduler advances the device.
fn draw(m: &mut Machine, n: usize) -> Vec<u64> {
    store(m, CR, RNGEN);
    let mut out = Vec::new();
    while out.len() < n {
        if load(m, SR) & DRDY == 0 {
            m.run_for(GlobalTime::from_nanos(1_000)).expect("it runs");
            continue;
        }
        out.push(load(m, DR));
    }
    out
}

#[test]
fn the_rng_decodes_where_rm0090_table_1_puts_it() {
    let m = boot(&[]);
    assert!(m.device("rng").is_some(), "no instance called `rng`");
    for space in ["mem", "dmabus"] {
        // A DMA master reaches it too: `RNG_DR` is a DMA source on the part.
        let view = m.space(space).expect("the space");
        assert!(
            view.read(RNG, Width::U32, MemAttrs::DEBUG).is_ok(),
            "nothing decodes the RNG's base in `{space}`"
        );
        assert!(
            view.read(RNG + 0x100, Width::U32, MemAttrs::DEBUG).is_err(),
            "the RNG answered above its twelve bytes in `{space}`"
        );
    }
}

#[test]
fn two_runs_of_the_same_board_deal_the_same_words_and_hash_the_same() {
    // The claim `ROADMAP.md` §0 makes about every machine, asserted on the one
    // device whose output is supposed to look like noise.
    let mut first = boot(&[]);
    let mut second = boot(&[]);

    let a = draw(&mut first, 16);
    let b = draw(&mut second, 16);
    assert_eq!(a, b, "two runs of one board dealt different numbers");

    // Byte-identical, not merely equal word by word: the state hash covers the
    // generator's position as well as what came out of it.
    assert_eq!(
        first.state_hash().expect("a hash"),
        second.state_hash().expect("a hash"),
        "two runs of one board reached different states"
    );
    assert_eq!(
        first.save().expect("a snapshot"),
        second.save().expect("a snapshot")
    );
}

#[test]
fn a_different_seed_is_a_different_board() {
    // The machine-level knob. One number in the board file covers every device
    // that needs entropy, and a run overrides it without editing anything.
    let mut ones = boot(&[]);
    let mut sevens = boot(&[("seed", "7")]);
    assert_ne!(draw(&mut ones, 8), draw(&mut sevens, 8));
    // Two boards at seed 7, on the other hand, agree.
    let mut also_sevens = boot(&[("seed", "7")]);
    assert_eq!(
        draw(&mut also_sevens, 8),
        draw(&mut boot(&[("seed", "7")]), 8)
    );
}

#[test]
fn a_snapshot_resumes_the_stream_rather_than_replaying_it() {
    // The failure this catches is a snapshot that carries the seed and not the
    // position: it restores, and hands the guest numbers it has already had.
    let mut saved = boot(&[]);
    let first = draw(&mut saved, 4);
    let bytes = saved.save().expect("the machine snapshots");
    let next = draw(&mut saved, 4);
    assert_ne!(first, next);

    let mut restored = boot(&[]);
    restored.load(&bytes).expect("the snapshot loads");
    assert_eq!(draw(&mut restored, 4), next, "the stream resumed");
}

#[test]
fn a_debugger_reading_dr_does_not_change_what_the_guest_gets() {
    // `ROADMAP.md` §15's invariant 5, at its sharpest. A memory-window refresh
    // that consumed a word would change the guest's numbers and the guest
    // could not tell.
    let mut watched = boot(&[]);
    store(&watched, CR, RNGEN);
    while load(&watched, SR) & DRDY == 0 {
        watched
            .run_for(GlobalTime::from_nanos(1_000))
            .expect("it runs");
    }
    for _ in 0..8 {
        peek(&watched, DR);
        assert_eq!(peek(&watched, SR) & DRDY, DRDY, "a debug read cleared DRDY");
    }
    let mut rest = vec![load(&watched, DR)];
    rest.extend(draw(&mut watched, 3));

    let mut alone = boot(&[]);
    assert_eq!(draw(&mut alone, 4), rest, "the debugger moved the stream");
}

#[test]
fn the_rngs_interrupt_reaches_the_handler_table_62_gives_it() {
    // The failure no unit test catches: a peripheral wired to the wrong
    // `cpu.irq{n}` still pends, the core still takes an exception, and the only
    // symptom is that somebody else's handler runs. The handler asks its own
    // `IPSR` what it is, and 80 has to come back as 96.
    let mut m = boot_with(firmware(), &[]);
    m.run_for(GlobalTime::from_nanos(20_000_000))
        .expect("it runs");
    assert_eq!(
        peek(&m, SRAM1),
        HASH_RNG_EXCEPTION,
        "the RNG's interrupt did not arrive as exception 96 (RM0090 Table 62, \
         position 80)"
    );
    // And the handler took a real word rather than the zero a `DRDY`-clear
    // read gives, so the interrupt was `DRDY` and not something else.
    // And it was `DRDY` that brought it there, with a real word behind it —
    // not a stray level from something else on the same net.
    assert_eq!(peek(&m, SRAM1 + 8) & DRDY, DRDY);
    assert_ne!(peek(&m, SRAM1 + 4), 0);
}
