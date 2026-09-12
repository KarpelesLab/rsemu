//! The STM32 hash processor on a board: where it decodes, which variant it was
//! built with, and — the substance — that vector 80 is **shared**.
//!
//! `src/dev/stm32/hash.rs` proves the register face and every published digest
//! one device at a time. Three things are only true of a machine, and
//! `machines/tests/stm32f4-hash.machine` is the smallest board on which they can
//! be said:
//!
//! 1. **It is at `0x5006_0400` and stops after `HR7`.** RM0090 Table 1 puts
//!    HASH one kilobyte below the RNG. An address is a `map` statement, and a
//!    `map` statement is only ever wrong on a board.
//! 2. **`variant = "f4"` reached the device.** An F4's `CR` has no `ALGO[1]`,
//!    so asking that board for SHA-256 gets MD5 — which is a strange-looking
//!    assertion until you notice it is the only way to tell from outside that
//!    the construction property was applied at all.
//! 3. **`wire hash.irq -> cpu.irq80` lands on exception 96, alongside the
//!    RNG.** RM0090 Table 62 calls position 80 `HASH_RNG` and means it: two
//!    peripherals, one pin, one handler. A wrong vector does not fail loudly —
//!    somebody else's handler runs — so the handler is asked what it is
//!    executing as, exactly as `tests/stm32f407_rng.rs` asks for the other
//!    half of the same line on the board that really has an RNG.
//!
//! # Why this is not on `machines/stm32f407.machine`
//!
//! An STM32F407 has no hash processor. RM0090 §25 applies to the F415/417 and
//! F43x dies, and DS8626 lists no cryptographic acceleration on an F407VG, so
//! putting the block on the shipped board would have made its "A real part, not
//! a synthesis" header false. The test board carries it instead, and promises
//! nothing.
//!
//! Sources: ST **RM0090** rev 21 §25 for the block, §24 for the RNG, Table 1 for
//! both addresses and Table 62 for the vector; FIPS 180-4 Appendix A and RFC
//! 1321 for the digests. The ARMv7-M Architecture Reference Manual, ARM DDI
//! 0403, for the handful of Thumb encodings below.

#![cfg(all(
    feature = "cpu-arm-v7m",
    feature = "dev-stm32-hash",
    feature = "dev-stm32-rng"
))]

use rsemu::core::clock::GlobalTime;
use rsemu::core::space::MemAttrs;
use rsemu::core::value::Width;
use rsemu::machine::{Machine, catalog};

/// The board, which is not in the catalog and is included by path.
const BOARD: &str = include_str!("../machines/tests/stm32f4-hash.machine");

/// The hash processor's base: AHB2 + 0x60400 (RM0090 Table 1).
const HASH: u64 = 0x5006_0400;
const CR: u64 = HASH;
const DIN: u64 = HASH + 0x04;
const STR: u64 = HASH + 0x08;
const HR0: u64 = HASH + 0x0c;
const SR: u64 = HASH + 0x24;

/// The RNG's base, one kilobyte above — the other half of vector 80.
const RNG: u64 = 0x5006_0800;

const CR_INIT: u64 = 1 << 2;
const CR_DATATYPE_BYTE: u64 = 0b10 << 4;
const CR_ALGO0: u64 = 1 << 7;
const CR_ALGO1: u64 = 1 << 18;
const STR_DCAL: u64 = 1 << 8;
const IMR_DCIE: u64 = 1 << 1;
const SR_DCIS: u64 = 1 << 1;

/// Where the board's SRAM starts.
const SRAM: u64 = 0x2000_0000;
/// The initial stack pointer: the top of the board's sixteen kilobytes.
const STACK: u32 = 0x2000_4000;
/// Where the program's entry point is.
const ENTRY: u32 = 0x100;
/// Where the `HASH_RNG` handler is.
const HANDLER: u32 = 0x200;

/// `HASH_RNG` is position **80** in RM0090 Table 62, so the handler runs as
/// exception 80 + 16 = 96 and its vector sits at 4 × 96 = 0x180.
const HASH_RNG_EXCEPTION: u64 = 96;

// ---------------------------------------------------------------------------
// The same very small Thumb-2 assembler `stm32f407_rng.rs` uses
// ---------------------------------------------------------------------------
//
// Copied rather than shared, because a `tests/` file is its own crate and the
// alternative is a `tests/common/` module for seven encodings. Each one is
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

/// `MRS Rd, IPSR` — encoding T1, A7.7.82. `SYSm` 5 is `IPSR` (DDI 0403
/// B5.2.3), which reads the exception the processor is *currently executing*.
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

/// A vector table, a program that hashes `"abc"` with the completion interrupt
/// armed, and a handler that records what it is executing as.
///
/// ```text
/// main:      NVIC_ISER2 = 1 << 16      ; interrupt 80 is ISER2 bit 16
///            HASH_IMR   = DCIE
///            HASH_CR    = INIT | DATATYPE(byte)   ; SHA-1, hash mode
///            HASH_DIN   = 0x00636261              ; "abc", little-endian
///            HASH_STR   = 24 | DCAL               ; three valid bytes
///            b .
///
/// handler:   NVIC_ICER2 = 1 << 16      ; the request is a level; drop the line
///            SRAM[0]    = IPSR         ; the number this test is about
///            SRAM[4]    = HASH_SR      ; which flag brought us here
///            SRAM[8]    = HASH_HR0     ; and the digest behind it
///            SRAM[12]   = RNG_SR       ; the other driver of the same pin
///            HASH_IMR   = 0            ; which is what drops the request
///            bx lr
/// ```
///
/// The handler masks its own NVIC line on entry for the same reason the RNG's
/// does: `DCIS` is still set when the core arrives, so the level is still high
/// and the NVIC would re-pend it the moment the handler returned.
fn firmware() -> Vec<u8> {
    let mut main: Vec<u16> = Vec::new();
    main.extend(load32(0, 0xe000_e108)); // NVIC_ISER2
    main.extend(load32(1, 1 << 16));
    main.push(str_imm(1, 0, 0x00));
    main.extend(load32(0, HASH as u32));
    main.push(movs(1, IMR_DCIE as u16));
    main.push(str_imm(1, 0, 0x20));
    main.push(movs(1, (CR_INIT | CR_DATATYPE_BYTE) as u16));
    main.push(str_imm(1, 0, 0x00));
    main.extend(load32(1, 0x0063_6261)); // "abc" as a little-endian word
    main.push(str_imm(1, 0, 0x04));
    main.extend(load32(1, (24 | STR_DCAL) as u32));
    main.push(str_imm(1, 0, 0x08));
    main.push(0xe7fe); // b .

    let mut handler: Vec<u16> = Vec::new();
    handler.extend(load32(0, 0xe000_e188)); // NVIC_ICER2
    handler.extend(load32(1, 1 << 16));
    handler.push(str_imm(1, 0, 0x00));
    handler.extend(mrs_ipsr(3));
    handler.extend(load32(2, SRAM as u32));
    handler.push(str_imm(3, 2, 0x00));
    handler.extend(load32(0, HASH as u32));
    handler.push(ldr_imm(1, 0, 0x24)); // HASH_SR
    handler.push(str_imm(1, 2, 0x04));
    handler.push(ldr_imm(1, 0, 0x0c)); // HASH_HR0
    handler.push(str_imm(1, 2, 0x08));
    handler.extend(load32(0, RNG as u32));
    handler.push(ldr_imm(1, 0, 0x04)); // RNG_SR
    handler.push(str_imm(1, 2, 0x0c));
    handler.extend(load32(0, HASH as u32));
    handler.push(movs(1, 0));
    handler.push(str_imm(1, 0, 0x20)); // HASH_IMR = 0
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

/// Build the board with `image` in its `firmware` slot.
fn boot_with(image: Vec<u8>) -> Machine {
    let mut options = catalog::build_options().expect("the catalog agrees with itself");
    options.realize.media.insert("firmware", image);
    let registry = catalog::registry().expect("a registry");
    match rsemu::machine::build("stm32f4-hash", BOARD, &registry, &options) {
        Ok(m) => m,
        Err(e) => panic!("the board does not realize: {e}"),
    }
}

/// A board with nothing to execute, driven over the bus.
fn boot() -> Machine {
    boot_with(vec![0u8; 0x100])
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

/// Push a message through `DIN` the way a driver with a byte buffer does: a
/// little-endian word per four bytes, over-reading the tail, with `NBLW`
/// counting the valid bits of the last one.
fn feed(m: &Machine, msg: &[u8]) -> u64 {
    for chunk in msg.chunks(4) {
        let mut w = [0u8; 4];
        w[..chunk.len()].copy_from_slice(chunk);
        store(m, DIN, u64::from(u32::from_le_bytes(w)));
    }
    ((msg.len() % 4) * 8) as u64
}

fn digest(m: &Machine, words: u64) -> Vec<u32> {
    (0..words).map(|i| load(m, HR0 + i * 4) as u32).collect()
}

#[test]
fn the_hash_decodes_where_rm0090_table_1_puts_it() {
    let m = boot();
    assert!(m.device("hash").is_some(), "no instance called `hash`");
    let view = m.space("mem").expect("the memory space");
    assert!(
        view.read(HASH, Width::U32, MemAttrs::DEBUG).is_ok(),
        "nothing decodes the hash processor's base"
    );
    assert!(
        view.read(HASH + 0x32c, Width::U32, MemAttrs::DEBUG).is_ok(),
        "`HR7` is outside the window"
    );
    // 0x330 is the first word past `HR7` and still inside the block's own
    // kilobyte, so a fault there is the aperture ending rather than the next
    // peripheral beginning. 0x400 would be the RNG.
    assert!(
        view.read(HASH + 0x330, Width::U32, MemAttrs::DEBUG)
            .is_err(),
        "the hash processor answered past its aperture"
    );
    // And it did not eat the kilobyte the RNG lives in.
    assert!(view.read(RNG, Width::U32, MemAttrs::DEBUG).is_ok());
}

#[test]
fn a_sha1_digest_driven_over_the_bus_matches_fips_180_4() {
    let m = boot();
    store(&m, CR, CR_INIT | CR_DATATYPE_BYTE);
    let nblw = feed(&m, b"abc");
    store(&m, STR, nblw | STR_DCAL);
    assert_eq!(load(&m, SR) & SR_DCIS, SR_DCIS);
    assert_eq!(
        digest(&m, 5),
        [
            0xa999_3e36,
            0x4706_816a,
            0xba3e_2571,
            0x7850_c26c,
            0x9cd0_d89d
        ]
    );
}

#[test]
fn the_board_asked_for_the_f4_variant() {
    // The only externally visible consequence of `variant = "f4"`: `ALGO[1]`
    // at bit 18 is not decoded, so `ALGO = 11` (SHA-256) reads back as
    // `ALGO = 01` and the block computes MD5. That is what an F4 does, and a
    // board that had silently instantiated the wide block would answer with
    // the SHA-256 of "abc" instead.
    let m = boot();
    store(&m, CR, CR_INIT | CR_DATATYPE_BYTE | CR_ALGO0 | CR_ALGO1);
    assert_eq!(load(&m, CR) & CR_ALGO1, 0, "bit 18 came back set");
    let nblw = feed(&m, b"abc");
    store(&m, STR, nblw | STR_DCAL);
    assert_eq!(
        digest(&m, 4),
        [0x9001_5098, 0x3cd2_4fb0, 0xd696_3f7d, 0x28e1_7f72],
        "MD5 of \"abc\", RFC 1321 Appendix A.5"
    );
}

#[test]
fn a_snapshot_of_a_half_finished_digest_resumes_it() {
    // The machine-level version of the context claim: the block is mid-message
    // when the snapshot is taken, and the restored board finishes the same
    // digest rather than one of whatever is left.
    let msg = [b'a'; 200];
    let saved = boot();
    store(&saved, CR, CR_INIT | CR_DATATYPE_BYTE);
    feed(&saved, &msg[..100]);
    let bytes = saved.save().expect("the machine snapshots");

    let mut restored = boot();
    restored.load(&bytes).expect("the snapshot loads");
    let nblw = feed(&restored, &msg[100..]);
    store(&restored, STR, nblw | STR_DCAL);
    assert_eq!(
        digest(&restored, 5),
        [
            0xe61c_fffe,
            0x0d91_95a5,
            0x25fc_6cf0,
            0x6ca2_d771,
            0x19c2_4a40
        ],
        "SHA-1 of two hundred 'a's"
    );
}

#[test]
fn the_hash_shares_vector_80_with_the_rng() {
    // RM0090 Table 62's position 80 is `HASH_RNG` and both peripherals on this
    // board drive it. `tests/stm32f407_rng.rs` asserts the RNG half on the
    // shipped board; this is the other one, and between them they say the line
    // is shared rather than merely named that way in a comment.
    let mut m = boot_with(firmware());
    m.run_for(GlobalTime::from_nanos(20_000_000))
        .expect("it runs");
    assert_eq!(
        peek(&m, SRAM),
        HASH_RNG_EXCEPTION,
        "the hash processor's interrupt did not arrive as exception 96 \
         (RM0090 Table 62, position 80)"
    );
    // It was `DCIS` that brought it there, with the right digest behind it --
    // not a stray level from the other device on the same net, whose `SR` the
    // handler also recorded and which has nothing set.
    assert_eq!(peek(&m, SRAM + 4) & SR_DCIS, SR_DCIS);
    assert_eq!(peek(&m, SRAM + 8), 0xa999_3e36);
    assert_eq!(peek(&m, SRAM + 12) & 1, 0, "the RNG had a word ready too");
}
