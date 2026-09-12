//! A DfuSe firmware onto a board, end to end.
//!
//! `src/dev/dfuse.rs` has the parser's own tests: what a truncated file, a
//! wrong CRC and an overlapping element do. This says the stronger thing that
//! only a machine can say.
//!
//! A `.dfu` file is **built in the test** — `CLAUDE.md` is explicit that
//! corpora are downloaded rather than vendored, and a DfuSe container is a
//! prefix, a target header and a CRC, so there is nothing to vendor — bound to
//! a media slot exactly as `rsemu run … --media firmware=fw.dfu` binds one, and
//! handed to a board that names a `dfu.loader`. Its three elements then land in
//! three different places: two in the on-chip flash array, through the flash
//! controller's own programming door, and one in SRAM. And the Cortex-M4 boots
//! out of the alias at zero and runs the code that arrived in element 1.
//!
//! That last part is the point. Nothing in the media pipeline carries an
//! address: `Media` is `{name, bytes}` and `MediaTable` binds one flat blob per
//! slot. A firmware that says where its own pieces go could not be expressed at
//! all before this device, and "it parsed" would not prove that it can be now.

#![cfg(all(
    feature = "dev-dfuse",
    feature = "cpu-arm-v7m",
    feature = "dev-stm32-flash"
))]

use rsemu::core::clock::GlobalTime;
use rsemu::core::space::MemAttrs;
use rsemu::core::value::Width;
use rsemu::dev::dfuse::{self, Image};
use rsemu::machine::{Machine, catalog};

const BOARD: &str = include_str!("../machines/tests/dfuse-loader.machine");

/// ST's own USB IDs, which is what every STM32 `.dfu` in circulation carries:
/// the DFU interface an STM32 bootloader exposes is 0483:df11.
const VENDOR: u16 = 0x0483;
/// The product half of the same pair.
const PRODUCT: u16 = 0xdf11;

/// Where the flash array lives, and where element 0 goes.
const FLASH: u64 = 0x0800_0000;
/// Where element 1 goes: the code.
const CODE: u64 = 0x0800_0100;
/// Where element 2 goes, in a different region entirely.
const SRAM_WORD: u64 = 0x2000_0010;

/// The word the guest stores, and the word element 2 carries.
const MARKER: u32 = 0x5678_1234;
/// Where the guest stores it.
const RESULT: u64 = 0x2000_0000;

/// The initial stack pointer: the top of the 32 KiB SRAM this board has.
const STACK: u32 = 0x2000_8000;

// ---------------------------------------------------------------------------
// Four Thumb-2 encodings
// ---------------------------------------------------------------------------
//
// The same four `tests/stm32f407_board.rs` uses, and for the same reason: the
// crate has no assembler, and a table of four encodings checked by hand against
// the ARMv7-M ARM (DDI 0403, A7.7) is not one.

/// `MOVW Rd, #imm16` — encoding T3, A7.7.76. `imm16` is `imm4:i:imm3:imm8`.
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
    0x6000 | ((off / 4) << 6) | (n << 3) | t
}

/// `B .` — encoding T2, A7.7.12. The displacement is from `PC`, which in Thumb
/// is the instruction's address plus four, so branching to itself is -4.
fn branch_to_self() -> u16 {
    0xe000 | ((-2i16 as u16) & 0x7ff)
}

/// The program: put [`MARKER`] at [`RESULT`], then spin.
fn code() -> Vec<u8> {
    let mut words: Vec<u16> = Vec::new();
    words.extend_from_slice(&movw(0, MARKER as u16));
    words.extend_from_slice(&movt(0, (MARKER >> 16) as u16));
    words.extend_from_slice(&movw(1, RESULT as u16));
    words.extend_from_slice(&movt(1, (RESULT >> 16) as u16));
    words.push(str_imm(0, 1, 0));
    words.push(branch_to_self());
    words.iter().flat_map(|w| w.to_le_bytes()).collect()
}

/// The eight bytes at the bottom of the vector table: `SP`, then the reset
/// vector with the Thumb bit set (DDI 0403, B1.5.5).
fn vectors() -> Vec<u8> {
    let mut out = Vec::new();
    out.extend_from_slice(&STACK.to_le_bytes());
    out.extend_from_slice(&((CODE as u32) | 1).to_le_bytes());
    out
}

/// The `.dfu` the test flashes: one target, three elements, three destinations.
fn firmware() -> Vec<u8> {
    let vectors = vectors();
    let code = code();
    let marker = MARKER.to_le_bytes();
    dfuse::build(
        VENDOR,
        PRODUCT,
        &[(
            0,
            Some("Internal Flash"),
            &[
                (FLASH as u32, &vectors),
                (CODE as u32, &code),
                (SRAM_WORD as u32, &marker),
            ],
        )],
    )
}

/// Build the board with `image` in its `firmware` slot.
fn board(image: Vec<u8>) -> Result<Machine, String> {
    let mut options = catalog::build_options().expect("the catalog agrees with itself");
    options.realize.media.insert("firmware", image);
    let registry = catalog::registry().expect("a registry");
    rsemu::machine::build("dfuse-loader", BOARD, &registry, &options).map_err(|e| e.to_string())
}

/// Read one word of the guest's memory space, without side effects.
fn peek(m: &Machine, addr: u64) -> u64 {
    m.space("mem")
        .expect("the memory space")
        .read(addr, Width::U32, MemAttrs::DEBUG)
        .expect("a mapped word")
}

#[test]
fn every_element_lands_at_the_address_the_file_gave_it() {
    let m = board(firmware()).expect("the board realizes");

    // Element 0, into the flash array — and visible through the boot alias at
    // zero as well, because it is the same array mapped twice.
    assert_eq!(peek(&m, FLASH), u64::from(STACK));
    assert_eq!(peek(&m, FLASH + 4), u64::from(CODE as u32 | 1));
    assert_eq!(peek(&m, 0), u64::from(STACK), "the boot alias sees it too");

    // Element 1, 256 bytes further up the same region.
    let first = code();
    assert_eq!(
        peek(&m, CODE),
        u64::from(u32::from_le_bytes([first[0], first[1], first[2], first[3]])),
    );

    // The gap between two elements is untouched, which on a flash array means
    // erased. A zero-filled array could not tell "not written" from "written
    // zero", which is why this board gives its flash no `image =`.
    assert_eq!(
        peek(&m, FLASH + 8),
        0xffff_ffff,
        "erased flash between them"
    );

    // Element 2, in a different region entirely — the thing a single-address
    // loader cannot do.
    assert_eq!(peek(&m, SRAM_WORD), u64::from(MARKER));
}

#[test]
fn the_guest_executes_the_code_that_arrived_in_an_element() {
    let mut m = board(firmware()).expect("the board realizes");
    assert_eq!(peek(&m, RESULT), 0, "nothing has run yet");
    m.run_for(GlobalTime::from_nanos(1_000_000))
        .expect("the core runs");
    assert_eq!(
        peek(&m, RESULT),
        u64::from(MARKER),
        "the core fetched out of the boot alias and ran what element 1 put in the flash"
    );
}

#[test]
fn a_firmware_aimed_at_a_part_this_board_is_not_fails_the_build_naming_the_address() {
    // 0x90000000 is an F4's FSMC/QSPI window, which this board does not have.
    // The bus would not have complained — an unmapped write on a board with an
    // open-bus policy is simply dropped — so the loader has to.
    let vectors = vectors();
    let file = dfuse::build(
        VENDOR,
        PRODUCT,
        &[(
            0,
            Some("Internal Flash"),
            &[(FLASH as u32, &vectors), (0x9000_0000, &[0xaa; 16])],
        )],
    );
    let e = board(file).expect_err("nothing is mapped at 0x90000000");
    assert!(e.contains("90000000"), "{e}");
    assert!(e.contains("element 1"), "{e}");
    assert!(e.contains("0483:df11"), "and which part it is for: {e}");
}

#[test]
fn a_damaged_download_fails_the_build_rather_than_loading_half_of_it() {
    // The CRC first: one flipped bit anywhere in the file.
    let mut file = firmware();
    file[40] ^= 0x01;
    let e = board(file).expect_err("the CRC no longer matches");
    assert!(e.contains("CRC"), "{e}");
    assert!(e.contains("verify-crc"), "{e}");

    // And a truncation, which is what an interrupted download looks like.
    let file = firmware();
    let e = board(file[..file.len() - 100].to_vec()).expect_err("short");
    assert!(e.contains("truncated") || e.contains("DFUImageSize"), "{e}");

    // Neither of those may be a panic, and nothing may have been written: the
    // machine never got built at all.
}

#[test]
fn a_raw_bin_in_the_slot_is_refused_with_a_message_that_says_what_it_is() {
    // The mistake somebody makes once: `--media firmware=blink.bin` against a
    // board whose firmware object is a DfuSe loader.
    let e = board(code()).expect_err("a .bin is not a .dfu");
    assert!(e.contains("DfuSe"), "{e}");
}

#[test]
fn the_file_the_test_builds_is_one_a_flasher_would_accept() {
    // `build` is used by the tests and by the fuzz target's seed, so its output
    // being a *valid* DfuSe file is load-bearing: the suffix's own CRC has to
    // check out under the convention the module documents, and the prefix's
    // DFUImageSize has to match the bytes.
    let file = firmware();
    let image = Image::parse(&file, true).expect("valid by construction");
    assert_eq!(image.suffix.id_vendor, VENDOR);
    assert_eq!(image.suffix.id_product, PRODUCT);
    assert_eq!(image.targets.len(), 1);
    assert_eq!(image.targets[0].elements.len(), 3);
    let stated = u32::from_le_bytes([
        file[file.len() - 4],
        file[file.len() - 3],
        file[file.len() - 2],
        file[file.len() - 1],
    ]);
    assert_eq!(dfuse::crc32(&file[..file.len() - 4]), stated);
}
