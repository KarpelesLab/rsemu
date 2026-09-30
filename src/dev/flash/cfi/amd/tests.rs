//! The AMD command set, against the S29JL064J datasheet's own tables.

use super::super::*;
use crate::core::props::{Media, Value};
use crate::core::state::{MachineShape, Migrations, StateReader, StateWriter};

const K: u64 = 1024;

/// The S29JL064J's layout (Table 3): eight 8 KiB sectors, 126 of 64 KiB,
/// eight more of 8 KiB — 8 MiB — as the machine-file list writes it.
fn jl064j_blocks() -> Value {
    Value::List(alloc::vec![
        Value::Uint(8),
        Value::Size(8 * K),
        Value::Uint(126),
        Value::Size(64 * K),
        Value::Uint(8),
        Value::Size(8 * K),
    ])
}

/// The properties a board writes for one S29JL064J wired x16 on a 16-bit bus.
fn jl064j_props() -> Props {
    Props::new()
        .with("size", Value::Size(8 * K * K))
        .with("width", 2u64)
        .with("interleave", 1u64)
        .with("command-set", "amd")
        .with("blocks", jl064j_blocks())
        .with("device", 0x227eu64)
        .with(
            "device-ext",
            Value::List(alloc::vec![Value::Uint(0x2202), Value::Uint(0x2201)]),
        )
        .with(
            "banks",
            Value::List(alloc::vec![
                Value::Uint(23),
                Value::Uint(48),
                Value::Uint(48),
                Value::Uint(23),
            ]),
        )
        .with("boot-flag", 1u64)
}

fn jl064j() -> Cfi {
    Cfi::new(&jl064j_props()).expect("an S29JL064J")
}

/// A 16-bit bus cycle at device **word** address `word`: the CPU's byte
/// address is twice it, because A1 drives the part's A0.
fn w16(cfi: &Cfi, word: u64, value: u16) {
    cfi.array()
        .write(word * 2, &value.to_le_bytes(), MemAttrs::DEFAULT)
        .expect("a halfword write is a legal bus cycle");
}

fn r16(cfi: &Cfi, word: u64) -> u16 {
    let mut b = [0u8; 2];
    cfi.array()
        .read(word * 2, &mut b, MemAttrs::DEFAULT)
        .expect("a halfword read");
    u16::from_le_bytes(b)
}

fn unlock(cfi: &Cfi) {
    w16(cfi, 0x555, 0xaa);
    w16(cfi, 0x2aa, 0x55);
}

fn program(cfi: &Cfi, word: u64, value: u16) {
    unlock(cfi);
    w16(cfi, 0x555, 0xa0);
    w16(cfi, word, value);
}

fn sector_erase(cfi: &Cfi, word: u64) {
    unlock(cfi);
    w16(cfi, 0x555, 0x80);
    unlock(cfi);
    w16(cfi, word, 0x30);
}

fn reset(cfi: &Cfi) {
    w16(cfi, 0, 0xf0);
}

#[test]
fn autoselect_reports_the_datasheets_codes() {
    let cfi = jl064j();
    unlock(&cfi);
    w16(&cfi, 0x555, 0x90);
    // §8.9's table, word mode: manufacturer 01h at X00; the device ID across
    // X01, X0E, X0F with 22h on DQ15-8; protection at X02; the Secured
    // Silicon indicator at X03.
    assert_eq!(r16(&cfi, 0x00), 0x0001);
    assert_eq!(r16(&cfi, 0x01), 0x227e);
    assert_eq!(r16(&cfi, 0x0e), 0x2202);
    assert_eq!(r16(&cfi, 0x0f), 0x2201);
    assert_eq!(r16(&cfi, 0x02), 0x0000, "sector 0 unprotected");
    assert_eq!(
        r16(&cfi, 0x03),
        0x0001,
        "neither factory nor customer locked"
    );
    // At a bank address, the same codes (bank 4 is A21-A19 = 111).
    assert_eq!(r16(&cfi, 0x38_0000 + 0x01), 0x227e);
    assert!(
        !cfi.array().is_reading_array(),
        "reads are not the array now"
    );
    // The reset command, and the array is back.
    reset(&cfi);
    assert_eq!(r16(&cfi, 0x00), 0xffff);
    assert!(cfi.array().is_reading_array(), "and the fast path with it");
}

#[test]
fn the_query_is_the_datasheets_tables_8_to_11_byte_for_byte() {
    let cfi = jl064j();
    w16(&cfi, 0x55, 0x98);
    // Every word-mode entry of Tables 8-11, 10h to 5Bh, zero where the
    // tables list nothing (51h-56h).
    #[rustfmt::skip]
    let expected: [(u64, u16); 76] = [
        (0x10, 0x51), (0x11, 0x52), (0x12, 0x59), (0x13, 0x02), (0x14, 0x00),
        (0x15, 0x40), (0x16, 0x00), (0x17, 0x00), (0x18, 0x00), (0x19, 0x00),
        (0x1a, 0x00), (0x1b, 0x27), (0x1c, 0x36), (0x1d, 0x00), (0x1e, 0x00),
        (0x1f, 0x03), (0x20, 0x00), (0x21, 0x09), (0x22, 0x0f), (0x23, 0x04),
        (0x24, 0x00), (0x25, 0x04), (0x26, 0x00), (0x27, 0x17), (0x28, 0x02),
        (0x29, 0x00), (0x2a, 0x00), (0x2b, 0x00), (0x2c, 0x03), (0x2d, 0x07),
        (0x2e, 0x00), (0x2f, 0x20), (0x30, 0x00), (0x31, 0x7d), (0x32, 0x00),
        (0x33, 0x00), (0x34, 0x01), (0x35, 0x07), (0x36, 0x00), (0x37, 0x20),
        (0x38, 0x00), (0x39, 0x00), (0x3a, 0x00), (0x3b, 0x00), (0x3c, 0x00),
        (0x40, 0x50), (0x41, 0x52), (0x42, 0x49), (0x43, 0x31), (0x44, 0x33),
        (0x45, 0x0c), (0x46, 0x02), (0x47, 0x01), (0x48, 0x01), (0x49, 0x04),
        (0x4a, 0x77), (0x4b, 0x00), (0x4c, 0x00), (0x4d, 0x85), (0x4e, 0x95),
        (0x4f, 0x01), (0x50, 0x00), (0x51, 0x00), (0x52, 0x00), (0x53, 0x00),
        (0x54, 0x00), (0x55, 0x00), (0x56, 0x00), (0x57, 0x04), (0x58, 0x17),
        (0x59, 0x30), (0x5a, 0x30), (0x5b, 0x17), (0x3d, 0x00), (0x3e, 0x00),
        (0x3f, 0x00),
    ];
    for (word, value) in expected {
        assert_eq!(r16(&cfi, word), value, "query word {word:#x}");
    }
    // The query is also reachable from autoselect (§9), and reset leaves it.
    reset(&cfi);
    unlock(&cfi);
    w16(&cfi, 0x555, 0x90);
    w16(&cfi, 0x55, 0x98);
    assert_eq!(r16(&cfi, 0x10), u16::from(b'Q'));
    reset(&cfi);
    assert_eq!(r16(&cfi, 0x10), 0xffff);
}

#[test]
fn a_program_clears_bits_and_never_sets_them() {
    let cfi = jl064j();
    program(&cfi, 0x1234, 0x12f4);
    assert_eq!(r16(&cfi, 0x1234), 0x12f4, "a read after is the array");
    program(&cfi, 0x1234, 0xff0f);
    assert_eq!(r16(&cfi, 0x1234), 0x1204, "ones in the data set nothing");
    assert_eq!(r16(&cfi, 0x1235), 0xffff, "one word, not two");
    assert!(cfi.array().is_dirty());
    // Data polling is already over: DQ7 is the true datum and a second read
    // toggles nothing (§11.1, §11.3).
    assert_eq!(r16(&cfi, 0x1234), r16(&cfi, 0x1234));
}

#[test]
fn a_sector_erase_takes_exactly_its_sector_eight_k_or_sixty_four_k() {
    let cfi = jl064j();
    // Sector 1 is words 0x1000-0x1fff (8 KiB); sector 9 is 0x10000-0x17fff
    // (64 KiB): Table 3's x16 ranges.
    for word in [
        0x0fff, 0x1000, 0x1fff, 0x2000, 0x0f_fff, 0x10_000, 0x17_fff, 0x18_000,
    ] {
        program(&cfi, word, 0x0000);
    }
    sector_erase(&cfi, 0x1800);
    assert_eq!(r16(&cfi, 0x0fff), 0, "sector 0 untouched");
    assert_eq!(r16(&cfi, 0x1000), 0xffff);
    assert_eq!(r16(&cfi, 0x1fff), 0xffff);
    assert_eq!(r16(&cfi, 0x2000), 0, "sector 2 untouched");

    sector_erase(&cfi, 0x12_345);
    assert_eq!(r16(&cfi, 0x0f_fff), 0, "the last 8 KiB sector untouched");
    assert_eq!(r16(&cfi, 0x10_000), 0xffff, "all 64 KiB");
    assert_eq!(r16(&cfi, 0x17_fff), 0xffff);
    assert_eq!(r16(&cfi, 0x18_000), 0, "and not the next");
}

#[test]
fn further_sectors_join_an_erase_within_its_time_out() {
    let cfi = jl064j();
    for word in [0x2000, 0x3000, 0x4000] {
        program(&cfi, word, 0);
    }
    sector_erase(&cfi, 0x2000);
    w16(&cfi, 0x3000, 0x30);
    // Suspend and resume are accepted and change nothing (§10.8).
    w16(&cfi, 0x3000, 0xb0);
    w16(&cfi, 0x3000, 0x30);
    assert_eq!(r16(&cfi, 0x2000), 0xffff);
    assert_eq!(r16(&cfi, 0x3000), 0xffff);
    assert_eq!(r16(&cfi, 0x4000), 0, "a sector nobody named");
}

#[test]
fn a_chip_erase_takes_everything_unprotected() {
    let cfi = Cfi::new(&jl064j_props().with("write-protect", true)).unwrap();
    // The WP# guards are sectors 0, 1, 140 and 141 (§8.11).
    let image = |cfi: &Cfi| {
        cfi.array().load_image(0, &[0u8; 64]).unwrap();
        cfi.array().load_image(0x2000, &[0u8; 64]).unwrap();
        cfi.array().load_image(0x4000, &[0u8; 64]).unwrap();
        cfi.array().load_image(0x7f_c000, &[0u8; 64]).unwrap();
    };
    image(&cfi);
    unlock(&cfi);
    w16(&cfi, 0x555, 0x80);
    unlock(&cfi);
    w16(&cfi, 0x555, 0x10);
    assert_eq!(r16(&cfi, 0x0000), 0, "SA0 is guarded");
    assert_eq!(r16(&cfi, 0x1000), 0, "SA1 is guarded");
    assert_eq!(r16(&cfi, 0x2000), 0xffff, "SA2 is not");
    assert_eq!(r16(&cfi, 0x3f_e000), 0, "SA140 is guarded");
    // And autoselect says which is which.
    unlock(&cfi);
    w16(&cfi, 0x555, 0x90);
    assert_eq!(r16(&cfi, 0x0002), 1);
    assert_eq!(r16(&cfi, 0x2002), 0);
    reset(&cfi);
    program(&cfi, 0x0000, 0x1234);
    assert_eq!(r16(&cfi, 0x0000), 0, "nor can it be programmed");
}

#[test]
fn unlock_bypass_programs_in_two_cycles_until_its_reset() {
    let cfi = jl064j();
    unlock(&cfi);
    w16(&cfi, 0x555, 0x20);
    for (i, word) in [0x100u64, 0x101, 0x102].into_iter().enumerate() {
        w16(&cfi, 0, 0xa0);
        w16(&cfi, word, 0x1111 * (i as u16 + 1));
    }
    assert_eq!(r16(&cfi, 0x100), 0x1111);
    assert_eq!(r16(&cfi, 0x102), 0x3333);
    // Only program and the bypass reset are valid here (§10.5.1): an
    // autoselect attempt is ignored.
    unlock(&cfi);
    w16(&cfi, 0x555, 0x90);
    w16(&cfi, 0, 0x00);
    // That `0x90, 0x00` was the bypass reset: two-cycle programs are over.
    w16(&cfi, 0, 0xa0);
    w16(&cfi, 0x200, 0x0000);
    assert_eq!(r16(&cfi, 0x200), 0xffff, "no longer bypassing");
    program(&cfi, 0x200, 0x0000);
    assert_eq!(r16(&cfi, 0x200), 0, "the four-cycle program still works");
}

#[test]
fn a_reset_or_a_broken_sequence_returns_to_the_array() {
    let cfi = jl064j();
    program(&cfi, 0x10, 0xabcd);
    unlock(&cfi);
    w16(&cfi, 0x555, 0x90);
    assert_eq!(r16(&cfi, 0x10), 0);
    reset(&cfi);
    assert_eq!(r16(&cfi, 0x10), 0xabcd);
    // An unlock at the wrong address is no unlock: the program that follows
    // it never starts.
    w16(&cfi, 0x555, 0xaa);
    w16(&cfi, 0x2ab, 0x55);
    w16(&cfi, 0x555, 0xa0);
    w16(&cfi, 0x10, 0x0000);
    assert_eq!(r16(&cfi, 0x10), 0xabcd);
    // A reset between cycles abandons the sequence (§10.2).
    unlock(&cfi);
    reset(&cfi);
    w16(&cfi, 0x555, 0xa0);
    w16(&cfi, 0x10, 0x0000);
    assert_eq!(r16(&cfi, 0x10), 0xabcd);
    // And A21-A11 are not decoded in a command cycle: an unlock at a bank
    // address works.
    w16(&cfi, 0x20_0555, 0xaa);
    w16(&cfi, 0x20_02aa, 0x55);
    w16(&cfi, 0x20_0555, 0xa0);
    w16(&cfi, 0x10, 0x0000);
    assert_eq!(r16(&cfi, 0x10), 0);
}

#[test]
fn the_secured_silicon_region_replaces_the_bottom_until_the_exit() {
    let serial: Vec<u8> = (0u8..16).collect();
    let cfi = Cfi::new(&jl064j_props().with("secsi", Media::new("esn", serial.clone()))).unwrap();
    program(&cfi, 0x0000, 0x5a5a);
    program(&cfi, 0x0080, 0x6b6b);
    unlock(&cfi);
    w16(&cfi, 0x555, 0x88);
    assert!(!cfi.array().is_reading_array());
    assert_eq!(r16(&cfi, 0x0000), 0x0100, "the region's first word");
    assert_eq!(r16(&cfi, 0x0007), 0x0f0e);
    assert_eq!(r16(&cfi, 0x0008), 0xffff, "the rest erased");
    assert_eq!(
        r16(&cfi, 0x0080),
        0x6b6b,
        "256 bytes is 128 words, then the array"
    );
    // Not locked, so it programs like flash — and only clears bits.
    program(&cfi, 0x0010, 0x1234);
    assert_eq!(r16(&cfi, 0x0010), 0x1234);
    assert_eq!(
        cfi.array().contents()[0x20],
        0xff,
        "the array under it is untouched"
    );
    // Exit: AA/55/90 then 00 (Table 12).
    unlock(&cfi);
    w16(&cfi, 0x555, 0x90);
    w16(&cfi, 0, 0x00);
    assert_eq!(r16(&cfi, 0x0000), 0x5a5a, "the array again");
    assert!(cfi.array().is_reading_array());
    // The region is flash: a reset keeps what was programmed into it.
    cfi.reset(ResetKind::Cold);
    unlock(&cfi);
    w16(&cfi, 0x555, 0x88);
    assert_eq!(r16(&cfi, 0x0010), 0x1234);
}

#[test]
fn a_factory_locked_region_says_so_and_refuses_a_program() {
    let cfi = Cfi::new(&jl064j_props().with("secsi-lock", "factory")).unwrap();
    unlock(&cfi);
    w16(&cfi, 0x555, 0x90);
    assert_eq!(r16(&cfi, 0x03), 0x0081);
    reset(&cfi);
    unlock(&cfi);
    w16(&cfi, 0x555, 0x88);
    program(&cfi, 0x0000, 0x0000);
    assert_eq!(r16(&cfi, 0x0000), 0xffff);
    sector_erase(&cfi, 0x0000);
    assert!(Cfi::new(&jl064j_props().with("secsi-lock", "maybe")).is_err());
}

#[test]
fn a_part_wired_x8_takes_the_byte_mode_addresses() {
    // One byte-wide part: 0xaaa/0x555 unlocks, byte addresses for the codes
    // (Table 12's "Byte" rows), and the query at 0xaa with 'Q' at 0x20.
    let cfi = Cfi::new(&jl064j_props().with("width", 1u64)).unwrap();
    let w8 = |at: u64, v: u8| cfi.array().write(at, &[v], MemAttrs::DEFAULT).unwrap();
    let r8 = |at: u64| {
        let mut b = [0u8];
        cfi.array().read(at, &mut b, MemAttrs::DEFAULT).unwrap();
        b[0]
    };
    w8(0xaaa, 0xaa);
    w8(0x555, 0x55);
    w8(0xaaa, 0x90);
    assert_eq!(r8(0x00), 0x01);
    assert_eq!(r8(0x02), 0x7e);
    assert_eq!(r8(0x1c), 0x02);
    assert_eq!(r8(0x1e), 0x01);
    w8(0, 0xf0);
    w8(0xaa, 0x98);
    assert_eq!(r8(0x20), b'Q');
    assert_eq!(r8(0x22), b'R');
    assert_eq!(r8(0x4e), 0x17, "8 MiB");
    w8(0, 0xf0);
    w8(0xaaa, 0xaa);
    w8(0x555, 0x55);
    w8(0xaaa, 0xa0);
    w8(0x1235, 0x42);
    assert_eq!(r8(0x1235), 0x42);
}

#[test]
fn an_intel_part_refuses_amd_only_properties_and_is_otherwise_unchanged() {
    let e = Cfi::new(
        &Props::new()
            .with("size", Value::Size(0x2000))
            .with("block", Value::Size(0x1000))
            .with("boot-flag", 1u64),
    )
    .expect_err("an AMD property on an Intel part")
    .to_string();
    assert!(e.contains("command-set"), "{e}");
    assert!(
        Cfi::new(
            &Props::new()
                .with("size", Value::Size(0x2000))
                .with("command-set", "toshiba")
        )
        .is_err()
    );
    let intel = Cfi::new(
        &Props::new()
            .with("size", Value::Size(0x2000))
            .with("block", Value::Size(0x1000)),
    )
    .unwrap();
    assert_eq!(intel.array().command_set(), CommandSet::Intel);
    // An AMD unlock means nothing to it: 0xaa is an unknown command, and the
    // Intel status register says so.
    intel
        .array()
        .write(0, &0x00aa_00aau32.to_le_bytes(), MemAttrs::DEFAULT)
        .unwrap();
    assert_eq!(intel.array().status(0), Some(0xb0));
    // And the query still says 0x0001.
    intel
        .array()
        .write(0, &0x0098_0098u32.to_le_bytes(), MemAttrs::DEFAULT)
        .unwrap();
    let mut b = [0u8; 4];
    intel
        .array()
        .read(0x13 * 4, &mut b, MemAttrs::DEFAULT)
        .unwrap();
    assert_eq!(u32::from_le_bytes(b), 0x0001_0001);
}

#[test]
fn geometry_and_bank_mismatches_are_refused() {
    let bad_banks = jl064j_props().with(
        "banks",
        Value::List(alloc::vec![Value::Uint(23), Value::Uint(48)]),
    );
    assert!(Cfi::new(&bad_banks).is_err(), "71 of 142 sectors");
    let bad_ext = jl064j_props().with("device-ext", Value::List(alloc::vec![Value::Uint(1)]));
    assert!(Cfi::new(&bad_ext).is_err());
    let big = jl064j_props().with("secsi", Media::new("esn", alloc::vec![0u8; 257]));
    assert!(Cfi::new(&big).is_err(), "257 bytes into 256");
}

#[test]
fn a_debug_read_sees_the_array_and_moves_nothing() {
    let cfi = jl064j();
    program(&cfi, 0x40, 0x1357);
    unlock(&cfi);
    w16(&cfi, 0x555, 0x90);
    let debug = MemAttrs {
        debug: true,
        ..MemAttrs::DEFAULT
    };
    let mut b = [0u8; 2];
    cfi.array().read(0x80, &mut b, debug).unwrap();
    assert_eq!(u16::from_le_bytes(b), 0x1357, "the contents, not the code");
    assert_eq!(r16(&cfi, 0x01), 0x227e, "still in autoselect");
    assert!(cfi.array().write(0, &[0xf0, 0], debug).is_err());
}

fn snapshot(cfi: &Cfi) -> Vec<u8> {
    let mut shape = MachineShape::new();
    shape.add_device("flash", CLASS.name).unwrap();
    let mut w = StateWriter::new(shape);
    {
        let mut chunk = w.chunk("flash", CLASS.name, CLASS.version).unwrap();
        cfi.save(&mut chunk).unwrap();
    }
    w.to_vec().unwrap()
}

#[test]
fn a_snapshot_carries_the_amd_modes_and_a_half_issued_sequence() {
    let cfi = jl064j();
    program(&cfi, 0x0000, 0x0f0f);
    // In the Secured Silicon Region, with something programmed into it, and
    // two cycles into an unlock.
    unlock(&cfi);
    w16(&cfi, 0x555, 0x88);
    program(&cfi, 0x0001, 0x00ff);
    unlock(&cfi);
    let bytes = snapshot(&cfi);

    let restored = jl064j();
    let reader = StateReader::new(&bytes).unwrap();
    let chunk = reader
        .load("flash", CLASS.name, CLASS.version, &Migrations::new())
        .unwrap();
    restored.load(&mut chunk.reader()).unwrap();
    assert_eq!(snapshot(&cfi), snapshot(&restored), "the state hash");
    // The third cycle finishes what the snapshot started: a program.
    for c in [&cfi, &restored] {
        w16(c, 0x555, 0xa0);
        w16(c, 0x0002, 0x1234);
    }
    assert_eq!(r16(&restored, 0x0002), 0x1234, "into the region");
    assert_eq!(r16(&restored, 0x0001), 0x00ff);
    assert_eq!(snapshot(&cfi), snapshot(&restored));
}
