//! What can be checked about the ROM without running it.
//!
//! The real proof is `tests/pc_at_boot.rs`, which boots a guest on it, and
//! `tests/pc_at_tables.rs`, which has a guest *walk the tables* — the searches
//! below are the same ones, in Rust, so that a broken table is a unit-test
//! failure rather than a mysterious guest.

use alloc::string::String;
use alloc::vec::Vec;

use super::platform::Platform;
use super::{BIOS_DATE, MODEL_BYTE, RESET_VECTOR, SEGMENT, SIZE, TABLES_OFFSET, image};

/// Where the image is decoded, as a physical address.
const BASE: u32 = (SEGMENT as u32) << 4;

/// Find `signature` on a 16-byte boundary, the way every one of the three
/// specifications says to search this segment. Answers the offset into the
/// image, which is the physical address less [`BASE`].
fn search(rom: &[u8], signature: &[u8]) -> Option<usize> {
    (0..rom.len())
        .step_by(16)
        .find(|&at| rom[at..].starts_with(signature))
}

/// The 8-bit sum every structure here has to bring to zero.
fn sum(bytes: &[u8]) -> u8 {
    bytes.iter().fold(0u8, |acc, &b| acc.wrapping_add(b))
}

/// Read a little-endian `u16` out of the image.
fn u16_at(rom: &[u8], at: usize) -> u16 {
    u16::from_le_bytes([rom[at], rom[at + 1]])
}

/// Read a little-endian `u32` out of the image.
fn u32_at(rom: &[u8], at: usize) -> u32 {
    u32::from_le_bytes([rom[at], rom[at + 1], rom[at + 2], rom[at + 3]])
}

/// Turn a physical address inside this segment into an offset into the image,
/// which is what a table pointing at another table gives us.
fn offset_of(address: u32) -> usize {
    assert!(
        address >= BASE && address < BASE + SIZE as u32,
        "{address:#x} is not inside the BIOS segment"
    );
    (address - BASE) as usize
}

/// `machines/pc-at.machine` with a second processor, exactly as
/// `tests/kvm_pc_at_smp.rs` patches it.
///
/// Here as well as there because the *tables* have to change with the board and
/// this is where that is checked; the anchors are asserted so that a silent
/// `str::replace` matching nothing cannot leave a one-processor board being
/// tested as if it had two.
#[cfg(feature = "dev-pc")]
pub(super) fn add_second_processor(text: &str) -> String {
    let mut text = String::from(text);

    const CPU0: &str = "  object cpu0 \"cpu.x86\" {\n\
                        \x20   clock   = cpu\n\
                        \x20   space   = mem\n\
                        \x20   iospace = \"port\"\n\
                        \x20   model   = \"80486\"\n\
                        \x20   engine  = \"interp\"\n\
                        \x20 }\n";
    assert!(text.contains(CPU0), "the `cpu0` object moved");
    text = text.replace(
        CPU0,
        &alloc::format!("{CPU0}{}", CPU0.replace("cpu0", "cpu1")),
    );

    const APICS: &str = "  object lapic0 \"pc.lapic\"  { clock = bus, id = 0, bus = \"apic\" }\n  \
                         object ioapic \"pc.ioapic\" { id = 1, bus = \"apic\" }";
    assert!(text.contains(APICS), "the APIC objects moved");
    text = text.replace(
        APICS,
        "  object lapic0 \"pc.lapic\"  { clock = bus, id = 0, bus = \"apic\" }\n  \
         object lapic1 \"pc.lapic\"  { clock = bus, id = 1, bus = \"apic\" }\n  \
         object ioapic \"pc.ioapic\" { id = 2, bus = \"apic\" }",
    );

    const LAPIC0_MAP: &str = "  map mem 0xfee00000 size 0x1000   = lapic0.regs";
    assert!(text.contains(LAPIC0_MAP), "the lapic0 mapping moved");
    text = text.replace(
        LAPIC0_MAP,
        "  map mem 0xfee00000 size 0x1000   = lapic0.regs\n  \
         map mem 0xfef00000 size 0x1000   = lapic1.regs",
    );

    const LAPIC0_WIRES: &str = "  wire lapic0.intr -> cpu0.intr\n  wire lapic0.nmi  -> cpu0.nmi";
    assert!(text.contains(LAPIC0_WIRES), "the lapic0 wires moved");
    text = text.replace(
        LAPIC0_WIRES,
        "  wire lapic0.intr -> cpu0.intr\n  \
         wire lapic0.nmi  -> cpu0.nmi\n  \
         wire lapic1.intr -> cpu1.intr\n  \
         wire lapic1.nmi  -> cpu1.nmi",
    );
    text
}

// ---------------------------------------------------------------------------
// the image
// ---------------------------------------------------------------------------

#[test]
fn the_image_is_byte_identical_across_builds() {
    // The determinism rule is not decoration here: a machine's state hash
    // includes what the firmware wrote, so an image that varied would make
    // every `pc-at` regression test irreproducible.
    assert_eq!(image(), image());
    assert_eq!(image().len(), SIZE);
}

#[test]
fn the_reset_vector_is_a_far_jump_into_this_segment() {
    // An 80486 fetches `0xfffffff0`, which `pc.rom`'s top alignment puts here.
    // A firmware whose first instruction is not a far jump never leaves the
    // 16-byte window it starts in.
    let rom = image();
    let at = RESET_VECTOR as usize;
    assert_eq!(rom[at], 0xea, "the reset vector is not a far jump");
    let target = u16::from_le_bytes([rom[at + 1], rom[at + 2]]);
    let segment = u16::from_le_bytes([rom[at + 3], rom[at + 4]]);
    assert_eq!(segment, SEGMENT);
    assert!(
        (target as usize) < RESET_VECTOR as usize,
        "the far jump targets {target:#06x}, which is not code"
    );
}

#[test]
fn the_identification_bytes_are_where_software_looks_for_them() {
    let rom = image();
    assert_eq!(&rom[0xfff5..0xfffd], BIOS_DATE);
    assert_eq!(rom[0xfffe], MODEL_BYTE);
}

#[test]
fn the_whole_image_sums_to_zero() {
    // The convention every PC ROM follows, and the only thing the last byte is
    // for. A checksum that does not come out is the cheapest signal that the
    // image was truncated or patched after assembly.
    assert_eq!(sum(&image()), 0);
}

#[test]
fn the_code_fits_below_the_tables_and_the_tables_below_the_reset_vector() {
    // The assembler panics on an overflow, so this is about the *other* ends:
    // the code must not have grown into the table block, and the tables must
    // not have grown into the sixteen bytes at the top.
    let rom = image();
    let code = rom[..TABLES_OFFSET as usize]
        .iter()
        .rposition(|&b| b != 0xff)
        .expect("the image is not empty");
    let used = rom[..RESET_VECTOR as usize]
        .iter()
        .rposition(|&b| b != 0xff)
        .expect("the image is not empty");
    // `fw-pcbios` does not imply `std`, and the whole crate is `no_std` without
    // it, so this diagnostic cannot be unconditional -- the per-feature sweep
    // builds `--no-default-features --features fw-pcbios` and there is no
    // `println!` there. The assertions below are the test; this only says how
    // much room is left.
    #[cfg(feature = "std")]
    std::println!(
        "the firmware occupies {code:#06x} bytes of code and reaches {used:#06x} with its tables"
    );
    assert!(
        code < TABLES_OFFSET as usize,
        "the code reaches {code:#06x}, which collides with the tables"
    );
    assert!(
        used < RESET_VECTOR as usize,
        "the tables reach {used:#06x}, which collides with the reset vector"
    );
    // Not a size limit, a sanity one: a firmware this small that suddenly
    // filled the socket would mean a runaway table.
    assert!(code < 0x4000, "the code is unexpectedly large: {code:#06x}");
}

#[test]
fn the_tables_land_where_the_documentation_says_they_do() {
    // `docs/platforms/pc-at.md` names these two addresses, so they are a claim
    // rather than an implementation detail. The rest are found by searching,
    // which is how a guest finds them and why only the first two are named.
    let tables = super::tables::generate(0xf_8000, &Platform::at());
    assert_eq!(tables.mp_pointer, 0xf_8000);
    assert_eq!(tables.mp_config, 0xf_8010);
    assert!(
        tables.smbios > tables.rsdp,
        "the searched-for structures come first"
    );
}

// ---------------------------------------------------------------------------
// the MultiProcessor specification's structures
// ---------------------------------------------------------------------------

/// Walk the MP configuration table the way an operating system does, and answer
/// the enabled processors' local APIC IDs and the entry count the header
/// declared.
fn walk_mp(rom: &[u8]) -> (Vec<u8>, u16) {
    let pointer = search(rom, b"_MP_").expect("an MP floating pointer in the BIOS segment");
    assert_eq!(sum(&rom[pointer..pointer + 16]), 0, "its checksum");
    assert_eq!(rom[pointer + 8], 1, "one 16-byte paragraph long");
    assert_eq!(rom[pointer + 9], 4, "version 1.4");
    assert_eq!(
        rom[pointer + 11],
        0,
        "feature byte 1 must be zero: a configuration table is present"
    );

    let config = offset_of(u32_at(rom, pointer + 4));
    assert_eq!(&rom[config..config + 4], b"PCMP");
    let length = usize::from(u16_at(rom, config + 4));
    assert_eq!(sum(&rom[config..config + length]), 0, "its checksum");
    assert_eq!(rom[config + 6], 4, "version 1.4");
    let count = u16_at(rom, config + 34);

    let mut ids = Vec::new();
    let mut at = config + 44;
    let mut seen = 0;
    while seen < count {
        let kind = rom[at];
        if kind == 0 {
            // *MP* §4.3.1: `CPU FLAGS` bit 0 is `EN`.
            if rom[at + 3] & 1 != 0 {
                ids.push(rom[at + 1]);
            }
            at += 20;
        } else {
            at += 8;
        }
        seen += 1;
    }
    assert_eq!(
        at,
        config + length,
        "the entries do not fill the declared base table length"
    );
    (ids, count)
}

#[test]
fn a_search_of_the_segment_finds_a_valid_mp_table() {
    let rom = image();
    let (ids, count) = walk_mp(&rom);
    assert_eq!(ids, [0], "the shipped board has one processor, APIC ID 0");
    // One processor, one bus, one I/O APIC, seven interrupt assignments, and
    // the two local ones.
    assert_eq!(count, 12);
}

#[test]
fn the_bootstrap_processor_is_the_one_marked_bp() {
    let rom = image();
    let pointer = search(&rom, b"_MP_").expect("a floating pointer");
    let config = offset_of(u32_at(&rom, pointer + 4));
    let first = config + 44;
    assert_eq!(rom[first], 0, "the first entry is a processor entry");
    // Bit 0 `EN` and bit 1 `BP` (*MP* §4.3.1, Table 4-4).
    assert_eq!(rom[first + 3] & 0b11, 0b11);
}

#[test]
fn the_timer_is_published_on_the_input_the_board_wires_it_to() {
    // ISA IRQ0 arrives on I/O APIC input 2, which is a fact about the board
    // (`wire pit0.out0 -> ioapic.irq2`) and not about this firmware. An
    // operating system that misses it loses its clock.
    let rom = image();
    let pointer = search(&rom, b"_MP_").expect("a floating pointer");
    let config = offset_of(u32_at(&rom, pointer + 4));
    let length = usize::from(u16_at(&rom, config + 4));
    let mut at = config + 44;
    let mut found = None;
    while at < config + length {
        if rom[at] == 3 && rom[at + 5] == 0 {
            found = Some(rom[at + 7]);
        }
        at += if rom[at] == 0 { 20 } else { 8 };
    }
    assert_eq!(found, Some(2), "ISA IRQ0 should reach INTIN 2");
}

// ---------------------------------------------------------------------------
// ACPI
// ---------------------------------------------------------------------------

/// Follow the RSDP into the XSDT and answer the tables it lists.
fn acpi_tables(rom: &[u8]) -> Vec<(String, usize)> {
    let rsdp = search(rom, b"RSD PTR ").expect("an RSDP in the BIOS segment");
    assert_eq!(sum(&rom[rsdp..rsdp + 20]), 0, "the first checksum");
    assert_eq!(sum(&rom[rsdp..rsdp + 36]), 0, "the extended checksum");
    assert_eq!(rom[rsdp + 15], 2, "revision 2, so the XSDT is valid");

    let xsdt = offset_of(
        u32::try_from(u64::from_le_bytes(
            rom[rsdp + 24..rsdp + 32].try_into().expect("eight bytes"),
        ))
        .expect("an address inside the segment"),
    );
    assert_eq!(&rom[xsdt..xsdt + 4], b"XSDT");
    let length = u32_at(rom, xsdt + 4) as usize;
    assert_eq!(sum(&rom[xsdt..xsdt + length]), 0, "the XSDT's checksum");

    let mut out = Vec::new();
    let mut at = xsdt + 36;
    while at + 8 <= xsdt + length {
        let table = offset_of(
            u32::try_from(u64::from_le_bytes(
                rom[at..at + 8].try_into().expect("eight bytes"),
            ))
            .expect("an address inside the segment"),
        );
        let signature = String::from_utf8_lossy(&rom[table..table + 4]).into_owned();
        let table_len = u32_at(rom, table + 4) as usize;
        assert_eq!(
            sum(&rom[table..table + table_len]),
            0,
            "{signature}'s checksum"
        );
        out.push((signature, table));
        at += 8;
    }
    out
}

/// The APIC IDs the MADT's processor structures carry.
fn madt_processors(rom: &[u8], madt: usize) -> Vec<u8> {
    let length = u32_at(rom, madt + 4) as usize;
    let mut ids = Vec::new();
    // 36 for the header, then the local APIC address and the flags.
    let mut at = madt + 44;
    while at < madt + length {
        let kind = rom[at];
        let len = usize::from(rom[at + 1]);
        assert!(len >= 2, "a MADT structure with no length");
        // Table 5.22: type 0, Processor Local APIC. Bit 0 of its flags is
        // Enabled.
        if kind == 0 && u32_at(rom, at + 4) & 1 != 0 {
            ids.push(rom[at + 3]);
        }
        at += len;
    }
    assert_eq!(at, madt + length, "a structure overran the MADT");
    ids
}

#[test]
fn a_search_of_the_segment_finds_the_acpi_tables() {
    let rom = image();
    let tables = acpi_tables(&rom);
    let names: Vec<&str> = tables.iter().map(|(s, _)| s.as_str()).collect();
    assert_eq!(names, ["FACP", "APIC"]);

    // The FADT's DSDT pointer resolves to a DSDT, which is what an operating
    // system needs before it will use anything else in the set.
    let fadt = tables[0].1;
    let dsdt = offset_of(u32_at(&rom, fadt + 40));
    assert_eq!(&rom[dsdt..dsdt + 4], b"DSDT");
    let dsdt_len = u32_at(&rom, dsdt + 4) as usize;
    assert_eq!(sum(&rom[dsdt..dsdt + dsdt_len]), 0);
    // The 64-bit pointer has to agree with the 32-bit one (*ACPI* §5.2.9).
    assert_eq!(
        u64::from_le_bytes(rom[fadt + 140..fadt + 148].try_into().expect("eight bytes")),
        u64::from(u32_at(&rom, fadt + 40))
    );
}

#[test]
fn the_madt_lists_the_processors_the_mp_table_does() {
    let rom = image();
    let tables = acpi_tables(&rom);
    let madt = tables.iter().find(|(s, _)| s == "APIC").expect("a MADT").1;
    let (mp, _) = walk_mp(&rom);
    assert_eq!(madt_processors(&rom, madt), mp);
    // And the local APIC address is the board's, not a constant here.
    assert_eq!(u32_at(&rom, madt + 36), Platform::at().lapic);
}

#[test]
fn the_rsdt_and_the_xsdt_list_the_same_tables() {
    // An operating system too old for the XSDT has to reach the same place.
    let rom = image();
    let rsdp = search(&rom, b"RSD PTR ").expect("an RSDP");
    let rsdt = offset_of(u32_at(&rom, rsdp + 16));
    assert_eq!(&rom[rsdt..rsdt + 4], b"RSDT");
    let length = u32_at(&rom, rsdt + 4) as usize;
    let from_rsdt: Vec<u32> = (36..length)
        .step_by(4)
        .map(|at| u32_at(&rom, rsdt + at))
        .collect();
    let from_xsdt: Vec<u32> = acpi_tables(&rom)
        .iter()
        .map(|(_, at)| BASE + *at as u32)
        .collect();
    assert_eq!(from_rsdt, from_xsdt);
}

// ---------------------------------------------------------------------------
// SMBIOS
// ---------------------------------------------------------------------------

#[test]
fn a_search_of_the_segment_finds_the_smbios_entry_point() {
    let rom = image();
    let eps = search(&rom, b"_SM_").expect("an SMBIOS entry point");
    assert_eq!(sum(&rom[eps..eps + 0x1f]), 0, "its checksum");
    assert_eq!(&rom[eps + 16..eps + 21], b"_DMI_");
    assert_eq!(sum(&rom[eps + 16..eps + 0x1f]), 0, "the intermediate one");

    let length = usize::from(u16_at(&rom, eps + 22));
    let structures = offset_of(u32_at(&rom, eps + 24));
    let count = u16_at(&rom, eps + 28);

    // Walk the structure table: a header, a formatted area, then a string set
    // ended by a double null (*SMBIOS* §6.1).
    let mut at = structures;
    let mut seen = 0;
    let mut kinds = Vec::new();
    while seen < count {
        kinds.push(rom[at]);
        at += usize::from(rom[at + 1]);
        while rom[at] != 0 || rom[at + 1] != 0 {
            at += 1;
        }
        at += 2;
        seen += 1;
    }
    assert_eq!(at, structures + length, "the declared table length");
    assert_eq!(kinds, [0, 1, 4, 127], "BIOS, system, one processor, end");
}

// ---------------------------------------------------------------------------
// the board, not a constant
// ---------------------------------------------------------------------------

#[test]
#[cfg(feature = "dev-pc")]
fn a_two_processor_board_publishes_two_processors() {
    // The claim the whole file exists to support: the same firmware source,
    // built for a board with a second processor in it, publishes tables that
    // say so — in both the MP table and the MADT, and with the second
    // processor's own APIC ID rather than an assumed one.
    let text = add_second_processor(crate::dev::pc::PC_AT);
    let platform = Platform::from_machine("pc-at-smp.machine", &text).expect("it resolves");
    let rom = super::image_for(&platform);

    let (mp, count) = walk_mp(&rom);
    assert_eq!(mp, [0, 1]);
    assert_eq!(count, 13, "one more entry than the one-processor board");

    let tables = acpi_tables(&rom);
    let madt = tables.iter().find(|(s, _)| s == "APIC").expect("a MADT").1;
    assert_eq!(madt_processors(&rom, madt), [0, 1]);

    // And the one-processor board still publishes one, from the same code.
    let (mp, _) = walk_mp(&image());
    assert_eq!(mp, [0]);
}

#[test]
#[cfg(feature = "dev-pc")]
fn the_shipped_board_is_what_the_default_image_describes() {
    // `image()` takes no arguments and has to describe some board; this is the
    // assertion that the board it describes is the one in `machines/`.
    assert_eq!(
        image(),
        super::image_for_machine("pc-at.machine", crate::dev::pc::PC_AT).expect("it resolves")
    );
}
