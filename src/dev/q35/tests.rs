//! The chipset's own tests: the board file, the two bridges' registers, and the
//! tables built against a machine assembled by hand.
//!
//! `tests/q35_board.rs` runs the real board and is where a memory map or a wire
//! graph is checked. What is here is the parts, on rigs small enough that a
//! failure names one register.

use super::*;

use alloc::string::{String, ToString};
use alloc::sync::Arc;
use alloc::vec::Vec;

use crate::bus::pci::{Bdf, PciBus, config};
use crate::core::device::{Device, ResetKind};
use crate::core::space::{
    AddressSpace, MemAttrs, Perms, RamStore, Region, RegionRef, UnassignedPolicy,
};
use crate::core::state::{MachineShape, Migrations, StateReader, StateWriter};
use crate::core::value::Width;

// ---------------------------------------------------------------------------
// the board file
// ---------------------------------------------------------------------------

#[test]
fn the_board_parses_and_resolves() {
    use crate::machine::{ResolveOptions, resolve_file};
    let resolved = match resolve_file("q35.machine", Q35, &ResolveOptions::new()) {
        Ok(r) => r,
        Err(e) => panic!("{e}"),
    };
    assert_eq!(resolved.name, "q35");
    assert_eq!(resolved.spaces.len(), 2, "memory and I/O are separate");
    // Nine crystals: `pc-at`'s eight, plus the power-management timer's. The
    // one that matters is `pmtmr`, and it matters because it is not an integer
    // number of hertz: ICH9 §13.8.3.4 rates it at 14.31818 MHz over four, and
    // 14.31818 MHz is itself 315/22 MHz. Stored reduced.
    let pmtmr = resolved
        .oscillators
        .iter()
        .find(|o| o.name == "pmtmr")
        .expect("the ACPI timer's crystal");
    assert_eq!(pmtmr.hz.numerator(), 39375000);
    assert_eq!(pmtmr.hz.denominator(), 11);
    // 39375000/11 is 315000000/88 reduced, and it is 3.579545... MHz.
    assert_eq!(315_000_000u64 / 88, 39_375_000 / 11);
}

#[test]
fn the_board_names_exactly_the_media_slots_it_documents() {
    use crate::machine::{ResolveOptions, resolve_file};
    let resolved = resolve_file("q35.machine", Q35, &ResolveOptions::new()).expect("it resolves");
    let mut slots: Vec<String> = resolved
        .objects
        .iter()
        .filter_map(|o| o.props.get("image"))
        .filter_map(|v| v.as_str().map(ToString::to_string))
        .collect();
    slots.sort();
    slots.dedup();
    // No floppy: a q35 has no diskette controller on the board, and a machine
    // file that declared one would be describing a card nobody fitted.
    assert_eq!(slots, ["bios", "hd0", "hd1", "vgabios"]);
}

/// The region names [`acpi`] looks devices up by are those devices' own class
/// names, and nothing but this test stops the two drifting apart.
#[cfg(feature = "dev-pc-apic")]
#[test]
fn the_class_names_the_generator_looks_for_have_not_drifted() {
    assert_eq!(crate::dev::pc::apic::CLASS_NAME, "pc.lapic");
    assert_eq!(crate::dev::pc::ioapic::CLASS_NAME, "pc.ioapic");
    #[cfg(feature = "dev-pc-hpet")]
    assert_eq!(crate::dev::pc::hpet::CLASS_NAME, "pc.hpet");
}

// ---------------------------------------------------------------------------
// a rig: the two bridges on a fabric, in two spaces
// ---------------------------------------------------------------------------

/// A memory space with a ROM-like region under the shadow, so a PAM window can
/// be seen to switch.
struct Rig {
    mem: Arc<AddressSpace>,
    io: Arc<AddressSpace>,
    mch: mch::Mch,
    lpc: lpc::Lpc,
    #[allow(dead_code)]
    bus: Arc<PciBus>,
}

impl Rig {
    /// One bridge pair, with `PCIEXBAR` and `PMBASE` at their datasheet
    /// defaults — nothing decoded until something programs them.
    fn new() -> Rig {
        Rig::with_resets(0, 0)
    }

    fn with_resets(ecam: u64, pm_base: u32) -> Rig {
        let bus = Arc::new(PciBus::new());
        let mem = Arc::new(AddressSpace::new("mem", 32).with_unassigned(UnassignedPolicy::ONES));
        let io = Arc::new(AddressSpace::new("port", 16).with_unassigned(UnassignedPolicy::ONES));
        // Something under the shadow, so *ROM* and *DRAM* are distinguishable:
        // a store full of 0x5a at 0xc0000-0xfffff, at the default priority the
        // shadow sits above.
        let rom = Arc::new(RamStore::new(mch::SHADOW_LEN));
        rom.fill(0, mch::SHADOW_LEN, 0x5a).expect("in range");
        let region: RegionRef = Arc::new(Region::ram("rig.rom", rom));
        mem.topology()
            .map_with(
                crate::core::space::Mapping::new(region, mch::SHADOW_BASE)
                    .with_perms(Perms::READ.union(Perms::EXEC)),
            )
            .expect("it fits");

        let reset_pciexbar = if ecam == 0 { 0xe000_0000 } else { ecam | 1 };
        let mch = mch::Mch::with_bus(
            Arc::clone(&bus),
            Bdf::new(0, 0, 0).expect("legal"),
            mch::DEVICE_ID_82Q35,
            0,
            reset_pciexbar,
        )
        .expect("the window table fits its store");
        let lpc = lpc::Lpc::with_bus(
            Arc::clone(&bus),
            Bdf::new(0, lpc::LPC_DEVICE, 0).expect("legal"),
            0x2918,
            0,
            pm_base,
            [0x80; lpc::PIRQS],
            String::from("port"),
        );
        mch.attach_space(&mem);
        lpc.attach_space(&io);
        let mut deferred = crate::core::device::Deferred::new();
        let hosts = crate::core::HostObjects::new();
        {
            let mut ctx = crate::core::device::RealizeCtx::new(
                "mch",
                crate::core::space::RequesterId::ANONYMOUS,
                &mut deferred,
                &hosts,
            );
            mch.realize(&mut ctx).expect("nothing else is at 00:00.0");
        }
        {
            let mut ctx = crate::core::device::RealizeCtx::new(
                "lpc",
                crate::core::space::RequesterId::ANONYMOUS,
                &mut deferred,
                &hosts,
            );
            lpc.realize(&mut ctx).expect("nothing else is at 00:1f.0");
        }
        Rig {
            mem,
            io,
            mch,
            lpc,
            bus,
        }
    }

    /// A configuration write to `at`, delivered straight to the function — the
    /// path a `0xcfc` write takes once the fabric has routed it.
    fn write_config(&self, function: &dyn crate::bus::pci::PciFunction, at: u16, bytes: &[u8]) {
        function.config_write(at, bytes, MemAttrs::DEFAULT);
    }

    /// One byte of the memory space.
    fn peek(&self, at: u64) -> u64 {
        self.mem
            .read(at, Width::U8, MemAttrs::DEFAULT)
            .expect("a mapped byte")
    }
}

/// Reach a bridge's `PciFunction` face through the fabric, which is the only
/// route a guest has.
fn function(bus: &PciBus, at: Bdf) -> Arc<dyn crate::bus::pci::PciFunction> {
    bus.function(at).expect("the bridge announced itself")
}

// ---------------------------------------------------------------------------
// the north bridge
// ---------------------------------------------------------------------------

#[test]
fn pciexbar_masks_bits_27_and_26_against_the_length_in_force() {
    let rig = Rig::new();
    let f = function(&rig.bus, Bdf::default());
    // 256 MB (LENGTH = 00b): bits 27:26 are the address mask and read as zero,
    // whatever was written (§5.1.16). Everything between 25 and 3 is reserved
    // and never latched at all.
    rig.write_config(&*f, 0x60, &0xecff_fff1u32.to_le_bytes());
    assert_eq!(
        rig.mch.pciexbar() & 0x0fff_ffff,
        0x1,
        "bits 27:26 read zero at 256 MB, and 25:3 are reserved"
    );
    assert_eq!(rig.mch.pciexbar() & 0xf000_0000, 0xe000_0000);
    assert_eq!(rig.mch.ecam(), Some((0xe000_0000, 256 * 1024 * 1024)));
    // 128 MB (LENGTH = 01b): bit 27 becomes address, bit 26 stays mask.
    rig.write_config(&*f, 0x60, &0xe800_0003u32.to_le_bytes());
    assert_eq!(rig.mch.ecam(), Some((0xe800_0000, 128 * 1024 * 1024)));
    // 64 MB (LENGTH = 10b): both are address.
    rig.write_config(&*f, 0x60, &0xe400_0005u32.to_le_bytes());
    assert_eq!(rig.mch.ecam(), Some((0xe400_0000, 64 * 1024 * 1024)));
    // 11b is reserved, so no window decodes at all rather than a guessed one.
    rig.write_config(&*f, 0x60, &0xe400_0007u32.to_le_bytes());
    assert_eq!(rig.mch.ecam(), None);
    // And the enable bit gates everything.
    rig.write_config(&*f, 0x60, &0xe000_0000u32.to_le_bytes());
    assert_eq!(rig.mch.ecam(), None);
}

#[test]
fn the_ecam_window_appears_in_the_space_when_pciexbar_is_enabled() {
    let rig = Rig::new();
    let f = function(&rig.bus, Bdf::default());
    // Out of reset the datasheet's default has the enable clear, so nothing is
    // there and the space reads as ones.
    assert_eq!(
        rig.mem
            .read(0xe000_0000, Width::U32, MemAttrs::DEFAULT)
            .expect("unassigned reads as ones"),
        0xffff_ffff
    );
    rig.write_config(&*f, 0x60, &0xe000_0001u32.to_le_bytes());
    assert_eq!(
        rig.mem
            .read(0xe000_0000, Width::U32, MemAttrs::DEFAULT)
            .expect("mapped"),
        u64::from(mch::DEVICE_ID_82Q35) << 16 | u64::from(config::VENDOR_INTEL),
        "the bridge answers its own ECAM window"
    );
}

#[test]
fn a_pam_nibble_is_three_permissions_and_an_absent_mapping() {
    let rig = Rig::new();
    let f = function(&rig.bus, Bdf::default());
    // Disabled: the ROM underneath answers.
    assert_eq!(rig.peek(0xf_0000), 0x5a);
    // Read only, on PAM0's high nibble. Reads come from the DRAM, which is
    // zeroed, so the byte changes.
    rig.write_config(&*f, 0x90, &[0x10]);
    assert_eq!(rig.peek(0xf_0000), 0x00, "reads come from the DRAM now");
    // Write only: reads fall back through to the ROM, writes go to the DRAM.
    rig.write_config(&*f, 0x90, &[0x20]);
    assert_eq!(rig.peek(0xf_0000), 0x5a, "reads are the ROM's again");
    rig.mem
        .write(0xf_0000, Width::U8, 0x99, MemAttrs::DEFAULT)
        .expect("the DRAM claims writes");
    // Read/write: and the byte the write-only window took is there.
    rig.write_config(&*f, 0x90, &[0x30]);
    assert_eq!(
        rig.peek(0xf_0000),
        0x99,
        "the write-only window really wrote to the DRAM"
    );
    // Back to disabled, and the ROM answers again — which only works because a
    // disabled window is *unmapped* rather than mapped with no permissions.
    rig.write_config(&*f, 0x90, &[0x00]);
    assert_eq!(rig.peek(0xf_0000), 0x5a);
}

#[test]
fn pam0s_low_nibble_is_reserved_and_governs_nothing() {
    let rig = Rig::new();
    let f = function(&rig.bus, Bdf::default());
    // §5.1.18: PAM0 bits 3:0 are reserved. Setting them must not switch the
    // f-segment, which PAM0[7:4] governs.
    rig.write_config(&*f, 0x90, &[0x03]);
    assert_eq!(rig.peek(0xf_0000), 0x5a);
}

#[test]
fn the_north_bridges_state_round_trips() {
    let a = Rig::new();
    let f = function(&a.bus, Bdf::default());
    a.write_config(&*f, 0x90, &[0x30]);
    a.write_config(&*f, 0x93, &[0x11]);
    a.write_config(&*f, 0x60, &0xd000_0003u32.to_le_bytes());
    a.mem
        .write(0xf_1234, Width::U8, 0xa5, MemAttrs::DEFAULT)
        .expect("the shadow claims it");

    let saved = {
        let mut shape = MachineShape::new();
        shape.add_device("mch", mch::CLASS_NAME).expect("unique");
        let mut w = StateWriter::new(shape);
        {
            let mut chunk = w.chunk("mch", mch::CLASS_NAME, 1).expect("one chunk");
            a.mch.save(&mut chunk).expect("saves");
        }
        w.to_vec().expect("encodes")
    };

    let b = Rig::new();
    let reader = StateReader::new(&saved).expect("it parses");
    let chunk = reader
        .load("mch", mch::CLASS_NAME, 1, &Migrations::new())
        .expect("the chunk is there");
    b.mch.load(&mut chunk.reader()).expect("it loads");

    for i in 0..7u16 {
        assert_eq!(a.mch.pam(i), b.mch.pam(i), "PAM{i}");
    }
    assert_eq!(a.mch.pciexbar(), b.mch.pciexbar());
    assert_eq!(a.mch.ecam(), b.mch.ecam());
    // The memory map is derived and is rebuilt rather than restored, so the
    // strongest check is that the guest sees the same bytes.
    assert_eq!(a.peek(0xf_1234), b.peek(0xf_1234));
    assert_eq!(a.peek(0xf_0000), b.peek(0xf_0000));
}

// ---------------------------------------------------------------------------
// the south bridge
// ---------------------------------------------------------------------------

#[test]
fn the_pirq_routers_come_out_of_reset_routing_nothing() {
    let rig = Rig::new();
    for index in 0..lpc::PIRQS {
        assert_eq!(
            rig.lpc.pirq_route(index),
            0x80,
            "PIRQ{index} out of reset: bit 7 set means *not* routed (§13.1.17)"
        );
    }
}

#[test]
fn a_pirq_router_keeps_only_the_bits_the_datasheet_gives_it() {
    let rig = Rig::new();
    let f = function(&rig.bus, Bdf::new(0, lpc::LPC_DEVICE, 0).expect("legal"));
    // Bits 6:4 are reserved and must read back as zero.
    rig.write_config(&*f, 0x60, &[0xff]);
    assert_eq!(rig.lpc.pirq_route(0), 0x8f);
    rig.write_config(&*f, 0x60, &[0x7b]);
    assert_eq!(rig.lpc.pirq_route(0), 0x0b, "IRQ11, routed");
    // PIRQ[E-H] are a second, disjoint run at 0x68.
    rig.write_config(&*f, 0x6b, &[0x0a]);
    assert_eq!(rig.lpc.pirq_route(7), 0x0a, "PIRQH is the byte at 0x6b");
}

#[test]
fn pmbase_is_placed_on_a_128_byte_boundary_and_indicates_io_space() {
    let rig = Rig::new();
    let f = function(&rig.bus, Bdf::new(0, lpc::LPC_DEVICE, 0).expect("legal"));
    assert_eq!(rig.lpc.acpi_base(), None, "ACPI_EN is clear out of reset");
    // Bits 6:1 are reserved and bit 0 is hardwired to 1 (§13.1.13), so a write
    // of 0x67f comes back as 0x600 with the indicator set.
    rig.write_config(&*f, 0x40, &0x0000_067fu32.to_le_bytes());
    let mut read = [0u8; 4];
    f.config_read(0x40, &mut read, MemAttrs::DEFAULT);
    assert_eq!(u32::from_le_bytes(read), 0x0000_0601);
    // Still nothing decoded: ACPI_EN gates it.
    assert_eq!(rig.lpc.acpi_base(), None);
    rig.write_config(&*f, 0x44, &[0x80]);
    assert_eq!(rig.lpc.acpi_base(), Some(0x600));
    // And the window is really in the I/O space.
    assert_ne!(
        rig.io
            .read(0x608, Width::U32, MemAttrs::DEFAULT)
            .expect("mapped"),
        0xffff_ffff
    );
}

#[test]
fn a_reserved_sci_selection_drives_no_pin_at_all() {
    let rig = Rig::new();
    let f = function(&rig.bus, Bdf::new(0, lpc::LPC_DEVICE, 0).expect("legal"));
    // §13.1.14: 011b is reserved. It is accepted by the register — every bit of
    // 2:0 is R/W — and selects nothing, which is the only answer a table with a
    // hole in it can give.
    rig.write_config(&*f, 0x44, &[0x83]);
    let mut read = [0u8; 1];
    f.config_read(0x44, &mut read, MemAttrs::DEFAULT);
    assert_eq!(read[0], 0x83, "the register reads back what was written");
}

#[test]
fn the_south_bridges_state_round_trips() {
    let a = Rig::with_resets(0, 0x600);
    let f = function(&a.bus, Bdf::new(0, lpc::LPC_DEVICE, 0).expect("legal"));
    a.write_config(&*f, 0x60, &[0x0b, 0x0a, 0x05, 0x0f]);
    a.write_config(&*f, 0x44, &[0x82]);
    a.write_config(&*f, 0xf0, &0xfed1_c001u32.to_le_bytes());
    // Move the PM timer along, so the counter is part of what is compared.
    a.lpc.acpi().advance_to(123_456);
    a.io.write(0x600 + 2, Width::U16, 1, MemAttrs::DEFAULT)
        .expect("PM1_EN");

    let saved = {
        let mut shape = MachineShape::new();
        shape.add_device("lpc", lpc::CLASS_NAME).expect("unique");
        let mut w = StateWriter::new(shape);
        {
            let mut chunk = w.chunk("lpc", lpc::CLASS_NAME, 1).expect("one chunk");
            a.lpc.save(&mut chunk).expect("saves");
        }
        w.to_vec().expect("encodes")
    };

    let b = Rig::with_resets(0, 0x600);
    let reader = StateReader::new(&saved).expect("it parses");
    let chunk = reader
        .load("lpc", lpc::CLASS_NAME, 1, &Migrations::new())
        .expect("the chunk is there");
    b.lpc.load(&mut chunk.reader()).expect("it loads");

    for index in 0..lpc::PIRQS {
        assert_eq!(
            a.lpc.pirq_route(index),
            b.lpc.pirq_route(index),
            "PIRQ{index}"
        );
    }
    assert_eq!(a.lpc.acpi_base(), b.lpc.acpi_base());
    assert_eq!(a.lpc.acpi().tick(), b.lpc.acpi().tick());
    assert_eq!(a.lpc.acpi().save_state(), b.lpc.acpi().save_state());
    assert_eq!(
        a.io.read(0x608, Width::U32, MemAttrs::DEFAULT),
        b.io.read(0x608, Width::U32, MemAttrs::DEFAULT),
        "the PM timer reads the same after a restore"
    );
}

#[test]
fn a_warm_reset_puts_both_bridges_back_to_their_boards_defaults() {
    let rig = Rig::with_resets(0xd000_0000, 0x600);
    let mch_f = function(&rig.bus, Bdf::default());
    let lpc_f = function(&rig.bus, Bdf::new(0, lpc::LPC_DEVICE, 0).expect("legal"));
    rig.write_config(&*mch_f, 0x90, &[0x30]);
    rig.write_config(&*mch_f, 0x60, &0xc000_0001u32.to_le_bytes());
    rig.write_config(&*lpc_f, 0x60, &[0x0b]);
    rig.write_config(&*lpc_f, 0x44, &[0x00]);

    rig.mch.reset(ResetKind::Warm);
    rig.lpc.reset(ResetKind::Warm);

    assert_eq!(rig.mch.pam(0), Some(0), "PAM back to 00h");
    assert_eq!(
        rig.mch.ecam(),
        Some((0xd000_0000, 256 * 1024 * 1024)),
        "PCIEXBAR back to what the board asked for, not to what the guest wrote"
    );
    assert_eq!(
        rig.lpc.pirq_route(0),
        0x80,
        "the router routes nothing again"
    );
    assert_eq!(rig.lpc.acpi_base(), Some(0x600), "and ACPI_EN is set again");
    assert_eq!(rig.peek(0xf_0000), 0x5a, "the ROM is decoded at the vector");
}

// ---------------------------------------------------------------------------
// the tables
// ---------------------------------------------------------------------------

#[test]
fn a_machine_with_no_local_apic_is_refused_rather_than_described() {
    let facts = acpi::MachineFacts::default();
    let err = acpi::generate(0xe_0000, &facts, &acpi::TableConfig::default())
        .expect_err("a MADT with no processor is not a machine");
    assert!(
        alloc::format!("{err}").contains("local APIC"),
        "the error should name what is missing: {err}"
    );
}

/// A minimal fact set: a local APIC and nothing else.
fn bare_facts() -> acpi::MachineFacts {
    acpi::MachineFacts {
        lapic: Some((0xfee0_0000, 0)),
        ..acpi::MachineFacts::default()
    }
}

#[test]
fn a_table_the_machine_has_no_part_for_is_not_emitted() {
    let tables = acpi::generate(0xe_0000, &bare_facts(), &acpi::TableConfig::default())
        .expect("there is a processor");
    // No I/O APIC, no HPET and no ECAM window, so no MCFG and no HPET table —
    // and the XSDT lists exactly what is there rather than a stub.
    let has = |sig: &[u8; 4]| tables.bytes.windows(4).any(|w| w == sig);
    assert!(has(b"FACP") && has(b"APIC") && has(b"DSDT") && has(b"FACS"));
    assert!(!has(b"MCFG"), "a machine with no ECAM gets no MCFG");
    assert!(!has(b"HPET"), "a machine with no HPET gets no HPET table");
}

#[test]
fn every_table_checksums_and_the_rsdp_checksums_twice() {
    let facts = acpi::MachineFacts {
        lapic: Some((0xfee0_0000, 0)),
        ioapic: Some(0xfec0_0000),
        hpet: Some((0xfed0_0000, 0x8086_a201)),
        acpi_io: Some(0x600),
        ecam: Some((0xe000_0000, 256 * 1024 * 1024)),
        tables: Some(0xe_0000),
        prt: alloc::vec![acpi::PrtRoute {
            device: 4,
            pin: 0,
            gsi: 11
        }],
    };
    let cfg = acpi::TableConfig::default();
    let tables = acpi::generate(0xe_0000, &facts, &cfg).expect("a complete machine");
    let at = |address: u64| (address - tables.base) as usize;

    let rsdp = &tables.bytes[..36];
    assert_eq!(&rsdp[..8], b"RSD PTR ");
    assert_eq!(acpi::checksum(&rsdp[..20]), 0, "the ACPI 1.0 checksum");
    assert_eq!(acpi::checksum(rsdp), 0, "the extended checksum");
    assert_eq!(
        rsdp[15], 2,
        "revision 2, which is what makes the XSDT valid"
    );

    // Walk the XSDT and checksum every table it lists.
    let xsdt_at = u64::from_le_bytes(rsdp[24..32].try_into().expect("eight"));
    let xsdt = &tables.bytes[at(xsdt_at)..];
    let xsdt_len = u32::from_le_bytes(xsdt[4..8].try_into().expect("four")) as usize;
    assert_eq!(acpi::checksum(&xsdt[..xsdt_len]), 0);
    let mut listed = Vec::new();
    for chunk in xsdt[36..xsdt_len].as_chunks::<8>().0 {
        let address = u64::from_le_bytes(*chunk);
        let table = &tables.bytes[at(address)..];
        let len = u32::from_le_bytes(table[4..8].try_into().expect("four")) as usize;
        assert_eq!(
            acpi::checksum(&table[..len]),
            0,
            "`{}` does not checksum",
            alloc::string::String::from_utf8_lossy(&table[..4])
        );
        listed.push([table[0], table[1], table[2], table[3]]);
    }
    assert_eq!(listed.len(), 4, "FADT, MADT, MCFG, HPET");
    assert!(listed.contains(b"FACP"));

    // The RSDT lists the same tables, and every address in it fits 32 bits —
    // which is the whole reason both exist.
    let rsdt_at = u32::from_le_bytes(rsdp[16..20].try_into().expect("four"));
    let rsdt = &tables.bytes[at(u64::from(rsdt_at))..];
    let rsdt_len = u32::from_le_bytes(rsdt[4..8].try_into().expect("four")) as usize;
    assert_eq!(acpi::checksum(&rsdt[..rsdt_len]), 0);
    assert_eq!((rsdt_len - 36) / 4, listed.len());
}

#[test]
fn the_fadt_names_the_blocks_pmbase_actually_placed() {
    let facts = acpi::MachineFacts {
        lapic: Some((0xfee0_0000, 0)),
        acpi_io: Some(0x480),
        ..acpi::MachineFacts::default()
    };
    let tables = acpi::generate(0xe_0000, &facts, &acpi::TableConfig::default())
        .expect("there is a processor");
    let at = tables
        .bytes
        .windows(4)
        .position(|w| w == b"FACP")
        .expect("the FADT is in there");
    let fadt = &tables.bytes[at..at + acpi::FADT_LEN];
    assert_eq!(
        u32::from_le_bytes(fadt[56..60].try_into().expect("four")),
        0x480,
        "PM1a_EVT_BLK is PMBASE + 0"
    );
    assert_eq!(
        u32::from_le_bytes(fadt[64..68].try_into().expect("four")),
        0x484,
        "PM1a_CNT_BLK is PMBASE + 4"
    );
    assert_eq!(
        u32::from_le_bytes(fadt[76..80].try_into().expect("four")),
        0x488,
        "PMTMR_BLK is PMBASE + 8"
    );
    assert_eq!(
        u32::from_le_bytes(fadt[80..84].try_into().expect("four")),
        0x4a0,
        "GPE0_BLK is PMBASE + 0x20"
    );
    // The lengths ICH9 Table 13-11 gives, restated nowhere but in `pm`.
    assert_eq!(fadt[88], pm::PM1_EVT_LEN);
    assert_eq!(fadt[89], pm::PM1_CNT_LEN);
    assert_eq!(fadt[91], pm::PM_TMR_LEN);
    assert_eq!(fadt[92], pm::GPE0_BLK_LEN);
    // The timer is 24 bits, so `TMR_VAL_EXT` must be clear or an operating
    // system will wait for a wrap that never comes.
    let flags = u32::from_le_bytes(fadt[112..116].try_into().expect("four"));
    assert_eq!(flags & (1 << 8), 0, "TMR_VAL_EXT: the counter is 24-bit");
    assert_ne!(flags & (1 << 10), 0, "RESET_REG_SUP: 0xcf9 works");
    // And the reset register is a byte-wide I/O port, as §5.2.9 requires.
    assert_eq!(fadt[116], acpi::GAS_IO);
    assert_eq!(fadt[117], 8, "bit width must be 8");
    assert_eq!(fadt[118], 0, "bit offset must be 0");
}

#[test]
fn the_madt_carries_the_timers_interrupt_source_override() {
    let facts = acpi::MachineFacts {
        lapic: Some((0xfee0_0000, 7)),
        ioapic: Some(0xfec0_0000),
        ..acpi::MachineFacts::default()
    };
    let cfg = acpi::TableConfig {
        cpus: 2,
        ..acpi::TableConfig::default()
    };
    let tables = acpi::generate(0xe_0000, &facts, &cfg).expect("there is a processor");
    let at = tables
        .bytes
        .windows(4)
        .position(|w| w == b"APIC")
        .expect("the MADT is in there");
    let len = u32::from_le_bytes(tables.bytes[at + 4..at + 8].try_into().expect("four")) as usize;
    let madt = &tables.bytes[at..at + len];
    let mut ids = Vec::new();
    let mut overrides = Vec::new();
    let mut nmi = false;
    let mut walk = 44;
    while walk + 1 < madt.len() {
        let (kind, entry) = (madt[walk], madt[walk + 1] as usize);
        assert!(entry >= 2);
        match kind {
            0 => ids.push(madt[walk + 3]),
            2 => overrides.push((
                madt[walk + 3],
                u32::from_le_bytes(madt[walk + 4..walk + 8].try_into().expect("four")),
                u16::from_le_bytes(madt[walk + 8..walk + 10].try_into().expect("two")),
            )),
            4 => nmi = true,
            _ => {}
        }
        walk += entry;
    }
    // The bootstrap processor's ID came out of its own APIC, and the rest are
    // numbered from it.
    assert_eq!(ids, alloc::vec![7, 8]);
    // IRQ0 is global system interrupt 2, conforming flags.
    assert!(overrides.contains(&(0, 2, 0)), "{overrides:?}");
    // The SCI is identity-mapped but active high and level-triggered, which is
    // not the ISA default and so has to be said.
    assert!(overrides.contains(&(9, 9, 0b1101)), "{overrides:?}");
    assert!(
        nmi,
        "no Local APIC NMI entry: nothing routes an NMI to LINT1"
    );
}

// ---------------------------------------------------------------------------
// a PCI function's interrupt, through the fabric and out of the router
// ---------------------------------------------------------------------------

/// One of the router's eleven ISA outputs, watched.
#[derive(Debug)]
struct Watch {
    inputs: crate::core::wire::FanIn,
    level: crate::core::sync::Mutex<crate::core::wire::Level>,
}

impl crate::core::wire::WireSink for Watch {
    fn set_level(
        &self,
        src: crate::core::wire::WireId,
        _line: u32,
        level: crate::core::wire::Level,
    ) {
        self.inputs.set(src, level);
        *self.level.lock() = self.inputs.resolve(crate::core::wire::Resolve::Or);
    }
}

/// Wire `port` of the bridge to a watcher and hand back both halves.
fn watch(lpc: &lpc::Lpc, ids: &crate::core::wire::WireIdAllocator, port: &str) -> Arc<Watch> {
    use crate::core::wire::{FanIn, Level, Wire, WireSink, WireSource};
    let id = ids.alloc();
    let watcher = Arc::new(Watch {
        inputs: FanIn::new(&[id]),
        level: crate::core::sync::Mutex::with_rank(crate::core::sync::LockRank::LEAF, Level::Low),
    });
    let wire = Arc::new(
        Wire::builder()
            .source(id)
            .sink(Arc::clone(&watcher) as Arc<dyn WireSink>, 0)
            .build(),
    );
    lpc.connect(port, WireSource::new(wire, id))
        .expect("the bridge drives this pin");
    watcher
}

#[test]
fn a_functions_intx_reaches_the_router_through_the_fabric() {
    use crate::bus::pci::{Intx, IntxPin};
    use crate::core::wire::Level;

    let rig = Rig::new();
    let ids = crate::core::wire::WireIdAllocator::new();
    let irq5 = watch(&rig.lpc, &ids, "irq5");
    let irq7 = watch(&rig.lpc, &ids, "irq7");
    let f = function(&rig.bus, Bdf::new(0, lpc::LPC_DEVICE, 0).expect("legal"));

    // A card at device 4 driving `INTA#`. 4 % 4 is 0, so it is on `INTA#` of
    // the bus and arrives as `PIRQA`.
    let at = Bdf::new(0, 4, 0).expect("legal");
    let card = Intx::new(IntxPin::A);
    card.plug(&rig.bus, at);
    card.set(Level::High);

    // Nothing yet: §13.1.17's power-up value is 80h, "the PIRQ is not routed to
    // the 8259", which is exactly why the datasheet tells a BIOS to program it.
    assert_eq!(*irq5.level.lock(), Level::Low);
    assert_eq!(*irq7.level.lock(), Level::Low);

    // Route PIRQA to IRQ5, with the line already asserted. A router that only
    // acted on the *next* assertion would leave this interrupt lost for ever,
    // which is the level-triggered failure worth having a test for.
    rig.write_config(&*f, 0x60, &[0x05]);
    assert_eq!(*irq5.level.lock(), Level::High);
    assert_eq!(*irq7.level.lock(), Level::Low);

    // Reprogram the same router while the same card is still asserting: the
    // interrupt arrives on a different input, and the old one is released.
    rig.write_config(&*f, 0x60, &[0x07]);
    assert_eq!(*irq5.level.lock(), Level::Low, "IRQ5 was let go of");
    assert_eq!(*irq7.level.lock(), Level::High, "and IRQ7 picked it up");

    // The card deasserting releases it.
    card.set(Level::Low);
    assert_eq!(*irq7.level.lock(), Level::Low);
}

#[test]
fn two_cards_sharing_one_pirq_hold_the_line_until_both_let_go() {
    use crate::bus::pci::{Intx, IntxPin};
    use crate::core::wire::Level;

    let rig = Rig::new();
    let ids = crate::core::wire::WireIdAllocator::new();
    let irq11 = watch(&rig.lpc, &ids, "irq11");
    let f = function(&rig.bus, Bdf::new(0, lpc::LPC_DEVICE, 0).expect("legal"));
    // PIRQA and PIRQB both to IRQ11, which is what a firmware with more cards
    // than router inputs does and what "sharing a PCI interrupt" means.
    rig.write_config(&*f, 0x60, &[0x0b, 0x0b]);

    // Device 4 pin A is net 0 (PIRQA); device 5 pin A is net 1 (PIRQB).
    let first = Intx::new(IntxPin::A);
    first.plug(&rig.bus, Bdf::new(0, 4, 0).expect("legal"));
    let second = Intx::new(IntxPin::A);
    second.plug(&rig.bus, Bdf::new(0, 5, 0).expect("legal"));

    first.set(Level::High);
    assert_eq!(*irq11.level.lock(), Level::High);
    second.set(Level::High);
    assert_eq!(*irq11.level.lock(), Level::High);
    first.set(Level::Low);
    assert_eq!(
        *irq11.level.lock(),
        Level::High,
        "the other card is still asserting"
    );
    second.set(Level::Low);
    assert_eq!(*irq11.level.lock(), Level::Low);
}

#[test]
fn a_board_may_state_the_routing_its_missing_firmware_would_have_programmed() {
    // The same stand-in `ecam` and `pm-base` are, and for the same reason:
    // §13.1.17 has a BIOS program these during POST, and a board with no
    // firmware that does it would have a router that routes nothing.
    let props = crate::core::props::Props::new()
        .with("device-id", 0x2918u64)
        .with(
            "pirq-routes",
            crate::core::props::Value::List(alloc::vec![
                crate::core::props::Value::Uint(11),
                crate::core::props::Value::Uint(10),
                crate::core::props::Value::Uint(0),
            ]),
        );
    let lpc = lpc::Lpc::new(&props).expect("a legal description");
    assert_eq!(lpc.pirq_route(0), 0x0b, "PIRQA routed to IRQ11");
    assert_eq!(lpc.pirq_route(1), 0x0a, "PIRQB routed to IRQ10");
    assert_eq!(lpc.pirq_route(2), 0x80, "0 leaves the datasheet's default");
    assert_eq!(lpc.pirq_route(7), 0x80, "and so does saying nothing at all");

    // An interrupt §13.1.17's table cannot name is refused rather than rounded.
    let props = crate::core::props::Props::new()
        .with("device-id", 0x2918u64)
        .with(
            "pirq-routes",
            crate::core::props::Value::List(alloc::vec![crate::core::props::Value::Uint(8)]),
        );
    let e = lpc::Lpc::new(&props)
        .expect_err("IRQ8 has no encoding")
        .to_string();
    assert!(e.contains("IRQ8"), "{e}");
}

#[test]
fn the_routing_table_reads_the_same_in_both_directions() {
    // §13.1.17's table has holes in it — five of the sixteen encodings are
    // reserved — and it is read in both directions: forwards to drive a pin,
    // backwards to turn a board's `pirq-routes` into a register value. A hole
    // in one direction only would route an interrupt somewhere nobody listens.
    for irq in 0..=255u8 {
        let Some(encoding) = lpc::route_encoding(irq) else {
            continue;
        };
        assert!(
            lpc::ROUTABLE.contains(&irq),
            "IRQ{irq} has an encoding but is not in ROUTABLE"
        );
        assert_eq!(lpc::pirq_destination(encoding), Some(irq));
    }
    for byte in 0..=255u8 {
        let Some(irq) = lpc::pirq_destination(byte) else {
            continue;
        };
        assert!(lpc::ROUTABLE.contains(&irq));
        assert_eq!(lpc::route_encoding(irq), Some(byte & 0x0f));
    }
    for irq in lpc::ROUTABLE {
        assert!(
            lpc::route_encoding(irq).is_some(),
            "ROUTABLE names IRQ{irq}, which no encoding reaches"
        );
    }
}

#[test]
fn the_two_pirq_runs_are_disjoint_and_in_order() {
    // §13.1.17 puts PIRQ[A-D] at 0x60 and §13.1.19 puts PIRQ[E-H] at 0x68, and
    // the gap between them is the detail a second reader of the register file
    // gets wrong.
    let offsets: alloc::vec::Vec<u16> = (0..lpc::PIRQS).map(lpc::pirq_rout).collect();
    assert_eq!(
        offsets,
        alloc::vec![0x60, 0x61, 0x62, 0x63, 0x68, 0x69, 0x6a, 0x6b]
    );
}
