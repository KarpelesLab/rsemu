//! The tables the firmware publishes so that an operating system can find the
//! board's processors: MP 1.4, ACPI, and SMBIOS.
//!
//! # What is published, and where it lands
//!
//! Everything below is **data in the ROM image**, laid out by [`generate`] at
//! assembly time and placed at [`super::TABLES_OFFSET`] inside the 64 KiB
//! socket — physical `0xf8000` on `machines/pc-at.machine`. Nothing here is
//! copied at POST, and that is the point rather than an economy: the tables are
//! read-only to an operating system (*MP* §4: "The MP configuration information
//! is intended to be read-only to the operating system") and every search that
//! looks for them is defined over an address range this segment is inside.
//!
//! | Structure | Where a searcher looks | Why here is a legal answer |
//! | --- | --- | --- |
//! | MP floating pointer | the EBDA's first KiB, the last KiB of base memory, then `0xf0000`-`0xfffff` | *MP* §4, third location |
//! | MP configuration table | wherever the floating pointer says | *MP* §4: "non-reported system RAM or […] the BIOS read-only memory space" |
//! | RSDP | the EBDA's first KiB, then `0xe0000`-`0xfffff` | *ACPI* §5.2.5.1, second location |
//! | RSDT, XSDT, FADT, DSDT, MADT | wherever the pointer above them says | *ACPI* §5.2.5.1 |
//! | SMBIOS entry point | 16-byte boundaries in `0xf0000`-`0xfffff` | *SMBIOS* §5.2.1 |
//!
//! *MP* §4 permits the ROM only "if the system is not dynamically
//! reconfigurable", which is exactly what a board assembled from a `.machine`
//! file is: its processors are fixed before the first instruction runs.
//!
//! # Why the tables are not built at POST
//!
//! Because there is nothing to discover. A real BIOS builds these in RAM after
//! starting each application processor and asking it for its APIC ID; here the
//! processors are written down in the machine description, so the image is
//! assembled *for* the board — see [`super::platform`], which is the half of
//! this that reads the board, and [`super::image_for`], which is how a caller
//! says which board. What is left is byte layout, which is what this file is.
//!
//! # Two generators, and whether they can be one
//!
//! `src/dev/q35/acpi.rs` emits the same ACPI set from a *realized* machine,
//! and this file emits it from a *description*. They cannot share code today
//! and the obstacle is neither duplication nor taste:
//!
//! * that generator lives behind the `dev-q35` feature, and `fw-pcbios` must
//!   not imply a chipset it knows nothing about — the firmware is a legacy
//!   BIOS and the board it ships for has no q35 in it;
//! * its input, `MachineFacts`, is built by reading an `AddressSpace` that does
//!   not exist yet when a ROM image is assembled ([`super::platform`]);
//! * and its FADT names `q35.pm`'s register block, which is exactly the
//!   hardware this board does not have.
//!
//! What *would* be shared is the byte layout — the description header, the
//! checksum, the MADT's structures, the Generic Address Structure — and the
//! seam for that is a neutral module both can depend on, taking a facts struct
//! that either a survey or a description can fill. That is a move of files
//! this work does not own; the two generators are deliberately written to the
//! same shape so that the move is mechanical when someone makes it.
//!
//! Where both are present — rsemu's own BIOS in a `q35` board's socket — there
//! are two valid RSDPs in the search window, at `0xe0000` and `0xf8000`. *ACPI*
//! §5.2.5.1 has OSPM take the first it finds, which is the device's, and that
//! is the right outcome: the device describes the machine it was realized in,
//! including the parts this firmware has never heard of.
//!
//! # What is deliberately not published
//!
//! * **No FACS.** *ACPI* §5.2.10's table is the firmware's handshake for sleep
//!   and wake, and the FADT below declares `HW_REDUCED_ACPI` — the AT has no
//!   ACPI hardware register interface at all — for which the specification
//!   states the FACS is not used. Publishing a 64-byte structure in ROM that
//!   the operating system is expected to *write* would be worse than absent.
//! * **No MCFG and no HPET table.** Neither describes a processor, `pc-at`'s
//!   PCI configuration space is the 0xcf8 port pair rather than an ECAM window,
//!   and `src/dev/q35/acpi.rs` already emits both for the board that has them.
//! * **No `_PRT`, no interrupt link devices, no `_S5`.** The DSDT declares an
//!   empty `\_SB` and nothing else: this board has no ACPI power management to
//!   describe, and a sleep package pointing at a control register that does not
//!   exist is a lie an operating system would act on.
//!
//! # Sources
//!
//! * *MultiProcessor Specification* version 1.4 (Intel, May 1997): §4 for where
//!   the structures may live, §4.1 Table 4-1 for the floating pointer, §4.2
//!   Table 4-2 for the configuration table header, §4.3 Table 4-3 for the entry
//!   types and their lengths, §4.3.1 Table 4-4 for a processor entry, §4.3.2
//!   Tables 4-7 and 4-8 for a bus entry and its type strings, §4.3.3 Table 4-9
//!   for an I/O APIC entry, §4.3.4 Tables 4-10 and 4-11 for an I/O interrupt
//!   assignment, and §4.3.5 Table 4-12 for a local interrupt assignment.
//! * *ACPI Specification* revision 6.5 (UEFI Forum): §5.2.5.3 for the RSDP,
//!   §5.2.6 for the description header, §5.2.7 and §5.2.8 for the RSDT and
//!   XSDT, §5.2.9 and Tables 5.9-5.11 for the FADT, §5.2.11.1 for the DSDT,
//!   §5.2.12 and Tables 5.19-5.28 for the MADT, and §20.2 for AML.
//! * *SMBIOS Reference Specification* version 3.x (DMTF DSP0134): §5.2.1 for
//!   the 32-bit entry point, §6.1 for the structure header and the string set,
//!   §7.1 for BIOS Information, §7.2 for System Information, §7.5 for Processor
//!   Information and §7.46 for the end-of-table marker.
//!
//! No firmware source and no emulator source was read (`ROADMAP.md` §1).

use alloc::vec::Vec;

use super::platform::{EXTINT_LINTIN, ISA_BUS_ID, NMI_LINTIN, Platform};

// ---------------------------------------------------------------------------
// what the tables call this firmware
// ---------------------------------------------------------------------------

/// The creator identification an ACPI description header carries (*ACPI*
/// §5.2.6).
const CREATOR_ID: &[u8; 4] = b"RSMU";
/// The creator revision.
const CREATOR_REVISION: u32 = 1;
/// The OEM revision every table here carries (*ACPI* §5.2.6).
const OEM_REVISION: u32 = 1;
/// The eight-character OEM table identification (*ACPI* §5.2.6).
const OEM_TABLE_ID: &[u8; 8] = b"RSEMUPCA";

/// How long an ACPI description header is (*ACPI* §5.2.6, Table 5.4).
const HEADER_LEN: usize = 36;

/// How the structures are aligned against each other.
///
/// Sixteen, because three separate searches — *MP* §4.1, *ACPI* §5.2.5.1 and
/// *SMBIOS* §5.2.1 — all step on 16-byte boundaries, and it costs nothing to
/// keep the tables they point at equally tidy.
const ALIGN: u32 = 16;

// ---------------------------------------------------------------------------
// checksums
// ---------------------------------------------------------------------------

/// The 8-bit sum of `bytes`.
///
/// Every structure here is checked the same way and by three different
/// specifications: *MP* §4.1 ("must add up to zero"), *ACPI* §5.2.6 ("the
/// entire table, including the checksum field, must add to zero") and *SMBIOS*
/// §5.2.1.
#[must_use]
pub fn checksum(bytes: &[u8]) -> u8 {
    bytes.iter().fold(0u8, |sum, b| sum.wrapping_add(*b))
}

/// What to store in a checksum field so that [`checksum`] comes out zero.
fn correction(bytes: &[u8]) -> u8 {
    checksum(bytes).wrapping_neg()
}

// ---------------------------------------------------------------------------
// a table being built
// ---------------------------------------------------------------------------

/// A byte string under construction, with the little-endian appends every one
/// of these layouts is written in.
#[derive(Debug, Default)]
struct Buf {
    bytes: Vec<u8>,
}

impl Buf {
    fn new() -> Buf {
        Buf { bytes: Vec::new() }
    }

    fn u8(&mut self, v: u8) -> &mut Buf {
        self.bytes.push(v);
        self
    }

    fn u16(&mut self, v: u16) -> &mut Buf {
        self.bytes.extend_from_slice(&v.to_le_bytes());
        self
    }

    fn u32(&mut self, v: u32) -> &mut Buf {
        self.bytes.extend_from_slice(&v.to_le_bytes());
        self
    }

    fn u64(&mut self, v: u64) -> &mut Buf {
        self.bytes.extend_from_slice(&v.to_le_bytes());
        self
    }

    fn bytes(&mut self, b: &[u8]) -> &mut Buf {
        self.bytes.extend_from_slice(b);
        self
    }

    /// Pad out to `offset` bytes from the start.
    ///
    /// How the FADT is written: every field goes at the offset *ACPI* Table 5.9
    /// gives it, said out loud where it is written, so a missing field is a
    /// visible gap rather than a silent shift of everything after it.
    fn at(&mut self, offset: usize) -> &mut Buf {
        debug_assert!(self.bytes.len() <= offset, "a field overran its offset");
        self.bytes.resize(offset, 0);
        self
    }

    fn len(&self) -> usize {
        self.bytes.len()
    }

    fn take(self) -> Vec<u8> {
        self.bytes
    }
}

/// Start an ACPI description header (*ACPI* §5.2.6, Table 5.4), leaving
/// `Length` and `Checksum` for [`finish_acpi`].
fn acpi_header(signature: &[u8; 4], revision: u8, platform: &Platform) -> Buf {
    let mut t = Buf::new();
    t.bytes(signature)
        .u32(0)
        .u8(revision)
        .u8(0)
        .bytes(&platform.oem_id)
        .bytes(OEM_TABLE_ID)
        .u32(OEM_REVISION)
        .bytes(CREATOR_ID)
        .u32(CREATOR_REVISION);
    debug_assert_eq!(t.len(), HEADER_LEN);
    t
}

/// Fill in an ACPI description header's `Length` and `Checksum`.
fn finish_acpi(table: Buf) -> Vec<u8> {
    let mut bytes = table.take();
    let len = u32::try_from(bytes.len()).unwrap_or(u32::MAX);
    bytes[4..8].copy_from_slice(&len.to_le_bytes());
    bytes[9] = 0;
    bytes[9] = correction(&bytes);
    debug_assert_eq!(checksum(&bytes), 0);
    bytes
}

// ---------------------------------------------------------------------------
// the MultiProcessor specification's two structures
// ---------------------------------------------------------------------------

/// The revision byte both MP structures carry: `04h` is version 1.4 (*MP*
/// §4.1 and §4.2).
const MP_REVISION: u8 = 4;

/// `MP FEATURE INFORMATION BYTE 2` bit 7, `IMCRP`: the IMCR is present and PIC
/// mode is implemented (*MP* §4.1, Table 4-1).
const MP_IMCRP: u8 = 0x80;

/// How long the MP configuration table's header is (*MP* §4.2, Table 4-2: the
/// last field ends at offset 43).
const MP_HEADER_LEN: usize = 44;

/// `CPU FLAGS` bit 0, `EN`: the processor is usable (*MP* §4.3.1, Table 4-4).
const MP_CPU_ENABLED: u8 = 1 << 0;
/// `CPU FLAGS` bit 1, `BP`: the processor is the bootstrap one.
const MP_CPU_BOOTSTRAP: u8 = 1 << 1;
/// `I/O APIC FLAGS` bit 0, `EN`: the I/O APIC is usable (*MP* §4.3.3).
const MP_IOAPIC_ENABLED: u8 = 1 << 0;

/// `INTERRUPT TYPE` 0, `INT`: a vectored interrupt (*MP* §4.3.4, Table 4-11).
const MP_INT: u8 = 0;
/// `INTERRUPT TYPE` 1, `NMI`.
const MP_NMI: u8 = 1;
/// `INTERRUPT TYPE` 3, `ExtINT`: the vector comes from an external 8259A.
const MP_EXTINT: u8 = 3;

/// `PO`/`EL` both `00b`: polarity and trigger conform to the bus's own
/// specification, which for ISA is active high and edge triggered (*MP*
/// §4.3.4, Table 4-10).
const MP_CONFORMS: u16 = 0;

/// The MP floating pointer structure (*MP* §4.1, Table 4-1).
///
/// Sixteen bytes on a 16-byte boundary, whose whole content is a signature, the
/// physical address of the configuration table, and the two feature bytes that
/// say a configuration table exists at all and whether the IMCR does.
#[must_use]
pub fn mp_floating_pointer(config_at: u32, platform: &Platform) -> Vec<u8> {
    let mut t = Buf::new();
    t.bytes(b"_MP_").u32(config_at);
    // LENGTH, in 16-byte paragraphs: "The structure is 16 bytes or 1 paragraph
    // long; so this field contains 01h."
    t.u8(1).u8(MP_REVISION).u8(0);
    // MP FEATURE INFORMATION BYTE 1: zero means the configuration table is
    // present. A non-zero value would name one of Chapter 5's default
    // configurations, and §4 forbids those for a board whose processor count
    // varies — which is exactly what a `.machine` file makes it.
    t.u8(0);
    t.u8(if platform.imcr { MP_IMCRP } else { 0 });
    // MP FEATURE INFORMATION BYTES 3-5: "Reserved for future MP definitions.
    // Must be zero."
    t.bytes(&[0, 0, 0]);
    let mut bytes = t.take();
    debug_assert_eq!(bytes.len(), 16);
    bytes[10] = correction(&bytes);
    debug_assert_eq!(checksum(&bytes), 0);
    bytes
}

/// The MP configuration table: a header and the entries, sorted by entry type
/// as *MP* §4.3 requires ("The entries are sorted on ENTRY TYPE in ascending
/// order").
#[must_use]
pub fn mp_config(platform: &Platform) -> Vec<u8> {
    let mut entries = Buf::new();
    let mut count: u16 = 0;

    // Type 0, twenty bytes: one per processor (*MP* §4.3.1).
    for cpu in &platform.processors {
        let mut flags = MP_CPU_ENABLED;
        if cpu.bootstrap {
            flags |= MP_CPU_BOOTSTRAP;
        }
        entries
            .u8(0)
            .u8(cpu.apic_id)
            .u8(cpu.apic_version)
            .u8(flags)
            .u32(cpu.signature)
            .u32(cpu.features)
            // Two reserved double words.
            .u64(0);
        count += 1;
    }

    // Type 1, eight bytes: the bus the interrupt sources below are on. The
    // string is six characters, blank-filled, and *MP* Table 4-8 spells the
    // Industry Standard Architecture "ISA".
    entries.u8(1).u8(ISA_BUS_ID).bytes(b"ISA   ");
    count += 1;

    if let Some(ioapic) = platform.ioapic {
        // Type 2, eight bytes (*MP* §4.3.3).
        entries
            .u8(2)
            .u8(ioapic.id)
            .u8(ioapic.version)
            .u8(MP_IOAPIC_ENABLED)
            .u32(ioapic.address);
        count += 1;

        // Type 3, eight bytes each: one per bus interrupt source that reaches
        // the I/O APIC (*MP* §4.3.4).
        for irq in &platform.interrupts {
            let intin = u8::try_from(irq.gsi.saturating_sub(ioapic.gsi_base)).unwrap_or(u8::MAX);
            entries
                .u8(3)
                .u8(MP_INT)
                .u16(MP_CONFORMS)
                .u8(ISA_BUS_ID)
                .u8(irq.irq)
                .u8(ioapic.id)
                .u8(intin);
            count += 1;
        }
    }

    // Type 4, eight bytes each (*MP* §4.3.5). Both are addressed to local APIC
    // ID 0FFh, which Table 4-12's I/O equivalent defines as every one of them:
    // an operating system that starts an application processor gets the same
    // local wiring on it.
    if platform.extint {
        entries
            .u8(4)
            .u8(MP_EXTINT)
            .u16(MP_CONFORMS)
            .u8(ISA_BUS_ID)
            .u8(0)
            .u8(0xff)
            .u8(EXTINT_LINTIN);
        count += 1;
    }
    entries
        .u8(4)
        .u8(MP_NMI)
        .u16(MP_CONFORMS)
        .u8(ISA_BUS_ID)
        .u8(0)
        .u8(0xff)
        .u8(NMI_LINTIN);
    count += 1;

    let entries = entries.take();
    let mut t = Buf::new();
    t.bytes(b"PCMP");
    t.u16(u16::try_from(MP_HEADER_LEN + entries.len()).unwrap_or(u16::MAX));
    t.u8(MP_REVISION).u8(0);
    t.bytes(&platform.oem_id).bytes(b"  ");
    t.bytes(&platform.product_id);
    // OEM TABLE POINTER and OEM TABLE SIZE: there is no OEM table.
    t.u32(0).u16(0);
    t.u16(count);
    t.u32(platform.lapic);
    // EXTENDED TABLE LENGTH and its checksum: "A zero value in this field
    // indicates that no extended entries are present."
    t.u16(0).u8(0).u8(0);
    debug_assert_eq!(t.len(), MP_HEADER_LEN);
    t.bytes(&entries);

    let mut bytes = t.take();
    bytes[7] = correction(&bytes);
    debug_assert_eq!(checksum(&bytes), 0);
    bytes
}

// ---------------------------------------------------------------------------
// ACPI
// ---------------------------------------------------------------------------

/// The RSDP revision that makes the XSDT half of the structure valid (*ACPI*
/// §5.2.5.3, Table 5.3: revision 2 is "ACPI 2.0 and later").
const RSDP_REVISION: u8 = 2;
/// How long an ACPI 2.0-and-later RSDP is.
const RSDP_LEN: usize = 36;

/// The FADT revision this firmware emits, and why it is not the sibling
/// generator's.
///
/// `src/dev/q35/acpi.rs` emits revision 3 because the board it describes *has*
/// an ACPI register block to point the PM1 and GPE fields at. This board has
/// none: no PM1 event or control block, no power management timer, no SCI, and
/// no SMI command port. Revision 3 has no way to say that — every one of those
/// fields would be a zero the specification calls required — while revision 5
/// introduced the flag that says it exactly, `HW_REDUCED_ACPI`. So the choice
/// is between a revision 3 table that claims registers at address zero and a
/// revision 6 table that states there is no ACPI hardware register interface,
/// and the second is true.
const FADT_REVISION: u8 = 6;
/// The FADT length that goes with [`FADT_REVISION`]: *ACPI* 6.x's last field is
/// `Hypervisor Vendor Identity`, which ends at 276.
const FADT_LEN: usize = 276;

/// `Flags[2]`: `PROC_C1`, C1 is supported on every processor (*ACPI* Table
/// 5.10).
const FADT_PROC_C1: u32 = 1 << 2;
/// `Flags[4]`: `PWR_BUTTON`, the power button is a control method device —
/// which is how Table 5.10 spells "there is no fixed power button".
const FADT_PWR_BUTTON: u32 = 1 << 4;
/// `Flags[5]`: likewise the sleep button.
const FADT_SLP_BUTTON: u32 = 1 << 5;
/// `Flags[20]`: `HW_REDUCED_ACPI`, the ACPI hardware register interface is not
/// implemented.
const FADT_HW_REDUCED: u32 = 1 << 20;

/// `IAPC_BOOT_ARCH[0]`: there are user-visible devices on the LPC or ISA bus
/// (*ACPI* Table 5.11).
const IAPC_LEGACY_DEVICES: u16 = 1 << 0;
/// `IAPC_BOOT_ARCH[1]`: a port 60/64 keyboard controller is present.
const IAPC_8042: u16 = 1 << 1;

/// The CMOS index the century byte lives at, which is what the FADT's `CENTURY`
/// field names (*ACPI* §5.2.9).
///
/// `0x32`, the AT's own choice, and what `dev::pc::rtc` answers and
/// `src/fw/pcbios/system.rs` reads for `INT 1Ah AH=04h`.
const CMOS_CENTURY: u8 = 0x32;

/// `Preferred_PM_Profile` 1: Desktop (*ACPI* §5.2.9, Table 5.9).
const PM_PROFILE_DESKTOP: u8 = 1;

/// The RSDP (*ACPI* §5.2.5.3, Table 5.3).
#[must_use]
pub fn rsdp(platform: &Platform, rsdt_at: u32, xsdt_at: u32) -> Vec<u8> {
    let mut t = Buf::new();
    // "RSD PTR " — the trailing blank is part of the signature.
    t.bytes(b"RSD PTR ").u8(0).bytes(&platform.oem_id);
    t.u8(RSDP_REVISION).u32(rsdt_at);
    t.u32(u32::try_from(RSDP_LEN).unwrap_or(u32::MAX));
    t.u64(u64::from(xsdt_at)).u8(0).bytes(&[0, 0, 0]);
    let mut bytes = t.take();
    debug_assert_eq!(bytes.len(), RSDP_LEN);
    // "the first 20 bytes of this table, bytes 0 to 19, including the checksum
    // field" for the first sum, and "the entire table" for the second.
    bytes[8] = correction(&bytes[..20]);
    bytes[32] = correction(&bytes);
    debug_assert_eq!(checksum(&bytes[..20]), 0);
    debug_assert_eq!(checksum(&bytes), 0);
    bytes
}

/// The RSDT: 32-bit pointers to everything but the FACS and the DSDT (*ACPI*
/// §5.2.7).
#[must_use]
pub fn rsdt(platform: &Platform, tables: &[u32]) -> Vec<u8> {
    let mut t = acpi_header(b"RSDT", 1, platform);
    for address in tables {
        t.u32(*address);
    }
    finish_acpi(t)
}

/// The XSDT, which is the same list in 64-bit pointers (*ACPI* §5.2.8).
#[must_use]
pub fn xsdt(platform: &Platform, tables: &[u32]) -> Vec<u8> {
    let mut t = acpi_header(b"XSDT", 1, platform);
    for address in tables {
        t.u64(u64::from(*address));
    }
    finish_acpi(t)
}

/// The DSDT: a definition block with an empty `\_SB` scope (*ACPI* §5.2.11.1).
///
/// It is nearly empty because the board is: there is no ACPI-defined hardware
/// on an AT to describe, no power resources, no interrupt link devices and no
/// sleep states. What it must be is *present and well formed*, because the FADT
/// points at it and an operating system that cannot load the differentiated
/// definition block gives up on ACPI altogether — including on the MADT it had
/// already read the processors out of.
#[must_use]
pub fn dsdt(platform: &Platform) -> Vec<u8> {
    // Revision 2 or above is what makes an integer 64 bits wide (*ACPI*
    // §5.2.11.1); nothing here is an integer, and the revision is 2 so that a
    // term added later does not silently change width.
    let mut t = acpi_header(b"DSDT", 2, platform);
    // `Scope (\_SB) {}`. AML §20.2.5.1: ScopeOp `10h`, a PkgLength that counts
    // itself, then the name. `\_SB_` is RootChar plus one four-character
    // NameSeg, blank-padded with `_` as §20.2.2 requires.
    t.u8(0x10).u8(6).bytes(b"\\_SB_");
    finish_acpi(t)
}

/// The MADT: the processors and the interrupt controllers (*ACPI* §5.2.12).
///
/// This is the table the whole file exists for. Every processor the machine
/// description declares gets a Processor Local APIC structure, every ISA
/// interrupt whose global system interrupt differs from its IRQ number gets an
/// Interrupt Source Override, and the I/O APIC gets one of its own.
#[must_use]
pub fn madt(platform: &Platform) -> Vec<u8> {
    let mut t = acpi_header(b"APIC", 6, platform);
    // Table 5.19: the address every processor reaches its own local APIC at,
    // then the Multiple APIC flags.
    t.u32(platform.lapic);
    // Table 5.20: `PCAT_COMPAT`. True — the board has the 8259A pair, and an
    // operating system moving to the APICs has to mask them.
    t.u32(1);

    for (uid, cpu) in platform.processors.iter().enumerate() {
        // Table 5.22: type 0, length 8, the ACPI processor UID, the APIC ID,
        // and Table 5.23's flags — bit 0, Enabled.
        t.u8(0)
            .u8(8)
            .u8(u8::try_from(uid).unwrap_or(u8::MAX))
            .u8(cpu.apic_id)
            .u32(1);
    }

    if let Some(ioapic) = platform.ioapic {
        // Table 5.24: type 1, length 12.
        t.u8(1)
            .u8(12)
            .u8(ioapic.id)
            .u8(0)
            .u32(ioapic.address)
            .u32(ioapic.gsi_base);

        // Table 5.25: an Interrupt Source Override for every ISA interrupt that
        // does not arrive on the global system interrupt of the same number.
        // On the AT that is the timer and only the timer — `pit0.out0` reaches
        // `pic1.ir0` and `ioapic.irq2` — and an operating system that misses it
        // loses its tick the moment it stops using the 8259A.
        for irq in &platform.interrupts {
            if irq.gsi == u32::from(irq.irq) {
                continue;
            }
            t.u8(2).u8(10).u8(0).u8(irq.irq).u32(irq.gsi);
            // Table 5.26: `00b` polarity and `00b` trigger mode, conforming to
            // the ISA bus, which is active high and edge triggered.
            t.u16(0);
        }
    }

    // Table 5.28: type 4, length 6. `0xff` is every processor, and `LINTIN1` is
    // the NMI, which is how a PC has always been wired.
    t.u8(4).u8(6).u8(0xff).u16(0).u8(NMI_LINTIN);
    finish_acpi(t)
}

/// The FADT (*ACPI* §5.2.9, Table 5.9).
///
/// Every field is written at the offset the table gives it, with the padding
/// helper asserting that the one before it did not overrun. What is *not*
/// written is as deliberate: `SMI_CMD`, `ACPI_ENABLE`, the four PM blocks, the GPE blocks
/// and `RESET_REG` are all absent, because this board has none of them — which
/// is what `HW_REDUCED_ACPI` in the flags says out loud.
#[must_use]
pub fn fadt(platform: &Platform, dsdt_at: u32) -> Vec<u8> {
    let mut t = acpi_header(b"FACP", FADT_REVISION, platform);
    // 36: FIRMWARE_CTRL. Zero: there is no FACS, and §5.2.10's structure is not
    // used by a hardware-reduced platform.
    t.at(36).u32(0);
    t.at(40).u32(dsdt_at);
    // 44 is reserved (it was ACPI 1.0's INT_MODEL).
    t.at(45).u8(PM_PROFILE_DESKTOP);
    // 46: SCI_INT. There is no system control interrupt on this board.
    t.at(108).u8(CMOS_CENTURY);
    t.at(109).u16(if platform.kbc {
        IAPC_LEGACY_DEVICES | IAPC_8042
    } else {
        IAPC_LEGACY_DEVICES
    });
    t.at(112)
        .u32(FADT_PROC_C1 | FADT_PWR_BUTTON | FADT_SLP_BUTTON | FADT_HW_REDUCED);
    // 132: X_FIRMWARE_CTRL, as absent as its 32-bit half.
    t.at(132).u64(0);
    t.at(140).u64(u64::from(dsdt_at));
    t.at(FADT_LEN);
    finish_acpi(t)
}

// ---------------------------------------------------------------------------
// SMBIOS
// ---------------------------------------------------------------------------

/// The SMBIOS version the entry point declares.
const SMBIOS_MAJOR: u8 = 2;
/// The minor half of it. 2.8 is the last of the 32-bit entry point's line, and
/// the structures below are all 2.6-or-earlier shapes.
const SMBIOS_MINOR: u8 = 8;
/// How long the 32-bit entry point is (*SMBIOS* §5.2.1).
const SMBIOS_EPS_LEN: u8 = 0x1f;

/// One SMBIOS structure under construction: a header, its formatted area, and
/// the string set that follows it (*SMBIOS* §6.1).
struct Structure {
    body: Buf,
    strings: Vec<u8>,
    count: u8,
}

impl Structure {
    /// Start a structure of `kind` with `handle`. The length byte is filled in
    /// by [`finish`](Structure::finish).
    fn new(kind: u8, handle: u16) -> Structure {
        let mut body = Buf::new();
        body.u8(kind).u8(0).u16(handle);
        Structure {
            body,
            strings: Vec::new(),
            count: 0,
        }
    }

    /// Add `text` to the string set and answer its 1-based index, which is what
    /// a string field holds. A field with no string holds 0 (*SMBIOS* §6.1.3).
    fn string(&mut self, text: &str) -> u8 {
        self.strings.extend_from_slice(text.as_bytes());
        self.strings.push(0);
        self.count += 1;
        self.count
    }

    /// The formatted area, for the fields this structure defines.
    fn field(&mut self) -> &mut Buf {
        &mut self.body
    }

    /// The whole structure: the formatted area with its length filled in, then
    /// the string set, then the double null that ends it.
    fn finish(mut self) -> Vec<u8> {
        let len = u8::try_from(self.body.len()).unwrap_or(u8::MAX);
        let mut bytes = self.body.take();
        bytes[1] = len;
        if self.strings.is_empty() {
            // "If there are no strings, this is terminated with two null
            // bytes" (*SMBIOS* §6.1.3).
            bytes.push(0);
        } else {
            bytes.append(&mut self.strings);
        }
        bytes.push(0);
        bytes
    }
}

/// `BIOS Characteristics` bit 4: ISA is supported (*SMBIOS* §7.1.1).
const BIOS_ISA: u64 = 1 << 4;
/// Bit 22: the Enhanced Disk Drive specification is supported, which is what
/// `INT 13h AH=4xh` in `src/fw/pcbios/disk.rs` is.
const BIOS_EDD: u64 = 1 << 22;
/// Bit 28: `INT 09h`, 8042 keyboard services are supported.
const BIOS_8042: u64 = 1 << 28;
/// Bit 31: `INT 10h` CGA/monochrome video services are supported.
const BIOS_VIDEO: u64 = 1 << 31;
/// Characteristics extension byte 1, bit 0: ACPI is supported — which it is,
/// three tables up.
const BIOS_EXT_ACPI: u8 = 1 << 0;
/// Characteristics extension byte 2, bit 4: the system is a virtual machine.
const BIOS_EXT_VM: u8 = 1 << 4;

/// The SMBIOS structure table: BIOS Information, System Information, one
/// Processor Information per processor, and the end-of-table marker.
///
/// Nothing in the tree reads this yet — `ROADMAP.md` phase 6a names SMBIOS
/// beside ACPI and it is cheap once the processors have already been read off
/// the machine, which is the only field here that is not a constant.
///
/// Answers the structures, how many there are, and how long the largest one is:
/// the entry point declares all three and they are only knowable here.
#[must_use]
pub fn smbios_structures(platform: &Platform) -> (Vec<u8>, u16, u16) {
    let mut out: Vec<u8> = Vec::new();
    let mut count: u16 = 0;
    let mut largest: u16 = 0;
    let mut handle: u16 = 0;
    let mut push = |s: Vec<u8>, count: &mut u16, largest: &mut u16| {
        *largest = (*largest).max(u16::try_from(s.len()).unwrap_or(u16::MAX));
        *count += 1;
        out.extend_from_slice(&s);
    };

    // Type 0, BIOS Information (*SMBIOS* §7.1). The 2.4 form, whose last field
    // is the embedded controller version at offset 23.
    let mut bios = Structure::new(0, handle);
    let vendor = bios.string("rsemu");
    let version = bios.string("1.0");
    let date = bios.string(core::str::from_utf8(super::BIOS_DATE).unwrap_or("01/01/26"));
    bios.field().u8(vendor).u8(version);
    // The starting address segment, and the ROM size as `(n / 64K) - 1`.
    bios.field().u16(super::SEGMENT).u8(date).u8(0);
    bios.field()
        .u64(BIOS_ISA | BIOS_EDD | BIOS_8042 | BIOS_VIDEO);
    bios.field().u8(BIOS_EXT_ACPI).u8(BIOS_EXT_VM);
    // The system BIOS release, and an embedded controller that is not present
    // (0xff/0xff, *SMBIOS* §7.1).
    bios.field().u8(1).u8(0).u8(0xff).u8(0xff);
    push(bios.finish(), &mut count, &mut largest);
    handle += 1;

    // Type 1, System Information (*SMBIOS* §7.2). The 2.4 form.
    let mut system = Structure::new(1, handle);
    let manufacturer = system.string("rsemu");
    let product = system.string(product_name(platform));
    let version = system.string("1.0");
    system
        .field()
        .u8(manufacturer)
        .u8(product)
        .u8(version)
        .u8(0);
    // The UUID. All zeros, which §7.2.1 defines as "not present" and is the
    // only value a reproducible image can carry.
    system.field().u64(0).u64(0);
    // Wake-up type 6, Power Switch.
    system.field().u8(6).u8(0).u8(0);
    push(system.finish(), &mut count, &mut largest);
    handle += 1;

    // Type 4, Processor Information, one per processor (*SMBIOS* §7.5). The 2.6
    // form, whose last field is the processor family's 16-bit form.
    for (index, cpu) in platform.processors.iter().enumerate() {
        let mut entry = Structure::new(4, handle);
        let socket = entry.string(&alloc::format!("CPU{index}"));
        let manufacturer = entry.string("rsemu");
        entry.field().u8(socket);
        // Processor type 3, Central Processor.
        entry
            .field()
            .u8(3)
            .u8(family(cpu.signature))
            .u8(manufacturer);
        // The processor ID: the CPUID signature in the low double word and the
        // feature flags in the high one, which is the EAX/EDX pair §7.5.3 asks
        // for.
        entry.field().u32(cpu.signature).u32(cpu.features);
        // Version string, voltage, external clock.
        entry.field().u8(0).u8(0).u16(0);
        entry.field().u16(cpu.mhz).u16(cpu.mhz);
        // Status: bit 6 socket populated, bits 2:0 = 1, CPU enabled. Upgrade
        // 2, Unknown — this board's processors are not in sockets anything
        // could upgrade.
        entry.field().u8(0x41).u8(2);
        // The three cache handles, none of which is provided (0xffff).
        entry.field().u16(0xffff).u16(0xffff).u16(0xffff);
        // Serial number, asset tag and part number: no strings.
        entry.field().u8(0).u8(0).u8(0);
        // One core, enabled, one thread. Characteristics: none claimed.
        entry.field().u8(1).u8(1).u8(1).u16(0);
        entry.field().u16(u16::from(family(cpu.signature)));
        push(entry.finish(), &mut count, &mut largest);
        handle += 1;
    }

    // Type 127, End-of-Table (*SMBIOS* §7.46).
    let end = Structure::new(127, handle);
    push(end.finish(), &mut count, &mut largest);

    (out, count, largest)
}

/// The SMBIOS processor family code for an MP CPU signature.
///
/// *SMBIOS* §7.5.2's table: `05h` is an Intel386, `06h` an Intel486, and `02h`
/// is Unknown, which is the honest answer for anything else.
fn family(signature: u32) -> u8 {
    match signature & 0x0f00 {
        0x0300 => 0x05,
        0x0400 => 0x06,
        _ => 0x02,
    }
}

/// The machine's name, out of the MP product identification's blank padding.
fn product_name(platform: &Platform) -> &str {
    let text = core::str::from_utf8(&platform.product_id).unwrap_or("rsemu");
    text.trim_end()
}

/// The 32-bit SMBIOS entry point (*SMBIOS* §5.2.1).
///
/// `length` is how many bytes the structure table occupies and `largest` how
/// big its biggest structure is; the two are different fields and an operating
/// system sizes its read buffer from the second.
#[must_use]
pub fn smbios_entry_point(structures_at: u32, length: u16, count: u16, largest: u16) -> Vec<u8> {
    let mut t = Buf::new();
    t.bytes(b"_SM_").u8(0).u8(SMBIOS_EPS_LEN);
    t.u8(SMBIOS_MAJOR).u8(SMBIOS_MINOR);
    // The largest structure, the entry point revision (0: no formatted area),
    // and the five formatted bytes it would have used.
    t.u16(largest).u8(0).bytes(&[0; 5]);
    // The intermediate anchor, which is what an operating system that skipped
    // the first four bytes looks for.
    t.bytes(b"_DMI_").u8(0);
    t.u16(length).u32(structures_at).u16(count);
    // The BCD revision: 2.8.
    t.u8(0x28);
    let mut bytes = t.take();
    debug_assert_eq!(bytes.len(), usize::from(SMBIOS_EPS_LEN));
    // "the intermediate entry point structure […] bytes 10h to 1Eh" for the
    // second sum, and the whole structure for the first.
    bytes[21] = correction(&bytes[16..]);
    bytes[4] = correction(&bytes);
    debug_assert_eq!(checksum(&bytes), 0);
    debug_assert_eq!(checksum(&bytes[16..]), 0);
    bytes
}

// ---------------------------------------------------------------------------
// laying them out
// ---------------------------------------------------------------------------

/// Everything the generator produced, and where each structure a searcher looks
/// for ended up.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Tables {
    /// The bytes, laid out to be placed at [`base`](Tables::base).
    pub bytes: Vec<u8>,
    /// The guest-physical address the bytes have been laid out for.
    pub base: u32,
    /// Where the MP floating pointer structure ended up.
    pub mp_pointer: u32,
    /// Where the MP configuration table ended up.
    pub mp_config: u32,
    /// Where the RSDP ended up.
    pub rsdp: u32,
    /// Where the RSDT ended up.
    pub rsdt: u32,
    /// Where the XSDT ended up.
    pub xsdt: u32,
    /// Where the FADT ended up.
    pub fadt: u32,
    /// Where the DSDT ended up.
    pub dsdt: u32,
    /// Where the MADT ended up.
    pub madt: u32,
    /// Where the SMBIOS entry point ended up.
    pub smbios: u32,
}

/// Round `at` up to a multiple of [`ALIGN`].
fn align_up(at: u32) -> u32 {
    at.div_ceil(ALIGN) * ALIGN
}

/// Lay the whole set out to be placed at `base`.
///
/// The order is the one a reader wants: the two structures that are *searched
/// for* come first, so that a hexadecimal dump of `0xf8000` opens with `_MP_`
/// and `RSD PTR `.
#[must_use]
pub fn generate(base: u32, platform: &Platform) -> Tables {
    // Every table's length is known before any address is assigned, which is
    // what makes a single pass possible: the two that name addresses (the FADT
    // and the root tables) are built last, from addresses already fixed.
    let mp_config_bytes = mp_config(platform);
    let dsdt_bytes = dsdt(platform);
    let madt_bytes = madt(platform);
    let (smbios_bytes, smbios_count, smbios_largest) = smbios_structures(platform);
    // The FADT and the MADT are what the root tables list; the DSDT is not,
    // because the FADT points at it directly (*ACPI* §5.2.8).
    const LISTED: u32 = 2;

    let mut at = base;
    let mp_pointer = at;
    at = align_up(at + 16);
    let mp_config_at = at;
    at = align_up(at + u32::try_from(mp_config_bytes.len()).unwrap_or(0));
    let rsdp_at = at;
    at = align_up(at + u32::try_from(RSDP_LEN).unwrap_or(0));
    let dsdt_at = at;
    at = align_up(at + u32::try_from(dsdt_bytes.len()).unwrap_or(0));
    let madt_at = at;
    at = align_up(at + u32::try_from(madt_bytes.len()).unwrap_or(0));
    let fadt_at = at;
    at = align_up(at + u32::try_from(FADT_LEN).unwrap_or(0));
    let xsdt_at = at;
    at = align_up(at + u32::try_from(HEADER_LEN).unwrap_or(0) + 8 * LISTED);
    let rsdt_at = at;
    at = align_up(at + u32::try_from(HEADER_LEN).unwrap_or(0) + 4 * LISTED);
    let smbios_at = at;
    at = align_up(at + u32::from(SMBIOS_EPS_LEN));
    let smbios_structures_at = at;
    at = align_up(at + u32::try_from(smbios_bytes.len()).unwrap_or(0));

    let listed = [fadt_at, madt_at];
    debug_assert_eq!(listed.len(), LISTED as usize);
    let mut bytes = alloc::vec![0u8; (at - base) as usize];
    {
        let mut put = |address: u32, data: &[u8]| {
            let start = (address - base) as usize;
            bytes[start..start + data.len()].copy_from_slice(data);
        };
        put(mp_pointer, &mp_floating_pointer(mp_config_at, platform));
        put(mp_config_at, &mp_config_bytes);
        put(rsdp_at, &rsdp(platform, rsdt_at, xsdt_at));
        put(dsdt_at, &dsdt_bytes);
        put(madt_at, &madt_bytes);
        put(fadt_at, &fadt(platform, dsdt_at));
        put(xsdt_at, &xsdt(platform, &listed));
        put(rsdt_at, &rsdt(platform, &listed));
        put(
            smbios_at,
            &smbios_entry_point(
                smbios_structures_at,
                u16::try_from(smbios_bytes.len()).unwrap_or(u16::MAX),
                smbios_count,
                smbios_largest,
            ),
        );
        put(smbios_structures_at, &smbios_bytes);
    }
    Tables {
        bytes,
        base,
        mp_pointer,
        mp_config: mp_config_at,
        rsdp: rsdp_at,
        rsdt: rsdt_at,
        xsdt: xsdt_at,
        fadt: fadt_at,
        dsdt: dsdt_at,
        madt: madt_at,
        smbios: smbios_at,
    }
}
