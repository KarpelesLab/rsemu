//! The ACPI tables, **generated from the realized machine**.
//!
//! # The rule this module exists to obey
//!
//! `docs/platforms/riscv-virt.md` states it for a device tree and it is the
//! same claim here:
//!
//! > rsemu **generates** the device tree from the realized machine graph and
//! > passes it to firmware. That is a genuine test of the machine model: if the
//! > DTB can be produced mechanically from the topology, the topology is
//! > well-formed.
//!
//! So an address is never written down in this file. Every one is read back out
//! of the machine that was actually built:
//!
//! | Table field | Where it comes from |
//! | --- | --- |
//! | `MADT` Local Interrupt Controller Address | the base of the mapping whose region is the local APIC's |
//! | `MADT` Local APIC ID | a **debug read** of that APIC's own ID register |
//! | `MADT` I/O APIC Address | the base of the I/O APIC's mapping |
//! | `MCFG` base and bus range | the base and length of the mapping [`super::mch`] made for `PCIEXBAR` |
//! | `FADT` `PM1a_EVT_BLK`, `PM1a_CNT_BLK`, `PMTMR_BLK`, `GPE0_BLK` | the base of the mapping [`super::lpc`] made for `PMBASE`, plus Table 13-11's offsets |
//! | `HPET` Address and Event Timer Block ID | the HPET's mapping, and a debug read of its capabilities register |
//! | the address of every table | where the machine file mapped this device's own region |
//!
//! The two debug reads deserve a sentence, because "read the guest's own
//! hardware to describe it" is either elegant or a trap. Both registers are
//! read-only capability registers with no side effect at all — the local APIC's
//! ID at offset `20h` (*Intel SDM* Vol 3A §11.4.6) and the HPET's
//! `GCAP_ID` at offset `0` (*IA-PC HPET Specification* §2.3.4) — and both are
//! read with `MemAttrs::debug` set, so a device that did have a side effect
//! would be obliged to suppress it. Reading an *indexed* register this way
//! would not be safe, which is exactly why the **I/O APIC's** ID is declared
//! rather than read: reaching it means writing the index register first, and
//! that is a side effect. See [`TableConfig::ioapic_id`].
//!
//! # What is declared rather than derived, and why
//!
//! [`TableConfig`], and each field is a seam rather than a preference:
//!
//! * **The processor count and their APIC IDs.** A processor is not a region in
//!   any address space and there is no route from a `dyn Device` to a CPU
//!   (`core::device` keeps `Any` out of its supertrait chain deliberately) —
//!   the same limitation `dev::riscv::dt::CpuSpec` documents for harts. The
//!   *bootstrap* processor's ID is read from its APIC and cross-checked against
//!   the declaration, so the number cannot silently drift; the rest follow it.
//! * **The I/O APIC's ID and global system interrupt base**, for the reason
//!   above.
//! * **The SCI's interrupt.** It is `ACPI_CNTL[2:0]`'s to choose at run time and
//!   the board's to wire, and the table is written at reset when neither has
//!   happened yet.
//!
//! Closing all three needs a publication table like `dev::riscv::dt`'s, which
//! devices under `dev/pc/` would publish into. That is a change to files this
//! work does not own; the constants below are named so it is obvious what would
//! replace them.
//!
//! # How the tables reach the guest
//!
//! This device owns a `RamStore`, regenerates the whole set into it at **reset**
//! — the first moment the whole machine graph exists, which is the same
//! argument `dev::riscv::dt` makes — and publishes it as one region the machine
//! file maps. `machines/q35.machine` puts it at `0xe0000`, which is inside
//! ACPI §5.2.5.1's own search window:
//!
//! > OSPM finds the Root System Description Pointer (RSDP) structure by
//! > searching physical memory ranges on 16-byte boundaries for a valid Root
//! > System Description Pointer structure signature and checksum match […] in
//! > the BIOS read-only memory space between 0E0000h and 0FFFFFh.
//!
//! **On a real machine the firmware would allocate this and report it in the
//! E820 map as ACPI reclaim memory**, and that is where this should end up:
//! `generate` is a free function precisely so that `src/fw/pcbios` can call it
//! and stage the bytes itself, at which point this device and its mapping go
//! away. The seam is [`generate`] plus [`MachineFacts`]; nothing else here is
//! public API anyone should build on.
//!
//! Note what this means today: a *third-party* firmware that programs the PAM
//! registers to shadow `0xe0000` will hide these tables behind its own DRAM,
//! and it should — it would be publishing its own. rsemu's own BIOS does not
//! touch PAM, so on the default board the tables stay visible.
//!
//! # Sources
//!
//! *ACPI Specification*, revision 6.5 (UEFI Forum, openly published): §5.2.5.3
//! for the RSDP, §5.2.6 for the description header, §5.2.9 and Tables 5.9-5.11
//! for the FADT, §5.2.10 for the FACS, §5.2.12 and Tables 5.19-5.28 for the
//! MADT, §5.2.3.2 for the Generic Address Structure, and §20.2 for AML.
//! *IA-PC HPET Specification* revision 1.0a §3.2.4 for the HPET table.
//! *Intel I/O Controller Hub 9 (ICH9) Family Datasheet* 316972-004 Table 13-11
//! for the ACPI register block's offsets.
//!
//! **`MCFG` is the one layout not read from its own specification.** It is
//! defined in the *PCI Firmware Specification* revision 3.0 §4.1.2, Tables 4-2
//! and 4-3 — ACPI 6.5's own Table 5.6 reserves the signature and refers the
//! layout out to that document — and the specification is members-only at
//! PCI-SIG and could not be obtained. The layout below is the OSDev wiki's,
//! corroborated against the Ubuntu Firmware Test Suite's `mcfg` reference,
//! which cites Rev 3.0. That is a weaker citation than every other table here
//! and is flagged rather than smoothed over.
//!
//! No emulator source and no firmware source was consulted (`CLAUDE.md`,
//! provenance).

use alloc::boxed::Box;
use alloc::string::{String, ToString};
use alloc::sync::Arc;
use alloc::vec::Vec;

use crate::bus::pci::{Bdf, INTX_LINES, IntxPin, MAX_DEVICE, PciBus, buses, config, swizzle};
use crate::core::device::{Device, DeviceClass, PropertySpec, RealizeCtx, ResetKind};
use crate::core::props::{Props, ValueKind};
use crate::core::space::{AddressSpace, MemAttrs, RamStore, Region, RegionKind, RegionRef};
use crate::core::sync::{LockRank, Mutex};
use crate::core::value::Width;
use crate::core::{Error, Result};
use crate::machine::realize::{BindCtx, Instance};
use crate::machine::validate::{ClassSchema, PropSchema};

use super::{lpc, mch, pm};

/// The class name a machine description writes.
pub const CLASS_NAME: &str = "q35.acpi";

/// The snapshot chunk version. Bump with the encoding, never on its own.
const STATE_VERSION: u32 = 1;

/// The name this device's region carries.
pub const TABLES_REGION: &str = "q35.acpi.tables";

/// How big the region is unless a machine file says otherwise.
pub const DEFAULT_LEN: u64 = 64 * 1024;

/// The region name the local APIC publishes, which is its class name.
///
/// A string rather than `crate::dev::pc::apic::CLASS_NAME` because that module
/// is behind a feature this one does not depend on — a q35 board with no APIC
/// is expressible, and generating a MADT with no local APIC entry is the right
/// answer for it. `the_class_names_have_not_drifted` asserts the two agree
/// whenever both are compiled in.
const LAPIC_REGION: &str = "pc.lapic";

/// The region name the I/O APIC publishes. See [`LAPIC_REGION`].
const IOAPIC_REGION: &str = "pc.ioapic";

/// The region name the HPET publishes. See [`LAPIC_REGION`].
const HPET_REGION: &str = "pc.hpet";

/// Offset of the local APIC's ID register (*Intel SDM* Vol 3A §11.4.6).
const LAPIC_ID_REGISTER: u64 = 0x20;

/// Offset of the HPET's general capabilities register (HPET spec §2.3.4).
const HPET_GCAP_ID: u64 = 0x00;

// ---------------------------------------------------------------------------
// what the machine says about itself
// ---------------------------------------------------------------------------

/// Everything the tables need that the realized machine can be asked for.
///
/// Built by [`survey`]; every field is `Option` because a board may legitimately
/// lack the part, and a table that would have described it is then simply not
/// emitted.
#[derive(Debug, Clone, Default, PartialEq, Eq)]
pub struct MachineFacts {
    /// Where the local APIC's register page is, and the ID it reports.
    pub lapic: Option<(u64, u8)>,
    /// Where the I/O APIC's register page is.
    pub ioapic: Option<u64>,
    /// Where the HPET is, and the low half of its capabilities register.
    pub hpet: Option<(u64, u32)>,
    /// Where the ACPI register block decodes, in the I/O space.
    pub acpi_io: Option<u64>,
    /// Where the ECAM window is and how many bytes it covers.
    pub ecam: Option<(u64, u64)>,
    /// Where this device's own table region is mapped.
    pub tables: Option<u64>,
    /// One past the highest byte of RAM in the memory space.
    ///
    /// The low edge of the window a host bridge hands to the bus below it: a
    /// base address register may go anywhere the processor decodes that is not
    /// already memory, and this is where "already memory" stops.
    pub ram_top: Option<u64>,
    /// Where the configuration port pair decodes in the I/O space, and how
    /// wide it is.
    ///
    /// A hole in the bridge's own I/O window rather than a window: 0xcf8-0xcff
    /// is the bridge's register file, not an address anything downstream may
    /// be given.
    pub config_ports: Option<(u64, u64)>,
    /// What `_PRT` should say: one entry per (device number, pin) that reaches
    /// an interrupt. Empty where there is no fabric to ask, or where its router
    /// routes nothing. [`routing`] builds it.
    pub prt: Vec<PrtRoute>,
}

/// One row of `_PRT`: a slot's interrupt pin and the interrupt it reaches.
///
/// *ACPI Specification* revision 6.5 §6.2.13. A `_PRT` package is four fields —
/// address, pin, source and source index — and this carries the two that vary
/// plus the answer. The source is always the integer zero here, which §6.2.13
/// defines as "the interrupt is allocated from the global interrupt pool" and
/// makes the fourth field the global system interrupt itself.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub struct PrtRoute {
    /// The device number on bus 0. `_PRT`'s address field names every function
    /// of it (§6.2.13: "the low word must be 0xFFFF").
    pub device: u8,
    /// Which pin, 0 for `INTA#` through 3 for `INTD#` — `_PRT`'s own encoding,
    /// which is one less than the Interrupt Pin register's.
    pub pin: u8,
    /// The global system interrupt it arrives on.
    pub gsi: u32,
}

/// Follow aliases down to the region that actually answers.
fn leaf_of(region: &RegionRef) -> RegionRef {
    let mut here = Arc::clone(region);
    // Bounded because `Region::alias` names an already-built region, so the
    // graph is acyclic by construction; the bound is belt and braces.
    for _ in 0..16 {
        let Some(alias) = here.as_alias() else {
            return here;
        };
        here = Arc::clone(alias.target());
    }
    here
}

/// Where the region named `name` is mapped in `space`, and how long it is.
///
/// The lowest base, if a region is mapped more than once — which the shadow
/// aliases are — so the answer does not depend on iteration order.
fn find(space: &AddressSpace, name: &str) -> Option<(u64, u64)> {
    let view = space.view();
    let mut found: Option<(u64, u64)> = None;
    for (_, mapping) in view.mappings() {
        if leaf_of(&mapping.region).name() != name {
            continue;
        }
        let here = (mapping.base, mapping.region.len());
        found = Some(match found {
            Some(best) if best.0 <= here.0 => best,
            _ => here,
        });
    }
    found
}

/// One past the last byte of RAM in `space`, or `None` if there is none.
///
/// By [`RegionKind`] rather than by name, which is the only honest way to ask:
/// a machine file names its memory objects whatever it likes — `ram_low`,
/// `ram_high`, `dram` — and what makes a mapping memory is that the region
/// behind it is a store, not what it is called.
fn ram_top(space: &AddressSpace) -> Option<u64> {
    let view = space.view();
    let mut top: Option<u64> = None;
    for (_, mapping) in view.mappings() {
        if !matches!(leaf_of(&mapping.region).kind(), RegionKind::Ram(_)) {
            continue;
        }
        let end = mapping.base.saturating_add(mapping.region.len());
        top = Some(top.map_or(end, |best: u64| best.max(end)));
    }
    top
}

/// Read a 32-bit register through `space` with `MemAttrs::debug` set.
///
/// The flag is the whole point: a device that would have had a side effect is
/// obliged to suppress it, so this cannot move the machine (`CLAUDE.md`,
/// devices). A refusal is not an error — it means the device declines to be
/// read by a debugger, and the fact simply is not available.
fn peek(space: &AddressSpace, addr: u64) -> Option<u32> {
    let attrs = MemAttrs {
        debug: true,
        ..MemAttrs::default()
    };
    space.read(addr, Width::U32, attrs).ok().map(|v| v as u32)
}

/// The interrupt router on `bus`, found the way software finds it: by class
/// code `060100h`, an ISA bridge (*PCI Local Bus Specification* Rev 2.1
/// Appendix D).
///
/// Not by address. `machines/q35.machine` puts the bridge at 00:1f.0 because
/// that is where an ICH9 lives, but the object takes a `device` property and a
/// board may move it — and a generator that hard-coded the address would then
/// describe a machine that was not built, which is the one thing this module
/// exists not to do.
fn router(bus: &PciBus) -> Option<Bdf> {
    let attrs = MemAttrs {
        debug: true,
        ..MemAttrs::default()
    };
    bus.addresses().into_iter().find(|at| {
        let mut class = [0u8; 3];
        bus.config_read(*at, config::CLASS_CODE, &mut class, attrs);
        class == [0x00, 0x01, config::CLASS_BRIDGE]
    })
}

/// What `_PRT` should say about the functions on `bus`.
///
/// Two pieces of arithmetic and one read of the realized machine:
///
/// 1. [`crate::bus::pci::swizzle`], which turns a device number and one of
///    `INTA#`-`INTD#` into one of the bus's four interrupt nets — the rotation
///    the *PCI-to-PCI Bridge Architecture Specification* Revision 1.1 §9.1
///    defines and `src/bus/pci` implements;
/// 2. the router's **`PIRQ[n]_ROUT`** byte for that net (ICH9 §13.1.17), read
///    out of the bridge that was actually built, which says which ISA interrupt
///    the net comes out on.
///
/// The read carries `MemAttrs::debug`, and the register is one a debugger may
/// read: it is a plain latch with no side effect and it is not behind an index,
/// which is the distinction [`TableConfig::ioapic_id`] exists for.
///
/// # Every device number, not every device
///
/// The rows cover the whole bus — device 0 to [`MAX_DEVICE`], four pins each —
/// rather than only the functions that happen to answer at reset. `_PRT`
/// describes **wiring, not inventory**: a function that appears later reaches
/// the same net by the same rotation, and an operating system that read the
/// table once would have nothing to look up for it. Which is also why nothing
/// here consults a function's Interrupt Pin register — the routing exists
/// whether or not anything is plugged into it.
///
/// A net whose router routes nowhere — §13.1.17's power-up `80h`, or a reserved
/// encoding — contributes **no row**. A `_PRT` that claimed an interrupt
/// arrives somewhere it does not is worse than one that is silent about that
/// pin, and silence reads as "this pin has no routing" rather than as a wrong
/// one. A board with no router at all gets no rows, and therefore no `_PRT`.
#[must_use]
pub fn routing(bus: &PciBus) -> Vec<PrtRoute> {
    let attrs = MemAttrs {
        debug: true,
        ..MemAttrs::default()
    };
    let Some(router) = router(bus) else {
        return Vec::new();
    };
    // The four nets, resolved once: every device number reads the same four
    // routing registers, only rotated.
    let mut gsi = [None; INTX_LINES as usize];
    for (net, slot) in gsi.iter_mut().enumerate() {
        let mut route = [0u8; 1];
        bus.config_read(router, lpc::pirq_rout(net), &mut route, attrs);
        *slot = lpc::pirq_destination(route[0]).map(u32::from);
    }
    let mut out = Vec::new();
    for device in 0..=MAX_DEVICE {
        let at = Bdf {
            bus: 0,
            device,
            function: 0,
        };
        for pin in [IntxPin::A, IntxPin::B, IntxPin::C, IntxPin::D] {
            let (Some(index), Some(net)) = (pin.index(), swizzle(at, pin)) else {
                continue;
            };
            let Some(gsi) = gsi[net as usize] else {
                continue;
            };
            out.push(PrtRoute {
                device,
                pin: index,
                gsi,
            });
        }
    }
    out
}

/// Ask the realized machine about itself.
///
/// `mem` is the space the chipset decodes, `io` the one `PMBASE` places the
/// ACPI block in, and `bus` the fabric whose functions `_PRT` describes — a
/// board with no PCI fabric passes `None` and gets no `_PRT`.
#[must_use]
pub fn survey(mem: &AddressSpace, io: &AddressSpace, bus: Option<&PciBus>) -> MachineFacts {
    let lapic = find(mem, LAPIC_REGION).map(|(base, _)| {
        // The ID register holds it in bits 31:24 (SDM Vol 3A §11.4.6). A part
        // that declines a debug read leaves the ID at zero, which is the
        // bootstrap processor's own default and the least surprising answer.
        let id = peek(mem, base + LAPIC_ID_REGISTER).unwrap_or(0);
        (base, (id >> 24) as u8)
    });
    let hpet = find(mem, HPET_REGION).map(|(base, _)| {
        let caps = peek(mem, base + HPET_GCAP_ID).unwrap_or(0);
        (base, caps)
    });
    MachineFacts {
        lapic,
        ioapic: find(mem, IOAPIC_REGION).map(|(base, _)| base),
        hpet,
        acpi_io: find(io, lpc::ACPI_REGION).map(|(base, _)| base),
        ecam: find(mem, mch::ECAM_REGION),
        tables: find(mem, TABLES_REGION).map(|(base, _)| base),
        ram_top: ram_top(mem),
        config_ports: find(io, mch::CONFIG_REGION),
        prt: bus.map(routing).unwrap_or_default(),
    }
}

// ---------------------------------------------------------------------------
// what the board declares
// ---------------------------------------------------------------------------

/// The facts the address space cannot supply. See the module docs.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct TableConfig {
    /// The six-character OEM identification every table carries (§5.2.6).
    pub oem_id: [u8; 6],
    /// The eight-character OEM table identification (§5.2.6).
    pub oem_table_id: [u8; 8],
    /// The OEM revision (§5.2.6).
    pub oem_revision: u32,
    /// How many processors the MADT should describe.
    ///
    /// Not derivable: a processor is not a region. The first one's APIC ID is
    /// read from its APIC and the rest are numbered from it.
    pub cpus: u8,
    /// The I/O APIC's ID.
    ///
    /// Not derivable: it lives behind an index/data register pair, and reaching
    /// it means *writing* the index, which a debug read may not do.
    pub ioapic_id: u8,
    /// The global system interrupt the I/O APIC's input 0 is (§5.2.12.3).
    pub gsi_base: u32,
    /// Which interrupt the SCI appears on — `ACPI_CNTL[2:0]`'s choice, made by
    /// firmware after this table is written.
    pub sci_irq: u16,
}

impl Default for TableConfig {
    fn default() -> TableConfig {
        TableConfig {
            oem_id: *b"RSEMU ",
            oem_table_id: *b"RSEMUQ35",
            oem_revision: 1,
            cpus: 1,
            ioapic_id: 1,
            gsi_base: 0,
            // `000b`, which is what `ACPI_CNTL` comes out of reset selecting
            // (ICH9 §13.1.14) and what `machines/q35.machine` wires.
            sci_irq: 9,
        }
    }
}

// ---------------------------------------------------------------------------
// the bytes
// ---------------------------------------------------------------------------

/// How long a description header is (§5.2.6).
pub const HEADER_LEN: usize = 36;

/// The 8-bit sum every ACPI table has to bring to zero (§5.2.6).
#[must_use]
pub fn checksum(bytes: &[u8]) -> u8 {
    bytes.iter().fold(0u8, |sum, b| sum.wrapping_add(*b))
}

/// A table being built: a description header and a body.
struct Table {
    bytes: Vec<u8>,
}

impl Table {
    /// Start a table with `signature` and `revision`, leaving `Length` and
    /// `Checksum` to [`finish`](Table::finish).
    fn new(signature: &[u8; 4], revision: u8, cfg: &TableConfig) -> Table {
        let mut bytes = Vec::with_capacity(HEADER_LEN);
        bytes.extend_from_slice(signature);
        bytes.extend_from_slice(&0u32.to_le_bytes());
        bytes.push(revision);
        bytes.push(0);
        bytes.extend_from_slice(&cfg.oem_id);
        bytes.extend_from_slice(&cfg.oem_table_id);
        bytes.extend_from_slice(&cfg.oem_revision.to_le_bytes());
        // Creator ID and revision: who built the table, which here is rsemu.
        bytes.extend_from_slice(b"RSMU");
        bytes.extend_from_slice(&1u32.to_le_bytes());
        debug_assert_eq!(bytes.len(), HEADER_LEN);
        Table { bytes }
    }

    fn u8(&mut self, v: u8) -> &mut Table {
        self.bytes.push(v);
        self
    }

    fn u16(&mut self, v: u16) -> &mut Table {
        self.bytes.extend_from_slice(&v.to_le_bytes());
        self
    }

    fn u32(&mut self, v: u32) -> &mut Table {
        self.bytes.extend_from_slice(&v.to_le_bytes());
        self
    }

    fn u64(&mut self, v: u64) -> &mut Table {
        self.bytes.extend_from_slice(&v.to_le_bytes());
        self
    }

    fn zeros(&mut self, n: usize) -> &mut Table {
        self.bytes.resize(self.bytes.len() + n, 0);
        self
    }

    /// Pad out to `offset` bytes from the start of the table.
    ///
    /// How the FADT is written: every field goes at the offset Table 5.9 gives
    /// it, said out loud at the point it is written, so a missing field is a
    /// compile-time-visible gap rather than a silent shift of everything after
    /// it.
    fn at(&mut self, offset: usize) -> &mut Table {
        debug_assert!(self.bytes.len() <= offset, "a field overran its offset");
        self.bytes.resize(offset, 0);
        self
    }

    fn bytes(&mut self, b: &[u8]) -> &mut Table {
        self.bytes.extend_from_slice(b);
        self
    }

    /// Fill in `Length` and `Checksum` and hand over the table.
    fn finish(mut self) -> Vec<u8> {
        let len = self.bytes.len() as u32;
        self.bytes[4..8].copy_from_slice(&len.to_le_bytes());
        self.bytes[9] = 0;
        self.bytes[9] = checksum(&self.bytes).wrapping_neg();
        debug_assert_eq!(checksum(&self.bytes), 0);
        self.bytes
    }
}

/// A Generic Address Structure (§5.2.3.2, Table 5.1).
#[must_use]
pub fn gas(space_id: u8, bit_width: u8, bit_offset: u8, access_size: u8, address: u64) -> [u8; 12] {
    let mut out = [0u8; 12];
    out[0] = space_id;
    out[1] = bit_width;
    out[2] = bit_offset;
    out[3] = access_size;
    out[4..].copy_from_slice(&address.to_le_bytes());
    out
}

/// `Address Space ID` 0: system memory (§5.2.3.2).
pub const GAS_MEMORY: u8 = 0x00;
/// `Address Space ID` 1: system I/O.
pub const GAS_IO: u8 = 0x01;

/// The FADT revision this module emits, and the length that goes with it.
///
/// Revision 3 is ACPI 2.0's, and its last field is `X_GPE1_BLK` ending at 244.
/// It is chosen over 5 or 6 because everything above it is sleep control this
/// board does not implement, and a table that declared `SLEEP_CONTROL_REG` and
/// then ignored writes to it would be lying in a way revision 3 is not.
pub const FADT_REVISION: u8 = 3;
/// The FADT length that goes with [`FADT_REVISION`].
pub const FADT_LEN: usize = 244;

/// `Flags[0]`: `WBINVD` behaves (Table 5.10).
const FADT_WBINVD: u32 = 1 << 0;
/// `Flags[2]`: `PROC_C1`, C1 is supported on every processor.
const FADT_PROC_C1: u32 = 1 << 2;
/// `Flags[4]`: the power button is a control method device — and there is not
/// one, which Table 5.10 says is exactly how "no power button" is spelled.
const FADT_PWR_BUTTON: u32 = 1 << 4;
/// `Flags[5]`: likewise the sleep button.
const FADT_SLP_BUTTON: u32 = 1 << 5;
/// `Flags[10]`: `RESET_REG_SUP`, the reset register works.
const FADT_RESET_REG_SUP: u32 = 1 << 10;

/// `IAPC_BOOT_ARCH[0]`: there are user-visible devices on the LPC bus
/// (Table 5.11).
const IAPC_LEGACY_DEVICES: u16 = 1 << 0;
/// `IAPC_BOOT_ARCH[1]`: a port 60/64 keyboard controller is present.
const IAPC_8042: u16 = 1 << 1;

/// The port the chipset's reset control register decodes at, and the value that
/// pulls the line.
///
/// The one address in this file that is written down rather than derived, and
/// it is not a chipset register at all: it is the *board's* — `machines/`
/// hands `0xcf9` to `pc.sysctl` through the host bridge's pass-through, and a
/// firmware writes `0x06` there to reboot. Deriving it would mean asking the
/// I/O space where a region named `pc.sysctl` is mapped, and it is mapped at
/// four addresses of which this is not even one (it arrives through `0xcf8`'s
/// window). Named constants beat a fiction of derivation.
const RESET_PORT: u64 = 0xcf9;
/// What a firmware writes to [`RESET_PORT`] to reboot: `SYS_RST` plus
/// `RST_CPU`.
const RESET_VALUE: u8 = 0x06;

/// The CMOS index the century lives at.
///
/// `crate::dev::pc::rtc`'s `REG_CENTURY`, which is the AT BIOS's own choice and
/// is what the FADT's `CENTURY` field is for (§5.2.9).
const CMOS_CENTURY: u8 = 0x32;

/// The `_S5` sleep type this chipset uses: `111b`, *Soft Off* (ICH9 §13.8.3.3).
const SLP_TYP_S5: u64 = 0b111;

/// Build the DSDT: an `_S5` package and a PCI host bridge.
///
/// **What is deliberately not in it**, because each would be a claim this board
/// cannot back:
///
/// * `Method`, `If`, `Return` — [`super::aml`] cannot encode them, on purpose.
///
/// # `_CRS`, and why a board without one has no disk
///
/// The host bridge's `_CRS` is the list of *windows it produces* — the bus
/// numbers, the I/O ports and the physical addresses that belong to the bus
/// below it rather than to the processor. An operating system allocates every
/// base address register out of those windows, so a bridge that declares none
/// has declared that nothing downstream may be given an address:
///
/// ```text
///     pci 0000:00:04.0: BAR 0: no space for [mem size 0x00002000 64bit]
///     pci 0000:00:04.0: BAR 0: failed to assign [mem size 0x00002000 64bit]
/// ```
///
/// which is a Linux 6.6 kernel on `machines/q35-linux.machine` before this
/// existed, and is the whole distance between "the controller is enumerated"
/// and "the controller has a driver".
///
/// Every edge of it is read out of the realized machine, which is this file's
/// rule and here it is also the only way to be right:
///
/// | Window | Where its edges come from |
/// | --- | --- |
/// | bus numbers | 0 to 255: everything is on bus 0 and there is no Type 1 header to divide the range with (`docs/platforms/q35.md`) |
/// | I/O | 0 to 0xffff, with the hole [`MachineFacts::config_ports`] found the 0xcf8 pair at |
/// | memory | [`MachineFacts::ram_top`] up to the lowest of the APIC and HPET pages, with [`MachineFacts::ecam`]'s window cut out |
///
/// The ECAM window is cut out and then declared again on a separate `PNP0C02`
/// motherboard device, which is what a firmware does with it and what makes an
/// operating system willing to *use* the window: Linux refuses a memory-mapped
/// configuration space that is reserved neither in e820 nor by a motherboard
/// device's `_CRS`, and says so —
/// `PCI: MMCONFIG at [mem ...] not reserved in ACPI motherboard resources`.
///
/// # `_PRT`, and the one form of it this board can honestly emit
///
/// `_PRT` *is* here, from [`MachineFacts::prt`], and it is the **fixed** form:
/// each package's source is the integer zero and its source index is a global
/// system interrupt (ACPI §6.2.13). The alternative form names a PCI interrupt
/// link device, and a link device is only meaningful with `_PRS`, `_CRS`,
/// `_SRS` and `_DIS` — four methods that read and write the router's own
/// configuration register, and [`super::aml`] deliberately cannot encode a
/// method. Emitting link devices without them would be a `_PRT` that an
/// operating system follows into a device it cannot then program, which is
/// worse than the fixed form.
///
/// The fixed form is not a lie on *this* board, and the reason is a property of
/// its wiring rather than a convenience: `machines/q35.machine` takes every one
/// of the router's eleven outputs to the 8259A input it names **and** to the
/// I/O APIC input of the same number, so the global system interrupt and the
/// ISA interrupt are the same number and one table is true in both modes. What
/// it does not survive is a guest *reprogramming* `PIRQ[n]_ROUT` after the
/// tables were generated — the table then describes where the interrupt used to
/// go. That is exactly why the routing this reads comes from the realized
/// bridge rather than from a constant: a board states its power-up routing with
/// `q35.lpc`'s `pirq-routes` (a stand-in for the POST §13.1.17 asks for), the
/// tables are generated from it, and the two cannot disagree. A firmware that
/// wants to move it publishes its own tables, which is what a firmware is for.
#[must_use]
pub fn dsdt(facts: &MachineFacts, cfg: &TableConfig) -> Vec<u8> {
    use super::aml;
    let mut body = Vec::new();
    // `\_S0`: the working state. `000b` on this chipset.
    let mut s0 = aml::integer(0);
    s0.extend_from_slice(&aml::integer(0));
    body.extend_from_slice(&aml::name("\\_S0_", &aml::package(2, &s0)));
    // `\_S5`: soft off, which is what an ACPI shutdown writes into `SLP_TYP`.
    let mut s5 = aml::integer(SLP_TYP_S5);
    s5.extend_from_slice(&aml::integer(0));
    body.extend_from_slice(&aml::name("\\_S5_", &aml::package(2, &s5)));

    let mut pci0 = Vec::new();
    // `PNP0A08` is a PCI Express host bridge and `PNP0A03` a conventional one;
    // ACPI §6.1.2 has the `_CID` carry the older identifier so an operating
    // system that knows only the latter still binds a driver.
    pci0.extend_from_slice(
        &aml::name_eisa_id("_HID", "PNP0A08").expect("a well-formed EISA identifier"),
    );
    pci0.extend_from_slice(
        &aml::name_eisa_id("_CID", "PNP0A03").expect("a well-formed EISA identifier"),
    );
    pci0.extend_from_slice(&aml::name("_ADR", &aml::integer(0)));
    pci0.extend_from_slice(&aml::name("_UID", &aml::integer(0)));
    // `_BBN`: the bus number this bridge is the root of. Zero, because
    // everything on this board is on bus 0 — see the module docs on root ports.
    pci0.extend_from_slice(&aml::name("_BBN", &aml::integer(0)));
    // `_CRS`: the windows this bridge produces for the bus below it.
    pci0.extend_from_slice(&aml::name("_CRS", &host_bridge_crs(facts)));
    // `_PRT` (§6.2.13). Absent rather than empty when nothing on the bus
    // interrupts: an empty package is a claim that no slot has an interrupt,
    // and a board whose router simply has not been programmed has not made
    // that claim.
    if !facts.prt.is_empty() {
        let mut rows = Vec::new();
        for route in &facts.prt {
            let mut row = Vec::new();
            // §6.2.13: "the low word must be 0xFFFF", which names every
            // function of the device rather than function zero.
            row.extend_from_slice(&aml::integer((u64::from(route.device) << 16) | 0xffff));
            row.extend_from_slice(&aml::integer(u64::from(route.pin)));
            // Source zero: allocated from the global interrupt pool, so the
            // next field is the interrupt itself rather than an index into a
            // link device's possible settings.
            row.extend_from_slice(&aml::integer(0));
            row.extend_from_slice(&aml::integer(u64::from(route.gsi)));
            rows.extend_from_slice(&aml::package(4, &row));
        }
        let count = u8::try_from(facts.prt.len()).unwrap_or(u8::MAX);
        pci0.extend_from_slice(&aml::name("_PRT", &aml::package(count, &rows)));
    }
    let mut devices = aml::device("PCI0", &pci0);
    // The ECAM window, as a motherboard resource. `PNP0C02` is ACPI §9.15's
    // "PNP Motherboard Registers": a device that exists only to say that an
    // address is spoken for.
    if let Some((base, len)) = facts.ecam
        && let Ok(min) = u32::try_from(base)
        && let Ok(max) = u32::try_from(base + len - 1)
    {
        let mut res = Vec::new();
        res.extend_from_slice(
            &aml::name_eisa_id("_HID", "PNP0C02").expect("a well-formed EISA identifier"),
        );
        res.extend_from_slice(&aml::name("_UID", &aml::integer(1)));
        res.extend_from_slice(&aml::name(
            "_CRS",
            &aml::resource_template(&aml::dword_memory(min, max, false)),
        ));
        devices.extend_from_slice(&aml::device("PCIE", &res));
    }
    body.extend_from_slice(&aml::scope("\\_SB_", &devices));

    let mut table = Table::new(b"DSDT", 2, cfg);
    table.bytes(&body);
    table.finish()
}

/// The host bridge's `_CRS`, as a resource template.
///
/// See [`dsdt`]'s own documentation for where each edge comes from and why the
/// table is useless without this.
fn host_bridge_crs(facts: &MachineFacts) -> Vec<u8> {
    use super::aml;

    /// What the memory window is rounded to. A megabyte, which is the
    /// granularity every PC firmware has placed a PCI hole on, and coarse
    /// enough that a base address register with a large alignment still fits
    /// at the bottom of the window.
    const WINDOW_ALIGN: u64 = 1 << 20;
    /// The ceiling when nothing is mapped above the window. Four gigabytes:
    /// this emits 32-bit descriptors, so a window that reached higher could not
    /// be described by one.
    const FOUR_GIB: u64 = 1 << 32;

    let mut out = Vec::new();
    // Bus numbers. The whole range, because there is no Type 1 header in
    // `src/bus/pci` and therefore no second bus to divide it with.
    out.extend_from_slice(&aml::bus_number_range(0, 0xff));

    // The I/O window, with the configuration port pair cut out of it.
    let ports = facts.config_ports.and_then(|(base, len)| {
        let end = base + len;
        (base > 0 && end <= 0x1_0000).then_some((base as u32, end as u32))
    });
    match ports {
        Some((base, end)) => {
            out.extend_from_slice(&aml::dword_io(0, base - 1));
            if end <= 0xffff {
                out.extend_from_slice(&aml::dword_io(end, 0xffff));
            }
        }
        None => out.extend_from_slice(&aml::dword_io(0, 0xffff)),
    }

    // The memory window: from the top of RAM to the lowest thing the processor
    // already decodes above it, which on every board this runs on is one of the
    // three APIC or HPET pages.
    let start = facts
        .ram_top
        .unwrap_or(WINDOW_ALIGN)
        .div_ceil(WINDOW_ALIGN)
        .saturating_mul(WINDOW_ALIGN);
    let ceiling = [
        facts.ioapic,
        facts.hpet.map(|(base, _)| base),
        facts.lapic.map(|(base, _)| base),
    ]
    .into_iter()
    .flatten()
    .filter(|base| *base > start)
    .min()
    .unwrap_or(FOUR_GIB)
    .min(FOUR_GIB);

    // ...with the ECAM window cut out of it, because that address is decoded
    // already and a base address register placed on top of it would be two
    // things answering one address.
    let hole = facts
        .ecam
        .map(|(base, len)| (base, base.saturating_add(len)))
        .filter(|(base, end)| *end > start && *base < ceiling);
    let mut window = |from: u64, to: u64| {
        if to <= from {
            return;
        }
        let (Ok(min), Ok(max)) = (u32::try_from(from), u32::try_from(to - 1)) else {
            return;
        };
        out.extend_from_slice(&aml::dword_memory(min, max, true));
    };
    match hole {
        Some((base, end)) => {
            window(start, base.max(start));
            window(end.max(start), ceiling);
        }
        None => window(start, ceiling),
    }
    aml::resource_template(&out)
}

/// Build the FACS (§5.2.10, Table 5.13).
///
/// Sixty-four bytes of almost nothing: the FADT has to point at one, and a
/// `FIRMWARE_CTRL` of zero on a table that is not hardware-reduced is a
/// missing structure rather than an absent feature.
#[must_use]
pub fn facs() -> Vec<u8> {
    let mut out = Vec::with_capacity(64);
    out.extend_from_slice(b"FACS");
    out.extend_from_slice(&64u32.to_le_bytes());
    // Hardware Signature: must change if the table set changes. Zero, and it is
    // honest — nothing here resumes from S4, so nothing ever compares it.
    out.extend_from_slice(&0u32.to_le_bytes());
    // Firmware Waking Vector, Global Lock, Flags: all zero.
    out.resize(24, 0);
    // X Firmware Waking Vector.
    out.extend_from_slice(&0u64.to_le_bytes());
    // Version 2, which is what ACPI 5.1 Errata A defines and covers every field
    // written here. There is no checksum: the FACS is not a description header.
    out.push(2);
    out.resize(64, 0);
    out
}

/// Build the MADT (§5.2.12).
#[must_use]
pub fn madt(facts: &MachineFacts, cfg: &TableConfig) -> Option<Vec<u8>> {
    let (lapic_base, bsp_id) = facts.lapic?;
    let mut table = Table::new(b"APIC", 6, cfg);
    // Table 5.19: the address every processor reaches its own local APIC at.
    table.u32(lapic_base as u32);
    // Table 5.20: `PCAT_COMPAT`. True — this board has the 8259A pair, and an
    // operating system switching to the APICs has to mask them.
    table.u32(1);
    for index in 0..cfg.cpus {
        // Table 5.22: type 0, length 8.
        table.u8(0).u8(8).u8(index).u8(bsp_id.wrapping_add(index));
        // Table 5.23: bit 0, Enabled.
        table.u32(1);
    }
    if let Some(ioapic) = facts.ioapic {
        // Table 5.24: type 1, length 12.
        table.u8(1).u8(12).u8(cfg.ioapic_id).u8(0);
        table.u32(ioapic as u32).u32(cfg.gsi_base);
        // Table 5.25: ISA IRQ0 reaches I/O APIC input 2 rather than input 0,
        // which the board's own wiring onto `ioapic.irq2` is the other half of.
        // Which *chip* drives it is not this entry's business and deliberately
        // so: on `machines/q35-linux.machine` the HPET's `LEG_RT_CNF` swaps
        // comparator 0 in for the 8254 there, and the offset is the same either
        // way. Without this entry an operating system loses its timer tick the
        // moment it stops using the 8259A.
        table.u8(2).u8(10).u8(0).u8(0);
        table.u32(cfg.gsi_base + 2);
        // Table 5.26: `00b` polarity and `00b` trigger mode, conforming to the
        // ISA bus, which is edge-triggered and active high.
        table.u16(0);
        // And the SCI. It is identity-mapped, but its flags are not the ISA
        // default: ICH9 §13.1.14 says "When the interrupt is mapped to APIC
        // interrupts 9, 10 or 11, the APIC should be programmed for active-high
        // reception", and an SCI is level-triggered by definition (§5.2.9's
        // SCI_INT: "treated as a shareable, level, active low interrupt").
        table.u8(2).u8(10).u8(0).u8(cfg.sci_irq as u8);
        table.u32(cfg.gsi_base + u32::from(cfg.sci_irq));
        // Polarity `01b` active high, trigger `11b` level.
        table.u16(0b1101);
    }
    // Table 5.28: type 4, length 6. Every processor's `LINT1` is the NMI, which
    // is how a PC has always been wired.
    table.u8(4).u8(6).u8(0xff).u16(0).u8(1);
    Some(table.finish())
}

/// Build the MCFG. See the module docs on this one's weaker citation.
#[must_use]
pub fn mcfg(facts: &MachineFacts, cfg: &TableConfig) -> Option<Vec<u8>> {
    let (base, len) = facts.ecam?;
    let buses = len / super::ecam::BUS_STRIDE;
    if buses == 0 {
        return None;
    }
    let mut table = Table::new(b"MCFG", 1, cfg);
    // Eight reserved bytes between the header and the first allocation.
    table.zeros(8);
    table.u64(base);
    // Segment group 0, and the buses this window covers.
    table.u16(0).u8(0).u8((buses - 1) as u8);
    table.u32(0);
    Some(table.finish())
}

/// Build the HPET description table (*IA-PC HPET Specification* §3.2.4).
#[must_use]
pub fn hpet(facts: &MachineFacts, cfg: &TableConfig) -> Option<Vec<u8>> {
    let (base, caps) = facts.hpet?;
    let mut table = Table::new(b"HPET", 1, cfg);
    // Event Timer Block ID: "contents of the block's General_Cap&ID register",
    // read back out of the part rather than restated.
    table.u32(caps);
    table.bytes(&gas(GAS_MEMORY, 0, 0, 0, base));
    // HPET Number 0: the first such block on the board.
    table.u8(0);
    // Main Counter Minimum Clock_tick in Periodic Mode. Zero, and it is a real
    // gap: the number is how few ticks a comparator may be set to without
    // losing an interrupt, and `pc.hpet` completes its comparisons without
    // modelling their duration, so it has no lower bound to report.
    table.u16(0);
    // Page Protection: `0`, no guarantee — the block is one mapping in an
    // address space, and nothing here protects a page around it.
    table.u8(0);
    Some(table.finish())
}

/// Build the FADT (§5.2.9, Table 5.9).
///
/// Every field is written at the offset the table gives it, with
/// `Table::at` asserting that the one before it did not overrun.
#[must_use]
pub fn fadt(facts: &MachineFacts, cfg: &TableConfig, facs_at: u64, dsdt_at: u64) -> Vec<u8> {
    let io = facts.acpi_io;
    let block = |offset: u64| io.map_or(0, |base| base + offset);
    let mut t = Table::new(b"FACP", FADT_REVISION, cfg);
    t.at(36).u32(facs_at as u32);
    t.at(40).u32(dsdt_at as u32);
    // 44 is reserved (it was ACPI 1.0's INT_MODEL).
    // 45: Preferred_PM_Profile. 1 is Desktop, which is what a q35 is.
    t.at(45).u8(1);
    t.at(46).u16(cfg.sci_irq);
    // 48-53: SMI_CMD, ACPI_ENABLE, ACPI_DISABLE. All zero, and that is a
    // statement rather than a gap: this board has no SMI path, so there is no
    // port an operating system could write to ask firmware for ownership, and
    // §5.2.9 makes a zero `SMI_CMD` mean exactly "the system does not support
    // the legacy-to-ACPI transition".
    t.at(56).u32(block(pm::PM1_STS) as u32);
    t.at(64).u32(block(pm::PM1_CNT) as u32);
    t.at(76).u32(block(pm::PM1_TMR) as u32);
    t.at(80).u32(block(pm::GPE0_STS) as u32);
    t.at(88).u8(pm::PM1_EVT_LEN);
    t.at(89).u8(pm::PM1_CNT_LEN);
    t.at(91).u8(pm::PM_TMR_LEN);
    t.at(92).u8(pm::GPE0_BLK_LEN);
    // 96, 98: P_LVL2_LAT and P_LVL3_LAT. Table 5.9: a value over 100 means C2
    // is not supported and one over 1000 means C3 is not — which is true, since
    // nothing here implements a C state.
    t.at(96).u16(0x0fff);
    t.at(98).u16(0x0fff);
    t.at(108).u8(CMOS_CENTURY);
    t.at(109).u16(IAPC_LEGACY_DEVICES | IAPC_8042);
    t.at(112)
        .u32(FADT_WBINVD | FADT_PROC_C1 | FADT_PWR_BUTTON | FADT_SLP_BUTTON | FADT_RESET_REG_SUP);
    // 116: RESET_REG. §5.2.9 requires bit width 8 and bit offset 0.
    t.at(116).bytes(&gas(GAS_IO, 8, 0, 1, RESET_PORT));
    t.at(128).u8(RESET_VALUE);
    t.at(132).u64(facs_at);
    t.at(140).u64(dsdt_at);
    // The 64-bit forms of the blocks. §5.2.9: where both exist the 64-bit one
    // wins whenever it is usable, so these have to agree with the 32-bit ones
    // above, and they do because both come from `block`.
    t.at(148).bytes(&gas(
        GAS_IO,
        u32::from(pm::PM1_EVT_LEN) as u8 * 8,
        0,
        2,
        block(pm::PM1_STS),
    ));
    t.at(172).bytes(&gas(
        GAS_IO,
        u32::from(pm::PM1_CNT_LEN) as u8 * 8,
        0,
        2,
        block(pm::PM1_CNT),
    ));
    t.at(208).bytes(&gas(
        GAS_IO,
        u32::from(pm::PM_TMR_LEN) as u8 * 8,
        0,
        3,
        block(pm::PM1_TMR),
    ));
    // §5.2.9: for the GPE blocks "OSPM ignores bit width, bit offset and access
    // size", so they are zero rather than invented.
    t.at(220).bytes(&gas(GAS_IO, 0, 0, 0, block(pm::GPE0_STS)));
    t.at(FADT_LEN);
    t.finish()
}

/// Build the RSDP (§5.2.5.3, Table 5.3).
#[must_use]
pub fn rsdp(cfg: &TableConfig, rsdt_at: u64, xsdt_at: u64) -> Vec<u8> {
    let mut out = Vec::with_capacity(36);
    // "RSD PTR " — the trailing blank is part of the signature.
    out.extend_from_slice(b"RSD PTR ");
    out.push(0);
    out.extend_from_slice(&cfg.oem_id);
    // Revision 2, which is what makes the XSDT half of this structure valid.
    out.push(2);
    out.extend_from_slice(&(rsdt_at as u32).to_le_bytes());
    out.extend_from_slice(&36u32.to_le_bytes());
    out.extend_from_slice(&xsdt_at.to_le_bytes());
    out.push(0);
    out.resize(36, 0);
    // The first checksum covers "only the first 20 bytes of this table, bytes 0
    // to 19, including the checksum field".
    out[8] = checksum(&out[..20]).wrapping_neg();
    // The second covers "the entire table, including both checksum fields".
    out[32] = checksum(&out).wrapping_neg();
    debug_assert_eq!(checksum(&out[..20]), 0);
    debug_assert_eq!(checksum(&out), 0);
    out
}

/// Everything the generator produced, and where it goes.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Tables {
    /// The bytes, laid out to be placed at [`base`](Tables::base).
    pub bytes: Vec<u8>,
    /// The guest-physical address the bytes have been laid out for.
    pub base: u64,
    /// Where the RSDP ended up — the start, always, which is what makes the
    /// 16-byte-boundary search in §5.2.5.1 find it.
    pub rsdp: u64,
}

/// How the tables are aligned against each other.
///
/// Sixteen for everything, because §5.2.5.1's search is on 16-byte boundaries
/// and it costs nothing to keep the rest of the set tidy; sixty-four for the
/// FACS, which §5.2.10 requires to be "aligned on a 64-byte boundary".
const ALIGN: u64 = 16;
/// The FACS's own alignment (§5.2.10).
const FACS_ALIGN: u64 = 64;

/// Round `at` up to a multiple of `align`.
fn align_up(at: u64, align: u64) -> u64 {
    at.div_ceil(align) * align
}

/// Generate the whole table set, laid out to be placed at `base`.
///
/// This is the seam the module docs promise: a firmware that wanted to allocate
/// the tables itself and report them in its own memory map calls exactly this,
/// with the same [`MachineFacts`] a [`survey`] produced.
///
/// # Errors
///
/// [`Error::Config`] if the machine has no local APIC — a MADT with no
/// processor in it describes a machine an operating system cannot start on, and
/// silently emitting one would turn a board wiring mistake into a mysterious
/// hang.
pub fn generate(base: u64, facts: &MachineFacts, cfg: &TableConfig) -> Result<Tables> {
    if facts.lapic.is_none() {
        return Err(Error::Config {
            at: CLASS_NAME.to_string(),
            message: String::from(
                "no local APIC is mapped in this machine's memory space, so the MADT would \
                 describe no processor: add a `pc.lapic` and map its `regs` region",
            ),
        });
    }
    let dsdt_bytes = dsdt(facts, cfg);
    let facs_bytes = facs();
    let madt_bytes = madt(facts, cfg);
    let mcfg_bytes = mcfg(facts, cfg);
    let hpet_bytes = hpet(facts, cfg);

    // How many tables the RSDT and XSDT will list, so their own sizes are known
    // before any address is assigned.
    let listed = 1
        + usize::from(madt_bytes.is_some())
        + usize::from(mcfg_bytes.is_some())
        + usize::from(hpet_bytes.is_some());

    // Lay everything out. The RSDP is first, because it is what the guest
    // searches for and the search is over a range this whole blob sits inside.
    let mut at = base + align_up(36, ALIGN);
    let facs_at = align_up(at, FACS_ALIGN);
    at = facs_at + facs_bytes.len() as u64;
    let dsdt_at = align_up(at, ALIGN);
    at = dsdt_at + dsdt_bytes.len() as u64;

    let mut others: Vec<(u64, Vec<u8>)> = Vec::with_capacity(listed);
    for table in [madt_bytes, mcfg_bytes, hpet_bytes].into_iter().flatten() {
        let here = align_up(at, ALIGN);
        at = here + table.len() as u64;
        others.push((here, table));
    }
    let fadt_at = align_up(at, ALIGN);
    let fadt_bytes = fadt(facts, cfg, facs_at, dsdt_at);
    at = fadt_at + fadt_bytes.len() as u64;

    let xsdt_at = align_up(at, ALIGN);
    at = xsdt_at + (HEADER_LEN + listed * 8) as u64;
    let rsdt_at = align_up(at, ALIGN);
    at = rsdt_at + (HEADER_LEN + listed * 4) as u64;

    // The XSDT and the RSDT list the same tables; the DSDT and the FACS are
    // *not* among them, because the FADT points at those two directly (§5.2.8).
    let mut xsdt = Table::new(b"XSDT", 1, cfg);
    let mut rsdt = Table::new(b"RSDT", 1, cfg);
    xsdt.u64(fadt_at);
    rsdt.u32(fadt_at as u32);
    for (address, _) in &others {
        xsdt.u64(*address);
        rsdt.u32(*address as u32);
    }
    let xsdt_bytes = xsdt.finish();
    let rsdt_bytes = rsdt.finish();
    debug_assert_eq!(xsdt_bytes.len(), HEADER_LEN + listed * 8);
    debug_assert_eq!(rsdt_bytes.len(), HEADER_LEN + listed * 4);

    let mut bytes = alloc::vec![0u8; (at - base) as usize];
    let mut put = |address: u64, data: &[u8]| {
        let start = (address - base) as usize;
        bytes[start..start + data.len()].copy_from_slice(data);
    };
    put(base, &rsdp(cfg, rsdt_at, xsdt_at));
    put(facs_at, &facs_bytes);
    put(dsdt_at, &dsdt_bytes);
    for (address, table) in &others {
        put(*address, table);
    }
    put(fadt_at, &fadt_bytes);
    put(xsdt_at, &xsdt_bytes);
    put(rsdt_at, &rsdt_bytes);

    Ok(Tables {
        bytes,
        base,
        rsdp: base,
    })
}

// ---------------------------------------------------------------------------
// the device
// ---------------------------------------------------------------------------

/// The ACPI tables, as a device that stages them into guest memory.
#[derive(Debug)]
pub struct AcpiTables {
    store: Arc<RamStore>,
    region: RegionRef,
    len: u64,
    cfg: TableConfig,
    iospace: String,
    /// The fabric `_PRT` describes, if the board has one.
    bus: Option<Arc<PciBus>>,
    /// The two spaces the survey walks. `None` until [`Instance::bind`].
    /// [`LockRank::LEAF`].
    spaces: Mutex<Option<(Arc<AddressSpace>, Arc<AddressSpace>)>>,
    /// What the last generation found, kept so a test can ask what the machine
    /// said about itself. [`LockRank::LEAF`].
    facts: Mutex<MachineFacts>,
}

impl AcpiTables {
    /// Validate `props` and build the device.
    ///
    /// # Errors
    ///
    /// [`Error::Property`] for a property this class does not know or a value
    /// outside its range; [`Error::Config`] for an OEM identifier that does not
    /// fit its field.
    pub fn new(props: &Props) -> Result<AcpiTables> {
        let mut r = props.reader();
        let len = r.or_size("size", DEFAULT_LEN)?;
        let iospace = r.or_str("iospace", "port")?.to_string();
        let bus_name = r.or_str("bus", "pci0")?.to_string();
        let oem_id = r.or_str("oem-id", "RSEMU")?.to_string();
        let oem_table_id = r.or_str("oem-table-id", "RSEMUQ35")?.to_string();
        let cpus = r.or_range("cpus", 1u64, 1..=255)?;
        let ioapic_id = r.or_range("ioapic-id", 1u64, 0..=255)?;
        let gsi_base = r.or_range("gsi-base", 0u64, 0..=u64::from(u32::MAX))?;
        let sci_irq = r.or_range("sci-irq", 9u64, 0..=255)?;
        r.finish()?;
        let fit = |text: &str, width: usize, name: &str| -> Result<Vec<u8>> {
            if text.len() > width {
                return Err(Error::Config {
                    at: CLASS_NAME.to_string(),
                    message: alloc::format!(
                        "`{name}` is {width} characters in an ACPI description header \
                         (§5.2.6) and `{text}` is {}",
                        text.len()
                    ),
                });
            }
            let mut out = text.as_bytes().to_vec();
            out.resize(width, b' ');
            Ok(out)
        };
        let mut cfg = TableConfig {
            cpus: cpus as u8,
            ioapic_id: ioapic_id as u8,
            gsi_base: gsi_base as u32,
            sci_irq: sci_irq as u16,
            ..TableConfig::default()
        };
        cfg.oem_id.copy_from_slice(&fit(&oem_id, 6, "oem-id")?);
        cfg.oem_table_id
            .copy_from_slice(&fit(&oem_table_id, 8, "oem-table-id")?);
        // Acquiring a host object *is* allocation (`core::hosts`), so it
        // belongs in `new` beside the rest of it; nothing is announced.
        let bus = buses::attach(props, &bus_name)?;
        Ok(AcpiTables::with_config(len, cfg, iospace).on_bus(bus))
    }

    /// The same device, built from a configuration a test already has.
    #[must_use]
    pub fn with_config(len: u64, cfg: TableConfig, iospace: String) -> AcpiTables {
        let store = Arc::new(RamStore::new(len));
        let region: RegionRef = Arc::new(Region::ram(TABLES_REGION, Arc::clone(&store)));
        AcpiTables {
            store,
            region,
            len,
            cfg,
            iospace,
            bus: None,
            spaces: Mutex::with_rank(LockRank::LEAF, None),
            facts: Mutex::with_rank(LockRank::LEAF, MachineFacts::default()),
        }
    }

    /// The fabric whose functions `_PRT` describes.
    ///
    /// A plain strong handle, and it closes no cycle: this device is not a
    /// function on the bus, so nothing on the bus can reach back to it. The
    /// build owns the fabric anyway, under [`buses`].
    #[must_use]
    pub fn on_bus(mut self, bus: Arc<PciBus>) -> AcpiTables {
        self.bus = Some(bus);
        self
    }

    /// What the last generation found the machine to be.
    #[must_use]
    pub fn facts(&self) -> MachineFacts {
        self.facts.lock().clone()
    }

    /// The declared half of the tables.
    #[must_use]
    pub fn config(&self) -> &TableConfig {
        &self.cfg
    }

    /// The store the tables are written into.
    #[must_use]
    pub fn store(&self) -> &Arc<RamStore> {
        &self.store
    }

    /// Survey the machine and write the tables into the store.
    ///
    /// Called at reset, which is the first moment the whole graph exists — the
    /// argument `dev::riscv::dt` makes at length, and it holds harder here
    /// because the addresses this reads are set by *configuration registers*
    /// that only have their reset values once reset has run.
    ///
    /// # Errors
    ///
    /// Whatever [`generate`] refuses, plus [`Error::Config`] if the tables do
    /// not fit the region the machine file gave them.
    pub fn regenerate(&self) -> Result<()> {
        let spaces = self.spaces.lock().clone();
        let Some((mem, io)) = spaces else {
            // Not bound. Nothing to describe, and nothing has asked yet.
            return Ok(());
        };
        let facts = survey(&mem, &io, self.bus.as_deref());
        *self.facts.lock() = facts.clone();
        let Some(base) = facts.tables else {
            return Err(Error::Config {
                at: CLASS_NAME.to_string(),
                message: alloc::format!(
                    "the tables have to be at an address the guest can search, and this \
                     device's `{TABLES_REGION}` region is not mapped: add a `map` statement \
                     placing it between 0xe0000 and 0xfffff (ACPI §5.2.5.1)"
                ),
            });
        };
        let tables = generate(base, &facts, &self.cfg)?;
        if tables.bytes.len() as u64 > self.len {
            return Err(Error::Config {
                at: CLASS_NAME.to_string(),
                message: alloc::format!(
                    "the tables came to {} bytes and this device's region is {}: raise `size`",
                    tables.bytes.len(),
                    self.len
                ),
            });
        }
        self.store.fill(0, self.len, 0)?;
        self.store.write_at(0, &tables.bytes)?;
        Ok(())
    }
}

/// The `q35.acpi` device class.
pub static CLASS: DeviceClass = DeviceClass {
    name: CLASS_NAME,
    version: STATE_VERSION,
    summary: "the ACPI tables, generated from the realized machine and staged in guest memory",
    properties: &[
        PropertySpec {
            name: "size",
            kind: ValueKind::Size,
            required: false,
            summary: "how much guest memory the tables are staged in (default 64K)",
        },
        PropertySpec {
            name: "iospace",
            kind: ValueKind::Str,
            required: false,
            summary: "the space the ACPI register block is decoded in, so the FADT can find it \
                      (default `port`)",
        },
        PropertySpec {
            name: "bus",
            kind: ValueKind::Str,
            required: false,
            summary: "the PCI fabric whose functions `_PRT` describes (default `pci0`)",
        },
        PropertySpec {
            name: "oem-id",
            kind: ValueKind::Str,
            required: false,
            summary: "the six-character OEM identification every table carries",
        },
        PropertySpec {
            name: "oem-table-id",
            kind: ValueKind::Str,
            required: false,
            summary: "the eight-character OEM table identification",
        },
        PropertySpec {
            name: "cpus",
            kind: ValueKind::Uint,
            required: false,
            summary: "how many processors the MADT describes; not derivable, because a processor \
                      is not a region in any address space",
        },
        PropertySpec {
            name: "ioapic-id",
            kind: ValueKind::Uint,
            required: false,
            summary: "the I/O APIC's ID; not derivable, because reaching it means writing its \
                      index register, which a debug read may not do",
        },
        PropertySpec {
            name: "gsi-base",
            kind: ValueKind::Uint,
            required: false,
            summary: "the global system interrupt the I/O APIC's input 0 is (default 0)",
        },
        PropertySpec {
            name: "sci-irq",
            kind: ValueKind::Uint,
            required: false,
            summary: "which interrupt the SCI appears on — ACPI_CNTL[2:0]'s choice, which \
                      firmware makes after this table is written (default 9)",
        },
    ],
    construct: |props| Ok(Box::new(AcpiTables::new(props)?)),
};

impl Device for AcpiTables {
    fn class(&self) -> &'static DeviceClass {
        &CLASS
    }

    fn realize(&self, _ctx: &mut RealizeCtx<'_>) -> Result<()> {
        Ok(())
    }

    fn reset(&self, _kind: ResetKind) {
        // Both kinds: the tables are a *description*, not guest state, and a
        // machine that warm-reset with a stale set would be describing the
        // board it used to be. A failure cannot be reported from here — the
        // trait gives `reset` no result — so it leaves the region zeroed, which
        // an RSDP search reads as "no tables", and `regenerate` is public so a
        // test gets the error.
        if self.regenerate().is_err() {
            let _ = self.store.fill(0, self.len, 0);
        }
    }

    fn region(&self, name: &str) -> Option<RegionRef> {
        matches!(name, "" | "tables").then(|| Arc::clone(&self.region))
    }

    // No `save`/`load`. The tables are derived state — a pure function of the
    // machine's topology and its chipset registers — and `CLAUDE.md` says
    // derived state is never serialized. A snapshot load runs a reset sweep,
    // which rebuilds them from the machine that was restored.
}

impl Instance for AcpiTables {
    fn bind(&self, ctx: &BindCtx<'_>) -> Result<()> {
        let mem = ctx.space().ok_or_else(|| Error::Config {
            at: String::from(ctx.path()),
            message: String::from(
                "the tables are generated from a machine's memory map, so this device needs it: \
                 add `space = mem` to the object that declares it",
            ),
        })?;
        let io = ctx
            .space_named(&self.iospace)
            .ok_or_else(|| Error::Config {
                at: String::from(ctx.path()),
                message: alloc::format!(
                    "the FADT names the ACPI register block's I/O addresses, and this machine has \
                 no space called `{}`: name it with `iospace = \"…\"`",
                    self.iospace
                ),
            })?;
        *self.spaces.lock() = Some((Arc::clone(mem), Arc::clone(io)));
        Ok(())
    }
}

/// Add [`CLASS`] to a registry.
///
/// # Errors
///
/// [`Error::Config`] if the name is claimed.
pub fn register(registry: &mut crate::core::Registry) -> Result<()> {
    registry.add(&CLASS)
}

/// Bind [`CLASS`] into the machine graph.
///
/// # Errors
///
/// [`Error::Config`] if the class is bound twice.
pub fn bind(bindings: &mut crate::machine::Bindings) -> Result<()> {
    bindings.bind(CLASS_NAME, |props| Ok(Arc::new(AcpiTables::new(props)?)))
}

/// What the validator should know about `q35.acpi`.
#[must_use]
pub fn schema() -> ClassSchema {
    ClassSchema::new(CLASS_NAME)
        .prop(PropSchema::new("size", ValueKind::Size))
        .prop(PropSchema::new("iospace", ValueKind::Str))
        .prop(PropSchema::new("bus", ValueKind::Str))
        .prop(PropSchema::new("oem-id", ValueKind::Str))
        .prop(PropSchema::new("oem-table-id", ValueKind::Str))
        .prop(PropSchema::new("cpus", ValueKind::Uint).range(1, 255))
        .prop(PropSchema::new("ioapic-id", ValueKind::Uint).range(0, 255))
        .prop(PropSchema::new("gsi-base", ValueKind::Uint).range(0, u64::from(u32::MAX)))
        .prop(PropSchema::new("sci-irq", ValueKind::Uint).range(0, 255))
        .region("")
        .region("tables")
}
