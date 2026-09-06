//! `fw_cfg`: the selector/data register pair a firmware reads its own
//! configuration out of, and the ACPI tables this board hands OVMF through it.
//!
//! # Why a board needs this at all
//!
//! `machines/q35-uefi.machine` has no [`super::acpi`] device, and deliberately:
//! a UEFI firmware publishes its own ACPI set and hands the operating system an
//! RSDP through the EFI configuration table (UEFI 2.10 §4.6), so a board that
//! *also* staged a generated RSDP at `0xe0000` would be describing itself
//! twice. The consequence was that it described itself **nowhere**: an OVMF
//! build takes its tables from `fw_cfg` and this board had none, so a Linux
//! kernel booted through the EFI stub printed
//!
//! ```text
//! APIC: ACPI MADT or MP tables are not detected
//! ```
//!
//! and fell back to virtual wire mode. The firmware is not going to invent a
//! MADT; the *board* has to tell it what it is made of. This is the seam that
//! does it, and the tables it hands over are [`super::acpi`]'s — the same
//! generator, reading the same realized machine, packaged differently.
//!
//! # What the interface is
//!
//! Two registers in the I/O space and nothing else:
//!
//! ```text
//!   0x510  selector   16-bit write: which item subsequent data reads return
//!   0x511  data        8-bit read:  the next byte of the selected item
//! ```
//!
//! A write to the selector rewinds the item's read cursor to zero; each data
//! read hands back one byte and advances it. Items are numbered: a handful of
//! architectural keys, plus a **file directory** at key `0x19` naming
//! everything else. A firmware looks a file up by name in that directory, gets
//! back the key and the size, selects the key, and reads the bytes.
//!
//! Both numbers and both semantics are EDK II's, from
//! `OvmfPkg/Include/IndustryStandard/QemuFwCfg.h` and
//! `OvmfPkg/Library/QemuFwCfgLib` — BSD-2-Clause-Patent, and therefore a
//! permitted reference under `CLAUDE.md`. The interface itself originated in
//! QEMU, whose source is off limits to this repository; what is implemented
//! here is what the firmware we serve is written to consume, established from
//! the firmware's own permissively-licensed source.
//!
//! ## The signature is an interface identifier, not a claim
//!
//! Key `0x0000` reads `QEMU` because `QemuFwCfgInitialize` refuses the
//! interface unless it does — it is the four bytes that mean "this register
//! pair is the fw_cfg register pair", the way a class code of `010802h` is what
//! makes `NvmExpressDxe` bind to this board's disk controller. Answering
//! anything else is answering nothing: the firmware degrades cleanly and takes
//! no tables, which is exactly the state this device exists to leave.
//!
//! ## No DMA, on purpose
//!
//! Key `0x0001` reads `1`: the interface version, **without** `FW_CFG_F_DMA`.
//! The DMA path would have this device fetch a descriptor from guest memory and
//! copy an item into a guest buffer of the guest's choosing — a bus master
//! walking structures the guest built, inside an I/O write, with the
//! processor's `BUS`-ranked lock held. That is the class of hazard the virtio
//! work found live (four zero bytes of disk data are a notify of queue 0), and
//! it buys nothing here: EDK II's `InternalQemuFwCfgReadBytes` falls back to
//! `IoReadFifo8` — one `rep insb` — whenever the feature bit is clear, and a
//! whole ACPI table set is a few tens of kilobytes. The register block is
//! therefore two bytes wide and `0x514` is not decoded at all. A guest that
//! needs the DMA path (Linux's `fw_cfg` sysfs driver wants it; an MMIO
//! transport has nothing else) is the reason to add it, and then the
//! termination argument has to be made rather than assumed.
//!
//! ## What the permissive source does not settle
//!
//! Everything a firmware *does* is in EDK II, so nothing here had to be guessed
//! from a source this repository may not read. What EDK II cannot say is what
//! the device should do when a guest does something the firmware never does,
//! and four of those are decisions rather than facts:
//!
//! * a read past the end of an item hands back **zero** and leaves the cursor
//!   where it was — the firmware always knows a file's size from the directory
//!   before it reads it;
//! * a read of the **selector** faults rather than answering, because a
//!   register nothing is documented to read has no documented value and a bus
//!   fault says "not that" where a zero would not;
//! * a byte write to the selector's low half selects a key below 256, which is
//!   what a little-endian bus carrying half a 16-bit store would do;
//! * a write to the data register is accepted and dropped, since this device
//!   serves no writable file and faulting a probe would be worse than ignoring
//!   it.
//!
//! Each is stated here so that the day one of them turns out to matter, it is
//! visible as a choice rather than as an assumption.
//!
//! # What is served
//!
//! | Key | What |
//! | --- | --- |
//! | `0x0000` | the signature, `QEMU` |
//! | `0x0001` | the interface version, `1` |
//! | `0x0005` | the boot processor count |
//! | `0x000f` | the maximum processor count |
//! | `0x0019` | the file directory |
//! | `0x0020…` | one key per file, in the directory's order |
//!
//! and three files: `etc/acpi/tables`, `etc/acpi/rsdp` and `etc/table-loader`.
//! Everything else — `etc/e820`, `etc/system-states`, `etc/smbios/*`,
//! `bootorder`, `opt/…` — is deliberately **absent**, and every one of them is
//! a lookup EDK II is written to survive: memory sizing falls back to CMOS
//! `0x34`/`0x35` (`PlatformInitLib`'s `PlatformGetSystemMemorySizeBelow4gb`),
//! S3 stays disabled, SMBIOS comes from the firmware volume's own tables, and
//! the boot order stays the one in the variable store. A key that is not served
//! reads as zeroes, which is what an unset item does.
//!
//! # The blob and the linker script
//!
//! ACPI tables contain *addresses*, and nothing here knows where the firmware
//! will put them — it allocates them itself, from its own pool, after this
//! device has answered. So the tables are handed over as a **blob whose
//! pointers are offsets**, plus a script telling the firmware how to turn each
//! offset into an address:
//!
//! * `etc/acpi/tables` — FACS, DSDT, MADT, MCFG, HPET, FADT and XSDT, laid out
//!   back to back at 16-byte alignment (64 for the FACS, §5.2.10). Every
//!   pointer field holds the *offset* of its target inside this same blob, and
//!   every `Checksum` byte is **zero**.
//! * `etc/acpi/rsdp` — a revision 2 RSDP whose `XsdtAddress` is likewise the
//!   XSDT's offset into the tables blob.
//! * `etc/table-loader` — an array of 128-byte commands:
//!   `Allocate` (download this file into memory at this alignment),
//!   `AddPointer` (the field at this offset in this file is an offset into that
//!   file; add that file's base to it) and
//!   `AddChecksum` (sum this range and store the correction here).
//!
//! The checksums are zero in the blob because the firmware computes each one as
//! `0x100 - sum(range)` and stores it — over a range that *includes* the
//! checksum byte, so a non-zero one there would leave the table summing to
//! minus itself rather than to zero. Getting that backwards produces a table
//! set that a firmware silently declines to install, which is the failure this
//! whole file is about.
//!
//! Command order is load bearing in one place: every `AddPointer` that writes
//! into a range comes before the `AddChecksum` that covers it. The rest of the
//! order is what decides the order the firmware *installs* the tables in, since
//! EDK II identifies a table by following each `AddPointer` and looking for a
//! signature at the far end (`QemuFwCfgAcpi.c`, second pass).
//!
//! # Sources
//!
//! *ACPI Specification* revision 6.5 for every table byte — but by way of
//! [`super::acpi`], which builds them; this file only lays them out. EDK II
//! (BSD-2-Clause-Patent): `OvmfPkg/Include/IndustryStandard/QemuFwCfg.h` for
//! the register addresses, the keys and the directory's big-endian encoding;
//! `OvmfPkg/Include/IndustryStandard/QemuLoader.h` for the three command
//! structures and their 128-byte envelope;
//! `OvmfPkg/Library/AcpiPlatformLib/QemuFwCfgAcpi.c` for what the firmware
//! validates and in what order, including the checksum rule above;
//! `OvmfPkg/Library/QemuFwCfgLib/QemuFwCfgLib.c` for the directory walk and
//! `QemuFwCfgDxe.c` for the signature and feature-bit probe;
//! `OvmfPkg/Library/PlatformInitLib` for what a *missing* file falls back to.
//! No emulator source of any licence was consulted (`CLAUDE.md`, provenance).

use alloc::boxed::Box;
use alloc::collections::BTreeMap;
use alloc::string::{String, ToString};
use alloc::sync::Arc;
use alloc::vec::Vec;

use crate::bus::pci::{PciBus, buses};
use crate::core::device::{Device, DeviceClass, PropertySpec, RealizeCtx, ResetKind};
use crate::core::error::BusError;
use crate::core::props::{Props, ValueKind};
use crate::core::space::{AddressSpace, MemAttrs, MemOps, MemResult, Region, RegionRef};
use crate::core::state::{ChunkReader, ChunkWriter, Sink, Source};
use crate::core::sync::{LockRank, Mutex};
use crate::core::{Error, Result};
use crate::machine::realize::{BindCtx, Instance};
use crate::machine::validate::{ClassSchema, PropSchema};

use super::acpi::{self, MachineFacts, TableConfig};

/// The class name a machine description writes.
pub const CLASS_NAME: &str = "q35.fwcfg";

/// The snapshot chunk version. Bump with the encoding, never on its own.
const STATE_VERSION: u32 = 1;

/// How wide the register block is: a 16-bit selector and an 8-bit data port.
pub const REGISTER_WINDOW_LEN: u64 = 2;

/// Offset of the selector register inside the block.
const SELECTOR: u64 = 0;
/// Offset of the data register.
const DATA: u64 = 1;

/// Key `0x0000`: the four bytes that identify the interface.
const KEY_SIGNATURE: u16 = 0x0000;
/// Key `0x0001`: the interface version and its feature bits.
const KEY_INTERFACE_VERSION: u16 = 0x0001;
/// Key `0x0005`: how many processors are running.
const KEY_SMP_CPU_COUNT: u16 = 0x0005;
/// Key `0x000f`: how many processors there could ever be.
const KEY_MAX_CPU_COUNT: u16 = 0x000f;
/// Key `0x0019`: the file directory.
const KEY_FILE_DIR: u16 = 0x0019;

/// The first key a file is given. Below this the numbering is architectural.
const FIRST_FILE_KEY: u16 = 0x0020;

/// The signature key's contents.
const SIGNATURE: [u8; 4] = *b"QEMU";

/// The interface version this device implements: 1, with no feature bits — in
/// particular not `FW_CFG_F_DMA`. See the module docs.
const INTERFACE_VERSION: u32 = 1;

/// How long a file name is in the directory, including its terminating NUL.
const FNAME_SIZE: usize = 56;

/// How long one loader command is, whichever kind it is.
const LOADER_ENTRY_LEN: usize = 128;

/// `QemuLoaderCmdAllocate`.
const CMD_ALLOCATE: u32 = 1;
/// `QemuLoaderCmdAddPointer`.
const CMD_ADD_POINTER: u32 = 2;
/// `QemuLoaderCmdAddChecksum`.
const CMD_ADD_CHECKSUM: u32 = 3;

/// `QemuLoaderAllocHigh`: anywhere the firmware likes.
const ZONE_HIGH: u8 = 1;
/// `QemuLoaderAllocFSeg`: the legacy `0xf0000` segment, for a searcher that
/// only looks there. EDK II ignores the zone — it allocates from its own pool
/// either way — but the field is part of the command and a firmware that did
/// honour it should be told the truth.
const ZONE_FSEG: u8 = 2;

/// The file holding every table but the RSDP.
const FILE_TABLES: &str = "etc/acpi/tables";
/// The file holding the RSDP.
const FILE_RSDP: &str = "etc/acpi/rsdp";
/// The file holding the script that places the other two.
const FILE_LOADER: &str = "etc/table-loader";

/// What the tables blob is aligned to when the firmware places it: 64, because
/// the FACS is first in it and §5.2.10 requires a 64-byte boundary.
const TABLES_ALIGN: u32 = 64;
/// What the RSDP blob is aligned to: 16, because §5.2.5.1's search is on
/// 16-byte boundaries.
const RSDP_ALIGN: u32 = 16;

/// How the tables inside the blob are aligned against each other.
const ALIGN: u64 = 16;

// ---------------------------------------------------------------------------
// the files
// ---------------------------------------------------------------------------

/// A pointer field the firmware has to relocate: where it is, how wide it is,
/// and which file it points into.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct Pointer {
    /// The file the field lives in.
    pointer_file: &'static str,
    /// The file its value is an offset into.
    pointee_file: &'static str,
    /// Where in `pointer_file` the field is.
    offset: u32,
    /// How wide the field is: 1, 2, 4 or 8.
    size: u8,
}

/// A checksum the firmware has to compute: what to sum, and where the answer
/// goes.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct Checksum {
    /// The file it is computed over and stored in.
    file: &'static str,
    /// Where the answer goes.
    result: u32,
    /// Where the summed range starts.
    start: u32,
    /// How long it is.
    length: u32,
}

/// The whole ACPI hand-off: three files, in the order they are given keys.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct AcpiFiles {
    /// `etc/acpi/tables`.
    pub tables: Vec<u8>,
    /// `etc/acpi/rsdp`.
    pub rsdp: Vec<u8>,
    /// `etc/table-loader`.
    pub loader: Vec<u8>,
}

/// Round `at` up to a multiple of `align`.
fn align_up(at: u64, align: u64) -> u64 {
    at.div_ceil(align) * align
}

/// Zero the `Checksum` byte of the description header at `offset`.
///
/// The firmware computes `0x100 - sum(range)` over a range that includes this
/// byte, so it has to start at zero for the result to bring the table to zero.
fn clear_checksum(blob: &mut [u8], offset: u64, at: usize) {
    blob[offset as usize + at] = 0;
}

/// Build the three files that describe `facts` to a firmware.
///
/// The tables themselves are [`super::acpi`]'s; what is here is the layout, the
/// offsets that stand in for addresses, and the script that turns them back
/// into addresses.
///
/// # Errors
///
/// [`Error::Config`] if the machine has no local APIC — the same refusal
/// [`acpi::generate`] makes, and for the same reason: a table set with no
/// processor in it describes a machine an operating system cannot start on.
pub fn acpi_files(facts: &MachineFacts, cfg: &TableConfig) -> Result<AcpiFiles> {
    if facts.lapic.is_none() {
        return Err(Error::Config {
            at: CLASS_NAME.to_string(),
            message: String::from(
                "no local APIC is mapped in this machine's memory space, so the MADT handed to \
                 the firmware would describe no processor: add a `pc.lapic` and map its `regs` \
                 region",
            ),
        });
    }

    let mut blob: Vec<u8> = Vec::new();
    let mut pointers: Vec<Pointer> = Vec::new();
    let mut checksums: Vec<Checksum> = Vec::new();

    // Lay a table into the blob and hand back where it went.
    let place = |blob: &mut Vec<u8>, bytes: &[u8], align: u64| -> u64 {
        let at = align_up(blob.len() as u64, align);
        blob.resize(at as usize, 0);
        blob.extend_from_slice(bytes);
        at
    };

    // The FACS goes first, so that the blob's own 64-byte alignment is the
    // FACS's (§5.2.10). It has no description header and therefore no checksum.
    let facs_at = place(&mut blob, &acpi::facs(), 64);
    let dsdt_at = place(&mut blob, &acpi::dsdt(facts, cfg), ALIGN);

    // Everything the XSDT lists, in the order the firmware will install them.
    let mut listed: Vec<u64> = Vec::new();
    for table in [
        acpi::madt(facts, cfg),
        acpi::mcfg(facts, cfg),
        acpi::hpet(facts, cfg),
    ]
    .into_iter()
    .flatten()
    {
        listed.push(place(&mut blob, &table, ALIGN));
    }
    // The FADT is listed last, so that the tables which name nothing are
    // installed before the one that names the FACS and the DSDT. Those two are
    // reached only through it — §5.2.8 keeps them out of the XSDT — so the walk
    // meets the FADT first and its pointees after, which is the order a run of
    // this board produces a kernel log for: `FACP`, then `DSDT`, then `FACS`,
    // each at the address the firmware chose.
    let fadt_at = place(&mut blob, &acpi::fadt(facts, cfg, facs_at, dsdt_at), ALIGN);
    listed.push(fadt_at);
    let xsdt_at = place(&mut blob, &acpi::xsdt(cfg, &listed), ALIGN);

    // The XSDT's entries: each holds the offset of a table in this same blob.
    for index in 0..listed.len() {
        let offset = xsdt_at + acpi::HEADER_LEN as u64 + 8 * index as u64;
        pointers.push(Pointer {
            pointer_file: FILE_TABLES,
            pointee_file: FILE_TABLES,
            offset: offset as u32,
            size: 8,
        });
    }
    // The FADT's four: `FIRMWARE_CTRL` and `DSDT` (32-bit, Table 5.9) and their
    // 64-bit forms `X_FIRMWARE_CTRL` and `X_DSDT`. Both pairs, because §5.2.9
    // has an operating system prefer the 64-bit field where it is non-zero and
    // fall back to the 32-bit one where it is not — so a set that filled in
    // only one of each pair would be trusting every consumer to agree about
    // which.
    for (offset, size) in [(36u64, 4u8), (40, 4), (132, 8), (140, 8)] {
        pointers.push(Pointer {
            pointer_file: FILE_TABLES,
            pointee_file: FILE_TABLES,
            offset: (fadt_at + offset) as u32,
            size,
        });
    }

    // The RSDP: a separate file because a firmware that honours the zone puts
    // it where a legacy scan can find it and the tables where it likes.
    // `RsdtAddress` is left zero — this set has no RSDT, since the XSDT
    // describes the same tables with room for a 64-bit address, and EDK II
    // builds its own root tables from what it installs regardless.
    let mut rsdp = acpi::rsdp(cfg, 0, xsdt_at);
    pointers.push(Pointer {
        pointer_file: FILE_RSDP,
        pointee_file: FILE_TABLES,
        offset: 24,
        size: 8,
    });

    // Now the checksums, after every pointer that writes into these ranges.
    for at in listed.iter().copied().chain([xsdt_at, dsdt_at]) {
        let length = u32::from_le_bytes([
            blob[at as usize + 4],
            blob[at as usize + 5],
            blob[at as usize + 6],
            blob[at as usize + 7],
        ]);
        clear_checksum(&mut blob, at, acpi::CHECKSUM_OFFSET);
        checksums.push(Checksum {
            file: FILE_TABLES,
            result: at as u32 + acpi::CHECKSUM_OFFSET as u32,
            start: at as u32,
            length,
        });
    }
    // The RSDP carries two: the first covers bytes 0-19, which is all an ACPI
    // 1.0 consumer knows about, and the second the whole 36 (§5.2.5.3).
    rsdp[8] = 0;
    rsdp[32] = 0;
    checksums.push(Checksum {
        file: FILE_RSDP,
        result: 8,
        start: 0,
        length: 20,
    });
    checksums.push(Checksum {
        file: FILE_RSDP,
        result: 32,
        start: 0,
        length: 36,
    });

    // And the script itself.
    let mut loader: Vec<u8> = Vec::new();
    loader.extend_from_slice(&allocate(FILE_RSDP, RSDP_ALIGN, ZONE_FSEG));
    loader.extend_from_slice(&allocate(FILE_TABLES, TABLES_ALIGN, ZONE_HIGH));
    for pointer in &pointers {
        loader.extend_from_slice(&add_pointer(*pointer));
    }
    for checksum in &checksums {
        loader.extend_from_slice(&add_checksum(*checksum));
    }

    Ok(AcpiFiles {
        tables: blob,
        rsdp,
        loader,
    })
}

/// A `QEMU_LOADER_ENTRY` of `kind`, with `body` laid into its command union.
fn entry(kind: u32, body: &[u8]) -> [u8; LOADER_ENTRY_LEN] {
    let mut out = [0u8; LOADER_ENTRY_LEN];
    out[..4].copy_from_slice(&kind.to_le_bytes());
    out[4..4 + body.len()].copy_from_slice(body);
    out
}

/// A file name as a command carries it: NUL-terminated, in a fixed field.
fn fname(name: &str) -> [u8; FNAME_SIZE] {
    let mut out = [0u8; FNAME_SIZE];
    // Every name in this file is a constant well under the field, and a longer
    // one is truncated rather than overrunning — it would then not be found in
    // the directory, which is a loud failure rather than a silent overwrite.
    let bytes = name.as_bytes();
    let take = bytes.len().min(FNAME_SIZE - 1);
    out[..take].copy_from_slice(&bytes[..take]);
    out
}

/// `QemuLoaderCmdAllocate`: download `file` and place it at `align`.
fn allocate(file: &str, align: u32, zone: u8) -> [u8; LOADER_ENTRY_LEN] {
    let mut body = Vec::with_capacity(FNAME_SIZE + 5);
    body.extend_from_slice(&fname(file));
    body.extend_from_slice(&align.to_le_bytes());
    body.push(zone);
    entry(CMD_ALLOCATE, &body)
}

/// `QemuLoaderCmdAddPointer`: relocate one field.
fn add_pointer(p: Pointer) -> [u8; LOADER_ENTRY_LEN] {
    let mut body = Vec::with_capacity(2 * FNAME_SIZE + 5);
    body.extend_from_slice(&fname(p.pointer_file));
    body.extend_from_slice(&fname(p.pointee_file));
    body.extend_from_slice(&p.offset.to_le_bytes());
    body.push(p.size);
    entry(CMD_ADD_POINTER, &body)
}

/// `QemuLoaderCmdAddChecksum`: sum a range and store the correction.
fn add_checksum(c: Checksum) -> [u8; LOADER_ENTRY_LEN] {
    let mut body = Vec::with_capacity(FNAME_SIZE + 12);
    body.extend_from_slice(&fname(c.file));
    body.extend_from_slice(&c.result.to_le_bytes());
    body.extend_from_slice(&c.start.to_le_bytes());
    body.extend_from_slice(&c.length.to_le_bytes());
    entry(CMD_ADD_CHECKSUM, &body)
}

// ---------------------------------------------------------------------------
// what is served, and where the cursor is
// ---------------------------------------------------------------------------

/// Everything the interface answers with, keyed by selector.
///
/// Derived state: a pure function of the realized machine, rebuilt at reset and
/// never serialized (`CLAUDE.md`, devices).
#[derive(Debug, Clone, Default, PartialEq, Eq)]
struct Content {
    /// Every item, in key order — which is also the order a directory lists
    /// them, so nothing here depends on a hash.
    items: BTreeMap<u16, Vec<u8>>,
    /// What each file is called, for a trace of what a firmware asked for.
    names: BTreeMap<u16, String>,
}

impl Content {
    /// Build the whole set from `files`, which are named and already ordered.
    fn build(cpus: u8, files: Vec<(String, Vec<u8>)>) -> Content {
        let mut items = BTreeMap::new();
        let mut names = BTreeMap::new();
        items.insert(KEY_SIGNATURE, SIGNATURE.to_vec());
        items.insert(
            KEY_INTERFACE_VERSION,
            INTERFACE_VERSION.to_le_bytes().to_vec(),
        );
        // The processor counts are 16-bit and *little*-endian, where the
        // directory below is big-endian. That is not a guess: EDK II reads this
        // one with a bare `QemuFwCfgRead16` and the directory's fields through
        // `SwapBytes16`/`SwapBytes32`, so the two halves of the interface
        // genuinely differ. Answering it matters — `PlatformMaxCpuCountInitialization`
        // reads zero as "the platform does not know" and then counts
        // application processors by timeout rather than by number.
        items.insert(KEY_SMP_CPU_COUNT, u16::from(cpus).to_le_bytes().to_vec());
        items.insert(KEY_MAX_CPU_COUNT, u16::from(cpus).to_le_bytes().to_vec());

        // The directory, and the keys it hands out.
        let mut dir: Vec<u8> = Vec::new();
        dir.extend_from_slice(&(files.len() as u32).to_be_bytes());
        for (index, (name, bytes)) in files.into_iter().enumerate() {
            let key = FIRST_FILE_KEY + index as u16;
            dir.extend_from_slice(&(bytes.len() as u32).to_be_bytes());
            dir.extend_from_slice(&key.to_be_bytes());
            // Reserved, and zero.
            dir.extend_from_slice(&0u16.to_be_bytes());
            dir.extend_from_slice(&fname(&name));
            items.insert(key, bytes);
            names.insert(key, name);
        }
        items.insert(KEY_FILE_DIR, dir);
        Content { items, names }
    }

    /// The bytes key `selector` answers with, if any.
    fn item(&self, selector: u16) -> Option<&[u8]> {
        self.items.get(&selector).map(Vec::as_slice)
    }
}

/// The guest-visible half: which item is selected and how far into it the last
/// read got.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
struct Cursor {
    /// The selector the guest last wrote.
    selector: u16,
    /// How many bytes of it have been read.
    offset: u64,
}

/// How many selector writes the trace below remembers.
const TRACE_LEN: usize = 64;

/// The served set plus the cursor into it, under one lock.
///
/// One lock rather than two because every data read touches both, and a rank
/// cannot order a lock against itself.
#[derive(Debug, Default)]
struct Served {
    content: Content,
    cursor: Cursor,
    /// The last [`TRACE_LEN`] selectors the guest wrote, newest last.
    ///
    /// A **diagnostic**, not machine state: nothing the guest can read depends
    /// on it, a snapshot does not carry it, and it is bounded so that a guest
    /// spinning on the selector cannot grow it. It exists because a firmware
    /// that goes quiet is this board's recurring problem and "which files did
    /// it ask for" is otherwise unanswerable from outside — the answer would
    /// live in a `DEBUG()` log on a debug port this board does not have.
    trace: Vec<u16>,
}

impl Served {
    /// Record a selector write.
    fn traced(&mut self, selector: u16) {
        if self.trace.len() == TRACE_LEN {
            self.trace.remove(0);
        }
        self.trace.push(selector);
    }
}

/// The register block, as the address space sees it.
#[derive(Debug)]
struct Ports(Arc<Mutex<Served>>);

impl MemOps for Ports {
    fn read(&self, offset: u64, dst: &mut [u8], attrs: MemAttrs) -> MemResult {
        if offset != DATA || dst.len() != 1 {
            // The selector is write-only and the data register is a byte wide:
            // `IoReadFifo8` is the only read the interface defines, and a wider
            // one would be a guest asking for something this pair cannot do.
            return Err(BusError::BadAccess);
        }
        let mut served = self.0.lock();
        let cursor = served.cursor;
        let byte = served
            .content
            .item(cursor.selector)
            .and_then(|bytes| bytes.get(cursor.offset as usize).copied())
            // Past the end of an item, and every unset key, reads as zero.
            .unwrap_or(0);
        // A debugger may look at the byte under the cursor; it may not move it.
        if !attrs.debug {
            served.cursor.offset = cursor.offset.saturating_add(1);
        }
        dst[0] = byte;
        Ok(())
    }

    fn write(&self, offset: u64, src: &[u8], attrs: MemAttrs) -> MemResult {
        if attrs.debug {
            // Both registers have a side effect a debugger must not cause: one
            // rewinds the cursor and the other advances it.
            return Err(BusError::BadAccess);
        }
        match (offset, src.len()) {
            // The selector, whole. A write rewinds the item.
            (SELECTOR, 2) => {
                let selector = u16::from_le_bytes([src[0], src[1]]);
                let mut served = self.0.lock();
                served.cursor = Cursor {
                    selector,
                    offset: 0,
                };
                served.traced(selector);
                Ok(())
            }
            // Half of it: a byte store to the low half of a 16-bit register is
            // an ordinary thing for a bus to carry, and it selects a key below
            // 256 — which every architectural key is.
            (SELECTOR, 1) => {
                let mut served = self.0.lock();
                served.cursor = Cursor {
                    selector: u16::from(src[0]),
                    offset: 0,
                };
                served.traced(u16::from(src[0]));
                Ok(())
            }
            // The data register is writable on the real interface, for files a
            // firmware writes back. This device serves none, so a write is
            // accepted and dropped rather than faulting a guest that probes.
            (DATA, 1) => Ok(()),
            _ => Err(BusError::BadAccess),
        }
    }
}

// ---------------------------------------------------------------------------
// the device
// ---------------------------------------------------------------------------

/// The `fw_cfg` interface, answering out of the realized machine.
#[derive(Debug)]
pub struct FwCfg {
    served: Arc<Mutex<Served>>,
    ports: RegionRef,
    cfg: TableConfig,
    iospace: String,
    /// The fabric `_PRT` describes, if the board has one.
    bus: Option<Arc<PciBus>>,
    /// The two spaces the survey walks. `None` until [`Instance::bind`].
    /// [`LockRank::LEAF`].
    spaces: Mutex<Option<(Arc<AddressSpace>, Arc<AddressSpace>)>>,
    /// What the last generation found. [`LockRank::LEAF`].
    facts: Mutex<MachineFacts>,
}

impl FwCfg {
    /// Validate `props` and build the device.
    ///
    /// # Errors
    ///
    /// [`Error::Property`] for a property this class does not know or a value
    /// outside its range; [`Error::Config`] for an OEM identifier that does not
    /// fit its field.
    pub fn new(props: &Props) -> Result<FwCfg> {
        let mut r = props.reader();
        let iospace = r.or_str("iospace", "port")?.to_string();
        let bus_name = r.or_str("bus", "pci0")?.to_string();
        let cfg = TableConfig::read(&mut r)?;
        r.finish()?;
        let bus = buses::attach(props, &bus_name)?;
        Ok(FwCfg::with_config(cfg, iospace).on_bus(bus))
    }

    /// The same device, built from a configuration a test already has.
    #[must_use]
    pub fn with_config(cfg: TableConfig, iospace: String) -> FwCfg {
        let served = Arc::new(Mutex::with_rank(LockRank::LEAF, Served::default()));
        let ports: RegionRef = Arc::new(Region::io(
            "q35.fwcfg.regs",
            REGISTER_WINDOW_LEN,
            Arc::new(Ports(Arc::clone(&served))) as Arc<dyn MemOps>,
        ));
        FwCfg {
            served,
            ports,
            cfg,
            iospace,
            bus: None,
            spaces: Mutex::with_rank(LockRank::LEAF, None),
            facts: Mutex::with_rank(LockRank::LEAF, MachineFacts::default()),
        }
    }

    /// The fabric `_PRT` describes. See [`acpi::AcpiTables::on_bus`].
    #[must_use]
    pub fn on_bus(mut self, bus: Arc<PciBus>) -> FwCfg {
        self.bus = Some(bus);
        self
    }

    /// What the last generation found the machine to be.
    #[must_use]
    pub fn facts(&self) -> MachineFacts {
        self.facts.lock().clone()
    }

    /// Which item is selected and how far into it the guest has read — the
    /// whole of what a snapshot carries.
    #[must_use]
    pub fn cursor(&self) -> (u16, u64) {
        let cursor = self.served.lock().cursor;
        (cursor.selector, cursor.offset)
    }

    /// The last few selectors the guest wrote, oldest first, each paired with
    /// the file it names where it names one.
    ///
    /// What a firmware asked this board for, in order. A **diagnostic**: it is
    /// bounded, a snapshot does not carry it, and nothing the guest can read
    /// depends on it. It exists because a firmware that goes quiet is this
    /// board's recurring problem, and a selector write leaves nothing behind
    /// in any address space.
    #[must_use]
    pub fn trace(&self) -> Vec<(u16, Option<String>)> {
        let served = self.served.lock();
        served
            .trace
            .iter()
            .map(|key| (*key, served.content.names.get(key).cloned()))
            .collect()
    }

    /// The bytes key `selector` answers with, as a test or a trace sees them.
    #[must_use]
    pub fn item(&self, selector: u16) -> Option<Vec<u8>> {
        self.served
            .lock()
            .content
            .item(selector)
            .map(<[u8]>::to_vec)
    }

    /// The file `name` names, if it is served.
    #[must_use]
    pub fn file(&self, name: &str) -> Option<Vec<u8>> {
        let served = self.served.lock();
        let key = served
            .content
            .names
            .iter()
            .find_map(|(key, file)| (file == name).then_some(*key))?;
        served.content.item(key).map(<[u8]>::to_vec)
    }

    /// Which file each key holds, in key order — what a trace of the selectors
    /// a firmware wrote has to be read against.
    #[must_use]
    pub fn directory(&self) -> Vec<(u16, String)> {
        self.served
            .lock()
            .content
            .names
            .iter()
            .map(|(key, name)| (*key, name.clone()))
            .collect()
    }

    /// Survey the machine and rebuild everything served.
    ///
    /// Called at reset, which is the first moment the whole graph exists — the
    /// same argument [`acpi::AcpiTables::regenerate`] makes, and it holds
    /// harder here: the addresses this reads come from configuration registers
    /// that only have their reset values once reset has run.
    ///
    /// # Errors
    ///
    /// Whatever [`acpi_files`] refuses.
    pub fn regenerate(&self) -> Result<()> {
        let spaces = self.spaces.lock().clone();
        let Some((mem, io)) = spaces else {
            // Not bound. Nothing to describe, and nothing has asked yet.
            return Ok(());
        };
        // The survey reads other devices, so it happens with no lock of this
        // device's held (`CLAUDE.md`, re-entrancy).
        let facts = acpi::survey(&mem, &io, self.bus.as_deref());
        let files = acpi_files(&facts, &self.cfg)?;
        let content = Content::build(
            self.cfg.cpus,
            alloc::vec![
                // Named in the order the directory lists them, which is the
                // order the keys are handed out in.
                (String::from(FILE_RSDP), files.rsdp),
                (String::from(FILE_TABLES), files.tables),
                (String::from(FILE_LOADER), files.loader),
            ],
        );
        *self.facts.lock() = facts;
        self.served.lock().content = content;
        Ok(())
    }
}

/// The `q35.fwcfg` device class.
pub static CLASS: DeviceClass = DeviceClass {
    name: CLASS_NAME,
    version: STATE_VERSION,
    summary: "the fw_cfg selector/data pair, answering a firmware with ACPI tables generated \
              from the realized machine",
    properties: &[
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
            summary: "how many processors the MADT describes and key 0x05 reports; not \
                      derivable, because a processor is not a region in any address space",
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
            summary: "which interrupt the SCI appears on (default 9)",
        },
    ],
    construct: |props| Ok(Box::new(FwCfg::new(props)?)),
};

impl Device for FwCfg {
    fn class(&self) -> &'static DeviceClass {
        &CLASS
    }

    fn realize(&self, _ctx: &mut RealizeCtx<'_>) -> Result<()> {
        Ok(())
    }

    fn reset(&self, _kind: ResetKind) {
        // Both kinds, for [`acpi::AcpiTables`]'s reason: what is served is a
        // *description*, and a machine that warm-reset with a stale one would
        // be describing the board it used to be. A failure cannot be reported
        // from here, so it leaves the interface empty — which reads as "no
        // tables" rather than as wrong ones — and `regenerate` is public so a
        // test gets the error.
        //
        // The cursor is rewound whatever happens, and before anything else: a
        // cursor into an item that is about to be rebuilt is not a cursor into
        // anything, and that is true of a board whose device was never bound as
        // much as of one whose tables were regenerated.
        {
            let mut served = self.served.lock();
            served.cursor = Cursor::default();
            served.trace.clear();
        }
        if self.regenerate().is_err() {
            self.served.lock().content = Content::default();
        }
    }

    fn region(&self, name: &str) -> Option<RegionRef> {
        matches!(name, "" | "regs").then(|| Arc::clone(&self.ports))
    }

    fn save(&self, w: &mut ChunkWriter<'_>) -> Result<()> {
        // The cursor and nothing else. What is served is derived from the
        // machine's topology and is rebuilt by the reset sweep a load runs —
        // but *where the guest is inside an item* is guest-visible state, and a
        // firmware snapshotted halfway through reading a table has to come back
        // halfway through reading it.
        let cursor = self.served.lock().cursor;
        w.write_u16(cursor.selector)?;
        w.write_u64(cursor.offset)
    }

    fn load(&self, r: &mut ChunkReader<'_>) -> Result<()> {
        let cursor = Cursor {
            selector: r.read_u16()?,
            offset: r.read_u64()?,
        };
        self.served.lock().cursor = cursor;
        Ok(())
    }
}

impl Instance for FwCfg {
    fn bind(&self, ctx: &BindCtx<'_>) -> Result<()> {
        let mem = ctx.space().ok_or_else(|| Error::Config {
            at: String::from(ctx.path()),
            message: String::from(
                "the tables handed to the firmware are generated from a machine's memory map, so \
                 this device needs it: add `space = mem` to the object that declares it",
            ),
        })?;
        let io = ctx
            .space_named(&self.iospace)
            .ok_or_else(|| Error::Config {
                at: String::from(ctx.path()),
                message: alloc::format!(
                    "the FADT names the ACPI register block's I/O addresses, and this machine \
                     has no space called `{}`: name it with `iospace = \"…\"`",
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
    bindings.bind(CLASS_NAME, |props| Ok(Arc::new(FwCfg::new(props)?)))
}

/// What the validator should know about `q35.fwcfg`.
#[must_use]
pub fn schema() -> ClassSchema {
    ClassSchema::new(CLASS_NAME)
        .prop(PropSchema::new("iospace", ValueKind::Str))
        .prop(PropSchema::new("bus", ValueKind::Str))
        .prop(PropSchema::new("oem-id", ValueKind::Str))
        .prop(PropSchema::new("oem-table-id", ValueKind::Str))
        .prop(PropSchema::new("cpus", ValueKind::Uint).range(1, 255))
        .prop(PropSchema::new("ioapic-id", ValueKind::Uint).range(0, 255))
        .prop(PropSchema::new("gsi-base", ValueKind::Uint).range(0, u64::from(u32::MAX)))
        .prop(PropSchema::new("sci-irq", ValueKind::Uint).range(0, 255))
        .region("")
        .region("regs")
}

#[cfg(test)]
mod tests {
    use super::*;

    use crate::core::space::{Mapping, UnassignedPolicy};
    use crate::core::state::{MachineShape, Migrations, StateReader, StateWriter};
    use crate::core::value::Width;

    /// The facts a fully populated q35 presents — the same machine
    /// `super::super::tests` describes, restated here because these tests are
    /// about the packaging and want to name their own input.
    fn facts() -> MachineFacts {
        MachineFacts {
            lapic: Some((0xfee0_0000, 0)),
            ioapic: Some(0xfec0_0000),
            hpet: Some((0xfed0_0000, 0x8086_a201)),
            acpi_io: Some(0x600),
            ecam: Some((0xe000_0000, 256 * 1024 * 1024)),
            tables: None,
            ram_top: Some(0x1010_0000),
            config_ports: Some((0xcf8, 8)),
            prt: Vec::new(),
        }
    }

    /// The name a command's file field holds, at `at`.
    fn name_at(cmd: &[u8], at: usize) -> String {
        let field = &cmd[at..at + FNAME_SIZE];
        assert_eq!(field[FNAME_SIZE - 1], 0, "the field is NUL-terminated");
        let end = field.iter().position(|b| *b == 0).expect("NUL-terminated");
        String::from_utf8_lossy(&field[..end]).into_owned()
    }

    #[test]
    fn a_machine_with_no_local_apic_is_refused_rather_than_described() {
        let err = acpi_files(&MachineFacts::default(), &TableConfig::default())
            .expect_err("a MADT with no processor is not a machine");
        assert!(
            alloc::format!("{err}").contains("local APIC"),
            "the error should name what is missing: {err}"
        );
    }

    #[test]
    fn every_checksum_the_script_fills_in_starts_at_zero() {
        // The rule the whole hand-off turns on: EDK II stores
        // `0x100 - sum(range)` at `ResultOffset`, over a range that *includes*
        // that byte. A table shipped with its checksum already correct comes
        // out summing to minus that byte rather than to zero, and a firmware
        // that cannot checksum a table declines to install it — silently.
        let files = acpi_files(&facts(), &TableConfig::default()).expect("there is a processor");
        let of = |name: &str| -> &[u8] {
            match name {
                FILE_TABLES => &files.tables,
                FILE_RSDP => &files.rsdp,
                _ => panic!("no such file {name}"),
            }
        };
        let mut checksums = 0;
        for cmd in files.loader.chunks(LOADER_ENTRY_LEN) {
            if u32::from_le_bytes(cmd[..4].try_into().expect("four")) != CMD_ADD_CHECKSUM {
                continue;
            }
            let name = name_at(cmd, 4);
            let result = u32::from_le_bytes(cmd[60..64].try_into().expect("four")) as usize;
            assert_eq!(of(&name)[result], 0, "{name} + {result:#x}");
            checksums += 1;
        }
        // Five tables with a description header — DSDT, MADT, MCFG, HPET, FADT
        // — plus the XSDT, plus the RSDP's pair. The FACS has no header and
        // therefore no checksum (§5.2.10).
        assert_eq!(checksums, 8, "one per table plus the RSDP's pair");
    }

    #[test]
    fn every_command_names_a_file_that_is_served_and_a_field_inside_it() {
        let files = acpi_files(&facts(), &TableConfig::default()).expect("there is a processor");
        assert_eq!(files.loader.len() % LOADER_ENTRY_LEN, 0);
        let len = |name: &str| -> usize {
            match name {
                FILE_TABLES => files.tables.len(),
                FILE_RSDP => files.rsdp.len(),
                _ => panic!("no such file {name}"),
            }
        };
        let (mut allocates, mut pointers) = (0, 0);
        for cmd in files.loader.chunks(LOADER_ENTRY_LEN) {
            match u32::from_le_bytes(cmd[..4].try_into().expect("four")) {
                CMD_ALLOCATE => {
                    let align = u32::from_le_bytes(cmd[60..64].try_into().expect("four"));
                    // EDK II refuses an alignment above a page, and something
                    // that is not a power of two is not an alignment.
                    assert!(align.is_power_of_two() && align <= 4096, "{align}");
                    let _ = len(&name_at(cmd, 4));
                    allocates += 1;
                }
                CMD_ADD_POINTER => {
                    let size = cmd[120] as usize;
                    assert!(matches!(size, 1 | 2 | 4 | 8), "pointer size {size}");
                    let at = u32::from_le_bytes(cmd[116..120].try_into().expect("four")) as usize;
                    let file = name_at(cmd, 4);
                    assert!(at + size <= len(&file), "{file} + {at:#x} is past its end");
                    let _ = len(&name_at(cmd, 60));
                    pointers += 1;
                }
                CMD_ADD_CHECKSUM => {
                    let file = name_at(cmd, 4);
                    let start = u32::from_le_bytes(cmd[64..68].try_into().expect("four")) as usize;
                    let length = u32::from_le_bytes(cmd[68..72].try_into().expect("four")) as usize;
                    assert!(start + length <= len(&file), "{file} + {start:#x}");
                }
                other => panic!("command {other} is not one of the three"),
            }
        }
        assert_eq!(allocates, 2, "the tables and the RSDP");
        // Four XSDT entries, the FADT's four, and the RSDP's one.
        assert_eq!(pointers, 9);
    }

    #[test]
    fn a_pointer_field_holds_an_offset_into_the_file_it_points_at() {
        // Which is what makes the blob relocatable: nothing in it is an
        // address, so the firmware's own allocator decides where it lands.
        let files = acpi_files(&facts(), &TableConfig::default()).expect("there is a processor");
        for cmd in files.loader.chunks(LOADER_ENTRY_LEN) {
            if u32::from_le_bytes(cmd[..4].try_into().expect("four")) != CMD_ADD_POINTER {
                continue;
            }
            let at = u32::from_le_bytes(cmd[116..120].try_into().expect("four")) as usize;
            let size = cmd[120] as usize;
            let bytes = if name_at(cmd, 4) == FILE_RSDP {
                &files.rsdp
            } else {
                &files.tables
            };
            let mut value = [0u8; 8];
            value[..size].copy_from_slice(&bytes[at..at + size]);
            let value = u64::from_le_bytes(value) as usize;
            assert_eq!(
                name_at(cmd, 60),
                FILE_TABLES,
                "everything points at one file"
            );
            assert!(
                value < files.tables.len(),
                "{value:#x} is not an offset into a {} byte file",
                files.tables.len()
            );
            // And what it points at is a table, not the middle of one.
            let signature = &files.tables[value..value + 4];
            assert!(
                [
                    b"FACS", b"DSDT", b"APIC", b"MCFG", b"HPET", b"FACP", b"XSDT"
                ]
                .iter()
                .any(|s| signature == *s),
                "the field at {at:#x} points at {signature:?}, which is not a table"
            );
        }
    }

    /// A device with its registers mapped, and nothing else. Unbound, so it
    /// describes no machine: what these two tests are about is the cursor.
    fn rig() -> (Arc<AddressSpace>, Arc<FwCfg>) {
        let io = Arc::new(AddressSpace::new("port", 16).with_unassigned(UnassignedPolicy::ONES));
        let dev = Arc::new(FwCfg::with_config(
            TableConfig::default(),
            String::from("port"),
        ));
        let region = dev.region("regs").expect("the device publishes one");
        io.topology()
            .map_with(Mapping::new(region, 0x510))
            .expect("it fits");
        (io, dev)
    }

    #[test]
    fn a_snapshot_taken_mid_transfer_comes_back_mid_transfer() {
        // The cursor is the only guest-visible state here — what is *served* is
        // a description of the machine, rebuilt by the reset sweep a load runs
        // — but a firmware caught halfway through a table has to come back
        // halfway through it, and a snapshot that dropped the offset would hand
        // it the same bytes twice.
        let (io, a) = rig();
        io.write(0x510, Width::U16, 0x0019, MemAttrs::DEFAULT)
            .expect("the selector");
        for _ in 0..5 {
            io.read(0x511, Width::U8, MemAttrs::DEFAULT)
                .expect("the data port");
        }

        let saved = {
            let mut shape = MachineShape::new();
            shape.add_device("fwcfg", CLASS_NAME).expect("unique");
            let mut w = StateWriter::new(shape);
            {
                let mut chunk = w.chunk("fwcfg", CLASS_NAME, STATE_VERSION).expect("one");
                a.save(&mut chunk).expect("saves");
            }
            w.to_vec().expect("encodes")
        };

        let (_, b) = rig();
        let reader = StateReader::new(&saved).expect("it parses");
        let chunk = reader
            .load("fwcfg", CLASS_NAME, STATE_VERSION, &Migrations::new())
            .expect("the chunk is there");
        b.load(&mut chunk.reader()).expect("it loads");
        assert_eq!(a.cursor(), b.cursor());
        assert_eq!(a.cursor(), (0x0019, 5));
    }

    #[test]
    fn a_reset_rewinds_the_interface() {
        // A cursor into an item that has just been rebuilt is not a cursor into
        // anything, and a firmware always selects before it reads.
        let (io, dev) = rig();
        io.write(0x510, Width::U16, 0x0005, MemAttrs::DEFAULT)
            .expect("the selector");
        io.read(0x511, Width::U8, MemAttrs::DEFAULT)
            .expect("the data port");
        assert_eq!(dev.cursor(), (0x0005, 1));
        dev.reset(ResetKind::Warm);
        assert_eq!(dev.cursor(), (0, 0));
    }

    #[test]
    fn the_selector_is_write_only_and_the_data_port_is_a_byte_wide() {
        // `IoReadFifo8` is the only read the interface defines, and the
        // selector answers none: a guest reading it is asking for something
        // this pair cannot do, and a bus fault says so where a zero would not.
        //
        // Note what is *not* asserted: a two-byte read at 0x511. The block is
        // two bytes wide, so such an access is split by the address space and
        // only its first byte ever reaches this device — the width check inside
        // it is unreachable from a board and would be a test of the space.
        let (io, _dev) = rig();
        assert!(io.read(0x510, Width::U8, MemAttrs::DEFAULT).is_err());
        assert!(io.read(0x510, Width::U16, MemAttrs::DEFAULT).is_err());
        assert!(io.read(0x511, Width::U8, MemAttrs::DEFAULT).is_ok());
    }
}
