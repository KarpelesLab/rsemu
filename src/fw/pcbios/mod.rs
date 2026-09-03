//! A minimal legacy PC BIOS, in a 64 KiB ROM image rsemu assembles itself.
//!
//! [`image`] returns the bytes. Bind them to the `pc-at` board's `bios` media
//! slot and the board boots with no external firmware at all:
//!
//! ```no_run
//! # #[cfg(all(feature = "machine-pc-at", feature = "std"))] {
//! let mut options = rsemu::machine::BuildOptions::new();
//! options.realize.media.insert("bios", rsemu::fw::pcbios::image());
//! # }
//! ```
//!
//! # Sources, and what was deliberately not read
//!
//! Every interrupt function and every register write below is cited on the item
//! that performs it. The register of sources is:
//!
//! * **Ralf Brown's Interrupt List** for the interrupt ABI — which register
//!   carries which argument, and what a function returns. RBIL is a catalogue of
//!   *interfaces*, compiled from observation and from vendor documentation; it
//!   contains no implementation.
//! * The **BIOS Data Area** field list as documented on the OSDev wiki and in
//!   RBIL's `MEMORY` file. The BDA is an ABI: its offsets are facts that any
//!   program which reads `0040:0013` depends on.
//! * The chip data sheets for everything that is a register write: **Intel
//!   8259A** (the ICW/OCW sequence), **Intel 8254** (the control word and the
//!   read-back command), **Motorola MC146818** (the CMOS map and status
//!   registers A-D), **Intel 8042** (the controller commands and the output
//!   port), **Intel 8237A** (the mode/mask/address registers and the page
//!   latches), and **NEC µPD765A** (the command and result phases).
//! * **T13/1410D (ATA/ATAPI-6)** for `IDENTIFY DEVICE`, `READ SECTOR(S)` and
//!   the command-block register file.
//! * The **Intel SDM** Volume 2 for every instruction encoding, through
//!   [`crate::fw::asm16`].
//! * The **PCI Firmware Specification** 3.0 §5.2.2 for the option-ROM header
//!   and how an expansion ROM is entered.
//!
//! **The IBM PC/AT Technical Reference's printed BIOS listing was not used.**
//! It is Copyright IBM with no licence grant of any kind — publication is not a
//! licence — so for our purposes it sits with the copyleft sources: the *board*
//! facts it documents (which chip answers which port, which IRQ a device lands
//! on) are equally available from the data sheets and are used from those, and
//! its code was not read. No emulator's firmware was read either; the common
//! ones are GPL or LGPL (`ROADMAP.md` §1).
//!
//! # What it does
//!
//! POST brings up the chipset, fills the BIOS Data Area, scans for option ROMs,
//! finds the disks and boots. It is enough for **FreeDOS 1.3 to boot off a
//! diskette** and reach an interactive prompt, which is `ROADMAP.md` phase 6a's
//! gate; `tests/pc_at_boot.rs` runs that, gated on an image nothing here
//! vendors.
//!
//! | | |
//! | --- | --- |
//! | `INT 10h` | video: set mode, cursor, teletype, scroll, read and write cells, write string |
//! | `INT 11h` | the equipment word |
//! | `INT 12h` | base memory size |
//! | `INT 13h` | disk: the IDE channel and the µPD765 both, read, write and format, plus the EDD subset |
//! | `INT 15h` | `E820`, `E801`, `AH=88h` — the memory map — and `AH=87h`, block move |
//! | `INT 16h` | keyboard, out of the buffer `INT 09h` fills |
//! | `INT 19h` | the bootstrap loader |
//! | `INT 1Ah` | the tick count, the real-time clock, and the PCI BIOS interface (`AH=B1h`) |
//! | `INT 08h`/`09h` | the timer and keyboard interrupt service routines |
//!
//! It also answers the **PCI BIOS interface** (*PCI BIOS Specification* 2.1)
//! over configuration mechanism #1, so a driver or an option ROM can find a
//! function by vendor and device or by class code and read and write its
//! configuration space without knowing how a configuration cycle is generated
//! here. `src/fw/pcbios/pci.rs` is that module, and it also argues which of
//! the specification's functions are refused. POST *probes* for the mechanism
//! rather than assuming it, so a board with no host bridge has no PCI BIOS and
//! says so — `tests/pc_at_pci_bios.rs` runs both cases.
//!
//! It also **publishes the tables an operating system enumerates the board
//! through**, as data in the ROM at [`TABLES_OFFSET`]: an MP 1.4 floating
//! pointer and configuration table, an ACPI RSDP with an RSDT, XSDT, FADT,
//! MADT and DSDT, and an SMBIOS structure table. Every one of them is
//! generated from the *machine description* — see [`platform`] for what is read
//! out of it and [`tables`] for the byte layout — so a board with a second
//! processor in it publishes a table with two, which is the whole point:
//! `tests/kvm_pc_at_smp.rs` could start a second processor before this existed
//! and no operating system could have found it.
//!
//! # What it does not do
//!
//! Named rather than discovered later:
//!
//! * **No `B106h`, `B10Eh` or `B10Fh` in the PCI BIOS interface**, and no
//!   32-bit BIOS32 service directory. `src/fw/pcbios/pci.rs` argues each
//!   refusal: a special cycle would be a configuration write nothing on the
//!   fabric sees, an interrupt routing table would describe a south bridge
//!   this board does not have, and a 32-bit entry point is not something a
//!   16-bit assembler can emit.
//! * **No FACS, no MCFG, no HPET table, and a nearly empty DSDT.** Each is a
//!   claim about hardware this board does not have; [`tables`] says which and
//!   why.
//! * **`INT 15h AH=89h`, switch to protected mode, returns carry.** `AH=87h`
//!   is here; handing the machine over permanently is not, and no guest needs
//!   it — every one of them sets protected mode up itself.
//! * **No `INT 10h AH=11h`**, the character-generator group. `AL=30h` answers
//!   with a *pointer to the font being displayed*, and on this board that font
//!   is `pc.video`'s: the adapter draws the glyphs, and there is no font in
//!   this ROM at all. Putting a copy of one here would answer the question
//!   with a different font from the one on the screen, which is a worse answer
//!   than carry — and it would put a font's provenance inside the firmware
//!   (`ROADMAP.md` §1), where `pc.video`'s original one is already argued.
//!   FreeDOS calls it once while booting and does not mind.
//! * **`INT 10h AH=06h` scrolls the whole screen** rather than the requested
//!   rectangle when the line count is non-zero; the rectangle *is* honoured for
//!   a clear (`AL=0`), which is the case programs actually use it for.
//! * **Text mode only**, because `pc.video` is a text-mode CRTC. Setting a
//!   graphics mode records the number and changes nothing.
//! * **No serial, parallel, or PS/2 mouse services** (`INT 14h`, `INT 17h`,
//!   `INT 15h AH=C2h`): the board has none of those devices.
//! * **The keyboard is US-layout and set 1**, decoded from the translated codes
//!   the 8042 produces. Extended (`E0`-prefixed) keys are dropped rather than
//!   half-decoded.

use alloc::vec::Vec;

use crate::fw::asm16::{AX, Asm, DS, Label, Mem, R16, SP, Sreg};

mod disk;
mod keyboard;
mod pci;
pub mod platform;
mod post;
mod system;
pub mod tables;
mod video;

pub use platform::Platform;
pub use tables::Tables;

#[cfg(test)]
mod tests;

// ---------------------------------------------------------------------------
// where everything lives
// ---------------------------------------------------------------------------

/// The segment the image is decoded at. `pc.rom` top-aligns a system BIOS, so a
/// 64 KiB image in the board's 128 KiB socket lands at `0xf0000` — and at
/// `0xffff0000` through the high alias the reset vector fetches from.
pub const SEGMENT: u16 = 0xf000;

/// How big the image is. One segment: a real-mode BIOS cannot address more of
/// itself without a far jump, and 64 KiB is the socket `machines/pc-at.machine`
/// declares.
pub const SIZE: usize = 0x1_0000;

/// The segment of the Extended BIOS Data Area, the last kilobyte of base
/// memory. The BIOS reports 639 KiB at `0040:0013` and keeps its own tables
/// here, which is what the EBDA is for and why every PC since 1987 has one.
const EBDA_SEGMENT: u16 = 0x9fc0;

/// The BIOS Data Area's segment. Every `[offset]` in a handler below is
/// relative to this, because the handlers load `DS` with it.
const BDA_SEGMENT: u16 = 0x0040;

/// Where the POST stack lives — immediately below the boot sector's landing
/// pad, so the two never overlap.
const POST_STACK: u16 = 0x7c00;

/// Where the MP, ACPI and SMBIOS tables are laid out inside the image.
///
/// Halfway up the socket: the code below occupies the first few kilobytes and
/// asserts that it has not reached here, and the tables have the rest of the
/// segment. The physical address that follows from it is `0xf8000`, which is
/// inside every search window the three specifications define ([`tables`]) —
/// being *inside the BIOS ROM* is the property that matters, not the
/// particular offset.
pub const TABLES_OFFSET: u16 = 0x8000;

/// Where `IDENTIFY DEVICE` data is staged during POST. Below the stack and
/// below the boot sector, and dead by the time either matters.
const IDENTIFY_BUFFER: u16 = 0x0600;

// -- BIOS Data Area offsets, relative to segment 0x0040 ----------------------
//
// RBIL's `MEMORY.LST` and the OSDev wiki's "BDA" page. These are an ABI: a
// program that reads `0040:0013` for the memory size is reading this table.

/// The EBDA segment (word).
const BDA_EBDA: u16 = 0x0e;
/// The equipment word.
const BDA_EQUIPMENT: u16 = 0x10;
/// Base memory in kilobytes (word), which `INT 12h` returns.
const BDA_MEMSIZE: u16 = 0x13;
/// Keyboard shift flags: shift, control, alt, and the four lock states.
const BDA_KBFLAG: u16 = 0x17;
/// The keyboard buffer's head offset (word), within this same segment.
const BDA_KBHEAD: u16 = 0x1a;
/// The keyboard buffer's tail offset (word).
const BDA_KBTAIL: u16 = 0x1c;
/// The keyboard buffer itself: sixteen words of scan code and character.
const BDA_KBBUF: u16 = 0x1e;
/// Which floppy motors are running, and which drive was selected last.
const BDA_MOTOR: u16 = 0x3f;
/// Ticks until the floppy motors are switched off.
const BDA_MOTOR_TIMEOUT: u16 = 0x40;
/// The current video mode.
const BDA_VIDEO_MODE: u16 = 0x49;
/// Text columns (word).
const BDA_COLUMNS: u16 = 0x4a;
/// The size of a video page in bytes (word).
const BDA_PAGE_SIZE: u16 = 0x4c;
/// The active page's offset into video memory (word).
const BDA_PAGE_OFFSET: u16 = 0x4e;
/// Eight cursor positions, one per page: column in the low byte, row in the
/// high one.
const BDA_CURSOR: u16 = 0x50;
/// The cursor's start and end raster lines (word).
const BDA_CURSOR_SHAPE: u16 = 0x60;
/// The active display page.
const BDA_ACTIVE_PAGE: u16 = 0x62;
/// The CRTC's index port (word): `0x3d4` on a colour adapter.
const BDA_CRTC_PORT: u16 = 0x63;
/// The last value written to the CGA mode-select register.
const BDA_MODE_SELECT: u16 = 0x65;
/// The 18.2 Hz tick count (dword).
const BDA_TICKS: u16 = 0x6c;
/// Set when the tick count wrapped past midnight.
const BDA_ROLLOVER: u16 = 0x70;
/// How many fixed disks POST found.
const BDA_HD_COUNT: u16 = 0x75;
/// The keyboard buffer's start offset (word).
const BDA_KBBUF_START: u16 = 0x80;
/// The keyboard buffer's end offset (word), one past the last byte.
const BDA_KBBUF_END: u16 = 0x82;
/// Text rows minus one.
const BDA_ROWS: u16 = 0x84;
/// Character-cell height in scan lines (word).
const BDA_CHAR_HEIGHT: u16 = 0x85;

/// How many bytes of keyboard buffer the BDA holds: sixteen two-byte entries.
const KBBUF_BYTES: u16 = 32;

// -- our own tables, in the EBDA --------------------------------------------

/// The EBDA's size in kilobytes, which is what its first byte holds.
const EBDA_SIZE: u16 = 0x00;
/// Fixed disk 0: logical cylinders (word).
const EBDA_HD_CYLINDERS: u16 = 0x02;
/// Fixed disk 0: logical heads (word).
const EBDA_HD_HEADS: u16 = 0x04;
/// Fixed disk 0: logical sectors per track (word).
const EBDA_HD_SECTORS: u16 = 0x06;
/// Fixed disk 0: bit 0 set if present, bit 1 set if it supports LBA.
const EBDA_HD_FLAGS: u16 = 0x08;
/// Fixed disk 0: the LBA28 capacity from `IDENTIFY` words 60-61 (dword).
const EBDA_HD_CAPACITY: u16 = 0x0a;
/// Scratch: the LBA a transfer is aimed at, low word.
const EBDA_LBA_LOW: u16 = 0x10;
/// Scratch: the LBA a transfer is aimed at, high word.
const EBDA_LBA_HIGH: u16 = 0x12;
/// Scratch: the ATA command byte a transfer is in the middle of, so the
/// per-sector loop can tell a read from a write without holding a register.
const EBDA_COMMAND: u16 = 0x16;
/// Diskette scratch: sectors still to transfer (word).
const EBDA_FD_COUNT: u16 = 0x18;
/// Diskette scratch: the sector number the next command asks for.
const EBDA_FD_SECTOR: u16 = 0x1a;
/// Diskette scratch: the cylinder the head is seeking to.
const EBDA_FD_CYLINDER: u16 = 0x1b;
/// Diskette scratch: the head.
const EBDA_FD_HEAD: u16 = 0x1c;
/// Diskette scratch: how many sectors have been transferred so far, which is
/// what `AL` reports back whether the transfer finished or stopped short.
const EBDA_FD_DONE: u16 = 0x1d;
/// Diskette scratch: sectors per track, from the CMOS drive type.
const EBDA_FD_SPT: u16 = 0x1e;
/// Diskette scratch: the µPD765 command a transfer is running — `READ DATA` or
/// `WRITE DATA`. Two things have to agree about a transfer's direction, the
/// chip's opcode and the 8237's mode register, and both are derived from this
/// one byte so that they cannot disagree.
const EBDA_FD_CMD: u16 = 0x1f;
/// How many `E820` entries [`EBDA_E820`] holds (word).
const EBDA_E820_COUNT: u16 = 0x14;
/// The `E820` memory map, built at POST: twenty bytes per entry.
const EBDA_E820: u16 = 0x20;
/// The seven result bytes of the last µPD765 command — `ST0`, `ST1`, `ST2`,
/// `C`, `H`, `R`, `N`. Immediately past the `E820` table.
const EBDA_FD_RESULT: u16 = EBDA_E820 + E820_ENTRY * 4;

/// The six-byte pseudo-descriptor `INT 15h AH=87h` hands to `LGDT`: a 16-bit
/// limit and a 32-bit base, built from the caller's `ES:SI`. It lives here
/// rather than on the stack because `LGDT` takes a memory operand and the
/// handler's stack frame is what `[bp+n]` names.
const EBDA_GDTR: u16 = EBDA_FD_RESULT + 7;

// -- the PCI BIOS interface's own scratch ------------------------------------
//
// POST writes [`EBDA_PCI_MECHANISM`]; the rest belong to a single `INT 1Ah
// AH=B1h` call and are dead between calls. They live here rather than on the
// stack because `pci`'s scan loop holds its cursor in `BX` and its accumulator
// in `EAX`, and a 16-bit `PUSH` cannot save the latter.

/// Which PCI configuration access mechanism POST found, as the byte
/// `AH=B101h` answers with: `0x01` for mechanism #1, zero for none.
const EBDA_PCI_MECHANISM: u16 = EBDA_GDTR + 6;
/// What the scan compares each function against: a vendor/device pair, or a
/// class code in the low three bytes (dword).
const EBDA_PCI_MATCH: u16 = EBDA_PCI_MECHANISM + 2;
/// How many further matches the caller's index still asks to skip (word).
const EBDA_PCI_INDEX: u16 = EBDA_PCI_MATCH + 4;
/// The vendor/device dword, held across the header-type read (dword).
const EBDA_PCI_SAVE: u16 = EBDA_PCI_INDEX + 2;
/// How far the scan advances after this function: one function, or eight to
/// skip the rest of a single-function device.
const EBDA_PCI_STEP: u16 = EBDA_PCI_SAVE + 4;
/// Which comparison the scan is making: zero for vendor and device, one for a
/// class code.
const EBDA_PCI_MODE: u16 = EBDA_PCI_STEP + 1;

/// How many bytes the next 8237 channel-2 transfer moves, less one — 511 for a
/// sector, four per sector less one for a `FORMAT TRACK` address-field list.
///
/// Not beside the other diskette fields because that block runs up against the
/// `E820` table at [`EBDA_E820`]; it is the diskette's for all that it lives
/// here.
const EBDA_FD_LENGTH: u16 = EBDA_PCI_MODE + 2;

/// How many bytes one `E820` entry occupies: base, length, type.
const E820_ENTRY: u16 = 20;

// -- the interrupt frame -----------------------------------------------------
//
// Every service handler below opens with `push ds; push es; pusha; mov bp,sp`,
// so `[bp+n]` names the caller's registers and its `IRET` frame. A handler
// returns a value by writing the *saved* register, and sets carry by writing
// the *saved* flags — the only way to affect a caller that is resumed by
// `IRET`.

/// The saved `SI`.
const F_SI: i32 = 2;
/// The saved `BP`.
const F_BP: i32 = 4;
/// The saved `BX`.
const F_BX: i32 = 8;
/// The saved `DX`.
const F_DX: i32 = 10;
/// The saved `CX`.
const F_CX: i32 = 12;
/// The saved `AX`.
const F_AX: i32 = 14;
/// The saved `ES`.
const F_ES: i32 = 16;
/// The saved `DS`.
const F_DS: i32 = 18;
/// The caller's `FLAGS`, as `IRET` will restore them.
const F_FLAGS: i32 = 24;

// The `PUSHAD` frame. `INT 15h` and the PCI BIOS interface both open with
// `push ds; push es; pushad`, because both are specified in terms of 32-bit
// registers and a 16-bit `PUSHA` would drop the halves the caller cares about.
// Four bytes per register, so every offset differs from the `PUSHA` frame
// above.

/// The saved `EDI` — where a configuration register number arrives.
const G_EDI: i32 = 0;
/// The saved `ESI` — `AH=87h`'s descriptor table, and a find function's index.
const G_ESI: i32 = 4;
/// The saved `EBX` — `E820`'s continuation index, and a bus/device pair.
const G_EBX: i32 = 16;
/// The saved `EDX` — where `'SMAP'` arrives, and where `'PCI '` goes back.
const G_EDX: i32 = 20;
/// The saved `ECX` — a buffer size, or a configuration register's value.
const G_ECX: i32 = 24;
/// The saved `EAX` — the function number, and the return code in `AH`.
const G_EAX: i32 = 28;
/// The caller's `FLAGS` in the `PUSHAD` frame.
const G_FLAGS: i32 = 40;

/// The carry flag's bit in `FLAGS` (SDM Vol. 1 §3.4.3).
const FLAG_CF: u16 = 0x0001;
/// The zero flag's bit, which `INT 16h AH=01h` answers with.
const FLAG_ZF: u16 = 0x0040;

// ---------------------------------------------------------------------------
// labels
// ---------------------------------------------------------------------------

/// Every label the firmware's parts share.
///
/// Created up front because the code is emitted in one pass and almost every
/// reference is forward: the interrupt vector table is built by POST out of
/// handlers assembled after it.
pub(crate) struct Labels {
    // entry points
    pub post: Label,
    pub set_vector: Label,
    pub unimplemented: Label,
    pub iret_stub: Label,
    pub eoi_master: Label,
    pub eoi_slave: Label,

    // interrupt handlers
    pub int08: Label,
    pub int09: Label,
    pub int10: Label,
    pub int11: Label,
    pub int12: Label,
    pub int13: Label,
    pub int15: Label,
    pub int16: Label,
    pub int18: Label,
    pub int19: Label,
    pub int1a: Label,
    pub int1a_pci: Label,

    // video primitives
    pub putc: Label,
    pub puts: Label,
    pub put_dec: Label,
    pub cell_offset: Label,
    pub scroll_up: Label,
    pub set_cursor_hw: Label,
    pub clear_screen: Label,

    // keyboard primitives
    pub kb_enqueue: Label,
    pub kb_scan_plain: Label,
    pub kb_scan_shift: Label,

    // disk primitives
    pub ata_read: Label,
    pub ata_wait_drq: Label,
    pub ata_wait_ready: Label,
    pub chs_to_lba: Label,
    pub disk_ok: Label,
    pub disk_fail: Label,

    // diskette primitives
    pub fd_out: Label,
    pub fd_in: Label,
    pub fd_drain: Label,
    pub fd_start: Label,
    pub fd_seek: Label,
    pub fd_dma: Label,
    pub fd_xfer_one: Label,
    pub fd_results: Label,
    pub fd_geometry: Label,

    // the PCI BIOS interface
    pub pci_detect: Label,
    pub pci_select: Label,
    pub pci_read_dword: Label,
    pub pci_scan: Label,

    // POST helpers
    pub cmos_read: Label,
    pub kbc_wait_write: Label,
    pub kbc_wait_read: Label,
    pub e820_template: Label,
}

impl Labels {
    fn new(a: &mut Asm) -> Labels {
        Labels {
            post: a.label(),
            set_vector: a.label(),
            unimplemented: a.label(),
            iret_stub: a.label(),
            eoi_master: a.label(),
            eoi_slave: a.label(),
            int08: a.label(),
            int09: a.label(),
            int10: a.label(),
            int11: a.label(),
            int12: a.label(),
            int13: a.label(),
            int15: a.label(),
            int16: a.label(),
            int18: a.label(),
            int19: a.label(),
            int1a: a.label(),
            int1a_pci: a.label(),
            putc: a.label(),
            puts: a.label(),
            put_dec: a.label(),
            cell_offset: a.label(),
            scroll_up: a.label(),
            set_cursor_hw: a.label(),
            clear_screen: a.label(),
            kb_enqueue: a.label(),
            kb_scan_plain: a.label(),
            kb_scan_shift: a.label(),
            ata_read: a.label(),
            ata_wait_drq: a.label(),
            ata_wait_ready: a.label(),
            chs_to_lba: a.label(),
            disk_ok: a.label(),
            disk_fail: a.label(),
            fd_out: a.label(),
            fd_in: a.label(),
            fd_drain: a.label(),
            fd_start: a.label(),
            fd_seek: a.label(),
            fd_dma: a.label(),
            fd_xfer_one: a.label(),
            fd_results: a.label(),
            fd_geometry: a.label(),
            pci_detect: a.label(),
            pci_select: a.label(),
            pci_read_dword: a.label(),
            pci_scan: a.label(),
            cmos_read: a.label(),
            kbc_wait_write: a.label(),
            kbc_wait_read: a.label(),
            e820_template: a.label(),
        }
    }
}

// ---------------------------------------------------------------------------
// shared emission helpers
// ---------------------------------------------------------------------------

/// A service handler's prologue: interrupts back on, and a frame `[bp+n]`
/// addresses the caller's registers through.
pub(crate) fn enter(a: &mut Asm) {
    a.sti();
    a.pushs(DS);
    a.pushs(crate::fw::asm16::ES);
    a.pusha();
    a.mov(crate::fw::asm16::BP, SP);
}

/// A service handler's epilogue.
pub(crate) fn leave(a: &mut Asm) {
    a.popa();
    a.pops(crate::fw::asm16::ES);
    a.pops(DS);
    a.iret();
}

/// Set the carry flag the caller will be resumed with.
pub(crate) fn set_cf(a: &mut Asm) {
    a.alui(crate::fw::asm16::Alu::OR, Mem::bp(F_FLAGS), FLAG_CF);
}

/// Clear the carry flag the caller will be resumed with.
pub(crate) fn clear_cf(a: &mut Asm) {
    a.alui(crate::fw::asm16::Alu::AND, Mem::bp(F_FLAGS), !FLAG_CF);
}

/// `mov <sreg>, imm16`, which the instruction set has no encoding for: a
/// segment register can only be loaded from a register or from memory.
pub(crate) fn load_seg(a: &mut Asm, seg: Sreg, value: u16, scratch: R16) {
    a.movi(scratch, value);
    a.movsr(seg, scratch);
}

/// Point `DS` at the BIOS Data Area, clobbering `AX`.
pub(crate) fn ds_bda(a: &mut Asm) {
    load_seg(a, DS, BDA_SEGMENT, AX);
}

/// Point `DS` at the Extended BIOS Data Area, clobbering `AX`.
pub(crate) fn ds_ebda(a: &mut Asm) {
    load_seg(a, DS, EBDA_SEGMENT, AX);
}

// ---------------------------------------------------------------------------
// the image
// ---------------------------------------------------------------------------

/// The date the ROM reports at `F000:FFF5`, in the `mm/dd/yy` form software
/// has parsed since 1981.
///
/// A constant, not the build date: the image must be byte-identical on every
/// host and in every year, or a machine's state hash depends on when it was
/// compiled (`CLAUDE.md`, determinism).
const BIOS_DATE: &[u8; 8] = b"01/01/26";

/// The model byte at `F000:FFFE`. `0xFC` is the PC/AT, which is the board this
/// firmware is written for.
const MODEL_BYTE: u8 = 0xfc;

/// Where the reset vector's far jump sits: `F000:FFF0`, which an 80486 reaches
/// as `0xfffffff0` through the high alias and as `0xffff0` after its first far
/// jump.
const RESET_VECTOR: u16 = 0xfff0;

/// Assemble the ROM for `machines/pc-at.machine` as it ships.
///
/// Deterministic: the same source produces the same 65,536 bytes on every host,
/// and this module's own tests assert it.
///
/// The tables it publishes describe [`Platform::at`] — one processor. A board
/// with two needs [`image_for`] or [`image_for_machine`], because a table that
/// says "one processor" on a two-processor board is how an operating system
/// comes to run on half a machine.
///
/// # Panics
///
/// If the firmware source referenced a label it never bound, or overflowed the
/// socket. Both are bugs in this module rather than anything a caller can
/// cause, and both would otherwise ship as a ROM that does not boot.
#[must_use]
pub fn image() -> Vec<u8> {
    image_for(&Platform::at())
}

/// Assemble the ROM for a board this firmware is about to be put in.
///
/// The code is the same in every image; what changes is the tables
/// [`tables::generate`] lays into it, which is why this takes a [`Platform`]
/// rather than a processor count: an operating system needs the APIC IDs, the
/// I/O APIC and the interrupt assignments too, and all of them come from the
/// same description.
///
/// # Panics
///
/// As [`image`].
#[must_use]
pub fn image_for(platform: &Platform) -> Vec<u8> {
    // 0xff, because that is what an erased EPROM holds and what this board's
    // `unassigned = read-as-ones` answers with: a jump into a gap then behaves
    // the same way whether the gap is inside the chip or outside it.
    let mut a = Asm::new(SIZE, 0xff);
    let l = Labels::new(&mut a);

    post::emit(&mut a, &l);
    video::emit(&mut a, &l);
    keyboard::emit(&mut a, &l);
    disk::emit(&mut a, &l);
    system::emit(&mut a, &l);
    pci::emit(&mut a, &l);

    // The tables, as data. Nothing branches into them and POST does not copy
    // them: an operating system finds them by searching this segment, which is
    // where all three specifications say to look.
    assert!(
        a.here() <= TABLES_OFFSET,
        "the firmware's code reached {:#06x} and would overwrite its own tables",
        a.here()
    );
    let base = (u32::from(SEGMENT) << 4) + u32::from(TABLES_OFFSET);
    let tables = tables::generate(base, platform);
    assert!(
        tables.bytes.len() <= usize::from(RESET_VECTOR - TABLES_OFFSET),
        "the tables do not fit in the socket: {} bytes",
        tables.bytes.len()
    );
    a.seek(TABLES_OFFSET);
    a.db(&tables.bytes);

    // The reset vector and the identification bytes an AT puts in the last
    // sixteen. RBIL's `MEMORY.LST` and the AT's published memory map: the date
    // is eight ASCII characters at F000:FFF5 and the model byte is at
    // F000:FFFE.
    a.seek(RESET_VECTOR);
    a.jmpf_label(SEGMENT, l.post);
    a.seek(0xfff5);
    a.db(BIOS_DATE);
    a.db(&[0x00, MODEL_BYTE]);

    let mut bytes = a.finish();

    // The 8-bit sum of the whole image is zero, which is the convention every
    // PC ROM follows and the only thing the last byte is for.
    let sum = bytes[..SIZE - 1]
        .iter()
        .fold(0u8, |acc, &b| acc.wrapping_add(b));
    bytes[SIZE - 1] = 0u8.wrapping_sub(sum);
    bytes
}

/// Assemble the ROM for the board `text` describes.
///
/// The form a caller who has a `.machine` file in hand wants: `rsemu run
/// pc-at` takes this route, so a user who adds a second processor to their copy
/// of the board gets firmware that says so.
///
/// The text is resolved with the description's *own* parameter defaults, which
/// is the only thing a media slot can do — the image has to exist before
/// [`crate::machine::build`] is called, so there is no resolved machine to take
/// overrides from. A board whose processor count came from a `param` a caller
/// overrides should resolve it itself and go through
/// [`Platform::from_resolved`], which is public for exactly that.
///
/// # Errors
///
/// [`crate::Error::Config`] if `text` is not a machine description, or
/// describes a board with no processor or no local APIC — see
/// [`Platform::from_machine`].
pub fn image_for_machine(file: &str, text: &str) -> crate::Result<Vec<u8>> {
    Ok(image_for(&Platform::from_machine(file, text)?))
}
