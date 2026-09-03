//! **A guest finds a PCI device through rsemu's own BIOS, and checks the
//! answer against the hardware.**
//!
//! [`rsemu::fw::pcbios`] implements the PCI BIOS interface — `INT 1Ah AH=B1h`
//! of the *PCI BIOS Specification* 2.1 — over configuration mechanism #1.
//! `src/fw/pcbios/pci.rs` says what each function does; this is the evidence
//! that it does it, from the only place that can produce evidence: a program
//! running on the board.
//!
//! The guest is a boot sector assembled with [`rsemu::fw::asm16`], the same
//! assembler the firmware is written with, and it does what a DOS-era driver
//! does:
//!
//! 1. `AX=B101h`, the installation check, and keeps `EDX`, `AL`, `BX`, `CL`
//!    and the flags the service returned;
//! 2. `AX=B102h` for the vendor and device identification
//!    `machines/pc-at.machine` gives its display adapter, and `AX=B103h` for
//!    the class code the same card hardwires — two different searches that
//!    have to name the same function;
//! 3. `AX=B10Ah`, `B109h` and `B108h` to read that function's configuration
//!    space as a Dword, a word and a byte;
//! 4. the same registers **directly**, through `0xcf8`/`0xcfc`, with no BIOS
//!    involved — which is the check that the service is reporting the bus
//!    rather than reciting a constant;
//! 5. `B10Bh` to *write* a register the host bridge declares writable, read
//!    back through the ports, and put it back;
//! 6. four calls that must fail, each with its own return code: a search for a
//!    device that is not there, a search with `FFFFh` as the vendor, a word
//!    read of an odd register, and `B106h`, which this firmware refuses.
//!
//! # The negative control
//!
//! The same firmware on the same board with the `0xcf8` window unmapped. POST
//! probes for the configuration mechanism rather than being told about it, so
//! a board with no window has no PCI BIOS — and every function, the
//! installation check included, comes back with carry set. A firmware that
//! answered `B101h` from a constant would pass every test above and fail this
//! one.
//!
//! No accelerator: the guest talks to a host bridge and a display adapter's
//! configuration space and never leaves real mode, so the interpreter runs it
//! everywhere.

#![cfg(all(
    feature = "cpu-x86",
    feature = "dev-pc",
    feature = "dev-pc-apic",
    feature = "dev-pc-video",
    feature = "dev-pc-floppy",
    feature = "dev-pc-ide",
    feature = "dev-pc-hpet",
    feature = "fw-pcbios",
    feature = "machine-pc-at"
))]

use rsemu::core::clock::GlobalTime;
use rsemu::core::device::ResetKind;
use rsemu::core::space::MemAttrs;
use rsemu::core::value::Width;
use rsemu::fw::asm16::{AX, Asm, BX, CX, DI, DS, DX, Mem, SI, SP, SS};
use rsemu::machine::{Machine, build};

// ---------------------------------------------------------------------------
// the board's own numbers
// ---------------------------------------------------------------------------

/// The vendor identification `machines/pc-at.machine` gives `vgacard`, which is
/// `pc.vga-pci`'s default.
const VGA_VENDOR: u16 = 0x1234;
/// Its device identification.
const VGA_DEVICE: u16 = 0x1111;
/// Where the machine file puts it: `device = 2`, function 0. The PCI BIOS
/// packs that into one byte as device in bits 7-3 and function in bits 2-0
/// (*PCI BIOS* §4.3), so `BL` comes back `0x10`.
const VGA_DEVFN: u8 = 2 << 3;
/// Base class 03 (display), sub-class 00 (VGA-compatible), programming
/// interface 00 — what `pc.vga-pci` hardwires at configuration offset 09h.
const VGA_CLASS: u32 = 0x0003_0000;

/// The host bridge is device 0, function 0 — an 82441FX, which answers only on
/// function 0.
const PMC_DEVFN: u8 = 0;
/// Configuration register 0Dh, the Master Latency Timer: read/write on this
/// part and connected to nothing, which is what makes it the register to prove
/// a configuration *write* with (Intel 82441FX datasheet §3.2.8).
const REG_LATENCY: u16 = 0x0d;
/// The Dword that register lives in.
const REG_LATENCY_DWORD: u8 = 0x0c;
/// What the guest writes there.
const LATENCY_PROBE: u16 = 0x28;

/// A vendor and device pair no function on this board has.
const ABSENT_VENDOR: u16 = 0xdead;
/// Its device half.
const ABSENT_DEVICE: u16 = 0xbeef;

// ---------------------------------------------------------------------------
// where the guest leaves what it found
// ---------------------------------------------------------------------------

/// Where the boot sector lands.
const BOOT: u16 = 0x7c00;
/// The block at `0x0500` every PC has left free since 1981.
const SCRATCH: u16 = 0x0500;

/// It ran at all.
const OFF_STARTED: u16 = SCRATCH;
/// It finished.
const OFF_DONE: u16 = SCRATCH + 2;
/// `EDX` from the installation check: the `'PCI '` signature.
const OFF_SIGNATURE: u16 = SCRATCH + 4;
/// A Dword read of the display adapter's register 00h, through the ports.
const OFF_DIRECT_ID: u16 = SCRATCH + 8;
/// A Dword read of the host bridge's register 0Ch after the BIOS wrote it.
const OFF_DIRECT_LATENCY: u16 = SCRATCH + 12;

/// The first of the fixed-size blocks each service call's answer goes into.
const RECORDS: u16 = SCRATCH + 0x10;
/// How big one is: `AX`, `BX`, `ECX`, `FLAGS`.
const RECORD: u16 = 10;

/// Which block each call uses.
const R_PRESENT: u16 = 0;
/// `B102h` for the display adapter.
const R_FIND_ID: u16 = 1;
/// `B103h` for the same card's class code.
const R_FIND_CLASS: u16 = 2;
/// `B10Ah`, register 00h.
const R_READ_DWORD: u16 = 3;
/// `B109h`, register 0Ah.
const R_READ_WORD: u16 = 4;
/// `B108h`, register 0Bh.
const R_READ_BYTE: u16 = 5;
/// `B108h` of the host bridge's latency timer, before it is written.
const R_LATENCY: u16 = 6;
/// `B102h` for a device that is not there.
const R_MISS: u16 = 7;
/// `B102h` with `FFFFh` as the vendor.
const R_BAD_VENDOR: u16 = 8;
/// `B109h` at an odd register number.
const R_BAD_REGISTER: u16 = 9;
/// `B106h`, generate special cycle.
const R_SPECIAL: u16 = 10;

/// Where block `n` starts.
fn record_at(n: u16) -> u16 {
    RECORDS + n * RECORD
}

/// What [`OFF_STARTED`] holds.
const STARTED: u16 = 0xb101;
/// What [`OFF_DONE`] holds.
const DONE: u16 = 0x600d;

// ---------------------------------------------------------------------------
// the guest
// ---------------------------------------------------------------------------

/// Save `AX`, `BX`, `ECX` and the flags the last `INT 1Ah` returned.
///
/// `MOV` does not touch the flags, so the `PUSHF` at the end still sees the
/// carry the service set — which is the return this interface is really about.
fn record(a: &mut Asm, n: u16) {
    let at = record_at(n);
    a.movto(Mem::abs(at), AX);
    a.movto(Mem::abs(at + 2), BX);
    a.movto32(Mem::abs(at + 4), CX);
    a.pushf();
    a.pop(AX);
    a.movto(Mem::abs(at + 8), AX);
}

/// Save only `AX` and the flags — enough for a call that is expected to fail.
fn record_status(a: &mut Asm, n: u16) {
    let at = record_at(n);
    a.movto(Mem::abs(at), AX);
    a.pushf();
    a.pop(AX);
    a.movto(Mem::abs(at + 8), AX);
}

/// A configuration Dword read done by the guest itself: `CONFIG_ADDRESS` at
/// `0xcf8`, then `CONFIG_DATA` at `0xcfc` (*PCI Local Bus* §3.7.4.1). Bus 0,
/// because that is the only bus this board has. Answers in `EAX`.
fn direct_read(a: &mut Asm, devfn: u8, register: u8) {
    a.movi32(
        AX,
        0x8000_0000 | (u32::from(devfn) << 8) | u32::from(register & 0xfc),
    );
    a.movi(DX, 0x0cf8);
    a.out_dx_eax();
    a.movi(DX, 0x0cfc);
    a.in_eax_dx();
}

/// Assemble the boot sector.
#[allow(clippy::too_many_lines)]
fn boot_sector() -> Vec<u8> {
    let mut a = Asm::new(usize::from(BOOT) + 512, 0x00);
    a.seek(BOOT);

    a.cli();
    a.movi(AX, 0);
    a.movsr(DS, AX);
    a.movsr(SS, AX);
    a.movi(SP, BOOT);
    a.sti();
    a.movmi(Mem::abs(OFF_STARTED), STARTED);

    // -- B101h, the installation check --------------------------------------
    a.movi(AX, 0xb101);
    a.int(0x1a);
    a.movto32(Mem::abs(OFF_SIGNATURE), DX);
    record(&mut a, R_PRESENT);

    // -- B102h, find by vendor and device -----------------------------------
    a.movi(AX, 0xb102);
    a.movi(CX, VGA_DEVICE);
    a.movi(DX, VGA_VENDOR);
    a.movi(SI, 0);
    a.int(0x1a);
    record(&mut a, R_FIND_ID);

    // -- B103h, find by class code ------------------------------------------
    a.movi(AX, 0xb103);
    a.movi32(CX, VGA_CLASS);
    a.movi(SI, 0);
    a.int(0x1a);
    record(&mut a, R_FIND_CLASS);

    // -- the three read widths, at what B102h said --------------------------
    //
    // `BX` comes out of the block `B102h`'s answer went into rather than being
    // written down here: if the service named the wrong function, these read
    // the wrong function's registers and the comparison below fails.
    for (function, register, block) in [
        (0xb10au16, 0x00u16, R_READ_DWORD),
        (0xb109, 0x0a, R_READ_WORD),
        (0xb108, 0x0b, R_READ_BYTE),
    ] {
        a.movi(AX, function);
        a.mov(BX, Mem::abs(record_at(R_FIND_ID) + 2));
        a.movi(DI, register);
        a.int(0x1a);
        record(&mut a, block);
    }

    // -- the same registers, straight off the bus ---------------------------
    direct_read(&mut a, VGA_DEVFN, 0x00);
    a.movto32(Mem::abs(OFF_DIRECT_ID), AX);

    // -- B108h then B10Bh: read a writable register, write it, look ---------
    a.movi(AX, 0xb108);
    a.movi(BX, u16::from(PMC_DEVFN));
    a.movi(DI, REG_LATENCY);
    a.int(0x1a);
    record(&mut a, R_LATENCY);

    a.movi(AX, 0xb10b);
    a.movi(BX, u16::from(PMC_DEVFN));
    a.movi(DI, REG_LATENCY);
    a.movi(CX, LATENCY_PROBE);
    a.int(0x1a);

    direct_read(&mut a, PMC_DEVFN, REG_LATENCY_DWORD);
    a.movto32(Mem::abs(OFF_DIRECT_LATENCY), AX);

    // Put it back, out of the block the read went into.
    a.movi(AX, 0xb10b);
    a.movi(BX, u16::from(PMC_DEVFN));
    a.movi(DI, REG_LATENCY);
    a.mov(CX, Mem::abs(record_at(R_LATENCY) + 4));
    a.int(0x1a);

    // -- the four that must fail --------------------------------------------
    a.movi(AX, 0xb102);
    a.movi(CX, ABSENT_DEVICE);
    a.movi(DX, ABSENT_VENDOR);
    a.movi(SI, 0);
    a.int(0x1a);
    record_status(&mut a, R_MISS);

    a.movi(AX, 0xb102);
    a.movi(CX, VGA_DEVICE);
    a.movi(DX, 0xffff);
    a.movi(SI, 0);
    a.int(0x1a);
    record_status(&mut a, R_BAD_VENDOR);

    a.movi(AX, 0xb109);
    a.movi(BX, u16::from(PMC_DEVFN));
    a.movi(DI, 0x0b);
    a.int(0x1a);
    record_status(&mut a, R_BAD_REGISTER);

    a.movi(AX, 0xb106);
    a.movi(BX, 0);
    a.int(0x1a);
    record_status(&mut a, R_SPECIAL);

    a.movmi(Mem::abs(OFF_DONE), DONE);
    let spin = a.here_label();
    a.hlt();
    a.jmp(spin);

    assert!(
        a.here() <= BOOT + 510,
        "the boot sector is {} bytes and 510 is all a sector has",
        a.here() - BOOT
    );
    a.seek(BOOT + 510);
    a.db(&[0x55, 0xaa]);

    let image = a.finish();
    image[usize::from(BOOT)..].to_vec()
}

/// A 1.44 MB diskette with that sector on it.
fn diskette() -> Vec<u8> {
    let mut image = boot_sector();
    assert_eq!(image.len(), 512, "a boot sector is one sector");
    image.resize(1_474_560, 0);
    image
}

// ---------------------------------------------------------------------------
// the board
// ---------------------------------------------------------------------------

/// Build `text` with rsemu's own BIOS in its socket and the prober on the
/// diskette.
fn board(name: &str, text: &str, bios: Vec<u8>) -> Machine {
    let mut options = rsemu::machine::catalog::build_options().expect("this build's classes");
    options.realize.media.insert("bios", bios);
    options.realize.media.insert("vgabios", Vec::new());
    options.realize.media.insert("floppy", diskette());
    for slot in ["disk", "hd0", "hd1", "cd0", "cd1"] {
        options.realize.media.insert(slot, Vec::new());
    }
    let registry = rsemu::machine::catalog::registry().expect("this build's registry");
    let mut m = build(name, text, &registry, &options)
        .unwrap_or_else(|e| panic!("{name} does not realize: {e}"));
    m.reset(ResetKind::Cold);
    m.sweep();
    m
}

/// A word of guest memory, read as a debugger reads.
fn peek16(m: &Machine, at: u16) -> u16 {
    m.space("mem")
        .expect("the memory space")
        .read(u64::from(at), Width::U16, MemAttrs::DEBUG)
        .unwrap_or(0) as u16
}

/// A dword of guest memory.
fn peek32(m: &Machine, at: u16) -> u32 {
    u32::from(peek16(m, at)) | (u32::from(peek16(m, at + 2)) << 16)
}

/// What one service call answered.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct Call {
    /// The whole of `AX`: the return code in `AH`, and whatever the function
    /// puts in `AL`.
    ax: u16,
    /// `BH` bus, `BL` device and function.
    bx: u16,
    /// The value a read function answers with, or the last bus number.
    ecx: u32,
    /// Whether the service set carry, which is how every one of these reports
    /// failure.
    carry: bool,
}

impl Call {
    /// The return code, which is `AH` (*PCI BIOS* §4).
    fn code(self) -> u8 {
        (self.ax >> 8) as u8
    }
}

/// Everything the guest brought back.
#[derive(Debug)]
struct Found {
    signature: u32,
    direct_id: u32,
    direct_latency: u32,
    calls: Vec<Call>,
}

fn run(name: &str, text: &str, bios: Vec<u8>) -> Found {
    let mut m = board(name, text, bios);
    for _ in 0..3000 {
        m.run_for(GlobalTime::from_nanos(1_000_000))
            .expect("the board runs");
        if peek16(&m, OFF_DONE) == DONE {
            break;
        }
    }
    assert_eq!(
        peek16(&m, OFF_STARTED),
        STARTED,
        "the boot sector never ran: `INT 19h` did not reach it"
    );
    assert_eq!(
        peek16(&m, OFF_DONE),
        DONE,
        "the guest did not finish calling the PCI BIOS"
    );
    Found {
        signature: peek32(&m, OFF_SIGNATURE),
        direct_id: peek32(&m, OFF_DIRECT_ID),
        direct_latency: peek32(&m, OFF_DIRECT_LATENCY),
        calls: (0..=R_SPECIAL)
            .map(|n| {
                let at = record_at(n);
                Call {
                    ax: peek16(&m, at),
                    bx: peek16(&m, at + 2),
                    ecx: peek32(&m, at + 4),
                    carry: peek16(&m, at + 8) & 1 != 0,
                }
            })
            .collect(),
    }
}

/// `machines/pc-at.machine` as it ships.
fn probe_stock() -> Found {
    run(
        "pc-at.machine",
        rsemu::dev::pc::PC_AT,
        rsemu::fw::pcbios::image(),
    )
}

// ---------------------------------------------------------------------------
// the tests
// ---------------------------------------------------------------------------

#[test]
fn the_installation_check_answers_what_the_specification_says_it_does() {
    let found = probe_stock();
    let present = found.calls[R_PRESENT as usize];
    assert!(!present.carry, "carry set: {present:?}");
    assert_eq!(present.code(), 0x00, "the return code is not SUCCESSFUL");
    // *PCI BIOS* §4.2: EDX is `'PCI '`, with `'P'` in DL.
    assert_eq!(found.signature, 0x2049_4350, "the signature in EDX");
    // AL is the hardware mechanism: bit 0 is configuration mechanism #1, and
    // the special-cycle bits are clear because there is no such path here.
    assert_eq!(present.ax & 0xff, 0x01, "the hardware mechanism byte");
    // BX is the interface level, 2.10.
    assert_eq!(present.bx, 0x0210, "the interface level version");
    // CL is the last bus number: this board has exactly bus 0.
    assert_eq!(present.ecx & 0xff, 0, "the last PCI bus number");
}

#[test]
fn two_different_searches_name_the_display_adapter_the_machine_file_declares() {
    let found = probe_stock();
    let by_id = found.calls[R_FIND_ID as usize];
    let by_class = found.calls[R_FIND_CLASS as usize];
    assert!(!by_id.carry, "B102h failed: {by_id:?}");
    assert!(!by_class.carry, "B103h failed: {by_class:?}");
    assert_eq!(by_id.code(), 0x00);
    assert_eq!(by_class.code(), 0x00);
    // `object vgacard "pc.vga-pci" { bus = "pci0", device = 2 }`, and nothing
    // in the firmware knows that number.
    assert_eq!(
        by_id.bx,
        u16::from(VGA_DEVFN),
        "B102h named bus {:#04x} device/function {:#04x}",
        by_id.bx >> 8,
        by_id.bx & 0xff
    );
    assert_eq!(
        by_class.bx, by_id.bx,
        "the vendor/device search and the class search named different functions"
    );
}

#[test]
fn what_the_service_reads_is_what_the_ports_read() {
    // The claim that matters: `B108h`, `B109h` and `B10Ah` are reporting the
    // bus, not reciting the machine file. The guest read the same registers
    // itself through 0xcf8/0xcfc with no BIOS involved.
    let found = probe_stock();
    let expected = u32::from(VGA_VENDOR) | (u32::from(VGA_DEVICE) << 16);
    assert_eq!(
        found.direct_id, expected,
        "the board's own configuration space is not what the machine file says"
    );

    let dword = found.calls[R_READ_DWORD as usize];
    assert!(!dword.carry, "B10Ah failed: {dword:?}");
    assert_eq!(
        dword.ecx, found.direct_id,
        "B10Ah disagrees with 0xcf8/0xcfc"
    );

    // Register 0Ah is the sub-class and base class as a word, and 0Bh is the
    // base class alone: both are the top half of the class-code Dword the
    // search matched, read at two narrower widths through the byte enables the
    // low bits of the I/O address stand for.
    let word = found.calls[R_READ_WORD as usize];
    assert!(!word.carry, "B109h failed: {word:?}");
    assert_eq!(word.ecx & 0xffff, u32::from((VGA_CLASS >> 8) as u16));

    let byte = found.calls[R_READ_BYTE as usize];
    assert!(!byte.carry, "B108h failed: {byte:?}");
    assert_eq!(byte.ecx & 0xff, VGA_CLASS >> 16);
}

#[test]
fn a_configuration_write_through_the_service_reaches_the_device() {
    let found = probe_stock();
    let before = found.calls[R_LATENCY as usize];
    assert!(!before.carry, "B108h failed: {before:?}");
    // The host bridge comes out of reset with a zero latency timer, and the
    // guest wrote 0x28 to it through `B10Bh`.
    assert_eq!(before.ecx & 0xff, 0, "the latency timer was not zero");
    assert_eq!(
        (found.direct_latency >> 8) & 0xff,
        u32::from(LATENCY_PROBE),
        "0xcf8/0xcfc does not see what B10Bh wrote: {:#010x}",
        found.direct_latency
    );
}

#[test]
fn the_four_failures_each_get_their_own_return_code() {
    let found = probe_stock();
    for (block, code, what) in [
        (R_MISS, 0x86u8, "a device that is not on the bus"),
        (R_BAD_VENDOR, 0x83, "FFFFh as a vendor identification"),
        (
            R_BAD_REGISTER,
            0x87,
            "an odd register number for a word read",
        ),
        (R_SPECIAL, 0x81, "B106h, which this firmware refuses"),
    ] {
        let call = found.calls[block as usize];
        assert!(call.carry, "{what}: carry clear, {call:?}");
        assert_eq!(call.code(), code, "{what}: wrong return code, {call:?}");
    }
}

#[test]
fn a_board_with_no_configuration_window_has_no_pci_bios() {
    // The negative control. POST *probes* for mechanism #1 rather than being
    // told about it, so unmapping the window is enough: every function,
    // including the installation check, has to say so. A firmware that
    // answered B101h out of a constant would pass every test above and fail
    // this one.
    let found = run(
        "pc-at-nopci.machine",
        &without_config_window(),
        stock_bios(),
    );
    for (n, call) in found.calls.iter().enumerate() {
        assert!(
            call.carry,
            "call {n} succeeded on a board with no PCI: {call:?}"
        );
        assert_eq!(
            call.code(),
            0x81,
            "call {n} should be FUNC_NOT_SUPPORTED, {call:?}"
        );
    }
    assert_ne!(
        found.signature, 0x2049_4350,
        "the signature was written anyway"
    );
}

/// The stock image; the tables it carries describe the stock board, which is
/// the board this test unmaps one port window of.
fn stock_bios() -> Vec<u8> {
    rsemu::fw::pcbios::image()
}

/// `machines/pc-at.machine` with the mechanism #1 window taken out of the port
/// map, and nothing else changed.
///
/// The host bridge is still there and still shadows the ROM; what is gone is
/// the *decode* — which is exactly the thing a firmware can find out by
/// asking, and the reason POST asks.
fn without_config_window() -> String {
    let text = String::from(rsemu::dev::pc::PC_AT);
    const WINDOW: &str = "  map port 0x0cf8 size 0x0008 = pmc.config";
    assert!(text.contains(WINDOW), "the 0xcf8 mapping moved");
    text.replace(WINDOW, "")
}
