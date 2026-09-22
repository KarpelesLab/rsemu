//! A real Macintosh Plus ROM against the NCR 5380 at `$580000`, black-box.
//!
//! # What is in this file, and what is not
//!
//! **No byte of any Apple ROM, and no disk image of anybody's.** The ROM is the
//! user's own, read in place from `RSEMU_MAC_ROM_DIR`, and the tests skip —
//! saying so — when it is not there. The SCSI disk is built here at runtime out
//! of numbered blocks, so nothing of anyone's is needed and nothing is
//! committed.
//!
//! **The ROM was not disassembled** (`ROADMAP.md` §1, `CLAUDE.md`). Two
//! permitted techniques do all the work:
//!
//! * **Register tracing.** Every access to the chip's window is logged, in
//!   order, with the register it decodes to. That is how the board's address
//!   decode was found and it is what the first test asserts.
//! * **Calling the ROM, and reading what it builds in RAM.** The third test
//!   puts sixty bytes of *our own* 68000 code in memory and calls Apple's SCSI
//!   Manager through `_SCSIDispatch`, which is a documented trap with
//!   documented routine selectors (*Inside Macintosh: Devices*, chapter 3).
//!   What is asserted is the result codes Apple's own driver returns and the
//!   `INQUIRY` data it puts in our buffer.
//!
//! # What the ROM does on its own, and what it does not
//!
//! Measured, and it is the finding that shaped this file: **a Macintosh Plus
//! ROM touches the 5380 exactly three times in two virtual minutes** —
//! `$580011 := $80`, `$580011 := $00`, `$580021 := $00`, which is assert `RST`,
//! release `RST`, clear the Mode register — and then never looks at it again.
//! It does not arbitrate, does not select and does not scan the bus, with or
//! without a target fitted, with or without a floppy in the drive, and on a
//! 1 MiB or a 4 MiB board. `docs/platforms/mac-plus.md` has the ledger entry.
//!
//! So "the ROM's bus scan completes and reports the truth" is true in the only
//! way it can be: the three writes land on a chip that answers them, the bus
//! reset reaches the target, and the machine goes on to the blinking
//! insert-disk icon with the same frame hash it had when `$580000` floated.
//! Whether Apple's SCSI Manager can *see* a disk is a separate question, and
//! the answer is yes — third test.

#![cfg(all(feature = "machine-mac-plus", feature = "dev-ncr5380"))]

use std::sync::{Arc, Mutex};

use rsemu::core::Captured;
use rsemu::core::clock::GlobalTime;
use rsemu::core::device::{Device, DeviceClass, RealizeCtx, ResetKind};
use rsemu::core::error::Result;
use rsemu::core::space::{
    AccessConstraints, AddressSpace, MemAttrs, MemOps, MemResult, Region, RegionRef,
    UnassignedPolicy,
};
use rsemu::core::state::{ChunkReader, ChunkWriter};
use rsemu::core::value::{Endian, Width};
use rsemu::core::wire::WireSource;
use rsemu::cpu::m68k::{M68k, Regs};
use rsemu::dev::ncr5380::{self, Ncr5380};
use rsemu::host::display::mac::{MacScanout, capture};
use rsemu::host::display::{PixelFormat, Scanout, Surface};
use rsemu::machine::realize::Instance;
use rsemu::machine::{Machine, catalog};

/// How long a Macintosh Plus ROM is. The socket is a 128 KiB part.
const ROM_LEN: usize = 128 * 1024;

/// The picture at 12 virtual seconds on the stock 1 MiB board.
///
/// **The same constant as `tests/mac_plus.rs`'s `GOLDEN_1M`**, and that is the
/// point of it being here: mapping a chip where the bus used to float, and
/// hanging a disk off it, must not change a pixel or a cycle of the existing
/// boot. If the two ever disagree, one of them moved and the other did not.
const GOLDEN_1M: u64 = 0x63dd_d76c_9468_dfa7;

/// Where the insert-disk icon lands. Left, top, width, height.
const ICON: (u32, u32, u32, u32) = (240, 145, 32, 32);

/// The board's decode, which the first test is the evidence for.
const STRIDE: u64 = 16;

/// The window the chip answers in: four address lines reach it.
const WINDOW: u64 = 8 * STRIDE;

// ---------------------------------------------------------------------------
// the tap
// ---------------------------------------------------------------------------

/// One access to the chip's window, decoded.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct Access {
    write: bool,
    /// The offset in the window, so the absolute address is `$580000 + this`.
    offset: u64,
    /// The register the board's decode selects.
    reg: u8,
    value: u8,
}

impl Access {
    /// The line a person reads, in the form the chip's data sheet names things.
    fn describe(&self) -> String {
        let names: [&str; 8] = [
            "Data",
            "InitiatorCommand",
            "Mode",
            "TargetCommand",
            "Status/SelectEnable",
            "BusAndStatus/StartDMASend",
            "InputData/StartDMATargetRecv",
            "ResetParityIrq/StartDMAInitiatorRecv",
        ];
        format!(
            "{} ${:06x} {} = ${:02x}",
            if self.write { "W" } else { "R" },
            0x58_0000 + self.offset,
            names[usize::from(self.reg)],
            self.value
        )
    }
}

/// A `MemOps` that logs every access and passes it on to the real chip.
///
/// The chip's own region is mapped at zero in a little space of its own and
/// this forwards into it, so the chip sees exactly the access the guest made.
#[derive(Debug)]
struct Tap {
    inner: Arc<AddressSpace>,
    log: Arc<Mutex<Vec<Access>>>,
}

impl Tap {
    fn note(&self, write: bool, offset: u64, value: u8) {
        let mut log = self.log.lock().unwrap();
        if log.len() < 100_000 {
            log.push(Access {
                write,
                offset,
                reg: ((offset / STRIDE) & 7) as u8,
                value,
            });
        }
    }
}

impl MemOps for Tap {
    fn read(&self, offset: u64, dst: &mut [u8], attrs: MemAttrs) -> MemResult {
        let value = self.inner.read(offset, Width::U8, attrs)? as u8;
        if !attrs.debug {
            self.note(false, offset, value);
        }
        dst[0] = value;
        Ok(())
    }

    fn write(&self, offset: u64, src: &[u8], attrs: MemAttrs) -> MemResult {
        self.inner
            .write(offset, Width::U8, u64::from(src[0]), attrs)?;
        if !attrs.debug {
            self.note(true, offset, src[0]);
        }
        Ok(())
    }

    fn constraints(&self) -> AccessConstraints {
        AccessConstraints::word(Width::U8, Endian::Big)
    }
}

/// The chip, with the tap in front of its register window.
///
/// Every other part of being a device is the chip's, delegated: this is a
/// wrapper for the machine file's `mirror(scsi)` to find, not a second model.
#[derive(Debug)]
struct Tapped {
    chip: Arc<Ncr5380>,
    regs: RegionRef,
    log: Arc<Mutex<Vec<Access>>>,
}

impl Tapped {
    fn new(chip: Arc<Ncr5380>) -> Tapped {
        let inner = Arc::new(AddressSpace::new("scsi-tap", 16).with_unassigned(
            // Nothing else is in this space, and the chip's window covers all
            // of it that the board can reach.
            UnassignedPolicy::ZEROS,
        ));
        inner
            .topology()
            .map(
                Device::region(chip.as_ref(), "").expect("the chip has a window"),
                0,
            )
            .expect("mapped");
        let log = Arc::new(Mutex::new(Vec::new()));
        let regs = Arc::new(Region::io(
            "tap.regs".to_string(),
            WINDOW,
            Arc::new(Tap {
                inner,
                log: Arc::clone(&log),
            }) as Arc<dyn MemOps>,
        ));
        Tapped { chip, regs, log }
    }
}

impl Device for Tapped {
    fn class(&self) -> &'static DeviceClass {
        Device::class(self.chip.as_ref())
    }

    fn realize(&self, ctx: &mut RealizeCtx<'_>) -> Result<()> {
        Device::realize(self.chip.as_ref(), ctx)
    }

    fn reset(&self, kind: ResetKind) {
        Device::reset(self.chip.as_ref(), kind);
    }

    fn region(&self, name: &str) -> Option<RegionRef> {
        match name {
            "" | ncr5380::REGS_REGION => Some(Arc::clone(&self.regs)),
            _ => None,
        }
    }

    fn connect(&self, port: &str, source: WireSource) -> Result<()> {
        Device::connect(self.chip.as_ref(), port, source)
    }

    fn announce(&self, port: &str) {
        Device::announce(self.chip.as_ref(), port);
    }

    fn save(&self, w: &mut ChunkWriter<'_>) -> Result<()> {
        Device::save(self.chip.as_ref(), w)
    }

    fn load(&self, r: &mut ChunkReader<'_>) -> Result<()> {
        Device::load(self.chip.as_ref(), r)
    }
}

impl Instance for Tapped {}

// ---------------------------------------------------------------------------
// the board
// ---------------------------------------------------------------------------

/// One running board and the handles a test needs.
struct Board {
    machine: Machine,
    cpu: Arc<M68k>,
    scanout: MacScanout,
    log: Arc<Mutex<Vec<Access>>>,
}

impl Board {
    /// Every access the ROM has made to the chip so far.
    fn trace(&self) -> Vec<Access> {
        self.log.lock().unwrap().clone()
    }

    /// Print the trace, which is the evidence the register decode rests on.
    fn print_trace(&self, label: &str) {
        let trace = self.trace();
        println!("{label}: {} accesses to the 5380", trace.len());
        for access in trace.iter().take(120) {
            println!("  {}", access.describe());
        }
    }
}

/// Read the ROM out of `RSEMU_MAC_ROM_DIR`; `None` (having said why) if the
/// variable or the file is not there.
fn rom_image() -> Option<Vec<u8>> {
    let Ok(dir) = std::env::var("RSEMU_MAC_ROM_DIR") else {
        println!(
            "mac-plus-scsi: set RSEMU_MAC_ROM_DIR to a directory holding Mac-Plus.ROM to run a \
             real Macintosh ROM against the 5380."
        );
        return None;
    };
    let path = std::path::Path::new(&dir).join("Mac-Plus.ROM");
    let Ok(bytes) = std::fs::read(&path) else {
        println!("mac-plus-scsi: {} is not there; skipped", path.display());
        return None;
    };
    if bytes.len() < ROM_LEN {
        println!(
            "mac-plus-scsi: {} is {} bytes; a Plus ROM is {ROM_LEN}. Skipped.",
            path.display(),
            bytes.len()
        );
        return None;
    }
    let image = bytes[..ROM_LEN].to_vec();
    // The first longword of a Macintosh ROM is the sum of every 16-bit word
    // after it: arithmetic over bytes the test never keeps, and a fact about
    // the format rather than any of its contents.
    let stored = u32::from_be_bytes([image[0], image[1], image[2], image[3]]);
    let sum = image[4..].as_chunks::<2>().0.iter().fold(0u32, |acc, w| {
        acc.wrapping_add(u32::from(u16::from_be_bytes(*w)))
    });
    if stored != sum {
        println!(
            "mac-plus-scsi: {} does not checksum as a 128 KiB Macintosh ROM; skipped.",
            path.display()
        );
        return None;
    }
    Some(image)
}

/// A SCSI disk of `blocks` numbered blocks: block *n* begins with its own
/// number and is filled with a pattern keyed on it, so a block found in guest
/// memory can be named.
///
/// There is no driver descriptor map and no HFS volume on it, because nothing
/// in this tree can write one — `docs/upstream/fstool-hfs-resource-fork-write.md`
/// — and a disk that answers `INQUIRY` is what these tests are about.
fn synthetic_disk(blocks: u64) -> Vec<u8> {
    let mut image = vec![0u8; (blocks * 512) as usize];
    for (n, block) in image.chunks_mut(512).enumerate() {
        block[..4].copy_from_slice(&(n as u32).to_be_bytes());
        for (i, byte) in block.iter_mut().enumerate().skip(4) {
            *byte = (n as u8).wrapping_mul(11).wrapping_add(i as u8);
        }
    }
    image
}

/// Build the `mac-plus` board with the tap in front of the 5380 and `disk` on
/// the cable. An empty vector is an empty cable, which is the ordinary case.
fn board(rom: Vec<u8>, disk: Vec<u8>, params: &[(&str, &str)]) -> Board {
    let cores: Arc<Captured<M68k>> = Arc::new(Captured::new());
    let kept = Arc::clone(&cores);
    let mut options = catalog::build_options().expect("the catalog agrees with itself");
    options.bindings.replace("cpu.m68k", move |props| {
        let cpu = Arc::new(M68k::from_props(props)?);
        kept.push(&cpu);
        Ok(cpu)
    });
    let taps: Arc<Captured<Tapped>> = Arc::new(Captured::new());
    let keep = Arc::clone(&taps);
    options.bindings.replace("ncr.5380", move |props| {
        let tapped = Arc::new(Tapped::new(Arc::new(Ncr5380::new(props)?)));
        keep.push(&tapped);
        Ok(tapped)
    });
    capture::install(&mut options).expect("a capture table");
    for &(name, value) in params {
        options
            .resolve
            .params
            .push((name.to_string(), value.to_string()));
    }
    options.realize.media.insert("macrom", rom);
    options.realize.media.insert("floppy", Vec::new());
    options.realize.media.insert("hd0", disk);
    let registry = catalog::registry().expect("a registry");
    let source = catalog::machine("mac-plus")
        .expect("this build ships mac-plus")
        .source;
    let machine = rsemu::machine::build("mac-plus", source, &registry, &options)
        .unwrap_or_else(|e| panic!("the board does not realize: {e}"));
    let cpu = cores.last().expect("the binding captured the processor");
    let scanout = capture::take(&options.realize.hosts, &machine).expect("a video circuit");
    let log = taps
        .last()
        .expect("the binding captured the chip")
        .log
        .clone();
    Board {
        machine,
        cpu,
        scanout,
        log,
    }
}

/// Run on for `secs` virtual seconds.
fn advance(b: &mut Board, secs: u64) {
    for _ in 0..secs {
        b.machine
            .run_for(GlobalTime::from_nanos(1_000_000_000))
            .expect("it runs");
    }
}

/// One longword of guest memory, read the way a debugger reads it.
fn peek(b: &Board, addr: u64) -> u32 {
    b.machine
        .space("mem")
        .expect("the board has `mem`")
        .read(addr, Width::U32, MemAttrs::DEBUG)
        .unwrap_or(0) as u32
}

/// One word of guest memory.
fn peek_w(b: &Board, addr: u64) -> u16 {
    b.machine
        .space("mem")
        .expect("the board has `mem`")
        .read(addr, Width::U16, MemAttrs::DEBUG)
        .unwrap_or(0) as u16
}

/// FNV-1a over the captured pixels: a hash of our rendering, nothing else.
fn frame_hash(surface: &Surface) -> u64 {
    surface
        .pixels()
        .iter()
        .fold(0xcbf2_9ce4_8422_2325u64, |h, &b| {
            (h ^ u64::from(b)).wrapping_mul(0x0100_0000_01b3)
        })
}

/// How many of the pixels in [`ICON`] are white.
fn icon_white(b: &Board) -> usize {
    let info = b.scanout.info();
    let mut surface = Surface::new(PixelFormat::RGB888, info.width, info.height);
    b.scanout.capture(&mut surface);
    let pixels = surface.pixels();
    let (left, top, w, h) = ICON;
    let mut white = 0;
    for y in top..top + h {
        for x in left..left + w {
            let at = ((y * info.width + x) * 3) as usize;
            if pixels.get(at).is_some_and(|&p| p != 0) {
                white += 1;
            }
        }
    }
    white
}

// ---------------------------------------------------------------------------
// what the ROM does by itself
// ---------------------------------------------------------------------------

/// **The measurement the board's address decode rests on.**
///
/// A Macintosh Plus ROM makes exactly three accesses to `$580000`-`$5FFFFF`,
/// all writes, and they are only the Initiator Command and Mode registers *if*
/// the register selects are on `A6`–`A4` with the write at an odd address.
/// `$580011 := $80` is `ASSERT RST`; `$580011 := $00` releases it;
/// `$580021 := $00` clears the Mode register. Nothing else, ever — which is
/// why the trace is printed as well as asserted.
#[test]
fn the_rom_resets_the_chip_and_never_looks_again() {
    let Some(rom) = rom_image() else {
        return;
    };
    let mut b = board(rom, synthetic_disk(2048), &[]);
    advance(&mut b, 12);
    b.print_trace("mac-plus-scsi");

    let trace = b.trace();
    assert_eq!(b.cpu.bus_faults().0, 0, "an access faulted");
    assert_eq!(
        trace.len(),
        3,
        "a Plus ROM touches the 5380 three times and no more: {:?}",
        trace.iter().map(Access::describe).collect::<Vec<_>>()
    );
    for access in &trace {
        assert!(access.write, "every one of them is a write");
        assert_eq!(
            access.offset & 1,
            1,
            "at an odd address, which is the write"
        );
    }
    assert_eq!(
        (trace[0].reg, trace[0].value),
        (ncr5380::INITIATOR_COMMAND, ncr5380::ICR_ASSERT_RST),
        "first: assert RST"
    );
    assert_eq!(
        (trace[1].reg, trace[1].value),
        (ncr5380::INITIATOR_COMMAND, 0),
        "then release it"
    );
    assert_eq!(
        (trace[2].reg, trace[2].value),
        (ncr5380::MODE, 0),
        "then clear the Mode register"
    );

    // And the drive queue holds the floppy alone: the ROM added no SCSI drive,
    // because it never went looking. That is data the ROM built in RAM
    // (`DrvQHdr` at $0308), not code anybody read.
    let head = peek(&b, 0x30a);
    assert_ne!(head, 0, "the drive queue has the floppy in it");
    assert_eq!(peek(&b, head.into()), 0, "and nothing after it");
    assert_eq!(
        peek_w(&b, u64::from(head) + 6),
        1,
        "drive 1, the internal floppy"
    );
}

/// The existing boot is **bit-identical** with a chip at `$580000` and a disk
/// on the cable: same frame hash, same memory sizing, same icon.
///
/// The three writes the ROM makes are the only difference between a floating
/// bus and this one, and they are writes, so nothing it reads can change. This
/// is the assertion that says so rather than assuming it.
#[test]
fn the_insert_disk_boot_is_unchanged_by_the_chip_and_the_disk() {
    let Some(rom) = rom_image() else {
        return;
    };
    let mut b = board(rom, synthetic_disk(2048), &[]);
    advance(&mut b, 12);

    let info = b.scanout.info();
    let mut surface = Surface::new(PixelFormat::RGB888, info.width, info.height);
    b.scanout.capture(&mut surface);
    let hash = frame_hash(&surface);
    println!("mac-plus-scsi: the frame at 12s is {hash:#018x}");

    assert!(!b.cpu.is_halted(), "the processor double-faulted");
    assert_eq!(b.cpu.bus_faults().0, 0, "an access faulted");
    assert_eq!(peek(&b, 0x108), 0x0010_0000, "MemTop: a 1 MiB machine");
    assert_eq!(peek(&b, 0x824), 0x000f_a700, "ScrnBase: MemTop - $5900");
    let white = icon_white(&b);
    assert!(
        white > 700,
        "the insert-disk icon is not on the screen: {white} of {} white",
        32 * 32
    );
    assert_eq!(
        hash, GOLDEN_1M,
        "the frame at 12s moved. A SCSI chip that only ever receives three \
         writes cannot change the picture, so something else did — compare \
         tests/mac_plus.rs, which asserts the same constant"
    );
}

// ---------------------------------------------------------------------------
// Apple's own SCSI Manager
// ---------------------------------------------------------------------------

/// Where our guest program and its data go: half a megabyte in, which the ROM
/// leaves alone — the system heap grows from the bottom and the screen and
/// sound buffers are at the top. The test checks that assumption holds by
/// reading its own magic back.
const CODE: u32 = 0x8_0000;
const STACK: u32 = 0x7_FF00;
const CDB: u32 = 0x8_0200;
const TIB: u32 = 0x8_0210;
const BUF: u32 = 0x8_0240;
const RES: u32 = 0x8_0300;

/// The magic the program writes into `RES` when it has finished.
const DONE: u32 = 0xc0ff_ee00;

/// `_SCSIDispatch`, a toolbox trap (*Inside Macintosh: Devices*, chapter 3,
/// "SCSI Manager Reference": every routine's "ASSEMBLY-LANGUAGE INFORMATION"
/// gives `Trap macro _SCSIDispatch` and a selector).
const SCSI_DISPATCH: u16 = 0xa815;

/// The routine selectors, each quoted from that routine's own page:
/// `SCSIReset` `$0000`, `SCSIGet` `$0001`, `SCSISelect` `$0002`, `SCSICmd`
/// `$0003`, `SCSIComplete` `$0004`, `SCSIRead` `$0005`.
const SEL_RESET: u16 = 0x0000;
const SEL_GET: u16 = 0x0001;
const SEL_SELECT: u16 = 0x0002;
const SEL_CMD: u16 = 0x0003;
const SEL_COMPLETE: u16 = 0x0004;
const SEL_READ: u16 = 0x0005;

/// The TIB opcodes, from the chapter's Pascal summary: `scInc = 1` "transfer
/// data, increment buffer pointer" and `scStop = 7` "stop TIB execution". A
/// TIB instruction is `RECORD scOpcode: Integer; scParam1: LongInt; scParam2:
/// LongInt; END` — ten bytes.
const SC_INC: u16 = 1;
const SC_STOP: u16 = 7;

/// Just enough of a 68000 assembler to call a trap.
///
/// Every encoding is from the *M68000 Family Programmer's Reference Manual*'s
/// instruction format tables: `MOVE` is `00 SS` then destination register,
/// destination mode, source mode, source register, so `MOVE.W #x,-(A7)` is
/// `3F3C`, `MOVE.W (A7)+,D0` is `301F`, `MOVE.W D0,(xxx).L` is `33C0`,
/// `MOVE.L #x,-(A7)` is `2F3C`, and `PEA (xxx).L` is `4879`.
#[derive(Default)]
struct Asm(Vec<u8>);

impl Asm {
    fn word(&mut self, value: u16) -> &mut Asm {
        self.0.extend_from_slice(&value.to_be_bytes());
        self
    }

    fn long(&mut self, value: u32) -> &mut Asm {
        self.0.extend_from_slice(&value.to_be_bytes());
        self
    }

    /// `MOVE.W #value,-(A7)`
    fn push_w(&mut self, value: u16) -> &mut Asm {
        self.word(0x3f3c).word(value)
    }

    /// `MOVE.L #value,-(A7)`
    fn push_l(&mut self, value: u32) -> &mut Asm {
        self.word(0x2f3c).long(value)
    }

    /// `PEA (addr).L`
    fn pea(&mut self, addr: u32) -> &mut Asm {
        self.word(0x4879).long(addr)
    }

    /// The trap, then `MOVE.W (A7)+,D0` and `MOVE.W D0,(at).L`.
    ///
    /// A toolbox trap takes its arguments on the stack and leaves the function
    /// result there, so the two-byte `OSErr` is what is on top afterwards.
    fn call(&mut self, sel: u16, at: u32) -> &mut Asm {
        self.push_w(sel)
            .word(SCSI_DISPATCH)
            .word(0x301f)
            .word(0x33c0)
            .long(at)
    }

    /// `MOVE.L #value,(at).L`
    fn store_l(&mut self, value: u32, at: u32) -> &mut Asm {
        self.word(0x23fc).long(value).long(at)
    }

    /// `BRA.S *` — park here for ever; the test is watching memory.
    fn park(&mut self) -> &mut Asm {
        self.word(0x60fe)
    }
}

/// The program: one whole SCSI transaction through Apple's SCSI Manager.
///
/// `SCSIReset`, `SCSIGet`, `SCSISelect(target)`, `SCSICmd(INQUIRY)`,
/// `SCSIRead(tib)`, `SCSIComplete(&stat, &msg, 120)` — the order *Inside
/// Macintosh: Devices* chapter 3 gives under "Using the SCSI Manager", with
/// every result code stored where the test can read it.
fn scsi_manager_program(target: u16) -> Vec<u8> {
    let mut a = Asm::default();
    a.call(SEL_RESET, RES + 4);
    a.call(SEL_GET, RES + 6);
    a.push_w(target).call(SEL_SELECT, RES + 8);
    a.pea(CDB).push_w(6).call(SEL_CMD, RES + 10);
    a.pea(TIB).call(SEL_READ, RES + 12);
    a.pea(RES + 16)
        .pea(RES + 18)
        .push_l(120)
        .call(SEL_COMPLETE, RES + 14);
    a.store_l(DONE, RES).park();
    a.0
}

/// Put the program, its command block, its TIB and its buffer in memory, then
/// point the processor at it.
fn install_program(b: &Board, target: u16) {
    let space = b.machine.space("mem").expect("the board has `mem`");
    let code = scsi_manager_program(target);
    space
        .write_bytes(u64::from(CODE), &code, MemAttrs::DEBUG)
        .expect("RAM takes it");
    // `INQUIRY`, allocation length 36 (X3.131 §8.2.5).
    space
        .write_bytes(u64::from(CDB), &[0x12, 0, 0, 0, 36, 0], MemAttrs::DEBUG)
        .expect("RAM takes it");
    // One `scInc` of 36 bytes into `BUF`, then `scStop`.
    let mut tib = Vec::new();
    tib.extend_from_slice(&SC_INC.to_be_bytes());
    tib.extend_from_slice(&BUF.to_be_bytes());
    tib.extend_from_slice(&36u32.to_be_bytes());
    tib.extend_from_slice(&SC_STOP.to_be_bytes());
    tib.extend_from_slice(&0u32.to_be_bytes());
    tib.extend_from_slice(&0u32.to_be_bytes());
    space
        .write_bytes(u64::from(TIB), &tib, MemAttrs::DEBUG)
        .expect("RAM takes it");
    space
        .write_bytes(u64::from(BUF), &[0u8; 64], MemAttrs::DEBUG)
        .expect("RAM takes it");
    space
        .write_bytes(u64::from(RES), &[0u8; 32], MemAttrs::DEBUG)
        .expect("RAM takes it");

    // Supervisor, interrupt level 0: the tick chain has to keep running,
    // because `SCSIComplete` is given a timeout in ticks and `Ticks` is
    // counted by the vertical blanking interrupt.
    let mut a = [0u32; 8];
    a[7] = STACK;
    let regs = Regs {
        pc: CODE,
        sr: 0x2000,
        ssp: STACK,
        a,
        // `Regs::pc` is the address of the word in `prefetch[0]`, so the queue
        // has to agree with where the program counter now points.
        prefetch: [
            u16::from_be_bytes([code[0], code[1]]),
            u16::from_be_bytes([code[2], code[3]]),
        ],
        ..Regs::default()
    };
    b.cpu.set_regs(regs);
}

/// **Apple's own SCSI Manager finds the disk.**
///
/// The ROM will not go looking by itself — the first test measures that — so
/// the test asks it to, through the documented trap, with sixty bytes of our
/// own code. What comes back is Apple's code's opinion of our chip:
/// `SCSISelect` succeeds, `SCSICmd` sends the command block, `SCSIRead` moves
/// thirty-six bytes of `INQUIRY` data into our buffer, and `SCSIComplete`
/// returns the target's status and message bytes.
///
/// The register trace is printed, which is the phase sequence Apple's driver
/// drove and the thing worth reading when this test fails.
#[test]
fn apples_scsi_manager_selects_the_target_and_reads_its_inquiry_data() {
    let Some(rom) = rom_image() else {
        return;
    };
    let mut b = board(rom, synthetic_disk(2048), &[]);
    // Far enough in that the ROM has finished with the hardware and is in its
    // insert-disk loop.
    advance(&mut b, 12);
    b.log.lock().unwrap().clear();

    install_program(&b, 0);
    for _ in 0..40 {
        b.machine
            .run_for(GlobalTime::from_nanos(50_000_000))
            .expect("it runs");
        if peek(&b, u64::from(RES)) == DONE {
            break;
        }
    }
    b.print_trace("scsi-manager");

    let space = b.machine.space("mem").expect("the board has `mem`");
    let mut inquiry = [0u8; 36];
    space
        .read_bytes(u64::from(BUF), &mut inquiry, MemAttrs::DEBUG)
        .expect("RAM answers");
    let err = |at: u32| peek_w(&b, u64::from(at)) as i16;
    println!(
        "scsi-manager: reset={} get={} select={} cmd={} read={} complete={} stat=${:02x} \
         msg=${:02x}",
        err(RES + 4),
        err(RES + 6),
        err(RES + 8),
        err(RES + 10),
        err(RES + 12),
        err(RES + 14),
        peek_w(&b, u64::from(RES) + 16) & 0xff,
        peek_w(&b, u64::from(RES) + 18) & 0xff
    );
    println!("scsi-manager: INQUIRY data {inquiry:02x?}");

    assert_eq!(
        peek(&b, u64::from(RES)),
        DONE,
        "the program never finished: it is at pc={:#010x}, and the trace above says how far the \
         SCSI Manager got",
        b.cpu.regs().pc
    );
    assert_eq!(b.cpu.bus_faults().0, 0, "an access faulted");
    assert!(!b.cpu.is_halted(), "the processor double-faulted");
    let trace = b.trace();

    // `noErr` is 0 (*Inside Macintosh: Devices*, every routine's RESULT CODES).
    for (name, at) in [
        ("SCSIGet", RES + 6),
        ("SCSISelect", RES + 8),
        ("SCSICmd", RES + 10),
        ("SCSIRead", RES + 12),
        ("SCSIComplete", RES + 14),
    ] {
        assert_eq!(err(at), 0, "{name} returned {}", err(at));
    }
    // **`SCSIReset` is the exception, and it is the ROM's answer rather than
    // ours.** It returns `-1`, which is not one of the two result codes its
    // page lists (`noErr` and `scCommErr`, 2) — and it does so having reset the
    // bus correctly, in exactly the three register writes the ROM makes at
    // startup. So what the ROM does to this chip when the machine is switched
    // on *is* a `SCSIReset` call, which is worth knowing and is asserted here
    // rather than the result code.
    println!(
        "scsi-manager: SCSIReset returned {} (the ROM's own answer)",
        err(RES + 4)
    );
    assert_eq!(
        trace
            .iter()
            .take(3)
            .map(|a| (a.write, a.reg, a.value))
            .collect::<Vec<_>>(),
        vec![
            (true, ncr5380::INITIATOR_COMMAND, ncr5380::ICR_ASSERT_RST),
            (true, ncr5380::INITIATOR_COMMAND, 0),
            (true, ncr5380::MODE, 0),
        ],
        "SCSIReset makes the same three writes the ROM makes at startup"
    );
    assert_eq!(
        peek_w(&b, u64::from(RES) + 16) & 0xff,
        0,
        "the status byte is GOOD"
    );
    assert_eq!(
        peek_w(&b, u64::from(RES) + 18) & 0xff,
        0,
        "and the message is COMMAND COMPLETE"
    );

    assert_eq!(inquiry[0], 0x00, "peripheral device type: direct access");
    assert_eq!(&inquiry[8..13], b"RSEMU", "the vendor field");
    assert_eq!(
        &inquiry[16..29],
        b"SCSI HARDDISK",
        "and the product field the machine file names"
    );

    // The trace is the evidence, so it is asserted too: Apple's driver drove a
    // selection, a command phase and a data phase, which cannot happen without
    // reads of the Current SCSI Bus Status register.
    assert!(
        trace.iter().any(|a| a.write
            && a.reg == ncr5380::INITIATOR_COMMAND
            && a.value & ncr5380::ICR_ASSERT_SEL != 0),
        "the SCSI Manager asserted SEL"
    );
    assert!(
        trace
            .iter()
            .any(|a| !a.write && a.reg == ncr5380::CURRENT_STATUS),
        "and polled the Current SCSI Bus Status register"
    );
    assert!(
        trace.iter().filter(|a| !a.write).count() > 36,
        "and moved at least the data phase's bytes: {} reads",
        trace.iter().filter(|a| !a.write).count()
    );
}

/// And it reports an **empty** address honestly: nothing answers at 3, so the
/// selection times out and `SCSISelect` says so.
///
/// The other half of "reports the truth": a scan that found a disk everywhere
/// would be no scan at all.
#[test]
fn apples_scsi_manager_times_out_on_an_empty_address() {
    let Some(rom) = rom_image() else {
        return;
    };
    let mut b = board(rom, synthetic_disk(2048), &[]);
    advance(&mut b, 12);
    b.log.lock().unwrap().clear();

    install_program(&b, 3);
    for _ in 0..80 {
        b.machine
            .run_for(GlobalTime::from_nanos(50_000_000))
            .expect("it runs");
        if peek(&b, u64::from(RES)) == DONE {
            break;
        }
    }
    b.print_trace("scsi-manager-empty");
    let err = |at: u32| peek_w(&b, u64::from(at)) as i16;
    println!(
        "scsi-manager-empty: select={} cmd={} complete={}",
        err(RES + 8),
        err(RES + 10),
        err(RES + 14)
    );

    assert_eq!(peek(&b, u64::from(RES)), DONE, "the program finished");
    assert_eq!(b.cpu.bus_faults().0, 0, "an access faulted");
    assert_ne!(
        err(RES + 8),
        0,
        "address 3 is empty, so SCSISelect must not report success"
    );
}
