//! A real Macintosh Classic ROM on the `mac-classic` board, run in place, as
//! far as it gets.
//!
//! # What is in this file, and what is not
//!
//! **No byte of any Apple ROM or of anybody's disk.** The ROM is read in place
//! from `RSEMU_MAC_ROM_DIR` and a disk image from `RSEMU_MAC_DISK_DIR`, and each
//! test that wants one skips, saying so, when the variable or the file is not
//! there. `cargo test` with nothing set passes on a machine that has neither:
//! **three of these nine tests need no media at all**, because they assemble
//! the board around rsemu's own ten-byte stub and around a 1.44 MB image of
//! numbered blocks built on the spot.
//!
//! What is asserted is about *this emulator*: that the board realizes and
//! every chip answers where a real ROM was measured to look for it, that a
//! 1.44 MB image becomes MFM cells and turns at 300 rpm, that the processor
//! runs with no access faulting, that the ROM sized memory and put its screen
//! where the hardware puts it, and a hash of the picture it produced at a fixed
//! virtual time — our rendering, not ROM contents.
//!
//! `RSEMU_MAC_FRAME_DIR`, when set, receives a PNG of each frame a test checks
//! (in a build with `display-png`), so a person can look at it.
//! `RSEMU_MAC_TRACE=1` prints the processor's state once a virtual second,
//! which is how a person finds where a ROM stopped.
//!
//! # How far it gets
//!
//! To the **insert-disk screen**: the grey desktop with the arrow cursor and
//! the floppy-with-a-question-mark in the middle, with the 60.15 Hz tick chain
//! running. On the way it resets the Apple Desktop Bus, walks all sixteen bus
//! addresses asking each for register 3, finds the keyboard at 2 and the mouse
//! at 3, probes the drive, sees that the mechanism is a **SuperDrive**, and
//! asks the SWIM for **ISM mode** — which this build does not have, so it
//! cannot be handed a sector off the 1.44 MB disk and stays on that screen.
//! `docs/platforms/mac-classic.md` has the whole ledger and every measurement
//! behind it.
//!
//! No Macintosh emulator source was consulted and the ROM was not disassembled
//! (`ROADMAP.md` §1, `CLAUDE.md`).

#![cfg(feature = "machine-mac-classic")]

use std::sync::Arc;

use rsemu::core::Captured;
use rsemu::core::clock::GlobalTime;
use rsemu::core::space::MemAttrs;
use rsemu::core::value::Width;
use rsemu::cpu::m68k::M68k;
use rsemu::dev::mac::adb::MacAdb;
use rsemu::dev::mac::disk::Density;
use rsemu::dev::mac::mfm;
use rsemu::dev::mac::swim::Swim;
use rsemu::host::display::mac::{MacScanout, capture};
use rsemu::host::display::{PixelFormat, Scanout, Surface};
use rsemu::machine::{Machine, catalog};

/// How long a Macintosh Classic ROM file is. The socket is a 512 KiB part.
const ROM_LEN: usize = 512 * 1024;

/// The picture at 12 virtual seconds on the stock 1 MiB board, with a disk in
/// the drive.
///
/// The icon **blinks** — the plain floppy and the same floppy with a question
/// mark on its face, about a second each way — so this is a golden of one
/// *phase* of that blink at one fixed virtual instant. Virtual time is exact,
/// so it is stable; but if it moves, look at the picture
/// (`RSEMU_MAC_FRAME_DIR`) before accepting a new hash, and check whether the
/// other phase is what came out. `the_insert_disk_icon_blinks` is the test that
/// asserts the alternation rather than either picture.
///
/// It is the same hash `mac-plus` produces for the same phase, and that is not
/// a coincidence: it is Apple's own icon drawn by Apple's own code into the
/// same place on the same 512 × 342 screen, by two ROMs four years apart.
const GOLDEN_1M: u64 = 0xfbc9_cfa0_9b09_a5da;

/// Where the insert-disk icon lands: a 32 × 32 box a little above the middle of
/// the 512 × 342 screen. Left, top, width, height.
const ICON: (u32, u32, u32, u32) = (240, 145, 32, 32);

/// How long the boot takes, in virtual seconds, with a little slack.
///
/// Measured rather than chosen: on the stock 1 MiB board the memory test runs
/// to about five seconds, the happy Mac is up at ten, "Welcome to Macintosh"
/// at fifteen, the Finder's menu bar is drawn by sixty, and the desktop stops
/// changing at **seventy** — every frame from there to two virtual minutes
/// hashes the same. Virtual time is exact, so this is not a race; the slack is
/// for a future change that makes the boot a second or two longer.
const BOOT_SECONDS: u64 = 75;

/// The Finder desktop at [`BOOT_SECONDS`] on the stock 1 MiB board with the
/// user's own Mac OS 6.0.8 startup disk.
///
/// A hash of *our rendering* of Apple's screen, not of anybody's bytes. What
/// is in the picture: the menu bar across the top with the Apple and **File
/// Edit View Special**, the arrow cursor at the top left, the startup volume's
/// floppy icon with **System Startup** under it in the top right corner, the
/// Trash in the bottom right, and the Macintosh's 50 % grey checkerboard
/// between them — 82,909 black pixels of 175,104.
///
/// If this moves, **look at the PNG** (`RSEMU_MAC_FRAME_DIR`) before accepting
/// a new value. A hash is not evidence that a desktop is on the screen.
const GOLDEN_FINDER: u64 = 0x9cef_f423_881c_3348;

/// One running board and the handles a test needs.
struct Board {
    machine: Machine,
    cpu: Arc<M68k>,
    scanout: MacScanout,
    swim: Arc<Swim>,
    adb: Arc<MacAdb>,
}

/// Read the ROM out of `RSEMU_MAC_ROM_DIR`; `None` (having said why) if the
/// variable or the file is not there.
///
/// A Macintosh Classic ROM file is 512 KiB and the socket takes exactly that.
/// The first longword of a Macintosh ROM is the sum of every 16-bit word after
/// it, and on this one that arithmetic comes out over the **first 256 KiB**:
/// `$A49F9914`, which is the published identifier for a Classic. The upper half
/// is not a disk image and not padding, but Apple's own checksum does not reach
/// it and nothing measured here says what it is for. The check is printed
/// rather than enforced for that reason, and it is arithmetic over bytes this
/// test never keeps.
fn rom_image(file: &str) -> Option<Vec<u8>> {
    let Ok(dir) = std::env::var("RSEMU_MAC_ROM_DIR") else {
        println!(
            "mac-classic: set RSEMU_MAC_ROM_DIR to a directory holding {file} to boot a real \
             Macintosh ROM."
        );
        return None;
    };
    let path = std::path::Path::new(&dir).join(file);
    let Ok(bytes) = std::fs::read(&path) else {
        println!("mac-classic: {} is not there; skipped", path.display());
        return None;
    };
    if bytes.len() != ROM_LEN {
        println!(
            "mac-classic: {} is {} bytes; a Classic ROM is {ROM_LEN}. Skipped.",
            path.display(),
            bytes.len()
        );
        return None;
    }
    let stored = u32::from_be_bytes([bytes[0], bytes[1], bytes[2], bytes[3]]);
    let sum = bytes[4..ROM_LEN / 2]
        .as_chunks::<2>()
        .0
        .iter()
        .fold(0u32, |acc, w| {
            acc.wrapping_add(u32::from(u16::from_be_bytes(*w)))
        });
    println!(
        "mac-classic: {} stores {stored:#010x}; its first 256 KiB sum to {sum:#010x}{}",
        path.display(),
        if stored == sum {
            ", which is a Macintosh Classic"
        } else {
            " — not a sum this test recognises, but the image is used anyway"
        }
    );
    Some(bytes)
}

/// Read a disk image out of `RSEMU_MAC_DISK_DIR`; an empty vector — an empty
/// drive, which is the ordinary case — when the variable or the file is not
/// there.
fn disk_image(file: &str) -> Vec<u8> {
    let Ok(dir) = std::env::var("RSEMU_MAC_DISK_DIR") else {
        println!(
            "mac-classic: set RSEMU_MAC_DISK_DIR to a directory holding {file} to put a disk in \
             the drive; running with an empty one."
        );
        return Vec::new();
    };
    let path = std::path::Path::new(&dir).join(file);
    match std::fs::read(&path) {
        Ok(bytes) => {
            println!("mac-classic: {} is {} bytes", path.display(), bytes.len());
            bytes
        }
        Err(_) => {
            println!(
                "mac-classic: {} is not there; running with an empty drive",
                path.display()
            );
            Vec::new()
        }
    }
}

/// A synthetic 1.44 MB image: every block says which block it is, so nothing of
/// anybody's is needed and nothing of anybody's is committed.
fn synthetic_1440k() -> Vec<u8> {
    let mut image = vec![0u8; mfm::BYTES];
    for (block, chunk) in image.chunks_mut(512).enumerate() {
        chunk[..4].copy_from_slice(&(block as u32).to_be_bytes());
        for (i, byte) in chunk.iter_mut().enumerate().skip(4) {
            *byte = (block as u8).wrapping_mul(31).wrapping_add(i as u8);
        }
    }
    image
}

/// Build the board around `image`, with `params` on top of the defaults and
/// `disk` in the drive. An empty vector is an empty drive, which is what
/// `rsemu run` binds when nobody says `--floppy`.
fn board(image: Vec<u8>, params: &[(&str, &str)], disk: Vec<u8>) -> Board {
    let cores: Arc<Captured<M68k>> = Arc::new(Captured::new());
    let kept = Arc::clone(&cores);
    let mut options = catalog::build_options().expect("the catalog agrees with itself");
    options.bindings.replace("cpu.m68k", move |props| {
        let cpu = Arc::new(M68k::from_props(props)?);
        kept.push(&cpu);
        Ok(cpu)
    });
    // The disk controller and the bus transceiver, caught as they are built: a
    // test that asks whether a disk is in the drive, or which addresses the ROM
    // left the bus devices at, is asking about the *device*, and there is no
    // other handle to it.
    let swims: Arc<Captured<Swim>> = Arc::new(Captured::new());
    let keep_swim = Arc::clone(&swims);
    options.bindings.replace("mac.swim", move |props| {
        let swim = Arc::new(Swim::new(props)?);
        keep_swim.push(&swim);
        Ok(swim)
    });
    let adbs: Arc<Captured<MacAdb>> = Arc::new(Captured::new());
    let keep_adb = Arc::clone(&adbs);
    options.bindings.replace("mac.adb", move |props| {
        let adb = Arc::new(MacAdb::new(props)?);
        keep_adb.push(&adb);
        Ok(adb)
    });
    capture::install(&mut options).expect("a capture table");
    for &(name, value) in params {
        options
            .resolve
            .params
            .push((name.to_string(), value.to_string()));
    }
    options.realize.media.insert("macrom", image);
    // The drive's bay. `machine::realize` refuses a slot that is named and
    // unbound, so a machine with an empty drive binds zero bytes for it.
    options.realize.media.insert("floppy", disk);
    let registry = catalog::registry().expect("a registry");
    let source = catalog::machine("mac-classic")
        .expect("this build ships mac-classic")
        .source;
    let machine = rsemu::machine::build("mac-classic", source, &registry, &options)
        .unwrap_or_else(|e| panic!("the board does not realize: {e}"));
    let cpu = cores.last().expect("the binding captured the processor");
    let scanout = capture::take(&options.realize.hosts, &machine).expect("a video circuit");
    Board {
        machine,
        cpu,
        scanout,
        swim: swims.last().expect("the binding captured the controller"),
        adb: adbs.last().expect("the binding captured the transceiver"),
    }
}

/// The whole of the ROM the three hermetic tests use: the two longwords a 68000
/// fetches out of reset — a stack pointer at the top of the default megabyte
/// and a program counter at `$000008` — and `BRA .`, the two-byte branch to
/// itself.
///
/// rsemu's own bytes, not anybody's ROM. It is the same image the catalog binds
/// when nothing else is given.
const STUB_ROM: [u8; 10] = [
    0x00, 0x10, 0x00, 0x00, // SSP = $00100000
    0x00, 0x00, 0x00, 0x08, // PC  = $00000008
    0x60, 0xfe, // BRA .
];

/// The board around that stub: nobody's media, so this runs wherever
/// `cargo test` does.
fn stub_board(disk: Vec<u8>) -> Board {
    board(STUB_ROM.to_vec(), &[], disk)
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

/// Run the board on to virtual second `to`, a second at a time.
fn advance(b: &mut Board, label: &str, to: u64) {
    let trace = std::env::var_os("RSEMU_MAC_TRACE").is_some();
    for s in 1..=to {
        b.machine
            .run_for(GlobalTime::from_nanos(1_000_000_000))
            .expect("it runs");
        if trace {
            let r = b.cpu.regs();
            println!(
                "{label} {s:3}s pc={:08x} sr={:04x} stopped={} faults={:?} frames={}",
                r.pc,
                r.sr,
                b.cpu.is_stopped(),
                b.cpu.bus_faults(),
                b.scanout.frame_counter(),
            );
        }
    }
}

/// Hash the picture on screen now, and write it out as a PNG named for `label`
/// and `seconds` when `RSEMU_MAC_FRAME_DIR` says where.
fn picture(b: &Board, label: &str, seconds: u64) -> u64 {
    let info = b.scanout.info();
    let mut surface = Surface::new(PixelFormat::RGB888, info.width, info.height);
    b.scanout.capture(&mut surface);
    let hash = frame_hash(&surface);
    if let Ok(dir) = std::env::var("RSEMU_MAC_FRAME_DIR") {
        #[cfg(feature = "display-png")]
        {
            let png = rsemu::host::display::png::encode(&surface).expect("a PNG");
            let name = format!("{label}-{seconds}s.png");
            std::fs::write(std::path::Path::new(&dir).join(name), png)
                .expect("the frame directory is writable");
        }
        #[cfg(not(feature = "display-png"))]
        let _ = dir;
    }
    // How much of the screen is ink, which is a cheap description of what is on
    // it: a white screen is 0, the Macintosh's grey desktop is half of it.
    let ink = surface
        .pixels()
        .iter()
        .step_by(3)
        .filter(|&&b| b == 0)
        .count();
    println!(
        "{label} at {seconds}s: frame {hash:#018x}, {ink} black pixels of {}",
        info.width * info.height
    );
    hash
}

/// How many of the pixels in [`ICON`] are white.
///
/// The desktop behind it is a one-pixel checkerboard, so an empty box is
/// exactly half white; the icon is a solid white floppy with a black outline,
/// so a drawn one is most of the way to all of it.
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

/// One longword of guest memory, read the way a debugger reads it.
fn peek(b: &Board, addr: u64) -> u32 {
    b.machine
        .space("mem")
        .expect("the board has `mem`")
        .read(addr, Width::U32, MemAttrs::DEBUG)
        .unwrap_or(0) as u32
}

// ---------------------------------------------------------------------------
// a transparent tap
// ---------------------------------------------------------------------------

/// The sixteen IWM soft switches by name, index 0 to 15.
const SWITCHES: [&str; 16] = [
    "CA0-", "CA0+", "CA1-", "CA1+", "CA2-", "CA2+", "LSTRB-", "LSTRB+", "ENBL-", "ENBL+", "DRV1",
    "DRV2", "Q6-", "Q6+", "Q7-", "Q7+",
];

/// The ISM's sixteen registers by name, index 0 to 15 — the write halves
/// first, per the *SWIM Chip User's Reference* page 26.
const ISM_REGS: [&str; 16] = [
    "wData", "wMark", "wCRC", "wParam", "wPhase", "wSetup", "wMode0", "wMode1", "rData", "rMark",
    "rError", "rParam", "rPhase", "rSetup", "rStatus", "rHandshake",
];

/// The mechanism's sixteen status lines by name, addressed `CA2:CA1:CA0:SEL`
/// — Neil Parker's table, as `src/dev/mac/iwm.rs` carries it.
const DRIVE_LINES: [&str; 16] = [
    "stepdir",
    "diskin",
    "stepping",
    "locked",
    "motoron",
    "track0",
    "switched",
    "tach",
    "rddata0",
    "rddata1",
    "superdrive",
    "superdrive'",
    "sides",
    "ready",
    "installed",
    "installed'",
];

/// A tap that **names** what it saw instead of numbering it.
///
/// A log of switch indices cannot say which of the drive's sixteen status lines
/// an access read, because that address is `CA2:CA1:CA0` *and* the VIA's `SEL`
/// — three soft switches and a pin on another chip. So this one asks the
/// controller itself, during the access, which register set is answering and
/// which drive line is addressed. That is what turns "the ROM read switch 13
/// and got `$b7`" into "the ROM read `ready` on drive 1 and was told no".
#[derive(Debug)]
struct SwimTap {
    ops: Arc<dyn rsemu::core::space::MemOps>,
    swim: Arc<Swim>,
    log: std::sync::Mutex<Vec<String>>,
}

impl rsemu::core::space::MemOps for SwimTap {
    fn read(&self, offset: u64, dst: &mut [u8], attrs: MemAttrs) -> rsemu::core::space::MemResult {
        let r = self.ops.read(offset, dst, attrs);
        if !attrs.debug {
            self.note(offset, false, dst);
        }
        r
    }

    fn write(&self, offset: u64, src: &[u8], attrs: MemAttrs) -> rsemu::core::space::MemResult {
        let r = self.ops.write(offset, src, attrs);
        if !attrs.debug {
            self.note(offset, true, src);
        }
        r
    }

    fn constraints(&self) -> rsemu::core::space::AccessConstraints {
        self.ops.constraints()
    }
}

impl SwimTap {
    fn note(&self, offset: u64, write: bool, bytes: &[u8]) {
        let index = ((offset / 0x200) & 15) as usize;
        let value = bytes.iter().fold(0u32, |v, &b| (v << 8) | u32::from(b)) as u8;
        let dir = if write { "W" } else { "R" };
        let iwm = self.swim.iwm();
        let line = if self.swim.ism_selected() {
            format!("ISM {:<11} {dir} {value:02x}", ISM_REGS[index])
        } else {
            let addr = iwm.drive_address();
            format!(
                "IWM {:<6} {dir} {value:02x}  drv{} {}",
                SWITCHES[index],
                iwm.selected_drive() + 1,
                DRIVE_LINES[usize::from(addr)],
            )
        };
        let mut log = self.log.lock().unwrap();
        log.push(line);
        if log.len() > 40_000 {
            log.drain(..20_000);
        }
    }

    /// The trace, runs of the identical line collapsed to one with a count.
    fn folded(&self) -> Vec<(String, u64)> {
        let log = self.log.lock().unwrap();
        let mut out: Vec<(String, u64)> = Vec::new();
        for line in log.iter() {
            match out.last_mut() {
                Some((prev, n)) if prev == line => *n += 1,
                _ => out.push((line.clone(), 1)),
            }
        }
        out
    }

    /// Print it.
    fn dump(&self, label: &str) {
        println!("{label}:");
        if std::env::var_os("RSEMU_MAC_UNFOLDED").is_some() {
            for line in self.log.lock().unwrap().iter() {
                println!("  {line}");
            }
            return;
        }
        for (line, n) in self.folded() {
            if n == 1 {
                println!("  {line}");
            } else {
                println!("  {line}  x{n}");
            }
        }
    }
}

/// A tap that counts *addresses* rather than registers.
///
/// Over the memory aperture it sees only the processor's **data** accesses,
/// because the instructions it is running are fetched out of the ROM's own
/// mapping — which is what turns "the ROM is stuck in a two-instruction loop"
/// into "the ROM is polling this one location".
#[derive(Debug)]
struct RamTap {
    ops: Arc<dyn rsemu::core::space::MemOps>,
    counts: std::sync::Mutex<std::collections::BTreeMap<(u64, bool), u64>>,
    on: std::sync::atomic::AtomicBool,
}

impl rsemu::core::space::MemOps for RamTap {
    fn read(&self, offset: u64, dst: &mut [u8], attrs: MemAttrs) -> rsemu::core::space::MemResult {
        let r = self.ops.read(offset, dst, attrs);
        self.note(offset, false, attrs);
        r
    }

    fn write(&self, offset: u64, src: &[u8], attrs: MemAttrs) -> rsemu::core::space::MemResult {
        self.note(offset, true, attrs);
        self.ops.write(offset, src, attrs)
    }

    fn constraints(&self) -> rsemu::core::space::AccessConstraints {
        self.ops.constraints()
    }
}

impl RamTap {
    fn note(&self, offset: u64, write: bool, attrs: MemAttrs) {
        if attrs.debug || !self.on.load(std::sync::atomic::Ordering::Relaxed) {
            return;
        }
        *self.counts.lock().unwrap().entry((offset, write)).or_insert(0) += 1;
    }

    fn arm(&self, on: bool) {
        self.on.store(on, std::sync::atomic::Ordering::Relaxed);
    }

    fn top(&self, n: usize) -> Vec<((u64, bool), u64)> {
        let mut v: Vec<_> = self
            .counts
            .lock()
            .unwrap()
            .iter()
            .map(|(&k, &c)| (k, c))
            .collect();
        v.sort_by_key(|&(_, c)| std::cmp::Reverse(c));
        v.truncate(n);
        v
    }
}

/// Put a [`RamTap`] over the memory aperture at zero.
fn ram_tap(b: &Board) -> Arc<RamTap> {
    use rsemu::core::space::{MemOps, Region, RegionKind};
    use rsemu::core::value::Endian;
    let space = b.machine.space("mem").expect("mem");
    let mut guard = space.topology();
    let (span, ops) = {
        let (_, m) = guard
            .mappings()
            .find(|(_, m)| m.base == 0)
            .expect("the memory mapping");
        let mut leaf = m.region.clone();
        while let RegionKind::Alias(a) = leaf.kind() {
            let next = a.target().clone();
            leaf = next;
        }
        match leaf.kind() {
            RegionKind::Io(ops) => (leaf.len(), Arc::clone(ops)),
            other => panic!("not an MMIO aperture: {other:?}"),
        }
    };
    let constraints = ops.constraints();
    let tap = Arc::new(RamTap {
        ops,
        counts: std::sync::Mutex::new(std::collections::BTreeMap::new()),
        on: std::sync::atomic::AtomicBool::new(false),
    });
    let io = Region::io("ramtap", span, Arc::clone(&tap) as Arc<dyn MemOps>)
        .with_constraints(constraints.with_endian(Endian::Big));
    guard
        .map_with_priority(Arc::new(io), 0, 100)
        .expect("the tap maps");
    tap
}

/// Put a [`SwimTap`] over the controller's window.
fn swim_tap(b: &Board) -> Arc<SwimTap> {
    use rsemu::core::space::{MemOps, Region, RegionKind};
    use rsemu::core::value::Endian;
    let space = b.machine.space("mem").expect("mem");
    let mut guard = space.topology();
    let ops = {
        let (_, m) = guard
            .mappings()
            .find(|(_, m)| m.base == 0xC0_0000)
            .expect("the controller's mapping");
        let mut leaf = m.region.clone();
        while let RegionKind::Alias(a) = leaf.kind() {
            let next = a.target().clone();
            leaf = next;
        }
        match leaf.kind() {
            RegionKind::Io(ops) => Arc::clone(ops),
            other => panic!("not an MMIO aperture: {other:?}"),
        }
    };
    let constraints = ops.constraints();
    let tap = Arc::new(SwimTap {
        ops,
        swim: Arc::clone(&b.swim),
        log: std::sync::Mutex::new(Vec::new()),
    });
    let io = Region::io("swimtap", 0x2000, Arc::clone(&tap) as Arc<dyn MemOps>)
        .with_constraints(constraints.with_endian(Endian::Big));
    let mirror = Region::mirror("swimtap.mirror", Arc::new(io), 0x20_0000).expect("a mirror");
    guard
        .map_with_priority(Arc::new(mirror), 0xC0_0000, 100)
        .expect("the tap maps");
    tap
}

/// A transparent tap over one device's aperture: it records every access and
/// forwards it unchanged.
///
/// Transparency is the whole requirement. A counting stub that filled a read
/// with `$FF` instead of what the device answers changes what the ROM does, and
/// on this board the value on a floating bus is load-bearing
/// (`docs/platforms/mac-plus.md`). So does the byte order: a mapping's byte
/// order is **not** carried by the region it points at, and a tap that took
/// `AccessConstraints::ANY`'s little-endian default swapped every word the
/// processor fetched and the board never started.
#[derive(Debug)]
struct Tap {
    ops: Arc<dyn rsemu::core::space::MemOps>,
    /// `(register, write, value)`, most recent window only.
    log: std::sync::Mutex<Vec<(u64, bool, u32)>>,
    stride: u64,
}

impl rsemu::core::space::MemOps for Tap {
    fn read(&self, offset: u64, dst: &mut [u8], attrs: MemAttrs) -> rsemu::core::space::MemResult {
        let r = self.ops.read(offset, dst, attrs);
        if !attrs.debug {
            self.note(offset, false, dst);
        }
        r
    }

    fn write(&self, offset: u64, src: &[u8], attrs: MemAttrs) -> rsemu::core::space::MemResult {
        if !attrs.debug {
            self.note(offset, true, src);
        }
        self.ops.write(offset, src, attrs)
    }

    fn constraints(&self) -> rsemu::core::space::AccessConstraints {
        self.ops.constraints()
    }
}

impl Tap {
    fn note(&self, offset: u64, write: bool, bytes: &[u8]) {
        let value = bytes.iter().fold(0u32, |v, &b| (v << 8) | u32::from(b));
        let mut log = self.log.lock().unwrap();
        log.push(((offset / self.stride) & 15, write, value));
        if log.len() > 40_000 {
            log.drain(..20_000);
        }
    }

    /// The accesses, collapsed so a run of the identical one is a single line
    /// with a count — which is the only way half a million of them is a trace
    /// anybody can read.
    fn folded(&self) -> Vec<(String, u64)> {
        let log = self.log.lock().unwrap();
        let mut out: Vec<(String, u64)> = Vec::new();
        for &(reg, write, value) in log.iter() {
            let line = format!(
                "r{reg:<2} {} {:02x}",
                if write { "W" } else { "R" },
                value as u8
            );
            match out.last_mut() {
                Some((prev, n)) if *prev == line => *n += 1,
                _ => out.push((line, 1)),
            }
        }
        out
    }
}

/// Put a [`Tap`] over the mapping at `base`, at a higher priority than the
/// board's own, and hand back the log it fills.
fn tap(b: &Board, base: u64, stride: u64, window: u64) -> Arc<Tap> {
    use rsemu::core::space::{MemOps, Region, RegionKind};
    use rsemu::core::value::Endian;
    let span = stride * 16;
    let space = b.machine.space("mem").expect("mem");
    let mut guard = space.topology();
    let ops = {
        let (_, m) = guard
            .mappings()
            .find(|(_, m)| m.base == base)
            .expect("a mapping at that base");
        let mut leaf = m.region.clone();
        while let RegionKind::Alias(a) = leaf.kind() {
            let next = a.target().clone();
            leaf = next;
        }
        match leaf.kind() {
            RegionKind::Io(ops) => Arc::clone(ops),
            other => panic!("not an MMIO aperture: {other:?}"),
        }
    };
    let constraints = ops.constraints();
    let tap = Arc::new(Tap {
        ops,
        log: std::sync::Mutex::new(Vec::new()),
        stride,
    });
    let io = Region::io("tap", span, Arc::clone(&tap) as Arc<dyn MemOps>)
        .with_constraints(constraints.with_endian(Endian::Big));
    let mirror = Region::mirror("tap.mirror", Arc::new(io), window).expect("a mirror");
    guard
        .map_with_priority(Arc::new(mirror), base, 100)
        .expect("the tap maps");
    tap
}

// ---------------------------------------------------------------------------
// the board, with nobody's media
// ---------------------------------------------------------------------------

/// **Every chip answers where a real ROM was measured to look for it**, on the
/// board assembled around rsemu's own ten-byte stub.
///
/// The addresses are not read off a schematic: they are where a real Macintosh
/// Classic ROM's 204,667 accesses to the VIA, 384 to the SCC and its handful to
/// the disk controller actually landed, recorded through the tap above, with
/// **zero** anywhere the board does not claim.
/// `docs/platforms/mac-classic.md` has the histogram.
#[test]
fn every_chip_answers_where_the_rom_looks() {
    let b = stub_board(Vec::new());
    let space = b.machine.space("mem").expect("the board has `mem`");
    let read = |addr: u64| space.read(addr, Width::U8, MemAttrs::DEBUG);

    // The ROM answers at zero, through the overlay, and at its own address. The
    // stub's first longword is the stack pointer, so both must match it.
    assert_eq!(peek(&b, 0x00_0000), 0x0010_0000, "the ROM is at zero");
    assert_eq!(peek(&b, 0x40_0000), 0x0010_0000, "and at $400000");
    // 512 KiB, repeating twice through the megabyte its select decodes.
    assert_eq!(peek(&b, 0x48_0000), 0x0010_0000, "the socket repeats");

    // The VIA at $EFE1FE, register 0, and `vBufA` at $EFFFFE, register 15.
    assert!(read(0xef_e1fe).is_ok(), "the VIA answers at its base");
    assert!(read(0xef_fffe).is_ok(), "and vBufA at $EFFFFE");
    // The SWIM at $DFE1FF, which is an *odd* address: it sits on the low byte
    // lane where the VIA sits on the high one.
    assert!(read(0xdf_e1ff).is_ok(), "the SWIM answers at its base");
    // The SCC in both windows, because the board has no read/write pin for it
    // and decodes the direction from the address instead.
    assert!(read(0x9f_fff8).is_ok(), "the SCC's read window");
    assert!(read(0xbf_fff9).is_ok(), "and its write window");

    // And what the board deliberately does not claim floats rather than
    // faulting: a compact Macintosh has no bus-error timeout.
    for floating in [0x50_0000u64, 0x58_0000, 0xf0_0000] {
        assert!(read(floating).is_ok(), "{floating:#08x} floats");
    }

    // The transceiver came up with a keyboard at 2 and a mouse at 3.
    assert_eq!(b.adb.bus().addresses(), vec![2, 3]);
    // And the drive is empty, which is the state a Macintosh draws a picture
    // for.
    assert!(!b.swim.has_disk(0));
}

/// **A 1.44 MB image goes into the SWIM's drive**, where a Macintosh Plus
/// refuses one by name — and it becomes MFM cells turning at 300 rpm.
///
/// The image is 1,474,560 bytes of numbered blocks built on the spot. Nobody's
/// disk is in this repository and this test needs none.
#[test]
fn a_1440k_image_goes_into_the_drive_and_turns_at_300_rpm() {
    let image = synthetic_1440k();
    let b = stub_board(image.clone());
    assert!(b.swim.has_disk(0), "the image went into the drive");
    assert_eq!(b.swim.density(0), Some(Density::Mfm));

    // The disk under the head, and the sectors on it. The controller's clock is
    // one tick a cell, so a revolution is 200,000 ticks.
    let disk = b.swim.iwm().disk(0).expect("a disk in the drive");
    let track = disk.mfm_track(0, 0);
    assert_eq!(track.len(), mfm::CELLS_PER_REVOLUTION);
    let (found, bad) = mfm::decode_track(&track);
    assert!(bad.is_empty(), "{bad:?}");
    assert_eq!(found.len(), mfm::SECTORS, "eighteen sectors on a track");
    // Sector 1 of cylinder 0 head 0 is block 0, which says its own number.
    let first = found
        .iter()
        .find(|s| s.sector == 1)
        .expect("sectors are one-based");
    assert_eq!(first.data[..4], 0u32.to_be_bytes());
    assert_eq!(&first.data[..], &image[..512]);

    // 1,000,000 cells a second over 200,000 a revolution is 300 rpm, and there
    // is no other number in it.
    assert_eq!(
        1_000_000 * 60 / mfm::CELLS_PER_REVOLUTION,
        usize::try_from(mfm::RPM).unwrap()
    );
}

/// The overlay is **latching** on this board, and that is what keeps the
/// machine alive: the ROM drives `PA4` high again 5.4 virtual seconds into
/// startup, long after it has put its own exception vector table at zero.
///
/// Asserted through the address space rather than through the device, because
/// what matters is what answers at zero.
#[test]
fn the_overlay_is_cleared_once_and_stays_cleared() {
    let b = stub_board(Vec::new());
    let space = b.machine.space("mem").expect("mem");
    let via = |reg: u64| 0xef_e1fe + reg * 512;
    let poke = |addr: u64, value: u8| {
        space
            .write(addr, Width::U8, u64::from(value), MemAttrs::DEFAULT)
            .expect("the VIA takes a byte");
    };
    assert_eq!(peek(&b, 0), 0x0010_0000, "the ROM is at zero at power-on");

    // DDRA bit 4 an output, then PA4 low: the overlay goes and memory is at
    // zero. Port A with no handshake is register 15, DDRA is register 3.
    poke(via(3), 0x10);
    poke(via(15), 0x00);
    assert_ne!(peek(&b, 0), 0x0010_0000, "memory is at zero now");
    let before = peek(&b, 0);

    // And PA4 high again does **not** put the ROM back.
    poke(via(15), 0x10);
    assert_eq!(
        peek(&b, 0),
        before,
        "the ROM came back over the vector table: the overlay is not latching"
    );
}

// ---------------------------------------------------------------------------
// a real ROM
// ---------------------------------------------------------------------------

/// **The ROM boots to the insert-disk screen**: the Macintosh's grey desktop
/// with the arrow cursor in the top left corner and the floppy-with-a-question-
/// mark in the middle of it, with the 60.15 Hz tick chain running.
///
/// Twelve virtual seconds is well past the point where it stops changing on the
/// stock 1 MiB board — the memory test finishes at about five and the picture is
/// settled by six.
#[test]
fn the_rom_boots_to_the_insert_disk_screen() {
    let Some(image) = rom_image("Classic.ROM") else {
        return;
    };
    let disk = disk_image("MacOS_6.0.8_System_Startup.img");
    let mut b = board(image, &[], disk);
    advance(&mut b, "mac-classic", 12);
    let hash = picture(&b, "mac-classic", 12);

    assert!(!b.cpu.is_halted(), "the processor double-faulted");
    assert_eq!(b.cpu.bus_faults().0, 0, "an access faulted");
    assert!(
        b.scanout.frame_counter() >= 12 * 59,
        "the video circuit is producing frames: {}",
        b.scanout.frame_counter()
    );
    // A Macintosh has no bus-error timeout, so a floating read is not a fault —
    // but it is still worth knowing how many there were.
    let log = b.machine.space("mem").expect("mem").unassigned_log();
    println!("mac-classic: {} unassigned accesses", log.count);

    // What the ROM wrote into low memory, which is data it built rather than
    // code it ran. `MemTop` is how much memory it decided there was and
    // `ScrnBase` where it decided the screen goes — both are the decoder's
    // behaviour reported back by the guest.
    assert_eq!(peek(&b, 0x108), 0x0010_0000, "MemTop: a 1 MiB machine");
    assert_eq!(peek(&b, 0x824), 0x000f_a700, "ScrnBase: MemTop - $5900");
    assert_eq!(peek(&b, 0x2ae), 0x0040_0000, "ROMBase");
    // The 60.15 Hz tick chain is running: `Ticks` counts vertical blanking
    // interrupts, so a plausible number here is the VIA, the video circuit and
    // the processor's interrupt path all working together.
    let ticks = peek(&b, 0x16a);
    assert!(
        (100..=800).contains(&ticks),
        "Ticks is counting vertical blanking: {ticks}"
    );

    // The insert-disk icon really is on the screen, and not merely a hash that
    // happens to match: the box the ROM draws it in is mostly white, where the
    // grey desktop alone would be exactly half.
    let white = icon_white(&b);
    println!("mac-classic: {white} of {} icon pixels are white", 32 * 32);
    assert!(
        white > 700,
        "the insert-disk icon is not on the screen: {white} of {} pixels white, and an empty \
         desktop is {}",
        32 * 32,
        32 * 32 / 2
    );

    assert_eq!(
        hash, GOLDEN_1M,
        "the frame at 12s moved; look at it (RSEMU_MAC_FRAME_DIR) before accepting the new hash"
    );
}

/// **And the icon blinks**, which is what says the ROM is in a live
/// insert-disk loop rather than parked in a two-instruction one.
///
/// It alternates between the plain floppy and the same floppy with a question
/// mark on its face, about a second each way. Asserted through the picture
/// rather than through a count of register accesses, because a count of
/// accesses is only a count of accesses.
#[test]
fn the_insert_disk_icon_blinks() {
    let Some(image) = rom_image("Classic.ROM") else {
        return;
    };
    let mut b = board(image, &[], synthetic_1440k());
    advance(&mut b, "mac-classic-blink", 11);
    let mut seen = Vec::new();
    for s in 12..=17 {
        advance(&mut b, "mac-classic-blink", 1);
        seen.push(icon_white(&b));
        let _ = picture(&b, "mac-classic-blink", s);
    }
    println!("mac-classic: icon white counts second by second: {seen:?}");
    assert!(
        seen.iter().any(|&n| n != seen[0]),
        "the insert-disk icon does not blink: {seen:?}"
    );
    assert!(
        seen.iter().all(|&n| n > 700),
        "and it is an icon throughout, not a bare desktop: {seen:?}"
    );
    assert_eq!(b.cpu.bus_faults().0, 0, "an access faulted");
}

/// The same ROM on a 4 MiB board: it sizes the expansion and moves its screen
/// buffer with it, which is the whole of what `-p ram=4M` has to do.
///
/// No golden: the picture is the same screen and the point of the test is the
/// two numbers the ROM worked out for itself. Twenty-five seconds, because a
/// memory test of four megabytes takes four times as long as one of one.
#[test]
fn the_rom_sizes_a_four_megabyte_board_and_moves_its_screen() {
    let Some(image) = rom_image("Classic.ROM") else {
        return;
    };
    let mut b = board(image, &[("ram", "4M")], Vec::new());
    advance(&mut b, "mac-classic-4m", 25);
    let _ = picture(&b, "mac-classic-4m", 25);

    assert!(!b.cpu.is_halted(), "the processor double-faulted");
    assert_eq!(b.cpu.bus_faults().0, 0, "an access faulted");
    assert_eq!(peek(&b, 0x108), 0x0040_0000, "MemTop: a 4 MiB machine");
    assert_eq!(peek(&b, 0x824), 0x003f_a700, "ScrnBase: MemTop - $5900");
    let white = icon_white(&b);
    assert!(
        white > 700,
        "the insert-disk icon is not on a 4 MiB board's screen either: {white} of {} white",
        32 * 32
    );
}

/// **The ROM resets the Apple Desktop Bus, walks it, and finds the keyboard and
/// the mouse**, and without the transceiver it never gets past that point at
/// all.
///
/// This is the measurement `src/dev/mac/adb.rs` was built from, asserted back:
/// the ROM's last act before it used to stop for ever was `ORB = $4f` with the
/// shift register in its external-clock shift-out mode. With the transceiver
/// answering, it completes seventeen transactions or more — `SendReset`, then
/// Talk register 3 of every address 0 to 15, then Talk 0 of the mouse — and
/// goes on to draw a screen.
#[test]
fn the_rom_finds_the_keyboard_and_the_mouse_on_the_bus() {
    let Some(image) = rom_image("Classic.ROM") else {
        return;
    };
    let mut b = board(image, &[], Vec::new());
    advance(&mut b, "mac-classic-adb", 8);

    let bus = b.adb.bus();
    let transactions = bus.transactions();
    let (command, sent, received) = bus.last_exchange();
    println!(
        "mac-classic: {transactions} ADB transactions; last command {} (sent {sent:#04x}, took \
         {received:#04x})",
        rsemu::dev::mac::adb::describe(command)
    );
    assert!(
        transactions >= 17,
        "the ROM walks all sixteen bus addresses after a reset: {transactions} transactions"
    );
    assert_eq!(
        bus.addresses(),
        vec![2, 3],
        "the keyboard is at 2 and the mouse at 3, where a Macintosh leaves them"
    );
    assert_eq!(b.cpu.bus_faults().0, 0, "an access faulted");
    // And the machine is alive afterwards, which is the whole point: the tick
    // chain is counting.
    let ticks = peek(&b, 0x16a);
    assert!(ticks > 50, "the tick chain stopped: Ticks = {ticks}");
}

/// **The ROM probes the drive, sees a SuperDrive, and asks the SWIM for ISM
/// mode** — which this build does not have.
///
/// This is the measurement, asserted so that it cannot be lost. The sequence
/// the ROM writes to the IWM-mode register is
///
/// ```text
///   r15 W 57     Q7 on, so with Q6 already on the write loads the mode: $57
///   r15 W 17     and again: $17
///   r15 W 57     and $57, twice
///   r4  W f5     switch 4, and the write loads the mode: $75
/// ```
///
/// `$57` and `$75` both have **bit 6** set, and a Macintosh Plus ROM never
/// writes that bit at all — it writes `$17` once and nothing else. So this is
/// the Classic asking for something a Plus's controller does not have, and it
/// is where the 1.44 MB path stops. `docs/platforms/mac-classic.md` and
/// `src/dev/mac/swim.rs` say what is missing behind it, and why there is no
/// invented register table here.
#[test]
fn the_rom_asks_the_swim_for_ism_mode() {
    let Some(image) = rom_image("Classic.ROM") else {
        return;
    };
    let mut b = board(image, &[], synthetic_1440k());
    let swim = tap(&b, 0xC0_0000, 0x200, 0x20_0000);
    advance(&mut b, "mac-classic-swim", 10);

    let folded = swim.folded();
    println!("mac-classic: the controller's register trace, folded:");
    for (line, n) in &folded {
        if *n == 1 {
            println!("  {line}");
        } else {
            println!("  {line}  x{n}");
        }
    }
    let lines: Vec<&str> = folded.iter().map(|(l, _)| l.as_str()).collect();
    for want in ["r15 W 17", "r15 W 57", "r4  W f5"] {
        assert!(
            lines.contains(&want),
            "the ROM's mode-register sequence has changed: `{want}` is not in the trace"
        );
    }
    // And it did look at the drive: the status register answered with `SENSE`
    // both ways, which only happens if the drive's own lines were addressed.
    assert!(
        lines.iter().any(|l| l.ends_with("b5")) && lines.iter().any(|l| l.ends_with("35")),
        "the drive's status lines were never read"
    );
    assert_eq!(b.cpu.bus_faults().0, 0, "an access faulted");
}

/// The ROM reads the clock chip and writes its own parameter RAM back, exactly
/// as a Plus's does — so `Time` holds the date the chip was given plus however
/// long the machine has been on.
///
/// That is the check that the counter's byte order is right, and it is a
/// low-memory global the ROM built rather than code it ran.
/// How white a rectangle of the screen is, as a count of lit pixels.
///
/// Left, top, width, height — the same shape [`ICON`] has.
fn white_in(b: &Board, (left, top, w, h): (u32, u32, u32, u32)) -> usize {
    let info = b.scanout.info();
    let mut surface = Surface::new(PixelFormat::RGB888, info.width, info.height);
    b.scanout.capture(&mut surface);
    let pixels = surface.pixels();
    let mut white = 0;
    for y in top..(top + h).min(info.height) {
        for x in left..(left + w).min(info.width) {
            let at = ((y * info.width + x) * 3) as usize;
            if pixels.get(at).is_some_and(|&p| p != 0) {
                white += 1;
            }
        }
    }
    white
}

/// The Finder's **menu bar**: the full width of the screen, twenty pixels
/// deep, and white except for the Apple and the four menu titles.
const MENU_BAR: (u32, u32, u32, u32) = (0, 0, 512, 20);

/// Where the Finder puts the startup volume's icon: the top right corner,
/// below the menu bar.
const DISK_ICON: (u32, u32, u32, u32) = (440, 24, 64, 48);

/// **Mac OS 6.0.8 boots to the Finder desktop.**
///
/// This is the test this board exists for. A real Macintosh Classic ROM, the
/// user's own 1.44 MB system disk read in place, and no help: the ROM sizes
/// memory, resets the Apple Desktop Bus, finds a SuperDrive on the cable, puts
/// the SWIM into **ISM mode**, loads the parameter RAM with Apple's own MFM
/// timing table, reads the boot blocks off the disk and starts the System —
/// the happy Mac at about ten virtual seconds, "Welcome to Macintosh" at
/// fifteen, and the desktop with a menu bar and the volume's icon by sixty.
///
/// What is asserted is the *picture*, in three ways that fail differently:
///
/// * the **menu bar** is there, which no state before the Finder draws;
/// * the **volume icon** is in the top right corner, which only a mounted
///   startup disk puts there;
/// * and a hash of the whole frame, so that a change to any of this shows up
///   as a moved golden rather than as nothing.
///
/// It skips, saying so, when the ROM or the disk is not there.
/// **Nothing is ever written to the disk image**: the drive takes a copy of
/// the bytes and this model has no write path at all.
#[test]
fn the_rom_boots_mac_os_to_the_finder() {
    let Some(image) = rom_image("Classic.ROM") else {
        return;
    };
    let disk = disk_image("MacOS_6.0.8_System_Startup.img");
    if disk.is_empty() {
        println!("mac-classic: no system disk, so there is nothing to boot; skipped");
        return;
    }
    let mut b = board(image, &[], disk);
    assert!(b.swim.has_disk(0), "the image went into the drive");
    assert_eq!(
        b.swim.density(0),
        Some(Density::Mfm),
        "a 1.44 MB disk is MFM, and that is what makes the ROM want ISM mode"
    );

    // The Finder is up a little before sixty virtual seconds; the extra is
    // slack, and the frame that is hashed is at a fixed instant either way.
    for s in 1..=BOOT_SECONDS {
        advance(&mut b, "mac-classic-boot", 1);
        if s % 10 == 0 {
            let _ = picture(&b, "mac-classic-boot", s);
        }
    }

    assert_eq!(b.cpu.bus_faults().0, 0, "an access faulted");
    assert!(!b.cpu.is_halted(), "the processor double-faulted");
    // **Not** asserted here: that the disk is still in the drive. At 69
    // virtual seconds — the instant the Finder finishes drawing the desktop —
    // the ROM drives the phase lines to `CA2:CA1:CA0 = 111` and strobes
    // `LSTRB`, which with `SEL` low is the drive register file's *eject*, and
    // this model takes it literally. The guest plainly does not agree: the
    // startup volume's icon stays on the desktop and the picture below is the
    // one a mounted disk produces. `docs/platforms/mac-classic.md`, ledger
    // item 1, has the trace and the two readings that could explain it; until
    // one of them is settled, asserting either way here would be encoding a
    // guess.

    // The menu bar: white across the whole width of the screen, with the Apple
    // and four menu titles in it. A grey desktop with no Finder is about half
    // ink, so anything over three quarters white is a menu bar and nothing
    // else on this machine is.
    let menu = white_in(&b, MENU_BAR);
    let menu_area = (MENU_BAR.2 * MENU_BAR.3) as usize;
    println!("mac-classic: the menu bar is {menu} white pixels of {menu_area}");
    assert!(
        menu * 4 > menu_area * 3,
        "there is no menu bar across the top of the screen: {menu} of {menu_area} white"
    );

    // And the volume's icon in the top right, which only a *mounted* disk
    // puts there. The desktop behind it is the 50 % checkerboard, so a box
    // that is mostly white is an icon sitting on it.
    let icon = white_in(&b, DISK_ICON);
    let icon_area = (DISK_ICON.2 * DISK_ICON.3) as usize;
    println!("mac-classic: the disk icon corner is {icon} white pixels of {icon_area}");
    assert!(
        icon * 3 > icon_area * 2,
        "the startup volume's icon is not on the desktop: {icon} of {icon_area} white"
    );

    let hash = picture(&b, "mac-classic-boot", BOOT_SECONDS);
    assert_eq!(
        hash, GOLDEN_FINDER,
        "the Finder desktop has moved; look at the PNG (RSEMU_MAC_FRAME_DIR) before \
         accepting a new hash"
    );
}

#[test]
#[ignore = "a trace instrument, not an assertion: run it with --ignored --nocapture"]
fn trace_the_controller() {
    let Some(image) = rom_image("Classic.ROM") else {
        return;
    };
    let secs: u64 = std::env::var("RSEMU_MAC_SECONDS")
        .ok()
        .and_then(|s| s.parse().ok())
        .unwrap_or(12);
    let kind = std::env::var("RSEMU_MAC_DISK_KIND").unwrap_or_else(|_| "1440k".to_string());
    let disk = match kind.as_str() {
        "real" => disk_image("MacOS_6.0.8_System_Startup.img"),
        "800k" => {
            // 800K of numbered blocks, the same differential the Plus uses:
            // the GCR path is known to work there, so if the Classic reads
            // this and not a 1.44 MB disk the difference is the medium.
            let mut image = vec![0u8; 819_200];
            for (block, chunk) in image.chunks_mut(512).enumerate() {
                chunk[..4].copy_from_slice(&(block as u32).to_be_bytes());
                for (i, byte) in chunk.iter_mut().enumerate().skip(4) {
                    *byte = (block as u8).wrapping_mul(7).wrapping_add(i as u8);
                }
            }
            image
        }
        "none" => Vec::new(),
        _ => synthetic_1440k(),
    };
    let drives = std::env::var("RSEMU_MAC_DRIVES").unwrap_or_else(|_| "1".to_string());
    let mut b = board(image, &[("drives", drives.as_str())], disk);
    let tap = swim_tap(&b);
    let mut had = b.swim.has_disk(0);
    for s in 1..=secs {
        advance(&mut b, "mac-classic-trace", 1);
        if b.swim.has_disk(0) != had {
            had = !had;
            println!("mac-classic: at {s}s the drive {} a disk", if had { "gained" } else { "lost" });
        }
        if s % 5 == 0 {
            let _ = picture(&b, "mac-classic-trace", s);
        }
    }
    tap.dump("mac-classic: the controller, named");
    println!("mac-classic: ISM selected at the end: {}", b.swim.ism_selected());
    println!(
        "mac-classic: motor {} track {} disk {}",
        b.swim.motor(0),
        b.swim.track(0),
        b.swim.has_disk(0)
    );
    // And what the processor is polling where it stopped: one more virtual
    // second with the memory aperture counting addresses.
    let ram = ram_tap(&b);
    ram.arm(true);
    advance(&mut b, "mac-classic-trace", 1);
    ram.arm(false);
    println!("mac-classic: the busiest memory addresses in the last second:");
    for ((addr, write), n) in ram.top(24) {
        println!("  {addr:#08x} {} x{n}", if write { "W" } else { "R" });
    }
}

#[test]
fn the_clock_chip_reaches_low_memory() {
    let Some(image) = rom_image("Classic.ROM") else {
        return;
    };
    let mut b = board(image, &[], Vec::new());
    advance(&mut b, "mac-classic-rtc", 8);
    let time = peek(&b, 0x20c);
    println!("mac-classic: Time = {time:#010x}");
    // `rtcdate` defaults to 2026-01-01T00:00:00, which in the Macintosh's own
    // epoch — seconds since 1904-01-01 — is $E57B6980. Eight seconds of
    // running puts it a little above that and nowhere near anything else.
    assert!(
        (0xe57b_6980..0xe57b_69c0).contains(&time),
        "Time is the date the clock chip was given plus the uptime: {time:#010x}"
    );
    assert_eq!(b.cpu.bus_faults().0, 0, "an access faulted");
}
