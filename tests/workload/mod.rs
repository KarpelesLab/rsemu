//! The committed workload set: what the frame-hash regression asserts and what
//! the benchmark measures.
//!
//! One module, two consumers. `tests/frame_hash.rs` runs each workload for a
//! fixed number of emulated frames and compares the state and framebuffer
//! hashes against `tests/goldens/frame-hashes.txt`; `benches/frame_time.rs`
//! runs the same workloads, re-checks the same goldens at the same frame, and
//! then keeps going with a stopwatch. Sharing the definitions is the point: a
//! benchmark that measures a different workload from the one the regression
//! pins is measuring nothing in particular, and a benchmark whose guest stopped
//! doing the work would otherwise look like a speedup.
//!
//! # Why every fixture is generated
//!
//! `ROADMAP.md` §13's phase-3 gate names *three commercial titles*, and
//! `CLAUDE.md` forbids committing or fetching them. Everything here is
//! therefore synthesised in this file — a NROM image assembled by the small
//! 6502 assembler below, a Game Boy cartridge from
//! [`rsemu::dev::gb::cart::synthetic_image`], hand-encoded RV64I, and the
//! Apple 1's own committed monitor. That makes the regression runnable in CI
//! with no download and no licence question, and it makes the benchmark
//! reproducible by anyone. It does **not** make it a substitute for the gate's
//! commercial titles; `benches/frame_time.rs` says so where a reader will see
//! it, and `RSEMU_BENCH_NES_ROM` points the NES workload at a user's own
//! cartridge for the measurement the gate actually asks for.
//!
//! # Determinism
//!
//! Every workload is a machine built from a generated image and advanced by a
//! fixed span of *virtual* time per frame. Nothing here reads the host clock,
//! seeds anything from the environment, or depends on how fast the host is —
//! the emulated work is bit-identical run to run, and only the wall-clock
//! measurement around it varies. [`Workload::run`] is what both consumers use,
//! so there is one definition of "a frame" rather than two.

// Two targets include this file and neither uses all of it: the regression does
// not care about frame *timing* and the benchmark does not care about the
// golden file's parser. Splitting it further to satisfy the lint would put the
// workload definitions somewhere neither consumer naturally reads.
#![allow(dead_code)]

use std::collections::BTreeMap;

use rsemu::core::clock::GlobalTime;
use rsemu::host::display::{PixelFormat, Scanout, Surface};
// `catalog` is imported by each boot function rather than here: in a build with
// no machine feature there are none, and an unused import is an error under
// CI's `-D warnings`.
use rsemu::machine::Machine;

/// Serialises everything in this module across the whole test binary.
///
/// Held for a [`Booted`]'s entire life, not just its construction — `cargo
/// test` runs tests in parallel threads of one process and two of the seams a
/// workload goes through are process-wide by design:
///
/// * `host::display::nes::capture` keeps constructed PPUs in a `static`, so one
///   test's `take` can land between another's build and its own.
/// * In a `no_std` build `core::sync` uses its `single` backend, whose locks
///   report a *concurrent* claim as a would-be deadlock rather than blocking —
///   which is the whole point of that checker (`ROADMAP.md` §0). Two machines
///   driven from two libtest threads reach the same process-wide `Global` and
///   trip it, and the panic reads like an emulator bug when it is a harness
///   one. That is why the guard lives in `Booted` rather than being dropped at
///   the end of `boot`.
///
/// `host::display::PROCESS_WIDE` and `tests/spi_panel.rs` document the same
/// hazard and take the same approach.
static SERIALISE: std::sync::Mutex<()> = std::sync::Mutex::new(());

/// Take [`SERIALISE`], ignoring poisoning.
///
/// A test that panicked while holding it has already failed and reported why;
/// turning every *other* test into a second failure about a poisoned mutex
/// buries the one that matters.
fn serialise() -> std::sync::MutexGuard<'static, ()> {
    SERIALISE.lock().unwrap_or_else(|e| e.into_inner())
}

/// Points the NES workload at a cartridge the person running it owns.
///
/// The phase-3 gate names three commercial titles; this is the only lawful way
/// for this repository to run one. It changes what is measured, so the golden
/// file cannot apply — `tests/frame_hash.rs` refuses to compare when it is set.
pub(crate) const NES_ROM_ENV: &str = "RSEMU_BENCH_NES_ROM";

/// The cartridge path [`NES_ROM_ENV`] names, if it names one.
pub(crate) fn nes_rom_override() -> Option<String> {
    std::env::var(NES_ROM_ENV).ok().filter(|p| !p.is_empty())
}

/// How much virtual time one "frame" is for a machine that has no display.
///
/// Sixty per virtual second, so the fps and ×real-time figures the benchmark
/// prints mean the same thing for every row of the table.
const NOMINAL_FRAME_NS: u64 = 16_666_667;

// ---------------------------------------------------------------------------
// the workload table
// ---------------------------------------------------------------------------

/// One committed workload: a machine, an image to run on it, and how far.
pub(crate) struct Workload {
    /// The name used in the golden file and in the benchmark's table.
    pub(crate) name: &'static str,
    /// One line saying what the guest actually does, for the report.
    pub(crate) what: &'static str,
    /// Frames the regression runs, and the point the golden hashes describe.
    ///
    /// Also the benchmark's warm-up: samples before this frame are discarded,
    /// which conveniently drops each guest's setup phase as well as a cold
    /// instruction cache.
    pub(crate) frames: u32,
    /// How often the regression records a framebuffer hash.
    pub(crate) checkpoint_every: u32,
    /// Builds the machine. Boxed because each arm captures a different image.
    build: fn() -> Booted,
}

/// A built machine plus whatever can look at its picture.
///
/// Not `Send`: it carries [`SERIALISE`]'s guard, so only one of these exists in
/// the process at a time and it never leaves the thread that built it. Drop it
/// before booting the next one.
pub(crate) struct Booted {
    /// The machine itself.
    pub(crate) machine: Machine,
    /// Its display, if this build has an adapter for one.
    pub(crate) capture: Option<Capture>,
    /// Virtual time in one frame.
    span: GlobalTime,
    /// Kept, not used. See [`SERIALISE`].
    _guard: std::sync::MutexGuard<'static, ()>,
}

impl Booted {
    fn wrap(
        guard: std::sync::MutexGuard<'static, ()>,
        machine: Machine,
        scanout: Option<Box<dyn Scanout>>,
    ) -> Booted {
        let capture = scanout.map(Capture::new);
        // A device that publishes no rate — or no device at all — gets the
        // nominal 60 Hz, so "one frame" means one comparable thing across the
        // whole table.
        let ns = capture
            .as_ref()
            .map(Capture::frame_period_ns)
            .filter(|ns| *ns != 0)
            .unwrap_or(NOMINAL_FRAME_NS);
        Booted {
            machine,
            capture,
            span: GlobalTime::from_nanos(ns),
            _guard: guard,
        }
    }

    /// Advance exactly one frame of virtual time.
    ///
    /// # Panics
    ///
    /// If the machine faults, which is a bug in the fixture or the crate.
    pub(crate) fn step(&mut self) {
        self.machine.run_for(self.span).expect("the machine runs");
    }

    /// Advance `frames` frames.
    ///
    /// # Panics
    ///
    /// As [`step`](Self::step).
    pub(crate) fn step_many(&mut self, frames: u32) {
        for _ in 0..frames {
            self.step();
        }
    }

    /// Virtual nanoseconds one [`step`](Self::step) covers.
    pub(crate) fn frame_period_ns(&self) -> u64 {
        self.span.as_nanos()
    }
}

/// Every workload this build can run, in a stable order.
///
/// Empty in a build with no machine features, which is a build with nothing to
/// measure rather than a failure.
// Not a `vec![]`: every push below is `#[cfg]`-gated, and a literal cannot have
// its elements compiled out one at a time. In a build with no machine feature
// there are no pushes left, so the binding is not mutated either.
#[allow(clippy::vec_init_then_push, unused_mut)]
pub(crate) fn all() -> Vec<Workload> {
    let mut out: Vec<Workload> = Vec::new();

    #[cfg(all(feature = "machine-nes", feature = "dev-nes-ppu"))]
    out.push(Workload {
        name: "nes-ntsc",
        what: "full-screen background, 64 sprites, scrolling, APU on, \
               a 256-byte read-modify-write loop in WRAM",
        // One virtual second. Long enough that the ROM is past its two-vblank
        // warm-up, its palette upload, its nametable fill and its OAM fill, and
        // well into the steady state the gate is actually about; short enough
        // that the whole file runs in the debug profile on every commit. Four
        // checkpoints, because a divergence that shows up only at the last one
        // is a different bug from one that shows up at the first.
        frames: 60,
        checkpoint_every: 15,
        build: boot_nes,
    });

    #[cfg(feature = "machine-gameboy")]
    out.push(Workload {
        name: "gameboy",
        what: "LCD on with a filled tile map, scrolling, \
               a 4 KiB read-modify-write loop in WRAM",
        frames: 60,
        checkpoint_every: 15,
        build: boot_gameboy,
    });

    #[cfg(feature = "machine-apple1")]
    out.push(Workload {
        name: "apple1",
        what: "RSMON at its prompt: a 6502 polling the PIA, no display device",
        frames: 60,
        checkpoint_every: 30,
        build: boot_apple1,
    });

    #[cfg(feature = "machine-riscv-virt")]
    out.push(Workload {
        name: "riscv-virt",
        what: "an RV64I integer loop through DRAM: add, store, load, shift",
        frames: 60,
        checkpoint_every: 30,
        build: boot_riscv_virt,
    });

    out
}

impl Workload {
    /// Build the machine and everything needed to look at it.
    ///
    /// Serialised against every other boot in the process: two of the seams
    /// involved keep process-wide tables.
    ///
    /// # Panics
    ///
    /// If the machine will not build — a bug in this file or in the crate,
    /// never in the caller.
    pub(crate) fn boot(&self) -> Booted {
        (self.build)()
    }

    /// Boot, run `frames` frames, and call `checkpoint` after each one.
    ///
    /// The closure is handed the frame number (1-based) and the booted machine.
    /// Returning `false` stops the run early, which is how the benchmark ends
    /// on a sample count rather than a frame count.
    ///
    /// # Panics
    ///
    /// As [`boot`](Self::boot), or if the machine faults.
    pub(crate) fn run<F>(&self, frames: u32, mut checkpoint: F)
    where
        F: FnMut(u32, &mut Booted) -> bool,
    {
        let mut booted = self.boot();
        for frame in 1..=frames {
            booted.step();
            if !checkpoint(frame, &mut booted) {
                break;
            }
        }
    }
}

// ---------------------------------------------------------------------------
// looking at the picture
// ---------------------------------------------------------------------------

/// A scanout and the surface it is captured into, kept together so a caller
/// does not reallocate the surface every frame.
pub(crate) struct Capture {
    scanout: Box<dyn Scanout>,
    surface: Surface,
}

impl Capture {
    fn new(scanout: Box<dyn Scanout>) -> Capture {
        let info = scanout.info();
        let surface = Surface::new(PixelFormat::RGBA8888, info.width, info.height);
        Capture { scanout, surface }
    }

    /// Virtual nanoseconds in one of this device's frames.
    fn frame_period_ns(&self) -> u64 {
        self.scanout.frame_period_ns()
    }

    /// Frames the device has completed since reset.
    pub(crate) fn frame_counter(&self) -> u64 {
        self.scanout.frame_counter()
    }

    /// Capture the current frame and hash it.
    pub(crate) fn hash(&mut self) -> u64 {
        self.scanout.capture(&mut self.surface);
        self.surface.hash()
    }

    /// The most recently captured frame, for a caller that wants the pixels
    /// rather than a hash — a PNG for `docs/`, say.
    pub(crate) fn surface(&self) -> &Surface {
        &self.surface
    }

    /// How many distinct pixel values the current frame holds.
    ///
    /// The regression's guard against a guest that silently stopped doing the
    /// work: a blank screen has a perfectly stable hash, so pinning the hash
    /// alone would happily bless a ROM that stopped rendering.
    pub(crate) fn distinct_colours(&mut self) -> usize {
        self.scanout.capture(&mut self.surface);
        let mut seen: BTreeMap<[u8; 4], ()> = BTreeMap::new();
        for pixel in self.surface.pixels().as_chunks::<4>().0 {
            seen.insert(*pixel, ());
        }
        seen.len()
    }
}

// ---------------------------------------------------------------------------
// the machines
// ---------------------------------------------------------------------------

#[cfg(all(feature = "machine-nes", feature = "dev-nes-ppu"))]
fn boot_nes() -> Booted {
    use rsemu::host::display::nes::capture;
    use rsemu::machine::catalog;

    let image = nes_rom();
    let guard = serialise();
    let entry = catalog::machine("nes-ntsc").expect("this build ships nes-ntsc");
    let mut options = catalog::build_options().expect("the catalog agrees with itself");
    options.realize.media.insert("cart", image.as_slice());
    capture::clear();
    capture::install(&mut options).expect("the interception installs");
    let registry = catalog::registry().expect("a registry");
    let machine = rsemu::machine::build(entry.name, entry.source, &registry, &options)
        .expect("the NES realizes");
    let scanout = capture::take().expect("the machine has a PPU");
    Booted::wrap(guard, machine, Some(Box::new(scanout)))
}

#[cfg(feature = "machine-gameboy")]
fn boot_gameboy() -> Booted {
    use rsemu::machine::catalog;

    // ROM only, no cartridge RAM: the smallest cartridge that can hold the
    // program below, so the measurement is of the console rather than a mapper.
    let image = rsemu::dev::gb::cart::synthetic_image(2, 0x00, 0x00, GAMEBOY_PROGRAM);
    let guard = serialise();
    let machine = catalog::build_catalog("gameboy", &[("cart", &image)]).expect("the GB realizes");
    // `host::display` has no Game Boy adapter yet, so there is nothing to hash
    // a frame from — the state hash is the whole regression here.
    Booted::wrap(guard, machine, None)
}

#[cfg(feature = "machine-apple1")]
fn boot_apple1() -> Booted {
    use rsemu::machine::catalog;

    let guard = serialise();
    let machine = catalog::build_catalog("apple1", &[("rom", rsemu::dev::apple1::RSMON)])
        .expect("the Apple 1 realizes");
    Booted::wrap(guard, machine, None)
}

#[cfg(feature = "machine-riscv-virt")]
fn boot_riscv_virt() -> Booted {
    use rsemu::machine::catalog;

    let firmware = riscv_firmware();
    let guard = serialise();
    let entry = catalog::machine("riscv-virt").expect("this build ships riscv-virt");
    let mut options = catalog::build_options().expect("the catalog agrees with itself");
    options
        .realize
        .media
        .insert("firmware", firmware.as_slice());
    // Both NOR banks erased, as a board with blank parts soldered on.
    options.realize.media.insert("flash0", &[][..]);
    options.realize.media.insert("flash1", &[][..]);
    // The board also names `disk` and `initrd`. Both default to empty here for
    // the same reason `rsemu run` defaults them: a slot a machine file names
    // must be bound or realize refuses, and this workload wants neither a root
    // filesystem nor a ramdisk.
    options.realize.media.insert("disk", &[][..]);
    options.realize.media.insert("initrd", &[][..]);
    // 16 MiB instead of the board's 128. The program below touches one word, so
    // the extra 112 MiB buys nothing and costs a great deal: `state_hash` walks
    // every byte of RAM, and the regression takes one every few frames. Sizing
    // the board to the workload is what makes this cheap enough to run on every
    // commit.
    options
        .resolve
        .params
        .push((String::from("ram"), String::from("16M")));
    let registry = catalog::registry().expect("a registry");
    let machine = rsemu::machine::build(entry.name, entry.source, &registry, &options)
        .expect("the virt board realizes");
    Booted::wrap(guard, machine, None)
}

// ---------------------------------------------------------------------------
// the NES image
// ---------------------------------------------------------------------------

/// A synthetic NROM cartridge whose program renders a full screen.
///
/// The gate's workload in miniature, and deliberately not a minimal one: a
/// background covering every tile, sprites on most scanlines, a scroll register
/// written every vblank so no two frames are identical, the APU's channels
/// running, and a read-modify-write loop over 256 bytes of WRAM so the CPU is
/// doing bus work rather than spinning on a branch. What it is *not* is a game:
/// see the module docs on the commercial-title half of the gate.
///
/// Sources: NESdev wiki — "PPU registers", "PPU power up state", "Init code",
/// "APU registers", "NROM". No emulator source was consulted (`CLAUDE.md`).
#[cfg(all(feature = "machine-nes", feature = "dev-nes-ppu"))]
fn nes_rom() -> Vec<u8> {
    // A user's own cartridge, when they point us at one. The gate names three
    // commercial titles and this is the only legal way for us to run them: the
    // bytes come from the person running the benchmark and are never fetched,
    // committed, or redistributed. A path that cannot be read is an error
    // rather than a quiet fall-back — otherwise a typo turns into a benchmark
    // of the synthetic ROM wearing the user's label.
    if let Some(path) = nes_rom_override() {
        return std::fs::read(&path).unwrap_or_else(|e| panic!("{NES_ROM_ENV}={path}: {e}"));
    }

    let mut asm = Asm6502::new(0xc000);
    asm.emit(&[0x78]); //             SEI
    asm.emit(&[0xd8]); //             CLD
    asm.emit(&[0xa2, 0xff]); //       LDX #$ff
    asm.emit(&[0x9a]); //             TXS
    asm.emit(&[0xa9, 0x00]); //       LDA #$00
    asm.emit(&[0x8d, 0x00, 0x20]); // STA $2000   NMI off while we set up
    asm.emit(&[0x8d, 0x01, 0x20]); // STA $2001   rendering off

    // Two vblanks: the 2C02 ignores $2000/$2001/$2005/$2006 until it has warmed
    // up, and waiting for the flag twice is what every cartridge does.
    asm.label("vbl1");
    asm.emit(&[0x2c, 0x02, 0x20]); // BIT $2002
    asm.branch(0x10, "vbl1"); //      BPL vbl1
    asm.label("vbl2");
    asm.emit(&[0x2c, 0x02, 0x20]); // BIT $2002
    asm.branch(0x10, "vbl2"); //      BPL vbl2

    // The 32 palette entries at $3f00, one write each.
    asm.emit(&[0xa9, 0x3f]); //       LDA #$3f
    asm.emit(&[0x8d, 0x06, 0x20]); // STA $2006
    asm.emit(&[0xa9, 0x00]); //       LDA #$00
    asm.emit(&[0x8d, 0x06, 0x20]); // STA $2006
    asm.emit(&[0xa2, 0x00]); //       LDX #$00
    asm.label("pal");
    asm.emit(&[0x8a]); //             TXA
    asm.emit(&[0x8d, 0x07, 0x20]); // STA $2007
    asm.emit(&[0xe8]); //             INX
    asm.emit(&[0xe0, 0x20]); //       CPX #$20
    asm.branch(0xd0, "pal"); //       BNE pal

    // The whole first nametable, 1 KiB, with a tile index that changes every
    // byte so the fetch path sees varied pattern data rather than one tile.
    asm.emit(&[0xa9, 0x20]); //       LDA #$20
    asm.emit(&[0x8d, 0x06, 0x20]); // STA $2006
    asm.emit(&[0xa9, 0x00]); //       LDA #$00
    asm.emit(&[0x8d, 0x06, 0x20]); // STA $2006
    asm.emit(&[0xa2, 0x04]); //       LDX #$04   four pages of 256
    asm.emit(&[0xa0, 0x00]); //       LDY #$00
    asm.label("nt");
    asm.emit(&[0x98]); //             TYA
    asm.emit(&[0x8d, 0x07, 0x20]); // STA $2007
    asm.emit(&[0xc8]); //             INY
    asm.branch(0xd0, "nt"); //        BNE nt
    asm.emit(&[0xca]); //             DEX
    asm.branch(0xd0, "nt"); //        BNE nt

    // All 64 objects, written a byte at a time through $2004. Y climbs by four
    // per object, so two or three sprites land on most scanlines and the
    // per-scanline evaluation actually has work to do.
    asm.emit(&[0xa9, 0x00]); //       LDA #$00
    asm.emit(&[0x8d, 0x03, 0x20]); // STA $2003
    asm.emit(&[0xa2, 0x00]); //       LDX #$00
    asm.label("oam");
    asm.emit(&[0x8a]); //             TXA
    asm.emit(&[0x8d, 0x04, 0x20]); // STA $2004
    asm.emit(&[0xe8]); //             INX
    asm.branch(0xd0, "oam"); //       BNE oam

    // The APU: all four channels enabled with something to play, so the mixer
    // and the frame counter are part of the measurement rather than idle.
    for (reg, value) in [
        (0x4015u16, 0x0fu8), // pulse 1+2, triangle, noise
        (0x4000, 0xbf),      // pulse 1: duty 2, constant volume 15
        (0x4001, 0x08),      // sweep off
        (0x4002, 0x40),
        (0x4003, 0x08),
        (0x4004, 0x7f), // pulse 2
        (0x4005, 0x08),
        (0x4006, 0x91),
        (0x4007, 0x08),
        (0x4008, 0xff), // triangle: linear counter loaded and held
        (0x400a, 0x30),
        (0x400b, 0x08),
        (0x400c, 0x3f), // noise
        (0x400e, 0x05),
        (0x400f, 0x08),
    ] {
        asm.emit(&[0xa9, value]);
        asm.emit(&[0x8d, (reg & 0xff) as u8, (reg >> 8) as u8]);
    }

    // Rendering on, then NMI on. $2001 = $1e is background and sprites with no
    // left-column clipping — the most expensive configuration the chip has, and
    // the one a game is in.
    asm.emit(&[0xa9, 0x1e]); //       LDA #$1e
    asm.emit(&[0x8d, 0x01, 0x20]); // STA $2001
    asm.emit(&[0xa9, 0x90]); //       LDA #$90   NMI on, background from $1000
    asm.emit(&[0x8d, 0x00, 0x20]); // STA $2000

    // The main loop: a read-modify-write pass over $0200-$02ff. Not busy-waiting
    // on a branch, because a benchmark whose CPU does nothing measures the PPU
    // alone and then claims to be a frame time.
    asm.label("main");
    asm.emit(&[0xa2, 0x00]); //       LDX #$00
    asm.label("work");
    asm.emit(&[0xbd, 0x00, 0x02]); // LDA $0200,X
    asm.emit(&[0x18]); //             CLC
    asm.emit(&[0x69, 0x07]); //       ADC #$07
    asm.emit(&[0x9d, 0x00, 0x02]); // STA $0200,X
    asm.emit(&[0xe8]); //             INX
    asm.branch(0xd0, "work"); //      BNE work
    asm.emit(&[0x4c, 0x00, 0x00]); // JMP main — patched below
    let jmp_operand = asm.here() - 2;

    // The NMI handler: scroll by one column per frame and count the frame. The
    // scroll write is what makes consecutive frames differ, so a frame hash
    // that stops changing is a real failure rather than a static picture.
    asm.label("nmi");
    asm.emit(&[0x48]); //             PHA
    asm.emit(&[0x2c, 0x02, 0x20]); // BIT $2002   reset the $2005 write latch
    asm.emit(&[0xa5, 0x10]); //       LDA $10
    asm.emit(&[0x8d, 0x05, 0x20]); // STA $2005   scroll X
    asm.emit(&[0xa9, 0x00]); //       LDA #$00
    asm.emit(&[0x8d, 0x05, 0x20]); // STA $2005   scroll Y
    asm.emit(&[0xe6, 0x10]); //       INC $10
    asm.emit(&[0x68]); //             PLA
    asm.emit(&[0x40]); //             RTI
    asm.label("irq");
    asm.emit(&[0x40]); //             RTI

    let (mut prg, labels) = asm.finish();
    let main = labels["main"];
    prg[jmp_operand] = (main & 0xff) as u8;
    prg[jmp_operand + 1] = (main >> 8) as u8;
    assert!(prg.len() < 0x3ffa, "the program collides with the vectors");
    prg.resize(0x4000, 0xea);
    let vector = |name: &str| {
        let addr = labels[name];
        [(addr & 0xff) as u8, (addr >> 8) as u8]
    };
    prg[0x3ffa..0x3ffc].copy_from_slice(&vector("nmi"));
    prg[0x3ffc..0x3ffe].copy_from_slice(&[0x00, 0xc0]);
    prg[0x3ffe..0x4000].copy_from_slice(&vector("irq"));

    // 8 KiB of pattern data. A cheap mixing function rather than a picture: what
    // matters is that no two adjacent tiles are alike, so the background fetch
    // produces a busy frame instead of a flat one.
    let mut chr = vec![0u8; 8192];
    for (i, byte) in chr.iter_mut().enumerate() {
        let i = i as u32;
        *byte =
            ((i.wrapping_mul(37) ^ (i >> 4).wrapping_mul(151)).wrapping_add(i >> 9) & 0xff) as u8;
    }

    let mut image = Vec::with_capacity(16 + prg.len() + chr.len());
    image.extend_from_slice(&[b'N', b'E', b'S', 0x1a, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]);
    image.extend_from_slice(&prg);
    image.extend_from_slice(&chr);
    image
}

/// The smallest 6502 assembler that keeps the program above readable.
///
/// Hand-computed branch offsets are how a synthetic ROM ends up quietly running
/// the wrong loop, and the failure then looks like an emulator bug. Two passes
/// over a label table costs forty lines and removes the whole class.
#[cfg(all(feature = "machine-nes", feature = "dev-nes-ppu"))]
struct Asm6502 {
    org: u16,
    bytes: Vec<u8>,
    labels: BTreeMap<&'static str, u16>,
    fixups: Vec<(usize, &'static str)>,
}

#[cfg(all(feature = "machine-nes", feature = "dev-nes-ppu"))]
impl Asm6502 {
    fn new(org: u16) -> Asm6502 {
        Asm6502 {
            org,
            bytes: Vec::new(),
            labels: BTreeMap::new(),
            fixups: Vec::new(),
        }
    }

    /// The address the next byte will be assembled at.
    fn pc(&self) -> u16 {
        self.org.wrapping_add(self.bytes.len() as u16)
    }

    /// The offset within the PRG image the next byte will be written at.
    fn here(&self) -> usize {
        self.bytes.len()
    }

    fn label(&mut self, name: &'static str) {
        let pc = self.pc();
        self.labels.insert(name, pc);
    }

    fn emit(&mut self, bytes: &[u8]) {
        self.bytes.extend_from_slice(bytes);
    }

    /// A relative branch whose target is resolved in [`finish`](Self::finish).
    fn branch(&mut self, opcode: u8, target: &'static str) {
        self.bytes.push(opcode);
        let at = self.bytes.len();
        self.bytes.push(0);
        self.fixups.push((at, target));
    }

    fn finish(mut self) -> (Vec<u8>, BTreeMap<&'static str, u16>) {
        for (at, target) in &self.fixups {
            let to = i32::from(*self.labels.get(target).expect("a branch to a real label"));
            let from = i32::from(self.org) + *at as i32 + 1;
            let offset = to - from;
            assert!(
                (-128..=127).contains(&offset),
                "branch to `{target}` is {offset} bytes, out of a 6502 branch's reach"
            );
            self.bytes[*at] = offset as i8 as u8;
        }
        (self.bytes, self.labels)
    }
}

// ---------------------------------------------------------------------------
// the Game Boy image
// ---------------------------------------------------------------------------

/// The program `synthetic_image` assembles at `$0150`.
///
/// Fills a few tiles and the visible half of the first tile map with rendering
/// off, turns the LCD on, then loops: a read-modify-write pass over 4 KiB of
/// work RAM and one write to `SCY`, so the picture moves and the CPU is doing
/// bus work. Sources: Pan Docs — "LCD Control", "Palettes", "VRAM Tile Data".
///
/// Written out as bytes rather than assembled, because at fifty-four of them a
/// table with the mnemonics beside it is the clearer artefact.
#[cfg(feature = "machine-gameboy")]
const GAMEBOY_PROGRAM: &[u8] = &[
    0x21, 0x00, 0x80, // $0150  LD   HL,$8000    tile data
    0x0e, 0x00, //       $0153  LD   C,$00       256 bytes
    0x3e, 0x00, //       $0155  LD   A,$00
    0x77, //             $0157  LD   (HL),A      <- fill
    0x3c, //             $0158  INC  A
    0x23, //             $0159  INC  HL
    0x0d, //             $015a  DEC  C
    0x20, 0xfa, //       $015b  JR   NZ,fill
    0x21, 0x00, 0x98, // $015d  LD   HL,$9800    tile map
    0x0e, 0x00, //       $0160  LD   C,$00
    0x3e, 0x00, //       $0162  LD   A,$00
    0x77, //             $0164  LD   (HL),A      <- map
    0x3c, //             $0165  INC  A
    0x23, //             $0166  INC  HL
    0x0d, //             $0167  DEC  C
    0x20, 0xfa, //       $0168  JR   NZ,map
    0x3e, 0xe4, //       $016a  LD   A,$e4
    0xe0, 0x47, //       $016c  LDH  ($47),A     BGP
    0x3e, 0x91, //       $016e  LD   A,$91
    0xe0, 0x40, //       $0170  LDH  ($40),A     LCDC: on, BG on, tiles at $8000
    0x21, 0x00, 0xc0, // $0172  LD   HL,$c000    <- main
    0x7e, //             $0175  LD   A,(HL)      <- work
    0xc6, 0x07, //       $0176  ADD  A,$07
    0x77, //             $0178  LD   (HL),A
    0x23, //             $0179  INC  HL
    0x7c, //             $017a  LD   A,H
    0xfe, 0xd0, //       $017b  CP   $d0
    0x20, 0xf6, //       $017d  JR   NZ,work
    0xf0, 0x42, //       $017f  LDH  A,($42)     SCY
    0x3c, //             $0181  INC  A
    0xe0, 0x42, //       $0182  LDH  ($42),A
    0x18, 0xec, //       $0184  JR   main
];

// ---------------------------------------------------------------------------
// the RISC-V image
// ---------------------------------------------------------------------------

/// An RV64I loop for the `virt` board's firmware slot, loaded at `0x8000_0000`.
///
/// Add, store, load and two shifts per iteration, with the loop closed by
/// `jalr` through a register the first two instructions compute. Every encoding
/// here is I-type or R-type on purpose: the J- and B-type immediates are
/// scrambled across the word, and hand-encoding one is the single most likely
/// way for a fixture like this to end up branching somewhere plausible and
/// wrong. Source: *The RISC-V Instruction Set Manual, Volume I*, chapter 2.
#[cfg(feature = "machine-riscv-virt")]
fn riscv_firmware() -> Vec<u8> {
    // x5 = t0 accumulator, x6 = t1 = 1, x7 = t2 = scratch address,
    // x28 = t3 reload, x29 = t4 shifted, x30 = t5 = the loop address.
    const PROGRAM: [u32; 12] = [
        0x0000_0f17, // auipc t5, 0        t5 = 0x80000000
        0x014f_0f13, // addi  t5, t5, 20   t5 = loop
        0x0000_1397, // auipc t2, 1        t2 = 0x80001008, a scratch word in DRAM
        0x0000_0293, // addi  t0, x0, 0
        0x0010_0313, // addi  t1, x0, 1
        0x0062_82b3, // loop: add t0, t0, t1
        0x0053_b023, // sd    t0, 0(t2)
        0x0003_be03, // ld    t3, 0(t2)
        0x003e_1e93, // slli  t4, t3, 3
        0x003e_de93, // srli  t4, t4, 3
        0x01d2_82b3, // add   t0, t0, t4
        0x000f_0067, // jalr  x0, 0(t5)
    ];
    PROGRAM.iter().flat_map(|w| w.to_le_bytes()).collect()
}

// ---------------------------------------------------------------------------
// the golden file
// ---------------------------------------------------------------------------

/// Where the regression's expected hashes live, relative to the manifest.
pub(crate) const GOLDEN_PATH: &str = "tests/goldens/frame-hashes.txt";

/// One recorded checkpoint: what a workload hashed to after `frame` frames.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) struct Golden {
    /// Frames run when this was recorded.
    pub(crate) frame: u32,
    /// [`Machine::state_hash`] at that point.
    pub(crate) state: u64,
    /// [`Surface::hash`] of the frame captured there, if the machine has a
    /// display in this build.
    pub(crate) frame_hash: Option<u64>,
}

/// The golden file, as `workload -> checkpoints`.
///
/// # Panics
///
/// If the file is missing or malformed. Both mean the repository is broken, not
/// the machine under test.
pub(crate) fn goldens() -> BTreeMap<String, Vec<Golden>> {
    let path = golden_file();
    let text = std::fs::read_to_string(&path).unwrap_or_else(|e| panic!("{}: {e}", path.display()));
    let mut out: BTreeMap<String, Vec<Golden>> = BTreeMap::new();
    for (n, line) in text.lines().enumerate() {
        let line = line.split('#').next().unwrap_or("").trim();
        if line.is_empty() {
            continue;
        }
        let mut field = line.split_whitespace();
        let mut next = |what: &str| {
            field
                .next()
                .unwrap_or_else(|| panic!("{}:{}: expected {what}", path.display(), n + 1))
        };
        let name = next("a workload name").to_owned();
        let frame = next("a frame number").parse().expect("a frame number");
        let state = parse_hash(next("a state hash"));
        let frame_hash = match next("a frame hash or `-`") {
            "-" => None,
            hex => Some(parse_hash(hex)),
        };
        out.entry(name).or_default().push(Golden {
            frame,
            state,
            frame_hash,
        });
    }
    out
}

/// Rewrite the golden file, keeping the rows for workloads this build cannot
/// run.
///
/// Merging rather than truncating is what lets the feature sweep bless a single
/// machine without silently deleting the rest of the table.
///
/// # Panics
///
/// If the file cannot be written.
pub(crate) fn bless(fresh: &BTreeMap<String, Vec<Golden>>) {
    let mut merged = goldens();
    for (name, rows) in fresh {
        merged.insert(name.clone(), rows.clone());
    }
    let mut text = String::from(GOLDEN_HEADER);
    for (name, rows) in &merged {
        for row in rows {
            let frame_hash = match row.frame_hash {
                Some(h) => format!("{h:#018x}"),
                None => String::from("-"),
            };
            text.push_str(&format!(
                "{name:<12} {:>5} {:#018x} {frame_hash}\n",
                row.frame, row.state
            ));
        }
    }
    let path = golden_file();
    std::fs::write(&path, text).unwrap_or_else(|e| panic!("{}: {e}", path.display()));
    eprintln!("blessed {}", path.display());
}

/// The absolute path of the golden file.
fn golden_file() -> std::path::PathBuf {
    std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join(GOLDEN_PATH)
}

fn parse_hash(text: &str) -> u64 {
    let hex = text.strip_prefix("0x").unwrap_or(text);
    u64::from_str_radix(hex, 16).unwrap_or_else(|e| panic!("`{text}` is not a hash: {e}"))
}

/// The comment the blessed file is written with.
const GOLDEN_HEADER: &str = "\
# Frame and state hashes for the committed workloads (ROADMAP.md §12).
#
# Columns: workload, frames run, Machine::state_hash, Surface::hash — the last
# `-` for a machine this build has no scanout adapter for.
#
# These are generated. If a change to an emulated device moved one of them, that
# is the regression doing its job: confirm the new behaviour is the behaviour
# you meant, then re-record with
#
#   RSEMU_BLESS_FRAME_HASHES=1 cargo test --all-features --test frame_hash
#
# and say in the commit message which device changed and why. A mismatch is
# never fixed by widening the test.
#
# The workloads themselves are generated too — see tests/workload/mod.rs. None
# of them is a commercial title, which is why the phase-3 fps gate is reported
# as unmet for licensing reasons rather than measured against one.
";
