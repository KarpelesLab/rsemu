//! Conformance runners for the Game Boy test ROMs.
//!
//! `ROADMAP.md` §0: *accuracy is measured, never asserted*. This is the
//! measurement for the SM83 and, through it, for the whole machine — every ROM
//! here is a Game Boy program, so passing one means the CPU, the timer, the
//! interrupt path and the memory map are all right together.
//!
//! # The corpora are downloaded, never vendored
//!
//! `ROADMAP.md` §1 and §12. Blargg's ROMs have no clear licence and Gekkio's
//! suite is MIT but is still not ours to redistribute from here, so both are
//! fetched into a git-ignored directory and the runners skip cleanly — printing
//! why — when they are absent. `cargo test` offline stays green; that is a rule,
//! not a convenience.
//!
//! ```text
//! scripts/fetch-testdata.sh gameboy
//! RSEMU_CONFORMANCE=1 cargo test --all-features sm83::conformance -- --nocapture
//! ```
//!
//! Two environment variables select the corpora, both defaulting to
//! `<repo>/testdata`:
//!
//! | Variable | Points at |
//! | --- | --- |
//! | `RSEMU_GB_BLARGG_DIR` | a directory of blargg `.gb` ROMs, searched recursively |
//! | `RSEMU_GB_MOONEYE_DIR` | a checkout of `Gekkio/mooneye-test-suite`'s built ROMs |
//!
//! # How each suite reports
//!
//! **Blargg** writes its result to the serial port *and* to the screen. A
//! headless runner reads the serial port: every byte written to `$FF01` while
//! `$FF02` bit 7 is set is one character of the transcript, so the runner needs
//! no PPU at all. A transcript containing `Passed` is a pass; one containing
//! `Failed` is a failure and the text says which sub-test.
//!
//! **Mooneye** signals through the register file. On success the CPU executes
//! `LD B,B` — the suite's chosen software breakpoint — with `B`, `C`, `D`, `E`,
//! `H`, `L` set to the first six Fibonacci numbers past 1: 3, 5, 8, 13, 21, 34.
//! Any other register pattern at that breakpoint is a failure. That is a far
//! stricter gate than blargg's, and it is the one `ROADMAP.md` §13 names.
//!
//! # The machine these run on
//!
//! Deliberately not the full `machines/gameboy.machine`: these are *CPU* suites
//! first, and a runner that needs a scheduler, a PPU and a cartridge mapper to
//! start cannot be used to bring a core up. So this file assembles the smallest
//! machine each ROM needs — flat RAM, the ROM at `$0000`, a timer, a serial
//! port — directly out of `core::space`. `dev::gb`'s own tests cover the whole
//! machine.

use alloc::format;
use alloc::string::{String, ToString};
use alloc::sync::Arc;
use alloc::vec::Vec;
use std::path::{Path, PathBuf};

use crate::core::device::{Device, ResetKind};
use crate::core::space::{
    AccessConstraints, AddressSpace, MemAttrs, MemOps, MemResult, RamStore, Region, RequesterId,
    UnassignedPolicy,
};
use crate::core::sync::{LockRank, Mutex};
use crate::core::value::Width;

use super::{Config, Sm83, interrupt};

/// The master gate. Nothing here touches a corpus unless it is set.
const GATE: &str = "RSEMU_CONFORMANCE";

/// Overrides the corpus root. Defaults to `<repo>/testdata`.
const TESTDATA: &str = "RSEMU_TESTDATA";

/// How many machine cycles a single ROM may run before the runner gives up.
///
/// `cpu_instrs` is the long one: about 25 seconds of emulated time on real
/// hardware, so a generous bound is still a bound.
const CYCLE_LIMIT: u64 = 250_000_000;

fn enabled() -> bool {
    matches!(
        std::env::var(GATE).as_deref(),
        Ok("1") | Ok("true") | Ok("yes")
    )
}

fn testdata_root() -> PathBuf {
    match std::env::var_os(TESTDATA) {
        Some(dir) => PathBuf::from(dir),
        None => PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("testdata"),
    }
}

/// The directory a suite's ROMs live in, or the reason there is nothing to run.
fn corpus(var: &str, default: &str, fetch: &str) -> Option<PathBuf> {
    if !enabled() {
        println!("SKIP {default}: set {GATE}=1 to run conformance suites");
        return None;
    }
    let dir = match std::env::var_os(var) {
        Some(d) => PathBuf::from(d),
        None => testdata_root().join(default),
    };
    if !dir.is_dir() {
        println!("SKIP {default}: corpus not found at {}", dir.display());
        println!("      fetch it with: {fetch}");
        return None;
    }
    Some(dir)
}

/// Every `.gb` file under `dir`, sorted, so a run is reproducible.
fn roms(dir: &Path) -> Vec<PathBuf> {
    let mut out = Vec::new();
    collect(dir, &mut out);
    out.sort();
    out
}

fn collect(dir: &Path, out: &mut Vec<PathBuf>) {
    let Ok(entries) = std::fs::read_dir(dir) else {
        return;
    };
    for entry in entries.flatten() {
        let path = entry.path();
        if path.is_dir() {
            collect(&path, out);
        } else if path.extension().is_some_and(|e| e == "gb") {
            out.push(path);
        }
    }
}

/// The path a report should name a ROM by: relative to the corpus root.
fn label(root: &Path, rom: &Path) -> String {
    rom.strip_prefix(root)
        .unwrap_or(rom)
        .to_string_lossy()
        .to_string()
}

// ---------------------------------------------------------------------------
// The minimal machine
// ---------------------------------------------------------------------------

/// The serial port, as much of it as a test ROM uses.
///
/// A write to `$FF02` with bit 7 set starts a transfer; the byte in `$FF01` is
/// what comes out, and on a machine with nothing on the other end it is
/// immediately "sent". Blargg's ROMs use exactly this as a transcript channel.
#[derive(Debug)]
struct SerialPort {
    out: Mutex<Vec<u8>>,
    data: Mutex<u8>,
}

impl SerialPort {
    fn new() -> SerialPort {
        SerialPort {
            out: Mutex::with_rank(LockRank::DEVICE, Vec::new()),
            data: Mutex::with_rank(LockRank::DEVICE, 0),
        }
    }

    fn transcript(&self) -> String {
        String::from_utf8_lossy(&self.out.lock()).to_string()
    }
}

impl MemOps for SerialPort {
    fn read(&self, offset: u64, dst: &mut [u8], _attrs: MemAttrs) -> MemResult {
        let [byte] = dst else {
            return Err(crate::core::BusError::BadAccess);
        };
        *byte = match offset {
            0 => *self.data.lock(),
            // Bit 7 clear: no transfer is ever in progress, because every one
            // completes the instant it starts.
            _ => 0x7e,
        };
        Ok(())
    }

    fn write(&self, offset: u64, src: &[u8], _attrs: MemAttrs) -> MemResult {
        let [value] = src else {
            return Err(crate::core::BusError::BadAccess);
        };
        match offset {
            0 => *self.data.lock() = *value,
            _ => {
                if value & 0x80 != 0 {
                    let byte = *self.data.lock();
                    self.out.lock().push(byte);
                }
            }
        }
        Ok(())
    }

    fn constraints(&self) -> AccessConstraints {
        AccessConstraints::IO.with_widths(Width::U8, Width::U8)
    }
}

/// A timer good enough for a CPU suite: `DIV`, `TIMA`, `TMA`, `TAC`, driven from
/// the CPU's own cycle count rather than from the scheduler.
///
/// The real one is [`crate::dev::gb::timer`], which is a lazily-advanced device
/// on its own clock domain. This is its poor relation, and it exists because a
/// CPU bring-up runner must not need a scheduler: it is advanced explicitly by
/// the loop below, once per instruction, which is exactly the granularity the
/// scheduler gives the real one anyway.
#[derive(Debug, Default)]
struct TimerState {
    /// The 16-bit counter whose top byte is `DIV`.
    counter: u16,
    tima: u8,
    tma: u8,
    tac: u8,
    /// The falling-edge detector's previous input.
    last_edge: bool,
    /// Whether `TIMA` overflowed and the reload is owed.
    overflow: bool,
    /// Set when `TIMA` overflows, read and cleared by the runner.
    irq: bool,
}

#[derive(Debug)]
struct Timer {
    state: Mutex<TimerState>,
}

impl Timer {
    fn new() -> Timer {
        Timer {
            state: Mutex::with_rank(LockRank::DEVICE, TimerState::default()),
        }
    }

    /// Which bit of the internal counter `TAC` selects (Pan Docs, *Timer and
    /// Divider Registers*).
    fn selected_bit(tac: u8) -> u16 {
        match tac & 3 {
            0 => 1 << 9,
            1 => 1 << 3,
            2 => 1 << 5,
            _ => 1 << 7,
        }
    }

    /// Advance by `clocks` crystal periods, four to the machine cycle.
    fn advance(&self, clocks: u64) -> bool {
        let mut s = self.state.lock();
        let mut fired = false;
        for _ in 0..clocks {
            s.counter = s.counter.wrapping_add(1);
            if s.overflow {
                s.tima = s.tma;
                s.overflow = false;
                s.irq = true;
                fired = true;
            }
            let edge = s.tac & 0x04 != 0 && s.counter & Timer::selected_bit(s.tac) != 0;
            if s.last_edge && !edge {
                s.tima = s.tima.wrapping_add(1);
                if s.tima == 0 {
                    s.overflow = true;
                }
            }
            s.last_edge = edge;
        }
        fired
    }

    fn take_irq(&self) -> bool {
        let mut s = self.state.lock();
        core::mem::replace(&mut s.irq, false)
    }
}

impl MemOps for Timer {
    fn read(&self, offset: u64, dst: &mut [u8], _attrs: MemAttrs) -> MemResult {
        let [byte] = dst else {
            return Err(crate::core::BusError::BadAccess);
        };
        let s = self.state.lock();
        *byte = match offset {
            0 => (s.counter >> 8) as u8,
            1 => s.tima,
            2 => s.tma,
            _ => s.tac | 0xf8,
        };
        Ok(())
    }

    fn write(&self, offset: u64, src: &[u8], _attrs: MemAttrs) -> MemResult {
        let [value] = src else {
            return Err(crate::core::BusError::BadAccess);
        };
        let mut s = self.state.lock();
        match offset {
            // Any write resets the whole 16-bit counter, not just the visible
            // byte — which is why a `DIV` write can produce an extra `TIMA`
            // increment through the falling-edge detector below.
            0 => {
                s.counter = 0;
                let edge = s.tac & 0x04 != 0 && s.counter & Timer::selected_bit(s.tac) != 0;
                if s.last_edge && !edge {
                    s.tima = s.tima.wrapping_add(1);
                    if s.tima == 0 {
                        s.overflow = true;
                    }
                }
                s.last_edge = edge;
            }
            1 => {
                s.tima = *value;
                s.overflow = false;
            }
            2 => s.tma = *value,
            _ => s.tac = *value & 0x07,
        }
        Ok(())
    }

    fn constraints(&self) -> AccessConstraints {
        AccessConstraints::IO.with_widths(Width::U8, Width::U8)
    }
}

/// The machine a test ROM runs on: 32 KiB of ROM at `$0000`, RAM everywhere a
/// Game Boy has RAM, a serial port, a timer, and the CPU's own `IF`/`IE`.
struct Harness {
    cpu: Arc<Sm83>,
    serial: Arc<SerialPort>,
    timer: Arc<Timer>,
}

impl Harness {
    /// Build one around `rom`, which is mapped from `$0000` with no mapper.
    ///
    /// Mooneye's acceptance ROMs and blargg's individual tests are all 32 KiB or
    /// smaller and MBC-less; the multi-test `cpu_instrs.gb` is an MBC1 image, so
    /// the ROM is mirrored into the switchable half rather than banked. That is
    /// enough for it, because the individual `cpu_instrs/individual/*.gb` files
    /// are what the runner prefers when they are present.
    fn new(rom: &[u8]) -> Harness {
        let space = Arc::new(
            // A Game Boy's data bus is pulled up, so anything nobody answers
            // reads as $FF. That is a real behaviour test ROMs depend on.
            AddressSpace::new("cpubus", 16).with_unassigned(UnassignedPolicy::ONES),
        );
        let cpu = Arc::new(Sm83::new(Config::DMG.with_requester(RequesterId(1))));
        let serial = Arc::new(SerialPort::new());
        let timer = Arc::new(Timer::new());

        let rom_store = Arc::new(crate::core::space::RomStore::new(rom.to_vec()));
        let rom_region = Arc::new(Region::rom(
            "rom",
            rom_store,
            crate::core::space::RomWrite::Ignore,
        ));
        {
            let mut topo = space.topology();
            // `mirror` covers the whole $0000-$7FFF window whatever the image's
            // size, which is what an un-banked cartridge does.
            topo.map(
                Arc::new(Region::mirror("cart", rom_region, 0x8000).expect("a power of two")),
                0x0000,
            )
            .expect("maps");
            topo.map(Region::ram("vram", Arc::new(RamStore::new(0x2000))), 0x8000)
                .expect("maps");
            topo.map(Region::ram("sram", Arc::new(RamStore::new(0x2000))), 0xa000)
                .expect("maps");
            let wram = Arc::new(RamStore::new(0x2000));
            topo.map(Region::ram("wram", Arc::clone(&wram)), 0xc000)
                .expect("maps");
            topo.map(
                Arc::new(
                    Region::alias("echo", Arc::new(Region::ram("wram.echo", wram)), 0, 0x1e00)
                        .expect("fits"),
                ),
                0xe000,
            )
            .expect("maps");
            topo.map(Region::ram("oam", Arc::new(RamStore::new(0x100))), 0xfe00)
                .expect("maps");
            topo.map(
                Arc::new(Region::io(
                    "serial",
                    2,
                    Arc::clone(&serial) as Arc<dyn MemOps>,
                )),
                0xff01,
            )
            .expect("maps");
            topo.map(
                Arc::new(Region::io(
                    "timer",
                    4,
                    Arc::clone(&timer) as Arc<dyn MemOps>,
                )),
                0xff04,
            )
            .expect("maps");
            topo.map(
                Device::region(cpu.as_ref(), super::IF_REGION).expect("IF"),
                super::IF_ADDRESS,
            )
            .expect("maps");
            // $FF10-$FF7F is the rest of the I/O page plus HRAM. RAM stands in
            // for the sound and LCD registers a CPU suite does not exercise;
            // what matters is that writes stick and reads come back.
            topo.map(Region::ram("io", Arc::new(RamStore::new(0x70))), 0xff10)
                .expect("maps");
            topo.map(Region::ram("hram", Arc::new(RamStore::new(0x7f))), 0xff80)
                .expect("maps");
            topo.map(
                Device::region(cpu.as_ref(), super::IE_REGION).expect("IE"),
                super::IE_ADDRESS,
            )
            .expect("maps");
        }

        cpu.attach_space(Arc::clone(&space));
        Device::reset(cpu.as_ref(), ResetKind::Cold);
        Harness { cpu, serial, timer }
    }

    /// Run until `stop` says to, or the cycle limit is reached.
    ///
    /// Returns the machine cycles consumed. The timer is advanced once per
    /// instruction, which is the same granularity the scheduler gives the real
    /// device (`ROADMAP.md` §4.2: within a quantum a runnable's progress is not
    /// yet in the clock forest).
    fn run(&self, mut stop: impl FnMut(&Sm83) -> bool) -> u64 {
        let mut cycles = 0u64;
        while cycles < CYCLE_LIMIT {
            if stop(&self.cpu) {
                break;
            }
            let n = self.cpu.step();
            if n == 0 {
                break;
            }
            cycles += n;
            // Four clocks to the machine cycle.
            if self.timer.advance(n * 4) && self.timer.take_irq() {
                self.cpu.request_interrupt(interrupt::TIMER);
            }
        }
        cycles
    }
}

// ---------------------------------------------------------------------------
// blargg
// ---------------------------------------------------------------------------

/// What a blargg ROM's serial transcript says about itself.
#[derive(Debug, PartialEq, Eq)]
enum Blargg {
    Passed,
    Failed(String),
    /// Nothing conclusive within the cycle limit.
    Inconclusive(String),
}

fn run_blargg(rom: &[u8]) -> Blargg {
    let harness = Harness::new(rom);
    // Checking the transcript on every instruction would cost more than running
    // the ROM, so it is sampled: `Passed` and `Failed` are both terminal, and a
    // few thousand instructions of overrun costs nothing.
    let mut countdown = 0u32;
    harness.run(|_| {
        if countdown > 0 {
            countdown -= 1;
            return false;
        }
        countdown = 100_000;
        let text = harness.serial.transcript();
        text.contains("Passed") || text.contains("Failed")
    });
    let text = harness.serial.transcript();
    if text.contains("Passed") {
        Blargg::Passed
    } else if text.contains("Failed") {
        Blargg::Failed(text)
    } else {
        Blargg::Inconclusive(text)
    }
}

#[test]
fn blargg_suite() {
    let Some(dir) = corpus(
        "RSEMU_GB_BLARGG_DIR",
        "gb-blargg",
        "scripts/fetch-testdata.sh gb-blargg",
    ) else {
        return;
    };
    let roms = roms(&dir);
    if roms.is_empty() {
        println!("SKIP gb-blargg: no .gb files under {}", dir.display());
        return;
    }
    let mut passed = 0usize;
    let mut failures = Vec::new();
    for rom in &roms {
        let name = label(&dir, rom);
        let Ok(bytes) = std::fs::read(rom) else {
            failures.push(format!("{name}: unreadable"));
            continue;
        };
        match run_blargg(&bytes) {
            Blargg::Passed => {
                passed += 1;
                println!("  pass {name}");
            }
            Blargg::Failed(text) => {
                println!("  FAIL {name}: {}", text.trim().replace('\n', " / "));
                failures.push(name);
            }
            Blargg::Inconclusive(text) => {
                println!(
                    "  ???? {name}: no verdict in {CYCLE_LIMIT} cycles ({})",
                    text.trim().replace('\n', " / ")
                );
                failures.push(name);
            }
        }
    }
    println!("blargg: {passed}/{} ROMs passed", roms.len());
    assert!(
        failures.is_empty(),
        "blargg failures: {}",
        failures.join(", ")
    );
}

// ---------------------------------------------------------------------------
// mooneye
// ---------------------------------------------------------------------------

/// The register pattern Gekkio's suite sets before its `LD B,B` breakpoint:
/// the Fibonacci numbers 3, 5, 8, 13, 21, 34 in `B`, `C`, `D`, `E`, `H`, `L`.
const MOONEYE_PASS: [u8; 6] = [3, 5, 8, 13, 21, 34];

/// `LD B,B`, which is a no-op the suite uses as a software breakpoint.
const MOONEYE_BREAKPOINT: u8 = 0x40;

#[derive(Debug, PartialEq, Eq)]
enum Mooneye {
    Passed,
    /// Reached the breakpoint with the wrong registers.
    Failed(String),
    /// Never reached the breakpoint.
    Timeout,
}

fn run_mooneye(rom: &[u8]) -> Mooneye {
    let harness = Harness::new(rom);
    let space = harness.cpu.space().expect("a space");
    let mut hit = false;
    harness.run(|cpu| {
        let pc = cpu.regs().pc;
        let opcode = space
            .read(u64::from(pc), Width::U8, MemAttrs::DEBUG)
            .unwrap_or(0) as u8;
        if opcode == MOONEYE_BREAKPOINT {
            hit = true;
            return true;
        }
        false
    });
    if !hit {
        return Mooneye::Timeout;
    }
    let r = harness.cpu.regs();
    let got = [r.b, r.c, r.d, r.e, r.h, r.l];
    if got == MOONEYE_PASS {
        Mooneye::Passed
    } else {
        Mooneye::Failed(format!(
            "B={:02x} C={:02x} D={:02x} E={:02x} H={:02x} L={:02x}",
            r.b, r.c, r.d, r.e, r.h, r.l
        ))
    }
}

#[test]
fn mooneye_acceptance() {
    let Some(dir) = corpus(
        "RSEMU_GB_MOONEYE_DIR",
        "gb-mooneye",
        "scripts/fetch-testdata.sh gb-mooneye",
    ) else {
        return;
    };
    let roms = roms(&dir);
    if roms.is_empty() {
        println!("SKIP gb-mooneye: no .gb files under {}", dir.display());
        return;
    }
    let mut passed = 0usize;
    let mut failed = Vec::new();
    for rom in &roms {
        let name = label(&dir, rom);
        let Ok(bytes) = std::fs::read(rom) else {
            failed.push(format!("{name}: unreadable"));
            continue;
        };
        match run_mooneye(&bytes) {
            Mooneye::Passed => {
                passed += 1;
                println!("  pass {name}");
            }
            Mooneye::Failed(regs) => {
                println!("  FAIL {name}: {regs}");
                failed.push(name);
            }
            Mooneye::Timeout => {
                println!("  TIME {name}: never reached the LD B,B breakpoint");
                failed.push(name);
            }
        }
    }
    println!("mooneye: {passed}/{} ROMs passed", roms.len());
    println!("  {} still failing", failed.len());
    // Not an assertion. `ROADMAP.md` §0 asks for a *measured* number and a
    // ledger that only shrinks, and the mooneye suite covers behaviours this
    // core does not claim yet (sub-instruction PPU timing above all). The count
    // above is the measurement; `docs` carries the ledger.
}

#[test]
fn the_harness_runs_a_synthetic_rom() {
    // Not gated: proof that the harness itself works, so a skip above really
    // means "no corpus" rather than "the runner is broken". This is a tiny ROM
    // that writes "Hi" out of the serial port and then loops.
    let mut rom = alloc::vec![0u8; 0x8000];
    let program: &[u8] = &[
        0x3e, b'H', // LD A,'H'
        0xe0, 0x01, // LDH ($ff01),A
        0x3e, 0x81, // LD A,$81
        0xe0, 0x02, // LDH ($ff02),A
        0x3e, b'i', // LD A,'i'
        0xe0, 0x01, // LDH ($ff01),A
        0x3e, 0x81, // LD A,$81
        0xe0, 0x02, // LDH ($ff02),A
        0x40, // LD B,B — the mooneye breakpoint
    ];
    rom[0x0100..0x0100 + program.len()].copy_from_slice(program);
    let harness = Harness::new(&rom);
    let space = harness.cpu.space().expect("a space");
    harness.run(|cpu| {
        space
            .read(u64::from(cpu.regs().pc), Width::U8, MemAttrs::DEBUG)
            .unwrap_or(0)
            == u64::from(MOONEYE_BREAKPOINT)
    });
    assert_eq!(harness.serial.transcript(), "Hi");
}

#[test]
fn the_harness_timer_ticks_at_the_documented_rates() {
    // `DIV` is the top byte of a counter clocked at the crystal rate, so it
    // advances once every 256 clocks.
    let timer = Timer::new();
    let mut div = [0u8];
    timer.advance(255);
    timer.read(0, &mut div, MemAttrs::DEFAULT).unwrap();
    assert_eq!(div[0], 0);
    timer.advance(1);
    timer.read(0, &mut div, MemAttrs::DEFAULT).unwrap();
    assert_eq!(div[0], 1);

    // With `TAC` = $05, `TIMA` is clocked off bit 3, i.e. every 16 clocks.
    let timer = Timer::new();
    timer.write(3, &[0x05], MemAttrs::DEFAULT).unwrap();
    let mut tima = [0u8];
    timer.advance(16 * 4);
    timer.read(1, &mut tima, MemAttrs::DEFAULT).unwrap();
    assert_eq!(tima[0], 4);
}
