//! Whole-machine conformance runners for the Game Boy.
//!
//! `ROADMAP.md` §0: *accuracy is measured, never asserted*. The SM83's own
//! runner (`cpu::sm83::conformance`) measures the processor on a hand-built
//! minimal bus; this one runs the **shipped machine** — `machines/gameboy.machine`,
//! realized through the catalog exactly as `rsemu run gameboy` would — because
//! Gekkio's acceptance suite is not a CPU suite. Nearly every ROM in it waits
//! for a real `LY` to reach 144 before it starts, and most of them measure the
//! timer, the LCD or the OAM DMA against the processor. A runner without those
//! devices does not fail those tests; it hangs before them.
//!
//! # Running
//!
//! ```text
//! scripts/fetch-testdata.sh gameboy
//! RSEMU_CONFORMANCE=1 cargo test --release --all-features gb::conformance -- --nocapture
//! ```
//!
//! | Variable | Points at |
//! | --- | --- |
//! | `RSEMU_GB_BLARGG_DIR` | a directory of blargg `.gb` ROMs, searched recursively |
//! | `RSEMU_GB_MOONEYE_DIR` | `Gekkio/mooneye-test-suite`'s built acceptance ROMs |
//! | `RSEMU_GB_FRAMES` | how many emulated frames a ROM may run for (default 4000) |
//!
//! Without the gate, or without a corpus, every runner prints why it is doing
//! nothing and passes. `cargo test` offline stays green; that is a rule
//! (CLAUDE.md, Testing), not a convenience.
//!
//! # How a result is read out
//!
//! There is deliberately no route from a `dyn Device` to a `GbSerial` — the core
//! keeps `Any` out of the supertrait chain on purpose — so the runners read
//! results the way `ROADMAP.md` §4.5 already promises anyone can: out of the
//! device's own **snapshot chunk**. Calling [`Device::save`] on one device is
//! cheap (the CPU's chunk is forty bytes), and doing it this way doubles as a
//! check that the chunk really is the architectural state.
//!
//! **Blargg** writes its verdict to the serial port a character at a time, which
//! is exactly why it can be run headless.
//!
//! **Mooneye** writes its verdict into the register file: `B`,`C`,`D`,`E`,`H`,`L`
//! = 3, 5, 8, 13, 21, 34 on success, and `$42` in all six on failure, before an
//! `LD B,B` software breakpoint. The register pattern is what this runner looks
//! for, since it survives whatever the ROM does next.

use alloc::format;
use alloc::string::{String, ToString};
use alloc::vec::Vec;
use std::path::{Path, PathBuf};

use crate::core::state::{MachineShape, Migrations, Source, StateReader, StateWriter};
use crate::machine::{Machine, catalog};

/// The master gate.
const GATE: &str = "RSEMU_CONFORMANCE";

/// Overrides the corpus root. Defaults to `<repo>/testdata`.
const TESTDATA: &str = "RSEMU_TESTDATA";

/// How many emulated frames a ROM may run for before the runner gives up.
const DEFAULT_FRAMES: u64 = 4000;

/// How many scheduler quanta to run between checks of the result.
///
/// A quantum here is bounded by whichever lazily-advanced device has the nearest
/// event, which on this machine is a mode change on the LCD — a few hundred
/// crystal periods. A few thousand of them is well under a frame.
const QUANTA_PER_CHECK: u32 = 2048;

fn enabled() -> bool {
    matches!(
        std::env::var(GATE).as_deref(),
        Ok("1") | Ok("true") | Ok("yes")
    )
}

fn frame_limit() -> u64 {
    std::env::var("RSEMU_GB_FRAMES")
        .ok()
        .and_then(|v| v.parse().ok())
        .unwrap_or(DEFAULT_FRAMES)
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

fn label(root: &Path, rom: &Path) -> String {
    rom.strip_prefix(root)
        .unwrap_or(rom)
        .to_string_lossy()
        .to_string()
}

// ---------------------------------------------------------------------------
// Reading a device's state without downcasting it
// ---------------------------------------------------------------------------

/// One device's snapshot chunk, as bytes.
///
/// `ROADMAP.md` §4.5's promise made useful: the chunk *is* the architectural
/// state, so anything that wants to observe a device from outside can read it
/// there rather than reaching for a downcast the core deliberately does not
/// offer.
fn chunk_of(machine: &Machine, path: &str) -> Option<Vec<u8>> {
    let entry = machine.device(path)?;
    let class = entry.class();
    let mut writer = StateWriter::new(MachineShape::new());
    {
        let mut chunk = writer.chunk(path, class.name, class.version).ok()?;
        entry.device().save(&mut chunk).ok()?;
    }
    let bytes = writer.to_vec().ok()?;
    let reader = StateReader::new(&bytes).ok()?;
    let chunk = reader
        .load(path, class.name, class.version, &Migrations::new())
        .ok()?;
    Some(chunk.into_data())
}

/// The six registers mooneye reports its verdict in.
fn verdict_registers(machine: &Machine) -> Option<[u8; 6]> {
    let data = chunk_of(machine, "cpu")?;
    // `cpu.sm83` writes A, F, B, C, D, E, H, L in that order.
    let mut r = crate::core::state::ChunkReader::new(&data);
    let mut byte = || r.read_u8().ok();
    let _a = byte()?;
    let _f = byte()?;
    Some([byte()?, byte()?, byte()?, byte()?, byte()?, byte()?])
}

/// Everything the serial port has sent.
fn serial_transcript(machine: &Machine) -> Option<String> {
    let data = chunk_of(machine, "link")?;
    let mut r = crate::core::state::ChunkReader::new(&data);
    // `gb.serial` writes SB, SC, the remaining clocks, its tick, then the
    // transcript.
    let _sb = r.read_u8().ok()?;
    let _sc = r.read_u8().ok()?;
    let _remaining = r.read_u64().ok()?;
    let _tick = r.read_u64().ok()?;
    let bytes = r.read_bytes().ok()?;
    Some(bytes.iter().map(|b| *b as char).collect())
}

/// How many frames the LCD controller has finished.
///
/// Read out of its chunk for the same reason as everything else here. The
/// framebuffer is the first three length-prefixed byte arrays; the frame counter
/// follows the register bytes.
///
/// **This walk moves with `gb.ppu`'s chunk layout**, and it fails quietly if it
/// does not: a `u64` read off the wrong offset still succeeds and returns a
/// number, the runner then thinks its frame budget is exhausted, and every ROM
/// reports "no verdict" while the machine is in fact perfectly healthy. That is
/// exactly what happened once. `the_harness_reads_the_register_file_and_the_
/// frame_counter` now checks the number rather than its existence, so the next
/// layout change fails a test instead of a suite.
fn frames(machine: &Machine) -> Option<u64> {
    let data = chunk_of(machine, "ppu")?;
    let mut r = crate::core::state::ChunkReader::new(&data);
    let _vram = r.read_bytes().ok()?;
    let _oam = r.read_bytes().ok()?;
    let _fb = r.read_bytes().ok()?;
    for _ in 0..14 {
        r.read_u8().ok()?;
    }
    let _window_active = r.read_bool().ok()?;
    let _lyc_match = r.read_bool().ok()?;
    let _dot = r.read_u64().ok()?;
    let _dots = r.read_u64().ok()?;
    r.read_u64().ok()
}

// ---------------------------------------------------------------------------
// The runner
// ---------------------------------------------------------------------------

/// Build the shipped Game Boy around `rom` and run it.
fn machine_for(rom: &[u8]) -> Result<Machine, String> {
    catalog::build_catalog("gameboy", &[("cart", rom)]).map_err(|e| e.to_string())
}

/// Roughly how many frames' worth of emulated time one batch of
/// [`QUANTA_PER_CHECK`] quanta covers when the LCD is switched **off**.
///
/// With the LCD running, a quantum ends at whichever lazily-advanced device has
/// the nearest event, which is a mode boundary — about 440 a frame, so a batch
/// is several frames. With the LCD off the controller has no events at all and
/// only the divider's do, at one every 256 crystal periods; a batch is then
/// about an eighth of a frame's worth of time.
const BATCHES_PER_FRAME_LCD_OFF: u64 = 8;

/// Run until `stop` says so or the budget runs out.
///
/// Returns whether `stop` fired.
///
/// **Two limits, not one.** The obvious budget is emulated frames, and it is the
/// one worth stating — but a ROM that switches the LCD off stops the frame
/// counter dead, and several of Gekkio's do exactly that. A budget that counted
/// only frames would then never expire and the runner would hang rather than
/// report a timeout. So the batch count is bounded too, generously enough that
/// it never binds while the LCD is running.
fn run(machine: &mut Machine, limit_frames: u64, mut stop: impl FnMut(&Machine) -> bool) -> bool {
    let start = frames(machine).unwrap_or(0);
    let max_batches = limit_frames
        .saturating_mul(BATCHES_PER_FRAME_LCD_OFF)
        .max(1);
    for _ in 0..max_batches {
        for _ in 0..QUANTA_PER_CHECK {
            if machine.run_quantum().is_err() {
                return stop(machine);
            }
        }
        if stop(machine) {
            return true;
        }
        if frames(machine).unwrap_or(u64::MAX).saturating_sub(start) >= limit_frames {
            return false;
        }
    }
    false
}

// ---------------------------------------------------------------------------
// blargg
// ---------------------------------------------------------------------------

/// The known-failures ledger for the whole-machine blargg run.
///
/// `ROADMAP.md` §0 asks every core to ship a ledger that *only ever shrinks*,
/// and this is it. It is **empty**, and the entry that used to be here is worth
/// recording because of how it went:
///
/// **`instr_timing`** passed 12/12 against the SM83 on its own
/// (`cpu::sm83::conformance`), where the timer is advanced in step with the
/// processor, and failed on the assembled machine. The cause was the
/// intra-quantum staleness `ROADMAP.md` §4.2 used to record as outstanding: a
/// [`LazyHandle`](crate::core::sched::LazyHandle) caught a device up only to
/// the tick the *scheduler* had published, which is the start of the current
/// quantum, so an instruction reading the timer read one that was up to five
/// machine cycles behind. `instr_timing` measures the timer against single
/// instructions, which is exactly that bias.
///
/// [`TickCursor`](crate::core::sched::TickCursor) closed it: the SM83 now
/// publishes its machine-cycle counter as it executes
/// ([`Sm83::attach_cursor`](crate::cpu::sm83::Sm83::attach_cursor)) and every
/// lazily-advanced device is converted onto that cycle through the oscillator
/// tree they share. The entry came out when that landed, and nothing has gone
/// back in.
const BLARGG_LEDGER: &[&str] = &[];

#[test]
fn blargg_on_the_shipped_machine() {
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
    let limit = frame_limit();
    let mut passed = 0usize;
    let mut failures = Vec::new();
    for rom in &roms {
        let name = label(&dir, rom);
        let Ok(bytes) = std::fs::read(rom) else {
            failures.push(format!("{name}: unreadable"));
            continue;
        };
        let mut machine = match machine_for(&bytes) {
            Ok(m) => m,
            Err(e) => {
                println!("  FAIL {name}: {e}");
                failures.push(name);
                continue;
            }
        };
        machine.reset(crate::core::device::ResetKind::Cold);
        run(&mut machine, limit, |m| {
            let text = serial_transcript(m).unwrap_or_default();
            text.contains("Passed") || text.contains("Failed")
        });
        let text = serial_transcript(&machine).unwrap_or_default();
        if text.contains("Passed") {
            passed += 1;
            println!("  pass {name}");
        } else if text.contains("Failed") {
            let ledgered = BLARGG_LEDGER.iter().any(|l| name.ends_with(l));
            let mark = if ledgered { "LDGR" } else { "FAIL" };
            println!("  {mark} {name}: {}", text.trim().replace('\n', " / "));
            if !ledgered {
                failures.push(name);
            }
        } else {
            println!(
                "???? {name}: no verdict in {limit} frames ({})",
                text.trim().replace('\n', " / ")
            );
            failures.push(name);
        }
    }
    println!(
        "blargg (whole machine): {passed}/{} ROMs passed, {} ledgered",
        roms.len(),
        BLARGG_LEDGER.len()
    );
    assert!(
        failures.is_empty(),
        "blargg failures: {}",
        failures.join(", ")
    );
}

// ---------------------------------------------------------------------------
// mooneye
// ---------------------------------------------------------------------------

/// The register pattern Gekkio's suite sets on success: the Fibonacci numbers
/// 3, 5, 8, 13, 21, 34 in `B`, `C`, `D`, `E`, `H`, `L`.
const MOONEYE_PASS: [u8; 6] = [3, 5, 8, 13, 21, 34];

/// The pattern it sets on failure: `$42` in all six.
const MOONEYE_FAIL: [u8; 6] = [0x42; 6];

/// The known-failures ledger for Gekkio's acceptance suite, with the reason for
/// each one.
///
/// `ROADMAP.md` §0 asks for a ledger that **only ever shrinks**, and for the
/// reason to be stated rather than implied. Anything failing that is *not* here
/// fails the test, so a regression in a passing ROM is a build break while the
/// seven below stay measured rather than hidden.
///
/// Three of them are the same fact: rsemu ships no boot ROM, because the DMG's
/// is 256 bytes of Nintendo's copyrighted code and vendoring it is not ours to
/// do (`ROADMAP.md` §1). Post-boot register values substitute, and these three
/// measure things a *table* of post-boot values cannot carry.
const MOONEYE_LEDGER: &[(&str, &str)] = &[
    (
        "boot_div-dmgABCmgb.gb",
        "no boot ROM: the divider's visible byte is documented as $AB at \
         handover and its low byte is not documented anywhere, so it starts \
         at zero rather than guessed at. This ROM measures the low byte.",
    ),
    (
        "boot_hwio-dmgABCmgb.gb",
        "no boot ROM: this compares the whole I/O page against what a real \
         boot ROM leaves behind, including registers whose handover values \
         are not documented.",
    ),
    (
        "serial/boot_sclk_align-dmgABCmgb.gb",
        "no boot ROM: this measures the serial shift clock's phase against \
         the divider at handover, which is a consequence of how long the \
         boot ROM ran rather than a value anything publishes.",
    ),
    (
        "ppu/intr_2_mode0_timing_sprites.gb",
        "the mode-3 object penalty is Pan Docs' documented approximation — \
         six dots each and up to five more for the first object in a \
         background tile — rather than a fetcher simulation. The ROM \
         tabulates 34 object configurations against the machine cycle mode 0 \
         arrives on, and the split is informative: its whole *count* column \
         — one to ten objects at x=0, costing 2 4 5 7 8 10 11 13 14 16 \
         machine cycles — comes out exactly right, so the eleven-and-six \
         rule and the phase are both right. Its *alignment* column does not: \
         ten objects at an x of 1, 5, 6 or 7 modulo 8 want a first-object \
         penalty this formula does not produce — 11, and at least 7, against \
         the 10 and 6 it gives. Four residues out of eight is a fetcher, not \
         a constant, and guessing one to make the table line up is the fit \
         `ROADMAP.md` §0 forbids.",
    ),
    (
        "ppu/lcdon_timing-GS.gb",
        "the first line after the LCD is switched on is modelled to the \
         machine cycle (`ppu::LCD_ON_SKIP`, and the scan reporting mode 0), \
         which is what `stat_lyc_onoff` needs and what this ROM's LY column \
         says. Its STAT and memory-gate columns disagree with each other by \
         one machine cycle unless the controller comes up on a *half*-cycle \
         boundary — its own header says the PPU is late by 2 T-cycles — and \
         this timeline is quantised to the CPU's machine cycle.",
    ),
    (
        "ppu/lcdon_write_timing-GS.gb",
        "the same 2-T-cycle phase as `lcdon_timing`.",
    ),
    (
        "timer/rapid_toggle.gb",
        "one missing increment out of sixteen. The ROM starts and stops the \
         timer every 17 machine cycles and counts on the falling edges that \
         produces; ours takes the interrupt one instruction later than \
         hardware (`BC` = $FFD8 against $FFD9). Traced to iteration 29 of \
         its loop, where the enabling write lands exactly on the divider \
         bit's falling edge: four clocks earlier and the edge falls inside \
         the window the timer is enabled for, which is the increment we do \
         not make. Whether the write should see the counter four clocks \
         earlier is a question about the whole write path, and fitting the \
         offset would defeat the rest of the group rather than pass this \
         one.",
    ),
];

/// Whether `name` is on the ledger, and why.
fn ledgered(name: &str) -> Option<&'static str> {
    MOONEYE_LEDGER
        .iter()
        .find(|(rom, _)| name.ends_with(rom))
        .map(|(_, why)| *why)
}

/// Which of Gekkio's ROMs target a DMG at all.
///
/// The suite ships variants for several models, named by suffix: `-dmgABC` and
/// `-GS` include the DMG, while `-dmg0`, `-mgb`, `-sgb`, `-sgb2` and `-S` are
/// other consoles and would fail on hardware too. Running them and counting the
/// failures would be measuring the wrong thing.
fn targets_dmg(name: &str) -> bool {
    let stem = name.rsplit('/').next().unwrap_or(name);
    let stem = stem.strip_suffix(".gb").unwrap_or(stem);
    match stem.rsplit_once('-') {
        Some((_, suffix)) => matches!(suffix, "dmgABC" | "dmgABCmgb" | "GS"),
        None => true,
    }
}

#[test]
fn mooneye_acceptance_on_the_shipped_machine() {
    let Some(dir) = corpus(
        "RSEMU_GB_MOONEYE_DIR",
        "gb-mooneye",
        "scripts/fetch-testdata.sh gb-mooneye",
    ) else {
        return;
    };
    let all = roms(&dir);
    let roms: Vec<_> = all
        .into_iter()
        .filter(|p| targets_dmg(&label(&dir, p)))
        .collect();
    if roms.is_empty() {
        println!("SKIP gb-mooneye: no DMG .gb files under {}", dir.display());
        return;
    }
    let limit = frame_limit();
    let mut passed = 0usize;
    let mut ledger_hits = 0usize;
    let mut failed = Vec::new();
    let record =
        |name: String, failed: &mut Vec<String>, ledger_hits: &mut usize| match ledgered(&name) {
            Some(why) => {
                *ledger_hits += 1;
                println!("  LDGR {name}: {why}");
            }
            None => failed.push(name),
        };
    for rom in &roms {
        let name = label(&dir, rom);
        let Ok(bytes) = std::fs::read(rom) else {
            failed.push(format!("{name}: unreadable"));
            continue;
        };
        let mut machine = match machine_for(&bytes) {
            Ok(m) => m,
            Err(e) => {
                println!("  FAIL {name}: {e}");
                failed.push(name);
                continue;
            }
        };
        machine.reset(crate::core::device::ResetKind::Cold);
        run(&mut machine, limit, |m| {
            matches!(
                verdict_registers(m),
                Some(MOONEYE_PASS) | Some(MOONEYE_FAIL)
            )
        });
        match verdict_registers(&machine) {
            Some(MOONEYE_PASS) => {
                passed += 1;
                println!("  pass {name}");
            }
            Some(MOONEYE_FAIL) => {
                if ledgered(&name).is_none() {
                    println!("  FAIL {name}");
                }
                record(name, &mut failed, &mut ledger_hits);
            }
            Some(regs) => {
                println!(
                    "  TIME {name}: no verdict in {limit} frames  \
                     (B={:02x} C={:02x} D={:02x} E={:02x} H={:02x} L={:02x})",
                    regs[0], regs[1], regs[2], regs[3], regs[4], regs[5]
                );
                record(name, &mut failed, &mut ledger_hits);
            }
            None => {
                println!("  ???? {name}: could not read the register file");
                failed.push(name);
            }
        }
    }
    println!(
        "mooneye acceptance (DMG subset): {passed}/{} ROMs passed, {ledger_hits} ledgered",
        roms.len()
    );
    // The number is the measurement `ROADMAP.md` §0 asks for; the assertion is
    // the ledger it also asks for. Only *unexplained* failures fail the test,
    // so the seven arguments in `MOONEYE_LEDGER` stay visible rather than
    // hidden, and a ROM that stops passing is a build break.
    assert!(
        failed.is_empty(),
        "mooneye failures with no ledger entry: {}",
        failed.join(", ")
    );
}

// ---------------------------------------------------------------------------
// Ungated: proof that the harness itself works
// ---------------------------------------------------------------------------

#[test]
fn the_harness_reads_a_synthetic_roms_serial_output() {
    // Not gated, so a skip above really means "no corpus" rather than "the
    // runner is broken". A tiny ROM that sends `Hi` and then loops.
    let program: &[u8] = &[
        0x3e, b'H', // LD A,'H'
        0xe0, 0x01, // LDH ($ff01),A
        0x3e, 0x81, // LD A,$81
        0xe0, 0x02, // LDH ($ff02),A
        0x3e, b'i', // LD A,'i'
        0xe0, 0x01, // LDH ($ff01),A
        0x3e, 0x81, // LD A,$81
        0xe0, 0x02, // LDH ($ff02),A
        0x18, 0xfe, // JR -2
    ];
    let rom = super::cart::synthetic_image(2, 0x00, 0x00, program);
    let mut machine = machine_for(&rom).expect("the shipped machine builds");
    machine.reset(crate::core::device::ResetKind::Cold);
    let done = run(&mut machine, 4, |m| {
        serial_transcript(m).unwrap_or_default() == "Hi"
    });
    assert!(done, "the transcript never reached `Hi`");
}

#[test]
fn the_harness_reads_the_register_file_and_the_frame_counter() {
    // `LD B,3 ; LD C,5 ; ... ; JR -2` — the mooneye success pattern, written by
    // hand so that the *decoder* is what is under test rather than a ROM.
    let program: &[u8] = &[
        0x06, 3, // LD B,3
        0x0e, 5, // LD C,5
        0x16, 8, // LD D,8
        0x1e, 13, // LD E,13
        0x26, 21, // LD H,21
        0x2e, 34, // LD L,34
        0x18, 0xfe, // JR -2
    ];
    let rom = super::cart::synthetic_image(2, 0x00, 0x00, program);
    let mut machine = machine_for(&rom).expect("the shipped machine builds");
    machine.reset(crate::core::device::ResetKind::Cold);
    let done = run(&mut machine, 4, |m| {
        verdict_registers(m) == Some(MOONEYE_PASS)
    });
    assert!(done, "the register pattern was never seen");
    // And the LCD really did run while that happened. The *value* matters, not
    // just that a `u64` came out: reading the counter off the wrong offset of a
    // changed chunk still decodes, and the failure it causes looks like every
    // ROM hanging rather than like a broken runner. Four frames were budgeted
    // and the pattern is set in the first few hundred cycles, so the counter is
    // small and non-negative — a misread lands nowhere near that.
    let frames = frames(&machine).expect("the frame counter decodes");
    assert!(
        frames <= 4,
        "the frame counter reads {frames} after at most four frames —          `frames()` is walking the wrong offset of the `gb.ppu` chunk"
    );
}

#[test]
fn only_the_dmg_variants_of_the_suite_are_selected() {
    assert!(targets_dmg("acceptance/timer/tim00.gb"));
    assert!(targets_dmg("acceptance/boot_regs-dmgABC.gb"));
    assert!(targets_dmg("acceptance/di_timing-GS.gb"));
    assert!(!targets_dmg("acceptance/boot_regs-sgb.gb"));
    assert!(!targets_dmg("acceptance/boot_div-dmg0.gb"));
    assert!(!targets_dmg("acceptance/boot_hwio-S.gb"));
}
