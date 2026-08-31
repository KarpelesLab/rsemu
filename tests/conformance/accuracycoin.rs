//! AccuracyCoin — the final phase-3 gate, read out of RAM with no screen.
//!
//! 141 accuracy tests (plus 5 "DRAW" pages that only print information) on one
//! NROM cartridge, MIT, © 2025 Chris Siebert,
//! <https://github.com/100thCoin/AccuracyCoin>.
//!
//! This is a **whole-machine** suite, not a CPU one: NMI suppression, sprite-zero
//! timing, DMC DMA bus conflicts and open-bus behaviour are all about the exact
//! clock alignment between the CPU, the PPU, the APU and DMA. It comes last on
//! purpose, and it is built to report per-test results while most of the machine
//! is still missing rather than being all-or-nothing.
//!
//! # Reading the results headlessly
//!
//! The ROM is menu-driven and normally read off the screen. It does not have to
//! be. Everything below is from the ROM's own commented source
//! (`AccuracyCoin.asm`, MIT) and its `README.md`; `docs/testing/accuracycoin.md`
//! has the long form.
//!
//! * **`$0400-$04FF` is the results page.** Every test has a fixed byte there,
//!   written by the engine's `RunTest` routine from whatever the test returned
//!   in A. [`TESTS`] maps each test to its address.
//! * **The result byte is a state in bits 0-1 and an error code in bits 2-7.**
//!   `0` not run, `1` PASS, `2` FAIL, `3` in progress. On a failure the error
//!   code is `byte >> 2`, and it is the number printed on screen after "FAIL" —
//!   the README's per-section tables list what each one means.
//! * **Five tests store their result on page 3 instead** — all of them into the
//!   one byte `result_DrawTest = $03FF`. Those are the "DRAW" pages, which
//!   display information and assert nothing; the ROM's own run-all loop skips
//!   them, and so does this runner.
//! * **`$0035` is the "running all tests" flag.** The engine sets it to 1 for
//!   the duration of a run-everything pass and clears it at the end — which is
//!   the completion signal a headless runner needs, and much better than
//!   guessing a frame count. `$0037` counts tests completed while it runs.
//! * **`$00EC` is a boot progress counter**, stepped `$00` → `$0D` through
//!   initialisation. If the ROM hangs before the menu, this says how far it got.
//! * **`$0500-$05FF` is per-test scratch**, cleared before each test — the
//!   region the ROM's own debug menu displays. It is diagnostic only, and after
//!   a full run it holds the *last* test's working values, not everyone's.
//!   `$0020-$002F` (unofficial-instruction operands) and `$0050-$006F` are the
//!   other scratch areas the debug menu shows.
//!
//! # Driving the menu
//!
//! Initialisation leaves the cursor at the top of the page (`menuCursorYPos =
//! $FF`), and the menu's NMI handler runs everything on the ROM when Start is
//! newly pressed there. So the whole sequence is: boot, hold Start for a frame
//! or two, release, then wait for `$0035` to go 1 and back to 0. No navigation,
//! no page changes, no rendering.
//!
//! The ROM edge-detects buttons (`controller_New`), so Start must be *released*
//! before it would be seen again — holding it forever presses it once.

use std::fmt::Write as _;

use crate::machine::{NesMachine, buttons};

// ---------------------------------------------------------------------------
// The RAM protocol
// ---------------------------------------------------------------------------

/// `RunningAllTests`: 1 while a run-everything pass is in progress.
pub(crate) const ADDR_RUNNING_ALL: u16 = 0x0035;

/// `PostAllTestTally`: how many tests have completed during the pass.
pub(crate) const ADDR_TALLY: u16 = 0x0037;

/// `Debug_EC`: boot progress, `$00` through `$0D`.
pub(crate) const ADDR_BOOT_PROGRESS: u16 = 0x00ec;

/// The value [`ADDR_BOOT_PROGRESS`] reaches once initialisation is complete.
pub(crate) const BOOT_COMPLETE: u8 = 0x0d;

/// First byte of the results page.
pub(crate) const RESULTS_BASE: u16 = 0x0400;

/// First byte of the per-test scratch region the ROM's debug menu displays.
pub(crate) const SCRATCH_START: u16 = 0x0500;

/// Last byte of it.
///
/// Past the end of the page: the two stress tests lay out one sample per dot of
/// a 341-dot scanline, which runs to `$0654`, and a dump that stopped at the
/// page boundary would cut it off exactly where the sprite fetches begin.
pub(crate) const SCRATCH_END: u16 = 0x0654;

/// What a result byte says about one test.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum Outcome {
    /// The test has not been run.
    NotRun,
    /// It passed.
    Pass,
    /// It failed, with the error code the ROM would print after "FAIL".
    Fail(u8),
    /// The engine marked it in progress and never came back — the machine hung
    /// inside the test.
    Hung,
}

/// Decode a result byte.
///
/// Bits 0-1 are the state and bits 2-7 the error code, exactly as the ROM's
/// `DrawTEST` routine reads them.
pub(crate) fn decode(byte: u8) -> Outcome {
    match byte & 0x03 {
        0 => Outcome::NotRun,
        1 => Outcome::Pass,
        2 => Outcome::Fail(byte >> 2),
        _ => Outcome::Hung,
    }
}

/// Render an error code the way the ROM prints it: `1`-`9`, then `A`, `B`, ...
///
/// The ROM writes the code straight into the nametable as a tile index, and the
/// character tiles run `0`-`9` then `A`-`Z`, so code 10 shows as `A` and the
/// README's tables for the longer sections run past `F` into `G`, `H`, `I`.
/// A base-16 formatter would misreport every one of those.
pub(crate) fn error_char(code: u8) -> char {
    match code {
        0..=9 => (b'0' + code) as char,
        10..=35 => (b'A' + code - 10) as char,
        _ => '?',
    }
}

/// One entry in the ROM's test table.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) struct Test {
    /// The page it appears on in the menu.
    pub(crate) suite: &'static str,
    /// Its name, as the ROM prints it.
    pub(crate) name: &'static str,
    /// The RAM byte holding its result.
    pub(crate) result: u16,
}

impl Test {
    /// Is this one of the five "DRAW" pages, which assert nothing?
    ///
    /// They are exactly the ones whose result byte lives on page 3, which is
    /// how the ROM's own run-all loop recognises and skips them.
    pub(crate) fn is_draw(&self) -> bool {
        self.result < RESULTS_BASE
    }
}

include!("accuracycoin_tests.rs");

// ---------------------------------------------------------------------------
// Driving the ROM
// ---------------------------------------------------------------------------

/// How long to let the ROM initialise before pressing anything.
const BOOT_FRAMES: u32 = 120;

/// How long Start is held. One frame would do; three survives a machine whose
/// frame boundary and NMI are slightly out of step.
const PRESS_FRAMES: u32 = 3;

/// Frames to wait for the run to start after Start is released.
const START_TIMEOUT_FRAMES: u32 = 120;

/// Frames to wait for a full pass. The ROM waits for vertical blank inside
/// almost every test, so 125 tests take thousands of frames; ten minutes of
/// emulated time is generous and still bounded.
const RUN_TIMEOUT_FRAMES: u32 = 36_000;

/// How the run ended.
#[derive(Debug, Clone, PartialEq, Eq)]
pub(crate) enum RunStatus {
    /// The ROM cleared its "running" flag: every test was attempted.
    Complete,
    /// The ROM never picked up the Start press.
    NeverStarted {
        /// `$00EC`, so the reader can see how far initialisation got.
        boot_progress: u8,
    },
    /// The pass began but never finished.
    TimedOut {
        /// `$0037`, the number of tests that completed.
        completed: u8,
    },
}

/// The outcome of one headless pass.
#[derive(Debug)]
pub(crate) struct Report {
    /// How the run ended.
    pub(crate) status: RunStatus,
    /// Per-test outcomes, in [`TESTS`] order, excluding the DRAW pages.
    pub(crate) results: Vec<(&'static Test, Outcome)>,
    /// The scratch region after the run — the last test's working values.
    pub(crate) scratch: Vec<u8>,
}

impl Report {
    /// Tests that passed.
    pub(crate) fn passed(&self) -> usize {
        self.results
            .iter()
            .filter(|(_, o)| *o == Outcome::Pass)
            .count()
    }

    /// Tests that ran and failed.
    pub(crate) fn failed(&self) -> Vec<(&'static Test, u8)> {
        self.results
            .iter()
            .filter_map(|(t, o)| match o {
                Outcome::Fail(code) => Some((*t, *code)),
                _ => None,
            })
            .collect()
    }

    /// Tests that were never attempted, which is what a partly-built machine
    /// looks like when it hangs or resets partway through.
    pub(crate) fn not_run(&self) -> Vec<&'static Test> {
        self.results
            .iter()
            .filter(|(_, o)| matches!(o, Outcome::NotRun | Outcome::Hung))
            .map(|(t, _)| *t)
            .collect()
    }

    /// A per-suite summary plus every failure with its error code.
    pub(crate) fn describe(&self) -> String {
        let mut s = String::new();
        let _ = writeln!(s, "status: {:?}", self.status);
        let _ = writeln!(
            s,
            "{} of {} tests passed, {} failed, {} never ran",
            self.passed(),
            self.results.len(),
            self.failed().len(),
            self.not_run().len()
        );

        let mut suite = "";
        for (test, outcome) in &self.results {
            if test.suite != suite {
                suite = test.suite;
                let _ = writeln!(s, "\n  {suite}");
            }
            let mark = match outcome {
                Outcome::Pass => "PASS  ".to_string(),
                Outcome::Fail(code) => format!("FAIL {}", error_char(*code)),
                Outcome::NotRun => "----  ".to_string(),
                Outcome::Hung => "HUNG  ".to_string(),
            };
            let _ = writeln!(s, "    {mark}  ${:04X}  {}", test.result, test.name);
        }
        s
    }
}

/// Which test to snapshot RAM for the moment it finishes.
///
/// `RSEMU_AC_WATCH=0450` dumps zero page and the per-test scratch as soon as
/// `$0450` stops reading "not run". Most of these tests measure something into
/// an array and then compare it against a table the ROM carries; the verdict
/// byte says only *that* the comparison failed, so without the array a failure
/// is a dead end. The run-all loop clears the scratch per test, so the dump has
/// to happen the frame the result appears.
pub(crate) const WATCH_ENV: &str = "RSEMU_AC_WATCH";

/// The address named by [`WATCH_ENV`], if it is set to a hex result address.
fn watched() -> Option<u16> {
    let text = std::env::var(WATCH_ENV).ok()?;
    u16::from_str_radix(text.trim().trim_start_matches("0x"), 16).ok()
}

/// Print a labelled hex dump of `range`.
fn dump(machine: &dyn NesMachine, label: &str, range: core::ops::RangeInclusive<u16>) {
    let start = *range.start();
    println!("  {label}:");
    let mut line = String::new();
    for addr in range {
        if (addr - start).is_multiple_of(16) {
            if !line.is_empty() {
                println!("{line}");
            }
            line.clear();
            let _ = write!(line, "    ${addr:04X} ");
        }
        let _ = write!(line, " {:02X}", machine.peek(addr));
    }
    if !line.is_empty() {
        println!("{line}");
    }
}

/// Boot the ROM, press Start at the menu, wait for the pass, read the results.
pub(crate) fn run(machine: &mut dyn NesMachine) -> Report {
    machine.set_controller1(buttons::NONE);
    machine.run_frames(BOOT_FRAMES);

    machine.set_controller1(buttons::START);
    machine.run_frames(PRESS_FRAMES);
    machine.set_controller1(buttons::NONE);

    // The pass runs inside the menu's NMI handler, so `$0035` goes to 1 within
    // a frame or two of the press being seen.
    let mut status = RunStatus::NeverStarted {
        boot_progress: machine.peek(ADDR_BOOT_PROGRESS),
    };
    for _ in 0..START_TIMEOUT_FRAMES {
        if machine.peek(ADDR_RUNNING_ALL) != 0 {
            status = RunStatus::TimedOut { completed: 0 };
            break;
        }
        machine.run_frames(1);
    }

    if matches!(status, RunStatus::TimedOut { .. }) {
        status = RunStatus::TimedOut {
            completed: machine.peek(ADDR_TALLY),
        };
        let watch = watched();
        let mut dumped = false;
        for _ in 0..RUN_TIMEOUT_FRAMES {
            if machine.peek(ADDR_RUNNING_ALL) == 0 {
                status = RunStatus::Complete;
                break;
            }
            if let Some(addr) = watch
                && !dumped
                && machine.peek(addr) != 0
            {
                dumped = true;
                println!(
                    "\n${addr:04X} finished with {:?}; RAM as it stood:",
                    decode(machine.peek(addr))
                );
                dump(machine, "zero page", 0x0000..=0x00ff);
                dump(machine, "scratch", SCRATCH_START..=SCRATCH_END);
                println!();
            }
            machine.run_frames(1);
        }
        if let RunStatus::TimedOut { completed } = &mut status {
            *completed = machine.peek(ADDR_TALLY);
            if let Some(cpu) = machine.cpu_state() {
                println!("  the machine stopped making progress at {cpu}");
            }
        }
    }

    let results = TESTS
        .iter()
        .filter(|t| !t.is_draw())
        .map(|t| (t, decode(machine.peek(t.result))))
        .collect();
    let scratch = (SCRATCH_START..=SCRATCH_END)
        .map(|addr| machine.peek(addr))
        .collect();

    Report {
        status,
        results,
        scratch,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_table_matches_what_the_rom_documents() {
        // 141 tests that assert plus 5 "DRAW" pages. If the table ever drifts
        // from the ROM, the runner is checking the wrong bytes — which is worse
        // than not running at all.
        assert_eq!(TESTS.len(), 146);
        assert_eq!(TESTS.iter().filter(|t| t.is_draw()).count(), 5);
        assert_eq!(TESTS.iter().filter(|t| !t.is_draw()).count(), 141);
    }

    #[test]
    fn every_result_address_is_distinct_and_on_a_page_the_rom_uses() {
        // The DRAW pages share one byte — `result_DrawTest` — because none of
        // them writes a verdict; the tests that assert must not.
        let mut seen: Vec<u16> = TESTS
            .iter()
            .filter(|t| !t.is_draw())
            .map(|t| t.result)
            .collect();
        seen.sort_unstable();
        let before = seen.len();
        seen.dedup();
        assert_eq!(seen.len(), before, "two tests share a result address");
        for t in &TESTS {
            assert!(
                t.result == 0x03ff || (0x0400..=0x04ff).contains(&t.result),
                "{} has result address ${:04X}, which is neither page",
                t.name,
                t.result
            );
        }
    }

    #[test]
    fn the_draw_pages_are_the_ones_the_rom_skips() {
        let draw: Vec<&str> = TESTS
            .iter()
            .filter(|t| t.is_draw())
            .map(|t| t.name)
            .collect();
        assert_eq!(
            draw,
            [
                "PPU Reset Flag",
                "CPU RAM",
                "CPU Registers",
                "PPU RAM",
                "Palette RAM"
            ]
        );
    }

    #[test]
    fn result_bytes_decode_the_way_the_rom_writes_them() {
        assert_eq!(decode(0x00), Outcome::NotRun);
        assert_eq!(decode(0x01), Outcome::Pass);
        assert_eq!(decode(0x03), Outcome::Hung);
        // Error code 1 in bits 2-7, state FAIL in bits 0-1.
        assert_eq!(decode(0x06), Outcome::Fail(1));
        assert_eq!(decode((7 << 2) | 2), Outcome::Fail(7));
        // A pass never carries an error code, even if the upper bits are dirty.
        assert_eq!(decode(0xfd), Outcome::Pass);
    }

    #[test]
    fn error_codes_render_as_the_rom_prints_them() {
        // The README's longer sections run past F: "Unofficial Instructions"
        // goes to K, which is 20.
        assert_eq!(error_char(1), '1');
        assert_eq!(error_char(9), '9');
        assert_eq!(error_char(10), 'A');
        assert_eq!(error_char(15), 'F');
        assert_eq!(error_char(16), 'G');
        assert_eq!(error_char(20), 'K');
    }

    // -----------------------------------------------------------------------
    // A fake that reproduces the ROM's observable protocol, so the driver is
    // tested without a NES.
    // -----------------------------------------------------------------------

    /// Reproduces exactly what the runner watches for: an edge-detected Start
    /// at the menu, `$0035` raised then lowered, results appearing on page 4.
    #[derive(Debug)]
    struct FakeCoin {
        ram: Vec<u8>,
        prev_buttons: u8,
        buttons: u8,
        booted: u32,
        /// Frames the pretend pass takes.
        run_length: u32,
        remaining: Option<u32>,
        /// If set, Start is ignored — a machine that never reaches the menu.
        deaf: bool,
        /// If set, the pass starts and never ends.
        never_finishes: bool,
        /// Result byte written for every test.
        result_byte: u8,
    }

    impl Default for FakeCoin {
        fn default() -> Self {
            FakeCoin {
                ram: vec![0; 0x800],
                prev_buttons: 0,
                buttons: 0,
                booted: 0,
                run_length: 50,
                remaining: None,
                deaf: false,
                never_finishes: false,
                result_byte: 0x01,
            }
        }
    }

    impl NesMachine for FakeCoin {
        fn run_frames(&mut self, frames: u32) {
            for _ in 0..frames {
                self.booted += 1;
                self.ram[usize::from(ADDR_BOOT_PROGRESS)] =
                    BOOT_COMPLETE.min(u8::try_from(self.booted).unwrap_or(BOOT_COMPLETE));
                let pressed = self.buttons & !self.prev_buttons;
                self.prev_buttons = self.buttons;

                if pressed & buttons::START != 0
                    && !self.deaf
                    && self.booted >= BOOT_FRAMES
                    && self.remaining.is_none()
                {
                    self.ram[usize::from(ADDR_RUNNING_ALL)] = 1;
                    self.remaining = Some(self.run_length);
                }

                if let Some(left) = &mut self.remaining {
                    if *left == 0 {
                        if !self.never_finishes {
                            self.ram[usize::from(ADDR_RUNNING_ALL)] = 0;
                            for t in TESTS.iter().filter(|t| !t.is_draw()) {
                                self.ram[usize::from(t.result) & 0x7ff] = self.result_byte;
                            }
                        }
                    } else {
                        *left -= 1;
                        let done = self.run_length - *left;
                        self.ram[usize::from(ADDR_TALLY)] = u8::try_from(done).unwrap_or(u8::MAX);
                    }
                }
            }
        }

        fn set_controller1(&mut self, buttons: u8) {
            self.buttons = buttons;
        }

        fn peek(&self, addr: u16) -> u8 {
            self.ram[usize::from(addr) & 0x7ff]
        }
    }

    #[test]
    fn the_driver_boots_presses_start_and_reads_every_result() {
        let mut machine = FakeCoin::default();
        let report = run(&mut machine);
        assert_eq!(report.status, RunStatus::Complete);
        assert_eq!(report.results.len(), 141);
        assert_eq!(report.passed(), 141);
        assert!(report.failed().is_empty());
        assert!(report.describe().contains("141 of 141 tests passed"));
    }

    #[test]
    fn failures_come_back_with_their_error_codes() {
        // FAIL with error code 3.
        let mut machine = FakeCoin {
            result_byte: (3 << 2) | 2,
            ..FakeCoin::default()
        };
        let report = run(&mut machine);
        assert_eq!(report.status, RunStatus::Complete);
        assert_eq!(report.passed(), 0);
        assert_eq!(report.failed().len(), 141);
        assert!(report.failed().iter().all(|(_, code)| *code == 3));
        assert!(report.describe().contains("FAIL 3"));
    }

    #[test]
    fn a_rom_that_never_reaches_the_menu_is_reported_not_hung_on() {
        let mut machine = FakeCoin {
            deaf: true,
            ..FakeCoin::default()
        };
        let report = run(&mut machine);
        assert_eq!(
            report.status,
            RunStatus::NeverStarted {
                boot_progress: BOOT_COMPLETE
            }
        );
        // A partly-built machine still gets a per-test report, all "not run".
        assert_eq!(report.not_run().len(), 141);
    }

    #[test]
    fn a_pass_that_never_finishes_reports_how_far_it_got() {
        let mut machine = FakeCoin {
            never_finishes: true,
            run_length: 10,
            ..FakeCoin::default()
        };
        let report = run(&mut machine);
        match report.status {
            RunStatus::TimedOut { completed } => assert_eq!(completed, 10),
            other => panic!("expected a timeout, got {other:?}"),
        }
    }

    #[test]
    fn the_scratch_region_is_captured_for_diagnosis() {
        let mut machine = FakeCoin::default();
        let report = run(&mut machine);
        assert_eq!(
            report.scratch.len(),
            usize::from(SCRATCH_END - SCRATCH_START) + 1
        );
    }
}
