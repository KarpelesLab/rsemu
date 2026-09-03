//! **An interrupted run keeps what it accepted.**
//!
//! `rsemu run`'s `finish` flushes every drive on the way out, and every return
//! from `main` goes through it — but a signal is not a return. Before
//! `host::signal` existed, Ctrl-C on a headless run was `SIGINT` with its
//! default disposition: the process was gone before `finish` could run, and on
//! a qcow2 that is not staleness but inconsistency. The data clusters are
//! written through to the file as the guest writes them; the L1 and L2 tables
//! that say *where* they are live in `fstool`'s write-back cache until
//! something calls `sync`. Lose those and the image has the bytes and no way
//! to find them.
//!
//! That distinction is what this file is built around, because — as
//! `tests/medium_contract.rs` puts it for a different pair of defects — two
//! failures with one symptom need two assertions:
//!
//! * the **data cluster** reached the file, which is proved by finding the
//!   guest's marker in the image's raw bytes, and
//! * the **metadata** reached it too, which is proved by reopening the image
//!   as a qcow2 and reading the sector back through its L1/L2 tables.
//!
//! The first is also the test's synchronisation: it is how a spawned process
//! is known to have got as far as writing, without a sleep that is either
//! flaky or slow.
//!
//! # The control
//!
//! [`a_killed_run_loses_the_metadata_an_interrupted_one_keeps`] does the same
//! run and sends `SIGKILL`, which no in-process mechanism can survive, and
//! asserts the sector is **not** readable. Without it this file could pass on
//! a qcow2 that never needed flushing at all, and the whole thing would be
//! asserting nothing. If that control ever starts failing, the honest reading
//! is that something now flushes earlier — not that the test is wrong.
//!
//! # Why a subprocess, and why `sh -c kill`
//!
//! Signals are delivered to a process, so the thing under test has to be one.
//! `std` cannot send a signal either (`Child::kill` is `SIGKILL` and nothing
//! else), and `libc` is not a dependency this project has — so the signal is
//! sent the way `host::terminal` reaches raw mode, by running the program
//! every Unix already has. `kill` is a shell builtin, which is why it is
//! spelled `sh -c` rather than looked up on `PATH`.
//!
//! Signals are a host facility, so this whole file is native-only: it compiles
//! away on wasm, on any host that is not x86-64 Linux, and under
//! `--no-default-features`, rather than failing there. `cli` rather than `std`
//! because the binary this drives is the `cli` feature's target, and a test
//! that spawned a binary the build never produced would fail for a reason that
//! has nothing to do with signals.

#![cfg(all(
    feature = "cli",
    feature = "machine-apple1",
    target_os = "linux",
    target_arch = "x86_64"
))]

use std::process::{Child, Command, Stdio};
use std::time::{Duration, Instant};

/// How long to wait for an interrupted run to finish flushing and exit.
const SHUTDOWN: Duration = Duration::from_secs(60);

/// A child that is killed if the test panics before it is waited on.
///
/// A leaked `rsemu run --for 10m` on a build machine is somebody else's
/// problem an hour later, so it is this file's problem now.
struct Run(Option<Child>);

impl Run {
    fn pid(&self) -> u32 {
        self.0.as_ref().expect("the child is still here").id()
    }

    /// Send `signal` — `INT`, `TERM`, `HUP`, `KILL` — to the child, `times`
    /// over, from **one** shell.
    ///
    /// One shell rather than one per signal, and that is the difference
    /// between a test and a flake. Two `Command::new("sh")` calls are a fork
    /// and an exec apart — a couple of milliseconds — which is long enough for
    /// a run to notice the first signal, flush, and exit, and then the second
    /// `kill` has nothing to send to. `kill` is a builtin, so two of them in
    /// one shell land microseconds apart, inside the slice the run is still
    /// executing.
    fn signal_times(&self, signal: &str, times: usize) {
        let pid = self.pid();
        let script = vec![format!("kill -{signal} {pid}"); times].join("; ");
        let status = Command::new("sh")
            .arg("-c")
            .arg(&script)
            .status()
            .expect("sh is on this host");
        assert!(status.success(), "`{script}` refused");
    }

    /// Send `signal` once.
    fn signal(&self, signal: &str) {
        self.signal_times(signal, 1);
    }

    /// Wait for it to exit, or panic saying it did not.
    fn finish(&mut self, within: Duration) -> std::process::ExitStatus {
        let mut child = self.0.take().expect("waited on once");
        let deadline = Instant::now() + within;
        loop {
            match child.try_wait().expect("waiting on the child") {
                Some(status) => return status,
                None if Instant::now() >= deadline => {
                    let _ = child.kill();
                    let _ = child.wait();
                    panic!("the run did not exit within {within:?} of being signalled");
                }
                None => std::thread::sleep(Duration::from_millis(20)),
            }
        }
    }
}

impl Drop for Run {
    fn drop(&mut self) {
        if let Some(child) = self.0.as_mut() {
            let _ = child.kill();
            let _ = child.wait();
        }
    }
}

/// Spawn `rsemu run` with `args`, saying nothing to anybody.
fn spawn(args: &[&str]) -> Run {
    let child = Command::new(env!("CARGO_BIN_EXE_rsemu"))
        .arg("run")
        .args(args)
        .stdin(Stdio::null())
        .stdout(Stdio::null())
        .stderr(Stdio::null())
        .spawn()
        .expect("the binary this test was built beside");
    Run(Some(child))
}

// ---------------------------------------------------------------------------
// the cheap half: a run that is asked to stop, stops
// ---------------------------------------------------------------------------

/// Every signal `host::signal` takes over ends a run the same way: cleanly,
/// with the exit status of a run that finished.
///
/// `apple1` because it is the smallest machine in the tree that runs for as
/// long as it is asked to; what is under test is the process, not the board.
#[test]
fn each_shutdown_signal_ends_a_headless_run_with_a_successful_exit() {
    for signal in ["INT", "TERM", "HUP"] {
        let mut run = spawn(&["apple1", "--headless", "--quiet", "--for", "10m"]);
        // A run that has produced its first virtual microsecond is a run whose
        // loop is turning; there is no file to watch here, so give the process
        // its startup and then ask.
        std::thread::sleep(Duration::from_millis(300));
        run.signal(signal);
        let status = run.finish(SHUTDOWN);
        assert!(
            status.success(),
            "SIG{signal} should end a run through `finish`, not around it; got {status}"
        );
    }
}

/// The second one is not swallowed.
///
/// `SA_RESETHAND` puts the default disposition back before the handler runs,
/// so a user who decides the clean stop is taking too long gets what Ctrl-C
/// has always given them. Proved by the *absence* of a clean exit: a process
/// killed by `SIGINT` reports no exit code at all.
#[test]
fn a_second_interrupt_kills_the_process_rather_than_being_ignored() {
    let mut run = spawn(&["apple1", "--headless", "--quiet", "--for", "10m"]);
    std::thread::sleep(Duration::from_millis(300));
    // Both from one shell, microseconds apart: see `signal_times`. The first
    // is delivered to a running process at once, so the handler has entered —
    // and `SA_RESETHAND` has already put the default back — before the second
    // is sent. Two signals a fork apart would let the run exit in between and
    // prove nothing.
    run.signal_times("INT", 2);
    let status = run.finish(SHUTDOWN);
    assert!(
        status.code().is_none(),
        "a second SIGINT must reach the default disposition; the run exited {status} instead"
    );
}

// ---------------------------------------------------------------------------
// the honest half: the image on disk
// ---------------------------------------------------------------------------

/// Everything below needs a guest that writes to a drive, which means a board
/// with a CPU, a BIOS and an IDE cable on it.
#[cfg(all(
    feature = "cpu-x86",
    feature = "dev-blk",
    feature = "dev-pc",
    feature = "dev-pc-apic",
    feature = "dev-pc-video",
    feature = "dev-pc-floppy",
    feature = "dev-pc-ide",
    feature = "dev-pc-hpet",
    feature = "fw-pcbios",
    feature = "machine-pc-at"
))]
mod on_disk {
    use std::path::{Path, PathBuf};
    use std::sync::atomic::{AtomicU64, Ordering};
    use std::time::{Duration, Instant};

    use super::{Run, SHUTDOWN, spawn};

    use rsemu::dev::blk::{Image, ImageOptions};
    use rsemu::dev::medium::Medium;
    use rsemu::fw::asm16::{AX, Asm, BX, CX, DS, DX, ES, Mem, SP, SS};

    /// How long to wait for a spawned run to reach a state the test is looking
    /// for. Generous: a debug-profile `pc-at` POSTs and boots a diskette in
    /// well under this, with room for a loaded build machine.
    const PATIENCE: Duration = Duration::from_secs(120);

    /// A scratch path nobody else is using, for this process and this call.
    fn scratch(name: &str) -> PathBuf {
        static NEXT: AtomicU64 = AtomicU64::new(0);
        let mut path = std::env::temp_dir();
        path.push(format!(
            "rsemu-cli-interrupt-{}-{}-{name}",
            std::process::id(),
            NEXT.fetch_add(1, Ordering::Relaxed)
        ));
        path
    }

    /// Whether `needle` appears anywhere in the file's raw bytes.
    ///
    /// A qcow2's data clusters are written through as the guest writes them,
    /// so this answers "did the guest's write reach the host" without saying
    /// anything about whether the image can still be *read*.
    fn file_contains(path: &Path, needle: &[u8]) -> bool {
        let Ok(bytes) = std::fs::read(path) else {
            return false;
        };
        bytes.windows(needle.len()).any(|w| w == needle)
    }

    /// Wait until `f` answers true, or panic with `what`.
    fn wait_for(what: &str, mut f: impl FnMut() -> bool) {
        let deadline = Instant::now() + PATIENCE;
        while !f() {
            assert!(Instant::now() < deadline, "{what} within {PATIENCE:?}");
            std::thread::sleep(Duration::from_millis(50));
        }
    }

    /// Where the BIOS puts a boot sector.
    const BOOT: u16 = 0x7c00;
    /// The block at `0x0500` every PC has left free since 1981; the `INT 13h`
    /// status lands there so a failed write is distinguishable from a write
    /// that never happened, when anybody is looking with a debugger.
    const STATUS: u16 = 0x0500;
    /// A byte string that appears nowhere else on the host: what the raw scan
    /// looks for.
    const MARKER: &[u8; 16] = b"rsemu:kept-this!";
    /// Where the guest writes: cylinder 0, head 0, sector 2 — LBA 1, the
    /// sector after the master boot record, whatever geometry the drive
    /// reports.
    const SECTOR_AT: u64 = 512;

    /// A boot sector that writes **itself** to the fixed disk and then stops.
    ///
    /// Itself, so that what the image should hold afterwards needs no second
    /// definition: this function is both the program and the expectation.
    ///
    /// `INT 13h AH=03h`, `AL=1` sector, `CH=0`/`CL=2` for cylinder 0 sector 2,
    /// `DH=0`, `DL=80h` for the first fixed disk, buffer at `ES:BX` = the
    /// sector itself (IBM PC/AT BIOS interface, `INT 13h`). Deliberately **no**
    /// `FLUSH CACHE` afterwards: a guest is entitled to leave its writes in the
    /// drive's cache, and the whole point of `Machine::flush` is that the run
    /// keeps them anyway.
    fn boot_sector() -> Vec<u8> {
        let mut a = Asm::new(usize::from(BOOT) + 512, 0x00);
        a.seek(BOOT);

        a.cli();
        a.movi(AX, 0);
        a.movsr(DS, AX);
        a.movsr(ES, AX);
        a.movsr(SS, AX);
        a.movi(SP, BOOT);
        a.sti();

        a.movi(AX, 0x0301); // AH=03h write, AL=1 sector
        a.movi(CX, 0x0002); // CH=0 cylinder, CL=2 sector
        a.movi(DX, 0x0080); // DH=0 head, DL=80h first fixed disk
        a.movi(BX, BOOT); // ES:BX — this very sector
        a.int(0x13);
        a.movto(Mem::abs(STATUS), AX);

        let spin = a.here_label();
        a.hlt();
        a.jmp(spin);

        // Unreachable, and that is what makes it a good marker: nothing
        // executes it and nothing else in the tree spells it.
        a.db(MARKER);

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

    /// A 1.44 MB diskette holding that sector, written to a scratch path.
    fn diskette(name: &str) -> std::path::PathBuf {
        let mut image = boot_sector();
        assert_eq!(image.len(), 512, "a boot sector is one sector");
        image.resize(1_474_560, 0);
        let path = scratch(name);
        std::fs::write(&path, &image).expect("writing the diskette");
        path
    }

    /// Start `pc-at` with that diskette and a fresh qcow2 on the first IDE bay,
    /// and wait until the guest's sector is in the file's raw bytes.
    ///
    /// Returns the running child and the image path. At the moment it returns,
    /// the data cluster is on the host and the metadata that finds it is not —
    /// which is exactly the state an unflushed exit leaves behind for good.
    fn run_until_the_guest_has_written(tag: &str) -> (Run, std::path::PathBuf) {
        let floppy = diskette(&format!("{tag}.img"));
        let image = scratch(&format!("{tag}.qcow2"));
        let drive = format!("hd0={},new=8M", image.display());
        let run = spawn(&[
            "pc-at",
            "--headless",
            "--quiet",
            "--for",
            "10m",
            "--floppy",
            floppy.to_str().expect("a utf-8 scratch path"),
            "--drive",
            &drive,
        ]);
        wait_for("the guest writes its sector to the host file", || {
            file_contains(&image, MARKER)
        });
        // The CLI read the diskette into memory before the machine was built,
        // so nothing needs the file from here on.
        let _ = std::fs::remove_file(&floppy);
        (run, image)
    }

    /// What the image holds at LBA 1, read back through qcow2's own tables.
    ///
    /// `None` when the sector cannot be found — which is what an image whose
    /// L2 entry never reached the file looks like: the cluster is unallocated,
    /// so the format's answer is a hole, and a hole reads as zeros.
    fn sector_after_reopening(path: &std::path::Path) -> Option<Vec<u8>> {
        let image = Image::open(path, &ImageOptions::new().read_only(true))
            .expect("the image still opens as a qcow2");
        let mut got = vec![0u8; 512];
        image
            .read_at(SECTOR_AT, &mut got)
            .expect("a sector inside an 8 MiB drive");
        got.iter().any(|&b| b != 0).then_some(got)
    }

    /// The defect, and the fix: a `SIGINT` mid-run reaches `finish`, `finish`
    /// flushes, and the qcow2 can still find the guest's sector afterwards.
    #[test]
    fn an_interrupted_run_leaves_a_qcow2_that_can_still_find_the_guests_sector() {
        let (mut run, image) = run_until_the_guest_has_written("interrupted");
        run.signal("INT");
        let status = run.finish(SHUTDOWN);
        assert!(
            status.success(),
            "an interrupted run flushes and exits zero; got {status}"
        );

        let got = sector_after_reopening(&image)
            .expect("LBA 1 reads as a hole: the L1/L2 tables never reached the file");
        assert_eq!(
            got,
            boot_sector(),
            "the sector is there but is not what the guest wrote"
        );
        let _ = std::fs::remove_file(&image);
    }

    /// The control, and the reason the test above is worth anything.
    ///
    /// `SIGKILL` cannot be handled, so nothing in the process flushes: the
    /// marker is in the file — the cluster was written through — and the
    /// format cannot reach it. If this ever passes *and* the sector is
    /// readable, the assertion above has stopped proving the flush happened.
    #[test]
    fn a_killed_run_loses_the_metadata_an_interrupted_one_keeps() {
        let (mut run, image) = run_until_the_guest_has_written("killed");
        run.signal("KILL");
        let status = run.finish(SHUTDOWN);
        assert!(status.code().is_none(), "SIGKILL is not survivable");

        assert!(
            file_contains(&image, MARKER),
            "the data cluster should be in the file; this control tests the metadata, \
             and it cannot if the guest never wrote"
        );
        assert!(
            sector_after_reopening(&image).is_none(),
            "an unflushed qcow2 should have a hole where LBA 1 is — if it does not, \
             something now flushes without being asked and the interrupted-run test \
             above is no longer proving anything"
        );
        let _ = std::fs::remove_file(&image);
    }
}
