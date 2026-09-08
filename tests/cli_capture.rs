//! `rsemu run --capture` takes a character port's output at simulation speed.
//!
//! The flag exists because there was no fast way to get a log out of a guest.
//! `--console` attaches a terminal, and a terminal session is **paced**: the run
//! loop sleeps off whatever each ten-millisecond slice did not use, so a second
//! of guest time costs a second of yours. That is right for a machine somebody
//! is typing at and wrong for a capture — the EDK II boot log that
//! `tests/q35_uefi.rs` reads is 1 400 seconds of guest time, which is 23 minutes
//! of wall clock through a terminal and under two minutes unpaced. `--headless`
//! was already unpaced and printed no character port at all, so between the two
//! of them the log was reachable only slowly.
//!
//! What is asserted here is the binary's own wiring, the way
//! `tests/cli_screenshot.rs` asserts `--screenshot`'s: that the bytes reach the
//! file, that they reach stdout when no file is named, that the run really is
//! unpaced, and that the three ways of asking for something impossible are
//! refused **before** the machine runs rather than after.
//!
//! The Apple 1 is the board for it: one character port, a monitor in ROM that
//! greets the world on that port within a fifth of a virtual second, and no
//! image to fetch.
//!
//! # And the other half of `--capture`: the ports nobody named
//!
//! A run touches every character port the machine opened, not only the one a
//! flag pointed at, because a `CharPort` holds 64 KiB and then pushes back — and
//! a `uart.ns16550` whose port will not take a byte holds it in the transmit
//! register with `THRE` clear, which stops the guest that is writing to it. So
//! an unwatched port is drained and thrown away, and the bytes are counted so
//! that the run says what it discarded instead of losing it silently.
//!
//! That path had no hermetic test, because one port is all the Apple 1 has and
//! every shipped board with two of them wants a firmware image the repository
//! cannot ship. [`machines/tests/two-uarts.machine`] is the board for it — an
//! 8086 with COM1 and COM2 and nothing else, run **by path** rather than by
//! catalog name — and [`a_port_nobody_is_watching_is_drained_rather_than_left_to_fill`]
//! is the test: a guest that writes twice a port's capacity to a port nobody
//! named, and then goes on to say something on the one somebody did.
//!
//! [`machines/tests/two-uarts.machine`]: ../machines/tests/two-uarts.machine

#![cfg(all(feature = "cli", feature = "machine-apple1"))]

use std::path::PathBuf;
use std::process::Command;
use std::time::{Duration, Instant};

/// A scratch path nobody else in this run will pick.
fn scratch(name: &str) -> PathBuf {
    std::env::temp_dir().join(format!("rsemu-capture-{}-{name}", std::process::id()))
}

/// Run the shipped binary and hand back success, stdout, stderr and how long it
/// took.
fn run(args: &[&str]) -> (bool, String, String, Duration) {
    let started = Instant::now();
    let out = Command::new(env!("CARGO_BIN_EXE_rsemu"))
        .args(args)
        .output()
        .expect("the binary this test was built alongside");
    (
        out.status.success(),
        String::from_utf8_lossy(&out.stdout).into_owned(),
        String::from_utf8_lossy(&out.stderr).into_owned(),
        started.elapsed(),
    )
}

/// The bytes go to the file, and the run says how many did.
#[test]
fn a_capture_writes_the_guests_own_bytes_to_a_file() {
    let log = scratch("apple1.log");
    let _ = std::fs::remove_file(&log);
    let (ok, stdout, stderr, _) = run(&[
        "run",
        "apple1",
        "--for",
        "1s",
        "--capture",
        &format!("console={}", log.display()),
    ]);
    assert!(ok, "the run failed\n{stderr}");

    let text = std::fs::read_to_string(&log).expect("--capture wrote the file it was given");
    assert!(
        text.contains("RSMON"),
        "the monitor greets the world on the console within a fifth of a virtual second, and \
         what was captured is {text:?}"
    );
    // Announced before the run, so a person watching a long one knows where the
    // bytes are going.
    assert!(
        stderr.contains("capturing `console` to"),
        "the run said nothing about what it was capturing:\n{stderr}"
    );
    // And counted after it, beside `--screenshot`'s line and `--record-audio`'s.
    assert!(
        stdout.contains("capture ") && stdout.contains("(`console`, "),
        "the summary does not report the capture:\n{stdout}"
    );
    let _ = std::fs::remove_file(&log);
}

/// No `=<file>` is stdout, which is what a pipeline wants.
#[test]
fn a_capture_with_no_file_goes_to_stdout() {
    let (ok, stdout, stderr, _) =
        run(&["run", "apple1", "--for", "1s", "-q", "--capture", "console"]);
    assert!(ok, "the run failed\n{stderr}");
    assert!(
        stdout.contains("RSMON"),
        "the guest's own bytes did not reach stdout:\n{stdout}"
    );
}

/// The point of the flag: the machine is **not** held to wall clock.
///
/// Calibrated rather than assumed. A paced run cannot finish in less than the
/// span it was given — it sleeps off the difference — so "finished in less than
/// the span" is exactly the property under test. But a host that simulates an
/// Apple 1 slower than an Apple 1 runs cannot demonstrate it either way, and
/// failing on such a host would be a flake rather than a defect. So the first
/// run measures, and the assertion is made only where there is room to make it.
#[test]
fn a_capture_runs_the_machine_unpaced() {
    let log = scratch("paced.log");
    let (ok, _, stderr, one) = run(&[
        "run",
        "apple1",
        "--for",
        "1s",
        "-q",
        "--capture",
        &format!("console={}", log.display()),
    ]);
    assert!(ok, "the run failed\n{stderr}");
    if one >= Duration::from_millis(500) {
        println!(
            "cli_capture: this host took {one:?} to simulate one second of a 1 MHz 6502, which \
             is too close to real time for `unpaced` and `paced` to be told apart; the timing \
             claim is skipped rather than flaked"
        );
        let _ = std::fs::remove_file(&log);
        return;
    }

    // Ten times the virtual time, on a host that has just shown it manages a
    // second in under half of one.
    let span = Duration::from_secs(10);
    let (ok, stdout, stderr, ten) = run(&[
        "run",
        "apple1",
        "--for",
        "10s",
        "-q",
        "--capture",
        &format!("console={}", log.display()),
    ]);
    assert!(ok, "the run failed\n{stderr}");
    assert!(
        stdout.contains("ran to 10000000000 ns"),
        "the run did not cover the span it was given:\n{stdout}"
    );
    assert!(
        ten < span,
        "ten seconds of guest time took {ten:?} of wall clock, which is what a *paced* run \
         costs; the capture loop is supposed to be the unpaced one (one second took {one:?})"
    );
    let _ = std::fs::remove_file(&log);
}

/// A port this machine never opened is a refusal, and the refusal says which
/// ports it did open.
#[test]
fn a_port_this_machine_never_opened_is_refused() {
    let (ok, _, stderr, _) = run(&["run", "apple1", "--for", "1s", "--capture", "debug"]);
    assert!(!ok, "a capture of a port that does not exist succeeded");
    assert!(
        stderr.contains("--capture debug") && stderr.contains("`console`"),
        "the refusal does not say what this machine actually opened:\n{stderr}"
    );
}

/// A terminal and a capture on one port would each get half the bytes, because
/// a `CharPort` hands each byte to whoever asks first.
#[test]
fn a_terminal_and_a_capture_on_one_port_are_refused() {
    let (ok, _, stderr, _) = run(&[
        "run",
        "apple1",
        "--console",
        "console",
        "--capture",
        "console",
    ]);
    assert!(!ok, "two listeners on one port were accepted");
    assert!(
        stderr.contains("two listeners on one port"),
        "the refusal does not explain itself:\n{stderr}"
    );
}

/// Two ports interleaved on stdout cannot be told apart afterwards, and the fix
/// is one character long.
#[test]
fn two_captures_cannot_share_stdout() {
    let (ok, _, stderr, _) = run(&[
        "run",
        "apple1",
        "--capture",
        "console",
        "--capture",
        "keyboard",
    ]);
    assert!(!ok, "two ports were pointed at one stream");
    assert!(
        stderr.contains("both go to stdout"),
        "the refusal does not explain itself:\n{stderr}"
    );
}

// ---------------------------------------------------------------------------
// the ports nobody named
// ---------------------------------------------------------------------------

/// The fixture board with two character ports, run by path.
///
/// Not a catalog name: it is not a machine anybody ships, and `machines/tests/`
/// is where a board that exists for one test lives. The absolute path is built
/// here rather than relying on the working directory, which is not something a
/// test binary should assume.
#[cfg(all(feature = "cpu-x86", feature = "dev-uart-ns16550"))]
const TWO_UARTS: &str = concat!(
    env!("CARGO_MANIFEST_DIR"),
    "/machines/tests/two-uarts.machine"
);

/// How many bytes the guest writes to the port nobody is watching.
///
/// **Twice `rsemu::host::chardev::PORT_CAPACITY`**, and that is the whole design
/// of the test: a run that did not drain that port would fill it after 64 KiB,
/// the 16550 would hold the next byte in its transmit register with `THRE`
/// clear, and the guest — which polls that bit — would still be in this loop
/// when the clock ran out. It would therefore never reach [`AFTER`]. Anything
/// less than a port's capacity would pass whether the port was drained or not.
#[cfg(all(feature = "cpu-x86", feature = "dev-uart-ns16550"))]
const BLAST: usize = 2 * 64 * 1024;

/// What the guest says on COM1 before it starts.
#[cfg(all(feature = "cpu-x86", feature = "dev-uart-ns16550"))]
const BEFORE: &str = "com1 before";

/// And after it has finished, which is the line that cannot appear if the
/// unwatched port stalled it.
#[cfg(all(feature = "cpu-x86", feature = "dev-uart-ns16550"))]
const AFTER: &str = "com1 after 131072 bytes on com2";

/// The 8086 program the fixture board boots: talk on COM1, blast COM2, talk on
/// COM1 again, halt.
///
/// Hand-assembled rather than fetched, which is what makes this hermetic —
/// there is no toolchain here and nothing is downloaded. Every instruction is
/// register-to-register or an I/O access, so the program needs no stack, no data
/// segment and no memory at all; the only address it depends on is the one the
/// processor fetches out of reset.
///
/// ```text
///         cli
///         say "com1 before\r\n" on 0x3f8
///         mov  bx, 2
/// round:  mov  cx, 0                  ; 0 means 65536 times
/// inner:  mov  dx, 0x2fd              ; COM2's line status
///   poll: in   al, dx
///         test al, 0x20               ; THRE: is the holding register free?
///         jz   poll                   ; no — wait, exactly as a boot ROM does
///         mov  dx, 0x2f8
///         mov  al, 'x'
///         out  dx, al
///         loop inner
///         dec  bx
///         jnz  round
///         say "com1 after …\r\n" on 0x3f8
/// stop:   hlt
///         jmp  stop
/// ```
///
/// The image is the full 64 KiB of the socket, because the reset vector has to
/// land at the top of it: an 8086 fetches its first instruction from
/// `FFFF:0000`, which on this board is sixteen bytes from the end of the ROM,
/// and what is there is `JMP F000:0000` — the far jump that puts `CS` where the
/// program is. (Intel 8086 Family User's Manual: reset, and the intersegment
/// direct `JMP`.)
#[cfg(all(feature = "cpu-x86", feature = "dev-uart-ns16550"))]
fn two_uarts_rom() -> Vec<u8> {
    /// COM1, the port a `--capture` names in the first of the two tests.
    const COM1: u16 = 0x3f8;
    /// COM2, the one nothing is listening to in that test.
    const COM2: u16 = 0x2f8;

    /// `mov dx, imm16`.
    fn mov_dx(code: &mut Vec<u8>, port: u16) {
        code.push(0xba);
        code.extend_from_slice(&port.to_le_bytes());
    }

    /// Spin until the transmitter of the UART at `base` is free.
    ///
    /// The `jz` goes back to the `in`, not to the `mov dx` before it: three
    /// bytes fewer per turn of a loop this program takes 131 072 times.
    fn wait_thre(code: &mut Vec<u8>, base: u16) {
        mov_dx(code, base + 5);
        code.push(0xec); // in al, dx
        code.extend_from_slice(&[0xa8, 0x20]); // test al, 0x20
        code.extend_from_slice(&[0x74, 0xfb]); // jz -5
    }

    /// Wait for the transmitter, then hand it one byte.
    fn put(code: &mut Vec<u8>, base: u16, byte: u8) {
        wait_thre(code, base);
        mov_dx(code, base);
        code.extend_from_slice(&[0xb0, byte]); // mov al, imm8
        code.push(0xee); // out dx, al
    }

    /// The same, for a whole line.
    fn say(code: &mut Vec<u8>, base: u16, text: &str) {
        for byte in text.bytes() {
            put(code, base, byte);
        }
    }

    let mut code: Vec<u8> = Vec::new();
    code.push(0xfa); // cli
    say(&mut code, COM1, &format!("{BEFORE}\r\n"));

    // Two rounds of a `loop` that counts 65 536 times, because `CX` is sixteen
    // bits and one round of it is exactly a port's capacity.
    assert_eq!(BLAST, 2 * 0x10000, "the ROM emits two full CX rounds");
    code.extend_from_slice(&[0xbb, 0x02, 0x00]); // mov bx, 2
    let round = code.len();
    code.extend_from_slice(&[0xb9, 0x00, 0x00]); // mov cx, 0
    let inner = code.len();
    wait_thre(&mut code, COM2);
    mov_dx(&mut code, COM2);
    code.extend_from_slice(&[0xb0, b'x']); // mov al, 'x'
    code.push(0xee); // out dx, al
    let back = i8::try_from(inner as isize - (code.len() as isize + 2)).expect("a near loop");
    code.extend_from_slice(&[0xe2, back as u8]); // loop inner
    code.push(0x4b); // dec bx
    let back = i8::try_from(round as isize - (code.len() as isize + 2)).expect("a near loop");
    code.extend_from_slice(&[0x75, back as u8]); // jnz round

    say(&mut code, COM1, &format!("{AFTER}\r\n"));
    let stop = code.len();
    code.push(0xf4); // hlt
    let back = i8::try_from(stop as isize - (code.len() as isize + 2)).expect("a near jump");
    code.extend_from_slice(&[0xeb, back as u8]); // jmp stop

    // The socket, with the program at the bottom and the reset vector at the
    // top. Everything between is the zero the `rom` class pads with.
    let mut image = vec![0u8; 0x10000];
    assert!(code.len() < 0xfff0, "the program overruns the reset vector");
    image[..code.len()].copy_from_slice(&code);
    image[0xfff0..0xfff5].copy_from_slice(&[0xea, 0x00, 0x00, 0x00, 0xf0]);
    image
}

/// Put the fixture board's ROM somewhere the binary can read it.
#[cfg(all(feature = "cpu-x86", feature = "dev-uart-ns16550"))]
fn two_uarts_image(name: &str) -> PathBuf {
    let path = scratch(name);
    std::fs::write(&path, two_uarts_rom()).expect("a scratch file for the fixture ROM");
    path
}

/// A guest that writes twice a port's capacity to a port nothing is watching
/// keeps running, and the run says how much it threw away.
///
/// The claim under test is the one that cannot be made on a one-port board: a
/// `--capture` names COM1, nothing names COM2, and COM2 is drained anyway. The
/// discrimination is [`BLAST`] — 128 KiB against a `CharPort`'s 64 — so a run
/// that left the unwatched port to fill would stop in the middle of that loop
/// with [`AFTER`] never printed, whatever else it reported.
#[test]
#[cfg(all(feature = "cpu-x86", feature = "dev-uart-ns16550"))]
fn a_port_nobody_is_watching_is_drained_rather_than_left_to_fill() {
    let rom = two_uarts_image("com1.rom");
    let log = scratch("two-uarts-com1.log");
    let _ = std::fs::remove_file(&log);
    let (ok, stdout, stderr, _) = run(&[
        "run",
        TWO_UARTS,
        "--media",
        &format!("firmware={}", rom.display()),
        // Ten virtual seconds against the second and a half the program needs,
        // so a slower accounting of the same instructions is still inside it.
        "--for",
        "10s",
        "--capture",
        &format!("com1={}", log.display()),
    ]);
    assert!(ok, "the run failed\n{stderr}");

    // Said before the run, so a person watching a long one knows that a stream
    // is being thrown away, and which.
    assert!(
        stderr.contains("`com2` drained and discarded"),
        "the run did not say what it was discarding:\n{stderr}"
    );

    let text = std::fs::read_to_string(&log).expect("--capture wrote the file it was given");
    assert!(
        text.contains(BEFORE),
        "the guest never reached its own COM1; what was captured is {text:?}"
    );
    assert!(
        text.contains(AFTER),
        "the guest wrote {BLAST} bytes to a port nothing was listening to and never came back \
         from it — which is exactly what a run that let that port fill looks like, because the \
         16550 holds the byte it cannot deliver with THRE clear. What COM1 got was {text:?}"
    );

    // And counted after it, so a discarded stream is visible rather than
    // silent. The number is the guest's own: it wrote that many bytes.
    assert!(
        stdout.contains(&format!("discarded   {BLAST} bytes")) && stdout.contains("`com2`"),
        "the summary does not account for what was thrown away:\n{stdout}"
    );
    let _ = std::fs::remove_file(&log);
    let _ = std::fs::remove_file(&rom);
}

/// The same board with the flag on the other port: what was discarded before is
/// captured now, and the port that was captured is discarded.
///
/// The pair is the point. Nothing about this board decides which stream is
/// interesting — `--capture` does, and every port it does not name is drained
/// and counted, whichever one that is.
#[test]
#[cfg(all(feature = "cpu-x86", feature = "dev-uart-ns16550"))]
fn the_port_a_capture_does_not_name_is_the_one_that_is_discarded() {
    let rom = two_uarts_image("com2.rom");
    let log = scratch("two-uarts-com2.log");
    let _ = std::fs::remove_file(&log);
    let (ok, stdout, stderr, _) = run(&[
        "run",
        TWO_UARTS,
        "--media",
        &format!("firmware={}", rom.display()),
        "--for",
        "10s",
        "--capture",
        &format!("com2={}", log.display()),
    ]);
    assert!(ok, "the run failed\n{stderr}");
    assert!(
        stderr.contains("`com1` drained and discarded"),
        "the other port is the discarded one now:\n{stderr}"
    );

    let bytes = std::fs::read(&log).expect("--capture wrote the file it was given");
    assert_eq!(
        bytes.len(),
        BLAST,
        "the guest wrote {BLAST} bytes to COM2 and the capture has {}",
        bytes.len()
    );
    assert!(
        bytes.iter().all(|b| *b == b'x'),
        "the capture holds something other than what the program sent"
    );
    // COM1's two lines are what is thrown away this time, and they are what the
    // summary has to name.
    assert!(
        stdout.contains("`com1`") && stdout.contains("discarded   "),
        "the summary does not account for what was thrown away:\n{stdout}"
    );
    let _ = std::fs::remove_file(&log);
    let _ = std::fs::remove_file(&rom);
}
