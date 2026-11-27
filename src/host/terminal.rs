//! The process's own terminal, as a [`CharDevice`].
//!
//! This is the backend that makes a guest's console visible: stdin becomes
//! bytes the guest can read, and bytes the guest writes become stdout. Pure
//! `std` — no `libc`, no `termios` crate, nothing outside the dependency
//! budget (`CLAUDE.md`).
//!
//! # Raw mode, and how it is reached without `libc`
//!
//! An interactive guest console wants **raw mode**: no line buffering, no host
//! echo, one byte per keystroke. `std` exposes no terminal control at all, so
//! the only pure-`std` route to it is the program every Unix already has:
//!
//! ```text
//! stty -g          → an opaque string describing the current settings
//! stty raw -echo   → character-at-a-time, host echo off
//! stty <saved>     → put it back, on the way out
//! ```
//!
//! [`Terminal::open`] runs those with stdin inherited, so `stty` acts on this
//! process's controlling terminal. When it works, [`Terminal::is_raw`] is true
//! and every keystroke reaches the guest immediately. When it does not — no
//! `stty` on the host, stdin redirected from a file or a pipe, Windows — the
//! terminal stays **cooked**: the host buffers a line and echoes it, the guest
//! sees the whole line when Return is pressed, and it echoes the line a second
//! time. That is degraded, not broken, and [`Terminal::is_raw`] says which one
//! you got so a caller can tell the user.
//!
//! Raw mode is restored by [`Terminal`]'s `Drop`, and by [`restore`] on the two
//! ways out that never reach a `Drop`:
//!
//! * **A second shutdown signal.** [`signal`](super::signal) hands the first
//!   one to the run loop and, by design, lets the second kill the process. A
//!   run loop therefore calls [`restore`] as soon as it sees the first, before
//!   it starts flushing — so the kill, if it comes, leaves a cooked terminal.
//!   What is left is the instant between the signal and the loop noticing it
//!   (a slice of virtual time at most): closing that would mean changing
//!   terminal settings from inside a signal handler.
//! * **A panic.** Entering raw mode installs a panic hook that restores the
//!   terminal *before* the message is printed, so the message is not
//!   stair-stepped down a raw screen and the shell that follows is usable.
//!
//! A hard kill (`SIGKILL`) skips all of it, as it would with any program;
//! `stty sane` fixes it.
//!
//! # Why a thread
//!
//! `std` has no non-blocking stdin either, and a `CharDevice` must never block
//! — it is read from inside the scheduler, where blocking stops virtual time.
//! So one thread does nothing but block on stdin and hand bytes over a channel.
//! `CLAUDE.md`'s "submit jobs, never spawn threads" governs `core/`, `dev/`,
//! `machine/` and `ir/`; a permanently-blocked reader cannot go on the task
//! pool without occupying a worker forever, and `host/` is where `std` and its
//! threads are allowed to be.

use std::collections::VecDeque;
use std::io::{Read, Write};
use std::process::{Command, Stdio};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::mpsc::{Receiver, TryRecvError, channel};
use std::sync::{Mutex, Once, OnceLock};

use super::chardev::{CharDevice, CharPort};

/// The byte a user presses to get out: Ctrl-C.
///
/// In raw mode the kernel no longer turns this into a signal, so a guest that
/// never exits would trap the terminal forever. It is consumed here rather than
/// forwarded, and shows up as [`Terminal::interrupted`].
const INTERRUPT: u8 = 0x03;

/// The `stty -g` string that undoes raw mode: the settings the terminal had
/// the first time this process made it raw.
///
/// Process-wide rather than a field of [`Terminal`], because the two callers
/// that most need it have no `Terminal` to hand: a panic hook, and a run loop
/// that has just been told a signal arrived. There is one controlling terminal
/// per process, so there is one of these.
///
/// Written once and never locked. The first entry into raw mode is the one
/// that saw what the shell left; a later one, made while the terminal is
/// already raw, would only record raw settings as the thing to go back to.
/// And a reader that cannot block is what a panic hook needs — a lock here
/// would be one more thing to be holding when the panic happened.
static COOKED: OnceLock<String> = OnceLock::new();

/// Whether the terminal is raw and [`COOKED`] is owed to it.
///
/// Whoever swaps this to `false` does the restoring, so two callers racing —
/// a panic hook and a `Drop`, say — run `stty` once between them.
static RAW: AtomicBool = AtomicBool::new(false);

/// Put the terminal back the way [`Terminal::open`] found it, now.
///
/// Does nothing if it was never made raw or has already been restored, so it
/// is safe to call from every path that might be the last one: the first
/// sight of a shutdown signal, a panic hook, and `Drop`. A [`Terminal`] that
/// is still alive keeps working afterwards, as a cooked one.
pub fn restore() {
    if RAW.swap(false, Ordering::SeqCst)
        && let Some(saved) = COOKED.get()
    {
        let _ = stty(&[saved]);
    }
}

/// Restore the terminal before a panic message is printed.
///
/// Chained in front of whatever hook was there, once per process, and only
/// once raw mode has actually been entered — a process that never touched the
/// terminal keeps the hook it had.
fn restore_on_panic() {
    static INSTALLED: Once = Once::new();
    INSTALLED.call_once(|| {
        let previous = std::panic::take_hook();
        std::panic::set_hook(Box::new(move |info| {
            restore();
            previous(info);
        }));
    });
}

/// The process's controlling terminal, as a character stream.
///
/// One per process, in practice: two of these would fight over `stty` and over
/// stdin. `open` does not enforce that, because a test constructing a second
/// one in cooked mode does no harm.
#[derive(Debug)]
pub struct Terminal {
    /// Bytes from the reader thread, and whatever a short read left over.
    input: Mutex<Input>,
    /// Whether this terminal entered raw mode. What undoes it is in [`COOKED`].
    raw: bool,
    /// Set when the user pressed Ctrl-C in raw mode.
    interrupted: AtomicBool,
    /// Whether the last byte written was a carriage return, so that a line
    /// feed arriving in the *next* write is still the second half of one line
    /// break. See [`expand_line_breaks`].
    after_cr: AtomicBool,
}

/// The receiving end of the reader thread, plus the bytes not yet handed out.
#[derive(Debug)]
struct Input {
    rx: Receiver<Vec<u8>>,
    pending: VecDeque<u8>,
    /// Set once the reader thread has gone — stdin reached end of file.
    closed: bool,
}

impl Terminal {
    /// Take over the terminal, entering raw mode if this host allows it.
    ///
    /// Never fails: a host with no `stty`, or a stdin that is not a terminal,
    /// gets a cooked [`Terminal`] rather than an error. Check
    /// [`is_raw`](Terminal::is_raw) to find out which.
    #[must_use]
    pub fn open() -> Terminal {
        let raw = enter_raw_mode();
        let (tx, rx) = channel();
        // Detached on purpose: it is blocked in `read` for the life of the
        // process and there is no portable way to interrupt that. Process exit
        // is what ends it.
        std::thread::Builder::new()
            .name(String::from("rsemu-stdin"))
            .spawn(move || {
                let mut stdin = std::io::stdin();
                let mut buf = [0u8; 256];
                loop {
                    match stdin.read(&mut buf) {
                        Ok(0) | Err(_) => break,
                        Ok(n) => {
                            if tx.send(buf[..n].to_vec()).is_err() {
                                break;
                            }
                        }
                    }
                }
            })
            .ok();
        Terminal {
            input: Mutex::new(Input {
                rx,
                pending: VecDeque::new(),
                closed: false,
            }),
            raw,
            interrupted: AtomicBool::new(false),
            after_cr: AtomicBool::new(false),
        }
    }

    /// Whether the terminal is in raw mode.
    ///
    /// `false` means the host is line-buffering and echoing: the guest sees a
    /// whole line at a time and each line appears twice. Worth telling the user.
    #[must_use]
    pub fn is_raw(&self) -> bool {
        self.raw
    }

    /// Whether the user has pressed Ctrl-C.
    ///
    /// Only ever true in raw mode; in cooked mode the kernel still turns Ctrl-C
    /// into a signal and this process never sees the byte.
    #[must_use]
    pub fn interrupted(&self) -> bool {
        self.interrupted.load(Ordering::Relaxed)
    }

    /// Whether stdin has reached end of file.
    ///
    /// A piped script that has run out is done; an interactive session never
    /// reports this.
    #[must_use]
    pub fn at_eof(&self) -> bool {
        let mut input = self.input.lock().expect("terminal input lock");
        self.refill(&mut input);
        input.closed && input.pending.is_empty()
    }

    /// Move bytes both ways between this terminal and `port`.
    ///
    /// The whole of the host's per-frame work: whatever the user typed goes to
    /// the guest, whatever the guest printed goes to the screen. Call it
    /// between run slices — never from inside a device.
    ///
    /// Returns how many bytes moved, in both directions together. A run loop
    /// with a script on its stdin uses that plus [`at_eof`](Terminal::at_eof)
    /// to know when there is nothing left to do.
    ///
    /// **A recorded session does not use this one.** Feeding the port here
    /// decides *when* a keystroke arrives, and that instant would be a host
    /// scheduling artefact rather than a point on the guest's timeline — the
    /// thing [`core::record`](crate::core::record) exists to stop. A caller
    /// with a recorder attached posts through
    /// [`typed`](Terminal::typed) instead and lets the machine deliver at a
    /// round boundary; [`printed`](Terminal::printed) is the other half, and is
    /// the same either way because output is not input.
    pub fn pump(&self, port: &CharPort) -> usize {
        self.typed(|bytes| port.feed(bytes)) + self.printed(port)
    }

    /// Hand whatever the user has typed to `take`, a chunk at a time.
    ///
    /// The input half of [`pump`](Terminal::pump), split out so a recorded
    /// session can post the bytes to a
    /// [`Recorder`](crate::core::record::Recorder) rather than push them into a
    /// port. `take` reports how many of the chunk it accepted, which is what
    /// the return value adds up — a run loop uses it to notice that a piped
    /// script has gone quiet.
    pub fn typed(&self, mut take: impl FnMut(&[u8]) -> usize) -> usize {
        let mut moved = 0;
        let mut buf = [0u8; 256];
        loop {
            let n = self.read(&mut buf);
            if n == 0 {
                break;
            }
            moved += take(&buf[..n]);
        }
        moved
    }

    /// Put whatever the guest printed on the screen, returning how many bytes
    /// went.
    ///
    /// The output half of [`pump`](Terminal::pump). Nothing about this half
    /// changes under a recording: what the guest wrote is a function of the
    /// machine, not an input to it.
    pub fn printed(&self, port: &CharPort) -> usize {
        let out = port.drain();
        if out.is_empty() {
            return 0;
        }
        let moved = self.write(&out);
        self.flush();
        moved
    }

    /// Drain the reader thread's channel into `pending`.
    fn refill(&self, input: &mut Input) {
        loop {
            match input.rx.try_recv() {
                Ok(chunk) => input.pending.extend(chunk),
                Err(TryRecvError::Empty) => break,
                Err(TryRecvError::Disconnected) => {
                    input.closed = true;
                    break;
                }
            }
        }
    }
}

impl CharDevice for Terminal {
    fn read(&self, dst: &mut [u8]) -> usize {
        let mut input = self.input.lock().expect("terminal input lock");
        self.refill(&mut input);
        let mut taken = 0;
        while taken < dst.len() {
            let Some(byte) = input.pending.pop_front() else {
                break;
            };
            if byte == INTERRUPT && self.raw {
                self.interrupted.store(true, Ordering::Relaxed);
                continue;
            }
            dst[taken] = byte;
            taken += 1;
        }
        taken
    }

    fn write(&self, src: &[u8]) -> usize {
        // A guest that ends its lines with a bare carriage return — which every
        // 1970s console does — would otherwise overwrite one line forever: raw
        // mode turns the kernel's own output processing off, and even a cooked
        // terminal only expands a *newline*, never a return. The expansion is a
        // property of this backend rather than of the stream, which carries
        // bytes and says nothing about what they mean.
        let mut buf = Vec::with_capacity(src.len() + 8);
        let mut after_cr = self.after_cr.load(Ordering::Relaxed);
        expand_line_breaks(src, &mut after_cr, &mut buf);
        self.after_cr.store(after_cr, Ordering::Relaxed);
        let ok = std::io::stdout().lock().write_all(&buf);
        if ok.is_err() { 0 } else { src.len() }
    }

    fn flush(&self) {
        let _ = std::io::stdout().flush();
    }
}

impl Drop for Terminal {
    fn drop(&mut self) {
        if self.raw {
            restore();
        }
    }
}

/// Turn a guest's line endings into the `CR LF` a raw terminal needs.
///
/// A bare carriage return and a bare line feed each become one line break, and
/// a `CR LF` pair becomes one rather than two. `after_cr` carries "the last
/// byte was a carriage return" **across calls**, because the pair does not
/// have to arrive in one: a guest printing through a UART emits a character at
/// a time, the run loop drains the port once a slice, and a slice boundary
/// falls between the two halves as readily as anywhere else. Looking ahead
/// only within one buffer turned every pair split that way into a blank line,
/// at no pattern a user could name.
fn expand_line_breaks(src: &[u8], after_cr: &mut bool, out: &mut Vec<u8>) {
    for &byte in src {
        match byte {
            b'\r' => {
                out.extend_from_slice(b"\r\n");
                *after_cr = true;
            }
            // The second half of a pair whose first half already broke the
            // line, whichever write that was in.
            b'\n' if *after_cr => *after_cr = false,
            b'\n' => out.extend_from_slice(b"\r\n"),
            byte => {
                out.push(byte);
                *after_cr = false;
            }
        }
    }
}

/// Put the controlling terminal into raw mode, leaving what undoes it in
/// [`COOKED`].
///
/// `false` when there is nothing to restore, which covers every failure: no
/// `stty`, stdin is not a terminal, or the settings string came back empty.
fn enter_raw_mode() -> bool {
    let Some(saved) = stty(&["-g"]) else {
        return false;
    };
    let saved = saved.trim().to_string();
    if saved.is_empty() {
        return false;
    }
    // The way back is recorded, and the panic hook armed, *before* the
    // terminal changes: there is no instant at which it is raw and nothing
    // knows how to undo that.
    // An earlier entry's settings stand: see `COOKED`.
    let _ = COOKED.set(saved);
    restore_on_panic();
    RAW.store(true, Ordering::SeqCst);
    // `raw` alone leaves output post-processing on some hosts and off on
    // others; `-echo` is the half that matters, since the guest echoes.
    if stty(&["raw", "-echo"]).is_none() {
        // Nothing changed, so there is nothing to put back.
        RAW.store(false, Ordering::SeqCst);
        return false;
    }
    true
}

/// Run `stty` against this process's terminal, returning its stdout.
///
/// `stdin` is inherited so `stty` sees the terminal rather than a pipe; that is
/// also what makes this return `None` when stdin is redirected, which is the
/// answer we want.
fn stty(args: &[&str]) -> Option<String> {
    let output = Command::new("stty")
        .args(args)
        .stdin(Stdio::inherit())
        .stdout(Stdio::piped())
        .stderr(Stdio::null())
        .output()
        .ok()?;
    output
        .status
        .success()
        .then(|| String::from_utf8_lossy(&output.stdout).into_owned())
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Under `cargo test` stdin is not a terminal, so this is the cooked path —
    /// which is the one that has to keep working on a build machine.
    #[test]
    fn a_terminal_opens_without_a_tty_and_reports_that_it_is_cooked() {
        let term = Terminal::open();
        assert!(!term.is_raw(), "cargo test has no controlling terminal");
        assert!(!term.interrupted());
        // Reading must not block, whatever stdin is.
        let mut buf = [0u8; 4];
        let _ = term.read(&mut buf);
    }

    /// Every way a guest ends a line, as the terminal is sent it.
    fn expanded(writes: &[&[u8]]) -> Vec<u8> {
        let mut out = Vec::new();
        let mut after_cr = false;
        for write in writes {
            expand_line_breaks(write, &mut after_cr, &mut out);
        }
        out
    }

    #[test]
    fn each_kind_of_line_ending_is_one_line_break() {
        assert_eq!(expanded(&[b"a\rb"]), b"a\r\nb");
        assert_eq!(expanded(&[b"a\nb"]), b"a\r\nb");
        assert_eq!(expanded(&[b"a\r\nb"]), b"a\r\nb");
        // Two of them are two.
        assert_eq!(expanded(&[b"a\r\rb"]), b"a\r\n\r\nb");
        assert_eq!(expanded(&[b"a\n\nb"]), b"a\r\n\r\nb");
        assert_eq!(expanded(&[b"a\r\n\r\nb"]), b"a\r\n\r\nb");
    }

    /// The pair does not have to arrive in one write: the run loop drains the
    /// port once a slice, and a slow UART's `CR` and `LF` land either side of
    /// a slice boundary as often as not.
    #[test]
    fn a_cr_lf_pair_split_across_two_writes_is_still_one_line_break() {
        assert_eq!(expanded(&[b"a\r", b"\nb"]), b"a\r\nb");
        assert_eq!(expanded(&[b"a\r", b"", b"\nb"]), b"a\r\nb");
        // Only the line feed *immediately* after: anything between ends it.
        assert_eq!(expanded(&[b"a\r", b"b\n"]), b"a\r\nb\r\n");
        assert_eq!(expanded(&[b"a\r\n", b"\nb"]), b"a\r\n\r\nb");
    }

    #[test]
    fn pumping_moves_guest_output_toward_the_host() {
        let term = Terminal::open();
        let port = CharPort::new();
        port.write(b"");
        assert_eq!(port.pending_output(), 0);
        // Nothing to say and nothing to hear: a pump on an idle port is a
        // no-op, which is what it does thousands of times a second.
        term.pump(&port);
        assert_eq!(port.pending_output(), 0);
        assert_eq!(port.pending_input(), 0);
    }
}
