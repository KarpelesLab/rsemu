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
//! Raw mode is restored by [`Terminal`]'s `Drop`. A hard kill (`SIGKILL`)
//! skips that, as it would with any program; `stty sane` fixes it.
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
use std::sync::Mutex;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::mpsc::{Receiver, TryRecvError, channel};

use super::chardev::{CharDevice, CharPort};

/// The byte a user presses to get out: Ctrl-C.
///
/// In raw mode the kernel no longer turns this into a signal, so a guest that
/// never exits would trap the terminal forever. It is consumed here rather than
/// forwarded, and shows up as [`Terminal::interrupted`].
const INTERRUPT: u8 = 0x03;

/// The process's controlling terminal, as a character stream.
///
/// One per process, in practice: two of these would fight over `stty` and over
/// stdin. `open` does not enforce that, because a test constructing a second
/// one in cooked mode does no harm.
#[derive(Debug)]
pub struct Terminal {
    /// Bytes from the reader thread, and whatever a short read left over.
    input: Mutex<Input>,
    /// The `stty -g` string to restore on the way out, if raw mode was entered.
    saved: Option<String>,
    /// Set when the user pressed Ctrl-C in raw mode.
    interrupted: AtomicBool,
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
        let saved = enter_raw_mode();
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
            saved,
            interrupted: AtomicBool::new(false),
        }
    }

    /// Whether the terminal is in raw mode.
    ///
    /// `false` means the host is line-buffering and echoing: the guest sees a
    /// whole line at a time and each line appears twice. Worth telling the user.
    #[must_use]
    pub fn is_raw(&self) -> bool {
        self.saved.is_some()
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
    pub fn pump(&self, port: &CharPort) -> usize {
        let mut moved = 0;
        let mut buf = [0u8; 256];
        loop {
            let n = self.read(&mut buf);
            if n == 0 {
                break;
            }
            moved += port.feed(&buf[..n]);
        }
        let out = port.drain();
        if !out.is_empty() {
            moved += self.write(&out);
            self.flush();
        }
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
            if byte == INTERRUPT && self.saved.is_some() {
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
        let mut i = 0;
        while i < src.len() {
            match src[i] {
                b'\r' => {
                    buf.extend_from_slice(b"\r\n");
                    // A guest that already sends CR LF gets one line break, not
                    // two.
                    if src.get(i + 1) == Some(&b'\n') {
                        i += 1;
                    }
                }
                b'\n' => buf.extend_from_slice(b"\r\n"),
                byte => buf.push(byte),
            }
            i += 1;
        }
        let ok = std::io::stdout().lock().write_all(&buf);
        if ok.is_err() { 0 } else { src.len() }
    }

    fn flush(&self) {
        let _ = std::io::stdout().flush();
    }
}

impl Drop for Terminal {
    fn drop(&mut self) {
        if let Some(saved) = self.saved.take() {
            let _ = stty(&[&saved]);
        }
    }
}

/// Put the controlling terminal into raw mode, returning what to restore.
///
/// `None` when there is nothing to restore, which covers every failure: no
/// `stty`, stdin is not a terminal, or the settings string came back empty.
fn enter_raw_mode() -> Option<String> {
    let saved = stty(&["-g"])?;
    let saved = saved.trim().to_string();
    if saved.is_empty() {
        return None;
    }
    // `raw` alone leaves output post-processing on some hosts and off on
    // others; `-echo` is the half that matters, since the guest echoes.
    stty(&["raw", "-echo"])?;
    Some(saved)
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
