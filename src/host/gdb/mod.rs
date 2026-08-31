//! The GDB remote serial protocol, over TCP (`ROADMAP.md` §8).
//!
//! ```console
//! $ rsemu debug apple1 --gdb :1234
//! $ gdb -ex 'target remote :1234'
//! ```
//!
//! This is the highest-leverage tool for every later phase: building x86
//! protected mode or bringing up a kernel without a debugger is a self-inflicted
//! wound. It is also the code path that violates `ROADMAP.md` §15's invariant 5
//! first if nobody is careful, so that invariant has a section of its own below.
//!
//! # Layout
//!
//! | Module | Covers |
//! | --- | --- |
//! | [`packet`] | framing, checksums, `+`/`-`, escapes, run-length encoding |
//! | [`arch`] | per-CPU register maps and the `qXfer:features:read` XML |
//! | [`target`] | the [`DebugTarget`] seam, and a [`Machine`] behind it |
//! | [`stub`] | the protocol: one packet in, one reply out |
//! | this module | the listener, the session loop, and [`serve`] |
//!
//! The split is deliberate: [`stub::Stub`] owns no socket and no machine, so the
//! whole protocol is tested against a fake target with neither, and
//! [`target::MachineTarget`] is tested against a real machine with no socket.
//! Only this module needs both.
//!
//! # Invariant 5: every debugger access is a debug access
//!
//! Reading a device register from GDB must not acknowledge an interrupt, pop a
//! FIFO or advance a pointer. There is exactly one constructor for access
//! attributes in this subsystem — [`target::debug_attrs`] — and it starts from
//! [`MemAttrs::DEBUG`](crate::core::space::MemAttrs::DEBUG). Nothing else here
//! builds a `MemAttrs`, so honouring the invariant is a property of the code's
//! shape rather than of everyone remembering.
//!
//! It matters more here than anywhere else because watchpoints are *polled*:
//! the watched bytes are re-read after every clock tick, so one non-debug read
//! would be a side effect a million times a second.
//!
//! # Stopping the world
//!
//! A debugger must not race the scheduler (`ROADMAP.md` §4.7). It does not have
//! to here, because the session loop and the machine share one thread: virtual
//! time advances only inside [`DebugTarget::resume`] and
//! [`DebugTarget::step`], both called from [`GdbServer::poll`] and never while a
//! packet is being answered. `Machine::run_until` returns at a quantum boundary
//! with every runnable unwound back to the scheduler — the safe point of §4.7 in
//! the `Deterministic` threading mode, which is the only mode `Machine` drives.
//! When the parallel mode lands, the same call site is where its stop-the-world
//! barrier goes; nothing above it changes.
//!
//! # What is not here
//!
//! * **Read and access watchpoints.** `Z3` and `Z4` need to observe a guest
//!   *read*, which needs a hook on the access path; `core::space` has none, and
//!   a debug read cannot see one. They are refused, not faked. `Z2` — stop when
//!   the watched bytes change — is polled and does work.
//! * **A register view on `Device`.** There is no route from a `dyn Device` to a
//!   concrete CPU, so registers are read and written through the device's
//!   snapshot chunk with a per-class byte map. See [`arch`].
//! * **A gdbarch for every core.** A target description gives a client the
//!   register file; upstream GDB additionally insists on knowing the machine,
//!   and has no 6502. `rsemu debug` says so at startup rather than letting the
//!   user find out from GDB's error. [`arch`] has the long form.
//!
//! # Sources
//!
//! The GDB manual's "Remote Protocol" and "Target Descriptions" appendices;
//! `docs/system/debug-protocols.md` for why implementing a published wire
//! protocol from its specification is not a provenance problem.

pub mod arch;
pub mod packet;
pub mod stub;
pub mod target;

use std::io::{ErrorKind, Read, Write};
use std::net::{SocketAddr, TcpListener, TcpStream, ToSocketAddrs};
use std::time::Duration;

use crate::machine::Machine;

pub use stub::Outcome;
pub use target::{DebugTarget, MachineTarget, Stop, StopKind, TargetError, TargetResult};

/// How long an idle poll sleeps rather than spinning on a socket that has
/// nothing to say.
///
/// One millisecond: below a person's reaction time, and it keeps a halted
/// session off the CPU entirely.
const IDLE_SLEEP: Duration = Duration::from_millis(1);

/// Where a session has got to.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Progress {
    /// No client has ever connected. The machine is held stopped, so that
    /// attaching lands on the reset vector rather than wherever the guest had
    /// got to.
    Waiting,
    /// A client is connected and has the machine stopped.
    Halted,
    /// A client is connected and has asked the machine to run.
    Running,
    /// A client attached and detached. The machine is free to run.
    Detached,
    /// The client sent `k`: shut the machine down.
    Kill,
}

/// One accepted connection.
#[derive(Debug)]
struct Conn {
    stream: TcpStream,
    peer: Option<SocketAddr>,
    framer: packet::Framer,
    stub: stub::Stub,
    /// Bytes written but not yet accepted by the socket.
    pending: Vec<u8>,
}

/// A GDB remote-protocol server listening on a TCP port.
///
/// Single connection at a time, on purpose: two debuggers driving one machine's
/// execution state would fight, and the second one's `continue` would be the
/// first one's mystery.
#[derive(Debug)]
pub struct GdbServer {
    listener: TcpListener,
    conn: Option<Conn>,
    /// Whether the machine is held stopped until the first client attaches.
    wait_for_attach: bool,
    /// Set once a client has attached and gone.
    detached: bool,
}

impl GdbServer {
    /// Bind a listener.
    ///
    /// `addr` may be `1234`, `:1234`, `host:1234` or `[::1]:1234`. **A bare
    /// port or a leading colon binds the loopback interface only**: the far end
    /// of this socket can read and write every byte of guest memory and change
    /// the program counter, so exposing it to the network is a decision someone
    /// has to make explicitly by naming an address (`0.0.0.0:1234`).
    ///
    /// # Errors
    ///
    /// An address that does not parse or resolve, or a port that cannot be
    /// bound.
    pub fn bind(addr: &str) -> std::io::Result<GdbServer> {
        let resolved = resolve(addr)?;
        let listener = TcpListener::bind(&resolved[..])?;
        listener.set_nonblocking(true)?;
        Ok(GdbServer {
            listener,
            conn: None,
            wait_for_attach: true,
            detached: false,
        })
    }

    /// Let the machine run before anyone attaches.
    ///
    /// The default is to hold it stopped, which is what `rsemu debug` wants:
    /// attaching then lands on the reset vector rather than wherever a
    /// free-running guest happened to be.
    #[must_use]
    pub fn without_waiting(mut self) -> GdbServer {
        self.wait_for_attach = false;
        self
    }

    /// The address actually bound, which is how a test finds the ephemeral port
    /// it asked for with `:0`.
    ///
    /// # Errors
    ///
    /// Whatever the operating system says about the socket.
    pub fn local_addr(&self) -> std::io::Result<SocketAddr> {
        self.listener.local_addr()
    }

    /// Whether a client is connected.
    #[must_use]
    pub fn is_attached(&self) -> bool {
        self.conn.is_some()
    }

    /// Service the debugger, and let the machine run if it has been told to.
    ///
    /// One call is one turn of the session loop: accept a connection if one is
    /// waiting, read and answer whatever packets have arrived, and advance the
    /// machine by one slice if the client asked for that. It never blocks for
    /// longer than a millisecond, so a caller can pump a console between calls.
    ///
    /// # Errors
    ///
    /// A socket error other than "would block" and other than a peer that hung
    /// up — that one closes the connection and is reported as
    /// [`Progress::Detached`].
    pub fn poll(&mut self, target: &mut dyn DebugTarget) -> std::io::Result<Progress> {
        self.accept()?;
        let Some(conn) = self.conn.as_mut() else {
            std::thread::sleep(IDLE_SLEEP);
            return Ok(if self.wait_for_attach && !self.detached {
                Progress::Waiting
            } else {
                Progress::Detached
            });
        };

        let mut out = Vec::new();
        let mut outcome = Outcome::Continue;
        let mut closed = false;

        let mut buf = [0u8; 1024];
        match conn.stream.read(&mut buf) {
            Ok(0) => closed = true,
            Ok(n) => {
                for byte in buf.get(..n).unwrap_or(&[]) {
                    if let Some(event) = conn.framer.push(*byte) {
                        match conn.stub.on_event(event, target, &mut out) {
                            Outcome::Continue => {}
                            other => outcome = other,
                        }
                    }
                }
            }
            Err(e) if e.kind() == ErrorKind::WouldBlock => {}
            Err(e) if e.kind() == ErrorKind::Interrupted => {}
            Err(_) => closed = true,
        }

        if !closed && outcome == Outcome::Continue {
            conn.stub.drive(target, &mut out);
        }

        conn.pending.extend_from_slice(&out);
        if !closed && !Self::flush(conn) {
            closed = true;
        }

        let running = conn.stub.is_running();
        match outcome {
            Outcome::Kill => {
                self.conn = None;
                return Ok(Progress::Kill);
            }
            Outcome::Detach => {
                // Flush the `OK` before hanging up, or GDB reports the detach
                // as a protocol error.
                let _ = conn.stream.flush();
                closed = true;
            }
            Outcome::Continue => {}
        }

        if closed {
            self.conn = None;
            self.detached = true;
            return Ok(Progress::Detached);
        }
        if running {
            Ok(Progress::Running)
        } else {
            // Nothing to do but wait for the next packet.
            std::thread::sleep(IDLE_SLEEP);
            Ok(Progress::Halted)
        }
    }

    /// Take a waiting connection, if there is one.
    fn accept(&mut self) -> std::io::Result<()> {
        if self.conn.is_some() {
            return Ok(());
        }
        match self.listener.accept() {
            Ok((stream, peer)) => {
                stream.set_nonblocking(true)?;
                // Debug traffic is small and latency-sensitive: a stop reply
                // held back by Nagle's algorithm is a debugger that feels
                // broken.
                let _ = stream.set_nodelay(true);
                self.conn = Some(Conn {
                    stream,
                    peer: Some(peer),
                    framer: packet::Framer::new(),
                    stub: stub::Stub::new(),
                    pending: Vec::new(),
                });
                Ok(())
            }
            Err(e) if e.kind() == ErrorKind::WouldBlock => Ok(()),
            Err(e) if e.kind() == ErrorKind::Interrupted => Ok(()),
            Err(e) => Err(e),
        }
    }

    /// Push as much of the pending output as the socket will take.
    ///
    /// Returns false when the peer has gone. A short write is normal on a
    /// non-blocking socket and simply leaves the rest queued for the next poll.
    fn flush(conn: &mut Conn) -> bool {
        while !conn.pending.is_empty() {
            match conn.stream.write(&conn.pending) {
                Ok(0) => return false,
                Ok(n) => {
                    conn.pending.drain(..n);
                }
                Err(e) if e.kind() == ErrorKind::WouldBlock => return true,
                Err(e) if e.kind() == ErrorKind::Interrupted => {}
                Err(_) => return false,
            }
        }
        let _ = conn.stream.flush();
        true
    }

    /// The connected client's address, for a status line.
    #[must_use]
    pub fn peer(&self) -> Option<SocketAddr> {
        self.conn.as_ref().and_then(|c| c.peer)
    }
}

/// Turn `1234`, `:1234` or `host:1234` into addresses to bind.
fn resolve(addr: &str) -> std::io::Result<Vec<SocketAddr>> {
    let spec = if addr.starts_with(':') {
        format!("127.0.0.1{addr}")
    } else if addr.chars().all(|c| c.is_ascii_digit()) && !addr.is_empty() {
        format!("127.0.0.1:{addr}")
    } else {
        addr.to_string()
    };
    let list: Vec<SocketAddr> = spec.to_socket_addrs()?.collect();
    if list.is_empty() {
        return Err(std::io::Error::new(
            ErrorKind::InvalidInput,
            format!("`{addr}` resolved to no address"),
        ));
    }
    Ok(list)
}

/// Why [`serve`] returned.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ExitReason {
    /// The client sent `k`.
    Killed,
    /// The caller's stop condition fired — a deadline, or a console interrupt.
    Stopped,
}

/// Run `machine` under a debugger until it is killed or `keep_going` says stop.
///
/// The whole session loop in one call, so a front end is three lines. Between
/// turns it calls `keep_going`, which is where a console is pumped and a
/// Ctrl-C on the emulator's own terminal is noticed; returning `false` from it
/// ends the session.
///
/// # Errors
///
/// A socket error the session cannot continue past.
pub fn serve(
    machine: &mut Machine,
    server: &mut GdbServer,
    mut keep_going: impl FnMut(&mut Machine) -> bool,
) -> std::io::Result<ExitReason> {
    let mut target = MachineTarget::new(machine);
    loop {
        let progress = server.poll(&mut target)?;
        if progress == Progress::Kill {
            return Ok(ExitReason::Killed);
        }
        if progress == Progress::Detached {
            // Nobody is watching: let it run rather than freezing a guest whose
            // debugger has gone home.
            if let Err(e) = target.resume() {
                return Err(std::io::Error::other(e.to_string()));
            }
        }
        if !keep_going(target.machine_mut()) {
            return Ok(ExitReason::Stopped);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_bare_port_binds_the_loopback_only() {
        let addrs = resolve("1234").expect("a bare port");
        assert!(addrs.iter().all(|a| a.ip().is_loopback()), "{addrs:?}");
        let addrs = resolve(":1234").expect("a leading colon");
        assert!(addrs.iter().all(|a| a.ip().is_loopback()), "{addrs:?}");
        // An explicit address is honoured as written, which is the only way to
        // expose the port.
        let addrs = resolve("0.0.0.0:1234").expect("an explicit address");
        assert!(addrs.iter().any(|a| a.ip().is_unspecified()), "{addrs:?}");
    }

    #[test]
    fn a_nonsense_address_is_an_error_not_a_panic() {
        assert!(resolve("").is_err());
        assert!(resolve("not a host name at all:1").is_err());
    }

    #[test]
    fn an_ephemeral_port_reports_where_it_landed() {
        let server = GdbServer::bind(":0").expect("bind");
        let addr = server.local_addr().expect("local_addr");
        assert!(addr.port() != 0);
        assert!(addr.ip().is_loopback());
        assert!(!server.is_attached());
    }
}
