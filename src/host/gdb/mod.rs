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
//! | [`target`] | the [`DebugTarget`] seam, the memory map, and a [`Machine`] behind it |
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
//! packet is being answered.
//!
//! **That holds under `parallel` too**, which is worth stating because it is
//! the mode where two cores really are on host threads of their own. A round
//! joins every job it submitted before it returns, and that join *is* §4.7's
//! rendezvous — so `Machine::run_until` and `Machine::step_until` both come back
//! with every runnable unwound to the scheduler whatever the threading mode is,
//! and there is no instant at which a packet is answered while a guest core is
//! mid-instruction. The stub therefore never has to raise a stop-the-world
//! barrier of its own; if `Machine` ever grew a way to leave a round in flight
//! across a return, [`Machine::stop_the_world`](crate::machine::Machine::stop_the_world)
//! is what would go around each of those two calls. `tests/gdb_multicpu.rs`
//! debugs a two-core machine in that mode rather than leaving this a claim.
//!
//! What `parallel` does cost the debugger is *speed*, not correctness: a
//! breakpoint-checking run advances one clock tick at a time, and under
//! `parallel` each of those ticks is a job dispatch per core.
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
//! * **`load` into a ROM.** `qXfer:memory-map:read` claims `flash` for a range
//!   whose device published an erase geometry — see [`MemKind`] — and the three
//!   `vFlash` packets program it. A `rom` object publishes none, because there
//!   is nothing behind it that could take the write: `core::space` refuses a
//!   write to a read-only mapping *including a debug one*, deliberately and in
//!   as many words. So `load` into a board's ROM still fails, and fails with
//!   GDB naming the read-only region rather than with a bus error from
//!   nowhere.
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
use std::net::{SocketAddr, TcpListener, TcpStream};
use std::time::Duration;

use crate::machine::Machine;

pub use stub::Outcome;
pub use target::{
    DebugTarget, MachineTarget, MemKind, MemRegion, Stop, StopKind, TargetError, TargetResult,
    memory_map_xml,
};

/// How long an idle poll waits rather than spinning on a socket that has
/// nothing to say.
///
/// One millisecond: below a person's reaction time, and it keeps a halted
/// session off the CPU entirely. It is the longest a poll holds its caller,
/// not how long a packet waits — see [`wait_for_packet`].
const IDLE_SLEEP: Duration = Duration::from_millis(1);

/// How many bytes one poll takes from the socket.
///
/// Twice the `PacketSize` the stub advertises, so a whole packet — a 4 KiB `X`
/// of a `load`, escapes and all — arrives in one turn rather than being
/// reassembled across four.
const READ_CHUNK: usize = 8192;

/// How many bytes of reply may be waiting for a client before the server stops
/// taking requests from it.
///
/// A megabyte: several hundred times the largest reply the stub produces, so a
/// debugger that is merely slow never notices. One that has stopped reading is
/// another matter. Every request earns a reply, and a `-` earns a *copy of the
/// last one* for a single byte, so a peer that writes and never reads could
/// otherwise grow this process's heap at ninety times the rate it sent — and
/// the far end of this socket is whoever reached the port.
const MAX_PENDING: usize = 1024 * 1024;

/// Wait up to [`IDLE_SLEEP`] for the client to say something.
///
/// What a halted session does between packets, in place of an unconditional
/// sleep. A sleep made every request wait out the rest of its millisecond
/// whether or not the bytes were already there: GDB's protocol is strictly
/// request and reply, a single `stepi` with a register window open is dozens
/// of round trips, and each of them cost a full tick.
///
/// A blocking `peek` with a receive timeout returns the moment a byte arrives
/// and consumes nothing, so the next poll reads it normally. The socket is put
/// back to non-blocking before returning; if the mode cannot be changed at all
/// this degrades to the sleep it replaces.
fn wait_for_packet(stream: &TcpStream) {
    if stream.set_nonblocking(false).is_err() {
        std::thread::sleep(IDLE_SLEEP);
        return;
    }
    if stream.set_read_timeout(Some(IDLE_SLEEP)).is_ok() {
        // The result is deliberately unread: data, a timeout, a hang-up and an
        // error are all "go round again", and the poll's own read is what
        // tells them apart.
        let _ = stream.peek(&mut [0u8; 1]);
    } else {
        std::thread::sleep(IDLE_SLEEP);
    }
    let _ = stream.set_nonblocking(true);
}

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
    /// Bytes read from the socket and not yet acted on, because acting on them
    /// would have taken `pending` past [`MAX_PENDING`]. Empty except while a
    /// client is not reading its replies.
    unread: Vec<u8>,
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

        // Nothing new is read while requests already read are still waiting
        // for room: what the client sends next stays in the kernel's buffer,
        // where its own flow control pays for it.
        if conn.unread.is_empty() && conn.pending.len() < MAX_PENDING {
            let mut buf = [0u8; READ_CHUNK];
            match conn.stream.read(&mut buf) {
                Ok(0) => closed = true,
                Ok(n) => conn.unread.extend_from_slice(buf.get(..n).unwrap_or(&[])),
                // `TimedOut` is how Windows spells a receive timeout, and one
                // is armed on this socket by `wait_for_packet`.
                Err(e)
                    if matches!(
                        e.kind(),
                        ErrorKind::WouldBlock | ErrorKind::Interrupted | ErrorKind::TimedOut
                    ) => {}
                Err(_) => closed = true,
            }
        }

        // Act on what has been read, a byte at a time, for as long as there is
        // room for the replies. The check is per byte rather than per read
        // because one byte can be a whole request: a `-` asks for the last
        // packet again.
        let unread = std::mem::take(&mut conn.unread);
        let mut taken = 0;
        for byte in &unread {
            if conn.pending.len().saturating_add(out.len()) >= MAX_PENDING {
                break;
            }
            taken += 1;
            if let Some(event) = conn.framer.push(*byte) {
                match conn.stub.on_event(event, target, &mut out) {
                    Outcome::Continue => {}
                    other => outcome = other,
                }
            }
        }
        conn.unread = unread;
        conn.unread.drain(..taken);
        // A client that is not reading is not waited *on* either: its next
        // bytes are already in the socket, so a wait for them would return at
        // once and the session would spin.
        let throttled = !conn.unread.is_empty() || conn.pending.len() >= MAX_PENDING;

        if !closed && outcome == Outcome::Continue {
            // Whatever the packets just decided is what the machine is about
            // to do, and `drive` is the only thing here that advances virtual
            // time — so the machine is told *before* it moves rather than
            // after. A device that freezes while the core is halted (an
            // STM32's `DBGMCU`, through `Device::debug_halt`) needs the level
            // in place for the slice, not a turn late.
            //
            // A single-step is deliberately not un-halting: `s` is serviced
            // inside `on_event` above, with the level still where the last
            // stop left it. That is what the hardware does too — stepping is
            // halting repeatedly, and a watchdog frozen at a breakpoint stays
            // frozen while you walk through the loop.
            target.set_debug_halted(!conn.stub.is_running());
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
            // Nothing is holding the core any more, whether the client said
            // `D` or simply went away. A machine that kept a frozen watchdog
            // after its debugger hung up would be a machine that never reset.
            target.set_debug_halted(false);
            return Ok(Progress::Detached);
        }
        if running {
            Ok(Progress::Running)
        } else {
            // Nothing to do but wait for the next packet — and to wake when it
            // comes, rather than a tick later.
            if throttled {
                std::thread::sleep(IDLE_SLEEP);
            } else {
                wait_for_packet(&conn.stream);
            }
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
                    unread: Vec::new(),
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
///
/// The rule — a bare port or a leading colon binds the loopback interface only
/// — and the reason for it live in [`crate::host::listen`], so that the two
/// frontends which hand a stranger something dangerous cannot drift apart about
/// what an address means.
fn resolve(addr: &str) -> std::io::Result<Vec<SocketAddr>> {
    crate::host::listen::resolve(addr)
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

    /// A target nothing is ever asked of: the two tests below are about the
    /// socket, and never get as far as a packet that needs a machine.
    #[derive(Debug)]
    struct Inert;

    impl DebugTarget for Inert {
        fn cpu_count(&self) -> usize {
            0
        }
        fn cpu_path(&self, _cpu: usize) -> TargetResult<&str> {
            Err(TargetError::NoSuchCpu)
        }
        fn arch(&self, _cpu: usize) -> TargetResult<&'static arch::Arch> {
            Err(TargetError::NoSuchCpu)
        }
        fn read_registers(&self, _cpu: usize) -> TargetResult<Vec<u8>> {
            Err(TargetError::NoSuchCpu)
        }
        fn write_registers(&mut self, _cpu: usize, _data: &[u8]) -> TargetResult<()> {
            Err(TargetError::NoSuchCpu)
        }
        fn read_register(&self, _cpu: usize, _index: usize) -> TargetResult<Vec<u8>> {
            Err(TargetError::NoSuchCpu)
        }
        fn write_register(&mut self, _cpu: usize, _index: usize, _data: &[u8]) -> TargetResult<()> {
            Err(TargetError::NoSuchCpu)
        }
        fn read_memory(&self, _cpu: usize, _addr: u64, _dst: &mut [u8]) -> TargetResult<()> {
            Err(TargetError::Fault)
        }
        fn write_memory(&mut self, _cpu: usize, _addr: u64, _src: &[u8]) -> TargetResult<()> {
            Err(TargetError::Fault)
        }
        fn add_breakpoint(&mut self, _addr: u64, _hardware: bool) -> TargetResult<()> {
            Ok(())
        }
        fn remove_breakpoint(&mut self, _addr: u64, _hardware: bool) -> TargetResult<()> {
            Ok(())
        }
        fn step(&mut self, _cpu: usize) -> TargetResult<Stop> {
            Err(TargetError::NoSuchCpu)
        }
        fn resume(&mut self) -> TargetResult<Option<Stop>> {
            Ok(None)
        }
    }

    /// Attach a client and turn the crank until the server has it.
    fn attached(server: &mut GdbServer, target: &mut Inert) -> TcpStream {
        let client = TcpStream::connect(server.local_addr().expect("addr")).expect("connect");
        for _ in 0..1000 {
            server.poll(target).expect("poll");
            if server.is_attached() {
                return client;
            }
        }
        panic!("the server never accepted the connection");
    }

    /// Waking on the client's packet must not have turned the halted wait
    /// into a spin: with nothing arriving, each poll still gives the host its
    /// millisecond back. A lower bound only — how *long* a sleep overshoots is
    /// the host's business and asserting on it would be a flaky test.
    #[test]
    fn a_halted_session_with_nothing_to_read_stays_off_the_cpu() {
        let mut server = GdbServer::bind(":0").expect("bind");
        let mut target = Inert;
        let _client = attached(&mut server, &mut target);
        let start = std::time::Instant::now();
        for _ in 0..50 {
            assert_eq!(server.poll(&mut target).expect("poll"), Progress::Halted);
        }
        assert!(
            start.elapsed() >= IDLE_SLEEP * 25,
            "50 idle polls took {:?}; the wait is not waiting",
            start.elapsed()
        );
    }

    /// And the wait leaves the socket as it found it: the byte it noticed is
    /// still there for the poll to read, and a packet that arrives during a
    /// wait is answered by the very next turn.
    #[test]
    fn a_packet_that_arrives_during_the_wait_is_answered_next_turn() {
        let mut server = GdbServer::bind(":0").expect("bind");
        let mut target = Inert;
        let mut client = attached(&mut server, &mut target);
        client
            .set_read_timeout(Some(Duration::from_secs(5)))
            .expect("timeout");
        for _ in 0..20 {
            client.write_all(b"$qAttached#8f").expect("write");
            // At most two turns: one whose wait the packet ends, one to answer.
            server.poll(&mut target).expect("poll");
            server.poll(&mut target).expect("poll");
            let mut reply = [0u8; 6];
            client.read_exact(&mut reply).expect("the reply");
            assert_eq!(&reply, b"+$1#31");
        }
    }

    /// The far end of this socket is whoever reached the port, and it does not
    /// have to read. Every request earns a reply and a `-` earns a copy of the
    /// last one for a single byte, so without a bound a peer that only writes
    /// grows this process at many times the rate it sends.
    #[test]
    fn a_client_that_never_reads_cannot_grow_the_reply_queue_without_bound() {
        let mut server = GdbServer::bind(":0").expect("bind");
        let mut target = Inert;
        let mut client = attached(&mut server, &mut target);
        client.set_nonblocking(true).expect("non-blocking");

        // One real request, so there is a reply worth repeating — about ninety
        // bytes of feature list — and then nothing but "say that again".
        client.write_all(b"$qSupported#37").expect("write");
        let naks = [packet::NAK; 16 * 1024];
        let pending = |server: &GdbServer| server.conn.as_ref().map_or(0, |c| c.pending.len());
        // One reply of slack: the check is made before a request is taken.
        let most = MAX_PENDING + 4096;

        let mut throttled = 0;
        for _ in 0..20_000 {
            // Whatever the socket will take; a full buffer is the point.
            let _ = client.write(&naks);
            server.poll(&mut target).expect("poll");
            assert!(server.is_attached(), "not reading is not hanging up");
            let queued = pending(&server);
            assert!(queued <= most, "{queued} bytes of reply queued");
            if queued >= MAX_PENDING {
                throttled += 1;
                if throttled >= 20 {
                    break;
                }
            }
        }
        assert!(
            throttled >= 20,
            "the queue never filled: {}",
            pending(&server)
        );

        // And it is back pressure rather than a hang-up: once the client reads,
        // the queue drains and the requests held back are answered after all.
        let mut received = 0usize;
        let mut buf = [0u8; 65536];
        for _ in 0..20_000 {
            server.poll(&mut target).expect("poll");
            match client.read(&mut buf) {
                Ok(n) => received += n,
                Err(e) if e.kind() == ErrorKind::WouldBlock => {}
                Err(e) => panic!("read: {e}"),
            }
            if received > MAX_PENDING * 2 {
                break;
            }
        }
        assert!(
            received > MAX_PENDING * 2,
            "only {received} bytes ever arrived"
        );
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
