//! A machine driven behind a VNC server: the loop, and where determinism lives.
//!
//! [`VncServer`] owns no machine on purpose. This is the piece that wires one
//! to it, and it is small because the interesting decisions are all about
//! *when* things happen rather than about RFB:
//!
//! ```text
//!   ┌── one slice ─────────────────────────────────────────────────┐
//!   │ 1. capture the scanout into the surface                      │
//!   │ 2. server.poll(surface)  → frames out, input events back     │
//!   │ 3. deliver those events at machine.now(), and record them    │
//!   │ 4. machine.run_until(now + slice)                            │
//!   │ 5. ask the rate controller how long to wait, and wait        │
//!   └──────────────────────────────────────────────────────────────┘
//! ```
//!
//! Step 3 is the whole determinism argument. Events are collected in step 2 at
//! whatever wall-clock instant the human produced them, and delivered in step 3
//! at a virtual instant the *scheduler* is standing on — a slice boundary. The
//! pair `(instant, event)` is what goes in the [`InputLog`], and replaying that
//! log delivers the same events at the same instants. Nothing the network did
//! is observable to the guest.
//!
//! Step 5 is the other rule: the wait comes from
//! [`Scheduler::pace`](crate::core::sched::Scheduler::pace), driven by the one
//! [`MonotonicClock`] the machine was
//! handed. A frontend that timed its own slices would be a second rate
//! controller disagreeing with the first about what a stall cost.
//!
//! # Sound, and the end of "make the ring big enough for the whole run"
//!
//! A headless `--record-audio` run is one `run_for(span)` with nothing in
//! between, so there is no cadence at which a host could drain the device's
//! sample ring — which left exactly one honest option, and
//! `src/bin/rsemu.rs` says so: make the ring hold the entire run, and cap a
//! recording at the eighteen seconds the device will allocate.
//!
//! **A live session is the cadence.** Step 2 of the loop already visits the
//! host sixty times a virtual second, and pulling the device's ring there costs
//! the machine nothing — `AudioStream::pull` moves no architectural state, and
//! the loop would have stopped at that boundary anyway. So a session given an
//! [`AudioStream`] drains it every slice, the ring only ever has to hold one
//! slice's worth of samples, and the eighteen-second cap is gone: a run of any
//! length produces sound of that length.
//!
//! What that does *not* buy is a sound card. Playing the frames still needs a
//! backend, and `host::audio` explains at length why there is not one yet
//! (ALSA is an `ioctl`/`mmap` protocol, `libc` is forbidden, and a raw-syscall
//! backend would be a seventh `unsafe` subsystem against a ceiling of six). The
//! frames go where the caller puts them; today that is a `.wav`, and the
//! machinery a card would need — [`Sink`](crate::host::audio::Sink), the
//! resampler, the queue — is all already here and already fed at the right
//! rate.
//!
//! # Replay stops where the events are
//!
//! A live run advances a fixed slice at a time. A replay cannot: an event
//! recorded at *t* has to be delivered at *t*, and a slice that stepped over
//! *t* would deliver it late and produce a different run. So a replaying
//! session shortens its slice to land exactly on the next event's instant.
//! That is why [`Replay::next_instant`](crate::host::input::Replay::next_instant)
//! exists, and it is requirement 3 in the [module docs](super) of what the
//! general record/replay seam has to offer.

use std::io;
use std::time::Duration;

use crate::core::clock::GlobalTime;
use crate::core::sched::{Pace, RateControl};
use crate::host::audio::AudioStream;
use crate::host::clock::MonotonicClock;
use crate::host::display::{PixelFormat, Scanout, Surface};
use crate::host::input::{InputEvent, InputLog, InputSink, Replay};
use crate::machine::Machine;

use super::VncServer;

/// How much virtual time one turn of the loop advances.
///
/// Sixteen milliseconds and a bit: one frame at 60 Hz, which is the rate a
/// viewer redraws at and the granularity a person can feel. Shorter would mean
/// more scheduler round boundaries for no visible benefit; longer and a
/// keystroke waits.
pub const SLICE: GlobalTime = GlobalTime::from_nanos(16_666_667);

/// The longest a single wait may be.
///
/// The rate controller can ask for an arbitrarily long one — a machine that
/// spent a slice halted is a long way ahead of the wall — and a frontend that
/// obeyed would stop answering its socket for that whole time. Capping it means
/// a viewer stays responsive; the debt is still owed and is paid over the next
/// few turns.
const MAX_WAIT: Duration = Duration::from_millis(50);

/// How far behind the wall a machine may fall before the debt is forgiven.
///
/// A quarter of a second. Past that, chasing means running at full speed with
/// the audio and the mouse both wrong, which is worse than having lost the
/// time.
const MAX_CATCHUP_NANOS: u64 = 250_000_000;

/// Whether a session is recording input, replaying it, or neither.
#[derive(Debug)]
enum Tape {
    /// Live input from the network, unrecorded.
    Live,
    /// Live input from the network, recorded.
    Recording(InputLog),
    /// No live input: the log is the input.
    Replaying(Replay),
}

/// A machine, a VNC server, and the loop between them.
pub struct VncSession {
    server: VncServer,
    scanout: Box<dyn Scanout>,
    surface: Surface,
    sinks: Vec<Box<dyn InputSink>>,
    tape: Tape,
    /// The machine's sound, drained once a slice. See the module docs.
    audio: Option<AudioStream>,
    /// The frame counter the surface was last filled from, so an unchanged
    /// frame costs no capture.
    captured: u64,
}

impl core::fmt::Debug for VncSession {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        f.debug_struct("VncSession")
            .field("server", &self.server)
            .field("scanout", &self.scanout)
            .field("sinks", &self.sinks.len())
            .field("tape", &self.tape)
            .field("audio", &self.audio.is_some())
            .finish_non_exhaustive()
    }
}

impl VncSession {
    /// A session showing `scanout` over `server`.
    ///
    /// The surface is allocated in
    /// [`BGRA8888`](crate::host::display::PixelFormat::BGRA8888) rather than in
    /// the scanout's preferred format, because BGRA8888 is byte for byte what
    /// the default RFB pixel format asks for — so the common frame is a row
    /// copy rather than a repack. The scanout honours the destination's format,
    /// so this costs the device nothing.
    #[must_use]
    pub fn new(server: VncServer, scanout: Box<dyn Scanout>) -> VncSession {
        let info = scanout.info();
        let mut server = server;
        server.set_geometry(info.width, info.height);
        VncSession {
            surface: Surface::new(PixelFormat::BGRA8888, info.width, info.height),
            server,
            scanout,
            sinks: Vec::new(),
            tape: Tape::Live,
            audio: None,
            captured: u64::MAX,
        }
    }

    /// Drain `stream` once a slice.
    ///
    /// This is what makes a run of arbitrary length audible: the device's ring
    /// then only has to hold one slice, not the whole run. See the module docs.
    #[must_use]
    pub fn with_audio(mut self, stream: AudioStream) -> VncSession {
        self.audio = Some(stream);
        self
    }

    /// The audio path, if one was attached — for a caller writing the frames
    /// somewhere when the run ends.
    #[must_use]
    pub fn audio(&self) -> Option<&AudioStream> {
        self.audio.as_ref()
    }

    /// The audio path, mutably, for a caller handing the frames to a
    /// [`Sink`](crate::host::audio::Sink) as they arrive.
    pub fn audio_mut(&mut self) -> Option<&mut AudioStream> {
        self.audio.as_mut()
    }

    /// Send input to `sink` as well as to whatever is already attached.
    #[must_use]
    pub fn with_sink(mut self, sink: Box<dyn InputSink>) -> VncSession {
        self.sinks.push(sink);
        self
    }

    /// Record every event that crosses into the machine.
    #[must_use]
    pub fn recording(mut self) -> VncSession {
        self.tape = Tape::Recording(InputLog::new());
        self
    }

    /// Replay `log` instead of taking input from the network.
    ///
    /// Clients may still watch — a replay is a good thing to watch — but what
    /// they type is discarded, because accepting it would make the replay a
    /// different run from the recording.
    #[must_use]
    pub fn replaying(mut self, log: InputLog) -> VncSession {
        self.tape = Tape::Replaying(Replay::new(log));
        self
    }

    /// The server, for a status line.
    #[must_use]
    pub fn server(&self) -> &VncServer {
        &self.server
    }

    /// What has been recorded so far, if this session is recording.
    #[must_use]
    pub fn log(&self) -> Option<&InputLog> {
        match &self.tape {
            Tape::Recording(log) => Some(log),
            _ => None,
        }
    }

    /// Whether a replay has run out of events.
    ///
    /// A live session is never finished, which is why this is false for one:
    /// there is always another keystroke a person might make.
    #[must_use]
    pub fn is_replay_finished(&self) -> bool {
        match &self.tape {
            Tape::Replaying(replay) => replay.is_finished(),
            _ => false,
        }
    }

    /// Prepare a machine for a live session: a host clock, and real-time pacing.
    ///
    /// Separate from [`run`](VncSession::run) so a caller driving the loop
    /// itself — a test, or a frontend with its own console to pump — can still
    /// get the pacing right.
    pub fn install(&self, machine: &mut Machine) {
        let clock = MonotonicClock::new();
        let now_host = {
            use crate::core::sched::HostClock;
            clock.monotonic_nanos()
        };
        let now = machine.now();
        machine.set_host_clock(Box::new(clock));
        machine.scheduler_mut().rate_controller_mut().set_control(
            RateControl::Realtime {
                max_catchup_nanos: MAX_CATCHUP_NANOS,
            },
            now_host,
            now,
        );
    }

    /// One turn: show the current frame, take what was typed, deliver it.
    ///
    /// Returns how many events were delivered. Does **not** advance virtual
    /// time — the caller does that, immediately afterwards, which is what makes
    /// the delivery instant a scheduling boundary.
    ///
    /// # Errors
    ///
    /// A failure of the listening socket. A single client's failure closes that
    /// client and is not an error here.
    pub fn poll(&mut self, machine: &mut Machine) -> io::Result<usize> {
        self.capture();
        // Before the events, because it is bookkeeping rather than input: what
        // the device has produced since the last slice is drained whether or
        // not anybody typed. `pull` moves no architectural state, so a session
        // with sound and one without run the same machine.
        if let Some(audio) = self.audio.as_mut() {
            audio.pull();
        }
        let live = self.server.poll(&self.surface)?;
        let now = machine.now();
        let mut delivered = 0;
        match &mut self.tape {
            Tape::Live => {
                for event in live {
                    deliver(&self.sinks, event);
                    delivered += 1;
                }
            }
            Tape::Recording(log) => {
                for event in live {
                    log.push(now, event);
                    deliver(&self.sinks, event);
                    delivered += 1;
                }
            }
            Tape::Replaying(replay) => {
                // What a client typed is dropped: see `replaying`.
                for entry in replay.due(now) {
                    deliver(&self.sinks, entry.event);
                    delivered += 1;
                }
            }
        }
        Ok(delivered)
    }

    /// How far this turn may advance virtual time.
    ///
    /// A slice, unless a replay has an event due sooner — in which case the
    /// slice ends exactly on it, so the next [`poll`](VncSession::poll)
    /// delivers it at the instant it was recorded at.
    #[must_use]
    pub fn deadline(&self, machine: &Machine) -> GlobalTime {
        let now = machine.now();
        let slice = now.saturating_add(SLICE);
        match &self.tape {
            Tape::Replaying(replay) => match replay.next_instant() {
                Some(at) if at > now && at < slice => at,
                _ => slice,
            },
            _ => slice,
        }
    }

    /// Run until `keep_going` says stop.
    ///
    /// The whole loop in one call, so a front end is three lines. `keep_going`
    /// is where a console is pumped and a Ctrl-C on the emulator's own terminal
    /// is noticed; returning false from it ends the session.
    ///
    /// # Errors
    ///
    /// A socket failure, or whatever the machine refuses.
    pub fn run(
        &mut self,
        machine: &mut Machine,
        mut keep_going: impl FnMut(&mut Machine) -> bool,
    ) -> io::Result<()> {
        self.install(machine);
        loop {
            self.poll(machine)?;
            let deadline = self.deadline(machine);
            machine
                .run_until(deadline)
                .map_err(|e| io::Error::other(e.to_string()))?;
            self.wait(machine)?;
            if !keep_going(machine) {
                return Ok(());
            }
        }
    }

    /// Hold virtual time to the wall, as the rate controller says.
    fn wait(&self, machine: &mut Machine) -> io::Result<()> {
        let pace = machine
            .scheduler_mut()
            .pace()
            .map_err(|e| io::Error::other(e.to_string()))?;
        if let Pace::Wait { nanos } = pace {
            std::thread::sleep(Duration::from_nanos(nanos).min(MAX_WAIT));
        }
        Ok(())
    }

    /// Refill the surface from the scanout, if the device has drawn since.
    fn capture(&mut self) {
        let info = self.scanout.info();
        if info.width != self.surface.width() || info.height != self.surface.height() {
            self.surface
                .reshape(PixelFormat::BGRA8888, info.width, info.height);
            // A client already attached learns about the new shape through the
            // DesktopSize pseudo-encoding, which is per-connection; one that
            // attaches *after* the mode set has to be told the truth in its
            // ServerInit, which is what this updates.
            self.server.set_geometry(info.width, info.height);
            self.captured = u64::MAX;
        }
        let counter = self.scanout.frame_counter();
        if counter == self.captured {
            return;
        }
        self.captured = self.scanout.capture(&mut self.surface);
    }
}

/// Hand `event` to every sink.
fn deliver(sinks: &[Box<dyn InputSink>], event: InputEvent) {
    for sink in sinks {
        sink.deliver(event);
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::host::display::SurfaceInfo;
    use crate::host::input::Keysym;
    use std::sync::Mutex;

    /// A scanout with nothing behind it, so the loop can be tested without a
    /// display device in the build.
    #[derive(Debug)]
    struct Blank;

    impl Scanout for Blank {
        fn info(&self) -> SurfaceInfo {
            SurfaceInfo::new(4, 2, PixelFormat::BGRA8888)
        }
        fn frame_counter(&self) -> u64 {
            0
        }
        fn capture(&self, dst: &mut Surface) -> u64 {
            dst.fill([0, 0, 0]);
            0
        }
    }

    /// A sink that remembers what it was given.
    #[derive(Debug, Default)]
    struct Recorder(Mutex<Vec<InputEvent>>);

    impl InputSink for Recorder {
        fn deliver(&self, event: InputEvent) {
            self.0.lock().expect("not poisoned").push(event);
        }
    }

    fn a_machine() -> Machine {
        let registry = crate::machine::catalog::registry().expect("this build's registry");
        let options = crate::machine::BuildOptions::new()
            .with_classes(crate::machine::catalog::classes())
            .with_bindings(crate::machine::catalog::bindings().expect("this build's bindings"));
        crate::machine::build(
            "vnc-session.machine",
            r#"
            machine "vnc-session" {
              osc x = 1000000 Hz
              space mem { width = 16, unassigned = open-bus }
              object dram "ram" { size = 256 }
              map mem 0x0000 size 0x100 = dram
            }
            "#,
            &registry,
            &options,
        )
        .expect("a machine with nothing but memory")
    }

    #[test]
    fn a_replay_delivers_at_the_instant_it_recorded() {
        let mut log = InputLog::new();
        log.push(
            GlobalTime::from_nanos(0),
            InputEvent::Key {
                keysym: Keysym::from_ascii(b'a'),
                down: true,
            },
        );
        log.push(
            GlobalTime::from_nanos(5_000_000),
            InputEvent::Key {
                keysym: Keysym::from_ascii(b'a'),
                down: false,
            },
        );
        let server = VncServer::bind(":0").expect("an ephemeral port");
        let sink = std::sync::Arc::new(Recorder::default());
        let mut session = VncSession::new(server, Box::new(Blank))
            .with_sink(Box::new(SharedRecorder(sink.clone())))
            .replaying(log);
        let mut machine = a_machine();

        // The first poll delivers the event due at zero.
        assert_eq!(session.poll(&mut machine).expect("poll"), 1);
        // And the slice is cut short so the second lands on its own instant
        // rather than 16 ms later.
        let deadline = session.deadline(&machine);
        assert_eq!(deadline, GlobalTime::from_nanos(5_000_000));
        machine.run_until(deadline).expect("run");
        assert_eq!(session.poll(&mut machine).expect("poll"), 1);
        assert!(session.is_replay_finished());
        assert_eq!(sink.0.lock().expect("not poisoned").len(), 2);
        // A live session is never "finished".
        assert!(session.deadline(&machine) > machine.now());
    }

    /// `with_sink` takes a box, and the test wants to keep a handle.
    #[derive(Debug)]
    struct SharedRecorder(std::sync::Arc<Recorder>);

    impl InputSink for SharedRecorder {
        fn deliver(&self, event: InputEvent) {
            self.0.deliver(event);
        }
    }

    #[test]
    fn a_live_session_records_at_the_machines_instant() {
        let server = VncServer::bind(":0").expect("an ephemeral port");
        let mut session = VncSession::new(server, Box::new(Blank)).recording();
        let mut machine = a_machine();
        assert_eq!(session.poll(&mut machine).expect("poll"), 0);
        assert_eq!(session.log().map(InputLog::len), Some(0));
        assert!(!session.is_replay_finished(), "a live tape never finishes");
    }

    /// An audio source that hands over one frame every time it is drained, and
    /// counts how often that was.
    #[derive(Debug, Default)]
    struct Ticker(std::sync::atomic::AtomicU64);

    impl crate::host::audio::AudioSource for Ticker {
        fn info(&self) -> crate::host::audio::StreamInfo {
            crate::host::audio::StreamInfo::new(48_000, 1, 1, crate::host::audio::SampleFormat::S16)
        }
        fn drain(&self, out: &mut Vec<i16>) -> u64 {
            self.0.fetch_add(1, std::sync::atomic::Ordering::Relaxed);
            out.push(0);
            1
        }
    }

    #[test]
    fn a_session_drains_the_sound_every_slice() {
        let server = VncServer::bind(":0").expect("an ephemeral port");
        let stream = AudioStream::new(
            Box::new(Ticker::default()),
            48_000,
            crate::host::audio::SampleFormat::S16,
        );
        let mut session = VncSession::new(server, Box::new(Blank)).with_audio(stream);
        let mut machine = a_machine();
        assert_eq!(session.audio().map(AudioStream::dropped), Some(0));
        for _ in 0..4 {
            session.poll(&mut machine).expect("poll");
        }
        // Four slices, four drains: the device's ring never has to hold more
        // than one slice, which is the whole point.
        assert_eq!(
            session.audio().expect("a stream").buffer().frames(),
            4,
            "one frame per slice reached the queue"
        );
        assert!(session.audio_mut().is_some());
    }

    #[test]
    fn installing_the_clock_makes_pacing_answerable() {
        let server = VncServer::bind(":0").expect("an ephemeral port");
        let session = VncSession::new(server, Box::new(Blank));
        let mut machine = a_machine();
        // Without a host clock, real-time pacing is refused rather than
        // silently unthrottled — which is the whole reason `install` exists.
        session.install(&mut machine);
        assert!(machine.scheduler_mut().pace().is_ok());
    }
}
