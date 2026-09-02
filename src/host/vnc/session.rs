//! A machine driven behind a VNC server: the loop, and where determinism lives.
//!
//! [`VncServer`] owns no machine on purpose. This is the piece that wires one
//! to it, and it is small because the interesting decisions are all about
//! *when* things happen rather than about RFB:
//!
//! ```text
//!   ┌── one slice ──────────────────────────────────────────────────┐
//!   │ 1. capture the scanout into the surface                       │
//!   │ 2. server.poll(surface)  → frames out, input events back      │
//!   │ 3. post those events to the machine's recorder                │
//!   │ 4. machine.run_until(now + slice) — whose first round is where │
//!   │    the recorder delivers them, stamped with that round         │
//!   │ 5. ask the rate controller how long to wait, and wait         │
//!   └───────────────────────────────────────────────────────────────┘
//! ```
//!
//! Step 3 is the whole determinism argument, and it is no longer this module's
//! to make: an event collected in step 2 is **posted to the machine's
//! [`Recorder`](crate::core::record::Recorder)**, which delivers it at the top
//! of the next scheduling round and stamps it with that round's own instant.
//! This loop therefore decides *nothing* about when a keystroke lands, which is
//! the property a private log could not have — a frontend that stamps its own
//! instants has to be trusted to stamp a round boundary, and nothing checked
//! that it did.
//!
//! A session with no recorder attached delivers straight to its sinks, because
//! there is no seam to go through: that is a live run nobody is recording, and
//! `--record-input` is what attaches one.
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
//! # Why a replay no longer has to shorten its slice
//!
//! It used to. An event recorded at *t* has to be delivered at *t*, and a slice
//! that stepped over *t* would deliver it late — so a replaying session cut its
//! slice short to land exactly on the next event's instant. That was
//! requirement 3 in the [module docs](super) of what the general seam had to
//! offer, and the seam answers it in a stronger form: an instant in the
//! recording *is* a scheduling-round boundary, because that is the only place
//! the recorder ever stamps one, and
//! [`Machine::run_until`](crate::machine::Machine::run_until) declines a round
//! it cannot finish rather than splitting it (§11.6). A replay that reaches the
//! same boundaries — which a deterministic run does, by definition — therefore
//! stands on *t* whatever the caller's slice was. The slice is a frame rate
//! again, and nothing about the run depends on it.

use std::io;
use std::sync::Arc;
use std::time::Duration;

use crate::core::clock::GlobalTime;
use crate::core::record::Channel;
use crate::core::sched::{Pace, RateControl};
use crate::host::audio::AudioStream;
use crate::host::clock::MonotonicClock;
use crate::host::display::{PixelFormat, Scanout, Surface};
use crate::host::input::{self, Feed, InputSink};
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

/// A machine, a VNC server, and the loop between them.
pub struct VncSession {
    server: VncServer,
    scanout: Box<dyn Scanout>,
    surface: Surface,
    /// Where an event goes: the machine's own end of the input channel, which
    /// is also what a recorder delivers a replayed payload to.
    feed: Arc<Feed>,
    /// The channel this session posts on.
    channel: Channel,
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
            .field("feed", &self.feed)
            .field("channel", &self.channel.to_string())
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
            feed: Arc::new(Feed::new()),
            channel: input::channel(input::DEFAULT_STREAM),
            audio: None,
            captured: u64::MAX,
        }
    }

    /// Post on `name` rather than on [`input::DEFAULT_STREAM`].
    ///
    /// For a process serving two machines, whose recordings must not name one
    /// stream between them.
    #[must_use]
    pub fn on_stream(mut self, name: &str) -> VncSession {
        self.channel = input::channel(name);
        self
    }

    /// The record/replay channel this session's events cross on.
    #[must_use]
    pub fn channel(&self) -> &Channel {
        &self.channel
    }

    /// The far end of that channel: what a recorded payload is delivered to.
    #[must_use]
    pub fn feed(&self) -> &Arc<Feed> {
        &self.feed
    }

    /// Register this session's channel with `recorder`.
    ///
    /// Call it **before the machine is built** when the host-object table is
    /// going to be sealed, and before the first run in any case: a channel
    /// registered late would have missed everything before it, and
    /// [`Recorder::register`](crate::core::record::Recorder::register) says so.
    ///
    /// # Errors
    ///
    /// [`crate::Error::Config`] if the recorder is already sealed.
    pub fn attach(&self, recorder: &crate::core::record::Recorder) -> crate::Result<()> {
        recorder.register(self.channel.clone(), input::sink(&self.feed))
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
    pub fn with_sink(self, sink: Arc<dyn InputSink>) -> VncSession {
        self.feed.attach(sink);
        self
    }

    /// The server, for a status line.
    #[must_use]
    pub fn server(&self) -> &VncServer {
        &self.server
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

    /// One turn: show the current frame and offer what was typed to the machine.
    ///
    /// Returns how many events crossed the seam. Does **not** advance virtual
    /// time and does not itself decide when an event lands: with a recorder
    /// attached the events are *posted*, and the machine delivers them at the
    /// top of the round the caller's next `run_until` starts — which is the
    /// same instant this used to stamp, arrived at by the machine rather than
    /// by a frontend. With no recorder they go straight to the sinks, because
    /// then there is no seam to go through.
    ///
    /// In a replaying session a client's keystrokes are discarded, which is
    /// [`Recorder::post`](crate::core::record::Recorder::post)'s own rule: a
    /// replay that also took live input would be a different run wearing a
    /// recording's name.
    ///
    /// # Errors
    ///
    /// A failure of the listening socket, or a recorder that does not know this
    /// session's channel — which is a wiring mistake
    /// ([`attach`](VncSession::attach) was not called) rather than something to
    /// paper over by delivering unrecorded.
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
        let mut crossed = 0;
        match machine.recorder() {
            Some(recorder) => {
                for event in live {
                    recorder
                        .post(&self.channel, &event.encode())
                        .map_err(|e| io::Error::other(e.to_string()))?;
                    crossed += 1;
                }
            }
            None => {
                for event in live {
                    self.feed.deliver(event);
                    crossed += 1;
                }
            }
        }
        Ok(crossed)
    }

    /// How far this turn may advance virtual time: one [`SLICE`].
    ///
    /// A replay does not need a shorter one — see the module docs.
    #[must_use]
    pub fn deadline(&self, machine: &Machine) -> GlobalTime {
        machine.now().saturating_add(SLICE)
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

#[cfg(test)]
mod tests {
    use super::*;
    use crate::host::display::SurfaceInfo;
    use crate::host::input::{InputEvent, Keysym};
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
    struct Seen(Mutex<Vec<InputEvent>>);

    impl InputSink for Seen {
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
    fn a_replay_delivers_at_the_instant_it_was_recorded_at() {
        use crate::core::record::{InputEvent as Record, InputLog, Recorder};

        // A recording made elsewhere, with one event on the session's channel
        // at an instant this machine's rounds land on.
        let at = GlobalTime::from_nanos(5_000_000);
        let mut log = InputLog::new();
        let channel = crate::host::input::channel(crate::host::input::DEFAULT_STREAM);
        log.push(Record {
            at,
            channel: channel.clone(),
            payload: InputEvent::Key {
                keysym: Keysym::from_ascii(b'a'),
                down: true,
            }
            .encode()
            .to_vec(),
        })
        .expect("an empty log takes anything");

        let seen = Arc::new(Seen::default());
        let server = VncServer::bind(":0").expect("an ephemeral port");
        let session = VncSession::new(server, Box::new(Blank))
            .with_sink(Arc::clone(&seen) as Arc<dyn InputSink>);
        let replay = Arc::new(Recorder::replaying(log));
        session.attach(&replay).expect("a fresh recorder");

        let mut machine = a_machine();
        machine
            .set_recorder(Arc::clone(&replay))
            .expect("a deterministic machine");
        let mut session = session;

        // Before the event's instant, nothing.
        session.poll(&mut machine).expect("poll");
        machine.run_until(at).expect("run");
        assert!(seen.0.lock().expect("not poisoned").is_empty());

        // The round that starts on it delivers it, and the frontend's slice had
        // nothing to do with when.
        session.poll(&mut machine).expect("poll");
        let deadline = session.deadline(&machine);
        assert_eq!(deadline, machine.now().saturating_add(SLICE));
        machine.run_until(deadline).expect("run");
        assert_eq!(seen.0.lock().expect("not poisoned").len(), 1);
        assert_eq!(replay.cursor(), 1);
    }

    #[test]
    fn a_session_with_no_recorder_delivers_straight_to_its_sinks() {
        let seen = Arc::new(Seen::default());
        let server = VncServer::bind(":0").expect("an ephemeral port");
        let mut session = VncSession::new(server, Box::new(Blank))
            .with_sink(Arc::clone(&seen) as Arc<dyn InputSink>);
        let mut machine = a_machine();
        // Nothing is connected, so nothing was typed — what is asserted is that
        // the unrecorded path exists and reports honestly.
        assert_eq!(session.poll(&mut machine).expect("poll"), 0);
        assert_eq!(session.channel().to_string(), "input:vnc");
        assert_eq!(session.feed().len(), 1);
        // And the feed is what a recorded payload lands in.
        crate::core::record::InputSink::deliver(
            &**session.feed(),
            &InputEvent::Key {
                keysym: Keysym::RETURN,
                down: true,
            }
            .encode(),
        );
        assert_eq!(seen.0.lock().expect("not poisoned").len(), 1);
    }

    #[test]
    fn a_stream_can_be_named() {
        let server = VncServer::bind(":0").expect("an ephemeral port");
        let session = VncSession::new(server, Box::new(Blank)).on_stream("second");
        assert_eq!(session.channel().to_string(), "input:second");
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
