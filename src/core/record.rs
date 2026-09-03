//! The record/replay seam: every non-deterministic input, as `(virtual instant,
//! payload)` (`ROADMAP.md` §4.5, "Record/replay").
//!
//! `CLAUDE.md` states the rule without an exception: *"Any non-deterministic
//! input crossing into the machine goes through the record/replay seam, or it
//! is a determinism bug."* This module is that seam. Until it existed, three
//! devices had written the same paragraph into their own documentation
//! explaining that the seam did not exist and what they were doing instead —
//! the Game Boy's joypad, the PC's MC146818, and `virtio-rng`. Those paragraphs
//! are the specification this module was written against.
//!
//! # What is actually non-deterministic in this tree
//!
//! Measured rather than assumed. A machine's state at instant *t* is a function
//! of its initial state and of everything that crossed into it since, and most
//! of rsemu is already inside that function.
//!
//! The list below was re-derived from the tree rather than carried forward,
//! and re-deriving it moved four rows. **A "yes" here means the mechanism
//! exists and something uses it**; where the mechanism exists and nothing in
//! `src/` reaches for it, the row says so, because a seam that only tests use
//! is a seam the product does not have.
//!
//! | Input | Status |
//! | --- | --- |
//! | A frontend's keys and pointer | **through here** — `host::input` posts twelve-byte records on `input:vnc`, and `host::vnc::session` is the **only** [`Recorder::register`] call site in `src/`. A USB HID pointer rides the same channel |
//! | Gamepad buttons | **registerable, registered only by tests** — a NES pad table, a Game Boy `GbPad` and a Master System `SmsPads` are host objects under `pad:` and each ships a `channel()`/`sink()` pair; `tests/joypad_replay.rs` wires them and nothing in `src/` does |
//! | Network frames | **registerable, registered only by tests** — `dev::net::link::ports` ships the pair and `tests/ne2000_board.rs` wires it. A `PktkitLink` built the other way holds a *private* port that is not in the table at all, and its own docs call that the unrecorded form of the same path |
//! | Keystrokes and terminal bytes | **not through here today.** A character port *is* a host object, so a sealed table would refuse the board — but `host::chardev` ships no `channel()`/`sink()` pair, `host::terminal`'s pump feeds the port directly, and `tests/record_replay.rs` hand-rolls the two closures locally. The gap is two functions beside `ports::open`, and until they exist the CLI's own console is unrecorded |
//! | Guest-visible randomness | already deterministic: `virtio-rng` is a seeded SplitMix64, and says so in its own docs |
//! | The real-time clock | already deterministic: the MC146818 takes its epoch from a `time` property and advances from its own clock domain, never the host's |
//! | The host wall clock | not guest-visible: `HostClock` is injected into the scheduler from above the `std` line and only feeds `Pace::Wait` |
//! | Host file I/O **completion** | nothing to timestamp — every block backend completes inside the guest access that issued it. When one becomes asynchronous, its completion is a channel |
//! | Host file I/O **content** | **not covered, and this is not a timing question.** A drive's bytes are host state outside the recording: a replay against an image the host has since edited diverges, and nothing checks that it is the same image. A medium snapshotted by *reference* compares an identity string, never contents (`machine::Timeline` has the rewind half of this) |
//! | A debugger writing guest state | **not covered** — `host::gdb` lets a TCP peer set registers and memory mid-run. It is a deliberate power rather than an oversight, but a session debugged and recorded at once is not replayable and nothing says so |
//! | Constructor interception | **not covered** — `Bindings::replace` swaps a class for another at build time, which is how `accel` substitutes a KVM core. A build-time door parallel to the host-object table, and the reason an accelerated board is out of scope for replay twice over |
//! | Thread interleaving under `parallel` | **cannot go through here**, which is why replay is deterministic-mode only — see below |
//!
//! # The chokepoint is the host-object table, not the device
//!
//! The obvious design is a `Device` trait method, and it is the wrong one: a
//! device that wants to cheat simply does not call it and nothing notices. The
//! useful observation is that **every door a *device* has onto the host is
//! [`HostObjects`](crate::core::hosts::HostObjects)** — the doors that are not
//! are the frontend's, the debugger's and the embedder's, which the table above
//! lists and none of which a device can reach:
//!
//! * a character port is opened by name through it;
//! * a pad table is opened by name through it — including the Game Boy's and
//!   the Master System's, which reached the host through a
//!   [`Captured`](crate::core::hosts::Captured) table until they were converted
//!   and now open a named `pad:` object like everything else;
//! * a NIC's port is opened by name through it;
//! * and even the remaining *capture* users go through it — a host that wants
//!   a concrete `Arc<NesPpu>` to draw from can only obtain one out of a
//!   `Captured` table, which is itself a host object under
//!   [`HostKind::CAPTURE`](crate::core::hosts::HostKind::CAPTURE).
//!
//! So enforcement is a property of that table rather than of each device.
//! [`HostObjects::seal`](crate::core::hosts::HostObjects::seal) puts a recorder
//! in front of it: from that point on, opening a host object whose channel the
//! recorder has not registered is
//! [`Error::Config`] naming the channel, at
//! *build* time, before the machine has executed an instruction. A device that
//! reaches for the host without declaring itself does not get a mis-recorded
//! run; it gets a machine that refuses to realize.
//!
//! ## The seal is a mechanism, and nothing in `src/` turns it on
//!
//! Stated plainly because the paragraph above reads like a description of what
//! the product does, and it is a description of what the product *can* do.
//! Every `seal` call site in the tree is a test. `machine::realize` carries the
//! table into every device's `Props` and never seals it; the binary builds a
//! recorder, attaches a VNC session and calls `Machine::set_recorder` without
//! sealing. A board therefore realizes whether or not its inputs are declared,
//! and `--record-input` on a machine with a console records the VNC events and
//! not the console.
//!
//! There is a structural reason it has stayed opt-in, and it is worth naming
//! because it is the thing to fix rather than a preference:
//! **[`HostKind`] files three different things under one name space**, and the
//! seal checks all of them alike:
//!
//! * **Doors that are channels** — `chardev`, `pad`, `netdev`, `capture`. An
//!   input arrives as bytes at an instant, so it can be logged and re-delivered,
//!   and demanding a channel for one is exactly right.
//! * **Rendezvous inside the machine** — `pci-bus`, `usb-bus`, `i2c-bus`,
//!   `spi-bus`, `ata-bay`, `sd-slot`, `floppy-drive`, `apic-bus`, `signal`,
//!   `riscv.dt`. How two devices find each other. Nothing crosses from the host
//!   at all, so there is nothing to record and nothing to demand.
//! * **A door that cannot be a channel** — `medium`. Host bytes really do cross
//!   here, but the guest *pulls* them a sector at a time rather than receiving
//!   them at an instant, so no `(instant, payload)` log describes it. What that
//!   needs is an identity check on the image, which is `dev::medium`'s
//!   `Snapshot::Reference` — and a weak one, as the table above says — not a
//!   channel.
//!
//! Only the first group belongs to the seal, and it holds all three, so sealing
//! any board with a PCI or USB bus in it fails on an object that was never an
//! input. The seal works on the Apple 1, the Game Boy and a two-device NIC
//! board, which is exactly the set of boards the tests seal.
//!
//! The fix is a mark on the kind — `HostKind::door()` beside `HostKind::new()`,
//! with `seal` checking only the doors — and it touches every kind's definition
//! site across `bus/` and `dev/`, so it is described here rather than made
//! here. `tests::the_seal_cannot_tell_a_door_from_a_rendezvous` pins the
//! current behaviour so the change has something to flip.
//!
//! What the seal does **not** stop even once it is on is a device that
//! manufactures non-determinism internally — a device calling the host clock in
//! its own read path would never touch this table. Nothing in the type system
//! stops that; what stops it is that `no_std` is the default for `dev/`, CI
//! builds `--no-default-features`, and such a device would not compile there.
//!
//! # Delivery happens at a round boundary, and only there
//!
//! A host thread posts input whenever it likes — [`Recorder::post`] is callable
//! from anywhere and takes no virtual time. Nothing is delivered at that
//! moment. The machine drains the queue at the top of each scheduling round
//! ([`Recorder::deliver`]), stamps everything drained with the round's own
//! [`GlobalTime`], and only then hands it to the sink.
//!
//! That indirection is the whole mechanism, and it is what a private per-device
//! queue cannot give you. Delivering at the instant the host thread happened to
//! call would make the *instant itself* a host-scheduling artefact: two runs of
//! one recording would inject the same bytes at two different virtual times and
//! the guest would see two different machines. Draining on a boundary makes the
//! instant a function of the guest's own timeline, so a replay that reaches the
//! same boundaries — which a deterministic run does, by definition — delivers
//! at exactly the same points.
//!
//! ```text
//!   record:   host thread ──post──► pending ──┐
//!                                             ├─ round boundary at t ─► sink
//!             log.push((t, channel, bytes)) ◄─┘
//!
//!   replay:   log ── entries with at <= t ────► sink        (post is discarded)
//! ```
//!
//! Replay delivers in log order and record delivers in the order it appended to
//! the log, so the two orders are the same object rather than two orders that
//! have to be argued equal.
//!
//! # Replay is deterministic-mode only, and that is structural
//!
//! [`ThreadingMode::Parallel`](crate::core::sched::ThreadingMode::Parallel)
//! joins every job before a round returns, which is a real rendezvous — the
//! *round boundaries* are reproducible under it. They are not what makes a run
//! reproducible. Inside a round, two CPU threads interleave their accesses to
//! shared RAM and to shared MMIO in an order the host scheduler picks, and the
//! ticks each one reports back depend on what it read. Making that replayable
//! would mean logging every memory access, which is not a seam, it is a second
//! emulator.
//!
//! So [`Machine::set_recorder`](crate::machine::Machine::set_recorder) refuses a
//! machine that is not in a deterministic threading mode, for the same reason
//! and in the same shape as
//! [`Machine::state_hash`](crate::machine::Machine::state_hash): a recording
//! taken from a parallel run is a sample, and the call that would produce it
//! returns an error rather than a comment somebody has to have read.
//!
//! # There is a second seam, and it is a different shape on purpose
//!
//! [`usermode::journal`](crate::usermode::journal) is also called "the
//! record/replay seam" in its own documentation, and it is not this one. The
//! difference is direction, and it is real:
//!
//! | | this module | `usermode::journal` |
//! | --- | --- | --- |
//! | who initiates | the **host** pushes | the **guest** pulls |
//! | when it happens | at a round boundary, decided here | inside a guest instruction, decided by the guest |
//! | what is keyed on | `(instant, channel)` | the *order* the questions arrive in |
//! | what a replay does | re-delivers at the same instant | answers without asking the host |
//!
//! A syscall's result cannot be delivered at a round boundary: the guest is
//! blocked on it mid-instruction, and the answer's *place in the sequence* is
//! what identifies it. A keystroke cannot be pulled: nothing in the guest asks
//! for it, and the instant it arrives is the only thing that decides what the
//! machine does with it. Forcing either into the other's shape would break the
//! one it was forced into.
//!
//! That was written as a deferral — "a design review, not a commit" — and the
//! review has now been held. **The split stands, and the reason is sharper
//! than direction: each seam's key is the other's check.** Here the key is
//! `(instant, channel)` and a channel with no sink is skipped, which is what
//! makes `rsemu replay` work on a headless host with nothing attached. There
//! the key is position in the sequence and a mismatched tag is a divergence
//! reported at once. Swap them and each loses the property it exists for.
//!
//! The tempting unification is that [`Recorder::deliver`] is *itself* a pull —
//! the machine asks "what arrived?" once per round — so input could be
//! journalled as one question per round boundary. It could, and a recorded
//! Apple 1 session would go from three entries to one per scheduling round
//! unless empty answers were elided, which is instant-keying reintroduced; a
//! round's *N* payloads across *M* channels would have to be re-encoded inside
//! one answer's bytes; and the missing-sink tolerance would be gone.
//! `usermode::journal` carries the argument in full from the other side.
//!
//! What they share, and as of this review actually do rather than aspire to,
//! is the **vocabulary and the file discipline**: a [`GlobalTime`] stamp, a
//! byte payload, an encoder over `core::state`'s [`Sink`] and [`Source`], a
//! magic, a format version of its own, tagged entries, one canonical encoding,
//! and a fuzz target holding the reader to it. The journal had none of the last
//! five and now has all of them. A session that has both ends up with two
//! sections of one vocabulary rather than two file formats.
//!
//! One asymmetry survives on purpose. A recording carries a
//! [`MachineShape`] because a wrong board *silently accepts* input on a channel
//! whose name happens to match; a journal carries no fingerprint because a
//! wrong program diverges at the first question whose tag differs, which is a
//! better check and comes for free. A third mechanism for either job is still a
//! design review, not a commit.
//!
//! # How a device with a private mechanism adopts this
//!
//! Three did, and the recipe is the same each time: **the device keeps the
//! object the host was already reaching for, and that object becomes a
//! channel.** What it gives up is the *stamping* — deciding which instant an
//! input arrived at — and that is exactly the part it should give up, because a
//! device that timestamps its own input has to be trusted to pick a round
//! boundary and nothing checks that it did. What it gains is the file format,
//! the shape check, the rewind hook, and the seal.
//!
//! [`dev::net::link::ports`](crate::dev::net::link::ports) is the worked
//! example. Its `NetPort` had grown a `Vec<(tick, frame)>` of its own; the log
//! is gone, the receive path is untouched, and the wiring is three lines:
//!
//! ```text
//! // `dev::net::link::ports`, which this module cannot name: `core` is never
//! // feature-gated and a NIC is.
//! let port = ports::open(hosts, "net0")?;
//! recorder.register(ports::channel("net0"), ports::sink(&port))?;
//! ```
//!
//! `dev::gb::joypad::pads` and `dev::sms::io::pads` are the same shape with a
//! *level* payload rather than a queued one — a held-button mask, which is what
//! a matrix is — and so they register no rewind hook: a level is architectural
//! state that the snapshot already carries. `host::input` is the same shape
//! again with a payload of fixed twelve-byte event records, and its channel is
//! the one case with no host object behind it, because a frontend is not part
//! of the machine (`host::input` says why).
//!
//! A frame, a button and a keystroke are all payloads. None of them needed a
//! second mechanism.
//!
//! ## What a *new* device has to do to be covered
//!
//! Four things, and only the first two are about this module:
//!
//! 1. **Open the host object through
//!    [`HostObjects`](crate::core::hosts::HostObjects)** — which a device does
//!    already, through `Props::host`, or it has no way to reach the host at all.
//! 2. **Ship a `channel()` and a `sink()` beside the `open()`**, in the same
//!    module, so a caller can wire the two together without knowing what the
//!    object is. This is the step that gets skipped: `chardev` has an `open`
//!    and no pair, so the CLI's console is unrecorded while a test that writes
//!    the two closures by hand records it fine. A pair is ten lines.
//! 3. **Register a rewind hook if the object queues bytes the guest has not
//!    read yet**, and none if it holds a level — a held-button mask is
//!    architectural state the snapshot already carries.
//! 4. **Call [`Recorder::post`] from wherever the host thread arrives**, and
//!    never deliver directly. A device that keeps a private path for "when
//!    there is no recorder" has two paths, and the recorded one is the one
//!    nobody exercises.
//!
//! None of that is enforced. Step 2's absence is invisible — nothing fails, the
//! recording is simply short — which is the argument for the seal being on by
//! default rather than opt-in.
//!
//! # The recording is a file, so it is a parser
//!
//! [`InputLog::encode`] and [`InputLog::decode`] use `core::state`'s
//! [`Sink`] and [`Source`], inherit its little-endian fixed-width discipline,
//! and carry the machine's
//! [`MachineShape`] so a recording replayed
//! into a different board fails with a
//! [`ShapeDiff`](crate::core::state::ShapeDiff) rather than by injecting
//! keystrokes into whatever device happens to answer to that name. The reader
//! never panics, never allocates against a claimed count, and enforces the one
//! canonical ordering it writes, exactly as the snapshot reader does. `fuzz/`
//! carries a target for it.
//!
//! That shape was **written and never read** until
//! [`Machine::set_recorder`](crate::machine::Machine::set_recorder) started
//! diffing it: it overwrote the recorded shape with the machine's own, so the
//! sentence above described an intent rather than a check. It is a check now —
//! [`Recorder::shape`] is the accessor — and an empty shape still means unknown
//! provenance rather than a mismatch, so a log built by hand replays.
//!
//! # Cross-host replay
//!
//! The property the format exists for, and the one nothing tested: a recording
//! written here replays *elsewhere*. Two runs inside one process agreeing
//! proves reproducibility on this host and nothing about any other, however
//! many times it is asserted.
//!
//! What makes it checkable is pinning the artefact rather than re-deriving it.
//! `tests/record_replay.rs` holds a frozen recording and the state hash it
//! replays to as constants, so the comparison crosses processes, builds and
//! machines; CI already runs the test suite on three operating systems and two
//! instruction sets, which makes those three hosts without a line of CI
//! change. It has been checked by hand on a 32-bit target too, which is the
//! informative direction: `usize` is four bytes there, and neither the
//! encoding nor the hash moves.
//!
//! Everything in the format is chosen for that: fixed-width little-endian
//! integers, a `u128` of raw 2⁻⁶⁴-second units rather than a rounded
//! nanosecond count, `BTreeMap` rather than a hash map, lengths as `u64` rather
//! than `usize`, and no float anywhere near the time path.
//!
//! # Example
//!
//! ```
//! use std::sync::Arc;
//! use rsemu::core::clock::GlobalTime;
//! use rsemu::core::hosts::HostKind;
//! use rsemu::core::record::{Channel, FnSink, Recorder};
//! use rsemu::core::sync::{LockRank, Mutex};
//!
//! let seen: Arc<Mutex<Vec<u8>>> = Arc::new(Mutex::with_rank(LockRank::LEAF, Vec::new()));
//! let sink = {
//!     let seen = Arc::clone(&seen);
//!     Arc::new(FnSink::new("demo", move |bytes: &[u8]| {
//!         seen.lock().extend_from_slice(bytes);
//!     }))
//! };
//!
//! let channel = Channel::new(HostKind::new("chardev"), "console");
//! let recorder = Recorder::recording();
//! recorder.register(channel.clone(), sink).unwrap();
//!
//! recorder.post(&channel, b"hi").unwrap();
//! recorder.deliver(GlobalTime::from_nanos(1_000_000)).unwrap();
//! assert_eq!(&*seen.lock(), b"hi");
//!
//! let log = recorder.log();
//! assert_eq!(log.len(), 1);
//! assert_eq!(log.events()[0].at, GlobalTime::from_nanos(1_000_000));
//! ```

use alloc::boxed::Box;
use alloc::collections::BTreeMap;
use alloc::format;
use alloc::string::{String, ToString};
use alloc::sync::Arc;
use alloc::vec::Vec;
use core::fmt;

use crate::core::clock::GlobalTime;
use crate::core::error::{Error, Result};
use crate::core::hosts::HostKind;
use crate::core::state::{MachineShape, Sink, SliceSource, Source};
use crate::core::sync::{LockRank, Mutex};

/// Magic at the start of every recording.
const MAGIC: [u8; 8] = *b"RSEMURPL";

/// The recording container format version.
///
/// Independent of [`FORMAT_VERSION`](crate::core::state::FORMAT_VERSION): a
/// recording embeds a snapshot's *shape* but none of its chunks, so the two
/// framings change for different reasons.
pub const LOG_FORMAT_VERSION: u32 = 1;

/// Tag byte introducing one more input event.
const TAG_EVENT: u8 = 0x01;

/// Tag byte marking the end of the event list.
const TAG_END: u8 = 0x00;

/// The largest payload one event may carry, in bytes.
///
/// A jumbo Ethernet frame is 9 KiB and a pasted terminal buffer is smaller than
/// that, so 64 KiB is comfortably above every input this seam carries and well
/// below anything worth an allocation refusal. It is a *format* limit, checked
/// on write as well as on read, so a corrupt length cannot become a large
/// allocation even before the bytes-remaining check catches it.
pub const MAX_PAYLOAD: usize = 64 * 1024;

/// Build an [`Error::State`] for a malformed recording.
fn log_error(message: String) -> Error {
    Error::State(message)
}

// ---------------------------------------------------------------------------
// Channel
// ---------------------------------------------------------------------------

/// One stream of non-deterministic input, named the way the host names it.
///
/// A `(kind, name)` pair — the same pair
/// [`HostObjects`](crate::core::hosts::HostObjects) files objects under, which
/// is what lets the table check a channel against the recorder without either
/// side knowing what the other's objects are. `chardev:console` is the Apple
/// 1's keyboard, `pad:player1` is a NES controller, `net:eth0` is a NIC's
/// frames.
///
/// The kind is stored as a [`String`] rather than a [`HostKind`] because a
/// recording is decoded from bytes and `HostKind` wraps a `&'static str`.
/// [`Channel::is_kind`] compares back against one.
#[derive(Debug, Clone, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct Channel {
    kind: String,
    name: String,
}

impl Channel {
    /// The channel for `name` under `kind`.
    #[must_use]
    pub fn new(kind: HostKind, name: &str) -> Channel {
        Channel::from_parts(kind.as_str(), name)
    }

    /// The channel for a kind that came off the wire rather than from a `const`.
    #[must_use]
    pub fn from_parts(kind: &str, name: &str) -> Channel {
        Channel {
            kind: kind.to_string(),
            name: name.to_string(),
        }
    }

    /// The host-object kind half.
    #[must_use]
    pub fn kind(&self) -> &str {
        &self.kind
    }

    /// The object-name half.
    #[must_use]
    pub fn name(&self) -> &str {
        &self.name
    }

    /// Whether this channel names `kind`.
    #[must_use]
    pub fn is_kind(&self, kind: HostKind) -> bool {
        self.kind == kind.as_str()
    }
}

impl fmt::Display for Channel {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{}:{}", self.kind, self.name)
    }
}

// ---------------------------------------------------------------------------
// InputSink
// ---------------------------------------------------------------------------

/// Where a channel's payloads are put once the machine has decided *when*.
///
/// Deliberately narrow: bytes in, nothing out. A sink is a character port's
/// feed queue, a pad's button byte, a NIC's receive queue. It is called from
/// the run loop at a round boundary with **no machine lock held**, so it may
/// take its own — but it must not call back into the recorder and it must not
/// block.
///
/// `Send + Sync` from the first commit like every other device-facing trait,
/// and `Debug` so a failing test can print which channel it was.
pub trait InputSink: Send + Sync + fmt::Debug {
    /// Deliver one payload.
    ///
    /// A sink that cannot take all of it drops the remainder and says nothing:
    /// back pressure is the *host's* problem on the way in, not the run loop's
    /// on the way out. A recording faithfully re-delivers what was delivered,
    /// including a payload the sink then partly discarded, because that is what
    /// happened.
    fn deliver(&self, payload: &[u8]);

    /// Forget anything queued and not yet consumed by the guest.
    ///
    /// Called by [`Recorder::rewind_to`]. The default does nothing, which is
    /// right for a sink that keeps no queue of its own; a character port
    /// overrides it to clear, because the bytes sitting in its queue at the
    /// rewind target will be re-delivered from the log and would otherwise
    /// arrive twice.
    fn on_rewind(&self) {}
}

/// An [`InputSink`] made from a closure.
///
/// The adapter that keeps `core::record` from having to know what a character
/// port is: the host, or a test, supplies the two lines that connect a channel
/// to whatever object it feeds. `label` is what a `Debug` print shows.
///
/// The rewind hook is a second, optional closure rather than a second type,
/// because most sinks do not need one and the ones that do — anything holding
/// bytes the guest has not read yet — need exactly one line.
pub struct FnSink<F> {
    label: &'static str,
    deliver: F,
    rewind: Option<Box<dyn Fn() + Send + Sync>>,
}

impl<F: Fn(&[u8]) + Send + Sync> FnSink<F> {
    /// A sink that calls `deliver`, printing as `label`.
    pub fn new(label: &'static str, deliver: F) -> FnSink<F> {
        FnSink {
            label,
            deliver,
            rewind: None,
        }
    }

    /// Also call `rewind` when the timeline goes backwards.
    ///
    /// A character port passes `move || port.clear()` here: the bytes queued at
    /// the rewind target are re-delivered from the log, so a port that kept
    /// them would hand the guest each one twice.
    #[must_use]
    pub fn on_rewind(mut self, rewind: impl Fn() + Send + Sync + 'static) -> FnSink<F> {
        self.rewind = Some(Box::new(rewind));
        self
    }
}

impl<F> fmt::Debug for FnSink<F> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("FnSink")
            .field("label", &self.label)
            .field("rewinds", &self.rewind.is_some())
            .finish()
    }
}

impl<F: Fn(&[u8]) + Send + Sync> InputSink for FnSink<F> {
    fn deliver(&self, payload: &[u8]) {
        (self.deliver)(payload);
    }

    fn on_rewind(&self) {
        if let Some(rewind) = &self.rewind {
            rewind();
        }
    }
}

/// A sink that discards everything, for a channel whose object a replay does
/// not have — a recording examined offline, a headless `rsemu replay`.
#[derive(Debug, Clone, Copy, Default)]
pub struct NullSink;

impl InputSink for NullSink {
    fn deliver(&self, _payload: &[u8]) {}
}

// ---------------------------------------------------------------------------
// InputEvent and InputLog
// ---------------------------------------------------------------------------

/// One non-deterministic input, at the virtual instant it crossed in.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct InputEvent {
    /// The scheduling-round boundary this was delivered on.
    pub at: GlobalTime,
    /// Which stream it belongs to.
    pub channel: Channel,
    /// The bytes, at most [`MAX_PAYLOAD`] of them.
    pub payload: Vec<u8>,
}

/// A recording: every input that crossed into one machine, in delivery order.
///
/// The shape is the machine's structural fingerprint at the moment recording
/// started, so replaying into a different board fails with a diff (§4.5,
/// "Machine identity is a diff, not a boolean").
#[derive(Debug, Clone, Default, PartialEq, Eq)]
pub struct InputLog {
    shape: MachineShape,
    events: Vec<InputEvent>,
}

impl InputLog {
    /// An empty log for a machine of no particular shape.
    #[must_use]
    pub fn new() -> InputLog {
        InputLog::default()
    }

    /// An empty log for a machine of this shape.
    #[must_use]
    pub fn for_shape(shape: MachineShape) -> InputLog {
        InputLog {
            shape,
            events: Vec::new(),
        }
    }

    /// The machine shape this recording was taken from.
    #[must_use]
    pub fn shape(&self) -> &MachineShape {
        &self.shape
    }

    /// Replace the shape, for a recorder that learned it after construction.
    pub fn set_shape(&mut self, shape: MachineShape) {
        self.shape = shape;
    }

    /// Every event, in delivery order.
    #[must_use]
    pub fn events(&self) -> &[InputEvent] {
        &self.events
    }

    /// How many events there are.
    #[must_use]
    pub fn len(&self) -> usize {
        self.events.len()
    }

    /// Whether nothing was recorded.
    #[must_use]
    pub fn is_empty(&self) -> bool {
        self.events.is_empty()
    }

    /// The instant of the last event, if there is one.
    #[must_use]
    pub fn last_instant(&self) -> Option<GlobalTime> {
        self.events.last().map(|e| e.at)
    }

    /// Append an event.
    ///
    /// # Errors
    ///
    /// [`Error::State`] if `at` is before the last event already logged — a log
    /// is delivery-ordered by construction — or if the payload is longer than
    /// [`MAX_PAYLOAD`].
    pub fn push(&mut self, event: InputEvent) -> Result<()> {
        if event.payload.len() > MAX_PAYLOAD {
            return Err(log_error(format!(
                "input payload of {} bytes on channel `{}` exceeds the {MAX_PAYLOAD}-byte limit",
                event.payload.len(),
                event.channel
            )));
        }
        if let Some(last) = self.events.last()
            && event.at < last.at
        {
            return Err(log_error(format!(
                "input event at {} follows one at {}: a recording is delivery-ordered",
                event.at.raw(),
                last.at.raw()
            )));
        }
        self.events.push(event);
        Ok(())
    }

    /// The index of the first event at or after `at`.
    ///
    /// What a rewind sets the replay cursor to. A binary search rather than a
    /// scan because a long session's log is the one thing in this module that
    /// is genuinely large.
    #[must_use]
    pub fn index_at(&self, at: GlobalTime) -> usize {
        self.events.partition_point(|e| e.at < at)
    }

    /// Encode the recording.
    ///
    /// # Errors
    ///
    /// [`Error::State`] if an event exceeds [`MAX_PAYLOAD`], which a log built
    /// through [`InputLog::push`] cannot contain.
    pub fn encode(&self) -> Result<Vec<u8>> {
        let mut out: Vec<u8> = Vec::new();
        out.write_all(&MAGIC)?;
        out.write_u32(LOG_FORMAT_VERSION)?;
        self.shape.encode_into(&mut out)?;
        for event in &self.events {
            if event.payload.len() > MAX_PAYLOAD {
                return Err(log_error(format!(
                    "input payload of {} bytes on channel `{}` exceeds the \
                     {MAX_PAYLOAD}-byte limit",
                    event.payload.len(),
                    event.channel
                )));
            }
            out.write_u8(TAG_EVENT)?;
            out.write_u128(event.at.raw())?;
            out.write_str(event.channel.kind())?;
            out.write_str(event.channel.name())?;
            out.write_bytes(&event.payload)?;
        }
        out.write_u8(TAG_END)?;
        Ok(out)
    }

    /// Decode a recording, rejecting anything that is not the canonical form
    /// [`InputLog::encode`] writes.
    ///
    /// This is a parser on untrusted input and behaves like `core::state`'s: it
    /// never panics, never trusts a length it has not compared against the
    /// bytes remaining, and never allocates against a claimed count.
    ///
    /// # Errors
    ///
    /// [`Error::State`] naming what was expected: a bad magic, an unsupported
    /// format version, an out-of-order or oversized event, an unknown tag, or
    /// trailing bytes.
    pub fn decode(bytes: &[u8]) -> Result<InputLog> {
        let mut src = SliceSource::new(bytes);
        let magic = src.take(MAGIC.len())?;
        if magic != MAGIC {
            return Err(log_error(format!(
                "not a recording: magic {magic:02x?}, expected {MAGIC:02x?}"
            )));
        }
        let format = src.read_u32()?;
        if format != LOG_FORMAT_VERSION {
            return Err(log_error(format!(
                "recording format version {format} (this build reads {LOG_FORMAT_VERSION})"
            )));
        }
        let shape = MachineShape::decode_from(&mut src)?;
        let mut log = InputLog::for_shape(shape);
        loop {
            match src.read_u8()? {
                TAG_END => break,
                TAG_EVENT => {
                    let at = GlobalTime::from_raw(src.read_u128()?);
                    let kind = src.read_string()?;
                    let name = src.read_string()?;
                    let payload = src.read_bytes()?;
                    if payload.len() > MAX_PAYLOAD {
                        return Err(log_error(format!(
                            "recorded payload of {} bytes exceeds the {MAX_PAYLOAD}-byte limit",
                            payload.len()
                        )));
                    }
                    log.push(InputEvent {
                        at,
                        channel: Channel::from_parts(&kind, &name),
                        payload: payload.to_vec(),
                    })?;
                }
                tag => {
                    return Err(log_error(format!(
                        "unknown tag 0x{tag:02x} in a recording (expected 0x00 or 0x01)"
                    )));
                }
            }
        }
        if src.remaining() != 0 {
            return Err(log_error(format!(
                "{} trailing byte(s) after the end of a recording",
                src.remaining()
            )));
        }
        Ok(log)
    }
}

// ---------------------------------------------------------------------------
// Mode
// ---------------------------------------------------------------------------

/// What a [`Recorder`] does with the input that reaches it.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Mode {
    /// Host input is delivered and logged.
    Record,
    /// Host input is **discarded**; the log is delivered instead.
    ///
    /// Discarding rather than merging is the point. A replay that also accepted
    /// live input would be a different run wearing a recording's name, and the
    /// bug it was made to reproduce would not reproduce.
    Replay,
}

impl fmt::Display for Mode {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(match self {
            Mode::Record => "record",
            Mode::Replay => "replay",
        })
    }
}

// ---------------------------------------------------------------------------
// Recorder
// ---------------------------------------------------------------------------

/// The mutable half of a recorder, behind one lock.
///
/// One [`Mutex`] rather than four because the invariant is across them: a
/// delivery reads the channel table, drains the pending queue and appends to
/// the log, and a rewind rewrites the cursor and the queue together. The lock
/// is never held across a call into a sink — [`Recorder::deliver`] takes the
/// batch, releases, and *then* delivers (`CLAUDE.md`, the re-entrancy
/// contract).
#[derive(Debug)]
struct RecorderState {
    channels: BTreeMap<Channel, Arc<dyn InputSink>>,
    pending: Vec<(Channel, Vec<u8>)>,
    log: InputLog,
    cursor: usize,
    sealed: bool,
}

/// The seam itself: one per machine, shared by every host thread that feeds it.
///
/// See the module documentation for what it is and why delivery happens where
/// it does.
#[derive(Debug)]
pub struct Recorder {
    mode: Mode,
    /// [`LockRank::LEAF`]: taken at a round boundary and from host threads,
    /// never from inside a guest access, and never held across an outward call.
    state: Mutex<RecorderState>,
}

impl Recorder {
    /// A recorder that logs what the host feeds it.
    #[must_use]
    pub fn recording() -> Recorder {
        Recorder::with(Mode::Record, InputLog::new())
    }

    /// A recorder that replays `log` and ignores the host.
    #[must_use]
    pub fn replaying(log: InputLog) -> Recorder {
        Recorder::with(Mode::Replay, log)
    }

    fn with(mode: Mode, log: InputLog) -> Recorder {
        Recorder {
            mode,
            state: Mutex::with_rank(
                LockRank::LEAF,
                RecorderState {
                    channels: BTreeMap::new(),
                    pending: Vec::new(),
                    log,
                    cursor: 0,
                    sealed: false,
                },
            ),
        }
    }

    /// Which way this recorder runs.
    #[must_use]
    pub fn mode(&self) -> Mode {
        self.mode
    }

    /// Bind `channel` to the object its payloads go to.
    ///
    /// Registering the same channel twice replaces the sink, which is what a
    /// rebuilt machine wants: the recording is keyed by name, and the object
    /// behind the name is a different `Arc` in the replay run than it was in
    /// the recorded one.
    ///
    /// # Errors
    ///
    /// [`Error::Config`] if the recorder has been sealed onto a host-object
    /// table. Registration is a build-time act; a channel appearing after the
    /// machine is running would silently have missed everything before it.
    pub fn register(&self, channel: Channel, sink: Arc<dyn InputSink>) -> Result<()> {
        let mut state = self.state.lock();
        if state.sealed {
            return Err(Error::Config {
                at: channel.to_string(),
                message: String::from(
                    "a channel cannot be registered after the recorder is sealed onto a \
                     host-object table: register every channel before the machine is built",
                ),
            });
        }
        state.channels.insert(channel, sink);
        Ok(())
    }

    /// Whether `channel` has been registered.
    #[must_use]
    pub fn knows(&self, channel: &Channel) -> bool {
        self.state.lock().channels.contains_key(channel)
    }

    /// Every registered channel, in name order.
    #[must_use]
    pub fn channels(&self) -> Vec<Channel> {
        self.state.lock().channels.keys().cloned().collect()
    }

    /// Stop accepting new channels.
    ///
    /// Called by [`HostObjects::seal`](crate::core::hosts::HostObjects::seal).
    /// Separate from sealing the table so a recorder used without one — a unit
    /// test, a replay with no machine — is not forced through it.
    pub fn seal(&self) {
        self.state.lock().sealed = true;
    }

    /// Whether new channels are still accepted.
    #[must_use]
    pub fn is_sealed(&self) -> bool {
        self.state.lock().sealed
    }

    /// Offer `payload` on `channel`. **Nothing is delivered yet.**
    ///
    /// Callable from any thread at any moment; the machine decides *when*. In
    /// [`Mode::Replay`] the payload is discarded and `Ok(false)` comes back, so
    /// a host that pumps a live terminal into a replaying machine gets an
    /// honest answer rather than a silently different run.
    ///
    /// Returns whether the payload was queued.
    ///
    /// # Errors
    ///
    /// [`Error::Config`] if `channel` is not registered — the seam's own rule,
    /// applied to the host as well as to the device. [`Error::State`] if the
    /// payload is longer than [`MAX_PAYLOAD`].
    pub fn post(&self, channel: &Channel, payload: &[u8]) -> Result<bool> {
        if payload.len() > MAX_PAYLOAD {
            return Err(log_error(format!(
                "input payload of {} bytes on channel `{channel}` exceeds the \
                 {MAX_PAYLOAD}-byte limit",
                payload.len()
            )));
        }
        let mut state = self.state.lock();
        if !state.channels.contains_key(channel) {
            return Err(Error::Config {
                at: channel.to_string(),
                message: String::from(
                    "no such input channel: register it with the recorder before posting to it",
                ),
            });
        }
        if self.mode == Mode::Replay {
            return Ok(false);
        }
        state.pending.push((channel.clone(), payload.to_vec()));
        Ok(true)
    }

    /// Deliver everything due at `now`, returning how many payloads went out.
    ///
    /// Called by the machine at the top of a scheduling round and nowhere else.
    /// In [`Mode::Record`] that is whatever the host has posted since the last
    /// round, stamped `now` and appended to the log. In [`Mode::Replay`] it is
    /// every logged event at or before `now` that the cursor has not passed.
    ///
    /// The lock is released before the first sink is called, so a sink may take
    /// its own and a slow one does not block a host thread that is posting.
    ///
    /// # Errors
    ///
    /// [`Error::State`] if the log refuses an append, which means virtual time
    /// went backwards between rounds.
    pub fn deliver(&self, now: GlobalTime) -> Result<usize> {
        // Phase one, under the lock: decide what goes out, and to whom.
        let batch: Vec<(Arc<dyn InputSink>, Vec<u8>)> = {
            let mut state = self.state.lock();
            match self.mode {
                Mode::Record => {
                    let drained = core::mem::take(&mut state.pending);
                    let mut batch = Vec::with_capacity(drained.len());
                    for (channel, payload) in drained {
                        // An unregistered channel cannot reach here: `post`
                        // refuses one and a sink is never unregistered.
                        let Some(sink) = state.channels.get(&channel).cloned() else {
                            continue;
                        };
                        state.log.push(InputEvent {
                            at: now,
                            channel,
                            payload: payload.clone(),
                        })?;
                        batch.push((sink, payload));
                    }
                    batch
                }
                Mode::Replay => {
                    // The log is delivery-ordered, so where the cursor stops is
                    // a binary search rather than a walk — which matters on a
                    // long session, where this runs once per scheduling round.
                    let end = state
                        .log
                        .events
                        .partition_point(|e| e.at <= now)
                        .max(state.cursor);
                    let mut batch = Vec::with_capacity(end - state.cursor);
                    for index in state.cursor..end {
                        let event = &state.log.events[index];
                        // A channel the replay has no object for is skipped
                        // rather than fatal: replaying on a machine with no
                        // terminal attached is a legitimate thing to do, and
                        // the cursor still passes it so the timeline is right.
                        if let Some(sink) = state.channels.get(&event.channel) {
                            batch.push((Arc::clone(sink), event.payload.clone()));
                        }
                    }
                    state.cursor = end;
                    batch
                }
            }
        };

        // Phase two, with nothing held.
        for (sink, payload) in &batch {
            sink.deliver(payload);
        }
        Ok(batch.len())
    }

    /// The recording so far.
    #[must_use]
    pub fn log(&self) -> InputLog {
        self.state.lock().log.clone()
    }

    /// Record the machine's shape on the log, so a replay can check it.
    pub fn set_shape(&self, shape: MachineShape) {
        self.state.lock().log.set_shape(shape);
    }

    /// The shape the log carries: for a replay, the board the recording was
    /// taken from.
    ///
    /// The half of "machine identity is a diff, not a boolean" that lives on
    /// this side. A recording written by [`InputLog::encode`] has always
    /// carried it; until
    /// [`Machine::set_recorder`](crate::machine::Machine::set_recorder) read it
    /// back, nothing ever compared it, and a recording of one board replayed
    /// into another delivered its input to whatever device answered to the same
    /// channel name.
    ///
    /// Empty for a log built by hand — a unit test, a fuzz case — which is why
    /// the check is skipped rather than failed in that case: an empty shape is
    /// *unknown provenance*, not a mismatch.
    #[must_use]
    pub fn shape(&self) -> MachineShape {
        self.state.lock().log.shape().clone()
    }

    /// How many logged events have been replayed.
    #[must_use]
    pub fn cursor(&self) -> usize {
        self.state.lock().cursor
    }

    /// Move the replay cursor back to `at` and forget queued input.
    ///
    /// The recorder's half of a rewind
    /// ([`Timeline`](crate::machine::Timeline)). Three things happen, and all
    /// three are needed:
    ///
    /// * the cursor moves to the first event at or after `at`, so the events
    ///   between there and where the machine had got to are delivered again;
    /// * anything the host posted and the machine has not yet taken is dropped,
    ///   because it belongs to a future that is being discarded;
    /// * every registered sink is told, so a port holding bytes the guest had
    ///   not read yet clears them rather than seeing them twice.
    ///
    /// In [`Mode::Record`] the log is **truncated** at `at`: a rewind followed
    /// by fresh input creates a new future, and keeping the old one would make
    /// the recording describe a run that never happened.
    pub fn rewind_to(&self, at: GlobalTime) {
        let sinks: Vec<Arc<dyn InputSink>> = {
            let mut state = self.state.lock();
            state.pending.clear();
            let index = state.log.index_at(at);
            state.cursor = index;
            if self.mode == Mode::Record {
                state.log.events.truncate(index);
            }
            state.channels.values().cloned().collect()
        };
        for sink in &sinks {
            sink.on_rewind();
        }
    }
}

#[cfg(test)]
mod tests;
