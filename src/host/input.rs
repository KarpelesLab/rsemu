//! The input seam: a person's keys and pointer, delivered at a virtual instant
//! (`ROADMAP.md` §8, §4.5).
//!
//! A frontend learns about a keystroke whenever the human makes one, which is
//! wall-clock time and therefore not something a machine may observe. So
//! nothing here reads a clock, nothing here holds a socket, and an event
//! becomes part of the guest's history only when a *caller* hands it to an
//! [`InputSink`] at a virtual instant it chose. That instant is what
//! [`InputLog`] records, and replaying the log at the same instants reproduces
//! the run — which is the whole of `CLAUDE.md`'s "any non-deterministic input
//! crossing into the machine goes through the record/replay seam".
//!
//! | Type | Role |
//! | --- | --- |
//! | [`Keysym`] | one key, named the way RFB names it: an X11 keysym |
//! | [`InputEvent`] | what happened: a key went down, a pointer moved |
//! | [`TimedInput`] | that event, plus the virtual instant it was delivered at |
//! | [`InputLog`] | the recording, and its byte format |
//! | [`Replay`] | that recording, played back |
//! | [`InputSink`] | where an event lands: a keyboard port, a pad, a tablet |
//! | [`KeyMap`] | keysym → AT scan codes, with the shift state that implies |
//!
//! # Shape
//!
//! ```text
//!   host                     seam (here)                    device (dev/)
//!   ────                     ───────────                    ─────────────
//!   VNC socket ─► InputEvent ─┬─► InputLog  (at = machine.now())
//!                             └─► KeyboardSink ─► CharPort ─► pc.kbc ─► guest
//!                                 PadSink      ─► nes::Pad ─► nes.io  ─► guest
//! ```
//!
//! The frontend collects events from its socket whenever they arrive, and
//! delivers the batch it has collected at the top of a slice — a scheduling
//! boundary chosen by the scheduler rather than by the network. Two runs given
//! the same `(instant, event)` sequence therefore compute the same thing,
//! whatever the host was doing at the time.
//!
//! # Why level for a pad and edge for a keyboard
//!
//! Because that is what the two pieces of hardware are. A NES controller is a
//! shift register the console samples: the host sets a bit and the guest reads
//! whatever is set at the moment it strobes, so [`PadSink`] keeps a held mask.
//! An AT keyboard is a serial link that sends a make code and, later, a break
//! code: the *transitions* are the data, so [`KeyboardSink`] emits bytes. A
//! seam that flattened both into one would be wrong about one of them.
//!
//! # `no_std`
//!
//! All of it. The sinks reach devices through seams that are themselves
//! `no_std` ([`CharDevice`](super::chardev::CharDevice), `dev::nes::input::Pad`),
//! so a wasm embedder and a deterministic test get the same path a VNC server
//! gets. Only the thing *producing* the events needs an operating system.

use alloc::boxed::Box;
use alloc::string::String;
use alloc::sync::Arc;
use alloc::vec::Vec;
use core::fmt;

use crate::core::clock::GlobalTime;

use super::chardev::CharPort;

// ---------------------------------------------------------------------------
// keys
// ---------------------------------------------------------------------------

/// One key, as an X11 keysym.
///
/// RFB names keys this way (RFC 6143 §7.5.4), X11 does, Wayland does, and a
/// browser's `KeyboardEvent` maps onto it with a table everyone already has.
/// So it is the seam's currency rather than any one guest's scan code: the
/// translation to *this machine's* keyboard is [`KeyMap`]'s job, and a machine
/// with a Sinclair membrane or a Famicom pad needs a different one.
///
/// An extensible enumeration in the `pktkit` style (`CLAUDE.md`): the space is
/// defined by somebody else and has thousands of members, so a Rust `enum`
/// would be a lie about exhaustiveness. Latin-1 keysyms are their own character
/// codes, which is why there is no constant for `A`.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
#[repr(transparent)]
pub struct Keysym(pub u32);

impl Keysym {
    /// `BackSpace`.
    pub const BACKSPACE: Keysym = Keysym(0xff08);
    /// `Tab`.
    pub const TAB: Keysym = Keysym(0xff09);
    /// `Return`.
    pub const RETURN: Keysym = Keysym(0xff0d);
    /// `Escape`.
    pub const ESCAPE: Keysym = Keysym(0xff1b);
    /// `Home`.
    pub const HOME: Keysym = Keysym(0xff50);
    /// `Left`.
    pub const LEFT: Keysym = Keysym(0xff51);
    /// `Up`.
    pub const UP: Keysym = Keysym(0xff52);
    /// `Right`.
    pub const RIGHT: Keysym = Keysym(0xff53);
    /// `Down`.
    pub const DOWN: Keysym = Keysym(0xff54);
    /// `Page_Up`.
    pub const PAGE_UP: Keysym = Keysym(0xff55);
    /// `Page_Down`.
    pub const PAGE_DOWN: Keysym = Keysym(0xff56);
    /// `End`.
    pub const END: Keysym = Keysym(0xff57);
    /// `Insert`.
    pub const INSERT: Keysym = Keysym(0xff63);
    /// `F1`. `F2` … `F12` follow it consecutively.
    pub const F1: Keysym = Keysym(0xffbe);
    /// `F12`.
    pub const F12: Keysym = Keysym(0xffc9);
    /// `Shift_L`.
    pub const SHIFT_L: Keysym = Keysym(0xffe1);
    /// `Shift_R`.
    pub const SHIFT_R: Keysym = Keysym(0xffe2);
    /// `Control_L`.
    pub const CONTROL_L: Keysym = Keysym(0xffe3);
    /// `Control_R`.
    pub const CONTROL_R: Keysym = Keysym(0xffe4);
    /// `Caps_Lock`.
    pub const CAPS_LOCK: Keysym = Keysym(0xffe5);
    /// `Alt_L`.
    pub const ALT_L: Keysym = Keysym(0xffe9);
    /// `Alt_R`.
    pub const ALT_R: Keysym = Keysym(0xffea);
    /// `Delete`.
    pub const DELETE: Keysym = Keysym(0xffff);

    /// The keysym for an ASCII character, which is the character itself.
    #[inline]
    #[must_use]
    pub const fn from_ascii(ch: u8) -> Keysym {
        Keysym(ch as u32)
    }

    /// The printable ASCII character this keysym is, if it is one.
    #[inline]
    #[must_use]
    pub const fn ascii(self) -> Option<u8> {
        if self.0 >= 0x20 && self.0 < 0x7f {
            #[allow(clippy::cast_possible_truncation)]
            Some(self.0 as u8)
        } else {
            None
        }
    }

    /// Whether this is one of the two shift keys.
    #[inline]
    #[must_use]
    pub const fn is_shift(self) -> bool {
        self.0 == Keysym::SHIFT_L.0 || self.0 == Keysym::SHIFT_R.0
    }
}

impl fmt::Display for Keysym {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self.ascii() {
            Some(ch) => write!(f, "{}", ch as char),
            None => write!(f, "0x{:04x}", self.0),
        }
    }
}

// ---------------------------------------------------------------------------
// events
// ---------------------------------------------------------------------------

/// Something a person did.
///
/// `#[non_exhaustive]` because gamepads, absolute tablets and touch are §8's
/// list and none of them is here yet; a `match` that handles what exists today
/// must keep compiling when they arrive.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[non_exhaustive]
pub enum InputEvent {
    /// A key went down (`down`) or came up.
    Key {
        /// Which key.
        keysym: Keysym,
        /// Down, rather than up.
        down: bool,
    },
    /// The pointer is at `(x, y)` with `buttons` held.
    ///
    /// Absolute, because that is what RFB sends (RFC 6143 §7.5.5) and what a
    /// tablet or a touchscreen is; a relative mouse is the difference between
    /// two of these, computed by whoever needs one. `buttons` is bit 0 for the
    /// left button, bit 1 middle, bit 2 right, then the wheel — the RFB button
    /// mask verbatim.
    Pointer {
        /// Pixels from the left of the framebuffer.
        x: u32,
        /// Pixels from the top of the framebuffer.
        y: u32,
        /// Which buttons are held.
        buttons: u8,
    },
}

/// How many bytes one event occupies in an [`InputLog`].
pub const EVENT_BYTES: usize = 12;

impl InputEvent {
    /// Kind tag for [`InputEvent::Key`], as it appears in a log.
    const KIND_KEY: u8 = 1;
    /// Kind tag for [`InputEvent::Pointer`].
    const KIND_POINTER: u8 = 2;

    /// The fixed-width encoding a log stores.
    ///
    /// Fixed width on purpose: a log is then seekable and a record's index is
    /// its offset, which is what rewind wants (§4.5). Little-endian, like every
    /// other byte format in the tree.
    #[must_use]
    pub const fn encode(self) -> [u8; EVENT_BYTES] {
        let (kind, flags, a, b) = match self {
            InputEvent::Key { keysym, down } => (InputEvent::KIND_KEY, down as u8, keysym.0, 0u32),
            InputEvent::Pointer { x, y, buttons } => (InputEvent::KIND_POINTER, buttons, x, y),
        };
        let a = a.to_le_bytes();
        let b = b.to_le_bytes();
        [
            kind, flags, 0, 0, a[0], a[1], a[2], a[3], b[0], b[1], b[2], b[3],
        ]
    }

    /// Read back what [`encode`](InputEvent::encode) wrote.
    ///
    /// `None` for a kind this build does not know, which means a log written by
    /// a newer rsemu rather than a corrupt one — the caller decides whether
    /// that is fatal.
    #[must_use]
    pub const fn decode(bytes: &[u8; EVENT_BYTES]) -> Option<InputEvent> {
        let a = u32::from_le_bytes([bytes[4], bytes[5], bytes[6], bytes[7]]);
        let b = u32::from_le_bytes([bytes[8], bytes[9], bytes[10], bytes[11]]);
        match bytes[0] {
            InputEvent::KIND_KEY => Some(InputEvent::Key {
                keysym: Keysym(a),
                down: bytes[1] != 0,
            }),
            InputEvent::KIND_POINTER => Some(InputEvent::Pointer {
                x: a,
                y: b,
                buttons: bytes[1],
            }),
            _ => None,
        }
    }
}

/// An event and the virtual instant it was delivered at.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct TimedInput {
    /// When, in virtual time. **Not** when the human pressed the key: when the
    /// machine was told.
    pub at: GlobalTime,
    /// What.
    pub event: InputEvent,
}

// ---------------------------------------------------------------------------
// the log
// ---------------------------------------------------------------------------

/// The eight-byte magic a log starts with.
const LOG_MAGIC: [u8; 8] = *b"RSEMUIN\x00";

/// The log format version. Bump it with the encoding, never on its own.
const LOG_VERSION: u32 = 1;

/// How many bytes a log's header occupies: magic, version, record count.
const LOG_HEADER: usize = 16;

/// How many bytes one record occupies: the instant, then the event.
///
/// The instant is [`GlobalTime`]'s **raw** 2⁻⁶⁴-second count and not a
/// nanosecond figure. That matters: `GlobalTime::as_nanos` rounds, and an event
/// replayed a fraction of a nanosecond away from where it was recorded lands on
/// a different tick of a fast clock domain — which is a divergent run, found
/// six hours later as a state-hash mismatch.
const RECORD_BYTES: usize = 16 + EVENT_BYTES;

/// A recording of every input event that crossed into a machine.
///
/// Append-only and sorted by [`TimedInput::at`], because that is the order it
/// is written in and the order it is replayed in; events at the same instant
/// keep their insertion order, which is the tie-break the scheduler's own
/// sequence counter uses (§4.2).
///
/// This is deliberately a *self-contained* file rather than a chunk of a
/// machine snapshot: it is the piece a bug report attaches, and it has to be
/// readable by a build that cannot reconstruct the machine. When the general
/// record/replay seam lands, this becomes one source's stream inside it — the
/// [`vnc`](super::vnc) module docs say exactly what that seam has to offer for
/// this one to be deleted.
#[derive(Debug, Clone, Default, PartialEq, Eq)]
pub struct InputLog {
    events: Vec<TimedInput>,
}

impl InputLog {
    /// An empty log.
    #[must_use]
    pub const fn new() -> InputLog {
        InputLog { events: Vec::new() }
    }

    /// Append an event delivered at `at`.
    ///
    /// A caller that goes backwards in time is recording something that could
    /// not have happened; the log keeps what it was given rather than sorting,
    /// so the mistake shows up in [`is_ordered`](InputLog::is_ordered) instead
    /// of being hidden.
    pub fn push(&mut self, at: GlobalTime, event: InputEvent) {
        self.events.push(TimedInput { at, event });
    }

    /// Everything recorded, in order.
    #[must_use]
    pub fn events(&self) -> &[TimedInput] {
        &self.events
    }

    /// How many events are recorded.
    #[must_use]
    pub fn len(&self) -> usize {
        self.events.len()
    }

    /// Whether nothing was recorded.
    #[must_use]
    pub fn is_empty(&self) -> bool {
        self.events.is_empty()
    }

    /// Whether the instants never go backwards.
    #[must_use]
    pub fn is_ordered(&self) -> bool {
        self.events.windows(2).all(|w| w[0].at <= w[1].at)
    }

    /// The whole log as bytes.
    #[must_use]
    pub fn encode(&self) -> Vec<u8> {
        let mut out = Vec::with_capacity(LOG_HEADER + self.events.len() * RECORD_BYTES);
        out.extend_from_slice(&LOG_MAGIC);
        out.extend_from_slice(&LOG_VERSION.to_le_bytes());
        #[allow(clippy::cast_possible_truncation)]
        out.extend_from_slice(&(self.events.len() as u32).to_le_bytes());
        for entry in &self.events {
            out.extend_from_slice(&entry.at.raw().to_le_bytes());
            out.extend_from_slice(&entry.event.encode());
        }
        out
    }

    /// Read back what [`encode`](InputLog::encode) wrote.
    ///
    /// # Errors
    ///
    /// A short file, a wrong magic, a version this build does not know, or a
    /// record whose kind it does not know. All four are the same kind of
    /// mistake — this is not the file you think it is — and all four say so.
    pub fn decode(bytes: &[u8]) -> crate::core::Result<InputLog> {
        let bad = |message: &str| crate::Error::Config {
            at: String::from("input log"),
            message: String::from(message),
        };
        if bytes.len() < LOG_HEADER || bytes[..8] != LOG_MAGIC {
            return Err(bad("not an rsemu input log"));
        }
        let version = u32::from_le_bytes([bytes[8], bytes[9], bytes[10], bytes[11]]);
        if version != LOG_VERSION {
            return Err(bad("input log version is not one this build can read"));
        }
        let count = u32::from_le_bytes([bytes[12], bytes[13], bytes[14], bytes[15]]) as usize;
        let body = &bytes[LOG_HEADER..];
        if body.len() < count * RECORD_BYTES {
            return Err(bad("input log is shorter than its own record count"));
        }
        let mut log = InputLog::new();
        log.events.reserve(count);
        for i in 0..count {
            let record = &body[i * RECORD_BYTES..];
            let mut when = [0u8; 16];
            when.copy_from_slice(&record[..16]);
            let mut raw = [0u8; EVENT_BYTES];
            raw.copy_from_slice(&record[16..RECORD_BYTES]);
            let event = InputEvent::decode(&raw)
                .ok_or_else(|| bad("input log holds an event kind this build does not know"))?;
            log.events.push(TimedInput {
                at: GlobalTime::from_raw(u128::from_le_bytes(when)),
                event,
            });
        }
        Ok(log)
    }
}

/// A cursor over an [`InputLog`], for replaying one.
///
/// The replay counterpart of a socket: a frontend asks it, at the top of each
/// slice, for everything due by the instant the slice starts at, and delivers
/// exactly that. Because both the recording and the replay deliver at a slice
/// boundary, and because `Machine::run_until` is additive across boundaries
/// (§11.6), the two runs are the same run.
#[derive(Debug, Clone)]
pub struct Replay {
    log: InputLog,
    next: usize,
}

impl Replay {
    /// Start at the beginning of `log`.
    #[must_use]
    pub const fn new(log: InputLog) -> Replay {
        Replay { log, next: 0 }
    }

    /// Every event due at or before `at`, consumed.
    pub fn due(&mut self, at: GlobalTime) -> &[TimedInput] {
        let start = self.next;
        while self
            .log
            .events
            .get(self.next)
            .is_some_and(|entry| entry.at <= at)
        {
            self.next += 1;
        }
        &self.log.events[start..self.next]
    }

    /// Whether every event has been replayed.
    #[must_use]
    pub fn is_finished(&self) -> bool {
        self.next >= self.log.events.len()
    }

    /// How many events have not been replayed yet.
    #[must_use]
    pub fn remaining(&self) -> usize {
        self.log.events.len() - self.next
    }

    /// The instant of the next event, if there is one.
    ///
    /// What a replay-driven run uses to decide how far it may advance before it
    /// has to stop and deliver: running past an event's instant and delivering
    /// late would be a different run.
    #[must_use]
    pub fn next_instant(&self) -> Option<GlobalTime> {
        self.log.events.get(self.next).map(|entry| entry.at)
    }
}

// ---------------------------------------------------------------------------
// sinks
// ---------------------------------------------------------------------------

/// Where an event lands.
///
/// `&self`, and `Send + Sync`, like every device-facing trait in the tree: the
/// implementations hold the same handles a device holds, and a frontend may sit
/// on a different thread from the machine as long as it only delivers at a
/// slice boundary.
pub trait InputSink: Send + Sync + fmt::Debug {
    /// Deliver one event. An event this sink has no use for is dropped
    /// silently — a NES pad has nothing to do with a pointer.
    fn deliver(&self, event: InputEvent);
}

/// Deliver `event` to every sink in `sinks`.
///
/// A machine may have a keyboard *and* a pad, and a frontend should not have to
/// know which sink wants which event — each one decides for itself.
pub fn deliver_all(sinks: &[Box<dyn InputSink>], event: InputEvent) {
    for sink in sinks {
        sink.deliver(event);
    }
}

// ---------------------------------------------------------------------------
// the AT keyboard
// ---------------------------------------------------------------------------

/// A key's position on an AT keyboard, in **scan code set 2**.
///
/// Set 2 rather than set 1 because set 2 is what the keyboard itself sends: an
/// 8042 receives set 2 on the wire and translates to set 1 on the way to port
/// 0x60 only when its command byte says to
/// ([`TRANSLATE`](crate::dev::pc::kbc::TRANSLATE)). Feeding the port set 1 would
/// be modelling a keyboard that does not exist, and would break the moment a
/// guest turned translation off — which OS/2 and several DOS extenders do.
///
/// Source: the IBM PS/2 Technical Reference's keyboard scan code tables, as
/// reproduced on the OSDev wiki — the same documentation `dev::pc::kbc` cites
/// for its translation table, and cross-checked against it by a test below.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ScanCode {
    /// The set-2 code.
    pub code: u8,
    /// Whether the code is prefixed with `0xE0`: the keys added for the
    /// 101-key layout, which share a code with an original key.
    pub extended: bool,
    /// Whether reaching this character needs Shift held.
    pub shifted: bool,
}

impl ScanCode {
    /// A plain key.
    const fn plain(code: u8) -> ScanCode {
        ScanCode {
            code,
            extended: false,
            shifted: false,
        }
    }

    /// A key reached with Shift.
    const fn shift(code: u8) -> ScanCode {
        ScanCode {
            code,
            extended: false,
            shifted: true,
        }
    }

    /// One of the 101-key additions, prefixed `0xE0`.
    const fn ext(code: u8) -> ScanCode {
        ScanCode {
            code,
            extended: true,
            shifted: false,
        }
    }
}

/// The set-2 make code for the left Shift key.
const SET2_LSHIFT: u8 = 0x12;
/// The set-2 break prefix: the code after it is a key coming up.
const SET2_BREAK: u8 = 0xf0;
/// The set-2 extended prefix.
const SET2_EXTEND: u8 = 0xe0;

/// The set-2 code for one letter key, named by its lower-case character.
const fn letter(ch: u8) -> u8 {
    match ch {
        b'a' => 0x1c,
        b'b' => 0x32,
        b'c' => 0x21,
        b'd' => 0x23,
        b'e' => 0x24,
        b'f' => 0x2b,
        b'g' => 0x34,
        b'h' => 0x33,
        b'i' => 0x43,
        b'j' => 0x3b,
        b'k' => 0x42,
        b'l' => 0x4b,
        b'm' => 0x3a,
        b'n' => 0x31,
        b'o' => 0x44,
        b'p' => 0x4d,
        b'q' => 0x15,
        b'r' => 0x2d,
        b's' => 0x1b,
        b't' => 0x2c,
        b'u' => 0x3c,
        b'v' => 0x2a,
        b'w' => 0x1d,
        b'x' => 0x22,
        b'y' => 0x35,
        // The only letter left. A wildcard rather than a `b'z'` arm plus an
        // unreachable one, because this is a `const fn` and the caller has
        // already bounded the range.
        _ => 0x1a,
    }
}

/// Where a keysym is on an AT keyboard, or `None` for one that is not there.
///
/// Deliberately a `match` rather than a table: the mapping is sparse over a
/// 32-bit space, and a reader can check any single line of it against the
/// manual without counting array indices.
#[must_use]
#[allow(clippy::too_many_lines)]
pub const fn set2(keysym: Keysym) -> Option<ScanCode> {
    match keysym.0 {
        // -- the alphabet ---------------------------------------------------
        #[allow(clippy::cast_possible_truncation)]
        c @ 0x61..=0x7a => Some(ScanCode::plain(letter(c as u8))),
        #[allow(clippy::cast_possible_truncation)]
        c @ 0x41..=0x5a => Some(ScanCode::shift(letter(c as u8 + 0x20))),
        // -- the number row, unshifted then shifted -------------------------
        0x31 => Some(ScanCode::plain(0x16)),
        0x32 => Some(ScanCode::plain(0x1e)),
        0x33 => Some(ScanCode::plain(0x26)),
        0x34 => Some(ScanCode::plain(0x25)),
        0x35 => Some(ScanCode::plain(0x2e)),
        0x36 => Some(ScanCode::plain(0x36)),
        0x37 => Some(ScanCode::plain(0x3d)),
        0x38 => Some(ScanCode::plain(0x3e)),
        0x39 => Some(ScanCode::plain(0x46)),
        0x30 => Some(ScanCode::plain(0x45)),
        0x21 => Some(ScanCode::shift(0x16)), // !
        0x40 => Some(ScanCode::shift(0x1e)), // @
        0x23 => Some(ScanCode::shift(0x26)), // #
        0x24 => Some(ScanCode::shift(0x25)), // $
        0x25 => Some(ScanCode::shift(0x2e)), // %
        0x5e => Some(ScanCode::shift(0x36)), // ^
        0x26 => Some(ScanCode::shift(0x3d)), // &
        0x2a => Some(ScanCode::shift(0x3e)), // *
        0x28 => Some(ScanCode::shift(0x46)), // (
        0x29 => Some(ScanCode::shift(0x45)), // )
        // -- punctuation ----------------------------------------------------
        0x60 => Some(ScanCode::plain(0x0e)), // `
        0x7e => Some(ScanCode::shift(0x0e)), // ~
        0x2d => Some(ScanCode::plain(0x4e)), // -
        0x5f => Some(ScanCode::shift(0x4e)), // _
        0x3d => Some(ScanCode::plain(0x55)), // =
        0x2b => Some(ScanCode::shift(0x55)), // +
        0x5b => Some(ScanCode::plain(0x54)), // [
        0x7b => Some(ScanCode::shift(0x54)), // {
        0x5d => Some(ScanCode::plain(0x5b)), // ]
        0x7d => Some(ScanCode::shift(0x5b)), // }
        0x5c => Some(ScanCode::plain(0x5d)), // \
        0x7c => Some(ScanCode::shift(0x5d)), // |
        0x3b => Some(ScanCode::plain(0x4c)), // ;
        0x3a => Some(ScanCode::shift(0x4c)), // :
        0x27 => Some(ScanCode::plain(0x52)), // '
        0x22 => Some(ScanCode::shift(0x52)), // "
        0x2c => Some(ScanCode::plain(0x41)), // ,
        0x3c => Some(ScanCode::shift(0x41)), // <
        0x2e => Some(ScanCode::plain(0x49)), // .
        0x3e => Some(ScanCode::shift(0x49)), // >
        0x2f => Some(ScanCode::plain(0x4a)), // /
        0x3f => Some(ScanCode::shift(0x4a)), // ?
        0x20 => Some(ScanCode::plain(0x29)), // space
        // -- the keys with names --------------------------------------------
        0xff08 => Some(ScanCode::plain(0x66)), // BackSpace
        0xff09 => Some(ScanCode::plain(0x0d)), // Tab
        0xff0d => Some(ScanCode::plain(0x5a)), // Return
        0xff1b => Some(ScanCode::plain(0x76)), // Escape
        0xffe1 => Some(ScanCode::plain(0x12)), // Shift_L
        0xffe2 => Some(ScanCode::plain(0x59)), // Shift_R
        0xffe3 => Some(ScanCode::plain(0x14)), // Control_L
        0xffe4 => Some(ScanCode::ext(0x14)),   // Control_R
        0xffe5 => Some(ScanCode::plain(0x58)), // Caps_Lock
        0xffe9 => Some(ScanCode::plain(0x11)), // Alt_L
        0xffea => Some(ScanCode::ext(0x11)),   // Alt_R
        // -- the 101-key additions, all E0-prefixed -------------------------
        0xff50 => Some(ScanCode::ext(0x6c)), // Home
        0xff51 => Some(ScanCode::ext(0x6b)), // Left
        0xff52 => Some(ScanCode::ext(0x75)), // Up
        0xff53 => Some(ScanCode::ext(0x74)), // Right
        0xff54 => Some(ScanCode::ext(0x72)), // Down
        0xff55 => Some(ScanCode::ext(0x7d)), // Page_Up
        0xff56 => Some(ScanCode::ext(0x7a)), // Page_Down
        0xff57 => Some(ScanCode::ext(0x69)), // End
        0xff63 => Some(ScanCode::ext(0x70)), // Insert
        0xffff => Some(ScanCode::ext(0x71)), // Delete
        // -- the function row -----------------------------------------------
        0xffbe => Some(ScanCode::plain(0x05)), // F1
        0xffbf => Some(ScanCode::plain(0x06)), // F2
        0xffc0 => Some(ScanCode::plain(0x04)), // F3
        0xffc1 => Some(ScanCode::plain(0x0c)), // F4
        0xffc2 => Some(ScanCode::plain(0x03)), // F5
        0xffc3 => Some(ScanCode::plain(0x0b)), // F6
        0xffc4 => Some(ScanCode::plain(0x83)), // F7
        0xffc5 => Some(ScanCode::plain(0x0a)), // F8
        0xffc6 => Some(ScanCode::plain(0x01)), // F9
        0xffc7 => Some(ScanCode::plain(0x09)), // F10
        0xffc8 => Some(ScanCode::plain(0x78)), // F11
        0xffc9 => Some(ScanCode::plain(0x07)), // F12
        _ => None,
    }
}

/// Turns keysyms into the bytes an AT keyboard would have put on the wire.
///
/// Stateful, because Shift is: a client that sends the keysym `!` without ever
/// sending `Shift_L` — and several do, because on the client's own keyboard the
/// user really did hold shift and the client reports only the resulting
/// character — has to have the shift synthesised, and a client that *does* send
/// `Shift_L` must not have a second one synthesised on top. So the map tracks
/// whether the client is holding shift and only invents one when it is not.
#[derive(Debug, Clone, Default)]
pub struct KeyMap {
    /// Whether the client has told us a shift key is down.
    shift_held: bool,
}

impl KeyMap {
    /// A map with nothing held.
    #[must_use]
    pub const fn new() -> KeyMap {
        KeyMap { shift_held: false }
    }

    /// Whether a shift key is held, as far as the client has said.
    #[must_use]
    pub const fn shift_held(&self) -> bool {
        self.shift_held
    }

    /// Forget every held key: what a client disconnecting means.
    pub fn reset(&mut self) {
        self.shift_held = false;
    }

    /// The set-2 bytes for one key transition, appended to `out`.
    ///
    /// Returns whether the key was one this keyboard has. A key it does not
    /// have produces no bytes at all rather than a guess — a guest given a scan
    /// code for a key nobody pressed is worse off than one given nothing.
    pub fn encode(&mut self, keysym: Keysym, down: bool, out: &mut Vec<u8>) -> bool {
        let Some(sc) = set2(keysym) else {
            return false;
        };
        if keysym.is_shift() {
            self.shift_held = down;
        }
        // A character that needs shift, from a client that has not said it is
        // holding one: wrap the key in a make/break pair of its own, so the
        // guest sees exactly the sequence a person's hands would have produced.
        let synth = sc.shifted && !self.shift_held;
        if down && synth {
            out.push(SET2_LSHIFT);
        }
        // Order on the wire: E0 first, then F0 for a break, then the code —
        // an extended key coming up is `E0 F0 xx`.
        if sc.extended {
            out.push(SET2_EXTEND);
        }
        if !down {
            out.push(SET2_BREAK);
        }
        out.push(sc.code);
        if !down && synth {
            out.push(SET2_BREAK);
            out.push(SET2_LSHIFT);
        }
        true
    }
}

/// An [`InputSink`] that types at a character port.
///
/// The far end is whatever the machine description bound to it — on a PC that
/// is `pc.kbc`'s `port` property, and the 8042 reads set-2 scan codes off it
/// exactly as it would off the keyboard's clock and data lines. Back pressure
/// is the port's: a guest that has disabled scanning leaves the bytes queued
/// rather than losing them.
///
/// A [`CharPort`] rather than a [`CharDevice`](super::chardev::CharDevice),
/// because this is the *host* end of the seam and the trait's directions are
/// named from the device's: `CharDevice::write` is what the guest says, and
/// [`CharPort::feed`] is what the host says. Holding the trait object would
/// have typed at the wrong end of the wire.
pub struct KeyboardSink {
    port: Arc<CharPort>,
    map: crate::core::sync::Mutex<KeyMap>,
}

impl fmt::Debug for KeyboardSink {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("KeyboardSink")
            .field("port", &self.port)
            .finish_non_exhaustive()
    }
}

impl KeyboardSink {
    /// Type at `port`.
    #[must_use]
    pub fn new(port: Arc<CharPort>) -> KeyboardSink {
        KeyboardSink {
            port,
            map: crate::core::sync::Mutex::new(KeyMap::new()),
        }
    }
}

impl InputSink for KeyboardSink {
    fn deliver(&self, event: InputEvent) {
        let InputEvent::Key { keysym, down } = event else {
            return;
        };
        let mut bytes = Vec::new();
        // The lock covers the scan-code translation and nothing else: writing
        // to the port is an outward call, and it happens after the guard is
        // gone (`CLAUDE.md`, the re-entrancy contract).
        {
            let mut map = self.map.lock();
            if !map.encode(keysym, down, &mut bytes) {
                return;
            }
        }
        // `feed` and not `write`: the host produces scan codes for the guest
        // to read, which is the seam's host-to-guest direction.
        self.port.feed(&bytes);
    }
}

// ---------------------------------------------------------------------------
// the NES pad
// ---------------------------------------------------------------------------

/// Which NES button a keysym is, for [`PadSink`].
///
/// The arrow keys, `Z` and `X` for B and A, Return for Start and either Shift
/// for Select — the layout every NES emulator has used since the 1990s, which
/// makes it the one a person will try first. It is a *host* convention rather
/// than a hardware fact, which is why it lives here and not in `dev/nes`.
#[cfg(feature = "dev-nes-io")]
#[cfg_attr(docsrs, doc(cfg(feature = "dev-nes-io")))]
#[must_use]
pub const fn nes_button(keysym: Keysym) -> Option<u8> {
    use crate::dev::nes::input::buttons;
    match keysym.0 {
        0xff52 => Some(buttons::UP),
        0xff54 => Some(buttons::DOWN),
        0xff51 => Some(buttons::LEFT),
        0xff53 => Some(buttons::RIGHT),
        0x7a | 0x5a => Some(buttons::B),          // z, Z
        0x78 | 0x58 => Some(buttons::A),          // x, X
        0xff0d => Some(buttons::START),           // Return
        0xffe1 | 0xffe2 => Some(buttons::SELECT), // either Shift
        _ => None,
    }
}

/// An [`InputSink`] that holds buttons down on a NES controller.
///
/// Level rather than edge: the pad is a shift register the console samples, so
/// what the sink keeps is the held mask, and the guest reads whatever is set
/// when it strobes.
#[cfg(feature = "dev-nes-io")]
#[cfg_attr(docsrs, doc(cfg(feature = "dev-nes-io")))]
#[derive(Debug)]
pub struct PadSink {
    pad: Arc<crate::dev::nes::input::Pad>,
    port: usize,
    held: core::sync::atomic::AtomicU8,
}

#[cfg(feature = "dev-nes-io")]
impl PadSink {
    /// Drive controller `port` (0 or 1) of `pad`.
    #[must_use]
    pub fn new(pad: Arc<crate::dev::nes::input::Pad>, port: usize) -> PadSink {
        PadSink {
            pad,
            port,
            held: core::sync::atomic::AtomicU8::new(0),
        }
    }
}

#[cfg(feature = "dev-nes-io")]
impl InputSink for PadSink {
    fn deliver(&self, event: InputEvent) {
        use core::sync::atomic::Ordering;
        let InputEvent::Key { keysym, down } = event else {
            return;
        };
        let Some(bit) = nes_button(keysym) else {
            return;
        };
        let previous = self.held.load(Ordering::Relaxed);
        let next = if down {
            previous | bit
        } else {
            previous & !bit
        };
        self.held.store(next, Ordering::Relaxed);
        self.pad.set(self.port, next);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn an_event_survives_its_own_encoding() {
        for event in [
            InputEvent::Key {
                keysym: Keysym::from_ascii(b'a'),
                down: true,
            },
            InputEvent::Key {
                keysym: Keysym::DELETE,
                down: false,
            },
            InputEvent::Pointer {
                x: 719,
                y: 399,
                buttons: 0b101,
            },
        ] {
            let bytes = event.encode();
            assert_eq!(InputEvent::decode(&bytes), Some(event), "{event:?}");
        }
    }

    #[test]
    fn an_unknown_event_kind_is_none_not_a_panic() {
        let mut bytes = [0u8; EVENT_BYTES];
        bytes[0] = 0xfe;
        assert_eq!(InputEvent::decode(&bytes), None);
    }

    #[test]
    fn a_log_survives_its_own_encoding() {
        let mut log = InputLog::new();
        log.push(
            GlobalTime::from_nanos(0),
            InputEvent::Key {
                keysym: Keysym::from_ascii(b'A'),
                down: true,
            },
        );
        log.push(
            GlobalTime::from_nanos(16_000_000),
            InputEvent::Pointer {
                x: 3,
                y: 4,
                buttons: 1,
            },
        );
        let bytes = log.encode();
        let back = InputLog::decode(&bytes).expect("our own bytes");
        assert_eq!(back, log);
        assert!(back.is_ordered());
    }

    #[test]
    fn a_log_that_is_not_one_is_an_error() {
        assert!(InputLog::decode(b"").is_err());
        assert!(InputLog::decode(b"not a log at all").is_err());
        let mut bytes = InputLog::new().encode();
        bytes[8] = 0xff;
        assert!(
            InputLog::decode(&bytes).is_err(),
            "a version we cannot read"
        );
    }

    #[test]
    fn a_replay_hands_back_what_is_due_and_no_more() {
        let mut log = InputLog::new();
        for (i, ch) in (*b"abc").into_iter().enumerate() {
            log.push(
                GlobalTime::from_nanos(i as u64 * 10),
                InputEvent::Key {
                    keysym: Keysym::from_ascii(ch),
                    down: true,
                },
            );
        }
        let mut replay = Replay::new(log);
        assert_eq!(replay.next_instant(), Some(GlobalTime::from_nanos(0)));
        assert_eq!(replay.due(GlobalTime::from_nanos(5)).len(), 1);
        assert_eq!(replay.due(GlobalTime::from_nanos(5)).len(), 0);
        assert_eq!(replay.due(GlobalTime::from_nanos(25)).len(), 2);
        assert!(replay.is_finished());
        assert_eq!(replay.remaining(), 0);
    }

    /// The set-2 table here and `dev::pc::kbc`'s set-2-to-set-1 translation
    /// table come from the same manual. If they disagree, one of them is wrong
    /// — and this is the cheapest possible way to find out.
    #[cfg(feature = "dev-pc")]
    #[test]
    fn the_scan_codes_agree_with_the_controllers_translation_table() {
        use crate::dev::pc::kbc::TRANSLATE;
        // (keysym, the set-1 code the IBM technical reference prints)
        let expected: &[(Keysym, u8)] = &[
            (Keysym::from_ascii(b'a'), 0x1e),
            (Keysym::from_ascii(b'z'), 0x2c),
            (Keysym::from_ascii(b'1'), 0x02),
            (Keysym::from_ascii(b' '), 0x39),
            (Keysym::RETURN, 0x1c),
            (Keysym::ESCAPE, 0x01),
            (Keysym::BACKSPACE, 0x0e),
            (Keysym::TAB, 0x0f),
            (Keysym::SHIFT_L, 0x2a),
            (Keysym::CONTROL_L, 0x1d),
            (Keysym::F1, 0x3b),
            (Keysym::F12, 0x58),
            (Keysym::UP, 0x48),
            (Keysym::LEFT, 0x4b),
        ];
        for (keysym, set1) in expected {
            let sc = set2(*keysym).unwrap_or_else(|| panic!("{keysym} is on an AT keyboard"));
            assert_eq!(
                TRANSLATE[sc.code as usize], *set1,
                "{keysym}: set-2 {:#04x} should translate to set-1 {set1:#04x}",
                sc.code
            );
        }
    }

    #[test]
    fn a_shifted_character_gets_a_shift_the_client_did_not_send() {
        let mut map = KeyMap::new();
        let mut out = Vec::new();
        map.encode(Keysym::from_ascii(b'A'), true, &mut out);
        assert_eq!(out, [SET2_LSHIFT, 0x1c], "shift make, then A make");
        out.clear();
        map.encode(Keysym::from_ascii(b'A'), false, &mut out);
        assert_eq!(
            out,
            [SET2_BREAK, 0x1c, SET2_BREAK, SET2_LSHIFT],
            "A break, then shift break"
        );
    }

    #[test]
    fn a_client_holding_shift_does_not_get_a_second_one() {
        let mut map = KeyMap::new();
        let mut out = Vec::new();
        map.encode(Keysym::SHIFT_L, true, &mut out);
        assert_eq!(out, [SET2_LSHIFT]);
        assert!(map.shift_held());
        out.clear();
        map.encode(Keysym::from_ascii(b'A'), true, &mut out);
        assert_eq!(out, [0x1c], "no synthesised shift on top of a held one");
        map.reset();
        assert!(!map.shift_held());
    }

    #[test]
    fn an_extended_key_carries_its_prefix_both_ways() {
        let mut map = KeyMap::new();
        let mut out = Vec::new();
        map.encode(Keysym::UP, true, &mut out);
        assert_eq!(out, [SET2_EXTEND, 0x75]);
        out.clear();
        map.encode(Keysym::UP, false, &mut out);
        assert_eq!(out, [SET2_EXTEND, SET2_BREAK, 0x75]);
    }

    #[test]
    fn a_key_this_keyboard_does_not_have_produces_nothing() {
        let mut map = KeyMap::new();
        let mut out = Vec::new();
        assert!(
            !map.encode(Keysym(0xfe03), true, &mut out),
            "ISO_Level3_Shift is not on a 101-key board"
        );
        assert!(out.is_empty());
    }

    #[test]
    fn typing_at_a_port_puts_scan_codes_in_it() {
        let port = Arc::new(CharPort::new());
        let sink = KeyboardSink::new(port.clone());
        sink.deliver(InputEvent::Key {
            keysym: Keysym::from_ascii(b'a'),
            down: true,
        });
        sink.deliver(InputEvent::Key {
            keysym: Keysym::from_ascii(b'a'),
            down: false,
        });
        // What the guest would read off the keyboard's data line.
        let mut got = [0u8; 8];
        let n = crate::host::chardev::CharDevice::read(&*port, &mut got);
        assert_eq!(&got[..n], &[0x1c, SET2_BREAK, 0x1c]);
        // A pointer event is not a keystroke and must not become one.
        sink.deliver(InputEvent::Pointer {
            x: 1,
            y: 1,
            buttons: 1,
        });
        assert_eq!(port.pending_input(), 0);
    }

    #[cfg(feature = "dev-nes-io")]
    #[test]
    fn a_pad_holds_what_is_pressed_and_releases_what_is_not() {
        use crate::dev::nes::input::{Pad, buttons};
        let pad = Arc::new(Pad::new());
        let sink = PadSink::new(pad.clone(), 0);
        sink.deliver(InputEvent::Key {
            keysym: Keysym::from_ascii(b'x'),
            down: true,
        });
        sink.deliver(InputEvent::Key {
            keysym: Keysym::LEFT,
            down: true,
        });
        assert_eq!(pad.get(0), buttons::A | buttons::LEFT);
        sink.deliver(InputEvent::Key {
            keysym: Keysym::LEFT,
            down: false,
        });
        assert_eq!(pad.get(0), buttons::A);
        assert_eq!(pad.get(1), buttons::NONE, "the other port is untouched");
    }

    #[test]
    fn every_sink_is_send_and_sync() {
        fn assert_send_sync<T: Send + Sync>() {}
        assert_send_sync::<KeyboardSink>();
        assert_send_sync::<InputLog>();
        #[cfg(feature = "dev-nes-io")]
        assert_send_sync::<PadSink>();
    }
}
