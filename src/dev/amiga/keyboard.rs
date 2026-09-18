//! `amiga.keyboard`: the keyboard at the end of the cable, speaking Appendix G's
//! protocol on `KCLK` and `KDAT`.
//!
//! ```text
//!   osc periph = 1000000 Hz
//!   object kbd "amiga.keyboard" { clock = periph }
//!
//!   wire kbd.kclk  -> cia_a.cnt { pull = "up" }   # KCLK: the keyboard's, always
//!   wire cia_a.sp  -> kbd.kdat  { pull = "up" }   # KDAT: both ends drive it,
//!   wire kbd.kdat  -> cia_a.sp                    #   so it is two statements
//! ```
//!
//! This models the keyboard's documented **protocol**, not the microcontroller
//! inside it: there is no 6500/1 core here and no keyboard firmware image, and
//! none is wanted. What the manual specifies — the bit timing, the encoding,
//! the handshake, resynchronisation and the power-up stream — is behaviour of
//! the interface, and the interface is what the Amiga sees.
//!
//! # Sources
//!
//! *Amiga Hardware Reference Manual*, Commodore-Amiga Inc., 3rd edition:
//!
//! * **Appendix G, "Keyboard Interface"** (pp. 357-364) for everything on the
//!   wire: `KCLK` "unidirectional and always driven by the keyboard", `KDAT`
//!   "driven by both the keyboard and the computer", both open-collector with
//!   pull-ups at each end; the bit timing ("sets the KDAT line about 20
//!   microseconds before it pulls KCLK low. KCLK stays low for about 20
//!   microseconds, then goes high again. The processor waits another 20
//!   microseconds before changing KDAT"); the rotation ("the transmitted order
//!   is therefore 6-5-4-3-2-1-0-7") and the inversion ("a high level (+5V) is
//!   interpreted as 0, and a low level (0V) is interpreted as 1"); the
//!   handshake ("it must pulse KDAT low for at least 1 (one) microsecond");
//!   the out-of-sync rule (143 ms, clock out a 1, repeat, then `$F9` and the
//!   code again); Caps Lock; the power-up sequence (`$FD`, the keys held,
//!   `$FE`, then the LED off); the matrix table the host keymap is written
//!   from; and the special codes.
//! * **Chapter 8, "The Keyboard"** (pp. 251-254) for the other end: SP is the
//!   data line and CNT the clock, "the rising edge of this pulse clocks in the
//!   data", the handshake is "the processor pulsing the SP line low then high",
//!   and "the keyboard microprocessor holds keys in a 10 keycode type-ahead
//!   buffer". Also the raw keycode table, 40-67 hex.
//!
//! No emulator source was consulted — every Amiga emulator is GPL — and no
//! keyboard firmware was read or run (`ROADMAP.md` §1).
//!
//! # Time
//!
//! **One tick of this device's `clock` is one microsecond**, because every
//! number the manual gives is in microseconds or milliseconds and the keyboard
//! has its own crystal rather than a division of the Amiga's. A board gives it
//! a 1 MHz oscillator of its own. A different rate is not refused — a device
//! cannot see its domain's frequency — but every interval below then scales
//! with it.
//!
//! | interval | ticks | manual |
//! | --- | --- | --- |
//! | `KDAT` set before `KCLK` falls | [`DATA_SETUP_TICKS`] | "about 20 microseconds" |
//! | `KCLK` low | [`CLOCK_LOW_TICKS`] | "about 20 microseconds" |
//! | `KCLK` high before `KDAT` changes | [`DATA_HOLD_TICKS`] | "another 20 microseconds" |
//! | handshake timeout | [`HANDSHAKE_TIMEOUT_TICKS`] | "within 143 ms of the last clock" |
//!
//! # What is chosen where the manual is silent
//!
//! * **When the handshake latch is armed.** The manual has the keyboard latch
//!   the pulse in hardware; it does not say from when. This keyboard arms it
//!   at the rising edge of the last `KCLK` of a transmission — the instant the
//!   8520 raises its interrupt — and, because a pulse that starts while the
//!   keyboard is still holding `KDAT` low for a one cannot be seen on the line,
//!   it also samples the line at the moment it lets go.
//! * **When the next byte starts.** Once the pulse has been latched *and has
//!   ended*: the next data bit is put on the line at the instant `KDAT` comes
//!   back up. Starting while the computer still holds the line low would have
//!   the computer's own pulse read as a one.
//! * **Self-test** always passes and takes no time: the first sync bit's data
//!   is on the line at power-on and its clock falls [`DATA_SETUP_TICKS`]
//!   later. `$FC` is never sent.
//! * **The power-up sync interval.** "Slowly clocking out 1 bits, as described
//!   above" — the resync rule's 143 ms, which is what this uses.
//! * **Key repeat.** An Amiga keyboard sends one code per transition and the
//!   operating system repeats; a host that reports a held key as repeated
//!   downs is sending nothing new, so a down for a key already down (and an up
//!   for one already up) is dropped.
//! * **Overflow.** A movement that arrives with ten codes already waiting is
//!   lost, and `$FA` is queued as soon as there is room. Which movement a real
//!   keyboard drops is not documented.
//! * **How fast a person moves keys.** A host has no timing of its own inside
//!   a poll: a VNC client's burst, a paste, or a frontend that fell behind all
//!   deliver many movements at one instant, and taken straight into the type-
//!   ahead buffer everything after the first eleven — one on the line, ten
//!   waiting — is lost exactly as if the Amiga had stopped listening. When
//!   the first lost movement is a release the operating system repeats that
//!   key until another arrives: a stuck key nobody is holding. So host
//!   movements wait in a backlog in front of the controller and enter it at
//!   most one every [`MOVEMENT_TICKS`], faster than anyone types and several
//!   times slower than a code takes to cross the cable and be answered. The
//!   ten-code buffer and `$FA` still mean what the manual says: a computer
//!   that stops answering loses what arrives to a full buffer, whatever pace
//!   it was typed at. A movement during power-up sync, which transmits
//!   nothing, is taken at once.
//! * **Keys moved while synchronising after power-up** are not queued: they
//!   change what is held, and the power-up stream reports what is held.
//!
//! # Not modelled
//!
//! * **Reset warning (`$78`) and hard reset.** "Available on some A1000 and
//!   A2000 keyboards", and hard reset is "valid for all keyboards except the
//!   Amiga 500" — this is an A500's keyboard.
//! * **Phantom keys.** The host delivers whole key identities, not a scanned
//!   matrix, so there is nothing to ghost; the A500 controller's own policy is
//!   to "hold off sending any character until the matrix has cleared", which a
//!   person on a host keyboard would only ever experience as a lost key.
//! * **The Caps Lock LED** is state (it decides the up/down bit Caps Lock
//!   sends) and is readable through [`Keyboard::caps_lock_led`]; nothing draws
//!   it.
//!
//! # Determinism
//!
//! Keys are a non-deterministic input, so they cross the record/replay seam
//! (`CLAUDE.md`): the keyboard's state lives in a named host object,
//! [`Keyboard`], which the device opens by name from `new(props)`.
//! [`keys::channel`] is the channel, [`keys::sink`] what a payload does, and a
//! sealed build refuses a keyboard that has no channel. A payload is one byte
//! per key movement in the manual's own encoding — the raw keycode, with bit 7
//! set for a release. A host that has keysyms rather than keycodes translates
//! first (`host::input::amiga::AmigaKeyboardSink`).

use alloc::boxed::Box;
use alloc::collections::VecDeque;
use alloc::format;
use alloc::string::{String, ToString};
use alloc::sync::Arc;
use alloc::vec::Vec;
use core::fmt;

use crate::core::device::{Device, DeviceClass, PropertySpec, RealizeCtx, ResetKind, SinkPin};
use crate::core::error::{Error, Result};
use crate::core::props::{Props, ValueKind};
use crate::core::sched::{AccessKind, LazyHandle};
use crate::core::state::{ChunkReader, ChunkWriter, Sink, Source};
use crate::core::sync::{AtomicU64, LockRank, Mutex, Ordering};
use crate::core::wire::{Drive, Level, WireId, WireSink, WireSource};
use crate::machine::Instance;
use crate::machine::validate::{ClassSchema, PortDir, PropSchema};

/// The class name a machine file writes.
pub const CLASS_NAME: &str = "amiga.keyboard";

/// Snapshot version for this class's chunk encoding.
const STATE_VERSION: u32 = 2;

/// The host keyboard port a keyboard opens when its `keys` property is not
/// given.
pub const DEFAULT_KEYBOARD_PORT: &str = "keyboard";

/// The clock output: `KCLK`, which the Amiga wires to CIA-A's `CNT`.
pub const KCLK_PIN: &str = "kclk";
/// The data line: `KDAT`, CIA-A's `SP`. Bidirectional.
pub const KDAT_PIN: &str = "kdat";

/// Ticks `KDAT` is set before `KCLK` falls. "About 20 microseconds."
pub const DATA_SETUP_TICKS: u64 = 20;
/// Ticks `KCLK` stays low. "About 20 microseconds."
pub const CLOCK_LOW_TICKS: u64 = 20;
/// Ticks after `KCLK` rises before `KDAT` changes. "Another 20 microseconds."
pub const DATA_HOLD_TICKS: u64 = 20;
/// Ticks after the last clock of a transmission that the keyboard waits for a
/// handshake before it decides it is out of sync: 143 ms.
pub const HANDSHAKE_TIMEOUT_TICKS: u64 = 143_000;
/// How many codes the keyboard holds while the computer is not accepting them.
pub const TYPE_AHEAD: usize = 10;
/// Ticks between host key movements entering the controller: 5 ms, two hundred
/// movements a second. Not the manual's — it gives no scan rate — but a host
/// convention; see the module docs.
pub const MOVEMENT_TICKS: u64 = 5_000;
/// How many host movements wait in front of the controller before the newest
/// is dropped (and `$FA` owed): twenty seconds of them at [`MOVEMENT_TICKS`].
pub const HOST_BACKLOG: usize = 4_096;

/// The codes Appendix G and chapter 8 name.
pub mod code {
    /// Set on a key code to say the key was released.
    pub const KEY_UP: u8 = 0x80;
    /// The highest key code a key has: Right Amiga.
    pub const LAST_KEY: u8 = 0x67;
    /// Caps Lock, which sends only when pushed.
    pub const CAPS_LOCK: u8 = 0x62;
    /// "Reset warning. Ctrl-Amiga-Amiga has been pressed." Not sent by this
    /// model, which is an A500's keyboard.
    pub const RESET_WARNING: u8 = 0x78;
    /// "Last key code bad, next key is same code retransmitted."
    pub const LOST_SYNC: u8 = 0xf9;
    /// "Keyboard key buffer overflow."
    pub const BUFFER_OVERFLOW: u8 = 0xfa;
    /// "Keyboard self-test fail." Never sent by this model.
    pub const SELFTEST_FAILED: u8 = 0xfc;
    /// "Initiate power-up key stream."
    pub const POWER_UP_STREAM: u8 = 0xfd;
    /// "Terminate power-up key stream."
    pub const END_POWER_UP_STREAM: u8 = 0xfe;
}

/// A code in transmission order: rotated left one place, so bit 7 — the up/down
/// flag — goes last. "The transmitted order is therefore 6-5-4-3-2-1-0-7."
#[inline]
#[must_use]
pub const fn wire_order(code: u8) -> u8 {
    code.rotate_left(1)
}

/// What CIA-A's serial data register holds after a code has arrived: the
/// rotated code, inverted, because `KDAT` is active low and the 8520 shifts in
/// the *level*.
#[inline]
#[must_use]
pub const fn sdr_of(code: u8) -> u8 {
    !wire_order(code)
}

/// The code a serial data register byte carries: [`sdr_of`] undone, which is
/// what a keyboard driver computes.
#[inline]
#[must_use]
pub const fn code_of(sdr: u8) -> u8 {
    (!sdr).rotate_right(1)
}

/// A tick no event is scheduled for.
const NO_EVENT: u64 = u64::MAX;

/// How many key codes a key can have: `$00`-`$67`.
const KEY_COUNT: usize = code::LAST_KEY as usize + 1;

// ---------------------------------------------------------------------------
// the protocol
// ---------------------------------------------------------------------------

/// What the transmission in progress, or the one just finished, was.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Sending {
    /// Nothing yet.
    Nothing,
    /// A single one bit, looking for a handshake.
    Sync,
    /// A whole code.
    Code(u8),
}

/// Where in the protocol the keyboard is.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Phase {
    /// Nothing to send and nothing awaited.
    Idle,
    /// Clocking bits out. `bits` holds the rest of them, most significant
    /// first; `left` is how many; `step` is 0 for "clock falls next", 1 for
    /// "clock rises next", 2 for "hold ends next".
    Clocking { bits: u8, left: u8, step: u8 },
    /// Every bit is out; waiting for the handshake until `deadline`.
    Await { deadline: u64 },
}

#[derive(Debug, Clone, PartialEq, Eq)]
struct State {
    /// Ticks of the keyboard's clock simulated.
    ticks: u64,
    /// The tick the next protocol step falls on, or [`NO_EVENT`].
    next: u64,
    phase: Phase,
    sending: Sending,
    /// Whether the keyboard's `KDAT` stage is pulling low.
    kdat_low: bool,
    /// Whether its `KCLK` stage is.
    kclk_low: bool,
    /// Whether a handshake pulse has been seen since the latch was armed.
    latched: bool,
    /// Whether the latch is armed.
    armed: bool,
    /// Whether the computer was holding `KDAT` low when last seen.
    line_low: bool,
    /// Still synchronising after power-up, rather than after a timeout.
    powering_up: bool,
    /// A code the computer never acknowledged, to send again after `$F9`.
    lost: Option<u8>,
    /// The type-ahead buffer.
    queue: VecDeque<u8>,
    /// A movement was dropped and `$FA` is owed.
    overflowed: bool,
    /// Host movements not yet taken into the controller, oldest first.
    pending: VecDeque<u8>,
    /// The tick the oldest of `pending` is taken, or [`NO_EVENT`].
    admit_at: u64,
    /// The earliest tick the next host movement may be taken.
    quiet_until: u64,
    /// Which keys are down, one bit per code.
    held: [u8; KEY_COUNT.div_ceil(8)],
    /// The Caps Lock LED.
    caps: bool,
    /// Codes the computer has acknowledged, for a test or a monitor.
    acknowledged: u64,
}

impl State {
    /// Power-on: synchronising, with the first sync bit about to go out.
    fn power_on(ticks: u64) -> State {
        let mut st = State {
            ticks,
            next: NO_EVENT,
            phase: Phase::Idle,
            sending: Sending::Nothing,
            kdat_low: false,
            kclk_low: false,
            latched: false,
            armed: false,
            line_low: false,
            powering_up: true,
            lost: None,
            queue: VecDeque::new(),
            overflowed: false,
            pending: VecDeque::new(),
            admit_at: NO_EVENT,
            quiet_until: 0,
            held: [0; KEY_COUNT.div_ceil(8)],
            // The LED is lit through self-test and "finally … shut off" at the
            // end of the start-up sequence.
            caps: true,
            acknowledged: 0,
        };
        st.begin(Sending::Sync, ticks);
        st
    }

    fn is_held(&self, key: u8) -> bool {
        self.held[usize::from(key) / 8] & (1 << (key % 8)) != 0
    }

    fn set_held(&mut self, key: u8, down: bool) {
        let bit = 1 << (key % 8);
        let byte = &mut self.held[usize::from(key) / 8];
        if down {
            *byte |= bit;
        } else {
            *byte &= !bit;
        }
    }

    /// Start clocking out `what`, the first bit's data at `at`.
    ///
    /// The data is put on the line at `at` and the clock falls
    /// [`DATA_SETUP_TICKS`] later, so a transmission started from inside a
    /// step lands its first edge in the future rather than on the tick the
    /// device already stands on.
    fn begin(&mut self, what: Sending, at: u64) {
        let (bits, left) = match what {
            Sending::Sync => (0x80, 1),
            Sending::Code(c) => (wire_order(c), 8),
            Sending::Nothing => return,
        };
        self.sending = what;
        self.latched = false;
        self.armed = false;
        self.kdat_low = bits & 0x80 != 0;
        self.phase = Phase::Clocking {
            bits,
            left,
            step: 0,
        };
        self.next = at + DATA_SETUP_TICKS;
    }

    /// Start the next queued code at `at`, or go idle.
    fn send_next(&mut self, at: u64) {
        if self.overflowed && self.queue.len() < TYPE_AHEAD {
            self.overflowed = false;
            self.queue.push_back(code::BUFFER_OVERFLOW);
        }
        match self.queue.pop_front() {
            Some(c) => self.begin(Sending::Code(c), at),
            None => {
                self.phase = Phase::Idle;
                self.sending = Sending::Nothing;
                self.next = NO_EVENT;
            }
        }
    }

    /// Execute the step due at `self.next`, which the caller has made
    /// `self.ticks`. Returns whether `KDAT` was just let go of, so the caller
    /// can look at the line once it has driven it.
    fn step(&mut self) -> bool {
        match self.phase {
            Phase::Clocking { bits, left, step } => match step {
                0 => {
                    self.kclk_low = true;
                    self.phase = Phase::Clocking {
                        bits,
                        left,
                        step: 1,
                    };
                    self.next = self.ticks + CLOCK_LOW_TICKS;
                    false
                }
                1 => {
                    self.kclk_low = false;
                    if left == 1 {
                        // The 8520 has its eighth bit and raises its interrupt
                        // now; the handshake can come from here on.
                        self.armed = true;
                    }
                    self.phase = Phase::Clocking {
                        bits,
                        left,
                        step: 2,
                    };
                    self.next = self.ticks + DATA_HOLD_TICKS;
                    false
                }
                _ => {
                    if left > 1 {
                        let bits = bits << 1;
                        self.kdat_low = bits & 0x80 != 0;
                        self.phase = Phase::Clocking {
                            bits,
                            left: left - 1,
                            step: 0,
                        };
                        self.next = self.ticks + DATA_SETUP_TICKS;
                        false
                    } else {
                        // "After the end of the last KCLK pulse, the keyboard
                        // pulls KDAT high again." The deadline counts from the
                        // last clock, which rose one hold interval ago.
                        self.kdat_low = false;
                        let deadline = self.ticks - DATA_HOLD_TICKS + HANDSHAKE_TIMEOUT_TICKS;
                        self.phase = Phase::Await { deadline };
                        self.next = deadline;
                        true
                    }
                }
            },
            Phase::Await { .. } => {
                // No handshake in 143 ms. "The keyboard will then attempt to
                // restore sync by going into resync mode."
                if let Sending::Code(c) = self.sending {
                    self.lost.get_or_insert(c);
                }
                self.begin(Sending::Sync, self.ticks);
                false
            }
            Phase::Idle => {
                self.next = NO_EVENT;
                false
            }
        }
    }

    /// What the computer is doing to `KDAT`, as the keyboard can see it: the
    /// line's level while the keyboard itself is not pulling it.
    fn sense(&mut self, low: bool) {
        if self.kdat_low {
            // The keyboard's own stage is holding the line; the level says
            // nothing about the computer.
            return;
        }
        if low && self.armed {
            self.latched = true;
        }
        self.line_low = low;
        if !low && self.latched && matches!(self.phase, Phase::Await { .. }) {
            self.acknowledge();
        }
    }

    /// A handshake has been latched and the pulse is over.
    fn acknowledge(&mut self) {
        self.latched = false;
        self.armed = false;
        let at = self.ticks;
        match self.sending {
            Sending::Sync if self.powering_up => {
                // "First, it sends an initiate power-up key stream code … followed
                // by the key codes of all depressed keys … a terminate key
                // stream code." No `$F9`: "The keyboard must not transmit a lost
                // sync code after re-synchronizing due to a power-up."
                self.powering_up = false;
                self.queue.clear();
                self.queue.push_back(code::POWER_UP_STREAM);
                for key in 0..=code::LAST_KEY {
                    if self.is_held(key) {
                        self.queue.push_back(key);
                    }
                }
                self.queue.push_back(code::END_POWER_UP_STREAM);
            }
            Sending::Sync => {
                // "It does this by transmitting a lost sync code … Then it
                // retransmits the code that had been garbled."
                if let Some(c) = self.lost.take() {
                    self.queue.push_front(c);
                    self.queue.push_front(code::LOST_SYNC);
                }
            }
            Sending::Code(c) => {
                self.acknowledged += 1;
                if c == code::END_POWER_UP_STREAM {
                    // "Finally, the Caps Lock LED is shut off."
                    self.caps = false;
                }
            }
            Sending::Nothing => {}
        }
        self.send_next(at);
    }

    /// One key movement from the host, as it arrives: taken now if the
    /// controller is ready for one, otherwise put behind the others waiting.
    fn arrive(&mut self, key: u8) {
        if self.pending.is_empty() && (self.powering_up || self.ticks >= self.quiet_until) {
            self.take(key);
            return;
        }
        if self.pending.len() >= HOST_BACKLOG {
            self.overflowed = true;
            return;
        }
        self.pending.push_back(key);
        if self.admit_at == NO_EVENT {
            self.admit_at = self.quiet_until.max(self.ticks);
        }
    }

    /// Take the oldest waiting host movement that is a transition, at
    /// `admit_at`, which the caller has made `self.ticks`.
    fn admit(&mut self) {
        while let Some(key) = self.pending.pop_front() {
            if self.take(key) {
                break;
            }
        }
        self.admit_at = if self.pending.is_empty() {
            NO_EVENT
        } else {
            self.ticks + MOVEMENT_TICKS
        };
    }

    /// Take `key` into the controller now. Returns whether it was a
    /// transition; only a transition spends the interval.
    fn take(&mut self, key: u8) -> bool {
        let moved = self.movement(key);
        if moved {
            self.quiet_until = self.ticks + MOVEMENT_TICKS;
        }
        moved
    }

    /// The earlier of the protocol's next step and the next admission.
    fn next_event(&self) -> u64 {
        self.next.min(self.admit_at)
    }

    /// One key movement taken into the controller: `key` is the raw code, bit
    /// 7 set for a release. Returns whether it changed what is held.
    fn movement(&mut self, key: u8) -> bool {
        let up = key & code::KEY_UP != 0;
        let key = key & !code::KEY_UP;
        if key > code::LAST_KEY || self.is_held(key) != up {
            return false;
        }
        self.set_held(key, !up);
        let sent = if key == code::CAPS_LOCK {
            // "It generates a keycode only when it is pushed down, never when it
            // is released … When pushing the Caps Lock key turns on the Caps
            // Lock LED, the up/down bit will be 0."
            if up {
                return true;
            }
            self.caps = !self.caps;
            if self.caps { key } else { key | code::KEY_UP }
        } else if up {
            key | code::KEY_UP
        } else {
            key
        };
        if self.powering_up {
            return true;
        }
        if self.queue.len() >= TYPE_AHEAD {
            self.overflowed = true;
            return true;
        }
        self.queue.push_back(sent);
        if self.phase == Phase::Idle {
            self.send_next(self.ticks);
        }
        true
    }

    fn pins(&self) -> (bool, bool) {
        (self.kclk_low, self.kdat_low)
    }
}

// ---------------------------------------------------------------------------
// the host object
// ---------------------------------------------------------------------------

#[derive(Debug, Default, Clone)]
struct Outputs {
    kclk: Option<WireSource>,
    kdat: Option<WireSource>,
}

/// An Amiga keyboard: the protocol state, and the door keys come in through.
///
/// The host object *is* the keyboard, the way `keypad::Keys` is the matrix: a
/// host opens it by name and presses keys on it, the device opens the same
/// name and gives it its wires and its clock.
pub struct Keyboard {
    state: Mutex<State>,
    ticks: AtomicU64,
    next_event: AtomicU64,
    out: Mutex<Outputs>,
    lazy: Mutex<Option<LazyHandle>>,
}

impl fmt::Debug for Keyboard {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let mut s = f.debug_struct("Keyboard");
        match self.state.try_lock() {
            Some(state) => s.field("state", &*state).finish(),
            None => s.field("state", &"<in use>").finish(),
        }
    }
}

impl Default for Keyboard {
    fn default() -> Keyboard {
        Keyboard::new()
    }
}

impl Keyboard {
    /// A keyboard just switched on, attached to nothing.
    #[must_use]
    pub fn new() -> Keyboard {
        let state = State::power_on(0);
        Keyboard {
            ticks: AtomicU64::new(0),
            next_event: AtomicU64::new(state.next_event()),
            state: Mutex::with_rank(LockRank::DEVICE, state),
            out: Mutex::with_rank(LockRank::WIRE, Outputs::default()),
            lazy: Mutex::with_rank(LockRank::LEAF, None),
        }
    }

    /// One key movement: the raw keycode, with [`code::KEY_UP`] set for a
    /// release.
    ///
    /// The device end of the record/replay channel. A code past
    /// [`code::LAST_KEY`] is ignored rather than refused, so a recording made
    /// against some other keyboard cannot abort a replay.
    pub fn key(&self, key: u8) {
        self.sync();
        self.update(|st| {
            st.arrive(key);
            false
        });
    }

    /// Press (`down`) or release the key with raw code `key`.
    pub fn press(&self, key: u8, down: bool) {
        self.key(if down { key } else { key | code::KEY_UP });
    }

    /// Whether the key with raw code `key` is down.
    #[must_use]
    pub fn held(&self, key: u8) -> bool {
        key <= code::LAST_KEY && self.state.lock().is_held(key)
    }

    /// Whether the Caps Lock LED is lit.
    #[must_use]
    pub fn caps_lock_led(&self) -> bool {
        self.state.lock().caps
    }

    /// How many codes the computer has acknowledged since power-on.
    #[must_use]
    pub fn acknowledged(&self) -> u64 {
        self.state.lock().acknowledged
    }

    /// Whether the keyboard is still waiting for the computer's first
    /// handshake after power-on.
    #[must_use]
    pub fn synchronising(&self) -> bool {
        self.state.lock().powering_up
    }

    /// How many codes are waiting to be sent, the one on the line excluded.
    #[must_use]
    pub fn queued(&self) -> usize {
        self.state.lock().queue.len()
    }

    /// How many host movements are waiting to enter the controller.
    #[must_use]
    pub fn backlog(&self) -> usize {
        self.state.lock().pending.len()
    }

    /// Whether the keyboard is pulling `(KCLK, KDAT)` low.
    #[must_use]
    pub fn lines(&self) -> (bool, bool) {
        self.state.lock().pins()
    }

    /// Ticks simulated.
    #[must_use]
    pub fn ticks(&self) -> u64 {
        self.ticks.load(Ordering::Relaxed)
    }

    /// Run the keyboard until `target` ticks have passed in total.
    ///
    /// One protocol step at a time, driving the lines between steps with no
    /// lock held: a clock edge reaches the 8520, which is a device with a lock
    /// of its own, and each edge has to arrive before the next is computed.
    pub fn advance_to(&self, target: u64) {
        loop {
            let stepped = {
                let mut st = self.state.lock();
                if st.next_event() <= target && st.next_event() != NO_EVENT {
                    let before = st.pins();
                    // The protocol's step first when both fall on one tick:
                    // a movement taken then queues behind whatever it does.
                    let released = if st.next <= st.admit_at {
                        st.ticks = st.ticks.max(st.next);
                        st.step()
                    } else {
                        st.ticks = st.ticks.max(st.admit_at);
                        st.admit();
                        false
                    };
                    self.publish(&st);
                    Some((before != st.pins(), released))
                } else {
                    if target > st.ticks {
                        st.ticks = target;
                    }
                    self.publish(&st);
                    None
                }
            };
            let Some((moved, released)) = stepped else {
                break;
            };
            if moved {
                self.refresh();
            }
            if released {
                self.look_at_the_line();
            }
        }
    }

    fn publish(&self, st: &State) {
        self.ticks.store(st.ticks, Ordering::Relaxed);
        self.next_event.store(st.next_event(), Ordering::Relaxed);
    }

    /// Apply `f`, then drive whatever moved — outside the lock — and look at
    /// `KDAT` if `f` let go of it.
    fn update(&self, f: impl FnOnce(&mut State) -> bool) {
        let (moved, released) = {
            let mut st = self.state.lock();
            let before = st.pins();
            let released = f(&mut st);
            self.publish(&st);
            (before != st.pins(), released)
        };
        if moved {
            self.refresh();
        }
        if released {
            self.look_at_the_line();
        }
    }

    /// Drive both output stages to what the state says, holding no lock.
    fn refresh(&self) {
        let (kclk, kdat) = self.state.lock().pins();
        let out = self.out.lock().clone();
        if let Some(src) = &out.kclk {
            src.drive(if kclk { Drive::Low } else { Drive::HiZ });
        }
        if let Some(src) = &out.kdat {
            src.drive(if kdat { Drive::Low } else { Drive::HiZ });
        }
    }

    /// Read `KDAT`'s net after letting go of it.
    ///
    /// A resolved net only delivers a *change*, and a computer that began its
    /// pulse while the keyboard was still holding the line low leaves it low
    /// when the keyboard lets go: no change, no delivery. So the keyboard asks.
    fn look_at_the_line(&self) {
        let source = self.out.lock().kdat.clone();
        let Some(source) = source else {
            return;
        };
        let low = source.net_level().is_low();
        self.update(|st| {
            st.sense(low);
            false
        });
    }

    fn sync(&self) {
        let handle = self.lazy.lock().clone();
        if let Some(handle) = handle {
            // A refusal is this keyboard's own catch-up further up the stack —
            // it drove `KDAT`, and the net came straight back here. The state
            // is already at the instant of that drive.
            let _ = handle.sync(AccessKind::Guest);
        }
    }

    fn power_on(&self) {
        self.update(|st| {
            let ticks = st.ticks;
            let line_low = st.line_low;
            *st = State::power_on(ticks);
            st.line_low = line_low;
            false
        });
    }
}

/// The build's named Amiga keyboards.
///
/// The same shape as `keypad::keys` and `gb::joypad::pads`: a name travels from
/// the machine file into the device constructor, and both ends resolve it
/// against the build's [`HostObjects`](crate::core::hosts::HostObjects).
pub mod keys {
    use super::Keyboard;
    use alloc::string::String;
    use alloc::sync::Arc;
    use alloc::vec::Vec;

    use crate::core::error::Result;
    use crate::core::hosts::{HostKind, HostObjects};
    use crate::core::props::Props;
    use crate::core::record::{Channel, FnSink, InputSink};

    /// The kind a keyboard is filed under in a build's host objects.
    pub const KIND: HostKind = HostKind::door("amiga-keyboard", make_sink);

    /// The keyboard `name` refers to in `hosts`, creating it on first mention.
    ///
    /// # Errors
    ///
    /// [`crate::Error::Config`] if another kind of object holds that name.
    pub fn open(hosts: &HostObjects, name: &str) -> Result<Arc<Keyboard>> {
        hosts.open(KIND, name, Keyboard::new)
    }

    /// The keyboard `name` refers to in the build `props` belongs to — the
    /// device's side. A `Props` that belongs to no build gets a private one.
    ///
    /// # Errors
    ///
    /// As [`open`].
    pub fn attach(props: &Props, name: &str) -> Result<Arc<Keyboard>> {
        props.host(KIND, name, Keyboard::new)
    }

    /// The keyboard called `name`, if a device or a host has opened it.
    ///
    /// # Errors
    ///
    /// As [`open`].
    pub fn get(hosts: &HostObjects, name: &str) -> Result<Option<Arc<Keyboard>>> {
        hosts.get(KIND, name)
    }

    /// Every open name, in order.
    #[must_use]
    pub fn names(hosts: &HostObjects) -> Vec<String> {
        hosts.names(KIND)
    }

    /// Bytes per recorded movement: one, the raw keycode with bit 7 set for a
    /// release — the manual's own encoding of a transition.
    pub const RECORD_BYTES: usize = 1;

    /// The channel the keyboard called `name` is typed on:
    /// `amiga-keyboard:keyboard`.
    #[must_use]
    pub fn channel(name: &str) -> Channel {
        Channel::new(KIND, name)
    }

    /// The sink that applies a recorded payload to `keyboard`, a movement per
    /// byte, in order.
    ///
    /// No rewind hook: the type-ahead buffer is part of the machine snapshot a
    /// rewind restores, and nothing is queued outside it.
    #[must_use]
    pub fn sink(keyboard: &Arc<Keyboard>) -> Arc<dyn InputSink> {
        let keyboard = Arc::clone(keyboard);
        Arc::new(FnSink::new("amiga-keyboard", move |payload: &[u8]| {
            for key in payload {
                keyboard.key(*key);
            }
        }))
    }

    fn make_sink(object: &Arc<dyn core::any::Any + Send + Sync>) -> Option<Arc<dyn InputSink>> {
        Some(sink(&Arc::clone(object).downcast::<Keyboard>().ok()?))
    }
}

// ---------------------------------------------------------------------------
// the device
// ---------------------------------------------------------------------------

/// `KDAT` as something a wire drives.
#[derive(Debug)]
struct DataPin {
    keyboard: Arc<Keyboard>,
}

impl WireSink for DataPin {
    fn set_level(&self, _src: WireId, _line: u32, level: Level) {
        self.keyboard.sync();
        let low = level.is_low();
        self.keyboard.update(|st| {
            st.sense(low);
            false
        });
    }
}

/// The `amiga.keyboard` device.
#[derive(Debug)]
pub struct AmigaKeyboard {
    keyboard: Arc<Keyboard>,
    pin: Arc<DataPin>,
}

impl AmigaKeyboard {
    /// Validate `props` and open the keyboard they name.
    ///
    /// # Errors
    ///
    /// [`Error::Property`] for a property this class does not know, or
    /// [`Error::Config`] if the name is held by something that is not a
    /// keyboard.
    pub fn new(props: &Props) -> Result<AmigaKeyboard> {
        let mut r = props.reader();
        let port = r.or_str("keys", DEFAULT_KEYBOARD_PORT)?.to_string();
        r.finish()?;
        let keyboard = keys::attach(props, &port)?;
        Ok(AmigaKeyboard::with(keyboard))
    }

    /// A device around a keyboard the caller already holds.
    #[must_use]
    pub fn with(keyboard: Arc<Keyboard>) -> AmigaKeyboard {
        let pin = Arc::new(DataPin {
            keyboard: Arc::clone(&keyboard),
        });
        AmigaKeyboard { keyboard, pin }
    }

    /// The keyboard.
    #[must_use]
    pub fn keyboard(&self) -> &Arc<Keyboard> {
        &self.keyboard
    }
}

impl Device for AmigaKeyboard {
    fn class(&self) -> &'static DeviceClass {
        &CLASS
    }

    fn realize(&self, _ctx: &mut RealizeCtx<'_>) -> Result<()> {
        Ok(())
    }

    fn reset(&self, kind: ResetKind) {
        // The keyboard is its own computer on the end of a cable and is powered
        // for as long as the Amiga is: only a power cycle restarts it.
        if kind == ResetKind::Cold {
            self.keyboard.power_on();
        }
    }

    fn save(&self, w: &mut ChunkWriter<'_>) -> Result<()> {
        let st = self.keyboard.state.lock().clone();
        w.write_u64(st.ticks)?;
        w.write_u64(st.next)?;
        match st.phase {
            Phase::Idle => w.write_u8(0)?,
            Phase::Clocking { bits, left, step } => {
                w.write_u8(1)?;
                w.write_u8(bits)?;
                w.write_u8(left)?;
                w.write_u8(step)?;
            }
            Phase::Await { deadline } => {
                w.write_u8(2)?;
                w.write_u64(deadline)?;
            }
        }
        match st.sending {
            Sending::Nothing => w.write_u16(0)?,
            Sending::Sync => w.write_u16(1)?,
            Sending::Code(c) => w.write_u16(0x100 | u16::from(c))?,
        }
        for v in [
            st.kdat_low,
            st.kclk_low,
            st.latched,
            st.armed,
            st.line_low,
            st.powering_up,
            st.overflowed,
            st.caps,
        ] {
            w.write_bool(v)?;
        }
        w.write_u16(st.lost.map_or(0, |c| 0x100 | u16::from(c)))?;
        w.write_bytes(&st.queue.iter().copied().collect::<Vec<u8>>())?;
        w.write_bytes(&st.held)?;
        w.write_u64(st.acknowledged)?;
        w.write_bytes(&st.pending.iter().copied().collect::<Vec<u8>>())?;
        w.write_u64(st.admit_at)?;
        w.write_u64(st.quiet_until)
    }

    fn load(&self, r: &mut ChunkReader<'_>) -> Result<()> {
        let bad = |what: &str| Error::State(format!("{CLASS_NAME}: {what}"));
        let mut st = State::power_on(0);
        st.ticks = r.read_u64()?;
        st.next = r.read_u64()?;
        st.phase = match r.read_u8()? {
            0 => Phase::Idle,
            1 => {
                let bits = r.read_u8()?;
                let left = r.read_u8()?;
                let step = r.read_u8()?;
                if !(1..=8).contains(&left) || step > 2 {
                    return Err(bad("a transmission step out of range"));
                }
                Phase::Clocking { bits, left, step }
            }
            2 => Phase::Await {
                deadline: r.read_u64()?,
            },
            _ => return Err(bad("an unknown protocol phase")),
        };
        st.sending = match r.read_u16()? {
            0 => Sending::Nothing,
            1 => Sending::Sync,
            v if v & 0x100 != 0 => Sending::Code(v as u8),
            _ => return Err(bad("an unknown transmission")),
        };
        st.kdat_low = r.read_bool()?;
        st.kclk_low = r.read_bool()?;
        st.latched = r.read_bool()?;
        st.armed = r.read_bool()?;
        st.line_low = r.read_bool()?;
        st.powering_up = r.read_bool()?;
        st.overflowed = r.read_bool()?;
        st.caps = r.read_bool()?;
        let lost = r.read_u16()?;
        st.lost = (lost & 0x100 != 0).then_some(lost as u8);
        let queue = r.read_bytes()?;
        if queue.len() > TYPE_AHEAD + 2 {
            return Err(bad("a type-ahead buffer longer than the keyboard's"));
        }
        st.queue = queue.iter().copied().collect();
        let held = r.read_bytes()?;
        if held.len() != st.held.len() {
            return Err(bad("a held-key map of the wrong size"));
        }
        st.held.copy_from_slice(held);
        st.acknowledged = r.read_u64()?;
        let pending = r.read_bytes()?;
        if pending.len() > HOST_BACKLOG {
            return Err(bad("a host backlog longer than the keyboard keeps"));
        }
        st.pending = pending.iter().copied().collect();
        st.admit_at = r.read_u64()?;
        st.quiet_until = r.read_u64()?;
        if st.next != NO_EVENT && st.next <= st.ticks && st.phase != Phase::Idle {
            // An event that is not in the future would stall catch-up.
            st.next = st.ticks + 1;
        }
        if st.pending.is_empty() {
            st.admit_at = NO_EVENT;
        } else if st.admit_at == NO_EVENT || st.admit_at <= st.ticks {
            st.admit_at = st.ticks + 1;
        }
        self.keyboard.update(|now| {
            *now = st;
            false
        });
        Ok(())
    }

    fn connect(&self, port: &str, source: WireSource) -> Result<()> {
        {
            let mut out = self.keyboard.out.lock();
            match port {
                KCLK_PIN => out.kclk = Some(source),
                KDAT_PIN => out.kdat = Some(source),
                _ => {
                    return Err(Error::Config {
                        at: port.to_string(),
                        message: format!("an Amiga keyboard drives `{KCLK_PIN}` and `{KDAT_PIN}`"),
                    });
                }
            }
        }
        self.keyboard.refresh();
        Ok(())
    }

    fn announce(&self, _port: &str) {
        self.keyboard.refresh();
    }

    fn sink(&self, port: &str, _sources: &[WireId]) -> Option<SinkPin> {
        (port == KDAT_PIN).then(|| SinkPin {
            sink: Arc::clone(&self.pin) as Arc<dyn WireSink>,
            line: 0,
        })
    }

    fn is_lazy(&self) -> bool {
        true
    }

    fn current_tick(&self) -> u64 {
        self.keyboard.ticks.load(Ordering::Relaxed)
    }

    fn advance_to(&self, tick: u64) {
        self.keyboard.advance_to(tick);
    }

    fn next_event_tick(&self) -> Option<u64> {
        match self.keyboard.next_event.load(Ordering::Relaxed) {
            NO_EVENT => None,
            tick => Some(tick),
        }
    }

    fn attach_lazy(&self, handle: LazyHandle) {
        *self.keyboard.lazy.lock() = Some(handle);
    }
}

impl Instance for AmigaKeyboard {}

/// The `amiga.keyboard` device class.
pub static CLASS: DeviceClass = DeviceClass {
    name: CLASS_NAME,
    version: STATE_VERSION,
    summary: "an Amiga keyboard: Appendix G's KCLK/KDAT protocol with its handshake, \
              resync, power-up stream and type-ahead buffer, keyed from the host",
    properties: &[PropertySpec {
        name: "keys",
        kind: ValueKind::Str,
        required: false,
        summary: "the host keyboard port it is typed on (default `keyboard`)",
    }],
    construct: |props| Ok(Box::new(AmigaKeyboard::new(props)?)),
};

/// Add [`CLASS`] to a registry.
///
/// # Errors
///
/// If something already claimed the name.
pub fn register(registry: &mut crate::core::Registry) -> Result<()> {
    registry.add(&CLASS)
}

/// Bind [`CLASS`] into the machine graph.
///
/// # Errors
///
/// If the class is already bound.
pub fn bind(bindings: &mut crate::machine::Bindings) -> Result<()> {
    bindings.bind(CLASS_NAME, |props| Ok(Arc::new(AmigaKeyboard::new(props)?)))
}

/// What the validator should know about `amiga.keyboard`.
#[must_use]
pub fn schema() -> ClassSchema {
    ClassSchema::new(CLASS_NAME)
        .prop(PropSchema::new("keys", ValueKind::Str))
        .port(KCLK_PIN, PortDir::Out)
        .port(KDAT_PIN, PortDir::InOut)
}

/// A short name for a code, for a monitor or a failing test.
#[must_use]
pub fn describe(code_value: u8) -> String {
    match code_value {
        code::LOST_SYNC => String::from("lost sync"),
        code::BUFFER_OVERFLOW => String::from("buffer overflow"),
        code::SELFTEST_FAILED => String::from("self-test failed"),
        code::POWER_UP_STREAM => String::from("power-up key stream"),
        code::END_POWER_UP_STREAM => String::from("end of power-up key stream"),
        c if c & code::KEY_UP != 0 => format!("${:02X} up", c & !code::KEY_UP),
        c => format!("${c:02X} down"),
    }
}

#[cfg(test)]
mod tests;
