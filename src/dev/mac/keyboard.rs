//! `mac.keyboard`: the Macintosh Plus keyboard on the end of its coiled cable,
//! speaking the Guide's request/response protocol on the VIA's shift register.
//!
//! ```text
//!   osc   kbdclk = 1000000 Hz
//!   object kbd "mac.keyboard" { clock = kbdclk }
//!
//!   wire kbd.clk  -> via.cb1  { pull = "up" }   # the clock: the keyboard's,
//!   wire kbd.data -> via.cb2  { pull = "up" }   #   always. the data line is
//!   wire via.cb2  -> kbd.data                   #   driven from both ends
//! ```
//!
//! This models the **protocol**, not the microcontroller inside the keyboard:
//! there is no 6805 core here and no keyboard firmware image, and none is
//! wanted. The bit timing, the four commands, the responses and the handshake
//! are behaviour of the interface, and the interface is what the Macintosh
//! sees.
//!
//! # Sources
//!
//! *Guide to the Macintosh Family Hardware*, 2nd edition (Apple Computer),
//! **chapter 7, "Macintosh Plus Mouse and Keyboard"**, pp. 281-284:
//!
//! * The wires (p. 282): "the Keyboard Data line is bidirectional, driven at
//!   some times by the computer, and at other times by the keyboard, but the
//!   **Keyboard Clock line is driven only by the keyboard**. All data transfers
//!   are synchronous with the signals on the Keyboard Clock line, and each
//!   transmission consists of 8 bits, with the highest-order bit first."
//! * Keyboard to computer (p. 282): "eight cycles of 330 µs each (160 µs low,
//!   170 µs high) on the normally high Keyboard Clock line. It places a data
//!   bit on the data line 40 µs before the falling edge of each clock cycle and
//!   maintains it for 330 µs. The VIA in the computer latches the data bit into
//!   its Shift register on the **rising edge** of the Keyboard Clock signal."
//! * Computer to keyboard (p. 282): "eight cycles of 400 µs each (180 µs low,
//!   220 µs high) … On the **falling edge** of each keyboard clock cycle, the
//!   Macintosh Plus places a data bit on the data line and holds it there for
//!   400 µs. The keyboard reads the data bit 80 µs after the rising edge."
//! * The handshake (p. 282): "Only the computer can initiate communication …
//!   The computer signals that it is ready to begin communication by pulling
//!   the Keyboard Data line low. Upon detecting this signal, the keyboard
//!   starts sending a Keyboard Clock signal, and the computer responds by
//!   sending an 8-bit command … The last bit of the command leaves the Keyboard
//!   Data line low; the computer then indicates that it is ready to receive the
//!   keyboard's response by setting the Keyboard Data line high."
//! * The sequence (p. 282): "The first command the computer transmits is the
//!   Model Number command. The keyboard's response to this command is to reset
//!   itself and send back its model number … If the computer does not receive a
//!   response within 0.5 second, it transmits the Model Number command again."
//!   Then Inquiry every 0.25 second, "if no key transition has occurred after
//!   0.25 second, the keyboard sends back a Null response."
//! * **Table 7-4** (p. 283) for the four commands and their responses, and the
//!   paragraph below it for the key-transition format: "bit 7 high means a
//!   key-up transition, and bit 7 low means a key-down transition. Bit 0 is
//!   always high."
//!
//! and the *Synertek SY6522* data sheet for the shift register at the other
//! end. **No emulator source was consulted and no ROM was disassembled**
//! (`ROADMAP.md` §1, `CLAUDE.md`); where this file makes a choice the Guide
//! does not state, it says so below.
//!
//! # Which shift-register mode this is, and why it is the only one it can be
//!
//! The 6522 has eight shift-register modes in ACR bits 4-2, and the clock is
//! what chooses between them: three take it from timer 2, two from φ2, and two
//! from **CB1 as an input**. The Guide settles it in one sentence — the clock
//! line "is driven only by the keyboard" — so the computer cannot be generating
//! it, and the T2 and φ2 modes are all out. That leaves exactly two, and they
//! are the two directions of the same link:
//!
//! | phase | ACR bits 4-2 | data sheet's name |
//! | --- | --- | --- |
//! | the command, computer to keyboard | `111` (`$1C` in ACR) | shift out under control of external CB1 clock |
//! | the response, keyboard to computer | `011` (`$0C`) | shift in under control of external CB1 clock |
//!
//! A black-box trace of a real ROM agrees: every keyboard transaction it
//! attempts is `ACR = $1C` followed by `SR = $16`, retried every 0.53 virtual
//! seconds — the Guide's half-second Model Number timeout — for ever.
//!
//! The one place the ROM uses a third mode is to raise the flag in the first
//! place. `ACR = $18` is mode `110`, shift out under φ2, and writing `SR = $00`
//! into it walks a zero onto CB2 at the chip's own rate: that is how the
//! computer "pulls the Keyboard Data line low", with no clock from the keyboard
//! to do it with. It then disables the register and re-enables it in mode
//! `111`, which is why [`super::via`]'s output stage is a flip-flop that
//! survives a mode change rather than a tap on bit 7 — the line has to stay low
//! across the gap.
//!
//! # Time
//!
//! **One tick of this device's `clock` is one microsecond**, because every
//! number the Guide gives is in microseconds and the keyboard has a crystal of
//! its own rather than a division of the computer's. A board gives it a 1 MHz
//! oscillator. A different rate is not refused — a device cannot see its
//! domain's frequency — but every interval below then scales with it.
//!
//! # What is chosen where the Guide is silent
//!
//! * **How long after the data line goes low the first clock falls.** The Guide
//!   says only "upon detecting this signal". A real keyboard polls its input
//!   pin; this one latches the level, which cannot miss a signal a poll can,
//!   and starts one transmit bit time later ([`START_TICKS`]).
//! * **The turnaround.** The Guide gives the computer's raising of the data
//!   line as the go-ahead but no delay after it, so the response begins one
//!   receive bit time later ([`TURNAROUND_TICKS`]) — long enough for the
//!   computer to have reprogrammed ACR, short enough to be invisible against
//!   the quarter-second poll.
//! * **How long the keyboard takes to reset itself** for a Model Number
//!   command ([`RESET_TICKS`], 5 ms). The Guide says it resets; it does not say
//!   for how long. Anything under the computer's half-second timeout behaves
//!   identically, and 5 ms is a plausible microcontroller reset.
//! * **A computer that never raises the data line.** The transaction is
//!   abandoned after [`TURNAROUND_TIMEOUT_TICKS`] and the keyboard goes back to
//!   watching the line, which is what lets the computer's own half-second
//!   timeout and retry work rather than deadlocking against it.
//! * **An unknown command** is answered with Null. The Guide lists four and
//!   does not say what a fifth does.
//! * **Key repeat** is the operating system's on a Macintosh, so a down for a
//!   key already down, and an up for one already up, are dropped.
//!
//! # Not modelled
//!
//! * **The separate keypad** and the Keypad response (`$79`) prefix the Guide
//!   describes for the arrow and keypad keys: this is a keyboard with nothing
//!   plugged into it, so its Model Number response has bit 7 clear.
//! * **A host keymap.** [`Keyboard::key`] takes the Guide's own transition
//!   code, which is what crosses the cable; turning a keysym into one needs
//!   Figure 7-6's table and belongs in `host/input`.
//!
//! # Determinism
//!
//! Keys are a non-deterministic input, so they cross the record/replay seam
//! (`CLAUDE.md`): the keyboard's state lives in a named host object,
//! [`Keyboard`], which the device opens by name from `new(props)`.
//! [`keys::channel`] is the channel and [`keys::sink`] what a payload does —
//! one byte per transition, in the Guide's encoding.

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
pub const CLASS_NAME: &str = "mac.keyboard";

/// The snapshot chunk version. Bump with the encoding, never on its own.
pub const STATE_VERSION: u32 = 1;

/// The host keyboard port a keyboard opens when its `keys` property is not
/// given.
pub const DEFAULT_KEYBOARD_PORT: &str = "keyboard";

/// The clock output: the Keyboard Clock line, which a Macintosh wires to the
/// VIA's CB1. Driven only by the keyboard.
pub const CLK_PIN: &str = "clk";
/// The Keyboard Data line, the VIA's CB2. Driven from both ends.
pub const DATA_PIN: &str = "data";

// -- the Guide's timing, in microseconds -------------------------------------

/// A computer-to-keyboard clock cycle: "eight cycles of 400 µs each".
pub const TX_CYCLE_TICKS: u64 = 400;
/// How long that cycle holds the clock low: "180 µs low".
pub const TX_LOW_TICKS: u64 = 180;
/// How long after the rising edge the keyboard reads the data bit: "80 µs".
///
/// Recorded because it is the Guide's, and *not* used as an instant to sample
/// at: the command bit is frozen while the clock is low instead, for the
/// reason the command step spells out.
pub const TX_SAMPLE_TICKS: u64 = 80;

/// A keyboard-to-computer clock cycle: "eight cycles of 330 µs each".
pub const RX_CYCLE_TICKS: u64 = 330;
/// How long that cycle holds the clock low: "160 µs low".
pub const RX_LOW_TICKS: u64 = 160;
/// How far before the falling edge the keyboard puts its bit on the line:
/// "40 µs before the falling edge of each clock cycle".
pub const RX_SETUP_TICKS: u64 = 40;

/// How long after the computer pulls the data line low the first clock falls.
/// Not the Guide's — see the module docs.
pub const START_TICKS: u64 = TX_CYCLE_TICKS;
/// How long after the computer raises the data line the response begins. Not
/// the Guide's — see the module docs.
pub const TURNAROUND_TICKS: u64 = RX_CYCLE_TICKS;
/// How long the keyboard waits for that raise before giving up: 0.25 s, half
/// the computer's own retry interval.
pub const TURNAROUND_TIMEOUT_TICKS: u64 = 250_000;
/// How long a Model Number command takes to answer: the keyboard "resets
/// itself" first. Not the Guide's — see the module docs.
pub const RESET_TICKS: u64 = 5_000;
/// How long an Inquiry waits for a key before answering Null: "if no key
/// transition has occurred after 0.25 second".
pub const INQUIRY_TIMEOUT_TICKS: u64 = 250_000;

/// How many transitions the keyboard holds while the computer is not asking.
pub const TYPE_AHEAD: usize = 16;

/// The commands the computer sends and the responses it gets, from Table 7-4.
pub mod code {
    /// Inquiry: a key transition, or [`NULL`] after a quarter of a second.
    pub const INQUIRY: u8 = 0x10;
    /// Instant: the same, without the quarter-second wait.
    pub const INSTANT: u8 = 0x14;
    /// Model Number: the keyboard resets itself and answers with [`model`].
    pub const MODEL_NUMBER: u8 = 0x16;
    /// Test: the keyboard self-tests and answers [`ACK`] or [`NAK`].
    pub const TEST: u8 = 0x36;

    /// "Key transition code or Null (`$7B`)."
    pub const NULL: u8 = 0x7b;
    /// A passing self-test.
    pub const ACK: u8 = 0x7d;
    /// A failing one. Never sent by this model.
    pub const NAK: u8 = 0x77;
    /// The Keypad response, which prefixes a transition from the separate
    /// keypad. Never sent by this model, which has nothing plugged into it.
    pub const KEYPAD: u8 = 0x79;

    /// Set on a transition code to say the key was released. "Bit 7 high means
    /// a key-up transition, and bit 7 low means a key-down transition."
    pub const KEY_UP: u8 = 0x80;
    /// "Bit 0 is always high", so a transition code is odd and the six bits
    /// between are the key.
    pub const ALWAYS: u8 = 0x01;

    /// The Model Number response, built from Table 7-4's four fields.
    ///
    /// "Bit 0: 1. Bits 1-3: keyboard model number, 1-8. Bits 4-6: next device
    /// number, 1-8. Bit 7: 1 if another device connected."
    ///
    /// `model` and `next` are the *numbers*, 1 to 8, not the encoded fields.
    #[must_use]
    pub const fn model(model: u8, next: Option<u8>) -> u8 {
        let mut value = ALWAYS | ((model & 7) << 1);
        if let Some(next) = next {
            value |= 0x80 | ((next & 7) << 4);
        }
        value
    }

    /// What a Macintosh Plus keyboard with nothing plugged into it answers:
    /// model 1, no next device. `$03`.
    pub const MODEL_M0110: u8 = model(1, None);
}

/// A tick no event is scheduled for.
const NO_EVENT: u64 = u64::MAX;

/// How many bits one transmission is. "Each transmission consists of 8 bits,
/// with the highest-order bit first."
const BITS: u8 = 8;

// ---------------------------------------------------------------------------
// the protocol
// ---------------------------------------------------------------------------

/// Where in the protocol the keyboard is.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Phase {
    /// Nothing is happening. The data line is being watched for the computer's
    /// "ready to begin".
    Idle,
    /// It has gone low; the first clock edge is due.
    Starting,
    /// Clocking the computer's command in. `left` bits to go; `step` is 0 for
    /// "the clock falls next" and 1 for "it rises next".
    Command { got: u8, left: u8, step: u8 },
    /// The command is in and the computer has yet to raise the data line.
    Turnaround { command: u8, deadline: u64 },
    /// The response is chosen and its first bit is due.
    Answering { byte: u8 },
    /// Clocking the response out. `bits` holds what is left of it, most
    /// significant first; `step` is 0 for "put the bit on the line next", 1 for
    /// "the clock falls next", 2 for "it rises next".
    Response { bits: u8, left: u8, step: u8 },
    /// Every bit is out; the last one is still being held for its 330 µs.
    Trailing,
}

#[derive(Debug, Clone, PartialEq, Eq)]
struct State {
    /// Ticks of the keyboard's own clock simulated. One is a microsecond.
    ticks: u64,
    /// The tick the next protocol step falls on, or [`NO_EVENT`].
    next: u64,
    phase: Phase,
    /// Whether the keyboard's clock stage is pulling the line low.
    clk_low: bool,
    /// Whether its data stage is.
    data_low: bool,
    /// Whether the *line* was low when last seen — the computer's level and
    /// the keyboard's own, resolved.
    line_low: bool,
    /// The command bit the line was carrying while the clock was last low.
    ///
    /// See [`State::command_step`] for why the bit is frozen there rather than
    /// read back off the line afterwards.
    latched: bool,
    /// Transitions waiting to be asked for, oldest first.
    queue: VecDeque<u8>,
    /// Which keys are down, one bit per six-bit key number.
    held: [u8; 8],
    /// The keyboard's model number, 1 to 8.
    model: u8,
    /// How many commands the computer has completed, for a test or a monitor.
    commands: u64,
    /// The last command the keyboard received, for the same.
    last_command: u8,
    /// The last response it sent.
    last_response: u8,
}

impl State {
    /// Power-on: idle, watching the line, with nothing typed.
    fn power_on(ticks: u64, model: u8) -> State {
        State {
            ticks,
            next: NO_EVENT,
            phase: Phase::Idle,
            clk_low: false,
            data_low: false,
            line_low: false,
            latched: false,
            queue: VecDeque::new(),
            held: [0; 8],
            model,
            commands: 0,
            last_command: 0,
            last_response: 0,
        }
    }

    /// What the two output stages are pulling, as a pair.
    fn pins(&self) -> (bool, bool) {
        (self.clk_low, self.data_low)
    }

    /// The level on the data line changed.
    ///
    /// The only thing an idle keyboard does with it is start: "the computer
    /// signals that it is ready to begin communication by pulling the Keyboard
    /// Data line low".
    fn sense(&mut self, low: bool) {
        self.line_low = low;
        match self.phase {
            // While the clock is low the line is carrying the command bit the
            // computer placed on the falling edge, so every level that arrives
            // in that window is that bit.
            Phase::Command { step: 1, .. } => self.latched = !low,
            Phase::Idle if low => {
                self.phase = Phase::Starting;
                self.next = self.ticks + START_TICKS;
            }
            // "The computer then indicates that it is ready to receive the
            // keyboard's response by setting the Keyboard Data line high."
            Phase::Turnaround { command, .. } if !low => {
                self.begin_answer(command);
            }
            _ => {}
        }
    }

    /// Choose the response to `command` and schedule its first bit.
    fn begin_answer(&mut self, command: u8) {
        self.commands = self.commands.wrapping_add(1);
        self.last_command = command;
        let (byte, delay) = match command {
            code::MODEL_NUMBER => (code::model(self.model, None), RESET_TICKS),
            code::TEST => (code::ACK, TURNAROUND_TICKS),
            code::INSTANT => (self.take().unwrap_or(code::NULL), TURNAROUND_TICKS),
            code::INQUIRY => match self.take() {
                Some(key) => (key, TURNAROUND_TICKS),
                // "If no key transition has occurred after 0.25 second, the
                // keyboard sends back a Null response to let the computer know
                // it's still there." The wait is real: a key pressed during it
                // is answered instead, which is checked when the moment comes.
                None => (code::NULL, INQUIRY_TIMEOUT_TICKS),
            },
            // The Guide lists four commands and does not say what a fifth
            // does. Null is the answer that keeps the link alive.
            _ => (code::NULL, TURNAROUND_TICKS),
        };
        self.phase = Phase::Answering { byte };
        self.next = self.ticks + delay;
    }

    /// The oldest waiting transition, if there is one.
    fn take(&mut self) -> Option<u8> {
        self.queue.pop_front()
    }

    /// A key moved: the Guide's transition code, with [`code::KEY_UP`] set for
    /// a release.
    ///
    /// A movement that changes nothing is dropped — key repeat belongs to the
    /// operating system on a Macintosh — and one that arrives with the buffer
    /// full is lost, because a keyboard with nowhere to put it has no way to
    /// say so: Table 7-4 has no overflow response.
    fn arrive(&mut self, transition: u8) {
        let key = (transition & !code::KEY_UP) >> 1;
        let down = transition & code::KEY_UP == 0;
        let (word, bit) = (usize::from(key) / 8, key % 8);
        let was = self.held[word] & (1 << bit) != 0;
        if was == down {
            return;
        }
        if down {
            self.held[word] |= 1 << bit;
        } else {
            self.held[word] &= !(1 << bit);
        }
        if self.queue.len() < TYPE_AHEAD {
            // Bit 0 is always high on the wire whatever the caller passed.
            self.queue.push_back(transition | code::ALWAYS);
        }
    }

    /// One protocol step, at `self.next`.
    fn step(&mut self) {
        match self.phase {
            Phase::Idle | Phase::Turnaround { .. } => self.next = NO_EVENT,
            Phase::Starting => {
                self.phase = Phase::Command {
                    got: 0,
                    left: BITS,
                    step: 0,
                };
                self.step();
            }
            Phase::Command { got, left, step } => self.command_step(got, left, step),
            Phase::Answering { byte } => {
                // An Inquiry that waited out its quarter second answers with
                // whatever arrived during it, if anything did.
                let byte = if byte == code::NULL && self.last_command == code::INQUIRY {
                    self.take().unwrap_or(code::NULL)
                } else {
                    byte
                };
                self.last_response = byte;
                self.phase = Phase::Response {
                    bits: byte,
                    left: BITS,
                    step: 0,
                };
                self.step();
            }
            Phase::Response { bits, left, step } => self.response_step(bits, left, step),
            Phase::Trailing => {
                self.data_low = false;
                self.phase = Phase::Idle;
                self.next = NO_EVENT;
                // The computer may already be holding the line low for the
                // next command; whether it is, is answered by the sweep the
                // device does after letting go.
            }
        }
    }

    /// One step of clocking the computer's command in.
    ///
    /// "On the falling edge of each keyboard clock cycle, the Macintosh Plus
    /// places a data bit on the data line and holds it there for 400 µs. The
    /// keyboard reads the data bit 80 µs after the rising edge."
    ///
    /// # Why the bit is frozen rather than read at the Guide's instant
    ///
    /// Those two sentences name the same bit: the computer holds it for the
    /// whole 400 µs cycle, so reading it 260 µs in gets what was placed at the
    /// start. The second sentence is a statement about setup and hold, not
    /// about which bit is read — and it only stays equivalent while the
    /// computer honours the hold. An *emulated* computer does not: the shift
    /// register's interrupt tells it the last bit is out, and it can reprogram
    /// ACR and take CB2 back in no guest time at all, 80 µs before this
    /// keyboard would have looked. So the keyboard freezes what the line
    /// carries while the clock is low — the same bit, by the same guarantee,
    /// and one nothing can take back afterwards.
    ///
    /// The other half of the same problem is in [`super::via`]: the shift
    /// register completes its count on the *trailing* edge, so the interrupt
    /// does not arrive until the pulse that carried the bit is over.
    fn command_step(&mut self, got: u8, left: u8, step: u8) {
        if step == 0 {
            self.clk_low = true;
            self.phase = Phase::Command { got, left, step: 1 };
            // The line is pulled low for a zero and released for a one, so what
            // the keyboard reads is the level, not its inverse. This is the
            // level *before* the falling edge has been delivered; the computer
            // answering that edge overwrites it through `sense`.
            self.latched = !self.line_low;
            self.next = self.ticks + TX_LOW_TICKS;
            return;
        }
        self.clk_low = false;
        let got = (got << 1) | u8::from(self.latched);
        if left > 1 {
            self.phase = Phase::Command {
                got,
                left: left - 1,
                step: 0,
            };
            self.next = self.ticks + (TX_CYCLE_TICKS - TX_LOW_TICKS);
        } else {
            self.phase = Phase::Turnaround {
                command: got,
                deadline: self.ticks + TURNAROUND_TIMEOUT_TICKS,
            };
            self.next = self.ticks + TURNAROUND_TIMEOUT_TICKS;
            // The computer may have raised the line already: this edge is the
            // one that completes its shift register's count, and it can answer
            // the interrupt before the keyboard's next event.
            if !self.line_low {
                self.begin_answer(got);
            }
        }
    }

    /// One step of clocking the response out.
    ///
    /// "It places a data bit on the data line 40 µs before the falling edge of
    /// each clock cycle and maintains it for 330 µs. The VIA in the computer
    /// latches the data bit into its Shift register on the rising edge."
    fn response_step(&mut self, bits: u8, left: u8, step: u8) {
        match step {
            0 => {
                // Open collector: a zero pulls the line down, a one lets go.
                self.data_low = bits & 0x80 == 0;
                self.phase = Phase::Response {
                    bits,
                    left,
                    step: 1,
                };
                self.next = self.ticks + RX_SETUP_TICKS;
            }
            1 => {
                self.clk_low = true;
                self.phase = Phase::Response {
                    bits,
                    left,
                    step: 2,
                };
                self.next = self.ticks + RX_LOW_TICKS;
            }
            _ => {
                self.clk_low = false;
                if left > 1 {
                    self.phase = Phase::Response {
                        bits: bits << 1,
                        left: left - 1,
                        step: 0,
                    };
                    // The bit is held 330 µs from when it went on the line, so
                    // the next one goes on 40 µs before the next falling edge.
                    self.next = self.ticks + (RX_CYCLE_TICKS - RX_SETUP_TICKS - RX_LOW_TICKS);
                } else {
                    self.phase = Phase::Trailing;
                    self.next = self.ticks + (RX_CYCLE_TICKS - RX_SETUP_TICKS - RX_LOW_TICKS);
                }
            }
        }
    }

    /// The tick the device owes itself, whichever of its two timers is nearer.
    fn next_event(&self) -> u64 {
        match self.phase {
            Phase::Turnaround { deadline, .. } => deadline,
            _ => self.next,
        }
    }
}

// ---------------------------------------------------------------------------
// the host object
// ---------------------------------------------------------------------------

#[derive(Debug, Default, Clone)]
struct Outputs {
    clk: Option<WireSource>,
    data: Option<WireSource>,
}

/// A Macintosh Plus keyboard: the protocol state, and the door keys come in
/// through.
///
/// The host object *is* the keyboard, the way `amiga::keyboard::Keyboard` is: a
/// host opens it by name and presses keys on it, the device opens the same name
/// and gives it its wires and its clock.
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
    /// A keyboard just switched on, attached to nothing: model 1, the Macintosh
    /// Plus keyboard's own.
    #[must_use]
    pub fn new() -> Keyboard {
        Keyboard::with_model(1)
    }

    /// The same, answering Model Number with `model` — 1 to 8, Table 7-4's
    /// range.
    #[must_use]
    pub fn with_model(model: u8) -> Keyboard {
        let state = State::power_on(0, model.clamp(1, 8));
        Keyboard {
            ticks: AtomicU64::new(0),
            next_event: AtomicU64::new(state.next_event()),
            state: Mutex::with_rank(LockRank::DEVICE, state),
            out: Mutex::with_rank(LockRank::WIRE, Outputs::default()),
            lazy: Mutex::with_rank(LockRank::LEAF, None),
        }
    }

    /// One key movement, in the Guide's own encoding: the transition code, with
    /// [`code::KEY_UP`] set for a release.
    ///
    /// The device end of the record/replay channel.
    pub fn key(&self, transition: u8) {
        self.sync();
        self.update(|st| st.arrive(transition));
    }

    /// Press (`down`) or release the key whose down-transition code is `key`.
    pub fn press(&self, key: u8, down: bool) {
        self.key(if down { key } else { key | code::KEY_UP });
    }

    /// Whether the key whose down-transition code is `key` is down.
    #[must_use]
    pub fn held(&self, key: u8) -> bool {
        let n = (key & !code::KEY_UP) >> 1;
        self.state.lock().held[usize::from(n) / 8] & (1 << (n % 8)) != 0
    }

    /// How many commands the computer has completed since power-on.
    #[must_use]
    pub fn commands(&self) -> u64 {
        self.state.lock().commands
    }

    /// The last command the keyboard received, and the last response it sent.
    #[must_use]
    pub fn last_exchange(&self) -> (u8, u8) {
        let st = self.state.lock();
        (st.last_command, st.last_response)
    }

    /// How many transitions are waiting to be asked for.
    #[must_use]
    pub fn queued(&self) -> usize {
        self.state.lock().queue.len()
    }

    /// Whether the keyboard is pulling `(clock, data)` low.
    #[must_use]
    pub fn lines(&self) -> (bool, bool) {
        self.state.lock().pins()
    }

    /// Ticks simulated — microseconds, on a board that clocks it at 1 MHz.
    #[must_use]
    pub fn ticks(&self) -> u64 {
        self.ticks.load(Ordering::Relaxed)
    }

    /// Run the keyboard until `target` ticks have passed in total.
    ///
    /// One protocol step at a time, driving the lines between steps with no
    /// lock held: an edge reaches the VIA, which is a device with a lock of its
    /// own, and each edge has to arrive before the next is computed.
    pub fn advance_to(&self, target: u64) {
        loop {
            let moved = {
                let mut st = self.state.lock();
                let due = st.next_event();
                if due != NO_EVENT && due <= target {
                    let before = st.pins();
                    st.ticks = st.ticks.max(due);
                    // A turnaround that reached its deadline is a computer that
                    // never said it was ready. Give up on it rather than hold
                    // the cable: the computer's own half-second timeout is what
                    // recovers the link.
                    if matches!(st.phase, Phase::Turnaround { .. }) {
                        st.phase = Phase::Idle;
                        st.next = NO_EVENT;
                        st.clk_low = false;
                        st.data_low = false;
                    } else {
                        st.step();
                    }
                    self.publish(&st);
                    Some(before != st.pins())
                } else {
                    if target > st.ticks {
                        st.ticks = target;
                    }
                    self.publish(&st);
                    None
                }
            };
            let Some(moved) = moved else { break };
            if moved {
                self.refresh();
            }
            // Always, not only when a pin moved: the step that ends a
            // transaction puts the keyboard back to watching the line, and the
            // computer may have been holding it low for the next command since
            // before the last bit went out. A transaction that ended on a one
            // moves nothing as it finishes, so a check conditional on movement
            // is exactly the one that misses it.
            self.look_at_the_line();
        }
    }

    fn publish(&self, st: &State) {
        self.ticks.store(st.ticks, Ordering::Relaxed);
        self.next_event.store(st.next_event(), Ordering::Relaxed);
    }

    /// Apply `f`, then drive whatever moved — outside the lock.
    fn update(&self, f: impl FnOnce(&mut State)) {
        let moved = {
            let mut st = self.state.lock();
            let before = st.pins();
            f(&mut st);
            self.publish(&st);
            before != st.pins()
        };
        if moved {
            self.refresh();
        }
    }

    /// Drive both output stages to what the state says, holding no lock.
    fn refresh(&self) {
        let (clk, data) = self.state.lock().pins();
        let out = self.out.lock().clone();
        if let Some(src) = &out.clk {
            src.drive(if clk { Drive::Low } else { Drive::HiZ });
        }
        if let Some(src) = &out.data {
            src.drive(if data { Drive::Low } else { Drive::HiZ });
        }
    }

    /// Read the data line after letting go of it.
    ///
    /// A resolved net only delivers a *change*, and the computer that was
    /// already holding the line low while the keyboard held it too leaves it
    /// low when the keyboard lets go: no change, no delivery. So the keyboard
    /// asks. It is also how the level a transaction ends on is seen at all —
    /// the next command's "ready" may already be on the line.
    fn look_at_the_line(&self) {
        let source = self.out.lock().data.clone();
        let Some(source) = source else {
            return;
        };
        let low = source.net_level().is_low();
        // Unconditionally, rather than only on a change: `sense` is what starts
        // a transaction, and a level that was already there when the keyboard
        // went back to watching for it is not a change.
        self.update(|st| st.sense(low));
    }

    fn sync(&self) {
        let handle = self.lazy.lock().clone();
        if let Some(handle) = handle {
            // A refusal is this keyboard's own catch-up further up the stack —
            // it drove a line, and the net came straight back here. The state
            // is already at the instant of that drive.
            let _ = handle.sync(AccessKind::Guest);
        }
    }

    fn power_on(&self) {
        self.update(|st| {
            let (ticks, model, line_low) = (st.ticks, st.model, st.line_low);
            *st = State::power_on(ticks, model);
            st.line_low = line_low;
        });
    }
}

/// The build's named Macintosh keyboards.
///
/// The same shape as `amiga::keyboard::keys`: a name travels from the machine
/// file into the device constructor, and both ends resolve it against the
/// build's [`HostObjects`](crate::core::hosts::HostObjects).
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
    pub const KIND: HostKind = HostKind::door("mac-keyboard", make_sink);

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

    /// Bytes per recorded movement: one, the Guide's transition code with bit 7
    /// set for a release.
    pub const RECORD_BYTES: usize = 1;

    /// The channel the keyboard called `name` is typed on:
    /// `mac-keyboard:keyboard`.
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
        Arc::new(FnSink::new("mac-keyboard", move |payload: &[u8]| {
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

/// The data line as something a wire drives.
#[derive(Debug)]
struct DataPin {
    keyboard: Arc<Keyboard>,
}

impl WireSink for DataPin {
    fn set_level(&self, _src: WireId, _line: u32, level: Level) {
        self.keyboard.sync();
        let low = level.is_low();
        self.keyboard.update(|st| st.sense(low));
    }
}

/// The `mac.keyboard` device.
#[derive(Debug)]
pub struct MacKeyboard {
    keyboard: Arc<Keyboard>,
    pin: Arc<DataPin>,
}

impl MacKeyboard {
    /// Validate `props` and open the keyboard they name.
    ///
    /// # Errors
    ///
    /// [`Error::Property`] for a property this class does not know or a model
    /// number outside Table 7-4's 1-8, or [`Error::Config`] if the name is held
    /// by something that is not a keyboard.
    pub fn new(props: &Props) -> Result<MacKeyboard> {
        let mut r = props.reader();
        let port = r.or_str("keys", DEFAULT_KEYBOARD_PORT)?.to_string();
        let model = r.or("model", 1u64)?;
        r.finish()?;
        if !(1..=8).contains(&model) {
            return Err(Error::Property(format!(
                "property `model`: Table 7-4 gives the keyboard model number as 1-8, not {model}"
            )));
        }
        let keyboard = keys::attach(props, &port)?;
        Ok(MacKeyboard::with(keyboard))
    }

    /// A device around a keyboard the caller already holds.
    #[must_use]
    pub fn with(keyboard: Arc<Keyboard>) -> MacKeyboard {
        let pin = Arc::new(DataPin {
            keyboard: Arc::clone(&keyboard),
        });
        MacKeyboard { keyboard, pin }
    }

    /// The keyboard.
    #[must_use]
    pub fn keyboard(&self) -> &Arc<Keyboard> {
        &self.keyboard
    }
}

impl Device for MacKeyboard {
    fn class(&self) -> &'static DeviceClass {
        &CLASS
    }

    fn realize(&self, _ctx: &mut RealizeCtx<'_>) -> Result<()> {
        Ok(())
    }

    fn reset(&self, kind: ResetKind) {
        // The keyboard is its own computer on the end of a cable and is powered
        // for as long as the Macintosh is: only a power cycle restarts it. A
        // Model Number command resets it too, which is the protocol's own path
        // and not this one.
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
            Phase::Starting => w.write_u8(1)?,
            Phase::Command { got, left, step } => {
                w.write_u8(2)?;
                w.write_u8(got)?;
                w.write_u8(left)?;
                w.write_u8(step)?;
            }
            Phase::Turnaround { command, deadline } => {
                w.write_u8(3)?;
                w.write_u8(command)?;
                w.write_u64(deadline)?;
            }
            Phase::Answering { byte } => {
                w.write_u8(4)?;
                w.write_u8(byte)?;
            }
            Phase::Response { bits, left, step } => {
                w.write_u8(5)?;
                w.write_u8(bits)?;
                w.write_u8(left)?;
                w.write_u8(step)?;
            }
            Phase::Trailing => w.write_u8(6)?,
        }
        for v in [st.clk_low, st.data_low, st.line_low, st.latched] {
            w.write_bool(v)?;
        }
        w.write_bytes(&st.queue.iter().copied().collect::<Vec<u8>>())?;
        w.write_bytes(&st.held)?;
        w.write_u8(st.model)?;
        w.write_u64(st.commands)?;
        w.write_u8(st.last_command)?;
        w.write_u8(st.last_response)
    }

    fn load(&self, r: &mut ChunkReader<'_>) -> Result<()> {
        let bad = |what: &str| Error::State(format!("{CLASS_NAME}: {what}"));
        let mut st = State::power_on(0, 1);
        st.ticks = r.read_u64()?;
        st.next = r.read_u64()?;
        let bits_left = |left: u8, step: u8| -> Result<()> {
            if !(1..=BITS).contains(&left) || step > 2 {
                return Err(bad("a transmission step out of range"));
            }
            Ok(())
        };
        st.phase = match r.read_u8()? {
            0 => Phase::Idle,
            1 => Phase::Starting,
            2 => {
                let (got, left, step) = (r.read_u8()?, r.read_u8()?, r.read_u8()?);
                bits_left(left, step)?;
                Phase::Command { got, left, step }
            }
            3 => Phase::Turnaround {
                command: r.read_u8()?,
                deadline: r.read_u64()?,
            },
            4 => Phase::Answering { byte: r.read_u8()? },
            5 => {
                let (bits, left, step) = (r.read_u8()?, r.read_u8()?, r.read_u8()?);
                bits_left(left, step)?;
                Phase::Response { bits, left, step }
            }
            6 => Phase::Trailing,
            _ => return Err(bad("an unknown protocol phase")),
        };
        st.clk_low = r.read_bool()?;
        st.data_low = r.read_bool()?;
        st.line_low = r.read_bool()?;
        st.latched = r.read_bool()?;
        let queue = r.read_bytes()?;
        if queue.len() > TYPE_AHEAD {
            return Err(bad("a type-ahead buffer longer than the keyboard's"));
        }
        st.queue = queue.iter().copied().collect();
        let held = r.read_bytes()?;
        if held.len() != st.held.len() {
            return Err(bad("a held-key map of the wrong size"));
        }
        st.held.copy_from_slice(held);
        st.model = r.read_u8()?.clamp(1, 8);
        st.commands = r.read_u64()?;
        st.last_command = r.read_u8()?;
        st.last_response = r.read_u8()?;
        if st.next != NO_EVENT && st.next <= st.ticks && st.phase != Phase::Idle {
            // An event that is not in the future would stall catch-up.
            st.next = st.ticks + 1;
        }
        self.keyboard.update(|now| *now = st);
        Ok(())
    }

    fn connect(&self, port: &str, source: WireSource) -> Result<()> {
        {
            let mut out = self.keyboard.out.lock();
            match port {
                CLK_PIN => out.clk = Some(source),
                DATA_PIN => out.data = Some(source),
                _ => {
                    return Err(Error::Config {
                        at: port.to_string(),
                        message: format!(
                            "a Macintosh keyboard drives `{CLK_PIN}` and `{DATA_PIN}`"
                        ),
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
        (port == DATA_PIN).then(|| SinkPin {
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

impl Instance for MacKeyboard {}

/// The `mac.keyboard` device class.
pub static CLASS: DeviceClass = DeviceClass {
    name: CLASS_NAME,
    version: STATE_VERSION,
    summary: "a Macintosh Plus keyboard: the Guide's clock/data protocol with its four \
              commands, keyed from the host",
    properties: &[
        PropertySpec {
            name: "keys",
            kind: ValueKind::Str,
            required: false,
            summary: "the host keyboard port it is typed on (default `keyboard`)",
        },
        PropertySpec {
            name: "model",
            kind: ValueKind::Uint,
            required: false,
            summary: "the model number it answers Model Number with, 1-8 (default 1)",
        },
    ],
    construct: |props| Ok(Box::new(MacKeyboard::new(props)?)),
};

/// Add [`CLASS`] to a registry.
///
/// # Errors
///
/// [`crate::core::Error::Config`] if something already claimed
/// the name.
pub fn register(registry: &mut crate::core::Registry) -> Result<()> {
    registry.add(&CLASS)
}

/// Bind [`CLASS`] into the machine graph.
///
/// # Errors
///
/// [`crate::core::Error::Config`] if the class is already bound.
pub fn bind(bindings: &mut crate::machine::Bindings) -> Result<()> {
    bindings.bind(CLASS_NAME, |props| Ok(Arc::new(MacKeyboard::new(props)?)))
}

/// What the validator should know about `mac.keyboard`.
#[must_use]
pub fn schema() -> ClassSchema {
    ClassSchema::new(CLASS_NAME)
        .prop(PropSchema::new("keys", ValueKind::Str))
        .prop(PropSchema::new("model", ValueKind::Uint).range(1, 8))
        .port(CLK_PIN, PortDir::Out)
        .port(DATA_PIN, PortDir::InOut)
}

/// A short name for a byte on the cable, for a monitor or a failing test.
#[must_use]
pub fn describe(byte: u8) -> String {
    match byte {
        code::INQUIRY => String::from("Inquiry"),
        code::INSTANT => String::from("Instant"),
        code::MODEL_NUMBER => String::from("Model Number"),
        code::TEST => String::from("Test"),
        code::NULL => String::from("Null"),
        code::ACK => String::from("ACK"),
        code::NAK => String::from("NAK"),
        code::KEYPAD => String::from("Keypad"),
        b if b & code::KEY_UP != 0 => format!("${:02X} up", b & !code::KEY_UP),
        b => format!("${b:02X} down"),
    }
}

#[cfg(test)]
mod tests;
