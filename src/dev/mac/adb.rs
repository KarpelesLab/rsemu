//! `mac.adb`: the Apple Desktop Bus transceiver a compact Macintosh talks to
//! through the VIA's shift register, and the keyboard and mouse on the bus.
//!
//! ```text
//!   osc    adbclk = 1000000 Hz
//!   object adb "mac.adb" { clock = adbclk }
//!
//!   wire adb.clk  -> via.cb1 { pull = "up" }   # the transceiver's clock
//!   wire adb.data -> via.cb2 { pull = "up" }   # driven from both ends
//!   wire via.cb2  -> adb.data
//!   wire via.pb4  -> adb.st0 { pull = "up" }   # the two state lines
//!   wire via.pb5  -> adb.st1 { pull = "up" }
//!   wire adb.int  -> via.pb3 { pull = "up" }   # open collector, asserted low
//! ```
//!
//! # What this is, and what it is not
//!
//! A Macintosh with ADB does not drive the bus itself. A microcontroller — the
//! transceiver — sits between the computer and the four-wire connector and does
//! the bus timing; the computer hands it a byte at a time over a **byte-wide
//! link made of the VIA's shift register and two state lines**. This models
//! that link and the bus behind it. There is no 68HC05 core here and no
//! transceiver firmware image, and none is wanted: the link is what the
//! computer sees.
//!
//! # How the link was established — by measurement, not by a table
//!
//! `CLAUDE.md` forbids reading any Macintosh emulator's source and forbids
//! disassembling Apple's ROM, and this file had no document that states the
//! link's protocol. So it was built the way `docs/platforms/mac-plus.md`
//! describes: a transparent tap over the VIA recording every register access a
//! real Macintosh Classic ROM makes, and reasoning about what the hardware must
//! do to satisfy it. What that measurement says, in the ROM's own order:
//!
//! ```text
//!   DDRB  = $f7     PB3 is the only input: the transceiver's attention line
//!   ACR   = $1c     mode 111 — shift out under an external clock on CB1
//!   SR    = <byte>  the byte the computer wants the transceiver to have
//!   ORB   = $4f     PB5 = 0, PB4 = 0 — the state lines, both low
//!   ...             and then nothing at all, for ever
//! ```
//!
//! Two things follow with no room for doubt. **The clock is the
//! transceiver's**, because the computer selected the 6522's only shift-out
//! mode that takes its clock from CB1 as an input (the same argument
//! [`super::keyboard`] makes for the Plus's keyboard, and the same two modes).
//! And **a write to the state lines is what starts a transfer**, because the
//! state write is the last thing the computer does before it waits.
//!
//! The rest — which state means what, and how many bytes each transaction is —
//! was settled by answering the ROM and watching what it did next. Those
//! findings are commented in the source, at `Phase` and `State::state_changed`, each
//! marked as a measurement rather than a citation.
//!
//! # The bus, and what is on it
//!
//! The command byte's shape is the Apple Desktop Bus one — four bits of device
//! address, two of command, two of register. This model carries a **keyboard at
//! address 2** and a **mouse at address 3**, which are the addresses a Macintosh
//! assigns them, and answers Talk of register 3 (the identity register) and
//! register 0 (the data register). Everything else is answered as "no device",
//! which is what the bus looks like with nothing plugged in — and a Macintosh
//! boots to the desktop with an unplugged keyboard, so that path has to work.
//!
//! # Time
//!
//! **One tick of this device's `clock` is one microsecond**, as
//! [`super::keyboard`]'s is, because every interval here is in microseconds. A
//! board gives it a 1 MHz oscillator.

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
use crate::core::wire::{Drive, FanIn, Level, Resolve, WireId, WireSink, WireSource};
use crate::machine::Instance;
use crate::machine::validate::{ClassSchema, PortDir, PropSchema};

/// The class name a machine file writes.
pub const CLASS_NAME: &str = "mac.adb";

/// The snapshot chunk version. Bump with the encoding, never on its own.
pub const STATE_VERSION: u32 = 1;

/// The transceiver's clock output, which a Macintosh wires to the VIA's CB1.
pub const CLK_PIN: &str = "clk";
/// The data line, the VIA's CB2. Driven from both ends.
pub const DATA_PIN: &str = "data";
/// The attention line, which a Macintosh wires to the VIA's `PB3`. Open
/// collector, asserted low.
pub const INT_PIN: &str = "int";
/// State line 0, the VIA's `PB4`.
pub const ST0_PIN: &str = "st0";
/// State line 1, the VIA's `PB5`.
pub const ST1_PIN: &str = "st1";

/// A tick no event is scheduled for.
const NO_EVENT: u64 = u64::MAX;

/// How many bits one transfer over the link is.
const BITS: u8 = 8;

// -- the link's timing -------------------------------------------------------
//
// **Inferred**, and the inference is stated rather than dressed up as a
// citation: no document here gives the rate the transceiver clocks the VIA at,
// and the measurement cannot give it either — the ROM waits for the shift
// register's interrupt and does not care how long it took. So these are chosen
// to be slow enough to be a real microcontroller and fast enough to be
// invisible against the ROM's own timeouts, and the one thing that is measured
// is that Apple's ROM accepts them.

/// How long one bit of the link takes.
pub const BIT_TICKS: u64 = 50;
/// How much of that the clock is low for.
pub const LOW_TICKS: u64 = 25;
/// How long after the computer moves the state lines the first clock falls.
pub const START_TICKS: u64 = 40;
/// How long the transceiver holds its attention line low once it has something
/// to say.
pub const ATTENTION_TICKS: u64 = 200;

/// The command byte's fields, and the two device addresses a Macintosh uses.
///
/// The Apple Desktop Bus command byte is four bits of device address, two bits
/// of command and two bits of register. Every one of these was *confirmed*
/// against the commands a real Macintosh Classic ROM issues — it resets the
/// bus, then walks the addresses asking each for register 3 — rather than taken
/// on trust; see the module docs.
pub mod cmd {
    /// Talk: the device sends the register back. Bits 3-2 of the command byte.
    pub const TALK: u8 = 0b11;
    /// Listen: the device takes the two bytes that follow.
    pub const LISTEN: u8 = 0b10;
    /// Flush, and the reserved encodings. Bits 3-2 zero.
    pub const RESERVED: u8 = 0b00;

    /// The whole byte that resets every device on the bus.
    pub const RESET: u8 = 0x00;

    /// Where a Macintosh's keyboard ends up.
    pub const ADDR_KEYBOARD: u8 = 2;
    /// And its mouse.
    pub const ADDR_MOUSE: u8 = 3;

    /// The address field.
    #[must_use]
    pub const fn address(byte: u8) -> u8 {
        byte >> 4
    }

    /// The command field.
    #[must_use]
    pub const fn command(byte: u8) -> u8 {
        (byte >> 2) & 3
    }

    /// The register field.
    #[must_use]
    pub const fn register(byte: u8) -> u8 {
        byte & 3
    }
}

/// One device on the bus.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct BusDevice {
    /// Where it answers, 1 to 15. Zero is "not present".
    address: u8,
    /// The address it powers up at and goes back to on a bus reset.
    default_address: u8,
    /// Register 3's low byte: the device handler identifier.
    handler: u8,
    /// Whether it has something to report — a key movement or a mouse
    /// movement — which is what makes the transceiver ask for attention.
    pending: bool,
    /// Register 0, two bytes, as the device would send it.
    data: [u8; 2],
}

impl BusDevice {
    /// The Apple Standard Keyboard: address 2, handler 1.
    ///
    /// The handler identifier is the one thing here a document would settle and
    /// the measurement does not: the ROM asks for register 3 and does not act
    /// differently on what comes back, so this is **inferred** to be the
    /// standard keyboard's 1.
    const fn keyboard() -> BusDevice {
        BusDevice {
            address: cmd::ADDR_KEYBOARD,
            default_address: cmd::ADDR_KEYBOARD,
            handler: 1,
            pending: false,
            // "No key" in both halves. An ADB keyboard reports two key
            // transitions per Talk 0 and pads an unused half with $FF.
            data: [0xff, 0xff],
        }
    }

    /// The mouse: address 3, handler 1.
    const fn mouse() -> BusDevice {
        BusDevice {
            address: cmd::ADDR_MOUSE,
            default_address: cmd::ADDR_MOUSE,
            handler: 1,
            pending: false,
            // Button up, no movement: bit 7 of the first byte is the button
            // (1 = released) and the seven bits below each byte are a signed
            // delta.
            data: [0x80, 0x80],
        }
    }

    /// Register 3 as a Talk of it answers: the address in the low nibble of the
    /// high byte with service-request enable above it, and the handler
    /// identifier below.
    fn register3(&self) -> [u8; 2] {
        [0x60 | (self.address & 0x0f), self.handler]
    }
}

/// Where the transceiver is in a transaction.
///
/// **Measured, not cited.** The computer writes the two state lines and waits;
/// what each value means was settled by answering one interpretation and
/// watching whether the ROM went on. The values are `(ST1, ST0)` as the VIA's
/// `PB5`/`PB4` carry them, and `0` is the state the ROM's very first write
/// selects — `ORB = $4f`, both lines low — which is what makes "0 is the
/// command byte" the only reading that fits.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Phase {
    /// Nothing is happening; the state lines are being watched.
    Idle,
    /// A transfer is due to start.
    Starting { out: bool },
    /// Clocking a byte across. `bits` is what is left of an outgoing byte, most
    /// significant first, or what has been gathered of an incoming one; `step`
    /// is 0 for "the clock falls next" and 1 for "it rises next".
    Xfer {
        out: bool,
        bits: u8,
        left: u8,
        step: u8,
    },
    /// The byte is across and the computer has yet to move the state lines.
    Between,
}

/// Which byte of a transaction is next.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Slot {
    /// The command byte, which the computer sends.
    Command,
    /// The first data byte.
    Even,
    /// The second.
    Odd,
}

#[derive(Debug, Clone, PartialEq, Eq)]
struct State {
    /// Ticks of the transceiver's own clock. One is a microsecond.
    ticks: u64,
    /// The tick the next step falls on, or [`NO_EVENT`].
    next: u64,
    phase: Phase,
    /// The state lines as the computer last drove them, `ST1 << 1 | ST0`.
    lines: u8,
    /// Whether the transceiver's clock stage is pulling CB1 low.
    clk_low: bool,
    /// Whether its data stage is pulling CB2 low.
    data_low: bool,
    /// Whether the data line was low when last seen.
    line_low: bool,
    /// The data bit the line carried while the clock was last low. Frozen
    /// there for the reason [`super::keyboard`]'s `command_step` spells out:
    /// the computer can take CB2 back in no guest time at all once its shift
    /// register's count completes.
    latched: bool,
    /// Movement the mouse has counted and nobody has read yet, in the seven
    /// bits two's complement its register 0 carries. See [`State::mouse`].
    owed_x: i16,
    owed_y: i16,
    /// Whether the attention line is being pulled low.
    int_low: bool,
    /// The **unsolicited** byte going across, or zero: the transceiver
    /// clocking the idle computer's shift register because a device has
    /// something to say, rather than a byte of a transaction the computer
    /// started.
    ///
    /// # Why this exists, and it is a measurement
    ///
    /// Pulling the attention line is not enough on its own, and the boot does
    /// not show it because nothing moves a mouse during a boot. Once Mac OS
    /// 6.0.8 is up, a Macintosh Classic leaves the link like this — read
    /// through the VIA with a debugger, so nothing here moved a flag:
    ///
    /// ```text
    ///   ORB  $7f    PB5:PB4 = 1:1, which is state 3: idle
    ///   DDRB $f7    PB3, the attention line, is the only input
    ///   ACR  $0c    mode 011 — shift *in* under an external clock on CB1
    ///   IER  $a7    and the shift-register interrupt is enabled
    /// ```
    ///
    /// and then it touches `ORB`, `SR` and `ACR` **not once** for the next
    /// virtual minute, whatever the attention line does. Holding that line low
    /// for ten thousand times as long changes nothing. The computer is not
    /// polling a pin; it is sitting in shift-in mode waiting for a byte.
    ///
    /// So the transceiver clocks one at it. With that, the same machine runs
    /// **seventy-two ADB transactions a virtual second** — `ORB = $4f` with
    /// `SR = $3c`, Talk 0 of address 3, then states 1, 2 and 3 — and the
    /// pointer moves. The differential says the byte's *value* is not what
    /// matters: `$3C` and `$FF` both start it, and the computer issues its own
    /// `$3C` either way. What matters is that a byte arrives, because that is
    /// what sets the VIA's shift-register flag.
    ///
    /// **What the byte is** was settled by the same instrument, one stage at a
    /// time — count the reports, the polls, the answers and the bytes that
    /// leave, and the one that drops is the answer. Here none of them dropped:
    /// one report, one poll, one device answering it, and ten bytes out — and
    /// **every one of the ten was the pull-up**. A byte with nothing behind it
    /// wakes the computer and then leaves it retrying: the trace shows it
    /// driving states 1 and 2 *five times over*, reading `$FF` each time,
    /// before giving up and starting a transaction of its own that it then
    /// abandons.
    ///
    /// So the computer does not treat an unsolicited byte as a doorbell to
    /// answer with a transaction. It treats it as the **command byte of a
    /// reply it must now collect**, and goes straight to states 1 and 2 for
    /// the two data bytes. The transceiver therefore polls the device itself
    /// and hands the computer the command it used, with the reply already
    /// waiting behind it.
    ///
    /// One thing is still **inferred**: that the byte is that command rather
    /// than some status of the transceiver's own. What is measured is that a
    /// byte with a reply behind it works, a byte without one does not, and
    /// that the computer's own next act is to read two data bytes.
    unsolicited: u8,
    /// The tick the attention line may go back up on.
    int_until: u64,
    /// The command byte of the transaction in progress.
    command: u8,
    /// Which byte comes next.
    slot: Slot,
    /// The two bytes a Talk will hand over, most significant first.
    reply: [u8; 2],
    /// Whether the addressed device answered the command at all.
    answered: bool,
    /// The two bytes a Listen has gathered.
    heard: [u8; 2],
    /// The devices on the bus.
    devices: [BusDevice; 2],
    /// Key transitions waiting to be reported: an ADB key code with bit 7 set
    /// for a release.
    keys: VecDeque<u8>,
    /// How many command bytes the computer has sent — one per transaction.
    transactions: u64,
    /// Four places one report has to pass, counted separately.
    ///
    /// The instrument `tests/mac_plus.rs` used to settle where a mouse's
    /// counts were going, on the other Macintosh: count the *stages* rather
    /// than the ends, and the one that drops is the answer. There, all four
    /// turning out equal was itself the finding.
    ///
    /// In order: a report handed to the transceiver, a Talk of a device's
    /// register 0, one of those a device answered, and a byte the transceiver
    /// actually shifted out to the computer.
    counts: Counters,
    /// The last command byte, and the last byte sent each way, for the same.
    last_command: u8,
    last_sent: u8,
    last_received: u8,
}

/// Where one input report can go missing between the host and the computer.
///
/// Four counters, one per stage, so a rate can be quoted rather than a verdict
/// (`tests/mac_plus.rs` is where this instrument was first earned).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct Counters {
    /// Reports the host handed the transceiver: a movement or a key.
    pub reports: u64,
    /// Talks of a device's register 0 the computer issued.
    pub polls: u64,
    /// How many of those a device answered.
    pub answered: u64,
    /// Bytes the transceiver shifted out to the computer, in any transfer.
    pub sent: u64,
    /// How many of those were the pulled-up bus rather than a device's:
    /// `$FF`, which is what a transfer with nobody answering puts on the wire.
    pub sent_empty: u64,
    /// The last four bytes shifted out, oldest in the high byte.
    pub last_four: u32,
}

/// How many key transitions the keyboard holds while nobody is asking.
pub const TYPE_AHEAD: usize = 16;

impl State {
    fn power_on(ticks: u64) -> State {
        State {
            ticks,
            next: NO_EVENT,
            phase: Phase::Idle,
            // Both lines pulled up until the computer drives them, which is
            // state 3 — idle, and the right thing for a transceiver that has
            // not been spoken to.
            lines: 3,
            clk_low: false,
            data_low: false,
            line_low: false,
            latched: false,
            owed_x: 0,
            owed_y: 0,
            int_low: false,
            unsolicited: 0,
            int_until: 0,
            command: 0,
            slot: Slot::Command,
            reply: [0xff, 0xff],
            answered: false,
            heard: [0, 0],
            devices: [BusDevice::keyboard(), BusDevice::mouse()],
            keys: VecDeque::new(),
            transactions: 0,
            counts: Counters::default(),
            last_command: 0,
            last_sent: 0,
            last_received: 0,
        }
    }

    /// What the three output stages are pulling, as a triple.
    fn pins(&self) -> (bool, bool, bool) {
        (self.clk_low, self.data_low, self.int_low)
    }

    /// The level on the data line changed.
    fn sense(&mut self, low: bool) {
        self.line_low = low;
        // While the clock is low the line carries the bit the computer's shift
        // register presented on the falling edge.
        if let Phase::Xfer {
            out: false,
            step: 1,
            ..
        } = self.phase
        {
            self.latched = !low;
        }
    }

    /// One of the two state lines moved. `lines` is `ST1 << 1 | ST0`.
    ///
    /// **This is the measurement the whole file turns on.** The computer's last
    /// act before it waits is a write to `ORB` that moves these two lines, so
    /// this is where a transfer begins. Which value means what:
    ///
    /// * **0** — the command byte, computer to transceiver. The ROM's first
    ///   write is `ORB = $4f`, both lines low, immediately after loading `SR`
    ///   with a byte and putting the shift register in its **out** mode, so the
    ///   byte is going that way and this is where a transaction starts.
    /// * **1** and **2** — the two data bytes, in whichever direction the
    ///   command asked for. A Talk sends; a Listen receives.
    /// * **3** — idle. The transceiver does nothing and the transaction is
    ///   over.
    fn state_changed(&mut self, lines: u8) {
        if lines == self.lines {
            return;
        }
        self.lines = lines;
        // The computer has taken the link over, so an unsolicited byte that was
        // still going across is abandoned — and the flag has to go with it, or
        // the *transaction's* last transfer would end in the wrong phase.
        self.unsolicited = 0;
        match lines {
            0 => {
                self.slot = Slot::Command;
                self.phase = Phase::Starting { out: false };
                self.next = self.ticks + START_TICKS;
            }
            1 | 2 => {
                self.slot = if lines == 1 { Slot::Even } else { Slot::Odd };
                // A Talk hands the device's register over; anything else takes
                // the computer's byte. A command nothing answered hands over
                // nothing at all, and the computer's own timeout is what
                // recovers — which is what an empty bus looks like.
                // A Talk hands the device's register over and anything else
                // takes the computer's byte. The transfer happens either way,
                // **including when no device answered**: the transceiver simply
                // drives nothing and the computer's shift register latches the
                // pulled-up bus, which is `$FF`. That is what an absent device
                // looks like and it is measured rather than assumed — a
                // transceiver that stayed quiet instead left the ROM waiting on
                // a shift-register interrupt that never came, at the first
                // address of its scan.
                let talk = cmd::command(self.command) == cmd::TALK;
                self.phase = Phase::Starting { out: talk };
                self.next = self.ticks + START_TICKS;
            }
            _ => {
                if self.phase != Phase::Idle {
                    self.finish();
                }
                self.phase = Phase::Idle;
                self.next = NO_EVENT;
                self.announce_if_owed();
            }
        }
    }

    /// A transaction ended at the idle state: apply whatever it asked for.
    fn finish(&mut self) {
        if cmd::command(self.command) == cmd::LISTEN && cmd::register(self.command) == 3 {
            // Listen register 3 is how a Macintosh moves a device off a
            // colliding address. The low byte is the handler identifier and the
            // low nibble of the high byte the new address.
            let addr = self.heard[0] & 0x0f;
            let handler = self.heard[1];
            if let Some(d) = self
                .devices
                .iter_mut()
                .find(|d| d.address == cmd::address(self.command))
            {
                if addr != 0 {
                    d.address = addr;
                }
                // $FE and $FF are "keep the handler you have" in the register's
                // own encoding; anything else is a request to change it, and
                // this model has one handler per device so it keeps its own.
                let _ = handler;
            }
        }
    }

    /// Choose what a Talk answers, or clear `answered` if nothing does.
    fn begin_command(&mut self, byte: u8) {
        // One per command byte, which is one per transaction. Counting the
        // *end* of a transaction instead counts almost nothing: a Macintosh
        // Classic ROM's bus scan goes state 0 -> 1 -> 2 -> 0 for sixteen
        // addresses running and only reaches the idle state once, at the end.
        self.transactions = self.transactions.wrapping_add(1);
        self.last_received = byte;
        self.apply_command(byte);
    }

    /// Work out what a command byte means and which device answers it.
    ///
    /// Split out of [`State::begin_command`] because the transceiver issues
    /// one **itself** when a device has something to say: it autopolls the
    /// device and then hands the computer the command it used
    /// ([`State::ask_for_attention`]). That is a command nothing received, so
    /// it must not move `last_received` or the count of bytes the computer
    /// sent.
    fn apply_command(&mut self, byte: u8) {
        self.command = byte;
        self.last_command = byte;
        self.heard = [0, 0];
        if byte == cmd::RESET {
            for d in &mut self.devices {
                d.address = d.default_address;
                d.pending = false;
            }
            self.keys.clear();
            self.answered = false;
            return;
        }
        let addr = cmd::address(byte);
        let register = cmd::register(byte);
        self.answered = false;
        if cmd::command(byte) == cmd::TALK && register == 0 {
            self.counts.polls += 1;
        }
        if cmd::command(byte) != cmd::TALK {
            // A Listen is answered by whichever device holds the address; a
            // reserved encoding by nobody.
            self.answered =
                cmd::command(byte) == cmd::LISTEN && self.devices.iter().any(|d| d.address == addr);
            return;
        }
        let keys = &mut self.keys;
        let Some(d) = self.devices.iter_mut().find(|d| d.address == addr) else {
            return;
        };
        match register {
            3 => {
                self.reply = d.register3();
                self.answered = true;
            }
            0 if d.address == cmd::ADDR_KEYBOARD => {
                // Counted here and at the mouse's arm below rather than at the
                // top, because `answered` is also what a Talk of register 3
                // and a Listen set.
                // An ADB keyboard answers Talk 0 with two key transitions and
                // says nothing at all when it has none: a device with no data
                // does not drive the bus, and the transceiver reports that by
                // not sending a byte.
                let first = keys.pop_front();
                if let Some(first) = first {
                    self.reply = [first, keys.pop_front().unwrap_or(0xff)];
                    self.answered = true;
                    self.counts.answered += 1;
                }
                d.pending = !keys.is_empty();
            }
            0 if d.pending => {
                self.reply = d.data;
                d.pending = false;
                self.answered = true;
                self.counts.answered += 1;
                if d.default_address == cmd::ADDR_MOUSE {
                    // What was owed has now been handed over.
                    self.owed_x = 0;
                    self.owed_y = 0;
                }
            }
            _ => {}
        }
    }

    /// A byte arrived from the computer.
    fn took(&mut self, byte: u8) {
        match self.slot {
            Slot::Command => self.begin_command(byte),
            Slot::Even => {
                self.heard[0] = byte;
                self.last_received = byte;
            }
            Slot::Odd => {
                self.heard[1] = byte;
                self.last_received = byte;
            }
        }
    }

    /// Which byte a Talk sends in the slot the state lines selected.
    ///
    /// `$FF` when nothing answered: a device that has no data does not drive
    /// the bus, so the computer latches the pull-ups. It is also how the
    /// computer tells an empty address from an occupied one, because no
    /// device's register 3 reads `$FF $FF`.
    fn to_send(&self) -> u8 {
        // The transceiver's own command byte, on its way to an idle computer.
        if self.unsolicited != 0 {
            return self.unsolicited;
        }
        if !self.answered {
            return 0xff;
        }
        match self.slot {
            Slot::Even | Slot::Command => self.reply[0],
            Slot::Odd => self.reply[1],
        }
    }

    /// One step of the link, at `self.next`.
    fn step(&mut self) {
        match self.phase {
            Phase::Idle | Phase::Between => self.next = NO_EVENT,
            Phase::Starting { out } => {
                let bits = if out { self.to_send() } else { 0 };
                if out {
                    self.last_sent = bits;
                    if !self.answered {
                        self.counts.sent_empty += 1;
                    }
                    self.counts.last_four = (self.counts.last_four << 8) | u32::from(bits);
                }
                self.phase = Phase::Xfer {
                    out,
                    bits,
                    left: BITS,
                    step: 0,
                };
                self.step();
            }
            Phase::Xfer {
                out,
                bits,
                left,
                step,
            } => self.xfer_step(out, bits, left, step),
        }
    }

    /// One bit across the link.
    ///
    /// The clock is the transceiver's, and [`super::via`]'s external shift-out
    /// mode presents a bit on the **falling** edge and completes its count on
    /// the **rising** one, while its shift-in mode latches on the rising edge.
    /// So one bit is: pull the clock low, hold, let it go.
    fn xfer_step(&mut self, out: bool, bits: u8, left: u8, step: u8) {
        if step == 0 {
            if out {
                // Open collector: a zero pulls the line down, a one lets go.
                // The bit has to be on the line before the rising edge the
                // computer latches on, and putting it there with the falling
                // edge gives it the whole low half to settle in.
                self.data_low = bits & 0x80 == 0;
            }
            self.clk_low = true;
            // The level *before* the computer answers this edge; a shift-out
            // presenting its bit overwrites it through `sense`.
            self.latched = !self.line_low;
            self.phase = Phase::Xfer {
                out,
                bits,
                left,
                step: 1,
            };
            self.next = self.ticks + LOW_TICKS;
            return;
        }
        self.clk_low = false;
        let bits = if out {
            bits << 1
        } else {
            (bits << 1) | u8::from(self.latched)
        };
        if left > 1 {
            self.phase = Phase::Xfer {
                out,
                bits,
                left: left - 1,
                step: 0,
            };
            self.next = self.ticks + (BIT_TICKS - LOW_TICKS);
        } else {
            if !out {
                self.took(bits);
            } else {
                self.data_low = false;
                self.counts.sent += 1;
                // **A reply is spent once it has been read.** The device drove
                // the bus for one transaction and the transceiver has nothing
                // further of its own; a computer that comes back for states 1
                // and 2 again is asking a question nobody is answering, and
                // must get the pulled-up bus.
                //
                // Measured, and it is the difference between a pointer that
                // lands where it is put and one that goes twice as far: the
                // computer reads the announced reply at states 1 and 2, then
                // issues a Talk of its own and reads them *again*, and with
                // the reply still standing it counted the same movement twice.
                if self.slot == Slot::Odd {
                    self.answered = false;
                }
            }
            // An unsolicited byte is not part of a transaction, so the link
            // goes straight back to idle and the *next* thing a device has to
            // say can be announced the same way. Leaving it `Between` would
            // wedge the link until the computer happened to start one.
            self.phase = if core::mem::take(&mut self.unsolicited) != 0 {
                Phase::Idle
            } else {
                Phase::Between
            };
            self.next = NO_EVENT;
            self.announce_if_owed();
        }
    }

    /// A key moved: an ADB key code, with bit 7 set for a release.
    fn key(&mut self, transition: u8) {
        self.counts.reports += 1;
        if self.keys.len() < TYPE_AHEAD {
            self.keys.push_back(transition);
        }
        if let Some(d) = self
            .devices
            .iter_mut()
            .find(|d| d.default_address == cmd::ADDR_KEYBOARD)
        {
            d.pending = true;
        }
        self.ask_for_attention();
    }

    /// The mouse moved, or its button changed.
    fn mouse(&mut self, dx: i8, dy: i8, down: bool) {
        self.counts.reports += 1;
        // **Movement accumulates until it is read.** A mouse counts; a report
        // that arrives while the link is busy has to add to what is owed
        // rather than replace it, or every report the computer was too busy to
        // collect is simply lost. Measured: a pointer sent across the screen
        // arrived twenty-seven pixels short, and fourteen of its hundred and
        // twenty reports had never been answered.
        //
        // Seven bits two's complement, so the total saturates at the most one
        // report can carry rather than wrapping into the opposite direction.
        self.owed_x = (self.owed_x + i16::from(dx)).clamp(-64, 63);
        self.owed_y = (self.owed_y + i16::from(dy)).clamp(-64, 63);
        let (dx, dy) = (self.owed_x as i8, self.owed_y as i8);
        let pack = |d: i8, bit: bool| -> u8 {
            let v = (d as u8) & 0x7f;
            if bit { v } else { v | 0x80 }
        };
        if let Some(d) = self
            .devices
            .iter_mut()
            .find(|d| d.default_address == cmd::ADDR_MOUSE)
        {
            // "Bit 7 of the first byte is the button, 0 while it is down"; the
            // seven bits below each byte are a signed delta, y first.
            d.data = [pack(dy, down), pack(dx, false)];
            d.pending = true;
        }
        self.ask_for_attention();
    }

    /// Pull the attention line low for a while: the transceiver has something
    /// for the computer.
    fn ask_for_attention(&mut self) {
        self.int_low = true;
        self.int_until = self.ticks + ATTENTION_TICKS;
        // **And clock a byte at the computer**, which is the half that makes
        // the attention line worth anything. See `State::unsolicited`.
        //
        // A report that arrives while the link is busy is **owed**, not
        // dropped: the device keeps its data and the announcement goes out
        // when the link next falls idle ([`State::announce_if_owed`]). Losing
        // it instead is what left a pointer short of where it was sent.
        if self.phase != Phase::Idle || self.lines != 3 {
            return;
        }
        let Some(address) = self
            .devices
            .iter()
            .find(|d| d.pending)
            .map(|d| d.address)
            .or_else(|| (!self.keys.is_empty()).then_some(cmd::ADDR_KEYBOARD))
        else {
            return;
        };
        // The transceiver polls the device **itself** and then hands the
        // computer the command it used, so the reply is already waiting when
        // the computer comes to collect it at states 1 and 2.
        let command = (address << 4) | (cmd::TALK << 2);
        self.apply_command(command);
        self.unsolicited = command;
        self.phase = Phase::Starting { out: true };
        self.next = self.ticks + START_TICKS;
    }

    /// Announce a device's data if one has some and the link has fallen idle.
    ///
    /// Called wherever the link can become idle: at the end of a transfer and
    /// when the computer puts the state lines back to 3.
    fn announce_if_owed(&mut self) {
        if self.phase == Phase::Idle
            && self.lines == 3
            && (self.devices.iter().any(|d| d.pending) || !self.keys.is_empty())
        {
            self.ask_for_attention();
        }
    }

    /// The tick the device owes itself.
    fn next_event(&self) -> u64 {
        if self.int_low {
            return self.next.min(self.int_until);
        }
        self.next
    }
}

// ---------------------------------------------------------------------------
// the host object
// ---------------------------------------------------------------------------

#[derive(Debug, Default, Clone)]
struct Outputs {
    clk: Option<WireSource>,
    data: Option<WireSource>,
    int: Option<WireSource>,
}

/// An Apple Desktop Bus transceiver with a keyboard and a mouse on it.
///
/// The host object *is* the bus: a host opens it by name and moves the mouse or
/// presses keys on it, and the device opens the same name and gives it its
/// wires and its clock — the shape [`super::keyboard`] uses.
pub struct Adb {
    state: Mutex<State>,
    ticks: AtomicU64,
    next_event: AtomicU64,
    out: Mutex<Outputs>,
    lazy: Mutex<Option<LazyHandle>>,
}

impl fmt::Debug for Adb {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let mut s = f.debug_struct("Adb");
        match self.state.try_lock() {
            Some(state) => s.field("state", &*state).finish(),
            None => s.field("state", &"<in use>").finish(),
        }
    }
}

impl Default for Adb {
    fn default() -> Adb {
        Adb::new()
    }
}

impl Adb {
    /// A transceiver just switched on, with a keyboard at address 2 and a
    /// mouse at address 3.
    #[must_use]
    pub fn new() -> Adb {
        let state = State::power_on(0);
        Adb {
            ticks: AtomicU64::new(0),
            next_event: AtomicU64::new(state.next_event()),
            state: Mutex::with_rank(LockRank::DEVICE, state),
            out: Mutex::with_rank(LockRank::WIRE, Outputs::default()),
            lazy: Mutex::with_rank(LockRank::LEAF, None),
        }
    }

    /// One key movement: an ADB key code, with bit 7 set for a release.
    pub fn key(&self, transition: u8) {
        self.sync();
        self.update(|st| st.key(transition));
    }

    /// Press (`down`) or release the key whose code is `key`.
    pub fn press(&self, key: u8, down: bool) {
        self.key(if down { key & 0x7f } else { key | 0x80 });
    }

    /// The mouse moved by `(dx, dy)` with its button up or `down`.
    pub fn mouse(&self, dx: i8, dy: i8, down: bool) {
        self.sync();
        self.update(|st| st.mouse(dx, dy, down));
    }

    /// How many transactions the computer has started: one per command byte.
    #[must_use]
    pub fn transactions(&self) -> u64 {
        self.state.lock().transactions
    }

    /// The last command byte, the last byte sent to the computer and the last
    /// byte taken from it.
    #[must_use]
    pub fn last_exchange(&self) -> (u8, u8, u8) {
        let st = self.state.lock();
        (st.last_command, st.last_sent, st.last_received)
    }

    /// Where each device on the bus answers now, in bus order.
    #[must_use]
    pub fn addresses(&self) -> Vec<u8> {
        self.state
            .lock()
            .devices
            .iter()
            .map(|d| d.address)
            .collect()
    }

    /// The transceiver's own state, for a trace: the state lines, which byte
    /// of a transaction is next, whether the addressed device answered, the
    /// command byte, and whether a transfer is in flight.
    #[must_use]
    pub fn probe(&self) -> (u8, u8, bool, u8, bool) {
        let st = self.state.lock();
        let slot = match st.slot {
            Slot::Command => 0,
            Slot::Even => 1,
            Slot::Odd => 2,
        };
        (
            st.lines,
            slot,
            st.answered,
            st.command,
            st.phase != Phase::Idle,
        )
    }

    /// The four stages one input report passes through, counted separately.
    ///
    /// See [`Counters`]: the one that drops is where a report is going.
    #[must_use]
    pub fn counters(&self) -> Counters {
        self.state.lock().counts
    }

    /// Whether the transceiver is pulling `(clock, data, attention)` low.
    #[must_use]
    pub fn lines(&self) -> (bool, bool, bool) {
        self.state.lock().pins()
    }

    /// Ticks simulated — microseconds, on a board that clocks it at 1 MHz.
    #[must_use]
    pub fn ticks(&self) -> u64 {
        self.ticks.load(Ordering::Relaxed)
    }

    /// Run the transceiver until `target` ticks have passed in total.
    ///
    /// One step at a time, driving the lines between steps with no lock held:
    /// an edge reaches the VIA, which has a lock of its own, and each edge has
    /// to arrive before the next is computed.
    pub fn advance_to(&self, target: u64) {
        loop {
            let moved = {
                let mut st = self.state.lock();
                let due = st.next_event();
                if due != NO_EVENT && due <= target {
                    let before = st.pins();
                    st.ticks = st.ticks.max(due);
                    if st.int_low && st.ticks >= st.int_until {
                        st.int_low = false;
                    }
                    if st.next != NO_EVENT && st.next <= st.ticks {
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

    /// Drive all three output stages to what the state says, holding no lock.
    fn refresh(&self) {
        let (clk, data, int) = self.state.lock().pins();
        let out = self.out.lock().clone();
        for (src, low) in [(&out.clk, clk), (&out.data, data), (&out.int, int)] {
            if let Some(src) = src {
                src.drive(if low { Drive::Low } else { Drive::HiZ });
            }
        }
    }

    fn sync(&self) {
        let handle = self.lazy.lock().clone();
        if let Some(handle) = handle {
            let _ = handle.sync(AccessKind::Guest);
        }
    }

    fn power_on(&self) {
        self.update(|st| {
            let (ticks, lines, line_low) = (st.ticks, st.lines, st.line_low);
            *st = State::power_on(ticks);
            st.lines = lines;
            st.line_low = line_low;
        });
    }
}

/// The build's named Apple Desktop Buses.
///
/// The same door shape [`super::keyboard::keys`] uses: a name travels from the
/// machine file into the device constructor, and both ends resolve it against
/// the build's [`HostObjects`](crate::core::hosts::HostObjects).
pub mod bus {
    use super::Adb;
    use alloc::string::String;
    use alloc::sync::Arc;
    use alloc::vec::Vec;

    use crate::core::error::Result;
    use crate::core::hosts::{HostKind, HostObjects};
    use crate::core::props::Props;
    use crate::core::record::{Channel, FnSink, InputSink};

    /// The kind a transceiver is filed under in a build's host objects.
    pub const KIND: HostKind = HostKind::door("mac-adb", make_sink);

    /// The bus `name` refers to in `hosts`, creating it on first mention.
    ///
    /// # Errors
    ///
    /// [`crate::Error::Config`] if another kind of object holds that name.
    pub fn open(hosts: &HostObjects, name: &str) -> Result<Arc<Adb>> {
        hosts.open(KIND, name, Adb::new)
    }

    /// The bus `name` refers to in the build `props` belongs to — the device's
    /// side.
    ///
    /// # Errors
    ///
    /// As [`open`].
    pub fn attach(props: &Props, name: &str) -> Result<Arc<Adb>> {
        props.host(KIND, name, Adb::new)
    }

    /// The bus called `name`, if a device or a host has opened it.
    ///
    /// # Errors
    ///
    /// As [`open`].
    pub fn get(hosts: &HostObjects, name: &str) -> Result<Option<Arc<Adb>>> {
        hosts.get(KIND, name)
    }

    /// Every open name, in order.
    #[must_use]
    pub fn names(hosts: &HostObjects) -> Vec<String> {
        hosts.names(KIND)
    }

    /// Bytes per recorded event: three, `(kind, a, b)`.
    ///
    /// `kind` 0 is a key transition in `a`; `kind` 1 is a mouse movement, `a`
    /// and `b` the two signed deltas with the button in bit 7 of `a`.
    pub const RECORD_BYTES: usize = 3;

    /// The channel this bus's input arrives on: `mac-adb:<name>`.
    #[must_use]
    pub fn channel(name: &str) -> Channel {
        Channel::new(KIND, name)
    }

    /// The sink that applies a recorded payload to `adb`, an event per three
    /// bytes, in order.
    #[must_use]
    pub fn sink(adb: &Arc<Adb>) -> Arc<dyn InputSink> {
        let adb = Arc::clone(adb);
        Arc::new(FnSink::new("mac-adb", move |payload: &[u8]| {
            for event in payload.as_chunks::<RECORD_BYTES>().0 {
                match event[0] {
                    0 => adb.key(event[1]),
                    _ => adb.mouse(
                        (event[1] & 0x7f) as i8,
                        event[2] as i8,
                        event[1] & 0x80 != 0,
                    ),
                }
            }
        }))
    }

    fn make_sink(object: &Arc<dyn core::any::Any + Send + Sync>) -> Option<Arc<dyn InputSink>> {
        Some(sink(&Arc::clone(object).downcast::<Adb>().ok()?))
    }
}

// ---------------------------------------------------------------------------
// the device
// ---------------------------------------------------------------------------

/// The data line as something a wire drives.
#[derive(Debug)]
struct DataPin {
    adb: Arc<Adb>,
}

impl WireSink for DataPin {
    fn set_level(&self, _src: WireId, _line: u32, level: Level) {
        self.adb.sync();
        let low = level.is_low();
        self.adb.update(|st| st.sense(low));
    }
}

/// One of the two state lines.
#[derive(Debug)]
struct StatePin {
    adb: Arc<Adb>,
    /// 0 for `ST0`, 1 for `ST1`.
    which: u8,
    inputs: FanIn,
}

impl WireSink for StatePin {
    fn set_level(&self, src: WireId, _line: u32, level: Level) {
        self.inputs.set(src, level);
        let high = self.inputs.resolve(Resolve::And).is_high();
        self.adb.sync();
        let bit = 1u8 << self.which;
        self.adb.update(|st| {
            let lines = if high {
                st.lines | bit
            } else {
                st.lines & !bit
            };
            st.state_changed(lines);
        });
    }
}

/// The `mac.adb` device.
#[derive(Debug)]
pub struct MacAdb {
    adb: Arc<Adb>,
    data: Arc<DataPin>,
    st0: Mutex<Vec<Arc<StatePin>>>,
}

impl MacAdb {
    /// Validate `props` and open the bus they name.
    ///
    /// # Errors
    ///
    /// [`Error::Property`] for a property this class does not know, or
    /// [`Error::Config`] if the name is held by something that is not a bus.
    pub fn new(props: &Props) -> Result<MacAdb> {
        let mut r = props.reader();
        let port = r.or_str("bus", "adb")?.to_string();
        r.finish()?;
        let adb = bus::attach(props, &port)?;
        Ok(MacAdb::with(adb))
    }

    /// A device around a bus the caller already holds.
    #[must_use]
    pub fn with(adb: Arc<Adb>) -> MacAdb {
        let data = Arc::new(DataPin {
            adb: Arc::clone(&adb),
        });
        MacAdb {
            adb,
            data,
            st0: Mutex::with_rank(LockRank::LEAF, Vec::new()),
        }
    }

    /// The bus.
    #[must_use]
    pub fn bus(&self) -> &Arc<Adb> {
        &self.adb
    }
}

impl Device for MacAdb {
    fn class(&self) -> &'static DeviceClass {
        &CLASS
    }

    fn realize(&self, _ctx: &mut RealizeCtx<'_>) -> Result<()> {
        Ok(())
    }

    fn reset(&self, kind: ResetKind) {
        // The transceiver is a computer of its own and is powered for as long
        // as the Macintosh is; only a power cycle restarts it. A bus reset is
        // the protocol's own path and not this one.
        if kind == ResetKind::Cold {
            self.adb.power_on();
        }
    }

    fn save(&self, w: &mut ChunkWriter<'_>) -> Result<()> {
        let st = self.adb.state.lock().clone();
        w.write_u64(st.ticks)?;
        w.write_u64(st.next)?;
        match st.phase {
            Phase::Idle => w.write_u8(0)?,
            Phase::Starting { out } => {
                w.write_u8(1)?;
                w.write_bool(out)?;
            }
            Phase::Xfer {
                out,
                bits,
                left,
                step,
            } => {
                w.write_u8(2)?;
                w.write_bool(out)?;
                w.write_u8(bits)?;
                w.write_u8(left)?;
                w.write_u8(step)?;
            }
            Phase::Between => w.write_u8(3)?,
        }
        w.write_u8(st.lines)?;
        for v in [st.clk_low, st.data_low, st.line_low, st.latched, st.int_low] {
            w.write_bool(v)?;
        }
        w.write_u64(st.int_until)?;
        w.write_u8(st.command)?;
        w.write_u8(match st.slot {
            Slot::Command => 0,
            Slot::Even => 1,
            Slot::Odd => 2,
        })?;
        w.write_bytes(&st.reply)?;
        w.write_bool(st.answered)?;
        w.write_bytes(&st.heard)?;
        for d in &st.devices {
            w.write_u8(d.address)?;
            w.write_u8(d.default_address)?;
            w.write_u8(d.handler)?;
            w.write_bool(d.pending)?;
            w.write_bytes(&d.data)?;
        }
        w.write_bytes(&st.keys.iter().copied().collect::<Vec<u8>>())?;
        w.write_u64(st.transactions)?;
        w.write_u8(st.last_command)?;
        w.write_u8(st.last_sent)?;
        w.write_u8(st.last_received)
    }

    fn load(&self, r: &mut ChunkReader<'_>) -> Result<()> {
        let bad = |what: &str| Error::State(format!("{CLASS_NAME}: {what}"));
        let mut st = State::power_on(0);
        st.ticks = r.read_u64()?;
        st.next = r.read_u64()?;
        st.phase = match r.read_u8()? {
            0 => Phase::Idle,
            1 => Phase::Starting {
                out: r.read_bool()?,
            },
            2 => {
                let out = r.read_bool()?;
                let (bits, left, step) = (r.read_u8()?, r.read_u8()?, r.read_u8()?);
                if !(1..=BITS).contains(&left) || step > 1 {
                    return Err(bad("a transfer step out of range"));
                }
                Phase::Xfer {
                    out,
                    bits,
                    left,
                    step,
                }
            }
            3 => Phase::Between,
            _ => return Err(bad("an unknown link phase")),
        };
        st.lines = r.read_u8()? & 3;
        st.clk_low = r.read_bool()?;
        st.data_low = r.read_bool()?;
        st.line_low = r.read_bool()?;
        st.latched = r.read_bool()?;
        st.int_low = r.read_bool()?;
        st.int_until = r.read_u64()?;
        st.command = r.read_u8()?;
        st.slot = match r.read_u8()? {
            0 => Slot::Command,
            1 => Slot::Even,
            2 => Slot::Odd,
            _ => return Err(bad("an unknown transaction slot")),
        };
        let reply = r.read_bytes()?;
        if reply.len() != 2 {
            return Err(bad("a reply that is not two bytes"));
        }
        st.reply.copy_from_slice(reply);
        st.answered = r.read_bool()?;
        let heard = r.read_bytes()?;
        if heard.len() != 2 {
            return Err(bad("a listened pair that is not two bytes"));
        }
        st.heard.copy_from_slice(heard);
        for i in 0..st.devices.len() {
            let d = &mut st.devices[i];
            d.address = r.read_u8()? & 0x0f;
            d.default_address = r.read_u8()? & 0x0f;
            d.handler = r.read_u8()?;
            d.pending = r.read_bool()?;
            let data = r.read_bytes()?;
            if data.len() != 2 {
                return Err(bad("a device register that is not two bytes"));
            }
            d.data.copy_from_slice(data);
        }
        let keys = r.read_bytes()?;
        if keys.len() > TYPE_AHEAD {
            return Err(bad("a type-ahead buffer longer than the keyboard's"));
        }
        st.keys = keys.iter().copied().collect();
        st.transactions = r.read_u64()?;
        st.last_command = r.read_u8()?;
        st.last_sent = r.read_u8()?;
        st.last_received = r.read_u8()?;
        if st.next != NO_EVENT && st.next <= st.ticks && st.phase != Phase::Idle {
            // An event that is not in the future would stall catch-up.
            st.next = st.ticks + 1;
        }
        self.adb.update(|now| *now = st);
        Ok(())
    }

    fn connect(&self, port: &str, source: WireSource) -> Result<()> {
        {
            let mut out = self.adb.out.lock();
            match port {
                CLK_PIN => out.clk = Some(source),
                DATA_PIN => out.data = Some(source),
                INT_PIN => out.int = Some(source),
                _ => {
                    return Err(Error::Config {
                        at: port.to_string(),
                        message: format!(
                            "an ADB transceiver drives `{CLK_PIN}`, `{DATA_PIN}` and `{INT_PIN}`"
                        ),
                    });
                }
            }
        }
        self.adb.refresh();
        Ok(())
    }

    fn announce(&self, _port: &str) {
        self.adb.refresh();
    }

    fn sink(&self, port: &str, sources: &[WireId]) -> Option<SinkPin> {
        if port == DATA_PIN {
            return Some(SinkPin {
                sink: Arc::clone(&self.data) as Arc<dyn WireSink>,
                line: 0,
            });
        }
        let which = match port {
            ST0_PIN => 0,
            ST1_PIN => 1,
            _ => return None,
        };
        let pin = Arc::new(StatePin {
            adb: Arc::clone(&self.adb),
            which,
            inputs: FanIn::new(sources),
        });
        // The pin is kept alive here: a net holds only a `Weak` to its sinks.
        self.st0.lock().push(Arc::clone(&pin));
        Some(SinkPin { sink: pin, line: 0 })
    }

    fn is_lazy(&self) -> bool {
        true
    }

    fn current_tick(&self) -> u64 {
        self.adb.ticks.load(Ordering::Relaxed)
    }

    fn advance_to(&self, tick: u64) {
        self.adb.advance_to(tick);
    }

    fn next_event_tick(&self) -> Option<u64> {
        match self.adb.next_event.load(Ordering::Relaxed) {
            NO_EVENT => None,
            tick => Some(tick),
        }
    }

    fn attach_lazy(&self, handle: LazyHandle) {
        *self.adb.lazy.lock() = Some(handle);
    }
}

impl Instance for MacAdb {}

/// The `mac.adb` device class.
pub static CLASS: DeviceClass = DeviceClass {
    name: CLASS_NAME,
    version: STATE_VERSION,
    summary: "an Apple Desktop Bus transceiver on the VIA's shift register, with a keyboard at \
              address 2 and a mouse at address 3",
    properties: &[PropertySpec {
        name: "bus",
        kind: ValueKind::Str,
        required: false,
        summary: "the host bus port it is typed and pointed on (default `adb`)",
    }],
    construct: |props| Ok(Box::new(MacAdb::new(props)?)),
};

/// Add [`CLASS`] to a registry.
///
/// # Errors
///
/// [`crate::core::Error::Config`] if something already claimed the name.
pub fn register(registry: &mut crate::core::Registry) -> Result<()> {
    registry.add(&CLASS)
}

/// Bind [`CLASS`] into the machine graph.
///
/// # Errors
///
/// [`crate::core::Error::Config`] if the class is already bound.
pub fn bind(bindings: &mut crate::machine::Bindings) -> Result<()> {
    bindings.bind(CLASS_NAME, |props| Ok(Arc::new(MacAdb::new(props)?)))
}

/// What the validator should know about `mac.adb`.
#[must_use]
pub fn schema() -> ClassSchema {
    ClassSchema::new(CLASS_NAME)
        .prop(PropSchema::new("bus", ValueKind::Str))
        .port(CLK_PIN, PortDir::Out)
        .port(DATA_PIN, PortDir::InOut)
        .port(INT_PIN, PortDir::Out)
        .port(ST0_PIN, PortDir::In)
        .port(ST1_PIN, PortDir::In)
}

/// A short name for a command byte, for a monitor or a failing test.
#[must_use]
pub fn describe(byte: u8) -> String {
    if byte == cmd::RESET {
        return String::from("SendReset");
    }
    let (addr, reg) = (cmd::address(byte), cmd::register(byte));
    match cmd::command(byte) {
        cmd::TALK => format!("Talk {addr} r{reg}"),
        cmd::LISTEN => format!("Listen {addr} r{reg}"),
        _ if reg == 1 => format!("Flush {addr}"),
        _ => format!("reserved ${byte:02x}"),
    }
}

#[cfg(test)]
mod tests;
