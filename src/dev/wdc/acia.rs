//! The W65C51N ACIA: four registers, one serial line.
//!
//! # Sources
//!
//! Everything below is from the **W65C51N Asynchronous Communications
//! Interface Adapter (ACIA) data sheet, Western Design Center, March 18 2024**
//! (`www.WDC65xx.com`, and mirrored at `eater.net/datasheets/w65c51n.pdf`):
//!
//! * Table 1, "ACIA Register Selection", page 16 — the RS1/RS0 decode, and the
//!   fact that the four addresses are *six* registers because read and write
//!   differ at two of them.
//! * "Status Register", page 9 — the bit assignment and the reset table.
//! * "Control Register", page 11 — word length, stop bits, clock source and
//!   the sixteen baud rates.
//! * "Command Register", page 13 — DTR, IRD, TIC, REM, PME, PMC.
//! * "Reset (RESB)", page 15 — what a hardware reset leaves behind.
//!
//! No emulator was consulted (`ROADMAP.md` §1). Where this file states a
//! number, that number is in the data sheet.
//!
//! # The register map
//!
//! ```text
//!   +0  read: receiver data       write: transmit data
//!   +1  read: status              write: programmed reset (data ignored)
//!   +2  read/write: command
//!   +3  read/write: control
//! ```
//!
//! Only the command and control registers can both be read and written. The
//! programmed reset performs no data transfer at all: it clears bits 4-0 of the
//! command register and the overrun bit of the status register, and leaves the
//! control register alone (data sheet, Table 1's note).
//!
//! ## Status
//!
//! ```text
//!   7 IRQ   6 DSRB   5 DCDB   4 TDRE   3 RDRF   2 OVRN   1 FE   0 PE
//! ```
//!
//! Reading the status register clears bit 7. Reading the *data* register
//! clears RDRF and the three error bits, which is why the receive loop is
//! "poll status, then read data" and never the other way round.
//!
//! `DSRB` and `DCDB` read as 0 — modem ready, carrier detected. There is no
//! modem here and nothing that could ever change them, so the data sheet's
//! "a level change causes an immediate interrupt" has no source to fire from.
//!
//! `OVRN`, `FE` and `PE` are always 0, and that is a fact about the *seam*
//! rather than a simplification. [`CharDevice`] carries bytes, not bit cells,
//! so a framing or parity error has no way to occur — and parity is disabled on
//! this part in any case, see the command register below. Overrun is the
//! interesting one: on the chip it happens because the receiver *shift*
//! register finishes another character while the processor has not yet read the
//! last one, and the old byte is lost. Under a queue-backed port there is no
//! shift register and nothing is lost: a byte the host has sent simply waits
//! until the receiver register is free. Taking it early to raise the flag would
//! mean throwing away a byte in order to report having thrown one away.
//!
//! ## The transmit erratum, and why it is not the default
//!
//! The W65C51N loads the transmit data register and the transmit *shift*
//! register at the same time, so **TDRE is permanently 1**: the data sheet
//! says so in as many words ("The Transmitter Data Register Empty (TDRE) bit is
//! always a 1 … TDRE bit cannot be polled to determine when to write the next
//! byte to the TDR/TSR. A delay loop should be used"). Software written for a
//! 6551 — which is to say almost all of it — polls TDRE, sees it set, writes
//! the next byte immediately, and drops characters on a real WDC part.
//!
//! rsemu models the **correct** 6551 contract by default: TDRE is 1 when the
//! transmitter can take a byte and 0 while it is holding one. Set
//! `erratum = true` for the W65C51N's own behaviour, in which TDRE reads 1 no
//! matter what. Which is which:
//!
//! | `erratum` | TDRE reads | Models |
//! | --- | --- | --- |
//! | `false` (default) | `1` only when the transmitter is free | the 6551 contract, and every other 6551-family part |
//! | `true` | always `1` | the W65C51N silicon, data sheet page 10 |
//!
//! The erratum is deliberately not the default even though it is the
//! documented behaviour of the named part, because a machine whose serial port
//! silently drops characters is a bug report, not a demonstration. Back
//! pressure still applies in both modes: a host that will not take the byte
//! leaves it in the transmit register, so nothing is lost either way — what
//! `erratum = true` changes is whether *software* can see that.
//!
//! ## Command
//!
//! ```text
//!   7-6 PMC   5 PME   4 REM   3-2 TIC   1 IRD   0 DTR
//! ```
//!
//! Bit 0 (DTR) enables every selected interrupt as well as driving DTRB, and
//! bit 1 (IRD) disables the receiver interrupt on its own. Parity is disabled
//! on the W65C51N whatever bits 7-5 say — the data sheet's own table gives "use
//! but no parity" for all four PMC codes — so those bits are stored, read back,
//! and have no effect. Transmitter interrupts are documented as never to be
//! enabled on this part, so TIC selects nothing here but the RTSB level, which
//! nothing in an rsemu machine observes yet.
//!
//! ## Control
//!
//! ```text
//!   7 SBN   6-5 WL   4 RCS   3-0 SBR
//! ```
//!
//! Stored, read back, and reported by [`Acia::baud`] and [`Acia::word_length`],
//! and otherwise inert: the byte pipe under this device has no bit time. The
//! *pace* a machine file gives the device (see below) is what stands in for
//! the baud rate, and it is set in the machine file rather than derived from
//! this register, which is a stated limit rather than an oversight —
//! re-rating a clock domain from a guest register write is a `core::clock`
//! feature (`ROADMAP.md` §4.2, "runtime re-rating") that no machine has needed
//! yet.
//!
//! # Pacing
//!
//! With `paced = true` the transmitter holds each byte until a tick of its
//! clock domain releases it, so a machine file that rates that domain at the
//! character rate — 19200 baud 8-N-1 is 1920 characters a second — makes a
//! guest polling TDRE wait as long as the hardware would. With `paced = false`
//! the byte is gone before the store finishes, which is what a test wants.

use alloc::boxed::Box;
use alloc::string::String;
use alloc::sync::Arc;
use core::fmt;

use crate::core::device::{Device, DeviceClass, PropertySpec, RealizeCtx, ResetKind};
use crate::core::error::{BusError, Error, Result};
use crate::core::props::{Props, ValueKind};
use crate::core::sched::{Budget, Consumed};
use crate::core::space::{AccessConstraints, MemAttrs, MemOps, MemResult, Region, RegionRef};
use crate::core::state::{ChunkReader, ChunkWriter, Sink, Source};
use crate::core::sync::{LockRank, Mutex};
use crate::core::value::{Endian, Width};
use crate::core::wire::{Level, WireSource};
use crate::host::chardev::{CharDevice, ports};
use crate::machine::realize::Instance;

/// The class name a machine description writes.
const CLASS_NAME: &str = "wdc.w65c51n";

/// The snapshot chunk version. Bump with the encoding, never on its own.
const STATE_VERSION: u32 = 1;

/// The character port a machine file gets if it names none.
const DEFAULT_PORT: &str = "console";

/// How many bytes of address space the four registers occupy.
///
/// The chip decodes RS0 and RS1 only, so a board that gives it an eight-kilobyte
/// window sees these four repeated all through it — which is exactly what
/// `machines/beneater-6502.machine` writes as `mirror(acia)`.
pub const REGISTER_COUNT: u64 = 4;

/// The name of the interrupt output pin.
pub const IRQ_PIN: &str = "irq";

// -- status register (data sheet page 9) ------------------------------------

/// Bit 7: an interrupt has occurred. Cleared by reading the status register.
const ST_IRQ: u8 = 0x80;
/// Bit 4: the transmit data register is empty.
const ST_TDRE: u8 = 0x10;
/// Bit 3: the receiver data register is full.
const ST_RDRF: u8 = 0x08;
/// Bit 2: a byte arrived before the last one was read.
///
/// Never set here — see the module docs on overrun — but named and asserted on
/// by the tests, because "always 0" is a claim worth checking rather than a
/// bit worth forgetting.
#[cfg(test)]
const ST_OVRN: u8 = 0x04;

// Bits 6 (DSRB) and 5 (DCDB) are 0 here — see the module docs — and bits 1
// (FE) and 0 (PE) cannot occur on a byte pipe, so neither has a constant.

// -- command register (data sheet page 13) ----------------------------------

/// Bit 0: data terminal ready, and the master enable for every interrupt.
const CMD_DTR: u8 = 0x01;
/// Bit 1: receiver interrupt request disabled.
const CMD_IRD: u8 = 0x02;
/// Bits 4-0, which a programmed reset clears.
const CMD_PROGRAMMED_RESET: u8 = 0x1f;

// -- control register (data sheet page 11) ----------------------------------

/// Bits 3-0: the selected baud rate.
const CTRL_SBR: u8 = 0x0f;
/// Bits 6-5: the word length.
const CTRL_WL: u8 = 0x60;

/// The sixteen baud rates of control-register bits 3-0, in order.
///
/// Code 0 is the external clock divided by sixteen, which with the 1.8432 MHz
/// crystal the data sheet names is 115200. The three rates the table gives to
/// two decimal places (109.92, 134.58) are rounded down here to whole bits per
/// second; nothing in rsemu uses this for timing, only for
/// [`Acia::baud`]'s answer.
const BAUD_RATES: [u32; 16] = [
    115_200, 50, 75, 109, 134, 150, 300, 600, 1200, 1800, 2400, 3600, 4800, 7200, 9600, 19_200,
];

/// The W65C51N as a device: four registers over a [`CharDevice`].
///
/// Two-phase like every device (`ROADMAP.md` §4.4): [`Acia::new`] validates
/// properties, opens its character port and builds the region;
/// [`Device::realize`] does nothing, because a `map` statement places the
/// region and the realizer does that afterwards.
#[derive(Debug)]
pub struct Acia {
    regs: Arc<Registers>,
    region: RegionRef,
}

/// The register block, as something an address space can dispatch to.
struct Registers {
    state: Mutex<State>,
    port: Arc<dyn CharDevice>,
    /// The name the port was opened under, for `Debug` and for diagnostics.
    port_name: String,
    /// Whether the transmitter waits for a clock tick before letting a byte go.
    paced: bool,
    /// Whether TDRE reads 1 unconditionally — the W65C51N's own behaviour.
    erratum: bool,
    /// The interrupt output, connected at realize time.
    irq: Mutex<Option<WireSource>>,
}

/// Everything the guest can see or change.
#[derive(Debug, Default, Clone, Copy, PartialEq, Eq)]
struct State {
    /// The command register.
    command: u8,
    /// The control register.
    control: u8,
    /// The receiver data register. Meaningless unless `rdrf`.
    rdr: u8,
    /// Status bit 3: a byte is waiting to be read.
    rdrf: bool,
    /// The transmit data register. Meaningless unless `tx_full`.
    tdr: u8,
    /// The transmitter is holding a byte the host has not taken.
    tx_full: bool,
    /// Status bit 7, latched: cleared by reading the status register.
    irq: bool,
}

impl fmt::Debug for Registers {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let mut s = f.debug_struct("Registers");
        s.field("port", &self.port_name)
            .field("paced", &self.paced)
            .field("erratum", &self.erratum);
        match self.state.try_lock() {
            Some(state) => s.field("state", &*state).finish(),
            None => s.field("state", &"<in use>").finish(),
        }
    }
}

impl Acia {
    /// Validate `props` and build the device.
    ///
    /// # Errors
    ///
    /// [`Error::Property`] if a property is of the wrong kind, or if one this
    /// class does not know was given.
    pub fn new(props: &Props) -> Result<Acia> {
        let mut r = props.reader();
        let port_name = r.or("port", String::from(DEFAULT_PORT))?;
        let paced = r.or("paced", true)?;
        let erratum = r.or("erratum", false)?;
        r.finish()?;
        Ok(Acia::with_port(
            ports::open(&port_name),
            port_name,
            paced,
            erratum,
        ))
    }

    /// Build one against a character device the caller already has.
    ///
    /// The route a test takes: it holds the other end of the port and does not
    /// have to go through the name table to find it.
    #[must_use]
    pub fn with_port(
        port: Arc<dyn CharDevice>,
        port_name: String,
        paced: bool,
        erratum: bool,
    ) -> Acia {
        let regs = Arc::new(Registers {
            state: Mutex::with_rank(LockRank::DEVICE, State::default()),
            port,
            port_name,
            paced,
            erratum,
            irq: Mutex::with_rank(LockRank::WIRE, None),
        });
        let region = Arc::new(Region::io(
            "acia",
            REGISTER_COUNT,
            Arc::clone(&regs) as Arc<dyn MemOps>,
        ));
        Acia { regs, region }
    }

    /// The name of the character port this device is attached to.
    #[must_use]
    pub fn port_name(&self) -> &str {
        &self.regs.port_name
    }

    /// Whether the transmitter waits for a clock tick before letting a byte go.
    #[must_use]
    pub fn is_paced(&self) -> bool {
        self.regs.paced
    }

    /// Whether TDRE reads 1 unconditionally — the W65C51N's own behaviour.
    #[must_use]
    pub fn has_erratum(&self) -> bool {
        self.regs.erratum
    }

    /// The status register as software would read it, disturbing nothing.
    #[must_use]
    pub fn status(&self) -> u8 {
        self.regs.status(&self.regs.state.lock())
    }

    /// The baud rate control-register bits 3-0 select.
    ///
    /// Reported, never used: see the module docs on why the pace comes from the
    /// machine file instead.
    #[must_use]
    pub fn baud(&self) -> u32 {
        BAUD_RATES[(self.regs.state.lock().control & CTRL_SBR) as usize]
    }

    /// The word length control-register bits 6-5 select, in bits.
    #[must_use]
    pub fn word_length(&self) -> u8 {
        // 00 -> 8, 01 -> 7, 10 -> 6, 11 -> 5 (data sheet page 11).
        8 - ((self.regs.state.lock().control & CTRL_WL) >> 5)
    }

    /// Connect the interrupt output.
    pub fn connect_irq(&self, source: WireSource) {
        *self.regs.irq.lock() = Some(source);
        self.regs.refresh_irq();
    }

    /// The level the interrupt output is driving.
    ///
    /// High is "requesting". The pin on the chip is `IRQB` and is active low
    /// and open drain; inverting it is a `wire.not` device's job when one
    /// exists, and until then every rsemu core takes an active-high request
    /// (`ROADMAP.md` §4.3, and `machines/nes-ntsc.machine`'s note on `/NMI`).
    #[must_use]
    pub fn irq_level(&self) -> Level {
        self.regs.irq_level(&self.regs.state.lock())
    }
}

impl Registers {
    /// The status register's value for a given state.
    fn status(&self, state: &State) -> u8 {
        let mut status = 0u8;
        if state.irq {
            status |= ST_IRQ;
        }
        // Bits 6 and 5 stay clear: DSRB and DCDB are low, which the data sheet
        // calls the true condition — ready, and carrier detected.
        if self.erratum || !state.tx_full {
            status |= ST_TDRE;
        }
        if state.rdrf {
            status |= ST_RDRF;
        }
        // OVRN, FE and PE stay clear; see the module docs for why none of the
        // three can occur on a byte pipe.
        status
    }

    /// Whether an enabled interrupt condition is standing.
    ///
    /// Bit 0 of the command register enables every selected interrupt, and bit
    /// 1 disables the receiver's on its own. The transmitter's is never enabled
    /// on this part, and DSRB/DCDB never change, so the receiver is the only
    /// source there is.
    fn irq_pending(state: &State) -> bool {
        state.rdrf && state.command & CMD_DTR != 0 && state.command & CMD_IRD == 0
    }

    fn irq_level(&self, state: &State) -> Level {
        if state.irq { Level::High } else { Level::Low }
    }

    /// Drive the interrupt pin to whatever the latch now says.
    ///
    /// Called with no lock held: a sink is free to call back into this device,
    /// and the re-entrancy contract in `core::device` is that outward calls
    /// happen after the critical section, never inside it.
    fn refresh_irq(&self) {
        let level = self.irq_level(&self.state.lock());
        let port = self.irq.lock().clone();
        if let Some(port) = port {
            port.set(level);
        }
    }

    /// Take a byte from the host if the receiver's register is free.
    ///
    /// The port's lock is a leaf and this device's is `LockRank::DEVICE`, so
    /// taking one inside the other is the ranked order rather than a violation
    /// of it. It has to be nested: a read of the status register must answer
    /// *now*, so there is nothing to defer.
    ///
    /// Returns whether the interrupt latch moved, so the caller can drive the
    /// pin outside the lock.
    fn poll_receiver(&self) -> bool {
        let mut state = self.state.lock();
        if state.rdrf {
            // The register is still full, so the byte stays on the host's side
            // of the seam rather than being taken and dropped. See the module
            // docs on overrun.
            return false;
        }
        let Some(byte) = self.port.read_byte() else {
            return false;
        };
        state.rdr = byte;
        state.rdrf = true;
        if Registers::irq_pending(&state) && !state.irq {
            state.irq = true;
            return true;
        }
        false
    }

    /// Hand the byte the transmitter is holding to the host.
    ///
    /// Returns whether one went. A port that will not take it leaves the
    /// transmitter full, which stalls a guest polling TDRE — that is back
    /// pressure arriving as the hardware would deliver it, not a dropped
    /// character.
    fn transmit(&self) -> bool {
        let byte = {
            let state = self.state.lock();
            if !state.tx_full {
                return false;
            }
            state.tdr
        };
        if !self.port.write_byte(byte) {
            return false;
        }
        self.state.lock().tx_full = false;
        true
    }

    /// Read one register. `debug` suppresses every side effect.
    ///
    /// Returns the byte and whether the interrupt pin wants refreshing.
    fn read_register(&self, index: u8, debug: bool) -> (u8, bool) {
        let mut moved = if debug { false } else { self.poll_receiver() };
        let mut state = self.state.lock();
        let byte = match index {
            // +0 read: the receiver data register. Reading it clears RDRF, and
            // the three error bits with it — they are "self-clearing (i.e. they
            // are automatically cleared after a read of the Receiver Data
            // Register)", data sheet page 10, and here they were never set.
            0 => {
                if !debug {
                    state.rdrf = false;
                }
                state.rdr
            }
            // +1 read: status. Bit 7 "goes to a 0 when the Status Register is
            // read" — page 10.
            1 => {
                let status = self.status(&state);
                if !debug && state.irq {
                    state.irq = false;
                    moved = true;
                }
                status
            }
            2 => state.command,
            _ => state.control,
        };
        (byte, moved)
    }

    /// Write one register.
    ///
    /// Returns whether a byte now wants transmitting and whether the interrupt
    /// pin wants refreshing.
    fn write_register(&self, index: u8, value: u8) -> (bool, bool) {
        let mut state = self.state.lock();
        match index {
            // +0 write: the transmit data register, which on this part loads
            // the shift register at the same instant.
            0 => {
                state.tdr = value;
                state.tx_full = true;
                return (true, false);
            }
            // +1 write: the programmed reset. "The programmed Reset operation
            // does not cause any data transfer, but is used to clear bits 4
            // through 0 in the Command Register and bit 2 in the Status
            // Register. The Control Register is unchanged" — data sheet,
            // Table 1's note. The data written is a don't-care.
            1 => {
                // ...and bit 2 of the status register, which on this seam is
                // already always clear.
                state.command &= !CMD_PROGRAMMED_RESET;
            }
            2 => state.command = value,
            _ => state.control = value,
        }
        // A command-register write can enable or disable the receiver
        // interrupt, so the latch has to be re-evaluated.
        let want = Registers::irq_pending(&state);
        if want != state.irq {
            state.irq = want;
            return (false, true);
        }
        (false, false)
    }
}

impl MemOps for Registers {
    fn read(&self, offset: u64, dst: &mut [u8], attrs: MemAttrs) -> MemResult {
        let [byte] = dst else {
            return Err(BusError::BadAccess);
        };
        let (value, moved) = self.read_register((offset & 3) as u8, attrs.debug);
        *byte = value;
        if moved {
            self.refresh_irq();
        }
        Ok(())
    }

    fn write(&self, offset: u64, src: &[u8], attrs: MemAttrs) -> MemResult {
        let [value] = src else {
            return Err(BusError::BadAccess);
        };
        if attrs.debug {
            // A debug write to +0 would put a character on the wire and to +1
            // would reset the chip. Neither is something the core can make
            // harmless, so it is refused rather than guessed at
            // (`ROADMAP.md` §15, invariant 5).
            return Err(BusError::BadAccess);
        }
        let (transmit, moved) = self.write_register((offset & 3) as u8, *value);
        if transmit && !self.paced {
            // Unpaced: the wire is infinitely fast, so the byte is gone before
            // the store instruction finishes and TDRE never reads clear. That
            // is the mode a test runs in.
            self.transmit();
        }
        if moved {
            self.refresh_irq();
        }
        Ok(())
    }

    fn constraints(&self) -> AccessConstraints {
        // A 6551 is on an 8-bit bus. A 16-bit read of the data register is not
        // a thing that can happen, and accepting one would invent a byte order.
        AccessConstraints::word(Width::U8, Endian::Little)
    }
}

impl Device for Acia {
    fn class(&self) -> &'static DeviceClass {
        &ACIA_CLASS
    }

    fn realize(&self, _ctx: &mut RealizeCtx<'_>) -> Result<()> {
        // Nothing outward: a `map` statement places the region.
        Ok(())
    }

    fn reset(&self, _kind: ResetKind) {
        // "Upon reset, the Command Register and the Control Register are
        // cleared (all bits set to 0). The Status Register is cleared with the
        // exception of … Data Set Ready and Data Carrier Detect … and the
        // transmitter Empty bit, which is set" — data sheet page 15. Both
        // kinds of reset on a board like this pull RESB.
        *self.regs.state.lock() = State::default();
        self.regs.refresh_irq();
    }

    fn save(&self, w: &mut ChunkWriter<'_>) -> Result<()> {
        let state = *self.regs.state.lock();
        w.write_u8(state.command)?;
        w.write_u8(state.control)?;
        w.write_u8(state.rdr)?;
        w.write_u8(state.tdr)?;
        w.write_bool(state.rdrf)?;
        w.write_bool(state.tx_full)?;
        w.write_bool(state.irq)
        // The port's queues are deliberately absent: what a user has typed and
        // not yet been read, and what the terminal has shown, are the host's
        // state and not the machine's (`ROADMAP.md` §4.5).
    }

    fn load(&self, r: &mut ChunkReader<'_>) -> Result<()> {
        let state = State {
            command: r.read_u8()?,
            control: r.read_u8()?,
            rdr: r.read_u8()?,
            tdr: r.read_u8()?,
            rdrf: r.read_bool()?,
            tx_full: r.read_bool()?,
            irq: r.read_bool()?,
        };
        *self.regs.state.lock() = state;
        self.regs.refresh_irq();
        Ok(())
    }

    fn region(&self, name: &str) -> Option<RegionRef> {
        // `""` for `map … = acia`, `"regs"` for anyone who prefers to say which.
        matches!(name, "" | "regs").then(|| Arc::clone(&self.region))
    }

    fn connect(&self, port: &str, source: WireSource) -> Result<()> {
        if port != IRQ_PIN {
            return Err(Error::Config {
                at: String::from(port),
                message: alloc::format!("the ACIA drives only `{IRQ_PIN}`"),
            });
        }
        self.connect_irq(source);
        Ok(())
    }

    fn announce(&self, port: &str) {
        if port == IRQ_PIN {
            self.regs.refresh_irq();
        }
    }

    fn is_runnable(&self) -> bool {
        // Not because it executes anything, but because a bit time is real and
        // a tick of its clock domain is one character time. The scheduler hands
        // out those ticks; the alternative is a device reading a clock, which
        // nothing below `host/` may do (`CLAUDE.md`).
        self.regs.paced
    }

    fn run(&self, budget: Budget) -> Consumed {
        // At most one character per call, which is right however many ticks
        // this budget covers: a guest that respects TDRE cannot write the next
        // one until this one has gone. A guest that ignores TDRE — which on a
        // real W65C51N is every guest, because the bit lies — overwrites the
        // transmit register instead, and loses the byte exactly as the silicon
        // would.
        self.regs.transmit();
        // Poll the receiver here as well as on a register read, so that a byte
        // arriving while the guest is busy is latched at the tick it arrived
        // on rather than at the next poll.
        if self.regs.poll_receiver() {
            self.regs.refresh_irq();
        }
        Consumed::new(budget.ticks)
    }
}

impl Instance for Acia {}

/// The `wdc.w65c51n` device class.
pub static ACIA_CLASS: DeviceClass = DeviceClass {
    name: CLASS_NAME,
    version: STATE_VERSION,
    summary: "WDC W65C51N ACIA: data, status, command and control over a character port",
    properties: &[
        PropertySpec {
            name: "port",
            kind: ValueKind::Str,
            required: false,
            summary: "the character port to attach to, by name (default \"console\")",
        },
        PropertySpec {
            name: "paced",
            kind: ValueKind::Bool,
            required: false,
            summary: "whether the transmitter sends one byte per clock tick (default true)",
        },
        PropertySpec {
            name: "erratum",
            kind: ValueKind::Bool,
            required: false,
            summary: "model the W65C51N's permanently-set TDRE bit (default false)",
        },
    ],
    construct: |props| Ok(Box::new(Acia::new(props)?)),
};

/// Add [`ACIA_CLASS`] to a registry.
///
/// # Errors
///
/// [`Error::Config`] if something already claimed the name.
pub fn register(registry: &mut crate::core::Registry) -> Result<()> {
    registry.add(&ACIA_CLASS)
}

/// Bind [`ACIA_CLASS`] into the machine graph.
///
/// # Errors
///
/// [`Error::Config`] if the class is already bound.
pub fn bind(bindings: &mut crate::machine::Bindings) -> Result<()> {
    bindings.bind(CLASS_NAME, |props| Ok(Arc::new(Acia::new(props)?)))
}

/// What the validator should know about `wdc.w65c51n`.
#[must_use]
pub fn schema() -> crate::machine::validate::ClassSchema {
    use crate::machine::validate::{ClassSchema, PortDir, PropSchema};
    ClassSchema::new(CLASS_NAME)
        .prop(PropSchema::new("port", ValueKind::Str))
        .prop(PropSchema::new("paced", ValueKind::Bool))
        .prop(PropSchema::new("erratum", ValueKind::Bool))
        .port(IRQ_PIN, PortDir::Out)
        .region("")
        .region("regs")
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::props::Value;
    use crate::core::state::{MachineShape, Migrations, StateReader, StateWriter};
    use crate::host::chardev::CharPort;
    use alloc::string::ToString;
    use alloc::vec::Vec;

    /// An ACIA with the far end of its port in hand, running flat out.
    fn wired() -> (Acia, Arc<CharPort>) {
        let port = Arc::new(CharPort::new());
        let acia = Acia::with_port(
            Arc::clone(&port) as Arc<dyn CharDevice>,
            "test".to_string(),
            false,
            false,
        );
        (acia, port)
    }

    fn peek(acia: &Acia, index: u64) -> u8 {
        let mut byte = [0u8; 1];
        acia.regs
            .read(index, &mut byte, MemAttrs::DEFAULT)
            .expect("a byte read is legal");
        byte[0]
    }

    fn peek_debug(acia: &Acia, index: u64) -> u8 {
        let mut byte = [0u8; 1];
        acia.regs
            .read(index, &mut byte, MemAttrs::DEBUG)
            .expect("a byte read is legal");
        byte[0]
    }

    fn poke(acia: &Acia, index: u64, value: u8) {
        acia.regs
            .write(index, &[value], MemAttrs::DEFAULT)
            .expect("a byte write is legal");
    }

    /// What RSMON writes at reset: 8-N-1 at 19200, no interrupts.
    fn initialise(acia: &Acia) {
        poke(acia, 3, 0x1f);
        poke(acia, 2, 0x0b);
    }

    #[test]
    fn the_reset_state_is_the_data_sheets() {
        // Data sheet page 15: command and control cleared, status cleared but
        // for the transmitter-empty bit, which is set.
        let (acia, _port) = wired();
        initialise(&acia);
        acia.reset(ResetKind::Cold);
        assert_eq!(peek(&acia, 2), 0x00, "command");
        assert_eq!(peek(&acia, 3), 0x00, "control");
        assert_eq!(peek(&acia, 1), ST_TDRE, "status is TDRE and nothing else");
    }

    #[test]
    fn a_byte_written_to_the_data_register_reaches_the_port() {
        let (acia, port) = wired();
        initialise(&acia);
        poke(&acia, 0, b'A');
        assert_eq!(port.drain(), b"A".to_vec());
        assert_eq!(peek(&acia, 1) & ST_TDRE, ST_TDRE, "unpaced: already gone");
    }

    #[test]
    fn a_byte_from_the_port_sets_rdrf_and_reading_the_data_clears_it() {
        let (acia, port) = wired();
        initialise(&acia);
        assert_eq!(peek(&acia, 1) & ST_RDRF, 0, "nothing waiting");

        port.feed(b"Q");
        assert_eq!(peek(&acia, 1) & ST_RDRF, ST_RDRF);
        assert_eq!(peek(&acia, 0), b'Q');
        assert_eq!(peek(&acia, 1) & ST_RDRF, 0, "reading the data cleared it");
    }

    #[test]
    fn a_byte_the_receiver_has_no_room_for_waits_rather_than_being_lost() {
        // The chip would call this an overrun and drop the older byte. Under a
        // queue-backed port there is nothing to drop: the second byte is still
        // on the host's side and arrives in order. See the module docs.
        let (acia, port) = wired();
        initialise(&acia);
        port.feed(b"AB");
        assert_eq!(peek(&acia, 1) & (ST_RDRF | ST_OVRN), ST_RDRF);
        assert_eq!(
            peek(&acia, 1) & (ST_RDRF | ST_OVRN),
            ST_RDRF,
            "polling again must not report an overrun it invented"
        );
        assert_eq!(peek(&acia, 0), b'A', "the older byte, in order");
        assert_eq!(peek(&acia, 1) & ST_RDRF, ST_RDRF, "and then the newer");
        assert_eq!(peek(&acia, 0), b'B');
        assert_eq!(peek(&acia, 1) & (ST_RDRF | ST_OVRN), 0);
    }

    #[test]
    fn the_erratum_is_a_property_and_the_default_is_the_correct_behaviour() {
        // Paced, so the transmitter really is busy for a while.
        let port = Arc::new(CharPort::new());
        let correct = Acia::with_port(
            Arc::clone(&port) as Arc<dyn CharDevice>,
            "test".to_string(),
            true,
            false,
        );
        initialise(&correct);
        poke(&correct, 0, b'X');
        assert_eq!(
            peek(&correct, 1) & ST_TDRE,
            0,
            "the default models the 6551 contract: TDRE is clear while busy"
        );
        assert!(port.drain().is_empty(), "and the byte has not gone yet");

        let broken = Acia::with_port(
            Arc::new(CharPort::new()) as Arc<dyn CharDevice>,
            "test".to_string(),
            true,
            true,
        );
        initialise(&broken);
        poke(&broken, 0, b'X');
        assert_eq!(
            peek(&broken, 1) & ST_TDRE,
            ST_TDRE,
            "erratum = true: TDRE is set even though the transmitter is full"
        );
        assert!(broken.has_erratum());
    }

    #[test]
    fn the_transmitter_holds_the_byte_until_a_clock_tick_releases_it() {
        let port = Arc::new(CharPort::new());
        let acia = Acia::with_port(
            Arc::clone(&port) as Arc<dyn CharDevice>,
            "test".to_string(),
            true,
            false,
        );
        initialise(&acia);
        poke(&acia, 0, b'H');
        assert!(port.drain().is_empty(), "one character time has not passed");

        let consumed = acia.run(Budget {
            until: crate::core::clock::GlobalTime::from_nanos(0),
            ticks: 1,
        });
        assert_eq!(consumed.ticks, 1, "the domain must advance");
        assert_eq!(port.drain(), b"H".to_vec());
        assert_eq!(acia.status() & ST_TDRE, ST_TDRE);
        assert!(acia.is_paced());
    }

    #[test]
    fn a_port_that_will_not_take_the_byte_keeps_the_guest_waiting() {
        // Back pressure as the hardware would deliver it: TDRE stays clear, so
        // a guest polling it spins rather than losing the character.
        let (acia, port) = wired();
        initialise(&acia);
        port.write(&alloc::vec![b'x'; crate::host::chardev::PORT_CAPACITY]);
        poke(&acia, 0, b'A');
        assert_eq!(acia.status() & ST_TDRE, 0, "still holding it");
        let _ = port.drain();
        acia.regs.transmit();
        assert_eq!(acia.status() & ST_TDRE, ST_TDRE);
        assert_eq!(port.drain(), b"A".to_vec(), "and nothing was lost");
    }

    #[test]
    fn a_programmed_reset_clears_five_command_bits_and_not_the_control() {
        // Table 1's note, which is the one place the programmed reset differs
        // from RESB: the control register survives it.
        let (acia, _port) = wired();
        poke(&acia, 3, 0x1f);
        poke(&acia, 2, 0xff);

        poke(&acia, 1, 0x00); // the data is a don't-care
        assert_eq!(peek(&acia, 2), 0xe0, "bits 7-5 survive, 4-0 are cleared");
        assert_eq!(peek(&acia, 3), 0x1f, "the control register is unchanged");
        assert_eq!(peek(&acia, 1) & ST_OVRN, 0);
    }

    #[test]
    fn the_control_register_is_read_back_and_decoded() {
        let (acia, _port) = wired();
        // $1F: 19200 baud, baud-rate generator, 8 bits, one stop bit — what
        // Ben Eater's board and RSMON both program.
        poke(&acia, 3, 0x1f);
        assert_eq!(peek(&acia, 3), 0x1f);
        assert_eq!(acia.baud(), 19_200);
        assert_eq!(acia.word_length(), 8);
        // Code 0 is the crystal over sixteen: 1.8432 MHz / 16.
        poke(&acia, 3, 0x10);
        assert_eq!(acia.baud(), 115_200);
        // Word length 11 is five bits.
        poke(&acia, 3, 0x70);
        assert_eq!(acia.word_length(), 5);
    }

    #[test]
    fn the_interrupt_follows_the_command_register() {
        let (acia, port) = wired();
        acia.connect_irq(dummy_source());

        // DTR clear disables every interrupt, which is the reset state.
        poke(&acia, 2, 0x00);
        port.feed(b"a");
        let _ = peek(&acia, 1);
        assert_eq!(acia.irq_level(), Level::Low, "DTR = 0 disables them all");
        let _ = peek(&acia, 0);

        // DTR set and IRD clear: the receiver interrupt is enabled.
        poke(&acia, 2, CMD_DTR);
        port.feed(b"b");
        assert_eq!(peek(&acia, 1) & ST_IRQ, ST_IRQ, "bit 7 with the flag");
        assert_eq!(
            acia.irq_level(),
            Level::Low,
            "and reading the status cleared it"
        );

        // RSMON's own $0B has IRD set, so nothing interrupts.
        let _ = peek(&acia, 0);
        poke(&acia, 2, 0x0b);
        port.feed(b"c");
        assert_eq!(peek(&acia, 1) & ST_IRQ, 0);
        assert_eq!(acia.irq_level(), Level::Low);
    }

    #[test]
    fn a_debug_access_changes_nothing() {
        // Invariant 5: a debugger read must not pop a FIFO or clear a flag.
        let (acia, port) = wired();
        initialise(&acia);
        port.feed(b"Z");
        // A debug read of the status does not even poll the port...
        assert_eq!(peek_debug(&acia, 1) & ST_RDRF, 0);
        // ...and once the byte is latched, a debug read of the data leaves it.
        assert_eq!(peek(&acia, 1) & ST_RDRF, ST_RDRF);
        assert_eq!(peek_debug(&acia, 0), b'Z');
        assert_eq!(peek(&acia, 1) & ST_RDRF, ST_RDRF, "still waiting");
        assert_eq!(peek(&acia, 0), b'Z');
        assert_eq!(peek(&acia, 1) & ST_RDRF, 0);

        // A debug write is refused rather than guessed at.
        assert_eq!(
            acia.regs.write(0, b"!", MemAttrs::DEBUG),
            Err(BusError::BadAccess)
        );
        assert!(port.drain().is_empty());
    }

    #[test]
    fn only_byte_accesses_are_accepted() {
        let (acia, _port) = wired();
        assert_eq!(
            acia.regs.read(0, &mut [0u8; 2], MemAttrs::DEFAULT),
            Err(BusError::BadAccess)
        );
        assert_eq!(
            acia.regs.write(0, &[0, 0], MemAttrs::DEFAULT),
            Err(BusError::BadAccess)
        );
        assert_eq!(acia.regs.constraints().min, Width::U8);
        assert_eq!(acia.regs.constraints().max, Width::U8);
    }

    #[test]
    fn the_whole_register_block_is_the_region() {
        let (acia, _port) = wired();
        let region = acia.region("").expect("the default region");
        assert_eq!(region.len(), REGISTER_COUNT);
        assert!(acia.region("regs").is_some());
        assert!(acia.region("data").is_none());
    }

    #[test]
    fn a_snapshot_round_trips_to_identical_state() {
        let (saved, port) = wired();
        initialise(&saved);
        port.feed(b"S");
        assert_eq!(peek(&saved, 1) & ST_RDRF, ST_RDRF);

        let mut shape = MachineShape::new();
        shape.add_device("acia", CLASS_NAME).unwrap();
        let mut w = StateWriter::new(shape);
        {
            let mut chunk = w.chunk("acia", CLASS_NAME, STATE_VERSION).unwrap();
            saved.save(&mut chunk).unwrap();
        }
        let bytes = w.to_vec().unwrap();

        let (restored, _other) = wired();
        let reader = StateReader::new(&bytes).unwrap();
        let chunk = reader
            .load("acia", CLASS_NAME, STATE_VERSION, &Migrations::new())
            .unwrap();
        restored.load(&mut chunk.reader()).unwrap();

        let before: Vec<u8> = (0..4).map(|i| peek_debug(&saved, i)).collect();
        let after: Vec<u8> = (0..4).map(|i| peek_debug(&restored, i)).collect();
        assert_eq!(before, after);
        assert_eq!(peek(&restored, 0), b'S', "the latched byte came across");
    }

    #[test]
    fn properties_are_checked() {
        assert!(Acia::new(&Props::new()).is_ok(), "everything has a default");
        let acia = Acia::new(&Props::new().with("port", "test.acia.props")).expect("a name");
        assert_eq!(acia.port_name(), "test.acia.props");
        assert!(acia.is_paced());
        assert!(!acia.has_erratum());
        ports::close("test.acia.props");

        let acia = Acia::new(&Props::new().with("paced", Value::Bool(false))).expect("unpaced");
        assert!(!acia.is_paced());
        assert!(!acia.is_runnable(), "nothing to schedule");

        let err = Acia::new(&Props::new().with("prot", "console"))
            .expect_err("a typo")
            .to_string();
        assert!(err.contains("prot") && err.contains("port"), "{err}");
    }

    #[test]
    fn the_class_is_registrable_and_describes_itself() {
        let mut registry = crate::core::Registry::new();
        register(&mut registry).expect("a fresh registry");
        let class = registry.get(CLASS_NAME).expect("registered");
        assert_eq!(class.version, STATE_VERSION);
        assert_eq!(class.properties.len(), 3);
        let device = (class.construct)(&Props::new().with("port", "test.acia.registry"))
            .expect("defaults are enough");
        assert_eq!(device.class().name, CLASS_NAME);
        // It drives one pin and no other.
        assert!(device.connect("cts", dummy_source()).is_err());
        ports::close("test.acia.registry");
    }

    /// A wire with one source, so the pin has something to drive.
    fn dummy_source() -> WireSource {
        use crate::core::wire::{Wire, WireId};
        let src = WireId::new(1);
        WireSource::new(Wire::builder().source(src).build_shared(), src)
    }
}
