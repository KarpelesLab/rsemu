//! The Macintosh's Zilog Z8530 SCC: two serial channels, and the two carrier
//! detects a mouse's quadrature arrives on.
//!
//! # Sources
//!
//! * *Zilog Z8030/Z8530 SCC Serial Communications Controller* technical
//!   manual, for the register model: sixteen write registers and sixteen read
//!   registers per channel reached through **one pointer**, which a control
//!   write sets and the next control access consumes.
//! * *Guide to the Macintosh Family Hardware*, 2nd edition, chapter 3, for the
//!   addresses, and the "Serial" chapter for what a Macintosh hangs off the
//!   chip.
//!
//! No emulator source was consulted (`ROADMAP.md` §1).
//!
//! # The decode is four addresses, twice
//!
//! A Macintosh reaches the SCC through two separate windows, because the chip
//! has no read/write pin of its own on this board — the address decides:
//!
//! ```text
//!   $9F_FFF8  channel B control   read      $BF_FFF9  channel B control  write
//!   $9F_FFFA  channel A control   read      $BF_FFFB  channel A control  write
//!   $9F_FFFC  channel B data      read      $BF_FFFD  channel B data     write
//!   $9F_FFFE  channel A data      read      $BF_FFFF  channel A data     write
//! ```
//!
//! `A1` picks the channel and `A2` picks data over control, in both windows,
//! so `(offset >> 1) & 3` decodes either — the read window's addresses are
//! even and the write window's odd, and that parity is the only difference.
//! Only `A1` and `A2` are decoded, so each window repeats every eight bytes
//! through the two megabytes its select covers, which is how
//! `machines/mac-plus.machine` maps them.
//!
//! # What is modelled, and what is not
//!
//! Enough for a ROM to initialise the chip, find no serial device and go on,
//! plus the two `DCD` inputs, because that is where a Macintosh Plus's mouse
//! movement interrupts come from.
//!
//! * **The register pointer and all thirty-two registers.** A control write
//!   with the pointer at zero sets it (and may carry a command in bits 3-5);
//!   the next control access uses it and resets it to zero. `WR9`'s two
//!   top bits reset a channel or the chip.
//! * **`RR0`**, computed: `Tx Buffer Empty` and `Tx Underrun/EOM` always set
//!   because nothing here ever holds a character, `Rx Character Available`
//!   always clear because nothing ever arrives, and `DCD`/`CTS` from the pins.
//! * **`RR1`** as `All Sent` with no error, `RR2` vectored on channel B and
//!   raw on channel A, `RR3`'s interrupt-pending bits on channel A, and the
//!   write registers read back where the manual says they are.
//! * **`DCD` external/status interrupts**, which is the mouse path.
//!
//! # `/INT` is a pin, and the pin is what matters
//!
//! The chip pulls `/INT` when a channel has an interrupt pending, that
//! channel's `WR1` enables the condition, **and** `WR9`'s Master Interrupt
//! Enable is set. All three, and `WR9` is the one register the two channels
//! share, so MIE is a property of the chip rather than of a channel.
//!
//! Software lets go of it by writing `Reset Ext/Status Interrupts` — command 2
//! in bits 5-3 of a control write with the register pointer at zero. That
//! command has to move the *wire*, not just the state behind it. It did not
//! once, and the cost was the whole machine: a Macintosh runs `/INT` straight
//! into `IPL1`, so a handler that did everything the manual asks returned to
//! an interrupt that was still there and was entered again, for ever, the
//! first time anything moved a carrier detect. Every path out of
//! `Shared::write` ends at `Shared::refresh` for that reason, and
//! `src/dev/mac/scc/tests.rs` watches the net rather than the registers.
//!
//! **Not modelled**: any actual serial traffic, the baud-rate generator, the
//! DPLL, SDLC, and `/WREQ` — a Macintosh wires that last to the VIA's `PA7`,
//! and with no DMA and no transmission in progress it sits where the VIA's
//! pull-up puts it, which is what leaving it unwired already gives.

use alloc::boxed::Box;
use alloc::string::String;
use alloc::sync::Arc;
use alloc::vec::Vec;

use crate::core::device::{Device, DeviceClass, RealizeCtx, ResetKind, SinkPin};
use crate::core::error::{BusError, Error, Result};
use crate::core::props::Props;
use crate::core::space::{AccessConstraints, MemAttrs, MemOps, MemResult, Region, RegionRef};
use crate::core::state::{ChunkReader, ChunkWriter, Sink, Source};
use crate::core::sync::{LockRank, Mutex};
use crate::core::value::{Endian, Width};
use crate::core::wire::{FanIn, Level, Resolve, WireId, WireSink, WireSource};
use crate::machine::realize::Instance;
use crate::machine::validate::{ClassSchema, PortDir};

/// The class name a machine description writes.
pub const CLASS_NAME: &str = "mac.scc";

/// The snapshot chunk version. Bump with the encoding, never on its own.
pub const STATE_VERSION: u32 = 1;

/// How many bytes each window decodes before it repeats: `A1` and `A2` only.
pub const WINDOW_SPAN: u64 = 8;

/// The output pin that carries `/INT`, high here when the chip is asking.
const IRQ_PIN: &str = "irq";
/// Channel A's carrier detect — a Macintosh Plus's mouse `X1`.
const DCDA_PIN: &str = "dcda";
/// Channel B's carrier detect — its mouse `Y1`.
const DCDB_PIN: &str = "dcdb";

const LINE_DCDA: u32 = 0;
const LINE_DCDB: u32 = 1;

/// `RR0` bit 3: the carrier-detect input.
const RR0_DCD: u8 = 1 << 3;
/// `RR0` bit 2: the transmit buffer is empty. Always, here.
const RR0_TX_EMPTY: u8 = 1 << 2;
/// `RR0` bit 5: clear to send. Always, here: nothing is plugged in and the
/// line is pulled to its asserted state by the absent peripheral's driver.
const RR0_CTS: u8 = 1 << 5;
/// `RR0` bit 6: transmit underrun / end of message, set after a reset.
const RR0_TX_UNDERRUN: u8 = 1 << 6;

/// `WR1` bit 0: external/status interrupts are enabled for this channel.
const WR1_EXT_IE: u8 = 1 << 0;
/// `WR15` bit 3: a `DCD` transition is one of the external status changes.
const WR15_DCD_IE: u8 = 1 << 3;
/// `WR9` bit 1: the interrupt vector includes a status code.
const WR9_VIS: u8 = 1 << 1;
/// `WR9` bit 3: the master interrupt enable. With it clear the chip keeps its
/// interrupt-pending bits but never pulls `/INT`.
const WR9_MIE: u8 = 1 << 3;
/// `WR9` bit 4: the status code occupies bits 6-4 rather than 3-1.
const WR9_STATUS_HIGH: u8 = 1 << 4;
/// `WR9` bits 7-6: the reset command.
const WR9_RESET: u8 = 0xc0;

/// One channel's registers and the state behind them.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct Channel {
    /// The sixteen write registers, read back where the manual says they are.
    wr: [u8; 16],
    /// `DCD` as the far end is driving it.
    ///
    /// Snapshotted, for the reason `mac.via` gives about its own pins: the
    /// realize sweep that ends a restore re-announces every level, and one
    /// that came back wrong arrives as a transition.
    dcd: bool,
    /// The `DCD` level the last external/status latch caught, which is what
    /// `RR0` reports until `Reset Ext/Status Interrupts` releases it.
    dcd_latched: bool,
    /// An external status change is pending for this channel.
    ext_ip: bool,
}

impl Channel {
    fn fresh() -> Channel {
        Channel {
            wr: [0; 16],
            // Nothing plugged in: the pin floats to its pull-up, which for an
            // active-low `/DCD` means no carrier.
            dcd: true,
            dcd_latched: true,
            ext_ip: false,
        }
    }

    /// `RR0` as software reads it.
    fn rr0(&self) -> u8 {
        let mut value = RR0_TX_EMPTY | RR0_CTS | RR0_TX_UNDERRUN;
        // `/DCD` is active low on the pin and the bit reads the *asserted*
        // sense, so a high pin is a clear bit.
        if !self.dcd_latched {
            value |= RR0_DCD;
        }
        value
    }

    /// Whether this channel is asking for an interrupt.
    fn asserting(&self) -> bool {
        self.ext_ip && self.wr[1] & WR1_EXT_IE != 0
    }
}

/// Everything the chip owns.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct State {
    /// Channel A, then channel B.
    ch: [Channel; 2],
    /// The register pointer, which the whole chip shares.
    pointer: u8,
}

impl State {
    fn fresh() -> State {
        State {
            ch: [Channel::fresh(); 2],
            pointer: 0,
        }
    }

    fn asserting(&self) -> bool {
        self.ch[0].asserting() || self.ch[1].asserting()
    }

    /// Whether `/INT` is pulled.
    ///
    /// The manual gates the pin on `WR9`'s Master Interrupt Enable, and that
    /// bit is *not* per channel — `WR9` is the one register both channels
    /// share. An interrupt-pending bit still sets and `RR3` still shows it;
    /// what MIE decides is whether the chip asks anyone.
    fn irq(&self) -> bool {
        self.ch[0].wr[9] & WR9_MIE != 0 && self.asserting()
    }

    /// The vector `RR2` returns on channel B, modified by status when `WR9`
    /// says to.
    ///
    /// The manual's Table: with no interrupt pending the status code is
    /// `011`; channel A's external/status change is `101` and channel B's is
    /// `001`.
    fn vector(&self) -> u8 {
        let base = self.ch[1].wr[2];
        if self.ch[0].wr[9] & WR9_VIS == 0 {
            return base;
        }
        let status = if self.ch[0].asserting() {
            0b101
        } else if self.ch[1].asserting() {
            0b001
        } else {
            0b011
        };
        if self.ch[0].wr[9] & WR9_STATUS_HIGH != 0 {
            (base & !0x70) | (status << 4)
        } else {
            (base & !0x0e) | (status << 1)
        }
    }

    /// `RR3`, which only channel A answers: the interrupt-pending bits.
    fn rr3(&self) -> u8 {
        let mut value = 0;
        if self.ch[0].asserting() {
            value |= 1 << 3; // channel A external/status
        }
        if self.ch[1].asserting() {
            value |= 1 << 0; // channel B external/status
        }
        value
    }
}

/// The chip, as something an address space can dispatch to.
struct Shared {
    state: Mutex<State>,
    out: Mutex<Option<WireSource>>,
}

impl core::fmt::Debug for Shared {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        let mut s = f.debug_struct("Scc");
        match self.state.try_lock() {
            Some(state) => s.field("state", &*state).finish(),
            None => s.field("state", &"<in use>").finish(),
        }
    }
}

impl Shared {
    /// Drive `/INT` to whatever the state now says, with no lock held.
    fn refresh(&self) {
        let asserting = self.state.lock().irq();
        let out = self.out.lock().clone();
        if let Some(src) = &out {
            src.set(Level::from(asserting));
        }
    }

    /// A level arrived on one of the two carrier-detect inputs.
    fn dcd(&self, line: u32, level: bool) {
        let changed = {
            let mut state = self.state.lock();
            let ch = &mut state.ch[line as usize];
            if ch.dcd == level {
                return;
            }
            ch.dcd = level;
            // The manual: an external status change latches `RR0` and raises
            // the interrupt, and `RR0` keeps reporting the latched value until
            // `Reset Ext/Status Interrupts` unlatches it.
            let enabled = ch.wr[15] & WR15_DCD_IE != 0;
            if enabled {
                ch.dcd_latched = level;
                ch.ext_ip = true;
            } else {
                ch.dcd_latched = level;
            }
            state.asserting()
        };
        let _ = changed;
        self.refresh();
    }

    /// Read `rr` of channel `c`.
    fn read_register(&self, state: &State, c: usize, rr: u8) -> u8 {
        match rr {
            0 => state.ch[c].rr0(),
            // All Sent, and no residue: nothing is in flight to have an error.
            1 => 0x06,
            // "RR2 contains the interrupt vector... When accessed in channel
            // B, the vector includes status information."
            2 => {
                if c == 1 {
                    state.vector()
                } else {
                    state.ch[c].wr[2]
                }
            }
            3 => {
                if c == 0 {
                    state.rr3()
                } else {
                    0
                }
            }
            // The receive buffer, which never holds anything here.
            8 => 0,
            10 => 0,
            // The write registers the manual mirrors into the read space.
            4 => state.ch[c].wr[4],
            5 => state.ch[c].wr[5],
            12 => state.ch[c].wr[12],
            13 => state.ch[c].wr[13],
            14 => state.ch[c].wr[14],
            15 => state.ch[c].wr[15],
            // RR6, RR7, RR9 and RR11 have no meaning outside SDLC.
            _ => 0,
        }
    }

    /// One access, decoded. `data` is the data address rather than control.
    fn read(&self, c: usize, data: bool, debug: bool) -> u8 {
        let mut state = self.state.lock();
        if data {
            // The receive buffer is `RR8` by another name, and it is empty.
            return 0;
        }
        let rr = state.pointer;
        let value = self.read_register(&state, c, rr);
        if !debug {
            // "After the read or write, the pointer resets to zero."
            state.pointer = 0;
        }
        value
    }

    /// One write, decoded.
    ///
    /// **Every** path out of the critical section falls through to
    /// [`Shared::refresh`], and that is not tidiness. A `Reset Ext/Status
    /// Interrupts` command is a *control* write with the pointer at zero, so
    /// an early `return` from that branch left the chip's own state saying it
    /// had stopped asking while `/INT` stayed where it was — and a Macintosh
    /// wires `/INT` straight to `IPL1`, so the processor re-entered the
    /// handler for ever, having done everything the manual asks of it. That
    /// is what a carrier-detect transition used to do to this board
    /// (`docs/platforms/mac-plus.md`).
    fn write(&self, c: usize, data: bool, value: u8) {
        {
            let mut state = self.state.lock();
            self.write_locked(&mut state, c, data, value);
        }
        self.refresh();
    }

    /// The register side of [`Shared::write`], with the lock held.
    fn write_locked(&self, state: &mut State, c: usize, data: bool, value: u8) {
        if data {
            // A character handed to a transmitter nothing is listening to.
            // It leaves immediately, which is why `RR0` never clears its
            // Tx Buffer Empty bit.
            state.ch[c].wr[8] = value;
            return;
        }
        let pointer = state.pointer;
        if pointer == 0 {
            // The manual: with the pointer at zero, bits 2-0 are the next
            // register, and bits 5-3 a command. `Point High` (command 1)
            // adds eight.
            let command = (value >> 3) & 7;
            let mut next = value & 7;
            if command == 1 {
                next += 8;
            }
            state.pointer = next;
            match command {
                // "Reset Ext/Status Interrupts": unlatch `RR0` and drop
                // the pending bit.
                2 => {
                    state.ch[c].dcd_latched = state.ch[c].dcd;
                    state.ch[c].ext_ip = false;
                }
                // "Reset Highest IUS" — nothing here nests, so it is the
                // same as clearing this channel's pending bit.
                5 => state.ch[c].ext_ip = false,
                _ => {}
            }
            return;
        }
        state.pointer = 0;
        state.ch[c].wr[pointer as usize] = value;
        if pointer == 9 {
            // `WR9` is the one register both channels share, and its top
            // two bits are the reset command.
            state.ch[0].wr[9] = value;
            state.ch[1].wr[9] = value;
            match value & WR9_RESET {
                0x40 => reset_channel(&mut state.ch[1]),
                0x80 => reset_channel(&mut state.ch[0]),
                0xc0 => {
                    let dcd = [state.ch[0].dcd, state.ch[1].dcd];
                    *state = State::fresh();
                    for (ch, level) in state.ch.iter_mut().zip(dcd) {
                        ch.dcd = level;
                        ch.dcd_latched = level;
                    }
                }
                _ => {}
            }
        }
    }
}

/// A channel reset: everything but what the far end is driving.
fn reset_channel(ch: &mut Channel) {
    let dcd = ch.dcd;
    *ch = Channel::fresh();
    ch.dcd = dcd;
    ch.dcd_latched = dcd;
}

/// One of the two windows the chip answers in.
#[derive(Debug)]
struct Window {
    shared: Arc<Shared>,
}

impl MemOps for Window {
    fn read(&self, offset: u64, dst: &mut [u8], attrs: MemAttrs) -> MemResult {
        let [byte] = dst else {
            return Err(BusError::BadAccess);
        };
        let select = ((offset >> 1) & 3) as usize;
        // Bit 0 of the select is the channel: 0 is B. Bit 1 is data over
        // control.
        *byte = self
            .shared
            .read(1 - (select & 1), select & 2 != 0, attrs.debug);
        Ok(())
    }

    fn write(&self, offset: u64, src: &[u8], attrs: MemAttrs) -> MemResult {
        let [value] = src else {
            return Err(BusError::BadAccess);
        };
        if attrs.debug {
            // A debug write to a control address would move the register
            // pointer, which changes what the guest's next read means. There
            // is no harmless version of that (invariant 5).
            return Err(BusError::BadAccess);
        }
        let select = ((offset >> 1) & 3) as usize;
        self.shared.write(1 - (select & 1), select & 2 != 0, *value);
        Ok(())
    }

    fn constraints(&self) -> AccessConstraints {
        // An eight-bit part on one lane of the 68000's word bus.
        AccessConstraints::word(Width::U8, Endian::Big)
    }
}

/// One of the two carrier-detect inputs.
#[derive(Debug)]
struct DcdPin {
    shared: Arc<Shared>,
    line: u32,
    inputs: FanIn,
}

impl WireSink for DcdPin {
    fn set_level(&self, src: WireId, _line: u32, level: Level) {
        self.inputs.set(src, level);
        let high = self.inputs.resolve(Resolve::And).is_high();
        self.shared.dcd(self.line, high);
    }
}

/// A Z8530 SCC as a Macintosh decodes it.
#[derive(Debug)]
pub struct Scc {
    shared: Arc<Shared>,
    read_region: RegionRef,
    write_region: RegionRef,
    /// The pins, kept alive here: a net holds only a `Weak` to its sinks.
    pins: Mutex<Vec<Arc<DcdPin>>>,
}

impl Scc {
    /// Build the chip. It takes no properties.
    ///
    /// # Errors
    ///
    /// [`Error::Property`] if a property this class does not know was given.
    pub fn new(props: &Props) -> Result<Scc> {
        props.reader().finish()?;
        Ok(Scc::build())
    }

    /// The same, for a test that has no `Props` to hand.
    #[must_use]
    pub fn build() -> Scc {
        let shared = Arc::new(Shared {
            state: Mutex::with_rank(LockRank::DEVICE, State::fresh()),
            out: Mutex::with_rank(LockRank::LEAF, None),
        });
        let region = |name: &str| -> RegionRef {
            Arc::new(Region::io(
                name,
                WINDOW_SPAN,
                Arc::new(Window {
                    shared: Arc::clone(&shared),
                }) as Arc<dyn MemOps>,
            ))
        };
        let read_region = region("mac.scc.read");
        let write_region = region("mac.scc.write");
        Scc {
            shared,
            read_region,
            write_region,
            pins: Mutex::with_rank(LockRank::LEAF, Vec::new()),
        }
    }

    /// Whether the chip is asserting its interrupt output.
    ///
    /// This is the **pin**, which is `WR9`'s Master Interrupt Enable and the
    /// pending bits together — not just the pending bits. A test that asks
    /// the state instead of the pin cannot see the defect that made a
    /// carrier-detect transition lock a Macintosh Plus up, because the state
    /// was right the whole time and the wire was not.
    #[must_use]
    pub fn irq(&self) -> bool {
        self.shared.state.lock().irq()
    }

    /// The register pointer, which the whole chip shares.
    #[must_use]
    pub fn pointer(&self) -> u8 {
        self.shared.state.lock().pointer
    }

    /// Read a control or data address the way the address space would, for a
    /// test. `channel` is 0 for A and 1 for B.
    #[must_use]
    pub fn peek(&self, channel: usize, data: bool) -> u8 {
        self.shared.read(channel & 1, data, false)
    }

    /// Write one the same way.
    pub fn poke(&self, channel: usize, data: bool, value: u8) {
        self.shared.write(channel & 1, data, value);
    }

    /// Set the level the far end is driving onto a carrier detect.
    pub fn set_dcd(&self, channel: usize, level: bool) {
        self.shared.dcd((channel & 1) as u32, level);
    }

    /// One write register as the chip holds it, for a test.
    ///
    /// Most of them are *not* readable through [`Scc::peek`] — the read and
    /// write register files are different files, and `RR1`, for one, is a
    /// computed status byte that has nothing to do with `WR1`. A test that
    /// wants to know how software configured the chip has to come in this way.
    #[must_use]
    pub fn write_register(&self, channel: usize, index: usize) -> u8 {
        self.shared.state.lock().ch[channel & 1].wr[index & 15]
    }
}

impl Device for Scc {
    fn class(&self) -> &'static DeviceClass {
        &SCC_CLASS
    }

    fn realize(&self, _ctx: &mut RealizeCtx<'_>) -> Result<()> {
        // Nothing outward: `map` statements place the two windows.
        Ok(())
    }

    fn reset(&self, _kind: ResetKind) {
        {
            let mut state = self.shared.state.lock();
            let dcd = [state.ch[0].dcd, state.ch[1].dcd];
            *state = State::fresh();
            for (ch, level) in state.ch.iter_mut().zip(dcd) {
                ch.dcd = level;
                ch.dcd_latched = level;
            }
        }
        self.shared.refresh();
    }

    fn save(&self, w: &mut ChunkWriter<'_>) -> Result<()> {
        let state = *self.shared.state.lock();
        w.write_u8(state.pointer)?;
        for ch in &state.ch {
            for reg in ch.wr {
                w.write_u8(reg)?;
            }
            w.write_bool(ch.dcd_latched)?;
            w.write_bool(ch.ext_ip)?;
            w.write_bool(ch.dcd)?;
        }
        Ok(())
    }

    fn load(&self, r: &mut ChunkReader<'_>) -> Result<()> {
        {
            let mut state = self.shared.state.lock();
            let mut next = State::fresh();
            next.pointer = r.read_u8()? & 0xf;
            for ch in &mut next.ch {
                for reg in &mut ch.wr {
                    *reg = r.read_u8()?;
                }
                ch.dcd_latched = r.read_bool()?;
                ch.ext_ip = r.read_bool()?;
                ch.dcd = r.read_bool()?;
            }
            *state = next;
        }
        self.shared.refresh();
        Ok(())
    }

    fn region(&self, name: &str) -> Option<RegionRef> {
        match name {
            "" | "read" => Some(Arc::clone(&self.read_region)),
            "write" => Some(Arc::clone(&self.write_region)),
            _ => None,
        }
    }

    fn connect(&self, port: &str, source: WireSource) -> Result<()> {
        if port != IRQ_PIN {
            return Err(Error::Config {
                at: String::from(port),
                message: String::from("an SCC drives `irq`"),
            });
        }
        *self.shared.out.lock() = Some(source);
        self.shared.refresh();
        Ok(())
    }

    fn announce(&self, _port: &str) {
        self.shared.refresh();
    }

    fn sink(&self, port: &str, sources: &[WireId]) -> Option<SinkPin> {
        let line = match port {
            DCDA_PIN => LINE_DCDA,
            DCDB_PIN => LINE_DCDB,
            _ => return None,
        };
        let pin = Arc::new(DcdPin {
            shared: Arc::clone(&self.shared),
            line,
            inputs: FanIn::new(sources),
        });
        self.pins.lock().push(Arc::clone(&pin));
        Some(SinkPin { sink: pin, line })
    }
}

impl Instance for Scc {}

/// The `mac.scc` device class.
pub static SCC_CLASS: DeviceClass = DeviceClass {
    name: CLASS_NAME,
    version: STATE_VERSION,
    summary: "a Z8530 SCC as a Macintosh decodes it: two channels, one register pointer",
    properties: &[],
    construct: |props| Ok(Box::new(Scc::new(props)?)),
};

/// Add [`SCC_CLASS`] to a registry.
///
/// # Errors
///
/// [`Error::Config`] if something already claimed the name.
pub fn register(registry: &mut crate::core::Registry) -> Result<()> {
    registry.add(&SCC_CLASS)
}

/// Bind [`SCC_CLASS`] into the machine graph.
///
/// # Errors
///
/// [`Error::Config`] if the class is already bound.
pub fn bind(bindings: &mut crate::machine::Bindings) -> Result<()> {
    bindings.bind(CLASS_NAME, |props| Ok(Arc::new(Scc::new(props)?)))
}

/// What the validator should know about `mac.scc`.
#[must_use]
pub fn schema() -> ClassSchema {
    ClassSchema::new(CLASS_NAME)
        .region("")
        .region("read")
        .region("write")
        .port(IRQ_PIN, PortDir::Out)
        .port(DCDA_PIN, PortDir::In)
        .port(DCDB_PIN, PortDir::In)
}

#[cfg(test)]
mod tests;
