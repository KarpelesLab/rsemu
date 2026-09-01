//! An STM32 USART, on the character-device seam.
//!
//! # Two register maps, one chip
//!
//! ST redesigned this peripheral between the F4 and the F7, and the two
//! layouts are genuinely different rather than one being a superset:
//!
//! | | F1/F2/**F4** (RM0090 §30.6) | F0/F3/F7/L4/**H7** (RM0410 §34.8) |
//! | --- | --- | --- |
//! | status | `SR` at `+0x00` | `ISR` at `+0x1c` |
//! | data | `DR` at `+0x04`, read *and* write | `RDR` at `+0x24`, `TDR` at `+0x28` |
//! | control | `CR1`–`CR3` at `+0x0c`…`+0x14` | `CR1`–`CR3` at `+0x00`…`+0x08` |
//! | baud | `BRR` at `+0x08` | `BRR` at `+0x0c` |
//! | clearing a flag | read `SR`, then read or write the data register | write a one to `ICR` at `+0x20` |
//! | `CR1` layout | `UE` at 13, `TE` at 3, `RE` at 2 | `UE` at 0, `TE` at 3, `RE` at 2 |
//!
//! Firmware written for one does not run on the other, so this is a
//! **construction property**, `variant`, and not something to average out. The
//! flag *positions* inside the status word do agree for the ten bits that
//! matter (`PE`, `FE`, `NF`, `ORE`, `IDLE`, `RXNE`, `TC`, `TXE`, `LBD`/`LBDF`,
//! `CTS`/`CTSIF`), so one set of semantics serves both and only the decode
//! differs.
//!
//! The default is `"f4"`, because [`machines/stm32f407.machine`] is an F407
//! and a property whose default matches the board it ships with is one fewer
//! line in the file.
//!
//! # What the flags mean here
//!
//! * `TXE` — the transmit holding register is free. Writing the data register
//!   clears it; the byte reaches the host on the next tick of this device's
//!   clock domain, and `TXE` sets again then. That is what makes back pressure
//!   real: a host that will not take a byte stalls a guest polling `TXE`,
//!   exactly as a slow wire would.
//! * `TC` — the shift register is empty as well. It sets when the byte
//!   actually goes, and firmware that waits on it before dropping `TE` or
//!   sleeping gets the answer it is waiting for.
//! * `RXNE` — a byte is in the receive register. There is **no FIFO** on an F4
//!   USART, so a second byte arriving before the first is read sets `ORE` and
//!   is lost, which is the overrun firmware is supposed to handle and usually
//!   does not.
//!
//! # The pins
//!
//! **There are none.** A USART's transmit pin carries a bit stream, and this
//! model's data path is [`host::chardev`](crate::host::chardev) — a byte at a
//! time, to a terminal. So `TX` and `RX` are not wires and a board does not
//! connect them to a GPIO's alternate-function inputs; the only pin here is
//! `irq`. Firmware still configures `MODER` and `AFR` for its `TX` pin and
//! that configuration is modelled faithfully by [`gpio`](super::gpio), because
//! firmware does it and expects the registers to read back — it simply is not
//! what carries the data. A peripheral whose pin genuinely *is* a level, an
//! SPI clock or a timer compare output, is the case the GPIO's `af{n}` inputs
//! exist for.
//!
//! # Baud rate
//!
//! `BRR` is stored and reported and **does not change the byte rate**. The
//! device moves at most one byte per tick of its clock domain, and a machine
//! file sets that domain to the character rate it wants. Modelling `BRR`
//! properly would mean a device recomputing its own event period from a
//! divisor, which is a real thing to want and is not what makes a console
//! work; [`Usart::baud_divisor`] reports the programmed value so a board that
//! cares can act on it.
//!
//! # Sources
//!
//! ST **RM0090** rev 21 §30 ("Universal synchronous asynchronous receiver
//! transmitter") for the F4 layout, and ST **RM0410** rev 4 §34 for the F7's.
//! No emulator source of any licence was consulted (`ROADMAP.md` §1).
//!
//! [`machines/stm32f407.machine`]: https://github.com/KarpelesLab/rsemu/blob/master/machines/stm32f407.machine

use alloc::boxed::Box;
use alloc::format;
use alloc::string::{String, ToString};
use alloc::sync::Arc;
use core::fmt;

use crate::core::device::{Device, DeviceClass, PropertySpec, RealizeCtx, ResetKind};
use crate::core::error::BusError;
use crate::core::error::{Error, Result};
use crate::core::props::{Props, ValueKind};
use crate::core::sched::{Budget, Consumed};
use crate::core::space::{AccessConstraints, MemAttrs, MemOps, MemResult, Region, RegionRef};
use crate::core::state::{ChunkReader, ChunkWriter, Sink, Source};
use crate::core::sync::{LockRank, Mutex};
use crate::core::value::{Endian, Width};
use crate::core::wire::{Level, WireSource};
use crate::host::chardev::{CharDevice, ports};
use crate::machine::Instance;
use crate::machine::validate::{ClassSchema, PortDir, PropSchema};

/// The class name a machine description writes.
const CLASS_NAME: &str = "st.usart";

/// The snapshot chunk version. Bump it with the encoding, never on its own.
const STATE_VERSION: u32 = 1;

/// The character port a machine file gets if it names none.
const DEFAULT_PORT: &str = "console";

/// The name of the interrupt output pin.
pub const IRQ_PIN: &str = "irq";

/// How many bytes the register block occupies.
///
/// `0x2c` covers the F7 layout's `TDR` at `+0x28`; the F4 stops at `GTPR`.
/// One number for both, because a board's `map` statement should not have to
/// know which variant it named.
pub const REGISTER_BYTES: u64 = 0x2c;

// -- status flags, in the positions both layouts agree on --------------------

/// Parity error.
const SR_PE: u32 = 1 << 0;
/// Framing error.
const SR_FE: u32 = 1 << 1;
/// Noise detected.
const SR_NF: u32 = 1 << 2;
/// Overrun: a byte arrived while the last one was still unread.
const SR_ORE: u32 = 1 << 3;
/// The line has been idle.
const SR_IDLE: u32 = 1 << 4;
/// The receive register holds a byte.
const SR_RXNE: u32 = 1 << 5;
/// Transmission complete: the shift register is empty too.
const SR_TC: u32 = 1 << 6;
/// The transmit holding register is free.
const SR_TXE: u32 = 1 << 7;

/// The flags that come up set: nothing to send, and the last nothing was sent.
const SR_RESET: u32 = SR_TC | SR_TXE;

/// `ICR`'s writable bits on an F7 — the flags a write of one clears. `TXE` and
/// `RXNE` are not among them: they follow the data registers.
const ICR_MASK: u32 = SR_PE | SR_FE | SR_NF | SR_ORE | SR_IDLE | SR_TC;

// -- control bits ------------------------------------------------------------

/// `CR1.RE` — receiver enable. Same bit in both layouts.
const CR1_RE: u32 = 1 << 2;
/// `CR1.TE` — transmitter enable. Same bit in both layouts.
const CR1_TE: u32 = 1 << 3;
/// `CR1.IDLEIE`.
const CR1_IDLEIE: u32 = 1 << 4;
/// `CR1.RXNEIE`.
const CR1_RXNEIE: u32 = 1 << 5;
/// `CR1.TCIE`.
const CR1_TCIE: u32 = 1 << 6;
/// `CR1.TXEIE`.
const CR1_TXEIE: u32 = 1 << 7;
/// `CR1.PEIE`.
const CR1_PEIE: u32 = 1 << 8;
/// `CR1.UE` on the F4: bit 13.
const CR1_UE_F4: u32 = 1 << 13;
/// `CR1.UE` on the F7: bit 0.
const CR1_UE_F7: u32 = 1 << 0;
/// `CR3.EIE` — the error interrupt, which is what makes `ORE` raise one.
const CR3_EIE: u32 = 1 << 0;

// ---------------------------------------------------------------------------
// Variant
// ---------------------------------------------------------------------------

/// Which register layout this instance has.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Variant {
    /// The F1/F2/F4 map: `SR` at `+0x00` and one `DR` at `+0x04`.
    F4,
    /// The F0/F3/F7/L4/H7 map: `CR1` at `+0x00`, `ISR`/`ICR`/`RDR`/`TDR` at
    /// the top.
    F7,
}

impl Variant {
    /// The spelling a machine file writes.
    #[must_use]
    pub fn as_str(self) -> &'static str {
        match self {
            Variant::F4 => "f4",
            Variant::F7 => "f7",
        }
    }

    /// Which bit of `CR1` is `UE`.
    fn ue(self) -> u32 {
        match self {
            Variant::F4 => CR1_UE_F4,
            Variant::F7 => CR1_UE_F7,
        }
    }
}

// ---------------------------------------------------------------------------
// State
// ---------------------------------------------------------------------------

/// Everything the guest can see or change.
#[derive(Debug, Default, Clone, Copy, PartialEq, Eq)]
struct State {
    cr1: u32,
    cr2: u32,
    cr3: u32,
    brr: u32,
    gtpr: u32,
    /// The status flags, in the shared bit positions.
    sr: u32,
    /// The byte the receiver is holding, valid while `RXNE` is set.
    rdr: u8,
    /// The byte the transmitter is holding, valid while `TXE` is clear.
    tdr: u8,
    /// Whether the `SR` read that arms the F4's read-read clear has happened.
    /// Meaningless on an F7, which clears through `ICR`.
    sr_read: bool,
    /// The level the interrupt output is at.
    irq: bool,
}

impl State {
    fn reset() -> State {
        State {
            sr: SR_RESET,
            ..State::default()
        }
    }
}

// ---------------------------------------------------------------------------
// The register block
// ---------------------------------------------------------------------------

/// The register block, as something an address space can dispatch to.
struct Registers {
    state: Mutex<State>,
    port: Arc<dyn CharDevice>,
    /// The name the port was opened under, for `Debug` and diagnostics.
    port_name: String,
    variant: Variant,
    /// The interrupt output, connected at realize time.
    irq: Mutex<Option<WireSource>>,
}

impl fmt::Debug for Registers {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let mut s = f.debug_struct("Registers");
        s.field("port", &self.port_name)
            .field("variant", &self.variant);
        match self.state.try_lock() {
            Some(state) => s.field("state", &*state),
            None => s.field("state", &"<locked>"),
        };
        s.finish()
    }
}

impl Registers {
    /// Whether the peripheral is switched on.
    fn enabled(&self, state: &State) -> bool {
        state.cr1 & self.variant.ue() != 0
    }

    /// Whether an enabled interrupt condition is standing (RM0090 Table 149).
    fn irq_pending(&self, state: &State) -> bool {
        if !self.enabled(state) {
            return false;
        }
        let cr1 = state.cr1;
        let sr = state.sr;
        (sr & SR_TXE != 0 && cr1 & CR1_TXEIE != 0)
            || (sr & SR_TC != 0 && cr1 & CR1_TCIE != 0)
            || (sr & SR_RXNE != 0 && cr1 & CR1_RXNEIE != 0)
            || (sr & SR_IDLE != 0 && cr1 & CR1_IDLEIE != 0)
            || (sr & SR_PE != 0 && cr1 & CR1_PEIE != 0)
            // "ORE, NF or FE bits in the USART_SR register" — the error
            // interrupt is `CR3.EIE` and not a `CR1` bit.
            || (sr & (SR_ORE | SR_NF | SR_FE) != 0 && state.cr3 & CR3_EIE != 0)
    }

    /// Drive the interrupt pin to whatever the latch now says.
    ///
    /// Called with **no lock held**: a sink is free to call back into this
    /// device, and the re-entrancy contract is that outward calls happen after
    /// the critical section rather than inside it.
    fn refresh_irq(&self) {
        let level = {
            let mut state = self.state.lock();
            state.irq = self.irq_pending(&state);
            Level::from_bool(state.irq)
        };
        let source = self.irq.lock().clone();
        if let Some(source) = source {
            source.set(level);
        }
    }

    /// Take a byte from the host if the receiver is enabled and free.
    ///
    /// Returns whether anything moved.
    fn poll_receiver(&self) -> bool {
        {
            let state = self.state.lock();
            if !self.enabled(&state) || state.cr1 & CR1_RE == 0 {
                return false;
            }
            if state.sr & SR_RXNE != 0 {
                // The register is still full, so the byte stays on the host's
                // side of the seam rather than being taken and dropped. An
                // overrun is what happens to a byte the *line* delivered, and
                // the host queue is not the line.
                return false;
            }
        }
        // The port's lock is a leaf and this device's is `LockRank::DEVICE`,
        // so taking one inside the other would be the ranked order — but it
        // is not needed here, and a read that answers now is cheaper outside.
        let Some(byte) = self.port.read_byte() else {
            return false;
        };
        let mut state = self.state.lock();
        state.rdr = byte;
        state.sr |= SR_RXNE;
        state.sr_read = false;
        true
    }

    /// Hand the byte the transmitter is holding to the host.
    ///
    /// Returns whether anything moved. A port that will not take it leaves
    /// `TXE` clear, which stalls a guest polling it — back pressure arriving
    /// as the hardware would deliver it, not a dropped character.
    fn transmit(&self) -> bool {
        let byte = {
            let state = self.state.lock();
            if state.sr & SR_TXE != 0 {
                return false;
            }
            state.tdr
        };
        if !self.port.write_byte(byte) {
            return false;
        }
        let mut state = self.state.lock();
        state.sr |= SR_TXE | SR_TC;
        true
    }

    /// The F4's read-read/read-write flag clear (RM0090 §30.6.1).
    ///
    /// `ORE` and the error flags are "cleared by a software sequence: a read
    /// to the USART_SR register followed by a read to the USART_DR register".
    /// An F7 clears them through `ICR` instead and this does nothing there.
    fn clear_after_data_access(state: &mut State) {
        if state.sr_read {
            state.sr &= !(SR_ORE | SR_NF | SR_FE | SR_PE | SR_IDLE);
            state.sr_read = false;
        }
    }

    /// Read the data register: the byte, and `RXNE` cleared.
    fn read_data(&self, state: &mut State, debug: bool) -> u32 {
        if debug {
            return u32::from(state.rdr);
        }
        state.sr &= !SR_RXNE;
        Registers::clear_after_data_access(state);
        u32::from(state.rdr)
    }

    /// Write the data register: the byte is latched and `TXE` goes clear.
    fn write_data(&self, state: &mut State, value: u32) {
        if state.cr1 & CR1_TE == 0 || !self.enabled(state) {
            // A disabled transmitter latches nothing. Firmware that forgot
            // `TE` should see `TXE` stay set and its bytes go nowhere, which
            // is what happens on the part.
            return;
        }
        state.tdr = value as u8;
        state.sr &= !(SR_TXE | SR_TC);
        Registers::clear_after_data_access(state);
    }

    /// Read one register. Returns the value.
    fn read_register(&self, offset: u64, debug: bool) -> u32 {
        let mut state = self.state.lock();
        match (self.variant, offset) {
            (Variant::F4, 0x00) => {
                let sr = state.sr;
                if !debug {
                    // The read that arms the two-step clear. A debug read must
                    // not arm it (`ROADMAP.md` §15, invariant 5).
                    state.sr_read = true;
                }
                sr
            }
            (Variant::F4, 0x04) => self.read_data(&mut state, debug),
            (Variant::F4, 0x08) => state.brr,
            (Variant::F4, 0x0c) => state.cr1,
            (Variant::F4, 0x10) => state.cr2,
            (Variant::F4, 0x14) => state.cr3,
            (Variant::F4, 0x18) => state.gtpr,

            (Variant::F7, 0x00) => state.cr1,
            (Variant::F7, 0x04) => state.cr2,
            (Variant::F7, 0x08) => state.cr3,
            (Variant::F7, 0x0c) => state.brr,
            (Variant::F7, 0x10) => state.gtpr,
            // `ISR` has no read side effect at all on an F7: `ICR` is what
            // clears a flag, which is the whole reason ST changed it.
            (Variant::F7, 0x1c) => state.sr,
            // "The USART_ICR register should be written only …"; it reads as
            // zero.
            (Variant::F7, 0x20) => 0,
            (Variant::F7, 0x24) => self.read_data(&mut state, debug),
            // `TDR` is write-only in effect; a read returns what was last put
            // there, which is what the part does and what nothing relies on.
            (Variant::F7, 0x28) => u32::from(state.tdr),
            // `RTOR` and `RQR` are not modelled; they read as zero, as does
            // anything else in the aperture.
            _ => 0,
        }
    }

    /// Write one register.
    fn write_register(&self, offset: u64, value: u32) {
        let mut state = self.state.lock();
        match (self.variant, offset) {
            (Variant::F4, 0x00) => {
                // "The software sequence … TC bit can also be cleared by
                // writing a '0' to it." Only the two writable flags.
                state.sr &= value | !(SR_TC | SR_RXNE);
            }
            (Variant::F4, 0x04) => self.write_data(&mut state, value),
            (Variant::F4, 0x08) => state.brr = value & 0xffff,
            (Variant::F4, 0x0c) => state.cr1 = value,
            (Variant::F4, 0x10) => state.cr2 = value,
            (Variant::F4, 0x14) => state.cr3 = value,
            (Variant::F4, 0x18) => state.gtpr = value & 0xffff,

            (Variant::F7, 0x00) => state.cr1 = value,
            (Variant::F7, 0x04) => state.cr2 = value,
            (Variant::F7, 0x08) => state.cr3 = value,
            (Variant::F7, 0x0c) => state.brr = value & 0xffff,
            (Variant::F7, 0x10) => state.gtpr = value & 0xffff,
            // `ISR` is read-only on an F7.
            (Variant::F7, 0x1c) => {}
            (Variant::F7, 0x20) => state.sr &= !(value & ICR_MASK),
            (Variant::F7, 0x24) => {}
            (Variant::F7, 0x28) => self.write_data(&mut state, value),
            _ => {}
        }
    }
}

impl MemOps for Registers {
    fn read(&self, offset: u64, dst: &mut [u8], attrs: MemAttrs) -> MemResult {
        let [a, b, c, d] = dst else {
            return Err(BusError::BadAccess);
        };
        if !attrs.debug {
            // A byte the host has delivered should be visible the instant
            // firmware looks, not one scheduler tick later.
            self.poll_receiver();
        }
        let value = self.read_register(offset & !3, attrs.debug);
        let bytes = value.to_le_bytes();
        (*a, *b, *c, *d) = (bytes[0], bytes[1], bytes[2], bytes[3]);
        if !attrs.debug {
            self.refresh_irq();
        }
        Ok(())
    }

    fn write(&self, offset: u64, src: &[u8], attrs: MemAttrs) -> MemResult {
        let [a, b, c, d] = src else {
            return Err(BusError::BadAccess);
        };
        if attrs.debug {
            // A debug write to the data register would put a character on the
            // console and one to `CR1` would change when the guest is next
            // interrupted. Neither can be made harmless, so it is refused
            // rather than guessed at (`ROADMAP.md` §15, invariant 5).
            return Err(BusError::BadAccess);
        }
        self.write_register(offset & !3, u32::from_le_bytes([*a, *b, *c, *d]));
        self.refresh_irq();
        Ok(())
    }

    fn constraints(&self) -> AccessConstraints {
        // "The peripheral registers have to be accessed by words (32 bits)"
        // — RM0090 §30.6. Byte access to `DR` is a thing firmware does anyway
        // on some parts, but this one says words, so words is the model.
        AccessConstraints::word(Width::U32, Endian::Little)
    }
}

// ---------------------------------------------------------------------------
// The device
// ---------------------------------------------------------------------------

/// An STM32 USART.
#[derive(Debug)]
pub struct Usart {
    regs: Arc<Registers>,
    region: RegionRef,
}

impl Usart {
    /// Validate `props` and build the device.
    ///
    /// # Errors
    ///
    /// [`Error::Property`] if a property is of the wrong kind or value, or if
    /// one this class does not know was given.
    pub fn new(props: &Props) -> Result<Usart> {
        let mut r = props.reader();
        let port_name = r.or("port", String::from(DEFAULT_PORT))?;
        let variant = match r.or_enum("variant", "f4", &["f4", "f7"])? {
            "f7" => Variant::F7,
            _ => Variant::F4,
        };
        r.finish()?;
        Ok(Usart::with_port(
            ports::attach(props, &port_name)?,
            port_name,
            variant,
        ))
    }

    /// Build one against a character device the caller already has — the route
    /// a test takes, holding the other end of the port.
    #[must_use]
    pub fn with_port(port: Arc<dyn CharDevice>, port_name: String, variant: Variant) -> Usart {
        let regs = Arc::new(Registers {
            state: Mutex::with_rank(LockRank::DEVICE, State::reset()),
            port,
            port_name,
            variant,
            irq: Mutex::with_rank(LockRank::WIRE, None),
        });
        let region = Arc::new(Region::io(
            "usart",
            REGISTER_BYTES,
            Arc::clone(&regs) as Arc<dyn MemOps>,
        ));
        Usart { regs, region }
    }

    /// Which register layout this instance has.
    #[must_use]
    pub fn variant(&self) -> Variant {
        self.regs.variant
    }

    /// `BRR`'s programmed value — the mantissa and fraction of the divisor.
    ///
    /// Reported, not acted on: see the module documentation.
    #[must_use]
    pub fn baud_divisor(&self) -> u32 {
        self.regs.state.lock().brr
    }

    /// The level the interrupt output is driving. High is "requesting".
    #[must_use]
    pub fn irq_level(&self) -> Level {
        Level::from_bool(self.regs.state.lock().irq)
    }

    /// Connect the interrupt output.
    pub fn connect_irq(&self, source: WireSource) {
        *self.regs.irq.lock() = Some(source);
        self.regs.refresh_irq();
    }
}

impl Device for Usart {
    fn class(&self) -> &'static DeviceClass {
        &CLASS
    }

    fn realize(&self, _ctx: &mut RealizeCtx<'_>) -> Result<()> {
        // Nothing outward: a `map` statement places the region.
        Ok(())
    }

    fn reset(&self, _kind: ResetKind) {
        // Both kinds of reset on an STM32 pull the peripheral's reset line.
        // The port's queues are not touched: what the user has typed is the
        // host's state, not the machine's.
        *self.regs.state.lock() = State::reset();
        self.regs.refresh_irq();
    }

    fn save(&self, w: &mut ChunkWriter<'_>) -> Result<()> {
        let state = *self.regs.state.lock();
        for value in [
            state.cr1, state.cr2, state.cr3, state.brr, state.gtpr, state.sr,
        ] {
            w.write_u32(value)?;
        }
        w.write_u8(state.rdr)?;
        w.write_u8(state.tdr)?;
        w.write_bool(state.sr_read)?;
        w.write_bool(state.irq)
        // The port's queues are deliberately absent: what a user has typed and
        // not yet been read, and what the terminal has shown, are the host's
        // state and not the machine's (`ROADMAP.md` §4.5).
    }

    fn load(&self, r: &mut ChunkReader<'_>) -> Result<()> {
        let mut state = State::reset();
        state.cr1 = r.read_u32()?;
        state.cr2 = r.read_u32()?;
        state.cr3 = r.read_u32()?;
        state.brr = r.read_u32()?;
        state.gtpr = r.read_u32()?;
        state.sr = r.read_u32()?;
        state.rdr = r.read_u8()?;
        state.tdr = r.read_u8()?;
        state.sr_read = r.read_bool()?;
        state.irq = r.read_bool()?;
        *self.regs.state.lock() = state;
        self.regs.refresh_irq();
        Ok(())
    }

    fn region(&self, name: &str) -> Option<RegionRef> {
        matches!(name, "" | "regs").then(|| Arc::clone(&self.region))
    }

    fn connect(&self, port: &str, source: WireSource) -> Result<()> {
        if port != IRQ_PIN {
            return Err(Error::Config {
                at: port.to_string(),
                message: format!("a USART drives one pin, `{IRQ_PIN}`"),
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
        // Not because it executes anything, but because a character time is
        // real: the receiver has to be filled from the host and a refused
        // transmission has to be retried, and the scheduler is the only thing
        // allowed to decide when (`CLAUDE.md`: a device never reads the wall
        // clock). One tick of this device's domain is one character time.
        true
    }

    fn run(&self, budget: Budget) -> Consumed {
        // At most one character per call, which is right however many ticks
        // the budget covers: a board sets the domain to the character rate.
        let moved = self.regs.transmit() | self.regs.poll_receiver();
        if moved {
            self.regs.refresh_irq();
        }
        Consumed::new(budget.ticks)
    }
}

impl Instance for Usart {}

/// The `st.usart` device class.
pub static CLASS: DeviceClass = DeviceClass {
    name: CLASS_NAME,
    version: STATE_VERSION,
    summary: "STM32 USART over a character port, in the F4 (SR/DR) or F7 (ISR/RDR/TDR) layout",
    properties: &[
        PropertySpec {
            name: "port",
            kind: ValueKind::Str,
            required: false,
            summary: "the character port to attach to, by name (default \"console\")",
        },
        PropertySpec {
            name: "variant",
            kind: ValueKind::Str,
            required: false,
            summary: "which register layout: \"f4\" (SR/DR) or \"f7\" (ISR/RDR/TDR)",
        },
    ],
    construct: |props| Ok(Box::new(Usart::new(props)?)),
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
    bindings.bind(CLASS_NAME, |props| Ok(Arc::new(Usart::new(props)?)))
}

/// What the validator should know about `st.usart`.
#[must_use]
pub fn schema() -> ClassSchema {
    ClassSchema::new(CLASS_NAME)
        .prop(PropSchema::new("port", ValueKind::Str))
        .prop(PropSchema::new("variant", ValueKind::Str).values(&["f4", "f7"]))
        .region("")
        .region("regs")
        // An M-profile board wires this straight to the core: `wire
        // usart2.irq -> cpu.irq38`. There is no controller in between.
        .port(IRQ_PIN, PortDir::Out)
}

#[cfg(test)]
mod tests {
    use super::*;
    use alloc::vec::Vec;

    use crate::core::clock::GlobalTime;
    use crate::core::props::Value;
    use crate::core::registry::Registry;
    use crate::core::state::{MachineShape, Migrations, StateReader, StateWriter};
    use crate::core::sync::{AtomicU32, Ordering};
    use crate::core::wire::{Level, Wire, WireId, WireIdAllocator, WireSink};
    use crate::host::chardev::CharPort;

    /// Where each register is, in the variant under test.
    struct Map {
        sr: u64,
        rdr: u64,
        tdr: u64,
        brr: u64,
        cr1: u64,
        cr3: u64,
        icr: Option<u64>,
        /// The `CR1` bit that switches the peripheral on.
        ue: u32,
    }

    const F4_MAP: Map = Map {
        sr: 0x00,
        rdr: 0x04,
        tdr: 0x04,
        brr: 0x08,
        cr1: 0x0c,
        cr3: 0x14,
        icr: None,
        ue: CR1_UE_F4,
    };

    const F7_MAP: Map = Map {
        sr: 0x1c,
        rdr: 0x24,
        tdr: 0x28,
        brr: 0x0c,
        cr1: 0x00,
        cr3: 0x08,
        icr: Some(0x20),
        ue: CR1_UE_F7,
    };

    /// A USART with the far end of its port in hand.
    fn wired(variant: Variant) -> (Usart, Arc<CharPort>, &'static Map) {
        let port = Arc::new(CharPort::new());
        let usart = Usart::with_port(
            Arc::clone(&port) as Arc<dyn CharDevice>,
            "test".to_string(),
            variant,
        );
        let map = match variant {
            Variant::F4 => &F4_MAP,
            Variant::F7 => &F7_MAP,
        };
        (usart, port, map)
    }

    fn peek(usart: &Usart, offset: u64) -> u32 {
        let mut word = [0u8; 4];
        usart
            .regs
            .read(offset, &mut word, MemAttrs::DEFAULT)
            .expect("a word read is legal");
        u32::from_le_bytes(word)
    }

    fn peek_debug(usart: &Usart, offset: u64) -> u32 {
        let mut word = [0u8; 4];
        usart
            .regs
            .read(offset, &mut word, MemAttrs::DEBUG)
            .expect("a word read is legal");
        u32::from_le_bytes(word)
    }

    fn poke(usart: &Usart, offset: u64, value: u32) {
        usart
            .regs
            .write(offset, &value.to_le_bytes(), MemAttrs::DEFAULT)
            .expect("a word write is legal");
    }

    /// Switch the peripheral on with the transmitter and receiver enabled.
    fn enable(usart: &Usart, map: &Map) {
        poke(usart, map.cr1, map.ue | CR1_TE | CR1_RE);
    }

    /// One tick of the device's clock domain — one character time.
    fn tick(usart: &Usart) {
        Device::run(
            usart,
            Budget {
                until: GlobalTime::ZERO,
                ticks: 1,
            },
        );
    }

    #[test]
    fn a_byte_written_to_the_data_register_reaches_the_host() {
        for variant in [Variant::F4, Variant::F7] {
            let (usart, port, map) = wired(variant);
            enable(&usart, map);
            assert_eq!(peek(&usart, map.sr) & SR_TXE, SR_TXE, "{variant:?}: idle");

            poke(&usart, map.tdr, u32::from(b'H'));
            assert_eq!(
                peek(&usart, map.sr) & (SR_TXE | SR_TC),
                0,
                "{variant:?}: the holding register is busy"
            );
            assert!(port.drain().is_empty(), "{variant:?}: not until a tick");

            tick(&usart);
            assert_eq!(port.drain(), b"H", "{variant:?}");
            assert_eq!(
                peek(&usart, map.sr) & (SR_TXE | SR_TC),
                SR_TXE | SR_TC,
                "{variant:?}: and now it is free"
            );
        }
    }

    #[test]
    fn a_disabled_transmitter_latches_nothing() {
        let (usart, port, map) = wired(Variant::F4);
        // `UE` set but `TE` clear: firmware that forgot half its init.
        poke(&usart, map.cr1, map.ue);
        poke(&usart, map.tdr, u32::from(b'X'));
        assert_eq!(peek(&usart, map.sr) & SR_TXE, SR_TXE);
        tick(&usart);
        assert!(port.drain().is_empty());

        // And with `UE` clear, nothing at all works.
        let (usart, port, map) = wired(Variant::F4);
        poke(&usart, map.cr1, CR1_TE);
        poke(&usart, map.tdr, u32::from(b'X'));
        tick(&usart);
        assert!(port.drain().is_empty());
    }

    #[test]
    fn a_byte_from_the_host_arrives_in_the_data_register() {
        for variant in [Variant::F4, Variant::F7] {
            let (usart, port, map) = wired(variant);
            enable(&usart, map);
            port.feed(b"Z");
            // Visible the instant firmware looks, not one tick later.
            assert_eq!(peek(&usart, map.sr) & SR_RXNE, SR_RXNE, "{variant:?}");
            assert_eq!(peek(&usart, map.rdr), u32::from(b'Z'), "{variant:?}");
            assert_eq!(
                peek(&usart, map.sr) & SR_RXNE,
                0,
                "{variant:?}: reading the data register clears RXNE"
            );
        }
    }

    #[test]
    fn a_disabled_receiver_leaves_the_byte_on_the_hosts_side() {
        let (usart, port, map) = wired(Variant::F4);
        poke(&usart, map.cr1, map.ue); // no `RE`
        port.feed(b"Q");
        assert_eq!(peek(&usart, map.sr) & SR_RXNE, 0);
        // The byte was not taken and dropped: enabling the receiver finds it.
        enable(&usart, map);
        assert_eq!(peek(&usart, map.sr) & SR_RXNE, SR_RXNE);
        assert_eq!(peek(&usart, map.rdr), u32::from(b'Q'));
    }

    #[test]
    fn the_two_layouts_clear_a_flag_in_their_own_ways() {
        // The F4: "a read to the USART_SR register followed by a read to the
        // USART_DR register" (RM0090 §30.6.1).
        let (usart, _port, map) = wired(Variant::F4);
        enable(&usart, map);
        usart.regs.state.lock().sr |= SR_ORE;
        assert_eq!(peek(&usart, map.rdr) as u8, 0);
        assert_eq!(
            peek(&usart, map.sr) & SR_ORE,
            SR_ORE,
            "a data read alone does not clear it"
        );
        let _ = peek(&usart, map.sr); // arms the sequence
        let _ = peek(&usart, map.rdr); // completes it
        assert_eq!(peek(&usart, map.sr) & SR_ORE, 0);

        // The F7: write a one to `ICR`. `ISR` has no read side effect at all,
        // which is the whole reason ST changed it.
        let (usart, _port, map) = wired(Variant::F7);
        enable(&usart, map);
        usart.regs.state.lock().sr |= SR_ORE;
        let _ = peek(&usart, map.sr);
        let _ = peek(&usart, map.rdr);
        assert_eq!(
            peek(&usart, map.sr) & SR_ORE,
            SR_ORE,
            "an F7 does not clear on a read sequence"
        );
        poke(&usart, map.icr.unwrap(), SR_ORE);
        assert_eq!(peek(&usart, map.sr) & SR_ORE, 0);
        // `ICR` reads as zero.
        assert_eq!(peek(&usart, map.icr.unwrap()), 0);
    }

    #[test]
    fn the_f4_clears_tc_by_a_write_of_zero() {
        let (usart, _port, map) = wired(Variant::F4);
        enable(&usart, map);
        assert_eq!(peek(&usart, map.sr) & SR_TC, SR_TC);
        poke(&usart, map.sr, !SR_TC);
        assert_eq!(peek(&usart, map.sr) & SR_TC, 0);
        // And a write cannot *set* a flag.
        poke(&usart, map.sr, 0xffff_ffff);
        assert_eq!(peek(&usart, map.sr) & SR_TC, 0);
    }

    #[test]
    fn an_enabled_condition_raises_the_interrupt_pin() {
        #[derive(Debug, Default)]
        struct Probe {
            level: AtomicU32,
        }
        impl WireSink for Probe {
            fn set_level(&self, _src: WireId, _line: u32, level: Level) {
                self.level
                    .store(u32::from(level.is_high()), Ordering::Relaxed);
            }
        }

        let (usart, port, map) = wired(Variant::F4);
        let ids = WireIdAllocator::new();
        let id = ids.alloc();
        let probe = Arc::new(Probe::default());
        let wire = Wire::builder()
            .source(id)
            .sink(Arc::clone(&probe) as Arc<dyn WireSink>, 0)
            .build_shared();
        Device::connect(&usart, IRQ_PIN, WireSource::new(wire, id)).expect("a USART drives irq");
        assert!(
            Device::connect(&usart, "dma", dummy_source()).is_err(),
            "and nothing else"
        );

        enable(&usart, map);
        assert_eq!(probe.level.load(Ordering::Relaxed), 0, "no enable bit set");

        // `RXNEIE` and a byte: this is the pin a machine file wires to the
        // core's `irq38`.
        poke(&usart, map.cr1, map.ue | CR1_TE | CR1_RE | CR1_RXNEIE);
        port.feed(b"!");
        let _ = peek(&usart, map.sr);
        assert_eq!(probe.level.load(Ordering::Relaxed), 1);
        assert_eq!(usart.irq_level(), Level::High);
        let _ = peek(&usart, map.rdr);
        assert_eq!(probe.level.load(Ordering::Relaxed), 0, "serviced");

        // `EIE` is what makes an error raise one; `CR1` has no bit for it.
        usart.regs.state.lock().sr |= SR_ORE;
        usart.regs.refresh_irq();
        assert_eq!(probe.level.load(Ordering::Relaxed), 0);
        poke(&usart, map.cr3, CR3_EIE);
        assert_eq!(probe.level.load(Ordering::Relaxed), 1);

        // And a peripheral that is switched off requests nothing, whatever its
        // flags say.
        poke(&usart, map.cr1, 0);
        assert_eq!(probe.level.load(Ordering::Relaxed), 0);
    }

    #[test]
    fn a_debug_access_changes_nothing() {
        // Invariant 5: a status register is exactly the trap this is for.
        let (usart, port, map) = wired(Variant::F4);
        enable(&usart, map);
        port.feed(b"D");
        // A debug read does not even poll the port...
        assert_eq!(peek_debug(&usart, map.sr) & SR_RXNE, 0);
        // ...and once the byte is latched, a debug read of it leaves `RXNE`.
        assert_eq!(peek(&usart, map.sr) & SR_RXNE, SR_RXNE);
        assert_eq!(peek_debug(&usart, map.rdr), u32::from(b'D'));
        assert_eq!(peek(&usart, map.sr) & SR_RXNE, SR_RXNE, "still waiting");

        // Nor does it arm the F4's two-step clear. A fresh device, because a
        // real `SR` read above has already armed this one.
        let (fresh, _p, map) = wired(Variant::F4);
        enable(&fresh, map);
        fresh.regs.state.lock().sr |= SR_ORE;
        let _ = peek_debug(&fresh, map.sr);
        let _ = peek(&fresh, map.rdr);
        assert_eq!(peek(&fresh, map.sr) & SR_ORE, SR_ORE);

        // A debug write is refused rather than guessed at.
        assert_eq!(
            usart
                .regs
                .write(map.tdr, &u32::from(b'!').to_le_bytes(), MemAttrs::DEBUG),
            Err(BusError::BadAccess)
        );
        assert!(port.drain().is_empty());
    }

    #[test]
    fn only_a_full_word_is_a_legal_access() {
        let (usart, _port, map) = wired(Variant::F4);
        let mut byte = [0u8; 1];
        assert_eq!(
            usart.regs.read(map.sr, &mut byte, MemAttrs::DEFAULT),
            Err(BusError::BadAccess)
        );
        assert_eq!(
            usart.regs.constraints(),
            AccessConstraints::word(Width::U32, Endian::Little)
        );
    }

    #[test]
    fn brr_is_stored_and_reported_and_changes_no_rate() {
        let (usart, port, map) = wired(Variant::F4);
        enable(&usart, map);
        // 168 MHz / 4 APB1, 115200 baud, OVER8=0: mantissa 22, fraction 13.
        poke(&usart, map.brr, (22 << 4) | 13);
        assert_eq!(usart.baud_divisor(), (22 << 4) | 13);
        assert_eq!(peek(&usart, map.brr), (22 << 4) | 13);
        // Still one byte per tick, because the domain is the rate.
        poke(&usart, map.tdr, u32::from(b'a'));
        tick(&usart);
        assert_eq!(port.drain(), b"a");
    }

    #[test]
    fn a_reset_returns_the_documented_flags() {
        let (usart, _port, map) = wired(Variant::F4);
        enable(&usart, map);
        poke(&usart, map.brr, 0x1234);
        poke(&usart, map.tdr, u32::from(b'z'));
        Device::reset(&usart, ResetKind::Cold);
        assert_eq!(
            peek(&usart, map.sr),
            SR_RESET,
            "TC and TXE set, nothing else"
        );
        assert_eq!(peek(&usart, map.brr), 0);
        assert_eq!(peek(&usart, map.cr1), 0);
    }

    #[test]
    fn a_snapshot_round_trips_to_identical_state() {
        let (saved, port, map) = wired(Variant::F4);
        enable(&saved, map);
        poke(&saved, map.brr, (22 << 4) | 13);
        poke(&saved, map.cr3, CR3_EIE);
        port.feed(b"S");
        assert_eq!(peek(&saved, map.sr) & SR_RXNE, SR_RXNE);

        let mut shape = MachineShape::new();
        shape.add_device("usart", CLASS_NAME).unwrap();
        let mut w = StateWriter::new(shape);
        {
            let mut chunk = w.chunk("usart", CLASS_NAME, STATE_VERSION).unwrap();
            Device::save(&saved, &mut chunk).unwrap();
        }
        let bytes = w.to_vec().unwrap();

        let (restored, _other, _) = wired(Variant::F4);
        let reader = StateReader::new(&bytes).unwrap();
        let chunk = reader
            .load("usart", CLASS_NAME, STATE_VERSION, &Migrations::new())
            .unwrap();
        Device::load(&restored, &mut chunk.reader()).unwrap();

        let before: Vec<u32> = (0..7).map(|i| peek_debug(&saved, i * 4)).collect();
        let after: Vec<u32> = (0..7).map(|i| peek_debug(&restored, i * 4)).collect();
        assert_eq!(before, after);
        assert_eq!(
            peek(&restored, map.rdr),
            u32::from(b'S'),
            "the byte came across"
        );
    }

    #[test]
    fn a_property_this_class_does_not_know_is_a_typo() {
        let props = Props::new().with("variant", Value::from("f7"));
        assert_eq!(Usart::new(&props).unwrap().variant(), Variant::F7);
        assert_eq!(Variant::F7.as_str(), "f7");
        assert_eq!(
            Usart::new(&Props::new()).unwrap().variant(),
            Variant::F4,
            "the board this ships with is an F407"
        );
        assert!(Usart::new(&Props::new().with("variant", Value::from("h7"))).is_err());
        assert!(Usart::new(&Props::new().with("varient", Value::from("f4"))).is_err());
    }

    #[test]
    fn the_class_is_registrable_and_agrees_with_its_schema() {
        let mut reg = Registry::new();
        register(&mut reg).unwrap();
        assert!(register(&mut reg).is_err(), "twice is a collision");
        let device = reg.create(CLASS_NAME, &Props::new()).unwrap();
        assert_eq!(device.class().name, CLASS_NAME);

        let schema = schema();
        let port = schema.port_named(IRQ_PIN).expect("irq");
        assert_eq!(port.dir, PortDir::Out);
        assert!(schema.port_named("rx").is_none());

        let (usart, _p, _m) = wired(Variant::F4);
        assert!(Device::region(&usart, "").is_some());
        assert!(Device::region(&usart, "regs").is_some());
        assert!(Device::region(&usart, "fifo").is_none());
    }

    /// A wire with one source, so a pin has something to drive.
    fn dummy_source() -> WireSource {
        let id = WireId::new(9);
        WireSource::new(Wire::builder().source(id).build_shared(), id)
    }
}
