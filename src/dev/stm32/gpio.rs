//! An STM32 general-purpose I/O port.
//!
//! One class, `st.gpio`, instantiated once per port: an STM32F407 has eleven
//! of them, `GPIOA` through `GPIOK`, identical but for their reset values and
//! their base addresses. Both are properties, so the class knows nothing about
//! port letters and a board that adds a twelfth port writes another `object`
//! rather than another device.
//!
//! # The registers
//!
//! Ten of them in forty bytes, all 32-bit (RM0090 §8.4):
//!
//! | Offset | Register | What it does |
//! | --- | --- | --- |
//! | `0x00` | `MODER` | two bits per pin: input, output, alternate function, analogue |
//! | `0x04` | `OTYPER` | one bit per pin: push-pull or open-drain |
//! | `0x08` | `OSPEEDR` | two bits per pin: slew rate |
//! | `0x0C` | `PUPDR` | two bits per pin: pull-up, pull-down or neither |
//! | `0x10` | `IDR` | what the pins are at. Read-only |
//! | `0x14` | `ODR` | what the port drives, when a pin is an output |
//! | `0x18` | `BSRR` | atomic set and reset. Write-only |
//! | `0x1C` | `LCKR` | freeze a pin's configuration until the next reset |
//! | `0x20` | `AFRL` | `AFR[0]`: four bits each for pins 0–7 |
//! | `0x24` | `AFRH` | `AFR[1]`: four bits each for pins 8–15 |
//!
//! `BSRR` is the interesting one and the reason the port is usable from an
//! interrupt handler: a write of `1 << n` sets pin *n*, a write of
//! `1 << (n + 16)` clears it, and neither is a read-modify-write, so two
//! contexts touching different pins of one port cannot lose each other's
//! changes. `ODR` cannot do that and firmware that uses it is racy on real
//! hardware too. Where a single write asks for both, **set wins** — RM0090
//! §8.4.7, "if both BSx and BRx are set, the BSx bit has priority".
//!
//! `LCKR` is a three-write key sequence (§8.4.8): `LCKK` set with the mask,
//! `LCKK` clear with the same mask, `LCKK` set with the same mask, then a
//! read. Get it right and the named pins' `MODER`, `OTYPER`, `OSPEEDR`,
//! `PUPDR` and `AFR` bits stop accepting writes until reset. Get it wrong and
//! nothing happens, which is exactly the trap firmware falls into.
//!
//! # The pins
//!
//! Each of the sixteen pins is three things in this model:
//!
//! * an output, `p0` … `p15`, which a board wires to whatever is watching —
//!   an LED, a scope, another chip's input;
//! * an external input, `in0` … `in15`, for whatever is driving the pin from
//!   outside when the port is not;
//! * an **alternate-function input**, `af0` … `af15`, which is how a
//!   peripheral gets the pin.
//!
//! That last one is the mux. `MODER` decides which of the three drives the
//! output: `01` (output) drives `ODR`, `10` (alternate function) drives
//! whatever is on `af{n}`, and `00`/`11` (input and analogue) drive nothing,
//! so the pin follows `in{n}`. A machine file wires a USART's transmit pin to
//! `gpioa.af2` and firmware that has not yet set `MODER` and `AFR` sees
//! nothing come out of it — which is the behaviour that makes a missing
//! `GPIO_Init` look like the bug it is.
//!
//! What is **not** modelled: `AFR`'s *value*. Selecting AF7 rather than AF8 on
//! a pin picks which peripheral of several the mux connects, and this model
//! has one `af{n}` line rather than sixteen. The nibble is stored and reads
//! back, so firmware configures normally; a board that wires two peripherals
//! to one pin would need the selection honoured, and does not exist yet.
//! `OTYPER`, `OSPEEDR` and `PUPDR` are likewise stored and read back but
//! change no level: open-drain, slew rate and pull resistors are analogue
//! properties of a net this model does not have.
//!
//! # Sources
//!
//! *STM32F405/415, STM32F407/417, STM32F427/437 and STM32F429/439 advanced
//! Arm-based 32-bit MCUs*, ST **RM0090** rev 21, §8 "General-purpose I/Os
//! (GPIO)" — §8.3 for the behaviour and §8.4 for the register map and reset
//! values. No emulator source of any licence was consulted (`ROADMAP.md` §1).

use alloc::boxed::Box;
use alloc::format;
use alloc::string::ToString;
use alloc::sync::Arc;
use alloc::vec::Vec;
use core::fmt;

use crate::core::device::{Device, DeviceClass, PropertySpec, RealizeCtx, ResetKind, SinkPin};
use crate::core::error::BusError;
use crate::core::error::{Error, Result};
use crate::core::props::{Props, ValueKind};
use crate::core::space::{AccessConstraints, MemAttrs, MemOps, MemResult, Region, RegionRef};
use crate::core::state::{ChunkReader, ChunkWriter, Sink, Source};
use crate::core::sync::{AtomicU32, LockRank, Mutex, Ordering};
use crate::core::value::{Endian, Width};
use crate::core::wire::{FanIn, Level, Resolve, WireId, WireSink, WireSource};
use crate::machine::Instance;
use crate::machine::validate::{ClassSchema, PortDir, PropSchema, port_index};

/// The class name a machine description writes.
const CLASS_NAME: &str = "st.gpio";

/// The snapshot chunk version. Bump it with the encoding, never on its own.
const STATE_VERSION: u32 = 1;

/// How many pins a port has. Every STM32 port is sixteen wide, including the
/// ones the package bonds four of.
pub const PINS: u32 = 16;

/// How many bytes the ten registers occupy.
///
/// The port is *allocated* a kilobyte in the memory map; only these forty
/// bytes decode. A board maps `REGISTER_BYTES` and everything above reads as
/// whatever the space's unassigned policy says, which is more honest than
/// mirroring registers across an aperture the chip does not mirror them in.
pub const REGISTER_BYTES: u64 = 0x28;

/// `MODER` pin mode: general-purpose output, driven by `ODR`.
const MODE_OUTPUT: u32 = 0b01;
/// `MODER` pin mode: alternate function, driven by the `af{n}` input.
const MODE_ALTERNATE: u32 = 0b10;

/// `LCKR`'s lock key, bit 16 (RM0090 §8.4.8).
const LCKK: u32 = 1 << 16;

/// The writable half of `LCKR`.
const LCK_MASK: u32 = 0xffff;

// ---------------------------------------------------------------------------
// State
// ---------------------------------------------------------------------------

/// Everything the guest can see or change.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct State {
    moder: u32,
    otyper: u32,
    ospeedr: u32,
    pupdr: u32,
    odr: u32,
    lckr: u32,
    afr: [u32; 2],
    /// How far through `LCKR`'s three-write key sequence we are, and with
    /// which mask. `None` is "not started".
    lock_step: Option<(u8, u32)>,
    /// Whether the configuration registers are frozen. Cleared only by reset.
    locked: bool,
}

/// The reset values of the four configuration registers.
///
/// Properties rather than constants because they differ per port and the class
/// does not know which port it is: `GPIOA` comes up with `PA13`–`PA15` in
/// alternate function for the debug port and `GPIOB` with `PB3`/`PB4`, while
/// every other port comes up all-input (RM0090 §8.4.1, §8.4.3, §8.4.4).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct ResetValues {
    /// `MODER`'s reset value — `0xa800_0000` for `GPIOA`, `0x0000_0280` for
    /// `GPIOB`, zero elsewhere.
    pub moder: u32,
    /// `OSPEEDR`'s reset value — `0x0000_00c0` for `GPIOB`, zero elsewhere.
    pub ospeedr: u32,
    /// `PUPDR`'s reset value — `0x6400_0000` for `GPIOA`, `0x0000_0100` for
    /// `GPIOB`, zero elsewhere.
    pub pupdr: u32,
}

impl State {
    fn reset(values: &ResetValues) -> State {
        State {
            moder: values.moder,
            otyper: 0,
            ospeedr: values.ospeedr,
            pupdr: values.pupdr,
            odr: 0,
            lckr: 0,
            afr: [0; 2],
            lock_step: None,
            locked: false,
        }
    }

    /// The two `MODER` bits of pin `n`.
    fn mode(&self, n: u32) -> u32 {
        (self.moder >> (n * 2)) & 0b11
    }
}

// ---------------------------------------------------------------------------
// The register block
// ---------------------------------------------------------------------------

/// The register block, as something an address space can dispatch to.
struct Registers {
    state: Mutex<State>,
    reset_values: ResetValues,
    /// Level driven from outside on each pin, and by an alternate-function
    /// peripheral on each pin. **Not guest state**: what is driving a pin from
    /// outside is the other device's business and it will drive it again
    /// (`ROADMAP.md` §4.5).
    ///
    /// Atomics rather than a lock: a pin level is one bit, it is read on every
    /// register access, and a device asserting a pin from inside a write the
    /// port itself issued must not have to enter the port's critical section
    /// to do it.
    pads: Pads,
    /// The sixteen pin outputs, connected at realize time.
    out: Mutex<[Option<WireSource>; PINS as usize]>,
}

/// What is arriving on the pins from outside the port.
#[derive(Debug, Default)]
struct Pads {
    /// One bit per pin: the level an external driver is holding it at.
    external: AtomicU32,
    /// One bit per pin: the level the alternate-function peripheral is
    /// driving.
    alternate: AtomicU32,
}

impl Pads {
    fn get(&self, kind: PadKind) -> u32 {
        match kind {
            PadKind::External => self.external.load(Ordering::Acquire),
            PadKind::Alternate => self.alternate.load(Ordering::Acquire),
        }
    }

    fn set(&self, kind: PadKind, n: u32, high: bool) {
        let slot = match kind {
            PadKind::External => &self.external,
            PadKind::Alternate => &self.alternate,
        };
        let bit = 1u32 << n;
        if high {
            slot.fetch_or(bit, Ordering::Release);
        } else {
            slot.fetch_and(!bit, Ordering::Release);
        }
    }
}

impl fmt::Debug for Registers {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let mut s = f.debug_struct("Registers");
        match self.state.try_lock() {
            Some(state) => s.field("state", &*state),
            None => s.field("state", &"<locked>"),
        };
        s.field("reset", &self.reset_values).finish()
    }
}

impl Registers {
    /// What each pin is at: `ODR` where the pin is an output, the alternate
    /// function's level where `MODER` says so, and whatever is driving it from
    /// outside elsewhere.
    fn pin_levels(&self, state: &State) -> u32 {
        let external = self.pads.get(PadKind::External);
        let alternate = self.pads.get(PadKind::Alternate);
        let mut out = 0u32;
        for n in 0..PINS {
            let bit = 1u32 << n;
            let level = match state.mode(n) {
                MODE_OUTPUT => state.odr & bit,
                MODE_ALTERNATE => alternate & bit,
                // Input (`00`) and analogue (`11`): the port drives nothing,
                // so the pin is whatever the outside world has it at.
                _ => external & bit,
            };
            out |= level;
        }
        out
    }

    /// Drive the pin outputs to whatever the port now says.
    ///
    /// Called with **no lock held**: a sink is free to call back into this
    /// device, and the re-entrancy contract is that outward calls happen after
    /// the critical section rather than inside it (`CLAUDE.md`, "Concurrency").
    fn refresh_pins(&self) {
        let levels = {
            let state = self.state.lock();
            self.pin_levels(&state)
        };
        let sources: Vec<Option<WireSource>> = self.out.lock().clone().into_iter().collect();
        for (n, source) in sources.iter().enumerate() {
            let Some(source) = source else { continue };
            source.set(Level::from_bool(levels & (1 << n) != 0));
        }
    }

    /// Whether pin `n`'s configuration is frozen by `LCKR`.
    fn is_locked(state: &State, n: u32) -> bool {
        state.locked && state.lckr & (1 << n) != 0
    }

    /// Apply `value` to a two-bits-per-pin configuration register, leaving
    /// locked pins alone.
    fn write_paired(state: &State, old: u32, value: u32) -> u32 {
        let mut out = value;
        for n in 0..PINS {
            if Registers::is_locked(state, n) {
                let mask = 0b11 << (n * 2);
                out = (out & !mask) | (old & mask);
            }
        }
        out
    }

    /// The same for a one-bit-per-pin register.
    fn write_single(state: &State, old: u32, value: u32) -> u32 {
        let mut out = value & 0xffff;
        for n in 0..PINS {
            if Registers::is_locked(state, n) {
                let mask = 1 << n;
                out = (out & !mask) | (old & mask);
            }
        }
        out
    }

    /// The same for one of the two four-bits-per-pin alternate-function
    /// registers. `half` is 0 for pins 0–7 and 1 for pins 8–15.
    fn write_afr(state: &State, old: u32, value: u32, half: u32) -> u32 {
        let mut out = value;
        for slot in 0..8u32 {
            let n = half * 8 + slot;
            if Registers::is_locked(state, n) {
                let mask = 0xf << (slot * 4);
                out = (out & !mask) | (old & mask);
            }
        }
        out
    }

    /// `LCKR`'s three-write key sequence, then a read (RM0090 §8.4.8).
    ///
    /// Any write that does not continue the sequence abandons it. That is what
    /// makes the lock hard to arm by accident and easy to fail to arm on
    /// purpose.
    fn write_lckr(state: &mut State, value: u32) {
        if state.locked {
            return;
        }
        let mask = value & LCK_MASK;
        let key = value & LCKK != 0;
        state.lock_step = match (state.lock_step, key) {
            // Write 1: LCKK set.
            (None, true) => Some((1, mask)),
            // Write 2: LCKK clear, same mask.
            (Some((1, m)), false) if m == mask => Some((2, mask)),
            // Write 3: LCKK set, same mask. The next *read* commits it.
            (Some((2, m)), true) if m == mask => Some((3, mask)),
            // Anything else restarts, or gives up.
            (_, true) => Some((1, mask)),
            (_, false) => None,
        };
        // Only the mask is kept: `LCKK` reads back the *lock*, not the last
        // thing written to it, so a half-finished sequence never looks armed.
        state.lckr = mask;
    }

    /// Reading `LCKR` after the third write is what commits the lock.
    fn read_lckr(state: &mut State, debug: bool) -> u32 {
        // The read is the fourth step of the sequence and the one that
        // commits. A *debug* read must not: it would arm a lock nobody asked
        // for (`ROADMAP.md` §15, invariant 5).
        if !debug
            && !state.locked
            && let Some((3, mask)) = state.lock_step
        {
            state.locked = true;
            state.lckr = mask;
            state.lock_step = None;
        }
        state.lckr | if state.locked { LCKK } else { 0 }
    }

    /// Read one register. Returns the value and whether a pin level may have
    /// moved.
    fn read_register(&self, offset: u64, debug: bool) -> (u32, bool) {
        let mut state = self.state.lock();
        let value = match offset {
            0x00 => state.moder,
            0x04 => state.otyper,
            0x08 => state.ospeedr,
            0x0c => state.pupdr,
            // IDR is the pins themselves, not `ODR`: a pin shorted low reads
            // low however hard the port drives it high. Read-only, and it
            // clears nothing, so a debug read of it is free.
            0x10 => self.pin_levels(&state),
            0x14 => state.odr,
            // "These bits are write-only and can be accessed in word, half-word
            // or byte mode. A read to these bits returns the value 0x0000."
            0x18 => 0,
            0x1c => Registers::read_lckr(&mut state, debug),
            0x20 => state.afr[0],
            0x24 => state.afr[1],
            _ => 0,
        };
        // The only read with a side effect is `LCKR`'s commit, and it changes
        // no pin.
        (value, false)
    }

    /// Write one register. Returns whether a pin level may have moved.
    fn write_register(&self, offset: u64, value: u32) -> bool {
        let mut state = self.state.lock();
        match offset {
            0x00 => state.moder = Registers::write_paired(&state, state.moder, value),
            0x04 => state.otyper = Registers::write_single(&state, state.otyper, value),
            0x08 => state.ospeedr = Registers::write_paired(&state, state.ospeedr, value),
            0x0c => state.pupdr = Registers::write_paired(&state, state.pupdr, value),
            // IDR is read-only.
            0x10 => return false,
            0x14 => state.odr = value & 0xffff,
            0x18 => {
                // The atomic set/reset register. Reset first, then set, so a
                // write naming a pin in both halves leaves it set — "if both
                // BSx and BRx are set, the BSx bit has priority" (§8.4.7).
                let odr = (state.odr & !(value >> 16)) | (value & 0xffff);
                state.odr = odr & 0xffff;
            }
            0x1c => Registers::write_lckr(&mut state, value),
            0x20 => state.afr[0] = Registers::write_afr(&state, state.afr[0], value, 0),
            0x24 => state.afr[1] = Registers::write_afr(&state, state.afr[1], value, 1),
            _ => return false,
        }
        true
    }
}

impl MemOps for Registers {
    fn read(&self, offset: u64, dst: &mut [u8], attrs: MemAttrs) -> MemResult {
        let [a, b, c, d] = dst else {
            return Err(BusError::BadAccess);
        };
        let (value, moved) = self.read_register(offset & !3, attrs.debug);
        let bytes = value.to_le_bytes();
        (*a, *b, *c, *d) = (bytes[0], bytes[1], bytes[2], bytes[3]);
        if moved {
            self.refresh_pins();
        }
        Ok(())
    }

    fn write(&self, offset: u64, src: &[u8], attrs: MemAttrs) -> MemResult {
        let [a, b, c, d] = src else {
            return Err(BusError::BadAccess);
        };
        if attrs.debug {
            // A debug write to `BSRR` or `ODR` would move a pin, and one to
            // `LCKR` would advance the key sequence. Neither can be made
            // harmless, so it is refused rather than guessed at
            // (`ROADMAP.md` §15, invariant 5).
            return Err(BusError::BadAccess);
        }
        let value = u32::from_le_bytes([*a, *b, *c, *d]);
        if self.write_register(offset & !3, value) {
            self.refresh_pins();
        }
        Ok(())
    }

    fn constraints(&self) -> AccessConstraints {
        // "The peripheral registers can be written in word, half-word or byte
        // mode" is true of `BSRR` and `ODR` alone, and a narrow access to the
        // rest is an implementation-defined mess nobody's firmware does.
        // One width, aligned, is the honest model.
        AccessConstraints::word(Width::U32, Endian::Little)
    }
}

// ---------------------------------------------------------------------------
// The device
// ---------------------------------------------------------------------------

/// An STM32 GPIO port.
#[derive(Debug)]
pub struct Gpio {
    regs: Arc<Registers>,
    region: RegionRef,
    /// The input pins the machine layer has taken. The device keeps the strong
    /// reference: a net holds its sinks weakly, so a pin nothing else kept
    /// alive would die on handover and the wire would deliver to nothing.
    pins: Mutex<Vec<Arc<PadPin>>>,
}

impl Gpio {
    /// Validate `props` and build the port.
    ///
    /// # Errors
    ///
    /// [`Error::Property`] if a property is of the wrong kind or value, or if
    /// one this class does not know was given.
    pub fn new(props: &Props) -> Result<Gpio> {
        let mut r = props.reader();
        let moder = r.or_range("moder-reset", 0u64, 0..=u64::from(u32::MAX))? as u32;
        let ospeedr = r.or_range("ospeedr-reset", 0u64, 0..=u64::from(u32::MAX))? as u32;
        let pupdr = r.or_range("pupdr-reset", 0u64, 0..=u64::from(u32::MAX))? as u32;
        r.finish()?;
        Ok(Gpio::with_reset(ResetValues {
            moder,
            ospeedr,
            pupdr,
        }))
    }

    /// Build one with the given reset values — the route a test takes.
    #[must_use]
    pub fn with_reset(reset_values: ResetValues) -> Gpio {
        let regs = Arc::new(Registers {
            state: Mutex::with_rank(LockRank::DEVICE, State::reset(&reset_values)),
            reset_values,
            pads: Pads::default(),
            out: Mutex::with_rank(LockRank::WIRE, [const { None }; PINS as usize]),
        });
        let region = Arc::new(Region::io(
            "gpio",
            REGISTER_BYTES,
            Arc::clone(&regs) as Arc<dyn MemOps>,
        ));
        Gpio {
            regs,
            region,
            pins: Mutex::with_rank(LockRank::DEVICE, Vec::new()),
        }
    }

    /// What each pin is at, one bit per pin.
    #[must_use]
    pub fn pin_levels(&self) -> u32 {
        let state = self.regs.state.lock();
        self.regs.pin_levels(&state)
    }

    /// Drive pin `n` from outside the port.
    ///
    /// Ignored where `MODER` has made the pin an output — a port driving a pin
    /// wins over whatever is on the other end of it, which is what a short
    /// circuit is and what this model does not have.
    pub fn set_external(&self, n: u32, level: bool) {
        if n >= PINS {
            return;
        }
        self.regs.pads.set(PadKind::External, n, level);
        self.regs.refresh_pins();
    }

    /// Drive pin `n`'s alternate function.
    pub fn set_alternate(&self, n: u32, level: bool) {
        if n >= PINS {
            return;
        }
        self.regs.pads.set(PadKind::Alternate, n, level);
        self.regs.refresh_pins();
    }

    /// The alternate-function number `AFR` selects for pin `n`, 0–15.
    ///
    /// Stored and reported; nothing in this model routes on it. See the module
    /// documentation.
    #[must_use]
    pub fn alternate_function(&self, n: u32) -> u8 {
        if n >= PINS {
            return 0;
        }
        let state = self.regs.state.lock();
        let (half, slot) = if n < 8 { (0, n) } else { (1, n - 8) };
        ((state.afr[half] >> (slot * 4)) & 0xf) as u8
    }
}

impl Device for Gpio {
    fn class(&self) -> &'static DeviceClass {
        &CLASS
    }

    fn realize(&self, _ctx: &mut RealizeCtx<'_>) -> Result<()> {
        // Nothing outward: a `map` statement places the region.
        Ok(())
    }

    fn reset(&self, _kind: ResetKind) {
        // Both kinds of reset on an STM32 pull the port's reset, and it is what
        // clears `LCKR` — the lock is documented to survive everything else.
        // The *pad* levels are untouched: what is driving a pin from outside is
        // the other device's state, and inventing a level for an input is how
        // four chips got their reset wrong last week.
        *self.regs.state.lock() = State::reset(&self.regs.reset_values);
        self.regs.refresh_pins();
    }

    fn save(&self, w: &mut ChunkWriter<'_>) -> Result<()> {
        let state = *self.regs.state.lock();
        for value in [
            state.moder,
            state.otyper,
            state.ospeedr,
            state.pupdr,
            state.odr,
            state.lckr,
            state.afr[0],
            state.afr[1],
        ] {
            w.write_u32(value)?;
        }
        match state.lock_step {
            None => w.write_u8(0)?,
            Some((step, mask)) => {
                w.write_u8(step)?;
                w.write_u32(mask)?;
            }
        }
        w.write_bool(state.locked)
        // The pad levels are deliberately absent: they are the levels *other*
        // devices are driving, and each will restore its own and drive them
        // again (`ROADMAP.md` §4.5).
    }

    fn load(&self, r: &mut ChunkReader<'_>) -> Result<()> {
        let mut state = State::reset(&self.regs.reset_values);
        state.moder = r.read_u32()?;
        state.otyper = r.read_u32()?;
        state.ospeedr = r.read_u32()?;
        state.pupdr = r.read_u32()?;
        state.odr = r.read_u32()?;
        state.lckr = r.read_u32()?;
        state.afr[0] = r.read_u32()?;
        state.afr[1] = r.read_u32()?;
        let step = r.read_u8()?;
        state.lock_step = match step {
            0 => None,
            1..=3 => Some((step, r.read_u32()?)),
            other => {
                return Err(Error::State(format!(
                    "snapshot is at step {other} of a three-write lock sequence"
                )));
            }
        };
        state.locked = r.read_bool()?;
        *self.regs.state.lock() = state;
        self.regs.refresh_pins();
        Ok(())
    }

    fn region(&self, name: &str) -> Option<RegionRef> {
        matches!(name, "" | "regs").then(|| Arc::clone(&self.region))
    }

    fn connect(&self, port: &str, source: WireSource) -> Result<()> {
        let n = port_index(port, "p", PINS).ok_or_else(|| Error::Config {
            at: port.to_string(),
            message: format!("a GPIO port drives `p0`…`p{}`", PINS - 1),
        })?;
        self.regs.out.lock()[n as usize] = Some(source);
        self.regs.refresh_pins();
        Ok(())
    }

    fn announce(&self, port: &str) {
        if port_index(port, "p", PINS).is_some() {
            self.regs.refresh_pins();
        }
    }

    fn sink(&self, port: &str, sources: &[WireId]) -> Option<SinkPin> {
        let (n, kind) = match port_index(port, "in", PINS) {
            Some(n) => (n, PadKind::External),
            None => (port_index(port, "af", PINS)?, PadKind::Alternate),
        };
        let pin = Arc::new(PadPin::new(Arc::clone(&self.regs), n, kind, sources));
        self.pins.lock().push(Arc::clone(&pin));
        Some(SinkPin { sink: pin, line: n })
    }
}

impl Instance for Gpio {}

/// The `st.gpio` device class.
pub static CLASS: DeviceClass = DeviceClass {
    name: CLASS_NAME,
    version: STATE_VERSION,
    summary: "STM32 GPIO port: MODER/OTYPER/OSPEEDR/PUPDR, IDR/ODR, the atomic BSRR, LCKR and AFR",
    properties: &[
        PropertySpec {
            name: "moder-reset",
            kind: ValueKind::Uint,
            required: false,
            summary: "MODER's reset value (0xa8000000 for GPIOA, 0x280 for GPIOB, else 0)",
        },
        PropertySpec {
            name: "ospeedr-reset",
            kind: ValueKind::Uint,
            required: false,
            summary: "OSPEEDR's reset value (0xc0 for GPIOB, else 0)",
        },
        PropertySpec {
            name: "pupdr-reset",
            kind: ValueKind::Uint,
            required: false,
            summary: "PUPDR's reset value (0x64000000 for GPIOA, 0x100 for GPIOB, else 0)",
        },
    ],
    construct: |props| Ok(Box::new(Gpio::new(props)?)),
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
    bindings.bind(CLASS_NAME, |props| Ok(Arc::new(Gpio::new(props)?)))
}

/// What the validator should know about `st.gpio`.
#[must_use]
pub fn schema() -> ClassSchema {
    ClassSchema::new(CLASS_NAME)
        .prop(PropSchema::new("moder-reset", ValueKind::Uint).range(0, u64::from(u32::MAX)))
        .prop(PropSchema::new("ospeedr-reset", ValueKind::Uint).range(0, u64::from(u32::MAX)))
        .prop(PropSchema::new("pupdr-reset", ValueKind::Uint).range(0, u64::from(u32::MAX)))
        .region("")
        .region("regs")
        // `p{n}` is the pin; `in{n}` is what drives it from outside when the
        // port does not; `af{n}` is the peripheral the mux selects.
        .port_bank("p", PortDir::Out, PINS)
        .port_bank("in", PortDir::In, PINS)
        .port_bank("af", PortDir::In, PINS)
}

// ---------------------------------------------------------------------------
// Input pins
// ---------------------------------------------------------------------------

/// Which of a pin's two inputs a [`PadPin`] drives.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum PadKind {
    /// Whatever is on the other end of the pin.
    External,
    /// The peripheral `MODER`'s alternate-function mode selects.
    Alternate,
}

/// One of a port's input pins, as something a wire can drive.
///
/// Keeps a [`FanIn`] and wire-ORs its sources, because a wire hands each sink
/// the level of the *driver that changed* rather than the resolved level of
/// the net (§4.3).
///
/// It holds the register block, not the [`Gpio`]: the port owns the pin, and a
/// pin that owned the port back would be a reference cycle nothing could drop.
#[derive(Debug)]
pub struct PadPin {
    regs: Arc<Registers>,
    pin: u32,
    kind: PadKind,
    inputs: FanIn,
    resolve: Resolve,
}

impl PadPin {
    fn new(regs: Arc<Registers>, pin: u32, kind: PadKind, sources: &[WireId]) -> PadPin {
        PadPin {
            regs,
            pin,
            kind,
            inputs: FanIn::new(sources),
            resolve: Resolve::Or,
        }
    }

    /// Which pin of the port this is.
    #[must_use]
    pub fn pin(&self) -> u32 {
        self.pin
    }

    /// The per-source levels currently seen.
    #[must_use]
    pub fn inputs(&self) -> &FanIn {
        &self.inputs
    }
}

impl WireSink for PadPin {
    fn set_level(&self, src: WireId, _line: u32, level: Level) {
        self.inputs.set(src, level);
        let high = self.inputs.resolve(self.resolve).is_high();
        // The pad bit first, in one atomic, and the outward call after it: a
        // sink is free to call back into this port.
        self.regs.pads.set(self.kind, self.pin, high);
        self.regs.refresh_pins();
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::props::Value;
    use crate::core::registry::Registry;
    use crate::core::state::{MachineShape, Migrations, StateReader, StateWriter};
    use crate::core::wire::{Wire, WireIdAllocator};

    /// A port with every register at zero — the reset state of `GPIOC`
    /// upwards.
    fn port() -> Gpio {
        Gpio::with_reset(ResetValues::default())
    }

    fn peek(gpio: &Gpio, offset: u64) -> u32 {
        let mut word = [0u8; 4];
        gpio.regs
            .read(offset, &mut word, MemAttrs::DEFAULT)
            .expect("a word read is legal");
        u32::from_le_bytes(word)
    }

    fn peek_debug(gpio: &Gpio, offset: u64) -> u32 {
        let mut word = [0u8; 4];
        gpio.regs
            .read(offset, &mut word, MemAttrs::DEBUG)
            .expect("a word read is legal");
        u32::from_le_bytes(word)
    }

    fn poke(gpio: &Gpio, offset: u64, value: u32) {
        gpio.regs
            .write(offset, &value.to_le_bytes(), MemAttrs::DEFAULT)
            .expect("a word write is legal");
    }

    /// Make pin `n` a push-pull output.
    fn as_output(gpio: &Gpio, n: u32) {
        let moder = peek(gpio, 0x00) | (MODE_OUTPUT << (n * 2));
        poke(gpio, 0x00, moder);
    }

    #[test]
    fn bsrr_sets_and_resets_without_a_read_modify_write() {
        let gpio = port();
        as_output(&gpio, 5);
        as_output(&gpio, 7);
        as_output(&gpio, 9);
        // A write of `1 << n` sets pin n...
        poke(&gpio, 0x18, 1 << 5);
        assert_eq!(peek(&gpio, 0x14), 1 << 5, "ODR");
        // ...and `1 << (n + 16)` clears it, leaving every other pin alone.
        poke(&gpio, 0x18, 1 << 7);
        poke(&gpio, 0x18, 1 << (5 + 16));
        assert_eq!(peek(&gpio, 0x14), 1 << 7);
        // "If both BSx and BRx are set, the BSx bit has priority" — RM0090
        // §8.4.7. Reset first, then set, is what produces that.
        poke(&gpio, 0x18, (1 << 9) | (1 << (9 + 16)));
        assert_eq!(peek(&gpio, 0x14), (1 << 7) | (1 << 9));
        // And it reads as zero: "these bits are write-only".
        assert_eq!(peek(&gpio, 0x18), 0);
    }

    #[test]
    fn moder_decides_which_of_the_three_drives_a_pin() {
        let gpio = port();
        // Input: the pin is whatever the outside world has it at, and `ODR`
        // does not reach it.
        poke(&gpio, 0x14, 0xffff);
        gpio.set_external(3, true);
        assert_eq!(peek(&gpio, 0x10) & (1 << 3), 1 << 3, "IDR follows the pad");
        gpio.set_external(3, false);
        assert_eq!(peek(&gpio, 0x10) & (1 << 3), 0, "not ODR");

        // Output: `ODR` drives it, and the outside world does not.
        as_output(&gpio, 3);
        assert_eq!(peek(&gpio, 0x10) & (1 << 3), 1 << 3);
        gpio.set_external(3, false);
        assert_eq!(peek(&gpio, 0x10) & (1 << 3), 1 << 3, "the port wins");

        // Alternate function: the peripheral drives it. This is how a USART
        // gets its transmit pin, and why firmware that forgets `MODER` sees
        // nothing come out.
        poke(&gpio, 0x00, MODE_ALTERNATE << (3 * 2));
        assert_eq!(peek(&gpio, 0x10) & (1 << 3), 0);
        gpio.set_alternate(3, true);
        assert_eq!(peek(&gpio, 0x10) & (1 << 3), 1 << 3);

        // Analogue drives nothing either.
        poke(&gpio, 0x00, 0b11 << (3 * 2));
        assert_eq!(peek(&gpio, 0x10) & (1 << 3), 0);
    }

    #[test]
    fn a_pin_output_reaches_a_wire() {
        #[derive(Debug, Default)]
        struct Probe {
            level: crate::core::sync::AtomicU32,
        }
        impl WireSink for Probe {
            fn set_level(&self, _src: WireId, _line: u32, level: Level) {
                self.level.store(
                    u32::from(level.is_high()),
                    crate::core::sync::Ordering::Relaxed,
                );
            }
        }

        let gpio = port();
        let ids = WireIdAllocator::new();
        let id = ids.alloc();
        let probe = Arc::new(Probe::default());
        let wire = Wire::builder()
            .source(id)
            .sink(Arc::clone(&probe) as Arc<dyn WireSink>, 0)
            .build_shared();
        Device::connect(&gpio, "p13", WireSource::new(wire, id)).expect("a port drives p13");
        assert!(
            Device::connect(&gpio, "p16", dummy_source()).is_err(),
            "a port is sixteen pins wide"
        );

        let level = || probe.level.load(crate::core::sync::Ordering::Relaxed);
        as_output(&gpio, 13);
        poke(&gpio, 0x18, 1 << 13);
        assert_eq!(level(), 1);
        poke(&gpio, 0x18, 1 << (13 + 16));
        assert_eq!(level(), 0);
    }

    #[test]
    fn an_input_pin_reaches_idr_and_survives_the_handover() {
        let gpio = port();
        let src = WireId::new(1);
        let pin = Device::sink(&gpio, "in9", &[src]).expect("in9");
        assert_eq!(pin.line, 9);
        // A net holds its sinks weakly, so the port has to own this one.
        let weak = Arc::downgrade(&pin.sink);
        drop(pin);
        let alive = weak.upgrade().expect("the port still owns the pin");
        alive.set_level(src, 0, Level::High);
        assert_eq!(peek(&gpio, 0x10), 1 << 9);

        let af = Device::sink(&gpio, "af9", &[src]).expect("af9");
        af.sink.set_level(src, 0, Level::High);
        // Still an input pin, so the alternate function is not selected and
        // changes nothing about what `IDR` reads.
        assert_eq!(peek(&gpio, 0x10), 1 << 9);
        assert!(Device::sink(&gpio, "in16", &[src]).is_none());
        assert!(
            Device::sink(&gpio, "p0", &[src]).is_none(),
            "p0 is an output"
        );
    }

    #[test]
    fn the_lock_needs_its_whole_key_sequence() {
        let gpio = port();
        // Two writes and a read is not the sequence, so nothing locks.
        poke(&gpio, 0x1c, LCKK | 0x0001);
        poke(&gpio, 0x1c, 0x0001);
        assert_eq!(peek(&gpio, 0x1c) & LCKK, 0, "not committed yet");
        poke(&gpio, 0x00, 0xffff_ffff);
        assert_eq!(peek(&gpio, 0x00), 0xffff_ffff, "still writable");

        // The whole sequence: set, clear, set, read.
        poke(&gpio, 0x00, 0);
        poke(&gpio, 0x1c, LCKK | 0x0003);
        poke(&gpio, 0x1c, 0x0003);
        poke(&gpio, 0x1c, LCKK | 0x0003);
        assert_eq!(peek(&gpio, 0x1c) & LCKK, LCKK, "committed by the read");

        // Pins 0 and 1 are frozen; everything above them is not.
        poke(&gpio, 0x00, 0xffff_ffff);
        assert_eq!(
            peek(&gpio, 0x00),
            0xffff_fff0,
            "the locked pins kept their MODER bits"
        );
        poke(&gpio, 0x04, 0xffff);
        assert_eq!(peek(&gpio, 0x04), 0xfffc);
        poke(&gpio, 0x20, 0xffff_ffff);
        assert_eq!(peek(&gpio, 0x20), 0xffff_ff00);
        // `ODR` is not a configuration register and is never locked.
        poke(&gpio, 0x14, 0xffff);
        assert_eq!(peek(&gpio, 0x14), 0xffff);

        // "This register is used to lock the configuration of the port bits
        // when a correct write sequence is applied… until the next MCU reset."
        Device::reset(&gpio, ResetKind::Cold);
        poke(&gpio, 0x00, 0xffff_ffff);
        assert_eq!(peek(&gpio, 0x00), 0xffff_ffff);
    }

    #[test]
    fn a_debug_access_changes_nothing() {
        let gpio = port();
        // Invariant 5. `LCKR`'s commit is the port's one read side effect, and
        // a debug read must not perform it.
        poke(&gpio, 0x1c, LCKK | 0x0001);
        poke(&gpio, 0x1c, 0x0001);
        poke(&gpio, 0x1c, LCKK | 0x0001);
        assert_eq!(peek_debug(&gpio, 0x1c) & LCKK, 0, "not committed by a peek");
        assert_eq!(peek(&gpio, 0x1c) & LCKK, LCKK, "committed by a real read");

        // A debug write to `BSRR` would move a pin and one to `LCKR` would
        // advance the sequence, so it is refused rather than guessed at.
        let gpio = port();
        assert_eq!(
            gpio.regs.write(0x18, &1u32.to_le_bytes(), MemAttrs::DEBUG),
            Err(BusError::BadAccess)
        );
        assert_eq!(peek(&gpio, 0x14), 0);
    }

    #[test]
    fn a_reset_restores_the_ports_own_values_and_not_its_neighbours() {
        // `GPIOA` comes up with PA13–PA15 in alternate function for the debug
        // port; every port above B comes up all-input (RM0090 §8.4.1).
        let porta = Gpio::with_reset(ResetValues {
            moder: 0xa800_0000,
            ospeedr: 0,
            pupdr: 0x6400_0000,
        });
        assert_eq!(peek(&porta, 0x00), 0xa800_0000);
        assert_eq!(peek(&porta, 0x0c), 0x6400_0000);
        poke(&porta, 0x00, 0);
        Device::reset(&porta, ResetKind::Warm);
        assert_eq!(peek(&porta, 0x00), 0xa800_0000);
    }

    #[test]
    fn a_reset_does_not_invent_a_level_for_an_input_pin() {
        let gpio = port();
        gpio.set_external(4, true);
        Device::reset(&gpio, ResetKind::Cold);
        assert_eq!(
            peek(&gpio, 0x10) & (1 << 4),
            1 << 4,
            "whatever is driving a pin is still driving it"
        );
    }

    #[test]
    fn only_a_full_word_is_a_legal_access() {
        let gpio = port();
        let mut byte = [0u8; 1];
        assert_eq!(
            gpio.regs.read(0x14, &mut byte, MemAttrs::DEFAULT),
            Err(BusError::BadAccess)
        );
        assert_eq!(
            gpio.regs.constraints(),
            AccessConstraints::word(Width::U32, Endian::Little)
        );
    }

    #[test]
    fn the_alternate_function_nibble_reads_back() {
        let gpio = port();
        // AF7 on PA2 is USART2_TX on an STM32F407 (DS8626 Table 9).
        poke(&gpio, 0x20, 7 << (2 * 4));
        assert_eq!(gpio.alternate_function(2), 7);
        poke(&gpio, 0x24, 0xb << (5 * 4));
        assert_eq!(gpio.alternate_function(13), 0xb);
        assert_eq!(gpio.alternate_function(0), 0);
    }

    #[test]
    fn a_snapshot_round_trips_to_identical_state() {
        let saved = port();
        as_output(&saved, 12);
        poke(&saved, 0x18, 1 << 12);
        poke(&saved, 0x04, 0x00f0);
        poke(&saved, 0x20, 0x0765_4321);
        poke(&saved, 0x1c, LCKK | 0x0001);
        poke(&saved, 0x1c, 0x0001);
        poke(&saved, 0x1c, LCKK | 0x0001);
        assert_eq!(peek(&saved, 0x1c) & LCKK, LCKK);

        let mut shape = MachineShape::new();
        shape.add_device("gpio", CLASS_NAME).unwrap();
        let mut w = StateWriter::new(shape);
        {
            let mut chunk = w.chunk("gpio", CLASS_NAME, STATE_VERSION).unwrap();
            Device::save(&saved, &mut chunk).unwrap();
        }
        let bytes = w.to_vec().unwrap();

        let restored = port();
        let reader = StateReader::new(&bytes).unwrap();
        let chunk = reader
            .load("gpio", CLASS_NAME, STATE_VERSION, &Migrations::new())
            .unwrap();
        Device::load(&restored, &mut chunk.reader()).unwrap();

        let before: Vec<u32> = (0..10).map(|i| peek_debug(&saved, i * 4)).collect();
        let after: Vec<u32> = (0..10).map(|i| peek_debug(&restored, i * 4)).collect();
        assert_eq!(before, after);
        // And the lock came across as a lock, not as a register value.
        poke(&restored, 0x00, 0xffff_ffff);
        assert_eq!(peek(&restored, 0x00), 0xffff_fffc);
    }

    #[test]
    fn a_property_this_class_does_not_know_is_a_typo() {
        let props = Props::new().with("moder-reset", Value::from(0xa800_0000u64));
        assert_eq!(peek(&Gpio::new(&props).unwrap(), 0x00), 0xa800_0000);
        let props = Props::new().with("moder_reset", Value::from(0u64));
        assert!(Gpio::new(&props).is_err());
    }

    #[test]
    fn the_class_is_registrable_and_constructs_through_the_registry() {
        let mut reg = Registry::new();
        register(&mut reg).unwrap();
        assert!(register(&mut reg).is_err(), "twice is a collision");
        let device = reg.create(CLASS_NAME, &Props::new()).unwrap();
        assert_eq!(device.class().name, CLASS_NAME);
    }

    #[test]
    fn the_schema_and_the_device_agree_about_pins_and_regions() {
        let gpio = port();
        let schema = schema();
        let src = WireId::new(1);
        for n in [0u32, 15] {
            assert!(schema.port_named(&format!("p{n}")).is_some());
            assert!(schema.port_named(&format!("in{n}")).is_some());
            assert!(schema.port_named(&format!("af{n}")).is_some());
            assert!(Device::sink(&gpio, &format!("in{n}"), &[src]).is_some());
            assert!(Device::sink(&gpio, &format!("af{n}"), &[src]).is_some());
        }
        assert!(schema.port_named("p16").is_none());
        assert!(Device::region(&gpio, "").is_some());
        assert!(Device::region(&gpio, "regs").is_some());
        assert!(Device::region(&gpio, "pins").is_none());
    }

    /// A wire with one source, so a pin has something to drive.
    fn dummy_source() -> WireSource {
        let id = WireId::new(9);
        WireSource::new(Wire::builder().source(id).build_shared(), id)
    }
}
