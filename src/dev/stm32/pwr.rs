//! The STM32 power controller.
//!
//! `st.pwr` is the other half of a vendor startup sequence. `SystemClock_Config`
//! turns on the power interface's own clock, writes `PWR_CR.VOS` (or `CR1.VOS`
//! on an L4) to pick a voltage scale, and then **waits for the regulator**:
//! an F42x/F43x spins on `CSR.VOSRDY` and again on `CSR.ODRDY` and
//! `CSR.ODSWRDY` for the over-drive, and an L4 spins until `SR2.VOSF` goes
//! clear. As with [`rcc`](super::rcc), a RAM cell never answers, so the spin
//! never ends.
//!
//! The other thing it owns is one bit: `DBP`, "disable backup domain write
//! protection". Until firmware sets it, the RCC's `BDCR` — the LSE, `RTCSEL`,
//! `RTCEN` — is write-protected and every write to it is dropped.
//!
//! # How `DBP` reaches the RCC
//!
//! **As a wire**, `pwr.dbp -> rcc.dbp`, and not as a handle. On the die it is
//! a signal from the power block to the backup-domain interface, so a level on
//! a net is the honest model; it is also the only model that does not put one
//! device-rank lock inside another. The RCC samples the level from an atomic
//! its `dbp` sink stores into, so the `BDCR` write path never calls into this
//! device at all, and the ranked lock order in `core::sync` is never tested.
//!
//! A board that instantiates no `st.pwr` leaves `rcc.dbp` unwired, and an RCC
//! with an unwired `dbp` treats the backup domain as **unprotected** — nothing
//! is modelling the protection, and a domain that could never be opened would
//! be a board bug wearing a device bug's clothes.
//!
//! # Time
//!
//! A regulator transition takes tens of microseconds. As in the RCC, that is
//! not a host-clock reading: the device is lazily advanced, it holds its own
//! tick in its own clock domain, and the guest access that polls the flag
//! catches it up. `ready-delay` is a count of those ticks.
//!
//! # Variants
//!
//! | `variant` | Part | What it changes |
//! | --- | --- | --- |
//! | `f4` | F405/415, F407/417 | `CR`/`CSR`; `VOS` is one bit at 14, no over-drive |
//! | `f42x` | F427/437, F429/439 | `VOS` is `[15:14]`, and `ODEN`/`ODSWEN` answer with `ODRDY`/`ODSWRDY` |
//! | `l4` | L4x5, L4x6 | `CR1`…`CR4`, `SR1`/`SR2`, `SCR`, the `PUCRx`/`PDCRx` pull registers |
//! | `l4plus` | L4+ | as `l4`, plus `CR5` and its `R1MODE` |
//!
//! # Sources
//!
//! * ST **RM0090** rev 21, §5 "Power control (PWR)" — §5.5 for the register
//!   map, §5.1.4 for the backup domain and `DBP`, §5.1.5 for the voltage
//!   regulator and the over-drive.
//! * ST **RM0351** rev 9, §5 "Power control (PWR)" — §5.4 for the register map
//!   and §5.1.5 for the dynamic voltage scaling `VOSF` reports.
//!
//! No emulator source of any licence was consulted (`ROADMAP.md` §1).
//!
//! # Known deviations
//!
//! * `LPMS`, `SLEEPDEEP` and the standby/stop entry paths are storage only:
//!   the core's `WFI` does not consult this device yet, and the issue that
//!   asked for this one said that half could come later.
//! * The `PUCRx`/`PDCRx` pull-up and pull-down registers read back and change
//!   no pad: a pull resistor is an analogue property of a net this tree does
//!   not model, exactly as in [`gpio`](super::gpio).
//! * `PVD`/`PVM` never trip. There is no analogue supply to compare against,
//!   so `PVDO` and `PVMOx` stay clear rather than reporting an invented level.

use alloc::boxed::Box;
use alloc::format;
use alloc::string::ToString;
use alloc::sync::Arc;
use core::fmt;

use crate::core::device::{Device, DeviceClass, PropertySpec, RealizeCtx, ResetKind};
use crate::core::error::{BusError, Error, Result};
use crate::core::props::{Props, ValueKind};
use crate::core::sched::{AccessKind, LazyHandle};
use crate::core::space::{AccessConstraints, MemAttrs, MemOps, MemResult, Region, RegionRef};
use crate::core::state::{ChunkReader, ChunkWriter, Sink, Source};
use crate::core::sync::{AtomicU64, LockRank, Mutex, Ordering};
use crate::core::value::{Endian, Width};
use crate::core::wire::{Level, WireSource};
use crate::machine::Instance;
use crate::machine::validate::{ClassSchema, PortDir, PropSchema};

/// The class name a machine description writes.
const CLASS_NAME: &str = "st.pwr";

/// The snapshot chunk version. Bump it with the encoding, never on its own.
const STATE_VERSION: u32 = 1;

/// How many 32-bit words the widest layout occupies: the L4+ map ends at `CR5`
/// (`+0x80`), so thirty-three words covers every variant and the snapshot
/// encoding is one shape.
const WORDS: usize = 0x21;

/// "No deadline": nothing is in transition.
const NO_DEADLINE: u64 = u64::MAX;

/// The default regulator transition, in ticks of this device's clock domain.
///
/// Eight ticks is a microsecond of an 8 MHz can. As in the RCC it is
/// deliberately short and a board that wants the datasheet's figure writes
/// `ready-delay`.
const DEFAULT_READY_DELAY: u64 = 8;

/// The name of the backup-domain write-protection output.
pub const DBP_PIN: &str = "dbp";

// ---------------------------------------------------------------------------
// Variants
// ---------------------------------------------------------------------------

/// Which family's register map this instance has.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Variant {
    /// RM0090's `CR`/`CSR` as an F405/407 has them: `VOS` is the single bit 14
    /// and there is no over-drive.
    F4,
    /// The same map on an F427/F429: `VOS` is `[15:14]` and `ODEN`/`ODSWEN`
    /// exist.
    F42x,
    /// RM0351's map: `CR1`…`CR4`, `SR1`/`SR2`, `SCR`, `PUCRx`/`PDCRx`.
    L4,
    /// As [`Variant::L4`], plus `CR5` at `+0x80`.
    L4Plus,
}

impl Variant {
    /// The spelling a machine file writes.
    #[must_use]
    pub fn as_str(self) -> &'static str {
        match self {
            Variant::F4 => "f4",
            Variant::F42x => "f42x",
            Variant::L4 => "l4",
            Variant::L4Plus => "l4plus",
        }
    }

    /// Whether this is one of the two RM0090 layouts.
    fn is_f4(self) -> bool {
        matches!(self, Variant::F4 | Variant::F42x)
    }

    /// How many bytes of the peripheral's kilobyte actually decode.
    #[must_use]
    pub fn register_bytes(self) -> u64 {
        match self {
            // `CR` and `CSR` and nothing else (RM0090 §5.5.3).
            Variant::F4 | Variant::F42x => 0x08,
            // Through `PDCRH` at `+0x5c` (RM0351 §5.4.13).
            Variant::L4 => 0x60,
            // Through `CR5` at `+0x80`.
            Variant::L4Plus => 0x84,
        }
    }

    /// `CR`/`CR1`'s reset value.
    fn cr_reset(self) -> u32 {
        match self {
            // `VOS` = 1, the single bit 14 (RM0090 §5.5.1, "Reset value:
            // 0x0000 4000" on an F405/407).
            Variant::F4 => 0x0000_4000,
            // `VOS` = 11, scale 1 (the same section, "0x0000 C000" on an
            // F427/F429).
            Variant::F42x => 0x0000_c000,
            // `VOS` = 01, range 1 (RM0351 §5.4.1, "Reset value: 0x0000 0200").
            Variant::L4 | Variant::L4Plus => 0x0000_0200,
        }
    }

    /// `DBP`'s bit. Bit 8 in both families (RM0090 §5.5.1, RM0351 §5.4.1).
    fn dbp_bit(self) -> u32 {
        8
    }

    /// The `VOS` field, as `(shift, width)`.
    fn vos(self) -> (u32, u32) {
        match self {
            Variant::F4 => (14, 1),
            Variant::F42x => (14, 2),
            Variant::L4 | Variant::L4Plus => (9, 2),
        }
    }
}

// -- register offsets --------------------------------------------------------

/// F4: `PWR_CR`. L4: `PWR_CR1`. Both at `+0x00`.
const CR: u64 = 0x00;
/// F4: `PWR_CSR` at `+0x04`.
const F4_CSR: u64 = 0x04;
/// L4: `PWR_CR2`.
const L4_CR2: u64 = 0x04;
/// L4: `PWR_CR3`.
const L4_CR3: u64 = 0x08;
/// L4: `PWR_SR1`.
const L4_SR1: u64 = 0x10;
/// L4: `PWR_SR2`.
const L4_SR2: u64 = 0x14;
/// L4: `PWR_SCR`, which is write-1-to-clear and reads as zero.
const L4_SCR: u64 = 0x18;
/// L4+: `PWR_CR5`.
const L4_CR5: u64 = 0x80;

// -- F4 `CR` and `CSR` bits (RM0090 §5.5.1, §5.5.2) --------------------------

/// `CR.ODEN` — switch the over-drive on.
const F4_CR_ODEN: u32 = 1 << 16;
/// `CR.ODSWEN` — switch the over-drive into the core's supply.
const F4_CR_ODSWEN: u32 = 1 << 17;
/// `CSR.VOSRDY` — the regulator has reached the selected scale.
const F4_CSR_VOSRDY: u32 = 1 << 14;
/// `CSR.ODRDY`.
const F4_CSR_ODRDY: u32 = 1 << 16;
/// `CSR.ODSWRDY`.
const F4_CSR_ODSWRDY: u32 = 1 << 17;
/// `CSR.CWUF`/`CSR.CSBF` are cleared through `CR`; the flags themselves are
/// hardware-owned, and so is everything the model does not drive.
const F4_CSR_READ_ONLY: u32 =
    0b111 | F4_CSR_VOSRDY | F4_CSR_ODRDY | F4_CSR_ODSWRDY | (1 << 3) | (1 << 18) | (1 << 19);

// -- L4 `SR2` bits (RM0351 §5.4.6) -------------------------------------------

/// `SR2.VOSF` — **set while the regulator is changing range**, not when it is
/// ready. Firmware waits for it to go clear, which is the opposite polarity to
/// the F4's `VOSRDY` and the single most common way to get this peripheral
/// wrong.
const L4_SR2_VOSF: u32 = 1 << 10;
/// `SR2.REGLPS`/`REGLPF`, and the `PVDO`/`PVMOx` comparator outputs: every bit
/// of `SR2` is hardware-owned.
const L4_SR2_READ_ONLY: u32 = 0xffff_ffff;

// ---------------------------------------------------------------------------
// State
// ---------------------------------------------------------------------------

/// Everything the guest can see or change, plus the device's own position.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct State {
    /// The register file, indexed by `offset / 4`.
    words: [u32; WORDS],
    /// When the regulator finishes its current transition.
    vos_at: u64,
    /// When the over-drive comes up, and when it switches in (F42x only).
    od_at: u64,
    odsw_at: u64,
    /// The tick this device has been advanced to.
    tick: u64,
}

impl State {
    fn reset(variant: Variant) -> State {
        let mut words = [0u32; WORDS];
        words[(CR / 4) as usize] = variant.cr_reset();
        if variant.is_f4() {
            // The regulator is already at the scale `CR` selects out of reset.
            words[(F4_CSR / 4) as usize] = F4_CSR_VOSRDY;
        } else {
            // `CR3.EIWUL` comes up set (RM0351 §5.4.3, "Reset value:
            // 0x0000 8000").
            words[(L4_CR3 / 4) as usize] = 0x0000_8000;
            if variant == Variant::L4Plus {
                // `CR5.R1MODE` comes up set: range 1 boost is off.
                words[(L4_CR5 / 4) as usize] = 0x0000_0100;
            }
        }
        State {
            words,
            vos_at: NO_DEADLINE,
            od_at: NO_DEADLINE,
            odsw_at: NO_DEADLINE,
            tick: 0,
        }
    }

    #[inline]
    fn word(&self, offset: u64) -> u32 {
        self.words[(offset / 4) as usize]
    }

    #[inline]
    fn word_mut(&mut self, offset: u64) -> &mut u32 {
        &mut self.words[(offset / 4) as usize]
    }
}

// ---------------------------------------------------------------------------
// The register block
// ---------------------------------------------------------------------------

/// The register block, as something an address space can dispatch to.
struct Registers {
    state: Mutex<State>,
    variant: Variant,
    ready_delay: u64,
    /// The `DBP` output, connected at realize time.
    dbp: Mutex<Option<WireSource>>,
    /// The lock-free half of the lazy contract.
    tick: AtomicU64,
    next_event: AtomicU64,
    lazy: Mutex<Option<LazyHandle>>,
}

impl fmt::Debug for Registers {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let mut s = f.debug_struct("Registers");
        s.field("variant", &self.variant)
            .field("ready-delay", &self.ready_delay);
        match self.state.try_lock() {
            Some(state) => s.field("tick", &state.tick),
            None => s.field("tick", &"<locked>"),
        };
        s.finish()
    }
}

impl Registers {
    /// Catch the device up before answering an access. No lock held.
    fn sync(&self, attrs: MemAttrs) {
        let handle = self.lazy.lock().clone();
        let Some(handle) = handle else { return };
        let kind = if attrs.debug {
            AccessKind::Debug
        } else {
            AccessKind::Guest
        };
        let _ = handle.sync(kind);
    }

    /// Republish what the lock-free lazy surface reads.
    fn republish(&self, state: &State) {
        self.tick.store(state.tick, Ordering::Relaxed);
        let next = state.vos_at.min(state.od_at).min(state.odsw_at);
        let next = if next <= state.tick { u64::MAX } else { next };
        self.next_event.store(next, Ordering::Relaxed);
    }

    /// Whether `DBP` is set.
    fn dbp_high(&self, state: &State) -> bool {
        state.word(CR) & (1 << self.variant.dbp_bit()) != 0
    }

    /// Drive the `DBP` output to whatever `CR` now says.
    ///
    /// Called with **no lock held**: the sink is the RCC, and the re-entrancy
    /// contract is that an outward call happens after the critical section.
    fn refresh_dbp(&self) {
        let level = {
            let state = self.state.lock();
            Level::from_bool(self.dbp_high(&state))
        };
        let source = self.dbp.lock().clone();
        if let Some(source) = source {
            source.set(level);
        }
    }

    /// Simulate forward: the regulator and the over-drive finish transitioning.
    fn advance_to(&self, tick: u64) {
        let mut state = self.state.lock();
        if tick <= state.tick {
            return;
        }
        state.tick = tick;
        if state.vos_at <= tick {
            state.vos_at = NO_DEADLINE;
            if self.variant.is_f4() {
                *state.word_mut(F4_CSR) |= F4_CSR_VOSRDY;
            } else {
                // "0: the regulator is ready in the selected voltage range."
                *state.word_mut(L4_SR2) &= !L4_SR2_VOSF;
            }
        }
        if state.od_at <= tick {
            state.od_at = NO_DEADLINE;
            *state.word_mut(F4_CSR) |= F4_CSR_ODRDY;
        }
        if state.odsw_at <= tick {
            state.odsw_at = NO_DEADLINE;
            *state.word_mut(F4_CSR) |= F4_CSR_ODSWRDY;
        }
        self.republish(&state);
    }

    fn read_register(&self, offset: u64) -> u32 {
        let state = self.state.lock();
        if !self.variant.is_f4() && offset == L4_SCR {
            // "This register is write-only": the clear register reads as zero.
            return 0;
        }
        state.word(offset)
    }

    /// Write one register, then whatever it implies.
    fn write_register(&self, offset: u64, value: u32) {
        {
            let mut state = self.state.lock();
            let before = state.word(offset);
            if self.variant.is_f4() {
                self.write_f4(&mut state, offset, value, before);
            } else {
                self.write_l4(&mut state, offset, value, before);
            }
            self.republish(&state);
        }
        self.refresh_dbp();
    }

    fn write_f4(&self, state: &mut State, offset: u64, value: u32, before: u32) {
        match offset {
            CR => {
                *state.word_mut(CR) = value;
                let (shift, width) = self.variant.vos();
                let mask = ((1u32 << width) - 1) << shift;
                if value & mask != before & mask {
                    // The regulator has to move, so it stops being ready first
                    // and answers `ready-delay` ticks later (RM0090 §5.1.5).
                    *state.word_mut(F4_CSR) &= !F4_CSR_VOSRDY;
                    state.vos_at = state.tick.saturating_add(self.ready_delay);
                }
                if self.variant == Variant::F42x {
                    for (bit, rdy, at) in [
                        (F4_CR_ODEN, F4_CSR_ODRDY, 0),
                        (F4_CR_ODSWEN, F4_CSR_ODSWRDY, 1),
                    ] {
                        let was = before & bit != 0;
                        let is = value & bit != 0;
                        if is && !was {
                            let deadline = state.tick.saturating_add(self.ready_delay);
                            if at == 0 {
                                state.od_at = deadline;
                            } else {
                                state.odsw_at = deadline;
                            }
                        } else if !is && was {
                            *state.word_mut(F4_CSR) &= !rdy;
                            if at == 0 {
                                state.od_at = NO_DEADLINE;
                            } else {
                                state.odsw_at = NO_DEADLINE;
                            }
                        }
                    }
                }
                // "CWUF: clear wake-up flag … cleared by hardware two system
                // clock cycles later" and "CSBF: clear standby flag". Both
                // clear a `CSR` flag and neither is storage.
                let csr = state.word_mut(F4_CSR);
                *csr &= !(value & 0b11);
                let cr = state.word_mut(CR);
                *cr &= !0b1100;
            }
            F4_CSR => {
                let keep = F4_CSR_READ_ONLY;
                *state.word_mut(F4_CSR) = (before & keep) | (value & !keep);
            }
            _ => {}
        }
    }

    fn write_l4(&self, state: &mut State, offset: u64, value: u32, before: u32) {
        match offset {
            CR => {
                *state.word_mut(CR) = value;
                let (shift, width) = self.variant.vos();
                let mask = ((1u32 << width) - 1) << shift;
                if value & mask != before & mask {
                    // "1: the regulator output voltage is changing"
                    // (RM0351 §5.4.6).
                    *state.word_mut(L4_SR2) |= L4_SR2_VOSF;
                    state.vos_at = state.tick.saturating_add(self.ready_delay);
                }
            }
            L4_SR1 | L4_SR2 => {
                // Both are read-only; `SR1` is cleared through `SCR`.
                let keep = if offset == L4_SR2 {
                    L4_SR2_READ_ONLY
                } else {
                    0xffff_ffff
                };
                *state.word_mut(offset) = (before & keep) | (value & !keep);
            }
            L4_SCR => {
                // "CWUFx: clear wake-up flag x … writing 1 clears it."
                *state.word_mut(L4_SR1) &= !(value & 0x0000_011f);
            }
            L4_CR2 | L4_CR3 => {
                *state.word_mut(offset) = value;
            }
            _ => {
                *state.word_mut(offset) = value;
            }
        }
    }
}

impl MemOps for Registers {
    fn read(&self, offset: u64, dst: &mut [u8], attrs: MemAttrs) -> MemResult {
        let [a, b, c, d] = dst else {
            return Err(BusError::BadAccess);
        };
        self.sync(attrs);
        let value = self.read_register(offset & !3);
        let bytes = value.to_le_bytes();
        (*a, *b, *c, *d) = (bytes[0], bytes[1], bytes[2], bytes[3]);
        Ok(())
    }

    fn write(&self, offset: u64, src: &[u8], attrs: MemAttrs) -> MemResult {
        let [a, b, c, d] = src else {
            return Err(BusError::BadAccess);
        };
        if attrs.debug {
            // A debug write to `CR` would unlock the backup domain and move
            // the `dbp` wire. It is refused rather than guessed at
            // (`ROADMAP.md` §15, invariant 5).
            return Err(BusError::BadAccess);
        }
        self.sync(attrs);
        self.write_register(offset & !3, u32::from_le_bytes([*a, *b, *c, *d]));
        Ok(())
    }

    fn constraints(&self) -> AccessConstraints {
        AccessConstraints::word(Width::U32, Endian::Little)
    }
}

// ---------------------------------------------------------------------------
// The device
// ---------------------------------------------------------------------------

/// An STM32 power controller.
#[derive(Debug)]
pub struct Pwr {
    regs: Arc<Registers>,
    region: RegionRef,
}

impl Pwr {
    /// Validate `props` and build the device.
    ///
    /// # Errors
    ///
    /// [`Error::Property`] if a property is of the wrong kind or value, or if
    /// one this class does not know was given.
    pub fn new(props: &Props) -> Result<Pwr> {
        let mut r = props.reader();
        let variant = match r.or_enum("variant", "f4", &["f4", "f42x", "l4", "l4plus"])? {
            "f42x" => Variant::F42x,
            "l4" => Variant::L4,
            "l4plus" => Variant::L4Plus,
            _ => Variant::F4,
        };
        let ready_delay = r.or("ready-delay", DEFAULT_READY_DELAY)?;
        r.finish()?;
        Ok(Pwr::with_config(variant, ready_delay))
    }

    /// Build one directly — the route a test takes.
    #[must_use]
    pub fn with_config(variant: Variant, ready_delay: u64) -> Pwr {
        let regs = Arc::new(Registers {
            state: Mutex::with_rank(LockRank::DEVICE, State::reset(variant)),
            variant,
            ready_delay,
            dbp: Mutex::with_rank(LockRank::WIRE, None),
            tick: AtomicU64::new(0),
            next_event: AtomicU64::new(u64::MAX),
            lazy: Mutex::with_rank(LockRank::LEAF, None),
        });
        let region = Arc::new(Region::io(
            "pwr",
            variant.register_bytes(),
            Arc::clone(&regs) as Arc<dyn MemOps>,
        ));
        Pwr { regs, region }
    }

    /// Which register layout this instance has.
    #[must_use]
    pub fn variant(&self) -> Variant {
        self.regs.variant
    }

    /// Whether backup-domain write protection is currently disabled.
    #[must_use]
    pub fn dbp(&self) -> bool {
        let state = self.regs.state.lock();
        self.regs.dbp_high(&state)
    }

    /// Set or clear `CR.DBP` as a guest write would, wire and all.
    ///
    /// The route a harness takes when it wants the backup domain open and has
    /// no guest to run: it goes through the same write path, so the `dbp`
    /// output moves exactly as it would for firmware.
    pub fn set_dbp(&self, high: bool) {
        let bit = 1 << self.regs.variant.dbp_bit();
        let cr = self.regs.state.lock().word(CR);
        let next = if high { cr | bit } else { cr & !bit };
        self.regs.write_register(CR, next);
    }

    /// Connect the `DBP` output.
    pub fn connect_dbp(&self, source: WireSource) {
        *self.regs.dbp.lock() = Some(source);
        self.regs.refresh_dbp();
    }
}

impl Device for Pwr {
    fn class(&self) -> &'static DeviceClass {
        &CLASS
    }

    fn realize(&self, _ctx: &mut RealizeCtx<'_>) -> Result<()> {
        Ok(())
    }

    fn reset(&self, _kind: ResetKind) {
        {
            let mut state = self.regs.state.lock();
            let tick = state.tick;
            *state = State::reset(self.regs.variant);
            state.tick = tick;
            self.regs.republish(&state);
        }
        self.regs.refresh_dbp();
    }

    fn save(&self, w: &mut ChunkWriter<'_>) -> Result<()> {
        let state = *self.regs.state.lock();
        for word in state.words {
            w.write_u32(word)?;
        }
        w.write_u64(state.vos_at)?;
        w.write_u64(state.od_at)?;
        w.write_u64(state.odsw_at)?;
        w.write_u64(state.tick)
        // The wire handle is the machine's wiring, not the chip's state.
    }

    fn load(&self, r: &mut ChunkReader<'_>) -> Result<()> {
        let mut state = State::reset(self.regs.variant);
        for word in &mut state.words {
            *word = r.read_u32()?;
        }
        state.vos_at = r.read_u64()?;
        state.od_at = r.read_u64()?;
        state.odsw_at = r.read_u64()?;
        state.tick = r.read_u64()?;
        {
            let mut held = self.regs.state.lock();
            *held = state;
            self.regs.republish(&held);
        }
        self.regs.refresh_dbp();
        Ok(())
    }

    fn region(&self, name: &str) -> Option<RegionRef> {
        matches!(name, "" | "regs").then(|| Arc::clone(&self.region))
    }

    fn connect(&self, port: &str, source: WireSource) -> Result<()> {
        if port != DBP_PIN {
            return Err(Error::Config {
                at: port.to_string(),
                message: format!("a PWR drives one pin, `{DBP_PIN}`"),
            });
        }
        self.connect_dbp(source);
        Ok(())
    }

    fn announce(&self, port: &str) {
        if port == DBP_PIN {
            self.regs.refresh_dbp();
        }
    }

    fn is_lazy(&self) -> bool {
        // The regulator takes time to change range, and the guest's poll of
        // `VOSRDY`/`VOSF` is what has to see it finish.
        true
    }

    fn current_tick(&self) -> u64 {
        self.regs.tick.load(Ordering::Relaxed)
    }

    fn advance_to(&self, tick: u64) {
        self.regs.advance_to(tick);
    }

    fn next_event_tick(&self) -> Option<u64> {
        match self.regs.next_event.load(Ordering::Relaxed) {
            u64::MAX => None,
            at => Some(at),
        }
    }

    fn attach_lazy(&self, handle: LazyHandle) {
        *self.regs.lazy.lock() = Some(handle);
    }
}

impl Instance for Pwr {}

/// The `st.pwr` device class.
pub static CLASS: DeviceClass = DeviceClass {
    name: CLASS_NAME,
    version: STATE_VERSION,
    summary: "STM32 power control: DBP, the voltage-scaling ready flag and the F42x over-drive",
    properties: &[
        PropertySpec {
            name: "variant",
            kind: ValueKind::Str,
            required: false,
            summary: "which register layout: \"f4\", \"f42x\" (over-drive), \"l4\" or \"l4plus\"",
        },
        PropertySpec {
            name: "ready-delay",
            kind: ValueKind::Uint,
            required: false,
            summary: "how many ticks of this device's clock domain a regulator transition \
                      takes (default 8)",
        },
    ],
    construct: |props| Ok(Box::new(Pwr::new(props)?)),
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
    bindings.bind(CLASS_NAME, |props| Ok(Arc::new(Pwr::new(props)?)))
}

/// What the validator should know about `st.pwr`.
#[must_use]
pub fn schema() -> ClassSchema {
    ClassSchema::new(CLASS_NAME)
        .prop(PropSchema::new("variant", ValueKind::Str).values(&["f4", "f42x", "l4", "l4plus"]))
        .prop(PropSchema::new("ready-delay", ValueKind::Uint))
        .region("")
        .region("regs")
        // `wire pwr.dbp -> rcc.dbp`: the backup domain opens when this goes
        // high, and it is a level on a net rather than a handle because that
        // is what it is on the die.
        .port(DBP_PIN, PortDir::Out)
}

#[cfg(test)]
mod tests;
