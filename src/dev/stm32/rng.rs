//! The STM32 true random number generator.
//!
//! Three registers — `CR`, `SR`, `DR` — and one determinism problem. Firmware
//! enables the block, polls `SR.DRDY` and reads `DR`; a mbedTLS or a lwIP port
//! does it on its first TLS handshake, and a part without it is a spin that
//! never ends.
//!
//! # Where the numbers come from, and why that is the interesting part
//!
//! Not from the host. `ROADMAP.md` §0 says a machine run twice produces a
//! bit-identical state hash, and reading a host entropy source would be a
//! wall-clock read by another name. So the stream is
//! [`core::rand::Stream`](crate::core::rand::Stream) — a seeded SplitMix64,
//! shared with `virtio-rng` rather than written out twice — started from
//! [`derive_seed`] of the board's `seed` property and **this instance's path**.
//!
//! That derivation is the whole of the seed design and it is worth stating
//! plainly, because the next two crypto peripherals will copy it:
//!
//! * A seed is not an input *arriving* at an instant, so it is not something
//!   [`core::record`](crate::core::record) can log. That module's own table
//!   files guest-visible randomness under *already deterministic*, not under
//!   an unsealed door. A seed is part of the machine's **initial state**, and
//!   initial state is written in the machine description.
//! * The precedent for "a number that would otherwise come from the host is
//!   written in the board file" is already in this directory: `st.rtc`'s
//!   `epoch`, for exactly this reason, in exactly this board.
//! * A board therefore declares **one** number — `param seed = 1` — and hands
//!   it to every device that needs one. That is the machine-level knob, and
//!   `rsemu run … -p seed=7` overrides it for a run without any new grammar,
//!   `RealizeOptions` field or CLI flag.
//! * What a bare property cannot give is *distinctness*: two `st.rng` objects
//!   handed the same number would deal the guest the same words twice, which
//!   looks like working hardware until firmware compares them. That is fixed
//!   here rather than in the machine, by mixing [`RealizeCtx::path`] — the
//!   snapshot chunk key, so it is stable for the life of the machine and a
//!   restored snapshot derives the same number.
//!
//! None of this is a security primitive. A guest that needs unpredictable
//! bytes must not ask an emulator whose purpose is to do the same thing twice.
//!
//! # The registers
//!
//! | Offset | Register | Bits |
//! | --- | --- | --- |
//! | `0x00` | `CR` | `RNGEN` 2, `IE` 3, `CED` 5 — the last only where the part has it |
//! | `0x04` | `SR` | `DRDY` 0, `CECS` 1, `SECS` 2, `CEIS` 5, `SEIS` 6 |
//! | `0x08` | `DR` | `RNDATA[31:0]`, read-only |
//!
//! `CED` (clock error detection **disable**) arrived with the L4 and is on the
//! L4+/G0/G4/H7/WB parts; an F4 has no such bit and this model does not invent
//! one, which is why it is the `ced` property rather than an assumption. The
//! four `SR` error bits are the same on every part.
//!
//! # Timing
//!
//! `DRDY` sets `latency` ticks of the device's own clock domain after `RNGEN`
//! goes high, and `latency` ticks after each `DR` read. The default is **40**,
//! which is the manual's figure: a new random number is available every forty
//! periods of `RNG_CLK`. A board puts the RNG clock on the device
//! (`clock = hse * 6`, the F4's 48 MHz PLL48CLK) and the tick count is then a
//! real duration rather than a number of scheduler rounds.
//!
//! The block is [lazily advanced](crate::core::sched): it registers no event
//! loop, publishes the tick its next word is due, and the scheduler catches it
//! up before the guest's poll of `SR` is answered. Nothing here sleeps and
//! nothing reads the host clock.
//!
//! # `DR` and the debugger
//!
//! This is the sharpest instance in the tree of `ROADMAP.md` §15's invariant 5:
//! **a debug read must not pop a FIFO, clear a status bit or advance a
//! pointer.** A memory-window refresh in a debugger that consumed a word would
//! change the numbers the guest gets, and the guest cannot tell.
//!
//! So the word is drawn from the stream *when `DRDY` sets*, not when `DR` is
//! read, and it sits in `dr` until somebody takes it. A debug read returns that
//! held word and touches nothing else — not `DRDY`, not the stream position,
//! not the deadline, and not even the lazy catch-up. A guest read returns the
//! same word, clears `DRDY` and arms the next one. Reading `DR` with `DRDY`
//! clear reads zero either way and consumes nothing.
//!
//! # Errors, and why they are a property
//!
//! `SECS`/`SEIS` and `CECS`/`CEIS` are what a driver's error path is written
//! against, and a model in which they can never set is a model in which that
//! path is never executed. They do not happen spontaneously here — a
//! spontaneous one would be non-determinism by the back door — so `fault`
//! injects one, once per reset, on the `RNGEN` edge that starts the block:
//!
//! * `fault = "seed"` sets `SECS` and `SEIS`, and generation **stops**: no
//!   further `DRDY`. Recovery is the manual's: clear `SEIS` by writing zero to
//!   it, then clear and set `RNGEN` to reinitialize and restart the RNG.
//!   Clearing `SEIS` alone does nothing, which is the part of the sequence
//!   drivers get wrong.
//! * `fault = "clock"` sets `CECS` and `CEIS` and generation **continues** —
//!   the RNG clock being too slow does not stop the block, it means the
//!   numbers should not be trusted. `CEIS` is cleared by writing zero to it and
//!   `CECS` by the condition going away, which here is `RNGEN` going low.
//!   With `CR.CED` set, clock error detection is off and neither bit sets.
//!
//! One injection per reset, deliberately: a permanently faulted block is a
//! board that cannot get past its own entropy check, and what a test needs is
//! to see the error path taken and then recovered from.
//!
//! # Sources
//!
//! ST **RM0090** rev 21 §24 "Random number generator (RNG)" for the F4 block —
//! the register map, the forty-period figure, the two error conditions and the
//! seed-error recovery sequence — and ST **RM0351** rev 9 §26 for the L4's
//! `CED` bit. RM0090 **Table 62** gives the interrupt position, which lives in
//! the board file and not here. No emulator source of any licence was
//! consulted (`ROADMAP.md` §1).
//!
//! [`derive_seed`]: crate::core::rand::derive_seed
//! [`RealizeCtx::path`]: crate::core::device::RealizeCtx::path

use alloc::boxed::Box;
use alloc::format;
use alloc::string::String;
use alloc::sync::Arc;
use core::fmt;

use crate::core::device::{Device, DeviceClass, PropertySpec, RealizeCtx, ResetKind};
use crate::core::error::{BusError, Error, Result};
use crate::core::props::{Props, ValueKind};
use crate::core::rand::{Stream, derive_seed};
use crate::core::sched::{AccessKind, LazyHandle};
use crate::core::space::{AccessConstraints, MemAttrs, MemOps, MemResult, Region, RegionRef};
use crate::core::state::{ChunkReader, ChunkWriter, Sink, Source};
use crate::core::sync::{AtomicU64, LockRank, Mutex, Ordering};
use crate::core::value::{Endian, Width};
use crate::core::wire::{Level, WireSource};
use crate::machine::Instance;
use crate::machine::validate::{ClassSchema, PortDir, PropSchema};

/// The class name a machine description writes.
const CLASS_NAME: &str = "st.rng";

/// The snapshot chunk version. Bump it with the encoding, never on its own.
const STATE_VERSION: u32 = 1;

/// The name of the interrupt pin a board wires to its NVIC.
pub const IRQ_PIN: &str = "irq";

/// How many bytes the block decodes: `CR`, `SR`, `DR`. The kilobyte RM0090
/// Table 1 allots it is a hole above that, so a wrong offset faults rather than
/// answering something plausible.
pub const REGISTER_BYTES: u64 = 0x0c;

/// `CR.RNGEN`, bit 2.
const CR_RNGEN: u32 = 1 << 2;
/// `CR.IE`, bit 3.
const CR_IE: u32 = 1 << 3;
/// `CR.CED`, bit 5: clock error detection **disable**, on the parts that have
/// it.
const CR_CED: u32 = 1 << 5;

/// `SR.DRDY`, bit 0.
const SR_DRDY: u32 = 1 << 0;
/// `SR.CECS`, bit 1: the clock error's current status, hardware-driven.
const SR_CECS: u32 = 1 << 1;
/// `SR.SECS`, bit 2: the seed error's current status, hardware-driven.
const SR_SECS: u32 = 1 << 2;
/// `SR.CEIS`, bit 5: the latched clock error, cleared by writing zero.
const SR_CEIS: u32 = 1 << 5;
/// `SR.SEIS`, bit 6: the latched seed error, cleared by writing zero.
const SR_SEIS: u32 = 1 << 6;

/// "It takes 40 periods of the RNG_CLK clock signal between two consecutive
/// random numbers" — RM0090 §24.3.
const DEFAULT_LATENCY: u64 = 40;

/// A deadline that is not set.
const NO_DEADLINE: u64 = u64::MAX;

// ---------------------------------------------------------------------------
// Faults
// ---------------------------------------------------------------------------

/// Which error a `fault` property arms, for a driver's error path.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum Fault {
    /// The block works. The default, and what a real part does.
    #[default]
    None,
    /// One seed error on the first enable: `SECS`, `SEIS`, and no more data
    /// until `RNGEN` is toggled.
    Seed,
    /// One clock error on the first enable: `CECS`, `CEIS`, and data anyway.
    Clock,
}

impl Fault {
    /// The spelling a machine file writes.
    #[must_use]
    pub const fn name(self) -> &'static str {
        match self {
            Fault::None => "none",
            Fault::Seed => "seed",
            Fault::Clock => "clock",
        }
    }

    /// Parse one.
    fn parse(text: &str) -> Option<Fault> {
        match text {
            "none" => Some(Fault::None),
            "seed" => Some(Fault::Seed),
            "clock" => Some(Fault::Clock),
            _ => None,
        }
    }
}

// ---------------------------------------------------------------------------
// State
// ---------------------------------------------------------------------------

/// Everything the guest can see or change, plus the stream's position.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct State {
    /// `CR`, as written.
    cr: u32,
    /// `SR`, as the hardware and the software between them leave it.
    sr: u32,
    /// The word sitting in `DR`. Drawn when `DRDY` set, so a debug read can see
    /// it without moving anything.
    dr: u32,
    /// When the next word lands, in this device's ticks; [`NO_DEADLINE`] for
    /// "nothing is coming".
    ready_at: u64,
    /// Where the device has advanced to, in its own clock domain.
    tick: u64,
    /// The generator, position included.
    stream: Stream,
    /// Whether the configured `fault` is still waiting for an `RNGEN` edge.
    armed: bool,
}

impl State {
    /// The reset state of a block seeded with `stream`.
    fn reset(stream: Stream, fault: Fault) -> State {
        State {
            cr: 0,
            sr: 0,
            dr: 0,
            ready_at: NO_DEADLINE,
            tick: 0,
            stream,
            armed: fault != Fault::None,
        }
    }

    /// Whether the block is switched on.
    const fn enabled(&self) -> bool {
        self.cr & CR_RNGEN != 0
    }

    /// Whether a seed error is standing, which is what stops generation.
    const fn halted(&self) -> bool {
        self.sr & SR_SECS != 0
    }

    /// Whether an enabled interrupt condition is standing.
    ///
    /// "An interrupt is generated when DRDY = 1, SEIS = 1 or CEIS = 1"
    /// (RM0090 §24.3.1, `CR.IE`).
    const fn irq_pending(&self) -> bool {
        self.cr & CR_IE != 0 && self.sr & (SR_DRDY | SR_SEIS | SR_CEIS) != 0
    }
}

// ---------------------------------------------------------------------------
// The register block
// ---------------------------------------------------------------------------

/// The register block, as something an address space can dispatch to.
struct Registers {
    state: Mutex<State>,
    /// How many ticks a word takes.
    latency: u64,
    /// Whether the part has `CR.CED`.
    ced: bool,
    /// Which error the board asked for, if any.
    fault: Fault,
    /// The interrupt output, taken at realize.
    irq: Mutex<Option<WireSource>>,
    /// The catch-up handle the access paths sync through (§4.2).
    lazy: Mutex<Option<LazyHandle>>,
    /// [`State::tick`], republished on every change: the scheduler asks
    /// [`Device::current_tick`] with its slot held at
    /// [`LockRank::LEAF`](crate::core::sync::LockRank::LEAF), so that call may
    /// not take a lock.
    tick: AtomicU64,
    /// The absolute tick of the next word, or [`u64::MAX`] for none. Same
    /// no-lock rule.
    next_event: AtomicU64,
}

impl fmt::Debug for Registers {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let mut s = f.debug_struct("Registers");
        s.field("latency", &self.latency)
            .field("ced", &self.ced)
            .field("fault", &self.fault);
        match self.state.try_lock() {
            Some(state) => s.field("state", &*state),
            None => s.field("state", &"<locked>"),
        };
        s.finish()
    }
}

impl Registers {
    /// Republish what the lock-free lazy surface reads.
    ///
    /// Called from inside every critical section that can move the tick or
    /// change when the next word lands.
    fn publish(&self, state: &State) {
        self.tick.store(state.tick, Ordering::Relaxed);
        // The contract says a reported event is strictly in the future, or
        // catch-up makes no progress and the device stalls where it stands.
        let next = if state.ready_at <= state.tick {
            NO_DEADLINE
        } else {
            state.ready_at
        };
        self.next_event.store(next, Ordering::Relaxed);
    }

    /// Arm the next word, if anything should be coming.
    fn arm(&self, state: &mut State) {
        state.ready_at = if state.enabled() && !state.halted() && state.sr & SR_DRDY == 0 {
            state.tick.saturating_add(self.latency.max(1))
        } else {
            NO_DEADLINE
        };
    }

    /// Drive the interrupt pin to whatever the flags now say.
    ///
    /// Called with **no lock of this device held**: a sink is free to call
    /// straight back in, and the re-entrancy contract is that outward calls
    /// happen after the critical section rather than inside it.
    fn refresh_irq(&self) {
        let level = Level::from_bool(self.state.lock().irq_pending());
        let source = self.irq.lock().clone();
        if let Some(source) = source {
            source.set(level);
        }
    }

    /// Advance to `target` of the block's own clock domain.
    fn advance_to(&self, target: u64) {
        {
            let mut state = self.state.lock();
            if target <= state.tick {
                return;
            }
            state.tick = target;
            if state.ready_at <= target {
                // The word is drawn *here*, when `DRDY` sets, so that a debug
                // read of `DR` can see the same word the guest will get
                // without drawing one of its own.
                state.dr = state.stream.next_u32();
                state.sr |= SR_DRDY;
                state.ready_at = NO_DEADLINE;
            }
            self.publish(&state);
        }
        self.refresh_irq();
    }

    /// Catch the block up before an access is dispatched to it (§4.2).
    ///
    /// A debug access advances nothing (`ROADMAP.md` §15, invariant 5).
    fn sync(&self, attrs: MemAttrs) {
        if attrs.debug {
            return;
        }
        let handle = self.lazy.lock().clone();
        let Some(handle) = handle else {
            return;
        };
        // A refusal means catch-up for this device is already running further
        // up the stack; the access still has to be answered from where the
        // block stands.
        let _ = handle.sync(AccessKind::Guest);
    }

    /// What `CR` reads back: the bits this part implements, and zero elsewhere.
    fn cr_mask(&self) -> u32 {
        CR_RNGEN | CR_IE | if self.ced { CR_CED } else { 0 }
    }

    /// Read one register.
    ///
    /// `debug` is honoured here rather than by the caller because `DR` is the
    /// one register in this block whose read is destructive.
    fn read_register(&self, offset: u64, debug: bool) -> core::result::Result<u32, BusError> {
        let mut state = self.state.lock();
        match offset {
            0x00 => Ok(state.cr & self.cr_mask()),
            0x04 => Ok(state.sr),
            0x08 => {
                // "If DRDY = 0, the RNG_DR register contains zero" — and a
                // read of it in that condition consumes nothing either way.
                if state.sr & SR_DRDY == 0 {
                    return Ok(0);
                }
                let word = state.dr;
                if debug {
                    // The whole point of invariant 5: a memory-window refresh
                    // must not take the guest's word away from it.
                    return Ok(word);
                }
                state.sr &= !SR_DRDY;
                self.arm(&mut state);
                self.publish(&state);
                Ok(word)
            }
            _ => Err(BusError::BadAccess),
        }
    }

    /// Write one register.
    fn write_register(&self, offset: u64, value: u32) -> MemResult {
        let mut state = self.state.lock();
        match offset {
            0x00 => {
                let was = state.enabled();
                state.cr = value & self.cr_mask();
                let now = state.enabled();
                if now && !was {
                    self.start(&mut state);
                } else if !now && was {
                    // Switched off: the pending word is gone and so are both
                    // *current* statuses, which are conditions rather than
                    // latches. `CEIS` and `SEIS` stay, because only software
                    // clears those.
                    state.sr &= !(SR_DRDY | SR_CECS | SR_SECS);
                    state.dr = 0;
                    state.ready_at = NO_DEADLINE;
                }
            }
            0x04 => {
                // `CEIS` and `SEIS` are `rc_w0`: "cleared by writing it to 0".
                // Everything else in `SR` is hardware's.
                for bit in [SR_CEIS, SR_SEIS] {
                    if value & bit == 0 {
                        state.sr &= !bit;
                    }
                }
            }
            // "RNG_DR … Read-only."
            0x08 => {}
            _ => return Err(BusError::BadAccess),
        }
        self.publish(&state);
        Ok(())
    }

    /// `RNGEN` has just gone high: inject a pending fault, or start counting.
    fn start(&self, state: &mut State) {
        if state.armed {
            state.armed = false;
            match self.fault {
                Fault::Seed => {
                    state.sr |= SR_SECS | SR_SEIS;
                }
                // "CED: Clock error detection … 1: Clock error detection is
                // disabled" — with detection off there is nothing to report,
                // and the block goes on producing numbers regardless.
                Fault::Clock if state.cr & CR_CED == 0 => {
                    state.sr |= SR_CECS | SR_CEIS;
                }
                Fault::Clock | Fault::None => {}
            }
        }
        self.arm(state);
    }
}

impl MemOps for Registers {
    fn read(&self, offset: u64, dst: &mut [u8], attrs: MemAttrs) -> MemResult {
        if offset >= REGISTER_BYTES {
            return Err(BusError::BadAccess);
        }
        self.sync(attrs);
        let value = self.read_register(offset, attrs.debug)?;
        let bytes = value.to_le_bytes();
        for (slot, byte) in dst.iter_mut().zip(bytes) {
            *slot = byte;
        }
        if !attrs.debug {
            // Taking the word may have dropped the interrupt.
            self.refresh_irq();
        }
        Ok(())
    }

    fn write(&self, offset: u64, src: &[u8], attrs: MemAttrs) -> MemResult {
        if offset >= REGISTER_BYTES {
            return Err(BusError::BadAccess);
        }
        if attrs.debug {
            // A debug write to `CR` would change when the guest is next
            // interrupted and one to `SR` would drop an error flag the guest
            // has not seen. Neither can be made harmless.
            return Err(BusError::BadAccess);
        }
        let mut word = [0u8; 4];
        for (slot, byte) in word.iter_mut().zip(src) {
            *slot = *byte;
        }
        self.sync(attrs);
        self.write_register(offset, u32::from_le_bytes(word))?;
        // Outside the critical section, as the re-entrancy contract requires.
        self.refresh_irq();
        Ok(())
    }

    fn constraints(&self) -> AccessConstraints {
        // "The peripheral registers can be accessed by words (32-bit) only" —
        // RM0090 §24.4.
        AccessConstraints::word(Width::U32, Endian::Little)
    }
}

// ---------------------------------------------------------------------------
// The device
// ---------------------------------------------------------------------------

/// An STM32 random number generator.
#[derive(Debug)]
pub struct Rng {
    regs: Arc<Registers>,
    region: RegionRef,
    /// The board's seed, before this instance's path is mixed into it.
    board_seed: u64,
}

impl Rng {
    /// Validate `props` and build the block.
    ///
    /// # Errors
    ///
    /// [`Error::Property`] if a property is of the wrong kind or value, or if
    /// one this class does not know was given.
    pub fn new(props: &Props) -> Result<Rng> {
        let mut r = props.reader();
        let seed = r.or("seed", 0u64)?;
        let latency = r.or("latency", DEFAULT_LATENCY)?;
        let ced = r.or("ced", false)?;
        let spelling = r.or_str("fault", "none")?;
        let Some(fault) = Fault::parse(spelling) else {
            return Err(Error::Property(format!(
                "`fault` is `none`, `seed` or `clock`, not `{spelling}`"
            )));
        };
        r.touch("clock");
        r.finish()?;
        Ok(Rng::build(seed, latency, ced, fault))
    }

    /// Build one directly — the route a test takes.
    ///
    /// The stream is seeded from `seed` alone until [`Device::realize`] mixes
    /// the instance path into it, so a test that wants the derivation calls
    /// [`Rng::seed_from_path`].
    #[must_use]
    pub fn build(seed: u64, latency: u64, ced: bool, fault: Fault) -> Rng {
        let regs = Arc::new(Registers {
            state: Mutex::with_rank(LockRank::DEVICE, State::reset(Stream::new(seed), fault)),
            latency,
            ced,
            fault,
            irq: Mutex::with_rank(LockRank::WIRE, None),
            lazy: Mutex::with_rank(LockRank::LEAF, None),
            tick: AtomicU64::new(0),
            next_event: AtomicU64::new(NO_DEADLINE),
        });
        let region = Arc::new(Region::io(
            "rng",
            REGISTER_BYTES,
            Arc::clone(&regs) as Arc<dyn MemOps>,
        ));
        Rng {
            regs,
            region,
            board_seed: seed,
        }
    }

    /// Re-seed the stream from the board seed and this instance's path.
    ///
    /// What [`Device::realize`] does, and the reason two `st.rng` objects on
    /// one board given one `seed` still deal different words. Called before the
    /// machine has executed an instruction, so nothing observable has happened
    /// yet (`CLAUDE.md`, "Devices": two-phase construction).
    pub fn seed_from_path(&self, path: &str) {
        let mut state = self.regs.state.lock();
        state.stream = Stream::new(derive_seed(self.board_seed, path));
    }

    /// The seed the board handed this instance, before the path is mixed in.
    #[must_use]
    pub fn board_seed(&self) -> u64 {
        self.board_seed
    }

    /// The seed this instance's stream actually runs from.
    #[must_use]
    pub fn stream_seed(&self) -> u64 {
        self.regs.state.lock().stream.seed()
    }

    /// How many ticks a word takes.
    #[must_use]
    pub fn latency(&self) -> u64 {
        self.regs.latency
    }

    /// Which error this block is configured to inject.
    #[must_use]
    pub fn fault(&self) -> Fault {
        self.regs.fault
    }

    /// The tick the block has been advanced to, in its own domain.
    #[must_use]
    pub fn tick(&self) -> u64 {
        self.regs.tick.load(Ordering::Relaxed)
    }

    /// Advance to `tick` of the block's own clock domain.
    ///
    /// What [`Device::advance_to`] does; a test that is not running a scheduler
    /// calls it directly.
    pub fn advance_to(&self, tick: u64) {
        self.regs.advance_to(tick);
    }

    /// The level the interrupt pin is being driven to.
    #[must_use]
    pub fn irq_level(&self) -> Level {
        Level::from_bool(self.regs.state.lock().irq_pending())
    }

    /// Take the interrupt output.
    pub fn connect_irq(&self, source: WireSource) {
        *self.regs.irq.lock() = Some(source);
        self.regs.refresh_irq();
    }
}

impl Device for Rng {
    fn class(&self) -> &'static DeviceClass {
        &CLASS
    }

    fn realize(&self, ctx: &mut RealizeCtx<'_>) -> Result<()> {
        // The one outward-facing fact this device needs is its own name, and
        // it is only available here: `new(props)` has no path.
        self.seed_from_path(ctx.path());
        Ok(())
    }

    fn reset(&self, _kind: ResetKind) {
        // Both kinds. Every bit here is APB register state with no battery
        // behind it, and the stream goes back to its seed so that a
        // reset-and-rerun deals the same numbers — which is what makes the
        // reset path reproducible rather than merely deterministic-ish.
        {
            let mut state = self.regs.state.lock();
            // The tick survives, for the reason `st.iwdg`'s reset gives: it is
            // not architectural state but this device's cursor in its own
            // clock domain, and the domain does not rewind because a chip on
            // it was reset.
            let tick = state.tick;
            let mut stream = state.stream;
            stream.rewind();
            *state = State::reset(stream, self.regs.fault);
            state.tick = tick;
            self.regs.publish(&state);
        }
        self.regs.refresh_irq();
    }

    fn save(&self, w: &mut ChunkWriter<'_>) -> Result<()> {
        let state = *self.regs.state.lock();
        w.write_u32(state.cr)?;
        w.write_u32(state.sr)?;
        w.write_u32(state.dr)?;
        w.write_u64(state.ready_at)?;
        // The cursor in the clock domain: the scheduler restores the domain,
        // and without this the two would disagree and a pending word would
        // land at the wrong instant.
        w.write_u64(state.tick)?;
        w.write_bool(state.armed)?;
        // The generator's **position**, without which a restored snapshot
        // deals the guest numbers it has already had.
        state.stream.save(w)
    }

    fn load(&self, r: &mut ChunkReader<'_>) -> Result<()> {
        let cr = r.read_u32()?;
        let sr = r.read_u32()?;
        let dr = r.read_u32()?;
        let ready_at = r.read_u64()?;
        let tick = r.read_u64()?;
        let armed = r.read_bool()?;
        {
            let mut state = self.regs.state.lock();
            state.cr = cr;
            state.sr = sr;
            state.dr = dr;
            state.ready_at = ready_at;
            state.tick = tick;
            state.armed = armed;
            state.stream.load(r)?;
            if state.ready_at != NO_DEADLINE && state.ready_at < state.tick {
                return Err(Error::State(format!(
                    "snapshot has an RNG word due at tick {} on a block already at {}",
                    state.ready_at, state.tick
                )));
            }
            self.regs.publish(&state);
        }
        self.regs.refresh_irq();
        Ok(())
    }

    fn region(&self, name: &str) -> Option<RegionRef> {
        matches!(name, "" | "regs").then(|| Arc::clone(&self.region))
    }

    fn connect(&self, port: &str, source: WireSource) -> Result<()> {
        if port != IRQ_PIN {
            return Err(Error::Config {
                at: String::from(port),
                message: format!("an RNG drives one pin, `{IRQ_PIN}`"),
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

    fn is_lazy(&self) -> bool {
        // A word lands some number of ticks after the guest asked for one, and
        // the guest's poll of `SR` is what has to see it. The scheduler owns
        // that time; this device never sleeps and never reads the host clock.
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
            NO_DEADLINE => None,
            at => Some(at),
        }
    }

    fn attach_lazy(&self, handle: LazyHandle) {
        *self.regs.lazy.lock() = Some(handle);
    }
}

impl Instance for Rng {}

/// The `st.rng` device class.
pub static CLASS: DeviceClass = DeviceClass {
    name: CLASS_NAME,
    version: STATE_VERSION,
    summary: "STM32 random number generator: CR, SR and DR, from a stream the machine seeds",
    properties: &[
        PropertySpec {
            name: "seed",
            kind: ValueKind::Uint,
            required: false,
            summary: "the board's seed; mixed with this instance's path, so two blocks differ",
        },
        PropertySpec {
            name: "latency",
            kind: ValueKind::Uint,
            required: false,
            summary: "ticks of the RNG clock per word (RM0090 §24.3 gives 40, the default)",
        },
        PropertySpec {
            name: "ced",
            kind: ValueKind::Bool,
            required: false,
            summary: "whether the part has CR.CED (an F4 has not; an L4/G0/G4/H7/WB has)",
        },
        PropertySpec {
            name: "fault",
            kind: ValueKind::Str,
            required: false,
            summary: "`none` (default), `seed` or `clock`: one injected error, for a driver's \
                      error path",
        },
    ],
    construct: |props| Ok(Box::new(Rng::new(props)?)),
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
    bindings.bind(CLASS_NAME, |props| Ok(Arc::new(Rng::new(props)?)))
}

/// What the validator should know about `st.rng`.
#[must_use]
pub fn schema() -> ClassSchema {
    ClassSchema::new(CLASS_NAME)
        .prop(PropSchema::new("seed", ValueKind::Uint))
        .prop(PropSchema::new("latency", ValueKind::Uint))
        .prop(PropSchema::new("ced", ValueKind::Bool))
        .prop(PropSchema::new("fault", ValueKind::Str).values(&["none", "seed", "clock"]))
        .region("")
        .region("regs")
        .port(IRQ_PIN, PortDir::Out)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::device::Deferred;
    use crate::core::hosts::HostObjects;
    use crate::core::props::Value;
    use crate::core::space::RequesterId;
    use crate::core::state::{MachineShape, Migrations, StateReader, StateWriter};
    use crate::core::sync::AtomicBool;
    use crate::core::wire::{Wire, WireId, WireIdAllocator, WireSink};
    use alloc::vec::Vec;

    const CR: u64 = 0x00;
    const SR: u64 = 0x04;
    const DR: u64 = 0x08;

    /// A block at `seed`, realized at `path` the way the machine layer does.
    fn rng_at(seed: u64, path: &str) -> Rng {
        let d = Rng::build(seed, 40, false, Fault::None);
        realize(&d, path);
        d
    }

    /// The default test block: one instance called `rng`.
    fn rng_at_seed(seed: u64) -> Rng {
        rng_at(seed, "rng")
    }

    /// Run the realize phase, which is where the path reaches the seed.
    fn realize(d: &Rng, path: &str) {
        let hosts = HostObjects::new();
        let mut deferred = Deferred::new();
        let mut ctx = RealizeCtx::new(path, RequesterId(1), &mut deferred, &hosts);
        Device::realize(d, &mut ctx).expect("an RNG realizes");
    }

    fn read(d: &Rng, offset: u64) -> u32 {
        let mut buf = [0u8; 4];
        d.regs
            .read(offset, &mut buf, MemAttrs::DEFAULT)
            .expect("a word read is legal");
        u32::from_le_bytes(buf)
    }

    fn peek(d: &Rng, offset: u64) -> u32 {
        let mut buf = [0u8; 4];
        d.regs
            .read(offset, &mut buf, MemAttrs::DEBUG)
            .expect("a debug word read is legal");
        u32::from_le_bytes(buf)
    }

    fn write(d: &Rng, offset: u64, value: u32) {
        d.regs
            .write(offset, &value.to_le_bytes(), MemAttrs::DEFAULT)
            .expect("a word write is legal");
    }

    /// Switch the block on and run out the first word's latency.
    fn enable(d: &Rng) {
        write(d, CR, CR_RNGEN);
        d.advance_to(d.tick() + d.latency());
    }

    /// `n` words out of a block, polling the way firmware does.
    fn words(d: &Rng, n: usize) -> Vec<u32> {
        write(d, CR, CR_RNGEN);
        let mut out = Vec::new();
        while out.len() < n {
            if read(d, SR) & SR_DRDY == 0 {
                d.advance_to(d.tick() + 1);
                continue;
            }
            out.push(read(d, DR));
        }
        out
    }

    // -----------------------------------------------------------------------
    // The register face
    // -----------------------------------------------------------------------

    #[test]
    fn drdy_is_clear_until_rngen_and_dr_reads_zero_meanwhile() {
        let d = rng_at_seed(1);
        assert_eq!(read(&d, SR), 0);
        assert_eq!(read(&d, DR), 0);
        // And time passing on a disabled block produces nothing.
        d.advance_to(10_000);
        assert_eq!(read(&d, SR), 0);
        assert_eq!(read(&d, DR), 0);
    }

    #[test]
    fn cr_reads_back_only_the_bits_the_part_implements() {
        let d = rng_at_seed(1);
        write(&d, CR, 0xffff_ffff);
        // An F4 has `RNGEN` and `IE` and nothing else in `CR`.
        assert_eq!(read(&d, CR), CR_RNGEN | CR_IE);

        let l4 = Rng::build(1, 40, true, Fault::None);
        realize(&l4, "rng");
        write(&l4, CR, 0xffff_ffff);
        assert_eq!(read(&l4, CR), CR_RNGEN | CR_IE | CR_CED);
    }

    #[test]
    fn the_aperture_stops_at_dr_and_dr_is_read_only() {
        let d = rng_at_seed(1);
        let mut buf = [0u8; 4];
        assert!(d.regs.read(0x0c, &mut buf, MemAttrs::DEFAULT).is_err());
        // A write to `DR` is dropped rather than faulting: it is a read-only
        // register inside the block, not a hole.
        enable(&d);
        let before = read(&d, SR);
        write(&d, DR, 0xdead_beef);
        assert_eq!(read(&d, SR), before);
    }

    #[test]
    fn drdy_clears_on_read_and_comes_back_after_the_latency() {
        let d = rng_at_seed(3);
        write(&d, CR, CR_RNGEN);
        // Nothing until the forty ticks are up.
        d.advance_to(d.latency() - 1);
        assert_eq!(read(&d, SR) & SR_DRDY, 0);
        d.advance_to(d.latency());
        assert_eq!(read(&d, SR) & SR_DRDY, SR_DRDY);

        let first = read(&d, DR);
        assert_eq!(read(&d, SR) & SR_DRDY, 0, "a read of DR clears DRDY");
        assert_eq!(read(&d, DR), 0, "and DR reads zero while DRDY is clear");

        d.advance_to(d.tick() + d.latency());
        assert_eq!(read(&d, SR) & SR_DRDY, SR_DRDY);
        let second = read(&d, DR);
        assert_ne!(first, second, "the stream moved on");
    }

    // -----------------------------------------------------------------------
    // Determinism — the substance of the device
    // -----------------------------------------------------------------------

    #[test]
    fn the_same_seed_gives_the_same_words_and_a_different_seed_does_not() {
        let a = words(&rng_at_seed(7), 16);
        assert_eq!(a, words(&rng_at_seed(7), 16));
        assert_ne!(a, words(&rng_at_seed(8), 16));
    }

    #[test]
    fn two_instances_in_one_machine_do_not_share_a_stream() {
        // One board seed, two objects: the instance path is what separates
        // them, and without the derivation this assertion is an equality.
        let a = words(&rng_at(1, "rng"), 8);
        let b = words(&rng_at(1, "rng2"), 8);
        assert_ne!(a, b);
        assert_ne!(
            rng_at(1, "rng").stream_seed(),
            rng_at(1, "rng2").stream_seed()
        );
        // And the derivation is a function of the two, so it survives a
        // rebuild of the same board.
        assert_eq!(
            rng_at(1, "rng").stream_seed(),
            rng_at(1, "rng").stream_seed()
        );
    }

    #[test]
    fn a_reset_deals_the_same_numbers_again() {
        let d = rng_at_seed(5);
        let first = words(&d, 4);
        Device::reset(&d, ResetKind::Cold);
        assert_eq!(read(&d, CR), 0, "and the registers are back to reset");
        assert_eq!(words(&d, 4), first);
    }

    // -----------------------------------------------------------------------
    // `MemAttrs::debug`
    // -----------------------------------------------------------------------

    #[test]
    fn a_debug_read_of_dr_neither_consumes_a_word_nor_clears_drdy() {
        let d = rng_at_seed(11);
        enable(&d);
        assert_eq!(read(&d, SR) & SR_DRDY, SR_DRDY);

        // Ten refreshes of a debugger's memory window.
        let seen = peek(&d, DR);
        for _ in 0..10 {
            assert_eq!(peek(&d, DR), seen, "a debug read is repeatable");
            assert_eq!(peek(&d, SR) & SR_DRDY, SR_DRDY, "and does not clear DRDY");
        }
        // The guest then gets exactly the word the debugger was looking at,
        // and the stream is where it would have been with no debugger at all.
        assert_eq!(read(&d, DR), seen);

        let undisturbed = rng_at_seed(11);
        enable(&undisturbed);
        assert_eq!(read(&undisturbed, DR), seen);
    }

    #[test]
    fn a_debug_read_advances_no_time_and_a_debug_write_is_refused() {
        let d = rng_at_seed(2);
        enable(&d);
        let before = d.tick();
        assert_eq!(peek(&d, SR) & SR_DRDY, SR_DRDY);
        assert_eq!(d.tick(), before);

        let buf = [0u8; 4];
        assert!(d.regs.write(CR, &buf, MemAttrs::DEBUG).is_err());
        assert!(d.regs.write(SR, &buf, MemAttrs::DEBUG).is_err());
        assert_eq!(read(&d, CR), CR_RNGEN, "the refused writes changed nothing");
    }

    // -----------------------------------------------------------------------
    // The interrupt
    // -----------------------------------------------------------------------

    /// Counts the edges on a pin.
    #[derive(Debug, Default)]
    struct Probe {
        high: AtomicBool,
    }

    impl WireSink for Probe {
        fn set_level(&self, _src: WireId, _line: u32, level: Level) {
            self.high.store(level.is_high(), Ordering::Relaxed);
        }
    }

    fn watch(d: &Rng) -> Arc<Probe> {
        let ids = WireIdAllocator::new();
        let id = ids.alloc();
        let probe = Arc::new(Probe::default());
        let wire = Wire::builder()
            .source(id)
            .sink(Arc::clone(&probe) as Arc<dyn WireSink>, 0)
            .build_shared();
        Device::connect(d, IRQ_PIN, WireSource::new(wire, id)).expect("the irq pin");
        probe
    }

    #[test]
    fn ie_raises_the_interrupt_wire_when_a_word_is_ready() {
        let d = rng_at_seed(4);
        let probe = watch(&d);
        assert!(!probe.high.load(Ordering::Relaxed));

        // `RNGEN` alone does not interrupt, and neither does a ready word
        // while `IE` is clear.
        write(&d, CR, CR_RNGEN);
        d.advance_to(d.latency());
        assert_eq!(read(&d, SR) & SR_DRDY, SR_DRDY);
        assert!(!probe.high.load(Ordering::Relaxed));

        write(&d, CR, CR_RNGEN | CR_IE);
        assert!(probe.high.load(Ordering::Relaxed), "DRDY and IE");

        // Taking the word drops it, and the next one raises it again.
        read(&d, DR);
        assert!(!probe.high.load(Ordering::Relaxed));
        d.advance_to(d.tick() + d.latency());
        assert!(probe.high.load(Ordering::Relaxed));
    }

    #[test]
    fn only_this_ones_pin_exists() {
        let d = rng_at_seed(1);
        assert!(Device::connect(&d, "nreset", dummy_source()).is_err());
        assert_eq!(
            schema().port_named(IRQ_PIN).map(|p| p.dir),
            Some(PortDir::Out)
        );
    }

    fn dummy_source() -> WireSource {
        let id = WireId::new(9);
        WireSource::new(Wire::builder().source(id).build_shared(), id)
    }

    // -----------------------------------------------------------------------
    // Errors
    // -----------------------------------------------------------------------

    #[test]
    fn a_seeded_fault_sets_seis_and_recovers_only_after_rngen_is_toggled() {
        let d = Rng::build(1, 40, false, Fault::Seed);
        realize(&d, "rng");
        let probe = watch(&d);

        write(&d, CR, CR_RNGEN | CR_IE);
        let sr = read(&d, SR);
        assert_eq!(sr & SR_SECS, SR_SECS);
        assert_eq!(sr & SR_SEIS, SR_SEIS);
        assert!(
            probe.high.load(Ordering::Relaxed),
            "SEIS interrupts under IE"
        );

        // No data comes while the seed error stands, however long is waited.
        d.advance_to(10_000);
        assert_eq!(read(&d, SR) & SR_DRDY, 0);

        // Clearing `SEIS` alone is the half of the sequence that does not work.
        write(&d, SR, !SR_SEIS);
        assert_eq!(read(&d, SR) & SR_SEIS, 0);
        assert_eq!(read(&d, SR) & SR_SECS, SR_SECS, "SECS is hardware's");
        d.advance_to(d.tick() + 10_000);
        assert_eq!(read(&d, SR) & SR_DRDY, 0, "still halted");

        // "clear and set the RNGEN bit to reinitialize and restart the RNG."
        write(&d, CR, CR_IE);
        assert_eq!(read(&d, SR) & SR_SECS, 0);
        write(&d, CR, CR_RNGEN | CR_IE);
        d.advance_to(d.tick() + d.latency());
        assert_eq!(read(&d, SR) & SR_DRDY, SR_DRDY, "and it restarts");
        assert!(probe.high.load(Ordering::Relaxed));
    }

    #[test]
    fn a_clock_fault_latches_ceis_and_still_produces_data() {
        let d = Rng::build(1, 40, false, Fault::Clock);
        realize(&d, "rng");
        write(&d, CR, CR_RNGEN);
        let sr = read(&d, SR);
        assert_eq!(sr & SR_CECS, SR_CECS);
        assert_eq!(sr & SR_CEIS, SR_CEIS);

        // A slow clock does not stop the block; it means the numbers are not
        // to be trusted.
        d.advance_to(d.latency());
        assert_eq!(read(&d, SR) & SR_DRDY, SR_DRDY);
        assert_ne!(read(&d, DR), 0);

        write(&d, SR, !SR_CEIS);
        assert_eq!(read(&d, SR) & SR_CEIS, 0);
    }

    #[test]
    fn ced_switches_the_clock_error_off_entirely() {
        let d = Rng::build(1, 40, true, Fault::Clock);
        realize(&d, "rng");
        // "1: Clock error detection is disabled" — set before the block is
        // enabled, which is what the manual requires.
        write(&d, CR, CR_CED);
        write(&d, CR, CR_CED | CR_RNGEN);
        assert_eq!(read(&d, SR) & (SR_CECS | SR_CEIS), 0);
        d.advance_to(d.latency());
        assert_eq!(read(&d, SR) & SR_DRDY, SR_DRDY);
    }

    // -----------------------------------------------------------------------
    // Snapshots
    // -----------------------------------------------------------------------

    #[test]
    fn the_stream_position_is_part_of_the_state_hash() {
        let saved = rng_at_seed(13);
        let first = words(&saved, 3);
        let at = saved.tick();

        let mut shape = MachineShape::new();
        shape.add_device("rng", CLASS_NAME).unwrap();
        let mut w = StateWriter::new(shape);
        {
            let mut chunk = w.chunk("rng", CLASS_NAME, STATE_VERSION).unwrap();
            Device::save(&saved, &mut chunk).unwrap();
        }
        let bytes = w.to_vec().unwrap();

        // The saved block runs on; what it produces next is what a restore has
        // to reproduce.
        let next = words(&saved, 3);
        assert_ne!(first, next);

        let restored = rng_at_seed(13);
        let reader = StateReader::new(&bytes).unwrap();
        let chunk = reader
            .load("rng", CLASS_NAME, STATE_VERSION, &Migrations::new())
            .unwrap();
        Device::load(&restored, &mut chunk.reader()).unwrap();

        assert_eq!(
            restored.tick(),
            at,
            "the cursor in the clock domain comes back"
        );
        assert_eq!(words(&restored, 3), next, "and it deals the same three");
    }

    #[test]
    fn a_snapshot_round_trips_to_an_identical_state() {
        let saved = rng_at_seed(17);
        write(&saved, CR, CR_RNGEN | CR_IE);
        saved.advance_to(saved.latency());
        read(&saved, SR);

        let mut shape = MachineShape::new();
        shape.add_device("rng", CLASS_NAME).unwrap();
        let mut w = StateWriter::new(shape);
        {
            let mut chunk = w.chunk("rng", CLASS_NAME, STATE_VERSION).unwrap();
            Device::save(&saved, &mut chunk).unwrap();
        }
        let bytes = w.to_vec().unwrap();

        let restored = rng_at_seed(17);
        let reader = StateReader::new(&bytes).unwrap();
        let chunk = reader
            .load("rng", CLASS_NAME, STATE_VERSION, &Migrations::new())
            .unwrap();
        Device::load(&restored, &mut chunk.reader()).unwrap();

        // Copied out one at a time: two `LockRank::DEVICE` locks at once is a
        // lock-order violation, and `core::sync` says so rather than hanging.
        let before = *saved.regs.state.lock();
        let after = *restored.regs.state.lock();
        assert_eq!(after, before);
        assert_eq!(restored.tick(), saved.tick());
        assert_eq!(
            Device::next_event_tick(&restored),
            Device::next_event_tick(&saved)
        );
        assert_eq!(restored.irq_level(), saved.irq_level());
        // And re-saving gives the same bytes, which is the state hash.
        let mut shape = MachineShape::new();
        shape.add_device("rng", CLASS_NAME).unwrap();
        let mut again = StateWriter::new(shape);
        {
            let mut chunk = again.chunk("rng", CLASS_NAME, STATE_VERSION).unwrap();
            Device::save(&restored, &mut chunk).unwrap();
        }
        assert_eq!(again.to_vec().unwrap(), bytes);
    }

    #[test]
    fn a_corrupt_snapshot_is_refused() {
        let d = rng_at_seed(1);
        let mut shape = MachineShape::new();
        shape.add_device("rng", CLASS_NAME).unwrap();
        let mut w = StateWriter::new(shape);
        {
            let mut chunk = w.chunk("rng", CLASS_NAME, STATE_VERSION).unwrap();
            // cr, sr, dr, then a deadline in the past of the tick after it.
            chunk.write_u32(CR_RNGEN).unwrap();
            chunk.write_u32(0).unwrap();
            chunk.write_u32(0).unwrap();
            chunk.write_u64(10).unwrap();
            chunk.write_u64(100).unwrap();
            chunk.write_bool(false).unwrap();
            chunk.write_u64(0).unwrap();
        }
        let bytes = w.to_vec().unwrap();
        let reader = StateReader::new(&bytes).unwrap();
        let chunk = reader
            .load("rng", CLASS_NAME, STATE_VERSION, &Migrations::new())
            .unwrap();
        assert!(Device::load(&d, &mut chunk.reader()).is_err());
    }

    // -----------------------------------------------------------------------
    // Properties
    // -----------------------------------------------------------------------

    #[test]
    fn the_class_takes_its_four_properties_and_nothing_else() {
        let props = Props::new()
            .with("seed", 3u64)
            .with("latency", 8u64)
            .with("ced", true)
            .with("fault", "clock");
        let d = Rng::new(&props).expect("every property is legal");
        assert_eq!(d.board_seed(), 3);
        assert_eq!(d.latency(), 8);
        assert_eq!(d.fault(), Fault::Clock);

        assert!(Rng::new(&Props::new().with("fault", "sometimes")).is_err());
        assert!(Rng::new(&Props::new().with("bits", 32u64)).is_err());
        // `clock` belongs to the machine and is accepted without complaint.
        assert!(Rng::new(&Props::new().with("clock", Value::Uint(48_000_000))).is_ok());
    }
}
