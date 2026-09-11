//! The STM32L0/L4/L4+ Firewall.
//!
//! One class, `st.firewall`. It is a small peripheral and an unusual one: it
//! fences three regions — a **code segment** in flash, a **non-volatile data
//! segment** in flash, and a **volatile data segment** in SRAM — and an access
//! that breaks the rules is not a fault. It is a **system reset**. There is no
//! status bit to poll and no handler to write; the part reboots and
//! `RCC_CSR.FWRSTF` is the only evidence left.
//!
//! That makes it the one peripheral in this tree whose model has to sit *in*
//! the bus path rather than beside it, and the shape below follows from that.
//!
//! # The registers (RM0351 §4.4, base `0x4001_1c00`)
//!
//! | Offset | Register | Field |
//! | --- | --- | --- |
//! | `0x00` | `FW_CSSA` | `ADD[23:8]`, the code segment's start, 256-byte granular |
//! | `0x04` | `FW_CSL` | `LENG[21:8]`, its length |
//! | `0x08` | `FW_NVDSSA` | `ADD[23:8]`, the non-volatile data segment's start |
//! | `0x0c` | `FW_NVDSL` | `LENG[21:8]`, its length |
//! | `0x10` | `FW_VDSSA` | `ADD[15:6]`, the volatile data segment's start, 64-byte granular |
//! | `0x14` | `FW_VDSL` | `LENG[15:6]`, its length |
//! | `0x20` | `FW_CR` | `FPA` 0, `VDS` 1, `VDE` 2 |
//!
//! The two flash segments' start addresses are **offsets into the flash**, and
//! the volatile one's is an offset into SRAM1: the fields are the low bits of
//! an address whose top bits the silicon already knows. `flash-base` and
//! `sram-base` are properties rather than constants so an L0 — which has the
//! same block at different addresses — is a machine file and not a second
//! class.
//!
//! A segment whose length is zero is disabled. That is why the reset state,
//! all zeroes, protects nothing even once the firewall is on.
//!
//! # Enabling it: `SYSCFG_CFGR1.FWDIS`
//!
//! There is no enable bit here. The firewall is switched on by clearing
//! `FWDIS` in [`SYSCFG_CFGR1`](super::syscfg) — bit 0 of the register at
//! `SYSCFG + 0x04`, which resets high and which software can only ever clear.
//! So the machine file draws a wire:
//!
//! ```text
//! wire syscfg.fwdis -> firewall.fwdis
//! ```
//!
//! and the level on it is `FWDIS` itself: high is *disabled*. An unwired
//! firewall is therefore permanently disabled, which is the right default —
//! a board that did not say it has one does not get one.
//!
//! Once enabled the segment registers are read-only (RM0351 §4.4). `FW_CR` is
//! not: `FPA` has to be settable from inside the protected code, which is the
//! whole exit protocol.
//!
//! # The state machine
//!
//! Three states, and the middle one is what the peripheral is for.
//!
//! * **Idle** — `FWDIS` still set. Nothing is checked.
//! * **Closed** — the state the firewall enters when it is enabled. Any access
//!   to the code segment or to the non-volatile data segment, and any access to
//!   the volatile data segment unless `FW_CR.VDS` says it is shared, is a
//!   system reset. The one exception is a **fetch of the first word of the code
//!   segment**: that is the call gate, the single entry point, and it opens the
//!   firewall.
//! * **Opened** — the protected segments are reachable. It stays open until the
//!   processor fetches an instruction outside the code segment (and outside the
//!   volatile data segment, if `VDE` allows execution there).
//!
//! Leaving the code segment is where `FPA` earns its name. Quoting ST's own
//! HAL: "when FPA bit is set, any code executed outside the protected segment
//! will close the Firewall", and "when FPA bit is reset, any code executed
//! outside the protected segment when the Firewall is opened will generate a
//! system reset". So the exit protocol is: set `FPA`, then branch out. The
//! hardware clears `FPA` as it closes, so the next exit needs its own.
//!
//! **An interrupt taken while the firewall is open is an exit like any other.**
//! The exception's vector fetch lands outside the code segment and the firewall
//! judges it by `FPA`, which is why ST's guidance is to keep interrupts masked
//! inside the protected code. This model does not give an exception entry a
//! special case; see "What is not modelled".
//!
//! # Why this device sits in the bus path
//!
//! The firewall watches the *instruction address bus*: it has to see every
//! fetch, not only the ones that land in a protected segment, because "the
//! processor left the code segment" is a fetch **outside** it. Nothing in
//! `core::space` reports an access to a device that is not the target of it,
//! so the model is a filter: the object declares the real memory map with
//! `space = mem`, publishes a `bus` region covering the processor's whole
//! address range, and the board gives the processor a space containing only
//! that region.
//!
//! ```text
//! space cpubus 32
//! object firewall "st.firewall" { space = mem }
//! map cpubus 0 size 4G = firewall.bus
//! ```
//!
//! A DMA master reaches the segments the same way — map `firewall.bus` into its
//! space too — and needs no special case here, because a controller never
//! fetches and every rule that is not about fetches is about addresses alone.
//!
//! An access the firewall refuses returns [`BusError::Protected`] *and* pulses
//! the reset line. The error is not the interesting half: on the part the reset
//! is asserted and the access never completes, and returning the bytes anyway
//! would hand out exactly what the peripheral exists to withhold.
//!
//! `MemAttrs::debug` accesses are passed straight through, unjudged and with no
//! effect on the open/closed state. A debugger that rebooted the machine by
//! looking at it would be useless, and `ROADMAP.md` §15's invariant 5 forbids
//! the state change outright.
//!
//! # What is not modelled
//!
//! * **The interrupt special case.** RM0351 §4.3.5 discusses what happens when
//!   an exception is taken with the firewall open; this model treats the
//!   handler's fetch as an ordinary exit, so with `FPA` clear it resets. That
//!   is the conservative reading and it matches ST's own advice to mask
//!   interrupts inside the protected code, but a firmware that relies on the
//!   documented interrupt behaviour would see a reset this part does not give.
//! * **The second word of the call gate.** RM0351 describes the gate as two
//!   words of `NOP`; this model requires only that entry be at the first one
//!   and does not check what is written there.
//! * **The firewall's own clock gate.** `RCC_AHB2ENR`/`APB2ENR`'s bit is not
//!   consulted, as everywhere else in this tree.
//!
//! # Sources
//!
//! ST **RM0351** rev 9 §4 "Firewall (FW)" and §9.2.2 for `SYSCFG_CFGR1`, ST
//! **RM0432** §4, and ST's `stm32l4xx_hal_firewall` documentation for the `FPA`
//! wording quoted above. Register offsets cross-checked against ST's own CMSIS
//! `FIREWALL_TypeDef`. No emulator source of any licence was consulted
//! (`ROADMAP.md` §1).

use alloc::boxed::Box;
use alloc::format;
use alloc::string::{String, ToString};
use alloc::sync::Arc;
use alloc::vec::Vec;
use core::fmt;

use crate::core::device::{Device, DeviceClass, PropertySpec, RealizeCtx, ResetKind, SinkPin};
use crate::core::error::BusError;
use crate::core::error::{Error, Result};
use crate::core::props::{Props, ValueKind};
use crate::core::space::{
    AccessConstraints, AddressSpace, MemAttrs, MemOps, MemResult, Region, RegionRef,
};
use crate::core::state::{ChunkReader, ChunkWriter, Sink, Source};
use crate::core::sync::{LockRank, Mutex};
use crate::core::value::{Endian, Width};
use crate::core::wire::{FanIn, Level, Resolve, WireId, WireSink, WireSource};
use crate::machine::Instance;
use crate::machine::realize::BindCtx;
use crate::machine::validate::{ClassSchema, PortDir, PropSchema};

/// The class name a machine description writes.
const CLASS_NAME: &str = "st.firewall";

/// The snapshot chunk version. Bump it with the encoding, never on its own.
const STATE_VERSION: u32 = 1;

/// How many bytes of registers the block decodes: up to and including `FW_CR`.
const REGISTER_BYTES: u64 = 0x24;

/// `FW_CSSA`/`FW_NVDSSA`'s `ADD[23:8]` — a 256-byte-granular offset into flash.
const SSA_MASK: u32 = 0x00ff_ff00;

/// `FW_CSL`/`FW_NVDSL`'s `LENG[21:8]` — a 256-byte-granular length.
const SL_MASK: u32 = 0x003f_ff00;

/// `FW_VDSSA`/`FW_VDSL`'s `[15:6]` — 64-byte-granular, and in SRAM1.
const VDS_MASK: u32 = 0x0000_ffc0;

/// `FW_CR.FPA`: firewall pre-arm. Set it before leaving the code segment and
/// the exit closes the firewall; leave it clear and the exit is a reset.
const CR_FPA: u32 = 1 << 0;

/// `FW_CR.VDS`: the volatile data segment is shared with non-protected code.
const CR_VDS: u32 = 1 << 1;

/// `FW_CR.VDE`: the volatile data segment is executable.
const CR_VDE: u32 = 1 << 2;

/// `FW_CR`'s writable bits.
const CR_MASK: u32 = CR_FPA | CR_VDS | CR_VDE;

/// The default `flash-base`: where an L4's main flash array lives.
const DEFAULT_FLASH_BASE: u64 = 0x0800_0000;

/// The default `sram-base`: where an L4's SRAM1 lives.
const DEFAULT_SRAM_BASE: u64 = 0x2000_0000;

/// The default `bus-size`: the whole of a 32-bit processor's address range.
const DEFAULT_BUS_SIZE: u64 = 1 << 32;

// ---------------------------------------------------------------------------
// Segments and verdicts
// ---------------------------------------------------------------------------

/// Which of the three protected regions an access falls in.
///
/// Ordered by how restrictive the answer is, because an access that straddles
/// two is judged by the stricter (`Code` before `Nvds` before `Vds`); nothing
/// legitimate straddles, and a model that picked the looser one would be a way
/// through.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Segment {
    /// None of them.
    Unprotected,
    /// The code segment.
    Code,
    /// The non-volatile data segment.
    Nvds,
    /// The volatile data segment.
    Vds,
}

/// What the firewall decided about an access.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Verdict {
    /// Let it through.
    Allow,
    /// Reset the machine. The access does not complete.
    Reset,
}

// ---------------------------------------------------------------------------
// State
// ---------------------------------------------------------------------------

/// Everything the guest can see or change, plus the two bits of the state
/// machine that have no register.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct State {
    cssa: u32,
    csl: u32,
    nvdssa: u32,
    nvdsl: u32,
    vdssa: u32,
    vdsl: u32,
    cr: u32,
    /// Whether the call gate has been entered and not yet left.
    open: bool,
    /// `SYSCFG_CFGR1.FWDIS`, as the wire last delivered it.
    ///
    /// **Saved**, for the reason `st.syscfg` saves its input levels: it is a
    /// level a sibling is driving, and the order two devices' chunks load in is
    /// not something either of them may depend on. A restore that came back
    /// with this false would protect nothing until SYSCFG happened to
    /// republish.
    disabled: bool,
}

impl Default for State {
    /// Every register zero, closed, and disabled — `FWDIS` resets high.
    fn default() -> State {
        State {
            cssa: 0,
            csl: 0,
            nvdssa: 0,
            nvdsl: 0,
            vdssa: 0,
            vdsl: 0,
            cr: 0,
            open: false,
            disabled: true,
        }
    }
}

// ---------------------------------------------------------------------------
// The shared core
// ---------------------------------------------------------------------------

/// What the register block and the bus filter both reach.
struct Shared {
    state: Mutex<State>,
    /// Where the two flash segments are measured from.
    flash_base: u64,
    /// Where the volatile data segment is measured from.
    sram_base: u64,
    /// The memory map the filter forwards to, once bound.
    ///
    /// Cloned out and the lock released before the forwarded access: that
    /// access takes a [`LockRank::TOPOLOGY`] guard and this is a leaf.
    downstream: Mutex<Option<Arc<AddressSpace>>>,
    /// The reset output, pulsed on an illegal access.
    reset_out: Mutex<Option<WireSource>>,
}

impl fmt::Debug for Shared {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let mut s = f.debug_struct("Shared");
        s.field("flash_base", &self.flash_base)
            .field("sram_base", &self.sram_base);
        match self.state.try_lock() {
            Some(state) => s.field("state", &*state),
            None => s.field("state", &"<locked>"),
        };
        s.finish()
    }
}

impl Shared {
    /// The code segment, as `(start, len)`; `len` is zero when it is disabled.
    fn code(&self, state: &State) -> (u64, u64) {
        (
            self.flash_base.wrapping_add(u64::from(state.cssa)),
            u64::from(state.csl),
        )
    }

    /// The non-volatile data segment.
    fn nvds(&self, state: &State) -> (u64, u64) {
        (
            self.flash_base.wrapping_add(u64::from(state.nvdssa)),
            u64::from(state.nvdsl),
        )
    }

    /// The volatile data segment.
    fn vds(&self, state: &State) -> (u64, u64) {
        (
            self.sram_base.wrapping_add(u64::from(state.vdssa)),
            u64::from(state.vdsl),
        )
    }

    /// Which segment `[addr, addr + len)` touches, strictest first.
    fn segment_of(&self, state: &State, addr: u64, len: u64) -> Segment {
        for (seg, (start, size)) in [
            (Segment::Code, self.code(state)),
            (Segment::Nvds, self.nvds(state)),
            (Segment::Vds, self.vds(state)),
        ] {
            if size != 0 && overlaps(addr, len, start, size) {
                return seg;
            }
        }
        Segment::Unprotected
    }

    /// Whether a fetch at `addr` counts as still being inside the protected
    /// code — the code segment, or the volatile data segment when `VDE` says
    /// that is executable.
    fn executing_inside(&self, state: &State, addr: u64, len: u64) -> bool {
        match self.segment_of(state, addr, len) {
            Segment::Code => true,
            Segment::Vds => state.cr & CR_VDE != 0,
            _ => false,
        }
    }

    /// Judge one access, advancing the state machine.
    ///
    /// Takes the state lock and releases it; the caller pulses the reset line
    /// afterwards, outside the critical section.
    fn judge(&self, addr: u64, len: u64, fetch: bool) -> Verdict {
        let mut state = self.state.lock();
        if state.disabled {
            return Verdict::Allow;
        }

        if state.open {
            if !fetch || self.executing_inside(&state, addr, len) {
                // Open, and either a data access — which the protected code is
                // entitled to make anywhere — or a fetch that has not left.
                return Verdict::Allow;
            }
            // The processor is leaving. `FPA` decides whether that is the
            // documented exit or the failure the peripheral exists to catch.
            if state.cr & CR_FPA == 0 {
                return Verdict::Reset;
            }
            state.cr &= !CR_FPA;
            state.open = false;
            // And the fetch itself is now judged as a closed firewall would —
            // it lands outside the segments, so it falls through below.
        }

        match self.segment_of(&state, addr, len) {
            Segment::Unprotected => Verdict::Allow,
            Segment::Code => {
                // The call gate: the single entry point, and it is the *first
                // word* of the segment. Anything else in the segment — a jump
                // into the middle of it, a read of it by unprotected code — is
                // the illegal entry this peripheral exists for.
                let (start, _) = self.code(&state);
                if fetch && addr == start {
                    state.open = true;
                    Verdict::Allow
                } else {
                    Verdict::Reset
                }
            }
            Segment::Nvds => Verdict::Reset,
            Segment::Vds => {
                if state.cr & CR_VDS == 0 {
                    Verdict::Reset
                } else if fetch && state.cr & CR_VDE == 0 {
                    // Shared, but not executable.
                    Verdict::Reset
                } else {
                    Verdict::Allow
                }
            }
        }
    }

    /// Pulse the reset line.
    ///
    /// **Never called with the state lock held.** A reset reaches every device
    /// on the machine, this one included (`CLAUDE.md`, "Concurrency").
    fn pulse_reset(&self) {
        let source = self.reset_out.lock().clone();
        if let Some(source) = source {
            source.pulse(Level::High);
        }
    }

    /// The space the filter forwards to.
    fn downstream(&self) -> Option<Arc<AddressSpace>> {
        self.downstream.lock().clone()
    }

    /// Judge, and reset if that is the answer. `true` means "let it through".
    fn gate(&self, addr: u64, len: u64, attrs: MemAttrs) -> MemResult {
        if attrs.debug {
            // No judgement and no state change: a debugger that rebooted the
            // machine by looking at it would be useless (`ROADMAP.md` §15).
            return Ok(());
        }
        match self.judge(addr, len, attrs.is_fetch()) {
            Verdict::Allow => Ok(()),
            Verdict::Reset => {
                self.pulse_reset();
                Err(BusError::Protected)
            }
        }
    }
}

/// Whether `[a, a + alen)` and `[b, b + blen)` share a byte.
fn overlaps(a: u64, alen: u64, b: u64, blen: u64) -> bool {
    let aend = a.saturating_add(alen.max(1));
    let bend = b.saturating_add(blen);
    a < bend && b < aend
}

// ---------------------------------------------------------------------------
// The register block
// ---------------------------------------------------------------------------

/// `FW_CSSA`…`FW_CR`, as something an address space can dispatch to.
#[derive(Debug)]
struct Registers {
    shared: Arc<Shared>,
}

impl Registers {
    fn read_register(&self, offset: u64) -> u32 {
        let state = self.shared.state.lock();
        match offset {
            0x00 => state.cssa,
            0x04 => state.csl,
            0x08 => state.nvdssa,
            0x0c => state.nvdsl,
            0x10 => state.vdssa,
            0x14 => state.vdsl,
            0x20 => state.cr,
            _ => 0,
        }
    }

    fn write_register(&self, offset: u64, value: u32) {
        let mut state = self.shared.state.lock();
        // "The Firewall segment registers can be written only when the
        // Firewall is disabled" (RM0351 §4.4). `FW_CR` is not one of them: the
        // protected code has to be able to set `FPA` on its way out.
        if offset != 0x20 && !state.disabled {
            return;
        }
        match offset {
            0x00 => state.cssa = value & SSA_MASK,
            0x04 => state.csl = value & SL_MASK,
            0x08 => state.nvdssa = value & SSA_MASK,
            0x0c => state.nvdsl = value & SL_MASK,
            0x10 => state.vdssa = value & VDS_MASK,
            0x14 => state.vdsl = value & VDS_MASK,
            0x20 => state.cr = value & CR_MASK,
            _ => {}
        }
    }
}

impl MemOps for Registers {
    fn read(&self, offset: u64, dst: &mut [u8], _attrs: MemAttrs) -> MemResult {
        let [a, b, c, d] = dst else {
            return Err(BusError::BadAccess);
        };
        // Nothing here pops or clears on a read, so a debug read is the same
        // read (`ROADMAP.md` §15, invariant 5).
        let bytes = self.read_register(offset & !3).to_le_bytes();
        (*a, *b, *c, *d) = (bytes[0], bytes[1], bytes[2], bytes[3]);
        Ok(())
    }

    fn write(&self, offset: u64, src: &[u8], attrs: MemAttrs) -> MemResult {
        let [a, b, c, d] = src else {
            return Err(BusError::BadAccess);
        };
        if attrs.debug {
            // A debug write to `FW_CR` would clear `FPA` and arm a reset the
            // guest did not ask for, and one to a segment register would move
            // the fence. Neither has a harmless version.
            return Err(BusError::BadAccess);
        }
        self.write_register(offset & !3, u32::from_le_bytes([*a, *b, *c, *d]));
        Ok(())
    }

    fn constraints(&self) -> AccessConstraints {
        AccessConstraints::word(Width::U32, Endian::Little)
    }
}

// ---------------------------------------------------------------------------
// The bus filter
// ---------------------------------------------------------------------------

/// The processor's whole address range, judged and then forwarded.
#[derive(Debug)]
struct Filter {
    shared: Arc<Shared>,
}

impl MemOps for Filter {
    fn read(&self, offset: u64, dst: &mut [u8], attrs: MemAttrs) -> MemResult {
        let Some(down) = self.shared.downstream() else {
            return Err(BusError::Unassigned);
        };
        self.shared.gate(offset, dst.len() as u64, attrs)?;
        down.read_bytes(offset, dst, attrs)
    }

    fn write(&self, offset: u64, src: &[u8], attrs: MemAttrs) -> MemResult {
        let Some(down) = self.shared.downstream() else {
            return Err(BusError::Unassigned);
        };
        self.shared.gate(offset, src.len() as u64, attrs)?;
        down.write_bytes(offset, src, attrs)
    }

    fn constraints(&self) -> AccessConstraints {
        // Whatever is behind the filter decides; a width rule of its own would
        // refuse accesses the memory accepts.
        AccessConstraints::ANY
    }
}

// ---------------------------------------------------------------------------
// The device
// ---------------------------------------------------------------------------

/// An STM32 Firewall.
#[derive(Debug)]
pub struct Firewall {
    shared: Arc<Shared>,
    regs: RegionRef,
    bus: RegionRef,
    /// The `fwdis` input pin; the device keeps the strong reference because a
    /// net holds its sinks weakly.
    pins: Mutex<Vec<Arc<FwdisPin>>>,
}

impl Firewall {
    /// Validate `props` and build the firewall.
    ///
    /// # Errors
    ///
    /// [`Error::Property`] if a property is of the wrong kind or value, or if
    /// one this class does not know was given.
    pub fn new(props: &Props) -> Result<Firewall> {
        let mut r = props.reader();
        let flash_base = r.or_addr("flash-base", DEFAULT_FLASH_BASE)?;
        let sram_base = r.or_addr("sram-base", DEFAULT_SRAM_BASE)?;
        let bus_size = r.or_size("bus-size", DEFAULT_BUS_SIZE)?;
        r.finish()?;
        if bus_size == 0 {
            return Err(Error::Property(String::from(
                "`bus-size` is the address range the firewall filters, and a range of nothing \
                 filters nothing",
            )));
        }
        Ok(Firewall::build(flash_base, sram_base, bus_size))
    }

    /// Build one directly — the route a test takes.
    #[must_use]
    pub fn build(flash_base: u64, sram_base: u64, bus_size: u64) -> Firewall {
        let shared = Arc::new(Shared {
            state: Mutex::with_rank(LockRank::DEVICE, State::default()),
            flash_base,
            sram_base,
            downstream: Mutex::with_rank(LockRank::LEAF, None),
            reset_out: Mutex::with_rank(LockRank::WIRE, None),
        });
        let regs = Arc::new(Region::io(
            "firewall",
            REGISTER_BYTES,
            Arc::new(Registers {
                shared: Arc::clone(&shared),
            }) as Arc<dyn MemOps>,
        )) as RegionRef;
        let bus = Arc::new(Region::io(
            "firewall-bus",
            bus_size,
            Arc::new(Filter {
                shared: Arc::clone(&shared),
            }) as Arc<dyn MemOps>,
        )) as RegionRef;
        Firewall {
            shared,
            regs,
            bus,
            pins: Mutex::with_rank(LockRank::DEVICE, Vec::new()),
        }
    }

    /// Point the filter at the memory map it fences.
    ///
    /// Normally done by [`Instance::bind`] from the object's `space =`
    /// property; a test that builds its own space calls this.
    pub fn attach_bus(&self, space: &Arc<AddressSpace>) {
        *self.shared.downstream.lock() = Some(Arc::clone(space));
    }

    /// Whether `SYSCFG_CFGR1.FWDIS` still has the firewall switched off.
    #[must_use]
    pub fn disabled(&self) -> bool {
        self.shared.state.lock().disabled
    }

    /// Whether the call gate has been entered and not yet left.
    #[must_use]
    pub fn is_open(&self) -> bool {
        self.shared.state.lock().open
    }

    /// Drive `FWDIS` directly — the route a test with no SYSCFG takes.
    ///
    /// High is *disabled*, which is the sense of the bit and of the wire.
    pub fn set_fwdis(&self, level: bool) {
        let mut state = self.shared.state.lock();
        state.disabled = level;
        if level {
            // Back to idle: nothing is checked, so nothing is open either.
            state.open = false;
        }
    }
}

/// One `fwdis` input, as something a wire can drive.
#[derive(Debug)]
pub struct FwdisPin {
    shared: Arc<Shared>,
    inputs: FanIn,
}

impl FwdisPin {
    /// The per-source levels currently seen.
    #[must_use]
    pub fn inputs(&self) -> &FanIn {
        &self.inputs
    }
}

impl WireSink for FwdisPin {
    fn set_level(&self, src: WireId, _line: u32, level: Level) {
        self.inputs.set(src, level);
        let high = self.inputs.resolve(Resolve::Or).is_high();
        let mut state = self.shared.state.lock();
        state.disabled = high;
        if high {
            state.open = false;
        }
    }
}

impl Device for Firewall {
    fn class(&self) -> &'static DeviceClass {
        &CLASS
    }

    fn realize(&self, _ctx: &mut RealizeCtx<'_>) -> Result<()> {
        // Nothing outward: `bind` takes the space and `map` statements place
        // the two regions.
        Ok(())
    }

    fn reset(&self, _kind: ResetKind) {
        // Both kinds. A system reset is the only thing that sets `FWDIS` again,
        // and SYSCFG is what will republish it — but the firewall's own state
        // goes back to idle here rather than waiting for that wire, because the
        // machine is not allowed to run one instruction protected by a
        // half-reset firewall.
        *self.shared.state.lock() = State::default();
    }

    fn save(&self, w: &mut ChunkWriter<'_>) -> Result<()> {
        let state = *self.shared.state.lock();
        w.write_u64(self.shared.flash_base)?;
        w.write_u64(self.shared.sram_base)?;
        w.write_u32(state.cssa)?;
        w.write_u32(state.csl)?;
        w.write_u32(state.nvdssa)?;
        w.write_u32(state.nvdsl)?;
        w.write_u32(state.vdssa)?;
        w.write_u32(state.vdsl)?;
        w.write_u32(state.cr)?;
        w.write_bool(state.open)?;
        w.write_bool(state.disabled)?;
        Ok(())
    }

    fn load(&self, r: &mut ChunkReader<'_>) -> Result<()> {
        let flash_base = r.read_u64()?;
        let sram_base = r.read_u64()?;
        if flash_base != self.shared.flash_base || sram_base != self.shared.sram_base {
            return Err(Error::State(format!(
                "snapshot has a firewall over flash at {flash_base:#x} and SRAM at \
                 {sram_base:#x}, this one is over {:#x} and {:#x}",
                self.shared.flash_base, self.shared.sram_base
            )));
        }
        let state = State {
            cssa: r.read_u32()?,
            csl: r.read_u32()?,
            nvdssa: r.read_u32()?,
            nvdsl: r.read_u32()?,
            vdssa: r.read_u32()?,
            vdsl: r.read_u32()?,
            cr: r.read_u32()?,
            open: r.read_bool()?,
            disabled: r.read_bool()?,
        };
        *self.shared.state.lock() = state;
        Ok(())
    }

    fn region(&self, name: &str) -> Option<RegionRef> {
        match name {
            "" | "regs" => Some(Arc::clone(&self.regs)),
            "bus" => Some(Arc::clone(&self.bus)),
            _ => None,
        }
    }

    fn connect(&self, port: &str, source: WireSource) -> Result<()> {
        if port != "reset" {
            return Err(Error::Config {
                at: port.to_string(),
                message: String::from("a firewall drives one pin, `reset`"),
            });
        }
        *self.shared.reset_out.lock() = Some(source);
        Ok(())
    }

    fn sink(&self, port: &str, sources: &[WireId]) -> Option<SinkPin> {
        if port != "fwdis" {
            return None;
        }
        let pin = Arc::new(FwdisPin {
            shared: Arc::clone(&self.shared),
            inputs: FanIn::new(sources),
        });
        self.pins.lock().push(Arc::clone(&pin));
        Some(SinkPin { sink: pin, line: 0 })
    }
}

/// The machine layer's half: the filter has to know what it is in front of.
impl Instance for Firewall {
    fn bind(&self, ctx: &BindCtx<'_>) -> Result<()> {
        let space = ctx.space().ok_or_else(|| Error::Config {
            at: String::from(ctx.path()),
            message: String::from(
                "a firewall sits in front of a memory map and forwards to it: add `space = mem` \
                 to the object, and give the processor a space whose only region is \
                 `<this>.bus`",
            ),
        })?;
        self.attach_bus(space);
        Ok(())
    }
}

/// The `st.firewall` device class.
pub static CLASS: DeviceClass = DeviceClass {
    name: CLASS_NAME,
    version: STATE_VERSION,
    summary: "STM32L0/L4 Firewall: three protected segments whose call gate is the only way in",
    properties: &[
        PropertySpec {
            name: "flash-base",
            kind: ValueKind::Addr,
            required: false,
            summary: "where FW_CSSA and FW_NVDSSA are measured from (0x08000000)",
        },
        PropertySpec {
            name: "sram-base",
            kind: ValueKind::Addr,
            required: false,
            summary: "where FW_VDSSA is measured from (0x20000000)",
        },
        PropertySpec {
            name: "bus-size",
            kind: ValueKind::Size,
            required: false,
            summary: "how much address space the `bus` filter covers (4 GiB)",
        },
    ],
    construct: |props| Ok(Box::new(Firewall::new(props)?)),
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
    bindings.bind(CLASS_NAME, |props| Ok(Arc::new(Firewall::new(props)?)))
}

/// What the validator should know about `st.firewall`.
#[must_use]
pub fn schema() -> ClassSchema {
    ClassSchema::new(CLASS_NAME)
        .prop(PropSchema::new("flash-base", ValueKind::Addr))
        .prop(PropSchema::new("sram-base", ValueKind::Addr))
        .prop(PropSchema::new("bus-size", ValueKind::Size))
        .region("")
        .region("regs")
        .region("bus")
        .port("reset", PortDir::Out)
        .port("fwdis", PortDir::In)
}

#[cfg(test)]
mod tests;
