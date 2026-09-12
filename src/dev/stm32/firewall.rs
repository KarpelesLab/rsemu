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
//!   system reset. The one way in is the call gate.
//! * **Opened** — the protected segments are reachable. It stays open until the
//!   processor fetches an instruction outside the protected code — the code
//!   segment, and the volatile data segment while `VDE = 1` and `VDS = 0`.
//!
//! # The call gate is three words and the entry is the second
//!
//! RM0351 §4.3.6 "call gate sequence" is precise about this, and it is the part
//! that is easy to get wrong:
//!
//! > The "call gate" is composed of 3 words located on the first three 32-bit
//! > addresses of the base address of the code segment and of the Volatile data
//! > segment if it is declared as not shared (VDS = 0) and executable
//! > (VDE = 1). – 1st word: Dummy 32-bit words always closed in order to
//! > protect the "call gate" opening from an access due to a prefetch buffer.
//! > – 2nd and 3rd words: 2 specific 32-bit words called "call gate" and always
//! > opened.
//!
//! So the entry point is **`CSSA + 4`**, not `CSSA`, and the first word exists
//! precisely to be forbidden: a prefetch buffer running ahead of a branch to
//! `CSSA + 4` would otherwise open the gate by accident. Opening takes **two**
//! fetches, `CSSA + 4` then `CSSA + 8`, with nothing in between:
//!
//! > The 2nd word and 3rd word execution must not be interrupted by any
//! > intermediate instruction fetch; otherwise, the Firewall is not considered
//! > open and comes back to a close state. Then, executing the 3rd word after
//! > receiving the intermediate instruction fetch would generate a system reset
//! > as a consequence.
//!
//! which is modelled literally: a fetch of `+4` arms the sequence, a fetch of
//! `+8` while armed opens the firewall, any other fetch disarms it, and a fetch
//! of `+8` that is not armed is a reset. That is also the honest answer to "the
//! gate's words are not required to be `NOP`s" — the hardware never looks at
//! what is *written* there. What it checks is the order of the fetches, and a
//! model can check exactly that.
//!
//! The volatile data segment carries a gate of its own on the same three words
//! from `VDSSA`, but only while it is executable and not shared (`VDE = 1`,
//! `VDS = 0`); a shared segment needs no gate because nothing fences it.
//!
//! # Closing it, and what an interrupt does
//!
//! Leaving the protected code is where `FPA` earns its name. RM0351 §4.3.6,
//! "Closing the Firewall": the protected code writes the Firewall Pre Arm Flag
//! and then jumps to any executable location outside the segments, and "if the
//! Firewall Pre Arm Flag is not set when the protected code jumps to a non
//! protected segment, a reset is generated". The hardware clears `FPA` as it
//! closes, so the next exit needs its own.
//!
//! **An interrupt is not a special case, and that is the whole of it.** The
//! firewall snoops the AMBA bus (§4.3.1). It sees a fetch outside the code
//! segment and has no way to know whether a branch or an exception entry put
//! the processor there, so it judges that fetch by `FPA` like any other exit.
//! RM0351 §4.3.2, "Interrupts management", states the consequence rather than a
//! rule of its own:
//!
//! > The code protected by the Firewall must not be interruptible. It is up to
//! > the user code to disable any interrupt source before executing the code
//! > protected by the Firewall. If this constraint is not respected, if an
//! > interrupt comes while the protected code is executed (Firewall opened),
//! > the Firewall will be closed as soon as the interrupt subroutine is
//! > executed. When the code returns back to the protected code area, a
//! > Firewall alarm will raise since the "call gate" sequence will not be
//! > applied and a reset will be generated.
//!
//! ST **AN4730** §2.2 splits that into the two `FPA` cases explicitly. With the
//! flag left set, the handler's first fetch **closes** the firewall and the
//! *return* into the middle of the protected routine is the reset, "since the
//! protected code continues from the point where it was interrupted, without
//! re-opening the FIREWALL through a Call gate sequence". With the flag clear —
//! which is what AN4730's own Figure 4 call gate does, clearing `FPA` on entry
//! and setting it again on the way out — "the reset is generated by the
//! FIREWALL as soon as the interrupt is served".
//!
//! Both of those *are* the ordinary-exit rule applied to the handler's fetch.
//! So the core does not have to tell this device that an exception was taken,
//! and no seam was added for one: a model that gave an exception entry a case
//! of its own would be modelling a rule the silicon does not have. §4.3.2
//! settles the other direction too — "There is no interrupt generated by the
//! Firewall" — so the reset line is this device's only output.
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
//! # A bus master gets a different window
//!
//! A DMA controller is *not* judged by the processor's rules, which is why it
//! cannot be handed the processor's window:
//!
//! > All DMA accesses to the protected segments are forbidden, whatever the
//! > Firewall state, and generate a system reset. (§4.3.4, and again in §4.2)
//!
//! "Whatever the Firewall state" is the whole difference. While the firewall is
//! open the code segment is readable by the core and still a reset for a
//! master, so the device publishes a **second** filter region, `dma`, which a
//! board maps into a master's space:
//!
//! ```text
//! map dmabus 0 size 4G = firewall.dma
//! ```
//!
//! It judges by address alone, in every state, and moves no part of the state
//! machine — a controller never fetches, so it can neither open the gate nor
//! close it. Which region an access arrived through is how the firewall knows
//! which master made it; keying on the requester id in `MemAttrs` instead would
//! oblige every board to allocate ids that nothing else on an STM32 uses.
//!
//! `FW_CR.VDS` is deliberately *not* consulted here. Sharing is a rule about
//! the processor (Table 18 is a processor table), and the DMA sentence is
//! unqualified in all three places it appears — §4.2, §4.3.4 and AN4730 §2.1.
//! The literal reading is the conservative one and it is what is implemented;
//! if a real design turns out to put a DMA buffer in a shared volatile segment
//! and run, this is the line to revisit.
//!
//! # The clock gate
//!
//! `RCC_APB2ENR.FWEN` is bit 7 on an L4, and it is not an ordinary enable:
//! "Set by software, reset by hardware. Software can only write 1. A write at 0
//! has no effect" (§6.4.16). It is step 1 of §4.3.5's initialization procedure,
//! so a firmware that forgets it writes the segment registers into a block that
//! is not listening.
//!
//! A board draws `wire rcc.apb2en7 -> firewall.clken` and the register block
//! goes dead while that is low: writes are dropped and reads return zero. The
//! *fence* is not gated, because it cannot need to be — `FWEN` can never be
//! cleared, so a firewall that reached the enabled state has a clock by
//! construction. An unwired `clken` is clocked, which is the right default for
//! a board that did not model its RCC.
//!
//! # What is not modelled
//!
//! * **`FW_CR`'s own access rule.** RM0351 §4.3.5: while `NVDSL` is non-zero
//!   the configuration register may be reached only when the firewall is
//!   opened, and AN4730 §1.6 adds that an access in the closed state generates a
//!   reset. Here `FW_CR` answers in every state.
//! * **AN4730 §2.3's write-buffer hazard.** On the part, `FPA` written across
//!   the AHB/APB bridge may not be visible by the time the next instruction
//!   returns from the call gate, which is why ST's procedure reads it back.
//!   There is no bridge latency in this model, so the hazard cannot arise; the
//!   read-back is harmless either way.
//!
//! # Sources
//!
//! ST **RM0351** rev 11 §4 "Firewall (FW)" — §4.3.2 for interrupts and DMA,
//! §4.3.4 Table 18 for the segment access matrix, §4.3.5 for initialization,
//! §4.3.6 for the call-gate sequence and the closing rule — plus §6.4.16 for
//! `RCC_APB2ENR.FWEN` and §9.2.2 for `SYSCFG_CFGR1`; ST **RM0432** §4; and ST
//! **AN4730** rev 2 §2.1 (DMA) and §2.2 (interrupts). Register offsets
//! cross-checked against ST's own CMSIS `FIREWALL_TypeDef`. No emulator source
//! of any licence was consulted (`ROADMAP.md` §1).

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
use crate::core::sync::{AtomicBool, LockRank, Mutex, Ordering};
use crate::core::value::{Endian, Width};
use crate::core::wire::{FanIn, Level, Resolve, WireId, WireSink, WireSource};
use crate::machine::Instance;
use crate::machine::realize::BindCtx;
use crate::machine::validate::{ClassSchema, PortDir, PropSchema};

/// The class name a machine description writes.
const CLASS_NAME: &str = "st.firewall";

/// The snapshot chunk version. Bump it with the encoding, never on its own.
const STATE_VERSION: u32 = 2;

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

/// How far a closed firewall has got through the two-fetch call-gate sequence.
///
/// RM0351 §4.3.6: the gate's second and third words "must not be interrupted by
/// any intermediate instruction fetch", so half a sequence is a state and not a
/// flag — and it has to remember *whose* gate it is, because the volatile data
/// segment has one of its own.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Gate {
    /// Nothing started, or an intermediate fetch threw the sequence away.
    Idle,
    /// The second word at `base + 4` was fetched; `base + 8` now opens it.
    Second(u64),
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
    /// How far through the call-gate sequence a closed firewall is.
    gate: Gate,
    /// `SYSCFG_CFGR1.FWDIS`, as the wire last delivered it.
    ///
    /// **Saved**, for the reason `st.syscfg` saves its input levels: it is a
    /// level a sibling is driving, and the order two devices' chunks load in is
    /// not something either of them may depend on. A restore that came back
    /// with this false would protect nothing until SYSCFG happened to
    /// republish.
    disabled: bool,
    /// `RCC_APB2ENR.FWEN`, as the `clken` wire last delivered it.
    ///
    /// The *pin level*, not the answer: a pin nothing drives sits low, and a
    /// board that declared no `clken` wire must still get a clocked block. So
    /// the question is asked through [`Shared::clocked`], which consults
    /// `clken_wired` first. Saved for the reason `disabled` is — it is a level
    /// a sibling drives, and chunk load order is nobody's to depend on.
    clocked: bool,
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
            gate: Gate::Idle,
            disabled: true,
            clocked: false,
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
    /// Whether a board drew a `clken` wire.
    ///
    /// Topology rather than state, so it is not serialized: it is set when the
    /// machine layer asks for the sink and it answers the question a bare pin
    /// level cannot, which is whether the low it is sitting at means "the RCC
    /// has not been told to clock this" or "nobody models an RCC here".
    clken_wired: AtomicBool,
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
    /// Whether the register block has a clock.
    ///
    /// A board that wired `clken` is asserting that the clock is the RCC's to
    /// give, so the pin decides; one that did not gets a clocked block, which
    /// is the same default `fwdis` takes in the other direction.
    fn clocked(&self, state: &State) -> bool {
        !self.clken_wired.load(Ordering::Relaxed) || state.clocked
    }

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

    /// Whether the volatile data segment is executable *protected* code.
    ///
    /// RM0351 §4.3.4: "The VDS bit gets priority over the VDE bit, this last
    /// bit value being ignored in such a case" — a shared segment is ordinary
    /// memory, so running out of it is running outside the fence, not inside
    /// it. Only `VDS = 0, VDE = 1` makes it part of the protected code.
    fn vds_is_protected_code(state: &State) -> bool {
        state.cr & CR_VDS == 0 && state.cr & CR_VDE != 0
    }

    /// Whether a fetch at `addr` counts as still being inside the protected
    /// code — the code segment, or an executable, unshared volatile segment.
    fn executing_inside(&self, state: &State, addr: u64, len: u64) -> bool {
        match self.segment_of(state, addr, len) {
            Segment::Code => true,
            Segment::Vds => Shared::vds_is_protected_code(state),
            _ => false,
        }
    }

    /// Where a call gate may live: the code segment always, and the volatile
    /// data segment while it is executable and unshared (RM0351 §4.3.6).
    ///
    /// A zero-length segment is not protected, so it has no gate either — the
    /// alternative would arm the sequence on unprotected memory.
    fn gate_bases(&self, state: &State) -> [Option<u64>; 2] {
        let (code, code_len) = self.code(state);
        let (vds, vds_len) = self.vds(state);
        [
            (code_len != 0).then_some(code),
            (vds_len != 0 && Shared::vds_is_protected_code(state)).then_some(vds),
        ]
    }

    /// Judge one access from the processor, advancing the state machine.
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
            // The processor is leaving, whether it meant to or because an
            // exception was taken: the firewall snoops addresses and cannot
            // tell those apart (§4.3.2). `FPA` decides which it is.
            if state.cr & CR_FPA == 0 {
                return Verdict::Reset;
            }
            state.cr &= !CR_FPA;
            state.open = false;
            state.gate = Gate::Idle;
            // And the fetch itself is now judged as a closed firewall would —
            // it lands outside the segments, so it falls through below.
        }

        if fetch {
            let bases = self.gate_bases(&state);
            if let Some(verdict) = Shared::judge_gate_fetch(&mut state, bases, addr) {
                return verdict;
            }
        }

        match self.segment_of(&state, addr, len) {
            Segment::Unprotected => Verdict::Allow,
            // The gate words were dealt with above, so anything still here is
            // the illegal entry the peripheral exists for: a jump into the
            // middle of the routine, a read of it by unprotected code, or the
            // dummy first word that exists to catch a runaway prefetch.
            Segment::Code | Segment::Nvds => Verdict::Reset,
            Segment::Vds => {
                // Shared means shared: "Read/write/execute accesses allowed if
                // VDS = 1 (whatever VDE bit value)" (§4.3.4, Table 18).
                if state.cr & CR_VDS != 0 {
                    Verdict::Allow
                } else {
                    Verdict::Reset
                }
            }
        }
    }

    /// The call-gate sequence, for a fetch made while the firewall is closed.
    ///
    /// `Some` when this fetch *is* part of a gate and needs no further
    /// judgement; `None` when it is an ordinary fetch, in which case any
    /// half-finished sequence has been thrown away by the time this returns —
    /// §4.3.6's "must not be interrupted by any intermediate instruction
    /// fetch".
    fn judge_gate_fetch(state: &mut State, bases: [Option<u64>; 2], addr: u64) -> Option<Verdict> {
        if let Gate::Second(base) = state.gate
            && addr == base.wrapping_add(8)
        {
            state.gate = Gate::Idle;
            state.open = true;
            return Some(Verdict::Allow);
        }
        state.gate = Gate::Idle;
        for base in bases.into_iter().flatten() {
            if addr == base.wrapping_add(4) {
                state.gate = Gate::Second(base);
                return Some(Verdict::Allow);
            }
            if addr == base.wrapping_add(8) {
                // The third word reached without the second, or after an
                // intermediate fetch threw the sequence away: "executing the
                // 3rd word after receiving the intermediate instruction fetch
                // would generate a system reset as a consequence".
                return Some(Verdict::Reset);
            }
        }
        None
    }

    /// Judge one access from a bus master.
    ///
    /// There is no state machine here and no `FW_CR` bit that softens it:
    /// "All DMA accesses to the protected segments are forbidden, whatever the
    /// Firewall state, and generate a system reset" (§4.3.4).
    fn judge_master(&self, addr: u64, len: u64) -> Verdict {
        let state = self.state.lock();
        if state.disabled {
            return Verdict::Allow;
        }
        match self.segment_of(&state, addr, len) {
            Segment::Unprotected => Verdict::Allow,
            _ => Verdict::Reset,
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

    /// Judge a processor access, and reset if that is the answer.
    fn filter_cpu(&self, addr: u64, len: u64, attrs: MemAttrs) -> MemResult {
        if attrs.debug {
            // No judgement and no state change: a debugger that rebooted the
            // machine by looking at it would be useless (`ROADMAP.md` §15).
            return Ok(());
        }
        self.settle(self.judge(addr, len, attrs.is_fetch()))
    }

    /// Judge a bus master's access, and reset if that is the answer.
    fn filter_master(&self, addr: u64, len: u64, attrs: MemAttrs) -> MemResult {
        if attrs.debug {
            return Ok(());
        }
        self.settle(self.judge_master(addr, len))
    }

    /// Turn a verdict into a result, pulsing the reset line outside the lock.
    fn settle(&self, verdict: Verdict) -> MemResult {
        match verdict {
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
        if !self.shared.clocked(&state) {
            // `RCC_APB2ENR.FWEN` is step 1 of §4.3.5's procedure and the block
            // is deaf until it is set. Returning zero rather than a bus fault
            // is what an STM32 does with an unclocked APB peripheral, and it is
            // also the failure a firmware that skipped the step would see:
            // every segment register reads back as it was never written.
            return 0;
        }
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
        if !self.shared.clocked(&state) {
            return;
        }
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

/// Which set of rules a filter region applies.
///
/// The two are genuinely different rule sets, not one with a flag: the
/// processor's is a state machine that fetches drive, and a master's is a fixed
/// address test that "whatever the Firewall state" makes independent of it.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Master {
    /// The processor, whose fetches move the state machine.
    Cpu,
    /// Any other bus master, for which every segment is always forbidden.
    Dma,
}

/// A master's whole address range, judged and then forwarded.
#[derive(Debug)]
struct Filter {
    shared: Arc<Shared>,
    master: Master,
}

impl Filter {
    /// Judge `[offset, offset + len)` by this region's rules.
    fn judge(&self, offset: u64, len: u64, attrs: MemAttrs) -> MemResult {
        match self.master {
            Master::Cpu => self.shared.filter_cpu(offset, len, attrs),
            Master::Dma => self.shared.filter_master(offset, len, attrs),
        }
    }
}

impl MemOps for Filter {
    fn read(&self, offset: u64, dst: &mut [u8], attrs: MemAttrs) -> MemResult {
        let Some(down) = self.shared.downstream() else {
            return Err(BusError::Unassigned);
        };
        self.judge(offset, dst.len() as u64, attrs)?;
        down.read_bytes(offset, dst, attrs)
    }

    fn write(&self, offset: u64, src: &[u8], attrs: MemAttrs) -> MemResult {
        let Some(down) = self.shared.downstream() else {
            return Err(BusError::Unassigned);
        };
        self.judge(offset, src.len() as u64, attrs)?;
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
    dma: RegionRef,
    /// The `fwdis` and `clken` input pins; the device keeps the strong
    /// references because a net holds its sinks weakly.
    pins: Mutex<Vec<Arc<InputPin>>>,
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
            clken_wired: AtomicBool::new(false),
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
                master: Master::Cpu,
            }) as Arc<dyn MemOps>,
        )) as RegionRef;
        let dma = Arc::new(Region::io(
            "firewall-dma",
            bus_size,
            Arc::new(Filter {
                shared: Arc::clone(&shared),
                master: Master::Dma,
            }) as Arc<dyn MemOps>,
        )) as RegionRef;
        Firewall {
            shared,
            regs,
            bus,
            dma,
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

    /// Whether the block's `RCC_APB2ENR.FWEN` clock is running.
    #[must_use]
    pub fn clocked(&self) -> bool {
        let state = self.shared.state.lock();
        self.shared.clocked(&state)
    }

    /// Drive `FWDIS` directly — the route a test with no SYSCFG takes.
    ///
    /// High is *disabled*, which is the sense of the bit and of the wire.
    pub fn set_fwdis(&self, level: bool) {
        let mut state = self.shared.state.lock();
        State::apply_fwdis(&mut state, level);
    }

    /// Drive `FWEN` directly — the route a test with no RCC takes.
    ///
    /// Driving the pin at all is what makes it load-bearing, exactly as a
    /// board's `wire` is.
    pub fn set_clken(&self, level: bool) {
        self.shared.clken_wired.store(true, Ordering::Relaxed);
        self.shared.state.lock().clocked = level;
    }
}

impl State {
    /// Take a new `FWDIS` level, from the wire or from a test.
    fn apply_fwdis(state: &mut State, high: bool) {
        state.disabled = high;
        if high {
            // Back to idle: nothing is checked, so nothing is open and no
            // half-finished call gate is remembered either.
            state.open = false;
            state.gate = Gate::Idle;
        }
    }
}

/// Which of the device's two level inputs an [`InputPin`] is.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Input {
    /// `SYSCFG_CFGR1.FWDIS`: high switches the firewall off.
    Fwdis,
    /// `RCC_APB2ENR.FWEN`: low leaves the register block unclocked.
    Clken,
}

/// One level input, as something a wire can drive.
#[derive(Debug)]
pub struct InputPin {
    shared: Arc<Shared>,
    which: Input,
    inputs: FanIn,
}

impl InputPin {
    /// The per-source levels currently seen.
    #[must_use]
    pub fn inputs(&self) -> &FanIn {
        &self.inputs
    }
}

impl WireSink for InputPin {
    fn set_level(&self, src: WireId, _line: u32, level: Level) {
        self.inputs.set(src, level);
        let high = self.inputs.resolve(Resolve::Or).is_high();
        let mut state = self.shared.state.lock();
        match self.which {
            Input::Fwdis => State::apply_fwdis(&mut state, high),
            Input::Clken => state.clocked = high,
        }
    }
}

impl Device for Firewall {
    fn class(&self) -> &'static DeviceClass {
        &CLASS
    }

    fn realize(&self, _ctx: &mut RealizeCtx<'_>) -> Result<()> {
        // Nothing outward: `bind` takes the space and `map` statements place
        // the three regions.
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
        // A half-finished call gate is guest-visible: restore into `Idle` and a
        // routine that had fetched `CSSA + 4` would reset on `CSSA + 8`.
        match state.gate {
            Gate::Idle => w.write_bool(false)?,
            Gate::Second(base) => {
                w.write_bool(true)?;
                w.write_u64(base)?;
            }
        }
        w.write_bool(state.disabled)?;
        w.write_bool(state.clocked)?;
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
            gate: if r.read_bool()? {
                Gate::Second(r.read_u64()?)
            } else {
                Gate::Idle
            },
            disabled: r.read_bool()?,
            clocked: r.read_bool()?,
        };
        *self.shared.state.lock() = state;
        Ok(())
    }

    fn region(&self, name: &str) -> Option<RegionRef> {
        match name {
            "" | "regs" => Some(Arc::clone(&self.regs)),
            "bus" => Some(Arc::clone(&self.bus)),
            "dma" => Some(Arc::clone(&self.dma)),
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
        let which = match port {
            "fwdis" => Input::Fwdis,
            "clken" => Input::Clken,
            _ => return None,
        };
        if which == Input::Clken {
            // A board that drew the wire has said the clock is the RCC's to
            // give, and an RCC's `FWEN` resets low.
            self.shared.clken_wired.store(true, Ordering::Relaxed);
        }
        let pin = Arc::new(InputPin {
            shared: Arc::clone(&self.shared),
            which,
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
            summary: "how much address space the `bus` and `dma` filters cover (4 GiB)",
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
        .region("dma")
        .port("reset", PortDir::Out)
        .port("fwdis", PortDir::In)
        .port("clken", PortDir::In)
}

#[cfg(test)]
mod tests;
