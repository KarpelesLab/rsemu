//! The STM32 system configuration controller.
//!
//! One class, `st.syscfg`. Most of it is a handful of latches nobody looks at
//! twice; the part that matters is `EXTICR1`…`EXTICR4`, the **pin multiplexer
//! in front of [`st.exti`](super::exti)**. EXTI line *n* is not wired to a
//! port: it is wired to whichever port's pin *n* these four registers select,
//! and a driver that enables an interrupt without writing them gets an
//! interrupt from the wrong pin — which is the single most common way an
//! STM32 button handler is wrong on real hardware.
//!
//! # The registers
//!
//! Written from two manuals, because ST changed the block between families and
//! `variant` selects which (RM0090 §9.2 for `"f4"`, RM0351 §9.2 for `"l4"`):
//!
//! | Offset | `"f4"` | `"l4"` |
//! | --- | --- | --- |
//! | `0x00` | `MEMRMP` | `MEMRMP` |
//! | `0x04` | `PMC` | `CFGR1` |
//! | `0x08`–`0x14` | `EXTICR1`–`EXTICR4` | `EXTICR1`–`EXTICR4` |
//! | `0x18` | — | `SCSR` |
//! | `0x1c` | — | `CFGR2` |
//! | `0x20` | `CMPCR` | `SWPR` |
//! | `0x24` | — | `SKR` |
//! | `0x28` | — | `SWPR2` |
//!
//! `EXTICR{k}` holds four four-bit fields, one per line, each naming a port:
//! `0` is A, `1` is B, and so on up to `ports - 1`. A field naming a port the
//! package does not have selects nothing and the line stays low, which is what
//! the die does with a pin that was never bonded.
//!
//! # How a pin reaches a line
//!
//! ```text
//! wire gpioa.p0     -> syscfg.pa0     # every port that may drive line 0
//! wire gpiob.p0     -> syscfg.pb0
//! wire syscfg.exti0 -> exti.line0     # whichever of them EXTICR1 selected
//! ```
//!
//! The mux is here rather than in `st.exti` because `EXTICR` is a `SYSCFG`
//! register: the selection is this block's state, it is what a snapshot has to
//! carry, and EXTI's own input is then one plain level per line whether that
//! line is a GPIO or the RTC alarm.
//!
//! Writing `EXTICR` **republishes** the newly selected port's level straight
//! away, so a firmware that points a line at a pin already sitting high
//! produces a rising edge on that line if it was low before. That is what the
//! hardware does and it is worth knowing: the classic symptom is one spurious
//! interrupt immediately after `HAL_GPIO_Init`.
//!
//! # `MEMRMP`, and why the boot alias is a window rather than a mapping
//!
//! `MEMRMP.MEM_MODE` picks what answers at address zero: the main flash, the
//! system bootloader, an external memory, or SRAM. A board that wants that to
//! work gives this object the space it lives in and the address of each
//! encoding's memory, and maps the block's **`boot` region** where the alias
//! belongs:
//!
//! ```text
//! object syscfg "st.syscfg" {
//!   space        = mem
//!   boot-size    = 1M
//!   boot-sources = [ 0x08000000, 0, 0, 0x20000000 ]   # index = MEM_MODE
//! }
//! map mem 0 size 1M = syscfg.boot
//! ```
//!
//! The `boot` region is a **decoder, not a mapping**: an access to it is
//! forwarded to `boot-sources[MEM_MODE]` plus the offset. That is deliberate
//! and it is not the obvious implementation, which would be to retopologise the
//! space from the `MEMRMP` write handler. It cannot be: that write arrives
//! *through* the space, so the topology lock is already held for reading on
//! this very thread, and [`AddressSpace::topology`] would deadlock against it
//! while [`AddressSpace::try_topology`] can only ever fail. The documented
//! alternative — a [`Deferred`](crate::core::device::Deferred) action — is
//! drained after a scheduler *event*, a whole quantum after the `LDR` that the
//! remap was supposed to redirect. A decoder has neither problem, and it is
//! also what the die is: the alias at zero is address decoding, not a second
//! copy of the flash.
//!
//! The sources are resolved once, at bind, into a private space of their own,
//! so a later retopology of the machine's map is not seen through the alias.
//! Nothing on an STM32 retopologises anything, and the alternative was to hold
//! a foreign space's read guard inside an MMIO handler.
//!
//! An encoding whose entry is `0` has no memory behind it — the F407 has
//! neither a system bootloader image nor an FSMC here — and reads back as a
//! bus fault, which is what an unmapped alias does. A board that gives no
//! `boot-sources` at all publishes no `boot` region and `MEMRMP` is then a
//! latch, as it was.
//!
//! # What is here and not modelled
//!
//! * **`SCSR`'s SRAM2 erase completes instantly and erases nothing**, because
//!   the RAM belongs to another object. `SRAM2BSY` therefore always reads zero
//!   and a polling loop finishes.
//! * **`SWPR`/`SWPR2`/`SKR` latch but do not protect.** The bits are set-only
//!   as the manual says and survive into a snapshot; no write to SRAM2 is
//!   refused because of them.
//!
//! # Sources
//!
//! ST **RM0090** rev 21 §9 "System configuration controller (SYSCFG)" and ST
//! **RM0351** rev 9 §9 for the L4 block. No emulator source of any licence was
//! consulted (`ROADMAP.md` §1).
//!
//! [`AddressSpace::topology`]: crate::core::space::AddressSpace::topology
//! [`AddressSpace::try_topology`]: crate::core::space::AddressSpace::try_topology

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
    AccessConstraints, AddressSpace, Mapping, MemAttrs, MemOps, MemResult, Region, RegionRef,
};
use crate::core::state::{ChunkReader, ChunkWriter, Sink, Source};
use crate::core::sync::{AtomicU64, LockRank, Mutex, Ordering};
use crate::core::value::{Endian, Width};
use crate::core::wire::{FanIn, Level, Resolve, WireId, WireSink, WireSource};
use crate::machine::Instance;
use crate::machine::realize::BindCtx;
use crate::machine::validate::{ClassSchema, PortDir, PropSchema, port_index};

/// The class name a machine description writes.
const CLASS_NAME: &str = "st.syscfg";

/// The snapshot chunk version. Bump it with the encoding, never on its own.
const STATE_VERSION: u32 = 1;

/// How many GPIO lines the mux covers. One per pin of a port.
pub const LINES: u32 = 16;

/// The port letters `EXTICR` can name, in field-value order.
///
/// `0` is A and `10` is K, which is as far as any STM32 goes (RM0090 §9.2.3
/// stops at I; RM0090's F429 adds J and K).
const PORT_LETTERS: &[u8] = b"abcdefghijk";

/// How many ports the widest part has.
pub const MAX_PORTS: u32 = 11;

/// How many `MEM_MODE` encodings the widest variant decodes.
///
/// Two bits on an F407 and three on an L4, so the table is eight long and an
/// F407 simply never selects past index 3.
pub const MEM_MODES: usize = 8;

/// The `boot-sources` entry that means "this encoding has no memory".
///
/// Spelled `0` in a machine file, because address zero is where the alias
/// *is*: an encoding that aliased the alias would be the one address a board
/// can never mean.
const NO_SOURCE: u64 = u64::MAX;

/// How many bytes an F4's registers occupy: up to and including `CMPCR`.
const F4_BYTES: u64 = 0x24;

/// How many bytes an L4's occupy: up to and including `SWPR2`.
const L4_BYTES: u64 = 0x2c;

/// `CFGR2`'s single-write-one-to-clear flag: SRAM2 parity error (RM0351
/// §9.2.6).
const CFGR2_SPF: u32 = 1 << 8;

/// `CFGR2`'s four lock bits, which software can set and only a reset clears.
const CFGR2_LOCKS: u32 = 0xf;

/// `CMPCR`'s I/O compensation cell power-down bit (RM0090 §9.2.8).
const CMPCR_CMP_PD: u32 = 1 << 0;

/// `CMPCR`'s ready flag, read-only.
const CMPCR_READY: u32 = 1 << 8;

/// `CFGR1`'s firewall-disable bit on an L4: reset high, and software may only
/// ever clear it.
const CFGR1_FWDIS: u32 = 1 << 0;

/// `MEMRMP`'s `MEM_MODE` field — two bits on an F4, three on an L4, and the
/// wider mask everywhere because the narrow variant's own mask has already cut
/// bit 2 off before this is applied.
const MEM_MODE_MASK: u32 = 0x7;

// ---------------------------------------------------------------------------
// Variants
// ---------------------------------------------------------------------------

/// Which family's `SYSCFG` this is.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Variant {
    /// The F2/F4 block: `MEMRMP`, `PMC`, `EXTICR`, `CMPCR`.
    F4,
    /// The L4 block: `MEMRMP`, `CFGR1`, `EXTICR`, `SCSR`, `CFGR2`, `SWPR`,
    /// `SKR`, `SWPR2`.
    L4,
}

impl Variant {
    /// The spelling a machine file writes.
    #[must_use]
    pub const fn name(self) -> &'static str {
        match self {
            Variant::F4 => "f4",
            Variant::L4 => "l4",
        }
    }

    /// How many bytes of registers the block decodes.
    #[must_use]
    pub const fn register_bytes(self) -> u64 {
        match self {
            Variant::F4 => F4_BYTES,
            Variant::L4 => L4_BYTES,
        }
    }

    /// `MEMRMP`'s writable bits.
    ///
    /// `MEM_MODE[1:0]` on an F407; an F42x/F43x adds `FB_MODE` and `SWP_FMC`
    /// and an L4 has a three-bit `MEM_MODE` plus `FB_MODE`. The narrower mask
    /// is the F407's, which is the part `machines/stm32f407.machine` models.
    #[must_use]
    const fn memrmp_mask(self) -> u32 {
        match self {
            Variant::F4 => 0x0000_0003,
            Variant::L4 => 0x0000_0107,
        }
    }

    /// The writable bits of the register at `0x04`.
    ///
    /// `PMC` on an F4: `ADCxDC2[18:16]` and `MII_RMII_SEL` at 23. `CFGR1` on
    /// an L4: `FWDIS` (handled separately), `BOOSTEN` at 8, the fast-mode-plus
    /// bits at 22:16 and `FPU_IE[31:26]`.
    #[must_use]
    const fn cfgr1_mask(self) -> u32 {
        match self {
            Variant::F4 => 0x0087_0000,
            Variant::L4 => 0xfc7f_0100,
        }
    }
}

// ---------------------------------------------------------------------------
// State
// ---------------------------------------------------------------------------

/// Everything the guest can see or change, plus the mux's input levels.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
struct State {
    memrmp: u32,
    /// `PMC` on an F4, `CFGR1` on an L4.
    cfgr1: u32,
    exticr: [u32; 4],
    /// `CMPCR.CMP_PD` on an F4; unused on an L4.
    cmp_pd: bool,
    cfgr2: u32,
    swpr: u32,
    swpr2: u32,
    /// One bit per pin, per port: the level that port is driving pin *n* at.
    ///
    /// **Saved**, and for the reason `st.exti` saves its own input latch: this
    /// is the level the mux *republishes* when `EXTICR` changes, so a snapshot
    /// that dropped it would have the next `EXTICR` write publish a low that
    /// is not what the port is at — a phantom falling edge on an EXTI line,
    /// arriving some indeterminate time after the restore.
    inputs: [u16; MAX_PORTS as usize],
}

// ---------------------------------------------------------------------------
// The boot alias
// ---------------------------------------------------------------------------

/// The window `MEMRMP` points: an address decoder, not a second mapping.
///
/// See the module documentation for why this forwards rather than
/// retopologising.
#[derive(Debug)]
struct BootAlias {
    /// What each `MEM_MODE` encoding aliases, or [`NO_SOURCE`].
    sources: [u64; MEM_MODES],
    /// Those sources, resolved at bind into a space of this block's own.
    ///
    /// Cloned out and the lock released before the forwarded access, because
    /// that access takes a [`LockRank::TOPOLOGY`] guard and this one is a leaf
    /// — holding it across would invert the ladder.
    space: Mutex<Option<Arc<AddressSpace>>>,
    /// Where `MEM_MODE` currently points. An atomic and not the register lock:
    /// this is read on every fetch the core makes through the alias.
    base: AtomicU64,
}

impl BootAlias {
    /// A decoder for `sources`, with every encoding empty until `select`.
    fn new(sources: [u64; MEM_MODES]) -> BootAlias {
        BootAlias {
            sources,
            space: Mutex::with_rank(LockRank::LEAF, None),
            base: AtomicU64::new(NO_SOURCE),
        }
    }

    /// Whether a board asked for a boot alias at all.
    fn configured(&self) -> bool {
        self.sources.iter().any(|s| *s != NO_SOURCE)
    }

    /// Point the window at what `MEM_MODE` selects.
    fn select(&self, mode: u32) {
        let base = self
            .sources
            .get(mode as usize)
            .copied()
            .unwrap_or(NO_SOURCE);
        self.base.store(base, Ordering::Relaxed);
    }

    /// The space and base an access should be forwarded to.
    fn target(&self) -> Option<(Arc<AddressSpace>, u64)> {
        let base = self.base.load(Ordering::Relaxed);
        if base == NO_SOURCE {
            return None;
        }
        let space = self.space.lock().clone()?;
        Some((space, base))
    }
}

impl MemOps for BootAlias {
    fn read(&self, offset: u64, dst: &mut [u8], attrs: MemAttrs) -> MemResult {
        // `attrs` travels unchanged, `MemAttrs::debug` included: a decoder has
        // no state to disturb and the memory behind it makes its own decision.
        let Some((space, base)) = self.target() else {
            return Err(BusError::Unassigned);
        };
        space.read_bytes(base.wrapping_add(offset), dst, attrs)
    }

    fn write(&self, offset: u64, src: &[u8], attrs: MemAttrs) -> MemResult {
        let Some((space, base)) = self.target() else {
            return Err(BusError::Unassigned);
        };
        space.write_bytes(base.wrapping_add(offset), src, attrs)
    }

    fn constraints(&self) -> AccessConstraints {
        // Whatever is on the far side decides; a decoder that imposed a width
        // of its own would refuse accesses the aliased memory accepts.
        AccessConstraints::ANY
    }
}

// ---------------------------------------------------------------------------
// The register block
// ---------------------------------------------------------------------------

/// The register block, as something an address space can dispatch to.
struct Registers {
    state: Mutex<State>,
    variant: Variant,
    /// How many ports this package bonds.
    ports: u32,
    /// `MEMRMP`'s reset value — the BOOT pins decide it, and a board says
    /// which.
    memrmp_reset: u32,
    /// The sixteen line outputs, connected at realize time.
    out: Mutex<[Option<WireSource>; LINES as usize]>,
    /// What `MEMRMP` points at address zero.
    boot: Arc<BootAlias>,
}

impl fmt::Debug for Registers {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let mut s = f.debug_struct("Registers");
        s.field("variant", &self.variant)
            .field("ports", &self.ports);
        match self.state.try_lock() {
            Some(state) => s.field("state", &*state),
            None => s.field("state", &"<locked>"),
        };
        s.finish()
    }
}

impl Registers {
    /// The reset state (RM0090 §9.2, RM0351 §9.2).
    fn reset_state(&self) -> State {
        State {
            memrmp: self.memrmp_reset & self.variant.memrmp_mask(),
            // An L4 comes out of reset with the firewall disabled; an F4's
            // `PMC` is all zero.
            cfgr1: match self.variant {
                Variant::F4 => 0,
                Variant::L4 => CFGR1_FWDIS,
            },
            ..State::default()
        }
    }

    /// Which port `EXTICR` has selected for line `n`, or `None` if it names one
    /// this package does not have.
    fn selected_port(&self, state: &State, n: u32) -> Option<u32> {
        let field = (state.exticr[(n / 4) as usize] >> ((n % 4) * 4)) & 0xf;
        (field < self.ports).then_some(field)
    }

    /// What each of the sixteen line outputs should be at.
    fn line_levels(&self, state: &State) -> u32 {
        let mut out = 0u32;
        for n in 0..LINES {
            let Some(port) = self.selected_port(state, n) else {
                continue;
            };
            if state.inputs[port as usize] & (1 << n) != 0 {
                out |= 1 << n;
            }
        }
        out
    }

    /// Drive the line outputs.
    ///
    /// Called with **no lock held**: an EXTI told a line moved will raise an
    /// interrupt from inside this call and the core may read back into this
    /// block, so the outward call happens after the critical section
    /// (`CLAUDE.md`, "Concurrency").
    fn drive(&self, levels: u32) {
        let sources: [Option<WireSource>; LINES as usize] = self.out.lock().clone();
        for (n, source) in sources.iter().enumerate() {
            let Some(source) = source else { continue };
            source.set(Level::from_bool(levels & (1 << n) != 0));
        }
    }

    /// Republish every line output from whatever the state now says.
    fn refresh(&self) {
        let levels = {
            let state = self.state.lock();
            self.line_levels(&state)
        };
        self.drive(levels);
    }

    /// A port drove one of its pins.
    fn set_pin(&self, port: u32, pin: u32, high: bool) {
        if port >= self.ports || pin >= LINES {
            return;
        }
        let levels = {
            let mut state = self.state.lock();
            let bit = 1u16 << pin;
            if high {
                state.inputs[port as usize] |= bit;
            } else {
                state.inputs[port as usize] &= !bit;
            }
            self.line_levels(&state)
        };
        self.drive(levels);
    }

    /// Read one register.
    fn read_register(&self, offset: u64) -> u32 {
        let state = self.state.lock();
        match (self.variant, offset) {
            (_, 0x00) => state.memrmp,
            (_, 0x04) => state.cfgr1,
            (_, 0x08..=0x14) if offset.is_multiple_of(4) => {
                state.exticr[((offset - 0x08) / 4) as usize]
            }
            (Variant::F4, 0x20) => {
                // "READY: compensation cell ready flag" — the cell is ready
                // once it has been powered, and this model has no settling
                // time to wait out.
                if state.cmp_pd {
                    CMPCR_CMP_PD | CMPCR_READY
                } else {
                    0
                }
            }
            // SRAM2ER self-clears the moment the erase is done, and here the
            // erase is not done at all, so both it and SRAM2BSY read zero.
            (Variant::L4, 0x18) => 0,
            (Variant::L4, 0x1c) => state.cfgr2,
            (Variant::L4, 0x20) => state.swpr,
            // "SKR: SRAM2 write protection key register" is write-only.
            (Variant::L4, 0x24) => 0,
            (Variant::L4, 0x28) => state.swpr2,
            _ => 0,
        }
    }

    /// Write one register. Returns whether a line output may have moved.
    fn write_register(&self, offset: u64, value: u32) -> bool {
        let mut state = self.state.lock();
        match (self.variant, offset) {
            (_, 0x00) => {
                state.memrmp = value & self.variant.memrmp_mask();
                // Moving the window is a relaxed store into the decoder, so it
                // is safe under the register lock and it lands before this
                // write's own bus cycle ends — which is what firmware that
                // remaps and branches to zero depends on.
                self.boot.select(state.memrmp & MEM_MODE_MASK);
            }
            (_, 0x04) => {
                let mut next = value & self.variant.cfgr1_mask();
                if self.variant == Variant::L4 {
                    // "FWDIS … this bit is cleared by software writing 0. It
                    // can only be set by a system reset." So the latch is one
                    // way and a write of one to an already-cleared bit does
                    // nothing.
                    next |= state.cfgr1 & CFGR1_FWDIS & value;
                }
                state.cfgr1 = next;
            }
            (_, 0x08..=0x14) if offset.is_multiple_of(4) => {
                state.exticr[((offset - 0x08) / 4) as usize] = value & 0xffff;
                return true;
            }
            (Variant::F4, 0x20) => state.cmp_pd = value & CMPCR_CMP_PD != 0,
            // The erase is a no-op, so there is nothing to start.
            (Variant::L4, 0x18) => {}
            (Variant::L4, 0x1c) => {
                // The four lock bits are set-only; `SPF` is write-one-to-clear.
                let locks = (state.cfgr2 | value) & CFGR2_LOCKS;
                let spf = state.cfgr2 & CFGR2_SPF & !(value & CFGR2_SPF);
                state.cfgr2 = locks | spf;
            }
            (Variant::L4, 0x20) => state.swpr |= value,
            (Variant::L4, 0x24) => {}
            (Variant::L4, 0x28) => state.swpr2 |= value,
            _ => {}
        }
        false
    }
}

impl MemOps for Registers {
    fn read(&self, offset: u64, dst: &mut [u8], _attrs: MemAttrs) -> MemResult {
        let [a, b, c, d] = dst else {
            return Err(BusError::BadAccess);
        };
        // Nothing here clears or advances on a read, so a debug read is the
        // same read (`ROADMAP.md` §15, invariant 5).
        let bytes = self.read_register(offset & !3).to_le_bytes();
        (*a, *b, *c, *d) = (bytes[0], bytes[1], bytes[2], bytes[3]);
        Ok(())
    }

    fn write(&self, offset: u64, src: &[u8], attrs: MemAttrs) -> MemResult {
        let [a, b, c, d] = src else {
            return Err(BusError::BadAccess);
        };
        if attrs.debug {
            // A debug write to `EXTICR` would move a line and could post an
            // interrupt; one to `CFGR1` would latch the firewall shut for
            // good. Neither has a harmless version.
            return Err(BusError::BadAccess);
        }
        let value = u32::from_le_bytes([*a, *b, *c, *d]);
        if self.write_register(offset & !3, value) {
            self.refresh();
        }
        Ok(())
    }

    fn constraints(&self) -> AccessConstraints {
        AccessConstraints::word(Width::U32, Endian::Little)
    }
}

// ---------------------------------------------------------------------------
// The device
// ---------------------------------------------------------------------------

/// An STM32 system configuration controller.
#[derive(Debug)]
pub struct Syscfg {
    regs: Arc<Registers>,
    region: RegionRef,
    /// The boot alias, when a board asked for one.
    boot_region: Option<RegionRef>,
    /// The input pins the machine layer has taken; the device keeps the strong
    /// reference because a net holds its sinks weakly.
    pins: Mutex<Vec<Arc<PortPin>>>,
}

impl Syscfg {
    /// Validate `props` and build the controller.
    ///
    /// # Errors
    ///
    /// [`Error::Property`] if a property is of the wrong kind or value, or if
    /// one this class does not know was given.
    pub fn new(props: &Props) -> Result<Syscfg> {
        let mut r = props.reader();
        let variant = match r.or_str("variant", "f4")? {
            "f4" => Variant::F4,
            "l4" => Variant::L4,
            other => {
                return Err(Error::Property(format!(
                    "`variant` is `f4` or `l4`, not `{other}`"
                )));
            }
        };
        let ports = r.or_range("ports", u64::from(MAX_PORTS), 1..=u64::from(MAX_PORTS))? as u32;
        let memrmp = r.or_range("memrmp-reset", 0u64, 0..=u64::from(u32::MAX))? as u32;
        let boot_size = r.or_size("boot-size", 0)?;
        let mut sources = [NO_SOURCE; MEM_MODES];
        if let Some(list) = r.optional_list("boot-sources")? {
            if list.len() > MEM_MODES {
                return Err(Error::Property(format!(
                    "`boot-sources` is indexed by MEM_MODE, so it has at most {MEM_MODES} \
                     entries, not {}",
                    list.len()
                )));
            }
            for (mode, value) in list.iter().enumerate() {
                let addr = value.to_addr("boot-sources")?;
                if addr != 0 {
                    sources[mode] = addr;
                }
            }
        }
        r.finish()?;
        if sources.iter().any(|s| *s != NO_SOURCE) && boot_size == 0 {
            return Err(Error::Property(String::from(
                "`boot-sources` names memory for the alias at zero but `boot-size` is 0, so the \
                 alias has no window to decode: give it the size of the widest source",
            )));
        }
        Ok(Syscfg::build_with_boot(
            variant, ports, memrmp, boot_size, sources,
        ))
    }

    /// Build one directly — the route a test takes.
    #[must_use]
    pub fn build(variant: Variant, ports: u32, memrmp_reset: u32) -> Syscfg {
        Syscfg::build_with_boot(variant, ports, memrmp_reset, 0, [NO_SOURCE; MEM_MODES])
    }

    /// Build one with a boot alias of `boot_size` bytes over `sources`.
    ///
    /// `sources` is indexed by `MEM_MODE`; [`NO_SOURCE`] is an encoding with no
    /// memory behind it.
    fn build_with_boot(
        variant: Variant,
        ports: u32,
        memrmp_reset: u32,
        boot_size: u64,
        sources: [u64; MEM_MODES],
    ) -> Syscfg {
        let ports = ports.clamp(1, MAX_PORTS);
        let boot = Arc::new(BootAlias::new(sources));
        let regs = Arc::new(Registers {
            state: Mutex::with_rank(LockRank::DEVICE, State::default()),
            variant,
            ports,
            memrmp_reset,
            out: Mutex::with_rank(LockRank::WIRE, [const { None }; LINES as usize]),
            boot: Arc::clone(&boot),
        });
        *regs.state.lock() = regs.reset_state();
        boot.select(regs.state.lock().memrmp & MEM_MODE_MASK);
        let region = Arc::new(Region::io(
            "syscfg",
            variant.register_bytes(),
            Arc::clone(&regs) as Arc<dyn MemOps>,
        ));
        let boot_region = (boot_size > 0)
            .then(|| Arc::new(Region::io("boot", boot_size, boot as Arc<dyn MemOps>)) as RegionRef);
        Syscfg {
            regs,
            region,
            boot_region,
            pins: Mutex::with_rank(LockRank::DEVICE, Vec::new()),
        }
    }

    /// Which family's block this is.
    #[must_use]
    pub fn variant(&self) -> Variant {
        self.regs.variant
    }

    /// How many ports the mux can select between.
    #[must_use]
    pub fn ports(&self) -> u32 {
        self.regs.ports
    }

    /// Drive port `port`'s pin `pin` — the route a test takes when there is no
    /// wire. Ports are numbered A = 0.
    pub fn set_pin(&self, port: u32, pin: u32, high: bool) {
        self.regs.set_pin(port, pin, high);
    }

    /// What line `n` is currently publishing to EXTI.
    #[must_use]
    pub fn line_level(&self, n: u32) -> bool {
        if n >= LINES {
            return false;
        }
        let state = self.regs.state.lock();
        self.regs.line_levels(&state) & (1 << n) != 0
    }

    /// Which port `EXTICR` has line `n` pointed at, or `None` for a field
    /// naming a port this package does not have.
    #[must_use]
    pub fn selected_port(&self, n: u32) -> Option<u32> {
        if n >= LINES {
            return None;
        }
        let state = self.regs.state.lock();
        self.regs.selected_port(&state, n)
    }

    /// `MEMRMP`'s `MEM_MODE` field — which memory answers at address zero.
    #[must_use]
    pub fn mem_mode(&self) -> u32 {
        self.regs.state.lock().memrmp & MEM_MODE_MASK
    }

    /// Resolve `boot-sources` against `space` and arm the alias.
    ///
    /// Normally done by [`Instance::bind`] from the object's `space =`
    /// property; a test that builds its own space calls this.
    ///
    /// # Errors
    ///
    /// [`Error::Config`] if an entry names an address `space` has nothing
    /// mapped at — the entry is the *base* of the memory the alias points at,
    /// so an address inside a region does not count.
    pub fn attach_boot_sources(&self, at: &str, space: &Arc<AddressSpace>) -> Result<()> {
        if !self.regs.boot.configured() {
            return Ok(());
        }
        // Resolve first, with only the machine's space held, and build the
        // private one afterwards: both guards are `LockRank::TOPOLOGY`, and
        // taking the second inside the first is the same rank twice.
        let mut found: Vec<Mapping> = Vec::new();
        {
            let view = space.view();
            for base in self.regs.boot.sources {
                if base == NO_SOURCE || found.iter().any(|m| m.base == base) {
                    continue;
                }
                let Some((_, mapping)) = view.mappings().find(|(_, m)| m.base == base) else {
                    return Err(Error::Config {
                        at: String::from(at),
                        message: format!(
                            "`boot-sources` names {base:#x}, and `{}` has nothing mapped there — \
                             the entry is the *base* of the memory the alias points at, not an \
                             address inside it",
                            space.name()
                        ),
                    });
                };
                found.push(mapping.clone());
            }
        }

        let sources = AddressSpace::new(format!("{at}.boot-sources"), space.bits());
        {
            let mut topo = sources.topology();
            for mapping in found {
                topo.map_with(mapping)?;
            }
        }
        *self.regs.boot.space.lock() = Some(Arc::new(sources));
        Ok(())
    }

    /// The guest address the boot alias currently forwards to, if any.
    ///
    /// `None` for an encoding this board gave no memory, and for a block with
    /// no `boot-sources` at all.
    #[must_use]
    pub fn boot_target(&self) -> Option<u64> {
        let base = self.regs.boot.base.load(Ordering::Relaxed);
        (base != NO_SOURCE).then_some(base)
    }
}

/// Port `letter` of `ports`, and the pin index, for a sink named `port`.
fn parse_pin(port: &str, ports: u32) -> Option<(u32, u32)> {
    for (index, letter) in PORT_LETTERS.iter().take(ports as usize).enumerate() {
        let prefix = format!("p{}", *letter as char);
        if let Some(pin) = port_index(port, &prefix, LINES) {
            return Some((index as u32, pin));
        }
    }
    None
}

impl Device for Syscfg {
    fn class(&self) -> &'static DeviceClass {
        &CLASS
    }

    fn realize(&self, _ctx: &mut RealizeCtx<'_>) -> Result<()> {
        // Nothing outward: a `map` statement places the region and the wire
        // graph brings the pins.
        Ok(())
    }

    fn reset(&self, _kind: ResetKind) {
        // Both kinds: `SYSCFG` is on the APB and has no battery behind it.
        // The pin levels survive, because they are what the ports are driving
        // and each port will keep driving them (`ROADMAP.md` §4.5) — this is
        // the mux, not the pad, and a mux that invented a level for its own
        // input would publish a falling edge nothing made.
        let memrmp = {
            let mut state = self.regs.state.lock();
            let inputs = state.inputs;
            *state = self.regs.reset_state();
            state.inputs = inputs;
            state.memrmp
        };
        // The BOOT pins decide what is at zero out of reset, so the window goes
        // back with the register.
        self.regs.boot.select(memrmp & MEM_MODE_MASK);
        self.regs.refresh();
    }

    fn save(&self, w: &mut ChunkWriter<'_>) -> Result<()> {
        let state = *self.regs.state.lock();
        w.write_u8(match self.regs.variant {
            Variant::F4 => 0,
            Variant::L4 => 1,
        })?;
        w.write_u32(self.regs.ports)?;
        w.write_u32(state.memrmp)?;
        w.write_u32(state.cfgr1)?;
        for value in state.exticr {
            w.write_u32(value)?;
        }
        w.write_bool(state.cmp_pd)?;
        w.write_u32(state.cfgr2)?;
        w.write_u32(state.swpr)?;
        w.write_u32(state.swpr2)?;
        w.write_seq_len(u64::from(MAX_PORTS))?;
        for value in state.inputs {
            w.write_u16(value)?;
        }
        Ok(())
    }

    fn load(&self, r: &mut ChunkReader<'_>) -> Result<()> {
        let variant = match r.read_u8()? {
            0 => Variant::F4,
            1 => Variant::L4,
            other => {
                return Err(Error::State(format!(
                    "snapshot has SYSCFG variant {other}, which this build does not know"
                )));
            }
        };
        let ports = r.read_u32()?;
        if variant != self.regs.variant || ports != self.regs.ports {
            return Err(Error::State(format!(
                "snapshot has a `{}` SYSCFG with {ports} port(s), this one is `{}` with {}",
                variant.name(),
                self.regs.variant.name(),
                self.regs.ports
            )));
        }
        let mut state = State {
            memrmp: r.read_u32()?,
            cfgr1: r.read_u32()?,
            ..State::default()
        };
        for value in &mut state.exticr {
            *value = r.read_u32()?;
        }
        state.cmp_pd = r.read_bool()?;
        state.cfgr2 = r.read_u32()?;
        state.swpr = r.read_u32()?;
        state.swpr2 = r.read_u32()?;
        let count = r.read_seq_len(2)?;
        if count != u64::from(MAX_PORTS) {
            return Err(Error::State(format!(
                "snapshot has {count} port(s) of pin levels, this build keeps {MAX_PORTS}"
            )));
        }
        for value in &mut state.inputs {
            *value = r.read_u16()?;
        }
        *self.regs.state.lock() = state;
        // The window is derived state and is never in the chunk: it is
        // recomputed from the `MEMRMP` that just came back.
        self.regs.boot.select(state.memrmp & MEM_MODE_MASK);
        self.regs.refresh();
        Ok(())
    }

    fn region(&self, name: &str) -> Option<RegionRef> {
        match name {
            "" | "regs" => Some(Arc::clone(&self.region)),
            "boot" => self.boot_region.clone(),
            _ => None,
        }
    }

    fn connect(&self, port: &str, source: WireSource) -> Result<()> {
        let n = port_index(port, "exti", LINES).ok_or_else(|| Error::Config {
            at: port.to_string(),
            message: String::from("a SYSCFG drives `exti0`…`exti15`"),
        })?;
        self.regs.out.lock()[n as usize] = Some(source);
        self.regs.refresh();
        Ok(())
    }

    fn announce(&self, port: &str) {
        // The mux's output is a function of its inputs, so it has to drive
        // here: a line pointed at a pin that is already high comes up high.
        if port_index(port, "exti", LINES).is_some() {
            self.regs.refresh();
        }
    }

    fn sink(&self, port: &str, sources: &[WireId]) -> Option<SinkPin> {
        let (index, pin) = parse_pin(port, self.regs.ports)?;
        let sink = Arc::new(PortPin {
            regs: Arc::clone(&self.regs),
            port: index,
            pin,
            inputs: FanIn::new(sources),
        });
        self.pins.lock().push(Arc::clone(&sink));
        Some(SinkPin { sink, line: pin })
    }
}

/// The machine layer's half: the boot alias has to be told what it aliases.
impl Instance for Syscfg {
    fn bind(&self, ctx: &BindCtx<'_>) -> Result<()> {
        if !self.regs.boot.configured() {
            return Ok(());
        }
        let space = ctx.space().ok_or_else(|| Error::Config {
            at: String::from(ctx.path()),
            message: String::from(
                "`boot-sources` names memory in the space this block lives in, so the object has \
                 to declare one: add `space = mem`",
            ),
        })?;
        self.attach_boot_sources(ctx.path(), space)
    }
}

/// The `st.syscfg` device class.
pub static CLASS: DeviceClass = DeviceClass {
    name: CLASS_NAME,
    version: STATE_VERSION,
    summary: "STM32 system configuration controller: MEMRMP, PMC/CFGR1 and the EXTICR pin mux",
    properties: &[
        PropertySpec {
            name: "variant",
            kind: ValueKind::Str,
            required: false,
            summary: "which family's register map: `f4` (default) or `l4`",
        },
        PropertySpec {
            name: "ports",
            kind: ValueKind::Uint,
            required: false,
            summary: "how many GPIO ports EXTICR can select between, A upwards",
        },
        PropertySpec {
            name: "memrmp-reset",
            kind: ValueKind::Uint,
            required: false,
            summary: "MEMRMP's reset value, which the BOOT pins decide",
        },
        PropertySpec {
            name: "boot-size",
            kind: ValueKind::Size,
            required: false,
            summary: "how many bytes the `boot` alias decodes; 0 publishes no alias",
        },
        PropertySpec {
            name: "boot-sources",
            kind: ValueKind::List,
            required: false,
            summary: "what each MEM_MODE aliases, by base address, indexed by MEM_MODE; 0 is none",
        },
    ],
    construct: |props| Ok(Box::new(Syscfg::new(props)?)),
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
    bindings.bind(CLASS_NAME, |props| Ok(Arc::new(Syscfg::new(props)?)))
}

/// What the validator should know about `st.syscfg`.
#[must_use]
pub fn schema() -> ClassSchema {
    let mut schema = ClassSchema::new(CLASS_NAME)
        .prop(PropSchema::new("variant", ValueKind::Str).values(&["f4", "l4"]))
        .prop(PropSchema::new("ports", ValueKind::Uint).range(1, u64::from(MAX_PORTS)))
        .prop(PropSchema::new("memrmp-reset", ValueKind::Uint).range(0, u64::from(u32::MAX)))
        .prop(PropSchema::new("boot-size", ValueKind::Size))
        .prop(PropSchema::new("boot-sources", ValueKind::List))
        .region("")
        .region("regs")
        .region("boot")
        .port_bank("exti", PortDir::Out, LINES);
    // One bank per port letter. The schema declares every letter the widest
    // part has, because it cannot see what `ports` was set to; the device
    // refuses a pin of a port this package does not bond.
    for letter in PORT_LETTERS {
        schema = schema.port_bank(format!("p{}", *letter as char), PortDir::In, LINES);
    }
    schema
}

// ---------------------------------------------------------------------------
// Input pins
// ---------------------------------------------------------------------------

/// One port's pin, as something a wire can drive.
///
/// Keeps a [`FanIn`] and wire-ORs its sources, because a wire hands each sink
/// the level of the *driver that changed* rather than the resolved level of
/// the net (`ROADMAP.md` §4.3).
#[derive(Debug)]
pub struct PortPin {
    regs: Arc<Registers>,
    port: u32,
    pin: u32,
    inputs: FanIn,
}

impl PortPin {
    /// Which port this pin belongs to, A = 0.
    #[must_use]
    pub fn port(&self) -> u32 {
        self.port
    }

    /// Which pin of that port it is.
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

impl WireSink for PortPin {
    fn set_level(&self, src: WireId, _line: u32, level: Level) {
        self.inputs.set(src, level);
        let high = self.inputs.resolve(Resolve::Or).is_high();
        self.regs.set_pin(self.port, self.pin, high);
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::props::Value;
    use crate::core::registry::Registry;
    use crate::core::state::{MachineShape, Migrations, StateReader, StateWriter};
    use crate::core::sync::{AtomicU32, Ordering};
    use crate::core::wire::{Wire, WireIdAllocator};

    /// An F407's: the F4 map and the nine ports RM0090 §9.2.3 lists.
    fn syscfg() -> Syscfg {
        Syscfg::build(Variant::F4, 9, 0)
    }

    fn peek(s: &Syscfg, offset: u64) -> u32 {
        let mut word = [0u8; 4];
        s.regs
            .read(offset, &mut word, MemAttrs::DEFAULT)
            .expect("a word read is legal");
        u32::from_le_bytes(word)
    }

    fn poke(s: &Syscfg, offset: u64, value: u32) {
        s.regs
            .write(offset, &value.to_le_bytes(), MemAttrs::DEFAULT)
            .expect("a word write is legal");
    }

    /// Point line `n` at port `port` (A = 0).
    fn select(s: &Syscfg, n: u32, port: u32) {
        let offset = 0x08 + u64::from(n / 4) * 4;
        let shift = (n % 4) * 4;
        let value = (peek(s, offset) & !(0xf << shift)) | (port << shift);
        poke(s, offset, value);
    }

    #[derive(Debug, Default)]
    struct Probe {
        level: AtomicU32,
        changes: AtomicU32,
    }

    impl WireSink for Probe {
        fn set_level(&self, _src: WireId, _line: u32, level: Level) {
            self.level
                .store(u32::from(level.is_high()), Ordering::Relaxed);
            self.changes.fetch_add(1, Ordering::Relaxed);
        }
    }

    fn watch(s: &Syscfg, port: &str) -> Arc<Probe> {
        let ids = WireIdAllocator::new();
        let id = ids.alloc();
        let probe = Arc::new(Probe::default());
        let wire = Wire::builder()
            .source(id)
            .sink(Arc::clone(&probe) as Arc<dyn WireSink>, 0)
            .build_shared();
        Device::connect(s, port, WireSource::new(wire, id)).expect("a pin this device drives");
        probe
    }

    fn high(p: &Probe) -> bool {
        p.level.load(Ordering::Relaxed) == 1
    }

    #[test]
    fn exticr_picks_which_port_drives_a_line() {
        let s = syscfg();
        let line0 = watch(&s, "exti0");
        // Reset is port A for every line.
        assert_eq!(s.selected_port(0), Some(0));
        s.set_pin(1, 0, true); // PB0 high
        assert!(!high(&line0), "line 0 is watching port A");
        s.set_pin(0, 0, true); // PA0 high
        assert!(high(&line0));

        // Point it at B; A goes low but B is high, so the line stays high.
        s.set_pin(0, 0, false);
        assert!(!high(&line0));
        select(&s, 0, 1);
        assert_eq!(s.selected_port(0), Some(1));
        assert!(high(&line0), "switching the mux republishes at once");
    }

    #[test]
    fn each_line_selects_its_own_pin_of_the_selected_port() {
        let s = syscfg();
        let line5 = watch(&s, "exti5");
        select(&s, 5, 2); // port C
        s.set_pin(2, 4, true);
        assert!(!high(&line5), "line 5 is pin 5, not pin 4");
        s.set_pin(2, 5, true);
        assert!(high(&line5));
        assert!(s.line_level(5));
    }

    #[test]
    fn exticr_fields_are_four_bits_and_four_to_a_register() {
        let s = syscfg();
        poke(&s, 0x08, 0x0000_3210);
        assert_eq!(peek(&s, 0x08), 0x3210);
        for n in 0..4 {
            assert_eq!(s.selected_port(n), Some(n));
        }
        // EXTICR2 covers lines 4–7, EXTICR3 8–11, EXTICR4 12–15.
        poke(&s, 0x14, 0x0000_0004);
        assert_eq!(s.selected_port(12), Some(4));
        // Only the low half-word is writable.
        poke(&s, 0x0c, 0xffff_ffff);
        assert_eq!(peek(&s, 0x0c), 0xffff);
    }

    #[test]
    fn a_port_this_package_does_not_bond_selects_nothing() {
        // An F407VG in an LQFP100 has A–E and H. Naming port J is a field the
        // die decodes and the package has no pin for.
        let s = syscfg();
        let line0 = watch(&s, "exti0");
        s.set_pin(0, 0, true);
        assert!(high(&line0));
        select(&s, 0, 9); // port J, past `ports = 9`
        assert_eq!(s.selected_port(0), None);
        assert!(!high(&line0));
        assert!(Device::sink(&s, "pj0", &[WireId::new(1)]).is_none());
        assert!(Device::sink(&s, "pi0", &[WireId::new(1)]).is_some());
    }

    #[test]
    fn a_pin_reaches_the_mux_through_a_wire() {
        let s = syscfg();
        let line3 = watch(&s, "exti3");
        select(&s, 3, 3); // port D
        let src = WireId::new(1);
        let pin = Device::sink(&s, "pd3", &[src]).expect("pd3");
        assert_eq!(pin.line, 3);
        // A net holds its sinks weakly, so the device has to own this one.
        let weak = Arc::downgrade(&pin.sink);
        drop(pin);
        let alive = weak.upgrade().expect("the controller still owns it");
        alive.set_level(src, 0, Level::High);
        assert!(high(&line3));
        alive.set_level(src, 0, Level::Low);
        assert!(!high(&line3));
    }

    #[test]
    fn memrmp_reads_back_and_only_mem_mode_is_writable() {
        let s = Syscfg::build(Variant::F4, 9, 0);
        assert_eq!(peek(&s, 0x00), 0);
        poke(&s, 0x00, 0x3);
        assert_eq!(peek(&s, 0x00), 0x3, "MEM_MODE = SRAM at zero");
        assert_eq!(s.mem_mode(), 3);
        poke(&s, 0x00, 0xffff_ffff);
        assert_eq!(peek(&s, 0x00), 0x3, "and only MEM_MODE is writable");

        // A board whose BOOT pins select the system bootloader says so.
        let boot = Syscfg::build(Variant::F4, 9, 1);
        assert_eq!(peek(&boot, 0x00), 1);
        poke(&boot, 0x00, 0);
        Device::reset(&boot, ResetKind::Cold);
        assert_eq!(peek(&boot, 0x00), 1);
    }

    /// An F407-shaped board: 64 KiB of "flash" at 0x08000000 and 16 KiB of
    /// "SRAM" at 0x20000000, with the alias over both at zero.
    ///
    /// Small stores rather than the part's real sizes, because what is being
    /// tested is the decode and not the geometry.
    fn boot_rig() -> (Arc<Syscfg>, Arc<AddressSpace>) {
        use crate::core::space::RamStore;

        const FLASH: u64 = 0x0800_0000;
        const SRAM: u64 = 0x2000_0000;
        let s = Arc::new(Syscfg::build_with_boot(
            Variant::F4,
            9,
            0,
            0x1_0000,
            [
                FLASH, NO_SOURCE, NO_SOURCE, SRAM, NO_SOURCE, NO_SOURCE, NO_SOURCE, NO_SOURCE,
            ],
        ));
        let space = Arc::new(AddressSpace::new("mem", 32));
        {
            let mut topo = space.topology();
            topo.map(
                Arc::new(Region::ram("flash", Arc::new(RamStore::new(0x1_0000)))) as RegionRef,
                FLASH,
            )
            .expect("flash maps");
            topo.map(
                Arc::new(Region::ram("sram", Arc::new(RamStore::new(0x4000)))) as RegionRef,
                SRAM,
            )
            .expect("sram maps");
            topo.map(Device::region(s.as_ref(), "boot").expect("a boot alias"), 0)
                .expect("the alias maps");
            topo.map(Arc::clone(&s.region), 0x4001_3800)
                .expect("the registers map");
        }
        (s, space)
    }

    /// Bind `s` to `space` the way the machine layer would.
    fn bind_boot(s: &Syscfg, space: &Arc<AddressSpace>) {
        s.attach_boot_sources("syscfg", space)
            .expect("the sources resolve");
    }

    #[test]
    fn memrmp_switches_the_boot_alias_to_sram() {
        // The whole point of the register: what is at zero changes.
        let (s, space) = boot_rig();
        bind_boot(&s, &space);

        let attrs = MemAttrs::DEFAULT;
        space
            .write(0x0800_0010, Width::U32, 0xf1a5_4001, attrs)
            .expect("flash is writable in this rig");
        space
            .write(0x2000_0010, Width::U32, 0x5a5a_0001, attrs)
            .expect("sram");

        assert_eq!(s.boot_target(), Some(0x0800_0000));
        assert_eq!(space.read(0x10, Width::U32, attrs).ok(), Some(0xf1a5_4001));

        // MEM_MODE = 11: SRAM1 at zero (RM0090 §9.2.1).
        space
            .write(0x4001_3800, Width::U32, 3, attrs)
            .expect("MEMRMP is writable");
        assert_eq!(s.boot_target(), Some(0x2000_0000));
        assert_eq!(
            space.read(0x10, Width::U32, attrs).ok(),
            Some(0x5a5a_0001),
            "0x00000000 still reads flash after a remap to SRAM"
        );

        // And it is the same memory, not a copy: a store through the alias is
        // visible at SRAM's own address.
        space
            .write(0x20, Width::U32, 0xdead_beef, attrs)
            .expect("the alias is writable");
        assert_eq!(
            space.read(0x2000_0020, Width::U32, MemAttrs::DEBUG).ok(),
            Some(0xdead_beef)
        );

        // Back to flash, and a reset takes the window back with the register.
        space
            .write(0x4001_3800, Width::U32, 0, attrs)
            .expect("back");
        assert_eq!(space.read(0x10, Width::U32, attrs).ok(), Some(0xf1a5_4001));
        space
            .write(0x4001_3800, Width::U32, 3, attrs)
            .expect("sram");
        Device::reset(s.as_ref(), ResetKind::Cold);
        assert_eq!(s.boot_target(), Some(0x0800_0000));
    }

    #[test]
    fn a_mem_mode_this_board_gave_no_memory_faults_rather_than_aliasing_the_wrong_thing() {
        // MEM_MODE = 01 is the system bootloader and 10 the FSMC; an F407VG in
        // an LQFP100 has neither modelled here, and a silent alias to flash
        // would be a board that lies about what it is.
        let (s, space) = boot_rig();
        bind_boot(&s, &space);
        space
            .write(0x4001_3800, Width::U32, 1, MemAttrs::DEFAULT)
            .expect("MEMRMP");
        assert_eq!(s.boot_target(), None);
        assert!(space.read(0x10, Width::U32, MemAttrs::DEFAULT).is_err());
    }

    #[test]
    fn an_unbound_alias_and_a_bad_source_are_both_refused_rather_than_silent() {
        // Nothing bound: the window has no space to forward into.
        let (_s, space) = boot_rig();
        assert!(space.read(0x10, Width::U32, MemAttrs::DEFAULT).is_err());

        // A `boot-sources` entry naming an address the space has nothing at is
        // a machine-file bug, and bind is where it is caught.
        let other = Arc::new(Syscfg::build_with_boot(
            Variant::F4,
            9,
            0,
            0x1000,
            [
                0xdead_0000,
                NO_SOURCE,
                NO_SOURCE,
                NO_SOURCE,
                NO_SOURCE,
                NO_SOURCE,
                NO_SOURCE,
                NO_SOURCE,
            ],
        ));
        assert!(other.attach_boot_sources("syscfg", &space).is_err());

        // And a block with no `boot-sources` publishes no alias at all.
        assert!(Device::region(&syscfg(), "boot").is_none());
    }

    #[test]
    fn boot_sources_without_a_window_is_a_property_error() {
        let props = Props::new().with(
            "boot-sources",
            Value::from(alloc::vec![Value::from(0x0800_0000u64)]),
        );
        assert!(Syscfg::new(&props).is_err(), "no `boot-size`");
        let sized = Props::new()
            .with(
                "boot-sources",
                Value::from(alloc::vec![Value::from(0x0800_0000u64)]),
            )
            .with("boot-size", Value::from(0x1000u64));
        assert!(Syscfg::new(&sized).is_ok());
    }

    #[test]
    fn the_f4_compensation_cell_is_ready_once_it_is_powered() {
        let s = syscfg();
        assert_eq!(peek(&s, 0x20), 0);
        poke(&s, 0x20, CMPCR_CMP_PD);
        assert_eq!(peek(&s, 0x20), CMPCR_CMP_PD | CMPCR_READY);
        // READY is read-only: writing it without CMP_PD leaves it clear.
        poke(&s, 0x20, CMPCR_READY);
        assert_eq!(peek(&s, 0x20), 0);
    }

    #[test]
    fn the_l4_map_is_a_different_map() {
        let s = Syscfg::build(Variant::L4, 8, 0);
        assert_eq!(s.variant().register_bytes(), L4_BYTES);
        // FWDIS comes up set and software may only ever clear it.
        assert_eq!(peek(&s, 0x04) & CFGR1_FWDIS, CFGR1_FWDIS);
        poke(&s, 0x04, 0);
        assert_eq!(peek(&s, 0x04) & CFGR1_FWDIS, 0);
        poke(&s, 0x04, CFGR1_FWDIS);
        assert_eq!(peek(&s, 0x04) & CFGR1_FWDIS, 0, "only a reset sets it");
        Device::reset(&s, ResetKind::Cold);
        assert_eq!(peek(&s, 0x04) & CFGR1_FWDIS, CFGR1_FWDIS);

        // CFGR2's locks are set-only and SPF is write-one-to-clear.
        poke(&s, 0x1c, CFGR2_LOCKS);
        assert_eq!(peek(&s, 0x1c), CFGR2_LOCKS);
        poke(&s, 0x1c, 0);
        assert_eq!(peek(&s, 0x1c), CFGR2_LOCKS);

        // SWPR and SWPR2 are set-only; SKR is write-only.
        poke(&s, 0x20, 0x0000_00f0);
        poke(&s, 0x20, 0x0000_0003);
        assert_eq!(peek(&s, 0x20), 0x0000_00f3);
        poke(&s, 0x28, 0x8000_0000);
        assert_eq!(peek(&s, 0x28), 0x8000_0000);
        poke(&s, 0x24, 0xca);
        assert_eq!(peek(&s, 0x24), 0, "SKR is write-only");
        // And the erase never leaves SRAM2 busy, so a polling loop finishes.
        poke(&s, 0x18, 1);
        assert_eq!(peek(&s, 0x18), 0);
    }

    #[test]
    fn an_f4_does_not_decode_the_l4_s_registers() {
        let s = syscfg();
        assert_eq!(s.variant().register_bytes(), F4_BYTES);
        poke(&s, 0x1c, 0xffff_ffff);
        assert_eq!(peek(&s, 0x1c), 0, "CFGR2 is not an F4 register");
    }

    #[test]
    fn a_debug_write_is_refused_and_a_debug_read_is_free() {
        let s = syscfg();
        s.set_pin(1, 0, true);
        assert_eq!(
            s.regs.write(0x08, &1u32.to_le_bytes(), MemAttrs::DEBUG),
            Err(BusError::BadAccess)
        );
        assert_eq!(s.selected_port(0), Some(0), "the mux did not move");
        let mut word = [0u8; 4];
        s.regs
            .read(0x08, &mut word, MemAttrs::DEBUG)
            .expect("a peek at EXTICR1 is free");
        assert_eq!(u32::from_le_bytes(word), 0);
    }

    #[test]
    fn a_reset_clears_the_mux_but_not_the_levels_the_ports_are_driving() {
        let s = syscfg();
        let line0 = watch(&s, "exti0");
        select(&s, 0, 1);
        s.set_pin(1, 0, true);
        s.set_pin(0, 0, true);
        assert!(high(&line0));
        Device::reset(&s, ResetKind::Warm);
        assert_eq!(s.selected_port(0), Some(0), "back to port A");
        assert!(high(&line0), "PA0 is still being held high by its port");
    }

    #[test]
    fn only_a_full_word_is_a_legal_access() {
        let s = syscfg();
        let mut byte = [0u8; 1];
        assert_eq!(
            s.regs.read(0x08, &mut byte, MemAttrs::DEFAULT),
            Err(BusError::BadAccess)
        );
        assert_eq!(
            s.regs.constraints(),
            AccessConstraints::word(Width::U32, Endian::Little)
        );
    }

    #[test]
    fn a_snapshot_round_trips_to_identical_state() {
        let saved = Syscfg::build(Variant::L4, 8, 1);
        poke(&saved, 0x00, 0x7);
        poke(&saved, 0x04, 0x0080_0100);
        poke(&saved, 0x08, 0x0000_1234);
        poke(&saved, 0x0c, 0x0000_5670);
        poke(&saved, 0x10, 0x0000_0007);
        poke(&saved, 0x14, 0x0000_0021);
        poke(&saved, 0x1c, 0x5);
        poke(&saved, 0x20, 0x0f0f_0f0f);
        poke(&saved, 0x28, 0x1234_5678);
        // EXTICR1 = 0x1234 points line 2 at port C, and EXTICR4's top field
        // is zero, so line 15 is still watching port A.
        saved.set_pin(2, 2, true);
        saved.set_pin(0, 15, true);

        let mut shape = MachineShape::new();
        shape.add_device("syscfg", CLASS_NAME).unwrap();
        let mut w = StateWriter::new(shape);
        {
            let mut chunk = w.chunk("syscfg", CLASS_NAME, STATE_VERSION).unwrap();
            Device::save(&saved, &mut chunk).unwrap();
        }
        let bytes = w.to_vec().unwrap();

        let restored = Syscfg::build(Variant::L4, 8, 1);
        let reader = StateReader::new(&bytes).unwrap();
        let chunk = reader
            .load("syscfg", CLASS_NAME, STATE_VERSION, &Migrations::new())
            .unwrap();
        Device::load(&restored, &mut chunk.reader()).unwrap();

        let offsets = [
            0x00, 0x04, 0x08, 0x0c, 0x10, 0x14, 0x18, 0x1c, 0x20, 0x24, 0x28,
        ];
        let before: Vec<u32> = offsets.iter().map(|o| peek(&saved, *o)).collect();
        let after: Vec<u32> = offsets.iter().map(|o| peek(&restored, *o)).collect();
        assert_eq!(before, after);
        // And the levels came across: the mux publishes the same lines.
        let lines_before: Vec<bool> = (0..LINES).map(|n| saved.line_level(n)).collect();
        let lines_after: Vec<bool> = (0..LINES).map(|n| restored.line_level(n)).collect();
        assert_eq!(lines_before, lines_after);
        assert!(lines_after.iter().any(|v| *v), "something was high");
    }

    #[test]
    fn a_snapshot_of_another_variant_is_refused() {
        let saved = Syscfg::build(Variant::L4, 8, 0);
        let mut shape = MachineShape::new();
        shape.add_device("syscfg", CLASS_NAME).unwrap();
        let mut w = StateWriter::new(shape);
        {
            let mut chunk = w.chunk("syscfg", CLASS_NAME, STATE_VERSION).unwrap();
            Device::save(&saved, &mut chunk).unwrap();
        }
        let bytes = w.to_vec().unwrap();
        let reader = StateReader::new(&bytes).unwrap();
        let chunk = reader
            .load("syscfg", CLASS_NAME, STATE_VERSION, &Migrations::new())
            .unwrap();
        assert!(Device::load(&syscfg(), &mut chunk.reader()).is_err());
    }

    #[test]
    fn a_property_this_class_does_not_know_is_a_typo() {
        let props = Props::new().with("variant", Value::from("l4"));
        assert_eq!(Syscfg::new(&props).unwrap().variant(), Variant::L4);
        assert!(Syscfg::new(&Props::new().with("variant", Value::from("f7"))).is_err());
        assert!(Syscfg::new(&Props::new().with("port-count", Value::from(9u64))).is_err());
        assert_eq!(Syscfg::new(&Props::new()).unwrap().ports(), MAX_PORTS);
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
        let s = Syscfg::build(Variant::F4, MAX_PORTS, 0);
        let schema = schema();
        let src = WireId::new(1);
        for name in ["pa0", "pb15", "pk7"] {
            assert!(schema.port_named(name).is_some(), "{name}");
            assert!(Device::sink(&s, name, &[src]).is_some(), "{name}");
        }
        assert!(schema.port_named("pl0").is_none());
        assert!(schema.port_named("pa16").is_none());
        for n in [0u32, 15] {
            assert!(schema.port_named(&format!("exti{n}")).is_some());
        }
        assert!(schema.port_named("exti16").is_none());
        assert!(Device::region(&s, "").is_some());
        assert!(Device::region(&s, "regs").is_some());
        assert!(Device::region(&s, "mux").is_none());
    }
}
