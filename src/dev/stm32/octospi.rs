//! The STM32 **OCTOSPI** memory interface, and its memory-mapped window.
//!
//! # What this peripheral is for
//!
//! An SPI controller moves bytes. OCTOSPI moves *a memory map*: in
//! **memory-mapped mode** the external flash appears as a window in the
//! address space, and a load or an instruction fetch inside that window is
//! turned by hardware into a complete flash frame — instruction, address,
//! alternate bytes, dummy cycles, data — built from the registers the driver
//! programmed once. That is what makes execute-in-place possible, and it is
//! the reason the peripheral exists rather than being a wider SPI block.
//!
//! Here, that is not a shortcut: a fetch from the window really does clock a
//! frame down [`crate::bus::spi`] to whatever slave is on the chip select, so
//! a guest executing out of this window is executing the bytes it programmed
//! into a real [`flash.spinor`](crate::dev::flash::spinor) through a page
//! program, not a copy the model kept on the side.
//!
//! # Which manual
//!
//! Written from ST's application note **AN5050** *"Getting started with
//! Octo-SPI … interfaces on STM32 MCUs"* (rev 14, and rev 1 / DocID030787 for
//! the STM32L4+ original), together with the register layouts ST publishes in
//! its **CMSIS device headers and HAL under BSD-3-Clause** — `stm32h7b3xx.h`,
//! `stm32u575xx.h`, `stm32h7xx_hal_ospi.h`, which are permissively licensed and
//! so may be read (`ROADMAP.md` §1). The reference manuals are **RM0432**
//! (L4+), **RM0455** and **RM0468** (H7A3/H7B3 and H723), **RM0438** (L5) and
//! **RM0456** (U5), chapter *"Octo-SPI interface (OCTOSPI)"`.
//!
//! **RM0433's STM32H7 has a QUADSPI, not an OCTOSPI.** Reaching for RM0433
//! because it says "H7" is the first mistake this peripheral invites, and it
//! gets you a different register file.
//!
//! # The four functional modes (`CR.FMODE`)
//!
//! | `FMODE` | Mode | How a transaction starts |
//! | --- | --- | --- |
//! | `00` | indirect write | writing `AR`, or `DR` when there is no address phase |
//! | `01` | indirect read | writing `AR`, or `IR` when there is no address phase |
//! | `10` | automatic status polling | as indirect read; the answer is matched against `PSMAR`/`PSMKR` |
//! | `11` | **memory-mapped** | a bus access inside the window |
//!
//! # Where the command comes from
//!
//! Three parallel sets of the same four registers, and which one is used is
//! the whole of the peripheral's cleverness:
//!
//! * `CCR`/`TCR`/`IR`/`ABR` — every indirect transaction, and a **read** from
//!   the memory-mapped window.
//! * `WCCR`/`WTCR`/`WIR`/`WABR` — a **write** to the memory-mapped window.
//! * `WPCCR`/`WPTCR`/`WPIR`/`WPABR` — a wrapped burst read. Stored and read
//!   back here; this model issues no wrapped bursts, because `DCR2.WRAPSIZE`
//!   describes a memory feature the parts on this fabric do not have.
//!
//! In memory-mapped mode the address comes from the **bus transaction**, never
//! from `AR`: AN5050 §3.3.3 is explicit that `DR` "has no meaning and returns
//! 0" and `DLR` "has no meaning" while `FMODE = 11`, and this model answers the
//! same way.
//!
//! # One data line, and what a `DCYC` means here
//!
//! `bus::spi` is a single-line fabric, so `CCR`'s `IMODE`/`ADMODE`/`ABMODE`/
//! `DMODE` are stored, read back, and used only to decide whether each phase
//! happens at all — not how many wires carry it. Dummy cycles are therefore
//! converted to whole bytes at **eight cycles to the byte**, which is exactly
//! right for every single-line command (`0Bh` fast read is `DCYC = 8`, one
//! byte) and is a documented approximation for a quad one. A configuration
//! copied from a real board's quad setup will not line up byte for byte with
//! the flash's expectations, and the module says so rather than inventing a
//! wire count the fabric does not have.
//!
//! # Time
//!
//! **Deliberately zero**, and this one is forced rather than chosen. A
//! memory-mapped access happens *inside a guest load*: the CPU is mid-access,
//! and there is no way for a device to yield to the scheduler and resume the
//! load later. So the frame is clocked to completion within the access, and
//! the peripheral takes no clock domain.
//!
//! The consequence is unusually tidy. `CR.TCEN` and `LPTR` exist so that
//! hardware releases `NCS` after an idle period, letting the memory drop into
//! standby (AN5050 §9.2.2); this model **already releases the chip select at
//! the end of every transaction**, so it is permanently in the state the
//! timeout counter exists to produce, and `TOF` correspondingly never sets.
//! The registers are stored and read back so a driver sees its own
//! configuration — `CR`'s `TCEN` (bit 3) with them, and `DMAEN` (bit 2), which
//! is inert for the plainer reason that nothing in this tree is a DMA peer
//! this peripheral could hand a burst to.
//!
//! # `MemAttrs::debug` and the window
//!
//! A debug read of the memory-mapped window is **refused**, and that is the
//! honest answer rather than a limitation worked around. Reaching the flash
//! means asserting a chip select and clocking a frame — moving another
//! device's command state machine, and interrupting whatever frame the guest
//! had in flight on the same bus. There is no side-effect-free route through a
//! bus. A debugger that wants the contents reads the flash device's own
//! snapshot chunk, which is where `ROADMAP.md` §4.5 puts the architectural
//! state anyway.

use alloc::boxed::Box;
use alloc::string::{String, ToString};
use alloc::sync::Arc;
use core::fmt;

use crate::bus::spi::{ChipSelect, Link, MAX_CHIP_SELECTS, SpiBus, buses};
use crate::core::device::{Device, DeviceClass, PropertySpec, RealizeCtx, ResetKind};
use crate::core::error::{BusError, Error, Result};
use crate::core::props::{Props, ValueKind};
use crate::core::space::{AccessConstraints, MemAttrs, MemOps, MemResult, Region, RegionRef};
use crate::core::state::{ChunkReader, ChunkWriter, Sink, Source};
use crate::core::sync::{AtomicBool, LockRank, Mutex, Ordering};
use crate::core::value::{Endian, Width};
use crate::core::wire::{Level, WireSource};
use crate::machine::realize::Instance;
use crate::machine::validate::{ClassSchema, PropSchema};

/// The class name a machine description writes.
pub const CLASS_NAME: &str = "stm32.octospi";

/// The snapshot chunk version. Bump with the encoding, never on its own.
const STATE_VERSION: u32 = 1;

/// How much address space the register block occupies.
pub const REGISTER_BYTES: u64 = 0x400;

/// The architectural memory-mapped aperture: 256 MiB per instance.
///
/// AN5050 §3.3.3 — OCTOSPI1 answers `0x9000_0000`-`0x9fff_ffff` and OCTOSPI2
/// `0x7000_0000`-`0x7fff_ffff`. The *base* belongs to the board and is a `map`
/// statement; only the size is the peripheral's.
pub const WINDOW_BYTES: u64 = 256 * 1024 * 1024;

// -- CR (offset 0x000) ------------------------------------------------------

/// Peripheral enable.
const CR_EN: u32 = 1 << 0;
/// Abort the transaction in flight. Self-clearing.
const CR_ABORT: u32 = 1 << 1;
/// FIFO threshold, bits 12:8.
const CR_FTHRES_SHIFT: u32 = 8;
/// And its mask, once shifted down.
const CR_FTHRES_MASK: u32 = 0x1f;
/// Transfer error interrupt enable.
const CR_TEIE: u32 = 1 << 16;
/// Transfer complete interrupt enable.
const CR_TCIE: u32 = 1 << 17;
/// FIFO threshold interrupt enable.
const CR_FTIE: u32 = 1 << 18;
/// Status match interrupt enable.
const CR_SMIE: u32 = 1 << 19;
/// Timeout interrupt enable.
const CR_TOIE: u32 = 1 << 20;
/// Automatic poll mode stop: polling ends on the first match.
const CR_APMS: u32 = 1 << 22;
/// Polling match mode: clear is "every masked bit matches", set is "any".
const CR_PMM: u32 = 1 << 23;
/// Functional mode, bits 29:28.
const CR_FMODE_SHIFT: u32 = 28;
/// And its mask, once shifted down.
const CR_FMODE_MASK: u32 = 0x3;

/// `FMODE` = indirect write.
const FMODE_WRITE: u32 = 0;
/// `FMODE` = indirect read.
const FMODE_READ: u32 = 1;
/// `FMODE` = automatic status polling.
const FMODE_POLL: u32 = 2;
/// `FMODE` = memory-mapped.
const FMODE_MAPPED: u32 = 3;

// -- DCR1 (0x008) -----------------------------------------------------------

/// Device size, bits 20:16. The part holds `2^(DEVSIZE + 1)` bytes.
const DCR1_DEVSIZE_SHIFT: u32 = 16;
/// And its mask, once shifted down.
const DCR1_DEVSIZE_MASK: u32 = 0x1f;

// -- SR (0x020) and FCR (0x024) --------------------------------------------

/// Transfer error: an access outside the device, or a refused transaction.
const SR_TEF: u32 = 1 << 0;
/// Transfer complete.
const SR_TCF: u32 = 1 << 1;
/// FIFO threshold reached. Hardware-cleared; there is no `FCR` bit for it.
const SR_FTF: u32 = 1 << 2;
/// Status match, in automatic polling mode.
const SR_SMF: u32 = 1 << 3;
/// Timeout. Never set here; see the module docs.
const SR_TOF: u32 = 1 << 4;
/// Busy: a transaction is in flight and the chip select is asserted.
const SR_BUSY: u32 = 1 << 5;
/// FIFO level, bits 13:8.
const SR_FLEVEL_SHIFT: u32 = 8;
/// And its mask, once shifted down.
const SR_FLEVEL_MASK: u32 = 0x3f;

/// The `FCR` bits, which are the write-one-to-clear halves of `SR`.
const FCR_MASK: u32 = SR_TEF | SR_TCF | SR_SMF | SR_TOF;

/// How many bytes the FIFO holds, for `FLEVEL`.
///
/// The model produces a byte on demand rather than buffering ahead, so
/// `FLEVEL` reports how much of the transaction is still to come, capped here.
/// A driver uses it to decide how many `DR` reads are safe, and that answer is
/// the same either way.
const FIFO_BYTES: u64 = 32;

// -- CCR (0x100) ------------------------------------------------------------

/// Instruction mode, bits 2:0. Zero skips the phase.
const CCR_IMODE_SHIFT: u32 = 0;
/// Instruction size, bits 5:4: `n + 1` bytes.
const CCR_ISIZE_SHIFT: u32 = 4;
/// Address mode, bits 10:8.
const CCR_ADMODE_SHIFT: u32 = 8;
/// Address size, bits 13:12.
const CCR_ADSIZE_SHIFT: u32 = 12;
/// Alternate-byte mode, bits 18:16.
const CCR_ABMODE_SHIFT: u32 = 16;
/// Alternate-byte size, bits 21:20.
const CCR_ABSIZE_SHIFT: u32 = 20;
/// Data mode, bits 26:24.
const CCR_DMODE_SHIFT: u32 = 24;
/// A three-bit `*MODE` field.
const MODE_MASK: u32 = 0x7;
/// A two-bit `*SIZE` field.
const SIZE_MASK: u32 = 0x3;

// -- TCR (0x108) ------------------------------------------------------------

/// Dummy cycles, bits 4:0.
const TCR_DCYC_MASK: u32 = 0x1f;

/// What an idle SPI line reads as, and what this peripheral drives during a
/// dummy or a read.
const IDLE_BYTE: u8 = 0xff;

fn config(message: String) -> Error {
    Error::Config {
        at: CLASS_NAME.to_string(),
        message,
    }
}

// ---------------------------------------------------------------------------
// the command a set of registers describes
// ---------------------------------------------------------------------------

/// One of the three parallel register sets, decoded.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct Command {
    /// Whether the instruction phase happens, and the instruction itself.
    instruction: Option<(u32, u8)>,
    /// How many address bytes, or none.
    address_bytes: u8,
    /// The alternate bytes and how many of them, or none.
    alternate: Option<(u32, u8)>,
    /// How many dummy bytes sit between the header and the data.
    dummy_bytes: u8,
    /// Whether a data phase happens at all.
    has_data: bool,
}

impl Command {
    /// Decode `ccr` and `tcr` with the instruction and alternate bytes from
    /// `ir` and `abr`.
    fn decode(ccr: u32, tcr: u32, ir: u32, abr: u32) -> Command {
        let field = |shift: u32, mask: u32| (ccr >> shift) & mask;
        let imode = field(CCR_IMODE_SHIFT, MODE_MASK);
        let admode = field(CCR_ADMODE_SHIFT, MODE_MASK);
        let abmode = field(CCR_ABMODE_SHIFT, MODE_MASK);
        let dmode = field(CCR_DMODE_SHIFT, MODE_MASK);
        // Every `*SIZE` field is `n + 1` bytes: 8, 16, 24 or 32 bits.
        let isize = (field(CCR_ISIZE_SHIFT, SIZE_MASK) + 1) as u8;
        let asize = (field(CCR_ADSIZE_SHIFT, SIZE_MASK) + 1) as u8;
        let absize = (field(CCR_ABSIZE_SHIFT, SIZE_MASK) + 1) as u8;
        Command {
            instruction: (imode != 0).then_some((ir, isize)),
            address_bytes: if admode != 0 { asize } else { 0 },
            alternate: (abmode != 0).then_some((abr, absize)),
            // Eight cycles to the byte. The module docs say why, and what it
            // costs on a multi-line configuration.
            dummy_bytes: ((tcr & TCR_DCYC_MASK).div_ceil(8)) as u8,
            has_data: dmode != 0,
        }
    }
}

// ---------------------------------------------------------------------------
// state
// ---------------------------------------------------------------------------

/// Everything the guest can see or change.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
struct State {
    cr: u32,
    dcr1: u32,
    dcr2: u32,
    dcr3: u32,
    dcr4: u32,
    sr: u32,
    dlr: u32,
    ar: u32,
    psmkr: u32,
    psmar: u32,
    pir: u32,
    ccr: u32,
    tcr: u32,
    ir: u32,
    abr: u32,
    lptr: u32,
    wpccr: u32,
    wptcr: u32,
    wpir: u32,
    wpabr: u32,
    wccr: u32,
    wtcr: u32,
    wir: u32,
    wabr: u32,
    hlcr: u32,
    /// Whether an indirect transaction is open, with the chip select asserted.
    ///
    /// This is the state a snapshot must carry that nothing else would think
    /// to: a machine saved between a driver's `AR` write and its last `DR`
    /// read has a frame open on the flash, and restoring it as idle would
    /// leave the part selected forever.
    open: bool,
    /// Whether the open transaction is writing rather than reading.
    writing: bool,
    /// How many data bytes of it are still to move.
    remaining: u64,
}

impl State {
    /// The functional mode `CR` selects.
    const fn fmode(&self) -> u32 {
        (self.cr >> CR_FMODE_SHIFT) & CR_FMODE_MASK
    }

    /// How many bytes the attached memory holds: `2^(DEVSIZE + 1)`
    /// (AN5050 §5.2).
    const fn device_bytes(&self) -> u64 {
        1u64 << (((self.dcr1 >> DCR1_DEVSIZE_SHIFT) & DCR1_DEVSIZE_MASK) + 1)
    }

    /// The FIFO threshold, in bytes.
    const fn fifo_threshold(&self) -> u64 {
        (((self.cr >> CR_FTHRES_SHIFT) & CR_FTHRES_MASK) + 1) as u64
    }

    /// `SR`, with the live fields folded in.
    fn status(&self) -> u32 {
        let mut sr = self.sr & !(SR_BUSY | SR_FTF | (SR_FLEVEL_MASK << SR_FLEVEL_SHIFT));
        if self.open {
            sr |= SR_BUSY;
        }
        let level = if self.open && !self.writing {
            self.remaining.min(FIFO_BYTES)
        } else {
            0
        };
        sr |= (level as u32 & SR_FLEVEL_MASK) << SR_FLEVEL_SHIFT;
        if self.open && (self.writing || level >= self.fifo_threshold()) {
            sr |= SR_FTF;
        }
        sr
    }

    /// The read command: `CCR`/`TCR`/`IR`/`ABR`.
    fn read_command(&self) -> Command {
        Command::decode(self.ccr, self.tcr, self.ir, self.abr)
    }

    /// The write command: `WCCR`/`WTCR`/`WIR`/`WABR`.
    fn write_command(&self) -> Command {
        Command::decode(self.wccr, self.wtcr, self.wir, self.wabr)
    }
}

// ---------------------------------------------------------------------------
// the device
// ---------------------------------------------------------------------------

/// An STM32 OCTOSPI memory interface.
#[derive(Debug)]
pub struct Octospi {
    shared: Arc<Shared>,
    regs: RegionRef,
    window: RegionRef,
}

struct Shared {
    state: Mutex<State>,
    bus: Option<Arc<SpiBus>>,
    cs: ChipSelect,
    /// How big the memory-mapped aperture this instance decodes is.
    window: u64,
    /// The interrupt output, connected at realize time.
    irq: Mutex<Option<WireSource>>,
    /// The level it is being held at. An atomic so a debug read is free.
    irq_level: AtomicBool,
}

impl fmt::Debug for Shared {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let mut s = f.debug_struct("Shared");
        s.field("cs", &self.cs).field("window", &self.window);
        match self.state.try_lock() {
            Some(state) => s.field("state", &*state).finish(),
            None => s.field("state", &"<in use>").finish(),
        }
    }
}

impl Shared {
    /// Whether any enabled source is asserting.
    fn irq_state(state: &State) -> bool {
        let sr = state.status();
        let cr = state.cr;
        (cr & CR_TEIE != 0 && sr & SR_TEF != 0)
            || (cr & CR_TCIE != 0 && sr & SR_TCF != 0)
            || (cr & CR_FTIE != 0 && sr & SR_FTF != 0)
            || (cr & CR_SMIE != 0 && sr & SR_SMF != 0)
            || (cr & CR_TOIE != 0 && sr & SR_TOF != 0)
    }

    /// Re-drive the interrupt line from the state.
    fn publish_irq(&self) {
        let level = Level::from_bool(Shared::irq_state(&self.state.lock()));
        self.irq_level.store(level.is_high(), Ordering::Relaxed);
        let port = self.irq.lock().clone();
        if let Some(port) = port {
            port.set(level);
        }
    }

    /// Exchange one byte with whatever is on the chip select.
    fn byte(&self, out: u8) -> u8 {
        self.bus
            .as_ref()
            .map_or(IDLE_BYTE, |bus| bus.transfer(u32::from(out)) as u8)
    }

    /// Assert the chip select and clock the header phases of `cmd`.
    ///
    /// Called with no lock of ours held: it reaches another device.
    fn open(&self, cmd: &Command, address: u32) {
        if let Some(bus) = &self.bus {
            bus.select(Some(self.cs));
        }
        if let Some((instruction, bytes)) = cmd.instruction {
            for i in (0..bytes).rev() {
                self.byte((instruction >> (8 * u32::from(i))) as u8);
            }
        }
        for i in (0..cmd.address_bytes).rev() {
            self.byte((address >> (8 * u32::from(i))) as u8);
        }
        if let Some((alternate, bytes)) = cmd.alternate {
            for i in (0..bytes).rev() {
                self.byte((alternate >> (8 * u32::from(i))) as u8);
            }
        }
        for _ in 0..cmd.dummy_bytes {
            self.byte(IDLE_BYTE);
        }
    }

    /// Release the chip select.
    fn close(&self) {
        if let Some(bus) = &self.bus {
            bus.select(None);
        }
    }

    /// Run a complete self-contained frame and return what came back.
    fn frame(&self, cmd: &Command, address: u32, data: &mut [u8], writing: bool) {
        self.open(cmd, address);
        if cmd.has_data {
            for byte in data.iter_mut() {
                let got = self.byte(if writing { *byte } else { IDLE_BYTE });
                if !writing {
                    *byte = got;
                }
            }
        }
        self.close();
    }
}

impl Octospi {
    /// Validate `props` and build the peripheral.
    ///
    /// Properties:
    ///
    /// * `link` — required, and `"transactional"` is the only value this
    ///   peripheral can take; see the error text for why.
    /// * `bus` — the named [`SpiBus`] the memory sits on. Required.
    /// * `cs` — which chip select the memory answers on.
    /// * `window` — how much address space the memory-mapped aperture decodes.
    ///   Defaults to the architectural 256 MiB; a board that maps less says so
    ///   here rather than truncating the region in its `map` statement.
    ///
    /// # Errors
    ///
    /// [`Error::Property`] for an unknown or missing property,
    /// [`Error::Config`] for an unusable `link`, a `cs` out of range, or a
    /// `window` that is not a power of two.
    pub fn new(props: &Props) -> Result<Octospi> {
        let mut r = props.reader();
        let link_name = r.require_str("link")?.to_string();
        let bus_name = r.require_str("bus")?.to_string();
        let cs = r.or_range("cs", 0u64, 0..=(MAX_CHIP_SELECTS as u64 - 1))?;
        let window = r.or_size("window", WINDOW_BYTES)?;
        r.finish()?;

        let link = Link::from_name(&link_name).ok_or_else(|| {
            config(alloc::format!(
                "`link` is `{link_name}`; it must be one of {:?} — see docs/buses/low-speed.md",
                Link::NAMES
            ))
        })?;
        if link != Link::Transactional {
            return Err(config(String::from(
                "OCTOSPI can only be modelled `transactional`, and the property is still \
                 required so the machine file says so out loud: a memory-mapped access happens \
                 inside a guest load, and a `wired` link would have to pace edges through the \
                 scheduler while the CPU is mid-access — which the core cannot express. Put a \
                 `spi.controller` or an `stm32.spi` on the bus if you want the edges.",
            )));
        }
        if !window.is_power_of_two() || window > WINDOW_BYTES {
            return Err(config(alloc::format!(
                "`window` is {window:#x}; the aperture is a power of two of at most \
                 {WINDOW_BYTES:#x} (AN5050 §3.3.3's 256 MiB per instance)"
            )));
        }
        let bus = buses::attach(props, &bus_name)?;
        Ok(Octospi::with_bus(Some(bus), ChipSelect(cs as u8), window))
    }

    /// A peripheral on a bus the caller already holds.
    #[must_use]
    pub fn with_bus(bus: Option<Arc<SpiBus>>, cs: ChipSelect, window: u64) -> Octospi {
        let shared = Arc::new(Shared {
            state: Mutex::with_rank(LockRank::DEVICE, State::default()),
            bus,
            cs,
            window,
            irq: Mutex::with_rank(LockRank::WIRE, None),
            irq_level: AtomicBool::new(false),
        });
        let regs: RegionRef = Arc::new(Region::io(
            "octospi",
            REGISTER_BYTES,
            Arc::new(RegisterBlock {
                shared: Arc::clone(&shared),
            }) as Arc<dyn MemOps>,
        ));
        let window_region: RegionRef = Arc::new(Region::io(
            "octospi-mem",
            window,
            Arc::new(MappedWindow {
                shared: Arc::clone(&shared),
            }) as Arc<dyn MemOps>,
        ));
        Octospi {
            shared,
            regs,
            window: window_region,
        }
    }

    /// The bus the memory sits on.
    #[must_use]
    pub fn bus(&self) -> Option<&Arc<SpiBus>> {
        self.shared.bus.as_ref()
    }

    /// How much address space the memory-mapped aperture decodes.
    #[must_use]
    pub fn window_bytes(&self) -> u64 {
        self.shared.window
    }

    /// `SR`, as software would read it.
    #[must_use]
    pub fn status(&self) -> u32 {
        self.shared.state.lock().status()
    }

    /// Whether a transaction is open with the chip select asserted.
    #[must_use]
    pub fn busy(&self) -> bool {
        self.shared.state.lock().open
    }

    /// Whether the interrupt output is asserted.
    #[must_use]
    pub fn irq_asserted(&self) -> bool {
        self.shared.irq_level.load(Ordering::Relaxed)
    }
}

// ---------------------------------------------------------------------------
// the register block
// ---------------------------------------------------------------------------

struct RegisterBlock {
    shared: Arc<Shared>,
}

impl fmt::Debug for RegisterBlock {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("RegisterBlock").finish_non_exhaustive()
    }
}

/// What a register write asks for once the state lock is released.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum After {
    Nothing,
    /// Open a transaction: this command, at this address, writing or reading.
    Start {
        cmd: Command,
        address: u32,
        writing: bool,
    },
    /// The transaction ended; release the chip select.
    Close,
    /// Run one automatic-status poll of this command at this address.
    Poll {
        cmd: Command,
        address: u32,
    },
}

impl RegisterBlock {
    fn read_register(&self, offset: u64, debug: bool) -> u32 {
        let state = self.shared.state.lock();
        match offset {
            0x000 => state.cr,
            0x008 => state.dcr1,
            0x00c => state.dcr2,
            0x010 => state.dcr3,
            0x014 => state.dcr4,
            0x020 => state.status(),
            0x040 => state.dlr,
            0x048 => state.ar,
            // `DR` is handled by the caller, which has to clock the bus and so
            // cannot hold this lock. AN5050 §3.3.3: in memory-mapped mode it
            // "has no meaning and returns 0".
            0x050 => 0,
            0x080 => state.psmkr,
            0x088 => state.psmar,
            0x090 => state.pir,
            0x100 => state.ccr,
            0x108 => state.tcr,
            0x110 => state.ir,
            0x120 => state.abr,
            0x130 => state.lptr,
            0x140 => state.wpccr,
            0x148 => state.wptcr,
            0x150 => state.wpir,
            0x160 => state.wpabr,
            0x180 => state.wccr,
            0x188 => state.wtcr,
            0x190 => state.wir,
            0x1a0 => state.wabr,
            0x200 => state.hlcr,
            _ => {
                let _ = debug;
                0
            }
        }
    }

    fn write_register(&self, offset: u64, value: u32) -> After {
        let mut state = self.shared.state.lock();
        match offset {
            0x000 => {
                let was_enabled = state.cr & CR_EN != 0;
                state.cr = value & !CR_ABORT;
                if value & CR_ABORT != 0 || (was_enabled && state.cr & CR_EN == 0) {
                    // An abort — or disabling the peripheral — ends whatever
                    // was in flight and releases the chip select. `ABORT` is
                    // self-clearing, which is why it is masked out above.
                    if state.open {
                        state.open = false;
                        state.remaining = 0;
                        state.sr |= SR_TCF;
                        return After::Close;
                    }
                }
                After::Nothing
            }
            0x008 => {
                state.dcr1 = value;
                After::Nothing
            }
            0x00c => {
                state.dcr2 = value;
                After::Nothing
            }
            0x010 => {
                state.dcr3 = value;
                After::Nothing
            }
            0x014 => {
                state.dcr4 = value;
                After::Nothing
            }
            // `SR` is read-only; `FCR` clears what it is written with.
            0x020 => After::Nothing,
            0x024 => {
                state.sr &= !(value & FCR_MASK);
                After::Nothing
            }
            0x040 => {
                state.dlr = value;
                After::Nothing
            }
            0x048 => {
                state.ar = value;
                // Writing `AR` is what triggers an indirect transaction that
                // has an address phase.
                self.trigger(&mut state, value)
            }
            0x080 => {
                state.psmkr = value;
                After::Nothing
            }
            0x088 => {
                state.psmar = value;
                After::Nothing
            }
            0x090 => {
                state.pir = value;
                After::Nothing
            }
            0x100 => {
                state.ccr = value;
                // Deliberately not a trigger. A transaction with no address
                // phase starts when the *instruction* is written, and a
                // driver programs `CCR` before `IR`; triggering here would
                // clock a frame carrying whatever `IR` happened to hold.
                After::Nothing
            }
            0x108 => {
                state.tcr = value;
                After::Nothing
            }
            0x110 => {
                state.ir = value;
                if state.read_command().address_bytes == 0 {
                    self.trigger(&mut state, 0)
                } else {
                    After::Nothing
                }
            }
            0x120 => {
                state.abr = value;
                After::Nothing
            }
            0x130 => {
                state.lptr = value;
                After::Nothing
            }
            0x140 => {
                state.wpccr = value;
                After::Nothing
            }
            0x148 => {
                state.wptcr = value;
                After::Nothing
            }
            0x150 => {
                state.wpir = value;
                After::Nothing
            }
            0x160 => {
                state.wpabr = value;
                After::Nothing
            }
            0x180 => {
                state.wccr = value;
                After::Nothing
            }
            0x188 => {
                state.wtcr = value;
                After::Nothing
            }
            0x190 => {
                state.wir = value;
                After::Nothing
            }
            0x1a0 => {
                state.wabr = value;
                After::Nothing
            }
            0x200 => {
                state.hlcr = value;
                After::Nothing
            }
            _ => After::Nothing,
        }
    }

    /// Begin an indirect transaction, if the mode calls for one.
    fn trigger(&self, state: &mut State, address: u32) -> After {
        if state.cr & CR_EN == 0 || state.open {
            return After::Nothing;
        }
        let cmd = state.read_command();
        match state.fmode() {
            FMODE_POLL => After::Poll { cmd, address },
            mode @ (FMODE_READ | FMODE_WRITE) => {
                let writing = mode == FMODE_WRITE;
                if !cmd.has_data {
                    // A command with no data phase — `06h` write enable, `20h`
                    // sector erase — is complete the moment its header has
                    // been clocked.
                    state.sr |= SR_TCF;
                    return After::Start {
                        cmd,
                        address,
                        writing,
                    };
                }
                // `DLR` holds the length less one (ST's HAL writes
                // `DLR = n - 1` and reads back `n = DLR + 1`).
                state.remaining = u64::from(state.dlr) + 1;
                state.open = true;
                state.writing = writing;
                After::Start {
                    cmd,
                    address,
                    writing,
                }
            }
            // Memory-mapped: nothing indirect happens. AN5050 §3.3.3 says
            // `DLR` has no meaning in this mode, and `AR` is not the address —
            // the bus transaction is.
            _ => After::Nothing,
        }
    }
}

impl MemOps for RegisterBlock {
    fn read(&self, offset: u64, dst: &mut [u8], attrs: MemAttrs) -> MemResult {
        if !matches!(dst.len(), 1 | 2 | 4) {
            return Err(BusError::BadAccess);
        }
        let register = offset & !3;
        let within = offset - register;
        if within + dst.len() as u64 > 4 {
            return Err(BusError::BadAccess);
        }
        if register == 0x050 {
            let result = self.read_data(dst, attrs);
            if !attrs.debug {
                self.shared.publish_irq();
            }
            return result;
        }
        let bytes = self.read_register(register, attrs.debug).to_le_bytes();
        for (i, byte) in dst.iter_mut().enumerate() {
            *byte = bytes[within as usize + i];
        }
        Ok(())
    }

    fn write(&self, offset: u64, src: &[u8], attrs: MemAttrs) -> MemResult {
        if !matches!(src.len(), 1 | 2 | 4) {
            return Err(BusError::BadAccess);
        }
        let register = offset & !3;
        let within = offset - register;
        if within + src.len() as u64 > 4 {
            return Err(BusError::BadAccess);
        }
        if attrs.debug {
            // A debug write would start a transaction or push a byte into a
            // flash, neither of which the core can make harmless.
            return Err(BusError::BadAccess);
        }
        if register == 0x050 {
            return self.write_data(src);
        }
        // A narrow store reaches its own lane and leaves the rest alone.
        let old = self.read_register(register, true);
        let mut bytes = old.to_le_bytes();
        for (i, byte) in src.iter().enumerate() {
            bytes[within as usize + i] = *byte;
        }
        let after = self.write_register(register, u32::from_le_bytes(bytes));
        self.settle(after);
        self.shared.publish_irq();
        Ok(())
    }

    fn constraints(&self) -> AccessConstraints {
        AccessConstraints::ANY
            .with_widths(Width::U8, Width::U32)
            .with_endian(Endian::Little)
    }
}

impl RegisterBlock {
    /// Perform the outward half of a register write.
    fn settle(&self, after: After) {
        match after {
            After::Nothing => {}
            After::Start {
                cmd,
                address,
                writing,
            } => {
                self.shared.open(&cmd, address);
                let close = {
                    let state = self.shared.state.lock();
                    !state.open
                };
                if close {
                    // A header-only command: nothing more to clock, so the
                    // chip select rises here — which is where the flash
                    // commits it.
                    self.shared.close();
                }
                let _ = writing;
            }
            After::Close => self.shared.close(),
            After::Poll { cmd, address } => self.poll(&cmd, address),
        }
    }

    /// One automatic-status poll.
    ///
    /// Exactly one, deliberately. The parts on this fabric complete a program
    /// or an erase inside the frame that commits it (`dev::flash::spinor`'s
    /// module docs say why), so a status that does not match on the first read
    /// cannot come to match: repeating would be an unbounded loop that learns
    /// nothing. `PIR`, the polling interval, is stored and read back and paces
    /// nothing.
    ///
    /// `APMS` still means what it means: with it set the transaction stops on
    /// the match, so `TCF` follows `SMF`; with it clear the peripheral would
    /// keep polling, so `TCF` does not set and the driver is expected to write
    /// `ABORT` — which this model honours and which is what ends the frame.
    fn poll(&self, cmd: &Command, address: u32) {
        let (bytes, mask, expect, any, stop) = {
            let state = self.shared.state.lock();
            (
                (u64::from(state.dlr) + 1).min(4) as usize,
                state.psmkr,
                state.psmar,
                state.cr & CR_PMM != 0,
                state.cr & CR_APMS != 0,
            )
        };
        let mut buf = alloc::vec![0u8; bytes];
        self.shared.frame(cmd, address, &mut buf, false);
        let mut got = 0u32;
        for (i, byte) in buf.iter().enumerate() {
            got |= u32::from(*byte) << (8 * i);
        }
        let matched = if any {
            (got ^ expect) & mask != mask
        } else {
            got & mask == expect & mask
        };
        let mut state = self.shared.state.lock();
        if matched {
            state.sr |= SR_SMF;
            if stop {
                state.sr |= SR_TCF;
            }
        }
    }

    /// Read from `DR`, popping bytes out of the open transaction.
    fn read_data(&self, dst: &mut [u8], attrs: MemAttrs) -> MemResult {
        if attrs.debug {
            // Popping a byte out of a flash is exactly the side effect
            // `MemAttrs::debug` forbids.
            dst.fill(0);
            return Ok(());
        }
        let mut close = false;
        for byte in dst.iter_mut() {
            let take = {
                let mut state = self.shared.state.lock();
                if !state.open || state.writing || state.remaining == 0 {
                    false
                } else {
                    state.remaining -= 1;
                    if state.remaining == 0 {
                        state.open = false;
                        state.sr |= SR_TCF;
                        close = true;
                    }
                    true
                }
            };
            *byte = if take { self.shared.byte(IDLE_BYTE) } else { 0 };
        }
        if close {
            self.shared.close();
        }
        Ok(())
    }

    /// Write to `DR`, pushing bytes into the open transaction.
    fn write_data(&self, src: &[u8]) -> MemResult {
        let mut close = false;
        for byte in src {
            let push = {
                let mut state = self.shared.state.lock();
                if !state.open || !state.writing || state.remaining == 0 {
                    false
                } else {
                    state.remaining -= 1;
                    if state.remaining == 0 {
                        state.open = false;
                        state.sr |= SR_TCF;
                        close = true;
                    }
                    true
                }
            };
            if push {
                self.shared.byte(*byte);
            }
        }
        if close {
            self.shared.close();
        }
        Ok(())
    }
}

// ---------------------------------------------------------------------------
// the memory-mapped window
// ---------------------------------------------------------------------------

/// The aperture the flash appears in.
struct MappedWindow {
    shared: Arc<Shared>,
}

impl fmt::Debug for MappedWindow {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("MappedWindow").finish_non_exhaustive()
    }
}

impl MappedWindow {
    /// Whether the peripheral is in a position to answer, and the command to
    /// answer with.
    fn armed(&self, writing: bool) -> Option<Command> {
        let state = self.shared.state.lock();
        if state.cr & CR_EN == 0 || state.fmode() != FMODE_MAPPED || state.open {
            return None;
        }
        Some(if writing {
            state.write_command()
        } else {
            state.read_command()
        })
    }

    /// Whether `offset .. offset + len` is inside the memory `DEVSIZE`
    /// describes.
    fn in_device(&self, offset: u64, len: u64) -> bool {
        let state = self.shared.state.lock();
        offset.saturating_add(len) <= state.device_bytes()
    }

    fn fault(&self) -> BusError {
        self.shared.state.lock().sr |= SR_TEF;
        BusError::BadAccess
    }
}

impl MemOps for MappedWindow {
    fn read(&self, offset: u64, dst: &mut [u8], attrs: MemAttrs) -> MemResult {
        if attrs.debug {
            // See the module docs: reaching the flash means clocking a frame,
            // which moves another device's state machine. There is no
            // side-effect-free route through a bus, so the honest answer is to
            // refuse rather than to quietly disturb the guest.
            return Err(BusError::BadAccess);
        }
        let Some(cmd) = self.armed(false) else {
            // A window nothing has configured is a window that does not
            // decode: AN5050's peripheral answers only while `FMODE = 11`.
            return Err(BusError::Unassigned);
        };
        if !self.in_device(offset, dst.len() as u64) {
            // ST's own errata put it plainly: an access at or above
            // `2^(DEVSIZE+1)` "should get an error response".
            return Err(self.fault());
        }
        let Ok(address) = u32::try_from(offset) else {
            return Err(self.fault());
        };
        self.shared.frame(&cmd, address, dst, false);
        Ok(())
    }

    fn write(&self, offset: u64, src: &[u8], attrs: MemAttrs) -> MemResult {
        if attrs.debug {
            return Err(BusError::BadAccess);
        }
        let Some(cmd) = self.armed(true) else {
            return Err(BusError::Unassigned);
        };
        if !self.in_device(offset, src.len() as u64) {
            return Err(self.fault());
        }
        let Ok(address) = u32::try_from(offset) else {
            return Err(self.fault());
        };
        // The write set is a separate command, and a board that never
        // programmed `WCCR` gets a frame with no data phase — which is what
        // the silicon does with an unconfigured write path.
        let mut buf = src.to_vec();
        self.shared.frame(&cmd, address, &mut buf, true);
        Ok(())
    }

    fn constraints(&self) -> AccessConstraints {
        // Firmware executes out of this window and copies megabytes through
        // it, and both arrive as whatever width the core felt like.
        AccessConstraints::ANY
            .with_widths(Width::U8, Width::U64)
            .with_endian(Endian::Little)
    }
}

// ---------------------------------------------------------------------------
// Device
// ---------------------------------------------------------------------------

impl Device for Octospi {
    fn class(&self) -> &'static DeviceClass {
        &CLASS
    }

    fn realize(&self, _ctx: &mut RealizeCtx<'_>) -> Result<()> {
        // Nothing outward: `map` statements place both regions.
        Ok(())
    }

    fn connect(&self, port: &str, source: WireSource) -> Result<()> {
        if port != "irq" {
            return Err(Error::Config {
                at: String::from(port),
                message: String::from("an OCTOSPI drives only `irq`"),
            });
        }
        *self.shared.irq.lock() = Some(source);
        self.shared.publish_irq();
        Ok(())
    }

    fn announce(&self, _port: &str) {
        self.shared.publish_irq();
    }

    fn reset(&self, _kind: ResetKind) {
        let was_open = {
            let mut state = self.shared.state.lock();
            let open = state.open;
            *state = State::default();
            open
        };
        if was_open {
            // A reset with a frame open must release the chip select, or the
            // part on the other end stays selected for ever.
            self.shared.close();
        }
        self.shared.publish_irq();
    }

    fn save(&self, w: &mut ChunkWriter<'_>) -> Result<()> {
        let state = *self.shared.state.lock();
        for value in [
            state.cr,
            state.dcr1,
            state.dcr2,
            state.dcr3,
            state.dcr4,
            state.sr,
            state.dlr,
            state.ar,
            state.psmkr,
            state.psmar,
            state.pir,
            state.ccr,
            state.tcr,
            state.ir,
            state.abr,
            state.lptr,
            state.wpccr,
            state.wptcr,
            state.wpir,
            state.wpabr,
            state.wccr,
            state.wtcr,
            state.wir,
            state.wabr,
            state.hlcr,
        ] {
            w.write_u32(value)?;
        }
        // The open transaction. A snapshot taken between an `AR` write and the
        // last `DR` read has the flash selected and part-way through a frame;
        // restoring it as idle would strand the part.
        w.write_bool(state.open)?;
        w.write_bool(state.writing)?;
        w.write_u64(state.remaining)
    }

    fn load(&self, r: &mut ChunkReader<'_>) -> Result<()> {
        let mut values = [0u32; 25];
        for value in &mut values {
            *value = r.read_u32()?;
        }
        let open = r.read_bool()?;
        let writing = r.read_bool()?;
        let remaining = r.read_u64()?;
        let state = State {
            cr: values[0],
            dcr1: values[1],
            dcr2: values[2],
            dcr3: values[3],
            dcr4: values[4],
            sr: values[5],
            dlr: values[6],
            ar: values[7],
            psmkr: values[8],
            psmar: values[9],
            pir: values[10],
            ccr: values[11],
            tcr: values[12],
            ir: values[13],
            abr: values[14],
            lptr: values[15],
            wpccr: values[16],
            wptcr: values[17],
            wpir: values[18],
            wpabr: values[19],
            wccr: values[20],
            wtcr: values[21],
            wir: values[22],
            wabr: values[23],
            hlcr: values[24],
            open,
            writing,
            // Bounded by the snapshot's own length field, so a corrupt one
            // cannot become four billion outstanding bytes.
            remaining: if open {
                remaining.min(u64::from(values[6]) + 1)
            } else {
                0
            },
        };
        let was_open = {
            let mut slot = self.shared.state.lock();
            let was = slot.open;
            *slot = state;
            was
        };
        if was_open && !state.open {
            self.shared.close();
        }
        self.shared.publish_irq();
        Ok(())
    }

    fn region(&self, name: &str) -> Option<RegionRef> {
        match name {
            "" | "regs" => Some(Arc::clone(&self.regs)),
            "mem" | "flash" => Some(Arc::clone(&self.window)),
            _ => None,
        }
    }
}

impl Instance for Octospi {}

/// The `stm32.octospi` device class.
pub static CLASS: DeviceClass = DeviceClass {
    name: CLASS_NAME,
    version: STATE_VERSION,
    summary: "STM32 OCTOSPI: indirect read and write, automatic status polling, and a \
              memory-mapped window a CPU can execute out of",
    properties: &[
        PropertySpec {
            name: "link",
            kind: ValueKind::Str,
            required: true,
            summary: "`transactional`; required so the machine file states the choice",
        },
        PropertySpec {
            name: "bus",
            kind: ValueKind::Str,
            required: true,
            summary: "the named SPI bus the external memory sits on",
        },
        PropertySpec {
            name: "cs",
            kind: ValueKind::Uint,
            required: false,
            summary: "which chip select the memory answers on (default 0)",
        },
        PropertySpec {
            name: "window",
            kind: ValueKind::Size,
            required: false,
            summary: "how much address space the memory-mapped aperture decodes (default 256M)",
        },
    ],
    construct: |props| Ok(Box::new(Octospi::new(props)?)),
};

/// Add [`CLASS`] to a registry.
///
/// # Errors
///
/// [`Error::Config`] if something already claimed the name.
pub fn register(registry: &mut crate::core::Registry) -> Result<()> {
    registry.add(&CLASS)
}

/// Bind [`CLASS`] into the machine graph.
///
/// # Errors
///
/// [`Error::Config`] if the class is already bound.
pub fn bind(bindings: &mut crate::machine::Bindings) -> Result<()> {
    bindings.bind(CLASS_NAME, |props| Ok(Arc::new(Octospi::new(props)?)))
}

/// What the validator should know about `stm32.octospi`.
#[must_use]
pub fn schema() -> ClassSchema {
    ClassSchema::new(CLASS_NAME)
        .prop(
            PropSchema::new("link", ValueKind::Str)
                .required()
                .values(Link::NAMES),
        )
        .prop(PropSchema::new("bus", ValueKind::Str).required())
        .prop(PropSchema::new("cs", ValueKind::Uint).range(0, MAX_CHIP_SELECTS as u64 - 1))
        .prop(PropSchema::new("window", ValueKind::Size))
        .port("irq", crate::machine::validate::PortDir::Out)
        .region("")
        .region("regs")
        .region("mem")
        .region("flash")
}

#[cfg(test)]
mod tests;
