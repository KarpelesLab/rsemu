//! An Intel 8259A programmable interrupt controller.
//!
//! # Sources
//!
//! * *Intel 8259A/8259A-2/8259A-8 Programmable Interrupt Controller* data
//!   sheet. The initialization sequence (ICW1-ICW4), the operation command
//!   words (OCW1-OCW3), the priority resolver, the end-of-interrupt commands,
//!   special mask mode, poll mode, the automatic-rotation modes and the
//!   spurious `IR7` are all from it.
//! * *IBM Personal Computer AT Technical Reference* (1984) for the wiring: two
//!   of these chips, the slave's `INT` on the master's `IR2`, and the vector
//!   bases the BIOS programs (0x08 and 0x70).
//! * The OSDev wiki's *8259 PIC* page, for the same facts restated by people
//!   who have tested them against hardware.
//!
//! No emulator source was consulted (`CLAUDE.md`, provenance).
//!
//! # The register block
//!
//! Two bytes, because the chip decodes one address line:
//!
//! ```text
//!   0  write  ICW1 when bit 4 is set;
//!             OCW2 when bits 4 and 3 are clear;
//!             OCW3 when bit 4 is clear and bit 3 is set
//!      read   IRR or ISR, whichever the last OCW3 selected, or the poll byte
//!   1  write  ICW2/ICW3/ICW4 while initialization is in progress, else OCW1
//!      read   IMR
//! ```
//!
//! One write port carrying three different words, told apart by two bits of
//! the data, is the part that surprises people. It is also why a driver that
//! writes OCW2 with bit 4 accidentally set silently re-initializes the chip.
//!
//! # The acknowledge cycle, which is the point of the chip
//!
//! `INT` is a level: it says "something is pending". The vector is *not* on
//! that wire. When the CPU takes the interrupt it runs two `INTA` pulses, and
//! the controller answers with `vector_base | level` — that is
//! [`IntAck::acknowledge`], and it is where a request moves from *requested*
//! (IRR) to *in service* (ISR).
//!
//! If nothing is pending when the acknowledge arrives, the chip answers with
//! the **spurious** vector `base | 7` and sets no ISR bit. That is not a
//! defensive default: a request can genuinely disappear between the CPU
//! sampling `INT` and acknowledging it — a short noise pulse on an
//! edge-triggered input, or a device that withdrew a level — and the data
//! sheet defines the answer. It is why every PC interrupt handler checks
//! whether IRQ7 is really in service before sending an EOI.
//!
//! # Cascading
//!
//! A slave's `INT` output drives one of the master's `IR` inputs, so the two
//! pins are one net. On the second `INTA` pulse the master, seeing that the
//! winning level is one of the cascade levels in its ICW3, puts the slave's ID
//! on the cascade bus and *the slave* drives the vector. Here that is a
//! delegation: [`Device::attach_int_ack`] gives the master a weak handle on the
//! slave's handler, and [`IntAck::acknowledge`] forwards to it while still
//! setting the master's own ISR bit for the cascade level. Which is exactly
//! why a handler for a slave interrupt must send an EOI to *both* chips.
//!
//! # The ELCR, which is not part of the chip
//!
//! A second one-byte region, [`region("elcr")`](Device::region), carries the
//! chipset's edge/level control register — 0x4d0 for the master, 0x4d1 for the
//! slave. It is a per-line override of ICW1's LTIM, and it exists because LTIM
//! is one bit for all eight inputs while a machine with PCI needs the timer
//! edge-triggered and a shared PCI line level-triggered at the same time. See
//! `Elcr`. Firmware that configures PCI interrupts programs it, and a chip
//! that ignored it would mis-trigger every shared interrupt afterwards.

use alloc::boxed::Box;
use alloc::format;
use alloc::string::{String, ToString};
use alloc::sync::{Arc, Weak};
use alloc::vec::Vec;

use crate::core::device::{Device, DeviceClass, PropertySpec, RealizeCtx, ResetKind, SinkPin};
use crate::core::error::{BusError, Error, Result};
use crate::core::props::{Props, ValueKind};
use crate::core::space::{AccessConstraints, MemAttrs, MemOps, MemResult, Region, RegionRef};
use crate::core::state::{ChunkReader, ChunkWriter, Sink, Source};
use crate::core::sync::{LockRank, Mutex};
use crate::core::value::{Endian, Width};
use crate::core::wire::{FanIn, IntAck, Level, Resolve, WireId, WireSink, WireSource};
use crate::machine::realize::Instance;
use crate::machine::validate::ClassSchema;

/// The class name a machine description writes.
pub const CLASS_NAME: &str = "pc.pic";

/// The snapshot chunk version. Bump with the encoding, never on its own.
const STATE_VERSION: u32 = 1;

/// How much address space the register block answers.
///
/// One address line, two ports. The PC/AT decodes the master at 0x20-0x21 and
/// the slave at 0xa0-0xa1, but that is the board's business, not the chip's.
pub const REGISTER_WINDOW_LEN: u64 = 2;

/// How much address space the edge/level control register answers.
///
/// One byte, mapped at 0x4d0 for the master and 0x4d1 for the slave.
pub const ELCR_WINDOW_LEN: u64 = 1;

/// How many interrupt request inputs the part has.
pub const INPUTS: u8 = 8;

/// The level a chip with nothing pending reports on an acknowledge.
///
/// The lowest priority input, so a spurious interrupt costs the least: the
/// handler that gets it is the one least likely to be doing something urgent.
const SPURIOUS_LEVEL: u8 = 7;

// -- ICW1 (port 0, bit 4 set) -----------------------------------------------

/// The bit that makes a write to port 0 an ICW1 rather than an OCW2 or OCW3.
const ICW1_INIT: u8 = 0x10;
/// IC4: an ICW4 will follow. Clear means every ICW4 option reads as zero.
const ICW1_IC4: u8 = 0x01;
/// SNGL: one chip in the system, so no ICW3 is sent.
const ICW1_SNGL: u8 = 0x02;
/// LTIM: the IR inputs are level-triggered rather than edge-triggered.
const ICW1_LTIM: u8 = 0x08;

// -- ICW4 -------------------------------------------------------------------

/// AEOI: the in-service bit is reset at the end of the acknowledge cycle.
const ICW4_AEOI: u8 = 0x02;
/// SFNM: special fully nested mode, for a master with cascaded slaves.
const ICW4_SFNM: u8 = 0x10;

// -- OCW3 (port 0, bit 4 clear, bit 3 set) ----------------------------------

/// The bit that makes a write to port 0 an OCW3 rather than an OCW2.
const OCW3_SELECT: u8 = 0x08;
/// RIS: with RR set, read ISR instead of IRR.
const OCW3_RIS: u8 = 0x01;
/// RR: this word carries a read-register command.
const OCW3_RR: u8 = 0x02;
/// P: the next read of port 0 is a poll.
const OCW3_POLL: u8 = 0x04;
/// SMM: the special mask mode bit, meaningful only with ESMM set.
const OCW3_SMM: u8 = 0x20;
/// ESMM: this word carries a special-mask-mode command.
const OCW3_ESMM: u8 = 0x40;

// -- OCW2 command codes, from its top three bits ----------------------------

/// Clear the rotate-in-automatic-EOI flag.
const OCW2_CLEAR_ROTATE_AEOI: u8 = 0b000;
/// Non-specific EOI: clear the highest-priority in-service bit.
const OCW2_EOI: u8 = 0b001;
/// No operation.
const OCW2_NOP: u8 = 0b010;
/// Specific EOI: clear the in-service bit named in bits 0-2.
const OCW2_SPECIFIC_EOI: u8 = 0b011;
/// Set the rotate-in-automatic-EOI flag.
const OCW2_SET_ROTATE_AEOI: u8 = 0b100;
/// Non-specific EOI, then make the level just cleared the lowest priority.
const OCW2_ROTATE_EOI: u8 = 0b101;
/// Make the level named in bits 0-2 the lowest priority. No EOI.
const OCW2_SET_PRIORITY: u8 = 0b110;
/// Specific EOI, then make that level the lowest priority.
const OCW2_ROTATE_SPECIFIC_EOI: u8 = 0b111;

/// The poll byte's "an interrupt is pending" bit.
const POLL_PENDING: u8 = 0x80;

/// Which chip of a cascade this one is.
///
/// On real silicon the `SP/EN` pin decides — held high for a master, low for a
/// slave — and in buffered mode the same pin becomes an output, at which point
/// ICW4's M/S bit decides instead. Neither is a thing a machine description
/// can wire, so it is a property. It selects the meaning of ICW3 and nothing
/// else.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Mode {
    /// ICW3 is a bitmap of the inputs a slave is cascaded onto.
    Master,
    /// ICW3 is this chip's own three-bit cascade ID.
    Slave,
    /// The only chip in the system: no ICW3 is sent at all.
    Single,
}

impl Mode {
    /// Parse the `mode` property.
    fn parse(text: &str) -> Mode {
        match text {
            "slave" => Mode::Slave,
            "single" => Mode::Single,
            _ => Mode::Master,
        }
    }

    /// The property spelling, for `Debug` and diagnostics.
    fn as_str(self) -> &'static str {
        match self {
            Mode::Master => "master",
            Mode::Slave => "slave",
            Mode::Single => "single",
        }
    }
}

/// Everything the guest can see or change.
///
/// The default is all zeros, which is *not* a documented power-on state: the
/// data sheet says nothing about the chip before ICW1, because in a real
/// machine nothing reads it before the BIOS initializes it. Zero is the
/// reproducible choice, and determinism is a first-class mode.
#[derive(Debug, Default, Clone, PartialEq, Eq)]
struct State {
    /// Interrupt request register: what has been requested.
    irr: u8,
    /// In-service register: what a handler is currently running for.
    isr: u8,
    /// Interrupt mask register, from OCW1. A set bit inhibits that level.
    imr: u8,
    /// ICW2, with the low three bits forced clear: the vector is `base | level`.
    vector_base: u8,
    /// ICW3. A cascade bitmap on a master, this chip's ID on a slave.
    icw3: u8,
    /// Where the initialization sequence has got to: 0 = not initializing,
    /// 1 = ICW2 expected, 2 = ICW3 expected, 3 = ICW4 expected.
    init_step: u8,
    /// ICW1's IC4: whether an ICW4 closes the sequence.
    expect_icw4: bool,
    /// ICW1's SNGL: no ICW3 is sent.
    single: bool,
    /// ICW1's LTIM: the inputs are level-triggered.
    level_triggered: bool,
    /// ICW4's AEOI: the acknowledge cycle sets no in-service bit.
    auto_eoi: bool,
    /// ICW4's SFNM: a request at a level already in service is let through, so
    /// a higher-priority interrupt inside a cascaded slave still gets out.
    sfnm: bool,
    /// The level that currently has the highest priority. Rotation moves it.
    priority_base: u8,
    /// OCW3's RIS: a read of port 0 returns ISR rather than IRR.
    read_isr: bool,
    /// OCW3's special mask mode: an in-service bit no longer inhibits lower
    /// levels, so a handler can unmask below itself by masking its own level.
    special_mask: bool,
    /// An OCW3 poll command is armed and the next read of port 0 answers it.
    poll_armed: bool,
    /// OCW2's rotate-in-automatic-EOI flag.
    rotate_on_aeoi: bool,
    /// What each IR pin is doing, which is not the same as IRR: an
    /// edge-triggered input latches into IRR and stays there, while the pin
    /// may already have gone away. Kept so a rising edge can be recognized as
    /// one, and saved for the same reason.
    pin_level: u8,
    /// The edge/level control register: a set bit makes that line
    /// level-triggered. **Not part of the 8259A** — see `Elcr`.
    elcr: u8,
    /// Whether the ELCR has ever been written.
    ///
    /// A board with no ELCR at all leaves the trigger mode to ICW1's LTIM, and
    /// a chip that came up on such a board must keep behaving that way rather
    /// than silently reading an all-zero latch as "every line edge-triggered".
    /// So the latch only takes over once firmware has actually programmed it.
    elcr_written: bool,
}

impl State {
    /// Whether `line` is level-triggered.
    ///
    /// The ELCR overrides LTIM per line once it has been programmed, which is
    /// the whole reason the chipset has one: an AT wants IRQ0 and IRQ1 edge
    /// triggered and a shared PCI line level triggered, and LTIM is one bit
    /// for the whole chip.
    fn line_is_level(&self, line: u8) -> bool {
        if self.elcr_written {
            self.elcr & (1 << line) != 0
        } else {
            self.level_triggered
        }
    }

    /// The level at priority position `index`.
    fn level_at(&self, index: u8) -> u8 {
        self.priority_base.wrapping_add(index) & 7
    }

    /// The priority position of the highest-priority in-service level, or 8
    /// when nothing is in service.
    fn in_service_priority(&self) -> u8 {
        (0..INPUTS)
            .find(|i| self.isr & (1 << self.level_at(*i)) != 0)
            .unwrap_or(INPUTS)
    }

    /// The level the priority resolver would offer the CPU, if any.
    ///
    /// The highest-priority unmasked request that outranks everything already
    /// in service. Two modes bend that:
    ///
    /// * special mask mode drops the in-service test entirely — the mask is
    ///   then the *only* thing that inhibits a level, which is what lets a
    ///   handler enable interrupts below its own;
    /// * SFNM lets a request at the level already in service through, because
    ///   on a master that level is a whole slave and a higher-priority request
    ///   inside it must not be held up by the one being serviced.
    fn resolve(&self) -> Option<u8> {
        let candidates = self.irr & !self.imr;
        if candidates == 0 {
            return None;
        }
        let blocked_at = if self.special_mask {
            INPUTS
        } else {
            self.in_service_priority()
        };
        for i in 0..INPUTS {
            let level = self.level_at(i);
            if candidates & (1 << level) == 0 {
                continue;
            }
            // The first candidate found is the highest priority one. If it is
            // blocked, so is everything below it, so there is no point looking
            // further.
            return if i < blocked_at || (i == blocked_at && self.sfnm) {
                Some(level)
            } else {
                None
            };
        }
        None
    }

    /// Clear the highest-priority in-service bit, and say which it was.
    ///
    /// This is the non-specific EOI: the chip resolves "the interrupt that was
    /// just being handled" from priority alone, which is correct precisely
    /// because a handler cannot be interrupted by anything of lower priority.
    fn nonspecific_eoi(&mut self) -> Option<u8> {
        let index = self.in_service_priority();
        if index >= INPUTS {
            return None;
        }
        let level = self.level_at(index);
        self.isr &= !(1 << level);
        Some(level)
    }

    /// Make `level` the lowest priority, so the one after it becomes highest.
    fn rotate_to(&mut self, level: u8) {
        self.priority_base = level.wrapping_add(1) & 7;
    }

    /// Apply ICW1: the chip's own reset.
    fn begin_init(&mut self, icw1: u8) {
        self.expect_icw4 = icw1 & ICW1_IC4 != 0;
        self.single = icw1 & ICW1_SNGL != 0;
        self.level_triggered = icw1 & ICW1_LTIM != 0;
        self.imr = 0;
        self.isr = 0;
        self.read_isr = false;
        self.special_mask = false;
        self.poll_armed = false;
        self.priority_base = 0;
        self.rotate_on_aeoi = false;
        // ICW1 clears every ICW4 option; if IC4 is set they are programmed
        // again in a moment, and if it is clear the data sheet says they read
        // as zero (non-buffered, no auto-EOI).
        self.auto_eoi = false;
        self.sfnm = false;
        // "The edge sense circuit is reset, meaning that following
        // initialization an interrupt request input must make a low-to-high
        // transition to generate an interrupt" — so the latched requests go,
        // but the *remembered pin levels* stay: a line that is already high
        // and stays high produces no new edge. In level mode there is no edge
        // to speak of and IRR simply follows the pins.
        //
        // The ELCR is left alone on purpose: it belongs to the chipset, not to
        // the 8259A, and an ICW1 does not reach across the board to clear it.
        self.irr = self.pin_level & self.level_mask();
        self.init_step = 1;
    }

    /// Which lines are currently level-triggered, as a bitmap.
    fn level_mask(&self) -> u8 {
        let mut mask = 0u8;
        for line in 0..INPUTS {
            if self.line_is_level(line) {
                mask |= 1 << line;
            }
        }
        mask
    }

    /// Program the ELCR and re-derive IRR for the lines that changed sense.
    ///
    /// A line that has just become level-triggered stops having a latch, so its
    /// request is whatever the pin is doing right now — which may mean dropping
    /// a request latched while it was an edge input.
    fn write_elcr(&mut self, value: u8) {
        self.elcr = value;
        self.elcr_written = true;
        let level = self.level_mask();
        self.irr = (self.irr & !level) | (self.pin_level & level);
    }

    /// Advance the initialization sequence by one control word.
    fn init_word(&mut self, value: u8) {
        match self.init_step {
            1 => {
                // ICW2. The low three bits are supplied by the level, so a
                // base of 0x08 and IR3 is vector 0x0b.
                self.vector_base = value & 0xf8;
                self.init_step = if !self.single {
                    2
                } else if self.expect_icw4 {
                    3
                } else {
                    0
                };
            }
            2 => {
                self.icw3 = value;
                self.init_step = if self.expect_icw4 { 3 } else { 0 };
            }
            _ => {
                // ICW4. The buffered-mode bits (BUF, M/S) are accepted and
                // ignored: they configure `SP/EN` as an output that enables a
                // bus transceiver, and this model has no transceiver — the
                // master/slave role is the `mode` property. µPM (bit 0) is
                // likewise ignored, because 8080/8085 mode changes how the
                // vector is formed and no PC ever selects it.
                self.auto_eoi = value & ICW4_AEOI != 0;
                self.sfnm = value & ICW4_SFNM != 0;
                self.init_step = 0;
            }
        }
    }
}

/// The register block, as something an address space can dispatch to, plus the
/// pins that hang off it.
///
/// The device owns this `Arc` and hands out weak references, which is what both
/// [`Device::sink`] and [`Device::int_ack`] require.
#[derive(Debug)]
struct Registers {
    state: Mutex<State>,
    /// The `INT` output, at [`LockRank::LEAF`] so the line can be driven with
    /// nothing else held. Driving it re-enters whatever is listening — a CPU,
    /// or another 8259A — so holding the state lock across it would be the
    /// re-entrancy bug this chip exists to demonstrate.
    out: Mutex<Option<WireSource>>,
    /// Per input line, what answers an acknowledge that lands on it: a slave's
    /// handler, weakly held because the machine owns both devices.
    acks: Mutex<[Option<Weak<dyn IntAck>>; INPUTS as usize]>,
    mode: Mode,
}

/// The edge/level control register, as a region of its own.
///
/// The ELCR is **not** part of the 8259A. It is a chipset latch that sits
/// between the board's interrupt lines and the controller's IR pins, one byte
/// per chip, and each bit overrides ICW1's LTIM for one line. It exists because
/// LTIM is a single bit for the whole chip and a PCI machine needs IRQ0 and
/// IRQ1 edge-triggered while the shared PCI lines are level-triggered.
///
/// It lives on this device rather than on a chipset device of its own because
/// it is per-controller state that only the controller can act on, and a
/// separate device would have to reach into this one's IRR to do it. A machine
/// file maps it at 0x4d0 for the master and 0x4d1 for the slave.
#[derive(Debug)]
struct Elcr {
    regs: Arc<Registers>,
}

impl MemOps for Elcr {
    fn read(&self, _offset: u64, dst: &mut [u8], _attrs: MemAttrs) -> MemResult {
        let [byte] = dst else {
            return Err(BusError::BadAccess);
        };
        // A pure latch read: no side effect, so a debug read needs no special
        // case and gets the same answer the guest would.
        *byte = self.regs.state.lock().elcr;
        Ok(())
    }

    fn write(&self, _offset: u64, src: &[u8], attrs: MemAttrs) -> MemResult {
        let [value] = src else {
            return Err(BusError::BadAccess);
        };
        if attrs.debug {
            // Changing a line's trigger mode changes whether a still-asserted
            // device re-interrupts, which is not a harmless thing for a
            // debugger to do behind the guest's back.
            return Err(BusError::BadAccess);
        }
        self.regs.write_elcr(*value);
        Ok(())
    }

    fn constraints(&self) -> AccessConstraints {
        AccessConstraints::word(Width::U8, Endian::Little)
    }
}

/// One IR input pin.
///
/// The [`FanIn`] is why this is a separate object per line: several devices may
/// wire-OR onto one IR line — on a PC/AT, IRQ5 is a whole ISA slot's worth of
/// cards — and a pin told "low" must know whether some *other* driver is still
/// asserting before it withdraws the request.
#[derive(Debug)]
struct InputPin {
    regs: Arc<Registers>,
    line: u8,
    inputs: FanIn,
}

impl WireSink for InputPin {
    fn set_level(&self, src: WireId, _line: u32, level: Level) {
        self.inputs.set(src, level);
        let high = self.inputs.resolve(Resolve::Or).is_high();
        self.regs.set_pin(self.line, high);
        self.regs.refresh();
    }
}

/// An Intel 8259A programmable interrupt controller.
#[derive(Debug)]
pub struct Pic8259 {
    regs: Arc<Registers>,
    region: RegionRef,
    /// The chipset's edge/level latch, thirty-odd addresses away from the chip
    /// itself, so a separate region.
    elcr: RegionRef,
    /// The device's own references to its input pins. A net holds only weak
    /// ones, so something has to keep them alive.
    pins: Mutex<Vec<Arc<InputPin>>>,
}

impl Registers {
    /// Record a pin's level and turn it into a request.
    ///
    /// The whole edge/level distinction lives here. Edge-triggered inputs latch
    /// on the rising edge and are cleared by the acknowledge, so a line that
    /// stays high requests exactly once. Level-triggered inputs make IRR a
    /// mirror of the pins, so a device still asserting after its EOI is
    /// serviced again immediately — which is what shared PCI-style interrupts
    /// need and what an edge-triggered line famously loses.
    fn set_pin(&self, line: u8, high: bool) {
        let mask = 1u8 << line;
        let mut state = self.state.lock();
        let was_high = state.pin_level & mask != 0;
        if high {
            state.pin_level |= mask;
        } else {
            state.pin_level &= !mask;
        }
        if state.line_is_level(line) {
            if high {
                state.irr |= mask;
            } else {
                state.irr &= !mask;
            }
        } else if high && !was_high {
            state.irr |= mask;
        }
    }

    /// Which ELCR bits this chip's board lets software change.
    ///
    /// A board fact, not a chip one: the AT hardwires the lines whose timing
    /// the machine itself depends on. On the master those are IR0 (the 8254
    /// tick), IR1 (the keyboard) and IR2 (the cascade, which is the slave's
    /// `INT` and edge-triggered by construction); on the slave they are IR0 —
    /// IRQ8, the RTC — and IR5 — IRQ13, the coprocessor error. Those bits read
    /// back zero however they are written. A single chip belongs to no such
    /// board, so nothing is fixed.
    fn elcr_mask(&self) -> u8 {
        match self.mode {
            Mode::Master => !0b0000_0111,
            Mode::Slave => !0b0010_0001,
            Mode::Single => 0xff,
        }
    }

    /// Program the ELCR, then re-evaluate `INT`.
    fn write_elcr(&self, value: u8) {
        {
            let mut state = self.state.lock();
            let masked = value & self.elcr_mask();
            state.write_elcr(masked);
        }
        self.refresh();
    }

    /// Drive the `INT` output. Never called with the state lock held.
    fn drive(&self, asserted: bool) {
        let out = self.out.lock().clone();
        if let Some(out) = out {
            out.set(Level::from_bool(asserted));
        }
    }

    /// Recompute `INT` from the current state and drive it.
    fn refresh(&self) {
        let asserted = self.state.lock().resolve().is_some();
        self.drive(asserted);
    }

    /// Whether a winning level should be answered by a cascaded slave.
    ///
    /// Only a master interprets ICW3 as a bitmap; on a slave the same register
    /// is its own ID, and reading it as a cascade mask would make a slave try
    /// to delegate to itself.
    fn cascades(&self, state: &State, level: u8) -> bool {
        self.mode == Mode::Master && state.icw3 & (1 << level) != 0
    }

    /// Move the winning level from requested to in service, and report it.
    ///
    /// `None` means nothing was pending: the caller answers spuriously.
    fn take_request(&self, state: &mut State) -> Option<u8> {
        let level = state.resolve()?;
        if !state.line_is_level(level) {
            // The edge latch is what is being consumed. In level mode there is
            // no latch, so IRR keeps following the pin and the ISR bit is the
            // only thing holding the request off.
            state.irr &= !(1 << level);
        }
        if state.auto_eoi {
            // AEOI resets the in-service bit at the end of the second INTA
            // pulse, so it is never observably set. The mode exists for systems
            // that never nest; the rotation option makes it round-robin.
            if state.rotate_on_aeoi {
                state.rotate_to(level);
            }
        } else {
            state.isr |= 1 << level;
        }
        Some(level)
    }

    /// The read a poll command answers.
    fn poll_byte(&self, state: &mut State) -> u8 {
        state.poll_armed = false;
        match self.take_request(state) {
            // Bit 7 says there is something, bits 2-0 say what. Delegation is
            // deliberately not done here: a poll is software asking this chip
            // what it has, and software that polls a cascade polls both chips.
            Some(level) => POLL_PENDING | level,
            None => 0,
        }
    }

    /// Read one of the two ports. `debug` suppresses every side effect.
    fn read_port(&self, port: u8, debug: bool) -> u8 {
        let mut state = self.state.lock();
        match port {
            0 if state.poll_armed && !debug => self.poll_byte(&mut state),
            // A debugger reading port 0 with a poll armed gets the selected
            // register and leaves the poll for the guest, because answering it
            // would set an in-service bit nobody will ever EOI.
            0 if state.read_isr => state.isr,
            0 => state.irr,
            _ => state.imr,
        }
    }

    /// Write port 0: ICW1, OCW2 or OCW3, told apart by bits 4 and 3.
    fn write_command(&self, state: &mut State, value: u8) {
        if value & ICW1_INIT != 0 {
            state.begin_init(value);
            return;
        }
        if value & OCW3_SELECT != 0 {
            // OCW3.
            if value & OCW3_ESMM != 0 {
                state.special_mask = value & OCW3_SMM != 0;
            }
            if value & OCW3_RR != 0 {
                state.read_isr = value & OCW3_RIS != 0;
            }
            if value & OCW3_POLL != 0 {
                state.poll_armed = true;
            }
            return;
        }
        // OCW2.
        let level = value & 7;
        match value >> 5 {
            OCW2_EOI => {
                state.nonspecific_eoi();
            }
            OCW2_SPECIFIC_EOI => state.isr &= !(1 << level),
            OCW2_ROTATE_EOI => {
                if let Some(cleared) = state.nonspecific_eoi() {
                    state.rotate_to(cleared);
                }
            }
            OCW2_ROTATE_SPECIFIC_EOI => {
                state.isr &= !(1 << level);
                state.rotate_to(level);
            }
            OCW2_SET_PRIORITY => state.rotate_to(level),
            OCW2_SET_ROTATE_AEOI => state.rotate_on_aeoi = true,
            OCW2_CLEAR_ROTATE_AEOI => state.rotate_on_aeoi = false,
            // 010 is the data sheet's no-op, and everything else is one of the
            // codes above; `value >> 5` has no other values.
            _ => debug_assert_eq!(value >> 5, OCW2_NOP),
        }
    }

    /// Write one of the two ports.
    fn write_port(&self, port: u8, value: u8) {
        {
            let mut state = self.state.lock();
            if port == 0 {
                self.write_command(&mut state, value);
            } else if state.init_step != 0 {
                state.init_word(value);
            } else {
                // OCW1: the mask. Unmasking a level whose request is still
                // latched in IRR raises `INT` on the spot, which the refresh
                // below takes care of.
                state.imr = value;
            }
        }
        self.refresh();
    }
}

impl IntAck for Registers {
    fn acknowledge(&self) -> u32 {
        let (vector, delegate) = {
            let mut state = self.state.lock();
            match self.take_request(&mut state) {
                Some(level) => {
                    let slave = self
                        .cascades(&state, level)
                        .then(|| self.acks.lock()[level as usize].clone())
                        .flatten();
                    (u32::from(state.vector_base | level), slave)
                }
                // Nothing pending. A request can go away between the CPU
                // sampling `INT` and acknowledging it, and the data sheet's
                // answer is IR7's vector with no in-service bit set — which is
                // exactly how a handler tells a spurious IRQ7 from a real one.
                None => (u32::from(state.vector_base | SPURIOUS_LEVEL), None),
            }
        };
        // Outside the lock, because the slave's acknowledge drops its own `INT`
        // and that lands straight back on this chip's IR pin.
        let vector = match delegate.as_ref().and_then(Weak::upgrade) {
            Some(slave) => slave.acknowledge(),
            // A cascade level with no slave behind it: the master answers for
            // itself. That is a machine wired with an ICW3 bit the board does
            // not have, not something to panic over.
            None => vector,
        };
        self.refresh();
        vector
    }
}

impl MemOps for Registers {
    fn read(&self, offset: u64, dst: &mut [u8], attrs: MemAttrs) -> MemResult {
        let [byte] = dst else {
            return Err(BusError::BadAccess);
        };
        *byte = self.read_port((offset & 1) as u8, attrs.debug);
        if !attrs.debug {
            // A poll consumes a request, so `INT` may have to fall.
            self.refresh();
        }
        Ok(())
    }

    fn write(&self, offset: u64, src: &[u8], attrs: MemAttrs) -> MemResult {
        let [value] = src else {
            return Err(BusError::BadAccess);
        };
        if attrs.debug {
            // There is no harmless write. Every value on either port either
            // re-initializes the chip, acknowledges an interrupt or changes
            // which levels reach the CPU (`ROADMAP.md` §15, invariant 5).
            return Err(BusError::BadAccess);
        }
        self.write_port((offset & 1) as u8, *value);
        Ok(())
    }

    fn constraints(&self) -> AccessConstraints {
        // An 8-bit part on an 8-bit bus, and the two ports are unrelated
        // registers: a 16-bit access would invent an order between an ICW1 and
        // the ICW2 that must follow it.
        AccessConstraints::word(Width::U8, Endian::Little)
    }
}

impl Pic8259 {
    /// Validate `props` and build the device.
    ///
    /// # Errors
    ///
    /// [`Error::Property`] if `mode` is not one
    /// of `master`, `slave` or `single`, or if a property this class does not
    /// know was given.
    pub fn new(props: &Props) -> Result<Pic8259> {
        let mut r = props.reader();
        let mode = Mode::parse(r.or_enum("mode", "master", &["master", "slave", "single"])?);
        r.finish()?;
        Ok(Pic8259::with_mode(mode))
    }

    /// One in the default configuration: a master, uninitialized.
    #[must_use]
    pub fn default_device() -> Pic8259 {
        Pic8259::with_mode(Mode::Master)
    }

    fn with_mode(mode: Mode) -> Pic8259 {
        let regs = Arc::new(Registers {
            state: Mutex::with_rank(LockRank::DEVICE, State::default()),
            out: Mutex::with_rank(LockRank::LEAF, None),
            acks: Mutex::with_rank(LockRank::LEAF, [const { None }; INPUTS as usize]),
            mode,
        });
        let region: RegionRef = Arc::new(Region::io(
            CLASS_NAME,
            REGISTER_WINDOW_LEN,
            Arc::clone(&regs) as Arc<dyn MemOps>,
        ));
        let elcr: RegionRef = Arc::new(Region::io(
            "pc.pic.elcr",
            ELCR_WINDOW_LEN,
            Arc::new(Elcr {
                regs: Arc::clone(&regs),
            }) as Arc<dyn MemOps>,
        ));
        Pic8259 {
            regs,
            region,
            elcr,
            pins: Mutex::with_rank(LockRank::LEAF, Vec::new()),
        }
    }

    /// Which chip of a cascade this is, as the property spells it.
    #[must_use]
    pub fn mode(&self) -> &'static str {
        self.regs.mode.as_str()
    }

    /// Whether `INT` is currently asserted.
    #[must_use]
    pub fn int_asserted(&self) -> bool {
        self.regs.state.lock().resolve().is_some()
    }

    /// The in-service register, for tests and the monitor.
    #[must_use]
    pub fn in_service(&self) -> u8 {
        self.regs.state.lock().isr
    }

    /// The interrupt request register, for tests and the monitor.
    #[must_use]
    pub fn requested(&self) -> u8 {
        self.regs.state.lock().irr
    }

    /// The input pin number `port` names, if it names one.
    fn pin_number(port: &str) -> Option<u8> {
        let level: u8 = port.strip_prefix("ir")?.parse().ok()?;
        (level < INPUTS).then_some(level)
    }
}

/// The `pc.pic` device class.
pub static CLASS: DeviceClass = DeviceClass {
    name: CLASS_NAME,
    version: STATE_VERSION,
    summary: "Intel 8259A programmable interrupt controller",
    properties: &[PropertySpec {
        name: "mode",
        kind: ValueKind::Str,
        required: false,
        summary: "which chip of a cascade this is: master, slave or single (default master)",
    }],
    construct: |props| Ok(Box::new(Pic8259::new(props)?)),
};

impl Device for Pic8259 {
    fn class(&self) -> &'static DeviceClass {
        &CLASS
    }

    fn realize(&self, _ctx: &mut RealizeCtx<'_>) -> Result<()> {
        Ok(())
    }

    fn reset(&self, _kind: ResetKind) {
        {
            let mut state = self.regs.state.lock();
            *state = State::default();
        }
        self.regs.drive(false);
    }

    fn region(&self, name: &str) -> Option<RegionRef> {
        match name {
            "" | "regs" => Some(Arc::clone(&self.region)),
            "elcr" => Some(Arc::clone(&self.elcr)),
            _ => None,
        }
    }

    fn sink(&self, port: &str, sources: &[WireId]) -> Option<SinkPin> {
        let line = Pic8259::pin_number(port)?;
        // The fan-in can only be built now: it is told its sources at
        // construction and no `WireId` existed when this chip was made.
        let pin = Arc::new(InputPin {
            regs: Arc::clone(&self.regs),
            line,
            inputs: FanIn::new(sources),
        });
        self.pins.lock().push(Arc::clone(&pin));
        Some(SinkPin {
            sink: pin,
            line: u32::from(line),
        })
    }

    fn connect(&self, port: &str, source: WireSource) -> Result<()> {
        if port != "int" {
            return Err(Error::Config {
                at: port.to_string(),
                message: String::from("an 8259A drives one pin, `int`"),
            });
        }
        *self.regs.out.lock() = Some(source);
        Ok(())
    }

    fn attach_int_ack(&self, port: &str, ack: Weak<dyn IntAck>) {
        // Kept per line, because which pin a slave hangs off is exactly what
        // ICW3 names. Anything that is not an IR pin has nothing to answer.
        if let Some(line) = Pic8259::pin_number(port) {
            self.regs.acks.lock()[line as usize] = Some(ack);
        }
    }

    fn int_ack(&self, port: &str) -> Option<Arc<dyn IntAck>> {
        // The device owns this `Arc`; the net gets a `Weak`, so building one
        // here would hand out a reference that is already dead.
        (port == "int").then(|| Arc::clone(&self.regs) as Arc<dyn IntAck>)
    }

    fn announce(&self, port: &str) {
        if port == "int" {
            self.regs.refresh();
        }
    }

    fn save(&self, w: &mut ChunkWriter<'_>) -> Result<()> {
        let state = self.regs.state.lock();
        for byte in [
            state.irr,
            state.isr,
            state.imr,
            state.vector_base,
            state.icw3,
            state.init_step,
            state.priority_base,
            state.pin_level,
            state.elcr,
        ] {
            w.write_u8(byte)?;
        }
        for flag in [
            state.expect_icw4,
            state.single,
            state.level_triggered,
            state.auto_eoi,
            state.sfnm,
            state.read_isr,
            state.special_mask,
            state.poll_armed,
            state.rotate_on_aeoi,
            state.elcr_written,
        ] {
            w.write_bool(flag)?;
        }
        Ok(())
        // The attached acknowledge handlers and the `INT` source are wiring,
        // not state: the machine rebuilds them (`ROADMAP.md` §4.5).
    }

    fn load(&self, r: &mut ChunkReader<'_>) -> Result<()> {
        // A struct literal rather than a default and a run of assignments, so
        // the field order and the wire order are the same list and a field
        // added to one without the other does not compile.
        let state = State {
            irr: r.read_u8()?,
            isr: r.read_u8()?,
            imr: r.read_u8()?,
            vector_base: r.read_u8()?,
            icw3: r.read_u8()?,
            init_step: r.read_u8()?,
            priority_base: r.read_u8()?,
            pin_level: r.read_u8()?,
            elcr: r.read_u8()?,
            expect_icw4: r.read_bool()?,
            single: r.read_bool()?,
            level_triggered: r.read_bool()?,
            auto_eoi: r.read_bool()?,
            sfnm: r.read_bool()?,
            read_isr: r.read_bool()?,
            special_mask: r.read_bool()?,
            poll_armed: r.read_bool()?,
            rotate_on_aeoi: r.read_bool()?,
            elcr_written: r.read_bool()?,
        };
        // Both are indices into an eight-entry order; out of range they would
        // make the priority resolver nonsense rather than merely wrong.
        if state.init_step > 3 {
            return Err(Error::State(format!(
                "snapshot has the 8259A at initialization step {}, of at most 3",
                state.init_step
            )));
        }
        if state.priority_base >= INPUTS {
            return Err(Error::State(format!(
                "snapshot makes level {} the highest priority, of {INPUTS} inputs",
                state.priority_base
            )));
        }
        *self.regs.state.lock() = state;
        self.regs.refresh();
        Ok(())
    }
}

impl Instance for Pic8259 {}

/// Add [`CLASS`] to a registry.
///
/// # Errors
///
/// [`Error::Config`] if the name is claimed.
pub fn register(registry: &mut crate::core::Registry) -> Result<()> {
    registry.add(&CLASS)
}

/// Bind [`CLASS`] into the machine graph.
///
/// # Errors
///
/// [`Error::Config`] if the class is bound twice.
pub fn bind(bindings: &mut crate::machine::Bindings) -> Result<()> {
    bindings.bind(CLASS_NAME, |props| Ok(Arc::new(Pic8259::new(props)?)))
}

/// What the validator should know about `pc.pic`.
#[must_use]
pub fn schema() -> ClassSchema {
    use crate::machine::validate::{PortDir, PropSchema};
    let mut schema = ClassSchema::new(CLASS_NAME)
        .prop(PropSchema::new("mode", ValueKind::Str).values(&["master", "slave", "single"]))
        .region("")
        .region("regs")
        .region("elcr")
        .port("int", PortDir::Out);
    for line in 0..INPUTS {
        schema = schema.port(format!("ir{line}"), PortDir::In);
    }
    schema
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::state::{MachineShape, Migrations, StateReader, StateWriter};
    use crate::core::sync::{AtomicU32, Ordering};
    use crate::core::wire::{Wire, WireIdAllocator};

    /// A machine file's ICW1 for an edge-triggered chip that will be sent an
    /// ICW3 and an ICW4.
    const ICW1_CASCADE: u8 = ICW1_INIT | ICW1_IC4;
    /// ICW4 selecting 8086 mode and nothing else.
    const ICW4_8086: u8 = 0x01;
    /// The vector base the PC/AT BIOS gives the master.
    const MASTER_BASE: u8 = 0x08;
    /// The vector base the PC/AT BIOS gives the slave.
    const SLAVE_BASE: u8 = 0x70;

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

    impl Probe {
        fn high(&self) -> bool {
            self.level.load(Ordering::Relaxed) != 0
        }
    }

    /// A chip with all eight inputs and its `INT` wired to a probe.
    struct Bench {
        pic: Pic8259,
        ids: WireIdAllocator,
        src: WireId,
        pins: Vec<Arc<dyn WireSink>>,
        int: Arc<Wire>,
        int_id: WireId,
        probe: Arc<Probe>,
    }

    fn bench(mode: &str) -> Bench {
        let pic = Pic8259::new(&Props::new().with("mode", mode)).expect("a legal mode");
        let ids = WireIdAllocator::new();
        let src = ids.alloc();
        let pins: Vec<Arc<dyn WireSink>> = (0..INPUTS)
            .map(|line| {
                pic.sink(&format!("ir{line}"), &[src])
                    .expect("every IR pin exists")
                    .sink
            })
            .collect();
        let int_id = ids.alloc();
        let probe = Arc::new(Probe::default());
        let int = Wire::builder()
            .source(int_id)
            .sink(Arc::clone(&probe) as Arc<dyn WireSink>, 0)
            .build_shared();
        pic.connect("int", WireSource::new(Arc::clone(&int), int_id))
            .expect("an 8259A drives int");
        Bench {
            pic,
            ids,
            src,
            pins,
            int,
            int_id,
            probe,
        }
    }

    impl Bench {
        fn poke(&self, port: u64, value: u8) {
            self.pic
                .regs
                .write(port, &[value], MemAttrs::DEFAULT)
                .expect("a byte write is legal");
        }

        fn peek(&self, port: u64) -> u8 {
            let mut byte = [0u8; 1];
            self.pic
                .regs
                .read(port, &mut byte, MemAttrs::DEFAULT)
                .expect("a byte read is legal");
            byte[0]
        }

        fn peek_debug(&self, port: u64) -> u8 {
            let mut byte = [0u8; 1];
            self.pic
                .regs
                .read(port, &mut byte, MemAttrs::DEBUG)
                .expect("a debug byte read is legal");
            byte[0]
        }

        fn set(&self, line: u8, high: bool) {
            self.pins[line as usize].set_level(self.src, u32::from(line), Level::from_bool(high));
        }

        fn raise(&self, line: u8) {
            self.set(line, true);
        }

        /// The AT BIOS's initialization, with `icw1` chosen by the caller so a
        /// test can select level triggering or auto-EOI.
        fn init(&self, icw1: u8, base: u8, icw3: u8, icw4: u8) {
            self.poke(0, icw1);
            self.poke(1, base);
            if icw1 & ICW1_SNGL == 0 {
                self.poke(1, icw3);
            }
            if icw1 & ICW1_IC4 != 0 {
                self.poke(1, icw4);
            }
        }

        fn ack(&self) -> u32 {
            self.pic.regs.acknowledge()
        }

        /// The ELCR's `MemOps`. A second `Elcr` over the same `Registers` is
        /// the same register: it holds nothing of its own.
        fn elcr_ops(&self) -> Elcr {
            Elcr {
                regs: Arc::clone(&self.pic.regs),
            }
        }

        fn poke_elcr(&self, value: u8) {
            self.elcr_ops()
                .write(0, &[value], MemAttrs::DEFAULT)
                .expect("a byte write is legal");
        }

        fn peek_elcr(&self) -> u8 {
            let mut byte = [0u8; 1];
            self.elcr_ops()
                .read(0, &mut byte, MemAttrs::DEFAULT)
                .expect("a byte read is legal");
            byte[0]
        }
    }

    #[test]
    fn the_initialization_sequence_leaves_the_documented_state() {
        let b = bench("master");
        b.poke(1, 0xff); // a mask ICW1 must clear
        b.poke(0, ICW1_CASCADE | ICW1_LTIM);
        {
            let state = b.pic.regs.state.lock();
            assert_eq!(state.init_step, 1, "ICW2 is expected next");
            assert_eq!(state.imr, 0, "ICW1 clears the mask");
            assert_eq!(state.isr, 0);
            assert_eq!(state.priority_base, 0, "IR0 is highest");
            assert!(!state.read_isr, "and a read of port 0 returns IRR");
            assert!(!state.special_mask);
            assert!(state.level_triggered);
        }
        b.poke(1, MASTER_BASE | 0x03); // the low three bits are ignored
        assert_eq!(b.pic.regs.state.lock().init_step, 2);
        b.poke(1, 0x04);
        assert_eq!(b.pic.regs.state.lock().init_step, 3);
        b.poke(1, ICW4_8086 | ICW4_AEOI);

        let state = b.pic.regs.state.lock();
        assert_eq!(state.init_step, 0, "the sequence is over");
        assert_eq!(state.vector_base, MASTER_BASE);
        assert_eq!(state.icw3, 0x04);
        assert!(state.auto_eoi);
        assert!(!state.sfnm);
    }

    #[test]
    fn a_chip_told_it_is_alone_is_not_sent_an_icw3() {
        let b = bench("single");
        b.poke(0, ICW1_INIT | ICW1_SNGL | ICW1_IC4);
        b.poke(1, MASTER_BASE);
        assert_eq!(
            b.pic.regs.state.lock().init_step,
            3,
            "ICW4 comes straight after ICW2"
        );
        b.poke(1, ICW4_8086);
        assert_eq!(b.pic.regs.state.lock().init_step, 0);
    }

    #[test]
    fn a_request_asserts_int_and_the_acknowledge_returns_its_vector() {
        let b = bench("master");
        b.init(ICW1_CASCADE, MASTER_BASE, 0x04, ICW4_8086);
        assert!(!b.probe.high(), "nothing is pending yet");

        b.raise(0);
        assert!(b.probe.high());
        assert_eq!(b.peek(0), 0x01, "IRR bit 0");

        assert_eq!(b.ack(), u32::from(MASTER_BASE));
        assert_eq!(b.pic.in_service(), 0x01);
        assert!(!b.probe.high(), "and INT falls, because nothing else is up");
    }

    #[test]
    fn a_lower_priority_request_waits_for_the_end_of_interrupt() {
        let b = bench("master");
        b.init(ICW1_CASCADE, MASTER_BASE, 0x04, ICW4_8086);
        b.raise(1);
        assert_eq!(b.ack(), u32::from(MASTER_BASE | 1));

        b.raise(3);
        assert!(!b.probe.high(), "IR3 is below IR1, which is in service");
        assert_eq!(b.pic.requested() & 0x08, 0x08, "but it is still requested");

        b.poke(0, 0x20); // non-specific EOI
        assert!(b.probe.high(), "and now it gets through");
        assert_eq!(b.ack(), u32::from(MASTER_BASE | 3));
        assert_eq!(b.pic.in_service(), 0x08);
    }

    #[test]
    fn a_higher_priority_request_preempts_one_in_service() {
        let b = bench("master");
        b.init(ICW1_CASCADE, MASTER_BASE, 0x04, ICW4_8086);
        b.raise(5);
        assert_eq!(b.ack(), u32::from(MASTER_BASE | 5));

        b.raise(2);
        assert!(b.probe.high(), "IR2 outranks IR5");
        assert_eq!(b.ack(), u32::from(MASTER_BASE | 2));
        assert_eq!(b.pic.in_service(), 0x24, "both are in service, nested");

        // And unwinding is in the opposite order, because a non-specific EOI
        // always clears the highest-priority in-service bit.
        b.poke(0, 0x20);
        assert_eq!(b.pic.in_service(), 0x20);
        b.poke(0, 0x20);
        assert_eq!(b.pic.in_service(), 0x00);
    }

    #[test]
    fn rotating_on_end_of_interrupt_moves_the_level_to_the_back() {
        let b = bench("master");
        b.init(ICW1_CASCADE, MASTER_BASE, 0x04, ICW4_8086);
        b.raise(0);
        assert_eq!(b.ack(), u32::from(MASTER_BASE));
        b.poke(0, 0xa0); // rotate on non-specific EOI
        assert_eq!(
            b.pic.regs.state.lock().priority_base,
            1,
            "IR1 is now highest and IR0 lowest"
        );

        // With IR0 at the back, a simultaneous IR0 and IR4 goes to IR4.
        b.raise(0);
        b.raise(4);
        assert_eq!(b.ack(), u32::from(MASTER_BASE | 4));
    }

    #[test]
    fn a_masked_line_does_not_assert_int() {
        let b = bench("master");
        b.init(ICW1_CASCADE, MASTER_BASE, 0x04, ICW4_8086);
        b.poke(1, 0x02); // mask IR1
        assert_eq!(b.peek(1), 0x02, "OCW1 reads back");

        b.raise(1);
        assert!(!b.probe.high());
        assert_eq!(b.peek(0), 0x02, "the request is latched all the same");

        b.poke(1, 0x00);
        assert!(b.probe.high(), "unmasking releases it");
        assert_eq!(b.ack(), u32::from(MASTER_BASE | 1));
    }

    #[test]
    fn special_mask_mode_lets_a_handler_unmask_below_itself() {
        let b = bench("master");
        b.init(ICW1_CASCADE, MASTER_BASE, 0x04, ICW4_8086);
        b.raise(2);
        assert_eq!(b.ack(), u32::from(MASTER_BASE | 2));

        b.raise(6);
        assert!(!b.probe.high(), "IR2 is in service and blocks IR6");
        b.poke(0, OCW3_SELECT | OCW3_ESMM | OCW3_SMM);
        assert!(b.probe.high(), "special mask mode ignores the ISR");
        assert_eq!(b.ack(), u32::from(MASTER_BASE | 6));
    }

    #[test]
    fn acknowledging_with_nothing_pending_gives_the_spurious_vector() {
        let b = bench("master");
        b.init(ICW1_CASCADE, MASTER_BASE, 0x04, ICW4_8086);
        // The race the data sheet describes: the request went away between the
        // CPU sampling INT and acknowledging it.
        assert_eq!(b.ack(), u32::from(MASTER_BASE | SPURIOUS_LEVEL));
        assert_eq!(b.pic.in_service(), 0, "and no in-service bit is set");
        assert!(!b.probe.high());
    }

    #[test]
    fn automatic_end_of_interrupt_sets_no_in_service_bit() {
        let b = bench("master");
        b.init(ICW1_CASCADE, MASTER_BASE, 0x04, ICW4_8086 | ICW4_AEOI);
        b.raise(4);
        assert_eq!(b.ack(), u32::from(MASTER_BASE | 4));
        assert_eq!(b.pic.in_service(), 0);
        // Which is exactly why AEOI does not nest: a second request is taken
        // with nothing holding it off.
        b.raise(6);
        assert!(b.probe.high());
    }

    #[test]
    fn an_edge_triggered_line_requests_once_but_a_level_re_requests() {
        let edge = bench("master");
        edge.init(ICW1_CASCADE, MASTER_BASE, 0x04, ICW4_8086);
        edge.raise(3); // and the device keeps the line high
        assert_eq!(edge.ack(), u32::from(MASTER_BASE | 3));
        edge.poke(0, 0x20);
        assert!(
            !edge.probe.high(),
            "the latch was consumed; only a new rising edge requests again"
        );
        edge.set(3, false);
        edge.set(3, true);
        assert!(edge.probe.high());

        let level = bench("master");
        level.init(ICW1_CASCADE | ICW1_LTIM, MASTER_BASE, 0x04, ICW4_8086);
        level.raise(3);
        assert_eq!(level.ack(), u32::from(MASTER_BASE | 3));
        level.poke(0, 0x20);
        assert!(
            level.probe.high(),
            "the line is still asserted, so the request is still there"
        );
        // Until the device withdraws it, at which point IRR follows.
        level.set(3, false);
        assert!(!level.probe.high());
        assert_eq!(level.pic.requested() & 0x08, 0);
    }

    #[test]
    fn the_elcr_selects_the_trigger_mode_one_line_at_a_time() {
        let b = bench("master");
        b.init(ICW1_CASCADE, MASTER_BASE, 0x04, ICW4_8086);
        assert_eq!(b.peek_elcr(), 0, "an AT powers up with every line edge");

        // IR5 level-triggered, IR6 left edge-triggered: one byte, two senses.
        b.poke_elcr(0x20);
        assert_eq!(b.peek_elcr(), 0x20);

        b.raise(5); // and the device keeps the line asserted
        assert_eq!(b.ack(), u32::from(MASTER_BASE | 5));
        b.poke(0, 0x20);
        assert!(
            b.probe.high(),
            "a level line still asserted after EOI requests again"
        );
        assert_eq!(b.ack(), u32::from(MASTER_BASE | 5));
        b.poke(0, 0x20);

        // Clearing the bit puts the same line back to edge triggering. The
        // request it is already holding stays — the ELCR changes the input
        // path, it does not reach into the latch — but it is now a latch, so
        // once acknowledged the still-asserted device stops re-interrupting.
        b.poke_elcr(0x00);
        assert!(b.probe.high(), "the request it already held survives");
        assert_eq!(b.ack(), u32::from(MASTER_BASE | 5));
        b.poke(0, 0x20);
        assert!(
            !b.probe.high(),
            "and as an edge input it does not re-request"
        );
        b.set(5, false);
        b.set(5, true);
        assert!(b.probe.high(), "only a fresh edge gets through now");
    }

    #[test]
    fn the_lines_the_board_hardwires_read_back_as_edge() {
        // A board fact, not a chip one: the AT's timer, keyboard and cascade
        // lines are edge-triggered whatever firmware writes.
        let master = bench("master");
        master.poke_elcr(0xff);
        assert_eq!(master.peek_elcr(), 0xf8, "IR0, IR1 and IR2 are fixed");

        let slave = bench("slave");
        slave.poke_elcr(0xff);
        assert_eq!(slave.peek_elcr(), 0xde, "IRQ8 and IRQ13 are fixed");

        // And a chip on no such board has nothing hardwired.
        let single = bench("single");
        single.poke_elcr(0xff);
        assert_eq!(single.peek_elcr(), 0xff);
    }

    #[test]
    fn an_unwritten_elcr_leaves_the_trigger_mode_to_ltim() {
        // A board with no ELCR at all: an all-zero latch must not be read as
        // "every line edge-triggered" and override LTIM.
        let b = bench("master");
        b.init(ICW1_CASCADE | ICW1_LTIM, MASTER_BASE, 0x04, ICW4_8086);
        b.raise(4);
        assert_eq!(b.ack(), u32::from(MASTER_BASE | 4));
        b.poke(0, 0x20);
        assert!(b.probe.high(), "LTIM still says level");

        // Writing the latch, even with the same value it already read, is what
        // hands the decision over: IR4 is edge-triggered from here on, so the
        // request it is holding is the last one it makes.
        b.poke_elcr(0x00);
        assert_eq!(b.ack(), u32::from(MASTER_BASE | 4));
        b.poke(0, 0x20);
        assert!(!b.probe.high());
    }

    #[test]
    fn a_debug_write_to_the_elcr_is_refused_but_a_debug_read_is_not() {
        let b = bench("master");
        b.poke_elcr(0x18);
        let mut byte = [0u8; 1];
        b.elcr_ops().read(0, &mut byte, MemAttrs::DEBUG).unwrap();
        assert_eq!(byte[0], 0x18);
        assert!(b.elcr_ops().write(0, &[0x00], MemAttrs::DEBUG).is_err());
        assert_eq!(b.peek_elcr(), 0x18, "and nothing changed");
    }

    #[test]
    fn a_poll_acknowledges_and_a_debug_read_does_not() {
        let b = bench("master");
        b.init(ICW1_CASCADE, MASTER_BASE, 0x04, ICW4_8086);
        b.raise(5);

        b.poke(0, OCW3_SELECT | OCW3_POLL);
        assert_eq!(
            b.peek_debug(0),
            0x20,
            "a debug read answers with IRR and leaves the poll armed"
        );
        assert_eq!(b.pic.in_service(), 0, "and acknowledges nothing");
        assert!(b.probe.high());

        assert_eq!(b.peek(0), POLL_PENDING | 5);
        assert_eq!(b.pic.in_service(), 0x20, "the guest's read did acknowledge");
        assert!(!b.probe.high());

        // With nothing pending the poll byte says so.
        b.poke(0, OCW3_SELECT | OCW3_POLL);
        assert_eq!(b.peek(0) & POLL_PENDING, 0);
    }

    #[test]
    fn ocw3_selects_which_register_port_zero_reads() {
        let b = bench("master");
        b.init(ICW1_CASCADE, MASTER_BASE, 0x04, ICW4_8086);
        b.raise(1);
        b.ack();
        b.raise(0);

        assert_eq!(b.peek(0), 0x01, "IRR by default");
        b.poke(0, OCW3_SELECT | OCW3_RR | OCW3_RIS);
        assert_eq!(b.peek(0), 0x02, "and now ISR");
        b.poke(0, OCW3_SELECT | OCW3_RR);
        assert_eq!(b.peek(0), 0x01);
    }

    #[test]
    fn a_debug_write_is_refused_because_no_value_is_harmless() {
        let b = bench("master");
        assert!(b.pic.regs.write(0, &[0x20], MemAttrs::DEBUG).is_err());
        assert!(b.pic.regs.write(1, &[0xff], MemAttrs::DEBUG).is_err());
    }

    #[test]
    fn an_access_that_is_not_a_single_byte_is_refused() {
        let b = bench("master");
        assert!(
            b.pic
                .regs
                .read(0, &mut [0u8; 2], MemAttrs::DEFAULT)
                .is_err()
        );
        assert!(b.pic.regs.write(0, &[0u8; 2], MemAttrs::DEFAULT).is_err());
    }

    /// A master with a slave on IR2, wired the way the AT is and the way the
    /// realizer would do it: one net carrying the slave's `INT` to the
    /// master's `IR2`, and the slave's acknowledge handler attached to that
    /// same pin.
    fn cascade() -> (Bench, Bench) {
        let master = bench("master");
        let slave = bench("slave");

        // The slave's INT drives the master's IR2: one net, two pins. The
        // master's IR2 already has a pin from `bench`, but a second one on the
        // same line is what a machine file wiring two nets to one pin would
        // produce, and the chip's state is shared, so it behaves the same.
        let slave_int = slave.ids.alloc();
        let ir2 = master.pic.sink("ir2", &[slave_int]).expect("IR2 exists");
        let net = Wire::builder()
            .source(slave_int)
            .sink(ir2.sink, ir2.line)
            .build_shared();
        slave
            .pic
            .connect("int", WireSource::new(net, slave_int))
            .expect("a slave drives int");
        let ack = slave.pic.int_ack("int").expect("a slave answers INTA");
        master.pic.attach_int_ack("ir2", Arc::downgrade(&ack));

        master.init(ICW1_CASCADE, MASTER_BASE, 0x04, ICW4_8086);
        slave.init(ICW1_CASCADE, SLAVE_BASE, 0x02, ICW4_8086);
        (master, slave)
    }

    #[test]
    fn a_cascaded_slave_supplies_the_vector_and_the_master_records_the_level() {
        let (master, slave) = cascade();
        slave.raise(3); // IRQ11 on an AT
        assert!(
            master.probe.high(),
            "the slave's INT reached the master's IR2"
        );

        assert_eq!(
            master.ack(),
            u32::from(SLAVE_BASE | 3),
            "the second INTA pulse is answered by the slave"
        );
        assert_eq!(master.pic.in_service(), 0x04, "master ISR bit 2");
        assert_eq!(slave.pic.in_service(), 0x08, "slave ISR bit 3");
        assert!(!master.probe.high());

        // Which is why the handler must EOI both, and in that order: the
        // master still has IR2 in service after the slave is done.
        slave.poke(0, 0x20);
        assert_eq!(slave.pic.in_service(), 0);
        assert_eq!(master.pic.in_service(), 0x04);
        master.poke(0, 0x20);
        assert_eq!(master.pic.in_service(), 0);
    }

    #[test]
    fn a_cascade_level_the_master_does_not_know_about_is_answered_by_the_master() {
        let (master, slave) = cascade();
        // Re-initialize the master with an empty cascade mask: ICW3 is what
        // decides, not the presence of a handler.
        master.init(ICW1_CASCADE, MASTER_BASE, 0x00, ICW4_8086);
        slave.raise(3);
        assert_eq!(master.ack(), u32::from(MASTER_BASE | 2));
    }

    #[test]
    fn a_snapshot_round_trips_the_whole_chip() {
        let saved = bench("master");
        saved.init(ICW1_CASCADE | ICW1_LTIM, MASTER_BASE, 0x04, ICW4_8086);
        saved.raise(1);
        saved.ack();
        saved.raise(4);
        saved.poke(1, 0x40); // mask IR6
        saved.poke(0, 0xc0 | 3); // set priority: IR3 lowest
        saved.poke(0, OCW3_SELECT | OCW3_RR | OCW3_RIS);
        saved.poke(0, OCW3_SELECT | OCW3_POLL);
        saved.poke_elcr(0x28);

        let mut shape = MachineShape::new();
        shape.add_device("pic", CLASS.name).unwrap();
        let mut w = StateWriter::new(shape);
        {
            let mut chunk = w.chunk("pic", CLASS.name, CLASS.version).unwrap();
            saved.pic.save(&mut chunk).unwrap();
        }
        let bytes = w.to_vec().unwrap();

        let restored = bench("master");
        let reader = StateReader::new(&bytes).unwrap();
        let chunk = reader
            .load("pic", CLASS.name, CLASS.version, &Migrations::new())
            .unwrap();
        restored.pic.load(&mut chunk.reader()).unwrap();

        // Copied out one at a time: two chips' state locks are both at
        // `LockRank::DEVICE`, and holding one while taking the other is the
        // rank violation `core::sync` exists to catch.
        let after = restored.pic.regs.state.lock().clone();
        let before = saved.pic.regs.state.lock().clone();
        assert_eq!(after, before, "every field came back");
        assert!(
            restored.probe.high(),
            "and the INT output was recomputed from it"
        );

        // The bytes of a second save are the bytes of the first, which is the
        // property the other devices in this tree assert.
        let mut shape = MachineShape::new();
        shape.add_device("pic", CLASS.name).unwrap();
        let mut w = StateWriter::new(shape);
        {
            let mut chunk = w.chunk("pic", CLASS.name, CLASS.version).unwrap();
            restored.pic.save(&mut chunk).unwrap();
        }
        assert_eq!(w.to_vec().unwrap(), bytes);
    }

    #[test]
    fn a_reset_puts_every_line_down() {
        let b = bench("master");
        b.init(ICW1_CASCADE, MASTER_BASE, 0x04, ICW4_8086);
        b.raise(0);
        assert!(b.probe.high());
        b.pic.reset(ResetKind::Cold);
        assert!(!b.probe.high());
        assert_eq!(b.pic.requested(), 0);
        assert_eq!(b.pic.in_service(), 0);
    }

    #[test]
    fn properties_and_pins_are_checked_rather_than_ignored() {
        let pic = Pic8259::new(&Props::new().with("mode", "slave")).expect("slave is a mode");
        assert_eq!(pic.mode(), "slave");
        assert!(Pic8259::new(&Props::new().with("mode", "primary")).is_err());
        assert!(Pic8259::new(&Props::new().with("mdoe", "master")).is_err());

        let pic = Pic8259::default_device();
        assert_eq!(pic.mode(), "master");
        assert!(pic.sink("ir8", &[WireId::new(1)]).is_none());
        assert!(pic.sink("int", &[WireId::new(1)]).is_none());
        assert!(pic.int_ack("ir0").is_none());
        assert!(
            pic.connect("ir0", WireSource::new(dummy_wire(), WireId::new(1)))
                .is_err()
        );
        assert!(pic.region("").is_some());
        assert_eq!(
            pic.region("elcr").expect("the ELCR is published").len(),
            ELCR_WINDOW_LEN
        );
        assert!(pic.region("porta").is_none());
    }

    fn dummy_wire() -> Arc<Wire> {
        Wire::builder().source(WireId::new(1)).build_shared()
    }

    #[test]
    fn the_int_net_settles_low_when_the_last_request_is_taken() {
        // `announce` must drive, because a fresh net is low and a chip that
        // came back from a snapshot with a request pending would otherwise
        // never tell anyone.
        let b = bench("master");
        b.init(ICW1_CASCADE, MASTER_BASE, 0x04, ICW4_8086);
        b.raise(7);
        assert!(b.probe.high());
        b.pic.announce("int");
        assert!(b.probe.high());
        assert_eq!(b.int.level_of(b.int_id), Some(Level::High));
        b.ack();
        b.pic.announce("int");
        assert!(!b.probe.high());
    }
}
