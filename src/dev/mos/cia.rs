//! The MOS 8520 CIA: two ports, two timers, a 24-bit TOD counter and a shift
//! register.
//!
//! # Sources
//!
//! * **MOS 6526 Complex Interface Adapter data sheet** (MOS Technology /
//!   Commodore Semiconductor Group) for everything the two parts share: the
//!   sixteen registers and what a read or a write of each does, both timers and
//!   their control registers, the PB6/PB7 output modes, the serial shift
//!   register, the interrupt control register's read-to-clear and its
//!   `SET/CLEAR` write, the passive pull-ups on the port pins, and the reset
//!   state ("the timer latches are set to all ones, all other registers to
//!   zero").
//! * **MOS 8520 CIA data sheet** for the one section where the two differ: the
//!   8520 replaces the 6526's BCD time-of-day clock with a **24-bit binary
//!   counter** clocked from the TOD pin, in registers `$8`-`$A`, leaving `$B`
//!   unused. The latching rule survives the change and moves with the byte
//!   order: reading the **MSB** (`$A`) latches all three bytes, reading the
//!   **LSB** (`$8`) releases them.
//! * **Amiga Hardware Reference Manual** (Commodore-Amiga) for what a board
//!   does with the part, which is recorded in [`the board section`](self#on-an-amiga)
//!   below rather than implemented here.
//!
//! No emulator was consulted for any of it (`ROADMAP.md` §1); every Amiga
//! emulator in existence is GPL and none of them was opened.
//!
//! # The register map
//!
//! ```text
//!   $0 PRA    $4 TA LO   $8 TOD  7-0    $C SDR
//!   $1 PRB    $5 TA HI   $9 TOD 15-8    $D ICR
//!   $2 DDRA   $6 TB LO   $A TOD 23-16   $E CRA
//!   $3 DDRB   $7 TB HI   $B unused      $F CRB
//! ```
//!
//! The chip decodes four register-select lines and nothing else. **Where those
//! four lines come from is a board fact**: on an Amiga they are address lines
//! A8-A11 rather than A0-A3, so the sixteen registers are 256 bytes apart — see
//! below.
//!
//! # What is modelled
//!
//! * **Both ports and both direction registers**, pin by pin. Every pin is a
//!   wire: `pa0`…`pa7` and `pb0`…`pb7` are bidirectional, driving strongly when
//!   the direction register says output and presenting the chip's **passive
//!   pull-up** ([`Drive::WeakHigh`]) when it says input. A pin nothing is wired
//!   to therefore reads as a one, which is what the data sheet says a floating
//!   input does and what an Amiga's `/RDY`, `/TRK0`, `/WPRO` and `/CHNG` lines
//!   depend on.
//! * **Both timers**: 16-bit down-counters with their own latches, one-shot and
//!   continuous, the force-load strobe, the write-to-the-high-byte-while-
//!   stopped load, and timer B's four input modes — φ2, CNT, **timer A
//!   underflows** and timer A underflows gated by CNT. The chained mode is the
//!   one an Amiga uses to get a delay longer than 92 ms out of an E clock.
//! * **PB6 and PB7 as timer outputs**, in both pulse and toggle mode, and the
//!   rule that `PBON` makes the pin an output whatever `DDRB` says.
//! * **The TOD counter**: 24 bits, clocked by positive edges on the `tod` pin,
//!   with its alarm, the latch-on-MSB-read / release-on-LSB-read rule, the
//!   `CRB7` bit that sends a write to the alarm instead of the clock, and the
//!   stop-on-MSB-write / start-on-LSB-write rule that lets software set a time
//!   without the counter rolling under it.
//! * **The shift register**, both ways round: in on positive CNT edges, out at
//!   half the timer A underflow rate with CNT driven as the shift clock, and
//!   `ICR3` after the eighth bit either way. An Amiga keyboard arrives through
//!   the input half.
//! * **The interrupt control register**: five latched flags, a mask with the
//!   `SET/CLEAR` write, the combinational `IR` bit, and the read that clears
//!   every flag and releases the pin. A [`MemAttrs::debug`] read does none of
//!   it.
//! * **The `/PC` strobe**, which goes low for one cycle after a read or a write
//!   of `PRB`, and **`/FLAG`**, whose falling edge sets `ICR4`. Wiring one to
//!   the other is how a parallel port handshakes.
//!
//! # What is stored but not acted on
//!
//! * **`CRA` bit 7.** On a 6526 it selects between a 50 Hz and a 60 Hz TOD
//!   input; the 8520 counts edges on a pin and has no divider to select, so the
//!   bit reads back and does nothing.
//! * **Register `$B`.** The 6526's hours register; the 8520 has no fourth TOD
//!   byte. It reads zero and a write is dropped.
//! * **Contention on a port pin.** A pin the device drives reads back what the
//!   device drives. On silicon a strong external driver fighting an output
//!   stage wins or smokes; `core::wire` can represent that and this model does
//!   not consult it.
//!
//! # What is absent, and why
//!
//! * **The one-cycle interrupt pipeline.** On the real part a flag set in the
//!   same cycle as an `ICR` read has a documented race, and the 6526 and 8520
//!   resolve it differently from each other. Nothing here reproduces it: the
//!   flags are set at the cycle the counter reaches zero and read at the cycle
//!   of the access, with no pipeline between them.
//! * **`TOD` alarm equality while the registers are latched.** The comparison
//!   is against the running counter, as the data sheet describes; the latch
//!   affects only what a read returns.
//!
//! # On an Amiga
//!
//! An Amiga has two of these and decodes them in a way that is **entirely the
//! board's business** — nothing below this paragraph is implemented here:
//!
//! * CIA-A answers at `$BFE001` and CIA-B at `$BFD000`. The 68000 has no `A0`
//!   pin, so the odd address selects the lower byte lane (`/LDS`) and the even
//!   one the upper (`/UDS`): one chip sits on `D0`-`D7` and the other on
//!   `D8`-`D15`, and a word access at `$BFE000` reaches both at once.
//! * The register-select lines are `A8`-`A11`, so register *n* of CIA-A is at
//!   `$BFE001 + n * 0x100` and register *n* of CIA-B at `$BFD000 + n * 0x100`.
//! * φ2 is the **E clock**, the 68000's clock divided by ten: 709379 Hz on a
//!   PAL machine and 715909 Hz on an NTSC one. That is the `clock` a machine
//!   file gives the object.
//! * The TOD pins are driven by the display: CIA-A's from the **vertical**
//!   blank (50 or 60 Hz) and CIA-B's from **horizontal** sync. They are
//!   different rates on purpose, and neither is the E clock.
//! * CIA-A port A carries the floppy `/RDY`, `/TRK0`, `/WPRO` and `/CHNG`
//!   inputs and the power LED and `/OVL` outputs; CIA-B's two ports carry the
//!   floppy control lines and the parallel port. The model has pins; a board
//!   decides what they mean.

use alloc::boxed::Box;
use alloc::string::String;
use alloc::sync::Arc;
use alloc::vec::Vec;
use core::fmt;

use crate::core::device::{Device, DeviceClass, RealizeCtx, ResetKind, SinkPin};
use crate::core::error::{BusError, Error, Result};
use crate::core::props::Props;
use crate::core::sched::{AccessKind, LazyHandle};
use crate::core::space::{AccessConstraints, MemAttrs, MemOps, MemResult, Region, RegionRef};
use crate::core::state::{ChunkReader, ChunkWriter, Sink, Source};
use crate::core::sync::{AtomicU64, LockRank, Mutex, Ordering};
use crate::core::value::{Endian, Width};
use crate::core::wire::{Drive, FanIn, Level, Resolve, WireId, WireSink, WireSource};
use crate::machine::realize::Instance;
use crate::machine::validate::port_index;

/// The class name a machine description writes.
const CLASS_NAME: &str = "mos.8520";

/// The snapshot chunk version. Bump with the encoding, never on its own.
const STATE_VERSION: u32 = 1;

/// How many bytes of address space the sixteen registers occupy.
///
/// Sixteen, because the chip decodes RS0-RS3 and the model leaves every board's
/// idea of where those four lines come from to the board.
pub const REGISTER_COUNT: u64 = 16;

/// How many pins each port has.
pub const PORT_PINS: u32 = 8;

/// The interrupt output. High is "requesting", as everywhere else in this tree;
/// the pin on the chip is `/IRQ` and is open-drain and active low.
pub const IRQ_PIN: &str = "irq";
/// The `/PC` handshake output, which carries its true polarity: it idles high
/// and pulses low for one cycle after an access to `PRB`.
pub const PC_PIN: &str = "pc";
/// The serial data pin, an input or an output as `CRA6` says.
pub const SP_PIN: &str = "sp";
/// The serial clock pin, likewise, and also timer input.
pub const CNT_PIN: &str = "cnt";
/// The `/FLAG` input: a falling edge sets `ICR4`.
pub const FLAG_PIN: &str = "flag";
/// The TOD clock input: a rising edge advances the counter.
pub const TOD_PIN: &str = "tod";
/// The name a port A pin takes, before its number: `pa0`…`pa7`.
pub const PORT_A_PREFIX: &str = "pa";
/// And a port B pin: `pb0`…`pb7`.
pub const PORT_B_PREFIX: &str = "pb";

// -- the lines a sink is known by -------------------------------------------

/// `pa0`, and the seven above it.
const LINE_PA: u32 = 0;
/// `pb0`, and the seven above it.
const LINE_PB: u32 = 8;
/// The TOD input.
const LINE_TOD: u32 = 16;
/// The CNT input.
const LINE_CNT: u32 = 17;
/// The SP input.
const LINE_SP: u32 = 18;
/// The `/FLAG` input.
const LINE_FLAG: u32 = 19;

// -- interrupt control ------------------------------------------------------

/// `ICR` bit 0: timer A underflowed.
const ICR_TA: u8 = 0x01;
/// `ICR` bit 1: timer B underflowed.
const ICR_TB: u8 = 0x02;
/// `ICR` bit 2: the TOD counter reached the alarm.
const ICR_ALARM: u8 = 0x04;
/// `ICR` bit 3: the shift register finished a byte.
const ICR_SP: u8 = 0x08;
/// `ICR` bit 4: a falling edge on `/FLAG`.
const ICR_FLAG: u8 = 0x10;
/// The five that are flags.
const ICR_SOURCES: u8 = 0x1f;
/// `ICR` bit 7: `IR` on a read, `SET/CLEAR` on a write. Never a flag.
const ICR_IR: u8 = 0x80;

// -- the control registers --------------------------------------------------

/// `CRx` bit 0: run.
const CR_START: u8 = 0x01;
/// `CRx` bit 1: the timer's output appears on PB6 (A) or PB7 (B).
const CR_PBON: u8 = 0x02;
/// `CRx` bit 2: 1 toggles the port output, 0 pulses it for one cycle.
const CR_OUTMODE: u8 = 0x04;
/// `CRx` bit 3: 1 is one-shot, 0 is continuous.
const CR_ONESHOT: u8 = 0x08;
/// `CRx` bit 4: force the latch into the counter. A strobe: it never reads back.
const CR_LOAD: u8 = 0x10;
/// `CRA` bit 5: count positive CNT transitions instead of φ2.
const CRA_INMODE: u8 = 0x20;
/// `CRA` bit 6: the serial port shifts out instead of in.
const CRA_SPMODE: u8 = 0x40;
/// `CRA` bit 7: the 6526's 50/60 Hz TOD divider select, unused on an 8520.
const CRA_TODIN: u8 = 0x80;
/// `CRB` bits 6-5: timer B's input.
const CRB_INMODE: u8 = 0x60;
/// `CRB` bit 7: a write to `$8`-`$A` sets the alarm rather than the clock.
const CRB_ALARM: u8 = 0x80;

/// `CRB` input mode 0: count φ2.
const TB_IN_PHI2: u8 = 0x00;
/// Mode 1: count positive CNT transitions.
const TB_IN_CNT: u8 = 0x20;
/// Mode 2: count timer A underflows. The Amiga's long-delay mode.
const TB_IN_TA: u8 = 0x40;
/// Mode 3: count timer A underflows that happen while CNT is high.
const TB_IN_TA_CNT: u8 = 0x60;

/// The TOD counter's width.
const TOD_MASK: u32 = 0x00ff_ffff;

/// "Nothing scheduled", as [`Shared::next_event`] spells it.
const NO_EVENT: u64 = u64::MAX;

/// The MOS 8520 as a device.
///
/// Two-phase like every device (`ROADMAP.md` §4.4): [`Cia::new`] validates
/// properties and builds the register block, and [`Device::realize`] does
/// nothing because a `map` statement places the region.
#[derive(Debug)]
pub struct Cia {
    shared: Arc<Shared>,
    region: RegionRef,
    /// The sinks handed out so far, kept alive because a net holds only a weak
    /// reference to them (`ROADMAP.md` §4.3).
    pins: Mutex<Vec<Arc<CiaPin>>>,
}

/// Everything both halves of the device reach.
struct Shared {
    state: Mutex<State>,
    /// φ2 ticks simulated, published for the scheduler's lock-free question.
    ticks: AtomicU64,
    /// The tick the next event falls on, or [`NO_EVENT`].
    next_event: AtomicU64,
    /// The output pins, connected at realize time.
    out: Mutex<Outputs>,
    /// The catch-up handle the register block syncs through.
    lazy: Mutex<Option<LazyHandle>>,
}

/// Where each output pin drives, once a `wire` statement has named it.
#[derive(Debug, Default)]
struct Outputs {
    irq: Option<WireSource>,
    pc: Option<WireSource>,
    sp: Option<WireSource>,
    cnt: Option<WireSource>,
    pa: [Option<WireSource>; 8],
    pb: [Option<WireSource>; 8],
}

/// Everything the guest can see or change.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct State {
    /// φ2 ticks simulated. The authoritative copy; the atomic mirrors it.
    ticks: u64,

    // -- the ports ----------------------------------------------------------
    pra: u8,
    prb: u8,
    ddra: u8,
    ddrb: u8,
    /// What the outside is driving onto port A's pins. A one is the chip's own
    /// pull-up, which is why this resets to all ones rather than to zero.
    pa_in: u8,
    /// And port B's.
    pb_in: u8,

    // -- the timers ---------------------------------------------------------
    ta: u16,
    tb: u16,
    ta_latch: u16,
    tb_latch: u16,
    cra: u8,
    crb: u8,
    /// PB6's level, whichever output mode timer A is in.
    pb6: bool,
    /// PB7's, for timer B.
    pb7: bool,
    /// The tick timer A last underflowed on, which is what makes a one-cycle
    /// pulse on PB6 observable at the cycle it happens and at no other.
    ta_underflow_at: u64,
    /// The same for timer B and PB7.
    tb_underflow_at: u64,

    // -- the time-of-day counter -------------------------------------------
    tod: u32,
    tod_alarm: u32,
    /// What a read returns while the latch holds.
    tod_latch: u32,
    /// Whether a read of `$A` has latched and no read of `$8` has released it.
    tod_latched: bool,
    /// Set by a write of the MSB and cleared by a write of the LSB, so that
    /// software can set a time without the counter running under the write.
    tod_halted: bool,

    // -- the serial shift register -----------------------------------------
    /// What a read of `$C` returns: the last byte shifted in, or the last byte
    /// written.
    sdr: u8,
    /// The shift register proper.
    sr_shift: u8,
    /// Bits of the byte in progress.
    sr_bits: u8,
    /// Output mode: a byte written to `$C` that has not started shifting.
    sr_buffer: u8,
    /// Whether [`State::sr_buffer`] holds one.
    sr_pending: bool,
    /// Output mode: whether a byte is shifting out now.
    sr_active: bool,
    /// Output mode: which half of the CNT period the next underflow is. A bit
    /// leaves every *second* underflow, which is what makes the shift rate half
    /// the underflow rate.
    sr_half: bool,
    /// What the SP pin drives in output mode.
    sp_out: bool,
    /// And CNT, which is the shift clock.
    cnt_out: bool,
    /// The level last seen on the CNT pin, for the positive-edge detector.
    cnt_in: bool,
    /// The level last seen on the SP pin, which is what an input shift samples.
    sp_in: bool,
    /// The level last seen on `/FLAG`, for the negative-edge detector.
    flag_in: bool,
    /// The level last seen on the TOD pin, for its positive-edge detector.
    tod_in: bool,

    // -- interrupts and the handshake --------------------------------------
    icr_data: u8,
    icr_mask: u8,
    /// The tick `/PC` returns high on. Zero means it is already high.
    pc_until: u64,
}

impl Default for State {
    fn default() -> State {
        State::fresh(0)
    }
}

impl State {
    /// The reset state, keeping `ticks`.
    ///
    /// "The port pins are set as inputs and port registers to zero… the timer
    /// control registers are set to zero and the timer latches to all ones. All
    /// other registers are reset to zero" — 6526 data sheet, RESET. The
    /// counters come up holding the latch, because that is what the next thing
    /// to read them would see on the part and because an undefined number is
    /// not a deterministic machine (`ROADMAP.md` §0).
    fn fresh(ticks: u64) -> State {
        State {
            ticks,
            pra: 0,
            prb: 0,
            ddra: 0,
            ddrb: 0,
            pa_in: 0xff,
            pb_in: 0xff,
            ta: 0xffff,
            tb: 0xffff,
            ta_latch: 0xffff,
            tb_latch: 0xffff,
            cra: 0,
            crb: 0,
            pb6: false,
            pb7: false,
            ta_underflow_at: u64::MAX,
            tb_underflow_at: u64::MAX,
            tod: 0,
            tod_alarm: 0,
            tod_latch: 0,
            tod_latched: false,
            tod_halted: false,
            sdr: 0,
            sr_shift: 0,
            sr_bits: 0,
            sr_buffer: 0,
            sr_pending: false,
            sr_active: false,
            sr_half: false,
            sp_out: true,
            cnt_out: true,
            cnt_in: true,
            sp_in: true,
            flag_in: true,
            tod_in: false,
            icr_data: 0,
            icr_mask: 0,
            pc_until: 0,
        }
    }
}

impl fmt::Debug for Shared {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let mut s = f.debug_struct("Shared");
        match self.state.try_lock() {
            Some(state) => s.field("state", &*state).finish(),
            None => s.field("state", &"<in use>").finish(),
        }
    }
}

impl Cia {
    /// Validate `props` and build the device.
    ///
    /// # Errors
    ///
    /// [`Error::Property`] if a property this
    /// class does not know was given. It takes none: everything that varies
    /// between the two CIAs on a board is wiring.
    pub fn new(props: &Props) -> Result<Cia> {
        props.reader().finish()?;
        Ok(Cia::bare())
    }

    /// One with no properties to read.
    #[must_use]
    pub fn bare() -> Cia {
        let shared = Arc::new(Shared {
            state: Mutex::with_rank(LockRank::DEVICE, State::fresh(0)),
            ticks: AtomicU64::new(0),
            next_event: AtomicU64::new(NO_EVENT),
            out: Mutex::with_rank(LockRank::WIRE, Outputs::default()),
            lazy: Mutex::with_rank(LockRank::WIRE, None),
        });
        shared.publish(&shared.state.lock());
        let port = Arc::new(CiaRegs {
            shared: Arc::clone(&shared),
        });
        let region = Arc::new(Region::io("cia", REGISTER_COUNT, port as Arc<dyn MemOps>));
        Cia {
            shared,
            region,
            pins: Mutex::with_rank(LockRank::WIRE, Vec::new()),
        }
    }

    /// Drive every port A pin at once, as a board with no wires would.
    ///
    /// A one is the idle level, because the pins have pull-ups. Pins the guest
    /// has made outputs ignore it.
    pub fn set_port_a(&self, level: u8) {
        {
            let mut state = self.shared.state.lock();
            if state.pa_in == level {
                return;
            }
            state.pa_in = level;
        }
        self.shared.refresh();
    }

    /// The same for port B.
    pub fn set_port_b(&self, level: u8) {
        {
            let mut state = self.shared.state.lock();
            if state.pb_in == level {
                return;
            }
            state.pb_in = level;
        }
        self.shared.refresh();
    }

    /// What port A's pins are at: the output register where the direction
    /// register says output, and the driven level everywhere else.
    #[must_use]
    pub fn port_a(&self) -> u8 {
        self.shared.state.lock().port_a()
    }

    /// The same for port B, timer outputs included.
    #[must_use]
    pub fn port_b(&self) -> u8 {
        self.shared.state.lock().port_b()
    }

    /// Timer A's counter, without disturbing anything.
    #[must_use]
    pub fn timer_a(&self) -> u16 {
        self.shared.state.lock().ta
    }

    /// Timer B's counter.
    #[must_use]
    pub fn timer_b(&self) -> u16 {
        self.shared.state.lock().tb
    }

    /// The 24-bit TOD counter, live rather than latched.
    #[must_use]
    pub fn tod(&self) -> u32 {
        self.shared.state.lock().tod
    }

    /// What `CRA` bit 7 holds.
    ///
    /// On a 6526 it picks between a 50 Hz and a 60 Hz TOD input. An 8520 counts
    /// edges on a pin and has no divider to pick, so the bit is stored, read
    /// back and otherwise ignored — but a board that wants to know which rate
    /// software believes it configured can ask.
    #[must_use]
    pub fn todin(&self) -> bool {
        self.shared.state.lock().cra & CRA_TODIN != 0
    }

    /// The interrupt control register as software would read it, `IR` included
    /// and without clearing it.
    #[must_use]
    pub fn icr(&self) -> u8 {
        State::visible_icr(&self.shared.state.lock())
    }

    /// φ2 ticks the timers have counted.
    #[must_use]
    pub fn ticks(&self) -> u64 {
        self.shared.ticks.load(Ordering::Relaxed)
    }

    /// The level the interrupt output is driving; high is "requesting".
    #[must_use]
    pub fn irq_level(&self) -> Level {
        State::irq(&self.shared.state.lock())
    }

    /// Deliver one whole pulse on the TOD pin: a rising edge, which is the one
    /// that counts, and the fall that readies the next.
    ///
    /// The pin exists as a wire as well; this is the entry point for a board
    /// that generates the tick itself rather than routing a signal to it.
    pub fn tod_pulse(&self) {
        self.shared.edge(LINE_TOD, true);
        self.shared.edge(LINE_TOD, false);
    }

    /// Set the level on the TOD pin. A rising edge advances the counter.
    pub fn set_tod_pin(&self, level: bool) {
        self.shared.edge(LINE_TOD, level);
    }

    /// Deliver one positive edge on CNT: a serial shift in input mode, and a
    /// count for whichever timer is in a CNT input mode.
    pub fn cnt_edge(&self, level: bool) {
        self.shared.edge(LINE_CNT, level);
    }

    /// Set the level on the SP pin, which an input shift samples.
    pub fn set_sp(&self, level: bool) {
        self.shared.edge(LINE_SP, level);
    }

    /// Set the level on `/FLAG`; a falling edge sets `ICR4`.
    pub fn set_flag(&self, level: bool) {
        self.shared.edge(LINE_FLAG, level);
    }

    /// Connect one output pin by name.
    ///
    /// # Errors
    ///
    /// [`Error::Config`] if the chip drives no such pin.
    pub fn connect_pin(&self, port: &str, source: WireSource) -> Result<()> {
        {
            let mut out = self.shared.out.lock();
            match port {
                IRQ_PIN => out.irq = Some(source),
                PC_PIN => out.pc = Some(source),
                SP_PIN => out.sp = Some(source),
                CNT_PIN => out.cnt = Some(source),
                _ => {
                    if let Some(n) = port_index(port, PORT_A_PREFIX, PORT_PINS) {
                        out.pa[n as usize] = Some(source);
                    } else if let Some(n) = port_index(port, PORT_B_PREFIX, PORT_PINS) {
                        out.pb[n as usize] = Some(source);
                    } else {
                        return Err(Error::Config {
                            at: String::from(port),
                            message: alloc::format!(
                                "an 8520 drives `{IRQ_PIN}`, `{PC_PIN}`, `{SP_PIN}`, `{CNT_PIN}` \
                                 and `{PORT_A_PREFIX}0`…`{PORT_B_PREFIX}7`"
                            ),
                        });
                    }
                }
            }
        }
        self.shared.refresh();
        Ok(())
    }

    /// Connect the catch-up handle the register block syncs through (§4.2).
    pub fn attach_lazy(&self, handle: LazyHandle) {
        *self.shared.lazy.lock() = Some(handle);
    }

    /// Run the chip until `target` φ2 ticks have passed in total.
    ///
    /// The catch-up entry point. Running backwards is a no-op, not an error.
    pub fn advance_to(&self, target: u64) {
        self.shared.advance_to(target);
    }
}

impl Shared {
    /// Publish what the scheduler may ask for without taking a lock.
    fn publish(&self, state: &State) {
        self.ticks.store(state.ticks, Ordering::Relaxed);
        self.next_event.store(state.next_event(), Ordering::Relaxed);
    }

    /// Drive every output pin to whatever the state now says.
    ///
    /// Called with no lock held: the re-entrancy contract in `core::device` is
    /// that outward calls happen after the critical section, never inside it.
    fn refresh(&self) {
        let pins = self.state.lock().pins();
        let out = self.out.lock();
        if let Some(src) = &out.irq {
            src.set(pins.irq);
        }
        if let Some(src) = &out.pc {
            src.set(pins.pc);
        }
        if let Some(src) = &out.sp {
            src.drive(pins.sp);
        }
        if let Some(src) = &out.cnt {
            src.drive(pins.cnt);
        }
        for (src, drive) in out.pa.iter().zip(pins.pa) {
            if let Some(src) = src {
                src.drive(drive);
            }
        }
        for (src, drive) in out.pb.iter().zip(pins.pb) {
            if let Some(src) = src {
                src.drive(drive);
            }
        }
    }

    /// Bring the chip up to date before an access.
    ///
    /// A debug access advances nothing (`ROADMAP.md` §15, invariant 5).
    fn sync(&self, debug: bool) {
        let handle = self.lazy.lock().clone();
        let Some(handle) = handle else {
            return;
        };
        let kind = if debug {
            AccessKind::Debug
        } else {
            AccessKind::Guest
        };
        // A refusal means catch-up for this chip is already running further up
        // the stack. The access still has to be answered, and answering it from
        // where the timers stand is the only defined thing to do.
        let _ = handle.sync(kind);
    }

    fn advance_to(&self, target: u64) {
        let moved = {
            let mut state = self.state.lock();
            if target <= state.ticks {
                return;
            }
            let before = state.pins();
            let elapsed = target - state.ticks;
            state.run(elapsed);
            state.ticks = target;
            self.publish(&state);
            before != state.pins()
        };
        if moved {
            self.refresh();
        }
    }

    /// A level arriving on one of the input pins.
    fn edge(&self, line: u32, level: bool) {
        // The chip has to be at the current tick before an edge lands on it:
        // an edge that counts a timer or sets a flag does so at the instant the
        // wire moved, not at the end of the scheduler's quantum.
        self.sync(false);
        let moved = {
            let mut state = self.state.lock();
            let before = state.pins();
            state.input(line, level);
            self.publish(&state);
            before != state.pins()
        };
        if moved {
            self.refresh();
        }
    }
}

impl State {
    // -- what the pins are at -----------------------------------------------

    /// Port A as a read of `PRA` sees it.
    fn port_a(&self) -> u8 {
        (self.pra & self.ddra) | (self.pa_in & !self.ddra)
    }

    /// Port B as a read of `PRB` sees it, timer outputs included.
    ///
    /// "PB ON … this overrides the DDRB bit" (6526 data sheet, CRA/CRB), so a
    /// timer output is what the pin carries whichever way the direction
    /// register points.
    fn port_b(&self) -> u8 {
        let mut value = (self.prb & self.ddrb) | (self.pb_in & !self.ddrb);
        if self.cra & CR_PBON != 0 {
            value = (value & !0x40) | (u8::from(self.ta_out()) << 6);
        }
        if self.crb & CR_PBON != 0 {
            value = (value & !0x80) | (u8::from(self.tb_out()) << 7);
        }
        value
    }

    /// What timer A puts on PB6.
    ///
    /// Toggle mode keeps a level, which [`State::pb6`] holds. Pulse mode has no
    /// level to keep — "a single positive pulse of one cycle duration following
    /// a timer underflow" — so it is read off the tick the underflow happened
    /// on, which is a tick the scheduler always stops the machine at.
    fn ta_out(&self) -> bool {
        if self.cra & CR_OUTMODE != 0 {
            self.pb6
        } else {
            self.ta_underflow_at == self.ticks
        }
    }

    /// The same for timer B and PB7.
    fn tb_out(&self) -> bool {
        if self.crb & CR_OUTMODE != 0 {
            self.pb7
        } else {
            self.tb_underflow_at == self.ticks
        }
    }

    /// `IR`'s own expression: any flag that is enabled.
    fn irq(&self) -> Level {
        if self.icr_data & self.icr_mask & ICR_SOURCES != 0 {
            Level::High
        } else {
            Level::Low
        }
    }

    /// `ICR` as it reads: the flags, plus `IR` if any enabled one is set.
    fn visible_icr(&self) -> u8 {
        let mut value = self.icr_data & ICR_SOURCES;
        if self.irq() == Level::High {
            value |= ICR_IR;
        }
        value
    }

    /// Every output pin's stage, which is what [`Shared::refresh`] pushes and
    /// what [`Shared::advance_to`] compares to decide whether anything moved.
    fn pins(&self) -> Pins {
        let mut pa = [Drive::HiZ; 8];
        let mut pb = [Drive::HiZ; 8];
        for (n, slot) in pa.iter_mut().enumerate() {
            // An input pin is not driven — it is held up by the passive pull-up
            // the data sheet gives every port pin.
            *slot = if self.ddra & (1 << n) != 0 {
                Drive::strong(Level::from(self.pra & (1 << n) != 0))
            } else {
                Drive::WeakHigh
            };
        }
        let pb_level = self.port_b();
        for (n, slot) in pb.iter_mut().enumerate() {
            let timer_out =
                (n == 6 && self.cra & CR_PBON != 0) || (n == 7 && self.crb & CR_PBON != 0);
            *slot = if timer_out || self.ddrb & (1 << n) != 0 {
                Drive::strong(Level::from(pb_level & (1 << n) != 0))
            } else {
                Drive::WeakHigh
            };
        }
        Pins {
            irq: self.irq(),
            // `/PC` is low for the one cycle after a PRB access and high the
            // rest of the time, which is its true polarity rather than this
            // tree's asserted-high convention: it is wired to things like
            // `/FLAG` that are polarity-sensitive.
            pc: Level::from(self.pc_until <= self.ticks),
            sp: if self.cra & CRA_SPMODE != 0 {
                Drive::strong(Level::from(self.sp_out))
            } else {
                Drive::HiZ
            },
            cnt: if self.cra & CRA_SPMODE != 0 {
                Drive::strong(Level::from(self.cnt_out))
            } else {
                Drive::HiZ
            },
            pa,
            pb,
        }
    }

    // -- time ---------------------------------------------------------------

    /// Whether timer A counts φ2 right now.
    fn ta_counts_phi2(&self) -> bool {
        self.cra & CR_START != 0 && self.cra & CRA_INMODE == 0
    }

    /// Whether timer B counts φ2 right now.
    fn tb_counts_phi2(&self) -> bool {
        self.crb & CR_START != 0 && self.crb & CRB_INMODE == TB_IN_PHI2
    }

    /// Whether timer B counts timer A's underflows right now.
    ///
    /// Mode 3 gates them on CNT being high, which is a level this model tracks,
    /// so the two chained modes differ by exactly that test.
    fn tb_counts_ta(&self) -> bool {
        self.crb & CR_START != 0
            && match self.crb & CRB_INMODE {
                TB_IN_TA => true,
                TB_IN_TA_CNT => self.cnt_in,
                _ => false,
            }
    }

    /// Count `n` into a continuous timer, answering the new counter and how
    /// many times it underflowed.
    ///
    /// A counter holding `N` underflows after `N + 1` counts and reloads from
    /// the latch: "the timer counts down from the latched value to zero,
    /// generates an interrupt and reloads the latched value" (6526 data sheet,
    /// TIMER A).
    fn count_free(counter: u16, latch: u16, n: u64) -> (u16, u64) {
        let first = u64::from(counter) + 1;
        if n < first {
            (counter - n as u16, 0)
        } else {
            let after = n - first;
            let period = u64::from(latch) + 1;
            (latch - (after % period) as u16, 1 + after / period)
        }
    }

    /// The tick of the last underflow in a run of `underflows` that started at
    /// `from` with the counter at `counter`.
    fn last_underflow_at(from: u64, counter: u16, latch: u16, underflows: u64) -> u64 {
        let first = from + u64::from(counter) + 1;
        first + (underflows - 1) * (u64::from(latch) + 1)
    }

    /// Count `n` φ2 ticks — or, for timer B in a chained mode, `n` timer A
    /// underflows — into both timers, setting the flags they raise.
    fn run(&mut self, elapsed: u64) {
        let start = self.ticks;

        // -- timer A --------------------------------------------------------
        let mut ta_underflows = 0u64;
        if self.ta_counts_phi2() {
            if self.cra & CR_ONESHOT != 0 {
                let first = u64::from(self.ta) + 1;
                if elapsed < first {
                    self.ta -= elapsed as u16;
                } else {
                    // "In one-shot mode the timer will count down, generate an
                    // interrupt, reload the latch and stop" — and the START bit
                    // is cleared by the chip itself.
                    self.ta = self.ta_latch;
                    self.cra &= !CR_START;
                    ta_underflows = 1;
                    self.ta_underflow_at = start + first;
                }
            } else {
                let (counter, count) = State::count_free(self.ta, self.ta_latch, elapsed);
                if count > 0 {
                    self.ta_underflow_at =
                        State::last_underflow_at(start, self.ta, self.ta_latch, count);
                }
                self.ta = counter;
                ta_underflows = count;
            }
        }
        if ta_underflows > 0 {
            self.icr_data |= ICR_TA;
            self.pb6 = self.timer_output(self.cra, self.pb6, ta_underflows);
            self.shift_out(ta_underflows);
        }

        // -- timer B --------------------------------------------------------
        //
        // Either φ2, or timer A's underflows, which is the mode that turns a
        // 92-millisecond maximum into an hour and a half.
        let phi2 = self.tb_counts_phi2();
        let tb_input = if phi2 {
            elapsed
        } else if self.tb_counts_ta() {
            ta_underflows
        } else {
            0
        };
        if tb_input > 0 {
            let before = self.tb;
            let one_shot = self.crb & CR_ONESHOT != 0;
            let mut tb_underflows = 0u64;
            if one_shot {
                let first = u64::from(self.tb) + 1;
                if tb_input < first {
                    self.tb -= tb_input as u16;
                } else {
                    self.tb = self.tb_latch;
                    self.crb &= !CR_START;
                    tb_underflows = 1;
                }
            } else {
                let (counter, count) = State::count_free(self.tb, self.tb_latch, tb_input);
                self.tb = counter;
                tb_underflows = count;
            }
            if tb_underflows > 0 {
                // A chained underflow happens on the tick of the timer A
                // underflow that caused it, and the scheduler stops the machine
                // on every one of those, so the last one is the current tick's.
                self.tb_underflow_at = if !phi2 {
                    self.ta_underflow_at
                } else if one_shot {
                    start + u64::from(before) + 1
                } else {
                    State::last_underflow_at(start, before, self.tb_latch, tb_underflows)
                };
                self.icr_data |= ICR_TB;
                self.pb7 = self.timer_output(self.crb, self.pb7, tb_underflows);
            }
        }
    }

    /// What a timer's port output is after `underflows` of them.
    ///
    /// Toggle mode flips on each; pulse mode is high for the single cycle of an
    /// underflow, which [`State::pins`] reads off the underflow tick rather
    /// than off a level (6526 data sheet, "TIMER A OUTPUT MODES").
    fn timer_output(&self, cr: u8, level: bool, underflows: u64) -> bool {
        if cr & CR_OUTMODE != 0 {
            level ^ (underflows % 2 == 1)
        } else {
            level
        }
    }

    /// Shift `underflows` of timer A's underflows through the serial port, if
    /// it is shifting out.
    ///
    /// "TIMER A is used for the baud rate generator… data is shifted out at 1/2
    /// the underflow rate of TIMER A" (6526 data sheet, SERIAL PORT), and CNT
    /// is the shift clock the receiver counts. The loop is bounded: eight bits
    /// is sixteen underflows, and the one byte a guest may have buffered is
    /// sixteen more, after which nothing is shifting and the rest of the budget
    /// is skipped.
    fn shift_out(&mut self, underflows: u64) {
        if self.cra & CRA_SPMODE == 0 {
            return;
        }
        if !self.sr_active && self.sr_pending {
            self.start_shift();
        }
        let mut left = underflows;
        while left > 0 && self.sr_active {
            left -= 1;
            self.cnt_out = !self.cnt_out;
            self.sr_half = !self.sr_half;
            if self.sr_half {
                continue;
            }
            // A full CNT period: one bit leaves, most significant first.
            self.sp_out = self.sr_shift & 0x80 != 0;
            self.sr_shift <<= 1;
            self.sr_bits += 1;
            if self.sr_bits == 8 {
                self.icr_data |= ICR_SP;
                self.sr_active = false;
                if self.sr_pending {
                    self.start_shift();
                }
            }
        }
        if !self.sr_active {
            // CNT is only a clock while something is being clocked.
            self.cnt_out = true;
        }
    }

    /// Move the buffered byte into the shift register.
    fn start_shift(&mut self) {
        self.sr_shift = self.sr_buffer;
        self.sr_pending = false;
        self.sr_active = true;
        self.sr_bits = 0;
        self.sr_half = false;
    }

    /// A level arriving on an input pin.
    fn input(&mut self, line: u32, level: bool) {
        match line {
            LINE_TOD => {
                let rising = level && !self.tod_in;
                self.tod_in = level;
                if rising {
                    self.tod_tick();
                }
            }
            LINE_FLAG => {
                // "FLAG: a negative edge on this pin sets ICR4" (6526 data
                // sheet). The pin is active low and keeps its own polarity.
                let falling = !level && self.flag_in;
                self.flag_in = level;
                if falling {
                    self.icr_data |= ICR_FLAG;
                }
            }
            LINE_SP => self.sp_in = level,
            LINE_CNT => {
                let rising = level && !self.cnt_in;
                self.cnt_in = level;
                if rising {
                    self.cnt_edge();
                }
            }
            0..=7 => {
                let bit = 1u8 << line;
                self.pa_in = (self.pa_in & !bit) | (u8::from(level) << line);
            }
            8..=15 => {
                let n = line - LINE_PB;
                let bit = 1u8 << n;
                self.pb_in = (self.pb_in & !bit) | (u8::from(level) << n);
            }
            // No other line exists; `sink` hands out no other number.
            _ => {}
        }
    }

    /// One positive edge on CNT: a shift in input mode, and a count for
    /// whichever timer is counting CNT.
    fn cnt_edge(&mut self) {
        if self.cra & CRA_SPMODE == 0 {
            // "In input mode, data on the SP pin is shifted into the shift
            // register on the rising edge of CNT. After 8 CNT pulses the data
            // is transferred to the Serial Data Register and an interrupt is
            // generated."
            self.sr_shift = (self.sr_shift << 1) | u8::from(self.sp_in);
            self.sr_bits += 1;
            if self.sr_bits == 8 {
                self.sdr = self.sr_shift;
                self.sr_bits = 0;
                self.icr_data |= ICR_SP;
            }
        }
        let mut ta_underflow = false;
        if self.cra & CR_START != 0 && self.cra & CRA_INMODE != 0 {
            ta_underflow = self.count_one_a();
        }
        let counts = match self.crb & CRB_INMODE {
            TB_IN_CNT => self.crb & CR_START != 0,
            TB_IN_TA | TB_IN_TA_CNT => self.crb & CR_START != 0 && ta_underflow,
            _ => false,
        };
        if counts {
            self.count_one_b();
        }
    }

    /// Count one into timer A, answering whether it underflowed.
    fn count_one_a(&mut self) -> bool {
        if self.ta == 0 {
            self.ta = self.ta_latch;
            if self.cra & CR_ONESHOT != 0 {
                self.cra &= !CR_START;
            }
            self.icr_data |= ICR_TA;
            self.ta_underflow_at = self.ticks;
            self.pb6 = self.timer_output(self.cra, self.pb6, 1);
            true
        } else {
            self.ta -= 1;
            false
        }
    }

    /// Count one into timer B.
    fn count_one_b(&mut self) {
        if self.tb == 0 {
            self.tb = self.tb_latch;
            if self.crb & CR_ONESHOT != 0 {
                self.crb &= !CR_START;
            }
            self.icr_data |= ICR_TB;
            self.tb_underflow_at = self.ticks;
            self.pb7 = self.timer_output(self.crb, self.pb7, 1);
        } else {
            self.tb -= 1;
        }
    }

    /// One TOD count, and the alarm comparison that follows it.
    fn tod_tick(&mut self) {
        if self.tod_halted {
            return;
        }
        self.tod = self.tod.wrapping_add(1) & TOD_MASK;
        if self.tod == self.tod_alarm {
            self.icr_data |= ICR_ALARM;
        }
    }

    /// The absolute tick the next event falls on, or [`NO_EVENT`].
    ///
    /// Only φ2 puts an event on the calendar. A CNT edge, a TOD edge and a
    /// `/FLAG` edge all arrive from a wire and carry their own instant, and the
    /// TOD counter is not on this clock at all.
    fn next_event(&self) -> u64 {
        let mut next = NO_EVENT;
        let ta_at = if self.ta_counts_phi2() {
            let at = self.ticks + u64::from(self.ta) + 1;
            next = next.min(at);
            Some(at)
        } else {
            None
        };
        if self.tb_counts_phi2() {
            next = next.min(self.ticks + u64::from(self.tb) + 1);
        } else if self.tb_counts_ta() {
            // The tick of the timer A underflow that takes timer B through
            // zero. A one-shot timer A has only the one underflow, and that
            // tick is already on the calendar above.
            if let Some(at) = ta_at.filter(|_| self.cra & CR_ONESHOT == 0) {
                next = next.min(at + u64::from(self.tb) * (u64::from(self.ta_latch) + 1));
            }
        }
        // The `/PC` strobe has to come back up on its own cycle.
        if self.pc_until > self.ticks {
            next = next.min(self.pc_until);
        }
        next
    }
}

/// Every output pin's stage at one instant.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct Pins {
    irq: Level,
    pc: Level,
    sp: Drive,
    cnt: Drive,
    pa: [Drive; 8],
    pb: [Drive; 8],
}

// ---------------------------------------------------------------------------
// the register block
// ---------------------------------------------------------------------------

/// The memory-mapped registers.
struct CiaRegs {
    shared: Arc<Shared>,
}

impl fmt::Debug for CiaRegs {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("CiaRegs").finish_non_exhaustive()
    }
}

impl CiaRegs {
    /// Read one register. `debug` suppresses every side effect.
    fn read_register(&self, index: u8, debug: bool) -> u8 {
        let mut state = self.shared.state.lock();
        match index {
            0x0 => state.port_a(),
            0x1 => {
                if !debug {
                    // "/PC goes low for one cycle following a read or a write
                    // of PRB" — the parallel handshake.
                    state.pc_until = state.ticks + 1;
                }
                state.port_b()
            }
            0x2 => state.ddra,
            0x3 => state.ddrb,
            0x4 => state.ta as u8,
            0x5 => (state.ta >> 8) as u8,
            0x6 => state.tb as u8,
            0x7 => (state.tb >> 8) as u8,
            // The TOD bytes. Reading the MSB latches all three and reading the
            // LSB releases them; the counter never stops.
            0x8 => {
                let value = if state.tod_latched {
                    state.tod_latch
                } else {
                    state.tod
                };
                if !debug {
                    state.tod_latched = false;
                }
                value as u8
            }
            0x9 => {
                let value = if state.tod_latched {
                    state.tod_latch
                } else {
                    state.tod
                };
                (value >> 8) as u8
            }
            0xa => {
                if !debug {
                    state.tod_latch = state.tod;
                    state.tod_latched = true;
                }
                let value = if state.tod_latched {
                    state.tod_latch
                } else {
                    state.tod
                };
                (value >> 16) as u8
            }
            // The 6526's hours register. An 8520 has no fourth TOD byte.
            0xb => 0,
            0xc => state.sdr,
            // "All flags remain set until the DATA register is read, whereupon
            // the register is cleared and the /IRQ line returns high."
            0xd => {
                let value = state.visible_icr();
                if !debug {
                    state.icr_data = 0;
                }
                value
            }
            // The force-load strobe is write-only and reads back as zero.
            0xe => state.cra & !CR_LOAD,
            _ => state.crb & !CR_LOAD,
        }
    }

    /// Write one register.
    fn write_register(&self, index: u8, value: u8) {
        let mut state = self.shared.state.lock();
        match index {
            0x0 => state.pra = value,
            0x1 => {
                state.prb = value;
                state.pc_until = state.ticks + 1;
            }
            0x2 => state.ddra = value,
            0x3 => state.ddrb = value,
            0x4 => state.ta_latch = (state.ta_latch & 0xff00) | u16::from(value),
            // "The timer latch is loaded into the timer on any timer underflow,
            // on a force load, or following a write to the high byte of the
            // prescaler while the timer is stopped."
            0x5 => {
                state.ta_latch = (state.ta_latch & 0x00ff) | (u16::from(value) << 8);
                if state.cra & CR_START == 0 {
                    state.ta = state.ta_latch;
                }
            }
            0x6 => state.tb_latch = (state.tb_latch & 0xff00) | u16::from(value),
            0x7 => {
                state.tb_latch = (state.tb_latch & 0x00ff) | (u16::from(value) << 8);
                if state.crb & CR_START == 0 {
                    state.tb = state.tb_latch;
                }
            }
            // The TOD bytes, or the alarm if CRB7 says so. Writing the MSB
            // stops the counter and writing the LSB starts it again, so that a
            // three-byte write cannot be overtaken by a tick.
            0x8 => {
                if state.crb & CRB_ALARM != 0 {
                    state.tod_alarm = (state.tod_alarm & 0xff_ff00) | u32::from(value);
                } else {
                    state.tod = (state.tod & 0xff_ff00) | u32::from(value);
                    state.tod_halted = false;
                }
            }
            0x9 => {
                if state.crb & CRB_ALARM != 0 {
                    state.tod_alarm = (state.tod_alarm & 0xff_00ff) | (u32::from(value) << 8);
                } else {
                    state.tod = (state.tod & 0xff_00ff) | (u32::from(value) << 8);
                }
            }
            0xa => {
                if state.crb & CRB_ALARM != 0 {
                    state.tod_alarm = (state.tod_alarm & 0x00_ffff) | (u32::from(value) << 16);
                } else {
                    state.tod = (state.tod & 0x00_ffff) | (u32::from(value) << 16);
                    state.tod_halted = true;
                }
            }
            0xb => {}
            0xc => {
                state.sdr = value;
                if state.cra & CRA_SPMODE != 0 {
                    state.sr_buffer = value;
                    state.sr_pending = true;
                    if !state.sr_active {
                        state.start_shift();
                    }
                }
            }
            // "Bit 7 of the data written determines whether the mask bits
            // written are set or cleared."
            0xd => {
                if value & ICR_IR != 0 {
                    state.icr_mask |= value & ICR_SOURCES;
                } else {
                    state.icr_mask &= !(value & ICR_SOURCES);
                }
            }
            0xe => {
                let was = state.cra;
                state.cra = value & !CR_LOAD;
                if value & CR_LOAD != 0 {
                    state.ta = state.ta_latch;
                }
                // "In toggle mode the output is set high when the timer is
                // started", which is the transition rather than the level.
                if value & CR_START != 0 && was & CR_START == 0 && value & CR_OUTMODE != 0 {
                    state.pb6 = true;
                }
                if (was ^ value) & CRA_SPMODE != 0 {
                    // Turning the shift register round abandons the byte in
                    // progress; the direction decides what the pins even are.
                    state.sr_bits = 0;
                    state.sr_active = false;
                    state.sr_pending = false;
                    state.sr_half = false;
                    state.cnt_out = true;
                }
            }
            _ => {
                let was = state.crb;
                state.crb = value & !CR_LOAD;
                if value & CR_LOAD != 0 {
                    state.tb = state.tb_latch;
                }
                if value & CR_START != 0 && was & CR_START == 0 && value & CR_OUTMODE != 0 {
                    state.pb7 = true;
                }
            }
        }
        self.shared.publish(&state);
    }
}

impl MemOps for CiaRegs {
    fn read(&self, offset: u64, dst: &mut [u8], attrs: MemAttrs) -> MemResult {
        let [byte] = dst else {
            return Err(BusError::BadAccess);
        };
        // First, and outside every lock this device owns: a counter read has to
        // see the count at the cycle it happened on.
        self.shared.sync(attrs.debug);
        *byte = self.read_register((offset & 0xf) as u8, attrs.debug);
        if !attrs.debug {
            self.shared.publish(&self.shared.state.lock());
            self.shared.refresh();
        }
        Ok(())
    }

    fn write(&self, offset: u64, src: &[u8], attrs: MemAttrs) -> MemResult {
        let [value] = src else {
            return Err(BusError::BadAccess);
        };
        if attrs.debug {
            // A debug write would start a timer, clear a mask bit or strobe a
            // handshake line (`ROADMAP.md` §15, invariant 5).
            return Err(BusError::BadAccess);
        }
        // A write is as time-sensitive as a read: starting a timer starts it
        // from the cycle of the write.
        self.shared.sync(false);
        self.write_register((offset & 0xf) as u8, *value);
        self.shared.refresh();
        Ok(())
    }

    fn constraints(&self) -> AccessConstraints {
        // One byte lane. On an Amiga a word access at the base reaches *two*
        // CIAs, one per lane, and splitting it is the board's decode rather
        // than anything this chip could answer.
        AccessConstraints::word(Width::U8, Endian::Little)
    }
}

// ---------------------------------------------------------------------------
// input pins
// ---------------------------------------------------------------------------

/// One input pin, as something a wire can drive.
///
/// Keeps a [`FanIn`] and wire-ORs its sources, because a wire hands each sink
/// the level of the *driver that changed* rather than the resolved level of the
/// net (`ROADMAP.md` §4.3).
///
/// It holds the device's shared state rather than the [`Cia`] itself: the device
/// owns the pin, and a pin that owned the device back would be a reference cycle
/// nothing could drop.
#[derive(Debug)]
pub struct CiaPin {
    shared: Arc<Shared>,
    line: u32,
    inputs: FanIn,
}

impl CiaPin {
    fn new(shared: Arc<Shared>, line: u32, sources: &[WireId]) -> CiaPin {
        CiaPin {
            shared,
            line,
            inputs: FanIn::new(sources),
        }
    }

    /// Which of the device's input lines this is.
    #[must_use]
    pub fn line(&self) -> u32 {
        self.line
    }

    /// The per-source levels currently seen.
    #[must_use]
    pub fn inputs(&self) -> &FanIn {
        &self.inputs
    }
}

impl WireSink for CiaPin {
    fn set_level(&self, src: WireId, _line: u32, level: Level) {
        self.inputs.set(src, level);
        let high = self.inputs.resolve(Resolve::Or).is_high();
        self.shared.edge(self.line, high);
    }
}

// ---------------------------------------------------------------------------
// the device
// ---------------------------------------------------------------------------

impl Device for Cia {
    fn class(&self) -> &'static DeviceClass {
        &CIA_CLASS
    }

    fn realize(&self, _ctx: &mut RealizeCtx<'_>) -> Result<()> {
        // Nothing outward: a `map` statement places the region.
        Ok(())
    }

    fn reset(&self, _kind: ResetKind) {
        let mut state = self.shared.state.lock();
        // The levels other devices are driving onto the pins are theirs, not
        // ours, and survive our reset (`ROADMAP.md` §4.5).
        let (pa_in, pb_in, cnt_in, sp_in, flag_in, tod_in) = (
            state.pa_in,
            state.pb_in,
            state.cnt_in,
            state.sp_in,
            state.flag_in,
            state.tod_in,
        );
        *state = State::fresh(state.ticks);
        state.pa_in = pa_in;
        state.pb_in = pb_in;
        state.cnt_in = cnt_in;
        state.sp_in = sp_in;
        state.flag_in = flag_in;
        state.tod_in = tod_in;
        self.shared.publish(&state);
        drop(state);
        self.shared.refresh();
    }

    fn save(&self, w: &mut ChunkWriter<'_>) -> Result<()> {
        let state = *self.shared.state.lock();
        w.write_u64(state.ticks)?;
        w.write_u8(state.pra)?;
        w.write_u8(state.prb)?;
        w.write_u8(state.ddra)?;
        w.write_u8(state.ddrb)?;
        w.write_u16(state.ta)?;
        w.write_u16(state.tb)?;
        w.write_u16(state.ta_latch)?;
        w.write_u16(state.tb_latch)?;
        w.write_u8(state.cra)?;
        w.write_u8(state.crb)?;
        w.write_bool(state.pb6)?;
        w.write_bool(state.pb7)?;
        w.write_u64(state.ta_underflow_at)?;
        w.write_u64(state.tb_underflow_at)?;
        w.write_u32(state.tod)?;
        w.write_u32(state.tod_alarm)?;
        w.write_u32(state.tod_latch)?;
        w.write_bool(state.tod_latched)?;
        w.write_bool(state.tod_halted)?;
        w.write_u8(state.sdr)?;
        w.write_u8(state.sr_shift)?;
        w.write_u8(state.sr_bits)?;
        w.write_u8(state.sr_buffer)?;
        w.write_bool(state.sr_pending)?;
        w.write_bool(state.sr_active)?;
        w.write_bool(state.sr_half)?;
        w.write_bool(state.sp_out)?;
        w.write_bool(state.cnt_out)?;
        w.write_u8(state.icr_data)?;
        w.write_u8(state.icr_mask)?;
        w.write_u64(state.pc_until)
        // The input levels are deliberately absent: `pa_in`, `pb_in`, `cnt_in`,
        // `sp_in`, `flag_in` and `tod_in` are what *other* devices are driving,
        // and each will restore its own state and drive them again (§4.5).
    }

    fn load(&self, r: &mut ChunkReader<'_>) -> Result<()> {
        let mut state = self.shared.state.lock();
        let keep = *state;
        let mut next = State::fresh(r.read_u64()?);
        next.pra = r.read_u8()?;
        next.prb = r.read_u8()?;
        next.ddra = r.read_u8()?;
        next.ddrb = r.read_u8()?;
        next.ta = r.read_u16()?;
        next.tb = r.read_u16()?;
        next.ta_latch = r.read_u16()?;
        next.tb_latch = r.read_u16()?;
        next.cra = r.read_u8()?;
        next.crb = r.read_u8()?;
        next.pb6 = r.read_bool()?;
        next.pb7 = r.read_bool()?;
        next.ta_underflow_at = r.read_u64()?;
        next.tb_underflow_at = r.read_u64()?;
        next.tod = r.read_u32()? & TOD_MASK;
        next.tod_alarm = r.read_u32()? & TOD_MASK;
        next.tod_latch = r.read_u32()? & TOD_MASK;
        next.tod_latched = r.read_bool()?;
        next.tod_halted = r.read_bool()?;
        next.sdr = r.read_u8()?;
        next.sr_shift = r.read_u8()?;
        next.sr_bits = r.read_u8()? % 8;
        next.sr_buffer = r.read_u8()?;
        next.sr_pending = r.read_bool()?;
        next.sr_active = r.read_bool()?;
        next.sr_half = r.read_bool()?;
        next.sp_out = r.read_bool()?;
        next.cnt_out = r.read_bool()?;
        next.icr_data = r.read_u8()? & ICR_SOURCES;
        next.icr_mask = r.read_u8()? & ICR_SOURCES;
        next.pc_until = r.read_u64()?;
        next.pa_in = keep.pa_in;
        next.pb_in = keep.pb_in;
        next.cnt_in = keep.cnt_in;
        next.sp_in = keep.sp_in;
        next.flag_in = keep.flag_in;
        next.tod_in = keep.tod_in;
        *state = next;
        self.shared.publish(&state);
        drop(state);
        self.shared.refresh();
        Ok(())
    }

    fn region(&self, name: &str) -> Option<RegionRef> {
        matches!(name, "" | "regs").then(|| Arc::clone(&self.region))
    }

    fn connect(&self, port: &str, source: WireSource) -> Result<()> {
        self.connect_pin(port, source)
    }

    fn announce(&self, _port: &str) {
        self.shared.refresh();
    }

    fn sink(&self, port: &str, sources: &[WireId]) -> Option<SinkPin> {
        let line = match port {
            TOD_PIN => LINE_TOD,
            CNT_PIN => LINE_CNT,
            SP_PIN => LINE_SP,
            FLAG_PIN => LINE_FLAG,
            _ => {
                if let Some(n) = port_index(port, PORT_A_PREFIX, PORT_PINS) {
                    LINE_PA + n
                } else {
                    LINE_PB + port_index(port, PORT_B_PREFIX, PORT_PINS)?
                }
            }
        };
        let pin = Arc::new(CiaPin::new(Arc::clone(&self.shared), line, sources));
        self.pins.lock().push(Arc::clone(&pin));
        Some(SinkPin { sink: pin, line })
    }

    // -- lazily advanced (`ROADMAP.md` §4.2) ---------------------------------

    /// Yes. A timer counter read has to report the count at the cycle of the
    /// read, and an underflow has to reach the CPU on the cycle it happens.
    fn is_lazy(&self) -> bool {
        true
    }

    fn current_tick(&self) -> u64 {
        self.shared.ticks.load(Ordering::Relaxed)
    }

    fn advance_to(&self, tick: u64) {
        Cia::advance_to(self, tick);
    }

    fn next_event_tick(&self) -> Option<u64> {
        match self.shared.next_event.load(Ordering::Relaxed) {
            NO_EVENT => None,
            tick => Some(tick),
        }
    }

    fn attach_lazy(&self, handle: LazyHandle) {
        Cia::attach_lazy(self, handle);
    }
}

impl Instance for Cia {}

/// The `mos.8520` device class.
pub static CIA_CLASS: DeviceClass = DeviceClass {
    name: CLASS_NAME,
    version: STATE_VERSION,
    summary: "MOS 8520 CIA: two ports, two timers, a 24-bit TOD counter and a shift register",
    properties: &[],
    construct: |props| Ok(Box::new(Cia::new(props)?)),
};

/// Add [`CIA_CLASS`] to a registry.
///
/// # Errors
///
/// [`Error::Config`] if something already claimed the name.
pub fn register(registry: &mut crate::core::Registry) -> Result<()> {
    registry.add(&CIA_CLASS)
}

/// Bind [`CIA_CLASS`] into the machine graph.
///
/// # Errors
///
/// [`Error::Config`] if the class is already bound.
pub fn bind(bindings: &mut crate::machine::Bindings) -> Result<()> {
    bindings.bind(CLASS_NAME, |props| Ok(Arc::new(Cia::new(props)?)))
}

/// What the validator should know about `mos.8520`.
#[must_use]
pub fn schema() -> crate::machine::validate::ClassSchema {
    use crate::machine::validate::{ClassSchema, PortDir};
    ClassSchema::new(CLASS_NAME)
        .port(IRQ_PIN, PortDir::Out)
        .port(PC_PIN, PortDir::Out)
        .port(SP_PIN, PortDir::InOut)
        .port(CNT_PIN, PortDir::InOut)
        .port(FLAG_PIN, PortDir::In)
        .port(TOD_PIN, PortDir::In)
        .port_bank(PORT_A_PREFIX, PortDir::InOut, PORT_PINS)
        .port_bank(PORT_B_PREFIX, PortDir::InOut, PORT_PINS)
        .region("")
        .region("regs")
}

#[cfg(test)]
mod tests;
