//! The Integrated Woz Machine and the 400K/800K drive on the end of its
//! cable.
//!
//! # Sources
//!
//! * Apple's *IWM Specification* (Apple Computer, 1982), for the sixteen soft
//!   switches, the four register pairs `Q6`/`Q7` select between, the mode
//!   register's bits and the write handshake.
//! * *Guide to the Macintosh Family Hardware*, 2nd edition, "Disk Interface"
//!   — where the chip is decoded, and the drive's own register file: an
//!   address made of `CA0`, `CA1`, `CA2` and the `SEL` line the VIA drives,
//!   sixteen readable status lines and four writable controls.
//!
//! No emulator source was consulted and no ROM was disassembled
//! (`ROADMAP.md` §1, `CLAUDE.md`); where the documents were ambiguous the
//! answer came from watching which addresses a real ROM touches and what it
//! waits for.
//!
//! # The decode, and why every access is a switch
//!
//! The IWM has no address pins in the ordinary sense. Its sixteen addresses
//! are **soft switches**: the low bit of the address number sets or clears one
//! internal line, and it does so whether the access was a read or a write.
//! Reading `$DFE1FF + 2 * 512` clears `CA1`; reading `$DFE1FF + 3 * 512` sets
//! it. A ROM therefore drives this chip almost entirely with *reads*, which is
//! exactly what a trace of one shows.
//!
//! ```text
//!   0/1   CA0     off / on
//!   2/3   CA1     off / on
//!   4/5   CA2     off / on
//!   6/7   LSTRB   off / on     the strobe that writes a drive register
//!   8/9   ENABLE  off / on     the drive motor
//!   10/11 SELECT  drive 1 / 2
//!   12/13 Q6      off / on
//!   14/15 Q7      off / on
//! ```
//!
//! and `Q7:Q6` then choose what a read of *any* of those sixteen addresses
//! returns:
//!
//! ```text
//!   0 0   the data register — what the head has shifted in
//!   0 1   the status register
//!   1 0   the write handshake
//!   1 1   nothing (a write here loads the mode register)
//! ```
//!
//! A Macintosh puts the chip's register selects on **A9-A12** like the VIA's,
//! so its sixteen addresses are 512 bytes apart, the block repeats every
//! 8 KiB, and the published base is `$DFE1FF` — an *odd* address, because the
//! IWM sits on the low byte lane where the VIA sits on the high one.
//!
//! # The drive's own registers
//!
//! The drive is addressed by `CA2:CA1:CA0` **and the `SEL` line**, which comes
//! from the VIA's `PA5` rather than from the IWM: four bits, sixteen readable
//! status lines, whose selected value appears as bit 7 of the IWM's status
//! register (`SENSE`). Writing is four registers addressed by `CA1:CA0` with
//! the data on `CA2`, latched when `LSTRB` goes high:
//!
//! ```text
//!   CA1 CA0  CA2=0            CA2=1
//!    0   0   step toward 79   step toward 0
//!    0   1   issue one step   (no-op)
//!    1   0   motor on         motor off
//!    1   1   eject            (no-op)
//! ```
//!
//! # What is modelled, and what is not
//!
//! The switches, the mode and status registers, the write handshake, the
//! drive's status lines, stepping, the motor, and a disk that can be present,
//! absent or write protected. **The data path is not here**: `RDDATA` reads as
//! a line that never changes, so a guest sees a drive that spins and steps and
//! never delivers a sector. That is enough for a ROM to find no bootable disk
//! and say so, and it is what `docs/platforms/mac-plus.md` records as the next
//! thing to build.

use alloc::boxed::Box;
use alloc::sync::Arc;
use alloc::vec::Vec;

use crate::core::device::{Device, DeviceClass, PropertySpec, RealizeCtx, ResetKind, SinkPin};
use crate::core::error::{BusError, Error, Result};
use crate::core::props::{Props, ValueKind};
use crate::core::space::{AccessConstraints, MemAttrs, MemOps, MemResult, Region, RegionRef};
use crate::core::state::{ChunkReader, ChunkWriter, Sink, Source};
use crate::core::sync::{LockRank, Mutex};
use crate::core::value::{Endian, Width};
use crate::core::wire::{FanIn, Level, Resolve, WireId, WireSink};
use crate::machine::realize::Instance;
use crate::machine::validate::{ClassSchema, PortDir, PropSchema};

/// The class name a machine description writes.
pub const CLASS_NAME: &str = "mac.iwm";

/// The snapshot chunk version. Bump with the encoding, never on its own.
pub const STATE_VERSION: u32 = 1;

/// How many bytes of address space the sixteen switches occupy: the board puts
/// the register selects on A9-A12, so `16 * 512`.
pub const REGISTER_SPAN: u64 = 0x2000;

/// How far apart two switches are, in bytes.
const REGISTER_STRIDE: u64 = 0x200;

/// The input pin the VIA's `PA5` drives: the drive register file's fourth
/// address bit.
const SEL_PIN: &str = "sel";

/// The highest track a 400K/800K mechanism can step to.
pub const MAX_TRACK: u8 = 79;

// -- the soft switches -------------------------------------------------------

const SW_CA0: u8 = 1 << 0;
const SW_CA1: u8 = 1 << 1;
const SW_CA2: u8 = 1 << 2;
const SW_LSTRB: u8 = 1 << 3;
const SW_ENABLE: u8 = 1 << 4;
const SW_SELECT: u8 = 1 << 5;
const SW_Q6: u8 = 1 << 6;
const SW_Q7: u8 = 1 << 7;

/// The status register's bit 5: the drive enable line, as software reads it.
const STATUS_ENABLE: u8 = 1 << 5;
/// Its bit 7: the selected drive status line.
const STATUS_SENSE: u8 = 1 << 7;
/// The write handshake's bit 6: no underrun has happened.
const HANDSHAKE_NO_UNDERRUN: u8 = 1 << 6;
/// Its bit 7: the write buffer will take another byte.
const HANDSHAKE_READY: u8 = 1 << 7;

/// One 400K/800K mechanism.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct Mechanism {
    /// Whether a drive is plugged in at all. An external port with nothing on
    /// it has this clear, and `/DRVIN` then reads high.
    installed: bool,
    /// Whether a disk is in it.
    disk: bool,
    /// Whether that disk is write protected.
    write_protect: bool,
    /// Whether it is a double-sided (800K) mechanism.
    double_sided: bool,
    /// Which cylinder the head is over, 0 to [`MAX_TRACK`].
    track: u8,
    /// The step direction the last write to register 0 set: `false` steps
    /// toward track 79, `true` toward track 0.
    outward: bool,
    /// Whether the motor is running.
    motor: bool,
    /// The "a disk has been swapped" line, set when a disk appears or goes and
    /// cleared by the drive's own reset register.
    switched: bool,
}

impl Mechanism {
    fn fresh(installed: bool) -> Mechanism {
        Mechanism {
            installed,
            disk: false,
            write_protect: false,
            double_sided: true,
            track: 0,
            outward: false,
            motor: false,
            switched: false,
        }
    }

    /// The status line `addr` selects, where `addr` is `CA2:CA1:CA0:SEL`.
    ///
    /// Every line is read *asserted low* except where the Guide's table names
    /// it without a bar, which is why so many of these are negations. A drive
    /// that is not installed lets go of the cable and every line reads as the
    /// pull-up, which is `true`.
    fn sense(&self, addr: u8) -> bool {
        if !self.installed {
            return true;
        }
        match addr {
            // DIRTN: which way a step will go.
            0 => self.outward,
            // /CSTIN: low while a disk is in place.
            1 => !self.disk,
            // /STEP: high once the step this model completed instantly is done.
            2 => true,
            // /WRTPRT: low while the disk is write protected.
            3 => !(self.disk && self.write_protect),
            // MOTORON: low while the motor is running.
            4 => !self.motor,
            // /TK0: low while the head is over track 0.
            5 => self.track != 0,
            // SWITCHED: high once a disk has been changed.
            6 => self.switched,
            // TACH: the tachometer. A line that never moves is a drive that
            // reports no rotation, which is what a stopped motor looks like.
            7 => false,
            // RDDATA0 and RDDATA1, the two heads. No data path yet.
            8 | 9 => false,
            // SIDES: high on a double-sided mechanism.
            10 => self.double_sided,
            // /READY: low once the motor is up to speed, which here is as soon
            // as it is running with a disk in place.
            11 => !(self.motor && self.disk),
            // /DRVIN: low while a drive is connected.
            12 => false,
            // The Guide leaves 13, 14 and 15 unassigned on this mechanism.
            _ => true,
        }
    }

    /// Apply the write register `CA1:CA0` addresses, with `CA2` as its data.
    fn control(&mut self, addr: u8, data: bool) {
        if !self.installed {
            return;
        }
        match addr {
            // Step direction.
            0 => self.outward = data,
            // One step, on the zero.
            1 => {
                if !data {
                    if self.outward {
                        self.track = self.track.saturating_sub(1);
                    } else if self.track < MAX_TRACK {
                        self.track += 1;
                    }
                }
            }
            // The motor: on for a zero, off for a one.
            2 => self.motor = !data,
            // Eject, on the zero.
            _ => {
                if !data && self.disk {
                    self.disk = false;
                    self.switched = true;
                }
            }
        }
    }
}

/// Everything the chip owns.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct State {
    /// The eight soft switches, one per bit — see the `SW_*` constants.
    switches: u8,
    /// The mode register, loaded by a write while `Q7:Q6` is `11`.
    mode: u8,
    /// The data register. Nothing fills it while there is no data path.
    data: u8,
    /// The two mechanisms the chip's `SELECT` line picks between.
    drives: [Mechanism; 2],
    /// `SEL`, which the VIA drives and the drive register file uses as its
    /// fourth address bit.
    ///
    /// Snapshotted, for the reason `mac.via` gives about its own pins: a level
    /// that came back wrong turns the realize sweep at the end of a restore
    /// into a change.
    sel: bool,
}

impl State {
    fn fresh(installed: [bool; 2]) -> State {
        State {
            switches: 0,
            mode: 0,
            data: 0,
            drives: [
                Mechanism::fresh(installed[0]),
                Mechanism::fresh(installed[1]),
            ],
            sel: true,
        }
    }

    /// Which mechanism `SELECT` is pointing at.
    fn selected(&self) -> usize {
        usize::from(self.switches & SW_SELECT != 0)
    }

    /// The drive register address: `CA2:CA1:CA0:SEL`.
    fn drive_address(&self) -> u8 {
        let s = self.switches;
        (u8::from(s & SW_CA2 != 0) << 3)
            | (u8::from(s & SW_CA1 != 0) << 2)
            | (u8::from(s & SW_CA0 != 0) << 1)
            | u8::from(self.sel)
    }

    /// The status register as software reads it.
    ///
    /// The IWM specification: bits 0-4 mirror the mode register, bit 5 is the
    /// drive enable, bit 6 reads zero, and bit 7 is the selected drive's
    /// status line.
    fn status(&self) -> u8 {
        let mut value = self.mode & 0x1f;
        if self.switches & SW_ENABLE != 0 {
            value |= STATUS_ENABLE;
        }
        // `SENSE` is the drive's line, and the drive's lines are asserted low;
        // the bit reads the *pin*, so a line at rest reads one.
        if self.drives[self.selected()].sense(self.drive_address()) {
            value |= STATUS_SENSE;
        }
        value
    }

    /// What a read of any of the sixteen addresses returns now.
    fn read_value(&self) -> u8 {
        match (self.switches & SW_Q7 != 0, self.switches & SW_Q6 != 0) {
            (false, false) => self.data,
            (false, true) => self.status(),
            // The write handshake: always ready, never underrun, because
            // nothing here ever takes time over a byte.
            (true, false) => HANDSHAKE_READY | HANDSHAKE_NO_UNDERRUN,
            // Reading the mode register is not a thing the chip does.
            (true, true) => 0xff,
        }
    }

    /// Set the soft switch `index` selects, and apply whatever that changed.
    fn switch(&mut self, index: u8) {
        let bit = 1u8 << (index >> 1);
        let on = index & 1 != 0;
        let before = self.switches;
        if on {
            self.switches |= bit;
        } else {
            self.switches &= !bit;
        }
        // `LSTRB` going high is what latches a drive control register; the
        // address and the data are the `CA` lines as they stand at that moment.
        if bit == SW_LSTRB && on && before & SW_LSTRB == 0 {
            let s = self.switches;
            let addr = (u8::from(s & SW_CA1 != 0) << 1) | u8::from(s & SW_CA0 != 0);
            let data = s & SW_CA2 != 0;
            let which = self.selected();
            self.drives[which].control(addr, data);
        }
    }
}

/// The chip, as something an address space can dispatch to.
struct Shared {
    state: Mutex<State>,
}

impl core::fmt::Debug for Shared {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        let mut s = f.debug_struct("Iwm");
        match self.state.try_lock() {
            Some(state) => s.field("state", &*state).finish(),
            None => s.field("state", &"<in use>").finish(),
        }
    }
}

impl MemOps for Shared {
    fn read(&self, offset: u64, dst: &mut [u8], attrs: MemAttrs) -> MemResult {
        let [byte] = dst else {
            return Err(BusError::BadAccess);
        };
        let index = ((offset / REGISTER_STRIDE) & 0xf) as u8;
        let mut state = self.state.lock();
        if attrs.debug {
            // A debugger must be able to look without moving a switch, and
            // every address here moves one. So a debug read answers from the
            // state as it stands and changes nothing (invariant 5).
            *byte = state.read_value();
            return Ok(());
        }
        state.switch(index);
        *byte = state.read_value();
        Ok(())
    }

    fn write(&self, offset: u64, src: &[u8], attrs: MemAttrs) -> MemResult {
        let [value] = src else {
            return Err(BusError::BadAccess);
        };
        if attrs.debug {
            // Every address is a switch, so there is no harmless debug write.
            return Err(BusError::BadAccess);
        }
        let index = ((offset / REGISTER_STRIDE) & 0xf) as u8;
        let mut state = self.state.lock();
        state.switch(index);
        match (state.switches & SW_Q7 != 0, state.switches & SW_Q6 != 0) {
            // The mode register.
            (true, true) => state.mode = *value & 0x7f,
            // The data register, which starts a write to the disk. There is no
            // data path yet, so the byte is kept and goes nowhere.
            (true, false) => state.data = *value,
            _ => {}
        }
        Ok(())
    }

    fn constraints(&self) -> AccessConstraints {
        // An eight-bit part on the low lane of the 68000's word bus.
        AccessConstraints::word(Width::U8, Endian::Big)
    }
}

/// The `SEL` input.
#[derive(Debug)]
struct SelPin {
    shared: Arc<Shared>,
    inputs: FanIn,
}

impl WireSink for SelPin {
    fn set_level(&self, src: WireId, _line: u32, level: Level) {
        self.inputs.set(src, level);
        let high = self.inputs.resolve(Resolve::And).is_high();
        self.shared.state.lock().sel = high;
    }
}

/// An Integrated Woz Machine and the mechanisms on its cable.
#[derive(Debug)]
pub struct Iwm {
    shared: Arc<Shared>,
    region: RegionRef,
    /// Which of the two cable positions has a drive on it, for reset.
    installed: [bool; 2],
    /// The pin, kept alive here: a net holds only a `Weak` to its sinks.
    pins: Mutex<Vec<Arc<SelPin>>>,
}

impl Iwm {
    /// Build the chip.
    ///
    /// `drives` says how many mechanisms are on the cable: 1 for a Macintosh
    /// Plus with only its internal drive, 2 with an external one plugged in.
    ///
    /// # Errors
    ///
    /// [`Error::Property`] if `drives` is not 1 or 2, or if a property this
    /// class does not know was given.
    pub fn new(props: &Props) -> Result<Iwm> {
        let mut r = props.reader();
        let drives = r.or("drives", 1u64)?;
        r.finish()?;
        if drives == 0 || drives > 2 {
            return Err(Error::Property(alloc::format!(
                "property `drives`: an IWM's cable takes one or two mechanisms, not {drives}"
            )));
        }
        Ok(Iwm::with_drives([true, drives == 2]))
    }

    /// The same, saying exactly which cable positions are occupied.
    #[must_use]
    pub fn with_drives(installed: [bool; 2]) -> Iwm {
        let shared = Arc::new(Shared {
            state: Mutex::with_rank(LockRank::DEVICE, State::fresh(installed)),
        });
        let region = Arc::new(Region::io(
            CLASS_NAME,
            REGISTER_SPAN,
            Arc::clone(&shared) as Arc<dyn MemOps>,
        ));
        Iwm {
            shared,
            region,
            installed,
            pins: Mutex::with_rank(LockRank::LEAF, Vec::new()),
        }
    }

    /// Read a switch the way the address space would, for a test.
    #[must_use]
    pub fn peek(&self, index: u8) -> u8 {
        let mut byte = [0u8; 1];
        let _ = self.shared.read(
            u64::from(index & 0xf) * REGISTER_STRIDE,
            &mut byte,
            MemAttrs::DEFAULT,
        );
        byte[0]
    }

    /// Write one the same way.
    pub fn poke(&self, index: u8, value: u8) {
        let _ = self.shared.write(
            u64::from(index & 0xf) * REGISTER_STRIDE,
            &[value],
            MemAttrs::DEFAULT,
        );
    }

    /// Set the level the VIA is driving onto `SEL`, for a test with no wire
    /// graph.
    pub fn set_sel(&self, high: bool) {
        self.shared.state.lock().sel = high;
    }

    /// Whether the motor of drive `which` is running.
    #[must_use]
    pub fn motor(&self, which: usize) -> bool {
        self.shared.state.lock().drives[which & 1].motor
    }

    /// Which cylinder drive `which`'s head is over.
    #[must_use]
    pub fn track(&self, which: usize) -> u8 {
        self.shared.state.lock().drives[which & 1].track
    }

    /// Whether drive `which` has a disk in it.
    #[must_use]
    pub fn has_disk(&self, which: usize) -> bool {
        self.shared.state.lock().drives[which & 1].disk
    }

    /// Put a disk in drive `which`, or take one out.
    pub fn set_disk(&self, which: usize, present: bool, write_protect: bool) {
        let mut state = self.shared.state.lock();
        let drive = &mut state.drives[which & 1];
        if drive.disk != present {
            drive.switched = true;
        }
        drive.disk = present;
        drive.write_protect = write_protect;
    }

    /// The soft switches, for a test that wants to see what a ROM set.
    #[must_use]
    pub fn switches(&self) -> u8 {
        self.shared.state.lock().switches
    }

    /// The mode register the guest last wrote.
    #[must_use]
    pub fn mode(&self) -> u8 {
        self.shared.state.lock().mode
    }
}

impl Device for Iwm {
    fn class(&self) -> &'static DeviceClass {
        &IWM_CLASS
    }

    fn realize(&self, _ctx: &mut RealizeCtx<'_>) -> Result<()> {
        // Nothing outward: a `map` statement places the region and the wire
        // graph brings `SEL`.
        Ok(())
    }

    fn reset(&self, _kind: ResetKind) {
        let mut state = self.shared.state.lock();
        let sel = state.sel;
        // A disk stays in the drive across a reset: it is a thing in a slot,
        // not a register. The chip's switches and the mechanism's motor and
        // head position do come back.
        let disks = [
            (state.drives[0].disk, state.drives[0].write_protect),
            (state.drives[1].disk, state.drives[1].write_protect),
        ];
        *state = State::fresh(self.installed);
        state.sel = sel;
        for (drive, (disk, wp)) in state.drives.iter_mut().zip(disks) {
            drive.disk = disk;
            drive.write_protect = wp;
        }
    }

    fn save(&self, w: &mut ChunkWriter<'_>) -> Result<()> {
        let state = *self.shared.state.lock();
        w.write_u8(state.switches)?;
        w.write_u8(state.mode)?;
        w.write_u8(state.data)?;
        for drive in &state.drives {
            w.write_bool(drive.installed)?;
            w.write_bool(drive.disk)?;
            w.write_bool(drive.write_protect)?;
            w.write_bool(drive.double_sided)?;
            w.write_u8(drive.track)?;
            w.write_bool(drive.outward)?;
            w.write_bool(drive.motor)?;
            w.write_bool(drive.switched)?;
        }
        w.write_bool(state.sel)
    }

    fn load(&self, r: &mut ChunkReader<'_>) -> Result<()> {
        let mut state = self.shared.state.lock();
        let mut next = State::fresh(self.installed);
        next.switches = r.read_u8()?;
        next.mode = r.read_u8()? & 0x7f;
        next.data = r.read_u8()?;
        for drive in &mut next.drives {
            drive.installed = r.read_bool()?;
            drive.disk = r.read_bool()?;
            drive.write_protect = r.read_bool()?;
            drive.double_sided = r.read_bool()?;
            drive.track = r.read_u8()?.min(MAX_TRACK);
            drive.outward = r.read_bool()?;
            drive.motor = r.read_bool()?;
            drive.switched = r.read_bool()?;
        }
        next.sel = r.read_bool()?;
        *state = next;
        Ok(())
    }

    fn region(&self, name: &str) -> Option<RegionRef> {
        matches!(name, "" | "regs").then(|| Arc::clone(&self.region))
    }

    fn sink(&self, port: &str, sources: &[WireId]) -> Option<SinkPin> {
        if port != SEL_PIN {
            return None;
        }
        let pin = Arc::new(SelPin {
            shared: Arc::clone(&self.shared),
            inputs: FanIn::new(sources),
        });
        self.pins.lock().push(Arc::clone(&pin));
        Some(SinkPin { sink: pin, line: 0 })
    }
}

impl Instance for Iwm {}

/// The `mac.iwm` device class.
pub static IWM_CLASS: DeviceClass = DeviceClass {
    name: CLASS_NAME,
    version: STATE_VERSION,
    summary: "an Integrated Woz Machine: sixteen soft switches, and the 400K/800K drive on its \
              cable",
    properties: &[PropertySpec {
        name: "drives",
        kind: ValueKind::Uint,
        required: false,
        summary: "how many mechanisms are on the cable, 1 or 2 (default 1)",
    }],
    construct: |props| Ok(Box::new(Iwm::new(props)?)),
};

/// Add [`IWM_CLASS`] to a registry.
///
/// # Errors
///
/// [`Error::Config`] if something already claimed the name.
pub fn register(registry: &mut crate::core::Registry) -> Result<()> {
    registry.add(&IWM_CLASS)
}

/// Bind [`IWM_CLASS`] into the machine graph.
///
/// # Errors
///
/// [`Error::Config`] if the class is already bound.
pub fn bind(bindings: &mut crate::machine::Bindings) -> Result<()> {
    bindings.bind(CLASS_NAME, |props| Ok(Arc::new(Iwm::new(props)?)))
}

/// What the validator should know about `mac.iwm`.
#[must_use]
pub fn schema() -> ClassSchema {
    ClassSchema::new(CLASS_NAME)
        .prop(PropSchema::new("drives", ValueKind::Uint).range(1, 2))
        .region("")
        .region("regs")
        .port(SEL_PIN, PortDir::In)
}

#[cfg(test)]
mod tests;
