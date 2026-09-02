//! A NEC µPD765A floppy disk controller, as the PC/AT wires it.
//!
//! # Sources
//!
//! * *NEC µPD765A/µPD765B Single/Double Density Floppy Disk Controller* data
//!   sheet. The three phases, the main status register bits, the command set
//!   with its parameter and result byte orders, the four status registers, and
//!   the multi-sector/multi-track termination rules all come from it.
//! * *IBM Personal Computer AT Technical Reference* (1984), the diskette
//!   adapter section: the eight ports at `0x3f0-0x3f7`, the digital output
//!   register — which is a board latch, not part of the chip — the digital
//!   input register's disk-change bit, and the configuration control register.
//! * Ralf Brown's Interrupt List, ports section, for the register-level
//!   behaviour the data sheet leaves to the board.
//!
//! **No emulator source was consulted** (`CLAUDE.md`, provenance). The µPD765
//! is a 1979 part and its data sheet is the primary source for every number
//! here.
//!
//! # The register block
//!
//! ```text
//!   0  --   not decoded by the AT's adapter
//!   1  --   not decoded
//!   2  DOR  digital output register (write): drive select, /RESET, the DMA and
//!           interrupt gate, four motor enables. A board latch, not a chip
//!           register.
//!   3  --   not decoded
//!   4  MSR  main status register (read): RQM, DIO, NDMA, CB, and one seek bit
//!           per drive. Polled constantly; it is the whole handshake.
//!   5  FIFO data register (read and write): parameters in, results out.
//!   6  --   not decoded
//!   7  DIR  digital input register (read): bit 7 is DSKCHG.
//!      CCR  configuration control register (write): the data rate.
//! ```
//!
//! # The three phases
//!
//! Every operation is a **command phase** (the CPU writes an opcode and its
//! parameters), an optional **execution phase** (data moves, by DMA or by the
//! CPU polling the data register), and a **result phase** (the CPU reads result
//! bytes until `RQM`/`DIO` say there are none left). `Phase` is that state
//! machine, made explicit because the main status register is a pure function
//! of it: firmware that reads a wrong `RQM` does not fail, it hangs.
//!
//! # Time is not modelled
//!
//! **Seeks and transfers complete instantly.** A real head takes milliseconds
//! to step and a real sector takes hundreds of microseconds to pass under it;
//! here the seek is done, and the interrupt asserted, before the `out` that
//! started it has returned. This is deliberate — a device may not sleep, read a
//! clock, or spawn anything (`CLAUDE.md`, concurrency) — and the consequence is
//! precise: firmware that starts a seek and *then* polls or waits for the
//! interrupt still works, because the interrupt arrives before the poll, while
//! a program that *times* a seek measures zero. Nothing in a PC's boot path
//! does the latter. The alternative — taking a clock domain and posting the
//! completion as a scheduled event — is the other correct choice and is what
//! `ROADMAP.md` §4.2's event queue is for; it is not needed to boot a disk and
//! it is not here.
//!
//! # The medium
//!
//! One image, in memory, in the drive selected as unit 0. Its geometry is
//! inferred from its length by default, because the five standard PC floppy
//! formats have five distinct lengths and a raw image carries nothing else:
//!
//! ```text
//!    368640  40/2/9    360K   double density, 5.25"
//!    737280  80/2/9    720K   double density, 3.5"
//!   1228800  80/2/15   1.2M   high density,   5.25"
//!   1474560  80/2/18   1.44M  high density,   3.5"
//!   2949120  80/2/36   2.88M  extra density,  3.5"
//! ```
//!
//! An **empty or absent image is a drive with no disk in it** — a machine with
//! no floppy is an ordinary machine — and every command that needs a medium
//! answers with `ST0`'s not-ready bit rather than faulting.
//!
//! # Deliberate simplifications
//!
//! * A raw sector image has no ID fields, so a read takes the cylinder from the
//!   command rather than checking it against where the head actually is. Real
//!   hardware answers `ND` when the two disagree; nothing here can tell.
//! * `FORMAT TRACK` consumes its four ID bytes per sector and writes the filler
//!   byte over each named sector's data field. The header it would have written
//!   has nowhere to go.
//! * The motor enables are latched and reported but not enforced: a drive whose
//!   motor is off still answers, because refusing would only break firmware
//!   that forgot a bit no emulated disk needs to spin.

use alloc::boxed::Box;
use alloc::collections::VecDeque;
use alloc::format;
use alloc::string::{String, ToString};
use alloc::sync::Arc;
use alloc::vec::Vec;
use core::fmt;

use crate::core::device::{Device, DeviceClass, PropertySpec, RealizeCtx, ResetKind};
use crate::core::error::{BusError, Error, Result};
use crate::core::props::{Props, ValueKind};
use crate::core::space::{AccessConstraints, MemAttrs, MemOps, MemResult, Region, RegionRef};
use crate::core::state::{ChunkReader, ChunkWriter, Sink, Source};
use crate::core::sync::{LockRank, Mutex};
use crate::core::value::{Endian, Width};
use crate::core::wire::{DmaPeripheral, Level, WireSource};
use crate::machine::realize::Instance;
use crate::machine::validate::{ClassSchema, PortDir, PropSchema};

/// The class name a machine description writes.
pub const CLASS_NAME: &str = "pc.fdc";

/// The snapshot chunk version. Bump with the encoding, never on its own.
const STATE_VERSION: u32 = 1;

/// How much address space the register block answers: `0x3f0-0x3f7`.
pub const REGISTER_WINDOW_LEN: u64 = 8;

/// How many bytes a sector holds. Every PC floppy format uses 512, which is
/// also the only size code (`N = 2`) this model transfers.
pub const SECTOR_LEN: u64 = 512;

/// The `N` code for a 512-byte sector: the size is `128 << N`.
const N_512: u8 = 2;

/// How many drives the chip selects between.
const DRIVES: usize = 4;

// -- register offsets -------------------------------------------------------

/// Digital output register (write only): a board latch ahead of the chip.
const REG_DOR: u64 = 2;
/// Main status register (read only).
const REG_MSR: u64 = 4;
/// The data register: command bytes in, result bytes out.
const REG_DATA: u64 = 5;
/// Digital input register (read) and configuration control register (write).
const REG_DIR_CCR: u64 = 7;

// -- DOR (offset 2) ---------------------------------------------------------

/// Drive select, bits 0-1.
const DOR_DRIVE: u8 = 0x03;
/// Controller reset — **active low**. Clearing it holds the chip in reset, and
/// every BIOS starts by doing exactly that.
const DOR_RESET: u8 = 0x04;
/// Gates `DRQ` and `INT` onto the bus. With it clear the chip still works, but
/// the board delivers neither signal, so software must poll.
const DOR_DMA_INT: u8 = 0x08;

// -- MSR (offset 4) ---------------------------------------------------------

/// Request for master: the data register may be accessed.
const MSR_RQM: u8 = 0x80;
/// Data input/output: set when the controller has a byte *for* the CPU.
const MSR_DIO: u8 = 0x40;
/// Execution phase in non-DMA mode.
const MSR_NDMA: u8 = 0x20;
/// Controller busy: a command is in progress.
const MSR_CB: u8 = 0x10;

// -- ST0 --------------------------------------------------------------------

/// Head address at the end of the command, bit 2.
const ST0_HD: u8 = 0x04;
/// Not ready: the drive has no disk in it, or is not there at all.
const ST0_NR: u8 = 0x08;
/// Equipment check: a recalibrate that never found track 0.
const ST0_EC: u8 = 0x10;
/// Seek end: a seek or recalibrate completed.
const ST0_SE: u8 = 0x20;
/// Interrupt code 01 — abnormal termination.
const ST0_ABNORMAL: u8 = 0x40;
/// Interrupt code 10 — an invalid command.
const ST0_INVALID: u8 = 0x80;
/// Interrupt code 11 — the ready line changed state, which is what the chip
/// reports for each of its four drives after a reset.
const ST0_READY_CHANGED: u8 = 0xc0;

// -- ST1 --------------------------------------------------------------------

/// Not writable: the medium is write protected.
const ST1_NW: u8 = 0x02;
/// No data: the requested sector is not on the track.
const ST1_ND: u8 = 0x04;
/// End of cylinder: the command ran past `EOT` without a terminal count.
const ST1_EN: u8 = 0x80;

// -- ST3 --------------------------------------------------------------------

/// Head address, bit 2.
const ST3_HD: u8 = 0x04;
/// Two side: the drive is double sided. Every geometry here is.
const ST3_TS: u8 = 0x08;
/// Track 0: the head is over the outermost track.
const ST3_T0: u8 = 0x10;
/// Ready.
const ST3_RY: u8 = 0x20;
/// Write protected.
const ST3_WP: u8 = 0x40;

// -- commands ---------------------------------------------------------------

/// The opcode occupies the low five bits; `MT`, `MFM` and `SK` ride above it.
const CMD_MASK: u8 = 0x1f;
/// Multi-track: after the last sector of a track, continue on the other head.
const CMD_MT: u8 = 0x80;

/// Specify the step rate and head load times.
const CMD_SPECIFY: u8 = 0x03;
/// Sense drive status: one `ST3`.
const CMD_SENSE_DRIVE: u8 = 0x04;
/// Write data.
const CMD_WRITE_DATA: u8 = 0x05;
/// Read data.
const CMD_READ_DATA: u8 = 0x06;
/// Recalibrate: step to cylinder 0.
const CMD_RECALIBRATE: u8 = 0x07;
/// Sense interrupt status: `ST0` and the present cylinder.
const CMD_SENSE_INTERRUPT: u8 = 0x08;
/// Read the sector header under the head.
const CMD_READ_ID: u8 = 0x0a;
/// Format a whole track.
const CMD_FORMAT_TRACK: u8 = 0x0d;
/// Seek to a cylinder.
const CMD_SEEK: u8 = 0x0f;

/// `SPECIFY`'s second byte, bit 0: `ND`, select non-DMA mode.
const SPECIFY_ND: u8 = 0x01;

// ---------------------------------------------------------------------------
// Geometry
// ---------------------------------------------------------------------------

/// The shape of the medium: how the linear image is cut into tracks.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct Geometry {
    cylinders: u8,
    heads: u8,
    sectors: u8,
}

impl Geometry {
    /// How many bytes a medium of this shape holds.
    fn len(&self) -> u64 {
        u64::from(self.cylinders) * u64::from(self.heads) * u64::from(self.sectors) * SECTOR_LEN
    }
}

/// The five formats `geometry = "auto"` recognises, by image length.
///
/// Length is the only thing a raw sector image carries — there is no header —
/// and these five lengths are distinct, which is what makes the inference sound
/// rather than a guess.
const STANDARD: [(u64, Geometry, &str); 5] = [
    (
        368_640,
        Geometry {
            cylinders: 40,
            heads: 2,
            sectors: 9,
        },
        "360K",
    ),
    (
        737_280,
        Geometry {
            cylinders: 80,
            heads: 2,
            sectors: 9,
        },
        "720K",
    ),
    (
        1_228_800,
        Geometry {
            cylinders: 80,
            heads: 2,
            sectors: 15,
        },
        "1.2M",
    ),
    (
        1_474_560,
        Geometry {
            cylinders: 80,
            heads: 2,
            sectors: 18,
        },
        "1.44M",
    ),
    (
        2_949_120,
        Geometry {
            cylinders: 80,
            heads: 2,
            sectors: 36,
        },
        "2.88M",
    ),
];

/// The geometry an image of `len` bytes must have, or an error naming the
/// length and every length that would have worked.
fn infer_geometry(name: &str, len: u64) -> Result<Geometry> {
    for (bytes, geom, _) in STANDARD {
        if bytes == len {
            return Ok(geom);
        }
    }
    let mut known = String::new();
    for (bytes, geom, label) in STANDARD {
        if !known.is_empty() {
            known.push_str(", ");
        }
        known.push_str(&format!(
            "{bytes} ({label}, {}/{}/{})",
            geom.cylinders, geom.heads, geom.sectors
        ));
    }
    Err(Error::Property(format!(
        "floppy image `{name}` is {len} bytes, which is no standard geometry; \
         `geometry = \"auto\"` recognises {known}, and any other image needs an \
         explicit `geometry = \"cylinders/heads/sectors\"`"
    )))
}

/// Parse `geometry`, which is either `"auto"` or `"cylinders/heads/sectors"`.
fn parse_geometry(spec: &str, name: &str, len: u64) -> Result<Geometry> {
    if spec == "auto" {
        return infer_geometry(name, len);
    }
    let mut parts = spec.split('/');
    let mut next = |what: &str| -> Result<u8> {
        let field = parts.next().unwrap_or("");
        field.parse::<u8>().map_err(|_| {
            Error::Property(format!(
                "geometry `{spec}`: `{field}` is not a {what} count in 0..=255 \
                 (write `cylinders/heads/sectors`, as in \"80/2/18\", or \"auto\")"
            ))
        })
    };
    let geom = Geometry {
        cylinders: next("cylinder")?,
        heads: next("head")?,
        sectors: next("sector")?,
    };
    if parts.next().is_some() {
        return Err(Error::Property(format!(
            "geometry `{spec}` has more than the three fields cylinders/heads/sectors"
        )));
    }
    if geom.cylinders == 0 || geom.heads == 0 || geom.sectors == 0 {
        return Err(Error::Property(format!(
            "geometry `{spec}`: a medium needs at least one cylinder, head and sector"
        )));
    }
    if geom.len() != len {
        return Err(Error::Property(format!(
            "geometry `{spec}` describes {} bytes but image `{name}` is {len}",
            geom.len()
        )));
    }
    Ok(geom)
}

// ---------------------------------------------------------------------------
// The state machine
// ---------------------------------------------------------------------------

/// Which phase the controller is in. The main status register is a pure
/// function of this, so it is one field rather than a handful of flags.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Phase {
    /// Nothing in progress: the next byte written is an opcode.
    Idle,
    /// An opcode has arrived and its parameters are being gathered.
    Command,
    /// Data is moving, by DMA or through the data register.
    Execution,
    /// Result bytes are waiting to be read.
    Result,
}

impl Phase {
    fn as_u8(self) -> u8 {
        match self {
            Phase::Idle => 0,
            Phase::Command => 1,
            Phase::Execution => 2,
            Phase::Result => 3,
        }
    }

    fn from_u8(v: u8) -> Result<Phase> {
        match v {
            0 => Ok(Phase::Idle),
            1 => Ok(Phase::Command),
            2 => Ok(Phase::Execution),
            3 => Ok(Phase::Result),
            other => Err(Error::State(format!("unknown fdc phase {other}"))),
        }
    }
}

/// Which way the execution phase moves bytes.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Dir {
    /// Read data: the controller has bytes for memory.
    ToCpu,
    /// Write data: memory has bytes for the controller.
    ToDevice,
    /// Format track: memory has four ID bytes per sector.
    Format,
}

impl Dir {
    fn as_u8(self) -> u8 {
        match self {
            Dir::ToCpu => 0,
            Dir::ToDevice => 1,
            Dir::Format => 2,
        }
    }

    fn from_u8(v: u8) -> Result<Dir> {
        match v {
            0 => Ok(Dir::ToCpu),
            1 => Ok(Dir::ToDevice),
            2 => Ok(Dir::Format),
            other => Err(Error::State(format!(
                "unknown fdc transfer direction {other}"
            ))),
        }
    }
}

/// Where a data transfer has got to.
///
/// The chip keeps the sector address in its own registers and increments it as
/// it goes, which is why the result phase reports the address of the sector
/// *after* the last one transferred rather than the last one itself.
#[derive(Debug, Clone, Copy)]
struct Xfer {
    dir: Dir,
    drive: u8,
    /// The cylinder, head and sector currently under transfer.
    c: u8,
    h: u8,
    r: u8,
    /// The size code, echoed into the result.
    n: u8,
    /// The last sector of the track, from the command's `EOT` parameter.
    eot: u8,
    /// Multi-track: cross to the second head rather than stopping at `EOT`.
    mt: bool,
    /// `FORMAT TRACK`'s filler byte.
    filler: u8,
}

/// Everything the guest can see or change.
struct State {
    phase: Phase,
    /// The command byte, `MT`/`MFM`/`SK` bits and all.
    command: u8,
    /// Parameter bytes gathered so far.
    params: Vec<u8>,
    /// How many parameters the command in progress takes.
    params_needed: u8,
    /// Result bytes still to be read, oldest first.
    results: VecDeque<u8>,
    /// The board's digital output register.
    dor: u8,
    /// The board's configuration control register: the data rate, which
    /// nothing here divides by but which software writes and expects to stick.
    ccr: u8,
    /// `SPECIFY`'s two bytes. The step rate and head load times are not
    /// simulated — see the module docs — but bit 0 of the second selects
    /// non-DMA mode, and that is honoured.
    specify: [u8; 2],
    /// Present cylinder, per drive.
    pcn: [u8; DRIVES],
    /// Which drives have a seek waiting to be sensed. This is the main status
    /// register's low nibble, and the data sheet clears it on `SENSE INTERRUPT
    /// STATUS` rather than when the head stops.
    seeking: u8,
    /// The `ST0` each pending seek will report.
    seek_st0: [u8; DRIVES],
    /// How many of the four post-reset `SENSE INTERRUPT STATUS` answers are
    /// still owed.
    reset_senses: u8,
    /// The last `ST0` computed.
    st0: u8,
    /// The last `ST1` computed.
    st1: u8,
    /// The last `ST2` computed.
    st2: u8,
    /// The last `ST3` computed.
    st3: u8,
    /// Whether an interrupt is pending. Whether it reaches the pin also depends
    /// on the DOR's gate.
    irq: bool,
    /// Set by a non-DMA data-register access that reset `INT` and re-raised it
    /// in the same breath, so [`Registers::refresh`] takes the pin low before
    /// driving it high again and an edge-triggered controller sees one request
    /// per byte rather than one per command.
    ///
    /// Transient and derived: it is set inside one access and consumed by the
    /// `refresh` that ends the same access, so it is never live across two of
    /// them and is not serialized (`CLAUDE.md`: derived state is never saved).
    retrigger: bool,
    /// `DSKCHG`, per drive.
    changed: [bool; DRIVES],
    /// Whether anything has been written to the medium.
    dirty: bool,
    /// The execution phase's buffer: one sector, or a format track's ID field.
    buf: Vec<u8>,
    /// How far into `buf` the transfer has got.
    pos: u64,
    /// The transfer in progress, if any.
    xfer: Option<Xfer>,
    /// The medium. Not serialized — see [`Device::save`].
    image: Vec<u8>,
    geom: Geometry,
    readonly: bool,
}

impl fmt::Debug for State {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        // Neither the image nor the sector buffer is printed: a 1.4 MB hex dump
        // in a `{:?}` helps nobody.
        f.debug_struct("State")
            .field("phase", &self.phase)
            .field("command", &self.command)
            .field("params", &self.params)
            .field("results", &self.results)
            .field("dor", &self.dor)
            .field("pcn", &self.pcn)
            .field("st0", &self.st0)
            .field("xfer", &self.xfer)
            .field("pos", &self.pos)
            .field("image_len", &self.image.len())
            .field("geometry", &self.geom)
            .finish()
    }
}

impl State {
    fn new(image: Vec<u8>, geom: Geometry, readonly: bool) -> State {
        State {
            phase: Phase::Idle,
            command: 0,
            params: Vec::new(),
            params_needed: 0,
            results: VecDeque::new(),
            dor: 0,
            ccr: 0,
            specify: [0; 2],
            pcn: [0; DRIVES],
            seeking: 0,
            seek_st0: [0; DRIVES],
            reset_senses: 0,
            st0: 0,
            st1: 0,
            st2: 0,
            st3: 0,
            irq: false,
            retrigger: false,
            // A drive reads "disk changed" until a step is taken with a medium
            // in it, which is what the line means at power-on.
            changed: [true; DRIVES],
            dirty: false,
            buf: Vec::new(),
            pos: 0,
            xfer: None,
            image,
            geom,
            readonly,
        }
    }

    /// Everything a controller reset clears.
    ///
    /// The DOR is a board latch and survives; so does the medium, and so do the
    /// head positions, because resetting the chip does not move a drive's head.
    fn reset_controller(&mut self) {
        self.phase = Phase::Idle;
        self.command = 0;
        self.params.clear();
        self.params_needed = 0;
        self.results.clear();
        self.specify = [0; 2];
        self.seeking = 0;
        self.seek_st0 = [0; DRIVES];
        self.reset_senses = 0;
        self.st0 = 0;
        self.st1 = 0;
        self.st2 = 0;
        self.st3 = 0;
        self.irq = false;
        self.retrigger = false;
        self.buf.clear();
        self.pos = 0;
        self.xfer = None;
    }

    /// Whether the execution phase moves bytes through the data register.
    ///
    /// Two things select it: `SPECIFY`'s `ND` bit, which is the chip's own
    /// control, and the AT's DMA gate, because with the gate closed no `DRQ`
    /// ever reaches the 8237 and polling is the only way a byte can move.
    fn non_dma(&self) -> bool {
        self.dor & DOR_DMA_INT == 0 || self.specify[1] & SPECIFY_ND != 0
    }

    /// Whether the interrupt reaches the pin.
    ///
    /// The AT gates `INT` with the same DOR bit that gates `DRQ`, so a machine
    /// that has not enabled DMA gets no floppy interrupts either — which is why
    /// every BIOS writes `0x0c` rather than `0x04`.
    fn irq_level(&self) -> bool {
        self.irq && self.dor & DOR_DMA_INT != 0
    }

    /// Whether `DRQ` is asserted: the execution phase has bytes to move and the
    /// board is delivering the request.
    fn drq_level(&self) -> bool {
        self.phase == Phase::Execution && !self.non_dma() && self.pos < self.buf.len() as u64
    }

    /// The main status register, as a read would produce it.
    fn msr(&self) -> u8 {
        let mut v = self.seeking & 0x0f;
        match self.phase {
            Phase::Idle => v |= MSR_RQM,
            Phase::Command => v |= MSR_RQM | MSR_CB,
            Phase::Result => v |= MSR_RQM | MSR_DIO | MSR_CB,
            Phase::Execution => {
                v |= MSR_CB;
                if self.non_dma() {
                    v |= MSR_NDMA;
                    if self.pos < self.buf.len() as u64 {
                        // Always high while a byte is pending: there is no
                        // rotational delay here to lower it across.
                        v |= MSR_RQM;
                    }
                    if matches!(self.xfer.map(|x| x.dir), Some(Dir::ToCpu)) {
                        v |= MSR_DIO;
                    }
                }
                // In DMA mode `RQM` stays low: the data register is not the
                // CPU's to touch while the 8237 owns the transfer.
            }
        }
        v
    }

    /// Whether drive `unit` has a medium in it.
    ///
    /// Only unit 0 can: this class takes one `image`, because a machine
    /// description that wants a second drive instantiates a second controller
    /// or waits for per-drive media slots. The other three select cleanly and
    /// report "not ready", which is what an AT with one drive does.
    fn medium(&self, unit: u8) -> bool {
        unit == 0 && !self.image.is_empty()
    }

    /// The linear sector number of a CHS address, or `None` when the address is
    /// not on the medium.
    ///
    /// **The formula, written down because the off-by-one here is the classic
    /// floppy bug**: sectors are numbered from 1, heads and cylinders from 0,
    /// and a raw image stores every sector of a cylinder's first head before
    /// any of its second's:
    ///
    /// ```text
    ///   lba = (cylinder * heads + head) * sectors_per_track + (sector - 1)
    /// ```
    ///
    /// Get it wrong and the machine boots a sector full of the wrong data,
    /// which presents as a corrupt boot record rather than as an error.
    fn lba(&self, c: u8, h: u8, r: u8) -> Option<u64> {
        if c >= self.geom.cylinders || h >= self.geom.heads || r == 0 || r > self.geom.sectors {
            return None;
        }
        Some(
            (u64::from(c) * u64::from(self.geom.heads) + u64::from(h))
                * u64::from(self.geom.sectors)
                + u64::from(r - 1),
        )
    }

    /// The byte range a CHS address occupies, if it is on the medium.
    fn sector_range(&self, c: u8, h: u8, r: u8) -> Option<(usize, usize)> {
        let lba = self.lba(c, h, r)?;
        let start = lba * SECTOR_LEN;
        let end = start + SECTOR_LEN;
        if end > self.image.len() as u64 {
            return None;
        }
        Some((start as usize, end as usize))
    }

    // -- the command phase --------------------------------------------------

    /// A parameter byte, or zero if a corrupt snapshot left it missing.
    fn param(&self, i: usize) -> u8 {
        self.params.get(i).copied().unwrap_or(0)
    }

    /// Take the first byte of a command and work out how many follow.
    fn start_command(&mut self, byte: u8) {
        self.command = byte;
        self.params.clear();
        let needed = match byte & CMD_MASK {
            CMD_SPECIFY => 2,
            CMD_SENSE_DRIVE => 1,
            CMD_WRITE_DATA | CMD_READ_DATA => 8,
            CMD_RECALIBRATE => 1,
            CMD_SENSE_INTERRUPT => 0,
            CMD_READ_ID => 1,
            CMD_FORMAT_TRACK => 5,
            CMD_SEEK => 2,
            _ => {
                self.invalid();
                return;
            }
        };
        self.params_needed = needed;
        if needed == 0 {
            self.execute();
        } else {
            self.phase = Phase::Command;
        }
    }

    /// The answer to a command the chip does not have: one `ST0` of `0x80`.
    ///
    /// Firmware probes with an illegal opcode to find out whether a controller
    /// is there at all, so this is a path that matters rather than an
    /// afterthought. No interrupt: the data sheet raises one at the start of a
    /// result phase only for commands that had an execution phase.
    fn invalid(&mut self) {
        self.st0 = ST0_INVALID;
        self.results.clear();
        self.results.push_back(ST0_INVALID);
        self.phase = Phase::Result;
    }

    /// Enter the result phase, raising the interrupt if this command has one.
    fn enter_result(&mut self, raise: bool) {
        self.phase = Phase::Result;
        if raise {
            self.irq = true;
        }
    }

    /// Run the command now that its parameters are in.
    fn execute(&mut self) {
        match self.command & CMD_MASK {
            CMD_SPECIFY => {
                self.specify = [self.param(0), self.param(1)];
                // No result and no interrupt: the chip just remembers.
                self.phase = Phase::Idle;
            }
            CMD_SENSE_DRIVE => self.sense_drive(),
            CMD_RECALIBRATE => self.seek_to(0, true),
            CMD_SENSE_INTERRUPT => self.sense_interrupt(),
            CMD_SEEK => self.seek_to(self.param(1), false),
            CMD_READ_DATA => self.start_transfer(Dir::ToCpu),
            CMD_WRITE_DATA => self.start_transfer(Dir::ToDevice),
            CMD_READ_ID => self.read_id(),
            CMD_FORMAT_TRACK => self.start_format(),
            _ => self.invalid(),
        }
    }

    /// `SENSE DRIVE STATUS`: one `ST3`, which is the drive's own wires.
    fn sense_drive(&mut self) {
        let unit = self.param(0) & 0x03;
        let head = (self.param(0) >> 2) & 1;
        let mut st3 = unit | ST3_TS;
        if head != 0 {
            st3 |= ST3_HD;
        }
        if self.pcn[unit as usize] == 0 {
            st3 |= ST3_T0;
        }
        if self.medium(unit) {
            st3 |= ST3_RY;
            if self.readonly {
                st3 |= ST3_WP;
            }
        }
        self.st3 = st3;
        self.results.clear();
        self.results.push_back(st3);
        // No interrupt: this command has no execution phase.
        self.enter_result(false);
    }

    /// `RECALIBRATE` and `SEEK`.
    ///
    /// Both move the head and raise the interrupt, and **neither has a result
    /// phase** — `SENSE INTERRUPT STATUS` is how the CPU learns what happened,
    /// which is why a driver always issues one afterwards.
    fn seek_to(&mut self, cylinder: u8, recalibrate: bool) {
        let unit = self.param(0) & 0x03;
        let head = if recalibrate {
            0
        } else {
            (self.param(0) >> 2) & 1
        };
        let mut st0 = ST0_SE | unit;
        if head != 0 {
            st0 |= ST0_HD;
        }
        if self.medium(unit) {
            self.pcn[unit as usize] = cylinder;
            // A step with a disk in the drive is what clears `DSKCHG`.
            self.changed[unit as usize] = false;
        } else {
            // Nothing to step against: a recalibrate never finds track 0, which
            // is what the equipment-check bit reports.
            self.pcn[unit as usize] = if recalibrate { 0 } else { cylinder };
            st0 |= ST0_ABNORMAL | ST0_NR;
            if recalibrate {
                st0 |= ST0_EC;
            }
        }
        self.st0 = st0;
        self.seek_st0[unit as usize] = st0;
        self.seeking |= 1 << unit;
        self.irq = true;
        self.phase = Phase::Idle;
    }

    /// `SENSE INTERRUPT STATUS`: `ST0` and the present cylinder.
    ///
    /// Three cases, in the data sheet's order. After a reset the chip owes one
    /// answer per drive, each with interrupt code 11 — the reset-polling
    /// sequence every BIOS runs. Then a seek that has completed. Then nothing,
    /// which is an invalid command and one byte of `0x80`.
    fn sense_interrupt(&mut self) {
        // Issuing the command is the acknowledgement, whatever it answers.
        self.irq = false;
        self.results.clear();
        if self.reset_senses > 0 {
            let unit = DRIVES as u8 - self.reset_senses;
            self.reset_senses -= 1;
            let st0 = ST0_READY_CHANGED | unit;
            self.st0 = st0;
            self.results.push_back(st0);
            self.results.push_back(self.pcn[unit as usize]);
            self.enter_result(false);
        } else if self.seeking != 0 {
            let unit = self.seeking.trailing_zeros() as u8;
            self.seeking &= !(1 << unit);
            let st0 = self.seek_st0[unit as usize];
            self.st0 = st0;
            self.results.push_back(st0);
            self.results.push_back(self.pcn[unit as usize]);
            self.enter_result(false);
        } else {
            self.invalid();
        }
    }

    /// `READ ID`: the header of the sector under the head.
    ///
    /// There is no rotational position to model, so the answer is always the
    /// first sector of the current track. What a driver uses this for — finding
    /// out which cylinder the head is really over — is answered truthfully.
    fn read_id(&mut self) {
        let unit = self.param(0) & 0x03;
        let head = (self.param(0) >> 2) & 1;
        let c = self.pcn[unit as usize];
        if !self.medium(unit) || self.lba(c, head, 1).is_none() {
            let st0 = ST0_ABNORMAL | ST0_NR | (head << 2) | unit;
            self.finish_results(st0, ST1_ND, 0, c, head, 1, N_512);
            return;
        }
        let st0 = (head << 2) | unit;
        self.finish_results(st0, 0, 0, c, head, 1, N_512);
    }

    /// Push the seven result bytes of a data command and enter the result
    /// phase. These commands *do* raise the interrupt.
    #[allow(clippy::too_many_arguments)]
    fn finish_results(&mut self, st0: u8, st1: u8, st2: u8, c: u8, h: u8, r: u8, n: u8) {
        self.st0 = st0;
        self.st1 = st1;
        self.st2 = st2;
        self.results.clear();
        for byte in [st0, st1, st2, c, h, r, n] {
            self.results.push_back(byte);
        }
        self.enter_result(true);
    }

    // -- the execution phase ------------------------------------------------

    /// `READ DATA` and `WRITE DATA`: eight parameters, seven results, and one
    /// or more sectors in between.
    fn start_transfer(&mut self, dir: Dir) {
        let unit = self.param(0) & 0x03;
        // The `H` parameter and the command's `HD` bit describe the same head
        // and every driver sets them alike; the chip selects with `HD`.
        let head = (self.param(0) >> 2) & 1;
        let (c, r, n, eot) = (self.param(1), self.param(3), self.param(4), self.param(5));
        let base = (head << 2) | unit;

        if !self.medium(unit) {
            self.finish_results(ST0_ABNORMAL | ST0_NR | base, 0, 0, c, head, r, n);
            return;
        }
        if dir == Dir::ToDevice && self.readonly {
            // A write to a protected medium terminates before a single byte
            // moves, with `ST1`'s not-writable bit.
            self.finish_results(ST0_ABNORMAL | base, ST1_NW, 0, c, head, r, n);
            return;
        }
        if n != N_512 {
            // Every PC floppy has 512-byte sectors, so a command asking for
            // another size is asking for a sector that is not on this medium.
            self.finish_results(ST0_ABNORMAL | base, ST1_ND, 0, c, head, r, n);
            return;
        }
        self.xfer = Some(Xfer {
            dir,
            drive: unit,
            c,
            h: head,
            r,
            n,
            eot,
            mt: self.command & CMD_MT != 0,
            filler: 0,
        });
        if !self.load_sector() {
            self.finish_transfer(false, ST1_ND);
            return;
        }
        self.phase = Phase::Execution;
        if self.non_dma() {
            // In non-DMA mode the interrupt is the request: one per byte.
            self.irq = true;
        }
    }

    /// `FORMAT TRACK`: five parameters, then four ID bytes per sector, then the
    /// same seven results a read or write produces.
    fn start_format(&mut self) {
        let unit = self.param(0) & 0x03;
        let head = (self.param(0) >> 2) & 1;
        let (n, sectors, filler) = (self.param(1), self.param(2), self.param(4));
        let base = (head << 2) | unit;
        let c = self.pcn[unit as usize];
        if !self.medium(unit) {
            self.finish_results(ST0_ABNORMAL | ST0_NR | base, 0, 0, c, head, 1, n);
            return;
        }
        if self.readonly {
            self.finish_results(ST0_ABNORMAL | base, ST1_NW, 0, c, head, 1, n);
            return;
        }
        if n != N_512 || sectors == 0 {
            self.finish_results(ST0_ABNORMAL | base, ST1_ND, 0, c, head, 1, n);
            return;
        }
        self.xfer = Some(Xfer {
            dir: Dir::Format,
            drive: unit,
            c,
            h: head,
            r: 1,
            n,
            eot: sectors,
            mt: false,
            filler,
        });
        // Four bytes per sector — C, H, R, N — arriving through the same
        // channel the data would use.
        self.buf = alloc::vec![0u8; usize::from(sectors) * 4];
        self.pos = 0;
        self.phase = Phase::Execution;
        if self.non_dma() {
            self.irq = true;
        }
    }

    /// Fill `buf` with the sector under the transfer's address, reporting
    /// whether there was one.
    fn load_sector(&mut self) -> bool {
        let Some(x) = self.xfer else { return false };
        let Some((start, end)) = self.sector_range(x.c, x.h, x.r) else {
            return false;
        };
        self.pos = 0;
        if x.dir == Dir::ToDevice {
            // A write buffers a whole sector and commits it when the sector is
            // full, so a transfer the DMA controller cuts short still writes a
            // whole sector — with zeros in the tail, as the chip does.
            self.buf = alloc::vec![0u8; SECTOR_LEN as usize];
        } else {
            self.buf = self.image[start..end].to_vec();
        }
        true
    }

    /// Commit the buffered sector to the medium.
    fn flush_sector(&mut self) {
        let Some(x) = self.xfer else { return };
        if x.dir != Dir::ToDevice || self.readonly {
            return;
        }
        let Some((start, end)) = self.sector_range(x.c, x.h, x.r) else {
            return;
        };
        let len = core::cmp::min(self.buf.len(), end - start);
        self.image[start..start + len].copy_from_slice(&self.buf[..len]);
        self.dirty = true;
    }

    /// The address the chip's sector counter holds once the command stops: the
    /// one *after* the last sector transferred.
    ///
    /// The data sheet's multi-sector table. Without multi-track, passing `EOT`
    /// bumps the cylinder and resets the sector to 1; with it, the first head's
    /// `EOT` crosses to the second head instead.
    fn next_address(x: &Xfer) -> (u8, u8, u8) {
        if x.r < x.eot {
            (x.c, x.h, x.r.wrapping_add(1))
        } else if x.mt && x.h == 0 {
            (x.c, 1, 1)
        } else if x.mt {
            (x.c.wrapping_add(1), 0, 1)
        } else {
            (x.c.wrapping_add(1), x.h, 1)
        }
    }

    /// Move to the next sector, or end the command if there is not one.
    fn advance_sector(&mut self) {
        let Some(x) = self.xfer.as_mut() else { return };
        if x.r < x.eot {
            x.r = x.r.wrapping_add(1);
        } else if x.mt && x.h == 0 {
            x.h = 1;
            x.r = 1;
        } else {
            // Ran off the end of the cylinder without a terminal count. The
            // data sheet ends the command here, abnormally, with `EN` set.
            self.finish_transfer(false, ST1_EN);
            return;
        }
        if !self.load_sector() {
            self.finish_transfer(false, ST1_ND);
        }
    }

    /// End a data transfer and produce its result bytes.
    fn finish_transfer(&mut self, normal: bool, st1: u8) {
        let Some(x) = self.xfer.take() else { return };
        let (c, h, r) = Self::next_address(&x);
        let mut st0 = (x.h << 2) | x.drive;
        if !normal {
            st0 |= ST0_ABNORMAL;
        }
        self.buf.clear();
        self.pos = 0;
        self.finish_results(st0, st1, 0, c, h, r, x.n);
    }

    /// Write the filler byte over the data field of every sector the format's
    /// ID field named.
    fn apply_format(&mut self) {
        let Some(x) = self.xfer else { return };
        // Taken out so the image can be written while the IDs are read.
        let ids = core::mem::take(&mut self.buf);
        if !self.readonly {
            for id in ids.as_chunks::<4>().0 {
                // C, H, R, N — the address the chip would write into the
                // sector's header. A raw image has no header, so what survives
                // a format is the data field.
                if let Some((start, end)) = self.sector_range(id[0], id[1], id[2]) {
                    self.image[start..end].fill(x.filler);
                    self.dirty = true;
                }
            }
        }
        self.buf = ids;
        self.finish_transfer(true, 0);
    }

    /// What happens after a byte crosses the data path, whichever way it went.
    fn step(&mut self, terminal: bool) {
        let Some(x) = self.xfer else { return };
        if x.dir == Dir::Format {
            if terminal || self.pos >= self.buf.len() as u64 {
                self.apply_format();
            }
            return;
        }
        if terminal {
            // The controller's count expired: a normal end, and the result
            // phase reports the next sector address.
            self.finish_transfer(true, 0);
            return;
        }
        if self.pos >= self.buf.len() as u64 {
            self.advance_sector();
        }
    }

    /// Take one byte of a device-to-memory transfer.
    fn take_byte(&mut self, terminal: bool) -> u8 {
        let byte = self.buf.get(self.pos as usize).copied().unwrap_or(0xff);
        self.pos += 1;
        self.step(terminal);
        byte
    }

    /// Give one byte to a memory-to-device transfer.
    fn put_byte(&mut self, byte: u8, terminal: bool) {
        if let Some(slot) = self.buf.get_mut(self.pos as usize) {
            *slot = byte;
            self.pos += 1;
        }
        let full = self.pos >= self.buf.len() as u64;
        if (full || terminal) && matches!(self.xfer.map(|x| x.dir), Some(Dir::ToDevice)) {
            self.flush_sector();
        }
        self.step(terminal);
    }

    /// Whether the data register is the CPU's to use for `dir` data right now.
    fn non_dma_execution(&self, dir: Dir) -> bool {
        self.phase == Phase::Execution
            && self.non_dma()
            && matches!(self.xfer.map(|x| x.dir), Some(d) if d == dir)
    }

    // -- the register file --------------------------------------------------

    /// Read the data register. `debug` suppresses every side effect.
    fn read_data(&mut self, debug: bool) -> u8 {
        match self.phase {
            Phase::Result => {
                if debug {
                    return self.results.front().copied().unwrap_or(0xff);
                }
                let byte = self.results.pop_front().unwrap_or(0xff);
                // Reading the first result byte is the acknowledgement.
                self.irq = false;
                if self.results.is_empty() {
                    self.phase = Phase::Idle;
                }
                byte
            }
            Phase::Execution if self.non_dma_execution(Dir::ToCpu) => {
                if debug {
                    return self.buf.get(self.pos as usize).copied().unwrap_or(0xff);
                }
                // The data-register access resets `INT` and the next byte
                // raises it again — `take_byte` leaves it raised either way,
                // for the next byte or for the result phase it has just
                // entered. So what has to reach the pin is the **fall between
                // them**, and that is what `retrigger` asks for. Assigning
                // `self.irq` here instead was wrong twice over: it held the
                // line high across every byte, and the 8259A input it lands on
                // is edge-triggered, so a command delivered one request and
                // then went silent however many bytes were left; and when the
                // byte was the last one it overwrote the `true` the result
                // phase had just set, losing the end-of-command interrupt.
                self.retrigger = true;
                self.take_byte(false)
            }
            // The data sheet leaves a read the direction bit forbids undefined;
            // an undriven AT bus reads as ones.
            _ => 0xff,
        }
    }

    /// Write the data register.
    fn write_data(&mut self, byte: u8) {
        match self.phase {
            Phase::Idle => self.start_command(byte),
            Phase::Command => {
                self.params.push(byte);
                if self.params.len() >= usize::from(self.params_needed) {
                    self.execute();
                }
            }
            Phase::Execution
                if self.non_dma_execution(Dir::ToDevice) || self.non_dma_execution(Dir::Format) =>
            {
                self.put_byte(byte, false);
                // The same fall, for the same reason; see `read_data`.
                self.retrigger = true;
            }
            // A write while the controller has results for the CPU, or while
            // the 8237 owns the data path, is swallowed: the chip is not
            // listening.
            _ => {}
        }
    }

    /// Write the digital output register, which is where a reset comes from.
    fn write_dor(&mut self, value: u8) {
        let was = self.dor;
        self.dor = value;
        if was & DOR_RESET != 0 && value & DOR_RESET == 0 {
            // Held in reset. Nothing is remembered and nothing is pending.
            self.reset_controller();
        } else if was & DOR_RESET == 0 && value & DOR_RESET != 0 {
            // Released: the chip finishes its reset, interrupts, and then owes
            // one `SENSE INTERRUPT STATUS` answer per drive.
            self.reset_controller();
            self.reset_senses = DRIVES as u8;
            self.irq = true;
        }
    }

    /// The digital input register.
    ///
    /// The AT's adapter drives only bit 7; the other seven lines float, and an
    /// undriven ISA bus reads as ones.
    fn dir_register(&self) -> u8 {
        let unit = (self.dor & DOR_DRIVE) as usize;
        // An empty drive reads "changed" for ever: there is nothing to step
        // against that would clear it.
        let changed = self.changed[unit] || !self.medium(unit as u8);
        if changed { 0xff } else { 0x7f }
    }
}

// ---------------------------------------------------------------------------
// The device
// ---------------------------------------------------------------------------

/// The register block, as something an address space can dispatch to and as the
/// DMA controller's peer.
struct Registers {
    state: Mutex<State>,
    /// `IRQ6` on a PC, at [`LockRank::LEAF`] so it can be driven with nothing
    /// else held.
    irq_out: Mutex<Option<WireSource>>,
    /// `DRQ2` on a PC.
    drq_out: Mutex<Option<WireSource>>,
}

impl fmt::Debug for Registers {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let mut s = f.debug_struct("Registers");
        match self.state.try_lock() {
            Some(state) => s.field("state", &*state).finish(),
            None => s.field("state", &"<in use>").finish(),
        }
    }
}

impl Registers {
    /// Drive one pin. Never called with the state lock held.
    fn drive(out: &Mutex<Option<WireSource>>, asserted: bool) {
        let pin = out.lock().clone();
        if let Some(pin) = pin {
            pin.set(Level::from_bool(asserted));
        }
    }

    /// Recompute both output pins, with the state lock released before either
    /// is driven — the re-entrancy contract.
    fn refresh(&self) {
        let (irq, drq, retrigger) = {
            let mut state = self.state.lock();
            (
                state.irq_level(),
                state.drq_level(),
                core::mem::take(&mut state.retrigger),
            )
        };
        if retrigger && irq {
            // The byte time between two non-DMA requests, with no duration
            // because this chip does not model transfer time (see the module
            // header). Zero-length or not, the fall happened on real hardware
            // and a wire delivers it, which is the whole difference between a
            // controller that latches one request and one that latches each.
            Registers::drive(&self.irq_out, false);
        }
        Registers::drive(&self.irq_out, irq);
        Registers::drive(&self.drq_out, drq);
    }

    /// Read one register. `debug` suppresses every side effect.
    fn read_register(&self, offset: u64, debug: bool) -> u8 {
        let mut state = self.state.lock();
        match offset {
            REG_MSR => state.msr(),
            REG_DATA => state.read_data(debug),
            REG_DIR_CCR => state.dir_register(),
            // The DOR is write-only on the AT's adapter, and offsets 0, 1, 3
            // and 6 are not decoded at all. Nothing drives the bus.
            _ => 0xff,
        }
    }

    /// Write one register.
    fn write_register(&self, offset: u64, value: u8) {
        {
            let mut state = self.state.lock();
            match offset {
                REG_DOR => state.write_dor(value),
                REG_DATA => state.write_data(value),
                // The configuration control register: the data rate. Nothing
                // here has a data rate to change, but software writes it.
                REG_DIR_CCR => state.ccr = value,
                // The main status register is read-only, and the rest are not
                // decoded.
                _ => {}
            }
        }
        self.refresh();
    }
}

impl MemOps for Registers {
    fn read(&self, offset: u64, dst: &mut [u8], attrs: MemAttrs) -> MemResult {
        let [byte] = dst else {
            return Err(BusError::BadAccess);
        };
        *byte = self.read_register(offset & 7, attrs.debug);
        if !attrs.debug {
            self.refresh();
        }
        Ok(())
    }

    fn write(&self, offset: u64, src: &[u8], attrs: MemAttrs) -> MemResult {
        let [value] = src else {
            return Err(BusError::BadAccess);
        };
        if attrs.debug {
            // A debug write to the DOR would reset the chip and one to the data
            // register would start a command. Neither can be made harmless
            // (`ROADMAP.md` §15, invariant 5).
            return Err(BusError::BadAccess);
        }
        self.write_register(offset & 7, *value);
        Ok(())
    }

    fn constraints(&self) -> AccessConstraints {
        // An 8-bit part on the AT's 8-bit peripheral bus. A word access to the
        // register file is not a thing that happens.
        AccessConstraints::word(Width::U8, Endian::Little)
    }
}

impl DmaPeripheral for Registers {
    fn dma_read(&self, terminal: bool) -> u8 {
        let byte = {
            let mut state = self.state.lock();
            if !matches!(state.xfer.map(|x| x.dir), Some(Dir::ToCpu))
                || state.phase != Phase::Execution
                || state.non_dma()
            {
                return 0xff;
            }
            state.take_byte(terminal)
        };
        self.refresh();
        byte
    }

    fn dma_write(&self, byte: u8, terminal: bool) {
        {
            let mut state = self.state.lock();
            if !matches!(
                state.xfer.map(|x| x.dir),
                Some(Dir::ToDevice) | Some(Dir::Format)
            ) || state.phase != Phase::Execution
                || state.non_dma()
            {
                return;
            }
            state.put_byte(byte, terminal);
        }
        self.refresh();
    }

    fn dma_ready(&self) -> bool {
        self.state.lock().drq_level()
    }
}

/// The drive a controller's medium can be reached through.
///
/// # Why this exists
///
/// [`Fdc765::contents`] is public and useless to anybody but the code that
/// constructed the controller — and on a realized board that is
/// [`Bindings`](crate::machine::realize::Bindings), which refuses a second
/// binding for `pc.fdc` once [`crate::dev::pc::bind`] has claimed the name. So
/// a test that wants to know what the guest actually wrote to the diskette has
/// no route to the object that holds it, and has to settle for reading the
/// bytes back through the same controller that wrote them — which cannot tell
/// "the write landed on the medium" from "the write landed in the sector
/// buffer and the medium was never touched".
///
/// This is the rendezvous that closes it, in the shape
/// [`crate::dev::ata::bays`] already has: the controller files itself under a
/// name in the build's [`HostObjects`], and whoever built the machine looks it
/// up afterwards. The name is the `drive` property, [`drives::DEFAULT_NAME`] when a
/// machine file does not say — and a `HostObjects` belongs to one build, so two
/// boards in one process do not collide however they are named.
///
/// # The lock
///
/// [`LockRank::LEAF`], deliberately unlike [`bays::BAY_RANK`], because nothing
/// a guest access reaches ever takes this: an adapter consults a *bay* on every
/// register access to find its drive, while this controller holds its own
/// medium directly and looks at this object never. It is written once at
/// construction and read by the host afterwards. Every accessor clones the
/// controller out and releases this lock before touching the controller's own
/// `DEVICE`-ranked state, so the ladder is walked downward even so.
///
/// [`HostObjects`]: crate::core::hosts::HostObjects
/// [`bays::BAY_RANK`]: crate::dev::ata::bays::BAY_RANK
pub mod drives {
    use alloc::string::String;
    use alloc::sync::Arc;
    use alloc::vec::Vec;
    use core::fmt;

    use super::Registers;
    use crate::core::error::Result;
    use crate::core::hosts::{HostKind, HostObjects};
    use crate::core::props::Props;
    use crate::core::sync::{LockRank, Mutex};

    /// The kind a floppy drive is filed under in a build's `HostObjects`.
    pub const KIND: HostKind = HostKind::new("floppy-drive");

    /// The name a controller files itself under when a machine file does not
    /// say. `fd0`, which is what the first diskette drive has been called for
    /// long enough that nobody has to look it up.
    pub const DEFAULT_NAME: &str = "fd0";

    /// One diskette drive: at most one controller's medium, by name.
    pub struct Drive {
        fdc: Mutex<Option<Arc<Registers>>>,
    }

    impl fmt::Debug for Drive {
        fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
            f.debug_struct("Drive")
                .field("occupied", &self.fdc.lock().is_some())
                .finish()
        }
    }

    impl Default for Drive {
        fn default() -> Drive {
            Drive::new()
        }
    }

    impl Drive {
        /// An empty drive.
        #[must_use]
        pub fn new() -> Drive {
            Drive {
                fdc: Mutex::with_rank(LockRank::LEAF, None),
            }
        }

        /// Put a controller in it, if it is empty.
        ///
        /// # Errors
        ///
        /// The controller back, unchanged, if one is already there. The caller
        /// has the names and makes the message.
        pub(super) fn fit(&self, regs: Arc<Registers>) -> core::result::Result<(), Arc<Registers>> {
            let mut held = self.fdc.lock();
            if held.is_some() {
                return Err(regs);
            }
            *held = Some(regs);
            Ok(())
        }

        /// Whether a controller has filed itself here.
        #[must_use]
        pub fn is_occupied(&self) -> bool {
            self.fdc.lock().is_some()
        }

        /// A copy of the medium, as the controller holds it now.
        ///
        /// `None` when no controller is filed here; an empty `Vec` when there
        /// is one and its drive has no disk in it.
        #[must_use]
        pub fn contents(&self) -> Option<Vec<u8>> {
            // Cloned out and this lock released before the controller's own
            // state lock is taken, which is the ladder's direction.
            let regs = self.fdc.lock().clone()?;
            Some(regs.state.lock().image.clone())
        }

        /// Whether anything has been written to the medium.
        #[must_use]
        pub fn is_dirty(&self) -> Option<bool> {
            let regs = self.fdc.lock().clone()?;
            Some(regs.state.lock().dirty)
        }

        /// The medium's geometry, as `(cylinders, heads, sectors)`.
        #[must_use]
        pub fn geometry(&self) -> Option<(u8, u8, u8)> {
            let regs = self.fdc.lock().clone()?;
            let g = regs.state.lock().geom;
            Some((g.cylinders, g.heads, g.sectors))
        }
    }

    /// The drive `name` refers to in `hosts`, creating it on first mention.
    ///
    /// The **host** side of the rendezvous: called after a build to look at
    /// what the controller it built is holding.
    ///
    /// # Errors
    ///
    /// [`crate::Error::Config`] if another kind of host object is already open
    /// under that name.
    pub fn open(hosts: &HostObjects, name: &str) -> Result<Arc<Drive>> {
        hosts.open(KIND, name, Drive::new)
    }

    /// The drive `name` refers to in the build these properties are being read
    /// for, creating it on first mention.
    ///
    /// The **device** side, called from `new(props)`. A `Props` that belongs to
    /// no build gets a private drive, so a controller a unit test constructed
    /// directly still works and simply meets nobody.
    ///
    /// # Errors
    ///
    /// As [`open`].
    pub fn attach(props: &Props, name: &str) -> Result<Arc<Drive>> {
        props.host(KIND, name, Drive::new)
    }

    /// The drive called `name`, if it has been opened.
    ///
    /// # Errors
    ///
    /// As [`open`].
    pub fn get(hosts: &HostObjects, name: &str) -> Result<Option<Arc<Drive>>> {
        hosts.get(KIND, name)
    }

    /// Forget `name`, reporting whether there was one.
    pub fn close(hosts: &HostObjects, name: &str) -> bool {
        hosts.close(KIND, name)
    }

    /// Every open drive name, in name order.
    #[must_use]
    pub fn names(hosts: &HostObjects) -> Vec<String> {
        hosts.names(KIND)
    }
}

/// A NEC µPD765A floppy disk controller.
#[derive(Debug)]
pub struct Fdc765 {
    regs: Arc<Registers>,
    region: RegionRef,
    /// The name this controller filed itself under, for `Debug` and for the
    /// error message when two claim one drive.
    drive_name: String,
}

impl Fdc765 {
    /// Validate `props` and build the device.
    ///
    /// # Errors
    ///
    /// [`Error::Property`] if a property is of the wrong kind, if one this
    /// class does not know was given, or if the image's length matches no
    /// geometry.
    pub fn new(props: &Props) -> Result<Fdc765> {
        let mut r = props.reader();
        let media = r.optional_media("image")?;
        let geometry: String = r.or("geometry", String::from("auto"))?;
        let readonly: bool = r.or("readonly", false)?;
        let drive: String = r.or("drive", String::from(drives::DEFAULT_NAME))?;
        r.finish()?;
        let (name, image) = match media {
            Some(m) => (m.name().to_string(), m.bytes().to_vec()),
            None => (String::from("<none>"), Vec::new()),
        };
        let fdc = Fdc765::with_image(name, image, &geometry, readonly)?;
        // The rendezvous, and the last thing `new` does: a device that filed
        // itself and then failed to build would leave a half-made controller
        // in the build's host table.
        drives::attach(props, &drive)?
            .fit(Arc::clone(&fdc.regs))
            .map_err(|_| Error::Config {
                at: drive.clone(),
                message: String::from(
                    "two floppy controllers claim one drive; give one of them its own \
                     `drive = \"...\"`",
                ),
            })?;
        Ok(Fdc765 {
            drive_name: drive,
            ..fdc
        })
    }

    /// Build one around an image the caller already has.
    ///
    /// An empty image is a drive with no disk in it, and is not an error.
    ///
    /// # Errors
    ///
    /// [`Error::Property`] if `geometry` does not parse, or if it is `"auto"`
    /// and the image's length matches no standard format.
    pub fn with_image(
        name: String,
        image: Vec<u8>,
        geometry: &str,
        readonly: bool,
    ) -> Result<Fdc765> {
        let geom = if image.is_empty() {
            // No disk. Nothing reads the geometry, because every command that
            // would consult it reports "not ready" first.
            Geometry {
                cylinders: 0,
                heads: 0,
                sectors: 0,
            }
        } else {
            parse_geometry(geometry, &name, image.len() as u64)?
        };
        let regs = Arc::new(Registers {
            state: Mutex::with_rank(LockRank::DEVICE, State::new(image, geom, readonly)),
            irq_out: Mutex::with_rank(LockRank::LEAF, None),
            drq_out: Mutex::with_rank(LockRank::LEAF, None),
        });
        let region: RegionRef = Arc::new(Region::io(
            CLASS_NAME,
            REGISTER_WINDOW_LEN,
            Arc::clone(&regs) as Arc<dyn MemOps>,
        ));
        Ok(Fdc765 {
            regs,
            region,
            // Not filed anywhere: this constructor takes no `Props`, so there
            // is no build to file it in. `new` overwrites this.
            drive_name: String::from("<unattached>"),
        })
    }

    /// The name this controller is filed under in its build's host objects.
    #[must_use]
    pub fn drive_name(&self) -> &str {
        &self.drive_name
    }

    /// A copy of the medium, for a test or a tool that wants to see what the
    /// guest wrote.
    ///
    /// Reachable only from whoever constructed the controller. On a realized
    /// board that is the machine's binding table, so a test goes through
    /// [`drives::get`] instead.
    #[must_use]
    pub fn contents(&self) -> Vec<u8> {
        self.regs.state.lock().image.clone()
    }

    /// Whether anything has been written to the medium.
    #[must_use]
    pub fn is_dirty(&self) -> bool {
        self.regs.state.lock().dirty
    }

    /// Whether the interrupt output is asserted.
    #[must_use]
    pub fn irq_asserted(&self) -> bool {
        self.regs.state.lock().irq_level()
    }

    /// The medium's geometry, as `(cylinders, heads, sectors)`. All zero when
    /// there is no disk in the drive.
    #[must_use]
    pub fn geometry(&self) -> (u8, u8, u8) {
        let g = self.regs.state.lock().geom;
        (g.cylinders, g.heads, g.sectors)
    }
}

/// The `pc.fdc` device class.
pub static CLASS: DeviceClass = DeviceClass {
    name: CLASS_NAME,
    version: STATE_VERSION,
    summary: "NEC uPD765A floppy disk controller, with DMA and non-DMA transfer",
    properties: &[
        PropertySpec {
            name: "image",
            kind: ValueKind::Media,
            required: false,
            summary: "the media slot a raw sector image is bound to; absent means an empty drive",
        },
        PropertySpec {
            name: "geometry",
            kind: ValueKind::Str,
            required: false,
            summary: "\"auto\" (default, from the image length) or \"cylinders/heads/sectors\"",
        },
        PropertySpec {
            name: "readonly",
            kind: ValueKind::Bool,
            required: false,
            summary: "write protect the medium: a write sets ST1's not-writable bit \
                      (default false)",
        },
        PropertySpec {
            name: "drive",
            kind: ValueKind::Str,
            required: false,
            summary: "the host-object name the medium is reachable under (default \"fd0\")",
        },
    ],
    construct: |props| Ok(Box::new(Fdc765::new(props)?)),
};

impl Device for Fdc765 {
    fn class(&self) -> &'static DeviceClass {
        &CLASS
    }

    fn realize(&self, _ctx: &mut RealizeCtx<'_>) -> Result<()> {
        // Nothing outward: a `map` places the region and the realizer hands the
        // wires over, both after every device has been constructed.
        Ok(())
    }

    fn reset(&self, _kind: ResetKind) {
        {
            let mut state = self.regs.state.lock();
            state.reset_controller();
            state.dor = 0;
            state.ccr = 0;
            state.pcn = [0; DRIVES];
            // The change line is asserted at power-on until a step proves a
            // disk is there, which is what a BIOS's first recalibrate is for.
            state.changed = [true; DRIVES];
        }
        self.regs.refresh();
    }

    fn region(&self, name: &str) -> Option<RegionRef> {
        matches!(name, "" | "regs").then(|| Arc::clone(&self.region))
    }

    fn connect(&self, port: &str, source: WireSource) -> Result<()> {
        match port {
            "irq" => *self.regs.irq_out.lock() = Some(source),
            "drq" => *self.regs.drq_out.lock() = Some(source),
            _ => {
                return Err(Error::Config {
                    at: port.to_string(),
                    message: String::from("a uPD765 drives two pins, `irq` and `drq`"),
                });
            }
        }
        Ok(())
    }

    fn dma_peripheral(&self, port: &str) -> Option<Arc<dyn DmaPeripheral>> {
        // The bytes travel the same net the request does, in the other
        // direction — `core::wire::DmaPeripheral`.
        (port == "drq").then(|| Arc::clone(&self.regs) as Arc<dyn DmaPeripheral>)
    }

    fn announce(&self, port: &str) {
        if matches!(port, "irq" | "drq") {
            self.regs.refresh();
        }
    }

    fn save(&self, w: &mut ChunkWriter<'_>) -> Result<()> {
        let state = self.regs.state.lock();
        w.write_u8(state.phase.as_u8())?;
        w.write_u8(state.command)?;
        w.write_seq_len(state.params.len() as u64)?;
        for byte in &state.params {
            w.write_u8(*byte)?;
        }
        w.write_u8(state.params_needed)?;
        w.write_seq_len(state.results.len() as u64)?;
        for byte in &state.results {
            w.write_u8(*byte)?;
        }
        w.write_u8(state.dor)?;
        w.write_u8(state.ccr)?;
        w.write_u8(state.specify[0])?;
        w.write_u8(state.specify[1])?;
        for unit in 0..DRIVES {
            w.write_u8(state.pcn[unit])?;
            w.write_u8(state.seek_st0[unit])?;
            w.write_bool(state.changed[unit])?;
        }
        w.write_u8(state.seeking)?;
        w.write_u8(state.reset_senses)?;
        w.write_u8(state.st0)?;
        w.write_u8(state.st1)?;
        w.write_u8(state.st2)?;
        w.write_u8(state.st3)?;
        w.write_bool(state.irq)?;
        // Whether the medium has been written to *is* serialized: a caller
        // needs it to know that a snapshot's disk differs from the file it came
        // from.
        w.write_bool(state.dirty)?;
        w.write_seq_len(state.buf.len() as u64)?;
        w.write_all(&state.buf)?;
        w.write_u64(state.pos)?;
        match state.xfer {
            None => w.write_bool(false)?,
            Some(x) => {
                w.write_bool(true)?;
                w.write_u8(x.dir.as_u8())?;
                for byte in [x.drive, x.c, x.h, x.r, x.n, x.eot, x.filler] {
                    w.write_u8(byte)?;
                }
                w.write_bool(x.mt)?;
            }
        }
        Ok(())
        // **The medium is deliberately absent.** A disk image is media the
        // machine was built with, exactly as `pc.rom`'s firmware image is, and
        // snapshotting a mounted image belongs to the storage-snapshot design in
        // `ROADMAP.md` §4.5 — which does not exist yet. Restoring into a machine
        // built with a different disk is therefore the caller's mistake to
        // avoid, not something this chunk can detect.
    }

    fn load(&self, r: &mut ChunkReader<'_>) -> Result<()> {
        let phase = Phase::from_u8(r.read_u8()?)?;
        let command = r.read_u8()?;
        let param_count = r.read_seq_len(1)?;
        if param_count > 8 {
            return Err(Error::State(format!(
                "snapshot has {param_count} command parameter(s); no uPD765 command takes over 8"
            )));
        }
        let mut params = Vec::with_capacity(param_count as usize);
        for _ in 0..param_count {
            params.push(r.read_u8()?);
        }
        let params_needed = r.read_u8()?;
        let result_count = r.read_seq_len(1)?;
        if result_count > 7 {
            return Err(Error::State(format!(
                "snapshot has {result_count} result byte(s); no uPD765 command returns over 7"
            )));
        }
        let mut results = VecDeque::with_capacity(result_count as usize);
        for _ in 0..result_count {
            results.push_back(r.read_u8()?);
        }
        let dor = r.read_u8()?;
        let ccr = r.read_u8()?;
        let specify = [r.read_u8()?, r.read_u8()?];
        let mut pcn = [0u8; DRIVES];
        let mut seek_st0 = [0u8; DRIVES];
        let mut changed = [false; DRIVES];
        for unit in 0..DRIVES {
            pcn[unit] = r.read_u8()?;
            seek_st0[unit] = r.read_u8()?;
            changed[unit] = r.read_bool()?;
        }
        let seeking = r.read_u8()?;
        let reset_senses = r.read_u8()?;
        if reset_senses > DRIVES as u8 {
            return Err(Error::State(format!(
                "snapshot owes {reset_senses} reset senses for {DRIVES} drives"
            )));
        }
        let st0 = r.read_u8()?;
        let st1 = r.read_u8()?;
        let st2 = r.read_u8()?;
        let st3 = r.read_u8()?;
        let irq = r.read_bool()?;
        let dirty = r.read_bool()?;
        let buf_len = r.read_seq_len(1)?;
        let buf = r.take(buf_len as usize)?.to_vec();
        let pos = r.read_u64()?;
        let xfer = if r.read_bool()? {
            let dir = Dir::from_u8(r.read_u8()?)?;
            let drive = r.read_u8()?;
            let c = r.read_u8()?;
            let h = r.read_u8()?;
            let sector = r.read_u8()?;
            let n = r.read_u8()?;
            let eot = r.read_u8()?;
            let filler = r.read_u8()?;
            let mt = r.read_bool()?;
            Some(Xfer {
                dir,
                drive,
                c,
                h,
                r: sector,
                n,
                eot,
                mt,
                filler,
            })
        } else {
            None
        };

        {
            let mut state = self.regs.state.lock();
            state.phase = phase;
            state.command = command;
            state.params = params;
            state.params_needed = params_needed;
            state.results = results;
            state.dor = dor;
            state.ccr = ccr;
            state.specify = specify;
            state.pcn = pcn;
            state.seek_st0 = seek_st0;
            state.changed = changed;
            state.seeking = seeking;
            state.reset_senses = reset_senses;
            state.st0 = st0;
            state.st1 = st1;
            state.st2 = st2;
            state.st3 = st3;
            state.irq = irq;
            state.dirty = dirty;
            state.buf = buf;
            state.pos = pos;
            state.xfer = xfer;
        }
        self.regs.refresh();
        Ok(())
    }
}

impl Instance for Fdc765 {}

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
    bindings.bind(CLASS_NAME, |props| Ok(Arc::new(Fdc765::new(props)?)))
}

/// What the validator should know about `pc.fdc`.
#[must_use]
pub fn schema() -> ClassSchema {
    ClassSchema::new(CLASS_NAME)
        .prop(PropSchema::new("image", ValueKind::Media))
        .prop(PropSchema::new("geometry", ValueKind::Str))
        .prop(PropSchema::new("readonly", ValueKind::Bool))
        .prop(PropSchema::new("drive", ValueKind::Str))
        .region("")
        .region("regs")
        .port("irq", PortDir::Out)
        .port("drq", PortDir::Out)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::state::{MachineShape, Migrations, StateReader, StateWriter};
    use crate::core::sync::{AtomicU32, Ordering};
    use crate::core::wire::{Wire, WireId, WireIdAllocator, WireSink};

    /// A 1.44M image whose every byte says which sector it is in, so a transfer
    /// that reads the wrong sector cannot pass by accident.
    fn image_1440k() -> Vec<u8> {
        let mut image = alloc::vec![0u8; 1_474_560];
        for (lba, sector) in image.as_chunks_mut::<512>().0.iter_mut().enumerate() {
            for (i, byte) in sector.iter_mut().enumerate() {
                *byte = (lba as u8) ^ (i as u8);
            }
        }
        image
    }

    /// What the sector at `lba` holds.
    fn sector_of(image: &[u8], lba: usize) -> Vec<u8> {
        image[lba * 512..(lba + 1) * 512].to_vec()
    }

    #[derive(Debug, Default)]
    struct Probe {
        level: AtomicU32,
        /// Low-to-high transitions seen. The level alone cannot tell a second
        /// request from the first one still standing, and on an edge-triggered
        /// 8259A input that is exactly the difference that matters.
        rises: AtomicU32,
    }

    impl Probe {
        fn high(&self) -> bool {
            self.level.load(Ordering::Relaxed) != 0
        }

        fn rises(&self) -> u32 {
            self.rises.load(Ordering::Relaxed)
        }
    }

    impl WireSink for Probe {
        fn set_level(&self, _src: WireId, _line: u32, level: Level) {
            let was = self
                .level
                .swap(u32::from(level.is_high()), Ordering::Relaxed);
            if level.is_high() && was == 0 {
                self.rises.fetch_add(1, Ordering::Relaxed);
            }
        }
    }

    /// A controller with both output pins wired to probes.
    struct Rig {
        fdc: Fdc765,
        irq: Arc<Probe>,
        drq: Arc<Probe>,
    }

    fn rig_with(image: Vec<u8>, readonly: bool) -> Rig {
        let fdc = Fdc765::with_image("test".to_string(), image, "auto", readonly)
            .expect("a standard geometry");
        let ids = WireIdAllocator::new();
        let irq = Arc::new(Probe::default());
        let drq = Arc::new(Probe::default());
        for (port, probe) in [("irq", Arc::clone(&irq)), ("drq", Arc::clone(&drq))] {
            let id = ids.alloc();
            let wire = Wire::builder()
                .source(id)
                .sink(probe as Arc<dyn WireSink>, 0)
                .build_shared();
            fdc.connect(port, WireSource::new(wire, id))
                .expect("both pins exist");
        }
        Rig { fdc, irq, drq }
    }

    fn rig() -> Rig {
        rig_with(image_1440k(), false)
    }

    impl Rig {
        fn peek(&self, offset: u64) -> u8 {
            let mut byte = [0u8; 1];
            self.fdc
                .regs
                .read(offset, &mut byte, MemAttrs::DEFAULT)
                .expect("a byte read is legal");
            byte[0]
        }

        fn poke(&self, offset: u64, value: u8) {
            self.fdc
                .regs
                .write(offset, &[value], MemAttrs::DEFAULT)
                .expect("a byte write is legal");
        }

        fn msr(&self) -> u8 {
            self.peek(REG_MSR)
        }

        /// Bring the controller up the way a BIOS does: pulse the reset, open
        /// the DMA gate, spin the motor, and swallow the four reset senses.
        fn power_on(&self) {
            self.poke(REG_DOR, 0x00);
            self.poke(REG_DOR, 0x1c);
            for _ in 0..DRIVES {
                self.command(&[CMD_SENSE_INTERRUPT]);
                let _ = self.results();
            }
        }

        /// Write a command byte and its parameters, checking the handshake as a
        /// driver would.
        fn command(&self, bytes: &[u8]) {
            for byte in bytes {
                assert_eq!(
                    self.msr() & (MSR_RQM | MSR_DIO),
                    MSR_RQM,
                    "the controller must be asking for a command byte"
                );
                self.poke(REG_DATA, *byte);
            }
        }

        /// Drain the result phase the way a driver does: while `RQM` and `DIO`
        /// both say there is a byte.
        fn results(&self) -> Vec<u8> {
            let mut out = Vec::new();
            while self.msr() & (MSR_RQM | MSR_DIO | MSR_CB) == (MSR_RQM | MSR_DIO | MSR_CB) {
                out.push(self.peek(REG_DATA));
            }
            out
        }

        /// Move `count` bytes out of the controller through the DMA seam,
        /// pulsing the terminal count on the last one.
        fn dma_in(&self, count: usize) -> Vec<u8> {
            let peer = self
                .fdc
                .dma_peripheral("drq")
                .expect("the controller offers one on drq");
            let mut out = Vec::with_capacity(count);
            for i in 0..count {
                assert!(peer.dma_ready(), "DRQ must still be asserted at byte {i}");
                out.push(peer.dma_read(i + 1 == count));
            }
            out
        }

        /// Move `bytes` into the controller through the DMA seam.
        fn dma_out(&self, bytes: &[u8]) {
            let peer = self
                .fdc
                .dma_peripheral("drq")
                .expect("the controller offers one on drq");
            for (i, byte) in bytes.iter().enumerate() {
                assert!(peer.dma_ready(), "DRQ must still be asserted at byte {i}");
                peer.dma_write(*byte, i + 1 == bytes.len());
            }
        }
    }

    #[test]
    fn a_reset_reports_each_drive_and_then_says_invalid() {
        // The sequence every BIOS runs: pulse /RESET, then poll the chip with
        // SENSE INTERRUPT STATUS until it stops answering.
        let rig = rig();
        rig.poke(REG_DOR, 0x00);
        rig.poke(REG_DOR, 0x0c);
        assert!(rig.irq.high(), "the reset interrupts");
        assert_eq!(rig.msr(), MSR_RQM, "and the chip is ready for a command");

        for unit in 0..4u8 {
            rig.command(&[CMD_SENSE_INTERRUPT]);
            let out = rig.results();
            assert_eq!(out.len(), 2, "ST0 and the present cylinder");
            assert_eq!(out[0], ST0_READY_CHANGED | unit);
        }
        assert!(!rig.irq.high(), "sensing the interrupt drops it");

        rig.command(&[CMD_SENSE_INTERRUPT]);
        assert_eq!(
            rig.results(),
            alloc::vec![ST0_INVALID],
            "and then there is nothing left to say"
        );
    }

    #[test]
    fn holding_the_chip_in_reset_forgets_a_command_in_progress() {
        let rig = rig();
        rig.power_on();
        rig.poke(REG_DATA, 0x46); // a read, whose parameters never arrive
        assert_eq!(rig.msr() & MSR_CB, MSR_CB);
        rig.poke(REG_DOR, 0x10); // /RESET low, motor still on
        assert_eq!(rig.msr(), MSR_RQM, "back to idle with nothing pending");
    }

    #[test]
    fn recalibrate_seeks_to_zero_and_interrupts() {
        let rig = rig();
        rig.power_on();
        rig.command(&[CMD_SEEK, 0x00, 40]);
        rig.command(&[CMD_SENSE_INTERRUPT]);
        assert_eq!(rig.results()[1], 40, "the head moved out to cylinder 40");

        rig.command(&[CMD_RECALIBRATE, 0x00]);
        assert!(rig.irq.high(), "a recalibrate ends with an interrupt");
        assert_eq!(rig.msr() & 0x0f, 0x01, "drive 0 is in the seek mode");
        // No result phase: SENSE INTERRUPT STATUS is how the CPU finds out.
        assert_eq!(rig.msr() & MSR_DIO, 0);

        rig.command(&[CMD_SENSE_INTERRUPT]);
        assert_eq!(
            rig.results(),
            alloc::vec![ST0_SE, 0],
            "seek end on drive 0, present cylinder 0"
        );
        assert!(!rig.irq.high(), "and the interrupt is dropped");
        assert_eq!(rig.msr() & 0x0f, 0, "and the seek bit with it");
    }

    #[test]
    fn seek_reports_the_cylinder_it_was_given() {
        let rig = rig();
        rig.power_on();
        rig.command(&[CMD_SEEK, 0x04, 17]); // head 1, drive 0
        assert!(rig.irq.high());
        rig.command(&[CMD_SENSE_INTERRUPT]);
        let out = rig.results();
        assert_eq!(out[0], ST0_SE | ST0_HD, "seek end, head 1");
        assert_eq!(out[1], 17);
    }

    #[test]
    fn read_data_delivers_the_first_sector_through_dma() {
        let image = image_1440k();
        let rig = rig_with(image.clone(), false);
        rig.power_on();
        rig.command(&[CMD_SPECIFY, 0xdf, 0x02]);
        // MFM read, drive 0 head 0, C=0 H=0 R=1 N=2 EOT=18 GPL=0x1b DTL=0xff.
        rig.command(&[0x46, 0x00, 0, 0, 1, 2, 18, 0x1b, 0xff]);
        assert!(rig.drq.high(), "the execution phase asks for service");
        assert_eq!(rig.msr() & (MSR_CB | MSR_RQM | MSR_NDMA), MSR_CB);

        let got = rig.dma_in(512);
        assert_eq!(got, sector_of(&image, 0));
        assert!(!rig.drq.high(), "and DRQ drops when the count expires");
        assert!(rig.irq.high(), "the result phase interrupts");

        // ST0, ST1, ST2, then the address of the sector *after* the last one
        // transferred — the data sheet's sector counter.
        assert_eq!(rig.results(), alloc::vec![0x00, 0x00, 0x00, 0, 0, 2, 2]);
        assert!(!rig.irq.high(), "reading the first result byte drops it");
    }

    #[test]
    fn a_multitrack_read_crosses_to_the_second_head() {
        let image = image_1440k();
        let rig = rig_with(image.clone(), false);
        rig.power_on();
        // MT|MFM, starting at the last sector of head 0's cylinder 0.
        rig.command(&[CMD_MT | 0x46, 0x00, 0, 0, 18, 2, 18, 0x1b, 0xff]);
        let got = rig.dma_in(1024);
        // (0*2+0)*18 + 17 = 17, then (0*2+1)*18 + 0 = 18.
        assert_eq!(got[..512], sector_of(&image, 17)[..]);
        assert_eq!(got[512..], sector_of(&image, 18)[..]);

        let out = rig.results();
        assert_eq!(out[0], ST0_HD, "the command ended on head 1");
        assert_eq!(&out[3..], &[0, 1, 2, 2], "cylinder 0, head 1, sector 2");
    }

    #[test]
    fn a_read_that_runs_off_the_track_without_a_terminal_count_ends_abnormally() {
        // The data sheet: with no TC the command stops at EOT and sets EN.
        let rig = rig();
        rig.power_on();
        rig.command(&[0x46, 0x00, 0, 0, 1, 2, 1, 0x1b, 0xff]);
        let peer = rig.fdc.dma_peripheral("drq").expect("a peripheral on drq");
        for _ in 0..512 {
            peer.dma_read(false);
        }
        let out = rig.results();
        assert_eq!(out[0], ST0_ABNORMAL);
        assert_eq!(out[1], ST1_EN);
    }

    #[test]
    fn write_data_changes_the_image_and_reads_back() {
        let image = image_1440k();
        let rig = rig_with(image.clone(), false);
        rig.power_on();
        // Sector 3 of cylinder 0, head 0 is linear sector 2.
        let payload: Vec<u8> = (0..512u32).map(|i| (i as u8).wrapping_mul(3)).collect();
        rig.command(&[0x45, 0x00, 0, 0, 3, 2, 18, 0x1b, 0xff]);
        rig.dma_out(&payload);
        assert_eq!(rig.results()[0], 0x00, "a normal termination");
        assert!(rig.fdc.is_dirty());
        assert_eq!(sector_of(&rig.fdc.contents(), 2), payload);
        assert_eq!(
            sector_of(&rig.fdc.contents(), 1),
            sector_of(&image, 1),
            "and no neighbour was touched"
        );

        rig.command(&[0x46, 0x00, 0, 0, 3, 2, 18, 0x1b, 0xff]);
        assert_eq!(rig.dma_in(512), payload);
        assert_eq!(rig.results()[0], 0x00);
    }

    #[test]
    fn a_write_to_a_readonly_medium_sets_write_protect() {
        let image = image_1440k();
        let rig = rig_with(image.clone(), true);
        rig.power_on();
        rig.command(&[0x45, 0x00, 0, 0, 1, 2, 18, 0x1b, 0xff]);
        let out = rig.results();
        assert_eq!(out[0], ST0_ABNORMAL);
        assert_eq!(out[1] & ST1_NW, ST1_NW, "ST1's not-writable bit");
        assert!(!rig.fdc.is_dirty());
        assert_eq!(rig.fdc.contents(), image, "and the medium is untouched");

        // And the drive says so when asked directly.
        rig.command(&[CMD_SENSE_DRIVE, 0x00]);
        let st3 = rig.results();
        assert_eq!(st3.len(), 1, "SENSE DRIVE STATUS returns one byte");
        assert_eq!(
            st3[0] & (ST3_WP | ST3_RY | ST3_T0),
            ST3_WP | ST3_RY | ST3_T0
        );
    }

    #[test]
    fn an_unrecognised_command_answers_with_one_invalid_byte() {
        // Firmware probes with an illegal opcode to find out whether there is a
        // controller here at all.
        let rig = rig();
        rig.power_on();
        for opcode in [0x00u8, 0x01, 0x1a, 0x9c] {
            rig.command(&[opcode]);
            assert_eq!(
                rig.results(),
                alloc::vec![ST0_INVALID],
                "opcode {opcode:#04x} is not a uPD765 command"
            );
            assert!(!rig.irq.high(), "and an invalid command does not interrupt");
        }
    }

    #[test]
    fn read_id_answers_the_header_under_the_head() {
        // 0x4a is READ ID with the MFM bit — the form every PC BIOS issues, and
        // opcode 0x0a underneath (data sheet, command summary).
        let rig = rig();
        rig.power_on();
        rig.command(&[CMD_SEEK, 0x00, 5]);
        rig.command(&[CMD_SENSE_INTERRUPT]);
        let _ = rig.results();
        rig.command(&[0x4a, 0x00]);
        let out = rig.results();
        assert_eq!(out.len(), 7);
        assert_eq!(out[0], 0x00, "a normal termination");
        assert_eq!(&out[3..], &[5, 0, 1, N_512], "cylinder 5, head 0, sector 1");
    }

    #[test]
    fn a_drive_with_no_disk_reports_not_ready() {
        let rig = rig_with(Vec::new(), false);
        rig.power_on();
        rig.command(&[0x46, 0x00, 0, 0, 1, 2, 18, 0x1b, 0xff]);
        let out = rig.results();
        assert_eq!(out[0] & ST0_NR, ST0_NR, "not ready, rather than a fault");
        assert_eq!(out[0] & 0xc0, ST0_ABNORMAL);

        rig.command(&[CMD_SENSE_DRIVE, 0x00]);
        assert_eq!(rig.results()[0] & ST3_RY, 0, "and the drive says so");
        assert_eq!(
            rig.peek(REG_DIR_CCR) & 0x80,
            0x80,
            "an empty drive reads as changed"
        );
    }

    #[test]
    fn a_debug_read_does_not_advance_the_result_phase() {
        let rig = rig();
        rig.power_on();
        rig.command(&[0x4a, 0x00]);
        assert!(rig.irq.high());
        let mut byte = [0u8; 1];
        rig.fdc
            .regs
            .read(REG_DATA, &mut byte, MemAttrs::DEBUG)
            .expect("a debug read is legal");
        assert_eq!(byte[0], 0x00, "ST0, peeked");
        assert!(rig.irq.high(), "and the interrupt is still asserted");
        assert_eq!(
            rig.msr() & (MSR_RQM | MSR_DIO | MSR_CB),
            MSR_RQM | MSR_DIO | MSR_CB,
            "still in the result phase"
        );
        assert_eq!(rig.results().len(), 7, "with every byte still there");

        // A debug write is refused rather than made harmless.
        assert!(
            rig.fdc
                .regs
                .write(REG_DOR, &[0x00], MemAttrs::DEBUG)
                .is_err()
        );
    }

    #[test]
    fn non_dma_mode_moves_a_sector_through_the_data_register() {
        let image = image_1440k();
        let rig = rig_with(image.clone(), false);
        rig.power_on();
        // The AT's DMA gate closed: no DRQ can reach the 8237, so the CPU moves
        // the bytes itself.
        rig.poke(REG_DOR, 0x14);
        rig.command(&[0x46, 0x00, 0, 0, 1, 2, 1, 0x1b, 0xff]);
        assert!(!rig.drq.high(), "no DRQ with the gate closed");

        let mut got = Vec::new();
        while rig.msr() & (MSR_NDMA | MSR_RQM | MSR_DIO) == (MSR_NDMA | MSR_RQM | MSR_DIO) {
            got.push(rig.peek(REG_DATA));
        }
        assert_eq!(got, sector_of(&image, 0));
        // EOT with no terminal count: the data sheet ends the command with EN.
        let out = rig.results();
        assert_eq!(out[0], ST0_ABNORMAL);
        assert_eq!(out[1], ST1_EN);

        // The chip's own ND bit selects the same mode, and with the AT's gate
        // open the per-byte interrupt is delivered as well.
        rig.poke(REG_DOR, 0x1c);
        rig.command(&[CMD_SPECIFY, 0xdf, 0x03]);
        let before = rig.irq.rises();
        rig.command(&[0x46, 0x00, 0, 0, 1, 2, 1, 0x1b, 0xff]);
        assert!(rig.irq.high(), "a byte is waiting");
        assert_eq!(rig.irq.rises(), before + 1, "and it announced itself once");
        assert_eq!(rig.peek(REG_DATA), image[0]);
        assert!(rig.irq.high(), "and so is the next one");
        // The claim the level cannot make. `INT` is reset by the data-register
        // access and raised again by the next byte (uPD765A data sheet, the
        // non-DMA execution phase), so each byte is its own request. The 8259A
        // input this lands on is edge-triggered and latches `IRR` only on a
        // low-to-high transition, so a model that left the line high delivered
        // one interrupt for the whole command and the guest waited for ever.
        assert_eq!(
            rig.irq.rises(),
            before + 2,
            "the second byte is a second request, not the first one still standing"
        );
        let mut bytes = 1u32;
        while rig.msr() & MSR_NDMA != 0 {
            let _ = rig.peek(REG_DATA);
            bytes += 1;
        }
        assert_eq!(bytes, 512, "a whole sector, one byte at a time");
        // One rise per byte, plus the one that opened the result phase — which
        // the byte that ended the transfer used to overwrite with `false`.
        assert_eq!(rig.irq.rises(), before + bytes + 1);
        assert!(rig.irq.high(), "and the result phase is announced");
        let _ = rig.results();

        // And a write travels the same path.
        rig.command(&[CMD_SPECIFY, 0xdf, 0x03]);
        rig.command(&[0x45, 0x00, 0, 0, 2, 2, 2, 0x1b, 0xff]);
        assert_eq!(
            rig.msr() & (MSR_NDMA | MSR_DIO),
            MSR_NDMA,
            "the controller wants bytes rather than offering them"
        );
        for i in 0..512u32 {
            rig.poke(REG_DATA, (i as u8) ^ 0x5a);
        }
        let _ = rig.results();
        let written = sector_of(&rig.fdc.contents(), 1);
        assert_eq!(written[0], 0x5a);
        assert_eq!(written[511], 0xffu8 ^ 0x5a);
    }

    #[test]
    fn format_track_fills_the_track_and_does_not_hang() {
        let rig = rig();
        rig.power_on();
        // N=2, 18 sectors, gap 0x54, filler 0xf6 — what a DOS FORMAT writes.
        rig.command(&[CMD_FORMAT_TRACK, 0x00, 2, 18, 0x54, 0xf6]);
        let mut ids = Vec::new();
        for sector in 1..=18u8 {
            ids.extend_from_slice(&[0, 0, sector, 2]);
        }
        rig.dma_out(&ids);
        let out = rig.results();
        assert_eq!(out.len(), 7);
        assert_eq!(out[0], 0x00);
        let contents = rig.fdc.contents();
        assert!(contents[..18 * 512].iter().all(|b| *b == 0xf6));
        assert_eq!(
            sector_of(&contents, 18),
            sector_of(&image_1440k(), 18),
            "and the second head's track is untouched"
        );
    }

    #[test]
    fn an_image_of_no_known_geometry_is_refused_by_name() {
        let e = Fdc765::with_image("odd".to_string(), alloc::vec![0u8; 1000], "auto", false)
            .expect_err("1000 bytes is no floppy");
        let text = e.to_string();
        assert!(text.contains("1000"), "{text}");
        assert!(
            text.contains("1474560"),
            "and it lists what would have worked: {text}"
        );

        // An explicit geometry that does not match the image is refused too.
        assert!(Fdc765::with_image("odd".to_string(), image_1440k(), "80/2/9", false).is_err());
        // And one that does is taken.
        let fdc = Fdc765::with_image("ok".to_string(), image_1440k(), "80/2/18", false)
            .expect("an explicit geometry");
        assert_eq!(fdc.geometry(), (80, 2, 18));
    }

    #[test]
    fn the_undecoded_offsets_read_as_ones() {
        // The AT's adapter answers four of its eight ports.
        let rig = rig();
        for offset in [0, 1, 2, 3, 6] {
            assert_eq!(rig.peek(offset), 0xff, "offset {offset} is not decoded");
        }
    }

    #[test]
    fn an_access_that_is_not_a_single_byte_is_refused() {
        let rig = rig();
        assert!(
            rig.fdc
                .regs
                .read(REG_MSR, &mut [0u8; 2], MemAttrs::DEFAULT)
                .is_err()
        );
        assert!(
            rig.fdc
                .regs
                .write(REG_DOR, &[0u8; 4], MemAttrs::DEFAULT)
                .is_err()
        );
    }

    #[test]
    fn properties_are_checked_rather_than_ignored() {
        let fdc = Fdc765::new(&Props::new()).expect("an empty drive is a legal machine");
        assert_eq!(fdc.geometry(), (0, 0, 0));
        assert!(!fdc.irq_asserted());
        assert!(Fdc765::new(&Props::new().with("geometery", "auto")).is_err());

        // The path a machine file takes: a media slot the realizer has already
        // bound to bytes, plus the write-protect tab.
        let props = Props::new()
            .with(
                "image",
                crate::core::props::Media::new("floppy", image_1440k()),
            )
            .with("readonly", true);
        let fdc = Fdc765::new(&props).expect("a 1.44M image in a bound slot");
        assert_eq!(fdc.geometry(), (80, 2, 18));
    }

    #[test]
    fn the_medium_is_reachable_through_the_drive_a_build_filed_it_under() {
        use crate::core::hosts::HostObjects;

        // The shape a realized board has: the build owns a `HostObjects`, the
        // controller files itself in it, and whoever built the machine looks it
        // up afterwards — which is the only route to the medium once
        // `Bindings` has claimed `pc.fdc` and a test can no longer construct it.
        let hosts = Arc::new(HostObjects::new());
        let mut props = Props::new().with(
            "image",
            crate::core::props::Media::new("floppy", image_1440k()),
        );
        props.set_hosts(Arc::clone(&hosts));
        let fdc = Fdc765::new(&props).expect("a 1.44M image in a bound slot");
        assert_eq!(fdc.drive_name(), drives::DEFAULT_NAME);

        let drive = drives::get(&hosts, drives::DEFAULT_NAME)
            .expect("a drive, not a name collision")
            .expect("the controller filed itself");
        assert!(drive.is_occupied());
        assert_eq!(drive.geometry(), Some((80, 2, 18)));
        assert_eq!(drive.is_dirty(), Some(false));
        assert_eq!(
            drive.contents().expect("a medium").len(),
            image_1440k().len()
        );
        assert_eq!(drives::names(&hosts), alloc::vec![String::from("fd0")]);

        // What the whole thing is for: a write the *guest* made is visible on
        // the medium from outside the controller, so a test need not read it
        // back through the same sector buffer that wrote it.
        {
            let mut state = fdc.regs.state.lock();
            state.image[512] = 0xa5;
            state.dirty = true;
        }
        assert_eq!(drive.contents().expect("a medium")[512], 0xa5);
        assert_eq!(drive.is_dirty(), Some(true));

        // A second controller on the same name is a machine-file error rather
        // than a silent overwrite of whose medium is reachable.
        let mut second = Props::new();
        second.set_hosts(Arc::clone(&hosts));
        let e = Fdc765::new(&second).expect_err("fd0 is taken").to_string();
        assert!(e.contains("fd0"), "{e}");

        // And a name of its own is fine.
        let mut third = Props::new().with("drive", "fd1");
        third.set_hosts(Arc::clone(&hosts));
        let other = Fdc765::new(&third).expect("fd1 is free");
        assert_eq!(other.drive_name(), "fd1");
        assert_eq!(
            drives::get(&hosts, "fd1")
                .expect("no collision")
                .expect("filed")
                .contents(),
            Some(alloc::vec![]),
            "an empty drive is a controller with no disk, not an absent one"
        );

        // A controller a unit test built directly meets nobody, which is what
        // makes every other test in this file work without a build.
        assert_eq!(
            Fdc765::with_image(String::from("x"), Vec::new(), "auto", false)
                .expect("legal")
                .drive_name(),
            "<unattached>"
        );
    }

    #[test]
    fn a_snapshot_round_trips_the_controller_state() {
        let saved = rig();
        saved.power_on();
        saved.command(&[CMD_SPECIFY, 0xdf, 0x02]);
        saved.command(&[CMD_SEEK, 0x01, 12]); // drive 1, which has no disk
        saved.command(&[0x46, 0x00, 0, 0, 4, 2, 18, 0x1b, 0xff]);
        // Stop in the middle of the execution phase, which is the interesting
        // moment: half a sector is buffered and a transfer is live.
        let peer = saved
            .fdc
            .dma_peripheral("drq")
            .expect("a peripheral on drq");
        for _ in 0..100 {
            peer.dma_read(false);
        }

        let mut shape = MachineShape::new();
        shape.add_device("fdc", CLASS.name).unwrap();
        let mut w = StateWriter::new(shape);
        {
            let mut chunk = w.chunk("fdc", CLASS.name, CLASS.version).unwrap();
            saved.fdc.save(&mut chunk).unwrap();
        }
        let first = w.to_vec().unwrap();

        let restored = rig();
        let reader = StateReader::new(&first).unwrap();
        let chunk = reader
            .load("fdc", CLASS.name, CLASS.version, &Migrations::new())
            .unwrap();
        restored.fdc.load(&mut chunk.reader()).unwrap();

        let mut shape = MachineShape::new();
        shape.add_device("fdc", CLASS.name).unwrap();
        let mut w = StateWriter::new(shape);
        {
            let mut chunk = w.chunk("fdc", CLASS.name, CLASS.version).unwrap();
            restored.fdc.save(&mut chunk).unwrap();
        }
        assert_eq!(
            first,
            w.to_vec().unwrap(),
            "the same state saves the same bytes"
        );

        // And the restored controller carries on where the other stopped.
        assert_eq!(restored.msr() & MSR_CB, MSR_CB);
        assert!(restored.drq.high(), "the transfer is still live");
        assert_eq!(
            restored.dma_in(412),
            sector_of(&image_1440k(), 3)[100..].to_vec()
        );
        assert_eq!(restored.results()[3..].to_vec(), alloc::vec![0, 0, 5, 2]);

        // The other drive's head position came back too.
        restored.command(&[CMD_SENSE_INTERRUPT]);
        assert_eq!(restored.results()[1], 12);
    }
}
