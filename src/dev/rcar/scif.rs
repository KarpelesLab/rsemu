//! A Renesas SCIF / HSCIF — the serial port with FIFOs that Renesas has carried
//! from the SH-3 through the SH-4A into R-Car, on the character-device seam.
//!
//! # Sources
//!
//! * Renesas *SH7780 Hardware Manual*, section 25 ("Serial Communication
//!   Interface with FIFO (SCIF)"): §25.3 for the register descriptions — the
//!   bit layouts of `SCSMR`, `SCSCR`, `SCFSR`, `SCFCR`, `SCFDR`, `SCSPTR` and
//!   `SCLSR`, the "read 1, then write 0" rule for the status flags, and the
//!   trigger-level tables — and §25.5 ("SCIF Interrupt Sources and the DMAC")
//!   for which flag each enable gates. The SH7785 and SH7786 manuals' SCIF
//!   chapters describe the same block.
//! * Renesas *R-Car* series hardware manuals, the SCIF and HSCIF chapters,
//!   for the 32-bit-spaced register map every R-Car part shares with the SH-4A,
//!   the 128-stage HSCIF FIFOs and their trigger levels, and the HSCIF's
//!   sampling rate register `HSSRR`.
//!
//! No emulator, kernel, or boot loader source of any licence was consulted
//! (`ROADMAP.md` §1). Where the manuals were not to hand and this file relies on
//! recollection, it says so next to the constant, and the constant is one a
//! board author can check against the part's own manual.
//!
//! # The register file
//!
//! ```text
//!   0x00  SCSMR   16  serial mode: C/A, CHR, PE, O/E, STOP, CKS[1:0]
//!   0x04  SCBRR    8  bit rate
//!   0x08  SCSCR   16  control: TIE, RIE, TE, RE, REIE, CKE[1:0]
//!   0x0c  SCFTDR   8  transmit FIFO data (write only)
//!   0x10  SCFSR   16  status: ER, TEND, TDFE, BRK, FER, PER, RDF, DR
//!   0x14  SCFRDR   8  receive FIFO data (read only)
//!   0x18  SCFCR   16  FIFO control: RSTRG, RTRG, TTRG, MCE, TFRST, RFRST, LOOP
//!   0x1c  SCFDR   16  FIFO data counts (read only)
//!   0x20  SCSPTR  16  serial port: the RTS/CTS/SCK/TxD pins as GPIO
//!   0x24  SCLSR   16  line status: ORER
//!   0x30  DL      16  frequency division (baud rate generator, where fitted)
//!   0x34  CKS     16  clock select (baud rate generator, where fitted)
//!   0x40  HSSRR   16  HSCIF only: sampling rate, SRE and SRCYC[4:0]
//!   0x54  HSRTRGR 16  HSCIF only: receive FIFO data count trigger
//!   0x58  HSTTRGR 16  HSCIF only: transmit FIFO data count trigger
//! ```
//!
//! Every offset not in that table reads as zero and ignores writes — on the
//! part those are reserved, and a driver that touches one is a driver for a
//! different SCIF flavour (the SH7785's 64-stage `SCTFDR`/`SCRFDR` split, the
//! SCIFA/SCIFB layouts), which is not what this class models. `DL` and `CKS`
//! are the baud rate generator for an external `SCIF_CLK`; they read back what
//! was written and divide nothing. `HSRTRGR`/`HSTTRGR` likewise read back what
//! was written: the trigger levels come from `SCFCR` alone here, because the
//! rule by which the two registers override it was not something this author
//! could state from the manual with confidence. No register called `HSRER`
//! is modelled; none is known to this author at a documented offset.
//!
//! # Access widths
//!
//! The registers sit on 32-bit boundaries and the bus is little-endian on
//! every R-Car. Byte, halfword and word accesses are all accepted, naturally
//! aligned, and each reaches the lanes it covers: a byte read of `SCFSR + 1`
//! returns the error counts in the high byte, and a byte write to it cannot
//! disturb the flags in the low byte. Only an access that covers lane 0 of
//! `SCFRDR` pops the receive FIFO, and only one that covers lane 0 of
//! `SCFTDR` pushes the transmit FIFO.
//!
//! # The status flags
//!
//! §25.3.7's rule, modelled rather than approximated: a flag in `SCFSR`
//! (and `ORER` in `SCLSR`) is cleared by **writing 0 after having read it as
//! 1**. A flag that was read as 0 and set in between is not cleared by the
//! write, which is the whole point of the rule — the event that raced the
//! handler is not lost. Writing 1 changes nothing. Three flags have a second
//! condition:
//!
//! * `TDFE` clears only if the transmit FIFO holds *more* than the `TTRG`
//!   level; otherwise it is set again at once, as the manual says.
//! * `RDF` clears only if the receive FIFO holds *fewer* than the `RTRG`
//!   level.
//! * `DR` clears only once the receive FIFO is empty.
//!
//! `ER`, `BRK`, `FER`, `PER` and the per-FIFO error counts never become set,
//! because a character port cannot deliver a framing error, a parity error or
//! a break. `ORER` can, but only in loopback: the host side pushes back
//! rather than overrunning (see below).
//!
//! # Transmission, and why back pressure is modelled
//!
//! A byte written to `SCFTDR` enters the transmit FIFO and is offered to the
//! [`CharDevice`] immediately, exactly as `uart.ns16550` and `uart.pl011` do.
//! If the host will not take it the byte stays in the FIFO, `SCFDR`'s count
//! stays up, `TDFE` stays clear above the trigger and `TEND` stays clear, and
//! the guest waits exactly as it would on a slow wire. Dropping it instead
//! would make the emulated machine faster than any real one and lose output
//! under load.
//!
//! **Why not a baud-timed transmitter**: the two UARTs already in the tree do
//! not time characters from their divisors, and for the same reason — the
//! character rate is this device's **clock domain**, which the machine file
//! sets, and a device that recomputed its own event period from `SCBRR` would
//! be one more place for guest-programmed state to reach into the time model.
//! What a console needs is that a flag a driver polls eventually changes and
//! that bytes are never lost, and both hold. `SCBRR`, `SCSMR.CKS` and, on an
//! HSCIF, `HSSRR` are stored and reported ([`Scif::baud_rate`]) and do not
//! pace anything.
//!
//! `SCSCR.TE` gates the transmitter: bytes written while it is clear wait in
//! the FIFO, which is what the part does. `SCSCR.RE` gates the receiver: with
//! it clear nothing is taken from the host, so keystrokes typed before a driver
//! enables the port are not lost to it.
//!
//! # The receive data ready flag
//!
//! `DR` is the SCIF's receive timeout: set when fewer bytes than the `RTRG`
//! level are in the FIFO and no more have arrived for 1.5 frames. This model
//! sets it as soon as the FIFO holds any bytes below the trigger level — a
//! timeout of zero frames, earlier than hardware and never later — which is
//! the same simplification `uart.pl011` makes for `RTIS` and for the same
//! reason: the only thing a driver does about it is drain the FIFO.
//!
//! # The interrupt
//!
//! On an SH-4A the SCIF has four request lines (`ERI`, `RXI`, `BRI`, `TXI`);
//! on R-Car they are ORed into one GIC SPI per channel, and that is the single
//! `irq` output here, driven as a level:
//!
//! ```text
//!   irq = (TIE && TDFE)
//!       || (RIE && (RDF || DR))
//!       || ((RIE || REIE) && (ER || BRK || ORER))
//! ```
//!
//! That is §25.5's table: `RIE` enables `RXI` and `ERI`/`BRI`, and `REIE`
//! enables `ERI`/`BRI` on its own. The transmit-end interrupt some later SCIF
//! flavours add (`TEIE`) is not modelled.

use alloc::boxed::Box;
use alloc::collections::VecDeque;
use alloc::string::{String, ToString};
use alloc::sync::Arc;
use core::fmt;

use crate::core::device::{Device, DeviceClass, PropertySpec, RealizeCtx, ResetKind};
use crate::core::error::{BusError, Error, Result};
use crate::core::props::{Props, ValueKind};
use crate::core::sched::{Budget, Consumed};
use crate::core::space::{AccessConstraints, MemAttrs, MemOps, MemResult, Region, RegionRef};
use crate::core::state::{ChunkReader, ChunkWriter, Sink, Source};
use crate::core::sync::{LockRank, Mutex};
use crate::core::value::Width;
use crate::core::wire::{Level, WireSource};
use crate::host::chardev::{CharDevice, ports};
use crate::machine::realize::Instance;

/// The class name a machine description writes.
pub const CLASS_NAME: &str = "rcar.scif";

/// The snapshot chunk version. Bump with the encoding, never on its own.
const STATE_VERSION: u32 = 1;

/// How much address space the register block answers.
///
/// Everything the HSCIF defines lies below `0x5c`; the rest of the window reads
/// as zero. An R-Car puts its SCIF channels 4 KiB apart, and a board that maps
/// this window into such a slot leaves the remainder unmapped, which faults —
/// the part's own behaviour for an access that far out is not documented.
pub const REGISTER_WINDOW_LEN: u64 = 0x100;

/// The character port a machine file gets if it names none.
const DEFAULT_PORT: &str = "console";

/// The one output pin.
pub const IRQ_PIN: &str = "irq";

/// The transmit DMA request: high while the transmitter is enabled and its
/// FIFO has room.
pub const TX_DREQ_PIN: &str = "tx-dreq";

/// The receive DMA request: high while the receive FIFO holds data.
pub const RX_DREQ_PIN: &str = "rx-dreq";

// -- register offsets --------------------------------------------------------

const SCSMR: u64 = 0x00;
const SCBRR: u64 = 0x04;
const SCSCR: u64 = 0x08;
const SCFTDR: u64 = 0x0c;
const SCFSR: u64 = 0x10;
const SCFRDR: u64 = 0x14;
const SCFCR: u64 = 0x18;
const SCFDR: u64 = 0x1c;
const SCSPTR: u64 = 0x20;
const SCLSR: u64 = 0x24;
const DL: u64 = 0x30;
const CKS: u64 = 0x34;
const HSSRR: u64 = 0x40;
const HSRTRGR: u64 = 0x54;
const HSTTRGR: u64 = 0x58;

// -- SCSMR (§25.3.5) ---------------------------------------------------------

/// The bits of `SCSMR` that exist: C/A, CHR, PE, O/E, STOP and CKS[1:0].
/// Bit 2 is reserved.
const SMR_MASK: u16 = 0x00fb;

// -- SCSCR (§25.3.6) ---------------------------------------------------------

/// Transmit interrupt enable: `TXI` when `TDFE` is set.
const SCR_TIE: u16 = 1 << 7;
/// Receive interrupt enable: `RXI` on `RDF`/`DR`, and `ERI`/`BRI`.
const SCR_RIE: u16 = 1 << 6;
/// Transmit enable.
const SCR_TE: u16 = 1 << 5;
/// Receive enable.
const SCR_RE: u16 = 1 << 4;
/// Receive error interrupt enable: `ERI`/`BRI` without `RXI`.
const SCR_REIE: u16 = 1 << 3;
/// The bits of `SCSCR` that exist: the five above and CKE[1:0]. Bit 2 is
/// reserved on the SH7780 SCIF.
const SCR_MASK: u16 = 0x08fb;

/// `SCSCR.TEIE` (bit 11): the transmit-end interrupt, on `TEND`. R-Car's
/// SCIF has it; the SH7780's does not, which is why the mask above grew.
const SCR_TEIE: u16 = 1 << 11;

// -- SCFSR (§25.3.7) ---------------------------------------------------------

/// Receive error: a framing or parity error. Never set here.
const FSR_ER: u16 = 1 << 7;
/// Transmit end: the last stop bit has gone and the FIFO is empty.
const FSR_TEND: u16 = 1 << 6;
/// Transmit FIFO data empty: the FIFO holds no more than the `TTRG` level.
const FSR_TDFE: u16 = 1 << 5;
/// Break detected. Never set here.
const FSR_BRK: u16 = 1 << 4;
/// Receive FIFO data full: the FIFO holds at least the `RTRG` level.
const FSR_RDF: u16 = 1 << 1;
/// Receive data ready: data below the trigger level and no more arriving.
const FSR_DR: u16 = 1 << 0;
/// The flags a write of 0 can clear (after a read of 1). `FER` and `PER`
/// (bits 3 and 2) are read-only and follow the byte at the head of the FIFO.
const FSR_CLEARABLE: u16 = FSR_ER | FSR_TEND | FSR_TDFE | FSR_BRK | FSR_RDF | FSR_DR;
/// `SCFSR`'s reset value: `TEND` and `TDFE` set, the transmitter idle.
const FSR_RESET: u16 = FSR_TEND | FSR_TDFE;

// -- SCFCR (§25.3.8) ---------------------------------------------------------

/// Loopback: TxD is tied to RxD inside the part.
const FCR_LOOP: u16 = 1 << 0;
/// Receive FIFO reset, held while the bit is 1.
const FCR_RFRST: u16 = 1 << 1;
/// Transmit FIFO reset, held while the bit is 1.
const FCR_TFRST: u16 = 1 << 2;
/// The bits of `SCFCR` that exist: RSTRG[2:0], RTRG[1:0], TTRG[1:0], MCE,
/// TFRST, RFRST, LOOP.
const FCR_MASK: u16 = 0x07ff;

// -- SCLSR (§25.3.10) --------------------------------------------------------

/// Overrun error: a byte arrived with the receive FIFO full and was lost.
const LSR_ORER: u16 = 1 << 0;

/// `SCSPTR`'s bits: the four pin pairs, each an I/O direction and a data bit.
const SPTR_MASK: u16 = 0x00ff;
/// `SPB2DT`, which reads the RxD pin when `SPB2IO` is clear. An idle line is
/// a mark, so it reads 1.
const SPTR_SPB2DT: u16 = 1 << 0;
/// `SPB2IO`: whether `SPB2DT` drives TxD rather than reading RxD.
const SPTR_SPB2IO: u16 = 1 << 1;

/// `HSSRR.SRE`: sampling rate register enable.
const HSSRR_SRE: u16 = 1 << 15;
/// `HSSRR`'s bits: `SRE` and `SRCYC[4:0]`.
const HSSRR_MASK: u16 = HSSRR_SRE | 0x001f;

/// Which member of the family this instance is.
///
/// A construction property, not something a guest can change: the FIFO depth,
/// the trigger tables and the width of `SCFDR`'s counts are the silicon.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Variant {
    /// The SCIF: 16-stage FIFOs each way.
    Scif,
    /// The high-speed SCIF: 128-stage FIFOs each way, and `HSSRR`.
    Hscif,
}

impl Variant {
    /// The property value that names it.
    #[must_use]
    pub fn as_str(self) -> &'static str {
        match self {
            Variant::Scif => "scif",
            Variant::Hscif => "hscif",
        }
    }

    /// How many bytes each FIFO holds.
    #[must_use]
    pub fn depth(self) -> usize {
        match self {
            Variant::Scif => 16,
            Variant::Hscif => 128,
        }
    }

    /// The receive trigger level `SCFCR.RTRG` selects.
    ///
    /// SCIF: §25.3.8, 1/4/8/14. HSCIF: 1/32/64/120, from the R-Car HSCIF
    /// chapter as this author recalls it — **check against the part's manual**.
    fn rx_trigger(self, fcr: u16) -> usize {
        let code = usize::from((fcr >> 6) & 3);
        match self {
            Variant::Scif => [1, 4, 8, 14][code],
            Variant::Hscif => [1, 32, 64, 120][code],
        }
    }

    /// The transmit trigger level `SCFCR.TTRG` selects: `TDFE` is set while
    /// the transmit FIFO holds this many bytes or fewer.
    ///
    /// SCIF: §25.3.8, 8/4/2/0. HSCIF: 32/64/96/0, from the R-Car HSCIF
    /// chapter as this author recalls it — **check against the part's manual**.
    fn tx_trigger(self, fcr: u16) -> usize {
        let code = usize::from((fcr >> 4) & 3);
        match self {
            Variant::Scif => [8, 4, 2, 0][code],
            Variant::Hscif => [32, 64, 96, 0][code],
        }
    }

    /// `SCFDR` from the two counts.
    ///
    /// SCIF (§25.3.9): transmit count in bits 12:8, receive count in bits 4:0,
    /// five bits each because a count of 16 needs the fifth. HSCIF: the counts
    /// run to 128, which needs eight bits, so transmit is bits 15:8 and
    /// receive bits 7:0.
    fn fdr(self, tx: usize, rx: usize) -> u16 {
        match self {
            Variant::Scif => ((tx as u16 & 0x1f) << 8) | (rx as u16 & 0x1f),
            Variant::Hscif => ((tx as u16 & 0xff) << 8) | (rx as u16 & 0xff),
        }
    }
}

/// Everything the guest can see or change.
#[derive(Debug, Clone, PartialEq, Eq)]
struct State {
    rx: VecDeque<u8>,
    tx: VecDeque<u8>,
    smr: u16,
    brr: u8,
    scr: u16,
    /// The latched flags of `SCFSR` (the clearable ones).
    fsr: u16,
    /// The `SCFSR` flags a non-debug read last saw as 1: the only ones a
    /// following write of 0 may clear (§25.3.7).
    fsr_seen: u16,
    fcr: u16,
    sptr: u16,
    /// `SCLSR.ORER`.
    lsr: u16,
    /// Whether `ORER` was read as 1, for the same rule.
    lsr_seen: u16,
    dl: u16,
    cks: u16,
    hssrr: u16,
    hsrtrgr: u16,
    hsttrgr: u16,
}

impl State {
    /// The power-on and reset values (§25.3's register tables).
    fn new() -> State {
        State {
            rx: VecDeque::new(),
            tx: VecDeque::new(),
            smr: 0,
            brr: 0xff,
            scr: 0,
            fsr: FSR_RESET,
            fsr_seen: 0,
            fcr: 0,
            sptr: 0,
            lsr: 0,
            lsr_seen: 0,
            dl: 0,
            cks: 0,
            hssrr: 0,
            hsrtrgr: 0,
            hsttrgr: 0,
        }
    }
}

/// The register block, as something an address space can dispatch to.
struct Registers {
    variant: Variant,
    /// TXI and RXI go to the DMA controller rather than the interrupt
    /// controller: the channel's data moves by DMA request, and the CPU hears
    /// only the transmit end and the errors.
    dma: crate::core::sync::AtomicBool,
    state: Mutex<State>,
    /// The interrupt output, at [`LockRank::LEAF`] so it can be taken with
    /// nothing else held.
    out: Mutex<Option<WireSource>>,
    /// The DMA request outputs: transmit and receive.
    dreq: Mutex<[Option<WireSource>; 2]>,
    port: Arc<dyn CharDevice>,
    /// The name the port was opened under, for `Debug` and diagnostics.
    port_name: String,
    /// The functional clock, in hertz, or 0 if the board did not say.
    clock_hz: u32,
}

impl fmt::Debug for Registers {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let mut s = f.debug_struct("Registers");
        s.field("variant", &self.variant)
            .field("port", &self.port_name);
        match self.state.try_lock() {
            Some(state) => s.field("state", &*state).finish(),
            None => s.field("state", &"<in use>").finish(),
        }
    }
}

/// A SCIF or HSCIF channel.
#[derive(Debug)]
pub struct Scif {
    regs: Arc<Registers>,
    region: RegionRef,
}

impl Scif {
    /// Validate `props` and build the device.
    ///
    /// # Errors
    ///
    /// [`Error::Property`] if a property is of the wrong kind or out of range,
    /// or if one this class does not know was given.
    pub fn new(props: &Props) -> Result<Scif> {
        let mut r = props.reader();
        let port_name = r.or("port", String::from(DEFAULT_PORT))?;
        let link = r.optional_str("link")?.map(ToString::to_string);
        let side = r.or_enum("side", "a", &["a", "b"])?;
        let dma = r.or("dma", false)?;
        let clock_hz = r.or_range("clock-hz", 0u64, 0..=u64::from(u32::MAX))?;
        let variant = match r.or_enum("variant", "scif", &["scif", "hscif"])? {
            "hscif" => Variant::Hscif,
            _ => Variant::Scif,
        };
        r.finish()?;
        // A link to another device on the board wins over a host port: the
        // line goes to a chip, not out of the machine.
        if let Some(link) = link {
            let side = crate::bus::uart::Side::parse(side).unwrap_or(crate::bus::uart::Side::A);
            let end = crate::bus::uart::links::attach(props, &link)?.end(side);
            return Ok(Scif::with_port(
                end as Arc<dyn CharDevice>,
                alloc::format!("link:{link}"),
                variant,
                clock_hz as u32,
            )
            .with_dma(dma));
        }
        Ok(Scif::with_port(
            ports::attach(props, &port_name)?,
            port_name,
            variant,
            clock_hz as u32,
        )
        .with_dma(dma))
    }

    /// Build one against a character device the caller already has.
    #[must_use]
    pub fn with_port(
        port: Arc<dyn CharDevice>,
        port_name: String,
        variant: Variant,
        clock_hz: u32,
    ) -> Scif {
        let regs = Arc::new(Registers {
            variant,
            dma: crate::core::sync::AtomicBool::new(false),
            state: Mutex::with_rank(LockRank::DEVICE, State::new()),
            out: Mutex::with_rank(LockRank::LEAF, None),
            dreq: Mutex::with_rank(LockRank::LEAF, [None, None]),
            port,
            port_name,
            clock_hz,
        });
        let region: RegionRef = Arc::new(Region::io(
            "rcar.scif",
            REGISTER_WINDOW_LEN,
            Arc::clone(&regs) as Arc<dyn MemOps>,
        ));
        Scif { regs, region }
    }

    /// The same channel with TXI and RXI routed to the DMA controller instead
    /// of the interrupt controller (the `dma` property).
    #[must_use]
    pub fn with_dma(self, dma: bool) -> Scif {
        self.regs
            .dma
            .store(dma, crate::core::sync::Ordering::Relaxed);
        self
    }

    /// Which member of the family this is.
    #[must_use]
    pub fn variant(&self) -> Variant {
        self.regs.variant
    }

    /// The name of the character port this device is attached to.
    #[must_use]
    pub fn port_name(&self) -> &str {
        &self.regs.port_name
    }

    /// The bit rate the guest has programmed, in bits per second, rounded
    /// down — or `None` if the board gave no `clock-hz`.
    ///
    /// §25.3.4's formula for asynchronous mode, `N = Pφ / (64 × 2^(2n−1) × B)
    /// − 1`, solved for `B`: `Pφ / (16 × 2^(2n+1) × (N + 1))`, with `n` from
    /// `SCSMR.CKS`. On an HSCIF with `HSSRR.SRE` set the 16 samples per bit
    /// become `SRCYC + 1`. Integer arithmetic throughout; this is reporting,
    /// and nothing in the time model divides by it.
    #[must_use]
    pub fn baud_rate(&self) -> Option<u64> {
        if self.regs.clock_hz == 0 {
            return None;
        }
        let state = self.regs.state.lock();
        let n = u32::from(state.smr & 3);
        let samples = if self.regs.variant == Variant::Hscif && state.hssrr & HSSRR_SRE != 0 {
            u64::from(state.hssrr & 0x1f) + 1
        } else {
            16
        };
        let divisor = samples * (1u64 << (2 * n + 1)) * (u64::from(state.brr) + 1);
        Some(u64::from(self.regs.clock_hz) / divisor)
    }

    /// Move bytes between the chip and the host: drain the transmit FIFO and
    /// fill the receive FIFO.
    ///
    /// This is what [`Device::run`] does; a test that is not running a
    /// scheduler calls it directly.
    pub fn pump(&self) {
        self.regs.pump();
    }

    /// Whether the interrupt output is currently asserted.
    #[must_use]
    pub fn irq_asserted(&self) -> bool {
        self.regs.interrupt(&self.regs.state.lock())
    }
}

impl Registers {
    /// The combined interrupt, from the latched flags and the enables.
    ///
    /// With `dma` set, TXI and RXI are the DMA controller's requests (see
    /// [`dreqs`](Registers::dreqs)) and not interrupts.
    fn interrupt(&self, state: &State) -> bool {
        let scr = state.scr;
        let fsr = state.fsr;
        let dma = self.dma.load(crate::core::sync::Ordering::Relaxed);
        let txi = !dma && scr & SCR_TIE != 0 && fsr & FSR_TDFE != 0;
        let rxi = !dma && scr & SCR_RIE != 0 && fsr & (FSR_RDF | FSR_DR) != 0;
        let tei = scr & SCR_TEIE != 0 && fsr & FSR_TEND != 0;
        let eri = scr & (SCR_RIE | SCR_REIE) != 0
            && (fsr & (FSR_ER | FSR_BRK) != 0 || state.lsr & LSR_ORER != 0);
        txi || rxi || tei || eri
    }

    /// Drive the interrupt line. Never called with the state lock held.
    fn drive(&self, asserted: bool) {
        let out = self.out.lock().clone();
        if let Some(out) = out {
            out.set(Level::from_bool(asserted));
        }
    }

    /// The DMA request levels: transmit while the transmitter is on and the
    /// FIFO has room, receive while it holds data. On the silicon these are
    /// the TXI and RXI conditions routed to the DMA controller instead of the
    /// interrupt controller; here they are two wires a DMA controller paces
    /// itself by.
    fn dreqs(&self, state: &State) -> [bool; 2] {
        let tx = state.scr & SCR_TE != 0
            && state.fcr & FCR_TFRST == 0
            && state.tx.len() < self.variant.depth();
        let rx = state.scr & SCR_RE != 0 && state.fcr & FCR_RFRST == 0 && !state.rx.is_empty();
        [tx, rx]
    }

    /// Recompute and drive the interrupt line and the DMA requests from the
    /// current state.
    fn refresh(&self) {
        let (asserted, dreq) = {
            let state = self.state.lock();
            (self.interrupt(&state), self.dreqs(&state))
        };
        self.drive(asserted);
        let outs = self.dreq.lock().clone();
        for (out, level) in outs.iter().zip(dreq) {
            if let Some(out) = out {
                out.set(Level::from_bool(level));
            }
        }
    }

    /// Set the level-conditioned flags whose conditions now hold.
    ///
    /// Setting is automatic; clearing is the guest's, by the read-1-write-0
    /// rule, and is refused while the condition still holds (§25.3.7).
    fn settle(&self, state: &mut State) {
        if state.tx.len() <= self.variant.tx_trigger(state.fcr) {
            state.fsr |= FSR_TDFE;
        }
        let rtrg = self.variant.rx_trigger(state.fcr);
        if state.rx.len() >= rtrg {
            state.fsr |= FSR_RDF;
        } else if !state.rx.is_empty() {
            // The receive timeout, at zero frames: see the module docs.
            state.fsr |= FSR_DR;
        }
    }

    /// Push one byte into the receive FIFO, or record an overrun.
    fn receive(&self, state: &mut State, byte: u8) {
        if state.rx.len() >= self.variant.depth() {
            state.lsr |= LSR_ORER;
            return;
        }
        state.rx.push_back(byte);
    }

    /// Whether the receiver takes bytes: enabled, and not held in reset.
    fn receiving(state: &State) -> bool {
        state.scr & SCR_RE != 0 && state.fcr & FCR_RFRST == 0
    }

    /// Offer the transmit FIFO to the host, and fill the receive FIFO from it.
    ///
    /// In loopback the bytes never leave the chip: they arrive in this
    /// channel's own receiver, which is what the mode is for.
    fn pump(&self) {
        {
            let mut state = self.state.lock();
            let mut moved = false;
            if state.scr & SCR_TE != 0 && state.fcr & FCR_TFRST == 0 {
                while let Some(byte) = state.tx.front().copied() {
                    if state.fcr & FCR_LOOP != 0 {
                        state.tx.pop_front();
                        if Self::receiving(&state) {
                            self.receive(&mut state, byte);
                        }
                        moved = true;
                        continue;
                    }
                    if !self.port.write_byte(byte) {
                        break;
                    }
                    state.tx.pop_front();
                    moved = true;
                }
            }
            if moved && state.tx.is_empty() {
                // The last stop bit has gone with nothing behind it.
                state.fsr |= FSR_TEND;
            }
            if state.fcr & FCR_LOOP == 0 && Self::receiving(&state) {
                // Only as many as fit: bytes the host is still holding are not
                // bytes the receiver lost, so the port pushes back rather than
                // overrunning.
                while state.rx.len() < self.variant.depth() {
                    let Some(byte) = self.port.read_byte() else {
                        break;
                    };
                    state.rx.push_back(byte);
                }
            }
            self.settle(&mut state);
        }
        self.refresh();
    }

    /// Read the 32-bit slot at `slot`. `lanes` is the bit mask of the bytes
    /// the access covers; `debug` suppresses every side effect.
    fn read_register(&self, slot: u64, lanes: u32, debug: bool) -> u32 {
        let mut state = self.state.lock();
        let hscif = self.variant == Variant::Hscif;
        let value: u16 = match slot {
            SCSMR => state.smr,
            SCBRR => u16::from(state.brr),
            SCSCR => state.scr,
            SCFSR => {
                // FER/PER and the error counts are always zero: see the module
                // docs.
                let fsr = state.fsr;
                if !debug {
                    state.fsr_seen = fsr & lanes as u16 & FSR_CLEARABLE;
                }
                fsr
            }
            SCFRDR => {
                if lanes & 0xff == 0 {
                    0
                } else if debug {
                    u16::from(state.rx.front().copied().unwrap_or(0))
                } else {
                    let byte = state.rx.pop_front().unwrap_or(0);
                    u16::from(byte)
                }
            }
            SCFCR => state.fcr,
            SCFDR => self.variant.fdr(state.tx.len(), state.rx.len()),
            SCSPTR => {
                let mut v = state.sptr;
                if v & SPTR_SPB2IO == 0 {
                    // RxD is an input and the line is idle.
                    v |= SPTR_SPB2DT;
                }
                v
            }
            SCLSR => {
                if !debug {
                    state.lsr_seen = state.lsr & lanes as u16;
                }
                state.lsr
            }
            DL => state.dl,
            CKS => state.cks,
            HSSRR if hscif => state.hssrr,
            HSRTRGR if hscif => state.hsrtrgr,
            HSTTRGR if hscif => state.hsttrgr,
            // SCFTDR is write-only, and everything else is reserved.
            _ => 0,
        };
        // Draining SCFRDR below a level is not a clear: the flags stay latched
        // until the guest writes them (§25.3.7), so nothing is settled here.
        u32::from(value)
    }

    /// Write the bits `mask` of the 32-bit slot at `slot`.
    fn write_register(&self, slot: u64, value: u32, mask: u32) {
        let hscif = self.variant == Variant::Hscif;
        let v = value as u16;
        let m = mask as u16;
        let merge = |old: u16, bits: u16| (old & !(m & bits)) | (v & m & bits);
        let mut transmit = false;
        {
            let mut state = self.state.lock();
            match slot {
                SCSMR => state.smr = merge(state.smr, SMR_MASK),
                SCBRR => {
                    if m & 0xff != 0 {
                        state.brr = v as u8;
                    }
                }
                SCSCR => {
                    state.scr = merge(state.scr, SCR_MASK);
                    // Setting TE or RE may release bytes that were waiting.
                    transmit = true;
                }
                SCFTDR => {
                    if m & 0xff != 0 {
                        // TEND clears on a write to SCFTDR (§25.3.7). A byte
                        // written to a full FIFO, or one held in reset, is
                        // lost, as on the part.
                        state.fsr &= !FSR_TEND;
                        if state.fcr & FCR_TFRST == 0 && state.tx.len() < self.variant.depth() {
                            state.tx.push_back(v as u8);
                        }
                        transmit = true;
                    }
                }
                SCFSR => {
                    // Write 0 after reading 1 clears; write 1 does nothing.
                    let zeros = !v & m & state.fsr_seen & FSR_CLEARABLE;
                    let mut clear = zeros;
                    // A clear the condition refuses leaves the flag set and
                    // seen: the driver's next write of 0 may still take it.
                    if state.tx.len() <= self.variant.tx_trigger(state.fcr) {
                        clear &= !FSR_TDFE;
                    }
                    if state.rx.len() >= self.variant.rx_trigger(state.fcr) {
                        clear &= !FSR_RDF;
                    }
                    if !state.rx.is_empty() {
                        clear &= !FSR_DR;
                    }
                    state.fsr &= !clear;
                    state.fsr_seen &= !clear;
                }
                SCFCR => {
                    state.fcr = merge(state.fcr, FCR_MASK);
                    if state.fcr & FCR_TFRST != 0 {
                        state.tx.clear();
                    }
                    if state.fcr & FCR_RFRST != 0 {
                        state.rx.clear();
                    }
                    transmit = true;
                }
                SCSPTR => state.sptr = merge(state.sptr, SPTR_MASK),
                SCLSR => {
                    let zeros = !v & m & state.lsr_seen & LSR_ORER;
                    state.lsr &= !zeros;
                    state.lsr_seen &= !zeros;
                }
                DL => state.dl = merge(state.dl, 0xffff),
                CKS => state.cks = merge(state.cks, 0xffff),
                HSSRR if hscif => state.hssrr = merge(state.hssrr, HSSRR_MASK),
                HSRTRGR if hscif => state.hsrtrgr = merge(state.hsrtrgr, 0xffff),
                HSTTRGR if hscif => state.hsttrgr = merge(state.hsttrgr, 0xffff),
                // SCFRDR and SCFDR are read-only; the rest is reserved.
                _ => {}
            }
            self.settle(&mut state);
        }
        if transmit {
            // Outward, with the state lock released (`CLAUDE.md`, the
            // re-entrancy contract): offering the byte reaches the host.
            self.pump();
        } else {
            self.refresh();
        }
    }
}

/// The byte-lane mask of an access of `len` bytes at `offset`.
fn lane_mask(offset: u64, len: usize) -> u32 {
    let bytes = if len >= 4 {
        0xffff_ffff
    } else {
        (1u32 << (len * 8)) - 1
    };
    bytes << ((offset & 3) * 8)
}

impl MemOps for Registers {
    fn read(&self, offset: u64, dst: &mut [u8], attrs: MemAttrs) -> MemResult {
        if dst.is_empty() || dst.len() > 4 {
            return Err(BusError::BadAccess);
        }
        let lanes = lane_mask(offset, dst.len());
        let value = self.read_register(offset & !3, lanes, attrs.debug);
        let bytes = (value >> ((offset & 3) * 8)).to_le_bytes();
        dst.copy_from_slice(&bytes[..dst.len()]);
        if !attrs.debug {
            // Reading SCFRDR moves the FIFO count, which `SCFDR` shows and a
            // driver polls; the flags stay latched until written.
            self.refresh();
        }
        Ok(())
    }

    fn write(&self, offset: u64, src: &[u8], attrs: MemAttrs) -> MemResult {
        if src.is_empty() || src.len() > 4 {
            return Err(BusError::BadAccess);
        }
        if attrs.debug {
            // A debug write to SCFTDR would put a character on the console and
            // to SCFSR would acknowledge an interrupt the guest has not seen.
            // Neither can be made harmless (`ROADMAP.md` §15, invariant 5).
            return Err(BusError::BadAccess);
        }
        let mut value = 0u32;
        for (i, byte) in src.iter().enumerate() {
            value |= u32::from(*byte) << (i * 8);
        }
        let shift = (offset & 3) * 8;
        self.write_register(offset & !3, value << shift, lane_mask(offset, src.len()));
        Ok(())
    }

    fn constraints(&self) -> AccessConstraints {
        // Drivers reach the 8-bit registers with byte accesses and the 16-bit
        // ones with halfword accesses; a word access is legal on the
        // peripheral bus and gets the whole slot.
        AccessConstraints::IO
            .with_widths(Width::U8, Width::U32)
            .with_natural_alignment(true)
    }
}

/// The `rcar.scif` device class.
pub static CLASS: DeviceClass = DeviceClass {
    name: CLASS_NAME,
    version: STATE_VERSION,
    summary: "Renesas SCIF/HSCIF serial port with FIFOs, on a character port",
    properties: &[
        PropertySpec {
            name: "port",
            kind: ValueKind::Str,
            required: false,
            summary: "the character port to attach to, by name (default \"console\")",
        },
        PropertySpec {
            name: "link",
            kind: ValueKind::Str,
            required: false,
            summary: "a serial link to another device on the board, by name; wins over `port`",
        },
        PropertySpec {
            name: "side",
            kind: ValueKind::Str,
            required: false,
            summary: "which end of the `link` this is: \"a\" (the default) or \"b\"",
        },
        PropertySpec {
            name: "dma",
            kind: ValueKind::Bool,
            required: false,
            summary: "TXI/RXI request the DMA controller (tx-dreq/rx-dreq) instead of interrupting",
        },
        PropertySpec {
            name: "clock-hz",
            kind: ValueKind::Uint,
            required: false,
            summary: "the functional clock in Hz, for reporting the programmed baud rate (default 0: unknown)",
        },
        PropertySpec {
            name: "variant",
            kind: ValueKind::Str,
            required: false,
            summary: "\"scif\" (16-byte FIFOs, the default) or \"hscif\" (128-byte FIFOs, HSSRR)",
        },
    ],
    construct: |props| Ok(Box::new(Scif::new(props)?)),
};

impl Device for Scif {
    fn class(&self) -> &'static DeviceClass {
        &CLASS
    }

    fn realize(&self, _ctx: &mut RealizeCtx<'_>) -> Result<()> {
        // The output idles low and a fresh net is already low.
        Ok(())
    }

    fn reset(&self, _kind: ResetKind) {
        *self.regs.state.lock() = State::new();
        self.regs.drive(false);
    }

    fn region(&self, name: &str) -> Option<RegionRef> {
        matches!(name, "" | "regs").then(|| Arc::clone(&self.region))
    }

    fn connect(&self, port: &str, source: WireSource) -> Result<()> {
        let dreq = match port {
            TX_DREQ_PIN => Some(0),
            RX_DREQ_PIN => Some(1),
            _ => None,
        };
        if let Some(n) = dreq {
            self.regs.dreq.lock()[n] = Some(source);
            return Ok(());
        }
        if port != IRQ_PIN {
            return Err(Error::Config {
                at: port.to_string(),
                message: String::from(
                    "a SCIF drives one pin, `irq` — ERI, RXI, BRI and TXI ORed, as R-Car wires them",
                ),
            });
        }
        *self.regs.out.lock() = Some(source);
        Ok(())
    }

    fn announce(&self, port: &str) {
        if matches!(port, IRQ_PIN | TX_DREQ_PIN | RX_DREQ_PIN) {
            self.regs.refresh();
        }
    }

    fn is_runnable(&self) -> bool {
        // Not because it executes anything, but because the receiver has to be
        // filled from the host and a refused transmission retried, and the
        // scheduler is the only thing allowed to decide when.
        true
    }

    fn run(&self, budget: Budget) -> Consumed {
        self.regs.pump();
        Consumed::new(budget.ticks)
    }

    fn save(&self, w: &mut ChunkWriter<'_>) -> Result<()> {
        let state = self.regs.state.lock();
        for fifo in [&state.rx, &state.tx] {
            w.write_seq_len(fifo.len() as u64)?;
            for byte in fifo {
                w.write_u8(*byte)?;
            }
        }
        w.write_u8(state.brr)?;
        for half in [
            state.smr,
            state.scr,
            state.fsr,
            state.fsr_seen,
            state.fcr,
            state.sptr,
            state.lsr,
            state.lsr_seen,
            state.dl,
            state.cks,
            state.hssrr,
            state.hsrtrgr,
            state.hsttrgr,
        ] {
            w.write_u16(half)?;
        }
        Ok(())
        // The port's queues are the host's state, not the machine's, and are
        // deliberately absent (`ROADMAP.md` §4.5). The variant is a
        // construction property and is the machine file's to restate.
    }

    fn load(&self, r: &mut ChunkReader<'_>) -> Result<()> {
        let depth = self.regs.variant.depth();
        let mut state = State::new();
        for which in 0..2 {
            let count = r.read_seq_len(1)? as usize;
            if count > depth {
                return Err(Error::State(alloc::format!(
                    "snapshot has {count} byte(s) in a {depth}-byte {} FIFO",
                    self.regs.variant.as_str()
                )));
            }
            let fifo = if which == 0 {
                &mut state.rx
            } else {
                &mut state.tx
            };
            for _ in 0..count {
                fifo.push_back(r.read_u8()?);
            }
        }
        state.brr = r.read_u8()?;
        state.smr = r.read_u16()?;
        state.scr = r.read_u16()?;
        state.fsr = r.read_u16()?;
        state.fsr_seen = r.read_u16()?;
        state.fcr = r.read_u16()?;
        state.sptr = r.read_u16()?;
        state.lsr = r.read_u16()?;
        state.lsr_seen = r.read_u16()?;
        state.dl = r.read_u16()?;
        state.cks = r.read_u16()?;
        state.hssrr = r.read_u16()?;
        state.hsrtrgr = r.read_u16()?;
        state.hsttrgr = r.read_u16()?;
        *self.regs.state.lock() = state;
        self.regs.refresh();
        Ok(())
    }
}

impl Instance for Scif {}

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
    bindings.bind(CLASS_NAME, |props| Ok(Arc::new(Scif::new(props)?)))
}

/// What the validator should know about `rcar.scif`.
#[must_use]
pub fn schema() -> crate::machine::validate::ClassSchema {
    use crate::machine::validate::{ClassSchema, PortDir, PropSchema};
    ClassSchema::new(CLASS_NAME)
        .prop(PropSchema::new("port", ValueKind::Str))
        .prop(PropSchema::new("link", ValueKind::Str))
        .prop(PropSchema::new("dma", ValueKind::Bool))
        .prop(PropSchema::new("side", ValueKind::Str).values(&["a", "b"]))
        .prop(PropSchema::new("clock-hz", ValueKind::Uint).range(0, u64::from(u32::MAX)))
        .prop(PropSchema::new("variant", ValueKind::Str).values(&["scif", "hscif"]))
        .region("")
        .region("regs")
        .port(IRQ_PIN, PortDir::Out)
        .port(TX_DREQ_PIN, PortDir::Out)
        .port(RX_DREQ_PIN, PortDir::Out)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::state::{MachineShape, Migrations, StateReader, StateWriter};
    use crate::core::sync::{AtomicU32, Ordering};
    use crate::core::wire::{Wire, WireId, WireIdAllocator, WireSink};
    use crate::host::chardev::CharPort;
    use alloc::vec::Vec;

    fn wired(variant: Variant) -> (Scif, Arc<CharPort>) {
        let port = Arc::new(CharPort::new());
        let scif = Scif::with_port(
            Arc::clone(&port) as Arc<dyn CharDevice>,
            "test".to_string(),
            variant,
            66_666_666,
        );
        (scif, port)
    }

    fn read(s: &Scif, offset: u64, len: usize) -> u32 {
        let mut buf = [0u8; 4];
        s.regs
            .read(offset, &mut buf[..len], MemAttrs::DEFAULT)
            .expect("a register read is legal");
        u32::from_le_bytes(buf)
    }

    fn write(s: &Scif, offset: u64, len: usize, value: u32) {
        s.regs
            .write(offset, &value.to_le_bytes()[..len], MemAttrs::DEFAULT)
            .expect("a register write is legal");
    }

    fn fsr(s: &Scif) -> u16 {
        read(s, SCFSR, 2) as u16
    }

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

    fn with_irq(variant: Variant) -> (Scif, Arc<CharPort>, Arc<Probe>) {
        let (scif, port) = wired(variant);
        let ids = WireIdAllocator::new();
        let id = ids.alloc();
        let probe = Arc::new(Probe::default());
        let wire = Wire::builder()
            .source(id)
            .sink(Arc::clone(&probe) as Arc<dyn WireSink>, 0)
            .build_shared();
        scif.connect("irq", WireSource::new(wire, id))
            .expect("a SCIF drives irq");
        (scif, port, probe)
    }

    fn level(p: &Probe) -> u32 {
        p.level.load(Ordering::Relaxed)
    }

    #[test]
    fn registers_come_out_of_reset_with_the_manual_values() {
        let (s, _port) = wired(Variant::Scif);
        assert_eq!(read(&s, SCSMR, 2), 0);
        assert_eq!(read(&s, SCBRR, 1), 0xff);
        assert_eq!(read(&s, SCSCR, 2), 0);
        assert_eq!(read(&s, SCFSR, 2), 0x0060, "TEND and TDFE");
        assert_eq!(read(&s, SCFCR, 2), 0);
        assert_eq!(read(&s, SCFDR, 2), 0);
        assert_eq!(read(&s, SCLSR, 2), 0);
        assert_eq!(read(&s, 0x80, 4), 0, "a reserved offset reads zero");
        write(&s, 0x80, 4, 0xdead_beef);
        assert_eq!(read(&s, 0x80, 4), 0, "and ignores writes");
        assert_eq!(read(&s, HSSRR, 2), 0, "no HSSRR on a plain SCIF");
    }

    #[test]
    fn hello_reaches_the_host() {
        let (s, port) = wired(Variant::Scif);
        write(&s, SCSCR, 2, u32::from(SCR_TE));
        for &b in b"hello" {
            assert_ne!(fsr(&s) & FSR_TDFE, 0, "room in the FIFO");
            write(&s, SCFTDR, 1, u32::from(b));
        }
        assert_eq!(port.drain(), b"hello".to_vec());
        assert_ne!(fsr(&s) & FSR_TEND, 0, "and the transmitter is idle again");
        assert_eq!(read(&s, SCFDR, 2) >> 8, 0);
    }

    #[test]
    fn bytes_wait_for_te_and_for_a_host_that_takes_them() {
        let (s, port) = wired(Variant::Scif);
        write(&s, SCFTDR, 1, u32::from(b'x'));
        assert!(port.drain().is_empty(), "TE is clear");
        assert_eq!(read(&s, SCFDR, 2) >> 8, 1);
        assert_eq!(fsr(&s) & FSR_TEND, 0, "writing SCFTDR cleared TEND");

        while port.writable() {
            port.write(b".");
        }
        write(&s, SCSCR, 2, u32::from(SCR_TE));
        write(&s, SCFTDR, 1, u32::from(b'y'));
        assert_eq!(read(&s, SCFDR, 2) >> 8, 2, "the host refused both");
        let _ = port.drain();
        s.pump();
        assert_eq!(port.drain(), b"xy".to_vec(), "and nothing was lost");
        assert_ne!(fsr(&s) & FSR_TEND, 0);
    }

    #[test]
    fn a_received_byte_raises_the_interrupt_with_rie() {
        let (s, port, probe) = with_irq(Variant::Scif);
        write(&s, SCSCR, 2, u32::from(SCR_RE));
        port.feed(b"k");
        s.pump();
        assert_ne!(fsr(&s) & FSR_RDF, 0, "trigger level 1 is reached");
        assert_eq!(read(&s, SCFDR, 2) & 0x1f, 1);
        assert_eq!(level(&probe), 0, "not enabled yet");

        write(&s, SCSCR, 2, u32::from(SCR_RE | SCR_RIE));
        assert_eq!(level(&probe), 1);
        // The driver's handler: read the data, then clear RDF and DR.
        let _ = fsr(&s);
        assert_eq!(read(&s, SCFRDR, 1), u32::from(b'k'));
        write(&s, SCFSR, 2, u32::from(!(FSR_RDF | FSR_DR)));
        assert_eq!(fsr(&s) & (FSR_RDF | FSR_DR), 0);
        assert_eq!(level(&probe), 0);
    }

    #[test]
    fn rdf_cannot_be_cleared_while_the_fifo_is_at_the_trigger() {
        let (s, port) = wired(Variant::Scif);
        write(&s, SCSCR, 2, u32::from(SCR_RE));
        port.feed(b"ab");
        s.pump();
        let _ = fsr(&s);
        write(&s, SCFSR, 2, u32::from(!FSR_RDF));
        assert_ne!(fsr(&s) & FSR_RDF, 0, "two bytes are still at trigger 1");
    }

    #[test]
    fn a_flag_not_read_as_one_is_not_cleared_by_writing_zero() {
        let (s, _port) = wired(Variant::Scif);
        // TEND is set out of reset, but the guest has not read SCFSR.
        write(&s, SCFSR, 2, 0);
        assert_ne!(fsr(&s) & FSR_TEND, 0);
        // Having read it, the same write now clears it.
        write(&s, SCFSR, 2, 0);
        assert_eq!(fsr(&s) & FSR_TEND, 0);
        // TDFE stays: the empty FIFO is at or below the trigger.
        assert_ne!(fsr(&s) & FSR_TDFE, 0);
    }

    #[test]
    fn the_below_trigger_bytes_set_dr() {
        let (s, port) = wired(Variant::Scif);
        // RTRG = 01: trigger at 4 bytes.
        write(&s, SCFCR, 2, 1 << 6);
        write(&s, SCSCR, 2, u32::from(SCR_RE));
        port.feed(b"ab");
        s.pump();
        let f = fsr(&s);
        assert_eq!(f & FSR_RDF, 0);
        assert_ne!(f & FSR_DR, 0);
        write(&s, SCFSR, 2, u32::from(!FSR_DR));
        assert_ne!(fsr(&s) & FSR_DR, 0, "data is still there");
        let _ = read(&s, SCFRDR, 1);
        let _ = read(&s, SCFRDR, 1);
        write(&s, SCFSR, 2, u32::from(!FSR_DR));
        assert_eq!(fsr(&s) & FSR_DR, 0);
    }

    #[test]
    fn fifo_reset_bits_empty_the_fifos() {
        let (s, port) = wired(Variant::Scif);
        write(&s, SCSCR, 2, u32::from(SCR_RE));
        port.feed(b"abc");
        s.pump();
        write(&s, SCFTDR, 1, u32::from(b'z'));
        assert_eq!(read(&s, SCFDR, 2), (1 << 8) | 3);
        write(&s, SCFCR, 2, u32::from(FCR_RFRST | FCR_TFRST));
        assert_eq!(read(&s, SCFDR, 2), 0);
        // Held in reset: a byte written now is lost, and nothing is received.
        write(&s, SCFTDR, 1, u32::from(b'q'));
        port.feed(b"d");
        s.pump();
        assert_eq!(read(&s, SCFDR, 2), 0);
        write(&s, SCFCR, 2, 0);
        s.pump();
        assert_eq!(read(&s, SCFDR, 2), 1, "released, the receiver takes 'd'");
        write(&s, SCSCR, 2, u32::from(SCR_TE | SCR_RE));
        assert!(port.drain().is_empty(), "'z' and 'q' never went");
    }

    #[test]
    fn a_debug_read_pops_nothing_and_arms_no_clear() {
        let (s, port) = wired(Variant::Scif);
        write(&s, SCSCR, 2, u32::from(SCR_RE));
        port.feed(b"k");
        s.pump();
        let mut b = [0u8; 1];
        s.regs.read(SCFRDR, &mut b, MemAttrs::DEBUG).unwrap();
        assert_eq!(b[0], b'k');
        assert_eq!(read(&s, SCFDR, 2) & 0x1f, 1, "still there");

        // A debugger reading SCFSR must not make the guest's next 0 a clear.
        let (t, _p) = wired(Variant::Scif);
        let mut h = [0u8; 2];
        t.regs.read(SCFSR, &mut h, MemAttrs::DEBUG).unwrap();
        write(&t, SCFSR, 2, 0);
        assert_ne!(fsr(&t) & FSR_TEND, 0);

        assert!(s.regs.write(SCFTDR, b"x", MemAttrs::DEBUG).is_err());
    }

    #[test]
    fn byte_and_halfword_accesses_reach_their_lanes() {
        let (s, _port) = wired(Variant::Scif);
        write(&s, SCSCR, 1, 0x30);
        assert_eq!(read(&s, SCSCR, 2), 0x30);
        assert_eq!(read(&s, SCSCR, 4), 0x30);
        // A byte write to the high half of SCFSR cannot clear the flags.
        let _ = fsr(&s);
        write(&s, SCFSR + 1, 1, 0);
        assert_ne!(fsr(&s) & FSR_TEND, 0);
        // A byte read of the high half of SCBRR's slot reads zero.
        assert_eq!(read(&s, SCBRR + 1, 1), 0);
        assert!(s.regs.read(0, &mut [0u8; 8], MemAttrs::DEFAULT).is_err());
    }

    #[test]
    fn transmit_interrupt_follows_tdfe_and_tie() {
        let (s, _port, probe) = with_irq(Variant::Scif);
        write(&s, SCSCR, 2, u32::from(SCR_TIE));
        assert_eq!(level(&probe), 1, "the empty FIFO asks for data");
        write(&s, SCSCR, 2, 0);
        assert_eq!(level(&probe), 0);
    }

    #[test]
    fn loopback_overruns_into_orer() {
        let (s, port, probe) = with_irq(Variant::Scif);
        write(&s, SCFCR, 2, u32::from(FCR_LOOP));
        write(&s, SCSCR, 2, u32::from(SCR_TE | SCR_RE | SCR_REIE));
        for _ in 0..=16 {
            write(&s, SCFTDR, 1, u32::from(b'L'));
        }
        assert!(port.drain().is_empty(), "nothing reaches the host");
        assert_eq!(read(&s, SCFDR, 2) & 0x1f, 16);
        assert_eq!(read(&s, SCLSR, 2), u32::from(LSR_ORER));
        assert_eq!(level(&probe), 1, "REIE alone enables the overrun");
        write(&s, SCLSR, 2, 0);
        assert_eq!(read(&s, SCLSR, 2), 0);
    }

    #[test]
    fn hscif_has_deep_fifos_and_wide_counts() {
        let (s, port) = wired(Variant::Hscif);
        write(&s, SCSCR, 2, u32::from(SCR_RE));
        let bytes: Vec<u8> = (0..200u32).map(|i| i as u8).collect();
        port.feed(&bytes);
        s.pump();
        assert_eq!(read(&s, SCFDR, 2), 128, "receive count in bits 7:0");
        write(&s, HSSRR, 2, 0x8007);
        assert_eq!(read(&s, HSSRR, 2), 0x8007);
        // 8 samples, n = 0, N = 0: 66.67 MHz / (8 * 2 * 1).
        write(&s, SCBRR, 1, 0);
        assert_eq!(s.baud_rate(), Some(66_666_666 / 16));
    }

    #[test]
    fn baud_rate_follows_the_manual_formula() {
        let (s, _port) = wired(Variant::Scif);
        // Pφ = 66.67 MHz, n = 0, N = 17: 66_666_666 / (32 * 18) = 115740.
        write(&s, SCBRR, 1, 17);
        assert_eq!(s.baud_rate(), Some(115_740));
    }

    fn snapshot(s: &Scif) -> Vec<u8> {
        let mut shape = MachineShape::new();
        shape.add_device("scif", CLASS.name).unwrap();
        let mut w = StateWriter::new(shape);
        {
            let mut chunk = w.chunk("scif", CLASS.name, CLASS.version).unwrap();
            s.save(&mut chunk).unwrap();
        }
        w.to_vec().unwrap()
    }

    #[test]
    fn a_snapshot_round_trips_to_identical_state() {
        let (saved, port) = wired(Variant::Scif);
        write(&saved, SCSCR, 2, u32::from(SCR_RE | SCR_RIE));
        write(&saved, SCBRR, 1, 17);
        write(&saved, SCSMR, 2, 0x01);
        write(&saved, SCFCR, 2, 1 << 6);
        port.feed(b"abc");
        saved.pump();
        write(&saved, SCFTDR, 1, u32::from(b'q'));
        let _ = fsr(&saved);
        let bytes = snapshot(&saved);

        let (restored, _other) = wired(Variant::Scif);
        let reader = StateReader::new(&bytes).unwrap();
        let chunk = reader
            .load("scif", CLASS.name, CLASS.version, &Migrations::new())
            .unwrap();
        restored.load(&mut chunk.reader()).unwrap();
        let want = saved.regs.state.lock().clone();
        assert_eq!(*restored.regs.state.lock(), want);
        assert_eq!(snapshot(&restored), bytes, "identical state bytes");
        assert!(restored.irq_asserted());
        assert_eq!(read(&restored, SCFRDR, 1), u32::from(b'a'));
    }

    #[test]
    fn a_board_realizes_it_and_the_scheduler_pumps_it() {
        use crate::core::clock::GlobalTime;
        use crate::core::value::Width;

        let mut options = crate::machine::BuildOptions::new();
        options.classes.insert(schema());
        for s in crate::machine::builtin::schemas() {
            options.classes.insert(s);
        }
        bind(&mut options.bindings).expect("nothing else claims rcar.scif");
        crate::machine::builtin::bind(&mut options.bindings).expect("ram");
        let mut registry = crate::core::Registry::new();
        register(&mut registry).expect("nothing else claims rcar.scif");
        crate::machine::builtin::register(&mut registry).expect("ram");

        let text = concat!(
            "machine \"m\" {\n",
            "  osc xtal = 14745600 Hz\n",
            "  space mem { width = 32 }\n",
            "  object sram \"ram\" { size = 4K }\n",
            "  object scif0 \"rcar.scif\" { clock = xtal / 1280, port = console, clock-hz = 66666666 }\n",
            "  map mem 0x20000000 size 4K = sram\n",
            "  map mem 0xffe40000 size 0x100 = scif0\n",
            "}\n"
        );
        let mut machine = crate::machine::build("t.machine", text, &registry, &options)
            .expect("the board builds");
        let port = ports::get(&options.realize.hosts, "console")
            .expect("a port table")
            .expect("the SCIF opened its port");
        let space = Arc::clone(machine.space("mem").expect("mem"));
        let base = 0xffe4_0000u64;
        space
            .write(
                base + SCSCR,
                Width::U16,
                u64::from(SCR_TE | SCR_RE),
                MemAttrs::DEFAULT,
            )
            .expect("SCSCR");
        for &b in b"hi" {
            space
                .write(base + SCFTDR, Width::U8, u64::from(b), MemAttrs::DEFAULT)
                .expect("SCFTDR");
        }
        assert_eq!(port.drain(), b"hi".to_vec());

        port.feed(b"k");
        machine
            .run_for(GlobalTime::from_nanos(1_000_000))
            .expect("the machine runs");
        let fdr = space
            .read(base + SCFDR, Width::U16, MemAttrs::DEFAULT)
            .expect("SCFDR");
        assert_eq!(fdr & 0x1f, 1, "the scheduler's pump filled the FIFO");
        let byte = space
            .read(base + SCFRDR, Width::U8, MemAttrs::DEFAULT)
            .expect("SCFRDR");
        assert_eq!(byte, u64::from(b'k'));
    }

    #[test]
    fn properties_are_checked_rather_than_ignored() {
        let s = Scif::new(&Props::new().with("variant", "hscif")).expect("hscif is legal");
        assert_eq!(s.variant(), Variant::Hscif);
        assert_eq!(s.port_name(), DEFAULT_PORT);
        assert_eq!(s.baud_rate(), None, "no clock given");
        assert!(Scif::new(&Props::new().with("variant", "scifa")).is_err());
        assert!(Scif::new(&Props::new().with("prot", "x")).is_err());
    }
}
