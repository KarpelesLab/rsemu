//! The ARM PrimeCell UART (PL011), on the character-device seam.
//!
//! # Source
//!
//! *PrimeCell UART (PL011) Technical Reference Manual*, ARM DDI 0183 — the
//! register summary of chapter 3 and the operation of chapter 2. The
//! identification registers at the top of the window come from the same
//! chapter; they are not decoration, and §"Why the identification registers
//! matter" below says why. No driver of any licence was read.
//!
//! # It is not a 16550 with different offsets
//!
//! [`dev::uart::ns16550`](crate::dev::uart::ns16550) already exists and this
//! is deliberately not a variant of it. The two chips share the *idea* of a
//! UART and nothing else:
//!
//! | | 16550 | PL011 |
//! | --- | --- | --- |
//! | register width | 8 bits, three address lines | 32 bits, a 4 KiB window |
//! | which interrupt | one `IIR` register, a priority encoder | eleven raw bits, masked and cleared individually |
//! | acknowledging one | a side effect of reading `IIR` or `LSR` | an explicit write to `UARTICR` |
//! | baud | a 16-bit divisor latch behind `DLAB` | separate integer and fractional divisors |
//! | FIFO | on or off, one bit | on or off, plus a programmable trigger level each way |
//! | how software finds it | writes the scratch register and reads it back | reads eight identification registers |
//!
//! # The register file (TRM chapter 3)
//!
//! ```text
//!   0x000  UARTDR       data: the byte, plus the four error flags on a read
//!   0x004  UARTRSR/ECR  the same four error flags; any write clears them
//!   0x018  UARTFR       flags: RXFE, TXFF, RXFF, TXFE, BUSY
//!   0x020  UARTILPR     IrDA low-power counter (stored, never used)
//!   0x024  UARTIBRD     baud rate divisor, integer part
//!   0x028  UARTFBRD     baud rate divisor, fractional part
//!   0x02c  UARTLCR_H    line control: word length, FIFO enable, parity
//!   0x030  UARTCR       control: UARTEN, TXE, RXE, LBE
//!   0x034  UARTIFLS     FIFO level select: when RXIS and TXIS assert
//!   0x038  UARTIMSC     interrupt mask set/clear
//!   0x03c  UARTRIS      raw interrupt status
//!   0x040  UARTMIS      masked interrupt status: RIS & IMSC
//!   0x044  UARTICR      interrupt clear: write ones to clear
//!   0x048  UARTDMACR    DMA control (stored, never used)
//!   0xfe0  UARTPeriphID0-3, then UARTPCellID0-3 at 0xff0
//! ```
//!
//! # Why the identification registers matter
//!
//! On a device tree platform a PL011 is an **AMBA** peripheral, and an AMBA
//! bus does not trust `compatible` alone: it reads the four peripheral
//! identification registers and the four PrimeCell identification registers
//! out of the top of the device's own window and matches the driver against
//! the part number it finds there. A model that returned zero for them is a
//! model whose driver never binds, and the failure looks like "the console
//! node is in the tree and nothing came out of it" — which is the most
//! expensive kind of wrong. So [`PERIPH_ID`] and [`PCELL_ID`] are part of the
//! model, not an afterthought.
//!
//! # Transmission, and why back pressure is modelled
//!
//! A write to `UARTDR` is offered to the [`CharDevice`] immediately. If the
//! host will not take it — a full pipe, a terminal nobody is draining — the
//! byte stays in the transmit FIFO, `TXFF` reads set, and the guest waits
//! exactly as it would on real hardware. Dropping it instead would make the
//! emulated machine faster than any real one and lose output under load. The
//! 16550 model makes the same argument at the same length; it is the same
//! argument.
//!
//! # The receive timeout interrupt
//!
//! `RTIS` (bit 6) is the one a naive model leaves out and then spends an
//! afternoon on. The FIFO trigger level defaults to half full, so a driver
//! that has been handed three characters is never told about them by `RXIS`
//! alone — the receive *timeout* is what says "there is something in the FIFO
//! and no more is coming". This model asserts it whenever the receive FIFO is
//! non-empty, which is a timeout of zero bit periods: earlier than hardware,
//! never later, and a driver cannot tell the difference because the only thing
//! it does about it is drain the FIFO.

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
use crate::core::wire::{Level, WireId, WireSource};
use crate::host::chardev::{CharDevice, ports};
use crate::machine::realize::Instance;

use super::dt::{DtSource, NodeKind, NodeSpec};

/// The class name a machine description writes.
pub const CLASS_NAME: &str = "uart.pl011";

/// The snapshot chunk version. Bump with the encoding, never on its own.
const STATE_VERSION: u32 = 1;

/// How much address space the register block answers (TRM chapter 3).
pub const REGISTER_WINDOW_LEN: u64 = 0x1000;

/// How deep each FIFO is (TRM §2.3: 16 entries each way).
pub const FIFO_DEPTH: usize = 16;

/// The character port a machine file gets if it names none.
const DEFAULT_PORT: &str = "console";

/// The `UARTCLK` rate a board reports to its guest by default, in hertz.
///
/// Nothing here divides by it: the character rate is this device's clock
/// domain and the machine file sets that. It exists because a device tree has
/// to name a clock for the driver to enable, and because a driver that
/// computes a divisor from it should get a sane baud rate rather than a
/// division by zero.
const DEFAULT_CLOCK_HZ: u32 = 24_000_000;

// -- UARTFR, offset 0x018 ---------------------------------------------------

/// Clear to send.
const FR_CTS: u32 = 1 << 0;
/// Data set ready.
const FR_DSR: u32 = 1 << 1;
/// Data carrier detect.
const FR_DCD: u32 = 1 << 2;
/// The transmitter is busy shifting a character out.
const FR_BUSY: u32 = 1 << 3;
/// Receive FIFO empty.
const FR_RXFE: u32 = 1 << 4;
/// Transmit FIFO full.
const FR_TXFF: u32 = 1 << 5;
/// Receive FIFO full.
const FR_RXFF: u32 = 1 << 6;
/// Transmit FIFO empty.
const FR_TXFE: u32 = 1 << 7;

// -- UARTCR, offset 0x030 ---------------------------------------------------

/// UART enable.
///
/// Stored and reported, and deliberately **not** gating transmission. An early
/// console writes `UARTDR` without touching `UARTCR` at all, because on real
/// hardware firmware has already enabled the port — so a model that dropped
/// those bytes would lose exactly the output that matters most, the first
/// forty lines a kernel prints.
pub const CR_UARTEN: u32 = 1 << 0;
/// Loopback enable: the transmitter is wired back to the receiver.
pub const CR_LBE: u32 = 1 << 7;
/// Transmit enable.
pub const CR_TXE: u32 = 1 << 8;
/// Receive enable, which does gate the receiver.
pub const CR_RXE: u32 = 1 << 9;

// -- UARTLCR_H, offset 0x02c ------------------------------------------------

/// FIFO enable. With it clear the FIFOs are one byte deep, which is how a
/// PL011 pretends to be a much older part.
const LCRH_FEN: u32 = 1 << 4;

// -- interrupt bits, shared by RIS, MIS, IMSC and ICR ------------------------

/// Receive interrupt: the receive FIFO reached its trigger level.
const INT_RX: u32 = 1 << 4;
/// Transmit interrupt: the transmit FIFO fell to its trigger level.
const INT_TX: u32 = 1 << 5;
/// Receive timeout: there is data in the receive FIFO and no more is coming.
const INT_RT: u32 = 1 << 6;
/// Overrun error.
const INT_OE: u32 = 1 << 10;
/// Every bit the eleven-bit interrupt registers implement.
const INT_MASK: u32 = 0x7ff;

/// The identification registers at `0xfe0`, one byte each (TRM §3.3).
///
/// Part number `0x011`, designer `0x41` (Arm), revision 2, configuration 0 —
/// the values a PL011 r1p4 reports, split across four registers as
/// `PartNumber0`, `DesignerID0:PartNumber1`, `Revision:DesignerID1` and
/// `Configuration`.
pub const PERIPH_ID: [u8; 4] = [0x11, 0x10, 0x14, 0x00];

/// The PrimeCell identification registers at `0xff0` (TRM §3.3).
///
/// `0xb105f00d` little-endian across four registers, which is the same
/// constant every PrimeCell part reports and is what tells a bus that the
/// four registers below it are a peripheral id at all.
pub const PCELL_ID: [u8; 4] = [0x0d, 0xf0, 0x05, 0xb1];

/// Everything the guest can see or change.
#[derive(Debug, Clone, PartialEq, Eq, Default)]
struct State {
    /// Received bytes waiting to be read.
    rx: VecDeque<u8>,
    /// Bytes the host has not taken yet.
    tx: VecDeque<u8>,
    /// The error flags a `UARTDR` read reports in bits 11:8, and `UARTRSR`
    /// reports in bits 3:0.
    rsr: u32,
    /// Raw interrupt status, the sticky half. `RXIS`, `TXIS` and `RTIS` are
    /// computed from the FIFOs instead — see [`Registers::raw`].
    ris: u32,
    /// Interrupt mask set/clear.
    imsc: u32,
    cr: u32,
    lcr_h: u32,
    ifls: u32,
    ibrd: u32,
    fbrd: u32,
    ilpr: u32,
    dmacr: u32,
}

impl State {
    /// The reset state (TRM chapter 3's reset column).
    fn new() -> State {
        State {
            // Both trigger levels at one half, which is `0b010` in each field.
            ifls: 0x12,
            // `UARTCR` resets with TXE and RXE set and UARTEN clear.
            cr: CR_TXE | CR_RXE,
            ..State::default()
        }
    }

    /// How deep the FIFOs are right now: sixteen with `FEN` set, one without.
    fn depth(&self) -> usize {
        if self.lcr_h & LCRH_FEN != 0 {
            FIFO_DEPTH
        } else {
            1
        }
    }
}

/// The register block, as something an address space can dispatch to.
struct Registers {
    state: Mutex<State>,
    /// The interrupt output, at [`LockRank::LEAF`] so it can be taken with
    /// nothing else held.
    out: Mutex<Option<WireSource>>,
    port: Arc<dyn CharDevice>,
    /// The name the port was opened under, for `Debug` and diagnostics.
    port_name: String,
    /// What the device tree reports as `UARTCLK`.
    clock_hz: u32,
    /// The net the interrupt pin drives, so the board that describes itself to
    /// its guest can look the number up in its interrupt controller's own pin
    /// table rather than have it written down twice.
    irq_wire: Mutex<Option<WireId>>,
}

impl fmt::Debug for Registers {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let mut s = f.debug_struct("Registers");
        s.field("port", &self.port_name);
        match self.state.try_lock() {
            Some(state) => s.field("state", &*state).finish(),
            None => s.field("state", &"<in use>").finish(),
        }
    }
}

/// A PL011 UART.
#[derive(Debug)]
pub struct Pl011 {
    regs: Arc<Registers>,
    region: RegionRef,
}

impl Pl011 {
    /// Validate `props` and build the device.
    ///
    /// # Errors
    ///
    /// [`Error::Property`] if a property is of the wrong kind, or if one this
    /// class does not know was given.
    pub fn new(props: &Props) -> Result<Pl011> {
        let mut r = props.reader();
        let port_name = r.or("port", String::from(DEFAULT_PORT))?;
        let clock = r.or_range(
            "clock-hz",
            u64::from(DEFAULT_CLOCK_HZ),
            1..=u64::from(u32::MAX),
        )?;
        r.finish()?;
        Ok(Pl011::with_port(
            ports::attach(props, &port_name)?,
            port_name,
            clock as u32,
        ))
    }

    /// Build one against a character device the caller already has.
    #[must_use]
    pub fn with_port(port: Arc<dyn CharDevice>, port_name: String, clock_hz: u32) -> Pl011 {
        let regs = Arc::new(Registers {
            state: Mutex::with_rank(LockRank::DEVICE, State::new()),
            out: Mutex::with_rank(LockRank::LEAF, None),
            port,
            port_name,
            clock_hz,
            irq_wire: Mutex::with_rank(LockRank::LEAF, None),
        });
        let region: RegionRef = Arc::new(Region::io(
            CLASS_NAME,
            REGISTER_WINDOW_LEN,
            Arc::clone(&regs) as Arc<dyn MemOps>,
        ));
        Pl011 { regs, region }
    }

    /// The name of the character port this device is attached to.
    #[must_use]
    pub fn port_name(&self) -> &str {
        &self.regs.port_name
    }

    /// What the device tree reports as `UARTCLK`, in hertz.
    #[must_use]
    pub fn clock_hz(&self) -> u32 {
        self.regs.clock_hz
    }

    /// The net the interrupt pin drives, or `None` until a board wires it.
    #[must_use]
    pub fn irq_wire(&self) -> Option<WireId> {
        *self.regs.irq_wire.lock()
    }

    /// Move bytes between the chip and the host.
    ///
    /// This is what [`Device::run`] does; a test that is not running a
    /// scheduler calls it directly.
    pub fn pump(&self) {
        self.regs.pump();
    }

    /// Whether the combined interrupt output is asserted.
    #[must_use]
    pub fn irq_asserted(&self) -> bool {
        let state = self.regs.state.lock();
        Registers::raw(&state) & state.imsc != 0
    }
}

impl Registers {
    /// The receive trigger level, in bytes, from `UARTIFLS` bits 5:3.
    ///
    /// The five encodings are 1/8, 1/4, 1/2, 3/4 and 7/8 of the FIFO; anything
    /// else is reserved and is read here as one half, which is the reset
    /// value.
    fn rx_trigger(state: &State) -> usize {
        let depth = state.depth();
        match (state.ifls >> 3) & 7 {
            0 => depth / 8,
            1 => depth / 4,
            3 => depth * 3 / 4,
            4 => depth * 7 / 8,
            _ => depth / 2,
        }
        .max(1)
    }

    /// The transmit trigger level, from `UARTIFLS` bits 2:0: the interrupt
    /// asserts when the FIFO has fallen *to or below* this many entries.
    fn tx_trigger(state: &State) -> usize {
        let depth = state.depth();
        match state.ifls & 7 {
            0 => depth / 8,
            1 => depth / 4,
            3 => depth * 3 / 4,
            4 => depth * 7 / 8,
            _ => depth / 2,
        }
    }

    /// `UARTRIS`: the sticky bits, plus the three the FIFO levels decide.
    ///
    /// `RXIS`, `TXIS` and `RTIS` are *not* latched on a PL011 — they follow the
    /// FIFO levels, and the driver clears them by moving bytes rather than by
    /// writing `UARTICR`. Modelling them as sticky is the classic way to get a
    /// console that prints one character and then stops.
    fn raw(state: &State) -> u32 {
        let mut ris = state.ris;
        if state.rx.len() >= Self::rx_trigger(state) {
            ris |= INT_RX;
        }
        if !state.rx.is_empty() {
            // The receive timeout: see the module docs for why zero bit
            // periods is the honest simplification.
            ris |= INT_RT;
        }
        if state.tx.len() <= Self::tx_trigger(state) {
            ris |= INT_TX;
        }
        ris
    }

    /// `UARTFR`.
    fn flags(state: &State) -> u32 {
        let depth = state.depth();
        let mut fr = 0;
        if state.rx.is_empty() {
            fr |= FR_RXFE;
        }
        if state.rx.len() >= depth {
            fr |= FR_RXFF;
        }
        if state.tx.is_empty() {
            fr |= FR_TXFE;
        } else {
            // Something is still in the shift register as far as the guest is
            // concerned: the host has not taken it.
            fr |= FR_BUSY;
        }
        if state.tx.len() >= depth {
            fr |= FR_TXFF;
        }
        if state.cr & CR_LBE != 0 {
            // In loopback the modem control outputs appear as the status
            // inputs, which is what a driver's self-test checks.
            fr |= FR_CTS | FR_DSR | FR_DCD;
        }
        fr
    }

    /// Drive the interrupt line. Never called with the state lock held.
    fn drive(&self, asserted: bool) {
        let out = self.out.lock().clone();
        if let Some(out) = out {
            out.set(Level::from_bool(asserted));
        }
    }

    /// Recompute and drive the interrupt line from the current state.
    fn refresh(&self) {
        let asserted = {
            let state = self.state.lock();
            Self::raw(&state) & state.imsc != 0
        };
        self.drive(asserted);
    }

    /// Push one received byte into the FIFO, or record an overrun.
    fn receive(state: &mut State, byte: u8) {
        if state.rx.len() >= state.depth() {
            state.ris |= INT_OE;
            state.rsr |= 1 << 3;
            return;
        }
        state.rx.push_back(byte);
    }

    /// Offer the transmit FIFO to the host, and fill the receive FIFO from it.
    ///
    /// In loopback the bytes never leave the chip at all, which is the point
    /// of the mode.
    fn pump(&self) {
        {
            let mut state = self.state.lock();
            while let Some(byte) = state.tx.front().copied() {
                if state.cr & CR_LBE != 0 {
                    state.tx.pop_front();
                    Self::receive(&mut state, byte);
                    continue;
                }
                if !self.port.write_byte(byte) {
                    break;
                }
                state.tx.pop_front();
            }
            if state.cr & CR_LBE == 0 && state.cr & CR_RXE != 0 {
                while state.rx.len() < state.depth() {
                    let Some(byte) = self.port.read_byte() else {
                        break;
                    };
                    state.rx.push_back(byte);
                }
            }
        }
        self.refresh();
    }

    /// Read one register. `debug` suppresses every side effect.
    fn read_register(&self, offset: u64, debug: bool) -> u32 {
        // The identification registers first: they are constants and are read
        // before anything else exists, by a bus deciding whether to bind a
        // driver at all.
        if (0xfe0..0xff0).contains(&offset) {
            return u32::from(PERIPH_ID[((offset - 0xfe0) / 4) as usize]);
        }
        if (0xff0..0x1000).contains(&offset) {
            return u32::from(PCELL_ID[((offset - 0xff0) / 4) as usize]);
        }
        let mut state = self.state.lock();
        match offset {
            0x000 => {
                let errors = (state.rsr & 0xf) << 8;
                if debug {
                    // A debugger read must not pop the FIFO (`ROADMAP.md`
                    // §15, invariant 5).
                    return u32::from(state.rx.front().copied().unwrap_or(0)) | errors;
                }
                let byte = state.rx.pop_front().unwrap_or(0);
                u32::from(byte) | errors
            }
            0x004 => state.rsr & 0xf,
            0x018 => Self::flags(&state),
            0x020 => state.ilpr,
            0x024 => state.ibrd,
            0x028 => state.fbrd,
            0x02c => state.lcr_h,
            0x030 => state.cr,
            0x034 => state.ifls,
            0x038 => state.imsc,
            0x03c => Self::raw(&state) & INT_MASK,
            0x040 => Self::raw(&state) & state.imsc & INT_MASK,
            0x048 => state.dmacr,
            // `UARTICR` is write-only, and every other offset in the window is
            // unimplemented. Both read as zero rather than faulting: a driver
            // that probes the window should find nothing there, not an abort.
            _ => 0,
        }
    }

    /// Write one register.
    fn write_register(&self, offset: u64, value: u32) {
        {
            let mut state = self.state.lock();
            match offset {
                0x000 => {
                    if state.tx.len() < state.depth() {
                        state.tx.push_back(value as u8);
                    }
                    // Offered to the host at once rather than at the next
                    // scheduler tick, so a guest that writes a character and
                    // spins on TXFE makes progress inside one quantum.
                }
                // Any write to `UARTECR` clears the error flags.
                0x004 => state.rsr = 0,
                0x020 => state.ilpr = value & 0xff,
                0x024 => state.ibrd = value & 0xffff,
                0x028 => state.fbrd = value & 0x3f,
                0x02c => {
                    let was = state.lcr_h;
                    state.lcr_h = value & 0xff;
                    // Turning the FIFOs off empties them (TRM §3.3.7): a
                    // driver that switches mode expects to start clean.
                    if (was ^ state.lcr_h) & LCRH_FEN != 0 {
                        state.rx.clear();
                        state.tx.clear();
                    }
                }
                0x030 => state.cr = value & 0xffff,
                0x034 => state.ifls = value & 0x3f,
                0x038 => state.imsc = value & INT_MASK,
                // Write ones to clear. Only the sticky bits can be cleared this
                // way; RXIS, TXIS and RTIS follow the FIFOs and come back
                // immediately, which is the architecture and not a bug.
                0x044 => state.ris &= !(value & INT_MASK),
                0x048 => state.dmacr = value & 7,
                _ => {}
            }
        }
        if offset == 0x000 {
            // Outward, with the state lock released (`CLAUDE.md`, the
            // re-entrancy contract): offering the byte reaches the host.
            self.pump();
        } else {
            self.refresh();
        }
    }
}

impl MemOps for Registers {
    fn read(&self, offset: u64, dst: &mut [u8], attrs: MemAttrs) -> MemResult {
        // Registers are 32 bits wide; a driver that reads one with a halfword
        // access gets the half it asked for, which is what the AMBA bus does.
        let value = self.read_register(offset & !3, attrs.debug);
        let shift = (offset & 3) * 8;
        let bytes = (value >> shift).to_le_bytes();
        for (i, byte) in dst.iter_mut().enumerate() {
            *byte = bytes.get(i).copied().unwrap_or(0);
        }
        if !attrs.debug {
            // Reading `UARTDR` is what clears `RXIS` and `RTIS`: on a PL011
            // those two follow the FIFO level rather than being latched, so
            // draining the FIFO *is* the acknowledgement and the line has to
            // move with it. A model that only refreshed on a write leaves a
            // handler that has read every byte still being interrupted.
            self.refresh();
        }
        Ok(())
    }

    fn write(&self, offset: u64, src: &[u8], attrs: MemAttrs) -> MemResult {
        if attrs.debug {
            // A debug write to `UARTDR` would put a character on the console
            // and to `UARTIMSC` would change when the guest is next
            // interrupted. Neither can be made harmless.
            return Err(BusError::BadAccess);
        }
        let mut value = 0u32;
        for (i, byte) in src.iter().enumerate().take(4) {
            value |= u32::from(*byte) << (i * 8);
        }
        self.write_register(offset & !3, value << ((offset & 3) * 8));
        Ok(())
    }

    fn constraints(&self) -> AccessConstraints {
        // Byte, halfword and word accesses, naturally aligned. A driver
        // reaches `UARTDR` with a halfword access on some kernels and a word
        // access on others, and both are legal on an AMBA slave.
        AccessConstraints::IO
            .with_widths(Width::U8, Width::U32)
            .with_natural_alignment(true)
    }
}

impl DtSource for Registers {
    fn dt_spec(&self) -> NodeSpec {
        let mut spec = NodeSpec {
            kind: NodeKind::Pl011,
            name: "pl011",
            compatible: &["arm,pl011", "arm,primecell"],
            cells: alloc::vec::Vec::new(),
            strings: alloc::vec::Vec::new(),
            irq_wire: None,
        };
        spec.irq_wire = *self.irq_wire.lock();
        spec
    }
}

/// The `uart.pl011` device class.
pub static CLASS: DeviceClass = DeviceClass {
    name: CLASS_NAME,
    version: STATE_VERSION,
    summary: "ARM PrimeCell PL011 UART with FIFOs, on a character port",
    properties: &[
        PropertySpec {
            name: "port",
            kind: ValueKind::Str,
            required: false,
            summary: "the character port to attach to, by name (default \"console\")",
        },
        PropertySpec {
            name: "clock-hz",
            kind: ValueKind::Uint,
            required: false,
            summary: "what the device tree reports as UARTCLK, in Hz (default 24000000)",
        },
    ],
    construct: |props| Ok(Box::new(Pl011::new(props)?)),
};

impl Device for Pl011 {
    fn class(&self) -> &'static DeviceClass {
        &CLASS
    }

    fn realize(&self, ctx: &mut RealizeCtx<'_>) -> Result<()> {
        super::dt::publish(
            ctx.hosts(),
            &self.region,
            Arc::downgrade(&self.regs) as alloc::sync::Weak<dyn DtSource>,
        )
    }

    fn reset(&self, _kind: ResetKind) {
        *self.regs.state.lock() = State::new();
        self.regs.drive(false);
    }

    fn region(&self, name: &str) -> Option<RegionRef> {
        matches!(name, "" | "regs").then(|| Arc::clone(&self.region))
    }

    fn connect(&self, port: &str, source: WireSource) -> Result<()> {
        if port != "irq" {
            return Err(Error::Config {
                at: port.to_string(),
                message: String::from("a PL011 drives one pin, `irq` — the combined UARTINTR"),
            });
        }
        *self.regs.irq_wire.lock() = Some(source.id());
        *self.regs.out.lock() = Some(source);
        Ok(())
    }

    fn announce(&self, port: &str) {
        if port == "irq" {
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
        for word in [
            state.rsr,
            state.ris,
            state.imsc,
            state.cr,
            state.lcr_h,
            state.ifls,
            state.ibrd,
            state.fbrd,
            state.ilpr,
            state.dmacr,
        ] {
            w.write_u32(word)?;
        }
        Ok(())
        // The port's queues are the host's state, not the machine's, and are
        // deliberately absent (`ROADMAP.md` §4.5).
    }

    fn load(&self, r: &mut ChunkReader<'_>) -> Result<()> {
        let mut state = State::new();
        for which in 0..2 {
            let count = r.read_seq_len(1)? as usize;
            if count > FIFO_DEPTH {
                return Err(Error::State(alloc::format!(
                    "snapshot has {count} byte(s) in a {FIFO_DEPTH}-byte PL011 FIFO"
                )));
            }
            for _ in 0..count {
                let byte = r.read_u8()?;
                if which == 0 {
                    state.rx.push_back(byte);
                } else {
                    state.tx.push_back(byte);
                }
            }
        }
        state.rsr = r.read_u32()?;
        state.ris = r.read_u32()?;
        state.imsc = r.read_u32()?;
        state.cr = r.read_u32()?;
        state.lcr_h = r.read_u32()?;
        state.ifls = r.read_u32()?;
        state.ibrd = r.read_u32()?;
        state.fbrd = r.read_u32()?;
        state.ilpr = r.read_u32()?;
        state.dmacr = r.read_u32()?;
        *self.regs.state.lock() = state;
        self.regs.refresh();
        Ok(())
    }
}

impl Instance for Pl011 {}

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
    bindings.bind(CLASS_NAME, |props| Ok(Arc::new(Pl011::new(props)?)))
}

/// What the validator should know about `uart.pl011`.
#[must_use]
pub fn schema() -> crate::machine::validate::ClassSchema {
    use crate::machine::validate::{ClassSchema, PortDir, PropSchema};
    ClassSchema::new(CLASS_NAME)
        .prop(PropSchema::new("port", ValueKind::Str))
        .prop(PropSchema::new("clock-hz", ValueKind::Uint).range(1, u64::from(u32::MAX)))
        .region("")
        .region("regs")
        .port("irq", PortDir::Out)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::state::{MachineShape, Migrations, StateReader, StateWriter};
    use crate::core::sync::{AtomicU32, Ordering};
    use crate::core::wire::{Wire, WireIdAllocator, WireSink};
    use crate::host::chardev::CharPort;

    fn wired() -> (Pl011, Arc<CharPort>) {
        let port = Arc::new(CharPort::new());
        let uart = Pl011::with_port(
            Arc::clone(&port) as Arc<dyn CharDevice>,
            "test".to_string(),
            DEFAULT_CLOCK_HZ,
        );
        (uart, port)
    }

    fn peek(u: &Pl011, offset: u64) -> u32 {
        let mut bytes = [0u8; 4];
        u.regs
            .read(offset, &mut bytes, MemAttrs::DEFAULT)
            .expect("a word read is legal");
        u32::from_le_bytes(bytes)
    }

    fn poke(u: &Pl011, offset: u64, value: u32) {
        u.regs
            .write(offset, &value.to_le_bytes(), MemAttrs::DEFAULT)
            .expect("a word write is legal");
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

    impl Probe {
        fn high(&self) -> bool {
            self.level.load(Ordering::Relaxed) != 0
        }
    }

    fn with_irq() -> (Pl011, Arc<CharPort>, Arc<Probe>) {
        let (uart, port) = wired();
        let ids = WireIdAllocator::new();
        let id = ids.alloc();
        let probe = Arc::new(Probe::default());
        let wire = Wire::builder()
            .source(id)
            .sink(Arc::clone(&probe) as Arc<dyn WireSink>, 0)
            .build_shared();
        uart.connect("irq", WireSource::new(wire, id))
            .expect("a PL011 drives irq");
        (uart, port, probe)
    }

    #[test]
    fn a_written_byte_reaches_the_host() {
        let (uart, port) = wired();
        poke(&uart, 0x000, u32::from(b'A'));
        assert_eq!(port.drain(), b"A".to_vec());
        // And the transmit FIFO is empty again, which UARTFR must say.
        assert_ne!(peek(&uart, 0x018) & FR_TXFE, 0);
        assert_eq!(peek(&uart, 0x018) & FR_BUSY, 0);
    }

    #[test]
    fn a_host_byte_arrives_in_the_receive_fifo() {
        let (uart, port) = wired();
        poke(&uart, 0x02c, LCRH_FEN);
        port.feed(b"hi");
        assert_ne!(peek(&uart, 0x018) & FR_RXFE, 0, "nothing has been pumped");
        uart.pump();
        assert_eq!(peek(&uart, 0x018) & FR_RXFE, 0, "RXFE with two bytes in");
        assert_eq!(peek(&uart, 0x000) & 0xff, u32::from(b'h'));
        assert_eq!(peek(&uart, 0x000) & 0xff, u32::from(b'i'));
        assert_ne!(peek(&uart, 0x018) & FR_RXFE, 0, "the FIFO should be empty");
    }

    #[test]
    fn a_debug_read_of_the_data_register_does_not_pop_the_fifo() {
        // `ROADMAP.md` §15, invariant 5: a debugger's read must not be a
        // guest's read.
        let (uart, port) = wired();
        port.feed(b"x");
        uart.pump();
        let mut bytes = [0u8; 4];
        uart.regs
            .read(0x000, &mut bytes, MemAttrs::DEBUG)
            .expect("a debug read is legal");
        assert_eq!(bytes[0], b'x');
        assert_eq!(peek(&uart, 0x000) & 0xff, u32::from(b'x'), "still there");
    }

    #[test]
    fn a_debug_write_is_refused_rather_than_performed() {
        let (uart, port) = wired();
        assert!(
            uart.regs
                .write(0x000, &1u32.to_le_bytes(), MemAttrs::DEBUG)
                .is_err()
        );
        assert!(port.drain().is_empty(), "nothing may reach the console");
    }

    #[test]
    fn the_receive_interrupt_follows_the_fifo_rather_than_being_latched() {
        let (uart, port, probe) = with_irq();
        poke(&uart, 0x02c, LCRH_FEN);
        poke(&uart, 0x038, INT_RX | INT_RT);
        assert!(!probe.high());
        port.feed(b"a");
        uart.pump();
        // One byte is below the half-full trigger, so this is the *timeout*
        // interrupt and nothing else — the bit a naive model leaves out.
        assert!(probe.high(), "RTIS must assert with data in the FIFO");
        assert_eq!(peek(&uart, 0x03c) & INT_RX, 0, "not yet at the trigger");
        assert_ne!(peek(&uart, 0x03c) & INT_RT, 0);
        // Writing UARTICR does not clear it: the FIFO still has the byte.
        poke(&uart, 0x044, INT_MASK);
        assert!(probe.high(), "the byte is still in the FIFO");
        assert_eq!(peek(&uart, 0x000) & 0xff, u32::from(b'a'));
        assert!(!probe.high(), "draining the FIFO is what clears it");
    }

    #[test]
    fn the_masked_status_register_is_the_raw_one_and_the_mask() {
        let (uart, port) = wired();
        poke(&uart, 0x02c, LCRH_FEN);
        port.feed(b"a");
        uart.pump();
        assert_ne!(peek(&uart, 0x03c) & INT_RT, 0, "raw");
        assert_eq!(peek(&uart, 0x040) & INT_RT, 0, "masked off");
        poke(&uart, 0x038, INT_RT);
        assert_ne!(peek(&uart, 0x040) & INT_RT, 0, "masked in");
    }

    #[test]
    fn an_overrun_is_reported_and_cleared_through_the_error_registers() {
        // Loopback, because that is the only way this model can overrun at
        // all: bytes from the *host* stay in the character port until the
        // receive FIFO has room, which is back pressure rather than data loss
        // and is the same choice the transmit side makes. A byte the chip
        // generated itself has nowhere to wait.
        let (uart, _) = wired();
        poke(&uart, 0x02c, LCRH_FEN);
        poke(&uart, 0x030, CR_UARTEN | CR_TXE | CR_RXE | CR_LBE);
        for _ in 0..FIFO_DEPTH + 4 {
            poke(&uart, 0x000, u32::from(b'x'));
        }
        assert_ne!(peek(&uart, 0x03c) & INT_OE, 0, "the FIFO overflowed");
        assert_ne!(peek(&uart, 0x004) & 0x8, 0, "UARTRSR reports OE");
        poke(&uart, 0x004, 0);
        assert_eq!(peek(&uart, 0x004), 0, "any write to UARTECR clears it");
        poke(&uart, 0x044, INT_OE);
        assert_eq!(peek(&uart, 0x03c) & INT_OE, 0, "UARTICR clears the raw bit");
    }

    #[test]
    fn loopback_puts_a_transmitted_byte_in_this_chips_own_receiver() {
        let (uart, port) = wired();
        poke(&uart, 0x02c, LCRH_FEN);
        poke(&uart, 0x030, CR_UARTEN | CR_TXE | CR_RXE | CR_LBE);
        poke(&uart, 0x000, u32::from(b'L'));
        assert!(port.drain().is_empty(), "loopback must not reach the host");
        assert_eq!(peek(&uart, 0x000) & 0xff, u32::from(b'L'));
    }

    #[test]
    fn the_identification_registers_are_what_an_amba_bus_looks_for() {
        // A driver never binds without these, and the failure looks like a
        // console that is in the device tree and silent.
        let (uart, _) = wired();
        let mut periph = 0u32;
        for (i, at) in (0xfe0..0xff0).step_by(4).enumerate() {
            periph |= (peek(&uart, at) & 0xff) << (i * 8);
        }
        assert_eq!(periph, 0x0014_1011, "PL011, designer Arm, revision 2");
        let mut pcell = 0u32;
        for (i, at) in (0xff0..0x1000).step_by(4).enumerate() {
            pcell |= (peek(&uart, at) & 0xff) << (i * 8);
        }
        assert_eq!(pcell, 0xb105_f00d, "the PrimeCell constant");
    }

    /// A backend that refuses every byte until it is told not to — which is
    /// what a full pipe or an undrained terminal looks like to a device.
    #[derive(Debug, Default)]
    struct Blocked {
        open: AtomicU32,
        taken: Mutex<alloc::vec::Vec<u8>>,
    }

    impl CharDevice for Blocked {
        fn read(&self, _dst: &mut [u8]) -> usize {
            0
        }

        fn write(&self, src: &[u8]) -> usize {
            if self.open.load(Ordering::Relaxed) == 0 || src.is_empty() {
                return 0;
            }
            self.taken.lock().push(src[0]);
            1
        }

        fn writable(&self) -> bool {
            self.open.load(Ordering::Relaxed) != 0
        }
    }

    #[test]
    fn back_pressure_leaves_the_bytes_in_the_transmit_fifo() {
        // The point of modelling it at all: a guest that writes faster than
        // the host drains must *wait*, not have its output dropped.
        let host = Arc::new(Blocked::default());
        let uart = Pl011::with_port(
            Arc::clone(&host) as Arc<dyn CharDevice>,
            "test".to_string(),
            DEFAULT_CLOCK_HZ,
        );
        poke(&uart, 0x02c, LCRH_FEN);
        for byte in b"0123456789abcdefXY" {
            poke(&uart, 0x000, u32::from(*byte));
        }
        assert_ne!(peek(&uart, 0x018) & FR_TXFF, 0, "the FIFO must fill up");
        assert!(host.taken.lock().is_empty(), "the host took nothing");
        host.open.store(1, Ordering::Relaxed);
        uart.pump();
        assert_eq!(
            &*host.taken.lock(),
            b"0123456789abcdef",
            "sixteen fitted; the last two were refused at the FIFO, not lost by the host"
        );
        assert_ne!(peek(&uart, 0x018) & FR_TXFE, 0);
    }

    /// Everything `save` writes, as bytes — the state hash, in the only form
    /// this crate has one.
    fn snapshot(u: &Pl011) -> alloc::vec::Vec<u8> {
        let mut shape = MachineShape::new();
        shape.add_device("uart", CLASS.name).unwrap();
        let mut w = StateWriter::new(shape);
        {
            let mut chunk = w.chunk("uart", CLASS.name, CLASS.version).unwrap();
            u.save(&mut chunk).unwrap();
        }
        w.to_vec().unwrap()
    }

    #[test]
    fn state_round_trips_to_an_identical_hash() {
        let (uart, port) = wired();
        poke(&uart, 0x02c, LCRH_FEN);
        poke(&uart, 0x038, INT_RX);
        poke(&uart, 0x024, 0x1234);
        port.feed(b"abc");
        uart.pump();
        let bytes = snapshot(&uart);

        let (restored, _other) = wired();
        let reader = StateReader::new(&bytes).unwrap();
        let chunk = reader
            .load("uart", CLASS.name, CLASS.version, &Migrations::new())
            .unwrap();
        restored.load(&mut chunk.reader()).unwrap();

        assert_eq!(snapshot(&uart), snapshot(&restored), "the state hash");
        assert_eq!(peek(&restored, 0x024), 0x1234, "and the divisor came back");
        assert_eq!(
            peek(&restored, 0x000) & 0xff,
            u32::from(b'a'),
            "the FIFO too"
        );
    }
}
