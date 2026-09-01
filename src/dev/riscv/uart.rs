//! A National Semiconductor 16550 UART, on the character-device seam.
//!
//! # Sources
//!
//! * *PC16550D Universal Asynchronous Receiver/Transmitter with FIFOs* data
//!   sheet (National Semiconductor). The register table, the DLAB latch, the
//!   line- and modem-status bits, the FIFO control register and the interrupt
//!   identification priorities all come from it.
//! * `docs/devices/interrupts-timers.md` and `docs/platforms/riscv-virt.md` for
//!   where it sits on this board.
//!
//! Nothing about this chip is RISC-V specific: it lives here because the
//! `virt` board is the first machine that needs one, and its class is named
//! `uart.ns16550` rather than `riscv.*` so that it can move to `dev/uart/` the
//! day a second board wants it, without a machine file changing.
//!
//! # The register file
//!
//! ```text
//!   0  RBR read / THR write   the byte itself;  DLL when LCR bit 7 is set
//!   1  IER                    interrupt enables; DLM when LCR bit 7 is set
//!   2  IIR read / FCR write   which interrupt, and FIFO control
//!   3  LCR                    word length, stop bits, parity, and DLAB
//!   4  MCR                    modem control, including loopback
//!   5  LSR                    data ready, transmitter empty, errors
//!   6  MSR                    modem status
//!   7  SCR                    a scratch byte, which is how software finds the chip
//! ```
//!
//! Only three address lines are decoded, so the eight registers repeat through
//! whatever window a machine file gives them — which is what lets the
//! conventional 256-byte aperture work without eight mirrors.
//!
//! # Transmission, and why back pressure is modelled
//!
//! A write to `THR` is offered to the [`CharDevice`] immediately. If the host
//! will not take it — a full pipe, a terminal that is not being drained — the
//! byte stays in the holding register, `THRE` reads clear, and the guest waits
//! exactly as it would on real hardware. Dropping the byte instead would make
//! the emulated machine faster than any real one and lose output under load.

use alloc::boxed::Box;
use alloc::collections::VecDeque;
use alloc::string::{String, ToString};
use alloc::sync::{Arc, Weak};
use core::fmt;

use crate::core::device::{Device, DeviceClass, PropertySpec, RealizeCtx, ResetKind};
use crate::core::error::{BusError, Error, Result};
use crate::core::props::{Props, ValueKind};
use crate::core::sched::{Budget, Consumed};
use crate::core::space::{AccessConstraints, MemAttrs, MemOps, MemResult, Region, RegionRef};
use crate::core::state::{ChunkReader, ChunkWriter, Sink, Source};
use crate::core::sync::{LockRank, Mutex};
use crate::core::value::{Endian, Width};
use crate::core::wire::{Level, WireSource};
use crate::host::chardev::{CharDevice, ports};
use crate::machine::realize::Instance;

use super::dt::{DtSource, NodeSpec};

/// The class name a machine description writes.
pub const CLASS_NAME: &str = "uart.ns16550";

/// The snapshot chunk version. Bump with the encoding, never on its own.
const STATE_VERSION: u32 = 1;

/// How much address space the register block answers.
///
/// The chip decodes three address lines; the other five are ignored, so the
/// eight registers repeat sixteen times through this window. 256 bytes is what
/// RISC-V boards conventionally give it.
pub const REGISTER_WINDOW_LEN: u64 = 0x100;

/// How deep the receive FIFO is on a 16550 (data sheet: 16 bytes each way).
pub const FIFO_DEPTH: usize = 16;

/// The character port a machine file gets if it names none.
const DEFAULT_PORT: &str = "console";

/// The reference clock the device tree reports, in hertz.
///
/// 1.8432 MHz is the classic UART crystal; boards that quote 3.6864 MHz are
/// running it at twice that. Only the device tree ever looks at this — nothing
/// here divides by it, because the character rate is the device's clock domain
/// and the machine file sets that.
const DEFAULT_FREQUENCY_HZ: u32 = 3_686_400;

// -- LSR (offset 5) ---------------------------------------------------------

/// Data ready: the receiver holding register or FIFO is not empty.
const LSR_DR: u8 = 0x01;
/// Overrun: a byte arrived with the FIFO full and was lost.
const LSR_OE: u8 = 0x02;
/// Transmitter holding register empty.
const LSR_THRE: u8 = 0x20;
/// Transmitter empty: holding register *and* shift register.
const LSR_TEMT: u8 = 0x40;

// -- IER (offset 1) ---------------------------------------------------------

/// Enable the received-data-available interrupt.
const IER_ERBFI: u8 = 0x01;
/// Enable the transmitter-holding-register-empty interrupt.
const IER_ETBEI: u8 = 0x02;
/// Enable the receiver-line-status interrupt.
const IER_ELSI: u8 = 0x04;
/// The four bits an IER write may set; the top nibble is reserved.
const IER_MASK: u8 = 0x0f;

// -- IIR (offset 2) ---------------------------------------------------------

/// No interrupt pending. Bit 0 is *set* when there is nothing to report,
/// which catches every reader out once.
const IIR_NONE: u8 = 0x01;
/// Receiver line status — the highest priority.
const IIR_RLS: u8 = 0x06;
/// Received data available.
const IIR_RDA: u8 = 0x04;
/// Transmitter holding register empty.
const IIR_THRE: u8 = 0x02;
/// Both top bits set while the FIFOs are enabled, which is how software tells
/// a 16550A from a 16450.
const IIR_FIFO_ENABLED: u8 = 0xc0;

// -- LCR (offset 3) and MCR (offset 4) --------------------------------------

/// Divisor latch access bit: makes offsets 0 and 1 the baud rate divisor.
const LCR_DLAB: u8 = 0x80;
/// Loopback: the transmitter is wired to the receiver and the modem control
/// outputs to the modem status inputs.
const MCR_LOOP: u8 = 0x10;
/// `OUT2`, which on a PC gates the interrupt onto the bus. Software sets it
/// and would be baffled to read it back clear.
const MCR_MASK: u8 = 0x1f;

/// Everything the guest can see or change.
#[derive(Debug, Default)]
struct State {
    /// Received bytes waiting to be read.
    rx: VecDeque<u8>,
    /// The byte the transmitter is holding because the host would not take it.
    tx_hold: Option<u8>,
    ier: u8,
    lcr: u8,
    mcr: u8,
    fcr: u8,
    scr: u8,
    /// The baud rate divisor. Nothing divides by it — the character rate is
    /// this device's clock domain — but software writes it and reads it back.
    divisor: u16,
    /// Sticky line-status bits, cleared by reading LSR.
    errors: u8,
    /// The transmitter-empty interrupt is latched: set when the holding
    /// register empties or when the enable is turned on with it already empty,
    /// cleared by reading IIR or writing THR (data sheet, table 3).
    thre_latch: bool,
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
    frequency_hz: u32,
    /// The net the interrupt pin drives, so the device tree can look its
    /// number up in the PLIC's pin table rather than have it written twice.
    /// See [`dt`](super::dt).
    irq_wire: Mutex<Option<crate::core::wire::WireId>>,
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

/// A 16550 UART.
#[derive(Debug)]
pub struct Uart16550 {
    regs: Arc<Registers>,
    region: RegionRef,
}

impl Uart16550 {
    /// Validate `props` and build the device.
    ///
    /// # Errors
    ///
    /// [`Error::Property`] if a property is of the wrong kind, or if one this
    /// class does not know was given.
    pub fn new(props: &Props) -> Result<Uart16550> {
        let mut r = props.reader();
        let port_name = r.or("port", String::from(DEFAULT_PORT))?;
        let frequency = r.or_range(
            "frequency",
            u64::from(DEFAULT_FREQUENCY_HZ),
            1..=u64::from(u32::MAX),
        )?;
        r.finish()?;
        Ok(Uart16550::with_port(
            ports::attach(props, &port_name)?,
            port_name,
            frequency as u32,
        ))
    }

    /// Build one against a character device the caller already has.
    #[must_use]
    pub fn with_port(port: Arc<dyn CharDevice>, port_name: String, frequency_hz: u32) -> Uart16550 {
        let regs = Arc::new(Registers {
            state: Mutex::with_rank(LockRank::DEVICE, State::default()),
            out: Mutex::with_rank(LockRank::LEAF, None),
            port,
            port_name,
            frequency_hz,
            irq_wire: Mutex::with_rank(LockRank::LEAF, None),
        });
        let region: RegionRef = Arc::new(Region::io(
            "uart.ns16550",
            REGISTER_WINDOW_LEN,
            Arc::clone(&regs) as Arc<dyn MemOps>,
        ));
        Uart16550 { regs, region }
    }

    /// The name of the character port this device is attached to.
    #[must_use]
    pub fn port_name(&self) -> &str {
        &self.regs.port_name
    }

    /// The reference clock the device tree reports.
    #[must_use]
    pub fn frequency_hz(&self) -> u32 {
        self.regs.frequency_hz
    }

    /// Move bytes between the chip and the host: fill the receive FIFO, and
    /// retry a transmission the host refused.
    ///
    /// This is what [`Device::run`] does; a test that is not running a
    /// scheduler calls it directly.
    pub fn pump(&self) {
        self.regs.pump();
    }

    /// Whether the interrupt output is currently asserted.
    #[must_use]
    pub fn irq_asserted(&self) -> bool {
        Registers::interrupt(&self.regs.state.lock()) != IIR_NONE
    }
}

impl Registers {
    /// Which interrupt is being reported, in IIR's encoding.
    ///
    /// The priorities are the data sheet's: line status, then received data,
    /// then transmitter empty. Modem status is not modelled — nothing on this
    /// board has a modem.
    fn interrupt(state: &State) -> u8 {
        if state.ier & IER_ELSI != 0 && state.errors != 0 {
            return IIR_RLS;
        }
        if state.ier & IER_ERBFI != 0 && !state.rx.is_empty() {
            return IIR_RDA;
        }
        if state.ier & IER_ETBEI != 0 && state.thre_latch {
            return IIR_THRE;
        }
        IIR_NONE
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
        let asserted = Self::interrupt(&self.state.lock()) != IIR_NONE;
        self.drive(asserted);
    }

    /// Offer the holding byte to the host, reporting whether it went.
    ///
    /// In loopback the byte never reaches the host at all: it arrives in this
    /// chip's own receiver, which is what the mode is for and what a driver's
    /// self-test depends on.
    fn flush_tx(&self, state: &mut State) -> bool {
        let Some(byte) = state.tx_hold else {
            return false;
        };
        if state.mcr & MCR_LOOP != 0 {
            state.tx_hold = None;
            Self::receive(state, byte);
            state.thre_latch = true;
            return true;
        }
        if !self.port.write_byte(byte) {
            return false;
        }
        state.tx_hold = None;
        state.thre_latch = true;
        true
    }

    /// Push one received byte into the FIFO, or record an overrun.
    fn receive(state: &mut State, byte: u8) {
        if state.rx.len() >= FIFO_DEPTH {
            state.errors |= LSR_OE;
            return;
        }
        state.rx.push_back(byte);
    }

    /// Fill the receive FIFO from the host and retry a refused transmission.
    fn pump(&self) {
        {
            let mut state = self.state.lock();
            self.flush_tx(&mut state);
            if state.mcr & MCR_LOOP == 0 {
                while state.rx.len() < FIFO_DEPTH {
                    let Some(byte) = self.port.read_byte() else {
                        break;
                    };
                    state.rx.push_back(byte);
                }
            }
        }
        self.refresh();
    }

    /// The line status register, as a read would produce it.
    fn lsr(state: &State) -> u8 {
        let mut lsr = state.errors;
        if !state.rx.is_empty() {
            lsr |= LSR_DR;
        }
        if state.tx_hold.is_none() {
            // Nothing is held and nothing is shifting: both bits, because this
            // model's transmitter is either empty or blocked.
            lsr |= LSR_THRE | LSR_TEMT;
        }
        lsr
    }

    /// Read one register. `debug` suppresses every side effect.
    fn read_register(&self, index: u8, debug: bool) -> u8 {
        let mut state = self.state.lock();
        let dlab = state.lcr & LCR_DLAB != 0;
        match index {
            0 if dlab => state.divisor as u8,
            0 => {
                if debug {
                    return state.rx.front().copied().unwrap_or(0);
                }
                state.rx.pop_front().unwrap_or(0)
            }
            1 if dlab => (state.divisor >> 8) as u8,
            1 => state.ier,
            2 => {
                let iir = Self::interrupt(&state);
                if !debug && iir == IIR_THRE {
                    // Reading IIR clears a transmitter-empty interrupt, which
                    // is the one case where the read is the acknowledgement.
                    state.thre_latch = false;
                }
                let fifo = if state.fcr & 1 != 0 {
                    IIR_FIFO_ENABLED
                } else {
                    0
                };
                iir | fifo
            }
            3 => state.lcr,
            4 => state.mcr,
            5 => {
                let lsr = Self::lsr(&state);
                if !debug {
                    // The sticky error bits are cleared by the read.
                    state.errors = 0;
                }
                lsr
            }
            // No modem is attached. In loopback the modem control outputs
            // appear as the status inputs, which is what a driver's loopback
            // test checks: RTS→CTS, DTR→DSR, OUT1→RI, OUT2→DCD.
            6 => {
                if state.mcr & MCR_LOOP != 0 {
                    let mcr = state.mcr;
                    (mcr & 0x01) << 4 | (mcr & 0x02) << 4 | (mcr & 0x04) << 4 | (mcr & 0x08) << 4
                } else {
                    0
                }
            }
            _ => state.scr,
        }
    }

    /// Write one register.
    fn write_register(&self, index: u8, value: u8) {
        {
            let mut state = self.state.lock();
            let dlab = state.lcr & LCR_DLAB != 0;
            match index {
                0 if dlab => state.divisor = (state.divisor & 0xff00) | u16::from(value),
                0 => {
                    if state.tx_hold.is_some() {
                        // The guest wrote THR while it was full. Real hardware
                        // overwrites the holding register and loses a byte;
                        // there is nowhere else for it to go.
                        state.tx_hold = Some(value);
                    } else {
                        state.tx_hold = Some(value);
                        self.flush_tx(&mut state);
                    }
                    // A write clears the latch whether or not the byte went:
                    // the interrupt said "give me a byte" and it has one.
                    state.thre_latch = state.tx_hold.is_none();
                }
                1 if dlab => {
                    state.divisor = (state.divisor & 0x00ff) | (u16::from(value) << 8);
                }
                1 => {
                    let was = state.ier;
                    state.ier = value & IER_MASK;
                    // Enabling the transmit interrupt with the holding register
                    // already empty raises one immediately — which is how a
                    // driver starts a transmission at all.
                    if state.ier & IER_ETBEI != 0 && was & IER_ETBEI == 0 && state.tx_hold.is_none()
                    {
                        state.thre_latch = true;
                    }
                }
                2 => {
                    state.fcr = value;
                    // Bits 1 and 2 clear the receive and transmit FIFOs.
                    if value & 0x02 != 0 {
                        state.rx.clear();
                    }
                    if value & 0x04 != 0 {
                        state.tx_hold = None;
                        state.thre_latch = true;
                    }
                }
                3 => state.lcr = value,
                4 => {
                    state.mcr = value & MCR_MASK;
                    // Leaving loopback with a byte still held: it goes to the
                    // host now, as the pin is connected again.
                    self.flush_tx(&mut state);
                }
                // LSR and MSR are read-only status. A write is swallowed.
                5 | 6 => {}
                _ => state.scr = value,
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
        *byte = self.read_register((offset & 7) as u8, attrs.debug);
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
            // A debug write to THR would put a character on the console and to
            // IER would change when the guest is next interrupted. Neither can
            // be made harmless (`ROADMAP.md` §15, invariant 5).
            return Err(BusError::BadAccess);
        }
        self.write_register((offset & 7) as u8, *value);
        Ok(())
    }

    fn constraints(&self) -> AccessConstraints {
        // An 8-bit part on an 8-bit bus. A 32-bit read of the register file is
        // not a thing that happens, and accepting one would invent an order.
        AccessConstraints::word(Width::U8, Endian::Little)
    }
}

impl DtSource for Registers {
    fn dt_spec(&self) -> NodeSpec {
        let mut spec = NodeSpec::peripheral("serial", &["ns16550a"])
            .with_cells("clock-frequency", alloc::vec![self.frequency_hz])
            // Spelled out rather than left to the binding's defaults: this
            // board decodes three address lines and puts the registers one
            // byte apart, and a guest that assumed otherwise would find the
            // scratch register where the line status is.
            .with_cells("reg-shift", alloc::vec![0])
            .with_cells("reg-io-width", alloc::vec![1]);
        spec.irq_wire = *self.irq_wire.lock();
        spec
    }
}

/// The `uart.ns16550` device class.
pub static CLASS: DeviceClass = DeviceClass {
    name: CLASS_NAME,
    version: STATE_VERSION,
    summary: "16550 UART with FIFOs, on a character port",
    properties: &[
        PropertySpec {
            name: "port",
            kind: ValueKind::Str,
            required: false,
            summary: "the character port to attach to, by name (default \"console\")",
        },
        PropertySpec {
            name: "frequency",
            kind: ValueKind::Uint,
            required: false,
            summary: "the reference clock the device tree reports, in Hz (default 3686400)",
        },
    ],
    construct: |props| Ok(Box::new(Uart16550::new(props)?)),
};

impl Device for Uart16550 {
    fn class(&self) -> &'static DeviceClass {
        &CLASS
    }

    fn realize(&self, ctx: &mut RealizeCtx<'_>) -> Result<()> {
        // What this region is, for the board's device-tree generator.
        super::dt::publish(
            ctx.hosts(),
            &self.region,
            Arc::downgrade(&self.regs) as Weak<dyn DtSource>,
        )
    }

    fn reset(&self, _kind: ResetKind) {
        {
            let mut state = self.regs.state.lock();
            *state = State::default();
        }
        self.regs.drive(false);
    }

    fn region(&self, name: &str) -> Option<RegionRef> {
        matches!(name, "" | "regs").then(|| Arc::clone(&self.region))
    }

    fn connect(&self, port: &str, source: WireSource) -> Result<()> {
        if port != "irq" {
            return Err(Error::Config {
                at: port.to_string(),
                message: String::from("a 16550 drives one pin, `irq`"),
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
        // filled from the host and a refused transmission has to be retried,
        // and the scheduler is the only thing allowed to decide when
        // (`CLAUDE.md`: a device never reads the wall clock).
        true
    }

    fn run(&self, budget: Budget) -> Consumed {
        self.regs.pump();
        Consumed::new(budget.ticks)
    }

    fn save(&self, w: &mut ChunkWriter<'_>) -> Result<()> {
        let state = self.regs.state.lock();
        w.write_seq_len(state.rx.len() as u64)?;
        for byte in &state.rx {
            w.write_u8(*byte)?;
        }
        match state.tx_hold {
            None => w.write_bool(false)?,
            Some(byte) => {
                w.write_bool(true)?;
                w.write_u8(byte)?;
            }
        }
        for byte in [
            state.ier,
            state.lcr,
            state.mcr,
            state.fcr,
            state.scr,
            state.errors,
        ] {
            w.write_u8(byte)?;
        }
        w.write_u16(state.divisor)?;
        w.write_bool(state.thre_latch)
        // The port's queues are the host's state, not the machine's, and are
        // deliberately absent (`ROADMAP.md` §4.5).
    }

    fn load(&self, r: &mut ChunkReader<'_>) -> Result<()> {
        let count = r.read_seq_len(1)? as usize;
        if count > FIFO_DEPTH {
            return Err(Error::State(alloc::format!(
                "snapshot has {count} byte(s) in a {FIFO_DEPTH}-byte receive FIFO"
            )));
        }
        let mut state = State::default();
        for _ in 0..count {
            state.rx.push_back(r.read_u8()?);
        }
        state.tx_hold = if r.read_bool()? {
            Some(r.read_u8()?)
        } else {
            None
        };
        state.ier = r.read_u8()?;
        state.lcr = r.read_u8()?;
        state.mcr = r.read_u8()?;
        state.fcr = r.read_u8()?;
        state.scr = r.read_u8()?;
        state.errors = r.read_u8()?;
        state.divisor = r.read_u16()?;
        state.thre_latch = r.read_bool()?;
        *self.regs.state.lock() = state;
        self.regs.refresh();
        Ok(())
    }
}

impl Instance for Uart16550 {}

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
    bindings.bind(CLASS_NAME, |props| Ok(Arc::new(Uart16550::new(props)?)))
}

/// What the validator should know about `uart.ns16550`.
#[must_use]
pub fn schema() -> crate::machine::validate::ClassSchema {
    use crate::machine::validate::{ClassSchema, PortDir, PropSchema};
    ClassSchema::new(CLASS_NAME)
        .prop(PropSchema::new("port", ValueKind::Str))
        .prop(PropSchema::new("frequency", ValueKind::Uint).range(1, u64::from(u32::MAX)))
        .region("")
        .region("regs")
        .port("irq", PortDir::Out)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::state::{MachineShape, Migrations, StateReader, StateWriter};
    use crate::core::sync::{AtomicU32, Ordering};
    use crate::core::wire::{Wire, WireId, WireIdAllocator, WireSink};
    use crate::host::chardev::CharPort;

    fn wired() -> (Uart16550, Arc<CharPort>) {
        let port = Arc::new(CharPort::new());
        let uart = Uart16550::with_port(
            Arc::clone(&port) as Arc<dyn CharDevice>,
            "test".to_string(),
            DEFAULT_FREQUENCY_HZ,
        );
        (uart, port)
    }

    fn peek(u: &Uart16550, index: u64) -> u8 {
        let mut byte = [0u8; 1];
        u.regs
            .read(index, &mut byte, MemAttrs::DEFAULT)
            .expect("a byte read is legal");
        byte[0]
    }

    fn poke(u: &Uart16550, index: u64, value: u8) {
        u.regs
            .write(index, &[value], MemAttrs::DEFAULT)
            .expect("a byte write is legal");
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

    fn with_irq() -> (Uart16550, Arc<CharPort>, Arc<Probe>) {
        let (uart, port) = wired();
        let ids = WireIdAllocator::new();
        let id = ids.alloc();
        let probe = Arc::new(Probe::default());
        let wire = Wire::builder()
            .source(id)
            .sink(Arc::clone(&probe) as Arc<dyn WireSink>, 0)
            .build_shared();
        uart.connect("irq", WireSource::new(wire, id))
            .expect("a 16550 drives irq");
        (uart, port, probe)
    }

    #[test]
    fn a_byte_written_to_thr_reaches_the_host() {
        let (uart, port) = wired();
        poke(&uart, 0, b'A');
        assert_eq!(port.drain(), b"A".to_vec());
        assert_eq!(peek(&uart, 5) & (LSR_THRE | LSR_TEMT), LSR_THRE | LSR_TEMT);
    }

    #[test]
    fn a_byte_fed_by_the_host_is_read_from_rbr_once() {
        let (uart, port) = wired();
        port.feed(b"hi");
        uart.pump();
        assert_eq!(peek(&uart, 5) & LSR_DR, LSR_DR);
        assert_eq!(peek(&uart, 0), b'h');
        assert_eq!(peek(&uart, 0), b'i');
        assert_eq!(peek(&uart, 5) & LSR_DR, 0);
        assert_eq!(peek(&uart, 0), 0, "an empty FIFO reads as zero");
    }

    #[test]
    fn the_registers_repeat_because_only_three_address_lines_are_decoded() {
        let (uart, _port) = wired();
        poke(&uart, 7, 0x5a);
        assert_eq!(peek(&uart, 7), 0x5a);
        assert_eq!(peek(&uart, 0x0f), 0x5a, "offset 15 is offset 7");
        assert_eq!(peek(&uart, 0xff), 0x5a);
    }

    #[test]
    fn the_scratch_register_is_how_software_finds_the_chip() {
        let (uart, _port) = wired();
        for probe in [0x00u8, 0xa5, 0xff] {
            poke(&uart, 7, probe);
            assert_eq!(peek(&uart, 7), probe);
        }
    }

    #[test]
    fn dlab_swaps_the_first_two_registers_for_the_divisor() {
        let (uart, port) = wired();
        poke(&uart, 3, LCR_DLAB);
        poke(&uart, 0, 0x0c);
        poke(&uart, 1, 0x01);
        assert_eq!(peek(&uart, 0), 0x0c);
        assert_eq!(peek(&uart, 1), 0x01);
        assert!(port.drain().is_empty(), "and nothing was transmitted");

        // Clearing DLAB gives the data and enable registers back.
        poke(&uart, 3, 0x03);
        poke(&uart, 0, b'X');
        assert_eq!(port.drain(), b"X".to_vec());
    }

    #[test]
    fn received_data_raises_the_interrupt_and_reading_it_clears_it() {
        let (uart, port, probe) = with_irq();
        port.feed(b"z");
        uart.pump();
        assert_eq!(probe.level.load(Ordering::Relaxed), 0, "not enabled yet");

        poke(&uart, 1, IER_ERBFI);
        assert_eq!(probe.level.load(Ordering::Relaxed), 1);
        assert_eq!(peek(&uart, 2) & 0x0f, IIR_RDA);
        assert_eq!(peek(&uart, 0), b'z');
        assert_eq!(probe.level.load(Ordering::Relaxed), 0);
        assert_eq!(peek(&uart, 2) & 0x0f, IIR_NONE);
    }

    #[test]
    fn enabling_the_transmit_interrupt_with_an_empty_holding_register_raises_one() {
        // The case a naive edge model gets wrong, and the reason a driver can
        // start a transmission at all.
        let (uart, _port, probe) = with_irq();
        poke(&uart, 1, IER_ETBEI);
        assert_eq!(probe.level.load(Ordering::Relaxed), 1);
        assert_eq!(peek(&uart, 2) & 0x0f, IIR_THRE);
        // And reading IIR is the acknowledgement.
        assert_eq!(probe.level.load(Ordering::Relaxed), 0);
        // Writing a byte arms it again, because the byte leaves immediately.
        poke(&uart, 0, b'q');
        assert_eq!(probe.level.load(Ordering::Relaxed), 1);
    }

    #[test]
    fn line_status_beats_received_data_beats_transmitter_empty() {
        let (uart, port, _probe) = with_irq();
        poke(&uart, 1, IER_ERBFI | IER_ETBEI | IER_ELSI);
        port.feed(b"x");
        uart.pump();
        assert_eq!(peek(&uart, 2) & 0x0f, IIR_RDA, "data beats THRE");

        // Overrun the FIFO to raise a line status condition. Through
        // loopback, because bytes the host is still holding are not bytes the
        // receiver has lost: the port pushes back rather than overrunning.
        poke(&uart, 4, MCR_LOOP);
        for _ in 0..=FIFO_DEPTH {
            poke(&uart, 0, b'y');
        }
        assert_eq!(peek(&uart, 2) & 0x0f, IIR_RLS, "and line status beats both");
        assert_eq!(peek(&uart, 5) & LSR_OE, LSR_OE);
        assert_eq!(peek(&uart, 5) & LSR_OE, 0, "reading LSR clears it");
    }

    #[test]
    fn a_host_that_will_not_take_a_byte_stalls_the_transmitter() {
        // Back pressure arrives as the hardware would deliver it, rather than
        // as a dropped character.
        let (uart, port) = wired();
        // Fill the port so nothing more fits.
        while port.writable() {
            port.write(b".");
        }
        poke(&uart, 0, b'A');
        assert_eq!(peek(&uart, 5) & LSR_THRE, 0, "the holding register is full");
        let _ = port.drain();
        uart.pump();
        assert_eq!(peek(&uart, 5) & LSR_THRE, LSR_THRE);
        assert_eq!(port.drain(), b"A".to_vec(), "and the byte was not lost");
    }

    #[test]
    fn loopback_ties_the_transmitter_to_the_receiver() {
        let (uart, port) = wired();
        poke(&uart, 4, MCR_LOOP);
        poke(&uart, 0, b'L');
        assert!(port.drain().is_empty(), "nothing reaches the host");
        assert_eq!(peek(&uart, 5) & LSR_DR, LSR_DR);
        assert_eq!(peek(&uart, 0), b'L');
        // The modem status inputs mirror the control outputs.
        poke(&uart, 4, MCR_LOOP | 0x01);
        assert_eq!(peek(&uart, 6) & 0x10, 0x10, "RTS appears as CTS");
    }

    #[test]
    fn the_fifo_control_register_reports_a_16550a() {
        let (uart, _port) = wired();
        assert_eq!(peek(&uart, 2) & IIR_FIFO_ENABLED, 0, "FIFOs start off");
        poke(&uart, 2, 0x01);
        assert_eq!(peek(&uart, 2) & IIR_FIFO_ENABLED, IIR_FIFO_ENABLED);
    }

    #[test]
    fn clearing_the_receive_fifo_discards_what_was_in_it() {
        let (uart, port) = wired();
        port.feed(b"abc");
        uart.pump();
        poke(&uart, 2, 0x01 | 0x02);
        assert_eq!(peek(&uart, 5) & LSR_DR, 0);
    }

    #[test]
    fn a_debug_read_pops_nothing_and_a_debug_write_is_refused() {
        let (uart, port) = wired();
        port.feed(b"k");
        uart.pump();
        let mut byte = [0u8; 1];
        uart.regs.read(0, &mut byte, MemAttrs::DEBUG).unwrap();
        assert_eq!(byte[0], b'k');
        assert_eq!(peek(&uart, 5) & LSR_DR, LSR_DR, "still there");
        assert_eq!(peek(&uart, 0), b'k');

        assert!(uart.regs.write(0, b"x", MemAttrs::DEBUG).is_err());
        assert!(port.drain().is_empty());
    }

    #[test]
    fn an_access_that_is_not_a_single_byte_is_refused() {
        let (uart, _port) = wired();
        assert!(uart.regs.read(0, &mut [0u8; 2], MemAttrs::DEFAULT).is_err());
        assert!(uart.regs.write(0, &[0u8; 4], MemAttrs::DEFAULT).is_err());
    }

    #[test]
    fn a_snapshot_round_trips_the_register_file_and_the_fifo() {
        let (saved, port) = wired();
        port.feed(b"abc");
        saved.pump();
        poke(&saved, 3, LCR_DLAB);
        poke(&saved, 0, 0x0c);
        poke(&saved, 3, 0x1b);
        poke(&saved, 1, IER_ERBFI);
        poke(&saved, 7, 0x77);

        let mut shape = MachineShape::new();
        shape.add_device("uart", CLASS.name).unwrap();
        let mut w = StateWriter::new(shape);
        {
            let mut chunk = w.chunk("uart", CLASS.name, CLASS.version).unwrap();
            saved.save(&mut chunk).unwrap();
        }
        let bytes = w.to_vec().unwrap();

        let (restored, _other) = wired();
        let reader = StateReader::new(&bytes).unwrap();
        let chunk = reader
            .load("uart", CLASS.name, CLASS.version, &Migrations::new())
            .unwrap();
        restored.load(&mut chunk.reader()).unwrap();

        assert_eq!(peek(&restored, 3), 0x1b);
        assert_eq!(peek(&restored, 7), 0x77);
        assert_eq!(peek(&restored, 1), IER_ERBFI);
        assert_eq!(peek(&restored, 0), b'a', "and the FIFO came back");
        assert!(restored.irq_asserted());
    }

    #[test]
    fn properties_are_checked_rather_than_ignored() {
        let uart = Uart16550::new(&Props::new().with("frequency", 1_843_200u64))
            .expect("a frequency is legal");
        assert_eq!(uart.frequency_hz(), 1_843_200);
        assert_eq!(uart.port_name(), DEFAULT_PORT);
        assert!(Uart16550::new(&Props::new().with("prot", "x")).is_err());
    }
}
