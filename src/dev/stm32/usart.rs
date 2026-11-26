//! An STM32 USART, on the character-device seam.
//!
//! # Two register maps, one chip
//!
//! ST redesigned this peripheral between the F4 and the F7, and the two
//! layouts are genuinely different rather than one being a superset:
//!
//! | | F1/F2/**F4** (RM0090 §30.6) | F0/F3/F7/L4/**H7** (RM0410 §34.8) |
//! | --- | --- | --- |
//! | status | `SR` at `+0x00` | `ISR` at `+0x1c` |
//! | data | `DR` at `+0x04`, read *and* write | `RDR` at `+0x24`, `TDR` at `+0x28` |
//! | control | `CR1`–`CR3` at `+0x0c`…`+0x14` | `CR1`–`CR3` at `+0x00`…`+0x08` |
//! | baud | `BRR` at `+0x08` | `BRR` at `+0x0c` |
//! | clearing a flag | read `SR`, then read or write the data register | write a one to `ICR` at `+0x20` |
//! | `CR1` layout | `UE` at 13, `TE` at 3, `RE` at 2 | `UE` at 0, `TE` at 3, `RE` at 2 |
//!
//! Firmware written for one does not run on the other, so this is a
//! **construction property**, `variant`, and not something to average out. The
//! flag *positions* inside the status word do agree for the ten bits that
//! matter (`PE`, `FE`, `NF`, `ORE`, `IDLE`, `RXNE`, `TC`, `TXE`, `LBD`/`LBDF`,
//! `CTS`/`CTSIF`), so one set of semantics serves both and only the decode
//! differs.
//!
//! A third, **`"u5"`**, is the F7 layout grown up: the same offsets, plus
//! `RTOR` and `RQR` decoded, a `PRESC` at `+0x2c` and an `AUTOCR` at `+0x30`
//! (ST CMSIS `stm32u585xx.h` (Apache-2.0), `USART_TypeDef`; RM0456, the
//! USART chapter). It is named for the part it was written against, but the
//! H7, G4 and L4+ have the same FIFO-and-`PRESC` USART and only lack `AUTOCR`,
//! which reads as zero to firmware that never writes it. What it adds:
//!
//! * **`TEACK`/`REACK`** (`ISR` bits 21/22), set while `UE` and `TE`/`RE` are.
//!   `HAL_UART_Init` ends in `UART_CheckIdleState`, which polls them, so
//!   without them every HAL console times out before printing a byte.
//! * **The FIFO mode**, `CR1.FIFOEN`: eight bytes each way, `TXE` reading as
//!   `TXFNF` and `RXNE` as `RXFNE`, plus `TXFE`, `RXFF`, and the `TXFT`/`RXFT`
//!   thresholds `CR3.TXFTCFG`/`RXFTCFG` select. With `FIFOEN` clear the depth
//!   is one, which is the F7's behaviour exactly. The thresholds count what
//!   `HAL_UARTEx` counts: `TXFT` means at least that many *free* transmit
//!   entries, `RXFT` at least that many received bytes. `TXFE` is derived
//!   from the queue rather than latched, so `ICR.TXFECF` does not hold it
//!   clear while the FIFO stays empty.
//! * **`RQR.RXFRQ`/`TXFRQ`**, which flush the receive and transmit sides.
//! * **`UE` going low** discards whatever is queued and returns `ISR` to its
//!   reset value — "all current operations are discarded" — which is what
//!   firmware that toggles `UE` around a reconfiguration expects to find.
//!
//! An LPUART has the same register file with a 20-bit `BRR` and no `GTPR` or
//! `RTOR`; `lpuart = true` (with `variant = "u5"`) gives it exactly that.
//!
//! The default is `"f4"`, because [`machines/stm32f407.machine`] is an F407
//! and a property whose default matches the board it ships with is one fewer
//! line in the file.
//!
//! # What the flags mean here
//!
//! * `TXE` — the transmit holding register is free. Writing the data register
//!   clears it; the byte reaches the host on the next tick of this device's
//!   clock domain, and `TXE` sets again then. That is what makes back pressure
//!   real: a host that will not take a byte stalls a guest polling `TXE`,
//!   exactly as a slow wire would.
//! * `TC` — the shift register is empty as well. It sets when the byte
//!   actually goes, and firmware that waits on it before dropping `TE` or
//!   sleeping gets the answer it is waiting for.
//! * `RXNE` — a byte is in the receive register. There is **no FIFO** on an F4
//!   USART, so a second byte arriving before the first is read sets `ORE` and
//!   is lost, which is the overrun firmware is supposed to handle and usually
//!   does not.
//!
//! # The pins
//!
//! **No data pins.** A USART's transmit pin carries a bit stream, and this
//! model's data path is [`host::chardev`](crate::host::chardev) — a byte at a
//! time, to a terminal. So `TX` and `RX` are not wires and a board does not
//! connect them to a GPIO's alternate-function inputs. Firmware still
//! configures `MODER` and `AFR` for its `TX` pin and that configuration is
//! modelled faithfully by [`gpio`](super::gpio), because firmware does it and
//! expects the registers to read back — it simply is not what carries the
//! data. A peripheral whose pin genuinely *is* a level, an SPI clock or a timer
//! compare output, is the case the GPIO's `af{n}` inputs exist for.
//!
//! What a USART does drive is three request levels:
//!
//! | Pin | High while | Dropped by |
//! | --- | --- | --- |
//! | `irq` | an enabled interrupt condition stands (RM0090 Table 149) | servicing it |
//! | `dma-rx` | `CR3.DMAR` and `RXNE` (on a `u5` with the FIFO on, `RXFNE`) | the DMA's read of `DR`/`RDR` |
//! | `dma-tx` | `CR3.DMAT` and `TXE` (`TXFNF`) | the DMA's write of `DR`/`TDR` |
//!
//! RM0090 §30.6.7 and RM0351 §40.5.15: "DMA mode can be enabled for
//! transmission by setting DMAT bit in the USART_CR3 register. Data is loaded
//! from a SRAM area … to the USART_DR register whenever the TXE bit is set";
//! reception the same with `DMAR` and `RXNE`. The DMA request is a level
//! ([`st.dma`](super::dma)'s contract), and the controller's ordinary bus
//! access to the data register is what clears the flag, exactly as a CPU read
//! would — so circular-buffer reception, the pattern every Cube UART driver
//! uses, fills its ring with no firmware access to the USART at all, because
//! the receiver is polled on the scheduler's quantum. Which DMA channel or
//! DMAMUX input a request reaches is the part's request table, and that is
//! the board file's to say.
//!
//! Two *inputs* arrive from the reset and clock controller — `enable`, which
//! is `RCC_APB1ENR.USART2EN`, and `reset`, which is `RCC_APB1RSTR.USART2RST`.
//! A board draws them or it does not; an undrawn `enable` leaves the USART
//! clocked. [`gate`] has the rules and what a gated block does.
//!
//! # Baud rate
//!
//! `BRR` is stored and reported and **does not change the byte rate**. The
//! device moves at most one byte per tick of its clock domain, and a machine
//! file sets that domain to the character rate it wants. Modelling `BRR`
//! properly would mean a device recomputing its own event period from a
//! divisor, which is a real thing to want and is not what makes a console
//! work; [`Usart::baud_divisor`] reports the programmed value so a board that
//! cares can act on it.
//!
//! # Sources
//!
//! ST **RM0090** rev 21 §30 ("Universal synchronous asynchronous receiver
//! transmitter") for the F4 layout, and ST **RM0410** rev 4 §34 for the F7's.
//! The `u5` layout is from ST **RM0456** (STM32U5), the USART and LPUART
//! chapters, with offsets and bit positions from the `USART_TypeDef` and
//! `USART_*_Pos` definitions in ST CMSIS `stm32u585xx.h` (Apache-2.0); what
//! `HAL_UART_Init` polls is from ST's BSD-3-Clause U5 HAL, read for behaviour.
//! No emulator source of any licence was consulted (`ROADMAP.md` §1).
//!
//! [`machines/stm32f407.machine`]: https://github.com/KarpelesLab/rsemu/blob/master/machines/stm32f407.machine

use alloc::boxed::Box;
use alloc::format;
use alloc::string::{String, ToString};
use alloc::sync::Arc;
use core::fmt;

use crate::core::device::{Device, DeviceClass, PropertySpec, RealizeCtx, ResetKind, SinkPin};
use crate::core::error::BusError;
use crate::core::error::{Error, Result};
use crate::core::props::{Props, ValueKind};
use crate::core::sched::{Budget, Consumed};
use crate::core::space::{AccessConstraints, MemAttrs, MemOps, MemResult, Region, RegionRef};
use crate::core::state::{ChunkReader, ChunkWriter, Sink, Source};
use crate::core::sync::{LockRank, Mutex};
use crate::core::value::{Endian, Width};
use crate::core::wire::{Level, WireId, WireSource};
use crate::dev::stm32::gate::{self, ClockGate, Gated};
use crate::host::chardev::{CharDevice, ports};
use crate::machine::Instance;
use crate::machine::validate::{ClassSchema, PortDir, PropSchema};

/// The class name a machine description writes.
const CLASS_NAME: &str = "st.usart";

/// The snapshot chunk version. Bump it with the encoding, never on its own.
const STATE_VERSION: u32 = 1;

/// The character port a machine file gets if it names none.
const DEFAULT_PORT: &str = "console";

/// The name of the interrupt output pin.
pub const IRQ_PIN: &str = "irq";
/// The receive DMA request: `CR3.DMAR` and `RXNE`.
pub const DMA_RX_PIN: &str = "dma-rx";
/// The transmit DMA request: `CR3.DMAT` and `TXE`.
pub const DMA_TX_PIN: &str = "dma-tx";

/// How many bytes the register block occupies.
///
/// `0x2c` covers the F7 layout's `TDR` at `+0x28`; the F4 stops at `GTPR`.
/// One number for both, because a board's `map` statement should not have to
/// know which variant it named.
pub const REGISTER_BYTES: u64 = 0x2c;

/// How many bytes the `u5` register block occupies: through `AUTOCR`.
pub const REGISTER_BYTES_U5: u64 = 0x34;

/// The `u5` FIFO depth, each way (RM0456: "TXFIFO and RXFIFO … 8 data").
pub const FIFO_DEPTH: usize = 8;

// -- status flags, in the positions both layouts agree on --------------------

/// Parity error.
const SR_PE: u32 = 1 << 0;
/// Framing error.
const SR_FE: u32 = 1 << 1;
/// Noise detected.
const SR_NF: u32 = 1 << 2;
/// Overrun: a byte arrived while the last one was still unread.
const SR_ORE: u32 = 1 << 3;
/// The line has been idle.
const SR_IDLE: u32 = 1 << 4;
/// The receive register holds a byte.
const SR_RXNE: u32 = 1 << 5;
/// Transmission complete: the shift register is empty too.
const SR_TC: u32 = 1 << 6;
/// The transmit holding register is free.
const SR_TXE: u32 = 1 << 7;

/// The flags that come up set: nothing to send, and the last nothing was sent.
const SR_RESET: u32 = SR_TC | SR_TXE;

/// `ICR`'s writable bits on an F7 — the flags a write of one clears. `TXE` and
/// `RXNE` are not among them: they follow the data registers.
const ICR_MASK: u32 = SR_PE | SR_FE | SR_NF | SR_ORE | SR_IDLE | SR_TC;

// -- control bits ------------------------------------------------------------

/// `CR1.RE` — receiver enable. Same bit in both layouts.
const CR1_RE: u32 = 1 << 2;
/// `CR1.TE` — transmitter enable. Same bit in both layouts.
const CR1_TE: u32 = 1 << 3;
/// `CR1.IDLEIE`.
const CR1_IDLEIE: u32 = 1 << 4;
/// `CR1.RXNEIE`.
const CR1_RXNEIE: u32 = 1 << 5;
/// `CR1.TCIE`.
const CR1_TCIE: u32 = 1 << 6;
/// `CR1.TXEIE`.
const CR1_TXEIE: u32 = 1 << 7;
/// `CR1.PEIE`.
const CR1_PEIE: u32 = 1 << 8;
/// `CR1.UE` on the F4: bit 13.
const CR1_UE_F4: u32 = 1 << 13;
/// `CR1.UE` on the F7: bit 0.
const CR1_UE_F7: u32 = 1 << 0;
/// `CR3.EIE` — the error interrupt, which is what makes `ORE` raise one.
const CR3_EIE: u32 = 1 << 0;
/// `CR3.DMAR`: receive by DMA (bit 6 in every layout: RM0090 §30.6.6,
/// RM0351 §40.8.3, `USART_CR3_DMAR_Pos` in `stm32u585xx.h`).
const CR3_DMAR: u32 = 1 << 6;
/// `CR3.DMAT`: transmit by DMA (bit 7 in every layout).
const CR3_DMAT: u32 = 1 << 7;

// -- the u5 additions (ST CMSIS `stm32u585xx.h`, `USART_*_Pos`) --------------

/// `CR1.FIFOEN`.
const CR1_FIFOEN: u32 = 1 << 29;
/// `CR1.TXFEIE`.
const CR1_TXFEIE: u32 = 1 << 30;
/// `CR1.RXFFIE`.
const CR1_RXFFIE: u32 = 1 << 31;
/// `CR3.TXFTIE`.
const CR3_TXFTIE: u32 = 1 << 23;
/// `CR3.RXFTCFG`, three bits.
const CR3_RXFTCFG_SHIFT: u32 = 25;
/// `CR3.RXFTIE`.
const CR3_RXFTIE: u32 = 1 << 28;
/// `CR3.TXFTCFG`, three bits.
const CR3_TXFTCFG_SHIFT: u32 = 29;
/// `ISR.TEACK`: the transmitter has acknowledged `TE`.
const ISR_TEACK: u32 = 1 << 21;
/// `ISR.REACK`: the receiver has acknowledged `RE`.
const ISR_REACK: u32 = 1 << 22;
/// `ISR.TXFE`: the transmit FIFO is empty.
const ISR_TXFE: u32 = 1 << 23;
/// `ISR.RXFF`: the receive FIFO is full.
const ISR_RXFF: u32 = 1 << 24;
/// `ISR.RXFT`: the receive FIFO has reached its threshold.
const ISR_RXFT: u32 = 1 << 26;
/// `ISR.TXFT`: the transmit FIFO has reached its threshold.
const ISR_TXFT: u32 = 1 << 27;
/// `RQR.RXFRQ`: flush the received data.
const RQR_RXFRQ: u32 = 1 << 3;
/// `RQR.TXFRQ`: flush the transmit data.
const RQR_TXFRQ: u32 = 1 << 4;
/// `PRESC.PRESCALER`, four bits.
const PRESC_MASK: u32 = 0xf;
/// `AUTOCR`'s defined bits: `TDN`, `TRIGPOL`, `TRIGEN`, `IDLEDIS`, `TRIGSEL`.
const AUTOCR_MASK: u32 = 0x007f_ffff;

/// How many entries each `TXFTCFG`/`RXFTCFG` encoding stands for: 1/8, 1/4,
/// 1/2, 3/4 and 7/8 of eight, then "empty"/"full". 6 and 7 are reserved and
/// never match.
const THRESHOLDS: [usize; 6] = [1, 2, 4, 6, 7, 8];

// ---------------------------------------------------------------------------
// Variant
// ---------------------------------------------------------------------------

/// Which register layout this instance has.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Variant {
    /// The F1/F2/F4 map: `SR` at `+0x00` and one `DR` at `+0x04`.
    F4,
    /// The F0/F3/F7/L4/H7 map: `CR1` at `+0x00`, `ISR`/`ICR`/`RDR`/`TDR` at
    /// the top.
    F7,
    /// The F7 map plus `PRESC`, `AUTOCR`, `TEACK`/`REACK` and the eight-deep
    /// FIFO mode: the STM32U5's USART and LPUART.
    U5,
}

impl Variant {
    /// The spelling a machine file writes.
    #[must_use]
    pub fn as_str(self) -> &'static str {
        match self {
            Variant::F4 => "f4",
            Variant::F7 => "f7",
            Variant::U5 => "u5",
        }
    }

    /// Which bit of `CR1` is `UE`.
    fn ue(self) -> u32 {
        match self {
            Variant::F4 => CR1_UE_F4,
            Variant::F7 | Variant::U5 => CR1_UE_F7,
        }
    }

    /// How many bytes of the aperture decode.
    fn register_bytes(self) -> u64 {
        match self {
            Variant::F4 | Variant::F7 => REGISTER_BYTES,
            Variant::U5 => REGISTER_BYTES_U5,
        }
    }
}

// ---------------------------------------------------------------------------
// The u5 FIFO
// ---------------------------------------------------------------------------

/// One direction's FIFO on a `u5`. A ring would be faster; eight bytes moved
/// down by one is not worth the indices.
#[derive(Debug, Default, Clone, Copy, PartialEq, Eq)]
struct Fifo {
    buf: [u8; FIFO_DEPTH],
    len: u8,
}

impl Fifo {
    fn len(&self) -> usize {
        usize::from(self.len)
    }

    fn front(&self) -> Option<u8> {
        (self.len > 0).then_some(self.buf[0])
    }

    fn push(&mut self, byte: u8) {
        if self.len() < FIFO_DEPTH {
            self.buf[self.len()] = byte;
            self.len += 1;
        }
    }

    fn pop(&mut self) -> Option<u8> {
        let byte = self.front()?;
        self.buf.copy_within(1.., 0);
        self.len -= 1;
        Some(byte)
    }

    fn clear(&mut self) {
        *self = Fifo::default();
    }
}

// ---------------------------------------------------------------------------
// State
// ---------------------------------------------------------------------------

/// Everything the guest can see or change.
#[derive(Debug, Default, Clone, Copy, PartialEq, Eq)]
struct State {
    cr1: u32,
    cr2: u32,
    cr3: u32,
    brr: u32,
    gtpr: u32,
    /// The status flags, in the shared bit positions.
    sr: u32,
    /// The byte the receiver is holding, valid while `RXNE` is set.
    rdr: u8,
    /// The byte the transmitter is holding, valid while `TXE` is clear.
    tdr: u8,
    /// Whether the `SR` read that arms the F4's read-read clear has happened.
    /// Meaningless on an F7, which clears through `ICR`.
    sr_read: bool,
    /// The level the interrupt output is at.
    irq: bool,
    /// `PRESC` (`u5`).
    presc: u32,
    /// `AUTOCR` (`u5`).
    autocr: u32,
    /// `RTOR` (`u5`; stored, the timeout is not modelled).
    rtor: u32,
    /// The transmit side (`u5`), one deep unless `FIFOEN`.
    txq: Fifo,
    /// The receive side (`u5`), one deep unless `FIFOEN`.
    rxq: Fifo,
}

impl State {
    fn reset() -> State {
        State {
            sr: SR_RESET,
            ..State::default()
        }
    }
}

// ---------------------------------------------------------------------------
// The register block
// ---------------------------------------------------------------------------

/// The register block, as something an address space can dispatch to.
struct Registers {
    state: Mutex<State>,
    port: Arc<dyn CharDevice>,
    /// The name the port was opened under, for `Debug` and diagnostics.
    port_name: String,
    variant: Variant,
    /// An LPUART: a 20-bit `BRR`, and no `GTPR` or `RTOR`.
    lpuart: bool,
    /// The interrupt output, connected at realize time.
    irq: Mutex<Option<WireSource>>,
    /// The receive DMA request output, if a board wired one.
    dma_rx: Mutex<Option<WireSource>>,
    /// The transmit DMA request output, if a board wired one.
    dma_tx: Mutex<Option<WireSource>>,
    /// `RCC_APB1ENR.USART2EN` and `RCC_APB1RSTR.USART2RST`, as a board's wires
    /// deliver them.
    gate: ClockGate,
}

impl fmt::Debug for Registers {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let mut s = f.debug_struct("Registers");
        s.field("port", &self.port_name)
            .field("variant", &self.variant);
        match self.state.try_lock() {
            Some(state) => s.field("state", &*state),
            None => s.field("state", &"<locked>"),
        };
        s.finish()
    }
}

impl Registers {
    /// Whether the peripheral is switched on.
    fn enabled(&self, state: &State) -> bool {
        state.cr1 & self.variant.ue() != 0
    }

    /// Whether an enabled interrupt condition is standing (RM0090 Table 149).
    fn irq_pending(&self, state: &State) -> bool {
        if !self.enabled(state) {
            return false;
        }
        let cr1 = state.cr1;
        let sr = self.isr(state);
        (sr & SR_TXE != 0 && cr1 & CR1_TXEIE != 0)
            || (sr & SR_TC != 0 && cr1 & CR1_TCIE != 0)
            || (sr & SR_RXNE != 0 && cr1 & CR1_RXNEIE != 0)
            || (sr & SR_IDLE != 0 && cr1 & CR1_IDLEIE != 0)
            || (sr & SR_PE != 0 && cr1 & CR1_PEIE != 0)
            // "ORE, NF or FE bits in the USART_SR register" — the error
            // interrupt is `CR3.EIE` and not a `CR1` bit.
            || (sr & (SR_ORE | SR_NF | SR_FE) != 0 && state.cr3 & CR3_EIE != 0)
            // The `u5` FIFO conditions. The flags are never set on the other
            // variants, so these cost them nothing.
            || (sr & ISR_TXFE != 0 && cr1 & CR1_TXFEIE != 0)
            || (sr & ISR_RXFF != 0 && cr1 & CR1_RXFFIE != 0)
            || (sr & ISR_TXFT != 0 && state.cr3 & CR3_TXFTIE != 0)
            || (sr & ISR_RXFT != 0 && state.cr3 & CR3_RXFTIE != 0)
    }

    /// Whether an access of `len` bytes at `offset` is one this layout takes:
    /// a word, or on the bridged layouts a naturally aligned byte or
    /// half-word (see [`MemOps::constraints`]).
    fn lane_access(&self, offset: u64, len: usize) -> bool {
        match len {
            4 => true,
            1 | 2 => self.variant != Variant::F4 && offset.is_multiple_of(len as u64),
            _ => false,
        }
    }

    /// The two DMA request levels, `(rx, tx)`: the data flag ANDed with its
    /// `CR3` enable, on a switched-on peripheral.
    fn dma_requests(&self, state: &State) -> (bool, bool) {
        if !self.enabled(state) {
            return (false, false);
        }
        let sr = self.isr(state);
        (
            sr & SR_RXNE != 0 && state.cr3 & CR3_DMAR != 0,
            sr & SR_TXE != 0 && state.cr3 & CR3_DMAT != 0,
        )
    }

    /// The status word as the guest reads it. On a `u5` that is the stored
    /// flags plus the ones derived from `CR1` and the FIFOs; elsewhere it is
    /// the stored flags.
    fn isr(&self, state: &State) -> u32 {
        if self.variant != Variant::U5 {
            return state.sr;
        }
        let mut isr = state.sr;
        if self.enabled(state) {
            // "TEACK: … set/reset by hardware, when the TE bit is set/reset";
            // the same for `REACK`. Instant here: there is no baud clock to
            // wait a bit time on.
            if state.cr1 & CR1_TE != 0 {
                isr |= ISR_TEACK;
            }
            if state.cr1 & CR1_RE != 0 {
                isr |= ISR_REACK;
            }
        }
        if state.cr1 & CR1_FIFOEN != 0 {
            let tx = state.txq.len();
            let rx = state.rxq.len();
            if tx == 0 {
                isr |= ISR_TXFE;
            }
            if rx == FIFO_DEPTH {
                isr |= ISR_RXFF;
            }
            let txft = ((state.cr3 >> CR3_TXFTCFG_SHIFT) & 7) as usize;
            if THRESHOLDS.get(txft).is_some_and(|&n| FIFO_DEPTH - tx >= n) {
                isr |= ISR_TXFT;
            }
            let rxft = ((state.cr3 >> CR3_RXFTCFG_SHIFT) & 7) as usize;
            if THRESHOLDS.get(rxft).is_some_and(|&n| rx >= n) {
                isr |= ISR_RXFT;
            }
        }
        isr
    }

    /// A `u5`'s FIFO depth: eight with `FIFOEN`, one without.
    fn depth(state: &State) -> usize {
        if state.cr1 & CR1_FIFOEN != 0 {
            FIFO_DEPTH
        } else {
            1
        }
    }

    /// Bring a `u5`'s stored `TXE`/`RXNE` in line with its queues — which is
    /// what makes them `TXFNF` and `RXFNE` in FIFO mode.
    fn sync_fifo_flags(state: &mut State) {
        if state.txq.len() < Registers::depth(state) {
            state.sr |= SR_TXE;
        } else {
            state.sr &= !SR_TXE;
        }
        if state.rxq.len() > 0 {
            state.sr |= SR_RXNE;
        } else {
            state.sr &= !SR_RXNE;
        }
    }

    /// Drive the interrupt pin and both DMA requests to whatever the
    /// registers now say.
    ///
    /// Called with **no lock held**: a sink is free to call back into this
    /// device, and the re-entrancy contract is that outward calls happen after
    /// the critical section rather than inside it.
    fn refresh_irq(&self) {
        let (level, rx, tx) = {
            let mut state = self.state.lock();
            state.irq = self.irq_pending(&state);
            let (rx, tx) = self.dma_requests(&state);
            (
                Level::from_bool(state.irq),
                Level::from_bool(rx),
                Level::from_bool(tx),
            )
        };
        for (pin, level) in [(&self.irq, level), (&self.dma_rx, rx), (&self.dma_tx, tx)] {
            let source = pin.lock().clone();
            if let Some(source) = source {
                source.set(level);
            }
        }
    }

    /// Take a byte from the host if the receiver is enabled and free.
    ///
    /// Returns whether anything moved.
    fn poll_receiver(&self) -> bool {
        {
            let state = self.state.lock();
            if !self.enabled(&state) || state.cr1 & CR1_RE == 0 {
                return false;
            }
            if self.variant == Variant::U5 {
                if state.rxq.len() >= Registers::depth(&state) {
                    // Full: the byte stays on the host's side, as below.
                    return false;
                }
            } else if state.sr & SR_RXNE != 0 {
                // The register is still full, so the byte stays on the host's
                // side of the seam rather than being taken and dropped. An
                // overrun is what happens to a byte the *line* delivered, and
                // the host queue is not the line.
                return false;
            }
        }
        // The port's lock is a leaf and this device's is `LockRank::DEVICE`,
        // so taking one inside the other would be the ranked order — but it
        // is not needed here, and a read that answers now is cheaper outside.
        let Some(byte) = self.port.read_byte() else {
            return false;
        };
        let mut state = self.state.lock();
        if self.variant == Variant::U5 {
            state.rxq.push(byte);
            Registers::sync_fifo_flags(&mut state);
            return true;
        }
        state.rdr = byte;
        state.sr |= SR_RXNE;
        state.sr_read = false;
        true
    }

    /// Hand the byte the transmitter is holding to the host.
    ///
    /// Returns whether anything moved. A port that will not take it leaves
    /// `TXE` clear, which stalls a guest polling it — back pressure arriving
    /// as the hardware would deliver it, not a dropped character.
    fn transmit(&self) -> bool {
        if self.variant == Variant::U5 {
            return self.transmit_fifo();
        }
        let byte = {
            let state = self.state.lock();
            if state.sr & SR_TXE != 0 {
                return false;
            }
            state.tdr
        };
        if !self.port.write_byte(byte) {
            return false;
        }
        let mut state = self.state.lock();
        state.sr |= SR_TXE | SR_TC;
        true
    }

    /// [`transmit`](Self::transmit) on a `u5`: the head of the queue goes, and
    /// `TC` sets once nothing is left behind it.
    fn transmit_fifo(&self) -> bool {
        let Some(byte) = self.state.lock().txq.front() else {
            return false;
        };
        if !self.port.write_byte(byte) {
            return false;
        }
        let mut state = self.state.lock();
        state.txq.pop();
        if state.txq.len() == 0 {
            state.sr |= SR_TC;
        }
        Registers::sync_fifo_flags(&mut state);
        true
    }

    /// The F4's read-read/read-write flag clear (RM0090 §30.6.1).
    ///
    /// `ORE` and the error flags are "cleared by a software sequence: a read
    /// to the USART_SR register followed by a read to the USART_DR register".
    /// An F7 clears them through `ICR` instead and this does nothing there.
    fn clear_after_data_access(state: &mut State) {
        if state.sr_read {
            state.sr &= !(SR_ORE | SR_NF | SR_FE | SR_PE | SR_IDLE);
            state.sr_read = false;
        }
    }

    /// Read the data register: the byte, and `RXNE` cleared.
    fn read_data(&self, state: &mut State, debug: bool) -> u32 {
        if self.variant == Variant::U5 {
            // An empty FIFO reads the last byte again, as the part does.
            if debug {
                return u32::from(state.rxq.front().unwrap_or(state.rdr));
            }
            if let Some(byte) = state.rxq.pop() {
                state.rdr = byte;
            }
            Registers::sync_fifo_flags(state);
            return u32::from(state.rdr);
        }
        if debug {
            return u32::from(state.rdr);
        }
        state.sr &= !SR_RXNE;
        Registers::clear_after_data_access(state);
        u32::from(state.rdr)
    }

    /// Write the data register: the byte is latched and `TXE` goes clear.
    fn write_data(&self, state: &mut State, value: u32) {
        if state.cr1 & CR1_TE == 0 || !self.enabled(state) {
            // A disabled transmitter latches nothing. Firmware that forgot
            // `TE` should see `TXE` stay set and its bytes go nowhere, which
            // is what happens on the part.
            return;
        }
        if self.variant == Variant::U5 {
            // A write to a full FIFO is lost: firmware is told by `TXFNF`.
            if state.txq.len() < Registers::depth(state) {
                state.txq.push(value as u8);
                state.tdr = value as u8;
                state.sr &= !SR_TC;
            }
            Registers::sync_fifo_flags(state);
            return;
        }
        state.tdr = value as u8;
        state.sr &= !(SR_TXE | SR_TC);
        Registers::clear_after_data_access(state);
    }

    /// Read one register. Returns the value.
    fn read_register(&self, offset: u64, debug: bool) -> u32 {
        let mut state = self.state.lock();
        match (self.variant, offset) {
            (Variant::F4, 0x00) => {
                let sr = state.sr;
                if !debug {
                    // The read that arms the two-step clear. A debug read must
                    // not arm it (`ROADMAP.md` §15, invariant 5).
                    state.sr_read = true;
                }
                sr
            }
            (Variant::F4, 0x04) => self.read_data(&mut state, debug),
            (Variant::F4, 0x08) => state.brr,
            (Variant::F4, 0x0c) => state.cr1,
            (Variant::F4, 0x10) => state.cr2,
            (Variant::F4, 0x14) => state.cr3,
            (Variant::F4, 0x18) => state.gtpr,

            // The `u5` additions first; it shares the rest with the F7.
            (Variant::U5, 0x10 | 0x14) if self.lpuart => 0,
            (Variant::U5, 0x14) => state.rtor,
            // `RQR` is write-only.
            (Variant::U5, 0x18) => 0,
            (Variant::U5, 0x1c) => self.isr(&state),
            (Variant::U5, 0x2c) => state.presc,
            (Variant::U5, 0x30) => state.autocr,

            (Variant::F7 | Variant::U5, 0x00) => state.cr1,
            (Variant::F7 | Variant::U5, 0x04) => state.cr2,
            (Variant::F7 | Variant::U5, 0x08) => state.cr3,
            (Variant::F7 | Variant::U5, 0x0c) => state.brr,
            (Variant::F7 | Variant::U5, 0x10) => state.gtpr,
            // `ISR` has no read side effect at all on an F7: `ICR` is what
            // clears a flag, which is the whole reason ST changed it.
            (Variant::F7, 0x1c) => state.sr,
            // "The USART_ICR register should be written only …"; it reads as
            // zero.
            (Variant::F7 | Variant::U5, 0x20) => 0,
            (Variant::F7 | Variant::U5, 0x24) => self.read_data(&mut state, debug),
            // `TDR` is write-only in effect; a read returns what was last put
            // there, which is what the part does and what nothing relies on.
            (Variant::F7 | Variant::U5, 0x28) => u32::from(state.tdr),
            // `RTOR` and `RQR` are not modelled; they read as zero, as does
            // anything else in the aperture.
            _ => 0,
        }
    }

    /// Write one register.
    fn write_register(&self, offset: u64, value: u32) {
        let mut state = self.state.lock();
        match (self.variant, offset) {
            (Variant::F4, 0x00) => {
                // "The software sequence … TC bit can also be cleared by
                // writing a '0' to it." Only the two writable flags.
                state.sr &= value | !(SR_TC | SR_RXNE);
            }
            (Variant::F4, 0x04) => self.write_data(&mut state, value),
            (Variant::F4, 0x08) => state.brr = value & 0xffff,
            (Variant::F4, 0x0c) => state.cr1 = value,
            (Variant::F4, 0x10) => state.cr2 = value,
            (Variant::F4, 0x14) => state.cr3 = value,
            (Variant::F4, 0x18) => state.gtpr = value & 0xffff,

            (Variant::U5, 0x00) => {
                let was_enabled = self.enabled(&state);
                state.cr1 = value;
                if was_enabled && !self.enabled(&state) {
                    // "When UE = 0 … all current operations are discarded …
                    // all the USART_ISR status flags are reset" (RM0456).
                    state.txq.clear();
                    state.rxq.clear();
                    state.sr = SR_RESET;
                }
                Registers::sync_fifo_flags(&mut state);
            }
            (Variant::U5, 0x0c) if self.lpuart => state.brr = value & 0xf_ffff,
            (Variant::U5, 0x10 | 0x14) if self.lpuart => {}
            (Variant::U5, 0x14) => state.rtor = value,
            (Variant::U5, 0x18) => {
                if value & RQR_RXFRQ != 0 {
                    state.rxq.clear();
                }
                if value & RQR_TXFRQ != 0 {
                    state.txq.clear();
                    state.sr |= SR_TC;
                }
                Registers::sync_fifo_flags(&mut state);
            }
            (Variant::U5, 0x2c) => state.presc = value & PRESC_MASK,
            (Variant::U5, 0x30) => state.autocr = value & AUTOCR_MASK,

            (Variant::F7, 0x00) => state.cr1 = value,
            (Variant::F7 | Variant::U5, 0x04) => state.cr2 = value,
            (Variant::F7 | Variant::U5, 0x08) => state.cr3 = value,
            (Variant::F7 | Variant::U5, 0x0c) => state.brr = value & 0xffff,
            (Variant::F7 | Variant::U5, 0x10) => state.gtpr = value & 0xffff,
            // `ISR` is read-only on an F7.
            (Variant::F7 | Variant::U5, 0x1c) => {}
            (Variant::F7 | Variant::U5, 0x20) => state.sr &= !(value & ICR_MASK),
            (Variant::F7 | Variant::U5, 0x24) => {}
            (Variant::F7 | Variant::U5, 0x28) => self.write_data(&mut state, value),
            _ => {}
        }
    }
}

impl Gated for Registers {
    fn clock_gate(&self) -> &ClockGate {
        &self.gate
    }

    fn gate_reset(&self) {
        // `RCC_APB1RSTR.USART2RST`: the register file goes back to its reset
        // values. The host port's queues do not — what a user has typed is the
        // host's state and not the machine's, exactly as at `Device::reset`.
        *self.state.lock() = State::reset();
        self.refresh_irq();
    }
}

impl MemOps for Registers {
    fn read(&self, offset: u64, dst: &mut [u8], attrs: MemAttrs) -> MemResult {
        if !self.lane_access(offset, dst.len()) {
            return Err(BusError::BadAccess);
        }
        if !self.gate.live() {
            // No `RCC_APB1ENR.USART2EN`, or `USART2RST` standing: the block is
            // not readable and the value that comes back is zero
            // ([`gate`](super::gate)). A debug read sees the same nothing,
            // because that is what the bus itself returns.
            dst.fill(0);
            return Ok(());
        }
        if !attrs.debug {
            // A byte the host has delivered should be visible the instant
            // firmware looks, not one scheduler tick later.
            self.poll_receiver();
        }
        // A narrow read is a word read at the APB side, side effects and all
        // (a byte read of `RDR` consumes the character); the access gets its
        // own lanes of it.
        let value = self.read_register(offset & !3, attrs.debug);
        let lane = (offset & 3) as usize;
        dst.copy_from_slice(&value.to_le_bytes()[lane..lane + dst.len()]);
        if !attrs.debug {
            self.refresh_irq();
        }
        Ok(())
    }

    fn write(&self, offset: u64, src: &[u8], attrs: MemAttrs) -> MemResult {
        if !self.lane_access(offset, src.len()) {
            return Err(BusError::BadAccess);
        }
        if attrs.debug {
            // A debug write to the data register would put a character on the
            // console and one to `CR1` would change when the guest is next
            // interrupted. Neither can be made harmless, so it is refused
            // rather than guessed at (`ROADMAP.md` §15, invariant 5).
            return Err(BusError::BadAccess);
        }
        if !self.gate.live() {
            // Not effective, character and all: a write to an unclocked
            // peripheral is lost, so this one reaches no chardev.
            return Ok(());
        }
        // The bridge duplicates a narrow write's data on every lane and makes
        // it a word write (RM0434 §13.4.6): a byte `0xB0` is `0xB0B0B0B0`.
        let value = match *src {
            [b] => u32::from(b) * 0x0101_0101,
            [l, h] => u32::from(u16::from_le_bytes([l, h])) * 0x0001_0001,
            [a, b, c, d] => u32::from_le_bytes([a, b, c, d]),
            _ => return Err(BusError::BadAccess),
        };
        self.write_register(offset & !3, value);
        self.refresh_irq();
        Ok(())
    }

    fn constraints(&self) -> AccessConstraints {
        if self.variant == Variant::F4 {
            // "The peripheral registers have to be accessed by words (32
            // bits)" — RM0090 §30.6, and that is the model for the F4 layout:
            // its bridge's narrow-access rule was not checked against RM0090.
            return AccessConstraints::word(Width::U32, Endian::Little);
        }
        // RM0434 §36.8 says words too, but the register is behind the AHB/APB
        // bridge, which "changes any AHB byte or half-word transfer into a
        // 32-bit APB transfer" (§13.4.6) rather than faulting it — and a DMA
        // channel with `PSIZE` = byte, which is how every HAL UART DMA is
        // set up, depends on it. So narrow accesses are taken, naturally
        // aligned, and widened in `read`/`write`.
        AccessConstraints::word(Width::U32, Endian::Little).with_widths(Width::U8, Width::U32)
    }
}

// ---------------------------------------------------------------------------
// The device
// ---------------------------------------------------------------------------

/// An STM32 USART.
#[derive(Debug)]
pub struct Usart {
    regs: Arc<Registers>,
    region: RegionRef,
}

impl Usart {
    /// Validate `props` and build the device.
    ///
    /// # Errors
    ///
    /// [`Error::Property`] if a property is of the wrong kind or value, or if
    /// one this class does not know was given.
    pub fn new(props: &Props) -> Result<Usart> {
        let mut r = props.reader();
        let port_name = r.or("port", String::from(DEFAULT_PORT))?;
        let variant = match r.or_enum("variant", "f4", &["f4", "f7", "u5"])? {
            "f7" => Variant::F7,
            "u5" => Variant::U5,
            _ => Variant::F4,
        };
        let lpuart = r.or("lpuart", false)?;
        r.finish()?;
        if lpuart && variant != Variant::U5 {
            return Err(Error::Property(
                "`lpuart` needs `variant = \"u5\"`: an LPUART is modelled on that register file only"
                    .to_string(),
            ));
        }
        let port = ports::attach(props, &port_name)?;
        Ok(Usart::build(port, port_name, variant, lpuart))
    }

    /// Build an LPUART against a character device the caller already has: the
    /// `u5` register file with a 20-bit `BRR`, and `GTPR` and `RTOR` reading
    /// as zero.
    #[must_use]
    pub fn lpuart_with_port(port: Arc<dyn CharDevice>, port_name: String) -> Usart {
        Usart::build(port, port_name, Variant::U5, true)
    }

    /// Whether this is an LPUART.
    #[must_use]
    pub fn is_lpuart(&self) -> bool {
        self.regs.lpuart
    }

    /// Build one against a character device the caller already has — the route
    /// a test takes, holding the other end of the port.
    #[must_use]
    pub fn with_port(port: Arc<dyn CharDevice>, port_name: String, variant: Variant) -> Usart {
        Usart::build(port, port_name, variant, false)
    }

    fn build(
        port: Arc<dyn CharDevice>,
        port_name: String,
        variant: Variant,
        lpuart: bool,
    ) -> Usart {
        let regs = Arc::new(Registers {
            state: Mutex::with_rank(LockRank::DEVICE, State::reset()),
            port,
            port_name,
            variant,
            lpuart,
            irq: Mutex::with_rank(LockRank::WIRE, None),
            dma_rx: Mutex::with_rank(LockRank::WIRE, None),
            dma_tx: Mutex::with_rank(LockRank::WIRE, None),
            gate: ClockGate::new(),
        });
        let region = Arc::new(Region::io(
            "usart",
            variant.register_bytes(),
            Arc::clone(&regs) as Arc<dyn MemOps>,
        ));
        Usart { regs, region }
    }

    /// Which register layout this instance has.
    #[must_use]
    pub fn variant(&self) -> Variant {
        self.regs.variant
    }

    /// `BRR`'s programmed value — the mantissa and fraction of the divisor.
    ///
    /// Reported, not acted on: see the module documentation.
    #[must_use]
    pub fn baud_divisor(&self) -> u32 {
        self.regs.state.lock().brr
    }

    /// The level the interrupt output is driving. High is "requesting".
    #[must_use]
    pub fn irq_level(&self) -> Level {
        Level::from_bool(self.regs.state.lock().irq)
    }

    /// Connect the interrupt output.
    pub fn connect_irq(&self, source: WireSource) {
        *self.regs.irq.lock() = Some(source);
        self.regs.refresh_irq();
    }
}

impl Device for Usart {
    fn class(&self) -> &'static DeviceClass {
        &CLASS
    }

    fn realize(&self, _ctx: &mut RealizeCtx<'_>) -> Result<()> {
        // Nothing outward: a `map` statement places the region.
        Ok(())
    }

    fn reset(&self, _kind: ResetKind) {
        // Both kinds of reset on an STM32 pull the peripheral's reset line.
        // The port's queues are not touched: what the user has typed is the
        // host's state, not the machine's.
        *self.regs.state.lock() = State::reset();
        self.regs.refresh_irq();
    }

    fn save(&self, w: &mut ChunkWriter<'_>) -> Result<()> {
        let state = *self.regs.state.lock();
        for value in [
            state.cr1, state.cr2, state.cr3, state.brr, state.gtpr, state.sr,
        ] {
            w.write_u32(value)?;
        }
        w.write_u8(state.rdr)?;
        w.write_u8(state.tdr)?;
        w.write_bool(state.sr_read)?;
        w.write_bool(state.irq)?;
        // The `u5` state follows, on a `u5` only: the variant is a construction
        // property, so the F4 and F7 encodings are what they always were.
        if self.regs.variant == Variant::U5 {
            w.write_u32(state.presc)?;
            w.write_u32(state.autocr)?;
            w.write_u32(state.rtor)?;
            for q in [state.txq, state.rxq] {
                w.write_u8(q.len)?;
                for byte in q.buf {
                    w.write_u8(byte)?;
                }
            }
        }
        Ok(())
        // The port's queues are deliberately absent: what a user has typed and
        // not yet been read, and what the terminal has shown, are the host's
        // state and not the machine's (`ROADMAP.md` §4.5).
    }

    fn load(&self, r: &mut ChunkReader<'_>) -> Result<()> {
        let mut state = State::reset();
        state.cr1 = r.read_u32()?;
        state.cr2 = r.read_u32()?;
        state.cr3 = r.read_u32()?;
        state.brr = r.read_u32()?;
        state.gtpr = r.read_u32()?;
        state.sr = r.read_u32()?;
        state.rdr = r.read_u8()?;
        state.tdr = r.read_u8()?;
        state.sr_read = r.read_bool()?;
        state.irq = r.read_bool()?;
        if self.regs.variant == Variant::U5 {
            state.presc = r.read_u32()? & PRESC_MASK;
            state.autocr = r.read_u32()? & AUTOCR_MASK;
            state.rtor = r.read_u32()?;
            for q in [&mut state.txq, &mut state.rxq] {
                let len = r.read_u8()?;
                if usize::from(len) > FIFO_DEPTH {
                    return Err(Error::State(format!(
                        "snapshot has {len} bytes in an {FIFO_DEPTH}-deep FIFO"
                    )));
                }
                q.len = len;
                for byte in &mut q.buf {
                    *byte = r.read_u8()?;
                }
            }
        }
        *self.regs.state.lock() = state;
        self.regs.refresh_irq();
        Ok(())
    }

    fn region(&self, name: &str) -> Option<RegionRef> {
        matches!(name, "" | "regs").then(|| Arc::clone(&self.region))
    }

    fn connect(&self, port: &str, source: WireSource) -> Result<()> {
        let pin = match port {
            IRQ_PIN => &self.regs.irq,
            DMA_RX_PIN => &self.regs.dma_rx,
            DMA_TX_PIN => &self.regs.dma_tx,
            _ => {
                return Err(Error::Config {
                    at: port.to_string(),
                    message: format!(
                        "a USART drives `{IRQ_PIN}`, `{DMA_RX_PIN}` and `{DMA_TX_PIN}`"
                    ),
                });
            }
        };
        *pin.lock() = Some(source);
        self.regs.refresh_irq();
        Ok(())
    }

    fn announce(&self, port: &str) {
        if matches!(port, IRQ_PIN | DMA_RX_PIN | DMA_TX_PIN) {
            self.regs.refresh_irq();
        }
    }

    fn sink(&self, port: &str, sources: &[WireId]) -> Option<SinkPin> {
        // `enable` and `reset`, and the USART has no other input: its data
        // path is the character seam rather than a pin.
        gate::sink(&self.regs, port, sources)
    }

    fn is_runnable(&self) -> bool {
        // Not because it executes anything, but because a character time is
        // real: the receiver has to be filled from the host and a refused
        // transmission has to be retried, and the scheduler is the only thing
        // allowed to decide when (`CLAUDE.md`: a device never reads the wall
        // clock). One tick of this device's domain is one character time.
        true
    }

    fn run(&self, budget: Budget) -> Consumed {
        if !self.regs.gate.live() {
            // No clock, no character time: the transmitter does not shift and
            // the receiver does not sample. Both pick up where they stopped
            // when the gate opens again.
            return Consumed::new(budget.ticks);
        }
        // At most one character per call, which is right however many ticks
        // the budget covers: a board sets the domain to the character rate.
        let moved = self.regs.transmit() | self.regs.poll_receiver();
        if moved {
            self.regs.refresh_irq();
        }
        Consumed::new(budget.ticks)
    }
}

impl Instance for Usart {}

/// The `st.usart` device class.
pub static CLASS: DeviceClass = DeviceClass {
    name: CLASS_NAME,
    version: STATE_VERSION,
    summary: "STM32 USART over a character port, in the F4 (SR/DR), F7 (ISR/RDR/TDR) or U5 (FIFO, PRESC) layout",
    properties: &[
        PropertySpec {
            name: "port",
            kind: ValueKind::Str,
            required: false,
            summary: "the character port to attach to, by name (default \"console\")",
        },
        PropertySpec {
            name: "variant",
            kind: ValueKind::Str,
            required: false,
            summary: "which register layout: \"f4\" (SR/DR), \"f7\" (ISR/RDR/TDR) or \"u5\" (F7 plus FIFO, PRESC, AUTOCR)",
        },
        PropertySpec {
            name: "lpuart",
            kind: ValueKind::Bool,
            required: false,
            summary: "an LPUART (20-bit BRR, no GTPR/RTOR); u5 only (default false)",
        },
    ],
    construct: |props| Ok(Box::new(Usart::new(props)?)),
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
    bindings.bind(CLASS_NAME, |props| Ok(Arc::new(Usart::new(props)?)))
}

/// What the validator should know about `st.usart`.
#[must_use]
pub fn schema() -> ClassSchema {
    let schema = ClassSchema::new(CLASS_NAME)
        .prop(PropSchema::new("port", ValueKind::Str))
        .prop(PropSchema::new("variant", ValueKind::Str).values(&["f4", "f7", "u5"]))
        .prop(PropSchema::new("lpuart", ValueKind::Bool))
        .region("")
        .region("regs")
        // An M-profile board wires this straight to the core: `wire
        // usart2.irq -> cpu.irq38`. There is no controller in between.
        .port(IRQ_PIN, PortDir::Out)
        // The request levels a DMA channel (or a DMAMUX input) takes; which
        // one is the part's request table, so the board file says.
        .port(DMA_RX_PIN, PortDir::Out)
        .port(DMA_TX_PIN, PortDir::Out);
    // `RCC_APB1ENR.USART2EN` and `RCC_APB1RSTR.USART2RST`.
    gate::ports(schema)
}

#[cfg(test)]
mod tests {
    use super::*;
    use alloc::vec::Vec;

    use crate::core::clock::GlobalTime;
    use crate::core::props::Value;
    use crate::core::registry::Registry;
    use crate::core::state::{MachineShape, Migrations, StateReader, StateWriter};
    use crate::core::sync::{AtomicU32, Ordering};
    use crate::core::wire::{Level, Wire, WireId, WireIdAllocator, WireSink};
    use crate::host::chardev::CharPort;

    /// Where each register is, in the variant under test.
    struct Map {
        sr: u64,
        rdr: u64,
        tdr: u64,
        brr: u64,
        cr1: u64,
        cr3: u64,
        icr: Option<u64>,
        /// The `CR1` bit that switches the peripheral on.
        ue: u32,
    }

    const F4_MAP: Map = Map {
        sr: 0x00,
        rdr: 0x04,
        tdr: 0x04,
        brr: 0x08,
        cr1: 0x0c,
        cr3: 0x14,
        icr: None,
        ue: CR1_UE_F4,
    };

    const F7_MAP: Map = Map {
        sr: 0x1c,
        rdr: 0x24,
        tdr: 0x28,
        brr: 0x0c,
        cr1: 0x00,
        cr3: 0x08,
        icr: Some(0x20),
        ue: CR1_UE_F7,
    };

    /// A USART with the far end of its port in hand.
    fn wired(variant: Variant) -> (Usart, Arc<CharPort>, &'static Map) {
        let port = Arc::new(CharPort::new());
        let usart = Usart::with_port(
            Arc::clone(&port) as Arc<dyn CharDevice>,
            "test".to_string(),
            variant,
        );
        let map = match variant {
            Variant::F4 => &F4_MAP,
            Variant::F7 | Variant::U5 => &F7_MAP,
        };
        (usart, port, map)
    }

    fn peek(usart: &Usart, offset: u64) -> u32 {
        let mut word = [0u8; 4];
        usart
            .regs
            .read(offset, &mut word, MemAttrs::DEFAULT)
            .expect("a word read is legal");
        u32::from_le_bytes(word)
    }

    fn peek_debug(usart: &Usart, offset: u64) -> u32 {
        let mut word = [0u8; 4];
        usart
            .regs
            .read(offset, &mut word, MemAttrs::DEBUG)
            .expect("a word read is legal");
        u32::from_le_bytes(word)
    }

    fn poke(usart: &Usart, offset: u64, value: u32) {
        usart
            .regs
            .write(offset, &value.to_le_bytes(), MemAttrs::DEFAULT)
            .expect("a word write is legal");
    }

    /// Switch the peripheral on with the transmitter and receiver enabled.
    fn enable(usart: &Usart, map: &Map) {
        poke(usart, map.cr1, map.ue | CR1_TE | CR1_RE);
    }

    /// One tick of the device's clock domain — one character time.
    fn tick(usart: &Usart) {
        Device::run(
            usart,
            Budget {
                until: GlobalTime::ZERO,
                ticks: 1,
            },
        );
    }

    #[test]
    fn a_peripheral_whose_clock_is_disabled_reads_as_zero() {
        // GitHub issue #14, and the reason `gate` exists: with
        // `RCC_APB1ENR.USART2EN` clear the block is not readable — every
        // register answers zero — and a write to it is not effective, so the
        // character never reaches the chardev.
        let (usart, port, map) = wired(Variant::F4);
        enable(&usart, map);
        assert_eq!(peek(&usart, map.sr) & SR_TXE, SR_TXE);

        let id = WireId::new(1);
        let pin = Device::sink(&usart, gate::ENABLE_PIN, &[id]).expect("a USART has `enable`");
        // Drawing the wire is the assertion that the clock is RCC's to give,
        // and an `xxENR` bit resets low.
        assert_eq!(peek(&usart, map.sr), 0, "the whole block reads as zero");
        assert_eq!(peek(&usart, map.cr1), 0);
        assert_eq!(peek_debug(&usart, map.sr), 0, "and so does a debug read");

        poke(&usart, map.tdr, u32::from(b'H'));
        tick(&usart);
        assert!(port.drain().is_empty(), "the write was not effective");

        pin.sink.set_level(id, 0, Level::High);
        // The registers kept their values across the gating: removing a clock
        // is not a reset (RM0090 §7.3.13 has `USART2RST` for that).
        assert_eq!(peek(&usart, map.cr1), map.ue | CR1_TE | CR1_RE);
        assert_eq!(peek(&usart, map.sr) & SR_TXE, SR_TXE, "nothing was queued");

        poke(&usart, map.tdr, u32::from(b'H'));
        tick(&usart);
        assert_eq!(port.drain(), b"H", "and now the clock is running");
    }

    #[test]
    fn an_unwired_enable_leaves_the_usart_clocked() {
        // Every board written before the gate existed draws no `enable` wire,
        // and none of them may go deaf because this landed.
        let (usart, port, map) = wired(Variant::F4);
        enable(&usart, map);
        poke(&usart, map.tdr, u32::from(b'x'));
        tick(&usart);
        assert_eq!(port.drain(), b"x");
    }

    #[test]
    fn the_reset_pin_puts_the_register_file_back() {
        // `RCC_APB1RSTR.USART2RST`, as a level: the block is deaf while it
        // stands and comes back at its reset values when it is let go.
        let (usart, _port, map) = wired(Variant::F4);
        enable(&usart, map);
        poke(&usart, map.brr, 0x1234);
        assert_eq!(peek(&usart, map.brr), 0x1234);

        let id = WireId::new(2);
        let pin = Device::sink(&usart, gate::RESET_PIN, &[id]).expect("a USART has `reset`");
        pin.sink.set_level(id, 0, Level::High);
        assert_eq!(peek(&usart, map.brr), 0, "held in reset, the block is deaf");
        pin.sink.set_level(id, 0, Level::Low);
        assert_eq!(
            peek(&usart, map.brr),
            0,
            "and what it kept is the reset value"
        );
        assert_eq!(peek(&usart, map.cr1), 0, "`UE` included");
        assert_eq!(peek(&usart, map.sr), SR_RESET);
    }

    #[test]
    fn a_byte_written_to_the_data_register_reaches_the_host() {
        for variant in [Variant::F4, Variant::F7, Variant::U5] {
            let (usart, port, map) = wired(variant);
            enable(&usart, map);
            assert_eq!(peek(&usart, map.sr) & SR_TXE, SR_TXE, "{variant:?}: idle");

            poke(&usart, map.tdr, u32::from(b'H'));
            assert_eq!(
                peek(&usart, map.sr) & (SR_TXE | SR_TC),
                0,
                "{variant:?}: the holding register is busy"
            );
            assert!(port.drain().is_empty(), "{variant:?}: not until a tick");

            tick(&usart);
            assert_eq!(port.drain(), b"H", "{variant:?}");
            assert_eq!(
                peek(&usart, map.sr) & (SR_TXE | SR_TC),
                SR_TXE | SR_TC,
                "{variant:?}: and now it is free"
            );
        }
    }

    #[test]
    fn a_disabled_transmitter_latches_nothing() {
        let (usart, port, map) = wired(Variant::F4);
        // `UE` set but `TE` clear: firmware that forgot half its init.
        poke(&usart, map.cr1, map.ue);
        poke(&usart, map.tdr, u32::from(b'X'));
        assert_eq!(peek(&usart, map.sr) & SR_TXE, SR_TXE);
        tick(&usart);
        assert!(port.drain().is_empty());

        // And with `UE` clear, nothing at all works.
        let (usart, port, map) = wired(Variant::F4);
        poke(&usart, map.cr1, CR1_TE);
        poke(&usart, map.tdr, u32::from(b'X'));
        tick(&usart);
        assert!(port.drain().is_empty());
    }

    #[test]
    fn a_byte_from_the_host_arrives_in_the_data_register() {
        for variant in [Variant::F4, Variant::F7, Variant::U5] {
            let (usart, port, map) = wired(variant);
            enable(&usart, map);
            port.feed(b"Z");
            // Visible the instant firmware looks, not one tick later.
            assert_eq!(peek(&usart, map.sr) & SR_RXNE, SR_RXNE, "{variant:?}");
            assert_eq!(peek(&usart, map.rdr), u32::from(b'Z'), "{variant:?}");
            assert_eq!(
                peek(&usart, map.sr) & SR_RXNE,
                0,
                "{variant:?}: reading the data register clears RXNE"
            );
        }
    }

    #[test]
    fn a_disabled_receiver_leaves_the_byte_on_the_hosts_side() {
        let (usart, port, map) = wired(Variant::F4);
        poke(&usart, map.cr1, map.ue); // no `RE`
        port.feed(b"Q");
        assert_eq!(peek(&usart, map.sr) & SR_RXNE, 0);
        // The byte was not taken and dropped: enabling the receiver finds it.
        enable(&usart, map);
        assert_eq!(peek(&usart, map.sr) & SR_RXNE, SR_RXNE);
        assert_eq!(peek(&usart, map.rdr), u32::from(b'Q'));
    }

    #[test]
    fn the_two_layouts_clear_a_flag_in_their_own_ways() {
        // The F4: "a read to the USART_SR register followed by a read to the
        // USART_DR register" (RM0090 §30.6.1).
        let (usart, _port, map) = wired(Variant::F4);
        enable(&usart, map);
        usart.regs.state.lock().sr |= SR_ORE;
        assert_eq!(peek(&usart, map.rdr) as u8, 0);
        assert_eq!(
            peek(&usart, map.sr) & SR_ORE,
            SR_ORE,
            "a data read alone does not clear it"
        );
        let _ = peek(&usart, map.sr); // arms the sequence
        let _ = peek(&usart, map.rdr); // completes it
        assert_eq!(peek(&usart, map.sr) & SR_ORE, 0);

        // The F7: write a one to `ICR`. `ISR` has no read side effect at all,
        // which is the whole reason ST changed it.
        let (usart, _port, map) = wired(Variant::F7);
        enable(&usart, map);
        usart.regs.state.lock().sr |= SR_ORE;
        let _ = peek(&usart, map.sr);
        let _ = peek(&usart, map.rdr);
        assert_eq!(
            peek(&usart, map.sr) & SR_ORE,
            SR_ORE,
            "an F7 does not clear on a read sequence"
        );
        poke(&usart, map.icr.unwrap(), SR_ORE);
        assert_eq!(peek(&usart, map.sr) & SR_ORE, 0);
        // `ICR` reads as zero.
        assert_eq!(peek(&usart, map.icr.unwrap()), 0);
    }

    #[test]
    fn the_f4_clears_tc_by_a_write_of_zero() {
        let (usart, _port, map) = wired(Variant::F4);
        enable(&usart, map);
        assert_eq!(peek(&usart, map.sr) & SR_TC, SR_TC);
        poke(&usart, map.sr, !SR_TC);
        assert_eq!(peek(&usart, map.sr) & SR_TC, 0);
        // And a write cannot *set* a flag.
        poke(&usart, map.sr, 0xffff_ffff);
        assert_eq!(peek(&usart, map.sr) & SR_TC, 0);
    }

    /// A sink that records the last level it was driven to.
    #[derive(Debug, Default)]
    struct LevelProbe {
        level: AtomicU32,
    }

    impl WireSink for LevelProbe {
        fn set_level(&self, _src: WireId, _line: u32, level: Level) {
            self.level
                .store(u32::from(level.is_high()), Ordering::Relaxed);
        }
    }

    impl LevelProbe {
        fn high(&self) -> bool {
            self.level.load(Ordering::Relaxed) != 0
        }
    }

    /// Wire `pin` of `usart` to a fresh probe.
    fn probe_pin(usart: &Usart, pin: &str) -> Arc<LevelProbe> {
        let ids = WireIdAllocator::new();
        let id = ids.alloc();
        let probe = Arc::new(LevelProbe::default());
        let wire = Wire::builder()
            .source(id)
            .sink(Arc::clone(&probe) as Arc<dyn WireSink>, 0)
            .build_shared();
        Device::connect(usart, pin, WireSource::new(wire, id)).expect("a USART drives it");
        probe
    }

    #[test]
    fn the_dma_requests_follow_the_data_flags_and_their_cr3_enables() {
        for variant in [Variant::F4, Variant::F7, Variant::U5] {
            let (usart, port, map) = wired(variant);
            let rx = probe_pin(&usart, DMA_RX_PIN);
            let tx = probe_pin(&usart, DMA_TX_PIN);

            // Switched off, nothing is requested whatever CR3 says.
            poke(&usart, map.cr3, CR3_DMAR | CR3_DMAT);
            assert!(!rx.high() && !tx.high(), "{variant:?}: UE clear");

            enable(&usart, map);
            // TXE stands out of reset: DMAT alone asks for a byte to send.
            assert!(tx.high(), "{variant:?}: TXE with DMAT");
            assert!(!rx.high(), "{variant:?}: nothing received yet");
            poke(&usart, map.cr3, CR3_DMAR);
            assert!(!tx.high(), "{variant:?}: DMAT clear");

            // A byte arrives: RXNE with DMAR raises the request, and the
            // read of the data register — the DMA's beat — drops it.
            port.feed(b"z");
            tick(&usart);
            assert!(rx.high(), "{variant:?}: RXNE with DMAR");
            assert_eq!(peek(&usart, map.rdr) & 0xff, u32::from(b'z'));
            assert!(!rx.high(), "{variant:?}: serviced by the read");

            // DMAR clear: the same byte only raises RXNE.
            poke(&usart, map.cr3, 0);
            port.feed(b"y");
            tick(&usart);
            assert_ne!(peek_debug(&usart, map.sr) & SR_RXNE, 0);
            assert!(!rx.high(), "{variant:?}: DMAR clear");
        }
    }

    #[test]
    fn a_bridged_layout_widens_byte_and_halfword_accesses() {
        let (usart, port, map) = wired(Variant::U5);
        enable(&usart, map);
        // A byte read of RDR — a DMA channel with PSIZE = byte — consumes the
        // character, as the word read the bridge makes of it does.
        port.feed(b"k");
        tick(&usart);
        let mut byte = [0u8; 1];
        usart
            .regs
            .read(map.rdr, &mut byte, MemAttrs::DEFAULT)
            .expect("the bridge takes a byte");
        assert_eq!(byte, *b"k");
        assert_eq!(peek_debug(&usart, map.sr) & SR_RXNE, 0, "consumed");
        // A byte write of TDR is the byte on every lane: one character out.
        usart
            .regs
            .write(map.tdr, b"m", MemAttrs::DEFAULT)
            .expect("the bridge takes a byte");
        for _ in 0..4 {
            tick(&usart);
        }
        assert_eq!(port.drain(), b"m");
        // A half-word read of the second half of ISR is its upper lanes.
        let isr = peek_debug(&usart, map.sr);
        let mut half = [0u8; 2];
        usart
            .regs
            .read(map.sr + 2, &mut half, MemAttrs::DEFAULT)
            .expect("aligned half-word");
        assert_eq!(u32::from(u16::from_le_bytes(half)), isr >> 16);
        // Misaligned is still refused, and the F4 layout keeps words only.
        assert!(
            usart
                .regs
                .read(map.sr + 1, &mut half, MemAttrs::DEFAULT)
                .is_err()
        );
        let (f4, _port, f4map) = wired(Variant::F4);
        assert!(
            f4.regs
                .read(f4map.rdr, &mut byte, MemAttrs::DEFAULT)
                .is_err()
        );
    }

    #[test]
    fn the_dma_requests_survive_a_debug_read_and_a_snapshot() {
        let (usart, port, map) = wired(Variant::U5);
        let rx = probe_pin(&usart, DMA_RX_PIN);
        enable(&usart, map);
        poke(&usart, map.cr3, CR3_DMAR);
        port.feed(b"q");
        tick(&usart);
        assert!(rx.high());
        // A debugger's look at RDR is not the DMA's beat.
        let _ = peek_debug(&usart, map.rdr);
        assert!(rx.high(), "a debug read services nothing");

        // The level is derived, not stored: a restored device drives it again.
        let mut shape = MachineShape::new();
        shape.add_device("usart", CLASS_NAME).unwrap();
        let mut w = StateWriter::new(shape);
        {
            let mut chunk = w.chunk("usart", CLASS_NAME, STATE_VERSION).unwrap();
            Device::save(&usart, &mut chunk).unwrap();
        }
        let bytes = w.to_vec().unwrap();
        let (restored, _port, _) = wired(Variant::U5);
        let rx2 = probe_pin(&restored, DMA_RX_PIN);
        let reader = StateReader::new(&bytes).unwrap();
        let chunk = reader
            .load("usart", CLASS_NAME, STATE_VERSION, &Migrations::new())
            .unwrap();
        Device::load(&restored, &mut chunk.reader()).unwrap();
        assert!(
            rx2.high(),
            "RXNE and DMAR came back, and so did the request"
        );
    }

    #[test]
    fn an_enabled_condition_raises_the_interrupt_pin() {
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

        let (usart, port, map) = wired(Variant::F4);
        let ids = WireIdAllocator::new();
        let id = ids.alloc();
        let probe = Arc::new(Probe::default());
        let wire = Wire::builder()
            .source(id)
            .sink(Arc::clone(&probe) as Arc<dyn WireSink>, 0)
            .build_shared();
        Device::connect(&usart, IRQ_PIN, WireSource::new(wire, id)).expect("a USART drives irq");
        assert!(
            Device::connect(&usart, "dma", dummy_source()).is_err(),
            "and nothing else"
        );

        enable(&usart, map);
        assert_eq!(probe.level.load(Ordering::Relaxed), 0, "no enable bit set");

        // `RXNEIE` and a byte: this is the pin a machine file wires to the
        // core's `irq38`.
        poke(&usart, map.cr1, map.ue | CR1_TE | CR1_RE | CR1_RXNEIE);
        port.feed(b"!");
        let _ = peek(&usart, map.sr);
        assert_eq!(probe.level.load(Ordering::Relaxed), 1);
        assert_eq!(usart.irq_level(), Level::High);
        let _ = peek(&usart, map.rdr);
        assert_eq!(probe.level.load(Ordering::Relaxed), 0, "serviced");

        // `EIE` is what makes an error raise one; `CR1` has no bit for it.
        usart.regs.state.lock().sr |= SR_ORE;
        usart.regs.refresh_irq();
        assert_eq!(probe.level.load(Ordering::Relaxed), 0);
        poke(&usart, map.cr3, CR3_EIE);
        assert_eq!(probe.level.load(Ordering::Relaxed), 1);

        // And a peripheral that is switched off requests nothing, whatever its
        // flags say.
        poke(&usart, map.cr1, 0);
        assert_eq!(probe.level.load(Ordering::Relaxed), 0);
    }

    #[test]
    fn a_debug_access_changes_nothing() {
        // Invariant 5: a status register is exactly the trap this is for.
        let (usart, port, map) = wired(Variant::F4);
        enable(&usart, map);
        port.feed(b"D");
        // A debug read does not even poll the port...
        assert_eq!(peek_debug(&usart, map.sr) & SR_RXNE, 0);
        // ...and once the byte is latched, a debug read of it leaves `RXNE`.
        assert_eq!(peek(&usart, map.sr) & SR_RXNE, SR_RXNE);
        assert_eq!(peek_debug(&usart, map.rdr), u32::from(b'D'));
        assert_eq!(peek(&usart, map.sr) & SR_RXNE, SR_RXNE, "still waiting");

        // Nor does it arm the F4's two-step clear. A fresh device, because a
        // real `SR` read above has already armed this one.
        let (fresh, _p, map) = wired(Variant::F4);
        enable(&fresh, map);
        fresh.regs.state.lock().sr |= SR_ORE;
        let _ = peek_debug(&fresh, map.sr);
        let _ = peek(&fresh, map.rdr);
        assert_eq!(peek(&fresh, map.sr) & SR_ORE, SR_ORE);

        // A debug write is refused rather than guessed at.
        assert_eq!(
            usart
                .regs
                .write(map.tdr, &u32::from(b'!').to_le_bytes(), MemAttrs::DEBUG),
            Err(BusError::BadAccess)
        );
        assert!(port.drain().is_empty());
    }

    #[test]
    fn only_a_full_word_is_a_legal_access() {
        let (usart, _port, map) = wired(Variant::F4);
        let mut byte = [0u8; 1];
        assert_eq!(
            usart.regs.read(map.sr, &mut byte, MemAttrs::DEFAULT),
            Err(BusError::BadAccess)
        );
        assert_eq!(
            usart.regs.constraints(),
            AccessConstraints::word(Width::U32, Endian::Little)
        );
    }

    #[test]
    fn brr_is_stored_and_reported_and_changes_no_rate() {
        let (usart, port, map) = wired(Variant::F4);
        enable(&usart, map);
        // 168 MHz / 4 APB1, 115200 baud, OVER8=0: mantissa 22, fraction 13.
        poke(&usart, map.brr, (22 << 4) | 13);
        assert_eq!(usart.baud_divisor(), (22 << 4) | 13);
        assert_eq!(peek(&usart, map.brr), (22 << 4) | 13);
        // Still one byte per tick, because the domain is the rate.
        poke(&usart, map.tdr, u32::from(b'a'));
        tick(&usart);
        assert_eq!(port.drain(), b"a");
    }

    #[test]
    fn a_reset_returns_the_documented_flags() {
        let (usart, _port, map) = wired(Variant::F4);
        enable(&usart, map);
        poke(&usart, map.brr, 0x1234);
        poke(&usart, map.tdr, u32::from(b'z'));
        Device::reset(&usart, ResetKind::Cold);
        assert_eq!(
            peek(&usart, map.sr),
            SR_RESET,
            "TC and TXE set, nothing else"
        );
        assert_eq!(peek(&usart, map.brr), 0);
        assert_eq!(peek(&usart, map.cr1), 0);
    }

    #[test]
    fn a_snapshot_round_trips_to_identical_state() {
        let (saved, port, map) = wired(Variant::F4);
        enable(&saved, map);
        poke(&saved, map.brr, (22 << 4) | 13);
        poke(&saved, map.cr3, CR3_EIE);
        port.feed(b"S");
        assert_eq!(peek(&saved, map.sr) & SR_RXNE, SR_RXNE);

        let mut shape = MachineShape::new();
        shape.add_device("usart", CLASS_NAME).unwrap();
        let mut w = StateWriter::new(shape);
        {
            let mut chunk = w.chunk("usart", CLASS_NAME, STATE_VERSION).unwrap();
            Device::save(&saved, &mut chunk).unwrap();
        }
        let bytes = w.to_vec().unwrap();

        let (restored, _other, _) = wired(Variant::F4);
        let reader = StateReader::new(&bytes).unwrap();
        let chunk = reader
            .load("usart", CLASS_NAME, STATE_VERSION, &Migrations::new())
            .unwrap();
        Device::load(&restored, &mut chunk.reader()).unwrap();

        let before: Vec<u32> = (0..7).map(|i| peek_debug(&saved, i * 4)).collect();
        let after: Vec<u32> = (0..7).map(|i| peek_debug(&restored, i * 4)).collect();
        assert_eq!(before, after);
        assert_eq!(
            peek(&restored, map.rdr),
            u32::from(b'S'),
            "the byte came across"
        );
    }

    #[test]
    fn a_property_this_class_does_not_know_is_a_typo() {
        let props = Props::new().with("variant", Value::from("f7"));
        assert_eq!(Usart::new(&props).unwrap().variant(), Variant::F7);
        assert_eq!(Variant::F7.as_str(), "f7");
        assert_eq!(
            Usart::new(&Props::new()).unwrap().variant(),
            Variant::F4,
            "the board this ships with is an F407"
        );
        assert!(Usart::new(&Props::new().with("variant", Value::from("h7"))).is_err());
        assert!(Usart::new(&Props::new().with("varient", Value::from("f4"))).is_err());
    }

    #[test]
    fn the_class_is_registrable_and_agrees_with_its_schema() {
        let mut reg = Registry::new();
        register(&mut reg).unwrap();
        assert!(register(&mut reg).is_err(), "twice is a collision");
        let device = reg.create(CLASS_NAME, &Props::new()).unwrap();
        assert_eq!(device.class().name, CLASS_NAME);

        let schema = schema();
        let port = schema.port_named(IRQ_PIN).expect("irq");
        assert_eq!(port.dir, PortDir::Out);
        assert!(schema.port_named("rx").is_none());

        let (usart, _p, _m) = wired(Variant::F4);
        assert!(Device::region(&usart, "").is_some());
        assert!(Device::region(&usart, "regs").is_some());
        assert!(Device::region(&usart, "fifo").is_none());
    }

    // -- the u5 variant -------------------------------------------------------

    const PRESC: u64 = 0x2c;
    const AUTOCR: u64 = 0x30;
    const RQR: u64 = 0x18;
    const GTPR: u64 = 0x10;
    const RTOR: u64 = 0x14;

    #[test]
    fn the_u5_hal_init_sequence_completes_and_prints() {
        // What `HAL_UART_Init` does on a U5: `UE` off, `CR1`/`CR2`/`CR3`,
        // `PRESC`, `BRR`, then `HAL_UARTEx_DisableFifoMode` (read `CR1`, drop
        // `UE`, drop `FIFOEN`, write it back), `UE` on, and
        // `UART_CheckIdleState` polling `TEACK` and `REACK`.
        let (usart, port, map) = wired(Variant::U5);
        assert_eq!(usart.region.len(), REGISTER_BYTES_U5);
        assert_eq!(peek(&usart, map.sr), SR_RESET, "ISR's reset value");
        poke(&usart, map.cr1, 0);
        poke(&usart, map.cr1, CR1_TE | CR1_RE);
        poke(&usart, 0x04, 0);
        poke(&usart, map.cr3, 0);
        poke(&usart, PRESC, 0);
        poke(&usart, map.brr, 0x0115); // 32 MHz / 115200
        let cr1 = peek(&usart, map.cr1);
        poke(&usart, map.cr1, cr1 & !CR1_UE_F7 & !CR1_FIFOEN);
        poke(&usart, map.cr1, cr1 | CR1_UE_F7);
        let isr = peek(&usart, map.sr);
        assert_eq!(isr & (ISR_TEACK | ISR_REACK), ISR_TEACK | ISR_REACK);
        assert_eq!(
            isr & (ISR_TXFE | ISR_TXFT),
            0,
            "no FIFO flags out of FIFO mode"
        );
        assert_eq!(peek(&usart, PRESC), 0);
        assert_eq!(peek(&usart, map.brr), 0x0115);

        // `HAL_UART_Transmit`: wait `TXE`, write `TDR`, and finally wait `TC`.
        for &byte in b"ok" {
            assert_eq!(peek(&usart, map.sr) & SR_TXE, SR_TXE);
            poke(&usart, map.tdr, u32::from(byte));
            assert_eq!(peek(&usart, map.sr) & SR_TXE, 0, "one deep without FIFOEN");
            tick(&usart);
        }
        assert_eq!(peek(&usart, map.sr) & SR_TC, SR_TC);
        assert_eq!(port.drain(), b"ok");

        // And `TEACK` follows `TE`.
        poke(&usart, map.cr1, CR1_UE_F7 | CR1_RE);
        assert_eq!(peek(&usart, map.sr) & (ISR_TEACK | ISR_REACK), ISR_REACK);
    }

    #[test]
    fn the_u5_fifo_holds_eight_each_way_and_reports_its_thresholds() {
        let (usart, port, map) = wired(Variant::U5);
        // TXFTCFG = 0b010 (half: four free), RXFTCFG = 0b001 (a quarter: two).
        poke(
            &usart,
            map.cr3,
            (0b010 << CR3_TXFTCFG_SHIFT) | (0b001 << CR3_RXFTCFG_SHIFT),
        );
        poke(&usart, map.cr1, CR1_FIFOEN | CR1_UE_F7 | CR1_TE | CR1_RE);
        let isr = peek(&usart, map.sr);
        assert_eq!(
            isr & (ISR_TXFE | ISR_TXFT | SR_TXE),
            ISR_TXFE | ISR_TXFT | SR_TXE
        );

        for byte in b"01234" {
            poke(&usart, map.tdr, u32::from(*byte));
        }
        let isr = peek(&usart, map.sr);
        assert_eq!(isr & ISR_TXFE, 0);
        assert_eq!(
            isr & ISR_TXFT,
            0,
            "three free is below the threshold of four"
        );
        assert_eq!(isr & SR_TXE, SR_TXE, "TXFNF: not full yet");
        for byte in b"567X" {
            poke(&usart, map.tdr, u32::from(*byte));
        }
        assert_eq!(peek(&usart, map.sr) & SR_TXE, 0, "full; the ninth was lost");
        for _ in 0..8 {
            tick(&usart);
        }
        assert_eq!(port.drain(), b"01234567");
        let isr = peek(&usart, map.sr);
        assert_eq!(isr & (ISR_TXFE | SR_TC), ISR_TXFE | SR_TC);

        port.feed(b"abcdefghij");
        // Reading `ISR` pulls one byte in at a time; ticks bring the rest.
        let _ = peek(&usart, map.sr);
        for _ in 0..10 {
            tick(&usart);
        }
        let isr = peek(&usart, map.sr);
        assert_eq!(
            isr & (ISR_RXFF | ISR_RXFT | SR_RXNE),
            ISR_RXFF | ISR_RXFT | SR_RXNE
        );
        let got: Vec<u8> = (0..7).map(|_| peek(&usart, map.rdr) as u8).collect();
        assert_eq!(got, b"abcdefg");
        // Each read polled the host first, so the FIFO has taken "hij" too.
        assert_eq!(peek_debug(&usart, map.rdr), u32::from(b'h'));
        assert_eq!(peek(&usart, map.rdr), u32::from(b'h'));
        assert_eq!(peek(&usart, map.rdr), u32::from(b'i'));
        assert_eq!(peek_debug(&usart, map.sr) & ISR_RXFT, 0, "one is below two");
        assert_eq!(peek(&usart, map.rdr), u32::from(b'j'));
        assert_eq!(peek(&usart, map.sr) & SR_RXNE, 0);
        assert_eq!(
            peek(&usart, map.rdr),
            u32::from(b'j'),
            "empty reads the last again"
        );
    }

    #[test]
    fn the_u5_fifo_interrupts_follow_their_enables() {
        let (usart, _port, map) = wired(Variant::U5);
        poke(&usart, map.cr1, CR1_FIFOEN | CR1_UE_F7 | CR1_TE);
        assert_eq!(usart.irq_level(), Level::Low);
        poke(
            &usart,
            map.cr1,
            CR1_FIFOEN | CR1_UE_F7 | CR1_TE | CR1_TXFEIE,
        );
        assert_eq!(usart.irq_level(), Level::High, "TXFE with TXFEIE");
        poke(&usart, map.tdr, u32::from(b'x'));
        assert_eq!(usart.irq_level(), Level::Low);
        poke(&usart, map.cr3, CR3_TXFTIE); // TXFTCFG 0: one free entry
        assert_eq!(usart.irq_level(), Level::High, "TXFT with TXFTIE");
    }

    #[test]
    fn the_u5_flushes_on_rqr_and_on_ue_going_low() {
        let (usart, port, map) = wired(Variant::U5);
        poke(&usart, map.cr1, CR1_FIFOEN | CR1_UE_F7 | CR1_TE | CR1_RE);
        port.feed(b"rx");
        let _ = peek(&usart, map.sr);
        tick(&usart);
        poke(&usart, map.tdr, u32::from(b't'));
        poke(&usart, RQR, RQR_RXFRQ | RQR_TXFRQ);
        let isr = peek_debug(&usart, map.sr);
        assert_eq!(isr & (SR_RXNE | ISR_TXFE | SR_TC), ISR_TXFE | SR_TC);
        assert_eq!(peek(&usart, RQR), 0, "write-only");
        tick(&usart);
        assert!(port.drain().is_empty(), "the flushed byte never left");

        poke(&usart, map.tdr, u32::from(b'u'));
        usart.regs.state.lock().sr |= SR_ORE;
        poke(&usart, map.cr1, CR1_FIFOEN | CR1_TE | CR1_RE);
        assert_eq!(peek(&usart, map.sr), SR_RESET | ISR_TXFE | ISR_TXFT);
        poke(&usart, map.cr1, CR1_FIFOEN | CR1_UE_F7 | CR1_TE | CR1_RE);
        tick(&usart);
        assert!(port.drain().is_empty(), "UE = 0 discarded it");
    }

    #[test]
    fn the_u5_extra_registers_store_their_defined_bits() {
        let (usart, _port, map) = wired(Variant::U5);
        poke(&usart, PRESC, 0xffff_ffff);
        poke(&usart, AUTOCR, 0xffff_ffff);
        poke(&usart, RTOR, 0x1234_5678);
        poke(&usart, map.brr, 0xf_ffff);
        assert_eq!(peek(&usart, PRESC), 0xf);
        assert_eq!(peek(&usart, AUTOCR), AUTOCR_MASK);
        assert_eq!(peek(&usart, RTOR), 0x1234_5678);
        assert_eq!(peek(&usart, map.brr), 0xffff, "a USART's BRR is 16 bits");

        // An LPUART: 20-bit `BRR`, no `GTPR`, no `RTOR`.
        let port = Arc::new(CharPort::new());
        let lp = Usart::lpuart_with_port(Arc::clone(&port) as Arc<dyn CharDevice>, "lp".into());
        assert!(lp.is_lpuart());
        poke(&lp, map.brr, 0xfff_ffff);
        poke(&lp, GTPR, 0xffff);
        poke(&lp, RTOR, 0xffff);
        assert_eq!(peek(&lp, map.brr), 0xf_ffff);
        assert_eq!((peek(&lp, GTPR), peek(&lp, RTOR)), (0, 0));
        poke(&lp, map.cr1, CR1_UE_F7 | CR1_TE);
        poke(&lp, map.tdr, u32::from(b'L'));
        tick(&lp);
        assert_eq!(port.drain(), b"L");

        let props = Props::new()
            .with("variant", Value::from("u5"))
            .with("lpuart", Value::from(true));
        assert!(Usart::new(&props).unwrap().is_lpuart());
        let props = Props::new().with("lpuart", Value::from(true));
        assert!(Usart::new(&props).is_err(), "an LPUART is a u5");
    }

    #[test]
    fn a_u5_snapshot_round_trips_to_identical_state() {
        let (saved, port, map) = wired(Variant::U5);
        poke(&saved, map.cr1, CR1_FIFOEN | CR1_UE_F7 | CR1_TE | CR1_RE);
        poke(&saved, PRESC, 3);
        poke(&saved, AUTOCR, 0x2_0004);
        poke(&saved, RTOR, 77);
        for byte in b"abc" {
            poke(&saved, map.tdr, u32::from(*byte));
        }
        port.feed(b"xy");
        let _ = peek(&saved, map.sr);
        tick(&saved);
        let _ = port.drain();

        let mut shape = MachineShape::new();
        shape.add_device("usart", CLASS_NAME).unwrap();
        let mut w = StateWriter::new(shape);
        {
            let mut chunk = w.chunk("usart", CLASS_NAME, STATE_VERSION).unwrap();
            Device::save(&saved, &mut chunk).unwrap();
        }
        let bytes = w.to_vec().unwrap();

        let (restored, other, _) = wired(Variant::U5);
        let reader = StateReader::new(&bytes).unwrap();
        let chunk = reader
            .load("usart", CLASS_NAME, STATE_VERSION, &Migrations::new())
            .unwrap();
        Device::load(&restored, &mut chunk.reader()).unwrap();

        let before: Vec<u32> = (0..13).map(|i| peek_debug(&saved, i * 4)).collect();
        let after: Vec<u32> = (0..13).map(|i| peek_debug(&restored, i * 4)).collect();
        assert_eq!(before, after);
        let saved_state = *saved.regs.state.lock();
        assert_eq!(saved_state, *restored.regs.state.lock());
        tick(&restored);
        tick(&restored);
        assert_eq!(other.drain(), b"bc", "the transmit FIFO came across");
    }

    /// A wire with one source, so a pin has something to drive.
    fn dummy_source() -> WireSource {
        let id = WireId::new(9);
        WireSource::new(Wire::builder().source(id).build_shared(), id)
    }
}
