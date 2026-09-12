//! The STM32 SPI peripheral, in both of the generations ST shipped it in.
//!
//! # Two generations, one register map
//!
//! ST revised this block once, and — unlike the [`usart`](super::usart), where
//! the revision moved every register — it moved nothing. `CR1`, `CR2`, `SR`,
//! `DR`, `CRCPR`, `RXCRCR`, `TXCRCR`, `I2SCFGR` and `I2SPR` sit at `0x00`…
//! `0x20` in both; the baud-rate divisor is `2^(BR + 1)` in both; and
//! `MSTR`/`CPOL`/`CPHA`/`LSBFIRST`/`BR`, `SSM`/`SSI`/`SSOE`, `MODF`, `OVR`,
//! `RXONLY`, `BIDIMODE` and the CRC machinery mean the same thing in both.
//! What the revision changed is the **data path**:
//!
//! | | `variant = "f4"` (RM0090 §28) | `variant = "f7"` (RM0351 §42) |
//! | --- | --- | --- |
//! | parts | F1, F2, **F4**, L1 | F0, F3, **F7**, L4, L4+, L5, G0, G4, WB |
//! | buffers | one word each way | a **four-byte FIFO** each way |
//! | frame size | `CR1.DFF`: 8 or 16 bits | `CR2.DS[3:0]`: **4 to 16 bits** |
//! | `CR1` bit 11 | `DFF` | `CRCL`, the CRC *length* |
//! | `CR2` reset | `0x0000` | `0x0700` — `DS` powers up at eight bits |
//! | `RXNE` rises | on a whole word | at the threshold `CR2.FRXTH` picks |
//! | `TXE` means | the buffer is free | the Tx FIFO is at or below half |
//! | fill levels | none | `SR.FRLVL[10:9]`, `SR.FTLVL[12:11]` |
//! | `DR` access width | irrelevant | an 8-bit access moves **one** byte, a 16-bit access **two** |
//! | odd-length DMA | n/a | `CR2.LDMA_TX`, `CR2.LDMA_RX` |
//!
//! The value is spelled `"f7"` to match [`usart`](super::usart)'s vocabulary
//! for the same generation of ST redesign — a board that writes
//! `variant = "f7"` for its USART writes the same word here. The manual this
//! half was written from is ST **RM0351** (STM32L4x5/L4x6), chapter **42**,
//! *Serial peripheral interface (SPI)*; the block is the same one in the other
//! families named above. The default is `"f4"`, because
//! [`machines/spi-flash.machine`] and [`machines/spi-panel.machine`] are F4
//! boards and a default that matches the boards in the tree is one fewer line
//! in each of them.
//!
//! # Why that is a property and not a second class
//!
//! [`i2c`](super::i2c) refuses the same move, and the difference is the point.
//! I²C v1 and v2 share a *name*: different registers at different offsets, and
//! — the part that decides it — a different **transfer engine**, since v2's
//! `CR2` carries `NBYTES`/`AUTOEND` and the hardware runs the transfer that v1
//! drives byte by byte from software. Nothing could serve both.
//!
//! Here one engine serves both. `Shared::advance_to`, `Shared::begin`,
//! `Shared::finish`, `Shared::tx_bit`, `Shared::capture`,
//! `Shared::check_mode_fault` and the whole slave face are written once and
//! are correct for both generations; what forks is how a frame is *fetched*
//! and *deposited*, which is a dozen lines at each end. A second class would
//! have been a second copy of the engine, kept in step by hand — the exact
//! drift [`crate::bus::spi`] exists to prevent between its own two links.
//!
//! **Neither half silently half-works as the other**, which is the thing a
//! `variant` has to get right, and it gets it right by being faithful rather
//! than by refusing: an F4-targeted driver that sets `CR1.DFF` on an `"f7"`
//! instance sets `CRCL` and keeps eight-bit frames, and an L4-targeted driver
//! that programs `CR2.DS` and `FRXTH` on an `"f4"` instance writes reserved
//! bits that read back zero and never sees `RXNE` per byte. That is what the
//! silicon does to each of them.
//!
//! The **H7's SPI is a third IP, not a further revision** (RM0433): its
//! configuration is split across `CFG1`/`CFG2`, `CR2` holds a `TSIZE` transfer
//! counter, `CR1` a `CSTART` bit, the data register is split into `TXDR` and
//! `RXDR`, the flags are `TXP`/`RXP`/`EOT`, and the frame size runs to 32 bits.
//! Nothing in the `0x00`-`0x20` map above survives that. A board with an H7
//! gets a second module rather than a third value of this property.
//!
//! # Register map (RM0090 §28.5.10 Table 130; RM0351 §42.6.10)
//!
//! | Offset | Name | Reset (`f4`) | Reset (`f7`) | Notes |
//! | --- | --- | --- | --- | --- |
//! | `0x00` | `CR1` | `0x0000` | `0x0000` | mode, framing, baud rate, `SPE` |
//! | `0x04` | `CR2` | `0x0000` | `0x0700` | `SSOE`, DMA and interrupt enables, `DS` |
//! | `0x08` | `SR` | `0x0002` | `0x0002` | `TXE` is set out of reset |
//! | `0x0c` | `DR` | `0x0000` | `0x0000` | **two buffers**: a write loads Tx, a read pops Rx |
//! | `0x10` | `CRCPR` | `0x0007` | `0x0007` | the CRC polynomial |
//! | `0x14` | `RXCRCR` | `0x0000` | `0x0000` | read-only |
//! | `0x18` | `TXCRCR` | `0x0000` | `0x0000` | read-only |
//! | `0x1c` | `I2SCFGR` | `0x0000` | `0x0000` | see below |
//! | `0x20` | `I2SPR` | `0x0002` | `0x0002` | see below |
//!
//! Every register is sixteen bits in a thirty-two bit slot; §28.5 says
//! accesses are by half-word or word. Byte access is **not defined by the
//! manual at all** for the F4 — ST's own headers do it to `DR` in 8-bit frame
//! format, so it is accepted here and reaches the low half, and the module says
//! so rather than pretending the manual answered. On the `"f7"` half byte
//! access to `DR` is not a liberty but the *specified* way to move one byte:
//! RM0351 §42.4.9 makes the access width part of the semantics.
//!
//! # The FIFO, and why the access width is the interesting part
//!
//! RM0351 §42.4.9: the `"f7"` block has a 32-bit — that is, **four-byte** —
//! FIFO in each direction, and how many frames a `DR` access moves depends on
//! how wide the access is:
//!
//! * With `DS ≤ 8` a frame is one byte. An 8-bit write to `DR` pushes **one**
//!   byte, so one frame; a 16-bit write pushes **two**, so two frames, low byte
//!   first. That is ST's "data packing", and it is why a driver written for
//!   this part uses `*(volatile uint8_t *)&SPI->DR` and a driver written for
//!   the F4 does not care.
//! * With `DS > 8` a frame is two bytes, little-endian, right-aligned in `DR`.
//! * `CR2.FRXTH` decides when `RXNE` rises: set, at one byte in the Rx FIFO;
//!   clear, at two. A driver that leaves it clear and then reads single bytes
//!   waits for ever, which is the bug this bit exists to cause.
//! * `SR.FRLVL`/`SR.FTLVL` report the fill in quarters. The manual gives four
//!   codes for a FIFO with five occupancies, so *three* bytes and *four* bytes
//!   both read `11`; this model saturates, and says so here because a driver
//!   that waits for `FTLVL == 00` is waiting on the one code that is exact.
//! * `TXE` is "at or below half" — two bytes — not "empty", and `BSY` is "a
//!   frame is shifting **or** the Tx FIFO is not empty".
//! * A frame that completes with no room in the Rx FIFO sets `OVR` and is
//!   lost, cleared by the read-`DR`-then-read-`SR` sequence exactly as on the
//!   F4.
//!
//! ## `DS`, and what a non-byte-multiple frame does
//!
//! `CR2.DS[3:0]` is `frame bits - 1`, so `0b0011` is four bits and `0b1111` is
//! sixteen. RM0351 §42.6.2: values below `0b0011` are "not used" and the
//! hardware **forces `0b0111`, eight bits** — so a driver that writes zero gets
//! a working eight-bit peripheral rather than a wedged one, and this model
//! forces the same value on the way in so a read-back tells the truth.
//!
//! A frame is **right-aligned in `DR`** whatever its width, and it moves as a
//! *whole frame* on the wire: `DS = 5` clocks five bits, not eight and not a
//! rounded-up byte. It still occupies one FIFO byte, because the FIFO is a byte
//! FIFO; five-bit frames therefore pack four to a full FIFO, and the top three
//! bits of each byte are not on the wire at all.
//!
//! ## `CRCL`, and a CRC wider than a frame
//!
//! On the `"f7"` half `CR1` bit 11 is `CRCL`, the **CRC length**: clear is an
//! 8-bit CRC, set is a 16-bit one (§42.6.1). The calculators are fed `DS`-bit
//! data at that width, which is the whole of "CRC over the programmable size".
//! When the CRC is wider than a frame — `CRCL` set with `DS = 8`, the ordinary
//! case — it goes out as `ceil(CRC bits / DS)` frames, most significant first,
//! and the received CRC is reassembled from the same number before it is
//! compared. On the `"f4"` half the CRC length *is* the frame length, so that
//! reduces to the single frame it always was.
//!
//! ## `LDMA_TX` and `LDMA_RX`, which exist for an odd count
//!
//! §42.4.9 again. With packing on, a DMA moves two bytes per access, so an
//! **odd** number of data does not divide into accesses: the last write has a
//! byte too many and the last read finds a byte too few. The two bits tell the
//! peripheral the count is odd.
//!
//! * `LDMA_TX` — a 16-bit write to `DR` pushes its low byte and **holds** its
//!   high byte back; the next write promotes the held byte ahead of its own.
//!   Five bytes written as three 16-bit accesses therefore put exactly five
//!   frames on the wire and the sixth byte is never sent — it is dropped when
//!   the stream ends (`SPE` or `TXDMAEN` clears). The model is one byte deeper
//!   in the pipeline than the silicon, which counts DMA requests and so knows
//!   which access is the last; a register block cannot know that, and holding
//!   the byte is the only way to get the *frame count* right, which is what the
//!   bit is for. The disclosed cost: the held byte occupies a FIFO entry, so
//!   `FTLVL` and `BSY` stand until the stream is ended — which is what a
//!   driver does anyway, in the DMA-complete callback that clears `TXDMAEN`.
//! * `LDMA_RX` — the `RXNE` threshold falls to one byte **once the stream has
//!   drained**: no frame in flight and the Tx FIFO empty. Then and only then a
//!   lone byte can be the odd last one, so `RXNE` rises for it even with
//!   `FRXTH` clear, and the 16-bit read that follows pops the one byte and
//!   answers with it in the low half. Without this the DMA stalls on the last
//!   byte of an odd count, which is the stall the bit exists to prevent.
//!
//! Both are gated on their direction's `DMAEN`, as §42.6.2 says ("it has
//! significance only if the TXDMAEN bit is set").
//!
//! # The parts real drivers trip over, and which are modelled
//!
//! * **`SSM`/`SSI`/`SSOE`, and `MODF`.** §28.3.1: with `SSM` set "the slave
//!   select information is driven internally by the value of the `SSI` bit …
//!   the external NSS pin remains free"; with `SSM` clear and `SSOE` set, NSS
//!   "is driven low when the master starts the communication and is kept low
//!   until the SPI is disabled". A master that sees its NSS low takes a **mode
//!   fault** (§28.3.10): `MODF` sets, `SPE` clears, `MSTR` clears — the
//!   peripheral demotes itself to a slave — and *"hardware does not allow the
//!   setting of the `SPE` and `MSTR` bits while the `MODF` bit is set"*. That
//!   last clause is the one that turns a driver bug into a peripheral that
//!   will not start, and it is modelled: a write of `SPE` while `MODF` stands
//!   is dropped. Clearing takes the manual's two steps — an access to `SR`,
//!   then a write to `CR1`.
//! * **`OVR` is not cleared by clearing it.** §28.3.10: "clearing the `OVR`
//!   bit is done by a read from the `SPI_DR` register followed by a read
//!   access to the `SPI_SR` register", and until then the receive buffer is
//!   *frozen* — every further frame is dropped rather than overwriting it.
//! * **`DR` is two registers.** A write goes to the Tx side, a read comes from
//!   the Rx side, and in 8-bit frame format §28.5.4 says the top half of a read
//!   is forced to zero.
//! * **Receive-only masters clock themselves.** §28.3.4: with `RXONLY` set (or
//!   `BIDIMODE` set and `BIDIOE` clear) a master "communication starts
//!   immediately and stops when the `SPE` bit is cleared" — no `DR` write is
//!   needed and none is expected, which is how a driver reads a flash without
//!   writing dummy bytes.
//!
//! # Both link models, as the fabric demands
//!
//! `link` is required and has no default, exactly as
//! [`crate::bus::spi::controller`]'s is and for the reason
//! `docs/buses/low-speed.md` gives. A frame costs `bits × 2^(BR+1)` ticks of
//! this peripheral's clock domain either way — §28.5.1's baud-rate divisor is
//! `2^(BR+1)`, so one bit is one `SCK` period is `2^(BR+1)` ticks of `PCLK` —
//! so a driver polling `BSY` sees the same timing under both. What differs is
//! only whether the edges exist. `DS` is visible in that number: a five-bit
//! frame costs five bit times, not eight.
//!
//! # Slave mode
//!
//! With `MSTR` clear the peripheral generates no clock and starts nothing; it
//! *answers*. That half is reached through the fabric's own
//! [`SlavePins`] on the `sck-in`, `mosi-in`,
//! `nss-in` and `miso-out` pins, so another controller — or a guest bit-banging
//! GPIO — clocks it and `DR`, `TXE`, `RXNE`, `OVR` and `BSY` move exactly as
//! they would in master mode. On the `"f7"` half the FIFOs are in that path
//! too: a slave answers from its Tx FIFO and deposits into its Rx FIFO.
//!
//! **NSS is split into two pins**, `nss` (out) and `nss-in` (in). The real part
//! has one bidirectional pin; an rsemu wire has fixed drivers and cannot be
//! tri-stated, which is the same split
//! [`crate::dev::sitronix`] makes for the ST7272A's `SDA`.
//!
//! # What is not modelled, and says so
//!
//! `I2SCFGR` and `I2SPR` are stored and read back — a driver that probes them
//! must see its own writes — but **I²S itself is not implemented**: setting
//! `I2SMOD` does not turn this into an audio interface, and `CHSIDE` and `UDR`
//! never set. Modelling half an I²S peripheral would be worse than modelling
//! none, and a machine that wants one should say so. The `FRF` (TI frame
//! format) bit is likewise stored; `FRE` never sets, because TI framing
//! changes where NSS pulses and nothing in this tree watches for that.
//!
//! **`CR2.NSSP` is stored and does not pulse.** §42.4.5 restricts it to a
//! master with `CPHA = 0`, and what it does is drop NSS for one clock period
//! *between* frames — which every part in this tree would read as the end of a
//! command, since a serial flash commits on the rising edge of its chip select
//! (`crate::dev::flash::spinor`). A driver sets `NSSP` for a
//! one-frame-per-select protocol and there is no such part here yet; a pulse
//! implemented against parts that would mis-commit on it would be a worse model
//! than an honest register, so it is an honest register.
//!
//! No emulator source of any licence was consulted (`ROADMAP.md` §1).
//!
//! [`machines/spi-flash.machine`]: https://github.com/KarpelesLab/rsemu/blob/master/machines/spi-flash.machine
//! [`machines/spi-panel.machine`]: https://github.com/KarpelesLab/rsemu/blob/master/machines/spi-panel.machine

use alloc::boxed::Box;
use alloc::string::{String, ToString};
use alloc::sync::Arc;
use alloc::vec::Vec;
use core::fmt;

use crate::bus::spi::{
    BitOrder, ChipSelect, Format, Link, MAX_CHIP_SELECTS, Mode, SlavePins, SpiBus, SpiSlave, buses,
    pin as slave_pin,
};
use crate::core::device::{Device, DeviceClass, PropertySpec, RealizeCtx, ResetKind, SinkPin};
use crate::core::error::{BusError, Error, Result};
use crate::core::props::{Props, ValueKind};
use crate::core::sched::{AccessKind, LazyHandle};
use crate::core::space::{AccessConstraints, MemAttrs, MemOps, MemResult, Region, RegionRef};
use crate::core::state::{ChunkReader, ChunkWriter, Sink, Source};
use crate::core::sync::{AtomicBool, AtomicU64, LockRank, Mutex, Ordering};
use crate::core::value::{Endian, Width};
use crate::core::wire::{Level, WireId, WireSink, WireSource};
use crate::machine::realize::Instance;
use crate::machine::validate::{ClassSchema, PortDir, PropSchema};

/// The class name a machine description writes.
pub const CLASS_NAME: &str = "stm32.spi";

/// The snapshot chunk version. Bump with the encoding, never on its own.
///
/// Two since the `"f7"` variant landed: the chunk carries both FIFOs, the
/// `LDMA_TX` held byte and the CRC-transfer position, and a version-1 chunk has
/// none of them. Pre-1.0 and no migration, because no version-1 snapshot of
/// this class outlives the release it was written by.
const STATE_VERSION: u32 = 2;

/// How much address space the peripheral occupies.
///
/// The registers stop at `0x23`; the STM32 bus gives every peripheral a 1 KiB
/// slot (RM0090 Table 1), and reads above the last register answer zero.
pub const REGISTER_BYTES: u64 = 0x400;

/// The last defined register offset, past which reads answer zero.
const LAST_REGISTER: u64 = 0x20;

// -- CR1 (§28.5.1) ----------------------------------------------------------

/// Clock phase.
const CR1_CPHA: u16 = 1 << 0;
/// Clock polarity.
const CR1_CPOL: u16 = 1 << 1;
/// Master configuration.
const CR1_MSTR: u16 = 1 << 2;
/// Baud-rate control, bits 5:3. The divisor is `2^(BR + 1)`.
const CR1_BR_SHIFT: u32 = 3;
/// And its mask, once shifted down.
const CR1_BR_MASK: u16 = 0x7;
/// SPI enable.
const CR1_SPE: u16 = 1 << 6;
/// Least significant bit first.
const CR1_LSBFIRST: u16 = 1 << 7;
/// Internal slave select — the value `SSM` substitutes for the NSS pin.
const CR1_SSI: u16 = 1 << 8;
/// Software slave management.
const CR1_SSM: u16 = 1 << 9;
/// Receive only.
const CR1_RXONLY: u16 = 1 << 10;
/// Data frame format: set is sixteen bits, clear is eight. `"f4"` only.
const CR1_DFF: u16 = 1 << 11;
/// The same bit on the `"f7"` block, where it is the CRC *length*: set is a
/// sixteen-bit CRC, clear an eight-bit one (RM0351 §42.6.1). The frame size
/// moved to `CR2.DS`, so bit 11 was free to mean something else — which is why
/// an F4 driver that writes `DFF` here gets a wider CRC and the same eight-bit
/// frames it started with.
const CR1_CRCL: u16 = 1 << 11;
/// The next transfer carries the CRC.
const CR1_CRCNEXT: u16 = 1 << 12;
/// Hardware CRC calculation enable.
const CR1_CRCEN: u16 = 1 << 13;
/// Output enable in bidirectional mode.
const CR1_BIDIOE: u16 = 1 << 14;
/// Bidirectional data mode: one data wire instead of two.
const CR1_BIDIMODE: u16 = 1 << 15;

// -- CR2 (§28.5.2) ----------------------------------------------------------

/// Rx buffer DMA enable.
const CR2_RXDMAEN: u16 = 1 << 0;
/// Tx buffer DMA enable.
const CR2_TXDMAEN: u16 = 1 << 1;
/// SS output enable.
const CR2_SSOE: u16 = 1 << 2;
/// Frame format: set selects TI mode.
const CR2_FRF: u16 = 1 << 4;
/// Error interrupt enable.
const CR2_ERRIE: u16 = 1 << 5;
/// Rx buffer not empty interrupt enable.
const CR2_RXNEIE: u16 = 1 << 6;
/// Tx buffer empty interrupt enable.
const CR2_TXEIE: u16 = 1 << 7;
/// Everything the `"f4"` `CR2` defines. Bit 3 is forced to zero by hardware
/// (§28.5.2).
const CR2_MASK_F4: u16 =
    CR2_RXDMAEN | CR2_TXDMAEN | CR2_SSOE | CR2_FRF | CR2_ERRIE | CR2_RXNEIE | CR2_TXEIE;

// -- CR2, the `"f7"` additions (RM0351 §42.6.2) ------------------------------

/// NSS pulse management: pulse NSS between frames. Stored, not acted on — see
/// the module docs.
const CR2_NSSP: u16 = 1 << 3;
/// Data size, bits 11:8. The field holds `bits - 1`.
const CR2_DS_SHIFT: u32 = 8;
/// And its mask, once shifted down.
const CR2_DS_MASK: u16 = 0xf;
/// The code `DS` holds for eight bits, which is what the hardware forces when
/// software writes one of the three "not used" values below it.
const DS_CODE_EIGHT: u16 = 0b0111;
/// The smallest `DS` code the manual defines: four-bit frames.
const DS_CODE_MIN: u16 = 0b0011;
/// FIFO reception threshold: set, `RXNE` rises at one byte; clear, at two.
const CR2_FRXTH: u16 = 1 << 12;
/// The number of data to receive by DMA is odd.
const CR2_LDMA_RX: u16 = 1 << 13;
/// The number of data to transmit by DMA is odd.
const CR2_LDMA_TX: u16 = 1 << 14;
/// Everything the `"f7"` `CR2` defines. Bit 15 is reserved.
const CR2_MASK_FIFO: u16 =
    CR2_MASK_F4 | CR2_NSSP | (CR2_DS_MASK << CR2_DS_SHIFT) | CR2_FRXTH | CR2_LDMA_RX | CR2_LDMA_TX;

/// What `CR2` powers up with on the `"f7"` block: `DS` at eight bits, which is
/// the whole of RM0351 §42.6.2's `0x0700` reset value.
const CR2_RESET_FIFO: u16 = DS_CODE_EIGHT << CR2_DS_SHIFT;

// -- SR (§28.5.3) -----------------------------------------------------------

/// Receive buffer not empty.
const SR_RXNE: u16 = 1 << 0;
/// Transmit buffer empty.
const SR_TXE: u16 = 1 << 1;
/// Underrun. I²S only; never set here.
const SR_UDR: u16 = 1 << 3;
/// The received CRC did not match.
const SR_CRCERR: u16 = 1 << 4;
/// Master mode fault.
const SR_MODF: u16 = 1 << 5;
/// Overrun: a frame arrived with the receive buffer still full.
const SR_OVR: u16 = 1 << 6;
/// Busy: a transfer is in flight.
const SR_BSY: u16 = 1 << 7;
/// TI-mode frame error. Never set here; see the module docs.
const SR_FRE: u16 = 1 << 8;
/// FIFO reception level, bits 10:9. `"f7"` only (RM0351 §42.6.3).
const SR_FRLVL_SHIFT: u32 = 9;
/// FIFO transmission level, bits 12:11.
const SR_FTLVL_SHIFT: u32 = 11;
/// The two-bit field both levels are.
const SR_LVL_MASK: u16 = 0x3;
/// Both level fields at once, for clearing them before they are recomputed.
const SR_LEVELS: u16 = (SR_LVL_MASK << SR_FRLVL_SHIFT) | (SR_LVL_MASK << SR_FTLVL_SHIFT);

/// What `SR` reads as out of reset: the transmit buffer is empty.
const SR_RESET: u16 = SR_TXE;

/// The polynomial `CRCPR` powers up with (§28.5.5).
const CRCPR_RESET: u16 = 0x0007;

/// How deep each `"f7"` FIFO is, in bytes.
///
/// RM0351 §42.4.9 calls it a 32-bit FIFO, so four bytes — which is four frames
/// at `DS ≤ 8` and two at `DS > 8`, because the FIFO is a byte FIFO and a wide
/// frame takes two of them.
const FIFO_BYTES: usize = 4;

/// What `I2SPR` powers up with (§28.5.9).
const I2SPR_RESET: u16 = 0x0002;

/// "Nothing scheduled", as [`Shared::next_event`] spells it.
const NO_EVENT: u64 = u64::MAX;

/// The pin names a machine description wires.
pub mod pin {
    /// The serial clock this peripheral drives as a master.
    pub const SCK: &str = "sck";
    /// Data out to the slaves, as a master.
    pub const MOSI: &str = "mosi";
    /// Data in from the slaves, as a master.
    pub const MISO: &str = "miso";
    /// The slave-select output, as a master with `SSOE` set. Active low.
    pub const NSS: &str = "nss";
    /// The slave-select input: what a master watches for a mode fault, and
    /// what selects this peripheral when it is the slave. Active low.
    pub const NSS_IN: &str = "nss-in";
    /// The clock input, as a slave.
    pub const SCK_IN: &str = "sck-in";
    /// The data input, as a slave.
    pub const MOSI_IN: &str = "mosi-in";
    /// The data output, as a slave.
    pub const MISO_OUT: &str = "miso-out";

    /// The wire line the MISO input answers on.
    pub const MISO_LINE: u32 = 0;
    /// The wire line the NSS input answers on.
    pub const NSS_IN_LINE: u32 = 1;
}

// ---------------------------------------------------------------------------
// the variant
// ---------------------------------------------------------------------------

/// Which generation of the block an instance is.
///
/// The module docs say why this is a property rather than a second class, and
/// what each value costs a driver written for the other.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Variant {
    /// RM0090 §28: one word each way, `CR1.DFF` picks eight or sixteen bits.
    /// F1, F2, F4, L1.
    F4,
    /// RM0351 §42: a four-byte FIFO each way and a four-to-sixteen-bit
    /// `CR2.DS`. F0, F3, F7, L4, L4+, L5, G0, G4, WB.
    Fifo,
}

impl Variant {
    /// The name a machine description writes.
    #[must_use]
    pub const fn name(self) -> &'static str {
        match self {
            Variant::F4 => "f4",
            Variant::Fifo => "f7",
        }
    }

    /// Every name [`Variant::from_name`] answers to, for the validator.
    pub const NAMES: &'static [&'static str] = &["f4", "f7"];

    /// The variant `name` refers to, if it is one.
    #[must_use]
    pub fn from_name(name: &str) -> Option<Variant> {
        match name {
            "f4" => Some(Variant::F4),
            "f7" => Some(Variant::Fifo),
            _ => None,
        }
    }

    /// Whether this generation has the FIFOs.
    const fn has_fifo(self) -> bool {
        matches!(self, Variant::Fifo)
    }

    /// The bits `CR2` keeps out of a write.
    const fn cr2_mask(self) -> u16 {
        match self {
            Variant::F4 => CR2_MASK_F4,
            Variant::Fifo => CR2_MASK_FIFO,
        }
    }

    /// What `CR2` powers up with.
    const fn cr2_reset(self) -> u16 {
        match self {
            Variant::F4 => 0,
            Variant::Fifo => CR2_RESET_FIFO,
        }
    }
}

// ---------------------------------------------------------------------------
// the FIFO
// ---------------------------------------------------------------------------

/// One of the `"f7"` block's four-byte FIFOs (RM0351 §42.4.9).
///
/// A **byte** FIFO, because that is what the silicon has: a frame of eight bits
/// or fewer occupies one entry and a wider one occupies two, which is what
/// makes `FRLVL`/`FTLVL` count in bytes and what makes the access width of a
/// `DR` access decide how many frames it moves.
///
/// Four bytes shift rather than wrap. A ring would need a head index in the
/// snapshot for no gain at this depth, and a straight array compares equal
/// only when the *contents* are equal, which is what the round-trip test wants.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
struct Fifo {
    bytes: [u8; FIFO_BYTES],
    len: u8,
}

impl Fifo {
    /// How many bytes are in it.
    const fn len(&self) -> usize {
        self.len as usize
    }

    /// How many more bytes it can take.
    const fn free(&self) -> usize {
        FIFO_BYTES - self.len as usize
    }

    /// Append `byte`, or report that there was no room.
    ///
    /// RM0351 §42.4.9: a write to a full Tx FIFO is ignored, and a frame that
    /// completes with a full Rx FIFO sets `OVR` — so neither direction wants a
    /// panic here, and both want to know.
    fn push(&mut self, byte: u8) -> bool {
        if self.len() == FIFO_BYTES {
            return false;
        }
        self.bytes[self.len()] = byte;
        self.len += 1;
        true
    }

    /// Take the oldest byte.
    fn pop(&mut self) -> Option<u8> {
        if self.len == 0 {
            return None;
        }
        let byte = self.bytes[0];
        self.bytes.copy_within(1.., 0);
        self.len -= 1;
        self.bytes[self.len()] = 0;
        Some(byte)
    }

    /// The byte `n` places along, without taking it. The debug path.
    const fn at(&self, n: usize) -> Option<u8> {
        if n >= self.len as usize {
            None
        } else {
            Some(self.bytes[n])
        }
    }

    /// Drop everything.
    fn clear(&mut self) {
        *self = Fifo::default();
    }

    /// The two-bit level code `FRLVL`/`FTLVL` report for `bytes` occupancy.
    ///
    /// RM0351 §42.6.3 gives four codes — empty, quarter, half, full — for a
    /// four-byte FIFO, which has five occupancies. Three bytes has no code of
    /// its own, so it saturates into `11` along with four; the module docs say
    /// so, because `00` is the only code a driver can treat as exact.
    const fn level(bytes: usize) -> u16 {
        if bytes >= 3 { 3 } else { bytes as u16 }
    }
}

// ---------------------------------------------------------------------------
// state
// ---------------------------------------------------------------------------

/// Everything the guest can see or change.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct State {
    /// Which generation this instance is. Construction-time and never moves,
    /// but it lives here because every decision in the engine consults it.
    variant: Variant,
    /// Domain ticks simulated. The authoritative copy; the atomic mirrors it.
    ticks: u64,
    cr1: u16,
    cr2: u16,
    sr: u16,
    /// The Tx buffer. A `DR` write lands here.
    tx: u16,
    /// The Rx buffer. A `DR` read comes from here.
    rx: u16,
    crcpr: u16,
    rxcrc: u16,
    txcrc: u16,
    i2scfgr: u16,
    i2spr: u16,
    /// Whether the Tx buffer holds a word nothing has shifted yet.
    tx_pending: bool,
    /// The word in the shift register, going out.
    shift: u16,
    /// Whether that shift register is running.
    busy: bool,
    /// Whether the frame in flight is the CRC rather than data.
    crc_frame: bool,
    /// The tick the in-flight frame began on.
    started: u64,
    /// Edges emitted so far, in [`Link::Wired`].
    edges: u32,
    /// Bits captured from MISO so far, in [`Link::Wired`].
    shift_in: u32,
    /// Whether the NSS output is being held low.
    nss_low: bool,
    /// The level the NSS input is at. Reset does not move it — it belongs to
    /// whatever drives it (`ROADMAP.md` §4.5).
    nss_in: Level,
    /// Whether a `DR` read has happened since `OVR` set, which is the first
    /// half of §28.3.10's clearing sequence.
    ovr_dr_read: bool,
    /// Whether `SR` has been accessed since `MODF` set, which is the first
    /// half of §28.3.10's other clearing sequence.
    modf_sr_seen: bool,

    // -- `"f7"` only ---------------------------------------------------------
    /// The transmit FIFO. Unused, and always empty, on an `"f4"`.
    tx_fifo: Fifo,
    /// The receive FIFO.
    rx_fifo: Fifo,
    /// The high byte of the last packed `DR` write, held back by `LDMA_TX`
    /// until another write promotes it. See the module docs.
    tx_held: Option<u8>,
    /// How many frames of the CRC transfer are still to go, or zero.
    ///
    /// One for every `"f4"` CRC and for a `"f7"` CRC that fits in a frame;
    /// more when `CRCL` is wider than `DS`, which is the ordinary
    /// sixteen-bit-CRC-over-eight-bit-data case.
    crc_left: u8,
    /// The CRC being reassembled from those frames as they arrive.
    crc_in: u16,
}

impl State {
    /// A freshly reset peripheral of `variant`.
    fn new(variant: Variant) -> State {
        State {
            variant,
            ticks: 0,
            cr1: 0,
            cr2: variant.cr2_reset(),
            sr: SR_RESET,
            tx: 0,
            rx: 0,
            crcpr: CRCPR_RESET,
            rxcrc: 0,
            txcrc: 0,
            i2scfgr: 0,
            i2spr: I2SPR_RESET,
            tx_pending: false,
            shift: 0,
            busy: false,
            crc_frame: false,
            started: 0,
            edges: 0,
            shift_in: 0,
            nss_low: false,
            // A pin nothing drives sits at its inactive level rather than the
            // low a fresh net idles at: a master on a board that never wires
            // NSS must not take a mode fault for the machine file's silence.
            nss_in: Level::High,
            ovr_dr_read: false,
            modf_sr_seen: false,
            tx_fifo: Fifo::default(),
            rx_fifo: Fifo::default(),
            tx_held: None,
            crc_left: 0,
            crc_in: 0,
        }
    }

    /// How many bits a frame carries.
    ///
    /// `CR1.DFF` on an `"f4"` (§28.5.1); `CR2.DS[3:0] + 1` on an `"f7"`, where
    /// RM0351 §42.6.2 forces the three codes below `0b0011` to eight bits. The
    /// forcing also happens on the way *in*, so a read-back of `CR2` agrees
    /// with this — it is here as well because `CR2` can be restored from a
    /// snapshot written by a version that did not force it.
    fn frame_bits(&self) -> u8 {
        match self.variant {
            Variant::F4 => {
                if self.cr1 & CR1_DFF != 0 {
                    16
                } else {
                    8
                }
            }
            Variant::Fifo => {
                let code = (self.cr2 >> CR2_DS_SHIFT) & CR2_DS_MASK;
                let code = if code < DS_CODE_MIN {
                    DS_CODE_EIGHT
                } else {
                    code
                };
                (code + 1) as u8
            }
        }
    }

    /// How many FIFO bytes one frame occupies: one, or two above eight bits.
    fn frame_bytes(&self) -> usize {
        if self.frame_bits() > 8 { 2 } else { 1 }
    }

    /// How wide the CRC calculators are.
    ///
    /// The frame width on an `"f4"`, where §28.3.6 ties the two together
    /// ("CRC8 for 8-bit data, CRC16 for 16-bit data"); `CR1.CRCL` on an
    /// `"f7"`, where RM0351 §42.6.1 unties them.
    fn crc_bits(&self) -> u8 {
        match self.variant {
            Variant::F4 => self.frame_bits(),
            Variant::Fifo => {
                if self.cr1 & CR1_CRCL != 0 {
                    16
                } else {
                    8
                }
            }
        }
    }

    /// How many frames the CRC takes to send, most significant first.
    fn crc_frames(&self) -> u8 {
        self.crc_bits().div_ceil(self.frame_bits()).max(1)
    }

    /// The framing `CR1` and `CR2` describe.
    fn format(&self) -> Format {
        Format::new(
            Mode::from_cpol_cpha(self.cr1 & CR1_CPOL != 0, self.cr1 & CR1_CPHA != 0),
            self.frame_bits(),
            if self.cr1 & CR1_LSBFIRST != 0 {
                BitOrder::LsbFirst
            } else {
                BitOrder::MsbFirst
            },
        )
    }

    /// Half of one `SCK` period, in domain ticks.
    ///
    /// §28.5.1's divisor is `2^(BR + 1)` of `PCLK`, so a whole bit is that
    /// many ticks and a half period is `2^BR` — never zero, which the
    /// scheduler requires or catch-up stops making progress.
    fn half_period(&self) -> u64 {
        1u64 << ((self.cr1 >> CR1_BR_SHIFT) & CR1_BR_MASK)
    }

    /// How many wire edges one frame takes: two per bit.
    fn total_edges(&self) -> u32 {
        u32::from(self.format().bits) * 2
    }

    /// The tick the in-flight frame completes on.
    fn end_tick(&self) -> u64 {
        self.started
            .saturating_add(u64::from(self.total_edges()) * self.half_period())
    }

    fn is_master(&self) -> bool {
        self.cr1 & CR1_MSTR != 0
    }

    fn is_enabled(&self) -> bool {
        self.cr1 & CR1_SPE != 0
    }

    /// Whether the peripheral drives no data line: `RXONLY`, or bidirectional
    /// with the output disabled (§28.3.4).
    fn receive_only(&self) -> bool {
        self.cr1 & CR1_RXONLY != 0 || (self.cr1 & CR1_BIDIMODE != 0 && self.cr1 & CR1_BIDIOE == 0)
    }

    /// Whether the peripheral keeps no received word: bidirectional with the
    /// output enabled (§28.3.4).
    fn transmit_only(&self) -> bool {
        self.cr1 & CR1_BIDIMODE != 0 && self.cr1 & CR1_BIDIOE != 0
    }

    /// The level the NSS pin sees, whichever way `SSM` says to look (§28.3.1).
    fn nss_level(&self) -> Level {
        if self.cr1 & CR1_SSM != 0 {
            Level::from_bool(self.cr1 & CR1_SSI != 0)
        } else {
            self.nss_in
        }
    }

    /// Whether a master should be taking a mode fault right now (§28.3.10).
    ///
    /// Only when the peripheral is not itself driving NSS: with `SSOE` set the
    /// output is its own, and a master cannot fault on the level it drives.
    fn mode_fault_due(&self) -> bool {
        self.is_master()
            && !(self.cr1 & CR1_SSM == 0 && self.cr2 & CR2_SSOE != 0)
            && self.nss_level().is_low()
    }

    /// Whether the NSS output should be low: §28.3.1's "driven low when the
    /// master starts the communication and kept low until the SPI is
    /// disabled".
    fn nss_output_low(&self) -> bool {
        self.is_master() && self.is_enabled() && self.cr1 & CR1_SSM == 0 && self.cr2 & CR2_SSOE != 0
    }

    // -- the `"f7"` data path ------------------------------------------------

    /// How many bytes are queued to go out, the held one included.
    ///
    /// `LDMA_TX`'s held byte is *in* the FIFO as far as the guest can tell —
    /// it occupies an entry and it is going to be sent — so it counts towards
    /// `FTLVL` and towards `TXE`. What it does not do is start a frame.
    fn tx_total(&self) -> usize {
        self.tx_fifo.len() + usize::from(self.tx_held.is_some())
    }

    /// How many bytes must be in the Rx FIFO for `RXNE` to rise.
    ///
    /// `CR2.FRXTH` decides it (RM0351 §42.6.2), except at the very end of an
    /// odd-length DMA read: with `LDMA_RX` set and the stream drained — nothing
    /// in flight and nothing left to send — a lone byte can only be the odd
    /// last one, so it raises `RXNE` even with `FRXTH` clear. See the module
    /// docs for why that condition is the one a register block can know.
    fn rxne_threshold(&self) -> usize {
        if self.cr2 & (CR2_LDMA_RX | CR2_RXDMAEN) == (CR2_LDMA_RX | CR2_RXDMAEN)
            && !self.busy
            && self.tx_total() == 0
        {
            return 1;
        }
        if self.cr2 & CR2_FRXTH != 0 { 1 } else { 2 }
    }

    /// Recompute the flags the FIFO levels derive.
    ///
    /// A no-op on an `"f4"`, which has no levels and whose `TXE`, `RXNE` and
    /// `BSY` are set and cleared at the moments §28.3.7 names. On an `"f7"`
    /// all four are *functions of the fill*, so they are recomputed rather
    /// than edited, and every path that touches a FIFO ends here.
    fn refresh(&mut self) {
        if !self.variant.has_fifo() {
            return;
        }
        let tx = self.tx_total();
        let rx = self.rx_fifo.len();
        self.sr = (self.sr & !SR_LEVELS)
            | (Fifo::level(rx) << SR_FRLVL_SHIFT)
            | (Fifo::level(tx) << SR_FTLVL_SHIFT);
        // §42.4.9: `TXE` is "at or below half", not "empty".
        if tx <= FIFO_BYTES / 2 {
            self.sr |= SR_TXE;
        } else {
            self.sr &= !SR_TXE;
        }
        if rx > 0 && rx >= self.rxne_threshold() {
            self.sr |= SR_RXNE;
        } else {
            self.sr &= !SR_RXNE;
        }
        // "A frame is being shifted, or the Tx FIFO is not empty" — and a CRC
        // transfer still owing a frame counts, because a driver polling `BSY`
        // before dropping its chip select would otherwise cut the CRC in half.
        // A disabled peripheral is not busy whatever it is holding, which is
        // the same clause `SPE` falling already relies on.
        if self.busy || (self.is_enabled() && (tx > 0 || self.crc_left > 0)) {
            self.sr |= SR_BSY;
        } else {
            self.sr &= !SR_BSY;
        }
    }

    /// Take the next outgoing frame, right-aligned, or report there is none.
    ///
    /// One buffer on an `"f4"`; `frame_bytes()` little-endian bytes out of the
    /// Tx FIFO on an `"f7"`, so a nine-bit frame waits for its second byte
    /// rather than going out half-formed.
    fn pop_tx_frame(&mut self) -> Option<u16> {
        match self.variant {
            Variant::F4 => {
                if !self.tx_pending {
                    return None;
                }
                self.tx_pending = false;
                Some(self.tx)
            }
            Variant::Fifo => {
                let need = self.frame_bytes();
                if self.tx_fifo.len() < need {
                    return None;
                }
                let lo = u16::from(self.tx_fifo.pop().unwrap_or(0));
                let word = if need == 2 {
                    lo | (u16::from(self.tx_fifo.pop().unwrap_or(0)) << 8)
                } else {
                    lo
                };
                Some((u32::from(word) & self.format().mask()) as u16)
            }
        }
    }

    /// What [`State::pop_tx_frame`] would answer, without taking it.
    fn peek_tx_frame(&self) -> Option<u16> {
        match self.variant {
            Variant::F4 => self.tx_pending.then_some(self.tx),
            Variant::Fifo => {
                let need = self.frame_bytes();
                if self.tx_fifo.len() < need {
                    return None;
                }
                let lo = u16::from(self.tx_fifo.at(0)?);
                let word = if need == 2 {
                    lo | (u16::from(self.tx_fifo.at(1)?) << 8)
                } else {
                    lo
                };
                Some((u32::from(word) & self.format().mask()) as u16)
            }
        }
    }

    /// Queue `byte` to go out, dropping it if the FIFO is full.
    ///
    /// §42.4.9: a write to a full Tx FIFO is ignored. `TXE` is what stops a
    /// driver getting here, and one that writes anyway loses the byte exactly
    /// as it would on the part.
    fn push_tx_byte(&mut self, byte: u8) {
        let _ = self.tx_fifo.push(byte);
    }

    /// Whether `LDMA_TX`'s held byte applies to this write.
    ///
    /// §42.6.2 gates it on `TXDMAEN`, and packing only exists at `DS ≤ 8`.
    fn ldma_tx_active(&self) -> bool {
        self.variant.has_fifo()
            && self.cr2 & (CR2_LDMA_TX | CR2_TXDMAEN) == (CR2_LDMA_TX | CR2_TXDMAEN)
            && self.frame_bytes() == 1
    }

    /// Empty the transmit side, held byte included.
    ///
    /// What `SPE` falling does. §42.4.10's disable procedure says to wait for
    /// `FTLVL == 00` first; a guest that does not wait would otherwise leave
    /// `BSY` set for ever, since nothing clocks a disabled peripheral. The
    /// **receive** FIFO is deliberately left alone — the manual lets a driver
    /// read what already arrived after disabling.
    fn flush_tx(&mut self) {
        self.tx_fifo.clear();
        self.tx_held = None;
        self.tx_pending = false;
    }
}

/// One turn of the CRC, MSB first.
///
/// §28.3.6 says only that the calculator is "CRC8 for 8-bit data" and "CRC16
/// for 16-bit data" with the `CRCPR` polynomial; it does not write the
/// recurrence down, so this is the conventional non-reflected, zero-initial
/// form, and a guest checking a CRC against a peer that uses another
/// convention will disagree with this model exactly as it would with itself.
///
/// `data_bits` and `crc_bits` are separate because RM0351 §42.6.1's `CRCL`
/// unties them: an `"f7"` may run a sixteen-bit CRC over five-bit frames. On
/// an `"f4"` they are always equal and this is the function it always was.
fn crc_step(crc: u16, data: u16, data_bits: u8, crc_bits: u8, poly: u16) -> u16 {
    let width = u32::from(crc_bits);
    let mask: u32 = if width >= 32 {
        u32::MAX
    } else {
        (1u32 << width) - 1
    };
    let top = 1u32 << (width - 1);
    let mut acc = u32::from(crc) & mask;
    for i in (0..u32::from(data_bits)).rev() {
        let bit = (u32::from(data) >> i) & 1;
        let msb = acc & top != 0;
        acc = (acc << 1) & mask;
        if msb != (bit != 0) {
            acc ^= u32::from(poly) & mask;
        }
    }
    acc as u16
}

// ---------------------------------------------------------------------------
// the device
// ---------------------------------------------------------------------------

/// An STM32F4 SPI peripheral.
#[derive(Debug)]
pub struct Stm32Spi {
    shared: Arc<Shared>,
    pins: Arc<SlavePins>,
    region: RegionRef,
}

/// The wire outputs, all optional until a machine description connects them.
#[derive(Debug, Default)]
struct Pins {
    sck: Option<WireSource>,
    mosi: Option<WireSource>,
    nss: Option<WireSource>,
}

/// Everything both halves of the device reach.
struct Shared {
    state: Mutex<State>,
    /// Which generation this instance is. Construction-time, so `reset` can
    /// rebuild a [`State`] without consulting the one it is replacing.
    variant: Variant,
    /// How words reach the slaves, written down in the machine file.
    link: Link,
    /// The bus this peripheral drives as a master, transactionally.
    bus: Option<Arc<SpiBus>>,
    /// Which chip select on that bus the NSS output corresponds to.
    cs: ChipSelect,
    /// Domain ticks simulated, for the scheduler's lock-free question.
    ticks: AtomicU64,
    /// The tick the next edge or completion falls on, or [`NO_EVENT`].
    next_event: AtomicU64,
    /// The level last seen on the MISO input.
    miso: AtomicBool,
    /// The interrupt output level, published for a debug read.
    irq_level: AtomicBool,
    pins: Mutex<Pins>,
    /// The interrupt output, connected at realize time.
    irq: Mutex<Option<WireSource>>,
    /// Input pins handed out by [`Device::sink`], kept alive here because a net
    /// refers to its sinks weakly (`core::device`, §4.3's weak edge).
    sinks: Mutex<Vec<Arc<InputSink>>>,
    /// The catch-up handle the register block syncs through.
    lazy: Mutex<Option<LazyHandle>>,
}

impl fmt::Debug for Shared {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let mut s = f.debug_struct("Shared");
        s.field("variant", &self.variant)
            .field("link", &self.link)
            .field("cs", &self.cs);
        match self.state.try_lock() {
            Some(state) => s.field("state", &*state).finish(),
            None => s.field("state", &"<in use>").finish(),
        }
    }
}

impl Stm32Spi {
    /// Validate `props` and build the peripheral.
    ///
    /// Properties:
    ///
    /// * `variant` — `"f4"` (RM0090 §28, no FIFO) or `"f7"` (RM0351 §42, a
    ///   four-byte FIFO each way and a four-to-sixteen-bit `DS`). Defaults to
    ///   `"f4"`; the module docs say why this is a property and what a driver
    ///   written for one sees on the other.
    /// * `link` — `"transactional"` or `"wired"`. Required, and deliberately
    ///   so: `docs/buses/low-speed.md` asks for the choice to be made rather
    ///   than defaulted into.
    /// * `bus` — the named [`SpiBus`] this peripheral drives as a master.
    ///   Required for `transactional`.
    /// * `cs` — which chip select on that bus the NSS output corresponds to.
    ///
    /// # Errors
    ///
    /// [`Error::Property`] for an unknown or missing property,
    /// [`Error::Config`] for a `link` this module does not know or a `cs` out
    /// of range.
    pub fn new(props: &Props) -> Result<Stm32Spi> {
        let mut r = props.reader();
        let variant =
            Variant::from_name(r.or_enum("variant", "f4", Variant::NAMES)?).unwrap_or(Variant::F4);
        let link_name = r.require_str("link")?.to_string();
        let bus_name = r.optional_str("bus")?.map(String::from);
        let cs = r.or_range("cs", 0u64, 0..=(MAX_CHIP_SELECTS as u64 - 1))?;
        r.finish()?;

        let link = Link::from_name(&link_name).ok_or_else(|| Error::Config {
            at: String::from(CLASS_NAME),
            message: alloc::format!(
                "`link` is `{link_name}`; it must be one of {:?} — see docs/buses/low-speed.md \
                 for which to pick",
                Link::NAMES
            ),
        })?;
        if link == Link::Transactional && bus_name.is_none() {
            return Err(Error::Config {
                at: String::from(CLASS_NAME),
                message: String::from(
                    "a `transactional` SPI master reaches its slaves through a named bus; give \
                     it `bus = \"spi1\"` and name the same bus on each slave",
                ),
            });
        }
        let bus = bus_name
            .as_deref()
            .map(|name| buses::attach(props, name))
            .transpose()?;
        Ok(Stm32Spi::with_bus(variant, link, bus, ChipSelect(cs as u8)))
    }

    /// A peripheral on a bus the caller already holds.
    #[must_use]
    pub fn with_bus(
        variant: Variant,
        link: Link,
        bus: Option<Arc<SpiBus>>,
        cs: ChipSelect,
    ) -> Stm32Spi {
        let shared = Arc::new(Shared {
            state: Mutex::with_rank(LockRank::DEVICE, State::new(variant)),
            variant,
            link,
            bus,
            cs,
            ticks: AtomicU64::new(0),
            next_event: AtomicU64::new(NO_EVENT),
            miso: AtomicBool::new(true),
            irq_level: AtomicBool::new(false),
            pins: Mutex::with_rank(LockRank::WIRE, Pins::default()),
            irq: Mutex::with_rank(LockRank::WIRE, None),
            sinks: Mutex::with_rank(LockRank::WIRE, Vec::new()),
            lazy: Mutex::with_rank(LockRank::WIRE, None),
        });
        let pins = Arc::new(SlavePins::new(Arc::clone(&shared) as Arc<dyn SpiSlave>));
        let port = Arc::new(RegisterBlock {
            shared: Arc::clone(&shared),
            pins: Arc::clone(&pins),
        });
        let region = Arc::new(Region::io(
            "stm32-spi",
            REGISTER_BYTES,
            port as Arc<dyn MemOps>,
        ));
        Stm32Spi {
            shared,
            pins,
            region,
        }
    }

    /// Which generation of the block this is.
    #[must_use]
    pub fn variant(&self) -> Variant {
        self.shared.variant
    }

    /// How this peripheral carries a word.
    #[must_use]
    pub fn link(&self) -> Link {
        self.shared.link
    }

    /// The bus it drives as a master, if it has one.
    #[must_use]
    pub fn bus(&self) -> Option<&Arc<SpiBus>> {
        self.shared.bus.as_ref()
    }

    /// Its slave-side pins, for a controller that clocks it directly.
    #[must_use]
    pub fn pins(&self) -> &Arc<SlavePins> {
        &self.pins
    }

    /// Domain ticks simulated.
    #[must_use]
    pub fn ticks(&self) -> u64 {
        self.shared.ticks.load(Ordering::Relaxed)
    }

    /// `SR`, as software would read it — without the side effects a read has.
    #[must_use]
    pub fn status(&self) -> u16 {
        self.shared.state.lock().sr
    }

    /// The framing `CR1` currently describes.
    #[must_use]
    pub fn format(&self) -> Format {
        self.shared.state.lock().format()
    }

    /// Whether the interrupt output is asserted.
    #[must_use]
    pub fn irq_asserted(&self) -> bool {
        self.shared.irq_level.load(Ordering::Relaxed)
    }

    /// Run the peripheral until `target` domain ticks have passed in total.
    pub fn advance_to(&self, target: u64) {
        self.shared.advance_to(target);
    }
}

// ---------------------------------------------------------------------------
// the engine
// ---------------------------------------------------------------------------

/// One wire action the engine wants performed once the state lock is released.
#[derive(Debug, Clone, Copy)]
enum Emit {
    Sck(Level),
    Mosi(Level),
    Nss(Level),
}

impl Shared {
    /// Republish what the lock-free side reads, after recomputing the flags a
    /// FIFO fill decides.
    ///
    /// Taking `&mut` is what keeps [`State::refresh`] from being forgotten:
    /// every path that ends in a publish gets its levels, `TXE`, `RXNE` and
    /// `BSY` brought into line with the FIFOs first, and on an `"f4"` the
    /// refresh is a no-op.
    fn publish(&self, state: &mut State) {
        state.refresh();
        self.ticks.store(state.ticks, Ordering::Relaxed);
        self.next_event
            .store(Shared::next_event(state), Ordering::Relaxed);
    }

    /// Whether a master has a frame it could start on the next tick.
    ///
    /// A receive-only master always has one — it clocks itself for as long as
    /// it is enabled (§28.3.4) — and so does one with a whole frame waiting in
    /// its Tx FIFO or a CRC transfer part-way through. The last two matter
    /// because the `"f7"` block can hold several frames: without them the
    /// device would fall idle with work queued and nothing would wake it, since
    /// only a register write calls `Shared::begin` from outside.
    fn pending(state: &State) -> bool {
        state.is_master()
            && state.is_enabled()
            && (state.receive_only() || state.crc_left > 0 || state.peek_tx_frame().is_some())
    }

    /// The tick the next thing happens on, or [`NO_EVENT`].
    fn next_event(state: &State) -> u64 {
        if !state.busy {
            if Shared::pending(state) {
                return state.ticks.saturating_add(1);
            }
            return NO_EVENT;
        }
        let half = state.half_period();
        let next = state
            .started
            .saturating_add((u64::from(state.edges) + 1) * half);
        next.max(state.ticks.saturating_add(1))
    }

    /// Bring the peripheral up to date before an access.
    ///
    /// A debug access advances nothing (`ROADMAP.md` §15, invariant 5).
    fn sync(&self, attrs: MemAttrs) {
        let handle = self.lazy.lock().clone();
        let Some(handle) = handle else {
            return;
        };
        let kind = if attrs.debug {
            AccessKind::Debug
        } else {
            AccessKind::Guest
        };
        // A refusal means catch-up is already running further up the stack;
        // answering from where the peripheral stands is the only defined thing
        // to do.
        let _ = handle.sync(kind);
    }

    fn emit(&self, action: Emit) {
        let port = {
            let pins = self.pins.lock();
            match action {
                Emit::Sck(_) => pins.sck.clone(),
                Emit::Mosi(_) => pins.mosi.clone(),
                Emit::Nss(_) => pins.nss.clone(),
            }
        };
        let level = match action {
            Emit::Sck(l) | Emit::Mosi(l) | Emit::Nss(l) => l,
        };
        if let Some(port) = port {
            port.set(level);
        }
    }

    /// Re-drive every output from the state, for the realize sweep.
    fn announce_all(&self) {
        let (sck, nss) = {
            let state = self.state.lock();
            (
                state.format().mode.idle_level(),
                Level::from_bool(!state.nss_low),
            )
        };
        self.emit(Emit::Sck(sck));
        self.emit(Emit::Nss(nss));
        self.publish_irq();
    }

    /// Whether any enabled interrupt source is asserting (§28.3.11's table).
    fn irq_state(state: &State) -> bool {
        let sr = state.sr;
        let cr2 = state.cr2;
        (cr2 & CR2_TXEIE != 0 && sr & SR_TXE != 0)
            || (cr2 & CR2_RXNEIE != 0 && sr & SR_RXNE != 0)
            || (cr2 & CR2_ERRIE != 0 && sr & (SR_MODF | SR_OVR | SR_CRCERR | SR_FRE | SR_UDR) != 0)
    }

    fn publish_irq(&self) {
        let level = Level::from_bool(Shared::irq_state(&self.state.lock()));
        self.irq_level.store(level.is_high(), Ordering::Relaxed);
        let port = self.irq.lock().clone();
        if let Some(port) = port {
            port.set(level);
        }
    }

    /// Move the NSS output and, transactionally, the bus's chip select.
    fn drive_nss(&self, low: bool) {
        self.emit(Emit::Nss(Level::from_bool(!low)));
        if self.link == Link::Transactional
            && let Some(bus) = &self.bus
        {
            bus.select(low.then_some(self.cs));
        }
    }

    /// Re-drive `NSS` from a snapshot, without pretending the line just moved.
    ///
    /// The restore twin of [`Shared::drive_nss`], and the difference is the
    /// whole point. A fresh bus has nothing selected, so re-driving a `low`
    /// `NSS` through `SpiBus::select` would look to the part like a chip select
    /// that had this instant fallen — and a slave's `select` is where it starts
    /// a frame, so the mid-frame state the snapshot just restored would be
    /// thrown away. [`SpiBus::restore_select`] puts the line back where it was
    /// and tells nobody, which is right because nothing moved.
    ///
    /// A deselected peripheral claims nothing: the other master on the bus may
    /// hold the line, and its own load restores that.
    fn restore_nss(&self, low: bool) {
        self.emit(Emit::Nss(Level::from_bool(!low)));
        if low
            && self.link == Link::Transactional
            && let Some(bus) = &self.bus
        {
            bus.restore_select(Some(self.cs));
        }
    }

    /// Start a frame, if the peripheral is in a position to.
    ///
    /// Called with the state lock held. Returns whether one started.
    fn begin(state: &mut State) -> bool {
        if state.busy || !state.is_enabled() || !state.is_master() {
            return false;
        }
        // §28.3.6: with `CRCNEXT` set the next frame carries the CRC instead
        // of the data, and the calculators are frozen while it does. §42.4.11
        // adds only that the CRC may be wider than a frame, in which case it
        // takes more than one of them.
        if state.crc_left == 0 && state.cr1 & (CR1_CRCEN | CR1_CRCNEXT) == (CR1_CRCEN | CR1_CRCNEXT)
        {
            state.crc_left = state.crc_frames();
            state.crc_in = 0;
            state.cr1 &= !CR1_CRCNEXT;
        }
        if state.crc_left > 0 {
            // Most significant frame first, which is the order the bits go out
            // in and so the order they have to be cut in.
            let shift = u32::from(state.frame_bits()) * u32::from(state.crc_left - 1);
            let word = if shift >= 16 {
                0
            } else {
                u32::from(state.txcrc) >> shift
            };
            state.shift = (word & state.format().mask()) as u16;
            state.crc_frame = true;
        } else if state.receive_only() {
            // §28.3.4: the clock free-runs and nothing is driven out.
            state.shift = 0xffff;
            state.crc_frame = false;
        } else if let Some(frame) = state.pop_tx_frame() {
            state.shift = frame;
            state.crc_frame = false;
        } else {
            return false;
        }
        // The Tx buffer emptied into the shift register, which is exactly what
        // §28.3.7 says sets `TXE`. On an `"f7"` `TXE` is a function of the
        // fill instead, and `refresh` below overwrites this.
        state.sr |= SR_TXE;
        state.sr |= SR_BSY;
        state.busy = true;
        state.started = state.ticks;
        state.edges = 0;
        state.shift_in = 0;
        state.refresh();
        true
    }

    /// A frame finished with `received` on the data line.
    fn finish(state: &mut State, received: u16) {
        let format = state.format();
        let received = (received as u32 & format.mask()) as u16;
        state.busy = false;
        state.sr &= !SR_BSY;
        if state.crc_frame {
            // §28.3.6: at the end of a CRC transfer the received value is
            // compared with the calculated one. Reassembled across however many
            // frames it took, most significant first.
            let bits = u32::from(state.frame_bits());
            state.crc_in = ((u32::from(state.crc_in) << bits) | u32::from(received)) as u16;
            state.crc_left = state.crc_left.saturating_sub(1);
            if state.crc_left == 0 {
                state.crc_frame = false;
                let crc_bits = u32::from(state.crc_bits());
                let mask = if crc_bits >= 16 {
                    u16::MAX
                } else {
                    (1u16 << crc_bits) - 1
                };
                if state.crc_in & mask != state.rxcrc & mask {
                    state.sr |= SR_CRCERR;
                }
            }
            state.refresh();
            return;
        }
        if state.cr1 & CR1_CRCEN != 0 {
            let poly = state.crcpr;
            let data_bits = format.bits;
            let crc_bits = state.crc_bits();
            state.txcrc = crc_step(state.txcrc, state.shift, data_bits, crc_bits, poly);
            state.rxcrc = crc_step(state.rxcrc, received, data_bits, crc_bits, poly);
        }
        if state.transmit_only() {
            // §28.3.4: nothing arrives on a wire the peripheral is driving.
            state.refresh();
            return;
        }
        match state.variant {
            Variant::F4 => {
                if state.sr & SR_RXNE != 0 {
                    // §28.3.10: "the receiver buffer contents are not updated
                    // with the newly received data" — the buffer freezes and
                    // the frame is lost.
                    state.sr |= SR_OVR;
                    state.ovr_dr_read = false;
                } else {
                    state.rx = received;
                    state.sr |= SR_RXNE;
                }
            }
            Variant::Fifo => {
                // §42.4.9: a frame that completes with no room left in the Rx
                // FIFO is lost and sets `OVR`. Room is counted in *bytes*,
                // because a wide frame needs two of them.
                let need = state.frame_bytes();
                if state.rx_fifo.free() < need {
                    state.sr |= SR_OVR;
                    state.ovr_dr_read = false;
                } else {
                    state.rx_fifo.push(received as u8);
                    if need == 2 {
                        state.rx_fifo.push((received >> 8) as u8);
                    }
                }
            }
        }
        state.refresh();
    }

    /// The bit of the shift register that goes out for bit index `n`.
    fn tx_bit(state: &State, n: u32) -> Level {
        let format = state.format();
        let bit = match format.order {
            BitOrder::MsbFirst => (state.shift >> (u32::from(format.bits) - 1 - n)) & 1,
            BitOrder::LsbFirst => (state.shift >> n) & 1,
        };
        Level::from_bool(bit != 0)
    }

    /// Fold a sampled MISO bit into the received word.
    fn capture(state: &mut State, n: u32, level: Level) {
        if !level.as_bool() {
            return;
        }
        let format = state.format();
        match format.order {
            BitOrder::MsbFirst => state.shift_in |= 1 << (u32::from(format.bits) - 1 - n),
            BitOrder::LsbFirst => state.shift_in |= 1 << n,
        }
    }

    /// Simulate forward to `target` domain ticks.
    ///
    /// Runs with **no lock held across an outward call**: each step decides
    /// what to do under the state lock, releases it, then drives the wire or
    /// reaches the slave (`core::device`, the re-entrancy contract).
    fn advance_to(&self, target: u64) {
        loop {
            enum Step {
                Done,
                /// A wired edge: drive these, in order, then loop.
                Edges(Vec<Emit>),
                /// A transactional frame: hand it to the bus, store the reply.
                Word(u16),
                /// A frame began; present its first bit and loop.
                Present,
            }

            let step = {
                let mut state = self.state.lock();
                if !state.busy {
                    // A receive-only master starts the next frame the instant
                    // the previous one ends, for as long as `SPE` stands.
                    if state.ticks < target && Shared::begin(&mut state) {
                        self.publish(&mut state);
                        Step::Present
                    } else {
                        state.ticks = state.ticks.max(target);
                        self.publish(&mut state);
                        Step::Done
                    }
                } else {
                    let half = state.half_period();
                    match self.link {
                        Link::Transactional => {
                            let end = state.end_tick();
                            if end > target {
                                state.ticks = target;
                                self.publish(&mut state);
                                Step::Done
                            } else {
                                state.ticks = end;
                                Step::Word(state.shift)
                            }
                        }
                        Link::Wired => {
                            let edge_at = state
                                .started
                                .saturating_add((u64::from(state.edges) + 1) * half);
                            if edge_at > target {
                                state.ticks = target;
                                self.publish(&mut state);
                                Step::Done
                            } else {
                                state.ticks = edge_at;
                                let k = state.edges;
                                let format = state.format();
                                let idle = format.mode.idle_level();
                                let level = if k.is_multiple_of(2) {
                                    idle.inverted()
                                } else {
                                    idle
                                };
                                let bit = k / 2;
                                let mut out = Vec::new();
                                if format.mode.samples_on(level) {
                                    let miso = Level::from_bool(self.miso.load(Ordering::Relaxed));
                                    Shared::capture(&mut state, bit, miso);
                                } else {
                                    let next = k.div_ceil(2).min(u32::from(format.bits) - 1);
                                    out.push(Emit::Mosi(Shared::tx_bit(&state, next)));
                                }
                                out.push(Emit::Sck(level));
                                state.edges += 1;
                                if state.edges >= state.total_edges() {
                                    let received = state.shift_in as u16;
                                    Shared::finish(&mut state, received);
                                }
                                self.publish(&mut state);
                                Step::Edges(out)
                            }
                        }
                    }
                }
            };

            match step {
                Step::Done => {
                    self.publish_irq();
                    return;
                }
                Step::Edges(actions) => {
                    for action in actions {
                        self.emit(action);
                    }
                    self.publish_irq();
                }
                Step::Word(word) => {
                    // Outward, with no lock of ours held.
                    let reply = self
                        .bus
                        .as_ref()
                        .map_or(0xffff, |bus| bus.transfer(u32::from(word)) as u16);
                    {
                        let mut state = self.state.lock();
                        Shared::finish(&mut state, reply);
                        self.publish(&mut state);
                    }
                    self.publish_irq();
                }
                Step::Present => {
                    self.present_first_bit();
                    self.publish_irq();
                }
            }
        }
    }

    /// Put the first bit on MOSI before the first clock edge.
    ///
    /// With CPHA 0 that edge is the one that samples it, so a shift register
    /// that only loaded on the first edge would present the wrong bit.
    fn present_first_bit(&self) {
        if self.link != Link::Wired {
            return;
        }
        let level = {
            let state = self.state.lock();
            if !state.busy {
                return;
            }
            Shared::tx_bit(&state, 0)
        };
        self.emit(Emit::Mosi(level));
    }

    /// Re-evaluate the mode-fault condition after something moved NSS or `CR1`.
    ///
    /// Returns whether the fault fired, so the caller can drive the outputs
    /// the demotion implies once it has let go of the lock.
    fn check_mode_fault(state: &mut State) -> bool {
        if !state.mode_fault_due() {
            return false;
        }
        // §28.3.10, and this is the whole of it: the flag sets, the peripheral
        // switches itself off, and it demotes itself to a slave.
        state.sr |= SR_MODF;
        state.cr1 &= !(CR1_SPE | CR1_MSTR);
        state.busy = false;
        state.sr &= !SR_BSY;
        state.nss_low = false;
        state.modf_sr_seen = false;
        // `SPE` fell, so the transmit side goes with it — see `State::flush_tx`
        // for why a peripheral nothing clocks may not be left holding bytes.
        // A CRC transfer part-way through goes too, or re-enabling the
        // peripheral would put its remaining frame on the wire out of nowhere.
        state.flush_tx();
        state.crc_left = 0;
        state.crc_frame = false;
        state.refresh();
        true
    }
}

// ---------------------------------------------------------------------------
// the register block
// ---------------------------------------------------------------------------

struct RegisterBlock {
    shared: Arc<Shared>,
    pins: Arc<SlavePins>,
}

impl fmt::Debug for RegisterBlock {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("RegisterBlock").finish_non_exhaustive()
    }
}

/// What a register access asks for once the state lock is released.
#[derive(Debug, Clone, Copy, Default)]
struct After {
    /// Move the NSS output (and the transactional bus's chip select) to this.
    nss: Option<bool>,
    /// Re-read the slave-side shifter's framing from `CR1`.
    reframe: bool,
    /// Present the first bit of a frame that just began.
    started: bool,
    /// Re-drive every output.
    announce: bool,
}

impl RegisterBlock {
    /// Read one register. `debug` suppresses every side effect.
    fn read_register(&self, offset: u64, debug: bool) -> u16 {
        if offset == 0x0c && self.shared.variant.has_fifo() {
            // The FIFO's `DR` is not a register with a value — how much it
            // moves depends on the access width. [`MemOps::read`] calls
            // [`RegisterBlock::read_dr_fifo`] with the real one; anything that
            // arrives here without one gets the half-word semantics, which is
            // what ST's own headers do by default.
            return self.read_dr_fifo(2, debug);
        }
        let mut state = self.shared.state.lock();
        match offset {
            0x00 => state.cr1,
            0x04 => state.cr2,
            0x08 => {
                let value = state.sr;
                if !debug {
                    // The trap `MemAttrs::debug` exists for, twice over. A
                    // read of `SR` is the *second half* of §28.3.10's overrun
                    // clearing sequence and the *first half* of its mode-fault
                    // one; a debugger that took either step would leave the
                    // guest's own sequence half-consumed and its next read
                    // lying.
                    if state.sr & SR_MODF != 0 {
                        state.modf_sr_seen = true;
                    }
                    if state.sr & SR_OVR != 0 && state.ovr_dr_read {
                        state.sr &= !SR_OVR;
                        state.ovr_dr_read = false;
                    }
                    // §28.5.3: `FRE` is cleared by reading `SR`.
                    state.sr &= !SR_FRE;
                }
                value
            }
            0x0c => {
                let value = state.rx;
                if !debug {
                    // And the third trap: a debugger that read `DR` would pop
                    // the guest's word and clear `RXNE`, and the guest would
                    // then read a stale one.
                    state.sr &= !SR_RXNE;
                    if state.sr & SR_OVR != 0 {
                        state.ovr_dr_read = true;
                    }
                }
                // §28.5.4: in 8-bit frame format the top half of a read is
                // forced to zero.
                if state.cr1 & CR1_DFF == 0 {
                    value & 0xff
                } else {
                    value
                }
            }
            0x10 => state.crcpr,
            0x14 => state.rxcrc,
            0x18 => state.txcrc,
            0x1c => state.i2scfgr,
            0x20 => state.i2spr,
            _ => 0,
        }
    }

    /// Pop `DR` on an `"f7"`, where the access width is part of the meaning.
    ///
    /// RM0351 §42.4.9: an 8-bit read takes **one** byte out of the Rx FIFO and
    /// a 16-bit read takes **two**, so an eight-bit-frame driver can unpack a
    /// pair in one access. Above eight bits a frame *is* two bytes, so any
    /// access takes the whole frame.
    ///
    /// A short FIFO is not an error: a 16-bit read that finds one byte answers
    /// with it in the low half and leaves the upper half zero, which is the
    /// last read of an odd-length DMA (`LDMA_RX`, module docs).
    ///
    /// `debug` reads the same bytes and **pops nothing** — the `MemAttrs::debug`
    /// rule, and the sharpest case of it in this file: a debugger that dumped
    /// the block would eat the guest's data, and the guest would never know.
    fn read_dr_fifo(&self, len: usize, debug: bool) -> u16 {
        let mut state = self.shared.state.lock();
        let want = if len == 1 && state.frame_bytes() == 1 {
            1
        } else {
            2
        };
        if debug {
            let lo = u16::from(state.rx_fifo.at(0).unwrap_or(0));
            let hi = u16::from(state.rx_fifo.at(1).unwrap_or(0));
            return if want == 1 { lo } else { lo | (hi << 8) };
        }
        let mut value = 0u16;
        for i in 0..want {
            match state.rx_fifo.pop() {
                Some(byte) => value |= u16::from(byte) << (8 * i),
                None => break,
            }
        }
        if state.sr & SR_OVR != 0 {
            // The first half of §28.3.10's clearing sequence, unchanged by the
            // FIFO: a read of `DR`, then a read of `SR`.
            state.ovr_dr_read = true;
        }
        state.refresh();
        value
    }

    /// Push `DR` on an `"f7"`, where the access width is part of the meaning.
    ///
    /// The mirror of [`RegisterBlock::read_dr_fifo`]: an 8-bit write queues one
    /// byte and a 16-bit write queues two, low byte first — two frames at
    /// `DS ≤ 8`, one at `DS > 8`. A 32-bit write is not a width the manual
    /// defines for `DR`; it is taken as its low half-word, which is the same
    /// liberty the `"f4"` half already takes.
    ///
    /// `LDMA_TX` is the one wrinkle, and the module docs carry the argument.
    fn write_dr_fifo(&self, src: &[u8]) -> After {
        let mut after = After::default();
        let mut state = self.shared.state.lock();
        let two = src.len() > 1;
        let lo = src[0];
        let hi = if two { src[1] } else { 0 };
        // Whatever the last packed write held back goes in ahead of this one:
        // it was written first and the wire order is the write order.
        if let Some(held) = state.tx_held.take() {
            state.push_tx_byte(held);
        }
        state.push_tx_byte(lo);
        if two {
            if state.ldma_tx_active() {
                state.tx_held = Some(hi);
            } else {
                state.push_tx_byte(hi);
            }
        }
        after.started = Shared::begin(&mut state);
        self.shared.publish(&mut state);
        after
    }

    /// Write one register, reporting what has to happen once the lock is
    /// released.
    fn write_register(&self, offset: u64, value: u16) -> After {
        let mut after = After::default();
        let mut state = self.shared.state.lock();
        match offset {
            0x00 => {
                let was_format = state.format();
                let was_nss = state.nss_output_low();
                let modf = state.sr & SR_MODF != 0;
                let mut next = value;
                if modf {
                    // §28.3.10: "hardware does not allow the setting of the
                    // SPE and MSTR bits while the MODF bit is set". This is
                    // the clause that turns a driver's missing NSS pull-up
                    // into a peripheral that silently will not start.
                    next &= !(CR1_SPE | CR1_MSTR);
                    if state.modf_sr_seen {
                        // The second half of the clearing sequence.
                        state.sr &= !SR_MODF;
                        state.modf_sr_seen = false;
                    }
                }
                let crc_rising = next & CR1_CRCEN != 0 && state.cr1 & CR1_CRCEN == 0;
                state.cr1 = next;
                if crc_rising {
                    // §28.5.5: enabling the calculator resets both registers.
                    state.rxcrc = 0;
                    state.txcrc = 0;
                }
                if state.cr1 & CR1_SPE == 0 {
                    // Disabling the peripheral stops whatever was in flight
                    // and clears `BSY` (§28.3.7).
                    state.busy = false;
                    state.sr &= !SR_BSY;
                    // And empties the transmit side, which on an `"f7"` is
                    // what keeps `BSY` — a function of `FTLVL` there — from
                    // standing for ever. `State::flush_tx` has the argument.
                    state.flush_tx();
                    state.crc_left = 0;
                    state.crc_frame = false;
                }
                Shared::check_mode_fault(&mut state);
                if state.format() != was_format {
                    after.reframe = true;
                }
                let nss = state.nss_output_low();
                state.nss_low = nss;
                if nss != was_nss {
                    after.nss = Some(nss);
                }
                after.started = Shared::begin(&mut state);
                after.announce = true;
                self.shared.publish(&mut state);
            }
            0x04 => {
                let was_nss = state.nss_output_low();
                let was_format = state.format();
                let mut next = value & state.variant.cr2_mask();
                if state.variant.has_fifo() {
                    // §42.6.2: the three `DS` codes below `0b0011` are "not
                    // used" and the hardware forces eight bits. Forcing it on
                    // the way in is what makes a read-back tell the truth
                    // rather than reporting a size the peripheral is not using.
                    if (next >> CR2_DS_SHIFT) & CR2_DS_MASK < DS_CODE_MIN {
                        next = (next & !(CR2_DS_MASK << CR2_DS_SHIFT))
                            | (DS_CODE_EIGHT << CR2_DS_SHIFT);
                    }
                }
                state.cr2 = next;
                if !state.ldma_tx_active() {
                    // The packed stream is over — `TXDMAEN` or `LDMA_TX` went
                    // away — so the byte held back for a frame that will never
                    // be asked for goes with it.
                    state.tx_held = None;
                }
                if state.format() != was_format {
                    after.reframe = true;
                }
                Shared::check_mode_fault(&mut state);
                let nss = state.nss_output_low();
                state.nss_low = nss;
                if nss != was_nss {
                    after.nss = Some(nss);
                }
                self.shared.publish(&mut state);
            }
            0x08 => {
                // §28.5.3: every bit is read-only except `CRCERR`, which is
                // cleared by writing zero to it. A *write* to `SR` is also the
                // first half of the mode-fault clearing sequence.
                if state.sr & SR_MODF != 0 {
                    state.modf_sr_seen = true;
                }
                if value & SR_CRCERR == 0 {
                    state.sr &= !SR_CRCERR;
                }
            }
            0x0c => {
                let format = state.format();
                state.tx = format.truncate(u32::from(value)) as u16;
                state.tx_pending = true;
                state.sr &= !SR_TXE;
                after.started = Shared::begin(&mut state);
                self.shared.publish(&mut state);
            }
            0x10 => state.crcpr = value,
            // `RXCRCR` and `TXCRCR` are read-only (§28.5.6, §28.5.7).
            0x14 | 0x18 => {}
            0x1c => state.i2scfgr = value,
            0x20 => state.i2spr = value,
            _ => {}
        }
        after
    }

    /// Perform the outward half of an access.
    fn settle(&self, after: After) {
        if after.reframe {
            // The slave-side shifter caches the framing it was built with, and
            // the fabric offers no way to change it without abandoning the
            // word in flight — which is what §28.5.1 says to do anyway
            // ("`DFF` should be written only when SPI is disabled").
            self.pins.reset();
        }
        if let Some(low) = after.nss {
            self.shared.drive_nss(low);
        }
        if after.started {
            self.shared.present_first_bit();
        }
        if after.announce {
            self.shared.announce_all();
        }
        self.shared.publish_irq();
    }
}

impl MemOps for RegisterBlock {
    fn read(&self, offset: u64, dst: &mut [u8], attrs: MemAttrs) -> MemResult {
        if !matches!(dst.len(), 1 | 2 | 4) {
            return Err(BusError::BadAccess);
        }
        let register = offset & !3;
        let within = offset - register;
        if within + dst.len() as u64 > 4 || (dst.len() == 4 && within != 0) {
            return Err(BusError::BadAccess);
        }
        if register > LAST_REGISTER {
            dst.fill(0);
            return Ok(());
        }
        self.shared.sync(attrs);
        if register == 0x0c && self.shared.variant.has_fifo() {
            // `DR` is a FIFO port, not a value in a word: the access width
            // decides how much comes out, and the byte lane within the word
            // does not enter into it — ST's own accessor is a `uint8_t` cast at
            // offset zero and the manual defines nothing else.
            let bytes = self.read_dr_fifo(dst.len(), attrs.debug).to_le_bytes();
            for (i, byte) in dst.iter_mut().enumerate() {
                *byte = bytes.get(i).copied().unwrap_or(0);
            }
            if !attrs.debug {
                self.shared.publish_irq();
            }
            return Ok(());
        }
        let value = u32::from(self.read_register(register, attrs.debug));
        let bytes = value.to_le_bytes();
        for (i, byte) in dst.iter_mut().enumerate() {
            *byte = bytes[(within as usize + i).min(3)];
        }
        if !attrs.debug {
            // A read of `DR` or `SR` clears flags, and a cleared flag can drop
            // the interrupt line.
            self.shared.publish_irq();
        }
        Ok(())
    }

    fn write(&self, offset: u64, src: &[u8], attrs: MemAttrs) -> MemResult {
        if !matches!(src.len(), 1 | 2 | 4) {
            return Err(BusError::BadAccess);
        }
        let register = offset & !3;
        let within = offset - register;
        if within + src.len() as u64 > 4 || (src.len() == 4 && within != 0) {
            return Err(BusError::BadAccess);
        }
        if attrs.debug {
            // A debug write would start a frame or move a chip select, neither
            // of which the core can make harmless.
            return Err(BusError::BadAccess);
        }
        if register > LAST_REGISTER {
            return Ok(());
        }
        self.shared.sync(attrs);
        if register == 0x0c && self.shared.variant.has_fifo() {
            // As above, and for the same reason: how many frames this write
            // queues is a property of how wide it is.
            let after = self.write_dr_fifo(src);
            self.settle(after);
            return Ok(());
        }
        // A byte write reaches its own lane of the sixteen-bit register; the
        // rest keeps what it had, which is what a narrow store on this bus
        // does.
        let value = if src.len() == 1 && within == 1 {
            let old = self.read_register(register, true);
            (old & 0x00ff) | (u16::from(src[0]) << 8)
        } else if src.len() == 1 {
            let old = self.read_register(register, true);
            (old & 0xff00) | u16::from(src[0])
        } else {
            u16::from(src[0]) | (u16::from(src[1]) << 8)
        };
        let after = self.write_register(register, value);
        self.settle(after);
        Ok(())
    }

    fn constraints(&self) -> AccessConstraints {
        AccessConstraints::ANY
            .with_widths(Width::U8, Width::U32)
            .with_endian(Endian::Little)
    }
}

// ---------------------------------------------------------------------------
// the slave face
// ---------------------------------------------------------------------------

impl SpiSlave for Shared {
    fn format(&self) -> Format {
        self.state.lock().format()
    }

    fn select(&self, selected: bool) {
        let mut state = self.state.lock();
        // NSS is what selects a slave, and it is the same pin a master watches
        // for a mode fault (§28.3.1). `SlavePins` has already turned the wire's
        // active-low level into this boolean.
        state.nss_in = Level::from_bool(!selected);
        Shared::check_mode_fault(&mut state);
    }

    fn transfer(&self, mosi: u32) -> u32 {
        let mut state = self.state.lock();
        if state.is_master() || !state.is_enabled() {
            // Not listening. A master's own MISO input is not this path, and a
            // disabled peripheral drives nothing.
            return u32::MAX;
        }
        let out = state.shift;
        // The shift register reloads from whatever the transmit side holds —
        // the single buffer on an `"f4"`, the head of the Tx FIFO on an
        // `"f7"`. A slave with nothing queued repeats zeroes, which is what an
        // unloaded shift register does.
        state.shift = state.pop_tx_frame().unwrap_or(0);
        state.sr |= SR_TXE;
        Shared::finish(&mut state, mosi as u16);
        u32::from(out)
    }

    fn peek(&self) -> u32 {
        let state = self.state.lock();
        if state.is_master() || !state.is_enabled() {
            return u32::MAX;
        }
        u32::from(state.peek_tx_frame().unwrap_or(0))
    }
}

// ---------------------------------------------------------------------------
// the input pins
// ---------------------------------------------------------------------------

/// One of the peripheral's own input pins — the ones `SlavePins` does not own.
struct InputSink {
    shared: Arc<Shared>,
    pins: Arc<SlavePins>,
    line: u32,
}

impl fmt::Debug for InputSink {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("InputSink")
            .field("line", &self.line)
            .finish()
    }
}

impl WireSink for InputSink {
    fn set_level(&self, _src: WireId, _line: u32, level: Level) {
        match self.line {
            pin::MISO_LINE => {
                self.shared.miso.store(level.as_bool(), Ordering::Relaxed);
            }
            pin::NSS_IN_LINE => {
                // The pin serves both roles: it selects this peripheral when
                // it is the slave, and it is what a master faults on.
                self.pins.drive(slave_pin::CS, level);
                let faulted = {
                    let mut state = self.shared.state.lock();
                    state.nss_in = level;
                    Shared::check_mode_fault(&mut state)
                };
                if faulted {
                    self.shared.drive_nss(false);
                    self.shared.publish_irq();
                }
            }
            _ => {}
        }
    }
}

// ---------------------------------------------------------------------------
// Device
// ---------------------------------------------------------------------------

impl Device for Stm32Spi {
    fn class(&self) -> &'static DeviceClass {
        &CLASS
    }

    fn realize(&self, _ctx: &mut RealizeCtx<'_>) -> Result<()> {
        // Nothing outward: a `map` statement places the region and `wire`
        // statements connect the pins.
        Ok(())
    }

    fn reset(&self, _kind: ResetKind) {
        {
            let mut state = self.shared.state.lock();
            // The tick is kept: `Machine::reset` does not rewind a clock
            // domain, and a lazily advanced device that zeroed its own tick
            // would then be asked to advance backwards.
            let ticks = state.ticks;
            // And the input level is kept: it belongs to whatever drives it,
            // and resetting this device does not move another device's pin.
            let nss_in = state.nss_in;
            *state = State {
                ticks,
                nss_in,
                ..State::new(self.shared.variant)
            };
            self.shared.publish(&mut state);
        }
        self.shared.miso.store(true, Ordering::Relaxed);
        self.pins.reset();
        self.shared.drive_nss(false);
        self.shared.announce_all();
    }

    fn save(&self, w: &mut ChunkWriter<'_>) -> Result<()> {
        let state = *self.shared.state.lock();
        w.write_u64(state.ticks)?;
        w.write_u16(state.cr1)?;
        w.write_u16(state.cr2)?;
        w.write_u16(state.sr)?;
        w.write_u16(state.tx)?;
        w.write_u16(state.rx)?;
        w.write_u16(state.crcpr)?;
        w.write_u16(state.rxcrc)?;
        w.write_u16(state.txcrc)?;
        w.write_u16(state.i2scfgr)?;
        w.write_u16(state.i2spr)?;
        w.write_bool(state.tx_pending)?;
        w.write_u16(state.shift)?;
        w.write_bool(state.busy)?;
        w.write_bool(state.crc_frame)?;
        w.write_u64(state.started)?;
        w.write_u32(state.edges)?;
        w.write_u32(state.shift_in)?;
        w.write_bool(state.nss_low)?;
        // The two half-consumed flag-clearing sequences. A snapshot taken
        // between a driver's `DR` read and its `SR` read is a snapshot with
        // half of §28.3.10's overrun sequence done, and restoring it as
        // untouched would make the guest's next read lie.
        w.write_bool(state.ovr_dr_read)?;
        w.write_bool(state.modf_sr_seen)?;
        // The `"f7"` data path. Written unconditionally — an `"f4"`'s FIFOs are
        // empty and cost nine bytes, and a chunk whose shape depended on a
        // construction property would be one more thing a loader has to agree
        // about before it can read the first field.
        for byte in state.tx_fifo.bytes {
            w.write_u8(byte)?;
        }
        w.write_u8(state.tx_fifo.len)?;
        for byte in state.rx_fifo.bytes {
            w.write_u8(byte)?;
        }
        w.write_u8(state.rx_fifo.len)?;
        w.write_bool(state.tx_held.is_some())?;
        w.write_u8(state.tx_held.unwrap_or(0))?;
        // Where a multi-frame CRC transfer had got to. A snapshot taken between
        // the two halves of a sixteen-bit CRC over eight-bit frames is a
        // snapshot mid-CRC, and restoring it as "not started" would send the
        // high half twice.
        w.write_u8(state.crc_left)?;
        w.write_u16(state.crc_in)?;
        let (rx, tx, count, selected, sck, mosi, loaded) = self.pins.snapshot();
        w.write_u32(rx)?;
        w.write_u32(tx)?;
        w.write_u8(count)?;
        w.write_bool(selected)?;
        w.write_bool(sck)?;
        w.write_bool(mosi)?;
        w.write_bool(loaded)
        // `nss_in` and the MISO level are not saved: they are levels *another*
        // device is driving, and that device restores its own state and drives
        // them again (`ROADMAP.md` §4.5).
    }

    fn load(&self, r: &mut ChunkReader<'_>) -> Result<()> {
        let mut state = State {
            // Not in the chunk: it is a construction property, and a snapshot
            // restored into a differently built machine is a machine-shape
            // mismatch rather than something to paper over here.
            variant: self.shared.variant,
            ticks: r.read_u64()?,
            cr1: r.read_u16()?,
            cr2: r.read_u16()?,
            sr: r.read_u16()?,
            tx: r.read_u16()?,
            rx: r.read_u16()?,
            crcpr: r.read_u16()?,
            rxcrc: r.read_u16()?,
            txcrc: r.read_u16()?,
            i2scfgr: r.read_u16()?,
            i2spr: r.read_u16()?,
            tx_pending: r.read_bool()?,
            shift: r.read_u16()?,
            busy: r.read_bool()?,
            crc_frame: r.read_bool()?,
            started: r.read_u64()?,
            edges: r.read_u32()?,
            shift_in: r.read_u32()?,
            nss_low: r.read_bool()?,
            nss_in: Level::High,
            ovr_dr_read: r.read_bool()?,
            modf_sr_seen: r.read_bool()?,
            tx_fifo: Fifo::default(),
            rx_fifo: Fifo::default(),
            tx_held: None,
            crc_left: 0,
            crc_in: 0,
        };
        for i in 0..FIFO_BYTES {
            state.tx_fifo.bytes[i] = r.read_u8()?;
        }
        state.tx_fifo.len = r.read_u8()?.min(FIFO_BYTES as u8);
        for i in 0..FIFO_BYTES {
            state.rx_fifo.bytes[i] = r.read_u8()?;
        }
        state.rx_fifo.len = r.read_u8()?.min(FIFO_BYTES as u8);
        let held = r.read_bool()?;
        let held_byte = r.read_u8()?;
        state.tx_held = held.then_some(held_byte);
        state.crc_left = r.read_u8()?;
        state.crc_in = r.read_u16()?;
        let pins = (
            r.read_u32()?,
            r.read_u32()?,
            r.read_u8()?,
            r.read_bool()?,
            r.read_bool()?,
            r.read_bool()?,
            r.read_bool()?,
        );
        {
            let mut slot = self.shared.state.lock();
            state.nss_in = slot.nss_in;
            *slot = state;
            self.shared.publish(&mut slot);
        }
        self.pins.restore(pins);
        self.shared.restore_nss(state.nss_low);
        self.shared.announce_all();
        Ok(())
    }

    fn region(&self, name: &str) -> Option<RegionRef> {
        matches!(name, "" | "regs").then(|| Arc::clone(&self.region))
    }

    fn sink(&self, port: &str, _sources: &[WireId]) -> Option<SinkPin> {
        // The slave-side data pins are the fabric's own.
        let slave_line = match port {
            pin::SCK_IN => Some(slave_pin::SCK),
            pin::MOSI_IN => Some(slave_pin::MOSI),
            _ => None,
        };
        if let Some(line) = slave_line {
            return Some(SinkPin {
                sink: self.pins.sink(line),
                line,
            });
        }
        let line = match port {
            pin::MISO => pin::MISO_LINE,
            pin::NSS_IN => pin::NSS_IN_LINE,
            _ => return None,
        };
        let sink = Arc::new(InputSink {
            shared: Arc::clone(&self.shared),
            pins: Arc::clone(&self.pins),
            line,
        });
        // Kept, because a net refers to its sinks weakly.
        self.shared.sinks.lock().push(Arc::clone(&sink));
        Some(SinkPin {
            sink: sink as Arc<dyn WireSink>,
            line,
        })
    }

    fn connect(&self, port: &str, source: WireSource) -> Result<()> {
        match port {
            pin::MISO_OUT => {
                self.pins.connect_miso(source);
                return Ok(());
            }
            "irq" => {
                *self.shared.irq.lock() = Some(source);
                self.shared.publish_irq();
                return Ok(());
            }
            _ => {}
        }
        let mut pins = self.shared.pins.lock();
        match port {
            pin::SCK => pins.sck = Some(source),
            pin::MOSI => pins.mosi = Some(source),
            pin::NSS => pins.nss = Some(source),
            _ => {
                return Err(Error::Config {
                    at: String::from(port),
                    message: alloc::format!(
                        "an STM32 SPI drives `{}`, `{}`, `{}`, `{}` and `irq`",
                        pin::SCK,
                        pin::MOSI,
                        pin::NSS,
                        pin::MISO_OUT
                    ),
                });
            }
        }
        drop(pins);
        self.shared.announce_all();
        Ok(())
    }

    fn announce(&self, port: &str) {
        if port == pin::MISO_OUT {
            self.pins.publish_miso();
        } else {
            self.shared.announce_all();
        }
    }

    // -- lazily advanced (`ROADMAP.md` §4.2) ---------------------------------

    /// Yes. A frame takes real time, a driver polls `BSY` and `TXE` to find
    /// out when it is done, and the answer has to be the one at the cycle of
    /// the poll.
    fn is_lazy(&self) -> bool {
        true
    }

    fn current_tick(&self) -> u64 {
        self.shared.ticks.load(Ordering::Relaxed)
    }

    fn advance_to(&self, tick: u64) {
        Stm32Spi::advance_to(self, tick);
    }

    fn next_event_tick(&self) -> Option<u64> {
        match self.shared.next_event.load(Ordering::Relaxed) {
            NO_EVENT => None,
            tick => Some(tick),
        }
    }

    fn attach_lazy(&self, handle: LazyHandle) {
        *self.shared.lazy.lock() = Some(handle);
    }
}

impl Instance for Stm32Spi {}

/// The `stm32.spi` device class.
pub static CLASS: DeviceClass = DeviceClass {
    name: CLASS_NAME,
    version: STATE_VERSION,
    summary: "STM32 SPI, F4 (RM0090 §28) or F0/F3/F7/L4/G4/WB (RM0351 §42) by `variant`: \
              CR1/CR2/SR/DR/CRCPR, master and slave, the four CPOL/CPHA modes, SSM/SSI/SSOE \
              and the mode fault, and on the later block a four-byte FIFO each way with \
              FRLVL/FTLVL/FRXTH, a 4-to-16-bit DS, CRCL and LDMA_RX/LDMA_TX",
    properties: &[
        PropertySpec {
            name: "variant",
            kind: ValueKind::Str,
            required: false,
            summary: "which generation: `f4` (no FIFO, DFF) or `f7` (FIFO, DS). Default `f4`",
        },
        PropertySpec {
            name: "link",
            kind: ValueKind::Str,
            required: true,
            summary: "how words reach the slaves: `transactional` or `wired`",
        },
        PropertySpec {
            name: "bus",
            kind: ValueKind::Str,
            required: false,
            summary: "the named SPI bus this peripheral masters, for `transactional`",
        },
        PropertySpec {
            name: "cs",
            kind: ValueKind::Uint,
            required: false,
            summary: "which chip select on that bus the NSS output stands for (default 0)",
        },
    ],
    construct: |props| Ok(Box::new(Stm32Spi::new(props)?)),
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
    bindings.bind(CLASS_NAME, |props| Ok(Arc::new(Stm32Spi::new(props)?)))
}

/// What the validator should know about `stm32.spi`.
#[must_use]
pub fn schema() -> ClassSchema {
    ClassSchema::new(CLASS_NAME)
        .prop(PropSchema::new("variant", ValueKind::Str).values(Variant::NAMES))
        .prop(
            PropSchema::new("link", ValueKind::Str)
                .required()
                .values(Link::NAMES),
        )
        .prop(PropSchema::new("bus", ValueKind::Str))
        .prop(PropSchema::new("cs", ValueKind::Uint).range(0, MAX_CHIP_SELECTS as u64 - 1))
        .port(pin::SCK, PortDir::Out)
        .port(pin::MOSI, PortDir::Out)
        .port(pin::NSS, PortDir::Out)
        .port(pin::MISO, PortDir::In)
        .port(pin::SCK_IN, PortDir::In)
        .port(pin::MOSI_IN, PortDir::In)
        .port(pin::NSS_IN, PortDir::In)
        .port(pin::MISO_OUT, PortDir::Out)
        .port("irq", PortDir::Out)
        .region("")
        .region("regs")
}

#[cfg(test)]
mod tests;
