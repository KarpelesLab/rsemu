//! The A3000's Super DMAC: the board's half of its SCSI port.
//!
//! An A3000 has two chips where an A2091 card has two chips: a **WD33C93A**,
//! which is the SCSI bus interface controller and lives in
//! [`crate::dev::wd33c93`], and Commodore's own **DMAC**, which is a bus master
//! that moves a data phase's bytes between the SCSI chip and 32-bit memory.
//! This is the second of the two, and it contains **no SCSI register and no
//! SCSI command**: it decodes addresses, counts bytes into memory and passes
//! two of its addresses straight through to the controller.
//!
//! # The register map
//!
//! "The WD 33C93 registers are actually mapped into the DMAC register space"
//! (*The A3000+ System Specification*, §2.4.1) — so a machine file maps one
//! window at `$DD0000` and both chips are behind it. Table 2-5, with the rows
//! the A3000 has:
//!
//! | Address | Name | Type | Function |
//! | --- | --- | --- | --- |
//! | `$DD0004` | `WTC` | R/W | Word Transfer Count (obsolete) |
//! | `$DD0008` | `CONTR` | R/W | Control |
//! | `$DD000C` | `ACR` | R/W | Address Control |
//! | `$DD0010` | `ST_DMA` | Strobe | Start DMA |
//! | `$DD0014` | `FLUSH` | Strobe | Flush FIFO |
//! | `$DD0018` | `CLR_INT` | Strobe | Clear Interrupts |
//! | `$DD001C` | `ISTR` | Read | Interrupt Status |
//! | `$DD003C` | `SP_DMA` | Strobe | Stop DMA |
//! | `$DD0040` | `SASR` | Write | the WD33C93's Address register, as a longword |
//! | `$DD0041` | `SASR` | Read | the same, as a byte |
//! | `$DD0047` | `SCMD` | R/W | the WD33C93's indirect register, as a byte |
//! | `$DD0049` | `SASR` | R/W | the byte-wide Address register of the *enhanced* part |
//!
//! Every DMAC register behaves as a whole longword "even if accessed via word
//! or byte instructions" (§2.4), which is what [`Regs::read_long`] is: one
//! value per aligned longword, spliced for a narrower access. The `SASR` and
//! `SCMD` addresses are the exception, because the thing behind them is a
//! byte-wide register on another chip.
//!
//! `$DD0049` is the *enhanced* DMAC's mapping and an A3000 does not have it;
//! it is decoded here anyway because decoding it costs nothing, changes nothing
//! a driver can see on this board, and means an A3000T or an A4000 machine file
//! does not need a second model. The way software tells the two parts apart is
//! `WTC` bit 2, which is read/write on the old part and fixed at zero on the
//! new one (§2.4.1) — and it is read/write here, so this board answers as the
//! A3000's own DMAC.
//!
//! # `CONTR` and `ISTR`
//!
//! Table 2-6, both registers, every bit:
//!
//! | Register | Value | Name | Function |
//! | --- | --- | --- | --- |
//! | `CONTR` | `8` | `DMAENA` | DMA enabled |
//! | | `4` | `PREST` | reset the WD33C93 |
//! | | `2` | `INTENA` | interrupt enable |
//! | | `1` | `DMADIR` | direction: high is memory → SCSI |
//! | `ISTR` | `80` | `INT_F` | interrupt follow |
//! | | `40` | `INT_S` | interrupt SCSI |
//! | | `20` | `E_INT` | end of process |
//! | | `10` | `INT_P` | interrupt pending |
//! | | `02` | `FF` | FIFO full |
//! | | `01` | `FE` | FIFO empty |
//!
//! "Bits INT_F, INT_S, and E_INT each reflect the status of the WD33C93
//! interrupt line, which is an active high interrupt. The INT_P bit is low if
//! INTENA is cleared, otherwise it indicates the status of the WD33C93 line"
//! (§2.4.1) — so all four are a live view of one wire, and that is exactly how
//! they are computed here rather than being latched.
//!
//! # What is not modelled, and why it does not matter
//!
//! **The FIFO.** The real part has a FIFO between the SCSI chip and memory,
//! four longwords deep on an A3000 and eight on an A3000+, and `FLUSH` exists
//! to push its tail out after the WD33C93 says the transfer is over. Here each
//! byte reaches memory in the call that carried it, so the FIFO is always empty
//! — `FE` reads 1 and `FF` reads 0 — and `FLUSH` has nothing to do. A driver
//! that strobes `FLUSH` gets the same memory either way, which is the test that
//! says the simplification is safe.
//!
//! **`WTC`.** "The WTC register is considered obselete" (§2.4.1); the transfer
//! length that matters is the WD33C93's own Transfer Count register. It is a
//! plain read/write longword here, which is the part it still plays: telling
//! software which DMAC it is talking to.
//!
//! **Odd-byte alignment.** "the ACR, physically contained in the RAMSEY chip,
//! will actually round any value written to it down to an even word, since the
//! DMAC doesn't support odd-byte aligned transfers" (§2.4.1). The rounding is
//! modelled; the RAMSEY chip is not.
//!
//! # Sources
//!
//! *The A3000+ System Specification*, Commodore-Amiga — §2.4 the DMAC, §2.4.1
//! the register map (Table 2-5) and the control and interrupt bits (Table 2-6).
//! It documents the enhanced part *and* states which rows the A3000's own has,
//! which is why it is the citation for a board that shipped first.
//! *WD33C93A SCSI Bus Interface Controller*, Western Digital, for what is
//! behind `SASR` and `SCMD` — see [`crate::dev::wd33c93`].
//!
//! **No emulator source, no AROS source and no Kickstart disassembly was
//! consulted** (`CLAUDE.md`, provenance).

#[cfg(test)]
mod tests;

use alloc::boxed::Box;
use alloc::format;
use alloc::string::{String, ToString};
use alloc::sync::{Arc, Weak};
use core::fmt;

use crate::core::device::{Device, DeviceClass, ExportId, PropertySpec, RealizeCtx, ResetKind};
use crate::core::error::{BusError, Error, Result};
use crate::core::props::{Props, ValueKind};
use crate::core::space::{
    AccessConstraints, AddressSpace, MemAttrs, MemOps, MemResult, Region, RegionRef, RequesterId,
};
use crate::core::state::{ChunkReader, ChunkWriter, Sink, Source};
use crate::core::sync::{LockRank, Mutex};
use crate::core::value::{Endian, Width};
use crate::core::wire::{Level, WireSource};
use crate::dev::wd33c93::{ControllerPort, DmaPort};
use crate::machine::realize::{BindCtx, Instance};
use crate::machine::validate::{ClassSchema, PortDir, PropSchema};

/// The class name a machine file writes.
pub const CLASS_NAME: &str = "amiga.sdmac";

/// The snapshot chunk version. Bump with the encoding, never on its own.
const STATE_VERSION: u32 = 1;

/// The region a `map` statement places at `$DD0000`.
pub const REGS_REGION: &str = "regs";

/// How much the window decodes. The board selects the whole of `$DD0000`–
/// `$DDFFFF` for this chip; the registers live in the first `$80` of it and the
/// rest of the page repeats them, which is what a decoder with no address pin
/// below `A7` does.
pub const REGS_WINDOW_LEN: u64 = 0x1000;

/// How much of an offset the chip decodes.
const DECODE: u64 = 0x7f;

/// The interrupt output. The A3000 wires it to `_INT2`, the net Paula reports
/// as `PORTS`; the pin is named for what the chip drives rather than for which
/// net the board puts it on.
pub const INT_PIN: &str = "int";

/// The link property naming the `wd.33c93` whose two registers this chip
/// forwards to.
pub const SCSI_LINK: &str = "scsi";

// -- the register offsets (Table 2-5) ----------------------------------------

/// `DAWR`, the `DACK` width. Not in Table 2-5 — the A3000+'s DMAC dropped it —
/// but Kickstart writes `3` to `$00DD0003` before it touches anything else, so
/// the A3000's part has it where its A2091 ancestor does. Write-only: nothing
/// here has a `DACK` pulse to widen, and the value is kept only so that a write
/// does not vanish into a hole a driver can detect.
pub const DAWR: u64 = 0x00;

/// Word Transfer Count, obsolete.
pub const WTC: u64 = 0x04;
/// Control.
pub const CONTR: u64 = 0x08;
/// Address Control.
pub const ACR: u64 = 0x0c;
/// Start DMA.
pub const ST_DMA: u64 = 0x10;
/// Flush FIFO.
pub const FLUSH: u64 = 0x14;
/// Clear Interrupts.
pub const CLR_INT: u64 = 0x18;
/// Interrupt Status.
pub const ISTR: u64 = 0x1c;
/// Stop DMA.
pub const SP_DMA: u64 = 0x3c;
/// The first of the three longwords the WD33C93 is behind.
pub const SCSI_BASE: u64 = 0x40;
/// One past the last of them.
pub const SCSI_END: u64 = 0x4c;

// -- `CONTR` (Table 2-6) -----------------------------------------------------
//
// Table 2-6's second column is a **bit number**, not a mask. The `ISTR` half of
// the same table makes that unambiguous — it lists 7, 6, 5, 4, 1, 0 for six
// bits — and reading the `CONTR` half as masks instead puts `PREST` where
// `INTENA` is, so the ROM's "enable the interrupt now that the command is
// issued" becomes "reset the SCSI chip the instant it has been told to select".
// Which it is was settled by watching Kickstart: it writes `$0C`, reads back
// `$04`, writes `$00` to set up, and writes `$04` again immediately after
// putting `Select-with-ATN` in the Command register. Only one reading of the
// table makes that a sane driver.

/// DMA is enabled: set by `ST_DMA`, cleared by `SP_DMA` and by reset. Bit 8.
pub const DMAENA: u32 = 1 << 8;
/// Writing this high resets the WD33C93. Bit 4.
pub const PREST: u32 = 1 << 4;
/// The DMAC may drive its interrupt output. Bit 2.
pub const INTENA: u32 = 1 << 2;
/// High: memory → SCSI. Low: SCSI → memory. Bit 1.
pub const DMADIR: u32 = 1 << 1;

// -- `ISTR` (Table 2-6) ------------------------------------------------------

/// Interrupt follow.
pub const INT_F: u32 = 0x80;
/// Interrupt SCSI.
pub const INT_S: u32 = 0x40;
/// End of process.
pub const E_INT: u32 = 0x20;
/// Interrupt pending: the line, gated by `INTENA`.
pub const INT_P: u32 = 0x10;
/// The FIFO holds a whole longword or more.
pub const FF: u32 = 0x02;
/// The FIFO holds less than one longword.
pub const FE: u32 = 0x01;

/// Where this chip's registers sit in the ranked lock order.
///
/// Below [`crate::dev::wd33c93::CHIP_RANK`], because a register write here
/// forwards into the controller, and above nothing in particular: the DMA port
/// takes it and then writes guest memory with it released.
pub const REGS_RANK: LockRank = LockRank::new(0x4b80);

// ---------------------------------------------------------------------------
// the registers
// ---------------------------------------------------------------------------

/// Everything the DMAC holds.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct Regs {
    /// `DAWR`, which nothing reads.
    pub dawr: u32,
    /// `WTC`, kept because software reads bit 2 back to identify the part.
    pub wtc: u32,
    /// `CONTR`'s four bits.
    pub contr: u32,
    /// `ACR`, rounded down to an even word on every write.
    pub acr: u32,
}

impl Regs {
    /// Out of reset: DMA off, no interrupt, address zero.
    pub const fn reset() -> Regs {
        Regs {
            dawr: 0,
            wtc: 0,
            contr: 0,
            acr: 0,
        }
    }

    /// The longword at an aligned offset, for everything but the three the
    /// WD33C93 is behind.
    ///
    /// `interrupt` is the state of the controller's `INTRQ` line, which is what
    /// four of `ISTR`'s six bits are. `None` is an offset that is not a
    /// readable register: §2.4.1's strobes "cause a specific action to take
    /// place when read or written, regardless of the data value involved", and
    /// a strobe does not drive the data bus, so what a read of one returns is
    /// whatever the bus was carrying.
    #[must_use]
    pub const fn read_long(&self, offset: u64, interrupt: bool) -> Option<u32> {
        match offset {
            DAWR => None,
            WTC => Some(self.wtc),
            CONTR => Some(self.contr),
            ACR => Some(self.acr),
            ISTR => {
                let line = if interrupt { INT_F | INT_S | E_INT } else { 0 };
                let pending = if interrupt && self.contr & INTENA != 0 {
                    INT_P
                } else {
                    0
                };
                // The FIFO is never anything but empty here; see the module
                // documentation for why that is safe.
                Some(line | pending | FE)
            }
            _ => None,
        }
    }
}

/// Whether one **byte address** is one of the WD33C93's two, and which.
///
/// `Some(false)` is `A0` low — the Address register on a write, Auxiliary
/// Status on a read. `Some(true)` is `A0` high, the register the Address
/// register names.
///
/// # The rule, and where it comes from
///
/// The chip's two byte-wide registers sit on two **byte lanes** of the
/// DMAC's longword register block: lane 1 (`$…1`, `D23`–`D16`) is `SASR` and
/// lane 3 (`$…3`, `D7`–`D0`) is `SCMD`. Every row of Table 2-5 falls out of
/// that one rule —
///
/// | Row | Lane | Register |
/// | --- | --- | --- |
/// | `$00DD0041` `SASR_B` read | 1 | `SASR` |
/// | `$00DD0047` `SCMD_B` | 3 | `SCMD` |
/// | `$00DD0049` `SASR_B` | 1 | `SASR` |
///
/// — and so does `$00DD0043`, which the table does not list and which
/// Kickstart's `scsi.device` uses for `SCMD` on an A3000. That is the lane
/// mapping §2.4.1 calls "a little strangely … based on 68030 behavior, rather
/// than 68030 specifications, so properly designed 68040 cards could not access
/// these registers as bytes": a byte write from a 68030 drives every lane, so
/// which lane the chip is on does not matter to it, and does to a 68040.
///
/// Read as *longword* offsets instead, Table 2-5's rows disagree with what the
/// ROM does — it writes register numbers to `$…49` and data to `$…43`, so
/// `$00DD0040`–`$…43` would have to be `SASR` *and* `SCMD` at once. The lane
/// rule is what reconciles the table with the machine.
///
/// A wide access covering both lanes takes the lower one, which is the
/// table's `$00DD0040` `SASR_L` row.
#[must_use]
pub const fn scsi_lane(offset: u64) -> Option<bool> {
    if offset < SCSI_BASE || offset >= SCSI_END {
        return None;
    }
    match offset & 3 {
        1 => Some(false),
        3 => Some(true),
        _ => None,
    }
}

/// Which byte of an access the chip answers, and on which of its two
/// addresses: `(index into the access, A0 high)`.
#[must_use]
pub const fn scsi_select(offset: u64, len: usize) -> Option<(usize, bool)> {
    let mut i = 0;
    while i < len {
        if let Some(a0) = scsi_lane(offset + i as u64) {
            return Some((i, a0));
        }
        i += 1;
    }
    None
}

// ---------------------------------------------------------------------------
// the chip
// ---------------------------------------------------------------------------

/// The shared half of the device.
struct Chip {
    regs: Mutex<Regs>,
    /// The controller, once `bind` has resolved the `scsi` link.
    scsi: Mutex<Option<ControllerPort>>,
    /// The memory a transfer moves through, weakly so the space may own this
    /// device without a cycle.
    space: Mutex<Option<Weak<AddressSpace>>>,
    requester: Mutex<RequesterId>,
    int: Mutex<Option<WireSource>>,
}

impl fmt::Debug for Chip {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("Chip")
            .field("regs", &*self.regs.lock())
            .field("scsi", &self.scsi.lock().is_some())
            .finish_non_exhaustive()
    }
}

impl Chip {
    /// The controller, cloned out so nothing of this chip's is held while it
    /// runs a command.
    fn controller(&self) -> Option<ControllerPort> {
        self.scsi.lock().clone()
    }

    /// Whether the controller is asserting `INTRQ`.
    fn interrupt(&self) -> bool {
        self.controller().is_some_and(|c| c.irq_asserted())
    }

    /// Drive the interrupt pin: the controller's line, gated by `INTENA`.
    fn refresh(&self) {
        let enabled = self.regs.lock().contr & INTENA != 0;
        let high = enabled && self.interrupt();
        let source = self.int.lock().clone();
        if let Some(source) = source {
            source.set(Level::from_bool(high));
        }
    }

    /// One of the controller's two byte addresses.
    fn read_scsi(&self, a0: bool, debug: bool) -> u8 {
        // Nothing linked: the two addresses are an empty socket, and an empty
        // socket floats. Zero is what this board's undriven bus carries.
        let Some(scsi) = self.controller() else {
            return 0;
        };
        if a0 {
            scsi.read_register(debug)
        } else {
            scsi.read_aux()
        }
    }

    fn write_scsi(&self, a0: bool, byte: u8) {
        let Some(scsi) = self.controller() else {
            return;
        };
        if a0 {
            scsi.write_register(byte);
        } else {
            scsi.write_address(byte);
        }
        self.refresh();
    }

    /// One longword read of the DMAC's own registers, or `None` where nothing
    /// drives the bus.
    fn read_long(&self, offset: u64, debug: bool) -> Option<u32> {
        // Reading `ISTR` is a driver asking whether the SCSI chip wants
        // service, and it is the only thing in this model that asks. A debug
        // read asks nothing: it takes the line as it stands.
        let interrupt = if offset == ISTR && !debug {
            self.controller().is_some_and(|c| c.poll_irq())
        } else {
            self.interrupt()
        };
        self.regs.lock().read_long(offset, interrupt)
    }

    /// One longword write of the DMAC's own registers, and every strobe it can
    /// be.
    fn write_long(&self, offset: u64, value: u32) {
        match offset {
            DAWR => self.regs.lock().dawr = value,
            WTC => self.regs.lock().wtc = value,
            CONTR => {
                let reset = value & PREST != 0;
                {
                    let mut regs = self.regs.lock();
                    // `DMAENA` is not a bit software sets: §2.4.1 says the
                    // strobes own it. Everything else it writes lands.
                    let held = regs.contr & DMAENA;
                    regs.contr = (value & (PREST | INTENA | DMADIR)) | held;
                }
                if reset && let Some(scsi) = self.controller() {
                    scsi.master_reset();
                }
                self.refresh();
            }
            // §2.4.1: "the ACR … will actually round any value written to it
            // down to an even word, since the DMAC doesn't support odd-byte
            // aligned transfers".
            ACR => self.regs.lock().acr = value & !1,
            ST_DMA => self.regs.lock().contr |= DMAENA,
            SP_DMA => {
                let mut regs = self.regs.lock();
                regs.contr &= !DMAENA;
            }
            // Nothing to flush: every byte reached memory in the call that
            // carried it. See the module documentation.
            FLUSH => {}
            CLR_INT => {
                // §2.4.1: the strobe "clears all interrupts registered by ISTR,
                // and negates the DMAC's interrupt output line". Every bit of
                // ISTR is a live view of the WD33C93's own line, which only
                // that chip can clear — a host does it by reading the SCSI
                // Status register — so what is left for this strobe to do is
                // the second half of the sentence, and it is done by driving
                // the pin again rather than by latching anything.
                self.refresh();
            }
            ISTR => {}
            _ => {}
        }
    }
}

/// The DMAC as the controller's data path.
#[derive(Debug)]
struct Port(Arc<Chip>);

impl Port {
    /// The space and the address a transfer is at, if DMA is running in the
    /// direction asked for.
    fn ready(&self, memory_to_scsi: bool) -> Option<(Arc<AddressSpace>, u64, MemAttrs)> {
        let (contr, acr) = {
            let regs = self.0.regs.lock();
            (regs.contr, regs.acr)
        };
        if contr & DMAENA == 0 {
            return None;
        }
        if (contr & DMADIR != 0) != memory_to_scsi {
            return None;
        }
        let space = self.0.space.lock().clone()?.upgrade()?;
        let requester = *self.0.requester.lock();
        Some((
            space,
            u64::from(acr),
            MemAttrs::DEFAULT.with_requester(requester),
        ))
    }

    /// Move the address on by `n`.
    fn advance(&self, n: usize) {
        let mut regs = self.0.regs.lock();
        regs.acr = regs.acr.wrapping_add(n as u32);
    }
}

impl DmaPort for Port {
    fn fetch(&self, dst: &mut [u8]) -> usize {
        let Some((space, at, attrs)) = self.ready(true) else {
            return 0;
        };
        if space.read_bytes(at, dst, attrs).is_err() {
            return 0;
        }
        self.advance(dst.len());
        dst.len()
    }

    fn store(&self, src: &[u8]) -> usize {
        let Some((space, at, attrs)) = self.ready(false) else {
            return 0;
        };
        if space.write_bytes(at, src, attrs).is_err() {
            return 0;
        }
        self.advance(src.len());
        src.len()
    }
}

// ---------------------------------------------------------------------------
// the window
// ---------------------------------------------------------------------------

/// `$DD0000`–`$DD007F`, repeating through the page.
#[derive(Debug)]
struct RegsWindow(Arc<Chip>);

/// Which aligned longword an offset is in, and where in it the access starts.
const fn split(offset: u64) -> (u64, usize) {
    let base = offset & !3;
    (base & DECODE & !3, (offset & 3) as usize)
}

impl MemOps for RegsWindow {
    fn read(&self, offset: u64, dst: &mut [u8], attrs: MemAttrs) -> MemResult {
        let at = offset & DECODE;
        if let Some((i, a0)) = scsi_select(at, dst.len()) {
            // One chip access, whatever the width: the controller is a
            // byte-wide part on one lane, and the lanes it is not on float.
            for byte in dst.iter_mut() {
                *byte = attrs.bus;
            }
            dst[i] = self.0.read_scsi(a0, attrs.debug);
            if !attrs.debug {
                self.0.refresh();
            }
            return Ok(());
        }
        let (base, within) = split(offset);
        if within + dst.len() > 4 {
            return Err(BusError::BadAccess);
        }
        match self.0.read_long(base, attrs.debug) {
            Some(value) => dst.copy_from_slice(&value.to_be_bytes()[within..within + dst.len()]),
            None => dst.fill(attrs.bus),
        }
        if !attrs.debug {
            self.0.refresh();
        }
        Ok(())
    }

    fn write(&self, offset: u64, src: &[u8], attrs: MemAttrs) -> MemResult {
        if attrs.debug {
            // Four of these offsets are strobes that act "regardless of the
            // data value involved" (§2.4.1) and two pass a byte to a chip that
            // will run a SCSI command with it. None can be made harmless
            // (`ROADMAP.md` §15, invariant 5).
            return Err(BusError::BadAccess);
        }
        let at = offset & DECODE;
        if let Some((i, a0)) = scsi_select(at, src.len()) {
            self.0.write_scsi(a0, src[i]);
            return Ok(());
        }
        let (base, within) = split(offset);
        if within + src.len() > 4 {
            return Err(BusError::BadAccess);
        }
        // §2.4: "it is impossible to independently access individual words or
        // bytes within these registers … any writes to DMAC registers must
        // write all significant bits". A narrow write therefore splices into
        // what is there rather than being refused: the bits it did not carry
        // are whatever the chip already had, which is the closest thing to
        // "undefined" that a deterministic model can offer.
        let mut value = self.0.read_long(base, true).unwrap_or(0).to_be_bytes();
        value[within..within + src.len()].copy_from_slice(src);
        self.0.write_long(base, u32::from_be_bytes(value));
        self.0.refresh();
        Ok(())
    }

    fn constraints(&self) -> AccessConstraints {
        AccessConstraints {
            min: Width::U8,
            natural_alignment: false,
            ..AccessConstraints::word(Width::U32, Endian::Big)
        }
    }
}

// ---------------------------------------------------------------------------
// the device
// ---------------------------------------------------------------------------

/// The A3000's DMA controller.
#[derive(Debug)]
pub struct Sdmac {
    chip: Arc<Chip>,
    regs: RegionRef,
    /// What `scsi` names, resolved at bind.
    scsi_path: Option<String>,
}

impl Sdmac {
    /// Validate `props` and build the chip.
    ///
    /// # Errors
    ///
    /// [`Error::Property`] if a property is of the wrong kind or unknown.
    pub fn new(props: &Props) -> Result<Sdmac> {
        let mut r = props.reader();
        let scsi = r.optional_link(SCSI_LINK)?.map(|l| l.as_str().to_string());
        r.finish()?;
        Ok(Sdmac::with_link(scsi))
    }

    /// Build one that will be told about its controller later.
    #[must_use]
    pub fn with_link(scsi_path: Option<String>) -> Sdmac {
        let chip = Arc::new(Chip {
            regs: Mutex::with_rank(REGS_RANK, Regs::reset()),
            scsi: Mutex::with_rank(LockRank::LEAF, None),
            space: Mutex::with_rank(LockRank::LEAF, None),
            requester: Mutex::with_rank(LockRank::LEAF, RequesterId(0)),
            int: Mutex::with_rank(LockRank::LEAF, None),
        });
        Sdmac {
            regs: Arc::new(Region::io(
                format!("{CLASS_NAME}.{REGS_REGION}"),
                REGS_WINDOW_LEN,
                Arc::new(RegsWindow(Arc::clone(&chip))),
            )),
            chip,
            scsi_path,
        }
    }

    /// Give the chip its controller, and give the controller this chip's data
    /// path — what `bind` does with the `scsi` link.
    pub fn attach_scsi(&self, port: ControllerPort) {
        port.attach_dma(Arc::new(Port(Arc::clone(&self.chip))) as Arc<dyn DmaPort>);
        *self.chip.scsi.lock() = Some(port);
    }

    /// Give the chip the memory a transfer moves through.
    pub fn attach_space(&self, space: &Arc<AddressSpace>, requester: RequesterId) {
        *self.chip.space.lock() = Some(Arc::downgrade(space));
        *self.chip.requester.lock() = requester;
    }

    /// The registers as they stand.
    #[must_use]
    pub fn regs(&self) -> Regs {
        *self.chip.regs.lock()
    }

    /// `ISTR` as a read would return it, without the read's pin refresh.
    #[must_use]
    pub fn istr(&self) -> u32 {
        let interrupt = self.chip.interrupt();
        self.chip
            .regs
            .lock()
            .read_long(ISTR, interrupt)
            .unwrap_or(0)
    }

    /// Whether the chip is driving its interrupt output.
    #[must_use]
    pub fn interrupting(&self) -> bool {
        self.chip.regs.lock().contr & INTENA != 0 && self.chip.interrupt()
    }
}

/// The `amiga.sdmac` device class.
pub static CLASS: DeviceClass = DeviceClass {
    name: CLASS_NAME,
    version: STATE_VERSION,
    summary: "the A3000's Super DMAC: the SCSI data path into 32-bit memory, and the two \
              addresses the WD33C93A sits behind",
    properties: &[PropertySpec {
        name: SCSI_LINK,
        kind: ValueKind::Link,
        required: false,
        summary: "the `wd.33c93` whose two registers this chip's window forwards to",
    }],
    construct: |props| Ok(Box::new(Sdmac::new(props)?)),
};

impl Device for Sdmac {
    fn class(&self) -> &'static DeviceClass {
        &CLASS
    }

    fn realize(&self, _ctx: &mut RealizeCtx<'_>) -> Result<()> {
        // Nothing outward: `map` places the window and the wire graph brings
        // the pin.
        Ok(())
    }

    fn reset(&self, _kind: ResetKind) {
        // Both kinds. "Reset or the SP_DMA strobe cause this to go low" (§2.4.1,
        // of DMAENA); the whole register file goes with it. The controller is
        // its own device and resets itself.
        *self.chip.regs.lock() = Regs::reset();
        self.chip.refresh();
    }

    fn region(&self, name: &str) -> Option<RegionRef> {
        match name {
            "" | REGS_REGION => Some(Arc::clone(&self.regs)),
            _ => None,
        }
    }

    fn connect(&self, port: &str, source: WireSource) -> Result<()> {
        if port != INT_PIN {
            return Err(Error::Config {
                at: port.to_string(),
                message: String::from("the Super DMAC drives one pin: `int`"),
            });
        }
        *self.chip.int.lock() = Some(source);
        Ok(())
    }

    fn announce(&self, port: &str) {
        if port == INT_PIN {
            self.chip.refresh();
        }
    }

    fn save(&self, w: &mut ChunkWriter<'_>) -> Result<()> {
        let regs = *self.chip.regs.lock();
        w.write_u32(regs.dawr)?;
        w.write_u32(regs.wtc)?;
        w.write_u32(regs.contr)?;
        w.write_u32(regs.acr)?;
        Ok(())
    }

    fn load(&self, r: &mut ChunkReader<'_>) -> Result<()> {
        let regs = Regs {
            dawr: r.read_u32()?,
            wtc: r.read_u32()?,
            contr: r.read_u32()? & (DMAENA | PREST | INTENA | DMADIR),
            acr: r.read_u32()? & !1,
        };
        *self.chip.regs.lock() = regs;
        // A restore does not re-run the wire graph; put the pin where the
        // registers and the controller's line say.
        self.chip.refresh();
        Ok(())
    }
}

/// The machine layer's half: the chip has to be told which controller is behind
/// its two addresses, and which memory a transfer moves through.
impl Instance for Sdmac {
    fn bind(&self, ctx: &BindCtx<'_>) -> Result<()> {
        if let Some(space) = ctx.space() {
            self.attach_space(space, ctx.requester());
        }
        let Some(path) = &self.scsi_path else {
            return Ok(());
        };
        let port = ctx
            .export_as::<ControllerPort>(path, ExportId::SCSI_CONTROLLER)
            .map_err(|e| Error::Config {
                at: ctx.path().to_string(),
                message: format!("`{SCSI_LINK}` has to name a `wd.33c93`: {e}"),
            })?;
        self.attach_scsi((*port).clone());
        Ok(())
    }
}

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
    bindings.bind(CLASS_NAME, |props| Ok(Arc::new(Sdmac::new(props)?)))
}

/// What the validator should know about `amiga.sdmac`.
#[must_use]
pub fn schema() -> ClassSchema {
    ClassSchema::new(CLASS_NAME)
        .prop(PropSchema::new(SCSI_LINK, ValueKind::Link))
        .region("")
        .region(REGS_REGION)
        .port(INT_PIN, PortDir::Out)
}
