//! The ACPI register block an ICH9 decodes at `PMBASE`.
//!
//! # Why this is not its own device
//!
//! Because on the part it is not. The ICH9 datasheet opens chapter 13 by saying
//! so:
//!
//! > The LPC bridge function of the ICH9 resides in PCI Device 31:Function 0.
//! > This function contains many other functional units, such as DMA and
//! > Interrupt controllers, Timers, Power Management, System Management, GPIO,
//! > RTC, and LPC Configuration Registers.
//!
//! So the power-management registers *are* the LPC function's, reached through
//! the base address that function's `PMBASE` register holds. [`super::lpc`] is
//! the device; this file is the register block it decodes, and the split is one
//! of files rather than of objects (`CLAUDE.md`: a device is one file until it
//! genuinely isn't — and a 128-byte register block with a counter in it is
//! where "isn't" begins).
//!
//! # What is modelled
//!
//! The four registers ACPI names in the FADT, from ICH9 Table 13-11
//! (`ACPI and Legacy I/O Register Map`):
//!
//! ```text
//!   PMBASE + 00h  PM1_STS   PM1 Status    PM1a_EVT_BLK      R/WC   16-bit
//!   PMBASE + 02h  PM1_EN    PM1 Enable    PM1a_EVT_BLK + 2  R/W    16-bit
//!   PMBASE + 04h  PM1_CNT   PM1 Control   PM1a_CNT_BLK      R/W    32-bit
//!   PMBASE + 08h  PM1_TMR   PM1 Timer     PMTMR_BLK         RO     32-bit
//!   PMBASE + 20h  GPE0_STS  GP Event 0    GPE0_BLK          R/WC   64-bit
//!   PMBASE + 28h  GPE0_EN   GP Event 0    GPE0_BLK + 8      R/W    64-bit
//! ```
//!
//! `GPE0_STS`/`GPE0_EN` are here as *storage* and nothing else: no event source
//! in this tree drives a general purpose event, so the bits read back what was
//! written and never set themselves. They exist because the FADT has to name a
//! `GPE0_BLK` and an operating system reads it during initialisation; a block
//! that master-aborted would be worse than one that is empty.
//!
//! Everything else in Table 13-11 — `PROC_CNT`, the `LVn` registers, `SMI_EN`,
//! `SMI_STS`, `ALT_GP_SMI_*`, `DEVACT_STS`, the TCO block — reads as zero and
//! discards writes. That is the datasheet's own rule for the reserved parts of
//! this window ("All reserved bits and registers will always return 0 when
//! read, and will have no effect when written"), extended to the registers that
//! are real on the part and inert on this board.
//!
//! # The timer, which is the interesting part
//!
//! §13.8.3.4, `PM1_TMR`:
//!
//! > Timer Value (TMR_VAL) — RO. Returns the running count of the PM timer.
//! > This counter runs off a 3.579545 MHz clock (14.31818 MHz divided by 4). It
//! > is reset to 0 during a PCI reset […]
//! >
//! > Anytime bit 22 of the timer goes HIGH to LOW (bits referenced from 0 to
//! > 23), the TMROF_STS bit (PMBASE + 00h, bit 0) is set. The High-to-Low
//! > transition will occur every 2.3435 seconds. If the TMROF_EN bit (PMBASE +
//! > 02h, bit 0) is set, an SCI interrupt is also generated.
//!
//! Twenty-four bits, and bit 22's high-to-low transitions are exactly the
//! instants bit 23 changes, so the rule is "every 2²³ counts" whichever way it
//! is written. (§13.8.3.1 states the same bit as *going high*, which is the
//! same period out of phase; §13.8.3.4 is the more specific of the two and is
//! what is implemented.)
//!
//! The counter is a **lazily advanced** device in its own clock domain
//! (`ROADMAP.md` §4.2): one domain tick is one count, the domain's rate is the
//! machine file's business, and nothing here divides by a frequency — which is
//! the rule that keeps floats out of the time path. Its `next_event_tick` is
//! the next 2²³ boundary, so the scheduler stops there and the status bit is
//! set on the tick it belongs to rather than whenever somebody next looks.
//!
//! # What is deliberately not done
//!
//! **Sleep.** `SLP_TYP` and `SLP_EN` are modelled as registers — `SLP_EN` is
//! write-only and reads back zero, as §13.8.3.3 says — and writing
//! `SLP_TYP = 111b` with `SLP_EN` does **not** switch the machine off. It
//! cannot: the seam a guest's power request lands on is
//! [`crate::dev::riscv::syscon`]'s `signals` host object, which is behind a
//! feature this board has nothing to do with. Moving that seam somewhere both
//! boards can reach — `core::hosts` is the obvious place, beside the character
//! ports — is a small change to a file this work does not own, and it is the
//! one thing standing between this register and a working `S5`.
//!
//! # Sources
//!
//! *Intel I/O Controller Hub 9 (ICH9) Family Datasheet*, order number
//! 316972-004: chapter 13's opening for what the LPC function contains, Table
//! 13-11 for the register map, §13.8.3.1-§13.8.3.4 for `PM1_STS`, `PM1_EN`,
//! `PM1_CNT` and `PM1_TMR`. *ACPI Specification* §4.8.3 for what the fixed
//! hardware registers mean to an operating system.
//!
//! No emulator source was consulted (`CLAUDE.md`, provenance).

use alloc::sync::Arc;
use alloc::sync::Weak;
use core::fmt;
use core::sync::atomic::{AtomicU64, Ordering};

use crate::core::error::BusError;
use crate::core::sched::{AccessKind, LazyHandle};
use crate::core::space::{AccessConstraints, MemAttrs, MemOps, MemResult};
use crate::core::sync::{LockRank, Mutex};
use crate::core::value::{Endian, Width};

/// How much I/O space `PMBASE` claims: 128 bytes, on a 128-byte boundary
/// (§13.1.13).
pub const BLOCK_LEN: u64 = 128;

/// `PM1_STS`, and the FADT's `PM1a_EVT_BLK`.
pub const PM1_STS: u64 = 0x00;
/// `PM1_EN`, the second half of `PM1a_EVT_BLK`.
pub const PM1_EN: u64 = 0x02;
/// `PM1_CNT`, and the FADT's `PM1a_CNT_BLK`.
pub const PM1_CNT: u64 = 0x04;
/// `PM1_TMR`, and the FADT's `PMTMR_BLK`.
pub const PM1_TMR: u64 = 0x08;
/// `GPE0_STS`, and the FADT's `GPE0_BLK`.
pub const GPE0_STS: u64 = 0x20;
/// `GPE0_EN`, the second half of `GPE0_BLK`.
pub const GPE0_EN: u64 = 0x28;

/// How many bytes `PM1a_EVT_BLK` covers: status and enable together.
pub const PM1_EVT_LEN: u8 = 4;
/// How many bytes `PM1a_CNT_BLK` covers.
pub const PM1_CNT_LEN: u8 = 2;
/// How many bytes `PMTMR_BLK` covers.
pub const PM_TMR_LEN: u8 = 4;
/// How many bytes `GPE0_BLK` covers: status and enable together, eight each.
pub const GPE0_BLK_LEN: u8 = 16;

/// `PM1_STS[0]` / `PM1_EN[0]`: the timer overflowed (§13.8.3.1).
pub const TMROF: u16 = 1 << 0;
/// `PM1_STS[5]` / `PM1_EN[5]`: BIOS was asked for something (`GBL_STS`).
pub const GBL: u16 = 1 << 5;
/// `PM1_STS[8]` / `PM1_EN[8]`: the power button.
pub const PWRBTN: u16 = 1 << 8;
/// `PM1_STS[10]` / `PM1_EN[10]`: the RTC alarm.
pub const RTC: u16 = 1 << 10;

/// Every `PM1_STS` bit this block implements. The rest are wake and
/// PCI-Express status this board has no source for, and read as zero.
const PM1_STS_MASK: u16 = TMROF | GBL | PWRBTN | RTC;

/// Every `PM1_EN` bit this block implements.
const PM1_EN_MASK: u16 = TMROF | GBL | PWRBTN | RTC;

/// `PM1_CNT[0]`: events raise an SCI rather than an SMI (§13.8.3.3).
pub const SCI_EN: u32 = 1 << 0;
/// `PM1_CNT[12:10]`: which sleep state `SLP_EN` asks for.
pub const SLP_TYP: u32 = 0b111 << 10;
/// `PM1_CNT[13]`: go there. Write-only; reads as zero.
pub const SLP_EN: u32 = 1 << 13;

/// Every `PM1_CNT` bit a write keeps. `SLP_EN` and `GBL_RLS` are write-only and
/// are handled by hand.
const PM1_CNT_MASK: u32 = SCI_EN | SLP_TYP;

/// How wide the timer is: twenty-four bits (§13.8.3.4).
pub const TIMER_BITS: u32 = 24;

/// The counter's modulus.
const TIMER_MODULUS: u64 = 1 << TIMER_BITS;

/// How many counts between two settings of `TMROF_STS`.
///
/// Bit 22 goes high to low every 2²³ counts, which at 3.579545 MHz is the
/// 2.3435 seconds §13.8.3.4 quotes.
const TMROF_PERIOD: u64 = 1 << 23;

/// Who the block tells when its `SCI` may have changed level.
///
/// The bridge, and only the bridge: `ACPI_CNTL[2:0]` picks which of five
/// interrupt lines the SCI internally appears on, and that register belongs to
/// [`super::lpc`] rather than to this block. Held weakly, because the bridge
/// owns the block.
pub trait SciSink: Send + Sync {
    /// The block's `SCI` condition may have moved; re-drive the pins.
    ///
    /// Called with no lock of the block's held, so an implementation is free to
    /// take its own and to drive a wire.
    fn sci_changed(&self);
}

/// The architectural state of the block.
#[derive(Debug, Clone, Copy, Default)]
struct State {
    /// `PM1_STS`. Write-one-to-clear.
    sts: u16,
    /// `PM1_EN`.
    en: u16,
    /// `PM1_CNT`, minus its write-only bits.
    cnt: u32,
    /// `GPE0_STS`. Storage: nothing on this board raises a general purpose
    /// event.
    gpe_sts: u64,
    /// `GPE0_EN`. Storage, for the same reason.
    gpe_en: u64,
    /// The last sleep request the guest made, as `(SLP_TYP, seen)`.
    ///
    /// Kept so a test — and one day a power seam — can see that the guest asked
    /// for `S5`, rather than the request vanishing into a write-only bit. It is
    /// guest-visible in exactly the sense a latched request is, so it is saved.
    slp_request: Option<u8>,
}

/// The ACPI register block: the four fixed registers, the counter and the SCI.
pub struct AcpiBlock {
    /// [`LockRank::DEVICE`], released before the SCI pin is driven.
    state: Mutex<State>,
    /// The tick of this block's own clock domain the counter stands at. An
    /// atomic because [`crate::core::device::Device::current_tick`] may not
    /// take a lock.
    tick: AtomicU64,
    /// The next tick `TMROF_STS` is due, published for the scheduler. Also
    /// lock-free, for the same reason.
    next_event: AtomicU64,
    /// Who to tell when the SCI's level may have moved. [`LockRank::LEAF`],
    /// cloned out and released before the call — the SCI is a pin on the
    /// *bridge*, because which of five interrupts it appears on is
    /// `ACPI_CNTL`'s business and not this block's.
    sink: Mutex<Option<Weak<dyn SciSink>>>,
    /// The handle that catches this block up before an access reaches it.
    lazy: Mutex<Option<LazyHandle>>,
}

impl fmt::Debug for AcpiBlock {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let mut s = f.debug_struct("AcpiBlock");
        match self.state.try_lock() {
            Some(state) => s.field("state", &*state),
            None => s.field("state", &"<in use>"),
        };
        s.field("tick", &self.tick.load(Ordering::Relaxed)).finish()
    }
}

impl Default for AcpiBlock {
    fn default() -> AcpiBlock {
        AcpiBlock::new()
    }
}

impl AcpiBlock {
    /// A block out of reset: every register zero and the counter at zero.
    #[must_use]
    pub fn new() -> AcpiBlock {
        let block = AcpiBlock {
            state: Mutex::with_rank(LockRank::DEVICE, State::default()),
            tick: AtomicU64::new(0),
            next_event: AtomicU64::new(TMROF_PERIOD),
            sink: Mutex::with_rank(LockRank::LEAF, None),
            lazy: Mutex::with_rank(LockRank::LEAF, None),
        };
        block.publish();
        block
    }

    /// The counter, as `PM1_TMR` reports it.
    #[must_use]
    pub fn timer(&self) -> u32 {
        (self.tick.load(Ordering::Relaxed) % TIMER_MODULUS) as u32
    }

    /// `PM1_STS`.
    #[must_use]
    pub fn status(&self) -> u16 {
        self.state.lock().sts
    }

    /// `PM1_CNT`.
    #[must_use]
    pub fn control(&self) -> u32 {
        self.state.lock().cnt
    }

    /// The sleep state the guest last asked for with `SLP_EN`, if it has.
    ///
    /// Nothing acts on it — see the module docs — so this is how a test asks
    /// whether the guest tried.
    #[must_use]
    pub fn sleep_request(&self) -> Option<u8> {
        self.state.lock().slp_request
    }

    /// Install who is told when the SCI's level may have moved.
    pub fn set_sink(&self, sink: Weak<dyn SciSink>) {
        *self.sink.lock() = Some(sink);
    }

    /// Whether an enabled status bit is asking for an SCI right now.
    ///
    /// §13.8.3.3: `SCI_EN` "Selects the SCI interrupt or the SMI# interrupt for
    /// various events including the bits in the PM1_STS register". This board
    /// has no SMI path, so with `SCI_EN` clear an enabled event raises nothing
    /// — which is what a machine whose firmware has not enabled ACPI looks
    /// like, and is why this is not simply the OR of status and enable.
    #[must_use]
    pub fn sci_asserted(&self) -> bool {
        let s = self.state.lock();
        s.cnt & SCI_EN != 0 && s.sts & s.en & PM1_STS_MASK != 0
    }

    /// Install the handle that catches this block up before an access.
    pub fn set_lazy(&self, handle: LazyHandle) {
        *self.lazy.lock() = Some(handle);
    }

    /// The tick the counter stands at.
    #[must_use]
    pub fn tick(&self) -> u64 {
        self.tick.load(Ordering::Relaxed)
    }

    /// The next tick the scheduler must not run past.
    #[must_use]
    pub fn next_event_tick(&self) -> Option<u64> {
        Some(self.next_event.load(Ordering::Relaxed))
    }

    /// Publish where the next `TMROF_STS` falls, for the scheduler.
    fn publish(&self) {
        let now = self.tick.load(Ordering::Relaxed);
        // Strictly greater than `now`, or catch-up makes no progress.
        let next = (now / TMROF_PERIOD + 1) * TMROF_PERIOD;
        self.next_event.store(next, Ordering::Relaxed);
    }

    /// Everything reset does to this block.
    ///
    /// The counter goes to zero: §13.8.3.4 says it "is reset to 0 during a PCI
    /// reset", and rsemu's warm reset is what stands in for `PCIRST#` on this
    /// board.
    pub fn reset(&self) {
        *self.state.lock() = State::default();
        self.tick.store(0, Ordering::Relaxed);
        self.publish();
        self.drive();
    }

    /// Advance the counter to `target` of this block's own clock domain.
    pub fn advance_to(&self, target: u64) {
        let now = self.tick.load(Ordering::Relaxed);
        if target <= now {
            // Running backwards is a no-op, not an error.
            return;
        }
        // How many 2²³ boundaries were crossed. One is enough to set the bit;
        // the bit does not count.
        let crossed = target / TMROF_PERIOD > now / TMROF_PERIOD;
        self.tick.store(target, Ordering::Relaxed);
        self.publish();
        if crossed {
            let mut state = self.state.lock();
            state.sts |= TMROF;
            drop(state);
            self.drive();
        }
    }

    /// Tell whoever is listening that the SCI's level may have moved, with no
    /// lock held.
    fn drive(&self) {
        let sink = self.sink.lock().clone();
        if let Some(sink) = sink.as_ref().and_then(Weak::upgrade) {
            sink.sci_changed();
        }
    }

    /// Catch the counter up before an access is dispatched to it (§4.2).
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
        // A refusal means catch-up is already running further up the stack; the
        // access is still answered from where the block stands.
        let _ = handle.sync(kind);
    }

    /// The 32 bits a read of the dword containing `offset` sees.
    fn dword(&self, aligned: u64) -> u32 {
        let state = self.state.lock();
        match aligned {
            PM1_STS => u32::from(state.sts) | (u32::from(state.en) << 16),
            PM1_CNT => state.cnt,
            PM1_TMR => self.timer(),
            GPE0_STS => state.gpe_sts as u32,
            v if v == GPE0_STS + 4 => (state.gpe_sts >> 32) as u32,
            GPE0_EN => state.gpe_en as u32,
            v if v == GPE0_EN + 4 => (state.gpe_en >> 32) as u32,
            // Everything else in the 128-byte window: "All reserved bits and
            // registers will always return 0 when read" (§13.8.3).
            _ => 0,
        }
    }

    /// Take a write of `value` to the dword at `aligned`, with `mask` marking
    /// the bytes the access actually covered. Reports whether the SCI may have
    /// moved.
    fn write_dword(&self, aligned: u64, value: u32, mask: u32) -> bool {
        let mut state = self.state.lock();
        match aligned {
            PM1_STS => {
                // Write-one-to-clear on the status half, ordinary R/W on the
                // enable half, and both live in one dword because that is how
                // the part decodes them.
                let sts = (value & mask) as u16;
                state.sts &= !(sts & PM1_STS_MASK);
                let en_mask = (mask >> 16) as u16;
                let en = (value >> 16) as u16;
                state.en = (state.en & !en_mask) | (en & en_mask & PM1_EN_MASK);
                true
            }
            PM1_CNT => {
                state.cnt = (state.cnt & !(mask & PM1_CNT_MASK)) | (value & mask & PM1_CNT_MASK);
                if value & mask & SLP_EN != 0 {
                    // §13.8.3.3: "Setting this bit causes the system to
                    // sequence into the Sleep state defined by the SLP_TYP
                    // field." Latched rather than acted on — see the module
                    // docs — and `SLP_EN` itself reads back as zero because it
                    // is write-only.
                    state.slp_request = Some(((state.cnt & SLP_TYP) >> 10) as u8);
                }
                true
            }
            GPE0_STS => {
                let clear = u64::from(value & mask);
                state.gpe_sts &= !clear;
                false
            }
            v if v == GPE0_STS + 4 => {
                state.gpe_sts &= !(u64::from(value & mask) << 32);
                false
            }
            GPE0_EN => {
                state.gpe_en = (state.gpe_en & !u64::from(mask)) | u64::from(value & mask);
                false
            }
            v if v == GPE0_EN + 4 => {
                let m = u64::from(mask) << 32;
                state.gpe_en = (state.gpe_en & !m) | (u64::from(value & mask) << 32);
                false
            }
            // Reserved: "will have no effect when written" (§13.8.3).
            _ => false,
        }
    }

    /// Everything a snapshot has to carry.
    #[must_use]
    pub fn save_state(&self) -> [u64; 6] {
        let s = *self.state.lock();
        [
            u64::from(s.sts),
            u64::from(s.en),
            u64::from(s.cnt),
            s.gpe_sts,
            s.gpe_en,
            // `None` and `Some(n)` in one word: 0 is none, `n + 1` is a
            // request. `SLP_TYP` is three bits, so nothing is lost.
            s.slp_request.map_or(0, |t| u64::from(t) + 1),
        ]
    }

    /// Restore from [`save_state`](AcpiBlock::save_state), masked exactly as a
    /// guest write is.
    pub fn load_state(&self, words: [u64; 6], tick: u64) {
        {
            let mut s = self.state.lock();
            s.sts = words[0] as u16 & PM1_STS_MASK;
            s.en = words[1] as u16 & PM1_EN_MASK;
            s.cnt = words[2] as u32 & PM1_CNT_MASK;
            s.gpe_sts = words[3];
            s.gpe_en = words[4];
            s.slp_request = (words[5] > 0).then(|| ((words[5] - 1) & 0x7) as u8);
        }
        self.tick.store(tick, Ordering::Relaxed);
        self.publish();
        self.drive();
    }
}

impl MemOps for AcpiBlock {
    fn read(&self, offset: u64, dst: &mut [u8], attrs: MemAttrs) -> MemResult {
        if offset.saturating_add(dst.len() as u64) > BLOCK_LEN {
            return Err(BusError::BadAccess);
        }
        // The counter has to be current before it is read, and a debug read
        // must not move the machine's clock — `LazyHandle` is told which this
        // is and decides.
        self.sync(attrs);
        let aligned = offset & !3;
        if offset - aligned + dst.len() as u64 > 4 {
            // An access straddling two dwords is not one this decode can
            // express, and no `in` instruction issues one.
            return Err(BusError::BadAccess);
        }
        let word = self.dword(aligned).to_le_bytes();
        for (i, slot) in dst.iter_mut().enumerate() {
            *slot = word[(offset - aligned) as usize + i];
        }
        Ok(())
    }

    fn write(&self, offset: u64, src: &[u8], attrs: MemAttrs) -> MemResult {
        if offset.saturating_add(src.len() as u64) > BLOCK_LEN {
            return Err(BusError::BadAccess);
        }
        if attrs.debug {
            // Every register here has a side effect a debugger must not cause:
            // a write to `PM1_STS` clears a status bit, and one to `PM1_CNT`
            // asks the machine to go to sleep.
            return Err(BusError::BadAccess);
        }
        self.sync(attrs);
        let aligned = offset & !3;
        let first = offset - aligned;
        if first + src.len() as u64 > 4 {
            return Err(BusError::BadAccess);
        }
        let mut value = [0u8; 4];
        let mut mask = [0u8; 4];
        for (i, byte) in src.iter().enumerate() {
            value[first as usize + i] = *byte;
            mask[first as usize + i] = 0xff;
        }
        let moved = self.write_dword(aligned, u32::from_le_bytes(value), u32::from_le_bytes(mask));
        if moved {
            self.drive();
        }
        Ok(())
    }

    fn constraints(&self) -> AccessConstraints {
        // Byte, word and dword, little-endian: an operating system reads
        // `PM1_TMR` as a dword and `PM1_STS` as a word, and firmware pokes
        // single bytes.
        AccessConstraints::IO
            .with_widths(Width::U8, Width::U32)
            .with_endian(Endian::Little)
    }
}

/// A block behind an `Arc`, which is what the LPC holds.
#[must_use]
pub fn block() -> Arc<AcpiBlock> {
    Arc::new(AcpiBlock::new())
}

#[cfg(test)]
mod tests {
    use super::*;

    fn read32(block: &AcpiBlock, offset: u64) -> u32 {
        let mut buf = [0u8; 4];
        block
            .read(offset, &mut buf, MemAttrs::default())
            .expect("in range");
        u32::from_le_bytes(buf)
    }

    fn write32(block: &AcpiBlock, offset: u64, value: u32) {
        block
            .write(offset, &value.to_le_bytes(), MemAttrs::default())
            .expect("in range");
    }

    #[test]
    fn the_timer_counts_its_own_domain_one_for_one() {
        let block = AcpiBlock::new();
        assert_eq!(read32(&block, PM1_TMR), 0);
        block.advance_to(12_345);
        assert_eq!(read32(&block, PM1_TMR), 12_345);
        // Twenty-four bits, and no more (§13.8.3.4).
        block.advance_to(TIMER_MODULUS + 7);
        assert_eq!(read32(&block, PM1_TMR), 7);
    }

    #[test]
    fn the_timer_never_runs_backwards() {
        let block = AcpiBlock::new();
        block.advance_to(1_000);
        block.advance_to(500);
        assert_eq!(block.tick(), 1_000);
    }

    #[test]
    fn tmrof_sts_is_set_every_two_to_the_twentythird() {
        let block = AcpiBlock::new();
        assert_eq!(
            block.next_event_tick(),
            Some(TMROF_PERIOD),
            "the scheduler is told where the bit is due"
        );
        block.advance_to(TMROF_PERIOD - 1);
        assert_eq!(block.status() & TMROF, 0);
        block.advance_to(TMROF_PERIOD);
        assert_eq!(block.status() & TMROF, TMROF);
        assert_eq!(block.next_event_tick(), Some(TMROF_PERIOD * 2));
    }

    #[test]
    fn a_status_bit_is_cleared_by_writing_one_to_it() {
        let block = AcpiBlock::new();
        block.advance_to(TMROF_PERIOD);
        assert_eq!(block.status() & TMROF, TMROF);
        // Writing zero changes nothing: this is write-one-to-clear.
        write32(&block, PM1_STS, 0);
        assert_eq!(block.status() & TMROF, TMROF);
        write32(&block, PM1_STS, u32::from(TMROF));
        assert_eq!(block.status() & TMROF, 0);
    }

    #[test]
    fn the_enable_half_shares_the_status_registers_dword() {
        let block = AcpiBlock::new();
        // A word write at PMBASE+2 is how a driver sets PM1_EN.
        block
            .write(PM1_EN, &TMROF.to_le_bytes(), MemAttrs::default())
            .expect("in range");
        let both = read32(&block, PM1_STS);
        assert_eq!(both >> 16, u32::from(TMROF));
        assert_eq!(both & 0xffff, 0, "no status bit was set by enabling one");
    }

    #[test]
    fn the_sci_needs_status_and_enable_and_sci_en() {
        let block = AcpiBlock::new();
        block.advance_to(TMROF_PERIOD);
        assert!(!block.sci_asserted(), "status alone is not an interrupt");
        block
            .write(PM1_EN, &TMROF.to_le_bytes(), MemAttrs::default())
            .expect("in range");
        assert!(
            !block.sci_asserted(),
            "with SCI_EN clear the event would be an SMI, which this board has no path for"
        );
        write32(&block, PM1_CNT, SCI_EN);
        assert!(block.sci_asserted());
        // And clearing the status drops it again.
        write32(&block, PM1_STS, u32::from(TMROF));
        assert!(!block.sci_asserted());
    }

    #[test]
    fn slp_en_is_write_only_and_latches_the_type() {
        let block = AcpiBlock::new();
        assert_eq!(block.sleep_request(), None);
        // S5: SLP_TYP = 111b with SLP_EN, which is what an ACPI shutdown is.
        write32(&block, PM1_CNT, (0b111 << 10) | SLP_EN);
        assert_eq!(block.sleep_request(), Some(0b111));
        assert_eq!(
            read32(&block, PM1_CNT) & SLP_EN,
            0,
            "SLP_EN is write-only and reads back zero (§13.8.3.3)"
        );
        assert_eq!(read32(&block, PM1_CNT) & SLP_TYP, 0b111 << 10);
    }

    #[test]
    fn the_gpe_block_is_storage_and_does_not_master_abort() {
        let block = AcpiBlock::new();
        write32(&block, GPE0_EN, 0xdead_beef);
        assert_eq!(read32(&block, GPE0_EN), 0xdead_beef);
        assert_eq!(read32(&block, GPE0_STS), 0, "nothing raises a GPE here");
    }

    #[test]
    fn a_reserved_register_reads_zero_and_swallows_writes() {
        let block = AcpiBlock::new();
        write32(&block, 0x44, 0xffff_ffff);
        assert_eq!(read32(&block, 0x44), 0);
    }

    #[test]
    fn a_debug_write_is_refused_and_a_debug_read_is_not() {
        let block = AcpiBlock::new();
        block.advance_to(TMROF_PERIOD);
        let debug = MemAttrs {
            debug: true,
            ..MemAttrs::default()
        };
        assert!(block.write(PM1_STS, &[0x01, 0x00], debug).is_err());
        assert_eq!(block.status() & TMROF, TMROF, "a debug write cleared it");
        let mut buf = [0u8; 2];
        block.read(PM1_STS, &mut buf, debug).expect("readable");
        assert_eq!(u16::from_le_bytes(buf) & TMROF, TMROF);
    }

    #[test]
    fn an_access_off_the_end_of_the_window_is_refused() {
        let block = AcpiBlock::new();
        let mut buf = [0u8; 4];
        assert!(
            block
                .read(BLOCK_LEN - 2, &mut buf, MemAttrs::default())
                .is_err()
        );
        // And one straddling two dwords, which no `in` issues.
        assert!(block.read(0x02, &mut buf, MemAttrs::default()).is_err());
    }

    #[test]
    fn state_round_trips() {
        let block = AcpiBlock::new();
        block.advance_to(TMROF_PERIOD + 99);
        write32(&block, PM1_CNT, SCI_EN | (0b101 << 10) | SLP_EN);
        block
            .write(PM1_EN, &TMROF.to_le_bytes(), MemAttrs::default())
            .expect("in range");
        let saved = block.save_state();
        let tick = block.tick();

        let restored = AcpiBlock::new();
        restored.load_state(saved, tick);
        assert_eq!(restored.save_state(), saved);
        assert_eq!(restored.tick(), tick);
        assert_eq!(restored.timer(), block.timer());
        assert_eq!(restored.sleep_request(), Some(0b101));
    }
}
