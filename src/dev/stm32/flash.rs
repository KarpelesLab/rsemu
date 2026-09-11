//! The STM32 embedded flash interface.
//!
//! `st.flash` is the controller in front of the on-chip flash array: the wait
//! states firmware sets before it raises the clock, the key sequences that
//! unlock the array, and the program and erase engine a bootloader uses to
//! write the application it just received. It is also, and less obviously, the
//! thing that makes the array *readable* — the array is this device's, not a
//! `rom` object's, because a part that can be programmed cannot be modelled by
//! a region whose contents never change.
//!
//! Written from ST **RM0090** rev 21 §3 (the F405/407/415/417/427/429 embedded
//! flash) and ST **RM0351** rev 9 §3 (the L4). The L4+ layout in **RM0432** §3
//! differs from the L4 only in page size and one status bit, so it is a variant
//! here rather than a third register map. No emulator source of any licence was
//! consulted (`ROADMAP.md` §1).
//!
//! # The array and the interface: one device, two regions
//!
//! This is the design question the device exists to answer, so it is answered
//! here rather than in a commit message.
//!
//! The array has to be **fast to read** — a Cortex-M fetches every instruction
//! through it — and **impossible to write except through the controller's
//! rules**. Those two requirements pull in opposite directions, and there are
//! three ways to reconcile them:
//!
//! 1. Make the array a [`Region::io`] and answer every access from a handler.
//!    Correct, and it turns every instruction fetch on the machine into a
//!    virtual call and a lock. [`Region::split`] is this option with nicer
//!    ergonomics: it requires *both* sides to be plain I/O regions, so it
//!    cannot keep the read side on a store.
//! 2. Map the array `Perms::RX` and have the device flip it to writable while
//!    `CR.PG` is set. This needs a topology change per program — a flatten and
//!    a generation bump on a hot path — and, worse, it is *wrong*: a write that
//!    reached the store directly would skip the erased-state check, the
//!    alignment check and write protection. The controller must see the write,
//!    not merely permit it.
//! 3. **Two overlapping mappings**: a [`RamStore`]-backed region that answers
//!    reads and fetches, and an I/O region at the same address that answers
//!    writes. This is a shape `core::space` already has a name for — the
//!    *directed split* in [`flat`](crate::core::space::flat), where
//!    `FlatEntry::write_to` carries the write winner when it is not the read
//!    winner — and it was built for a Master System cartridge that reads a ROM
//!    bank and writes the RAM behind it. A flash array is the same board one
//!    layer in.
//!
//! Option 3 is what is implemented. Reads and fetches resolve to
//! `FlatTarget::Ram` and never call this device at all; writes resolve to
//! [`Program`], which applies RM0090 §3.6 / RM0351 §3.3 and only then touches
//! the store.
//!
//! The two children live inside **one** container region, published as
//! `flash.array`, rather than as two regions the board must map twice at the
//! same address. That placement is a property of the *chip* — it is how the
//! flash interface decodes its own slave port — and not of the board, which
//! only knows that the array answers at `0x0800_0000`. A machine file that had
//! to write the permission split itself could get it wrong in a way that made
//! flash silently writable as RAM, which is exactly the defect this device
//! exists to remove.
//!
//! # What is not modelled
//!
//! * **The caches.** `ACR.ICEN`, `DCEN`, `PRFTEN`, `ICRST` and `DCRST` read
//!   back and change nothing; `LATENCY` is recorded and readable
//!   ([`Flash::latency`]) and costs no time. There is no instruction prefetch
//!   queue here to flush.
//! * **`PCROP` and `RDP`.** The registers exist and hold what is written to
//!   them, and nothing enforces them: proprietary-code readout protection needs
//!   the fetch path to know whether the *current* PC is inside the protected
//!   range, and readout protection needs the debug seam to consult a device.
//!   Both are real features and both are follow-on work; a model that stored
//!   the bits and claimed enforcement would be worse than one that says so.
//! * **ECC.** `ECCR` reads as its reset value; nothing injects a correction.
//! * **Bus stalling during an operation.** A real part stalls a fetch from the
//!   bank being erased. Here the array stays readable and only `SR.BSY` says
//!   an operation is running, which is what firmware polls.
//! * **`FSTPG` row timing.** Fast programming is accepted and behaves as
//!   ordinary double-word programming, so `MISERR` and `FASTERR` never set.
//!
//! # Time
//!
//! A program or an erase takes time, and firmware polls `SR.BSY` for it. As in
//! [`pwr`](super::pwr) and [`rcc`](super::rcc) that is not a host-clock
//! reading: the device is lazily advanced and holds a deadline in ticks of its
//! own clock domain. The defaults are deliberately short — nothing here is
//! measuring how long a real sector erase takes — and a board that wants the
//! datasheet's figures writes `program-time`, `erase-time` and
//! `mass-erase-time`.

use alloc::boxed::Box;
use alloc::format;
use alloc::string::{String, ToString};
use alloc::sync::Arc;
use alloc::vec::Vec;
use core::fmt;

use crate::core::device::{Device, DeviceClass, PropertySpec, RealizeCtx, ResetKind};
use crate::core::error::{BusError, Error, Result};
use crate::core::props::{Props, ValueKind};
use crate::core::sched::{AccessKind, LazyHandle};
use crate::core::space::{
    AccessConstraints, Mapping, MemAttrs, MemOps, MemResult, Perms, RamStore, Region, RegionRef,
};
use crate::core::state::{ChunkReader, ChunkWriter, Sink, Source};
use crate::core::sync::{AtomicU64, LockRank, Mutex, Ordering};
use crate::core::value::{Endian, Width};
use crate::core::wire::{Level, WireSource};
use crate::machine::Instance;
use crate::machine::validate::{ClassSchema, PortDir, PropSchema};

/// The class name a machine description writes.
const CLASS_NAME: &str = "st.flash";

/// The snapshot chunk version. Bump it with the encoding, never on its own.
const STATE_VERSION: u32 = 1;

/// The reset output: `OBL_LAUNCH` generates a system reset (RM0351 §3.7.5).
pub const RESET_PIN: &str = "reset";

/// The interrupt output, driven by `EOPIE` and `ERRIE`.
pub const IRQ_PIN: &str = "irq";

/// How many 32-bit words the widest layout occupies: the L4 map ends at
/// `WRP2BR` (`+0x50`), so twenty-one words covers every variant and the
/// snapshot encoding is one shape.
const WORDS: usize = 0x15;

/// How many option words a variant can have. The L4 has nine: `OPTR`, two
/// `PCROP` pairs and two `WRP` pairs.
const OPT_WORDS: usize = 9;

/// "Nothing is running."
const NO_DEADLINE: u64 = u64::MAX;

/// Default program time, in ticks of this device's clock domain.
const DEFAULT_PROGRAM_TIME: u64 = 8;
/// Default page/sector erase time.
const DEFAULT_ERASE_TIME: u64 = 64;
/// Default bank/mass erase time.
const DEFAULT_MASS_ERASE_TIME: u64 = 512;

// -- the key sequences (RM0090 §3.5.1, RM0351 §3.3.5) ------------------------

/// `FLASH_KEYR` ← this, then [`KEY2`], clears `CR.LOCK`.
const KEY1: u32 = 0x4567_0123;
/// The second half of the unlock sequence.
const KEY2: u32 = 0xCDEF_89AB;
/// `FLASH_OPTKEYR` ← this, then [`OPTKEY2`], clears the option lock.
const OPTKEY1: u32 = 0x0819_2A3B;
/// The second half of the option unlock sequence.
const OPTKEY2: u32 = 0x4C5D_6E7F;
/// `FLASH_PDKEYR` ← this, then [`PDKEY2`], allows `ACR.RUN_PD` to be written
/// (RM0351 §3.7.2). The power-down mode itself is not modelled.
const PDKEY1: u32 = 0x0415_2637;
/// The second half of the power-down unlock sequence.
const PDKEY2: u32 = 0xFAFB_FCFD;

// ---------------------------------------------------------------------------
// Variants
// ---------------------------------------------------------------------------

/// Which family's register map and erase geometry this instance has.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Variant {
    /// RM0090 §3.7: sector-based erase with **unequal** sectors, `PSIZE`
    /// programming parallelism, `OPTCR`/`OPTCR1`.
    F4,
    /// RM0351 §3.7: 2 KiB pages, 64-bit programming delivered as two 32-bit
    /// writes, `OPTR` and the `PCROP`/`WRP` ranges.
    L4,
    /// RM0432 §3: as [`Variant::L4`], with 4 KiB pages in dual-bank mode and
    /// 8 KiB pages in single-bank mode, plus `SR.PEMPTY`.
    L4Plus,
}

impl Variant {
    /// The spelling a machine file writes.
    #[must_use]
    pub fn as_str(self) -> &'static str {
        match self {
            Variant::F4 => "f4",
            Variant::L4 => "l4",
            Variant::L4Plus => "l4plus",
        }
    }

    /// Whether this is the RM0090 layout.
    fn is_f4(self) -> bool {
        matches!(self, Variant::F4)
    }

    /// How many bytes of the peripheral's kilobyte actually decode.
    #[must_use]
    pub fn register_bytes(self) -> u64 {
        match self {
            // Through `OPTCR1` at `+0x18` (RM0090 §3.9.8).
            Variant::F4 => 0x1c,
            // Through `WRP2BR` at `+0x50` (RM0351 §3.7.17).
            Variant::L4 | Variant::L4Plus => 0x54,
        }
    }

    /// Which register offsets are option bytes — the words `OPTSTRT` commits
    /// and a reset or `OBL_LAUNCH` reloads.
    fn option_words(self) -> &'static [u64] {
        match self {
            Variant::F4 => &[F4_OPTCR, F4_OPTCR1],
            Variant::L4 | Variant::L4Plus => &[
                L4_OPTR,
                L4_PCROP1SR,
                L4_PCROP1ER,
                L4_WRP1AR,
                L4_WRP1BR,
                L4_PCROP2SR,
                L4_PCROP2ER,
                L4_WRP2AR,
                L4_WRP2BR,
            ],
        }
    }
}

// -- F4 register offsets (RM0090 §3.9.9, the register map) -------------------

/// `FLASH_ACR`, both families.
const ACR: u64 = 0x00;
/// F4: `FLASH_KEYR`.
const F4_KEYR: u64 = 0x04;
/// F4: `FLASH_OPTKEYR`.
const F4_OPTKEYR: u64 = 0x08;
/// F4: `FLASH_SR`.
const F4_SR: u64 = 0x0c;
/// F4: `FLASH_CR`.
const F4_CR: u64 = 0x10;
/// F4: `FLASH_OPTCR`.
const F4_OPTCR: u64 = 0x14;
/// F4: `FLASH_OPTCR1`, the write protection for sectors 12–23.
const F4_OPTCR1: u64 = 0x18;

// -- L4 register offsets (RM0351 §3.7.18) ------------------------------------

/// L4: `FLASH_PDKEYR`.
const L4_PDKEYR: u64 = 0x04;
/// L4: `FLASH_KEYR`.
const L4_KEYR: u64 = 0x08;
/// L4: `FLASH_OPTKEYR`.
const L4_OPTKEYR: u64 = 0x0c;
/// L4: `FLASH_SR`.
const L4_SR: u64 = 0x10;
/// L4: `FLASH_CR`.
const L4_CR: u64 = 0x14;
/// L4: `FLASH_ECCR`.
const L4_ECCR: u64 = 0x18;
/// L4: `FLASH_OPTR`.
const L4_OPTR: u64 = 0x20;
/// L4: `FLASH_PCROP1SR`.
const L4_PCROP1SR: u64 = 0x24;
/// L4: `FLASH_PCROP1ER`.
const L4_PCROP1ER: u64 = 0x28;
/// L4: `FLASH_WRP1AR`.
const L4_WRP1AR: u64 = 0x2c;
/// L4: `FLASH_WRP1BR`.
const L4_WRP1BR: u64 = 0x30;
/// L4: `FLASH_PCROP2SR`.
const L4_PCROP2SR: u64 = 0x44;
/// L4: `FLASH_PCROP2ER`.
const L4_PCROP2ER: u64 = 0x48;
/// L4: `FLASH_WRP2AR`.
const L4_WRP2AR: u64 = 0x4c;
/// L4: `FLASH_WRP2BR`.
const L4_WRP2BR: u64 = 0x50;

// -- SR bits -----------------------------------------------------------------

/// `SR.EOP`, both families.
const SR_EOP: u32 = 1 << 0;
/// `SR.OPERR`, both families.
const SR_OPERR: u32 = 1 << 1;
/// L4 `SR.PROGERR`: the double word was not erased (RM0351 §3.7.6).
const SR_PROGERR: u32 = 1 << 3;
/// `SR.WRPERR`, both families.
const SR_WRPERR: u32 = 1 << 4;
/// `SR.PGAERR`, both families: an alignment error.
const SR_PGAERR: u32 = 1 << 5;
/// F4 `SR.PGPERR`: the access width disagrees with `CR.PSIZE`.
const F4_SR_PGPERR: u32 = 1 << 6;
/// L4 `SR.SIZERR`: the access was a byte or a half-word.
const L4_SR_SIZERR: u32 = 1 << 6;
/// `SR.PGSERR`, both families: the control register was not set up for this.
const SR_PGSERR: u32 = 1 << 7;
/// `SR.BSY`, both families.
const SR_BSY: u32 = 1 << 16;
/// L4+ `SR.PEMPTY`: the first location of the boot bank is erased.
const SR_PEMPTY: u32 = 1 << 17;

/// Every F4 error bit, for `OPERR` and for the interrupt.
const F4_ERRORS: u32 =
    SR_OPERR | SR_WRPERR | SR_PGAERR | F4_SR_PGPERR | SR_PGSERR | (1 << 8/* RDERR */);
/// Every L4 error bit.
const L4_ERRORS: u32 = SR_OPERR
    | SR_PROGERR
    | SR_WRPERR
    | SR_PGAERR
    | L4_SR_SIZERR
    | SR_PGSERR
    | (1 << 8/* MISERR */)
    | (1 << 9/* FASTERR */)
    | (1 << 14/* RDERR */)
    | (1 << 15/* OPTVERR */);

// -- CR bits -----------------------------------------------------------------

/// `CR.PG`, both families.
const CR_PG: u32 = 1 << 0;
/// F4 `CR.SER` — sector erase.
const F4_CR_SER: u32 = 1 << 1;
/// F4 `CR.MER` — mass erase of sectors 0–11.
const F4_CR_MER: u32 = 1 << 2;
/// F4 `CR.MER1` — mass erase of sectors 12–23 on a 2 MiB part.
const F4_CR_MER1: u32 = 1 << 15;
/// L4 `CR.PER` — page erase.
const L4_CR_PER: u32 = 1 << 1;
/// L4 `CR.MER1` — bank 1 mass erase.
const L4_CR_MER1: u32 = 1 << 2;
/// L4 `CR.BKER` — which bank `PNB` counts in.
const L4_CR_BKER: u32 = 1 << 11;
/// L4 `CR.MER2` — bank 2 mass erase.
const L4_CR_MER2: u32 = 1 << 15;
/// `CR.STRT`, both families.
const CR_STRT: u32 = 1 << 16;
/// L4 `CR.OPTSTRT`.
const L4_CR_OPTSTRT: u32 = 1 << 17;
/// L4 `CR.FSTPG` — fast (row) programming.
const L4_CR_FSTPG: u32 = 1 << 18;
/// `CR.EOPIE`, both families.
const CR_EOPIE: u32 = 1 << 24;
/// `CR.ERRIE`, both families.
const CR_ERRIE: u32 = 1 << 25;
/// L4 `CR.OBL_LAUNCH` — reload the option bytes and reset.
const L4_CR_OBL_LAUNCH: u32 = 1 << 27;
/// L4 `CR.OPTLOCK`.
const L4_CR_OPTLOCK: u32 = 1 << 30;
/// `CR.LOCK`, both families.
const CR_LOCK: u32 = 1 << 31;

/// F4 `OPTCR.OPTLOCK` — the option lock lives in `OPTCR`, not in `CR`.
const F4_OPTCR_OPTLOCK: u32 = 1 << 0;
/// F4 `OPTCR.OPTSTRT`.
const F4_OPTCR_OPTSTRT: u32 = 1 << 1;

/// The bits of `CR` an L4 erase or program uses, which `LOCK` protects. The F4
/// needs no such mask: RM0090 §3.5.1 makes the *whole* of `CR` inaccessible
/// while `LOCK` is set, where an L4 splits the register between two locks.
const L4_CR_OPERATION: u32 =
    CR_PG | L4_CR_PER | L4_CR_MER1 | L4_CR_MER2 | L4_CR_BKER | CR_STRT | L4_CR_FSTPG | (0xff << 3);

/// A megabyte, the F4's bank size.
const BANK: u64 = 1 << 20;

// ---------------------------------------------------------------------------
// Geometry
// ---------------------------------------------------------------------------

/// Where sector `snb` of an F4 lives, as `(offset, length)`.
///
/// **The sectors are not equal** (RM0090 Table 5, "Flash module organization"):
/// four of 16 KiB, one of 64 KiB, then 128 KiB to the end of the megabyte. A
/// 2 MiB part repeats the pattern in a second bank whose sectors are numbered
/// from 12 (Table 6), which is why the bank is factored out rather than the
/// table being written twice.
///
/// `None` if the part has no such sector.
fn f4_sector(size: u64, snb: u32) -> Option<(u64, u64)> {
    let (bank, index) = if snb < 12 {
        (0u64, snb)
    } else {
        (1u64, snb - 12)
    };
    let (within, len) = match index {
        0..=3 => (u64::from(index) * 16 * 1024, 16 * 1024),
        4 => (64 * 1024, 64 * 1024),
        5..=11 => (u64::from(index - 4) * 128 * 1024, 128 * 1024),
        _ => return None,
    };
    let offset = bank * BANK + within;
    (offset.checked_add(len)? <= size).then_some((offset, len))
}

/// Which sector an F4 offset is in, for the write-protection check.
fn f4_sector_of(size: u64, offset: u64) -> Option<u32> {
    (0..24).find(|&snb| {
        f4_sector(size, snb).is_some_and(|(base, len)| offset >= base && offset - base < len)
    })
}

/// Where page `pnb` of `bank` lives on an L4, as `(offset, length)`.
///
/// Pages are uniform here — 2 KiB on an L4 (RM0351 §3.2), 4 KiB dual-bank or
/// 8 KiB single-bank on an L4+ (RM0432 §3.3.1) — so the only subtlety is that
/// `BKER` is meaningless unless `OPTR.DUALBANK` put a second bank there.
fn l4_page(size: u64, page: u64, dual: bool, pnb: u32, bank: bool) -> Option<(u64, u64)> {
    let bank_len = if dual { size / 2 } else { size };
    let base = if dual && bank { bank_len } else { 0 };
    let offset = base.checked_add(u64::from(pnb).checked_mul(page)?)?;
    (offset.checked_add(page)? <= size).then_some((offset, page))
}

// ---------------------------------------------------------------------------
// State
// ---------------------------------------------------------------------------

/// What the device is waiting to finish.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Pending {
    /// Nothing.
    None,
    /// Fill `[offset, offset + len)` with ones.
    Erase { offset: u64, len: u64 },
    /// Program `bytes[..len]` at `offset`, clearing bits only.
    Program {
        offset: u64,
        bytes: [u8; 8],
        len: u8,
    },
    /// Commit the live option registers into the option-byte storage.
    Option,
}

/// Everything the guest can see or change, plus the device's own position.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct State {
    /// The register file, indexed by `offset / 4`.
    words: [u32; WORDS],
    /// The option bytes as stored, which a reset and `OBL_LAUNCH` load.
    stored: [u32; OPT_WORDS],
    /// The L4's double-word latch: the offset of the first word and its value.
    latch: Option<(u64, u32)>,
    /// When the running operation finishes.
    busy_at: u64,
    /// What finishing it does.
    pending: Pending,
    /// How far through each key sequence the guest is.
    key_step: u8,
    optkey_step: u8,
    pdkey_step: u8,
    /// A wrong key locks the interface until the next reset (RM0090 §3.5.1).
    key_error: bool,
    optkey_error: bool,
    /// The tick this device has been advanced to.
    tick: u64,
}

impl State {
    fn reset(variant: Variant, stored: [u32; OPT_WORDS]) -> State {
        let mut words = [0u32; WORDS];
        if variant.is_f4() {
            // "Reset value: 0x8000 0000" — the interface comes up locked
            // (RM0090 §3.9.7).
            words[(F4_CR / 4) as usize] = CR_LOCK;
        } else {
            // `ACR` comes up with the caches on: "Reset value: 0x0000 0600"
            // (RM0351 §3.7.1).
            words[(ACR / 4) as usize] = 0x0000_0600;
            // Both locks are set out of reset (§3.7.5).
            words[(L4_CR / 4) as usize] = CR_LOCK | L4_CR_OPTLOCK;
        }
        let mut state = State {
            words,
            stored,
            latch: None,
            busy_at: NO_DEADLINE,
            pending: Pending::None,
            key_step: 0,
            optkey_step: 0,
            pdkey_step: 0,
            key_error: false,
            optkey_error: false,
            tick: 0,
        };
        state.load_option_bytes(variant);
        state
    }

    /// Copy the option-byte storage into the registers that shadow it. What a
    /// reset does, and what `CR.OBL_LAUNCH` does on demand (RM0351 §3.4.2).
    fn load_option_bytes(&mut self, variant: Variant) {
        for (slot, &offset) in variant.option_words().iter().enumerate() {
            self.words[(offset / 4) as usize] = self.stored[slot];
        }
    }

    /// The reverse: what `OPTSTRT` commits.
    fn store_option_bytes(&mut self, variant: Variant) {
        for (slot, &offset) in variant.option_words().iter().enumerate() {
            self.stored[slot] = self.words[(offset / 4) as usize];
        }
    }

    #[inline]
    fn word(&self, offset: u64) -> u32 {
        self.words[(offset / 4) as usize]
    }

    #[inline]
    fn word_mut(&mut self, offset: u64) -> &mut u32 {
        &mut self.words[(offset / 4) as usize]
    }
}

/// The default option-byte storage for a variant.
fn default_option_bytes(variant: Variant) -> [u32; OPT_WORDS] {
    let mut out = [0u32; OPT_WORDS];
    match variant {
        Variant::F4 => {
            // "Reset value: 0x0FFF AAED" — `nWRP` all ones (no sector is
            // protected), `RDP` = 0xAA (level 0) (RM0090 §3.9.8).
            out[0] = 0x0fff_aaed;
            // `OPTCR1`'s `nWRP` for sectors 12–23, likewise unprotected.
            out[1] = 0x0fff_0000;
        }
        Variant::L4 | Variant::L4Plus => {
            // The value a factory part reads back: `RDP` = 0xAA, `nBOOT1`
            // set, both watchdogs in software mode (RM0351 §3.4.1).
            out[0] = 0xffef_f8aa;
            // Every `PCROP` and `WRP` range starts above where it ends, which
            // is how an option byte says "no protection here" (§3.7.9): the
            // start field is all ones and the end field zero.
            out[1] = 0x0000_ffff;
            out[2] = 0x0000_0000;
            out[3] = 0x0000_00ff;
            out[4] = 0x0000_00ff;
            out[5] = 0x0000_ffff;
            out[6] = 0x0000_0000;
            out[7] = 0x0000_00ff;
            out[8] = 0x0000_00ff;
        }
    }
    out
}

// ---------------------------------------------------------------------------
// The shared core
// ---------------------------------------------------------------------------

/// Everything both regions and the device body reach.
struct Shared {
    state: Mutex<State>,
    variant: Variant,
    /// The array. Reads and fetches come straight off this store; writes
    /// arrive through [`Program`] and land here only if the rules allow it.
    array: Arc<RamStore>,
    size: u64,
    /// L4 page size, in bytes.
    page: u64,
    program_time: u64,
    erase_time: u64,
    mass_erase_time: u64,
    /// The `OBL_LAUNCH` reset output, connected at wiring time.
    reset_out: Mutex<Option<WireSource>>,
    /// The interrupt output.
    irq_out: Mutex<Option<WireSource>>,
    /// The lock-free half of the lazy contract.
    tick: AtomicU64,
    next_event: AtomicU64,
    lazy: Mutex<Option<LazyHandle>>,
}

impl fmt::Debug for Shared {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let mut s = f.debug_struct("Shared");
        s.field("variant", &self.variant).field("size", &self.size);
        match self.state.try_lock() {
            Some(state) => s.field("tick", &state.tick),
            None => s.field("tick", &"<locked>"),
        };
        s.finish()
    }
}

/// What a register or array write asks the caller to do once the lock is
/// released. Outward calls never happen inside the critical section
/// (`CLAUDE.md`, "Concurrency").
#[derive(Debug, Clone, Copy, Default)]
struct After {
    /// `OBL_LAUNCH` was written: pulse the reset line.
    reset: bool,
}

impl Shared {
    // -- the lazy seam -------------------------------------------------------

    /// Catch the device up before answering an access. No lock held.
    fn sync(&self, attrs: MemAttrs) {
        let handle = self.lazy.lock().clone();
        let Some(handle) = handle else { return };
        let kind = if attrs.debug {
            AccessKind::Debug
        } else {
            AccessKind::Guest
        };
        let _ = handle.sync(kind);
    }

    /// Republish what the lock-free lazy surface reads.
    fn republish(&self, state: &State) {
        self.tick.store(state.tick, Ordering::Relaxed);
        let next = if state.busy_at <= state.tick {
            u64::MAX
        } else {
            state.busy_at
        };
        self.next_event.store(next, Ordering::Relaxed);
    }

    /// Finish whatever was running, if its deadline has passed.
    fn advance_to(&self, tick: u64) {
        let mut state = self.state.lock();
        if tick <= state.tick {
            return;
        }
        state.tick = tick;
        if state.busy_at <= tick {
            let pending = state.pending;
            state.busy_at = NO_DEADLINE;
            state.pending = Pending::None;
            self.complete(&mut state, pending);
        }
        self.republish(&state);
        drop(state);
        self.refresh_irq();
    }

    /// Apply a finished operation to the array and raise `EOP`.
    fn complete(&self, state: &mut State, pending: Pending) {
        match pending {
            Pending::None => return,
            Pending::Erase { offset, len } => {
                // An erased cell reads all ones; that is the whole reason a
                // program can only clear bits.
                let _ = self.array.fill(offset, len, 0xff);
            }
            Pending::Program { offset, bytes, len } => {
                // Programming clears bits and never sets them (RM0090 §3.6.2,
                // RM0351 §3.3.7), so the committed value is the AND of what
                // was there with what was written.
                let len = usize::from(len);
                let mut old = [0u8; 8];
                let _ = self.array.read_at(offset, &mut old[..len]);
                let mut new = [0u8; 8];
                for i in 0..len {
                    new[i] = old[i] & bytes[i];
                }
                let _ = self.array.write_at(offset, &new[..len]);
            }
            Pending::Option => state.store_option_bytes(self.variant),
        }
        let sr = self.sr_offset();
        *state.word_mut(sr) &= !SR_BSY;
        *state.word_mut(sr) |= SR_EOP;
    }

    // -- layout helpers ------------------------------------------------------

    fn sr_offset(&self) -> u64 {
        if self.variant.is_f4() { F4_SR } else { L4_SR }
    }

    fn cr_offset(&self) -> u64 {
        if self.variant.is_f4() { F4_CR } else { L4_CR }
    }

    fn errors_mask(&self) -> u32 {
        if self.variant.is_f4() {
            F4_ERRORS
        } else {
            L4_ERRORS
        }
    }

    /// Whether the array is unlocked for programming.
    fn unlocked(&self, state: &State) -> bool {
        state.word(self.cr_offset()) & CR_LOCK == 0
    }

    /// Whether the option bytes are unlocked.
    fn opt_unlocked(&self, state: &State) -> bool {
        if self.variant.is_f4() {
            state.word(F4_OPTCR) & F4_OPTCR_OPTLOCK == 0
        } else {
            state.word(L4_CR) & L4_CR_OPTLOCK == 0
        }
    }

    fn busy(&self, state: &State) -> bool {
        state.word(self.sr_offset()) & SR_BSY != 0
    }

    /// Whether `OPTR.DUALBANK` says there are two banks (RM0351 §3.4.1 bit 21).
    fn dual_bank(&self, state: &State) -> bool {
        !self.variant.is_f4() && state.word(L4_OPTR) & (1 << 21) != 0
    }

    /// Raise one or more error flags, and `OPERR` with them.
    fn fail(&self, state: &mut State, flags: u32) {
        let sr = self.sr_offset();
        let errie = state.word(self.cr_offset()) & CR_ERRIE != 0;
        // F4: "OPERR … is set only if error interrupts are enabled (ERRIE=1)"
        // (RM0090 §3.9.5). The L4 sets it for every unsuccessful operation
        // (RM0351 §3.7.6), which is the more useful of the two and not a
        // difference a model may average away.
        let operr = if self.variant.is_f4() && !errie {
            0
        } else {
            SR_OPERR
        };
        *state.word_mut(sr) |= flags | operr;
    }

    // -- write protection ----------------------------------------------------

    /// Whether any byte of `[offset, offset + len)` is write-protected.
    fn protected(&self, state: &State, offset: u64, len: u64) -> bool {
        if self.variant.is_f4() {
            self.f4_protected(state, offset, len)
        } else {
            self.l4_protected(state, offset, len)
        }
    }

    /// `OPTCR.nWRP` / `OPTCR1.nWRP`: a **zero** protects its sector
    /// (RM0090 §3.9.8, "nWRP: Not write protect").
    fn f4_protected(&self, state: &State, offset: u64, len: u64) -> bool {
        let mut at = offset;
        while at < offset.saturating_add(len) {
            let Some(snb) = f4_sector_of(self.size, at) else {
                return false;
            };
            let (base, sector_len) = match f4_sector(self.size, snb) {
                Some(pair) => pair,
                None => return false,
            };
            let (reg, bit) = if snb < 12 {
                (F4_OPTCR, 16 + snb)
            } else {
                (F4_OPTCR1, 16 + (snb - 12))
            };
            if state.word(reg) & (1 << bit) == 0 {
                return true;
            }
            at = base.saturating_add(sector_len);
        }
        false
    }

    /// `WRPxAR`/`WRPxBR`: two inclusive page ranges per bank, disabled when
    /// the start page is above the end page (RM0351 §3.7.12).
    fn l4_protected(&self, state: &State, offset: u64, len: u64) -> bool {
        let dual = self.dual_bank(state);
        let bank_len = if dual { self.size / 2 } else { self.size };
        let end = offset.saturating_add(len.saturating_sub(1));
        for (reg, bank) in [
            (L4_WRP1AR, 0u64),
            (L4_WRP1BR, 0),
            (L4_WRP2AR, 1),
            (L4_WRP2BR, 1),
        ] {
            if bank == 1 && !dual {
                continue;
            }
            let value = state.word(reg);
            let (strt, stop) = (u64::from(value & 0xff), u64::from((value >> 16) & 0xff));
            if strt > stop {
                continue;
            }
            let base = bank * bank_len;
            let lo = base + strt * self.page;
            let hi = base + (stop + 1) * self.page;
            if offset < hi && end >= lo {
                return true;
            }
        }
        false
    }

    // -- the array's write side ----------------------------------------------

    /// A guest store into the flash window.
    ///
    /// Never returns a bus error for a *flash* reason: the controller reports
    /// through `SR`, and a `BusFault` where the silicon sets `PGSERR` would
    /// send firmware down a path it never takes on the part.
    fn program(&self, offset: u64, src: &[u8], attrs: MemAttrs) -> MemResult {
        if attrs.debug {
            // A debugger writing flash is a debugger *loading* flash: it drives
            // the programming interface over SWD rather than storing through
            // the AHB, so the honest model of the side door is a direct poke
            // that touches no status bit, consumes no double-word latch and
            // does not start an operation. `MemAttrs::debug`'s rule is that the
            // access must not disturb the device's visible state, and this is
            // the only write that satisfies it.
            return self.array.write_at(offset, src);
        }
        if offset
            .checked_add(src.len() as u64)
            .is_none_or(|e| e > self.size)
        {
            return Err(BusError::BadAccess);
        }
        self.sync(attrs);
        {
            let mut state = self.state.lock();
            if self.variant.is_f4() {
                self.program_f4(&mut state, offset, src);
            } else {
                self.program_l4(&mut state, offset, src);
            }
            self.republish(&state);
        }
        self.refresh_irq();
        Ok(())
    }

    /// RM0090 §3.6.2: parallelism is `CR.PSIZE` and a program only clears bits.
    fn program_f4(&self, state: &mut State, offset: u64, src: &[u8]) {
        if !self.unlocked(state) || state.word(F4_CR) & CR_PG == 0 || self.busy(state) {
            // "PGSERR … set when a write access to the Flash memory is
            // performed by the code while the control register has not been
            // correctly configured" (RM0090 §3.9.5). The array keeps its
            // contents and the bus sees no fault, which is why a stray `STR`
            // into a locked flash is invisible to firmware that does not read
            // `SR`.
            self.fail(state, SR_PGSERR);
            return;
        }
        if self.protected(state, offset, src.len() as u64) {
            self.fail(state, SR_WRPERR);
            return;
        }
        let psize = (state.word(F4_CR) >> 8) & 0b11;
        let width = 1u64 << psize;
        if src.len() as u64 != width {
            // "PGPERR … the size of the access is not consistent with the
            // parallelism configured in PSIZE".
            self.fail(state, F4_SR_PGPERR);
            return;
        }
        if !offset.is_multiple_of(width) {
            self.fail(state, SR_PGAERR);
            return;
        }
        let mut bytes = [0u8; 8];
        bytes[..src.len()].copy_from_slice(src);
        self.start(
            state,
            Pending::Program {
                offset,
                bytes,
                len: src.len() as u8,
            },
            self.program_time,
        );
    }

    /// RM0351 §3.3.7: one double word, delivered as two 32-bit writes.
    fn program_l4(&self, state: &mut State, offset: u64, src: &[u8]) {
        let cr = state.word(L4_CR);
        if !self.unlocked(state) || cr & (CR_PG | L4_CR_FSTPG) == 0 || self.busy(state) {
            state.latch = None;
            self.fail(state, SR_PGSERR);
            return;
        }
        if src.len() != 4 {
            // "SIZERR … set if the size of the access is a byte or half-word".
            self.fail(state, L4_SR_SIZERR);
            return;
        }
        if self.protected(state, offset, 8) {
            state.latch = None;
            self.fail(state, SR_WRPERR);
            return;
        }
        let value = u32::from_le_bytes([src[0], src[1], src[2], src[3]]);
        match state.latch {
            None => {
                if !offset.is_multiple_of(8) {
                    // "PGAERR … the first word to be programmed is not aligned
                    // with a double word address".
                    self.fail(state, SR_PGAERR);
                    return;
                }
                // The first half sets nothing: `BSY` goes high when the second
                // word arrives and the double word is actually programmed.
                state.latch = Some((offset, value));
            }
            Some((first, low)) => {
                state.latch = None;
                if offset != first.wrapping_add(4) {
                    // "… or the second word doesn't belong to the same double
                    // word address".
                    self.fail(state, SR_PGAERR);
                    return;
                }
                let mut old = [0u8; 8];
                let _ = self.array.read_at(first, &mut old);
                if old != [0xff; 8] {
                    // "PROGERR … set if the word to write is not previously
                    // erased" — and nothing is programmed.
                    self.fail(state, SR_PROGERR);
                    return;
                }
                let mut bytes = [0u8; 8];
                bytes[..4].copy_from_slice(&low.to_le_bytes());
                bytes[4..].copy_from_slice(&value.to_le_bytes());
                self.start(
                    state,
                    Pending::Program {
                        offset: first,
                        bytes,
                        len: 8,
                    },
                    self.program_time,
                );
            }
        }
    }

    /// Set `BSY` and arm the deadline.
    fn start(&self, state: &mut State, pending: Pending, ticks: u64) {
        let sr = self.sr_offset();
        *state.word_mut(sr) |= SR_BSY;
        state.pending = pending;
        state.busy_at = state.tick.saturating_add(ticks.max(1));
    }

    // -- the register block --------------------------------------------------

    fn read_register(&self, offset: u64) -> u32 {
        let state = self.state.lock();
        // The key registers are write-only; reading one returns zero rather
        // than the last key written, so a snapshot of the bus never carries it.
        let write_only = if self.variant.is_f4() {
            offset == F4_KEYR || offset == F4_OPTKEYR
        } else {
            offset == L4_KEYR || offset == L4_OPTKEYR || offset == L4_PDKEYR
        };
        if write_only {
            return 0;
        }
        state.word(offset)
    }

    /// Write one register. Returns what must happen outside the lock.
    fn write_register(&self, offset: u64, value: u32) -> MemResult<After> {
        let after = {
            let mut state = self.state.lock();
            let result = if self.variant.is_f4() {
                self.write_f4(&mut state, offset, value)
                    .map(|()| After::default())
            } else {
                self.write_l4(&mut state, offset, value)
            };
            self.republish(&state);
            result?
        };
        self.refresh_irq();
        Ok(after)
    }

    fn write_f4(&self, state: &mut State, offset: u64, value: u32) -> MemResult {
        match offset {
            ACR => {
                // `LATENCY[2:0]`, `PRFTEN`, `ICEN`, `DCEN`, `ICRST`, `DCRST`
                // (RM0090 §3.9.1). Everything else reads as zero.
                *state.word_mut(ACR) = value & 0x0000_1f07;
            }
            F4_KEYR => self.write_key(state, value)?,
            F4_OPTKEYR => self.write_optkey(state, value)?,
            F4_SR => {
                // Every flag is write-1-to-clear; `BSY` is hardware's.
                let clear = value & (SR_EOP | F4_ERRORS);
                *state.word_mut(F4_SR) &= !clear;
            }
            F4_CR => self.write_cr_f4(state, value),
            F4_OPTCR | F4_OPTCR1 => self.write_optcr_f4(state, offset, value),
            _ => {}
        }
        Ok(())
    }

    fn write_cr_f4(&self, state: &mut State, value: u32) {
        let before = state.word(F4_CR);
        if before & CR_LOCK != 0 {
            // "The FLASH_CR register is not accessible in write mode when the
            // LOCK bit is set" (RM0090 §3.5.1). `LOCK` itself is already set,
            // so there is nothing a write could do.
            return;
        }
        // `LOCK` is set by software and cleared only by the key sequence.
        let lock = value & CR_LOCK;
        *state.word_mut(F4_CR) = (value & !CR_LOCK & !CR_STRT) | lock;
        if value & CR_STRT == 0 {
            return;
        }
        if self.busy(state) {
            self.fail(state, SR_PGSERR);
            return;
        }
        let range = if value & F4_CR_MER != 0 {
            Some((0, self.size.min(BANK)))
        } else if value & F4_CR_MER1 != 0 {
            Some((BANK, self.size.saturating_sub(BANK)))
        } else if value & F4_CR_SER != 0 {
            f4_sector(self.size, (value >> 3) & 0b1111)
        } else {
            None
        };
        let Some((offset, len)) = range.filter(|&(_, len)| len > 0) else {
            self.fail(state, SR_PGSERR);
            return;
        };
        if self.protected(state, offset, len) {
            self.fail(state, SR_WRPERR);
            return;
        }
        let time = if value & (F4_CR_MER | F4_CR_MER1) != 0 {
            self.mass_erase_time
        } else {
            self.erase_time
        };
        self.start(state, Pending::Erase { offset, len }, time);
    }

    fn write_optcr_f4(&self, state: &mut State, offset: u64, value: u32) {
        if !self.opt_unlocked(state) {
            return;
        }
        if offset == F4_OPTCR1 {
            *state.word_mut(F4_OPTCR1) = value & 0x0fff_0000;
            return;
        }
        // `OPTLOCK` is set by software, cleared only by the option key
        // sequence; `OPTSTRT` is a strobe rather than storage.
        let lock = value & F4_OPTCR_OPTLOCK;
        *state.word_mut(F4_OPTCR) = (value & !F4_OPTCR_OPTSTRT & !F4_OPTCR_OPTLOCK) | lock;
        if value & F4_OPTCR_OPTSTRT == 0 {
            return;
        }
        if self.busy(state) {
            self.fail(state, SR_PGSERR);
            return;
        }
        self.start(state, Pending::Option, self.erase_time);
    }

    fn write_l4(&self, state: &mut State, offset: u64, value: u32) -> MemResult<After> {
        let mut after = After::default();
        match offset {
            ACR => {
                // `LATENCY[2:0]`, `PRFTEN`, `ICEN`, `DCEN`, `ICRST`, `DCRST`,
                // `RUN_PD`, `SLEEP_PD` (RM0351 §3.7.1).
                *state.word_mut(ACR) = value & 0x0000_7f07;
            }
            L4_KEYR => self.write_key(state, value)?,
            L4_OPTKEYR => self.write_optkey(state, value)?,
            L4_PDKEYR => {
                // The power-down key sequence gates `ACR.RUN_PD`; nothing here
                // models the power-down itself, so it is tracked and no more.
                state.pdkey_step = match (state.pdkey_step, value) {
                    (0, PDKEY1) => 1,
                    (1, PDKEY2) => 0,
                    _ => 0,
                };
            }
            L4_SR => {
                let clear = value & (SR_EOP | L4_ERRORS | SR_PEMPTY);
                *state.word_mut(L4_SR) &= !clear;
            }
            L4_CR => after = self.write_cr_l4(state, value),
            L4_ECCR => {
                // `ECCC` and `ECCD` are write-1-to-clear; nothing sets them.
                *state.word_mut(L4_ECCR) &= !(value & 0xc000_0000);
            }
            L4_OPTR | L4_PCROP1SR | L4_PCROP1ER | L4_WRP1AR | L4_WRP1BR | L4_PCROP2SR
            | L4_PCROP2ER | L4_WRP2AR | L4_WRP2BR
                if self.opt_unlocked(state) =>
            {
                *state.word_mut(offset) = value;
            }
            _ => {}
        }
        Ok(after)
    }

    fn write_cr_l4(&self, state: &mut State, value: u32) -> After {
        let mut after = After::default();
        let before = state.word(L4_CR);
        let locked = before & CR_LOCK != 0;
        let opt_locked = before & L4_CR_OPTLOCK != 0;

        // Both locks are set by software and cleared only by their key
        // sequence, and setting `OPTLOCK` sets `LOCK` with it (RM0351 §3.7.5).
        let mut next = before;
        if !locked {
            next = (next & !L4_CR_OPERATION) | (value & L4_CR_OPERATION);
        }
        next = (next & !(CR_EOPIE | CR_ERRIE | (1 << 26)))
            | (value & (CR_EOPIE | CR_ERRIE | (1 << 26)));
        if value & CR_LOCK != 0 {
            next |= CR_LOCK;
        }
        if value & L4_CR_OPTLOCK != 0 {
            next |= L4_CR_OPTLOCK | CR_LOCK;
        }
        // `STRT`, `OPTSTRT` and `OBL_LAUNCH` are strobes, not storage.
        next &= !(CR_STRT | L4_CR_OPTSTRT | L4_CR_OBL_LAUNCH);
        *state.word_mut(L4_CR) = next;

        if value & CR_STRT != 0 && !locked {
            self.erase_l4(state, value);
        }
        if value & L4_CR_OPTSTRT != 0 && !opt_locked {
            if self.busy(state) {
                self.fail(state, SR_PGSERR);
            } else {
                self.start(state, Pending::Option, self.erase_time);
            }
        }
        if value & L4_CR_OBL_LAUNCH != 0 {
            if opt_locked || self.busy(state) {
                self.fail(state, SR_PGSERR);
            } else {
                // "Option byte loading … generates a reset of the device"
                // (RM0351 §3.7.5). The stored bytes become the live ones and
                // the machine is reset; the pulse happens outside the lock.
                state.load_option_bytes(self.variant);
                after.reset = true;
            }
        }
        after
    }

    fn erase_l4(&self, state: &mut State, value: u32) {
        if self.busy(state) {
            self.fail(state, SR_PGSERR);
            return;
        }
        let dual = self.dual_bank(state);
        let bank_len = if dual { self.size / 2 } else { self.size };
        let range = if value & (L4_CR_MER1 | L4_CR_MER2) == (L4_CR_MER1 | L4_CR_MER2) {
            Some((0, self.size))
        } else if value & L4_CR_MER1 != 0 {
            Some((0, bank_len))
        } else if value & L4_CR_MER2 != 0 {
            dual.then_some((bank_len, bank_len))
        } else if value & L4_CR_PER != 0 {
            l4_page(
                self.size,
                self.page,
                dual,
                (value >> 3) & 0xff,
                value & L4_CR_BKER != 0,
            )
        } else {
            None
        };
        let Some((offset, len)) = range.filter(|&(_, len)| len > 0) else {
            self.fail(state, SR_PGSERR);
            return;
        };
        if self.protected(state, offset, len) {
            self.fail(state, SR_WRPERR);
            return;
        }
        let time = if value & (L4_CR_MER1 | L4_CR_MER2) != 0 {
            self.mass_erase_time
        } else {
            self.erase_time
        };
        self.start(state, Pending::Erase { offset, len }, time);
    }

    /// `KEYR`: `KEY1` then `KEY2` clears `LOCK`.
    fn write_key(&self, state: &mut State, value: u32) -> MemResult {
        if state.key_error {
            // "… a bus error is detected if the KEYR is written again"
            // (RM0351 §3.3.5). The F4 simply ignores it.
            return if self.variant.is_f4() {
                Ok(())
            } else {
                Err(BusError::BadAccess)
            };
        }
        let cr = self.cr_offset();
        if state.word(cr) & CR_LOCK == 0 {
            return Ok(());
        }
        match (state.key_step, value) {
            (0, KEY1) => state.key_step = 1,
            (1, KEY2) => {
                state.key_step = 0;
                *state.word_mut(cr) &= !CR_LOCK;
            }
            _ => {
                // "In case of a wrong key sequence … the FLASH_CR register is
                // locked until the next reset."
                state.key_step = 0;
                state.key_error = true;
            }
        }
        Ok(())
    }

    /// `OPTKEYR`: `OPTKEY1` then `OPTKEY2` clears the option lock.
    fn write_optkey(&self, state: &mut State, value: u32) -> MemResult {
        if state.optkey_error {
            return if self.variant.is_f4() {
                Ok(())
            } else {
                Err(BusError::BadAccess)
            };
        }
        if self.opt_unlocked(state) {
            return Ok(());
        }
        match (state.optkey_step, value) {
            (0, OPTKEY1) => state.optkey_step = 1,
            (1, OPTKEY2) => {
                state.optkey_step = 0;
                if self.variant.is_f4() {
                    *state.word_mut(F4_OPTCR) &= !F4_OPTCR_OPTLOCK;
                } else {
                    *state.word_mut(L4_CR) &= !L4_CR_OPTLOCK;
                }
            }
            _ => {
                state.optkey_step = 0;
                state.optkey_error = true;
            }
        }
        Ok(())
    }

    // -- outputs -------------------------------------------------------------

    /// Drive the interrupt line to whatever `SR` and `CR` now say.
    ///
    /// Called with **no lock held**: the sink is the core's NVIC.
    fn refresh_irq(&self) {
        let level = {
            let state = self.state.lock();
            let sr = state.word(self.sr_offset());
            let cr = state.word(self.cr_offset());
            let eop = sr & SR_EOP != 0 && cr & CR_EOPIE != 0;
            let err = sr & self.errors_mask() != 0 && cr & CR_ERRIE != 0;
            Level::from_bool(eop || err)
        };
        let source = self.irq_out.lock().clone();
        if let Some(source) = source {
            source.set(level);
        }
    }

    /// Pulse the reset line, as `OBL_LAUNCH` asks.
    fn pulse_reset(&self) {
        let source = self.reset_out.lock().clone();
        if let Some(source) = source {
            source.pulse(Level::High);
        }
    }
}

// ---------------------------------------------------------------------------
// The two region faces
// ---------------------------------------------------------------------------

/// The controller's register block.
#[derive(Debug)]
struct Registers {
    shared: Arc<Shared>,
}

impl MemOps for Registers {
    fn read(&self, offset: u64, dst: &mut [u8], attrs: MemAttrs) -> MemResult {
        let [a, b, c, d] = dst else {
            return Err(BusError::BadAccess);
        };
        self.shared.sync(attrs);
        let value = self.shared.read_register(offset & !3);
        let bytes = value.to_le_bytes();
        (*a, *b, *c, *d) = (bytes[0], bytes[1], bytes[2], bytes[3]);
        Ok(())
    }

    fn write(&self, offset: u64, src: &[u8], attrs: MemAttrs) -> MemResult {
        let [a, b, c, d] = src else {
            return Err(BusError::BadAccess);
        };
        if attrs.debug {
            // A debug write to `KEYR` would unlock the array and a debug write
            // to `CR` would start an erase. Refused rather than guessed at, as
            // in `st.pwr` (`ROADMAP.md` §15, invariant 5).
            return Err(BusError::BadAccess);
        }
        self.shared.sync(attrs);
        let after = self
            .shared
            .write_register(offset & !3, u32::from_le_bytes([*a, *b, *c, *d]))?;
        if after.reset {
            self.shared.pulse_reset();
        }
        Ok(())
    }

    fn constraints(&self) -> AccessConstraints {
        AccessConstraints::word(Width::U32, Endian::Little)
    }
}

/// The array's **write** side: the half of the flash window that is this
/// device rather than a store. See the module header.
#[derive(Debug)]
struct Program {
    shared: Arc<Shared>,
}

impl MemOps for Program {
    fn read(&self, offset: u64, dst: &mut [u8], _attrs: MemAttrs) -> MemResult {
        // Unreachable through the container, whose read winner is the store.
        // Implemented anyway so that an embedder mapping this region on its
        // own gets the array rather than a fault.
        self.shared.array.read_at(offset, dst)
    }

    fn write(&self, offset: u64, src: &[u8], attrs: MemAttrs) -> MemResult {
        self.shared.program(offset, src, attrs)
    }

    fn constraints(&self) -> AccessConstraints {
        // A byte, a half-word, a word or a double word all reach the
        // controller: rejecting the wrong width here would raise a bus fault
        // where the part sets `PGPERR`/`SIZERR`, and firmware reads those.
        // No bursts, though — a program is one register-width store and a
        // block copy into flash is not a transfer the part can perform, so the
        // dispatcher should refuse it rather than hand this a long slice to
        // report `SIZERR` about.
        AccessConstraints::IO
    }
}

// ---------------------------------------------------------------------------
// The device
// ---------------------------------------------------------------------------

/// An STM32 embedded flash interface, and the array behind it.
#[derive(Debug)]
pub struct Flash {
    shared: Arc<Shared>,
    regs: RegionRef,
    array: RegionRef,
}

/// A short-hand for a configuration error.
fn config(message: String) -> Error {
    Error::Config {
        at: String::from(CLASS_NAME),
        message,
    }
}

impl Flash {
    /// Validate `props` and build the device.
    ///
    /// # Errors
    ///
    /// [`Error::Property`] for an unknown or ill-typed property,
    /// [`Error::Config`] for a size the geometry cannot describe or an image
    /// that does not fit.
    pub fn new(props: &Props) -> Result<Flash> {
        let mut r = props.reader();
        let variant = match r.or_enum("variant", "f4", &["f4", "l4", "l4plus"])? {
            "l4" => Variant::L4,
            "l4plus" => Variant::L4Plus,
            _ => Variant::F4,
        };
        let size = r.require_size("size")?;
        let image = r
            .optional_media("image")?
            .map(crate::core::props::Media::to_bytes);
        let optr = r.optional::<u64>("optr")?;
        let program_time = r.or("program-time", DEFAULT_PROGRAM_TIME)?;
        let erase_time = r.or("erase-time", DEFAULT_ERASE_TIME)?;
        let mass_erase_time = r.or("mass-erase-time", DEFAULT_MASS_ERASE_TIME)?;
        r.finish()?;

        if size == 0 || !size.is_multiple_of(2048) {
            return Err(config(format!(
                "a flash array of {size} byte(s): every geometry in these manuals is built from \
                 whole 2 KiB pages"
            )));
        }
        if usize::try_from(size).is_err() {
            return Err(config(format!(
                "a flash of {size} byte(s) is larger than this host's address space"
            )));
        }
        if variant.is_f4() && size > 2 * BANK {
            return Err(config(format!(
                "an F4's sector map runs to 2 MiB (RM0090 Table 6) and this part is {size} byte(s)"
            )));
        }
        if let Some(image) = &image
            && image.len() as u64 > size
        {
            return Err(config(format!(
                "the bound image is {} byte(s) and the flash is {size}",
                image.len()
            )));
        }

        let mut stored = default_option_bytes(variant);
        if let Some(optr) = optr {
            stored[0] = u32::try_from(optr).map_err(|_| {
                config(format!(
                    "`optr` is a 32-bit option register and {optr:#x} is not"
                ))
            })?;
        }

        // An L4's pages are 2 KiB; an L4+'s are 4 KiB when `OPTR.DUALBANK`
        // splits the array and 8 KiB when it does not (RM0432 §3.3.1).
        let dual = stored[0] & (1 << 21) != 0;
        let page = match variant {
            Variant::F4 => 0,
            Variant::L4 => 2048,
            Variant::L4Plus if dual => 4096,
            Variant::L4Plus => 8192,
        };

        let array_store = Arc::new(RamStore::new(size));
        // Erased, not zeroed: an unwritten part reads all ones, and firmware
        // that finds zeroes concludes the array has already been programmed.
        array_store
            .fill(0, size, 0xff)
            .map_err(|_| config(String::from("the flash array could not be erased")))?;
        if let Some(image) = image {
            array_store
                .write_at(0, &image)
                .map_err(|_| config(String::from("the flash refused its initial image")))?;
        }

        let shared = Arc::new(Shared {
            state: Mutex::with_rank(LockRank::DEVICE, State::reset(variant, stored)),
            variant,
            array: Arc::clone(&array_store),
            size,
            page,
            program_time,
            erase_time,
            mass_erase_time,
            reset_out: Mutex::with_rank(LockRank::WIRE, None),
            irq_out: Mutex::with_rank(LockRank::WIRE, None),
            tick: AtomicU64::new(0),
            next_event: AtomicU64::new(u64::MAX),
            lazy: Mutex::with_rank(LockRank::LEAF, None),
        });

        let regs: RegionRef = Arc::new(Region::io(
            "flash.regs",
            variant.register_bytes(),
            Arc::new(Registers {
                shared: Arc::clone(&shared),
            }) as Arc<dyn MemOps>,
        ));

        // The directed split the module header argues for: one container, a
        // read-and-execute child on the store and a write-only child on this
        // device. `core::space::flat` resolves reads and writes separately, so
        // a fetch never calls into a device and a store never reaches the array
        // without passing through `CR`.
        let read_side: RegionRef = Arc::new(Region::ram("flash.array", array_store));
        let write_side: RegionRef = Arc::new(Region::io(
            "flash.program",
            size,
            Arc::new(Program {
                shared: Arc::clone(&shared),
            }) as Arc<dyn MemOps>,
        ));
        let array: RegionRef = Arc::new(Region::container(
            "flash",
            size,
            alloc::vec![
                Mapping::new(read_side, 0).with_perms(Perms::RX),
                Mapping::new(write_side, 0).with_perms(Perms::WRITE),
            ],
        ));

        Ok(Flash {
            shared,
            regs,
            array,
        })
    }

    /// Which register layout this instance has.
    #[must_use]
    pub fn variant(&self) -> Variant {
        self.shared.variant
    }

    /// How many bytes of array there are.
    #[must_use]
    pub fn size(&self) -> u64 {
        self.shared.size
    }

    /// `ACR.LATENCY`, the wait-state count firmware asked for.
    #[must_use]
    pub fn latency(&self) -> u32 {
        self.shared.state.lock().word(ACR) & 0b111
    }

    /// Whether `CR.LOCK` is clear.
    #[must_use]
    pub fn unlocked(&self) -> bool {
        let state = self.shared.state.lock();
        self.shared.unlocked(&state)
    }

    /// The array, for a test, a debugger or a loader.
    ///
    /// Never has a side effect on the controller: this is the same side door
    /// [`MemAttrs::debug`] takes.
    #[must_use]
    pub fn contents(&self) -> Vec<u8> {
        let mut out = alloc::vec![0u8; self.shared.size as usize];
        let _ = self.shared.array.read_at(0, &mut out);
        out
    }

    /// Put `bytes` into the array at `offset`, ignoring flash semantics.
    ///
    /// The loader's door, not the guest's.
    ///
    /// # Errors
    ///
    /// [`Error::Config`] if the image runs off the end of the part.
    pub fn load_image(&self, offset: u64, bytes: &[u8]) -> Result<()> {
        self.shared.array.write_at(offset, bytes).map_err(|_| {
            config(format!(
                "an image of {} byte(s) at {offset:#x} does not fit in a flash of {}",
                bytes.len(),
                self.shared.size
            ))
        })
    }

    /// Read one register as the guest would, for a test.
    #[must_use]
    pub fn peek(&self, offset: u64) -> u32 {
        self.shared.read_register(offset & !3)
    }

    /// Write one register as the guest would, for a test. Returns whether the
    /// write asked for a system reset.
    ///
    /// # Errors
    ///
    /// [`BusError::BadAccess`] where the guest would take one — an L4 `KEYR`
    /// write after a wrong key.
    pub fn poke(&self, offset: u64, value: u32) -> MemResult<bool> {
        let after = self.shared.write_register(offset & !3, value)?;
        if after.reset {
            self.shared.pulse_reset();
        }
        Ok(after.reset)
    }

    /// Store into the flash window as the guest would, for a test.
    ///
    /// # Errors
    ///
    /// [`BusError::BadAccess`] for an access off the end of the array.
    pub fn store(&self, offset: u64, src: &[u8]) -> MemResult {
        self.shared.program(offset, src, MemAttrs::DEFAULT)
    }

    /// Connect the interrupt output.
    pub fn connect_irq(&self, source: WireSource) {
        *self.shared.irq_out.lock() = Some(source);
        self.shared.refresh_irq();
    }

    /// Connect the `OBL_LAUNCH` reset output.
    pub fn connect_reset(&self, source: WireSource) {
        *self.shared.reset_out.lock() = Some(source);
    }
}

impl Device for Flash {
    fn class(&self) -> &'static DeviceClass {
        &CLASS
    }

    fn realize(&self, _ctx: &mut RealizeCtx<'_>) -> Result<()> {
        // Nothing outward: the board's `map` statements place both regions and
        // its `wire` statements bring the two outputs.
        Ok(())
    }

    fn reset(&self, _kind: ResetKind) {
        // Both kinds, and **the array survives both**: flash is non-volatile,
        // which is the whole reason this is a device rather than a `ram`
        // object. The option bytes are reloaded from storage, which is what
        // makes `OPTSTRT` mean something.
        {
            let mut state = self.shared.state.lock();
            let tick = state.tick;
            let stored = state.stored;
            *state = State::reset(self.shared.variant, stored);
            state.tick = tick;
            self.shared.republish(&state);
        }
        self.shared.refresh_irq();
    }

    fn save(&self, w: &mut ChunkWriter<'_>) -> Result<()> {
        // The array first and length-prefixed, so a part whose size changed
        // between snapshot and restore fails loudly rather than half-loading.
        w.write_bytes(&self.contents())?;
        let state = *self.shared.state.lock();
        for word in state.words {
            w.write_u32(word)?;
        }
        for word in state.stored {
            w.write_u32(word)?;
        }
        match state.latch {
            Some((offset, value)) => {
                w.write_bool(true)?;
                w.write_u64(offset)?;
                w.write_u32(value)?;
            }
            None => {
                w.write_bool(false)?;
                w.write_u64(0)?;
                w.write_u32(0)?;
            }
        }
        w.write_u64(state.busy_at)?;
        match state.pending {
            Pending::None => {
                w.write_u8(0)?;
                w.write_u64(0)?;
                w.write_u64(0)?;
                w.write_all(&[0u8; 8])?;
            }
            Pending::Erase { offset, len } => {
                w.write_u8(1)?;
                w.write_u64(offset)?;
                w.write_u64(len)?;
                w.write_all(&[0u8; 8])?;
            }
            Pending::Program { offset, bytes, len } => {
                w.write_u8(2)?;
                w.write_u64(offset)?;
                w.write_u64(u64::from(len))?;
                w.write_all(&bytes)?;
            }
            Pending::Option => {
                w.write_u8(3)?;
                w.write_u64(0)?;
                w.write_u64(0)?;
                w.write_all(&[0u8; 8])?;
            }
        }
        w.write_u8(state.key_step)?;
        w.write_u8(state.optkey_step)?;
        w.write_u8(state.pdkey_step)?;
        w.write_bool(state.key_error)?;
        w.write_bool(state.optkey_error)?;
        w.write_u64(state.tick)
        // The wire handles are the machine's wiring, not the chip's state.
    }

    fn load(&self, r: &mut ChunkReader<'_>) -> Result<()> {
        let bytes = r.read_bytes()?;
        if bytes.len() as u64 != self.shared.size {
            return Err(Error::State(format!(
                "the snapshot holds {} byte(s) of flash and this part is {}",
                bytes.len(),
                self.shared.size
            )));
        }
        let mut state = State::reset(self.shared.variant, [0; OPT_WORDS]);
        for word in &mut state.words {
            *word = r.read_u32()?;
        }
        for word in &mut state.stored {
            *word = r.read_u32()?;
        }
        let has_latch = r.read_bool()?;
        let latch_offset = r.read_u64()?;
        let latch_value = r.read_u32()?;
        state.latch = has_latch.then_some((latch_offset, latch_value));
        state.busy_at = r.read_u64()?;
        let tag = r.read_u8()?;
        let offset = r.read_u64()?;
        let len = r.read_u64()?;
        let raw = r.take(8)?;
        let mut bytes8 = [0u8; 8];
        bytes8.copy_from_slice(raw);
        state.pending = match tag {
            0 => Pending::None,
            1 => Pending::Erase { offset, len },
            2 => Pending::Program {
                offset,
                bytes: bytes8,
                len: u8::try_from(len)
                    .map_err(|_| Error::State(format!("a program of {len} byte(s)")))?,
            },
            3 => Pending::Option,
            other => {
                return Err(Error::State(format!(
                    "unknown pending flash operation {other}"
                )));
            }
        };
        state.key_step = r.read_u8()?;
        state.optkey_step = r.read_u8()?;
        state.pdkey_step = r.read_u8()?;
        state.key_error = r.read_bool()?;
        state.optkey_error = r.read_bool()?;
        state.tick = r.read_u64()?;

        self.shared
            .array
            .write_at(0, bytes)
            .map_err(|_| Error::State(String::from("the flash refused its snapshot contents")))?;
        {
            let mut held = self.shared.state.lock();
            *held = state;
            self.shared.republish(&held);
        }
        self.shared.refresh_irq();
        Ok(())
    }

    fn region(&self, name: &str) -> Option<RegionRef> {
        match name {
            "" | "regs" => Some(Arc::clone(&self.regs)),
            "array" => Some(Arc::clone(&self.array)),
            _ => None,
        }
    }

    fn connect(&self, port: &str, source: WireSource) -> Result<()> {
        match port {
            IRQ_PIN => self.connect_irq(source),
            RESET_PIN => self.connect_reset(source),
            _ => {
                return Err(Error::Config {
                    at: port.to_string(),
                    message: format!(
                        "a flash interface drives `{IRQ_PIN}` and `{RESET_PIN}`, not `{port}`"
                    ),
                });
            }
        }
        Ok(())
    }

    fn announce(&self, port: &str) {
        if port == IRQ_PIN {
            self.shared.refresh_irq();
        }
    }

    fn is_lazy(&self) -> bool {
        // A program or an erase takes time, and the guest's poll of `SR.BSY`
        // is what has to see it finish.
        true
    }

    fn current_tick(&self) -> u64 {
        self.shared.tick.load(Ordering::Relaxed)
    }

    fn advance_to(&self, tick: u64) {
        self.shared.advance_to(tick);
    }

    fn next_event_tick(&self) -> Option<u64> {
        match self.shared.next_event.load(Ordering::Relaxed) {
            u64::MAX => None,
            at => Some(at),
        }
    }

    fn attach_lazy(&self, handle: LazyHandle) {
        *self.shared.lazy.lock() = Some(handle);
    }
}

impl Instance for Flash {}

/// The `st.flash` device class.
pub static CLASS: DeviceClass = DeviceClass {
    name: CLASS_NAME,
    version: STATE_VERSION,
    summary: "STM32 embedded flash: the wait states, the unlock keys, and a programmable array",
    properties: &[
        PropertySpec {
            name: "variant",
            kind: ValueKind::Str,
            required: false,
            summary: "which manual: \"f4\" (RM0090 sectors), \"l4\" (RM0351 pages) or \"l4plus\"",
        },
        PropertySpec {
            name: "size",
            kind: ValueKind::Uint,
            required: true,
            summary: "how many bytes of array the part has",
        },
        PropertySpec {
            name: "image",
            kind: ValueKind::Media,
            required: false,
            summary: "a media slot whose bytes the array starts out holding",
        },
        PropertySpec {
            name: "optr",
            kind: ValueKind::Uint,
            required: false,
            summary: "the option register as the option bytes hold it (F4: OPTCR; L4: OPTR)",
        },
        PropertySpec {
            name: "program-time",
            kind: ValueKind::Uint,
            required: false,
            summary: "ticks of this device's clock domain a program takes (default 8)",
        },
        PropertySpec {
            name: "erase-time",
            kind: ValueKind::Uint,
            required: false,
            summary: "ticks a page or sector erase takes (default 64)",
        },
        PropertySpec {
            name: "mass-erase-time",
            kind: ValueKind::Uint,
            required: false,
            summary: "ticks a bank or mass erase takes (default 512)",
        },
    ],
    construct: |props| Ok(Box::new(Flash::new(props)?)),
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
    bindings.bind(CLASS_NAME, |props| Ok(Arc::new(Flash::new(props)?)))
}

/// What the validator should know about `st.flash`.
#[must_use]
pub fn schema() -> ClassSchema {
    ClassSchema::new(CLASS_NAME)
        .prop(PropSchema::new("variant", ValueKind::Str).values(&["f4", "l4", "l4plus"]))
        .prop(PropSchema::new("size", ValueKind::Uint).required())
        .prop(PropSchema::new("image", ValueKind::Media))
        .prop(PropSchema::new("optr", ValueKind::Uint))
        .prop(PropSchema::new("program-time", ValueKind::Uint))
        .prop(PropSchema::new("erase-time", ValueKind::Uint))
        .prop(PropSchema::new("mass-erase-time", ValueKind::Uint))
        .region("")
        .region("regs")
        // The array, as one container whose read half is a store and whose
        // write half is the controller. See the module header.
        .region("array")
        .port(IRQ_PIN, PortDir::Out)
        .port(RESET_PIN, PortDir::Out)
}

#[cfg(test)]
mod tests;
