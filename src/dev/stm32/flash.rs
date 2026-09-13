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
//! **The two families do not share a base address.** RM0090 §2.3 puts the F4's
//! flash interface at `0x4002_3C00`, the kilobyte above the RCC; RM0351 §2.2.2
//! puts the L4's at `0x4002_2000`. Neither is in this file — a base is a `map`
//! statement — but a board derived from the wrong one decodes nothing, and the
//! two numbers being close enough to look interchangeable is how that happens.
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
//!    *directed split* the flattener resolves with
//!    [`FlatEntry::write_to`](crate::core::space::FlatEntry::write_to), which
//!    carries the write winner when it is not the read winner — and it was
//!    built for a Master System cartridge that reads a ROM bank and writes the
//!    RAM behind it. A flash array is the same board one layer in.
//!
//! Option 3 is what is implemented. Reads and fetches resolve to
//! `FlatTarget::Ram` and never call this device at all; writes resolve to the
//! private `Program` handler, which applies RM0090 §3.6 / RM0351 §3.3 and only
//! then touches the store. A part with a read-side protection to enforce is
//! the one exception, and it is opted into rather than paid for by everybody —
//! see "What the read side costs, and when", below.
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
//! # Protection: what a fetch may do that a load may not
//!
//! Three protections, and they are not three of a kind. **`WRP`**, and the
//! program/erase half of **`PCROP`**, are judged where every write already
//! goes — the controller — and need nothing new. The read half is the
//! interesting one.
//!
//! `PCROP` makes a range execute-only: "The protected area is execute-only: it
//! can only be reached by the STM32 CPU, as an instruction code, while all
//! other accesses (DMA, debug and CPU data read, write and erase) are strictly
//! prohibited" (RM0351 §3.5.2). The silicon tells the two apart by **which bus
//! the access arrived on** — `SR.RDERR` is "set by hardware when an address to
//! be read through the D-bus belongs to a read protected area" (§3.7.5) — and
//! **not** by where the program counter is. That is the distinction
//! [`core::space`](crate::core::space) already carries: [`MemAttrs::purpose`]
//! is [`AccessPurpose::FETCH`](crate::core::space::AccessPurpose::FETCH) for
//! the I-code fetch a Cortex-M makes and
//! [`DATA`](crate::core::space::AccessPurpose::DATA) for everything else, so
//! the whole `PCROP` read check is `attrs.is_fetch()` and no PC anywhere.
//!
//! **A literal pool is therefore refused, and that is the hardware.**
//! `LDR r0, =const` inside the protected range is a D-bus read *of* the range
//! *by code in* the range, and the part raises `RDERR` for it exactly as it
//! would for a debugger's read — which is why PCROP'd firmware is compiled
//! execute-only (`-mslow-flash-data`, `armcc --no_literal_pools`; ST AN4701).
//! A model that let the region read itself would happily run firmware that
//! faults on silicon.
//!
//! `RDP` is the other half. Level 1: "In debug mode or when code is running
//! from boot RAM or boot loader, the Flash main memory … \[is\] totally
//! inaccessible. In these modes, a read or write access to the Flash generates
//! a bus error" (RM0351 §3.5.1; RM0090 §3.6.3 says it of the F4). "In debug
//! mode" is [`MemAttrs::debug`] — the access came from a debugger — so **the
//! device refuses its own debug accesses**, and `core::space` never learns
//! what an option byte is. [`Device::debug_halt`] carries the half of the same
//! sentence that no attribute could: RM0351 §3.5.3's note that at level 1 the
//! array cannot be programmed or erased *while the debug features are
//! connected*, which is a rule about a **guest** store made while something
//! has the core stopped.
//!
//! Level 1 → level 0 mass-erases the part (§3.5.1), level 2 is irreversible
//! and refuses every further option write, and a `PCROP` area may be grown but
//! never shrunk. Those are guest-visible, and tested.
//!
//! ## What the read side costs, and when
//!
//! The read child above is a [`RamStore`]: a fetch resolves to
//! `FlatTarget::Ram` and never calls this device at all. A protection that has
//! to *judge* a read needs the read, so a part that has one publishes an I/O
//! read child instead — one virtual call and two relaxed loads per fetch, and
//! no lock. The choice is made once, in [`Flash::new`]: a board arms it by
//! giving option bytes that arm `RDP` or a `PCROP` range (`optr`,
//! `pcrop1sr`/`pcrop1er`/…), or by writing `read-guard = true` for a part
//! whose firmware arms them itself. A region tree is immutable once realized,
//! so this cannot be decided later, and a machine that never mentions
//! protection pays exactly nothing — which is the point.
//!
//! One consequence, stated rather than left to be found: protection is
//! enforced from the **live** option registers (`PCROP`, `WRP`) and from the
//! **stored** option byte (`RDP`), not from a third copy that only an
//! option-byte load updates. So a shadow write takes effect before
//! `OBL_LAUNCH` would have loaded it. The writes that could *weaken* a
//! protection are refused where they arrive — a `PCROP` range may only grow,
//! and `RDP` reads the byte an `OPTSTRT` programmed — so nothing escapes that
//! way; what is missing is the window in which a part is already programmed
//! and not yet reloaded.
//!
//! # What is not modelled
//!
//! * **The caches.** `ACR.ICEN`, `DCEN`, `PRFTEN`, `ICRST` and `DCRST` read
//!   back and change nothing; `LATENCY` is recorded and readable
//!   ([`Flash::latency`]) and costs no time. There is no instruction prefetch
//!   queue here to flush.
//! * **The F4's `PCROP`.** `OPTCR.SPRMOD`, and the inverted `nWRPi` meaning it
//!   brings, are an F42x/F43x feature (RM0090 §3.9.8); this variant decodes an
//!   F405/407 — four-bit `SNB`, no second-bank sector numbers. The four
//!   `pcrop*` properties are refused on `variant = "f4"` rather than silently
//!   accepted. The F4's `RDP` *is* modelled, because every F4 has it.
//! * **What `RDP` erases besides flash.** "The backup registers (RTC_BKPxR in
//!   the RTC) and the SRAM2 are also erased" by a level 1 → 0 regression
//!   (RM0351 §3.5.1). Those belong to other devices and this one has no wire
//!   to them; the flash array is erased, and that is the part that is here.
//! * **Level 2's other half.** That level 2 refuses every option-byte change
//!   and cannot be left is modelled. That it also disables the debug port, the
//!   boot from RAM and the bootloader belongs to the debug and boot plumbing,
//!   not to this register block.
//! * **ECC.** `ECCR` reads as its reset value; nothing injects a correction.
//! * **Bus stalling during an operation.** A real part stalls a fetch from the
//!   bank being erased. Here the array stays readable and only `SR.BSY` says
//!   an operation is running, which is what firmware polls.
//! * **`FSTPG` row timing.** Fast programming is accepted and behaves as
//!   ordinary double-word programming, so `MISERR` and `FASTERR` never set.
//! * **Write-back to a host file.** The array's contents are in the snapshot,
//!   so a settings page survives a save and restore, but nothing writes them
//!   back to the image on `unrealize`. That wants the [`Medium`] seam
//!   [`cfi`](crate::dev::flash::cfi) already uses — a `persist` property and
//!   a `flush` — and it is deliberately not bolted on here: the seam brings a
//!   snapshot policy and a read-only-medium error path with it, and half of
//!   that is worse than none.
//!
//! [`Medium`]: crate::dev::medium::Medium
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
use crate::core::sync::{AtomicBool, AtomicU32, AtomicU64, LockRank, Mutex, Ordering};
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
/// F4 `SR.RDERR`: a `PCROP` read error (RM0090 §3.9.5). The F405/407 this
/// variant decodes has no `PCROP`, so nothing sets it here; the bit is named
/// because it is in the error mask.
const F4_SR_RDERR: u32 = 1 << 8;
/// L4 `SR.RDERR`: "Set by hardware when an address to be read through the
/// D-bus belongs to a read protected area of the flash (PCROP protection)"
/// (RM0351 §3.7.5).
const L4_SR_RDERR: u32 = 1 << 14;

/// Every F4 error bit, for `OPERR` and for the interrupt.
const F4_ERRORS: u32 = SR_OPERR | SR_WRPERR | SR_PGAERR | F4_SR_PGPERR | SR_PGSERR | F4_SR_RDERR;
/// Every L4 error bit.
const L4_ERRORS: u32 = SR_OPERR
    | SR_PROGERR
    | SR_WRPERR
    | SR_PGAERR
    | L4_SR_SIZERR
    | SR_PGSERR
    | (1 << 8/* MISERR */)
    | (1 << 9/* FASTERR */)
    | L4_SR_RDERR
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
/// L4 `CR.RDERRIE` — `SR.RDERR` has its own interrupt enable, which is why a
/// `PCROP` read error is not raised through `fail` with the rest (RM0351 §3.6).
const L4_CR_RDERRIE: u32 = 1 << 26;
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

/// The `RDP` byte that means level 0, no protection (RM0351 Table 14,
/// RM0090 §3.6.3).
const RDP_LEVEL0: u32 = 0xaa;
/// The `RDP` byte that means level 2 — "an irreversible operation".
const RDP_LEVEL2: u32 = 0xcc;
/// Where the `RDP` byte sits in the F4's option register: `OPTCR[15:8]`
/// (RM0090 §3.9.8). An L4 has it in `OPTR[7:0]` (RM0351 §3.7.8).
const F4_OPTCR_RDP_SHIFT: u32 = 8;
/// L4 `PCROP1ER.PCROP_RDP`, a set-only bit: "1: PCROP area is erased when the
/// RDP level is decreased from Level 1 to Level 0 (full mass erase)"
/// (RM0351 §3.7.10).
const L4_PCROP_RDP: u32 = 1 << 31;
/// A `PCROP` range is expressed in double words: "Bank x Base address +
/// [PCROPx_STRT x 0x8] (included) to … [(PCROPx_END+1) x 0x8] (excluded)"
/// (RM0351 §3.5.2).
const PCROP_GRAIN: u64 = 8;

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

/// The `RDP` level a variant's option word encodes (RM0351 Table 14,
/// RM0090 §3.6.3): `0xAA` is level 0, `0xCC` is level 2, **anything else** is
/// level 1 — including a blank byte, which is why a virgin part is protected.
fn rdp_level_of(variant: Variant, word: u32) -> u8 {
    let byte = if variant.is_f4() {
        (word >> F4_OPTCR_RDP_SHIFT) & 0xff
    } else {
        word & 0xff
    };
    match byte {
        RDP_LEVEL0 => 0,
        RDP_LEVEL2 => 2,
        _ => 1,
    }
}

/// The area a `PCROP` start/end pair describes within its own bank, as
/// `(offset, len)`; a zero length is "no area", which an option byte spells by
/// putting the start above the end (RM0351 §3.7.9, and the factory value).
fn pcrop_area(strt: u32, end: u32) -> (u64, u64) {
    let strt = u64::from(strt & 0xffff);
    let end = u64::from(end & 0xffff);
    if strt > end {
        return (0, 0);
    }
    // The end offset is **inclusive**, so the area runs to `(END + 1) * 8`.
    (strt * PCROP_GRAIN, (end + 1 - strt) * PCROP_GRAIN)
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
    ///
    /// The F4 keeps `OPTLOCK` and `OPTSTRT` in the same register as its option
    /// *bytes*, and neither is one: the lock is cleared by the key sequence and
    /// set by a reset ("Reset value: 0x0FFF AAED", bit 0 — RM0090 §3.9.8), and
    /// `OPTSTRT` is a strobe. Storing them as written would make a part come
    /// back from a reset with its option register already unlocked, because
    /// that is how it was when the commit happened.
    fn store_option_bytes(&mut self, variant: Variant) {
        for (slot, &offset) in variant.option_words().iter().enumerate() {
            let mut value = self.words[(offset / 4) as usize];
            if variant.is_f4() && offset == F4_OPTCR {
                value = (value | F4_OPTCR_OPTLOCK) & !F4_OPTCR_OPTSTRT;
            }
            self.stored[slot] = value;
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
    /// The read-side policy, republished lock-free whenever the option
    /// registers move, so that a guarded fetch costs two relaxed loads and no
    /// lock. One entry per bank, packed `start << 32 | end`, both byte offsets
    /// into the array; `0` when that bank has no `PCROP` area.
    pcrop: [AtomicU64; 2],
    /// The `RDP` level in force, likewise published for the read side.
    rdp: AtomicU32,
    /// Whether a debugger has the core stopped ([`Device::debug_halt`]).
    ///
    /// Not guest state and not in the snapshot: it belongs to whatever is
    /// debugging, which says so again the moment it reattaches.
    halted: AtomicBool,
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

    /// Republish what the lock-free lazy and read-side surfaces read.
    fn republish(&self, state: &State) {
        self.rdp
            .store(u32::from(self.rdp_level(state)), Ordering::Relaxed);
        for bank in 0..2 {
            let (offset, len) = self.pcrop_range(state, bank);
            let packed = if len == 0 {
                0
            } else {
                (offset << 32) | (offset + len)
            };
            self.pcrop[bank].store(packed, Ordering::Relaxed);
        }
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
            Pending::Option => self.commit_options(state),
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

    /// Which register holds `RDP`: `OPTCR` on an F4, `OPTR` on an L4.
    fn optr_offset(&self) -> u64 {
        if self.variant.is_f4() {
            F4_OPTCR
        } else {
            L4_OPTR
        }
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

    // -- the protections ----------------------------------------------------

    /// The `RDP` level the option **bytes** hold.
    ///
    /// The stored copy rather than the live register, and the difference
    /// matters: a level change is defined as an option-byte *programming*
    /// event — the 1→0 regression mass-erases the array at the moment it is
    /// programmed (RM0351 §3.5.1) — so the byte is the level. Reading the live
    /// shadow instead would let a guest lower the level by writing a register
    /// it may write at any time, with no erase and no reset.
    fn rdp_level(&self, state: &State) -> u8 {
        rdp_level_of(self.variant, state.stored[0])
    }

    /// The `PCROP` area of `bank`, as `(offset, len)` into the array.
    ///
    /// Read from the live option registers, as write protection is: those are
    /// the values the interface compares against, and the writes that could
    /// weaken them are refused where they arrive ([`Shared::pcrop_write`]).
    /// An F4 has none — `SPRMOD` is an F42x/F43x bit and this variant decodes
    /// an F405/407 (see the module header).
    fn pcrop_range(&self, state: &State, bank: usize) -> (u64, u64) {
        if self.variant.is_f4() {
            return (0, 0);
        }
        let dual = self.dual_bank(state);
        if bank == 1 && !dual {
            return (0, 0);
        }
        let (sr, er) = if bank == 0 {
            (L4_PCROP1SR, L4_PCROP1ER)
        } else {
            (L4_PCROP2SR, L4_PCROP2ER)
        };
        let (within, len) = pcrop_area(state.word(sr), state.word(er));
        if len == 0 {
            return (0, 0);
        }
        let base = if dual {
            (bank as u64) * (self.size / 2)
        } else {
            0
        };
        let offset = base.saturating_add(within);
        if offset >= self.size {
            return (0, 0);
        }
        (offset, len.min(self.size - offset))
    }

    /// Whether `[offset, offset + len)` meets a `PCROP` area, from the
    /// lock-free mirror the read side uses.
    fn pcrop_hit(&self, offset: u64, len: u64) -> bool {
        let end = offset.saturating_add(len);
        self.pcrop.iter().any(|packed| {
            let packed = packed.load(Ordering::Relaxed);
            packed != 0 && offset < (packed & 0xffff_ffff) && end > (packed >> 32)
        })
    }

    /// Whether `[offset, offset + len)` meets a `PCROP` area, with the state
    /// lock held — the program and erase side of the same question.
    fn pcrop_protected(&self, state: &State, offset: u64, len: u64) -> bool {
        let end = offset.saturating_add(len);
        (0..2).any(|bank| {
            let (base, area) = self.pcrop_range(state, bank);
            area != 0 && offset < base + area && end > base
        })
    }

    /// Whether a program or an erase is barred because something is debugging.
    ///
    /// "When the memory read protection level is selected (RDP level = 1), it
    /// is not possible to program or erase Flash memory if the CPU debug
    /// features are connected (JTAG or single wire)" (RM0351 §3.5.3, and the
    /// same note under RM0090's *Write protections*, which adds "even if
    /// nWRPi = 1"). Neither manual names a flag for the refusal; `WRPERR` is
    /// the one both sections are about, and the one firmware polls.
    ///
    /// This is the half of "debug mode" that no access attribute can carry:
    /// the access is the *guest's* own store, made while a debugger has the
    /// core stopped. [`Device::debug_halt`] is the only seam that says so.
    fn debug_locked(&self, state: &State) -> bool {
        self.rdp_level(state) >= 1 && self.halted.load(Ordering::Acquire)
    }

    /// Raise `SR.RDERR` for a `PCROP` read, and nothing else.
    ///
    /// Not through [`Shared::fail`]: `RDERR` is not an operation error — it
    /// has its own enable (`CR.RDERRIE`) and does not set `OPERR`
    /// (RM0351 §3.6, Table 16).
    fn raise_rderr(&self) {
        // The two families put it in different bits of different registers,
        // and `L4_SR`'s offset is the F4's `CR` — so the variant is asked
        // rather than assumed, even though only an L4 can arm `PCROP` here.
        let bit = if self.variant.is_f4() {
            F4_SR_RDERR
        } else {
            L4_SR_RDERR
        };
        {
            let mut state = self.state.lock();
            let sr = self.sr_offset();
            *state.word_mut(sr) |= bit;
        }
        self.refresh_irq();
    }

    /// The array's read side when a protection has to judge it: what
    /// [`Guarded`] answers with.
    ///
    /// The whole of `PCROP` and the read half of `RDP` are decided here, on
    /// two relaxed loads and without the state lock, because this is an
    /// instruction fetch on a Cortex-M board.
    fn read_array(&self, offset: u64, dst: &mut [u8], attrs: MemAttrs) -> MemResult {
        if attrs.debug && self.rdp.load(Ordering::Relaxed) >= 1 {
            // "In debug mode … the Flash main memory … [is] totally
            // inaccessible. In these modes, a read or write access to the
            // Flash generates a bus error" (RM0351 §3.5.1; RM0090 §3.6.3 says
            // the same). The device refuses its own debug access — nothing in
            // `core::space` knows what an option byte is.
            return Err(BusError::Protected);
        }
        if !attrs.is_fetch() && self.pcrop_hit(offset, dst.len() as u64) {
            // "The protected area is execute-only … all other accesses (DMA,
            // debug and CPU data read, write and erase) are strictly
            // prohibited" (RM0351 §3.5.2), and the silicon tells the two apart
            // by the bus the access arrived on, not by where the PC is: "an
            // address to be read through the D-bus" (§3.7.5). A literal-pool
            // load by the protected code itself is a D-bus read and lands
            // here, which is exactly why PCROP firmware is built execute-only.
            dst.fill(0);
            if !attrs.debug {
                // A debug access may not move a status bit (`ROADMAP.md` §15,
                // invariant 5) — it is refused the bytes and nothing else.
                self.raise_rderr();
            }
            return Ok(());
        }
        self.array.read_at(offset, dst)
    }

    // -- the option bytes ----------------------------------------------------

    /// An option register on an L4, with the two rules that stop a protection
    /// from being written away.
    fn write_option_l4(&self, state: &mut State, offset: u64, value: u32) {
        if self.rdp_level(state) == 2 {
            // "only read operations can be performed on the option bytes.
            // Option bytes cannot be programmed nor erased … When attempting
            // to modify the options bytes, the protection error flag WRPERR is
            // set in the Flash_SR register" (RM0351 §3.5.1).
            self.fail(state, SR_WRPERR);
            return;
        }
        let value = match offset {
            L4_PCROP1SR | L4_PCROP1ER | L4_PCROP2SR | L4_PCROP2ER => {
                self.pcrop_write(state, offset, value)
            }
            _ => value,
        };
        *state.word_mut(offset) = value;
    }

    /// What a `PCROP` register write may do: grow the area, never shrink it.
    ///
    /// "If the user options modification tries to clear PCROP or to decrease
    /// the PCROP area, the options programming is launched but PCROP area
    /// stays unchanged. On the contrary, it is possible to increase the PCROP
    /// area" (RM0351 §3.5.2). The manual states that of the option
    /// *programming*; it is applied at the register because this model
    /// enforces from the live registers ([`Shared::pcrop_range`]), and a
    /// shadow write that shrank the area would defeat the protection without
    /// programming anything. `PCROP_RDP` is `rs` — set-only — in hardware
    /// (§3.7.10), and only the full mass erase of an `RDP` regression clears
    /// it.
    fn pcrop_write(&self, state: &State, offset: u64, value: u32) -> u32 {
        let (sr, er) = if offset == L4_PCROP1SR || offset == L4_PCROP1ER {
            (L4_PCROP1SR, L4_PCROP1ER)
        } else {
            (L4_PCROP2SR, L4_PCROP2ER)
        };
        let value = if offset == er {
            value | (state.word(er) & L4_PCROP_RDP)
        } else {
            value
        };
        let (old_start, old_len) = pcrop_area(state.word(sr), state.word(er));
        if old_len == 0 {
            return value;
        }
        let (strt, end) = if offset == sr {
            (value, state.word(er))
        } else {
            (state.word(sr), value)
        };
        let (start, len) = pcrop_area(strt, end);
        let grows = len != 0 && start <= old_start && start + len >= old_start + old_len;
        if grows { value } else { state.word(offset) }
    }

    /// `OPTSTRT`: start programming the option bytes.
    fn start_options(&self, state: &mut State) {
        if self.busy(state) {
            self.fail(state, SR_PGSERR);
            return;
        }
        if self.rdp_level(state) == 2 {
            self.fail(state, SR_WRPERR);
            return;
        }
        // An `RDP` regression erases the part before it reprograms the
        // options, so it costs a mass erase rather than an option-page
        // program (RM0351 §3.5.1).
        let time = if self.regression(state) {
            self.mass_erase_time
        } else {
            self.erase_time
        };
        self.start(state, Pending::Option, time);
    }

    /// Whether committing the live option registers would take `RDP` from
    /// level 1 back to level 0 — the one option write that erases the part.
    fn regression(&self, state: &State) -> bool {
        self.rdp_level(state) == 1
            && rdp_level_of(self.variant, state.word(self.optr_offset())) == 0
    }

    /// What `OPTSTRT` finishes: the live option registers become the option
    /// bytes, unless this is the write that erases the part first.
    fn commit_options(&self, state: &mut State) {
        if self.regression(state) {
            self.regress_rdp(state);
            return;
        }
        state.store_option_bytes(self.variant);
    }

    /// `RDP` level 1 → level 0, which is the only way out of read protection
    /// and costs the user code to take it.
    ///
    /// "When the RDP is reprogrammed to the value 0xAA to move from Level 1 to
    /// Level 0, a mass erase of the Flash main memory is performed if
    /// PCROP_RDP is set … If the bit PCROP_RDP is cleared … the full mass
    /// erase is replaced by a partial mass erase that is successive page
    /// erases … except for the pages protected by PCROP" (RM0351 §3.5.1).
    /// RM0090 §3.6.3 has the same rule for the F4 without the PCROP half, and
    /// adds what both do with the rest: "The other option bytes including
    /// write protections remain unchanged from before the mass-erase
    /// operation."
    ///
    /// The backup registers and SRAM2, which the same sentence erases, are not
    /// this device's to erase and are not modelled — see the module header.
    fn regress_rdp(&self, state: &mut State) {
        let full = self.variant.is_f4() || state.word(L4_PCROP1ER) & L4_PCROP_RDP != 0;
        if full {
            let _ = self.array.fill(0, self.size, 0xff);
        } else {
            let page = self.page.max(1);
            let mut at = 0;
            while at < self.size {
                let len = page.min(self.size - at);
                if !self.pcrop_protected(state, at, len) {
                    let _ = self.array.fill(at, len, 0xff);
                }
                at += page;
            }
        }
        let mut stored = state.stored;
        stored[0] = if self.variant.is_f4() {
            (stored[0] & !(0xff << F4_OPTCR_RDP_SHIFT)) | (RDP_LEVEL0 << F4_OPTCR_RDP_SHIFT)
        } else {
            (stored[0] & !0xff) | RDP_LEVEL0
        };
        if full && !self.variant.is_f4() {
            // "PCROP is disable[d]", and `PCROP_RDP` "is reset after a full
            // mass erase due to a change of RDP from Level 1 to Level 0"
            // (§3.7.10) — so the area goes back to the factory shape, whose
            // start is above its end.
            for (sr, er) in [(L4_PCROP1SR, L4_PCROP1ER), (L4_PCROP2SR, L4_PCROP2ER)] {
                if let Some(slot) = self.option_slot(sr) {
                    stored[slot] = 0x0000_ffff;
                }
                if let Some(slot) = self.option_slot(er) {
                    stored[slot] = 0x0000_0000;
                }
            }
        }
        state.stored = stored;
        // Options, shadow and array agree again: there is no state in which
        // half of a regression has happened. The F4's `OPTLOCK` shares a
        // register with its option bytes and is not one of them — programming
        // the options does not re-lock the interface, only a reset does — so
        // it survives the reload.
        if self.variant.is_f4() {
            let optlock = state.word(F4_OPTCR) & F4_OPTCR_OPTLOCK;
            state.load_option_bytes(self.variant);
            *state.word_mut(F4_OPTCR) = (state.word(F4_OPTCR) & !F4_OPTCR_OPTLOCK) | optlock;
        } else {
            state.load_option_bytes(self.variant);
        }
    }

    /// Where an option register sits in the stored option-byte array.
    fn option_slot(&self, offset: u64) -> Option<usize> {
        self.variant
            .option_words()
            .iter()
            .position(|&o| o == offset)
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
    ///
    /// A `PCROP` area counts: "Any PCROP protected address is also write
    /// protected and any write access to one of these addresses will trigger
    /// WRPERR. Any PCROP area is also erase protected" (RM0351 §3.5.2) — which
    /// is also what makes a mass erase impossible while one is armed, since
    /// the erase asks about the whole bank.
    fn protected(&self, state: &State, offset: u64, len: u64) -> bool {
        if self.pcrop_protected(state, offset, len) {
            return true;
        }
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
            if self.rdp.load(Ordering::Relaxed) >= 1 {
                // The write half of RM0351 §3.5.1: at level 1 the array is
                // inaccessible to a debugger in both directions. The loader's
                // door below is exactly the door read protection closes.
                return Err(BusError::Protected);
            }
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
        if self.protected(state, offset, src.len() as u64) || self.debug_locked(state) {
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
        if self.protected(state, offset, 8) || self.debug_locked(state) {
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
            // `SNB[6:3]`, four bits, as an F405/407 has it (RM0090 §3.9.7).
            // An F42x/F43x widens the field to reach its second bank's
            // sectors 12-23; this model does not decode that width, so on a
            // 2 MiB part those sectors are reachable through `MER1` and not
            // one at a time. `f4_sector` describes them regardless, because
            // the geometry is Table 6 and the field width is this variant.
            f4_sector(self.size, (value >> 3) & 0b1111)
        } else {
            None
        };
        let Some((offset, len)) = range.filter(|&(_, len)| len > 0) else {
            self.fail(state, SR_PGSERR);
            return;
        };
        if self.protected(state, offset, len) || self.debug_locked(state) {
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
        if self.rdp_level(state) == 2 {
            // "User option bytes can no longer be changed" (RM0090 §3.6.3).
            // The F4's manual names no flag for the refusal, so unlike the L4
            // — which names `WRPERR` — nothing is raised.
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
        self.start_options(state);
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
                self.write_option_l4(state, offset, value);
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
        next = (next & !(CR_EOPIE | CR_ERRIE | L4_CR_RDERRIE))
            | (value & (CR_EOPIE | CR_ERRIE | L4_CR_RDERRIE));
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
            self.start_options(state);
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
        if self.protected(state, offset, len) || self.debug_locked(state) {
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
            // On an L4 `RDERR` is enabled by `RDERRIE` and by nothing else
            // (RM0351 Table 16); an F4 has no such bit and `ERRIE` covers it.
            let (errors, rderr) = if self.variant.is_f4() {
                (self.errors_mask(), false)
            } else {
                (
                    self.errors_mask() & !L4_SR_RDERR,
                    sr & L4_SR_RDERR != 0 && cr & L4_CR_RDERRIE != 0,
                )
            };
            let err = sr & errors != 0 && cr & CR_ERRIE != 0;
            Level::from_bool(eop || err || rderr)
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

/// The array's **read** side when a protection has to judge a read.
///
/// Installed in place of the [`RamStore`] child when — and only when — the
/// part has `PCROP` or `RDP` to enforce, because it costs a virtual call per
/// instruction fetch. See the module header.
#[derive(Debug)]
struct Guarded {
    shared: Arc<Shared>,
}

impl MemOps for Guarded {
    fn read(&self, offset: u64, dst: &mut [u8], attrs: MemAttrs) -> MemResult {
        self.shared.read_array(offset, dst, attrs)
    }

    fn write(&self, offset: u64, src: &[u8], attrs: MemAttrs) -> MemResult {
        // Unreachable through the container, whose write winner is `Program`;
        // implemented so that mapping this region alone still obeys `CR`.
        self.shared.program(offset, src, attrs)
    }

    fn constraints(&self) -> AccessConstraints {
        // Everything a store would have taken, bursts included: this stands in
        // for memory, and a `memcpy` out of flash is an ordinary thing to do.
        AccessConstraints::ANY
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
        let pcrop = [
            r.optional::<u64>("pcrop1sr")?,
            r.optional::<u64>("pcrop1er")?,
            r.optional::<u64>("pcrop2sr")?,
            r.optional::<u64>("pcrop2er")?,
        ];
        let read_guard = r.or("read-guard", false)?;
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
        for (name, value) in ["pcrop1sr", "pcrop1er", "pcrop2sr", "pcrop2er"]
            .into_iter()
            .zip(pcrop)
        {
            let Some(value) = value else { continue };
            if variant.is_f4() {
                return Err(config(format!(
                    "`{name}` is an L4 option register; the F4's PCROP is `OPTCR.SPRMOD` on an \
                     F42x/F43x, which this variant does not decode"
                )));
            }
            let value = u32::try_from(value).map_err(|_| {
                config(format!(
                    "`{name}` is a 32-bit register and {value:#x} is not"
                ))
            })?;
            let offset = match name {
                "pcrop1sr" => L4_PCROP1SR,
                "pcrop1er" => L4_PCROP1ER,
                "pcrop2sr" => L4_PCROP2SR,
                _ => L4_PCROP2ER,
            };
            let slot = variant
                .option_words()
                .iter()
                .position(|&o| o == offset)
                .expect("every L4 PCROP register is an option word");
            stored[slot] = value;
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

        // The read side is judged by the device only if there is something to
        // judge: see the module header, "What that costs, and when".
        let state = State::reset(variant, stored);
        let guard = read_guard
            || rdp_level_of(variant, stored[0]) >= 1
            || (!variant.is_f4()
                && [(L4_PCROP1SR, L4_PCROP1ER), (L4_PCROP2SR, L4_PCROP2ER)]
                    .into_iter()
                    .any(|(sr, er)| pcrop_area(state.word(sr), state.word(er)).1 != 0));

        let shared = Arc::new(Shared {
            state: Mutex::with_rank(LockRank::DEVICE, state),
            variant,
            array: Arc::clone(&array_store),
            size,
            page,
            program_time,
            erase_time,
            mass_erase_time,
            reset_out: Mutex::with_rank(LockRank::WIRE, None),
            irq_out: Mutex::with_rank(LockRank::WIRE, None),
            pcrop: [AtomicU64::new(0), AtomicU64::new(0)],
            rdp: AtomicU32::new(0),
            halted: AtomicBool::new(false),
            tick: AtomicU64::new(0),
            next_event: AtomicU64::new(u64::MAX),
            lazy: Mutex::with_rank(LockRank::LEAF, None),
        });

        // The read side's lock-free mirror starts out agreeing with the option
        // bytes rather than with zero.
        {
            let state = shared.state.lock();
            shared.republish(&state);
        }

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
        let read_side: RegionRef = if guard {
            Arc::new(Region::io(
                "flash.guarded",
                size,
                Arc::new(Guarded {
                    shared: Arc::clone(&shared),
                }) as Arc<dyn MemOps>,
            ))
        } else {
            Arc::new(Region::ram("flash.array", array_store))
        };
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

    fn debug_halt(&self, halted: bool) {
        // The half of "debug mode" that no access attribute carries: at `RDP`
        // level 1 the part refuses to program or erase while the debug
        // features are connected, however ordinary the access looks
        // (RM0351 §3.5.3). See [`Shared::debug_locked`].
        self.shared.halted.store(halted, Ordering::Release);
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
            name: "pcrop1sr",
            kind: ValueKind::Uint,
            required: false,
            summary: "L4 FLASH_PCROP1SR as the option bytes hold it: bank 1's PCROP start",
        },
        PropertySpec {
            name: "pcrop1er",
            kind: ValueKind::Uint,
            required: false,
            summary: "L4 FLASH_PCROP1ER: bank 1's PCROP end, and PCROP_RDP in bit 31",
        },
        PropertySpec {
            name: "pcrop2sr",
            kind: ValueKind::Uint,
            required: false,
            summary: "L4 FLASH_PCROP2SR: bank 2's PCROP start, on a dual-bank part",
        },
        PropertySpec {
            name: "pcrop2er",
            kind: ValueKind::Uint,
            required: false,
            summary: "L4 FLASH_PCROP2ER: bank 2's PCROP end",
        },
        PropertySpec {
            name: "read-guard",
            kind: ValueKind::Bool,
            required: false,
            summary: "judge every array read in the controller, for a part whose firmware arms \
                      PCROP or RDP itself (implied when the option bytes arm either)",
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
        .prop(PropSchema::new("pcrop1sr", ValueKind::Uint))
        .prop(PropSchema::new("pcrop1er", ValueKind::Uint))
        .prop(PropSchema::new("pcrop2sr", ValueKind::Uint))
        .prop(PropSchema::new("pcrop2er", ValueKind::Uint))
        .prop(PropSchema::new("read-guard", ValueKind::Bool))
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
