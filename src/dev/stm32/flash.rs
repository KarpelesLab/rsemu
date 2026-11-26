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
//! The **U5** (`variant = "u5"`, STM32U575/U585) is a third register map,
//! written from ST **RM0456** §7 (embedded flash memory) with every offset and
//! bit position taken from `FLASH_TypeDef` and the `FLASH_*_Pos` definitions in
//! ST CMSIS `stm32u585xx.h` (Apache-2.0); ST's STM32U5 HAL
//! (`stm32u5xx_hal_flash.c`, `_flash_ex.c`, BSD-3-Clause) was read for what
//! firmware writes and polls, and none of it is copied here. See "The U5",
//! below.
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
//! That handler is also where the **loader's door** is: a write marked
//! [`MemAttrs::debug`] is a direct poke into the array, because a debugger
//! programming flash drives the part's programming interface over SWD rather
//! than storing through the AHB. `Program` publishes that door, and the
//! sectors behind it, through [`MemOps::flash_layout`] — which is what lets the
//! gdbstub declare the array `flash` in `qXfer:memory-map:read` and serve
//! `vFlashErase`/`vFlashWrite`/`vFlashDone` for it without `core::space` having
//! to relax anything (`src/host/gdb/target.rs`).
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
//! # The U5
//!
//! RM0456 §7 keeps the L4's shape: pages, `PNB`/`BKER`, `MER1`/`MER2`, the
//! `KEYR`/`OPTKEYR` key pairs and `OPTSTRT`/`OBL_LAUNCH`. What differs:
//!
//! * **The map.** TrustZone splits `KEYR`, `SR` and `CR` into non-secure and
//!   secure copies (`NSKEYR`/`SECKEYR` at `+0x08`/`+0x0C`, `NSSR`/`SECSR` at
//!   `+0x20`/`+0x24`, `NSCR`/`SECCR` at `+0x28`/`+0x2C`), `OPTR` moves to
//!   `+0x40`, and the block runs to `PRIVBB2R4` at `+0xFC`. This model is a
//!   **`TZEN = 0`** part — the option-byte default — so firmware drives the
//!   `NS` registers, and every secure-only register (`SECKEYR`, `SECSR`,
//!   `SECCR`, `SECBBxRy`, `SECHDPCR`) reads as zero and ignores writes, and
//!   `OPTR.TZEN` cannot be set. The OEM key registers are write-only and
//!   their `OEMxLOCK` flags never set.
//! * **Geometry.** Always two banks of `size / 2` (1 MiB each on a 2 MiB
//!   U575/U585, 512 KiB on a 1 MiB U575), 8 KiB pages, `PNB[6:0]` counting
//!   within the bank `BKER` names (RM0456 §7.3.1). `SWAP_BANK` is stored and
//!   does not swap the banks' addresses.
//! * **Quad-word programming** (RM0456 §7.3.7). Four 32-bit stores into one
//!   aligned 16-byte quad-word are collected in a write buffer and committed
//!   together on the fourth; while the buffer is partly filled `NSSR.WDW` is
//!   set, which is what HAL's `FLASH_WaitForLastOperation` polls beside `BSY`.
//!   The words may arrive in any order within the quad-word. A store into a
//!   *different* quad-word while one is partly filled is `PGAERR` and drops
//!   the buffer; a byte or half-word store is `SIZERR`; a store with `PG`
//!   clear, with the interface locked, while busy, or while an earlier program
//!   error is still uncleared is `PGSERR`; a quad-word whose cells are not all
//!   ones is `PROGERR` and nothing is written — **except** a quad-word of all
//!   zeroes, which RM0456 allows over programmed cells. `NSCR.BWR` is the
//!   burst variant: the same rules over eight quad-words (128 aligned bytes).
//! * **Flags.** `EOP` is set only when `EOPIE` is, and `OPERR` only when
//!   `ERRIE` is (RM0456 §7.9, `FLASH_NSSR`). `OPTWERR` is the U5's option-write error.
//! * **HAL's mass erase.** `FLASH_MassErase(FLASH_BANK_BOTH)` sets
//!   `MER1|STRT` and then `MER2|STRT` with no wait between them. The model
//!   folds that second strobe into the running bank-1 erase rather than
//!   calling it a sequence error, so the one-call HAL path works.
//! * **Option bytes** are storage plus the protections this file already
//!   enforces: `OPTR` (reset value: `RDP` = `0xAA`, `DUALBANK`, the watchdogs
//!   in software mode, `TZEN` = 0), `NSBOOTADD0R`/`1R`, `SECBOOTADD0R`, the
//!   `SECWMxRy` watermarks (stored, not enforced — no secure world), and
//!   `WRPxAR`/`WRPxBR`, which **are** enforced: two inclusive page ranges per
//!   bank, disabled when the start is above the end, with `UNLOCK` able to
//!   be cleared but set again only at `RDP` level 0. `RDP` levels 0, 1 and 2
//!   behave as above; there is no `PCROP` on a U5. The power-down requests
//!   (`ACR.PDREQx`, `PDKEYxR`) and the low-power read mode are not modelled:
//!   `PDREQx` read back zero and `NSSR.PD1`/`PD2` never set.
//!
//! # The WB
//!
//! `variant = "wb"` is the STM32WB55/WB35 interface (ST **RM0434** §3), at
//! `0x5800_4000` on those parts. Every offset and bit is `FLASH_TypeDef` and
//! the `FLASH_*_Pos` definitions in ST CMSIS `stm32wb55xx.h` v1.12.4
//! (Apache-2.0); ST's STM32WB HAL (`stm32wbxx_hal_flash.c`, BSD-3-Clause) was
//! read for what firmware polls — `BSY`, then `CFGBSY` — and none of it is
//! copied here. It keeps the L4's shape for the registers CPU1 drives
//! (`KEYR` `+0x08`, `SR` `+0x10`, `CR` `+0x14`, `OPTR` `+0x20`) and differs in
//! what a second core and its wireless stack bring:
//!
//! * **One bank of 4 KiB pages** (256 on a 1 MiB WB55, 128 on a 512 KiB WB35;
//!   RM0434 §3.3.1). `CR.PNB[7:0]` names the page; there is no `BKER` and no
//!   `MER2`. Double-word programming as on the L4, two 32-bit stores into one
//!   aligned double word, with the U5's exception: a double word **of all
//!   zeroes** may be programmed over one already programmed ("except if the
//!   data to write is 0x0000 0000 0000 0000", `FLASH_SR.PROGERR`). A program
//!   or erase over an uncleared earlier program error is `PGSERR`.
//! * **`CFGBSY` and `PESD`.** `SR.CFGBSY` is set "when first word is sent for
//!   program, or when setting STRT for erase" and cleared when the operation
//!   ends; HAL's `FLASH_WaitForLastOperation` polls it after `BSY`.
//!   `ACR.PES` (or CPU2's `C2ACR.PES`) suspends any *new* program or erase:
//!   `SR.PESD` reads one, an operation requested meanwhile holds `BSY` and
//!   `CFGBSY` and starts when both `PES` bits clear (RM0434 §3.7).
//! * **The secure area.** `SFR.SFSA` is the first 4 KiB page of the flash
//!   that belongs to CPU2's wireless stack, from there to the end of the
//!   array, unless `SFR.FSD` is set. CPU1 is shut out of it as RM0434 says:
//!   a **program** there "is ignored and a bus error is generated" (§3.3.8);
//!   a page **erase** of a secure page is "aborted" with a CPU1 bus error and
//!   no `WRPERR` (Table 5); a **read** "triggers RDERR" (§3.6.4) and returns
//!   zeroes, like a `PCROP` data read — instruction fetches included, since
//!   the area is "protected against execution, read and write from third
//!   parties"; a **debugger** gets a bus error both ways ("The debugger has
//!   no access to … the secure … memory areas"). `OPTR.ESE` reads as the
//!   inverse of `FSD`. The device's own side door ([`Flash::load_image`],
//!   the `image` property) still writes there, which is how a board installs
//!   the stack image the CPU2 model runs.
//! * **What CPU1 may not do at all.** A **mass erase** "by the CPU1 is
//!   ignored and a bus error is generated" (§3.3.7) — `MER|STRT` is a bus
//!   fault and nothing is erased. `SFR` and `SRRVR` "can only be written by
//!   the CPU2. A write access from the CPU1 is ignored and a bus error
//!   generated" (§3.10.18–19). A `CR` write between the two words of a double
//!   word, or while `PESD` holds an operation, is a bus error; so are a wrong
//!   key and any key written after the unlock it was for (§3.3.8,
//!   "Programming errors causing a bus error"). Setting `LOCK` sets `OPTLOCK`
//!   too (§3.4.2).
//! * **`PCROP`** is two areas, A (`PCROP1ASR`/`PCROP1AER`) and B
//!   (`PCROP1BSR`/`PCROP1BER`), in **2 KiB** units, start above end meaning
//!   none and start equal to end meaning *the whole flash* (RM0434 Table 15).
//!   `WRP1AR`/`WRP1BR` are two 4 KiB page ranges. Both are enforced as on the
//!   L4. An `RDP` 1 → 0 regression erases the non-secure pages (and keeps the
//!   `PCROP` ones if `PCROP_RDP` is clear) — "except for the pages protected
//!   by CPU2 security (SFSA)" (§3.6.1, Table 13).
//! * **CPU2's registers.** `C2ACR`, `C2SR` and `C2CR` are decoded: `C2ACR.PES`
//!   is live, because it suspends CPU1's operations too, `C2SR` reports
//!   `PESD`, and `C2CR` is storage. CPU2's own program and erase engine is
//!   not modelled here; the CPU2 model owns what the stack writes.
//! * **Option bytes**: `OPTR`, `PCROP1ASR`/`AER`/`BSR`/`BER`, `WRP1AR`/`BR`,
//!   `IPCCBR`, `SFR` and `SRRVR` are storage committed by `OPTSTRT` and
//!   loaded by `OBL_LAUNCH`, with `optr`, `sfr` and `srrvr` properties for a
//!   board's own values. The default `SFR` puts `SFSA` where ST's current
//!   full BLE stack installs (`stm32wb5x_BLE_Stack_full_fw.bin` v1.25.0 at
//!   `0x080D_0000` on a 1 MiB part, `0x0805_C000` on 512 KiB and
//!   `0x0801_C000` on 256 KiB — the STM32CubeWB coprocessor binaries release
//!   notes, which also give the rule "the maximum flash memory that can be
//!   used by the application is up to the Secure Flash Start Address"), with
//!   `DDS` set as on a part with a stack installed. The default `SRRVR`
//!   points `SBRV` at `SFSA` (`C2OPT` = 1, flash) and secures SRAM2a from
//!   page `0x0A` and SRAM2b from page `0x0F` — figures reported from a
//!   working WB55 with the BLE stack (ST community), not from a manual.
//! * **Not modelled**: ECC (`ECCR` is its reset value plus `ECCIE`), the OTP
//!   area and the option-byte page in the information block (they are not in
//!   this array), `MISSERR`/`FASTERR` (fast programming behaves as ordinary
//!   programming), the bus stall a `CR` write sees while `CFGBSY` is set by
//!   the other CPU, and the HSI16 the controller switches on by itself.
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
    AccessConstraints, EraseBlocks, FlashLayout, Mapping, MemAttrs, MemOps, MemResult, Perms,
    RamStore, Region, RegionRef,
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

/// How many 32-bit words the widest layout occupies: the U5 map ends at
/// `PRIVBB2R4` (`+0xFC`).
const WORDS: usize = 0x40;

/// How many register words an F4 or L4 snapshot carries. The L4 map ends at
/// `WRP2BR` (`+0x50`), and those snapshots were written as twenty-one words
/// before the U5 widened [`WORDS`]; keeping the count keeps the encoding.
const LEGACY_WORDS: usize = 0x15;

/// How many option words a variant can have. The U5 has twelve (see
/// [`Variant::option_words`]).
const OPT_WORDS: usize = 12;

/// How many option words an F4 or L4 snapshot carries — the L4's nine,
/// as the encoding had them before the U5.
const LEGACY_OPT_WORDS: usize = 9;

/// The widest single program: a U5 burst of eight quad-words.
const PROGRAM_MAX: usize = 128;

/// How many program bytes an F4 or L4 snapshot carries for a pending program.
const LEGACY_PROGRAM_BYTES: usize = 8;

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
    /// RM0456 §7: the STM32U5 — two banks of 8 KiB pages, quad-word
    /// programming, the non-secure register copies of a `TZEN = 0` part.
    U5,
    /// RM0434 §3: the STM32WB55/WB35 — one bank of 4 KiB pages, the L4's
    /// double words, and CPU2's secure area at the top of the array.
    Wb,
}

impl Variant {
    /// The spelling a machine file writes.
    #[must_use]
    pub fn as_str(self) -> &'static str {
        match self {
            Variant::F4 => "f4",
            Variant::L4 => "l4",
            Variant::L4Plus => "l4plus",
            Variant::U5 => "u5",
            Variant::Wb => "wb",
        }
    }

    /// Whether this is the RM0090 layout.
    fn is_f4(self) -> bool {
        matches!(self, Variant::F4)
    }

    /// Whether this is the RM0456 layout.
    fn is_u5(self) -> bool {
        matches!(self, Variant::U5)
    }

    /// Whether this is the RM0351/RM0432 layout.
    fn is_l4(self) -> bool {
        matches!(self, Variant::L4 | Variant::L4Plus)
    }

    /// Whether this is the RM0434 layout.
    fn is_wb(self) -> bool {
        matches!(self, Variant::Wb)
    }

    /// Whether this variant has `PCROP` areas: the L4 family's and the WB's.
    fn has_pcrop(self) -> bool {
        self.is_l4() || self.is_wb()
    }

    /// How many register words a snapshot of this variant carries.
    fn saved_words(self) -> usize {
        if self.is_u5() || self.is_wb() {
            WORDS
        } else {
            LEGACY_WORDS
        }
    }

    /// How many option words a snapshot of this variant carries.
    fn saved_option_words(self) -> usize {
        if self.is_u5() || self.is_wb() {
            OPT_WORDS
        } else {
            LEGACY_OPT_WORDS
        }
    }

    /// How many bytes of a pending program a snapshot carries.
    fn saved_program_bytes(self) -> usize {
        if self.is_u5() {
            PROGRAM_MAX
        } else {
            LEGACY_PROGRAM_BYTES
        }
    }

    /// How many bytes of the peripheral's kilobyte actually decode.
    #[must_use]
    pub fn register_bytes(self) -> u64 {
        match self {
            // Through `OPTCR1` at `+0x18` (RM0090 §3.9.8).
            Variant::F4 => 0x1c,
            // Through `WRP2BR` at `+0x50` (RM0351 §3.7.17).
            Variant::L4 | Variant::L4Plus => 0x54,
            // Through `PRIVBB2R4` at `+0xFC` (`FLASH_TypeDef`).
            Variant::U5 => 0x100,
            // Through `SRRVR` at `+0x84` (`FLASH_TypeDef`, `stm32wb55xx.h`).
            Variant::Wb => 0x88,
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
            Variant::U5 => &[
                U5_OPTR,
                U5_NSBOOTADD0R,
                U5_NSBOOTADD1R,
                U5_SECBOOTADD0R,
                U5_SECWM1R1,
                U5_SECWM1R2,
                U5_WRP1AR,
                U5_WRP1BR,
                U5_SECWM2R1,
                U5_SECWM2R2,
                U5_WRP2AR,
                U5_WRP2BR,
            ],
            Variant::Wb => &[
                L4_OPTR,
                WB_PCROP1ASR,
                WB_PCROP1AER,
                L4_WRP1AR,
                L4_WRP1BR,
                WB_PCROP1BSR,
                WB_PCROP1BER,
                WB_IPCCBR,
                WB_SFR,
                WB_SRRVR,
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

// -- U5 register offsets (ST CMSIS `stm32u585xx.h`, `FLASH_TypeDef`) ---------

/// U5: `FLASH_NSKEYR`.
const U5_NSKEYR: u64 = 0x08;
/// U5: `FLASH_SECKEYR` — secure, so RAZ/WI on a `TZEN = 0` part.
const U5_SECKEYR: u64 = 0x0c;
/// U5: `FLASH_OPTKEYR`.
const U5_OPTKEYR: u64 = 0x10;
/// U5: `FLASH_PDKEY1R`.
const U5_PDKEY1R: u64 = 0x18;
/// U5: `FLASH_PDKEY2R`.
const U5_PDKEY2R: u64 = 0x1c;
/// U5: `FLASH_NSSR`.
const U5_NSSR: u64 = 0x20;
/// U5: `FLASH_NSCR`.
const U5_NSCR: u64 = 0x28;
/// U5: `FLASH_ECCR`.
const U5_ECCR: u64 = 0x30;
/// U5: `FLASH_OPTR`.
const U5_OPTR: u64 = 0x40;
/// U5: `FLASH_NSBOOTADD0R`.
const U5_NSBOOTADD0R: u64 = 0x44;
/// U5: `FLASH_NSBOOTADD1R`.
const U5_NSBOOTADD1R: u64 = 0x48;
/// U5: `FLASH_SECBOOTADD0R`.
const U5_SECBOOTADD0R: u64 = 0x4c;
/// U5: `FLASH_SECWM1R1`.
const U5_SECWM1R1: u64 = 0x50;
/// U5: `FLASH_SECWM1R2`.
const U5_SECWM1R2: u64 = 0x54;
/// U5: `FLASH_WRP1AR`.
const U5_WRP1AR: u64 = 0x58;
/// U5: `FLASH_WRP1BR`.
const U5_WRP1BR: u64 = 0x5c;
/// U5: `FLASH_SECWM2R1`.
const U5_SECWM2R1: u64 = 0x60;
/// U5: `FLASH_SECWM2R2`.
const U5_SECWM2R2: u64 = 0x64;
/// U5: `FLASH_WRP2AR`.
const U5_WRP2AR: u64 = 0x68;
/// U5: `FLASH_WRP2BR`.
const U5_WRP2BR: u64 = 0x6c;
/// U5: `FLASH_OEM1KEYR1`, the first of four write-only OEM key words.
const U5_OEM1KEYR1: u64 = 0x70;
/// U5: `FLASH_OEM2KEYR2`, the last of them.
const U5_OEM2KEYR2: u64 = 0x7c;
/// U5: `FLASH_PRIVCFGR` — `SPRIV` and `NSPRIV`.
const U5_PRIVCFGR: u64 = 0xc4;
/// U5: `FLASH_PRIVBB1R1`, the first privilege block-based word of bank 1.
const U5_PRIVBB1R1: u64 = 0xd0;
/// U5: `FLASH_PRIVBB1R4`.
const U5_PRIVBB1R4: u64 = 0xdc;
/// U5: `FLASH_PRIVBB2R1`.
const U5_PRIVBB2R1: u64 = 0xf0;
/// U5: `FLASH_PRIVBB2R4`.
const U5_PRIVBB2R4: u64 = 0xfc;

/// U5 `ACR`: `LATENCY[3:0]`, `PRFTEN`, `LPM`, `SLEEP_PD`. `PDREQ1`/`PDREQ2`
/// are left out — the bank power-down is not modelled (see the module header).
const U5_ACR_MASK: u32 = 0xf | (1 << 8) | (1 << 11) | (1 << 14);
/// U5 `NSSR.OPTWERR`: an option write that was refused.
const U5_SR_OPTWERR: u32 = 1 << 13;
/// U5 `NSSR.WDW`: the write buffer holds part of a quad-word.
const U5_SR_WDW: u32 = 1 << 17;
/// U5 `NSCR.BWR` — burst programming of eight quad-words.
const U5_CR_BWR: u32 = 1 << 14;
/// U5 `NSCR.PNB[6:0]`, at bit 3.
const U5_CR_PNB: u32 = 0x7f << 3;
/// U5 `OPTR.TZEN`, which this model holds at zero.
const U5_OPTR_TZEN: u32 = 1 << 31;
/// U5 `OPTR` bits a write may change: everything defined but `TZEN`
/// (bit 11 is reserved).
const U5_OPTR_WRITABLE: u32 = 0x7fff_f7ff;
/// U5 `WRPxyR.UNLOCK`.
const U5_WRP_UNLOCK: u32 = 1 << 31;
/// U5 `WRPxyR`: `PSTRT[6:0]`, `PEND[22:16]`, `UNLOCK`.
const U5_WRP_MASK: u32 = 0x807f_007f;
/// U5 `NSBOOTADDxR`/`SECBOOTADD0R`: the address field, bits 31:7.
const U5_BOOTADD_MASK: u32 = 0xffff_ff80;
/// U5 `SECBOOTADD0R.BOOT_LOCK`.
const U5_BOOT_LOCK: u32 = 1 << 0;
/// U5 `SECWMxR1`: `SECWMx_PSTRT`, `SECWMx_PEND`.
const U5_SECWM_R1_MASK: u32 = 0x007f_007f;
/// U5 `SECWMxR2`: `HDPx_PEND`, `HDPxEN`.
const U5_SECWM_R2_MASK: u32 = 0x807f_0000;

// -- WB register offsets (ST CMSIS `stm32wb55xx.h`, `FLASH_TypeDef`) ---------
//
// `ACR`, `KEYR`, `OPTKEYR`, `SR`, `CR`, `ECCR`, `OPTR` and `WRP1AR`/`WRP1BR`
// sit where the L4's do, and the `L4_*` names are used for them.

/// WB: `FLASH_PCROP1ASR` — the L4's `PCROP1SR` offset, in 2 KiB units here.
const WB_PCROP1ASR: u64 = 0x24;
/// WB: `FLASH_PCROP1AER`, with `PCROP_RDP` in bit 31.
const WB_PCROP1AER: u64 = 0x28;
/// WB: `FLASH_PCROP1BSR`.
const WB_PCROP1BSR: u64 = 0x34;
/// WB: `FLASH_PCROP1BER`.
const WB_PCROP1BER: u64 = 0x38;
/// WB: `FLASH_IPCCBR`, the IPCC mailbox buffer's double-word offset in SRAM2.
const WB_IPCCBR: u64 = 0x3c;
/// WB: `FLASH_C2ACR`, CPU2's access control.
const WB_C2ACR: u64 = 0x5c;
/// WB: `FLASH_C2SR`, CPU2's status.
const WB_C2SR: u64 = 0x60;
/// WB: `FLASH_C2CR`, CPU2's control.
const WB_C2CR: u64 = 0x64;
/// WB: `FLASH_SFR`, the secure flash start — CPU2-writable only.
const WB_SFR: u64 = 0x80;
/// WB: `FLASH_SRRVR`, secure SRAM2 starts and the CPU2 reset vector.
const WB_SRRVR: u64 = 0x84;

/// WB `ACR`: `LATENCY[2:0]`, `PRFTEN`, `ICEN`, `DCEN`, `ICRST`, `DCRST`,
/// `PES`, `EMPTY` (RM0434 §3.10.1).
const WB_ACR_MASK: u32 = 0x7 | (0x1f << 8) | WB_ACR_PES | WB_ACR_EMPTY;
/// WB `ACR.PES` / `C2ACR.PES`: suspend new program and erase operations.
const WB_ACR_PES: u32 = 1 << 15;
/// WB `ACR.EMPTY`: the first double word of the user flash is erased.
const WB_ACR_EMPTY: u32 = 1 << 16;
/// WB `ACR` and `C2ACR` out of reset: both caches on (RM0434 §3.10.1, §3.10.15).
const WB_ACR_RESET: u32 = 0x0000_0600;
/// WB `C2ACR`'s writable bits: `PRFTEN`, `ICEN`, `ICRST`, `PES`. Bit 10 is
/// reserved and keeps its reset value.
const WB_C2ACR_MASK: u32 = (1 << 8) | (1 << 9) | (1 << 11) | WB_ACR_PES;
/// WB `C2CR`'s storage bits: `PG`, `PER`, `MER`, `PNB`, `FSTPG`, `EOPIE`,
/// `ERRIE`, `RDERRIE` (RM0434 §3.10.17). `STRT` is a strobe.
const WB_C2CR_MASK: u32 = 0x7ff | L4_CR_FSTPG | CR_EOPIE | CR_ERRIE | L4_CR_RDERRIE;
/// WB `SR.CFGBSY`: a program or erase has its configuration in use.
const WB_SR_CFGBSY: u32 = 1 << 18;
/// WB `SR.PESD`: new program/erase operations are suspended.
const WB_SR_PESD: u32 = 1 << 19;
/// WB `CR.MER` — the one mass erase, which CPU1 may not start.
const WB_CR_MER: u32 = 1 << 2;
/// WB `CR.PNB[7:0]`, at bit 3.
const WB_CR_PNB: u32 = 0xff << 3;
/// The `CR` bits a WB keeps as storage while unlocked.
const WB_CR_STORED: u32 =
    CR_PG | L4_CR_PER | WB_CR_MER | WB_CR_PNB | L4_CR_FSTPG | CR_EOPIE | CR_ERRIE | L4_CR_RDERRIE;
/// The WB program errors that, left uncleared, make the next program or
/// erase a sequence error: `PROGERR`, `WRPERR`, `PGAERR`, `SIZERR`, `PGSERR`,
/// `MISSERR`, `FASTERR` (RM0434 §3.3.8).
const WB_PROGRAM_ERRORS: u32 =
    SR_PROGERR | SR_WRPERR | SR_PGAERR | L4_SR_SIZERR | SR_PGSERR | (1 << 8) | (1 << 9);
/// WB `OPTR.ESE`: read-only, the inverse of `SFR.FSD`.
const WB_OPTR_ESE: u32 = 1 << 8;
/// WB `OPTR`'s writable bits: `RDP`, `BOR_LEV`, the three `nRST_*`, the
/// watchdog options, `nBOOT1`, `SRAM2_PE`, `SRAM2_RST`, `nSWBOOT0`, `nBOOT0`
/// and `AGC_TRIM` (RM0434 §3.10.7).
const WB_OPTR_WRITABLE: u32 = 0xef8f_7eff;
/// WB `PCROPxySR`/`PCROPxyER`: the 2 KiB page field, `[8:0]`.
const WB_PCROP_FIELD: u32 = 0x1ff;
/// WB `PCROP1AER.PCROP_RDP` — the L4's bit, in the same place.
const WB_PCROP_RDP: u32 = L4_PCROP_RDP;
/// WB `WRP1xR`: `WRP1x_STRT[7:0]` and `WRP1x_END[23:16]`.
const WB_WRP_MASK: u32 = 0x00ff_00ff;
/// WB `IPCCBR.IPCCDBA[13:0]`.
const WB_IPCCBR_MASK: u32 = 0x3fff;
/// WB `SFR.FSD`: "System and flash memory non-secure".
const WB_SFR_FSD: u32 = 1 << 8;
/// WB `SFR.DDS`: CPU2 debug access disabled.
const WB_SFR_DDS: u32 = 1 << 12;
/// WB `SFR.SFSA[7:0]`.
const WB_SFR_SFSA: u32 = 0xff;
/// A WB page: 4 KiB (RM0434 §3.3.1), which is also `SFSA`'s and `WRP`'s unit.
pub const WB_PAGE: u64 = 4 * 1024;
/// A WB `PCROP` unit: 2 KiB (RM0434 §3.6.2).
const WB_PCROP_GRAIN: u64 = 2 * 1024;

/// Where ST's full BLE stack v1.25.0 installs, as a page number, by flash
/// size — and so the default `SFSA` (STM32CubeWB
/// `Projects/STM32WB_Copro_Wireless_Binaries/STM32WB5x/Release_Notes.html`
/// and `STM32WB3x/Release_Notes.html`, "Wireless Coprocessor Binary Table",
/// `stm32wb5x_BLE_Stack_full_fw.bin` / `stm32wb3x_BLE_Stack_full_fw.bin`).
/// A size not in the table defaults to no secure area at all.
#[must_use]
pub fn wb_default_sfsa(size: u64) -> Option<u32> {
    match size {
        0x10_0000 => Some(0xd0),
        0x0a_0000 => Some(0x7c),
        0x08_0000 => Some(0x5c),
        0x04_0000 => Some(0x1c),
        _ => None,
    }
}

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
/// Every U5 error bit (RM0456 `FLASH_NSSR`).
const U5_ERRORS: u32 =
    SR_OPERR | SR_PROGERR | SR_WRPERR | SR_PGAERR | L4_SR_SIZERR | SR_PGSERR | U5_SR_OPTWERR;
/// The U5 program errors that, left uncleared, make the next program a
/// sequence error.
const U5_PROGRAM_ERRORS: u32 = SR_PROGERR | SR_WRPERR | SR_PGAERR | L4_SR_SIZERR | SR_PGSERR;

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

/// The bits of the U5 `NSCR` that `LOCK` protects.
const U5_CR_OPERATION: u32 =
    CR_PG | L4_CR_PER | L4_CR_MER1 | U5_CR_PNB | L4_CR_BKER | U5_CR_BWR | L4_CR_MER2 | CR_STRT;

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

/// A U5 page: 8 KiB (RM0456 §7.3.1).
const U5_PAGE: u64 = 8 * 1024;

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

/// The area a WB `PCROPxySR`/`PCROPxyER` pair describes, as `(offset, len)`
/// into a `size`-byte array (RM0434 §3.6.2, Table 15): 2 KiB units from
/// `STRT` to `END` inclusive, none when the start is above the end — and the
/// **whole flash** when they are equal.
fn wb_pcrop_area(strt: u32, end: u32, size: u64) -> (u64, u64) {
    let strt = u64::from(strt & WB_PCROP_FIELD);
    let end = u64::from(end & WB_PCROP_FIELD);
    if strt > end {
        return (0, 0);
    }
    if strt == end {
        return (0, size);
    }
    let offset = (strt * WB_PCROP_GRAIN).min(size);
    let stop = ((end + 1) * WB_PCROP_GRAIN).min(size);
    (offset, stop - offset)
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
        bytes: [u8; PROGRAM_MAX],
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
    /// The U5's write buffer: the aligned base of the quad-word (or burst)
    /// being filled, one bit per word that has arrived — zero when empty —
    /// and the words themselves.
    wbuf_base: u64,
    wbuf_mask: u32,
    wbuf: [u8; PROGRAM_MAX],
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
    /// WB: the ticks an operation requested while `PESD` was set still needs
    /// once the suspension lifts; zero when nothing is suspended.
    suspended: u64,
}

impl State {
    fn reset(variant: Variant, stored: [u32; OPT_WORDS]) -> State {
        let mut words = [0u32; WORDS];
        if variant.is_f4() {
            // "Reset value: 0x8000 0000" — the interface comes up locked
            // (RM0090 §3.9.7).
            words[(F4_CR / 4) as usize] = CR_LOCK;
        } else if variant.is_u5() {
            // `ACR` resets to zero wait states with the prefetch off, and
            // `NSCR` with both locks set (RM0456 §7.9).
            words[(U5_NSCR / 4) as usize] = CR_LOCK | L4_CR_OPTLOCK;
        } else if variant.is_wb() {
            // "Reset value: 0x0000 0600" for `ACR` and `C2ACR`, "0xC000 0000"
            // for `CR` (RM0434 §3.10.1, §3.10.5, §3.10.15). `ACR.EMPTY` is the
            // empty check's and the caller sets it.
            words[(ACR / 4) as usize] = WB_ACR_RESET;
            words[(WB_C2ACR / 4) as usize] = WB_ACR_RESET;
            words[(L4_CR / 4) as usize] = CR_LOCK | L4_CR_OPTLOCK;
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
            wbuf_base: 0,
            wbuf_mask: 0,
            wbuf: [0xff; PROGRAM_MAX],
            busy_at: NO_DEADLINE,
            pending: Pending::None,
            key_step: 0,
            optkey_step: 0,
            pdkey_step: 0,
            key_error: false,
            optkey_error: false,
            tick: 0,
            suspended: 0,
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
        Variant::U5 => {
            // `OPTR` for a dual-bank `TZEN = 0` part at `RDP` level 0:
            // `RDP` = 0xAA, `BOR_LEV` = 0, the three `nRST_*` and both
            // `SRAMx_RST`, both watchdogs in software mode and running in
            // Stop/Standby, `DUALBANK`, the three SRAM ECC disables (active
            // low), `nSWBOOT0`, `nBOOT0` and `PA15_PUPEN` all set; `SWAP_BANK`,
            // the two `HSLV` bits and `TZEN` clear (RM0456 §7.4; the bit
            // positions are `FLASH_OPTR_*` in `stm32u585xx.h`). A board with
            // other option bytes writes `optr`.
            out[0] = 0x1fef_f8aa;
            // Boot address 0 is the user flash, boot address 1 the system
            // bootloader at `0x0BF9_0000`; the low seven bits are reserved.
            out[1] = 0x0800_007f;
            out[2] = 0x0bf9_007f;
            // The secure boot address, which a `TZEN = 0` part never uses.
            out[3] = 0x0c00_007c;
            // No secure watermark area (start above end) and no HDP area.
            out[4] = 0x0000_007f;
            out[5] = 0x0000_0000;
            // `WRP` areas disabled — start page above end page — and
            // unlocked.
            out[6] = 0x8000_007f;
            out[7] = 0x8000_007f;
            out[8] = 0x0000_007f;
            out[9] = 0x0000_0000;
            out[10] = 0x8000_007f;
            out[11] = 0x8000_007f;
        }
        Variant::Wb => {
            // `OPTR` as ST ships it, "0b001x 1101 1xxx 1111 x111 0001 1010
            // 1010" with the `x` bits zero: `AGC_TRIM` = 1, `nBOOT0`,
            // `nSWBOOT0`, `SRAM2_PE` (parity off), `nBOOT1`, both watchdogs in
            // software mode and running in Stop/Standby, the three `nRST_*`,
            // `BOR_LEV` = 0, `RDP` = 0xAA. `ESE` (bit 8) is not stored: it
            // reads back from `SFR.FSD` (RM0434 §3.4.1, §3.10.7).
            out[0] = 0x2d8f_70aa;
            // `PCROP` A and B start above where they end — no area — and
            // `PCROP_RDP` set, as shipped (§3.4.1).
            out[1] = WB_PCROP_FIELD;
            out[2] = WB_PCROP_RDP;
            // `WRP` A and B the same way: start page 0xFF, end page 0.
            out[3] = 0x0000_00ff;
            out[4] = 0x0000_00ff;
            out[5] = WB_PCROP_FIELD;
            out[6] = 0;
            // `IPCCBR`: "0bXXXX … XX00 0000 0000 0000", offset zero.
            out[7] = 0;
            // `SFR` and `SRRVR` depend on the part's size; `Flash::new`
            // fills them. Until then, no secure area.
            out[8] = WB_SFR_DDS | WB_SFR_FSD;
            out[9] = 0;
        }
    }
    out
}

/// The default WB `SFR` and `SRRVR` for a part of `size` bytes: the secure
/// area where ST's full BLE stack installs (see [`wb_default_sfsa`]), CPU2's
/// debug disabled, CPU2 booting from `SFSA` in flash, and SRAM2a/SRAM2b secure
/// from 1 KiB page `0x0A`/`0x0F` (see the module header for where those come
/// from). A size the table does not know gets no secure area.
fn wb_default_secure(size: u64) -> (u32, u32) {
    let Some(sfsa) = wb_default_sfsa(size) else {
        return (WB_SFR_DDS | WB_SFR_FSD, 0);
    };
    let sfr = WB_SFR_DDS | sfsa;
    // `C2OPT` (31) = flash; `NBRSD` (30) = 0, `SNBRSA[29:25]` = 0x0F; `BRSD`
    // (23) = 0, `SBRSA[22:18]` = 0x0A; `SBRV[17:0]` the word offset of SFSA.
    let sbrv = (sfsa << 12) / 4;
    let srrvr = (1 << 31) | (0x0f << 25) | (0x0a << 18) | sbrv;
    (sfr, srrvr)
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
    /// WB: the byte offset CPU2's secure area starts at, likewise published;
    /// `size` when there is none.
    secure: AtomicU64,
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
        self.secure
            .store(self.secure_start(state), Ordering::Relaxed);
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
                let len = usize::from(len).min(PROGRAM_MAX);
                let mut old = [0u8; PROGRAM_MAX];
                let _ = self.array.read_at(offset, &mut old[..len]);
                let mut new = [0u8; PROGRAM_MAX];
                for i in 0..len {
                    new[i] = old[i] & bytes[i];
                }
                let _ = self.array.write_at(offset, &new[..len]);
            }
            Pending::Option => self.commit_options(state),
        }
        let sr = self.sr_offset();
        *state.word_mut(sr) &= !SR_BSY;
        // On a U5 "this bit is set only if the end of operation interrupts
        // are enabled (EOPIE = 1)" (RM0456 §7.9, `FLASH_NSSR`), and RM0434
        // §3.10.4 says the same of the WB's `FLASH_SR`.
        let eop_needs_ie = self.variant.is_u5() || self.variant.is_wb();
        if !eop_needs_ie || state.word(self.cr_offset()) & CR_EOPIE != 0 {
            *state.word_mut(sr) |= SR_EOP;
        }
    }

    // -- layout helpers ------------------------------------------------------

    fn sr_offset(&self) -> u64 {
        match self.variant {
            Variant::F4 => F4_SR,
            Variant::U5 => U5_NSSR,
            Variant::L4 | Variant::L4Plus | Variant::Wb => L4_SR,
        }
    }

    fn cr_offset(&self) -> u64 {
        match self.variant {
            Variant::F4 => F4_CR,
            Variant::U5 => U5_NSCR,
            Variant::L4 | Variant::L4Plus | Variant::Wb => L4_CR,
        }
    }

    /// Which register holds `RDP`: `OPTCR` on an F4, `OPTR` on an L4 or a U5.
    fn optr_offset(&self) -> u64 {
        match self.variant {
            Variant::F4 => F4_OPTCR,
            Variant::U5 => U5_OPTR,
            Variant::L4 | Variant::L4Plus | Variant::Wb => L4_OPTR,
        }
    }

    fn errors_mask(&self) -> u32 {
        match self.variant {
            Variant::F4 => F4_ERRORS,
            Variant::U5 => U5_ERRORS,
            // The WB's `SR` has the L4's error bits in the L4's places
            // (RM0434 §3.10.4).
            Variant::L4 | Variant::L4Plus | Variant::Wb => L4_ERRORS,
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
            state.word(self.cr_offset()) & L4_CR_OPTLOCK == 0
        }
    }

    fn busy(&self, state: &State) -> bool {
        state.word(self.sr_offset()) & SR_BSY != 0
    }

    /// Whether `OPTR.DUALBANK` says there are two banks (RM0351 §3.4.1 bit 21).
    ///
    /// A U5 is always two banks of `size / 2` here: its `DUALBANK` matters
    /// only to the smaller parts' single-bank mode, which is not modelled.
    fn dual_bank(&self, state: &State) -> bool {
        match self.variant {
            // RM0434 §3.2: "single bank architecture".
            Variant::F4 | Variant::Wb => false,
            Variant::U5 => true,
            Variant::L4 | Variant::L4Plus => state.word(L4_OPTR) & (1 << 21) != 0,
        }
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
        if self.variant.is_wb() {
            // On a WB the two "banks" are areas A and B of the one bank.
            let (sr, er) = if bank == 0 {
                (WB_PCROP1ASR, WB_PCROP1AER)
            } else {
                (WB_PCROP1BSR, WB_PCROP1BER)
            };
            return wb_pcrop_area(state.word(sr), state.word(er), self.size);
        }
        // A U5 has no `PCROP` at all; its execute-only areas are the secure
        // world's, which a `TZEN = 0` part does not have.
        if !self.variant.is_l4() {
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

    /// WB: where CPU2's secure area starts, as a byte offset into the array —
    /// `SFSA` pages, or the end of the array when `SFR.FSD` is set or on any
    /// other variant (RM0434 §3.6.4).
    fn secure_start(&self, state: &State) -> u64 {
        if !self.variant.is_wb() {
            return self.size;
        }
        let sfr = state.word(WB_SFR);
        if sfr & WB_SFR_FSD != 0 {
            return self.size;
        }
        (u64::from(sfr & WB_SFR_SFSA) * WB_PAGE).min(self.size)
    }

    /// Whether `[offset, offset + len)` reaches into CPU2's secure area, from
    /// the lock-free mirror.
    fn secure_hit(&self, offset: u64, len: u64) -> bool {
        offset.saturating_add(len) > self.secure.load(Ordering::Relaxed)
    }

    /// WB: whether `ACR.PES` or `C2ACR.PES` is set — `SR.PESD`.
    fn pesd(&self, state: &State) -> bool {
        self.variant.is_wb() && (state.word(ACR) | state.word(WB_C2ACR)) & WB_ACR_PES != 0
    }

    /// WB: the empty check the option-byte loader makes, into `ACR.EMPTY`
    /// (RM0434 §3.3.2): set when the first double word of the user flash is
    /// erased.
    fn empty_check(&self, state: &mut State) {
        if !self.variant.is_wb() {
            return;
        }
        let mut first = [0u8; 8];
        let _ = self.array.read_at(0, &mut first);
        let acr = state.word(ACR) & !WB_ACR_EMPTY;
        *state.word_mut(ACR) = acr | if first == [0xff; 8] { WB_ACR_EMPTY } else { 0 };
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
        if self.secure_hit(offset, dst.len() as u64) {
            // CPU2's secure area. "The debugger has no access to … the secure
            // … memory areas", and "any CPU1 access to a CPU2 security area
            // triggers RDERR" (RM0434 §3.6.4): the bytes are zeroes, as for a
            // `PCROP` data read, and a fetch is no exception — the area is
            // "protected against execution, read and write from third
            // parties".
            if attrs.debug {
                return Err(BusError::Protected);
            }
            dst.fill(0);
            self.raise_rderr();
            return Ok(());
        }
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
        // Only the L4 family and the WB have `PCROP`, so only they can make
        // this partial. `PCROP_RDP` lives in the first area's end register on
        // both (`PCROP1ER` = `PCROP1AER`, `+0x28`).
        let keep_pcrop = self.variant.has_pcrop() && state.word(L4_PCROP1ER) & L4_PCROP_RDP == 0;
        // "A requested mass erase performs a partial mass erase (a sequence of
        // page erases), except for the pages protected by CPU2 security
        // (SFSA). This is done to keep the CPU2 secure code" (RM0434 §3.6.1).
        let secure = self.secure_start(state);
        let full = !keep_pcrop && secure >= self.size;
        if full {
            let _ = self.array.fill(0, self.size, 0xff);
        } else {
            let page = self.page.max(1);
            let mut at = 0;
            while at < self.size {
                let len = page.min(self.size - at);
                let kept = (keep_pcrop && self.pcrop_protected(state, at, len))
                    || at.saturating_add(len) > secure;
                if !kept {
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
        if !keep_pcrop && self.variant.has_pcrop() {
            // "PCROP is disable[d]", and `PCROP_RDP` "is reset after a full
            // mass erase due to a change of RDP from Level 1 to Level 0"
            // (§3.7.10) — so the area goes back to the factory shape, whose
            // start is above its end. (A WB keeping its secure pages still
            // erased every `PCROP` page, so its areas go too.)
            let pairs = if self.variant.is_wb() {
                [(WB_PCROP1ASR, WB_PCROP1AER), (WB_PCROP1BSR, WB_PCROP1BER)]
            } else {
                [(L4_PCROP1SR, L4_PCROP1ER), (L4_PCROP2SR, L4_PCROP2ER)]
            };
            for (sr, er) in pairs {
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
        // The U5 has the F4's rule: "This bit is set only if error interrupts
        // are enabled (ERRIE = 1)" (RM0456 §7.9, `FLASH_NSSR`).
        // So has the WB (RM0434 §3.10.4: "set only if error interrupts are
        // enabled").
        let operr_needs_ie = self.variant.is_f4() || self.variant.is_u5() || self.variant.is_wb();
        let operr = if operr_needs_ie && !errie {
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
        match self.variant {
            Variant::F4 => self.f4_protected(state, offset, len),
            Variant::U5 => self.u5_protected(state, offset, len),
            // The WB's `WRP1AR`/`WRP1BR` are the L4's bank-1 pair, 4 KiB pages
            // and eight-bit fields, in a part with one bank.
            Variant::L4 | Variant::L4Plus | Variant::Wb => self.l4_protected(state, offset, len),
        }
    }

    /// `WRPxAR`/`WRPxBR` on a U5: two inclusive page ranges per bank in
    /// `PSTRT[6:0]`/`PEND[22:16]`, disabled when the start page is above the
    /// end page (RM0456 §7.6, `FLASH_WRP1AR`).
    fn u5_protected(&self, state: &State, offset: u64, len: u64) -> bool {
        let bank_len = self.size / 2;
        let end = offset.saturating_add(len.saturating_sub(1));
        for (reg, bank) in [
            (U5_WRP1AR, 0u64),
            (U5_WRP1BR, 0),
            (U5_WRP2AR, 1),
            (U5_WRP2BR, 1),
        ] {
            let value = state.word(reg);
            let (strt, stop) = (u64::from(value & 0x7f), u64::from((value >> 16) & 0x7f));
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

    /// The array's erase geometry, as a loader outside the guest sees it.
    ///
    /// The F4's sectors are unequal (RM0090 Table 5), so equal-sized runs are
    /// coalesced and the result is three runs per bank rather than a blocksize
    /// that is a lie about eleven of the twelve. The L4's pages are uniform
    /// (RM0351 §3.2, RM0432 §3.3.1), so it is one run whatever `DUALBANK`
    /// says: both banks have the same page size and the two are contiguous.
    fn layout(&self) -> FlashLayout {
        if !self.variant.is_f4() {
            return FlashLayout::uniform(self.size, self.page);
        }
        let mut blocks: Vec<EraseBlocks> = Vec::new();
        for snb in 0..24 {
            let Some((offset, len)) = f4_sector(self.size, snb) else {
                continue;
            };
            match blocks.last_mut() {
                Some(run) if run.blocksize == len && run.offset + run.length == offset => {
                    run.length += len;
                }
                _ => blocks.push(EraseBlocks {
                    offset,
                    length: len,
                    blocksize: len,
                }),
            }
        }
        FlashLayout {
            blocks,
            erased: 0xff,
        }
    }

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
            if self.secure_hit(offset, src.len() as u64) {
                // Except into CPU2's secure area, which a debugger cannot
                // reach on a WB (RM0434 §3.6.4).
                return Err(BusError::Protected);
            }
            return self.array.write_at(offset, src);
        }
        if offset
            .checked_add(src.len() as u64)
            .is_none_or(|e| e > self.size)
        {
            return Err(BusError::BadAccess);
        }
        self.sync(attrs);
        let result = {
            let mut state = self.state.lock();
            let result = match self.variant {
                Variant::F4 => {
                    self.program_f4(&mut state, offset, src);
                    Ok(())
                }
                Variant::U5 => {
                    self.program_u5(&mut state, offset, src);
                    Ok(())
                }
                Variant::L4 | Variant::L4Plus => {
                    self.program_l4(&mut state, offset, src);
                    Ok(())
                }
                Variant::Wb => self.program_wb(&mut state, offset, src),
            };
            self.republish(&state);
            result
        };
        self.refresh_irq();
        result
    }

    /// RM0434 §3.3.8: the L4's double word, with the WB's bus errors.
    ///
    /// The one variant whose program path *does* raise bus errors, because
    /// RM0434 lists them: a program into the secure area "is ignored and a
    /// bus error is generated", and so is "requesting a new programming
    /// request, when the previous one has not finished".
    fn program_wb(&self, state: &mut State, offset: u64, src: &[u8]) -> MemResult {
        if src.len() == 8 {
            // A 64-bit store is its two words, in order.
            self.program_wb(state, offset, &src[..4])?;
            return self.program_wb(state, offset + 4, &src[4..]);
        }
        let cr = state.word(L4_CR);
        if !self.unlocked(state) || cr & (CR_PG | L4_CR_FSTPG) == 0 {
            state.latch = None;
            self.fail(state, SR_PGSERR);
            return Ok(());
        }
        if src.len() != 4 {
            // "Any attempt to write byte (8 bits) or half-word (16 bits) sets
            // SIZERR".
            state.latch = None;
            self.fail(state, L4_SR_SIZERR);
            return Ok(());
        }
        let value = u32::from_le_bytes([src[0], src[1], src[2], src[3]]);
        let Some((first, low)) = state.latch else {
            if !offset.is_multiple_of(8) {
                self.fail(state, SR_PGAERR);
                return Ok(());
            }
            if self.secure_hit(offset, 8) {
                return Err(BusError::Protected);
            }
            if self.busy(state) {
                return Err(BusError::BadAccess);
            }
            if state.word(L4_SR) & WB_PROGRAM_ERRORS != 0 {
                // "PGSERR is set also if PROGERR, SIZERR, PGAERR, WRPERR,
                // MISSERR, FASTERR, or PGSERR is set due to a previous
                // programming error."
                self.fail(state, SR_PGSERR);
                return Ok(());
            }
            if self.protected(state, offset, 8) || self.debug_locked(state) {
                self.fail(state, SR_WRPERR);
                return Ok(());
            }
            // The first word sets `CFGBSY` (read back from the latch) and
            // nothing else; the second one starts the program.
            state.latch = Some((offset, value));
            return Ok(());
        };
        state.latch = None;
        if offset != first.wrapping_add(4) {
            self.fail(state, SR_PGAERR);
            return Ok(());
        }
        let mut old = [0u8; 8];
        let _ = self.array.read_at(first, &mut old);
        if old != [0xff; 8] && (low != 0 || value != 0) {
            // "PROGERR … except if the data to write is 0x0000 0000 0000
            // 0000" (RM0434 §3.10.4).
            self.fail(state, SR_PROGERR);
            return Ok(());
        }
        let mut bytes = [0u8; PROGRAM_MAX];
        bytes[..4].copy_from_slice(&low.to_le_bytes());
        bytes[4..8].copy_from_slice(&value.to_le_bytes());
        self.start(
            state,
            Pending::Program {
                offset: first,
                bytes,
                len: 8,
            },
            self.program_time,
        );
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
        let mut bytes = [0u8; PROGRAM_MAX];
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
                let mut bytes = [0u8; PROGRAM_MAX];
                bytes[..4].copy_from_slice(&low.to_le_bytes());
                bytes[4..8].copy_from_slice(&value.to_le_bytes());
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

    /// RM0456 §7.3.7: one quad-word — or, with `BWR`, eight — collected from
    /// 32-bit stores in the write buffer and programmed when it is full.
    fn program_u5(&self, state: &mut State, offset: u64, src: &[u8]) {
        let cr = state.word(U5_NSCR);
        if !self.unlocked(state) || cr & CR_PG == 0 {
            // A store with `PG` clear or the interface locked programs
            // nothing and is a sequence error; any half-filled quad-word goes
            // with it.
            self.u5_drop_buffer(state);
            self.fail(state, SR_PGSERR);
            return;
        }
        if src.is_empty() || !src.len().is_multiple_of(4) {
            // "SIZERR … set when the size of the access is a byte or a
            // half-word during a program sequence. Only quad-word programming
            // is allowed by means of successive word accesses".
            self.u5_drop_buffer(state);
            self.fail(state, L4_SR_SIZERR);
            return;
        }
        // A double-word store (`STRD`, a 64-bit bus access) is two word
        // writes to the buffer.
        for (i, &value) in src.as_chunks::<4>().0.iter().enumerate() {
            if !self.program_word_u5(state, offset + 4 * i as u64, value) {
                return;
            }
        }
    }

    /// One word into the U5 write buffer. `false` if it raised an error.
    fn program_word_u5(&self, state: &mut State, offset: u64, value: [u8; 4]) -> bool {
        let burst = state.word(U5_NSCR) & U5_CR_BWR != 0;
        let unit: u64 = if burst { PROGRAM_MAX as u64 } else { 16 };
        let base = offset & !(unit - 1);
        if !offset.is_multiple_of(4) {
            self.u5_drop_buffer(state);
            self.fail(state, SR_PGAERR);
            return false;
        }
        if state.wbuf_mask == 0 {
            // The first word of a quad-word starts a program, so this is
            // where the conditions on starting one are checked: nothing
            // running, no earlier program error left uncleared.
            let sr = state.word(U5_NSSR);
            if self.busy(state) || sr & U5_PROGRAM_ERRORS != 0 {
                self.fail(state, SR_PGSERR);
                return false;
            }
            if self.protected(state, base, unit) || self.debug_locked(state) {
                self.fail(state, SR_WRPERR);
                return false;
            }
            state.wbuf_base = base;
            state.wbuf = [0xff; PROGRAM_MAX];
        } else if base != state.wbuf_base {
            // "PGAERR … set when the data to program cannot be contained in
            // the same quad-word (128-bit) flash memory in case of standard
            // programming", and the half-filled quad-word is lost.
            self.u5_drop_buffer(state);
            self.fail(state, SR_PGAERR);
            return false;
        }
        let slot = ((offset - base) / 4) as usize;
        state.wbuf[slot * 4..slot * 4 + 4].copy_from_slice(&value);
        state.wbuf_mask |= 1 << slot;
        let full = if burst { u32::MAX } else { 0xf };
        if state.wbuf_mask != full {
            *state.word_mut(U5_NSSR) |= U5_SR_WDW;
            return true;
        }
        let len = unit as usize;
        let bytes = state.wbuf;
        self.u5_drop_buffer(state);
        // Every quad-word must be erased first — "PROGERR … set when a
        // quad-word address to be programmed contains a value different from
        // all 1 before programming, except if the data to write is all 0" —
        // and a burst fails as a whole.
        let mut old = [0u8; PROGRAM_MAX];
        let _ = self.array.read_at(base, &mut old[..len]);
        let refused = old[..len]
            .as_chunks::<16>()
            .0
            .iter()
            .zip(bytes[..len].as_chunks::<16>().0)
            .any(|(was, new)| was.iter().any(|&b| b != 0xff) && new.iter().any(|&b| b != 0));
        if refused {
            self.fail(state, SR_PROGERR);
            return false;
        }
        self.start(
            state,
            Pending::Program {
                offset: base,
                bytes,
                len: len as u8,
            },
            self.program_time,
        );
        true
    }

    /// Forget a half-filled U5 quad-word, and `WDW` with it.
    fn u5_drop_buffer(&self, state: &mut State) {
        if !self.variant.is_u5() {
            return;
        }
        state.wbuf_mask = 0;
        state.wbuf = [0xff; PROGRAM_MAX];
        *state.word_mut(U5_NSSR) &= !U5_SR_WDW;
    }

    /// Set `BSY` and arm the deadline.
    fn start(&self, state: &mut State, pending: Pending, ticks: u64) {
        let sr = self.sr_offset();
        *state.word_mut(sr) |= SR_BSY;
        state.pending = pending;
        if self.pesd(state) {
            // "All new requested program and erase operations are not
            // started, but suspended" until both `PES` bits clear, with
            // `BSY` and `CFGBSY` asserted meanwhile (RM0434 §3.7, §3.10.5).
            state.suspended = ticks.max(1);
            state.busy_at = NO_DEADLINE;
            return;
        }
        state.busy_at = state.tick.saturating_add(ticks.max(1));
    }

    /// WB: a `PES` bit moved. If neither is set any more, a suspended
    /// operation starts now.
    fn resume(&self, state: &mut State) {
        if state.suspended == 0 || self.pesd(state) {
            return;
        }
        state.busy_at = state.tick.saturating_add(state.suspended);
        state.suspended = 0;
    }

    // -- the register block --------------------------------------------------

    fn read_register(&self, offset: u64) -> u32 {
        let state = self.state.lock();
        // The key registers are write-only; reading one returns zero rather
        // than the last key written, so a snapshot of the bus never carries it.
        let write_only = match self.variant {
            Variant::F4 => offset == F4_KEYR || offset == F4_OPTKEYR,
            Variant::U5 => {
                matches!(
                    offset,
                    U5_NSKEYR | U5_SECKEYR | U5_OPTKEYR | U5_PDKEY1R | U5_PDKEY2R
                ) || (U5_OEM1KEYR1..=U5_OEM2KEYR2).contains(&offset)
            }
            Variant::L4 | Variant::L4Plus => {
                offset == L4_KEYR || offset == L4_OPTKEYR || offset == L4_PDKEYR
            }
            Variant::Wb => offset == L4_KEYR || offset == L4_OPTKEYR,
        };
        if write_only {
            return 0;
        }
        if self.variant.is_wb() {
            let pesd = if self.pesd(&state) { WB_SR_PESD } else { 0 };
            match offset {
                L4_SR => {
                    // `CFGBSY` is "set when first word is sent for program, or
                    // when setting STRT for erase" and reset when the
                    // operation ends: the latch, or `BSY`.
                    let cfgbsy = if self.busy(&state) || state.latch.is_some() {
                        WB_SR_CFGBSY
                    } else {
                        0
                    };
                    return state.word(L4_SR) | cfgbsy | pesd;
                }
                WB_C2SR => return state.word(WB_C2SR) | pesd,
                L4_OPTR => {
                    // `ESE` "indicates whether the system security is enabled
                    // user flash memory FSD = 0".
                    let ese = if state.word(WB_SFR) & WB_SFR_FSD == 0 {
                        WB_OPTR_ESE
                    } else {
                        0
                    };
                    return (state.word(L4_OPTR) & !WB_OPTR_ESE) | ese;
                }
                _ => {}
            }
        }
        state.word(offset)
    }

    /// Write one register. Returns what must happen outside the lock.
    fn write_register(&self, offset: u64, value: u32) -> MemResult<After> {
        let after = {
            let mut state = self.state.lock();
            let result = match self.variant {
                Variant::F4 => self
                    .write_f4(&mut state, offset, value)
                    .map(|()| After::default()),
                Variant::U5 => self.write_u5(&mut state, offset, value),
                Variant::L4 | Variant::L4Plus => self.write_l4(&mut state, offset, value),
                Variant::Wb => self.write_wb(&mut state, offset, value),
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

    fn write_u5(&self, state: &mut State, offset: u64, value: u32) -> MemResult<After> {
        let mut after = After::default();
        match offset {
            ACR => *state.word_mut(ACR) = value & U5_ACR_MASK,
            U5_NSKEYR => self.write_key(state, value)?,
            U5_OPTKEYR => self.write_optkey(state, value)?,
            U5_PDKEY1R => {
                // Tracked and no more: the bank power-down it unlocks is not
                // modelled, so `PDREQ1` reads back zero whatever is written.
                state.pdkey_step = match (state.pdkey_step, value) {
                    (0, PDKEY1) => 1,
                    _ => 0,
                };
            }
            U5_NSSR => {
                // Write one to clear; `BSY`, `WDW`, the OEM locks and the
                // power-down flags are hardware's.
                let clear = value & (SR_EOP | U5_ERRORS);
                *state.word_mut(U5_NSSR) &= !clear;
            }
            U5_NSCR => after = self.write_cr_u5(state, value),
            U5_ECCR => {
                // `ECCIE` is a plain enable; `ECCC` and `ECCD` are
                // write-1-to-clear and nothing here sets them.
                let eccr = state.word(U5_ECCR);
                *state.word_mut(U5_ECCR) =
                    ((eccr & !(1 << 24)) | (value & (1 << 24))) & !(value & 0xc000_0000);
            }
            U5_PRIVCFGR => *state.word_mut(U5_PRIVCFGR) = value & 0b11,
            U5_PRIVBB1R1..=U5_PRIVBB1R4 | U5_PRIVBB2R1..=U5_PRIVBB2R4 => {
                // Storage: privilege is not enforced on the array here.
                *state.word_mut(offset) = value;
            }
            U5_OPTR | U5_NSBOOTADD0R | U5_NSBOOTADD1R | U5_SECBOOTADD0R | U5_SECWM1R1
            | U5_SECWM1R2 | U5_WRP1AR | U5_WRP1BR | U5_SECWM2R1 | U5_SECWM2R2 | U5_WRP2AR
            | U5_WRP2BR
                if self.opt_unlocked(state) =>
            {
                self.write_option_u5(state, offset, value);
            }
            // `SECKEYR`, `PDKEY2R`, `SECSR`, `SECCR`, `OPSR`, the OEM keys, the
            // `SECBB` words and `SECHDPCR`: secure, write-only with nothing
            // behind them, or read-only. RAZ/WI on a `TZEN = 0` part.
            _ => {}
        }
        Ok(after)
    }

    /// A U5 option register: each keeps its reserved bits, `TZEN` stays zero
    /// and a `WRP` area's `UNLOCK` can be cleared but not set again above
    /// `RDP` level 0.
    fn write_option_u5(&self, state: &mut State, offset: u64, value: u32) {
        if self.rdp_level(state) == 2 {
            // At level 2 the option bytes are frozen; `OPTWERR` is the U5's
            // flag for an option write that was not allowed.
            self.fail(state, U5_SR_OPTWERR);
            return;
        }
        let old = state.word(offset);
        let mask = match offset {
            U5_OPTR => U5_OPTR_WRITABLE,
            U5_NSBOOTADD0R | U5_NSBOOTADD1R => U5_BOOTADD_MASK,
            U5_SECBOOTADD0R => U5_BOOTADD_MASK | U5_BOOT_LOCK,
            U5_SECWM1R1 | U5_SECWM2R1 => U5_SECWM_R1_MASK,
            U5_SECWM1R2 | U5_SECWM2R2 => U5_SECWM_R2_MASK,
            _ => U5_WRP_MASK,
        };
        let mut new = (old & !mask) | (value & mask);
        if matches!(offset, U5_WRP1AR | U5_WRP1BR | U5_WRP2AR | U5_WRP2BR) {
            if old & U5_WRP_UNLOCK == 0 && self.rdp_level(state) != 0 {
                // A locked area cannot be moved or unlocked until an `RDP`
                // regression takes the part back to level 0.
                new = old;
            } else if old & U5_WRP_UNLOCK == 0 {
                new = (new & !U5_WRP_UNLOCK) | (value & U5_WRP_UNLOCK);
            }
        }
        *state.word_mut(offset) = new;
    }

    fn write_cr_u5(&self, state: &mut State, value: u32) -> After {
        let mut after = After::default();
        let before = state.word(U5_NSCR);
        let locked = before & CR_LOCK != 0;
        let opt_locked = before & L4_CR_OPTLOCK != 0;

        // Both locks are set by software and cleared only by their key
        // sequences. Unlike an L4, setting `OPTLOCK` leaves `LOCK` alone:
        // RM0456 describes them as two independent set-only bits.
        let mut next = before;
        if !locked {
            next = (next & !U5_CR_OPERATION) | (value & U5_CR_OPERATION);
        }
        next = (next & !(CR_EOPIE | CR_ERRIE)) | (value & (CR_EOPIE | CR_ERRIE));
        next |= value & (CR_LOCK | L4_CR_OPTLOCK);
        // `STRT`, `OPTSTRT` and `OBL_LAUNCH` are strobes, not storage.
        next &= !(CR_STRT | L4_CR_OPTSTRT | L4_CR_OBL_LAUNCH);
        *state.word_mut(U5_NSCR) = next;

        if value & CR_STRT != 0 && !locked {
            self.erase_u5(state, value);
        }
        if value & L4_CR_OPTSTRT != 0 && !opt_locked {
            self.start_options(state);
        }
        if value & L4_CR_OBL_LAUNCH != 0 {
            if opt_locked || self.busy(state) {
                self.fail(state, SR_PGSERR);
            } else {
                state.load_option_bytes(self.variant);
                after.reset = true;
            }
        }
        after
    }

    /// `NSCR.STRT` on a U5: a page, a bank, or both banks.
    fn erase_u5(&self, state: &mut State, value: u32) {
        let bank_len = self.size / 2;
        let both = L4_CR_MER1 | L4_CR_MER2;
        if self.busy(state) {
            // HAL's `FLASH_MassErase(FLASH_BANK_BOTH)` writes `MER1|STRT` and
            // then `MER2|STRT` without waiting in between (see the module
            // header). Fold the second into the bank-1 erase already running
            // rather than refusing it.
            if value & both == both
                && state.pending
                    == (Pending::Erase {
                        offset: 0,
                        len: bank_len,
                    })
            {
                if self.protected(state, bank_len, bank_len) {
                    self.fail(state, SR_WRPERR);
                } else {
                    state.pending = Pending::Erase {
                        offset: 0,
                        len: self.size,
                    };
                    state.busy_at = state
                        .busy_at
                        .max(state.tick.saturating_add(self.mass_erase_time.max(1)));
                }
                return;
            }
            self.fail(state, SR_PGSERR);
            return;
        }
        if state.word(U5_NSSR) & U5_PROGRAM_ERRORS != 0
            || value & CR_PG != 0
            || state.wbuf_mask != 0
        {
            // An erase may not start over an uncleared program error, beside
            // `PG`, or with a quad-word half-written.
            self.fail(state, SR_PGSERR);
            return;
        }
        let range = if value & both == both {
            Some((0, self.size))
        } else if value & L4_CR_MER1 != 0 {
            Some((0, bank_len))
        } else if value & L4_CR_MER2 != 0 {
            Some((bank_len, bank_len))
        } else if value & L4_CR_PER != 0 {
            let pnb = u64::from((value & U5_CR_PNB) >> 3);
            let within = pnb * self.page;
            let bank = if value & L4_CR_BKER != 0 { bank_len } else { 0 };
            (within + self.page <= bank_len).then_some((bank + within, self.page))
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
        let time = if value & both != 0 {
            self.mass_erase_time
        } else {
            self.erase_time
        };
        self.start(state, Pending::Erase { offset, len }, time);
    }

    fn write_wb(&self, state: &mut State, offset: u64, value: u32) -> MemResult<After> {
        let mut after = After::default();
        match offset {
            ACR => {
                *state.word_mut(ACR) = value & WB_ACR_MASK;
                self.resume(state);
            }
            L4_KEYR => self.write_key(state, value)?,
            L4_OPTKEYR => self.write_optkey(state, value)?,
            L4_SR => {
                // Write one to clear; `BSY`, `CFGBSY`, `PESD` and `OPTNV` are
                // hardware's.
                let clear = value & (SR_EOP | L4_ERRORS);
                *state.word_mut(L4_SR) &= !clear;
            }
            L4_CR => after = self.write_cr_wb(state, value)?,
            L4_ECCR => {
                // `ECCIE` is a plain enable; `ECCC` and `ECCD` are
                // write-1-to-clear and nothing here sets them.
                let eccr = state.word(L4_ECCR);
                *state.word_mut(L4_ECCR) =
                    ((eccr & !(1 << 24)) | (value & (1 << 24))) & !(value & 0xc000_0000);
            }
            L4_OPTR | WB_PCROP1ASR | WB_PCROP1AER | L4_WRP1AR | L4_WRP1BR | WB_PCROP1BSR
            | WB_PCROP1BER | WB_IPCCBR
                if self.opt_unlocked(state) =>
            {
                self.write_option_wb(state, offset, value);
            }
            WB_C2ACR => {
                // CPU2's, but a `PES` here suspends CPU1's operations too.
                let old = state.word(WB_C2ACR);
                *state.word_mut(WB_C2ACR) = (value & WB_C2ACR_MASK) | (old & !WB_C2ACR_MASK);
                self.resume(state);
            }
            WB_C2CR => {
                // Storage: CPU2's own program and erase engine is not modelled
                // here (see the module header). `STRT` is a strobe.
                *state.word_mut(WB_C2CR) = value & WB_C2CR_MASK;
            }
            WB_SFR | WB_SRRVR => {
                // "This register provides write access security and can only
                // be written by the CPU2. A write access from the CPU1 is
                // ignored and a bus error generated" (RM0434 §3.10.18–19).
                return Err(BusError::Protected);
            }
            // `C2SR`'s flags are CPU2's to clear, and nothing sets them here.
            _ => {}
        }
        Ok(after)
    }

    /// A WB option register, kept to its defined bits, with the L4's rules on
    /// `RDP` level 2 and on shrinking a `PCROP` area.
    fn write_option_wb(&self, state: &mut State, offset: u64, value: u32) {
        if self.rdp_level(state) == 2 {
            // "When attempting to modify the options bytes, the protection
            // error flag WRPERR is set" (RM0434 §3.6.1).
            self.fail(state, SR_WRPERR);
            return;
        }
        let old = state.word(offset);
        let new = match offset {
            L4_OPTR => (old & !WB_OPTR_WRITABLE) | (value & WB_OPTR_WRITABLE),
            L4_WRP1AR | L4_WRP1BR => value & WB_WRP_MASK,
            WB_IPCCBR => value & WB_IPCCBR_MASK,
            WB_PCROP1AER => {
                // `PCROP_RDP` is set-only (`rs`).
                (value & (WB_PCROP_FIELD | WB_PCROP_RDP)) | (old & WB_PCROP_RDP)
            }
            _ => value & WB_PCROP_FIELD,
        };
        let new = match offset {
            WB_PCROP1ASR | WB_PCROP1AER | WB_PCROP1BSR | WB_PCROP1BER => {
                // "If the user options modification tries to clear PCROP or to
                // decrease the PCROP areas, the options programming is
                // launched but PCROP areas stays unchanged" (RM0434 §3.6.2).
                let (sr, er) = if offset == WB_PCROP1ASR || offset == WB_PCROP1AER {
                    (WB_PCROP1ASR, WB_PCROP1AER)
                } else {
                    (WB_PCROP1BSR, WB_PCROP1BER)
                };
                let (old_start, old_len) = wb_pcrop_area(state.word(sr), state.word(er), self.size);
                let (strt, end) = if offset == sr {
                    (new, state.word(er))
                } else {
                    (state.word(sr), new)
                };
                let (start, len) = wb_pcrop_area(strt, end, self.size);
                let grows = old_len == 0
                    || (len != 0 && start <= old_start && start + len >= old_start + old_len);
                if grows {
                    new
                } else {
                    // The set-only bit still sticks.
                    old | (new & WB_PCROP_RDP)
                }
            }
            _ => new,
        };
        *state.word_mut(offset) = new;
    }

    fn write_cr_wb(&self, state: &mut State, value: u32) -> MemResult<After> {
        let mut after = After::default();
        if state.latch.is_some() {
            // "FLASH_CR register write between the two accesses of a double
            // word programming" is a bus error (RM0434 §3.3.8).
            return Err(BusError::BadAccess);
        }
        if self.pesd(state) && self.busy(state) {
            // "When the PESD bit … is set and a program or an erase operation
            // is ongoing, the register write access causes a bus error"
            // (§3.10.5).
            return Err(BusError::BadAccess);
        }
        let before = state.word(L4_CR);
        if before & CR_LOCK != 0 {
            // "After reset, write is not allowed in the FLASH_CR" until the
            // key sequence (§3.3.6); `LOCK` is already set, and it implies
            // `OPTLOCK`, so there is nothing a write could do.
            return Ok(after);
        }
        let opt_locked = before & L4_CR_OPTLOCK != 0;
        let mut next = (before & !WB_CR_STORED) | (value & WB_CR_STORED);
        // Both locks are set-only; "If LOCK is set by software, OPTLOCK is
        // automatically set as well" (§3.4.2).
        if value & CR_LOCK != 0 {
            next |= CR_LOCK | L4_CR_OPTLOCK;
        }
        if value & L4_CR_OPTLOCK != 0 {
            next |= L4_CR_OPTLOCK;
        }
        *state.word_mut(L4_CR) = next & !(CR_STRT | L4_CR_OPTSTRT | L4_CR_OBL_LAUNCH);

        if value & CR_STRT != 0 {
            self.erase_wb(state, value)?;
        }
        if value & L4_CR_OPTSTRT != 0 && !opt_locked {
            self.start_options(state);
        }
        if value & L4_CR_OBL_LAUNCH != 0 && !opt_locked {
            if self.busy(state) {
                self.fail(state, SR_PGSERR);
            } else {
                state.load_option_bytes(self.variant);
                after.reset = true;
            }
        }
        Ok(after)
    }

    /// `CR.STRT` on a WB: a page erase. CPU1 cannot mass-erase, nor erase a
    /// secure page.
    fn erase_wb(&self, state: &mut State, value: u32) -> MemResult {
        if value & WB_CR_MER != 0 {
            // "A flash memory mass erase by the CPU1 is ignored and a bus
            // error is generated" (RM0434 §3.3.7, Table 6).
            return Err(BusError::Protected);
        }
        if self.busy(state)
            || state.word(L4_SR) & WB_PROGRAM_ERRORS != 0
            || value & (CR_PG | L4_CR_FSTPG) != 0
            || value & L4_CR_PER == 0
        {
            // Over a running operation, over an uncleared program error, with
            // `PG`/`FSTPG` beside `PER`, or with nothing selected.
            self.fail(state, SR_PGSERR);
            return Ok(());
        }
        let page = u64::from((value & WB_CR_PNB) >> 3);
        let offset = page * WB_PAGE;
        if offset + WB_PAGE > self.size {
            self.fail(state, SR_PGSERR);
            return Ok(());
        }
        if self.secure_hit(offset, WB_PAGE) {
            // A secure page "requested by CPU1": "Page erase aborted (no page
            // erase started)", a CPU1 bus error and no `WRPERR` (Table 5).
            return Err(BusError::Protected);
        }
        if self.protected(state, offset, WB_PAGE) || self.debug_locked(state) {
            self.fail(state, SR_WRPERR);
            return Ok(());
        }
        self.start(
            state,
            Pending::Erase {
                offset,
                len: WB_PAGE,
            },
            self.erase_time,
        );
        Ok(())
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
            // A WB makes "any subsequent write to FLASH_KEYR … after
            // unlocking the respective feature" a bus error (RM0434 §3.3.8).
            return if self.variant.is_wb() {
                Err(BusError::BadAccess)
            } else {
                Ok(())
            };
        }
        match (state.key_step, value) {
            (0, KEY1) => state.key_step = 1,
            (1, KEY2) => {
                state.key_step = 0;
                *state.word_mut(cr) &= !CR_LOCK;
            }
            _ => {
                // "In case of a wrong key sequence … the FLASH_CR register is
                // locked until the next reset." A WB faults on the wrong key
                // itself: "a bus error is detected and a hard fault interrupt
                // is generated" (RM0434 §3.3.6).
                state.key_step = 0;
                state.key_error = true;
                if self.variant.is_wb() {
                    return Err(BusError::BadAccess);
                }
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
            return if self.variant.is_wb() {
                Err(BusError::BadAccess)
            } else {
                Ok(())
            };
        }
        match (state.optkey_step, value) {
            (0, OPTKEY1) => state.optkey_step = 1,
            (1, OPTKEY2) => {
                state.optkey_step = 0;
                if self.variant.is_f4() {
                    *state.word_mut(F4_OPTCR) &= !F4_OPTCR_OPTLOCK;
                } else {
                    let cr = self.cr_offset();
                    *state.word_mut(cr) &= !L4_CR_OPTLOCK;
                }
            }
            _ => {
                state.optkey_step = 0;
                state.optkey_error = true;
                if self.variant.is_wb() {
                    return Err(BusError::BadAccess);
                }
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
            let (errors, rderr) = if !self.variant.has_pcrop() {
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

    fn flash_layout(&self) -> Option<FlashLayout> {
        // The promise this makes is the one [`Shared::program`] already keeps:
        // a debug write here is the loader's door — a direct poke into the
        // array that moves no status bit — so a debugger can put an image in
        // without unlocking anything, and an erase is that poke with `0xff`,
        // which is what a sector reads as afterwards (RM0090 §3.6.3).
        Some(self.shared.layout())
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
        let variant = match r.or_enum("variant", "f4", &["f4", "l4", "l4plus", "u5", "wb"])? {
            "l4" => Variant::L4,
            "l4plus" => Variant::L4Plus,
            "u5" => Variant::U5,
            "wb" => Variant::Wb,
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
        let sfr = r.optional::<u64>("sfr")?;
        let srrvr = r.optional::<u64>("srrvr")?;
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
        if variant.is_u5() && (!size.is_multiple_of(2 * U5_PAGE) || size / 2 / U5_PAGE > 128) {
            return Err(config(format!(
                "a U5 array is two banks of up to 128 8 KiB pages (RM0456 §7.3.1), and {size} \
                 byte(s) is not"
            )));
        }
        if variant.is_wb() && (!size.is_multiple_of(WB_PAGE) || size > 256 * WB_PAGE) {
            return Err(config(format!(
                "a WB array is one bank of up to 256 4 KiB pages (RM0434 §3.3.1), and {size} \
                 byte(s) is not"
            )));
        }
        if !variant.is_wb() && (sfr.is_some() || srrvr.is_some()) {
            return Err(config(String::from(
                "`sfr` and `srrvr` are the WB's secure option registers; only `wb` has them",
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
            if variant.is_u5() {
                return Err(config(format!(
                    "`{name}` is an L4 option register; a U5 has no PCROP"
                )));
            }
            if variant.is_wb() && name.starts_with("pcrop2") {
                return Err(config(format!(
                    "`{name}` is the L4's second bank; a WB's areas are PCROP1A (`pcrop1sr`, \
                     `pcrop1er`) and PCROP1B, which only firmware programs here"
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
            Variant::U5 => U5_PAGE,
            Variant::Wb => WB_PAGE,
        };
        if variant.is_wb() {
            let (default_sfr, default_srrvr) = wb_default_secure(size);
            let word = |name: &str, value: Option<u64>, default: u32| -> Result<u32> {
                value.map_or(Ok(default), |v| {
                    u32::try_from(v).map_err(|_| {
                        config(format!(
                            "`{name}` is a 32-bit option register and {v:#x} is not"
                        ))
                    })
                })
            };
            let slot = |offset: u64| {
                variant
                    .option_words()
                    .iter()
                    .position(|&o| o == offset)
                    .expect("SFR and SRRVR are WB option words")
            };
            stored[slot(WB_SFR)] =
                word("sfr", sfr, default_sfr)? & (WB_SFR_DDS | WB_SFR_FSD | WB_SFR_SFSA);
            stored[slot(WB_SRRVR)] = word("srrvr", srrvr, default_srrvr)? & !(1 << 24);
        }
        if variant.is_u5() && stored[0] & U5_OPTR_TZEN != 0 {
            return Err(config(String::from(
                "`optr` sets TZEN, and this model is a TZEN = 0 part: TrustZone's secure flash \
                 world is not modelled",
            )));
        }

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
            || (variant.is_l4()
                && [(L4_PCROP1SR, L4_PCROP1ER), (L4_PCROP2SR, L4_PCROP2ER)]
                    .into_iter()
                    .any(|(sr, er)| pcrop_area(state.word(sr), state.word(er)).1 != 0))
            // A WB with a secure area judges every read, because CPU1 may not
            // read it; so does one whose option bytes arm a PCROP area.
            || (variant.is_wb()
                && (state.word(WB_SFR) & WB_SFR_FSD == 0
                    || [(WB_PCROP1ASR, WB_PCROP1AER), (WB_PCROP1BSR, WB_PCROP1BER)]
                        .into_iter()
                        .any(|(sr, er)| wb_pcrop_area(state.word(sr), state.word(er), size).1 != 0)));

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
            secure: AtomicU64::new(size),
            halted: AtomicBool::new(false),
            tick: AtomicU64::new(0),
            next_event: AtomicU64::new(u64::MAX),
            lazy: Mutex::with_rank(LockRank::LEAF, None),
        });

        // The read side's lock-free mirror starts out agreeing with the option
        // bytes rather than with zero.
        {
            let mut state = shared.state.lock();
            shared.empty_check(&mut state);
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
        // Three bits on an F4 or L4, four on a U5 (`FLASH_ACR_LATENCY_Msk`).
        let mask = if self.shared.variant.is_u5() {
            0xf
        } else {
            0b111
        };
        self.shared.state.lock().word(ACR) & mask
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
            self.shared.empty_check(&mut state);
            self.shared.republish(&state);
        }
        self.shared.refresh_irq();
    }

    fn save(&self, w: &mut ChunkWriter<'_>) -> Result<()> {
        // The array first and length-prefixed, so a part whose size changed
        // between snapshot and restore fails loudly rather than half-loading.
        w.write_bytes(&self.contents())?;
        let variant = self.shared.variant;
        let state = *self.shared.state.lock();
        // An F4 or L4 writes exactly the shape it always has; only a U5's
        // chunk is wider (see `LEGACY_WORDS`).
        for &word in &state.words[..variant.saved_words()] {
            w.write_u32(word)?;
        }
        for &word in &state.stored[..variant.saved_option_words()] {
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
        let program_bytes = variant.saved_program_bytes();
        let zeros = [0u8; PROGRAM_MAX];
        match state.pending {
            Pending::None => {
                w.write_u8(0)?;
                w.write_u64(0)?;
                w.write_u64(0)?;
                w.write_all(&zeros[..program_bytes])?;
            }
            Pending::Erase { offset, len } => {
                w.write_u8(1)?;
                w.write_u64(offset)?;
                w.write_u64(len)?;
                w.write_all(&zeros[..program_bytes])?;
            }
            Pending::Program { offset, bytes, len } => {
                w.write_u8(2)?;
                w.write_u64(offset)?;
                w.write_u64(u64::from(len))?;
                w.write_all(&bytes[..program_bytes])?;
            }
            Pending::Option => {
                w.write_u8(3)?;
                w.write_u64(0)?;
                w.write_u64(0)?;
                w.write_all(&zeros[..program_bytes])?;
            }
        }
        w.write_u8(state.key_step)?;
        w.write_u8(state.optkey_step)?;
        w.write_u8(state.pdkey_step)?;
        w.write_bool(state.key_error)?;
        w.write_bool(state.optkey_error)?;
        w.write_u64(state.tick)?;
        if variant.is_u5() {
            // The write buffer: a snapshot taken between the second and the
            // fourth word of a quad-word must finish it on the other side.
            w.write_u64(state.wbuf_base)?;
            w.write_u32(state.wbuf_mask)?;
            w.write_all(&state.wbuf)?;
        }
        if variant.is_wb() {
            // An operation `PES` is holding back, which starts when it lifts.
            w.write_u64(state.suspended)?;
        }
        Ok(())
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
        let variant = self.shared.variant;
        let mut state = State::reset(variant, [0; OPT_WORDS]);
        state.words = [0; WORDS];
        for word in &mut state.words[..variant.saved_words()] {
            *word = r.read_u32()?;
        }
        for word in &mut state.stored[..variant.saved_option_words()] {
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
        let program_bytes = variant.saved_program_bytes();
        let raw = r.take(program_bytes)?;
        let mut program = [0u8; PROGRAM_MAX];
        program[..program_bytes].copy_from_slice(raw);
        state.pending = match tag {
            0 => Pending::None,
            1 => Pending::Erase { offset, len },
            2 => Pending::Program {
                offset,
                bytes: program,
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
        if variant.is_u5() {
            state.wbuf_base = r.read_u64()?;
            state.wbuf_mask = r.read_u32()?;
            state.wbuf.copy_from_slice(r.take(PROGRAM_MAX)?);
        }
        if variant.is_wb() {
            state.suspended = r.read_u64()?;
        }

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
            summary: "which manual: \"f4\" (RM0090 sectors), \"l4\" (RM0351 pages), \"l4plus\", \
                      \"u5\" (RM0456 quad-word programming) or \"wb\" (RM0434, CPU2's secure \
                      area)",
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
            name: "sfr",
            kind: ValueKind::Uint,
            required: false,
            summary: "WB FLASH_SFR as the option bytes hold it: SFSA, FSD, DDS (default: SFSA where \
                      ST's full BLE stack installs for this size, DDS set)",
        },
        PropertySpec {
            name: "srrvr",
            kind: ValueKind::Uint,
            required: false,
            summary: "WB FLASH_SRRVR: the secure SRAM2a/2b starts and CPU2's reset vector",
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
        .prop(
            PropSchema::new("variant", ValueKind::Str).values(&["f4", "l4", "l4plus", "u5", "wb"]),
        )
        .prop(PropSchema::new("size", ValueKind::Uint).required())
        .prop(PropSchema::new("image", ValueKind::Media))
        .prop(PropSchema::new("optr", ValueKind::Uint))
        .prop(PropSchema::new("pcrop1sr", ValueKind::Uint))
        .prop(PropSchema::new("pcrop1er", ValueKind::Uint))
        .prop(PropSchema::new("pcrop2sr", ValueKind::Uint))
        .prop(PropSchema::new("pcrop2er", ValueKind::Uint))
        .prop(PropSchema::new("sfr", ValueKind::Uint))
        .prop(PropSchema::new("srrvr", ValueKind::Uint))
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
