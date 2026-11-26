//! The STM32 power controller.
//!
//! `st.pwr` is the other half of a vendor startup sequence. `SystemClock_Config`
//! turns on the power interface's own clock, writes `PWR_CR.VOS` (or `CR1.VOS`
//! on an L4) to pick a voltage scale, and then **waits for the regulator**:
//! an F42x/F43x spins on `CSR.VOSRDY` and again on `CSR.ODRDY` and
//! `CSR.ODSWRDY` for the over-drive, and an L4 spins until `SR2.VOSF` goes
//! clear. As with [`rcc`](super::rcc), a RAM cell never answers, so the spin
//! never ends.
//!
//! The other thing it owns is one bit: `DBP`, "disable backup domain write
//! protection". Until firmware sets it, the RCC's `BDCR` — the LSE, `RTCSEL`,
//! `RTCEN` — is write-protected and every write to it is dropped.
//!
//! # How `DBP` reaches the RCC
//!
//! **As a wire**, `pwr.dbp -> rcc.dbp`, and not as a handle. On the die it is
//! a signal from the power block to the backup-domain interface, so a level on
//! a net is the honest model; it is also the only model that does not put one
//! device-rank lock inside another. The RCC samples the level from an atomic
//! its `dbp` sink stores into, so the `BDCR` write path never calls into this
//! device at all, and the ranked lock order in `core::sync` is never tested.
//!
//! A board that instantiates no `st.pwr` leaves `rcc.dbp` unwired, and an RCC
//! with an unwired `dbp` treats the backup domain as **unprotected** — nothing
//! is modelling the protection, and a domain that could never be opened would
//! be a board bug wearing a device bug's clothes.
//!
//! # The clock gate
//!
//! Two *inputs* arrive from the reset and clock controller — `enable`, which is
//! **`RCC_APB1ENR` bit 28, `PWREN`**, and `reset`, which is `RCC_APB1RSTR` bit
//! 28, `PWRRST` (RM0090 §7.3.10–§7.3.15 for the enable registers; the bit
//! position is the peripheral's slot in `APB1`, and it is the same one in
//! both). On an L4 the pair is `RCC_APB1ENR1.PWREN` and `RCC_APB1RSTR1.PWRRST`
//! at the same bit (RM0351 §6.4.17):
//!
//! ```text
//!   wire rcc.apb1en28  -> pwr.enable
//!   wire rcc.apb1rst28 -> pwr.reset
//! ```
//!
//! This is the gate a vendor startup sequence trips over first: `PWREN` is the
//! write that has to come *before* `PWR_CR.VOS`, and a firmware that forgets it
//! finds a `VOS` that will not take and a `VOSRDY` that never moves — which,
//! with the gate wired, is now what this model does too. A board draws the
//! wires or it does not; an **undrawn `enable` leaves the controller clocked**,
//! so no machine file written before the gate changes behaviour.
//! [`gate`] has the rules.
//!
//! ## What a gated PWR does with `dbp`
//!
//! **It holds the level it had.** `DBP` is a bit of `CR`, and a block with no
//! clock accepts no write to `CR`, so the guest cannot move it — the honest
//! answer is therefore that it cannot change, and the wire is left exactly
//! where it was rather than being forced low. Forcing it low would be a
//! *different* claim: that removing the peripheral clock re-arms the
//! backup-domain write protection. Nothing in RM0090 §5.1.4 says so, and the
//! protection is in the backup domain, which is on `VSWITCH` and is the part
//! of the chip that is explicitly *not* gated by `APB1` — that is what makes it
//! the backup domain. A peripheral whose output is *combinational* from its
//! register file — an interrupt, a DMA request — does go low when the clock
//! does, because there is no clock left to compute it with; `DBP` is a latched
//! configuration bit that something else reads, and it stays.
//!
//! `reset` is the pin that does move it: `PWRRST` puts `CR` back to its reset
//! value, where `DBP` is clear, and [`Gated::gate_reset`](super::gate::Gated)
//! republishes the level from outside the lock like every other write path.
//!
//! # Time
//!
//! A regulator transition takes tens of microseconds. As in the RCC, that is
//! not a host-clock reading: the device is lazily advanced, it holds its own
//! tick in its own clock domain, and the guest access that polls the flag
//! catches it up. `ready-delay` is a count of those ticks.
//!
//! # Variants
//!
//! | `variant` | Part | What it changes |
//! | --- | --- | --- |
//! | `f4` | F405/415, F407/417 | `CR`/`CSR`; `VOS` is one bit at 14, no over-drive |
//! | `f42x` | F427/437, F429/439 | `VOS` is `[15:14]`, and `ODEN`/`ODSWEN` answer with `ODRDY`/`ODSWRDY` |
//! | `l4` | L4x5, L4x6 | `CR1`…`CR4`, `SR1`/`SR2`, `SCR`, the `PUCRx`/`PDCRx` pull registers |
//! | `l4plus` | L4+ | as `l4`, plus `CR5` and its `R1MODE` |
//! | `u5` | U575/U585 | RM0456's map: `VOSR`, `SVMSR`, `DBPR`, `CR3.REGSEL` — see below |
//! | `wb` | WB55 | RM0434's map: the L4's, plus the SMPS, CPU2's `C2CR1`/`C2CR3`, `EXTSCR` and `CR4.C2BOOT` — see below |
//!
//! # The U5 variant
//!
//! A U5 rearranged every register the startup path touches (ST CMSIS
//! `stm32u585xx.h`, `PWR_TypeDef`), and ST's U5 HAL spins on four of them:
//!
//! | firmware writes | and waits for | model |
//! | --- | --- | --- |
//! | `VOSR.VOS[17:16]` (`HAL_PWREx_ControlVoltageScaling`) | `VOSR.VOSRDY` (15), then `SVMSR.ACTVOSRDY` (15) | a `VOS` change clears both, and `ready-delay` ticks later sets both and copies `VOS` into `SVMSR.ACTVOS[17:16]` |
//! | `VOSR.BOOSTEN` (18), by the same call and by `HAL_RCC_ClockConfig` | `VOSR.BOOSTRDY` (14) | set `ready-delay` ticks after `BOOSTEN` rises, cleared the moment it falls |
//! | `CR3.REGSEL` (1) (`HAL_PWREx_ConfigSupply`) | `SVMSR.REGS` (1) to match | follows `REGSEL` after `ready-delay` ticks |
//! | `DBPR.DBP` (0) (`HAL_PWR_EnableBkUpAccess`) | `DBPR.DBP` reading back | storage, and it drives the `dbp` output exactly as `CR.DBP` does on an F4 or an L4 |
//!
//! Out of reset the regulator is the LDO in range 4 with `VOSRDY` and
//! `ACTVOSRDY` set (`VOSR` = `SVMSR` = `0x0000_8000`), so a firmware that
//! reads `SVMSR.ACTVOS` first sees the range it is in. `SVMCR`'s monitor
//! enables (`UVMEN`, `IO2VMEN`, `AVM1EN`, `AVM2EN`, bits 24–27) answer at once
//! with the matching `SVMSR.VDDxRDY` bit: the supplies are present, and there
//! is no analogue level to compare. `SR.CSSF` clears `STOPF`/`SBF`, `WUSCR`
//! clears `WUSR`, and `UCPDR` (`HAL_PWREx_DisableUCPDDeadBattery` writes
//! `UCPD_DBDIS`), `SECCFGR`, `PRIVCFGR`, `APCR` and the pull registers are
//! storage.
//!
//! The gate on a U5 is **`RCC_AHB3ENR.PWREN`, bit 2** — `wire rcc.ahb3en2 ->
//! pwr.enable` — and there is no `PWRRST`: `AHB3RSTR` has no bit for the PWR,
//! so a U5 board draws no `reset` wire.
//!
//! # The WB variant
//!
//! An STM32WB55's PWR (RM0434 §5.6 and Table 33; ST CMSIS `stm32wb55xx.h`,
//! `PWR_TypeDef`) is the L4's register file with a second CPU's controls
//! added. What firmware waits on:
//!
//! | firmware writes | and waits for | model |
//! | --- | --- | --- |
//! | `CR1.VOS[10:9]` (`HAL_PWREx_ControlVoltageScaling`) | `SR2.VOSF` (10) to go **clear** | as on an L4: a change sets `VOSF`, `ready-delay` ticks later it clears (§5.6.6: "1: the regulator output voltage is changing") |
//! | `CR1.LPR` (14) clear (`HAL_PWREx_DisableLowPowerRunMode`) | `SR2.REGLPF` (9) to go clear | `REGLPF` follows `LPR`, either way, after `ready-delay` ticks ("remains at 1 until the regulator is ready in main mode") |
//! | `CR5.SMPSEN` (15) (`LL_PWR_SMPS_SetMode`) | `SR2.SMPSF` (1) / `SMPSBF` (0), read by `LL_PWR_SMPS_GetEffectiveMode` | `ready-delay` ticks after `SMPSEN` moves, `SMPSF` = `SMPSEN` and `SMPSBF` = its complement; `SMPSEN` itself is also the **`smpsen`** output, which the RCC's `SMPSSWS` needs (RM0434 §6.4.9) |
//! | `CR1.DBP` (8) (`HAL_PWR_EnableBkUpAccess`, which the WB RCC HAL then polls) | `CR1.DBP` reading back | storage, and it drives the `dbp` output |
//! | `CR4.C2BOOT` (15) (`HAL_PWREx_ReleaseCore`) | nothing here — CPU2 starts | drives the **`c2boot`** output |
//!
//! `SR1` is read-only and `SCR` clears it (write one; bits 0–4 and 7–14),
//! reading as zero. `EXTSCR`'s CPU1/CPU2 stop and standby flags (8–11) and
//! `CRPF` (13) are read-only and cleared by `C1CSSF` (0), `C2CSSF` (1) and
//! `CCRPF` (2), which read as zero; `C1DS`/`C2DS` read zero, because nothing
//! here sleeps. `CR2`, `CR3`, `CR4`'s other bits, `CR5`'s SMPS trims, the
//! pull registers and CPU2's **`C2CR1`/`C2CR3`** (its low-power mode and
//! wake-up enables) are storage. Out of reset (RM0434 §5.6, agreeing with
//! ST's `STM32WB55_CM4.svd`): `CR1` = `0x200` (range 1), `CR3` = `C2CR3` =
//! `0x8000` (`EIWUL`), `CR5` = `0x427X` with the factory `SMPSVOS` trim read
//! as zero.
//!
//! ## `c2boot`
//!
//! The `c2boot` output is **high exactly while `CR4.C2BOOT` reads one**, and
//! the CPU2 model's `boot` input is wired to it. RM0434 §5.3: "Following
//! reset the CPU2 is prevented from booting by the C2BOOT bit. The CPU2 boots
//! only once the CPU1 has set the C2BOOT bit", and after a Stop or Standby
//! exit the bit decides whether CPU2 boots again. Nothing in the chapter
//! stops a running CPU2 when the bit is cleared — the bit is consulted at a
//! reset or a low-power exit, and CPU1 clears it as routine before it
//! enters CSTOP mode (§5.3) — so the level is a permission, and the
//! consumer treats its **rising edge** as the release and ignores a fall.
//! Keeping that latch in CPU2 rather than here means the register reads back
//! what was written, as the hardware's does, and one wire carries the whole
//! fact. A system reset clears `C2BOOT`, and with it
//! the level, which is right: CPU2 is held again until firmware sets it.
//! (`C2BOOT` is retained in Standby, §2.4; Standby is not modelled.)
//!
//! A WB's PWR has no clock-enable bit (`APB1ENR1` has no `PWREN`), so a WB
//! board draws neither `enable` nor `reset` and the controller is always
//! clocked.
//!
//! # Sources
//!
//! * ST **RM0090** rev 21, §5 "Power control (PWR)" — §5.5 for the register
//!   map, §5.1.4 for the backup domain and `DBP`, §5.1.5 for the voltage
//!   regulator and the over-drive.
//! * ST **RM0351** rev 9, §5 "Power control (PWR)" — §5.4 for the register map
//!   and §5.1.5 for the dynamic voltage scaling `VOSF` reports.
//! * ST **RM0456**, §10 "Power control (PWR)", for the U5; offsets and bits
//!   from ST's CMSIS `stm32u585xx.h` (`PWR_TypeDef`, `PWR_*_Pos`; Apache-2.0),
//!   and the flags that must move from the polling loops of ST's U5 HAL
//!   (`stm32u5xx_hal_pwr_ex.c`, BSD-3-Clause), whose code is not reproduced.
//! * ST **RM0434** rev 16, §5 "Power control (PWR)", for the STM32WB55:
//!   §5.1 for the SMPS step-down converter's modes, §5.3 (and §2.4) for CPU2
//!   boot, §5.6.1–§5.6.23 and Table 33 for every register, bit and reset
//!   value. The offsets and bits agree with ST's CMSIS `stm32wb55xx.h`
//!   (`PWR_TypeDef`, `PWR_*_Pos`; Apache-2.0) and the reset values with
//!   ST's `STM32WB55_CM4.svd` (Apache-2.0); the flags that must move were
//!   read off the polling loops of ST's WB HAL and LL
//!   (`stm32wbxx_hal_pwr_ex.c`, `stm32wbxx_ll_pwr.h`, BSD-3-Clause), whose
//!   code is not reproduced.
//!
//! No emulator source of any licence was consulted (`ROADMAP.md` §1).
//!
//! # Known deviations
//!
//! * `LPMS`, `SLEEPDEEP` and the standby/stop entry paths are storage only:
//!   the core's `WFI` does not consult this device yet, and the issue that
//!   asked for this one said that half could come later.
//! * The `PUCRx`/`PDCRx` pull-up and pull-down registers read back and change
//!   no pad: a pull resistor is an analogue property of a net this tree does
//!   not model, exactly as in [`gpio`](super::gpio).
//! * `PVD`/`PVM` never trip. There is no analogue supply to compare against,
//!   so `PVDO` and `PVMOx` stay clear rather than reporting an invented level.
//! * **WB `SR2` resets to `SMPSBF`** (`0x1`, bypass). RM0434 disagrees with
//!   itself here: §5.1 says "After a POR reset the SMPS step-down converter
//!   is in Bypass mode", and `SMPSEN` resets clear (§5.6.8), but §5.6.6
//!   (and the SVD) give `SR2` a reset value of `0x2`, which is `SMPSF`, step
//!   down. The prose and `CR5` win; no HAL loop waits on either. `REGLPS`
//!   (the low-power regulator has started) reads zero, the reset value.
//! * **`SMPSEN` is not cleared by a BORH**, which on silicon forces the SMPS
//!   into bypass with `SMPSEN` reset (§5.6.8): there is no supply to brown
//!   out.
//! * **`EXTSCR.C2DS` reads zero.** RM0434 §5.6.23 has it read 1 while CPU2
//!   is "in deepsleep or hold by C2BOOT"; whether CPU2 is held is the CPU2
//!   model's state, and the register's reset value is `0`.
//! * **WB radio and CPU2 events are never raised**: `SR1`'s BLE/802.15.4
//!   activity and wake-up flags, `C2HF`, `CRPEF`, `BORHF` and `SMPSFBF`, and
//!   `EXTSCR`'s `CRPF` and `C2STOPF`/`C2SBF`, have no source. They read zero
//!   and their clear bits work.

use alloc::boxed::Box;
use alloc::format;
use alloc::string::ToString;
use alloc::sync::Arc;
use core::fmt;

use crate::core::device::{Device, DeviceClass, PropertySpec, RealizeCtx, ResetKind, SinkPin};
use crate::core::error::{BusError, Error, Result};
use crate::core::props::{Props, ValueKind};
use crate::core::sched::{AccessKind, LazyHandle};
use crate::core::space::{AccessConstraints, MemAttrs, MemOps, MemResult, Region, RegionRef};
use crate::core::state::{ChunkReader, ChunkWriter, Sink, Source};
use crate::core::sync::{AtomicU64, LockRank, Mutex, Ordering};
use crate::core::value::{Endian, Width};
use crate::core::wire::{Level, WireId, WireSource};
use crate::dev::stm32::gate::{self, ClockGate, Gated};
use crate::machine::Instance;
use crate::machine::validate::{ClassSchema, PortDir, PropSchema};

/// The class name a machine description writes.
const CLASS_NAME: &str = "st.pwr";

/// The snapshot chunk version. Bump it with the encoding, never on its own.
///
/// Version 2 widened the register file to the U5's thirty-eight words.
const STATE_VERSION: u32 = 2;

/// How many 32-bit words the widest layout occupies: the U5 map ends at
/// `PDCRI` (`+0x94`), so thirty-eight words covers every variant and the
/// snapshot encoding is one shape.
const WORDS: usize = 0x26;

/// "No deadline": nothing is in transition.
const NO_DEADLINE: u64 = u64::MAX;

/// The default regulator transition, in ticks of this device's clock domain.
///
/// Eight ticks is a microsecond of an 8 MHz can. As in the RCC it is
/// deliberately short and a board that wants the datasheet's figure writes
/// `ready-delay`.
const DEFAULT_READY_DELAY: u64 = 8;

/// The name of the backup-domain write-protection output.
pub const DBP_PIN: &str = "dbp";

// ---------------------------------------------------------------------------
// Variants
// ---------------------------------------------------------------------------

/// Which family's register map this instance has.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Variant {
    /// RM0090's `CR`/`CSR` as an F405/407 has them: `VOS` is the single bit 14
    /// and there is no over-drive.
    F4,
    /// The same map on an F427/F429: `VOS` is `[15:14]` and `ODEN`/`ODSWEN`
    /// exist.
    F42x,
    /// RM0351's map: `CR1`…`CR4`, `SR1`/`SR2`, `SCR`, `PUCRx`/`PDCRx`.
    L4,
    /// As [`Variant::L4`], plus `CR5` at `+0x80`.
    L4Plus,
    /// RM0456's U5 map: `VOSR`, `SVMSR`, `DBPR` and the rest of
    /// `PWR_TypeDef`.
    U5,
    /// RM0434's STM32WB55 map: the L4's, plus `CR5`'s SMPS, `C2CR1`,
    /// `C2CR3`, `EXTSCR` and `CR4.C2BOOT`.
    WB,
}

impl Variant {
    /// The spelling a machine file writes.
    #[must_use]
    pub fn as_str(self) -> &'static str {
        match self {
            Variant::F4 => "f4",
            Variant::F42x => "f42x",
            Variant::L4 => "l4",
            Variant::L4Plus => "l4plus",
            Variant::U5 => "u5",
            Variant::WB => "wb",
        }
    }

    /// Whether this is one of the two RM0090 layouts.
    fn is_f4(self) -> bool {
        matches!(self, Variant::F4 | Variant::F42x)
    }

    /// How many bytes of the peripheral's kilobyte actually decode.
    #[must_use]
    pub fn register_bytes(self) -> u64 {
        match self {
            // `CR` and `CSR` and nothing else (RM0090 §5.5.3).
            Variant::F4 | Variant::F42x => 0x08,
            // Through `PDCRH` at `+0x5c` (RM0351 §5.4.13).
            Variant::L4 => 0x60,
            // Through `CR5` at `+0x80`.
            Variant::L4Plus => 0x84,
            // Through `PDCRI` at `+0x94`.
            Variant::U5 => 0x98,
            // Through `EXTSCR` at `+0x88` (ST CMSIS `stm32wb55xx.h`).
            Variant::WB => 0x8c,
        }
    }

    /// `CR`/`CR1`'s reset value.
    fn cr_reset(self) -> u32 {
        match self {
            // `VOS` = 1, the single bit 14 (RM0090 §5.5.1, "Reset value:
            // 0x0000 4000" on an F405/407).
            Variant::F4 => 0x0000_4000,
            // `VOS` = 11, scale 1 (the same section, "0x0000 C000" on an
            // F427/F429).
            Variant::F42x => 0x0000_c000,
            // `VOS` = 01, range 1 (RM0351 §5.4.1, "Reset value: 0x0000 0200").
            // A WB's is the same (RM0434 §5.6.1, "Reset value: 0x0000 0200").
            Variant::L4 | Variant::L4Plus | Variant::WB => 0x0000_0200,
            // Low-power mode bits all clear; `VOS` is not in `CR1` on a U5.
            Variant::U5 => 0,
        }
    }

    /// Where `DBP` lives, as `(register, bit)`: `CR` bit 8 in both older
    /// families (RM0090 §5.5.1, RM0351 §5.4.1), a register of its own on a U5
    /// (`DBPR` bit 0).
    fn dbp(self) -> (u64, u32) {
        match self {
            Variant::U5 => (U5_DBPR, 0),
            _ => (CR, 8),
        }
    }

    /// The `VOS` field, as `(shift, width)`.
    fn vos(self) -> (u32, u32) {
        match self {
            Variant::F4 => (14, 1),
            Variant::F42x => (14, 2),
            Variant::L4 | Variant::L4Plus | Variant::WB => (9, 2),
            // `VOSR.VOS[17:16]`.
            Variant::U5 => (16, 2),
        }
    }
}

// -- register offsets --------------------------------------------------------

/// F4: `PWR_CR`. L4: `PWR_CR1`. Both at `+0x00`.
const CR: u64 = 0x00;
/// F4: `PWR_CSR` at `+0x04`.
const F4_CSR: u64 = 0x04;
/// L4: `PWR_CR2`.
const L4_CR2: u64 = 0x04;
/// L4: `PWR_CR3`.
const L4_CR3: u64 = 0x08;
/// L4: `PWR_SR1`.
const L4_SR1: u64 = 0x10;
/// L4: `PWR_SR2`.
const L4_SR2: u64 = 0x14;
/// L4: `PWR_SCR`, which is write-1-to-clear and reads as zero.
const L4_SCR: u64 = 0x18;
/// L4+: `PWR_CR5`.
const L4_CR5: u64 = 0x80;
/// WB: `PWR_CR4`, with `C2BOOT` at 15.
const WB_CR4: u64 = 0x0c;
/// WB: `PWR_CR5`, the SMPS — at `+0x1c`, not the L4+'s `+0x80`.
const WB_CR5: u64 = 0x1c;
/// WB: `PWR_C2CR3`, CPU2's wake-up enables.
const WB_C2CR3: u64 = 0x84;
/// WB: `PWR_EXTSCR`, both CPUs' stop and standby flags and their clears.
const WB_EXTSCR: u64 = 0x88;
/// U5: `PWR_CR3`, with `REGSEL` at bit 1.
const U5_CR3: u64 = 0x08;
/// U5: `PWR_VOSR`.
const U5_VOSR: u64 = 0x0c;
/// U5: `PWR_SVMCR`, the supply monitor enables.
const U5_SVMCR: u64 = 0x10;
/// U5: `PWR_DBPR`.
const U5_DBPR: u64 = 0x28;
/// U5: `PWR_SR` — `CSSF` (write 1) at 0, `STOPF` at 1, `SBF` at 2.
const U5_SR: u64 = 0x38;
/// U5: `PWR_SVMSR`, every bit hardware-owned.
const U5_SVMSR: u64 = 0x3c;
/// U5: `PWR_BDSR`, read-only.
const U5_BDSR: u64 = 0x40;
/// U5: `PWR_WUSR`, the wake-up flags, read-only.
const U5_WUSR: u64 = 0x44;
/// U5: `PWR_WUSCR`, write-1-to-clear for `WUSR`; reads as zero.
const U5_WUSCR: u64 = 0x48;

// -- U5 bits (ST CMSIS `stm32u585xx.h`, `PWR_*_Pos`) -------------------------

/// `VOSR.BOOSTRDY` — the EPOD booster is ready.
const U5_VOSR_BOOSTRDY: u32 = 1 << 14;
/// `VOSR.VOSRDY` — the regulator has reached the selected range.
const U5_VOSR_VOSRDY: u32 = 1 << 15;
/// `VOSR.VOS[17:16]`.
const U5_VOS: u32 = 0b11 << 16;
/// `VOSR.BOOSTEN`.
const U5_VOSR_BOOSTEN: u32 = 1 << 18;
/// `CR3.REGSEL` — 0 LDO, 1 SMPS.
const U5_CR3_REGSEL: u32 = 1 << 1;
/// `SVMSR.REGS` — which regulator is actually supplying the core.
const U5_SVMSR_REGS: u32 = 1 << 1;
/// `SVMSR.ACTVOSRDY`.
const U5_SVMSR_ACTVOSRDY: u32 = 1 << 15;
/// `SVMSR.VDDUSBRDY`…`VDDA2RDY`, bits 24–27, which sit at the same positions
/// as their monitor enables `SVMCR.UVMEN`…`AVM2EN`.
const U5_MONITORS: u32 = 0xf << 24;

// -- WB bits (ST CMSIS `stm32wb55xx.h`, `PWR_*_Pos`) -------------------------

/// `CR1.LPR` — low-power run.
const WB_CR1_LPR: u32 = 1 << 14;
/// `CR4.C2BOOT` — boot CPU2.
const WB_CR4_C2BOOT: u32 = 1 << 15;
/// `CR5.SMPSEN` — the step-down converter in SMPS mode.
const WB_CR5_SMPSEN: u32 = 1 << 15;
/// `SR2.SMPSBF` — the SMPS is in bypass.
const WB_SR2_SMPSBF: u32 = 1 << 0;
/// `SR2.SMPSF` — the SMPS is stepping down.
const WB_SR2_SMPSF: u32 = 1 << 1;
/// `SR2.REGLPF` — the main regulator is in low-power mode.
const WB_SR2_REGLPF: u32 = 1 << 9;
/// `SCR`'s clear bits: `CWUF1`…`CWUF5` (0–4) and `CSMPSFBF`…`CC2HF` (7–14).
const WB_SCR_CLEARS: u32 = 0x7f9f;
/// `EXTSCR`'s hardware-owned flags: `C1SBF`…`C2STOPF` (8–11), `CRPF` (13),
/// `C1DS`/`C2DS` (14, 15).
const WB_EXTSCR_FLAGS: u32 = 0xef00;

/// The name of the WB's CPU2 boot output (`CR4.C2BOOT`).
pub const C2BOOT_PIN: &str = "c2boot";

/// The name of the WB's `CR5.SMPSEN` output, which the RCC's
/// `SMPSCR.SMPSSWS` reports through (RM0434 §6.4.9).
pub const SMPSEN_PIN: &str = "smpsen";

// -- F4 `CR` and `CSR` bits (RM0090 §5.5.1, §5.5.2) --------------------------

/// `CR.ODEN` — switch the over-drive on.
const F4_CR_ODEN: u32 = 1 << 16;
/// `CR.ODSWEN` — switch the over-drive into the core's supply.
const F4_CR_ODSWEN: u32 = 1 << 17;
/// `CSR.VOSRDY` — the regulator has reached the selected scale.
const F4_CSR_VOSRDY: u32 = 1 << 14;
/// `CSR.ODRDY`.
const F4_CSR_ODRDY: u32 = 1 << 16;
/// `CSR.ODSWRDY`.
const F4_CSR_ODSWRDY: u32 = 1 << 17;
/// `CSR.CWUF`/`CSR.CSBF` are cleared through `CR`; the flags themselves are
/// hardware-owned, and so is everything the model does not drive.
const F4_CSR_READ_ONLY: u32 =
    0b111 | F4_CSR_VOSRDY | F4_CSR_ODRDY | F4_CSR_ODSWRDY | (1 << 3) | (1 << 18) | (1 << 19);

// -- L4 `SR2` bits (RM0351 §5.4.6) -------------------------------------------

/// `SR2.VOSF` — **set while the regulator is changing range**, not when it is
/// ready. Firmware waits for it to go clear, which is the opposite polarity to
/// the F4's `VOSRDY` and the single most common way to get this peripheral
/// wrong.
const L4_SR2_VOSF: u32 = 1 << 10;
/// `SR2.REGLPS`/`REGLPF`, and the `PVDO`/`PVMOx` comparator outputs: every bit
/// of `SR2` is hardware-owned.
const L4_SR2_READ_ONLY: u32 = 0xffff_ffff;

// ---------------------------------------------------------------------------
// State
// ---------------------------------------------------------------------------

/// Everything the guest can see or change, plus the device's own position.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct State {
    /// The register file, indexed by `offset / 4`.
    words: [u32; WORDS],
    /// When the regulator finishes its current transition.
    vos_at: u64,
    /// When the over-drive comes up, and when it switches in (F42x). On a U5
    /// the same two deadlines are the EPOD booster's `BOOSTRDY` and the
    /// regulator switch that `SVMSR.REGS` reports; on a WB they are
    /// `SR2.REGLPF` following `CR1.LPR` and `SR2.SMPSF`/`SMPSBF` following
    /// `CR5.SMPSEN`.
    od_at: u64,
    odsw_at: u64,
    /// The tick this device has been advanced to.
    tick: u64,
}

impl State {
    fn reset(variant: Variant) -> State {
        let mut words = [0u32; WORDS];
        words[(CR / 4) as usize] = variant.cr_reset();
        if variant.is_f4() {
            // The regulator is already at the scale `CR` selects out of reset.
            words[(F4_CSR / 4) as usize] = F4_CSR_VOSRDY;
        } else if variant == Variant::WB {
            // RM0434 §5.6.3 and §5.6.22: `CR3.EIWUL` and `C2CR3.EIWUL` set;
            // §5.6.8: `CR5` = `0x427X` — `SMPSEN` clear, `SMPSSC` = 7, bits 9
            // and 14 (reserved) as the manual has them, and the
            // factory-trimmed `SMPSVOS` (`X`) as zero.
            words[(L4_CR3 / 4) as usize] = 0x0000_8000;
            words[(WB_C2CR3 / 4) as usize] = 0x0000_8000;
            words[(WB_CR5 / 4) as usize] = 0x0000_4270;
            // The SMPS in bypass, as RM0434 §5.1 says it is after a POR —
            // where §5.6.6's reset value says `SMPSF`; see the known
            // deviations.
            words[(L4_SR2 / 4) as usize] = WB_SR2_SMPSBF;
        } else if variant == Variant::U5 {
            // Range 4 (`VOS` = 00), reached: `VOSRDY` and `ACTVOSRDY` set,
            // the LDO supplying the core (RM0456 §10, `PWR_VOSR` and
            // `PWR_SVMSR`, "Reset value: 0x0000 8000").
            words[(U5_VOSR / 4) as usize] = U5_VOSR_VOSRDY;
            words[(U5_SVMSR / 4) as usize] = U5_SVMSR_ACTVOSRDY;
        } else {
            // `CR3.EIWUL` comes up set (RM0351 §5.4.3, "Reset value:
            // 0x0000 8000").
            words[(L4_CR3 / 4) as usize] = 0x0000_8000;
            if variant == Variant::L4Plus {
                // `CR5.R1MODE` comes up set: range 1 boost is off.
                words[(L4_CR5 / 4) as usize] = 0x0000_0100;
            }
        }
        State {
            words,
            vos_at: NO_DEADLINE,
            od_at: NO_DEADLINE,
            odsw_at: NO_DEADLINE,
            tick: 0,
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

// ---------------------------------------------------------------------------
// The register block
// ---------------------------------------------------------------------------

/// The register block, as something an address space can dispatch to.
struct Registers {
    state: Mutex<State>,
    variant: Variant,
    ready_delay: u64,
    /// The `DBP` output, connected at realize time.
    dbp: Mutex<Option<WireSource>>,
    /// A WB's `c2boot` output, likewise.
    c2boot: Mutex<Option<WireSource>>,
    /// A WB's `smpsen` output, likewise.
    smpsen: Mutex<Option<WireSource>>,
    /// The lock-free half of the lazy contract.
    tick: AtomicU64,
    next_event: AtomicU64,
    lazy: Mutex<Option<LazyHandle>>,
    /// `RCC_APB1ENR.PWREN` and `RCC_APB1RSTR.PWRRST`, as a board's wires
    /// deliver them. Wiring rather than chip state, so it sits beside the
    /// register file and not in it.
    gate: ClockGate,
}

impl fmt::Debug for Registers {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let mut s = f.debug_struct("Registers");
        s.field("variant", &self.variant)
            .field("ready-delay", &self.ready_delay);
        match self.state.try_lock() {
            Some(state) => s.field("tick", &state.tick),
            None => s.field("tick", &"<locked>"),
        };
        s.finish()
    }
}

impl Gated for Registers {
    fn clock_gate(&self) -> &ClockGate {
        &self.gate
    }

    fn gate_reset(&self) {
        // `RCC_APB1RSTR.PWRRST`: the register file and the three transition
        // deadlines go back to their reset values, and `dbp` is republished
        // from outside the lock — `CR`'s reset value has `DBP` clear, so this
        // is the one thing that *does* move the level. The device's own tick is
        // not state the reset line owns: it is where this device has been
        // advanced to, and rewinding it would be a time-travelling peripheral.
        // Both are the lines `Device::reset` draws, for the same reasons.
        {
            let mut state = self.state.lock();
            let tick = state.tick;
            *state = State::reset(self.variant);
            state.tick = tick;
            self.republish(&state);
        }
        self.refresh_dbp();
    }
}

impl Registers {
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
        let next = state.vos_at.min(state.od_at).min(state.odsw_at);
        let next = if next <= state.tick { u64::MAX } else { next };
        self.next_event.store(next, Ordering::Relaxed);
    }

    /// Whether `DBP` is set.
    fn dbp_high(&self, state: &State) -> bool {
        let (reg, bit) = self.variant.dbp();
        state.word(reg) & (1 << bit) != 0
    }

    /// Drive the `DBP` output to whatever `CR` now says.
    ///
    /// Called with **no lock held**: the sink is the RCC, and the re-entrancy
    /// contract is that an outward call happens after the critical section.
    fn refresh_dbp(&self) {
        let level = {
            let state = self.state.lock();
            Level::from_bool(self.dbp_high(&state))
        };
        let source = self.dbp.lock().clone();
        if let Some(source) = source {
            source.set(level);
        }
        self.refresh_wb_outputs();
    }

    /// Drive a WB's `c2boot` to `CR4.C2BOOT` and `smpsen` to `CR5.SMPSEN`.
    /// No lock held, as for `dbp`.
    ///
    /// Every path that republishes `dbp` comes through here too, so a reset,
    /// a load and a register write each move all three outputs.
    fn refresh_wb_outputs(&self) {
        if self.variant != Variant::WB {
            return;
        }
        let (boot, smps) = {
            let state = self.state.lock();
            (
                Level::from_bool(state.word(WB_CR4) & WB_CR4_C2BOOT != 0),
                Level::from_bool(state.word(WB_CR5) & WB_CR5_SMPSEN != 0),
            )
        };
        for (pin, level) in [(&self.c2boot, boot), (&self.smpsen, smps)] {
            let source = pin.lock().clone();
            if let Some(source) = source {
                source.set(level);
            }
        }
    }

    /// Simulate forward: the regulator and the over-drive finish transitioning.
    fn advance_to(&self, tick: u64) {
        let mut state = self.state.lock();
        if tick <= state.tick {
            return;
        }
        if !self.gate.live() {
            // No clock, so nothing counts: a transition caught by the gate
            // waits, and resumes with the ticks it had left rather than
            // finishing the instant the clock comes back. Sliding the
            // deadlines forward by the elapsed time is what "resumes where it
            // stopped" means for a device whose position is an absolute tick —
            // simply not advancing `tick` would leave the catch-up to fire
            // every deadline at once on the next access.
            let elapsed = tick - state.tick;
            let state = &mut *state;
            state.tick = tick;
            for at in [&mut state.vos_at, &mut state.od_at, &mut state.odsw_at] {
                if *at != NO_DEADLINE {
                    *at = at.saturating_add(elapsed);
                }
            }
            self.republish(state);
            return;
        }
        state.tick = tick;
        if self.variant == Variant::U5 {
            Self::advance_u5(&mut state, tick);
            self.republish(&state);
            return;
        }
        if self.variant == Variant::WB {
            Self::advance_wb(&mut state, tick);
            self.republish(&state);
            return;
        }
        if state.vos_at <= tick {
            state.vos_at = NO_DEADLINE;
            if self.variant.is_f4() {
                *state.word_mut(F4_CSR) |= F4_CSR_VOSRDY;
            } else {
                // "0: the regulator is ready in the selected voltage range."
                *state.word_mut(L4_SR2) &= !L4_SR2_VOSF;
            }
        }
        if state.od_at <= tick {
            state.od_at = NO_DEADLINE;
            *state.word_mut(F4_CSR) |= F4_CSR_ODRDY;
        }
        if state.odsw_at <= tick {
            state.odsw_at = NO_DEADLINE;
            *state.word_mut(F4_CSR) |= F4_CSR_ODSWRDY;
        }
        self.republish(&state);
    }

    /// The U5's three transitions finishing.
    fn advance_u5(state: &mut State, tick: u64) {
        if state.vos_at <= tick {
            state.vos_at = NO_DEADLINE;
            // The new range is reached: `VOSRDY`, and the range the
            // regulator is *actually* applying catches up with the request.
            let vos = state.word(U5_VOSR) & U5_VOS;
            *state.word_mut(U5_VOSR) |= U5_VOSR_VOSRDY;
            let svmsr = state.word_mut(U5_SVMSR);
            *svmsr = (*svmsr & !U5_VOS) | vos | U5_SVMSR_ACTVOSRDY;
        }
        if state.od_at <= tick {
            state.od_at = NO_DEADLINE;
            *state.word_mut(U5_VOSR) |= U5_VOSR_BOOSTRDY;
        }
        if state.odsw_at <= tick {
            state.odsw_at = NO_DEADLINE;
            let smps = state.word(U5_CR3) & U5_CR3_REGSEL != 0;
            let svmsr = state.word_mut(U5_SVMSR);
            *svmsr = if smps {
                *svmsr | U5_SVMSR_REGS
            } else {
                *svmsr & !U5_SVMSR_REGS
            };
        }
    }

    /// The WB's three transitions finishing: `VOSF` clears, `REGLPF` and the
    /// SMPS flags catch up with what `CR1.LPR` and `CR5.SMPSEN` now ask for.
    fn advance_wb(state: &mut State, tick: u64) {
        if state.vos_at <= tick {
            state.vos_at = NO_DEADLINE;
            *state.word_mut(L4_SR2) &= !L4_SR2_VOSF;
        }
        if state.od_at <= tick {
            state.od_at = NO_DEADLINE;
            let lpr = state.word(CR) & WB_CR1_LPR != 0;
            let sr2 = state.word_mut(L4_SR2);
            *sr2 = if lpr {
                *sr2 | WB_SR2_REGLPF
            } else {
                *sr2 & !WB_SR2_REGLPF
            };
        }
        if state.odsw_at <= tick {
            state.odsw_at = NO_DEADLINE;
            let smps = state.word(WB_CR5) & WB_CR5_SMPSEN != 0;
            let sr2 = state.word_mut(L4_SR2);
            *sr2 &= !(WB_SR2_SMPSF | WB_SR2_SMPSBF);
            *sr2 |= if smps { WB_SR2_SMPSF } else { WB_SR2_SMPSBF };
        }
    }

    fn read_register(&self, offset: u64) -> u32 {
        let state = self.state.lock();
        if self.variant == Variant::U5 && offset == U5_WUSCR {
            // A clear register, which holds nothing of its own.
            return 0;
        }
        if self.variant == Variant::WB && offset == WB_EXTSCR {
            // The three clear bits are write-only.
            return state.word(WB_EXTSCR) & WB_EXTSCR_FLAGS;
        }
        if !self.variant.is_f4() && self.variant != Variant::U5 && offset == L4_SCR {
            // "This register is write-only": the clear register reads as zero.
            return 0;
        }
        state.word(offset)
    }

    /// Write one register, then whatever it implies.
    fn write_register(&self, offset: u64, value: u32) {
        {
            let mut state = self.state.lock();
            let before = state.word(offset);
            if self.variant.is_f4() {
                self.write_f4(&mut state, offset, value, before);
            } else if self.variant == Variant::U5 {
                self.write_u5(&mut state, offset, value, before);
            } else if self.variant == Variant::WB {
                self.write_wb(&mut state, offset, value, before);
            } else {
                self.write_l4(&mut state, offset, value, before);
            }
            self.republish(&state);
        }
        self.refresh_dbp();
    }

    fn write_f4(&self, state: &mut State, offset: u64, value: u32, before: u32) {
        match offset {
            CR => {
                *state.word_mut(CR) = value;
                let (shift, width) = self.variant.vos();
                let mask = ((1u32 << width) - 1) << shift;
                if value & mask != before & mask {
                    // The regulator has to move, so it stops being ready first
                    // and answers `ready-delay` ticks later (RM0090 §5.1.5).
                    *state.word_mut(F4_CSR) &= !F4_CSR_VOSRDY;
                    state.vos_at = state.tick.saturating_add(self.ready_delay);
                }
                if self.variant == Variant::F42x {
                    for (bit, rdy, at) in [
                        (F4_CR_ODEN, F4_CSR_ODRDY, 0),
                        (F4_CR_ODSWEN, F4_CSR_ODSWRDY, 1),
                    ] {
                        let was = before & bit != 0;
                        let is = value & bit != 0;
                        if is && !was {
                            let deadline = state.tick.saturating_add(self.ready_delay);
                            if at == 0 {
                                state.od_at = deadline;
                            } else {
                                state.odsw_at = deadline;
                            }
                        } else if !is && was {
                            *state.word_mut(F4_CSR) &= !rdy;
                            if at == 0 {
                                state.od_at = NO_DEADLINE;
                            } else {
                                state.odsw_at = NO_DEADLINE;
                            }
                        }
                    }
                }
                // "CWUF: clear wake-up flag … cleared by hardware two system
                // clock cycles later" and "CSBF: clear standby flag". Both
                // clear a `CSR` flag and neither is storage.
                let csr = state.word_mut(F4_CSR);
                *csr &= !(value & 0b11);
                let cr = state.word_mut(CR);
                *cr &= !0b1100;
            }
            F4_CSR => {
                let keep = F4_CSR_READ_ONLY;
                *state.word_mut(F4_CSR) = (before & keep) | (value & !keep);
            }
            _ => {}
        }
    }

    fn write_u5(&self, state: &mut State, offset: u64, value: u32, before: u32) {
        let deadline = state.tick.saturating_add(self.ready_delay);
        match offset {
            U5_VOSR => {
                // `VOSRDY` and `BOOSTRDY` are the hardware's; `VOS` and
                // `BOOSTEN` are the guest's.
                let keep = U5_VOSR_VOSRDY | U5_VOSR_BOOSTRDY;
                let next = (before & keep) | (value & (U5_VOS | U5_VOSR_BOOSTEN));
                *state.word_mut(U5_VOSR) = next;
                if next & U5_VOS != before & U5_VOS {
                    // The regulator has to move: it is not ready in the new
                    // range until it gets there, and neither is the range it
                    // reports applying.
                    *state.word_mut(U5_VOSR) &= !U5_VOSR_VOSRDY;
                    *state.word_mut(U5_SVMSR) &= !U5_SVMSR_ACTVOSRDY;
                    state.vos_at = deadline;
                }
                let was = before & U5_VOSR_BOOSTEN != 0;
                let is = next & U5_VOSR_BOOSTEN != 0;
                if is && !was {
                    state.od_at = deadline;
                } else if !is && was {
                    *state.word_mut(U5_VOSR) &= !U5_VOSR_BOOSTRDY;
                    state.od_at = NO_DEADLINE;
                }
            }
            U5_CR3 => {
                *state.word_mut(U5_CR3) = value;
                if (value ^ before) & U5_CR3_REGSEL != 0 {
                    state.odsw_at = deadline;
                }
            }
            U5_SVMCR => {
                *state.word_mut(U5_SVMCR) = value;
                // A monitor reports its supply present the moment it is
                // enabled: the supplies are there, and there is no analogue
                // level to watch fall.
                let svmsr = state.word_mut(U5_SVMSR);
                *svmsr = (*svmsr & !U5_MONITORS) | (value & U5_MONITORS);
            }
            U5_SR => {
                // "CSSF: clear Stop and Standby flags" — write 1; the flags
                // themselves are read-only.
                if value & 1 != 0 {
                    *state.word_mut(U5_SR) &= !0b110;
                }
            }
            U5_WUSCR => {
                *state.word_mut(U5_WUSR) &= !(value & 0xff);
            }
            U5_SVMSR | U5_BDSR | U5_WUSR => {
                // Hardware-owned through and through.
            }
            _ => {
                *state.word_mut(offset) = value;
            }
        }
    }

    fn write_wb(&self, state: &mut State, offset: u64, value: u32, before: u32) {
        let deadline = state.tick.saturating_add(self.ready_delay);
        match offset {
            CR => {
                // `VOS` and `VOSF` are the L4's.
                self.write_l4(state, offset, value, before);
                if (value ^ before) & WB_CR1_LPR != 0 {
                    // The main regulator takes time to enter or leave low
                    // power, and `REGLPF` is what reports it arriving.
                    state.od_at = deadline;
                }
            }
            WB_CR5 => {
                *state.word_mut(WB_CR5) = value;
                if (value ^ before) & WB_CR5_SMPSEN != 0 {
                    state.odsw_at = deadline;
                }
            }
            L4_SCR => {
                // "Writing 1 clears the corresponding flag in `SR1`."
                *state.word_mut(L4_SR1) &= !(value & WB_SCR_CLEARS);
            }
            WB_EXTSCR => {
                // `C1CSSF` clears CPU1's stop and standby flags, `C2CSSF`
                // CPU2's, `CCRPF` the critical radio phase; nothing else
                // here is the guest's to write.
                let mut clear = 0;
                if value & 1 != 0 {
                    clear |= 0b11 << 8;
                }
                if value & 2 != 0 {
                    clear |= 0b11 << 10;
                }
                if value & 4 != 0 {
                    clear |= 1 << 13;
                }
                *state.word_mut(WB_EXTSCR) &= !clear;
            }
            L4_SR1 | L4_SR2 => {
                // Hardware-owned through and through.
            }
            _ => {
                // `CR2`, `CR3`, `CR4` (whose `C2BOOT` the caller republishes
                // on `c2boot`), the pulls, `C2CR1`, `C2CR3`: storage.
                *state.word_mut(offset) = value;
            }
        }
    }

    fn write_l4(&self, state: &mut State, offset: u64, value: u32, before: u32) {
        match offset {
            CR => {
                *state.word_mut(CR) = value;
                let (shift, width) = self.variant.vos();
                let mask = ((1u32 << width) - 1) << shift;
                if value & mask != before & mask {
                    // "1: the regulator output voltage is changing"
                    // (RM0351 §5.4.6).
                    *state.word_mut(L4_SR2) |= L4_SR2_VOSF;
                    state.vos_at = state.tick.saturating_add(self.ready_delay);
                }
            }
            L4_SR1 | L4_SR2 => {
                // Both are read-only; `SR1` is cleared through `SCR`.
                let keep = if offset == L4_SR2 {
                    L4_SR2_READ_ONLY
                } else {
                    0xffff_ffff
                };
                *state.word_mut(offset) = (before & keep) | (value & !keep);
            }
            L4_SCR => {
                // "CWUFx: clear wake-up flag x … writing 1 clears it."
                *state.word_mut(L4_SR1) &= !(value & 0x0000_011f);
            }
            L4_CR2 | L4_CR3 => {
                *state.word_mut(offset) = value;
            }
            _ => {
                *state.word_mut(offset) = value;
            }
        }
    }
}

impl MemOps for Registers {
    fn read(&self, offset: u64, dst: &mut [u8], attrs: MemAttrs) -> MemResult {
        let [a, b, c, d] = dst else {
            return Err(BusError::BadAccess);
        };
        if !self.gate.live() {
            // No `RCC_APB1ENR.PWREN`, or `PWRRST` standing: the block is not
            // readable and zero is what the bridge returns
            // ([`gate`](super::gate)). A debug read sees the same nothing,
            // because that is what the bus itself answers. Not even `sync`
            // runs: catching a stopped device up is the thing the gate exists
            // to prevent.
            (*a, *b, *c, *d) = (0, 0, 0, 0);
            return Ok(());
        }
        self.sync(attrs);
        let value = self.read_register(offset & !3);
        let bytes = value.to_le_bytes();
        (*a, *b, *c, *d) = (bytes[0], bytes[1], bytes[2], bytes[3]);
        Ok(())
    }

    fn write(&self, offset: u64, src: &[u8], attrs: MemAttrs) -> MemResult {
        let [a, b, c, d] = src else {
            return Err(BusError::BadAccess);
        };
        if attrs.debug {
            // A debug write to `CR` would unlock the backup domain and move
            // the `dbp` wire. It is refused rather than guessed at
            // (`ROADMAP.md` §15, invariant 5).
            return Err(BusError::BadAccess);
        }
        if !self.gate.live() {
            // Not effective: `VOS` does not move, no transition is armed and
            // `DBP` cannot change — which is why the `dbp` output keeps the
            // level it had rather than being driven low. See the module note.
            return Ok(());
        }
        self.sync(attrs);
        self.write_register(offset & !3, u32::from_le_bytes([*a, *b, *c, *d]));
        Ok(())
    }

    fn constraints(&self) -> AccessConstraints {
        AccessConstraints::word(Width::U32, Endian::Little)
    }
}

// ---------------------------------------------------------------------------
// The device
// ---------------------------------------------------------------------------

/// An STM32 power controller.
#[derive(Debug)]
pub struct Pwr {
    regs: Arc<Registers>,
    region: RegionRef,
}

impl Pwr {
    /// Validate `props` and build the device.
    ///
    /// # Errors
    ///
    /// [`Error::Property`] if a property is of the wrong kind or value, or if
    /// one this class does not know was given.
    pub fn new(props: &Props) -> Result<Pwr> {
        let mut r = props.reader();
        let variant =
            match r.or_enum("variant", "f4", &["f4", "f42x", "l4", "l4plus", "u5", "wb"])? {
                "f42x" => Variant::F42x,
                "u5" => Variant::U5,
                "wb" => Variant::WB,
                "l4" => Variant::L4,
                "l4plus" => Variant::L4Plus,
                _ => Variant::F4,
            };
        let ready_delay = r.or("ready-delay", DEFAULT_READY_DELAY)?;
        r.finish()?;
        Ok(Pwr::with_config(variant, ready_delay))
    }

    /// Build one directly — the route a test takes.
    #[must_use]
    pub fn with_config(variant: Variant, ready_delay: u64) -> Pwr {
        let regs = Arc::new(Registers {
            state: Mutex::with_rank(LockRank::DEVICE, State::reset(variant)),
            variant,
            ready_delay,
            dbp: Mutex::with_rank(LockRank::WIRE, None),
            c2boot: Mutex::with_rank(LockRank::WIRE, None),
            smpsen: Mutex::with_rank(LockRank::WIRE, None),
            tick: AtomicU64::new(0),
            next_event: AtomicU64::new(u64::MAX),
            lazy: Mutex::with_rank(LockRank::LEAF, None),
            gate: ClockGate::new(),
        });
        let region = Arc::new(Region::io(
            "pwr",
            variant.register_bytes(),
            Arc::clone(&regs) as Arc<dyn MemOps>,
        ));
        Pwr { regs, region }
    }

    /// Which register layout this instance has.
    #[must_use]
    pub fn variant(&self) -> Variant {
        self.regs.variant
    }

    /// Whether backup-domain write protection is currently disabled.
    #[must_use]
    pub fn dbp(&self) -> bool {
        let state = self.regs.state.lock();
        self.regs.dbp_high(&state)
    }

    /// Set or clear `CR.DBP` as a guest write would, wire and all.
    ///
    /// The route a harness takes when it wants the backup domain open and has
    /// no guest to run: it goes through the same write path, so the `dbp`
    /// output moves exactly as it would for firmware.
    pub fn set_dbp(&self, high: bool) {
        let (reg, bit) = self.regs.variant.dbp();
        let word = self.regs.state.lock().word(reg);
        let next = if high {
            word | (1 << bit)
        } else {
            word & !(1 << bit)
        };
        self.regs.write_register(reg, next);
    }

    /// Whether `CR4.C2BOOT` is set, which is what `c2boot` drives. Always
    /// false on a part with one CPU.
    #[must_use]
    pub fn c2boot(&self) -> bool {
        self.regs.variant == Variant::WB && self.regs.state.lock().word(WB_CR4) & WB_CR4_C2BOOT != 0
    }

    /// Connect the `DBP` output.
    pub fn connect_dbp(&self, source: WireSource) {
        *self.regs.dbp.lock() = Some(source);
        self.regs.refresh_dbp();
    }
}

impl Device for Pwr {
    fn class(&self) -> &'static DeviceClass {
        &CLASS
    }

    fn realize(&self, _ctx: &mut RealizeCtx<'_>) -> Result<()> {
        Ok(())
    }

    fn reset(&self, _kind: ResetKind) {
        {
            let mut state = self.regs.state.lock();
            let tick = state.tick;
            *state = State::reset(self.regs.variant);
            state.tick = tick;
            self.regs.republish(&state);
        }
        self.regs.refresh_dbp();
    }

    fn save(&self, w: &mut ChunkWriter<'_>) -> Result<()> {
        let state = *self.regs.state.lock();
        for word in state.words {
            w.write_u32(word)?;
        }
        w.write_u64(state.vos_at)?;
        w.write_u64(state.od_at)?;
        w.write_u64(state.odsw_at)?;
        w.write_u64(state.tick)
        // The wire handle is the machine's wiring, not the chip's state.
    }

    fn load(&self, r: &mut ChunkReader<'_>) -> Result<()> {
        let mut state = State::reset(self.regs.variant);
        for word in &mut state.words {
            *word = r.read_u32()?;
        }
        state.vos_at = r.read_u64()?;
        state.od_at = r.read_u64()?;
        state.odsw_at = r.read_u64()?;
        state.tick = r.read_u64()?;
        {
            let mut held = self.regs.state.lock();
            *held = state;
            self.regs.republish(&held);
        }
        self.regs.refresh_dbp();
        Ok(())
    }

    fn region(&self, name: &str) -> Option<RegionRef> {
        matches!(name, "" | "regs").then(|| Arc::clone(&self.region))
    }

    fn connect(&self, port: &str, source: WireSource) -> Result<()> {
        match port {
            DBP_PIN => self.connect_dbp(source),
            C2BOOT_PIN if self.regs.variant == Variant::WB => {
                *self.regs.c2boot.lock() = Some(source);
                self.regs.refresh_wb_outputs();
            }
            SMPSEN_PIN if self.regs.variant == Variant::WB => {
                *self.regs.smpsen.lock() = Some(source);
                self.regs.refresh_wb_outputs();
            }
            _ => {
                let message = if self.regs.variant == Variant::WB {
                    format!("a WB PWR drives `{DBP_PIN}`, `{C2BOOT_PIN}` and `{SMPSEN_PIN}`")
                } else {
                    format!(
                        "an `{}` PWR drives one pin, `{DBP_PIN}`; `{C2BOOT_PIN}` and \
                         `{SMPSEN_PIN}` are a WB's",
                        self.regs.variant.as_str()
                    )
                };
                return Err(Error::Config {
                    at: port.to_string(),
                    message,
                });
            }
        }
        Ok(())
    }

    fn sink(&self, port: &str, sources: &[WireId]) -> Option<SinkPin> {
        // `enable` and `reset`, and a PWR has no other input: `dbp` is the one
        // pin it had and it is an output, so neither name is taken.
        gate::sink(&self.regs, port, sources)
    }

    fn announce(&self, port: &str) {
        if port == DBP_PIN || port == C2BOOT_PIN || port == SMPSEN_PIN {
            self.regs.refresh_dbp();
        }
    }

    fn is_lazy(&self) -> bool {
        // The regulator takes time to change range, and the guest's poll of
        // `VOSRDY`/`VOSF` is what has to see it finish.
        true
    }

    fn current_tick(&self) -> u64 {
        self.regs.tick.load(Ordering::Relaxed)
    }

    fn advance_to(&self, tick: u64) {
        self.regs.advance_to(tick);
    }

    fn next_event_tick(&self) -> Option<u64> {
        match self.regs.next_event.load(Ordering::Relaxed) {
            u64::MAX => None,
            at => Some(at),
        }
    }

    fn attach_lazy(&self, handle: LazyHandle) {
        *self.regs.lazy.lock() = Some(handle);
    }
}

impl Instance for Pwr {}

/// The `st.pwr` device class.
pub static CLASS: DeviceClass = DeviceClass {
    name: CLASS_NAME,
    version: STATE_VERSION,
    summary: "STM32 power control: DBP, the voltage-scaling ready flags, the F42x over-drive, \
              the U5's EPOD booster and SMPS/LDO switch, and the WB's SMPS and CPU2 boot",
    properties: &[
        PropertySpec {
            name: "variant",
            kind: ValueKind::Str,
            required: false,
            summary: "which register layout: \"f4\", \"f42x\" (over-drive), \"l4\", \"l4plus\", \
                      \"u5\" or \"wb\" (RM0434)",
        },
        PropertySpec {
            name: "ready-delay",
            kind: ValueKind::Uint,
            required: false,
            summary: "how many ticks of this device's clock domain a regulator transition \
                      takes (default 8)",
        },
    ],
    construct: |props| Ok(Box::new(Pwr::new(props)?)),
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
    bindings.bind(CLASS_NAME, |props| Ok(Arc::new(Pwr::new(props)?)))
}

/// What the validator should know about `st.pwr`.
#[must_use]
pub fn schema() -> ClassSchema {
    let schema = ClassSchema::new(CLASS_NAME)
        .prop(
            PropSchema::new("variant", ValueKind::Str)
                .values(&["f4", "f42x", "l4", "l4plus", "u5", "wb"]),
        )
        .prop(PropSchema::new("ready-delay", ValueKind::Uint))
        .region("")
        .region("regs")
        // `wire pwr.dbp -> rcc.dbp`: the backup domain opens when this goes
        // high, and it is a level on a net rather than a handle because that
        // is what it is on the die.
        .port(DBP_PIN, PortDir::Out)
        // `wire pwr.c2boot -> cpu2.boot` on a WB: `CR4.C2BOOT`, which CPU2
        // takes the rising edge of. A WB-only pin; another variant refuses
        // the `connect`.
        .port(C2BOOT_PIN, PortDir::Out)
        // `wire pwr.smpsen -> rcc.smpsen` on a WB: `CR5.SMPSEN`, without
        // which the RCC's `SMPSSWS` reads "no clock".
        .port(SMPSEN_PIN, PortDir::Out);
    // `RCC_APB1ENR.PWREN` and `RCC_APB1RSTR.PWRRST`.
    gate::ports(schema)
}

#[cfg(test)]
mod tests;
