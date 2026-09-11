//! An STM32 TIM: the basic, general-purpose and advanced-control timers.
//!
//! One IP block with feature subsets, which is how ST documents it and how it
//! is modelled here — a single class, `st.tim`, with `variant`, `width` and
//! `channels` as construction properties:
//!
//! | `variant` | The parts it is | What it adds |
//! | --- | --- | --- |
//! | `"basic"` | `TIM6`, `TIM7` | nothing: a counter, a prescaler, `ARR`, and one update interrupt |
//! | `"general"` | `TIM2`–`TIM5`, `TIM9`–`TIM14` | capture/compare channels, down- and center-aligned counting |
//! | `"advanced"` | `TIM1`, `TIM8` | `RCR` repetition, `BDTR.MOE`, complementary outputs, split interrupt vectors |
//!
//! `width` is 16 or 32 — `TIM2` and `TIM5` on an F4 are the 32-bit ones — and
//! `channels` is how many capture/compare channels the instance bonds, which
//! is 4 on `TIM2`–`TIM5` and `TIM1`/`TIM8`, 2 on `TIM9`/`TIM12`/`TIM15`, and 1
//! on `TIM10`, `TIM11`, `TIM13`, `TIM14`, `TIM16` and `TIM17`.
//!
//! # What is modelled, and what is not
//!
//! It says what it is rather than implying it is the whole peripheral.
//! Modelled:
//!
//! * The counter, the prescaler and the auto-reload, in all three counting
//!   directions: up, down, and center-aligned with `CMS`.
//! * The **shadow registers**. `PSC` is always buffered and `ARR` is buffered
//!   when `CR1.ARPE` is set, so a write lands at the next update event and not
//!   before; `CCRx` is buffered when `CCMRx.OCxPE` is set. Clearing the preload
//!   bit pushes the written value through immediately, which is what RM0090
//!   §17.4.7's "the new value is taken in account immediately" means.
//! * The update event: overflow, underflow, or `EGR.UG`, gated by `CR1.UDIS`
//!   and `CR1.URS`, and on an advanced timer divided by `RCR`. It reloads every
//!   shadow, sets `SR.UIF`, and stops the counter when `CR1.OPM` is set.
//! * Output compare, every mode of `OCxM`: frozen, active/inactive/toggle on
//!   match, the two forced levels, and PWM modes 1 and 2. Each channel is an
//!   output **wire** a board routes wherever the pin goes.
//! * `CCER`'s enables and polarities, `BDTR.MOE`, the complementary outputs
//!   `CH1N`–`CH3N`, and the `CR2.OISx` idle levels the outputs take when `MOE`
//!   is clear.
//! * `SR`'s write-zero-to-clear flags, `DIER`'s enables, and the interrupt
//!   output — one `irq` pin for a general or basic timer, which is what the
//!   vector table gives them, plus `irq-up`, `irq-cc` and `irq-trg` on an
//!   advanced timer, which has a vector each.
//! * **Input capture**: `CCMRx`'s input side (`CCxS`'s direct, indirect and
//!   `TRC` selections, `ICxPSC`, `ICxF`), `CCER`'s three input polarities,
//!   `CCRx` latching `CNT` on the selected edge, `CCxIF`, the over-capture
//!   flag `CCxOF`, and `CCxIF` clearing when the guest reads `CCRx`. The `TIx`
//!   pins are real inputs (`ti1`–`ti4`), with `CR2.TI1S`'s XOR on `TI1`.
//! * **The slave-mode controller**: `SMS` reset, gated, trigger and external
//!   clock mode 1, the three encoder modes, `TS`'s eight selections including
//!   `TI1F_ED`, and the external trigger path `ETF`/`ETPS`/`ECE`/`ETP` on an
//!   `etr` pin. `CR2.MMS` drives `TRGO` out, so `wire tim1.trgo -> tim2.itr0`
//!   chains two timers exactly as the part's internal trigger matrix does.
//! * **DMA**: `DIER`'s `UDE`/`CCxDE`/`TDE` raise `dma-up`, `dma-ch1`–`dma-ch4`
//!   and `dma-trg`, which a board wires to an `st.dma` `reqN` pin; `DCR`/`DMAR`
//!   are the burst window, so a request drives a real transfer end to end.
//!
//! **Not** modelled, deliberately, and each of these is a register that reads
//! back what was written and does nothing else:
//!
//! * **Dead-time insertion and the break input.** `BDTR` is stored, `MOE`
//!   acts, and `DTG` does not: a complementary pair switches with zero dead
//!   time. `BKIN` is not a pin.
//! * **`CR2.CCPC`/`CCUS` preloaded channel control** and the commutation
//!   event, and with them `COMIE`/`COMDE`.
//! * **`OR1`/`OR2`.** They remap which pin or which timer an input comes from,
//!   and in this model that is the board's wiring; there is nothing for them to
//!   switch.
//! * **`SMCR.MSM`**, which on the part delays `TRGO` by a clock so a master and
//!   its slave start on the same edge. Propagation here is instantaneous.
//!
//! # The digital input filters
//!
//! `ICxF` and `ETF` are *N* consecutive samples at a division of `f_DTS`
//! (RM0090 §17.4.7's table, with `CR1.CKD` setting `t_DTS`). A level-based
//! model cannot sample, so the filter is modelled as the hold time those *N*
//! samples take: a new level is accepted only once it has stood for
//! `N × divider × t_DTS`, and a level that goes back before then is a glitch
//! and is dropped. The numbers are the manual's; the deadline is part of the
//! snapshot and is one of the instants [`Tim::next_event`] reports, so a filter
//! expiring wakes the device exactly as an overflow does.
//!
//! One departure, called out where it happens: `ETPS` divides the external
//! clock *after* the filter here, where RM0090 §17.3.1's block diagram puts it
//! before. A divider cannot be inserted into a level.
//!
//! # Time
//!
//! **Lazily advanced** (`ROADMAP.md` §4.2) on its own clock domain, one tick of
//! which is one `CK_INT` cycle. Nothing here reads a host clock or sleeps:
//! [`Tim::next_event_tick`] reports the exact tick at which the next thing a
//! program can observe happens — the next overflow, the next underflow, the
//! next compare match — and the scheduler will not let anything run past it.
//! So an interrupt lands on the cycle it is due on rather than at the end of a
//! quantum, and the arithmetic that finds that cycle is integer throughout:
//!
//! ```text
//!   counter clocks available in n ticks = (psc_counter + n) / (PSC + 1)
//!   ticks spent producing k of them     = k * (PSC + 1) - psc_counter
//! ```
//!
//! There is no floating point and no wall clock anywhere in this file, and the
//! prescaler's phase (`psc_counter`) is part of the snapshot, so a restore
//! resumes mid-division rather than rounding to the next counter clock.
//!
//! # The clock input
//!
//! The machine file binds the domain, exactly as it does for [`super::i2c`]:
//!
//! ```text
//!   object tim2 "st.tim" { clock = hse * 21 / 4, variant = "general", width = 32 }
//! ```
//!
//! That expression is **`CK_INT` itself**, not the APB bus clock. On an F4 the
//! two differ: RM0090 §7.2 says that when an APB prescaler is other than 1 the
//! timer clocks are twice the APB clock, so a board doubles the ratio itself
//! rather than this device guessing which half of that rule applies. `CR1.CKD`
//! is stored and does not divide the counter — it is the dead-time and digital
//! filter divider, neither of which is modelled.
//!
//! Peripheral clock **gating** — `RCC_APB1ENR` — is not an input here. When a
//! clock controller lands it may either publish a gated domain, in which case
//! this device needs no change, or drive an enable, in which case the seam is
//! one new input pin. Neither is invented in advance.
//!
//! # Sources
//!
//! ST **RM0090** rev 21: §16 ("Advanced-control timers TIM1 and TIM8"), §17
//! ("General-purpose timers TIM2 to TIM5"), §18 ("General-purpose timers TIM9
//! to TIM14") and §19 ("Basic timers TIM6 and TIM7"), with §7.2 for where
//! `CK_INT` comes from. **§18 is the small general-purpose timers**, not the
//! advanced ones — this file said otherwise until the chapter numbering was
//! checked against the anchors the rest of `dev/stm32/` cites (GPIO §8, DMA
//! §10, EXTI §12, WWDG §20, IWDG §21), which leave exactly one slot for the
//! advanced timers.
//!
//! ST **RM0351** §31–§34 is the same map on an L4. No emulator source of any
//! licence was consulted (`ROADMAP.md` §1).

use alloc::boxed::Box;
use alloc::format;
use alloc::string::{String, ToString};
use alloc::sync::Arc;
use alloc::vec::Vec;
use core::fmt;

use crate::core::device::{Device, DeviceClass, PropertySpec, RealizeCtx, ResetKind, SinkPin};
use crate::core::error::{BusError, Error, Result};
use crate::core::props::{Props, ValueKind};
use crate::core::sched::{AccessKind, LazyHandle};
use crate::core::space::{AccessConstraints, MemAttrs, MemOps, MemResult, Region, RegionRef};
use crate::core::state::{ChunkReader, ChunkWriter, Sink, Source};
use crate::core::sync::{AtomicU32, AtomicU64, LockRank, Mutex, Ordering};
use crate::core::value::{Endian, Width};
use crate::core::wire::{FanIn, Level, Resolve, WireId, WireSink, WireSource};
use crate::machine::Instance;
use crate::machine::validate::{ClassSchema, PortDir, PropSchema};

/// The class name a machine description writes.
const CLASS_NAME: &str = "st.tim";

/// The snapshot chunk version. Bump it with the encoding, never on its own.
///
/// Version 2 added the input side: the filters' raw and pending levels, the
/// capture prescalers, `ITRx`, `TRGI`, `OR1`/`OR2` and `DMAR`'s burst index.
const STATE_VERSION: u32 = 2;

/// How many bytes the register block occupies: `CR1` through `OR2`.
///
/// `OR2` is at `0x60` on an L4 (RM0351 §31.6.22), so the window runs to `0x64`
/// even though an F4 stops at `TIMx_OR`'s `0x50`. Everything between reads as
/// zero, which is what an unimplemented register in the peripheral's own
/// kilobyte does.
pub const REGISTER_BYTES: u64 = 0x64;

/// The most capture/compare channels any variant has.
pub const MAX_CHANNELS: usize = 4;

/// How many of those have a complementary output on an advanced timer.
///
/// `CH4` has none — RM0090 §16's complementary-output table stops at `OC3N`.
pub const COMPLEMENTARY_CHANNELS: usize = 3;

/// The combined interrupt output, and the only one a basic or general-purpose
/// timer has.
pub const IRQ_PIN: &str = "irq";

/// The advanced timer's update vector — `TIM1_UP` on an F4.
pub const IRQ_UP_PIN: &str = "irq-up";

/// The advanced timer's capture/compare vector — `TIM1_CC` on an F4.
pub const IRQ_CC_PIN: &str = "irq-cc";

/// The advanced timer's trigger/commutation vector — `TIM1_TRG_COM` on an F4.
///
/// `SR.TIF` lands here rather than on [`IRQ_UP_PIN`] or [`IRQ_CC_PIN`]; on a
/// general-purpose timer it folds into [`IRQ_PIN`] with everything else.
pub const IRQ_TRG_PIN: &str = "irq-trg";

/// The master-mode trigger output, `TRGO`, selected by `CR2.MMS`.
pub const TRGO_PIN: &str = "trgo";

/// The external trigger input, `ETR`.
pub const ETR_PIN: &str = "etr";

/// How many internal trigger inputs the selector can choose between.
///
/// `TS` is three bits and four of its eight codes are `ITR0`–`ITR3`. Which
/// timer each one comes from is a fact about the **part** — RM0090 Table 74 for
/// `TIM1`/`TIM8` and Table 86 for `TIM2`–`TIM5` — so it is wiring in the board
/// file, exactly as `st.dma`'s request matrix is.
pub const ITR_INPUTS: usize = 4;

/// The update DMA request, `TIMx_UP`, gated by `DIER.UDE`.
pub const DMA_UP_PIN: &str = "dma-up";

/// The trigger DMA request, `TIMx_TRIG`, gated by `DIER.TDE`.
pub const DMA_TRG_PIN: &str = "dma-trg";

// -- CR1 ---------------------------------------------------------------------

/// `CR1.CEN` — counter enable.
const CR1_CEN: u32 = 1 << 0;
/// `CR1.UDIS` — update disable: no update event is generated at all.
const CR1_UDIS: u32 = 1 << 1;
/// `CR1.URS` — update request source: only an over/underflow sets `UIF`.
const CR1_URS: u32 = 1 << 2;
/// `CR1.OPM` — one-pulse mode: `CEN` clears at the next update event.
const CR1_OPM: u32 = 1 << 3;
/// `CR1.DIR` — direction, when `CMS` is edge-aligned. 1 is down.
const CR1_DIR: u32 = 1 << 4;
/// `CR1.CMS` — center-aligned mode selection, bits 6:5.
const CR1_CMS_SHIFT: u32 = 5;
/// `CR1.ARPE` — auto-reload preload enable.
const CR1_ARPE: u32 = 1 << 7;
/// `CR1.CKD` — the dead-time and digital-filter clock division, bits 9:8.
const CR1_CKD_SHIFT: u32 = 8;
/// `CR1.UIFREMAP` — copy `SR.UIF` into `CNT` bit 31 on a read.
///
/// An L4/F7/G4 bit (RM0351 §31.6.1); RM0090's `CR1` has bits 15:10 reserved, so
/// an F4 board simply never sets it.
const CR1_UIFREMAP: u32 = 1 << 11;

/// What a basic timer implements of `CR1`: RM0090 §19.4.1, plus `UIFREMAP`,
/// which RM0351 §34.4.1 gives `TIM6`/`TIM7` as well.
const CR1_MASK_BASIC: u32 = CR1_CEN | CR1_UDIS | CR1_URS | CR1_OPM | CR1_ARPE | CR1_UIFREMAP;
/// What the others implement: everything above plus `DIR`, `CMS`, `CKD` and
/// `UIFREMAP`.
const CR1_MASK_FULL: u32 = 0x0bff;

// -- CR2 ---------------------------------------------------------------------

/// `CR2.MMS` — master mode selection, bits 6:4. Stored; `TRGO` is not modelled.
const CR2_MASK_BASIC: u32 = 0x0070;
/// `CR2` on a general-purpose timer: `CCDS`, `MMS`, `TI1S`.
const CR2_MASK_GENERAL: u32 = 0x00f8;
/// `CR2` on an advanced timer: the above plus `CCPC`, `CCUS` and the `OISx`
/// idle levels at bits 8–14.
const CR2_MASK_ADVANCED: u32 = 0x7ffd;

/// Bit 8 + 2n is `OISn+1`, the idle level channel n takes when `MOE` is clear.
const CR2_OIS_SHIFT: u32 = 8;

/// `CR2.TI1S` — `TI1` is the XOR of the `TI1`, `TI2` and `TI3` pins.
const CR2_TI1S: u32 = 1 << 7;
/// `CR2.MMS` — master mode selection, bits 6:4.
const CR2_MMS_SHIFT: u32 = 4;

// -- master mode selection, RM0090 §17.4.2 -----------------------------------

/// `EGR.UG` is the trigger output.
const MMS_RESET: u32 = 0b000;
/// The counter-enable signal is the trigger output.
const MMS_ENABLE: u32 = 0b001;
/// The update event is the trigger output.
const MMS_UPDATE: u32 = 0b010;
/// A pulse whenever `CC1IF` is about to be set.
const MMS_COMPARE_PULSE: u32 = 0b011;
/// `OC1REF` is the trigger output; `101`–`111` are `OC2REF`–`OC4REF`.
const MMS_OC1REF: u32 = 0b100;

// -- DIER --------------------------------------------------------------------

/// `DIER.UIE` — update interrupt enable.
const DIER_UIE: u32 = 1 << 0;
/// `DIER.CC1IE` — bit 1 + n is channel n's.
const DIER_CC1IE: u32 = 1 << 1;
/// `DIER.TIE` — trigger interrupt enable.
const DIER_TIE: u32 = 1 << 6;
/// `DIER.UDE` — update DMA request enable.
const DIER_UDE: u32 = 1 << 8;
/// `DIER.CC1DE` — bit 9 + n is channel n's DMA request enable.
const DIER_CC1DE: u32 = 1 << 9;
/// `DIER.TDE` — trigger DMA request enable.
const DIER_TDE: u32 = 1 << 14;

/// A basic timer has `UIE` and `UDE` and nothing else.
const DIER_MASK_BASIC: u32 = 0x0101;
/// A general-purpose timer's: `UIE`, `CCxIE`, `TIE`, `UDE`, `CCxDE`, `TDE`.
const DIER_MASK_GENERAL: u32 = 0x5f5f;
/// An advanced timer's: the above plus `COMIE`, `BIE` and `COMDE`.
const DIER_MASK_ADVANCED: u32 = 0x7fff;

// -- SR ----------------------------------------------------------------------

/// `SR.UIF` — update interrupt flag.
const SR_UIF: u32 = 1 << 0;
/// `SR.CC1IF` — bit 1 + n is channel n's.
const SR_CC1IF: u32 = 1 << 1;
/// `SR.TIF` — trigger interrupt flag.
const SR_TIF: u32 = 1 << 6;
/// `SR.CC1OF` — bit 9 + n is channel n's over-capture flag.
const SR_CC1OF: u32 = 1 << 9;

/// Every flag a basic timer has.
const SR_MASK_BASIC: u32 = SR_UIF;
/// A general-purpose timer's: `UIF`, `CCxIF`, `TIF`, `CCxOF`.
const SR_MASK_GENERAL: u32 = 0x1e5f & !0x0020;
/// An advanced timer's: the above plus `COMIF` and `BIF`.
const SR_MASK_ADVANCED: u32 = 0x1eff;

// -- EGR ---------------------------------------------------------------------

/// `EGR.UG` — re-initialize the counter and generate an update of the registers.
const EGR_UG: u32 = 1 << 0;
/// `EGR.CC1G` — bit 1 + n generates channel n's capture/compare event.
const EGR_CC1G: u32 = 1 << 1;
/// `EGR.TG` — generate a trigger event by software.
const EGR_TG: u32 = 1 << 6;

// -- CCMR / CCER -------------------------------------------------------------

/// `CCMRx.OCxPE` within a channel's byte.
const CCMR_OCPE: u32 = 1 << 3;
/// `CCMRx.OCxM` within a channel's byte: bits 6:4.
const CCMR_OCM_SHIFT: u32 = 4;
/// `CCMRx.CCxS` within a channel's byte: bits 1:0. Nonzero is input mode.
const CCMR_CCS_MASK: u32 = 0x3;
/// `CCxS = 01`: the channel captures its **own** `TIx`.
const CCS_TI_DIRECT: u32 = 0b01;
/// `CCxS = 10`: the channel captures the **other** `TIx` of its pair.
const CCS_TI_INDIRECT: u32 = 0b10;
/// `CCxS = 11`: the channel captures on `TRC`, the slave controller's trigger.
const CCS_TRC: u32 = 0b11;
/// `CCMRx.ICxPSC` within a channel's byte: bits 3:2, the capture prescaler.
const CCMR_ICPSC_SHIFT: u32 = 2;
/// `CCMRx.ICxF` within a channel's byte: bits 7:4, the input filter. The same
/// bits `OCxM`/`OCxCE` occupy in output mode, which is why `CCxS` decides.
const CCMR_ICF_SHIFT: u32 = 4;

/// `CCER.CCxE` within a channel's nibble.
const CCER_CCE: u32 = 1 << 0;
/// `CCER.CCxP` within a channel's nibble.
const CCER_CCP: u32 = 1 << 1;
/// `CCER.CCxNE` within a channel's nibble — advanced timers only.
const CCER_CCNE: u32 = 1 << 2;
/// `CCER.CCxNP` within a channel's nibble.
const CCER_CCNP: u32 = 1 << 3;

// -- output compare modes, RM0090 §17.4.7 ------------------------------------

/// Frozen: the comparison has no effect on the output.
const OCM_FROZEN: u32 = 0b000;
/// Set the channel active on match.
const OCM_ACTIVE: u32 = 0b001;
/// Set the channel inactive on match.
const OCM_INACTIVE: u32 = 0b010;
/// Toggle on match.
const OCM_TOGGLE: u32 = 0b011;
/// Force inactive, regardless of the comparison.
const OCM_FORCE_INACTIVE: u32 = 0b100;
/// Force active, regardless of the comparison.
const OCM_FORCE_ACTIVE: u32 = 0b101;
/// PWM mode 1: active while the counter is below the compare value.
const OCM_PWM1: u32 = 0b110;
/// PWM mode 2: the inverse of mode 1.
const OCM_PWM2: u32 = 0b111;

// -- SMCR, the slave-mode controller, RM0090 §17.4.3 -------------------------

/// `SMCR.SMS` — slave mode selection, bits 2:0.
const SMCR_SMS_MASK: u32 = 0x7;
/// `SMCR.TS` — trigger selection, bits 6:4.
const SMCR_TS_SHIFT: u32 = 4;
/// `SMCR.ETF` — external trigger filter, bits 11:8. The `ICxF` table again.
const SMCR_ETF_SHIFT: u32 = 8;
/// `SMCR.ETPS` — external trigger prescaler, bits 13:12: 1, 2, 4 or 8.
const SMCR_ETPS_SHIFT: u32 = 12;
/// `SMCR.ECE` — external clock enable: external clock **mode 2**, in which
/// `ETRF` clocks the counter whatever `SMS` says.
const SMCR_ECE: u32 = 1 << 14;
/// `SMCR.ETP` — external trigger polarity; 1 selects the falling edge.
const SMCR_ETP: u32 = 1 << 15;

/// What a general-purpose or advanced timer implements of `SMCR`: all of it.
///
/// `MSM` at bit 7 is in there and is storage — it delays `TRGO` by a clock on
/// the part so that a master and its slave start together, and propagation
/// here is already instantaneous.
const SMCR_MASK: u32 = 0xffff;

/// `SMS = 000`: the slave controller is off and `CK_INT` clocks the prescaler.
const SMS_DISABLED: u32 = 0b000;
/// Encoder mode 1: count on `TI2FP2` edges, direction from `TI1FP1`'s level.
const SMS_ENCODER1: u32 = 0b001;
/// Encoder mode 2: count on `TI1FP1` edges, direction from `TI2FP2`'s level.
const SMS_ENCODER2: u32 = 0b010;
/// Encoder mode 3: count on both inputs' edges.
const SMS_ENCODER3: u32 = 0b011;
/// Reset mode: a trigger reinitializes the counter and updates the registers.
const SMS_RESET: u32 = 0b100;
/// Gated mode: the counter is clocked while `TRGI` is high.
const SMS_GATED: u32 = 0b101;
/// Trigger mode: a trigger starts the counter, and nothing stops it.
const SMS_TRIGGER: u32 = 0b110;
/// External clock mode 1: `TRGI`'s rising edges clock the counter.
const SMS_EXT1: u32 = 0b111;

/// `TS = 100`: `TI1F_ED`, the *edge detector* — both edges of `TI1F`.
const TS_TI1F_ED: u32 = 0b100;
/// `TS = 101`: `TI1FP1`, filtered timer input 1 with channel 1's polarity.
const TS_TI1FP1: u32 = 0b101;
/// `TS = 110`: `TI2FP2`, filtered timer input 2 with channel 2's polarity.
const TS_TI2FP2: u32 = 0b110;
/// `TS = 111`: `ETRF`, the filtered external trigger.
const TS_ETRF: u32 = 0b111;

// -- the DMA request lines ---------------------------------------------------

/// The update request, `TIMx_UP`.
const DMA_UP: u32 = 1 << 0;
/// Channel 1's request, `TIMx_CH1`; bit 1 + n is channel n's.
const DMA_CC1: u32 = 1 << 1;
/// The trigger request, `TIMx_TRIG`.
const DMA_TRG: u32 = 1 << 5;

/// How deep a `TRGO` cascade may go before it is cut.
///
/// Two timers can be wired to clock one another — the part allows it, and a
/// board file has no way to tell. On silicon that merely oscillates; here it is
/// unbounded recursion, so the chain is cut at a depth no real topology
/// reaches. An F4 has six chainable timers.
const MAX_TRGO_DEPTH: u32 = 16;

// -- BDTR --------------------------------------------------------------------

/// `BDTR.MOE` — main output enable. Clear it and every output goes idle.
const BDTR_MOE: u32 = 1 << 15;

// -- register offsets --------------------------------------------------------

const OFF_CR1: u64 = 0x00;
const OFF_CR2: u64 = 0x04;
const OFF_SMCR: u64 = 0x08;
const OFF_DIER: u64 = 0x0c;
const OFF_SR: u64 = 0x10;
const OFF_EGR: u64 = 0x14;
const OFF_CCMR1: u64 = 0x18;
const OFF_CCMR2: u64 = 0x1c;
const OFF_CCER: u64 = 0x20;
const OFF_CNT: u64 = 0x24;
const OFF_PSC: u64 = 0x28;
const OFF_ARR: u64 = 0x2c;
const OFF_RCR: u64 = 0x30;
const OFF_CCR1: u64 = 0x34;
const OFF_BDTR: u64 = 0x44;
const OFF_DCR: u64 = 0x48;
const OFF_DMAR: u64 = 0x4c;
const OFF_OR1: u64 = 0x50;
const OFF_OR2: u64 = 0x60;

// ---------------------------------------------------------------------------
// Variant
// ---------------------------------------------------------------------------

/// Which subset of the block an instance is.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Variant {
    /// `TIM6`/`TIM7`: a counter and an update interrupt, nothing else.
    Basic,
    /// `TIM2`–`TIM5` and the small general-purpose timers.
    General,
    /// `TIM1`/`TIM8`: `RCR`, `BDTR` and complementary outputs.
    Advanced,
}

impl Variant {
    /// The spelling a machine file writes.
    #[must_use]
    pub fn as_str(self) -> &'static str {
        match self {
            Variant::Basic => "basic",
            Variant::General => "general",
            Variant::Advanced => "advanced",
        }
    }

    /// How many capture/compare channels this variant has by default.
    fn default_channels(self) -> u64 {
        match self {
            Variant::Basic => 0,
            Variant::General | Variant::Advanced => 4,
        }
    }
}

// ---------------------------------------------------------------------------
// Configuration
// ---------------------------------------------------------------------------

/// What a machine file fixed about this instance, and cannot change afterwards.
#[derive(Debug, Clone, Copy)]
struct Config {
    variant: Variant,
    /// The counter's width mask: `0xffff` or `0xffff_ffff`.
    mask: u32,
    /// How many capture/compare channels exist.
    channels: usize,
    /// What `ARR` reads out of reset.
    arr_reset: u32,
}

impl Config {
    /// Whether this instance has complementary outputs at all.
    fn advanced(&self) -> bool {
        matches!(self.variant, Variant::Advanced)
    }

    /// How many complementary outputs it bonds.
    fn complementary(&self) -> usize {
        if self.advanced() {
            self.channels.min(COMPLEMENTARY_CHANNELS)
        } else {
            0
        }
    }

    fn cr1_mask(&self) -> u32 {
        match self.variant {
            Variant::Basic => CR1_MASK_BASIC,
            _ => CR1_MASK_FULL,
        }
    }

    fn cr2_mask(&self) -> u32 {
        match self.variant {
            Variant::Basic => CR2_MASK_BASIC,
            Variant::General => CR2_MASK_GENERAL,
            Variant::Advanced => CR2_MASK_ADVANCED,
        }
    }

    fn dier_mask(&self) -> u32 {
        match self.variant {
            Variant::Basic => DIER_MASK_BASIC,
            Variant::General => DIER_MASK_GENERAL,
            Variant::Advanced => DIER_MASK_ADVANCED,
        }
    }

    fn sr_mask(&self) -> u32 {
        match self.variant {
            Variant::Basic => SR_MASK_BASIC,
            Variant::General => SR_MASK_GENERAL,
            Variant::Advanced => SR_MASK_ADVANCED,
        }
    }

    /// The `CCER` bits that exist: a nibble per channel, minus `CCxNE` where
    /// there is no complementary output.
    fn ccer_mask(&self) -> u32 {
        let mut mask = 0;
        for i in 0..self.channels {
            let nibble = CCER_CCE | CCER_CCP | CCER_CCNP;
            let extra = if i < self.complementary() {
                CCER_CCNE
            } else {
                0
            };
            mask |= (nibble | extra) << (4 * i);
        }
        mask
    }
}

// ---------------------------------------------------------------------------
// State
// ---------------------------------------------------------------------------

/// Everything the guest can see or change, plus the two shadow sets.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct Regs {
    cr1: u32,
    cr2: u32,
    smcr: u32,
    dier: u32,
    sr: u32,
    ccmr: [u32; 2],
    ccer: u32,
    bdtr: u32,
    dcr: u32,
    dmar: u32,

    /// The counter itself.
    cnt: u32,
    /// How many `CK_INT` ticks have accumulated towards the next counter clock.
    psc_count: u32,
    /// `PSC` as written: the preload register a read returns.
    psc: u32,
    /// `PSC` as the prescaler is actually dividing by, minus one.
    psc_shadow: u32,
    /// `ARR` as written.
    arr: u32,
    /// `ARR` as the counter is actually comparing against.
    arr_shadow: u32,
    /// `RCR` as written.
    rcr: u32,
    /// How many more over/underflows before the next update event.
    rcr_count: u32,
    /// `CCRx` as written.
    ccr: [u32; MAX_CHANNELS],
    /// `CCRx` as the comparator is actually using.
    ccr_shadow: [u32; MAX_CHANNELS],

    /// Which way the counter is going, in center-aligned mode.
    down: bool,
    /// Each channel's `OCxREF`, before polarity and the output enables.
    ///
    /// State rather than derived: in toggle mode it is a latch, and in frozen
    /// mode it holds whatever the last match left.
    ocref: [bool; MAX_CHANNELS],

    /// `OR1` and `OR2`. Stored and read back; what they remap — which pin a
    /// `TIx` or an `ITRx` actually comes from — is board wiring here, so there
    /// is nothing for them to switch.
    or: [u32; 2],

    /// The raw level each `TIx` pin's net last delivered.
    ti_raw: [bool; MAX_CHANNELS],
    /// The same after `CR2.TI1S` and the `ICxF` filter: `TIxF`.
    ti_filt: [bool; MAX_CHANNELS],
    /// A level waiting out its filter, and the tick it becomes `TIxF` at.
    ti_pend: [bool; MAX_CHANNELS],
    /// [`u64::MAX`] when nothing is pending.
    ti_pend_at: [u64; MAX_CHANNELS],
    /// `ICxPSC`'s phase: active edges seen since the last capture.
    ic_count: [u32; MAX_CHANNELS],

    /// The raw level on `ETR`.
    etr_raw: bool,
    /// The same after `ETF`; `ETP` is applied on top of this, not into it.
    etr_filt: bool,
    /// The pending `ETF` sample, as for a `TIx`.
    etr_pend: bool,
    etr_pend_at: u64,
    /// `ETPS`'s phase.
    etps_count: u32,

    /// The level on each `ITRx` input pin.
    itr: [bool; ITR_INPUTS],
    /// `TRGI` as the selector last resolved it — the edge detector's memory.
    trgi: bool,

    /// `DMAR`'s burst index, counting 0..=`DCR.DBL`.
    dma_index: u32,
    /// DMA requests raised and not yet pulsed out: [`DMA_UP`] and friends.
    ///
    /// A **set**, not a count. `st.dma` latches a request in one boolean per
    /// unit, so two requests raised between two of its own beats buy one beat
    /// there however many pulses arrive — and in a running machine the point is
    /// moot, because the scheduler walks a lazy device event by event and each
    /// update gets its own [`Shared::drive`] anyway.
    dma_pending: u32,
    /// `TRGO` pulses raised and not yet driven out.
    ///
    /// A count, unlike the DMA set: a slave in external clock mode 1 counts the
    /// pulses, so losing one loses a count. A caller that advances straight
    /// across several periods must still deliver every one of them.
    trgo_pulses: u32,
}

impl Regs {
    /// The state a peripheral reset leaves, for this configuration.
    fn reset(cfg: &Config) -> Regs {
        Regs {
            cr1: 0,
            cr2: 0,
            smcr: 0,
            dier: 0,
            sr: 0,
            ccmr: [0; 2],
            ccer: 0,
            // `MOE` is a bit only an advanced timer has; everything else
            // behaves as though its outputs were permanently enabled.
            bdtr: 0,
            dcr: 0,
            dmar: 0,
            cnt: 0,
            psc_count: 0,
            psc: 0,
            psc_shadow: 0,
            arr: cfg.arr_reset,
            arr_shadow: cfg.arr_reset,
            rcr: 0,
            rcr_count: 0,
            ccr: [0; MAX_CHANNELS],
            ccr_shadow: [0; MAX_CHANNELS],
            down: false,
            ocref: [false; MAX_CHANNELS],
            or: [0; 2],
            ti_raw: [false; MAX_CHANNELS],
            ti_filt: [false; MAX_CHANNELS],
            ti_pend: [false; MAX_CHANNELS],
            ti_pend_at: [u64::MAX; MAX_CHANNELS],
            ic_count: [0; MAX_CHANNELS],
            etr_raw: false,
            etr_filt: false,
            etr_pend: false,
            etr_pend_at: u64::MAX,
            etps_count: 0,
            itr: [false; ITR_INPUTS],
            trgi: false,
            dma_index: 0,
            dma_pending: 0,
            trgo_pulses: 0,
        }
    }

    /// `CR1.CMS`, 0 for edge-aligned.
    fn cms(&self) -> u32 {
        (self.cr1 >> CR1_CMS_SHIFT) & 0x3
    }

    /// Whether the counter is going down right now.
    fn counting_down(&self) -> bool {
        if self.cms() != 0 {
            self.down
        } else {
            self.cr1 & CR1_DIR != 0
        }
    }

    /// Channel `i`'s byte of `CCMR1`/`CCMR2`.
    fn ccmr_byte(&self, i: usize) -> u32 {
        (self.ccmr[i / 2] >> (8 * (i % 2))) & 0xff
    }

    /// Channel `i`'s `OCxM`.
    fn ocm(&self, i: usize) -> u32 {
        (self.ccmr_byte(i) >> CCMR_OCM_SHIFT) & 0x7
    }

    /// Whether channel `i` preloads its compare register.
    fn ocpe(&self, i: usize) -> bool {
        self.ccmr_byte(i) & CCMR_OCPE != 0
    }

    /// Whether channel `i` is configured as an input, in which case this model
    /// leaves its output alone.
    fn is_input(&self, i: usize) -> bool {
        self.ccmr_byte(i) & CCMR_CCS_MASK != 0
    }

    /// Channel `i`'s nibble of `CCER`.
    fn ccer_nibble(&self, i: usize) -> u32 {
        (self.ccer >> (4 * i)) & 0xf
    }

    /// `CCMRx.CCxS` for channel `i`.
    fn ccs(&self, i: usize) -> u32 {
        self.ccmr_byte(i) & CCMR_CCS_MASK
    }

    /// Channel `i`'s `ICxPSC`, as a shift: a capture every `1 << n` edges.
    fn icpsc(&self, i: usize) -> u32 {
        (self.ccmr_byte(i) >> CCMR_ICPSC_SHIFT) & 0x3
    }

    /// Channel `i`'s `ICxF`.
    fn icf(&self, i: usize) -> u32 {
        (self.ccmr_byte(i) >> CCMR_ICF_SHIFT) & 0xf
    }

    /// `CR1.CKD`, which sets `t_DTS` and so the filters' sampling rate.
    fn ckd(&self) -> u32 {
        (self.cr1 >> CR1_CKD_SHIFT) & 0x3
    }

    /// `SMCR.SMS`.
    fn sms(&self) -> u32 {
        self.smcr & SMCR_SMS_MASK
    }

    /// `SMCR.TS`.
    fn ts(&self) -> u32 {
        (self.smcr >> SMCR_TS_SHIFT) & 0x7
    }

    /// `CR2.MMS`.
    fn mms(&self) -> u32 {
        (self.cr2 >> CR2_MMS_SHIFT) & 0x7
    }

    /// Whether the slave controller is in one of the three encoder modes.
    fn encoder(&self) -> bool {
        matches!(self.sms(), SMS_ENCODER1 | SMS_ENCODER2 | SMS_ENCODER3)
    }
}

/// How many `CK_INT` ticks an input must hold a new level for a digital filter
/// coded `code` to accept it.
///
/// RM0090 §17.4.7's `ICxF` table, read as *N* consecutive samples at
/// `f_SAMPLING`, with `f_DTS = f_CK_INT / (1 << CKD)`. Codes `0001`–`0011`
/// sample at `f_CK_INT` itself; the rest sample at a division of `f_DTS`. The
/// numbers are the manual's, which makes them fact rather than anyone's
/// expression of it (`CLAUDE.md`, *Provenance*).
fn filter_ticks(code: u32, ckd: u32) -> u64 {
    /// `(sampling divider, N)` per `ICxF` code.
    const TABLE: [(u64, u64); 16] = [
        (1, 0),
        (1, 2),
        (1, 4),
        (1, 8),
        (2, 6),
        (2, 8),
        (4, 6),
        (4, 8),
        (8, 6),
        (8, 8),
        (16, 5),
        (16, 6),
        (16, 8),
        (32, 5),
        (32, 6),
        (32, 8),
    ];
    let (div, n) = TABLE[(code & 0xf) as usize];
    if n == 0 {
        return 0;
    }
    let dts = if code <= 3 { 1 } else { 1u64 << ckd.min(2) };
    n * div * dts
}

/// Feed a new raw level into a filtered input.
///
/// Returns whether the filter accepts the level at once. Otherwise `pend` and
/// `pend_at` are left describing what is waiting, and a level that goes back to
/// where the filter already is cancels whatever was pending — which is what a
/// glitch *is*.
fn filter_input(
    filt: bool,
    pend: &mut bool,
    pend_at: &mut u64,
    level: bool,
    hold: u64,
    now: u64,
) -> bool {
    if level == filt {
        *pend_at = u64::MAX;
        return false;
    }
    if hold == 0 {
        *pend_at = u64::MAX;
        return true;
    }
    if *pend_at == u64::MAX || *pend != level {
        *pend = level;
        *pend_at = now + hold;
    }
    false
}

// ---------------------------------------------------------------------------
// Outputs
// ---------------------------------------------------------------------------

/// The levels every pin of this device should be driving.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
struct Levels {
    irq: bool,
    irq_up: bool,
    irq_cc: bool,
    irq_trg: bool,
    trgo: bool,
    ch: [bool; MAX_CHANNELS],
    chn: [bool; COMPLEMENTARY_CHANNELS],
}

/// Where each output pin goes, once the machine graph has connected it.
#[derive(Debug, Default, Clone)]
struct Links {
    irq: Option<WireSource>,
    irq_up: Option<WireSource>,
    irq_cc: Option<WireSource>,
    irq_trg: Option<WireSource>,
    trgo: Option<WireSource>,
    ch: [Option<WireSource>; MAX_CHANNELS],
    chn: [Option<WireSource>; COMPLEMENTARY_CHANNELS],
    dma_up: Option<WireSource>,
    dma_ch: [Option<WireSource>; MAX_CHANNELS],
    dma_trg: Option<WireSource>,
}

// ---------------------------------------------------------------------------
// Shared
// ---------------------------------------------------------------------------

/// What the register block and the device share.
struct Shared {
    cfg: Config,
    regs: Mutex<Regs>,
    links: Mutex<Links>,
    lazy: Mutex<Option<LazyHandle>>,
    /// The tick reached, republished on every advance.
    ///
    /// The scheduler asks a lazily-advanced device where it is with its slot
    /// held at [`LockRank::LEAF`], so this may not be behind a lock.
    tick: AtomicU64,
    /// The tick of the next observable change, or [`u64::MAX`] for none.
    next_event: AtomicU64,
    /// How deep the current `TRGO` cascade is — see [`MAX_TRGO_DEPTH`].
    depth: AtomicU32,
}

impl fmt::Debug for Shared {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let mut s = f.debug_struct("Shared");
        s.field("cfg", &self.cfg)
            .field("tick", &self.tick.load(Ordering::Relaxed));
        match self.regs.try_lock() {
            Some(regs) => s.field("regs", &*regs),
            None => s.field("regs", &"<locked>"),
        };
        s.finish()
    }
}

impl Shared {
    // -- the counting model --------------------------------------------------

    /// Whether the counter is running at all.
    ///
    /// RM0090 §19.4.8 on `TIMx_ARR`: "The counter is blocked while the
    /// auto-reload value is null." That is a real behaviour and it is also what
    /// keeps a freshly reset timer from asking the scheduler for an event every
    /// single tick.
    fn counting(&self, regs: &Regs) -> bool {
        self.cnt_en(regs) && regs.arr_shadow != 0 && !self.externally_clocked(regs)
    }

    /// `CNT_EN`: whether the counter is enabled at all, gate included.
    ///
    /// This is what `CR2.MMS = 001` exports on `TRGO`, and it is `CEN` and the
    /// gated mode's `TRGI` level together — RM0090 §17.4.2.
    fn cnt_en(&self, regs: &Regs) -> bool {
        if regs.cr1 & CR1_CEN == 0 {
            return false;
        }
        if self.slave_enabled(regs) && regs.sms() == SMS_GATED {
            return self.trgi_level(regs);
        }
        true
    }

    /// Whether the slave-mode controller is doing anything.
    ///
    /// A basic timer has no `SMCR` at all — RM0090 §19.4 — so its register is
    /// never written and this is always false for one.
    fn slave_enabled(&self, regs: &Regs) -> bool {
        !matches!(self.cfg.variant, Variant::Basic) && regs.sms() != SMS_DISABLED
    }

    /// Whether something other than `CK_INT` is clocking the prescaler.
    ///
    /// The three encoder modes and external clock mode 1 take the counter off
    /// the internal clock, and so does `ECE` — external clock mode 2 — whatever
    /// `SMS` says (RM0090 §17.3.3: "If external clock mode 1 and external clock
    /// mode 2 are enabled at the same time, the external clock input is
    /// ETRF"). A device in this state has **no internal event to schedule**:
    /// everything it does happens on an edge arriving at a pin.
    fn externally_clocked(&self, regs: &Regs) -> bool {
        if matches!(self.cfg.variant, Variant::Basic) {
            return false;
        }
        if regs.smcr & SMCR_ECE != 0 {
            return true;
        }
        matches!(
            regs.sms(),
            SMS_ENCODER1 | SMS_ENCODER2 | SMS_ENCODER3 | SMS_EXT1
        )
    }

    // -- the slave-mode controller (RM0090 §17.3.15, §17.4.3) ----------------

    /// `TI1`'s source, before the filter: the pin, or `CR2.TI1S`'s XOR.
    ///
    /// RM0090 §17.4.2 on `TI1S`: "the TIMx_CH1, CH2 and CH3 pins are connected
    /// to the TI1 input (XOR combination)" — the Hall-sensor interface.
    fn ti_source(&self, regs: &Regs, i: usize) -> bool {
        if i == 0 && regs.cr2 & CR2_TI1S != 0 && self.cfg.channels >= 3 {
            regs.ti_raw[0] ^ regs.ti_raw[1] ^ regs.ti_raw[2]
        } else {
            regs.ti_raw[i]
        }
    }

    /// `TIxFPx`: the filtered input with its own channel's polarity applied.
    fn ti_fp(&self, regs: &Regs, i: usize) -> bool {
        regs.ti_filt[i] != (regs.ccer_nibble(i) & CCER_CCP != 0)
    }

    /// `ETRF`: the filtered external trigger, with `ETP` applied.
    fn etrf(&self, regs: &Regs) -> bool {
        regs.etr_filt != (regs.smcr & SMCR_ETP != 0)
    }

    /// `TRGI`'s level, for the selector `SMCR.TS` names.
    ///
    /// `TS = 100` is `TI1F_ED`, an *edge detector*: it has no level at all, so
    /// it reads low here and its events are delivered from the edge handler.
    fn trgi_level(&self, regs: &Regs) -> bool {
        match regs.ts() {
            t @ 0..=3 => regs.itr[t as usize],
            TS_TI1FP1 if self.cfg.channels >= 1 => self.ti_fp(regs, 0),
            TS_TI2FP2 if self.cfg.channels >= 2 => self.ti_fp(regs, 1),
            TS_ETRF => self.etrf(regs),
            // `TI1F_ED`, and the two `TIxFPx` codes on an instance that bonds
            // no such channel.
            _ => false,
        }
    }

    /// Re-resolve `TRGI` after something that feeds it moved, and act on the
    /// edge.
    ///
    /// `prev` is the level before the change. `MSM` is stored and changes
    /// nothing here: it exists on the part to delay `TRGO` by one clock so that
    /// a master and its slave start on the same edge, and propagation in this
    /// model is already instantaneous.
    fn settle_trigger(&self, regs: &mut Regs, prev: bool) {
        let now = self.trgi_level(regs);
        regs.trgi = now;
        if now == prev || !self.slave_enabled(regs) {
            return;
        }
        if regs.sms() == SMS_GATED {
            // "It is set when the counter starts or stops when gated mode is
            // selected" — both edges, and no counter action beyond the gate.
            regs.sr |= SR_TIF;
            self.request_dma(regs, DMA_TRG);
            return;
        }
        if now {
            self.on_trigger_event(regs);
        }
    }

    /// An active edge on `TRGI`, with the slave controller enabled.
    fn on_trigger_event(&self, regs: &mut Regs) {
        match regs.sms() {
            // "the counter and its prescaler can be reinitialized in response
            // to an event on a trigger input" — RM0090 §17.3.15.
            SMS_RESET => self.software_update(regs),
            SMS_TRIGGER => regs.cr1 |= CR1_CEN,
            // `ETRF` has its own path, because `ETPS` divides it there, and
            // `ECE` has already clocked the counter if it is set.
            SMS_EXT1 if regs.ts() != TS_ETRF && regs.smcr & SMCR_ECE == 0 => {
                self.external_clock(regs);
            }
            _ => {}
        }
        regs.sr |= SR_TIF;
        self.request_dma(regs, DMA_TRG);
        // `CCxS = 11` captures on `TRC`, which is this same trigger.
        for i in 0..self.cfg.channels {
            if regs.ccs(i) == CCS_TRC {
                self.do_capture(regs, i);
            }
        }
    }

    /// One clock from somewhere other than `CK_INT`.
    ///
    /// The prescaler sits *after* the clock mux (RM0090 §17.3.1's block
    /// diagram), so `PSC` divides an external clock exactly as it divides the
    /// internal one, and `psc_count` is the same phase either way.
    fn external_clock(&self, regs: &mut Regs) {
        if regs.cr1 & CR1_CEN == 0 || regs.arr_shadow == 0 {
            return;
        }
        regs.psc_count += 1;
        if regs.psc_count <= regs.psc_shadow {
            return;
        }
        regs.psc_count = 0;
        self.step(regs, 1);
        self.settle_matches(regs);
        self.settle_outputs(regs);
    }

    /// Raise a DMA request, if `DIER` has its enable set.
    ///
    /// The request is a **pulse** on the way out — [`Shared::drive`] takes it
    /// high and straight back low — which is what `st.dma`'s module
    /// documentation calls a single-item peripheral, and it buys exactly one
    /// beat per event.
    fn request_dma(&self, regs: &mut Regs, which: u32) {
        let enable = match which {
            DMA_UP => DIER_UDE,
            DMA_TRG => DIER_TDE,
            _ => DIER_CC1DE << (which.trailing_zeros() - 1),
        };
        if regs.dier & enable != 0 {
            regs.dma_pending |= which;
        }
    }

    /// `SR.UIF` is being set: the request and the trigger output that go with
    /// it.
    fn raise_uif(&self, regs: &mut Regs) {
        regs.sr |= SR_UIF;
        self.request_dma(regs, DMA_UP);
        if regs.mms() == MMS_UPDATE {
            regs.trgo_pulses = regs.trgo_pulses.saturating_add(1);
        }
    }

    // -- input capture (RM0090 §17.3.5) --------------------------------------

    /// Which `TIx` channel `ch` captures, if it captures a `TIx` at all.
    fn capture_source(&self, regs: &Regs, ch: usize) -> Option<usize> {
        match regs.ccs(ch) {
            // `CC1S = 01` is `TI1`, `10` is `TI2`; the channels pair 1/2, 3/4.
            CCS_TI_DIRECT => Some(ch),
            CCS_TI_INDIRECT => Some(ch ^ 1),
            _ => None,
        }
    }

    /// Whether this edge is the one `CCER`'s polarity selects.
    ///
    /// RM0090 §17.4.9 on `CCxNP`/`CCxP` in input mode: `00` is the rising edge,
    /// `01` the falling one, `11` both, and `10` is reserved.
    fn ic_active(&self, regs: &Regs, ch: usize, rising: bool) -> bool {
        let nibble = regs.ccer_nibble(ch);
        match (nibble & CCER_CCNP != 0, nibble & CCER_CCP != 0) {
            (true, true) => true,
            (false, true) => !rising,
            _ => rising,
        }
    }

    /// Latch `CNT` into `CCRx`, through `ICxPSC`.
    fn do_capture(&self, regs: &mut Regs, ch: usize) {
        if regs.ccer_nibble(ch) & CCER_CCE == 0 {
            // "the prescaler is reset as soon as CCxE = 0" — RM0090 §17.4.7.
            regs.ic_count[ch] = 0;
            return;
        }
        regs.ic_count[ch] += 1;
        if regs.ic_count[ch] < 1 << regs.icpsc(ch) {
            return;
        }
        regs.ic_count[ch] = 0;
        // "if the CCxIF flag was already high ... the over-capture flag CCxOF
        // is set" — and the captured value is overwritten all the same.
        if regs.sr & (SR_CC1IF << ch) != 0 {
            regs.sr |= SR_CC1OF << ch;
        }
        // An input channel has no preload: `CCRx` and its shadow are the same
        // register, and the comparator is not in the path at all.
        regs.ccr[ch] = regs.cnt;
        regs.ccr_shadow[ch] = regs.cnt;
        regs.sr |= SR_CC1IF << ch;
        self.request_dma(regs, DMA_CC1 << ch);
    }

    /// A filtered `TIx` has changed level.
    fn on_ti_edge(&self, regs: &mut Regs, i: usize, level: bool) {
        let prev = regs.trgi;
        for ch in 0..self.cfg.channels {
            if self.capture_source(regs, ch) == Some(i) && self.ic_active(regs, ch, level) {
                self.do_capture(regs, ch);
            }
        }
        if regs.encoder() {
            self.on_encoder_edge(regs, i);
        }
        if i == 0 && regs.ts() == TS_TI1F_ED && self.slave_enabled(regs) {
            // The edge detector fires on **both** edges of `TI1F`.
            self.on_trigger_event(regs);
        }
        self.settle_trigger(regs, prev);
    }

    /// `ETR` has changed level, after `ETF`.
    fn on_etr_edge(&self, regs: &mut Regs) {
        let prev = regs.trgi;
        if self.etrf(regs) {
            // `ETPS` divides the external *clock*; the trigger selector sees
            // the undivided `ETRF`. That is the one place this departs from
            // RM0090 §17.3.1's block diagram, because a level-based filter
            // cannot be placed after a divider.
            let div = 1u32 << ((regs.smcr >> SMCR_ETPS_SHIFT) & 0x3);
            regs.etps_count += 1;
            if regs.etps_count >= div {
                regs.etps_count = 0;
                let mode1 =
                    self.slave_enabled(regs) && regs.sms() == SMS_EXT1 && regs.ts() == TS_ETRF;
                if regs.smcr & SMCR_ECE != 0 || mode1 {
                    self.external_clock(regs);
                }
            }
        }
        self.settle_trigger(regs, prev);
    }

    /// A quadrature edge, in one of the three encoder modes.
    ///
    /// RM0090 §17.3.12's counting-direction table, written out as a rule:
    /// on a `TI1FP1` edge the counter goes **up** when the two signals differ
    /// and down when they agree; on a `TI2FP2` edge, the other way round. That
    /// is the phase relationship, and it is why mode 3 — both edges of both
    /// inputs — gets four counts per quadrature cycle.
    fn on_encoder_edge(&self, regs: &mut Regs, i: usize) {
        if self.cfg.channels < 2 || i > 1 {
            return;
        }
        let counts = matches!(
            (regs.sms(), i),
            (SMS_ENCODER1, 1) | (SMS_ENCODER2, 0) | (SMS_ENCODER3, 0 | 1)
        );
        if !counts {
            return;
        }
        let a = self.ti_fp(regs, 0);
        let b = self.ti_fp(regs, 1);
        let up = if i == 0 { a != b } else { a == b };
        if up {
            regs.cr1 &= !CR1_DIR;
        } else {
            regs.cr1 |= CR1_DIR;
        }
        self.external_clock(regs);
    }

    /// Re-run every `TIx` through its filter, and act on whatever it accepts.
    ///
    /// Called when a pin moved, and when something that *changes* the filter
    /// moved: `ICxF`, `CR1.CKD`, `CR2.TI1S`.
    fn refeed_ti(&self, regs: &mut Regs, now: u64) {
        for i in 0..self.cfg.channels {
            let level = self.ti_source(regs, i);
            let hold = filter_ticks(regs.icf(i), regs.ckd());
            let mut pend = regs.ti_pend[i];
            let mut at = regs.ti_pend_at[i];
            let accept = filter_input(regs.ti_filt[i], &mut pend, &mut at, level, hold, now);
            regs.ti_pend[i] = pend;
            regs.ti_pend_at[i] = at;
            if accept {
                regs.ti_filt[i] = level;
                self.on_ti_edge(regs, i, level);
            }
        }
    }

    /// The same for `ETR` and `ETF`.
    fn refeed_etr(&self, regs: &mut Regs, now: u64) {
        if matches!(self.cfg.variant, Variant::Basic) {
            return;
        }
        let hold = filter_ticks((regs.smcr >> SMCR_ETF_SHIFT) & 0xf, regs.ckd());
        let mut pend = regs.etr_pend;
        let mut at = regs.etr_pend_at;
        let accept = filter_input(regs.etr_filt, &mut pend, &mut at, regs.etr_raw, hold, now);
        regs.etr_pend = pend;
        regs.etr_pend_at = at;
        if accept {
            regs.etr_filt = regs.etr_raw;
            self.on_etr_edge(regs);
        }
    }

    /// The earliest tick a filter is due to accept a sample at.
    fn next_filter_deadline(&self, regs: &Regs) -> u64 {
        let mut best = regs.etr_pend_at;
        for i in 0..self.cfg.channels {
            best = best.min(regs.ti_pend_at[i]);
        }
        best
    }

    /// Accept every filter sample whose hold time has run out by `now`.
    fn settle_filters(&self, regs: &mut Regs, now: u64) {
        for i in 0..self.cfg.channels {
            if regs.ti_pend_at[i] <= now {
                regs.ti_pend_at[i] = u64::MAX;
                let level = regs.ti_pend[i];
                if level != regs.ti_filt[i] {
                    regs.ti_filt[i] = level;
                    self.on_ti_edge(regs, i, level);
                }
            }
        }
        if regs.etr_pend_at <= now {
            regs.etr_pend_at = u64::MAX;
            if regs.etr_pend != regs.etr_filt {
                regs.etr_filt = regs.etr_pend;
                self.on_etr_edge(regs);
            }
        }
    }

    /// A pin's net has delivered a new level.
    fn input_changed(&self, which: Input, level: bool) {
        // Catch the counter up first: a capture latches `CNT` as it is at the
        // instant of the edge, not as it was when the last quantum ended.
        self.sync(MemAttrs::DEFAULT);
        {
            let mut regs = self.regs.lock();
            let now = self.tick.load(Ordering::Relaxed);
            match which {
                Input::Ti(i) => {
                    if i < self.cfg.channels {
                        regs.ti_raw[i] = level;
                        self.refeed_ti(&mut regs, now);
                    }
                }
                Input::Etr => {
                    regs.etr_raw = level;
                    self.refeed_etr(&mut regs, now);
                }
                Input::Itr(i) => {
                    let prev = regs.trgi;
                    regs.itr[i] = level;
                    self.settle_trigger(&mut regs, prev);
                }
            }
            self.publish(&regs, now);
        }
        self.drive();
    }

    /// How many counter clocks until the next thing a program can observe.
    ///
    /// Never zero: the caller uses it as a step, and a step of nothing would
    /// stall the device where it stands. Between two of these instants *nothing
    /// changes*, which is what makes the closed-form advance below exact rather
    /// than merely fast.
    fn clocks_to_event(&self, regs: &Regs) -> u64 {
        let mask = u64::from(self.cfg.mask);
        let arr = u64::from(regs.arr_shadow);
        let cnt = u64::from(regs.cnt);
        let down = regs.counting_down();

        let mut best = if regs.cms() != 0 {
            // Center-aligned: up to `ARR`, then down to 0, an event at each end.
            if regs.down {
                if cnt > 0 { cnt } else { 1 }
            } else if cnt < arr {
                arr - cnt
            } else {
                1
            }
        } else if down {
            // The underflow happens on the clock after the counter reads zero.
            cnt + 1
        } else {
            // The overflow happens on the clock after the counter reads `ARR`.
            // A counter left above `ARR` — the classic "I lowered ARR under a
            // running counter" case — never matches the comparator, so it runs
            // up to the width's own wrap first and no update is generated
            // there.
            let to_uev = if cnt <= arr { arr - cnt + 1 } else { u64::MAX };
            to_uev.min(mask - cnt + 1)
        };

        for i in 0..self.cfg.channels {
            let ccr = u64::from(regs.ccr_shadow[i]);
            let distance = if down {
                if ccr < cnt { cnt - ccr } else { continue }
            } else if ccr > cnt {
                ccr - cnt
            } else {
                continue;
            };
            best = best.min(distance);
        }
        best.max(1)
    }

    /// An over/underflow has happened. Decide whether it is an update event.
    fn on_wrap(&self, regs: &mut Regs) {
        // RM0090 §16.3.1: the repetition counter divides the rate at which the
        // wrap becomes an update event. A timer without `RCR` keeps it at zero,
        // so every wrap is one.
        if regs.rcr_count > 0 {
            regs.rcr_count -= 1;
            return;
        }
        regs.rcr_count = regs.rcr;

        // `CR1.OPM`: "Counter stops counting at the next update event (clearing
        // the CEN bit)". The counter stops whether or not `UDIS` suppressed the
        // register update — what stops is the counter, not the bookkeeping.
        if regs.cr1 & CR1_OPM != 0 {
            regs.cr1 &= !CR1_CEN;
        }

        // `CR1.UDIS`: "The Update event is not generated, shadow registers keep
        // their value". The counter still wrapped; nothing else happened.
        if regs.cr1 & CR1_UDIS != 0 {
            return;
        }
        self.reload_shadows(regs);
        self.raise_uif(regs);
    }

    /// Load every preload register into its shadow. This is what an update
    /// event *is*.
    fn reload_shadows(&self, regs: &mut Regs) {
        regs.psc_shadow = regs.psc;
        regs.arr_shadow = regs.arr;
        for i in 0..MAX_CHANNELS {
            regs.ccr_shadow[i] = regs.ccr[i];
        }
    }

    /// Move the counter `clocks` counter-clocks forward, which the caller has
    /// already bounded by [`Shared::clocks_to_event`] so that at most one
    /// boundary is crossed.
    fn step(&self, regs: &mut Regs, clocks: u64) {
        let mask = u64::from(self.cfg.mask);
        let arr = u64::from(regs.arr_shadow);
        let cnt = u64::from(regs.cnt);

        if regs.cms() != 0 {
            if regs.down {
                let next = cnt.saturating_sub(clocks);
                regs.cnt = next as u32;
                if next == 0 {
                    regs.down = false;
                    self.on_wrap(regs);
                }
            } else {
                let next = cnt + clocks;
                if next >= arr {
                    // The peak is `ARR` itself: RM0090 §17.3.2 has the counter
                    // count "from 0 to ARR-1", generate the overflow, then come
                    // back down from `ARR`.
                    regs.cnt = arr as u32;
                    regs.down = true;
                    self.on_wrap(regs);
                } else {
                    regs.cnt = next as u32;
                }
            }
        } else if regs.cr1 & CR1_DIR != 0 {
            if clocks > cnt {
                self.on_wrap(regs);
                // The reload takes the shadow as it stands *after* the update,
                // which is the whole point of buffering `ARR`.
                regs.cnt = regs.arr_shadow;
            } else {
                regs.cnt = (cnt - clocks) as u32;
            }
        } else {
            let next = cnt + clocks;
            if cnt <= arr && next > arr {
                regs.cnt = 0;
                self.on_wrap(regs);
            } else {
                regs.cnt = (next & mask) as u32;
            }
        }
    }

    /// Set the capture/compare flags for any channel the counter has just
    /// landed on, and apply the match-driven output modes.
    fn settle_matches(&self, regs: &mut Regs) {
        // RM0090 §17.4.1 on `CR1.CMS`: in center-aligned mode 1 the flags are
        // set only while counting down, in mode 2 only while counting up, and
        // in mode 3 on both. Edge-aligned mode always sets them.
        let flags_allowed = match regs.cms() {
            0 | 3 => true,
            1 => regs.down,
            _ => !regs.down,
        };
        for i in 0..self.cfg.channels {
            // A channel configured as an input has no comparator in the path:
            // its `CCxIF` comes from a capture (RM0090 §17.3.5) and its `CCRx`
            // is a destination rather than a threshold.
            if regs.is_input(i) || regs.cnt != regs.ccr_shadow[i] {
                continue;
            }
            if flags_allowed {
                // "the trigger output sends a positive pulse when the CC1IF
                // flag is to be set (even if it was already high)" — the
                // compare-pulse master mode, RM0090 §17.4.2.
                if i == 0 && regs.mms() == MMS_COMPARE_PULSE {
                    regs.trgo_pulses = regs.trgo_pulses.saturating_add(1);
                }
                regs.sr |= SR_CC1IF << i;
                self.request_dma(regs, DMA_CC1 << i);
            }
            // The comparator drives the output whether or not the flag was
            // allowed: the gate above is on the interrupt flag, not on the
            // hardware.
            match regs.ocm(i) {
                OCM_ACTIVE => regs.ocref[i] = true,
                OCM_INACTIVE => regs.ocref[i] = false,
                OCM_TOGGLE => regs.ocref[i] = !regs.ocref[i],
                _ => {}
            }
        }
    }

    /// Recompute the channels whose reference output is a function of the
    /// counter rather than a latch.
    fn settle_outputs(&self, regs: &mut Regs) {
        let down = regs.counting_down();
        for i in 0..self.cfg.channels {
            if regs.is_input(i) {
                continue;
            }
            let ccr = regs.ccr_shadow[i];
            let pwm1 = if down {
                // "In downcounting mode, OCxREF is low as long as
                // CNT > CCRx, else it becomes high" — RM0090 §17.4.7.
                regs.cnt <= ccr
            } else {
                regs.cnt < ccr
            };
            regs.ocref[i] = match regs.ocm(i) {
                OCM_FORCE_INACTIVE => false,
                OCM_FORCE_ACTIVE => true,
                OCM_PWM1 => pwm1,
                OCM_PWM2 => !pwm1,
                // Frozen and the three match-driven modes hold their latch:
                // the first by definition, the others until the next match.
                OCM_FROZEN | OCM_ACTIVE | OCM_INACTIVE | OCM_TOGGLE => regs.ocref[i],
                _ => regs.ocref[i],
            };
        }
    }

    /// Advance to `target`, in ticks of this device's own clock domain.
    ///
    /// Every step is integer arithmetic on the prescaler's phase, and each
    /// iteration either lands exactly on the next observable instant or
    /// consumes the whole remaining span without reaching one. Either way the
    /// counter ends where the hardware's would.
    fn advance_locked(&self, regs: &mut Regs, target: u64) {
        let mut now = self.tick.load(Ordering::Relaxed);
        self.settle_filters(regs, now);
        while now < target {
            // A filter's hold time is an instant in its own right: an input
            // edge accepted mid-span may capture `CNT`, clock the counter or
            // reset it, so the counter may not be run past one.
            let bound = self.next_filter_deadline(regs).max(now + 1).min(target);
            now = self.run_counter(regs, now, bound);
            self.settle_filters(regs, now);
        }
        self.tick.store(now, Ordering::Relaxed);
        self.publish(regs, now);
    }

    /// Run the counter from `now` to `target`, and return `target`.
    fn run_counter(&self, regs: &mut Regs, from: u64, target: u64) -> u64 {
        let mut now = from;
        while now < target {
            if !self.counting(regs) {
                now = target;
                break;
            }
            let per = u64::from(regs.psc_shadow) + 1;
            let remaining = target - now;
            let available = (u64::from(regs.psc_count) + remaining) / per;
            if available == 0 {
                // Not even one counter clock fits: the prescaler keeps the
                // phase, which is why it is in the snapshot.
                regs.psc_count += remaining as u32;
                now = target;
                break;
            }
            let step = available.min(self.clocks_to_event(regs));
            // The first counter clock costs what is left of the current
            // division; each one after it costs a whole division.
            let used = step * per - u64::from(regs.psc_count);
            now += used;
            regs.psc_count = 0;
            self.step(regs, step);
            self.settle_matches(regs);
            self.settle_outputs(regs);
        }
        now
    }

    /// The tick of the next observable change, or [`u64::MAX`] for none.
    fn compute_next_event(&self, regs: &Regs, now: u64) -> u64 {
        // A pending filter sample is an event even when the counter is stopped
        // or clocked from a pin: accepting it can start, clock or reset the
        // counter, and nothing else will wake the device to do it.
        let filters = self.next_filter_deadline(regs).max(now + 1);
        if !self.counting(regs) {
            return filters;
        }
        let per = u64::from(regs.psc_shadow) + 1;
        let clocks = self.clocks_to_event(regs);
        // `clocks >= 1` and `psc_count < per`, so this is at least one tick in
        // the future — which is what the scheduler requires of it.
        let counter = now.saturating_add(clocks * per - u64::from(regs.psc_count));
        counter.min(filters)
    }

    fn publish(&self, regs: &Regs, now: u64) {
        self.next_event
            .store(self.compute_next_event(regs, now), Ordering::Relaxed);
    }

    // -- outputs -------------------------------------------------------------

    /// Whether the main output enable lets anything out.
    ///
    /// Only an advanced timer has `BDTR`; on the others the outputs are always
    /// enabled, so there is nothing to gate with.
    fn moe(&self, regs: &Regs) -> bool {
        !self.cfg.advanced() || regs.bdtr & BDTR_MOE != 0
    }

    /// What channel `i` idles at when `MOE` is clear: `CR2.OISx`.
    fn idle_level(&self, regs: &Regs, i: usize) -> bool {
        if !self.cfg.advanced() {
            return false;
        }
        // Bit 8 is `OIS1`, 10 `OIS2`, 12 `OIS3`, 14 `OIS4`.
        regs.cr2 & (1 << (CR2_OIS_SHIFT + 2 * i as u32)) != 0
    }

    /// What complementary output `i` idles at: `CR2.OISxN`, one bit above its
    /// channel's.
    fn idle_level_n(&self, regs: &Regs, i: usize) -> bool {
        regs.cr2 & (1 << (CR2_OIS_SHIFT + 2 * i as u32 + 1)) != 0
    }

    /// Every pin's level, as a pure function of the state.
    fn levels(&self, regs: &Regs) -> Levels {
        let mut out = Levels::default();
        let uif = regs.sr & SR_UIF != 0 && regs.dier & DIER_UIE != 0;
        let mut ccif = false;
        for i in 0..self.cfg.channels {
            if regs.sr & (SR_CC1IF << i) != 0 && regs.dier & (DIER_CC1IE << i) != 0 {
                ccif = true;
            }
        }
        let tif = regs.sr & SR_TIF != 0 && regs.dier & DIER_TIE != 0;
        out.irq_up = uif;
        out.irq_cc = ccif;
        // `TIM1_TRG_COM` on an F4; a general-purpose timer has one vector for
        // the lot, so the trigger folds into it with everything else.
        out.irq_trg = tif;
        out.irq = uif || ccif || tif;

        // `CR2.MMS`. The three pulse modes drive nothing here — a pulse is not
        // a level — and are emitted by `drive` instead.
        out.trgo = match regs.mms() {
            MMS_ENABLE => self.cnt_en(regs),
            m @ MMS_OC1REF..=0b111 => {
                let i = (m - MMS_OC1REF) as usize;
                i < self.cfg.channels && regs.ocref[i]
            }
            _ => false,
        };

        let moe = self.moe(regs);
        for i in 0..self.cfg.channels {
            let nibble = regs.ccer_nibble(i);
            out.ch[i] = if !moe {
                self.idle_level(regs, i)
            } else if nibble & CCER_CCE == 0 {
                // The pin is released to whatever else drives it; a wire has no
                // way to say that, so it reads inactive.
                false
            } else {
                regs.ocref[i] != (nibble & CCER_CCP != 0)
            };
            if i < self.cfg.complementary() {
                out.chn[i] = if !moe {
                    self.idle_level_n(regs, i)
                } else if nibble & CCER_CCNE == 0 {
                    false
                } else {
                    // The complement of `OCxREF`, then its own polarity —
                    // RM0090 §16, its complementary-output table, with a dead
                    // time of zero.
                    !regs.ocref[i] != (nibble & CCER_CCNP != 0)
                };
            }
        }
        out
    }

    /// Drive every pin, with **no lock of this device held** — the re-entrancy
    /// contract (`ROADMAP.md` §4.4): a sink may call straight back in.
    fn drive(&self) {
        let (levels, trgo_pulses, dma) = {
            let mut regs = self.regs.lock();
            let trgo = core::mem::take(&mut regs.trgo_pulses);
            let dma = core::mem::take(&mut regs.dma_pending);
            (self.levels(&regs), trgo, dma)
        };
        let links = self.links.lock().clone();
        if let Some(src) = &links.irq {
            src.set(Level::from_bool(levels.irq));
        }
        if let Some(src) = &links.irq_up {
            src.set(Level::from_bool(levels.irq_up));
        }
        if let Some(src) = &links.irq_cc {
            src.set(Level::from_bool(levels.irq_cc));
        }
        if let Some(src) = &links.irq_trg {
            src.set(Level::from_bool(levels.irq_trg));
        }
        if let Some(src) = &links.trgo {
            src.set(Level::from_bool(levels.trgo));
        }
        for i in 0..MAX_CHANNELS {
            if let Some(src) = &links.ch[i] {
                src.set(Level::from_bool(levels.ch[i]));
            }
        }
        for i in 0..COMPLEMENTARY_CHANNELS {
            if let Some(src) = &links.chn[i] {
                src.set(Level::from_bool(levels.chn[i]));
            }
        }

        if trgo_pulses != 0 || dma != 0 {
            // A pulse re-enters whatever the net is wired to, which may be a
            // slave timer that pulses in turn. The depth guard is the only
            // thing standing between a board that wires two timers into a ring
            // and an unbounded stack.
            let depth = self.depth.fetch_add(1, Ordering::Relaxed);
            if depth < MAX_TRGO_DEPTH {
                if let Some(src) = &links.trgo {
                    for _ in 0..trgo_pulses {
                        src.set(Level::High);
                        src.set(Level::from_bool(levels.trgo));
                    }
                }
                for (bit, src) in dma_lines(&links) {
                    if dma & bit != 0
                        && let Some(src) = src
                    {
                        src.set(Level::High);
                        src.set(Level::Low);
                    }
                }
            }
            self.depth.fetch_sub(1, Ordering::Relaxed);
        }
    }

    /// Catch up before answering an access.
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
        let _ = handle.sync(kind);
    }

    // -- registers -----------------------------------------------------------

    /// `EGR.UG`, or a reset arriving from the slave-mode controller if one
    /// existed.
    fn software_update(&self, regs: &mut Regs) {
        // "the shadow registers keep their value" with `UDIS` set, "however the
        // counter and the prescaler are reinitialized if the UG bit is set".
        let suppressed = regs.cr1 & CR1_UDIS != 0;
        if !suppressed {
            self.reload_shadows(regs);
            regs.rcr_count = regs.rcr;
            // `URS` selects *what may set UIF*, not what may reload: with it
            // set, only a real over/underflow raises the interrupt.
            if regs.cr1 & CR1_URS == 0 {
                self.raise_uif(regs);
            }
        }
        // "The counter is cleared if the center-aligned mode is selected or if
        // DIR=0 (upcounting), else it takes the auto-reload value."
        regs.cnt = if regs.cms() == 0 && regs.cr1 & CR1_DIR != 0 {
            regs.arr_shadow
        } else {
            0
        };
        regs.psc_count = 0;
        if regs.cms() != 0 {
            regs.down = false;
        }
        // `CR1.OPM` is deliberately not honoured here. A software update is how
        // firmware *loads* a one-pulse timer before starting it, and stopping
        // the counter on that load would stop the pulse before it began.
        self.settle_outputs(regs);
    }

    /// Read one 32-bit register. No side effects at all, so a debug read needs
    /// no special case beyond not advancing the clock.
    fn read_register(&self, regs: &Regs, offset: u64) -> u32 {
        let basic = matches!(self.cfg.variant, Variant::Basic);
        match offset {
            OFF_CR1 => regs.cr1,
            OFF_CR2 => regs.cr2,
            OFF_SMCR if !basic => regs.smcr,
            OFF_DIER => regs.dier,
            OFF_SR => regs.sr,
            // "These bits are always read as 0" — `EGR` is write-only.
            OFF_EGR => 0,
            OFF_CCMR1 if self.cfg.channels > 0 => regs.ccmr[0],
            OFF_CCMR2 if self.cfg.channels > 2 => regs.ccmr[1],
            OFF_CCER if self.cfg.channels > 0 => regs.ccer,
            OFF_CNT => {
                // `CR1.UIFREMAP`: "UIF status bit is copied to TIMx_CNT
                // register bit 31" — RM0351 §31.6.1. It is the read that is
                // remapped, not the counter.
                if regs.cr1 & CR1_UIFREMAP != 0 {
                    let uif = u32::from(regs.sr & SR_UIF != 0) << 31;
                    (regs.cnt & 0x7fff_ffff) | uif
                } else {
                    regs.cnt
                }
            }
            OFF_PSC => regs.psc,
            OFF_ARR => regs.arr,
            OFF_RCR if self.cfg.advanced() => regs.rcr,
            OFF_BDTR if self.cfg.advanced() => regs.bdtr,
            OFF_DCR if !basic => regs.dcr,
            // `DMAR` is a window onto another register; the caller resolves
            // which one, because doing so advances the burst index and a debug
            // read may not (`ROADMAP.md` §15, invariant 5).
            OFF_DMAR if !basic => 0,
            OFF_OR1 => regs.or[0],
            OFF_OR2 => regs.or[1],
            _ => {
                // The compare registers, and everything this variant does not
                // implement, which reads as zero rather than faulting: the
                // aperture is the peripheral's whole kilobyte on the real part.
                if let Some(i) = channel_of(offset)
                    && i < self.cfg.channels
                {
                    return regs.ccr[i];
                }
                0
            }
        }
    }

    /// Write one 32-bit register.
    fn write_register(&self, regs: &mut Regs, offset: u64, value: u32, now: u64) {
        let cfg = self.cfg;
        let basic = matches!(cfg.variant, Variant::Basic);
        match offset {
            OFF_CR1 => {
                let was_arpe = regs.cr1 & CR1_ARPE != 0;
                let mut wanted = value & cfg.cr1_mask();
                if regs.cms() != 0 || regs.encoder() {
                    // "DIR: ... This bit is read only when the timer is
                    // configured in Center-aligned mode or Encoder mode" —
                    // RM0090 §17.4.1.
                    wanted = (wanted & !CR1_DIR) | (regs.cr1 & CR1_DIR);
                }
                regs.cr1 = wanted;
                if was_arpe && regs.cr1 & CR1_ARPE == 0 {
                    // Preload switched off: "the new value is taken into
                    // account immediately".
                    regs.arr_shadow = regs.arr;
                }
                // `CKD` sets `t_DTS`, and every digital filter samples at a
                // division of it.
                self.refeed_ti(regs, now);
                self.refeed_etr(regs, now);
                self.settle_outputs(regs);
            }
            OFF_CR2 => {
                regs.cr2 = value & cfg.cr2_mask();
                // `TI1S` changes what feeds `TI1`'s filter.
                self.refeed_ti(regs, now);
            }
            OFF_SMCR if !basic => {
                regs.smcr = value & SMCR_MASK;
                // `ETF` may have moved, and so may the trigger selector. A
                // reconfiguration is not an edge: the selector's memory is
                // re-seeded rather than compared, so that pointing `TS` at an
                // input that happens to be high does not fire a trigger.
                self.refeed_etr(regs, now);
                regs.trgi = self.trgi_level(regs);
            }
            OFF_DIER => regs.dier = value & cfg.dier_mask(),
            // Every flag is `rc_w0`: a zero clears it and a one leaves it be.
            OFF_SR => regs.sr &= value | !cfg.sr_mask(),
            OFF_EGR => {
                if value & EGR_UG != 0 {
                    self.software_update(regs);
                    if regs.mms() == MMS_RESET {
                        // "the UG bit from the TIMx_EGR register is used as
                        // trigger output" — RM0090 §17.4.2.
                        regs.trgo_pulses = regs.trgo_pulses.saturating_add(1);
                    }
                }
                for i in 0..cfg.channels {
                    if value & (EGR_CC1G << i) != 0 {
                        regs.sr |= SR_CC1IF << i;
                        self.request_dma(regs, DMA_CC1 << i);
                    }
                }
                if value & EGR_TG != 0 && cfg.channels > 0 {
                    regs.sr |= SR_TIF;
                    self.request_dma(regs, DMA_TRG);
                }
            }
            OFF_CCMR1 | OFF_CCMR2 if cfg.channels > 0 => {
                let which = usize::from(offset == OFF_CCMR2);
                if which == 1 && cfg.channels <= 2 {
                    return;
                }
                regs.ccmr[which] = value;
                // Same rule as `ARPE`: dropping `OCxPE` pushes the written
                // compare value through at once. An input channel has no
                // preload at all, so its shadow simply follows.
                for i in 0..cfg.channels {
                    if !regs.ocpe(i) || regs.is_input(i) {
                        regs.ccr_shadow[i] = regs.ccr[i];
                    }
                }
                // `ICxF` moved, so the filters have to be re-fed.
                self.refeed_ti(regs, now);
                self.settle_outputs(regs);
            }
            OFF_CCER if cfg.channels > 0 => {
                regs.ccer = value & cfg.ccer_mask();
                for i in 0..cfg.channels {
                    // "the prescaler is reset as soon as CCxE = 0."
                    if regs.ccer_nibble(i) & CCER_CCE == 0 {
                        regs.ic_count[i] = 0;
                    }
                }
                // `CCxP` is `TIxFPx`'s polarity, and `TIxFPx` may be `TRGI`.
                regs.trgi = self.trgi_level(regs);
            }
            OFF_CNT => {
                regs.cnt = value & cfg.mask;
                self.settle_outputs(regs);
            }
            // `PSC` has no preload-enable bit: it is always buffered, and the
            // shadow follows at the next update event.
            OFF_PSC => regs.psc = value & 0xffff,
            OFF_ARR => {
                regs.arr = value & cfg.mask;
                if regs.cr1 & CR1_ARPE == 0 {
                    regs.arr_shadow = regs.arr;
                }
                self.settle_outputs(regs);
            }
            OFF_RCR if cfg.advanced() => regs.rcr = value & 0xff,
            OFF_BDTR if cfg.advanced() => regs.bdtr = value & 0xffff,
            OFF_DCR if !basic => {
                regs.dcr = value & 0x1f1f;
                // A new burst description restarts the burst.
                regs.dma_index = 0;
            }
            OFF_DMAR if !basic => {}
            OFF_OR1 => regs.or[0] = value,
            OFF_OR2 => regs.or[1] = value,
            _ => {
                if let Some(i) = channel_of(offset)
                    && i < cfg.channels
                {
                    regs.ccr[i] = value & cfg.mask;
                    if !regs.ocpe(i) {
                        regs.ccr_shadow[i] = regs.ccr[i];
                    }
                    self.settle_outputs(regs);
                }
            }
        }
    }
}

impl Shared {
    /// Which register `DMAR` currently stands in for.
    ///
    /// RM0090 §17.4.18: the access lands on `TIMx_CR1 + DBA + DMA index`, in
    /// 32-bit words. `DBA` and `DBL` are five bits each, so the furthest this
    /// reaches is `0x3e * 4`, well past the registers that exist — and a
    /// register that does not exist reads as zero, as everywhere else here.
    fn dmar_offset(&self, regs: &Regs) -> u64 {
        (u64::from(regs.dcr & 0x1f) + u64::from(regs.dma_index)) * 4
    }

    /// The same, and advance the burst index.
    ///
    /// "The DMA index is automatically incremented after each access and reset
    /// when it reaches DBL": the index wraps at `DBL + 1`, so a burst that
    /// walks `ARR`, `CCR1`, `CCR2` restarts at `ARR` on the fourth access.
    fn dmar_step(&self, regs: &mut Regs) -> u64 {
        let offset = self.dmar_offset(regs);
        let dbl = (regs.dcr >> 8) & 0x1f;
        regs.dma_index = if regs.dma_index >= dbl {
            0
        } else {
            regs.dma_index + 1
        };
        offset
    }

    /// What a **guest** read of `offset` changes, beyond answering.
    ///
    /// RM0090 §17.4.5 on `CCxIF`: "It is cleared by software by writing it to 0
    /// or by reading the captured data stored in the TIMx_CCRx register."
    /// `CCxOF` is not — that one only clears on a write of zero — and neither
    /// happens on a debug read.
    fn read_side_effects(&self, regs: &mut Regs, offset: u64) -> bool {
        let Some(i) = channel_of(offset) else {
            return false;
        };
        if i >= self.cfg.channels || !regs.is_input(i) || regs.sr & (SR_CC1IF << i) == 0 {
            return false;
        }
        regs.sr &= !(SR_CC1IF << i);
        true
    }
}

/// Each DMA request line, paired with the bit that asks for it.
fn dma_lines(links: &Links) -> [(u32, &Option<WireSource>); 2 + MAX_CHANNELS] {
    [
        (DMA_UP, &links.dma_up),
        (DMA_CC1, &links.dma_ch[0]),
        (DMA_CC1 << 1, &links.dma_ch[1]),
        (DMA_CC1 << 2, &links.dma_ch[2]),
        (DMA_CC1 << 3, &links.dma_ch[3]),
        (DMA_TRG, &links.dma_trg),
    ]
}

/// Which input pin `port` names.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Input {
    /// `ti1`–`ti4`, the channel input pins.
    Ti(usize),
    /// `etr`, the external trigger.
    Etr,
    /// `itr0`–`itr3`, the internal triggers a board wires another timer's
    /// `TRGO` to.
    Itr(usize),
}

impl Input {
    /// The line number the device knows this pin by, for [`SinkPin`].
    fn line(self) -> u32 {
        match self {
            Input::Ti(i) => i as u32,
            Input::Etr => MAX_CHANNELS as u32,
            Input::Itr(i) => (MAX_CHANNELS + 1 + i) as u32,
        }
    }
}

/// Which input pin `port` names, if any.
fn input_pin(port: &str) -> Option<Input> {
    if port == ETR_PIN {
        return Some(Input::Etr);
    }
    if let Some(digit) = port.strip_prefix("itr") {
        return match digit {
            "0" => Some(Input::Itr(0)),
            "1" => Some(Input::Itr(1)),
            "2" => Some(Input::Itr(2)),
            "3" => Some(Input::Itr(3)),
            _ => None,
        };
    }
    match port.strip_prefix("ti")? {
        "1" => Some(Input::Ti(0)),
        "2" => Some(Input::Ti(1)),
        "3" => Some(Input::Ti(2)),
        "4" => Some(Input::Ti(3)),
        _ => None,
    }
}

/// Which DMA request output pin `port` names: the bit it carries.
fn dma_pin(port: &str) -> Option<u32> {
    match port {
        DMA_UP_PIN => Some(DMA_UP),
        DMA_TRG_PIN => Some(DMA_TRG),
        "dma-ch1" => Some(DMA_CC1),
        "dma-ch2" => Some(DMA_CC1 << 1),
        "dma-ch3" => Some(DMA_CC1 << 2),
        "dma-ch4" => Some(DMA_CC1 << 3),
        _ => None,
    }
}

/// One input pin: the sink a net delivers to, and the sources it fans in.
#[derive(Debug)]
struct InputPin {
    shared: Arc<Shared>,
    which: Input,
    inputs: FanIn,
}

impl WireSink for InputPin {
    fn set_level(&self, src: WireId, _line: u32, level: Level) {
        self.inputs.set(src, level);
        let high = self.inputs.resolve(Resolve::Or).is_high();
        self.shared.input_changed(self.which, high);
    }
}

/// Which capture/compare channel `offset` names, if any.
fn channel_of(offset: u64) -> Option<usize> {
    if (OFF_CCR1..OFF_CCR1 + 4 * MAX_CHANNELS as u64).contains(&offset) {
        Some(((offset - OFF_CCR1) / 4) as usize)
    } else {
        None
    }
}

// ---------------------------------------------------------------------------
// The device
// ---------------------------------------------------------------------------

/// An STM32 TIM.
pub struct Tim {
    shared: Arc<Shared>,
    region: RegionRef,
    /// The input pins handed out so far. A net holds only a weak reference to
    /// its sinks, so somebody has to own them, and the device is the somebody
    /// (`ROADMAP.md` §4.3). No cycle: a pin holds [`Shared`], not [`Tim`].
    pins: Mutex<Vec<Arc<InputPin>>>,
}

impl fmt::Debug for Tim {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("Tim")
            .field("shared", &self.shared)
            .finish_non_exhaustive()
    }
}

impl Tim {
    /// Validate `props` and build the device.
    ///
    /// # Errors
    ///
    /// [`Error::Property`] if a property is of the wrong kind or value, if one
    /// this class does not know was given, or if the combination names no real
    /// part — a basic timer with capture/compare channels, for instance.
    pub fn new(props: &Props) -> Result<Tim> {
        let mut r = props.reader();
        let variant = match r.or_enum("variant", "general", &["basic", "general", "advanced"])? {
            "basic" => Variant::Basic,
            "advanced" => Variant::Advanced,
            _ => Variant::General,
        };
        let width = r.or::<u64>("width", 16)?;
        let channels = r.or::<u64>("channels", variant.default_channels())?;
        let arr_reset = r.or::<u64>("arr-reset", 0)?;
        r.finish()?;

        let mask: u32 = match width {
            16 => 0xffff,
            32 => 0xffff_ffff,
            _ => {
                return Err(Error::Property(format!(
                    "`width`: a TIM counter is 16 or 32 bits wide, not {width}"
                )));
            }
        };
        if channels > MAX_CHANNELS as u64 {
            return Err(Error::Property(format!(
                "`channels`: a TIM has at most {MAX_CHANNELS} channels, not {channels}"
            )));
        }
        if matches!(variant, Variant::Basic) && channels != 0 {
            return Err(Error::Property(String::from(
                "`channels`: a basic timer (TIM6/TIM7) has no capture/compare channels; \
                 a part that has them is `variant = \"general\"` or `\"advanced\"`",
            )));
        }
        if arr_reset > u64::from(mask) {
            return Err(Error::Property(format!(
                "`arr-reset`: {arr_reset:#x} does not fit a {width}-bit auto-reload"
            )));
        }

        Ok(Tim::with_config(Config {
            variant,
            mask,
            channels: channels as usize,
            arr_reset: arr_reset as u32,
        }))
    }

    /// Build one from a configuration the caller already has — the route a test
    /// takes.
    fn with_config(cfg: Config) -> Tim {
        let regs = Regs::reset(&cfg);
        let shared = Arc::new(Shared {
            cfg,
            regs: Mutex::with_rank(LockRank::DEVICE, regs),
            links: Mutex::with_rank(LockRank::WIRE, Links::default()),
            lazy: Mutex::new(None),
            tick: AtomicU64::new(0),
            next_event: AtomicU64::new(u64::MAX),
            depth: AtomicU32::new(0),
        });
        {
            let regs = shared.regs.lock();
            shared.publish(&regs, 0);
        }
        let region = Arc::new(Region::io(
            "st.tim.regs",
            REGISTER_BYTES,
            Arc::clone(&shared) as Arc<dyn MemOps>,
        ));
        Tim {
            shared,
            region,
            pins: Mutex::with_rank(LockRank::WIRE, Vec::new()),
        }
    }

    /// Which subset of the block this instance is.
    #[must_use]
    pub fn variant(&self) -> Variant {
        self.shared.cfg.variant
    }

    /// How many capture/compare channels it has.
    #[must_use]
    pub fn channels(&self) -> usize {
        self.shared.cfg.channels
    }

    /// The counter, as the guest would read `CNT`.
    #[must_use]
    pub fn counter(&self) -> u32 {
        self.shared.regs.lock().cnt
    }

    /// The tick this device's own next observable change falls on, if it has
    /// one.
    #[must_use]
    pub fn next_event(&self) -> Option<u64> {
        match self.shared.next_event.load(Ordering::Relaxed) {
            u64::MAX => None,
            tick => Some(tick),
        }
    }

    /// Advance to `target` ticks of this device's clock domain.
    pub fn advance_to(&self, target: u64) {
        {
            let mut regs = self.shared.regs.lock();
            self.shared.advance_locked(&mut regs, target);
        }
        // The re-entrancy contract: the lock is released before anything
        // outward happens (`ROADMAP.md` §4.4).
        self.shared.drive();
    }

    /// Advance by `ticks` more.
    pub fn advance_by(&self, ticks: u64) {
        self.advance_to(self.shared.tick.load(Ordering::Relaxed) + ticks);
    }

    /// Connect the catch-up handle the register block syncs through.
    pub fn attach_lazy(&self, handle: LazyHandle) {
        *self.shared.lazy.lock() = Some(handle);
    }
}

impl Device for Tim {
    fn class(&self) -> &'static DeviceClass {
        &CLASS
    }

    fn realize(&self, _ctx: &mut RealizeCtx<'_>) -> Result<()> {
        // Nothing outward: a `map` statement places the region, and every
        // output idles low, which is where a fresh net already is.
        Ok(())
    }

    fn reset(&self, _kind: ResetKind) {
        {
            let mut regs = self.shared.regs.lock();
            let raw_ti = regs.ti_raw;
            let raw_etr = regs.etr_raw;
            let itr = regs.itr;
            *regs = Regs::reset(&self.shared.cfg);
            // A reset clears the filters and the selector, but it does not
            // change what the *nets* are driving: those levels belong to
            // whatever is on the other end of the wire.
            regs.ti_raw = raw_ti;
            regs.etr_raw = raw_etr;
            regs.itr = itr;
            // The tick is the clock domain's position, not this device's state,
            // and `Machine::reset` does not rewind domains — rewinding it here
            // would ask the next catch-up to replay every cycle since power-on.
            let now = self.shared.tick.load(Ordering::Relaxed);
            self.shared.publish(&regs, now);
        }
        self.shared.drive();
    }

    fn save(&self, w: &mut ChunkWriter<'_>) -> Result<()> {
        let regs = *self.shared.regs.lock();
        for value in [
            regs.cr1,
            regs.cr2,
            regs.smcr,
            regs.dier,
            regs.sr,
            regs.ccmr[0],
            regs.ccmr[1],
            regs.ccer,
            regs.bdtr,
            regs.dcr,
            regs.dmar,
            regs.cnt,
            regs.psc_count,
            regs.psc,
            regs.psc_shadow,
            regs.arr,
            regs.arr_shadow,
            regs.rcr,
            regs.rcr_count,
            regs.or[0],
            regs.or[1],
            regs.etps_count,
            regs.dma_index,
            regs.dma_pending,
        ] {
            w.write_u32(value)?;
        }
        for i in 0..MAX_CHANNELS {
            w.write_u32(regs.ccr[i])?;
            w.write_u32(regs.ccr_shadow[i])?;
            w.write_bool(regs.ocref[i])?;
            w.write_bool(regs.ti_raw[i])?;
            w.write_bool(regs.ti_filt[i])?;
            w.write_bool(regs.ti_pend[i])?;
            w.write_u64(regs.ti_pend_at[i])?;
            w.write_u32(regs.ic_count[i])?;
        }
        for i in 0..ITR_INPUTS {
            w.write_bool(regs.itr[i])?;
        }
        w.write_bool(regs.etr_raw)?;
        w.write_bool(regs.etr_filt)?;
        w.write_bool(regs.etr_pend)?;
        w.write_u64(regs.etr_pend_at)?;
        w.write_bool(regs.trgi)?;
        w.write_u32(regs.trgo_pulses)?;
        w.write_bool(regs.down)?;
        // The domain position. The next-event tick is derived from the rest and
        // is recomputed by `load`, so it is not written (`CLAUDE.md`: derived
        // state is never serialized).
        w.write_u64(self.shared.tick.load(Ordering::Relaxed))
    }

    fn load(&self, r: &mut ChunkReader<'_>) -> Result<()> {
        let mut regs = Regs::reset(&self.shared.cfg);
        regs.cr1 = r.read_u32()?;
        regs.cr2 = r.read_u32()?;
        regs.smcr = r.read_u32()?;
        regs.dier = r.read_u32()?;
        regs.sr = r.read_u32()?;
        regs.ccmr[0] = r.read_u32()?;
        regs.ccmr[1] = r.read_u32()?;
        regs.ccer = r.read_u32()?;
        regs.bdtr = r.read_u32()?;
        regs.dcr = r.read_u32()?;
        regs.dmar = r.read_u32()?;
        regs.cnt = r.read_u32()?;
        regs.psc_count = r.read_u32()?;
        regs.psc = r.read_u32()?;
        regs.psc_shadow = r.read_u32()?;
        regs.arr = r.read_u32()?;
        regs.arr_shadow = r.read_u32()?;
        regs.rcr = r.read_u32()?;
        regs.rcr_count = r.read_u32()?;
        regs.or[0] = r.read_u32()?;
        regs.or[1] = r.read_u32()?;
        regs.etps_count = r.read_u32()?;
        regs.dma_index = r.read_u32()?;
        regs.dma_pending = r.read_u32()?;
        for i in 0..MAX_CHANNELS {
            regs.ccr[i] = r.read_u32()?;
            regs.ccr_shadow[i] = r.read_u32()?;
            regs.ocref[i] = r.read_bool()?;
            regs.ti_raw[i] = r.read_bool()?;
            regs.ti_filt[i] = r.read_bool()?;
            regs.ti_pend[i] = r.read_bool()?;
            regs.ti_pend_at[i] = r.read_u64()?;
            regs.ic_count[i] = r.read_u32()?;
        }
        for i in 0..ITR_INPUTS {
            regs.itr[i] = r.read_bool()?;
        }
        regs.etr_raw = r.read_bool()?;
        regs.etr_filt = r.read_bool()?;
        regs.etr_pend = r.read_bool()?;
        regs.etr_pend_at = r.read_u64()?;
        regs.trgi = r.read_bool()?;
        regs.trgo_pulses = r.read_u32()?;
        regs.down = r.read_bool()?;
        let tick = r.read_u64()?;
        {
            let mut slot = self.shared.regs.lock();
            *slot = regs;
            self.shared.tick.store(tick, Ordering::Relaxed);
            self.shared.publish(&regs, tick);
        }
        self.shared.drive();
        Ok(())
    }

    fn region(&self, name: &str) -> Option<RegionRef> {
        matches!(name, "" | "regs").then(|| Arc::clone(&self.region))
    }

    fn connect(&self, port: &str, source: WireSource) -> Result<()> {
        let cfg = self.shared.cfg;
        {
            let mut links = self.shared.links.lock();
            match port {
                IRQ_PIN => links.irq = Some(source),
                IRQ_UP_PIN | IRQ_CC_PIN if !cfg.advanced() => {
                    return Err(Error::Config {
                        at: port.to_string(),
                        message: format!(
                            "only an advanced timer splits its interrupt across vectors; \
                             this one drives `{IRQ_PIN}`"
                        ),
                    });
                }
                IRQ_UP_PIN => links.irq_up = Some(source),
                IRQ_CC_PIN => links.irq_cc = Some(source),
                IRQ_TRG_PIN if !cfg.advanced() => {
                    return Err(Error::Config {
                        at: port.to_string(),
                        message: format!(
                            "only an advanced timer splits its interrupt across vectors; \
                             this one drives `{IRQ_PIN}`"
                        ),
                    });
                }
                IRQ_TRG_PIN => links.irq_trg = Some(source),
                TRGO_PIN if matches!(cfg.variant, Variant::Basic) || cfg.channels > 0 => {
                    links.trgo = Some(source);
                }
                DMA_UP_PIN => links.dma_up = Some(source),
                DMA_TRG_PIN if cfg.channels > 0 => links.dma_trg = Some(source),
                _ => match (channel_pin(port), dma_pin(port)) {
                    (Some((i, false)), _) if i < cfg.channels => links.ch[i] = Some(source),
                    (Some((i, true)), _) if i < cfg.complementary() => links.chn[i] = Some(source),
                    (_, Some(bit)) if bit.trailing_zeros() as usize <= cfg.channels => {
                        links.dma_ch[bit.trailing_zeros() as usize - 1] = Some(source);
                    }
                    _ => {
                        return Err(Error::Config {
                            at: port.to_string(),
                            message: format!(
                                "this timer drives `{IRQ_PIN}`, `{TRGO_PIN}`, `{DMA_UP_PIN}` \
                                 and `ch1`…`ch{}`",
                                cfg.channels
                            ),
                        });
                    }
                },
            }
        }
        self.shared.drive();
        Ok(())
    }

    fn sink(&self, port: &str, sources: &[WireId]) -> Option<SinkPin> {
        let cfg = self.shared.cfg;
        let which = input_pin(port)?;
        // A basic timer has neither channels nor a slave controller, and an
        // instance that bonds two channels has no `TI3`.
        match which {
            Input::Ti(i) if i >= cfg.channels => return None,
            Input::Etr | Input::Itr(_) if matches!(cfg.variant, Variant::Basic) => return None,
            _ => {}
        }
        let pin = Arc::new(InputPin {
            shared: Arc::clone(&self.shared),
            which,
            inputs: FanIn::new(sources),
        });
        self.pins.lock().push(Arc::clone(&pin));
        Some(SinkPin {
            sink: pin,
            line: which.line(),
        })
    }

    fn announce(&self, _port: &str) {
        self.shared.drive();
    }

    // -- lazily advanced (`ROADMAP.md` §4.2) --------------------------------

    fn is_lazy(&self) -> bool {
        true
    }

    fn current_tick(&self) -> u64 {
        self.shared.tick.load(Ordering::Relaxed)
    }

    fn advance_to(&self, tick: u64) {
        Tim::advance_to(self, tick);
    }

    fn next_event_tick(&self) -> Option<u64> {
        Tim::next_event(self)
    }

    fn attach_lazy(&self, handle: LazyHandle) {
        Tim::attach_lazy(self, handle);
    }
}

impl MemOps for Shared {
    fn read(&self, offset: u64, dst: &mut [u8], attrs: MemAttrs) -> MemResult {
        self.sync(attrs);
        let reg = offset & !3;
        let basic = matches!(self.cfg.variant, Variant::Basic);
        let (value, changed) = {
            let mut regs = self.regs.lock();
            if reg == OFF_DMAR && !basic {
                // A debug read may look through the window without moving the
                // burst index along, which is the whole of invariant 5 here.
                let target = if attrs.debug {
                    self.dmar_offset(&regs)
                } else {
                    self.dmar_step(&mut regs)
                };
                // `DMAR` standing in for itself would recurse; the part does
                // not document the case and zero is the honest answer.
                let value = if target == OFF_DMAR {
                    0
                } else {
                    self.read_register(&regs, target)
                };
                // The window is a read of the register it stands over, so a
                // burst that walks a captured `CCRx` clears its `CCxIF` exactly
                // as a direct read would.
                let changed = !attrs.debug && self.read_side_effects(&mut regs, target);
                (value, changed)
            } else {
                let value = self.read_register(&regs, reg);
                let changed = !attrs.debug && self.read_side_effects(&mut regs, reg);
                (value, changed)
            }
        };
        if changed {
            self.drive();
        }
        match dst.len() {
            2 => {
                let half = if offset & 2 != 0 { value >> 16 } else { value };
                dst.copy_from_slice(&(half as u16).to_le_bytes());
                Ok(())
            }
            4 => {
                dst.copy_from_slice(&value.to_le_bytes());
                Ok(())
            }
            _ => Err(BusError::BadAccess),
        }
    }

    fn write(&self, offset: u64, src: &[u8], attrs: MemAttrs) -> MemResult {
        if attrs.debug {
            return Err(BusError::BadAccess);
        }
        self.sync(attrs);
        let reg = offset & !3;
        {
            let mut regs = self.regs.lock();
            let value = match src.len() {
                2 => {
                    let current = self.read_register(&regs, reg);
                    let half = u32::from(u16::from_le_bytes([src[0], src[1]]));
                    if offset & 2 != 0 {
                        (current & 0x0000_ffff) | (half << 16)
                    } else {
                        (current & 0xffff_0000) | half
                    }
                }
                4 => u32::from_le_bytes([src[0], src[1], src[2], src[3]]),
                _ => return Err(BusError::BadAccess),
            };
            let now = self.tick.load(Ordering::Relaxed);
            let target = if reg == OFF_DMAR && !matches!(self.cfg.variant, Variant::Basic) {
                let target = self.dmar_step(&mut regs);
                if target == OFF_DMAR { reg } else { target }
            } else {
                reg
            };
            self.write_register(&mut regs, target, value, now);
            self.publish(&regs, now);
        }
        self.drive();
        Ok(())
    }

    fn constraints(&self) -> AccessConstraints {
        AccessConstraints::word(Width::U32, Endian::Little).with_widths(Width::U16, Width::U32)
    }
}

/// Which channel output pin `port` names: its index, and whether it is the
/// complementary one.
fn channel_pin(port: &str) -> Option<(usize, bool)> {
    let rest = port.strip_prefix("ch")?;
    let (digits, complementary) = match rest.strip_suffix('n') {
        Some(digits) => (digits, true),
        None => (rest, false),
    };
    // `ch1`…`ch4`, one-based as the reference manual numbers them, with no
    // second spelling: `ch01` is a typo, not another name for the same net.
    let index: usize = match digits {
        "1" => 0,
        "2" => 1,
        "3" => 2,
        "4" => 3,
        _ => return None,
    };
    Some((index, complementary))
}

impl Instance for Tim {}

/// The `st.tim` device class.
pub static CLASS: DeviceClass = DeviceClass {
    name: CLASS_NAME,
    version: STATE_VERSION,
    summary: "STM32 TIM: counter, prescaler, output compare, input capture, \
              the slave-mode controller and the DMA burst window",
    properties: &[
        PropertySpec {
            name: "variant",
            kind: ValueKind::Str,
            required: false,
            summary: "which subset: \"basic\", \"general\" (default) or \"advanced\"",
        },
        PropertySpec {
            name: "width",
            kind: ValueKind::Uint,
            required: false,
            summary: "the counter's width in bits: 16 (default) or 32",
        },
        PropertySpec {
            name: "channels",
            kind: ValueKind::Uint,
            required: false,
            summary: "how many capture/compare channels, 0-4 (default 4, or 0 for \"basic\")",
        },
        PropertySpec {
            name: "arr-reset",
            kind: ValueKind::Uint,
            required: false,
            summary: "what ARR reads out of reset (default 0, which is RM0090's value)",
        },
    ],
    construct: |props| Ok(Box::new(Tim::new(props)?)),
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
    bindings.bind(CLASS_NAME, |props| Ok(Arc::new(Tim::new(props)?)))
}

/// What the validator should know about `st.tim`.
#[must_use]
pub fn schema() -> ClassSchema {
    ClassSchema::new(CLASS_NAME)
        .prop(PropSchema::new("variant", ValueKind::Str).values(&["basic", "general", "advanced"]))
        .prop(PropSchema::new("width", ValueKind::Uint).range(16, 32))
        .prop(PropSchema::new("channels", ValueKind::Uint).range(0, MAX_CHANNELS as u64))
        .prop(PropSchema::new("arr-reset", ValueKind::Uint).range(0, u64::from(u32::MAX)))
        .region("")
        .region("regs")
        // An M-profile board wires these straight to the core, with the vector
        // number coming from the part's table and living in the machine file:
        // `wire tim2.irq -> cpu.irq28`.
        .port(IRQ_PIN, PortDir::Out)
        .port(IRQ_UP_PIN, PortDir::Out)
        .port(IRQ_CC_PIN, PortDir::Out)
        .port(IRQ_TRG_PIN, PortDir::Out)
        .port("ch1", PortDir::Out)
        .port("ch2", PortDir::Out)
        .port("ch3", PortDir::Out)
        .port("ch4", PortDir::Out)
        .port("ch1n", PortDir::Out)
        .port("ch2n", PortDir::Out)
        .port("ch3n", PortDir::Out)
        // The master half of a chain: `wire tim1.trgo -> tim2.itr0`.
        .port(TRGO_PIN, PortDir::Out)
        // The request lines `st.dma` takes on `req0`–`req7`. Which stream each
        // one belongs on is RM0090 Table 43, a fact about the part, so it is
        // written in the board file and not here.
        .port(DMA_UP_PIN, PortDir::Out)
        .port("dma-ch1", PortDir::Out)
        .port("dma-ch2", PortDir::Out)
        .port("dma-ch3", PortDir::Out)
        .port("dma-ch4", PortDir::Out)
        .port(DMA_TRG_PIN, PortDir::Out)
        // The slave half, and the capture inputs.
        .port("ti1", PortDir::In)
        .port("ti2", PortDir::In)
        .port("ti3", PortDir::In)
        .port("ti4", PortDir::In)
        .port(ETR_PIN, PortDir::In)
        .port("itr0", PortDir::In)
        .port("itr1", PortDir::In)
        .port("itr2", PortDir::In)
        .port("itr3", PortDir::In)
}

#[cfg(test)]
mod tests;
