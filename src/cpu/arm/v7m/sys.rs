//! The ARMv7-M system model: exception numbers, priorities, and the
//! memory-mapped blocks at `0xE000E000`.
//!
//! This is where an ARMv7E-M core differs most from an ARMv5TE one. ARMv5
//! has seven processor modes with banked registers and an interrupt
//! controller that belongs to whatever SoC you bolted on; ARMv7-M has two
//! modes, two stack pointers, and an interrupt controller the *architecture*
//! specifies down to the register offsets. So the NVIC, the SysTick timer,
//! the System Control Block and the MPU are part of this module rather than
//! of some machine's device list.
//!
//! # The private peripheral bus is private
//!
//! `0xE0000000`–`0xE00FFFFF` is the PPB, and DDI 0403 B3.1 makes it reachable
//! only from the processor that owns it — a DMA engine cannot see another
//! core's NVIC. [`Sys`] therefore lives inside the core and the interpreter
//! routes accesses to it *before* consulting the address space, rather than
//! being a device a machine has to remember to map. A machine that maps
//! something else at `0xE000E000` will find the processor wins, which is what
//! hardware does.
//!
//! # What is modelled and what is not
//!
//! | Block | State |
//! | --- | --- |
//! | NVIC | enable / pending / active bitmaps, per-exception priority, `STIR` |
//! | SysTick | the full 24-bit reload counter, `COUNTFLAG`, the `TICKINT` interrupt |
//! | SCB | `CPUID`, `ICSR`, `VTOR`, `AIRCR`, `SCR`, `CCR`, `SHPR1-3`, `SHCSR`, `CFSR`, `HFSR`, `MMFAR`, `BFAR`, `CPACR` |
//! | MPU | eight regions, `RBAR`/`RASR` with sub-region disable and `PRIVDEFENA` |
//! | FPU | **not implemented.** `CPACR` exists so that `CP10`/`CP11` accesses raise a `NOCP` UsageFault rather than being silently ignored |
//! | DWT / ITM / FPB / TPIU | not implemented; reads return zero and writes are dropped |
//!
//! # Sources
//!
//! DDI 0403 B1.5 (the exception model), B3.2 (the System Control Space), B3.3
//! (the MPU), B3.4 (the NVIC), B3.5 (SysTick). No emulator source of any
//! licence was consulted (`ROADMAP.md` §1).

use core::fmt;

// ---------------------------------------------------------------------------
// Exception numbers
// ---------------------------------------------------------------------------

/// An exception number, as `IPSR` holds it (DDI 0403 B1.5.2).
///
/// A `#[repr(transparent)]` newtype rather than an enum: the external
/// interrupts are an open-ended range the SoC decides the size of, and `IPSR`
/// round-trips whatever is in it.
#[repr(transparent)]
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct Exception(pub u16);

impl Exception {
    /// Thread mode: not an exception at all. `IPSR` reads zero.
    pub const THREAD: Exception = Exception(0);
    /// Reset. Priority −3, the highest there is.
    pub const RESET: Exception = Exception(1);
    /// Non-maskable interrupt. Priority −2; `PRIMASK` and `BASEPRI` cannot
    /// touch it and neither can `FAULTMASK`.
    pub const NMI: Exception = Exception(2);
    /// HardFault. Priority −1, and where every escalated fault ends up.
    pub const HARD_FAULT: Exception = Exception(3);
    /// MemManage: an MPU permission or background-region violation.
    pub const MEM_MANAGE: Exception = Exception(4);
    /// BusFault: the memory system refused the access.
    pub const BUS_FAULT: Exception = Exception(5);
    /// UsageFault: undefined instruction, invalid state, divide by zero,
    /// unaligned access with `CCR.UNALIGN_TRP`, or a coprocessor that is not
    /// there.
    pub const USAGE_FAULT: Exception = Exception(6);
    /// `SVC`.
    pub const SVCALL: Exception = Exception(11);
    /// The debug monitor. Not implemented; the exception number is reserved
    /// so that `SHPR3` byte 0 has somewhere to go.
    pub const DEBUG_MONITOR: Exception = Exception(12);
    /// `PendSV`, the deferred context switch.
    pub const PEND_SV: Exception = Exception(14);
    /// The SysTick timer.
    pub const SYSTICK: Exception = Exception(15);
    /// External interrupt zero. Interrupt *n* is `Exception(16 + n)`.
    pub const IRQ0: Exception = Exception(16);

    /// How many exception numbers this core implements.
    ///
    /// Sixteen system exceptions plus 240 external interrupts, which is the
    /// most a Cortex-M4 or M7 supports. The architecture allows 496; nothing
    /// in the design depends on the number except the width of the bitmaps.
    pub const COUNT: usize = 256;

    /// The vector table offset of this exception's entry.
    #[must_use]
    pub const fn vector_offset(self) -> u32 {
        (self.0 as u32) * 4
    }

    /// Whether this is a fault whose handler can be disabled, and which
    /// therefore escalates to HardFault when it cannot be taken.
    #[must_use]
    pub const fn is_configurable_fault(self) -> bool {
        matches!(self.0, 4..=6)
    }

    /// A short name, for tracing and for a monitor's fault report.
    #[must_use]
    pub fn name(self) -> &'static str {
        match self.0 {
            0 => "thread",
            1 => "reset",
            2 => "nmi",
            3 => "hardfault",
            4 => "memmanage",
            5 => "busfault",
            6 => "usagefault",
            11 => "svcall",
            12 => "debugmon",
            14 => "pendsv",
            15 => "systick",
            16.. => "irq",
            _ => "reserved",
        }
    }
}

impl fmt::Display for Exception {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        if self.0 >= 16 {
            write!(f, "irq{}", self.0 - 16)
        } else {
            f.write_str(self.name())
        }
    }
}

// ---------------------------------------------------------------------------
// Fault status bits
// ---------------------------------------------------------------------------

/// `CFSR`, `HFSR` and their bit names (DDI 0403 B3.2.15–B3.2.16).
///
/// One module of constants rather than an enum, because the registers are
/// sticky bitmaps a handler ORs together and a debugger prints whole.
pub mod fsr {
    /// `MMFSR.IACCVIOL` — an instruction fetch violated the MPU.
    pub const MM_IACCVIOL: u32 = 1 << 0;
    /// `MMFSR.DACCVIOL` — a data access violated the MPU.
    pub const MM_DACCVIOL: u32 = 1 << 1;
    /// `MMFSR.MUNSTKERR` — the exception return's unstacking faulted.
    pub const MM_MUNSTKERR: u32 = 1 << 3;
    /// `MMFSR.MSTKERR` — exception entry's stacking faulted.
    pub const MM_MSTKERR: u32 = 1 << 4;
    /// `MMFSR.MMARVALID` — `MMFAR` holds the faulting address.
    pub const MM_MMARVALID: u32 = 1 << 7;

    /// `BFSR.IBUSERR` — an instruction fetch was refused.
    pub const BF_IBUSERR: u32 = 1 << 8;
    /// `BFSR.PRECISERR` — a data access was refused, and `BFAR` says where.
    pub const BF_PRECISERR: u32 = 1 << 9;
    /// `BFSR.IMPRECISERR` — a buffered write was refused later.
    pub const BF_IMPRECISERR: u32 = 1 << 10;
    /// `BFSR.UNSTKERR` — the exception return's unstacking was refused.
    pub const BF_UNSTKERR: u32 = 1 << 11;
    /// `BFSR.STKERR` — exception entry's stacking was refused.
    pub const BF_STKERR: u32 = 1 << 12;
    /// `BFSR.BFARVALID` — `BFAR` holds the faulting address.
    pub const BF_BFARVALID: u32 = 1 << 15;

    /// `UFSR.UNDEFINSTR` — an encoding this architecture does not define.
    pub const UF_UNDEFINSTR: u32 = 1 << 16;
    /// `UFSR.INVSTATE` — an attempt to enter ARM state, or an `EPSR.T` of
    /// zero.
    pub const UF_INVSTATE: u32 = 1 << 17;
    /// `UFSR.INVPC` — an illegal `EXC_RETURN` or exception-return context.
    pub const UF_INVPC: u32 = 1 << 18;
    /// `UFSR.NOCP` — a coprocessor instruction with no coprocessor.
    pub const UF_NOCP: u32 = 1 << 19;
    /// `UFSR.UNALIGNED` — an unaligned access with `CCR.UNALIGN_TRP` set.
    pub const UF_UNALIGNED: u32 = 1 << 24;
    /// `UFSR.DIVBYZERO` — `SDIV`/`UDIV` by zero with `CCR.DIV_0_TRP` set.
    pub const UF_DIVBYZERO: u32 = 1 << 25;

    /// `HFSR.VECTTBL` — the vector fetch itself faulted.
    pub const HF_VECTTBL: u32 = 1 << 1;
    /// `HFSR.FORCED` — a configurable fault escalated to here.
    pub const HF_FORCED: u32 = 1 << 30;
    /// `HFSR.DEBUGEVT` — a debug event with the monitor disabled.
    pub const HF_DEBUGEVT: u32 = 1 << 31;
}

/// `CCR` bit names (DDI 0403 B3.2.8).
pub mod ccr {
    /// Allow a return to Thread mode with exceptions still active.
    pub const NONBASETHRDENA: u32 = 1 << 0;
    /// Let unprivileged code write `STIR`.
    pub const USERSETMPEND: u32 = 1 << 1;
    /// Trap unaligned accesses instead of performing them.
    pub const UNALIGN_TRP: u32 = 1 << 3;
    /// Trap division by zero instead of returning zero.
    pub const DIV_0_TRP: u32 = 1 << 4;
    /// Ignore data bus faults inside HardFault, NMI and `FAULTMASK` handlers.
    pub const BFHFNMIGN: u32 = 1 << 8;
    /// Force eight-byte stack alignment on exception entry.
    pub const STKALIGN: u32 = 1 << 9;
}

/// `SHCSR` bit names (DDI 0403 B3.2.13).
pub mod shcsr {
    /// MemManage is enabled.
    pub const MEMFAULTENA: u32 = 1 << 16;
    /// BusFault is enabled.
    pub const BUSFAULTENA: u32 = 1 << 17;
    /// UsageFault is enabled.
    pub const USGFAULTENA: u32 = 1 << 18;
}

/// `CONTROL` bit names (DDI 0403 B1.4.4).
pub mod control {
    /// Thread mode is unprivileged.
    pub const NPRIV: u32 = 1 << 0;
    /// Thread mode uses the process stack.
    pub const SPSEL: u32 = 1 << 1;
    /// Floating-point context is active. Never set: there is no FPU.
    pub const FPCA: u32 = 1 << 2;
}

// ---------------------------------------------------------------------------
// EXC_RETURN
// ---------------------------------------------------------------------------

/// The `EXC_RETURN` values an exception entry can put in `LR`
/// (DDI 0403 B1.5.8).
pub mod exc_return {
    /// Return to Handler mode, using the main stack.
    pub const HANDLER_MSP: u32 = 0xffff_fff1;
    /// Return to Thread mode, using the main stack.
    pub const THREAD_MSP: u32 = 0xffff_fff9;
    /// Return to Thread mode, using the process stack.
    pub const THREAD_PSP: u32 = 0xffff_fffd;

    /// Whether a `PC` value is an `EXC_RETURN` rather than an address.
    ///
    /// The architecture reserves the whole of `0xF0000000`–`0xFFFFFFFF` for
    /// this, and a branch to any of it from Handler mode is an exception
    /// return; only the three values above (plus their floating-point
    /// variants) are legal, and the rest raise `UFSR.INVPC`.
    #[must_use]
    pub const fn is_magic(value: u32) -> bool {
        value & 0xf000_0000 == 0xf000_0000
    }
}

// ---------------------------------------------------------------------------
// The system block
// ---------------------------------------------------------------------------

/// How wide the exception bitmaps are, in `u32` words.
const WORDS: usize = Exception::COUNT / 32;

/// The default `CPUID` value: an ARM-designed Cortex-M4, revision r0p1.
///
/// Implementer `0x41` ("A" for ARM), variant 0, architecture `0xF` (ARMv7-M),
/// part number `0xC24` (Cortex-M4), revision 1. Firmware reads this to
/// discover what it is running on, so it has to be a real part number rather
/// than zero; the number is a published fact from the Cortex-M4 TRM.
pub const CPUID_CORTEX_M4: u32 = 0x410f_c241;

/// The default `CPUID` value for a Cortex-M7 r1p0: part number `0xC27`.
pub const CPUID_CORTEX_M7: u32 = 0x411f_c271;

/// Everything behind `0xE000E000`, plus the exception bookkeeping the NVIC
/// and the SCB share.
#[derive(Debug, Clone)]
pub struct Sys {
    /// One bit per exception: the handler may be taken.
    ///
    /// System exceptions that cannot be disabled read as enabled here so that
    /// the arbiter has one uniform test.
    pub enable: [u32; WORDS],
    /// One bit per exception: the handler is waiting to run.
    pub pending: [u32; WORDS],
    /// One bit per exception: the handler is running, or was preempted while
    /// running.
    pub active: [u32; WORDS],
    /// Eight-bit priority per exception. Ignored for exceptions 1–3, whose
    /// priorities are architecturally fixed and negative.
    pub priority: [u8; Exception::COUNT],
    /// How many of the priority bits are implemented, counted from the top.
    ///
    /// Writes to the rest are dropped and they read as zero, which is how
    /// firmware discovers the number.
    pub priority_bits: u8,

    /// `VTOR`: where the vector table is.
    pub vtor: u32,
    /// `AIRCR.PRIGROUP`: how many low priority bits are sub-priority and so
    /// do not participate in preemption.
    pub prigroup: u8,
    /// `SCR`.
    pub scr: u32,
    /// `CCR`.
    pub ccr: u32,
    /// `SHCSR`'s writable enable bits. The active/pending bits are derived
    /// from [`Sys::active`] and [`Sys::pending`] rather than stored twice.
    pub shcsr: u32,
    /// `CFSR`: `MMFSR`, `BFSR` and `UFSR` in one word.
    pub cfsr: u32,
    /// `HFSR`.
    pub hfsr: u32,
    /// `MMFAR`.
    pub mmfar: u32,
    /// `BFAR`.
    pub bfar: u32,
    /// `AFSR`. Nothing here sets it; it exists so a read does not fault.
    pub afsr: u32,
    /// `CPACR`. Zero at reset, which is what makes `CP10`/`CP11` raise
    /// `NOCP`.
    pub cpacr: u32,
    /// `CPUID`.
    pub cpuid: u32,
    /// A `SYSRESETREQ` was written to `AIRCR`. The machine, not the core,
    /// decides what a system reset does.
    pub reset_requested: bool,

    /// `SYST_CSR`.
    pub syst_csr: u32,
    /// `SYST_RVR`, 24 bits.
    pub syst_rvr: u32,
    /// `SYST_CVR`, 24 bits.
    pub syst_cvr: u32,
    /// `SYST_CALIB`. `NOREF` and `SKEW` set, `TENMS` zero: the reference
    /// clock is not modelled and calibration is unknown, which is exactly
    /// what those bits mean.
    pub syst_calib: u32,

    /// `MPU_CTRL`.
    pub mpu_ctrl: u32,
    /// `MPU_RNR`.
    pub mpu_rnr: u32,
    /// `MPU_RBAR` for each of the eight regions.
    pub mpu_rbar: [u32; MPU_REGIONS],
    /// `MPU_RASR` for each of the eight regions.
    pub mpu_rasr: [u32; MPU_REGIONS],
    /// How many MPU regions this instance has: [`MPU_REGIONS`], or zero for
    /// a part built without the option.
    ///
    /// `MPU_TYPE.DREGION` reads this, which is how firmware discovers there
    /// is no MPU; with it zero the registers are read-as-zero, write-ignored
    /// and every access is permitted.
    pub mpu_regions: u8,
}

/// How many MPU regions this core has. Eight is the Cortex-M4 and M7 default.
pub const MPU_REGIONS: usize = 8;

impl Default for Sys {
    fn default() -> Sys {
        Sys::new(CPUID_CORTEX_M4, 8, MPU_REGIONS as u8)
    }
}

impl Sys {
    /// The reset state of the whole block (DDI 0403 B3.2.2's reset column).
    #[must_use]
    pub fn new(cpuid: u32, priority_bits: u8, mpu_regions: u8) -> Sys {
        let mut sys = Sys {
            enable: [0; WORDS],
            pending: [0; WORDS],
            active: [0; WORDS],
            priority: [0; Exception::COUNT],
            priority_bits: priority_bits.clamp(1, 8),
            vtor: 0,
            prigroup: 0,
            scr: 0,
            // `STKALIGN` reads as one and is RAO/WI on a Cortex-M4 and M7:
            // eight-byte stack alignment on exception entry is not optional
            // on those parts (Cortex-M4 TRM, "Configuration and Control
            // Register").
            ccr: ccr::STKALIGN,
            shcsr: 0,
            cfsr: 0,
            hfsr: 0,
            mmfar: 0,
            bfar: 0,
            afsr: 0,
            cpacr: 0,
            cpuid,
            reset_requested: false,
            syst_csr: 0,
            syst_rvr: 0,
            syst_cvr: 0,
            // NOREF | SKEW.
            syst_calib: (1 << 31) | (1 << 30),
            mpu_ctrl: 0,
            mpu_rnr: 0,
            mpu_rbar: [0; MPU_REGIONS],
            mpu_rasr: [0; MPU_REGIONS],
            mpu_regions: mpu_regions.min(MPU_REGIONS as u8),
        };
        // The exceptions with no enable bit are permanently enabled, so the
        // arbiter never has to special-case them.
        for n in [1u16, 2, 3, 11, 14, 15] {
            sys.set_enable(Exception(n), true);
        }
        sys
    }

    /// Read one exception's bit out of a bitmap.
    #[inline]
    fn get(map: &[u32; WORDS], e: Exception) -> bool {
        let n = e.0 as usize;
        n < Exception::COUNT && map[n / 32] & (1 << (n % 32)) != 0
    }

    /// Write one exception's bit into a bitmap.
    #[inline]
    fn set(map: &mut [u32; WORDS], e: Exception, on: bool) {
        let n = e.0 as usize;
        if n >= Exception::COUNT {
            return;
        }
        if on {
            map[n / 32] |= 1 << (n % 32);
        } else {
            map[n / 32] &= !(1 << (n % 32));
        }
    }

    /// Whether the exception's handler may be taken.
    #[must_use]
    pub fn is_enabled(&self, e: Exception) -> bool {
        match e.0 {
            4 => self.shcsr & shcsr::MEMFAULTENA != 0,
            5 => self.shcsr & shcsr::BUSFAULTENA != 0,
            6 => self.shcsr & shcsr::USGFAULTENA != 0,
            _ => Sys::get(&self.enable, e),
        }
    }

    /// Enable or disable an exception.
    pub fn set_enable(&mut self, e: Exception, on: bool) {
        Sys::set(&mut self.enable, e, on);
    }

    /// Whether the exception is waiting to run.
    #[must_use]
    pub fn is_pending(&self, e: Exception) -> bool {
        Sys::get(&self.pending, e)
    }

    /// Make an exception pending, or take that back.
    pub fn set_pending(&mut self, e: Exception, on: bool) {
        Sys::set(&mut self.pending, e, on);
    }

    /// Whether the exception's handler is on the stack.
    #[must_use]
    pub fn is_active(&self, e: Exception) -> bool {
        Sys::get(&self.active, e)
    }

    /// Mark an exception's handler active or finished.
    pub fn set_active(&mut self, e: Exception, on: bool) {
        Sys::set(&mut self.active, e, on);
    }

    /// Whether any exception at all is active.
    #[must_use]
    pub fn any_active(&self) -> bool {
        self.active.iter().any(|w| *w != 0)
    }

    /// How many exceptions are active. `ICSR.RETTOBASE` is "exactly one".
    #[must_use]
    pub fn active_count(&self) -> u32 {
        self.active.iter().map(|w| w.count_ones()).sum()
    }

    /// The mask that drops a priority's sub-priority bits.
    ///
    /// `PRIGROUP` names the *last* sub-priority bit, so a `PRIGROUP` of *n*
    /// makes bits `[n:0]` sub-priority and bits `[7:n+1]` group priority
    /// (DDI 0403 B1.5.4). A `PRIGROUP` of seven leaves one group bit.
    #[must_use]
    pub const fn group_mask(&self) -> u8 {
        let sub_bits = (self.prigroup as u32) + 1;
        if sub_bits >= 8 { 0 } else { (!0u8) << sub_bits }
    }

    /// An exception's priority, as a signed value where lower wins.
    ///
    /// Reset, NMI and HardFault are architecturally −3, −2 and −1 and have no
    /// priority register. Everything else is its eight-bit priority with the
    /// sub-priority bits masked off, because preemption compares group
    /// priorities only.
    #[must_use]
    pub fn priority_of(&self, e: Exception) -> i32 {
        match e.0 {
            1 => -3,
            2 => -2,
            3 => -1,
            _ => i32::from(
                self.priority[(e.0 as usize) & (Exception::COUNT - 1)] & self.group_mask(),
            ),
        }
    }

    /// The priority a write to a priority register actually stores.
    ///
    /// Only the top [`Sys::priority_bits`] bits are implemented; the rest
    /// read as zero, which is how CMSIS discovers the number at run time.
    #[must_use]
    pub const fn quantize_priority(&self, value: u8) -> u8 {
        let drop = 8 - self.priority_bits;
        if drop >= 8 {
            0
        } else {
            (value >> drop) << drop
        }
    }

    /// The highest-priority pending, enabled exception, if any.
    ///
    /// Ties go to the lowest exception number, which is the architecture's
    /// rule and the reason NMI beats HardFault at the same nominal priority
    /// (DDI 0403 B1.5.4).
    #[must_use]
    pub fn highest_pending(&self) -> Option<(Exception, i32)> {
        let mut best: Option<(Exception, i32)> = None;
        for (word_index, word) in self.pending.iter().enumerate() {
            let mut bits = *word;
            while bits != 0 {
                let bit = bits.trailing_zeros();
                bits &= bits - 1;
                let e = Exception((word_index * 32 + bit as usize) as u16);
                if !self.is_enabled(e) {
                    continue;
                }
                let p = self.priority_of(e);
                match best {
                    Some((_, bp)) if bp <= p => {}
                    _ => best = Some((e, p)),
                }
            }
        }
        best
    }

    /// The lowest priority among the active exceptions, which is the priority
    /// the processor is currently executing at before the masks are applied.
    #[must_use]
    pub fn active_priority(&self) -> i32 {
        let mut prio = 256;
        for (word_index, word) in self.active.iter().enumerate() {
            let mut bits = *word;
            while bits != 0 {
                let bit = bits.trailing_zeros();
                bits &= bits - 1;
                let e = Exception((word_index * 32 + bit as usize) as u16);
                let p = self.priority_of(e);
                if p < prio {
                    prio = p;
                }
            }
        }
        prio
    }

    /// The `ICSR.VECTPENDING` field: the pending exception that would be
    /// taken next, or zero.
    #[must_use]
    pub fn vect_pending(&self) -> u32 {
        self.highest_pending().map_or(0, |(e, _)| u32::from(e.0))
    }
}

// ---------------------------------------------------------------------------
// The register map
// ---------------------------------------------------------------------------

/// Base of the private peripheral bus.
pub const PPB_BASE: u32 = 0xe000_0000;
/// One byte past the private peripheral bus.
pub const PPB_END: u32 = 0xe010_0000;

/// Whether an address is inside the private peripheral bus.
#[inline]
#[must_use]
pub const fn in_ppb(addr: u32) -> bool {
    addr >= PPB_BASE && addr < PPB_END
}

impl Sys {
    /// Read a word from the private peripheral bus.
    ///
    /// `debug` suppresses the side effects a read otherwise has —
    /// `SYST_CSR.COUNTFLAG` clears when it is read, and a debugger's
    /// register window must not be what clears it (`ROADMAP.md` §15,
    /// invariant 5).
    ///
    /// Returns `None` for an address this core does not implement, which the
    /// caller turns into a bus fault. Everything the *architecture* defines
    /// but this core does not implement — DWT, ITM, FPB, TPIU — reads as
    /// zero instead, because firmware probes those and a fault would be a
    /// worse answer than "not present".
    #[must_use]
    #[allow(clippy::too_many_lines)] // One arm per register; splitting hides the map.
    pub fn read_word(&mut self, addr: u32, debug: bool) -> Option<u32> {
        let word = addr & !3;
        Some(match word {
            // SysTick.
            0xe000_e010 => {
                let v = self.syst_csr;
                if !debug {
                    // COUNTFLAG reads as one once and then clears.
                    self.syst_csr &= !(1 << 16);
                }
                v
            }
            0xe000_e014 => self.syst_rvr,
            0xe000_e018 => self.syst_cvr,
            0xe000_e01c => self.syst_calib,

            // NVIC. Every one of these five blocks is eight words wide and
            // indexed by exception number minus sixteen.
            0xe000_e100..=0xe000_e11c => self.irq_word(&self.enable, word - 0xe000_e100),
            0xe000_e180..=0xe000_e19c => self.irq_word(&self.enable, word - 0xe000_e180),
            0xe000_e200..=0xe000_e21c => self.irq_word(&self.pending, word - 0xe000_e200),
            0xe000_e280..=0xe000_e29c => self.irq_word(&self.pending, word - 0xe000_e280),
            0xe000_e300..=0xe000_e31c => self.irq_word(&self.active, word - 0xe000_e300),
            0xe000_e400..=0xe000_e4ec => {
                let first = 16 + (word - 0xe000_e400);
                let mut v = 0;
                for k in 0..4 {
                    let n = (first + k) as usize;
                    if n < Exception::COUNT {
                        v |= u32::from(self.priority[n]) << (8 * k);
                    }
                }
                v
            }

            // SCB.
            0xe000_ed00 => self.cpuid,
            0xe000_ed04 => self.icsr(),
            0xe000_ed08 => self.vtor,
            0xe000_ed0c => 0xfa05_0000 | (u32::from(self.prigroup) << 8),
            0xe000_ed10 => self.scr,
            0xe000_ed14 => self.ccr,
            0xe000_ed18..=0xe000_ed20 => {
                let first = 4 + (word - 0xe000_ed18);
                let mut v = 0;
                for k in 0..4 {
                    v |= u32::from(self.priority[(first + k) as usize]) << (8 * k);
                }
                v
            }
            0xe000_ed24 => self.shcsr_read(),
            0xe000_ed28 => self.cfsr,
            0xe000_ed2c => self.hfsr,
            // DFSR: no debug unit, so nothing ever sets it.
            0xe000_ed30 => 0,
            0xe000_ed34 => self.mmfar,
            0xe000_ed38 => self.bfar,
            0xe000_ed3c => self.afsr,
            0xe000_ed88 => self.cpacr,

            // MPU.
            0xe000_ed90 => u32::from(self.mpu_regions) << 8,
            0xe000_ed94 => self.mpu_ctrl,
            0xe000_ed98 => self.mpu_rnr,
            0xe000_ed9c | 0xe000_eda4 | 0xe000_edac | 0xe000_edb4 => {
                let n = (self.mpu_rnr as usize) & (MPU_REGIONS - 1);
                self.mpu_rbar[n]
            }
            0xe000_eda0 | 0xe000_eda8 | 0xe000_edb0 | 0xe000_edb8 => {
                let n = (self.mpu_rnr as usize) & (MPU_REGIONS - 1);
                self.mpu_rasr[n]
            }

            // STIR is write-only; a read returns zero rather than faulting.
            0xe000_ef00 => 0,

            // Everything else inside the PPB: the ID and feature registers,
            // and the trace and debug blocks this core does not implement.
            _ if in_ppb(word) => 0,
            _ => return None,
        })
    }

    /// Write a word to the private peripheral bus.
    ///
    /// Returns `false` for an address this core does not implement.
    #[allow(clippy::too_many_lines)] // One arm per register; splitting hides the map.
    pub fn write_word(&mut self, addr: u32, value: u32) -> bool {
        let word = addr & !3;
        match word {
            0xe000_e010 => {
                // COUNTFLAG is read-only.
                self.syst_csr = (self.syst_csr & (1 << 16)) | (value & 0x7);
            }
            0xe000_e014 => self.syst_rvr = value & 0x00ff_ffff,
            0xe000_e018 => {
                // A write of any value clears the counter *and* COUNTFLAG.
                self.syst_cvr = 0;
                self.syst_csr &= !(1 << 16);
            }
            0xe000_e01c => {}

            0xe000_e100..=0xe000_e11c => self.irq_set(Bitmap::Enable, word - 0xe000_e100, value),
            0xe000_e180..=0xe000_e19c => self.irq_clear(Bitmap::Enable, word - 0xe000_e180, value),
            0xe000_e200..=0xe000_e21c => self.irq_set(Bitmap::Pending, word - 0xe000_e200, value),
            0xe000_e280..=0xe000_e29c => self.irq_clear(Bitmap::Pending, word - 0xe000_e280, value),
            // IABR is read-only.
            0xe000_e300..=0xe000_e31c => {}
            0xe000_e400..=0xe000_e4ec => {
                let first = 16 + (word - 0xe000_e400);
                for k in 0..4 {
                    let n = (first + k) as usize;
                    if n < Exception::COUNT {
                        self.priority[n] = self.quantize_priority((value >> (8 * k)) as u8);
                    }
                }
            }

            0xe000_ed00 => {}
            0xe000_ed04 => {
                // ICSR's writable bits are the four pend/clear controls.
                if value & (1 << 31) != 0 {
                    self.set_pending(Exception::NMI, true);
                }
                if value & (1 << 28) != 0 {
                    self.set_pending(Exception::PEND_SV, true);
                }
                if value & (1 << 27) != 0 {
                    self.set_pending(Exception::PEND_SV, false);
                }
                if value & (1 << 26) != 0 {
                    self.set_pending(Exception::SYSTICK, true);
                }
                if value & (1 << 25) != 0 {
                    self.set_pending(Exception::SYSTICK, false);
                }
            }
            // The vector table is at least 32 words and must be aligned to
            // its own size rounded up to a power of two; the architecture
            // makes bits [6:0] read as zero.
            0xe000_ed08 => self.vtor = value & 0xffff_ff80,
            0xe000_ed0c => {
                // Every write needs the key in the top half, or it is
                // ignored entirely (DDI 0403 B3.2.6).
                if value >> 16 != 0x05fa {
                    return true;
                }
                self.prigroup = ((value >> 8) & 7) as u8;
                if value & (1 << 2) != 0 {
                    self.reset_requested = true;
                }
                if value & (1 << 1) != 0 {
                    // VECTCLRACTIVE: clear every active bit. Deprecated, and
                    // only meaningful to a debugger, but cheap to honour.
                    self.active = [0; WORDS];
                }
            }
            0xe000_ed10 => self.scr = value & 0x1e,
            // STKALIGN is RAO/WI on this part; the rest are writable.
            0xe000_ed14 => self.ccr = (value & 0x0000_031b) | ccr::STKALIGN,
            0xe000_ed18..=0xe000_ed20 => {
                let first = 4 + (word - 0xe000_ed18);
                for k in 0..4 {
                    let n = (first + k) as usize;
                    self.priority[n] = self.quantize_priority((value >> (8 * k)) as u8);
                }
            }
            0xe000_ed24 => {
                self.shcsr = value & (shcsr::MEMFAULTENA | shcsr::BUSFAULTENA | shcsr::USGFAULTENA);
                // The pended bits are writable too, and are how a debugger
                // injects a fault.
                self.set_pending(Exception::USAGE_FAULT, value & (1 << 12) != 0);
                self.set_pending(Exception::MEM_MANAGE, value & (1 << 13) != 0);
                self.set_pending(Exception::BUS_FAULT, value & (1 << 14) != 0);
                self.set_pending(Exception::SVCALL, value & (1 << 15) != 0);
            }
            // The fault status registers are write-one-to-clear.
            0xe000_ed28 => self.cfsr &= !value,
            0xe000_ed2c => self.hfsr &= !value,
            0xe000_ed30 => {}
            0xe000_ed34 => self.mmfar = value,
            0xe000_ed38 => self.bfar = value,
            0xe000_ed3c => self.afsr = value,
            0xe000_ed88 => self.cpacr = value & 0x00f0_0000,

            0xe000_ed90 => {}
            0xe000_ed94..=0xe000_edb8 if self.mpu_regions == 0 => {}
            0xe000_ed94 => self.mpu_ctrl = value & 0x7,
            0xe000_ed98 => self.mpu_rnr = value & 0xff,
            0xe000_ed9c | 0xe000_eda4 | 0xe000_edac | 0xe000_edb4 => {
                // `VALID` makes the write select a region as well as set it,
                // which is what lets firmware program eight regions without
                // touching RNR between them.
                let n = if value & (1 << 4) != 0 {
                    let n = (value & 0xf) as usize & (MPU_REGIONS - 1);
                    self.mpu_rnr = n as u32;
                    n
                } else {
                    (self.mpu_rnr as usize) & (MPU_REGIONS - 1)
                };
                self.mpu_rbar[n] = (value & !0x1f) | (n as u32);
            }
            0xe000_eda0 | 0xe000_eda8 | 0xe000_edb0 | 0xe000_edb8 => {
                let n = (self.mpu_rnr as usize) & (MPU_REGIONS - 1);
                self.mpu_rasr[n] = value;
            }

            0xe000_ef00 => {
                let n = (value & 0x1ff) as u16;
                if usize::from(n) + 16 < Exception::COUNT {
                    self.set_pending(Exception(n + 16), true);
                }
            }

            _ if in_ppb(word) => {}
            _ => return false,
        }
        true
    }

    /// One word of an NVIC bitmap, shifted so bit zero is external interrupt
    /// `32 * index`.
    fn irq_word(&self, map: &[u32; WORDS], offset: u32) -> u32 {
        let first = 16 + (offset / 4) * 32;
        let mut v = 0u32;
        for k in 0..32u32 {
            let n = (first + k) as usize;
            if n < Exception::COUNT && map[n / 32] & (1 << (n % 32)) != 0 {
                v |= 1 << k;
            }
        }
        v
    }

    fn irq_set(&mut self, which: Bitmap, offset: u32, value: u32) {
        self.irq_rmw(which, offset, value, true);
    }

    fn irq_clear(&mut self, which: Bitmap, offset: u32, value: u32) {
        self.irq_rmw(which, offset, value, false);
    }

    fn irq_rmw(&mut self, which: Bitmap, offset: u32, value: u32, on: bool) {
        let first = 16 + (offset / 4) * 32;
        for k in 0..32u32 {
            if value & (1 << k) == 0 {
                continue;
            }
            let e = Exception((first + k) as u16);
            if usize::from(e.0) >= Exception::COUNT {
                continue;
            }
            match which {
                Bitmap::Enable => Sys::set(&mut self.enable, e, on),
                Bitmap::Pending => Sys::set(&mut self.pending, e, on),
            }
        }
    }

    /// `ICSR`, assembled from the bitmaps rather than stored.
    fn icsr(&self) -> u32 {
        let mut v = 0u32;
        v |= self.vect_pending() << 12;
        if self.pending.iter().enumerate().any(|(i, w)| {
            // ISRPENDING reports *external* interrupts only.
            if i == 0 { *w & !0xffff != 0 } else { *w != 0 }
        }) {
            v |= 1 << 22;
        }
        if self.active_count() <= 1 {
            v |= 1 << 11;
        }
        if self.is_pending(Exception::PEND_SV) {
            v |= 1 << 28;
        }
        if self.is_pending(Exception::SYSTICK) {
            v |= 1 << 26;
        }
        if self.is_pending(Exception::NMI) {
            v |= 1 << 31;
        }
        v
    }

    /// `SHCSR`, with the active and pending bits filled in from the bitmaps.
    fn shcsr_read(&self) -> u32 {
        let mut v = self.shcsr;
        let act = |e: u16, bit: u32, v: &mut u32| {
            if self.is_active(Exception(e)) {
                *v |= 1 << bit;
            }
        };
        act(4, 0, &mut v);
        act(5, 1, &mut v);
        act(6, 3, &mut v);
        act(11, 7, &mut v);
        act(12, 8, &mut v);
        act(14, 10, &mut v);
        act(15, 11, &mut v);
        let pend = |e: u16, bit: u32, v: &mut u32| {
            if self.is_pending(Exception(e)) {
                *v |= 1 << bit;
            }
        };
        pend(6, 12, &mut v);
        pend(4, 13, &mut v);
        pend(5, 14, &mut v);
        pend(11, 15, &mut v);
        v
    }

    /// Advance SysTick by `ticks` processor clocks, returning whether it
    /// wrapped.
    ///
    /// The scheduler owns time (`ROADMAP.md` §4.2), and SysTick counts the
    /// *processor* clock, so the core drives it from the cycles it charged
    /// rather than from anything resembling a wall clock. `CLKSOURCE` selects
    /// an external reference on real parts; with no reference modelled, both
    /// settings count the same clock and `SYST_CALIB.NOREF` says so.
    pub fn tick_systick(&mut self, ticks: u64) -> bool {
        if self.syst_csr & 1 == 0 || self.syst_rvr == 0 {
            return false;
        }
        let mut wrapped = false;
        let mut left = ticks;
        while left > 0 {
            let cur = self.syst_cvr & 0x00ff_ffff;
            if cur == 0 {
                // A zero counter reloads on the next clock rather than
                // counting through zero again (DDI 0403 B3.3.1).
                self.syst_cvr = self.syst_rvr & 0x00ff_ffff;
                left -= 1;
                continue;
            }
            let step = left.min(u64::from(cur));
            let next = cur - (step as u32);
            self.syst_cvr = next;
            left -= step;
            if next == 0 {
                wrapped = true;
                self.syst_csr |= 1 << 16;
            }
        }
        wrapped
    }
}

/// Which NVIC bitmap a set/clear register pair addresses.
#[derive(Debug, Clone, Copy)]
enum Bitmap {
    Enable,
    Pending,
}

// ---------------------------------------------------------------------------
// The MPU
// ---------------------------------------------------------------------------

/// What an access is trying to do, for the MPU and for fault reporting.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Access {
    /// An instruction fetch.
    Fetch,
    /// A data read.
    Read,
    /// A data write.
    Write,
}

impl Sys {
    /// Whether the MPU permits `access` at `addr`.
    ///
    /// The rules, in the order the architecture applies them
    /// (DDI 0403 B3.5.3): a disabled MPU permits everything; otherwise the
    /// *highest-numbered* enabled region containing the address decides, and
    /// if none contains it the access succeeds only for privileged code with
    /// `PRIVDEFENA` set.
    ///
    /// `HFNMIENA` clear disables the MPU entirely while the execution
    /// priority is negative, which is what keeps a fault handler able to run
    /// when the MPU configuration is what broke.
    #[must_use]
    pub fn mpu_permits(&self, addr: u32, access: Access, privileged: bool, priority: i32) -> bool {
        if self.mpu_regions == 0 || self.mpu_ctrl & 1 == 0 {
            return true;
        }
        if priority < 0 && self.mpu_ctrl & 0b10 == 0 {
            return true;
        }
        let mut decision = None;
        for n in 0..MPU_REGIONS {
            let rasr = self.mpu_rasr[n];
            if rasr & 1 == 0 {
                continue;
            }
            // SIZE names the region's size as a power of two minus one, so
            // the smallest region the architecture allows is 32 bytes.
            let size_field = (rasr >> 1) & 0x1f;
            if size_field < 4 {
                continue;
            }
            let bits = size_field + 1;
            let size = if bits >= 32 { 0u32 } else { 1u32 << bits };
            let mask = if bits >= 32 { 0u32 } else { !(size - 1) };
            let base = self.mpu_rbar[n] & mask;
            if bits < 32 && (addr & mask) != base {
                continue;
            }
            // Sub-region disable, for regions of 256 bytes and up. Each
            // eighth of the region can be switched out.
            if bits >= 8 {
                let sub = if bits >= 32 {
                    // A whole-address-space region's eighths are 512 MiB.
                    (addr >> 29) & 7
                } else {
                    ((addr - base) >> (bits - 3)) & 7
                };
                if (rasr >> 8) & (1 << sub) != 0 {
                    continue;
                }
            }
            decision = Some(rasr);
        }
        let Some(rasr) = decision else {
            // The background region: the default memory map, available to
            // privileged code when PRIVDEFENA says so.
            return privileged && self.mpu_ctrl & 0b100 != 0;
        };
        if access == Access::Fetch && rasr & (1 << 28) != 0 {
            return false;
        }
        let ap = (rasr >> 24) & 7;
        match (ap, privileged, access) {
            (0b000, _, _) => false,
            (0b001, true, _) => true,
            (0b001, false, _) => false,
            (0b010, true, _) => true,
            (0b010, false, Access::Write) => false,
            (0b010, false, _) => true,
            (0b011, _, _) => true,
            (0b101, true, Access::Write) => false,
            (0b101, true, _) => true,
            (0b101, false, _) => false,
            (0b110 | 0b111, _, Access::Write) => false,
            (0b110 | 0b111, _, _) => true,
            _ => false,
        }
    }
}
