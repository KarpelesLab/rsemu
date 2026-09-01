//! Executing the x87 escapes and the SSE instructions.
//!
//! The arithmetic is [`crate::float`]'s and the registers are [`super::fpu`]'s;
//! what is here is the *instruction* layer between them — operand plumbing,
//! the stack discipline, the condition codes, and the three exception gates.
//!
//! # The three gates, in the order hardware applies them
//!
//! | | x87 | SSE |
//! | --- | --- | --- |
//! | the feature is absent | `#UD` | `#UD` |
//! | `CR0.EM` set | `#NM` — software emulation is the point | `#UD` |
//! | `CR4.OSFXSR` clear | — | `#UD` |
//! | `CR0.TS` set | `#NM` | `#NM` |
//! | an exception is pending | `#MF` *before* the instruction | — |
//!
//! The asymmetry in the `CR0.EM` row is the one worth stating: `CR0.EM` exists
//! so an operating system can emulate an absent 387 in an `#NM` handler, and
//! there has never been a software-emulation protocol for SSE, so setting it
//! makes an SSE instruction invalid rather than trappable (*Intel SDM* volume 1
//! §11.5.1, and volume 3 table 2-2).
//!
//! # What an unmasked exception does
//!
//! It does *not* write the destination. Both units record the exception and
//! leave the result where it was, so the handler sees the operands
//! (§8.5.1 and §11.5.3). x87 then defers: `ES` and `B` go up and the fault
//! waits for the next floating-point instruction. SSE raises `#XM` on the
//! spot, and only if `CR4.OSXMMEXCPT` says the operating system installed a
//! handler for it — otherwise it is `#UD`, which is Intel's way of making a
//! kernel that forgot to set the bit fail loudly rather than at random.
//!
//! # What is deliberately not here
//!
//! * **The transcendentals** — `F2XM1`, `FYL2X`, `FYL2XP1`, `FPTAN`, `FPATAN`,
//!   `FSIN`, `FCOS`, `FSINCOS`. Computing them to the last bit of an 80-bit
//!   significand without a host `f64` is a subproject, and an approximation
//!   would be a silently wrong answer rather than a missing one. Their
//!   encodings are unassigned in [`super::isa`], so they raise `#UD` and a
//!   guest finds out immediately.
//! * **`FBLD`/`FBSTP`**, the packed-decimal pair, and **`FISTTP`**, which
//!   belongs to SSE3.
//! * **`C1`'s "the result was rounded up" meaning.** `float`'s kernel rounds
//!   once and reports the exceptions, not the direction it moved, so `C1` is
//!   cleared after an arithmetic instruction rather than set from the
//!   rounding. Its other meaning — the direction of a stack fault — *is*
//!   modelled, because that one is load-bearing.
//! * **The real-address-mode `FNSTENV` image.** The four layouts differ only
//!   in how the instruction and data pointers are packed; the control, status
//!   and tag words are at the same offsets in all four, and those are what
//!   software reads. The protected-mode packing is written in all cases.
//!
//! # Sources
//!
//! *Intel SDM* volume 1 chapters 8, 10 and 11; volume 2 for every instruction
//! reference and for `FXSAVE`'s memory image; volume 3 §2.5 and table 2-2 for
//! the control-register gating. No copyleft emulator or soft-float library was
//! consulted (`CLAUDE.md`, provenance).

use crate::float::x87::{self as f80, F80, Precision};
use crate::float::{B32, B64, Category, Env, Flags, Round, binary};

use super::exec::{Ex, Exec, Fault, VEC_MF, VEC_NM, VEC_UD, VEC_XM};
use super::flags;
use super::fpu::{cw, mxcsr, sw};
use super::isa::{Arg, Fields, Op};
use super::prot::{cr0, cr4};

/// The seven constants `FLD1` and its six siblings push.
///
/// These are the correctly-rounded 80-bit encodings of the values themselves,
/// which are facts about the numbers rather than anyone's table: `pi/2 * 2^63`
/// rounded to sixty-four bits is `0xc90f_daa2_2168_c235` however it is
/// computed. *Intel SDM* volume 1 §8.3.5 lists them.
///
/// One rounding subtlety is **not** modelled and is recorded here rather than
/// hidden: real hardware rounds these constants according to `RC`, because it
/// keeps them to more than 64 bits internally. This pushes the
/// round-to-nearest encoding whatever `RC` says.
mod konst {
    use crate::float::x87::F80;

    /// `+1.0`.
    pub(super) const ONE: F80 = F80::new(0x3fff, 0x8000_0000_0000_0000);
    /// `log2(10)`.
    pub(super) const L2T: F80 = F80::new(0x4000, 0xd49a_784b_cd1b_8afe);
    /// `log2(e)`.
    pub(super) const L2E: F80 = F80::new(0x3fff, 0xb8aa_3b29_5c17_f0bc);
    /// `pi`.
    pub(super) const PI: F80 = F80::new(0x4000, 0xc90f_daa2_2168_c235);
    /// `log10(2)`.
    pub(super) const LG2: F80 = F80::new(0x3ffd, 0x9a20_9a84_fbcf_f799);
    /// `ln(2)`.
    pub(super) const LN2: F80 = F80::new(0x3ffe, 0xb172_17f7_d1cf_79ac);
}

impl Exec<'_> {
    // -----------------------------------------------------------------
    // Gating
    // -----------------------------------------------------------------

    /// Whether this instance has an x87 unit at all.
    #[inline]
    fn has_x87(&self) -> bool {
        self.cfg.features.fpu
    }

    /// The three x87 gates, in hardware's order.
    ///
    /// `CR0.EM` and `CR0.TS` both divert to `#NM` and mean opposite things:
    /// the first says there is no unit and software will emulate it, the
    /// second says the unit's state belongs to another task and has to be
    /// swapped in first. The processor does not distinguish them and neither
    /// does this — the handler reads `CR0` and finds out.
    fn x87_gate(&mut self, op: Op) -> Ex<()> {
        // A part with no unit never reaches here: `Exec::execute` sends the
        // whole escape range down the coprocessor path before the operation
        // is consulted, because that is what an 80386 with no 80387 does.
        debug_assert!(self.has_x87());
        if self.state.sys.cr0 & (cr0::EM | cr0::TS) != 0 {
            return Err(Fault::bare(VEC_NM));
        }
        // The deferred `#MF`. The six "no-wait" forms skip it deliberately:
        // they are the instructions an `#MF` handler runs, and taking the
        // exception again inside its own handler is an infinite regress.
        if !op.x87_no_wait() && self.x87_pending_fault() {
            return Err(Fault::bare(VEC_MF));
        }
        Ok(())
    }

    /// Whether a pending unmasked x87 exception may be delivered as `#MF`.
    ///
    /// Only with `CR0.NE` set. With it clear the 387's `FERR#` pin is the
    /// route, and a PC's chipset turns that into IRQ 13 — no such pin is
    /// modelled here, so the exception simply stays pending rather than being
    /// delivered down a path that does not exist. Every guest that unmasks an
    /// exception sets `NE` first; a guest that does not has asked for the
    /// external route and gets nothing instead of getting it wrong.
    pub(super) fn x87_pending_fault(&self) -> bool {
        self.state.x87.pending() && self.state.sys.cr0 & cr0::NE != 0
    }

    /// The SSE gates. See this module's table.
    fn sse_gate(&mut self, op: Op) -> Ex<()> {
        let features = self.cfg.features;
        match op {
            // `FXSAVE` and `FXRSTOR` are gated by `FXSR` alone: they are how
            // an operating system saves the state, so they have to work
            // before it has committed to using SSE.
            Op::FXSAVE | Op::FXRSTOR => {
                if !features.fxsr {
                    return Err(Fault::bare(VEC_UD));
                }
                if self.state.sys.cr0 & cr0::TS != 0 {
                    return Err(Fault::bare(VEC_NM));
                }
                return Ok(());
            }
            // The fences are not floating-point instructions and are gated by
            // nothing: `SFENCE` came with SSE and the other two with SSE2, and
            // on a core with one bus each is already satisfied.
            Op::SFENCE | Op::LFENCE | Op::MFENCE => {
                if !features.sse {
                    return Err(Fault::bare(VEC_UD));
                }
                return Ok(());
            }
            _ => {}
        }
        let needs_sse2 = matches!(
            op,
            Op::ADDPD
                | Op::ADDSD
                | Op::ANDNPD
                | Op::ANDPD
                | Op::CMPPD
                | Op::CMPSD
                | Op::COMISD
                | Op::CVTPD2PS
                | Op::CVTPS2PD
                | Op::CVTSD2SI
                | Op::CVTSD2SS
                | Op::CVTSI2SD
                | Op::CVTSS2SD
                | Op::CVTTSD2SI
                | Op::DIVPD
                | Op::DIVSD
                | Op::MAXPD
                | Op::MAXSD
                | Op::MINPD
                | Op::MINSD
                | Op::MOVAPD
                | Op::MOVD
                | Op::MOVDQA
                | Op::MOVDQU
                | Op::MOVHPD
                | Op::MOVLPD
                | Op::MOVMSKPD
                | Op::MOVQ
                | Op::MOVSD
                | Op::MOVUPD
                | Op::MULPD
                | Op::MULSD
                | Op::ORPD
                | Op::PAND
                | Op::PANDN
                | Op::POR
                | Op::PXOR
                | Op::SHUFPD
                | Op::SQRTPD
                | Op::SQRTSD
                | Op::SUBPD
                | Op::SUBSD
                | Op::UCOMISD
                | Op::UNPCKHPD
                | Op::UNPCKLPD
                | Op::XORPD
        );
        if !features.sse || (needs_sse2 && !features.sse2) {
            return Err(Fault::bare(VEC_UD));
        }
        // `CR0.EM` is `#UD` here rather than `#NM`: there is no
        // software-emulation protocol for SSE, so a kernel that set the bit
        // meant "no SIMD", not "call me".
        if self.state.sys.cr0 & cr0::EM != 0 || self.state.sys.cr4 & cr4::OSFXSR == 0 {
            return Err(Fault::bare(VEC_UD));
        }
        if self.state.sys.cr0 & cr0::TS != 0 {
            return Err(Fault::bare(VEC_NM));
        }
        Ok(())
    }

    /// Record the SIMD exceptions an operation raised, and fault if any of
    /// them is unmasked.
    ///
    /// Returns `Ok(())` when the instruction may go on to write its
    /// destination, and otherwise propagates — an unmasked exception leaves
    /// the destination exactly as the handler needs to see it, and the `?` at
    /// every call site is what implements that.
    fn sse_raise(&mut self, flags: Flags) -> Ex<()> {
        if !self.state.sse.raise(flags) {
            return Ok(());
        }
        if self.state.sys.cr4 & cr4::OSXMMEXCPT == 0 {
            // The operating system unmasked an exception and gave the
            // processor nowhere to deliver it. `#UD` is what the architecture
            // says, and it is the loud failure a silent one would hide.
            return Err(Fault::bare(VEC_UD));
        }
        Err(Fault::bare(VEC_XM))
    }

    // -----------------------------------------------------------------
    // Memory
    // -----------------------------------------------------------------

    /// Read the ten bytes of an `m80fp`.
    fn read_f80(&mut self, sr: u8, off: u64) -> Ex<F80> {
        let lo = self.read_mem(sr, off, 8)?;
        let hi = self.read_mem(sr, off.wrapping_add(8), 2)? as u16;
        Ok(F80::new(hi, lo))
    }

    /// Write the ten bytes of an `m80fp`.
    fn write_f80(&mut self, sr: u8, off: u64, value: F80) -> Ex<()> {
        self.write_mem(sr, off, 8, value.sig)?;
        self.write_mem(sr, off.wrapping_add(8), 2, u64::from(value.sign_exp))
    }

    /// Read a 128-bit operand as `[low, high]`, two bus transfers.
    fn read_xmm_mem(&mut self, sr: u8, off: u64) -> Ex<[u64; 2]> {
        let lo = self.read_mem(sr, off, 8)?;
        let hi = self.read_mem(sr, off.wrapping_add(8), 8)?;
        Ok([lo, hi])
    }

    /// Write a 128-bit operand.
    fn write_xmm_mem(&mut self, sr: u8, off: u64, value: [u64; 2]) -> Ex<()> {
        self.write_mem(sr, off, 8, value[0])?;
        self.write_mem(sr, off.wrapping_add(8), 8, value[1])
    }

    /// Refuse a memory operand that is not sixteen-byte aligned.
    ///
    /// The `MOVAPS`/`MOVAPD`/`MOVDQA` family is defined to raise `#GP(0)` on a
    /// misaligned address rather than to do the access slowly, and software
    /// leans on it: the unaligned form exists precisely so that the aligned
    /// one can be a checked assertion (*Intel SDM* volume 2, `MOVAPS`).
    fn require_aligned(&self, off: u64) -> Ex<()> {
        if off.is_multiple_of(16) {
            Ok(())
        } else {
            Err(Fault::gp(0))
        }
    }

    // -----------------------------------------------------------------
    // x87
    // -----------------------------------------------------------------

    /// Execute one x87 instruction.
    pub(super) fn x87_instruction(&mut self, f: &Fields) -> Ex<()> {
        let op = f.insn.op;
        self.x87_gate(op)?;
        // The instruction pointer, data pointer and opcode the environment
        // reports. **All three** are frozen across the control instructions,
        // and it has to be all three: a handler's first act is `FNSTENV`, and
        // if that updated the data pointer to its own save area then the `FDP`
        // field the handler then reads would name the save area instead of the
        // operand that faulted — which is the entire purpose of the field.
        //
        // The list is the nine control instructions (*Intel SDM* volume 1
        // §8.1.8); everything else, `FDECSTP` and `FFREE` included, updates.
        if !op.x87_freezes_pointers() {
            let x = &mut self.state.x87;
            x.last_ip = self.start_ip;
            x.last_cs = self.entry.cs;
            // Eleven bits: the low three of the escape byte and all eight of
            // the ModRM byte.
            let modrm = f
                .modrm
                .map_or(0u16, |m| u16::from((m.md << 6) | (m.reg << 3) | m.rm));
            x.last_op = (u16::from(f.opcode & 7) << 8) | modrm;
            if let Some((sr, off)) = self.ea {
                x.last_dp = off;
                x.last_ds = self.entry.segment(sr);
            }
        }
        match op {
            Op::FLD => self.x87_load(f),
            Op::FILD => self.x87_load_int(f),
            Op::FLD1 => self.x87_push_checked(konst::ONE),
            Op::FLDZ => self.x87_push_checked(F80::ZERO),
            Op::FLDL2T => self.x87_push_checked(konst::L2T),
            Op::FLDL2E => self.x87_push_checked(konst::L2E),
            Op::FLDPI => self.x87_push_checked(konst::PI),
            Op::FLDLG2 => self.x87_push_checked(konst::LG2),
            Op::FLDLN2 => self.x87_push_checked(konst::LN2),
            Op::FST | Op::FSTP => self.x87_store(f, op == Op::FSTP),
            Op::FIST | Op::FISTP => self.x87_store_int(f, op == Op::FISTP),
            Op::FXCH => self.x87_exchange(f),
            Op::FFREE => {
                self.state.x87.free(Self::sti(f));
                Ok(())
            }
            Op::FINCSTP => {
                self.state.x87.inc_top();
                self.state.x87.status &= !sw::C1;
                Ok(())
            }
            Op::FDECSTP => {
                self.state.x87.dec_top();
                self.state.x87.status &= !sw::C1;
                Ok(())
            }
            Op::FNOP => Ok(()),
            Op::FCHS | Op::FABS => self.x87_sign(op == Op::FABS),
            Op::FADD | Op::FSUB | Op::FSUBR | Op::FMUL | Op::FDIV | Op::FDIVR => {
                self.x87_arith(f, op, false)
            }
            Op::FADDP | Op::FSUBP | Op::FSUBRP | Op::FMULP | Op::FDIVP | Op::FDIVRP => {
                self.x87_arith(f, Self::unpopped(op), true)
            }
            Op::FIADD | Op::FISUB | Op::FISUBR | Op::FIMUL | Op::FIDIV | Op::FIDIVR => {
                self.x87_arith_int(f, op)
            }
            Op::FSQRT => self.x87_unary(f80::sqrt),
            // `FRNDINT` discards the precision control, and `FSCALE` below
            // does too. That is not an omission: *Intel SDM* volume 1 §8.1.5.2
            // gives a **closed** list of the instructions `PC` affects — the
            // add, subtract, multiply and divide families and `FSQRT` — and
            // neither of these is on it. Rounding to an integer is not a
            // precision-control operation to begin with.
            Op::FRNDINT => self.x87_unary(|v, _pc, env| f80::round_to_integral(v, env)),
            Op::FSCALE => self.x87_scale(),
            Op::FXTRACT => self.x87_extract(),
            Op::FPREM | Op::FPREM1 => self.x87_remainder(op == Op::FPREM1),
            Op::FCOM | Op::FCOMP => self.x87_compare(f, true, if op == Op::FCOMP { 1 } else { 0 }),
            Op::FCOMPP => self.x87_compare(f, true, 2),
            Op::FUCOM | Op::FUCOMP => {
                self.x87_compare(f, false, if op == Op::FUCOMP { 1 } else { 0 })
            }
            Op::FUCOMPP => self.x87_compare(f, false, 2),
            Op::FICOM | Op::FICOMP => self.x87_compare_int(f, op == Op::FICOMP),
            Op::FTST => self.x87_test(),
            Op::FXAM => {
                self.x87_examine();
                Ok(())
            }
            Op::FCOMI | Op::FCOMIP => self.x87_compare_flags(f, true, op == Op::FCOMIP),
            Op::FUCOMI | Op::FUCOMIP => self.x87_compare_flags(f, false, op == Op::FUCOMIP),
            Op::FCMOVB
            | Op::FCMOVE
            | Op::FCMOVBE
            | Op::FCMOVU
            | Op::FCMOVNB
            | Op::FCMOVNE
            | Op::FCMOVNBE
            | Op::FCMOVNU => self.x87_cmov(f, op),
            Op::FNINIT => {
                self.state.x87.init();
                Ok(())
            }
            Op::FNCLEX => {
                self.state.x87.clear_exceptions();
                Ok(())
            }
            Op::FLDCW => {
                let value = self.read_arg(f, Arg::Ew, 2)? as u16;
                self.state.x87.control = value;
                // Loading a control word can unmask an exception that is
                // already flagged, and the summary bit has to catch up — this
                // is how an `FLDCW` becomes the instruction *after* which the
                // pending `#MF` fires.
                self.x87_resummarise();
                Ok(())
            }
            Op::FNSTCW => {
                let value = u64::from(self.state.x87.control);
                self.write_arg(f, Arg::Ew, 2, value)
            }
            Op::FNSTSW => {
                let value = u64::from(self.state.x87.status);
                if f.modrm.is_some_and(super::isa::ModRm::is_register) {
                    // `DF E0` is `FNSTSW AX` and nothing else: the `r/m` field
                    // is part of the opcode rather than an operand, so a `REX`
                    // prefix cannot redirect it to `R8W` the way it would for
                    // a real `Ew`.
                    self.state.regs.set_word(0, value as u16);
                    return Ok(());
                }
                self.write_arg(f, Arg::Ew, 2, value)
            }
            Op::FNSTENV => self.x87_store_env(f),
            Op::FLDENV => self.x87_load_env(f),
            Op::FNSAVE => self.x87_save(f),
            Op::FRSTOR => self.x87_restore(f),
            // Every operation `Op::is_x87` reports is handled above; a new one
            // reaching here is a table that grew without its implementation.
            _ => Err(Fault::bare(VEC_UD)),
        }
    }

    /// The non-popping form of a popping arithmetic operation.
    const fn unpopped(op: Op) -> Op {
        match op {
            Op::FADDP => Op::FADD,
            Op::FSUBP => Op::FSUB,
            Op::FSUBRP => Op::FSUBR,
            Op::FMULP => Op::FMUL,
            Op::FDIVP => Op::FDIV,
            _ => Op::FDIVR,
        }
    }

    /// The stack index the ModRM `r/m` field names.
    fn sti(f: &Fields) -> u8 {
        f.modrm.map_or(0, |m| m.rm)
    }

    /// Recompute `ES` and `B` from the flags and the masks.
    ///
    /// `ES` is treated as a **combinational** function of the six flags and
    /// the six masks rather than as a latch — §8.1.3 defines it as "set when
    /// any unmasked exception bits are set", present tense — and that is what
    /// makes the rest of the model hang together. It is why `FNSTENV`, whose
    /// documented side effect is to mask everything, takes the unit out of the
    /// faulting state without a separate clear, so a handler can execute
    /// floating-point instructions without immediately re-faulting; and why an
    /// `FLDCW` that unmasks an already-flagged exception makes the *next*
    /// instruction take `#MF`. A latched `ES` would need both of those written
    /// as special cases, and would leave `FLDENV` able to restore a summary
    /// bit that disagrees with the flags beside it.
    fn x87_resummarise(&mut self) {
        let x = &mut self.state.x87;
        let live = x.status & sw::EXCEPTIONS & x.unmasked();
        if live != 0 {
            x.status |= sw::ES | sw::B;
        } else {
            x.status &= !(sw::ES | sw::B);
        }
    }

    /// The environment and precision the control word selects.
    fn x87_env(&self) -> (Env, Precision) {
        (self.state.x87.env(), self.state.x87.precision())
    }

    /// Push a value, taking a stack overflow if `ST(7)` is occupied.
    ///
    /// The register that becomes the new `ST(0)` is the current `ST(7)`, so
    /// that — not `ST(0)` — is the one whose tag decides.
    fn x87_push_checked(&mut self, value: F80) -> Ex<()> {
        if self.state.x87.occupied(7) {
            // §8.5.1.1: `IE` and `SF` with `C1` set, and the masked response
            // still moves `TOP` and writes the indefinite. That is why a
            // program that overflows once keeps producing indefinites.
            if self.state.x87.stack_fault(true) {
                return Ok(());
            }
            self.state.x87.dec_top();
            self.state.x87.set(0, F80::INDEFINITE);
            return Ok(());
        }
        self.state.x87.status &= !sw::C1;
        self.state.x87.dec_top();
        self.state.x87.set(0, value);
        Ok(())
    }

    /// Read `ST(i)`, or record a stack underflow and return `None`.
    ///
    /// The second element says whether the caller may still write a result:
    /// an unmasked `IE` leaves the destination alone.
    fn x87_st(&mut self, i: u8) -> (Option<F80>, bool) {
        if self.state.x87.occupied(i) {
            return (Some(self.state.x87.raw(i)), true);
        }
        let unmasked = self.state.x87.stack_fault(false);
        (None, !unmasked)
    }

    /// `FLD`: push a value from memory or from another stack register.
    fn x87_load(&mut self, f: &Fields) -> Ex<()> {
        let (env, _) = self.x87_env();
        let (value, flags) = match f.insn.src {
            Arg::Sti => {
                let (v, may_write) = self.x87_st(Self::sti(f));
                let Some(v) = v else {
                    if may_write {
                        return self.x87_push_indefinite();
                    }
                    return Ok(());
                };
                (v, Flags::NONE)
            }
            Arg::Mf32 => {
                let (sr, off) = self.ea();
                let bits = self.read_mem(sr, off, 4)?;
                f80::from_binary::<B32>(bits, env)
            }
            Arg::Mf64 => {
                let (sr, off) = self.ea();
                let bits = self.read_mem(sr, off, 8)?;
                f80::from_binary::<B64>(bits, env)
            }
            _ => {
                // `m80fp` is the register format, so the ten bytes are moved
                // without conversion — an unsupported encoding is loaded as
                // it stands and only complained about when something tries to
                // compute with it.
                let (sr, off) = self.ea();
                let v = self.read_f80(sr, off)?;
                (v, Flags::NONE)
            }
        };
        if self.state.x87.raise(flags) {
            return Ok(());
        }
        self.x87_push_checked(value)
    }

    /// The masked response to reading an empty register on a push.
    fn x87_push_indefinite(&mut self) -> Ex<()> {
        // The overflow check still applies: pushing an indefinite onto a full
        // stack is a second fault, and the first one has already been
        // recorded.
        if self.state.x87.occupied(7) && self.state.x87.stack_fault(true) {
            return Ok(());
        }
        self.state.x87.dec_top();
        self.state.x87.set(0, F80::INDEFINITE);
        Ok(())
    }

    /// `FILD`: push an integer from memory, converted exactly.
    fn x87_load_int(&mut self, f: &Fields) -> Ex<()> {
        let (env, _) = self.x87_env();
        let bits = Self::int_bits(f.insn.src);
        let (sr, off) = self.ea();
        let raw = self.read_mem(sr, off, bits / 8)?;
        let (value, flags) = f80::from_signed(raw as i64, u32::from(bits), env);
        if self.state.x87.raise(flags) {
            return Ok(());
        }
        self.x87_push_checked(value)
    }

    /// How many bits an integer memory operand has.
    const fn int_bits(arg: Arg) -> u8 {
        match arg {
            Arg::Mi16 => 16,
            Arg::Mi64 => 64,
            _ => 32,
        }
    }

    /// `FST` and `FSTP`.
    fn x87_store(&mut self, f: &Fields, pop: bool) -> Ex<()> {
        let (env, _) = self.x87_env();
        // A stack underflow does not skip the store: §8.5.1.1's masked
        // response is to write the QNaN indefinite to the destination —
        // memory included — and carry on, which is what lets a program that
        // has lost track of the stack still produce a recognisable value
        // rather than leaving whatever was there.
        let value = match self.x87_st(0) {
            (Some(value), _) => value,
            (None, false) => return Ok(()),
            (None, true) => F80::INDEFINITE,
        };
        let flags = match f.insn.dst {
            Arg::Sti => {
                self.state.x87.set(Self::sti(f), value);
                Flags::NONE
            }
            Arg::Mf32 => {
                let (bits, fl) = f80::to_binary::<B32>(value, env);
                if self.state.x87.raise(fl) {
                    return Ok(());
                }
                let (sr, off) = self.ea();
                self.write_mem(sr, off, 4, bits)?;
                Flags::NONE
            }
            Arg::Mf64 => {
                let (bits, fl) = f80::to_binary::<B64>(value, env);
                if self.state.x87.raise(fl) {
                    return Ok(());
                }
                let (sr, off) = self.ea();
                self.write_mem(sr, off, 8, bits)?;
                Flags::NONE
            }
            _ => {
                // `m80fp` again: ten bytes out, unchanged.
                let (sr, off) = self.ea();
                self.write_f80(sr, off, value)?;
                Flags::NONE
            }
        };
        if self.state.x87.raise(flags) {
            return Ok(());
        }
        if pop {
            self.state.x87.pop();
        }
        Ok(())
    }

    /// `FIST` and `FISTP`.
    fn x87_store_int(&mut self, f: &Fields, pop: bool) -> Ex<()> {
        let (env, _) = self.x87_env();
        let bits = Self::int_bits(f.insn.dst);
        let value = match self.x87_st(0) {
            (Some(value), _) => value,
            (None, false) => return Ok(()),
            // The indefinite converts to the *integer* indefinite, which is
            // the masked response the manual gives for both faults at once.
            (None, true) => F80::INDEFINITE,
        };
        let (int, fl) = f80::to_signed(value, u32::from(bits), env);
        if self.state.x87.raise(fl) {
            return Ok(());
        }
        let (sr, off) = self.ea();
        self.write_mem(sr, off, bits / 8, int as u64)?;
        if pop {
            self.state.x87.pop();
        }
        Ok(())
    }

    /// `FXCH`: swap `ST(0)` with `ST(i)`, tags and all.
    fn x87_exchange(&mut self, f: &Fields) -> Ex<()> {
        let i = Self::sti(f);
        if !self.state.x87.occupied(0) || !self.state.x87.occupied(i) {
            // §8.5.1.1's masked response: fill whichever registers are empty
            // with the indefinite and exchange them anyway.
            if self.state.x87.stack_fault(false) {
                return Ok(());
            }
            if !self.state.x87.occupied(0) {
                self.state.x87.set(0, F80::INDEFINITE);
            }
            if !self.state.x87.occupied(i) {
                self.state.x87.set(i, F80::INDEFINITE);
            }
        } else {
            self.state.x87.status &= !sw::C1;
        }
        let a = self.state.x87.raw(0);
        let b = self.state.x87.raw(i);
        self.state.x87.set(0, b);
        self.state.x87.set(i, a);
        Ok(())
    }

    /// `FCHS` and `FABS`, which move one bit and nothing else.
    fn x87_sign(&mut self, absolute: bool) -> Ex<()> {
        let (value, may_write) = self.x87_st(0);
        let Some(value) = value else {
            if may_write {
                self.state.x87.set(0, F80::INDEFINITE);
            }
            return Ok(());
        };
        // An unsupported encoding is not a value, so flipping its sign would
        // be inventing one: the indefinite is the masked answer.
        if matches!(f80::classify(value), f80::X87Class::Unsupported) {
            if self.state.x87.raise(Flags::INVALID) {
                return Ok(());
            }
            self.state.x87.set(0, F80::INDEFINITE);
            return Ok(());
        }
        let sign_exp = if absolute {
            value.sign_exp & 0x7fff
        } else {
            value.sign_exp ^ 0x8000
        };
        self.state.x87.status &= !sw::C1;
        self.state.x87.set(0, F80::new(sign_exp, value.sig));
        Ok(())
    }

    /// The two-operand arithmetic, in all four of its operand shapes.
    fn x87_arith(&mut self, f: &Fields, op: Op, pop: bool) -> Ex<()> {
        let (env, pc) = self.x87_env();
        // `dst` names where the result goes and `src` where the other operand
        // comes from; `DC` and `DE` swap the two round, which is the whole
        // difference between `FSUB` and its reversed form at those opcodes.
        let dst_index = match f.insn.dst {
            Arg::Sti => Self::sti(f),
            _ => 0,
        };
        let (a, may_write) = self.x87_st(dst_index);
        let b = match f.insn.src {
            Arg::St0 => {
                let (v, w) = self.x87_st(0);
                (v, w)
            }
            Arg::Sti => {
                let (v, w) = self.x87_st(Self::sti(f));
                (v, w)
            }
            Arg::Mf32 => {
                let (sr, off) = self.ea();
                let bits = self.read_mem(sr, off, 4)?;
                let (v, fl) = f80::from_binary::<B32>(bits, env);
                if self.state.x87.raise(fl) {
                    return Ok(());
                }
                (Some(v), true)
            }
            _ => {
                let (sr, off) = self.ea();
                let bits = self.read_mem(sr, off, 8)?;
                let (v, fl) = f80::from_binary::<B64>(bits, env);
                if self.state.x87.raise(fl) {
                    return Ok(());
                }
                (Some(v), true)
            }
        };
        let (Some(a), Some(b)) = (a, b.0) else {
            // A stack underflow: the masked response writes the indefinite.
            if may_write && b.1 {
                self.state.x87.set(dst_index, F80::INDEFINITE);
                if pop {
                    self.state.x87.pop();
                }
            }
            return Ok(());
        };
        let (value, fl) = match op {
            Op::FADD => f80::add(a, b, pc, env),
            Op::FSUB => f80::sub(a, b, pc, env),
            Op::FSUBR => f80::sub(b, a, pc, env),
            Op::FMUL => f80::mul(a, b, pc, env),
            Op::FDIV => f80::div(a, b, pc, env),
            _ => f80::div(b, a, pc, env),
        };
        self.x87_finish(dst_index, value, fl, pop)
    }

    /// The integer-source arithmetic: `FIADD` and its five siblings.
    fn x87_arith_int(&mut self, f: &Fields, op: Op) -> Ex<()> {
        let (env, pc) = self.x87_env();
        let bits = Self::int_bits(f.insn.src);
        let (sr, off) = self.ea();
        let raw = self.read_mem(sr, off, bits / 8)?;
        let (b, fl) = f80::from_signed(raw as i64, u32::from(bits), env);
        if self.state.x87.raise(fl) {
            return Ok(());
        }
        let (a, may_write) = self.x87_st(0);
        let Some(a) = a else {
            if may_write {
                self.state.x87.set(0, F80::INDEFINITE);
            }
            return Ok(());
        };
        let (value, fl) = match op {
            Op::FIADD => f80::add(a, b, pc, env),
            Op::FISUB => f80::sub(a, b, pc, env),
            Op::FISUBR => f80::sub(b, a, pc, env),
            Op::FIMUL => f80::mul(a, b, pc, env),
            Op::FIDIV => f80::div(a, b, pc, env),
            _ => f80::div(b, a, pc, env),
        };
        self.x87_finish(0, value, fl, false)
    }

    /// A one-operand operation on `ST(0)`.
    fn x87_unary(&mut self, op: fn(F80, Precision, Env) -> (F80, Flags)) -> Ex<()> {
        let (env, pc) = self.x87_env();
        let (a, may_write) = self.x87_st(0);
        let Some(a) = a else {
            if may_write {
                self.state.x87.set(0, F80::INDEFINITE);
            }
            return Ok(());
        };
        let (value, fl) = op(a, pc, env);
        self.x87_finish(0, value, fl, false)
    }

    /// `FSCALE`: `ST(0) * 2^trunc(ST(1))`.
    fn x87_scale(&mut self) -> Ex<()> {
        let (env, pc) = self.x87_env();
        let (a, aw) = self.x87_st(0);
        let (b, bw) = self.x87_st(1);
        let (Some(a), Some(b)) = (a, b) else {
            if aw && bw {
                self.state.x87.set(0, F80::INDEFINITE);
            }
            return Ok(());
        };
        // A NaN scale factor propagates, and `scale` cannot see that because
        // it takes an integer. Borrowing the multiply's NaN rules is not a
        // shortcut: `FSCALE` *is* a multiplication by a power of two, so the
        // rule for which of two NaNs survives is the same one.
        if matches!(
            f80::classify(b),
            f80::X87Class::Unsupported
                | f80::X87Class::Ieee(Category::SignalingNan | Category::QuietNan)
        ) {
            let (value, fl) = f80::mul(a, b, pc, env);
            return self.x87_finish(0, value, fl, false);
        }
        // The scale factor is *truncated*, whatever `RC` says: `FSCALE`'s
        // definition names the integer part rather than the rounded value.
        let trunc = env.round(Round::TowardZero);
        let (by, fl) = f80::to_signed(b, 64, trunc);
        // An infinite or out-of-range factor saturates far outside the
        // exponent range, so `scale` delivers the right infinity or zero
        // rather than a wrapped exponent. `to_signed`'s out-of-range answer is
        // the integer indefinite, whose sign says nothing about the operand's,
        // so the sign is read off the encoding instead.
        let by = if fl.contains(Flags::INVALID) {
            if b.sign() {
                i64::from(i32::MIN)
            } else {
                i64::from(i32::MAX)
            }
        } else {
            by
        };
        let (value, fl) = f80::scale(a, by, env);
        self.x87_finish(0, value, fl, false)
    }

    /// `FXTRACT`: the exponent replaces `ST(0)`, and the significand is pushed
    /// on top of it.
    fn x87_extract(&mut self) -> Ex<()> {
        let (env, _) = self.x87_env();
        let (a, may_write) = self.x87_st(0);
        let Some(a) = a else {
            if may_write {
                self.state.x87.set(0, F80::INDEFINITE);
                return self.x87_push_indefinite();
            }
            return Ok(());
        };
        let (exp, sig, fl) = f80::extract(a, env);
        if self.state.x87.raise(fl) {
            return Ok(());
        }
        self.state.x87.set(0, exp);
        self.x87_push_checked(sig)
    }

    /// `FPREM` and `FPREM1`.
    fn x87_remainder(&mut self, ieee: bool) -> Ex<()> {
        let (env, _) = self.x87_env();
        let (a, aw) = self.x87_st(0);
        let (b, bw) = self.x87_st(1);
        let (Some(a), Some(b)) = (a, b) else {
            if aw && bw {
                self.state.x87.set(0, F80::INDEFINITE);
            }
            return Ok(());
        };
        let r = f80::remainder(a, b, ieee, env);
        if self.state.x87.raise(r.flags) {
            return Ok(());
        }
        // The quotient's low three bits land in three condition codes that are
        // not adjacent and are not in order: `Q0` in `C1`, `Q1` in `C3` and
        // `Q2` in `C0` (*Intel SDM* volume 2, `FPREM`). `C2` says the
        // reduction is only partial, and then the other three mean nothing.
        let q = r.quotient;
        self.state.x87.set_condition(
            !r.incomplete && q & 4 != 0,
            !r.incomplete && q & 1 != 0,
            r.incomplete,
            !r.incomplete && q & 2 != 0,
        );
        self.state.x87.set(0, r.value);
        Ok(())
    }

    /// Record a result: the flags first, then the write, then the pop.
    fn x87_finish(&mut self, dest: u8, value: F80, flags: Flags, pop: bool) -> Ex<()> {
        if self.state.x87.raise(flags) {
            // An unmasked exception leaves the destination and the stack
            // exactly as the handler needs to see them.
            return Ok(());
        }
        // `C1` would say "the result was rounded up" on hardware; the kernel
        // does not report the direction, so it is cleared. The module
        // documentation says so rather than leaving it to be discovered.
        self.state.x87.status &= !sw::C1;
        self.state.x87.set(dest, value);
        if pop {
            self.state.x87.pop();
        }
        Ok(())
    }

    /// The condition-code compares: `FCOM`, `FUCOM` and their popping forms.
    ///
    /// `signalling` distinguishes `FCOM`, which raises invalid for **any**
    /// NaN, from `FUCOM`, which raises it only for a signaling one — the
    /// distinction IEEE 754-2019 §5.11 draws between the two comparison
    /// families, and the reason a language that wants `x != x` to work uses
    /// the unordered form.
    fn x87_compare(&mut self, f: &Fields, signalling: bool, pops: u8) -> Ex<()> {
        let (env, _) = self.x87_env();
        // Three operand shapes share these opcodes, and the row says which:
        // a memory source (`FCOM m32fp`), a named register (`FCOM ST(i)`,
        // whose row carries it in the *destination* slot because Intel writes
        // it as the only operand), and no operand at all — `FCOMPP` and
        // `FUCOMPP`, which always compare against `ST(1)`.
        let (b, bw) = match f.insn.src {
            Arg::Mf32 => {
                let (sr, off) = self.ea();
                let bits = self.read_mem(sr, off, 4)?;
                let (v, fl) = f80::from_binary::<B32>(bits, env);
                if self.state.x87.raise(fl) {
                    return Ok(());
                }
                (Some(v), true)
            }
            Arg::Mf64 => {
                let (sr, off) = self.ea();
                let bits = self.read_mem(sr, off, 8)?;
                let (v, fl) = f80::from_binary::<B64>(bits, env);
                if self.state.x87.raise(fl) {
                    return Ok(());
                }
                (Some(v), true)
            }
            _ => {
                let which = if f.insn.dst == Arg::Sti {
                    Self::sti(f)
                } else {
                    1
                };
                self.x87_st(which)
            }
        };
        let (a, aw) = self.x87_st(0);
        let (Some(a), Some(b)) = (a, b) else {
            if aw && bw {
                // The masked response to comparing an empty register is the
                // unordered result, which is the one a program tests for.
                self.state.x87.set_condition(true, false, true, true);
                self.x87_pop_n(pops);
            }
            return Ok(());
        };
        self.x87_compare_values(a, b, signalling, pops, false)
    }

    /// `FICOM`/`FICOMP`: the same, against an integer in memory.
    fn x87_compare_int(&mut self, f: &Fields, pop: bool) -> Ex<()> {
        let (env, _) = self.x87_env();
        let bits = Self::int_bits(f.insn.src);
        let (sr, off) = self.ea();
        let raw = self.read_mem(sr, off, bits / 8)?;
        let (b, fl) = f80::from_signed(raw as i64, u32::from(bits), env);
        if self.state.x87.raise(fl) {
            return Ok(());
        }
        let (a, may_write) = self.x87_st(0);
        let Some(a) = a else {
            if may_write {
                self.state.x87.set_condition(true, false, true, true);
                self.x87_pop_n(u8::from(pop));
            }
            return Ok(());
        };
        self.x87_compare_values(a, b, true, u8::from(pop), false)
    }

    /// `FTST`: compare `ST(0)` with `+0.0`.
    fn x87_test(&mut self) -> Ex<()> {
        let (a, may_write) = self.x87_st(0);
        let Some(a) = a else {
            if may_write {
                self.state.x87.set_condition(true, false, true, true);
            }
            return Ok(());
        };
        self.x87_compare_values(a, F80::ZERO, true, 0, false)
    }

    /// `FCOMI` and friends: the compare that writes `EFLAGS` instead.
    fn x87_compare_flags(&mut self, f: &Fields, signalling: bool, pop: bool) -> Ex<()> {
        let (a, aw) = self.x87_st(0);
        let (b, bw) = self.x87_st(Self::sti(f));
        let (Some(a), Some(b)) = (a, b) else {
            if aw && bw {
                self.set_unordered_flags();
                self.x87_pop_n(u8::from(pop));
            }
            return Ok(());
        };
        self.x87_compare_values(a, b, signalling, u8::from(pop), true)
    }

    /// The shared body of every x87 comparison.
    fn x87_compare_values(
        &mut self,
        a: F80,
        b: F80,
        signalling: bool,
        pops: u8,
        to_flags: bool,
    ) -> Ex<()> {
        use core::cmp::Ordering;
        let kind = |v: F80| match f80::classify(v) {
            f80::X87Class::Ieee(Category::SignalingNan) => Some(true),
            f80::X87Class::Ieee(Category::QuietNan) => Some(false),
            // An unsupported encoding is not a value; every comparison against
            // one is invalid and unordered.
            f80::X87Class::Unsupported => Some(true),
            _ => None,
        };
        let (ka, kb) = (kind(a), kind(b));
        let any_nan = ka.is_some() || kb.is_some();
        let signalling_nan = ka == Some(true) || kb == Some(true);
        if (signalling_nan || (signalling && any_nan)) && self.state.x87.raise(Flags::INVALID) {
            return Ok(());
        }
        let order = if any_nan { None } else { f80::compare(a, b) };
        if to_flags {
            match order {
                Some(Ordering::Equal) => self.set_compare_flags(true, false, false),
                Some(Ordering::Greater) => self.set_compare_flags(false, false, false),
                Some(Ordering::Less) => self.set_compare_flags(false, false, true),
                None => self.set_unordered_flags(),
            }
        } else {
            // C3 is "equal", C0 is "less than", and both are set together with
            // C2 for unordered (*Intel SDM* volume 1 §8.3.4, table 8-3).
            match order {
                Some(Ordering::Equal) => self.state.x87.set_condition(false, false, false, true),
                Some(Ordering::Greater) => self.state.x87.set_condition(false, false, false, false),
                Some(Ordering::Less) => self.state.x87.set_condition(true, false, false, false),
                None => self.state.x87.set_condition(true, false, true, true),
            }
        }
        self.x87_pop_n(pops);
        Ok(())
    }

    /// `ZF`, `PF` and `CF` as `FCOMI` sets them; `OF`, `SF` and `AF` cleared.
    fn set_compare_flags(&mut self, zf: bool, pf: bool, cf: bool) {
        let mut value = self.state.regs.eflags & !(flags::ZF | flags::PF | flags::CF);
        value &= !(flags::OF | flags::SF | flags::AF);
        if zf {
            value |= flags::ZF;
        }
        if pf {
            value |= flags::PF;
        }
        if cf {
            value |= flags::CF;
        }
        self.set_flags(value);
    }

    /// The unordered answer: all three set.
    fn set_unordered_flags(&mut self) {
        self.set_compare_flags(true, true, true);
    }

    /// Pop zero, one or two registers.
    fn x87_pop_n(&mut self, n: u8) {
        for _ in 0..n {
            self.state.x87.pop();
        }
    }

    /// `FXAM`: classify `ST(0)` into the condition codes.
    ///
    /// The one instruction that can look at an empty register without
    /// faulting — telling empty from zero is what it is for (*Intel SDM*
    /// volume 2, `FXAM`, table 8-4's encoding).
    fn x87_examine(&mut self) {
        let sign = self.state.x87.raw(0).sign();
        let (c3, c2, c0) = if !self.state.x87.occupied(0) {
            (true, false, true)
        } else {
            match f80::classify(self.state.x87.raw(0)) {
                f80::X87Class::Unsupported => (false, false, false),
                f80::X87Class::Ieee(c) => match c {
                    Category::SignalingNan | Category::QuietNan => (false, false, true),
                    Category::PositiveInfinity | Category::NegativeInfinity => (false, true, true),
                    Category::PositiveZero | Category::NegativeZero => (true, false, false),
                    Category::PositiveSubnormal | Category::NegativeSubnormal => {
                        (true, true, false)
                    }
                    Category::PositiveNormal | Category::NegativeNormal => (false, true, false),
                },
            }
        };
        // `C1` is the sign bit here rather than a rounding indicator, and it
        // is read off the encoding even for an empty register.
        self.state.x87.set_condition(c0, sign, c2, c3);
    }

    /// `FCMOVcc`: the conditional moves, which read the *integer* flags.
    fn x87_cmov(&mut self, f: &Fields, op: Op) -> Ex<()> {
        if !self.cfg.features.cmov {
            return Err(Fault::bare(VEC_UD));
        }
        let take = match op {
            Op::FCMOVB => self.flag(flags::CF),
            Op::FCMOVE => self.flag(flags::ZF),
            Op::FCMOVBE => self.flag(flags::CF) || self.flag(flags::ZF),
            Op::FCMOVU => self.flag(flags::PF),
            Op::FCMOVNB => !self.flag(flags::CF),
            Op::FCMOVNE => !self.flag(flags::ZF),
            Op::FCMOVNBE => !self.flag(flags::CF) && !self.flag(flags::ZF),
            _ => !self.flag(flags::PF),
        };
        if !take {
            return Ok(());
        }
        let (value, may_write) = self.x87_st(Self::sti(f));
        let Some(value) = value else {
            if may_write {
                self.state.x87.set(0, F80::INDEFINITE);
            }
            return Ok(());
        };
        self.state.x87.set(0, value);
        Ok(())
    }

    // -- The environment and the save areas -----------------------------

    /// How wide the `FNSTENV` fields are: two bytes or four.
    fn env_wide(f: &Fields) -> bool {
        f.opsize != 2
    }

    /// `FNSTENV`: write the control, status and tag words and the pointers,
    /// then mask every exception.
    fn x87_store_env(&mut self, f: &Fields) -> Ex<()> {
        let (sr, off) = self.ea();
        let wide = Self::env_wide(f);
        self.write_env(sr, off, wide)?;
        // The documented side effect, and the reason `FNSTENV` is what an
        // exception handler runs first: it takes the unit out of the state
        // that would immediately fault again (*Intel SDM* volume 2,
        // `FSTENV`).
        self.state.x87.control |= cw::MASKS;
        self.x87_resummarise();
        Ok(())
    }

    /// The seven fields of the environment, at either width.
    fn write_env(&mut self, sr: u8, off: u64, wide: bool) -> Ex<()> {
        let x = self.state.x87;
        let step = if wide { 4 } else { 2 };
        let at = |n: u64| off.wrapping_add(n * step);
        self.write_mem(sr, at(0), step as u8, u64::from(x.control))?;
        self.write_mem(sr, at(1), step as u8, u64::from(x.status))?;
        self.write_mem(sr, at(2), step as u8, u64::from(x.tag))?;
        if wide {
            self.write_mem(sr, at(3), 4, x.last_ip & 0xffff_ffff)?;
            // The selector and the eleven opcode bits share a doubleword.
            let sel = u64::from(x.last_cs) | (u64::from(x.last_op & 0x7ff) << 16);
            self.write_mem(sr, at(4), 4, sel)?;
            self.write_mem(sr, at(5), 4, x.last_dp & 0xffff_ffff)?;
            self.write_mem(sr, at(6), 4, u64::from(x.last_ds))?;
        } else {
            self.write_mem(sr, at(3), 2, x.last_ip & 0xffff)?;
            self.write_mem(sr, at(4), 2, u64::from(x.last_cs))?;
            self.write_mem(sr, at(5), 2, x.last_dp & 0xffff)?;
            self.write_mem(sr, at(6), 2, u64::from(x.last_ds))?;
        }
        Ok(())
    }

    /// `FLDENV`.
    fn x87_load_env(&mut self, f: &Fields) -> Ex<()> {
        let (sr, off) = self.ea();
        let wide = Self::env_wide(f);
        self.read_env(sr, off, wide)?;
        self.x87_resummarise();
        Ok(())
    }

    /// The reading half of [`Exec::write_env`].
    fn read_env(&mut self, sr: u8, off: u64, wide: bool) -> Ex<()> {
        let step = if wide { 4u64 } else { 2 };
        let at = |n: u64| off.wrapping_add(n * step);
        let control = self.read_mem(sr, at(0), step as u8)? as u16;
        let status = self.read_mem(sr, at(1), step as u8)? as u16;
        let tag = self.read_mem(sr, at(2), step as u8)? as u16;
        let (ip, cs, op, dp, ds) = if wide {
            let ip = self.read_mem(sr, at(3), 4)?;
            let sel = self.read_mem(sr, at(4), 4)?;
            let dp = self.read_mem(sr, at(5), 4)?;
            let ds = self.read_mem(sr, at(6), 4)?;
            (ip, sel as u16, ((sel >> 16) & 0x7ff) as u16, dp, ds as u16)
        } else {
            let ip = self.read_mem(sr, at(3), 2)?;
            let cs = self.read_mem(sr, at(4), 2)?;
            let dp = self.read_mem(sr, at(5), 2)?;
            let ds = self.read_mem(sr, at(6), 2)?;
            (ip, cs as u16, 0, dp, ds as u16)
        };
        let x = &mut self.state.x87;
        x.control = control;
        x.status = status;
        x.tag = tag;
        x.last_ip = ip;
        x.last_cs = cs;
        x.last_op = op;
        x.last_dp = dp;
        x.last_ds = ds;
        Ok(())
    }

    /// `FNSAVE`: the environment followed by the eight registers, then a
    /// re-initialisation.
    fn x87_save(&mut self, f: &Fields) -> Ex<()> {
        let (sr, off) = self.ea();
        let wide = Self::env_wide(f);
        self.write_env(sr, off, wide)?;
        let base = off.wrapping_add(if wide { 28 } else { 14 });
        // Stack order, not physical order: the image begins with `ST(0)`,
        // which is what the manual's diagram shows and what makes the image
        // meaningful without also knowing `TOP`.
        for i in 0..8u8 {
            let value = self.state.x87.raw(i);
            self.write_f80(sr, base.wrapping_add(u64::from(i) * 10), value)?;
        }
        self.state.x87.init();
        Ok(())
    }

    /// `FRSTOR`.
    fn x87_restore(&mut self, f: &Fields) -> Ex<()> {
        let (sr, off) = self.ea();
        let wide = Self::env_wide(f);
        self.read_env(sr, off, wide)?;
        let base = off.wrapping_add(if wide { 28 } else { 14 });
        for i in 0..8u8 {
            let value = self.read_f80(sr, base.wrapping_add(u64::from(i) * 10))?;
            // The register file is written directly rather than through
            // `set`, because the tag word came out of the image and must not
            // be recomputed: `FRSTOR` can legitimately restore a tag that
            // disagrees with the bits beside it, and software has used that.
            let p = self.state.x87.phys(i);
            self.state.x87.regs[p as usize] = value;
        }
        self.x87_resummarise();
        Ok(())
    }

    // -----------------------------------------------------------------
    // SSE
    // -----------------------------------------------------------------

    /// Execute one SSE, `FXSAVE`-family or fence instruction.
    pub(super) fn sse_instruction(&mut self, f: &Fields) -> Ex<()> {
        let op = f.insn.op;
        self.sse_gate(op)?;
        match op {
            Op::LFENCE | Op::MFENCE | Op::SFENCE => Ok(()),
            Op::FXSAVE => self.fxsave(f),
            Op::FXRSTOR => self.fxrstor(f),
            Op::LDMXCSR => {
                let value = self.read_arg(f, Arg::Ed, 4)? as u32;
                if value & !mxcsr::WRITABLE != 0 {
                    // A reserved bit is how a guest probes for a feature this
                    // core does not have; accepting one would answer yes.
                    return Err(Fault::gp(0));
                }
                self.state.sse.mxcsr = value;
                Ok(())
            }
            Op::STMXCSR => {
                let value = u64::from(self.state.sse.mxcsr);
                self.write_arg(f, Arg::Ed, 4, value)
            }
            _ => self.sse_data(f),
        }
    }

    /// Read the r/m operand of an SSE instruction as a 128-bit pair.
    ///
    /// A narrower memory operand is zero-extended, which is right for every
    /// instruction that reads one: the lanes above the operand are either
    /// unused or explicitly zeroed.
    fn sse_read_rm(&mut self, f: &Fields, arg: Arg, aligned: bool) -> Ex<[u64; 2]> {
        let register = f.modrm.is_some_and(super::isa::ModRm::is_register);
        if register || matches!(arg, Arg::Ux) {
            return Ok(self.state.sse.get(f.rm_num()));
        }
        let (sr, off) = self.ea();
        if aligned {
            self.require_aligned(off)?;
        }
        match arg {
            Arg::Wd => Ok([self.read_mem(sr, off, 4)?, 0]),
            Arg::Wq | Arg::Mq => Ok([self.read_mem(sr, off, 8)?, 0]),
            _ => self.read_xmm_mem(sr, off),
        }
    }

    /// Write the r/m operand of an SSE instruction.
    fn sse_write_rm(&mut self, f: &Fields, arg: Arg, aligned: bool, value: [u64; 2]) -> Ex<()> {
        let register = f.modrm.is_some_and(super::isa::ModRm::is_register);
        if register {
            let index = f.rm_num();
            match arg {
                // A register-to-register scalar move touches only the lanes
                // it names; a memory store writes exactly those bytes. Both
                // are "leave the rest alone", which is why one arm serves.
                Arg::Wd => {
                    let low = self.state.sse.low(index);
                    self.state
                        .sse
                        .set_low(index, (low & !0xffff_ffff) | (value[0] & 0xffff_ffff));
                }
                Arg::Wq | Arg::Mq => self.state.sse.set_low(index, value[0]),
                _ => self.state.sse.set(index, value),
            }
            return Ok(());
        }
        let (sr, off) = self.ea();
        if aligned {
            self.require_aligned(off)?;
        }
        match arg {
            Arg::Wd => self.write_mem(sr, off, 4, value[0] & 0xffff_ffff),
            Arg::Wq | Arg::Mq => self.write_mem(sr, off, 8, value[0]),
            _ => self.write_xmm_mem(sr, off, value),
        }
    }

    /// The data-movement, logic and arithmetic instructions.
    #[allow(clippy::too_many_lines)]
    fn sse_data(&mut self, f: &Fields) -> Ex<()> {
        let op = f.insn.op;
        let reg = f.reg_num();
        let env = self.state.sse.env();
        let from_memory = f.modrm.is_some_and(|m| !m.is_register());
        match op {
            // -- Moves ---------------------------------------------------
            Op::MOVUPS | Op::MOVUPD | Op::MOVDQU | Op::MOVAPS | Op::MOVAPD | Op::MOVDQA => {
                let aligned = matches!(op, Op::MOVAPS | Op::MOVAPD | Op::MOVDQA);
                if f.insn.dst == Arg::Vx {
                    let v = self.sse_read_rm(f, f.insn.src, aligned)?;
                    self.state.sse.set(reg, v);
                } else {
                    let v = self.state.sse.get(reg);
                    self.sse_write_rm(f, f.insn.dst, aligned, v)?;
                }
                Ok(())
            }
            Op::MOVSS | Op::MOVSD => {
                let wide = op == Op::MOVSD;
                if f.insn.dst == Arg::Vx {
                    let v = self.sse_read_rm(f, f.insn.src, false)?;
                    if from_memory {
                        // A load zeroes everything above the scalar; a
                        // register-to-register move does not. That asymmetry
                        // is the whole difference between the two encodings
                        // of one mnemonic (*Intel SDM* volume 2, `MOVSS`).
                        let low = if wide { v[0] } else { v[0] & 0xffff_ffff };
                        self.state.sse.set(reg, [low, 0]);
                    } else if wide {
                        self.state.sse.set_low(reg, v[0]);
                    } else {
                        let keep = self.state.sse.low(reg) & !0xffff_ffff;
                        self.state.sse.set_low(reg, keep | (v[0] & 0xffff_ffff));
                    }
                } else {
                    let v = self.state.sse.get(reg);
                    self.sse_write_rm(f, f.insn.dst, false, v)?;
                }
                Ok(())
            }
            Op::MOVQ => {
                if f.insn.dst == Arg::Vx {
                    // `F3 0F 7E` always zeroes the upper quadword, register
                    // source included.
                    let v = self.sse_read_rm(f, f.insn.src, false)?;
                    self.state.sse.set(reg, [v[0], 0]);
                } else if from_memory {
                    let v = self.state.sse.low(reg);
                    let (sr, off) = self.ea();
                    self.write_mem(sr, off, 8, v)?;
                } else {
                    let v = self.state.sse.low(reg);
                    self.state.sse.set(f.rm_num(), [v, 0]);
                }
                Ok(())
            }
            Op::MOVD => {
                let wide = f.rex_w();
                if f.insn.dst == Arg::Vx {
                    let value = self.read_arg(f, Arg::Ey, if wide { 8 } else { 4 })?;
                    let value = if wide { value } else { value & 0xffff_ffff };
                    self.state.sse.set(reg, [value, 0]);
                } else {
                    let value = self.state.sse.low(reg);
                    let value = if wide { value } else { value & 0xffff_ffff };
                    self.write_arg(f, Arg::Ey, if wide { 8 } else { 4 }, value)?;
                }
                Ok(())
            }
            Op::MOVLPS | Op::MOVLPD => {
                if f.insn.dst == Arg::Vx {
                    let v = self.sse_read_rm(f, Arg::Mq, false)?;
                    self.state.sse.set_low(reg, v[0]);
                } else {
                    let v = self.state.sse.low(reg);
                    self.sse_write_rm(f, Arg::Mq, false, [v, 0])?;
                }
                Ok(())
            }
            Op::MOVHPS | Op::MOVHPD => {
                if f.insn.dst == Arg::Vx {
                    let v = self.sse_read_rm(f, Arg::Mq, false)?;
                    self.state.sse.set_high(reg, v[0]);
                } else {
                    let v = self.state.sse.high(reg);
                    self.sse_write_rm(f, Arg::Mq, false, [v, 0])?;
                }
                Ok(())
            }
            Op::MOVHLPS => {
                let v = self.state.sse.high(f.rm_num());
                self.state.sse.set_low(reg, v);
                Ok(())
            }
            Op::MOVLHPS => {
                let v = self.state.sse.low(f.rm_num());
                self.state.sse.set_high(reg, v);
                Ok(())
            }
            Op::MOVMSKPS | Op::MOVMSKPD => {
                if from_memory {
                    // There is no memory form: the source is a register and
                    // the destination is a general register, so `mod != 11` is
                    // not an encoding at all.
                    return Err(Fault::bare(VEC_UD));
                }
                let v = self.state.sse.get(f.rm_num());
                let mask = if op == Op::MOVMSKPD {
                    (v[0] >> 63) | ((v[1] >> 63) << 1)
                } else {
                    let lane = |w: u64, half: u32| (w >> (31 + 32 * half)) & 1;
                    lane(v[0], 0)
                        | (lane(v[0], 1) << 1)
                        | (lane(v[1], 0) << 2)
                        | (lane(v[1], 1) << 3)
                };
                // The destination is a general register, and a 32-bit write
                // zero-extends as any other does.
                self.write_arg(f, Arg::Gy, if f.rex_w() { 8 } else { 4 }, mask)
            }

            // -- Bitwise -------------------------------------------------
            Op::ANDPS | Op::ANDPD | Op::PAND => self.sse_bitwise(f, |a, b| a & b),
            Op::ANDNPS | Op::ANDNPD | Op::PANDN => self.sse_bitwise(f, |a, b| !a & b),
            Op::ORPS | Op::ORPD | Op::POR => self.sse_bitwise(f, |a, b| a | b),
            Op::XORPS | Op::XORPD | Op::PXOR => self.sse_bitwise(f, |a, b| a ^ b),

            // -- Shuffles ------------------------------------------------
            Op::UNPCKLPS | Op::UNPCKHPS | Op::UNPCKLPD | Op::UNPCKHPD | Op::SHUFPS | Op::SHUFPD => {
                self.sse_shuffle(f, op)
            }

            // -- Comparisons that write the flags ------------------------
            Op::UCOMISS | Op::COMISS | Op::UCOMISD | Op::COMISD => self.sse_ordered(f, op),

            // -- Comparisons that write a mask ---------------------------
            Op::CMPPS | Op::CMPSS | Op::CMPPD | Op::CMPSD => self.sse_mask_compare(f, op),

            // -- Conversions ---------------------------------------------
            Op::CVTSI2SS | Op::CVTSI2SD => {
                let wide = f.rex_w();
                let raw = self.read_arg(f, Arg::Ey, if wide { 8 } else { 4 })? as i64;
                let bits = if wide { 64 } else { 32 };
                let (value, fl) = if op == Op::CVTSI2SS {
                    binary::from_signed::<B32>(raw, bits, env)
                } else {
                    binary::from_signed::<B64>(raw, bits, env)
                };
                self.sse_raise(fl)?;
                if op == Op::CVTSI2SS {
                    let keep = self.state.sse.low(reg) & !0xffff_ffff;
                    self.state.sse.set_low(reg, keep | value);
                } else {
                    self.state.sse.set_low(reg, value);
                }
                Ok(())
            }
            Op::CVTSS2SI | Op::CVTTSS2SI | Op::CVTSD2SI | Op::CVTTSD2SI => {
                let single = matches!(op, Op::CVTSS2SI | Op::CVTTSS2SI);
                let truncating = matches!(op, Op::CVTTSS2SI | Op::CVTTSD2SI);
                let src = self.sse_read_rm(f, f.insn.src, false)?;
                let env = if truncating {
                    env.round(Round::TowardZero)
                } else {
                    env
                };
                let wide = f.rex_w();
                let bits = if wide { 64 } else { 32 };
                let (value, fl) = if single {
                    binary::to_signed::<B32>(src[0] & 0xffff_ffff, bits, env)
                } else {
                    binary::to_signed::<B64>(src[0], bits, env)
                };
                self.sse_raise(fl)?;
                self.write_arg(f, Arg::Gy, if wide { 8 } else { 4 }, value as u64)
            }
            Op::CVTSS2SD => {
                let src = self.sse_read_rm(f, f.insn.src, false)?;
                let (value, fl) = binary::convert::<B32, B64>(src[0] & 0xffff_ffff, env);
                self.sse_raise(fl)?;
                self.state.sse.set_low(reg, value);
                Ok(())
            }
            Op::CVTSD2SS => {
                let src = self.sse_read_rm(f, f.insn.src, false)?;
                let (value, fl) = binary::convert::<B64, B32>(src[0], env);
                self.sse_raise(fl)?;
                let keep = self.state.sse.low(reg) & !0xffff_ffff;
                self.state.sse.set_low(reg, keep | value);
                Ok(())
            }
            Op::CVTPS2PD => {
                let src = self.sse_read_rm(f, f.insn.src, false)?;
                let (lo, f0) = binary::convert::<B32, B64>(src[0] & 0xffff_ffff, env);
                let (hi, f1) = binary::convert::<B32, B64>(src[0] >> 32, env);
                self.sse_raise(f0 | f1)?;
                self.state.sse.set(reg, [lo, hi]);
                Ok(())
            }
            Op::CVTPD2PS => {
                let src = self.sse_read_rm(f, f.insn.src, false)?;
                let (lo, f0) = binary::convert::<B64, B32>(src[0], env);
                let (hi, f1) = binary::convert::<B64, B32>(src[1], env);
                self.sse_raise(f0 | f1)?;
                self.state.sse.set(reg, [lo | (hi << 32), 0]);
                Ok(())
            }

            // -- Arithmetic ----------------------------------------------
            _ => self.sse_arith(f, op),
        }
    }

    /// The four bitwise operations, which are the same 128 bits whatever the
    /// lane width in the mnemonic says.
    fn sse_bitwise(&mut self, f: &Fields, op: fn(u64, u64) -> u64) -> Ex<()> {
        let reg = f.reg_num();
        let src = self.sse_read_rm(f, f.insn.src, false)?;
        let dst = self.state.sse.get(reg);
        self.state
            .sse
            .set(reg, [op(dst[0], src[0]), op(dst[1], src[1])]);
        Ok(())
    }

    /// `UNPCK*` and `SHUFP*`: lane selection with no arithmetic.
    fn sse_shuffle(&mut self, f: &Fields, op: Op) -> Ex<()> {
        let reg = f.reg_num();
        let src = self.sse_read_rm(f, f.insn.src, false)?;
        let dst = self.state.sse.get(reg);
        // The four singles of a register, low to high.
        let s = |v: [u64; 2], i: u32| (v[(i / 2) as usize] >> (32 * (i % 2))) & 0xffff_ffff;
        let imm = f.imm as u32;
        let value = match op {
            Op::UNPCKLPS => [s(dst, 0) | (s(src, 0) << 32), s(dst, 1) | (s(src, 1) << 32)],
            Op::UNPCKHPS => [s(dst, 2) | (s(src, 2) << 32), s(dst, 3) | (s(src, 3) << 32)],
            Op::UNPCKLPD => [dst[0], src[0]],
            Op::UNPCKHPD => [dst[1], src[1]],
            Op::SHUFPD => [
                if imm & 1 == 0 { dst[0] } else { dst[1] },
                if imm & 2 == 0 { src[0] } else { src[1] },
            ],
            // `SHUFPS`: the low two lanes come from the destination and the
            // high two from the source, each selected by two bits of the
            // immediate.
            _ => [
                s(dst, imm & 3) | (s(dst, (imm >> 2) & 3) << 32),
                s(src, (imm >> 4) & 3) | (s(src, (imm >> 6) & 3) << 32),
            ],
        };
        self.state.sse.set(reg, value);
        Ok(())
    }

    /// `UCOMISS`, `COMISS` and their double-precision counterparts.
    fn sse_ordered(&mut self, f: &Fields, op: Op) -> Ex<()> {
        use core::cmp::Ordering;
        let single = matches!(op, Op::UCOMISS | Op::COMISS);
        let signalling = matches!(op, Op::COMISS | Op::COMISD);
        let env = self.state.sse.env();
        let src = self.sse_read_rm(f, f.insn.src, false)?;
        let dst = self.state.sse.get(f.reg_num());
        let (a, b) = if single {
            (dst[0] & 0xffff_ffff, src[0] & 0xffff_ffff)
        } else {
            (dst[0], src[0])
        };
        let (order, fl) = if single {
            Self::ordered_pair::<B32>(a, b, signalling, env)
        } else {
            Self::ordered_pair::<B64>(a, b, signalling, env)
        };
        self.sse_raise(fl)?;
        match order {
            Some(Ordering::Equal) => self.set_compare_flags(true, false, false),
            Some(Ordering::Greater) => self.set_compare_flags(false, false, false),
            Some(Ordering::Less) => self.set_compare_flags(false, false, true),
            None => self.set_unordered_flags(),
        }
        Ok(())
    }

    /// The comparison behind `COMISS`/`UCOMISS`, at one format.
    /// Apply the environment's subnormal rule to a comparison operand.
    ///
    /// The arithmetic paths get this from `float`'s own decode, but a
    /// comparison never reaches the kernel — `binary::compare` reads bit
    /// patterns and takes no [`Env`] — so `#D` and `MXCSR.DAZ` have to be
    /// applied here or they are silently missing. Both matter: the SDM lists
    /// the denormal exception for every one of `COMISS`, `UCOMISS`, `CMPPS`
    /// and their double-precision counterparts, and with `DAZ` set a
    /// subnormal must compare **equal** to zero rather than merely close to
    /// it.
    fn compare_operand<F: crate::float::Format>(v: u64, env: Env) -> (u64, Flags) {
        if !matches!(
            binary::classify::<F>(v),
            Category::PositiveSubnormal | Category::NegativeSubnormal
        ) {
            return (v, Flags::NONE);
        }
        let flags = if env.subnormal_inputs.reports() {
            Flags::DENORMAL
        } else {
            Flags::NONE
        };
        if env.subnormal_inputs.flushes() {
            // A zero of the same sign, which is all `DAZ` ever substitutes.
            (v & F::SIGN, flags)
        } else {
            (v, flags)
        }
    }

    fn ordered_pair<F: crate::float::Format>(
        a: u64,
        b: u64,
        signalling: bool,
        env: Env,
    ) -> (Option<core::cmp::Ordering>, Flags) {
        let (a, fa) = Self::compare_operand::<F>(a, env);
        let (b, fb) = Self::compare_operand::<F>(b, env);
        let kind = |v: u64| match binary::classify::<F>(v) {
            Category::SignalingNan => Some(true),
            Category::QuietNan => Some(false),
            _ => None,
        };
        let (ka, kb) = (kind(a), kind(b));
        let any = ka.is_some() || kb.is_some();
        let snan = ka == Some(true) || kb == Some(true);
        let mut flags = fa | fb;
        if snan || (signalling && any) {
            flags |= Flags::INVALID;
        }
        let order = if any {
            None
        } else {
            binary::compare::<F>(a, b)
        };
        (order, flags)
    }

    /// `CMPPS`/`CMPSS`/`CMPPD`/`CMPSD`: a lane-wise mask of all ones or all
    /// zeros, under one of eight immediate predicates.
    fn sse_mask_compare(&mut self, f: &Fields, op: Op) -> Ex<()> {
        let single = matches!(op, Op::CMPPS | Op::CMPSS);
        let scalar = matches!(op, Op::CMPSS | Op::CMPSD);
        let predicate = (f.imm as u8) & 7;
        let env = self.state.sse.env();
        let reg = f.reg_num();
        let src = self.sse_read_rm(f, f.insn.src, false)?;
        let dst = self.state.sse.get(reg);
        let mut flags = Flags::NONE;
        let mut out = dst;
        if single {
            let lanes = if scalar { 1 } else { 4 };
            for i in 0..lanes {
                let a = (dst[i / 2] >> (32 * (i % 2))) & 0xffff_ffff;
                let b = (src[i / 2] >> (32 * (i % 2))) & 0xffff_ffff;
                let (r, fl) = Self::predicate::<B32>(a, b, predicate, env);
                flags |= fl;
                let mask = if r { 0xffff_ffffu64 } else { 0 };
                let shift = 32 * (i % 2);
                out[i / 2] = (out[i / 2] & !(0xffff_ffffu64 << shift)) | (mask << shift);
            }
        } else {
            let lanes = if scalar { 1 } else { 2 };
            for i in 0..lanes {
                let (r, fl) = Self::predicate::<B64>(dst[i], src[i], predicate, env);
                flags |= fl;
                out[i] = if r { u64::MAX } else { 0 };
            }
        }
        self.sse_raise(flags)?;
        self.state.sse.set(reg, out);
        Ok(())
    }

    /// One of the eight compare predicates (*Intel SDM* volume 2, `CMPPS`,
    /// table 3-7).
    ///
    /// Predicates 0-2 and 4-6 are the *ordered* half in the sense that they
    /// are false for an unordered pair — except that 4-6 are the negations of
    /// 0-2 and are therefore true for it. Only 3 and 7 ask about ordering
    /// directly. The signalling column matters too: `EQ`, `NEQ`, `ORD` and
    /// `UNORD` are quiet and the other four signal on any NaN.
    fn predicate<F: crate::float::Format>(a: u64, b: u64, which: u8, env: Env) -> (bool, Flags) {
        let (a, fa) = Self::compare_operand::<F>(a, env);
        let (b, fb) = Self::compare_operand::<F>(b, env);
        let is_nan = |v: u64| {
            matches!(
                binary::classify::<F>(v),
                Category::QuietNan | Category::SignalingNan
            )
        };
        let snan = |v: u64| binary::classify::<F>(v) == Category::SignalingNan;
        let unordered = is_nan(a) || is_nan(b);
        let quiet = matches!(which, 0 | 3 | 4 | 7);
        let mut flags = fa | fb;
        if (quiet && (snan(a) || snan(b))) || (!quiet && unordered) {
            flags |= Flags::INVALID;
        }
        let order = binary::compare::<F>(a, b);
        let result = match which {
            0 => order == Some(core::cmp::Ordering::Equal),
            1 => order == Some(core::cmp::Ordering::Less),
            2 => matches!(
                order,
                Some(core::cmp::Ordering::Less | core::cmp::Ordering::Equal)
            ),
            3 => unordered,
            4 => unordered || order != Some(core::cmp::Ordering::Equal),
            5 => unordered || order != Some(core::cmp::Ordering::Less),
            6 => {
                unordered
                    || !matches!(
                        order,
                        Some(core::cmp::Ordering::Less | core::cmp::Ordering::Equal)
                    )
            }
            _ => !unordered,
        };
        (result, flags)
    }

    /// The packed and scalar arithmetic: one kernel over four lane shapes.
    fn sse_arith(&mut self, f: &Fields, op: Op) -> Ex<()> {
        let env = self.state.sse.env();
        let reg = f.reg_num();
        let src = self.sse_read_rm(f, f.insn.src, false)?;
        let dst = self.state.sse.get(reg);
        let single = matches!(
            op,
            Op::ADDPS
                | Op::ADDSS
                | Op::SUBPS
                | Op::SUBSS
                | Op::MULPS
                | Op::MULSS
                | Op::DIVPS
                | Op::DIVSS
                | Op::MINPS
                | Op::MINSS
                | Op::MAXPS
                | Op::MAXSS
                | Op::SQRTPS
                | Op::SQRTSS
        );
        let scalar = matches!(
            op,
            Op::ADDSS
                | Op::ADDSD
                | Op::SUBSS
                | Op::SUBSD
                | Op::MULSS
                | Op::MULSD
                | Op::DIVSS
                | Op::DIVSD
                | Op::MINSS
                | Op::MINSD
                | Op::MAXSS
                | Op::MAXSD
                | Op::SQRTSS
                | Op::SQRTSD
        );
        let mut flags = Flags::NONE;
        let mut out = dst;
        if single {
            let lanes = if scalar { 1 } else { 4 };
            for i in 0..lanes {
                let shift = 32 * (i % 2);
                let a = (dst[i / 2] >> shift) & 0xffff_ffff;
                let b = (src[i / 2] >> shift) & 0xffff_ffff;
                let (v, fl) = Self::lane::<B32>(op, a, b, env);
                flags |= fl;
                out[i / 2] = (out[i / 2] & !(0xffff_ffffu64 << shift)) | (v << shift);
            }
        } else {
            let lanes = if scalar { 1 } else { 2 };
            for i in 0..lanes {
                let (v, fl) = Self::lane::<B64>(op, dst[i], src[i], env);
                flags |= fl;
                out[i] = v;
            }
        }
        self.sse_raise(flags)?;
        self.state.sse.set(reg, out);
        Ok(())
    }

    /// One lane of an SSE arithmetic operation.
    fn lane<F: crate::float::Format>(op: Op, a: u64, b: u64, env: Env) -> (u64, Flags) {
        match op {
            Op::ADDPS | Op::ADDSS | Op::ADDPD | Op::ADDSD => binary::add::<F>(a, b, env),
            Op::SUBPS | Op::SUBSS | Op::SUBPD | Op::SUBSD => binary::sub::<F>(a, b, env),
            Op::MULPS | Op::MULSS | Op::MULPD | Op::MULSD => binary::mul::<F>(a, b, env),
            Op::DIVPS | Op::DIVSS | Op::DIVPD | Op::DIVSD => binary::div::<F>(a, b, env),
            Op::MINPS | Op::MINSS | Op::MINPD | Op::MINSD => binary::min::<F>(a, b, env),
            Op::MAXPS | Op::MAXSS | Op::MAXPD | Op::MAXSD => binary::max::<F>(a, b, env),
            // The square root is unary: the *source* is the operand and the
            // destination lane is overwritten, which is why `a` goes unused.
            _ => binary::sqrt::<F>(b, env),
        }
    }

    // -----------------------------------------------------------------
    // FXSAVE and FXRSTOR
    // -----------------------------------------------------------------

    /// `FXSAVE`: the whole floating-point and SIMD state in 512 bytes.
    ///
    /// The layout is *Intel SDM* volume 2's `FXSAVE` table. Two details that
    /// a reader of the diagram alone gets wrong are called out where they
    /// happen: the tag word is **abridged** to one bit per register, and the
    /// instruction and data pointers are eight bytes each with `REX.W` and
    /// four bytes plus a selector without it.
    fn fxsave(&mut self, f: &Fields) -> Ex<()> {
        let (sr, off) = self.ea();
        // The area must be sixteen-byte aligned, for the same reason
        // `MOVAPS`'s operand must be.
        self.require_aligned(off)?;
        let wide = f.rex_w();
        let x = self.state.x87;
        self.write_mem(sr, off, 2, u64::from(x.control))?;
        self.write_mem(sr, off.wrapping_add(2), 2, u64::from(x.status))?;
        self.write_mem(sr, off.wrapping_add(4), 1, u64::from(x.abridged_tag()))?;
        self.write_mem(sr, off.wrapping_add(5), 1, 0)?;
        self.write_mem(sr, off.wrapping_add(6), 2, u64::from(x.last_op))?;
        if wide {
            self.write_mem(sr, off.wrapping_add(8), 8, x.last_ip)?;
            self.write_mem(sr, off.wrapping_add(16), 8, x.last_dp)?;
        } else {
            self.write_mem(sr, off.wrapping_add(8), 4, x.last_ip & 0xffff_ffff)?;
            self.write_mem(sr, off.wrapping_add(12), 2, u64::from(x.last_cs))?;
            self.write_mem(sr, off.wrapping_add(14), 2, 0)?;
            self.write_mem(sr, off.wrapping_add(16), 4, x.last_dp & 0xffff_ffff)?;
            self.write_mem(sr, off.wrapping_add(20), 2, u64::from(x.last_ds))?;
            self.write_mem(sr, off.wrapping_add(22), 2, 0)?;
        }
        self.write_mem(sr, off.wrapping_add(24), 4, u64::from(self.state.sse.mxcsr))?;
        self.write_mem(sr, off.wrapping_add(28), 4, u64::from(mxcsr::SUPPORTED))?;
        // The eight registers, in stack order, each padded from ten bytes to
        // sixteen.
        for i in 0..8u8 {
            let at = off.wrapping_add(32) + u64::from(i) * 16;
            let value = self.state.x87.raw(i);
            self.write_f80(sr, at, value)?;
            self.write_mem(sr, at.wrapping_add(10), 2, 0)?;
            self.write_mem(sr, at.wrapping_add(12), 4, 0)?;
        }
        // The sixteen `XMM` registers. All sixteen are written even outside
        // 64-bit mode, where the top eight are inaccessible: the area is 512
        // bytes either way, and a partial write would leave a restore reading
        // whatever was there.
        for i in 0..16u8 {
            let at = off.wrapping_add(160) + u64::from(i) * 16;
            let v = self.state.sse.get(i);
            self.write_xmm_mem(sr, at, v)?;
        }
        Ok(())
    }

    /// `FXRSTOR`.
    fn fxrstor(&mut self, f: &Fields) -> Ex<()> {
        let (sr, off) = self.ea();
        self.require_aligned(off)?;
        let wide = f.rex_w();
        let control = self.read_mem(sr, off, 2)? as u16;
        let status = self.read_mem(sr, off.wrapping_add(2), 2)? as u16;
        let tag = self.read_mem(sr, off.wrapping_add(4), 1)? as u8;
        let last_op = self.read_mem(sr, off.wrapping_add(6), 2)? as u16;
        let (ip, cs, dp, ds) = if wide {
            let ip = self.read_mem(sr, off.wrapping_add(8), 8)?;
            let dp = self.read_mem(sr, off.wrapping_add(16), 8)?;
            (ip, 0u16, dp, 0u16)
        } else {
            let ip = self.read_mem(sr, off.wrapping_add(8), 4)?;
            let cs = self.read_mem(sr, off.wrapping_add(12), 2)? as u16;
            let dp = self.read_mem(sr, off.wrapping_add(16), 4)?;
            let ds = self.read_mem(sr, off.wrapping_add(20), 2)? as u16;
            (ip, cs, dp, ds)
        };
        let mxcsr_value = self.read_mem(sr, off.wrapping_add(24), 4)? as u32;
        if mxcsr_value & !mxcsr::WRITABLE != 0 {
            // The same rule `LDMXCSR` follows: a reserved bit is a `#GP`,
            // because a restore that quietly dropped it would hide the bug in
            // whatever wrote the area.
            return Err(Fault::gp(0));
        }
        let mut regs = [F80::ZERO; 8];
        for (i, slot) in regs.iter_mut().enumerate() {
            *slot = self.read_f80(sr, off.wrapping_add(32) + (i as u64) * 16)?;
        }
        let mut xmm = [[0u64; 2]; 16];
        for (i, slot) in xmm.iter_mut().enumerate() {
            *slot = self.read_xmm_mem(sr, off.wrapping_add(160) + (i as u64) * 16)?;
        }
        let x = &mut self.state.x87;
        x.control = control;
        x.status = status;
        x.last_op = last_op;
        x.last_ip = ip;
        x.last_cs = cs;
        x.last_dp = dp;
        x.last_ds = ds;
        // The registers were saved in stack order and `TOP` came back with the
        // status word, so they go back through the same rotation.
        for (i, value) in regs.into_iter().enumerate() {
            let p = x.phys(i as u8);
            x.regs[p as usize] = value;
        }
        // The abridged tag word says only which registers are occupied; the
        // four-state tag is recomputed from what is in them, which is what
        // makes the abridgement lossless. It has to happen *after* both the
        // status word and the register images are back, because bit `i`
        // describes `ST(i)` and `TOP` decides which register that is.
        x.set_abridged_tag(tag);
        self.state.sse.mxcsr = mxcsr_value;
        self.state.sse.xmm = xmm;
        self.x87_resummarise();
        Ok(())
    }

    // -----------------------------------------------------------------
    // The two integer instructions that came with this family
    // -----------------------------------------------------------------

    /// `CMPXCHG8B`, and `CMPXCHG16B` with `REX.W`.
    ///
    /// `EDX:EAX` against a memory operand of twice the register width: equal
    /// means `ECX:EBX` is stored and `ZF` set, unequal means the memory value
    /// is loaded into `EDX:EAX` and `ZF` cleared. The two halves are separate
    /// bus transfers here — one core with one bus has nothing to interleave
    /// with them, so the `LOCK` prefix this instruction almost always carries
    /// is decoded and ignored exactly as it is everywhere else.
    ///
    /// *Intel SDM* volume 2, `CMPXCHG8B/CMPXCHG16B`.
    pub(super) fn cmpxchg8b(&mut self, f: &Fields) -> Ex<()> {
        if !self.cfg.features.cx8 {
            return Err(Fault::bare(VEC_UD));
        }
        let wide = f.rex_w();
        if wide && !self.sixty_four() {
            // `CMPXCHG16B` exists only in 64-bit mode; there is no `REX` to
            // ask for it anywhere else.
            return Err(Fault::bare(VEC_UD));
        }
        let half = if wide { 8u8 } else { 4 };
        let (sr, off) = self.ea();
        if wide {
            // The 128-bit form requires a sixteen-byte-aligned operand, and
            // says so with `#GP(0)` rather than by splitting the access.
            self.require_aligned(off)?;
        }
        let lo = self.read_mem(sr, off, half)?;
        let hi = self.read_mem(sr, off.wrapping_add(u64::from(half)), half)?;
        let width = u32::from(half) * 8;
        let mask = if width >= 64 {
            u64::MAX
        } else {
            (1u64 << width) - 1
        };
        let want_lo = self.state.regs.read(0, half, false) & mask;
        let want_hi = self.state.regs.read(2, half, false) & mask;
        if lo == want_lo && hi == want_hi {
            let new_lo = self.state.regs.read(3, half, false) & mask;
            let new_hi = self.state.regs.read(1, half, false) & mask;
            self.write_mem(sr, off, half, new_lo)?;
            self.write_mem(sr, off.wrapping_add(u64::from(half)), half, new_hi)?;
            self.set_flag(flags::ZF, true);
        } else {
            // The comparison failed, so the *memory* value is loaded — which
            // is what makes a compare-and-exchange loop terminate.
            self.state.regs.write(0, half, false, lo);
            self.state.regs.write(2, half, false, hi);
            self.set_flag(flags::ZF, false);
        }
        Ok(())
    }

    /// `WAIT`/`FWAIT`: take a pending unmasked exception now.
    ///
    /// The instruction exists for exactly this: an 8087 signalled its
    /// exceptions asynchronously, so a program that wanted to see them at a
    /// known point put an `FWAIT` there. On a 486 and later the same
    /// instruction is the synchronisation point for `#MF`.
    pub(super) fn fwait(&mut self) -> Ex<()> {
        // `CR0.MP` says a coprocessor is present, and with `CR0.TS` set that
        // pair is what makes `WAIT` participate in lazy state switching
        // (*Intel SDM* volume 3 §2.5).
        if self.state.sys.cr0 & (cr0::MP | cr0::TS) == (cr0::MP | cr0::TS) {
            return Err(Fault::bare(VEC_NM));
        }
        if self.x87_pending_fault() {
            return Err(Fault::bare(VEC_MF));
        }
        Ok(())
    }
}
