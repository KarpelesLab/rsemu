//! The x87 floating-point unit and the SSE register file, as *state*.
//!
//! The arithmetic itself lives in [`crate::float`] and nothing here duplicates
//! it: this module holds the registers, the control and status words, the tag
//! word, `MXCSR`, and the rules that turn a set of [`Flags`] into the
//! processor's visible reaction. Splitting it that way is what keeps the
//! guarantee `float`'s module documentation makes — there is no host `f32` or
//! `f64` anywhere on the path from a guest instruction to a guest result, so
//! two hosts compute the same bits.
//!
//! # The stack is the hard part
//!
//! x87's eight registers are a **rotating stack**, not a file. `ST(0)` is
//! whichever physical register `FSW.TOP` names, `ST(i)` is `(TOP + i) mod 8`,
//! and every physical register additionally carries two tag bits saying
//! whether it holds anything at all (*Intel SDM* volume 1 §8.1.7). Three
//! things follow that a naive "array of eight numbers" model gets wrong, and
//! each is modelled here:
//!
//! * **An empty register is not a zero.** Reading one is a stack *underflow*:
//!   `IE` and `SF` are set, `C1` is cleared to say which direction, and the
//!   masked response delivers the QNaN indefinite rather than `+0.0`
//!   (§8.5.1.1).
//! * **Pushing onto a full stack is an overflow**, with the same two flags and
//!   `C1` **set**. `TOP` still decrements and the destination is still written
//!   with the indefinite, which is why a program that overflows once keeps
//!   producing indefinites rather than recovering.
//! * `FXCH` swaps the *contents and the tags*, so the two registers keep their
//!   emptiness with their values.
//!
//! # Exceptions are deferred
//!
//! An unmasked x87 exception does not fault on the instruction that caused it.
//! The instruction sets `ES` and `B` in the status word and completes; the
//! **next** floating-point instruction — or an `FWAIT` — takes `#MF`
//! (§8.7, and volume 3 §6.15's `#MF` entry). That is why `FNSTSW`, `FNSTCW`,
//! `FNCLEX`, `FNINIT`, `FNSTENV` and `FNSAVE` exist in a "no-wait" form at
//! all: they are the instructions a handler needs to run *before* the pending
//! exception is allowed to fire.
//!
//! SSE is the opposite and simpler: `#XM` is raised by the instruction itself,
//! and only if `CR4.OSXMMEXCPT` says the operating system has an entry point
//! for it (volume 1 §11.5.2.2).
//!
//! # Sources
//!
//! *Intel SDM* volume 1 chapters 8 (the x87 unit: the register stack, the
//! control, status and tag words, the exception model), 10 (SSE and `MXCSR`)
//! and 11 (SSE2); volume 2 for `FXSAVE`'s memory image; volume 3 §2.5 for
//! `CR4.OSFXSR` and table 9-1 for the reset values. No copyleft emulator or
//! soft-float library was consulted (`CLAUDE.md`, provenance).

use crate::float::x87::{F80, Precision};
use crate::float::{Env, Flags, Round};

// ---------------------------------------------------------------------------
// The control word
// ---------------------------------------------------------------------------

/// The x87 control word's fields (*Intel SDM* volume 1 §8.1.5).
pub mod cw {
    /// Invalid-operation mask.
    pub const IM: u16 = 1 << 0;
    /// Denormal-operand mask.
    pub const DM: u16 = 1 << 1;
    /// Zero-divide mask.
    pub const ZM: u16 = 1 << 2;
    /// Overflow mask.
    pub const OM: u16 = 1 << 3;
    /// Underflow mask.
    pub const UM: u16 = 1 << 4;
    /// Precision (inexact) mask.
    pub const PM: u16 = 1 << 5;
    /// Every exception mask, in the same bit order
    /// [`Flags::to_x87_status`](crate::float::Flags::to_x87_status) uses —
    /// which is the whole reason the two can be compared directly.
    pub const MASKS: u16 = IM | DM | ZM | OM | UM | PM;
    /// Precision control, bits 9-8.
    pub const PC: u16 = 0x0300;
    /// How far to shift [`PC`] down.
    pub const PC_SHIFT: u32 = 8;
    /// Rounding control, bits 11-10.
    pub const RC: u16 = 0x0c00;
    /// How far to shift [`RC`] down.
    pub const RC_SHIFT: u32 = 10;
    /// Infinity control, bit 12. Meaningless from the 387 on — it selected the
    /// 287's projective infinities — but it has storage, and software reads it
    /// back.
    pub const IC: u16 = 1 << 12;

    /// What `FNINIT` loads: every exception masked, 64-bit precision, round to
    /// nearest (§8.1.5).
    pub const RESET: u16 = 0x037f;
}

// ---------------------------------------------------------------------------
// The status word
// ---------------------------------------------------------------------------

/// The x87 status word's fields (*Intel SDM* volume 1 §8.1.3).
pub mod sw {
    /// Invalid operation.
    pub const IE: u16 = 1 << 0;
    /// Denormal operand.
    pub const DE: u16 = 1 << 1;
    /// Zero divide.
    pub const ZE: u16 = 1 << 2;
    /// Overflow.
    pub const OE: u16 = 1 << 3;
    /// Underflow.
    pub const UE: u16 = 1 << 4;
    /// Precision.
    pub const PE: u16 = 1 << 5;
    /// Stack fault: the invalid operation was a stack overflow or underflow
    /// rather than an arithmetic one. `C1` says which.
    pub const SF: u16 = 1 << 6;
    /// Exception summary: some unmasked exception is pending.
    pub const ES: u16 = 1 << 7;
    /// Condition code 0.
    pub const C0: u16 = 1 << 8;
    /// Condition code 1 — also the stack-fault direction and the
    /// rounded-up indicator.
    pub const C1: u16 = 1 << 9;
    /// Condition code 2.
    pub const C2: u16 = 1 << 10;
    /// The top-of-stack pointer, bits 13-11.
    pub const TOP: u16 = 0x3800;
    /// How far to shift [`TOP`] down.
    pub const TOP_SHIFT: u32 = 11;
    /// Condition code 3.
    pub const C3: u16 = 1 << 14;
    /// Busy. On a 387 and later it is a copy of [`ES`].
    pub const B: u16 = 1 << 15;

    /// The six exception bits, in
    /// [`Flags::to_x87_status`](crate::float::Flags::to_x87_status)'s order.
    pub const EXCEPTIONS: u16 = IE | DE | ZE | OE | UE | PE;
    /// The four condition codes.
    pub const CONDITION: u16 = C0 | C1 | C2 | C3;
}

// ---------------------------------------------------------------------------
// The tag word
// ---------------------------------------------------------------------------

/// What one physical register's two tag bits say (*Intel SDM* volume 1
/// §8.1.7).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum Tag {
    /// A normal or subnormal number.
    Valid,
    /// A zero of either sign.
    Zero,
    /// A NaN, an infinity, or one of the unsupported encodings.
    Special,
    /// Nothing at all. **Not** a zero: reading one is a stack underflow.
    Empty,
}

impl Tag {
    /// The two-bit encoding.
    #[must_use]
    pub const fn bits(self) -> u16 {
        match self {
            Tag::Valid => 0,
            Tag::Zero => 1,
            Tag::Special => 2,
            Tag::Empty => 3,
        }
    }

    /// Decode two bits.
    #[must_use]
    pub const fn from_bits(bits: u16) -> Tag {
        match bits & 3 {
            0 => Tag::Valid,
            1 => Tag::Zero,
            2 => Tag::Special,
            _ => Tag::Empty,
        }
    }

    /// The tag a freshly written value earns.
    ///
    /// The processor recomputes it on every write rather than trusting what
    /// was there, which is what makes `FLD` of a NaN show `Special` without
    /// anything having to say so.
    ///
    /// Read off the **encoding** rather than through
    /// [`float::x87::classify`](crate::float::x87::classify), because the two
    /// answer different questions. `Special` is §8.1.7's "invalid, infinity
    /// **or denormal**", so a subnormal is `Special` even though it is an
    /// ordinary IEEE value; and a *pseudo*-denormal is `Special` too, even
    /// though `classify` reads it as the normal number it encodes — the tag
    /// describes the bits, not the value.
    #[must_use]
    pub const fn of(value: F80) -> Tag {
        // The exponent field is zero for a zero, a subnormal and a
        // pseudo-denormal; all ones for an infinity, a NaN and the two
        // pseudo-forms; and a clear integer bit anywhere else is an unnormal.
        // Only what is left over is `Valid`.
        if value.exp_field() == 0 {
            if value.sig == 0 {
                return Tag::Zero;
            }
            return Tag::Special;
        }
        if value.exp_field() == 0x7fff || value.sig & (1 << 63) == 0 {
            return Tag::Special;
        }
        Tag::Valid
    }
}

// ---------------------------------------------------------------------------
// The register stack
// ---------------------------------------------------------------------------

/// The x87 register stack and its three words.
///
/// The registers are held in **physical** order, indexed 0-7 as the tag word
/// and `FXSAVE`'s image number them; `ST(i)` is resolved through
/// [`X87::phys`]. Storing them rotated instead would make every save format
/// wrong in a way that only shows up in a guest that context-switches.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct X87 {
    /// The eight data registers, in physical order.
    pub regs: [F80; 8],
    /// The control word.
    pub control: u16,
    /// The status word, `TOP` included.
    pub status: u16,
    /// The tag word: two bits per physical register.
    pub tag: u16,
    /// The last instruction pointer the unit recorded, for `FNSTENV`.
    pub last_ip: u64,
    /// The code selector that went with it.
    pub last_cs: u16,
    /// The last data pointer.
    pub last_dp: u64,
    /// The data selector that went with it.
    pub last_ds: u16,
    /// The last opcode, eleven bits: the low three of the escape byte and all
    /// eight of the ModRM byte (*Intel SDM* volume 1 §8.1.8).
    pub last_op: u16,
}

impl Default for X87 {
    fn default() -> X87 {
        X87::new()
    }
}

impl X87 {
    /// The unit as `FNINIT` leaves it, which is also its power-on state.
    ///
    /// Every register empty, every exception masked, extended precision, round
    /// to nearest (*Intel SDM* volume 1 §8.1.5 and volume 3 table 9-1).
    #[must_use]
    pub const fn new() -> X87 {
        X87 {
            regs: [F80::ZERO; 8],
            control: cw::RESET,
            status: 0,
            // All ones: every register empty.
            tag: 0xffff,
            last_ip: 0,
            last_cs: 0,
            last_dp: 0,
            last_ds: 0,
            last_op: 0,
        }
    }

    /// Re-initialise, as `FNINIT` and `FINIT` do.
    pub const fn init(&mut self) {
        *self = X87::new();
    }

    /// The top-of-stack pointer.
    #[inline]
    #[must_use]
    pub const fn top(&self) -> u8 {
        ((self.status & sw::TOP) >> sw::TOP_SHIFT) as u8
    }

    /// Move the top-of-stack pointer.
    #[inline]
    pub const fn set_top(&mut self, top: u8) {
        self.status = (self.status & !sw::TOP) | (((top & 7) as u16) << sw::TOP_SHIFT);
    }

    /// Which physical register `ST(i)` is.
    #[inline]
    #[must_use]
    pub const fn phys(&self, i: u8) -> u8 {
        (self.top() + i) & 7
    }

    /// The tag of a **physical** register.
    #[inline]
    #[must_use]
    pub const fn tag_at(&self, phys: u8) -> Tag {
        Tag::from_bits(self.tag >> (2 * (phys as u32 & 7)))
    }

    /// Set the tag of a physical register.
    #[inline]
    pub const fn set_tag_at(&mut self, phys: u8, tag: Tag) {
        let shift = 2 * (phys as u32 & 7);
        self.tag = (self.tag & !(3 << shift)) | (tag.bits() << shift);
    }

    /// Whether `ST(i)` holds anything.
    #[inline]
    #[must_use]
    pub const fn occupied(&self, i: u8) -> bool {
        !matches!(self.tag_at(self.phys(i)), Tag::Empty)
    }

    /// Read `ST(i)` without checking whether it is there.
    #[inline]
    #[must_use]
    pub const fn raw(&self, i: u8) -> F80 {
        self.regs[self.phys(i) as usize]
    }

    /// Write `ST(i)` and recompute its tag.
    #[inline]
    pub fn set(&mut self, i: u8, value: F80) {
        let p = self.phys(i);
        self.regs[p as usize] = value;
        self.set_tag_at(p, Tag::of(value));
    }

    /// Mark `ST(i)` empty, as `FFREE` does.
    #[inline]
    pub const fn free(&mut self, i: u8) {
        let p = self.phys(i);
        self.set_tag_at(p, Tag::Empty);
    }

    /// Discard the top of the stack: mark it empty, then increment `TOP`.
    ///
    /// The order matters — the register that becomes empty is the one being
    /// left behind, not the one being uncovered.
    pub const fn pop(&mut self) {
        let p = self.phys(0);
        self.set_tag_at(p, Tag::Empty);
        self.set_top((self.top() + 1) & 7);
    }

    /// Decrement `TOP`, uncovering a new `ST(0)`, without writing it.
    #[inline]
    pub const fn dec_top(&mut self) {
        self.set_top((self.top() + 7) & 7);
    }

    /// Increment `TOP` without changing any tag, as `FINCSTP` does.
    ///
    /// Deliberately **not** [`X87::pop`]: `FINCSTP` leaves the tag word alone,
    /// so the register it steps over stays occupied and reappears after eight
    /// of them (*Intel SDM* volume 2, `FINCSTP`).
    #[inline]
    pub const fn inc_top(&mut self) {
        self.set_top((self.top() + 1) & 7);
    }

    /// The rounding-control field as a rounding direction.
    #[inline]
    #[must_use]
    pub const fn round(&self) -> Round {
        Round::from_x86_rc(((self.control & cw::RC) >> cw::RC_SHIFT) as u32)
    }

    /// The precision-control field.
    ///
    /// `01` is **reserved** (*Intel SDM* volume 1 §8.1.5.2 assigns only 00,
    /// 10 and 11), and the manual does not say what a processor does with it.
    /// This treats it as extended precision, which is what the field's reset
    /// value is; the choice is marked here rather than hidden, in the same way
    /// `float`'s `Propagate::LargerSignificand` marks the tie the manual
    /// leaves open.
    #[inline]
    #[must_use]
    pub const fn precision(&self) -> Precision {
        match Precision::from_pc(((self.control & cw::PC) >> cw::PC_SHIFT) as u32) {
            Some(p) => p,
            None => Precision::Extended,
        }
    }

    /// The arithmetic environment this control word describes.
    ///
    /// x87 has no flush-to-zero and no denormals-are-zero: the 80-bit exponent
    /// range is wide enough that Intel never added either, which is why
    /// [`Env::X87`] is used unmodified but for its rounding direction.
    #[inline]
    #[must_use]
    pub const fn env(&self) -> Env {
        Env::X87.round(self.round())
    }

    /// Which exceptions are **unmasked** in the control word.
    #[inline]
    #[must_use]
    pub const fn unmasked(&self) -> u16 {
        !self.control & cw::MASKS
    }

    /// Record a set of exceptions and report whether any of them is unmasked.
    ///
    /// The sticky bits are set either way — a masked exception is still
    /// recorded, which is what makes `FNSTSW` after a long computation useful
    /// — and `ES`/`B` are set only when one was unmasked, because those two
    /// are what make the *next* instruction take `#MF`.
    pub const fn raise(&mut self, flags: Flags) -> bool {
        let bits = (flags.to_x87_status() as u16) & sw::EXCEPTIONS;
        self.status |= bits;
        let unmasked = bits & self.unmasked();
        if unmasked != 0 {
            self.status |= sw::ES | sw::B;
        }
        unmasked != 0
    }

    /// Record a stack fault — an underflow or an overflow — and report whether
    /// `IE` is unmasked.
    ///
    /// `overflow` sets `C1` and an underflow clears it: §8.5.1.1's rule, and
    /// the only way a handler can tell the two apart.
    pub const fn stack_fault(&mut self, overflow: bool) -> bool {
        self.status |= sw::SF;
        if overflow {
            self.status |= sw::C1;
        } else {
            self.status &= !sw::C1;
        }
        self.raise(Flags::INVALID)
    }

    /// Clear the exception bits, as `FNCLEX` does. The condition codes and
    /// `TOP` are untouched.
    pub const fn clear_exceptions(&mut self) {
        self.status &= !(sw::EXCEPTIONS | sw::SF | sw::ES | sw::B);
    }

    /// Whether an unmasked exception is waiting to be delivered.
    #[inline]
    #[must_use]
    pub const fn pending(&self) -> bool {
        self.status & sw::ES != 0
    }

    /// Set the four condition codes at once.
    pub const fn set_condition(&mut self, c0: bool, c1: bool, c2: bool, c3: bool) {
        let mut s = self.status & !sw::CONDITION;
        if c0 {
            s |= sw::C0;
        }
        if c1 {
            s |= sw::C1;
        }
        if c2 {
            s |= sw::C2;
        }
        if c3 {
            s |= sw::C3;
        }
        self.status = s;
    }

    /// The abridged tag word `FXSAVE` writes: one bit per register, set when
    /// it is **not** empty.
    ///
    /// Indexed by **physical** register, matching the full tag word this
    /// abridges — §8.1.7's figure numbers it `TAG(7)`..`TAG(0)` for `R7`..`R0`
    /// — while the eight register *images* beside it in the `FXSAVE` area are
    /// in `ST(0)`..`ST(7)` order. Mixing the two conventions in one image
    /// looks wrong and is not: `FNSAVE` already does exactly that, with a
    /// physical tag word in its environment and stack-ordered registers after
    /// it, and software that converts between the two formats is written
    /// against that pairing.
    ///
    /// **This is a decision, and the alternative is defensible.** Some
    /// statements of the abridgement name the registers `STj` rather than
    /// `Rj`, which would make the bits stack-relative. Nothing here can tell
    /// the difference — the rotation cancels, so `FXSAVE` and `FXRSTOR`
    /// round-trip either way — and the divergence is visible only to a guest
    /// that reads or synthesises the byte itself, which is what a `ptrace`
    /// register fetch or a signal-frame conversion does. Recorded here rather
    /// than left to be discovered.
    ///
    /// Lossy on purpose: the four-state tag is recomputable from the register
    /// contents, and [`X87::set_abridged_tag`] does exactly that.
    #[must_use]
    pub const fn abridged_tag(&self) -> u8 {
        let mut out = 0u8;
        let mut i = 0u8;
        while i < 8 {
            if !matches!(self.tag_at(i), Tag::Empty) {
                out |= 1 << i;
            }
            i += 1;
        }
        out
    }

    /// Rebuild the full tag word from an abridged one and the register
    /// contents, as `FXRSTOR` does.
    ///
    /// The register file must already be restored, because the four-state tag
    /// is recomputed from what is in it. `TOP` need not be: the bits are
    /// physical, so this does not consult it — see [`X87::abridged_tag`] for
    /// why, and for what would change if that reading is wrong.
    pub const fn set_abridged_tag(&mut self, bits: u8) {
        let mut i = 0u8;
        while i < 8 {
            let tag = if bits & (1 << i) == 0 {
                Tag::Empty
            } else {
                Tag::of(self.regs[i as usize])
            };
            self.set_tag_at(i, tag);
            i += 1;
        }
    }
}

// ---------------------------------------------------------------------------
// MXCSR and the SSE register file
// ---------------------------------------------------------------------------

/// `MXCSR`'s fields (*Intel SDM* volume 1 §10.2.3).
pub mod mxcsr {
    /// Invalid operation.
    pub const IE: u32 = 1 << 0;
    /// Denormal operand.
    pub const DE: u32 = 1 << 1;
    /// Zero divide.
    pub const ZE: u32 = 1 << 2;
    /// Overflow.
    pub const OE: u32 = 1 << 3;
    /// Underflow.
    pub const UE: u32 = 1 << 4;
    /// Precision.
    pub const PE: u32 = 1 << 5;
    /// Denormals are zeros: a subnormal *source* is replaced by a zero of the
    /// same sign, and the denormal-operand exception it would have raised is
    /// suppressed (§10.2.3.4).
    pub const DAZ: u32 = 1 << 6;
    /// Invalid-operation mask.
    pub const IM: u32 = 1 << 7;
    /// Denormal-operand mask.
    pub const DM: u32 = 1 << 8;
    /// Zero-divide mask.
    pub const ZM: u32 = 1 << 9;
    /// Overflow mask.
    pub const OM: u32 = 1 << 10;
    /// Underflow mask.
    pub const UM: u32 = 1 << 11;
    /// Precision mask.
    pub const PM: u32 = 1 << 12;
    /// Rounding control, bits 14-13.
    pub const RC: u32 = 0x6000;
    /// How far to shift [`RC`] down.
    pub const RC_SHIFT: u32 = 13;
    /// Flush to zero: a tiny *result* becomes a zero of the same sign.
    pub const FTZ: u32 = 1 << 15;

    /// The six sticky flags, in
    /// [`Flags::to_mxcsr`](crate::float::Flags::to_mxcsr)'s order.
    pub const EXCEPTIONS: u32 = IE | DE | ZE | OE | UE | PE;
    /// The six masks.
    pub const MASKS: u32 = IM | DM | ZM | OM | UM | PM;
    /// How far the flags have to move to line up with the masks.
    pub const MASK_SHIFT: u32 = 7;

    /// Every bit that has storage. A write with anything else set raises
    /// `#GP(0)` — a guest probes for future extensions exactly that way, and
    /// silently accepting one would tell it a lie.
    pub const WRITABLE: u32 = EXCEPTIONS | DAZ | MASKS | RC | FTZ;

    /// The reset value: nothing raised, everything masked, round to nearest
    /// (*Intel SDM* volume 3 table 9-1 — `0x1f80`).
    pub const RESET: u32 = MASKS;

    /// The `MXCSR_MASK` an `FXSAVE` image reports: which bits `LDMXCSR` will
    /// accept.
    pub const SUPPORTED: u32 = WRITABLE;
}

/// The sixteen SSE registers and `MXCSR`.
///
/// A register is two `u64`s rather than a `u128` because every access to one
/// is a pair of 64-bit bus transfers — guest memory is addressed by byte
/// offset and transferred at a width the bus understands, and a 128-bit
/// integer would only have to be taken apart again.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Sse {
    /// `XMM0` through `XMM15`, each as `[low, high]`.
    pub xmm: [[u64; 2]; 16],
    /// The control and status register.
    pub mxcsr: u32,
}

impl Default for Sse {
    fn default() -> Sse {
        Sse::new()
    }
}

impl Sse {
    /// The reset state: every register zero, `MXCSR` at `0x1f80`.
    #[must_use]
    pub const fn new() -> Sse {
        Sse {
            xmm: [[0; 2]; 16],
            mxcsr: mxcsr::RESET,
        }
    }

    /// The low 64 bits of a register.
    #[inline]
    #[must_use]
    pub const fn low(&self, index: u8) -> u64 {
        self.xmm[(index & 15) as usize][0]
    }

    /// The high 64 bits of a register.
    #[inline]
    #[must_use]
    pub const fn high(&self, index: u8) -> u64 {
        self.xmm[(index & 15) as usize][1]
    }

    /// Replace the low 64 bits, leaving the high half alone — which is what
    /// every scalar operation does, and the reason `MOVSS xmm,xmm` and
    /// `MOVSS xmm,m32` differ.
    #[inline]
    pub const fn set_low(&mut self, index: u8, value: u64) {
        self.xmm[(index & 15) as usize][0] = value;
    }

    /// Replace the high 64 bits.
    #[inline]
    pub const fn set_high(&mut self, index: u8, value: u64) {
        self.xmm[(index & 15) as usize][1] = value;
    }

    /// Replace the whole register.
    #[inline]
    pub const fn set(&mut self, index: u8, value: [u64; 2]) {
        self.xmm[(index & 15) as usize] = value;
    }

    /// The whole register.
    #[inline]
    #[must_use]
    pub const fn get(&self, index: u8) -> [u64; 2] {
        self.xmm[(index & 15) as usize]
    }

    /// The rounding direction `MXCSR.RC` selects.
    #[inline]
    #[must_use]
    pub const fn round(&self) -> Round {
        Round::from_x86_rc((self.mxcsr & mxcsr::RC) >> mxcsr::RC_SHIFT)
    }

    /// The arithmetic environment `MXCSR` describes.
    ///
    /// `DAZ` and `FTZ` are exactly [`Env::daz`] and [`Env::ftz`] — that is what
    /// the profiles in `float` are *for*, and it is why nothing here has to
    /// re-implement flush-to-zero.
    #[inline]
    #[must_use]
    pub const fn env(&self) -> Env {
        Env::X86_SSE
            .round(self.round())
            .daz(self.mxcsr & mxcsr::DAZ != 0)
            .ftz(self.mxcsr & mxcsr::FTZ != 0)
    }

    /// Which exceptions are unmasked, in the *flag* bit positions.
    #[inline]
    #[must_use]
    pub const fn unmasked(&self) -> u32 {
        (!self.mxcsr >> mxcsr::MASK_SHIFT) & mxcsr::EXCEPTIONS
    }

    /// Record a set of exceptions and report whether any was unmasked.
    ///
    /// The sticky flag goes up either way, and that is load-bearing rather
    /// than tidy: `#XM` carries **no error code**, so `MXCSR` is the only
    /// channel by which the handler can learn which of the six fired
    /// (*Intel SDM* volume 1 §11.5.2.1 — the flags are set when their
    /// condition occurs — and §11.5.3, where the handler reads them back to
    /// classify the trap). A handler that found them all clear would decide
    /// the trap was spurious and return, and the faulting instruction would
    /// re-execute for ever.
    ///
    /// Unlike x87 this is not deferred: the caller raises `#XM` on the spot,
    /// and only if `CR4.OSXMMEXCPT` is set.
    pub const fn raise(&mut self, flags: Flags) -> bool {
        let bits = flags.to_mxcsr() & mxcsr::EXCEPTIONS;
        self.mxcsr |= bits;
        bits & self.unmasked() != 0
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_fresh_stack_is_entirely_empty() {
        // SDM volume 1 §8.1.5: `FNINIT` leaves the tag word all ones, which is
        // eight empty registers — not eight zeros.
        let f = X87::new();
        assert_eq!(f.tag, 0xffff);
        for i in 0..8 {
            assert_eq!(f.tag_at(i), Tag::Empty);
            assert!(!f.occupied(i));
        }
        assert_eq!(f.control, 0x037f);
        assert_eq!(f.top(), 0);
    }

    #[test]
    fn st_i_rotates_with_top() {
        // §8.1.7: ST(i) is physical register (TOP + i) mod 8.
        let mut f = X87::new();
        f.set_top(5);
        assert_eq!(f.phys(0), 5);
        assert_eq!(f.phys(3), 0);
        assert_eq!(f.phys(4), 1);
    }

    #[test]
    fn a_zero_tags_as_zero_and_a_nan_as_special() {
        let mut f = X87::new();
        f.set(0, F80::ZERO);
        assert_eq!(f.tag_at(f.phys(0)), Tag::Zero);
        f.set(0, F80::INDEFINITE);
        assert_eq!(f.tag_at(f.phys(0)), Tag::Special);
        f.set(0, F80::new(0x3fff, 1 << 63));
        assert_eq!(f.tag_at(f.phys(0)), Tag::Valid);
    }

    #[test]
    fn pop_frees_the_register_it_leaves_behind() {
        let mut f = X87::new();
        f.dec_top();
        f.set(0, F80::ZERO);
        let was = f.phys(0);
        f.pop();
        assert_eq!(f.tag_at(was), Tag::Empty);
        assert_eq!(f.top(), 0);
    }

    #[test]
    fn incstp_is_not_a_pop() {
        // SDM volume 2, `FINCSTP`: "this instruction does not change the tag
        // word" — the register it steps over stays occupied.
        let mut f = X87::new();
        f.dec_top();
        f.set(0, F80::ZERO);
        let was = f.phys(0);
        f.inc_top();
        assert_eq!(f.tag_at(was), Tag::Zero);
    }

    #[test]
    fn a_masked_exception_is_sticky_and_does_not_summarise() {
        let mut f = X87::new();
        assert!(!f.raise(Flags::INVALID | Flags::INEXACT));
        assert_eq!(f.status & sw::EXCEPTIONS, sw::IE | sw::PE);
        assert_eq!(f.status & sw::ES, 0);
        assert!(!f.pending());
    }

    #[test]
    fn an_unmasked_exception_sets_the_summary_and_the_busy_bit() {
        let mut f = X87::new();
        f.control &= !cw::IM;
        assert!(f.raise(Flags::INVALID));
        assert!(f.pending());
        assert_ne!(f.status & sw::B, 0);
        f.clear_exceptions();
        assert!(!f.pending());
        assert_eq!(f.status & sw::B, 0);
    }

    #[test]
    fn a_stack_fault_says_which_direction_it_was() {
        // §8.5.1.1: C1 is set for an overflow and cleared for an underflow.
        let mut f = X87::new();
        f.stack_fault(true);
        assert_ne!(f.status & sw::C1, 0);
        assert_ne!(f.status & sw::SF, 0);
        f.status &= !(sw::C1 | sw::SF);
        f.stack_fault(false);
        assert_eq!(f.status & sw::C1, 0);
        assert_ne!(f.status & sw::SF, 0);
    }

    #[test]
    fn precision_control_selects_the_three_precisions() {
        // §8.1.5.2. The reserved encoding 01 is a documented choice rather
        // than a measurement — see `X87::precision`.
        let mut f = X87::new();
        for (bits, want) in [
            (0u16, Precision::Single),
            (1, Precision::Extended),
            (2, Precision::Double),
            (3, Precision::Extended),
        ] {
            f.control = (f.control & !cw::PC) | (bits << cw::PC_SHIFT);
            assert_eq!(f.precision(), want, "PC = {bits:02b}");
        }
    }

    #[test]
    fn rounding_control_maps_onto_the_four_directions() {
        let mut f = X87::new();
        for (bits, want) in [
            (0u16, Round::TiesEven),
            (1, Round::TowardNegative),
            (2, Round::TowardPositive),
            (3, Round::TowardZero),
        ] {
            f.control = (f.control & !cw::RC) | (bits << cw::RC_SHIFT);
            assert_eq!(f.round(), want);
            assert_eq!(f.env().round, want);
        }
    }

    #[test]
    fn the_abridged_tag_word_round_trips_through_a_save() {
        // `TOP` is 6, so a physical-indexed abridgement and a stack-relative
        // one disagree — which is what makes this a test of the convention
        // rather than of the bit twiddling. The bits are physical and the
        // images are stack-ordered, exactly as `FNSAVE` pairs them.
        let mut f = X87::new();
        f.dec_top();
        f.set(0, F80::ZERO);
        f.dec_top();
        f.set(0, F80::INDEFINITE);
        assert_eq!(f.top(), 6);
        let bits = f.abridged_tag();
        assert_eq!(bits, 0b1100_0000, "R6 and R7 are occupied, not R0 and R1");

        // Restore the way `FXRSTOR` does: the eight images in `ST(i)` order
        // through the restored `TOP`, then the tags.
        let images: [F80; 8] = core::array::from_fn(|i| f.raw(i as u8));
        let mut g = X87::new();
        g.set_top(f.top());
        for (i, value) in images.into_iter().enumerate() {
            let p = g.phys(i as u8);
            g.regs[p as usize] = value;
        }
        g.set_abridged_tag(bits);
        assert_eq!(g.tag, f.tag);
        assert_eq!(g.regs, f.regs);
    }

    #[test]
    fn a_denormal_tags_special_and_so_does_a_pseudo_denormal() {
        // §8.1.7: `Special` is "invalid, infinity **or denormal**", so a
        // subnormal does not tag `Valid` however ordinary a value it is. A
        // pseudo-denormal — exponent field zero with the integer bit set —
        // tags `Special` too, even though it encodes a normal number: the tag
        // describes the bits.
        let mut f = X87::new();
        f.set(0, F80::new(0, 1));
        assert_eq!(f.tag_at(f.phys(0)), Tag::Special, "the smallest subnormal");
        f.set(0, F80::new(0, 1 << 63));
        assert_eq!(f.tag_at(f.phys(0)), Tag::Special, "a pseudo-denormal");
        // An unnormal: a non-zero exponent with the integer bit clear.
        f.set(0, F80::new(0x4000, 1));
        assert_eq!(f.tag_at(f.phys(0)), Tag::Special, "an unnormal");
        f.set(0, F80::new(0x4000, 1 << 63));
        assert_eq!(f.tag_at(f.phys(0)), Tag::Valid, "and an ordinary 2.0");
    }

    #[test]
    fn an_unmasked_sse_exception_still_records_which_one_it_was() {
        // `#XM` carries no error code, so `MXCSR` is the only way the handler
        // can tell an overflow from a divide by zero (§11.5.3). A flag left
        // clear here reads as "spurious" and the guest spins.
        let mut s = Sse::new();
        s.mxcsr &= !mxcsr::ZM;
        assert!(s.raise(Flags::DIV_BY_ZERO));
        assert_ne!(s.mxcsr & mxcsr::ZE, 0, "the cause is legible");
        // And the classification a handler performs — unmasked flags that are
        // set — names exactly that exception.
        let cause = (!s.mxcsr >> mxcsr::MASK_SHIFT) & s.mxcsr & mxcsr::EXCEPTIONS;
        assert_eq!(cause, mxcsr::ZE);
    }

    #[test]
    fn mxcsr_resets_with_everything_masked() {
        let s = Sse::new();
        assert_eq!(s.mxcsr, 0x1f80);
        assert_eq!(s.round(), Round::TiesEven);
        assert_eq!(s.unmasked(), 0);
    }

    #[test]
    fn mxcsr_flush_and_denormal_bits_reach_the_environment() {
        let mut s = Sse::new();
        s.mxcsr |= mxcsr::FTZ | mxcsr::DAZ;
        let env = s.env();
        assert!(env.flush_outputs);
        assert!(env.subnormal_inputs.flushes());
        assert!(!env.subnormal_inputs.reports());
    }

    #[test]
    fn a_masked_sse_exception_is_recorded_and_does_not_fault() {
        let mut s = Sse::new();
        assert!(!s.raise(Flags::INEXACT));
        assert_ne!(s.mxcsr & mxcsr::PE, 0);
        assert_eq!(s.unmasked(), 0);
    }

    #[test]
    fn a_scalar_write_leaves_the_high_half_alone() {
        let mut s = Sse::new();
        s.set(1, [0x1111_1111_1111_1111, 0x2222_2222_2222_2222]);
        s.set_low(1, 0xdead);
        assert_eq!(s.get(1), [0xdead, 0x2222_2222_2222_2222]);
    }
}
