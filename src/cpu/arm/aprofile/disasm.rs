//! The disassembler, built on the same decoders the interpreter uses.
//!
//! Not a side project: gdb's `disassemble`, the monitor's single-step display
//! and any trace log need it, and CLAUDE.md forbids describing the instruction
//! set twice. Everything here calls [`isa::decode`](super::isa::decode),
//! [`thumb::decode`](super::thumb::decode) or, on a part with Thumb-2,
//! [`thumb2::T32::decode_for`]; there is no second table.
//!
//! A Thumb listing for a Thumb-2 part is [`Listed::Thumb2`]: instructions of
//! either width, printed in UAL, with each instruction inside an `IT` block
//! carrying the block's condition. The listing follows `ITSTATE` from the
//! `IT` instructions it passes, the way `objdump` does; one that starts in
//! the middle of a block cannot know it is there.
//!
//! What this layer adds over those two is *address context*: a bare
//! [`Decoded`] prints a branch as `B +40` because it does not know where it
//! came from, while a [`Listed`] knows and can answer
//! [`Listed::branch_target`].
//!
//! ```
//! use rsemu::cpu::arm::aprofile::disasm::disassemble_arm;
//!
//! // e3a0_0042: MOV r0, #0x42
//! let d = disassemble_arm(0x8000, 0xe3a0_0042);
//! assert_eq!(format!("{d}"), "00008000: e3a00042  MOV r0, #66");
//! ```

use alloc::vec::Vec;
use core::fmt;

use super::arch::Arch;
use super::isa::{Cond, Decoded, Insn};
use super::thumb::Thumb;
use super::thumb2::{self, T32};
use crate::cpu::arm::t32::ItState;

/// Why a listing has a hole in it.
///
/// A listing does not stop at a hole and does not shorten: it carries the hole
/// as a value and keeps going, because "the first ten instructions were fine"
/// is exactly the case a monitor is looking at. Which *kind* of hole it is
/// matters to whoever reads the listing — an address that no page table maps is
/// a different problem from one that maps to nothing on the bus, and telling
/// them apart is the difference between "the guest has not mapped this yet" and
/// "this board has no memory there".
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Missing {
    /// The page tables map nothing at that virtual address.
    ///
    /// Only ever produced by a listing that was given virtual addresses —
    /// [`Arm::disassemble_virtual`](super::Arm::disassemble_virtual).
    Untranslated,
    /// Nothing answered at that physical address: the bus refused the read.
    Unmapped,
}

impl fmt::Display for Missing {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Missing::Untranslated => f.write_str("not mapped"),
            Missing::Unmapped => f.write_str("no memory"),
        }
    }
}

/// One instruction at a known address, in whichever state it was decoded in.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[non_exhaustive]
pub enum Listed {
    /// A 32-bit ARM instruction.
    Arm {
        /// Where it lives.
        addr: u32,
        /// The decoded form; its `raw` field holds the encoding.
        insn: Decoded,
    },
    /// A 16-bit Thumb instruction.
    Thumb {
        /// Where it lives.
        addr: u32,
        /// The raw halfword.
        raw: u16,
        /// The decoded form.
        insn: Thumb,
    },
    /// A Thumb instruction of either width, on a part with Thumb-2.
    Thumb2 {
        /// Where it lives.
        addr: u32,
        /// The decoded form, 16- or 32-bit.
        insn: T32,
        /// The `IT` block condition it executes under, if the listing
        /// placed it inside one.
        it: Option<Cond>,
    },
    /// Not every byte was readable — an unmapped page, or the end of a buffer.
    ///
    /// A monitor disassembling to the end of a region gets this rather than a
    /// panic or a decode of invented zeroes.
    Unreadable {
        /// Where the read failed.
        addr: u32,
        /// Which instruction set was being decoded.
        thumb: bool,
        /// What was missing.
        why: Missing,
    },
}

impl Listed {
    /// The address this instruction lives at.
    #[must_use]
    pub const fn addr(&self) -> u32 {
        match *self {
            Listed::Arm { addr, .. }
            | Listed::Thumb { addr, .. }
            | Listed::Thumb2 { addr, .. }
            | Listed::Unreadable { addr, .. } => addr,
        }
    }

    /// How many bytes it occupies.
    #[must_use]
    pub const fn byte_len(&self) -> u32 {
        match *self {
            Listed::Arm { .. } => 4,
            Listed::Thumb { .. } => 2,
            Listed::Thumb2 { insn, .. } => insn.byte_len(),
            // Advance by the width that was being attempted, so a listing
            // walks past a hole rather than sitting on it.
            Listed::Unreadable { thumb, .. } => {
                if thumb {
                    2
                } else {
                    4
                }
            }
        }
    }

    /// Whether this was decoded as Thumb.
    #[must_use]
    pub const fn is_thumb(&self) -> bool {
        matches!(
            self,
            Listed::Thumb { .. } | Listed::Thumb2 { .. } | Listed::Unreadable { thumb: true, .. }
        )
    }

    /// The absolute address a branch goes to, where that is a constant.
    ///
    /// `None` for a register branch (`BX`, `MOV pc, r0`) and for everything
    /// that is not a branch: those depend on state a static listing does not
    /// have.
    #[must_use]
    pub const fn branch_target(&self) -> Option<u32> {
        match *self {
            // ARM reads R15 as the instruction plus eight; Thumb, plus four.
            Listed::Arm { addr, insn } => match insn.insn {
                Insn::Branch { offset, .. } | Insn::BlxImm { offset } => {
                    Some(addr.wrapping_add(8).wrapping_add(offset as u32))
                }
                _ => None,
            },
            Listed::Thumb { addr, insn, .. } => match insn {
                Thumb::Branch { offset } | Thumb::BranchCond { offset, .. } => {
                    Some(addr.wrapping_add(4).wrapping_add(offset as u32))
                }
                _ => None,
            },
            Listed::Thumb2 { addr, insn, .. } => insn.branch_target(addr),
            Listed::Unreadable { .. } => None,
        }
    }
}

impl fmt::Display for Listed {
    /// `addr: encoding  MNEMONIC operands`, with a resolved branch target
    /// where there is one.
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match *self {
            Listed::Arm { addr, insn } => {
                write!(f, "{addr:08x}: {:08x}  ", insn.raw)?;
                match self.branch_target() {
                    Some(target) => match insn.insn {
                        Insn::Branch { link, .. } => {
                            let l = if link { "L" } else { "" };
                            write!(f, "B{l}{} 0x{target:08x}", insn.cond)
                        }
                        _ => write!(f, "BLX 0x{target:08x}"),
                    },
                    None => {
                        // A VFP word prints as VFP whatever the part: the
                        // listing has no configuration, and binutils does the
                        // same. `cond == 0b1111` is not VFP.
                        #[cfg(feature = "cpu-arm-aprofile-vfp")]
                        if insn.raw >> 28 != 0xf
                            && let Some(v) = super::vfpisa::decode(insn.raw)
                        {
                            return write!(f, "{}", v.display(insn.cond));
                        }
                        write!(f, "{insn}")
                    }
                }
            }
            Listed::Thumb { addr, raw, insn } => {
                write!(f, "{addr:08x}: {raw:04x}      ")?;
                match (self.branch_target(), insn) {
                    (Some(target), Thumb::BranchCond { cond, .. }) => {
                        write!(f, "B{cond} 0x{target:08x}")
                    }
                    (Some(target), _) => write!(f, "B 0x{target:08x}"),
                    (None, _) => write!(f, "{insn}"),
                }
            }
            Listed::Thumb2 { addr, insn, it } => {
                match insn {
                    T32::Narrow { raw, .. } => write!(f, "{addr:08x}: {raw:04x}      ")?,
                    T32::Wide(d) => {
                        write!(f, "{addr:08x}: {:04x} {:04x} ", d.raw >> 16, d.raw & 0xffff)?
                    }
                }
                write!(f, "{}", insn.ual(Some(addr), it))
            }
            Listed::Unreadable { addr, why, .. } => write!(f, "{addr:08x}: ??        <{why}>"),
        }
    }
}

/// Disassemble one ARM word at a known address, as an ARMv5TE part reads it.
#[must_use]
pub fn disassemble_arm(addr: u32, word: u32) -> Listed {
    disassemble_arm_for(&Arch::V5TE, addr, word)
}

/// Disassemble one ARM word at a known address, as the part `arch`
/// describes reads it — so a word that part would take as Undefined lists as
/// `UNDEFINED`, and the listing tells the truth about what will execute.
#[must_use]
pub fn disassemble_arm_for(arch: &Arch, addr: u32, word: u32) -> Listed {
    Listed::Arm {
        addr,
        insn: super::isa::decode_for(arch, word),
    }
}

/// Disassemble one Thumb halfword at a known address.
#[must_use]
pub fn disassemble_thumb(addr: u32, half: u16) -> Listed {
    Listed::Thumb {
        addr,
        raw: half,
        insn: super::thumb::decode(half),
    }
}

/// Disassemble one Thumb instruction for the part `arch` describes, given
/// its first halfword and the one after it (ignored unless the first starts
/// a 32-bit instruction on a Thumb-2 part), outside any `IT` block.
///
/// On a part without Thumb-2 this is [`disassemble_thumb`].
#[must_use]
pub fn disassemble_thumb_for(arch: &Arch, addr: u32, hw1: u16, hw2: u16) -> Listed {
    if arch.ext.thumb2 {
        Listed::Thumb2 {
            addr,
            insn: T32::decode_for(arch, hw1, hw2),
            it: None,
        }
    } else {
        disassemble_thumb(addr, hw1)
    }
}

/// Disassemble `count` instructions from `addr`, reading bytes through `read`.
///
/// `read` reports why a byte cannot be read, which becomes a
/// [`Listed::Unreadable`] rather than a decode of invented data. **The result
/// always holds `count` entries**: a hole is a value, the walk steps over it by
/// the width it was attempting, and the listing carries on. Returning fewer
/// entries would leave the caller unable to tell "the region ended" from "we
/// stopped for a reason we did not write down".
///
/// Bytes are assembled little-endian, which is the byte order of every ARM
/// instruction stream — even a big-endian ARMv5 fetches its instructions
/// little-endian unless the whole memory system is BE-32, in which case the
/// caller's `read` is what compensates.
pub fn disassemble_run(
    addr: u32,
    count: usize,
    thumb: bool,
    read: impl FnMut(u32) -> Result<u8, Missing>,
) -> Vec<Listed> {
    disassemble_run_for(&Arch::V5TE, addr, count, thumb, read)
}

/// [`disassemble_run`] for the part `arch` describes.
pub fn disassemble_run_for(
    arch: &Arch,
    addr: u32,
    count: usize,
    thumb: bool,
    mut read: impl FnMut(u32) -> Result<u8, Missing>,
) -> Vec<Listed> {
    let mut out = Vec::with_capacity(count);
    let mut at = addr;
    let mut it = ItState::NONE;
    for _ in 0..count {
        if thumb && arch.ext.thumb2 {
            let listed = thumb2_one(arch, at, &mut read, &mut it);
            at = at.wrapping_add(listed.byte_len());
            out.push(listed);
            continue;
        }
        let width = if thumb { 2 } else { 4 };
        let mut word = 0u32;
        // The first reason wins: an instruction straddling the end of a mapped
        // page is missing for the reason its *first* absent byte gives.
        let mut missing = None;
        for i in 0..width {
            match read(at.wrapping_add(i)) {
                Ok(byte) => word |= u32::from(byte) << (8 * i),
                Err(why) => missing = missing.or(Some(why)),
            }
        }
        let listed = if let Some(why) = missing {
            Listed::Unreadable {
                addr: at,
                thumb,
                why,
            }
        } else if thumb {
            disassemble_thumb(at, word as u16)
        } else {
            disassemble_arm_for(arch, at, word)
        };
        at = at.wrapping_add(listed.byte_len());
        out.push(listed);
    }
    out
}

/// One Thumb-2 listing entry: read a halfword, and a second when the first
/// starts a 32-bit instruction, then walk `ITSTATE` past it.
fn thumb2_one(
    arch: &Arch,
    at: u32,
    read: &mut impl FnMut(u32) -> Result<u8, Missing>,
    it: &mut ItState,
) -> Listed {
    let mut half = |a: u32| -> Result<u16, Missing> {
        Ok(u16::from(read(a)?) | (u16::from(read(a.wrapping_add(1))?) << 8))
    };
    let hw1 = match half(at) {
        Ok(h) => h,
        Err(why) => {
            return Listed::Unreadable {
                addr: at,
                thumb: true,
                why,
            };
        }
    };
    let hw2 = if thumb2::is_32bit(hw1) {
        match half(at.wrapping_add(2)) {
            Ok(h) => h,
            Err(why) => {
                return Listed::Unreadable {
                    addr: at,
                    thumb: true,
                    why,
                };
            }
        }
    } else {
        0
    };
    let insn = T32::decode_for(arch, hw1, hw2);
    let cond = it.in_block().then(|| Cond(it.cond()));
    *it = match insn.it_state() {
        Some(next) => next,
        None if it.in_block() => it.advance(),
        None => ItState::NONE,
    };
    Listed::Thumb2 {
        addr: at,
        insn,
        it: cond,
    }
}
