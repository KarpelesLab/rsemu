//! The disassembler, built on the same decoders the interpreter uses.
//!
//! Not a side project: gdb's `disassemble`, the monitor's single-step display
//! and any trace log need it, and CLAUDE.md forbids describing the instruction
//! set twice. Everything here calls [`isa::decode`](super::isa::decode) or
//! [`thumb::decode`](super::thumb::decode); there is no second table.
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

use super::isa::{Decoded, Insn};
use super::thumb::Thumb;

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
            | Listed::Unreadable { addr, .. } => addr,
        }
    }

    /// How many bytes it occupies.
    #[must_use]
    pub const fn byte_len(&self) -> u32 {
        match *self {
            Listed::Arm { .. } => 4,
            Listed::Thumb { .. } => 2,
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
            Listed::Thumb { .. } | Listed::Unreadable { thumb: true, .. }
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
                    None => write!(f, "{insn}"),
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
            Listed::Unreadable { addr, why, .. } => write!(f, "{addr:08x}: ??        <{why}>"),
        }
    }
}

/// Disassemble one ARM word at a known address.
#[must_use]
pub fn disassemble_arm(addr: u32, word: u32) -> Listed {
    Listed::Arm {
        addr,
        insn: super::isa::decode(word),
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
    mut read: impl FnMut(u32) -> Result<u8, Missing>,
) -> Vec<Listed> {
    let mut out = Vec::with_capacity(count);
    let mut at = addr;
    for _ in 0..count {
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
            disassemble_arm(at, word)
        };
        at = at.wrapping_add(listed.byte_len());
        out.push(listed);
    }
    out
}
