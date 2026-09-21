//! The MC68040's memory management unit.
//!
//! Everything here is M68040UM Section 3, and none of it is the 68030's.
//! The two units share a name, a purpose and almost nothing else — which is
//! why this is a module beside [`mmu`](super::mmu) rather than a set of
//! branches inside it:
//!
//! | | 68030 (MC68030UM §9) | 68040 (M68040UM §3) |
//! | --- | --- | --- |
//! | page size | eight sizes, 256 B to 32 KiB, from `TC`'s **PS** | two, 4 KiB or 8 KiB, from `TC`'s **P** |
//! | table shape | one to five levels, widths from `TC`'s `TIx` | exactly three, 7/7/6 or 7/7/5 bits |
//! | descriptors | six formats, short and long, with limits | three: table, page and indirect, all one long word |
//! | root pointers | 64-bit descriptors, `CRP` and `SRP` | 32-bit addresses, `URP` and `SRP` |
//! | reached by | `PMOVE` over the coprocessor interface | `MOVEC` |
//! | `TTx` | two, function-code base and mask | four, split instruction and data, an **S** field |
//! | ATC | 22 entries, fully associative | 64 entries, four-way set associative |
//! | fault report | the 68020's format `$A`/`$B` frames | format `$7`, thirty words |
//!
//! # What is not modelled
//!
//! - **`MDIS`.** There is no such input here; `TC`'s **E** bit is the switch
//!   (§3.6.2).

/// Translation control register bits (M68040UM Figure 3-4).
///
/// Sixteen bits, of which two exist: "Bits 13–0 are undefined (reserved)".
pub(super) mod tcr {
    /// Bit 15: translation enable.
    pub(in super::super) const E: u16 = 0x8000;
    /// Bit 14: page size — clear for 4 KiB, set for 8 KiB.
    pub(in super::super) const P: u16 = 0x4000;
    /// Every bit the register implements.
    pub(in super::super) const IMPLEMENTED: u16 = E | P;
}

/// Transparent translation register bits (M68040UM Figure 3-5).
///
/// "Bits 12–10, 7, 4, 3, 1, and 0 always read as zero", which leaves the
/// eight-bit base, the eight-bit mask, **E**, the two-bit **S** field, `U1`,
/// `U0`, the two-bit **CM** and **W**.
pub(super) mod ttr {
    /// Bit 15: enable.
    pub(in super::super) const E: u32 = 0x0000_8000;
    /// Bits 14–13: which privilege modes match.
    pub(super) const S_SHIFT: u32 = 13;
    /// Bit 9, bit 8: the user page attributes, echoed to `UPA1`/`UPA0`.
    pub(super) const U1: u32 = 0x0000_0200;
    /// See [`U1`].
    pub(super) const U0: u32 = 0x0000_0100;
    /// Bits 6–5: the cache mode.
    pub(super) const CM_SHIFT: u32 = 5;
    /// Bit 2: write protect.
    pub(in super::super) const W: u32 = 0x0000_0004;
    /// Every bit the register implements.
    pub(in super::super) const IMPLEMENTED: u32 =
        0xffff_0000 | E | (3 << S_SHIFT) | U1 | U0 | (3 << CM_SHIFT) | W;
}

/// MMU status register bits (M68040UM Figure 3-6).
pub(in super::super) mod mmusr {
    /// Bit 11: a transfer error happened during the table search. "If the
    /// B-bit is set, all other bits are zero."
    pub(in super::super) const B: u32 = 0x0000_0800;
    /// Bit 10: the page descriptor's **G** bit.
    pub(in super::super) const G: u32 = 0x0000_0400;
    /// Bit 9, bit 8: the page descriptor's user attributes.
    pub(in super::super) const U1: u32 = 0x0000_0200;
    /// See [`U1`].
    pub(in super::super) const U0: u32 = 0x0000_0100;
    /// Bit 7: supervisor protected.
    pub(in super::super) const S: u32 = 0x0000_0080;
    /// Bits 6–5: the cache mode, copied from the page descriptor.
    pub(in super::super) const CM_SHIFT: u32 = 5;
    /// Bit 4: modified.
    pub(in super::super) const M: u32 = 0x0000_0010;
    /// Bit 2: write protected — set if **W** was set in *any* descriptor the
    /// search read, which "does not indicate that a violation has occurred".
    pub(in super::super) const W: u32 = 0x0000_0004;
    /// Bit 1: a transparent translation register answered.
    pub(in super::super) const T: u32 = 0x0000_0002;
    /// Bit 0: resident.
    pub(in super::super) const R: u32 = 0x0000_0001;
    /// Every bit the register implements. Bit 3 is drawn as `O` in Figure
    /// 3-6 and written as zero (M68000PRM §6, *PTEST* (MC68040)).
    pub(in super::super) const IMPLEMENTED: u32 =
        0xffff_f000 | B | G | U1 | U0 | S | (3 << CM_SHIFT) | M | W | T | R;
}

/// The 68040's memory management registers.
///
/// `Copy` for the same reason the 68030's are: [`State`](super::exec::State)
/// is, and the register file travels by value through `regs()` and the
/// snapshot path.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub(super) struct Regs040 {
    /// Translation control (Figure 3-4).
    pub tcr: u16,
    /// User root pointer (Figure 3-3). Bits 8–0 must be zero.
    pub urp: u32,
    /// Supervisor root pointer.
    pub srp: u32,
    /// `ITT0` and `ITT1` — `IACR0`/`IACR1` on an MC68EC040.
    pub itt: [u32; 2],
    /// `DTT0` and `DTT1` — `DACR0`/`DACR1`.
    pub dtt: [u32; 2],
    /// What `PTEST` last reported (Figure 3-6).
    pub mmusr: u32,
}

impl Regs040 {
    /// A cold start: everything zero.
    pub(super) const RESET: Regs040 = Regs040 {
        tcr: 0,
        urp: 0,
        srp: 0,
        itt: [0; 2],
        dtt: [0; 2],
        mmusr: 0,
    };

    /// What the `RSTI` signal does: "the E-bits of the TCR and TTRs are
    /// cleared, disabling address translation ... A reset of the processor
    /// does not invalidate any entries in the ATCs or alter the page size"
    /// (§3.6.1). **P** surviving reset is the part worth spelling out: the
    /// manual says so twice, in §3.1.2 as well.
    pub(super) const fn reset_pin(&mut self) {
        self.tcr &= !tcr::E;
        self.itt[0] &= !ttr::E;
        self.itt[1] &= !ttr::E;
        self.dtt[0] &= !ttr::E;
        self.dtt[1] &= !ttr::E;
    }

    /// Whether paged translation is switched on.
    #[inline]
    pub(super) const fn enabled(&self) -> bool {
        self.tcr & tcr::E != 0
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn reset_clears_the_enables_and_leaves_the_page_size() {
        let mut regs = Regs040 {
            tcr: tcr::E | tcr::P,
            itt: [ttr::E; 2],
            dtt: [ttr::E; 2],
            ..Regs040::RESET
        };
        regs.reset_pin();
        assert!(!regs.enabled());
        // §3.6.1: "RSTI does not affect the P-bit of the TCR."
        assert_eq!(regs.tcr, tcr::P);
        assert_eq!(regs.itt, [0; 2]);
        assert_eq!(regs.dtt, [0; 2]);
    }
}
