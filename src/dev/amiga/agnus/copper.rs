//! The copper: a three-instruction coprocessor that watches the beam and writes
//! custom-chip registers.
//!
//! # The instruction set
//!
//! *Amiga Hardware Reference Manual*, 3rd edition, chapter 2 and Appendix A's
//! `COPINS` entry. Every instruction is two words, `IR1` and `IR2`:
//!
//! ```text
//!           IR1                          IR2
//!   MOVE    0000000 DA8..DA1 0           RD15..RD0
//!   WAIT    VP7..VP0 HP8..HP2 1          BFD VE6..VE0 HE8..HE2 0
//!   SKIP    VP7..VP0 HP8..HP2 1          BFD VE6..VE0 HE8..HE2 1
//! ```
//!
//! * **`MOVE`** writes `IR2` to the register at `DA`. It goes through the
//!   custom-chip bus with [`Origin::copper`](super::super::custom::Origin::copper),
//!   so whether the copper may touch that register — `COPCON`'s danger bit and
//!   Appendix B's `*`/`~` columns — is decided by the bus, not here.
//! * **`WAIT`** holds the copper "until the video beam counters are equal to
//!   (or greater than) the coordinates specified". "A position bit in
//!   instruction word 1 is used in comparing the positions with the actual beam
//!   counters if and only if the corresponding enable bit in instruction word 2
//!   is set to 1." Bit 15 of `IR2` is not an enable: it is `BFD`, and "the V7
//!   comparison cannot be masked". With `BFD` clear the blitter must also have
//!   finished.
//! * **`SKIP`** skips the next instruction if the same comparison holds.
//!
//! The comparison is on the combined 15 bits, vertical above horizontal —
//! "skip if the beam counter is equal to or greater than these combined bits
//! (bits 15 through 1)". Only the low eight bits of the vertical counter take
//! part — "line 256 will appear as a zero in the comparison" — and the low bit
//! of the horizontal counter does not.
//!
//! # Timing
//!
//! "The Copper is a two-cycle processor that requests the bus only during
//! odd-numbered memory cycles", memory cycles being one colour clock each. "The
//! MOVE and SKIP instructions require two memory cycles ... four memory cycle
//! times ... The WAIT instruction requires three memory cycles and six memory
//! cycle times; it takes one extra memory cycle to wake up."
//!
//! So the copper acts on every other count. **Which** parity is the manual's
//! "odd" is fixed here by its own example: a `WAIT` for horizontal `$E2` on a
//! PAL line whose last count is `$E2` must end (chapter 2, *A Copper Loop
//! Example*), which it can only do if the copper acts on even horizontal
//! counts. So it does.
//!
//! One instruction is therefore: fetch `IR1` on one cycle, fetch `IR2` on the
//! next and act on it — a `MOVE`'s write and a `SKIP`'s test happen there — and
//! for a `WAIT`, a third cycle on which the comparison holds, after which the
//! next fetch follows.
//!
//! # Restarts
//!
//! "At the start of each vertical blanking interval, COP1LC is automatically
//! used to start the program counter" (chapter 2, *Location Registers*), and
//! vertical blanking starts at line 0 (chapter 7, *Vertical Blanking
//! Interrupt*). The same paragraph also says "when the end of vertical
//! blanking occurs"; `COPINS` in Appendix A says "at the beginning of each
//! vertical blank time", and that is the reading taken. A strobe of `COPJMP1`
//! or `COPJMP2` reloads the program counter at once and abandons whatever the
//! copper was doing.
//!
//! # A `MOVE` to a register the copper may not write halts it
//!
//! The manual gives the ranges — "Those it cannot affect at all are numbered
//! $00 to $3E inclusive ... from $40 to $7E, are protected by" `CDANG`
//! (chapter 2, *Control Register*) — and says the protection is there so "a
//! runaway Copper (caused by a poorly formed instruction list)" cannot reach
//! the blitter. It does not say what the copper does next. **This is
//! inference from firmware, not from a manual:** it stops, and stays stopped
//! until `COPJMP1`, `COPJMP2` or the next field restarts it.
//!
//! The evidence is two unrelated ROMs that boot on real machines and cannot
//! have if it carried on. Kickstart 1.3's intuition loads its first `View`
//! before any screen exists, so `LoadView` copies `LOFCprList->start` out of a
//! null pointer — the longword at address 4, which is `ExecBase` — into
//! `COP2LC`, and `copinit`'s `COPJMP2` sends the copper into exec's library
//! base. AROS's graphics leaves `COP2LC` at zero and the copper runs the
//! exception vectors. Both "lists" open with `$0000 xxxx`, a `MOVE` to `$000`.
//! A copper that carried on from there went on to write `INTENA` with bits the
//! system needs cleared, and each ROM stopped for good waiting for an
//! interrupt that could no longer come; one that halts on the first word does
//! nothing at all, which is what the real machines evidently do.
//!
//! # Not modelled
//!
//! * **The `COPINS` dummy address.** The manual says the copper "generates"
//!   it on each instruction fetch. It is on the chip's internal register bus
//!   and has no effect anyone can see.

use super::beam::Beam;

/// Where the copper is in its instruction cycle.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Phase {
    /// The next cycle fetches `IR1`.
    Fetch1,
    /// The next cycle fetches `IR2` and acts on the instruction.
    Fetch2,
    /// A `WAIT` is holding; each cycle tests the comparison.
    Wait,
    /// A `MOVE` hit a register the copper may not write; nothing happens until
    /// a restart. See the module documentation.
    Halted,
}

impl Phase {
    /// The snapshot encoding.
    #[must_use]
    pub const fn code(self) -> u8 {
        match self {
            Phase::Fetch1 => 0,
            Phase::Fetch2 => 1,
            Phase::Wait => 2,
            Phase::Halted => 3,
        }
    }

    /// Decode [`code`](Self::code).
    #[must_use]
    pub const fn from_code(code: u8) -> Option<Phase> {
        match code {
            0 => Some(Phase::Fetch1),
            1 => Some(Phase::Fetch2),
            2 => Some(Phase::Wait),
            3 => Some(Phase::Halted),
            _ => None,
        }
    }
}

/// A `MOVE` the copper wants performed: the register offset and the word.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Move {
    /// The destination, as an offset from the custom-chip base.
    pub offset: u16,
    /// The word.
    pub value: u16,
}

/// The copper's registers and its instruction state.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Copper {
    /// `COP1LC`.
    pub cop1lc: u32,
    /// `COP2LC`.
    pub cop2lc: u32,
    /// The program counter.
    pub pc: u32,
    /// What the next cycle does.
    pub phase: Phase,
    /// The first instruction word, once fetched.
    pub ir1: u16,
    /// The second instruction word, once fetched.
    pub ir2: u16,
}

impl Default for Copper {
    fn default() -> Copper {
        Copper::new()
    }
}

impl Copper {
    /// Power-on: both location registers and the program counter zero, about to
    /// fetch.
    #[must_use]
    pub const fn new() -> Copper {
        Copper {
            cop1lc: 0,
            cop2lc: 0,
            pc: 0,
            phase: Phase::Fetch1,
            ir1: 0,
            ir2: 0,
        }
    }

    /// Jump to `COP1LC` (`to_second` false) or `COP2LC`.
    pub fn jump(&mut self, to_second: bool) {
        self.pc = if to_second { self.cop2lc } else { self.cop1lc };
        self.phase = Phase::Fetch1;
    }

    /// One copper cycle.
    ///
    /// `fetch` reads a word of chip RAM; `beam` is the position this cycle is
    /// on; `blitter_busy` is `BBUSY`. Returns the `MOVE` to perform, if this
    /// cycle completed one.
    pub fn cycle(
        &mut self,
        mut fetch: impl FnMut(u32) -> u16,
        beam: &Beam,
        blitter_busy: bool,
    ) -> Option<Move> {
        match self.phase {
            Phase::Fetch1 => {
                self.ir1 = fetch(self.pc);
                self.pc = self.pc.wrapping_add(2);
                self.phase = Phase::Fetch2;
                None
            }
            Phase::Fetch2 => {
                self.ir2 = fetch(self.pc);
                self.pc = self.pc.wrapping_add(2);
                self.phase = Phase::Fetch1;
                if self.ir1 & 1 == 0 {
                    return Some(Move {
                        offset: self.ir1 & 0x01fe,
                        value: self.ir2,
                    });
                }
                if self.ir2 & 1 == 0 {
                    self.phase = Phase::Wait;
                } else if condition(self.ir1, self.ir2, beam, blitter_busy) {
                    self.pc = self.pc.wrapping_add(4);
                }
                None
            }
            Phase::Wait => {
                if condition(self.ir1, self.ir2, beam, blitter_busy) {
                    self.phase = Phase::Fetch1;
                }
                None
            }
            Phase::Halted => None,
        }
    }

    /// Stop until the next restart, because the `MOVE` just returned was to a
    /// register the copper may not write with `danger` in the state it is in.
    ///
    /// Returns whether it stopped. The ranges are chapter 2's: `$00`–`$3E`
    /// never, `$40`–`$7E` only with `CDANG`. An Enhanced Chip Set copper
    /// (`ecs`) has Appendix C's instead: with `CDANG` everything, and without it
    /// everything from `$3E` up (`regs::ecs_copper_may_write`).
    pub fn halt_if_refused(&mut self, offset: u16, danger: bool, ecs: bool) -> bool {
        let refused = if ecs {
            !crate::dev::amiga::regs::ecs_copper_may_write(offset, danger)
        } else {
            offset < 0x40 || (offset < 0x80 && !danger)
        };
        if refused {
            self.phase = Phase::Halted;
        }
        refused
    }

    /// Whether the next cycle will fetch or complete an instruction — work that
    /// cannot be skipped over.
    #[must_use]
    pub const fn fetching(&self) -> bool {
        matches!(self.phase, Phase::Fetch1 | Phase::Fetch2)
    }

    /// Whether a `WAIT` is holding.
    #[must_use]
    pub const fn waiting(&self) -> bool {
        matches!(self.phase, Phase::Wait)
    }

    /// Whether a held `WAIT` needs the blitter to finish as well as the beam.
    #[must_use]
    pub const fn waits_for_blitter(&self) -> bool {
        matches!(self.phase, Phase::Wait) && self.ir2 & 0x8000 == 0
    }
}

/// Whether a `WAIT` or `SKIP` with these instruction words is satisfied.
#[must_use]
#[inline]
pub fn condition(ir1: u16, ir2: u16, beam: &Beam, blitter_busy: bool) -> bool {
    beam_reached(ir1, ir2, beam.vpos, beam.hpos) && (ir2 & 0x8000 != 0 || !blitter_busy)
}

/// The beam half of the comparison, for a position.
#[must_use]
#[inline]
pub fn beam_reached(ir1: u16, ir2: u16, vpos: u16, hpos: u16) -> bool {
    let (position, target) = masked(ir1, ir2, vpos, hpos);
    position >= target
}

/// The position and the target, both masked by `IR2`'s enables.
#[inline]
const fn masked(ir1: u16, ir2: u16, vpos: u16, hpos: u16) -> (u16, u16) {
    let position = ((vpos & 0xff) << 8) | (hpos & 0xfe);
    // VE6..VE0 and HE8..HE2 are bits 14-1; V7 is always compared.
    let mask = (ir2 & 0x7ffe) | 0x8000;
    (position & mask, ir1 & 0xfffe & mask)
}

/// How many counts from `beam` until the first even count on which a `WAIT`
/// with these words would be satisfied by the beam, if that happens before the
/// field ends. The count the beam is on now is not considered.
///
/// Arithmetic over positions and no simulation: the vertical half of the
/// masked comparison is examined a line at a time, and only a line whose
/// vertical half *equals* the target's is scanned count by count.
#[must_use]
pub fn ticks_until_reached(
    ir1: u16,
    ir2: u16,
    beam: &Beam,
    timing: impl Into<super::beam::Timing>,
) -> Option<u64> {
    let t = timing.into();
    let (_, target) = masked(ir1, ir2, 0, 0);
    let mask = (ir2 & 0x7ffe) | 0x8000;
    let target_v = target & 0xff00;
    let field_len = beam.field_len(t);

    // The rest of this line, after the count the beam is on.
    let mut ticks = 0u64;
    let len = beam.line_len(t);
    if beam.vpos < field_len {
        for h in beam.hpos.saturating_add(1)..len {
            ticks += 1;
            if h & 1 == 0 && beam_reached(ir1, ir2, beam.vpos, h) {
                return Some(ticks);
            }
        }
    }
    ticks = beam.ticks_to_line(t);

    let mut lol = beam.lol;
    let mut v = beam.vpos;
    loop {
        v = v.saturating_add(1);
        if v >= field_len {
            return None;
        }
        if t.alternate {
            lol = !lol;
        }
        let len = if t.alternate && lol {
            t.line + 1
        } else {
            t.line
        };
        let position_v = ((v & 0xff) << 8) & mask;
        if position_v > target_v {
            // Count 0 of this line is even and already past the target.
            return Some(ticks);
        }
        if position_v == target_v {
            for h in (0..len).step_by(2) {
                if beam_reached(ir1, ir2, v, h) {
                    return Some(ticks + u64::from(h));
                }
            }
        }
        ticks += u64::from(len);
    }
}

#[cfg(test)]
mod tests {
    use super::super::beam::Standard;
    use super::*;
    use alloc::vec::Vec;

    fn at(vpos: u16, hpos: u16) -> Beam {
        Beam {
            vpos,
            hpos,
            lof: true,
            lol: false,
        }
    }

    #[test]
    fn a_phase_survives_its_snapshot_code_and_a_bad_code_does_not() {
        for phase in [Phase::Fetch1, Phase::Fetch2, Phase::Wait, Phase::Halted] {
            assert_eq!(Phase::from_code(phase.code()), Some(phase));
        }
        assert_eq!(Phase::from_code(4), None);
    }

    #[test]
    fn the_manuals_end_of_list_wait_never_ends() {
        // "$FFFF,$FFFE ... the largest number that will ever appear in the
        // comparison is $FFE2." Line 255 is reached; count $FE never is.
        for v in 0..313 {
            for h in 0..227 {
                assert!(!beam_reached(0xffff, 0xfffe, v, h), "({h},{v})");
            }
        }
        assert_eq!(
            ticks_until_reached(0xffff, 0xfffe, &at(0, 0), Standard::Pal),
            None
        );
    }

    #[test]
    fn line_256_compares_as_line_zero() {
        // $9601,$FF00: wait for line 150, ignore horizontal.
        assert!(beam_reached(0x9601, 0xff00, 150, 0));
        assert!(!beam_reached(0x9601, 0xff00, 149, 226));
        assert!(
            !beam_reached(0x9601, 0xff00, 256 + 3, 0),
            "the vertical comparison is on eight bits"
        );
    }

    #[test]
    fn v7_cannot_be_masked_and_that_is_why_the_manual_needs_two_loops() {
        // "$0F01,$8F00 ; Wait for VP=0xxx1111" — only VE3-VE0 enabled, yet V7
        // still takes part. Past line 128 the masked position is at least $80,
        // which is above $0F: "the interrupt will happen on every scan line".
        assert!(!beam_reached(0x0f01, 0x8f00, 0x0e, 0));
        assert!(beam_reached(0x0f01, 0x8f00, 0x1f, 0));
        assert!(!beam_reached(0x0f01, 0x8f00, 0x10, 0), "$10 masks to $00");
        assert!(beam_reached(0x0f01, 0x8f00, 0x80, 0), "V7 leaks through");
    }

    #[test]
    fn the_horizontal_half_ignores_its_low_bit() {
        assert!(beam_reached(0x00e3, 0x80fe, 0, 0xe2));
        assert!(!beam_reached(0x00e3, 0x80fe, 0, 0xe1));
    }

    #[test]
    fn a_move_takes_two_cycles_and_a_skip_skips_one_instruction() {
        let mem: Vec<u16> = alloc::vec![
            0x0180, 0x0f00, // MOVE COLOR00
            0x6401, 0xff01, // SKIP if VP >= 100
            0x0182, 0x00f0, // MOVE COLOR01 (skipped)
            0x0184, 0x000f, // MOVE COLOR02
        ];
        let fetch = |addr: u32| mem[(addr / 2) as usize];
        let mut c = Copper::new();
        let beam = at(120, 0);
        assert_eq!(c.cycle(fetch, &beam, false), None);
        assert_eq!(
            c.cycle(fetch, &beam, false),
            Some(Move {
                offset: 0x180,
                value: 0x0f00
            })
        );
        assert_eq!(c.cycle(fetch, &beam, false), None);
        assert_eq!(c.cycle(fetch, &beam, false), None, "the SKIP, taken");
        assert_eq!(c.cycle(fetch, &beam, false), None);
        assert_eq!(
            c.cycle(fetch, &beam, false),
            Some(Move {
                offset: 0x184,
                value: 0x000f
            })
        );
    }

    #[test]
    fn a_wait_takes_a_third_cycle_to_wake_and_bfd_waits_for_the_blitter() {
        let mem: Vec<u16> = alloc::vec![0x0001, 0x7ffe, 0x0096, 0x8200];
        let fetch = |addr: u32| mem[(addr / 2) as usize];
        let mut c = Copper::new();
        let beam = at(10, 10);
        c.cycle(fetch, &beam, true);
        c.cycle(fetch, &beam, true);
        assert_eq!(c.phase, Phase::Wait);
        assert!(c.waits_for_blitter(), "BFD is clear");
        c.cycle(fetch, &beam, true);
        assert_eq!(
            c.phase,
            Phase::Wait,
            "the beam is there but the blitter is not"
        );
        c.cycle(fetch, &beam, false);
        assert_eq!(c.phase, Phase::Fetch1, "the wake cycle");
        c.cycle(fetch, &beam, false);
        assert!(c.cycle(fetch, &beam, false).is_some());
    }

    #[test]
    fn looking_ahead_for_a_wait_matches_stepping_the_beam() {
        let cases = [
            (0x9601u16, 0xff00u16),
            (0x2c07, 0xfffe),
            (0x00e3, 0x80fe),
            (0x0f01, 0x8f00),
            (0xffdf, 0xfffe),
            (0x2c41, 0x80fe),
        ];
        for std in [Standard::Pal, Standard::Ntsc] {
            for (ir1, ir2) in cases {
                for start in [at(0, 0), at(20, 100), at(150, 225), at(0x2c, 0x06)] {
                    let expect = {
                        let mut b = start;
                        let mut n = 0u64;
                        loop {
                            let crossing = b.advance(std, false);
                            n += 1;
                            if crossing == super::super::beam::Crossing::Field {
                                break None;
                            }
                            if b.hpos & 1 == 0 && beam_reached(ir1, ir2, b.vpos, b.hpos) {
                                break Some(n);
                            }
                        }
                    };
                    assert_eq!(
                        ticks_until_reached(ir1, ir2, &start, std),
                        expect,
                        "{std:?} {ir1:04x},{ir2:04x} from {start:?}"
                    );
                }
            }
        }
    }
}
