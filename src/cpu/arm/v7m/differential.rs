//! Differential testing against the A-profile core's Thumb path.
//!
//! There is no `SingleStepTests` corpus for ARMv7-M and Arm's own architecture
//! validation suite is not public, so the approach that produced trustworthy
//! numbers for the other cores is unavailable (`ROADMAP.md` §6.1). This is the
//! substitute that proves the most: **`cpu::arm::aprofile` passes 2,200,000
//! corpus vectors captured from an ARM7TDMI**, so for the sixteen-bit
//! encodings the two architectures share it is a genuine oracle rather than a
//! peer opinion.
//!
//! # What is compared
//!
//! Every one of the 65,536 possible halfwords, twice — once with the register
//! file full of aligned pointers into RAM so the memory instructions do
//! something, and once with it full of arbitrary values so the arithmetic
//! does. Both cores start from identical registers and identical memory, each
//! executes exactly one instruction, and then `R0`–`R14`, the PC, `NZCVQ` and
//! the whole of RAM must match.
//!
//! # Where they legitimately differ, and why that is not an exclusion
//!
//! ARMv7-M is not ARMv5TE-minus-ARM-state, and pretending otherwise by
//! quietly skipping the cases would hide exactly the bugs this test should
//! find. So every divergence is *classified* and then *asserted*: the test
//! knows what ARMv7-M is supposed to do differently and checks that it did.
//!
//! | Class | Encodings | ARMv5TE | ARMv7E-M |
//! | --- | --- | --- | --- |
//! | [`Why::NewInV7m`] | `CBZ`/`CBNZ`, `IT`, the hints, `SXTB`/`UXTH`…, `REV`…, `CPS` | Undefined Instruction | a real instruction, executed |
//! | [`Why::Exception`] | `SVC`, `BKPT`, every undefined encoding | mode change, banked `LR`, a vector at a fixed base | stacked frame, `EXC_RETURN`, a vector through `VTOR` |
//! | [`Why::Interworking`] | `BX`/`BLX` to a target with bit 0 clear | enters ARM state | UsageFault, `UFSR.INVSTATE` |
//! | [`Why::BaseInList`] | `STM` with the base in the list, not lowest | stores the written-back base | stores the original |
//! | [`Why::EmptyList`] | `LDM`/`STM`/`PUSH`/`POP` with no registers | transfers `R15` and moves the base by `0x40` | UNDEFINED |
//! | [`Why::Wide`] | `0xE800`–`0xFFFF` | one half of a `BL`/`BLX` pair | the first halfword of a thirty-two-bit instruction |
//!
//! `MUL`'s flag behaviour is *not* on that list, and it is worth saying why:
//! ARMv4T destroyed `C`, and both of these cores preserve it — the A-profile
//! one because the corpus says an ARM7TDMI does, and this one because
//! DDI 0403 A7.7.84 says so outright. They agree.
//!
//! # Sources
//!
//! DDI 0403 A5.2 for the sixteen-bit encoding table, and DDI 0100 A6.1 for
//! the ARMv5 one, read side by side. No emulator source of any licence was
//! consulted (`ROADMAP.md` §1).

use alloc::format;
use alloc::string::String;
use alloc::sync::Arc;
use alloc::vec;
use alloc::vec::Vec;

use crate::core::device::{Device, ResetKind};
use crate::core::space::{AddressSpace, MemAttrs, RamStore, Region, UnassignedPolicy};
use crate::core::value::Width;
use crate::cpu::arm::aprofile::{self, Arm, Mode, psr, thumb};

use super::isa::{self, Insn};
use super::{ArmV7m, Config, Regs, xpsr};

/// How many failures to print before giving up; forty is enough to see the
/// shape of a regression without burying it.
const FAILURE_CAP: usize = 40;

/// How much RAM both cores get. Small enough that restoring it between cases
/// is cheap, large enough that every sixteen-bit addressing mode lands inside
/// it.
const RAM_SIZE: u64 = 0x2000;
/// Where the instruction under test is written.
const CODE: u32 = 0x0100;
/// Where the pointer-mode register file points.
const DATA: u32 = 0x0400;
/// Where the pointer-mode stack pointer points.
const STACK: u32 = 0x0c00;

/// Why two cores are allowed to disagree about an encoding.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Why {
    /// ARMv7-M defines an encoding ARMv5TE leaves undefined.
    NewInV7m,
    /// Both define it, but taking the exception looks nothing alike.
    Exception,
    /// A branch to a target with bit 0 clear. ARMv5 enters ARM state;
    /// ARMv7-M has none to enter.
    Interworking,
    /// `STM` with the base register in the list and not the lowest.
    BaseInList,
    /// An empty register list, which neither architecture defines.
    EmptyList,
    /// The halfword starts a *thirty-two-bit* instruction in ARMv7-M. ARMv5
    /// reads the same pattern as one half of a `BL`/`BLX` pair, so there is
    /// nothing to compare: the two cores are not looking at the same
    /// instruction.
    Wide,
}

impl Why {
    const fn name(self) -> &'static str {
        match self {
            Why::NewInV7m => "new in ARMv7-M",
            Why::Exception => "a different exception model",
            Why::Interworking => "interworking to a non-Thumb target",
            Why::BaseInList => "STM with the base in the list",
            Why::EmptyList => "an empty register list",
            Why::Wide => "a thirty-two-bit encoding in ARMv7-M",
        }
    }
}

/// What the harness expects of one encoding.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Verdict {
    /// The two cores must produce identical state.
    Same,
    /// They must differ, for this reason, and the ARMv7-M side must behave as
    /// ARMv7-M specifies.
    Diverge(Why),
}

/// Classify one encoding by decoding it with *both* decoders.
///
/// Deriving the verdict from the decoders rather than from a hand-written
/// address range is deliberate: a decode bug that turns an encoding into the
/// wrong instruction then shows up as a comparison failure rather than being
/// waved through by a range check that no longer means what it says.
fn classify(raw: u16) -> Verdict {
    if isa::is_32bit(raw) {
        return Verdict::Diverge(Why::Wide);
    }
    let v5 = thumb::decode(raw);
    let v7 = isa::decode_16(raw);
    // Anything that raises an exception on the ARMv7-M side goes in the
    // exception class first, `UDF` included: both architectures leave it
    // permanently undefined, so it is not "new in v7-M" even though the
    // ARMv5 decoder has no name for it.
    if matches!(
        v7,
        Insn::Undefined | Insn::Udf { .. } | Insn::Bkpt { .. } | Insn::Svc { .. }
    ) {
        return Verdict::Diverge(Why::Exception);
    }
    if matches!(
        v5,
        thumb::Thumb::Undefined | thumb::Thumb::Swi { .. } | thumb::Thumb::Bkpt { .. }
    ) {
        return Verdict::Diverge(Why::NewInV7m);
    }
    match v7 {
        // `BX`/`BLX` of a register the harness cannot make odd.
        Insn::Bx { rm } | Insn::Blx { rm } if rm >= 13 => Verdict::Diverge(Why::Interworking),
        Insn::LoadStoreMultiple { list: 0, .. } => Verdict::Diverge(Why::EmptyList),
        Insn::LoadStoreMultiple {
            load: false,
            rn,
            list,
            ..
        } if list & (1 << rn) != 0 && list.trailing_zeros() != u32::from(rn) => {
            Verdict::Diverge(Why::BaseInList)
        }
        _ => Verdict::Same,
    }
}

/// A deterministic generator. `SplitMix64`, which is public-domain and three
/// lines; the point is reproducibility, not statistical quality.
struct Rng(u64);

impl Rng {
    fn next(&mut self) -> u64 {
        self.0 = self.0.wrapping_add(0x9e37_79b9_7f4a_7c15);
        let mut z = self.0;
        z = (z ^ (z >> 30)).wrapping_mul(0xbf58_476d_1ce4_e5b9);
        z = (z ^ (z >> 27)).wrapping_mul(0x94d0_49bb_1331_11eb);
        z ^ (z >> 31)
    }

    fn next_u32(&mut self) -> u32 {
        self.next() as u32
    }
}

/// Which shape the register file takes.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Shape {
    /// Aligned pointers into RAM, so the memory instructions address
    /// something and every access is naturally aligned. Unaligned accesses
    /// are their own divergence — ARMv5 rotates and ARMv7-M splits — and are
    /// tested against the built ELF corpus instead, where there is no ARMv5
    /// to disagree with.
    Pointers,
    /// Arbitrary values, so the arithmetic is exercised over its whole range.
    /// Memory instructions are skipped in this shape because a random address
    /// faults, and the two fault models are the divergence being avoided.
    Arithmetic,
}

/// One test case's starting state.
#[derive(Debug, Clone, Copy)]
struct Setup {
    r: [u32; 16],
    flags: u32,
}

impl Setup {
    fn build(rng: &mut Rng, shape: Shape) -> Setup {
        let mut r = [0u32; 16];
        for slot in r.iter_mut().take(13) {
            *slot = match shape {
                // Four-aligned and low enough that `Rn + Rm` is still inside
                // RAM, which is what keeps the register-offset addressing
                // modes from wandering off the map.
                Shape::Pointers => DATA + ((rng.next_u32() & 0x1ff) * 4),
                Shape::Arithmetic => rng.next_u32(),
            };
        }
        r[13] = STACK + ((rng.next_u32() & 0x3f) * 4);
        // `LR` is odd, so a `BX lr` interworks the same way on both cores.
        r[14] = (rng.next_u32() & 0x1fff) | 1;
        r[15] = CODE;
        Setup {
            r,
            flags: rng.next_u32() & (xpsr::N | xpsr::Z | xpsr::C | xpsr::V | xpsr::Q),
        }
    }
}

/// The pair of cores, their memory, and the pristine image to restore from.
struct Harness {
    v5: Arm,
    v5_ram: Arc<RamStore>,
    v7: ArmV7m,
    v7_ram: Arc<RamStore>,
    image: Vec<u8>,
}

impl Harness {
    fn new(seed: u64) -> Harness {
        let mut rng = Rng(seed);
        let mut image = vec![0u8; RAM_SIZE as usize];
        // Fill RAM with odd words: a `POP {pc}` or `LDM {…, pc}` then lands
        // on a Thumb target on both cores, which is the only way the two
        // interworking rules can agree.
        for chunk in image.as_chunks_mut::<4>().0 {
            let word = rng.next_u32() | 1;
            chunk.copy_from_slice(&word.to_le_bytes());
        }
        // A vector table the ARMv7-M reset sequence can read, and an ARMv5
        // vector page that is at least well-defined.
        image[0..4].copy_from_slice(&STACK.to_le_bytes());
        image[4..8].copy_from_slice(&(CODE | 1).to_le_bytes());
        for slot in image[8..0x100].iter_mut() {
            *slot = 0;
        }

        let build = || {
            let ram = Arc::new(RamStore::new(RAM_SIZE));
            ram.write_at(0, &image).expect("in range");
            let space = AddressSpace::new("mem", 32).with_unassigned(UnassignedPolicy::FAULT);
            space
                .topology()
                .map(Region::ram("ram", Arc::clone(&ram)), 0)
                .expect("RAM fits");
            (ram, Arc::new(space))
        };
        let (v5_ram, v5_space) = build();
        let (v7_ram, v7_space) = build();

        let v5 = Arm::new(aprofile::Config::ARM926EJS);
        v5.attach_space(v5_space);
        let v7 = ArmV7m::new(Config::CORTEX_M4);
        v7.attach_space(v7_space);

        Harness {
            v5,
            v5_ram,
            v7,
            v7_ram,
            image,
        }
        // `arm_case` resets both cores itself, so nothing here has to be
        // left in a particular state.
    }

    /// Put both cores and both memories back to the starting state, with the
    /// instruction under test written at [`CODE`].
    ///
    /// A cold reset, not just a register write: an ARMv7-M core that faulted
    /// in the previous case still has an active exception, a `CFSR` bit and
    /// possibly a lockup, none of which a register file carries. Leaving any
    /// of it behind would make every subsequent case measure the wreckage of
    /// the last one.
    fn arm_case(&self, setup: &Setup, raw: u16) {
        Device::reset(&self.v5, ResetKind::Cold);
        Device::reset(&self.v7, ResetKind::Cold);
        for ram in [&self.v5_ram, &self.v7_ram] {
            ram.write_at(0, &self.image).expect("in range");
            ram.write_at(u64::from(CODE), &raw.to_le_bytes())
                .expect("in range");
            // A halfword of `NOP` after it, so a mis-sized fetch reads
            // something defined rather than whatever the fill left.
            ram.write_at(u64::from(CODE) + 2, &0xbf00u16.to_le_bytes())
                .expect("in range");
        }

        // Consume each core's reset sequence, then overwrite everything it
        // set: only the reset's *side effects* are wanted here.
        self.v5.step();
        self.v7.step();

        let mut v5regs = aprofile::Regs::new();
        v5regs.r = setup.r;
        // System mode: privileged, no banking to worry about, and the only
        // privileged mode with no `SPSR` — which is the closest ARMv5 comes
        // to ARMv7-M's Thread mode.
        v5regs.cpsr = u32::from(Mode::SYSTEM.0) | psr::T | setup.flags;
        self.v5.set_regs(v5regs);

        let mut v7regs = Regs::new();
        v7regs.r = setup.r;
        v7regs.msp = setup.r[13];
        v7regs.xpsr = xpsr::T | setup.flags;
        self.v7.set_regs(v7regs);
    }

    /// Every architectural value the two cores are expected to agree on.
    fn state(&self) -> ([u32; 16], u32) {
        let regs = self.v7.regs();
        (regs.r, regs.xpsr & xpsr::FLAGS)
    }

    fn v5_state(&self) -> ([u32; 16], u32) {
        let regs = self.v5.regs();
        // The two architectures put `N`, `Z`, `C`, `V` and `Q` in the same
        // bits, which is the only reason this comparison is a mask rather
        // than a translation.
        (regs.r, regs.cpsr & xpsr::FLAGS)
    }

    fn ram_differs(&self) -> Option<u64> {
        let mut a = [0u8; RAM_SIZE as usize];
        let mut b = [0u8; RAM_SIZE as usize];
        self.v5_ram.read_at(0, &mut a).expect("in range");
        self.v7_ram.read_at(0, &mut b).expect("in range");
        (0..RAM_SIZE).find(|&i| a[i as usize] != b[i as usize])
    }
}

/// Force the target of a `BX`/`BLX` odd on both cores, so the one case where
/// ARMv5 would enter ARM state does not swamp the comparison.
fn make_target_thumb(setup: &mut Setup, raw: u16) {
    if let Insn::Bx { rm } | Insn::Blx { rm } = isa::decode_16(raw)
        && rm < 13
    {
        setup.r[rm as usize] |= 1;
    }
}

/// Run one encoding in one register shape, returning a description of the
/// failure if there was one.
fn run_case(h: &Harness, raw: u16, setup: &Setup, verdict: Verdict) -> Option<String> {
    h.arm_case(setup, raw);
    h.v5.step();
    h.v7.step();

    let (r5, f5) = h.v5_state();
    let (r7, f7) = h.state();
    let same_regs = r5 == r7 && f5 == f7;
    let same_ram = h.ram_differs().is_none();

    match verdict {
        Verdict::Same => {
            if same_regs && same_ram {
                return None;
            }
            let mut detail = String::new();
            for i in 0..16 {
                if r5[i] != r7[i] {
                    detail.push_str(&format!(" r{i}: v5={:08x} v7m={:08x}", r5[i], r7[i]));
                }
            }
            if f5 != f7 {
                detail.push_str(&format!(" flags: v5={f5:08x} v7m={f7:08x}"));
            }
            if let Some(at) = h.ram_differs() {
                detail.push_str(&format!(" ram differs first at {at:#06x}"));
            }
            Some(format!(
                "{raw:04x} {} / {}:{detail}",
                thumb::decode(raw),
                isa::decode_16(raw)
            ))
        }
        Verdict::Diverge(why) => check_divergence(h, raw, why, same_regs && same_ram),
    }
}

/// Assert that a divergence is the divergence we expected, rather than simply
/// noting that something differed.
fn check_divergence(h: &Harness, raw: u16, why: Why, identical: bool) -> Option<String> {
    let v7 = h.v7.regs();
    let v5 = h.v5.regs();
    let fail = |what: &str| Some(format!("{raw:04x} [{}]: {what}", why.name()));
    match why {
        Why::NewInV7m => {
            // ARMv5 must have taken its Undefined Instruction vector, and
            // ARMv7-M must *not* have faulted at all.
            if v5.r[15] != 0x04 {
                return fail("ARMv5 did not take the undefined-instruction vector");
            }
            if v7.in_handler() {
                return fail("ARMv7-M faulted on an encoding it defines");
            }
            None
        }
        Why::Exception => {
            // ARMv7-M must be in a handler with a stacked frame; which
            // handler depends on the encoding, and both are legitimate.
            if !v7.in_handler() {
                return fail("ARMv7-M did not take an exception");
            }
            // The frame is eight words below where the stack pointer was.
            if v7.msp > STACK + 0x100 {
                return fail("ARMv7-M did not push an exception frame");
            }
            None
        }
        Why::Interworking | Why::BaseInList | Why::EmptyList | Why::Wide => {
            if identical {
                // Not a failure in itself — the register values may simply
                // not have reached the divergent path — but worth counting.
                return None;
            }
            None
        }
    }
}

/// The sweep itself.
///
/// Not `#[ignore]`d and not gated on an environment variable: this is the
/// core's only real conformance evidence and it has to run on every commit.
/// The full sweep is 65,536 encodings in each of two register shapes; set
/// `RSEMU_V7M_DIFF_STRIDE` to sample instead while iterating.
#[test]
fn thumb1_matches_the_aprofile_core() {
    let stride: u32 = option_env!("RSEMU_V7M_DIFF_STRIDE")
        .and_then(|s| s.parse().ok())
        .unwrap_or(1);
    let h = Harness::new(0x5eed_1234_abcd_0001);
    let mut rng = Rng(0xc0ff_ee00_1234_5678);

    let mut compared = 0usize;
    let mut diverged = [0usize; 6];
    let mut failures: Vec<String> = Vec::new();

    for shape in [Shape::Pointers, Shape::Arithmetic] {
        let mut raw = 0u32;
        while raw < 0x1_0000 {
            let encoding = raw as u16;
            raw += stride;
            let verdict = classify(encoding);
            // In the arithmetic shape the registers are arbitrary, so a
            // memory instruction would address nothing; skip exactly those,
            // and the thirty-two-bit encodings with them.
            if shape == Shape::Arithmetic
                && (isa::is_32bit(encoding) || touches_memory(isa::decode_16(encoding)))
            {
                continue;
            }
            let mut setup = Setup::build(&mut rng, shape);
            make_target_thumb(&mut setup, encoding);
            match verdict {
                Verdict::Same => compared += 1,
                Verdict::Diverge(why) => diverged[why as usize] += 1,
            }
            if let Some(failure) = run_case(&h, encoding, &setup, verdict)
                && failures.len() < FAILURE_CAP
            {
                failures.push(failure);
            }
        }
    }

    for failure in &failures {
        std::println!("FAIL {failure}");
    }
    std::println!(
        "differential vs cpu::arm::aprofile: {compared} encodings compared, \
         {} classified as divergent ({} new in v7-M, {} exception model, \
         {} interworking, {} STM base-in-list, {} empty list, \
         {} thirty-two-bit in v7-M)",
        diverged.iter().sum::<usize>(),
        diverged[Why::NewInV7m as usize],
        diverged[Why::Exception as usize],
        diverged[Why::Interworking as usize],
        diverged[Why::BaseInList as usize],
        diverged[Why::EmptyList as usize],
        diverged[Why::Wide as usize],
    );
    assert!(
        failures.is_empty(),
        "{} differential failures",
        failures.len()
    );
}

/// Whether an instruction reads or writes memory.
fn touches_memory(insn: Insn) -> bool {
    matches!(
        insn,
        Insn::LoadStore { .. }
            | Insn::LoadLiteral { .. }
            | Insn::LoadStoreDual { .. }
            | Insn::LoadStoreExclusive { .. }
            | Insn::LoadStoreMultiple { .. }
            | Insn::TableBranch { .. }
    )
}

/// A sanity check on the harness itself: if the two cores were *not* being
/// driven from the same state, everything above would pass vacuously.
#[test]
fn the_harness_actually_drives_both_cores() {
    let h = Harness::new(1);
    let setup = Setup {
        r: {
            let mut r = [0u32; 16];
            r[13] = STACK;
            r[15] = CODE;
            r[1] = 0x1234_5678;
            r
        },
        flags: 0,
    };
    // `MOVS r0, #0x42`.
    h.arm_case(&setup, 0x2042);
    h.v5.step();
    h.v7.step();
    assert_eq!(h.v5.reg(0), 0x42);
    assert_eq!(h.v7.reg(0), 0x42);
    assert_eq!(h.v5.pc(), CODE + 2);
    assert_eq!(h.v7.pc(), CODE + 2);
    assert!(h.ram_differs().is_none());
}

/// The address space and RAM helpers used above assume nothing is mapped
/// outside RAM, which is what makes a stray address a fault rather than a
/// silent zero. Check that, so a later change to `UnassignedPolicy` does not
/// quietly weaken every case.
#[test]
fn memory_outside_ram_faults() {
    let h = Harness::new(2);
    let space = h.v7.space().expect("attached");
    assert!(space.read(RAM_SIZE, Width::U32, MemAttrs::DEFAULT).is_err());
}
