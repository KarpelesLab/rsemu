//! The interface a 6502 core must expose for these runners to drive it.
//!
//! **This file is the contract.** The 6502 core does not exist yet; the harness
//! is written against the smallest interface that can drive every runner here,
//! and [`new_cpu`] is the single seam where the core gets plugged in. Until it
//! is, every suite skips with a message saying so rather than failing.
//!
//! Deliberately small. Three required methods, one optional one. Anything a
//! runner can compute for itself — elapsed cycles, instruction bytes,
//! disassembly text — is not in the trait.
//!
//! See `docs/testing/cpu-interface.md` for the prose version, including the
//! semantics of each method and the traps.

use std::fmt;

/// One bus access. Exactly one per CPU cycle, no exceptions.
///
/// A 6502 has no idle cycles: every cycle is a read or a write, including the
/// dummy reads that page crossings and read-modify-write instructions perform
/// (`docs/cpu/6502.md`). The SingleStepTests corpus checks that access by
/// access, so a core that batches memory traffic at the end of an instruction
/// cannot pass this suite no matter how correct its results are.
pub(crate) trait Bus6502 {
    /// Read one byte. Called once per read cycle.
    fn read(&mut self, addr: u16) -> u8;
    /// Write one byte. Called once per write cycle.
    fn write(&mut self, addr: u16, value: u8);
}

/// The architectural register file, as the vector format models it.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub(crate) struct Regs {
    /// Program counter.
    pub(crate) pc: u16,
    /// Stack pointer (the low byte of `$01xx`).
    pub(crate) s: u8,
    /// Accumulator.
    pub(crate) a: u8,
    /// Index X.
    pub(crate) x: u8,
    /// Index Y.
    pub(crate) y: u8,
    /// Processor status, `NV1BDIZC`. Bit 5 reads as 1 on real silicon and the
    /// corpus encodes it that way, so a core that stores it as 0 fails every
    /// vector for a reason that has nothing to do with the instruction.
    pub(crate) p: u8,
}

/// Status-flag bit names, most significant first, for readable diffs.
pub(crate) const FLAG_NAMES: [(u8, char); 8] = [
    (0x80, 'N'),
    (0x40, 'V'),
    (0x20, '1'),
    (0x10, 'B'),
    (0x08, 'D'),
    (0x04, 'I'),
    (0x02, 'Z'),
    (0x01, 'C'),
];

/// Render a status byte as flag letters, lower-case where clear.
pub(crate) fn flags_str(p: u8) -> String {
    FLAG_NAMES
        .iter()
        .map(|&(bit, ch)| {
            if p & bit != 0 {
                ch
            } else {
                ch.to_ascii_lowercase()
            }
        })
        .collect()
}

impl fmt::Display for Regs {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(
            f,
            "PC:{:04X} A:{:02X} X:{:02X} Y:{:02X} P:{:02X}[{}] S:{:02X}",
            self.pc,
            self.a,
            self.x,
            self.y,
            self.p,
            flags_str(self.p),
            self.s
        )
    }
}

/// A 6502 interpreter, driven one instruction at a time.
///
/// `Send` because the vector runner shards the corpus across threads and gives
/// each thread its own core. Nothing is shared between them.
pub(crate) trait Cpu6502: Send {
    /// Overwrite the architectural state **and discard all microarchitectural
    /// state**: any half-executed instruction, any latched interrupt, any
    /// pipelined operand fetch. After this call the core must behave exactly as
    /// if it had been in this state at an instruction boundary with no
    /// interrupt pending. Getting this wrong shows up as a handful of vectors
    /// failing per opcode with no pattern, which is a miserable thing to debug.
    fn set_regs(&mut self, regs: Regs);

    /// Read the architectural state back.
    fn regs(&self) -> Regs;

    /// Execute exactly one instruction, driving `bus` once per cycle in cycle
    /// order, and return the number of cycles consumed.
    ///
    /// The returned count must equal the number of bus calls made. It is
    /// checked, so a core that returns a table-driven count while making a
    /// different number of accesses is caught immediately rather than passing
    /// on a technicality.
    fn step(&mut self, bus: &mut dyn Bus6502) -> u32;

    /// Disassemble the instruction at `pc`, given the bytes starting there.
    ///
    /// Optional. `nestest` compares disassembly text only when asked to
    /// (`RSEMU_NESTEST_DISASM=1`), because the reference log's text is one
    /// emulator's formatting convention rather than anything the hardware
    /// specifies. `ROADMAP.md` §6 wants the disassembler generated from the
    /// same table as the decoder, so it should exist; returning `None` just
    /// means this runner will not check it.
    fn disassemble(&self, pc: u16, bytes: &[u8]) -> Option<String> {
        let _ = (pc, bytes);
        None
    }
}

/// Which member of the family is under test.
///
/// The corpus is split the same way, and picking the wrong one produces a
/// confident, uniform, entirely wrong failure across the arithmetic opcodes.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum Variant {
    /// The original NMOS 6502, with a working decimal mode.
    Nmos6502,
    /// The Ricoh RP2A03 in the NES: decimal mode disabled in ADC/SBC, but the
    /// D flag itself still sets, clears, and pushes normally.
    Ricoh2A03,
}

impl Variant {
    /// The corpus subdirectory holding this variant's vectors.
    pub(crate) fn corpus_dir(self) -> &'static str {
        match self {
            Variant::Nmos6502 => "6502",
            Variant::Ricoh2A03 => "nes6502",
        }
    }
}

// ---------------------------------------------------------------------------
// The seam.
// ---------------------------------------------------------------------------

/// Construct a core at a defined post-reset state, or `None` if this build has
/// no 6502.
///
/// # Wiring up a real core
///
/// When `cpu/mos6502` lands, this becomes:
///
/// ```ignore
/// pub fn new_cpu(variant: Variant) -> Option<Box<dyn Cpu6502>> {
///     Some(Box::new(Adapter::new(variant)))
/// }
/// ```
///
/// with a small `Adapter` in this file forwarding the four methods to the real
/// core. Nothing else in the harness changes, and every suite switches from
/// "skipped" to "running" at once.
///
/// The adapter stays in the test tree on purpose: the shape the corpus wants —
/// a `&mut dyn Bus6502` and a whole instruction per call — is a testing shape,
/// not the shape the scheduler will drive the core with, and baking it into
/// `src/` would be letting the tests design the core.
pub(crate) fn new_cpu(variant: Variant) -> Option<Box<dyn Cpu6502>> {
    let _ = variant;
    None
}

/// Is a core available at all? Used for the skip message.
pub(crate) fn have_cpu() -> bool {
    new_cpu(Variant::Ricoh2A03).is_some()
}
