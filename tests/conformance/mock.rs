//! Tiny stand-in cores, so the harness can test itself before a real one exists.
//!
//! A conformance harness whose own correctness is asserted only by "it compiles"
//! is a harness that will report a green run against a broken core. These mocks
//! implement three opcodes between them — enough to drive a real upstream vector
//! end to end — plus four deliberately broken variants that each trip exactly
//! one of the checks the runner makes.
//!
//! Semantics are taken from the instruction reference in `docs/cpu/6502.md`
//! (masswerk / NESdev Obelisk): `LDA #` is two cycles, opcode fetch then operand
//! fetch, setting N and Z from the loaded value; the implied `NOP` still reads
//! the byte after the opcode and discards it; `STA abs` is four, the write
//! landing on the last one.
//!
//! These are not a 6502. Do not grow them into one — the real core is the
//! oracle (`CLAUDE.md`, CPU cores), and a second implementation in the test tree
//! would be a second thing to keep right.

use crate::cpu::{Bus6502, Cpu6502, Regs};

/// Set N and Z from a value, leaving the other flags alone.
fn nz(p: u8, value: u8) -> u8 {
    let cleared = p & !(0x80 | 0x02);
    cleared | (value & 0x80) | if value == 0 { 0x02 } else { 0 }
}

/// A correct implementation of `LDA #`, `NOP` and `STA abs`.
#[derive(Debug, Default)]
pub(crate) struct MockCpu {
    regs: Regs,
}

impl Cpu6502 for MockCpu {
    fn set_regs(&mut self, regs: Regs) {
        self.regs = regs;
    }

    fn regs(&self) -> Regs {
        self.regs
    }

    fn step(&mut self, bus: &mut dyn Bus6502) -> u32 {
        let op = bus.read(self.regs.pc);
        self.regs.pc = self.regs.pc.wrapping_add(1);
        match op {
            // LDA #imm
            0xa9 => {
                let value = bus.read(self.regs.pc);
                self.regs.pc = self.regs.pc.wrapping_add(1);
                self.regs.a = value;
                self.regs.p = nz(self.regs.p, value);
                2
            }
            // NOP — implied, but the next byte is still fetched and thrown away.
            0xea => {
                let _ = bus.read(self.regs.pc);
                2
            }
            // STA abs
            0x8d => {
                let lo = bus.read(self.regs.pc);
                self.regs.pc = self.regs.pc.wrapping_add(1);
                let hi = bus.read(self.regs.pc);
                self.regs.pc = self.regs.pc.wrapping_add(1);
                bus.write(u16::from(lo) | (u16::from(hi) << 8), self.regs.a);
                4
            }
            other => panic!("unimplemented opcode {other:02x}"),
        }
    }

    fn disassemble(&self, _pc: u16, bytes: &[u8]) -> Option<String> {
        match bytes.first()? {
            0xa9 => Some(format!("LDA #${:02X}", bytes.get(1)?)),
            0xea => Some("NOP".to_string()),
            0x8d => Some(format!("STA ${:02X}{:02X}", bytes.get(2)?, bytes.get(1)?)),
            _ => None,
        }
    }
}

/// `LDA #` that forgets to set N. Exercises the register diff.
#[derive(Debug, Default)]
pub(crate) struct BrokenLda {
    regs: Regs,
}

impl Cpu6502 for BrokenLda {
    fn set_regs(&mut self, regs: Regs) {
        self.regs = regs;
    }

    fn regs(&self) -> Regs {
        self.regs
    }

    fn step(&mut self, bus: &mut dyn Bus6502) -> u32 {
        let _op = bus.read(self.regs.pc);
        self.regs.pc = self.regs.pc.wrapping_add(1);
        let value = bus.read(self.regs.pc);
        self.regs.pc = self.regs.pc.wrapping_add(1);
        self.regs.a = value;
        // The bug: N is cleared unconditionally.
        self.regs.p = (self.regs.p & !(0x80 | 0x02)) | if value == 0 { 0x02 } else { 0 };
        2
    }
}

/// A `NOP` that scribbles on memory nothing asked it to touch.
#[derive(Debug, Default)]
pub(crate) struct StrayWrite {
    regs: Regs,
}

impl Cpu6502 for StrayWrite {
    fn set_regs(&mut self, regs: Regs) {
        self.regs = regs;
    }

    fn regs(&self) -> Regs {
        self.regs
    }

    fn step(&mut self, bus: &mut dyn Bus6502) -> u32 {
        let _ = bus.read(self.regs.pc);
        self.regs.pc = self.regs.pc.wrapping_add(1);
        let _ = bus.read(self.regs.pc);
        bus.write(0x1234, 0x99);
        3
    }
}

/// A core that never finishes an instruction.
#[derive(Debug)]
pub(crate) struct Runaway;

impl Cpu6502 for Runaway {
    fn set_regs(&mut self, _regs: Regs) {}

    fn regs(&self) -> Regs {
        Regs::default()
    }

    fn step(&mut self, bus: &mut dyn Bus6502) -> u32 {
        loop {
            let _ = bus.read(0);
        }
    }
}

/// A core that keeps its registers honestly and then panics on any opcode.
///
/// Registers matter here: a core that panicked *and* forgot its state would
/// diverge on the registers first, and the test would then prove nothing about
/// how a panic is reported.
#[derive(Debug, Default)]
pub(crate) struct Panicky {
    regs: Regs,
}

impl Cpu6502 for Panicky {
    fn set_regs(&mut self, regs: Regs) {
        self.regs = regs;
    }

    fn regs(&self) -> Regs {
        self.regs
    }

    fn step(&mut self, _bus: &mut dyn Bus6502) -> u32 {
        panic!("unimplemented opcode");
    }
}

/// A correct `NOP` that reports the wrong cycle count.
///
/// This is the failure mode a table-driven timing model has: the results are
/// right, the bus traffic is right, and the number the scheduler is charged is
/// not.
#[derive(Debug, Default)]
pub(crate) struct LyingCycleCount {
    regs: Regs,
}

impl Cpu6502 for LyingCycleCount {
    fn set_regs(&mut self, regs: Regs) {
        self.regs = regs;
    }

    fn regs(&self) -> Regs {
        self.regs
    }

    fn step(&mut self, bus: &mut dyn Bus6502) -> u32 {
        let _ = bus.read(self.regs.pc);
        self.regs.pc = self.regs.pc.wrapping_add(1);
        let _ = bus.read(self.regs.pc);
        3
    }
}

/// A flat 64 KiB of RAM with no trace and no guards.
///
/// The other direction from the mocks above: a real bus for a real core,
/// small enough that the seam check in `main.rs` can prove a bound adapter
/// actually executes an instruction without dragging in a corpus.
#[derive(Debug)]
pub(crate) struct RamBus {
    mem: Vec<u8>,
}

impl RamBus {
    /// A zeroed bus with `bytes` poked into it.
    pub(crate) fn with(bytes: &[(u16, u8)]) -> RamBus {
        let mut bus = RamBus {
            mem: vec![0; 0x1_0000],
        };
        for &(addr, value) in bytes {
            bus.mem[usize::from(addr)] = value;
        }
        bus
    }
}

impl Bus6502 for RamBus {
    fn read(&mut self, addr: u16) -> u8 {
        self.mem[usize::from(addr)]
    }

    fn write(&mut self, addr: u16, value: u8) {
        self.mem[usize::from(addr)] = value;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn nz_matches_the_reference_semantics() {
        assert_eq!(nz(0x00, 0x00), 0x02, "zero sets Z");
        assert_eq!(nz(0x00, 0x80), 0x80, "bit 7 sets N");
        assert_eq!(nz(0x82, 0x01), 0x00, "a non-zero positive clears both");
        assert_eq!(nz(0x7d, 0x01) & 0x7d, 0x7d, "other flags are untouched");
    }

    #[test]
    fn the_mock_disassembles_what_it_executes() {
        let cpu = MockCpu::default();
        assert_eq!(
            cpu.disassemble(0, &[0xa9, 0xc3]).as_deref(),
            Some("LDA #$C3")
        );
        assert_eq!(
            cpu.disassemble(0, &[0x8d, 0x34, 0x12]).as_deref(),
            Some("STA $1234")
        );
        assert_eq!(cpu.disassemble(0, &[0x00]), None);
    }
}
