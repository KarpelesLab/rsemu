//! Device-level tests for `cpu.spc700`: the address-space bus, the budget
//! accounting, the snapshot. Instruction semantics are measured against
//! `SingleStepTests/spc700` (`tests/conformance/spc700.rs`); the handful here
//! pin behaviours that corpus cannot see.

use alloc::sync::Arc;
use alloc::vec::Vec;

use crate::core::device::{Device, ResetKind};
use crate::core::props::Props;
use crate::core::space::{AddressSpace, RamStore, Region};
use crate::core::state::{ChunkWriter, MachineShape, Migrations, StateReader, StateWriter};

use super::{Mode, Regs, Spc700, flags};

fn cpu_with(code: &[u8]) -> (Arc<Spc700>, Arc<RamStore>) {
    let ram = Arc::new(RamStore::new(0x1_0000));
    for (i, b) in code.iter().enumerate() {
        ram.write_u8(0x0200 + i as u64, *b).unwrap();
    }
    // Reset vector → $0200.
    ram.write_u8(0xfffe, 0x00).unwrap();
    ram.write_u8(0xffff, 0x02).unwrap();
    let space = AddressSpace::new("smp", 16);
    space
        .topology()
        .map(Region::ram("aram", Arc::clone(&ram)), 0)
        .unwrap();
    let cpu = Arc::new(Spc700::new());
    cpu.attach_space(Arc::new(space));
    (cpu, ram)
}

fn snapshot(cpu: &Spc700) -> Vec<u8> {
    let mut writer = StateWriter::new(MachineShape::new());
    {
        let mut chunk: ChunkWriter<'_> = writer
            .chunk("cpu", super::CLASS.name, super::CLASS.version)
            .expect("a chunk");
        Device::save(cpu, &mut chunk).expect("saves");
    }
    writer.to_vec().expect("serialises")
}

fn restore(cpu: &Spc700, bytes: &[u8]) {
    let reader = StateReader::new(bytes).expect("well formed");
    let chunk = reader
        .load(
            "cpu",
            super::CLASS.name,
            super::CLASS.version,
            &Migrations::new(),
        )
        .expect("finds the chunk");
    Device::load(cpu, &mut chunk.reader()).expect("loads");
}

#[test]
fn reset_then_a_small_program() {
    // MOV A,#$42 ; MOV $10,A ; INC $10 ; MOV X,$10
    let (cpu, ram) = cpu_with(&[0xe8, 0x42, 0xc4, 0x10, 0xab, 0x10, 0xf8, 0x10]);
    assert_eq!(cpu.mode(), Mode::Reset);
    assert_eq!(cpu.step(), 2, "the reset sequence reads the vector");
    assert_eq!(cpu.regs().pc, 0x0200);
    assert_eq!(cpu.step(), 2, "MOV A,#imm");
    assert_eq!(cpu.step(), 4, "MOV dp,A reads its target first");
    assert_eq!(cpu.step(), 4, "INC dp");
    assert_eq!(cpu.step(), 3, "MOV X,dp");
    assert_eq!(ram.read_u8(0x10).unwrap(), 0x43);
    assert_eq!(cpu.regs().x, 0x43);
    assert_eq!(cpu.cycles(), 15);
}

#[test]
fn call_and_return_through_page_one() {
    // CALL $0300 ; ... at $0300: RET
    let (cpu, ram) = cpu_with(&[0x3f, 0x00, 0x03]);
    ram.write_u8(0x0300, 0x6f).unwrap();
    cpu.set_regs(Regs {
        sp: 0xef,
        pc: 0x0200,
        ..Regs::new()
    });
    assert_eq!(cpu.step(), 8);
    assert_eq!(cpu.regs().sp, 0xed);
    assert_eq!(ram.read_u8(0x01ef).unwrap(), 0x02, "return address high");
    assert_eq!(ram.read_u8(0x01ee).unwrap(), 0x03, "return address low");
    assert_eq!(cpu.step(), 5);
    assert_eq!(cpu.regs().pc, 0x0203);
}

#[test]
fn a_budget_carries_its_overshoot() {
    // DIV is twelve cycles; a budget of five must report five and owe seven.
    let (cpu, _ram) = cpu_with(&[0x9e, 0x00]);
    cpu.set_regs(Regs {
        x: 1,
        pc: 0x0200,
        ..Regs::new()
    });
    assert_eq!(cpu.run_budget(5), 5);
    assert_eq!(cpu.run_budget(5), 5, "paid from the debt");
    assert_eq!(cpu.cycles(), 12);
}

#[test]
fn a_snapshot_round_trip_reproduces_the_state_exactly() {
    let (cpu, _ram) = cpu_with(&[0xe8, 0x42, 0x60, 0x40, 0xef]);
    for _ in 0..6 {
        cpu.step();
    }
    assert_eq!(cpu.mode(), Mode::Sleeping);
    let first = snapshot(&cpu);
    let (other, _ram2) = cpu_with(&[]);
    restore(&other, &first);
    assert_eq!(other.regs(), cpu.regs());
    assert_eq!(other.regs().psw & flags::P, flags::P);
    assert_eq!(other.mode(), Mode::Sleeping);
    assert_eq!(
        snapshot(&other),
        first,
        "identical state image after a round trip"
    );
}

#[test]
fn a_reset_reruns_the_vector_fetch() {
    let (cpu, _ram) = cpu_with(&[0x00]);
    cpu.step();
    cpu.step();
    Device::reset(cpu.as_ref(), ResetKind::Warm);
    assert_eq!(cpu.mode(), Mode::Reset);
    cpu.step();
    assert_eq!(cpu.regs().pc, 0x0200);
}

#[test]
fn the_class_rejects_unknown_properties() {
    assert!(Spc700::from_props(&Props::new()).is_ok());
    assert!(Spc700::from_props(&Props::new().with("engin", "interp")).is_err());
}

#[test]
fn the_disassembler_reads_without_side_effects() {
    let (cpu, _ram) = cpu_with(&[0x8f, 0xaa, 0xf4]);
    let out = cpu.disassemble(0x0200, 1);
    assert_eq!(alloc::format!("{}", out[0]), "MOV $f4,#$aa");
}
