//! USSE2 tests. Every word here is assembled from the field positions in
//! `usse.rs` (and `ENCODING.md`) by the helpers below; none is copied out of
//! a firmware image. The two tests that read the real microkernel are gated
//! on `RSEMU_SGX_FW` and skip without it.

use super::*;
use alloc::collections::BTreeMap;
use alloc::vec::Vec;

// ---------------------------------------------------------------------------
// A tiny assembler, for the tests

/// A standard destination: temp `r<n>`.
fn d_r(n: u64) -> u64 {
    n << 21
}

/// `src1` / `src2` as temps.
fn s1_r(n: u64) -> u64 {
    n << 7
}
fn s2_r(n: u64) -> u64 {
    n
}

/// `src1` / `src2` as 7-bit immediates (ext bit, bank 2).
fn s1_imm(v: u64) -> u64 {
    b(49) | 2 << 30 | (v & 0x7f) << 7
}
fn s2_imm(v: u64) -> u64 {
    b(48) | 2 << 28 | (v & 0x7f)
}

/// The bitwise forms' wide `src2` immediate (§3), no rotate.
fn s2_wide(v: u64) -> u64 {
    b(48) | 2 << 28 | (v & 0x7f) | (v >> 7 & 0x7f) << 14 | (v >> 14 & 3) << 36
}

/// The long predicate field at `lo`: `0 none, 1 p0 … 4 p3, 5 !p0, 6 !p1`.
fn pred_long(v: u64, lo: u32) -> u64 {
    v << lo
}

fn alu(hw: u64, second: bool, dst: u64, s1: u64, s2: u64) -> u64 {
    op(hw) | if second { b(35) } else { 0 } | dst | s1 | s2
}

fn limm(dst: u64, v: u32) -> u64 {
    let v = u64::from(v);
    class(2) | 4 << 56 | dst | (v & 0x1f_ffff) | (v >> 21 & 0x1f) << 36 | (v >> 26 & 0x3f) << 44
}

fn ba(target: u64) -> u64 {
    class(0) | target
}

fn ba_link(target: u64) -> u64 {
    class(0) | b(41) | target
}

fn br(off: i32) -> u64 {
    class(0) | 1 << 38 | (off as u32 as u64 & 0xf_ffff)
}

fn lapc() -> u64 {
    class(0) | 2 << 38
}

fn setl(s1: u64) -> u64 {
    class(0) | 3 << 38 | s1
}

fn savl(dst: u64) -> u64 {
    class(0) | 4 << 38 | dst
}

fn nop() -> u64 {
    class(0) | 5 << 38
}

/// A test-form word: sub-op, test byte (§5.3 bit order), predicate
/// destination, write-back.
fn test_word(sub: u64, t: u8, pdst: u64, wb: Option<u64>, s1: u64, s2: u64) -> u64 {
    const POS: [u32; 8] = [42, 43, 40, 41, 39, 36, 37, 38];
    let tbits = POS.iter().enumerate().fold(0, |acc, (i, &p)| {
        acc | if t >> i & 1 != 0 { b(p) } else { 0 }
    });
    let dst = match wb {
        Some(n) => b(20) | d_r(n),
        None => b(51) | b(32),
    };
    op(9) | sub << 14 | tbits | pdst << 34 | dst | s1 | s2
}

const T_Z: u8 = 0x14;
const T_NZ: u8 = 0x18;
const T_N: u8 = 0x11;

fn ldr(dst: u64, reg: u64, drc: u64) -> u64 {
    class(2)
        | 6 << 56
        | dst
        | (reg & 0x7f)
        | (reg >> 7 & 0x7f) << 14
        | (reg >> 14 & 0xf) << 34
        | b(48)
        | 2 << 28
        | drc << 32
}

fn str_(reg: u64, s1: u64) -> u64 {
    class(2)
        | 6 << 56
        | b(51)
        | (reg & 0x7f)
        | (reg >> 7 & 0x7f) << 14
        | (reg >> 14 & 0xf) << 34
        | b(48)
        | 2 << 28
        | s1
}

/// `ld<size> r<dst>, r<base>, s1` with the encoder's default `b53`.
fn ld(size: u64, dst: u64, base: u64, s1: u64) -> u64 {
    op(29) | b(53) | size << 36 | d_r(dst) | base << 14 | s1
}

fn st(size: u64, base: u64, s1: u64, s2: u64) -> u64 {
    op(30) | b(53) | size << 36 | base << 14 | s1 | s2
}

fn rpt(n: u64) -> u64 {
    (n - 1) << 44
}

// ---------------------------------------------------------------------------
// A memory-and-registers bus

#[derive(Default)]
struct MockBus {
    /// Words by GPU address.
    mem: BTreeMap<u32, u32>,
    regs: BTreeMap<u32, u32>,
    reg_reads: Vec<u32>,
    reg_writes: Vec<(u32, u32)>,
    loads: Vec<(u32, Size)>,
    stores: Vec<(u32, Size, u32)>,
    emits: Vec<Emit>,
}

impl MockBus {
    fn program(base: u32, words: &[u64]) -> Self {
        let mut bus = MockBus::default();
        for (i, w) in words.iter().enumerate() {
            let a = base + 8 * i as u32;
            bus.mem.insert(a, *w as u32);
            bus.mem.insert(a + 4, (*w >> 32) as u32);
        }
        bus
    }
}

impl UsseBus for MockBus {
    fn load(&mut self, addr: u32, size: Size) -> MemResult<u32> {
        let word = self.mem.get(&(addr & !3)).copied().unwrap_or(0);
        let shift = (addr & 3) * 8;
        self.loads.push((addr, size));
        Ok(match size {
            Size::Byte => word >> shift & 0xff,
            Size::Word => word >> shift & 0xffff,
            Size::Dword => word,
        })
    }

    fn store(&mut self, addr: u32, size: Size, value: u32) -> MemResult {
        self.stores.push((addr, size, value));
        let old = self.mem.get(&(addr & !3)).copied().unwrap_or(0);
        let shift = (addr & 3) * 8;
        let mask = match size {
            Size::Byte => 0xffu32 << shift,
            Size::Word => 0xffffu32 << shift,
            Size::Dword => !0,
        };
        self.mem
            .insert(addr & !3, old & !mask | (value << shift) & mask);
        Ok(())
    }

    fn reg_read(&mut self, index: u32) -> u32 {
        self.reg_reads.push(index);
        self.regs.get(&index).copied().unwrap_or(0)
    }

    fn reg_write(&mut self, index: u32, value: u32) {
        self.reg_writes.push((index, value));
        self.regs.insert(index, value);
    }

    fn emit(&mut self, emit: &Emit) -> MemResult {
        self.emits.push(*emit);
        Ok(())
    }
}

const BASE: u32 = 0x1000_0000;

/// Runs `words` from instruction 0 until it stops.
fn run(words: &[u64], setup: impl FnOnce(&mut Usse)) -> (Usse, MockBus, Stop) {
    let mut bus = MockBus::program(BASE, words);
    let mut usse = Usse::new(BASE, 0);
    setup(&mut usse);
    let stop = usse.run(&mut bus, 1000);
    (usse, bus, stop)
}

fn dis(w: u64) -> String {
    disassemble(w, BASE, BASE)
}

/// The END bit for the ALU-shaped forms.
const END: u64 = 1 << 54;

// ---------------------------------------------------------------------------
// Decoding and disassembly

#[test]
fn limm_anchor_matches_the_encoder() {
    // §5.1: the encoder's own output for `limm r2, #0xdeadbeef`.
    let w = 0xfc23_7150_004d_beef;
    assert_eq!(limm(d_r(2), 0xdead_beef), w);
    let insn = decode(w).unwrap();
    assert_eq!(insn.op(), Op::Limm);
    assert_eq!(
        insn.args()[0].op,
        Operand::Reg {
            bank: Bank::Temp,
            num: 2
        }
    );
    assert_eq!(
        insn.args()[1].op,
        Operand::Imm {
            value: 0xdead_beef,
            style: ImmStyle::Hex
        }
    );
    assert_eq!(dis(w), "limm r2, #0xdeadbeef");
}

#[test]
fn every_destination_bank_decodes() {
    let cases = [
        (0, "r5"),
        (b(32), "o5"),
        (b(33), "pa5"),
        (b(51), "sa5"),
        (b(51) | b(32), "c5"),
        (b(33) | b(32), "r[i1+5]"),
        (b(51) | b(33) | b(32), "r[i2+5]"),
    ];
    for (bank, text) in cases {
        let w = alu(10, false, bank | d_r(5), s1_r(1), s2_r(2));
        assert_eq!(dis(w), alloc::format!("and {text}, r1, r2"), "{w:016x}");
    }
    let g = alu(10, false, b(51) | b(32) | d_r(70), s1_r(1), s2_r(2));
    assert_eq!(dis(g), "and g70, r1, r2");
    let i = alu(10, false, b(51) | b(33) | d_r(1), s1_r(1), s2_r(2));
    assert_eq!(dis(i), "and i1, r1, r2");
    let sa_idx = alu(10, false, b(33) | b(32) | d_r(0x60 | 3), s1_r(1), s2_r(2));
    assert_eq!(dis(sa_idx), "and sa[i1+3], r1, r2");
}

#[test]
fn every_source_bank_decodes() {
    let cases = [
        (0u64, 0u64, "r9"),
        (1, 0, "o9"),
        (2, 0, "pa9"),
        (3, 0, "sa9"),
        (0, 1, "r[i1+9]"),
        (1, 1, "c9"),
        (2, 1, "#9"),
        (3, 1, "r[i2+9]"),
    ];
    for (bank, ext, text) in cases {
        let w = alu(11, false, d_r(0), bank << 30 | ext << 49 | 9 << 7, s2_r(2));
        assert_eq!(dis(w), alloc::format!("xor r0, {text}, r2"));
        let w = alu(11, false, d_r(0), s1_r(2), bank << 28 | ext << 48 | 9);
        assert_eq!(dis(w), alloc::format!("xor r0, r2, {text}"));
    }
}

#[test]
fn bitwise_family_names_and_mov() {
    assert_eq!(
        dis(alu(10, false, d_r(0), s1_r(1), s2_r(2))),
        "and r0, r1, r2"
    );
    assert_eq!(
        dis(alu(10, true, d_r(0), s1_r(1), s2_r(2))),
        "or r0, r1, r2"
    );
    assert_eq!(dis(alu(10, true, d_r(0), s1_r(1), s2_imm(0))), "mov r0, r1");
    assert_eq!(
        dis(alu(11, false, d_r(0), s1_r(1), s2_imm(3))),
        "xor r0, r1, #3"
    );
    assert_eq!(
        dis(alu(12, false, d_r(0), s1_r(1), s2_imm(4))),
        "shl r0, r1, #4"
    );
    assert_eq!(
        dis(alu(12, true, d_r(0), s1_r(1), s2_imm(5))),
        "rol r0, r1, #5"
    );
    assert_eq!(
        dis(alu(13, false, d_r(0), s1_r(1), s2_imm(16))),
        "shr r0, r1, #0x10"
    );
    assert_eq!(
        dis(alu(13, true, d_r(0), s1_r(1), s2_imm(1))),
        "asr r0, r1, #1"
    );
    assert_eq!(
        dis(alu(10, false, d_r(0), s1_r(1), s2_wide(0x360))),
        "and r0, r1, #0x360"
    );
    let inv = alu(10, false, d_r(0), s1_r(1), s2_wide(0xf)) | b(43);
    assert_eq!(dis(inv), "and r0, r1, #0xf.inv");
    // §3: a rotate of 16 puts the immediate in the high half.
    let rot = alu(10, true, d_r(0), s1_r(1), s2_wide(0xad00)) | 16 << 38;
    assert_eq!(dis(rot), "or r0, r1, #0xad000000");
    let flagged = alu(10, true, d_r(0), s1_r(0), s2_imm(0)) | b(50) | rpt(3) | pred_long(5, 56);
    assert_eq!(dis(flagged), "(!p0) mov.syncs?.rpt3 r0, r0");
}

#[test]
fn test_form_decodes() {
    let w = test_word(0x30, T_NZ, 0, None, s1_r(0), s2_r(0));
    assert_eq!(dis(w), "and.test.t[nz] _, p0, r0, r0");
    let w = test_word(0x32, T_Z, 1, None, s1_r(7), s2_imm(1));
    assert_eq!(dis(w), "xor.test.t[z] _, p1, r7, #1");
    let w = test_word(0x17, T_NZ, 0, Some(5), s1_r(5), s2_imm(1));
    assert_eq!(dis(w), "imadd20n.test.t[nz] r5, p0, r5, r5, #1");
    let w = test_word(0x33, T_N, 2, None, s1_r(1), s2_imm(25)) | pred_long(6, 56);
    assert_eq!(dis(w), "(!p1) shl.test.t[n] _, p2, r1, #0x19");
    let w = test_word(0x17, 0x19, 2, None, s1_r(2), s2_r(3));
    assert_eq!(dis(w), "imadd20n.test.t[nz&n] _, p2, r2, r2, r3");
}

#[test]
fn mad_families_decode() {
    // §4: dst b27:21, src0 b20:14, src1 the standard slot, src2 likewise.
    let base = d_r(4) | 2 << 14 | s1_imm(8) | s2_r(3);
    assert_eq!(dis(op(21) | base), "imae r4, r2, #8, r3");
    assert_eq!(dis(op(21) | b(43) | base), "imae_b r4, r2, #8, r3");
    assert_eq!(dis(op(21) | b(56) | base), "imae r4, r2.hi, #8, r3");
    // Two-bit predicate at b58:57: 3 is !p0.
    assert_eq!(dis(op(21) | 3 << 57 | base), "(!p0) imae r4, r2, #8, r3");
    let dec = d_r(1) | 1 << 14 | s1_imm(1) | s2_imm(1);
    assert_eq!(dis(op(20) | b(43) | b(53) | dec), "imadd20 r1, r1, #1, -#1");
    assert_eq!(dis(op(20) | dec), "imadd20b r1, r1, #1, #1");
    // Opcode 26 has b39 set for a positive src2, and a signed immediate with
    // b41 set.
    assert_eq!(dis(op(26) | b(39) | b(41) | dec), "imadd26n r1, r1, #1, #1");
    assert_eq!(dis(op(26) | dec), "imadd26 r1, r1, #1, -#1");
    assert_eq!(
        dis(op(26) | b(39) | b(41) | d_r(1) | 1 << 14 | s1_imm(1) | s2_imm(0x7f)),
        "imadd26n r1, r1, #1, #-1"
    );
    // b52 on opcode 26 is unknown and stays undecoded.
    assert!(decode(op(26) | b(52)).is_none());
}

#[test]
fn memory_forms_decode() {
    assert_eq!(dis(ld(0, 1, 0, s1_imm(1))), "ldad r1, r0, #1, drc0");
    assert_eq!(dis(ld(1, 1, 0, s1_imm(1)) | b(32)), "ldaw r1, r0, #1, drc1");
    assert_eq!(dis(ld(2, 1, 0, s1_imm(1)) | b(42)), "ldlb r1, r0, #1, drc0");
    assert_eq!(
        dis(ld(0, 1, 0, s1_imm(1)) | b(39) | b(34)),
        "ldad pa1, pa0, #1, drc0"
    );
    // b53 is set in the encoder's base word: clear, it prints as f1.31.
    assert_eq!(
        dis(ld(0, 4, 3, s1_imm(0)) & !b(53) | b(41) | b(33) | rpt(4)),
        "ldad.f1.28.f1.31.f2.2.rpt4 r4, r3, #0, drc0"
    );
    assert_eq!(dis(st(0, 1, s1_imm(0x14), s2_r(2))), "stad r1, #0x14, r2");
    assert_eq!(
        dis(st(0, 1, s1_imm(0), s2_r(2)) | b(34) | b(50)),
        "stad sa1, #0, r2"
    );
    assert_eq!(dis(ldr(d_r(1), 9, 0)), "ldr r1, #9, drc0");
    assert_eq!(dis(ldr(d_r(10), 0x1c9, 1)), "ldr r10, #0x1c9, drc1");
    assert_eq!(dis(str_(0x229a, s1_r(0))), "str #0x229a, r0");
    assert_eq!(dis(str_(0x30d, s1_imm(8)) | 1 << 41), "(p0) str #0x30d, #8");
    assert_eq!(dis(str_(0x134a, s1_r(16)) | b(43)), "str.end #0x134a, r16");
}

#[test]
fn flow_forms_decode() {
    assert_eq!(
        disassemble(ba(0x25b9), 0xf200_0070, 0xf200_0000),
        "ba 0x25b9 -> 0xf2012dc8"
    );
    assert_eq!(dis(ba_link(0x10)), "ba.link 0x10 -> 0x10000080");
    assert_eq!(
        dis(ba(0x35) | pred_long(5, 56)),
        "(!p0) ba 0x35 -> 0x100001a8"
    );
    assert_eq!(
        disassemble(br(-8) | pred_long(1, 56), 0x100, 0),
        "(p0) br -8 -> 0x000000c0"
    );
    assert_eq!(dis(br(14)), "br +14 -> 0x10000070");
    assert_eq!(dis(lapc()), "lapc ");
    assert_eq!(dis(nop()), "nop ");
    assert_eq!(dis(nop() | b(50)), "nop.syncs? ");
    assert_eq!(dis(setl(s1_r(23))), "setl r23");
    assert_eq!(dis(savl(d_r(23))), "savl r23");
    assert_eq!(dis(class(2) | b(32)), "idf drc1, #0");
    assert_eq!(dis(class(2) | 1 << 56), "wdf drc0");
}

#[test]
fn op31_odd_forms_decode() {
    let phas = class(4) | 2 << 56 | b(51) | b(45) | 0x2987;
    assert_eq!(dis(phas), "phas.syncs? #0x2987, #0, t16:60, t14:0, t16:67");
    let phas = class(4) | 2 << 56 | 7 << 40;
    assert_eq!(dis(phas), "phas #0, #0, t16:60, t16:65, t16:66");
    assert_eq!(dis(class(4) | 2 << 56 | b(55) | b(51)), "op358.syncs? ");
    assert_eq!(
        dis(class(1) | 2 << 56 | 1),
        "smlsi #0, #0, #0, #1, #0, #0, #0, #0, #0, #0, #0"
    );
    assert_eq!(dis(class(2) | 5 << 56 | 2 << 4), "op205 #2");
    // EMIT: target b33:32, src0 b20:14, src1, src2, and a 14-bit immediate.
    let imm = 0x843u64;
    // Register operands name pairs: the fields hold n / 2.
    let emit = class(2)
        | 3 << 56
        | b(47)
        | b(45)
        | b(44)
        | 1 << 7
        | 2
        | (imm & 0x3f) << 22
        | (imm >> 6 & 0x3f) << 35;
    assert_eq!(dis(emit), "emit162.f2.4.f2.5 #0, r0, r2, r4, #0x843");
    assert_eq!(dis(0), "vmad r0, r0, r0, r0");
    assert_eq!(dis(class(0) | 7 << 38), ".word");
}

#[test]
fn table_forms_are_reachable() {
    // Each form must be the first match for its own match value, or an
    // earlier, wider form shadows it.
    for (i, f) in FORMS.iter().enumerate() {
        let first = FORMS
            .iter()
            .position(|g| f.value & g.mask == g.value)
            .unwrap();
        assert_eq!(
            first, i,
            "form {i} ({:?}) is shadowed by form {first}",
            f.name
        );
    }
}

// ---------------------------------------------------------------------------
// Execution

#[test]
fn limm_and_bitwise_execute() {
    let (u, _, stop) = run(
        &[
            limm(d_r(0), 0xf0f0_1234),
            limm(d_r(1), 0x0ff0_00ff),
            alu(10, false, d_r(2), s1_r(0), s2_r(1)),
            alu(10, true, d_r(3), s1_r(0), s2_r(1)),
            alu(11, false, d_r(4), s1_r(0), s2_r(1)),
            alu(12, false, d_r(5), s1_r(1), s2_imm(4)),
            alu(13, false, d_r(6), s1_r(0), s2_imm(4)),
            alu(13, true, d_r(7), s1_r(0), s2_imm(4)),
            alu(12, true, d_r(8), s1_r(0), s2_imm(8)),
            alu(10, false, d_r(9), s1_r(0), s2_wide(0xff)) | b(43),
            alu(10, true, d_r(10), s1_r(1), s2_imm(0)) | END,
        ],
        |_| {},
    );
    assert_eq!(stop, Stop::End { pc: 11 });
    assert_eq!(u.temp[2], 0x00f0_0034);
    assert_eq!(u.temp[3], 0xfff0_12ff);
    assert_eq!(u.temp[4], 0xff00_12cb);
    assert_eq!(u.temp[5], 0xff00_0ff0);
    assert_eq!(u.temp[6], 0x0f0f_0123);
    assert_eq!(u.temp[7], 0xff0f_0123);
    assert_eq!(u.temp[8], 0xf012_34f0);
    assert_eq!(u.temp[9], 0xf0f0_1200);
    assert_eq!(u.temp[10], 0x0ff0_00ff);
}

#[test]
fn banks_and_index_registers_execute() {
    let (u, _, stop) = run(
        &[
            // mov i1, r0 ; mov r1, sa[i1+2] ; mov o[i1+0], pa3
            alu(10, true, b(51) | b(33) | d_r(1), s1_r(0), s2_imm(0)),
            // src1 bank 0 with the extended bit: indexed through i1; the
            // number field is [6:5] bank (3, sa), [4:0] offset.
            alu(10, true, d_r(1), b(49) | (0x60 | 2) << 7, s2_imm(0)),
            // An indexed destination: bank 1 (o), offset 0.
            alu(
                10,
                true,
                b(33) | b(32) | d_r(0x20),
                2 << 30 | 3 << 7,
                s2_imm(0),
            ) | END,
        ],
        |u| {
            u.temp[0] = 5;
            u.secondary[7] = 0xabcd;
            u.primary[3] = 0x1234;
        },
    );
    assert_eq!(stop, Stop::End { pc: 3 });
    assert_eq!(u.index[1], 5);
    assert_eq!(u.temp[1], 0xabcd);
    assert_eq!(u.output[5], 0x1234);
}

#[test]
fn repeat_steps_register_operands() {
    let (u, _, _) = run(
        &[alu(10, true, d_r(8), s1_r(0), s2_imm(0)) | rpt(4) | END],
        |u| u.temp[..4].copy_from_slice(&[1, 2, 3, 4]),
    );
    assert_eq!(&u.temp[8..12], &[1, 2, 3, 4]);
}

#[test]
fn predicates_gate_execution() {
    let (u, _, _) = run(
        &[
            test_word(0x32, T_Z, 0, None, s1_r(0), s2_imm(1)), // p0 = r0 == 1
            limm(d_r(1), 11) | 1 << 41,                        // (p0)
            limm(d_r(2), 22) | 5 << 41,                        // (!p0)
            test_word(0x30, T_NZ, 3, None, s1_r(0), s2_imm(2)), // p3 = r0 & 2 != 0
            alu(10, true, d_r(3), s1_imm(33), s2_imm(0)) | pred_long(4, 56) | END,
        ],
        |u| u.temp[0] = 1,
    );
    assert!(u.preds[0]);
    assert!(!u.preds[3]);
    assert_eq!((u.temp[1], u.temp[2], u.temp[3]), (11, 0, 0));
}

#[test]
fn test_form_writes_back_and_tests_sign() {
    let (u, _, _) = run(
        &[
            // r5 = r5 - 1, p0 = r5 != 0  (the loop counter idiom)
            test_word(0x17, T_NZ, 0, Some(5), s1_r(5), s2_imm(1)),
            // p1 = r0 - r8 < 0
            test_word(0x17, T_N, 1, None, s1_r(0), s2_r(8)),
            // p2 = (r1 << 25) < 0, i.e. bit 6 of r1
            test_word(0x33, T_N, 2, None, s1_r(1), s2_imm(25)) | END,
        ],
        |u| {
            u.temp[5] = 1;
            u.temp[0] = 3;
            u.temp[8] = 7;
            u.temp[1] = 0x40;
        },
    );
    assert_eq!(u.temp[5], 0);
    assert_eq!(u.preds[..3], [false, true, true]);
}

#[test]
fn mad_executes() {
    let (u, _, _) = run(
        &[
            op(21) | d_r(4) | 2 << 14 | s1_imm(8) | s2_r(3), // r4 = r2*8 + r3
            op(20) | b(43) | b(53) | d_r(1) | 1 << 14 | s1_imm(1) | s2_imm(1), // r1 -= 1
            op(26) | b(41) | d_r(6) | 6 << 14 | s1_imm(1) | s2_r(7), // r6 = r6 - r7
            op(21) | b(56) | d_r(9) | 9 << 14 | s1_imm(1) | s2_imm(0) | END, // r9 = r9 >> 16
        ],
        |u| {
            u.temp[2] = 5;
            u.temp[3] = 100;
            u.temp[1] = 0;
            u.temp[6] = 10;
            u.temp[7] = 3;
            u.temp[9] = 0x1234_5678;
        },
    );
    assert_eq!(u.temp[4], 140);
    assert_eq!(u.temp[1], u32::MAX);
    assert_eq!(u.temp[6], 7);
    assert_eq!(u.temp[9], 0x1234);
}

#[test]
fn branch_call_and_return() {
    let (u, _, stop) = run(
        &[
            savl(d_r(23)),                                     // 0
            ba_link(5),                                        // 1: call 5
            setl(s1_r(23)),                                    // 2
            alu(10, true, d_r(1), s1_imm(1), s2_imm(0)) | END, // 3
            nop(),                                             // 4
            limm(d_r(2), 7),                                   // 5
            br(2),                                             // 6: skip 7
            limm(d_r(2), 8),                                   // 7
            lapc(),                                            // 8: back to 2
        ],
        |_| {},
    );
    assert_eq!(stop, Stop::End { pc: 4 });
    assert_eq!(u.temp[2], 7);
    assert_eq!(u.link, 0);
    assert_eq!(u.temp[1], 1);
    assert_eq!(u.retired, 7);
}

#[test]
fn backward_branch_loops() {
    // r1 = 3; loop: r2 += 2; r1 -= 1, p0 = r1 != 0; (p0) br -2
    let (u, _, _) = run(
        &[
            limm(d_r(1), 3),
            op(21) | d_r(2) | 2 << 14 | s1_imm(1) | s2_imm(2),
            test_word(0x17, T_NZ, 0, Some(1), s1_r(1), s2_imm(1)),
            br(-2) | pred_long(1, 56),
            nop() | b(43),
        ],
        |_| {},
    );
    assert_eq!(u.temp[2], 6);
    assert_eq!(u.temp[1], 0);
}

#[test]
fn ldr_str_go_to_the_register_file() {
    let (u, bus, _) = run(
        &[
            ldr(d_r(1), 0x46, 0),
            class(2) | 1 << 56, // wdf drc0
            str_(0x229a, s1_r(1)),
            str_(0x1a7, s1_imm(1)) | rpt(2),
            // An LDR whose number is a register.
            class(2) | 6 << 56 | d_r(3) | s2_r(2),
            nop() | b(43),
        ],
        |u| u.temp[2] = 0x10,
    );
    assert_eq!(bus.reg_reads, [0x46, 0x10]);
    assert_eq!(bus.reg_writes, [(0x229a, 0), (0x1a7, 1), (0x1a8, 1)]);
    let _ = u;
    let mut bus = MockBus::program(BASE, &[ldr(d_r(1), 0x46, 0) | rpt(2), nop() | b(43)]);
    bus.regs.insert(0x46, 0xaa);
    bus.regs.insert(0x47, 0xbb);
    let mut u = Usse::new(BASE, 0);
    u.run(&mut bus, 10);
    assert_eq!((u.temp[1], u.temp[2]), (0xaa, 0xbb));
}

#[test]
fn ld_st_go_to_memory() {
    let mut bus = MockBus::program(
        BASE,
        &[
            ld(0, 1, 0, s1_imm(1)),                     // r1 = [r0 + 4]
            ld(1, 2, 0, s1_imm(1)),                     // r2 = half [r0 + 2]
            ld(2, 3, 0, s1_imm(7)),                     // r3 = byte [r0 + 7]
            st(0, 4, s1_imm(2), s2_r(1)),               // [r4 + 8] = r1
            ld(0, 8, 0, s1_imm(0)) | rpt(3),            // r8..r10 = [r0], [r0+4], [r0+8]
            st(0, 4, s1_imm(0x10), s2_imm(0)) | rpt(2), // [r4+0x40], [r4+0x44] = 0
            nop() | b(43),
        ],
    );
    bus.mem.insert(0x2000, 0x1111_1111);
    bus.mem.insert(0x2004, 0xdead_beef);
    bus.mem.insert(0x2008, 0x2222_2222);
    let mut u = Usse::new(BASE, 0);
    u.temp[0] = 0x2000;
    u.temp[4] = 0x3000;
    let stop = u.run(&mut bus, 100);
    assert_eq!(stop, Stop::End { pc: 7 });
    assert_eq!(u.temp[1], 0xdead_beef);
    assert_eq!(u.temp[2], 0x1111);
    assert_eq!(u.temp[3], 0xde);
    assert_eq!(bus.mem[&0x3008], 0xdead_beef);
    assert_eq!(&u.temp[8..11], &[0x1111_1111, 0xdead_beef, 0x2222_2222]);
    assert_eq!(
        &bus.stores[1..],
        &[(0x3040, Size::Dword, 0), (0x3044, Size::Dword, 0)]
    );
}

#[test]
fn emit_reaches_the_bus() {
    let imm = 0x843u64;
    let emit = class(2)
        | 3 << 56
        | b(47)
        | 1 << 7
        | s2_imm(9)
        | (imm & 0x3f) << 22
        | (imm >> 6 & 0x3f) << 35;
    let (_, bus, _) = run(&[emit, nop() | b(43)], |u| {
        u.temp[..4].copy_from_slice(&[1, 2, 3, 4])
    });
    assert_eq!(bus.emits.len(), 1);
    assert_eq!(bus.emits[0].sources, [[1, 2], [3, 4], [9, 0]]);
    assert_eq!(bus.emits[0].imm, 0x843);
}

#[test]
fn stops_are_reported() {
    // Phase end with a declared next phase.
    let (_, _, stop) = run(&[class(4) | 2 << 56 | b(51) | 0x42], |_| {});
    assert_eq!(
        stop,
        Stop::Phase {
            pc: 0,
            next: Some(0x42)
        }
    );
    let (u, _, stop) = run(
        &[class(4) | 2 << 56 | 0x9, class(4) | 2 << 56 | b(55)],
        |_| {},
    );
    assert_eq!(
        stop,
        Stop::Phase {
            pc: 1,
            next: Some(0x9)
        }
    );
    assert_eq!(u.pc, 2);
    // Vector, RLP and unknown words.
    let (u, _, stop) = run(&[0], |_| {});
    assert!(matches!(stop, Stop::Unimplemented { pc: 0, word: 0, .. }));
    assert_eq!(u.pc, 0);
    let (_, _, stop) = run(&[op(14) | d_r(1)], |_| {});
    assert!(matches!(stop, Stop::Unimplemented { why: "RLP", .. }));
    let (_, _, stop) = run(&[op(26) | b(52)], |_| {});
    assert_eq!(
        stop,
        Stop::Undecodable {
            pc: 0,
            word: op(26) | b(52)
        }
    );
    // The per-instance predicate.
    let (_, _, stop) = run(&[nop() | 7 << 56], |_| {});
    assert!(matches!(
        stop,
        Stop::Unimplemented {
            why: "per-instance predicate",
            ..
        }
    ));
    // A budget.
    let (_, _, stop) = run(&[ba(0)], |_| {});
    assert_eq!(stop, Stop::Budget { pc: 0 });
    // A register past its bank.
    let (_, _, stop) = run(
        &[alu(10, true, d_r(127), s1_r(0), s2_imm(0)) | rpt(2)],
        |_| {},
    );
    assert!(matches!(stop, Stop::BadRegister { pc: 0, .. }));
}

#[test]
fn a_fetch_fault_stops() {
    struct Faulty;
    impl UsseBus for Faulty {
        fn load(&mut self, _: u32, _: Size) -> MemResult<u32> {
            Err(BusError::Unassigned)
        }
        fn store(&mut self, _: u32, _: Size, _: u32) -> MemResult {
            Ok(())
        }
        fn reg_read(&mut self, _: u32) -> u32 {
            0
        }
        fn reg_write(&mut self, _: u32, _: u32) {}
    }
    let mut u = Usse::new(0x100, 2);
    assert_eq!(
        u.run(&mut Faulty, 10),
        Stop::Fault {
            pc: 2,
            addr: 0x110,
            error: BusError::Unassigned
        }
    );
}

// ---------------------------------------------------------------------------
// The real microkernel, when it is there

#[cfg(feature = "std")]
mod firmware {
    use super::*;
    use std::path::PathBuf;
    use std::println;

    /// The directory holding the microkernel images (and optionally the
    /// reference listings `main.dis` / `slave.dis`), or `None` to skip.
    fn dir() -> Option<PathBuf> {
        let d = std::env::var_os("RSEMU_SGX_FW").map(PathBuf::from);
        if d.is_none() {
            println!("RSEMU_SGX_FW unset; skipping");
        }
        d
    }

    const IMAGES: [(&str, &str, u32); 2] = [
        ("ukernel_main_use.loaded.bin", "main.dis", 0xf200_0000),
        ("ukernel_slave_use.loaded.bin", "slave.dis", 0xf201_5000),
    ];

    fn words(image: &[u8]) -> impl Iterator<Item = (usize, u64)> + '_ {
        image
            .as_chunks::<8>()
            .0
            .iter()
            .enumerate()
            .map(|(i, c)| (i * 8, u64::from_le_bytes(*c)))
    }

    /// Whether a line may differ from the reference because the reference is
    /// wrong there. Two such classes are known, both in how it reads its
    /// measurements back rather than in the measurements: it adds the
    /// measuring input's register number to an indexed operand's offset
    /// (`sa[i1+2]` for a field of `0x60`), and it cannot rebuild a rotated
    /// wide immediate (`#0x8000ffff` for `0xffff` rotated by 16).
    fn reference_is_wrong(w: u64) -> bool {
        let Some(insn) = decode(w) else { return false };
        let indexed = insn
            .args()
            .iter()
            .any(|a| matches!(a.op, Operand::Indexed { .. }));
        let rotated = matches!(insn.op(), Op::Bitwise(_))
            && bit(w, 48)
            && field(w, 28, 2) == 2
            && field(w, 38, 5) != 0;
        indexed || rotated
    }

    /// Disassembles both microkernel images and compares, line by line, with
    /// the reference disassembler's listings. Every difference must be one of
    /// the reference's known errors.
    #[test]
    fn disassembly_matches_the_reference() {
        let Some(dir) = dir() else { return };
        let (mut total, mut same, mut excused) = (0usize, 0usize, 0usize);
        let mut unexplained = Vec::new();
        for (image, listing, base) in IMAGES {
            let image = std::fs::read(dir.join(image)).expect("image");
            let Ok(reference) = std::fs::read_to_string(dir.join(listing)) else {
                println!("{listing} missing; skipping the comparison");
                continue;
            };
            let reference: Vec<&str> = reference.lines().collect();
            for (off, w) in words(&image) {
                let pc = base + off as u32;
                let ours = alloc::format!(
                    "{pc:08x}: {:08x} {:08x}  {}",
                    w >> 32,
                    w as u32,
                    disassemble(w, pc, base)
                );
                let theirs = reference.get(off / 8).copied().unwrap_or("");
                total += 1;
                if ours == theirs {
                    same += 1;
                } else if reference_is_wrong(w) {
                    excused += 1;
                } else {
                    unexplained.push(alloc::format!("- {theirs}\n+ {ours}"));
                }
            }
        }
        if total == 0 {
            return;
        }
        println!(
            "disassembly: {same}/{total} lines identical ({:.2}%), {excused} differ where the \
             reference is wrong, {} unexplained",
            100.0 * same as f64 / total as f64,
            unexplained.len()
        );
        for diff in unexplained.iter().take(40) {
            println!("{diff}");
        }
        assert!(unexplained.is_empty());
    }

    /// Every word of both images decodes, but for the six known unknowns.
    #[test]
    fn microkernel_words_decode() {
        let Some(dir) = dir() else { return };
        let mut undecoded = 0;
        for (image, _, _) in IMAGES {
            let image = std::fs::read(dir.join(image)).expect("image");
            undecoded += words(&image).filter(|&(_, w)| decode(w).is_none()).count();
        }
        println!("undecodable words: {undecoded}");
        assert!(undecoded <= 6, "{undecoded} undecodable words");
    }

    /// Serves the image read-only, everything else as zero-initialised RAM,
    /// and records what the program touches.
    struct ImageBus {
        base: u32,
        image: Vec<u8>,
        ram: BTreeMap<u32, u32>,
        regs: BTreeMap<u32, u32>,
        log: Vec<String>,
        counts: BTreeMap<&'static str, usize>,
    }

    impl ImageBus {
        fn note(&mut self, kind: &'static str, text: String) {
            *self.counts.entry(kind).or_default() += 1;
            // A polling loop would fill the log with one line.
            if self.log.len() < 200 && self.log.last() != Some(&text) {
                self.log.push(text);
            }
        }

        fn word(&self, addr: u32) -> u32 {
            if let Some(v) = self.ram.get(&addr) {
                return *v;
            }
            let off = addr.wrapping_sub(self.base) as usize;
            match self.image.get(off..off + 4) {
                Some(b) => u32::from_le_bytes(b.try_into().unwrap()),
                None => 0,
            }
        }
    }

    impl UsseBus for ImageBus {
        fn fetch(&mut self, addr: u32) -> MemResult<u64> {
            Ok(u64::from(self.word(addr + 4)) << 32 | u64::from(self.word(addr)))
        }

        fn load(&mut self, addr: u32, size: Size) -> MemResult<u32> {
            let v = self.word(addr & !3) >> ((addr & 3) * 8);
            let v = match size {
                Size::Byte => v & 0xff,
                Size::Word => v & 0xffff,
                Size::Dword => v,
            };
            self.note("ld", alloc::format!("ld{size:?} [{addr:#010x}] -> {v:#x}"));
            Ok(v)
        }

        fn store(&mut self, addr: u32, size: Size, value: u32) -> MemResult {
            self.note(
                "st",
                alloc::format!("st{size:?} [{addr:#010x}] <- {value:#x}"),
            );
            let old = self.word(addr & !3);
            let shift = (addr & 3) * 8;
            let mask = match size {
                Size::Byte => 0xffu32 << shift,
                Size::Word => 0xffffu32 << shift,
                Size::Dword => !0,
            };
            self.ram
                .insert(addr & !3, old & !mask | (value << shift) & mask);
            Ok(())
        }

        fn reg_read(&mut self, index: u32) -> u32 {
            let v = self.regs.get(&index).copied().unwrap_or(0);
            self.note(
                "ldr",
                alloc::format!("ldr reg {:#06x} -> {v:#x}", index * 4),
            );
            v
        }

        fn reg_write(&mut self, index: u32, value: u32) {
            self.note(
                "str",
                alloc::format!("str reg {:#06x} <- {value:#x}", index * 4),
            );
            self.regs.insert(index, value);
        }

        fn emit(&mut self, emit: &Emit) -> MemResult {
            self.note("emit", alloc::format!("emit {emit:x?}"));
            Ok(())
        }
    }

    /// Runs the main microkernel from its start PC (core 0's start register
    /// holds 0x482, i.e. 0xf2002410) against a bus that serves the image, and
    /// reports how far it gets. A phase end continues at the declared next
    /// phase, as the hardware would schedule it; an unimplemented instruction
    /// is logged and stepped over, so one run surveys the missing semantics
    /// on the path rather than only the first. Never fails on what it finds.
    #[test]
    fn microkernel_runs() {
        let Some(dir) = dir() else { return };
        let (image, _, base) = IMAGES[0];
        let image = std::fs::read(dir.join(image)).expect("image");
        let mut bus = ImageBus {
            base,
            image,
            ram: BTreeMap::new(),
            regs: BTreeMap::new(),
            log: Vec::new(),
            counts: BTreeMap::new(),
        };
        let mut usse = Usse::new(base, 0x482);
        let budget = std::env::var("RSEMU_SGX_BUDGET")
            .ok()
            .and_then(|s| s.parse().ok())
            .unwrap_or(200_000u64);
        let mut hits: BTreeMap<u32, u64> = BTreeMap::new();
        let mut stops = Vec::new();
        let mut steps = 0u64;
        while steps < budget && stops.len() < 64 {
            steps += 1;
            *hits.entry(usse.pc).or_default() += 1;
            let Err(stop) = usse.step(&mut bus) else {
                continue;
            };
            stops.push(stop);
            match stop {
                Stop::Phase {
                    next: Some(next), ..
                } => usse.pc = next,
                Stop::Phase { next: None, pc } | Stop::End { pc } => usse.pc = pc,
                Stop::Unimplemented { pc, .. } => usse.pc = pc + 1,
                _ => break,
            }
        }
        println!(
            "microkernel: {} instructions retired in {steps} steps, {} distinct PCs",
            usse.retired,
            hits.len()
        );
        let describe = |pc: u32| {
            let addr = usse.address_of(pc);
            let off = addr.wrapping_sub(base) as usize;
            let w = bus
                .image
                .get(off..off + 8)
                .map_or(0, |b| u64::from_le_bytes(b.try_into().unwrap()));
            alloc::format!("{addr:08x}: {}", disassemble(w, addr, base))
        };
        for stop in &stops {
            let at = match *stop {
                Stop::End { .. } | Stop::Budget { .. } => None,
                Stop::Phase { pc, .. }
                | Stop::Unimplemented { pc, .. }
                | Stop::Undecodable { pc, .. }
                | Stop::BadRegister { pc, .. }
                | Stop::Fault { pc, .. } => Some(pc),
            };
            match at {
                Some(pc) => println!("  stop: {stop:x?}  [{}]", describe(pc)),
                None => println!("  stop: {stop:x?}"),
            }
        }
        let mut hot: Vec<_> = hits.iter().map(|(&pc, &n)| (n, pc)).collect();
        hot.sort_unstable_by(|a, b| b.cmp(a));
        println!("  hottest:");
        for (n, pc) in hot.iter().take(12) {
            println!("    {n:>8}  {}", describe(*pc));
        }
        println!("  final: {usse:?}");
        println!("  accesses: {:?}", bus.counts);
        for line in bus.log.iter().take(60) {
            println!("    {line}");
        }
    }
}
