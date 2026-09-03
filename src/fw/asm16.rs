//! A 16-bit x86 assembler that emits into a ROM image, written in Rust.
//!
//! # Why this exists
//!
//! [`crate::fw::pcbios`] is *guest* code: 16-bit real-mode x86 that a PC
//! fetches out of a ROM socket. `ROADMAP.md` §0 forbids a C toolchain in the
//! tree, and no external assembler may be required either — `cargo build` is
//! the whole build. Rust cannot target 16-bit x86 (LLVM's `x86_16` support has
//! never been a Rust target, and a `.code16` freestanding crate would need a
//! linker script and an assembler anyway), so the firmware is **emitted** by
//! this module rather than compiled: the BIOS is a Rust program that writes
//! machine code, and `cargo test` is enough to build and run it.
//!
//! The alternative designs and why they lost are in [`crate::fw`].
//!
//! # Source
//!
//! Every encoding here is from the *Intel 64 and IA-32 Architectures Software
//! Developer's Manual*, Volume 2 — the opcode maps in Appendix A and the
//! per-instruction encodings in Chapters 3-5 — plus §2.1 for the prefix bytes
//! and §2.1.5 (Table 2-1) for the 16-bit ModR/M addressing forms. No
//! assembler's source was consulted; the encodings are facts from the manual.
//!
//! # Shape
//!
//! One [`Asm`] owns a whole 64 KiB segment image and a cursor into it, because
//! a system BIOS is not a relocatable object: its reset vector is at a fixed
//! offset, its interrupt handlers are named by a table the same image builds,
//! and its data tables sit at addresses it hands to the guest. So a label is
//! an **absolute offset within the segment**, [`Asm::seek`] moves the cursor,
//! and [`Asm::finish`] resolves the fixups and hands back the bytes.
//!
//! Branches are always emitted in their *near* form (`E9 cw`, `0F 8x cw`),
//! never relaxed to the short one. Relaxation is an optimisation that costs
//! determinism — the same source must produce the same bytes — and the few
//! bytes it saves buy nothing in a 64 KiB image.

use alloc::vec;
use alloc::vec::Vec;

// ---------------------------------------------------------------------------
// operands
// ---------------------------------------------------------------------------

/// A 16-bit general register, by its three-bit encoding (SDM Vol. 2 Table 2-2).
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub struct R16(pub u8);

/// `AX`, and the accumulator every short-form instruction implies.
pub const AX: R16 = R16(0);
/// `CX`, the count register `LOOP` and the string prefixes use.
pub const CX: R16 = R16(1);
/// `DX`, which holds the port number for the register forms of `IN`/`OUT`.
pub const DX: R16 = R16(2);
/// `BX`, the only general register that is also a base register.
pub const BX: R16 = R16(3);
/// `SP`, the stack pointer.
pub const SP: R16 = R16(4);
/// `BP`, the frame pointer, which addresses `SS` by default.
pub const BP: R16 = R16(5);
/// `SI`, the source index for the string instructions.
pub const SI: R16 = R16(6);
/// `DI`, the destination index for the string instructions.
pub const DI: R16 = R16(7);

/// An 8-bit general register, by its three-bit encoding.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub struct R8(pub u8);

/// The low half of `AX`.
pub const AL: R8 = R8(0);
/// The low half of `CX`.
pub const CL: R8 = R8(1);
/// The low half of `DX`.
pub const DL: R8 = R8(2);
/// The low half of `BX`.
pub const BL: R8 = R8(3);
/// The high half of `AX`, which is where a BIOS call's function number lives.
pub const AH: R8 = R8(4);
/// The high half of `CX`.
pub const CH: R8 = R8(5);
/// The high half of `DX`.
pub const DH: R8 = R8(6);
/// The high half of `BX`.
pub const BH: R8 = R8(7);

/// A segment register, by its `MOV Sreg` encoding.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub struct Sreg(pub u8);

/// The extra segment, which the string instructions' destination always uses.
pub const ES: Sreg = Sreg(0);
/// The code segment.
pub const CS: Sreg = Sreg(1);
/// The stack segment.
pub const SS: Sreg = Sreg(2);
/// The data segment.
pub const DS: Sreg = Sreg(3);

/// The r/m sentinel [`Mem::abs`] uses: `mod=00 r/m=110` is a bare `disp16`,
/// which is the one form where `110` does *not* mean `[BP]` (SDM Table 2-1).
const RM_ABS: u8 = 0xff;

/// A 16-bit memory operand: a base/index pair, a displacement, and an optional
/// segment override.
#[derive(Clone, Copy, Debug)]
pub struct Mem {
    /// The `r/m` field, or [`RM_ABS`].
    rm: u8,
    /// The displacement, as a signed value so `mod=01` can be chosen.
    disp: i32,
    /// Whether a zero displacement must still be encoded — true for `[BP]`,
    /// whose `mod=00` encoding means `disp16` instead.
    force_disp: bool,
    /// A segment-override prefix, if the default segment is not wanted.
    seg: Option<Sreg>,
}

impl Mem {
    /// `[disp16]`, an absolute offset within the current data segment.
    #[must_use]
    pub fn abs(disp: u16) -> Mem {
        Mem {
            rm: RM_ABS,
            disp: i32::from(disp),
            force_disp: false,
            seg: None,
        }
    }

    /// `[BX+disp]`.
    #[must_use]
    pub fn bx(disp: i32) -> Mem {
        Mem::based(0b111, disp, false)
    }

    /// `[SI+disp]`.
    #[must_use]
    pub fn si(disp: i32) -> Mem {
        Mem::based(0b100, disp, false)
    }

    /// `[DI+disp]`.
    #[must_use]
    pub fn di(disp: i32) -> Mem {
        Mem::based(0b101, disp, false)
    }

    /// `[BP+disp]`, which addresses `SS` — the form an interrupt handler reads
    /// its caller's flags word through.
    #[must_use]
    pub fn bp(disp: i32) -> Mem {
        Mem::based(0b110, disp, true)
    }

    /// `[BX+SI+disp]`.
    #[must_use]
    pub fn bx_si(disp: i32) -> Mem {
        Mem::based(0b000, disp, false)
    }

    /// `[BX+DI+disp]`.
    #[must_use]
    pub fn bx_di(disp: i32) -> Mem {
        Mem::based(0b001, disp, false)
    }

    /// The same operand, reached through an explicit segment.
    #[must_use]
    pub fn seg(mut self, seg: Sreg) -> Mem {
        self.seg = Some(seg);
        self
    }

    fn based(rm: u8, disp: i32, force_disp: bool) -> Mem {
        Mem {
            rm,
            disp,
            force_disp,
            seg: None,
        }
    }
}

/// The ModR/M `r/m` operand: a register, or a memory reference.
#[derive(Clone, Copy, Debug)]
pub enum Rm {
    /// A register operand, `mod=11`.
    Reg(u8),
    /// A memory operand.
    Mem(Mem),
}

impl From<R16> for Rm {
    fn from(r: R16) -> Rm {
        Rm::Reg(r.0)
    }
}

impl From<R8> for Rm {
    fn from(r: R8) -> Rm {
        Rm::Reg(r.0)
    }
}

impl From<Mem> for Rm {
    fn from(m: Mem) -> Rm {
        Rm::Mem(m)
    }
}

/// The eight ALU operations that share the `00-3F` block of the opcode map,
/// in the order the map puts them (SDM Vol. 2 Appendix A).
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub struct Alu(pub u8);

impl Alu {
    /// `ADD`.
    pub const ADD: Alu = Alu(0);
    /// `OR`.
    pub const OR: Alu = Alu(1);
    /// `ADC`, add with carry.
    pub const ADC: Alu = Alu(2);
    /// `SBB`, subtract with borrow.
    pub const SBB: Alu = Alu(3);
    /// `AND`.
    pub const AND: Alu = Alu(4);
    /// `SUB`.
    pub const SUB: Alu = Alu(5);
    /// `XOR`.
    pub const XOR: Alu = Alu(6);
    /// `CMP`, which is `SUB` that keeps only the flags.
    pub const CMP: Alu = Alu(7);
}

/// A condition code, as the low nibble of a `Jcc` opcode.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub struct Cc(pub u8);

impl Cc {
    /// Overflow.
    pub const O: Cc = Cc(0x0);
    /// Below — `CF=1`, the unsigned "less than".
    pub const B: Cc = Cc(0x2);
    /// Above or equal — `CF=0`.
    pub const AE: Cc = Cc(0x3);
    /// Equal, i.e. zero.
    pub const E: Cc = Cc(0x4);
    /// Not equal.
    pub const NE: Cc = Cc(0x5);
    /// Below or equal.
    pub const BE: Cc = Cc(0x6);
    /// Above.
    pub const A: Cc = Cc(0x7);
    /// Sign set.
    pub const S: Cc = Cc(0x8);
    /// Sign clear.
    pub const NS: Cc = Cc(0x9);
    /// Less, signed.
    pub const L: Cc = Cc(0xc);
    /// Greater or equal, signed.
    pub const GE: Cc = Cc(0xd);
    /// Greater, signed.
    pub const G: Cc = Cc(0xf);
}

/// The shift and rotate operations of the `C0`/`D0` group, by `/digit`.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub struct Shift(pub u8);

impl Shift {
    /// Rotate left.
    pub const ROL: Shift = Shift(0);
    /// Rotate right.
    pub const ROR: Shift = Shift(1);
    /// Shift left.
    pub const SHL: Shift = Shift(4);
    /// Shift right, unsigned.
    pub const SHR: Shift = Shift(5);
    /// Shift right, signed.
    pub const SAR: Shift = Shift(7);
}

// ---------------------------------------------------------------------------
// labels
// ---------------------------------------------------------------------------

/// A forward or backward reference to an offset in the image.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub struct Label(usize);

/// What a fixup writes when the label it names is known.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
enum Fix {
    /// A signed 16-bit displacement from the end of the field.
    Rel16,
    /// The label's absolute offset, as a word.
    Abs16,
}

/// One unresolved reference: where it is, what it names, and how it is written.
#[derive(Clone, Copy, Debug)]
struct Fixup {
    at: usize,
    label: Label,
    kind: Fix,
}

// ---------------------------------------------------------------------------
// the assembler
// ---------------------------------------------------------------------------

/// An image under construction, with a cursor and a label table.
#[derive(Debug)]
pub struct Asm {
    image: Vec<u8>,
    at: usize,
    labels: Vec<Option<u16>>,
    fixups: Vec<Fixup>,
    /// Set once anything is written past the end of the image, so
    /// [`Asm::finish`] can refuse rather than silently truncate.
    overflow: bool,
}

impl Asm {
    /// A blank image of `size` bytes, filled with `fill`.
    ///
    /// `0xff` is the honest fill for an erased ROM and the value an unmapped
    /// ISA read returns, so a jump into a gap is a stream of `INC`/`INC` rather
    /// than a plausible-looking `ADD [BX+SI], AL`.
    #[must_use]
    pub fn new(size: usize, fill: u8) -> Asm {
        Asm {
            image: vec![fill; size],
            at: 0,
            labels: Vec::new(),
            fixups: Vec::new(),
            overflow: false,
        }
    }

    /// A fresh label, not yet bound to an offset.
    pub fn label(&mut self) -> Label {
        self.labels.push(None);
        Label(self.labels.len() - 1)
    }

    /// Bind `label` to the cursor.
    ///
    /// # Panics
    ///
    /// If the label is already bound. A label bound twice is a bug in the
    /// firmware source, and the image would be silently wrong.
    pub fn bind(&mut self, label: Label) {
        assert!(
            self.labels[label.0].is_none(),
            "label {} bound twice",
            label.0
        );
        self.labels[label.0] = Some(self.here());
    }

    /// A label bound to the cursor, in one step.
    pub fn here_label(&mut self) -> Label {
        let l = self.label();
        self.bind(l);
        l
    }

    /// The cursor, as an offset within the segment.
    #[must_use]
    pub fn here(&self) -> u16 {
        self.at as u16
    }

    /// Move the cursor to `offset`.
    pub fn seek(&mut self, offset: u16) {
        self.at = offset as usize;
    }

    /// Where `label` ended up. `None` until it is bound.
    #[must_use]
    pub fn offset_of(&self, label: Label) -> Option<u16> {
        self.labels[label.0]
    }

    /// Resolve every fixup and hand back the image.
    ///
    /// # Panics
    ///
    /// If a label was referenced and never bound, or if anything was written
    /// past the end of the image. Both are firmware-source bugs that would
    /// otherwise ship as a broken ROM.
    #[must_use]
    pub fn finish(mut self) -> Vec<u8> {
        assert!(!self.overflow, "the image overflowed its socket");
        for f in core::mem::take(&mut self.fixups) {
            let target = self.labels[f.label.0]
                .unwrap_or_else(|| panic!("label {} referenced but never bound", f.label.0));
            let value = match f.kind {
                Fix::Rel16 => target.wrapping_sub((f.at as u16).wrapping_add(2)),
                Fix::Abs16 => target,
            };
            self.image[f.at] = value as u8;
            self.image[f.at + 1] = (value >> 8) as u8;
        }
        self.image
    }

    // -- raw emission -------------------------------------------------------

    /// Raw bytes at the cursor.
    pub fn db(&mut self, bytes: &[u8]) {
        for &b in bytes {
            if self.at < self.image.len() {
                self.image[self.at] = b;
                self.at += 1;
            } else {
                self.overflow = true;
                self.at += 1;
            }
        }
    }

    /// A little-endian word at the cursor.
    pub fn dw(&mut self, word: u16) {
        self.db(&word.to_le_bytes());
    }

    /// A word holding the offset `label` ends up at.
    pub fn dw_label(&mut self, label: Label) {
        self.fixups.push(Fixup {
            at: self.at,
            label,
            kind: Fix::Abs16,
        });
        self.dw(0);
    }

    /// `count` copies of `byte`.
    pub fn fill(&mut self, byte: u8, count: usize) {
        for _ in 0..count {
            self.db(&[byte]);
        }
    }

    // -- ModR/M -------------------------------------------------------------

    /// Emit `opcode` with a ModR/M byte whose `reg` field is `reg`, preceded by
    /// any segment-override prefix the operand asks for.
    fn encode(&mut self, opcode: &[u8], reg: u8, rm: impl Into<Rm>) {
        let rm = rm.into();
        if let Rm::Mem(m) = rm
            && let Some(s) = m.seg
        {
            // SDM Vol. 2 §2.1.1: the segment-override prefixes are 0x26 (ES),
            // 0x2e (CS), 0x36 (SS) and 0x3e (DS), and they precede the opcode.
            self.db(&[0x26 | (s.0 << 3)]);
        }
        self.db(opcode);
        match rm {
            Rm::Reg(r) => self.db(&[0xc0 | (reg << 3) | r]),
            Rm::Mem(m) if m.rm == RM_ABS => {
                self.db(&[(reg << 3) | 0b110]);
                self.dw(m.disp as u16);
            }
            Rm::Mem(m) => {
                let (mode, width) = if m.disp == 0 && !m.force_disp {
                    (0u8, 0)
                } else if (-128..=127).contains(&m.disp) {
                    (1, 1)
                } else {
                    (2, 2)
                };
                self.db(&[(mode << 6) | (reg << 3) | m.rm]);
                match width {
                    1 => self.db(&[m.disp as u8]),
                    2 => self.dw(m.disp as u16),
                    _ => {}
                }
            }
        }
    }

    /// The 66h operand-size prefix, then a ModR/M instruction: how 32-bit
    /// operands are reached from 16-bit code (SDM Vol. 2 §2.1.2).
    fn encode32(&mut self, opcode: &[u8], reg: u8, rm: impl Into<Rm>) {
        self.db(&[0x66]);
        self.encode(opcode, reg, rm);
    }

    // -- MOV ----------------------------------------------------------------

    /// `MOV r16, imm16` — `B8+rw iw`.
    pub fn movi(&mut self, dst: R16, imm: u16) {
        self.db(&[0xb8 | dst.0]);
        self.dw(imm);
    }

    /// `MOV r16, imm16` where the immediate is a label's offset.
    pub fn movi_label(&mut self, dst: R16, label: Label) {
        self.db(&[0xb8 | dst.0]);
        self.dw_label(label);
    }

    /// `MOV r32, imm32` — `66 B8+rw id`.
    pub fn movi32(&mut self, dst: R16, imm: u32) {
        self.db(&[0x66, 0xb8 | dst.0]);
        self.db(&imm.to_le_bytes());
    }

    /// `MOV r8, imm8` — `B0+rb ib`.
    pub fn movi8(&mut self, dst: R8, imm: u8) {
        self.db(&[0xb0 | dst.0, imm]);
    }

    /// `MOV r16, r/m16` — `8B /r`.
    pub fn mov(&mut self, dst: R16, src: impl Into<Rm>) {
        self.encode(&[0x8b], dst.0, src);
    }

    /// `MOV r/m16, r16` — `89 /r`.
    pub fn movto(&mut self, dst: impl Into<Rm>, src: R16) {
        self.encode(&[0x89], src.0, dst);
    }

    /// `MOV r32, r/m32` — `66 8B /r`.
    pub fn mov32(&mut self, dst: R16, src: impl Into<Rm>) {
        self.encode32(&[0x8b], dst.0, src);
    }

    /// `MOV r/m32, r32` — `66 89 /r`.
    pub fn movto32(&mut self, dst: impl Into<Rm>, src: R16) {
        self.encode32(&[0x89], src.0, dst);
    }

    /// `MOV r8, r/m8` — `8A /r`.
    pub fn mov8(&mut self, dst: R8, src: impl Into<Rm>) {
        self.encode(&[0x8a], dst.0, src);
    }

    /// `MOV r/m8, r8` — `88 /r`.
    pub fn movto8(&mut self, dst: impl Into<Rm>, src: R8) {
        self.encode(&[0x88], src.0, dst);
    }

    /// `MOV r/m16, imm16` — `C7 /0 iw`.
    pub fn movmi(&mut self, dst: impl Into<Rm>, imm: u16) {
        self.encode(&[0xc7], 0, dst);
        self.dw(imm);
    }

    /// `MOV r/m32, imm32` — `66 C7 /0 id`.
    pub fn movmi32(&mut self, dst: impl Into<Rm>, imm: u32) {
        self.encode32(&[0xc7], 0, dst);
        self.db(&imm.to_le_bytes());
    }

    /// `MOV r/m16, imm16` where the immediate is a label's offset.
    pub fn movmi_label(&mut self, dst: impl Into<Rm>, label: Label) {
        self.encode(&[0xc7], 0, dst);
        self.dw_label(label);
    }

    /// `MOV r/m8, imm8` — `C6 /0 ib`.
    pub fn movmi8(&mut self, dst: impl Into<Rm>, imm: u8) {
        self.encode(&[0xc6], 0, dst);
        self.db(&[imm]);
    }

    /// `MOV Sreg, r/m16` — `8E /r`.
    pub fn movsr(&mut self, dst: Sreg, src: impl Into<Rm>) {
        self.encode(&[0x8e], dst.0, src);
    }

    /// `MOV r/m16, Sreg` — `8C /r`.
    pub fn movrs(&mut self, dst: impl Into<Rm>, src: Sreg) {
        self.encode(&[0x8c], src.0, dst);
    }

    /// `LEA r16, m` — `8D /r`.
    pub fn lea(&mut self, dst: R16, src: Mem) {
        self.encode(&[0x8d], dst.0, src);
    }

    /// `XCHG AX, r16` — `90+rw`.
    pub fn xchg_ax(&mut self, other: R16) {
        self.db(&[0x90 | other.0]);
    }

    // -- ALU ----------------------------------------------------------------

    /// `<op> r16, r/m16` — the `03`-column of the ALU block.
    pub fn alu(&mut self, op: Alu, dst: R16, src: impl Into<Rm>) {
        self.encode(&[0x03 | (op.0 << 3)], dst.0, src);
    }

    /// `<op> r/m16, r16` — the `01`-column.
    pub fn aluto(&mut self, op: Alu, dst: impl Into<Rm>, src: R16) {
        self.encode(&[0x01 | (op.0 << 3)], src.0, dst);
    }

    /// `<op> r32, r/m32` — `66` and the `03`-column.
    pub fn alu32(&mut self, op: Alu, dst: R16, src: impl Into<Rm>) {
        self.encode32(&[0x03 | (op.0 << 3)], dst.0, src);
    }

    /// `<op> r8, r/m8` — the `02`-column.
    pub fn alu8(&mut self, op: Alu, dst: R8, src: impl Into<Rm>) {
        self.encode(&[0x02 | (op.0 << 3)], dst.0, src);
    }

    /// `<op> r/m8, r8` — the `00`-column, whose base opcode is zero.
    pub fn aluto8(&mut self, op: Alu, dst: impl Into<Rm>, src: R8) {
        self.encode(&[op.0 << 3], src.0, dst);
    }

    /// `<op> r/m16, imm16` — `81 /op iw`.
    ///
    /// Always the wide form, never `83 /op ib`: the short one is a
    /// size optimisation and this assembler does not relax.
    pub fn alui(&mut self, op: Alu, dst: impl Into<Rm>, imm: u16) {
        self.encode(&[0x81], op.0, dst);
        self.dw(imm);
    }

    /// `<op> r/m32, imm32` — `66 81 /op id`.
    pub fn alui32(&mut self, op: Alu, dst: impl Into<Rm>, imm: u32) {
        self.encode32(&[0x81], op.0, dst);
        self.db(&imm.to_le_bytes());
    }

    /// `<op> r/m8, imm8` — `80 /op ib`.
    pub fn alui8(&mut self, op: Alu, dst: impl Into<Rm>, imm: u8) {
        self.encode(&[0x80], op.0, dst);
        self.db(&[imm]);
    }

    /// `TEST r/m8, r8` — `84 /r`.
    pub fn test8(&mut self, a: impl Into<Rm>, b: R8) {
        self.encode(&[0x84], b.0, a);
    }

    /// `TEST r/m8, imm8` — `F6 /0 ib`.
    pub fn testi8(&mut self, a: impl Into<Rm>, imm: u8) {
        self.encode(&[0xf6], 0, a);
        self.db(&[imm]);
    }

    /// `TEST r/m16, imm16` — `F7 /0 iw`.
    pub fn testi(&mut self, a: impl Into<Rm>, imm: u16) {
        self.encode(&[0xf7], 0, a);
        self.dw(imm);
    }

    /// `TEST r/m16, r16` — `85 /r`.
    pub fn test(&mut self, a: impl Into<Rm>, b: R16) {
        self.encode(&[0x85], b.0, a);
    }

    /// `INC r16` — `40+rw`.
    pub fn inc(&mut self, r: R16) {
        self.db(&[0x40 | r.0]);
    }

    /// `DEC r16` — `48+rw`.
    pub fn dec(&mut self, r: R16) {
        self.db(&[0x48 | r.0]);
    }

    /// `INC r/m8` — `FE /0`.
    pub fn incm8(&mut self, dst: impl Into<Rm>) {
        self.encode(&[0xfe], 0, dst);
    }

    /// `DEC r/m8` — `FE /1`.
    pub fn decm8(&mut self, dst: impl Into<Rm>) {
        self.encode(&[0xfe], 1, dst);
    }

    /// `INC r/m16` — `FF /0`.
    pub fn incm(&mut self, dst: impl Into<Rm>) {
        self.encode(&[0xff], 0, dst);
    }

    /// `DEC r/m16` — `FF /1`.
    pub fn decm(&mut self, dst: impl Into<Rm>) {
        self.encode(&[0xff], 1, dst);
    }

    /// `INC r/m32` — `66 FF /0`.
    pub fn incm32(&mut self, dst: impl Into<Rm>) {
        self.encode32(&[0xff], 0, dst);
    }

    /// `NEG r/m16` — `F7 /3`.
    pub fn neg(&mut self, dst: impl Into<Rm>) {
        self.encode(&[0xf7], 3, dst);
    }

    /// `NOT r/m8` — `F6 /2`.
    pub fn not8(&mut self, dst: impl Into<Rm>) {
        self.encode(&[0xf6], 2, dst);
    }

    /// `NOT r/m16` — `F7 /2`.
    pub fn not(&mut self, dst: impl Into<Rm>) {
        self.encode(&[0xf7], 2, dst);
    }

    /// `MUL r/m16` — `F7 /4`: `DX:AX = AX * r/m16`.
    pub fn mul(&mut self, src: impl Into<Rm>) {
        self.encode(&[0xf7], 4, src);
    }

    /// `MUL r/m32` — `66 F7 /4`: `EDX:EAX = EAX * r/m32`.
    pub fn mul32(&mut self, src: impl Into<Rm>) {
        self.encode32(&[0xf7], 4, src);
    }

    /// `MUL r/m8` — `F6 /4`: `AX = AL * r/m8`.
    pub fn mul8(&mut self, src: impl Into<Rm>) {
        self.encode(&[0xf6], 4, src);
    }

    /// `DIV r/m16` — `F7 /6`: `AX = DX:AX / r/m16`, remainder in `DX`.
    pub fn div(&mut self, src: impl Into<Rm>) {
        self.encode(&[0xf7], 6, src);
    }

    /// `DIV r/m8` — `F6 /6`: `AL = AX / r/m8`, remainder in `AH`.
    pub fn div8(&mut self, src: impl Into<Rm>) {
        self.encode(&[0xf6], 6, src);
    }

    /// `CBW` — sign-extend `AL` into `AX`.
    pub fn cbw(&mut self) {
        self.db(&[0x98]);
    }

    /// `CWD` — sign-extend `AX` into `DX:AX`.
    pub fn cwd(&mut self) {
        self.db(&[0x99]);
    }

    /// `<shift> r/m16, imm8` — `C1 /op ib`.
    pub fn shift(&mut self, op: Shift, dst: impl Into<Rm>, count: u8) {
        self.encode(&[0xc1], op.0, dst);
        self.db(&[count]);
    }

    /// `<shift> r/m8, imm8` — `C0 /op ib`.
    pub fn shift8(&mut self, op: Shift, dst: impl Into<Rm>, count: u8) {
        self.encode(&[0xc0], op.0, dst);
        self.db(&[count]);
    }

    /// `<shift> r/m32, imm8` — `66 C1 /op ib`.
    pub fn shift32(&mut self, op: Shift, dst: impl Into<Rm>, count: u8) {
        self.encode32(&[0xc1], op.0, dst);
        self.db(&[count]);
    }

    // -- stack --------------------------------------------------------------

    /// `PUSH r16` — `50+rw`.
    pub fn push(&mut self, r: R16) {
        self.db(&[0x50 | r.0]);
    }

    /// `POP r16` — `58+rw`.
    pub fn pop(&mut self, r: R16) {
        self.db(&[0x58 | r.0]);
    }

    /// `PUSH Sreg`. `ES`/`CS`/`SS`/`DS` are one-byte opcodes on the `06` grid.
    pub fn pushs(&mut self, s: Sreg) {
        self.db(&[0x06 | (s.0 << 3)]);
    }

    /// `POP Sreg`. There is no `POP CS`, and asking for one is a source bug.
    ///
    /// # Panics
    ///
    /// If `s` is `CS`.
    pub fn pops(&mut self, s: Sreg) {
        assert!(s != CS, "there is no POP CS");
        self.db(&[0x07 | (s.0 << 3)]);
    }

    /// `PUSHA` — all eight general registers (80186 and later).
    pub fn pusha(&mut self) {
        self.db(&[0x60]);
    }

    /// `POPA`.
    pub fn popa(&mut self) {
        self.db(&[0x61]);
    }

    /// `PUSHF`.
    pub fn pushf(&mut self) {
        self.db(&[0x9c]);
    }

    /// `POPF`.
    pub fn popf(&mut self) {
        self.db(&[0x9d]);
    }

    /// `PUSHAD` — `66 60`, the eight 32-bit registers.
    pub fn pushad(&mut self) {
        self.db(&[0x66, 0x60]);
    }

    /// `POPAD` — `66 61`.
    pub fn popad(&mut self) {
        self.db(&[0x66, 0x61]);
    }

    /// `PUSH imm16` — `68 iw` (80186 and later).
    pub fn pushi(&mut self, imm: u16) {
        self.db(&[0x68]);
        self.dw(imm);
    }

    /// `PUSH imm16` where the immediate is a label's offset — how a near
    /// return address is put under a far jump into an option ROM.
    pub fn pushi_label(&mut self, label: Label) {
        self.db(&[0x68]);
        self.dw_label(label);
    }

    // -- control ------------------------------------------------------------

    /// `JMP rel16` — `E9 cw`.
    pub fn jmp(&mut self, label: Label) {
        self.db(&[0xe9]);
        self.rel16(label);
    }

    /// `Jcc rel16` — `0F 8x cw` (80386 and later).
    pub fn jcc(&mut self, cc: Cc, label: Label) {
        self.db(&[0x0f, 0x80 | cc.0]);
        self.rel16(label);
    }

    /// `CALL rel16` — `E8 cw`.
    pub fn call(&mut self, label: Label) {
        self.db(&[0xe8]);
        self.rel16(label);
    }

    /// `CALL r/m16` — `FF /2`, a near indirect call.
    pub fn call_rm(&mut self, target: impl Into<Rm>) {
        self.encode(&[0xff], 2, target);
    }

    /// `CALL m16:16` — `FF /3`, the far indirect call an option ROM is entered
    /// through (PCI Firmware Specification 3.0 §5.2.2).
    pub fn callf_m(&mut self, target: Mem) {
        self.encode(&[0xff], 3, target);
    }

    /// `JMP ptr16:16` — `EA cd`, the far jump the reset vector holds.
    pub fn jmpf(&mut self, segment: u16, offset: u16) {
        self.db(&[0xea]);
        self.dw(offset);
        self.dw(segment);
    }

    /// `JMP ptr16:16` where the offset is a label — the form the reset vector
    /// holds, since the label it names is assembled later.
    pub fn jmpf_label(&mut self, segment: u16, label: Label) {
        self.db(&[0xea]);
        self.dw_label(label);
        self.dw(segment);
    }

    /// `JMP m16:16` — `FF /5`, a far indirect jump.
    pub fn jmpf_m(&mut self, target: Mem) {
        self.encode(&[0xff], 5, target);
    }

    /// `RET` — `C3`.
    pub fn ret(&mut self) {
        self.db(&[0xc3]);
    }

    /// `RETF` — `CB`.
    pub fn retf(&mut self) {
        self.db(&[0xcb]);
    }

    /// `IRET` — `CF`.
    pub fn iret(&mut self) {
        self.db(&[0xcf]);
    }

    /// `INT imm8` — `CD ib`.
    pub fn int(&mut self, vector: u8) {
        self.db(&[0xcd, vector]);
    }

    /// `LOOP rel8` — `E2 cb`. Short-form only: the instruction has no near
    /// encoding, so the target must be within 128 bytes and [`Asm::finish`]
    /// would rather panic than emit a wrong displacement.
    pub fn loop_(&mut self, label: Label) {
        self.db(&[0xe2]);
        self.rel8(label);
    }

    /// `HLT`.
    pub fn hlt(&mut self) {
        self.db(&[0xf4]);
    }

    /// `NOP`.
    pub fn nop(&mut self) {
        self.db(&[0x90]);
    }

    /// `CLI`.
    pub fn cli(&mut self) {
        self.db(&[0xfa]);
    }

    /// `STI`.
    pub fn sti(&mut self) {
        self.db(&[0xfb]);
    }

    /// `CLD`.
    pub fn cld(&mut self) {
        self.db(&[0xfc]);
    }

    /// `STC`.
    pub fn stc(&mut self) {
        self.db(&[0xf9]);
    }

    /// `CLC`.
    pub fn clc(&mut self) {
        self.db(&[0xf8]);
    }

    // -- strings and I/O ----------------------------------------------------

    /// The `F3` prefix. Emitted as its own call, immediately before the string
    /// instruction it repeats.
    pub fn rep(&mut self) {
        self.db(&[0xf3]);
    }

    /// `MOVSB`.
    pub fn movsb(&mut self) {
        self.db(&[0xa4]);
    }

    /// `MOVSW`.
    pub fn movsw(&mut self) {
        self.db(&[0xa5]);
    }

    /// `STOSB`.
    pub fn stosb(&mut self) {
        self.db(&[0xaa]);
    }

    /// `STOSW`.
    pub fn stosw(&mut self) {
        self.db(&[0xab]);
    }

    /// `LODSB`.
    pub fn lodsb(&mut self) {
        self.db(&[0xac]);
    }

    /// `LODSW`.
    pub fn lodsw(&mut self) {
        self.db(&[0xad]);
    }

    /// `INSW` — a word from the port in `DX` to `ES:DI` (80186 and later).
    /// With a `REP` prefix this is how a sector leaves an IDE drive.
    pub fn insw(&mut self) {
        self.db(&[0x6d]);
    }

    /// `OUTSW` — a word from `DS:SI` to the port in `DX`.
    pub fn outsw(&mut self) {
        self.db(&[0x6f]);
    }

    /// `IN AL, imm8` — `E4 ib`.
    pub fn in_al(&mut self, port: u8) {
        self.db(&[0xe4, port]);
    }

    /// `IN AL, DX` — `EC`.
    pub fn in_al_dx(&mut self) {
        self.db(&[0xec]);
    }

    /// `IN AX, DX` — `ED`.
    pub fn in_ax_dx(&mut self) {
        self.db(&[0xed]);
    }

    /// `IN EAX, DX` — `66 ED`, the same opcode under the operand-size prefix.
    ///
    /// The only way a 16-bit BIOS reads a 32-bit port, and PCI configuration
    /// mechanism #1 has two of them (*PCI Local Bus Specification* §3.7.4.1:
    /// `CONFIG_ADDRESS` "can only be accessed as a Dword").
    pub fn in_eax_dx(&mut self) {
        self.db(&[0x66, 0xed]);
    }

    /// `OUT imm8, AL` — `E6 ib`.
    pub fn out_al(&mut self, port: u8) {
        self.db(&[0xe6, port]);
    }

    /// `OUT DX, AL` — `EE`.
    pub fn out_dx_al(&mut self) {
        self.db(&[0xee]);
    }

    /// `OUT DX, AX` — `EF`.
    pub fn out_dx_ax(&mut self) {
        self.db(&[0xef]);
    }

    /// `OUT DX, EAX` — `66 EF`.
    pub fn out_dx_eax(&mut self) {
        self.db(&[0x66, 0xef]);
    }

    // -- system -------------------------------------------------------------
    //
    // The three instructions a real-mode BIOS needs in order to borrow the
    // protected-mode machinery for the length of one service call, and no
    // more. Everything else about protection is the guest's business.

    /// `LGDT m16&32` — `0F 01 /2` (SDM Vol. 2, `LGDT/LIDT`).
    ///
    /// With no operand-size prefix the base is loaded from 24 bits and the top
    /// byte is cleared, which is the 16-bit form and exactly what a table in
    /// the first megabyte needs.
    pub fn lgdt(&mut self, table: Mem) {
        self.encode(&[0x0f, 0x01], 2, table);
    }

    /// `MOV r32, CR0` — `0F 20 /r` with the `reg` field naming the control
    /// register. The operand is 32 bits whatever the code size, so there is no
    /// `66` prefix (SDM Vol. 2, `MOV — Move to/from Control Registers`).
    pub fn read_cr0(&mut self, dst: R16) {
        self.encode(&[0x0f, 0x20], 0, dst);
    }

    /// `MOV CR0, r32` — `0F 22 /r`.
    pub fn write_cr0(&mut self, src: R16) {
        self.encode(&[0x0f, 0x22], 0, src);
    }

    // -- fixups -------------------------------------------------------------

    fn rel16(&mut self, label: Label) {
        self.fixups.push(Fixup {
            at: self.at,
            label,
            kind: Fix::Rel16,
        });
        self.dw(0);
    }

    /// A one-byte relative displacement, resolved immediately.
    ///
    /// Only backward references are allowed, because a short forward branch
    /// cannot be checked for range until the target is known and this
    /// assembler does not relax. Everything forward uses [`Asm::jmp`] or
    /// [`Asm::jcc`], which are near.
    ///
    /// # Panics
    ///
    /// If the label is unbound or out of range.
    fn rel8(&mut self, label: Label) {
        let target = self.labels[label.0].expect("a short branch must be backward");
        let from = (self.at as u16).wrapping_add(1);
        let delta = target.wrapping_sub(from) as i16;
        let delta = i32::from(delta);
        assert!(
            (-128..=127).contains(&delta),
            "short branch out of range: {delta}"
        );
        self.db(&[delta as u8]);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Assemble one instruction and hand back the bytes it produced.
    fn one(f: impl FnOnce(&mut Asm)) -> Vec<u8> {
        let mut a = Asm::new(64, 0x00);
        f(&mut a);
        let n = a.here() as usize;
        a.finish()[..n].to_vec()
    }

    #[test]
    fn the_encodings_match_the_opcode_map() {
        // Spot checks against SDM Vol. 2 Appendix A, chosen because each is a
        // different shape: short form, ModR/M with a register, ModR/M with an
        // absolute address, a group opcode with a /digit, and a prefix.
        assert_eq!(one(|a| a.movi(AX, 0x1234)), [0xb8, 0x34, 0x12]);
        assert_eq!(one(|a| a.movi8(AH, 0x0e)), [0xb4, 0x0e]);
        assert_eq!(one(|a| a.mov(BX, AX)), [0x8b, 0xd8]);
        // 89 /r with mod=00 r/m=110, which SDM Table 2-1 reads as a bare disp16
        // rather than as [BP].
        assert_eq!(
            one(|a| a.movto(Mem::abs(0x0410), AX)),
            [0x89, 0x06, 0x10, 0x04]
        );
        assert_eq!(one(|a| a.alui(Alu::CMP, AX, 3)), [0x81, 0xf8, 0x03, 0x00]);
        assert_eq!(one(|a| a.int(0x13)), [0xcd, 0x13]);
        assert_eq!(
            one(|a| a.jmpf(0xf000, 0xe05b)),
            [0xea, 0x5b, 0xe0, 0x00, 0xf0]
        );
        assert_eq!(
            one(|a| a.movi32(CX, 20)),
            [0x66, 0xb9, 0x14, 0x00, 0x00, 0x00]
        );
        // The three system instructions. `MOV r32, CR0` takes no `66` prefix:
        // its operand is 32 bits in every code size, so emitting one would be
        // a different instruction rather than a longer encoding of this one.
        assert_eq!(
            one(|a| a.lgdt(Mem::abs(0x0078))),
            [0x0f, 0x01, 0x16, 0x78, 0x00]
        );
        assert_eq!(one(|a| a.read_cr0(AX)), [0x0f, 0x20, 0xc0]);
        assert_eq!(one(|a| a.write_cr0(AX)), [0x0f, 0x22, 0xc0]);
        // The 32-bit port pair, which is the same opcode as the 16-bit one
        // under the operand-size prefix rather than an opcode of its own.
        assert_eq!(one(|a| a.in_ax_dx()), [0xed]);
        assert_eq!(one(|a| a.in_eax_dx()), [0x66, 0xed]);
        assert_eq!(one(|a| a.out_dx_ax()), [0xef]);
        assert_eq!(one(|a| a.out_dx_eax()), [0x66, 0xef]);
    }

    #[test]
    fn a_memory_operand_picks_the_narrowest_displacement() {
        // SDM Table 2-1: mod=00 has no displacement, except that r/m=110 means
        // a bare disp16 — so [BP] has to be encoded as [BP+0].
        assert_eq!(one(|a| a.mov(AX, Mem::bx(0))), [0x8b, 0x07]);
        assert_eq!(one(|a| a.mov(AX, Mem::bx(4))), [0x8b, 0x47, 0x04]);
        assert_eq!(one(|a| a.mov(AX, Mem::bx(0x200))), [0x8b, 0x87, 0x00, 0x02]);
        assert_eq!(one(|a| a.mov(AX, Mem::bp(0))), [0x8b, 0x46, 0x00]);
        assert_eq!(
            one(|a| a.mov(AX, Mem::abs(0x413))),
            [0x8b, 0x06, 0x13, 0x04]
        );
        // A segment override is a prefix, so it lands before the opcode.
        assert_eq!(one(|a| a.mov(AX, Mem::bx(0).seg(ES))), [0x26, 0x8b, 0x07]);
    }

    #[test]
    fn a_backward_branch_and_a_forward_one_both_resolve() {
        let mut a = Asm::new(64, 0x90);
        let top = a.here_label();
        let out = a.label();
        a.jcc(Cc::E, out); // 4 bytes at 0
        a.jmp(top); // 3 bytes at 4
        a.bind(out); // offset 7
        a.hlt();
        let bytes = a.finish();
        assert_eq!(
            &bytes[..8],
            &[0x0f, 0x84, 0x03, 0x00, 0xe9, 0xf9, 0xff, 0xf4]
        );
    }

    #[test]
    #[should_panic(expected = "referenced but never bound")]
    fn an_unbound_label_is_a_build_failure_rather_than_a_broken_rom() {
        let mut a = Asm::new(64, 0);
        let l = a.label();
        a.jmp(l);
        let _ = a.finish();
    }
}
