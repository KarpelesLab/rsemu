//! A small x86-64 assembler: the instructions this backend actually emits.
//!
//! Safe Rust that appends to a `Vec<u8>`. Nothing here maps, protects or
//! executes anything — that is [`buf`](super::buf) — so a mistake in this file
//! is a wrong instruction rather than an unsound one, which is why the two are
//! separate.
//!
//! # Scope
//!
//! Deliberately not a general assembler. The encodings below are the ones
//! [`compile`](mod@super::compile) emits and no others, and each is transcribed
//! from the *Intel 64 and IA-32 Architectures Software Developer's Manual*,
//! volume 2 — the instruction-set reference, which describes the hardware
//! rather than anybody's implementation of it (CLAUDE.md, "Provenance").
//! Nothing outside the 1985 baseline plus 64-bit mode is used: no `popcnt`, no
//! `lzcnt`, no BMI. Those are extensions a host may lack, and a code generator
//! that faults on someone else's laptop is worse than one that emits four more
//! instructions — so [`compile`](mod@super::compile) open-codes population count
//! and derives leading and trailing zero counts from `BSR`/`BSF`, which have
//! been there since the 386.
//!
//! # Addressing
//!
//! One memory form: `[base + disp32]`. Every operand this backend touches is a
//! field of the execution context or a slot in the temporary frame, both of
//! which are exactly that. `mod = 2` with a full 32-bit displacement is used
//! even for a zero offset, because it removes the two encoding special cases
//! (`rbp`/`r13` at `mod = 0`, and the short form) at a cost of three bytes on
//! an instruction the processor decodes just as fast.

use alloc::vec::Vec;

/// A general-purpose register, by its architectural number.
///
/// The numbering is the encoding: `Reg as u8` is what goes in a ModRM field,
/// with bit 3 travelling in the REX prefix.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
#[allow(clippy::upper_case_acronyms, dead_code)]
pub enum Reg {
    /// The accumulator, and this backend's first scratch.
    Rax = 0,
    /// The count register — the only one a variable shift may take its amount
    /// from, which is why it is reserved for one.
    Rcx = 1,
    /// The second scratch.
    Rdx = 2,
    /// The execution context pointer, callee-saved.
    Rbx = 3,
    /// The stack pointer.
    Rsp = 4,
    /// Callee-saved, unused.
    Rbp = 5,
    /// Scratch, and the second argument register.
    Rsi = 6,
    /// Scratch, and the first argument register.
    Rdi = 7,
    /// Scratch, and the fifth argument register.
    R8 = 8,
    /// Scratch.
    R9 = 9,
    /// Scratch.
    R10 = 10,
    /// Scratch.
    R11 = 11,
    /// The temporary frame's base, callee-saved.
    R12 = 12,
    /// The host pointer, callee-saved.
    R13 = 13,
    /// The thunk table's base, callee-saved.
    R14 = 14,
    /// The software TLB's load set, callee-saved.
    R15 = 15,
}

impl Reg {
    #[inline]
    const fn low(self) -> u8 {
        (self as u8) & 7
    }
    #[inline]
    const fn high(self) -> u8 {
        (self as u8) >> 3
    }
}

/// A condition, by the low nibble of its `Jcc`/`SETcc`/`CMOVcc` opcode.
///
/// *Intel SDM* volume 2, appendix B's condition-code table.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
#[allow(dead_code)]
pub enum Cc {
    /// Below — unsigned less than. `CF = 1`.
    B = 0x2,
    /// Above or equal — unsigned greater or equal. `CF = 0`.
    Ae = 0x3,
    /// Equal. `ZF = 1`.
    E = 0x4,
    /// Not equal. `ZF = 0`.
    Ne = 0x5,
    /// Below or equal — unsigned less or equal.
    Be = 0x6,
    /// Above — unsigned greater than.
    A = 0x7,
    /// Less — signed less than.
    L = 0xc,
    /// Greater or equal — signed.
    Ge = 0xd,
    /// Less or equal — signed.
    Le = 0xe,
    /// Greater — signed.
    G = 0xf,
}

/// The `/digit` opcode extension of a group-1 arithmetic operation.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
#[allow(dead_code)]
pub enum Alu {
    /// `ADD`.
    Add = 0,
    /// `OR`.
    Or = 1,
    /// `AND`.
    And = 4,
    /// `SUB`.
    Sub = 5,
    /// `XOR`.
    Xor = 6,
    /// `CMP`.
    Cmp = 7,
}

impl Alu {
    /// The `r/m, r` opcode for this operation — `01`, `09`, `21`, `29`, `31`,
    /// `39`, which are `digit * 8 + 1`.
    #[inline]
    const fn rr_opcode(self) -> u8 {
        (self as u8) * 8 + 1
    }
}

/// The `/digit` extension of a shift or rotate.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
#[allow(dead_code)]
pub enum Shift {
    /// `ROL`.
    Rol = 0,
    /// `ROR`.
    Ror = 1,
    /// `SHL`.
    Shl = 4,
    /// `SHR`.
    Shr = 5,
    /// `SAR`.
    Sar = 7,
}

/// Where a `rel32` displacement was left, to be filled in later.
///
/// A forward jump names a place the assembler has not reached; this is the
/// four bytes it left behind. [`Asm::bind`] fills one in with the current
/// position, and [`Asm::bind_to`] with a recorded one.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Fixup(usize);

/// The assembler.
#[derive(Debug, Default)]
pub struct Asm {
    code: Vec<u8>,
}

impl Asm {
    /// An empty assembler.
    #[must_use]
    pub fn new() -> Asm {
        Asm::default()
    }

    /// The bytes emitted so far.
    #[inline]
    #[must_use]
    pub fn code(&self) -> &[u8] {
        &self.code
    }

    /// Take the bytes.
    #[must_use]
    pub fn finish(self) -> Vec<u8> {
        self.code
    }

    /// The current position, which is where the next byte lands.
    #[inline]
    #[must_use]
    pub fn here(&self) -> usize {
        self.code.len()
    }

    // ---- primitives ----------------------------------------------------

    #[inline]
    fn byte(&mut self, b: u8) {
        self.code.push(b);
    }

    #[inline]
    fn imm32(&mut self, v: i32) {
        self.code.extend_from_slice(&v.to_le_bytes());
    }

    /// A REX prefix, emitted only when it says something.
    ///
    /// `W` is 64-bit operand size, `R` extends the ModRM `reg` field, `B`
    /// extends `rm`. An all-zero REX still has to be emitted for a byte
    /// operation on `sil`/`dil`/`spl`/`bpl`, which is why [`Asm::setcc`] asks
    /// for one explicitly.
    #[inline]
    fn rex(&mut self, w: bool, r: u8, b: u8) {
        let bits = (u8::from(w) << 3) | (r << 2) | b;
        if bits != 0 {
            self.byte(0x40 | bits);
        }
    }

    /// A register-to-register ModRM byte.
    #[inline]
    fn modrm_rr(&mut self, reg: u8, rm: u8) {
        self.byte(0xc0 | (reg << 3) | rm);
    }

    /// A `[base + disp32]` ModRM, with the SIB byte `rsp`/`r12` needs.
    fn modrm_mem(&mut self, reg: u8, base: Reg, disp: i32) {
        self.byte(0x80 | (reg << 3) | base.low());
        if base.low() == 4 {
            // `rm = 100` means "there is a SIB byte"; 0x24 is index=none,
            // base=rsp/r12, scale 1 — that is, plain `[base + disp]`.
            self.byte(0x24);
        }
        self.imm32(disp);
    }

    // ---- moves ----------------------------------------------------------

    /// `push r64`.
    pub fn push(&mut self, r: Reg) {
        self.rex(false, 0, r.high());
        self.byte(0x50 + r.low());
    }

    /// `pop r64`.
    pub fn pop(&mut self, r: Reg) {
        self.rex(false, 0, r.high());
        self.byte(0x58 + r.low());
    }

    /// `ret`.
    pub fn ret(&mut self) {
        self.byte(0xc3);
    }

    /// `mov r64, imm64`.
    ///
    /// Always the ten-byte form. A code generator that picked the shortest
    /// encoding per constant would be a second thing to get wrong for a gain
    /// the instruction decoder does not care about.
    pub fn mov_ri(&mut self, dst: Reg, value: u64) {
        self.rex(true, 0, dst.high());
        self.byte(0xb8 + dst.low());
        self.code.extend_from_slice(&value.to_le_bytes());
    }

    /// `mov r32, imm32`, which zero-extends into the whole 64-bit register.
    ///
    /// Half the bytes of [`Asm::mov_ri`] for the small non-negative constants
    /// generated code is full of — a slot number, an instruction index — and
    /// the zero extension is what makes it a whole answer rather than a
    /// truncation the caller has to think about.
    pub fn mov_ri32(&mut self, dst: Reg, value: u32) {
        self.rex(false, 0, dst.high());
        self.byte(0xb8 + dst.low());
        self.code.extend_from_slice(&value.to_le_bytes());
    }

    /// `mov dst, src`, 64-bit.
    pub fn mov_rr(&mut self, dst: Reg, src: Reg) {
        self.rex(true, src.high(), dst.high());
        self.byte(0x89);
        self.modrm_rr(src.low(), dst.low());
    }

    /// `mov dst32, src32` — which zero-extends into the whole 64-bit register,
    /// and is therefore how a value is masked to 32 bits.
    pub fn mov_rr32(&mut self, dst: Reg, src: Reg) {
        self.rex(false, src.high(), dst.high());
        self.byte(0x89);
        self.modrm_rr(src.low(), dst.low());
    }

    /// `mov dst, [base + disp]`, 64-bit.
    pub fn mov_rm(&mut self, dst: Reg, base: Reg, disp: i32) {
        self.rex(true, dst.high(), base.high());
        self.byte(0x8b);
        self.modrm_mem(dst.low(), base, disp);
    }

    /// `mov [base + disp], src`, 64-bit.
    pub fn mov_mr(&mut self, base: Reg, disp: i32, src: Reg) {
        self.rex(true, src.high(), base.high());
        self.byte(0x89);
        self.modrm_mem(src.low(), base, disp);
    }

    /// `mov qword [base + disp], imm32`, sign-extended.
    pub fn mov_mi(&mut self, base: Reg, disp: i32, value: i32) {
        self.rex(true, 0, base.high());
        self.byte(0xc7);
        self.modrm_mem(0, base, disp);
        self.imm32(value);
    }

    /// `mov byte [base + disp], imm8`.
    pub fn mov_mi8(&mut self, base: Reg, disp: i32, value: u8) {
        self.rex(false, 0, base.high());
        self.byte(0xc6);
        self.modrm_mem(0, base, disp);
        self.byte(value);
    }

    /// A zero-extending load of `bytes` bytes from `[base + disp]`.
    ///
    /// One, two and four use `MOVZX`/32-bit `MOV`, all of which clear the upper
    /// half of the destination; eight is an ordinary 64-bit `MOV`.
    ///
    /// # Panics
    ///
    /// On a width that is not 1, 2, 4 or 8 — a caller that has not checked is
    /// a code-generator bug rather than a guest one.
    pub fn load_zx(&mut self, dst: Reg, base: Reg, disp: i32, bytes: u64) {
        match bytes {
            1 => {
                self.rex(false, dst.high(), base.high());
                self.byte(0x0f);
                self.byte(0xb6);
                self.modrm_mem(dst.low(), base, disp);
            }
            2 => {
                self.rex(false, dst.high(), base.high());
                self.byte(0x0f);
                self.byte(0xb7);
                self.modrm_mem(dst.low(), base, disp);
            }
            4 => {
                self.rex(false, dst.high(), base.high());
                self.byte(0x8b);
                self.modrm_mem(dst.low(), base, disp);
            }
            8 => self.mov_rm(dst, base, disp),
            _ => panic!("a guest access is 1, 2, 4 or 8 bytes wide"),
        }
    }

    // ---- arithmetic -----------------------------------------------------

    /// `op dst, src`, 64-bit.
    pub fn alu_rr(&mut self, op: Alu, dst: Reg, src: Reg) {
        self.rex(true, src.high(), dst.high());
        self.byte(op.rr_opcode());
        self.modrm_rr(src.low(), dst.low());
    }

    /// `op dst, imm32`, 64-bit with the immediate sign-extended.
    pub fn alu_ri(&mut self, op: Alu, dst: Reg, value: i32) {
        self.rex(true, 0, dst.high());
        self.byte(0x81);
        self.modrm_rr(op as u8, dst.low());
        self.imm32(value);
    }

    /// `op qword [base + disp], imm32`, 64-bit with the immediate
    /// sign-extended — a read-modify-write in one instruction.
    ///
    /// What a counter in the execution context is bumped with. The three
    /// instruction form it replaces also destroyed a scratch register, which
    /// is the part that stopped being free once the allocator wanted them.
    pub fn alu_mi(&mut self, op: Alu, base: Reg, disp: i32, value: i32) {
        self.rex(true, 0, base.high());
        self.byte(0x81);
        self.modrm_mem(op as u8, base, disp);
        self.imm32(value);
    }

    /// `op dst, [base + disp]`, 64-bit — the `r, r/m` direction.
    ///
    /// The `r/m, r` opcodes are `digit * 8 + 1`, so these are `digit * 8 + 3`.
    pub fn alu_rm(&mut self, op: Alu, dst: Reg, base: Reg, disp: i32) {
        self.rex(true, dst.high(), base.high());
        self.byte((op as u8) * 8 + 3);
        self.modrm_mem(dst.low(), base, disp);
    }

    /// `test dst, src`, 64-bit.
    pub fn test_rr(&mut self, dst: Reg, src: Reg) {
        self.rex(true, src.high(), dst.high());
        self.byte(0x85);
        self.modrm_rr(src.low(), dst.low());
    }

    /// `test dst32, imm32` — enough for the "is this bit set" tests, and one
    /// byte shorter than the 64-bit form on a value already known to be small.
    pub fn test_ri32(&mut self, dst: Reg, value: i32) {
        self.rex(false, 0, dst.high());
        self.byte(0xf7);
        self.modrm_rr(0, dst.low());
        self.imm32(value);
    }

    /// `not r64`.
    pub fn not(&mut self, r: Reg) {
        self.group3(2, r);
    }

    /// `neg r64`.
    pub fn neg(&mut self, r: Reg) {
        self.group3(3, r);
    }

    /// `mul r64` — `rdx:rax = rax * r`, unsigned.
    pub fn mul(&mut self, r: Reg) {
        self.group3(4, r);
    }

    /// `imul r64` — `rdx:rax = rax * r`, signed.
    pub fn imul1(&mut self, r: Reg) {
        self.group3(5, r);
    }

    fn group3(&mut self, digit: u8, r: Reg) {
        self.rex(true, 0, r.high());
        self.byte(0xf7);
        self.modrm_rr(digit, r.low());
    }

    /// `imul dst, src`, 64-bit, keeping the low half.
    pub fn imul_rr(&mut self, dst: Reg, src: Reg) {
        self.rex(true, dst.high(), src.high());
        self.byte(0x0f);
        self.byte(0xaf);
        self.modrm_rr(dst.low(), src.low());
    }

    /// `movsxd dst, src32` — sign-extend a 32-bit register into 64 bits.
    pub fn movsxd(&mut self, dst: Reg, src: Reg) {
        self.rex(true, dst.high(), src.high());
        self.byte(0x63);
        self.modrm_rr(dst.low(), src.low());
    }

    /// `bsr dst, src` — index of the highest set bit; `ZF` is set when `src`
    /// is zero, and `dst` is then undefined.
    pub fn bsr(&mut self, dst: Reg, src: Reg) {
        self.bit_scan(0xbd, dst, src);
    }

    /// `bsf dst, src` — index of the lowest set bit; see [`Asm::bsr`].
    pub fn bsf(&mut self, dst: Reg, src: Reg) {
        self.bit_scan(0xbc, dst, src);
    }

    fn bit_scan(&mut self, opcode: u8, dst: Reg, src: Reg) {
        self.rex(true, dst.high(), src.high());
        self.byte(0x0f);
        self.byte(opcode);
        self.modrm_rr(dst.low(), src.low());
    }

    /// `bswap r64` — reverse all eight bytes.
    pub fn bswap64(&mut self, r: Reg) {
        self.rex(true, 0, r.high());
        self.byte(0x0f);
        self.byte(0xc8 + r.low());
    }

    /// `bswap r32` — reverse the low four bytes, clearing the upper half.
    pub fn bswap32(&mut self, r: Reg) {
        self.rex(false, 0, r.high());
        self.byte(0x0f);
        self.byte(0xc8 + r.low());
    }

    // ---- shifts ---------------------------------------------------------

    /// `op r64, imm8`.
    pub fn shift_ri(&mut self, op: Shift, r: Reg, amount: u8) {
        self.rex(true, 0, r.high());
        self.byte(0xc1);
        self.modrm_rr(op as u8, r.low());
        self.byte(amount);
    }

    /// `op r32, imm8` — a 32-bit shift or rotate, which also clears the upper
    /// half of the register.
    pub fn shift_ri32(&mut self, op: Shift, r: Reg, amount: u8) {
        self.rex(false, 0, r.high());
        self.byte(0xc1);
        self.modrm_rr(op as u8, r.low());
        self.byte(amount);
    }

    /// `op r64, cl`.
    pub fn shift_rcl(&mut self, op: Shift, r: Reg) {
        self.rex(true, 0, r.high());
        self.byte(0xd3);
        self.modrm_rr(op as u8, r.low());
    }

    /// `op r32, cl` — the 32-bit form, whose count is masked to five bits,
    /// which is exactly a rotate within 32 bits.
    pub fn shift_rcl32(&mut self, op: Shift, r: Reg) {
        self.rex(false, 0, r.high());
        self.byte(0xd3);
        self.modrm_rr(op as u8, r.low());
    }

    // ---- conditions -----------------------------------------------------

    /// `setcc r8`, then zero-extend the register.
    ///
    /// The REX prefix is forced, because without one the byte registers 4..7
    /// are `ah`/`ch`/`dh`/`bh` rather than `spl`/`bpl`/`sil`/`dil` — the
    /// classic way to write a condition into the wrong half of a register.
    pub fn setcc(&mut self, cc: Cc, r: Reg) {
        self.byte(0x40 | r.high());
        self.byte(0x0f);
        self.byte(0x90 + cc as u8);
        self.modrm_rr(0, r.low());
        // `movzx r64, r8`, so the value is a canonical 0 or 1.
        self.rex(true, r.high(), r.high());
        self.byte(0x0f);
        self.byte(0xb6);
        self.modrm_rr(r.low(), r.low());
    }

    /// `cmovcc dst, src`, 64-bit.
    pub fn cmovcc(&mut self, cc: Cc, dst: Reg, src: Reg) {
        self.rex(true, dst.high(), src.high());
        self.byte(0x0f);
        self.byte(0x40 + cc as u8);
        self.modrm_rr(dst.low(), src.low());
    }

    /// `jcc rel32`, to be filled in later.
    #[must_use]
    pub fn jcc(&mut self, cc: Cc) -> Fixup {
        self.byte(0x0f);
        self.byte(0x80 + cc as u8);
        let at = self.here();
        self.imm32(0);
        Fixup(at)
    }

    /// `jmp rel32`, to be filled in later.
    #[must_use]
    pub fn jmp(&mut self) -> Fixup {
        self.byte(0xe9);
        let at = self.here();
        self.imm32(0);
        Fixup(at)
    }

    /// `call [base + disp]` — an indirect call through the thunk table.
    pub fn call_m(&mut self, base: Reg, disp: i32) {
        self.rex(false, 0, base.high());
        self.byte(0xff);
        self.modrm_mem(2, base, disp);
    }

    /// Point a fixup at the current position.
    pub fn bind(&mut self, f: Fixup) {
        let here = self.here();
        self.bind_to(f, here);
    }

    /// Point a fixup at `target`.
    ///
    /// # Panics
    ///
    /// If the displacement does not fit in 32 bits, which would need a block
    /// four gibibytes of machine code long.
    pub fn bind_to(&mut self, f: Fixup, target: usize) {
        let from = f.0 + 4;
        let delta = i64::try_from(target).unwrap_or(i64::MAX) - i64::try_from(from).unwrap_or(0);
        let delta = i32::try_from(delta).expect("a translation block is not four gibibytes long");
        self.code[f.0..f.0 + 4].copy_from_slice(&delta.to_le_bytes());
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use alloc::vec;

    /// The encodings, against what an assembler produces for the same
    /// mnemonics. Transcribed from *Intel SDM* volume 2's tables, and worth
    /// having as bytes: an off-by-one in a REX bit is a wrong *register*, which
    /// a functional test finds only if that register happens to be live.
    #[test]
    fn the_encodings_are_the_manuals() {
        let mut a = Asm::new();
        a.mov_rr(Reg::Rax, Reg::Rcx);
        assert_eq!(a.code(), &[0x48, 0x89, 0xc8]);

        let mut a = Asm::new();
        a.mov_rr(Reg::R12, Reg::R13);
        assert_eq!(a.code(), &[0x4d, 0x89, 0xec]);

        let mut a = Asm::new();
        a.mov_ri(Reg::Rdx, 0x1122_3344_5566_7788);
        assert_eq!(
            a.code(),
            &[0x48, 0xba, 0x88, 0x77, 0x66, 0x55, 0x44, 0x33, 0x22, 0x11]
        );

        // `[r12 + disp32]` is the case that needs a SIB byte.
        let mut a = Asm::new();
        a.mov_rm(Reg::Rax, Reg::R12, 0x18);
        assert_eq!(a.code(), &[0x49, 0x8b, 0x84, 0x24, 0x18, 0x00, 0x00, 0x00]);

        let mut a = Asm::new();
        a.alu_rr(Alu::Add, Reg::Rax, Reg::Rcx);
        assert_eq!(a.code(), &[0x48, 0x01, 0xc8]);
        let mut a = Asm::new();
        a.alu_rr(Alu::Cmp, Reg::Rsi, Reg::Rdi);
        assert_eq!(a.code(), &[0x48, 0x39, 0xfe]);

        let mut a = Asm::new();
        a.shift_ri(Shift::Sar, Reg::Rax, 63);
        assert_eq!(a.code(), &[0x48, 0xc1, 0xf8, 0x3f]);

        let mut a = Asm::new();
        a.imul_rr(Reg::Rax, Reg::Rcx);
        assert_eq!(a.code(), &[0x48, 0x0f, 0xaf, 0xc1]);

        let mut a = Asm::new();
        a.bswap64(Reg::Rax);
        assert_eq!(a.code(), &[0x48, 0x0f, 0xc8]);

        // The forced REX on `setcc`: `sil`, not `dh`.
        let mut a = Asm::new();
        a.setcc(Cc::E, Reg::Rsi);
        assert_eq!(a.code(), &[0x40, 0x0f, 0x94, 0xc6, 0x48, 0x0f, 0xb6, 0xf6]);

        let mut a = Asm::new();
        a.call_m(Reg::R14, 0x10);
        assert_eq!(a.code(), &[0x41, 0xff, 0x96, 0x10, 0x00, 0x00, 0x00]);

        // `mov esi, 7` — no REX, and the zero extension is the encoding's.
        let mut a = Asm::new();
        a.mov_ri32(Reg::Rsi, 7);
        assert_eq!(a.code(), &[0xbe, 0x07, 0x00, 0x00, 0x00]);
        let mut a = Asm::new();
        a.mov_ri32(Reg::R9, 0x1234);
        assert_eq!(a.code(), &[0x41, 0xb9, 0x34, 0x12, 0x00, 0x00]);

        // `add qword [rbx + 0x88], 1`.
        let mut a = Asm::new();
        a.alu_mi(Alu::Add, Reg::Rbx, 0x88, 1);
        assert_eq!(
            a.code(),
            &[
                0x48, 0x81, 0x83, 0x88, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00
            ]
        );

        let mut a = Asm::new();
        a.load_zx(Reg::Rax, Reg::Rdx, 0, 1);
        assert_eq!(a.code(), &[0x0f, 0xb6, 0x82, 0x00, 0x00, 0x00, 0x00]);

        let mut a = Asm::new();
        a.alu_rm(Alu::Cmp, Reg::Rdx, Reg::Rcx, 0);
        assert_eq!(a.code(), &[0x48, 0x3b, 0x91, 0x00, 0x00, 0x00, 0x00]);
    }

    #[test]
    fn a_forward_jump_is_patched_to_where_it_lands() {
        let mut a = Asm::new();
        let f = a.jcc(Cc::E);
        a.ret();
        a.bind(f);
        a.ret();
        // `0f 84 rel32` then `c3`, so the displacement is one.
        assert_eq!(a.code(), &[0x0f, 0x84, 0x01, 0x00, 0x00, 0x00, 0xc3, 0xc3]);
    }

    #[test]
    fn a_backward_jump_is_a_negative_displacement() {
        let mut a = Asm::new();
        let target = a.here();
        a.ret();
        let f = a.jmp();
        a.bind_to(f, target);
        assert_eq!(a.code(), vec![0xc3, 0xe9, 0xfa, 0xff, 0xff, 0xff]);
    }
}
