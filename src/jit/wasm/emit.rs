//! A WebAssembly binary encoder: LEB128, instructions, sections, module.
//!
//! Safe Rust over `alloc`, and — like [`jit::arm64::emit`](crate::jit::arm64)
//! — **not gated to any host**: encoding an instruction is arithmetic on
//! integers, so it compiles and is tested everywhere. Nothing here executes
//! anything.
//!
//! # The specification
//!
//! Every constant below is from the *WebAssembly Core Specification*
//! (W3C Recommendation, version 2.0), **§5 Binary Format**: §5.2.2 for the
//! integer encodings, §5.3 for the value and function types, §5.4 for the
//! instruction opcodes and §5.5 for the module and its sections. That document
//! is an open standard, which is the whole reason this backend could be
//! written at all (CLAUDE.md, "Provenance"): the alternative sources for "how
//! do I emit wasm" are copyleft toolchains.
//!
//! # The subset
//!
//! Deliberately the **MVP** instruction set — no sign-extension operators
//! (`i64.extend32_s` and friends, §5.4.6's `0xC0`–`0xC4`), no saturating
//! conversions, no bulk memory, no SIMD, no threads. A sign-extension is a
//! `shl`/`shr_s` pair here, which costs one extra instruction and buys
//! acceptance by every embedder that ever shipped, including the ones behind a
//! conservative content-security policy. `compile` never emits a float
//! instruction at all, which is not an accident either: guest floating point
//! is a helper call into `float::soft` (`ROADMAP.md` §9.1), so a guest's NaN
//! payloads and rounding can never become the host's.

use alloc::vec::Vec;

/// The eight bytes every module starts with: `\0asm`, then version 1.
///
/// Core specification §5.5.16.
pub const HEADER: [u8; 8] = [0x00, 0x61, 0x73, 0x6d, 0x01, 0x00, 0x00, 0x00];

/// Value types, as §5.3.1 encodes them.
pub mod ty {
    /// `i32`.
    pub const I32: u8 = 0x7f;
    /// `i64`.
    pub const I64: u8 = 0x7e;
}

/// Instruction opcodes, as §5.4 encodes them.
///
/// Only the ones [`compile`](mod@super::compile) emits. A backend that names an
/// opcode it does not emit is a backend with an untested constant in it.
#[allow(missing_docs)]
pub mod op {
    // §5.4.1 control instructions
    pub const BLOCK: u8 = 0x02;
    pub const IF: u8 = 0x04;
    pub const END: u8 = 0x0b;
    pub const BR: u8 = 0x0c;
    pub const BR_IF: u8 = 0x0d;
    pub const RETURN: u8 = 0x0f;
    pub const CALL: u8 = 0x10;
    /// The empty block type, `ϵ` — §5.3.3's `0x40`.
    pub const EMPTY: u8 = 0x40;

    // §5.4.2 parametric instructions
    pub const DROP: u8 = 0x1a;
    pub const SELECT: u8 = 0x1b;

    // §5.4.3 variable instructions
    pub const LOCAL_GET: u8 = 0x20;
    pub const LOCAL_SET: u8 = 0x21;
    pub const LOCAL_TEE: u8 = 0x22;

    // §5.4.4 memory instructions
    pub const I64_LOAD: u8 = 0x29;
    pub const I64_STORE: u8 = 0x37;

    // §5.4.5 numeric instructions — constants
    pub const I32_CONST: u8 = 0x41;
    pub const I64_CONST: u8 = 0x42;

    // §5.4.5 — i32 comparisons
    pub const I32_EQZ: u8 = 0x45;

    // §5.4.5 — i64 comparisons
    pub const I64_EQZ: u8 = 0x50;
    pub const I64_EQ: u8 = 0x51;
    pub const I64_NE: u8 = 0x52;
    pub const I64_LT_S: u8 = 0x53;
    pub const I64_LT_U: u8 = 0x54;
    pub const I64_GT_S: u8 = 0x55;
    pub const I64_GT_U: u8 = 0x56;
    pub const I64_LE_S: u8 = 0x57;
    pub const I64_LE_U: u8 = 0x58;
    pub const I64_GE_S: u8 = 0x59;
    pub const I64_GE_U: u8 = 0x5a;

    // §5.4.5 — i64 arithmetic
    pub const I64_CLZ: u8 = 0x79;
    pub const I64_CTZ: u8 = 0x7a;
    pub const I64_POPCNT: u8 = 0x7b;
    pub const I64_ADD: u8 = 0x7c;
    pub const I64_SUB: u8 = 0x7d;
    pub const I64_MUL: u8 = 0x7e;
    pub const I64_REM_U: u8 = 0x82;
    pub const I64_AND: u8 = 0x83;
    pub const I64_OR: u8 = 0x84;
    pub const I64_XOR: u8 = 0x85;
    pub const I64_SHL: u8 = 0x86;
    pub const I64_SHR_S: u8 = 0x87;
    pub const I64_SHR_U: u8 = 0x88;
    pub const I64_ROTL: u8 = 0x89;
    pub const I64_ROTR: u8 = 0x8a;

    // §5.4.5 — conversions
    pub const I64_EXTEND_I32_U: u8 = 0xad;
}

/// Append `v` as an unsigned LEB128, as §5.2.2 defines it.
pub fn uleb(out: &mut Vec<u8>, mut v: u64) {
    loop {
        let byte = (v & 0x7f) as u8;
        v >>= 7;
        if v == 0 {
            out.push(byte);
            return;
        }
        out.push(byte | 0x80);
    }
}

/// Append `v` as a signed LEB128, as §5.2.2 defines it.
///
/// The termination rule is the one the specification states and not the
/// obvious one: stop when the remaining value is all sign bits *and* the last
/// byte's own sign bit agrees. Getting that wrong produces an encoding every
/// validator rejects, which is why it has its own test.
pub fn sleb(out: &mut Vec<u8>, mut v: i64) {
    loop {
        let byte = (v & 0x7f) as u8;
        v >>= 7;
        let sign = byte & 0x40 != 0;
        if (v == 0 && !sign) || (v == -1 && sign) {
            out.push(byte);
            return;
        }
        out.push(byte | 0x80);
    }
}

/// How many bytes [`uleb`] would spend on `v`.
#[must_use]
pub fn uleb_len(v: u64) -> usize {
    let mut n = 1;
    let mut v = v >> 7;
    while v != 0 {
        n += 1;
        v >>= 7;
    }
    n
}

/// One function body under construction: the instruction stream, and the
/// locals it declares beyond its parameters.
///
/// A flat `Vec<u8>` with no fixups, which is the one place a wasm backend is
/// plainly easier than a native one: control flow is *structured*, so a
/// forward branch names a label depth rather than a byte displacement and
/// there is nothing to patch afterwards. See [`compile`](mod@super::compile) for
/// how a block's forward `brcond`s become that nesting.
#[derive(Debug, Default)]
pub struct Func {
    body: Vec<u8>,
    i64_locals: u32,
    i32_locals: u32,
}

impl Func {
    /// A body declaring `i64s` 64-bit locals and `i32s` 32-bit ones beyond its
    /// parameters.
    #[must_use]
    pub fn new(i64s: u32, i32s: u32) -> Func {
        Func {
            body: Vec::new(),
            i64_locals: i64s,
            i32_locals: i32s,
        }
    }

    /// The instruction stream so far.
    #[inline]
    #[must_use]
    pub fn code(&self) -> &[u8] {
        &self.body
    }

    /// A bare opcode.
    #[inline]
    pub fn op(&mut self, opcode: u8) {
        self.body.push(opcode);
    }

    /// An opcode with one unsigned immediate: `call`, `br`, `local.get`, …
    #[inline]
    pub fn op_u(&mut self, opcode: u8, imm: u32) {
        self.body.push(opcode);
        uleb(&mut self.body, u64::from(imm));
    }

    /// `i32.const`.
    #[inline]
    pub fn i32_const(&mut self, v: i32) {
        self.body.push(op::I32_CONST);
        sleb(&mut self.body, i64::from(v));
    }

    /// `i64.const`.
    #[inline]
    pub fn i64_const(&mut self, v: i64) {
        self.body.push(op::I64_CONST);
        sleb(&mut self.body, v);
    }

    /// `block` with the empty result type.
    #[inline]
    pub fn block(&mut self) {
        self.body.push(op::BLOCK);
        self.body.push(op::EMPTY);
    }

    /// `if` with the empty result type.
    #[inline]
    pub fn if_(&mut self) {
        self.body.push(op::IF);
        self.body.push(op::EMPTY);
    }

    /// `i64.load` / `i64.store` with a static offset.
    ///
    /// The alignment immediate is the *log2* of the assumed alignment
    /// (§5.4.4), and three is eight bytes — which every address this backend
    /// forms is, because the frame is a `u64` array.
    #[inline]
    pub fn mem64(&mut self, opcode: u8, offset: u32) {
        self.body.push(opcode);
        uleb(&mut self.body, 3);
        uleb(&mut self.body, u64::from(offset));
    }

    /// The body as a code-section entry: the locals declaration, the
    /// instructions, and the terminating `end`, prefixed by its own length.
    ///
    /// §5.5.13: a `code` entry is `size:u32` followed by a `func`, and a
    /// `func` is `vec(locals)` followed by an expression.
    #[must_use]
    pub fn finish(self) -> Vec<u8> {
        let mut inner = Vec::with_capacity(self.body.len() + 16);
        let groups = u64::from(u32::from(self.i64_locals > 0) + u32::from(self.i32_locals > 0));
        uleb(&mut inner, groups);
        if self.i64_locals > 0 {
            uleb(&mut inner, u64::from(self.i64_locals));
            inner.push(ty::I64);
        }
        if self.i32_locals > 0 {
            uleb(&mut inner, u64::from(self.i32_locals));
            inner.push(ty::I32);
        }
        inner.extend_from_slice(&self.body);
        inner.push(op::END);

        let mut out = Vec::with_capacity(inner.len() + uleb_len(inner.len() as u64));
        uleb(&mut out, inner.len() as u64);
        out.extend_from_slice(&inner);
        out
    }
}

/// A function type: parameters in, results out.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct FuncType {
    /// Parameter types, in order.
    pub params: Vec<u8>,
    /// Result types, in order.
    pub results: Vec<u8>,
}

impl FuncType {
    /// Encode as §5.3.4's `functype`.
    fn encode(&self, out: &mut Vec<u8>) {
        out.push(0x60);
        uleb(out, self.params.len() as u64);
        out.extend_from_slice(&self.params);
        uleb(out, self.results.len() as u64);
        out.extend_from_slice(&self.results);
    }
}

/// Assemble a module from the pieces [`compile`](mod@super::compile) produces.
///
/// The shape is fixed — types, imports, one function, one memory import, one
/// export, one body — so this is a function rather than a builder with a
/// dozen states nothing reaches.
///
/// * `types` are the function types, referenced by index.
/// * `imports` are `(module, name, type index)`, in function-index order, and
///   they occupy indices `0..imports.len()`.
/// * `memory` is `(module, name)` for the imported linear memory, which is the
///   embedder's own — the guest's RAM lives in it, so it must be
///   *imported* rather than defined here or generated code would be writing
///   into a memory nobody else can see (`ROADMAP.md` §11.2, and the
///   `SharedArrayBuffer` rule in CLAUDE.md's "Targets").
/// * `func_type` is the index of the exported function's type, and `body` its
///   finished [`Func`].
#[must_use]
pub fn module(
    types: &[FuncType],
    imports: &[(&str, &str, u32)],
    memory: (&str, &str),
    export: &str,
    func_type: u32,
    body: &[u8],
) -> Vec<u8> {
    let mut out = Vec::with_capacity(body.len() + 128);
    out.extend_from_slice(&HEADER);

    // §5.5.4 — the type section.
    let mut sec = Vec::new();
    uleb(&mut sec, types.len() as u64);
    for t in types {
        t.encode(&mut sec);
    }
    section(&mut out, 1, &sec);

    // §5.5.5 — the import section. The memory goes last so that the function
    // imports keep indices 0..n, which is what generated `call`s name.
    sec.clear();
    uleb(&mut sec, imports.len() as u64 + 1);
    for &(module, name, index) in imports {
        name_bytes(&mut sec, module);
        name_bytes(&mut sec, name);
        sec.push(0x00); // importdesc: func
        uleb(&mut sec, u64::from(index));
    }
    name_bytes(&mut sec, memory.0);
    name_bytes(&mut sec, memory.1);
    sec.push(0x02); // importdesc: mem
    sec.push(0x00); // limits: min only
    uleb(&mut sec, 1); // one 64 KiB page, which the embedder may exceed
    section(&mut out, 2, &sec);

    // §5.5.6 — the function section: one function, and its type.
    sec.clear();
    uleb(&mut sec, 1);
    uleb(&mut sec, u64::from(func_type));
    section(&mut out, 3, &sec);

    // §5.5.10 — the export section.
    sec.clear();
    uleb(&mut sec, 1);
    name_bytes(&mut sec, export);
    sec.push(0x00); // exportdesc: func
    uleb(&mut sec, imports.len() as u64);
    section(&mut out, 7, &sec);

    // §5.5.13 — the code section.
    sec.clear();
    uleb(&mut sec, 1);
    sec.extend_from_slice(body);
    section(&mut out, 10, &sec);

    out
}

/// A section: its id, its length, its contents (§5.5.2).
fn section(out: &mut Vec<u8>, id: u8, body: &[u8]) {
    out.push(id);
    uleb(out, body.len() as u64);
    out.extend_from_slice(body);
}

/// A `name`: a length-prefixed UTF-8 byte sequence (§5.2.4).
fn name_bytes(out: &mut Vec<u8>, s: &str) {
    uleb(out, s.len() as u64);
    out.extend_from_slice(s.as_bytes());
}

#[cfg(test)]
mod tests {
    use super::*;
    use alloc::vec;

    #[test]
    fn unsigned_leb128_matches_the_specifications_examples() {
        // Core specification §5.2.2, and the canonical worked examples every
        // encoder is checked against.
        let cases: &[(u64, &[u8])] = &[
            (0, &[0x00]),
            (1, &[0x01]),
            (127, &[0x7f]),
            (128, &[0x80, 0x01]),
            (624_485, &[0xe5, 0x8e, 0x26]),
            (
                u64::MAX,
                &[0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x01],
            ),
        ];
        for &(v, want) in cases {
            let mut out = Vec::new();
            uleb(&mut out, v);
            assert_eq!(out, want, "uleb({v})");
            assert_eq!(uleb_len(v), want.len(), "uleb_len({v})");
        }
    }

    #[test]
    fn signed_leb128_stops_on_the_sign_bit_and_not_on_the_value() {
        // The rule that is easy to get wrong: 64 needs two bytes unsigned-wise
        // but its low byte's bit 6 is set, so a one-byte encoding would read
        // back as -64.
        let cases: &[(i64, &[u8])] = &[
            (0, &[0x00]),
            (1, &[0x01]),
            (-1, &[0x7f]),
            (63, &[0x3f]),
            (64, &[0xc0, 0x00]),
            (-64, &[0x40]),
            (-65, &[0xbf, 0x7f]),
            (-123_456, &[0xc0, 0xbb, 0x78]),
        ];
        for &(v, want) in cases {
            let mut out = Vec::new();
            sleb(&mut out, v);
            assert_eq!(out, want, "sleb({v})");
        }
    }

    #[test]
    fn a_body_declares_its_locals_grouped_by_type() {
        // §5.5.13: `vec(locals)`, each `(count, valtype)`. Two groups when both
        // banks are used, one when only the i64 bank is, and — the case a
        // careless encoder gets wrong — a *zero-length* vector when neither is.
        let f = Func::new(3, 1);
        let out = f.finish();
        assert_eq!(&out[1..], &[0x02, 0x03, ty::I64, 0x01, ty::I32, op::END]);
        let f = Func::new(0, 0);
        let out = f.finish();
        assert_eq!(&out[1..], &[0x00, op::END]);
    }

    #[test]
    fn a_module_carries_its_header_and_its_sections_in_order() {
        let types = vec![FuncType {
            params: vec![ty::I32, ty::I32],
            results: vec![ty::I64],
        }];
        let mut f = Func::new(1, 0);
        f.i64_const(7);
        f.op(op::RETURN);
        let m = module(&types, &[], ("e", "m"), "b", 0, &f.finish());
        assert_eq!(&m[..8], &HEADER);
        // Section ids ascend, which the specification requires of the known
        // sections (§5.5.2): 1 type, 2 import, 3 function, 7 export, 10 code.
        let mut at = 8;
        let mut seen = Vec::new();
        while at < m.len() {
            seen.push(m[at]);
            at += 1;
            let mut len = 0u64;
            let mut shift = 0;
            loop {
                let b = m[at];
                at += 1;
                len |= u64::from(b & 0x7f) << shift;
                shift += 7;
                if b & 0x80 == 0 {
                    break;
                }
            }
            at += len as usize;
        }
        assert_eq!(seen, vec![1, 2, 3, 7, 10]);
        assert_eq!(at, m.len(), "a section length overran the module");
    }
}
