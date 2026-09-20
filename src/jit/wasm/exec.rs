//! The **reference executor**: a WebAssembly interpreter over exactly the
//! subset [`compile`](mod@super::compile) emits.
//!
//! # What this is for, stated plainly
//!
//! It is **not a speed path and never will be**. Interpreting wasm that was
//! generated from IR is slower than interpreting the IR, and a build that
//! reaches here has chosen correctness evidence over throughput. Say what it
//! *is* for instead, because it is load-bearing for two things a JIT backend
//! usually cannot have at once:
//!
//! 1. **The backend is executable on every target.** On a native host there is
//!    no `WebAssembly.Module` to hand a module to, so without this the only
//!    claim available would be `jit::arm64`'s — *"it emits the instructions
//!    the manual says it does"*, with agreement against the oracle asserted by
//!    code that has never run. With it, `engine = "jit-wasm"` joins
//!    `tests/riscv_virt_engines.rs` on the x86-64 runner that gates every
//!    commit, and the backend's own IR differential executes rather than
//!    inspects.
//! 2. **It is the portable half of the embedder seam.** `ROADMAP.md` §11.5's
//!    `rsemu.compile` is an *embedder* import: a browser has one and plain
//!    WASI does not (`docs/techniques/wasm-jit.md` says what WASI would need).
//!    The seam is [`abi`](super::abi) — a module's imports and its signature,
//!    which are data rather than code — and this is the implementation every
//!    build has, so a target with no embedder degrades in speed rather than
//!    failing to run: §9's rule, one level further down than it was written
//!    for.
//!
//! # What it is not
//!
//! It is not a general wasm engine and must never be sold as one. It
//! implements the opcodes [`emit::op`](super::emit::op) names and rejects
//! every other byte; it does not validate — the producer is one file away and
//! the modules are checked against a real engine separately
//! (`docs/techniques/wasm-jit.md`, "Validating the encoding"); and it has no
//! `f32`, no `f64`, no table, no `loop`, no multi-value and no bulk memory,
//! because the code generator emits none of them.
//!
//! # The specification
//!
//! Semantics are the *WebAssembly Core Specification* §4 (Execution): §4.4.1
//! for the numeric instructions — including the two rules this file would
//! otherwise get wrong, that a shift count is taken modulo the operand width
//! and that a division by zero traps — §4.4.5 for the memory instructions and
//! §4.4.8 for the structured control instructions and their label semantics.
//! Binary decoding is §5, as in [`emit`](super::emit).

use alloc::vec::Vec;

use super::emit::op;

/// Why the executor stopped without producing a value.
///
/// A malformed module is a bug in [`compile`](mod@super::compile) and a trap is a
/// bug in what it emitted; neither is a guest-visible condition, so both are
/// reported rather than swallowed.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[non_exhaustive]
pub enum ExecError {
    /// The module does not decode, or does not have the shape this backend
    /// emits.
    Malformed(&'static str),
    /// The execution trapped: an out-of-bounds access, a division by zero, an
    /// opcode outside the subset.
    Trap(&'static str),
    /// The body did not finish within the step budget.
    Exhausted,
}

impl core::fmt::Display for ExecError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            ExecError::Malformed(w) => write!(f, "malformed generated module: {w}"),
            ExecError::Trap(w) => write!(f, "generated code trapped: {w}"),
            ExecError::Exhausted => f.write_str("generated code did not terminate"),
        }
    }
}

/// What a generated module's imports resolve to.
///
/// One method for all four, because the executor has no use for their
/// distinctness: `func` is the import index ([`abi::func`](super::abi::func))
/// and `args` are its arguments in order. The implementation is
/// [`rt::Engine`](super::rt)'s, and it is where the IR's semantics live —
/// this file only moves numbers.
///
/// `mem` is the linear memory, handed in rather than held, because a load
/// import leaves its result in the frame and the frame is in that memory.
pub trait Env {
    /// Call import `func`.
    fn call(&mut self, func: u32, args: &[i64], mem: &mut [u8]) -> i64;
}

/// A decoded module, ready to run.
#[derive(Debug, Clone)]
pub struct Program {
    /// Locals beyond the parameters, in declaration order: `(count, is_i64)`.
    locals: Vec<(u32, bool)>,
    /// How many parameters the exported function takes.
    params: usize,
    /// How many arguments each imported function takes, by index.
    import_arity: Vec<usize>,
    /// The instruction stream, without the locals declaration.
    body: Vec<u8>,
    /// For each `block`/`if`, the offset of its matching `end`.
    ends: Vec<(u32, u32)>,
}

impl Program {
    /// How many locals the body declares beyond its parameters.
    #[must_use]
    pub fn local_count(&self) -> usize {
        self.locals.iter().map(|&(n, _)| n as usize).sum()
    }

    /// The matching `end` of the structured instruction at `pc`.
    fn end_of(&self, pc: u32) -> Option<u32> {
        self.ends
            .binary_search_by_key(&pc, |&(s, _)| s)
            .ok()
            .map(|i| self.ends[i].1)
    }
}

// ---------------------------------------------------------------------------
// Decoding
// ---------------------------------------------------------------------------

struct Reader<'a> {
    bytes: &'a [u8],
    at: usize,
}

impl<'a> Reader<'a> {
    fn new(bytes: &'a [u8]) -> Reader<'a> {
        Reader { bytes, at: 0 }
    }

    fn byte(&mut self) -> Result<u8, ExecError> {
        let b = *self
            .bytes
            .get(self.at)
            .ok_or(ExecError::Malformed("ran off the end"))?;
        self.at += 1;
        Ok(b)
    }

    fn uleb(&mut self) -> Result<u64, ExecError> {
        let mut out = 0u64;
        let mut shift = 0u32;
        loop {
            let b = self.byte()?;
            if shift >= 64 {
                return Err(ExecError::Malformed("an unsigned LEB128 is too long"));
            }
            out |= u64::from(b & 0x7f) << shift;
            shift += 7;
            if b & 0x80 == 0 {
                return Ok(out);
            }
        }
    }

    fn sleb(&mut self) -> Result<i64, ExecError> {
        let mut out = 0i64;
        let mut shift = 0u32;
        loop {
            let b = self.byte()?;
            if shift >= 64 {
                return Err(ExecError::Malformed("a signed LEB128 is too long"));
            }
            out |= i64::from(b & 0x7f) << shift;
            shift += 7;
            if b & 0x80 == 0 {
                if shift < 64 && b & 0x40 != 0 {
                    out |= -1i64 << shift;
                }
                return Ok(out);
            }
        }
    }

    fn take(&mut self, n: usize) -> Result<&'a [u8], ExecError> {
        let end = self
            .at
            .checked_add(n)
            .filter(|&e| e <= self.bytes.len())
            .ok_or(ExecError::Malformed("a length ran off the end"))?;
        let out = &self.bytes[self.at..end];
        self.at = end;
        Ok(out)
    }
}

/// Decode a module [`compile`](mod@super::compile) produced.
///
/// # Errors
///
/// [`ExecError::Malformed`] when the bytes are not a module of the shape this
/// backend emits: the header, a type section, an import section whose last
/// entry is the memory, one function, one export and one body.
pub fn parse(bytes: &[u8]) -> Result<Program, ExecError> {
    let mut r = Reader::new(bytes);
    if r.take(8)? != super::emit::HEADER {
        return Err(ExecError::Malformed("not a wasm module"));
    }

    let mut types: Vec<(usize, usize)> = Vec::new();
    let mut import_arity: Vec<usize> = Vec::new();
    let mut func_type: Option<usize> = None;
    let mut code: Option<Vec<u8>> = None;

    while r.at < r.bytes.len() {
        let id = r.byte()?;
        let len = r.uleb()? as usize;
        let body = r.take(len)?;
        let mut s = Reader::new(body);
        match id {
            1 => {
                let n = s.uleb()?;
                for _ in 0..n {
                    if s.byte()? != 0x60 {
                        return Err(ExecError::Malformed("a type that is not a functype"));
                    }
                    let np = s.uleb()? as usize;
                    let _ = s.take(np)?;
                    let nr = s.uleb()? as usize;
                    let _ = s.take(nr)?;
                    types.push((np, nr));
                }
            }
            2 => {
                let n = s.uleb()?;
                for _ in 0..n {
                    let ml = s.uleb()? as usize;
                    let _ = s.take(ml)?;
                    let nl = s.uleb()? as usize;
                    let _ = s.take(nl)?;
                    match s.byte()? {
                        0x00 => {
                            let t = s.uleb()? as usize;
                            let arity = types
                                .get(t)
                                .ok_or(ExecError::Malformed("an import names no type"))?
                                .0;
                            import_arity.push(arity);
                        }
                        0x02 => {
                            // The memory. Its limits, and then nothing else may
                            // follow: generated code calls imports by index and
                            // a function import after the memory would shift
                            // every one of them.
                            let flags = s.byte()?;
                            let _ = s.uleb()?;
                            if flags & 0x01 != 0 {
                                let _ = s.uleb()?;
                            }
                        }
                        _ => {
                            return Err(ExecError::Malformed("an import this backend never emits"));
                        }
                    }
                }
            }
            3 => {
                let n = s.uleb()?;
                if n != 1 {
                    return Err(ExecError::Malformed("a module with more than one function"));
                }
                func_type = Some(s.uleb()? as usize);
            }
            7 | 0 => {}
            10 => {
                let n = s.uleb()?;
                if n != 1 {
                    return Err(ExecError::Malformed(
                        "a code section with more than one body",
                    ));
                }
                let size = s.uleb()? as usize;
                code = Some(s.take(size)?.to_vec());
            }
            _ => return Err(ExecError::Malformed("a section this backend never emits")),
        }
    }

    let params = types
        .get(func_type.ok_or(ExecError::Malformed("no function section"))?)
        .ok_or(ExecError::Malformed("the function names no type"))?
        .0;
    let code = code.ok_or(ExecError::Malformed("no code section"))?;

    let mut s = Reader::new(&code);
    let groups = s.uleb()?;
    let mut locals = Vec::new();
    for _ in 0..groups {
        let n = s.uleb()? as u32;
        let t = s.byte()?;
        let is64 = match t {
            super::emit::ty::I64 => true,
            super::emit::ty::I32 => false,
            _ => {
                return Err(ExecError::Malformed(
                    "a local type this backend never emits",
                ));
            }
        };
        locals.push((n, is64));
    }
    let body = code[s.at..].to_vec();
    let ends = scan(&body)?;

    Ok(Program {
        locals,
        params,
        import_arity,
        body,
        ends,
    })
}

/// Pair every `block`/`if` with its matching `end`.
///
/// The one thing an interpreter of structured control flow cannot do lazily:
/// a `br` must land past the `end` of a label that has not been reached yet.
/// `compile` emits no `loop`, so every label is forward and this is a single
/// pass with a stack.
fn scan(body: &[u8]) -> Result<Vec<(u32, u32)>, ExecError> {
    let mut r = Reader::new(body);
    let mut open: Vec<usize> = Vec::new();
    let mut out: Vec<(u32, u32)> = Vec::new();
    while r.at < body.len() {
        let here = r.at;
        let code = r.byte()?;
        match code {
            op::BLOCK | op::IF => {
                let _ = r.byte()?;
                open.push(here);
            }
            op::END => {
                if let Some(start) = open.pop() {
                    out.push((start as u32, here as u32));
                }
            }
            op::BR | op::BR_IF | op::CALL | op::LOCAL_GET | op::LOCAL_SET | op::LOCAL_TEE => {
                let _ = r.uleb()?;
            }
            op::I64_LOAD | op::I64_STORE => {
                let _ = r.uleb()?;
                let _ = r.uleb()?;
            }
            op::I32_CONST | op::I64_CONST => {
                let _ = r.sleb()?;
            }
            _ => {}
        }
    }
    if !open.is_empty() {
        return Err(ExecError::Malformed("a structured block was never closed"));
    }
    out.sort_unstable();
    Ok(out)
}

// ---------------------------------------------------------------------------
// Execution
// ---------------------------------------------------------------------------

/// A budget on executed instructions, so a malformed body fails a fuzz case
/// rather than hanging it. The same reason `Interp::STEP_LIMIT` exists, and
/// the same generosity: a real block is a few hundred wasm instructions.
const STEP_LIMIT: u64 = 1 << 24;

/// An open label: where its `end` is, and the operand-stack height to restore.
#[derive(Debug, Clone, Copy)]
struct Label {
    end: u32,
    height: usize,
}

/// Run `p` with `args`, over `mem`, resolving imports through `env`.
///
/// Values are carried as `i64` throughout; an `i32` occupies the low 32 bits,
/// zero-extended, which is what makes `i64.extend_i32_u` free and is sound
/// because the generated body is well-typed by construction.
///
/// # Errors
///
/// [`ExecError`] — see its variants. None of them is a guest-visible
/// condition: they mean the code generator or this file is wrong.
#[allow(clippy::too_many_lines)]
pub fn run(p: &Program, args: &[i64], mem: &mut [u8], env: &mut dyn Env) -> Result<i64, ExecError> {
    if args.len() != p.params {
        return Err(ExecError::Malformed("the wrong number of arguments"));
    }
    let mut locals: Vec<i64> = Vec::with_capacity(p.params + p.local_count());
    locals.extend_from_slice(args);
    locals.resize(p.params + p.local_count(), 0);

    let mut stack: Vec<i64> = Vec::with_capacity(16);
    let mut ctrl: Vec<Label> = Vec::with_capacity(8);
    let mut call_args: Vec<i64> = Vec::with_capacity(8);
    let body = &p.body;
    let mut r = Reader::new(body);
    let mut steps = 0u64;

    macro_rules! pop {
        () => {
            stack.pop().ok_or(ExecError::Malformed("an empty stack"))?
        };
    }
    macro_rules! local {
        ($i:expr) => {
            *locals
                .get($i as usize)
                .ok_or(ExecError::Malformed("a local out of range"))?
        };
    }

    loop {
        steps += 1;
        if steps > STEP_LIMIT {
            return Err(ExecError::Exhausted);
        }
        let here = r.at as u32;
        let code = r.byte()?;
        match code {
            op::BLOCK => {
                let _ = r.byte()?;
                let end = p
                    .end_of(here)
                    .ok_or(ExecError::Malformed("a block with no end"))?;
                ctrl.push(Label {
                    end,
                    height: stack.len(),
                });
            }
            op::IF => {
                let _ = r.byte()?;
                let end = p
                    .end_of(here)
                    .ok_or(ExecError::Malformed("an if with no end"))?;
                if pop!() as u32 != 0 {
                    ctrl.push(Label {
                        end,
                        height: stack.len(),
                    });
                } else {
                    r.at = end as usize + 1;
                }
            }
            op::END => match ctrl.pop() {
                Some(_) => {}
                // The function's own `end`: whatever is on the stack is the
                // result. Reached only when the body fell through, which
                // `compile` closes with a status constant.
                None => return Ok(pop!()),
            },
            op::BR | op::BR_IF => {
                let depth = r.uleb()? as usize;
                let go = code == op::BR || pop!() as u32 != 0;
                if go {
                    let at = ctrl
                        .len()
                        .checked_sub(depth + 1)
                        .ok_or(ExecError::Malformed("a branch past every label"))?;
                    let label = ctrl[at];
                    ctrl.truncate(at);
                    stack.truncate(label.height);
                    r.at = label.end as usize + 1;
                }
            }
            op::RETURN => return Ok(pop!()),
            op::CALL => {
                let index = r.uleb()? as u32;
                let arity = *p
                    .import_arity
                    .get(index as usize)
                    .ok_or(ExecError::Malformed("a call to no import"))?;
                if stack.len() < arity {
                    return Err(ExecError::Malformed("an empty stack"));
                }
                call_args.clear();
                call_args.extend_from_slice(&stack[stack.len() - arity..]);
                stack.truncate(stack.len() - arity);
                stack.push(env.call(index, &call_args, mem));
            }
            op::DROP => {
                let _ = pop!();
            }
            op::SELECT => {
                let cond = pop!() as u32;
                let b = pop!();
                let a = pop!();
                stack.push(if cond != 0 { a } else { b });
            }
            op::LOCAL_GET => {
                let i = r.uleb()?;
                stack.push(local!(i));
            }
            op::LOCAL_SET | op::LOCAL_TEE => {
                let i = r.uleb()? as usize;
                let v = pop!();
                *locals
                    .get_mut(i)
                    .ok_or(ExecError::Malformed("a local out of range"))? = v;
                if code == op::LOCAL_TEE {
                    stack.push(v);
                }
            }
            op::I64_LOAD | op::I64_STORE => {
                let _align = r.uleb()?;
                let offset = r.uleb()?;
                if code == op::I64_LOAD {
                    let base = pop!() as u32;
                    let at = address(base, offset, 8, mem.len())?;
                    let mut buf = [0u8; 8];
                    buf.copy_from_slice(&mem[at..at + 8]);
                    stack.push(i64::from_le_bytes(buf));
                } else {
                    let v = pop!();
                    let base = pop!() as u32;
                    let at = address(base, offset, 8, mem.len())?;
                    mem[at..at + 8].copy_from_slice(&v.to_le_bytes());
                }
            }
            op::I32_CONST => {
                let v = r.sleb()?;
                stack.push(i64::from(v as u32));
            }
            op::I64_CONST => {
                let v = r.sleb()?;
                stack.push(v);
            }
            op::I32_EQZ => {
                let a = pop!() as u32;
                stack.push(i64::from(a == 0));
            }
            op::I64_EQZ => {
                let a = pop!();
                stack.push(i64::from(a == 0));
            }
            op::I64_EQ
            | op::I64_NE
            | op::I64_LT_S
            | op::I64_GT_S
            | op::I64_LE_S
            | op::I64_GE_S
            | op::I64_LT_U
            | op::I64_GT_U
            | op::I64_LE_U
            | op::I64_GE_U => {
                let b = pop!();
                let a = pop!();
                let (ua, ub) = (a as u64, b as u64);
                let yes = match code {
                    op::I64_EQ => a == b,
                    op::I64_NE => a != b,
                    op::I64_LT_S => a < b,
                    op::I64_GT_S => a > b,
                    op::I64_LE_S => a <= b,
                    op::I64_GE_S => a >= b,
                    op::I64_LT_U => ua < ub,
                    op::I64_GT_U => ua > ub,
                    op::I64_LE_U => ua <= ub,
                    _ => ua >= ub,
                };
                stack.push(i64::from(yes));
            }
            op::I64_CLZ => {
                let a = pop!();
                stack.push(i64::from(a.leading_zeros()));
            }
            op::I64_CTZ => {
                let a = pop!();
                stack.push(i64::from(a.trailing_zeros()));
            }
            op::I64_POPCNT => {
                let a = pop!();
                stack.push(i64::from(a.count_ones()));
            }
            op::I64_ADD | op::I64_SUB | op::I64_MUL | op::I64_AND | op::I64_OR | op::I64_XOR => {
                let b = pop!();
                let a = pop!();
                stack.push(match code {
                    op::I64_ADD => a.wrapping_add(b),
                    op::I64_SUB => a.wrapping_sub(b),
                    op::I64_MUL => a.wrapping_mul(b),
                    op::I64_AND => a & b,
                    op::I64_OR => a | b,
                    _ => a ^ b,
                });
            }
            op::I64_REM_U => {
                let b = pop!() as u64;
                let a = pop!() as u64;
                if b == 0 {
                    // §4.4.1: an integer division by zero traps. The only
                    // `rem_u` `compile` emits is by a type width, so this is
                    // unreachable through the supported path — and a trap
                    // reported is better than a panic in a fuzz case.
                    return Err(ExecError::Trap("remainder by zero"));
                }
                stack.push((a % b) as i64);
            }
            op::I64_SHL | op::I64_SHR_S | op::I64_SHR_U | op::I64_ROTL | op::I64_ROTR => {
                let b = pop!();
                let a = pop!();
                // §4.4.1: the shift count is taken modulo the operand width.
                let n = (b as u64 & 63) as u32;
                stack.push(match code {
                    op::I64_SHL => a.wrapping_shl(n),
                    op::I64_SHR_S => a.wrapping_shr(n),
                    op::I64_SHR_U => ((a as u64).wrapping_shr(n)) as i64,
                    op::I64_ROTL => a.rotate_left(n),
                    _ => a.rotate_right(n),
                });
            }
            op::I64_EXTEND_I32_U => {
                let a = pop!();
                stack.push(i64::from(a as u32));
            }
            _ => return Err(ExecError::Trap("an opcode outside the emitted subset")),
        }
    }
}

/// Resolve a memory address, or say it left the memory.
fn address(base: u32, offset: u64, width: usize, len: usize) -> Result<usize, ExecError> {
    let at = u64::from(base)
        .checked_add(offset)
        .ok_or(ExecError::Trap("an address that overflowed"))?;
    let end = at
        .checked_add(width as u64)
        .ok_or(ExecError::Trap("an access that overflowed"))?;
    if end > len as u64 {
        return Err(ExecError::Trap("an access outside linear memory"));
    }
    Ok(at as usize)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::jit::wasm::emit::{Func, FuncType, module, ty};
    use alloc::vec;

    struct NoImports;
    impl Env for NoImports {
        fn call(&mut self, _func: u32, _args: &[i64], _mem: &mut [u8]) -> i64 {
            unreachable!("this module imports no function")
        }
    }

    fn one(f: Func) -> Program {
        let types = vec![FuncType {
            params: vec![ty::I32, ty::I32],
            results: vec![ty::I64],
        }];
        let bytes = module(&types, &[], ("e", "m"), "b", 0, &f.finish());
        parse(&bytes).expect("the encoder produced a module this decodes")
    }

    #[test]
    fn a_body_that_falls_through_returns_what_it_left_on_the_stack() {
        let mut f = Func::new(0, 0);
        f.i64_const(-7);
        let p = one(f);
        let mut mem = vec![0u8; 64];
        assert_eq!(run(&p, &[0, 0], &mut mem, &mut NoImports), Ok(-7));
    }

    #[test]
    fn a_branch_leaves_its_block_and_skips_what_follows() {
        // block block (i64.const 1) (local.set 2) br 0 (i64.const 2)
        // (local.set 2) end (i64.const 3) (local.set 2) end (local.get 2)
        //
        // A `br 0` leaves the innermost block, so the middle store runs and
        // the inner one does not — the shape every `brcond` compiles to.
        let mut f = Func::new(1, 0);
        f.block();
        f.block();
        f.i64_const(1);
        f.op_u(op::LOCAL_SET, 2);
        f.op_u(op::BR, 0);
        f.i64_const(2);
        f.op_u(op::LOCAL_SET, 2);
        f.op(op::END);
        f.i64_const(3);
        f.op_u(op::LOCAL_SET, 2);
        f.op(op::END);
        f.op_u(op::LOCAL_GET, 2);
        let p = one(f);
        let mut mem = vec![0u8; 64];
        assert_eq!(run(&p, &[0, 0], &mut mem, &mut NoImports), Ok(3));
    }

    #[test]
    fn an_if_whose_condition_is_false_skips_to_its_end() {
        let mut f = Func::new(0, 0);
        f.i32_const(0);
        f.if_();
        f.i64_const(1);
        f.op(op::RETURN);
        f.op(op::END);
        f.i64_const(2);
        let p = one(f);
        let mut mem = vec![0u8; 64];
        assert_eq!(run(&p, &[0, 0], &mut mem, &mut NoImports), Ok(2));
        // And a true one takes the branch.
        let mut f = Func::new(0, 0);
        f.i32_const(1);
        f.if_();
        f.i64_const(1);
        f.op(op::RETURN);
        f.op(op::END);
        f.i64_const(2);
        let p = one(f);
        assert_eq!(run(&p, &[0, 0], &mut mem, &mut NoImports), Ok(1));
    }

    #[test]
    fn memory_is_reached_by_byte_offset_and_bounded() {
        let mut f = Func::new(0, 0);
        f.op_u(op::LOCAL_GET, 1);
        f.i64_const(0x0102_0304_0506_0708);
        f.mem64(op::I64_STORE, 8);
        f.op_u(op::LOCAL_GET, 1);
        f.mem64(op::I64_LOAD, 8);
        let p = one(f);
        let mut mem = vec![0u8; 64];
        assert_eq!(
            run(&p, &[0, 16], &mut mem, &mut NoImports),
            Ok(0x0102_0304_0506_0708)
        );
        // Little-endian, as §4.4.5 requires however the host is ordered.
        assert_eq!(mem[24], 0x08);
        assert_eq!(mem[31], 0x01);
        // And an access past the end is a trap, not a panic.
        assert!(matches!(
            run(&p, &[0, 60], &mut mem, &mut NoImports),
            Err(ExecError::Trap(_))
        ));
    }

    #[test]
    fn a_shift_count_is_taken_modulo_the_operand_width() {
        // §4.4.1. The rule `compile`'s shift lowering exists to work around,
        // so it had better be the rule this executor implements.
        let mut f = Func::new(0, 0);
        f.i64_const(1);
        f.i64_const(64);
        f.op(op::I64_SHL);
        let p = one(f);
        let mut mem = vec![0u8; 64];
        assert_eq!(run(&p, &[0, 0], &mut mem, &mut NoImports), Ok(1));
    }

    #[test]
    fn an_opcode_outside_the_subset_is_a_trap_rather_than_a_panic() {
        let mut f = Func::new(0, 0);
        f.op(0xa0); // f32.min — never emitted, and never will be
        let p = one(f);
        let mut mem = vec![0u8; 64];
        assert!(matches!(
            run(&p, &[0, 0], &mut mem, &mut NoImports),
            Err(ExecError::Trap(_))
        ));
    }
}
