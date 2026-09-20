//! The contract a generated module is compiled against: its imports, its
//! signature, its frame, and the status codes it returns.
//!
//! This is [`jit::arm64::abi`](crate::jit::arm64::abi) for a host that has no
//! registers and no pointers — and it carries **no `unsafe`**, which is the
//! whole difference. A native backend's boundary is a `Ctx` pointer generated
//! code dereferences; here the boundary is a *wasm import*, and an argument
//! crossing it is a number the embedder checks.
//!
//! # The signature
//!
//! ```text
//! (func (param $ctx i32) (param $frame i32) (result i64))
//! ```
//!
//! `$ctx` is opaque to generated code: it is handed back unexamined to every
//! import, and what it means is the embedder's business — a `Ctx` address in a
//! browser build, a zero in [`exec`](super::exec), which resolves it against
//! the engine it is already standing in. `$frame` is a **byte offset into the
//! imported linear memory**, which is what keeps guest RAM addressable by
//! offset rather than by `&mut [u8]` (CLAUDE.md, "Targets") and keeps the
//! whole thing legal in a `SharedArrayBuffer`.
//!
//! The result is a [`status`] code, and the five values are deliberately the
//! *same numbers* `jit::x86::rt::status` and `jit::arm64::abi::status` use.
//! A dispatcher that grew a second mapping would be a second thing to get
//! wrong.
//!
//! # The frame
//!
//! ```text
//! frame + 0            the out word: a successor PC, or a load's result
//! frame + 8 + 8*n      temporary n
//! ```
//!
//! One `u64` per [`Temp`](crate::ir::Temp), and the *out word* doubles as the
//! load result because the two are never live at once — a load happens inside
//! a block and the out word is written by its terminator.
//!
//! Temporaries live in wasm **locals**, not here: an engine allocates its own
//! registers, so [`ir::linear_scan`](crate::ir::linear_scan) is the one shared
//! piece of the native backends this one does not use. The frame carries only
//! the temporaries an [`InsnStart`](crate::ir::InsnStart) names, written
//! through at their definition, which is the same write-through both native
//! backends use to make a fault precise — and here it is the *only* way the
//! Rust side can see a temporary at all.

/// The status a generated block returns.
///
/// [`status::EXIT`] through [`status::SPENT`] are the native backends' own
/// numbers. [`status::ERROR`] is new, and it is here because a wasm function
/// cannot return a `Result`: the two conditions [`Interp`](crate::ir::Interp)
/// reports as `Err` — a malformed block, and a
/// [`Retry`](crate::core::error::BusError::Retry) a committed instruction may
/// not take — have to cross as a number and be rebuilt on the other side.
pub mod status {
    /// `exit_tb`: back to the dispatcher.
    pub const EXIT: i64 = 0;
    /// `goto_tb`: on to the statically known successor in the frame's out word.
    pub const GOTO: i64 = 1;
    /// `lookup_and_goto`: the successor is the computed PC in the out word.
    pub const LOOKUP: i64 = 2;
    /// A guest access faulted; the engine holds where and why.
    pub const FAULT: i64 = 3;
    /// The tick allowance was spent at a boundary.
    pub const SPENT: i64 = 4;
    /// The run has to be reported as an `Err`; the engine holds which.
    pub const ERROR: i64 = 5;
}

/// What an import is being asked to do, where one import serves several.
pub mod note {
    /// [`IrHost::charge`](crate::ir::IrHost::charge), with the tick count.
    pub const CHARGE: i32 = 0;
    /// [`IrHost::insn_start`](crate::ir::IrHost::insn_start), by mark index,
    /// at a boundary the tick allowance may end the block at.
    pub const BOUNDARY: i32 = 1;
    /// The same, at an **exit** boundary — one a terminator follows, which is
    /// never asked at, because [`InsnStart::pc`](crate::ir::InsnStart::pc) is
    /// then a static placeholder and the real successor is in the slot the
    /// map publishes. `ir::interp`'s `INSN_START` arm has the argument.
    pub const BOUNDARY_EXIT: i32 = 2;
}

/// What an import answers: nothing to do, or a [`status`] to return.
///
/// Zero is "carry on" so that generated code tests one value and branches
/// once. Anything else is the status the block returns *as it stands*, which
/// is why the codes are shared rather than translated: an import that answers
/// `3` has already recorded the fault, and generated code has only to hand the
/// number back.
pub mod answer {
    /// Carry on.
    pub const OK: i32 = 0;
}

/// The imported functions, in the order they take function indices.
pub mod func {
    /// `(param i32 ctx, i32 slot) -> i64` — [`Opcode::GET_SLOT`](crate::ir::Opcode::GET_SLOT).
    pub const SLOT: u32 = 0;
    /// `(param i32 ctx, i32 memop, i64 addr, i32 at) -> i32` — a guest load,
    /// leaving its value in the frame's out word.
    pub const LOAD: u32 = 1;
    /// `(param i32 ctx, i32 memop, i64 addr, i64 value, i32 at) -> i32` — a
    /// guest store.
    pub const STORE: u32 = 2;
    /// `(param i32 ctx, i32 kind, i64 arg) -> i32` — a [`note`](super::note).
    pub const NOTE: u32 = 3;
    /// How many there are.
    pub const COUNT: u32 = 4;
}

/// The import module name, and the four names within it.
///
/// One letter, because a module is emitted per block and a name is paid for in
/// every one of them. `ROADMAP.md` §11.5's convention — an embedder-supplied
/// import object, no bundled JS runtime — is what these hang off.
pub mod name {
    /// The import module every one of them lives in.
    pub const MODULE: &str = "e";
    /// The imported linear memory: the embedder's own, holding the frame.
    pub const MEMORY: &str = "m";
    /// [`func::SLOT`](super::func::SLOT).
    pub const SLOT: &str = "g";
    /// [`func::LOAD`](super::func::LOAD).
    pub const LOAD: &str = "l";
    /// [`func::STORE`](super::func::STORE).
    pub const STORE: &str = "s";
    /// [`func::NOTE`](super::func::NOTE).
    pub const NOTE: &str = "n";
    /// The one exported function: the block.
    pub const BLOCK: &str = "b";
}

/// The local index of the `$ctx` parameter.
pub const LOCAL_CTX: u32 = 0;
/// The local index of the `$frame` parameter.
pub const LOCAL_FRAME: u32 = 1;
/// The first local a temporary gets.
pub const LOCAL_TEMPS: u32 = 2;

/// Where temporary `n` lives in the frame.
#[inline]
#[must_use]
pub const fn temp_offset(n: u32) -> u32 {
    8 + 8 * n
}

/// How many bytes a frame for `temps` temporaries needs.
#[inline]
#[must_use]
pub const fn frame_bytes(temps: usize) -> usize {
    8 + 8 * temps
}
