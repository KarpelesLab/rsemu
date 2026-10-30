//! Executing VEX-encoded instructions.
//!
//! Decoding is `isa`'s: the VEX prefix is recognised there, its constraints
//! (`VEX.L`, `VEX.vvvv`, `VEX.W`) are applied to the row there, and an
//! encoding that breaks one reaches this file as `Op::UD`. What remains here
//! is execution, which is not implemented yet: every VEX encoding raises
//! `#UD`, which is what a part without AVX does.

use super::exec::{Ex, Exec, Fault, VEC_UD};
use super::isa::Fields;

impl Exec<'_> {
    /// Execute one VEX-encoded instruction.
    pub(super) fn vex_instruction(&mut self, _f: &Fields) -> Ex<()> {
        Err(Fault::bare(VEC_UD))
    }
}
