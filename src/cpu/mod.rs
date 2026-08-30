//! CPU cores.
//!
//! One Cargo feature per core (`ROADMAP.md` §3), so a NES build links a 6502
//! and nothing else. Cores are `no_std + alloc` and reach guest memory only
//! through [`AddressSpace`](crate::core::space::AddressSpace) — cycle
//! accounting is therefore per bus access rather than a post-hoc table of
//! instruction lengths (`ROADMAP.md` §6, CLAUDE.md "CPU cores").
//!
//! | Core | Feature | State |
//! | --- | --- | --- |
//! | `mos6502` | `cpu-mos6502` | cycle-accurate interpreter, illegal opcodes, disassembler |
//!
//! Every core ships an interpreter first; the IR frontend comes later and is
//! differentially tested against it forever. **The interpreter is the oracle.**

#[cfg(feature = "cpu-mos6502")]
#[cfg_attr(docsrs, doc(cfg(feature = "cpu-mos6502")))]
pub mod mos6502;
