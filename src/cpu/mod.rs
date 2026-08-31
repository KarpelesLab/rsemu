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
//! | `z80` | `cpu-z80` | cycle-accurate interpreter, every prefix page, MEMPTR, separate I/O space |

//! | `x86` | `cpu-x86` | Intel 8086/8088, real mode, hardware-checked against `SingleStepTests/8088` |

//! | `riscv` | `cpu-riscv` | RV64GC/RV32 interpreter, privileged modes, Sv39, software IEEE-754 |

//! | `m68k` | `cpu-m68k` | MC68000 interpreter with a modelled prefetch queue, exceptions, disassembler |
//!
//! Every core ships an interpreter first; the IR frontend comes later and is
//! differentially tested against it forever. **The interpreter is the oracle.**

#[cfg(feature = "cpu-m68k")]
#[cfg_attr(docsrs, doc(cfg(feature = "cpu-m68k")))]
pub mod m68k;

#[cfg(feature = "cpu-mos6502")]
#[cfg_attr(docsrs, doc(cfg(feature = "cpu-mos6502")))]
pub mod mos6502;

#[cfg(feature = "cpu-arm")]
#[cfg_attr(docsrs, doc(cfg(feature = "cpu-arm")))]
pub mod arm;

#[cfg(feature = "cpu-z80")]
#[cfg_attr(docsrs, doc(cfg(feature = "cpu-z80")))]
pub mod z80;

#[cfg(feature = "cpu-x86")]
#[cfg_attr(docsrs, doc(cfg(feature = "cpu-x86")))]
pub mod x86;

#[cfg(feature = "cpu-riscv")]
#[cfg_attr(docsrs, doc(cfg(feature = "cpu-riscv")))]
pub mod riscv;
