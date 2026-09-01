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

//! | `x86` | `cpu-x86` | Intel 8086/8088 real mode and 80386/80486 protected mode with paging, hardware-checked against `SingleStepTests/8088` |

//! | `riscv` | `cpu-riscv` | RV64GC/RV32 interpreter, privileged modes, Sv39, software IEEE-754 |

//! | `m68k` | `cpu-m68k` | MC68000 interpreter with a modelled prefetch queue, exceptions, disassembler |
//! | `mips` | `cpu-mips` | MIPS I / R3000A interpreter: branch and load delay slots, CP0, the 64-entry TLB, disassembler |
//!
//! Every core ships an interpreter first; the IR frontend comes later and is
//! differentially tested against it forever. **The interpreter is the oracle.**

#[cfg(feature = "cpu-m68k")]
#[cfg_attr(docsrs, doc(cfg(feature = "cpu-m68k")))]
pub mod m68k;

#[cfg(feature = "cpu-mips")]
#[cfg_attr(docsrs, doc(cfg(feature = "cpu-mips")))]
pub mod mips;

#[cfg(feature = "cpu-mos6502")]
#[cfg_attr(docsrs, doc(cfg(feature = "cpu-mos6502")))]
pub mod mos6502;

// The family module always compiles and links nothing; the cores underneath
// it are individually gated.
pub mod arm;

#[cfg(feature = "cpu-z80")]
#[cfg_attr(docsrs, doc(cfg(feature = "cpu-z80")))]
pub mod z80;

#[cfg(feature = "cpu-x86")]
#[cfg_attr(docsrs, doc(cfg(feature = "cpu-x86")))]
pub mod x86;

#[cfg(feature = "cpu-sm83")]
#[cfg_attr(docsrs, doc(cfg(feature = "cpu-sm83")))]
pub mod sm83;

#[cfg(feature = "cpu-riscv")]
#[cfg_attr(docsrs, doc(cfg(feature = "cpu-riscv")))]
pub mod riscv;
