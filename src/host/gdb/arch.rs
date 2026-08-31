//! Per-CPU register maps, and the target descriptions generated from them.
//!
//! `qXfer:features:read` is how a debugger learns a register file it was not
//! compiled knowing: the stub hands it an XML description, and the client builds
//! `info registers`, `$pc` and the `g` packet layout from that. Generating the
//! XML from the same table the `g` packet is built from means the two cannot
//! disagree, which is the classic gdbstub bug.
//!
//! # A description is not a port
//!
//! It is worth being exact about what this buys, because the usual claim —
//! "target descriptions let GDB debug any architecture" — is half true. A
//! description gives GDB the *registers*. GDB still needs a **gdbarch** for the
//! machine itself: its disassembler, its frame unwinder, its breakpoint
//! encoding. Where upstream has one ([`Arch::architecture`] names it), a new
//! core or a new register layout genuinely costs a debugger nothing. Where it
//! does not — the MOS 6502, the 8086 in real mode — stock GDB prints
//! *"Architecture rejected target-supplied description"* and `target remote`
//! fails, and no gdbstub can change that from this side. The description is
//! still correct and still complete, and a client that trusts it (rsemu's own,
//! in `tests/gdb_session.rs`; a GDB built with a port for that CPU) drives the
//! whole session.
//!
//! # Where the register values come from
//!
//! There is **no route from a `dyn Device` to a concrete CPU type**:
//! [`Device`](crate::core::device::Device) deliberately keeps `Any` out of its
//! supertrait chain, and the core exposes no register accessor. What it does
//! expose is the surface `ROADMAP.md` §4.5 already promises — a device's
//! snapshot chunk, which by invariant 3 *is* its architectural state, caches
//! excluded. So a register here is a byte offset into that chunk, and reading
//! the register file is [`Device::save`](crate::core::device::Device::save) into a scratch chunk; writing it is
//! [`Device::load`](crate::core::device::Device::load) of a patched one.
//!
//! That is a seam, and it is marked as one. The layout of a chunk is private to
//! the core that wrote it, so each entry records the class state **version** its
//! offsets were verified against and refuses to decode a different one
//! ([`Arch::check`]) rather than hand GDB plausible nonsense. When `core::device`
//! grows a register view, every table below collapses into a call to it and
//! nothing else in this module changes.
//!
//! # Sources
//!
//! GDB manual, "Target Descriptions" appendix, for the XML shape; and each
//! core's own `save` for the offsets.

use core::fmt::Write as _;

use crate::core::device::DeviceClass;

/// How GDB should present a register.
///
/// The predefined type names from the target-description DTD. `int` with an
/// explicit `bitsize` covers everything rsemu has; the two pointer types exist
/// so `$pc` and `$sp` print as addresses.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RegType {
    /// A plain integer.
    Int,
    /// An address in the instruction space.
    CodePtr,
    /// An address in the data space.
    DataPtr,
}

impl RegType {
    /// The name this type has in a target description.
    #[must_use]
    pub const fn as_str(self) -> &'static str {
        match self {
            RegType::Int => "int",
            RegType::CodePtr => "code_ptr",
            RegType::DataPtr => "data_ptr",
        }
    }
}

/// One register: what GDB calls it, and where it lives in the core's state
/// chunk.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct RegDesc {
    /// The name GDB shows, and the one `$name` resolves.
    pub name: &'static str,
    /// Width in bytes, both on the wire and in the chunk.
    pub bytes: usize,
    /// Byte offset of the register inside the device's state chunk. The chunk
    /// encoding is flat little-endian (`core::state`), so this is a slice.
    pub offset: usize,
    /// How GDB should present it.
    pub ty: RegType,
}

impl RegDesc {
    /// A general-purpose integer register.
    ///
    /// Every call site sits behind a `cpu-*` feature, so this is genuinely
    /// unused in a `--features gdb` build with no core enabled. That is a real
    /// configuration (it builds a stub that can serve no machine), hence the
    /// allow rather than a `cfg` disjunction that would need editing, and
    /// would warn again, every time a core lands.
    #[allow(dead_code)]
    const fn int(name: &'static str, bytes: usize, offset: usize) -> RegDesc {
        RegDesc {
            name,
            bytes,
            offset,
            ty: RegType::Int,
        }
    }
}

/// A counter that only moves when an instruction retires.
///
/// Single-stepping needs to know when *one instruction* has finished, and a
/// program counter cannot answer that: a two-byte branch to itself never moves
/// it, and a multi-cycle instruction has not moved it yet. Every rsemu core
/// keeps a cycle counter in its chunk, and that counter changing is the exact
/// signal wanted (`ROADMAP.md` §6: cycle accounting is per access, so it ticks
/// once work is actually done).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct RetireCounter {
    /// Byte offset in the state chunk.
    pub offset: usize,
    /// Width in bytes.
    pub bytes: usize,
}

/// Everything the stub needs to debug one kind of CPU.
#[derive(Debug)]
pub struct Arch {
    /// The device class this describes.
    pub class: &'static DeviceClass,
    /// The class state version whose chunk layout [`Arch::regs`]'s offsets were
    /// read off. A different version means the layout may have moved and the
    /// offsets are no longer trustworthy — see [`Arch::check`].
    pub verified_version: u32,
    /// The feature name in the target description.
    ///
    /// `org.rsemu.*` rather than `org.gnu.gdb.*` on purpose: an `org.gnu.gdb.…`
    /// name is a promise to GDB that the feature contains exactly the registers
    /// GDB's own gdbarch expects for it, in its order, and none of these do.
    pub feature: &'static str,
    /// The `<architecture>` element, when upstream GDB has a gdbarch by that
    /// name.
    ///
    /// `None` means it does not, and stock GDB will refuse the whole
    /// description — see the module docs. Every other client, and every GDB
    /// carrying a port for that CPU, reads the register map and works.
    pub architecture: Option<&'static str>,
    /// The registers, in `g`-packet order.
    pub regs: &'static [RegDesc],
    /// Which of [`Arch::regs`] is the program counter.
    pub pc: usize,
    /// The instruction-retirement signal, if this core has one.
    pub retire: Option<RetireCounter>,
}

impl Arch {
    /// Total width of the `g` packet, in bytes.
    #[must_use]
    pub fn packet_len(&self) -> usize {
        self.regs.iter().map(|r| r.bytes).sum()
    }

    /// The last byte of the chunk this map reaches, exclusive.
    ///
    /// A chunk shorter than this cannot be the one the map was written for.
    #[must_use]
    pub fn chunk_reach(&self) -> usize {
        let regs = self
            .regs
            .iter()
            .map(|r| r.offset + r.bytes)
            .max()
            .unwrap_or(0);
        match self.retire {
            Some(c) => regs.max(c.offset + c.bytes),
            None => regs,
        }
    }

    /// Whether this map still matches the class it describes.
    ///
    /// The one thing that can silently corrupt a debug session is a core
    /// changing its chunk layout under a table of byte offsets. A class
    /// version bump is exactly the event `ROADMAP.md` §4.5 requires for that,
    /// so it is the thing checked.
    #[must_use]
    pub fn check(&self) -> bool {
        self.class.version == self.verified_version
    }

    /// The target description GDB reads through `qXfer:features:read`.
    #[must_use]
    pub fn target_xml(&self) -> String {
        let mut xml = String::with_capacity(256 + self.regs.len() * 80);
        xml.push_str("<?xml version=\"1.0\"?>\n");
        xml.push_str("<!DOCTYPE target SYSTEM \"gdb-target.dtd\">\n");
        xml.push_str("<target version=\"1.0\">\n");
        if let Some(arch) = self.architecture {
            let _ = writeln!(xml, "  <architecture>{arch}</architecture>");
        }
        let _ = writeln!(xml, "  <feature name=\"{}\">", self.feature);
        for (i, reg) in self.regs.iter().enumerate() {
            let _ = writeln!(
                xml,
                "    <reg name=\"{}\" bitsize=\"{}\" type=\"{}\" regnum=\"{}\"/>",
                reg.name,
                reg.bytes * 8,
                reg.ty.as_str(),
                i
            );
        }
        xml.push_str("  </feature>\n");
        xml.push_str("</target>\n");
        xml
    }
}

// ---------------------------------------------------------------------------
// The table
// ---------------------------------------------------------------------------

/// The register map for a device class, if this build has one.
///
/// Adding a core to the debugger is one entry here. That is one entry more than
/// it should be — see the module docs — but it is the whole cost, and the target
/// description, the `g` packet and single-stepping all follow from it.
#[must_use]
pub fn for_class(class: &str) -> Option<&'static Arch> {
    ALL.iter().copied().find(|a| a.class.name == class)
}

/// Every register map this build knows.
#[must_use]
pub fn all() -> &'static [&'static Arch] {
    ALL
}

/// The maps, in no particular order; lookup is by class name.
static ALL: &[&Arch] = &[
    #[cfg(feature = "cpu-mos6502")]
    &MOS6502,
    #[cfg(feature = "cpu-z80")]
    &Z80,
    #[cfg(feature = "cpu-arm")]
    &ARM,
    #[cfg(feature = "cpu-riscv")]
    &RISCV,
    #[cfg(feature = "cpu-x86")]
    &I8086,
];

// -- MOS 6502 ---------------------------------------------------------------

/// `src/cpu/mos6502/mod.rs`'s `save`: `a x y s p` as bytes, `pc` as a `u16`,
/// then the cycle counter.
///
/// Re-verified at chunk version 3, which appends `waiting` at the end of the
/// chunk rather than beside `halted`, precisely so the prefix these offsets
/// index keeps the layout version 2 wrote. Nothing below moved.
#[cfg(feature = "cpu-mos6502")]
static MOS6502_REGS: &[RegDesc] = &[
    RegDesc::int("a", 1, 0),
    RegDesc::int("x", 1, 1),
    RegDesc::int("y", 1, 2),
    RegDesc {
        name: "sp",
        bytes: 1,
        offset: 3,
        ty: RegType::DataPtr,
    },
    RegDesc::int("p", 1, 4),
    RegDesc {
        name: "pc",
        bytes: 2,
        offset: 5,
        ty: RegType::CodePtr,
    },
];

/// The MOS 6502.
///
/// No `<architecture>`: upstream GDB has no 6502 gdbarch and will therefore
/// refuse this description. Every other client works — see the module docs.
#[cfg(feature = "cpu-mos6502")]
pub static MOS6502: Arch = Arch {
    class: &crate::cpu::mos6502::CLASS,
    verified_version: 3,
    feature: "org.rsemu.mos6502",
    architecture: None,
    regs: MOS6502_REGS,
    pc: 5,
    retire: Some(RetireCounter {
        offset: 7,
        bytes: 8,
    }),
};

// -- Zilog Z80 --------------------------------------------------------------

/// `src/cpu/z80/mod.rs`'s `save` writes thirteen `u16`s — `af bc de hl ix iy sp
/// pc wz af' bc' de' hl'` — then `i` and `r`. The order below is the one a Z80
/// programmer reads, which is why it is not the chunk's.
#[cfg(feature = "cpu-z80")]
static Z80_REGS: &[RegDesc] = &[
    RegDesc::int("af", 2, 0),
    RegDesc::int("bc", 2, 2),
    RegDesc::int("de", 2, 4),
    RegDesc::int("hl", 2, 6),
    RegDesc {
        name: "sp",
        bytes: 2,
        offset: 12,
        ty: RegType::DataPtr,
    },
    RegDesc {
        name: "pc",
        bytes: 2,
        offset: 14,
        ty: RegType::CodePtr,
    },
    RegDesc::int("ix", 2, 8),
    RegDesc::int("iy", 2, 10),
    RegDesc::int("af_", 2, 18),
    RegDesc::int("bc_", 2, 20),
    RegDesc::int("de_", 2, 22),
    RegDesc::int("hl_", 2, 24),
    RegDesc::int("i", 1, 26),
    RegDesc::int("r", 1, 27),
    // The undocumented internal latch. It is architectural state on this core
    // — flag bits 3 and 5 come out of it — so a debugger that hides it is
    // hiding the thing a Z80 conformance failure is usually about.
    RegDesc::int("wz", 2, 16),
];

/// The Zilog Z80.
#[cfg(feature = "cpu-z80")]
pub static Z80: Arch = Arch {
    class: &crate::cpu::z80::CLASS,
    verified_version: 1,
    feature: "org.rsemu.z80",
    architecture: None,
    regs: Z80_REGS,
    pc: 5,
    // 13 u16 + i + r + iff1 + iff2 + im + halted + ei_pending + after_ld_ir + q
    retire: Some(RetireCounter {
        offset: 35,
        bytes: 8,
    }),
};

// -- ARMv5TE ----------------------------------------------------------------

/// `src/cpu/arm/mod.rs`'s `save`: `r[0..16]` then `cpsr`, all `u32`.
#[cfg(feature = "cpu-arm")]
static ARM_REGS: [RegDesc; 17] = arm_regs();

#[cfg(feature = "cpu-arm")]
const fn arm_regs() -> [RegDesc; 17] {
    const NAMES: [&str; 16] = [
        "r0", "r1", "r2", "r3", "r4", "r5", "r6", "r7", "r8", "r9", "r10", "r11", "r12", "sp",
        "lr", "pc",
    ];
    let mut out = [RegDesc::int("cpsr", 4, 64); 17];
    let mut i = 0;
    while i < 16 {
        out[i] = RegDesc {
            name: NAMES[i],
            bytes: 4,
            offset: i * 4,
            ty: match i {
                13 => RegType::DataPtr,
                15 => RegType::CodePtr,
                _ => RegType::Int,
            },
        };
        i += 1;
    }
    out
}

/// The ARM926EJ-S-class core.
#[cfg(feature = "cpu-arm")]
pub static ARM: Arch = Arch {
    class: &crate::cpu::arm::CLASS,
    verified_version: 1,
    feature: "org.rsemu.arm",
    architecture: Some("arm"),
    regs: &ARM_REGS,
    pc: 15,
    // r[16] + cpsr + banked_sp_lr[6][2] + banked_r8_r12[2][5] + spsr[5], all u32.
    retire: Some(RetireCounter {
        offset: 176,
        bytes: 8,
    }),
};

// -- RISC-V -----------------------------------------------------------------

/// `src/cpu/riscv/mod.rs`'s `save`: `x[0..32]`, then `f[0..32]`, then `pc`, all
/// `u64`.
#[cfg(feature = "cpu-riscv")]
static RISCV_REGS: [RegDesc; 33] = riscv_regs();

#[cfg(feature = "cpu-riscv")]
const fn riscv_regs() -> [RegDesc; 33] {
    const NAMES: [&str; 32] = [
        "zero", "ra", "sp", "gp", "tp", "t0", "t1", "t2", "fp", "s1", "a0", "a1", "a2", "a3", "a4",
        "a5", "a6", "a7", "s2", "s3", "s4", "s5", "s6", "s7", "s8", "s9", "s10", "s11", "t3", "t4",
        "t5", "t6",
    ];
    let mut out = [RegDesc {
        name: "pc",
        bytes: 8,
        offset: 512,
        ty: RegType::CodePtr,
    }; 33];
    let mut i = 0;
    while i < 32 {
        out[i] = RegDesc {
            name: NAMES[i],
            bytes: 8,
            offset: i * 8,
            ty: if i == 2 {
                RegType::DataPtr
            } else {
                RegType::Int
            },
        };
        i += 1;
    }
    out
}

/// RV64GC.
///
/// The chunk stores every `x` register as a `u64` whatever `xlen` is, and this
/// map presents them that way. On an RV32 machine GDB is therefore told the
/// registers are 64 bits wide and shows a sign-extended value — correct
/// numbers, wrong width. Fixing it needs the core's `xlen` property, which is
/// not reachable through `dyn Device`; see the module docs.
#[cfg(feature = "cpu-riscv")]
pub static RISCV: Arch = Arch {
    class: &crate::cpu::riscv::CLASS,
    verified_version: 1,
    feature: "org.rsemu.riscv",
    architecture: Some("riscv:rv64"),
    regs: &RISCV_REGS,
    pc: 32,
    retire: Some(RetireCounter {
        offset: 520,
        bytes: 8,
    }),
};

// -- Intel 8086 -------------------------------------------------------------

/// `src/cpu/x86/mod.rs`'s `save` walks `Reg::ALL`: `ax cx dx bx sp bp si di es
/// cs ss ds ip flags`, every one a `u16`.
#[cfg(feature = "cpu-x86")]
static I8086_REGS: &[RegDesc] = &[
    RegDesc::int("ax", 2, 0),
    RegDesc::int("cx", 2, 2),
    RegDesc::int("dx", 2, 4),
    RegDesc::int("bx", 2, 6),
    RegDesc {
        name: "sp",
        bytes: 2,
        offset: 8,
        ty: RegType::DataPtr,
    },
    RegDesc::int("bp", 2, 10),
    RegDesc::int("si", 2, 12),
    RegDesc::int("di", 2, 14),
    RegDesc::int("es", 2, 16),
    RegDesc::int("cs", 2, 18),
    RegDesc::int("ss", 2, 20),
    RegDesc::int("ds", 2, 22),
    RegDesc {
        name: "ip",
        bytes: 2,
        offset: 24,
        ty: RegType::CodePtr,
    },
    RegDesc::int("flags", 2, 26),
];

/// The Intel 8086/8088 in real mode.
///
/// No `<architecture>`: GDB's `i8086` gdbarch expects the i386 register
/// numbering and a 32-bit `g` packet, and telling it "i8086" while sending this
/// layout would make it read the wrong halves of the wrong registers.
#[cfg(feature = "cpu-x86")]
pub static I8086: Arch = Arch {
    class: &crate::cpu::x86::CLASS,
    verified_version: 1,
    feature: "org.rsemu.i8086",
    architecture: None,
    regs: I8086_REGS,
    pc: 12,
    retire: Some(RetireCounter {
        offset: 28,
        bytes: 8,
    }),
};

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn every_map_still_matches_the_class_it_describes() {
        for arch in all() {
            assert!(
                arch.check(),
                "{}: the register map was written against state version {} but the class \
                 is at version {}. Re-read that core's `save` and move the offsets.",
                arch.class.name,
                arch.verified_version,
                arch.class.version
            );
        }
    }

    #[test]
    fn no_map_has_a_duplicate_name_or_an_out_of_range_pc() {
        for arch in all() {
            assert!(arch.pc < arch.regs.len(), "{}", arch.class.name);
            assert_eq!(
                arch.regs[arch.pc].ty,
                RegType::CodePtr,
                "{}: the register named as the pc is not typed as one",
                arch.class.name
            );
            for (i, reg) in arch.regs.iter().enumerate() {
                assert!(
                    !arch.regs[..i].iter().any(|other| other.name == reg.name),
                    "{}: two registers named `{}`",
                    arch.class.name,
                    reg.name
                );
                assert!(
                    matches!(reg.bytes, 1 | 2 | 4 | 8),
                    "{}: `{}` is {} bytes wide",
                    arch.class.name,
                    reg.name,
                    reg.bytes
                );
            }
        }
    }

    #[test]
    fn the_generated_description_names_every_register_once() {
        for arch in all() {
            let xml = arch.target_xml();
            assert!(xml.starts_with("<?xml"), "{}", arch.class.name);
            assert!(xml.contains(arch.feature), "{}", arch.class.name);
            for (i, reg) in arch.regs.iter().enumerate() {
                let expect = format!(
                    "<reg name=\"{}\" bitsize=\"{}\" type=\"{}\" regnum=\"{}\"/>",
                    reg.name,
                    reg.bytes * 8,
                    reg.ty.as_str(),
                    i
                );
                assert!(
                    xml.contains(&expect),
                    "{}: missing {expect}",
                    arch.class.name
                );
            }
        }
    }

    #[cfg(feature = "cpu-mos6502")]
    #[test]
    fn the_6502_g_packet_is_seven_bytes() {
        assert_eq!(MOS6502.packet_len(), 7);
        assert_eq!(MOS6502.chunk_reach(), 15);
    }
}
