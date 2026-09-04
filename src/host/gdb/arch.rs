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
//! # Naming a feature `org.gnu.gdb.*` is a promise
//!
//! A feature called `org.gnu.gdb.<arch>.core` is a claim that it contains
//! exactly the registers GDB's gdbarch for that architecture expects, in its
//! order and at its widths. Most maps below cannot make that claim and use an
//! `org.rsemu.*` name instead, which is why GDB rejects their descriptions and
//! falls back to a built-in layout. [`A64`] **can**: GDB's AArch64 gdbarch
//! wants `x0`-`x30`, `sp`, `pc` and `cpsr` and nothing else in that feature,
//! which is precisely what `cpu.arm.a64` has, so it claims the name and the
//! `<architecture>` with it and the description is accepted. That is the shape
//! to aim at; `org.rsemu.*` is the honest fallback, not the house style.
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
//! A chunk is flat little-endian, so for almost everything a register is a
//! slice of it and an offset is the whole story. Two things break that and
//! [`Computed`] is the answer to both: a **banked** register, where which
//! bytes are the register depends on the value of another one, and a register
//! **assembled** out of several fields — AArch64's `SP` and `PSTATE`
//! respectively. On that core they also sit behind a *variable-length* field,
//! so the arithmetic that would find them is not constant either.
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
    /// The offset of a register whose bytes are **not** a fixed slice of the
    /// chunk, and which [`Computed`] answers for instead.
    ///
    /// Two things in the tree need it. `cpu.arm.a64`'s `SP` is one of two
    /// banked registers chosen by `PSTATE`, and its `PSTATE` is assembled out
    /// of four separate fields; and both of them sit *after* a field whose
    /// width depends on the value in it (the exclusive monitor's address is
    /// written only when the monitor is armed), so even the arithmetic that
    /// would find them is not constant. A sentinel rather than an `Option` in
    /// the struct, so that the ordinary entries below keep reading as tables.
    pub const COMPUTED: usize = usize::MAX;

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

/// What a [`Computed`] hook did with a register.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Access {
    /// Not this hook's business: the register is an ordinary slice of the
    /// chunk at [`RegDesc::offset`].
    Slice,
    /// Answered.
    Done,
    /// The hook is the right one and refused — a `PSTATE` naming an exception
    /// level this core does not have, say. The stub replies `E22` rather than
    /// writing something the core would reject on `load`.
    Refused,
}

/// How a core answers for a register a byte offset cannot name.
///
/// The register map is a table of byte offsets into a snapshot chunk, and for
/// almost everything that is enough: a chunk is flat little-endian, so a
/// register is a slice. Two things break it, and both are AArch64's:
///
/// * a **banked** register — `SP` is `SP_EL0` or `SP_EL1` depending on
///   `PSTATE.EL` and `PSTATE.SPSel`, so which slice it is depends on the
///   contents of another one;
/// * an **assembled** register — AArch64 has no `PSTATE` field; gdb's `cpsr`
///   is `NZCV`, `DAIF`, `EL` and `SPSel` composed into one word, which is
///   four slices rather than none.
///
/// And on that core they sit behind a **variable-length field**: the exclusive
/// monitor writes its address only when it is armed, so everything after it
/// moves by eight bytes depending on a byte. `reach` is therefore a function
/// of the chunk too.
///
/// A hook returning [`Access::Slice`] leaves the register to its static
/// offset, so a map only writes the entries it has to.
#[derive(Debug)]
pub struct Computed {
    /// Append register `index`'s little-endian bytes to `out`.
    pub read: fn(chunk: &[u8], index: usize, out: &mut Vec<u8>) -> Access,
    /// Patch register `index` into `chunk` from `data`.
    pub write: fn(chunk: &mut [u8], index: usize, data: &[u8]) -> Access,
    /// The shortest chunk this map can decode at all.
    ///
    /// A constant rather than a function of the chunk: it is the *minimum*
    /// over every value the variable-length fields can take, so a chunk that
    /// reaches it is long enough for the hooks to look at the bytes that say
    /// how long it really is.
    pub reach: usize,
    /// Computed registers that **select where the others live**, and so have
    /// to be written before them.
    ///
    /// A whole-register-file write (`G`) hands over every register at once, in
    /// `g`-packet order, and that order is GDB's rather than this map's: on
    /// AArch64 `sp` is register 31 and the `cpsr` that says which of the two
    /// banked stack pointers `sp` *is* is register 33. Writing them in the
    /// packet's order would put the user's stack pointer in the bank the old
    /// `PSTATE` selected and then change the selection, so `G` would appear to
    /// lose it — and a "fix" that simply wrote it twice would have changed
    /// both banks, which is two registers the user did not ask for.
    ///
    /// So the write is ordered instead: everything named here goes in first.
    /// Empty for a map whose computed registers do not interact.
    pub selects: &'static [usize],
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
    /// The hook for registers [`RegDesc::COMPUTED`] marks, if this map has
    /// any.
    pub computed: Option<&'static Computed>,
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
            .filter(|r| r.offset != RegDesc::COMPUTED)
            .map(|r| r.offset + r.bytes)
            .max()
            .unwrap_or(0);
        let regs = match self.retire {
            Some(c) => regs.max(c.offset + c.bytes),
            None => regs,
        };
        match self.computed {
            Some(c) => regs.max(c.reach),
            None => regs,
        }
    }

    /// Read register `index`, if a hook owns it.
    ///
    /// `None` means the caller should take [`RegDesc::offset`]'s slice, which
    /// is the answer for every register on every core but one.
    #[must_use]
    pub fn read_computed(&self, chunk: &[u8], index: usize) -> Option<Option<Vec<u8>>> {
        let hook = self.computed?;
        let mut out = Vec::new();
        match (hook.read)(chunk, index, &mut out) {
            Access::Slice => None,
            Access::Done => Some(Some(out)),
            Access::Refused => Some(None),
        }
    }

    /// Write register `index`, if a hook owns it. See
    /// [`read_computed`](Arch::read_computed).
    #[must_use]
    pub fn write_computed(&self, chunk: &mut [u8], index: usize, data: &[u8]) -> Option<bool> {
        let hook = self.computed?;
        match (hook.write)(chunk, index, data) {
            Access::Slice => None,
            Access::Done => Some(true),
            Access::Refused => Some(false),
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
    #[cfg(feature = "cpu-arm-aprofile")]
    &ARM,
    #[cfg(feature = "cpu-arm-a64")]
    &A64,
    #[cfg(feature = "cpu-riscv")]
    &RISCV,
    #[cfg(feature = "cpu-x86")]
    &I8086,
];

// -- MOS 6502 ---------------------------------------------------------------

/// `src/cpu/mos6502/mod.rs`'s `save`: `a x y s p` as bytes, `pc` as a `u16`,
/// then the cycle counter.
///
/// Re-verified at chunk version 4. Both v3's `waiting` and v4's `core_bus` are
/// appended at the *end* of the chunk rather than slotted in beside the fields
/// they belong with, precisely so the prefix these offsets index keeps the
/// layout version 2 wrote. Nothing below moved.
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
    verified_version: 4,
    feature: "org.rsemu.mos6502",
    architecture: None,
    regs: MOS6502_REGS,
    pc: 5,
    retire: Some(RetireCounter {
        offset: 7,
        bytes: 8,
    }),
    computed: None,
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
    verified_version: 2,
    feature: "org.rsemu.z80",
    architecture: None,
    regs: Z80_REGS,
    pc: 5,
    // 13 u16 + i + r + iff1 + iff2 + im + halted + ei_pending + after_ld_ir + q
    retire: Some(RetireCounter {
        offset: 35,
        bytes: 8,
    }),
    computed: None,
};

// -- ARMv5TE ----------------------------------------------------------------

/// `src/cpu/arm/aprofile/mod.rs`'s `save`: `r[0..16]` then `cpsr`, all `u32`.
///
/// Re-verified at chunk version 3. v3 appends CP15's registers at the *end* of
/// the chunk, and only when the core was built with one, so the prefix these
/// offsets index is byte-for-byte what v2 wrote. Nothing below moved.
#[cfg(feature = "cpu-arm-aprofile")]
static ARM_REGS: [RegDesc; 17] = arm_regs();

#[cfg(feature = "cpu-arm-aprofile")]
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
#[cfg(feature = "cpu-arm-aprofile")]
pub static ARM: Arch = Arch {
    class: &crate::cpu::arm::aprofile::CLASS,
    verified_version: 3,
    feature: "org.rsemu.arm",
    architecture: Some("arm"),
    regs: &ARM_REGS,
    pc: 15,
    // r[16] + cpsr + banked_sp_lr[6][2] + banked_r8_r12[2][5] + spsr[5], all u32.
    retire: Some(RetireCounter {
        offset: 176,
        bytes: 8,
    }),
    computed: None,
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
    computed: None,
};

// -- Intel 8086 -------------------------------------------------------------

/// `src/cpu/x86/mod.rs`'s `save` walks `Reg::ALL`: `eax ecx edx ebx esp ebp
/// esi edi eip eflags cs ss ds es fs gs`, every one a `u32`.
///
/// That is **gdb's own i386 core ordering**, and the x86 core's `save` was
/// written to match it: the first sixty-four bytes of a saved core are the
/// `g` packet's register block, so these offsets are the identity map rather
/// than a translation that could drift.
///
/// Re-verified at chunk version 4, which widened the register file to
/// sixty-four bits and added long mode. Nothing here moved: the prefix still
/// holds each register's **low half** as a `u32` in the same order, and the
/// full-width block was appended at the end of the chunk for exactly that
/// reason. A 64-bit register view for gdb would be a new `Arch` with the
/// `org.gnu.gdb.i386.core` feature replaced, not an edit to this one.
#[cfg(feature = "cpu-x86")]
static I8086_REGS: &[RegDesc] = &[
    RegDesc::int("eax", 4, 0),
    RegDesc::int("ecx", 4, 4),
    RegDesc::int("edx", 4, 8),
    RegDesc::int("ebx", 4, 12),
    RegDesc {
        name: "esp",
        bytes: 4,
        offset: 16,
        ty: RegType::DataPtr,
    },
    RegDesc::int("ebp", 4, 20),
    RegDesc::int("esi", 4, 24),
    RegDesc::int("edi", 4, 28),
    RegDesc {
        name: "eip",
        bytes: 4,
        offset: 32,
        ty: RegType::CodePtr,
    },
    RegDesc::int("eflags", 4, 36),
    RegDesc::int("cs", 4, 40),
    RegDesc::int("ss", 4, 44),
    RegDesc::int("ds", 4, 48),
    RegDesc::int("es", 4, 52),
    RegDesc::int("fs", 4, 56),
    RegDesc::int("gs", 4, 60),
];

/// The Intel x86 core: 8086 through 80486.
///
/// No `<architecture>`: gdb's `i386` gdbarch expects the x87 register block
/// after the core sixteen, and claiming the architecture without supplying it
/// would make gdb read the wrong halves of the wrong registers. The custom
/// feature name gives it the sixteen it can use and nothing it cannot.
///
/// The core's snapshot *does* now carry the x87 and `XMM` registers — chunk
/// version 5 appended them after the long-mode block — but this map is
/// unchanged and deliberately so: exposing them means claiming the `i386`
/// architecture and matching gdb's exact register numbering for `st0`-`st7`,
/// `fctrl`, `fstat`, `ftag`, `fiseg`, `fioff`, `foseg`, `fooff`, `fop` and the
/// `XMM` block, which is a description to get right rather than three lines to
/// add. Until then the sixteen core registers are all this offers, and the
/// version below records that the appended block was read and considered.
///
/// Re-verified at chunk version 6, which appends the multiprocessor block —
/// the wait-for-SIPI state, the two `INIT` levels and the Start-Up page — after
/// the floating-point one. Appended again, so again nothing here moved. None of
/// it is a *register*, so there is nothing for this map to grow even when it
/// does claim the `i386` architecture.
///
/// Re-verified again at chunk version 7, which appends `IA32_MISC_ENABLE`
/// after the multiprocessor block. Appended, so nothing moved; and it is a
/// model-specific register rather than one of the sixteen, so this map has
/// nothing to grow.
///
/// And once more at chunk version 8, which appends the memory-type range
/// registers after that. Twenty more model-specific registers, appended again,
/// so nothing here moved and there is again nothing for a map of the sixteen
/// core registers to grow.
#[cfg(feature = "cpu-x86")]
pub static I8086: Arch = Arch {
    class: &crate::cpu::x86::CLASS,
    verified_version: 8,
    feature: "org.rsemu.i386",
    architecture: None,
    regs: I8086_REGS,
    pc: 8,
    retire: Some(RetireCounter {
        offset: 64,
        bytes: 8,
    }),
    computed: None,
};

// -- AArch64 ----------------------------------------------------------------

/// Where `cpu.arm.a64`'s chunk stops being a table of constants.
///
/// `src/cpu/arm/a64/mod.rs`'s `save` writes, in order: `x[0..31]` as `u64`,
/// the thirty-two SIMD&FP registers as a low and a high `u64` each, `pc`,
/// `cycles`, `debt`, `faults`, a `wfi` byte, then the exclusive monitor — one
/// byte saying whether it is armed, **followed by an eight-byte address only
/// when it is**. Everything after that moves.
///
/// So: `x` at `0`, the vectors at `248`, `pc` at `760` and `cycles` at `768`
/// are constants and are written as offsets in the table; `SP` and `PSTATE`
/// are behind the hole and are [`RegDesc::COMPUTED`].
#[cfg(feature = "cpu-arm-a64")]
mod a64_layout {
    /// The byte saying whether the exclusive monitor is armed.
    pub(super) const EXCLUSIVE_TAG: usize = 793;
    /// The first byte after the monitor when it is *not* armed.
    pub(super) const AFTER_MONITOR: usize = 794;
    /// What being armed adds to every offset from here on.
    pub(super) const MONITOR_ADDR: usize = 8;
    /// `PSTATE.NZCV`, as a `u32`, relative to the end of the monitor.
    pub(super) const NZCV: usize = 0;
    /// `PSTATE.DAIF`, as a `u64`.
    pub(super) const DAIF: usize = 4;
    /// `PSTATE.EL`, as one byte: `0` for EL0, `1` for EL1.
    pub(super) const EL: usize = 12;
    /// `PSTATE.SPSel`, as one byte.
    pub(super) const SPSEL: usize = 13;
    /// The thirty system-register words, of which `SP_EL0` is the first and
    /// `SP_EL1` the second.
    pub(super) const SYSREGS: usize = 14;
    /// How many of those words there are; the interrupt lines follow them.
    pub(super) const SYSREG_WORDS: usize = 30;
    /// The shortest chunk this map can decode: the whole thing with the
    /// monitor disarmed, up to and including the interrupt lines.
    pub(super) const MIN_REACH: usize = AFTER_MONITOR + SYSREGS + SYSREG_WORDS * 8 + 8;

    /// Where the fields after the exclusive monitor start, for this chunk.
    ///
    /// `None` when the chunk is too short to hold the byte that says.
    pub(super) fn base(chunk: &[u8]) -> Option<usize> {
        let armed = *chunk.get(EXCLUSIVE_TAG)? != 0;
        Some(AFTER_MONITOR + if armed { MONITOR_ADDR } else { 0 })
    }

    /// Whether `SP` currently names `SP_EL1` rather than `SP_EL0`.
    ///
    /// DDI 0487 D1: at EL0 the stack pointer is always `SP_EL0` whatever
    /// `SPSel` says. The core's own `SysRegs::sp_is_el1` is the same rule; it
    /// is repeated rather than called because there is no route from a
    /// `dyn Device` to it.
    pub(super) fn sp_is_el1(chunk: &[u8], base: usize) -> Option<bool> {
        let el = *chunk.get(base + EL)?;
        let spsel = *chunk.get(base + SPSEL)?;
        Some(el == 1 && spsel != 0)
    }

    /// Where the selected stack pointer lives in this chunk.
    pub(super) fn sp_offset(chunk: &[u8], base: usize) -> Option<usize> {
        let bank = usize::from(sp_is_el1(chunk, base)?);
        Some(base + SYSREGS + bank * 8)
    }
}

/// `x0`-`x30`, `sp`, `pc`, `cpsr`: gdb's `org.gnu.gdb.aarch64.core`, in gdb's
/// order.
#[cfg(feature = "cpu-arm-a64")]
static A64_REGS: [RegDesc; 34] = a64_regs();

#[cfg(feature = "cpu-arm-a64")]
const fn a64_regs() -> [RegDesc; 34] {
    const NAMES: [&str; 31] = [
        "x0", "x1", "x2", "x3", "x4", "x5", "x6", "x7", "x8", "x9", "x10", "x11", "x12", "x13",
        "x14", "x15", "x16", "x17", "x18", "x19", "x20", "x21", "x22", "x23", "x24", "x25", "x26",
        "x27", "x28", "x29", "x30",
    ];
    let mut out = [RegDesc::int("cpsr", 4, RegDesc::COMPUTED); 34];
    let mut i = 0;
    while i < 31 {
        out[i] = RegDesc::int(NAMES[i], 8, i * 8);
        i += 1;
    }
    out[31] = RegDesc {
        name: "sp",
        bytes: 8,
        offset: RegDesc::COMPUTED,
        ty: RegType::DataPtr,
    };
    out[32] = RegDesc {
        name: "pc",
        bytes: 8,
        offset: 760,
        ty: RegType::CodePtr,
    };
    out
}

/// `SP` and `PSTATE`, which no byte offset can name. See [`Computed`].
#[cfg(feature = "cpu-arm-a64")]
static A64_COMPUTED: Computed = Computed {
    read: a64_read,
    write: a64_write,
    reach: a64_layout::MIN_REACH,
    // `cpsr` is register 33 and selects which bank `sp` (31) names, so it goes
    // in first whatever order the packet has them in.
    selects: &[33],
};

/// A little-endian field of a chunk, widened.
#[cfg(feature = "cpu-arm-a64")]
fn field(chunk: &[u8], offset: usize, bytes: usize) -> Option<u64> {
    let slice = chunk.get(offset..offset.checked_add(bytes)?)?;
    let mut value = 0u64;
    for (i, byte) in slice.iter().enumerate() {
        value |= u64::from(*byte) << (i * 8);
    }
    Some(value)
}

/// `PSTATE` as gdb's `cpsr`, and the selected stack pointer.
///
/// The `cpsr` encoding is the one an exception would save in `SPSR_EL1`
/// (DDI 0487 D1.11): `NZCV` at 31:28, `DAIF` at 9:6, `M[4]` clear for AArch64
/// and `M[3:0]` naming the level and the stack pointer — `0b0000` for EL0t,
/// `0b0100` for EL1t, `0b0101` for EL1h. That is what gdb's AArch64 gdbarch
/// decodes, and it is what the core's own `SysRegs::spsr` composes.
#[cfg(feature = "cpu-arm-a64")]
fn a64_read(chunk: &[u8], index: usize, out: &mut Vec<u8>) -> Access {
    use a64_layout as l;
    let Some(base) = l::base(chunk) else {
        return Access::Refused;
    };
    match index {
        31 => match l::sp_offset(chunk, base).and_then(|at| field(chunk, at, 8)) {
            Some(sp) => {
                out.extend_from_slice(&sp.to_le_bytes());
                Access::Done
            }
            None => Access::Refused,
        },
        33 => {
            let (Some(nzcv), Some(daif), Some(el), Some(el1)) = (
                field(chunk, base + l::NZCV, 4),
                field(chunk, base + l::DAIF, 8),
                field(chunk, base + l::EL, 1),
                l::sp_is_el1(chunk, base),
            ) else {
                return Access::Refused;
            };
            // `DAIF` is masked to its four bits rather than trusted whole: the
            // chunk holds the core's own field and this is the register gdb
            // will write back.
            let pstate = (nzcv & 0xf000_0000) | (daif & 0x3c0) | (el << 2) | u64::from(el1);
            out.extend_from_slice(&(pstate as u32).to_le_bytes());
            Access::Done
        }
        _ => Access::Slice,
    }
}

/// The inverse of [`a64_read`].
///
/// A `PSTATE` whose `M[3:0]` names AArch32 or an exception level this core
/// does not have is **refused**, exactly as the core's own `restore_pstate`
/// refuses it: writing the byte anyway would produce a chunk whose `load`
/// fails, and a failing `load` is a debugger that corrupted the machine.
#[cfg(feature = "cpu-arm-a64")]
fn a64_write(chunk: &mut [u8], index: usize, data: &[u8]) -> Access {
    use a64_layout as l;
    let Some(base) = l::base(chunk) else {
        return Access::Refused;
    };
    match index {
        31 => {
            let (Some(at), Ok(value)) = (l::sp_offset(chunk, base), <[u8; 8]>::try_from(data))
            else {
                return Access::Refused;
            };
            match chunk.get_mut(at..at + 8) {
                Some(slot) => {
                    slot.copy_from_slice(&value);
                    Access::Done
                }
                None => Access::Refused,
            }
        }
        33 => {
            let Ok(bytes) = <[u8; 4]>::try_from(data) else {
                return Access::Refused;
            };
            let pstate = u32::from_le_bytes(bytes);
            let (el, spsel) = match pstate & 0x1f {
                0b0_0000 => (0u8, 0u8),
                0b0_0100 => (1, 0),
                0b0_0101 => (1, 1),
                // `M[4]` set is a return to AArch32, and anything else names
                // EL2 or EL3. Neither exists on this core.
                _ => return Access::Refused,
            };
            let nzcv = pstate & 0xf000_0000;
            let daif = u64::from(pstate & 0x3c0);
            let ok = (|| {
                chunk
                    .get_mut(base + l::NZCV..base + l::NZCV + 4)?
                    .copy_from_slice(&nzcv.to_le_bytes());
                chunk
                    .get_mut(base + l::DAIF..base + l::DAIF + 8)?
                    .copy_from_slice(&daif.to_le_bytes());
                *chunk.get_mut(base + l::EL)? = el;
                *chunk.get_mut(base + l::SPSEL)? = spsel;
                Some(())
            })();
            match ok {
                Some(()) => Access::Done,
                None => Access::Refused,
            }
        }
        _ => Access::Slice,
    }
}

/// The AArch64 core.
///
/// The one map in this file that claims an `org.gnu.gdb.*` feature name, and
/// it is entitled to: gdb's AArch64 gdbarch requires
/// `org.gnu.gdb.aarch64.core` to hold `x0`-`x30`, `sp`, `pc` and `cpsr`, in
/// that order and at those widths, and that is exactly the thirty-four
/// registers above. The `org.gnu.gdb.aarch64.fpu` feature is optional and is
/// not offered: the SIMD&FP file is in the chunk, but a `V` register is a
/// union type in a description rather than an integer, which is a description
/// to get right rather than three lines to add.
///
/// Because the promise is kept, `<architecture>aarch64</architecture>` is
/// claimed too — so a gdb with an AArch64 gdbarch **accepts** this description
/// rather than falling back to a built-in layout, which is the difference
/// between this core and `cpu.x86` below.
#[cfg(feature = "cpu-arm-a64")]
pub static A64: Arch = Arch {
    class: &crate::cpu::arm::a64::CLASS,
    verified_version: 2,
    feature: "org.gnu.gdb.aarch64.core",
    architecture: Some("aarch64"),
    regs: &A64_REGS,
    pc: 32,
    // `cycles`, straight after `pc`.
    retire: Some(RetireCounter {
        offset: 768,
        bytes: 8,
    }),
    computed: Some(&A64_COMPUTED),
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

    /// The sentinel and the hook have to agree about which registers are
    /// which.
    ///
    /// `MachineTarget::write_registers` reads the sentinel to decide what to
    /// defer and the hook to decide what to write, so a register marked one
    /// way and answered the other would be written twice or not at all. This
    /// is the only place the two are compared, and it is cheap.
    #[test]
    fn the_computed_marker_and_the_hook_name_the_same_registers() {
        for arch in all() {
            let Some(hook) = arch.computed else {
                for reg in arch.regs {
                    assert_ne!(
                        reg.offset,
                        RegDesc::COMPUTED,
                        "{}: `{}` is marked computed but the map has no hook",
                        arch.class.name,
                        reg.name
                    );
                }
                continue;
            };
            // A chunk of the minimum length, all zeroes: enough for the hooks
            // to decide whether a register is theirs, which is all this asks.
            let chunk = vec![0u8; hook.reach];
            for (i, reg) in arch.regs.iter().enumerate() {
                let mut out = Vec::new();
                let owned = (hook.read)(&chunk, i, &mut out) != Access::Slice;
                assert_eq!(
                    owned,
                    reg.offset == RegDesc::COMPUTED,
                    "{}: `{}` is marked {} but the hook says {}",
                    arch.class.name,
                    reg.name,
                    if reg.offset == RegDesc::COMPUTED {
                        "computed"
                    } else {
                        "a plain slice"
                    },
                    if owned { "it owns it" } else { "it does not" }
                );
                if owned {
                    assert_eq!(out.len(), reg.bytes, "{}: `{}`", arch.class.name, reg.name);
                }
            }
            for index in hook.selects {
                assert!(
                    arch.regs
                        .get(*index)
                        .is_some_and(|r| r.offset == RegDesc::COMPUTED),
                    "{}: `selects` names register {index}, which is not a computed one",
                    arch.class.name
                );
            }
        }
    }

    #[cfg(feature = "cpu-arm-a64")]
    #[test]
    fn the_aarch64_g_packet_is_gdbs_own_layout() {
        // 31 * 8 + sp + pc + a four-byte cpsr, which is what GDB's AArch64
        // gdbarch expects of `org.gnu.gdb.aarch64.core`.
        assert_eq!(A64.packet_len(), 268);
        assert_eq!(A64.regs[31].name, "sp");
        assert_eq!(A64.regs[A64.pc].name, "pc");
        assert_eq!(A64.regs[33].name, "cpsr");
        // The chunk has to be long enough for the fields behind the exclusive
        // monitor even when the monitor is disarmed and they are at their
        // lowest offsets.
        assert_eq!(A64.chunk_reach(), a64_layout::MIN_REACH);
    }

    #[cfg(feature = "cpu-mos6502")]
    #[test]
    fn the_6502_g_packet_is_seven_bytes() {
        assert_eq!(MOS6502.packet_len(), 7);
        assert_eq!(MOS6502.chunk_reach(), 15);
    }
}
