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
//! order and at its widths. A map that cannot make that claim uses an
//! `org.rsemu.*` name instead, which is why GDB rejects its description and
//! falls back to a built-in layout.
//!
//! Six maps here **can** make it, and do: [`A64`], [`AMD64`], [`I386`],
//! [`I8086`], [`V7M`] and [`M68K`]. GDB's AArch64 gdbarch wants `x0`-`x30`,
//! `sp`, `pc` and `cpsr` and nothing else in `org.gnu.gdb.aarch64.core`; its
//! x86 gdbarch wants the integer file *and* the eight x87 registers with their
//! eight control words in `org.gnu.gdb.i386.core`; its M-profile gdbarch wants
//! `r0`-`r12`, `sp`, `lr`, `pc` and `xpsr` in `org.gnu.gdb.arm.m-profile`; its
//! m68k gdbarch wants `d0`-`d7`, `a0`-`a5`, `fp`, `sp`, `ps` and `pc` in
//! `org.gnu.gdb.m68k.core`. Each of those is supplied exactly, so each claims
//! the `<architecture>` with it and the description is *accepted* rather than
//! rejected. That is the shape to aim at; `org.rsemu.*` is the honest fallback,
//! not the house style.
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
    /// An x87 80-bit extended-precision value.
    ///
    /// `i387_ext` is one of the target-description DTD's predefined types, and
    /// it is the only thing GDB will accept for `st0`-`st7` in
    /// `org.gnu.gdb.i386.core`. Ten bytes on the wire, and ten contiguous
    /// little-endian bytes in `cpu.x86`'s chunk — the 64-bit significand
    /// followed by the sign-and-exponent halfword, which is the in-memory
    /// layout of the 80-bit format itself (Intel SDM Vol. 1 §4.2.2).
    F80,
}

impl RegType {
    /// The name this type has in a target description.
    #[must_use]
    pub const fn as_str(self) -> &'static str {
        match self {
            RegType::Int => "int",
            RegType::CodePtr => "code_ptr",
            RegType::DataPtr => "data_ptr",
            RegType::F80 => "i387_ext",
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

/// A little-endian field of a chunk, widened.
///
/// Shared by every [`Computed`] hook below, because a chunk is flat
/// little-endian whatever the guest's byte order is (`core::state`) and every
/// hook that has to *assemble* a register rather than slice one needs the same
/// three lines.
#[cfg(any(feature = "cpu-x86", feature = "cpu-arm-a64"))]
fn field(chunk: &[u8], offset: usize, bytes: usize) -> Option<u64> {
    let slice = chunk.get(offset..offset.checked_add(bytes)?)?;
    let mut value = 0u64;
    for (i, byte) in slice.iter().enumerate() {
        value |= u64::from(*byte) << (i * 8);
    }
    Some(value)
}

// ---------------------------------------------------------------------------
// The table
// ---------------------------------------------------------------------------

/// The register map for a device class, if this build has one.
///
/// Adding a core to the debugger is one entry here. That is one entry more than
/// it should be — see the module docs — but it is the whole cost, and the target
/// description, the `g` packet and single-stepping all follow from it.
///
/// One class has **two** maps: `cpu.x86` is a 16-bit 8088 and an x86-64 part
/// depending on how the machine file configured it, and those are different
/// register files. This returns the narrow one, which is the safe answer for a
/// caller that knows nothing about the instance; [`for_cpu`] is what a debugger
/// with a machine in front of it should call.
#[must_use]
pub fn for_class(class: &str) -> Option<&'static Arch> {
    ALL.iter().copied().find(|a| a.class.name == class)
}

/// The register map for one *instance* of a CPU class.
///
/// `space_bits` is the width of the address space that CPU runs on, as the
/// machine file declared it, and `None` means it has none.
///
/// # Why the address space decides the x86 register view
///
/// `cpu.x86` is one class covering an 8088 and an x86-64 part, and GDB needs to
/// be told which: a 64-bit guest debugged through the 32-bit window cannot show
/// `R8`-`R15`, the high halves of `RAX`-`RDI`, or an `RIP` above four gigabytes,
/// and a real-mode guest presented as x86-64 gets disassembled as x86-64. The
/// register file is per instance and the map is per class, so something has to
/// choose.
///
/// The instance's `variant` property is what one would *like* to ask, and there
/// is no route to it: `Machine`'s `DeviceEntry` carries the class, the clock
/// domain, the requester and the space, and not the properties the class was
/// constructed from. What it does carry is the **address space**, and its width
/// is the same decision written down by the same person: every board in
/// `machines/` that says `variant = "x86-64"` gives its core a 64-bit space
/// (`q35`, `q35-uefi`, `q35-linux`, `q35-linux-smp`, `pc64`), and every board
/// that does not gives it thirty-two bits or fewer (`pc-at`, `pc-at-smp`,
/// `pc-apic`, and the 20-bit `x86-mini` fixture). A core on a bus wider than
/// thirty-two bits is one whose board was built for a 64-bit part.
///
/// It is a proxy and it is named as one. The fix that removes it is a register
/// view — or merely a variant accessor — on `Device`, which is
/// [`super`]'s standing "what is not here"; until then this is the honest
/// approximation, and `tests/gdb_x86_64.rs` pins both halves of it.
#[must_use]
pub fn for_cpu(class: &str, space_bits: Option<u32>) -> Option<&'static Arch> {
    #[cfg(feature = "cpu-x86")]
    if class == crate::cpu::x86::CLASS.name {
        return Some(if space_bits.is_some_and(|bits| bits > 32) {
            &AMD64
        } else {
            &I386
        });
    }
    let _ = space_bits;
    for_class(class)
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
    // The narrow x86 view comes first, so `for_class` — which has no instance
    // to look at — answers with the one that is right for a real-mode guest.
    #[cfg(feature = "cpu-x86")]
    &I386,
    #[cfg(feature = "cpu-x86")]
    &AMD64,
    #[cfg(feature = "cpu-x86")]
    &I8086,
    #[cfg(feature = "cpu-sm83")]
    &SM83,
    #[cfg(feature = "cpu-arm-v7m")]
    &V7M,
    #[cfg(feature = "cpu-m68k")]
    &M68K,
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

// -- Intel x86 --------------------------------------------------------------

/// Where `cpu.x86`'s chunk stops being a table of constants.
///
/// `src/cpu/x86/mod.rs`'s `save` opens with `Reg::ALL` — `eax ecx edx ebx esp
/// ebp esi edi eip eflags cs ss ds es fs gs`, every one a `u32` — and that is
/// **gdb's own i386 core ordering**, which the core's `save` was written to
/// match. The first sixty-four bytes of a saved core are therefore the 32-bit
/// `g` packet's register block, and [`I386_REGS`]'s offsets are the identity
/// map rather than a translation that could drift.
///
/// Everything a 64-bit debugger wants is behind a **variable-length field**.
/// The chunk continues with the segment descriptors, the tables, the control
/// and debug registers, some flags, and then the **prefetch queue**: a byte
/// saying how many bytes are queued, followed by exactly that many. The bus
/// interface unit's queue is architectural on this core (`src/cpu/x86/exec.rs`
/// says why), it is four bytes deep on an 8088 and sixteen on a 386, and how
/// full it is changes as the guest runs. So the long-mode block, and the
/// floating-point block behind it, move under the debugger between one stop and
/// the next, and no table of constants can name them.
///
/// That is what [`Computed`] is for, and it is the whole reason a 64-bit
/// register view took a mechanism rather than sixteen more lines: `RAX` is not
/// at an offset.
#[cfg(feature = "cpu-x86")]
mod x86_layout {
    /// The byte saying how many bytes are in the prefetch queue.
    ///
    /// 16 registers * 4 + `cycles` + six segments * 18 + `ldtr`/`task` * 18 +
    /// `gdtr`/`idtr` * 12 + `cr0` + `cr2` + `cr3` + eight debug registers +
    /// eight test registers + four flags + `open_bus` + `faults` +
    /// `last_fault`.
    pub(super) const QUEUE_TAG: usize = 377;
    /// What follows the queue before the long-mode block: `debt`, the three
    /// interrupt-line flags, the pending vector and the A20 gate.
    pub(super) const AFTER_QUEUE: usize = 13;

    /// The sixteen 64-bit general registers, then `RIP`, relative to the start
    /// of the long-mode block. `Reg::WIDE`'s order, which is ModRM's.
    pub(super) const WIDE: usize = 0;
    /// The x87 register file: eight ten-byte values, significand first.
    pub(super) const ST: usize = 208;
    /// `FCW`, as a `u16`.
    pub(super) const FCTRL: usize = 288;
    /// `FSW`.
    pub(super) const FSTAT: usize = 290;
    /// `FTW`.
    pub(super) const FTAG: usize = 292;
    /// The last x87 opcode.
    pub(super) const FOP: usize = 294;
    /// The last x87 instruction pointer, as a `u64`; gdb sees its low half.
    pub(super) const FIOFF: usize = 296;
    /// The last x87 data pointer, as a `u64`.
    pub(super) const FOOFF: usize = 304;
    /// The code selector that went with `FIOFF`, as a `u16`.
    pub(super) const FISEG: usize = 312;
    /// The data selector that went with `FOOFF`.
    pub(super) const FOSEG: usize = 314;
    /// The first byte after the floating-point control block.
    pub(super) const FP_END: usize = 316;

    /// The shortest chunk either x86 map can decode: the whole prefix with an
    /// empty queue, out to the end of the floating-point control block.
    pub(super) const MIN_REACH: usize = QUEUE_TAG + 1 + AFTER_QUEUE + FP_END;

    /// Where the long-mode block starts, for this chunk.
    ///
    /// `None` when the chunk is too short to hold the byte that says.
    pub(super) fn base(chunk: &[u8]) -> Option<usize> {
        let queued = usize::from(*chunk.get(QUEUE_TAG)?);
        Some(QUEUE_TAG + 1 + queued + AFTER_QUEUE)
    }

    /// Where gdb's general register `n` lives in the long-mode block.
    ///
    /// gdb's AMD64 gdbarch numbers the integer file `rax rbx rcx rdx rsi rdi
    /// rbp rsp r8`-`r15` — the AMD64 DWARF numbering, which is *not* the ModRM
    /// order the chunk is written in. This is the one place the two are
    /// reconciled.
    pub(super) const GP_TO_CHUNK: [usize; 16] =
        [0, 3, 1, 2, 6, 7, 5, 4, 8, 9, 10, 11, 12, 13, 14, 15];
}

/// Read the eight x87 registers and the eight control words gdb's
/// `org.gnu.gdb.i386.core` requires after the integer file.
///
/// `first` is the register number `st0` has in this map — 24 for [`AMD64`], 16
/// for [`I386`], which is the only thing that differs between the two.
///
/// The mapping from gdb's names to the chunk is the x87 environment as the
/// SDM lays it out (Vol. 1 §8.1.8): `fioff`/`fiseg` are the last instruction's
/// pointer and selector, `fooff`/`foseg` the last data operand's. gdb declares
/// all eight as 32-bit integers whatever the mode, so the halfword fields are
/// zero-extended on the way out.
#[cfg(feature = "cpu-x86")]
fn x87_read(chunk: &[u8], base: usize, index: usize, first: usize, out: &mut Vec<u8>) -> Access {
    use x86_layout as l;
    let Some(slot) = index.checked_sub(first) else {
        return Access::Slice;
    };
    if slot < 8 {
        let at = base + l::ST + slot * 10;
        return match chunk.get(at..at + 10) {
            Some(bytes) => {
                out.extend_from_slice(bytes);
                Access::Done
            }
            None => Access::Refused,
        };
    }
    let (offset, bytes) = match slot {
        8 => (l::FCTRL, 2),
        9 => (l::FSTAT, 2),
        10 => (l::FTAG, 2),
        11 => (l::FISEG, 2),
        12 => (l::FIOFF, 4),
        13 => (l::FOSEG, 2),
        14 => (l::FOOFF, 4),
        15 => (l::FOP, 2),
        _ => return Access::Slice,
    };
    match field(chunk, base + offset, bytes) {
        Some(value) => {
            out.extend_from_slice(&(value as u32).to_le_bytes());
            Access::Done
        }
        None => Access::Refused,
    }
}

/// The inverse of [`x87_read`].
///
/// A halfword field takes the low sixteen bits of what gdb sent, and
/// `fioff`/`fooff` take the low thirty-two of a field the core keeps as a
/// `u64` — leaving the high half alone rather than zeroing it, because gdb
/// never saw it and a whole-file write must not destroy state the client could
/// not read.
#[cfg(feature = "cpu-x86")]
fn x87_write(chunk: &mut [u8], base: usize, index: usize, first: usize, data: &[u8]) -> Access {
    use x86_layout as l;
    let Some(slot) = index.checked_sub(first) else {
        return Access::Slice;
    };
    if slot < 8 {
        let at = base + l::ST + slot * 10;
        let (Some(dst), true) = (chunk.get_mut(at..at + 10), data.len() == 10) else {
            return Access::Refused;
        };
        dst.copy_from_slice(data);
        return Access::Done;
    }
    let (offset, bytes) = match slot {
        8 => (l::FCTRL, 2),
        9 => (l::FSTAT, 2),
        10 => (l::FTAG, 2),
        11 => (l::FISEG, 2),
        12 => (l::FIOFF, 4),
        13 => (l::FOSEG, 2),
        14 => (l::FOOFF, 4),
        15 => (l::FOP, 2),
        _ => return Access::Slice,
    };
    let Ok(word) = <[u8; 4]>::try_from(data) else {
        return Access::Refused;
    };
    match chunk.get_mut(base + offset..base + offset + bytes) {
        Some(dst) => {
            dst.copy_from_slice(&word[..bytes]);
            Access::Done
        }
        None => Access::Refused,
    }
}

/// The sixteen integer registers gdb's 32-bit i386 gdbarch numbers first.
///
/// The identity map into the chunk's first sixty-four bytes — see
/// [`x86_layout`]. Re-verified at chunk version 8: every block the core has
/// added since version 2 (long mode, floating point, the multiprocessor state,
/// `IA32_MISC_ENABLE`, the MTRRs) was **appended**, precisely so this prefix
/// keeps the layout it has always had.
#[cfg(feature = "cpu-x86")]
static I386_REGS: [RegDesc; 32] = i386_regs();

#[cfg(feature = "cpu-x86")]
const fn i386_regs() -> [RegDesc; 32] {
    const NAMES: [&str; 16] = [
        "eax", "ecx", "edx", "ebx", "esp", "ebp", "esi", "edi", "eip", "eflags", "cs", "ss", "ds",
        "es", "fs", "gs",
    ];
    let mut out = [RegDesc::int("st0", 10, RegDesc::COMPUTED); 32];
    let mut i = 0;
    while i < 16 {
        out[i] = RegDesc {
            name: NAMES[i],
            bytes: 4,
            offset: i * 4,
            ty: match i {
                4 => RegType::DataPtr,
                8 => RegType::CodePtr,
                _ => RegType::Int,
            },
        };
        i += 1;
    }
    let mut i = 0;
    while i < 16 {
        out[16 + i] = x87_reg(i);
        i += 1;
    }
    out
}

/// One entry of the sixteen-register x87 block, in gdb's order.
///
/// Shared by both x86 maps, because gdb wants the same sixteen after the
/// integer file whether the integer file is thirty-two bits wide or sixty-four.
#[cfg(feature = "cpu-x86")]
const fn x87_reg(slot: usize) -> RegDesc {
    const ST: [&str; 8] = ["st0", "st1", "st2", "st3", "st4", "st5", "st6", "st7"];
    const CTRL: [&str; 8] = [
        "fctrl", "fstat", "ftag", "fiseg", "fioff", "foseg", "fooff", "fop",
    ];
    if slot < 8 {
        RegDesc {
            name: ST[slot],
            bytes: 10,
            offset: RegDesc::COMPUTED,
            ty: RegType::F80,
        }
    } else {
        RegDesc::int(CTRL[slot - 8], 4, RegDesc::COMPUTED)
    }
}

/// The x87 block, which sits behind the prefetch queue. See [`Computed`].
#[cfg(feature = "cpu-x86")]
static I386_COMPUTED: Computed = Computed {
    read: i386_read,
    write: i386_write,
    reach: x86_layout::MIN_REACH,
    // Nothing in this map selects where anything else lives.
    selects: &[],
};

#[cfg(feature = "cpu-x86")]
fn i386_read(chunk: &[u8], index: usize, out: &mut Vec<u8>) -> Access {
    if index < 16 {
        return Access::Slice;
    }
    match x86_layout::base(chunk) {
        Some(base) => x87_read(chunk, base, index, 16, out),
        None => Access::Refused,
    }
}

#[cfg(feature = "cpu-x86")]
fn i386_write(chunk: &mut [u8], index: usize, data: &[u8]) -> Access {
    if index < 16 {
        return Access::Slice;
    }
    match x86_layout::base(chunk) {
        Some(base) => x87_write(chunk, base, index, 16, data),
        None => Access::Refused,
    }
}

/// The Intel x86 core on a bus of thirty-two bits or fewer: 8086 through 80486.
///
/// This is gdb's `org.gnu.gdb.i386.core` in full — the sixteen integer
/// registers, then `st0`-`st7`, `fctrl`, `fstat`, `ftag`, `fiseg`, `fioff`,
/// `foseg`, `fooff` and `fop` — so the description is **accepted** rather than
/// rejected, and `info float` works. It used to be the sixteen under an
/// `org.rsemu.i386` name, which gdb refused before falling back to a built-in
/// layout that happened to agree; agreeing by luck is not the same as being
/// right.
///
/// No `<architecture>`, deliberately, and this is the one map in the file that
/// leaves it out on purpose rather than for want of a gdbarch. The class covers
/// an 8088 in real mode and an 80486 in protected mode, gdb has a *different*
/// gdbarch for each (`i8086` and `i386`), and nothing reachable from here says
/// which this instance is — so the choice is left to the user's `set
/// architecture`, which the description then validates against either.
#[cfg(feature = "cpu-x86")]
pub static I386: Arch = Arch {
    class: &crate::cpu::x86::CLASS,
    verified_version: 8,
    feature: "org.gnu.gdb.i386.core",
    architecture: None,
    regs: &I386_REGS,
    pc: 8,
    retire: Some(RetireCounter {
        offset: 64,
        bytes: 8,
    }),
    computed: Some(&I386_COMPUTED),
};

/// The same core under its older class name, which a machine file may still
/// use.
///
/// `cpu.i8086` and `cpu.x86` are one implementation — `X86::as_i8086` changes
/// the class an instance reports and nothing else — so this is [`I386`] with a
/// different `class` pointer, sharing the register table so the two cannot
/// drift.
///
/// **The version it is verified against is not [`I386`]'s, and that is a defect
/// in the core rather than here.** `cpu.i8086`'s `DeviceClass.version` is 6
/// where `cpu.x86`'s is 8, for byte-identical `save` code: two of the class's
/// appends (`IA32_MISC_ENABLE` and the MTRRs) bumped one name and not the
/// other. The chunk layout is version 8's whichever name wrote it, so this map
/// is correct; what is wrong is that a `cpu.i8086` snapshot claims a version
/// its bytes do not match, which is a migration hazard for anything that ever
/// reads one. Reuniting them belongs in `src/cpu/x86/mod.rs`, and when it
/// happens [`Arch::check`] fails here and says so.
#[cfg(feature = "cpu-x86")]
pub static I8086: Arch = Arch {
    class: &crate::cpu::x86::I8086_CLASS,
    verified_version: 6,
    feature: "org.gnu.gdb.i386.core",
    architecture: None,
    regs: &I386_REGS,
    pc: 8,
    retire: Some(RetireCounter {
        offset: 64,
        bytes: 8,
    }),
    computed: Some(&I386_COMPUTED),
};

// -- x86-64 -----------------------------------------------------------------

/// gdb's AMD64 core register file: the integer sixteen, `rip`, `eflags`, the
/// six selectors, then the x87 block.
///
/// Forty registers, which is `AMD64_NUM_GREGS + I387_NUM_REGS` — the number
/// gdb's own AMD64 gdbarch validates `org.gnu.gdb.i386.core` against. Supplying
/// fewer means the description is rejected and gdb reads the `g` packet through
/// its built-in layout, which for x86-64 does *not* match this chunk.
///
/// Only seven of them are slices: `eflags` and the six selectors are in the
/// 32-bit prefix at their old offsets. Everything else is behind the prefetch
/// queue.
#[cfg(feature = "cpu-x86")]
static AMD64_REGS: [RegDesc; 40] = amd64_regs();

#[cfg(feature = "cpu-x86")]
const fn amd64_regs() -> [RegDesc; 40] {
    const GP: [&str; 16] = [
        "rax", "rbx", "rcx", "rdx", "rsi", "rdi", "rbp", "rsp", "r8", "r9", "r10", "r11", "r12",
        "r13", "r14", "r15",
    ];
    const SEG: [&str; 6] = ["cs", "ss", "ds", "es", "fs", "gs"];
    let mut out = [RegDesc::int("rax", 8, RegDesc::COMPUTED); 40];
    let mut i = 0;
    while i < 16 {
        out[i] = RegDesc {
            name: GP[i],
            bytes: 8,
            offset: RegDesc::COMPUTED,
            // `rsp` is register 7 in gdb's numbering, not 4.
            ty: if i == 7 {
                RegType::DataPtr
            } else {
                RegType::Int
            },
        };
        i += 1;
    }
    out[16] = RegDesc {
        name: "rip",
        bytes: 8,
        offset: RegDesc::COMPUTED,
        ty: RegType::CodePtr,
    };
    // `eflags` and the selectors keep the offsets the 32-bit prefix gives them.
    out[17] = RegDesc::int("eflags", 4, 36);
    let mut i = 0;
    while i < 6 {
        out[18 + i] = RegDesc::int(SEG[i], 4, 40 + i * 4);
        i += 1;
    }
    let mut i = 0;
    while i < 16 {
        out[24 + i] = x87_reg(i);
        i += 1;
    }
    out
}

/// The long-mode and floating-point blocks, both behind the prefetch queue.
#[cfg(feature = "cpu-x86")]
static AMD64_COMPUTED: Computed = Computed {
    read: amd64_read,
    write: amd64_write,
    reach: x86_layout::MIN_REACH,
    selects: &[],
};

#[cfg(feature = "cpu-x86")]
fn amd64_read(chunk: &[u8], index: usize, out: &mut Vec<u8>) -> Access {
    use x86_layout as l;
    if (17..24).contains(&index) {
        return Access::Slice;
    }
    let Some(base) = l::base(chunk) else {
        return Access::Refused;
    };
    if index < 17 {
        let slot = if index == 16 {
            16
        } else {
            l::GP_TO_CHUNK[index]
        };
        let at = base + l::WIDE + slot * 8;
        return match chunk.get(at..at + 8) {
            Some(bytes) => {
                out.extend_from_slice(bytes);
                Access::Done
            }
            None => Access::Refused,
        };
    }
    x87_read(chunk, base, index, 24, out)
}

#[cfg(feature = "cpu-x86")]
fn amd64_write(chunk: &mut [u8], index: usize, data: &[u8]) -> Access {
    use x86_layout as l;
    if (17..24).contains(&index) {
        return Access::Slice;
    }
    let Some(base) = l::base(chunk) else {
        return Access::Refused;
    };
    if index < 17 {
        let slot = if index == 16 {
            16
        } else {
            l::GP_TO_CHUNK[index]
        };
        let at = base + l::WIDE + slot * 8;
        let (Some(dst), true) = (chunk.get_mut(at..at + 8), data.len() == 8) else {
            return Access::Refused;
        };
        dst.copy_from_slice(data);
        // **And the same register's low half, in the 32-bit prefix.**
        //
        // The register file is in the chunk twice, and `cpu.x86`'s `load` does
        // not simply let the second copy win: for everything but `R8`-`R15` it
        // takes the **upper** thirty-two bits from the wide block and the lower
        // thirty-two from the prefix. That is deliberate and it is right — a
        // 32-bit debugger writing `ebx` through a `P` packet edits the prefix,
        // and a wide block that overwrote it would discard the write — but it
        // means a 64-bit write that touched only the wide block would land its
        // high half and lose its low one, which is a debugger that silently
        // corrupts a register.
        //
        // `Reg::ALL`'s first eight are `Reg::WIDE`'s first eight in the same
        // order, so the prefix slot is the same index; `RIP` is prefix slot 8.
        // `R8`-`R15` have no prefix copy and `load` says so, so they are the
        // one case that is genuinely one write.
        if index < 8 || index == 16 {
            let low = if index == 16 { 32 } else { slot * 4 };
            let Some(dst) = chunk.get_mut(low..low + 4) else {
                return Access::Refused;
            };
            dst.copy_from_slice(&data[..4]);
        }
        return Access::Done;
    }
    x87_write(chunk, base, index, 24, data)
}

/// The Intel x86 core on a bus wider than thirty-two bits: an x86-64 part.
///
/// This is the map `q35-uefi`, `q35-linux` and `pc64` get, and it is the reason
/// [`Computed`] was worth generalising past AArch64: a 64-bit guest was being
/// debugged through a 32-bit window not because the state was missing — the
/// core has written `RAX`-`R15` and `RIP` at full width since chunk version 4 —
/// but because a length-prefixed prefetch queue sits in front of them and a
/// table of constants cannot see past it.
///
/// The **write** side has a second obstacle, and it is the one that was
/// suspected first: the register file appears in the chunk twice, and `load`
/// merges the two copies rather than letting the later one win — the upper half
/// from the wide block, the lower half from the 32-bit prefix. So a 64-bit
/// register write is genuinely two writes, and this map's hook does both. Both
/// obstacles are real; only the first one needed a mechanism.
///
/// `org.gnu.gdb.i386.core` with `<architecture>i386:x86-64</architecture>`, both
/// claimed on the same terms as [`A64`]'s: the forty registers gdb's AMD64
/// gdbarch asks for, in its numbering, at its widths. `org.gnu.gdb.i386.sse` is
/// optional and is not offered — the core's `XMM` file *is* in the chunk, but an
/// `xmm` register is a union of vector types in a description rather than an
/// integer, which is a description to get right rather than sixteen lines to
/// add. Same reason [`A64`] leaves `org.gnu.gdb.aarch64.fpu` out.
#[cfg(feature = "cpu-x86")]
pub static AMD64: Arch = Arch {
    class: &crate::cpu::x86::CLASS,
    verified_version: 8,
    feature: "org.gnu.gdb.i386.core",
    architecture: Some("i386:x86-64"),
    regs: &AMD64_REGS,
    pc: 16,
    retire: Some(RetireCounter {
        offset: 64,
        bytes: 8,
    }),
    computed: Some(&AMD64_COMPUTED),
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

// -- Sharp SM83 -------------------------------------------------------------

/// `src/cpu/sm83/mod.rs`'s `save`: `a f b c d e h l` as bytes, `sp` and `pc` as
/// `u16`s, then the cycle counter.
///
/// The eight registers are bytes and are presented as bytes, rather than as the
/// `AF`/`BC`/`DE`/`HL` pairs a Game Boy programmer reads. The chunk writes each
/// pair high half first, and the wire format is little-endian, so a two-byte
/// register at offset 0 would be `F:A` rather than `AF` — a pair would have to
/// be a [`Computed`] byte swap, and inventing state that is not in the chunk to
/// present it backwards is worse than showing the eight registers the core
/// actually has.
#[cfg(feature = "cpu-sm83")]
static SM83_REGS: &[RegDesc] = &[
    RegDesc::int("a", 1, 0),
    RegDesc::int("f", 1, 1),
    RegDesc::int("b", 1, 2),
    RegDesc::int("c", 1, 3),
    RegDesc::int("d", 1, 4),
    RegDesc::int("e", 1, 5),
    RegDesc::int("h", 1, 6),
    RegDesc::int("l", 1, 7),
    RegDesc {
        name: "sp",
        bytes: 2,
        offset: 8,
        ty: RegType::DataPtr,
    },
    RegDesc {
        name: "pc",
        bytes: 2,
        offset: 10,
        ty: RegType::CodePtr,
    },
];

/// The Sharp SM83 — the Game Boy's core.
///
/// No `<architecture>`: the SM83 is not a Z80 and upstream gdb has no gdbarch
/// for it, so stock gdb refuses this description exactly as it refuses the
/// 6502's. Every other client reads the map and works, and everything that does
/// *not* go through the register file — breakpoints, single-stepping, memory
/// reads and writes, watchpoints, `monitor` — works under stock gdb too. That
/// is most of what debugging a Game Boy ROM is, which is why the map is here
/// rather than waiting for a port that is never going to land.
#[cfg(feature = "cpu-sm83")]
pub static SM83: Arch = Arch {
    class: &crate::cpu::sm83::CLASS,
    verified_version: 1,
    feature: "org.rsemu.sm83",
    architecture: None,
    regs: SM83_REGS,
    pc: 9,
    retire: Some(RetireCounter {
        offset: 12,
        bytes: 8,
    }),
    computed: None,
};

// -- ARMv7-M ----------------------------------------------------------------

/// `src/cpu/arm/v7m/mod.rs`'s `save`: `r[0..16]` as `u32`, the *other* stack
/// pointer, a byte saying which one that is, then `xPSR`.
///
/// `r[13]` is the selected stack pointer and `r[15]` the program counter
/// (`Regs`'s own documentation), so gdb's `sp`, `lr` and `pc` are slices of the
/// register array and no banking hook is needed: the chunk already holds the
/// active bank where an ARM programmer expects it, with the inactive one beside
/// it. That is the opposite of AArch64, where the chunk holds both banks and
/// nothing says which is `SP` — which is why this map needs no [`Computed`] and
/// [`A64`] does.
#[cfg(feature = "cpu-arm-v7m")]
static V7M_REGS: [RegDesc; 17] = v7m_regs();

#[cfg(feature = "cpu-arm-v7m")]
const fn v7m_regs() -> [RegDesc; 17] {
    const NAMES: [&str; 16] = [
        "r0", "r1", "r2", "r3", "r4", "r5", "r6", "r7", "r8", "r9", "r10", "r11", "r12", "sp",
        "lr", "pc",
    ];
    // `xpsr` is register 16, straight after `pc`, and lives past `sp_other`
    // (a `u32`) and `sp_is_psp` (a byte).
    let mut out = [RegDesc::int("xpsr", 4, 69); 17];
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

/// The ARMv7E-M core: `stm32f407`'s Cortex-M4.
///
/// `org.gnu.gdb.arm.m-profile` is claimed, and it is a claim this map keeps:
/// gdb's ARM gdbarch looks for that feature *first*, and finding it is how gdb
/// learns the target is M-profile at all — it wants `r0`-`r12`, `sp`, `lr`,
/// `pc` and `xpsr`, in that order, and that is the seventeen above. Getting
/// that right is worth more here than on any other core in the tree: an
/// M-profile gdb knows about `EXC_RETURN` addresses and the exception stack
/// frame, so a fault in an interrupt handler unwinds instead of ending at a
/// magic `0xfffffff9`.
///
/// `<architecture>arm</architecture>` rather than `armv7e-m`: `arm` is the bfd
/// architecture name every gdb build with an ARM port answers to, and the
/// M-profile feature is what carries the profile.
#[cfg(feature = "cpu-arm-v7m")]
pub static V7M: Arch = Arch {
    class: &crate::cpu::arm::v7m::CLASS,
    verified_version: 1,
    feature: "org.gnu.gdb.arm.m-profile",
    architecture: Some("arm"),
    regs: &V7M_REGS,
    pc: 15,
    // `r[16]` + `sp_other` + `sp_is_psp` + `xpsr` + `primask` + `faultmask` +
    // `basepri` + `control`.
    retire: Some(RetireCounter {
        offset: 80,
        bytes: 8,
    }),
    computed: None,
};

// -- Motorola 68000 ---------------------------------------------------------

/// `src/cpu/m68k/mod.rs`'s `save`: `d[0..8]` then `a[0..8]` as `u32`, the other
/// stack pointer, `pc`, then `sr` as a `u16`.
///
/// gdb's m68k core numbers `d0`-`d7`, `a0`-`a5`, `fp`, `sp`, `ps`, `pc` — so
/// `fp` is `a6` and `sp` is `a7`, and both are slices of the address-register
/// array. `a[7]` is whichever stack pointer the status register's supervisor
/// bit currently selects, with the other one beside it, so gdb's `sp` needs no
/// banking hook. `ps` does need one: gdb declares it thirty-two bits wide and
/// the core keeps sixteen.
#[cfg(feature = "cpu-m68k")]
static M68K_REGS: [RegDesc; 18] = m68k_regs();

#[cfg(feature = "cpu-m68k")]
const fn m68k_regs() -> [RegDesc; 18] {
    const D: [&str; 8] = ["d0", "d1", "d2", "d3", "d4", "d5", "d6", "d7"];
    const A: [&str; 8] = ["a0", "a1", "a2", "a3", "a4", "a5", "fp", "sp"];
    let mut out = [RegDesc::int("ps", 4, RegDesc::COMPUTED); 18];
    let mut i = 0;
    while i < 8 {
        out[i] = RegDesc::int(D[i], 4, i * 4);
        i += 1;
    }
    let mut i = 0;
    while i < 8 {
        out[8 + i] = RegDesc {
            name: A[i],
            bytes: 4,
            offset: 32 + i * 4,
            ty: if i == 7 {
                RegType::DataPtr
            } else {
                RegType::Int
            },
        };
        i += 1;
    }
    out[17] = RegDesc {
        name: "pc",
        bytes: 4,
        offset: 68,
        ty: RegType::CodePtr,
    };
    out
}

/// Where `SR` lives in the chunk: after `d`, `a`, `other_sp` and `pc`.
#[cfg(feature = "cpu-m68k")]
const M68K_SR: usize = 72;

/// `ps`, which gdb wants as a word and the core keeps as a halfword.
///
/// Not a banking hook — the chunk's `a[7]` is already the selected stack
/// pointer — but a widening one. Writing `ps` therefore changes the supervisor
/// bit **without** swapping `a[7]` and `other_sp`, because the chunk's contract
/// is "`a[7]` is the active stack pointer": a debugger that flips `S` is telling
/// the core to run in the other mode with the stack pointer it can see, which
/// is what a user typing `set $ps` means and what the core's `load` will do.
#[cfg(feature = "cpu-m68k")]
static M68K_COMPUTED: Computed = Computed {
    read: m68k_read,
    write: m68k_write,
    reach: M68K_SR + 2,
    selects: &[],
};

#[cfg(feature = "cpu-m68k")]
fn m68k_read(chunk: &[u8], index: usize, out: &mut Vec<u8>) -> Access {
    if index != 16 {
        return Access::Slice;
    }
    match chunk.get(M68K_SR..M68K_SR + 2) {
        Some(sr) => {
            out.extend_from_slice(&[sr[0], sr[1], 0, 0]);
            Access::Done
        }
        None => Access::Refused,
    }
}

#[cfg(feature = "cpu-m68k")]
fn m68k_write(chunk: &mut [u8], index: usize, data: &[u8]) -> Access {
    if index != 16 {
        return Access::Slice;
    }
    let (Some(dst), true) = (chunk.get_mut(M68K_SR..M68K_SR + 2), data.len() == 4) else {
        return Access::Refused;
    };
    dst.copy_from_slice(&data[..2]);
    Access::Done
}

/// The MC68000: `m68k-mini`'s core.
///
/// `org.gnu.gdb.m68k.core` with `<architecture>m68k</architecture>`, kept
/// exactly: gdb's m68k gdbarch wants `d0`-`d7`, `a0`-`a5`, `fp`, `sp`, `ps` and
/// `pc` in that feature, which is the eighteen above. The floating-point file
/// (`org.gnu.gdb.m68k.fp`) is optional and there is nothing to put in it — a
/// bare 68000 has no FPU, and this core models none.
#[cfg(feature = "cpu-m68k")]
pub static M68K: Arch = Arch {
    class: &crate::cpu::m68k::CLASS,
    verified_version: 2,
    feature: "org.gnu.gdb.m68k.core",
    architecture: Some("m68k"),
    regs: &M68K_REGS,
    pc: 17,
    // `sr` + the two prefetch words.
    retire: Some(RetireCounter {
        offset: 78,
        bytes: 8,
    }),
    computed: Some(&M68K_COMPUTED),
};

// -- What is deliberately not here ------------------------------------------

// `cpu.mips` has no map, and it is the one core left out on purpose rather than
// for want of time.
//
// Two things are wrong with it at once. First, **gdb would reject the
// description whatever it said**: `mips_gdbarch_init` requires
// `org.gnu.gdb.mips.cpu`, `org.gnu.gdb.mips.cp0` *and* `org.gnu.gdb.mips.fpu`,
// and returns no gdbarch if any is missing — the FPU feature is not optional,
// with a comment upstream saying it should be and the backend is not ready. The
// core models no FPU, so an honest description cannot be accepted and a
// dishonest one would have gdb read thirty-four registers that are not there.
//
// Second, its **retirement counter has no fixed offset**: `save` writes the
// data and instruction caches with `write_bytes`, which is length-prefixed, and
// `cycles` comes after both. Single-stepping would fall back to comparing the
// program counter, which is wrong for a branch to itself, or need a `Computed`
// for the counter — and `RetireCounter` has no hook. Both are fixable; neither
// is fixable from this file alone, and `mips-mini` is a bring-up board rather
// than something anyone debugs a program on. `gameboy` and `stm32f407` are.

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
                    // Ten is the x87 extended-precision width, which gdb's
                    // `i387_ext` is and nothing else in the tree is.
                    matches!(reg.bytes, 1 | 2 | 4 | 8 | 10),
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
