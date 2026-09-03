//! Generating the device tree from the realized machine graph.
//!
//! `docs/platforms/riscv-virt.md` states the rule this module exists to obey:
//!
//! > rsemu **generates** the device tree from the realized machine graph and
//! > passes it to firmware. That is a genuine test of the machine model: if the
//! > DTB can be produced mechanically from the topology, the topology is
//! > well-formed.
//!
//! So nothing here writes an address, a size or an interrupt number down. Every
//! one of them is read back out of the machine that was actually built:
//!
//! | What | Where it comes from |
//! | --- | --- |
//! | a node's `reg` | the base and length of the mapping the `map` statement made |
//! | which node a mapping is | the [`NodeSpec`] the device published for that region |
//! | `interrupts` | the net a device's IRQ pin drives, looked up in the PLIC's own pin table |
//! | `timebase-frequency` | the clock domain the machine file gave the CLINT |
//! | `memory@…` | every RAM region mapped in the space |
//!
//! The two things that are *not* derived are the hart count and its ISA string,
//! and that is a limitation rather than a choice: a hart is not a region in any
//! address space, and there is no route from a `dyn Device` to a `Hart`
//! (`core::device` keeps `Any` out of the supertrait chain deliberately). They
//! are therefore declared on the `riscv.boot` object, and [`CpuSpec`] is what
//! carries them.
//!
//! # The publication seam
//!
//! A device publishes its [`NodeSpec`] into this machine's [`Publications`],
//! keyed by the region it also hands to a `map` statement. That is a seam
//! because `RealizeCtx` does not yet carry the machine graph, so a device and
//! the thing that wants to describe it have no other place to meet. When
//! `RealizeCtx` grows spaces and wires (`ROADMAP.md` §4.4), this table
//! collapses into it and the device code does not change.
//!
//! It is published **from `Device::realize`**, not from a constructor.
//! Announcing yourself into a table a sibling reads is an outward action and
//! `CLAUDE.md`'s two-phase rule puts those in realize — which is the mirror of
//! [`core::hosts`](crate::core::hosts)'s other half, where *acquiring* a port
//! is allocation and stays in `new`. The table itself is one of those host
//! objects, so it belongs to the build: [`table`] opens it from
//! [`RealizeCtx::hosts`](crate::core::RealizeCtx::hosts), and [`BootRom`] holds
//! the same handle as a field so the generator can read it back at reset.
//!
//! [`BootRom`]: super::boot::BootRom
//!
//! Keyed by *region identity*, not by name or instance path: a region is
//! allocated once by its device and lives exactly as long as it, so no two
//! entries in one machine's table can collide.
//!
//! # When the tree is built
//!
//! At **reset**, never at construction or bind. `machine::realize` builds wires
//! *after* it binds devices, so a device does not learn the [`WireId`] its IRQ
//! pin drives until `Device::connect`, which runs later still. Reset is the
//! first moment the whole graph exists, and realize ends with a cold reset, so
//! a freshly built machine has a freshly built tree.

use alloc::collections::BTreeMap;
use alloc::format;
use alloc::string::{String, ToString};
use alloc::sync::{Arc, Weak};
use alloc::vec::Vec;

use crate::core::error::{Error, Result};
use crate::core::hosts::{HostKind, HostObjects};
use crate::core::space::{AddressSpace, RegionKind, RegionRef};
use crate::core::sync::{LockRank, Mutex};
use crate::core::wire::WireId;

use super::fdt::FdtWriter;

/// What sort of node a region becomes, and the fields only that sort has.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum NodeKind {
    /// A CLINT: `interrupts-extended` naming every hart's timer and software
    /// interrupt, and the timebase the whole `/cpus` node is rated in.
    Clint {
        /// The rate `mtime` counts at, in hertz — the machine file's clock
        /// domain for the CLINT, read back rather than restated.
        timebase_hz: u32,
    },
    /// A PLIC: an interrupt controller with a phandle other nodes point at.
    Plic {
        /// How many interrupt sources it implements, for `riscv,ndev`.
        ndev: u32,
    },
    /// A system controller: reset and poweroff live behind it, as two root
    /// nodes that point back at this one.
    Syscon {
        /// The value written to request a poweroff.
        poweroff: u32,
        /// The value written to request a reboot.
        reboot: u32,
    },
    /// Anything else: a `reg`, a `compatible`, and possibly an interrupt.
    Peripheral,
}

/// What one device contributes to the tree.
///
/// The device fills this in; the generator supplies the address, the size and
/// the interrupt number, because those belong to the topology rather than to
/// the device.
#[derive(Debug, Clone)]
pub struct NodeSpec {
    /// Which sort of node, and its kind-specific fields.
    pub kind: NodeKind,
    /// The node's base name. The generator appends `@<address>`.
    pub name: &'static str,
    /// The `compatible` list, most specific first (Devicetree Specification
    /// §2.3.1).
    pub compatible: &'static [&'static str],
    /// Extra cell-valued properties, in the order they should appear.
    pub cells: Vec<(&'static str, Vec<u32>)>,
    /// Extra string-valued properties.
    pub strings: Vec<(&'static str, String)>,
    /// The net this device's interrupt output drives, if it has one and the
    /// machine wired it. Filled in by `Device::connect`.
    pub irq_wire: Option<WireId>,
}

impl NodeSpec {
    /// A peripheral node with no extra properties and no interrupt.
    #[must_use]
    pub fn peripheral(name: &'static str, compatible: &'static [&'static str]) -> NodeSpec {
        NodeSpec {
            kind: NodeKind::Peripheral,
            name,
            compatible,
            cells: Vec::new(),
            strings: Vec::new(),
            irq_wire: None,
        }
    }

    /// The same node with one extra cell-valued property.
    #[must_use]
    pub fn with_cells(mut self, name: &'static str, cells: Vec<u32>) -> NodeSpec {
        self.cells.push((name, cells));
        self
    }
}

/// A device that has something to say about itself in the device tree.
///
/// Implemented by the shared register block a device hands to
/// [`Region::io`](crate::core::space::Region::io), so the table can hold a weak
/// reference and prune itself when the machine goes away.
pub trait DtSource: Send + Sync {
    /// What this device is, as the tree should say it.
    fn dt_spec(&self) -> NodeSpec;

    /// PLIC only: which interrupt source number `wire` lands on.
    ///
    /// This is what turns `wire uart.irq -> plic.irq10` into
    /// `interrupts = <10>` without anybody writing `10` down twice.
    fn dt_plic_source(&self, wire: WireId) -> Option<u32> {
        let _ = wire;
        None
    }
}

/// The hart facts the address space cannot supply.
///
/// See the module docs: a hart is not a region, so these are declared on the
/// `riscv.boot` object instead of derived.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct CpuSpec {
    /// How many harts the machine has.
    pub harts: u32,
    /// The `riscv,isa` string, lower case as the binding wants it.
    pub isa: String,
    /// The `mmu-type` suffix (`sv39`, `sv48`), or empty for a hart with no MMU.
    pub mmu: String,
    /// The hart the firmware is entered on, for `boot_cpuid_phys`.
    pub boot_hart: u32,
}

/// Everything the generator needs that is not in the machine graph.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct TreeConfig {
    /// The `model` property and the second `compatible` entry.
    pub model: String,
    /// `/chosen/bootargs`, the kernel command line.
    pub bootargs: String,
    /// Where a ramdisk was staged, as `(first byte, one past the last)`.
    ///
    /// `None` when nothing staged one, which is every bare-metal run. The two
    /// addresses become `/chosen/linux,initrd-start` and `linux,initrd-end`:
    /// they are not in the Devicetree Specification, they are the convention
    /// Linux has read since it grew a device tree, and a kernel that finds
    /// neither one simply has no ramdisk.
    pub initrd: Option<(u64, u64)>,
    /// The harts.
    pub cpus: CpuSpec,
    /// The timebase to use when no CLINT published one.
    pub default_timebase_hz: u32,
}

// ---------------------------------------------------------------------------
// the publication table
// ---------------------------------------------------------------------------

/// One published region, held weakly so the table prunes itself.
struct Entry {
    /// The key: the identity of the region the device published.
    key: usize,
    /// Kept so a dead entry can be recognised without upgrading the source.
    region: Weak<crate::core::space::Region>,
    source: Weak<dyn DtSource>,
}

/// One machine's region-identity to describer table.
///
/// A host object rather than a `static` (see the module docs): a build opens one
/// through [`table`], every device in that build publishes into it, and its boot
/// ROM reads it back. Two machines in one process therefore describe themselves
/// independently, and a machine that is dropped takes its whole table with it.
#[derive(Default)]
pub struct Publications {
    /// [`LockRank::LEAF`]: nothing is locked while this is held.
    entries: Mutex<Vec<Entry>>,
}

impl core::fmt::Debug for Publications {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        // A `DtSource` has no `Debug` — it is a device's register block — so
        // the count is what a reader can be told.
        match self.entries.try_lock() {
            Some(entries) => f
                .debug_struct("Publications")
                .field("published", &entries.len())
                .finish(),
            None => f
                .debug_struct("Publications")
                .field("published", &"<in use>")
                .finish(),
        }
    }
}

impl Publications {
    /// An empty table.
    #[must_use]
    pub fn new() -> Publications {
        Publications {
            entries: Mutex::with_rank(LockRank::LEAF, Vec::new()),
        }
    }

    /// Publish `source` as the description of `region`.
    ///
    /// Called from a device's `realize`, once per mappable region it has. A
    /// second publication of the same region replaces the first, which is what
    /// a device rebuilt in place wants.
    pub fn publish(&self, region: &RegionRef, source: Weak<dyn DtSource>) {
        let key = key_of(region);
        let mut table = self.entries.lock();
        // Prune while we are here: an entry only ever dies by its device being
        // dropped, and that is exactly when nobody is looking.
        table.retain(|e| e.region.strong_count() > 0 && e.key != key);
        table.push(Entry {
            key,
            region: Arc::downgrade(region),
            source,
        });
    }

    /// The description published for `region`, if it is still live.
    #[must_use]
    pub fn lookup(&self, region: &RegionRef) -> Option<Arc<dyn DtSource>> {
        let key = key_of(region);
        let table = self.entries.lock();
        table
            .iter()
            .find(|e| e.key == key)
            .and_then(|e| e.source.upgrade())
    }

    /// How many live publications there are.
    #[must_use]
    pub fn len(&self) -> usize {
        self.entries.lock().len()
    }

    /// Whether nothing has been published.
    #[must_use]
    pub fn is_empty(&self) -> bool {
        self.entries.lock().is_empty()
    }
}

/// The kind a build's [`Publications`] is filed under.
pub const KIND: HostKind = HostKind::rendezvous("riscv.dt");

/// The name it is filed under: one per machine, so the name is a constant.
const TABLE_NAME: &str = "dt";

/// This build's publication table, creating it on first mention.
///
/// # Errors
///
/// [`Error::Config`] if another kind of host object is already open under this
/// name, which would be a collision between two host modules.
pub fn table(hosts: &HostObjects) -> Result<Arc<Publications>> {
    hosts.open(KIND, TABLE_NAME, Publications::new)
}

/// The same table, for a device that has properties rather than a context.
///
/// The boot ROM's, which needs the handle at construction because it
/// regenerates the tree at reset, where `&self` is all there is.
///
/// # Errors
///
/// As [`table`].
pub fn table_for(props: &crate::core::props::Props) -> Result<Arc<Publications>> {
    props.host(KIND, TABLE_NAME, Publications::new)
}

/// Publish `source` as the description of `region`, in `hosts`'s table.
///
/// The one line a device adds to its `Device::realize`.
///
/// # Errors
///
/// As [`table`].
pub fn publish(hosts: &HostObjects, region: &RegionRef, source: Weak<dyn DtSource>) -> Result<()> {
    table(hosts)?.publish(region, source);
    Ok(())
}

/// The identity of a region, as the table keys on it.
fn key_of(region: &RegionRef) -> usize {
    Arc::as_ptr(region) as *const u8 as usize
}

// ---------------------------------------------------------------------------
// walking the machine
// ---------------------------------------------------------------------------

/// One thing the generator found in the address space.
#[derive(Debug)]
struct Placed {
    base: u64,
    size: u64,
    spec: NodeSpec,
}

/// Follow aliases down to the region that actually answers.
///
/// A machine file may write `map mem 0x10000000 size 0x100 = mirror(uart)`, and
/// the mapping then names the window rather than the device. The device tree
/// wants the device.
fn leaf_of(region: &RegionRef) -> RegionRef {
    let mut here = Arc::clone(region);
    // Bounded because `Region::alias` names an already-built region, so the
    // graph is acyclic by construction; the bound is belt and braces.
    for _ in 0..16 {
        let Some(alias) = here.as_alias() else {
            return here;
        };
        here = Arc::clone(alias.target());
    }
    here
}

/// Everything in `space` that describes itself, plus every RAM region, in
/// address order.
fn survey(dt: &Publications, space: &AddressSpace) -> (Vec<Placed>, Vec<(u64, u64)>) {
    let mut placed = Vec::new();
    let mut memory = Vec::new();
    let view = space.view();
    for (_, mapping) in view.mappings() {
        let leaf = leaf_of(&mapping.region);
        let size = mapping.region.len();
        if let Some(source) = dt.lookup(&leaf) {
            placed.push(Placed {
                base: mapping.base,
                size,
                spec: source.dt_spec(),
            });
        } else if matches!(leaf.kind(), RegionKind::Ram(_)) {
            memory.push((mapping.base, size));
        }
    }
    // Address order, always: the tree is compared byte for byte by the tests
    // and hashed with the machine, and mapping order is an implementation
    // detail of the realizer.
    placed.sort_by_key(|p| p.base);
    memory.sort_by_key(|m| m.0);
    (placed, memory)
}

/// Generate the device tree for the machine `space` belongs to.
///
/// `dt` is the machine's own [`Publications`] — what its devices published
/// about themselves during realize. Passing it in rather than reaching for a
/// `static` is what stops one board describing another's peripherals.
///
/// # Errors
///
/// [`Error::Config`] if the space has no RAM to put a `memory` node on, or if
/// the tree cannot be encoded.
pub fn generate(dt: &Publications, space: &AddressSpace, cfg: &TreeConfig) -> Result<Vec<u8>> {
    let (placed, memory) = survey(dt, space);
    if memory.is_empty() {
        return Err(Error::Config {
            at: "device tree".to_string(),
            message: format!(
                "the address space `{}` has no RAM mapped, so the tree would have no \
                 `memory` node and nothing could be loaded",
                space.name()
            ),
        });
    }

    // Phandles: one per hart interrupt controller, then one for the PLIC, then
    // one for the syscon. Allocated up front because a node has to know the
    // number of a node that has not been written yet.
    let mut next_phandle = 1u32;
    let intc: Vec<u32> = (0..cfg.cpus.harts)
        .map(|_| {
            let p = next_phandle;
            next_phandle += 1;
            p
        })
        .collect();
    let plic_phandle = placed
        .iter()
        .find(|p| matches!(p.spec.kind, NodeKind::Plic { .. }))
        .map(|_| {
            let p = next_phandle;
            next_phandle += 1;
            p
        });
    let syscon_phandle = placed
        .iter()
        .find(|p| matches!(p.spec.kind, NodeKind::Syscon { .. }))
        .map(|_| {
            let p = next_phandle;
            next_phandle += 1;
            p
        });

    // Which PLIC source each device's IRQ pin lands on, taken from the PLIC's
    // own pin table rather than from anything anybody wrote down twice.
    let plic_pins: Option<Arc<dyn DtSource>> = {
        let view = space.view();
        let mut found = None;
        for (_, mapping) in view.mappings() {
            let leaf = leaf_of(&mapping.region);
            if let Some(source) = dt.lookup(&leaf)
                && matches!(source.dt_spec().kind, NodeKind::Plic { .. })
            {
                found = Some(source);
                break;
            }
        }
        found
    };
    let irq_of = |spec: &NodeSpec| -> Option<u32> {
        let wire = spec.irq_wire?;
        plic_pins.as_ref()?.dt_plic_source(wire)
    };

    let timebase = placed
        .iter()
        .find_map(|p| match p.spec.kind {
            NodeKind::Clint { timebase_hz } => Some(timebase_hz),
            _ => None,
        })
        .unwrap_or(cfg.default_timebase_hz);

    // The path of the first serial node, for `/chosen/stdout-path`.
    let stdout = placed
        .iter()
        .find(|p| p.spec.name == "serial")
        .map(|p| format!("/soc/serial@{:x}", p.base));

    let mut w = FdtWriter::new();
    w.set_boot_cpu(cfg.cpus.boot_hart);
    // The firmware and the kernel both relocate the tree, but a client program
    // that does not must at least be told where it is not allowed to write.
    w.begin_node("");
    w.prop_u32("#address-cells", 2);
    w.prop_u32("#size-cells", 2);
    w.prop_str_list("compatible", &["riscv-virtio"]);
    w.prop_str("model", &cfg.model);

    w.begin_node("chosen");
    if !cfg.bootargs.is_empty() {
        w.prop_str("bootargs", &cfg.bootargs);
    }
    if let Some(path) = &stdout {
        w.prop_str("stdout-path", path);
    }
    // Sixty-four bits each rather than one cell: the kernel reads whatever
    // width the property is, and an address that needs more than 32 bits is
    // the ordinary case on a board whose RAM starts at 0x80000000 and can be
    // large. `-end` is one past the last byte, which is what every reader of
    // these two properties has always assumed.
    if let Some((start, end)) = cfg.initrd {
        w.prop_u64("linux,initrd-start", start);
        w.prop_u64("linux,initrd-end", end);
    }
    w.end_node()?;

    // -- the harts ---------------------------------------------------------
    w.begin_node("cpus");
    w.prop_u32("#address-cells", 1);
    w.prop_u32("#size-cells", 0);
    w.prop_u32("timebase-frequency", timebase);
    for hart in 0..cfg.cpus.harts {
        w.begin_node(&format!("cpu@{hart}"));
        w.prop_str("device_type", "cpu");
        w.prop_u32("reg", hart);
        w.prop_str("status", "okay");
        w.prop_str_list("compatible", &["riscv"]);
        // Three spellings, because three generations of kernel read three
        // different ones. `riscv,isa` is deprecated but is what anything older
        // understands; `riscv,isa-base` and `riscv,isa-extensions` are the
        // current binding, and a kernel handed only the old one says
        // "Falling back to deprecated riscv,isa" and works out the extension
        // set by guessing.
        w.prop_str("riscv,isa", &cfg.cpus.isa);
        w.prop_str("riscv,isa-base", base_isa(&cfg.cpus.isa));
        let mut extensions = Vec::new();
        for name in isa_extensions(&cfg.cpus.isa) {
            extensions.extend_from_slice(name.as_bytes());
            extensions.push(0);
        }
        w.prop_bytes("riscv,isa-extensions", &extensions);
        if !cfg.cpus.mmu.is_empty() {
            w.prop_str("mmu-type", &format!("riscv,{}", cfg.cpus.mmu));
        }
        w.begin_node("interrupt-controller");
        w.prop_u32("#interrupt-cells", 1);
        w.prop_empty("interrupt-controller");
        w.prop_str_list("compatible", &["riscv,cpu-intc"]);
        w.prop_u32("phandle", intc[hart as usize]);
        w.end_node()?;
        w.end_node()?;
    }
    w.end_node()?;

    // -- memory ------------------------------------------------------------
    for (base, size) in &memory {
        w.begin_node(&format!("memory@{base:x}"));
        w.prop_str("device_type", "memory");
        w.prop_reg64(&[(*base, *size)]);
        w.end_node()?;
    }

    // -- the peripherals ---------------------------------------------------
    w.begin_node("soc");
    w.prop_u32("#address-cells", 2);
    w.prop_u32("#size-cells", 2);
    w.prop_str_list("compatible", &["simple-bus"]);
    w.prop_empty("ranges");
    for item in &placed {
        w.begin_node(&format!("{}@{:x}", item.spec.name, item.base));
        w.prop_str_list("compatible", item.spec.compatible);
        w.prop_reg64(&[(item.base, item.size)]);
        match item.spec.kind {
            NodeKind::Clint { .. } => {
                // Two cells per hart: the controller's phandle and the cause
                // number of the interrupt it drives — 3 for machine software,
                // 7 for machine timer (Privileged Architecture, table of
                // interrupt causes).
                let mut cells = Vec::with_capacity(cfg.cpus.harts as usize * 4);
                for phandle in &intc {
                    cells.extend_from_slice(&[*phandle, 3, *phandle, 7]);
                }
                w.prop_cells("interrupts-extended", &cells);
            }
            NodeKind::Plic { ndev } => {
                // Machine and supervisor external interrupts: causes 11 and 9.
                let mut cells = Vec::with_capacity(cfg.cpus.harts as usize * 4);
                for phandle in &intc {
                    cells.extend_from_slice(&[*phandle, 11, *phandle, 9]);
                }
                w.prop_cells("interrupts-extended", &cells);
                w.prop_empty("interrupt-controller");
                w.prop_u32("#interrupt-cells", 1);
                w.prop_u32("#address-cells", 0);
                w.prop_u32("riscv,ndev", ndev);
                if let Some(p) = plic_phandle {
                    w.prop_u32("phandle", p);
                }
            }
            NodeKind::Syscon { .. } => {
                if let Some(p) = syscon_phandle {
                    w.prop_u32("phandle", p);
                }
            }
            NodeKind::Peripheral => {}
        }
        if let (Some(irq), Some(parent)) = (irq_of(&item.spec), plic_phandle) {
            w.prop_u32("interrupt-parent", parent);
            w.prop_u32("interrupts", irq);
        }
        for (name, cells) in &item.spec.cells {
            w.prop_cells(name, cells);
        }
        for (name, value) in &item.spec.strings {
            w.prop_str(name, value);
        }
        w.end_node()?;
    }
    w.end_node()?;

    // -- poweroff and reboot, which point back at the syscon ---------------
    if let (Some(phandle), Some(syscon)) = (
        syscon_phandle,
        placed.iter().find_map(|p| match p.spec.kind {
            NodeKind::Syscon { poweroff, reboot } => Some((poweroff, reboot)),
            _ => None,
        }),
    ) {
        let (poweroff, reboot) = syscon;
        // One `compatible` each, not both on both: a firmware that matches on
        // the first string it recognises would otherwise bind the same driver
        // to the two nodes and end up with a machine that can reboot but not
        // switch off. OpenSBI reports exactly that ("Platform Shutdown Device:
        // ---"), which is how this was found.
        for (node, value, compatible) in [
            ("poweroff", poweroff, "syscon-poweroff"),
            ("reboot", reboot, "syscon-reboot"),
        ] {
            w.begin_node(node);
            w.prop_u32("value", value);
            w.prop_u32("offset", 0);
            w.prop_u32("regmap", phandle);
            w.prop_str_list("compatible", &[compatible]);
            w.end_node()?;
        }
    }

    w.end_node()?;
    w.finish()
}

/// The `riscv,isa-base` value for an ISA string: the width and `i`, nothing
/// else.
fn base_isa(isa: &str) -> &'static str {
    if isa.starts_with("rv32") {
        "rv32i"
    } else {
        "rv64i"
    }
}

/// The current binding's `riscv,isa-extensions` list, derived from the ISA
/// string the machine file declared.
///
/// Each single-letter extension becomes its own entry, and the two the
/// interpreter always implements are appended: `zicsr` (the CSR instructions,
/// without which nothing here could be programmed) and `zifencei`. `zicntr` is
/// there because `cycle`, `time` and `instret` are all readable — the counters
/// exist, whatever they count.
fn isa_extensions(isa: &str) -> Vec<String> {
    let letters = isa
        .strip_prefix("rv64")
        .or_else(|| isa.strip_prefix("rv32"))
        .unwrap_or(isa);
    let mut out: Vec<String> = Vec::new();
    for letter in letters.chars().filter(char::is_ascii_alphabetic) {
        let name = letter.to_ascii_lowercase().to_string();
        if !out.contains(&name) {
            out.push(name);
        }
    }
    for always in ["zicsr", "zifencei", "zicntr"] {
        out.push(always.to_string());
    }
    out
}

/// A human-readable rendering of a generated tree, for tests and for
/// `rsemu describe`.
///
/// Not a full `dtc -O dts` — it prints node names and property names with their
/// raw lengths, which is enough to assert that a node exists and to eyeball a
/// tree without a toolchain.
///
/// # Errors
///
/// [`Error::State`] if `dtb` is not a device tree this writer could have
/// produced.
pub fn describe(dtb: &[u8]) -> Result<String> {
    let word = |at: usize| -> Result<u32> {
        dtb.get(at..at + 4)
            .map(|b| u32::from_be_bytes([b[0], b[1], b[2], b[3]]))
            .ok_or_else(|| Error::State("device tree is truncated".to_string()))
    };
    if word(0)? != super::fdt::FDT_MAGIC {
        return Err(Error::State("not a flattened device tree".to_string()));
    }
    let off_struct = word(8)? as usize;
    let len_struct = word(36)? as usize;
    let off_strings = word(12)? as usize;

    let name_at = |at: usize| -> String {
        let end = dtb[at..].iter().position(|b| *b == 0).unwrap_or(0) + at;
        String::from_utf8_lossy(&dtb[at..end]).into_owned()
    };

    let mut out = String::new();
    let mut at = off_struct;
    let end = off_struct + len_struct;
    let mut depth = 0usize;
    let mut counts: BTreeMap<String, usize> = BTreeMap::new();
    while at + 4 <= end {
        let token = word(at)?;
        at += 4;
        match token {
            1 => {
                let name = name_at(at);
                at += name.len() + 1;
                at = at.next_multiple_of(4);
                for _ in 0..depth {
                    out.push_str("  ");
                }
                out.push_str(if name.is_empty() { "/" } else { &name });
                out.push_str(" {\n");
                depth += 1;
                *counts.entry(name).or_default() += 1;
            }
            2 => {
                depth = depth.saturating_sub(1);
                for _ in 0..depth {
                    out.push_str("  ");
                }
                out.push_str("};\n");
            }
            3 => {
                let len = word(at)? as usize;
                let name_off = word(at + 4)? as usize;
                at += 8;
                let name = name_at(off_strings + name_off);
                for _ in 0..=depth {
                    out.push_str("  ");
                }
                out.push_str(&format!("{name} [{len}]\n"));
                at += len;
                at = at.next_multiple_of(4);
            }
            4 => {}
            9 => break,
            other => {
                return Err(Error::State(format!("unknown device tree token {other}")));
            }
        }
    }
    Ok(out)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::space::{MemOps, RamStore, Region};

    /// A describer with nothing behind it, for the table's own tests.
    #[derive(Debug)]
    struct Fake;

    impl DtSource for Fake {
        fn dt_spec(&self) -> NodeSpec {
            NodeSpec::peripheral("fake", &["rsemu,fake"])
        }
    }

    impl MemOps for Fake {
        fn read(
            &self,
            _offset: u64,
            _dst: &mut [u8],
            _attrs: crate::core::space::MemAttrs,
        ) -> crate::core::space::MemResult {
            Ok(())
        }
        fn write(
            &self,
            _offset: u64,
            _src: &[u8],
            _attrs: crate::core::space::MemAttrs,
        ) -> crate::core::space::MemResult {
            Ok(())
        }
        fn constraints(&self) -> crate::core::space::AccessConstraints {
            crate::core::space::AccessConstraints::IO
        }
    }

    fn published(dt: &Publications) -> (RegionRef, Arc<Fake>) {
        let ops = Arc::new(Fake);
        let region: RegionRef = Arc::new(Region::io(
            "fake",
            0x100,
            Arc::clone(&ops) as Arc<dyn MemOps>,
        ));
        dt.publish(&region, Arc::downgrade(&ops) as Weak<dyn DtSource>);
        (region, ops)
    }

    #[test]
    fn a_published_region_finds_its_describer_again() {
        let dt = Publications::new();
        let (region, _ops) = published(&dt);
        let found = dt.lookup(&region).expect("published");
        assert_eq!(found.dt_spec().name, "fake");

        // A region nobody published is not in the table, and asking is not an
        // error — most regions in a machine are ordinary memory.
        let other: RegionRef = Arc::new(Region::ram("ram", Arc::new(RamStore::new(0x100))));
        assert!(dt.lookup(&other).is_none());

        // And another machine's table knows nothing about this region, which is
        // what stops one board describing another's peripherals.
        assert!(Publications::new().lookup(&region).is_none());
    }

    #[test]
    fn an_entry_dies_with_its_device() {
        let dt = Publications::new();
        let key = {
            let (region, _ops) = published(&dt);
            let key = key_of(&region);
            assert!(dt.lookup(&region).is_some());
            key
        };
        // Publishing anything prunes, which is when the dead entry goes.
        let (_region, _ops) = published(&dt);
        assert!(
            !dt.entries.lock().iter().any(|e| e.key == key),
            "a dropped device leaves nothing behind"
        );
        assert_eq!(dt.len(), 1, "and only the live publication is kept");
    }

    #[test]
    fn a_builds_table_is_opened_once_and_shared() {
        let hosts = crate::core::HostObjects::new();
        let a = table(&hosts).expect("a fresh table");
        let b = table(&hosts).expect("the same one");
        assert!(Arc::ptr_eq(&a, &b));
        assert!(a.is_empty());

        let elsewhere = crate::core::HostObjects::new();
        assert!(!Arc::ptr_eq(
            &a,
            &table(&elsewhere).expect("another build's")
        ));
    }

    /// The full generator is exercised against a real machine in
    /// [`super::super::tests`]; this only pins the failure a caller is most
    /// likely to hit.
    #[test]
    fn a_space_with_no_ram_says_so_rather_than_emitting_a_useless_tree() {
        let space = AddressSpace::new("mem", 64);
        let cfg = TreeConfig {
            model: "test".to_string(),
            bootargs: String::new(),
            initrd: None,
            cpus: CpuSpec {
                harts: 1,
                isa: "rv64imac".to_string(),
                mmu: "sv39".to_string(),
                boot_hart: 0,
            },
            default_timebase_hz: 10_000_000,
        };
        let e = generate(&Publications::new(), &space, &cfg)
            .expect_err("no memory")
            .to_string();
        assert!(e.contains("no RAM"), "{e}");
    }
}
