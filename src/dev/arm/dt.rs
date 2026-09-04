//! Generating the device tree from the realized machine graph.
//!
//! `docs/platforms/riscv-virt.md` set the rule and this board keeps it:
//!
//! > rsemu **generates** the device tree from the realized machine graph and
//! > passes it to firmware. That is a genuine test of the machine model: if
//! > the DTB can be produced mechanically from the topology, the topology is
//! > well-formed.
//!
//! So nothing here writes an address, a size or an interrupt number down.
//! Every one of them is read back out of the machine that was actually built:
//!
//! | What | Where it comes from |
//! | --- | --- |
//! | a node's `reg` | the base and length of the mapping the `map` statement made |
//! | which node a mapping is | the [`NodeSpec`] the device published for that region |
//! | the GIC's two `reg` entries | its two mappings, sorted, joined into one node |
//! | `interrupts` | the net a device's IRQ pin drives, looked up in the GIC's own pin table |
//! | `memory@…` | every RAM region mapped in the space |
//!
//! # What is declared rather than derived, and why
//!
//! Three things, and each is a fact about something that is **not a region in
//! an address space**, which is the only thing this generator can see:
//!
//! * **The processors.** A core is not a region and there is no route from a
//!   `dyn Device` to a `Cpu` (`core::device` keeps `Any` out of the supertrait
//!   chain deliberately), so the count and the `MPIDR_EL1` affinity values are
//!   declared on the `arm.boot` object. [`CpuSpec`] carries them.
//! * **The PSCI conduit.** `HVC` and `SMC` are instructions, not addresses.
//!   Whether this board answers them, and which one a guest should use, is a
//!   property of the core and the board together.
//! * **The generic timer's four interrupt ids.** The timer is *inside* the
//!   core, so the wire that carries it out to the GIC starts somewhere this
//!   generator cannot see. The numbers are declared and are the conventional
//!   ones; `machines/arm64-virt.machine` writes the same numbers in its
//!   `wire` statements, which is a duplication and is called out there.
//!
//! The RISC-V generator has exactly the same three-line limitation for exactly
//! the same reason, and both collapse the day `RealizeCtx` carries the machine
//! graph (`ROADMAP.md` §4.4).
//!
//! # The publication seam
//!
//! A device publishes its [`NodeSpec`] into this machine's [`Publications`],
//! keyed by the region it also hands to a `map` statement. It is published
//! **from `Device::realize`**, not from a constructor: announcing yourself
//! into a table others read is an outward action, where *acquiring* a host
//! object is allocation and stays in `new`
//! ([`core::hosts`](crate::core::hosts)).
//!
//! Keyed by *region identity*, not by name or instance path: a region is
//! allocated once by its device and lives exactly as long as it, so no two
//! entries in one machine's table can collide.
//!
//! # When the tree is built
//!
//! At **reset**, never at construction or bind. `machine::realize` builds
//! wires *after* it binds devices, so a device does not learn the [`WireId`]
//! its IRQ pin drives until `Device::connect`, which runs later still. Reset
//! is the first moment the whole graph exists, and realize ends with a cold
//! reset, so a freshly built machine has a freshly built tree.
//!
//! # Sources
//!
//! The *Devicetree Specification* v0.4 for the format and for the standard
//! properties (`reg`, `compatible`, `device_type`, `#address-cells`); the
//! *ARM Generic Interrupt Controller Architecture Specification v2.0* for what
//! a three-cell interrupt specifier means; and the *Arm Power State
//! Coordination Interface* (DEN 0022) for the `psci` node's contents. The
//! `enable-method = "psci"` spelling and the `arm,psci-0.2` / `arm,psci-1.0`
//! `compatible` strings were checked against Trusted Firmware-A's own device
//! trees (BSD-3-Clause) and against the ARM boot-wrapper (BSD-3-Clause), which
//! are permissive and are where they can be read without opening a copyleft
//! tree.

use alloc::format;
use alloc::string::{String, ToString};
use alloc::sync::{Arc, Weak};
use alloc::vec::Vec;

use crate::core::error::{Error, Result};
use crate::core::hosts::{HostKind, HostObjects};
use crate::core::space::{AddressSpace, RegionKind, RegionRef};
use crate::core::sync::{LockRank, Mutex};
use crate::core::wire::WireId;

use crate::dev::fdt::FdtWriter;

/// What sort of node a region becomes, and the fields only that sort has.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum NodeKind {
    /// The interrupt controller: a `phandle` other nodes point at, and two
    /// `reg` entries joined from two mappings.
    Gic,
    /// A PL011, which is an AMBA peripheral and therefore needs a clock node
    /// to point at before its driver will probe.
    Pl011,
    /// Anything else: a `reg`, a `compatible`, and possibly an interrupt.
    Peripheral,
}

/// An interrupt as a GIC device tree specifier: which of the three kinds, and
/// the number within that kind.
///
/// Not the architectural interrupt id: the binding numbers a shared
/// peripheral interrupt from 32 and a private one from 16, and *subtracts the
/// base again* — so architectural id 33 is `<0 1 …>` and id 27 is `<1 11 …>`.
/// Getting that wrong gives a driver that requests the wrong line and a device
/// whose interrupt never arrives.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct IntSpec {
    /// `0` for a shared peripheral interrupt, `1` for a private one — the
    /// first of the three cells.
    pub kind: u32,
    /// The number within that kind — the second cell.
    pub number: u32,
}

impl IntSpec {
    /// `IRQ_TYPE_LEVEL_HIGH`, the trigger every device on this board uses.
    pub const LEVEL_HIGH: u32 = 4;

    /// `IRQ_TYPE_LEVEL_LOW`, which is how the generic timer's private
    /// interrupts are conventionally described.
    pub const LEVEL_LOW: u32 = 8;

    /// The CPU mask a private interrupt carries in bits 15:8 on a GICv2.
    pub const PPI_CPU_MASK: u32 = 0xff00;

    /// A shared peripheral interrupt, level-triggered and active high.
    #[must_use]
    pub const fn spi(number: u32) -> IntSpec {
        IntSpec { kind: 0, number }
    }

    /// A private peripheral interrupt.
    #[must_use]
    pub const fn ppi(number: u32) -> IntSpec {
        IntSpec { kind: 1, number }
    }

    /// The three cells, as `interrupts` wants them.
    #[must_use]
    pub fn cells(&self) -> [u32; 3] {
        let flags = if self.kind == 1 {
            IntSpec::PPI_CPU_MASK | IntSpec::LEVEL_LOW
        } else {
            IntSpec::LEVEL_HIGH
        };
        [self.kind, self.number, flags]
    }
}

/// What one device contributes to the tree.
///
/// The device fills this in; the generator supplies the address, the size and
/// the interrupt specifier, because those belong to the topology rather than
/// to the device.
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
/// [`Region::io`](crate::core::space::Region::io), so the table can hold a
/// weak reference and prune itself when the machine goes away.
pub trait DtSource: Send + Sync {
    /// What this device is, as the tree should say it.
    fn dt_spec(&self) -> NodeSpec;

    /// Interrupt controller only: which interrupt `wire` lands on.
    ///
    /// This is what turns `wire uart.irq -> gic.spi1` into
    /// `interrupts = <0 1 4>` without anybody writing `1` down twice.
    fn dt_interrupt(&self, wire: WireId) -> Option<IntSpec> {
        let _ = wire;
        None
    }
}

/// The processor facts the address space cannot supply.
///
/// See the module docs: a core is not a region, so these are declared on the
/// `arm.boot` object instead of derived.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct CpuSpec {
    /// Each processor's `MPIDR_EL1` affinity, in `/cpus` order. The `reg`
    /// property of `cpu@N` is `Aff2:Aff1:Aff0`, which is the low 24 bits.
    pub mpidr: Vec<u64>,
    /// The `compatible` string each `cpu@N` node carries.
    pub compatible: String,
    /// The `enable-method`, or empty for a board that has none: `psci` or
    /// `spin-table`.
    pub enable_method: String,
    /// The spin table's address, when [`enable_method`](CpuSpec::enable_method)
    /// is `spin-table`.
    ///
    /// One 64-bit word per processor, starting here, and processor `i`'s
    /// `cpu-release-addr` is `release_addr + 8i` (Devicetree Specification
    /// v0.4 §3.8.1: "the physical address of a spin table entry that releases
    /// a secondary CPU from its spin loop"). The generator also reserves the
    /// page it lands in, because a table the kernel's own allocator can hand
    /// out is a table that gets overwritten before it is read.
    pub release_addr: Option<u64>,
    /// Where processors other than the first are waiting, whatever the tree
    /// says about how to start them.
    ///
    /// The same table as [`release_addr`](CpuSpec::release_addr) and a
    /// different question: a `psci` board still parks its secondaries on one,
    /// and still has to keep the kernel's allocator off it, even though the
    /// tree does not tell the guest where it is.
    pub parked_at: Option<u64>,
}

impl CpuSpec {
    /// The `reg` value for processor `index`: the affinity bits, which is what
    /// a kernel matches against the `MPIDR_EL1` it reads.
    #[must_use]
    pub fn reg(&self, index: usize) -> u32 {
        (self.mpidr.get(index).copied().unwrap_or(0) & 0x00ff_ffff) as u32
    }
}

/// How a guest calls firmware, if this board offers a firmware interface.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Conduit {
    /// `SMC`, which is what a kernel running at EL1 below a monitor uses.
    Smc,
    /// `HVC`, which is what a kernel running at EL1 below a hypervisor uses.
    Hvc,
}

impl Conduit {
    /// The `method` property's value.
    #[must_use]
    pub const fn as_str(self) -> &'static str {
        match self {
            Conduit::Smc => "smc",
            Conduit::Hvc => "hvc",
        }
    }
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
    /// addresses become `/chosen/linux,initrd-start` and `linux,initrd-end`.
    pub initrd: Option<(u64, u64)>,
    /// The processors.
    pub cpus: CpuSpec,
    /// The firmware conduit, if this board answers one.
    pub psci: Option<Conduit>,
    /// The generic timer's four private interrupt numbers, in the binding's
    /// order: secure physical, non-secure physical, virtual, hypervisor.
    ///
    /// Empty for a board that does not describe a timer at all.
    pub timer_ppi: Vec<u32>,
    /// What `UARTCLK` and the AMBA `apb_pclk` are rated at, in hertz.
    pub apb_clock_hz: u32,
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
/// A host object rather than a `static`: a build opens one through [`table`],
/// every device in that build publishes into it, and its boot ROM reads it
/// back. Two machines in one process therefore describe themselves
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
pub const KIND: HostKind = HostKind::rendezvous("arm.dt");

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

/// The interrupt controller's describer, if the machine has one.
fn controller(dt: &Publications, space: &AddressSpace) -> Option<Arc<dyn DtSource>> {
    let view = space.view();
    for (_, mapping) in view.mappings() {
        let leaf = leaf_of(&mapping.region);
        if let Some(source) = dt.lookup(&leaf)
            && source.dt_spec().kind == NodeKind::Gic
        {
            return Some(source);
        }
    }
    None
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

    // Phandles, allocated up front because a node has to name a node that has
    // not been written yet.
    let mut next_phandle = 1u32;
    let gic_phandle = placed
        .iter()
        .any(|p| p.spec.kind == NodeKind::Gic)
        .then(|| {
            let p = next_phandle;
            next_phandle += 1;
            p
        });
    let clock_phandle = placed
        .iter()
        .any(|p| p.spec.kind == NodeKind::Pl011)
        .then(|| {
            let p = next_phandle;
            next_phandle += 1;
            p
        });

    // Which interrupt each device's pin lands on, taken from the controller's
    // own pin table rather than from anything anybody wrote down twice.
    let pins = controller(dt, space);
    let irq_of = |spec: &NodeSpec| -> Option<IntSpec> {
        let wire = spec.irq_wire?;
        pins.as_ref()?.dt_interrupt(wire)
    };

    // The GIC's two apertures are one node with two `reg` entries, so they are
    // taken out of the peripheral list and handled on their own.
    let gic_regs: Vec<(u64, u64)> = placed
        .iter()
        .filter(|p| p.spec.kind == NodeKind::Gic)
        .map(|p| (p.base, p.size))
        .collect();
    let gic_spec = placed
        .iter()
        .find(|p| p.spec.kind == NodeKind::Gic)
        .map(|p| p.spec.clone());

    // The path of the first console, for `/chosen/stdout-path`.
    let stdout = placed
        .iter()
        .find(|p| p.spec.kind == NodeKind::Pl011)
        .map(|p| format!("/{}@{:x}", p.spec.name, p.base));

    let mut w = FdtWriter::new();
    w.set_boot_cpu(cfg.cpus.reg(0));
    // The release table, before anything else is written: it is memory the
    // guest must not allocate, because the processors waiting on it read it
    // long after the kernel has taken the rest of DRAM. A whole page, since
    // that is the granularity a guest reserves anything at.
    if let Some(base) = cfg.cpus.parked_at {
        let page = base & !0xfff;
        w.reserve(page, 0x1000);
    }
    w.begin_node("");
    w.prop_u32("#address-cells", 2);
    w.prop_u32("#size-cells", 2);
    w.prop_str_list("compatible", &["rsemu,arm64-virt"]);
    w.prop_str("model", &cfg.model);
    if let Some(phandle) = gic_phandle {
        // At the root, so every node below inherits it and only a node with a
        // different controller has to say so.
        w.prop_u32("interrupt-parent", phandle);
    }

    w.begin_node("chosen");
    if !cfg.bootargs.is_empty() {
        w.prop_str("bootargs", &cfg.bootargs);
    }
    if let Some(path) = &stdout {
        w.prop_str("stdout-path", path);
    }
    if let Some((start, end)) = cfg.initrd {
        // Sixty-four bits each rather than one cell: an address above 4 GiB is
        // the ordinary case on a board with a lot of DRAM, and `-end` is one
        // past the last byte, which is what every reader of these two
        // properties has always assumed.
        w.prop_u64("linux,initrd-start", start);
        w.prop_u64("linux,initrd-end", end);
    }
    w.end_node()?;

    // -- the processors -----------------------------------------------------
    w.begin_node("cpus");
    // One cell: `MPIDR_EL1` affinity 2:1:0 fits in 24 bits, and a board that
    // needed Aff3 would be a board with more than 16 million cores.
    w.prop_u32("#address-cells", 1);
    w.prop_u32("#size-cells", 0);
    for index in 0..cfg.cpus.mpidr.len() {
        let reg = cfg.cpus.reg(index);
        w.begin_node(&format!("cpu@{reg:x}"));
        w.prop_str("device_type", "cpu");
        w.prop_str_list("compatible", &[cfg.cpus.compatible.as_str()]);
        w.prop_u32("reg", reg);
        if !cfg.cpus.enable_method.is_empty() {
            w.prop_str("enable-method", &cfg.cpus.enable_method);
        }
        // One word per processor, and the boot processor gets one too: it is
        // not waiting on it, but a binding that is per-node reads better with
        // no hole in it, and a kernel that looks at cpu@0's is told an address
        // rather than nothing.
        if let Some(base) = cfg.cpus.release_addr {
            w.prop_u64("cpu-release-addr", base + 8 * index as u64);
        }
        w.end_node()?;
    }
    w.end_node()?;

    // -- memory -------------------------------------------------------------
    for (base, size) in &memory {
        w.begin_node(&format!("memory@{base:x}"));
        w.prop_str("device_type", "memory");
        w.prop_reg64(&[(*base, *size)]);
        w.end_node()?;
    }

    // -- the interrupt controller ------------------------------------------
    if let (Some(spec), Some(phandle)) = (&gic_spec, gic_phandle) {
        let base = gic_regs.first().map_or(0, |r| r.0);
        w.begin_node(&format!("{}@{base:x}", spec.name));
        w.prop_str_list("compatible", spec.compatible);
        // Three cells: kind, number, flags (GIC architecture specification and
        // the interrupt-controller binding).
        w.prop_u32("#interrupt-cells", 3);
        w.prop_empty("interrupt-controller");
        w.prop_u32("#address-cells", 0);
        // Both apertures, in address order: the distributor then the CPU
        // interface, which is what the binding's `reg` order means.
        w.prop_reg64(&gic_regs);
        w.prop_u32("phandle", phandle);
        w.end_node()?;
    }

    // -- the generic timer --------------------------------------------------
    //
    // A node with no `reg`: the timer is inside the core and has no address.
    // What it has is four private interrupts, and this board wires two of them
    // (see the module docs).
    if !cfg.timer_ppi.is_empty() {
        w.begin_node("timer");
        w.prop_str_list("compatible", &["arm,armv8-timer"]);
        let mut cells = Vec::with_capacity(cfg.timer_ppi.len() * 3);
        for ppi in &cfg.timer_ppi {
            cells.extend_from_slice(&IntSpec::ppi(*ppi).cells());
        }
        w.prop_cells("interrupts", &cells);
        w.end_node()?;
    }

    // -- the firmware interface --------------------------------------------
    if let Some(conduit) = cfg.psci {
        w.begin_node("psci");
        // Both strings, most specific first: a reader that only knows 0.2
        // matches the second and gets the subset it understands.
        w.prop_str_list("compatible", &["arm,psci-1.0", "arm,psci-0.2"]);
        w.prop_str("method", conduit.as_str());
        w.end_node()?;
    }

    // -- the AMBA reference clock ------------------------------------------
    //
    // A PL011 is an AMBA peripheral and its driver enables two clocks before
    // it reads a single register. A fixed-clock node is the smallest thing
    // that can be enabled.
    if let Some(phandle) = clock_phandle {
        w.begin_node("apb-pclk");
        w.prop_str_list("compatible", &["fixed-clock"]);
        w.prop_u32("#clock-cells", 0);
        w.prop_u32("clock-frequency", cfg.apb_clock_hz);
        w.prop_str("clock-output-names", "apb_pclk");
        w.prop_u32("phandle", phandle);
        w.end_node()?;
    }

    // -- the peripherals ----------------------------------------------------
    //
    // At the root rather than under a `soc` node, so that the addresses in
    // every `reg` are the physical ones the `map` statements made and no
    // `ranges` translation stands between them and the guest.
    for item in &placed {
        if item.spec.kind == NodeKind::Gic {
            continue;
        }
        w.begin_node(&format!("{}@{:x}", item.spec.name, item.base));
        w.prop_str_list("compatible", item.spec.compatible);
        w.prop_reg64(&[(item.base, item.size)]);
        if let Some(irq) = irq_of(&item.spec) {
            w.prop_cells("interrupts", &irq.cells());
        }
        if item.spec.kind == NodeKind::Pl011
            && let Some(phandle) = clock_phandle
        {
            // The same clock twice, which is what the binding asks for: the
            // UART's own reference and the bus clock its registers are on.
            w.prop_cells("clocks", &[phandle, phandle]);
            w.prop_str_list("clock-names", &["uartclk", "apb_pclk"]);
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
    w.finish()
}

/// A human-readable rendering of a generated tree, for tests and for
/// `rsemu describe`.
///
/// Not a full `dtc -O dts` — it prints node names and property names with
/// their raw lengths, which is enough to assert that a node exists and to
/// eyeball a tree without a toolchain.
///
/// # Errors
///
/// [`Error::State`] if `dtb` is not a device tree this writer could have
/// produced.
pub fn describe(dtb: &[u8]) -> Result<String> {
    let word = |at: usize| -> Result<u32> {
        dtb.get(at..at + 4)
            .map(|b| u32::from_be_bytes([b[0], b[1], b[2], b[3]]))
            .ok_or_else(|| Error::State(format!("device tree is truncated at {at:#x}")))
    };
    if word(0)? != crate::dev::fdt::FDT_MAGIC {
        return Err(Error::State("not a flattened device tree".to_string()));
    }
    // Byte offsets into the header, not word indices: the ten fields sit at
    // 0, 4, 8 ... and `word` takes an offset.
    let struct_off = word(8)? as usize;
    let strings_off = word(12)? as usize;
    let struct_len = word(36)? as usize;

    let name_at = |off: usize| -> String {
        let start = strings_off + off;
        let end = dtb[start..]
            .iter()
            .position(|b| *b == 0)
            .map_or(dtb.len(), |n| start + n);
        String::from_utf8_lossy(&dtb[start..end]).into_owned()
    };

    let mut out = String::new();
    let mut at = struct_off;
    let end = struct_off + struct_len;
    let mut depth = 0usize;
    while at < end {
        match word(at)? {
            1 => {
                let start = at + 4;
                let stop = dtb[start..]
                    .iter()
                    .position(|b| *b == 0)
                    .map_or(dtb.len(), |n| start + n);
                let name = String::from_utf8_lossy(&dtb[start..stop]);
                out.push_str(&format!(
                    "{:indent$}{} {{\n",
                    "",
                    if name.is_empty() { "/" } else { &name },
                    indent = depth * 2
                ));
                depth += 1;
                at = (stop + 1).next_multiple_of(4);
            }
            2 => {
                depth = depth.saturating_sub(1);
                out.push_str(&format!("{:indent$}}}\n", "", indent = depth * 2));
                at += 4;
            }
            3 => {
                let len = word(at + 4)? as usize;
                let name = name_at(word(at + 8)? as usize);
                out.push_str(&format!(
                    "{:indent$}{name} ({len} byte(s))\n",
                    "",
                    indent = depth * 2
                ));
                at += 12 + len.next_multiple_of(4);
            }
            9 => break,
            other => {
                return Err(Error::State(format!(
                    "device tree has token {other:#x} at offset {at:#x}"
                )));
            }
        }
    }
    Ok(out)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn an_interrupt_specifier_carries_the_bases_the_binding_subtracts() {
        // Architectural id 33 is SPI 1, and 27 is PPI 11. A generator that
        // wrote the architectural id would give a driver the wrong line.
        assert_eq!(IntSpec::spi(1).cells(), [0, 1, IntSpec::LEVEL_HIGH]);
        assert_eq!(
            IntSpec::ppi(11).cells(),
            [1, 11, IntSpec::PPI_CPU_MASK | IntSpec::LEVEL_LOW]
        );
    }

    #[test]
    fn a_processors_reg_is_its_affinity_and_not_its_whole_mpidr() {
        // Bit 31 of `MPIDR_EL1` is RES1 and is not part of the affinity; a
        // `reg` that carried it would match no processor the kernel looks for.
        let cpus = CpuSpec {
            mpidr: alloc::vec![0x8000_0000, 0x8000_0001],
            compatible: String::from("arm,armv8"),
            enable_method: String::from("psci"),
            release_addr: None,
            parked_at: None,
        };
        assert_eq!(cpus.reg(0), 0);
        assert_eq!(cpus.reg(1), 1);
    }

    #[test]
    fn a_conduit_names_itself_the_way_the_binding_spells_it() {
        assert_eq!(Conduit::Smc.as_str(), "smc");
        assert_eq!(Conduit::Hvc.as_str(), "hvc");
    }
}
