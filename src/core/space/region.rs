//! The region tree: what a machine description describes and what a human
//! reasons about (`ROADMAP.md` §4.1).
//!
//! A region is immutable once built, with exactly one exception: an
//! [`Alias`]'s offset lives in an atomic cell so that bank switching is a
//! *rebase* rather than a *retopology*. Everything else — the set of regions,
//! their sizes, their priorities — changes only by building a new node, which
//! is what makes "the region set is identical" a checkable property rather
//! than a promise.
//!
//! Cycles are impossible by construction: an alias names an already-built
//! `Arc<Region>`, and there is no way to mutate a node's children afterwards.

use super::attrs::{AccessConstraints, MemOps, Perms};
use super::store::{RamStore, RomStore};
use crate::core::error::Error;
use crate::core::value::Endian;
use alloc::string::{String, ToString};
use alloc::sync::Arc;
use alloc::vec::Vec;
use core::sync::atomic::{AtomicU64, Ordering};

/// A shared handle to a region.
///
/// Regions are shared rather than owned because an alias, a second address
/// space, and a snapshot walker all need to name the same node.
pub type RegionRef = Arc<Region>;

/// Identifies one [`Alias`] for the purposes of rebasing.
///
/// Allocated from a process-wide counter. Only ever compared and used as a map
/// key; the value has no meaning.
#[repr(transparent)]
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct AliasId(pub u64);

static NEXT_ALIAS_ID: AtomicU64 = AtomicU64::new(1);

/// Identifies one mapping in an [`AddressSpace`](super::AddressSpace)'s root
/// container, so it can later be moved or removed.
#[repr(transparent)]
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct MappingId(pub u64);

/// What happens to a write that lands on a [`Region::rom`] region.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
pub enum RomWrite {
    /// Swallow it. What a mask ROM does, and what a cartridge without a mapper
    /// register does.
    #[default]
    Ignore,
    /// Raise [`BusError::BadAccess`](crate::core::BusError::BadAccess), for a
    /// bus that reports a write to read-only memory.
    Fault,
}

/// How a container resolves an address covered by more than one child.
///
/// `Priority` is the deterministic default and the right model for PCI. The
/// wired variants exist because an open-bus system genuinely combines: the NES
/// `memory.Bus` in the sibling `gones` project OR-combines every handler
/// mapped at an address, and that behaviour is correct *there* and wrong as a
/// default (`ROADMAP.md` §4.1).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
pub enum CombinePolicy {
    /// Highest priority wins; ties are broken by mapping order, later wins.
    /// Only one child is ever consulted.
    #[default]
    Priority,
    /// Every covering child is read and the results are OR-ed; writes go to
    /// all of them.
    WiredOr,
    /// Every covering child is read and the results are AND-ed; writes go to
    /// all of them.
    WiredAnd,
    /// Like [`CombinePolicy::WiredOr`], but counts the overlaps so a machine
    /// can report a bus conflict instead of hiding it.
    Conflict,
}

/// The alias node: a window onto another region, with a slidable offset.
///
/// The offset is an [`AtomicU64`] because sliding it must be cheap. An MMC3
/// cartridge rebanks ~15 000 times a second; if that rebuilt a flat view and
/// invalidated every translation block the NES would be a slideshow
/// (`ROADMAP.md` §4.1).
#[derive(Debug)]
pub struct Alias {
    target: RegionRef,
    offset: Arc<AtomicU64>,
    id: AliasId,
    rebasable: bool,
    repeat: bool,
}

impl Alias {
    /// The region this window looks at.
    #[must_use]
    pub fn target(&self) -> &RegionRef {
        &self.target
    }

    /// The current offset into [`Alias::target`].
    #[inline]
    #[must_use]
    pub fn offset(&self) -> u64 {
        self.offset.load(Ordering::Relaxed)
    }

    /// This alias's identity, for
    /// [`AddressSpace::rebase`](super::AddressSpace::rebase).
    #[must_use]
    pub fn id(&self) -> AliasId {
        self.id
    }

    /// Whether sliding this alias is a rebase.
    ///
    /// True when the target resolves to a single leaf — RAM, ROM, I/O, or a
    /// chain of aliases onto one of those. When the target is a *container*,
    /// sliding the window changes *which regions* appear in it, and that is a
    /// retopology by definition: the flat view has to be rebuilt, so the
    /// offset cannot be updated behind the dispatcher's back.
    #[must_use]
    pub fn is_rebasable(&self) -> bool {
        self.rebasable
    }

    /// Whether the window repeats its target rather than ending at it.
    ///
    /// This is what an incompletely decoded address bus does: the NES routes
    /// only A0-A2 to the PPU, so its eight registers appear 1024 times between
    /// `$2000` and `$3FFF`. Modelling that as 1024 aliases would be honest and
    /// useless; a repeating window is one flat entry with a modulus.
    #[must_use]
    pub fn repeats(&self) -> bool {
        self.repeat
    }

    /// The repeat period — the target's size — when this window repeats.
    #[must_use]
    pub fn period(&self) -> Option<u64> {
        if self.repeat && !self.target.is_empty() {
            Some(self.target.len())
        } else {
            None
        }
    }

    pub(super) fn cell(&self) -> &Arc<AtomicU64> {
        &self.offset
    }
}

/// A child region placed inside a container.
#[derive(Debug, Clone)]
pub struct Mapping {
    /// The region being placed.
    pub region: RegionRef,
    /// Where its offset 0 lands in the parent's coordinates.
    pub base: u64,
    /// Higher wins where children overlap. Ties break by mapping order.
    pub priority: i32,
    /// What this placement permits — the *terms* on which the region answers
    /// here, which is not a property of the region. [`Perms::RWX`] unless
    /// somebody says otherwise.
    pub perms: Perms,
}

impl Mapping {
    /// Place `region` at `base` with priority 0 and no restriction.
    #[must_use]
    pub fn new(region: impl Into<RegionRef>, base: u64) -> Self {
        Mapping {
            region: region.into(),
            base,
            priority: 0,
            perms: Perms::RWX,
        }
    }

    /// Same mapping at a different priority.
    #[must_use]
    pub fn with_priority(mut self, priority: i32) -> Self {
        self.priority = priority;
        self
    }

    /// Same mapping on narrower terms.
    ///
    /// This is the whole of the permission mechanism from a caller's side. A
    /// read-only view of writable memory is `Mapping::new(region, base)
    /// .with_perms(Perms::RX)`; the store is untouched and can still be
    /// read-write somewhere else, which is what makes copy-on-write a mapping
    /// change rather than a store change.
    ///
    /// Where two mappings overlap and neither permits what the other does, the
    /// flattener resolves **reads and writes separately** — so "read this
    /// store, write that one" is two overlapping mappings, not a special
    /// region kind. See [`AddressSpace`](super::AddressSpace)'s module docs.
    #[must_use]
    pub fn with_perms(mut self, perms: Perms) -> Self {
        self.perms = perms;
        self
    }

    /// Where this mapping ends in the parent's coordinates, saturating.
    #[must_use]
    pub fn end(&self) -> u64 {
        self.base.saturating_add(self.region.len())
    }
}

/// A container: a window of address space with children placed inside it.
#[derive(Debug)]
pub struct Container {
    children: Vec<Mapping>,
    combine: CombinePolicy,
}

impl Container {
    /// The children, in mapping order.
    #[must_use]
    pub fn children(&self) -> &[Mapping] {
        &self.children
    }

    /// How overlaps between children are resolved.
    #[must_use]
    pub fn combine(&self) -> CombinePolicy {
        self.combine
    }
}

/// What a region is.
///
/// The five kinds of `ROADMAP.md` §4.1, with the payloads hoisted into named
/// types so that a container or an alias can grow a field without every match
/// arm in the crate changing shape.
#[derive(Debug)]
#[non_exhaustive]
pub enum RegionKind {
    /// Writable guest memory.
    Ram(Arc<RamStore>),
    /// Read-only memory; writes follow [`RomWrite`].
    Rom {
        /// The contents.
        store: Arc<RomStore>,
        /// What a write does.
        on_write: RomWrite,
    },
    /// MMIO: always a call into [`MemOps`].
    Io(Arc<dyn MemOps>),
    /// A window onto another region — a mirror, a bank window, a bridge
    /// aperture.
    Alias(Alias),
    /// A sub-tree with its own coordinate space.
    Container(Container),
}

/// A node in the region tree.
///
/// Built through the constructors below, then shared as a [`RegionRef`].
/// Immutable apart from an [`Alias`]'s offset cell.
#[derive(Debug)]
pub struct Region {
    name: String,
    len: u64,
    constraints: AccessConstraints,
    kind: RegionKind,
    /// How deep the tree under this node goes, counting this node as 1 and
    /// saturating.
    ///
    /// Computed once at construction rather than walked at flatten time, and
    /// the reason is not speed: [`TopologyGuard`](super::TopologyGuard) defers
    /// its flatten to the end of a batch, so the flatten happens where an
    /// error can no longer be returned to the caller who caused it. Depth is
    /// the only way flattening can fail, so it is checked when the mapping is
    /// *added* — which needs it to be O(1) there, not O(tree).
    depth: u32,
}

/// An aperture whose reads and writes reach different devices.
///
/// See [`Region::split`].
#[derive(Debug)]
struct Split {
    reads: Arc<dyn MemOps>,
    writes: Arc<dyn MemOps>,
    constraints: AccessConstraints,
}

impl MemOps for Split {
    fn read(&self, offset: u64, dst: &mut [u8], attrs: super::attrs::MemAttrs) -> super::MemResult {
        self.reads.read(offset, dst, attrs)
    }

    fn write(&self, offset: u64, src: &[u8], attrs: super::attrs::MemAttrs) -> super::MemResult {
        self.writes.write(offset, src, attrs)
    }

    fn constraints(&self) -> AccessConstraints {
        self.constraints
    }
}

impl Region {
    /// One aperture, two devices: reads go to `reads`, writes go to `writes`.
    ///
    /// A `map` statement routes *both halves* of an access to one region, and
    /// that is the right default — but it cannot describe an address where two
    /// different registers live, one readable and one writable. The NES has
    /// exactly that at `$4017`: a write reaches the APU's frame counter and a
    /// read reaches controller two, and neither half is optional (the frame
    /// counter's IRQ drives game logic, and the controller is player two).
    /// Without this the machine has to choose one and lose the other.
    ///
    /// The two sides keep their own `MemOps`, so each device still sees only
    /// the half of the access that belongs to it. The aperture's constraints
    /// are the **read** side's: `drives_data_bus` is a property of a read, and
    /// the widths must agree anyway.
    ///
    /// # Errors
    ///
    /// If either side is not a plain I/O region — a window or a container has
    /// no single [`MemOps`] to split — if they are different sizes, or if they
    /// accept different access widths.
    pub fn split(
        name: impl Into<String>,
        reads: impl Into<RegionRef>,
        writes: impl Into<RegionRef>,
    ) -> Result<Self, Error> {
        let name = name.into();
        let reads = reads.into();
        let writes = writes.into();
        let ops = |side: &RegionRef, which: &str| match side.kind() {
            RegionKind::Io(ops) => Ok(Arc::clone(ops)),
            _ => Err(Error::Config {
                at: name.clone(),
                message: alloc::format!(
                    "the {which} side of a split must be a plain I/O region, and `{}` is not",
                    side.name()
                ),
            }),
        };
        let read_ops = ops(&reads, "read")?;
        let write_ops = ops(&writes, "write")?;
        if reads.len() != writes.len() {
            return Err(Error::Config {
                at: name,
                message: alloc::format!(
                    "a split's two sides must be the same size: `{}` is {:#x} bytes and `{}` is                      {:#x}",
                    reads.name(),
                    reads.len(),
                    writes.name(),
                    writes.len()
                ),
            });
        }
        let constraints = reads.constraints;
        if constraints.min != writes.constraints.min || constraints.max != writes.constraints.max {
            return Err(Error::Config {
                at: name,
                message: alloc::format!(
                    "a split's two sides must accept the same access widths: `{}` and `{}` do not",
                    reads.name(),
                    writes.name()
                ),
            });
        }
        let len = reads.len();
        Ok(Region {
            name,
            len,
            constraints,
            depth: 1,
            kind: RegionKind::Io(Arc::new(Split {
                reads: read_ops,
                writes: write_ops,
                constraints,
            })),
        })
    }

    /// Writable memory backed by `store`, the size of the store.
    #[must_use]
    pub fn ram(name: impl Into<String>, store: Arc<RamStore>) -> Self {
        let len = store.len();
        Region {
            name: name.into(),
            len,
            constraints: AccessConstraints::ANY,
            depth: 1,
            kind: RegionKind::Ram(store),
        }
    }

    /// Read-only memory backed by `store`, the size of the store.
    #[must_use]
    pub fn rom(name: impl Into<String>, store: Arc<RomStore>, on_write: RomWrite) -> Self {
        let len = store.len();
        Region {
            name: name.into(),
            len,
            constraints: AccessConstraints::ANY,
            depth: 1,
            kind: RegionKind::Rom { store, on_write },
        }
    }

    /// An MMIO aperture of `len` bytes served by `ops`.
    ///
    /// The region's constraints come from [`MemOps::constraints`], so a device
    /// declares its access rules once and both the fast reject and its own
    /// handler agree.
    #[must_use]
    pub fn io(name: impl Into<String>, len: u64, ops: Arc<dyn MemOps>) -> Self {
        let constraints = ops.constraints();
        Region {
            name: name.into(),
            len,
            constraints,
            depth: 1,
            kind: RegionKind::Io(ops),
        }
    }

    /// A window of `len` bytes onto `target`, starting at `offset` inside it.
    ///
    /// # Errors
    ///
    /// If the window does not fit inside the target. A mirror that runs off
    /// the end of what it mirrors is a machine-description bug, and finding it
    /// at construction is much cheaper than finding it as a mystery bus fault.
    pub fn alias(
        name: impl Into<String>,
        target: impl Into<RegionRef>,
        offset: u64,
        len: u64,
    ) -> Result<Self, Error> {
        let name = name.into();
        let target = target.into();
        let end = offset.checked_add(len).ok_or_else(|| Error::Config {
            at: name.clone(),
            message: "alias window overflows".to_string(),
        })?;
        if end > target.len() {
            return Err(Error::Config {
                at: name,
                message: alloc::format!(
                    "alias window {offset:#x}..{end:#x} does not fit in target `{}` of {:#x} bytes",
                    target.name(),
                    target.len()
                ),
            });
        }
        let rebasable = target.resolves_to_leaf();
        let constraints = target.constraints;
        let depth = target.depth.saturating_add(1);
        Ok(Region {
            name,
            len,
            constraints,
            depth,
            kind: RegionKind::Alias(Alias {
                target,
                offset: Arc::new(AtomicU64::new(offset)),
                id: AliasId(NEXT_ALIAS_ID.fetch_add(1, Ordering::Relaxed)),
                rebasable,
                repeat: false,
            }),
        })
    }

    /// A window of `len` bytes in which `target` repeats end to end.
    ///
    /// The NES `$0000-$1FFF` (2 KiB of RAM, four times) and `$2000-$3FFF`
    /// (eight PPU registers, 1024 times) are both this, and both are one flat
    /// entry rather than four or 1024.
    ///
    /// # Errors
    ///
    /// If `target` is empty, or is not a single leaf. A repeating window onto
    /// a *container* would have to be reflattened per period, which is a
    /// different and much more expensive thing; build the container's contents
    /// into a leaf, or repeat the mapping explicitly.
    pub fn mirror(
        name: impl Into<String>,
        target: impl Into<RegionRef>,
        len: u64,
    ) -> Result<Self, Error> {
        let name = name.into();
        let target = target.into();
        if target.is_empty() {
            return Err(Error::Config {
                at: name,
                message: "cannot mirror a zero-sized region".to_string(),
            });
        }
        if !matches!(
            target.kind(),
            RegionKind::Ram(_) | RegionKind::Rom { .. } | RegionKind::Io(_)
        ) {
            return Err(Error::Config {
                at: name,
                message: "a repeating window's target must be RAM, ROM, or I/O".to_string(),
            });
        }
        let constraints = target.constraints;
        let depth = target.depth.saturating_add(1);
        Ok(Region {
            name,
            len,
            constraints,
            depth,
            kind: RegionKind::Alias(Alias {
                target,
                offset: Arc::new(AtomicU64::new(0)),
                id: AliasId(NEXT_ALIAS_ID.fetch_add(1, Ordering::Relaxed)),
                rebasable: false,
                repeat: true,
            }),
        })
    }

    /// A container of `len` bytes holding `children`, resolving overlaps by
    /// priority.
    #[must_use]
    pub fn container(name: impl Into<String>, len: u64, children: Vec<Mapping>) -> Self {
        Self::container_with(name, len, children, CombinePolicy::Priority)
    }

    /// A container with an explicit [`CombinePolicy`].
    #[must_use]
    pub fn container_with(
        name: impl Into<String>,
        len: u64,
        children: Vec<Mapping>,
        combine: CombinePolicy,
    ) -> Self {
        let depth = children
            .iter()
            .map(|m| m.region.depth)
            .max()
            .unwrap_or(0)
            .saturating_add(1);
        Region {
            name: name.into(),
            len,
            constraints: AccessConstraints::ANY,
            depth,
            kind: RegionKind::Container(Container { children, combine }),
        }
    }

    /// Same region with different access constraints.
    ///
    /// On an alias this narrows the *window* rather than the target: a 16-bit
    /// aperture onto a 32-bit device is a real thing, and the window's rules
    /// replace the target's for accesses that arrive through it.
    #[must_use]
    pub fn with_constraints(mut self, constraints: AccessConstraints) -> Self {
        self.constraints = constraints;
        self
    }

    /// Same region in `endian` byte order.
    #[must_use]
    pub fn with_endian(mut self, endian: Endian) -> Self {
        self.constraints = self.constraints.with_endian(endian);
        self
    }

    /// Same region, clipped or extended to `len` bytes.
    ///
    /// Used for an aperture that is decoded larger than its backing store —
    /// the region simply faults past the store's end.
    #[must_use]
    pub fn with_len(mut self, len: u64) -> Self {
        self.len = len;
        self
    }

    /// This region's name, as the machine description gave it.
    #[must_use]
    pub fn name(&self) -> &str {
        &self.name
    }

    /// Size in bytes.
    #[inline]
    #[must_use]
    pub fn len(&self) -> u64 {
        self.len
    }

    /// Whether the region is zero-sized.
    #[inline]
    #[must_use]
    pub fn is_empty(&self) -> bool {
        self.len == 0
    }

    /// What this region is.
    #[must_use]
    pub fn kind(&self) -> &RegionKind {
        &self.kind
    }

    /// How deep the tree rooted here goes, counting this node as 1.
    ///
    /// A leaf is 1; an alias is one more than its target; a container is one
    /// more than its deepest child. Maintained at construction, so a caller
    /// about to map this region can reject an over-deep tree in constant time
    /// — see [`TopologyGuard::map`](super::TopologyGuard::map).
    #[inline]
    #[must_use]
    pub fn depth(&self) -> u32 {
        self.depth
    }

    /// What this region accepts.
    #[inline]
    #[must_use]
    pub fn constraints(&self) -> AccessConstraints {
        self.constraints
    }

    /// The alias node, if this is an alias.
    #[must_use]
    pub fn as_alias(&self) -> Option<&Alias> {
        match &self.kind {
            RegionKind::Alias(a) => Some(a),
            _ => None,
        }
    }

    /// The container node, if this is a container.
    #[must_use]
    pub fn as_container(&self) -> Option<&Container> {
        match &self.kind {
            RegionKind::Container(c) => Some(c),
            _ => None,
        }
    }

    /// Whether this region is (or aliases) a single leaf rather than a tree.
    fn resolves_to_leaf(&self) -> bool {
        match &self.kind {
            RegionKind::Ram(_) | RegionKind::Rom { .. } | RegionKind::Io(_) => true,
            RegionKind::Alias(a) => a.rebasable && !a.repeat,
            RegionKind::Container(_) => false,
        }
    }
}
