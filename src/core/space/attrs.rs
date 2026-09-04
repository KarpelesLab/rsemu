//! Access attributes, region-level access constraints, and the [`MemOps`]
//! trait every I/O region is driven through (`ROADMAP.md` §4.1).

use crate::core::error::BusError;
use crate::core::value::{Endian, Width};
use core::fmt;

/// The result of a guest access.
///
/// Three outcomes, never two: `Ok`, a bus fault ([`BusError::Unassigned`] or
/// [`BusError::BadAccess`]), and [`BusError::Retry`]. `Retry` is only legal
/// *before* any side effect or partial transfer; the dispatcher downgrades a
/// late one to [`BusError::BadAccess`] rather than re-running a half-completed
/// access.
pub type MemResult<T = ()> = core::result::Result<T, BusError>;

/// The terms on which a mapping answers: which directions of access it
/// permits.
///
/// A bit set rather than an enumeration, because the three genuinely combine
/// and every one of the eight combinations is something a real system asks
/// for. The spelling is the one `/proc/*/maps` and `mprotect(2)` use, because
/// level 3's `Prot` **is** this type ([`usermode::Prot`] is an alias): a
/// process's page permissions and a board's decode are the same question asked
/// twice, and answering it twice is how the two drift apart.
///
/// # Where it lives, and why not on the region
///
/// Permission is a property of the **mapping**, not of the region it places. A
/// ROM chip is a ROM chip; whether *this* bus may write to it is a property of
/// the decode in front of it. The same `Arc<RamStore>` can legitimately be
/// read-write in one space and read-only in another, and that is exactly what
/// makes copy-on-write expressible — see [`Mapping::with_perms`].
///
/// Permissions **intersect** down the region tree: a child of a read-only
/// container is read-only however it was mapped, because the container's
/// decode is in front of it.
///
/// [`Mapping::with_perms`]: super::Mapping::with_perms
/// [`usermode::Prot`]: crate::usermode::Prot
#[repr(transparent)]
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct Perms(pub u8);

impl Perms {
    /// Nothing at all. Both directions raise [`BusError::Protected`].
    pub const NONE: Perms = Perms(0);
    /// The mapping may be read.
    pub const READ: Perms = Perms(1);
    /// The mapping may be written.
    pub const WRITE: Perms = Perms(2);
    /// Instructions may be fetched from the mapping.
    ///
    /// **Carried, not enforced.** Telling a fetch from a load is the master's
    /// job and no rsemu core marks one yet, so nothing here can distinguish
    /// them; enforcing it would put an unconditional branch on the read path
    /// for a bit nothing sets. The bit exists so a consumer's `PROT_EXEC`
    /// survives a round trip through a mapping and a snapshot, and so that the
    /// day a core marks its fetches this becomes a one-line change rather than
    /// a schema change.
    pub const EXEC: Perms = Perms(4);
    /// Readable and writable — ordinary memory.
    pub const RW: Perms = Perms(3);
    /// Readable and executable — a text segment.
    pub const RX: Perms = Perms(5);
    /// Everything, and the default: a mapping that says nothing about
    /// permission permits everything, so a machine that has never heard of
    /// this type behaves exactly as it did before it existed.
    pub const RWX: Perms = Perms(7);

    /// Whether every bit of `other` is set here.
    #[inline]
    #[must_use]
    pub const fn contains(self, other: Perms) -> bool {
        self.0 & other.0 == other.0
    }

    /// The union of two permission sets.
    #[must_use]
    pub const fn union(self, other: Perms) -> Perms {
        Perms(self.0 | other.0)
    }

    /// The intersection — what survives passing through a narrower mapping in
    /// front of this one.
    #[inline]
    #[must_use]
    pub const fn intersect(self, other: Perms) -> Perms {
        Perms(self.0 & other.0)
    }

    /// The same permissions, less `other`.
    #[must_use]
    pub const fn without(self, other: Perms) -> Perms {
        Perms(self.0 & !other.0)
    }

    /// Whether this is [`Perms::NONE`].
    #[must_use]
    pub const fn is_none(self) -> bool {
        self.0 == 0
    }
}

impl Default for Perms {
    /// [`Perms::RWX`] — see the constant for why the permissive value is the
    /// default rather than the restrictive one.
    fn default() -> Self {
        Perms::RWX
    }
}

impl fmt::Display for Perms {
    /// The `rwx` form `/proc/*/maps` uses, so a consumer that prints one does
    /// not have to reinvent the spelling.
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let bit = |set: bool, c: char| if set { c } else { '-' };
        write!(
            f,
            "{}{}{}",
            bit(self.contains(Perms::READ), 'r'),
            bit(self.contains(Perms::WRITE), 'w'),
            bit(self.contains(Perms::EXEC), 'x'),
        )
    }
}

/// Identifies the bus master behind an access.
///
/// Opaque to the core: a PCI requester ID, an AXI master ID, or a CPU index —
/// whatever the machine assigns. It exists so an IOMMU or a per-master filter
/// has something to key on without the core knowing what a PCI device is
/// (`ROADMAP.md` §15, invariant 1).
#[repr(transparent)]
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash, Default)]
pub struct RequesterId(pub u32);

impl RequesterId {
    /// The master that did not identify itself — the default for a CPU access
    /// in a machine with no IOMMU.
    pub const ANONYMOUS: RequesterId = RequesterId(0);
}

impl fmt::Display for RequesterId {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "requester#{}", self.0)
    }
}

/// Everything about an access that is not its address, width, or direction.
///
/// Carried on every access because retrofitting it is a rewrite of every
/// device signature. Constructed from [`MemAttrs::DEFAULT`] (or
/// [`MemAttrs::DEBUG`]) plus `with_*` builders, so adding an attribute later
/// does not break callers.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
#[non_exhaustive]
pub struct MemAttrs {
    /// Which bus master is asking.
    pub requester: RequesterId,
    /// The access is made in a secure world (ARM TrustZone, and anything
    /// shaped like it).
    pub secure: bool,
    /// The access is made from a privileged mode rather than user code.
    pub privileged: bool,
    /// Part of an exclusive/atomic sequence (`LDREX`/`STREX`, LR/SC).
    ///
    /// The core carries the flag; the monitor that implements the reservation
    /// lives with the CPU, not here.
    ///
    /// **That is only sound while one core owns an address space, and since
    /// SMP landed it is not.** Every core keeps its reservation privately
    /// (`cpu::riscv`'s `reservation`, `cpu::arm::a64`'s `exclusive`), so a
    /// sibling's store does not break it: a `sc.d`/`stxr` the architecture
    /// *requires* to fail succeeds, and the sibling's update is lost. Nothing
    /// in the tree reads this flag back — a **global monitor on the address
    /// space** is what would, and it does not exist yet.
    ///
    /// Reproduced hermetically on both architectures by
    /// `a_reservation_is_core_local_so_two_threads_lose_an_update` in
    /// `usermode::proof`, which is written to *fail* when the monitor lands
    /// and says what to change it to. It reaches every multiprocessor board:
    /// `arm64-virt-smp` and `pc-at-smp` both run kernel spinlocks built on
    /// exactly this sequence.
    pub exclusive: bool,
    /// The access comes from a debugger, a monitor, or a snapshot, and **must
    /// have no side effects** — no FIFO pop, no status-bit clear, no pointer
    /// advance (`ROADMAP.md` §15, invariant 5).
    ///
    /// The core honours this by never charging an access to the unassigned
    /// log; every MMIO device is required to honour it in its own handler.
    pub debug: bool,
    /// The last byte this master drove on its data bus.
    ///
    /// A bus is wires, not a function: nothing pulls an unanswered line to a
    /// defined level, so the charge the master itself last put there is what
    /// it reads back. Every machine that decodes fewer addresses than it has
    /// depends on this — a 6502 reading `$4000` gets `$40`, the high byte of
    /// its own operand — and so does every write-only register, which answers
    /// a read by not driving the bus at all.
    ///
    /// Carried on the access rather than remembered by the space because it
    /// belongs to the *master*: two masters on one bus have two latches, and
    /// a DMA cycle updates the one that drove it. A master with no such latch
    /// leaves it zero, which is what a bus with a pull-down does.
    pub bus: u8,
    /// The last byte on the master's **own, on-die** data bus.
    ///
    /// Usually the same byte as [`MemAttrs::bus`], and deliberately not always.
    /// A master with registers on its own die has two buses: the pins, and the
    /// wires inside it. A cycle *another* master ran — a DMA that stole the bus
    /// — moves the pins and not the inside; a read of an on-die register moves
    /// the inside and not the pins.
    ///
    /// The RP2A03 is the case that needs it. `$4015` is on the CPU's die, its
    /// bit 5 comes from the internal bus, and AccuracyCoin's "Internal Data
    /// Bus" test lands a DMC DMA in the middle of a `LDA $4015` to prove the
    /// sample byte does *not* reach it.
    pub core_bus: u8,
}

impl MemAttrs {
    /// An ordinary non-secure, unprivileged, non-debug access.
    pub const DEFAULT: MemAttrs = MemAttrs {
        requester: RequesterId::ANONYMOUS,
        secure: false,
        privileged: false,
        exclusive: false,
        debug: false,
        bus: 0,
        core_bus: 0,
    };

    /// A side-effect-free access, as issued by a debugger or a snapshot.
    ///
    /// Also privileged and secure: a monitor looks at the whole machine, and a
    /// debugger that could not read secure memory would be useless.
    pub const DEBUG: MemAttrs = MemAttrs {
        requester: RequesterId::ANONYMOUS,
        secure: true,
        privileged: true,
        exclusive: false,
        debug: true,
        bus: 0,
        core_bus: 0,
    };

    /// Same attributes, from `id`.
    #[must_use]
    pub const fn with_requester(mut self, id: RequesterId) -> Self {
        self.requester = id;
        self
    }

    /// Same attributes, with the secure flag set to `secure`.
    #[must_use]
    pub const fn with_secure(mut self, secure: bool) -> Self {
        self.secure = secure;
        self
    }

    /// Same attributes, with the privileged flag set to `privileged`.
    #[must_use]
    pub const fn with_privileged(mut self, privileged: bool) -> Self {
        self.privileged = privileged;
        self
    }

    /// Same attributes, with the exclusive flag set to `exclusive`.
    #[must_use]
    pub const fn with_exclusive(mut self, exclusive: bool) -> Self {
        self.exclusive = exclusive;
        self
    }

    /// Same attributes, carrying `bus` as the master's last driven byte.
    #[must_use]
    pub const fn with_bus(mut self, bus: u8) -> Self {
        self.bus = bus;
        self
    }

    /// Same attributes, carrying `bus` as the master's own on-die bus value.
    #[must_use]
    pub const fn with_core_bus(mut self, bus: u8) -> Self {
        self.core_bus = bus;
        self
    }

    /// Same attributes, with the debug flag set to `debug`.
    #[must_use]
    pub const fn with_debug(mut self, debug: bool) -> Self {
        self.debug = debug;
        self
    }
}

/// What a region will accept: widths, alignment, byte order, and the two
/// attribute filters cheap enough to check here.
///
/// This is a **fast reject**, not the whole guarantee (`ROADMAP.md` §4.1). A
/// register block with per-register rules still enforces its own inside
/// [`MemOps`]; what this catches is the byte write to a 32-bit-only aperture,
/// before a virtual call is made.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[non_exhaustive]
pub struct AccessConstraints {
    /// Narrowest single access accepted.
    pub min: Width,
    /// Widest single access accepted.
    pub max: Width,
    /// Require natural alignment of the *region-relative* offset.
    ///
    /// Region-relative rather than absolute: a device with 32-bit registers
    /// cares where the access lands in its register file, which is not the
    /// same question as where the aperture happens to sit in the space.
    pub natural_alignment: bool,
    /// Byte order of this region. A big-endian device on a little-endian bus
    /// is normal, not exotic.
    pub endian: Endian,
    /// Whether a transfer that is not a single legal width (a DMA burst, a ROM
    /// load, a debugger dump) may be handed to the region in one call.
    ///
    /// True for memory, false by default for I/O: a device that wants bursts
    /// says so, rather than being handed a 4096-byte "register write".
    pub allow_bulk: bool,
    /// Reject non-secure accesses.
    pub secure_only: bool,
    /// Reject unprivileged accesses.
    pub privileged_only: bool,
    /// Whether a read of this region drives the master's external data bus.
    ///
    /// True for anything on the far side of the pins, which is nearly
    /// everything. False for a register on the *master's own die*: the 2A03's
    /// `$4015` is read straight into the core, so the external bus keeps
    /// whatever was on it and the next open-bus read still sees the old byte
    /// (NESdev wiki, "APU": "this register is internal to the CPU and so the
    /// external CPU data bus is disconnected when reading it").
    ///
    /// Only a master that models an open-bus latch looks at it.
    pub drives_data_bus: bool,
}

impl AccessConstraints {
    /// Anything goes: any width, any alignment, little-endian, bursts allowed.
    pub const ANY: AccessConstraints = AccessConstraints {
        min: Width::U8,
        max: Width::U64,
        natural_alignment: false,
        endian: Endian::Little,
        allow_bulk: true,
        secure_only: false,
        privileged_only: false,
        drives_data_bus: true,
    };

    /// The default for an I/O region: any width, but no bulk bursts.
    pub const IO: AccessConstraints = AccessConstraints {
        allow_bulk: false,
        ..AccessConstraints::ANY
    };

    /// Exactly one width, naturally aligned, in `endian` byte order — the
    /// common shape of a hardware register block.
    #[must_use]
    pub const fn word(width: Width, endian: Endian) -> Self {
        AccessConstraints {
            min: width,
            max: width,
            natural_alignment: true,
            endian,
            allow_bulk: false,
            secure_only: false,
            privileged_only: false,
            drives_data_bus: true,
        }
    }

    /// Same constraints, but a read of this region leaves the master's data
    /// bus alone — the register is on the master's own die.
    #[must_use]
    pub const fn internal(mut self) -> Self {
        self.drives_data_bus = false;
        self
    }

    /// Same constraints, in `endian` byte order.
    #[must_use]
    pub const fn with_endian(mut self, endian: Endian) -> Self {
        self.endian = endian;
        self
    }

    /// Same constraints, accepting widths from `min` to `max` inclusive.
    #[must_use]
    pub const fn with_widths(mut self, min: Width, max: Width) -> Self {
        self.min = min;
        self.max = max;
        self
    }

    /// Same constraints, requiring (or not) natural alignment.
    #[must_use]
    pub const fn with_natural_alignment(mut self, require: bool) -> Self {
        self.natural_alignment = require;
        self
    }

    /// Same constraints, allowing (or not) bulk transfers.
    #[must_use]
    pub const fn with_bulk(mut self, allow: bool) -> Self {
        self.allow_bulk = allow;
        self
    }

    /// Same constraints, rejecting non-secure accesses.
    #[must_use]
    pub const fn with_secure_only(mut self, secure_only: bool) -> Self {
        self.secure_only = secure_only;
        self
    }

    /// Same constraints, rejecting unprivileged accesses.
    #[must_use]
    pub const fn with_privileged_only(mut self, privileged_only: bool) -> Self {
        self.privileged_only = privileged_only;
        self
    }

    /// Check a single width-typed access at region-relative `offset`.
    ///
    /// Returns [`BusError::BadAccess`] rather than silently widening or
    /// splitting: a 32-bit-only register must *reject* a byte write.
    #[inline]
    pub fn check(&self, offset: u64, width: Width, attrs: MemAttrs) -> MemResult {
        self.check_attrs(attrs)?;
        if width < self.min || width > self.max {
            return Err(BusError::BadAccess);
        }
        if self.natural_alignment && !width.is_aligned(offset) {
            return Err(BusError::BadAccess);
        }
        Ok(())
    }

    /// Check a transfer of `len` bytes at region-relative `offset`.
    ///
    /// A transfer whose length happens to be a legal access width is checked
    /// as one; anything else needs [`AccessConstraints::allow_bulk`].
    #[inline]
    pub fn check_bulk(&self, offset: u64, len: u64, attrs: MemAttrs) -> MemResult {
        if let Some(width) = Width::from_bytes(len)
            && self.check(offset, width, attrs).is_ok()
        {
            return Ok(());
        }
        self.check_attrs(attrs)?;
        if self.allow_bulk {
            Ok(())
        } else {
            Err(BusError::BadAccess)
        }
    }

    #[inline]
    fn check_attrs(&self, attrs: MemAttrs) -> MemResult {
        if (self.secure_only && !attrs.secure) || (self.privileged_only && !attrs.privileged) {
            return Err(BusError::BadAccess);
        }
        Ok(())
    }
}

impl Default for AccessConstraints {
    fn default() -> Self {
        AccessConstraints::ANY
    }
}

/// The behaviour of an I/O region: a call, every time.
///
/// Offsets are region-relative and in bytes, and buffers are byte slices in
/// ascending address order — the region's own [`AccessConstraints::endian`]
/// decides how a guest word maps onto them, and the dispatcher has already
/// done that conversion.
///
/// Implementors are `Send + Sync` from the first commit (`ROADMAP.md` §0) and
/// `Debug` so that a machine's memory map can be printed without every
/// container needing a hand-written formatter.
///
/// **Every implementation must honour [`MemAttrs::debug`]**: a debug read may
/// not pop a FIFO, clear a status bit, or advance a pointer
/// (`ROADMAP.md` §15, invariant 5).
pub trait MemOps: fmt::Debug + Send + Sync {
    /// Read `dst.len()` bytes from region-relative `offset`.
    fn read(&self, offset: u64, dst: &mut [u8], attrs: MemAttrs) -> MemResult;

    /// Write `src` to region-relative `offset`.
    fn write(&self, offset: u64, src: &[u8], attrs: MemAttrs) -> MemResult;

    /// What this region accepts. Defaults to [`AccessConstraints::IO`].
    ///
    /// Used as the region's constraints when one is built with
    /// [`Region::io`](super::Region::io), so a device declares its rules once.
    fn constraints(&self) -> AccessConstraints {
        AccessConstraints::IO
    }
}
