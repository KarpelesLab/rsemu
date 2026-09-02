//! Enhanced Configuration Access Mechanism: configuration space reached by
//! reading and writing memory.
//!
//! # What ECAM is
//!
//! A PCI Express root complex publishes one contiguous window of memory in
//! which the address *is* the configuration address. The Intel 3 Series
//! datasheet states the arithmetic in §5.1.16, in the PCIEXBAR register's own
//! description, and it is the only sentence this module implements:
//!
//! > PCI Express Base Address + Bus Number \* 1 MB + Device Number \* 32 KB +
//! > Function Number \* 4 KB
//!
//! So a function occupies 4 KiB, a device 32 KiB, a bus 1 MiB, and 256 buses
//! come to 256 MiB. The same decomposition is the *PCI Express Base
//! Specification*'s Enhanced Configuration Access Mechanism, which is where the
//! 4 KiB per function comes from in the first place: 256 bytes of PCI-compatible
//! header and 3840 bytes of **extended** configuration space above it.
//!
//! # How much of PCI Express this actually is
//!
//! Not much, and that is the point. ECAM is a *decode*, not a protocol: nothing
//! here knows about link training, TLPs, root ports, or the capability
//! structures that make a function an Express function. What a firmware and an
//! operating system need from a q35 in order to enumerate it is exactly this
//! decode plus the `MCFG` table that says where the window is
//! ([`super::acpi`]), and both come out of the same one-line formula above.
//!
//! # Extended configuration space, and why zeroes are the right answer
//!
//! [`crate::bus::pci`] models 256 bytes per function and says so:
//! `CONFIG_SPACE_LEN` is `0x100`. This module decodes the full 4 KiB anyway and
//! answers **zero** above `0xff`, which is not a stub — it is what a conforming
//! function with no extended capabilities looks like. The PCI Express Base
//! Specification puts an Extended Capability header at `0x100`, and an
//! all-zero header (capability ID 0, version 0, next pointer 0) is the defined
//! encoding for *there are none*. A traversal of the extended capability list
//! therefore terminates immediately and correctly, rather than reading
//! something it has to guess about.
//!
//! Ones would be wrong here in a way zeroes are not: ones is what a **master
//! abort** returns (*PCI Local Bus Specification* Rev 2.1 §3.7.4.1), and a
//! function that answered ones for its own unimplemented registers would be
//! indistinguishable from an empty slot. That distinction is already
//! [`PciFunction`](crate::bus::pci::PciFunction)'s contract; this window keeps
//! it.
//!
//! When a function in this tree grows a real extended capability, the seam is
//! `bus::pci`'s: widen `CONFIG_SPACE_LEN` and delete
//! [`Ecam::EXTENDED_ANSWERS_ZERO`]'s branch. Nothing else here changes.
//!
//! # `MemAttrs::debug`
//!
//! A debug **read** is forwarded, with the flag, and is genuinely safe — which
//! is a real difference from
//! [`ConfigPorts`](crate::bus::pci::ConfigPorts), where a debugger cannot even
//! read, because reaching a register there means writing the `CONFADD` latch
//! first and that moves the guest's own next access. ECAM carries the whole
//! address in the address, so there is no latch to disturb: a debugger's memory
//! window may sit over this range and poll it.
//!
//! A debug **write** is refused. A configuration write is how a BAR moves and
//! how a chipset's windows are switched, and there is no harmless subset of it
//! (`CLAUDE.md`: a debugger read must not move anything).
//!
//! # Sources
//!
//! * *Intel 3 Series Express Chipset Family Datasheet*, order number 316966-002,
//!   §5.1.16 (`PCIEXBAR`) for the address arithmetic and the three window sizes.
//! * *PCI Express Base Specification* for the Enhanced Configuration Access
//!   Mechanism and for the extended capability list's all-zero terminator.
//! * *PCI Local Bus Specification* Rev 2.1 §3.7.4.1 for what an unclaimed
//!   configuration cycle returns.
//!
//! No emulator source was consulted (`CLAUDE.md`, provenance).

use alloc::sync::{Arc, Weak};

use crate::bus::pci::{Bdf, CONFIG_SPACE_LEN, PciBus};
use crate::core::error::BusError;
use crate::core::space::{AccessConstraints, MemAttrs, MemOps, MemResult};
use crate::core::value::{Endian, Width};

/// How much of the window one function occupies: 4 KiB (§5.1.16).
pub const FUNCTION_STRIDE: u64 = 4096;

/// How much one device occupies: eight functions, 32 KiB.
pub const DEVICE_STRIDE: u64 = FUNCTION_STRIDE * 8;

/// How much one bus occupies: thirty-two devices, 1 MiB.
pub const BUS_STRIDE: u64 = DEVICE_STRIDE * 32;

/// The window PCIEXBAR's `LENGTH` field can select, smallest first.
///
/// §5.1.16: `00` is 256 MB and buses 0-255, `01` is 128 MB and buses 0-127,
/// `10` is 64 MB and buses 0-63. `11` is reserved.
pub const WINDOW_LENGTHS: [u64; 3] = [256 * 1024 * 1024, 128 * 1024 * 1024, 64 * 1024 * 1024];

/// The window a `LENGTH` encoding selects, or `None` for the reserved one.
#[must_use]
pub fn window_len(length: u8) -> Option<u64> {
    match length & 0x3 {
        0 => Some(WINDOW_LENGTHS[0]),
        1 => Some(WINDOW_LENGTHS[1]),
        2 => Some(WINDOW_LENGTHS[2]),
        _ => None,
    }
}

/// A memory window onto a PCI fabric's configuration space.
///
/// Holds nothing but the fabric: the address carries everything else, which is
/// the whole difference between this and
/// [`ConfigPorts`](crate::bus::pci::ConfigPorts). There is no latch, so there
/// is no lock, so there is nothing for the re-entrancy contract to be about — a
/// configuration write that retopologises an address space finds this object
/// holding no state at all.
///
/// # The fabric is held **weakly**, and it has to be
///
/// This window is a [`Region`](crate::core::space::Region)'s
/// [`MemOps`], and the region belongs to the host bridge —
/// which is itself a function *on this fabric*. A strong handle here would
/// close the loop `bridge → region → window → fabric → bridge`, and an `Arc`
/// cycle is a leak: the bridge could never be dropped, and a process that built
/// two machines would keep both for ever. LeakSanitizer found exactly that,
/// through the `q35_chipset` fuzz target, before this was a `Weak`.
///
/// `ROADMAP.md` §4.3 already names the shape — "the machine owns devices and an
/// interconnect merely refers to them" — and this is the same edge pointing the
/// other way. The upgrade costs one atomic per configuration cycle, which is a
/// cold path by construction: a configuration access is firmware enumerating a
/// bus, never a guest's inner loop.
///
/// A fabric that has gone away reads as **ones**, which is the same answer an
/// address nothing answers at gives (*PCI Local Bus Specification* Rev 2.1
/// §3.7.4.1). There is no state to be wrong about: a window onto a fabric that
/// no longer exists is a window onto an empty bus.
#[derive(Debug)]
pub struct Ecam {
    bus: Weak<PciBus>,
}

impl Ecam {
    /// Documented in the module docs: offsets at or above
    /// [`CONFIG_SPACE_LEN`] read as zero, which is a conforming function with
    /// no extended capabilities.
    pub const EXTENDED_ANSWERS_ZERO: bool = true;

    /// A window onto `bus`, held weakly — see the type's own documentation for
    /// why the edge cannot be strong.
    #[must_use]
    pub fn new(bus: &Arc<PciBus>) -> Ecam {
        Ecam {
            bus: Arc::downgrade(bus),
        }
    }

    /// The fabric this window reaches, if it is still there.
    #[must_use]
    pub fn bus(&self) -> Option<Arc<PciBus>> {
        self.bus.upgrade()
    }

    /// Split a window offset into the address it names and the register in it.
    ///
    /// The inverse of §5.1.16's sentence, and the only arithmetic in this file.
    #[must_use]
    pub fn split(offset: u64) -> (Bdf, u16) {
        let bdf = Bdf {
            bus: ((offset / BUS_STRIDE) & 0xff) as u8,
            device: ((offset / DEVICE_STRIDE) & 0x1f) as u8,
            function: ((offset / FUNCTION_STRIDE) & 0x7) as u8,
        };
        (bdf, (offset % FUNCTION_STRIDE) as u16)
    }
}

impl MemOps for Ecam {
    fn read(&self, offset: u64, dst: &mut [u8], attrs: MemAttrs) -> MemResult {
        let (bdf, register) = Ecam::split(offset);
        // An access that would run off the end of the function's 4 KiB is one
        // no instruction issues: a load is a single aligned operand and the
        // constraints below cap it at four bytes. Refusing is better than
        // inventing a rule for it.
        let end = u64::from(register) + dst.len() as u64;
        if end > FUNCTION_STRIDE {
            return Err(BusError::BadAccess);
        }
        if register >= CONFIG_SPACE_LEN {
            // Extended configuration space. Zero, and see the module docs: an
            // all-zero extended capability header is the encoding for "this
            // function has none", not a hole.
            dst.fill(0);
            return Ok(());
        }
        // A read that straddles `0xff` would be half register file and half
        // extended space. Nothing issues one — 0xfc is the last aligned dword —
        // and splitting it silently would hide a decode bug.
        if end > u64::from(CONFIG_SPACE_LEN) {
            return Err(BusError::BadAccess);
        }
        match self.bus() {
            Some(bus) => bus.config_read(bdf, register, dst, attrs),
            // The fabric is gone, so nothing answers, so the cycle master
            // aborts — which reads as ones.
            None => dst.fill(0xff),
        }
        Ok(())
    }

    fn write(&self, offset: u64, src: &[u8], attrs: MemAttrs) -> MemResult {
        if attrs.debug {
            // A configuration write moves BARs and switches chipset windows.
            // There is no harmless subset, so a debugger does not get one —
            // the same door `ConfigPorts` locks, from the other side.
            return Err(BusError::BadAccess);
        }
        let (bdf, register) = Ecam::split(offset);
        let end = u64::from(register) + src.len() as u64;
        if end > FUNCTION_STRIDE {
            return Err(BusError::BadAccess);
        }
        if register >= CONFIG_SPACE_LEN {
            // Extended space is read-only zeroes, so a write to it is dropped
            // rather than faulted: firmware writes registers it has not sized
            // all the time, and a fault on one would be inventing behaviour
            // (Rev 2.1 §6.1 — a write to a read-only register is dropped).
            return Ok(());
        }
        if end > u64::from(CONFIG_SPACE_LEN) {
            return Err(BusError::BadAccess);
        }
        if let Some(bus) = self.bus() {
            bus.config_write(bdf, register, src, attrs);
        }
        Ok(())
    }

    fn constraints(&self) -> AccessConstraints {
        // Byte, word and dword, little-endian, no bulk. A 64-bit access to
        // configuration space is a cycle no processor generates and the fabric
        // has no answer for; refusing it is more honest than splitting it.
        AccessConstraints::IO
            .with_widths(Width::U8, Width::U32)
            .with_endian(Endian::Little)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::bus::pci::{ConfigSpace, PciFunction, config};
    use crate::core::sync::{LockRank, Mutex};

    /// A function that answers with a fixed vendor and device id.
    #[derive(Debug)]
    struct Stub(Mutex<ConfigSpace>);

    impl Stub {
        fn new(vendor: u16, device: u16) -> Arc<Stub> {
            let mut c = ConfigSpace::new();
            c.hardwire(config::VENDOR_ID, u32::from(vendor), 2);
            c.hardwire(config::DEVICE_ID, u32::from(device), 2);
            c.allow(config::COMMAND, 2);
            Arc::new(Stub(Mutex::with_rank(LockRank::DEVICE, c)))
        }
    }

    impl PciFunction for Stub {
        fn config_read(&self, offset: u16, dst: &mut [u8], _attrs: MemAttrs) {
            self.0.lock().read(offset, dst);
        }
        fn config_write(&self, offset: u16, src: &[u8], _attrs: MemAttrs) {
            self.0.lock().write(offset, src);
        }
    }

    fn fabric() -> (Arc<PciBus>, Ecam) {
        let bus = Arc::new(PciBus::new());
        let ecam = Ecam::new(&bus);
        (bus, ecam)
    }

    #[test]
    fn a_window_onto_a_fabric_that_has_gone_reads_as_ones() {
        let ecam = {
            let bus = Arc::new(PciBus::new());
            bus.attach(Bdf::default(), Stub::new(0x8086, 0x29b0))
                .expect("empty");
            Ecam::new(&bus)
        };
        assert!(ecam.bus().is_none(), "the fabric was dropped");
        let mut id = [0u8; 4];
        ecam.read(0, &mut id, MemAttrs::DEFAULT).expect("ok");
        assert_eq!(u32::from_le_bytes(id), 0xffff_ffff);
        // And a write is dropped rather than faulting: an unclaimed cycle is
        // not an error.
        ecam.write(0x04, &[0x07, 0x00], MemAttrs::DEFAULT)
            .expect("ok");
    }

    #[test]
    fn the_address_is_the_address() {
        // §5.1.16's arithmetic, read backwards.
        assert_eq!(Ecam::split(0).0, Bdf::default());
        assert_eq!(
            Ecam::split(BUS_STRIDE * 3 + DEVICE_STRIDE * 31 + FUNCTION_STRIDE * 7 + 0x24),
            (
                Bdf {
                    bus: 3,
                    device: 31,
                    function: 7
                },
                0x24
            )
        );
    }

    #[test]
    fn a_function_answers_where_the_formula_puts_it() {
        let (bus, ecam) = fabric();
        let at = Bdf::new(0, 31, 0).expect("a legal address");
        bus.attach(at, Stub::new(0x8086, 0x2918)).expect("empty");
        let base = DEVICE_STRIDE * 31;
        let mut id = [0u8; 4];
        ecam.read(base, &mut id, MemAttrs::DEFAULT).expect("ok");
        assert_eq!(u32::from_le_bytes(id), 0x2918_8086);
        // And a byte read of the header type, because firmware does that.
        let mut byte = [0u8; 1];
        ecam.read(base + 0x0e, &mut byte, MemAttrs::DEFAULT)
            .expect("ok");
        assert_eq!(byte[0], 0);
    }

    #[test]
    fn an_empty_address_master_aborts_and_reads_as_ones() {
        let (_bus, ecam) = fabric();
        let mut id = [0u8; 4];
        ecam.read(DEVICE_STRIDE * 5, &mut id, MemAttrs::DEFAULT)
            .expect("ok");
        assert_eq!(u32::from_le_bytes(id), 0xffff_ffff);
    }

    #[test]
    fn extended_space_is_zero_not_ones() {
        let (bus, ecam) = fabric();
        bus.attach(Bdf::default(), Stub::new(0x8086, 0x29b0))
            .expect("empty");
        let mut header = [0u8; 4];
        ecam.read(0x100, &mut header, MemAttrs::DEFAULT)
            .expect("ok");
        assert_eq!(
            u32::from_le_bytes(header),
            0,
            "an all-zero extended capability header terminates the list"
        );
        // Even at an address nothing answers: the master abort belongs to the
        // 256-byte header, and above it the answer is the function's own
        // unimplemented registers either way.
        ecam.read(DEVICE_STRIDE * 9 + 0x800, &mut header, MemAttrs::DEFAULT)
            .expect("ok");
        assert_eq!(u32::from_le_bytes(header), 0);
    }

    #[test]
    fn a_write_reaches_the_function_and_a_debug_write_does_not() {
        let (bus, ecam) = fabric();
        let stub = Stub::new(0x8086, 0x29b0);
        bus.attach(Bdf::default(), Arc::clone(&stub) as Arc<dyn PciFunction>)
            .expect("empty");
        ecam.write(u64::from(config::COMMAND), &[0x07, 0x00], MemAttrs::DEFAULT)
            .expect("ok");
        let mut cmd = [0u8; 2];
        ecam.read(u64::from(config::COMMAND), &mut cmd, MemAttrs::DEFAULT)
            .expect("ok");
        assert_eq!(u16::from_le_bytes(cmd), 0x0007);

        let debug = MemAttrs {
            debug: true,
            ..MemAttrs::DEFAULT
        };
        assert!(
            ecam.write(u64::from(config::COMMAND), &[0x00, 0x00], debug)
                .is_err()
        );
        ecam.read(u64::from(config::COMMAND), &mut cmd, MemAttrs::DEFAULT)
            .expect("ok");
        assert_eq!(u16::from_le_bytes(cmd), 0x0007, "a debug write changed it");
    }

    #[test]
    fn a_debug_read_is_allowed_because_there_is_no_latch_to_disturb() {
        let (bus, ecam) = fabric();
        bus.attach(Bdf::default(), Stub::new(0x8086, 0x29b0))
            .expect("empty");
        let mut id = [0u8; 4];
        ecam.read(
            0,
            &mut id,
            MemAttrs {
                debug: true,
                ..MemAttrs::DEFAULT
            },
        )
        .expect("ok");
        assert_eq!(u32::from_le_bytes(id), 0x29b0_8086);
    }

    #[test]
    fn the_three_window_lengths_are_the_datasheets() {
        assert_eq!(window_len(0), Some(256 * 1024 * 1024));
        assert_eq!(window_len(1), Some(128 * 1024 * 1024));
        assert_eq!(window_len(2), Some(64 * 1024 * 1024));
        assert_eq!(window_len(3), None, "11 is reserved");
        // And each window is exactly the buses it claims to cover.
        assert_eq!(WINDOW_LENGTHS[0] / BUS_STRIDE, 256);
        assert_eq!(WINDOW_LENGTHS[1] / BUS_STRIDE, 128);
        assert_eq!(WINDOW_LENGTHS[2] / BUS_STRIDE, 64);
    }

    #[test]
    fn an_access_that_straddles_the_header_is_refused() {
        let (_bus, ecam) = fabric();
        let mut wide = [0u8; 4];
        assert!(ecam.read(0xfe, &mut wide, MemAttrs::DEFAULT).is_err());
        assert!(
            ecam.read(FUNCTION_STRIDE - 2, &mut wide, MemAttrs::DEFAULT)
                .is_err()
        );
    }
}
