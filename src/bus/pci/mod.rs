//! PCI: configuration space, the ports that reach it, and the fabric that
//! routes between them.
//!
//! # What a PCI fabric is, in this tree
//!
//! Three separable things, and they are three types here rather than one:
//!
//! * **[`Bdf`]** — an address on the fabric: a bus number, a device number and
//!   a function number. Everything about configuration space is keyed by one.
//! * **[`PciFunction`]** — what a device presents *to* configuration cycles.
//!   256 bytes of per-function register file, and nothing else. A function's
//!   memory and I/O windows are ordinary [`Region`](crate::core::space::Region)s
//!   that its own `Device` publishes; the fabric never sees them, exactly as a
//!   real bridge never sees anything but the address on the wires.
//! * **[`PciBus`]** — the fabric: which function answers at which [`Bdf`], and
//!   what happens when nothing does (a **master abort**, which reads as ones).
//!
//! [`ConfigPorts`] is the fourth thing and it is deliberately *not* part of the
//! fabric: it is an x86 host bridge's window onto it, and a machine with a
//! different processor reaches the same configuration space a different way.
//!
//! # Finding each other
//!
//! As in [`crate::bus::spi`] and [`crate::bus::i2c`]: a host bridge and the
//! functions on its bus are separate objects in a machine description, there is
//! no `core::bus` yet (`ROADMAP.md` §4), and a machine file can hand two
//! independently constructed devices only a *name*. So they meet through
//! [`buses`], a named rendezvous table, and both ends say `bus = "pci0"`.
//!
//! # What is deliberately not here yet
//!
//! * **Base address registers.** A BAR is a mapping that moves, and moving one
//!   is a retopology performed from inside a config write — the case
//!   `core::space`'s module docs open with. The mechanism exists
//!   ([`remap`](crate::core::space::TopologyGuard::remap)); no function in this
//!   tree has a BAR yet, so there is nothing to test one against and it is not
//!   written.
//! * **Interrupt routing.** `INTA#`-`INTD#` and the `PIRQ` swizzle belong to a
//!   south bridge, and there is not one.
//! * **Type 1 cycles and PCI-to-PCI bridges.** [`Bdf`] carries a bus number so
//!   that a second bus is expressible, but nothing forwards a cycle to one and
//!   so nothing here pretends to.
//! * **Extended (4 KiB) configuration space.** Only PCI Express has it, and
//!   only through a memory-mapped mechanism this module does not implement.
//!   [`CONFIG_SPACE_LEN`] is 256 bytes, which is all [`ConfigPorts`] can
//!   address.
//!
//! # Sources
//!
//! * *PCI Local Bus Specification, Revision 2.1* — §6.1 for the layout of
//!   configuration space and §6.2 for the Type 00h header's fields; §3.7.4.1
//!   for Configuration Mechanism #1, the `0xcf8`/`0xcfc` pair.
//! * *Intel 440FX PCIset: 82441FX PCI and Memory Controller (PMC) and 82442FX
//!   Data Bus Accelerator (DBX)*, order number 290549-001 — §3.1.1 and §3.1.2
//!   for `CONFADD` and `CONFDATA` as an actual host bridge implements them, and
//!   Table 1 for the header offsets a host bridge fills in.
//!
//! No emulator source was consulted for any of it (`CLAUDE.md`, provenance).

#[cfg(test)]
mod tests;

use alloc::collections::BTreeMap;
use alloc::format;
use alloc::string::String;
use alloc::sync::Arc;
use alloc::vec::Vec;
use core::fmt;

use crate::core::error::{BusError, Error, Result};
use crate::core::space::{AccessConstraints, MemAttrs, MemOps, MemResult};
use crate::core::sync::{LockRank, Mutex};
use crate::core::value::{Endian, Width};

// ---------------------------------------------------------------------------
// addressing
// ---------------------------------------------------------------------------

/// How many bytes of configuration space one function has.
///
/// 256, and no more: *PCI Local Bus Specification* Rev 2.1 §6.1. The 4 KiB
/// extended space is a PCI Express addition reached by a memory-mapped
/// mechanism this module does not implement.
pub const CONFIG_SPACE_LEN: u16 = 0x100;

/// The highest device number one bus can carry.
///
/// Five bits in the address, so 32 device numbers — Rev 2.1 §3.7.4.1's
/// `CONFIG_ADDRESS` bits 15:11.
pub const MAX_DEVICE: u8 = 31;

/// The highest function number one device can carry.
///
/// Three bits, so eight functions (Rev 2.1 §3.7.4.1, bits 10:8).
pub const MAX_FUNCTION: u8 = 7;

/// An address on the fabric: bus, device, function.
///
/// `Ord`, and ordered bus-then-device-then-function, because [`PciBus`] keys a
/// [`BTreeMap`] by it and enumeration order is guest-visible (`CLAUDE.md`,
/// determinism).
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash, Default)]
pub struct Bdf {
    /// The bus number. 0 is the one a host bridge sits on.
    pub bus: u8,
    /// The device number, 0-[`MAX_DEVICE`].
    pub device: u8,
    /// The function number, 0-[`MAX_FUNCTION`].
    pub function: u8,
}

impl Bdf {
    /// An address, refusing a device or function number that does not fit.
    ///
    /// # Errors
    ///
    /// [`Error::Config`] if `device` exceeds [`MAX_DEVICE`] or `function`
    /// exceeds [`MAX_FUNCTION`]. Silently masking either would put a device at
    /// an address nobody asked for, which is the kind of bug that shows up only
    /// as firmware finding the wrong chip.
    pub fn new(bus: u8, device: u8, function: u8) -> Result<Bdf> {
        if device > MAX_DEVICE || function > MAX_FUNCTION {
            return Err(Error::Config {
                at: format!("{bus:02x}:{device:02x}.{function}"),
                message: format!(
                    "a PCI bus carries device numbers 0-{MAX_DEVICE} and function \
                     numbers 0-{MAX_FUNCTION}"
                ),
            });
        }
        Ok(Bdf {
            bus,
            device,
            function,
        })
    }
}

impl fmt::Display for Bdf {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{:02x}:{:02x}.{}", self.bus, self.device, self.function)
    }
}

// ---------------------------------------------------------------------------
// the device seam
// ---------------------------------------------------------------------------

/// What a device presents to configuration cycles.
///
/// Byte-addressed with a slice, the same shape as
/// [`MemOps`], because a configuration access asks
/// exactly the same three questions — where, how wide, and is this a debugger.
///
/// `attrs` is not decoration: `MemAttrs::debug` reaches a function through here
/// and a debugger reading a status register must not clear it (`CLAUDE.md`,
/// devices).
pub trait PciFunction: fmt::Debug + Send + Sync {
    /// Answer a configuration read of `dst.len()` bytes at `offset`.
    ///
    /// `offset + dst.len()` is guaranteed by the caller to be within
    /// [`CONFIG_SPACE_LEN`]. A register a function does not implement reads as
    /// zero, not as ones: ones is what a *master abort* means, and a function
    /// that answered ones for its own reserved registers would be
    /// indistinguishable from one that is not there (Rev 2.1 §6.1).
    fn config_read(&self, offset: u16, dst: &mut [u8], attrs: MemAttrs);

    /// Take a configuration write of `src.len()` bytes at `offset`.
    ///
    /// Bounded as [`config_read`](PciFunction::config_read) is. A write to a
    /// read-only register is dropped, never faulted — there is no way to signal
    /// a fault on a configuration cycle, and firmware writes read-only
    /// registers all the time while sizing them.
    fn config_write(&self, offset: u16, src: &[u8], attrs: MemAttrs);
}

// ---------------------------------------------------------------------------
// the fabric
// ---------------------------------------------------------------------------

/// A PCI fabric: which function answers at which address.
///
/// The lock is at [`LockRank::BUS`], which is what makes the ordering work: a
/// configuration write may reach a device that retopologises, and `TOPOLOGY`
/// sits above `BUS`. So the routing table is *read and released* before the
/// function is called — never held across the call — which is the re-entrancy
/// contract written as code.
pub struct PciBus {
    functions: Mutex<BTreeMap<Bdf, Arc<dyn PciFunction>>>,
}

impl fmt::Debug for PciBus {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let mut s = f.debug_struct("PciBus");
        match self.functions.try_lock() {
            Some(map) => s.field("functions", &map.len()),
            None => s.field("functions", &"<in use>"),
        };
        s.finish()
    }
}

impl Default for PciBus {
    fn default() -> PciBus {
        PciBus::new()
    }
}

impl PciBus {
    /// A fabric with nothing on it.
    #[must_use]
    pub fn new() -> PciBus {
        PciBus {
            functions: Mutex::with_rank(LockRank::BUS, BTreeMap::new()),
        }
    }

    /// Put `function` at `at`.
    ///
    /// # Errors
    ///
    /// [`Error::Config`] if something already answers there. Two devices at one
    /// address is a machine-description bug, and the second one silently
    /// winning would be a machine nobody could debug.
    pub fn attach(&self, at: Bdf, function: Arc<dyn PciFunction>) -> Result<()> {
        let mut map = self.functions.lock();
        if map.contains_key(&at) {
            return Err(Error::Config {
                at: format!("{at}"),
                message: String::from("two PCI functions cannot share one address"),
            });
        }
        map.insert(at, function);
        Ok(())
    }

    /// Forget whatever is at `at`, reporting whether there was anything.
    pub fn detach(&self, at: Bdf) -> bool {
        self.functions.lock().remove(&at).is_some()
    }

    /// The function at `at`, if there is one.
    ///
    /// Clones the `Arc` out and releases the lock, so the caller may do
    /// anything it likes with the result — including a retopology.
    #[must_use]
    pub fn function(&self, at: Bdf) -> Option<Arc<dyn PciFunction>> {
        self.functions.lock().get(&at).cloned()
    }

    /// Every address that answers, in address order.
    #[must_use]
    pub fn addresses(&self) -> Vec<Bdf> {
        self.functions.lock().keys().copied().collect()
    }

    /// A configuration read, with a **master abort** where nothing answers.
    ///
    /// Rev 2.1 §3.7.4.1: a configuration read that is not claimed terminates in
    /// a master abort, and the host bridge returns all ones. That is precisely
    /// how firmware discovers an empty slot, so it is the interesting case
    /// rather than an error path.
    pub fn config_read(&self, at: Bdf, offset: u16, dst: &mut [u8], attrs: MemAttrs) {
        match self.function(at) {
            Some(f) => f.config_read(offset, dst, attrs),
            None => dst.fill(0xff),
        }
    }

    /// A configuration write, dropped where nothing answers.
    pub fn config_write(&self, at: Bdf, offset: u16, src: &[u8], attrs: MemAttrs) {
        if let Some(f) = self.function(at) {
            f.config_write(offset, src, attrs);
        }
    }
}

// ---------------------------------------------------------------------------
// configuration mechanism #1
// ---------------------------------------------------------------------------

/// How much I/O space [`ConfigPorts`] decodes: `0xcf8`-`0xcff`.
pub const CONFIG_PORT_WINDOW_LEN: u64 = 8;

/// `CONFADD` bit 31: a configuration cycle only happens when this is set.
const CONFIG_ENABLE: u32 = 0x8000_0000;

/// The bits of `CONFADD` a write keeps.
///
/// Bits 30:24 and 1:0 are reserved (82441FX §3.1.1) and read back as zero, so
/// firmware that writes `0x8000_0000 | (reg & 0xfc)` reads back what it wrote.
const CONFADD_MASK: u32 = CONFIG_ENABLE | 0x00ff_fffc;

/// The `0xcf8`/`0xcfc` port pair: Configuration Mechanism #1.
///
/// One eight-byte window, because that is one decode on the board: `CONFADD` at
/// `0xcf8`-`0xcfb` and `CONFDATA` at `0xcfc`-`0xcff`.
///
/// # The two rules that are easy to get wrong
///
/// * **`CONFADD` is Dword-only.** 82441FX §3.1.1: "CONFADD is a 32-bit register
///   accessed only when referenced as a Dword. A Byte or Word reference will
///   'pass through' the Configuration Address Register to the PCI Bus." So a
///   narrow access does not touch the latch, and goes instead to
///   [`set_passthrough`](ConfigPorts::set_passthrough) — the south bridge's
///   reset control register at `0xcf9` lives exactly there. With nothing
///   installed a narrow read gives ones and a narrow write goes nowhere, which
///   is an unclaimed I/O cycle.
/// * **`CONFDATA` is not.** A byte or word access anywhere in `0xcfc`-`0xcff`
///   is a configuration access to the corresponding bytes of the addressed
///   Dword: the register number comes from `CONFADD[7:2]` and the low two bits
///   of the *I/O address* select which bytes inside it. Firmware reads a vendor
///   ID as a word at `0xcfc` and a header type as a byte at `0xcfe`, so a model
///   that answered only Dwords would fail immediately.
#[derive(Debug)]
pub struct ConfigPorts {
    bus: Arc<PciBus>,
    /// The `CONFADD` latch. At [`LockRank::LEAF`]: it is read and released
    /// before the fabric is touched, so nothing is held across the call into a
    /// function.
    address: Mutex<u32>,
    /// What a byte or word reference inside `CONFADD` passes through to, as
    /// four bytes addressed 0-3. `None` means nothing claims those cycles.
    /// Also at [`LockRank::LEAF`], and cloned out before it is called.
    passthrough: Mutex<Option<Arc<dyn MemOps>>>,
}

impl ConfigPorts {
    /// The port pair onto `bus`.
    #[must_use]
    pub fn new(bus: Arc<PciBus>) -> ConfigPorts {
        ConfigPorts {
            bus,
            address: Mutex::with_rank(LockRank::LEAF, 0),
            passthrough: Mutex::with_rank(LockRank::LEAF, None),
        }
    }

    /// The fabric these ports reach.
    #[must_use]
    pub fn bus(&self) -> &Arc<PciBus> {
        &self.bus
    }

    /// Install what a byte or word reference inside `CONFADD` reaches.
    ///
    /// `ops` is addressed 0-3, the same offsets as `CONFADD`'s own four bytes,
    /// so a chip decoding `0xcf9` answers at offset 1.
    ///
    /// # Why this exists
    ///
    /// Because a real chipset's decode includes the byte enables and an
    /// [`AddressSpace`](crate::core::space::AddressSpace) decodes by address
    /// alone. `CONFADD` occupies `0xcf8`-`0xcfb` and is claimed only by a Dword
    /// access; the reset control register at `0xcf9` is claimed only by a byte
    /// access, and on a real board it is in a different chip. Both have to
    /// work, so the owner that needs all four bytes holds them and hands the
    /// rest on — which is 82441FX §3.1.1's own word for it, "pass through […]
    /// to the PCI Bus", and the south bridge is on that bus.
    pub fn set_passthrough(&self, ops: Arc<dyn MemOps>) {
        *self.passthrough.lock() = Some(ops);
    }

    /// What [`set_passthrough`](ConfigPorts::set_passthrough) installed.
    #[must_use]
    pub fn passthrough(&self) -> Option<Arc<dyn MemOps>> {
        self.passthrough.lock().clone()
    }

    /// The current `CONFADD` latch, for a snapshot.
    #[must_use]
    pub fn address(&self) -> u32 {
        *self.address.lock()
    }

    /// Restore the `CONFADD` latch from a snapshot.
    ///
    /// Masked exactly as a guest write is, so a corrupt or hand-written
    /// snapshot cannot install reserved bits the hardware could never hold.
    pub fn set_address(&self, value: u32) {
        *self.address.lock() = value & CONFADD_MASK;
    }

    /// Clear the latch, as `PCIRST#` does.
    pub fn reset(&self) {
        *self.address.lock() = 0;
    }

    /// Where in configuration space an access `offset` bytes into `CONFDATA`
    /// lands, or `None` if no cycle happens at all.
    fn target(&self, offset: u64) -> Option<(Bdf, u16)> {
        let addr = *self.address.lock();
        if addr & CONFIG_ENABLE == 0 {
            return None;
        }
        let bdf = Bdf {
            bus: ((addr >> 16) & 0xff) as u8,
            device: ((addr >> 11) & 0x1f) as u8,
            function: ((addr >> 8) & 0x07) as u8,
        };
        // The register number is `CONFADD[7:2]`, which names a Dword; the low
        // two bits of the *I/O address* pick the bytes inside it.
        let register = ((addr & 0xfc) as u16) | (offset & 0x3) as u16;
        Some((bdf, register))
    }
}

impl MemOps for ConfigPorts {
    fn read(&self, offset: u64, dst: &mut [u8], attrs: MemAttrs) -> MemResult {
        let len = dst.len() as u64;
        if offset.saturating_add(len) > CONFIG_PORT_WINDOW_LEN {
            return Err(BusError::BadAccess);
        }
        if offset < 4 {
            // `CONFADD`. A Dword read at 0xcf8 hands back the latch; anything
            // narrower passes through, and reads as ones if nothing claims it.
            if offset == 0 && len == 4 {
                dst.copy_from_slice(&self.address().to_le_bytes());
                return Ok(());
            }
            if offset + len > 4 {
                return Err(BusError::BadAccess);
            }
            return match self.passthrough() {
                Some(ops) => ops.read(offset, dst, attrs),
                None => {
                    dst.fill(0xff);
                    Ok(())
                }
            };
        }
        // `CONFDATA`. An access straddling the end of the Dword is not one this
        // decode can express, and no instruction issues one: `in` is a single
        // aligned operand.
        if offset - 4 + len > 4 {
            return Err(BusError::BadAccess);
        }
        match self.target(offset - 4) {
            // No enabled cycle: the ports are just I/O space with nothing
            // behind them, which reads as ones.
            None => dst.fill(0xff),
            Some((bdf, register)) => self.bus.config_read(bdf, register, dst, attrs),
        }
        Ok(())
    }

    fn write(&self, offset: u64, src: &[u8], attrs: MemAttrs) -> MemResult {
        let len = src.len() as u64;
        if offset.saturating_add(len) > CONFIG_PORT_WINDOW_LEN {
            return Err(BusError::BadAccess);
        }
        if attrs.debug {
            // Neither half of this window is safe for a debugger. A write to
            // `CONFADD` moves the address latch, so the guest's next
            // `CONFDATA` access lands on a different device; a write to
            // `CONFDATA` is a configuration write, which is how a BAR moves and
            // how a chipset's shadow windows are switched. There is no
            // harmless subset to allow.
            return Err(BusError::BadAccess);
        }
        if offset < 4 {
            if offset == 0 && len == 4 {
                let value = u32::from_le_bytes([src[0], src[1], src[2], src[3]]);
                *self.address.lock() = value & CONFADD_MASK;
                return Ok(());
            }
            if offset + len > 4 {
                return Err(BusError::BadAccess);
            }
            // A narrower write passes through. With nothing installed it is an
            // ordinary unclaimed I/O cycle, which is not an error.
            return match self.passthrough() {
                Some(ops) => ops.write(offset, src, attrs),
                None => Ok(()),
            };
        }
        if offset - 4 + len > 4 {
            return Err(BusError::BadAccess);
        }
        if let Some((bdf, register)) = self.target(offset - 4) {
            self.bus.config_write(bdf, register, src, attrs);
        }
        Ok(())
    }

    fn constraints(&self) -> AccessConstraints {
        // Byte, word and Dword, little-endian, and no bulk transfer: a 64-bit
        // access to a 32-bit port pair is a decode no PC performs, and
        // accepting one would mean inventing an answer for it.
        AccessConstraints::IO
            .with_widths(Width::U8, Width::U32)
            .with_endian(Endian::Little)
    }
}

// ---------------------------------------------------------------------------
// a config register file, for the functions that want one
// ---------------------------------------------------------------------------

/// 256 bytes of configuration space with a per-byte write mask.
///
/// Every PCI function has the same problem — most of the header is read-only,
/// some of it is read/write — and solving it once here keeps a function's own
/// code about the registers that are actually its own.
///
/// The mask is per **byte**, not per bit, because a bit mask would need a
/// second array to say which bits and this has been enough for every register
/// in the tree so far. A function whose register has read-only bits inside a
/// writable byte filters in its own
/// [`config_write`](PciFunction::config_write) before calling
/// [`write`](ConfigSpace::write).
#[derive(Debug, Clone)]
pub struct ConfigSpace {
    bytes: [u8; CONFIG_SPACE_LEN as usize],
    writable: [bool; CONFIG_SPACE_LEN as usize],
}

impl Default for ConfigSpace {
    fn default() -> ConfigSpace {
        ConfigSpace::new()
    }
}

impl ConfigSpace {
    /// All zero, and entirely read-only until [`allow`](ConfigSpace::allow)
    /// says otherwise.
    #[must_use]
    pub fn new() -> ConfigSpace {
        ConfigSpace {
            bytes: [0; CONFIG_SPACE_LEN as usize],
            writable: [false; CONFIG_SPACE_LEN as usize],
        }
    }

    /// Set `len` bytes at `offset` from `value`, little-endian, ignoring the
    /// write mask.
    ///
    /// For a function filling in its own hardwired registers, which is what a
    /// datasheet's "Default Value" column is.
    pub fn hardwire(&mut self, offset: u16, value: u32, len: u16) {
        let bytes = value.to_le_bytes();
        for i in 0..len.min(4) {
            self.set_byte(offset.saturating_add(i), bytes[i as usize]);
        }
    }

    /// Make `len` bytes at `offset` writable by the guest.
    pub fn allow(&mut self, offset: u16, len: u16) {
        for i in 0..len {
            let at = offset.saturating_add(i) as usize;
            if let Some(slot) = self.writable.get_mut(at) {
                *slot = true;
            }
        }
    }

    /// One byte, whatever the mask says.
    #[must_use]
    pub fn byte(&self, offset: u16) -> u8 {
        self.bytes.get(offset as usize).copied().unwrap_or(0)
    }

    /// Set one byte, whatever the mask says.
    pub fn set_byte(&mut self, offset: u16, value: u8) {
        if let Some(slot) = self.bytes.get_mut(offset as usize) {
            *slot = value;
        }
    }

    /// Whether the guest may write the byte at `offset`.
    #[must_use]
    pub fn is_writable(&self, offset: u16) -> bool {
        self.writable.get(offset as usize).copied().unwrap_or(false)
    }

    /// Copy out `dst.len()` bytes at `offset`, zero-filling past the end.
    pub fn read(&self, offset: u16, dst: &mut [u8]) {
        for (i, slot) in dst.iter_mut().enumerate() {
            *slot = self.byte(offset.saturating_add(i as u16));
        }
    }

    /// Take a guest write, honouring the mask, and report whether anything
    /// changed.
    ///
    /// The boolean is what lets a function do work only when a register really
    /// moved: firmware rewrites the same value constantly, and re-flattening an
    /// address space for a write that changed nothing is pure cost.
    pub fn write(&mut self, offset: u16, src: &[u8]) -> bool {
        let mut changed = false;
        for (i, byte) in src.iter().enumerate() {
            let at = offset.saturating_add(i as u16) as usize;
            if at < self.bytes.len() && self.writable[at] && self.bytes[at] != *byte {
                self.bytes[at] = *byte;
                changed = true;
            }
        }
        changed
    }

    /// Every byte, for a snapshot.
    #[must_use]
    pub fn bytes(&self) -> &[u8] {
        &self.bytes
    }

    /// Restore every **writable** byte from a snapshot, leaving the hardwired
    /// ones as this build states them.
    ///
    /// A snapshot must not be able to change a device's vendor ID: those bytes
    /// are a property of the model, not of the run — the same argument that
    /// keeps derived state out of a snapshot, from the other side.
    pub fn restore(&mut self, bytes: &[u8]) {
        for (at, byte) in bytes.iter().enumerate().take(self.bytes.len()) {
            if self.writable[at] {
                self.bytes[at] = *byte;
            }
        }
    }
}

/// The Type 00h configuration header's offsets.
///
/// *PCI Local Bus Specification* Rev 2.1 §6.1, and the same table restated for
/// one real part in the 82441FX datasheet's Table 1.
pub mod config {
    /// Vendor Identification. 16 bits, read-only; `0xffff` where nothing is.
    pub const VENDOR_ID: u16 = 0x00;
    /// Device Identification. 16 bits, read-only.
    pub const DEVICE_ID: u16 = 0x02;
    /// Command. 16 bits; which cycle types the function responds to.
    pub const COMMAND: u16 = 0x04;
    /// Status. 16 bits; some read-only, some write-one-to-clear.
    pub const STATUS: u16 = 0x06;
    /// Revision Identification. 8 bits, read-only.
    pub const REVISION_ID: u16 = 0x08;
    /// Class Code. 24 bits, read-only: programming interface, sub-class and
    /// base class, in that order from the low byte.
    pub const CLASS_CODE: u16 = 0x09;
    /// Cache Line Size. 8 bits.
    pub const CACHE_LINE_SIZE: u16 = 0x0c;
    /// Master Latency Timer. 8 bits.
    pub const LATENCY_TIMER: u16 = 0x0d;
    /// Header Type. 8 bits; `0x00` is the basic format and bit 7 marks a
    /// multi-function device.
    pub const HEADER_TYPE: u16 = 0x0e;
    /// Built-In Self Test. 8 bits.
    pub const BIST: u16 = 0x0f;
    /// The first of the six Type 00h base address registers.
    pub const BAR0: u16 = 0x10;

    /// Base class `0x06`: a bridge device (Rev 2.1 Appendix D).
    pub const CLASS_BRIDGE: u8 = 0x06;
    /// Sub-class `0x00` under [`CLASS_BRIDGE`]: a host bridge.
    pub const SUBCLASS_HOST_BRIDGE: u8 = 0x00;

    /// Intel's vendor ID.
    pub const VENDOR_INTEL: u16 = 0x8086;
}

/// The named rendezvous: how a host bridge and the functions on its bus find
/// each other.
///
/// Modelled on [`crate::bus::spi::buses`] and, under it,
/// [`crate::host::chardev::ports`] — a seam for the same reason, and it becomes
/// `core::bus`'s registry when that lands.
///
/// ```
/// # #[cfg(feature = "bus-pci")] {
/// use rsemu::bus::pci::buses;
/// use rsemu::core::HostObjects;
///
/// use std::sync::Arc;
///
/// let hosts = HostObjects::new();
/// let a = buses::open(&hosts, "pci0").unwrap();
/// let b = buses::open(&hosts, "pci0").unwrap();
/// assert!(Arc::ptr_eq(&a, &b), "the same name is the same fabric");
///
/// // And a second build's `pci0` is a second fabric, not this one.
/// let elsewhere = HostObjects::new();
/// let c = buses::open(&elsewhere, "pci0").unwrap();
/// assert!(!Arc::ptr_eq(&a, &c));
/// # }
/// ```
pub mod buses {
    use super::PciBus;
    use alloc::sync::Arc;

    use crate::core::error::Result;
    use crate::core::hosts::{HostKind, HostObjects};
    use crate::core::props::Props;

    /// The kind a PCI fabric is filed under in a build's [`HostObjects`].
    pub const KIND: HostKind = HostKind::new("pci-bus");

    /// The fabric `name` refers to in `hosts`, creating it on first mention.
    ///
    /// # Errors
    ///
    /// [`crate::Error::Config`] if another kind of host object is already open
    /// under that name.
    pub fn open(hosts: &HostObjects, name: &str) -> Result<Arc<PciBus>> {
        hosts.open(KIND, name, PciBus::new)
    }

    /// The fabric `name` refers to in the build these properties are being read
    /// for, creating it on first mention.
    ///
    /// The **device** side, called from `new(props)`.
    ///
    /// # Errors
    ///
    /// As [`open`].
    pub fn attach(props: &Props, name: &str) -> Result<Arc<PciBus>> {
        props.host(KIND, name, PciBus::new)
    }

    /// The fabric called `name`, if it has been opened.
    ///
    /// # Errors
    ///
    /// As [`open`].
    pub fn get(hosts: &HostObjects, name: &str) -> Result<Option<Arc<PciBus>>> {
        hosts.get(KIND, name)
    }

    /// Forget `name`, reporting whether there was one.
    pub fn close(hosts: &HostObjects, name: &str) -> bool {
        hosts.close(KIND, name)
    }
}
