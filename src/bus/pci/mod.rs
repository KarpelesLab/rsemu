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
//! [`Bars`] is the fifth, and it belongs to the function rather than to the
//! fabric for the reason above: the six base address registers and the
//! expansion ROM register are configuration space, but the windows they name
//! are ordinary mappings in an ordinary address space, and a bridge never sees
//! them. It is where the interesting problem lives — a BAR is a mapping that
//! *moves*, from inside a configuration write — and [`bar`]'s module docs carry
//! that argument.
//!
//! [`Intx`] is the sixth: a function's `INTA#`-`INTD#` pin. It is split across
//! three objects because the hardware is — the function owns the pin, the
//! fabric owns the four shared nets and the [`swizzle`] that says which one a
//! pin reaches, and the board owns what those nets are connected to. [`Intx`]'s
//! own documentation argues each third, and the argument matters: put the whole
//! thing in the function and the swizzle has nowhere to read a device number
//! from; put it in the board's wire graph and every machine file gets to spell
//! the rotation out for itself, wrongly.
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
//! * **I/O BARs that decode.** The register is complete and firmware can size
//!   and place one; mapping it is refused, because a configuration cycle
//!   travels through the I/O space and so the try-lock that saves every other
//!   case cannot help. [`bar`]'s module docs spell it out.
//! * **A board whose traces are not the standard rotation.** [`swizzle`] is the
//!   PCI-to-PCI Bridge specification's, applied to every device number on the
//!   bus. A board that wired its slots differently — and a real one may — has
//!   no way to say so yet.
//! * **Message-signalled interrupts.** No function in this tree has the
//!   capability, and `MSI` is a memory write rather than a pin, so it belongs
//!   to whichever function grows one first rather than here.
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
//!   configuration space and §6.2 for the Type 00h header's fields, with
//!   §6.2.2 for the Command register's space-enable bits, §6.2.4 for the
//!   Interrupt Line and Interrupt Pin registers, and §6.2.5.1 and §6.2.5.2 for
//!   the base address and expansion ROM registers; §2.2.6 for the `INTA#`-
//!   `INTD#` pins themselves, which is where level-sensitive and open-drain
//!   come from; §3.7.4.1 for Configuration Mechanism #1, the `0xcf8`/`0xcfc`
//!   pair; Appendix D for the class codes.
//! * *PCI-to-PCI Bridge Architecture Specification, Revision 1.1* — §9.1 and
//!   Table 9-1 for the interrupt swizzle.
//! * *Intel 440FX PCIset: 82441FX PCI and Memory Controller (PMC) and 82442FX
//!   Data Bus Accelerator (DBX)*, order number 290549-001 — §3.1.1 and §3.1.2
//!   for `CONFADD` and `CONFDATA` as an actual host bridge implements them, and
//!   Table 1 for the header offsets a host bridge fills in.
//!
//! No emulator source was consulted for any of it (`CLAUDE.md`, provenance).

pub mod bar;

#[cfg(test)]
mod tests;

pub use bar::{Bar, BarKind, Bars};

use alloc::collections::{BTreeMap, BTreeSet};
use alloc::format;
use alloc::string::String;
use alloc::sync::{Arc, Weak};
use alloc::vec::Vec;
use core::fmt;
use core::sync::atomic::{AtomicBool, Ordering};

use crate::core::error::{BusError, Error, Result};
use crate::core::space::{AccessConstraints, MemAttrs, MemOps, MemResult};
use crate::core::sync::{LockRank, Mutex};
use crate::core::value::{Endian, Width};
use crate::core::wire::{Level, WireSource};

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
/// The routing table is *read and released* before the function is called —
/// never held across the call — which is the re-entrancy contract written as
/// code: a configuration write may reach a device that retopologises, and
/// `TOPOLOGY` sits above everything here.
///
/// The lock is at [`LockRank::DEVICE`], **not** [`LockRank::BUS`], and the
/// difference is not cosmetic. `space.rs` states the invariant this obeys: *"A
/// CPU holds a `BUS`-ranked lock across the accesses it issues."* Every
/// configuration cycle arrives from inside one — a guest `IN` on `0xcfc`
/// reaches [`ConfigPorts`] through the address space with the core's execution
/// mutex ([`LockRank::BUS`]) already held — so a `BUS`-ranked table here is
/// unlockable by construction, and panics with `acquiring BUS while holding
/// BUS` the first time real firmware enumerates the bus. `DEVICE` sits below
/// `BUS` and above the `LEAF` locks each function holds, so the ladder runs the
/// one direction calls travel.
pub struct PciBus {
    functions: Mutex<BTreeMap<Bdf, Arc<dyn PciFunction>>>,
    intx: Mutex<IntxNets>,
}

impl fmt::Debug for PciBus {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let mut s = f.debug_struct("PciBus");
        match self.functions.try_lock() {
            Some(map) => s.field("functions", &map.len()),
            None => s.field("functions", &"<in use>"),
        };
        match self.intx.try_lock() {
            Some(nets) => s.field("intx", &nets.asserting.len()),
            None => s.field("intx", &"<in use>"),
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
            functions: Mutex::with_rank(LockRank::DEVICE, BTreeMap::new()),
            intx: Mutex::with_rank(INTX_RANK, IntxNets::default()),
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
    ///
    /// A function that goes away stops driving its interrupt pin, which is what
    /// [`release_intx`](PciBus::release_intx) is called for here: an open-drain
    /// net that kept a departed card's assertion would leave the line down for
    /// ever, and there would be nothing left to lift it.
    pub fn detach(&self, at: Bdf) -> bool {
        let gone = self.functions.lock().remove(&at).is_some();
        if gone {
            self.release_intx(at);
        }
        gone
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
// INTx#: the interrupt pins
// ---------------------------------------------------------------------------

/// How many interrupt nets a PCI bus has: `INTA#`, `INTB#`, `INTC#`, `INTD#`.
///
/// *PCI Local Bus Specification* Rev 2.1 §2.2.6, which also states the two
/// facts that decide everything else in this section: the pins are **level
/// sensitive, asserted low**, and they are **open drain** — so several
/// functions may share one net, and the net stays asserted until the last of
/// them lets go.
pub const INTX_LINES: u8 = 4;

/// Which interrupt pin a function drives: the value of its Interrupt Pin
/// register.
///
/// An extensible enumeration in the `pktkit` style (`CLAUDE.md`) rather than a
/// Rust `enum`, because it *is* a hardware register byte — Rev 2.1 §6.2.4 gives
/// 0 for a function that has no interrupt and 1-4 for `INTA#`-`INTD#`, and
/// leaves the rest undefined rather than illegal. Eight bits rather than the
/// convention's `u16` because the register is eight bits.
#[repr(transparent)]
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash, Default)]
pub struct IntxPin(pub u8);

impl IntxPin {
    /// This function has no interrupt pin (§6.2.4).
    pub const NONE: IntxPin = IntxPin(0);
    /// `INTA#`, which is the only pin a single-function device may use
    /// (§2.2.6).
    pub const A: IntxPin = IntxPin(1);
    /// `INTB#`.
    pub const B: IntxPin = IntxPin(2);
    /// `INTC#`.
    pub const C: IntxPin = IntxPin(3);
    /// `INTD#`.
    pub const D: IntxPin = IntxPin(4);

    /// The pin as an index 0-3, or `None` for [`IntxPin::NONE`] and for a value
    /// no specification defines.
    #[must_use]
    pub const fn index(self) -> Option<u8> {
        match self.0 {
            1..=4 => Some(self.0 - 1),
            _ => None,
        }
    }
}

/// Which bus interrupt net a function's pin lands on: **the swizzle**.
///
/// *PCI-to-PCI Bridge Architecture Specification* Revision 1.1 §9.1 and its
/// Table 9-1: the interrupt pins of the devices behind a bridge are rotated by
/// device number on the way to the bridge's own, so device 0's `INTA#` is the
/// bridge's `INTA#`, device 1's `INTA#` is its `INTB#`, and so on modulo four.
/// A system board applies the same rotation to the connectors on its root bus,
/// which is why four cards that each drive nothing but their own `INTA#` — and
/// §2.2.6 says a single-function device may drive nothing else — still arrive
/// on four different router inputs instead of piling onto one.
///
/// `None` for a function that drives no pin.
///
/// The rotation is exact rather than approximate: a device number is five bits
/// and 32 is a multiple of four, so `device % 4` is well defined however the
/// number was assigned.
#[must_use]
pub fn swizzle(at: Bdf, pin: IntxPin) -> Option<u8> {
    let index = pin.index()?;
    Some(at.device.wrapping_add(index) % INTX_LINES)
}

/// What the four bus interrupt nets are connected to.
///
/// One object rather than four wires, and the level is *given* rather than
/// asked for: a sink may not invent a level for an input pin, and a shared
/// open-drain net has no level of its own until every driver on it has been
/// counted. [`PciBus`] does that counting and hands over the result.
pub trait IntxSink: fmt::Debug + Send + Sync {
    /// Net `line` — 0 for `INTA#` through 3 for `INTD#` — is now at `level`.
    ///
    /// Called with no fabric lock held, so an implementor may do anything from
    /// inside it, including driving a wire that re-enters another function on
    /// this same bus.
    fn intx_changed(&self, line: u8, level: Level);
}

/// Where the interrupt nets' state sits in the ranked lock order.
///
/// **Below [`LockRank::BUS`] and above [`LockRank::DEVICE`]**, and forced
/// rather than chosen, for the reason the APIC bus roster's own rank gives:
/// `src/core/space.rs` states that a CPU holds a `BUS`-ranked lock across the
/// accesses it issues, so anything a guest access can reach must rank under
/// `BUS` — and a function raises its interrupt from inside a register write.
/// It ranks *above* `DEVICE` because delivery re-enters peers: the asserting
/// function has released its own state lock by then, and the sink takes its own
/// inside [`IntxSink::intx_changed`].
///
/// A number distinct from the APIC roster's `0x4c60` and the drive bays'
/// `0x4c41` so that a board holding two of them gets a deterministic order
/// rather than a deadlock.
pub const INTX_RANK: LockRank = LockRank::new(0x4c70);

/// Which functions are pulling which net down, and who to tell.
#[derive(Debug, Default)]
struct IntxNets {
    /// The `(net, function)` pairs currently asserting.
    ///
    /// A set of *drivers* rather than a level per net, because that is the one
    /// representation in which "the line stays asserted until both functions
    /// deassert" is true by construction instead of by careful bookkeeping.
    /// Ordered, so a listing is deterministic (`CLAUDE.md`).
    asserting: BTreeSet<(u8, Bdf)>,
    /// The south bridge, weakly: the fabric outlives it, and a strong handle
    /// would be the second half of a cycle.
    sink: Option<Weak<dyn IntxSink>>,
}

impl IntxNets {
    /// Whether anything is pulling `line` down.
    fn level(&self, line: u8) -> Level {
        let low = (line, Bdf::default());
        let high = (line.saturating_add(1), Bdf::default());
        Level::from_bool(self.asserting.range(low..high).next().is_some())
    }
}

impl PciBus {
    /// Install what the four interrupt nets reach, and tell it where they are.
    ///
    /// Weak, and the caller keeps the sink alive — a south bridge is owned by
    /// the machine, and the fabric owns the functions that drive it, so a
    /// strong handle here would close `fabric → sink → fabric`.
    ///
    /// The announcement is not a nicety. `ROADMAP.md` §4.3 requires realize to
    /// sweep the graph, "or interrupt lines come up wrong on some machines and
    /// only on some paths"; a sink installed after a function has already
    /// asserted would otherwise never hear about it.
    ///
    /// One sink, and the last registration wins — a fabric has one place its
    /// interrupt nets terminate, as a board has one south bridge.
    pub fn set_intx_sink(&self, sink: Weak<dyn IntxSink>) {
        let levels = {
            let mut nets = self.intx.lock();
            nets.sink = Some(sink);
            let mut levels = [Level::Low; INTX_LINES as usize];
            for (line, slot) in levels.iter_mut().enumerate() {
                *slot = nets.level(line as u8);
            }
            levels
        };
        // Outside the lock, because the sink drives wires from inside this.
        self.announce_intx(&levels);
    }

    /// The level net `line` is at.
    #[must_use]
    pub fn intx_level(&self, line: u8) -> Level {
        self.intx.lock().level(line)
    }

    /// Every function currently asserting `line`, in address order.
    ///
    /// For a monitor and for a test that wants to say *which* two functions are
    /// sharing a line rather than only that one of them is.
    #[must_use]
    pub fn intx_drivers(&self, line: u8) -> Vec<Bdf> {
        let nets = self.intx.lock();
        let low = (line, Bdf::default());
        let high = (line.saturating_add(1), Bdf::default());
        nets.asserting.range(low..high).map(|(_, at)| *at).collect()
    }

    /// The function at `at` drives its `pin` to `level`.
    ///
    /// The swizzle happens here, which is the whole reason this is the fabric's
    /// method and not the function's: a function knows which of its own four
    /// pins it drives and cannot know its device number, because the fabric is
    /// what assigned it.
    ///
    /// A function with [`IntxPin::NONE`] is silently ignored — it has no pin to
    /// drive, and refusing would make every caller check first.
    pub fn set_intx(&self, at: Bdf, pin: IntxPin, level: Level) {
        let Some(line) = swizzle(at, pin) else {
            return;
        };
        let (changed, now, sink) = {
            let mut nets = self.intx.lock();
            let before = nets.level(line);
            if level.is_high() {
                nets.asserting.insert((line, at));
            } else {
                nets.asserting.remove(&(line, at));
            }
            let after = nets.level(line);
            (before != after, after, nets.sink.clone())
        };
        // The lock is released before the sink is called, per the re-entrancy
        // contract: `intx_changed` drives a wire, and that wire reaches an
        // interrupt controller, a processor, and on the way back possibly a
        // sibling on this very bus.
        if changed && let Some(sink) = sink.and_then(|s| s.upgrade()) {
            sink.intx_changed(line, now);
        }
    }

    /// Drop every assertion the function at `at` is making.
    ///
    /// What [`detach`](PciBus::detach) calls, and what a function calls when it
    /// is unplugged from the fabric while still asserting.
    pub fn release_intx(&self, at: Bdf) {
        let (dropped, sink) = {
            let mut nets = self.intx.lock();
            let mut dropped = Vec::new();
            for line in 0..INTX_LINES {
                let before = nets.level(line);
                if nets.asserting.remove(&(line, at)) && before != nets.level(line) {
                    dropped.push(line);
                }
            }
            (dropped, nets.sink.clone())
        };
        if dropped.is_empty() {
            return;
        }
        if let Some(sink) = sink.and_then(|s| s.upgrade()) {
            for line in dropped {
                sink.intx_changed(line, Level::Low);
            }
        }
    }

    /// Tell the sink all four levels, with no lock held.
    fn announce_intx(&self, levels: &[Level; INTX_LINES as usize]) {
        let sink = self.intx.lock().sink.clone();
        if let Some(sink) = sink.and_then(|s| s.upgrade()) {
            for (line, level) in levels.iter().enumerate() {
                sink.intx_changed(line as u8, *level);
            }
        }
    }
}

/// One function's `INTx#` pin.
///
/// # Where the pin lives, and why it is here
///
/// Three objects each hold the part of this they can actually know, which is
/// also how the hardware divides it:
///
/// * **The function** owns the pin. Rev 2.1 §6.2.4 puts the Interrupt Pin
///   register in configuration space, per function, so the function is the only
///   thing that can honestly answer *which* pin it drives — and it is the only
///   thing that knows when its own condition is asserted.
/// * **The fabric** owns the net. `INTA#` is not a wire from one function to
///   one router input: it is one of four nets shared by every device on the
///   bus, and which net a function reaches depends on its **device number**,
///   which the fabric assigned and the function has never been told. So
///   [`swizzle`] is [`PciBus`]'s arithmetic, not this type's.
/// * **The board** owns what the nets reach — an [`IntxSink`] on a machine with
///   a south bridge, and on a machine without one the [`WireSource`] below,
///   which is the same pin taken straight off the card edge to an interrupt
///   controller. Both are true statements about one pin at two points on the
///   board, which is why this object drives both and neither is a special case.
///
/// The alternative — a wire per function drawn in the machine file — puts the
/// swizzle in the board author's arithmetic, where it can be silently wrong and
/// where the ACPI `_PRT` generator cannot read it back.
///
/// # Level, not edge
///
/// [`set`](Intx::set) takes the level the function's condition is *currently*
/// at and is called every time that condition is re-derived, exactly like the
/// [`WireSource`] it replaces. It never pulses: §2.2.6's pin is a level, and a
/// device that pulsed it would work until two devices shared a net.
pub struct Intx {
    /// Which of the four pins this function drives. Fixed at construction, as
    /// the read-only register it mirrors is.
    pin: IntxPin,
    /// The fabric and the address this function answers at, once it has been
    /// [`plug`](Intx::plug)ged in. **Weak**: the fabric holds every function on
    /// it, and this is reachable from one.
    plug: Mutex<Option<(Weak<PciBus>, Bdf)>>,
    /// The same pin brought out to the board as an ordinary wire, for a machine
    /// with no interrupt router.
    wire: Mutex<Option<WireSource>>,
    /// The level the pin is being held at, so a `Status` register's Interrupt
    /// Status bit and a monitor can read it without disturbing anything.
    level: AtomicBool,
}

impl fmt::Debug for Intx {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("Intx")
            .field("pin", &self.pin)
            .field("level", &self.level())
            .field("plugged", &self.plug.try_lock().map(|p| p.is_some()))
            .finish()
    }
}

impl Intx {
    /// A pin, driving nothing yet.
    ///
    /// Both mutexes are at [`LockRank::WIRE`], which is what they are: each is
    /// read, cloned out and released before anything is driven, so nothing here
    /// is ever held across a call into another device.
    #[must_use]
    pub fn new(pin: IntxPin) -> Intx {
        Intx {
            pin,
            plug: Mutex::with_rank(LockRank::WIRE, None),
            wire: Mutex::with_rank(LockRank::WIRE, None),
            level: AtomicBool::new(false),
        }
    }

    /// Which pin this is.
    #[must_use]
    pub fn pin(&self) -> IntxPin {
        self.pin
    }

    /// The level the pin is being held at.
    #[must_use]
    pub fn level(&self) -> Level {
        Level::from_bool(self.level.load(Ordering::Relaxed))
    }

    /// Plug the pin into `bus` at `at`, and drive the net with whatever level
    /// it is already at.
    ///
    /// Called from `Device::realize` beside [`PciBus::attach`], because a
    /// function that is not on the fabric has no device number and therefore no
    /// net.
    pub fn plug(&self, bus: &Arc<PciBus>, at: Bdf) {
        *self.plug.lock() = Some((Arc::downgrade(bus), at));
        bus.set_intx(at, self.pin, self.level());
    }

    /// Unplug the pin, releasing the net first.
    pub fn unplug(&self) {
        let plug = self.plug.lock().take();
        if let Some((bus, at)) = plug
            && let Some(bus) = bus.upgrade()
        {
            bus.release_intx(at);
        }
    }

    /// Bring the pin out to the board as a wire as well.
    pub fn connect(&self, source: WireSource) {
        *self.wire.lock() = Some(source);
        self.drive();
    }

    /// The pin's condition is now `level`.
    pub fn set(&self, level: Level) {
        self.level.store(level.is_high(), Ordering::Relaxed);
        self.drive();
    }

    /// Put both destinations where [`level`](Intx::level) says, with no lock
    /// held while either is called.
    fn drive(&self) {
        let level = self.level();
        let plug = self.plug.lock().clone();
        let wire = self.wire.lock().clone();
        if let Some((bus, at)) = plug
            && let Some(bus) = bus.upgrade()
        {
            bus.set_intx(at, self.pin, level);
        }
        if let Some(wire) = wire {
            wire.set(level);
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
///
/// # The fabric handle is **weak**, and that is not an optimisation
///
/// A host bridge is itself a function on the bus it bridges, so the natural
/// filing — put the ports in the bridge's register file, where the rest of its
/// registers are — closes a reference cycle: the fabric holds every function
/// strongly, the register file would hold the ports, and the ports would hold
/// the fabric. Nothing collects that, and the whole machine leaks. It has been
/// found twice by LeakSanitizer under two different fuzz targets, once in
/// [`crate::dev::q35::mch`] and once in [`crate::dev::pc::pmc`], and each was
/// fixed by moving the ports out to the `Device` — a fix that has to be
/// remembered every time someone writes a third host bridge.
///
/// So the weak handle is here instead, where it is remembered once. **The
/// owner must keep the fabric alive**, which every caller does anyway: a build
/// files its fabrics in its [`HostObjects`](crate::core::hosts::HostObjects)
/// under [`buses`], and a host bridge device holds its own `Arc<PciBus>`. With
/// the fabric gone the ports decode nothing — a read is all ones, which is what
/// a master abort gives, and a write goes nowhere.
#[derive(Debug)]
pub struct ConfigPorts {
    bus: Weak<PciBus>,
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
    ///
    /// The handle taken is **weak**; see the type's own documentation for why,
    /// and note the consequence — a caller that hands over the only strong
    /// reference gets ports that decode nothing.
    #[must_use]
    pub fn new(bus: Arc<PciBus>) -> ConfigPorts {
        ConfigPorts {
            bus: Arc::downgrade(&bus),
            address: Mutex::with_rank(LockRank::LEAF, 0),
            passthrough: Mutex::with_rank(LockRank::LEAF, None),
        }
    }

    /// The fabric these ports reach, if it still exists.
    #[must_use]
    pub fn bus(&self) -> Option<Arc<PciBus>> {
        self.bus.upgrade()
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
        match (self.target(offset - 4), self.bus()) {
            // No enabled cycle, or no fabric left to run one on: the ports are
            // just I/O space with nothing behind them, which reads as ones.
            (Some((bdf, register)), Some(bus)) => bus.config_read(bdf, register, dst, attrs),
            _ => dst.fill(0xff),
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
        if let Some((bdf, register)) = self.target(offset - 4)
            && let Some(bus) = self.bus()
        {
            bus.config_write(bdf, register, src, attrs);
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
    /// Expansion ROM Base Address. 32 bits (Rev 2.1 §6.2.5.2).
    pub const EXPANSION_ROM: u16 = 0x30;
    /// Interrupt Line. 8 bits; which input of the interrupt controller this
    /// function's pin was routed to. Firmware writes it, hardware ignores it.
    pub const INTERRUPT_LINE: u16 = 0x3c;
    /// Interrupt Pin. 8 bits, read-only: 0 for a function that has no
    /// interrupt, 1-4 for `INTA#`-`INTD#`.
    pub const INTERRUPT_PIN: u16 = 0x3d;

    /// `COMMAND[0]`: the function responds to I/O space accesses (§6.2.2).
    pub const COMMAND_IO: u16 = 0x0001;
    /// `COMMAND[1]`: the function responds to memory space accesses (§6.2.2).
    pub const COMMAND_MEMORY: u16 = 0x0002;
    /// `COMMAND[2]`: the function may act as a bus master (§6.2.2).
    pub const COMMAND_MASTER: u16 = 0x0004;

    /// Base class `0x06`: a bridge device (Rev 2.1 Appendix D).
    pub const CLASS_BRIDGE: u8 = 0x06;
    /// Sub-class `0x00` under [`CLASS_BRIDGE`]: a host bridge.
    pub const SUBCLASS_HOST_BRIDGE: u8 = 0x00;
    /// Base class `0x03`: a display controller (Rev 2.1 Appendix D).
    pub const CLASS_DISPLAY: u8 = 0x03;
    /// Sub-class `0x00` under [`CLASS_DISPLAY`]: VGA-compatible. With
    /// programming interface `0x00` that is class code `030000`, which is what
    /// a firmware looks for when it goes hunting for the console.
    pub const SUBCLASS_VGA: u8 = 0x00;

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
