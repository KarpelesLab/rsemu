//! The processor's local APIC, and the message bus every APIC shares.
//!
//! # Sources
//!
//! * *Intel 64 and IA-32 Architectures Software Developer's Manual*, Volume
//!   3A, chapter 10, "Advanced Programmable Interrupt Controller (APIC)". The
//!   register block (§10.4.1 and Table 10-1), the local vector table (§10.5.1),
//!   the timer and its divide configuration (§10.5.4), the interrupt command
//!   register (§10.6.1), the destination modes (§10.6.2), the priority
//!   registers and the acceptance rules (§10.8.3-§10.8.4), the end-of-interrupt
//!   register (§10.8.5), the spurious vector (§10.9), the error status register
//!   (§10.5.3) and the software-disabled state (§10.4.7.2) all come from it.
//!   Each is cited on the item it justifies.
//! * *MultiProcessor Specification* v1.4, §B.4, for the INIT/Start-Up sequence
//!   an application processor is brought up with.
//!
//! **No emulator source was consulted** (`CLAUDE.md`, provenance).
//!
//! # What a local APIC is, in this framework's terms
//!
//! It is an 8259A with more registers and a message inbox. Like the 8259A it
//! drives one processor's `INTR` pin and answers the acknowledge cycle with a
//! vector — [`IntAck`], the same seam — and like the 8259A it has an
//! in-service register and an end-of-interrupt. What is new is where requests
//! come from: not eight pins, but a 256-bit request register filled by
//! *messages*, from an I/O APIC, from another processor's interrupt command
//! register, or from this APIC's own timer.
//!
//! Two of its inputs are still pins, `lint0` and `lint1`, and one of the modes
//! they can be programmed into — `ExtINT` — means "forward the acknowledge to
//! whatever is out there". That is how a PC keeps its 8259A working after the
//! APIC is switched on, and here it is [`Device::attach_int_ack`] delegation,
//! exactly as an 8259A master delegates to its slave.
//!
//! # Time
//!
//! The APIC timer counts the processor's bus clock divided by the divide
//! configuration register, so this is a **lazily advanced** device
//! (`ROADMAP.md` §4.2) counted in its own clock domain. Nothing here reads a
//! host clock, sleeps, or converts a tick to a second: [`Device::advance_to`]
//! moves the count and [`Device::next_event_tick`] tells the scheduler the tick
//! the count reaches zero on, so the interrupt is raised on that tick rather
//! than at the end of whatever quantum contained it.
//!
//! The divisor is applied as exact integer arithmetic against the domain's own
//! ticks — `remaining` below is in *bus* ticks and the guest-visible current
//! count is that divided by the divisor — which is what `CLAUDE.md`'s "no
//! floats in the time path" asks for and what keeps a divide-by-128 timer from
//! drifting against a divide-by-1 one on the same crystal.
//!
//! # What is not here
//!
//! * **A relocatable register window.** `IA32_APIC_BASE`'s base-address field
//!   is reported rather than obeyed: moving the window is an address-space
//!   retopology and a device does not get to do that to itself, so a machine
//!   file's `map` places the page. The rest of the register is live — a guest's
//!   `RDMSR`/`WRMSR` reaches it through [`LocalController`], and clearing the
//!   enable bit does make this APIC transparent.
//! * **x2APIC.** The MSR interface and the 32-bit APIC IDs are a separate mode
//!   with its own register semantics, and nothing asks for it yet.
//! * **SMI delivery.** No core here has a system management mode to enter, so
//!   an SMI message is counted and dropped rather than pretended.

use alloc::boxed::Box;
use alloc::format;
use alloc::string::{String, ToString};
use alloc::sync::{Arc, Weak};
use alloc::vec::Vec;
use core::fmt;
use core::mem;

use crate::core::device::{Device, DeviceClass, PropertySpec, RealizeCtx, ResetKind, SinkPin};
use crate::core::error::{BusError, Error, Result};
use crate::core::props::{Props, ValueKind};
use crate::core::sched::{AccessKind, LazyHandle};
use crate::core::space::{AccessConstraints, MemAttrs, MemOps, MemResult, Region, RegionRef};
use crate::core::state::{ChunkReader, ChunkWriter, Sink, Source};
use crate::core::sync::{AtomicU64, LockRank, Mutex, Ordering};
use crate::core::value::{Endian, Width};
use crate::core::wire::{
    FanIn, IntAck, IntAckCycle, IntAckResponse, Level, LocalController, Resolve, Startup, WireId,
    WireSink, WireSource,
};
use crate::machine::realize::{BindCtx, Instance};
use crate::machine::validate::ClassSchema;

pub use bus::{ApicBus, Delivery, EoiSink, Message, Shorthand, Target};

/// The class name a machine description writes.
pub const CLASS_NAME: &str = "pc.lapic";

/// The snapshot chunk version. Bump with the encoding, never on its own.
const STATE_VERSION: u32 = 2;

/// How much address space the register block answers.
///
/// One 4 KiB page, of which the top three quarters are reserved (SDM Vol 3A
/// Table 10-1).
pub const REGISTER_WINDOW_LEN: u64 = 0x1000;

/// The page the register block sits at when firmware has not moved it.
///
/// The reset value of `IA32_APIC_BASE`'s base field (SDM Vol 3A §10.4.4). A
/// machine file writes it in a `map` statement; it is published here so a board
/// and a test agree on one number.
pub const DEFAULT_BASE: u64 = 0xfee0_0000;

/// The region name a board maps to get the **architectural** page: one
/// address, decoded to whichever processor is reading.
///
/// `map mem 0xfee00000 size 0x1000 = lapic0.window` on a multiprocessor board,
/// where `lapic0.regs` would give every processor the bootstrap processor's
/// registers.
///
/// It decodes on
/// [`MemAttrs::requester`](crate::core::space::MemAttrs::requester): each
/// `pc.lapic` names its processor with `cpu = …`, resolves that to a requester
/// id when the machine binds it, and claims it on the shared APIC bus, so one
/// page reaches whichever local APIC belongs to the processor doing the
/// reading. An access from anything that is not a processor on that bus — a
/// debugger, a snapshot — reaches the APIC that publishes the page. `ApicWindow`
/// in this module is the implementation, and `docs/platforms/pc-at.md` has the
/// argument for why it is a region rather than a mapping.
pub const WINDOW_REGION: &str = "window";

// ---------------------------------------------------------------------------
// the APIC message bus
// ---------------------------------------------------------------------------

/// The interconnect an interrupt message travels: how an I/O APIC, or another
/// processor's interrupt command register, reaches a local APIC.
///
/// On a P6 this was a three-wire serial bus between the parts; from the Pentium
/// 4 on it is an ordinary write on the system bus. Either way the *message* is
/// the architecture — a vector, a delivery mode, and a destination — and that
/// is what this models.
///
/// # Why a host object and not a wire
///
/// A wire carries a level, and this carries a vector to a subset of the
/// processors chosen by a destination field the sender computes at run time. So
/// the two ends meet by **name** in the build's
/// [`HostObjects`](crate::core::hosts::HostObjects), the same rendezvous
/// [`crate::dev::ata::bays`] and `host::chardev::ports` use: every APIC in a
/// machine names the same bus (`bus = "apic"`), and whichever is constructed
/// first creates it.
pub mod bus {
    use alloc::sync::{Arc, Weak};
    use alloc::vec::Vec;
    use core::fmt;

    use crate::core::error::Result;
    use crate::core::hosts::{HostKind, HostObjects};
    use crate::core::props::Props;
    use crate::core::space::{MemOps, RequesterId};
    use crate::core::sync::{AtomicBool, LockRank, Mutex, Ordering};

    /// The kind an APIC bus is filed under in a build's `HostObjects`.
    pub const KIND: HostKind = HostKind::rendezvous("apic-bus");

    /// The bus name an APIC gets when a machine description does not say.
    pub const DEFAULT_NAME: &str = "apic";

    /// Where the bus roster's lock sits in the ranked order.
    ///
    /// **Below [`LockRank::BUS`] and above [`LockRank::DEVICE`]**, and the
    /// order is forced rather than chosen: `src/core/space.rs` states that a
    /// CPU holds a `BUS`-ranked lock across the accesses it issues, so anything
    /// a guest MMIO write can reach must rank under `BUS` — and the roster is
    /// reached from an I/O APIC's register write. It ranks *above* `DEVICE`
    /// because a sender releases its own state lock before delivering and a
    /// receiver takes its own state lock inside `accept`.
    ///
    /// The same slot [`crate::dev::ata::bays::BAY_RANK`] occupies, with a
    /// distinct number so that a board holding both gets a deterministic order
    /// rather than a deadlock.
    pub const BUS_RANK: LockRank = LockRank::new(0x4c60);

    /// A message's delivery mode: bits 8-10 of an interrupt command register or
    /// of an I/O APIC redirection entry (SDM Vol 3A §10.6.1).
    ///
    /// An extensible enumeration in the `pktkit` style (`CLAUDE.md`) rather
    /// than a Rust `enum`: the field is three bits of a hardware register, one
    /// of its eight values is reserved, and a receiver that does not recognise
    /// a mode has to be able to say so rather than fail to compile.
    #[repr(transparent)]
    #[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
    pub struct Delivery(pub u8);

    impl Delivery {
        /// Deliver the vector to every destination named.
        pub const FIXED: Delivery = Delivery(0b000);
        /// Deliver to the one destination running at the lowest priority.
        pub const LOWEST: Delivery = Delivery(0b001);
        /// A system management interrupt. The vector is ignored.
        pub const SMI: Delivery = Delivery(0b010);
        /// A non-maskable interrupt. The vector is ignored.
        pub const NMI: Delivery = Delivery(0b100);
        /// Reset the destination and leave it waiting for a Start-Up.
        pub const INIT: Delivery = Delivery(0b101);
        /// Start a waiting processor at `vector << 12`.
        pub const STARTUP: Delivery = Delivery(0b110);
        /// The destination runs an acknowledge cycle against its external
        /// 8259A-compatible controller.
        pub const EXTINT: Delivery = Delivery(0b111);
    }

    /// Which processors an interrupt command register write is addressed to:
    /// bits 18-19, the destination shorthand (SDM Vol 3A §10.6.1).
    #[derive(Debug, Clone, Copy, PartialEq, Eq)]
    pub enum Shorthand {
        /// No shorthand: the destination field decides.
        Dest,
        /// The sender itself, whatever the destination field says.
        SelfOnly,
        /// Every APIC on the bus, the sender included.
        All,
        /// Every APIC on the bus except the sender.
        AllButSelf,
    }

    impl Shorthand {
        /// The shorthand bits 18-19 name.
        #[must_use]
        pub const fn from_bits(bits: u32) -> Shorthand {
            match bits & 3 {
                1 => Shorthand::SelfOnly,
                2 => Shorthand::All,
                3 => Shorthand::AllButSelf,
                _ => Shorthand::Dest,
            }
        }

        /// The two bits this shorthand is written as.
        #[must_use]
        pub const fn bits(self) -> u32 {
            match self {
                Shorthand::Dest => 0,
                Shorthand::SelfOnly => 1,
                Shorthand::All => 2,
                Shorthand::AllButSelf => 3,
            }
        }
    }

    /// One interrupt message.
    ///
    /// The union of what an interrupt command register write and an I/O APIC
    /// redirection entry can say, because they say the same things: the two
    /// register formats differ in layout and not in content.
    #[derive(Debug, Clone, Copy, PartialEq, Eq)]
    pub struct Message {
        /// The interrupt vector. Ignored by `SMI`, `NMI` and `INIT`; the page
        /// number a Start-Up begins at, in units of 4 KiB.
        pub vector: u8,
        /// What the destination does with it.
        pub delivery: Delivery,
        /// Whether `dest` is a logical destination rather than an APIC ID.
        pub logical: bool,
        /// The destination field: an APIC ID, a logical destination bitmap, or
        /// `0xff` for a broadcast.
        pub dest: u8,
        /// Whether the source is level-triggered, which is what a receiver
        /// records in its trigger-mode register so an end-of-interrupt knows to
        /// tell the I/O APIC.
        pub level_triggered: bool,
        /// The `INIT` level: false is the de-assert half of the level-triggered
        /// `INIT` pair (SDM Vol 3A §10.6.1).
        pub assert: bool,
    }

    impl Message {
        /// A fixed-delivery message to one APIC ID.
        #[must_use]
        pub const fn fixed(vector: u8, dest: u8) -> Message {
            Message {
                vector,
                delivery: Delivery::FIXED,
                logical: false,
                dest,
                level_triggered: false,
                assert: true,
            }
        }
    }

    /// Something a message can be delivered to: one local APIC.
    ///
    /// Held weakly by the bus — the machine owns devices and an interconnect
    /// merely refers to them (`ROADMAP.md` §4.3's weak edge).
    pub trait Target: Send + Sync + fmt::Debug {
        /// This APIC's ID, as its ID register currently reads.
        fn apic_id(&self) -> u8;

        /// Whether `dest` selects this APIC in logical destination mode, which
        /// the logical destination and destination format registers decide.
        fn logical_match(&self, dest: u8) -> bool;

        /// The arbitration priority this APIC would bid with, for a
        /// lowest-priority delivery.
        fn arbitration_priority(&self) -> u8;

        /// Take the message. Called with no bus lock held, so an implementation
        /// is free to take its own state lock and to drive its own pins.
        fn accept(&self, message: Message);
    }

    /// Something that wants to hear about an end-of-interrupt: an I/O APIC,
    /// which clears the remote IRR of every level-triggered redirection entry
    /// carrying that vector (82093AA datasheet §3.2.4).
    pub trait EoiSink: Send + Sync + fmt::Debug {
        /// A processor has written its end-of-interrupt register for a
        /// level-triggered interrupt at `vector`.
        fn eoi(&self, vector: u8);
    }

    /// The interconnect itself: who is on it, and how a message finds them.
    #[derive(Default)]
    pub struct ApicBus {
        targets: Mutex<Vec<Weak<dyn Target>>>,
        eoi: Mutex<Vec<Weak<dyn EoiSink>>>,
        /// Which processor reaches which register block — the roster the
        /// per-processor window decodes through. See
        /// [`attach_local`](ApicBus::attach_local).
        locals: Mutex<Vec<(RequesterId, Weak<dyn MemOps>)>>,
        /// Whether anything on this bus has published the architectural
        /// aperture. Set when a `map` statement asks a local APIC for its
        /// `window` region, which is the only way one is reached.
        window: AtomicBool,
    }

    impl fmt::Debug for ApicBus {
        fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
            f.debug_struct("ApicBus")
                .field("targets", &self.targets.lock().len())
                .field("eoi_sinks", &self.eoi.lock().len())
                .field("locals", &self.locals.lock().len())
                .field("window", &self.window.load(Ordering::Relaxed))
                .finish()
        }
    }

    impl ApicBus {
        /// An empty bus.
        #[must_use]
        pub fn new() -> ApicBus {
            ApicBus {
                targets: Mutex::with_rank(BUS_RANK, Vec::new()),
                eoi: Mutex::with_rank(BUS_RANK, Vec::new()),
                locals: Mutex::with_rank(BUS_RANK, Vec::new()),
                window: AtomicBool::new(false),
            }
        }

        /// Say that `requester`'s accesses belong to `ops`.
        ///
        /// The half of the model that makes one physical page mean a different
        /// register block to each processor. On silicon the local APIC is on
        /// the processor's own die and its aperture never reaches the bus,
        /// which is why `0xfee00000` can be every processor's own; here the
        /// initiator is carried instead, in
        /// [`MemAttrs::requester`](crate::core::space::MemAttrs::requester),
        /// and this is the table that reads it back.
        ///
        /// Called from a local APIC's `bind`, once it has asked the machine
        /// layer what id the processor its `cpu` property names stamps.
        ///
        /// Returns `false` if that requester is already claimed by a live
        /// entry: two local APICs naming one processor is a board that cannot
        /// mean anything, and the caller reports it rather than picking one.
        pub fn attach_local(&self, requester: RequesterId, ops: Weak<dyn MemOps>) -> bool {
            let mut locals = self.locals.lock();
            if locals
                .iter()
                .any(|(id, ops)| *id == requester && ops.strong_count() > 0)
            {
                return false;
            }
            locals.push((requester, ops));
            true
        }

        /// The register block `requester` reaches through the architectural
        /// page, if a local APIC on this bus claimed it.
        ///
        /// The roster lock is released before the caller touches what it holds:
        /// the answer is a device that takes its own state lock, and this bus
        /// never calls outward holding its own (`CLAUDE.md`, re-entrancy).
        #[must_use]
        pub fn local_for(&self, requester: RequesterId) -> Option<Arc<dyn MemOps>> {
            let locals = self.locals.lock();
            let found = locals
                .iter()
                .find(|(id, _)| *id == requester)
                .and_then(|(_, ops)| ops.upgrade());
            // Explicitly, and before the answer is handed back: the caller is
            // about to take that device's own state lock.
            drop(locals);
            found
        }

        /// How many processors have claimed a register block on this bus.
        #[must_use]
        pub fn local_count(&self) -> usize {
            self.locals
                .lock()
                .iter()
                .filter(|(_, ops)| ops.strong_count() > 0)
                .count()
        }

        /// Record that a board has mapped the architectural aperture.
        pub fn note_window(&self) {
            self.window.store(true, Ordering::Release);
        }

        /// Whether a board has mapped it.
        ///
        /// What makes a local APIC that does not know its processor an error
        /// rather than a curiosity: with a window on the bus, an APIC with no
        /// `cpu` property is a processor whose accesses would silently land on
        /// somebody else's registers, which is the defect the window exists to
        /// remove.
        #[must_use]
        pub fn has_window(&self) -> bool {
            self.window.load(Ordering::Acquire)
        }

        /// Put a local APIC on the bus.
        pub fn attach(&self, target: Weak<dyn Target>) {
            self.targets.lock().push(target);
        }

        /// Ask to hear about end-of-interrupt broadcasts.
        pub fn attach_eoi(&self, sink: Weak<dyn EoiSink>) {
            self.eoi.lock().push(sink);
        }

        /// How many live APICs are on the bus.
        #[must_use]
        pub fn len(&self) -> usize {
            self.targets
                .lock()
                .iter()
                .filter(|t| t.strong_count() > 0)
                .count()
        }

        /// Whether nothing is on the bus.
        #[must_use]
        pub fn is_empty(&self) -> bool {
            self.len() == 0
        }

        /// Every live APIC, with the roster lock already released.
        ///
        /// The bus never calls outward with its own lock held: a receiver takes
        /// its state lock and drives its processor's pin, and the sender is
        /// very often another device on this same bus (`CLAUDE.md`,
        /// re-entrancy).
        fn roster(&self) -> Vec<Arc<dyn Target>> {
            self.targets
                .lock()
                .iter()
                .filter_map(Weak::upgrade)
                .collect()
        }

        /// Deliver `message` to whichever APICs it selects.
        ///
        /// `from` is the sending APIC's ID, needed only by the two shorthands
        /// that mention the sender; an I/O APIC passes `None`, having no ID on
        /// this bus.
        pub fn deliver(&self, message: Message, from: Option<u8>, shorthand: Shorthand) {
            let roster = self.roster();
            let mut selected: Vec<&Arc<dyn Target>> = Vec::new();
            for target in &roster {
                let id = target.apic_id();
                let chosen = match shorthand {
                    Shorthand::SelfOnly => Some(id) == from,
                    Shorthand::All => true,
                    Shorthand::AllButSelf => Some(id) != from,
                    Shorthand::Dest => {
                        if message.logical {
                            target.logical_match(message.dest)
                        } else {
                            // 0xff is the physical broadcast; every other value
                            // names one APIC (SDM Vol 3A §10.6.2.1).
                            message.dest == 0xff || message.dest == id
                        }
                    }
                };
                if chosen {
                    selected.push(target);
                }
            }
            if message.delivery == Delivery::LOWEST {
                // The processor bidding lowest wins. Which processor that is is
                // implementation-specific in the SDM, so this picks the least
                // arbitration priority and breaks ties by the least APIC ID —
                // a rule, rather than whatever order the roster happens to be
                // in, because a guest-visible choice may not depend on that
                // (`CLAUDE.md`, determinism).
                let winner = selected
                    .iter()
                    .min_by_key(|t| (t.arbitration_priority(), t.apic_id()))
                    .copied();
                selected.clear();
                selected.extend(winner);
            }
            for target in selected {
                target.accept(message);
            }
        }

        /// Tell every I/O APIC that `vector` has been acknowledged.
        pub fn broadcast_eoi(&self, vector: u8) {
            let sinks: Vec<Arc<dyn EoiSink>> =
                self.eoi.lock().iter().filter_map(Weak::upgrade).collect();
            for sink in sinks {
                sink.eoi(vector);
            }
        }
    }

    /// The bus `name` refers to in `hosts`, creating it on first mention.
    ///
    /// The **host** side of the rendezvous, for a caller that wants to watch
    /// the traffic.
    ///
    /// # Errors
    ///
    /// [`crate::Error::Config`] if another kind of host object already holds
    /// that name.
    pub fn open(hosts: &HostObjects, name: &str) -> Result<Arc<ApicBus>> {
        hosts.open(KIND, name, ApicBus::new)
    }

    /// The bus `name` refers to in the build these properties belong to.
    ///
    /// The **device** side, called from `new(props)`. A `Props` that belongs to
    /// no build gets a private bus, so a device a unit test constructed
    /// directly still works and simply meets nobody.
    ///
    /// # Errors
    ///
    /// As [`open`].
    pub fn attach(props: &Props, name: &str) -> Result<Arc<ApicBus>> {
        props.host(KIND, name, ApicBus::new)
    }
}

// ---------------------------------------------------------------------------
// the register block
// ---------------------------------------------------------------------------

/// Registers are 16 bytes apart and 32 bits wide (SDM Vol 3A §10.4.1: "all
/// registers are accessed using 32-bit loads and stores ... aligned on 128-bit
/// boundaries").
const REG_STRIDE: u64 = 0x10;

const REG_ID: u64 = 0x020;
const REG_VERSION: u64 = 0x030;
const REG_TPR: u64 = 0x080;
const REG_APR: u64 = 0x090;
const REG_PPR: u64 = 0x0a0;
const REG_EOI: u64 = 0x0b0;
const REG_RRD: u64 = 0x0c0;
const REG_LDR: u64 = 0x0d0;
const REG_DFR: u64 = 0x0e0;
const REG_SVR: u64 = 0x0f0;
const REG_ISR: u64 = 0x100;
const REG_TMR: u64 = 0x180;
const REG_IRR: u64 = 0x200;
const REG_ESR: u64 = 0x280;
const REG_ICR_LOW: u64 = 0x300;
const REG_ICR_HIGH: u64 = 0x310;
const REG_LVT_BASE: u64 = 0x320;
const REG_TIMER_INIT: u64 = 0x380;
const REG_TIMER_CUR: u64 = 0x390;
const REG_TIMER_DIV: u64 = 0x3e0;

/// The version this model reports: an integrated APIC with six local vector
/// table entries (SDM Vol 3A §10.4.8, and Table 10-1's version register).
const VERSION: u32 = 0x14;

/// How many local vector table entries there are: timer, thermal, performance
/// counter, LINT0, LINT1, error. No CMCI entry, which is what a max-LVT-entry
/// field of 5 says.
const LVT_COUNT: usize = 6;

/// Local vector table indices, in register order from [`REG_LVT_BASE`].
const LVT_TIMER: usize = 0;
const LVT_LINT0: usize = 3;
const LVT_LINT1: usize = 4;
const LVT_ERROR: usize = 5;

/// The mask bit every local vector table entry has (bit 16).
const LVT_MASK: u32 = 1 << 16;
/// The trigger mode bit of a LINT entry (bit 15): set is level.
const LVT_LEVEL: u32 = 1 << 15;
/// The remote IRR bit of a LINT entry (bit 14), read-only.
const LVT_REMOTE_IRR: u32 = 1 << 14;
// Bit 13, beside those two, is a LINT entry's input pin polarity, and it has no
// constant because nothing applies it: a `core::wire` net carries an assertion
// rather than a voltage, so exclusive-oring the bit into the pin level would
// make an idle LINT pin read as asserted — and an idle LINT0 in `ExtINT` mode
// that reads as asserted is a processor asking an 8259A for a vector it has not
// got, for ever. `ioapic::ENTRY_ACTIVE_LOW` has the long form; `set_lint` below
// is where this one would have been applied. The bit is still writable and
// still reads back.

/// The delivery status bit (bit 12), read-only and always idle here.
const LVT_DELIVERY_STATUS: u32 = 1 << 12;
/// The timer mode field of the timer entry (bits 17-18).
const LVT_TIMER_MODE: u32 = 0b11 << 17;
/// Periodic, as the timer mode field spells it.
const TIMER_PERIODIC: u32 = 0b01 << 17;

/// The reset value of every local vector table entry: masked, and nothing else
/// (SDM Vol 3A Table 10-1).
const LVT_RESET: u32 = LVT_MASK;

/// The spurious-interrupt vector register's APIC software enable (bit 8).
const SVR_ENABLE: u32 = 1 << 8;

/// `IA32_APIC_BASE`'s bootstrap-processor flag (bit 8).
const APIC_BASE_BSP: u64 = 1 << 8;
/// `IA32_APIC_BASE`'s global enable (bit 11).
const APIC_BASE_ENABLE: u64 = 1 << 11;

/// Error status: a vector below 16 was sent (bit 5).
const ESR_SEND_ILLEGAL_VECTOR: u32 = 1 << 5;
/// Error status: a vector below 16 was received (bit 6).
const ESR_RECV_ILLEGAL_VECTOR: u32 = 1 << 6;
/// Error status: a register this APIC does not have was accessed (bit 7).
const ESR_ILLEGAL_REGISTER: u32 = 1 << 7;

/// The lowest vector a fixed interrupt may carry. 0-15 are the architecture's
/// own exception vectors and an APIC refuses to deliver them (SDM Vol 3A
/// §10.5.2).
const FIRST_LEGAL_VECTOR: u8 = 16;

/// A 256-bit interrupt register: ISR, TMR or IRR, eight 32-bit words with
/// vector *v* in bit *v* mod 32 of word *v* / 32.
type Bitmap = [u32; 8];

/// Set the bit for `vector`.
fn bitmap_set(map: &mut Bitmap, vector: u8) {
    map[usize::from(vector) >> 5] |= 1 << (vector & 31);
}

/// Clear the bit for `vector`.
fn bitmap_clear(map: &mut Bitmap, vector: u8) {
    map[usize::from(vector) >> 5] &= !(1 << (vector & 31));
}

/// Whether `vector`'s bit is set.
fn bitmap_get(map: &Bitmap, vector: u8) -> bool {
    map[usize::from(vector) >> 5] & (1 << (vector & 31)) != 0
}

/// The highest vector set, which is the highest priority one: an APIC's
/// priority *is* its vector number (SDM Vol 3A §10.8.3).
fn bitmap_highest(map: &Bitmap) -> Option<u8> {
    for word in (0..8).rev() {
        if map[word] != 0 {
            let bit = 31 - map[word].leading_zeros();
            return Some((word as u8) * 32 + bit as u8);
        }
    }
    None
}

/// Everything the guest can see or change.
#[derive(Debug, Clone, PartialEq, Eq)]
struct State {
    /// The ID register, bits 24-31. Latched from a pin on real silicon and
    /// from a property here, and writable afterwards.
    id: u8,
    /// The task priority register: the priority below which this processor
    /// refuses interrupts.
    tpr: u8,
    /// The logical destination register, bits 24-31.
    ldr: u8,
    /// The destination format register, bits 28-31. All ones is the flat
    /// model, all zeros the cluster model.
    dfr: u8,
    /// The spurious-interrupt vector register.
    svr: u32,
    /// The error status register, as a read will report it.
    esr: u32,
    /// Errors seen since the last write to the error status register. A write
    /// is what moves these into `esr`, which is the register's documented
    /// protocol (SDM Vol 3A §10.5.3).
    esr_pending: u32,
    /// In service: an acknowledge moved these out of `irr`.
    isr: Bitmap,
    /// Trigger mode: set for a vector that arrived level-triggered, so an
    /// end-of-interrupt knows to tell the I/O APIC.
    tmr: Bitmap,
    /// Interrupt request: accepted, not yet acknowledged.
    irr: Bitmap,
    /// The six local vector table entries.
    lvt: [u32; LVT_COUNT],
    /// The interrupt command register's low half, as last written.
    icr_low: u32,
    /// Its high half: the destination field in bits 24-31.
    icr_high: u32,
    /// The timer's initial count register.
    timer_initial: u32,
    /// Bus ticks until the timer expires, or zero when it is stopped.
    ///
    /// In *bus* ticks rather than in counter units, so the divisor is one
    /// exact multiplication rather than a second counter to keep in step. The
    /// guest-visible current count is this divided by the divisor, rounded up,
    /// which is the count that has not finished decrementing yet.
    timer_remaining: u64,
    /// The divide configuration register.
    timer_divide: u32,
    /// The tick, in this APIC's own clock domain, it has been advanced to.
    tick: u64,
    /// `IA32_APIC_BASE`. State without a writer in this build — see the module
    /// documentation — but it is architectural state and it snapshots.
    apic_base: u64,
    /// What each of the two local interrupt pins is doing, as its net resolved
    /// it.
    lint_level: [bool; 2],
    /// Whether an `ExtINT` request is outstanding on LINT0.
    extint: bool,
    /// Whether an `INIT` has been accepted and no Start-Up has followed.
    wait_for_sipi: bool,
    /// The `INIT` line's own level, which the level-triggered `INIT` pair
    /// drives.
    init_asserted: bool,
    /// The page a Start-Up named, until whoever starts processors takes it.
    startup: Option<u8>,
    /// An accepted `INIT` this APIC's processor has not been told about yet.
    ///
    /// `init_asserted` is the *line*; this is the *edge*. A processor runs one
    /// INIT sequence per message accepted, and it may not ask until after the
    /// de-assert half of the level-triggered pair has already dropped the line
    /// (SDM Vol 3A 10.6.1) — so the edge is latched here rather than
    /// reconstructed from the level, which could not tell "asserted and
    /// dropped again" from "never asserted".
    init_pending: bool,
}

impl Default for State {
    fn default() -> State {
        State {
            id: 0,
            tpr: 0,
            ldr: 0,
            // All ones: the flat model, which is the reset state (SDM Vol 3A
            // Table 10-1).
            dfr: 0xf,
            // Vector 0xff, APIC software-disabled.
            svr: 0x0000_00ff,
            esr: 0,
            esr_pending: 0,
            isr: [0; 8],
            tmr: [0; 8],
            irr: [0; 8],
            lvt: [LVT_RESET; LVT_COUNT],
            icr_low: 0,
            icr_high: 0,
            timer_initial: 0,
            timer_remaining: 0,
            timer_divide: 0,
            tick: 0,
            apic_base: DEFAULT_BASE | APIC_BASE_ENABLE,
            lint_level: [false; 2],
            extint: false,
            wait_for_sipi: false,
            init_asserted: false,
            startup: None,
            init_pending: false,
        }
    }
}

impl State {
    /// Whether the APIC is enabled in `IA32_APIC_BASE` — the hardware enable,
    /// which is what makes the register page exist at all (SDM Vol 3A
    /// §10.4.3). A hardware-disabled APIC is transparent: LINT0 becomes the
    /// processor's `INTR` and LINT1 its `NMI`.
    fn hardware_enabled(&self) -> bool {
        self.apic_base & APIC_BASE_ENABLE != 0
    }

    /// Whether the spurious-interrupt vector register's enable bit is set.
    ///
    /// Clear is the reset state, and in it "the mask bits for all the LVT
    /// entries are set" (SDM Vol 3A §10.4.7.2) — so this gates the local vector
    /// table rather than the message inbox, which keeps taking `NMI`, `INIT`
    /// and Start-Up.
    fn software_enabled(&self) -> bool {
        self.svr & SVR_ENABLE != 0
    }

    /// One local vector table entry as a read reports it, with the mask forced
    /// on while the APIC is software-disabled.
    fn lvt(&self, index: usize) -> u32 {
        if self.software_enabled() {
            self.lvt[index]
        } else {
            self.lvt[index] | LVT_MASK
        }
    }

    /// Whether entry `index` can deliver.
    fn lvt_active(&self, index: usize) -> bool {
        self.lvt(index) & LVT_MASK == 0
    }

    /// An entry's delivery mode, bits 8-10.
    fn lvt_delivery(&self, index: usize) -> Delivery {
        Delivery(((self.lvt[index] >> 8) & 7) as u8)
    }

    /// An entry's vector, bits 0-7.
    fn lvt_vector(&self, index: usize) -> u8 {
        self.lvt[index] as u8
    }

    /// The processor priority register (SDM Vol 3A §10.8.3.1).
    fn ppr(&self) -> u8 {
        let isrv = bitmap_highest(&self.isr).unwrap_or(0);
        if (self.tpr >> 4) >= (isrv >> 4) {
            self.tpr
        } else {
            isrv & 0xf0
        }
    }

    /// The arbitration priority register (SDM Vol 3A §10.8.4).
    fn apr(&self) -> u8 {
        let isrv = bitmap_highest(&self.isr).unwrap_or(0);
        let irrv = bitmap_highest(&self.irr).unwrap_or(0);
        if (self.tpr >> 4) >= (irrv >> 4) && (self.tpr >> 4) > (isrv >> 4) {
            self.tpr
        } else {
            ((isrv >> 4).max(irrv >> 4)) << 4
        }
    }

    /// The vector an acknowledge would take, if the priority registers let one
    /// through.
    ///
    /// "The interrupt is serviced when its priority class is higher than the
    /// processor priority register's" (SDM Vol 3A §10.8.4), which is a
    /// comparison of the top four bits and not of the whole vector.
    fn deliverable(&self) -> Option<u8> {
        let vector = bitmap_highest(&self.irr)?;
        ((vector >> 4) > (self.ppr() >> 4)).then_some(vector)
    }

    /// Whether an `ExtINT` request is waiting to be forwarded.
    ///
    /// Not gated by the priority registers, and it cannot be: the vector lives
    /// in the external controller and is not known until the acknowledge cycle
    /// runs, so there is nothing to compare against the processor priority.
    /// The `ExtINT` delivery mode's own definition is that the processor
    /// "respond[s] to the interrupt as if the interrupt originated in an
    /// externally connected 8259A-compatible controller" (SDM Vol 3A §10.5.1),
    /// which is a request the local APIC forwards rather than one it resolves.
    fn extint_pending(&self) -> bool {
        self.extint && (!self.hardware_enabled() || self.lvt_active(LVT_LINT0))
    }

    /// Whether `INTR` should be asserted.
    fn intr(&self) -> bool {
        if !self.hardware_enabled() {
            // Transparent: LINT0 *is* `INTR`. Whatever the pin says, the
            // processor sees.
            return self.lint_level[0];
        }
        self.extint_pending() || self.deliverable().is_some()
    }

    /// The timer's divisor, from bits 3, 1 and 0 of the divide configuration
    /// register (SDM Vol 3A §10.5.4). The three bits count 2, 4, 8 ... 128,
    /// except that all ones means divide by one.
    fn timer_divisor(&self) -> u64 {
        let field = ((self.timer_divide >> 1) & 0b100) | (self.timer_divide & 0b11);
        if field == 0b111 { 1 } else { 2 << field }
    }

    /// The current count register, which counts in timer units rather than in
    /// bus ticks.
    ///
    /// Rounded up, because a count is only spent once the divisor's last bus
    /// tick has gone by: with a divisor of 16 and one bus tick elapsed, the
    /// counter has not decremented yet and the guest must still read the count
    /// it wrote.
    fn timer_current(&self) -> u32 {
        let divisor = self.timer_divisor();
        u32::try_from(self.timer_remaining.div_ceil(divisor)).unwrap_or(u32::MAX)
    }

    /// Whether the timer entry selects periodic mode.
    fn timer_periodic(&self) -> bool {
        self.lvt[LVT_TIMER] & LVT_TIMER_MODE == TIMER_PERIODIC
    }

    /// Whether the timer's mode is one this model runs at all.
    ///
    /// One-shot and periodic are; the TSC-deadline mode (`10`) is not, because
    /// nothing here has a time-stamp counter to compare against, and `11` is
    /// reserved. A timer left in either simply does not count — which is what a
    /// part that does not enumerate the TSC-deadline feature would do with a
    /// reserved mode, and it is stated here rather than guessed at silently.
    fn timer_mode_runs(&self) -> bool {
        matches!(self.lvt[LVT_TIMER] & LVT_TIMER_MODE, 0 | TIMER_PERIODIC)
    }

    /// Bus ticks until the timer's next expiry.
    fn next_event(&self) -> Option<u64> {
        (self.timer_remaining > 0 && self.timer_mode_runs()).then_some(self.timer_remaining)
    }

    /// Advance the timer by `span` bus ticks, reporting whether it expired.
    ///
    /// Several periods inside one span collapse into one request, and that is
    /// the hardware's own behaviour rather than a shortcut: a second expiry
    /// sets a request bit that is already set, and the interrupt is lost. It is
    /// also what keeps a guest that ignores a fast periodic timer from costing
    /// one iteration per period here.
    fn timer_step(&mut self, span: u64) -> bool {
        if self.timer_remaining == 0 || !self.timer_mode_runs() {
            return false;
        }
        if span < self.timer_remaining {
            self.timer_remaining -= span;
            return false;
        }
        let rest = span - self.timer_remaining;
        let period = u64::from(self.timer_initial) * self.timer_divisor();
        self.timer_remaining = if self.timer_periodic() && period > 0 {
            period - (rest % period)
        } else {
            0
        };
        true
    }

    /// Record an error for the next write to the error status register, and
    /// raise the error interrupt if one is armed.
    ///
    /// "The local APIC ... generates an interrupt when an error is detected,
    /// using the vector in the LVT error register" (SDM Vol 3A §10.5.3). The
    /// interrupt is raised on the *transition*: an error bit already recorded
    /// and not yet read has already interrupted once.
    fn error(&mut self, bit: u32) {
        let fresh = self.esr_pending & bit == 0;
        self.esr_pending |= bit;
        if fresh && self.lvt_active(LVT_ERROR) {
            let vector = self.lvt_vector(LVT_ERROR);
            // Directly, not through `request`: an error interrupt carrying an
            // illegal vector would record another error and recurse.
            if vector >= FIRST_LEGAL_VECTOR {
                bitmap_set(&mut self.irr, vector);
                bitmap_clear(&mut self.tmr, vector);
            }
        }
    }

    /// Accept a fixed or lowest-priority vector.
    fn request(&mut self, vector: u8, level_triggered: bool) {
        if vector < FIRST_LEGAL_VECTOR {
            self.error(ESR_RECV_ILLEGAL_VECTOR);
            return;
        }
        bitmap_set(&mut self.irr, vector);
        if level_triggered {
            bitmap_set(&mut self.tmr, vector);
        } else {
            bitmap_clear(&mut self.tmr, vector);
        }
    }

    /// The APIC's own reset, which an `INIT` message performs as well as a
    /// power-up. "The state of the local APIC following an INIT reset is the
    /// same as it is after a power-up reset, except that the APIC ID and
    /// arbitration ID registers are preserved" (SDM Vol 3A §10.4.7.1).
    fn init_reset(&mut self) {
        let id = self.id;
        let base = self.apic_base;
        let pins = self.lint_level;
        let tick = self.tick;
        *self = State::default();
        self.id = id;
        self.apic_base = base;
        self.lint_level = pins;
        self.tick = tick;
    }
}

/// The register block, as something an address space can dispatch to, plus the
/// pins and peers that hang off it.
struct Registers {
    state: Mutex<State>,
    /// The `INTR` and `NMI` outputs, at [`LockRank::LEAF`] so a line can be
    /// driven with nothing else held.
    outs: Mutex<Outputs>,
    /// What answers an acknowledge this APIC forwards: the external
    /// 8259A-compatible controller wired to LINT0, weakly held because the
    /// machine owns both devices.
    extint_ack: Mutex<Option<Weak<dyn IntAck>>>,
    /// The catch-up handle the register paths sync through (§4.2).
    lazy: Mutex<Option<LazyHandle>>,
    /// The message bus this APIC sends on and receives from.
    bus: Arc<ApicBus>,
    /// [`State::tick`], published so [`Device::current_tick`] can answer with
    /// no lock, which the scheduler requires of it.
    tick: AtomicU64,
    /// The absolute tick of the next timer expiry, or [`u64::MAX`] for none.
    /// Same no-lock rule.
    next_event: AtomicU64,
}

/// The pins this APIC drives, once the machine has built them.
#[derive(Debug, Default)]
struct Outputs {
    /// The processor's `INTR` pin.
    intr: Option<WireSource>,
    /// The processor's `NMI` pin.
    nmi: Option<WireSource>,
}

impl fmt::Debug for Registers {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let mut s = f.debug_struct("Registers");
        s.field("tick", &self.tick.load(Ordering::Relaxed));
        match self.state.try_lock() {
            Some(state) => s.field("state", &*state).finish(),
            None => s.field("state", &"<in use>").finish(),
        }
    }
}

/// What a critical section decided has to happen once the lock is released.
///
/// Every outward action this device can take, collected while the state lock is
/// held and performed after it is dropped — the re-entrancy contract, and not a
/// theoretical one here: an interprocessor interrupt lands in a sibling APIC
/// that takes *its* state lock and drives *its* processor's pin.
#[derive(Debug, Default)]
struct Pending {
    /// The level `INTR` should be left at.
    intr: bool,
    /// An `NMI` edge to deliver.
    nmi: bool,
    /// A message to put on the bus, and how it is addressed.
    send: Option<(Message, Shorthand)>,
    /// A vector to announce as acknowledged, for the I/O APICs' remote IRR.
    eoi: Option<u8>,
}

impl Registers {
    /// Republish the two lock-free numbers. Called with the state lock held.
    fn publish(&self, state: &State) {
        self.tick.store(state.tick, Ordering::Relaxed);
        let at = match state.next_event() {
            Some(d) => state.tick.saturating_add(d),
            None => u64::MAX,
        };
        self.next_event.store(at, Ordering::Relaxed);
    }

    /// Perform everything a critical section decided on. No lock held.
    fn settle(&self, pending: Pending) {
        let outs = self.outs.lock().clone_sources();
        if let Some(intr) = &outs.0 {
            intr.set(Level::from_bool(pending.intr));
        }
        if pending.nmi
            && let Some(nmi) = &outs.1
        {
            // An edge, because `NMI` is edge-sensitive and a message is an
            // event rather than a level. Two `set` calls, so a pin that was
            // already high still sees one.
            nmi.set(Level::High);
            nmi.set(Level::Low);
        }
        if let Some(vector) = pending.eoi {
            self.bus.broadcast_eoi(vector);
        }
        if let Some((message, shorthand)) = pending.send {
            let from = self.state.lock().id;
            self.bus.deliver(message, Some(from), shorthand);
        }
    }

    /// Recompute `INTR` from the current state and drive it.
    fn refresh(&self) {
        let intr = self.state.lock().intr();
        self.settle(Pending {
            intr,
            ..Pending::default()
        });
    }

    /// Catch the APIC up before an access is dispatched to it (§4.2).
    fn sync(&self, attrs: MemAttrs) {
        let handle = self.lazy.lock().clone();
        let Some(handle) = handle else {
            return;
        };
        let kind = if attrs.debug {
            AccessKind::Debug
        } else {
            AccessKind::Guest
        };
        // A refusal means catch-up for this device is already running further
        // up the stack. The access still has to be answered, and answering it
        // from where the device stands is the only defined thing to do.
        let _ = handle.sync(kind);
    }

    /// Advance to `target` of this APIC's own clock domain.
    fn advance_to(&self, target: u64) {
        let pending = {
            let mut state = self.state.lock();
            if target <= state.tick {
                // Running backwards is a no-op, not an error.
                return;
            }
            let span = target - state.tick;
            state.tick = target;
            if state.timer_step(span) && state.lvt_active(LVT_TIMER) {
                let vector = state.lvt_vector(LVT_TIMER);
                state.request(vector, false);
            }
            self.publish(&state);
            Pending {
                intr: state.intr(),
                ..Pending::default()
            }
        };
        self.settle(pending);
    }

    /// Read one register. `debug` suppresses every side effect.
    fn read_register(&self, offset: u64, debug: bool) -> u32 {
        let mut state = self.state.lock();
        match offset {
            REG_ID => u32::from(state.id) << 24,
            // Bits 0-7 the version, bits 16-23 the number of LVT entries less
            // one (SDM Vol 3A Table 10-1).
            REG_VERSION => VERSION | ((LVT_COUNT as u32 - 1) << 16),
            REG_TPR => u32::from(state.tpr),
            REG_APR => u32::from(state.apr()),
            REG_PPR => u32::from(state.ppr()),
            // Write-only. A read is not driven by the part; zero is what the
            // reserved registers around it read as.
            REG_EOI => 0,
            // The remote read register. Remote reads are a P6 APIC-bus
            // transaction with no successor, so it reads as zero.
            REG_RRD => 0,
            REG_LDR => u32::from(state.ldr) << 24,
            REG_DFR => (u32::from(state.dfr) << 28) | 0x0fff_ffff,
            REG_SVR => state.svr,
            REG_ESR => state.esr,
            REG_ICR_LOW => state.icr_low,
            REG_ICR_HIGH => state.icr_high,
            REG_TIMER_INIT => state.timer_initial,
            REG_TIMER_CUR => state.timer_current(),
            REG_TIMER_DIV => state.timer_divide,
            _ if (REG_ISR..REG_ISR + 8 * REG_STRIDE).contains(&offset) => {
                state.isr[((offset - REG_ISR) / REG_STRIDE) as usize]
            }
            _ if (REG_TMR..REG_TMR + 8 * REG_STRIDE).contains(&offset) => {
                state.tmr[((offset - REG_TMR) / REG_STRIDE) as usize]
            }
            _ if (REG_IRR..REG_IRR + 8 * REG_STRIDE).contains(&offset) => {
                state.irr[((offset - REG_IRR) / REG_STRIDE) as usize]
            }
            _ if (REG_LVT_BASE..REG_LVT_BASE + LVT_COUNT as u64 * REG_STRIDE).contains(&offset) => {
                state.lvt(((offset - REG_LVT_BASE) / REG_STRIDE) as usize)
            }
            _ => {
                // "Illegal register address" is an error the part records
                // rather than a bus fault it raises (SDM Vol 3A §10.5.3), so
                // the read is answered with zeros.
                if !debug {
                    state.error(ESR_ILLEGAL_REGISTER);
                }
                0
            }
        }
    }

    /// Write one register, reporting what has to happen once the lock is out of
    /// the way.
    fn write_register(&self, offset: u64, value: u32) {
        let pending = {
            let mut state = self.state.lock();
            let mut pending = Pending::default();
            match offset {
                REG_ID => state.id = (value >> 24) as u8,
                REG_TPR => state.tpr = value as u8,
                REG_EOI => {
                    // "The only write that is architecturally defined is a
                    // write of 0" (SDM Vol 3A §10.8.5); the value is ignored
                    // either way, and what matters is which bit clears.
                    if let Some(vector) = bitmap_highest(&state.isr) {
                        bitmap_clear(&mut state.isr, vector);
                        if bitmap_get(&state.tmr, vector) {
                            bitmap_clear(&mut state.tmr, vector);
                            // A level-triggered interrupt's end-of-interrupt is
                            // broadcast to the I/O APICs so they can clear the
                            // matching remote IRR and let the line interrupt
                            // again (82093AA datasheet §3.2.4).
                            pending.eoi = Some(vector);
                        }
                    }
                }
                REG_LDR => state.ldr = (value >> 24) as u8,
                REG_DFR => state.dfr = (value >> 28) as u8,
                REG_SVR => {
                    // Vector (0-7), the APIC software enable (8), focus
                    // processor checking (9) and end-of-interrupt broadcast
                    // suppression (12). Bits 10-11 and everything above are
                    // reserved (SDM Vol 3A Figure 10-23).
                    state.svr = value & 0x0000_13ff;
                }
                REG_ESR => {
                    // The write is what latches: "the ESR must be written
                    // before it is read; the write updates the register with
                    // the error state accumulated since the last write" (SDM
                    // Vol 3A §10.5.3).
                    state.esr = core::mem::take(&mut state.esr_pending);
                }
                REG_ICR_HIGH => state.icr_high = value & 0xff00_0000,
                REG_ICR_LOW => {
                    // Vector (0-7), delivery mode (8-10), destination mode
                    // (11), level (14), trigger mode (15) and the destination
                    // shorthand (18-19). Bit 12 is the delivery status, which
                    // is read-only and always idle here because delivery is
                    // synchronous; bit 13 and the rest are reserved.
                    state.icr_low = value & 0x000c_cfff;
                    let message = Message {
                        vector: value as u8,
                        delivery: Delivery(((value >> 8) & 7) as u8),
                        logical: value & (1 << 11) != 0,
                        dest: (state.icr_high >> 24) as u8,
                        level_triggered: value & (1 << 15) != 0,
                        assert: value & (1 << 14) != 0,
                    };
                    if message.delivery == Delivery::FIXED && message.vector < FIRST_LEGAL_VECTOR {
                        state.error(ESR_SEND_ILLEGAL_VECTOR);
                    } else {
                        pending.send = Some((message, Shorthand::from_bits(value >> 18)));
                    }
                }
                REG_TIMER_INIT => {
                    state.timer_initial = value;
                    // "Writing to the initial count register starts the timer"
                    // and writing zero stops it (SDM Vol 3A §10.5.4).
                    state.timer_remaining = u64::from(value) * state.timer_divisor();
                }
                REG_TIMER_DIV => state.timer_divide = value & 0b1011,
                _ if (REG_LVT_BASE..REG_LVT_BASE + LVT_COUNT as u64 * REG_STRIDE)
                    .contains(&offset) =>
                {
                    let index = ((offset - REG_LVT_BASE) / REG_STRIDE) as usize;
                    // Delivery status and remote IRR are read-only, and the
                    // mask cannot be cleared while the APIC is software
                    // disabled (SDM Vol 3A §10.4.7.2).
                    let keep = state.lvt[index] & LVT_REMOTE_IRR;
                    let mut written = (value & !(LVT_DELIVERY_STATUS | LVT_REMOTE_IRR)) | keep;
                    if !state.software_enabled() {
                        written |= LVT_MASK;
                    }
                    state.lvt[index] = written;
                    if index == LVT_TIMER && !state.timer_mode_runs() {
                        state.timer_remaining = 0;
                    }
                }
                // Everything else is read-only or reserved. Recorded, not
                // faulted (SDM Vol 3A §10.5.3).
                _ => state.error(ESR_ILLEGAL_REGISTER),
            }
            self.publish(&state);
            pending.intr = state.intr();
            pending
        };
        self.settle(pending);
    }

    /// Drive one of the two local interrupt pins.
    fn set_lint(&self, index: usize, high: bool) {
        self.sync(MemAttrs::DEFAULT);
        let pending = {
            let mut state = self.state.lock();
            let was = state.lint_level[index];
            state.lint_level[index] = high;
            let entry = if index == 0 { LVT_LINT0 } else { LVT_LINT1 };
            let mut pending = Pending::default();
            if !state.hardware_enabled() {
                // Transparent: LINT0 is `INTR` and LINT1 is `NMI`. The
                // acknowledge still reaches the external controller, because
                // that is who is on the far side of the pin.
                if index == 0 {
                    state.extint = high;
                } else {
                    pending.nmi = high && !was;
                }
            } else {
                // The net's own level is the assertion. The entry's polarity
                // bit (13) is not applied to it — see the note beside the LVT
                // bit constants above.
                let asserted = high;
                let was_asserted = was;
                let level = state.lvt[entry] & LVT_LEVEL != 0;
                let edge = asserted && !was_asserted;
                match state.lvt_delivery(entry) {
                    Delivery::EXTINT if index == 0 => state.extint = asserted,
                    _ if !state.lvt_active(entry) => {}
                    Delivery::NMI => pending.nmi = edge,
                    Delivery::FIXED => {
                        if level {
                            if asserted {
                                let v = state.lvt_vector(entry);
                                state.request(v, true);
                            }
                        } else if edge {
                            let v = state.lvt_vector(entry);
                            state.request(v, false);
                        }
                    }
                    // `INIT` and Start-Up have no meaning on a pin here, and
                    // SMI has nowhere to go. Left alone rather than pretended.
                    _ => {}
                }
            }
            pending.intr = state.intr();
            pending
        };
        self.settle(pending);
    }

    /// Take a message off the bus.
    fn accept_message(&self, message: Message) {
        let pending = {
            let mut state = self.state.lock();
            let mut pending = Pending::default();
            match message.delivery {
                Delivery::FIXED | Delivery::LOWEST => {
                    state.request(message.vector, message.level_triggered);
                }
                Delivery::NMI => pending.nmi = true,
                Delivery::INIT => {
                    if message.assert {
                        state.init_reset();
                        state.wait_for_sipi = true;
                        state.init_asserted = true;
                        state.init_pending = true;
                    } else {
                        // The de-assert half of the level-triggered pair. On a
                        // P6 it reset the arbitration IDs; there is nothing
                        // here for it to do but drop the line.
                        state.init_asserted = false;
                    }
                }
                // "If the processor is not in the wait-for-SIPI state the
                // Start-Up IPI is ignored" — MP specification §B.4, and the
                // reason the universal algorithm sends two of them.
                Delivery::STARTUP if state.wait_for_sipi => {
                    state.wait_for_sipi = false;
                    state.startup = Some(message.vector);
                }
                // SMI has no system management mode to enter here and `ExtINT`
                // by message is not supported on any processor this models
                // (SDM Vol 3A §10.6.1). Dropped, and said so.
                _ => {}
            }
            self.publish(&state);
            pending.intr = state.intr();
            pending
        };
        self.settle(pending);
    }
}

impl Outputs {
    /// Both sources, cloned so the pins can be driven with the lock released.
    fn clone_sources(&self) -> (Option<WireSource>, Option<WireSource>) {
        (self.intr.clone(), self.nmi.clone())
    }
}

impl bus::Target for Registers {
    fn apic_id(&self) -> u8 {
        self.state.lock().id
    }

    fn logical_match(&self, dest: u8) -> bool {
        let state = self.state.lock();
        if state.dfr == 0xf {
            // The flat model: eight processors, one bit each, and 0xff is a
            // broadcast by construction (SDM Vol 3A §10.6.2.2).
            state.ldr & dest != 0
        } else {
            // The cluster model: the top nibble names a cluster and the bottom
            // one is a bitmap inside it.
            (state.ldr >> 4) == (dest >> 4) && (state.ldr & dest & 0x0f) != 0
        }
    }

    fn arbitration_priority(&self) -> u8 {
        self.state.lock().apr()
    }

    fn accept(&self, message: Message) {
        self.accept_message(message);
    }
}

impl IntAck for Registers {
    /// The processor has taken the interrupt and wants a vector.
    ///
    /// Three answers, in the order the part decides them: an `ExtINT` request
    /// is forwarded to whatever is on the far side of LINT0 and *that*
    /// controller's vector is the answer; otherwise the highest-priority
    /// request that outranks the processor priority moves from requested to in
    /// service; otherwise the spurious vector, with no in-service bit set (SDM
    /// Vol 3A §10.9).
    fn acknowledge(&self, cycle: IntAckCycle) -> IntAckResponse {
        let (answer, delegate) = {
            let mut state = self.state.lock();
            if state.extint_pending() {
                let ack = self.extint_ack.lock().clone();
                (None, ack)
            } else {
                match state.deliverable() {
                    Some(vector) => {
                        bitmap_clear(&mut state.irr, vector);
                        bitmap_set(&mut state.isr, vector);
                        (Some(u32::from(vector)), None)
                    }
                    None => (Some(state.svr & 0xff), None),
                }
            }
        };
        // Outside the lock: the external controller's acknowledge drops its own
        // `INT` output, which lands straight back on this APIC's LINT0 pin.
        let response = match answer {
            Some(vector) => IntAckResponse::Vector(vector),
            None => delegate
                .as_ref()
                .and_then(Weak::upgrade)
                .map(|ack| ack.acknowledge(cycle))
                .filter(|response| response.answered())
                // An `ExtINT` with nothing wired where the 8259A belongs is a
                // machine wired without one, not something to panic over. The
                // spurious vector is the defined answer to "you asked and there
                // is nothing".
                .unwrap_or_else(|| IntAckResponse::Vector(self.state.lock().svr & 0xff)),
        };
        self.refresh();
        response
    }
}

impl LocalController for Registers {
    fn take_startup(&self) -> Startup {
        let mut state = self.state.lock();
        // Three separate facts, and a processor that was not running for any of
        // the sequence sees all three at once: the INIT it owes a reset for,
        // whether the line is still holding it there, and the page a Start-Up
        // named behind it (`core::wire`'s `Startup`).
        let init = mem::replace(&mut state.init_pending, false);
        Startup {
            init,
            held: state.init_asserted,
            page: state.startup.take(),
        }
    }

    fn base_register(&self) -> u64 {
        self.state.lock().apic_base
    }

    fn set_base_register(&self, value: u64) {
        // What `LocalApic::set_apic_base` does, and the outward half is outside
        // the critical section for the reason every other path here is: the
        // enable bit decides whether `INTR` is this APIC's or LINT0's, and
        // driving that pin re-enters the processor.
        self.state.lock().apic_base = value;
        self.refresh();
    }
}

impl MemOps for Registers {
    fn read(&self, offset: u64, dst: &mut [u8], attrs: MemAttrs) -> MemResult {
        let [a, b, c, d] = dst else {
            return Err(BusError::BadAccess);
        };
        if !offset.is_multiple_of(REG_STRIDE) {
            // "Any access that touches bytes 4 through 15 of an APIC register
            // may cause undefined behavior" (SDM Vol 3A §10.4.1). Undefined is
            // not a thing an emulator gets to be, so it is a bus fault.
            return Err(BusError::BadAccess);
        }
        if !attrs.debug {
            self.sync(attrs);
        }
        if !self.state.lock().hardware_enabled() {
            // A hardware-disabled APIC has no register page: the aperture is
            // gone, and a read of it finds whatever the board does with an
            // unclaimed cycle.
            return Err(BusError::BadAccess);
        }
        let value = self.read_register(offset, attrs.debug);
        let bytes = value.to_le_bytes();
        *a = bytes[0];
        *b = bytes[1];
        *c = bytes[2];
        *d = bytes[3];
        Ok(())
    }

    fn write(&self, offset: u64, src: &[u8], attrs: MemAttrs) -> MemResult {
        let [a, b, c, d] = src else {
            return Err(BusError::BadAccess);
        };
        if !offset.is_multiple_of(REG_STRIDE) {
            return Err(BusError::BadAccess);
        }
        if attrs.debug {
            // There is no harmless write. An end-of-interrupt clears an
            // in-service bit, an interrupt command register write sends an
            // interrupt to another processor, and an initial count write starts
            // a timer (`ROADMAP.md` §15, invariant 5).
            return Err(BusError::BadAccess);
        }
        self.sync(attrs);
        if !self.state.lock().hardware_enabled() {
            return Err(BusError::BadAccess);
        }
        self.write_register(offset, u32::from_le_bytes([*a, *b, *c, *d]));
        Ok(())
    }

    fn constraints(&self) -> AccessConstraints {
        // 32 bits, naturally aligned: "all registers are accessed using 32-bit
        // loads and stores" (SDM Vol 3A §10.4.1). The 16-byte spacing is
        // checked in the handler, because a constraint cannot express it.
        AccessConstraints::word(Width::U32, Endian::Little)
    }
}

// ---------------------------------------------------------------------------
// the architectural page
// ---------------------------------------------------------------------------

/// The one page every processor reaches its **own** local APIC through.
///
/// # Why a board needs this at all
///
/// `IA32_APIC_BASE` comes out of reset naming `0xfee00000` (SDM Vol 3A
/// §10.4.4), and both of the tables that describe a multiprocessor PC have room
/// for exactly one local-APIC address (*MP* §4.2, *ACPI* §5.2.12) — because on
/// silicon the register block is **on the processor's own die** and its
/// aperture never reaches the system bus. Every processor therefore sees a
/// different thing at one address, which no `map` statement can say: decode in
/// [`core::space`](crate::core::space) is strictly address to region and
/// nothing on that path branches on who is asking.
///
/// It does not have to. The initiator is already carried —
/// [`MemAttrs::requester`](crate::core::space::MemAttrs::requester) is
/// allocated per object by the machine layer, stamped by `cpu.x86` on every
/// access it makes, rebuilt on both of KVM's exit paths, and delivered to
/// [`MemOps`] unchanged. So the *device* demultiplexes on it, and the address
/// space is untouched. This is that device.
///
/// # What it decodes to
///
/// The register block whose local APIC named this requester's processor in its
/// `cpu` property, and failing that the APIC that publishes this window — which
/// is the honest answer for an access that did not come from a processor at
/// all. A debugger reading `0xfee00000`, a DMA engine that wandered there, a
/// snapshot: none of them is a processor, none of them has an APIC, and the
/// bootstrap processor's page is what a person reading the machine expects to
/// see. Every attribute, `debug` included, is passed through untouched, so a
/// debug read is still refused a write and still pops nothing.
///
/// It is not a device of its own. A window is a *view* of the APICs on one bus
/// and has no state, no reset and no snapshot chunk of its own; making it an
/// object would put a fourth thing in every board file that has to agree with
/// the other three.
#[derive(Debug)]
struct ApicWindow {
    /// The bus whose roster says which processor is which.
    bus: Arc<ApicBus>,
    /// The register block an access from anything that is not a processor on
    /// that roster reaches.
    fallback: Arc<Registers>,
}

impl ApicWindow {
    /// Whose registers this access is for.
    fn target(&self, attrs: MemAttrs) -> Arc<dyn MemOps> {
        // The roster lock is taken and released inside `local_for`, before the
        // block below takes the target's own state lock: a guest access already
        // holds a `BUS`-ranked lock, the roster ranks under it, and a device's
        // state ranks under that (`bus::BUS_RANK`).
        self.bus
            .local_for(attrs.requester)
            .unwrap_or_else(|| Arc::clone(&self.fallback) as Arc<dyn MemOps>)
    }
}

impl MemOps for ApicWindow {
    fn read(&self, offset: u64, dst: &mut [u8], attrs: MemAttrs) -> MemResult {
        self.target(attrs).read(offset, dst, attrs)
    }

    fn write(&self, offset: u64, src: &[u8], attrs: MemAttrs) -> MemResult {
        self.target(attrs).write(offset, src, attrs)
    }

    fn constraints(&self) -> AccessConstraints {
        // The same register block whichever processor is reading, so the same
        // constraints: 32 bits, naturally aligned (SDM Vol 3A §10.4.1).
        AccessConstraints::word(Width::U32, Endian::Little)
    }
}

// ---------------------------------------------------------------------------
// the device
// ---------------------------------------------------------------------------

/// One processor's local APIC.
#[derive(Debug)]
pub struct LocalApic {
    regs: Arc<Registers>,
    region: RegionRef,
    /// The architectural page, built once whether or not a board maps it: it
    /// holds nothing a machine would have to pay for unmapped.
    window: RegionRef,
    /// The processor this APIC belongs to, as the machine file names it.
    ///
    /// Resolved to a [`RequesterId`](crate::core::space::RequesterId) at bind
    /// time, because the id is allocated by declaration order and a file cannot
    /// write it down.
    cpu: Option<String>,
    /// The device's own references to its input pins. A net holds only weak
    /// ones, so something has to keep them alive.
    pins: Mutex<Vec<Arc<LintPin>>>,
    /// Whether this APIC was declared the bootstrap processor.
    bsp: bool,
    /// The ID a reset restores, which is a pin on real silicon.
    reset_id: u8,
}

/// One of the two local interrupt pins.
///
/// A [`FanIn`] per pin, because LINT0 on a PC is the 8259A's `INT` output and
/// LINT1 is the wire-ORed `NMI` net — two sources on one pin is the normal
/// case, and a pin that only remembered "somebody said high" would drop the
/// line when either of them said low (`ROADMAP.md` §4.3).
#[derive(Debug)]
struct LintPin {
    regs: Arc<Registers>,
    index: usize,
    inputs: FanIn,
}

impl WireSink for LintPin {
    fn set_level(&self, src: WireId, _line: u32, level: Level) {
        self.inputs.set(src, level);
        let high = self.inputs.resolve(Resolve::Or).is_high();
        self.regs.set_lint(self.index, high);
    }
}

impl LocalApic {
    /// Validate `props` and build the device.
    ///
    /// # Errors
    ///
    /// [`Error::Property`] if `id` is not a
    /// value an eight-bit APIC ID can hold, or if a property this class does
    /// not know was given.
    pub fn new(props: &Props) -> Result<LocalApic> {
        let mut r = props.reader();
        let id = u8::try_from(r.or_range::<u64>("id", 0, 0..=255)?).unwrap_or(0);
        // The bootstrap processor is the one that comes out of reset running,
        // and on every board that has ever shipped it is the one with the
        // lowest APIC ID. A file that arranges otherwise says so.
        let bsp = r.or("bsp", id == 0)?;
        let name = r.or_str("bus", bus::DEFAULT_NAME)?.to_string();
        // Which processor's accesses this APIC answers through the
        // architectural page. A *link*, resolved to a requester id at bind
        // time: the id is allocated by declaration order and a machine file
        // must never write the number down.
        let cpu = r.optional_link("cpu")?.map(|l| l.as_str().to_string());
        r.finish()?;
        // Opening the bus is allocation rather than an outward action: a
        // get-or-create of a passive object in a table the caller already owns
        // (`core::hosts`, "which phase opens one").
        let bus = bus::attach(props, &name)?;
        let mut apic = LocalApic::with_bus(id, bsp, bus);
        apic.cpu = cpu;
        Ok(apic)
    }

    /// One in the default configuration: APIC ID 0, the bootstrap processor,
    /// on a bus of its own.
    #[must_use]
    pub fn default_device() -> LocalApic {
        LocalApic::with_bus(0, true, Arc::new(ApicBus::new()))
    }

    /// One with the ID and role given, on `bus`.
    #[must_use]
    pub fn with_bus(id: u8, bsp: bool, bus: Arc<ApicBus>) -> LocalApic {
        let mut state = State {
            id,
            ..State::default()
        };
        if bsp {
            state.apic_base |= APIC_BASE_BSP;
        }
        let regs = Arc::new(Registers {
            state: Mutex::with_rank(LockRank::DEVICE, state),
            outs: Mutex::with_rank(LockRank::LEAF, Outputs::default()),
            extint_ack: Mutex::with_rank(LockRank::LEAF, None),
            lazy: Mutex::with_rank(LockRank::LEAF, None),
            bus,
            tick: AtomicU64::new(0),
            next_event: AtomicU64::new(u64::MAX),
        });
        let region: RegionRef = Arc::new(Region::io(
            CLASS_NAME,
            REGISTER_WINDOW_LEN,
            Arc::clone(&regs) as Arc<dyn MemOps>,
        ));
        let window: RegionRef = Arc::new(Region::io(
            // The **same region name** as the register page, deliberately: it
            // is a local APIC's register page, and more than one survey in the
            // tree finds one by walking a space for a region called this
            // (`dev::q35::acpi::survey` does). A board that maps the window
            // instead of the page has not stopped having a local APIC there,
            // and a name that said otherwise would quietly cost it its ACPI
            // MADT.
            CLASS_NAME,
            REGISTER_WINDOW_LEN,
            Arc::new(ApicWindow {
                bus: Arc::clone(&regs.bus),
                fallback: Arc::clone(&regs),
            }) as Arc<dyn MemOps>,
        ));
        LocalApic {
            regs,
            region,
            window,
            cpu: None,
            pins: Mutex::with_rank(LockRank::LEAF, Vec::new()),
            bsp,
            reset_id: id,
        }
    }

    /// The message bus this APIC is on.
    #[must_use]
    pub fn bus(&self) -> &Arc<ApicBus> {
        &self.regs.bus
    }

    /// The APIC ID, as the ID register currently reads.
    #[must_use]
    pub fn id(&self) -> u8 {
        self.regs.state.lock().id
    }

    /// `IA32_APIC_BASE`, with its enable and bootstrap-processor flags.
    ///
    /// The register a guest's `RDMSR`/`WRMSR` reaches: the core forwards those
    /// two instructions here through [`LocalController::base_register`],
    /// because the register names state that lives in the controller rather
    /// than in the processor (SDM Vol 3A 10.4.3).
    #[must_use]
    pub fn apic_base(&self) -> u64 {
        self.regs.state.lock().apic_base
    }

    /// Set `IA32_APIC_BASE`, as a guest's `WRMSR` does.
    ///
    /// The base address field is *reported*, not obeyed: relocating the
    /// register page is an address-space retopology and a device does not get
    /// to do that to itself — a machine file's `map` places the window. The
    /// enable bit is obeyed, and clearing it makes the APIC transparent, with
    /// LINT0 acting as the processor's `INTR` and LINT1 as its `NMI` (SDM
    /// Vol 3A §10.4.3).
    pub fn set_apic_base(&self, value: u64) {
        {
            let mut state = self.regs.state.lock();
            state.apic_base = value;
        }
        self.regs.refresh();
    }

    /// Whether `INTR` is currently asserted.
    #[must_use]
    pub fn intr_asserted(&self) -> bool {
        self.regs.state.lock().intr()
    }

    /// The interrupt request register, as eight words.
    #[must_use]
    pub fn requested(&self) -> [u32; 8] {
        self.regs.state.lock().irr
    }

    /// The in-service register, as eight words.
    #[must_use]
    pub fn in_service(&self) -> [u32; 8] {
        self.regs.state.lock().isr
    }

    /// Whether an `INIT` has been accepted and no Start-Up has arrived yet.
    #[must_use]
    pub fn waiting_for_startup(&self) -> bool {
        self.regs.state.lock().wait_for_sipi
    }

    /// Whether the `INIT` line is asserted.
    #[must_use]
    pub fn init_asserted(&self) -> bool {
        self.regs.state.lock().init_asserted
    }

    /// Take the page a Start-Up named, if one has arrived.
    ///
    /// A Start-Up message tells the processor to begin executing at
    /// `CS:IP = vector << 8 : 0` — a real-mode segment whose base is
    /// `vector << 12` — from a halted, wait-for-SIPI state.
    ///
    /// **The processor takes it through [`LocalController`], not through
    /// here**: this is the accessor a test uses to look at the latch, and it
    /// consumes it, so a machine's processor and a test cannot both have it.
    pub fn take_startup(&self) -> Option<u8> {
        self.regs.state.lock().startup.take()
    }

    /// Advance to `tick` of this APIC's own clock domain.
    ///
    /// What [`Device::advance_to`] does; a test that is not running a scheduler
    /// calls this.
    pub fn advance_to(&self, tick: u64) {
        self.regs.advance_to(tick);
    }

    /// The tick this APIC has been advanced to.
    #[must_use]
    pub fn tick(&self) -> u64 {
        self.regs.tick.load(Ordering::Relaxed)
    }

    /// Which local interrupt pin `port` names, if it names one.
    fn lint_number(port: &str) -> Option<usize> {
        match port {
            "lint0" => Some(0),
            "lint1" => Some(1),
            _ => None,
        }
    }
}

/// The `pc.lapic` device class.
pub static CLASS: DeviceClass = DeviceClass {
    name: CLASS_NAME,
    version: STATE_VERSION,
    summary: "a processor's local APIC, with its timer and the interprocessor interrupt path",
    properties: &[
        PropertySpec {
            name: "id",
            kind: ValueKind::Uint,
            required: false,
            summary: "the APIC ID this part comes out of reset with, 0-255 (default 0)",
        },
        PropertySpec {
            name: "bsp",
            kind: ValueKind::Bool,
            required: false,
            summary: "whether this is the bootstrap processor (default: true for APIC ID 0)",
        },
        PropertySpec {
            name: "bus",
            kind: ValueKind::Str,
            required: false,
            summary: "the APIC message bus this part is on (default `apic`)",
        },
        PropertySpec {
            name: "cpu",
            kind: ValueKind::Link,
            required: false,
            summary: "the processor this APIC belongs to, for the `window` region's decode",
        },
    ],
    construct: |props| Ok(Box::new(LocalApic::new(props)?)),
};

impl Device for LocalApic {
    fn class(&self) -> &'static DeviceClass {
        &CLASS
    }

    fn realize(&self, _ctx: &mut RealizeCtx<'_>) -> Result<()> {
        // Announcing itself into a table others read is exactly the outward
        // action realize is for (`core::hosts`, "which phase opens one").
        self.regs
            .bus
            .attach(Arc::downgrade(&self.regs) as Weak<dyn Target>);
        Ok(())
    }

    fn reset(&self, _kind: ResetKind) {
        // Both kinds: there is no battery behind an APIC, and a timer that
        // survived a reset would keep interrupting a kernel that had not
        // programmed it.
        //
        // The LINT pin levels survive, for the reason the 8259A's `reset` gives
        // about the same event: a line the board is still holding high has not
        // moved, and nothing re-announces an unchanged level. Forgetting it
        // would cost a level-triggered request outright and turn the next
        // re-announcement into a fabricated edge.
        //
        // So does the tick, and for a sharper reason: it is not architectural
        // state at all but this device's cursor in its clock domain, and the
        // domain does not go back to zero because a chip on it was reset. A
        // reset that zeroed it would leave the two disagreeing by however long
        // the machine had been running, and the next catch-up would advance the
        // timer by all of it at once. `init_reset` keeps it for the same
        // reason and says so; this is the same fact about the same counter.
        let pending = {
            let mut state = self.regs.state.lock();
            let pins = state.lint_level;
            let tick = state.tick;
            *state = State::default();
            state.id = self.reset_id;
            state.lint_level = pins;
            state.tick = tick;
            if self.bsp {
                state.apic_base |= APIC_BASE_BSP;
            } else {
                // An application processor does not execute the reset vector.
                // The MP initialization protocol runs over the APIC bus at
                // power-up; one processor wins the BSP flag and every other one
                // "enters a wait-for-SIPI state" without ever fetching an
                // instruction (SDM Vol 3A 8.4.3, and MP specification 4.3.2).
                //
                // This part is the half of the pair that knows which processor
                // it is sitting in front of — `bsp` is its property, not the
                // core's — so this is where that is said. The processor is told
                // at its first instruction boundary, through `LocalController`,
                // which is the same route a later INIT takes.
                state.wait_for_sipi = true;
                state.init_pending = true;
            }
            self.regs.publish(&state);
            Pending {
                intr: state.intr(),
                ..Pending::default()
            }
        };
        self.regs.settle(pending);
    }

    fn region(&self, name: &str) -> Option<RegionRef> {
        if name == WINDOW_REGION {
            // Asking for the aperture is what *makes* a board multiprocessor
            // in this model, and it is the only route to one — so it is also
            // where the bus learns that every local APIC on it now has to know
            // its processor. `bind` is where that is enforced, and it runs
            // after every `map` statement has been resolved, so the flag is
            // set by then however the file is ordered.
            self.regs.bus.note_window();
            return Some(Arc::clone(&self.window));
        }
        matches!(name, "" | "regs").then(|| Arc::clone(&self.region))
    }

    fn sink(&self, port: &str, sources: &[WireId]) -> Option<SinkPin> {
        let index = LocalApic::lint_number(port)?;
        // The fan-in can only be built now: it is told its sources at
        // construction and no `WireId` existed when this APIC was made.
        //
        // Nothing seeds the pin level from it, and nothing needs to: a LINT
        // line idles low, a fresh `FanIn` holds every source low, and
        // `State::default` agrees with both.
        let pin = Arc::new(LintPin {
            regs: Arc::clone(&self.regs),
            index,
            inputs: FanIn::new(sources),
        });
        self.pins.lock().push(Arc::clone(&pin));
        Some(SinkPin {
            sink: pin,
            line: index as u32,
        })
    }

    fn connect(&self, port: &str, source: WireSource) -> Result<()> {
        let mut outs = self.regs.outs.lock();
        match port {
            "intr" => outs.intr = Some(source),
            "nmi" => outs.nmi = Some(source),
            _ => {
                return Err(Error::Config {
                    at: port.to_string(),
                    message: String::from("a local APIC drives `intr` and `nmi`"),
                });
            }
        }
        Ok(())
    }

    fn announce(&self, port: &str) {
        // `INTR` idles low and so does `NMI`, which is what a fresh net holds,
        // but a restored machine may not — so both are announced from state.
        if port == "intr" {
            self.regs.refresh();
        }
    }

    fn int_ack(&self, port: &str) -> Option<Arc<dyn IntAck>> {
        // The device owns this `Arc`; the net gets a `Weak`, so building one
        // here would hand out a reference that is already dead.
        (port == "intr").then(|| Arc::clone(&self.regs) as Arc<dyn IntAck>)
    }

    fn local_controller(&self, port: &str) -> Option<Arc<dyn LocalController>> {
        // The processor's own half of this part: where an INIT and a Start-Up
        // reach it, and where its `IA32_APIC_BASE` lives. Offered on the pin
        // that drives `INTR`, because on the hardware it is one connection —
        // the controller is inside the processor it interrupts. The device owns
        // this `Arc`; the core keeps a `Weak`, as it does for `int_ack`.
        (port == "intr").then(|| Arc::clone(&self.regs) as Arc<dyn LocalController>)
    }

    fn attach_int_ack(&self, port: &str, ack: Weak<dyn IntAck>) {
        // Only LINT0 has an acknowledge to forward: `ExtINT` is defined on it
        // and the architecture supports one external controller in a system.
        if port == "lint0" {
            *self.regs.extint_ack.lock() = Some(ack);
        }
    }

    fn is_lazy(&self) -> bool {
        true
    }

    fn current_tick(&self) -> u64 {
        self.regs.tick.load(Ordering::Relaxed)
    }

    fn advance_to(&self, tick: u64) {
        self.regs.advance_to(tick);
    }

    fn next_event_tick(&self) -> Option<u64> {
        match self.regs.next_event.load(Ordering::Relaxed) {
            u64::MAX => None,
            at => Some(at),
        }
    }

    fn attach_lazy(&self, handle: LazyHandle) {
        *self.regs.lazy.lock() = Some(handle);
    }

    fn save(&self, w: &mut ChunkWriter<'_>) -> Result<()> {
        let state = self.regs.state.lock();
        w.write_u8(state.id)?;
        w.write_u8(state.tpr)?;
        w.write_u8(state.ldr)?;
        w.write_u8(state.dfr)?;
        w.write_u32(state.svr)?;
        w.write_u32(state.esr)?;
        w.write_u32(state.esr_pending)?;
        for map in [&state.isr, &state.tmr, &state.irr] {
            for word in map {
                w.write_u32(*word)?;
            }
        }
        w.write_seq_len(LVT_COUNT as u64)?;
        for entry in state.lvt {
            w.write_u32(entry)?;
        }
        w.write_u32(state.icr_low)?;
        w.write_u32(state.icr_high)?;
        w.write_u32(state.timer_initial)?;
        w.write_u64(state.timer_remaining)?;
        w.write_u32(state.timer_divide)?;
        // The APIC's own position in its domain. The scheduler restores the
        // domain; without this the two would disagree and the timer would stand
        // still until the domain caught up with it.
        w.write_u64(state.tick)?;
        w.write_u64(state.apic_base)?;
        for level in state.lint_level {
            w.write_bool(level)?;
        }
        for flag in [
            state.extint,
            state.wait_for_sipi,
            state.init_asserted,
            state.init_pending,
        ] {
            w.write_bool(flag)?;
        }
        match state.startup {
            None => w.write_bool(false)?,
            Some(vector) => {
                w.write_bool(true)?;
                w.write_u8(vector)?;
            }
        }
        Ok(())
        // The bus roster, the pins and the forwarded acknowledge are the
        // machine's wiring, not this part's state (`ROADMAP.md` §4.5).
    }

    fn load(&self, r: &mut ChunkReader<'_>) -> Result<()> {
        let mut state = State {
            id: r.read_u8()?,
            tpr: r.read_u8()?,
            ldr: r.read_u8()?,
            dfr: r.read_u8()?,
            svr: r.read_u32()?,
            esr: r.read_u32()?,
            esr_pending: r.read_u32()?,
            ..State::default()
        };
        for map in [&mut state.isr, &mut state.tmr, &mut state.irr] {
            for word in map.iter_mut() {
                *word = r.read_u32()?;
            }
        }
        let entries = r.read_seq_len(4)? as usize;
        if entries != LVT_COUNT {
            return Err(Error::State(format!(
                "snapshot has {entries} local vector table entries, this APIC has {LVT_COUNT}"
            )));
        }
        for entry in &mut state.lvt {
            *entry = r.read_u32()?;
        }
        state.icr_low = r.read_u32()?;
        state.icr_high = r.read_u32()?;
        state.timer_initial = r.read_u32()?;
        state.timer_remaining = r.read_u64()?;
        state.timer_divide = r.read_u32()?;
        state.tick = r.read_u64()?;
        state.apic_base = r.read_u64()?;
        for level in &mut state.lint_level {
            *level = r.read_bool()?;
        }
        state.extint = r.read_bool()?;
        state.wait_for_sipi = r.read_bool()?;
        state.init_asserted = r.read_bool()?;
        state.init_pending = r.read_bool()?;
        state.startup = if r.read_bool()? {
            Some(r.read_u8()?)
        } else {
            None
        };
        // A divisor is read back out of this register on every timer step, and
        // bit 2 is reserved: a snapshot that set it would name a divisor the
        // hardware cannot select.
        if state.timer_divide & !0b1011 != 0 {
            return Err(Error::State(format!(
                "snapshot has an APIC timer divide configuration of {:#x}, which sets a reserved bit",
                state.timer_divide
            )));
        }
        {
            let mut current = self.regs.state.lock();
            *current = state;
            self.regs.publish(&current);
        }
        self.regs.refresh();
        Ok(())
    }
}

impl Instance for LocalApic {
    /// Claim the requester id of the processor this APIC belongs to.
    ///
    /// The whole of what makes the [`WINDOW_REGION`] page work, and the reason
    /// [`BindCtx::peer`](crate::machine::BindCtx::peer) exists: an object's
    /// requester id is allocated by declaration order, so a machine file names
    /// the *processor* (`cpu = cpu1`) and the id is looked up here.
    ///
    /// # Errors
    ///
    /// If `cpu` names nothing in this machine; if two local APICs on one bus
    /// name the same processor, which is a board that cannot mean anything; or
    /// if a board maps the architectural page and this APIC does not say whose
    /// it is — that last one being precisely the defect the page exists to
    /// remove, so it fails the build rather than answering with the wrong
    /// processor's registers.
    fn bind(&self, ctx: &BindCtx<'_>) -> Result<()> {
        let Some(cpu) = self.cpu.as_deref() else {
            if self.regs.bus.has_window() {
                return Err(Error::Config {
                    at: ctx.path().to_string(),
                    message: String::from(
                        "this board maps the architectural local-APIC page, so every `pc.lapic` \
                         on the bus has to say which processor it belongs to — add `cpu = <the \
                         processor's name>`, or the processor would read another one's registers",
                    ),
                });
            }
            return Ok(());
        };
        let peer = ctx.peer(cpu)?;
        let ops = Arc::downgrade(&self.regs) as Weak<dyn MemOps>;
        if !self.regs.bus.attach_local(peer.requester(), ops) {
            return Err(Error::Config {
                at: ctx.path().to_string(),
                message: format!(
                    "another local APIC on this bus already answers for `{cpu}`; a processor has \
                     one local APIC"
                ),
            });
        }
        Ok(())
    }
}

/// Add [`CLASS`] to a registry.
///
/// # Errors
///
/// [`Error::Config`] if the name is claimed.
pub fn register(registry: &mut crate::core::Registry) -> Result<()> {
    registry.add(&CLASS)
}

/// Bind [`CLASS`] into the machine graph.
///
/// # Errors
///
/// [`Error::Config`] if the class is bound twice.
pub fn bind(bindings: &mut crate::machine::Bindings) -> Result<()> {
    bindings.bind(CLASS_NAME, |props| Ok(Arc::new(LocalApic::new(props)?)))
}

/// What the validator should know about `pc.lapic`.
#[must_use]
pub fn schema() -> ClassSchema {
    use crate::machine::validate::{PortDir, PropSchema};
    ClassSchema::new(CLASS_NAME)
        .prop(PropSchema::new("id", ValueKind::Uint).range(0, 255))
        .prop(PropSchema::new("bsp", ValueKind::Bool))
        .prop(PropSchema::new("bus", ValueKind::Str))
        .prop(PropSchema::new("cpu", ValueKind::Link))
        .region("")
        .region("regs")
        .region(WINDOW_REGION)
        .port("intr", PortDir::Out)
        .port("nmi", PortDir::Out)
        .port("lint0", PortDir::In)
        .port("lint1", PortDir::In)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::space::RequesterId;
    use crate::core::state::{MachineShape, Migrations, StateReader, StateWriter};
    use crate::core::sync::{AtomicU32, Ordering as AtomicOrdering};
    use crate::core::wire::{Wire, WireIdAllocator};

    /// A vector a guest would actually use: above the sixteen the architecture
    /// keeps for itself.
    const TIMER_VECTOR: u8 = 0x40;

    /// The three local vector table entries these tests reach, at their own
    /// offsets rather than as indices into the array.
    const REG_LVT_TIMER: u64 = REG_LVT_BASE + LVT_TIMER as u64 * REG_STRIDE;
    const REG_LVT_LINT0: u64 = REG_LVT_BASE + LVT_LINT0 as u64 * REG_STRIDE;
    const REG_LVT_LINT1: u64 = REG_LVT_BASE + LVT_LINT1 as u64 * REG_STRIDE;

    #[derive(Debug, Default)]
    struct Probe {
        level: AtomicU32,
        edges: AtomicU32,
    }

    impl WireSink for Probe {
        fn set_level(&self, _src: WireId, _line: u32, level: Level) {
            self.level
                .store(u32::from(level.is_high()), AtomicOrdering::Relaxed);
            if level.is_high() {
                self.edges.fetch_add(1, AtomicOrdering::Relaxed);
            }
        }
    }

    impl Probe {
        fn high(&self) -> bool {
            self.level.load(AtomicOrdering::Relaxed) != 0
        }

        fn edges(&self) -> u32 {
            self.edges.load(AtomicOrdering::Relaxed)
        }
    }

    /// An 8259A-shaped stub: something that answers an acknowledge with a fixed
    /// vector, so `ExtINT` forwarding can be observed without a whole 8259A.
    #[derive(Debug)]
    struct Stub8259 {
        vector: u32,
        asked: AtomicU32,
    }

    impl IntAck for Stub8259 {
        fn acknowledge(&self, _cycle: IntAckCycle) -> IntAckResponse {
            self.asked.fetch_add(1, AtomicOrdering::Relaxed);
            IntAckResponse::Vector(self.vector)
        }
    }

    /// One APIC, its `INTR` and `NMI` wired to probes and both LINT pins driven.
    struct Bench {
        apic: LocalApic,
        lint: Vec<Arc<dyn WireSink>>,
        src: WireId,
        intr: Arc<Probe>,
        nmi: Arc<Probe>,
    }

    fn bench_on(bus: &Arc<ApicBus>, id: u8, bsp: bool) -> Bench {
        let apic = LocalApic::with_bus(id, bsp, Arc::clone(bus));
        let ids = WireIdAllocator::new();
        let src = ids.alloc();
        let lint: Vec<Arc<dyn WireSink>> = ["lint0", "lint1"]
            .iter()
            .map(|port| apic.sink(port, &[src]).expect("both LINT pins exist").sink)
            .collect();
        let intr = Arc::new(Probe::default());
        let nmi = Arc::new(Probe::default());
        for (port, probe) in [("intr", &intr), ("nmi", &nmi)] {
            let out = ids.alloc();
            let wire = Wire::builder()
                .source(out)
                .sink(Arc::clone(probe) as Arc<dyn WireSink>, 0)
                .build_shared();
            apic.connect(port, WireSource::new(wire, out))
                .expect("both output pins exist");
        }
        // What `realize` does. Called by hand because a unit test has no
        // machine to realize it, and delivery is meaningless until it happens.
        bus.attach(Arc::downgrade(&apic.regs) as Weak<dyn Target>);
        Bench {
            apic,
            lint,
            src,
            intr,
            nmi,
        }
    }

    fn bench() -> Bench {
        bench_on(&Arc::new(ApicBus::new()), 0, true)
    }

    impl Bench {
        fn poke(&self, offset: u64, value: u32) {
            self.apic
                .regs
                .write(offset, &value.to_le_bytes(), MemAttrs::DEFAULT)
                .expect("a 32-bit aligned write is legal");
        }

        fn peek(&self, offset: u64) -> u32 {
            self.peek_with(offset, MemAttrs::DEFAULT)
        }

        fn peek_with(&self, offset: u64, attrs: MemAttrs) -> u32 {
            let mut bytes = [0u8; 4];
            self.apic
                .regs
                .read(offset, &mut bytes, attrs)
                .expect("a 32-bit aligned read is legal");
            u32::from_le_bytes(bytes)
        }

        /// Software-enable the APIC, which is what firmware does before it can
        /// unmask anything.
        fn enable(&self) {
            self.poke(REG_SVR, SVR_ENABLE | 0xff);
        }

        fn drive_lint(&self, index: usize, level: Level) {
            self.lint[index].set_level(self.src, index as u32, level);
        }

        fn ack(&self) -> IntAckResponse {
            IntAck::acknowledge(&*self.apic.regs, IntAckCycle::vector_only())
        }
    }

    #[test]
    fn the_version_register_says_what_this_part_is() {
        let b = bench();
        let version = b.peek(REG_VERSION);
        assert_eq!(version & 0xff, VERSION, "an integrated APIC");
        assert_eq!(
            (version >> 16) & 0xff,
            LVT_COUNT as u32 - 1,
            "six local vector table entries, reported as the highest index"
        );
        assert_eq!(b.peek(REG_ID) >> 24, 0, "and the ID it was built with");
    }

    #[test]
    fn a_register_is_only_reachable_on_its_own_sixteen_byte_boundary() {
        let b = bench();
        let mut bytes = [0u8; 4];
        // "Any access that touches bytes 4 through 15 of an APIC register may
        // cause undefined behavior" (SDM Vol 3A §10.4.1).
        assert!(
            b.apic
                .regs
                .read(REG_VERSION + 4, &mut bytes, MemAttrs::DEFAULT)
                .is_err()
        );
        assert!(
            b.apic
                .regs
                .read(REG_VERSION, &mut [0u8; 1], MemAttrs::DEFAULT)
                .is_err(),
            "and a byte access is not a 32-bit load"
        );
    }

    #[test]
    fn the_timer_fires_on_the_tick_the_scheduler_was_told_about() {
        let b = bench();
        b.enable();
        b.poke(REG_LVT_TIMER, u32::from(TIMER_VECTOR));
        // Divide by 16: bits [3,1,0] = 011 (SDM Vol 3A §10.5.4).
        b.poke(REG_TIMER_DIV, 0b0011);
        b.poke(REG_TIMER_INIT, 100);

        // The whole point of a lazily advanced device: the tick the interrupt
        // lands on is a number the device *publishes*, computed from the count
        // and the divisor by exact integer arithmetic, and the scheduler stops
        // there. Nothing here consults a clock, and this assertion is what
        // would fail if it did -- the answer is 1600 because 100 x 16 is 1600,
        // not because 1600 of anything has elapsed anywhere.
        assert_eq!(Device::next_event_tick(&b.apic), Some(1600));
        assert_eq!(b.peek(REG_TIMER_CUR), 100, "and nothing has counted yet");

        b.apic.advance_to(1599);
        assert_eq!(b.peek(REG_TIMER_CUR), 1, "one count left");
        assert!(!b.intr.high(), "and no interrupt yet");

        b.apic.advance_to(1600);
        assert_eq!(b.peek(REG_TIMER_CUR), 0);
        assert!(b.intr.high(), "the count reached zero and INTR went up");
        assert_eq!(
            Device::next_event_tick(&b.apic),
            None,
            "a one-shot timer has nothing further to say"
        );
        assert_eq!(
            b.ack(),
            IntAckResponse::Vector(u32::from(TIMER_VECTOR)),
            "and the acknowledge cycle answers with the vector the LVT names"
        );
        assert!(!b.intr.high(), "which drops INTR, the request having moved");
    }

    #[test]
    fn the_timers_position_is_a_function_of_its_tick_and_nothing_else() {
        // The falsifiable form of "a device never reads the wall clock": the
        // current count is fully determined by the tick it was last advanced
        // to, so reading it a thousand times -- which takes real time -- moves
        // nothing at all.
        let b = bench();
        b.enable();
        b.poke(REG_TIMER_DIV, 0b1011); // divide by 1
        b.poke(REG_TIMER_INIT, 5_000);
        for _ in 0..1_000 {
            assert_eq!(b.peek(REG_TIMER_CUR), 5_000);
        }
        assert_eq!(b.apic.tick(), 0, "and the device has not moved either");
        b.apic.advance_to(1_234);
        assert_eq!(b.peek(REG_TIMER_CUR), 5_000 - 1_234);
    }

    #[test]
    fn a_periodic_timer_reloads_and_several_periods_collapse_into_one_request() {
        let b = bench();
        b.enable();
        b.poke(REG_LVT_TIMER, TIMER_PERIODIC | u32::from(TIMER_VECTOR));
        b.poke(REG_TIMER_DIV, 0b1011); // divide by 1
        b.poke(REG_TIMER_INIT, 10);
        assert_eq!(Device::next_event_tick(&b.apic), Some(10));

        b.apic.advance_to(10);
        assert!(b.intr.high());
        assert_eq!(
            Device::next_event_tick(&b.apic),
            Some(20),
            "and it re-arms for the next period"
        );

        // Twenty-five periods inside one step. The request bit is already set,
        // so they collapse -- which is what the hardware does with an interrupt
        // whose predecessor has not been serviced -- and the phase is preserved
        // exactly.
        b.apic.advance_to(263);
        assert_eq!(Device::next_event_tick(&b.apic), Some(270));
        assert_eq!(b.peek(REG_TIMER_CUR), 7);
    }

    #[test]
    fn a_masked_timer_still_counts_and_simply_does_not_interrupt() {
        let b = bench();
        b.enable();
        b.poke(REG_LVT_TIMER, LVT_MASK | u32::from(TIMER_VECTOR));
        b.poke(REG_TIMER_DIV, 0b1011);
        b.poke(REG_TIMER_INIT, 8);
        b.apic.advance_to(8);
        assert!(!b.intr.high(), "the mask blocks the interrupt");
        assert_eq!(b.peek(REG_TIMER_CUR), 0, "but not the counting");
    }

    #[test]
    fn a_debug_read_moves_nothing_and_a_debug_write_is_refused() {
        let b = bench();
        b.enable();
        b.poke(REG_LVT_TIMER, u32::from(TIMER_VECTOR));
        b.poke(REG_TIMER_DIV, 0b1011);
        b.poke(REG_TIMER_INIT, 4);
        b.apic.advance_to(4);
        b.ack();
        let debug = MemAttrs::DEBUG;
        // A debugger looking at the in-service register must not end the
        // interrupt it is looking at.
        assert_ne!(b.peek_with(REG_ISR + 2 * REG_STRIDE, debug), 0);
        assert!(
            b.apic
                .regs
                .write(REG_EOI, &0u32.to_le_bytes(), debug)
                .is_err(),
            "and there is no harmless write on this part"
        );
        assert_ne!(
            b.peek(REG_ISR + 2 * REG_STRIDE),
            0,
            "so the in-service bit is still there for the guest"
        );
    }

    #[test]
    fn nothing_pending_is_answered_with_the_spurious_vector() {
        let b = bench();
        b.poke(REG_SVR, SVR_ENABLE | 0xef);
        assert_eq!(b.ack(), IntAckResponse::Vector(0xef));
        assert_eq!(b.peek(REG_ISR + 7 * REG_STRIDE), 0, "and sets no ISR bit");
    }

    #[test]
    fn the_task_priority_holds_an_interrupt_off_until_it_is_lowered() {
        let bus = Arc::new(ApicBus::new());
        let b = bench_on(&bus, 0, true);
        b.enable();
        // Priority class 4 refuses everything in class 4 and below.
        b.poke(REG_TPR, 0x40);
        bus.deliver(Message::fixed(0x44, 0), None, Shorthand::Dest);
        assert_ne!(b.peek(REG_IRR + 2 * REG_STRIDE), 0, "the request is held");
        assert!(!b.intr.high(), "but not offered");
        b.poke(REG_TPR, 0x30);
        assert!(b.intr.high(), "and lowering the task priority offers it");
        assert_eq!(b.ack(), IntAckResponse::Vector(0x44));
        assert_eq!(
            b.peek(REG_PPR) & 0xf0,
            0x40,
            "the in-service vector now sets the processor priority"
        );
        b.poke(REG_EOI, 0);
        assert_eq!(b.peek(REG_PPR) & 0xf0, 0x30, "and the EOI gives it back");
    }

    #[test]
    fn a_vector_below_sixteen_is_refused_and_recorded() {
        let bus = Arc::new(ApicBus::new());
        let b = bench_on(&bus, 0, true);
        b.enable();
        bus.deliver(Message::fixed(0x0f, 0), None, Shorthand::Dest);
        assert_eq!(b.peek(REG_IRR), 0, "the architecture's own vectors are not");
        // The error is latched by a write and read afterwards, which is the
        // register's protocol (SDM Vol 3A §10.5.3).
        b.poke(REG_ESR, 0);
        assert_eq!(b.peek(REG_ESR), ESR_RECV_ILLEGAL_VECTOR);
    }

    #[test]
    fn an_extint_pin_forwards_the_acknowledge_to_the_controller_behind_it() {
        let b = bench();
        b.enable();
        let pic = Arc::new(Stub8259 {
            vector: 0x08,
            asked: AtomicU32::new(0),
        });
        b.apic
            .attach_int_ack("lint0", Arc::downgrade(&pic) as Weak<dyn IntAck>);
        // Virtual wire mode: LINT0 in ExtINT delivery, unmasked.
        b.poke(REG_LVT_LINT0, u32::from(Delivery::EXTINT.0) << 8);

        assert!(!b.intr.high());
        b.drive_lint(0, Level::High);
        assert!(b.intr.high(), "the 8259A's INT reaches the processor");
        assert_eq!(
            b.ack(),
            IntAckResponse::Vector(0x08),
            "and the vector comes from the 8259A, not from this part"
        );
        assert_eq!(pic.asked.load(AtomicOrdering::Relaxed), 1);
        assert_eq!(b.peek(REG_ISR), 0, "no local in-service bit is set");
    }

    #[test]
    fn a_nmi_lint_delivers_an_edge_and_a_fixed_one_a_vector() {
        let b = bench();
        b.enable();
        b.poke(REG_LVT_LINT1, u32::from(Delivery::NMI.0) << 8);
        b.drive_lint(1, Level::High);
        assert_eq!(b.nmi.edges(), 1, "one edge on the NMI pin");
        b.drive_lint(1, Level::Low);
        b.drive_lint(1, Level::High);
        assert_eq!(b.nmi.edges(), 2);

        b.poke(REG_LVT_LINT0, 0x33);
        b.drive_lint(0, Level::High);
        assert_eq!(b.ack(), IntAckResponse::Vector(0x33));
    }

    #[test]
    fn a_software_disabled_apic_masks_every_lvt_entry() {
        let b = bench();
        // Not enabled: SVR bit 8 is clear out of reset.
        b.poke(REG_LVT_LINT0, 0x33);
        assert_eq!(
            b.peek(REG_LVT_LINT0) & LVT_MASK,
            LVT_MASK,
            "the mask reads back set however it was written (SDM 10.4.7.2)"
        );
        b.drive_lint(0, Level::High);
        assert!(!b.intr.high(), "so nothing is delivered");
        b.enable();
        b.poke(REG_LVT_LINT0, 0x33);
        b.drive_lint(0, Level::Low);
        b.drive_lint(0, Level::High);
        assert!(b.intr.high(), "and enabling it makes the same pin work");
    }

    #[test]
    fn an_interprocessor_interrupt_reaches_the_apic_it_names() {
        let bus = Arc::new(ApicBus::new());
        let zero = bench_on(&bus, 0, true);
        let one = bench_on(&bus, 1, false);
        one.enable();

        // Destination 1, fixed delivery, vector 0x51: the ICR's high half first,
        // because writing the low half is what sends (SDM Vol 3A 10.6.1).
        zero.poke(REG_ICR_HIGH, 1 << 24);
        zero.poke(REG_ICR_LOW, 0x51);

        assert!(one.intr.high(), "the destination took it");
        assert!(!zero.intr.high(), "and the sender did not");
        assert_eq!(one.ack(), IntAckResponse::Vector(0x51));
    }

    #[test]
    fn the_shorthands_pick_out_the_sender_and_everyone_else() {
        let bus = Arc::new(ApicBus::new());
        let zero = bench_on(&bus, 0, true);
        let one = bench_on(&bus, 1, false);
        zero.enable();
        one.enable();

        zero.poke(REG_ICR_LOW, (Shorthand::SelfOnly.bits() << 18) | 0x61);
        assert!(zero.intr.high());
        assert!(!one.intr.high());
        assert_eq!(zero.ack(), IntAckResponse::Vector(0x61));
        zero.poke(REG_EOI, 0);

        zero.poke(REG_ICR_LOW, (Shorthand::AllButSelf.bits() << 18) | 0x62);
        assert!(!zero.intr.high());
        assert_eq!(one.ack(), IntAckResponse::Vector(0x62));
    }

    #[test]
    fn a_logical_destination_in_the_flat_model_is_a_bitmap() {
        let bus = Arc::new(ApicBus::new());
        let zero = bench_on(&bus, 0, true);
        let one = bench_on(&bus, 1, false);
        for (b, ldr) in [(&zero, 0x01u32), (&one, 0x02)] {
            b.enable();
            b.poke(REG_DFR, 0xf << 28);
            b.poke(REG_LDR, ldr << 24);
        }
        // Bit 1 only: APIC 1.
        zero.poke(REG_ICR_HIGH, 0x02 << 24);
        zero.poke(REG_ICR_LOW, (1 << 11) | 0x71);
        assert!(!zero.intr.high());
        assert!(one.intr.high());

        // Both bits: a broadcast to the two of them.
        zero.poke(REG_ICR_HIGH, 0x03 << 24);
        zero.poke(REG_ICR_LOW, (1 << 11) | 0x72);
        assert!(zero.intr.high());
    }

    #[test]
    fn lowest_priority_picks_the_processor_bidding_least() {
        let bus = Arc::new(ApicBus::new());
        let zero = bench_on(&bus, 0, true);
        let one = bench_on(&bus, 1, false);
        zero.enable();
        one.enable();
        // Processor 0 is busy at priority class 8, processor 1 is idle.
        zero.poke(REG_TPR, 0x80);
        zero.poke(REG_ICR_HIGH, 0xff << 24);
        zero.poke(REG_ICR_LOW, (u32::from(Delivery::LOWEST.0) << 8) | 0x73);
        assert!(one.intr.high(), "the idle one takes it");
        assert_eq!(zero.peek(REG_IRR + 3 * REG_STRIDE), 0);
    }

    #[test]
    fn init_then_start_up_leaves_a_processor_with_a_page_to_start_at() {
        let bus = Arc::new(ApicBus::new());
        let bsp = bench_on(&bus, 0, true);
        let ap = bench_on(&bus, 1, false);
        ap.enable();
        ap.poke(REG_TPR, 0x50);

        // The MP specification's universal startup algorithm (B.4): an INIT
        // level assert, an INIT level de-assert, then a Start-Up naming the
        // page the processor begins executing at.
        bsp.poke(REG_ICR_HIGH, 1 << 24);
        let init = (u32::from(Delivery::INIT.0) << 8) | (1 << 14) | (1 << 15);
        bsp.poke(REG_ICR_LOW, init);
        assert!(ap.apic.waiting_for_startup(), "the AP is held at INIT");
        assert!(ap.apic.init_asserted());
        assert_eq!(ap.peek(REG_TPR), 0, "and its APIC came back reset");
        assert_eq!(ap.apic.id(), 1, "except for its ID (SDM 10.4.7.1)");

        bsp.poke(REG_ICR_LOW, init & !(1 << 14));
        assert!(!ap.apic.init_asserted(), "the de-assert drops the line");
        assert!(ap.apic.waiting_for_startup(), "but it is still waiting");

        // Start-Up: the vector is the page, so 0x08 means 0x8000.
        bsp.poke(REG_ICR_LOW, (u32::from(Delivery::STARTUP.0) << 8) | 0x08);
        assert!(!ap.apic.waiting_for_startup());
        assert_eq!(ap.apic.take_startup(), Some(0x08));
        assert_eq!(ap.apic.take_startup(), None, "and it is taken once");

        // The second Start-Up the algorithm always sends is ignored, because
        // the processor is no longer waiting for one.
        bsp.poke(REG_ICR_LOW, (u32::from(Delivery::STARTUP.0) << 8) | 0x08);
        assert_eq!(ap.apic.take_startup(), None);
    }

    #[test]
    fn the_processor_takes_the_whole_sequence_through_its_controller() {
        // What a processor actually sees, and it is the same three messages as
        // the test above — asked for through `LocalController` rather than
        // through this device's own accessors, because that is the seam a core
        // is wired to.
        let bus = Arc::new(ApicBus::new());
        let bsp = bench_on(&bus, 0, true);
        let ap = bench_on(&bus, 1, false);
        let link = ap
            .apic
            .local_controller("intr")
            .expect("a local APIC is its processor's own controller");
        assert!(
            ap.apic.local_controller("lint0").is_none(),
            "and only on the pin that drives INTR"
        );
        assert_eq!(link.take_startup(), Startup::NONE, "nothing has happened");

        bsp.poke(REG_ICR_HIGH, 1 << 24);
        let init = (u32::from(Delivery::INIT.0) << 8) | (1 << 14) | (1 << 15);
        bsp.poke(REG_ICR_LOW, init);
        assert_eq!(
            link.take_startup(),
            Startup {
                init: true,
                held: true,
                page: None
            },
            "an INIT to run the sequence for, and the line still holding it"
        );
        assert_eq!(
            link.take_startup(),
            Startup {
                init: false,
                held: true,
                page: None
            },
            "the edge is reported once; the level is reported while it lasts"
        );

        bsp.poke(REG_ICR_LOW, init & !(1 << 14));
        assert_eq!(link.take_startup(), Startup::NONE, "the line dropped");

        bsp.poke(REG_ICR_LOW, (u32::from(Delivery::STARTUP.0) << 8) | 0x08);
        assert_eq!(
            link.take_startup(),
            Startup {
                init: false,
                held: false,
                page: Some(0x08)
            }
        );
        assert_eq!(link.take_startup(), Startup::NONE, "and taken once");
    }

    #[test]
    fn a_processor_that_was_not_asking_sees_the_whole_sequence_at_once() {
        // The case `Startup`'s three separate fields exist for: an application
        // processor is not executing while the bootstrap processor sends all
        // three messages, so its first ask is also its last.
        let bus = Arc::new(ApicBus::new());
        let bsp = bench_on(&bus, 0, true);
        let ap = bench_on(&bus, 1, false);
        bsp.poke(REG_ICR_HIGH, 1 << 24);
        let init = (u32::from(Delivery::INIT.0) << 8) | (1 << 14) | (1 << 15);
        bsp.poke(REG_ICR_LOW, init);
        bsp.poke(REG_ICR_LOW, init & !(1 << 14));
        bsp.poke(REG_ICR_LOW, (u32::from(Delivery::STARTUP.0) << 8) | 0x08);

        let link = ap.apic.local_controller("intr").expect("its controller");
        assert_eq!(
            link.take_startup(),
            Startup {
                init: true,
                held: false,
                page: Some(0x08)
            }
        );
    }

    #[test]
    fn a_reset_parks_an_application_processor_and_leaves_the_bootstrap_one_running() {
        // The MP initialization protocol at power-up (SDM Vol 3A 8.4.3): the
        // processor that loses it never fetches an instruction. This part is the
        // half of the pair that knows which one it is in front of.
        let bus = Arc::new(ApicBus::new());
        let bsp = bench_on(&bus, 0, true);
        let ap = bench_on(&bus, 1, false);
        bsp.apic.reset(ResetKind::Cold);
        ap.apic.reset(ResetKind::Cold);

        assert!(!bsp.apic.waiting_for_startup());
        assert_eq!(
            bsp.apic
                .local_controller("intr")
                .expect("its controller")
                .take_startup(),
            Startup::NONE,
            "the bootstrap processor is told nothing and runs the reset vector"
        );

        assert!(ap.apic.waiting_for_startup());
        let link = ap.apic.local_controller("intr").expect("its controller");
        assert_eq!(
            link.take_startup(),
            Startup {
                init: true,
                held: false,
                page: None
            },
            "an INIT with no line behind it: run the sequence and wait"
        );
        assert_eq!(link.take_startup(), Startup::NONE);
    }

    #[test]
    fn the_base_register_a_processor_reads_is_this_parts_own() {
        // `IA32_APIC_BASE` is reached by `RDMSR`, which is a *processor*
        // instruction naming state that lives here (SDM Vol 3A 10.4.3).
        let bench = bench_on(&Arc::new(ApicBus::new()), 0, true);
        let link = bench.apic.local_controller("intr").expect("its controller");
        assert_eq!(link.base_register(), bench.apic.apic_base());
        assert_eq!(
            link.base_register() & APIC_BASE_BSP,
            APIC_BASE_BSP,
            "and it says this is the bootstrap processor"
        );

        link.set_base_register(link.base_register() & !APIC_BASE_ENABLE);
        assert_eq!(bench.apic.apic_base() & APIC_BASE_ENABLE, 0);
        assert!(
            !bench.apic.regs.state.lock().hardware_enabled(),
            "and a hardware-disabled APIC is transparent"
        );
    }

    #[test]
    fn a_snapshot_round_trips_the_whole_part() {
        let bus = Arc::new(ApicBus::new());
        let saved = bench_on(&bus, 3, false);
        saved.enable();
        saved.poke(REG_TPR, 0x20);
        saved.poke(REG_LDR, 0x08 << 24);
        saved.poke(REG_LVT_TIMER, TIMER_PERIODIC | u32::from(TIMER_VECTOR));
        saved.poke(REG_TIMER_DIV, 0b0001); // divide by 4
        saved.poke(REG_TIMER_INIT, 250);
        saved.poke(REG_LVT_LINT0, 0x39);
        saved.drive_lint(0, Level::High);
        saved.apic.advance_to(1_003);
        saved.ack();

        let mut shape = MachineShape::new();
        shape.add_device("lapic", CLASS.name).unwrap();
        let mut w = StateWriter::new(shape);
        {
            let mut chunk = w.chunk("lapic", CLASS.name, CLASS.version).unwrap();
            saved.apic.save(&mut chunk).unwrap();
        }
        let bytes = w.to_vec().unwrap();

        let restored = bench_on(&Arc::new(ApicBus::new()), 0, true);
        let reader = StateReader::new(&bytes).unwrap();
        let chunk = reader
            .load("lapic", CLASS.name, CLASS.version, &Migrations::new())
            .unwrap();
        restored.apic.load(&mut chunk.reader()).unwrap();

        // Copied out one at a time: two parts' state locks are both at
        // `LockRank::DEVICE`, and holding one while taking the other is the
        // rank violation `core::sync` exists to catch.
        let after = restored.apic.regs.state.lock().clone();
        let before = saved.apic.regs.state.lock().clone();
        assert_eq!(after, before, "every field came back");
        assert_eq!(
            restored.apic.tick(),
            1_003,
            "the position in its domain too"
        );
        assert_eq!(
            Device::next_event_tick(&restored.apic),
            Device::next_event_tick(&saved.apic),
            "so the scheduler is told the same next event"
        );

        // The bytes of a second save are the bytes of the first, which is the
        // state hash this rule is really about.
        let mut shape = MachineShape::new();
        shape.add_device("lapic", CLASS.name).unwrap();
        let mut w = StateWriter::new(shape);
        {
            let mut chunk = w.chunk("lapic", CLASS.name, CLASS.version).unwrap();
            restored.apic.save(&mut chunk).unwrap();
        }
        assert_eq!(w.to_vec().unwrap(), bytes);
    }

    #[test]
    fn a_reset_stops_the_timer_and_drops_intr() {
        let b = bench();
        b.enable();
        b.poke(REG_LVT_TIMER, u32::from(TIMER_VECTOR));
        b.poke(REG_TIMER_DIV, 0b1011);
        b.poke(REG_TIMER_INIT, 3);
        b.apic.advance_to(3);
        assert!(b.intr.high());
        b.apic.reset(ResetKind::Cold);
        assert!(!b.intr.high());
        assert_eq!(Device::next_event_tick(&b.apic), None);
        assert_eq!(b.peek(REG_IRR + 2 * REG_STRIDE), 0);
    }

    // -----------------------------------------------------------------
    // the architectural page
    // -----------------------------------------------------------------

    /// The requester ids two processors would be allocated. Arbitrary and
    /// non-adjacent on purpose: nothing about the decode may depend on them
    /// being 1 and 2, which is what a machine that declared the APICs first
    /// would give them.
    const CPU0: RequesterId = RequesterId(7);
    const CPU1: RequesterId = RequesterId(9);

    /// Two APICs on one bus with a window over them, each having claimed its
    /// processor the way [`Instance::bind`] does.
    fn windowed() -> (Arc<ApicBus>, ApicWindow, Bench, Bench) {
        let bus = Arc::new(ApicBus::new());
        let a = bench_on(&bus, 0, true);
        let b = bench_on(&bus, 1, false);
        assert!(bus.attach_local(CPU0, Arc::downgrade(&a.apic.regs) as Weak<dyn MemOps>));
        assert!(bus.attach_local(CPU1, Arc::downgrade(&b.apic.regs) as Weak<dyn MemOps>));
        let window = ApicWindow {
            bus: Arc::clone(&bus),
            fallback: Arc::clone(&a.apic.regs),
        };
        (bus, window, a, b)
    }

    /// A 32-bit read of `offset` through `window`, as `requester` would make it.
    fn through(window: &ApicWindow, offset: u64, requester: RequesterId) -> u32 {
        let mut bytes = [0u8; 4];
        window
            .read(
                offset,
                &mut bytes,
                MemAttrs::DEFAULT.with_requester(requester),
            )
            .expect("the window answers");
        u32::from_le_bytes(bytes)
    }

    #[test]
    fn one_page_gives_each_processor_its_own_apic() {
        let (_bus, window, _a, _b) = windowed();
        // The whole claim, in four lines: the same offset, two initiators, two
        // answers (SDM Vol 3A §10.4.6, the local APIC ID register).
        assert_eq!(through(&window, REG_ID, CPU0) >> 24, 0);
        assert_eq!(through(&window, REG_ID, CPU1) >> 24, 1);
    }

    #[test]
    fn an_access_from_nothing_on_the_bus_reaches_the_apic_that_published_it() {
        let (_bus, window, _a, _b) = windowed();
        // A debugger, a DMA engine, a snapshot: none of them is a processor and
        // none of them has an APIC, so the page they see is the one the board
        // put the window on — the bootstrap processor's.
        assert_eq!(through(&window, REG_ID, RequesterId::ANONYMOUS) >> 24, 0);
        assert_eq!(through(&window, REG_ID, RequesterId(4242)) >> 24, 0);
    }

    #[test]
    fn a_write_through_the_window_lands_on_the_writers_own_apic() {
        let (_bus, window, a, b) = windowed();
        // The task-priority register, because it is per-processor state a real
        // scheduler writes on every context switch (SDM Vol 3A §10.8.3.1) —
        // the write that was silently going to the wrong APIC.
        window
            .write(
                REG_TPR,
                &0x40u32.to_le_bytes(),
                MemAttrs::DEFAULT.with_requester(CPU1),
            )
            .expect("the window takes it");
        assert_eq!(through(&window, REG_TPR, CPU1), 0x40);
        assert_eq!(through(&window, REG_TPR, CPU0), 0);
        assert_eq!(b.apic.regs.state.lock().tpr, 0x40);
        assert_eq!(a.apic.regs.state.lock().tpr, 0);
    }

    #[test]
    fn a_debug_access_keeps_its_attributes_through_the_window() {
        let (_bus, window, _a, _b) = windowed();
        // Passed through untouched, so the register block's own rules still
        // apply: a debug read is answered and a debug *write* is refused,
        // because there is no harmless write to an APIC (`ROADMAP.md` §15,
        // invariant 5).
        let mut bytes = [0u8; 4];
        window
            .read(REG_ID, &mut bytes, MemAttrs::DEBUG.with_requester(CPU1))
            .expect("a debug read is answered");
        assert_eq!(u32::from_le_bytes(bytes) >> 24, 1);
        assert!(
            window
                .write(
                    REG_EOI,
                    &0u32.to_le_bytes(),
                    MemAttrs::DEBUG.with_requester(CPU1),
                )
                .is_err()
        );
    }

    #[test]
    fn a_processor_cannot_have_two_local_apics() {
        let bus = Arc::new(ApicBus::new());
        let a = bench_on(&bus, 0, true);
        let b = bench_on(&bus, 1, false);
        assert!(bus.attach_local(CPU0, Arc::downgrade(&a.apic.regs) as Weak<dyn MemOps>));
        // Two `pc.lapic` objects naming one processor is a board that cannot
        // mean anything, and `bind` turns this into a configuration error
        // naming both rather than picking whichever bound first.
        assert!(!bus.attach_local(CPU0, Arc::downgrade(&b.apic.regs) as Weak<dyn MemOps>));
        assert_eq!(bus.local_count(), 1);
    }

    #[test]
    fn asking_for_the_window_is_what_tells_the_bus_it_has_one() {
        let bus = Arc::new(ApicBus::new());
        let apic = LocalApic::with_bus(0, true, Arc::clone(&bus));
        assert!(!bus.has_window(), "nothing has asked for the aperture");
        assert!(apic.region("regs").is_some());
        assert!(
            !bus.has_window(),
            "the register page is not the architectural page"
        );
        // A `map` statement resolving `lapicN.window` is the only route to one,
        // and it happens before any device is bound — which is what lets
        // `bind` insist that every APIC on the bus knows its processor.
        assert!(apic.region(WINDOW_REGION).is_some());
        assert!(bus.has_window());
    }
}
