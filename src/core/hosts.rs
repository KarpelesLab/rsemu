//! Host objects: the things a device is handed at construction that are not
//! data.
//!
//! A machine description carries *data* — numbers, strings, media slots. It
//! cannot carry an `Arc<CharPort>`, because there is nowhere in a text file to
//! write one. So the loop is closed by **name**, exactly as it is for a ROM
//! image ([`MediaTable`](crate::machine::MediaTable)):
//!
//! ```text
//! machine file:  object pia "apple1.pia" { port = "console" }   # names a port
//! caller:        let hosts = HostObjects::new();                # owns the table
//! construct:     props.host(chardev::KIND, "console", CharPort::new)
//! host:          ports::open(&hosts, "console")   ──► the same Arc<CharPort>
//! ```
//!
//! # Why this is a table and not a `static`
//!
//! It used to be several statics — one per kind of host object — and that was
//! the bug this module exists to remove. A process-wide table means two
//! machines of the same kind built in one process share every port, pad, signal
//! and bus they open, because both resolve the same names. Nothing said so; the
//! tests that would have noticed had been serialised against each other for
//! unrelated reasons, so the sharing looked like it worked.
//!
//! A `HostObjects` belongs to **one build**. It hangs off
//! [`RealizeOptions`](crate::machine::RealizeOptions), reaches a device through
//! [`Props::host`](crate::core::props::Props::host), and is reachable
//! afterwards by whoever owns the options. Two builds means two tables means
//! two of everything, by construction rather than by convention.
//!
//! # Which phase opens one
//!
//! **`new(props)`, not `realize(ctx)`.** `CLAUDE.md`'s two-phase rule says
//! `new` validates and allocates and `realize` performs every *outward* action,
//! and opening a host object is allocation: get-or-create of a passive object
//! whose table the caller already owns. It calls into no sibling device, it is
//! invisible to the guest, and it is the same seam media already uses — a
//! cartridge parses its ROM image in `new` for exactly this reason. Making it a
//! realize-time action would also force every device that holds a port to keep
//! it behind interior mutability, adding a lock to a path a UART reads on every
//! register access, in exchange for nothing.
//!
//! *Announcing yourself* into a table others read is the opposite case and does
//! belong in realize: see [`dev::riscv::dt`](crate::dev::riscv::dt), which
//! publishes a device-tree describer from `Device::realize`.
//!
//! # The table is also where the record/replay seam is enforced
//!
//! A host object is the *only* door a **device** has onto the host, so checking
//! every object against a recorder's registered channels is checking every
//! non-deterministic input a device can take, once, in one place, rather than
//! trusting each device to declare its own. [`HostObjects::seal`] is that check
//! and [`core::record`](crate::core::record) argues why it belongs here.
//!
//! Not every entry is such a door, which is the thing that kept the check
//! switched off. A PCI fabric, a drive bay and an APIC bus are filed here too,
//! and they are how two ends of one *build* find each other — nothing crosses
//! from the host at all — so a seal that demanded a channel for one refused
//! every board above the smallest. [`HostKind`] carries the distinction now,
//! and its own documentation has the four cases.
//!
//! [`machine::realize`](mod@crate::machine::realize) is what turns it on:
//! given a recorder it seals between constructing the devices and realizing
//! them, which is late enough that the doors are open and early enough that
//! nothing has acted.
//!
//! # Ordering
//!
//! A `BTreeMap` keyed by `(kind, name)`, so [`HostObjects::names`] is in name
//! order rather than hash order (`CLAUDE.md`, determinism).

use alloc::collections::BTreeMap;
use alloc::collections::btree_map::Entry;
use alloc::format;
use alloc::string::{String, ToString};
use alloc::sync::Arc;
use alloc::vec::Vec;
use core::any::Any;
use core::fmt;

use crate::core::error::{Error, Result};
use crate::core::record::{Channel, InputSink, Recorder};
use crate::core::sync::{LockRank, Mutex};

/// What sort of host object a name refers to.
///
/// A newtype over a `&'static str` rather than a Rust `enum`, because the kinds
/// are declared by the feature-gated modules that own them:
/// [`chardev::ports`](crate::host::chardev::ports) declares `chardev`, the NES
/// input port declares `pad`, and a build with neither has no opinion about
/// either.
///
/// The kind also fixes the *type* stored under it. Two modules must not claim
/// one kind name; [`HostObjects::open`] reports the collision rather than
/// handing back a pad where a character port was wanted.
///
/// # A kind carries its role, and the default is the strict one
///
/// One name space held three unrelated things and the
/// [seal](HostObjects::seal) checked all of them alike, so sealing any board
/// with a PCI bus in it failed on an object that was never an input. The role
/// is now part of the kind:
///
/// | constructor | what it means | a sealed table |
/// | --- | --- | --- |
/// | [`new`](HostKind::new) | a door, with no [`InputSink`] to put a recorded payload back through | **refuses it**, saying what to add |
/// | [`door`](HostKind::door) | a door that knows how to feed itself | wires it to the recorder |
/// | [`rendezvous`](HostKind::rendezvous) | two ends of one build meeting; nothing non-deterministic crosses in | ignores it |
/// | [`pulled`](HostKind::pulled) | host bytes that the *guest* asks for, a sector at a time | ignores it, and it stays a hole |
///
/// **[`new`] is a door on purpose.** A kind whose author has not thought about
/// the record/replay seam is exactly the kind that should stop a recorded
/// build, and the message names the two functions to write. Marking a kind as
/// carrying no input is a deliberate act, not a default.
///
/// [`new`]: HostKind::new
#[derive(Clone, Copy)]
pub struct HostKind {
    /// The name, and the whole of this type's identity — see the `PartialEq`
    /// impl below.
    name: &'static str,
    /// What the record/replay seam should do about objects filed under it.
    role: Role,
}

/// Turn a stored host object into the [`InputSink`] a recorded payload is
/// delivered through.
///
/// A plain `fn` pointer so a [`HostKind`] stays a `const`, and it takes the
/// erased `Arc` the table holds because `core::hosts` cannot name a `CharPort`
/// or a `NetPort` — the module that declares the kind does the downcast, which
/// is the same module that already ships `open()` and `sink()`.
///
/// `None` back means the object under that name is not the type the kind's
/// owner expected, which is the collision [`HostObjects::open`] reports.
pub type SinkFactory = fn(&Arc<dyn Any + Send + Sync>) -> Option<Arc<dyn InputSink>>;

/// What the record/replay seam does about a kind. See [`HostKind`].
#[derive(Clone, Copy)]
enum Role {
    /// Host input crosses here as `(instant, payload)`, with a way to put a
    /// recorded payload back if the declaring module supplied one.
    Door(Option<SinkFactory>),
    /// Two ends of one build finding each other. Nothing crosses from the host.
    Rendezvous,
    /// Host bytes the guest pulls rather than receives.
    Pulled,
}

// By hand, because deriving it would print the sink factory's *address* — a
// number that moves between builds, into whatever failing test or diagnostic
// printed the kind. The role's word is the useful half anyway.
impl fmt::Debug for HostKind {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let role = match self.role {
            Role::Door(Some(_)) => "door",
            Role::Door(None) => "door, no sink",
            Role::Rendezvous => "rendezvous",
            Role::Pulled => "pulled",
        };
        write!(f, "HostKind({}, {role})", self.name)
    }
}

// Identity is the **name alone**, deliberately and by hand rather than by
// derive. The table keys on `(HostKind, String)` and a channel is built from
// `kind.as_str()`, so a kind that compared its role too would file
// `HostKind::new("pad")` and `HostKind::door("pad", …)` under two different
// slots and a lookup would miss. Two modules declaring one kind name must agree
// about its role; they cannot disagree about its identity.
impl PartialEq for HostKind {
    fn eq(&self, other: &HostKind) -> bool {
        self.name == other.name
    }
}

impl Eq for HostKind {}

impl PartialOrd for HostKind {
    fn partial_cmp(&self, other: &HostKind) -> Option<core::cmp::Ordering> {
        Some(self.cmp(other))
    }
}

impl Ord for HostKind {
    fn cmp(&self, other: &HostKind) -> core::cmp::Ordering {
        self.name.cmp(other.name)
    }
}

impl core::hash::Hash for HostKind {
    fn hash<H: core::hash::Hasher>(&self, state: &mut H) {
        self.name.hash(state);
    }
}

impl HostKind {
    /// The kind a build's [`Captured`] tables are filed under, keyed by the
    /// name of the class whose constructor was intercepted.
    ///
    /// One well-known kind rather than one per host module: the class name
    /// already separates a captured PPU from a captured VDP, and a shared kind
    /// means a host can ask what a build captured without knowing which module
    /// installed the interception.
    ///
    /// A **rendezvous**, which is the one classification here that is a
    /// judgement rather than a reading. What a capture table carries out is a
    /// framebuffer or an audio ring, and that is output. What a frontend
    /// pushes *in* through one — `host::input::MouseSink` moving a captured HID
    /// mouse — is recorded on the frontend's own channel (`input:vnc`), not on
    /// one named after the capture, because a frontend is not part of the
    /// machine. Calling it a door would demand a `capture:nes.ppu` channel that
    /// nothing could ever sink, and no board with a screen could be recorded.
    pub const CAPTURE: HostKind = HostKind::rendezvous("capture");

    /// A kind whose role nobody has stated: **a door with no sink**.
    ///
    /// The strict default. A sealed table refuses an object filed under one,
    /// because a kind nobody has classified is a kind nobody has thought about
    /// the recording of. Fixing that is either
    /// [`door`](HostKind::door) — ship a `channel()` and a `sink()` beside the
    /// `open()`, ten lines — or [`rendezvous`](HostKind::rendezvous), if
    /// nothing non-deterministic crosses here at all.
    #[must_use]
    pub const fn new(name: &'static str) -> HostKind {
        HostKind {
            name,
            role: Role::Door(None),
        }
    }

    /// A door that can put a recorded payload back where it came from.
    ///
    /// `sink` is what [`HostObjects::seal`] calls to wire the object to the
    /// recorder, so a board whose console or joypad the caller did not think to
    /// declare is recorded anyway rather than refused.
    #[must_use]
    pub const fn door(name: &'static str, sink: SinkFactory) -> HostKind {
        HostKind {
            name,
            role: Role::Door(Some(sink)),
        }
    }

    /// A kind by which two ends of one build find each other.
    ///
    /// A PCI fabric, a drive bay, an APIC bus, a device-tree registry, a power
    /// signal the guest asserts outward. Nothing non-deterministic crosses into
    /// the machine here, so there is nothing to record and a sealed table has
    /// no business refusing one.
    #[must_use]
    pub const fn rendezvous(name: &'static str) -> HostKind {
        HostKind {
            name,
            role: Role::Rendezvous,
        }
    }

    /// A door whose bytes the guest **pulls**, so no `(instant, payload)` log
    /// describes it.
    ///
    /// `medium` is the only one: a drive's image really is host state crossing
    /// into a machine, but the guest asks for a sector when it wants one rather
    /// than receiving it at an instant. What that needs is an identity check on
    /// the image — `dev::medium`'s `Snapshot::Reference` — not a channel. A
    /// sealed table passes it, and [`core::record`](crate::core::record)'s
    /// table of what is covered still lists it as a hole, which it is.
    #[must_use]
    pub const fn pulled(name: &'static str) -> HostKind {
        HostKind {
            name,
            role: Role::Pulled,
        }
    }

    /// The name, for a diagnostic.
    #[must_use]
    pub const fn as_str(self) -> &'static str {
        self.name
    }

    /// Whether host input crosses at objects of this kind, so that a sealed
    /// table demands a channel for each of them.
    #[must_use]
    pub const fn is_door(self) -> bool {
        matches!(self.role, Role::Door(_))
    }

    /// How to build the [`InputSink`] for an object of this kind, if the module
    /// that declared it said.
    #[must_use]
    pub const fn sink_factory(self) -> Option<SinkFactory> {
        match self.role {
            Role::Door(sink) => sink,
            Role::Rendezvous | Role::Pulled => None,
        }
    }
}

impl fmt::Display for HostKind {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(self.name)
    }
}

/// One build's host objects, by kind and name.
///
/// Cheap to share: hand out `Arc<HostObjects>` and every holder sees the same
/// table. Two tables share nothing, which is the whole point — see the module
/// documentation.
pub struct HostObjects {
    /// [`LockRank::LEAF`]: nothing is ever locked while this is held.
    /// [`HostObjects::open`] deliberately builds the new object *outside* the
    /// critical section so that stays true even when the object's own
    /// constructor takes a lock.
    entries: Mutex<BTreeMap<(HostKind, String), Arc<dyn Any + Send + Sync>>>,
    /// The record/replay seal, if one has been applied. Read before `entries`
    /// is touched, never while it is held.
    policy: Mutex<InputPolicy>,
}

/// What a table does about host objects the record/replay seam does not know.
///
/// The enforcement `CLAUDE.md`'s determinism rule needs and could not have
/// before [`record`](crate::core::record) existed. A host object is the only
/// door from the host into a machine — a character port, a pad, a NIC's port,
/// and even the [`Captured`] table a host uses to reach a concrete device — so
/// checking every object against a recorder's registered channels is checking
/// every non-deterministic input, once, in one place.
///
/// Not an extensible enumeration: this is a two-state policy about one table,
/// and a third state would be a design change rather than an addition.
#[derive(Debug, Clone)]
pub enum InputPolicy {
    /// Anything may be opened. The default, and what a machine with no
    /// recording attached wants.
    Open,
    /// Only objects whose channel `recorder` has registered may be opened.
    ///
    /// Opening anything else is [`Error::Config`] naming the channel, at build
    /// time — see [`HostObjects::seal`].
    Sealed(Arc<Recorder>),
}

impl Default for HostObjects {
    fn default() -> HostObjects {
        HostObjects::new()
    }
}

impl fmt::Debug for HostObjects {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        // Read before `entries` is held: two leaf locks may not be nested, and
        // `try_lock` being exempt from the order check does not exempt what is
        // taken *inside* it.
        let sealed = self.is_sealed();
        match self.entries.try_lock() {
            // The values are `dyn Any` and have no useful `Debug`; the keys are
            // what the reader of a failing test wants anyway.
            Some(table) => f
                .debug_struct("HostObjects")
                .field(
                    "objects",
                    &table
                        .keys()
                        .map(|(kind, name)| format!("{kind}:{name}"))
                        .collect::<Vec<_>>(),
                )
                .field("sealed", &sealed)
                .finish(),
            None => f
                .debug_struct("HostObjects")
                .field("objects", &"<in use>")
                .field("sealed", &sealed)
                .finish(),
        }
    }
}

impl HostObjects {
    /// An empty table.
    #[must_use]
    pub fn new() -> HostObjects {
        HostObjects {
            entries: Mutex::with_rank(LockRank::LEAF, BTreeMap::new()),
            policy: Mutex::with_rank(LockRank::LEAF, InputPolicy::Open),
        }
    }

    /// The object called `name` under `kind`, creating it on first mention.
    ///
    /// Both ends call this: the device from `new(props)`, the host before it
    /// starts pumping bytes or pressing buttons. Whichever runs first makes the
    /// object; the other gets the same `Arc`.
    ///
    /// `make` runs with no lock held, so it may allocate and lock freely — and
    /// it may run and then be discarded, if another thread wins the race to
    /// insert. Whoever wins, every holder of the name has one object.
    ///
    /// # Errors
    ///
    /// [`Error::Config`] if `kind` already holds something of another type
    /// under `name` — two modules claiming one kind name, which is a bug in the
    /// build rather than in the machine file.
    pub fn open<T, F>(&self, kind: HostKind, name: &str, make: F) -> Result<Arc<T>>
    where
        T: Any + Send + Sync,
        F: FnOnce() -> T,
    {
        self.check_policy(kind, name)?;
        if let Some(found) = self.get::<T>(kind, name)? {
            return Ok(found);
        }
        let fresh = Arc::new(make());
        let mut table = self.entries.lock();
        match table.entry((kind, name.to_string())) {
            Entry::Occupied(slot) => downcast(kind, name, Arc::clone(slot.get())),
            Entry::Vacant(slot) => {
                slot.insert(Arc::clone(&fresh) as Arc<dyn Any + Send + Sync>);
                Ok(fresh)
            }
        }
    }

    /// The object called `name` under `kind`, if it has been opened.
    ///
    /// # Errors
    ///
    /// As [`HostObjects::open`], for a type mismatch.
    pub fn get<T: Any + Send + Sync>(&self, kind: HostKind, name: &str) -> Result<Option<Arc<T>>> {
        let found = {
            let table = self.entries.lock();
            table.get(&(kind, name.to_string())).map(Arc::clone)
        };
        found.map(|any| downcast(kind, name, any)).transpose()
    }

    /// Put `object` under `kind` and `name`, replacing anything already there.
    ///
    /// For a host that wants to supply the object rather than let the device
    /// create it — a scripted terminal, a replayed pad.
    ///
    /// # Panics
    ///
    /// Never. On a [sealed](HostObjects::seal) table an unregistered channel is
    /// refused by [`HostObjects::try_insert`]; this convenience form ignores
    /// that refusal, which is right for the caller that sealed the table in the
    /// first place and knows what it registered.
    pub fn insert<T: Any + Send + Sync>(&self, kind: HostKind, name: &str, object: Arc<T>) {
        let _ = self.try_insert(kind, name, object);
    }

    /// [`HostObjects::insert`], reporting a policy refusal instead of ignoring
    /// it.
    ///
    /// # Errors
    ///
    /// [`Error::Config`] if the table is sealed and no recorder channel matches
    /// `(kind, name)`.
    pub fn try_insert<T: Any + Send + Sync>(
        &self,
        kind: HostKind,
        name: &str,
        object: Arc<T>,
    ) -> Result<()> {
        self.check_policy(kind, name)?;
        self.entries.lock().insert(
            (kind, name.to_string()),
            object as Arc<dyn Any + Send + Sync>,
        );
        Ok(())
    }

    /// Wire every door this table holds to `recorder`, then refuse any that is
    /// still undeclared.
    ///
    /// The enforcement half of the record/replay seam, and the reason
    /// [`machine::realize`](mod@crate::machine::realize) does this after
    /// constructing a board rather than before: by then every device has opened
    /// the host objects it wants, so the doors among them can be *wired* rather
    /// than merely counted.
    ///
    /// Three things happen to each open object, in this order:
    ///
    /// * a [rendezvous](HostKind::rendezvous) or a [pulled](HostKind::pulled)
    ///   kind is skipped — a PCI fabric is not an input, and demanding a
    ///   channel for one is what kept this mechanism switched off;
    /// * a [door](HostKind::door) the recorder already knows is left alone, so
    ///   a caller that registered its own sink keeps it;
    /// * a door the recorder does not know is registered with the sink its own
    ///   module supplied — or, if that module supplied none, the seal fails
    ///   naming it.
    ///
    /// Afterwards the table is closed: opening a door the recorder still does
    /// not know is [`Error::Config`], so a caller that seals an **empty** table
    /// and then builds gets the strict form — every input must have been
    /// declared up front, and the board refuses to build otherwise. Both flows
    /// are used; `tests/record_replay.rs` has each.
    ///
    /// The recorder is *not* sealed here. Its channel list closes at the first
    /// [`Recorder::deliver`](crate::core::record::Recorder::deliver), which is
    /// the moment [`Recorder::register`]'s own reason bites — a channel added
    /// after the machine has run would silently have missed everything before
    /// it — and it leaves room for a frontend that can only be built once the
    /// machine it draws exists.
    ///
    /// [`Recorder::register`]: crate::core::record::Recorder::register
    ///
    /// # Errors
    ///
    /// [`Error::Config`] naming the first door already open that the recorder
    /// does not know and cannot be taught.
    pub fn seal(&self, recorder: Arc<Recorder>) -> Result<()> {
        // Cloned out before anything else is locked: `Recorder::register` takes
        // its own lock, and two leaf locks may not be nested.
        let open: Vec<((HostKind, String), Arc<dyn Any + Send + Sync>)> = self
            .entries
            .lock()
            .iter()
            .map(|(key, object)| (key.clone(), Arc::clone(object)))
            .collect();
        for ((kind, name), object) in &open {
            if !kind.is_door() {
                continue;
            }
            let channel = Channel::new(*kind, name);
            if recorder.knows(&channel) {
                continue;
            }
            let Some(make) = kind.sink_factory() else {
                return Err(unrecorded(*kind, name));
            };
            let Some(sink) = make(object) else {
                return Err(mistyped(*kind, name));
            };
            recorder.register(channel, sink)?;
        }
        *self.policy.lock() = InputPolicy::Sealed(recorder);
        Ok(())
    }

    /// Drop the seal, whatever it was.
    ///
    /// For a host that has finished a recording and wants the table back.
    pub fn unseal(&self) {
        *self.policy.lock() = InputPolicy::Open;
    }

    /// Whether this table is sealed onto a recorder.
    #[must_use]
    pub fn is_sealed(&self) -> bool {
        matches!(&*self.policy.lock(), InputPolicy::Sealed(_))
    }

    /// The policy check, run before any entry is touched so the two leaf locks
    /// are never held at once.
    fn check_policy(&self, kind: HostKind, name: &str) -> Result<()> {
        // A rendezvous is not an input. Checking one is what made sealing any
        // board with a PCI or USB bus impossible, and it is answered before the
        // policy lock is even taken.
        if !kind.is_door() {
            return Ok(());
        }
        // Cloned out rather than held: `Recorder::knows` takes its own lock, and
        // a leaf lock may not be held while another is acquired.
        let recorder = match &*self.policy.lock() {
            InputPolicy::Open => return Ok(()),
            InputPolicy::Sealed(recorder) => Arc::clone(recorder),
        };
        if recorder.knows(&Channel::new(kind, name)) {
            return Ok(());
        }
        Err(unrecorded(kind, name))
    }

    /// Forget `name` under `kind`, reporting whether there was one.
    ///
    /// Anything still holding the `Arc` keeps working; this only drops the
    /// table's own reference, so a later [`open`](HostObjects::open) of the same
    /// name makes a fresh object.
    pub fn close(&self, kind: HostKind, name: &str) -> bool {
        self.entries
            .lock()
            .remove(&(kind, name.to_string()))
            .is_some()
    }

    /// Every open name under `kind`, in name order.
    #[must_use]
    pub fn names(&self, kind: HostKind) -> Vec<String> {
        self.entries
            .lock()
            .keys()
            .filter(|(k, _)| *k == kind)
            .map(|(_, name)| name.clone())
            .collect()
    }

    /// How many objects are open, across every kind.
    #[must_use]
    pub fn len(&self) -> usize {
        self.entries.lock().len()
    }

    /// Whether nothing has been opened.
    #[must_use]
    pub fn is_empty(&self) -> bool {
        self.entries.lock().is_empty()
    }
}

/// The diagnostic for a host object that would bypass the record/replay seam.
///
/// Named rather than inlined because it is the message a device author will
/// read when their new device fails to realize under a recording, and it has to
/// tell them what to do rather than merely that something is wrong.
fn unrecorded(kind: HostKind, name: &str) -> Error {
    Error::Config {
        at: format!("{kind}:{name}"),
        message: String::from(
            "this host object would carry non-deterministic input into a machine whose \
             recorder has no channel for it, so the run could not be replayed \
             (CLAUDE.md, determinism). Ship a `channel()` and a `sink()` beside this \
             kind's `open()` and declare it with `HostKind::door`, or register the \
             channel with the recorder before building the machine. If nothing \
             non-deterministic crosses here, the kind is a `HostKind::rendezvous`",
        ),
    }
}

/// The diagnostic for a door whose own sink factory did not recognise the
/// object filed under it.
///
/// Two modules claiming one kind name, seen from the seal rather than from
/// [`HostObjects::open`]: the object is of the other module's type, so the
/// factory this kind carries cannot make a sink for it.
fn mistyped(kind: HostKind, name: &str) -> Error {
    Error::Config {
        at: format!("{kind}:{name}"),
        message: String::from(
            "the module that declared this host-object kind does not recognise the object \
             open under this name, so it cannot say where a recorded payload goes: two \
             modules have claimed one kind name",
        ),
    }
}

/// Recover the concrete type, or say which name is claimed twice.
fn downcast<T: Any + Send + Sync>(
    kind: HostKind,
    name: &str,
    any: Arc<dyn Any + Send + Sync>,
) -> Result<Arc<T>> {
    any.downcast::<T>().map_err(|_| Error::Config {
        at: format!("{kind}:{name}"),
        message: String::from("a host object of another type is already open under this name"),
    })
}

// ---------------------------------------------------------------------------
// capture
// ---------------------------------------------------------------------------

/// What a build's constructors handed the host, oldest first.
///
/// The other half of the same problem. `machine::build` returns
/// `Arc<dyn Device>` and there is no route from that to a concrete type —
/// `Device` keeps `Any` out of its supertrait chain deliberately — so a host
/// that needs an `Arc<NesPpu>` takes it at the only moment the concrete type
/// exists: construction. A capture table is what the intercepting constructor
/// pushes into, and it lives in [`HostObjects`] like every other build-scoped
/// host object, so two builds capture into two tables.
///
/// A `Vec` rather than one slot because a machine with two of something — a PC
/// with an MDA *and* a CGA — is not this type's business to refuse.
pub struct Captured<T> {
    /// [`LockRank::LEAF`]: pushed to from a constructor, read by the host, and
    /// nothing is locked while it is held.
    seen: Mutex<Vec<Arc<T>>>,
}

impl<T> Default for Captured<T> {
    fn default() -> Captured<T> {
        Captured::new()
    }
}

impl<T> fmt::Debug for Captured<T> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self.seen.try_lock() {
            Some(seen) => f
                .debug_struct("Captured")
                .field("len", &seen.len())
                .finish(),
            None => f
                .debug_struct("Captured")
                .field("len", &"<in use>")
                .finish(),
        }
    }
}

impl<T> Captured<T> {
    /// Nothing captured yet.
    #[must_use]
    pub fn new() -> Captured<T> {
        Captured {
            seen: Mutex::with_rank(LockRank::LEAF, Vec::new()),
        }
    }

    /// Record one, from the constructor that built it.
    pub fn push(&self, object: &Arc<T>) {
        self.seen.lock().push(Arc::clone(object));
    }

    /// The most recently constructed one, leaving the table as it is.
    #[must_use]
    pub fn last(&self) -> Option<Arc<T>> {
        self.seen.lock().last().map(Arc::clone)
    }

    /// The most recently constructed one, forgetting every earlier one.
    ///
    /// `None` when this build constructed none — a machine with no picture,
    /// which a host must be able to render nothing for.
    #[must_use]
    pub fn take(&self) -> Option<Arc<T>> {
        let mut seen = self.seen.lock();
        let last = seen.pop();
        seen.clear();
        last
    }

    /// Every one that was constructed, oldest first.
    #[must_use]
    pub fn all(&self) -> Vec<Arc<T>> {
        self.seen.lock().clone()
    }

    /// How many were constructed.
    #[must_use]
    pub fn len(&self) -> usize {
        self.seen.lock().len()
    }

    /// Whether none was.
    #[must_use]
    pub fn is_empty(&self) -> bool {
        self.seen.lock().is_empty()
    }

    /// Forget every handle.
    pub fn clear(&self) {
        self.seen.lock().clear();
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const A: HostKind = HostKind::new("test.a");
    const B: HostKind = HostKind::new("test.b");

    #[derive(Debug, PartialEq, Eq)]
    struct Thing(u32);

    #[derive(Debug)]
    struct Other;

    #[test]
    fn one_name_is_one_object_and_two_tables_share_nothing() {
        let left = HostObjects::new();
        let right = HostObjects::new();

        let a = left.open(A, "console", || Thing(1)).unwrap();
        let b = left.open(A, "console", || Thing(2)).unwrap();
        assert!(Arc::ptr_eq(&a, &b), "the same name is the same object");
        assert_eq!(*a, Thing(1), "and the second value was discarded");

        // The whole reason this type exists.
        let elsewhere = right.open(A, "console", || Thing(3)).unwrap();
        assert!(!Arc::ptr_eq(&a, &elsewhere));
        assert_eq!(*elsewhere, Thing(3));
    }

    #[test]
    fn kinds_do_not_collide_and_names_are_ordered() {
        let hosts = HostObjects::new();
        let a = hosts.open(A, "x", || Thing(1)).unwrap();
        let b = hosts.open(B, "x", || Thing(2)).unwrap();
        assert!(
            !Arc::ptr_eq(&a, &b),
            "one name under two kinds is two things"
        );

        hosts.open(A, "zulu", || Thing(3)).unwrap();
        hosts.open(A, "alpha", || Thing(4)).unwrap();
        assert_eq!(hosts.names(A), ["alpha", "x", "zulu"]);
        assert_eq!(hosts.names(B), ["x"]);
        assert_eq!(hosts.len(), 4);
        assert!(!hosts.is_empty());
    }

    #[test]
    fn a_type_collision_is_reported_rather_than_papered_over() {
        let hosts = HostObjects::new();
        hosts.open(A, "x", || Thing(1)).unwrap();
        let e = hosts.open(A, "x", || Other).unwrap_err().to_string();
        assert!(e.contains("another type"), "{e}");
    }

    #[test]
    fn closing_a_name_leaves_the_arc_alone() {
        let hosts = HostObjects::new();
        let held = hosts.open(A, "x", || Thing(7)).unwrap();
        assert!(hosts.close(A, "x"));
        assert!(!hosts.close(A, "x"));
        assert!(hosts.get::<Thing>(A, "x").unwrap().is_none());
        assert_eq!(*held, Thing(7), "the holder is unaffected");
        // And a re-open is a fresh object rather than the old one.
        let again = hosts.open(A, "x", || Thing(8)).unwrap();
        assert!(!Arc::ptr_eq(&held, &again));
    }

    #[test]
    fn a_host_may_supply_the_object_itself() {
        let hosts = HostObjects::new();
        let mine = Arc::new(Thing(42));
        hosts.insert(A, "x", Arc::clone(&mine));
        let theirs = hosts.open(A, "x", || Thing(0)).unwrap();
        assert!(Arc::ptr_eq(&mine, &theirs));
    }

    #[test]
    fn a_capture_table_remembers_in_order_and_takes_the_last() {
        let seen: Captured<Thing> = Captured::new();
        assert!(seen.is_empty());
        assert!(seen.take().is_none());

        let first = Arc::new(Thing(1));
        let second = Arc::new(Thing(2));
        seen.push(&first);
        seen.push(&second);
        assert_eq!(seen.len(), 2);
        assert_eq!(seen.all().len(), 2);
        assert_eq!(*seen.last().unwrap(), Thing(2));

        let took = seen.take().unwrap();
        assert!(Arc::ptr_eq(&took, &second));
        assert!(seen.is_empty(), "and the earlier one is forgotten");
    }
}
