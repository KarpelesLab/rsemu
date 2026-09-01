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
//! A host object is the *only* door from the host into a machine — and that
//! includes the [`Captured`] table, which is how a host reaches a concrete
//! device to press its buttons directly. So checking every object against a
//! recorder's registered channels is checking every non-deterministic input,
//! once, in one place, rather than trusting each device to declare its own.
//! [`HostObjects::seal`] is that check and
//! [`core::record`](crate::core::record) argues why it belongs here.
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
use crate::core::record::{Channel, Recorder};
use crate::core::sync::{LockRank, Mutex};

/// What sort of host object a name refers to.
///
/// An extensible enumeration in the `pktkit` style (`CLAUDE.md`) rather than a
/// Rust `enum`, because the kinds are declared by the feature-gated modules
/// that own them: [`chardev::ports`](crate::host::chardev::ports) declares
/// `chardev`, the NES input port declares `pad`, and a build with neither has
/// no opinion about either.
///
/// The kind also fixes the *type* stored under it. Two modules must not claim
/// one kind name; [`HostObjects::open`] reports the collision rather than
/// handing back a pad where a character port was wanted.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
#[repr(transparent)]
pub struct HostKind(pub &'static str);

impl HostKind {
    /// The kind a build's [`Captured`] tables are filed under, keyed by the
    /// name of the class whose constructor was intercepted.
    ///
    /// One well-known kind rather than one per host module: the class name
    /// already separates a captured PPU from a captured VDP, and a shared kind
    /// means a host can ask what a build captured without knowing which module
    /// installed the interception.
    pub const CAPTURE: HostKind = HostKind("capture");

    /// A kind from its name.
    #[must_use]
    pub const fn new(name: &'static str) -> HostKind {
        HostKind(name)
    }

    /// The name, for a diagnostic.
    #[must_use]
    pub const fn as_str(self) -> &'static str {
        self.0
    }
}

impl fmt::Display for HostKind {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(self.0)
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

    /// Refuse any host object the recorder has not registered as a channel.
    ///
    /// The enforcement half of the record/replay seam. Call it *before* the
    /// machine is built: every device opens its host objects from `new(props)`,
    /// so a board with an unrecorded input fails to realize rather than
    /// producing a recording that is quietly missing a stream.
    ///
    /// The recorder is sealed too, so the two lists cannot drift: after this,
    /// neither a new channel nor a new host object can appear.
    ///
    /// Sealing an already-populated table is allowed and checks what is already
    /// there, so a caller that builds first and seals second still gets the
    /// diagnostic — one round late, but before the first round runs.
    ///
    /// # Errors
    ///
    /// [`Error::Config`] naming the first host object already open that the
    /// recorder does not know about.
    pub fn seal(&self, recorder: Arc<Recorder>) -> Result<()> {
        let open: Vec<(HostKind, String)> = self.entries.lock().keys().cloned().collect();
        for (kind, name) in &open {
            if !recorder.knows(&Channel::new(*kind, name)) {
                return Err(unrecorded(*kind, name));
            }
        }
        recorder.seal();
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
             (CLAUDE.md, determinism). Register the channel with the recorder before \
             building the machine, or do not seal the host-object table",
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
