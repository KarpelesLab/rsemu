//! Realize: turn a resolved description into a running [`Machine`]
//! (`ROADMAP.md` §5's last stage, over §4's core).
//!
//! ```text
//! lex → parse → resolve → validate → realize → run
//!                                    ^^^^^^^
//! ```
//!
//! # What realize does, in order
//!
//! The order is not arbitrary — each step needs the one before it:
//!
//! 1. **Address spaces** (§4.1), from the `space` statements. Empty so far.
//! 2. **The oscillator forest** (§4.2): every `osc`, then every object's
//!    domain. A domain may hang off another object's domain (`cpu / 2`), so
//!    domains are added in dependency order and a cycle is an error naming the
//!    objects in it.
//! 3. **Construct** every device through the registry. Two-phase construction
//!    (§4.4): this validates properties and allocates, and does nothing
//!    observable.
//! 4. **Realize** every device, in declaration order, draining its deferred
//!    actions after each. A failure here unrealizes what was already realized
//!    and returns — a half-wired machine never escapes this function.
//! 5. **Map** every `map` statement into its space, through
//!    [`AddressSpace::topology`](crate::core::space::AddressSpace::topology).
//!    Nothing about this step is final any more: a space stays retopologisable
//!    after realize, which is what hot-plug and a BAR move need.
//! 6. **Bind**: hand each device its clock domain and its address space.
//! 7. **Wire** (§4.3): build one [`Wire`] per net, connect sinks weakly, hand
//!    each driving pin its [`WireSource`].
//! 8. **Reset** cold, then **sweep**: walk wire sources in topological order
//!    and announce their levels, or the machine comes up with an inverter
//!    idling low. The order is
//!    [`validate::realize_order`](crate::machine::validate::realize_order)'s —
//!    the same computation that rejects combinational wire cycles, so the
//!    check and the order can never disagree.
//!
//! # The `Instance` seam, and why it exists
//!
//! `core::device::Device` describes a device's *lifecycle* — realize, reset,
//! save, load — but nothing about its *connections*: there is no way to ask a
//! `dyn Device` for the region a `map` statement names, for the sink a `wire`
//! statement drives, or to hand it the source it drives. §4.4 says
//! `RealizeCtx` is where that happens, but `RealizeCtx` carries only a path, a
//! requester id and a deferred queue, and there is no route from `dyn Device`
//! to any other trait object without `Any` in the supertrait chain.
//!
//! So the connection half lives here, as [`Instance`], and a class opts in by
//! registering a constructor in [`Bindings`] beside its registry entry. The
//! registry stays the class table of record — a class with a binding but no
//! registry entry is rejected — so `rsemu devices` and the validator keep
//! seeing one list. When `core::device` grows either an accessor for a second
//! trait object or a `RealizeCtx` with space, wire and clock handles,
//! [`Bindings`] collapses into the registry and nothing else here changes.
//!
//! # Seams this layer ran into
//!
//! Recorded here because they are load-bearing and invisible from inside any
//! one module:
//!
//! * **`AddressSpace` topology used to take `&mut self`,** which meant a space
//!   handed out as an `Arc` (§4.4's `Initiator` needs one) could never be
//!   retopologised again. It now lives behind a `core::sync::RwLock` at
//!   `LockRank::TOPOLOGY` and `AddressSpace::topology()` hands out the write
//!   guard, so mapping before or after realize is the same operation and
//!   hot-plug is expressible. The order below is unchanged only because a
//!   machine still wants its memory map complete before the first access.
//! * **Catch-up reaches a device, not the scheduler.** Sync-on-access (§4.2)
//!   has to fire from `MemOps::read`, which has `&self` and runs with the bus
//!   lock held. `core::sched` answers that with
//!   [`LazyHandle`](crate::core::sched::LazyHandle): a shared handle to one
//!   lazily-advanced device that catches it up without taking any
//!   scheduler-ranked lock — `LockRank::SCHED` is *above* `LockRank::BUS`, so
//!   an access that reached back for one would invert the ladder. A class says
//!   so with [`Device::is_lazy`], and the realizer registers it on its clock
//!   domain and hands the handle straight back to the device through
//!   [`Device::attach_lazy`] — the device syncs at the top of its own read and
//!   write paths, because it is the only thing that knows which of its
//!   registers are sampled and it may publish several windows into one space.
//! * **A deferred action carries no context.** `Deferred`'s payload is
//!   `FnOnce() + Send` with no arguments, so an action can only touch what it
//!   captured — and it cannot capture the machine, which owns the queue. It
//!   works for a device acting on its own state; a deferred remap needs the
//!   drain to pass something in.
//! * **Realize-time errors lose the caret.** [`Resolved`] carries a span on
//!   every node, but realize returns [`Error`], not a `Diagnostic`, so the
//!   `file:line:col` §5 promises "always" stops at validate. The spans are
//!   there; a `realize` that returned `Diagnostic` would keep them.

use alloc::boxed::Box;
use alloc::collections::BTreeMap;
use alloc::format;
use alloc::string::{String, ToString};
use alloc::sync::{Arc, Weak};
use alloc::vec::Vec;

use crate::core::clock::{ClockForest, DomainId, Rational as ClockRational};
pub use crate::core::device::SinkPin;
use crate::core::device::{Deferred, Device, DeviceClass, RealizeCtx, ResetKind};
use crate::core::error::{Error, Result};
use crate::core::props::{Media, Props, Value, ValueKind};
use crate::core::registry::Registry;
use crate::core::sched::{Scheduler, SchedulerConfig};
use crate::core::space::{
    AddressSpace, Mapping as SpaceMapping, Region, RegionRef, RequesterId, UnassignedPolicy,
};
use crate::core::state::MachineShape;
use crate::core::value::Endian;
use crate::core::wire::{Wire, WireId, WireIdAllocator, WireSource};
use crate::machine::machine::{
    DeviceEntry, LazyAdapter, Machine, MachineParts, Net, PinRef, RunAdapter,
};
use crate::machine::resolver::{
    Clock, ClockParent, MapTarget, Mapping, ObjectId, Resolved, SpaceId,
};
use crate::machine::validate::{ClassSchema, ClassTable, realize_order};

// ---------------------------------------------------------------------------
// the Instance seam
// ---------------------------------------------------------------------------

/// What a device is told when the machine binds it to the rest of the machine.
#[derive(Debug)]
pub struct BindCtx<'a> {
    path: &'a str,
    requester: RequesterId,
    domain: Option<DomainId>,
    space: Option<&'a Arc<AddressSpace>>,
    spaces: &'a [(String, Arc<AddressSpace>)],
}

impl<'a> BindCtx<'a> {
    /// This instance's path — its snapshot chunk key (§4.5).
    pub fn path(&self) -> &'a str {
        self.path
    }

    /// The requester id accesses this device initiates should carry.
    pub fn requester(&self) -> RequesterId {
        self.requester
    }

    /// The clock domain the machine file gave it, if any.
    pub fn domain(&self) -> Option<DomainId> {
        self.domain
    }

    /// The address space it declared with `space = …` — *its* view, which is
    /// not necessarily the CPU's (§4.1).
    pub fn space(&self) -> Option<&'a Arc<AddressSpace>> {
        self.space
    }

    /// Any address space by name, for a device that reaches more than one.
    pub fn space_named(&self, name: &str) -> Option<&'a Arc<AddressSpace>> {
        self.spaces.iter().find(|(n, _)| n == name).map(|(_, s)| s)
    }
}

/// The machine-layer half of a device: how it connects to everything else.
///
/// Implemented beside [`Device`], never instead of it. Every method has a
/// default that answers "I have none of those", so a device that only responds
/// to MMIO implements [`Device::region`] and nothing more.
///
/// See the module docs for why this is not on `Device` itself.
pub trait Instance: Device {
    /// Told which clock domain and address space it belongs to.
    ///
    /// Runs after every region is mapped, so a device may read through its
    /// space from here.
    ///
    /// This is the one connection method that is *not* on
    /// [`Device`]: the rest were folded there so a
    /// class registers once instead of twice. `bind` stays because
    /// [`BindCtx`] is a machine-layer type — it disappears when `RealizeCtx`
    /// grows the space and clock handles `ROADMAP.md` §4.4 implies, and then
    /// `Instance` disappears with it.
    ///
    /// # Errors
    ///
    /// If the device needs something the machine file did not give it — a CPU
    /// with no address space, say.
    fn bind(&self, ctx: &BindCtx<'_>) -> Result<()> {
        let _ = ctx;
        Ok(())
    }
}

// ---------------------------------------------------------------------------
// bindings
// ---------------------------------------------------------------------------

/// Constructs a device and its machine-layer view together.
///
/// The same shape as `DeviceClass::construct`, returning an `Arc<dyn Instance>`
/// instead of a `Box<dyn Device>`. A class should implement one function and
/// have both entry points call it, so the two can never disagree.
pub type InstanceCtor = fn(&Props) -> Result<Arc<dyn Instance>>;

/// Class name → [`InstanceCtor`], for the classes that participate in the
/// machine graph.
///
/// Ordered by class name: iteration reaches error messages, and hash order is
/// forbidden anywhere it can be observed.
#[derive(Debug, Default)]
pub struct Bindings {
    entries: BTreeMap<&'static str, InstanceCtor>,
}

impl Bindings {
    /// No bindings: every device is constructed through the registry alone and
    /// participates in reset and snapshots but not in the memory map or the
    /// wire graph.
    pub fn new() -> Bindings {
        Bindings {
            entries: BTreeMap::new(),
        }
    }

    /// Bind `class`.
    ///
    /// # Errors
    ///
    /// If the class is bound twice — two features claiming one name would make
    /// the machine depend on registration order, exactly as it would in the
    /// registry.
    pub fn bind(&mut self, class: &'static str, ctor: InstanceCtor) -> Result<()> {
        if self.entries.contains_key(class) {
            return Err(Error::Config {
                at: class.to_string(),
                message: "device class bound twice".to_string(),
            });
        }
        self.entries.insert(class, ctor);
        Ok(())
    }

    /// Builder form of [`Bindings::bind`], panicking on a duplicate.
    ///
    /// # Panics
    ///
    /// If `class` is already bound.
    #[must_use]
    pub fn with(mut self, class: &'static str, ctor: InstanceCtor) -> Bindings {
        self.bind(class, ctor).expect("class bound twice");
        self
    }

    /// The constructor for `class`, if it has one.
    pub fn get(&self, class: &str) -> Option<InstanceCtor> {
        self.entries.get(class).copied()
    }

    /// Every bound class, in name order.
    pub fn classes(&self) -> impl Iterator<Item = &'static str> + '_ {
        self.entries.keys().copied()
    }

    /// How many classes are bound.
    pub fn len(&self) -> usize {
        self.entries.len()
    }

    /// Whether nothing is bound.
    pub fn is_empty(&self) -> bool {
        self.entries.is_empty()
    }
}

// ---------------------------------------------------------------------------
// media
// ---------------------------------------------------------------------------

/// The blobs bound to a machine's media slots: ROM images, disk images,
/// firmware.
///
/// # How media reaches a device
///
/// A device needs bytes, and nothing below `host/` may open a file — the core
/// is `no_std` and `CLAUDE.md` gives file access to the caller. So the loop is
/// closed by *name*:
///
/// ```text
/// machine file:  object cart "nes.nrom" { rom = "cart" }   # names a slot
/// caller:        rsemu run nes.machine --cart smb.nes      # binds the slot
/// realize:       rom = "cart"  ──►  Value::Media(40976 bytes)
/// device:        r.require_media("rom")?                   # sees only bytes
/// ```
///
/// The substitution happens before `construct`, so two-phase construction is
/// untouched: `new(props)` still validates and allocates everything, including
/// parsing the image, and nothing observable happens until realize. A class
/// opts in by declaring a [`PropertySpec`](crate::core::PropertySpec) of kind
/// [`ValueKind::Media`]; the realizer looks at nothing else.
///
/// A slot nothing is bound to is an error naming the slot, not a device that
/// quietly comes up empty.
#[derive(Debug, Clone, Default)]
pub struct MediaTable {
    entries: BTreeMap<String, Media>,
}

impl MediaTable {
    /// No media bound.
    pub fn new() -> MediaTable {
        MediaTable::default()
    }

    /// Bind `slot` to `bytes`, replacing anything already there.
    ///
    /// Replacing rather than refusing: a caller that passes `--cart` twice
    /// means the second one, the way every other command-line option works.
    pub fn insert(&mut self, slot: impl Into<String>, bytes: impl Into<Arc<[u8]>>) {
        let slot = slot.into();
        let media = Media::new(slot.clone(), bytes);
        self.entries.insert(slot, media);
    }

    /// Builder form of [`MediaTable::insert`].
    #[must_use]
    pub fn with(mut self, slot: impl Into<String>, bytes: impl Into<Arc<[u8]>>) -> MediaTable {
        self.insert(slot, bytes);
        self
    }

    /// What is bound to `slot`.
    pub fn get(&self, slot: &str) -> Option<&Media> {
        self.entries.get(slot)
    }

    /// Every bound slot, in name order.
    pub fn slots(&self) -> impl Iterator<Item = &str> + '_ {
        self.entries.keys().map(String::as_str)
    }

    /// How many slots are bound.
    pub fn len(&self) -> usize {
        self.entries.len()
    }

    /// Whether nothing is bound.
    pub fn is_empty(&self) -> bool {
        self.entries.is_empty()
    }
}

/// What realize needs beyond the description itself.
#[derive(Debug, Clone, Default)]
pub struct RealizeOptions {
    /// How the scheduler is configured: threading mode, quantum, rate control.
    ///
    /// Not in the `.machine` grammar yet — §4.2 says the threading mode is a
    /// machine property, and when the DSL grows a statement for it this is
    /// where it lands.
    pub scheduler: SchedulerConfig,
    /// The bytes bound to the machine's media slots. See [`MediaTable`].
    pub media: MediaTable,
}

impl RealizeOptions {
    /// Defaults: deterministic threading, unbounded rate, a 1 ms quantum, and
    /// no media bound.
    pub fn new() -> RealizeOptions {
        RealizeOptions::default()
    }

    /// Bind a media slot, as `rsemu run … --cart smb.nes` does.
    #[must_use]
    pub fn with_media(mut self, slot: impl Into<String>, bytes: impl Into<Arc<[u8]>>) -> Self {
        self.media.insert(slot, bytes);
        self
    }
}

// ---------------------------------------------------------------------------
// entry points
// ---------------------------------------------------------------------------

/// Realize a validated machine description.
///
/// Devices are constructed through `registry`; none of them takes part in the
/// memory map or the wire graph, because that needs [`Bindings`]. A description
/// with a `map` or `wire` statement therefore fails here with a message saying
/// which class is unbound — which is the honest answer, not a silent no-op.
///
/// # Errors
///
/// Anything wrong that only building the machine can find: an unknown class, a
/// region that does not fit, a clock that cannot be rated, a pin no device has.
pub fn realize(machine: &Resolved, registry: &Registry) -> Result<Machine> {
    realize_with(machine, registry, &Bindings::new(), &RealizeOptions::new())
}

/// Realize with class bindings and explicit options.
///
/// # Errors
///
/// As [`realize`].
pub fn realize_with(
    machine: &Resolved,
    registry: &Registry,
    bindings: &Bindings,
    options: &RealizeOptions,
) -> Result<Machine> {
    Realizer::new(machine, registry, bindings, options).run()
}

// ---------------------------------------------------------------------------
// the realizer
// ---------------------------------------------------------------------------

/// One device under construction.
struct Built {
    path: String,
    class: &'static DeviceClass,
    device: Arc<dyn Device>,
    instance: Option<Arc<dyn Instance>>,
    domain: Option<DomainId>,
    space: Option<usize>,
    requester: RequesterId,
}

/// The realizer's working state.
struct Realizer<'a> {
    machine: &'a Resolved,
    registry: &'a Registry,
    bindings: &'a Bindings,
    options: &'a RealizeOptions,
    spaces: Vec<(String, AddressSpace)>,
    forest: ClockForest,
    /// The domain each object was given, indexed like `Resolved::objects`.
    assigned: Vec<Option<DomainId>>,
    built: Vec<Built>,
    /// How many devices have been realized, so a failure unrealizes exactly
    /// those and not the ones that never acted.
    realized: usize,
    shape: MachineShape,
    deferred: Deferred,
}

impl<'a> Realizer<'a> {
    fn new(
        machine: &'a Resolved,
        registry: &'a Registry,
        bindings: &'a Bindings,
        options: &'a RealizeOptions,
    ) -> Realizer<'a> {
        Realizer {
            machine,
            registry,
            bindings,
            options,
            spaces: Vec::new(),
            forest: ClockForest::new(),
            assigned: Vec::new(),
            built: Vec::new(),
            realized: 0,
            shape: MachineShape::new(),
            deferred: Deferred::new(),
        }
    }

    fn run(mut self) -> Result<Machine> {
        self.build_spaces()?;
        self.build_clocks()?;
        self.construct()?;
        // From here on a failure has to undo what realize already did, so the
        // rest is wrapped: nothing observable may survive a failed realize.
        match self.assemble() {
            Ok(machine) => Ok(machine),
            Err(e) => {
                self.unrealize_all();
                Err(e)
            }
        }
    }

    /// Everything after the point where devices start acting outward.
    fn assemble(&mut self) -> Result<Machine> {
        self.realize_devices()?;
        let mut spaces = self.map_regions()?;
        let shared: Vec<(String, Arc<AddressSpace>)> = core::mem::take(&mut spaces)
            .into_iter()
            .map(|(name, space)| (name, Arc::new(space)))
            .collect();
        self.bind_devices(&shared)?;
        let (nets, sweep) = self.build_wires()?;

        let mut sched = Scheduler::new(
            core::mem::take(&mut self.forest),
            self.options.scheduler.clone(),
        );
        let devices = self.register_with_scheduler(&mut sched)?;

        let mut machine = Machine::assemble(MachineParts {
            name: self.machine.name.clone(),
            spaces: shared,
            sched,
            devices,
            nets,
            sweep,
            shape: core::mem::take(&mut self.shape),
            deferred: core::mem::take(&mut self.deferred),
        });
        // A machine is born cold, and the sweep that follows a reset is what
        // makes its wire graph consistent (§4.3).
        machine.reset(ResetKind::Cold);
        Ok(machine)
    }

    // -- spaces ------------------------------------------------------------

    fn build_spaces(&mut self) -> Result<()> {
        for space in &self.machine.spaces {
            let at = || format!("space `{}`", space.name);
            let mut r = space.props.reader();
            let bits: u32 = r
                .require_range("width", 1..=64)
                .map_err(|e| config(at(), e.to_string()))?;
            let unassigned = r
                .or_str("unassigned", "fault")
                .map_err(|e| config(at(), e.to_string()))?;
            let endian = r
                .or_str("endian", "little")
                .map_err(|e| config(at(), e.to_string()))?;
            let log = r
                .or("log-unassigned", false)
                .map_err(|e| config(at(), e.to_string()))?;
            // Realize does not assume it was validated: a machine built
            // straight from a `Resolved` still gets told about `unassigend`.
            r.finish().map_err(|e| config(at(), e.to_string()))?;

            let policy = match unassigned {
                "fault" => UnassignedPolicy::FAULT,
                // `open-bus` is §5's spelling. Until `core::space` grows a
                // last-value-on-the-bus policy it is reads-as-ones, which is
                // what the validator already documents.
                "open-bus" | "read-as-ones" => UnassignedPolicy::ONES,
                "read-as-zeros" => UnassignedPolicy::ZEROS,
                other => {
                    return Err(config(at(), format!("unknown unassigned policy `{other}`")));
                }
            };
            let policy = if log { policy.logged() } else { policy };
            let endian = match endian {
                "little" => Endian::Little,
                "big" => Endian::Big,
                other => return Err(config(at(), format!("unknown byte order `{other}`"))),
            };
            self.spaces.push((
                space.name.clone(),
                AddressSpace::new(space.name.clone(), bits)
                    .with_unassigned(policy)
                    .with_endian(endian),
            ));
        }
        Ok(())
    }

    // -- clocks ------------------------------------------------------------

    fn build_clocks(&mut self) -> Result<()> {
        let mut osc_roots = Vec::with_capacity(self.machine.oscillators.len());
        for osc in &self.machine.oscillators {
            let hz = to_clock_rational(&osc.hz).ok_or_else(|| {
                config(
                    format!("osc `{}`", osc.name),
                    "frequency is not a positive rational that fits in 64 bits",
                )
            })?;
            let root = self.forest.add_oscillator(&osc.name, hz)?;
            osc_roots.push(root);
        }

        // An object's domain may hang off another object's domain, so the
        // order is a dependency order rather than declaration order. The graph
        // is tiny; repeated passes are simpler than a topological sort and give
        // a better error, because whatever is left when progress stops is
        // exactly the cycle.
        let mut assigned: Vec<Option<DomainId>> = alloc::vec![None; self.machine.objects.len()];
        let mut pending: Vec<usize> = self
            .machine
            .objects
            .iter()
            .enumerate()
            .filter(|(_, o)| o.clock.is_some())
            .map(|(i, _)| i)
            .collect();

        while !pending.is_empty() {
            let mut progress = false;
            let mut next = Vec::new();
            for i in pending {
                let object = &self.machine.objects[i];
                let clock = object.clock.expect("filtered to objects with a clock");
                let parent = match parent_of(&clock, &osc_roots, &assigned) {
                    ParentLookup::Ready(id) => id,
                    ParentLookup::Waiting => {
                        next.push(i);
                        continue;
                    }
                    ParentLookup::Missing(name) => {
                        return Err(config(
                            object.name.clone(),
                            format!("clock parent `{name}` has no clock domain of its own"),
                        ));
                    }
                };
                let id = self
                    .forest
                    .add_domain(&object.name, parent, clock.mul, clock.div)?;
                assigned[i] = Some(id);
                progress = true;
            }
            if !progress {
                let names: Vec<&str> = next
                    .iter()
                    .map(|i| self.machine.objects[*i].name.as_str())
                    .collect();
                return Err(config(
                    self.machine.name.clone(),
                    format!(
                        "clock domains form a cycle: {}; one of them must divide an oscillator",
                        list(&names)
                    ),
                ));
            }
            pending = next;
        }
        self.assigned = assigned;
        Ok(())
    }

    // -- devices -----------------------------------------------------------

    fn construct(&mut self) -> Result<()> {
        for (i, object) in self.machine.objects.iter().enumerate() {
            // The registry is the class table of record even for a bound
            // class: `rsemu devices` and the validator read it, and a class
            // that exists only in a binding would be invisible to both.
            let Some(class) = self.registry.get(&object.class) else {
                // `create` composes the "is its feature enabled?" message and
                // the near-miss suggestion; `get` cannot, and reproducing them
                // here would be a second copy to keep in step.
                return Err(self
                    .registry
                    .create(&object.class, &object.props)
                    .err()
                    .unwrap_or_else(|| Error::UnknownClass(object.class.clone())));
            };
            // Requester ids are 1-based: `RequesterId::ANONYMOUS` is 0 and
            // means "nobody in particular", which no device is.
            let requester = RequesterId(u32::try_from(i + 1).unwrap_or(u32::MAX));
            // Media slots become bytes here, before construction, so a device
            // is fully built by `new(props)` (§4.4) and never has to reach out
            // for its own image.
            let bound = self.bind_media(object, class)?;
            let props = bound.as_ref().unwrap_or(&object.props);
            let (device, instance) = match self.bindings.get(&object.class) {
                Some(ctor) => {
                    let instance = ctor(props)?;
                    check_class(&object.name, class.name, instance.class().name)?;
                    let device: Arc<dyn Device> = instance.clone();
                    (device, Some(instance))
                }
                None => {
                    let device: Arc<dyn Device> =
                        Arc::from(self.registry.create(&object.class, props)?);
                    check_class(&object.name, class.name, device.class().name)?;
                    (device, None)
                }
            };
            self.shape.add_device(&object.name, class.name)?;
            self.built.push(Built {
                path: object.name.clone(),
                class,
                device,
                instance,
                domain: self.assigned[i],
                space: object.space.map(|SpaceId(id)| id as usize),
                requester,
            });
        }
        Ok(())
    }

    /// Substitute bound bytes for every media slot `object` names.
    ///
    /// Returns `None` when the object has no media property at all, which is
    /// every object in most machines — the common path clones nothing.
    fn bind_media(
        &self,
        object: &crate::machine::resolver::Object,
        class: &'static DeviceClass,
    ) -> Result<Option<Props>> {
        let mut out: Option<Props> = None;
        for spec in class
            .properties
            .iter()
            .filter(|p| p.kind == ValueKind::Media)
        {
            let slot = match object.props.get(spec.name) {
                // Already bytes: a caller built the `Props` itself, which is
                // how a test or an embedder skips the slot dance entirely.
                None | Some(Value::Media(_)) => continue,
                Some(Value::Str(slot)) => slot,
                Some(other) => {
                    return Err(config(
                        object.name.clone(),
                        format!(
                            "`{}` takes the name of a media slot, not {} {other}",
                            spec.name,
                            other.kind()
                        ),
                    ));
                }
            };
            let media = self.options.media.get(slot).ok_or_else(|| {
                let bound: Vec<&str> = self.options.media.slots().collect();
                config(
                    object.name.clone(),
                    format!(
                        "`{}` names the media slot `{slot}`, which nothing is bound to; bound \
                         slots are {}",
                        spec.name,
                        list(&bound)
                    ),
                )
            })?;
            out.get_or_insert_with(|| object.props.clone())
                .insert(spec.name, Value::Media(media.clone()));
        }
        Ok(out)
    }

    fn realize_devices(&mut self) -> Result<()> {
        for i in 0..self.built.len() {
            let device = Arc::clone(&self.built[i].device);
            let path = self.built[i].path.clone();
            let requester = self.built[i].requester;
            let mut ctx = RealizeCtx::new(&path, requester, &mut self.deferred);
            let outcome = device.realize(&mut ctx);
            // Drain whatever the handler queued before deciding what to do
            // with its result: an action pushed by a realize that then failed
            // still has to run, or the device is left mid-operation.
            self.deferred.drain();
            outcome?;
            self.realized += 1;
        }
        Ok(())
    }

    /// Undo realize, in reverse order, on the way out of a failure.
    ///
    /// Errors from `unrealize` are dropped: the machine is being abandoned and
    /// there is nothing left to report them to. The first failure is what the
    /// caller wanted to know.
    fn unrealize_all(&mut self) {
        for i in (0..self.realized).rev() {
            let device = Arc::clone(&self.built[i].device);
            let path = self.built[i].path.clone();
            let requester = self.built[i].requester;
            let mut ctx = RealizeCtx::new(&path, requester, &mut self.deferred);
            let _ = device.unrealize(&mut ctx);
        }
        self.deferred.drain();
    }

    fn bind_devices(&mut self, spaces: &[(String, Arc<AddressSpace>)]) -> Result<()> {
        for built in &self.built {
            let Some(instance) = built.instance.as_ref() else {
                continue;
            };
            let ctx = BindCtx {
                path: &built.path,
                requester: built.requester,
                domain: built.domain,
                space: built.space.and_then(|i| spaces.get(i)).map(|(_, s)| s),
                spaces,
            };
            instance.bind(&ctx)?;
        }
        Ok(())
    }

    /// Hand every runnable device — and every lazily-advanced one — to the
    /// scheduler, and produce the machine's device table.
    ///
    /// Borrows rather than consumes `self.built`: a failure here still has to
    /// be able to unrealize what was realized, and a device table that has been
    /// moved out cannot be walked back.
    ///
    /// # Lazily advanced devices (§4.2)
    ///
    /// A class that says [`Device::is_lazy`] is registered on its clock domain
    /// and handed the [`LazyHandle`](crate::core::sched::LazyHandle) back, so
    /// that its own `MemOps::read` can catch it up. The handle rather than a
    /// wrapper around the region, for two reasons: a device knows which of its
    /// registers are sampled and which are not, and a `map` statement may name
    /// several windows of one device (the APU names three), which a per-mapping
    /// wrapper would have to reproduce and keep in step.
    fn register_with_scheduler(&self, sched: &mut Scheduler) -> Result<Vec<DeviceEntry>> {
        let mut out = Vec::with_capacity(self.built.len());
        for built in &self.built {
            let mut runnable = None;
            if let Some(instance) = built.instance.as_ref()
                && instance.is_runnable()
            {
                let domain = built.domain.ok_or_else(|| {
                    config(
                        built.path.clone(),
                        "a device that takes execution budgets needs a clock domain",
                    )
                })?;
                runnable = Some(
                    sched.add_runnable(domain, Box::new(RunAdapter::new(Arc::clone(instance)))),
                );
            }
            let mut lazy = None;
            if built.device.is_lazy() {
                let domain = built.domain.ok_or_else(|| {
                    config(
                        built.path.clone(),
                        "a device that is advanced on access needs a clock domain: its tick is \
                         counted in one, and catch-up has no target without it",
                    )
                })?;
                let id = sched.add_lazy_device(
                    domain,
                    Box::new(LazyAdapter::new(Arc::clone(&built.device))),
                );
                let handle = sched
                    .lazy_handle(id)
                    .map_err(|e| config(built.path.clone(), e.to_string()))?;
                built.device.attach_lazy(handle);
                lazy = Some(id);
            }
            out.push(DeviceEntry {
                path: built.path.clone(),
                class: built.class,
                device: Arc::clone(&built.device),
                instance: built.instance.clone(),
                domain: built.domain,
                space: built.space,
                requester: built.requester,
                runnable,
                lazy,
            });
        }
        Ok(out)
    }

    // -- the memory map ----------------------------------------------------

    fn map_regions(&mut self) -> Result<Vec<(String, AddressSpace)>> {
        let spaces = core::mem::take(&mut self.spaces);
        for mapping in &self.machine.maps {
            let index = mapping.space.0 as usize;
            let Some((name, space)) = spaces.get(index) else {
                return Err(config(
                    self.machine.name.clone(),
                    format!("mapping names address space {index}, which does not exist"),
                ));
            };
            let at = format!("map {} {:#x}", name, mapping.base);
            let region = self.window(&mapping.target, mapping.size, &at)?;
            let (priority, endian) = mapping_attrs(mapping, &at)?;
            let region = match endian {
                // Per-mapping byte order is a property of the *window*, not of
                // the target — a 16-bit big-endian aperture onto a
                // little-endian device is an ordinary thing (§4.1) — so it
                // takes an alias to carry it.
                Some(e) => {
                    let order = match e {
                        Endian::Little => "little",
                        Endian::Big => "big",
                    };
                    Arc::new(
                        Region::alias(
                            format!("{}@{order}-endian", region.name()),
                            region,
                            0,
                            mapping.size,
                        )?
                        .with_endian(e),
                    )
                }
                None => region,
            };
            self.shape
                .add_region(name, region.name(), mapping.base, mapping.size);
            // One guard per statement rather than one for the whole loop:
            // consecutive `map` statements may name different spaces, and two
            // topology guards at once is a lock-order violation.
            space
                .topology()
                .map_with(SpaceMapping::new(region, mapping.base).with_priority(priority))?;
        }
        Ok(spaces)
    }

    /// The region a `map` statement puts at its base, clipped to `size`.
    fn window(&self, target: &MapTarget, size: u64, at: &str) -> Result<RegionRef> {
        match target {
            MapTarget::Region { .. } => {
                let region = self.base_region(target, at)?;
                match size.cmp(&region.len()) {
                    core::cmp::Ordering::Equal => Ok(region),
                    core::cmp::Ordering::Less => Ok(Arc::new(Region::alias(
                        format!("{}[{size:#x}]", region.name()),
                        region,
                        0,
                        size,
                    )?)),
                    core::cmp::Ordering::Greater => Err(config(
                        at.to_string(),
                        format!(
                            "`{}` is {:#x} bytes but the mapping is {size:#x}; use mirror() to \
                             repeat it",
                            region.name(),
                            region.len()
                        ),
                    )),
                }
            }
            MapTarget::Mirror { inner, .. } => {
                let region = self.base_region(inner, at)?;
                Ok(Arc::new(Region::mirror(
                    format!("mirror({})", region.name()),
                    region,
                    size,
                )?))
            }
            MapTarget::Alias { inner, offset, .. } => {
                let region = self.base_region(inner, at)?;
                Ok(Arc::new(Region::alias(
                    format!("alias({}+{offset:#x})", region.name()),
                    region,
                    *offset,
                    size,
                )?))
            }
        }
    }

    /// The region a target names, at its natural size.
    fn base_region(&self, target: &MapTarget, at: &str) -> Result<RegionRef> {
        match target {
            MapTarget::Region { object, region, .. } => {
                let built = self.device(*object, at)?;
                let name = region.as_deref().unwrap_or("");
                let instance = built.instance.as_ref().ok_or_else(|| {
                    config(
                        at.to_string(),
                        format!(
                            "`{}` is class `{}`, which publishes no regions to this build",
                            built.path, built.class.name
                        ),
                    )
                })?;
                instance.region(name).ok_or_else(|| {
                    config(
                        at.to_string(),
                        if name.is_empty() {
                            format!("`{}` has no region of its own", built.path)
                        } else {
                            format!("`{}` has no region `{name}`", built.path)
                        },
                    )
                })
            }
            // A window with no size of its own is just its target; the outer
            // window is what carries the size.
            MapTarget::Mirror { inner, .. } => self.base_region(inner, at),
            MapTarget::Alias { inner, offset, .. } => {
                let region = self.base_region(inner, at)?;
                let len = region.len().checked_sub(*offset).ok_or_else(|| {
                    config(
                        at.to_string(),
                        format!(
                            "alias offset {offset:#x} is past the end of `{}`",
                            region.name()
                        ),
                    )
                })?;
                Ok(Arc::new(Region::alias(
                    format!("alias({}+{offset:#x})", region.name()),
                    region,
                    *offset,
                    len,
                )?))
            }
        }
    }

    // -- wires -------------------------------------------------------------

    /// Build one net per connected group of pins, then hand out the sources.
    ///
    /// A net is a *connected component* of the wire statements, not one
    /// statement: `a.out -> c.in` and `b.out -> c.in` are one piece of copper
    /// with two drivers, which is exactly what §4.3's wired-OR is and what
    /// `Wire`'s per-source state exists for.
    fn build_wires(&mut self) -> Result<(Vec<Net>, Vec<PinRef>)> {
        let mut pins = Pins::default();
        for wire in &self.machine.wires {
            let from = pins.intern(wire.from.object, &wire.from.port);
            let to = pins.intern(wire.to.object, &wire.to.port);
            pins.drives[from] = true;
            pins.receives[to] = true;
            pins.union(from, to);
        }

        let allocator = WireIdAllocator::new();
        let mut nets: Vec<Net> = Vec::new();
        let mut ids: Vec<WireId> = alloc::vec![WireId::NONE; pins.len()];

        // Group pins by their root, in first-appearance order, so two runs over
        // one file build the nets and allocate the ids identically.
        let mut members: Vec<(usize, Vec<usize>)> = Vec::new();
        for pin in 0..pins.len() {
            let root = pins.find(pin);
            match members.iter_mut().find(|(r, _)| *r == root) {
                Some((_, group)) => group.push(pin),
                None => members.push((root, alloc::vec![pin])),
            }
        }

        for (_root, group) in members {
            let sources: Vec<usize> = group.iter().copied().filter(|p| pins.drives[*p]).collect();
            for pin in &sources {
                ids[*pin] = allocator.alloc();
            }
            let source_ids: Vec<WireId> = sources.iter().map(|p| ids[*p]).collect();
            let mut builder = Wire::builder().sources(&source_ids);
            let receivers: Vec<usize> =
                group.iter().copied().filter(|p| pins.receives[*p]).collect();
            for pin in receivers.iter().copied() {
                let (object, port) = pins.pin(pin);
                let built = self.device(object, "wire")?;
                let instance = built.instance.as_ref().ok_or_else(|| {
                    config(
                        built.path.clone(),
                        format!(
                            "class `{}` publishes no pins to this build, so `{port}` cannot be \
                             driven",
                            built.class.name
                        ),
                    )
                })?;
                let sink = instance.sink(port, &source_ids).ok_or_else(|| {
                    config(
                        built.path.clone(),
                        format!("no input pin `{port}` on this device"),
                    )
                })?;
                // Weak, always: the machine owns devices and a wire merely
                // refers to them, which is the weak edge §4.3 requires so an
                // IRQ/ack loop does not leak.
                builder = builder.sink_weak(Arc::downgrade(&sink.sink), sink.line);
            }
            let wire = builder.build_shared();

            let mut refs = Vec::with_capacity(sources.len());
            for pin in sources {
                let (object, port) = pins.pin(pin);
                let built = self.device(object, "wire")?;
                let instance = built.instance.as_ref().ok_or_else(|| {
                    config(
                        built.path.clone(),
                        format!(
                            "class `{}` publishes no pins to this build, so `{port}` cannot drive",
                            built.class.name
                        ),
                    )
                })?;
                instance
                    .connect(port, WireSource::new(Arc::clone(&wire), ids[pin]))
                    .map_err(|e| config(built.path.clone(), e.to_string()))?;
                // The reverse half of a vectored interrupt (`core::wire`'s
                // `IntAck`): a controller that answers an acknowledge cycle
                // offers a handler here, and every CPU on the same net is told
                // about it. Weak, for the same reason the sinks are: the
                // machine owns devices and the wire only refers to them.
                if let Some(ack) = instance.int_ack(port) {
                    let weak = Arc::downgrade(&ack);
                    for sink_pin in receivers.iter().copied() {
                        let (sink_object, sink_port) = pins.pin(sink_pin);
                        let sink_built = self.device(sink_object, "wire")?;
                        if let Some(sink_instance) = sink_built.instance.as_ref() {
                            sink_instance.attach_int_ack(sink_port, Weak::clone(&weak));
                        }
                    }
                    // Nothing here keeps `ack` alive: the driving device owns
                    // it, exactly as it owns the `Arc` behind a `SinkPin`. A
                    // device that built one on the fly would hand out a weak
                    // reference that is already dead, which is the same
                    // contract `Device::sink` has.
                    drop(ack);
                }
                // The data half of a DMA request line, by the same route: the
                // peripheral drives `DRQ`, and the controller on the other end
                // of that net is what moves its bytes.
                if let Some(peer) = instance.dma_peripheral(port) {
                    let weak = Arc::downgrade(&peer);
                    for sink_pin in receivers.iter().copied() {
                        let (sink_object, sink_port) = pins.pin(sink_pin);
                        let sink_built = self.device(sink_object, "wire")?;
                        if let Some(sink_instance) = sink_built.instance.as_ref() {
                            sink_instance.attach_dma_peripheral(sink_port, Weak::clone(&weak));
                        }
                    }
                    drop(peer);
                }
                refs.push(PinRef {
                    device: object.0 as usize,
                    port: port.to_string(),
                    id: ids[pin],
                });
            }
            nets.push(Net {
                wire,
                sources: refs,
            });
        }

        Ok((nets, self.sweep_order(&pins, &ids)?))
    }

    /// The order the sweep announces levels in (§4.3), as pins.
    fn sweep_order(&self, pins: &Pins, ids: &[WireId]) -> Result<Vec<PinRef>> {
        let mut table = ClassTable::new();
        for built in &self.built {
            if let Some(instance) = built.instance.as_ref()
                && instance.combinational()
            {
                table.insert(ClassSchema::new(built.class.name).combinational());
            }
        }
        // `realize_order` is the one computation: it returns the order *and*
        // rejects the combinational cycles that would have no order. Recomputing
        // it here would be two rules that could disagree.
        let order = realize_order(self.machine, &table).map_err(|d| Error::Config {
            at: self.machine.name.clone(),
            message: d.message,
        })?;

        let mut seen = alloc::vec![false; pins.len()];
        let mut out = Vec::new();
        for index in order {
            let Some(wire) = self.machine.wires.get(index) else {
                continue;
            };
            let Some(pin) = pins.lookup(wire.from.object, &wire.from.port) else {
                continue;
            };
            if core::mem::replace(&mut seen[pin], true) {
                continue;
            }
            out.push(PinRef {
                device: wire.from.object.0 as usize,
                port: wire.from.port.clone(),
                id: ids[pin],
            });
        }
        Ok(out)
    }

    fn device(&self, id: ObjectId, at: &str) -> Result<&Built> {
        self.built
            .get(id.0 as usize)
            .ok_or_else(|| config(at.to_string(), format!("object {} does not exist", id.0)))
    }
}

// ---------------------------------------------------------------------------
// helpers
// ---------------------------------------------------------------------------

/// Interned wire pins with a union-find over them.
#[derive(Debug, Default)]
struct Pins {
    names: Vec<(ObjectId, String)>,
    index: BTreeMap<(u32, String), usize>,
    parent: Vec<usize>,
    drives: Vec<bool>,
    receives: Vec<bool>,
}

impl Pins {
    fn intern(&mut self, object: ObjectId, port: &str) -> usize {
        let key = (object.0, port.to_string());
        if let Some(i) = self.index.get(&key) {
            return *i;
        }
        let i = self.names.len();
        self.names.push((object, port.to_string()));
        self.index.insert(key, i);
        self.parent.push(i);
        self.drives.push(false);
        self.receives.push(false);
        i
    }

    fn lookup(&self, object: ObjectId, port: &str) -> Option<usize> {
        self.index.get(&(object.0, port.to_string())).copied()
    }

    fn len(&self) -> usize {
        self.names.len()
    }

    fn pin(&self, i: usize) -> (ObjectId, &str) {
        let (object, port) = &self.names[i];
        (*object, port.as_str())
    }

    fn find(&self, mut x: usize) -> usize {
        // No path compression: `&self` keeps the caller's borrows simple and a
        // machine's wire graph is a handful of pins deep.
        while self.parent[x] != x {
            x = self.parent[x];
        }
        x
    }

    fn union(&mut self, a: usize, b: usize) {
        let (ra, rb) = (self.find(a), self.find(b));
        if ra != rb {
            // The lower index wins, so the net's identity is its
            // first-appearing pin whatever order the statements merge in.
            let (lo, hi) = if ra < rb { (ra, rb) } else { (rb, ra) };
            self.parent[hi] = lo;
        }
    }
}

/// Where a clock's parent domain came from.
enum ParentLookup {
    /// The parent exists.
    Ready(DomainId),
    /// The parent is another object that has not been given its domain yet.
    Waiting,
    /// The parent is an object with no clock of its own.
    Missing(String),
}

fn parent_of(clock: &Clock, osc_roots: &[DomainId], assigned: &[Option<DomainId>]) -> ParentLookup {
    match clock.parent {
        ClockParent::Osc(id) => match osc_roots.get(id.0 as usize) {
            Some(root) => ParentLookup::Ready(*root),
            None => ParentLookup::Missing(format!("oscillator {}", id.0)),
        },
        ClockParent::Object(id) => match assigned.get(id.0 as usize) {
            Some(Some(domain)) => ParentLookup::Ready(*domain),
            Some(None) => ParentLookup::Waiting,
            None => ParentLookup::Missing(format!("object {}", id.0)),
        },
    }
}

/// `priority` and `endian` off a `map` statement's trailing block.
fn mapping_attrs(mapping: &Mapping, at: &str) -> Result<(i32, Option<Endian>)> {
    let mut r = mapping.props.reader();
    let priority: i32 = r
        .or("priority", 0i32)
        .map_err(|e| config(at.to_string(), e.to_string()))?;
    let endian = match r
        .optional_str("endian")
        .map_err(|e| config(at.to_string(), e.to_string()))?
    {
        Some("little") => Some(Endian::Little),
        Some("big") => Some(Endian::Big),
        Some(other) => {
            return Err(config(
                at.to_string(),
                format!("unknown byte order `{other}`; expected `little` or `big`"),
            ));
        }
        None => None,
    };
    let unused = r.unused();
    if !unused.is_empty() {
        return Err(config(
            at.to_string(),
            format!(
                "unknown mapping attribute {}; a mapping takes `priority` and `endian`",
                list(&unused)
            ),
        ));
    }
    Ok((priority, endian))
}

/// A machine-file frequency as a clock-forest one.
///
/// The two `Rational`s are different types on purpose: the description's is
/// signed and 128-bit because an expression can produce anything, and the
/// forest's is unsigned and 64-bit because a frequency is neither negative nor
/// astronomically large. This is the check between them.
fn to_clock_rational(hz: &crate::machine::rational::Rational) -> Option<ClockRational> {
    let num = u64::try_from(hz.numerator()).ok()?;
    let den = u64::try_from(hz.denominator()).ok()?;
    ClockRational::new(num, den).ok()
}

fn check_class(instance: &str, wanted: &str, got: &str) -> Result<()> {
    if wanted == got {
        return Ok(());
    }
    Err(config(
        instance.to_string(),
        format!("class `{wanted}` constructed a device of class `{got}`"),
    ))
}

fn config(at: impl Into<String>, message: impl Into<String>) -> Error {
    Error::Config {
        at: at.into(),
        message: message.into(),
    }
}

/// `` `a`, `b` `` — the same shape the validator's messages use.
fn list(names: &[&str]) -> String {
    let mut out = String::new();
    for (i, name) in names.iter().enumerate() {
        if i != 0 {
            out.push_str(", ");
        }
        out.push('`');
        out.push_str(name);
        out.push('`');
    }
    if out.is_empty() {
        out.push_str("none");
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::device::{DeviceClass, PropertySpec};
    use crate::core::props::ValueKind;
    use crate::core::sched::{Budget, Consumed};
    use crate::core::space::{MemAttrs, MemOps, MemResult, RamStore};
    use crate::core::state::{ChunkReader, ChunkWriter, Migrations, Sink, Source, StateReader};
    use crate::core::sync::Mutex;
    use crate::core::value::Width;
    use crate::core::wire::{FanIn, Level, Resolve, WireSink};
    use crate::machine::{BuildOptions, build};
    use core::sync::atomic::{AtomicU32, AtomicU64, Ordering::Relaxed};

    // -----------------------------------------------------------------
    // test.ram — memory, and the region a `map` statement names
    // -----------------------------------------------------------------

    #[derive(Debug)]
    struct Ram {
        store: Arc<RamStore>,
        region: RegionRef,
    }

    impl Ram {
        fn new(props: &Props) -> Result<Ram> {
            let mut r = props.reader();
            let size = r.require_size("size")?;
            r.finish()?;
            let store = Arc::new(RamStore::new(size));
            let region = Arc::new(Region::ram("ram", Arc::clone(&store)));
            Ok(Ram { store, region })
        }
    }

    impl Device for Ram {
        fn class(&self) -> &'static DeviceClass {
            &RAM_CLASS
        }
        fn realize(&self, _ctx: &mut RealizeCtx<'_>) -> Result<()> {
            Ok(())
        }
        fn reset(&self, kind: ResetKind) {
            // A warm reset leaves memory alone: that is what makes a reset
            // vector's "did we come from power-on?" check work.
            if kind == ResetKind::Cold {
                let _ = self.store.fill(0, self.store.len(), 0);
            }
        }
        fn save(&self, w: &mut ChunkWriter<'_>) -> Result<()> {
            let mut bytes = alloc::vec![0u8; self.store.len() as usize];
            self.store.read_at(0, &mut bytes)?;
            w.write_bytes(&bytes)
        }
        fn load(&self, r: &mut ChunkReader<'_>) -> Result<()> {
            let bytes = r.read_bytes()?;
            self.store.write_at(0, bytes)?;
            Ok(())
        }
        fn region(&self, name: &str) -> Option<RegionRef> {
            name.is_empty().then(|| Arc::clone(&self.region))
        }
    }

    impl Instance for Ram {}

    static RAM_CLASS: DeviceClass = DeviceClass {
        name: "test.ram",
        version: 1,
        summary: "test-only read/write memory",
        properties: &[PropertySpec {
            name: "size",
            kind: ValueKind::Size,
            required: true,
            summary: "how many bytes",
        }],
        construct: |props| Ok(Box::new(Ram::new(props)?)),
    };

    fn make_ram(props: &Props) -> Result<Arc<dyn Instance>> {
        Ok(Arc::new(Ram::new(props)?))
    }

    // -----------------------------------------------------------------
    // test.timer — MMIO registers, an event handler, a deferred action
    // -----------------------------------------------------------------

    /// The timer's registers, shared between the device and its MMIO region.
    ///
    /// Four `u32`s: fire count, deferred-action count, output level, and the
    /// token of the last event. The test reads them the way a guest would,
    /// through the address space, rather than reaching into the device.
    #[derive(Debug, Default)]
    struct TimerRegs {
        fires: AtomicU32,
        deferred: AtomicU32,
        level: AtomicU32,
        token: AtomicU32,
    }

    impl TimerRegs {
        fn image(&self) -> [u8; 16] {
            let mut out = [0u8; 16];
            for (i, v) in [
                self.fires.load(Relaxed),
                self.deferred.load(Relaxed),
                self.level.load(Relaxed),
                self.token.load(Relaxed),
            ]
            .into_iter()
            .enumerate()
            {
                out[i * 4..i * 4 + 4].copy_from_slice(&v.to_le_bytes());
            }
            out
        }
    }

    impl MemOps for TimerRegs {
        fn read(&self, offset: u64, dst: &mut [u8], _attrs: MemAttrs) -> MemResult {
            // No side effects at all, so `MemAttrs::debug` needs no special
            // case: a debug read of these registers is the same read.
            let image = self.image();
            let start = offset as usize;
            let end = start
                .checked_add(dst.len())
                .filter(|e| *e <= image.len())
                .ok_or(crate::core::BusError::BadAccess)?;
            dst.copy_from_slice(&image[start..end]);
            Ok(())
        }

        fn write(&self, offset: u64, src: &[u8], _attrs: MemAttrs) -> MemResult {
            // Only the token register is writable; the counters are the
            // device's own state.
            if offset == 12 && src.len() == 4 {
                let mut b = [0u8; 4];
                b.copy_from_slice(src);
                self.token.store(u32::from_le_bytes(b), Relaxed);
                return Ok(());
            }
            Err(crate::core::BusError::BadAccess)
        }
    }

    #[derive(Debug)]
    struct Timer {
        regs: Arc<TimerRegs>,
        region: RegionRef,
        out: Mutex<Option<WireSource>>,
    }

    impl Timer {
        fn new(props: &Props) -> Result<Timer> {
            props.reader().finish()?;
            let regs = Arc::new(TimerRegs::default());
            let ops: Arc<dyn MemOps> = Arc::clone(&regs) as Arc<dyn MemOps>;
            let region = Arc::new(Region::io("timer.regs", 16, ops));
            Ok(Timer {
                regs,
                region,
                out: Mutex::new(None),
            })
        }
    }

    impl Device for Timer {
        fn class(&self) -> &'static DeviceClass {
            &TIMER_CLASS
        }
        fn realize(&self, _ctx: &mut RealizeCtx<'_>) -> Result<()> {
            Ok(())
        }
        fn reset(&self, _kind: ResetKind) {
            self.regs.fires.store(0, Relaxed);
            self.regs.deferred.store(0, Relaxed);
            self.regs.level.store(0, Relaxed);
            self.regs.token.store(0, Relaxed);
            // The line itself is driven by the sweep that follows reset, which
            // is the whole point of the sweep.
        }
        fn save(&self, w: &mut ChunkWriter<'_>) -> Result<()> {
            for v in self.regs.image() {
                w.write_u8(v)?;
            }
            Ok(())
        }
        fn load(&self, r: &mut ChunkReader<'_>) -> Result<()> {
            let mut image = [0u8; 16];
            for slot in &mut image {
                *slot = r.read_u8()?;
            }
            let word = |i: usize| {
                let mut b = [0u8; 4];
                b.copy_from_slice(&image[i * 4..i * 4 + 4]);
                u32::from_le_bytes(b)
            };
            self.regs.fires.store(word(0), Relaxed);
            self.regs.deferred.store(word(1), Relaxed);
            self.regs.level.store(word(2), Relaxed);
            self.regs.token.store(word(3), Relaxed);
            Ok(())
        }
        fn region(&self, name: &str) -> Option<RegionRef> {
            (name == "regs").then(|| Arc::clone(&self.region))
        }

        fn connect(&self, port: &str, source: WireSource) -> Result<()> {
            if port != "out" {
                return Err(config("test.timer", "no such output pin"));
            }
            *self.out.lock() = Some(source);
            Ok(())
        }

        fn announce(&self, port: &str) {
            if port == "out" {
                let level = Level::from_bool(self.regs.level.load(Relaxed) != 0);
                // Take the handle, release the lock, then drive: an outward
                // call under a lock is what the re-entrancy contract forbids.
                let out = self.out.lock().clone();
                if let Some(out) = out {
                    out.set(level);
                }
            }
        }

        fn event(&self, token: u64, deferred: &mut Deferred) {
            self.regs.fires.fetch_add(1, Relaxed);
            self.regs.token.store(token as u32, Relaxed);
            self.regs.level.store(1, Relaxed);
            let out = self.out.lock().clone();
            if let Some(out) = out {
                out.raise();
            }
            // The queue exists so a handler can act outward *after* it returns.
            let regs = Arc::clone(&self.regs);
            deferred.push(move || {
                regs.deferred.fetch_add(1, Relaxed);
            });
        }
    }

    impl Instance for Timer {}

    static TIMER_CLASS: DeviceClass = DeviceClass {
        name: "test.timer",
        version: 1,
        summary: "test-only timer with four registers and an output line",
        properties: &[],
        construct: |props| Ok(Box::new(Timer::new(props)?)),
    };

    fn make_timer(props: &Props) -> Result<Arc<dyn Instance>> {
        Ok(Arc::new(Timer::new(props)?))
    }

    // -----------------------------------------------------------------
    // test.not — a combinational inverter, the reason the sweep exists
    // -----------------------------------------------------------------

    /// The inverter's input pin: a fan-in plus the output it recomputes.
    #[derive(Debug)]
    struct Inverter {
        fan: Mutex<Option<FanIn>>,
        out: Mutex<Option<WireSource>>,
    }

    impl Inverter {
        fn level(&self) -> Level {
            self.fan
                .lock()
                .as_ref()
                .map_or(Level::Low, |f| f.resolve(Resolve::Or))
        }
    }

    impl WireSink for Inverter {
        fn set_level(&self, src: WireId, _line: u32, level: Level) {
            // Mutate own state in a short critical section...
            let resolved = {
                let fan = self.fan.lock();
                fan.as_ref().map(|f| {
                    f.set(src, level);
                    f.resolve(Resolve::Or)
                })
            };
            // ...release it, and only then act outward.
            let out = self.out.lock().clone();
            if let (Some(resolved), Some(out)) = (resolved, out) {
                out.set(resolved.inverted());
            }
        }
    }

    #[derive(Debug)]
    struct Not {
        pin: Arc<Inverter>,
    }

    impl Device for Not {
        fn class(&self) -> &'static DeviceClass {
            &NOT_CLASS
        }
        fn realize(&self, _ctx: &mut RealizeCtx<'_>) -> Result<()> {
            Ok(())
        }
        fn reset(&self, _kind: ResetKind) {}
        fn combinational(&self) -> bool {
            true
        }

        fn sink(&self, port: &str, sources: &[WireId]) -> Option<SinkPin> {
            if port != "in" {
                return None;
            }
            // The fan-in can only be built now: it is told its sources at
            // construction, and no `WireId` existed when this device was made.
            *self.pin.fan.lock() = Some(FanIn::new(sources));
            Some(SinkPin {
                // The machine keeps only a weak reference; this `Arc` is a
                // clone of one the device owns.
                sink: Arc::clone(&self.pin) as Arc<dyn WireSink>,
                line: 0,
            })
        }

        fn connect(&self, port: &str, source: WireSource) -> Result<()> {
            if port != "out" {
                return Err(config("test.not", "no such output pin"));
            }
            *self.pin.out.lock() = Some(source);
            Ok(())
        }

        fn announce(&self, port: &str) {
            if port == "out" {
                let level = self.pin.level().inverted();
                let out = self.pin.out.lock().clone();
                if let Some(out) = out {
                    out.set(level);
                }
            }
        }
    }

    impl Instance for Not {}

    fn new_not() -> Not {
        Not {
            pin: Arc::new(Inverter {
                fan: Mutex::new(None),
                out: Mutex::new(None),
            }),
        }
    }

    static NOT_CLASS: DeviceClass = DeviceClass {
        name: "test.not",
        version: 1,
        summary: "test-only inverter: idles high, which is why realize sweeps",
        properties: &[],
        construct: |_props| Ok(Box::new(new_not())),
    };

    fn make_not(_props: &Props) -> Result<Arc<dyn Instance>> {
        Ok(Arc::new(new_not()))
    }

    // -----------------------------------------------------------------
    // test.cpu — a stub that takes budgets and reaches its address space
    // -----------------------------------------------------------------

    /// The CPU's interrupt input.
    #[derive(Debug)]
    struct IrqPin {
        fan: Mutex<Option<FanIn>>,
    }

    impl IrqPin {
        fn level(&self) -> Level {
            self.fan
                .lock()
                .as_ref()
                .map_or(Level::Low, |f| f.resolve(Resolve::Or))
        }
    }

    impl WireSink for IrqPin {
        fn set_level(&self, src: WireId, _line: u32, level: Level) {
            if let Some(fan) = self.fan.lock().as_ref() {
                fan.set(src, level);
            }
        }
    }

    #[derive(Debug)]
    struct Cpu {
        pc: AtomicU64,
        sum: AtomicU64,
        cycles: AtomicU64,
        irq: Arc<IrqPin>,
        space: Mutex<Option<Arc<AddressSpace>>>,
        requester: Mutex<RequesterId>,
    }

    impl Cpu {
        fn new(props: &Props) -> Result<Cpu> {
            let mut r = props.reader();
            // Accepted so the fixture can carry §5's `engine = "interp"`.
            let _ = r.or_str("engine", "interp")?;
            r.finish()?;
            Ok(Cpu {
                pc: AtomicU64::new(0),
                sum: AtomicU64::new(0),
                cycles: AtomicU64::new(0),
                irq: Arc::new(IrqPin {
                    fan: Mutex::new(None),
                }),
                space: Mutex::new(None),
                requester: Mutex::new(RequesterId::ANONYMOUS),
            })
        }
    }

    impl Device for Cpu {
        fn class(&self) -> &'static DeviceClass {
            &CPU_CLASS
        }
        fn realize(&self, _ctx: &mut RealizeCtx<'_>) -> Result<()> {
            Ok(())
        }
        fn reset(&self, _kind: ResetKind) {
            self.pc.store(0, Relaxed);
            self.sum.store(0, Relaxed);
            self.cycles.store(0, Relaxed);
        }
        fn save(&self, w: &mut ChunkWriter<'_>) -> Result<()> {
            w.write_u64(self.pc.load(Relaxed))?;
            w.write_u64(self.sum.load(Relaxed))?;
            w.write_u64(self.cycles.load(Relaxed))
        }
        fn load(&self, r: &mut ChunkReader<'_>) -> Result<()> {
            self.pc.store(r.read_u64()?, Relaxed);
            self.sum.store(r.read_u64()?, Relaxed);
            self.cycles.store(r.read_u64()?, Relaxed);
            Ok(())
        }
        fn sink(&self, port: &str, sources: &[WireId]) -> Option<SinkPin> {
            if port != "irq" {
                return None;
            }
            *self.irq.fan.lock() = Some(FanIn::new(sources));
            Some(SinkPin {
                sink: Arc::clone(&self.irq) as Arc<dyn WireSink>,
                line: 0,
            })
        }

        fn is_runnable(&self) -> bool {
            true
        }

        fn run(&self, budget: Budget) -> Consumed {
            let space = self.space.lock().clone();
            let Some(space) = space else {
                return Consumed::default();
            };
            let attrs = MemAttrs::DEFAULT.with_requester(*self.requester.lock());
            let mut pc = self.pc.load(Relaxed);
            let mut sum = self.sum.load(Relaxed);
            for _ in 0..budget.ticks {
                if let Ok(v) = space.read(pc, Width::U8, attrs) {
                    sum = sum.wrapping_add(v);
                }
                // Guest arithmetic wraps by definition; the mask is the guest's
                // width, applied after the add rather than before.
                pc = pc.wrapping_add(1) & 0x7f;
            }
            let cycles = self.cycles.load(Relaxed).wrapping_add(budget.ticks);
            self.pc.store(pc, Relaxed);
            self.sum.store(sum, Relaxed);
            self.cycles.store(cycles, Relaxed);
            // Two effects a test can read back through the bus: how far the CPU
            // has run, and whether its interrupt line is asserted.
            let _ = space.write(0x0100, Width::U32, cycles & 0xffff_ffff, attrs);
            let _ = space.write(
                0x0104,
                Width::U8,
                u64::from(self.irq.level().is_high()),
                attrs,
            );
            Consumed::new(budget.ticks)
        }
    }

    impl Instance for Cpu {
        fn bind(&self, ctx: &BindCtx<'_>) -> Result<()> {
            let space = ctx
                .space()
                .ok_or_else(|| config(ctx.path(), "a cpu needs an address space (`space = …`)"))?;
            *self.space.lock() = Some(Arc::clone(space));
            *self.requester.lock() = ctx.requester();
            Ok(())
        }
    }

    static CPU_CLASS: DeviceClass = DeviceClass {
        name: "test.cpu",
        version: 1,
        summary: "test-only runnable that reads its own address space",
        properties: &[PropertySpec {
            name: "engine",
            kind: ValueKind::Str,
            required: false,
            summary: "which execution engine",
        }],
        construct: |props| Ok(Box::new(Cpu::new(props)?)),
    };

    fn make_cpu(props: &Props) -> Result<Arc<dyn Instance>> {
        Ok(Arc::new(Cpu::new(props)?))
    }

    // -----------------------------------------------------------------
    // test.cart — a class whose whole configuration is host-supplied bytes
    // -----------------------------------------------------------------

    /// A device that exists only to have a media property.
    ///
    /// It publishes the bound bytes as a read-only region, which is what a
    /// cartridge or a firmware ROM does and is the only way a test can prove
    /// the *right* bytes arrived rather than merely some.
    #[derive(Debug)]
    struct Cart {
        region: RegionRef,
    }

    impl Cart {
        fn new(props: &Props) -> Result<Cart> {
            let mut r = props.reader();
            let image = r.require_media("image")?;
            r.finish()?;
            let store = Arc::new(RamStore::new(image.len()));
            store.write_at(0, image.bytes())?;
            Ok(Cart {
                region: Arc::new(Region::ram("cart.image", store)),
            })
        }
    }

    impl Device for Cart {
        fn class(&self) -> &'static DeviceClass {
            &CART_CLASS
        }
        fn realize(&self, _ctx: &mut RealizeCtx<'_>) -> Result<()> {
            Ok(())
        }
        fn reset(&self, _kind: ResetKind) {}
        fn region(&self, name: &str) -> Option<RegionRef> {
            name.is_empty().then(|| Arc::clone(&self.region))
        }
    }

    impl Instance for Cart {}

    static CART_CLASS: DeviceClass = DeviceClass {
        name: "test.cart",
        version: 1,
        summary: "test-only device configured entirely by host-supplied bytes",
        properties: &[PropertySpec {
            name: "image",
            kind: ValueKind::Media,
            required: true,
            summary: "the image, as the name of a media slot",
        }],
        construct: |props| Ok(Box::new(Cart::new(props)?)),
    };

    // -----------------------------------------------------------------
    // test.witness / test.explode — the failed-realize path
    // -----------------------------------------------------------------

    /// How many times a witness has been unrealized.
    ///
    /// A `static` is safe here because exactly one test uses these two classes.
    static UNREALIZED: AtomicU32 = AtomicU32::new(0);

    #[derive(Debug)]
    struct Witness;

    impl Device for Witness {
        fn class(&self) -> &'static DeviceClass {
            &WITNESS_CLASS
        }
        fn realize(&self, _ctx: &mut RealizeCtx<'_>) -> Result<()> {
            Ok(())
        }
        fn unrealize(&self, _ctx: &mut RealizeCtx<'_>) -> Result<()> {
            UNREALIZED.fetch_add(1, Relaxed);
            Ok(())
        }
        fn reset(&self, _kind: ResetKind) {}
    }

    impl Instance for Witness {}

    static WITNESS_CLASS: DeviceClass = DeviceClass {
        name: "test.witness",
        version: 1,
        summary: "test-only device that records being unrealized",
        properties: &[],
        construct: |_props| Ok(Box::new(Witness)),
    };

    #[derive(Debug)]
    struct Explode;

    impl Device for Explode {
        fn class(&self) -> &'static DeviceClass {
            &EXPLODE_CLASS
        }
        fn realize(&self, ctx: &mut RealizeCtx<'_>) -> Result<()> {
            Err(ctx.error("this device always fails to realize"))
        }
        fn reset(&self, _kind: ResetKind) {}
    }

    impl Instance for Explode {}

    static EXPLODE_CLASS: DeviceClass = DeviceClass {
        name: "test.explode",
        version: 1,
        summary: "test-only device that fails to realize",
        properties: &[],
        construct: |_props| Ok(Box::new(Explode)),
    };

    // -----------------------------------------------------------------
    // fixtures
    // -----------------------------------------------------------------

    fn registry() -> Registry {
        let mut r = Registry::new();
        for class in [
            &RAM_CLASS,
            &TIMER_CLASS,
            &NOT_CLASS,
            &CPU_CLASS,
            &CART_CLASS,
            &WITNESS_CLASS,
            &EXPLODE_CLASS,
        ] {
            r.add(class).expect("distinct class names");
        }
        r
    }

    fn bindings() -> Bindings {
        Bindings::new()
            .with("test.ram", make_ram)
            .with("test.timer", make_timer)
            .with("test.not", make_not)
            .with("test.cpu", make_cpu)
            .with("test.cart", |props| Ok(Arc::new(Cart::new(props)?)))
            .with("test.witness", |_props| Ok(Arc::new(Witness)))
            .with("test.explode", |_props| Ok(Arc::new(Explode)))
    }

    /// One crystal, two domains at an exact 2:1 ratio, RAM mirrored into a
    /// 16-bit space, an MMIO aperture, and an inverter between the timer and
    /// the CPU's interrupt input.
    const TOY: &str = r#"
machine "toy" {
  osc master = 1000000 Hz

  space cpubus { width = 16, unassigned = read-as-ones }

  object wram  "test.ram"   { size = 2K }
  object cpu   "test.cpu"   { clock = master / 4, space = cpubus, engine = "interp" }
  object timer "test.timer" { clock = master / 8 }
  object inv   "test.not"   { }

  map cpubus 0x0000 size 0x2000 = mirror(wram)
  map cpubus 0x4000 size 0x0010 = timer.regs

  # The timer drives the inverter, which drives the CPU. Nothing has driven
  # anything yet at realize time, so `inv.out` must come up high on its own.
  wire timer.out -> inv.in
  wire inv.out   -> cpu.irq
}
"#;

    fn toy() -> Machine {
        let options = BuildOptions::new().with_bindings(bindings());
        match build("toy.machine", TOY, &registry(), &options) {
            Ok(m) => m,
            Err(e) => panic!("{e}"),
        }
    }

    /// One millisecond of virtual time.
    fn a_millisecond() -> crate::core::clock::GlobalTime {
        crate::core::clock::GlobalTime::from_nanos(1_000_000)
    }

    /// Read a byte the way a guest would.
    fn peek(machine: &Machine, addr: u64) -> u64 {
        machine
            .space("cpubus")
            .expect("cpubus")
            .read(addr, Width::U8, MemAttrs::DEFAULT)
            .expect("mapped")
    }

    /// Read one of the timer's registers through the bus.
    fn timer_reg(machine: &Machine, index: u64) -> u64 {
        machine
            .space("cpubus")
            .expect("cpubus")
            .read(0x4000 + index * 4, Width::U32, MemAttrs::DEFAULT)
            .expect("mapped")
    }

    // -----------------------------------------------------------------
    // tests
    // -----------------------------------------------------------------

    /// The media dance in one file: a slot named in the description, bytes
    /// bound by the caller, and a region a `map` statement can place.
    const WITH_MEDIA: &str = r#"
machine "handheld" {
  space bus { width = 16, unassigned = read-as-ones }
  object cart "test.cart" { image = "rom" }
  map bus 0x8000 size 4 = cart
}
"#;

    fn build_with_media(source: &str, media: MediaTable) -> Result<Machine> {
        let mut options = BuildOptions::new().with_bindings(bindings());
        options.realize.media = media;
        build("handheld.machine", source, &registry(), &options)
    }

    #[test]
    fn bound_media_reaches_the_device_as_bytes() {
        let image: &[u8] = &[0xde, 0xad, 0xbe, 0xef];
        let machine = build_with_media(WITH_MEDIA, MediaTable::new().with("rom", image))
            .expect("a bound slot");
        // Read back through the bus: the bytes travelled from the caller,
        // through the property system, into a region, into the memory map.
        let space = machine.space("bus").expect("bus");
        for (i, want) in image.iter().enumerate() {
            let got = space
                .read(0x8000 + i as u64, Width::U8, MemAttrs::DEFAULT)
                .expect("mapped");
            assert_eq!(got, u64::from(*want), "byte {i}");
        }
    }

    #[test]
    fn an_unbound_media_slot_names_itself_and_what_is_bound() {
        // The failure a user actually hits: they forgot `--cart`. The message
        // has to name the slot the file asked for *and* what was bound, or the
        // reader cannot tell a typo from an omission.
        let e = build_with_media(WITH_MEDIA, MediaTable::new())
            .expect_err("nothing bound")
            .to_string();
        assert!(e.contains("rom"), "{e}");
        assert!(e.contains("cart"), "{e}");

        let e = build_with_media(WITH_MEDIA, MediaTable::new().with("disk", &[0u8][..]))
            .expect_err("the wrong slot")
            .to_string();
        assert!(e.contains("`disk`"), "{e}");
    }

    #[test]
    fn a_media_property_that_is_not_a_slot_name_is_refused() {
        let source = r#"
machine "handheld" {
  space bus { width = 16, unassigned = read-as-ones }
  object cart "test.cart" { image = 4096 }
}
"#;
        let e = build_with_media(source, MediaTable::new().with("rom", &[0u8][..]))
            .expect_err("a number is not a slot")
            .to_string();
        assert!(e.contains("image"), "{e}");
    }

    #[test]
    fn media_bound_directly_as_a_value_skips_the_slot_dance() {
        // An embedder assembling `Props` itself — a wasm host, a test — puts
        // the bytes straight in, and realize leaves them alone.
        let props = Props::new().with("image", Media::new("inline", &[1u8, 2, 3][..]));
        let cart = Cart::new(&props).expect("bytes are bytes");
        assert_eq!(cart.region("").expect("a region").len(), 3);
    }

    #[test]
    fn a_class_with_no_media_property_clones_nothing() {
        // The common path: every object in the toy machine has no media, so
        // the substitution pass must not touch their properties at all. Proven
        // by the toy machine building with an empty media table, which it
        // would not if an absent slot were an error.
        let machine = toy();
        assert_eq!(machine.devices().len(), 4);
    }

    #[test]
    fn a_machine_is_built_from_source_text() {
        let machine = toy();
        assert_eq!(machine.name(), "toy");
        assert_eq!(machine.spaces().len(), 1);
        assert_eq!(machine.devices().len(), 4);
        // Instance paths are the names the file gave, in declaration order —
        // and they are the snapshot chunk keys.
        let paths: Vec<&str> = machine.devices().iter().map(DeviceEntry::path).collect();
        assert_eq!(paths, ["wram", "cpu", "timer", "inv"]);
        assert!(machine.shape().devices().contains_key("wram"));
    }

    #[test]
    fn regions_are_mapped_where_the_file_says() {
        let machine = toy();
        let space = machine.space("cpubus").expect("cpubus");
        space
            .write(0x0003, Width::U8, 0xa5, MemAttrs::DEFAULT)
            .expect("ram is writable");
        // 2 KiB mirrored four times across $0000-$1FFF.
        for base in [0x0000u64, 0x0800, 0x1000, 0x1800] {
            assert_eq!(peek(&machine, base + 3), 0xa5, "mirror at {base:#06x}");
        }
        // And the MMIO aperture is a different region entirely.
        assert_eq!(timer_reg(&machine, 0), 0, "no events have fired");
    }

    #[test]
    fn the_realize_sweep_brings_an_idle_inverter_up_high() {
        // The bug §4.3 exists to prevent: nothing has driven anything, so an
        // undriven net sits low — but an inverter's output is high while its
        // input is low, and a machine that skips the sweep comes up with the
        // CPU's interrupt line wrong.
        let machine = toy();
        let nets = machine.nets();
        assert_eq!(nets.len(), 2, "timer→inv and inv→cpu are two nets");
        assert_eq!(nets[0].wire().resolve(Resolve::Or), Level::Low);
        assert_eq!(
            nets[1].wire().resolve(Resolve::Or),
            Level::High,
            "the inverter must have announced its idle level"
        );
    }

    #[test]
    fn a_cpu_runs_against_its_own_address_space() {
        let mut machine = toy();
        machine.run_for(a_millisecond()).expect("the machine runs");

        // The CPU wrote its cycle count into RAM through the bus, so a
        // non-zero value here proves the whole path: budget → run → space →
        // region → store.
        let cycles = machine
            .space("cpubus")
            .expect("cpubus")
            .read(0x0100, Width::U32, MemAttrs::DEFAULT)
            .expect("ram");
        // 1 ms at master/4 = 250 kHz is 250 ticks.
        assert_eq!(cycles, 250);
        // And it saw the interrupt line the sweep left asserted.
        assert_eq!(peek(&machine, 0x0104), 1);
    }

    #[test]
    fn domains_sharing_a_crystal_keep_an_exact_ratio() {
        // §4.2's whole claim, at machine level: master/4 and master/8 are 2:1
        // forever, and the timer's counter advances even though nothing drives
        // it, because it descends from the same crystal.
        let mut machine = toy();
        machine.run_for(a_millisecond()).expect("runs");
        let cpu = machine.device("cpu").and_then(DeviceEntry::domain).unwrap();
        let timer = machine
            .device("timer")
            .and_then(DeviceEntry::domain)
            .unwrap();
        let cpu_ticks = machine.clocks().ticks(cpu).unwrap();
        let timer_ticks = machine.clocks().ticks(timer).unwrap();
        assert_eq!(cpu_ticks, 250);
        assert_eq!(timer_ticks, 125);
        assert_eq!(cpu_ticks, timer_ticks * 2);
    }

    #[test]
    fn an_event_reaches_its_device_and_the_deferred_queue_drains() {
        let mut machine = toy();
        machine
            .schedule_after_ticks("timer", 10, 0x2a)
            .expect("the timer has a clock domain");
        machine.run_for(a_millisecond()).expect("runs");

        assert_eq!(timer_reg(&machine, 0), 1, "the event fired once");
        assert_eq!(timer_reg(&machine, 1), 1, "the deferred action ran");
        assert_eq!(timer_reg(&machine, 2), 1, "the output went high");
        assert_eq!(timer_reg(&machine, 3), 0x2a, "the token came back");

        // The inverter followed its input down, and the CPU saw it.
        assert_eq!(
            machine.nets()[1].wire().resolve(Resolve::Or),
            Level::Low,
            "the inverter must drop its output when its input rises"
        );
    }

    #[test]
    fn an_event_for_an_unknown_device_is_refused() {
        let mut machine = toy();
        let e = machine
            .schedule_after_ticks("nosuch", 1, 0)
            .expect_err("no such device");
        assert!(e.to_string().contains("nosuch"), "{e}");
        // `inv` has no clock, so it cannot have a deadline either.
        let e = machine
            .schedule_after_ticks("inv", 1, 0)
            .expect_err("no clock");
        assert!(e.to_string().contains("clock domain"), "{e}");
    }

    #[test]
    fn a_snapshot_round_trips_to_an_identical_state_hash() {
        let mut machine = toy();
        machine.schedule_after_ticks("timer", 10, 7).unwrap();
        machine.run_for(a_millisecond()).expect("runs");

        let saved = machine.save().expect("saves");
        let hash = machine.state_hash().expect("hashes");

        // Into a second machine built from the same description, which is the
        // case that matters: a save state is loaded by a fresh process.
        let mut restored = toy();
        assert_ne!(
            restored.state_hash().expect("hashes"),
            hash,
            "a fresh machine must not already match a run one"
        );
        restored.load(&saved).expect("loads");
        assert_eq!(restored.state_hash().expect("hashes"), hash);
        assert_eq!(restored.save().expect("saves"), saved);

        // The state is really there, not just its hash: the timer's event
        // count, and the RAM the cpu wrote through the bus.
        assert_eq!(timer_reg(&restored, 0), 1);
        assert_eq!(
            restored
                .space("cpubus")
                .unwrap()
                .read(0x0100, Width::U32, MemAttrs::DEFAULT)
                .unwrap(),
            250
        );
    }

    #[test]
    fn a_snapshot_carries_the_clock_and_the_wires() {
        let mut machine = toy();
        machine.run_for(a_millisecond()).expect("runs");
        let saved = machine.save().expect("saves");

        let reader = StateReader::new(&saved).expect("well formed");
        let migrations = Migrations::new();
        let clocks = reader
            .load(
                crate::machine::machine::CLOCK_PATH,
                crate::machine::machine::CLOCK_CLASS,
                crate::machine::machine::MACHINE_STATE_VERSION,
                &migrations,
            )
            .expect("a clock chunk");
        let mut r = clocks.reader();
        assert_eq!(r.read_seq_len(8).expect("count"), 1, "one crystal");
        assert_eq!(r.read_u64().expect("units"), 1_000, "1 ms of master ticks");

        assert!(reader.find(crate::machine::machine::WIRE_PATH).is_some());
        assert!(reader.find(crate::machine::machine::SCHED_PATH).is_some());
    }

    #[test]
    fn a_snapshot_carries_the_scheduler_and_the_events_it_was_holding() {
        // §4.5: the scheduler is architectural state. Before it was in the
        // snapshot, a restored machine came back at virtual time zero with an
        // empty queue, and a pending timer simply never fired again.
        let mut machine = toy();
        // The timer runs at 125 kHz, so this is 8 ms out — still pending after
        // the millisecond below, which is the case that matters.
        machine.schedule_after_ticks("timer", 1_000, 7).unwrap();
        machine.run_for(a_millisecond()).expect("runs");
        assert_eq!(timer_reg(&machine, 0), 0, "not due yet");

        let saved = machine.save().expect("saves");
        let now = machine.now();
        assert!(now > crate::core::clock::GlobalTime::ZERO);

        let mut restored = toy();
        restored.load(&saved).expect("loads");
        assert_eq!(
            restored.now(),
            now,
            "virtual time is state, not a fresh start"
        );

        // Run both machines past the deadline. The restored one fires the event
        // it was saved holding, at the same instant, and ends in the same state.
        let deadline = crate::core::clock::GlobalTime::from_nanos(20_000_000);
        machine.run_until(deadline).expect("runs");
        restored.run_until(deadline).expect("runs");
        assert_eq!(timer_reg(&machine, 0), 1, "the saved machine fired it");
        assert_eq!(timer_reg(&restored, 0), 1, "and so did the restored one");
        assert_eq!(
            restored.state_hash().expect("hashes"),
            machine.state_hash().expect("hashes")
        );
    }

    #[test]
    fn a_snapshot_from_another_machine_is_a_diff_not_a_crash() {
        let machine = toy();
        let saved = machine.save().expect("saves");
        let options = BuildOptions::new().with_bindings(bindings());
        let mut small = build(
            "small.machine",
            "machine \"toy\" { object wram \"test.ram\" { size = 1K } }",
            &registry(),
            &options,
        )
        .expect("builds");
        let e = small.load(&saved).expect_err("different shape");
        assert!(e.to_string().contains("cpu"), "{e}");
    }

    #[test]
    fn reset_returns_a_run_machine_to_its_cold_state() {
        let mut machine = toy();
        let cold = machine.state_hash().expect("hashes");
        machine.schedule_after_ticks("timer", 10, 7).unwrap();
        machine.run_for(a_millisecond()).expect("runs");
        assert_ne!(machine.state_hash().expect("hashes"), cold);

        machine.reset(ResetKind::Cold);
        // Virtual time is not device state and keeps running across a reset,
        // so the comparison is over the devices and the wires.
        assert_eq!(timer_reg(&machine, 0), 0);
        assert_eq!(peek(&machine, 0x0100), 0, "cold reset clears memory");
        assert_eq!(
            machine.nets()[1].wire().resolve(Resolve::Or),
            Level::High,
            "the sweep runs after every reset, not only the first"
        );
    }

    #[test]
    fn a_warm_reset_leaves_memory_alone() {
        let mut machine = toy();
        machine
            .space("cpubus")
            .unwrap()
            .write(0x0010, Width::U8, 0x5a, MemAttrs::DEFAULT)
            .unwrap();
        machine.reset(ResetKind::Warm);
        assert_eq!(peek(&machine, 0x0010), 0x5a);
        machine.reset(ResetKind::Cold);
        assert_eq!(peek(&machine, 0x0010), 0x00);
    }

    #[test]
    fn two_sources_on_one_pin_are_a_wired_or() {
        // §4.3's classic bug: the APU deasserts IRQ while the cartridge still
        // asserts it. A sink that only knows "someone said low" drops a line
        // that must stay high.
        const TWO: &str = r#"
machine "two" {
  osc master = 1000000 Hz
  space cpubus { width = 16 }
  object cpu "test.cpu"   { clock = master / 4, space = cpubus }
  object a   "test.timer" { clock = master / 4 }
  object b   "test.timer" { clock = master / 4 }
  wire a.out -> cpu.irq
  wire b.out -> cpu.irq
}
"#;
        let options = BuildOptions::new().with_bindings(bindings());
        let machine = build("two.machine", TWO, &registry(), &options).expect("builds");
        assert_eq!(machine.nets().len(), 1, "one pin, one net, two drivers");
        let net = &machine.nets()[0];
        assert_eq!(net.sources().len(), 2);
        let (a, b) = (net.sources()[0].id, net.sources()[1].id);
        let wire = net.wire();

        wire.set(a, Level::High);
        assert_eq!(wire.resolve(Resolve::Or), Level::High);
        wire.set(b, Level::High);
        wire.set(a, Level::Low);
        assert_eq!(
            wire.resolve(Resolve::Or),
            Level::High,
            "b is still asserting"
        );
        wire.set(b, Level::Low);
        assert_eq!(wire.resolve(Resolve::Or), Level::Low);
    }

    #[test]
    fn a_combinational_wire_cycle_is_refused_by_realize() {
        // The validator rejects this too, but realize must not depend on
        // having been validated: the sweep needs an order and there is none.
        const RING: &str = r#"
machine "ring" {
  object x "test.not" { }
  object y "test.not" { }
  wire x.out -> y.in
  wire y.out -> x.in
}
"#;
        let options = BuildOptions::new().with_bindings(bindings());
        let e = build("ring.machine", RING, &registry(), &options).expect_err("a ring");
        assert!(e.to_string().contains("cycle"), "{e}");
    }

    #[test]
    fn a_class_the_registry_does_not_have_is_named() {
        const M: &str = r#"machine "m" { object d "test.nope" { } }"#;
        let options = BuildOptions::new().with_bindings(bindings());
        let e = build("m.machine", M, &registry(), &options).expect_err("unknown class");
        let text = e.to_string();
        assert!(text.contains("test.nope"), "{text}");
        assert!(text.contains("feature"), "{text}");
    }

    #[test]
    fn an_unbound_class_cannot_be_mapped() {
        // Constructed, reset and snapshotted — but it publishes no regions, so
        // naming one is an error that says exactly that rather than a silent
        // empty mapping.
        const M: &str = r#"
machine "m" {
  space s { width = 16 }
  object w "test.witness" { }
  map s 0 size 16 = w
}
"#;
        let options = BuildOptions::new(); // no bindings at all
        let e = build("m.machine", M, &registry(), &options).expect_err("unbound");
        assert!(e.to_string().contains("publishes no regions"), "{e}");
    }

    #[test]
    fn a_failed_realize_unrealizes_what_it_already_did() {
        const M: &str = r#"
machine "m" {
  object w "test.witness" { }
  object boom "test.explode" { }
}
"#;
        let before = UNREALIZED.load(Relaxed);
        let options = BuildOptions::new().with_bindings(bindings());
        let e = build("m.machine", M, &registry(), &options).expect_err("explodes");
        assert!(e.to_string().contains("always fails"), "{e}");
        assert!(
            UNREALIZED.load(Relaxed) > before,
            "the witness must have been unrealized"
        );
    }

    #[test]
    fn a_mapping_priority_decides_an_overlap() {
        // PCI BAR over RAM, boot-ROM shadowing, a cartridge window: §4.1's
        // overlap rule, spelled in the machine file.
        const M: &str = r#"
machine "m" {
  space s { width = 16 }
  object low  "test.ram" { size = 0x200 }
  object high "test.ram" { size = 0x100 }
  map s 0x0000 size 0x200 = low
  map s 0x0000 size 0x100 = high { priority = 1 }
  map s 0x1000 size 0x100 = high
}
"#;
        let options = BuildOptions::new().with_bindings(bindings());
        let machine = build("m.machine", M, &registry(), &options).expect("builds");
        let space = machine.space("s").expect("s");
        space
            .write(0x0000, Width::U8, 0xaa, MemAttrs::DEFAULT)
            .expect("writable");
        // 0x1000 is `high` and nothing else, so this says which store the
        // write landed in.
        assert_eq!(
            space
                .read(0x1000, Width::U8, MemAttrs::DEFAULT)
                .expect("mapped"),
            0xaa,
            "the higher priority mapping must win the overlap"
        );
        // And past `high`'s 0x100 bytes, `low` is still there.
        space
            .write(0x0150, Width::U8, 0x55, MemAttrs::DEFAULT)
            .expect("writable");
        assert_eq!(
            space
                .read(0x0150, Width::U8, MemAttrs::DEFAULT)
                .expect("mapped"),
            0x55
        );
    }

    #[test]
    fn a_mapping_may_reverse_the_byte_order_it_is_seen_through() {
        // A big-endian device on a little-endian bus is normal, not exotic
        // (§4.1) — and the same bytes seen through two windows is the test.
        const M: &str = r#"
machine "m" {
  space s { width = 16 }
  object r "test.ram" { size = 0x100 }
  map s 0x0000 size 0x100 = r
  map s 0x1000 size 0x100 = r { endian = "big" }
}
"#;
        let options = BuildOptions::new().with_bindings(bindings());
        let machine = build("m.machine", M, &registry(), &options).expect("builds");
        let space = machine.space("s").expect("s");
        space
            .write(0x0000, Width::U16, 0x1234, MemAttrs::DEFAULT)
            .expect("writable");
        assert_eq!(
            space
                .read(0x1000, Width::U16, MemAttrs::DEFAULT)
                .expect("mapped"),
            0x3412,
            "the same two bytes, read the other way round"
        );
    }

    #[test]
    fn an_unknown_mapping_attribute_says_what_is_accepted() {
        const M: &str = r#"
machine "m" {
  space s { width = 16 }
  object r "test.ram" { size = 0x100 }
  map s 0 size 0x100 = r { prority = 1 }
}
"#;
        let options = BuildOptions::new().with_bindings(bindings());
        let e = build("m.machine", M, &registry(), &options).expect_err("typo");
        let text = e.to_string();
        assert!(text.contains("prority"), "{text}");
        assert!(text.contains("priority"), "{text}");
    }

    #[test]
    fn a_mapping_larger_than_its_region_says_to_mirror() {
        const M: &str = r#"
machine "m" {
  space s { width = 16 }
  object r "test.ram" { size = 1K }
  map s 0 size 0x2000 = r
}
"#;
        let options = BuildOptions::new().with_bindings(bindings());
        let e = build("m.machine", M, &registry(), &options).expect_err("too big");
        assert!(e.to_string().contains("mirror()"), "{e}");
    }

    #[test]
    fn a_clock_may_hang_off_another_objects_domain() {
        const M: &str = r#"
machine "m" {
  osc master = 1000000 Hz
  space s { width = 16 }
  object cpu "test.cpu" { clock = master / 4, space = s }
  object t   "test.timer" { clock = cpu / 2 }
}
"#;
        let options = BuildOptions::new().with_bindings(bindings());
        let mut machine = build("m.machine", M, &registry(), &options).expect("builds");
        machine.run_for(a_millisecond()).expect("runs");
        let cpu = machine.device("cpu").and_then(DeviceEntry::domain).unwrap();
        let t = machine.device("t").and_then(DeviceEntry::domain).unwrap();
        assert_eq!(machine.clocks().ticks(cpu).unwrap(), 250);
        assert_eq!(machine.clocks().ticks(t).unwrap(), 125);
    }

    #[test]
    fn a_cpu_with_no_address_space_is_refused_at_bind() {
        const M: &str = r#"
machine "m" {
  osc master = 1000000 Hz
  object cpu "test.cpu" { clock = master / 4 }
}
"#;
        let options = BuildOptions::new().with_bindings(bindings());
        let e = build("m.machine", M, &registry(), &options).expect_err("no space");
        assert!(e.to_string().contains("address space"), "{e}");
    }

    #[test]
    fn realize_takes_a_bare_registry() {
        // The signature the CLI reaches for: no bindings, so nothing maps and
        // nothing wires, but the devices are constructed, reset and
        // snapshotted like any others.
        const M: &str = r#"
machine "bare" {
  object w "test.witness" { }
  object x "test.witness" { }
}
"#;
        let mut map = crate::machine::SourceMap::new();
        let root = map.add("bare.machine", M).expect("fits");
        let resolved = crate::machine::resolve(
            &mut map,
            root,
            &mut crate::machine::sources::NoIncludes,
            &crate::machine::ResolveOptions::new(),
        )
        .expect("resolves");
        let machine = realize(&resolved, &registry()).expect("realizes");
        assert_eq!(machine.devices().len(), 2);
        assert!(machine.devices().iter().all(|d| d.instance().is_none()));
        assert!(machine.nets().is_empty());
        assert!(!machine.save().expect("saves").is_empty());
    }

    #[test]
    fn a_scheduler_that_cannot_advance_is_reported_rather_than_hanging() {
        let mut map = crate::machine::SourceMap::new();
        let root = map.add("toy.machine", TOY).expect("fits");
        let resolved = crate::machine::resolve(
            &mut map,
            root,
            &mut crate::machine::sources::NoIncludes,
            &crate::machine::ResolveOptions::new(),
        )
        .expect("resolves");
        let mut options = RealizeOptions::new();
        options.scheduler.quantum = crate::core::clock::GlobalTime::ZERO;
        let mut machine = realize_with(&resolved, &registry(), &bindings(), &options)
            .expect("a zero quantum is legal to build");

        // One quantum is a no-op rather than an error: it is the run *loop*
        // that would spin.
        let report = machine.run_quantum().expect("a quantum runs");
        assert_eq!(report.from, report.to);
        let e = machine
            .run_for(a_millisecond())
            .expect_err("cannot advance");
        assert!(e.to_string().contains("quantum is zero"), "{e}");
    }

    #[test]
    fn bindings_refuse_a_duplicate_class() {
        let mut b = Bindings::new();
        b.bind("test.ram", make_ram).expect("first");
        let e = b.bind("test.ram", make_ram).expect_err("second");
        assert!(e.to_string().contains("twice"), "{e}");
        assert_eq!(b.len(), 1);
        assert!(!b.is_empty());
        assert_eq!(b.classes().collect::<Vec<_>>(), ["test.ram"]);
    }
}
