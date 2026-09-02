//! An accelerated **`cpu.x86` device**: a board's processor, running on the
//! host's own silicon.
//!
//! `ROADMAP.md` phase 7's gate opens *"the phase-6 machines boot under KVM
//! **with ≥ 2 vCPUs**"*, and [`kvm`](super::kvm) and [`board`] between them
//! already had everything except the thing that made it reachable from a
//! machine file: **a processor is a device**. A vector reaches a guest only
//! through an acknowledge cycle on [`Device::attach_int_ack`], and only a CPU
//! device sits on the far end of `wire lapic0.intr -> cpu0.intr`; `INIT` and
//! Start-Up arrive through [`LocalController::take_startup`], and only a CPU
//! device is offered one. A vCPU created beside a board is a harness; a vCPU
//! that *is* `cpu0` is the board.
//!
//! # How it gets in without editing `src/cpu/x86/`
//!
//! `engine` is validated in two places in `cpu::x86` — its property reader and
//! its validator schema — and both accept only `"interp"`. So this class is
//! **not** a new name and **not** a new property: it is
//! [`Bindings::replace`](crate::machine::Bindings::replace)`("cpu.x86", …)`,
//! the supported route for a host to construct something else for a class the
//! machine file already names. `machines/pc-apic.machine` is used verbatim,
//! `engine = "interp"` and all, and what changes is the engine underneath it.
//!
//! That is a limit worth stating rather than hiding: **a machine file cannot
//! yet say `engine = "kvm"`**, so choosing acceleration is a decision the
//! *host* makes, not the board. Closing that is four lines in `cpu::x86` —
//! `"kvm"` added to the reader's `or_enum` list and to `schema_for`'s `values`
//! — and it is not this module's file to edit.
//!
//! # The shape: an interpreter shell with a hypervisor underneath
//!
//! [`AccelCpu`] owns a real [`X86`], and that is the design rather than an
//! implementation detail. `cpu.x86` is eighteen properties, five input pins,
//! the [`IntAck`] and [`LocalController`] seams and a snapshot chunk at
//! **version 7**; a second implementation of all of that would be a second
//! thing to keep in step, and the first field to drift would be the one the
//! cross-engine half of the gate depends on. So the shell is the device, and
//! this type is the *engine*:
//!
//! | what | who answers |
//! | --- | --- |
//! | properties, class, snapshot chunk | the shell, verbatim |
//! | `intr`, `reset`, `init`, `a20` pins | the shell's own pins |
//! | the acknowledge cycle | the shell's `IntAck` list |
//! | `RESET`, `INIT`, Start-Up sequences | the shell **executes** them |
//! | every guest instruction | the vCPU |
//! | `nmi` | this module, because the shell's latch has no public taker |
//!
//! Running the restart sequences on the interpreter is not a shortcut. They
//! are exactly the states in which a processor executes *no* guest
//! instructions, `cpu::x86` already implements them from *Intel SDM* Vol 3A
//! Table 9-1 and the *MultiProcessor Specification* §B.4, and both engines
//! reading one implementation is the whole reason §4.6 asks them to agree
//! instruction-for-instruction. The result is pushed into the vCPU with
//! [`state::load_into_vcpu`] and the guest continues in hardware.
//!
//! The two directions are kept honest by one invariant: **the shell is always
//! current except when [`dirty`](AccelCpu) says the shell is ahead.** Every
//! hardware slice ends with [`state::store_from_vcpu`], so `save`,
//! `debug_translate`, a monitor read and [`state::differs`] all see the guest
//! as it is without this module doing anything special at those call sites.
//!
//! # Never reads the wall clock, never sleeps, never spawns a thread
//!
//! `CLAUDE.md` says a device does none of those, and a hypervisor client is
//! the hardest case for it — the usual KVM run loop is "block in `KVM_RUN`
//! until a host timer fires". This one does not have a host timer and does not
//! want one:
//!
//! * **No clock.** Nothing here calls a clock of any kind. Virtual time comes
//!   from the budget the scheduler hands `run`, and the budget is spent
//!   whatever the host took, exactly as [`VcpuRunnable`](super::kvm::VcpuRunnable)
//!   already does — the honest statement being that an accelerated guest's
//!   progress is *not* measured in this clock domain, so pretending to measure
//!   it would be worse than admitting it.
//! * **No sleep.** A halted guest is *parked*: `run` returns having consumed
//!   its budget and having entered nothing. A `HLT` under a userspace
//!   interrupt controller comes back as `KVM_EXIT_HLT` and the processor stays
//!   out of hardware until the board's own `INTR` is asserted **and** the
//!   guest can take it, which is the same predicate the interpreter uses.
//! * **No thread.** The vCPU is entered on whichever thread the scheduler's
//!   pool gave this runnable. Nothing here creates one.
//! * **No signal.** The stop protocol is [`kvm`](super::kvm)'s: a per-CPU exit
//!   flag written through to `KVM_CAP_IMMEDIATE_EXIT`. What that gives up —
//!   *a guest taking no exits is not preemptible* — is inherited unchanged,
//!   and on a board it has a sharper edge than the module documentation there
//!   suggests: under [`ThreadingMode::Parallel`] the scheduler's round does not
//!   end until every runnable returns, so a guest spinning with interrupts
//!   masked and no memory-mapped access stops the *machine's* virtual time,
//!   not only its own. [`ThreadingMode::Accel`] is the answer §4.2 designed and
//!   the scheduler does not implement it yet.
//!
//! # Determinism
//!
//! An accelerated run is not reproducible, and [`AccelCpus::open`] refuses a
//! [`ThreadingMode`] that claims it is — the same structural refusal
//! [`Vcpu::into_runnable`](super::kvm::Vcpu::into_runnable) and
//! `Machine::set_recorder` make, moved to the point where the decision is
//! actually taken. A board built through these bindings therefore *cannot* be
//! in `deterministic` threading, and `Machine::state_hash` on it refuses for
//! its own reasons without this module arranging anything.
//!
//! # What does not reach hardware yet, named rather than discovered
//!
//! * **The A20 gate.** It is an input pin on the shell and a mask on the
//!   shell's own accesses; the guest's hardware accesses are not masked. A
//!   board that closes the gate and relies on the megabyte wrap will not see
//!   it under acceleration. The gate is a chipset AND gate that rsemu models
//!   inside the core for want of anything between an initiator and its space,
//!   and a hypervisor has no equivalent place to put it.
//! * **A pending `NMI` and the `NMI` level are not in the snapshot.** The
//!   shell's edge latch can be set from outside but not *taken*, so this
//!   module keeps its own latch and delivers through `KVM_NMI`; the chunk
//!   consequently records `nmi_level = false, nmi_latch = false`. Making
//!   `Lines::take_nmi_pending` public on `X86` is the one-line change that
//!   would let the shell own the pin outright.
//! * **`halted` is not in the snapshot either.** `KVM_EXIT_HLT` leaves `RIP`
//!   past the `HLT` and there is no public setter for the shell's flag, so a
//!   snapshot taken while an accelerated processor idles restores into an
//!   interpreter that *resumes* rather than waits. `pub fn set_halted` closes
//!   it.
//! * **`INIT` held by this processor's own local controller** is tracked here
//!   and not mirrored into the shell's `init_peer`, for the same reason: no
//!   public setter. The `INIT` **pin**'s level, which is the one a wire drives,
//!   is the shell's and is saved.
//! * **The time-stamp counter**, as [`state`] already says.
//!
//! [`ThreadingMode::Parallel`]: crate::core::sched::ThreadingMode::Parallel
//! [`ThreadingMode::Accel`]: crate::core::sched::ThreadingMode::Accel

use alloc::string::{String, ToString};
use alloc::sync::{Arc, Weak};
use alloc::vec::Vec;

use crate::core::device::{
    DebugTranslation, Device, DeviceClass, Export, ExportId, Initiator, RealizeCtx, ResetKind,
    SinkPin,
};
use crate::core::error::{Error, Result};
use crate::core::exec::ExitReason;
use crate::core::props::Props;
use crate::core::sched::{Budget, Consumed, ThreadingMode};
use crate::core::space::{AddressSpace, RequesterId};
use crate::core::state::{ChunkReader, ChunkWriter};
use crate::core::sync::{AtomicBool, AtomicU64, LockRank, Mutex, Ordering};
use crate::core::wire::{
    DmaPeripheral, FanIn, IntAck, Level, LocalController, Resolve, WireId, WireSink, WireSource,
};
use crate::cpu::x86::{Variant, X86};
use crate::machine::{BindCtx, Bindings, Instance};

use super::board::{self, Plan};
use super::kvm::{IntrSource, Kvm, Vcpu, Vm};
use super::state;
use super::{AccelError, AccelResult};

/// How many guest entries one call to [`Device::run`] makes at most.
///
/// A bound on *exits*, not on instructions: between two of them the guest runs
/// at full speed for as long as it likes. It exists so that a board whose
/// devices need servicing — a guest hammering a port — still hands control
/// back to the scheduler within one round rather than starving every other
/// runnable, and it is deliberately generous, because the common case is a
/// guest that runs for milliseconds between exits and the cost of coming back
/// out is a full architectural-state read.
const MAX_ENTRIES: u64 = 4096;

/// How many restart sequences one call to [`Device::run`] executes before it
/// gives the scheduler its turn back.
///
/// `RESET`, `INIT` and Start-Up together are three; anything past that is a
/// board driving a pin every instant, and letting the loop run away would be
/// worse than being late.
const MAX_SEQUENCES: u32 = 8;

/// The rank of [`AccelCpus::installing`], one step outside
/// [`LockRank::MACHINE`].
///
/// A rank rather than [`LockRank::UNCHECKED`] because the order is real and
/// worth asserting: installing a board's memory map calls into
/// [`Vm::set_region`](super::kvm::Vm::set_region), whose slot table is itself
/// at `MACHINE`, and into the address space's topology lock below that. The
/// named ranks are spaced by `0x1000` precisely so one can be inserted
/// (`core::sync`).
const INSTALL_RANK: LockRank = LockRank::new(0x0800);

// ---------------------------------------------------------------------------
// the host object
// ---------------------------------------------------------------------------

/// The virtual machine a board's accelerated processors share, and the
/// constructor that puts them on it.
///
/// One [`Vm`] per machine, one [`Vcpu`] per `cpu.x86` object, allocated in the
/// order the machine file declares them — so `cpu0` is vCPU 0, which is the
/// processor a board's local APIC calls the bootstrap processor. That
/// correspondence is by construction order rather than by name, which is worth
/// knowing before writing a machine file that declares its processors
/// backwards.
///
/// It is a *host object* in the sense `ROADMAP.md` §4.4 uses: the thing a
/// front end builds before the machine and keeps afterwards. Nothing in
/// `core/` or `machine/` knows it exists.
#[derive(Debug)]
pub struct AccelCpus {
    kvm: Kvm,
    vm: Vm,
    mode: ThreadingMode,
    /// The topology generation the memory slots were installed from, or zero
    /// if they never were. An `AddressSpace` generation starts at one, so zero
    /// cannot collide with a real one.
    mapped: AtomicU64,
    /// What [`board::install_space`] last did, for a monitor or a test that
    /// wants to say which of a board's regions run in hardware.
    plan: Mutex<Option<Plan>>,
    /// Guards the install itself, so two processors binding at once do not
    /// both rewrite the slot table.
    ///
    /// Ranked at [`INSTALL_RANK`], *outside* [`LockRank::MACHINE`], because
    /// [`Vm`]'s own slot table is at `MACHINE` and this is held across it —
    /// the ladder runs in the direction calls travel and this call travels
    /// into the VM.
    installing: Mutex<()>,
    /// Every processor built through this table, weakly: the machine owns its
    /// devices, and a host object that kept them alive would outlive the
    /// machine that was supposed to own them (§4.3's weak edge, one level up).
    built: Mutex<Vec<Weak<AccelCpu>>>,
}

impl AccelCpus {
    /// Open `/dev/kvm`, create a VM, and prepare to build processors on it.
    ///
    /// `mode` is the threading mode the machine will be realized in, and it is
    /// checked here rather than trusted: hardware execution is not
    /// reproducible, so a mode that claims reproducibility is refused outright.
    ///
    /// # Errors
    ///
    /// [`AccelError::Unavailable`] if this host has no usable `/dev/kvm` —
    /// which a front end should treat as a reason to fall back to the
    /// interpreter, not as a failure — [`AccelError::Nondeterministic`] for a
    /// mode that claims reproducibility, and [`AccelError::Sys`] if the VM
    /// cannot be created.
    pub fn open(mode: ThreadingMode) -> AccelResult<Arc<AccelCpus>> {
        if mode.is_deterministic() {
            return Err(AccelError::Nondeterministic(mode));
        }
        let kvm = Kvm::open()?;
        let vm = kvm.create_vm()?;
        Ok(Arc::new(AccelCpus {
            kvm,
            vm,
            mode,
            mapped: AtomicU64::new(0),
            plan: Mutex::new(None),
            installing: Mutex::with_rank(INSTALL_RANK, ()),
            built: Mutex::new(Vec::new()),
        }))
    }

    /// Take `cpu.x86` over in `bindings`.
    ///
    /// After this, every `object … "cpu.x86"` in the machine file is built as
    /// an [`AccelCpu`] instead of an interpreter, with the same properties and
    /// the same class — [`Bindings::replace`] is the sanctioned interception
    /// and this is what it is for.
    ///
    /// `cpu.i8086` is deliberately **not** taken over: a 16-bit part is a
    /// different class, an 8088 is not the host's instruction set in any
    /// meaningful sense, and a board that asks for one means it.
    pub fn install(self: &Arc<Self>, bindings: &mut Bindings) {
        let host = Arc::clone(self);
        bindings.replace("cpu.x86", move |props: &Props| {
            let cpu = host.construct(props)?;
            Ok(cpu as Arc<dyn Instance>)
        });
    }

    /// Build one processor. The constructor `install` hands to [`Bindings`].
    fn construct(self: &Arc<Self>, props: &Props) -> Result<Arc<AccelCpu>> {
        let shell = X86::from_props_defaulting(props, Variant::I80486)?;
        let mut built = self.built.lock();
        let id = u32::try_from(built.len()).unwrap_or(u32::MAX);
        let cpu = Arc::new(AccelCpu {
            shell,
            id,
            host: Arc::clone(self),
            vcpu: Mutex::new(None),
            memory: Mutex::new(None),
            intc: Mutex::new(None),
            has_intc: AtomicBool::new(false),
            init_peer: AtomicBool::new(false),
            nmi: Arc::new(NmiLatch::default()),
            pins: Mutex::new(Vec::new()),
            dirty: AtomicBool::new(true),
            halted: AtomicBool::new(false),
            stopped: AtomicBool::new(false),
            entries: AtomicU64::new(0),
            failure: Mutex::new(None),
        });
        built.push(Arc::downgrade(&cpu));
        Ok(cpu)
    }

    /// The virtual machine these processors share.
    #[must_use]
    pub const fn vm(&self) -> &Vm {
        &self.vm
    }

    /// The `/dev/kvm` handle, for a caller that wants to ask about a
    /// capability.
    #[must_use]
    pub const fn kvm(&self) -> &Kvm {
        &self.kvm
    }

    /// The threading mode this table was opened for.
    #[must_use]
    pub const fn threading_mode(&self) -> ThreadingMode {
        self.mode
    }

    /// What the board's memory map became, once a processor has bound.
    #[must_use]
    pub fn plan(&self) -> Option<Plan> {
        self.plan.lock().clone()
    }

    /// Every processor built through this table that is still alive, in
    /// construction order — which is vCPU-id order.
    #[must_use]
    pub fn cpus(&self) -> Vec<Arc<AccelCpu>> {
        self.built.lock().iter().filter_map(Weak::upgrade).collect()
    }

    /// Install `space`'s flat view as memory slots, unless that has already
    /// been done for this topology generation.
    ///
    /// Called by every processor as it binds and again before every hardware
    /// slice, because a board is entitled to change its own memory map while
    /// it runs — a PAM register shadowing ROM into RAM, a PCI BAR moving —
    /// and the generation counter is how `core::space` announces that
    /// (§4.1).
    fn ensure_map(&self, space: &AddressSpace) -> AccelResult<()> {
        let generation = space.generation();
        if self.mapped.load(Ordering::Acquire) == generation {
            return Ok(());
        }
        let _guard = self.installing.lock();
        // Re-checked under the guard: the processor that was ahead of us has
        // done the work by now.
        if self.mapped.load(Ordering::Acquire) == generation {
            return Ok(());
        }
        let plan = board::install_space(&self.vm, space, 0)?;
        self.mapped.store(generation, Ordering::Release);
        drop(_guard);
        *self.plan.lock() = Some(plan);
        Ok(())
    }
}

// ---------------------------------------------------------------------------
// the NMI pin
// ---------------------------------------------------------------------------

/// The `NMI` pin's level and its edge latch.
///
/// This module's rather than the shell's, and the reason is narrow: an NMI
/// must be **taken** exactly once per rising edge, `X86` can be told a level
/// but its latch has no public taker, and a latch nobody can clear delivers
/// the same interrupt for ever. See the module documentation for what that
/// costs in the snapshot and for the one-line change that would end it.
#[derive(Debug, Default)]
struct NmiLatch {
    level: AtomicBool,
    pending: AtomicBool,
}

impl NmiLatch {
    /// Drive the pin, latching a rising edge.
    fn set(&self, asserted: bool) {
        if asserted && !self.level.swap(true, Ordering::AcqRel) {
            self.pending.store(true, Ordering::Release);
        } else if !asserted {
            self.level.store(false, Ordering::Release);
        }
    }

    /// Consume the latch, reporting whether an NMI was owed.
    fn take(&self) -> bool {
        if !self.pending.load(Ordering::Relaxed) {
            return false;
        }
        self.pending.swap(false, Ordering::AcqRel)
    }
}

/// The `NMI` pin, as something a [`Wire`](crate::core::wire::Wire) can drive.
///
/// Wire-ORed like every other pin on this core: a PC's NMI comes from the
/// parity checker *and* the coprocessor, and either releasing must not drop a
/// line the other is holding.
#[derive(Debug)]
struct NmiPin {
    latch: Arc<NmiLatch>,
    inputs: FanIn,
}

impl WireSink for NmiPin {
    fn set_level(&self, src: WireId, _line: u32, level: Level) {
        self.inputs.set(src, level);
        self.latch.set(self.inputs.resolve(Resolve::Or).is_high());
    }
}

// ---------------------------------------------------------------------------
// the processor
// ---------------------------------------------------------------------------

/// One accelerated `cpu.x86`.
///
/// A [`Device`] and a [`machine::Instance`](crate::machine::Instance) in every
/// respect — it reports the same class, accepts the same properties, offers
/// the same pins and writes the same snapshot chunk — whose guest instructions
/// execute on the host.
#[derive(Debug)]
pub struct AccelCpu {
    /// The interpreter this device is *shaped* like, and the holder of every
    /// piece of architectural state that is not currently inside the vCPU.
    shell: X86,
    id: u32,
    host: Arc<AccelCpus>,
    /// Created at [`Instance::bind`], when the address spaces are known.
    ///
    /// Behind an [`Arc`] inside the lock so that a caller clones it out and
    /// releases the lock before entering the guest: the [`Vcpu`]'s own lock is
    /// further down the ladder and holding this one across it would invert
    /// the order.
    vcpu: Mutex<Option<Arc<Vcpu>>>,
    memory: Mutex<Option<Arc<AddressSpace>>>,
    /// This processor's own interrupt controller — its local APIC — kept here
    /// as well as in the shell, because [`take_startup`] must be asked once
    /// per boundary and the shell only asks inside a step this engine does not
    /// take.
    ///
    /// [`take_startup`]: crate::core::wire::LocalController::take_startup
    intc: Mutex<Option<Weak<dyn LocalController>>>,
    has_intc: AtomicBool,
    /// Whether that controller reports `INIT` still asserted, which holds the
    /// processor in reset rather than restarting it.
    init_peer: AtomicBool,
    nmi: Arc<NmiLatch>,
    /// The pins this device owns, because a net holds its sinks weakly.
    pins: Mutex<Vec<Arc<NmiPin>>>,
    /// Whether the shell holds state the vCPU has not been given.
    ///
    /// Set by construction, by a reset, by a restart sequence and by a
    /// snapshot load; cleared by the push into hardware. The opposite
    /// direction needs no flag, because every hardware slice ends by writing
    /// the shell.
    dirty: AtomicBool,
    /// Whether the guest is waiting for an interrupt at a `HLT`.
    halted: AtomicBool,
    /// Whether the processor has stopped for good: a triple fault, or a
    /// backend failure. Cleared only by a reset.
    stopped: AtomicBool,
    /// How many guest entries this processor has made. Diagnostics.
    entries: AtomicU64,
    /// The last backend failure, for a test or a monitor that wants to know
    /// why a processor stopped.
    failure: Mutex<Option<String>>,
}

impl AccelCpu {
    /// This processor's vCPU index, which is also its position in the machine
    /// file's list of `cpu.x86` objects.
    #[must_use]
    pub const fn id(&self) -> u32 {
        self.id
    }

    /// The interpreter shell: the holder of this processor's architectural
    /// state whenever it is not inside the guest.
    ///
    /// Current after every [`Device::run`], so a caller reading registers
    /// between rounds reads what the guest actually has.
    #[must_use]
    pub const fn shell(&self) -> &X86 {
        &self.shell
    }

    /// The vCPU, once [`Instance::bind`] has created one.
    #[must_use]
    pub fn vcpu(&self) -> Option<Arc<Vcpu>> {
        self.vcpu.lock().clone()
    }

    /// How many times this processor has entered the guest.
    #[must_use]
    pub fn entries(&self) -> u64 {
        self.entries.load(Ordering::Relaxed)
    }

    /// Whether the guest is idling at a `HLT`.
    #[must_use]
    pub fn is_halted(&self) -> bool {
        self.halted.load(Ordering::Acquire)
    }

    /// Whether the processor has stopped for good — a triple fault, or a
    /// backend failure. [`failure`](AccelCpu::failure) says which.
    #[must_use]
    pub fn is_stopped(&self) -> bool {
        self.stopped.load(Ordering::Acquire)
    }

    /// The last backend failure, if there was one.
    #[must_use]
    pub fn failure(&self) -> Option<String> {
        self.failure.lock().clone()
    }

    /// This processor's own interrupt controller, if a machine wired one.
    fn controller(&self) -> Option<Arc<dyn LocalController>> {
        if !self.has_intc.load(Ordering::Relaxed) {
            return None;
        }
        let peer = self.intc.lock().clone();
        peer.and_then(|weak| weak.upgrade())
    }

    /// Ask this processor's controller what it has, and fold it into the
    /// shell's latches.
    ///
    /// The same three facts `cpu::x86` folds in once per instruction, asked
    /// once per slice instead — which is the granularity an accelerated
    /// processor has, because between two guest entries it is not at an
    /// instruction boundary this program can see. An `INIT` therefore stops
    /// this processor at the next exit rather than at the next instruction,
    /// and that is the same bound the module documentation gives for a stop
    /// request.
    ///
    /// No lock of ours is held across the call: the controller takes its own,
    /// and it is free to drive this processor's `INTR` pin back while it is
    /// answering (§4.7).
    fn poll_controller(&self) {
        let Some(intc) = self.controller() else {
            return;
        };
        let signal = intc.take_startup();
        if signal.init {
            self.shell.request_init();
        }
        self.init_peer.store(signal.held, Ordering::Release);
        if let Some(page) = signal.page {
            self.shell.start_up(page);
        }
    }

    /// Whether `INIT` is asserted by either of its two drivers.
    fn init_held(&self) -> bool {
        self.init_peer.load(Ordering::Acquire) || self.shell.init_held()
    }

    /// Whether the shell owes a sequence that runs *instead of* an
    /// instruction: a `RESET`, an `INIT`, a Start-Up, or being held in one of
    /// them.
    ///
    /// Every predicate here is non-consuming, which is what makes it safe to
    /// ask before deciding to step: a step taken when this is false would run
    /// a guest instruction on the interpreter, and the guest is not the
    /// interpreter's to run.
    fn sequence_owed(&self) -> bool {
        self.shell.reset_requested()
            || self.shell.reset_pending()
            || self.shell.init_requested()
            || self.shell.is_waiting_for_startup()
            || self.init_held()
    }

    /// Run whatever restart sequences are owed, on the interpreter.
    ///
    /// Returns `false` if the processor is parked — held in `INIT`, or waiting
    /// for a Start-Up that has not come — in which case there is nothing for
    /// hardware to do this slice.
    fn run_sequences(&self) -> bool {
        for _ in 0..MAX_SEQUENCES {
            if !self.sequence_owed() {
                return true;
            }
            // Zero cycles is how `cpu::x86` says "I am stopped, do not spin":
            // held in `INIT`, or in wait-for-SIPI with nothing to leave it.
            if self.shell.step() == 0 {
                return false;
            }
            // A restart is the one thing that outranks a `HLT`, and it has
            // just rewritten the register file.
            self.halted.store(false, Ordering::Release);
            self.stopped.store(false, Ordering::Release);
            self.dirty.store(true, Ordering::Release);
        }
        !self.sequence_owed()
    }

    /// One slice of guest execution. The body of [`Device::run`].
    fn slice(&self, ticks: u64) -> AccelResult<()> {
        if self.stopped.load(Ordering::Acquire) {
            return Ok(());
        }
        self.poll_controller();
        if !self.run_sequences() {
            return Ok(());
        }
        let Some(vcpu) = self.vcpu() else {
            return Err(AccelError::Unsupported(
                "this processor has no vCPU: the machine never bound it to an address space",
            ));
        };
        let space = self.memory.lock().clone();
        if let Some(space) = space {
            self.host.ensure_map(&space)?;
        }
        if self.dirty.swap(false, Ordering::AcqRel) {
            state::load_into_vcpu(&self.shell, &vcpu)?;
        }
        // An edge, taken once. It also ends a `HLT`, which is the whole point
        // of a non-maskable interrupt.
        if self.nmi.take() {
            vcpu.nmi()?;
            self.halted.store(false, Ordering::Release);
        }
        if self.halted.load(Ordering::Acquire) {
            // Parked. The predicate is the interpreter's: the pin is asserted
            // and the guest can take it. `interrupts_enabled` reads `IF` as of
            // the exit that halted us, which is the last thing the guest did.
            if !(self.shell.intr_asserted() && vcpu.interrupts_enabled()) {
                return Ok(());
            }
            self.halted.store(false, Ordering::Release);
        }
        let entries = ticks.clamp(1, MAX_ENTRIES);
        let run = vcpu.run_until_exit_with(entries, Some(self))?;
        self.entries
            .fetch_add(run.consumed.ticks, Ordering::Relaxed);
        // The shell is made current before anything is decided about the exit,
        // so a `save` between rounds needs no cooperation from this module.
        state::store_from_vcpu(&vcpu, &self.shell)?;
        if let Some(exit) = run.exit {
            match exit.reason {
                ExitReason::HALT => self.halted.store(true, Ordering::Release),
                ExitReason::SHUTDOWN => {
                    // A triple fault. The processor stops until something
                    // resets it, which is what the interpreter does too.
                    self.stopped.store(true, Ordering::Release);
                    *self.failure.lock() = Some("the guest shut down".to_string());
                }
                _ => {
                    self.stopped.store(true, Ordering::Release);
                    *self.failure.lock() = Some(alloc::format!(
                        "the guest stopped at {:#x} with {} ({:#x})",
                        exit.pc,
                        exit.reason,
                        exit.detail
                    ));
                }
            }
        }
        Ok(())
    }
}

/// The board on the far end of this processor's `INTR` pin — which is the
/// processor itself, because the level comes from the net and the vector comes
/// from whatever controller answers the acknowledge, and only the core sees
/// both.
impl IntrSource for AccelCpu {
    fn intr_asserted(&self) -> bool {
        self.shell.intr_asserted()
    }

    fn acknowledge(&self) -> u8 {
        self.shell.acknowledge()
    }
}

// ---------------------------------------------------------------------------
// the device
// ---------------------------------------------------------------------------

impl Device for AccelCpu {
    fn class(&self) -> &'static DeviceClass {
        self.shell.class()
    }

    fn realize(&self, ctx: &mut RealizeCtx<'_>) -> Result<()> {
        self.shell.realize(ctx)
    }

    fn debug_translate(&self, va: u64) -> DebugTranslation {
        self.shell.debug_translate(va)
    }

    /// Every pin but `nmi` is the shell's own.
    ///
    /// `intr`, `reset`, `init` and `a20` all end in a latch or a level the
    /// shell already keeps and already saves, and this engine reads them
    /// through `X86`'s public surface. `nmi` is the exception the module
    /// documentation argues.
    fn sink(&self, port: &str, sources: &[WireId]) -> Option<SinkPin> {
        if port == "nmi" {
            let pin = Arc::new(NmiPin {
                latch: Arc::clone(&self.nmi),
                inputs: FanIn::new(sources),
            });
            self.pins.lock().push(Arc::clone(&pin));
            return Some(SinkPin { sink: pin, line: 0 });
        }
        self.shell.sink(port, sources)
    }

    fn connect(&self, port: &str, source: WireSource) -> Result<()> {
        self.shell.connect(port, source)
    }

    fn announce(&self, port: &str) {
        self.shell.announce(port);
    }

    fn export(&self, which: ExportId) -> Option<Export> {
        self.shell.export(which)
    }

    fn region(&self, name: &str) -> Option<crate::core::space::RegionRef> {
        self.shell.region(name)
    }

    fn int_ack(&self, port: &str) -> Option<Arc<dyn IntAck>> {
        self.shell.int_ack(port)
    }

    fn attach_int_ack(&self, port: &str, ack: Weak<dyn IntAck>) {
        self.shell.attach_int_ack(port, ack);
    }

    fn dma_peripheral(&self, port: &str) -> Option<Arc<dyn DmaPeripheral>> {
        self.shell.dma_peripheral(port)
    }

    fn attach_dma_peripheral(&self, port: &str, peer: Weak<dyn DmaPeripheral>) {
        self.shell.attach_dma_peripheral(port, peer);
    }

    fn local_controller(&self, port: &str) -> Option<Arc<dyn LocalController>> {
        self.shell.local_controller(port)
    }

    /// Kept **twice**: once here, once in the shell.
    ///
    /// The shell's copy is what writes `init_peer` into a snapshot and what
    /// answers `IA32_APIC_BASE`; ours is what gets asked between guest entries,
    /// because the shell only asks inside a step this engine does not take.
    /// [`take_startup`](crate::core::wire::LocalController::take_startup) is
    /// consuming, so the two never both see one message — but they both write
    /// the *same* latches on the same shell, so nothing is lost either way.
    fn attach_local_controller(&self, port: &str, peer: Weak<dyn LocalController>) {
        if matches!(port, "intr" | "nmi") {
            *self.intc.lock() = Some(peer.clone());
            self.has_intc.store(true, Ordering::Release);
        }
        self.shell.attach_local_controller(port, peer);
    }

    fn is_runnable(&self) -> bool {
        true
    }

    /// Enter the guest.
    ///
    /// The whole budget is reported however long the host took, for the reason
    /// [`VcpuRunnable`](super::kvm::VcpuRunnable) gives: an accelerated guest's
    /// progress is measured by the host's silicon, not by a tick counter this
    /// engine could produce, and reporting *less* than the budget would make
    /// virtual time crawl while the guest ran at full speed.
    fn run(&self, budget: Budget) -> Consumed {
        if let Err(e) = self.slice(budget.ticks) {
            self.stopped.store(true, Ordering::Release);
            *self.failure.lock() = Some(e.to_string());
        }
        Consumed::new(budget.ticks)
    }

    /// A reset is the shell's, and the shell's post-reset state is then this
    /// processor's.
    ///
    /// Nothing is written into the vCPU here: the sequence itself has not run
    /// yet — `cpu::x86` leaves it pending, because a reset is a signal rather
    /// than a method call — and the next [`run`](Device::run) executes it and
    /// pushes the result.
    fn reset(&self, kind: ResetKind) {
        self.shell.reset(kind);
        self.nmi.take();
        self.nmi.set(false);
        self.init_peer.store(false, Ordering::Release);
        self.halted.store(false, Ordering::Release);
        self.stopped.store(false, Ordering::Release);
        *self.failure.lock() = None;
        self.dirty.store(true, Ordering::Release);
    }

    fn flush(&self) -> Result<()> {
        self.shell.flush()
    }

    /// The interpreter's chunk, verbatim — version 7, byte for byte.
    ///
    /// That is the cross-engine half of phase 7's gate and it is achieved by
    /// *not having a second format*: the shell is current after every slice,
    /// so what is written here is what the guest has. The three fields this
    /// engine keeps outside the shell — a pending `NMI`, the `NMI` level and
    /// `halted` — are named in the module documentation rather than smuggled
    /// into a chunk the interpreter could not read.
    fn save(&self, w: &mut ChunkWriter<'_>) -> Result<()> {
        self.shell.save(w)
    }

    fn load(&self, r: &mut ChunkReader<'_>) -> Result<()> {
        self.shell.load(r)?;
        // Whatever the vCPU has is now stale, and the next slice pushes.
        self.halted.store(false, Ordering::Release);
        self.stopped.store(false, Ordering::Release);
        self.dirty.store(true, Ordering::Release);
        Ok(())
    }
}

impl Initiator for AccelCpu {
    fn requester(&self) -> RequesterId {
        self.shell.requester()
    }
}

impl Instance for AccelCpu {
    /// The shell gets its address spaces the way it always did; the vCPU is
    /// created from the same ones.
    ///
    /// This is also where the board's memory map becomes memory slots. `bind`
    /// runs after every region is mapped, which is the earliest moment a flat
    /// view means anything.
    ///
    /// # Errors
    ///
    /// Whatever the shell refuses — a processor with no address space — and a
    /// configuration error naming this instance if the hypervisor refuses the
    /// map or the vCPU.
    fn bind(&self, ctx: &BindCtx<'_>) -> Result<()> {
        Instance::bind(&self.shell, ctx)?;
        let memory = self.shell.space().ok_or_else(|| Error::Config {
            at: ctx.path().to_string(),
            message: String::from("an accelerated x86 needs an address space (`space = mem`)"),
        })?;
        let io = self.shell.io_space();
        let fail = |what: AccelError| Error::Config {
            at: ctx.path().to_string(),
            message: what.to_string(),
        };
        self.host.ensure_map(&memory).map_err(fail)?;
        let mut vcpu = self
            .host
            .vm()
            .create_vcpu(self.id, Arc::clone(&memory), io)
            .map_err(fail)?;
        vcpu.set_requester(ctx.requester());
        *self.memory.lock() = Some(memory);
        *self.vcpu.lock() = Some(Arc::new(vcpu));
        Ok(())
    }
}

#[cfg(test)]
mod tests;
