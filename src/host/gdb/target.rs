//! What the protocol talks to: the debug seam, and a [`Machine`] behind it.
//!
//! [`DebugTarget`] is everything [`stub`](super::stub) needs and nothing more —
//! registers, memory, breakpoints, and two ways to make time pass. Keeping it a
//! trait means the packet layer is testable against a stub target with no
//! machine at all, which is how the protocol tests below run in a `--no-default-
//! features` build's worth of time.
//!
//! # The two rules
//!
//! **Every debugger access sets [`MemAttrs::debug`]** — `ROADMAP.md` §15
//! invariant 5. There is exactly one place in this file that builds a
//! `MemAttrs`, [`debug_attrs`], and it starts from [`MemAttrs::DEBUG`]. A
//! debugger read must not acknowledge an interrupt, pop a FIFO or advance a
//! pointer, and the watchpoint implementation below *polls memory after every
//! instruction*, so a single careless access would be a side effect a thousand
//! times a second.
//!
//! **Every address in a packet is virtual.** `m`, `M`, `X` and the `Z` packets
//! name addresses the *guest* uses, so with an MMU on they are not the
//! addresses the bus wants. [`MachineTarget::translate`] is where that is
//! answered, through [`Device::debug_translate`](crate::core::device::Device::debug_translate)
//! — which means the page-table walk is the core's, runs with `debug` set, and
//! sets no accessed or dirty bit on the way. A core with no MMU answers the
//! identity and pays nothing. Breakpoints need none of this: they are compared
//! against the program counter, which is itself virtual, so a `Z0` address and
//! a `$pc` are already in the same space. [`MachineTarget::read_physical`] is
//! the deliberate way out for a caller that really does mean a bus address.
//!
//! **Attaching stops the world.** The machine advances only inside
//! [`DebugTarget::resume`] and [`DebugTarget::step`], both of which are called
//! from the same thread that services packets and never while a packet is being
//! answered. `Machine::step_until` returns at a scheduling boundary with every
//! runnable unwound to the scheduler, which is the safe point of §4.7 in the
//! `Deterministic` threading mode — the only mode `Machine` drives. Nothing here
//! reaches into a running CPU, and nothing races the scheduler.

use core::fmt;

use crate::core::clock::GlobalTime;
use crate::core::device::DeviceClass;
use crate::core::space::{AddressSpace, MemAttrs, RequesterId};
use crate::core::state::{ChunkReader, MachineShape, StateReader, StateWriter};
use crate::machine::Machine;

use super::arch::Arch;

/// Attributes for every access a debugger makes.
///
/// The single constructor, so "does the gdbstub set `debug`?" has one place to
/// look. `requester` is the CPU's own, so an IOMMU or a per-master filter
/// translates a debugger read exactly as it would translate that CPU's.
#[must_use]
pub fn debug_attrs(requester: RequesterId) -> MemAttrs {
    MemAttrs::DEBUG.with_requester(requester)
}

/// Why a target refused.
#[derive(Debug)]
pub enum TargetError {
    /// No CPU with that index — a thread id the client made up.
    NoSuchCpu,
    /// No register with that number.
    NoSuchRegister,
    /// The guest bus refused the access, or there is no address space to make
    /// it in.
    Fault,
    /// The CPU's page tables map nothing at that virtual address.
    ///
    /// Distinct from [`TargetError::Fault`] because the two send a user to
    /// different places: a fault means the machine has no memory there, and
    /// this means the *guest* has not mapped any — usually that the debugger is
    /// looking at an address belonging to a process that is not current.
    Unmapped,
    /// A well-formed request this target cannot serve.
    Unsupported,
    /// A core changed its snapshot layout out from under its register map.
    LayoutMismatch {
        /// The device class whose layout moved.
        class: &'static str,
        /// The version the map was written against.
        expected: u32,
        /// The version the class is at now.
        found: u32,
    },
    /// Whatever the machine reported.
    Machine(crate::Error),
}

impl TargetError {
    /// The number this becomes in an `E<xx>` reply.
    ///
    /// GDB shows these to the user as errno values, so they are chosen to read
    /// sensibly: `EIO` for a bus fault, `EINVAL` for a bad request, `ESRCH` for
    /// a thread that does not exist.
    #[must_use]
    pub const fn code(&self) -> u8 {
        match self {
            TargetError::NoSuchCpu => 3,                                  // ESRCH
            TargetError::Fault | TargetError::Machine(_) => 5,            // EIO
            TargetError::Unmapped => 14,                                  // EFAULT
            TargetError::NoSuchRegister | TargetError::Unsupported => 22, // EINVAL
            TargetError::LayoutMismatch { .. } => 8,                      // ENOEXEC
        }
    }
}

impl fmt::Display for TargetError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            TargetError::NoSuchCpu => f.write_str("no such cpu"),
            TargetError::NoSuchRegister => f.write_str("no such register"),
            TargetError::Fault => f.write_str("the guest bus refused the access"),
            TargetError::Unmapped => f.write_str("nothing is mapped at that virtual address"),
            TargetError::Unsupported => f.write_str("unsupported"),
            TargetError::LayoutMismatch {
                class,
                expected,
                found,
            } => write!(
                f,
                "`{class}` state version {found} but its gdb register map was written \
                 against version {expected}"
            ),
            TargetError::Machine(e) => write!(f, "{e}"),
        }
    }
}

impl std::error::Error for TargetError {}

impl From<crate::Error> for TargetError {
    fn from(e: crate::Error) -> TargetError {
        TargetError::Machine(e)
    }
}

/// The usual result in this module.
pub type TargetResult<T> = Result<T, TargetError>;

/// Why the target stopped.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum StopKind {
    /// A step finished, or the client asked for a halt.
    Trap,
    /// A `Z0`/`Z1` breakpoint address was reached.
    Breakpoint,
    /// A `Z2` watchpoint's memory changed.
    Watchpoint {
        /// The first address in the watched range whose value changed.
        addr: u64,
    },
    /// The client sent Ctrl-C.
    Interrupt,
}

/// A stop, as a stop reply will report it.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Stop {
    /// Which CPU stopped, as an index into the target's CPU list.
    pub cpu: usize,
    /// What happened.
    pub kind: StopKind,
}

impl Stop {
    /// The signal number GDB is told about.
    #[must_use]
    pub const fn signal(&self) -> u8 {
        match self.kind {
            // SIGINT, so Ctrl-C in GDB reads as an interrupt rather than a
            // mysterious trap.
            StopKind::Interrupt => 2,
            _ => 5, // SIGTRAP
        }
    }
}

/// Which watchpoint kinds a target can honour.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct WatchSupport {
    /// `Z2` — stop when the watched bytes change.
    pub write: bool,
    /// `Z3` — stop when the guest reads the watched bytes.
    pub read: bool,
    /// `Z4` — stop on either.
    pub access: bool,
}

/// What the debugger can do to a machine.
///
/// Every method takes an explicit CPU index rather than an implicit "current
/// thread": which thread a packet applies to is the protocol's business (the
/// `H` packets), and leaking that into the target is how a stub ends up reading
/// the wrong CPU's registers after a `vCont`.
pub trait DebugTarget {
    /// How many CPUs this machine presents as GDB threads.
    fn cpu_count(&self) -> usize;

    /// The instance path of a CPU, which is also what `qThreadExtraInfo`
    /// reports.
    fn cpu_path(&self, cpu: usize) -> TargetResult<&str>;

    /// The register map for a CPU.
    fn arch(&self, cpu: usize) -> TargetResult<&'static Arch>;

    /// Every register, concatenated in `g`-packet order.
    fn read_registers(&self, cpu: usize) -> TargetResult<Vec<u8>>;

    /// Overwrite every register from a `G` payload.
    fn write_registers(&mut self, cpu: usize, data: &[u8]) -> TargetResult<()>;

    /// One register, by its number in the target description.
    fn read_register(&self, cpu: usize, index: usize) -> TargetResult<Vec<u8>>;

    /// Overwrite one register.
    fn write_register(&mut self, cpu: usize, index: usize, data: &[u8]) -> TargetResult<()>;

    /// Read guest memory as this CPU sees it, with no side effects.
    ///
    /// **`addr` is virtual.** Every address GDB sends in an `m`, `M`, `X` or
    /// `Z` packet is one the guest would use — that is what "as this CPU sees
    /// it" has always meant — so an implementation with an MMU must translate
    /// it, with [`Device::debug_translate`](crate::core::device::Device::debug_translate)
    /// or its own equivalent. On a machine with no MMU, or with the MMU off,
    /// that translation is the identity and nothing changes.
    fn read_memory(&self, cpu: usize, addr: u64, dst: &mut [u8]) -> TargetResult<()>;

    /// Write guest memory as this CPU sees it.
    ///
    /// `addr` is virtual, exactly as in [`read_memory`](DebugTarget::read_memory).
    fn write_memory(&mut self, cpu: usize, addr: u64, src: &[u8]) -> TargetResult<()>;

    /// Arm a breakpoint at `addr`. Arming one twice is not an error.
    fn add_breakpoint(&mut self, addr: u64) -> TargetResult<()>;

    /// Disarm a breakpoint. Disarming one that is not set is not an error.
    fn remove_breakpoint(&mut self, addr: u64) -> TargetResult<()>;

    /// Which watchpoint kinds this target honours.
    fn watch_support(&self) -> WatchSupport {
        WatchSupport::default()
    }

    /// Arm a write watchpoint over `len` bytes at `addr`.
    fn add_watchpoint(&mut self, _addr: u64, _len: u64) -> TargetResult<()> {
        Err(TargetError::Unsupported)
    }

    /// Disarm a write watchpoint.
    fn remove_watchpoint(&mut self, _addr: u64, _len: u64) -> TargetResult<()> {
        Err(TargetError::Unsupported)
    }

    /// Run one instruction on `cpu`.
    fn step(&mut self, cpu: usize) -> TargetResult<Stop>;

    /// Told that a continue is starting, before the first [`DebugTarget::resume`].
    ///
    /// A target that stopped *on* a breakpoint has to get off it before it
    /// starts looking again, or `continue` reports the same breakpoint forever.
    fn begin_resume(&mut self) {}

    /// Let the machine run for a bounded slice, and report a stop if one
    /// happened.
    ///
    /// Bounded rather than open-ended so the caller keeps servicing the socket:
    /// this is what makes Ctrl-C work.
    fn resume(&mut self) -> TargetResult<Option<Stop>>;

    /// Answer a `qRcmd` monitor command. `None` means "no such command".
    fn monitor(&mut self, _command: &str) -> Option<String> {
        None
    }
}

// ---------------------------------------------------------------------------
// A Machine as a target
// ---------------------------------------------------------------------------

/// How much virtual time one free-running slice covers.
///
/// Ten milliseconds, matching the console loop in `src/bin/rsemu.rs`: short
/// enough that a Ctrl-C is noticed straight away, long enough that the socket
/// poll is not the bottleneck.
const FREE_SLICE: GlobalTime = GlobalTime::from_nanos(10_000_000);

/// How many single ticks one breakpoint-checking slice covers.
///
/// With a breakpoint or a watchpoint armed the machine advances one clock tick
/// at a time so nothing can be stepped over, which costs one or two orders of
/// magnitude of speed. That is the price of not patching trap instructions into
/// guest memory — see [`MachineTarget`]'s docs.
const FINE_TICKS: u32 = 4096;

/// The largest span a single translation is trusted for.
///
/// One kibibyte, because that is the *smallest* page any MMU in the tree can
/// map: VMSAv5's tiny page (ARM ARM B4.3.2) is 1 KiB, and RISC-V's smallest is
/// four times that. A run that fits inside an aligned granule cannot straddle a
/// page boundary, so one translation covers it. Lowering this is always safe;
/// raising it needs an MMU with no page smaller than the new value.
const TRANSLATION_GRANULE: u64 = 1024;

/// How many ticks one instruction is allowed to take before the stepper gives
/// up and reports what it has.
///
/// The slowest documented instruction on any core rsemu has is well under
/// twenty cycles; the margin is for a core that is paying down scheduler debt,
/// which costs ticks without retiring anything.
const MAX_TICKS_PER_INSN: u32 = 4096;

/// One CPU, as the debugger sees it.
#[derive(Debug)]
struct Cpu {
    /// Index into `Machine::devices`.
    device: usize,
    path: String,
    class: &'static DeviceClass,
    arch: &'static Arch,
    domain: crate::core::clock::DomainId,
    requester: RequesterId,
    space: Option<usize>,
}

/// A watched range and the bytes it last held.
#[derive(Debug)]
struct Watch {
    addr: u64,
    len: u64,
    shadow: Vec<u8>,
}

/// A realized [`Machine`], as a GDB target.
///
/// # Breakpoints are compared, not planted
///
/// The usual gdbstub writes a trap instruction over the guest's code and puts
/// the original byte back afterwards. That needs a per-architecture trap
/// encoding, it does not work in ROM — which is where an Apple 1 or a NES spends
/// most of its time — and it makes the guest's own view of its code wrong. This
/// target instead compares each CPU's program counter against the armed set
/// after every clock tick. It costs speed while a breakpoint is armed and
/// nothing at all while none is, it works identically in RAM and ROM, and it
/// cannot corrupt the guest.
///
/// # Watchpoints are polled
///
/// `Z2` is honoured the same way: the watched bytes are read (with
/// [`MemAttrs::debug`] set, so the read itself cannot be what changed them) after
/// every tick and compared against a shadow copy. `Z3` and `Z4` — read and
/// access watchpoints — are **not** implementable here, and are refused rather
/// than faked: seeing a guest *read* requires interposing on the access path,
/// and `core::space` exposes no hook to interpose with. See the module docs of
/// [`super`].
#[derive(Debug)]
pub struct MachineTarget<'a> {
    machine: &'a mut Machine,
    cpus: Vec<Cpu>,
    breakpoints: Vec<u64>,
    watchpoints: Vec<Watch>,
    /// Per CPU: an address the resume loop must not report until the program
    /// counter has moved off it. Set by [`DebugTarget::begin_resume`].
    suppress: Vec<Option<u64>>,
}

impl<'a> MachineTarget<'a> {
    /// Wrap a machine, discovering its CPUs.
    ///
    /// A device is a CPU here when this build has a register map for its class
    /// ([`super::arch::for_class`]) and the machine gave it a clock domain. A
    /// core with no domain cannot be stepped, and presenting a thread that
    /// cannot be stepped is worse than not presenting it.
    #[must_use]
    pub fn new(machine: &'a mut Machine) -> MachineTarget<'a> {
        let mut cpus = Vec::new();
        for (index, entry) in machine.devices().iter().enumerate() {
            let Some(arch) = super::arch::for_class(entry.class().name) else {
                continue;
            };
            let Some(domain) = entry.domain() else {
                continue;
            };
            cpus.push(Cpu {
                device: index,
                path: entry.path().to_string(),
                class: entry.class(),
                arch,
                domain,
                requester: entry.requester(),
                space: entry.space_index(),
            });
        }
        let suppress = vec![None; cpus.len()];
        MachineTarget {
            machine,
            cpus,
            breakpoints: Vec::new(),
            watchpoints: Vec::new(),
            suppress,
        }
    }

    /// The machine being debugged.
    #[must_use]
    pub fn machine(&self) -> &Machine {
        self.machine
    }

    /// Read guest memory at a **physical** address, skipping translation.
    ///
    /// The untranslated half of the pair, and it is not a leftover: a monitor
    /// inspecting a boot ROM, a board bring-up check and anything that wants to
    /// see what is under a page table rather than through it all name bus
    /// addresses. GDB's own packets never do — there is no physical-address
    /// packet in the protocol — which is why this is an inherent method on the
    /// concrete target rather than part of [`DebugTarget`].
    ///
    /// # Errors
    ///
    /// [`TargetError::Fault`] if the bus refuses.
    pub fn read_physical(&self, cpu: usize, addr: u64, dst: &mut [u8]) -> TargetResult<()> {
        let entry = self.cpu(cpu)?;
        let space = self.space(entry)?;
        space
            .read_bytes(addr, dst, debug_attrs(entry.requester))
            .map_err(|_| TargetError::Fault)
    }

    /// Write guest memory at a **physical** address, skipping translation.
    ///
    /// See [`read_physical`](MachineTarget::read_physical).
    ///
    /// # Errors
    ///
    /// [`TargetError::Fault`] if the bus refuses.
    pub fn write_physical(&mut self, cpu: usize, addr: u64, src: &[u8]) -> TargetResult<()> {
        {
            let entry = self.cpu(cpu)?;
            let space = self.space(entry)?;
            space
                .write_bytes(addr, src, debug_attrs(entry.requester))
                .map_err(|_| TargetError::Fault)?;
        }
        self.resync_watchpoints();
        Ok(())
    }

    /// The machine being debugged, mutably.
    ///
    /// For a front end that has work of its own between session turns — pumping
    /// a console, checking a deadline. Running it from here would race the
    /// debugger's own idea of whether the guest is stopped, so do not.
    pub fn machine_mut(&mut self) -> &mut Machine {
        self.machine
    }

    fn cpu(&self, index: usize) -> TargetResult<&Cpu> {
        self.cpus.get(index).ok_or(TargetError::NoSuchCpu)
    }

    /// The address space a CPU's accesses go through.
    fn space(&self, cpu: &Cpu) -> TargetResult<&AddressSpace> {
        let index = cpu.space.ok_or(TargetError::Fault)?;
        self.machine
            .spaces()
            .get(index)
            .map(|entry| entry.space().as_ref())
            .ok_or(TargetError::Fault)
    }

    /// Where a virtual address lives, as this CPU's MMU maps it.
    ///
    /// The whole of the debugger's answer to paging: a `m` packet names an
    /// address the *guest* would use, and with the MMU on that is not the
    /// address the bus wants. A core with no MMU — or one whose MMU is off —
    /// answers [`DebugTranslation::Identity`](crate::core::device::DebugTranslation::Identity)
    /// and this is free.
    ///
    /// The walk itself is the core's, runs with [`MemAttrs::debug`] set, and
    /// sets no accessed or dirty bit; see
    /// [`Device::debug_translate`](crate::core::device::Device::debug_translate).
    ///
    /// # Errors
    ///
    /// [`TargetError::Unmapped`] when the tables map nothing there.
    pub fn translate(&self, cpu: usize, va: u64) -> TargetResult<u64> {
        let device = self.cpu(cpu)?.device;
        let entry = self
            .machine
            .devices()
            .get(device)
            .ok_or(TargetError::NoSuchCpu)?;
        entry
            .device()
            .debug_translate(va)
            .phys(va)
            .ok_or(TargetError::Unmapped)
    }

    /// Split a virtual range into runs that are contiguous in physical memory.
    ///
    /// Returns `(physical address, offset into the caller's buffer, length)`.
    ///
    /// Translating once per byte would be correct and unusably slow — a
    /// watchpoint poll runs this on every clock tick — and translating once for
    /// the whole range would be wrong the moment it crosses a page boundary,
    /// which a `m` packet for a 4 KiB region routinely does. So the range is cut
    /// on [`TRANSLATION_GRANULE`] boundaries: no MMU in the tree has a page
    /// smaller than that, so a run inside one is guaranteed contiguous, and the
    /// common case — a request that fits inside a single granule — is one
    /// translation and one bulk bus access, exactly as it was before this
    /// existed. Bulk, and not a byte at a time, because a 32-bit-only register
    /// must still see a 32-bit access.
    fn chunks(&self, cpu: usize, addr: u64, len: usize) -> TargetResult<Vec<(u64, usize, usize)>> {
        let mut out = Vec::new();
        let mut at = 0usize;
        while at < len {
            let va = addr.wrapping_add(at as u64);
            // How far to the next granule boundary, capped by what is left.
            let to_boundary = TRANSLATION_GRANULE - (va & (TRANSLATION_GRANULE - 1));
            let run = to_boundary.min((len - at) as u64) as usize;
            out.push((self.translate(cpu, va)?, at, run));
            at += run;
        }
        Ok(out)
    }

    /// A CPU's architectural state, as its snapshot chunk.
    ///
    /// This is `Device::save` into a chunk of its own rather than
    /// `Machine::save` of everything: a register read happens on every stop and
    /// on every tick of a breakpoint-checking run, and serialising the guest's
    /// RAM each time would make the debugger unusable.
    fn chunk(&self, cpu: &Cpu) -> TargetResult<Vec<u8>> {
        if !cpu.arch.check() {
            return Err(TargetError::LayoutMismatch {
                class: cpu.class.name,
                expected: cpu.arch.verified_version,
                found: cpu.class.version,
            });
        }
        let entry = self
            .machine
            .devices()
            .get(cpu.device)
            .ok_or(TargetError::NoSuchCpu)?;
        let mut writer = StateWriter::new(MachineShape::new());
        {
            let mut chunk = writer.chunk(&cpu.path, cpu.class.name, cpu.class.version)?;
            entry.device().save(&mut chunk)?;
        }
        let bytes = writer.to_vec()?;
        let reader = StateReader::new(&bytes)?;
        let (_, _, data) = reader.load_raw(&cpu.path)?;
        if data.len() < cpu.arch.chunk_reach() {
            return Err(TargetError::LayoutMismatch {
                class: cpu.class.name,
                expected: cpu.arch.verified_version,
                found: cpu.class.version,
            });
        }
        Ok(data.to_vec())
    }

    /// Put a patched chunk back.
    fn set_chunk(&mut self, cpu: usize, data: &[u8]) -> TargetResult<()> {
        let device = self.cpu(cpu)?.device;
        let entry = self
            .machine
            .devices()
            .get(device)
            .ok_or(TargetError::NoSuchCpu)?;
        let mut reader = ChunkReader::new(data);
        entry.device().load(&mut reader)?;
        Ok(())
    }

    /// A little-endian field of the chunk, widened.
    fn field(chunk: &[u8], offset: usize, bytes: usize) -> TargetResult<u64> {
        let slice = chunk
            .get(offset..offset.checked_add(bytes).ok_or(TargetError::Fault)?)
            .ok_or(TargetError::Fault)?;
        let mut value: u64 = 0;
        for (i, byte) in slice.iter().enumerate() {
            value |= u64::from(*byte) << (i * 8);
        }
        Ok(value)
    }

    /// A CPU's program counter.
    fn pc_of(&self, index: usize) -> TargetResult<u64> {
        let cpu = self.cpu(index)?;
        let chunk = self.chunk(cpu)?;
        let reg = cpu
            .arch
            .regs
            .get(cpu.arch.pc)
            .ok_or(TargetError::NoSuchRegister)?;
        Self::field(&chunk, reg.offset, reg.bytes)
    }

    /// A CPU's instruction-retirement counter, if it has one.
    fn retired(&self, index: usize) -> TargetResult<Option<u64>> {
        let cpu = self.cpu(index)?;
        let Some(counter) = cpu.arch.retire else {
            return Ok(None);
        };
        let chunk = self.chunk(cpu)?;
        Self::field(&chunk, counter.offset, counter.bytes).map(Some)
    }

    /// Advance virtual time by one tick of the finest CPU clock domain.
    ///
    /// The finest, so that on a machine with a fast and a slow core neither is
    /// stepped over. Time is advanced through `Machine::step_until`, which
    /// returns with every runnable unwound — the safe point of §4.7.
    fn tick(&mut self) -> TargetResult<()> {
        let now = self.machine.now();
        let mut deadline: Option<GlobalTime> = None;
        {
            let forest = self.machine.clocks();
            for cpu in &self.cpus {
                let Ok(tick) = forest.ticks(cpu.domain) else {
                    continue;
                };
                // A domain whose next tick has already gone by (it is behind
                // the timeline, or gated) needs the next one that has not.
                let mut ahead = 1u64;
                while let Ok(at) =
                    forest.global_time_of_tick(cpu.domain, tick.saturating_add(ahead))
                {
                    if at > now {
                        deadline = Some(match deadline {
                            Some(best) if best <= at => best,
                            _ => at,
                        });
                        break;
                    }
                    ahead += 1;
                    if ahead > 1024 {
                        break;
                    }
                }
            }
        }
        // No CPU could name a future tick — a machine with every clock gated.
        // Fall back to a plain slice so the scheduler's own events still fire.
        let deadline = deadline.unwrap_or_else(|| now.saturating_add(FREE_SLICE));
        // `step_until`, not `run_until`: one CPU tick is far inside a
        // scheduling round, and `run_until` declines a round its deadline falls
        // inside so that a sliced run and an unsliced one agree (§11.6). A
        // debugger is the one caller that needs the fragment instead — waiting
        // for the round's own boundary would step over every breakpoint between
        // here and it.
        self.machine.step_until(deadline)?;
        Ok(())
    }

    /// Re-read every watched range, and report the first one that moved.
    fn poll_watchpoints(&mut self) -> TargetResult<Option<u64>> {
        if self.watchpoints.is_empty() {
            return Ok(None);
        }
        let mut hit = None;
        for i in 0..self.watchpoints.len() {
            let (addr, len) = {
                let watch = &self.watchpoints[i];
                (watch.addr, watch.len)
            };
            let mut now = vec![0u8; usize::try_from(len).unwrap_or(0)];
            // A range the bus refuses is not a hit; it is a watchpoint the user
            // put somewhere there is no memory, and saying so once a tick would
            // drown the session.
            if self.read_memory(0, addr, &mut now).is_err() {
                continue;
            }
            let watch = &mut self.watchpoints[i];
            if watch.shadow != now {
                watch.shadow = now;
                if hit.is_none() {
                    hit = Some(addr);
                }
            }
        }
        Ok(hit)
    }

    /// Refresh every shadow without reporting a hit.
    ///
    /// Called after the debugger writes memory itself, so that GDB poking a
    /// watched byte does not immediately trip its own watchpoint.
    fn resync_watchpoints(&mut self) {
        for i in 0..self.watchpoints.len() {
            let (addr, len) = {
                let watch = &self.watchpoints[i];
                (watch.addr, watch.len)
            };
            let mut now = vec![0u8; usize::try_from(len).unwrap_or(0)];
            if self.read_memory(0, addr, &mut now).is_ok() {
                self.watchpoints[i].shadow = now;
            }
        }
    }

    /// Whether any CPU is standing on an armed breakpoint.
    fn breakpoint_hit(&mut self) -> TargetResult<Option<Stop>> {
        if self.breakpoints.is_empty() {
            return Ok(None);
        }
        for index in 0..self.cpus.len() {
            let pc = self.pc_of(index)?;
            if self.suppress.get(index).copied().flatten() == Some(pc) {
                continue;
            }
            if let Some(slot) = self.suppress.get_mut(index) {
                *slot = None;
            }
            if self.breakpoints.contains(&pc) {
                return Ok(Some(Stop {
                    cpu: index,
                    kind: StopKind::Breakpoint,
                }));
            }
        }
        Ok(None)
    }
}

impl DebugTarget for MachineTarget<'_> {
    fn cpu_count(&self) -> usize {
        self.cpus.len()
    }

    fn cpu_path(&self, cpu: usize) -> TargetResult<&str> {
        Ok(self.cpu(cpu)?.path.as_str())
    }

    fn arch(&self, cpu: usize) -> TargetResult<&'static Arch> {
        Ok(self.cpu(cpu)?.arch)
    }

    fn read_registers(&self, cpu: usize) -> TargetResult<Vec<u8>> {
        let entry = self.cpu(cpu)?;
        let chunk = self.chunk(entry)?;
        let mut out = Vec::with_capacity(entry.arch.packet_len());
        for reg in entry.arch.regs {
            let slice = chunk
                .get(reg.offset..reg.offset + reg.bytes)
                .ok_or(TargetError::NoSuchRegister)?;
            out.extend_from_slice(slice);
        }
        Ok(out)
    }

    fn write_registers(&mut self, cpu: usize, data: &[u8]) -> TargetResult<()> {
        let entry = self.cpu(cpu)?;
        if data.len() != entry.arch.packet_len() {
            return Err(TargetError::NoSuchRegister);
        }
        let mut chunk = self.chunk(entry)?;
        let mut at = 0usize;
        for reg in entry.arch.regs {
            let src = data.get(at..at + reg.bytes).ok_or(TargetError::Fault)?;
            let dst = chunk
                .get_mut(reg.offset..reg.offset + reg.bytes)
                .ok_or(TargetError::Fault)?;
            dst.copy_from_slice(src);
            at += reg.bytes;
        }
        self.set_chunk(cpu, &chunk)
    }

    fn read_register(&self, cpu: usize, index: usize) -> TargetResult<Vec<u8>> {
        let entry = self.cpu(cpu)?;
        let reg = entry
            .arch
            .regs
            .get(index)
            .ok_or(TargetError::NoSuchRegister)?;
        let chunk = self.chunk(entry)?;
        chunk
            .get(reg.offset..reg.offset + reg.bytes)
            .map(<[u8]>::to_vec)
            .ok_or(TargetError::NoSuchRegister)
    }

    fn write_register(&mut self, cpu: usize, index: usize, data: &[u8]) -> TargetResult<()> {
        let entry = self.cpu(cpu)?;
        let reg = *entry
            .arch
            .regs
            .get(index)
            .ok_or(TargetError::NoSuchRegister)?;
        if data.len() != reg.bytes {
            return Err(TargetError::NoSuchRegister);
        }
        let mut chunk = self.chunk(entry)?;
        let dst = chunk
            .get_mut(reg.offset..reg.offset + reg.bytes)
            .ok_or(TargetError::Fault)?;
        dst.copy_from_slice(data);
        self.set_chunk(cpu, &chunk)
    }

    fn read_memory(&self, cpu: usize, addr: u64, dst: &mut [u8]) -> TargetResult<()> {
        let entry = self.cpu(cpu)?;
        let space = self.space(entry)?;
        let attrs = debug_attrs(entry.requester);
        for (pa, at, len) in self.chunks(cpu, addr, dst.len())? {
            space
                .read_bytes(pa, &mut dst[at..at + len], attrs)
                .map_err(|_| TargetError::Fault)?;
        }
        Ok(())
    }

    fn write_memory(&mut self, cpu: usize, addr: u64, src: &[u8]) -> TargetResult<()> {
        {
            let entry = self.cpu(cpu)?;
            let space = self.space(entry)?;
            let attrs = debug_attrs(entry.requester);
            for (pa, at, len) in self.chunks(cpu, addr, src.len())? {
                space
                    .write_bytes(pa, &src[at..at + len], attrs)
                    .map_err(|_| TargetError::Fault)?;
            }
        }
        self.resync_watchpoints();
        Ok(())
    }

    fn add_breakpoint(&mut self, addr: u64) -> TargetResult<()> {
        if !self.breakpoints.contains(&addr) {
            self.breakpoints.push(addr);
        }
        Ok(())
    }

    fn remove_breakpoint(&mut self, addr: u64) -> TargetResult<()> {
        self.breakpoints.retain(|a| *a != addr);
        Ok(())
    }

    fn watch_support(&self) -> WatchSupport {
        WatchSupport {
            write: true,
            // Seeing a guest *read* needs a hook on the access path, and
            // `core::space` has none. Refused rather than faked.
            read: false,
            access: false,
        }
    }

    fn add_watchpoint(&mut self, addr: u64, len: u64) -> TargetResult<()> {
        if len == 0 || len > 4096 {
            return Err(TargetError::Unsupported);
        }
        if self
            .watchpoints
            .iter()
            .any(|w| w.addr == addr && w.len == len)
        {
            return Ok(());
        }
        let mut shadow = vec![0u8; usize::try_from(len).map_err(|_| TargetError::Unsupported)?];
        self.read_memory(0, addr, &mut shadow)?;
        self.watchpoints.push(Watch { addr, len, shadow });
        Ok(())
    }

    fn remove_watchpoint(&mut self, addr: u64, len: u64) -> TargetResult<()> {
        self.watchpoints
            .retain(|w| !(w.addr == addr && w.len == len));
        Ok(())
    }

    fn step(&mut self, cpu: usize) -> TargetResult<Stop> {
        let before_pc = self.pc_of(cpu)?;
        let before_retired = self.retired(cpu)?;
        for _ in 0..MAX_TICKS_PER_INSN {
            self.tick()?;
            let moved = match before_retired {
                Some(before) => self.retired(cpu)? != Some(before),
                // No retirement counter: fall back to the program counter,
                // which is right except for an instruction that branches to
                // itself.
                None => self.pc_of(cpu)? != before_pc,
            };
            if moved {
                break;
            }
        }
        if let Some(slot) = self.suppress.get_mut(cpu) {
            *slot = None;
        }
        if let Some(addr) = self.poll_watchpoints()? {
            return Ok(Stop {
                cpu,
                kind: StopKind::Watchpoint { addr },
            });
        }
        Ok(Stop {
            cpu,
            kind: StopKind::Trap,
        })
    }

    fn begin_resume(&mut self) {
        for index in 0..self.cpus.len() {
            let here = self.pc_of(index).ok();
            if let Some(slot) = self.suppress.get_mut(index) {
                *slot = here;
            }
        }
    }

    fn resume(&mut self) -> TargetResult<Option<Stop>> {
        // Nothing armed: run flat out. This is the case that matters for
        // "attach, look around, continue".
        if self.breakpoints.is_empty() && self.watchpoints.is_empty() {
            let deadline = self.machine.now().saturating_add(FREE_SLICE);
            self.machine.run_until(deadline)?;
            return Ok(None);
        }
        for _ in 0..FINE_TICKS {
            self.tick()?;
            if let Some(stop) = self.breakpoint_hit()? {
                return Ok(Some(stop));
            }
            if let Some(addr) = self.poll_watchpoints()? {
                return Ok(Some(Stop {
                    cpu: 0,
                    kind: StopKind::Watchpoint { addr },
                }));
            }
        }
        Ok(None)
    }

    fn monitor(&mut self, command: &str) -> Option<String> {
        let mut words = command.split_whitespace();
        match words.next()? {
            "help" => Some(
                "rsemu monitor commands:\n  \
                 devices    the device tree, with class and instance path\n  \
                 time       the machine's current virtual instant\n  \
                 hash       the machine state hash (ROADMAP.md \u{a7}0)\n"
                    .to_string(),
            ),
            "devices" => {
                let mut out = String::new();
                for entry in self.machine.devices() {
                    out.push_str(entry.path());
                    out.push_str("  ");
                    out.push_str(entry.class().name);
                    out.push('\n');
                }
                Some(out)
            }
            "time" => Some(format!("{} ns\n", self.machine.now().as_nanos())),
            "hash" => Some(match self.machine.state_hash() {
                Ok(hash) => format!("{hash:#018x}\n"),
                Err(e) => format!("cannot hash state: {e}\n"),
            }),
            _ => None,
        }
    }
}
