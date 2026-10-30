//! The hardware execution mode: the SGX runs its real firmware.
//!
//! Selected by `mode = "hw"`. The host's kicks and the core's own events
//! launch PDS programs from the event registers; those programs start USSE
//! tasks; the tasks — the real microkernel — read and write the register file
//! with `LDR`/`STR`, GPU memory through the SGX MMU, and start further PDS
//! programs with `EMIT`. Everything the host sees (the kernel CCB's read
//! offset, sync objects, the interrupt) is then the firmware's doing.
//!
//! Like the rest of this device, the model is read off the guest's binaries
//! (our notes `EVENTS.md`, `PDS_ENCODING.md`), not off any documentation.
//! Where the binaries do not pin a behaviour down the choice is marked `[G]`.
//!
//! # Events
//!
//! Each register bank has an event program: the master's at `0xa68`
//! (data rows in `0xa6c`), core 0's at `0x8a68`, core 1's at `0xca68`. A
//! write of bit 0 to a bank's `+0xac0` (start) or `+0xac8` (command kick)
//! raises an event in it:
//!
//! * a start runs the event program with `IR0 = IR1 = 0`;
//! * a command kick runs it with `IR1 = 0x40` — the bit the microkernel's
//!   default handler takes as "a kernel CCB command is waiting" (it tests
//!   `pa1 & 0x40`, and the event program puts `IR1` in `PA1`) \[strong\];
//! * the microkernel timer, enabled by bit 16 of `0x4144` \[G\], raises
//!   `IR0 = 2` every `timer-ticks` device ticks; the default handler counts
//!   those and runs its periodic work on them \[G\].
//!
//! Kicks are counted, not latched: the default handler consumes one CCB slot
//! per event, so a coalesced kick would strand a command. A core takes its
//! next event only when it has nothing queued or running \[G\]. Core 0 runs
//! its own bank's events; core 1 runs its own and the master's, whose program
//! starts the slave microkernel \[G\].
//!
//! # Tasks and phases
//!
//! A task runs on the clocked `run()` slice with a budget of `ipt`
//! instructions per device tick. It ends at END, or at a phase end whose
//! declared next phase is `#0` (how every microkernel task ends). A phase end
//! with a declared next phase continues there, counted from the task's code
//! base; one with nothing declared (opcode 358, the microkernel's "yield")
//! continues at the core's resume register, `+0xba0` \[G\].
//!
//! Branches count from the code-base register a task was launched through,
//! except for a task started by the master event program: the slave image's
//! branch targets only land with its own start as their base, so such a task
//! branches relative to its entry (`EVENTS.md` §6, rule (a)) \[G\]. Phase and
//! resume PCs still count from the code-base register.
//!
//! Secondary attributes belong to the core and persist from task to task;
//! primary attributes and temporaries belong to the task (temporaries start
//! at zero \[G\]). `EMIT` with target 0 launches a PDS program on core 0:
//! control word bits 27:23 are its data rows, the second operand its
//! address >> 4, the third its `IR0`, and immediate bit 10 makes it a
//! *secondary* program whose DMA fills secondary attributes \[G\].
//!
//! # Hardware the firmware waits on
//!
//! The fixed-function units are not modelled, so everything the firmware
//! starts completes at once. The firmware learns that by polling status
//! bits, and every poll in the microkernel waits for bits to become set; a
//! read by the USSE of a known status register therefore sees its completion
//! bits set (`completion`). An unknown poll — the same `LDR` at the same PC
//! returning the same value `POLL_PATIENCE` times in a row — is answered once
//! with all bits set, and logged, so a missing model shows up in the log
//! rather than as a hang \[G\].
//!
//! The host's view of the registers is unaffected by either: what it reads
//! is what was stored.

use alloc::boxed::Box;
use alloc::collections::{BTreeSet, VecDeque};
use alloc::format;
use alloc::string::String;
use alloc::vec::Vec;

use super::pds::{self, Dout, DoutType, Pds, PdsBus};
use super::usse::{self, BANK_SIZE, Emit, Size, Stop, Usse, UsseBus};
use super::{Shared, State};
use crate::core::error::{BusError, Result};
use crate::core::space::{AddressSpace, MemAttrs, MemResult};
use crate::core::state::{ChunkReader, ChunkWriter, Sink, Source};
use crate::core::value::Width;

/// USSE cores (the SGX543MP2 has two).
pub(super) const CORES: usize = 2;

/// Register banks: the master's, then each core's.
const MASTER_BANK: u32 = 0x0000;
const fn core_bank(core: usize) -> u32 {
    0x8000 + 0x4000 * core as u32
}

/// Offsets within a bank.
const EVENT_PDS: u32 = 0xa68;
const EVENT_ROWS: u32 = 0xa6c;
pub(super) const START: u32 = 0xac0;
pub(super) const KICK: u32 = 0xac8;
const RESUME_PC: u32 = 0xba0;

/// USE code-base registers: `0xa0c + 4·idx`, value `VA >> 6 | flags`.
const CODE_BASE: u32 = 0xa0c;
/// The MMU's eight page directories: index 0 at `0xc84`, 1–7 at
/// `0xc38 + 4·(i-1)` (the host programs 7, its own, at `0xc50`; the firmware
/// programs 0, an application's, at `0xc84`, and the driver's lock-up dump
/// names "EDM context index 7" and "TA context index 0" with those values).
const DIRECTORY0: u32 = 0xc84;
const DIRECTORY1: u32 = 0xc38;
/// Which directory each requester uses, a nibble each; the firmware's own
/// accesses take nibble 0 (its routine at f2013bd8 sets nibble 0 to 7 for
/// the kernel's context, or copies the TA's or 3D's nibble into it).
const BANK_SELECT: u32 = 0xc78;
const TIMER_CTL: u32 = 0x4144;
/// The timer period field \[G\]: the host writes 1000 here, the firmware
/// saves it, writes 0x10001 while it sets up, and puts it back.
const TIMER_PERIOD: u32 = 0xffff;

/// The command-kick event's input bit (`IR1`) and the timer's (`IR0`).
const IR1_KICK: u32 = 0x40;
const IR0_TIMER: u32 = 0x2;

/// What a PDS program may retire before it is abandoned as runaway.
const PDS_BUDGET: u64 = 4096;
/// Identical consecutive polls before an unknown one is forced to complete.
const POLL_PATIENCE: u32 = 64;
/// Log lines of each rate-limited kind before thinning out to one in 1000.
const LOG_BURST: u64 = 300;
/// Distinct registers reported as first touched by the firmware.
const LOG_REGS: usize = 600;

/// A USSE register read's completion bits: the bits each status register's
/// polls wait for, which are set because the work they wait on is finished
/// the moment it is started. From the polling loops in the microkernel (main
/// and slave images); the per-core copies at `0x8xxx`/`0xcxxx` share them.
fn completion(offset: u32) -> u32 {
    let local = if offset >= 0x8000 {
        offset & 0x3fff
    } else {
        offset
    };
    match ((0x4000..0x8000).contains(&offset), local) {
        (false, 0x118) => 0xff,
        // Bits 4-6, 9, 10, 15, 16, 19, 20, 26: never bit 14, the host's.
        (false, 0x12c) => 0x0419_86f0,
        (false, 0x138) => 0x7,
        (false, 0x260) => 0x1,
        (false, 0x348) => 0x1,
        (false, 0xb04) => 0x1,
        (true, 0x4288 | 0x4294 | 0x4348) => 0x1,
        (true, 0x433c) => 0x100,
        (true, 0x4360) => 0x3,
        (true, 0x436c) => 0x7,
        (true, 0x4d10) => 0x3,
        _ => 0,
    }
}

/// The page directory the firmware's own accesses go through.
fn edm_directory(s: &State) -> u32 {
    match s.reg(u64::from(BANK_SELECT)) & 0xf {
        0 => s.reg(u64::from(DIRECTORY0)),
        i => s.reg(u64::from(DIRECTORY1 + 4 * (i - 1))),
    }
}

/// Whether a register write can change [`edm_directory`].
fn moves_directory(offset: u32) -> bool {
    offset == BANK_SELECT || offset == DIRECTORY0 || (DIRECTORY1..DIRECTORY1 + 28).contains(&offset)
}

/// A PDS program waiting to run.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
struct Launch {
    base: u32,
    rows: u32,
    ir: [u32; 3],
    /// Its DMA and attribute writes go to the secondary attributes.
    secondary: bool,
    /// Its tasks branch relative to their own entry (the master's program).
    own_base: bool,
}

/// A USSE task.
#[derive(Debug)]
struct Task {
    usse: Box<Usse>,
    /// The code-base register it was launched through.
    cb: u8,
}

/// One USSE core and its PDS.
#[derive(Debug)]
struct Core {
    /// Secondary attributes: the core's, kept across tasks.
    sa: Vec<u32>,
    /// The PDS's persistent temporaries (DS0, DS1).
    persist: [[u32; 8]; 2],
    launches: VecDeque<Launch>,
    tasks: VecDeque<Task>,
    /// A timer event is due.
    timer: bool,
}

impl Default for Core {
    fn default() -> Core {
        Core {
            sa: alloc::vec![0; BANK_SIZE],
            persist: [[0; 8]; 2],
            launches: VecDeque::new(),
            tasks: VecDeque::new(),
            timer: false,
        }
    }
}

impl Core {
    fn idle(&self) -> bool {
        self.launches.is_empty() && self.tasks.is_empty()
    }
}

/// The last USSE register read, for spotting a poll nothing will answer.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
struct Poll {
    pc: u32,
    offset: u32,
    value: u32,
    count: u32,
}

/// What the log has already said; derived, never serialized.
#[derive(Debug, Default)]
struct Diag {
    launches: u64,
    /// Timer events, logged apart so that they do not drown the rest.
    timers: u64,
    stops: u64,
    regs: BTreeSet<(bool, u32)>,
    forced: BTreeSet<u32>,
}

impl Diag {
    fn thin(n: &mut u64) -> bool {
        *n += 1;
        *n <= LOG_BURST || (*n).is_multiple_of(1000)
    }
}

/// The engine's state: what the firmware is doing.
#[derive(Debug, Default)]
pub(super) struct Engine {
    cores: [Core; CORES],
    /// The firmware has been started (a start event was taken).
    started: bool,
    /// Device ticks since the last timer event.
    timer: u64,
    poll: Poll,
    diag: Diag,
}

/// The engine's knobs, from the device's properties.
#[derive(Clone, Copy, Debug)]
pub(super) struct Tuning {
    /// USSE instructions per device tick.
    pub ipt: u64,
    /// Device ticks between timer events.
    pub timer_ticks: u64,
}

/// GPU memory through the SGX MMU, with a small translation cache that lives
/// for one slice (the host may rewrite page tables between slices).
struct Mmu<'a> {
    space: &'a AddressSpace,
    directory: u32,
    attrs: MemAttrs,
    tlb: [(u32, u32); 16],
}

impl Mmu<'_> {
    fn phys32(&self, pa: u64) -> MemResult<u32> {
        self.space
            .read(pa, Width::U32, self.attrs)
            .map(|v| v as u32)
    }

    fn translate(&mut self, va: u32) -> MemResult<u64> {
        let page = va >> 12;
        let slot = &mut self.tlb[(page & 15) as usize];
        if slot.0 == page | 1 << 31 {
            let frame = slot.1;
            return Ok(u64::from(frame) | u64::from(va & 0xfff));
        }
        let pde = self.phys32(u64::from(self.directory & !0xfff) + u64::from(va >> 22) * 4)?;
        if pde & 1 == 0 {
            return Err(BusError::Unassigned);
        }
        let pte = self.phys32(u64::from(pde & !0xfff) + u64::from((va >> 12) & 0x3ff) * 4)?;
        if pte & 1 == 0 {
            return Err(BusError::Unassigned);
        }
        let frame = pte & !0xfff;
        self.tlb[(page & 15) as usize] = (page | 1 << 31, frame);
        Ok(u64::from(frame) | u64::from(va & 0xfff))
    }

    fn read(&mut self, va: u32, width: Width) -> MemResult<u32> {
        let pa = self.translate(va)?;
        self.space.read(pa, width, self.attrs).map(|v| v as u32)
    }

    fn write(&mut self, va: u32, width: Width, v: u32) -> MemResult {
        let pa = self.translate(va)?;
        self.space.write(pa, width, u64::from(v), self.attrs)
    }
}

/// The USSE's view of the world for one slice of one task.
struct UsseCtx<'a, 'b> {
    shared: &'a Shared,
    mmu: &'a mut Mmu<'b>,
    poll: &'a mut Poll,
    diag: &'a mut Diag,
    core: usize,
    /// The PC of the instruction executing, for the poll detector.
    pc: u32,
    /// Programs the task started with EMIT.
    emits: Vec<Launch>,
    /// The interrupt may have changed.
    irq: bool,
}

impl UsseCtx<'_, '_> {
    fn first_touch(&mut self, write: bool, offset: u32, v: u32) {
        if self.diag.regs.len() < LOG_REGS && self.diag.regs.insert((write, offset)) {
            self.shared.note(&format!(
                "sgx: core{} pc {:#x} first {} reg {offset:#06x} {} {v:#x}\n",
                self.core,
                self.pc,
                if write { "str" } else { "ldr" },
                if write { "<-" } else { "=" },
            ));
        }
    }
}

fn width(size: Size) -> Width {
    match size {
        Size::Byte => Width::U8,
        Size::Word => Width::U16,
        Size::Dword => Width::U32,
    }
}

impl UsseBus for UsseCtx<'_, '_> {
    fn load(&mut self, addr: u32, size: Size) -> MemResult<u32> {
        self.mmu.read(addr, width(size))
    }

    fn store(&mut self, addr: u32, size: Size, value: u32) -> MemResult {
        self.mmu.write(addr, width(size), value)
    }

    fn fetch(&mut self, addr: u32) -> MemResult<u64> {
        let lo = self.mmu.read(addr, Width::U32)?;
        let hi = self.mmu.read(addr.wrapping_add(4), Width::U32)?;
        Ok(u64::from(hi) << 32 | u64::from(lo))
    }

    fn reg_read(&mut self, index: u32) -> u32 {
        let offset = index.wrapping_mul(4);
        if offset >= super::REGISTER_WINDOW_LEN as u32 {
            self.first_touch(false, offset, 0);
            return 0;
        }
        let stored = self.shared.read_reg(u64::from(offset), false);
        let v = stored | completion(offset);
        self.first_touch(false, offset, v);
        let p = &mut *self.poll;
        if (p.pc, p.offset, p.value) == (self.pc, offset, v) {
            p.count += 1;
        } else {
            *p = Poll {
                pc: self.pc,
                offset,
                value: v,
                count: 1,
            };
        }
        if p.count >= POLL_PATIENCE {
            p.count = 0;
            if self.diag.forced.insert(self.pc) {
                self.shared.note(&format!(
                    "sgx: core{} pc {:#x} poll of reg {offset:#06x} (= {v:#x}) forced to complete\n",
                    self.core, self.pc
                ));
            }
            return !0;
        }
        v
    }

    fn reg_write(&mut self, index: u32, value: u32) {
        let offset = index.wrapping_mul(4);
        self.first_touch(true, offset, value);
        if offset >= super::REGISTER_WINDOW_LEN as u32 {
            return;
        }
        self.irq |= self.shared.usse_write_reg(offset, value);
        if moves_directory(offset) {
            self.mmu.directory = edm_directory(&self.shared.state.lock());
            self.mmu.tlb = [(0, 0); 16];
        }
    }

    fn emit(&mut self, emit: &Emit) -> MemResult {
        if emit.target != 0 {
            return Err(BusError::Unassigned);
        }
        let [ctl, pds, arg] = [emit.sources[0][0], emit.sources[1][0], emit.sources[2][0]];
        self.emits.push(Launch {
            base: pds << 4,
            rows: ctl >> 23 & 0x1f,
            ir: [arg, 0, 0],
            secondary: emit.imm & 0x400 != 0,
            own_base: false,
        });
        Ok(())
    }
}

/// The PDS's view: memory for its code and DMA, the attribute banks, and the
/// tasks it starts.
struct PdsCtx<'a, 'b> {
    mmu: &'a mut Mmu<'b>,
    sa: &'a mut [u32],
    pa: [u32; BANK_SIZE],
    launch: Launch,
    codebases: [u32; 16],
    core: usize,
    tasks: Vec<Task>,
    /// Attribute writes that fell outside a bank.
    lost: u32,
}

impl PdsCtx<'_, '_> {
    fn attr(&mut self, reg: u32, v: u32) {
        let bank: &mut [u32] = if self.launch.secondary {
            self.sa
        } else {
            &mut self.pa
        };
        match bank.get_mut(reg as usize) {
            Some(slot) => *slot = v,
            None => self.lost += 1,
        }
    }
}

impl PdsBus for PdsCtx<'_, '_> {
    fn load(&mut self, addr: u32) -> MemResult<u32> {
        self.mmu.read(addr, Width::U32)
    }

    fn dout(&mut self, out: &Dout) -> MemResult {
        let [w0, w1, _, _] = out.words;
        match out.kind {
            DoutType::DMA => {
                let first = w1 >> 8 & 0xfff;
                for i in 0..=(w1 & 0xff) {
                    let v = self.mmu.read(w0.wrapping_add(4 * i), Width::U32)?;
                    self.attr(first + i, v);
                }
            }
            DoutType::ATTR => self.attr(w1 >> 8 & 0xfff, w0),
            DoutType::USSE => {
                let cb = (w0 & 0xf) as u8;
                let pc = w0 >> 4 & 0xf_ffff;
                let base = self.codebases[usize::from(cb)];
                let mut usse = if self.launch.own_base {
                    Box::new(Usse::new(base.wrapping_add(pc.wrapping_mul(8)), 0))
                } else {
                    Box::new(Usse::new(base, pc))
                };
                usse.primary = self.pa;
                // [G] g85/g86 read as the core number (the trace header and a
                // per-core register check use them).
                usse.special[85 - 64] = self.core as u32;
                usse.special[86 - 64] = self.core as u32;
                self.tasks.push(Task { usse, cb });
            }
            _ => return Err(BusError::Unassigned),
        }
        Ok(())
    }
}

impl Engine {
    /// The register an event of bank `bank` launches, as a [`Launch`].
    fn event(state: &State, bank: u32, ir: [u32; 3]) -> Launch {
        Launch {
            base: state.reg(u64::from(bank + EVENT_PDS)),
            rows: state.reg(u64::from(bank + EVENT_ROWS)),
            ir,
            secondary: false,
            own_base: bank == MASTER_BANK,
        }
    }

    /// Take the next event for `core`, if it is idle and one is waiting.
    fn next_event(&mut self, shared: &Shared, core: usize) {
        if !self.cores[core].idle() {
            return;
        }
        let mut s = shared.state.lock();
        // Core 0 serves its own bank; core 1 its own and the master's.
        let banks: &[(usize, u32)] = if core == 0 {
            &[(1, core_bank(0))]
        } else {
            &[(2, core_bank(1)), (0, MASTER_BANK)]
        };
        for &(slot, bank) in banks {
            let launch = if s.hw_starts[slot] {
                s.hw_starts[slot] = false;
                self.started = true;
                Some(Engine::event(&s, bank, [0; 3]))
            } else if s.hw_kicks[slot] > 0 {
                s.hw_kicks[slot] -= 1;
                Some(Engine::event(&s, bank, [0, IR1_KICK, 0]))
            } else {
                None
            };
            if let Some(l) = launch {
                self.cores[core].launches.push_back(l);
                return;
            }
        }
        if core == 0 && self.cores[0].timer {
            self.cores[0].timer = false;
            let l = Engine::event(&s, core_bank(0), [IR0_TIMER, 0, 0]);
            self.cores[0].launches.push_back(l);
        }
    }

    /// Run a slice of `ticks` device ticks.
    pub(super) fn run(&mut self, shared: &Shared, ticks: u64, tuning: Tuning) {
        if core::mem::take(&mut shared.state.lock().hw_reset) {
            if shared.log.is_some() {
                let busy = self.cores.iter().filter(|c| !c.idle()).count();
                shared.note(&format!("sgx: soft reset ({busy} busy cores dropped)\n"));
            }
            // The attribute stores survive: a warm restart skips reloading
            // the secondary attributes (`EVENTS.md` §2) \[G\].
            let mut fresh = Engine {
                diag: core::mem::take(&mut self.diag),
                ..Engine::default()
            };
            for (new, old) in fresh.cores.iter_mut().zip(&mut self.cores) {
                new.sa = core::mem::take(&mut old.sa);
                new.persist = old.persist;
            }
            *self = fresh;
        }
        let (directory, codebases, timer_on) = {
            let s = shared.state.lock();
            let mut cb = [0u32; 16];
            for (i, c) in cb.iter_mut().enumerate() {
                *c = (s.reg(u64::from(CODE_BASE) + 4 * i as u64) & 0x03ff_ffff) << 6;
            }
            (
                edm_directory(&s),
                cb,
                s.reg(u64::from(TIMER_CTL)) & TIMER_PERIOD != 0 && self.started,
            )
        };
        // [G] The timer only runs once the firmware has been started: an
        // event before that would run the boot program again.
        if timer_on && tuning.timer_ticks != 0 {
            self.timer += ticks;
            if self.timer >= tuning.timer_ticks {
                self.timer %= tuning.timer_ticks;
                self.cores[0].timer = true;
            }
        }
        let Some(space) = shared.space() else {
            return;
        };
        let mut mmu = Mmu {
            space: &space,
            directory,
            attrs: MemAttrs {
                requester: *shared.requester.lock(),
                ..MemAttrs::DEFAULT
            },
            tlb: [(0, 0); 16],
        };
        let mut budget = ticks.saturating_mul(tuning.ipt);
        let mut irq = false;
        while budget > 0 {
            let mut progress = false;
            for core in 0..CORES {
                self.next_event(shared, core);
                let used = self.service(shared, &mut mmu, core, codebases, budget, &mut irq);
                progress |= used > 0;
                budget = budget.saturating_sub(used);
            }
            if !progress {
                break;
            }
        }
        if irq {
            shared.drive();
        }
    }

    /// One step of work on `core`: a whole PDS program, or a task slice.
    /// Returns the instructions it retired.
    fn service(
        &mut self,
        shared: &Shared,
        mmu: &mut Mmu<'_>,
        core: usize,
        codebases: [u32; 16],
        budget: u64,
        irq: &mut bool,
    ) -> u64 {
        if let Some(launch) = self.cores[core].launches.pop_front() {
            return self.run_pds(shared, mmu, core, launch, codebases);
        }
        if self.cores[core].tasks.is_empty() {
            return 0;
        }
        let Engine {
            cores, poll, diag, ..
        } = self;
        let c = &mut cores[core];
        let mut task = c.tasks.pop_front().expect("checked above");
        task.usse.secondary.copy_from_slice(&c.sa);
        let mut ctx = UsseCtx {
            shared,
            mmu,
            poll,
            diag,
            core,
            pc: 0,
            emits: Vec::new(),
            irq: false,
        };
        let before = task.usse.retired;
        let mut stop = None;
        for _ in 0..budget.min(1 << 16) {
            ctx.pc = task.usse.pc;
            if let Err(s) = task.usse.step(&mut ctx) {
                stop = Some(s);
                break;
            }
        }
        let used = task.usse.retired.wrapping_sub(before).max(1);
        // A task that runs on and on is usually spinning on something not
        // modelled: say where, now and then.
        if before >> 22 != task.usse.retired >> 22 && Diag::thin(&mut ctx.diag.stops) {
            shared.note(&format!(
                "sgx: core{core} task still running at {:#010x} after {} instructions\n",
                task.usse.address_of(task.usse.pc),
                task.usse.retired
            ));
        }
        let emits = core::mem::take(&mut ctx.emits);
        *irq |= ctx.irq;
        c.sa.copy_from_slice(&task.usse.secondary);
        // [G] Every EMIT-started program runs on core 0: the slave's EMITs
        // name main-image entries, which need core 0's secondary attributes.
        cores[0].launches.extend(emits);
        let keep = match stop {
            None => true,
            Some(s) => self.after_stop(shared, core, &mut task, s, codebases),
        };
        if keep {
            self.cores[core].tasks.push_front(task);
        }
        used
    }

    /// What a stop means for the task; true to keep it.
    fn after_stop(
        &mut self,
        shared: &Shared,
        core: usize,
        task: &mut Task,
        stop: Stop,
        codebases: [u32; 16],
    ) -> bool {
        let base = codebases[usize::from(task.cb)];
        // A PC counted from the code-base register, as the task counts it.
        let local = |pc: u32| {
            base.wrapping_add(pc.wrapping_mul(8))
                .wrapping_sub(task.usse.code_base)
                / 8
        };
        match stop {
            Stop::Budget { .. } => true,
            Stop::End { .. } | Stop::Phase { next: Some(0), .. } => false,
            Stop::Phase { next: Some(n), .. } => {
                task.usse.pc = local(n);
                true
            }
            Stop::Phase { next: None, .. } => {
                let resume = shared
                    .state
                    .lock()
                    .reg(u64::from(core_bank(core) + RESUME_PC));
                task.usse.pc = local(resume);
                true
            }
            // [G] op205 (`fd2…`) sits at the head of a routine that then
            // flushes and polls; nothing it could do is visible here.
            Stop::Unimplemented { pc, word, .. } if word >> 52 == 0xfd2 => {
                task.usse.pc = pc.wrapping_add(1);
                true
            }
            other => {
                if Diag::thin(&mut self.diag.stops) {
                    let pc = match other {
                        Stop::Unimplemented { pc, .. }
                        | Stop::Undecodable { pc, .. }
                        | Stop::BadRegister { pc, .. }
                        | Stop::Fault { pc, .. } => pc,
                        _ => task.usse.pc,
                    };
                    let addr = task.usse.address_of(pc);
                    let word = match other {
                        Stop::Unimplemented { word, .. }
                        | Stop::Undecodable { word, .. }
                        | Stop::BadRegister { word, .. } => word,
                        _ => 0,
                    };
                    shared.note(&format!(
                        "sgx: core{core} task killed at {addr:#010x}: {other:x?} [{}]\n",
                        usse::disassemble(word, addr, task.usse.code_base)
                    ));
                }
                false
            }
        }
    }

    /// Run one PDS program to its HALT.
    fn run_pds(
        &mut self,
        shared: &Shared,
        mmu: &mut Mmu<'_>,
        core: usize,
        launch: Launch,
        codebases: [u32; 16],
    ) -> u64 {
        let c = &mut self.cores[core];
        let mut pds = Pds::default();
        pds.ds[0][pds::PERSISTENT..].copy_from_slice(&c.persist[0]);
        pds.ds[1][pds::PERSISTENT..].copy_from_slice(&c.persist[1]);
        let mut ctx = PdsCtx {
            mmu,
            sa: &mut c.sa,
            pa: [0; BANK_SIZE],
            launch,
            codebases,
            core,
            tasks: Vec::new(),
            lost: 0,
        };
        let stop = match pds.launch(&mut ctx, launch.base, launch.rows, launch.ir) {
            Ok(()) => pds.run(&mut ctx, PDS_BUDGET),
            Err(stop) => stop,
        };
        let (tasks, lost) = (core::mem::take(&mut ctx.tasks), ctx.lost);
        c.persist[0].copy_from_slice(&pds.ds[0][pds::PERSISTENT..]);
        c.persist[1].copy_from_slice(&pds.ds[1][pds::PERSISTENT..]);
        let counter = if launch.ir == [IR0_TIMER, 0, 0] {
            &mut self.diag.timers
        } else {
            &mut self.diag.launches
        };
        if shared.log.is_some() && Diag::thin(counter) {
            let entries: Vec<String> = tasks
                .iter()
                .map(|t| format!("{:#010x}", t.usse.address_of(t.usse.pc)))
                .collect();
            let halted = matches!(stop, pds::Stop::Halt { .. });
            shared.note(&format!(
                "sgx: core{core} pds {:#010x}/{} ir {:x?}{} -> use {}{}{}\n",
                launch.base,
                launch.rows,
                launch.ir,
                if launch.secondary { " sec" } else { "" },
                if entries.is_empty() {
                    String::from("-")
                } else {
                    entries.join(",")
                },
                if halted {
                    String::new()
                } else {
                    format!(" STOP {stop:x?}")
                },
                if lost != 0 {
                    format!(" ({lost} attribute writes out of range)")
                } else {
                    String::new()
                },
            ));
        }
        self.cores[core].tasks.extend(tasks);
        pds.retired.max(1)
    }

    /// Whether every core is idle (for tests).
    #[cfg(test)]
    pub(super) fn idle(&self) -> bool {
        self.cores.iter().all(Core::idle)
    }

    /// A core's secondary attributes (for tests).
    #[cfg(test)]
    pub(super) fn sa(&self, core: usize) -> &[u32] {
        &self.cores[core].sa
    }

    // -- snapshots ----------------------------------------------------------

    pub(super) fn save(&self, w: &mut ChunkWriter<'_>) -> Result<()> {
        w.write_bool(self.started)?;
        w.write_u64(self.timer)?;
        let p = self.poll;
        for v in [p.pc, p.offset, p.value, p.count] {
            w.write_u32(v)?;
        }
        for c in &self.cores {
            words_out(w, &c.sa)?;
            words_out(w, &c.persist[0])?;
            words_out(w, &c.persist[1])?;
            w.write_bool(c.timer)?;
            w.write_seq_len(c.launches.len() as u64)?;
            for l in &c.launches {
                for v in [l.base, l.rows, l.ir[0], l.ir[1], l.ir[2]] {
                    w.write_u32(v)?;
                }
                w.write_bool(l.secondary)?;
                w.write_bool(l.own_base)?;
            }
            w.write_seq_len(c.tasks.len() as u64)?;
            for t in &c.tasks {
                w.write_u8(t.cb)?;
                usse_out(w, &t.usse)?;
            }
        }
        Ok(())
    }

    pub(super) fn load(r: &mut ChunkReader<'_>) -> Result<Engine> {
        let started = r.read_bool()?;
        let mut e = Engine {
            started,
            timer: r.read_u64()?,
            ..Engine::default()
        };
        e.poll = Poll {
            pc: r.read_u32()?,
            offset: r.read_u32()?,
            value: r.read_u32()?,
            count: r.read_u32()?,
        };
        for c in &mut e.cores {
            words_in(r, &mut c.sa)?;
            words_in(r, &mut c.persist[0])?;
            words_in(r, &mut c.persist[1])?;
            c.timer = r.read_bool()?;
            let n = r.read_seq_len(22)?;
            for _ in 0..n {
                let mut v = [0u32; 5];
                for x in &mut v {
                    *x = r.read_u32()?;
                }
                c.launches.push_back(Launch {
                    base: v[0],
                    rows: v[1],
                    ir: [v[2], v[3], v[4]],
                    secondary: r.read_bool()?,
                    own_base: r.read_bool()?,
                });
            }
            let n = r.read_seq_len(1024)?;
            for _ in 0..n {
                let cb = r.read_u8()?;
                let usse = usse_in(r)?;
                c.tasks.push_back(Task { usse, cb });
            }
        }
        Ok(e)
    }
}

fn words_out(w: &mut ChunkWriter<'_>, v: &[u32]) -> Result<()> {
    v.iter().try_for_each(|x| w.write_u32(*x))
}

fn words_in(r: &mut ChunkReader<'_>, v: &mut [u32]) -> Result<()> {
    for x in v {
        *x = r.read_u32()?;
    }
    Ok(())
}

/// A task's USSE state. The secondary bank is the core's and is copied in
/// before every slice, so it is not part of the task.
fn usse_out(w: &mut ChunkWriter<'_>, u: &Usse) -> Result<()> {
    w.write_u32(u.code_base)?;
    w.write_u32(u.pc)?;
    words_out(w, &u.temp)?;
    words_out(w, &u.output)?;
    words_out(w, &u.primary)?;
    words_out(w, &u.special)?;
    words_out(w, &u.constants)?;
    words_out(w, &u.index)?;
    for p in u.preds {
        w.write_bool(p)?;
    }
    w.write_u32(u.link)?;
    for s in u.smlsi {
        w.write_i32(s)?;
    }
    w.write_bool(u.next_phase.is_some())?;
    w.write_u32(u.next_phase.unwrap_or(0))?;
    w.write_u64(u.retired)
}

fn usse_in(r: &mut ChunkReader<'_>) -> Result<Box<Usse>> {
    let code_base = r.read_u32()?;
    let pc = r.read_u32()?;
    let mut u = Box::new(Usse::new(code_base, pc));
    words_in(r, &mut u.temp)?;
    words_in(r, &mut u.output)?;
    words_in(r, &mut u.primary)?;
    words_in(r, &mut u.special)?;
    words_in(r, &mut u.constants)?;
    words_in(r, &mut u.index)?;
    for p in &mut u.preds {
        *p = r.read_bool()?;
    }
    u.link = r.read_u32()?;
    for s in &mut u.smlsi {
        *s = r.read_i32()?;
    }
    let has = r.read_bool()?;
    let next = r.read_u32()?;
    u.next_phase = has.then_some(next);
    u.retired = r.read_u64()?;
    Ok(u)
}

/// The engine's state, compared field by field (for the round-trip test).
#[cfg(test)]
pub(super) fn fingerprint(e: &Engine) -> Vec<u8> {
    let mut shape = crate::core::state::MachineShape::new();
    shape.add_device("e", "e").unwrap();
    let mut wr = crate::core::state::StateWriter::new(shape);
    {
        let mut chunk = wr.chunk("e", "e", 1).unwrap();
        e.save(&mut chunk).unwrap();
    }
    wr.to_vec().unwrap()
}
