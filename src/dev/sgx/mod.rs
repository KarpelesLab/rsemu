//! A PowerVR SGX stand-in: the register block, and the part of the GPU's
//! own firmware — its "microkernel" — that the host driver talks to through
//! shared memory. Enough for the driver to start the core, check its build
//! and complete its kernel commands. A TA kick is drawn before it is
//! completed (the `render` module; `render = "on"`, the default): its control stream,
//! vertex and pixel programs run on the [`usse`] and [`pds`] interpreters and
//! the result lands in the render target. Transfer and 2D kicks are
//! completed without drawing.
//!
//! # What the host driver expects
//!
//! An SGX runs firmware the host loads at start-up, and from then on the two
//! sides meet in memory, not in registers:
//!
//! * a **host-control** block, whose word 0 bit 0 the firmware sets once it
//!   is running (the host polls it after every start);
//! * a **kernel command ring** (the *kernel CCB*): 256 slots of 32 bytes,
//!   with a control pair — write offset (host), read offset (firmware). The
//!   host fills a slot, bumps the write offset, and kicks register `0x8ac8`;
//!   the firmware consumes slots and advances the read offset;
//! * a **TA/3D control** block holding the addresses of both. Its word 0 is
//!   its own address, which is how it is found here: the host's set-up code
//!   patches it into the firmware image, not into any register, so the
//!   stand-in scans forward from the *kicker* word (whose address the host
//!   does put in register `0xac4`) for a block that names itself.
//!
//! Every address in that memory is a GPU virtual address, translated through
//! the SGX's own two-level MMU: the directory at register `0xc50` (the
//! kernel's), indexed by bits 31:22; tables of 4 KiB pages indexed by bits
//! 21:12; bit 0 of each entry is valid.
//!
//! # Commands
//!
//! Slot word 0 names the command; the stand-in acts on three and completes
//! all of them:
//!
//! | Word 0 | Command | Done here |
//! | --- | --- | --- |
//! | `0x220` | get misc info | fills the buffer at word 3 with this build's identity and structure sizes, then sets its word 0 bit 0 |
//! | `0xd1` | power | word 3 = 1 (off) sets host-control word 1 bit 3; 2 (idle) sets bit 2 |
//! | `0x1a7` | clean-up | sets host-control word 2 bit 0 |
//! | `0x176`, `0x157`, `0x132` | TA/3D, transfer, 2D kick | completes the work queued on the context (word 3) — below |
//!
//! # Kicks: completed, not drawn
//!
//! A kick names a hardware context; its commands sit on the context's own
//! ring between the read offset (the firmware's) and the write offset (the
//! host's). Each command lists the sync objects it reads and writes and the
//! status words its client polls, and the clients' waits are exact matches
//! with a half-second patience, after which they ask the driver to reset the
//! core. So each command is finished at once: a source's `ReadOpsComplete`
//! and a destination's `WriteOpsComplete` become the pending value the
//! command snapshotted plus one, status words get their values (and a TA
//! command's render-details and destination-list statuses go back to 0),
//! and the read offset catches up. Nothing is rendered.
//!
//! The misc-info answer is what the host checks the firmware against: DDK
//! 1.7.17 build 2145535, build options `0x1032241c`, and the sizes of the
//! shared structures, which must equal what the host was built with.
//!
//! After consuming commands the firmware raises event bit 14 in
//! `EVENT_STATUS` (`0x12c`); `irq` is up while an enabled event is pending
//! (`0x12c & 0x130` bit 14, or `0x118 & 0x110` bits 10–11), and a write to
//! `0x134` / `0x114` clears what it names.
//!
//! # Registers
//!
//! Plain storage, except: `0x024` (and the per-core copies at `0x8024`,
//! `0xc024`) reads the core revision; `0xb10` counts up on every read, which
//! the driver's lock-up watchdog takes as a core that is alive; `0x8ac0` and
//! `0x8ac8` are the start and command kicks.
//!
//! Once started, the stand-in also keeps host-control word `0x7c` — the
//! firmware's heartbeat, which the driver's lock-up timer counts down and
//! acts on at zero — written, on every scheduler slice, as live firmware
//! does.
//!
//! Work is done on the device's clock, never inside the register write: a
//! kick only marks it pending.
//!
//! # Sources
//!
//! No SGX documentation is public, and no driver source was consulted. All
//! of the above is read off the navi's own driver binaries — the kernel
//! module and the user-space set-up library — disassembled as data: the
//! register offsets and values they write and poll, the command ring they
//! fill, the structure sizes and build constants they pass and compare.
//!
//! # The shader engine, and the hardware mode
//!
//! [`usse`] holds the instruction set of the SGX's programmable core, the
//! USSE2, and [`pds`] that of the data sequencer that starts its tasks —
//! decoder, disassembler and interpreter each. With `mode = "hw"` the device
//! stops answering the host itself and runs the GPU's real firmware on them
//! instead (`hw.rs`): kicks raise events, events run the firmware's PDS
//! programs, and those start the microkernel, which then does through
//! memory and registers everything described above. The fixed-function
//! pipeline is not modelled; work the firmware starts on it completes at
//! once. That mode is bring-up work, so the stand-in stays the default.

mod hw;
pub mod pds;
pub mod raster;
mod render;
pub mod usse;

use alloc::boxed::Box;
use alloc::string::{String, ToString};
use alloc::sync::{Arc, Weak};
use alloc::vec::Vec;

use crate::core::device::{Device, DeviceClass, PropertySpec, RealizeCtx, ResetKind};
use crate::core::error::{BusError, Error, Result};
use crate::core::props::{Props, ValueKind};
use crate::core::space::{
    AccessConstraints, AddressSpace, MemAttrs, MemOps, MemResult, Region, RegionRef, RequesterId,
};
use crate::core::state::{ChunkReader, ChunkWriter, Sink, Source};
use crate::core::sync::{LockRank, Mutex};
use crate::core::value::{Endian, Width};
use crate::core::wire::{Level, WireSource};
use crate::machine::realize::{BindCtx, Instance};

/// The class name a machine description writes.
pub const CLASS_NAME: &str = "pvr.sgx";

/// The snapshot chunk version. Bump with the encoding, never on its own.
const STATE_VERSION: u32 = 2;

/// How much address space the register block answers.
pub const REGISTER_WINDOW_LEN: u64 = 0x1_0000;

/// The interrupt output.
pub const IRQ_PIN: &str = "irq";

const WORDS: usize = (REGISTER_WINDOW_LEN / 4) as usize;

const CORE_REVISION: [u64; 3] = [0x024, 0x8024, 0xc024];
const EVENT_STATUS2: u64 = 0x118;
const EVENT_ENABLE2: u64 = 0x110;
const EVENT_CLEAR2: u64 = 0x114;
const EVENT_STATUS: u64 = 0x12c;
const EVENT_ENABLE: u64 = 0x130;
const EVENT_CLEAR: u64 = 0x134;
const KICKER_ADDR: u64 = 0xac4;
const ALIVE_COUNTER: u64 = 0xb10;
const KERNEL_DIRECTORY: u64 = 0xc50;
const START_KICK: u64 = 0x8ac0;
const COMMAND_KICK: u64 = 0x8ac8;

/// The firmware-to-host event.
const EVENT_UKERNEL: u32 = 1 << 14;
const EVENT2_MASK: u32 = 0xc00;

/// TA/3D control: where the host-control block, the ring's control pair and
/// the ring itself are.
const TA3D_HOST_CTL: u32 = 0x04;
const TA3D_CCB_CTL: u32 = 0x08;
const TA3D_CCB_RING: u32 = 0xc4;
/// How far past the kicker to look for the TA/3D control block.
const TA3D_SCAN: u32 = 0x1_0000;

/// Host-control word the firmware refreshes to show it is alive.
const HOST_HEARTBEAT: u32 = 0x7c;
/// What it writes there.
const HEARTBEAT: u32 = 0x40;

const CCB_SLOTS: u32 = 256;
const CCB_SLOT: u32 = 32;

const CMD_MISC_INFO: u32 = 0x220;
const CMD_POWER: u32 = 0xd1;
const CMD_CLEANUP: u32 = 0x1a7;
const CMD_KICK_TA: u32 = 0x176;
const CMD_KICK_TRANSFER: u32 = 0x157;
const CMD_KICK_2D: u32 = 0x132;

/// A hardware context: its command ring's base and control pair.
/// A context's page directory, physical \[strong\]: the TA command and all it
/// names translate through it.
const CTX_DIRECTORY: u32 = 0x08;
const CTX_CCB_BASE: u32 = 0x0c;
const CTX_CCB_CTL: u32 = 0x10;
/// A context's command ring (a command never wraps: the ring has an
/// overflow area past its end).
const CONTEXT_CCB_SIZE: u32 = 0x1_0000;
/// TA command flag: the TA/3D dependency sync is the command's own.
const TA_DEPENDENCY: u32 = 1 << 9;

/// The misc-info answer: offset, value. Word 4 and word 6 (the core
/// revision, hardware and software) are added from the `revision` prop.
const MISC_INFO: &[(u32, u32)] = &[
    (0x0c, 0x0001_0711), // DDK 1.7.17
    (0x10, 0x0020_bcff), // build 2145535
    (0x1c, 0x1032_241c), // firmware build options
    // The shared structures' sizes, in the host's order.
    (0x28, 0xdc),  // 2D command
    (0x2c, 0x64),  // 2D command, shared
    (0x30, 0x288), // TA command
    (0x34, 0x1e0), // TA command, shared
    (0x38, 0x4c0), // transfer command
    (0x3c, 0xdc),  // transfer command, shared
    (0x40, 0xe4),  // 3D registers
    (0x44, 0x3c),  // parameter-buffer descriptor
    (0x48, 0x68),  // render context
    (0x4c, 0x1ec), // render details
    (0x50, 0x64),  // render-target data
    (0x54, 0x7c),  // render-target data set
    (0x58, 0xb4),  // transfer context
    (0x5c, 0x80),  // host control
    (0x60, 0x20),  // command slot
];

/// The guest-visible state.
#[derive(Debug, Clone, PartialEq, Eq)]
struct State {
    regs: Vec<u32>,
    start_pending: bool,
    kick_pending: bool,
    /// The TA/3D control block's GPU address, once found.
    ta3d: Option<u32>,
    /// Hardware mode: a start raised in each bank (master, core 0, core 1)
    /// and not yet taken.
    hw_starts: [bool; 3],
    /// Hardware mode: command kicks per bank not yet taken.
    hw_kicks: [u32; 3],
    /// Hardware mode: the host soft-reset the core; the engine drops what it
    /// was running at its next slice.
    hw_reset: bool,
}

impl Default for State {
    fn default() -> State {
        State {
            regs: alloc::vec![0; WORDS],
            start_pending: false,
            kick_pending: false,
            ta3d: None,
            hw_starts: [false; 3],
            hw_kicks: [0; 3],
            hw_reset: false,
        }
    }
}

impl State {
    fn reg(&self, offset: u64) -> u32 {
        self.regs[(offset / 4) as usize]
    }

    fn reg_mut(&mut self, offset: u64) -> &mut u32 {
        &mut self.regs[(offset / 4) as usize]
    }

    fn irq(&self) -> bool {
        self.reg(EVENT_STATUS) & self.reg(EVENT_ENABLE) & EVENT_UKERNEL != 0
            || self.reg(EVENT_STATUS2) & self.reg(EVENT_ENABLE2) & EVENT2_MASK != 0
    }

    /// Hardware mode: a write that raises an event (a start or command kick
    /// in one of the banks); true if it was one.
    fn hw_event(&mut self, offset: u64, v: u32) -> bool {
        let slot = match offset & !0x3fff {
            0x0000 => 0,
            0x8000 => 1,
            0xc000 => 2,
            _ => return false,
        };
        match (offset & 0x3fff) as u32 {
            hw::START => {
                if v & 1 != 0 {
                    self.hw_starts[slot] = true;
                }
                true
            }
            hw::KICK => {
                if v & 1 != 0 {
                    self.hw_kicks[slot] = self.hw_kicks[slot].saturating_add(1);
                }
                true
            }
            _ => false,
        }
    }
}

#[derive(Debug)]
struct Shared {
    revision: u32,
    state: Mutex<State>,
    irq: Mutex<Option<WireSource>>,
    bus: Mutex<Option<Weak<AddressSpace>>>,
    requester: Mutex<RequesterId>,
    /// Where every command the firmware side consumes is reported, if
    /// anywhere (`log = "port"`).
    log: Option<Arc<dyn crate::host::chardev::CharDevice>>,
    /// The hardware mode's knobs; `None` in the stand-in mode.
    hw: Option<hw::Tuning>,
    /// The stand-in draws each TA command before completing it (`render`).
    render: bool,
    /// The hardware mode's engine. Taken out for the length of a slice, so
    /// that no lock is held while the firmware reaches memory.
    engine: Mutex<Option<Box<hw::Engine>>>,
}

impl Shared {
    /// The sixteen USE code bases, as GPU addresses (`(reg & 0x3ff_ffff) << 6`).
    fn code_bases(&self) -> [u32; 16] {
        let s = self.state.lock();
        core::array::from_fn(|i| (s.reg(0xa0c + 4 * i as u64) & 0x03ff_ffff) << 6)
    }

    fn note(&self, text: &str) {
        if let Some(log) = &self.log {
            log.write(text.as_bytes());
        }
    }

    fn drive(&self) {
        let level = self.state.lock().irq();
        let out = self.irq.lock().clone();
        if let Some(out) = out {
            out.set(Level::from_bool(level));
        }
    }

    fn read_reg(&self, offset: u64, debug: bool) -> u32 {
        let mut s = self.state.lock();
        if CORE_REVISION.contains(&offset) {
            return self.revision;
        }
        let v = s.reg(offset);
        if offset == ALIVE_COUNTER && !debug {
            *s.reg_mut(offset) = v.wrapping_add(1);
        }
        v
    }

    fn write_reg(&self, offset: u64, v: u32) {
        let mut s = self.state.lock();
        if self.hw.is_some() && s.hw_event(offset, v) {
            return;
        }
        // [G] The host's soft reset (it writes 0x7ff to 0x4080 before every
        // start, and resets the core this way on recovery) stops whatever
        // the firmware was running. The firmware's own writes there reset
        // single units and do not count.
        if self.hw.is_some() && matches!(offset, 0x0080 | 0x4080) && v != 0 {
            s.hw_reset = true;
            s.hw_kicks = [0; 3];
            s.hw_starts = [false; 3];
        }
        match offset {
            EVENT_CLEAR => *s.reg_mut(EVENT_STATUS) &= !v,
            EVENT_CLEAR2 => *s.reg_mut(EVENT_STATUS2) &= !v,
            START_KICK if v & 1 != 0 => s.start_pending = true,
            COMMAND_KICK if v & 1 != 0 => s.kick_pending = true,
            _ => *s.reg_mut(offset) = v,
        }
    }

    /// A register write by the firmware (`STR`). Returns whether the
    /// interrupt may have changed.
    ///
    /// The firmware raises the host's event by writing its bit to the status
    /// register, so a firmware write there sets bits rather than replacing
    /// the word (it writes `0x4000` alone) \[G\]; clears are as the host's.
    fn usse_write_reg(&self, offset: u32, v: u32) -> bool {
        let offset = u64::from(offset);
        let mut s = self.state.lock();
        if s.hw_event(offset, v) {
            return false;
        }
        match offset {
            EVENT_STATUS => *s.reg_mut(EVENT_STATUS) |= v,
            EVENT_CLEAR => *s.reg_mut(EVENT_STATUS) &= !v,
            EVENT_STATUS2 => *s.reg_mut(EVENT_STATUS2) |= v,
            EVENT_CLEAR2 => *s.reg_mut(EVENT_STATUS2) &= !v,
            _ => {
                *s.reg_mut(offset) = v;
                return false;
            }
        }
        true
    }

    fn space(&self) -> Option<Arc<AddressSpace>> {
        self.bus.lock().as_ref().and_then(Weak::upgrade)
    }

    /// Hardware mode: run the firmware for `ticks` device ticks.
    fn run_hw(&self, ticks: u64, tuning: hw::Tuning) {
        let Some(mut engine) = self.engine.lock().take() else {
            return;
        };
        engine.run(self, ticks, tuning);
        *self.engine.lock() = Some(engine);
    }

    /// Do whatever a kick left pending.
    fn run(&self) {
        let (start, kick, directory, kicker, running) = {
            let mut s = self.state.lock();
            let work = (s.start_pending, s.kick_pending);
            s.start_pending = false;
            s.kick_pending = false;
            (
                work.0,
                work.1,
                s.reg(KERNEL_DIRECTORY),
                s.reg(KICKER_ADDR),
                s.ta3d.is_some(),
            )
        };
        if !start && !kick && !running {
            return;
        }
        let Some(space) = self.space() else {
            return;
        };
        let gpu = Gpu {
            space: &space,
            directory,
            attrs: MemAttrs {
                requester: *self.requester.lock(),
                ..MemAttrs::DEFAULT
            },
        };
        if start {
            let ta3d = find_ta3d(&gpu, kicker);
            self.state.lock().ta3d = ta3d;
            if let Some(ta3d) = ta3d
                && let Some(host) = gpu.read(ta3d + TA3D_HOST_CTL)
            {
                gpu.update(host, |v| v | 1);
            }
        }
        let ta3d = self.state.lock().ta3d;
        // The firmware's heartbeat: the host's lock-up timer counts this
        // word down while it stays put and resets the core when it reaches
        // zero, so running firmware keeps writing it.
        if let Some(ta3d) = ta3d
            && let Some(host) = gpu.read(ta3d + TA3D_HOST_CTL)
        {
            gpu.write(host + HOST_HEARTBEAT, HEARTBEAT);
        }
        let mut consumed = false;
        if kick && let Some(ta3d) = ta3d {
            consumed = self.drain(&gpu, ta3d);
        }
        if consumed {
            *self.state.lock().reg_mut(EVENT_STATUS) |= EVENT_UKERNEL;
            self.drive();
        }
    }

    /// Consume every slot between the read and write offsets.
    fn drain(&self, gpu: &Gpu<'_>, ta3d: u32) -> bool {
        let (Some(host), Some(ctl), Some(ring)) = (
            gpu.read(ta3d + TA3D_HOST_CTL),
            gpu.read(ta3d + TA3D_CCB_CTL),
            gpu.read(ta3d + TA3D_CCB_RING),
        ) else {
            return false;
        };
        let (Some(write), Some(mut read)) = (gpu.read(ctl), gpu.read(ctl + 4)) else {
            return false;
        };
        let write = write % CCB_SLOTS;
        let mut any = false;
        while read % CCB_SLOTS != write {
            let slot = ring + (read % CCB_SLOTS) * CCB_SLOT;
            let word = |i: u32| gpu.read(slot + 4 * i).unwrap_or(0);
            if self.log.is_some() {
                self.note(&alloc::format!(
                    "sgx: slot {read:3} cmd {:#05x} [{:#x} {:#x} {:#x} {:#x}]\n",
                    word(0),
                    word(2),
                    word(3),
                    word(4),
                    word(5)
                ));
            }
            match word(0) {
                CMD_MISC_INFO => {
                    let info = word(3);
                    for &(off, v) in MISC_INFO {
                        gpu.write(info + off, v);
                    }
                    gpu.write(info + 0x04, self.revision);
                    gpu.write(info + 0x18, self.revision);
                    gpu.update(info, |v| v | 1);
                }
                CMD_POWER => match word(3) {
                    1 => gpu.update(host + 4, |v| v | 1 << 3),
                    2 => gpu.update(host + 4, |v| v | 1 << 2),
                    _ => {}
                },
                CMD_CLEANUP => gpu.update(host + 8, |v| v | 1),
                CMD_KICK_TA | CMD_KICK_TRANSFER | CMD_KICK_2D => {
                    let code_bases = self.code_bases();
                    let mut draw = |cmd: u32| {
                        if !self.render {
                            return;
                        }
                        // The command and what it points at are in the
                        // issuing context's address space.
                        let Some(directory) = gpu.read(word(3) + CTX_DIRECTORY) else {
                            return;
                        };
                        let app = Gpu { directory, ..*gpu };
                        let done = render::render_ta(&app, cmd, code_bases);
                        if self.log.is_some() {
                            self.note(&alloc::format!("sgx:   render {done:x?}\n"));
                        }
                    };
                    let (from, to) = complete_kick(gpu, word(0), word(3), &mut draw);
                    if self.log.is_some() {
                        self.note(&alloc::format!(
                            "sgx:   context {:#x} ring {from:#x} -> {to:#x}\n",
                            word(3)
                        ));
                    }
                }
                _ => {}
            }
            read = (read + 1) % CCB_SLOTS;
            gpu.write(ctl + 4, read);
            any = true;
        }
        any
    }
}

/// Finish, without drawing anything, the work a kick put on a context's
/// command ring: every sync object the commands name is advanced as the
/// firmware would on completion, every status word the host waits on is
/// written, and the ring's read offset catches up with its write offset.
///
/// A sync object's Complete counter is set to the Pending value the command
/// snapshotted, plus one — never "Complete = Pending" wholesale, because the
/// host's waits are exact matches and CPU-side operations move the counters
/// too.
fn complete_kick(gpu: &Gpu<'_>, kind: u32, context: u32, draw: &mut dyn FnMut(u32)) -> (u32, u32) {
    let (Some(base), Some(ctl)) = (
        gpu.read(context + CTX_CCB_BASE),
        gpu.read(context + CTX_CCB_CTL),
    ) else {
        return (u32::MAX, u32::MAX);
    };
    let (Some(write), Some(mut read)) = (gpu.read(ctl), gpu.read(ctl + 4)) else {
        return (u32::MAX, u32::MAX);
    };
    let from = read;
    // One command per kick for a transfer; the others walk by size. The
    // bound is a guard against a corrupt size, not a limit a guest meets.
    for _ in 0..256 {
        if read == write {
            break;
        }
        let cmd = base.wrapping_add(read);
        let size = match kind {
            CMD_KICK_TA => {
                draw(cmd);
                complete_ta(gpu, cmd)
            }
            CMD_KICK_TRANSFER => {
                complete_transfer(gpu, cmd);
                read = write;
                break;
            }
            _ => complete_2d(gpu, cmd),
        };
        if size == 0 {
            read = write;
            break;
        }
        read = read.wrapping_add(size) & (CONTEXT_CCB_SIZE - 1);
    }
    gpu.write(ctl + 4, read);
    (from, read)
}

/// A sync object read: `ReadOpsComplete` = the snapshotted pending + 1.
fn sync_read(gpu: &Gpu<'_>, pending: u32, complete: u32) {
    if complete != 0 {
        gpu.write(complete, pending.wrapping_add(1));
    }
}

/// A sync object written: `WriteOpsComplete` = the snapshotted pending + 1.
fn sync_write(gpu: &Gpu<'_>, pending: u32, complete: u32) {
    if complete != 0 {
        gpu.write(complete, pending.wrapping_add(1));
    }
}

/// A TA/3D command; returns its size.
fn complete_ta(gpu: &Gpu<'_>, cmd: u32) -> u32 {
    let w = |a: u32| gpu.read(a).unwrap_or(0);
    let size = w(cmd);
    let flags = w(cmd + 0x1c);
    let details = w(cmd + 0x38);
    let dst_list = w(cmd + 0x3c);
    let s = cmd + 0x50;
    // The status words the client polls, {address, value}.
    for (at, count, max) in [(0xc0, w(s + 0x04), 32), (0x1c0, w(s + 0x08), 4)] {
        for i in 0..count.min(max) {
            let (a, v) = (w(s + at + 8 * i), w(s + at + 8 * i + 4));
            if a != 0 {
                gpu.write(a, v);
            }
        }
    }
    sync_read(gpu, w(s + 0x14), w(s + 0x18)); // TA
    sync_read(gpu, w(s + 0x24), w(s + 0x28)); // 3D
    if flags & TA_DEPENDENCY != 0 {
        sync_write(gpu, w(s + 0x34), w(s + 0x38));
    }
    for i in 0..w(s + 0x3c).min(8) {
        let e = s + 0x40 + 16 * i;
        sync_read(gpu, w(e), w(e + 4));
    }
    if dst_list != 0 {
        let status = w(dst_list);
        for i in 0..w(dst_list + 4).min(32) {
            let e = dst_list + 8 + 16 * i;
            sync_write(gpu, w(e + 8), w(e + 12));
        }
        if status != 0 {
            gpu.write(status, 0);
        }
    }
    if details != 0 {
        let status = w(details + 0xe4);
        if status != 0 {
            gpu.write(status, 0);
        }
    }
    size
}

/// A transfer (blit) command.
fn complete_transfer(gpu: &Gpu<'_>, cmd: u32) {
    let w = |a: u32| gpu.read(a).unwrap_or(0);
    let s = cmd + 0xa8;
    // The TA and 3D sync slots overlap destinations 1 and 2: read
    // everything before writing anything.
    let srcs: Vec<(u32, u32)> = (0..w(s).min(5))
        .map(|i| (w(s + 4 + 16 * i), w(s + 8 + 16 * i)))
        .collect();
    let dst_count = w(s + 0x84).min(5);
    let dsts: Vec<(u32, u32)> = (0..dst_count)
        .map(|i| (w(s + 0x88 + 16 * i + 8), w(s + 0x88 + 16 * i + 12)))
        .collect();
    let ta = (w(s + 0x98), w(s + 0x9c));
    let three_d = (w(s + 0xa8), w(s + 0xac));
    // Then the status words the client polls, {address, value} -- the
    // transfer queue's own fence among them.
    let statuses: Vec<(u32, u32)> = (0..w(s + 0xb8).min(8))
        .map(|i| (w(s + 0xbc + 8 * i), w(s + 0xc0 + 8 * i)))
        .collect();
    for (pending, complete) in srcs {
        sync_read(gpu, pending, complete);
    }
    for (pending, complete) in dsts {
        sync_write(gpu, pending, complete);
    }
    if dst_count < 2 {
        sync_write(gpu, ta.0, ta.1);
    }
    if dst_count < 3 {
        sync_write(gpu, three_d.0, three_d.1);
    }
    for (at, value) in statuses {
        if at != 0 {
            gpu.write(at, value);
        }
    }
}

/// A 2D command; returns its size.
fn complete_2d(gpu: &Gpu<'_>, cmd: u32) -> u32 {
    let w = |a: u32| gpu.read(a).unwrap_or(0);
    let size = w(cmd);
    let s = cmd + 0x78;
    for i in 0..w(s).min(3) {
        let e = s + 4 + 16 * i;
        sync_read(gpu, w(e), w(e + 4));
    }
    for e in [s + 0x34, s + 0x44, s + 0x54] {
        sync_write(gpu, w(e + 8), w(e + 12));
    }
    size
}

/// Look forward from the kicker for the block whose word 0 is its own
/// address.
fn find_ta3d(gpu: &Gpu<'_>, kicker: u32) -> Option<u32> {
    let start = kicker & !(CCB_SLOT - 1);
    (0..TA3D_SCAN / CCB_SLOT)
        .map(|i| start.wrapping_add(i * CCB_SLOT))
        .find(|&a| gpu.read(a) == Some(a))
}

/// GPU-virtual memory, through the kernel's directory.
#[derive(Clone, Copy)]
struct Gpu<'a> {
    space: &'a AddressSpace,
    directory: u32,
    attrs: MemAttrs,
}

impl Gpu<'_> {
    fn phys32(&self, pa: u64) -> Option<u32> {
        self.space
            .read(pa, Width::U32, self.attrs)
            .ok()
            .map(|v| v as u32)
    }

    fn translate(&self, va: u32) -> Option<u64> {
        let pde = self.phys32(u64::from(self.directory & !0xfff) + u64::from(va >> 22) * 4)?;
        if pde & 1 == 0 {
            return None;
        }
        let pte = self.phys32(u64::from(pde & !0xfff) + u64::from((va >> 12) & 0x3ff) * 4)?;
        (pte & 1 != 0).then(|| u64::from(pte & !0xfff) | u64::from(va & 0xfff))
    }

    fn read(&self, va: u32) -> Option<u32> {
        self.phys32(self.translate(va)?)
    }

    fn write(&self, va: u32, v: u32) {
        if let Some(pa) = self.translate(va) {
            let _ = self.space.write(pa, Width::U32, u64::from(v), self.attrs);
        }
    }

    fn update(&self, va: u32, f: impl FnOnce(u32) -> u32) {
        if let Some(v) = self.read(va) {
            self.write(va, f(v));
        }
    }
}

#[derive(Debug)]
struct Registers(Arc<Shared>);

impl MemOps for Registers {
    fn read(&self, offset: u64, dst: &mut [u8], attrs: MemAttrs) -> MemResult {
        if dst.len() != 4 {
            return Err(BusError::BadAccess);
        }
        let v = self.0.read_reg(offset & !3, attrs.debug);
        dst.copy_from_slice(&v.to_le_bytes());
        Ok(())
    }

    fn write(&self, offset: u64, src: &[u8], attrs: MemAttrs) -> MemResult {
        if src.len() != 4 || attrs.debug {
            return Err(BusError::BadAccess);
        }
        let v = u32::from_le_bytes([src[0], src[1], src[2], src[3]]);
        self.0.write_reg(offset & !3, v);
        self.0.drive();
        Ok(())
    }

    fn constraints(&self) -> AccessConstraints {
        AccessConstraints::word(Width::U32, Endian::Little)
    }
}

/// The SGX stand-in.
#[derive(Debug)]
pub struct Sgx {
    shared: Arc<Shared>,
    region: RegionRef,
}

impl Sgx {
    /// Build it.
    ///
    /// # Errors
    ///
    /// [`Error::Property`] for a property it does not take.
    pub fn new(props: &Props) -> Result<Sgx> {
        let mut r = props.reader();
        let revision = r.or_range("revision", 0x0001_0205u64, 0..=0xff_ffff)? as u32;
        let log = r.optional_str("log")?.map(ToString::to_string);
        let mode = r.or_str("mode", "standin")?.to_string();
        let ipt = r.or_range("ipt", 256u64, 1..=1 << 20)?;
        let timer_ticks = r.or_range("timer-ticks", 59u64, 0..=1 << 32)?;
        let render = r.or_enum("render", "on", &["on", "off"])? == "on";
        r.finish()?;
        let hw = match mode.as_str() {
            "standin" => None,
            "hw" => Some(hw::Tuning { ipt, timer_ticks }),
            other => {
                return Err(Error::Property(alloc::format!(
                    "mode: `{other}` is neither `standin` nor `hw`"
                )));
            }
        };
        let log = match log {
            Some(name) => Some(crate::host::chardev::ports::attach(props, &name)?
                as Arc<dyn crate::host::chardev::CharDevice>),
            None => None,
        };
        Ok(Sgx::build(revision, log, hw, render))
    }

    /// Build one reporting `revision` (major.minor.maintenance in bits
    /// 23:16, 15:8, 7:0).
    #[must_use]
    pub fn with_revision(revision: u32) -> Sgx {
        Sgx::build(revision, None, None, false)
    }

    fn build(
        revision: u32,
        log: Option<Arc<dyn crate::host::chardev::CharDevice>>,
        hw: Option<hw::Tuning>,
        render: bool,
    ) -> Sgx {
        let shared = Arc::new(Shared {
            log,
            render,
            revision,
            hw,
            engine: Mutex::with_rank(
                LockRank::DEVICE,
                hw.map(|_| Box::new(hw::Engine::default())),
            ),
            state: Mutex::with_rank(LockRank::DEVICE, State::default()),
            irq: Mutex::with_rank(LockRank::LEAF, None),
            bus: Mutex::with_rank(LockRank::LEAF, None),
            requester: Mutex::with_rank(LockRank::LEAF, RequesterId::ANONYMOUS),
        });
        let region: RegionRef = Arc::new(Region::io(
            "pvr.sgx",
            REGISTER_WINDOW_LEN,
            Arc::new(Registers(Arc::clone(&shared))) as Arc<dyn MemOps>,
        ));
        Sgx { shared, region }
    }

    /// Give it the memory it shares with the host.
    pub fn attach_space(&self, space: &Arc<AddressSpace>) {
        *self.shared.bus.lock() = Some(Arc::downgrade(space));
    }
}

/// The class descriptor.
pub static CLASS: DeviceClass = DeviceClass {
    name: CLASS_NAME,
    version: STATE_VERSION,
    summary: "a PowerVR SGX stand-in: registers, the firmware's command handshake, and TA commands drawn",
    properties: &[
        PropertySpec {
            name: "revision",
            kind: ValueKind::Uint,
            required: false,
            summary: "the core revision it reports, as 0x00MMmmpp (default 0x010205)",
        },
        PropertySpec {
            name: "log",
            kind: ValueKind::Str,
            required: false,
            summary: "a character port every consumed command is reported on, one line each",
        },
        PropertySpec {
            name: "mode",
            kind: ValueKind::Str,
            required: false,
            summary: "`standin` (default): answer the host here; `hw`: run the real firmware on the modelled engine",
        },
        PropertySpec {
            name: "ipt",
            kind: ValueKind::Uint,
            required: false,
            summary: "hw mode: shader-engine instructions per device tick (default 256)",
        },
        PropertySpec {
            name: "timer-ticks",
            kind: ValueKind::Uint,
            required: false,
            summary: "hw mode: device ticks between firmware timer events, 0 for none (default 59)",
        },
        PropertySpec {
            name: "render",
            kind: ValueKind::Str,
            required: false,
            summary: "stand-in: `on` (default) draws each TA command before completing it, `off` only completes it",
        },
    ],
    construct: |props| Ok(Box::new(Sgx::new(props)?)),
};

impl Device for Sgx {
    fn class(&self) -> &'static DeviceClass {
        &CLASS
    }

    fn realize(&self, _ctx: &mut RealizeCtx<'_>) -> Result<()> {
        Ok(())
    }

    fn reset(&self, _kind: ResetKind) {
        *self.shared.state.lock() = State::default();
        if self.shared.hw.is_some() {
            *self.shared.engine.lock() = Some(Box::default());
        }
        self.shared.drive();
    }

    fn region(&self, name: &str) -> Option<RegionRef> {
        matches!(name, "" | "regs").then(|| Arc::clone(&self.region))
    }

    fn connect(&self, port: &str, source: WireSource) -> Result<()> {
        if port != IRQ_PIN {
            return Err(Error::Config {
                at: port.to_string(),
                message: String::from("an SGX drives one pin, `irq`"),
            });
        }
        *self.shared.irq.lock() = Some(source);
        Ok(())
    }

    fn announce(&self, _port: &str) {
        self.shared.drive();
    }

    fn is_runnable(&self) -> bool {
        true
    }

    fn run(&self, budget: crate::core::sched::Budget) -> crate::core::sched::Consumed {
        match self.shared.hw {
            Some(tuning) => self.shared.run_hw(budget.ticks, tuning),
            None => self.shared.run(),
        }
        crate::core::sched::Consumed::new(budget.ticks)
    }

    fn save(&self, w: &mut ChunkWriter<'_>) -> Result<()> {
        let s = self.shared.state.lock().clone();
        for v in &s.regs {
            w.write_u32(*v)?;
        }
        w.write_bool(s.start_pending)?;
        w.write_bool(s.kick_pending)?;
        w.write_bool(s.ta3d.is_some())?;
        w.write_u32(s.ta3d.unwrap_or(0))?;
        for b in s.hw_starts {
            w.write_bool(b)?;
        }
        for k in s.hw_kicks {
            w.write_u32(k)?;
        }
        w.write_bool(s.hw_reset)?;
        let engine = self.shared.engine.lock();
        w.write_bool(engine.is_some())?;
        match engine.as_ref() {
            Some(e) => e.save(w),
            None => Ok(()),
        }
    }

    fn load(&self, r: &mut ChunkReader<'_>) -> Result<()> {
        let mut regs = Vec::with_capacity(WORDS);
        for _ in 0..WORDS {
            regs.push(r.read_u32()?);
        }
        let start_pending = r.read_bool()?;
        let kick_pending = r.read_bool()?;
        let has = r.read_bool()?;
        let ta3d = r.read_u32()?;
        let mut hw_starts = [false; 3];
        for b in &mut hw_starts {
            *b = r.read_bool()?;
        }
        let mut hw_kicks = [0; 3];
        for k in &mut hw_kicks {
            *k = r.read_u32()?;
        }
        let hw_reset = r.read_bool()?;
        let engine = if r.read_bool()? {
            Some(Box::new(hw::Engine::load(r)?))
        } else {
            None
        };
        if engine.is_some() != self.shared.hw.is_some() {
            return Err(Error::State(String::from(
                "snapshot taken in the other SGX mode",
            )));
        }
        *self.shared.state.lock() = State {
            regs,
            start_pending,
            kick_pending,
            ta3d: has.then_some(ta3d),
            hw_starts,
            hw_kicks,
            hw_reset,
        };
        *self.shared.engine.lock() = engine;
        self.shared.drive();
        Ok(())
    }
}

impl Instance for Sgx {
    fn bind(&self, ctx: &BindCtx<'_>) -> Result<()> {
        let space = ctx.space().ok_or_else(|| Error::Config {
            at: String::from(ctx.path()),
            message: String::from(
                "the SGX shares memory with its host and needs that address space (`space = mem`)",
            ),
        })?;
        self.attach_space(space);
        *self.shared.requester.lock() = ctx.requester();
        Ok(())
    }
}

/// Add the class to a registry.
///
/// # Errors
///
/// If something already claimed the name.
pub fn register(registry: &mut crate::core::Registry) -> Result<()> {
    registry.add(&CLASS)
}

/// Bind the class into the machine graph.
///
/// # Errors
///
/// If the name is already bound.
pub fn bind(bindings: &mut crate::machine::Bindings) -> Result<()> {
    bindings.bind(CLASS_NAME, |props| Ok(Arc::new(Sgx::new(props)?)))
}

/// The validator schema.
#[must_use]
pub fn schema() -> crate::machine::validate::ClassSchema {
    use crate::machine::validate::{ClassSchema, PortDir, PropSchema};
    ClassSchema::new(CLASS_NAME)
        .prop(PropSchema::new("revision", ValueKind::Uint).range(0, 0xff_ffff))
        .prop(PropSchema::new("log", ValueKind::Str))
        .prop(PropSchema::new("mode", ValueKind::Str))
        .prop(PropSchema::new("ipt", ValueKind::Uint).range(1, 1 << 20))
        .prop(PropSchema::new("timer-ticks", ValueKind::Uint).range(0, 1 << 32))
        .region("")
        .region("regs")
        .port(IRQ_PIN, PortDir::Out)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::space::RamStore;
    use crate::core::state::{MachineShape, Migrations, StateReader, StateWriter};

    /// GPU addresses are mapped one to one onto the first 1 MiB here; the
    /// directory is at 0x1000, its one table at 0x2000.
    fn board() -> (Sgx, Arc<AddressSpace>) {
        let space = Arc::new(AddressSpace::new("mem", 32));
        {
            let mut t = space.topology();
            t.map(Region::ram("ram", Arc::new(RamStore::new(0x10_0000))), 0)
                .unwrap();
        }
        let wr = |a: u64, v: u32| {
            space
                .write(a, Width::U32, u64::from(v), MemAttrs::DEFAULT)
                .unwrap();
        };
        wr(0x1000, 0x2001);
        for page in 0..0x100u32 {
            wr(0x2000 + u64::from(page) * 4, (page << 12) | 1);
        }
        let sgx = Sgx::with_revision(0x01_0205);
        sgx.attach_space(&space);
        reg_w(&sgx, KERNEL_DIRECTORY, 0x1000);
        (sgx, space)
    }

    fn reg_w(s: &Sgx, off: u64, v: u32) {
        Registers(Arc::clone(&s.shared))
            .write(off, &v.to_le_bytes(), MemAttrs::DEFAULT)
            .unwrap();
    }

    fn reg_r(s: &Sgx, off: u64) -> u32 {
        let mut b = [0u8; 4];
        Registers(Arc::clone(&s.shared))
            .read(off, &mut b, MemAttrs::DEFAULT)
            .unwrap();
        u32::from_le_bytes(b)
    }

    fn m(space: &AddressSpace, a: u32) -> u32 {
        space
            .read(u64::from(a), Width::U32, MemAttrs::DEFAULT)
            .unwrap() as u32
    }

    fn mw(space: &AddressSpace, a: u32, v: u32) {
        space
            .write(u64::from(a), Width::U32, u64::from(v), MemAttrs::DEFAULT)
            .unwrap();
    }

    const KICKER: u32 = 0x8000;
    const TA3D: u32 = 0x8040;
    const HOST: u32 = 0x9000;
    const CTL: u32 = 0x9100;
    const RING: u32 = 0xa000;

    /// The layout the host's set-up code builds, then a start.
    fn started() -> (Sgx, Arc<AddressSpace>) {
        let (sgx, space) = board();
        mw(&space, TA3D, TA3D);
        mw(&space, TA3D + TA3D_HOST_CTL, HOST);
        mw(&space, TA3D + TA3D_CCB_CTL, CTL);
        mw(&space, TA3D + TA3D_CCB_RING, RING);
        reg_w(&sgx, KICKER_ADDR, KICKER);
        reg_w(&sgx, START_KICK, 1);
        assert_eq!(m(&space, HOST) & 1, 0, "nothing inside the register write");
        sgx.shared.run();
        (sgx, space)
    }

    fn submit(space: &AddressSpace, words: &[u32]) {
        let w = m(space, CTL);
        for (i, v) in words.iter().enumerate() {
            mw(space, RING + w * CCB_SLOT + 4 * i as u32, *v);
        }
        mw(space, CTL, (w + 1) % CCB_SLOTS);
    }

    #[test]
    fn a_start_finds_the_control_block_and_reports_running() {
        let (sgx, space) = started();
        assert_eq!(sgx.shared.state.lock().ta3d, Some(TA3D));
        assert_eq!(m(&space, HOST) & 1, 1);
    }

    #[test]
    fn running_firmware_keeps_its_heartbeat_written() {
        let (sgx, space) = started();
        assert_eq!(m(&space, HOST + HOST_HEARTBEAT), HEARTBEAT);
        // The host's timer counts it down; the next slice puts it back.
        mw(&space, HOST + HOST_HEARTBEAT, 0);
        sgx.shared.run();
        assert_eq!(m(&space, HOST + HOST_HEARTBEAT), HEARTBEAT);
    }

    /// A context at 0xc000 whose ring is at 0x20000 with its control pair at
    /// 0xc100, holding `len` bytes of commands from offset 0.
    fn context(space: &AddressSpace, len: u32) -> u32 {
        let ctx = 0xc000;
        mw(space, ctx + CTX_CCB_BASE, 0x2_0000);
        mw(space, ctx + CTX_CCB_CTL, 0xc100);
        mw(space, 0xc100, len);
        mw(space, 0xc104, 0);
        ctx
    }

    fn kick(sgx: &Sgx, space: &AddressSpace, kind: u32, ctx: u32) {
        submit(space, &[kind, 0, 0, ctx]);
        reg_w(sgx, COMMAND_KICK, 1);
        sgx.shared.run();
    }

    #[test]
    fn a_ta_kick_completes_its_syncs_and_statuses_without_drawing() {
        let (sgx, space) = started();
        let cmd = 0x2_0000;
        let s = cmd + 0x50;
        mw(&space, cmd, 0x300); // size
        mw(&space, cmd + 0x1c, TA_DEPENDENCY);
        mw(&space, cmd + 0x38, 0xd000); // render details
        mw(&space, 0xd000 + 0xe4, 0xd100); // its status word
        mw(&space, 0xd100, 1);
        mw(&space, s + 0x04, 1); // one TA status
        mw(&space, s + 0xc0, 0xd200);
        mw(&space, s + 0xc4, 0x77);
        mw(&space, s + 0x14, 5); // TA sync: read pending 5
        mw(&space, s + 0x18, 0xe00c);
        mw(&space, s + 0x34, 9); // dependency: write pending 9
        mw(&space, s + 0x38, 0xe104);
        mw(&space, s + 0x3c, 1); // one source
        mw(&space, s + 0x40, 2);
        mw(&space, s + 0x44, 0xe20c);
        let ctx = context(&space, 0x300);
        kick(&sgx, &space, CMD_KICK_TA, ctx);
        assert_eq!(m(&space, 0xe00c), 6, "TA read complete = pending + 1");
        assert_eq!(m(&space, 0xe104), 10, "dependency write complete");
        assert_eq!(m(&space, 0xe20c), 3, "source read complete");
        assert_eq!(m(&space, 0xd200), 0x77, "the status word the client polls");
        assert_eq!(m(&space, 0xd100), 0, "render details free again");
        assert_eq!(m(&space, 0xc104), 0x300, "the ring is consumed");
    }

    #[test]
    fn a_transfer_kick_completes_sources_and_destinations() {
        let (sgx, space) = started();
        let s = 0x2_0000 + 0xa8;
        mw(&space, s, 1); // one source
        mw(&space, s + 4, 7);
        mw(&space, s + 8, 0xe00c);
        mw(&space, s + 0x84, 1); // one destination {ROP, ROC, WOP, WOC}
        mw(&space, s + 0x88 + 8, 4);
        mw(&space, s + 0x88 + 12, 0xe104);
        mw(&space, s + 0x98, 11); // TA sync write pending
        mw(&space, s + 0x9c, 0xe204);
        mw(&space, s + 0xb8, 1); // one status: the queue's fence
        mw(&space, s + 0xbc, 0xe300);
        mw(&space, s + 0xc0, 2);
        let ctx = context(&space, 0x200);
        kick(&sgx, &space, CMD_KICK_TRANSFER, ctx);
        assert_eq!(m(&space, 0xe00c), 8);
        assert_eq!(m(&space, 0xe104), 5);
        assert_eq!(m(&space, 0xe204), 12);
        assert_eq!(m(&space, 0xe300), 2, "the fence the client waits on");
        assert_eq!(m(&space, 0xc104), 0x200);
    }

    #[test]
    fn misc_info_answers_with_this_build() {
        let (sgx, space) = started();
        let info = 0xb000;
        mw(&space, info, 2);
        submit(&space, &[CMD_MISC_INFO, 0, 0, info]);
        reg_w(&sgx, COMMAND_KICK, 1);
        sgx.shared.run();
        assert_eq!(m(&space, info), 3, "done, the host's own bit kept");
        assert_eq!(m(&space, info + 0x04), 0x01_0205);
        assert_eq!(m(&space, info + 0x0c), 0x0001_0711);
        assert_eq!(m(&space, info + 0x10), 0x0020_bcff);
        assert_eq!(m(&space, info + 0x1c), 0x1032_241c);
        assert_eq!(m(&space, info + 0x60), 0x20);
        assert_eq!(m(&space, CTL + 4), 1, "the slot is consumed");
    }

    #[test]
    fn power_and_cleanup_set_their_flags() {
        let (sgx, space) = started();
        submit(&space, &[CMD_POWER, 0, 0, 2]);
        submit(&space, &[CMD_CLEANUP, 0, 1, 0x1234]);
        reg_w(&sgx, COMMAND_KICK, 1);
        sgx.shared.run();
        assert_eq!(m(&space, HOST + 4), 1 << 2);
        assert_eq!(m(&space, HOST + 8), 1);
        assert_eq!(m(&space, CTL + 4), 2);
    }

    #[test]
    fn the_interrupt_follows_the_enabled_event_and_its_clear() {
        let (sgx, space) = started();
        reg_w(&sgx, EVENT_ENABLE, EVENT_UKERNEL);
        submit(&space, &[0x1e5, 0, 0, 0]);
        reg_w(&sgx, COMMAND_KICK, 1);
        sgx.shared.run();
        assert!(sgx.shared.state.lock().irq());
        reg_w(&sgx, EVENT_CLEAR, EVENT_UKERNEL | 0x8000_0000);
        assert!(!sgx.shared.state.lock().irq());
    }

    #[test]
    fn the_core_reads_alive_and_reports_its_revision() {
        let (sgx, _) = board();
        let a = reg_r(&sgx, ALIVE_COUNTER);
        assert_ne!(reg_r(&sgx, ALIVE_COUNTER), a);
        assert_eq!(reg_r(&sgx, 0x024), 0x01_0205);
        assert_eq!(reg_r(&sgx, 0x8024), 0x01_0205);
        let mut b = [0u8; 4];
        let before = reg_r(&sgx, ALIVE_COUNTER);
        Registers(Arc::clone(&sgx.shared))
            .read(ALIVE_COUNTER, &mut b, MemAttrs::DEBUG)
            .unwrap();
        assert_eq!(
            reg_r(&sgx, ALIVE_COUNTER),
            before + 1,
            "a debugger read counts nothing"
        );
    }

    #[test]
    fn a_snapshot_round_trips() {
        let (sgx, _) = started();
        reg_w(&sgx, 0x4000, 1);
        reg_w(&sgx, COMMAND_KICK, 1);
        let save = |s: &Sgx| {
            let mut shape = MachineShape::new();
            shape.add_device("sgx", CLASS.name).unwrap();
            let mut wr = StateWriter::new(shape);
            {
                let mut chunk = wr.chunk("sgx", CLASS.name, CLASS.version).unwrap();
                s.save(&mut chunk).unwrap();
            }
            wr.to_vec().unwrap()
        };
        let bytes = save(&sgx);
        let back = Sgx::with_revision(0x01_0205);
        let reader = StateReader::new(&bytes).unwrap();
        let chunk = reader
            .load("sgx", CLASS.name, CLASS.version, &Migrations::new())
            .unwrap();
        back.load(&mut chunk.reader()).unwrap();
        let want = sgx.shared.state.lock().clone();
        assert_eq!(*back.shared.state.lock(), want);
        assert_eq!(save(&back), bytes);
    }

    // -- the hardware mode --------------------------------------------------

    mod hw_mode {
        use super::*;

        // A few USSE and PDS words, assembled from the field positions in
        // `usse.rs` and `pds.rs`; nothing here comes from a firmware image.

        const fn b(n: u32) -> u64 {
            1 << n
        }
        const fn class(c: u64) -> u64 {
            31 << 59 | c << 52
        }
        fn limm(r: u64, v: u32) -> u64 {
            let v = u64::from(v);
            class(2)
                | 4 << 56
                | r << 21
                | (v & 0x1f_ffff)
                | (v >> 21 & 0x1f) << 36
                | (v >> 26 & 0x3f) << 44
        }
        fn reg_field(reg: u64) -> u64 {
            (reg & 0x7f) | (reg >> 7 & 0x7f) << 14 | (reg >> 14 & 0xf) << 34 | b(48) | 2 << 28
        }
        /// `str #reg, <bank> n`, bank 0 temp, 2 primary, 3 secondary.
        fn str_(reg: u64, bank: u64, n: u64) -> u64 {
            class(2) | 6 << 56 | b(51) | reg_field(reg) | bank << 30 | n << 7
        }
        fn ldr(r: u64, reg: u64) -> u64 {
            class(2) | 6 << 56 | r << 21 | reg_field(reg)
        }
        /// `and.test.t[z] _, p0, r<n>, r<n>`.
        fn test_zero(n: u64) -> u64 {
            // t = 0x14: zero test (t2 → b40) combined with AND (t4 → b39).
            9 << 59 | 0x30 << 14 | b(40) | b(39) | b(51) | b(32) | n << 7 | n
        }
        fn ba_if_p0(target: u64) -> u64 {
            class(0) | 1 << 56 | target
        }
        fn ba(target: u64) -> u64 {
            class(0) | target
        }
        const PHAS0: u64 = (31 << 59) | (4 << 52) | (2 << 56) | (7 << 40);
        const NOP_SYNCS: u64 = (31 << 59) | (5 << 38) | (1 << 50);
        /// `emit162 #0, r0, r2, #arg, #imm` (operands name register pairs).
        fn emit(arg: u64, imm: u64) -> u64 {
            class(2)
                | 3 << 56
                | b(47)
                | 1 << 7
                | b(48)
                | 2 << 28
                | arg
                | (imm & 0x3f) << 22
                | (imm >> 6 & 0x3f) << 35
        }

        fn dout(row_a: u32, irsrc: bool, row_b: u32, sel: [u32; 4], t: u32) -> u32 {
            7 << 24
                | row_a << 18
                | u32::from(irsrc) << 17
                | (row_b >> 4) << 16
                | (row_b & 0xf) << 12
                | sel[0] << 10
                | sel[1] << 8
                | sel[2] << 6
                | sel[3] << 4
                | t
        }
        const HALT: u32 = 0xaf00_0000;
        /// DOUTU W0 for code-base 15, instruction `pc`, 32 temps.
        fn doutu_w0(pc: u32) -> u32 {
            8 << 24 | pc << 4 | 15
        }

        const CODE: u32 = 0x1_0000;
        const PDS: u32 = 0x2_0000;

        fn hw_board() -> (Sgx, Arc<AddressSpace>) {
            let (_, space) = board();
            let sgx = Sgx::build(
                0x01_0205,
                None,
                Some(hw::Tuning {
                    ipt: 64,
                    timer_ticks: 0,
                }),
                false,
            );
            sgx.attach_space(&space);
            reg_w(&sgx, KERNEL_DIRECTORY, 0x1000);
            // The firmware's requester uses directory 7, the kernel's.
            reg_w(&sgx, 0xc78, 7);
            reg_w(&sgx, 0xa48, CODE >> 6 | 0x0c00_0000);
            (sgx, space)
        }

        fn put64(space: &AddressSpace, at: u32, words: &[u64]) {
            for (i, w) in words.iter().enumerate() {
                mw(space, at + 8 * i as u32, *w as u32);
                mw(space, at + 8 * i as u32 + 4, (*w >> 32) as u32);
            }
        }

        /// A PDS program at `at`: data rows, then code.
        fn put_pds(space: &AddressSpace, at: u32, rows: &[[u32; 4]], code: &[u32]) {
            for (i, v) in rows.iter().flatten().chain(code).enumerate() {
                mw(space, at + 4 * i as u32, *v);
            }
        }

        fn run(sgx: &Sgx, ticks: u64) {
            sgx.run(crate::core::sched::Budget::of(ticks));
        }

        fn idle(sgx: &Sgx) -> bool {
            sgx.shared.engine.lock().as_ref().unwrap().idle()
        }

        #[test]
        fn a_start_runs_the_event_program_and_the_firmware_raises_the_host_event() {
            let (sgx, space) = hw_board();
            put_pds(
                &space,
                PDS,
                &[[doutu_w0(0), 1, 0, 0]],
                &[dout(0, false, 0, [0, 1, 2, 0], 5), HALT],
            );
            put64(
                &space,
                CODE,
                &[limm(0, EVENT_UKERNEL), str_(0x4b, 0, 0), PHAS0, NOP_SYNCS],
            );
            reg_w(&sgx, 0x8a68, PDS);
            reg_w(&sgx, 0x8a6c, 1);
            reg_w(&sgx, EVENT_ENABLE, EVENT_UKERNEL);
            reg_w(&sgx, START_KICK, 1);
            assert_eq!(
                reg_r(&sgx, EVENT_STATUS),
                0,
                "nothing inside the register write"
            );
            run(&sgx, 4);
            assert_eq!(reg_r(&sgx, EVENT_STATUS), EVENT_UKERNEL);
            assert!(sgx.shared.state.lock().irq());
            assert!(idle(&sgx), "the task ended at its phase end");
            reg_w(&sgx, EVENT_CLEAR, 0x8000_0000 | EVENT_UKERNEL);
            assert!(!sgx.shared.state.lock().irq());
        }

        #[test]
        fn each_command_kick_is_one_event_with_its_bit_in_pa1() {
            let (sgx, space) = hw_board();
            put_pds(
                &space,
                PDS,
                &[[0, 0x100, 0, 0], [doutu_w0(0), 1, 0, 0]],
                &[
                    dout(0, true, 0, [2, 0, 0, 0], 6), // douta ir0 -> pa0
                    dout(0, true, 1, [2, 1, 0, 0], 6), // douta ir1 -> pa1
                    dout(1, false, 0, [0, 1, 2, 0], 5),
                    HALT,
                ],
            );
            put64(
                &space,
                CODE,
                &[str_(0x800, 2, 1), str_(0x801, 2, 0), PHAS0, NOP_SYNCS],
            );
            reg_w(&sgx, 0x8a68, PDS);
            reg_w(&sgx, 0x8a6c, 2);
            reg_w(&sgx, COMMAND_KICK, 1);
            reg_w(&sgx, COMMAND_KICK, 1);
            assert_eq!(
                sgx.shared.state.lock().hw_kicks[1],
                2,
                "counted, not latched"
            );
            run(&sgx, 8);
            assert_eq!(sgx.shared.state.lock().hw_kicks[1], 0);
            assert_eq!(reg_r(&sgx, 0x2000), 0x40);
            assert_eq!(reg_r(&sgx, 0x2004), 0);
            assert!(idle(&sgx));
        }

        #[test]
        fn emit_starts_a_secondary_program_whose_dma_fills_the_core_attributes() {
            let (sgx, space) = hw_board();
            // Task 1: emit the secondary program, end. Task 2: report sa6.
            put64(
                &space,
                CODE,
                &[
                    limm(0, 2 << 23),
                    limm(2, (PDS + 0x100) >> 4),
                    emit(0, 0xc43),
                    PHAS0,
                    NOP_SYNCS,
                    str_(0x803, 3, 6),
                    PHAS0,
                    NOP_SYNCS,
                ],
            );
            put_pds(
                &space,
                PDS,
                &[[doutu_w0(0), 1, 0, 0]],
                &[dout(0, false, 0, [0, 1, 2, 0], 5), HALT],
            );
            put_pds(
                &space,
                PDS + 0x100,
                &[[0x3_0000, 3 | 5 << 8, 0, 0], [doutu_w0(5), 1, 0, 0]],
                &[
                    dout(0, false, 0, [0, 1, 0, 0], 3),
                    dout(1, false, 0, [0, 1, 2, 0], 5),
                    HALT,
                ],
            );
            for (i, v) in [11, 22, 33, 44].into_iter().enumerate() {
                mw(&space, 0x3_0000 + 4 * i as u32, v);
            }
            reg_w(&sgx, 0x8a68, PDS);
            reg_w(&sgx, 0x8a6c, 1);
            reg_w(&sgx, START_KICK, 1);
            run(&sgx, 8);
            assert_eq!(reg_r(&sgx, 0x200c), 22);
            let sa = sgx.shared.engine.lock().as_ref().unwrap().sa(0)[5..9].to_vec();
            assert_eq!(sa, [11, 22, 33, 44]);
        }

        #[test]
        fn polls_see_their_completion_and_an_unknown_one_is_forced() {
            let (sgx, space) = hw_board();
            put64(
                &space,
                CODE,
                &[
                    ldr(0, 0x4b),
                    str_(0x804, 0, 0),
                    ldr(1, 0x3000), // a register nothing sets
                    test_zero(1),
                    ba_if_p0(2),
                    str_(0x805, 0, 1),
                    PHAS0,
                    NOP_SYNCS,
                ],
            );
            put_pds(
                &space,
                PDS,
                &[[doutu_w0(0), 1, 0, 0]],
                &[dout(0, false, 0, [0, 1, 2, 0], 5), HALT],
            );
            reg_w(&sgx, 0x8a68, PDS);
            reg_w(&sgx, 0x8a6c, 1);
            reg_w(&sgx, START_KICK, 1);
            run(&sgx, 16);
            assert_eq!(reg_r(&sgx, 0x2010) & 0x40, 0x40, "a completion bit");
            assert_eq!(reg_r(&sgx, 0x2010) & EVENT_UKERNEL, 0, "never the host's");
            assert_eq!(
                reg_r(&sgx, EVENT_STATUS),
                0,
                "and the host still reads what is stored"
            );
            assert_eq!(reg_r(&sgx, 0x2014), !0, "the stuck poll was let go");
            assert!(idle(&sgx));
        }

        #[test]
        fn the_hosts_soft_reset_drops_a_running_task() {
            let (sgx, space) = hw_board();
            put_pds(
                &space,
                PDS,
                &[[doutu_w0(0), 1, 0, 0]],
                &[dout(0, false, 0, [0, 1, 2, 0], 5), HALT],
            );
            put64(&space, CODE, &[ba(0)]);
            reg_w(&sgx, 0x8a68, PDS);
            reg_w(&sgx, 0x8a6c, 1);
            reg_w(&sgx, START_KICK, 1);
            run(&sgx, 2);
            assert!(!idle(&sgx));
            reg_w(&sgx, COMMAND_KICK, 1);
            reg_w(&sgx, 0x4080, 0x7ff);
            reg_w(&sgx, 0x4080, 0);
            run(&sgx, 1);
            assert!(idle(&sgx));
            assert_eq!(sgx.shared.state.lock().hw_kicks, [0; 3]);
        }

        #[test]
        fn a_snapshot_round_trips_with_a_task_in_flight() {
            let (sgx, space) = hw_board();
            put_pds(
                &space,
                PDS,
                &[[doutu_w0(0), 1, 0, 0]],
                &[dout(0, false, 0, [0, 1, 2, 0], 5), HALT],
            );
            put64(&space, CODE, &[limm(3, 0x1234), ba(1)]);
            reg_w(&sgx, 0x8a68, PDS);
            reg_w(&sgx, 0x8a6c, 1);
            reg_w(&sgx, START_KICK, 1);
            reg_w(&sgx, COMMAND_KICK, 1);
            run(&sgx, 2);
            assert!(!idle(&sgx), "the task loops");
            let save = |s: &Sgx| {
                let mut shape = MachineShape::new();
                shape.add_device("sgx", CLASS.name).unwrap();
                let mut wr = StateWriter::new(shape);
                {
                    let mut chunk = wr.chunk("sgx", CLASS.name, CLASS.version).unwrap();
                    s.save(&mut chunk).unwrap();
                }
                wr.to_vec().unwrap()
            };
            let bytes = save(&sgx);
            let (back, _) = hw_board();
            let reader = StateReader::new(&bytes).unwrap();
            let chunk = reader
                .load("sgx", CLASS.name, CLASS.version, &Migrations::new())
                .unwrap();
            back.load(&mut chunk.reader()).unwrap();
            let want = sgx.shared.state.lock().clone();
            assert_eq!(*back.shared.state.lock(), want);
            let fp = |s: &Sgx| hw::fingerprint(s.shared.engine.lock().as_ref().unwrap());
            assert_eq!(fp(&back), fp(&sgx));
            assert_eq!(save(&back), bytes);
            // The other mode's snapshot is refused.
            let standin = Sgx::with_revision(0x01_0205);
            let chunk = reader
                .load("sgx", CLASS.name, CLASS.version, &Migrations::new())
                .unwrap();
            assert!(standin.load(&mut chunk.reader()).is_err());
        }
    }
}
