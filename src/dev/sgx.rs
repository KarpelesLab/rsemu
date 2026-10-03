//! A PowerVR SGX stand-in: the register block, and the part of the GPU's
//! own firmware — its "microkernel" — that the host driver talks to through
//! shared memory. Enough for the driver to start the core, check its build
//! and complete its kernel commands. **No rendering**: a kick that would draw
//! is acknowledged and nothing is drawn.
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
const STATE_VERSION: u32 = 1;

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
}

impl Default for State {
    fn default() -> State {
        State {
            regs: alloc::vec![0; WORDS],
            start_pending: false,
            kick_pending: false,
            ta3d: None,
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
}

#[derive(Debug)]
struct Shared {
    revision: u32,
    state: Mutex<State>,
    irq: Mutex<Option<WireSource>>,
    bus: Mutex<Option<Weak<AddressSpace>>>,
    requester: Mutex<RequesterId>,
}

impl Shared {
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
        match offset {
            EVENT_CLEAR => *s.reg_mut(EVENT_STATUS) &= !v,
            EVENT_CLEAR2 => *s.reg_mut(EVENT_STATUS2) &= !v,
            START_KICK if v & 1 != 0 => s.start_pending = true,
            COMMAND_KICK if v & 1 != 0 => s.kick_pending = true,
            _ => *s.reg_mut(offset) = v,
        }
    }

    fn space(&self) -> Option<Arc<AddressSpace>> {
        self.bus.lock().as_ref().and_then(Weak::upgrade)
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
                    complete_kick(gpu, word(0), word(3));
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
fn complete_kick(gpu: &Gpu<'_>, kind: u32, context: u32) {
    let (Some(base), Some(ctl)) = (
        gpu.read(context + CTX_CCB_BASE),
        gpu.read(context + CTX_CCB_CTL),
    ) else {
        return;
    };
    let (Some(write), Some(mut read)) = (gpu.read(ctl), gpu.read(ctl + 4)) else {
        return;
    };
    // One command per kick for a transfer; the others walk by size. The
    // bound is a guard against a corrupt size, not a limit a guest meets.
    for _ in 0..256 {
        if read == write {
            break;
        }
        let cmd = base.wrapping_add(read);
        let size = match kind {
            CMD_KICK_TA => complete_ta(gpu, cmd),
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
        r.finish()?;
        Ok(Sgx::with_revision(revision))
    }

    /// Build one reporting `revision` (major.minor.maintenance in bits
    /// 23:16, 15:8, 7:0).
    #[must_use]
    pub fn with_revision(revision: u32) -> Sgx {
        let shared = Arc::new(Shared {
            revision,
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
    summary: "a PowerVR SGX stand-in: registers and the firmware's command handshake, no rendering",
    properties: &[PropertySpec {
        name: "revision",
        kind: ValueKind::Uint,
        required: false,
        summary: "the core revision it reports, as 0x00MMmmpp (default 0x010205)",
    }],
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
        self.shared.run();
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
        w.write_u32(s.ta3d.unwrap_or(0))
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
        *self.shared.state.lock() = State {
            regs,
            start_pending,
            kick_pending,
            ta3d: has.then_some(ta3d),
        };
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
        let ctx = context(&space, 0x200);
        kick(&sgx, &space, CMD_KICK_TRANSFER, ctx);
        assert_eq!(m(&space, 0xe00c), 8);
        assert_eq!(m(&space, 0xe104), 5);
        assert_eq!(m(&space, 0xe204), 12);
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
}
