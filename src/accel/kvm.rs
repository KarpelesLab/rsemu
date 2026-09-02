//! The KVM acceleration backend: `/dev/kvm` by raw `ioctl`.
//!
//! `ROADMAP.md` §10's first item, and the substance of phase 7: *"`/dev/kvm`,
//! vCPU fd, the `kvm_run` shared page, MMIO/PIO exits routed back into the
//! address-space layer"*. Guest code runs on the host's own silicon; every
//! access the guest makes that is not plain RAM comes back out as an exit and
//! is served by the same [`AddressSpace`] and the same device models the
//! interpreter uses. That last clause is the whole point — an accelerated
//! machine is the same machine, running faster.
//!
//! # Sources
//!
//! Everything here was written from the published KVM ABI. The `ioctl`
//! numbers, the structure layouts and the exit-reason values are stable ABI
//! and are cited where they appear:
//!
//! * `Documentation/virt/kvm/api.rst` — the normative description of every
//!   `ioctl` used below, the `kvm_run` fields, and the capability numbers.
//! * `include/uapi/linux/kvm.h` — the numbers themselves, which are
//!   **transcribed** rather than included: `ROADMAP.md` §0 forbids a C
//!   toolchain in the tree, so there is no header to `#include` and no
//!   `bindgen` to run.
//! * `include/uapi/asm-generic/ioctl.h` for the `_IOC` encoding, which is
//!   reproduced as a `const fn` rather than as a table of magic constants — a
//!   number derived from its direction, type, ordinal and struct size is a
//!   number that can be checked by reading it.
//!
//! Every transcribed structure is followed by a `const` assertion on its size,
//! because the size is *part of* the `ioctl` number: a field of the wrong width
//! produces a request the kernel does not recognise, and the assertion turns
//! that into a compile error rather than an `ENOTTY` at run time.
//!
//! # The exit flag, and why there is no signal here
//!
//! `CLAUDE.md`: *"Stopping the world … uses the safe-point protocol: a
//! generation counter plus a per-CPU exit flag checked at block boundaries.
//! Never a signal — wasm has none."* The conventional way to pull a KVM vCPU
//! out of the guest is exactly a signal: block `SIGUSR1`, hand KVM a signal
//! mask with `KVM_SET_SIGNAL_MASK`, and `kill` the thread. That design is
//! unavailable here, and the resolution is worth stating rather than hiding.
//!
//! Three facts make the signal unnecessary for correctness:
//!
//! 1. **A `KVM_RUN` return *is* a block boundary.** The safe-point contract
//!    asks a core to check its flag between blocks and unwind. Under KVM the
//!    guest leaves hardware on every MMIO access, every port access, every
//!    `HLT`, every interrupt window and every timer tick, and
//!    [`Vcpu::run_to_exit`] checks [`ExitFlag::raised`] at each of those points
//!    before re-entering. So a stop request is honoured at the next exit,
//!    which on any machine with a running timer is microseconds away.
//! 2. **`immediate_exit` closes the race the signal existed to close.** The
//!    reason the conventional design needs a signal *mask* at all is that a
//!    flag checked in userspace can be set in the window between the check and
//!    the `VMENTER`, and the run would then not see it. `KVM_CAP_IMMEDIATE_EXIT`
//!    (capability 136) is a byte in the shared `kvm_run` page that the kernel
//!    itself re-checks with interrupts disabled just before entry, returning
//!    `EINTR` instead of entering. Writing the exit flag through to that byte
//!    makes the check-then-enter sequence atomic **with no signal and no
//!    signal mask**, which is strictly better than the usual arrangement, not
//!    a workaround for it.
//! 3. **Nothing in the crate above this needs anything else.** A retopology, a
//!    snapshot or a reset waits on [`SafePoint`](crate::core::sched::SafePoint),
//!    and this backend reaches that state by returning from `run_to_exit`.
//!
//! What is genuinely given up, said plainly: **a guest that takes no exits is
//! not preemptible by this mechanism.** A vCPU spinning in a register-only
//! loop with interrupts masked will run until something else makes it exit,
//! and there is nothing portable left to force one. The honest answers are the
//! ones the guest already provides — a periodic timer interrupt, which every
//! machine phase 6 boots has — and they bound the stop latency in practice
//! without bounding it in theory. A backend that wanted a hard bound would
//! need the host signal this project has ruled out, and the cost of ruling it
//! out is exactly this paragraph. It is written down instead of being
//! discovered.
//!
//! # Determinism
//!
//! An accelerated run is **not reproducible** and this module never pretends
//! otherwise. Instruction counts, the instant an interrupt is taken and the
//! host TSC are all outside our control, so the state hash of an accelerated
//! run is meaningless. [`Vcpu::into_runnable`] therefore *refuses* a
//! deterministic [`ThreadingMode`], which is the same structural refusal
//! `Machine::set_recorder` makes rather than a comment asking callers to be
//! careful.

use alloc::sync::Arc;
use alloc::vec::Vec;
use core::marker::PhantomData;
use core::mem::size_of;

use crate::core::exec::{Access, Exit, ExitMask, ExitReason, ExitingCore, Run};
use crate::core::sched::{Budget, Consumed, ExitFlag, ThreadingMode};
use crate::core::space::{AddressSpace, HOST_PAGE, MemAttrs, RamStore, RequesterId, RomStore};
use crate::core::sync::{LockRank, Mutex};

use super::sys::{self, Errno, Fd, PAGE_SIZE, SysResult};
use super::{AccelError, AccelResult};

// ---------------------------------------------------------------------------
// the ABI, transcribed
// ---------------------------------------------------------------------------

/// The `ioctl` type byte KVM claims: `'K' << 1 | 0x2e`… in practice the
/// literal `0xAE`, which `include/uapi/linux/kvm.h` spells `KVMIO`.
const KVMIO: u64 = 0xAE;

/// `_IOC_NONE`: the request carries no argument, or an integer one.
const DIR_NONE: u64 = 0;
/// `_IOC_WRITE`: userspace hands the kernel a structure.
const DIR_WRITE: u64 = 1;
/// `_IOC_READ`: the kernel fills a structure in.
const DIR_READ: u64 = 2;
/// `_IOC_READ | _IOC_WRITE`: userspace fills part of a structure in and the
/// kernel fills the rest. `KVM_GET_MSRS` is the one request here that is both.
const DIR_RW: u64 = DIR_READ | DIR_WRITE;

/// The `_IOC` encoding from `asm-generic/ioctl.h`, for the "asm-generic"
/// architectures — which x86, ARM and RISC-V all are.
///
/// `dir` occupies bits 30-31, the argument `size` bits 16-29, the type byte
/// bits 8-15 and the ordinal bits 0-7.
#[must_use]
const fn ioc(dir: u64, nr: u64, size: u64) -> u64 {
    (dir << 30) | (size << 16) | (KVMIO << 8) | nr
}

/// A request whose argument is an integer, or which takes none.
#[derive(Debug, Clone, Copy)]
struct ReqVal(u64);

/// A request whose argument is a pointer to `T`.
///
/// The pairing of the request number with the structure type is established
/// **once**, in the constant table below, and carried by the type system from
/// there. That is what makes [`ioctl_struct`] a safe function: its `unsafe`
/// block's invariant — *this number's UAPI argument is this Rust type* — is
/// upheld by construction rather than by every call site remembering.
#[derive(Debug)]
struct Req<T>(u64, PhantomData<fn() -> T>);

impl<T> Clone for Req<T> {
    fn clone(&self) -> Self {
        *self
    }
}
impl<T> Copy for Req<T> {}

impl<T> Req<T> {
    const fn new(nr: u64) -> Req<T> {
        Req(nr, PhantomData)
    }
}

/// `KVM_GET_API_VERSION`. Must answer 12; it has since 2.6.22 and the
/// documentation says an older number means "do not use this interface".
const KVM_GET_API_VERSION: ReqVal = ReqVal(ioc(DIR_NONE, 0x00, 0));
/// `KVM_CREATE_VM`. The argument is the machine type; 0 on x86.
const KVM_CREATE_VM: ReqVal = ReqVal(ioc(DIR_NONE, 0x01, 0));
/// `KVM_CHECK_EXTENSION`. The argument is a `KVM_CAP_*` number.
const KVM_CHECK_EXTENSION: ReqVal = ReqVal(ioc(DIR_NONE, 0x03, 0));
/// `KVM_GET_VCPU_MMAP_SIZE`: how much of the vCPU descriptor to map.
const KVM_GET_VCPU_MMAP_SIZE: ReqVal = ReqVal(ioc(DIR_NONE, 0x04, 0));
/// `KVM_CREATE_VCPU`. The argument is the vCPU id.
const KVM_CREATE_VCPU: ReqVal = ReqVal(ioc(DIR_NONE, 0x41, 0));
/// `KVM_SET_TSS_ADDR`: three pages of scratch an Intel part without
/// unrestricted-guest support needs to run real mode at all.
const KVM_SET_TSS_ADDR: ReqVal = ReqVal(ioc(DIR_NONE, 0x47, 0));
/// `KVM_RUN`: enter the guest. No argument.
const KVM_RUN: ReqVal = ReqVal(ioc(DIR_NONE, 0x80, 0));
/// `KVM_GET_TSC_KHZ`: the frequency the guest's time-stamp counter advances
/// at. The other half of what makes a `RDTSC` reading mean anything on the far
/// side of an engine switch.
const KVM_GET_TSC_KHZ: ReqVal = ReqVal(ioc(DIR_NONE, 0xa3, 0));

/// `KVM_SET_USER_MEMORY_REGION`.
const KVM_SET_USER_MEMORY_REGION: Req<KvmUserspaceMemoryRegion> = Req::new(ioc(
    DIR_WRITE,
    0x46,
    size_of::<KvmUserspaceMemoryRegion>() as u64,
));
/// `KVM_SET_IDENTITY_MAP_ADDR`: one page, likewise for real mode on VMX.
const KVM_SET_IDENTITY_MAP_ADDR: Req<u64> = Req::new(ioc(DIR_WRITE, 0x48, size_of::<u64>() as u64));
/// `KVM_GET_REGS`.
const KVM_GET_REGS: Req<KvmRegs> = Req::new(ioc(DIR_READ, 0x81, size_of::<KvmRegs>() as u64));
/// `KVM_SET_REGS`.
const KVM_SET_REGS: Req<KvmRegs> = Req::new(ioc(DIR_WRITE, 0x82, size_of::<KvmRegs>() as u64));
/// `KVM_GET_SREGS`.
const KVM_GET_SREGS: Req<KvmSregs> = Req::new(ioc(DIR_READ, 0x83, size_of::<KvmSregs>() as u64));
/// `KVM_SET_SREGS`.
const KVM_SET_SREGS: Req<KvmSregs> = Req::new(ioc(DIR_WRITE, 0x84, size_of::<KvmSregs>() as u64));
/// `KVM_INTERRUPT`: inject one vector, for a VM whose interrupt controller
/// lives in userspace — which every board in this crate's does.
const KVM_INTERRUPT: Req<KvmInterrupt> =
    Req::new(ioc(DIR_WRITE, 0x86, size_of::<KvmInterrupt>() as u64));
/// `KVM_GET_FPU`.
const KVM_GET_FPU: Req<KvmFpu> = Req::new(ioc(DIR_READ, 0x8c, size_of::<KvmFpu>() as u64));
/// `KVM_SET_FPU`.
const KVM_SET_FPU: Req<KvmFpu> = Req::new(ioc(DIR_WRITE, 0x8d, size_of::<KvmFpu>() as u64));
/// `KVM_GET_DEBUGREGS`.
const KVM_GET_DEBUGREGS: Req<KvmDebugregs> =
    Req::new(ioc(DIR_READ, 0xa1, size_of::<KvmDebugregs>() as u64));
/// `KVM_SET_DEBUGREGS`.
const KVM_SET_DEBUGREGS: Req<KvmDebugregs> =
    Req::new(ioc(DIR_WRITE, 0xa2, size_of::<KvmDebugregs>() as u64));

/// `KVM_GET_MSRS`, and `KVM_SET_MSRS` below it.
///
/// **The size in these two numbers is eight, not the size of the argument**,
/// and that is the ABI rather than a mistake: `struct kvm_msrs` ends in a
/// flexible array, so the `_IOC` size is the *header's* — two `__u32` — and
/// the entry count in the header says how much more the kernel may touch.
/// They are therefore plain `u64`s rather than [`Req`] values, because the
/// invariant `Req` exists to carry (*the number names this exact type*) is not
/// true of either.
const KVM_GET_MSRS: u64 = ioc(DIR_RW, 0x88, 8);
/// See [`KVM_GET_MSRS`].
const KVM_SET_MSRS: u64 = ioc(DIR_WRITE, 0x89, 8);

const _: () = assert!(
    PAGE_SIZE == HOST_PAGE,
    "core::space aligns its stores to a different page than this backend asks for"
);

/// `KVM_CAP_USER_MEMORY`: the memory-slot interface exists. Without it there
/// is nothing to run out of.
pub const KVM_CAP_USER_MEMORY: u64 = 3;
/// `KVM_CAP_SET_TSS_ADDR`.
pub const KVM_CAP_SET_TSS_ADDR: u64 = 4;
/// `KVM_CAP_NR_MEMSLOTS`: how many memory regions this VM may hold.
pub const KVM_CAP_NR_MEMSLOTS: u64 = 10;
/// `KVM_CAP_READONLY_MEM`: a memory slot may be marked read-only, so that a
/// guest *write* to it leaves hardware as an MMIO exit while reads and
/// **instruction fetches** are served straight from the host pages.
///
/// The capability firmware needs. KVM's instruction emulator declines to fetch
/// through an MMIO exit, so a ROM that is not a slot is a board that cannot
/// execute its own reset vector.
pub const KVM_CAP_READONLY_MEM: u64 = 81;
/// `KVM_CAP_IMMEDIATE_EXIT`: the signal-free way to decline to enter the
/// guest. See the module documentation.
pub const KVM_CAP_IMMEDIATE_EXIT: u64 = 136;

/// `KVM_MEM_READONLY`, the flag that makes a slot's pages read-only to the
/// guest.
const KVM_MEM_READONLY: u32 = 1 << 1;

/// The API version every kernel since 2.6.22 reports.
pub const KVM_API_VERSION: i64 = 12;

/// `struct kvm_userspace_memory_region`.
#[repr(C)]
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
struct KvmUserspaceMemoryRegion {
    slot: u32,
    flags: u32,
    guest_phys_addr: u64,
    memory_size: u64,
    userspace_addr: u64,
}
const _: () = assert!(size_of::<KvmUserspaceMemoryRegion>() == 32);

/// `struct kvm_regs` — the general-purpose file plus `rip` and `rflags`.
///
/// The field *order* is the UAPI's (`rax, rbx, rcx, rdx` — note that `rbx`
/// comes second, not `rcx`), not the encoding order the ModRM byte uses.
#[repr(C)]
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct KvmRegs {
    /// Accumulator.
    pub rax: u64,
    /// Base.
    pub rbx: u64,
    /// Count.
    pub rcx: u64,
    /// Data.
    pub rdx: u64,
    /// Source index.
    pub rsi: u64,
    /// Destination index.
    pub rdi: u64,
    /// Stack pointer.
    pub rsp: u64,
    /// Base pointer.
    pub rbp: u64,
    /// `R8` through `R15`, in that order.
    pub r: [u64; 8],
    /// Instruction pointer.
    pub rip: u64,
    /// The flags register, zero-extended.
    pub rflags: u64,
}
const _: () = assert!(size_of::<KvmRegs>() == 144);

/// `struct kvm_segment` — a selector with the descriptor cached behind it.
///
/// `limit` is the **expanded, inclusive** byte limit, the same convention
/// [`cpu::x86::prot::SegReg`](crate::cpu::x86::prot::SegReg) uses, with `g`
/// reported separately as the granularity bit that produced it.
#[repr(C)]
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct KvmSegment {
    /// The cached base address.
    pub base: u64,
    /// The cached limit, expanded and inclusive.
    pub limit: u32,
    /// The selector last loaded.
    pub selector: u16,
    /// The descriptor's four-bit type field.
    pub kind: u8,
    /// Present.
    pub present: u8,
    /// Descriptor privilege level.
    pub dpl: u8,
    /// Default operand size, or "big" for a stack segment.
    pub db: u8,
    /// Not a system descriptor.
    pub s: u8,
    /// A 64-bit code segment.
    pub l: u8,
    /// Granularity: the stored limit counted 4 KiB pages.
    pub g: u8,
    /// Available to software.
    pub avl: u8,
    /// The segment is unusable — a null selector loaded into a data register.
    pub unusable: u8,
    /// Padding to the structure's 8-byte alignment.
    pub padding: u8,
}
const _: () = assert!(size_of::<KvmSegment>() == 24);

/// `struct kvm_dtable` — the GDT and IDT registers.
#[repr(C)]
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct KvmDtable {
    /// The table's linear base.
    pub base: u64,
    /// Its inclusive byte limit.
    pub limit: u16,
    /// Padding to the structure's 8-byte alignment.
    pub padding: [u16; 3],
}
const _: () = assert!(size_of::<KvmDtable>() == 16);

/// `struct kvm_sregs` — segments, descriptor tables, control registers.
#[repr(C)]
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct KvmSregs {
    /// Code segment.
    pub cs: KvmSegment,
    /// Data segment.
    pub ds: KvmSegment,
    /// Extra segment.
    pub es: KvmSegment,
    /// The 386's first extra segment.
    pub fs: KvmSegment,
    /// The 386's second extra segment.
    pub gs: KvmSegment,
    /// Stack segment.
    pub ss: KvmSegment,
    /// The task register.
    pub tr: KvmSegment,
    /// The local descriptor table register.
    pub ldt: KvmSegment,
    /// The global descriptor table register.
    pub gdt: KvmDtable,
    /// The interrupt descriptor table register.
    pub idt: KvmDtable,
    /// Control register 0.
    pub cr0: u64,
    /// Control register 2: the last faulting linear address.
    pub cr2: u64,
    /// Control register 3: the page-table base.
    pub cr3: u64,
    /// Control register 4: the extension enables.
    pub cr4: u64,
    /// Control register 8: the task-priority register.
    pub cr8: u64,
    /// `IA32_EFER`.
    pub efer: u64,
    /// The local APIC's base address register.
    pub apic_base: u64,
    /// Which of the 256 interrupt vectors are pending injection.
    pub interrupt_bitmap: [u64; 4],
}
const _: () = assert!(size_of::<KvmSregs>() == 312);

/// `struct kvm_interrupt` — the vector to inject.
#[repr(C)]
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct KvmInterrupt {
    /// The interrupt vector, 0-255.
    pub irq: u32,
}
const _: () = assert!(size_of::<KvmInterrupt>() == 4);

/// `struct kvm_fpu` — the x87 and SSE register files, in `FXSAVE` shape.
///
/// The tag word is the **abridged** one `FXSAVE` writes — one bit per physical
/// register, set when the register is not empty — rather than the two-bits-per
/// register word `FNSTENV` produces. Converting between them is
/// [`state::fpu_from_kvm`](crate::accel::state::fpu_from_kvm)'s job, and it is
/// not lossless in the direction that matters: recovering *valid* from *zero*
/// from *special* means re-examining the register's own encoding, which is
/// exactly what `FXRSTOR` does (*Intel SDM* volume 1 §8.1.7).
#[repr(C)]
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct KvmFpu {
    /// The eight data registers, in physical order, each a 10-byte extended
    /// value in the low bytes of a 16-byte slot.
    pub fpr: [[u8; 16]; 8],
    /// The control word.
    pub fcw: u16,
    /// The status word.
    pub fsw: u16,
    /// The abridged tag word: bit `i` set means register `i` is not empty.
    pub ftwx: u8,
    /// Padding.
    pub pad1: u8,
    /// The last escape opcode, eleven bits.
    pub last_opcode: u16,
    /// The last instruction pointer.
    pub last_ip: u64,
    /// The last data pointer.
    pub last_dp: u64,
    /// `XMM0` through `XMM15`, little-endian.
    pub xmm: [[u8; 16]; 16],
    /// The SSE control and status register.
    pub mxcsr: u32,
    /// Padding.
    pub pad2: u32,
}
const _: () = assert!(size_of::<KvmFpu>() == 416);

/// `struct kvm_debugregs` — `DR0`-`DR3`, `DR6` and `DR7`.
///
/// `DR4` and `DR5` are aliases of `DR6` and `DR7` on every part that has them,
/// which is why neither the structure nor the hardware carries them.
#[repr(C)]
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct KvmDebugregs {
    /// `DR0` through `DR3`: the four breakpoint addresses.
    pub db: [u64; 4],
    /// `DR6`, the status register.
    pub dr6: u64,
    /// `DR7`, the control register.
    pub dr7: u64,
    /// Flags; none are defined and it must be zero.
    pub flags: u64,
    /// Reserved.
    pub reserved: [u64; 9],
}
const _: () = assert!(size_of::<KvmDebugregs>() == 128);

/// One entry of `struct kvm_msrs`.
#[repr(C)]
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct KvmMsrEntry {
    /// The model-specific register's number, as `RDMSR` takes it in `ECX`.
    pub index: u32,
    /// Reserved; zero.
    pub reserved: u32,
    /// The value.
    pub data: u64,
}
const _: () = assert!(size_of::<KvmMsrEntry>() == 16);

/// `struct kvm_msrs`, with the flexible array given a length.
///
/// The kernel reads `nmsrs` and touches exactly that many entries, so a
/// `const` generic is a faithful transcription rather than an approximation of
/// one: the type *is* the header plus `N` entries.
#[repr(C)]
#[derive(Debug, Clone, Copy)]
struct KvmMsrs<const N: usize> {
    nmsrs: u32,
    pad: u32,
    entries: [KvmMsrEntry; N],
}

/// Byte offsets into the `kvm_run` page.
///
/// Read as a table rather than transcribed as a `#[repr(C)]` struct on
/// purpose: `kvm_run`'s body is a union with dozens of arms, most of them for
/// architectures this build cannot target, and reproducing all of them to
/// reach four fields would be more ABI surface to get wrong, not less. The
/// header of the structure — everything up to the union at offset 32 — is
/// fixed, and so is each arm's own layout.
mod run {
    /// `__u8 request_interrupt_window`: ask to be let out when the guest can
    /// take an interrupt. The other field userspace writes.
    pub(super) const REQUEST_INTERRUPT_WINDOW: u64 = 0;
    /// `__u8 immediate_exit`: set before `KVM_RUN` to decline to enter the
    /// guest. The one field userspace *writes*.
    pub(super) const IMMEDIATE_EXIT: u64 = 1;
    /// `__u8 ready_for_interrupt_injection`: whether `KVM_INTERRUPT` would be
    /// accepted right now.
    pub(super) const READY_FOR_INTERRUPT: u64 = 12;
    /// `__u32 exit_reason`.
    pub(super) const EXIT_REASON: u64 = 8;
    /// `__u8 if_flag`: whether the guest has interrupts enabled.
    pub(super) const IF_FLAG: u64 = 13;

    /// `io.direction`: 0 in (`IN`), 1 out (`OUT`).
    pub(super) const IO_DIRECTION: u64 = 32;
    /// `io.size`, in bytes.
    pub(super) const IO_SIZE: u64 = 33;
    /// `io.port`.
    pub(super) const IO_PORT: u64 = 34;
    /// `io.count`: how many transfers, for a string instruction.
    pub(super) const IO_COUNT: u64 = 36;
    /// `io.data_offset`: where in this same page the data sits.
    pub(super) const IO_DATA_OFFSET: u64 = 40;

    /// `mmio.phys_addr`.
    pub(super) const MMIO_PHYS_ADDR: u64 = 32;
    /// `mmio.data[8]`.
    pub(super) const MMIO_DATA: u64 = 40;
    /// `mmio.len`.
    pub(super) const MMIO_LEN: u64 = 48;
    /// `mmio.is_write`.
    pub(super) const MMIO_IS_WRITE: u64 = 52;

    /// `fail_entry.hardware_entry_failure_reason`, and `hw.hardware_exit_reason`
    /// — the same offset, the first word of the union.
    pub(super) const HARDWARE_REASON: u64 = 32;
}

/// `KVM_EXIT_*`, the values `kvm_run.exit_reason` takes.
mod exit {
    pub(super) const UNKNOWN: u32 = 0;
    pub(super) const EXCEPTION: u32 = 1;
    pub(super) const IO: u32 = 2;
    pub(super) const HLT: u32 = 5;
    pub(super) const MMIO: u32 = 6;
    pub(super) const IRQ_WINDOW_OPEN: u32 = 7;
    pub(super) const SHUTDOWN: u32 = 8;
    pub(super) const FAIL_ENTRY: u32 = 9;
    pub(super) const INTR: u32 = 10;
    pub(super) const INTERNAL_ERROR: u32 = 17;
    pub(super) const SYSTEM_EVENT: u32 = 24;
}

/// `KVM_EXIT_IO_OUT`. The other direction is 0.
const IO_OUT: u8 = 1;

// ---------------------------------------------------------------------------
// the two ioctl wrappers
// ---------------------------------------------------------------------------

/// Perform a request whose argument is an integer.
#[allow(unsafe_code)]
fn ioctl_val(fd: &Fd, req: ReqVal, arg: u64) -> SysResult<i64> {
    // SAFETY: `req` is one of the `ReqVal` constants above, every one of which
    // is `_IOC_NONE`-encoded and takes its argument by value — an id, a
    // capability number, a physical address, or nothing at all. No pointer is
    // passed, so there is no memory for the kernel to touch and nothing for
    // this frame to keep alive.
    unsafe { sys::ioctl(fd, req.0, arg) }
}

/// Perform a request whose argument is a `T`.
#[allow(unsafe_code)]
fn ioctl_struct<T>(fd: &Fd, req: Req<T>, arg: &mut T) -> SysResult<i64> {
    // SAFETY: `Req<T>` can only be built by the constant table above, and each
    // entry there pairs the UAPI request number with the Rust transcription of
    // exactly that request's argument type — the size of which is *part of*
    // the number, so a mismatched transcription would be rejected by the
    // kernel as an unknown request rather than silently mis-parsed. `arg` is a
    // live, aligned, uniquely borrowed `T` for the whole call, which covers
    // both the read and the write direction.
    unsafe { sys::ioctl(fd, req.0, core::ptr::from_mut(arg) as u64) }
}

/// Perform `KVM_GET_MSRS` or `KVM_SET_MSRS`.
///
/// Separate from [`ioctl_struct`] because those two numbers encode the size of
/// the *header* rather than of the argument — see [`KVM_GET_MSRS`] — so the
/// type-carrying [`Req`] would be claiming something untrue.
#[allow(unsafe_code)]
fn ioctl_msrs<const N: usize>(fd: &Fd, req: u64, arg: &mut KvmMsrs<N>) -> SysResult<i64> {
    // SAFETY: `req` is one of the two constants above, whose UAPI argument is
    // `struct kvm_msrs`; `KvmMsrs<N>` is its `#[repr(C)]` transcription with
    // the flexible array given the length `arg.nmsrs` reports, and the caller
    // below is the only constructor — it sets `nmsrs` to `N` and never to more.
    // The kernel therefore touches the header plus exactly `N` entries, all of
    // which are inside this live, aligned, uniquely borrowed value for the
    // whole call.
    unsafe { sys::ioctl(fd, req, core::ptr::from_mut(arg) as u64) }
}

fn sys_err(what: &'static str) -> impl Fn(Errno) -> AccelError {
    move |errno| AccelError::Sys { what, errno }
}

// ---------------------------------------------------------------------------
// /dev/kvm
// ---------------------------------------------------------------------------

/// The path this backend opens. Not configurable: it is the device node's
/// name, not a preference.
pub const DEV_KVM: &[u8] = b"/dev/kvm\0";

/// An open handle on `/dev/kvm`.
#[derive(Debug)]
pub struct Kvm {
    fd: Fd,
    vcpu_mmap_size: u64,
}

impl Kvm {
    /// Open `/dev/kvm` and check that its ABI is the one this code was written
    /// against.
    ///
    /// # Errors
    ///
    /// [`AccelError::Unavailable`] where there is no `/dev/kvm` or this user
    /// cannot open it — which is the case every test in this module treats as
    /// *skip*, not as *fail*. [`AccelError::Sys`] for anything else.
    pub fn open() -> AccelResult<Kvm> {
        let fd = sys::open(DEV_KVM, sys::O_RDWR | sys::O_CLOEXEC).map_err(|errno| {
            match errno.is_unavailable() {
                true => AccelError::Unavailable(errno),
                false => AccelError::Sys {
                    what: "open /dev/kvm",
                    errno,
                },
            }
        })?;

        let version =
            ioctl_val(&fd, KVM_GET_API_VERSION, 0).map_err(sys_err("KVM_GET_API_VERSION"))?;
        if version != KVM_API_VERSION {
            return Err(AccelError::Unsupported(
                "the kernel reports a KVM API version other than 12",
            ));
        }

        let kvm = Kvm {
            fd,
            vcpu_mmap_size: 0,
        };
        if kvm.check_extension(KVM_CAP_USER_MEMORY) == 0 {
            return Err(AccelError::Unsupported(
                "this kernel has no KVM_CAP_USER_MEMORY, so there is no way to give a guest RAM",
            ));
        }
        let size = ioctl_val(&kvm.fd, KVM_GET_VCPU_MMAP_SIZE, 0)
            .map_err(sys_err("KVM_GET_VCPU_MMAP_SIZE"))?;
        #[allow(clippy::cast_sign_loss)]
        let vcpu_mmap_size = size as u64;
        if vcpu_mmap_size < PAGE_SIZE || !vcpu_mmap_size.is_multiple_of(PAGE_SIZE) {
            return Err(AccelError::Unsupported(
                "KVM_GET_VCPU_MMAP_SIZE is not a whole number of pages",
            ));
        }
        Ok(Kvm {
            vcpu_mmap_size,
            ..kvm
        })
    }

    /// Whether this host has a usable `/dev/kvm`.
    ///
    /// What every test in this crate gates on, and what a front end should ask
    /// before offering `--accel kvm`.
    #[must_use]
    pub fn is_available() -> bool {
        Kvm::open().is_ok()
    }

    /// Ask whether an extension is present. Zero means no; the value otherwise
    /// depends on the capability.
    #[must_use]
    pub fn check_extension(&self, cap: u64) -> i64 {
        ioctl_val(&self.fd, KVM_CHECK_EXTENSION, cap).unwrap_or(0)
    }

    /// How large the per-vCPU shared mapping is.
    #[must_use]
    pub const fn vcpu_mmap_size(&self) -> u64 {
        self.vcpu_mmap_size
    }

    /// Create a virtual machine.
    ///
    /// # Errors
    ///
    /// [`AccelError::Sys`] if `KVM_CREATE_VM` fails.
    pub fn create_vm(&self) -> AccelResult<Vm> {
        let fd = ioctl_val(&self.fd, KVM_CREATE_VM, 0).map_err(sys_err("KVM_CREATE_VM"))?;
        #[allow(clippy::cast_possible_truncation)]
        let vm = Vm {
            fd: Fd::from_raw(fd as i32),
            vcpu_mmap_size: self.vcpu_mmap_size,
            immediate_exit: self.check_extension(KVM_CAP_IMMEDIATE_EXIT) != 0,
            readonly_mem: self.check_extension(KVM_CAP_READONLY_MEM) != 0,
            slots: Mutex::with_rank(LockRank::MACHINE, Vec::new()),
        };
        vm.prepare_x86()?;
        Ok(vm)
    }
}

// ---------------------------------------------------------------------------
// a VM
// ---------------------------------------------------------------------------

/// One virtual machine: its memory slots and its vCPUs.
#[derive(Debug)]
pub struct Vm {
    fd: Fd,
    vcpu_mmap_size: u64,
    immediate_exit: bool,
    readonly_mem: bool,
    /// The stores backing each slot, kept alive for as long as the kernel
    /// holds their addresses. Dropping a store whose slot is still installed
    /// would free memory the guest is executing out of, so the VM owns a
    /// reference to it.
    slots: Mutex<Vec<Slot>>,
}

/// Which of `core::space`'s stores a slot is backed by.
#[derive(Debug, Clone)]
enum Store {
    Ram(Arc<RamStore>),
    Rom(Arc<RomStore>),
}

/// A window onto a backing store, as a hypervisor sees it.
///
/// The store is `core::space`'s own, which is the point: the guest's hardware,
/// the interpreter, the debugger, a DMA master and a snapshot all reach *the
/// same bytes*, with no copy and no second RAM type. That is what
/// [`RamStore::host_addr`] made possible — before it a board's declared `ram`
/// had allocation alignment 1 and could not be a memory slot at all.
///
/// A *window* rather than the whole store, because a board's flat view is
/// entitled to map part of one: an aliased aperture, a shadowed ROM, a region
/// split by a device that sits in the middle of it.
#[derive(Debug, Clone)]
pub struct Backing {
    store: Store,
    offset: u64,
    len: u64,
}

impl Backing {
    /// The whole of a [`RamStore`], read/write.
    #[must_use]
    pub fn ram(store: &Arc<RamStore>) -> Backing {
        Backing {
            len: store.len(),
            store: Store::Ram(Arc::clone(store)),
            offset: 0,
        }
    }

    /// The whole of a [`RomStore`], read-only.
    #[must_use]
    pub fn rom(store: &Arc<RomStore>) -> Backing {
        Backing {
            len: store.len(),
            store: Store::Rom(Arc::clone(store)),
            offset: 0,
        }
    }

    /// `len` bytes starting `offset` into the same store.
    ///
    /// # Errors
    ///
    /// [`AccelError::Unsupported`] if the window leaves the store.
    pub fn window(self, offset: u64, len: u64) -> AccelResult<Backing> {
        let whole = match &self.store {
            Store::Ram(r) => r.len(),
            Store::Rom(r) => r.len(),
        };
        let end = offset.saturating_add(len);
        if end > whole {
            return Err(AccelError::Unsupported(
                "a memory-slot window that leaves its backing store",
            ));
        }
        Ok(Backing {
            store: self.store,
            offset,
            len,
        })
    }

    /// Size of the window in bytes.
    #[must_use]
    pub const fn len(&self) -> u64 {
        self.len
    }

    /// Whether the window holds no bytes. A zero-length region is how the ABI
    /// spells *delete a slot*, so it is refused before the ioctl.
    #[must_use]
    pub const fn is_empty(&self) -> bool {
        self.len == 0
    }

    /// Whether the guest may write here.
    #[must_use]
    pub const fn is_writable(&self) -> bool {
        matches!(self.store, Store::Ram(_))
    }

    /// The host address the kernel is given.
    #[must_use]
    pub fn host_addr(&self) -> u64 {
        let base = match &self.store {
            Store::Ram(r) => r.host_addr(),
            Store::Rom(r) => r.host_addr(),
        };
        base + self.offset
    }

    /// The slot flags this backing needs.
    const fn flags(&self) -> u32 {
        match self.store {
            Store::Ram(_) => 0,
            Store::Rom(_) => KVM_MEM_READONLY,
        }
    }
}

/// One installed memory region.
#[derive(Debug)]
struct Slot {
    index: u32,
    guest_phys_addr: u64,
    backing: Backing,
}

impl Vm {
    /// Whether `KVM_CAP_IMMEDIATE_EXIT` is available, and therefore whether a
    /// stop request can be made race-free without a signal.
    #[must_use]
    pub const fn has_immediate_exit(&self) -> bool {
        self.immediate_exit
    }

    /// The scratch areas an Intel part without unrestricted-guest support
    /// needs before it can run 16-bit code.
    ///
    /// A failure is *not* an error: on AMD, and on Intel with unrestricted
    /// guest, the ioctls are either absent or unnecessary.
    ///
    /// # Where they go, and why not where they used to
    ///
    /// The kernel turns each of these into a **private memory slot**, and a
    /// user slot may not overlap one. They were placed *just under 4 GiB* —
    /// which is precisely where every PC board in this crate puts the second
    /// copy of its firmware socket, so that `CS:IP = f000:fff0` finds a reset
    /// vector. Installing that ROM as a slot would have been refused, on any
    /// host old enough to need the scratch at all, with an `EEXIST` naming
    /// neither party.
    ///
    /// They now sit in the **PCI memory hole** immediately below the local
    /// APIC's page. That range is decoded by devices on every board this crate
    /// describes, never by RAM or ROM, so it is never a user slot — which is
    /// the property that actually matters, and "somewhere nothing is mapped"
    /// was never it.
    fn prepare_x86(&self) -> AccelResult<()> {
        // `KVM_SET_TSS_ADDR` wants three consecutive pages; the identity map
        // sits immediately below them.
        const TSS_ADDR: u64 = 0xfeff_c000;
        const IDENTITY_MAP_ADDR: u64 = 0xfeff_b000;
        let _ = ioctl_val(&self.fd, KVM_SET_TSS_ADDR, TSS_ADDR);
        let mut addr = IDENTITY_MAP_ADDR;
        let _ = ioctl_struct(&self.fd, KVM_SET_IDENTITY_MAP_ADDR, &mut addr);
        Ok(())
    }

    /// Whether this kernel can mark a slot read-only, and therefore whether
    /// [`set_rom_region`](Vm::set_rom_region) will work.
    #[must_use]
    pub const fn has_readonly_mem(&self) -> bool {
        self.readonly_mem
    }

    /// Install a board's [`RamStore`] as guest physical memory at
    /// `guest_phys_addr`.
    ///
    /// The store is kept alive by the VM: the kernel holds its host address
    /// until the slot is removed, so dropping it early would hand the guest a
    /// window onto whatever the allocator did next.
    ///
    /// # Errors
    ///
    /// [`AccelError::Sys`] if the kernel refuses the region — which it does
    /// for an unaligned guest address, a size that is not a whole number of
    /// pages, or a slot index above `KVM_CAP_NR_MEMSLOTS`.
    pub fn set_memory_region(
        &self,
        index: u32,
        guest_phys_addr: u64,
        store: &Arc<RamStore>,
    ) -> AccelResult<()> {
        self.set_region(index, guest_phys_addr, Backing::ram(store))
    }

    /// Install a [`RomStore`] as a **read-only** slot.
    ///
    /// Reads and instruction fetches are served by hardware; a guest write
    /// leaves as an MMIO exit and is routed into the address space, where the
    /// region's [`RomWrite`](crate::core::space::RomWrite) policy decides what
    /// it does — the same answer the interpreter gives.
    ///
    /// # Errors
    ///
    /// [`AccelError::Unsupported`] if this kernel has no
    /// [`KVM_CAP_READONLY_MEM`], because installing firmware as writable
    /// memory instead would be a silently different machine.
    pub fn set_rom_region(
        &self,
        index: u32,
        guest_phys_addr: u64,
        store: &Arc<RomStore>,
    ) -> AccelResult<()> {
        if !self.readonly_mem {
            return Err(AccelError::Unsupported(
                "this kernel has no KVM_CAP_READONLY_MEM, so a ROM cannot be a memory slot",
            ));
        }
        self.set_region(index, guest_phys_addr, Backing::rom(store))
    }

    /// The general form both of the above are.
    ///
    /// # Errors
    ///
    /// [`AccelError::Unsupported`] for a misaligned region, [`AccelError::Sys`]
    /// if the kernel refuses it.
    pub fn set_region(
        &self,
        index: u32,
        guest_phys_addr: u64,
        backing: Backing,
    ) -> AccelResult<()> {
        if !backing.is_writable() && !self.readonly_mem {
            return Err(AccelError::Unsupported(
                "this kernel has no KVM_CAP_READONLY_MEM, so a ROM cannot be a memory slot",
            ));
        }
        // Zero is a multiple of the page size, so the size check below would
        // let an empty store through — and a zero `memory_size` is how the ABI
        // spells *delete this slot*. Installing nothing must not quietly
        // remove something.
        if backing.is_empty() {
            return Err(AccelError::Unsupported(
                "an empty backing store: a zero-length region is the ABI's way of deleting a slot",
            ));
        }
        if !guest_phys_addr.is_multiple_of(PAGE_SIZE) || !backing.len().is_multiple_of(PAGE_SIZE) {
            return Err(AccelError::Unsupported(
                "a KVM memory region needs a page-aligned guest address and a page-multiple size",
            ));
        }
        if !backing.host_addr().is_multiple_of(PAGE_SIZE) {
            return Err(AccelError::Unsupported(
                "a backing store whose allocation is not host-page aligned",
            ));
        }
        // A slot whose *geometry* changes has to be deleted and recreated: the
        // kernel accepts an in-place update only when the guest address and the
        // size are unchanged (it is then a flags-or-host-address move), and
        // refuses anything else with `EINVAL`. Doing the delete here rather
        // than making every caller remember is the whole point of a wrapper.
        let stale = {
            let slots = self.slots.lock();
            slots
                .iter()
                .find(|s| s.index == index)
                .map(|s| (s.guest_phys_addr, s.backing.len()))
        };
        if let Some((old_addr, old_len)) = stale
            && (old_addr != guest_phys_addr || old_len != backing.len())
        {
            self.delete_memory_region(index)?;
        }

        let mut region = KvmUserspaceMemoryRegion {
            slot: index,
            flags: backing.flags(),
            guest_phys_addr,
            memory_size: backing.len(),
            userspace_addr: backing.host_addr(),
        };
        ioctl_struct(&self.fd, KVM_SET_USER_MEMORY_REGION, &mut region)
            .map_err(sys_err("KVM_SET_USER_MEMORY_REGION"))?;
        let mut slots = self.slots.lock();
        slots.retain(|s| s.index != index);
        slots.push(Slot {
            index,
            guest_phys_addr,
            backing,
        });
        Ok(())
    }

    /// Remove a memory region.
    ///
    /// A zero `memory_size` is how the ABI spells "delete this slot"; the store
    /// is released with it, so a caller must be sure the guest is not running
    /// out of it.
    ///
    /// # Errors
    ///
    /// [`AccelError::Sys`] if the kernel refuses.
    pub fn delete_memory_region(&self, index: u32) -> AccelResult<()> {
        let mut region = KvmUserspaceMemoryRegion {
            slot: index,
            flags: 0,
            guest_phys_addr: 0,
            memory_size: 0,
            userspace_addr: 0,
        };
        ioctl_struct(&self.fd, KVM_SET_USER_MEMORY_REGION, &mut region)
            .map_err(sys_err("KVM_SET_USER_MEMORY_REGION (delete)"))?;
        self.slots.lock().retain(|s| s.index != index);
        Ok(())
    }

    /// The guest-physical bases of the installed slots, lowest index first.
    #[must_use]
    pub fn memory_regions(&self) -> Vec<(u32, u64, u64)> {
        let mut slots: Vec<(u32, u64, u64)> = self
            .slots
            .lock()
            .iter()
            .map(|s| (s.index, s.guest_phys_addr, s.backing.len()))
            .collect();
        slots.sort_unstable();
        slots
    }

    /// The frequency the guest's time-stamp counter runs at, in kHz.
    ///
    /// Needed to make a `RDTSC` value mean the same thing on both sides of an
    /// engine switch: the counter is a *rate* plus an offset, and carrying the
    /// offset without the rate would move a guest's idea of elapsed time.
    ///
    /// # Errors
    ///
    /// [`AccelError::Sys`] if the kernel has no `KVM_GET_TSC_KHZ`.
    pub fn tsc_khz(&self) -> AccelResult<u64> {
        let khz = ioctl_val(&self.fd, KVM_GET_TSC_KHZ, 0).map_err(sys_err("KVM_GET_TSC_KHZ"))?;
        #[allow(clippy::cast_sign_loss)]
        Ok(khz as u64)
    }

    /// Create a vCPU, mapping its shared `kvm_run` page.
    ///
    /// `memory` is the address space MMIO exits are routed into and `io` the
    /// one port accesses reach — the same spaces the interpreter is attached
    /// to, which is what makes an accelerated guest and an interpreted one see
    /// one machine.
    ///
    /// # Errors
    ///
    /// [`AccelError::Sys`] if the vCPU cannot be created or its page mapped.
    pub fn create_vcpu(
        &self,
        id: u32,
        memory: Arc<AddressSpace>,
        io: Option<Arc<AddressSpace>>,
    ) -> AccelResult<Vcpu> {
        let fd = ioctl_val(&self.fd, KVM_CREATE_VCPU, u64::from(id))
            .map_err(sys_err("KVM_CREATE_VCPU"))?;
        #[allow(clippy::cast_possible_truncation)]
        let fd = Fd::from_raw(fd as i32);
        let run = sys::map_shared(&fd, self.vcpu_mmap_size, 0)
            .map_err(sys_err("mmap the kvm_run page"))?;
        Ok(Vcpu {
            id,
            immediate_exit: self.immediate_exit,
            inner: Mutex::with_rank(LockRank::BUS, Inner { fd, run }),
            memory,
            io,
            requester: RequesterId::ANONYMOUS,
            exit_flag: ExitFlag::default(),
            mask: Mutex::with_rank(LockRank::LEAF, ExitMask::NONE),
            stats: Mutex::with_rank(LockRank::LEAF, VcpuStats::default()),
        })
    }
}

// ---------------------------------------------------------------------------
// a vCPU
// ---------------------------------------------------------------------------

/// The descriptor and the shared page, together.
///
/// One lock covers both because they are one resource: KVM requires that a
/// vCPU be driven by one thread at a time, and the `kvm_run` page is only
/// meaningful between that thread's `KVM_RUN` calls.
#[derive(Debug)]
struct Inner {
    fd: Fd,
    run: sys::Mapping,
}

/// How much a vCPU has done. Diagnostics only — none of it reaches the guest.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct VcpuStats {
    /// How many times `KVM_RUN` has returned.
    pub entries: u64,
    /// How many port accesses were routed into the I/O space.
    pub pio: u64,
    /// How many MMIO accesses were routed into the memory space.
    pub mmio: u64,
    /// How many times a stop request kept the guest from being entered.
    pub declined: u64,
}

/// One virtual CPU, running on the host's own silicon.
///
/// Implements [`ExitingCore`] — `ROADMAP.md` §4.6's execution-engine seam, and
/// the one `core::exec` was written for: *"an accel backend implements it by
/// translating its own exit structure into an [`Exit`]"*. It is therefore
/// interchangeable with an interpreter at the type level, which is what §4.6
/// means by *"the choice is a per-CPU config property"*.
#[derive(Debug)]
pub struct Vcpu {
    id: u32,
    immediate_exit: bool,
    inner: Mutex<Inner>,
    memory: Arc<AddressSpace>,
    io: Option<Arc<AddressSpace>>,
    requester: RequesterId,
    exit_flag: ExitFlag,
    mask: Mutex<ExitMask>,
    stats: Mutex<VcpuStats>,
}

impl Vcpu {
    /// This vCPU's index within its VM.
    #[must_use]
    pub const fn id(&self) -> u32 {
        self.id
    }

    /// Counters, for a test or a monitor.
    #[must_use]
    pub fn stats(&self) -> VcpuStats {
        *self.stats.lock()
    }

    /// Set the requester id every routed exit carries into the address space.
    ///
    /// The machine layer allocates one per initiator; until an accelerated CPU
    /// is a machine-file device this is how a caller supplies it.
    pub fn set_requester(&mut self, id: RequesterId) {
        self.requester = id;
    }

    /// Give this vCPU the scheduler's per-CPU exit flag.
    ///
    /// The safe-point protocol's other half. See the module documentation for
    /// what honouring it does and does not bound.
    pub fn set_exit_flag(&mut self, flag: ExitFlag) {
        self.exit_flag = flag;
    }

    /// The flag this vCPU checks between guest entries.
    #[must_use]
    pub fn exit_flag(&self) -> ExitFlag {
        self.exit_flag.clone()
    }

    /// The general-purpose registers.
    ///
    /// # Errors
    ///
    /// [`AccelError::Sys`] if `KVM_GET_REGS` fails.
    pub fn regs(&self) -> AccelResult<KvmRegs> {
        let inner = self.inner.lock();
        let mut regs = KvmRegs::default();
        ioctl_struct(&inner.fd, KVM_GET_REGS, &mut regs).map_err(sys_err("KVM_GET_REGS"))?;
        Ok(regs)
    }

    /// Set the general-purpose registers.
    ///
    /// # Errors
    ///
    /// [`AccelError::Sys`] if `KVM_SET_REGS` fails.
    pub fn set_regs(&self, regs: &KvmRegs) -> AccelResult<()> {
        let inner = self.inner.lock();
        let mut regs = *regs;
        ioctl_struct(&inner.fd, KVM_SET_REGS, &mut regs).map_err(sys_err("KVM_SET_REGS"))?;
        Ok(())
    }

    /// The segment and control registers.
    ///
    /// # Errors
    ///
    /// [`AccelError::Sys`] if `KVM_GET_SREGS` fails.
    pub fn sregs(&self) -> AccelResult<KvmSregs> {
        let inner = self.inner.lock();
        let mut sregs = KvmSregs::default();
        ioctl_struct(&inner.fd, KVM_GET_SREGS, &mut sregs).map_err(sys_err("KVM_GET_SREGS"))?;
        Ok(sregs)
    }

    /// Set the segment and control registers.
    ///
    /// # Errors
    ///
    /// [`AccelError::Sys`] if `KVM_SET_SREGS` fails.
    pub fn set_sregs(&self, sregs: &KvmSregs) -> AccelResult<()> {
        let inner = self.inner.lock();
        let mut sregs = *sregs;
        ioctl_struct(&inner.fd, KVM_SET_SREGS, &mut sregs).map_err(sys_err("KVM_SET_SREGS"))?;
        Ok(())
    }

    /// The x87 and SSE register files.
    ///
    /// # Errors
    ///
    /// [`AccelError::Sys`] if `KVM_GET_FPU` fails.
    pub fn fpu(&self) -> AccelResult<KvmFpu> {
        let inner = self.inner.lock();
        let mut fpu = KvmFpu::default();
        ioctl_struct(&inner.fd, KVM_GET_FPU, &mut fpu).map_err(sys_err("KVM_GET_FPU"))?;
        Ok(fpu)
    }

    /// Set the x87 and SSE register files.
    ///
    /// # Errors
    ///
    /// [`AccelError::Sys`] if `KVM_SET_FPU` fails.
    pub fn set_fpu(&self, fpu: &KvmFpu) -> AccelResult<()> {
        let inner = self.inner.lock();
        let mut fpu = *fpu;
        ioctl_struct(&inner.fd, KVM_SET_FPU, &mut fpu).map_err(sys_err("KVM_SET_FPU"))?;
        Ok(())
    }

    /// The debug registers.
    ///
    /// # Errors
    ///
    /// [`AccelError::Sys`] if `KVM_GET_DEBUGREGS` fails.
    pub fn debugregs(&self) -> AccelResult<KvmDebugregs> {
        let inner = self.inner.lock();
        let mut dregs = KvmDebugregs::default();
        ioctl_struct(&inner.fd, KVM_GET_DEBUGREGS, &mut dregs)
            .map_err(sys_err("KVM_GET_DEBUGREGS"))?;
        Ok(dregs)
    }

    /// Set the debug registers.
    ///
    /// # Errors
    ///
    /// [`AccelError::Sys`] if `KVM_SET_DEBUGREGS` fails.
    pub fn set_debugregs(&self, dregs: &KvmDebugregs) -> AccelResult<()> {
        let inner = self.inner.lock();
        let mut dregs = *dregs;
        ioctl_struct(&inner.fd, KVM_SET_DEBUGREGS, &mut dregs)
            .map_err(sys_err("KVM_SET_DEBUGREGS"))?;
        Ok(())
    }

    /// Read `N` model-specific registers, in the order asked for.
    ///
    /// # Errors
    ///
    /// [`AccelError::Sys`] if the ioctl fails, [`AccelError::Unsupported`] if
    /// the kernel read fewer than asked — which is how it reports an MSR this
    /// host does not implement, and which a caller must not read as a zero.
    pub fn msrs<const N: usize>(&self, indices: [u32; N]) -> AccelResult<[u64; N]> {
        let inner = self.inner.lock();
        let mut msrs = KvmMsrs::<N> {
            nmsrs: N as u32,
            pad: 0,
            entries: indices.map(|index| KvmMsrEntry {
                index,
                reserved: 0,
                data: 0,
            }),
        };
        let read =
            ioctl_msrs(&inner.fd, KVM_GET_MSRS, &mut msrs).map_err(sys_err("KVM_GET_MSRS"))?;
        if read != N as i64 {
            return Err(AccelError::Unsupported(
                "this host does not implement one of the model-specific registers asked for",
            ));
        }
        Ok(msrs.entries.map(|e| e.data))
    }

    /// Write `N` model-specific registers.
    ///
    /// # Errors
    ///
    /// [`AccelError::Sys`] if the ioctl fails, [`AccelError::Unsupported`] if
    /// the kernel accepted fewer than were offered.
    pub fn set_msrs<const N: usize>(&self, values: [(u32, u64); N]) -> AccelResult<()> {
        let inner = self.inner.lock();
        let mut msrs = KvmMsrs::<N> {
            nmsrs: N as u32,
            pad: 0,
            entries: values.map(|(index, data)| KvmMsrEntry {
                index,
                reserved: 0,
                data,
            }),
        };
        let set =
            ioctl_msrs(&inner.fd, KVM_SET_MSRS, &mut msrs).map_err(sys_err("KVM_SET_MSRS"))?;
        if set != N as i64 {
            return Err(AccelError::Unsupported(
                "this host refused one of the model-specific registers offered",
            ));
        }
        Ok(())
    }

    /// Whether the guest currently has interrupts enabled, as of the last
    /// exit.
    #[must_use]
    pub fn interrupts_enabled(&self) -> bool {
        self.inner.lock().run.load_u8(run::IF_FLAG) == Some(1)
    }

    /// Whether the guest would accept an injected vector right now, as of the
    /// last exit.
    #[must_use]
    pub fn ready_for_interrupt(&self) -> bool {
        self.inner.lock().run.load_u8(run::READY_FOR_INTERRUPT) == Some(1)
    }

    /// Ask to be let out of the guest as soon as it can take an interrupt.
    ///
    /// The userspace-interrupt-controller half of the design: a board's
    /// 8259A or local APIC is a [`Device`](crate::core::Device) here, so KVM
    /// has no interrupt state of its own to consult and has to be told when
    /// there is something waiting. Setting this produces a
    /// `KVM_EXIT_IRQ_WINDOW_OPEN` the moment `IF` and the interrupt shadow
    /// allow, and [`inject`](Vcpu::inject) is then accepted.
    pub fn request_interrupt_window(&self, want: bool) {
        let inner = self.inner.lock();
        inner
            .run
            .store_u8(run::REQUEST_INTERRUPT_WINDOW, u8::from(want));
    }

    /// Inject one interrupt vector.
    ///
    /// The vector is the one the board's own interrupt controller supplied on
    /// its acknowledge cycle — nothing here invents it.
    ///
    /// # Errors
    ///
    /// [`AccelError::Sys`] if `KVM_INTERRUPT` fails, which it does with
    /// `EEXIST` when one is already pending and `EINVAL` for a vector above
    /// 255.
    pub fn inject(&self, vector: u8) -> AccelResult<()> {
        let inner = self.inner.lock();
        let mut irq = KvmInterrupt {
            irq: u32::from(vector),
        };
        ioctl_struct(&inner.fd, KVM_INTERRUPT, &mut irq).map_err(sys_err("KVM_INTERRUPT"))?;
        Ok(())
    }

    /// Enter the guest once and report what came back, **without** routing
    /// anything.
    ///
    /// The raw form, for a test that wants to see an exit before this module
    /// has interpreted it. [`Vcpu::run_to_exit`] is what a machine uses.
    ///
    /// # Errors
    ///
    /// [`AccelError::Sys`] if `KVM_RUN` fails for a reason other than being
    /// interrupted.
    pub fn run_once(&self) -> AccelResult<RawExit> {
        let inner = self.inner.lock();
        self.enter(&inner)
    }

    /// One `KVM_RUN`, with the stop flag written through to `immediate_exit`
    /// first.
    fn enter(&self, inner: &Inner) -> AccelResult<RawExit> {
        let stopping = self.exit_flag.raised();
        // The signal-free half of the safe-point protocol: the kernel re-reads
        // this byte with interrupts disabled immediately before entering the
        // guest, so setting it here cannot lose a race against a stop request
        // the way a purely userspace check would. See the module docs.
        if self.immediate_exit {
            inner.run.store_u8(run::IMMEDIATE_EXIT, u8::from(stopping));
        } else if stopping {
            // No `KVM_CAP_IMMEDIATE_EXIT` on this kernel. Declining to enter at
            // all is still correct — it is only the *race* that is no longer
            // closed, and the window is one userspace check wide.
            self.stats.lock().declined += 1;
            return Ok(RawExit::Interrupted);
        }

        let result = ioctl_val(&inner.fd, KVM_RUN, 0);
        if self.immediate_exit {
            inner.run.store_u8(run::IMMEDIATE_EXIT, 0);
        }
        match result {
            Ok(_) => {}
            Err(Errno::EINTR) => {
                self.stats.lock().declined += 1;
                return Ok(RawExit::Interrupted);
            }
            Err(errno) => {
                return Err(AccelError::Sys {
                    what: "KVM_RUN",
                    errno,
                });
            }
        }
        self.stats.lock().entries += 1;

        let reason = u32::from_le_bytes(
            inner
                .run
                .load_le::<4>(run::EXIT_REASON)
                .ok_or(AccelError::Unsupported("the kvm_run page is too small"))?,
        );
        Ok(RawExit::Reason(reason))
    }

    /// Service a `KVM_EXIT_IO`, routing it into the I/O address space.
    fn service_pio(&self, inner: &Inner) -> AccelResult<()> {
        let direction = inner.run.load_u8(run::IO_DIRECTION).unwrap_or(0);
        let size = u64::from(inner.run.load_u8(run::IO_SIZE).unwrap_or(0));
        let port = u64::from(u16::from_le_bytes(
            inner.run.load_le::<2>(run::IO_PORT).unwrap_or_default(),
        ));
        let count = u64::from(u32::from_le_bytes(
            inner.run.load_le::<4>(run::IO_COUNT).unwrap_or_default(),
        ));
        let data_offset = u64::from_le_bytes(
            inner
                .run
                .load_le::<8>(run::IO_DATA_OFFSET)
                .unwrap_or_default(),
        );
        if size == 0 || size > 8 {
            return Err(AccelError::Unsupported(
                "a port access of an implausible width",
            ));
        }
        let Some(io) = self.io.as_ref() else {
            return Err(AccelError::Unsupported(
                "the guest touched a port and this vCPU was given no I/O address space",
            ));
        };

        let attrs = MemAttrs::DEFAULT.with_requester(self.requester);
        let mut buf = [0u8; 8];
        for i in 0..count {
            // A string instruction's transfers sit end to end in the page.
            let at = data_offset + i * size;
            if direction == IO_OUT {
                for b in 0..size {
                    buf[b as usize] = inner.run.load_u8(at + b).unwrap_or(0);
                }
                io.write_bytes(port, &buf[..size as usize], attrs)
                    .map_err(|err| AccelError::Bus { addr: port, err })?;
            } else {
                io.read_bytes(port, &mut buf[..size as usize], attrs)
                    .map_err(|err| AccelError::Bus { addr: port, err })?;
                for b in 0..size {
                    inner.run.store_u8(at + b, buf[b as usize]);
                }
            }
        }
        self.stats.lock().pio += count;
        Ok(())
    }

    /// Service a `KVM_EXIT_MMIO`, routing it into the memory address space.
    fn service_mmio(&self, inner: &Inner) -> AccelResult<()> {
        let addr = u64::from_le_bytes(
            inner
                .run
                .load_le::<8>(run::MMIO_PHYS_ADDR)
                .unwrap_or_default(),
        );
        let len = u32::from_le_bytes(inner.run.load_le::<4>(run::MMIO_LEN).unwrap_or_default());
        let is_write = inner.run.load_u8(run::MMIO_IS_WRITE) == Some(1);
        if len == 0 || len > 8 {
            return Err(AccelError::Unsupported(
                "an MMIO access of an implausible width",
            ));
        }
        let len = len as usize;

        let attrs = MemAttrs::DEFAULT.with_requester(self.requester);
        let mut buf = [0u8; 8];
        if is_write {
            for (b, slot) in buf[..len].iter_mut().enumerate() {
                *slot = inner.run.load_u8(run::MMIO_DATA + b as u64).unwrap_or(0);
            }
            self.memory
                .write_bytes(addr, &buf[..len], attrs)
                .map_err(|err| AccelError::Bus { addr, err })?;
        } else {
            self.memory
                .read_bytes(addr, &mut buf[..len], attrs)
                .map_err(|err| AccelError::Bus { addr, err })?;
            for (b, byte) in buf[..len].iter().enumerate() {
                inner.run.store_u8(run::MMIO_DATA + b as u64, *byte);
            }
        }
        self.stats.lock().mmio += 1;
        Ok(())
    }

    /// Run, servicing MMIO and PIO in place, until something the caller has to
    /// deal with happens.
    ///
    /// # Errors
    ///
    /// [`AccelError`] for a failure of the backend itself. A *guest*-caused
    /// stop is not an error: it comes back as an [`Exit`].
    pub fn run_until_exit(&self, max_entries: u64) -> AccelResult<Run> {
        let inner = self.inner.lock();
        let mut entries = 0u64;
        loop {
            if entries >= max_entries {
                return Ok(Run::completed(Consumed::new(entries)));
            }
            entries += 1;
            let raw = self.enter(&inner)?;
            let reason = match raw {
                // A stop request, honoured. Not an exit: nothing happened to
                // the guest and resuming is unconditional, which is exactly
                // what `Run::completed` means (`core::exec`).
                RawExit::Interrupted => return Ok(Run::completed(Consumed::new(entries))),
                RawExit::Reason(reason) => reason,
            };
            let pc = self.rip(&inner);
            match reason {
                exit::IO => self.service_pio(&inner)?,
                exit::MMIO => self.service_mmio(&inner)?,
                // Nothing to do and nothing to report: the guest was let out so
                // an interrupt could be injected, and the next entry does it.
                exit::IRQ_WINDOW_OPEN => {}
                exit::HLT => {
                    return Ok(Run::exited(
                        Consumed::new(entries),
                        Exit::new(ExitReason::HALT, pc, 0),
                    ));
                }
                exit::SHUTDOWN | exit::SYSTEM_EVENT => {
                    return Ok(Run::exited(
                        Consumed::new(entries),
                        Exit::new(ExitReason::SHUTDOWN, pc, 0),
                    ));
                }
                exit::INTR => return Ok(Run::completed(Consumed::new(entries))),
                exit::EXCEPTION => {
                    let detail = u64::from_le_bytes(
                        inner
                            .run
                            .load_le::<8>(run::HARDWARE_REASON)
                            .unwrap_or_default(),
                    );
                    return Ok(Run::exited(
                        Consumed::new(entries),
                        Exit::new(ExitReason::FAULT, pc, 0)
                            .with_detail(detail)
                            .with_access(0, Access::None),
                    ));
                }
                exit::FAIL_ENTRY | exit::INTERNAL_ERROR | exit::UNKNOWN => {
                    let detail = u64::from_le_bytes(
                        inner
                            .run
                            .load_le::<8>(run::HARDWARE_REASON)
                            .unwrap_or_default(),
                    );
                    return Ok(Run::exited(
                        Consumed::new(entries),
                        Exit::new(ExitReason::INTERNAL, pc, 0).with_detail(detail),
                    ));
                }
                other => {
                    // An exit this build does not know. `ExitReason` is an open
                    // enumeration precisely so that reporting it is better than
                    // pretending it did not happen (`core::exec`).
                    return Ok(Run::exited(
                        Consumed::new(entries),
                        Exit::new(ExitReason::INTERNAL, pc, 0).with_detail(u64::from(other)),
                    ));
                }
            }
        }
    }

    /// `RIP`, for an [`Exit`]'s `pc`. Zero if the register file cannot be read,
    /// which would already have failed the run.
    fn rip(&self, inner: &Inner) -> u64 {
        let mut regs = KvmRegs::default();
        match ioctl_struct(&inner.fd, KVM_GET_REGS, &mut regs) {
            Ok(_) => regs.rip,
            Err(_) => 0,
        }
    }

    /// Turn this vCPU into something [`Scheduler::add_runnable`] accepts —
    /// **only** in a threading mode that does not claim reproducibility.
    ///
    /// The refusal is structural on purpose. An accelerated run's instruction
    /// timing, interrupt instants and TSC all come from the host, so a state
    /// hash taken over one would be a number a regression suite would then
    /// bless. `ROADMAP.md` §0 makes determinism a property of the *mode*, and
    /// [`ThreadingMode::is_deterministic`] is the predicate everything that
    /// depends on it asks; this asks the same one.
    ///
    /// [`Scheduler::add_runnable`]: crate::core::sched::Scheduler::add_runnable
    ///
    /// # Errors
    ///
    /// [`AccelError::Nondeterministic`] for a mode that claims reproducibility.
    pub fn into_runnable(self, mode: ThreadingMode) -> AccelResult<VcpuRunnable> {
        if mode.is_deterministic() {
            return Err(AccelError::Nondeterministic(mode));
        }
        Ok(VcpuRunnable { vcpu: self })
    }
}

/// What one `KVM_RUN` produced, before it is interpreted.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RawExit {
    /// The guest was not entered, or was interrupted before it could run.
    Interrupted,
    /// `kvm_run.exit_reason`.
    Reason(u32),
}

impl ExitingCore for Vcpu {
    fn exit_mask(&self) -> ExitMask {
        *self.mask.lock()
    }

    fn set_exit_mask(&self, mask: ExitMask) {
        *self.mask.lock() = mask;
    }

    fn run_to_exit(&self, budget: Budget) -> Run {
        // `Budget` is in ticks of the core's own clock domain, and a KVM vCPU
        // has no such counter — the host's silicon is the clock. In
        // `ThreadingMode::Accel` virtual time is slaved to the host clock and
        // the scheduler is a deadline service (`ROADMAP.md` §4.2), so the
        // budget is used for what it can honestly mean here: a bound on how
        // many guest entries one call makes before returning to the caller.
        match self.run_until_exit(budget.ticks.max(1)) {
            Ok(run) => run,
            Err(_) => Run::exited(
                Consumed::new(0),
                Exit::new(ExitReason::INTERNAL, self.pc(), 0),
            ),
        }
    }

    fn pc(&self) -> u64 {
        self.regs().map(|r| r.rip).unwrap_or(0)
    }

    fn set_pc(&self, pc: u64) {
        if let Ok(mut regs) = self.regs() {
            regs.rip = pc;
            let _ = self.set_regs(&regs);
        }
    }

    fn sp(&self) -> u64 {
        self.regs().map(|r| r.rsp).unwrap_or(0)
    }

    fn set_sp(&self, sp: u64) {
        if let Ok(mut regs) = self.regs() {
            regs.rsp = sp;
            let _ = self.set_regs(&regs);
        }
    }
}

/// A [`Vcpu`] the scheduler can hand budgets to.
///
/// Only reachable through [`Vcpu::into_runnable`], which refuses a
/// deterministic threading mode — so this type existing at all is evidence
/// that the caller asked for a non-reproducible run.
#[derive(Debug)]
pub struct VcpuRunnable {
    vcpu: Vcpu,
}

impl VcpuRunnable {
    /// The vCPU underneath, for register access and diagnostics.
    #[must_use]
    pub const fn vcpu(&self) -> &Vcpu {
        &self.vcpu
    }
}

impl crate::core::sched::Runnable for VcpuRunnable {
    fn run(&mut self, budget: Budget) -> Consumed {
        // Report the whole budget however many entries it took: under
        // `ThreadingMode::Accel` the guest's progress is measured by the host
        // clock, not by a tick counter this backend could produce, and
        // reporting *less* than the budget would make virtual time crawl while
        // the guest ran at full speed.
        let _ = self.vcpu.run_until_exit(budget.ticks.max(1));
        Consumed::new(budget.ticks)
    }
}

#[cfg(test)]
mod tests;
