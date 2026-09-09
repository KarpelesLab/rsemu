//! The engine-independent architectural CPU state, for x86.
//!
//! `ROADMAP.md` phase 7: *"Snapshot compatibility across an engine switch also
//! requires an **engine-independent architectural CPU-state model**"*, and its
//! gate: *"snapshots taken under KVM restore under the JIT and vice versa"*.
//!
//! This module is that model's x86 half, and it exists because the two engines
//! already agree about almost everything. [`cpu::x86`](crate::cpu::x86) keeps
//! the segment registers as a selector plus a **cached** base, limit and access
//! rights — which is what the silicon does, and what `struct kvm_sregs` reports
//! — so the translation is field for field rather than a re-derivation. That
//! agreement is not a coincidence: both were written from the same *Intel SDM*
//! description of what a segment register holds.
//!
//! # What round-trips today
//!
//! | | rsemu | KVM | carried |
//! | --- | --- | --- | --- |
//! | `RAX`–`RDI`, `R8`–`R15`, `RIP`, `RFLAGS` | [`Regs`] | `kvm_regs` | **yes** |
//! | six segment registers, with base, limit and access rights | [`Sys::segs`] | `kvm_sregs` | **yes** |
//! | `GDTR`, `IDTR` | [`Sys::gdtr`], [`Sys::idtr`] | `kvm_sregs.gdt/idt` | **yes** |
//! | `LDTR`, `TR`, with their caches | [`Sys::ldtr`], [`Sys::task`] | `kvm_sregs.ldt/tr` | **yes** |
//! | `CR0`, `CR2`, `CR3`, `CR4`, `EFER` | [`Sys`] | `kvm_sregs` | **yes** |
//! | `STAR`, `LSTAR`, `CSTAR`, `SFMASK`, `KERNEL_GS_BASE` | [`Sys`] | `KVM_GET_MSRS` | **yes** |
//! | `IA32_MISC_ENABLE` | [`Sys::misc_enable`] | `KVM_GET_MSRS` | **the writable bits** |
//! | `IA32_MTRR_DEF_TYPE`, eleven fixed and eight variable ranges | [`Sys::mtrr_var`], [`Sys::mtrr_fix`] | `KVM_GET_MSRS` | **yes** |
//! | `IA32_TSC` | [`X86::cycles`] | `KVM_GET_MSRS` | **yes, as a value** |
//! | `DR0`–`DR3`, `DR6`, `DR7` | [`Sys::dr`] | `KVM_GET_DEBUGREGS` | **yes** |
//! | the x87 file, control, status and tag words | [`X87`] | `KVM_GET_FPU` | **yes** |
//! | `XMM0`–`XMM15` | [`Sse`] | `KVM_GET_FPU` | **yes** |
//! | `MXCSR` | [`Sse::mxcsr`] | `KVM_GET_FPU` | **out of hardware only** |
//! | the **interrupt shadow** | `int_shadow` | `KVM_GET_VCPU_EVENTS` | **yes** |
//! | the local APIC | `dev::pc::apic` | *not the kernel's* | **by construction** |
//! | `CR8`, the task-priority class | `dev::pc::apic` | `kvm_run.cr8` | **reconciled by `accel::cpu`** |
//! | `apic_base` | `core::wire::LocalController` | `kvm_sregs` | **carried, not applied** |
//!
//! # The local APIC is not on the honest list, and that is worth explaining
//!
//! `ROADMAP.md` phase 7 names "LAPIC/x2APIC state" as part of this deliverable,
//! and there is no `KVM_GET_LAPIC` anywhere in [`kvm`](super::kvm). That is not
//! an omission. This backend **never issues `KVM_CREATE_IRQCHIP`**, so a vCPU
//! here has no in-kernel local APIC to read out; the board's local APIC is
//! `dev::pc::apic`, an ordinary rsemu device with an ordinary chunk, and *both*
//! engines reach it the same way — through a store to its register page in the
//! address space the machine file mapped it into. There is consequently nothing
//! to translate: the same object, saved by the same `save`, restored by the same
//! `load`, whichever engine was running the processor beside it. `CR8` is the
//! same story told about one register, and [`tpr_through_space`] is the route.
//!
//! An in-kernel irqchip would make the APIC's state a *third* representation to
//! keep in step and would need a `KVM_GET_LAPIC`/`KVM_SET_LAPIC` translation of
//! its own. It would also move interrupt delivery out of the board, where
//! `machines/pc-apic.machine` puts it. Not doing it is the design.
//!
//! # The time-stamp counter: a value, never an offset
//!
//! The counter is the subtle one, and the shape of the answer is fixed by
//! `CLAUDE.md`'s rule that derived state is never serialized.
//!
//! A hypervisor implements `RDTSC` as the host's own counter plus a per-vCPU
//! **offset**. That offset is derived — from the value the guest should see and
//! from the host counter at the instant of the write — so it is meaningless on
//! any host but the one that computed it, and putting one in a snapshot would
//! produce a save state that could only be restored on the machine that took
//! it. What is architectural is the **value**: what a `RDTSC` in the guest
//! returns. Both engines have exactly that — `KVM_GET_MSRS` of `IA32_TSC` on
//! one side, [`X86::cycles`] on the other, because that is where this core's
//! `RDMSR` of `msr::TSC` reads from — so the value is what crosses, and the
//! destination engine recomputes whatever offset it needs. On KVM that
//! recomputation is a `KVM_SET_MSRS` of `IA32_TSC` and nothing else.
//!
//! **When** it is written matters as much as what. [`store_from_vcpu`] reads it
//! after every hardware slice, because otherwise a `save` between rounds would
//! write a counter belonging to whatever the interpreter last did.
//! [`load_into_vcpu`] deliberately does **not** write it, because that call runs
//! at the top of every slice whose shell is ahead — after a reset sequence,
//! after an instruction hardware could not fetch — and writing the counter there
//! would rewind the guest's clock by the userspace time between the store and
//! the load, on every slice. [`restore_into_vcpu`] is the call that does write
//! it, and `accel::cpu` selects it at the two moments where the offset genuinely
//! has to be recomputed: a snapshot load, and a reset.
//!
//! **What is not continuous is the rate.** An accelerated guest's counter
//! advances at `KVM_GET_TSC_KHZ`, roughly the host's; the interpreter's advances
//! at four ticks per bus cycle plus the manual's execution figures. A guest that
//! has calibrated its counter against a periodic timer — Linux does, against the
//! PIT or the HPET — and is then restored under the other engine will find its
//! calibration wrong, and will recalibrate or mark the clocksource unstable. The
//! rate is a property of the machine rather than of the guest, so no snapshot
//! field can fix it; a board that wants an engine switch to be invisible has to
//! pin the two rates together, which is a machine-configuration deliverable and
//! not this module's.
//!
//! # What does not cross, and why — the honest list
//!
//! Five things, and each says what closing it would take.
//!
//! * **`apic_base`.** Also the device's, and here the crate *does* have a seam:
//!   [`LocalController::base_register`](crate::core::wire::LocalController::base_register)
//!   is exactly a route from a core to the sibling that owns `IA32_APIC_BASE`,
//!   and `X86` already forwards `RDMSR`/`WRMSR` of it through that link. What
//!   is missing is not the route but a *holder*: [`store_from_vcpu`] is handed a
//!   core and a vCPU and has no third party to ask. [`ArchState`] carries the
//!   value so that a caller who has the link can apply it.
//! * **`MXCSR`, in the *into*-hardware direction.** `KVM_GET_FPU` reports it
//!   and `KVM_SET_FPU` does not write it — the kernel's set path fills the
//!   `fxsave` legacy area field by field and that one field is not among them.
//!   Measured, not assumed: `the_syscall_msrs_the_debug_registers_and_the_fpu_survive_hardware`
//!   sets it, reads it back, and finds it unchanged, which is why
//!   [`differs`] excludes it. A guest restored *onto* an accelerator therefore
//!   keeps whatever rounding mode and exception masks the vCPU already had, so
//!   a restored `DIVPS` can round the wrong way once. **What closing it takes:**
//!   `KVM_GET_XSAVE`/`KVM_SET_XSAVE` in place of `KVM_GET_FPU`, which is the
//!   next bullet, because the `XSAVE` legacy area *does* carry `MXCSR` and the
//!   kernel's set path writes it wholesale.
//! * **`XSAVE` beyond x87 and SSE.** `KVM_GET_FPU` is used rather than
//!   `KVM_GET_XSAVE` because the interpreter models exactly the state
//!   `kvm_fpu` holds: [`Sse`] is sixteen 128-bit lanes and there is no `YMM`
//!   upper half or `ZMM` file in `cpu::x86` for a wider component to land in.
//!   **What closing it takes**, in the order it would have to happen: (1) an
//!   AVX register file in `cpu::x86::fpu` and the instructions that use it,
//!   which is an ISA deliverable rather than a snapshot one; (2) `XCR0` — which
//!   is `KVM_GET_XCRS`, a separate ioctl — because the `XSAVE` area's layout is
//!   a function of it and a component's offset is not fixed; (3) the *standard*
//!   `XSAVE` format's component walk, since `KVM_GET_XSAVE`'s 4096-byte buffer
//!   holds a header plus components at `CPUID.(EAX=0Dh)` offsets, so a
//!   translation cannot be a struct-to-struct copy the way `kvm_fpu` is; (4) the
//!   chunk grows, and *that* is the change that would need a `cpu.x86` class
//!   version bump and a migration. Until (1) exists there is nothing to carry
//!   and the ioctl would move bytes into a file no instruction reads.
//! * **The x87 last-instruction selectors.** `kvm_fpu` carries `last_ip` and
//!   `last_dp` but not the `CS` and `DS` that went with them, so
//!   [`X87::last_cs`] and [`X87::last_ds`] are preserved from the destination
//!   rather than transferred. Only `FNSTENV` in a 16- or 32-bit form can
//!   observe the difference. **What closing it takes:** `KVM_GET_XSAVE` again —
//!   the legacy area has the two selector fields that `kvm_fpu` dropped.
//! * **The rest of `kvm_vcpu_events`.** The **interrupt shadow** now crosses —
//!   see below — but the structure carries three other things and none of them
//!   has anywhere to land: a **pending or injected exception** with its error
//!   code and payload, the **pending `NMI`**, and the **`NMI`-blocked** flag
//!   that says the processor is between an `NMI` and its `IRET`. The
//!   interpreter has no field for any of the three, and that is a statement
//!   about `cpu::x86` rather than about the bridge: it delivers an exception
//!   *within* the step that raised it, so between two instructions there is
//!   never one owed; its `NMI` latch lives on the pin rather than in the
//!   execution state; and it does not model `NMI` blocking at all — its
//!   `int_shadow` suppresses `NMI` and `INTR` together, which is `MOV SS`
//!   semantics rather than `IRET` semantics. [`overlay_events`] therefore
//!   *preserves* all three rather than zeroing them, so a vCPU that owed the
//!   guest a `#PF` still owes it after a slice-boundary push. **What closing
//!   them takes** is not an ioctl — it is a pending-event field in `cpu::x86`'s
//!   execution state and a chunk version to hold it, which is a core
//!   deliverable rather than a translation one.
//!
//! # The interrupt shadow, which used to be on that list
//!
//! It was the sharpest item there and it is now carried, so what follows is why
//! it needed its own ioctl and how the one boolean maps onto two bits.
//!
//! The shadow is the single instruction after a `STI`, a `MOV SS` or a `POP SS`
//! during which an external interrupt is not recognised (*Intel SDM* volume 2B,
//! `STI`; volume 3A §6.8.3). It exists so that `STI; HLT` cannot lose a wakeup
//! and so that the second half of an `SS:SP` reload cannot be interrupted. It
//! is not in `RFLAGS`, not in a control register and not in any model-specific
//! register: on hardware it is the VMCS's *interruptibility state* or the
//! VMCB's `INT_STATE`, and `KVM_GET_VCPU_EVENTS` is the only request that
//! reports it. So a bridge built out of `KVM_GET_REGS`, `KVM_GET_SREGS` and
//! `KVM_GET_MSRS` could not see it at any level of care.
//!
//! **It is the same shape of defect as the memory-type ranges**, and it is
//! worth naming the shape: a field *both* engines drop looks identical on both
//! sides, so an engine-versus-engine comparison of chunks passes. What catches
//! it is a guest that observes its own state — here, one whose interrupt
//! handler reads the `RIP` it interrupted off its own stack frame and so says
//! whether the shadowed instruction ran first.
//! `tests/x86_arch_state.rs`'s `shadow_guest` is that program.
//!
//! One boolean, two bits. `cpu::x86` has a single `int_shadow` where a
//! hypervisor distinguishes blocking-by-`STI` from blocking-by-`MOV SS`; the
//! way *out* of hardware is therefore "either bit means shadowed", and the way
//! *in* picks `MOV SS` for the reasons [`SHADOW_INTO_HARDWARE`] gives. An AMD
//! part does not distinguish them at all and reports both.
//!
//! # `CR8`, which also used to be on that list, is not a translation
//!
//! It is on the table above with a different verdict from everything else —
//! *reconciled by `accel::cpu`* — and the reason is that `CR8` was never this
//! module's to carry. It is the top nibble of the local APIC's task-priority
//! register seen through the processor (*Intel SDM* volume 3A §11.8.6.1), and
//! in this crate that APIC is a **device**. `cpu::x86` therefore has no `cr8`
//! field at all: `prot::Exec::write_task_priority` stores to the register page
//! whichever route the guest took, so there is one home for the byte and the
//! snapshot already carries it — in the APIC's chunk.
//!
//! On hardware there are two homes. `MOV CR8` lands in the vCPU without an
//! exit; a store to the APIC's page leaves hardware as MMIO and lands in the
//! device. Neither sees the other, so what was needed was not a field in
//! [`ArchState`] but for the two copies to agree — and the party that can make
//! them agree is the one holding both the memory space and the
//! [`LocalController`] link, which is `accel::cpu`. `AccelCpu::sync_task_priority`
//! is that reconciliation: a two-way compare against the value the two sides
//! last agreed on, once per slice, with the store made outside every lock this
//! crate holds. [`tpr_through_space`] and [`tpr_from_space`] are the two routes
//! it uses, and `kvm_run.cr8` — which `api.rst` documents as *"in
//! (pre_kvm_run), out (post_kvm_run)"* and valid only without an in-kernel
//! local APIC — is the vCPU's side of it, at the cost of no ioctl at all.
//!
//! [`X87`]: crate::cpu::x86::fpu::X87
//! [`Sys::misc_enable`]: crate::cpu::x86::prot::Sys::misc_enable
//! [`Sys::mtrr_var`]: crate::cpu::x86::prot::Sys::mtrr_var
//! [`Sys::mtrr_fix`]: crate::cpu::x86::prot::Sys::mtrr_fix
//! [`Sys::dr`]: crate::cpu::x86::prot::Sys::dr
//! [`X86::cycles`]: crate::cpu::x86::X86::cycles
//! [`LocalController`]: crate::core::wire::LocalController
//! [`Sse`]: crate::cpu::x86::fpu::Sse
//! [`Sse::mxcsr`]: crate::cpu::x86::fpu::Sse::mxcsr
//! [`X87::last_cs`]: crate::cpu::x86::fpu::X87::last_cs
//! [`X87::last_ds`]: crate::cpu::x86::fpu::X87::last_ds

use crate::cpu::x86::fpu::{Sse, Tag, X87};
use crate::cpu::x86::isa::seg;
use crate::cpu::x86::prot::{SegReg, Sys, TableReg, ar, misc_enable, msr, mtrr};
use crate::cpu::x86::{Regs, X86};
use crate::float::x87::F80;

use super::AccelResult;
use super::kvm::{
    KvmDebugregs, KvmDtable, KvmFpu, KvmRegs, KvmSegment, KvmSregs, KvmVcpuEvents, Vcpu, events,
};

// ---------------------------------------------------------------------------
// registers
// ---------------------------------------------------------------------------

/// The interpreter's general-purpose file as KVM's.
#[must_use]
pub fn regs_to_kvm(regs: &Regs) -> KvmRegs {
    KvmRegs {
        rax: regs.rax,
        rbx: regs.rbx,
        rcx: regs.rcx,
        rdx: regs.rdx,
        rsi: regs.rsi,
        rdi: regs.rdi,
        rsp: regs.rsp,
        rbp: regs.rbp,
        r: regs.r,
        rip: regs.rip,
        rflags: u64::from(regs.eflags),
    }
}

/// KVM's general-purpose file as the interpreter's.
///
/// **Takes the `kvm_sregs` too, and that is not a convenience.** rsemu holds a
/// segment selector in *two* places — [`Regs::cs`] and the selector inside
/// [`Sys::segs`] — where KVM holds it in one, and [`X86::set_regs`] recomputes
/// the real-mode segment bases from the [`Regs`] copy. A bridge that filled
/// only the [`Sys`] half therefore left `CS` at whatever the interpreter had
/// been reset to, and the next instruction fetched from the wrong megabyte.
/// That was a real bug in this module, found by
/// `the_interpreters_state_loads_into_a_vcpu_and_runs`, and the signature is
/// the shape that makes it unrepresentable.
#[must_use]
#[allow(clippy::cast_possible_truncation)]
pub fn regs_from_kvm(kvm: &KvmRegs, sregs: &KvmSregs) -> Regs {
    Regs {
        rax: kvm.rax,
        rbx: kvm.rbx,
        rcx: kvm.rcx,
        rdx: kvm.rdx,
        rsi: kvm.rsi,
        rdi: kvm.rdi,
        rsp: kvm.rsp,
        rbp: kvm.rbp,
        r: kvm.r,
        rip: kvm.rip,
        // `RFLAGS` bits 63-32 are reserved and read as zero (*Intel SDM*
        // volume 1 §3.4.3), so the truncation discards nothing.
        eflags: kvm.rflags as u32,
        es: sregs.es.selector,
        cs: sregs.cs.selector,
        ss: sregs.ss.selector,
        ds: sregs.ds.selector,
        fs: sregs.fs.selector,
        gs: sregs.gs.selector,
    }
}

// ---------------------------------------------------------------------------
// segments
// ---------------------------------------------------------------------------

/// One segment register, unpacked into the fields `kvm_segment` names.
///
/// rsemu keeps the access rights packed at their hardware bit positions, which
/// is what `LAR` returns and what a descriptor holds; KVM splits them into a
/// byte per field. Neither is more correct — this is the translation.
#[must_use]
pub fn seg_to_kvm(reg: &SegReg) -> KvmSegment {
    // The four-bit type field sits at bits 8-11 of the packed rights.
    let kind = ((reg.ar >> 8) & 0xf) as u8;
    KvmSegment {
        base: reg.base,
        limit: reg.limit,
        selector: reg.selector,
        kind,
        present: u8::from(reg.ar & ar::PRESENT != 0),
        dpl: ((reg.ar & ar::DPL) >> ar::DPL_SHIFT) as u8,
        db: u8::from(reg.ar & ar::DB != 0),
        s: u8::from(reg.ar & ar::S != 0),
        l: u8::from(reg.ar & ar::L != 0),
        g: u8::from(reg.ar & ar::GRANULAR != 0),
        avl: u8::from(reg.ar & ar::AVL != 0),
        // A descriptor with `P` clear is one hardware will not use. KVM wants
        // that stated as a separate flag, because VMX has a distinct
        // "unusable" bit in the guest access-rights field.
        unusable: u8::from(reg.ar & ar::PRESENT == 0),
        padding: 0,
    }
}

/// The same, in reverse.
#[must_use]
pub fn seg_from_kvm(kvm: &KvmSegment) -> SegReg {
    let mut rights = (u32::from(kvm.kind) & 0xf) << 8;
    if kvm.present != 0 && kvm.unusable == 0 {
        rights |= ar::PRESENT;
    }
    rights |= (u32::from(kvm.dpl) << ar::DPL_SHIFT) & ar::DPL;
    if kvm.db != 0 {
        rights |= ar::DB;
    }
    if kvm.s != 0 {
        rights |= ar::S;
    }
    if kvm.l != 0 {
        rights |= ar::L;
    }
    if kvm.g != 0 {
        rights |= ar::GRANULAR;
    }
    if kvm.avl != 0 {
        rights |= ar::AVL;
    }
    SegReg {
        selector: kvm.selector,
        base: kvm.base,
        limit: kvm.limit,
        ar: rights & ar::MASK,
    }
}

#[must_use]
#[allow(clippy::cast_possible_truncation)]
fn table_to_kvm(reg: &TableReg) -> KvmDtable {
    KvmDtable {
        base: reg.base,
        // A descriptor table's limit is sixteen bits in the register and in
        // every instruction that loads one, so this cannot lose anything a
        // real `LGDT` could have written.
        limit: reg.limit as u16,
        padding: [0; 3],
    }
}

#[must_use]
fn table_from_kvm(kvm: &KvmDtable) -> TableReg {
    TableReg {
        base: kvm.base,
        limit: u32::from(kvm.limit),
    }
}

/// The interpreter's system state as `kvm_sregs`.
///
/// `cr8`, `apic_base` and `interrupt_bitmap` are left zero: see the module
/// documentation for why they are not the core's to carry.
#[must_use]
pub fn sys_to_sregs(sys: &Sys) -> KvmSregs {
    KvmSregs {
        cs: seg_to_kvm(&sys.segs[seg::CS as usize]),
        ds: seg_to_kvm(&sys.segs[seg::DS as usize]),
        es: seg_to_kvm(&sys.segs[seg::ES as usize]),
        fs: seg_to_kvm(&sys.segs[seg::FS as usize]),
        gs: seg_to_kvm(&sys.segs[seg::GS as usize]),
        ss: seg_to_kvm(&sys.segs[seg::SS as usize]),
        tr: seg_to_kvm(&sys.task),
        ldt: seg_to_kvm(&sys.ldtr),
        gdt: table_to_kvm(&sys.gdtr),
        idt: table_to_kvm(&sys.idtr),
        cr0: u64::from(sys.cr0),
        cr2: sys.cr2,
        cr3: sys.cr3,
        cr4: sys.cr4,
        cr8: 0,
        efer: sys.efer,
        apic_base: 0,
        interrupt_bitmap: [0; 4],
    }
}

/// `kvm_sregs` as the interpreter's system state.
///
/// `base` supplies the fields KVM does not report — the debug registers, the
/// `SYSCALL` MSRs, the test registers — so a restore keeps whatever the
/// interpreter already had rather than zeroing state the accelerator merely
/// does not know about.
#[must_use]
#[allow(clippy::cast_possible_truncation)]
pub fn sregs_to_sys(kvm: &KvmSregs, base: &Sys) -> Sys {
    let mut sys = *base;
    sys.segs[seg::CS as usize] = seg_from_kvm(&kvm.cs);
    sys.segs[seg::DS as usize] = seg_from_kvm(&kvm.ds);
    sys.segs[seg::ES as usize] = seg_from_kvm(&kvm.es);
    sys.segs[seg::FS as usize] = seg_from_kvm(&kvm.fs);
    sys.segs[seg::GS as usize] = seg_from_kvm(&kvm.gs);
    sys.segs[seg::SS as usize] = seg_from_kvm(&kvm.ss);
    sys.task = seg_from_kvm(&kvm.tr);
    sys.ldtr = seg_from_kvm(&kvm.ldt);
    sys.gdtr = table_from_kvm(&kvm.gdt);
    sys.idtr = table_from_kvm(&kvm.idt);
    // `CR0`'s architectural width is 32 bits even in long mode; the high half
    // of KVM's `u64` is reserved and zero.
    sys.cr0 = kvm.cr0 as u32;
    sys.cr2 = kvm.cr2;
    sys.cr3 = kvm.cr3;
    sys.cr4 = kvm.cr4;
    sys.efer = kvm.efer;
    // The bases KVM keeps in the segment registers are the same two MSRs.
    sys.fs_base = kvm.fs.base;
    sys.gs_base = kvm.gs.base;
    sys
}

// ---------------------------------------------------------------------------
// model-specific registers
// ---------------------------------------------------------------------------

/// How many model-specific registers [`CARRIED_MSRS`] holds.
///
/// The five `SYSCALL` registers, `IA32_MISC_ENABLE`, `IA32_MTRR_DEF_TYPE`, the
/// eleven fixed memory-type ranges and the sixteen halves of the eight variable
/// ones. Thirty-four, and the arithmetic is written out rather than typed as a
/// literal so that a part configured with a different [`mtrr::VCNT`] moves the
/// array rather than silently truncating it.
pub const MSR_COUNT: usize = SYSCALL_MSRS + 2 + mtrr::FIXED_COUNT + 2 * mtrr::VCNT as usize;

/// The `SYSCALL` group's length, and the index of `IA32_MISC_ENABLE`.
const SYSCALL_MSRS: usize = 5;
/// Where `IA32_MISC_ENABLE` sits in [`CARRIED_MSRS`].
const MISC_AT: usize = SYSCALL_MSRS;
/// Where `IA32_MTRR_DEF_TYPE` sits.
const DEF_TYPE_AT: usize = MISC_AT + 1;
/// Where the eleven fixed memory-type ranges start.
const FIXED_AT: usize = DEF_TYPE_AT + 1;
/// Where the sixteen variable-range halves start.
const VAR_AT: usize = FIXED_AT + mtrr::FIXED_COUNT;

/// The model-specific registers **both engines model** and `kvm_sregs` does not
/// carry, in the order the two functions below use.
///
/// `EFER` is not here because it *is* in `kvm_sregs`, and `FS_BASE`/`GS_BASE`
/// are not because KVM reports them as the `fs` and `gs` segment bases — which
/// is where the hardware actually keeps them, and carrying them twice would
/// let the two copies disagree.
///
/// **The memory-type ranges and `IA32_MISC_ENABLE` are here because leaving
/// them out was a measured defect rather than a theoretical one.** A vCPU whose
/// guest had programmed `IA32_MTRR_DEF_TYPE` to `0xc06` and
/// `IA32_MTRR_FIX64K_00000` to all-write-back handed the interpreter two zeros,
/// because nothing read them out — and zero in `MTRR_DEF_TYPE` is not "the
/// default", it is `E = 0`, which the *Intel SDM* volume 3A §12.11.2.1 defines
/// as **every physical address uncacheable**. So a snapshot taken on hardware
/// restored an interpreter describing a machine whose memory types the guest
/// had never asked for, and the reverse restored a vCPU with the ranges its
/// firmware had set wiped back to reset. Both engines already keep every one of
/// these fields and already write them to their chunk; the gap was the bridge.
pub const CARRIED_MSRS: [u32; MSR_COUNT] = carried_msrs();

/// [`CARRIED_MSRS`] built from the *Intel SDM* volume 4 Table 2-2 addresses
/// rather than typed out.
///
/// Thirty-four hand-written hexadecimal constants is thirty-four chances to
/// transpose one, and a transposed memory-type range is invisible in any test
/// that uses a uniform state. The fixed-range order is
/// [`Sys::mtrr_fix`]'s — the one 64 KiB register, the two 16 KiB, the eight
/// 4 KiB — and the variable-range order is `RDMSR`'s, base then mask.
const fn carried_msrs() -> [u32; MSR_COUNT] {
    let mut out = [0u32; MSR_COUNT];
    out[0] = msr::STAR;
    out[1] = msr::LSTAR;
    out[2] = msr::CSTAR;
    out[3] = msr::SFMASK;
    out[4] = msr::KERNEL_GS_BASE;
    out[MISC_AT] = msr::MISC_ENABLE;
    out[DEF_TYPE_AT] = msr::MTRR_DEF_TYPE;
    out[FIXED_AT] = msr::MTRR_FIX64K;
    out[FIXED_AT + 1] = msr::MTRR_FIX16K;
    out[FIXED_AT + 2] = msr::MTRR_FIX16K + 1;
    let mut i = 0;
    while i < mtrr::FIXED_COUNT - 3 {
        out[FIXED_AT + 3 + i] = msr::MTRR_FIX4K + i as u32;
        i += 1;
    }
    let mut i = 0;
    while i < 2 * mtrr::VCNT as usize {
        out[VAR_AT + i] = msr::MTRR_PHYSBASE0 + i as u32;
        i += 1;
    }
    out
}

/// [`CARRIED_MSRS`] plus `IA32_TSC`, which is **read** on every transfer and
/// **written** only on a restore.
///
/// The asymmetry is the whole of the time-stamp counter design and it is
/// argued in the module documentation. One array rather than two calls because
/// `KVM_GET_MSRS` takes a list, and asking for the counter in the same ioctl
/// as the other thirty-four costs nothing.
pub const READ_MSRS: [u32; MSR_COUNT + 1] = read_msrs();

const fn read_msrs() -> [u32; MSR_COUNT + 1] {
    let mut out = [msr::TSC; MSR_COUNT + 1];
    let mut i = 0;
    while i < MSR_COUNT {
        out[i] = CARRIED_MSRS[i];
        i += 1;
    }
    out
}

/// Where `IA32_TSC` sits in a [`READ_MSRS`] result.
pub const TSC_AT: usize = MSR_COUNT;

/// The values of [`CARRIED_MSRS`], read out of the interpreter.
///
/// `IA32_MISC_ENABLE` goes across **whole**, not masked: the accelerator is
/// being told what part it is impersonating, and the bits outside
/// [`misc_enable::WRITABLE`] are the part's own capability statement rather
/// than anything the guest could have written. The reverse direction masks,
/// for exactly the mirrored reason — see [`msrs_from_kvm`].
#[must_use]
pub fn msrs_to_kvm(sys: &Sys) -> [(u32, u64); MSR_COUNT] {
    core::array::from_fn(|i| (CARRIED_MSRS[i], msr_value(sys, i)))
}

/// One carried register's value, by its index in [`CARRIED_MSRS`].
fn msr_value(sys: &Sys, i: usize) -> u64 {
    match i {
        0 => sys.star,
        1 => sys.lstar,
        2 => sys.cstar,
        3 => sys.sfmask,
        4 => sys.kernel_gs_base,
        MISC_AT => sys.misc_enable,
        DEF_TYPE_AT => sys.mtrr_def_type,
        _ if i < VAR_AT => sys.mtrr_fix[i - FIXED_AT],
        _ => sys.mtrr_var[i - VAR_AT],
    }
}

/// The same, in reverse: put the values of [`CARRIED_MSRS`] into `sys`.
///
/// `IA32_MISC_ENABLE` is the one field that is **merged rather than
/// replaced**, and the reason is that not every bit of it is guest state. Bits
/// 11 and 12 say *this part has no branch-trace store and no precise
/// event-based sampling*; a host kernel reports `0x1800` there out of reset
/// and rsemu's core reports `0x1` (`misc_enable::RESET`), because they are
/// describing two different processors. Copying the host's answer in would
/// change what the emulated part claims about itself for no reason the guest
/// asked for. What the guest *can* have changed is exactly
/// [`misc_enable::WRITABLE`] — a `WRMSR` to any other bit raises `#GP(0)` on
/// this core — so those bits transfer and the rest stay the part's.
pub fn msrs_from_kvm(values: &[u64; MSR_COUNT], sys: &mut Sys) {
    sys.star = values[0];
    sys.lstar = values[1];
    sys.cstar = values[2];
    sys.sfmask = values[3];
    sys.kernel_gs_base = values[4];
    sys.misc_enable =
        (sys.misc_enable & !misc_enable::WRITABLE) | (values[MISC_AT] & misc_enable::WRITABLE);
    sys.mtrr_def_type = values[DEF_TYPE_AT];
    for (i, slot) in sys.mtrr_fix.iter_mut().enumerate() {
        *slot = values[FIXED_AT + i];
    }
    for (i, slot) in sys.mtrr_var.iter_mut().enumerate() {
        *slot = values[VAR_AT + i];
    }
}

/// The first [`MSR_COUNT`] of a [`READ_MSRS`] result.
#[must_use]
pub fn carried_of(values: &[u64; MSR_COUNT + 1]) -> [u64; MSR_COUNT] {
    core::array::from_fn(|i| values[i])
}

// ---------------------------------------------------------------------------
// debug registers
// ---------------------------------------------------------------------------

/// The interpreter's debug registers as `kvm_debugregs`.
///
/// `DR4` and `DR5` have no field because they have no existence: with `CR4.DE`
/// clear they alias `DR6` and `DR7`, and with it set they raise `#UD`. rsemu
/// stores eight for the index arithmetic's sake; only six are state.
#[must_use]
pub fn dregs_to_kvm(sys: &Sys) -> KvmDebugregs {
    KvmDebugregs {
        db: [sys.dr[0], sys.dr[1], sys.dr[2], sys.dr[3]],
        dr6: sys.dr[6],
        dr7: sys.dr[7],
        flags: 0,
        reserved: [0; 9],
    }
}

/// The same, in reverse.
pub fn dregs_from_kvm(kvm: &KvmDebugregs, sys: &mut Sys) {
    sys.dr[0] = kvm.db[0];
    sys.dr[1] = kvm.db[1];
    sys.dr[2] = kvm.db[2];
    sys.dr[3] = kvm.db[3];
    sys.dr[6] = kvm.dr6;
    sys.dr[7] = kvm.dr7;
}

// ---------------------------------------------------------------------------
// the x87 and SSE files
// ---------------------------------------------------------------------------

/// The interpreter's floating-point state as `kvm_fpu`.
///
/// The tag word is **abridged** on the way out: `FXSAVE`'s format is one bit
/// per physical register, set when the register is not empty, and the
/// two-bit-per-register word rsemu keeps is what `FNSTENV` writes (*Intel SDM*
/// volume 1 §8.1.7). Nothing is lost going this way — the discarded
/// valid/zero/special distinction is recomputable from the register itself,
/// which is exactly what [`fpu_from_kvm`] does and what `FXRSTOR` does in
/// hardware.
#[must_use]
pub fn fpu_to_kvm(x87: &X87, sse: &Sse) -> KvmFpu {
    let mut fpu = KvmFpu {
        fcw: x87.control,
        fsw: x87.status,
        ftwx: 0,
        last_opcode: x87.last_op & 0x7ff,
        last_ip: x87.last_ip,
        last_dp: x87.last_dp,
        mxcsr: sse.mxcsr,
        ..KvmFpu::default()
    };
    for i in 0..8 {
        fpu.fpr[i][..10].copy_from_slice(&x87.regs[i].to_bytes());
        if Tag::from_bits(x87.tag >> (2 * i)) != Tag::Empty {
            fpu.ftwx |= 1 << i;
        }
    }
    for i in 0..16 {
        fpu.xmm[i][..8].copy_from_slice(&sse.xmm[i][0].to_le_bytes());
        fpu.xmm[i][8..].copy_from_slice(&sse.xmm[i][1].to_le_bytes());
    }
    fpu
}

/// `kvm_fpu` as the interpreter's floating-point state.
///
/// `base` supplies the two fields `kvm_fpu` has no room for — the code and data
/// selectors that went with `last_ip` and `last_dp` — for the same reason
/// [`sregs_to_sys`] takes one.
#[must_use]
pub fn fpu_from_kvm(kvm: &KvmFpu, base: &X87) -> (X87, Sse) {
    let mut x87 = X87 {
        regs: [F80::ZERO; 8],
        control: kvm.fcw,
        status: kvm.fsw,
        tag: 0,
        last_ip: kvm.last_ip,
        last_cs: base.last_cs,
        last_dp: kvm.last_dp,
        last_ds: base.last_ds,
        last_op: kvm.last_opcode & 0x7ff,
    };
    for i in 0..8 {
        let mut bytes = [0u8; 10];
        bytes.copy_from_slice(&kvm.fpr[i][..10]);
        x87.regs[i] = F80::from_bytes(bytes);
        // The abridged word says only *empty or not*. A register that is not
        // empty gets its full tag recomputed from its own encoding, which is
        // what `FXRSTOR` does; one that is empty keeps the encoding it had, as
        // hardware does — an empty register still holds bits.
        let tag = if kvm.ftwx & (1 << i) == 0 {
            Tag::Empty
        } else {
            Tag::of(x87.regs[i])
        };
        x87.tag |= tag.bits() << (2 * i);
    }
    let mut sse = Sse {
        xmm: [[0; 2]; 16],
        mxcsr: kvm.mxcsr,
    };
    for i in 0..16 {
        let mut lo = [0u8; 8];
        let mut hi = [0u8; 8];
        lo.copy_from_slice(&kvm.xmm[i][..8]);
        hi.copy_from_slice(&kvm.xmm[i][8..]);
        sse.xmm[i] = [u64::from_le_bytes(lo), u64::from_le_bytes(hi)];
    }
    (x87, sse)
}

// ---------------------------------------------------------------------------
// interruptibility: the one piece of architectural state no register holds
// ---------------------------------------------------------------------------

/// Which shadow bit this crate writes into hardware, and why it is the
/// *`MOV SS`* one rather than the `STI` one.
///
/// The interpreter has a single boolean where a hypervisor has two bits, so
/// the way back has to choose one. It writes `MOV SS` blocking, for two
/// reasons, and neither is arbitrary.
///
/// **It is the one that means what the interpreter means.** Blocking by `STI`
/// suppresses `INTR` and leaves `NMI` alone; blocking by `MOV SS` suppresses
/// both, because the whole point of that shadow is that an `SS:SP` pair is
/// half-loaded and *nothing* may be pushed onto it (*Intel SDM* volume 3A
/// §6.8.3, "masking exceptions and interrupts when switching stacks").
/// [`cpu::x86`](crate::cpu::x86) checks `int_shadow` ahead of *both* the `NMI`
/// latch and the `INTR` pin — see `exec::Exec::step` — so its shadow is the
/// `MOV SS` one whatever raised it. Writing that bit reproduces the emulated
/// processor rather than a slightly different one.
///
/// **It is also the one a VM entry will always accept.** VMX checks the
/// guest's interruptibility state on entry (*SDM* volume 3C, the checks on
/// guest non-register state), and blocking by `STI` is not a state a processor
/// can be in with `RFLAGS.IF` clear — which an interpreter shadow raised by a
/// `MOV SS` with interrupts already off legitimately is. Two bits describing
/// two different blocking conditions are also not a state the architecture
/// describes, so writing both instead of choosing would be inventing one.
pub const SHADOW_INTO_HARDWARE: u8 = events::SHADOW_INT_MOV_SS;

/// The interrupt shadow, out of a `kvm_vcpu_events` and into the interpreter.
///
/// **The only member of that structure both engines model**, and the module
/// documentation's list says what happens to the other three.
pub fn events_from_kvm(events: &KvmVcpuEvents, cpu: &X86) {
    cpu.set_interrupt_shadow(events.interrupt_shadow());
}

/// The interrupt shadow, onto a `kvm_vcpu_events` the vCPU already has.
///
/// **An overlay, for [`overlay_sys`]'s reason and one of its own.** The
/// pending exception, the pending and blocked `NMI` and the system-management
/// state are all things the accelerator knows and the interpreter has no field
/// for, so a wholesale write would replace them with zeros — a vCPU that owed
/// the guest a `#PF` would silently stop owing it.
///
/// The flags field is where that is expressed rather than in the members: KVM
/// reads `nmi.pending`, `sipi_vector` and the `smi` group **only** when the
/// matching flag is set (`api.rst` §4.32, and it names those four because a
/// running processor changes them underneath userspace), so clearing those
/// three bits is how a client says *keep your own*. What is left set is what
/// the kernel reported on the way out, `KVM_VCPUEVENT_VALID_SHADOW` included —
/// and a host that does not report the shadow leaves that bit clear, which is
/// the whole of the capability check.
pub fn overlay_events(shadowed: bool, out: &mut KvmVcpuEvents) {
    out.flags &= !(events::VCPUEVENT_VALID_NMI_PENDING
        | events::VCPUEVENT_VALID_SIPI_VECTOR
        | events::VCPUEVENT_VALID_SMM);
    if out.shadow_valid() {
        out.interrupt.shadow = if shadowed { SHADOW_INTO_HARDWARE } else { 0 };
    }
}

/// A vCPU's interruptibility with the interpreter's shadow written over it.
///
/// # Errors
///
/// [`AccelError::Sys`](super::AccelError::Sys) if `KVM_GET_VCPU_EVENTS` fails.
fn events_to_kvm(shadowed: bool, vcpu: &Vcpu) -> AccelResult<KvmVcpuEvents> {
    let mut out = vcpu.events()?;
    overlay_events(shadowed, &mut out);
    Ok(out)
}

// ---------------------------------------------------------------------------
// the local APIC's task priority, which is what CR8 is
// ---------------------------------------------------------------------------

/// Byte offset of the task-priority register within a local APIC's page.
///
/// *Intel SDM* volume 3A §10.4.4, table 10-1.
pub const APIC_TPR: u64 = 0x80;

/// Write `cr8` into a local APIC's task-priority register, through the address
/// space the machine file mapped that APIC into.
///
/// **This is the route the module documentation says is missing**, made
/// explicit rather than implied. `CR8` is not a register the core owns: it is
/// the top four bits of the local APIC's TPR, reached by the processor over a
/// dedicated path (SDM volume 3A §10.8.6.1). rsemu models the APIC as a
/// device, so the only honest way to put a value into it is the way software
/// does — a store to its register page. A caller that knows where its board put
/// that page can therefore complete the transfer; this module cannot know, and
/// does not guess.
///
/// # Errors
///
/// Whatever the address space says: a board with no APIC at `apic_base`, or one
/// whose APIC is hardware-disabled, refuses the access rather than swallowing
/// it.
pub fn tpr_through_space(
    space: &crate::core::space::AddressSpace,
    apic_base: u64,
    cr8: u8,
) -> crate::core::space::MemResult {
    space.write(
        apic_base + APIC_TPR,
        crate::core::Width::U32,
        u64::from(cr8) << 4,
        crate::core::space::MemAttrs::DEFAULT,
    )
}

/// Read a local APIC's task-priority register back as a `CR8` value.
///
/// A **debug** access, so that sampling the priority cannot disturb the device
/// — `CLAUDE.md` requires every MMIO device to honour that, and a state
/// transfer is precisely a debugger-shaped read.
///
/// # Errors
///
/// Whatever the address space says.
pub fn tpr_from_space(
    space: &crate::core::space::AddressSpace,
    apic_base: u64,
) -> crate::core::space::MemResult<u8> {
    let tpr = space.read(
        apic_base + APIC_TPR,
        crate::core::Width::U32,
        crate::core::space::MemAttrs::DEBUG,
    )?;
    #[allow(clippy::cast_possible_truncation)]
    Ok(((tpr >> 4) & 0xf) as u8)
}

// ---------------------------------------------------------------------------
// whole-core transfer
// ---------------------------------------------------------------------------

/// Overlay the fields rsemu owns onto a `kvm_sregs` the vCPU already has.
///
/// **This, rather than a wholesale replacement, is what a load must do**, and
/// the reason is a bug this module was written with and then found: `cr8`,
/// `apic_base` and `interrupt_bitmap` are KVM's and rsemu's [`Sys`] has no
/// field for any of them, so building a fresh `kvm_sregs` sets `apic_base` to
/// zero — which moves the guest's local APIC to physical address zero and makes
/// the very next `KVM_RUN` fail to enter. Preserving what the accelerator
/// already knows is both correct and the honest expression of the gap.
pub fn overlay_sys(sys: &Sys, sregs: &mut KvmSregs) {
    let fresh = sys_to_sregs(sys);
    let tr = usable_task_register(fresh.tr, sregs.tr);
    *sregs = KvmSregs {
        cr8: sregs.cr8,
        apic_base: sregs.apic_base,
        interrupt_bitmap: sregs.interrupt_bitmap,
        tr,
        ..fresh
    };
}

/// Pick a task register a hypervisor will accept: `ours` unless it is
/// unusable, in which case whatever the vCPU already had.
///
/// **This is not a fudge, and the reason is worth writing down.** *Intel SDM*
/// Vol 3A Table 9-1 gives `TR` a selector of `0000`, a base of 0, a limit of
/// `FFFFh` and a **present** 16-bit busy-TSS type after `RESET`;
/// [`Sys::reset`](crate::cpu::x86::prot::Sys::reset) leaves it zeroed, which is
/// one place the core is less specific than the table. That costs the
/// interpreter nothing — it never consults `TR` until a `LTR` or a task gate
/// loads one — and it costs a hypervisor the entire entry: VMX's guest-state
/// checks (*SDM* Vol 3C §26.3.1.2) require the task register to be *usable*,
/// and a VM entry with an unusable one fails with "invalid guest state" before
/// a single guest instruction runs. That failure was `0x80000021` on a board
/// whose firmware never touches `TR`.
///
/// Keeping the destination's is the same argument
/// [`overlay_sys`] already makes for `apic_base`: the field is one the
/// accelerator knows more about than the state being loaded, so writing an
/// empty one over it is a loss rather than a transfer. Fabricating Table 9-1's
/// descriptor here instead would put a value in the *translation* that neither
/// engine holds, and [`differs`] would then be comparing an invention.
fn usable_task_register(ours: KvmSegment, theirs: KvmSegment) -> KvmSegment {
    if ours.unusable == 0 { ours } else { theirs }
}

/// Copy the interpreter's architectural state into a vCPU.
///
/// Everything the table at the top of this module marks **yes**: the register
/// file, the system state, the five carried MSRs, the debug registers and the
/// x87/SSE files. What it does not carry is named there too.
///
/// # Errors
///
/// [`AccelError::Sys`](super::AccelError::Sys) if any `ioctl` fails, and
/// [`AccelError::Unsupported`](super::AccelError::Unsupported) if this host
/// refuses one of the model-specific registers — which is a real difference
/// between the two engines and is reported rather than dropped.
pub fn load_into_vcpu(cpu: &X86, vcpu: &Vcpu) -> AccelResult<()> {
    let sys = cpu.sys();
    let mut sregs = vcpu.sregs()?;
    overlay_sys(&sys, &mut sregs);
    vcpu.set_sregs(&sregs)?;
    vcpu.set_regs(&regs_to_kvm(&cpu.regs()))?;
    vcpu.set_msrs(msrs_to_kvm(&sys))?;
    vcpu.set_debugregs(&dregs_to_kvm(&sys))?;
    vcpu.set_fpu(&fpu_to_kvm(&cpu.x87(), &cpu.sse()))?;
    // Last, and it is a read-modify-write rather than a write: see
    // [`overlay_events`]. Without it a guest resumed between a `STI` and the
    // instruction after it takes an interrupt one instruction early, on the
    // half-loaded `SS:SP` the shadow exists to protect.
    vcpu.set_events(&events_to_kvm(cpu.interrupt_shadow(), vcpu)?)?;
    Ok(())
}

/// [`load_into_vcpu`], and **also** hand the vCPU the time-stamp counter.
///
/// The one call that writes `IA32_TSC`, and it is separate from
/// [`load_into_vcpu`] on purpose. That function runs at the top of every
/// hardware slice whose shell is ahead — after a reset sequence, after an
/// instruction hardware could not fetch — and a `KVM_SET_MSRS` of the counter
/// there would rewind the guest's clock by however long this process spent in
/// userspace between the store and the load, on every slice. A guest that
/// timed anything would see its counter stutter, which is a worse lie than the
/// rate change a restore already admits to.
///
/// A restore is the moment when writing it is right: the counter is being
/// handed a value that came from a snapshot, and the alternative is the vCPU's
/// own — which, for a freshly created vCPU, is a counter that started when this
/// process did. See the module documentation for what is and is not continuous
/// across the switch.
///
/// # Errors
///
/// As [`load_into_vcpu`], plus [`AccelError::Unsupported`](super::AccelError::Unsupported)
/// if this host refuses `IA32_TSC`.
pub fn restore_into_vcpu(cpu: &X86, vcpu: &Vcpu) -> AccelResult<()> {
    load_into_vcpu(cpu, vcpu)?;
    vcpu.set_msrs([(msr::TSC, cpu.cycles())])
}

/// Copy a vCPU's architectural state into the interpreter.
///
/// What phase 7's gate calls *"a snapshot taken under KVM restores under the
/// interpreter"*, minus the parts the module documentation names as missing.
///
/// # Errors
///
/// [`AccelError::Sys`](super::AccelError::Sys) if either `ioctl` fails.
pub fn store_from_vcpu(vcpu: &Vcpu, cpu: &X86) -> AccelResult<()> {
    let regs = vcpu.regs()?;
    let sregs = vcpu.sregs()?;
    let msrs = vcpu.msrs(READ_MSRS)?;
    let dregs = vcpu.debugregs()?;
    let fpu = vcpu.fpu()?;
    let events = vcpu.events()?;
    // Registers first, system state second, and the order is load-bearing:
    // [`X86::set_regs`] recomputes the segment caches from the selectors when
    // the core is in real mode, which is right for a debugger writing `CS` and
    // wrong here — the accelerator has just told us what the *cached* bases
    // are, and a 386 out of reset has a `CS` base of `ffff0000` that
    // `selector << 4` cannot express. Letting `set_sys` land last keeps the
    // authoritative copy.
    cpu.set_regs(regs_from_kvm(&regs, &sregs));
    let mut sys = sregs_to_sys(&sregs, &cpu.sys());
    msrs_from_kvm(&carried_of(&msrs), &mut sys);
    dregs_from_kvm(&dregs, &mut sys);
    cpu.set_sys(sys);
    let (x87, sse) = fpu_from_kvm(&fpu, &cpu.x87());
    cpu.set_x87(x87);
    cpu.set_sse(sse);
    events_from_kvm(&events, cpu);
    // The counter, unconditionally and in this direction only. What the
    // hypervisor reports for `IA32_TSC` is what a `RDTSC` in the guest would
    // have returned, and `X86::cycles` is where this core's `RDTSC` reads
    // from, so the shell being *current after every slice* has to include it —
    // otherwise `save` writes a counter belonging to whatever the interpreter
    // last did rather than to the guest.
    cpu.set_cycles(msrs[TSC_AT]);
    Ok(())
}

/// Whether a vCPU's architectural state matches the interpreter's, for the
/// fields both engines carry.
///
/// The comparison a cross-engine test makes. It deliberately compares the
/// *translated* forms rather than the raw ones, so a field this module gets
/// wrong shows up as a mismatch rather than being compared against itself.
///
/// # Errors
///
/// [`AccelError::Sys`](super::AccelError::Sys) if the vCPU's registers cannot be read.
pub fn differs(vcpu: &Vcpu, cpu: &X86) -> AccelResult<Option<&'static str>> {
    let regs = vcpu.regs()?;
    let sregs = vcpu.sregs()?;
    let ours = regs_to_kvm(&cpu.regs());
    if regs != ours {
        return Ok(Some(match () {
            () if regs.rip != ours.rip => "rip",
            () if regs.rflags != ours.rflags => "rflags",
            () if regs.rsp != ours.rsp => "rsp",
            () => "a general-purpose register",
        }));
    }
    let our_sregs = sys_to_sregs(&cpu.sys());
    if sregs.cs != our_sregs.cs {
        return Ok(Some("cs"));
    }
    if sregs.ss != our_sregs.ss || sregs.ds != our_sregs.ds || sregs.es != our_sregs.es {
        return Ok(Some("a data segment"));
    }
    if sregs.cr0 != our_sregs.cr0 || sregs.cr3 != our_sregs.cr3 || sregs.cr4 != our_sregs.cr4 {
        return Ok(Some("a control register"));
    }
    if sregs.gdt != our_sregs.gdt || sregs.idt != our_sregs.idt {
        return Ok(Some("a descriptor table register"));
    }
    let sys = cpu.sys();
    let mut theirs = vcpu.msrs(CARRIED_MSRS)?;
    let mut ours = msrs_to_kvm(&sys).map(|(_, data)| data);
    // `IA32_MISC_ENABLE` is compared only over the bits a guest can write, for
    // the reason [`msrs_from_kvm`] merges rather than replaces: the rest of it
    // is each processor's statement about itself, the two processors are not
    // the same one, and reporting that as a disagreement would make this
    // function cry wolf on every call the way an unfiltered `MXCSR` would.
    theirs[MISC_AT] &= misc_enable::WRITABLE;
    ours[MISC_AT] &= misc_enable::WRITABLE;
    if theirs != ours {
        // Name the register rather than the group: with thirty-four of them a
        // bare "a model-specific register" sends the reader to a diff.
        let at = (0..MSR_COUNT)
            .find(|&i| theirs[i] != ours[i])
            .unwrap_or_default();
        return Ok(Some(match at {
            0 => "IA32_STAR",
            1 => "IA32_LSTAR",
            2 => "IA32_CSTAR",
            3 => "IA32_FMASK",
            4 => "IA32_KERNEL_GS_BASE",
            MISC_AT => "IA32_MISC_ENABLE",
            DEF_TYPE_AT => "IA32_MTRR_DEF_TYPE",
            i if i < VAR_AT => "a fixed memory-type range",
            _ => "a variable memory-type range",
        }));
    }
    if vcpu.debugregs()? != dregs_to_kvm(&sys) {
        return Ok(Some("a debug register"));
    }
    if comparable(vcpu.fpu()?) != comparable(fpu_to_kvm(&cpu.x87(), &cpu.sse())) {
        return Ok(Some("the floating-point state"));
    }
    // Only the shadow, because it is the only member of `kvm_vcpu_events` the
    // interpreter has anywhere to put. The others are compared against nothing
    // rather than against zero — see the module documentation's honest list.
    if vcpu.events()?.interrupt_shadow() != cpu.interrupt_shadow() {
        return Ok(Some("the interrupt shadow"));
    }
    Ok(None)
}

/// A `kvm_fpu` with the fields that cannot be compared removed.
///
/// `MXCSR` because `KVM_SET_FPU` does not write it (see the module
/// documentation), so a loaded state and the accelerator's will legitimately
/// disagree there and reporting it would make [`differs`] cry wolf on every
/// call. The padding because it is padding.
fn comparable(mut fpu: KvmFpu) -> KvmFpu {
    fpu.mxcsr = 0;
    fpu.pad1 = 0;
    fpu.pad2 = 0;
    fpu
}

/// The state this module knows how to carry, as a plain value.
///
/// Useful to a caller that wants to hold a core's state between engines
/// without a live vCPU or a live interpreter at the other end.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ArchState {
    /// The general-purpose file.
    pub regs: KvmRegs,
    /// Segments, descriptor tables and control registers.
    pub sregs: KvmSregs,
    /// The values of [`CARRIED_MSRS`], in that order.
    pub msrs: [u64; MSR_COUNT],
    /// `DR0`-`DR3`, `DR6`, `DR7`.
    pub dregs: KvmDebugregs,
    /// The x87 and SSE files.
    pub fpu: KvmFpu,
    /// The interruptibility state: the interrupt shadow, and — for a value
    /// that came out of a vCPU and is going back into one — the pending
    /// exception, the pending and blocked `NMI` and the system-management
    /// state as well.
    ///
    /// **Only the shadow survives a trip through the interpreter**, because
    /// only the shadow has a field there. A caller moving state from one vCPU
    /// to another through an `ArchState` carries all of it; a caller going via
    /// [`into_interpreter`](ArchState::into_interpreter) does not, and the
    /// module documentation says what that costs.
    pub events: KvmVcpuEvents,
    /// `IA32_TSC`: the value a `RDTSC` in the guest would return at the instant
    /// this state was taken.
    ///
    /// **Architectural, and applied in both directions.** Both engines have
    /// this register — a hypervisor reports it through `KVM_GET_MSRS`, and this
    /// core's `RDTSC` reads [`X86::cycles`](crate::cpu::x86::X86::cycles) — so
    /// the *value* crosses. What does not cross is the **rate**, which is a
    /// property of the machine rather than of the guest; the module
    /// documentation says what that costs.
    ///
    /// Never an *offset*. A hypervisor implements the counter as host silicon
    /// plus a per-vCPU offset, and that offset is derived from this value and
    /// the host's own counter at the moment of the write. `CLAUDE.md` says
    /// derived state is never serialized, and this is the case it was written
    /// for: an offset would make a snapshot meaningless on every host but the
    /// one that took it, while the value is meaningful on all of them.
    pub tsc: u64,
}

impl ArchState {
    /// Read it out of a vCPU.
    ///
    /// # Errors
    ///
    /// [`AccelError::Sys`](super::AccelError::Sys) if either `ioctl` fails.
    pub fn from_vcpu(vcpu: &Vcpu) -> AccelResult<ArchState> {
        let msrs = vcpu.msrs(READ_MSRS)?;
        Ok(ArchState {
            regs: vcpu.regs()?,
            sregs: vcpu.sregs()?,
            msrs: carried_of(&msrs),
            dregs: vcpu.debugregs()?,
            fpu: vcpu.fpu()?,
            events: vcpu.events()?,
            tsc: msrs[TSC_AT],
        })
    }

    /// Read it out of the interpreter.
    ///
    /// [`tsc`](ArchState::tsc) is [`X86::cycles`](crate::cpu::x86::X86::cycles),
    /// because that is where this core's `RDTSC` reads from. It used to come
    /// back `None`, on the argument that a retired-cycle count is not a
    /// time-stamp counter — which confused the *rate* with the *quantity*. The
    /// quantity is the same one in both engines: what `RDTSC` returns. Only the
    /// rate differs, and a rate belongs to the machine rather than to the guest.
    #[must_use]
    pub fn from_interpreter(cpu: &X86) -> ArchState {
        let sys = cpu.sys();
        ArchState {
            regs: regs_to_kvm(&cpu.regs()),
            sregs: sys_to_sregs(&sys),
            msrs: msrs_to_kvm(&sys).map(|(_, data)| data),
            dregs: dregs_to_kvm(&sys),
            fpu: fpu_to_kvm(&cpu.x87(), &cpu.sse()),
            // Synthesised rather than read, because the interpreter has one
            // boolean where the structure has four members. The valid flag is
            // set so that the shadow travels; every other member is zero and
            // [`into_vcpu`](ArchState::into_vcpu) leaves the destination's
            // alone rather than writing these.
            events: KvmVcpuEvents {
                flags: events::VCPUEVENT_VALID_SHADOW,
                interrupt: crate::accel::kvm::KvmEventInterrupt {
                    shadow: if cpu.interrupt_shadow() {
                        SHADOW_INTO_HARDWARE
                    } else {
                        0
                    },
                    ..Default::default()
                },
                ..Default::default()
            },
            tsc: cpu.cycles(),
        }
    }

    /// The value `CR8` had on the accelerator, for a caller that has a route
    /// into the board's local APIC — see [`tpr_through_space`].
    #[must_use]
    #[allow(clippy::cast_possible_truncation)]
    pub const fn cr8(&self) -> u8 {
        (self.sregs.cr8 & 0xf) as u8
    }

    /// `IA32_APIC_BASE` as the accelerator had it.
    ///
    /// The crate does have a seam for this —
    /// [`LocalController::set_base_register`](crate::core::wire::LocalController::set_base_register),
    /// which `X86` already forwards `WRMSR` through — so a caller holding that
    /// link can complete the transfer. This type carries the value because
    /// [`store_from_vcpu`] is handed a core and a vCPU and has no third party
    /// to hand it to.
    #[must_use]
    pub const fn apic_base(&self) -> u64 {
        self.sregs.apic_base
    }

    /// Write it into a vCPU.
    ///
    /// `cr8`, `apic_base` and the pending-interrupt bitmap are left as the vCPU
    /// has them; see [`overlay_sys`] for why replacing them wholesale breaks
    /// the next entry.
    ///
    /// # Errors
    ///
    /// [`AccelError::Sys`](super::AccelError::Sys) if any `ioctl` fails.
    pub fn into_vcpu(self, vcpu: &Vcpu) -> AccelResult<()> {
        let mut sregs = vcpu.sregs()?;
        let keep = (sregs.cr8, sregs.apic_base, sregs.interrupt_bitmap, sregs.tr);
        sregs = self.sregs;
        sregs.cr8 = keep.0;
        sregs.apic_base = keep.1;
        sregs.interrupt_bitmap = keep.2;
        // The same rule `overlay_sys` follows, and for the reason
        // `usable_task_register` gives.
        sregs.tr = usable_task_register(sregs.tr, keep.3);
        vcpu.set_sregs(&sregs)?;
        vcpu.set_regs(&self.regs)?;
        let values: [(u32, u64); MSR_COUNT] =
            core::array::from_fn(|i| (CARRIED_MSRS[i], self.msrs[i]));
        vcpu.set_msrs(values)?;
        vcpu.set_debugregs(&self.dregs)?;
        vcpu.set_fpu(&self.fpu)?;
        vcpu.set_events(&events_to_kvm(self.events.interrupt_shadow(), vcpu)?)?;
        // The counter last, and unlike [`load_into_vcpu`] this **does** write
        // it: an `ArchState` is a state held outside both engines, so putting
        // one into a vCPU is a restore by definition rather than the resumption
        // of a slice. [`restore_into_vcpu`] makes the same call for the same
        // reason.
        vcpu.set_msrs([(msr::TSC, self.tsc)])
    }

    /// Write it into the interpreter.
    ///
    /// Registers first, system state second; see [`store_from_vcpu`] for why
    /// that order matters.
    pub fn into_interpreter(self, cpu: &X86) {
        cpu.set_regs(regs_from_kvm(&self.regs, &self.sregs));
        let mut sys = sregs_to_sys(&self.sregs, &cpu.sys());
        msrs_from_kvm(&self.msrs, &mut sys);
        dregs_from_kvm(&self.dregs, &mut sys);
        cpu.set_sys(sys);
        let (x87, sse) = fpu_from_kvm(&self.fpu, &cpu.x87());
        cpu.set_x87(x87);
        cpu.set_sse(sse);
        events_from_kvm(&self.events, cpu);
        cpu.set_cycles(self.tsc);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_real_mode_reset_segment_survives_the_round_trip() {
        // The one that matters most: a 386 resets with a `CS` *selector* of
        // f000 and a cached *base* of ffff0000, which is not expressible as
        // `selector << 4` and is exactly what a naive translation loses.
        let sys = Sys::reset();
        let sregs = sys_to_sregs(&sys);
        assert_eq!(sregs.cs.selector, 0xf000);
        assert_eq!(sregs.cs.base, 0xffff_0000);
        assert_eq!(sregs.cs.limit, 0xffff);
        assert_eq!(sregs.cs.present, 1);
        assert_eq!(sregs.cs.unusable, 0);
        assert_eq!(sregs.cs.s, 1);

        let back = sregs_to_sys(&sregs, &Sys::reset());
        assert_eq!(back.segs[seg::CS as usize], sys.segs[seg::CS as usize]);
        assert_eq!(back.idtr, sys.idtr);
        assert_eq!(back.gdtr, sys.gdtr);
        assert_eq!(back.cr0, sys.cr0);
    }

    #[test]
    fn a_protected_mode_descriptor_round_trips_bit_for_bit() {
        let reg = SegReg {
            selector: 0x0008,
            base: 0x1234_5678_9abc_0000,
            limit: 0xffff_ffff,
            ar: ar::PRESENT
                | ar::S
                | ar::CODE
                | ar::RW
                | ar::ACCESSED
                | ar::DB
                | ar::GRANULAR
                | ar::AVL
                | (2 << ar::DPL_SHIFT),
        };
        let kvm = seg_to_kvm(&reg);
        assert_eq!(kvm.dpl, 2);
        assert_eq!(kvm.g, 1);
        assert_eq!(kvm.db, 1);
        assert_eq!(kvm.avl, 1);
        // Type 0xb: code, readable, accessed.
        assert_eq!(kvm.kind, 0b1011);
        assert_eq!(seg_from_kvm(&kvm), reg);
    }

    #[test]
    fn a_long_mode_code_segment_keeps_its_l_bit() {
        let reg = SegReg {
            selector: 0x0010,
            base: 0,
            limit: 0xffff_ffff,
            ar: ar::PRESENT | ar::S | ar::CODE | ar::RW | ar::ACCESSED | ar::L | ar::GRANULAR,
        };
        let kvm = seg_to_kvm(&reg);
        assert_eq!(kvm.l, 1);
        assert_eq!(kvm.db, 0);
        assert_eq!(seg_from_kvm(&kvm), reg);
    }

    #[test]
    fn a_null_selector_comes_back_unusable() {
        let reg = SegReg::null();
        let kvm = seg_to_kvm(&reg);
        assert_eq!(kvm.present, 0);
        assert_eq!(kvm.unusable, 1);
        assert_eq!(seg_from_kvm(&kvm).ar & ar::PRESENT, 0);
    }

    #[test]
    fn the_general_purpose_file_round_trips() {
        let mut regs = Regs::new();
        regs.rax = 0x0123_4567_89ab_cdef;
        regs.rbx = 2;
        regs.rcx = 3;
        regs.rdx = 4;
        regs.rsi = 5;
        regs.rdi = 6;
        regs.rsp = 7;
        regs.rbp = 8;
        regs.r = [9, 10, 11, 12, 13, 14, 15, 16];
        regs.rip = 0xfff0;
        regs.eflags = 0x0246;

        let kvm = regs_to_kvm(&regs);
        // The UAPI order is rax, rbx, rcx, rdx — not the ModRM one. A
        // transposition here is the classic way to make a guest run garbage,
        // so it is asserted rather than assumed.
        assert_eq!(kvm.rbx, 2);
        assert_eq!(kvm.rcx, 3);
        assert_eq!(kvm.rflags, 0x0246);
        // The selectors come from `kvm_sregs`, so build one that agrees with
        // `regs` and assert the whole struct rather than the numeric half.
        let mut sys = Sys::reset();
        for (i, sel) in [regs.es, regs.cs, regs.ss, regs.ds, regs.fs, regs.gs]
            .into_iter()
            .enumerate()
        {
            sys.segs[i].selector = sel;
        }
        assert_eq!(regs_from_kvm(&kvm, &sys_to_sregs(&sys)), regs);
    }

    #[test]
    fn a_restore_leaves_the_two_copies_of_a_selector_agreeing() {
        // rsemu keeps `CS` in `Regs::cs` *and* in `Sys::segs[CS].selector`, and
        // `X86::set_regs` recomputes the real-mode base from the first. A
        // restore that filled only the second left the core fetching from
        // wherever it had last been reset to — which is the bug this test was
        // written for.
        let mut sys = Sys::reset();
        sys.segs[seg::CS as usize] = SegReg::real_code(0x0000);
        sys.segs[seg::DS as usize] = SegReg::real_data(0x1234);
        let sregs = sys_to_sregs(&sys);
        let regs = regs_from_kvm(&KvmRegs::default(), &sregs);
        assert_eq!(regs.cs, 0x0000, "Regs::cs must follow kvm_sregs.cs");
        assert_eq!(regs.ds, 0x1234);
        assert_eq!(regs.cs, sregs.cs.selector);
        assert_eq!(regs.ds, sregs.ds.selector);
    }

    #[test]
    fn the_segment_registers_are_not_transposed() {
        // Six registers with six distinguishable selectors: a swapped pair
        // here would be invisible in any test that used a uniform state.
        let mut sys = Sys::reset();
        for (i, sel) in [0x10u16, 0x20, 0x30, 0x40, 0x50, 0x60]
            .into_iter()
            .enumerate()
        {
            sys.segs[i] = SegReg::real_data(sel);
        }
        let sregs = sys_to_sregs(&sys);
        assert_eq!(sregs.es.selector, 0x10);
        assert_eq!(sregs.cs.selector, 0x20);
        assert_eq!(sregs.ss.selector, 0x30);
        assert_eq!(sregs.ds.selector, 0x40);
        assert_eq!(sregs.fs.selector, 0x50);
        assert_eq!(sregs.gs.selector, 0x60);
        let back = sregs_to_sys(&sregs, &Sys::reset());
        assert_eq!(back.segs, sys.segs);
    }

    #[test]
    fn the_fs_and_gs_bases_travel_with_their_segments() {
        let mut sys = Sys::reset();
        sys.segs[seg::FS as usize].base = 0x7fff_0000_0000;
        sys.segs[seg::GS as usize].base = 0x7fff_0000_1000;
        let back = sregs_to_sys(&sys_to_sregs(&sys), &Sys::reset());
        assert_eq!(back.fs_base, 0x7fff_0000_0000);
        assert_eq!(back.gs_base, 0x7fff_0000_1000);
    }

    #[test]
    fn the_carried_msrs_round_trip() {
        // The four `SYSCALL` registers plus `KERNEL_GS_BASE`: the set a 64-bit
        // guest loses across an engine switch if this does not work, with the
        // symptom being a kernel entry that jumps to zero. And, since the set
        // widened, every memory-type range too.
        let mut sys = Sys::reset();
        sys.star = 0x0023_0010_0000_0000;
        sys.lstar = 0xffff_8000_0010_0000;
        sys.cstar = 0xffff_8000_0010_1000;
        sys.sfmask = 0x0000_0000_0004_7700;
        sys.kernel_gs_base = 0xffff_8880_0000_0000;
        sys.mtrr_def_type = mtrr::DEF_E | mtrr::DEF_FE | 6;
        // Every register distinguishable from every other, so a transposition
        // in the address arithmetic shows up as a wrong value rather than
        // hiding behind a uniform state.
        for (i, slot) in sys.mtrr_fix.iter_mut().enumerate() {
            *slot = 0x0f00_0000_0000_0000 | i as u64;
        }
        for (i, slot) in sys.mtrr_var.iter_mut().enumerate() {
            *slot = 0x0e00_0000_0000_0000 | i as u64;
        }

        let pairs = msrs_to_kvm(&sys);
        assert_eq!(
            pairs.map(|(index, _)| index),
            CARRIED_MSRS,
            "the order the two functions agree on is the array's"
        );
        let mut back = Sys::reset();
        msrs_from_kvm(&pairs.map(|(_, data)| data), &mut back);
        assert_eq!(back.star, sys.star);
        assert_eq!(back.lstar, sys.lstar);
        assert_eq!(back.cstar, sys.cstar);
        assert_eq!(back.sfmask, sys.sfmask);
        assert_eq!(back.kernel_gs_base, sys.kernel_gs_base);
        assert_eq!(back.mtrr_def_type, sys.mtrr_def_type);
        assert_eq!(back.mtrr_fix, sys.mtrr_fix);
        assert_eq!(back.mtrr_var, sys.mtrr_var);
    }

    #[test]
    fn the_carried_msr_addresses_are_the_ones_the_sdm_names() {
        // The table is generated rather than typed, so what a test can add is
        // that the generator agrees with *Intel SDM* volume 4 Table 2-2. A
        // wrong address here is a `#GP` from the kernel at best and a silently
        // transferred neighbour at worst.
        assert_eq!(MSR_COUNT, 34, "5 syscall + 2 + 11 fixed + 16 variable");
        assert_eq!(
            &CARRIED_MSRS[..7],
            &[
                0xc000_0081,
                0xc000_0082,
                0xc000_0083,
                0xc000_0084,
                0xc000_0102,
                0x1a0,
                0x2ff
            ]
        );
        // The eleven fixed ranges: one 64 KiB, two 16 KiB, eight 4 KiB.
        assert_eq!(
            &CARRIED_MSRS[FIXED_AT..VAR_AT],
            &[
                0x250, 0x258, 0x259, 0x268, 0x269, 0x26a, 0x26b, 0x26c, 0x26d, 0x26e, 0x26f
            ]
        );
        // The eight variable ranges as base/mask pairs at consecutive
        // addresses, which is why `Sys::mtrr_var` is one flat array.
        assert_eq!(CARRIED_MSRS[VAR_AT], 0x200);
        assert_eq!(CARRIED_MSRS[MSR_COUNT - 1], 0x20f);
        // And the read list is the write list plus the counter, in that order,
        // so `carried_of` can take a prefix.
        assert_eq!(&READ_MSRS[..MSR_COUNT], &CARRIED_MSRS);
        assert_eq!(READ_MSRS[TSC_AT], msr::TSC);
    }

    #[test]
    fn misc_enable_carries_what_a_guest_wrote_and_not_what_the_part_is() {
        // The asymmetry `msrs_from_kvm` argues for. A host kernel reports
        // `0x1800` out of reset — bits 11 and 12, *no branch-trace store, no
        // precise event-based sampling* — and this core reports `0x1`. Those
        // describe two different processors, so copying the host's answer in
        // would change what the emulated part claims about itself.
        let mut sys = Sys::reset();
        assert_eq!(sys.misc_enable, misc_enable::RESET);

        // What a hypervisor might hand back: the host's capability bits, plus
        // the one bit a guest really did write (Linux clears the execute-disable
        // lock before it looks for `NX`, and firmware sets it).
        let mut from_hardware = [0u64; MSR_COUNT];
        from_hardware[MISC_AT] = 0x1800 | misc_enable::XD_DISABLE;
        msrs_from_kvm(&from_hardware, &mut sys);
        assert_eq!(
            sys.misc_enable & misc_enable::XD_DISABLE,
            misc_enable::XD_DISABLE,
            "the guest's write crossed"
        );
        assert_eq!(
            sys.misc_enable & 0x1800,
            0,
            "and the host's statement about its own silicon did not"
        );
        assert_eq!(
            sys.misc_enable & misc_enable::FAST_STRINGS,
            0,
            "a writable bit the hypervisor reported clear is cleared here too"
        );

        // The other direction is *not* masked: the accelerator is being told
        // what part it is impersonating.
        let mut ours = Sys::reset();
        ours.misc_enable = misc_enable::RESET | misc_enable::XD_DISABLE;
        assert_eq!(msrs_to_kvm(&ours)[MISC_AT].1, ours.misc_enable);
    }

    #[test]
    fn an_arch_state_carries_the_counter_as_a_value() {
        // `CLAUDE.md`: derived state is never serialized. A hypervisor's TSC
        // *offset* is derived from the value and the host's own counter, so it
        // is meaningless anywhere else; the value is meaningful everywhere.
        // This asserts the type stores the second one — a `u64`, unconditional,
        // rather than an `Option` that says "the other engine cannot use this".
        let cpu = X86::new(crate::cpu::x86::Config::X86_64);
        cpu.set_cycles(0x0000_4200_0000_0000);
        let state = ArchState::from_interpreter(&cpu);
        assert_eq!(state.tsc, 0x0000_4200_0000_0000);

        let far = X86::new(crate::cpu::x86::Config::X86_64);
        state.into_interpreter(&far);
        assert_eq!(
            far.cycles(),
            0x0000_4200_0000_0000,
            "and it lands where this core's RDTSC reads from"
        );
    }

    #[test]
    fn the_debug_registers_round_trip_and_dr4_dr5_are_not_invented() {
        let mut sys = Sys::reset();
        sys.dr = [
            0x1000,
            0x2000,
            0x3000,
            0x4000,
            0,
            0,
            0xffff_0ff0,
            0x0000_0405,
        ];
        let kvm = dregs_to_kvm(&sys);
        assert_eq!(kvm.db, [0x1000, 0x2000, 0x3000, 0x4000]);
        assert_eq!(kvm.dr6, 0xffff_0ff0);
        assert_eq!(kvm.dr7, 0x0000_0405);
        assert_eq!(kvm.flags, 0);

        let mut back = Sys::reset();
        back.dr[4] = 0xdead;
        back.dr[5] = 0xbeef;
        dregs_from_kvm(&kvm, &mut back);
        assert_eq!(back.dr[0..4], sys.dr[0..4]);
        assert_eq!(back.dr[6], sys.dr[6]);
        assert_eq!(back.dr[7], sys.dr[7]);
        // `DR4`/`DR5` are aliases and have no field in `kvm_debugregs`, so a
        // restore must leave whatever was there rather than zeroing it.
        assert_eq!(back.dr[4], 0xdead);
        assert_eq!(back.dr[5], 0xbeef);
    }

    #[test]
    fn the_x87_and_sse_files_round_trip() {
        let mut x87 = X87::new();
        // 2.0 in the extended format: biased exponent 0x4000, integer bit set.
        x87.regs[0] = F80::new(0x4000, 1 << 63);
        x87.regs[1] = F80::ZERO;
        // Registers 0 and 1 hold values; the other six are empty (0b11).
        x87.tag = Tag::Valid.bits() | (Tag::Zero.bits() << 2) | 0xfff0;
        x87.control = 0x027f;
        x87.status = 0x3800;
        x87.last_ip = 0x0000_1234_5678_9abc;
        x87.last_dp = 0x0000_0000_dead_beef;
        x87.last_op = 0x1d9;
        x87.last_cs = 0x0008;
        x87.last_ds = 0x0010;

        let mut sse = Sse::new();
        for (i, reg) in sse.xmm.iter_mut().enumerate() {
            *reg = [
                0x0101_0101_0101_0100 + i as u64,
                0x0202_0202_0202_0200 + i as u64,
            ];
        }
        sse.mxcsr = 0x1f80;

        let kvm = fpu_to_kvm(&x87, &sse);
        assert_eq!(kvm.ftwx & 1, 1, "register 0 holds a value");
        assert_eq!(kvm.ftwx & 2, 2, "and so does register 1, which holds zero");
        assert_eq!(kvm.ftwx & 0xfc, 0, "the other six are empty");

        let (back87, backsse) = fpu_from_kvm(&kvm, &x87);
        assert_eq!(back87.regs, x87.regs);
        assert_eq!(
            back87.tag, x87.tag,
            "the abridged word rebuilds the full one"
        );
        assert_eq!(back87.control, x87.control);
        assert_eq!(back87.status, x87.status);
        assert_eq!(back87.last_ip, x87.last_ip);
        assert_eq!(back87.last_dp, x87.last_dp);
        assert_eq!(back87.last_op, x87.last_op);
        assert_eq!(backsse, sse);
    }

    #[test]
    fn a_tag_that_disagrees_with_its_register_is_corrected_the_way_fxrstor_does() {
        // The one place the translation is deliberately lossy: `FXSAVE` keeps
        // one bit per register, so *valid* against *zero* against *special* is
        // recomputed from the encoding rather than carried. A tag word that
        // lied about its register comes back telling the truth — which is what
        // the hardware does, and worth a test because it looks like a bug.
        let mut x87 = X87::new();
        x87.regs[0] = F80::ZERO;
        x87.tag = Tag::Valid.bits(); // claims a normal number, holds zero
        let (back, _) = fpu_from_kvm(&fpu_to_kvm(&x87, &Sse::new()), &x87);
        assert_eq!(Tag::from_bits(back.tag), Tag::Zero);
    }

    #[test]
    fn an_empty_register_keeps_its_bits() {
        // An empty x87 register still holds an encoding, and `FXSAVE` writes
        // it. Zeroing it on the way back would be visible to a guest that
        // pops a register and then reloads the file.
        let mut x87 = X87::new();
        x87.regs[3] = F80::new(0x4000, 1 << 63);
        x87.tag = 0xffff; // every register empty
        let (back, _) = fpu_from_kvm(&fpu_to_kvm(&x87, &Sse::new()), &x87);
        assert_eq!(back.tag, 0xffff);
        assert_eq!(back.regs[3], x87.regs[3]);
    }

    #[test]
    fn cr8_reaches_a_local_apic_the_only_way_a_device_can_be_reached() {
        // The structural gap, closed as far as it can be: `CR8` is the local
        // APIC's task-priority register, the APIC is a device, and a device is
        // reached through the address space its registers are mapped into.
        use crate::core::space::{
            AccessConstraints, AddressSpace, MemAttrs, MemOps, MemResult, Region,
        };
        use crate::core::sync::Mutex;
        use alloc::sync::Arc;

        #[derive(Debug, Default)]
        struct FakeApic {
            tpr: Mutex<u32>,
        }
        impl MemOps for FakeApic {
            fn read(&self, offset: u64, dst: &mut [u8], _attrs: MemAttrs) -> MemResult {
                let value = if offset == APIC_TPR {
                    *self.tpr.lock()
                } else {
                    0
                };
                for (i, b) in dst.iter_mut().enumerate() {
                    *b = (value >> (8 * i)) as u8;
                }
                Ok(())
            }
            fn write(&self, offset: u64, src: &[u8], _attrs: MemAttrs) -> MemResult {
                if offset == APIC_TPR {
                    let mut value = 0u32;
                    for (i, b) in src.iter().enumerate() {
                        value |= u32::from(*b) << (8 * i);
                    }
                    *self.tpr.lock() = value;
                }
                Ok(())
            }
            fn constraints(&self) -> AccessConstraints {
                AccessConstraints::ANY
            }
        }

        let apic = Arc::new(FakeApic::default());
        let space = AddressSpace::new("mem", 32);
        space
            .topology()
            .map(
                Arc::new(Region::io(
                    "lapic",
                    0x1000,
                    Arc::clone(&apic) as Arc<dyn MemOps>,
                )),
                0xfee0_0000,
            )
            .expect("map the APIC page");

        tpr_through_space(&space, 0xfee0_0000, 0x0b).expect("write TPR");
        assert_eq!(*apic.tpr.lock(), 0xb0, "CR8 is the top nibble of the TPR");
        assert_eq!(tpr_from_space(&space, 0xfee0_0000).expect("read TPR"), 0x0b);
    }

    #[test]
    fn an_overlay_of_the_events_keeps_what_the_kernel_owns() {
        // The shape of the bug this guards: a wholesale write would replace
        // the pending exception and the blocked `NMI` with zeros, because the
        // interpreter has no field for either. What the flags field says is
        // *take these members from me*, so the three the kernel keeps live are
        // expressed by clearing their bits rather than by copying values.
        let mut ev = KvmVcpuEvents {
            flags: events::VCPUEVENT_VALID_SHADOW
                | events::VCPUEVENT_VALID_NMI_PENDING
                | events::VCPUEVENT_VALID_SMM
                | events::VCPUEVENT_VALID_PAYLOAD,
            ..Default::default()
        };
        ev.nmi.pending = 1;
        ev.nmi.masked = 1;
        ev.exception.injected = 1;
        ev.exception.nr = 14;
        ev.smi.smm = 1;

        overlay_events(true, &mut ev);
        assert_eq!(ev.interrupt.shadow, SHADOW_INTO_HARDWARE);
        assert!(ev.shadow_valid(), "the shadow is what this transfer is for");
        assert_eq!(
            ev.flags & events::VCPUEVENT_VALID_NMI_PENDING,
            0,
            "an emulated core has no pending-NMI field, so the kernel keeps its own"
        );
        assert_eq!(ev.flags & events::VCPUEVENT_VALID_SMM, 0);
        assert_ne!(
            ev.flags & events::VCPUEVENT_VALID_PAYLOAD,
            0,
            "the payload flag came from the kernel and goes back to it"
        );
        assert_eq!(ev.nmi.masked, 1, "and the values themselves are untouched");
        assert_eq!(ev.exception.nr, 14);

        overlay_events(false, &mut ev);
        assert_eq!(ev.interrupt.shadow, 0);
    }

    #[test]
    fn a_host_that_does_not_report_the_shadow_is_not_told_one() {
        // No `KVM_CAP_INTR_SHADOW` means the kernel leaves the valid bit clear
        // on the way out, and writing the field back would be writing a byte
        // it has said it does not read.
        let mut ev = KvmVcpuEvents::default();
        overlay_events(true, &mut ev);
        assert_eq!(ev.interrupt.shadow, 0);
        assert!(!ev.interrupt_shadow());
    }

    #[test]
    fn the_msrs_kvm_does_not_report_are_kept_from_the_base() {
        // The documented gap: `LSTAR` and friends are MSRs and are not in
        // `kvm_sregs`, so a restore must not zero them.
        let mut base = Sys::reset();
        base.lstar = 0xffff_8000_0010_0000;
        base.dr[7] = 0x400;
        let restored = sregs_to_sys(&sys_to_sregs(&Sys::reset()), &base);
        assert_eq!(restored.lstar, 0xffff_8000_0010_0000);
        assert_eq!(restored.dr[7], 0x400);
    }
}
