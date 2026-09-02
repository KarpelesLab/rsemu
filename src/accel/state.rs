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
//!
//! # What does not, and why — the honest list
//!
//! * **`CR8` and `apic_base`.** KVM has them; rsemu's [`Sys`] has neither,
//!   because the local APIC is a *device* in this crate (`dev::pc::apic`) and
//!   not part of the core's register file. Restoring an accelerated snapshot
//!   into the interpreter therefore has to hand these to that device, and there
//!   is no route from a core's state chunk to a sibling device's. Named in
//!   phase 7's deliverable list as *"LAPIC/x2APIC state"* and genuinely not
//!   done.
//! * **The rest of the MSRs.** `KVM_GET_MSRS`/`KVM_SET_MSRS` are not
//!   implemented here. rsemu keeps `STAR`, `LSTAR`, `CSTAR`, `SFMASK`,
//!   `FS_BASE`, `GS_BASE` and `KERNEL_GS_BASE` in [`Sys`], and KVM keeps the
//!   first four in MSRs rather than in `kvm_sregs`, so a 64-bit guest that has
//!   armed `SYSCALL` **loses those four** across an engine switch. The two
//!   `FS`/`GS` bases do survive, because KVM reports them as the `fs` and `gs`
//!   segment *bases*, which is where the hardware keeps them.
//! * **The x87 and SSE files.** `KVM_GET_FPU`/`KVM_GET_XSAVE` are not
//!   implemented; rsemu has [`fpu::X87`](crate::cpu::x86::fpu::X87) and
//!   [`fpu::Sse`](crate::cpu::x86::fpu::Sse) waiting for them. Phase 7 names
//!   *"the XSAVE area"* for the same reason.
//! * **The TSC offset**, and with it any guest that reads `RDTSC` and expects
//!   monotonicity across the switch.
//! * **`DR0`–`DR7`**, which rsemu has and `kvm_sregs` does not carry.
//!
//! Each of those is an `ioctl` and a transcribed structure away, and each is a
//! *deliverable* rather than something that falls out of this one. What is
//! here is the part the phase-7 gate's first sentence needs and the part a
//! 16- or 32-bit guest needs entirely.

use crate::cpu::x86::isa::seg;
use crate::cpu::x86::prot::{SegReg, Sys, TableReg, ar};
use crate::cpu::x86::{Regs, X86};

use super::AccelResult;
use super::kvm::{KvmDtable, KvmRegs, KvmSegment, KvmSregs, Vcpu};

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
    *sregs = KvmSregs {
        cr8: sregs.cr8,
        apic_base: sregs.apic_base,
        interrupt_bitmap: sregs.interrupt_bitmap,
        ..fresh
    };
}

/// Copy the interpreter's architectural state into a vCPU.
///
/// # Errors
///
/// [`AccelError::Sys`](super::AccelError::Sys) if any `ioctl` fails.
pub fn load_into_vcpu(cpu: &X86, vcpu: &Vcpu) -> AccelResult<()> {
    let mut sregs = vcpu.sregs()?;
    overlay_sys(&cpu.sys(), &mut sregs);
    vcpu.set_sregs(&sregs)?;
    vcpu.set_regs(&regs_to_kvm(&cpu.regs()))?;
    Ok(())
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
    // Registers first, system state second, and the order is load-bearing:
    // [`X86::set_regs`] recomputes the segment caches from the selectors when
    // the core is in real mode, which is right for a debugger writing `CS` and
    // wrong here — the accelerator has just told us what the *cached* bases
    // are, and a 386 out of reset has a `CS` base of `ffff0000` that
    // `selector << 4` cannot express. Letting `set_sys` land last keeps the
    // authoritative copy.
    cpu.set_regs(regs_from_kvm(&regs, &sregs));
    cpu.set_sys(sregs_to_sys(&sregs, &cpu.sys()));
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
    Ok(None)
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
}

impl ArchState {
    /// Read it out of a vCPU.
    ///
    /// # Errors
    ///
    /// [`AccelError::Sys`](super::AccelError::Sys) if either `ioctl` fails.
    pub fn from_vcpu(vcpu: &Vcpu) -> AccelResult<ArchState> {
        Ok(ArchState {
            regs: vcpu.regs()?,
            sregs: vcpu.sregs()?,
        })
    }

    /// Read it out of the interpreter.
    #[must_use]
    pub fn from_interpreter(cpu: &X86) -> ArchState {
        ArchState {
            regs: regs_to_kvm(&cpu.regs()),
            sregs: sys_to_sregs(&cpu.sys()),
        }
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
        let keep = (sregs.cr8, sregs.apic_base, sregs.interrupt_bitmap);
        sregs = self.sregs;
        sregs.cr8 = keep.0;
        sregs.apic_base = keep.1;
        sregs.interrupt_bitmap = keep.2;
        vcpu.set_sregs(&sregs)?;
        vcpu.set_regs(&self.regs)
    }

    /// Write it into the interpreter.
    ///
    /// Registers first, system state second; see [`store_from_vcpu`] for why
    /// that order matters.
    pub fn into_interpreter(self, cpu: &X86) {
        cpu.set_regs(regs_from_kvm(&self.regs, &self.sregs));
        cpu.set_sys(sregs_to_sys(&self.sregs, &cpu.sys()));
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
