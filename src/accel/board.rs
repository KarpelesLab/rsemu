//! Giving a hypervisor a **board's** memory map, rather than a hand-built one.
//!
//! `ROADMAP.md` phase 7's gate opens *"the phase-6 machines boot under KVM"*,
//! and the distance between that and the rest of [`accel`](crate::accel) is
//! exactly this file. A test rig can allocate two pages and call
//! [`Vm::set_memory_region`] itself; a *board* declares `object ram_low "ram" {
//! size = 640K }` and `map mem 0 size 640K = ram_low`, and something has to
//! turn the second into the first without a human transcribing it.
//!
//! # What it walks, and why that is the right thing to walk
//!
//! The input is an [`AddressSpace`]'s **flat view** — `core::space`'s own
//! derived, sorted, non-overlapping resolution of the region tree (§4.1). Not
//! the machine file, not the list of devices: the flat view is what the
//! interpreter dispatches through, so installing memory slots from it is the
//! strongest available statement that *the two engines see one machine*. A
//! shadowed ROM, an aliased aperture, a mapping a PCI BAR moved on top of —
//! all of them are already resolved here, by the same code the interpreter
//! trusts.
//!
//! Each entry becomes one of three things:
//!
//! | flat entry | slot | what a guest access does |
//! | --- | --- | --- |
//! | [`FlatTarget::Ram`] | read/write | hardware, no exit |
//! | [`FlatTarget::Rom`] | read-only (`KVM_MEM_READONLY`) | reads and **fetches** in hardware; a write exits and meets [`RomWrite`](crate::core::space::RomWrite) |
//! | [`FlatTarget::Io`] | none | every access exits and is served by the device model |
//!
//! The ROM row is not a refinement. KVM's instruction emulator declines to
//! *fetch* through an MMIO exit, so firmware that is not a memory slot is a
//! board that cannot execute its own reset vector — which is why [`RomStore`](crate::core::space::RomStore)
//! carries a page-aligned allocation too, and why this module refuses to
//! silently install a ROM as writable memory when the host lacks
//! `KVM_CAP_READONLY_MEM`.
//!
//! # What it refuses, out loud
//!
//! A hypervisor's memory slots are page-granular and a board's regions are
//! not. Everything that cannot be a slot is **reported rather than dropped**,
//! as a [`Skipped`] with a reason, because the failure mode of guessing here
//! is a guest that reads plausible rubbish out of a page the board meant to
//! decode somewhere else. In particular a repeating window (`mirror`) is
//! refused: the guest would see the first copy at every address instead of the
//! wrap the board asked for.
//!
//! An entry that is skipped is not lost — it simply stays MMIO, so the board's
//! own dispatch answers it. The cost is speed, never correctness.
//!
//! # Determinism
//!
//! Nothing here changes it: installing a slot does not make a run reproducible
//! and this module never says it does. The refusal that matters lives one
//! level down, in [`Vcpu::into_runnable`](super::kvm::Vcpu::into_runnable).
//!
//! [`Vm::set_memory_region`]: super::kvm::Vm::set_memory_region

use alloc::string::String;
use alloc::sync::Arc;
use alloc::vec::Vec;

use crate::core::space::{AddressSpace, EntryKind, FlatTarget, HOST_PAGE};

use super::AccelResult;
use super::kvm::{Backing, Vm};

/// One flat-view entry that could not become a memory slot, and why.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Skipped {
    /// Guest-physical address of the entry.
    pub start: u64,
    /// Its length in bytes.
    pub len: u64,
    /// Why it stayed MMIO. A sentence, because this is read by a person
    /// wondering why their board is slow or their firmware is not running.
    pub why: &'static str,
}

/// What [`install_space`] did.
#[derive(Debug, Clone, Default, PartialEq, Eq)]
pub struct Plan {
    /// Installed slots, as `(index, guest address, length, writable)`.
    pub slots: Vec<(u32, u64, u64, bool)>,
    /// Entries that stayed MMIO.
    pub skipped: Vec<Skipped>,
}

impl Plan {
    /// How many bytes of guest physical memory run in hardware.
    #[must_use]
    pub fn mapped_bytes(&self) -> u64 {
        self.slots.iter().map(|s| s.2).sum()
    }

    /// Whether any slot covers `addr`.
    #[must_use]
    pub fn covers(&self, addr: u64) -> bool {
        self.slots
            .iter()
            .any(|(_, base, len, _)| addr >= *base && addr - *base < *len)
    }

    /// A human-readable summary, for a monitor or a test that wants to say
    /// what happened rather than assert a shape.
    #[must_use]
    pub fn describe(&self) -> String {
        use core::fmt::Write as _;
        let mut out = String::new();
        for (index, base, len, writable) in &self.slots {
            let kind = if *writable { "ram" } else { "rom" };
            let _ = writeln!(out, "slot {index}: {base:#x}..{:#x} {kind}", base + len);
        }
        for s in &self.skipped {
            let _ = writeln!(
                out,
                "mmio: {:#x}..{:#x} ({})",
                s.start,
                s.start + s.len,
                s.why
            );
        }
        out
    }
}

/// The windows a space would install, decided without a hypervisor present.
///
/// Separate from [`install_space`] because the *decision* — which of a board's
/// regions can be hardware-backed — is the part with judgement in it, and a
/// judgement that can only be tested on a host with `/dev/kvm` is a judgement
/// that will not be tested.
#[derive(Debug, Clone, Default)]
pub struct Windows {
    /// Each guest-physical base and the window to install there.
    pub slots: Vec<(u64, Backing)>,
    /// Entries that stay MMIO.
    pub skipped: Vec<Skipped>,
}

/// Decide which of `space`'s flat entries can be memory slots.
///
/// `readonly_mem` is whether the host can mark a slot read-only; without it a
/// ROM is refused rather than installed as writable memory, because a board
/// whose firmware the guest can overwrite is a different board.
#[must_use]
pub fn plan_space(space: &AddressSpace, readonly_mem: bool) -> Windows {
    let view = space.view();
    let mut out = Windows::default();

    for entry in view.flat_view().entries() {
        let start = entry.start();
        let len = entry.len();
        let mut skip = |why: &'static str| {
            out.skipped.push(Skipped { start, len, why });
        };

        // Two responders, or a split read/write path: a slot can express
        // neither, and pretending otherwise would silently pick one.
        if entry.write_to().is_some() {
            skip("reads and writes go to different regions");
            continue;
        }
        let EntryKind::Single(leaf) = entry.kind() else {
            skip("more than one region answers here");
            continue;
        };

        let backing = match leaf.target() {
            FlatTarget::Ram(store) => Backing::ram(store),
            FlatTarget::Rom { store, .. } => Backing::rom(store),
            // Left as MMIO on purpose — this is a device, and the whole design
            // is that its accesses come back out to the model.
            FlatTarget::Io(_) => continue,
        };

        if leaf.period().is_some() {
            skip("a repeating window: hardware would see the first copy everywhere");
            continue;
        }
        if !backing.is_writable() && !readonly_mem {
            skip("this kernel cannot mark a slot read-only");
            continue;
        }
        if !start.is_multiple_of(HOST_PAGE) || !len.is_multiple_of(HOST_PAGE) {
            skip("not a whole number of host pages");
            continue;
        }
        let offset = leaf.offset();
        if !offset.is_multiple_of(HOST_PAGE) {
            skip("the window starts part-way into a host page of its store");
            continue;
        }
        match backing.window(offset, len) {
            Ok(window) => out.slots.push((start, window)),
            Err(_) => skip("the window leaves its backing store"),
        }
    }

    out
}

/// Install every part of `space` that can be a memory slot, starting at slot
/// index `first_slot`.
///
/// The stores are kept alive by the VM for as long as their slots exist, so a
/// caller may drop its own references. Idempotent in the useful sense: calling
/// it again after a retopology reinstalls from the current flat view, and a
/// slot whose geometry changed is deleted and recreated by
/// [`Vm::set_region`](super::kvm::Vm::set_region).
///
/// # Errors
///
/// [`AccelError::Sys`](super::AccelError::Sys) if the kernel refuses a region
/// [`plan_space`] thought was installable — a slot index above
/// `KVM_CAP_NR_MEMSLOTS`, most likely. A region that is merely *not* a slot is
/// not an error; it comes back in [`Plan::skipped`].
pub fn install_space(vm: &Vm, space: &AddressSpace, first_slot: u32) -> AccelResult<Plan> {
    let windows = plan_space(space, vm.has_readonly_mem());
    let mut plan = Plan {
        slots: Vec::new(),
        skipped: windows.skipped,
    };
    for (i, (start, window)) in windows.slots.into_iter().enumerate() {
        let index = first_slot + i as u32;
        let (len, writable) = (window.len(), window.is_writable());
        vm.set_region(index, start, window)?;
        plan.slots.push((index, start, len, writable));
    }
    Ok(plan)
}

/// The same, for a machine: install its memory space and report the plan.
///
/// A convenience with one piece of judgement in it — *which* space is the one
/// a vCPU executes out of. A board may declare several, and the CPU's is the
/// one its `space =` property named, so the caller supplies the name rather
/// than this guessing.
///
/// # Errors
///
/// [`AccelError::Unsupported`](super::AccelError::Unsupported) if the machine
/// has no space of that name, and whatever [`install_space`] returns
/// otherwise.
pub fn install_machine(
    vm: &Vm,
    machine: &crate::machine::Machine,
    space: &str,
    first_slot: u32,
) -> AccelResult<(Plan, Arc<AddressSpace>)> {
    let space = machine.space(space).ok_or(super::AccelError::Unsupported(
        "this machine has no address space of that name",
    ))?;
    let plan = install_space(vm, space, first_slot)?;
    Ok((plan, Arc::clone(space)))
}

#[cfg(test)]
mod tests;
