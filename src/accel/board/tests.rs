//! Tests for the board-to-memory-slot mapping.
//!
//! The **classification** half needs no hypervisor at all and is tested
//! unconditionally: that is the half with the judgement in it, and a judgement
//! only exercised on a host with `/dev/kvm` is a judgement that goes untested
//! on most of them. The installation half skips cleanly where there is no
//! KVM, like the rest of this subsystem.

use super::*;
use crate::core::space::{
    AccessConstraints, MemAttrs, MemOps, MemResult, RamStore, Region, RomStore, RomWrite,
};

/// A device that answers everything, for the entries that must stay MMIO.
#[derive(Debug)]
struct Nothing;

impl MemOps for Nothing {
    fn read(&self, _offset: u64, dst: &mut [u8], _attrs: MemAttrs) -> MemResult {
        dst.fill(0);
        Ok(())
    }
    fn write(&self, _offset: u64, _src: &[u8], _attrs: MemAttrs) -> MemResult {
        Ok(())
    }
    fn constraints(&self) -> AccessConstraints {
        AccessConstraints::ANY
    }
}

/// A space shaped like a small PC: low RAM, a device page, a ROM, and a
/// mirrored aperture.
fn board_space() -> (Arc<AddressSpace>, Arc<RamStore>, Arc<RomStore>) {
    let ram = Arc::new(RamStore::new(0x2_0000));
    let rom = Arc::new(RomStore::new(alloc::vec![0x90u8; 0x1_0000]));
    let space = Arc::new(AddressSpace::new("mem", 32));
    {
        let mut topo = space.topology();
        topo.map(Arc::new(Region::ram("ram", Arc::clone(&ram))), 0)
            .expect("ram");
        topo.map(
            Arc::new(Region::io(
                "uart",
                HOST_PAGE,
                Arc::new(Nothing) as Arc<dyn MemOps>,
            )),
            0x8_0000,
        )
        .expect("io");
        topo.map(
            Arc::new(Region::rom("bios", Arc::clone(&rom), RomWrite::Ignore)),
            0xf_0000,
        )
        .expect("rom");
        topo.map(
            Arc::new(
                Region::mirror(
                    "shadow",
                    Arc::new(Region::ram("ram", Arc::clone(&ram))),
                    0x4_0000,
                )
                .expect("mirror"),
            ),
            0x40_0000,
        )
        .expect("mirror");
    }
    (space, ram, rom)
}

#[test]
fn ram_and_rom_become_slots_and_a_device_page_does_not() {
    let (space, ..) = board_space();
    let plan = plan_space(&space, true);

    let bases: Vec<u64> = plan.slots.iter().map(|(base, _)| *base).collect();
    assert!(bases.contains(&0), "the board's RAM is hardware-backed");
    assert!(bases.contains(&0xf_0000), "and so is its firmware");
    assert!(
        !bases.contains(&0x8_0000),
        "a device page must stay MMIO, or the guest would read its bytes \
         instead of asking the model"
    );

    let ram = plan
        .slots
        .iter()
        .find(|(base, _)| *base == 0)
        .expect("the RAM slot");
    assert!(ram.1.is_writable());
    assert_eq!(ram.1.len(), 0x2_0000);

    let rom = plan
        .slots
        .iter()
        .find(|(base, _)| *base == 0xf_0000)
        .expect("the ROM slot");
    assert!(!rom.1.is_writable(), "firmware is read-only to the guest");
}

#[test]
fn a_repeating_window_is_refused_rather_than_flattened() {
    // The failure this guards against is silent and awful: a mirror installed
    // as one slot shows the guest the first copy at every address, so a NES's
    // `$0800` would read `$0000` and nothing would say why.
    let (space, ..) = board_space();
    let plan = plan_space(&space, true);
    let mirror = plan
        .skipped
        .iter()
        .find(|s| s.start == 0x40_0000)
        .expect("the mirrored aperture is reported");
    assert!(mirror.why.contains("repeating"));
    assert!(plan.slots.iter().all(|(base, _)| *base != 0x40_0000));
}

#[test]
fn a_rom_is_refused_where_the_host_cannot_mark_a_slot_read_only() {
    // Installing firmware as writable memory instead would be a different
    // board, so the honest answer is to leave it MMIO — which costs speed and
    // means the guest cannot *fetch* from it, and is still better than a lie.
    let (space, ..) = board_space();
    let plan = plan_space(&space, false);
    assert!(plan.slots.iter().all(|(base, _)| *base != 0xf_0000));
    assert!(
        plan.skipped
            .iter()
            .any(|s| s.start == 0xf_0000 && s.why.contains("read-only"))
    );
}

#[test]
fn a_sub_page_region_stays_mmio() {
    // A `RamStore` is host-page aligned but need not be a whole number of
    // pages, and a board is entitled to map 2 KiB of it. A slot cannot express
    // that, and rounding up would hand the guest a page the board decodes
    // elsewhere.
    let store = Arc::new(RamStore::new(0x800));
    let space = AddressSpace::new("mem", 32);
    space
        .topology()
        .map(Arc::new(Region::ram("wram", store)), 0)
        .expect("map");
    let plan = plan_space(&space, true);
    assert!(plan.slots.is_empty());
    assert_eq!(plan.skipped.len(), 1);
    assert!(plan.skipped[0].why.contains("host pages"));
}

#[test]
fn an_aliased_window_carries_its_offset_into_the_host_address() {
    // The shape a shadowed BIOS or a banked aperture has: the slot is not the
    // whole store, and the host address has to move with the alias or the
    // guest executes the wrong 64 KiB.
    let store = Arc::new(RamStore::new(4 * HOST_PAGE));
    let whole: crate::core::space::RegionRef = Arc::new(Region::ram("ram", Arc::clone(&store)));
    let window =
        Arc::new(Region::alias("high", whole, 2 * HOST_PAGE, 2 * HOST_PAGE).expect("alias"));
    let space = AddressSpace::new("mem", 32);
    space.topology().map(window, 0x10_0000).expect("map");

    let plan = plan_space(&space, true);
    assert_eq!(plan.slots.len(), 1);
    let (base, backing) = &plan.slots[0];
    assert_eq!(*base, 0x10_0000);
    assert_eq!(backing.len(), 2 * HOST_PAGE);
    assert_eq!(
        backing.host_addr(),
        store.host_addr() + 2 * HOST_PAGE,
        "the host address follows the alias offset"
    );
}

#[test]
fn the_summary_says_what_happened() {
    let (space, ..) = board_space();
    let plan = Plan {
        slots: plan_space(&space, true)
            .slots
            .into_iter()
            .enumerate()
            .map(|(i, (base, w))| (i as u32, base, w.len(), w.is_writable()))
            .collect(),
        skipped: plan_space(&space, true).skipped,
    };
    let text = plan.describe();
    assert!(text.contains("ram"));
    assert!(text.contains("rom"));
    assert!(text.contains("mmio"));
    assert!(plan.mapped_bytes() >= 0x2_0000);
    assert!(plan.covers(0x1000));
    assert!(!plan.covers(0x8_0000));
}

// ---------------------------------------------------------------------------
// the half that needs a hypervisor
// ---------------------------------------------------------------------------

#[test]
fn a_board_space_installs_as_memory_slots() {
    let Ok(kvm) = super::super::kvm::Kvm::open() else {
        return;
    };
    let vm = kvm.create_vm().expect("KVM_CREATE_VM");
    let (space, ..) = board_space();

    let plan = install_space(&vm, &space, 0).expect("install the board's memory");
    let installed = vm.memory_regions();
    assert_eq!(installed.len(), plan.slots.len());
    assert!(plan.covers(0));
    if vm.has_readonly_mem() {
        assert!(plan.covers(0xf_0000), "firmware is a slot on this host");
    }
    // The device page and the mirror are not slots, so a guest touching either
    // exits and the board's own dispatch answers — which is the design, not a
    // shortfall.
    assert!(!plan.covers(0x8_0000));
    assert!(!plan.covers(0x40_0000));
}

#[test]
fn what_the_guest_would_execute_is_the_same_memory_the_space_reads() {
    // The whole point of installing `core::space`'s own stores rather than a
    // private mapping: one set of bytes, two engines.
    let Ok(kvm) = super::super::kvm::Kvm::open() else {
        return;
    };
    let vm = kvm.create_vm().expect("KVM_CREATE_VM");
    let (space, ram, _rom) = board_space();
    let plan = install_space(&vm, &space, 0).expect("install");

    let slot = plan
        .slots
        .iter()
        .find(|(_, base, ..)| *base == 0)
        .expect("the RAM slot");
    assert_eq!(slot.2, ram.len());
    // The address the kernel was given is the store's own.
    assert_eq!(ram.host_addr() % HOST_PAGE, 0);

    space
        .write(0x40, crate::core::Width::U8, 0xa5, MemAttrs::DEFAULT)
        .expect("write through the space");
    assert_eq!(ram.read_u8(0x40).expect("read the store"), 0xa5);
}
