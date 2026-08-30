//! Tests for the address-space module.
//!
//! The last one builds the NES memory map. It lives here rather than in
//! `tests/` because it exercises the same public API an integration test
//! would, and `CLAUDE.md` puts tests beside the module they cover — but it is
//! an integration test in shape: it is the real memory map of a real machine,
//! and it is the thing that says whether this design works.

use super::*;
use alloc::sync::Arc;
use alloc::vec;
use alloc::vec::Vec;
use core::sync::atomic::{AtomicU64, AtomicUsize, Ordering};

// ---------------------------------------------------------------------------
// Test devices
// ---------------------------------------------------------------------------

/// A FIFO whose read pointer advances — unless the access is a debug access.
#[derive(Debug)]
struct Fifo {
    data: Vec<u8>,
    pos: AtomicUsize,
}

impl Fifo {
    fn new(data: &[u8]) -> Self {
        Fifo {
            data: data.to_vec(),
            pos: AtomicUsize::new(0),
        }
    }
}

impl MemOps for Fifo {
    fn read(&self, _offset: u64, dst: &mut [u8], attrs: MemAttrs) -> MemResult {
        for b in dst.iter_mut() {
            let p = self.pos.load(Ordering::Relaxed);
            *b = self.data.get(p).copied().unwrap_or(0);
            if !attrs.debug {
                self.pos.store(p + 1, Ordering::Relaxed);
            }
        }
        Ok(())
    }

    fn write(&self, _offset: u64, _src: &[u8], attrs: MemAttrs) -> MemResult {
        if !attrs.debug {
            self.pos.store(0, Ordering::Relaxed);
        }
        Ok(())
    }
}

/// A register block that accepts 32-bit accesses and nothing else.
#[derive(Debug, Default)]
struct Reg32 {
    value: AtomicU64,
    writes: AtomicU64,
}

impl MemOps for Reg32 {
    fn read(&self, _offset: u64, dst: &mut [u8], _attrs: MemAttrs) -> MemResult {
        let v = self.value.load(Ordering::Relaxed) as u32;
        dst.copy_from_slice(&v.to_le_bytes()[..dst.len()]);
        Ok(())
    }

    fn write(&self, _offset: u64, src: &[u8], _attrs: MemAttrs) -> MemResult {
        let mut b = [0u8; 4];
        b.copy_from_slice(src);
        self.value
            .store(u64::from(u32::from_le_bytes(b)), Ordering::Relaxed);
        self.writes.fetch_add(1, Ordering::Relaxed);
        Ok(())
    }

    fn constraints(&self) -> AccessConstraints {
        AccessConstraints::word(Width::U32, Endian::Little)
    }
}

/// A big-endian 16-bit register.
#[derive(Debug, Default)]
struct BeReg {
    value: AtomicU64,
}

impl MemOps for BeReg {
    fn read(&self, _offset: u64, dst: &mut [u8], _attrs: MemAttrs) -> MemResult {
        let v = self.value.load(Ordering::Relaxed) as u16;
        dst.copy_from_slice(&v.to_be_bytes()[..dst.len()]);
        Ok(())
    }

    fn write(&self, _offset: u64, src: &[u8], _attrs: MemAttrs) -> MemResult {
        let mut b = [0u8; 2];
        b.copy_from_slice(src);
        self.value
            .store(u64::from(u16::from_be_bytes(b)), Ordering::Relaxed);
        Ok(())
    }

    fn constraints(&self) -> AccessConstraints {
        AccessConstraints::word(Width::U16, Endian::Big)
    }
}

/// Always busy.
#[derive(Debug)]
struct Busy;

impl MemOps for Busy {
    fn read(&self, _offset: u64, _dst: &mut [u8], _attrs: MemAttrs) -> MemResult {
        Err(BusError::Retry)
    }

    fn write(&self, _offset: u64, _src: &[u8], _attrs: MemAttrs) -> MemResult {
        Err(BusError::Retry)
    }

    fn constraints(&self) -> AccessConstraints {
        AccessConstraints::ANY
    }
}

/// A byte-addressable scratch device, for the cases that need I/O semantics
/// but not I/O behaviour.
#[derive(Debug)]
struct Scratch {
    cells: Vec<AtomicU64>,
    reads: AtomicU64,
}

impl Scratch {
    fn new(len: usize) -> Self {
        let mut cells = Vec::new();
        cells.resize_with(len, || AtomicU64::new(0));
        Scratch {
            cells,
            reads: AtomicU64::new(0),
        }
    }
}

impl MemOps for Scratch {
    fn read(&self, offset: u64, dst: &mut [u8], _attrs: MemAttrs) -> MemResult {
        self.reads.fetch_add(1, Ordering::Relaxed);
        for (i, b) in dst.iter_mut().enumerate() {
            *b = self
                .cells
                .get(offset as usize + i)
                .ok_or(BusError::BadAccess)?
                .load(Ordering::Relaxed) as u8;
        }
        Ok(())
    }

    fn write(&self, offset: u64, src: &[u8], _attrs: MemAttrs) -> MemResult {
        for (i, b) in src.iter().enumerate() {
            self.cells
                .get(offset as usize + i)
                .ok_or(BusError::BadAccess)?
                .store(u64::from(*b), Ordering::Relaxed);
        }
        Ok(())
    }

    fn constraints(&self) -> AccessConstraints {
        AccessConstraints::ANY
    }
}

fn ram(name: &str, len: u64) -> (Arc<RamStore>, Region) {
    let store = Arc::new(RamStore::new(len));
    let region = Region::ram(name, store.clone());
    (store, region)
}

// ---------------------------------------------------------------------------
// Basics
// ---------------------------------------------------------------------------

#[test]
fn ram_round_trips_through_the_space() {
    let mut space = AddressSpace::new("mem", 32);
    let (store, region) = ram("ram", 0x1000);
    space.map(region, 0x1000).unwrap();

    space
        .write(0x1004, Width::U32, 0xdead_beef, MemAttrs::DEFAULT)
        .unwrap();
    assert_eq!(
        space.read(0x1004, Width::U32, MemAttrs::DEFAULT).unwrap(),
        0xdead_beef
    );
    // The bytes landed at the right offset in the store, not just at the right
    // address in the space.
    let mut raw = [0u8; 4];
    store.read_at(4, &mut raw).unwrap();
    assert_eq!(raw, 0xdead_beefu32.to_le_bytes());
    // Byte-granular views of the same word.
    assert_eq!(
        space.read(0x1004, Width::U8, MemAttrs::DEFAULT).unwrap(),
        0xef
    );
    assert_eq!(
        space.read(0x1007, Width::U8, MemAttrs::DEFAULT).unwrap(),
        0xde
    );
}

#[test]
fn a_region_that_does_not_fit_is_refused_at_map_time() {
    let mut space = AddressSpace::new("small", 16);
    let (_, region) = ram("big", 0x1_0000);
    assert!(space.map(region, 0x8000).is_err());
    assert!(space.flat_view().is_empty());
}

// ---------------------------------------------------------------------------
// Priority
// ---------------------------------------------------------------------------

#[test]
fn a_bar_at_higher_priority_covers_the_ram_under_it() {
    let mut space = AddressSpace::new("mem", 32);
    let (ram_store, ram_region) = ram("ram", 0x4000);
    let bar_ops = Arc::new(Scratch::new(0x100));
    let bar = Region::io("bar", 0x100, bar_ops.clone());

    space.map(ram_region, 0).unwrap();
    space.map_with_priority(bar, 0x1000, 1).unwrap();

    // The RAM underneath still exists and is untouched by a write to the BAR.
    ram_store.write_at(0x1000, &[0xaa]).unwrap();
    space
        .write(0x1000, Width::U8, 0x5a, MemAttrs::DEFAULT)
        .unwrap();
    assert_eq!(
        space.read(0x1000, Width::U8, MemAttrs::DEFAULT).unwrap(),
        0x5a
    );
    assert_eq!(ram_store.read_u8(0x1000).unwrap(), 0xaa);

    // Either side of the window is RAM again.
    assert_eq!(
        space.read(0x0fff, Width::U8, MemAttrs::DEFAULT).unwrap(),
        0x00
    );
    ram_store.write_at(0x1100, &[0x77]).unwrap();
    assert_eq!(
        space.read(0x1100, Width::U8, MemAttrs::DEFAULT).unwrap(),
        0x77
    );

    // Three entries: RAM, BAR, RAM.
    assert_eq!(space.flat_view().len(), 3);
}

#[test]
fn equal_priority_is_broken_by_mapping_order() {
    let mut space = AddressSpace::new("mem", 16);
    let (first, a) = ram("a", 0x100);
    let (second, b) = ram("b", 0x100);
    first.write_at(0, &[1]).unwrap();
    second.write_at(0, &[2]).unwrap();
    space.map(a, 0).unwrap();
    space.map(b, 0).unwrap();
    // Later wins, so a machine file reads top to bottom with overrides last.
    assert_eq!(space.read(0, Width::U8, MemAttrs::DEFAULT).unwrap(), 2);
}

#[test]
fn priority_inside_a_container_does_not_leak_out_of_it() {
    // A low-priority container whose child has a huge priority must still lose
    // to the sibling mapped over the container.
    let (inner_store, inner) = ram("inner", 0x100);
    let (outer_store, outer) = ram("outer", 0x100);
    inner_store.write_at(0, &[0x11]).unwrap();
    outer_store.write_at(0, &[0x22]).unwrap();

    let container = Region::container(
        "bridge",
        0x100,
        vec![Mapping::new(inner, 0).with_priority(1000)],
    );

    let mut space = AddressSpace::new("mem", 16);
    space.map_with_priority(container, 0, 0).unwrap();
    space.map_with_priority(outer, 0, 1).unwrap();
    assert_eq!(space.read(0, Width::U8, MemAttrs::DEFAULT).unwrap(), 0x22);
}

// ---------------------------------------------------------------------------
// Aliases and mirrors
// ---------------------------------------------------------------------------

#[test]
fn an_alias_is_the_same_memory_seen_twice() {
    let mut space = AddressSpace::new("mem", 16);
    let (store, region) = ram("ram", 0x800);
    let region: RegionRef = region.into();
    let mirror = Region::alias("mirror", region.clone(), 0, 0x800).unwrap();
    space.map(region, 0).unwrap();
    space.map(mirror, 0x800).unwrap();

    space
        .write(0x0004, Width::U16, 0x1234, MemAttrs::DEFAULT)
        .unwrap();
    assert_eq!(
        space.read(0x0804, Width::U16, MemAttrs::DEFAULT).unwrap(),
        0x1234
    );
    // And back the other way.
    space
        .write(0x0806, Width::U8, 0x99, MemAttrs::DEFAULT)
        .unwrap();
    assert_eq!(
        space.read(0x0006, Width::U8, MemAttrs::DEFAULT).unwrap(),
        0x99
    );
    assert_eq!(store.read_u8(6).unwrap(), 0x99);
}

#[test]
fn an_alias_window_must_fit_inside_its_target() {
    let (_, region) = ram("ram", 0x800);
    let region: RegionRef = region.into();
    assert!(Region::alias("bad", region.clone(), 0x400, 0x800).is_err());
    assert!(Region::alias("ok", region, 0x400, 0x400).is_ok());
}

#[test]
fn a_repeating_window_is_one_entry_however_many_times_it_mirrors() {
    let mut space = AddressSpace::new("mem", 16);
    let (store, region) = ram("ram", 0x800);
    let region: RegionRef = region.into();
    let mirrored = Region::mirror("ram-mirror", region, 0x2000).unwrap();
    space.map(mirrored, 0).unwrap();

    assert_eq!(space.flat_view().len(), 1, "a mirror is one flat entry");

    store.write_at(0x10, &[0x5a]).unwrap();
    for base in [0x0000u64, 0x0800, 0x1000, 0x1800] {
        assert_eq!(
            space
                .read(base + 0x10, Width::U8, MemAttrs::DEFAULT)
                .unwrap(),
            0x5a,
            "mirror at {base:#x}"
        );
    }
    space
        .write(0x1810, Width::U8, 0xa5, MemAttrs::DEFAULT)
        .unwrap();
    assert_eq!(store.read_u8(0x10).unwrap(), 0xa5);

    // A transfer is cut at the wrap rather than running off the end of the
    // store.
    let mut buf = [0u8; 4];
    space
        .read_bytes(0x07fe, &mut buf, MemAttrs::DEFAULT)
        .unwrap();
    store.write_at(0x7fe, &[1, 2]).unwrap();
    store.write_at(0x000, &[3, 4]).unwrap();
    space
        .read_bytes(0x07fe, &mut buf, MemAttrs::DEFAULT)
        .unwrap();
    assert_eq!(buf, [1, 2, 3, 4]);
}

// ---------------------------------------------------------------------------
// Rebase versus retopology
// ---------------------------------------------------------------------------

#[test]
fn a_rebase_slides_a_window_without_bumping_the_generation() {
    // The MMC3 case: a 8 KiB window onto a 128 KiB PRG ROM, rebanked.
    let mut rom = Vec::new();
    for bank in 0..16u8 {
        rom.extend(core::iter::repeat_n(bank, 0x2000));
    }
    let rom = Arc::new(RomStore::new(rom));
    let prg: RegionRef = Region::rom("prg", rom, RomWrite::Ignore).into();
    let window: RegionRef = Region::alias("prg-bank", prg, 0, 0x2000).unwrap().into();

    let mut space = AddressSpace::new("cpu", 16);
    space.map(window.clone(), 0x8000).unwrap();

    let gen_before = space.generation();
    let entries_before = space.flat_view().len();

    for bank in 0..16u64 {
        space.rebase(&window, bank * 0x2000).unwrap();
        assert_eq!(
            space.read(0x8000, Width::U8, MemAttrs::DEFAULT).unwrap(),
            bank,
            "bank {bank}"
        );
        assert_eq!(
            space.read(0x9fff, Width::U8, MemAttrs::DEFAULT).unwrap(),
            bank
        );
    }

    assert_eq!(
        space.generation(),
        gen_before,
        "a rebase must not invalidate a single translation block"
    );
    assert_eq!(space.flat_view().len(), entries_before);
}

#[test]
fn a_retopology_bumps_the_generation() {
    let mut space = AddressSpace::new("mem", 16);
    let (_, a) = ram("a", 0x100);
    let (_, b) = ram("b", 0x100);

    let g0 = space.generation();
    let id = space.map(a, 0).unwrap();
    let g1 = space.generation();
    assert!(g1 > g0, "map is a retopology");

    space.map(b, 0x100).unwrap();
    let g2 = space.generation();
    assert!(g2 > g1);

    space.remap(id, 0x200).unwrap();
    let g3 = space.generation();
    assert!(g3 > g2, "moving a mapping is a retopology, not a rebase");

    space.unmap(id).unwrap();
    assert!(space.generation() > g3);
}

#[test]
fn a_rebase_through_two_aliases_composes() {
    let store = Arc::new(RomStore::new((0..64u8).collect()));
    let rom: RegionRef = Region::rom("rom", store, RomWrite::Ignore).into();
    let inner: RegionRef = Region::alias("inner", rom, 0, 32).unwrap().into();
    let outer: RegionRef = Region::alias("outer", inner.clone(), 0, 16).unwrap().into();

    let mut space = AddressSpace::new("mem", 16);
    space.map(outer.clone(), 0).unwrap();

    assert_eq!(space.read(0, Width::U8, MemAttrs::DEFAULT).unwrap(), 0);
    space.rebase(&inner, 16).unwrap();
    assert_eq!(space.read(0, Width::U8, MemAttrs::DEFAULT).unwrap(), 16);
    space.rebase(&outer, 8).unwrap();
    assert_eq!(space.read(0, Width::U8, MemAttrs::DEFAULT).unwrap(), 24);
    space.rebase(&inner, 0).unwrap();
    assert_eq!(space.read(0, Width::U8, MemAttrs::DEFAULT).unwrap(), 8);
}

#[test]
fn a_rebase_that_is_really_a_retopology_is_refused() {
    let (_, inner) = ram("inner", 0x100);
    let container: RegionRef =
        Region::container("bridge", 0x100, vec![Mapping::new(inner, 0)]).into();
    let window: RegionRef = Region::alias("win", container, 0, 0x80).unwrap().into();

    let mut space = AddressSpace::new("mem", 16);
    space.map(window.clone(), 0).unwrap();
    // Sliding a window onto a container changes which regions appear in it.
    assert!(space.rebase(&window, 0x80).is_err());

    // And a window cannot be slid off the end of what it looks at.
    let (_, leaf) = ram("leaf", 0x100);
    let leaf: RegionRef = leaf.into();
    let ok: RegionRef = Region::alias("ok", leaf, 0, 0x80).unwrap().into();
    space.map(ok.clone(), 0x100).unwrap();
    assert!(space.rebase(&ok, 0x80).is_ok());
    assert!(space.rebase(&ok, 0x81).is_err());
    // Not an alias at all.
    let (_, plain) = ram("plain", 0x10);
    assert!(space.rebase(&plain.into(), 0).is_err());
}

// ---------------------------------------------------------------------------
// Constraints
// ---------------------------------------------------------------------------

#[test]
fn a_32_bit_only_register_rejects_a_byte_write() {
    let ops = Arc::new(Reg32::default());
    let mut space = AddressSpace::new("mem", 16);
    space.map(Region::io("reg", 4, ops.clone()), 0x100).unwrap();

    assert_eq!(
        space.write(0x100, Width::U8, 0x12, MemAttrs::DEFAULT),
        Err(BusError::BadAccess)
    );
    assert_eq!(
        space.read(0x100, Width::U16, MemAttrs::DEFAULT),
        Err(BusError::BadAccess)
    );
    // Rejected before the device saw anything.
    assert_eq!(ops.writes.load(Ordering::Relaxed), 0);

    space
        .write(0x100, Width::U32, 0xcafe_f00d, MemAttrs::DEFAULT)
        .unwrap();
    assert_eq!(
        space.read(0x100, Width::U32, MemAttrs::DEFAULT).unwrap(),
        0xcafe_f00d
    );
    assert_eq!(ops.writes.load(Ordering::Relaxed), 1);

    // Misaligned, and a bulk burst, are both rejected too.
    space
        .map(Region::io("reg2", 8, Arc::new(Reg32::default())), 0x200)
        .unwrap();
    assert_eq!(
        space.write(0x202, Width::U32, 0, MemAttrs::DEFAULT),
        Err(BusError::BadAccess)
    );
    let mut buf = [0u8; 8];
    assert_eq!(
        space.read_bytes(0x200, &mut buf, MemAttrs::DEFAULT),
        Err(BusError::BadAccess)
    );
}

#[test]
fn secure_and_privileged_regions_reject_the_wrong_master() {
    let (_, region) = ram("secure-ram", 0x100);
    let region = region.with_constraints(AccessConstraints::ANY.with_secure_only(true));
    let mut space = AddressSpace::new("mem", 16);
    space.map(region, 0).unwrap();

    assert_eq!(
        space.read(0, Width::U8, MemAttrs::DEFAULT),
        Err(BusError::BadAccess)
    );
    assert!(
        space
            .read(0, Width::U8, MemAttrs::DEFAULT.with_secure(true))
            .is_ok()
    );
    // A debug access is a secure one, so a monitor can still see the machine.
    assert!(space.read(0, Width::U8, MemAttrs::DEBUG).is_ok());
}

#[test]
fn per_region_endianness_is_honoured() {
    let be = Arc::new(BeReg::default());
    let mut space = AddressSpace::new("mem", 16);
    space.map(Region::io("be", 2, be.clone()), 0x10).unwrap();
    let (store, le) = ram("le", 2);
    space.map(le, 0x20).unwrap();

    space
        .write(0x10, Width::U16, 0x1234, MemAttrs::DEFAULT)
        .unwrap();
    space
        .write(0x20, Width::U16, 0x1234, MemAttrs::DEFAULT)
        .unwrap();

    // Same value, opposite byte order on the wire.
    let mut wire = [0u8; 2];
    be.read(0, &mut wire, MemAttrs::DEFAULT).unwrap();
    assert_eq!(wire, [0x12, 0x34]);
    store.read_at(0, &mut wire).unwrap();
    assert_eq!(wire, [0x34, 0x12]);

    // And the value round-trips through the space in both.
    assert_eq!(
        space.read(0x10, Width::U16, MemAttrs::DEFAULT).unwrap(),
        0x1234
    );
    assert_eq!(
        space.read(0x20, Width::U16, MemAttrs::DEFAULT).unwrap(),
        0x1234
    );
}

// ---------------------------------------------------------------------------
// Debug attribute
// ---------------------------------------------------------------------------

#[test]
fn a_debug_read_does_not_pop_a_fifo() {
    let fifo = Arc::new(Fifo::new(&[0x11, 0x22, 0x33]));
    let mut space = AddressSpace::new("mem", 16);
    space
        .map(Region::io("fifo", 1, fifo.clone()), 0x40)
        .unwrap();

    // The monitor can look as often as it likes.
    for _ in 0..5 {
        assert_eq!(space.read(0x40, Width::U8, MemAttrs::DEBUG).unwrap(), 0x11);
    }
    assert_eq!(fifo.pos.load(Ordering::Relaxed), 0);

    // The guest pops.
    assert_eq!(
        space.read(0x40, Width::U8, MemAttrs::DEFAULT).unwrap(),
        0x11
    );
    assert_eq!(space.read(0x40, Width::U8, MemAttrs::DEBUG).unwrap(), 0x22);
    assert_eq!(
        space.read(0x40, Width::U8, MemAttrs::DEFAULT).unwrap(),
        0x22
    );
    assert_eq!(fifo.pos.load(Ordering::Relaxed), 2);
}

#[test]
fn a_debug_access_does_not_move_the_unassigned_log() {
    let space = AddressSpace::new("mem", 16).with_unassigned(UnassignedPolicy::ONES.logged());
    space.read(0x1234, Width::U8, MemAttrs::DEBUG).unwrap();
    assert_eq!(space.unassigned_log().count, 0);
    space.read(0x1234, Width::U8, MemAttrs::DEFAULT).unwrap();
    let log = space.unassigned_log();
    assert_eq!(log.count, 1);
    assert_eq!(log.last_addr, 0x1234);
    assert!(!log.last_was_write);
    space
        .write(0x4321, Width::U8, 0, MemAttrs::DEFAULT)
        .unwrap();
    let log = space.unassigned_log();
    assert_eq!(log.count, 2);
    assert!(log.last_was_write);
}

// ---------------------------------------------------------------------------
// Unassigned policy
// ---------------------------------------------------------------------------

#[test]
fn unassigned_policies() {
    for (policy, expected) in [
        (UnassignedPolicy::FAULT, Err(BusError::Unassigned)),
        (UnassignedPolicy::ONES, Ok(0xffff)),
        (UnassignedPolicy::ZEROS, Ok(0)),
    ] {
        let space = AddressSpace::new("mem", 16).with_unassigned(policy);
        assert_eq!(
            space.read(0x1000, Width::U16, MemAttrs::DEFAULT),
            expected,
            "{policy:?}"
        );
        let write = space.write(0x1000, Width::U16, 0, MemAttrs::DEFAULT);
        assert_eq!(
            write.is_err(),
            policy.action == UnassignedAction::Fault,
            "{policy:?}"
        );
    }
}

#[test]
fn a_hole_between_two_regions_follows_the_policy() {
    let mut space = AddressSpace::new("mem", 16).with_unassigned(UnassignedPolicy::ONES);
    let (a_store, a) = ram("a", 2);
    let (b_store, b) = ram("b", 2);
    a_store.write_at(0, &[1, 2]).unwrap();
    b_store.write_at(0, &[3, 4]).unwrap();
    space.map(a, 0).unwrap();
    space.map(b, 6).unwrap();

    let mut buf = [0u8; 8];
    space.read_bytes(0, &mut buf, MemAttrs::DEFAULT).unwrap();
    assert_eq!(buf, [1, 2, 0xff, 0xff, 0xff, 0xff, 3, 4]);
}

// ---------------------------------------------------------------------------
// Retry
// ---------------------------------------------------------------------------

#[test]
fn retry_is_returned_before_a_commit_and_refused_after_one() {
    let mut space = AddressSpace::new("mem", 16);
    let (_, region) = ram("ram", 4);
    space.map(region, 0).unwrap();
    space.map(Region::io("busy", 4, Arc::new(Busy)), 4).unwrap();

    // Nothing has happened yet: the caller may retry.
    let mut buf = [0u8; 4];
    assert_eq!(
        space.read_bytes(4, &mut buf, MemAttrs::DEFAULT),
        Err(BusError::Retry)
    );

    // The RAM half already transferred, so re-running the access would read it
    // twice. That is a correctness bug, so the retry is refused.
    let mut buf = [0u8; 8];
    assert_eq!(
        space.read_bytes(0, &mut buf, MemAttrs::DEFAULT),
        Err(BusError::BadAccess)
    );
    assert_eq!(
        space.write_bytes(0, &[0; 8], MemAttrs::DEFAULT),
        Err(BusError::BadAccess)
    );
}

// ---------------------------------------------------------------------------
// Dirty tracking
// ---------------------------------------------------------------------------

#[test]
fn writes_mark_pages_dirty_and_reads_do_not() {
    let mut space = AddressSpace::new("mem", 32);
    let store = Arc::new(RamStore::with_page_bits(0x4000, 12));
    space.map(Region::ram("fb", store.clone()), 0).unwrap();

    assert_eq!(store.dirty_page_count(), 0);
    space.read(0x2000, Width::U32, MemAttrs::DEFAULT).unwrap();
    assert_eq!(store.dirty_page_count(), 0, "a read is not a write");

    space
        .write(0x2004, Width::U32, 1, MemAttrs::DEFAULT)
        .unwrap();
    assert!(store.is_page_dirty(2));
    assert!(!store.is_page_dirty(1));

    let mut pages = Vec::new();
    store.for_each_dirty_page(|p| pages.push(p));
    assert_eq!(pages, vec![2]);

    assert!(store.take_page_dirty(2));
    assert!(!store.take_page_dirty(2));

    // A write that straddles a page boundary dirties both.
    space
        .write_bytes(0x0ffe, &[0; 4], MemAttrs::DEFAULT)
        .unwrap();
    let mut pages = Vec::new();
    store.for_each_dirty_page(|p| pages.push(p));
    assert_eq!(pages, vec![0, 1]);

    store.clear_dirty();
    assert_eq!(store.dirty_page_count(), 0);

    // A debug *write* still dirties: the bytes really did change, and a
    // framebuffer that missed it would show stale pixels.
    space
        .write(0x3000, Width::U8, 0xff, MemAttrs::DEBUG)
        .unwrap();
    assert!(store.is_page_dirty(3));
}

// ---------------------------------------------------------------------------
// Dispatch
// ---------------------------------------------------------------------------

#[test]
fn a_dense_table_agrees_with_the_flat_view_everywhere() {
    let build = |policy: DispatchPolicy| {
        let mut space = AddressSpace::new("mem", 32).with_dispatch(policy);
        let store = Arc::new(RamStore::new(0x8000));
        for i in 0..0x8000u64 {
            store.write_u8(i, (i & 0xff) as u8).unwrap();
        }
        space.map(Region::ram("ram", store), 0x1_0000).unwrap();
        space
            .map(Region::io("io", 0x20, Arc::new(Scratch::new(0x20))), 0x4000)
            .unwrap();
        space
            .map(Region::io("io2", 0x8, Arc::new(Scratch::new(0x8))), 0x4100)
            .unwrap();
        space.with_unassigned(UnassignedPolicy::ONES)
    };
    let flat = build(DispatchPolicy::Flat);
    let dense = build(DispatchPolicy::Dense {
        page_bits: 12,
        cover: 0x2_0000,
    });
    assert!(flat.dispatch().is_none());
    let table = dense.dispatch().expect("dense table asked for");
    assert_eq!(table.len(), 0x20);

    // The 32-byte I/O window is a sub-page entry; a full RAM page is direct.
    assert_eq!(table.lookup(0x4000), Some(DispatchEntry::SubPage));
    assert_eq!(table.lookup(0x1_1000), {
        let i = dense.flat_view().find(0x1_1000).unwrap() as u32;
        Some(DispatchEntry::Direct(i))
    });
    assert_eq!(table.lookup(0x3000), Some(DispatchEntry::Unassigned));
    // Above the table's reach, lookups fall through to the flat view.
    assert_eq!(table.lookup(0x2_0000), None);

    for addr in [
        0u64,
        0x3fff,
        0x4000,
        0x4001,
        0x401f,
        0x4020,
        0x4100,
        0x4108,
        0xffff,
        0x1_0000,
        0x1_0001,
        0x1_7fff,
        0x1_8000,
        0x2_0000,
        0xffff_ffff,
    ] {
        assert_eq!(
            flat.read(addr, Width::U8, MemAttrs::DEFAULT),
            dense.read(addr, Width::U8, MemAttrs::DEFAULT),
            "{addr:#x}"
        );
        assert_eq!(flat.locate(addr), dense.locate(addr), "{addr:#x}");
    }
}

#[test]
fn auto_dispatch_declines_a_tiny_map() {
    let mut space = AddressSpace::new("mem", 16).with_dispatch(DispatchPolicy::Auto);
    let (_, region) = ram("ram", 0x800);
    space.map(region, 0).unwrap();
    assert!(space.dispatch().is_none(), "one entry needs no table");
}

// ---------------------------------------------------------------------------
// Combine policies
// ---------------------------------------------------------------------------

#[test]
fn a_wired_or_container_combines_its_children() {
    let (a_store, a) = ram("a", 1);
    let (b_store, b) = ram("b", 1);
    a_store.write_at(0, &[0b1010_0000]).unwrap();
    b_store.write_at(0, &[0b0000_0101]).unwrap();
    let bus = Region::container_with(
        "open-bus",
        1,
        vec![Mapping::new(a, 0), Mapping::new(b, 0)],
        CombinePolicy::WiredOr,
    );

    let mut space = AddressSpace::new("mem", 16);
    space.map(bus, 0).unwrap();
    assert_eq!(
        space.read(0, Width::U8, MemAttrs::DEFAULT).unwrap(),
        0b1010_0101
    );
    // A write reaches every responder, which is what a wired bus does.
    space.write(0, Width::U8, 0x0f, MemAttrs::DEFAULT).unwrap();
    assert_eq!(a_store.read_u8(0).unwrap(), 0x0f);
    assert_eq!(b_store.read_u8(0).unwrap(), 0x0f);
}

// ---------------------------------------------------------------------------
// The NES memory map — the shape this design has to support
// ---------------------------------------------------------------------------

/// PPU register file: eight registers, and reading `$2002` clears the vblank
/// flag unless the access is a debug access.
#[derive(Debug, Default)]
struct PpuRegs {
    status: AtomicU64,
    latch: AtomicU64,
}

impl MemOps for PpuRegs {
    fn read(&self, offset: u64, dst: &mut [u8], attrs: MemAttrs) -> MemResult {
        let v = match offset {
            2 => {
                let v = self.status.load(Ordering::Relaxed);
                if !attrs.debug {
                    self.status.store(v & 0x7f, Ordering::Relaxed);
                }
                v
            }
            _ => self.latch.load(Ordering::Relaxed),
        };
        dst[0] = v as u8;
        Ok(())
    }

    fn write(&self, _offset: u64, src: &[u8], _attrs: MemAttrs) -> MemResult {
        self.latch.store(u64::from(src[0]), Ordering::Relaxed);
        Ok(())
    }

    fn constraints(&self) -> AccessConstraints {
        AccessConstraints::word(Width::U8, Endian::Little)
    }
}

#[test]
fn the_nes_memory_map() {
    // $0000-$07FF  2 KiB internal RAM, mirrored through $1FFF
    // $2000-$2007  PPU registers, mirrored every 8 bytes through $3FFF
    // $4000-$401F  APU and I/O registers
    // $8000-$FFFF  cartridge PRG ROM, an 8 KiB switchable bank at $8000
    let wram = Arc::new(RamStore::new(0x800));
    let wram_region: RegionRef = Region::ram("wram", wram.clone()).into();
    let ppu_ops = Arc::new(PpuRegs::default());
    let ppu_region: RegionRef = Region::io("ppu", 8, ppu_ops.clone()).into();
    let apu_ops = Arc::new(Scratch::new(0x20));
    let mut prg = Vec::new();
    for bank in 0..8u8 {
        prg.extend(core::iter::repeat_n(bank, 0x2000));
    }
    let prg: RegionRef = Region::rom("prg", Arc::new(RomStore::new(prg)), RomWrite::Ignore).into();
    let bank_lo: RegionRef = Region::alias("prg-lo", prg.clone(), 0, 0x2000)
        .unwrap()
        .into();
    let bank_hi: RegionRef = Region::alias("prg-hi", prg, 0xc000, 0x4000).unwrap().into();

    // The 6502 sees an open bus: unmapped reads return the last thing on it,
    // which for our purposes is all-ones, and we want them counted.
    let mut cpu = AddressSpace::new("cpu", 16)
        .with_unassigned(UnassignedPolicy::ONES.logged())
        .with_dispatch(DispatchPolicy::Dense {
            page_bits: 12,
            cover: 0x1_0000,
        });

    cpu.map(
        Region::mirror("wram-mirror", wram_region, 0x2000).unwrap(),
        0,
    )
    .unwrap();
    cpu.map(
        Region::mirror("ppu-mirror", ppu_region, 0x2000).unwrap(),
        0x2000,
    )
    .unwrap();
    cpu.map(Region::io("apu-io", 0x20, apu_ops.clone()), 0x4000)
        .unwrap();
    cpu.map(bank_lo.clone(), 0x8000).unwrap();
    cpu.map(bank_hi, 0xc000).unwrap();

    // Five regions, five flat entries plus the two holes are not entries.
    assert_eq!(cpu.flat_view().len(), 5);

    // --- RAM, mirrored four times -------------------------------------
    cpu.write(0x0000, Width::U8, 0x42, MemAttrs::DEFAULT)
        .unwrap();
    for base in [0x0000u64, 0x0800, 0x1000, 0x1800] {
        assert_eq!(
            cpu.read(base, Width::U8, MemAttrs::DEFAULT).unwrap(),
            0x42,
            "RAM mirror at {base:#x}"
        );
    }
    cpu.write(0x1fff, Width::U8, 0x37, MemAttrs::DEFAULT)
        .unwrap();
    assert_eq!(wram.read_u8(0x7ff).unwrap(), 0x37);
    assert_eq!(
        cpu.read(0x07ff, Width::U8, MemAttrs::DEFAULT).unwrap(),
        0x37
    );

    // The 6502 stack lives at $0100-$01FF and is ordinary RAM.
    cpu.write(0x01fd, Width::U8, 0x80, MemAttrs::DEFAULT)
        .unwrap();
    assert_eq!(wram.read_u8(0x1fd).unwrap(), 0x80);

    // --- PPU registers, mirrored every eight bytes ---------------------
    ppu_ops.status.store(0x80, Ordering::Relaxed);
    // A debugger may look at $2002 without clearing vblank.
    assert_eq!(cpu.read(0x2002, Width::U8, MemAttrs::DEBUG).unwrap(), 0x80);
    assert_eq!(ppu_ops.status.load(Ordering::Relaxed), 0x80);
    // The guest reads it, at the last mirror in the range, and it clears.
    assert_eq!(
        cpu.read(0x3ffa, Width::U8, MemAttrs::DEFAULT).unwrap(),
        0x80
    );
    assert_eq!(ppu_ops.status.load(Ordering::Relaxed), 0x00);
    // $2000 and $3FF8 are the same register.
    cpu.write(0x3ff8, Width::U8, 0x1e, MemAttrs::DEFAULT)
        .unwrap();
    assert_eq!(
        cpu.read(0x2000, Width::U8, MemAttrs::DEFAULT).unwrap(),
        0x1e
    );
    // The PPU is byte-only; a 16-bit access to it is a bus error, not two
    // silent register reads.
    assert_eq!(
        cpu.read(0x2000, Width::U16, MemAttrs::DEFAULT),
        Err(BusError::BadAccess)
    );

    // --- APU and controller ports at $4000, 32 bytes -------------------
    cpu.write(0x4016, Width::U8, 0x01, MemAttrs::DEFAULT)
        .unwrap();
    assert_eq!(
        cpu.read(0x4016, Width::U8, MemAttrs::DEFAULT).unwrap(),
        0x01
    );
    // $4020 is past the end of the block: open bus, and counted.
    assert_eq!(
        cpu.read(0x4020, Width::U8, MemAttrs::DEFAULT).unwrap(),
        0xff
    );
    assert_eq!(cpu.unassigned_log().count, 1);
    assert_eq!(cpu.unassigned_log().last_addr, 0x4020);
    // So is the cartridge expansion area at $6000 with no WRAM fitted.
    assert_eq!(
        cpu.read(0x6000, Width::U8, MemAttrs::DEFAULT).unwrap(),
        0xff
    );

    // --- PRG ROM, and a mapper rebanking it ----------------------------
    assert_eq!(cpu.read(0x8000, Width::U8, MemAttrs::DEFAULT).unwrap(), 0);
    assert_eq!(cpu.read(0xc000, Width::U8, MemAttrs::DEFAULT).unwrap(), 6);
    assert_eq!(cpu.read(0xfffc, Width::U8, MemAttrs::DEFAULT).unwrap(), 7);
    // Writing ROM is swallowed, not faulted: that is how a mapper register is
    // written on most boards.
    cpu.write(0x8000, Width::U8, 0xff, MemAttrs::DEFAULT)
        .unwrap();
    assert_eq!(cpu.read(0x8000, Width::U8, MemAttrs::DEFAULT).unwrap(), 0);

    let gen_before = cpu.generation();
    for bank in 0..6u64 {
        cpu.rebase(&bank_lo, bank * 0x2000).unwrap();
        assert_eq!(
            cpu.read(0x8000, Width::U8, MemAttrs::DEFAULT).unwrap(),
            bank
        );
    }
    assert_eq!(
        cpu.generation(),
        gen_before,
        "an MMC3 rebanks 15000 times a second; none of them may be a retopology"
    );

    // --- The dispatch table over a 64 KiB space ------------------------
    let table = cpu.dispatch().expect("dense table asked for");
    assert_eq!(table.len(), 16);
    // RAM pages are the fast path.
    assert!(matches!(
        table.lookup(0x0000),
        Some(DispatchEntry::Direct(_))
    ));
    assert!(matches!(
        table.lookup(0x1000),
        Some(DispatchEntry::Direct(_))
    ));
    // The APU's 32 bytes at $4000 cannot be described by a 4 KiB page.
    assert_eq!(table.lookup(0x4000), Some(DispatchEntry::SubPage));
    // $6000-$6FFF has nothing in it at all.
    assert_eq!(table.lookup(0x6000), Some(DispatchEntry::Unassigned));
    // ROM pages are mapped, but not the RAM fast path.
    assert!(matches!(
        table.lookup(0x8000),
        Some(DispatchEntry::Mapped(_))
    ));

    // --- A DMA-shaped bulk transfer: OAM DMA copies a page of RAM ------
    for i in 0..0x100u64 {
        wram.write_u8(0x200 + i, i as u8).unwrap();
    }
    let mut oam = [0u8; 0x100];
    cpu.read_bytes(0x0200, &mut oam, MemAttrs::DEFAULT).unwrap();
    assert_eq!(oam[0x42], 0x42);
}

// ---------------------------------------------------------------------------
// Nesting
// ---------------------------------------------------------------------------

#[test]
fn a_bridge_window_translates_addresses_through_two_levels() {
    // A container standing in for a bridge: a device sits at $40 inside it,
    // and the bridge's aperture is at $1000 in the space. The device must see
    // its own register offsets, not the space's addresses.
    let ops = Arc::new(Scratch::new(0x10));
    let inner = Region::container(
        "bridge",
        0x100,
        vec![Mapping::new(Region::io("dev", 0x10, ops.clone()), 0x40)],
    );
    let outer = Region::container("root-bus", 0x1000, vec![Mapping::new(inner, 0x200)]);

    let mut space = AddressSpace::new("mem", 32).with_unassigned(UnassignedPolicy::ZEROS);
    space.map(outer, 0x1000).unwrap();

    // $1000 + $200 + $40 + 3
    space
        .write(0x1243, Width::U8, 0xab, MemAttrs::DEFAULT)
        .unwrap();
    let mut seen = [0u8; 1];
    ops.read(3, &mut seen, MemAttrs::DEFAULT).unwrap();
    assert_eq!(seen[0], 0xab, "the device sees its own register offset");

    // Only the device's 16 bytes are mapped; the rest of the bridge is a hole.
    assert_eq!(space.flat_view().len(), 1);
    assert_eq!(space.flat_view().entries()[0].start(), 0x1240);
    assert_eq!(space.flat_view().entries()[0].len(), 0x10);
}

#[test]
fn an_alias_onto_a_container_exposes_its_children_shifted() {
    let (store, region) = ram("ram", 0x100);
    store.write_at(0x80, &[0x5a]).unwrap();
    let container: RegionRef =
        Region::container("bus", 0x100, vec![Mapping::new(region, 0)]).into();
    // The top half of the container, seen at $2000.
    let window = Region::alias("upper-half", container, 0x80, 0x80).unwrap();

    let mut space = AddressSpace::new("mem", 16);
    space.map(window, 0x2000).unwrap();
    assert_eq!(
        space.read(0x2000, Width::U8, MemAttrs::DEFAULT).unwrap(),
        0x5a
    );
}

#[test]
fn a_container_does_not_decode_past_its_own_end() {
    let (store, region) = ram("ram", 0x100);
    store.write_at(0xff, &[0x11]).unwrap();
    // The container is only $80 wide, so the top half of the RAM inside it is
    // simply not decoded — a window does not wrap.
    let container = Region::container("narrow", 0x80, vec![Mapping::new(region, 0)]);
    let mut space = AddressSpace::new("mem", 16).with_unassigned(UnassignedPolicy::ONES);
    space.map(container, 0).unwrap();

    assert_eq!(space.flat_view().extent(), 0x80);
    assert_eq!(space.read(0x7f, Width::U8, MemAttrs::DEFAULT).unwrap(), 0);
    assert_eq!(
        space.read(0x80, Width::U8, MemAttrs::DEFAULT).unwrap(),
        0xff
    );
}
