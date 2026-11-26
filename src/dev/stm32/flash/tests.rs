//! `st.flash`, register by register and sector by sector.
//!
//! As in [`pwr`](super::super::pwr), the device is lazily advanced and the
//! tests drive [`Device::advance_to`] themselves rather than standing a
//! scheduler up.

use super::*;

use crate::core::props::Value;
use crate::core::space::{AddressSpace, FlatEntry, UnassignedPolicy};
use crate::core::state::{MachineShape, Migrations, StateReader, StateWriter};
use crate::core::wire::{Wire, WireIdAllocator, WireSink};

/// A 1 MiB F407, the part `machines/stm32f407.machine` models.
const F4_SIZE: u64 = 1024 * 1024;
/// A 1 MiB dual-bank L476, which is what makes `BKER` mean anything.
const L4_SIZE: u64 = 1024 * 1024;

fn build(variant: Variant, size: u64, optr: Option<u64>) -> Flash {
    let mut props = Props::new()
        .with("variant", Value::Str(String::from(variant.as_str())))
        .with("size", Value::Uint(size));
    if let Some(optr) = optr {
        props = props.with("optr", Value::Uint(optr));
    }
    Flash::new(&props).expect("a flash")
}

fn f4() -> Flash {
    build(Variant::F4, F4_SIZE, None)
}

/// An L4 with `OPTR.DUALBANK` set, so bank 2 exists and `BKER` selects it.
fn l4() -> Flash {
    build(Variant::L4, L4_SIZE, Some(0xffef_f8aa | (1 << 21)))
}

fn tick(flash: &Flash, ticks: u64) {
    let now = Device::current_tick(flash);
    Device::advance_to(flash, now + ticks);
}

/// Run whatever is pending to completion.
fn settle(flash: &Flash) {
    tick(flash, DEFAULT_MASS_ERASE_TIME + 1);
}

/// The unlock sequence, as firmware writes it.
fn unlock(flash: &Flash) {
    let keyr = match flash.variant() {
        Variant::F4 => F4_KEYR,
        Variant::U5 => U5_NSKEYR,
        Variant::L4 | Variant::L4Plus | Variant::Wb => L4_KEYR,
    };
    flash.poke(keyr, KEY1).expect("KEY1");
    flash.poke(keyr, KEY2).expect("KEY2");
}

/// The option unlock sequence.
fn opt_unlock(flash: &Flash) {
    let reg = match flash.variant() {
        Variant::F4 => F4_OPTKEYR,
        Variant::U5 => U5_OPTKEYR,
        Variant::L4 | Variant::L4Plus | Variant::Wb => L4_OPTKEYR,
    };
    flash.poke(reg, OPTKEY1).expect("OPTKEY1");
    flash.poke(reg, OPTKEY2).expect("OPTKEY2");
}

fn sr(flash: &Flash) -> u32 {
    flash.peek(flash.shared.sr_offset())
}

fn clear_sr(flash: &Flash) {
    let offset = flash.shared.sr_offset();
    flash.poke(offset, 0xffff_ffff).expect("a status clear");
}

fn cr(flash: &Flash) -> u64 {
    flash.shared.cr_offset()
}

/// A word out of the array, straight off the store — never through the
/// controller, so a test can read what a fetch would see.
fn word(flash: &Flash, offset: u64) -> u32 {
    let mut buf = [0u8; 4];
    flash
        .shared
        .array
        .read_at(offset, &mut buf)
        .expect("a read");
    u32::from_le_bytes(buf)
}

/// A level probe, for the `irq` and `reset` outputs.
#[derive(Debug, Default)]
struct Probe {
    high: crate::core::sync::AtomicU32,
    edges: crate::core::sync::AtomicU32,
}

impl Probe {
    fn is_high(&self) -> bool {
        self.high.load(Ordering::Relaxed) != 0
    }

    fn rising_edges(&self) -> u32 {
        self.edges.load(Ordering::Relaxed)
    }
}

impl WireSink for Probe {
    fn set_level(&self, _src: crate::core::wire::WireId, _line: u32, level: Level) {
        if level.is_high() {
            self.edges.fetch_add(1, Ordering::Relaxed);
        }
        self.high
            .store(u32::from(level.is_high()), Ordering::Relaxed);
    }
}

/// Wire `port` of `flash` to a fresh probe.
fn probe(flash: &Flash, port: &str) -> Arc<Probe> {
    let ids = WireIdAllocator::new();
    let id = ids.alloc();
    let sink = Arc::new(Probe::default());
    let wire = Wire::builder()
        .source(id)
        .sink(Arc::clone(&sink) as Arc<dyn WireSink>, 0)
        .build_shared();
    Device::connect(flash, port, WireSource::new(wire, id)).expect("a connection");
    sink
}

// ---------------------------------------------------------------------------
// Geometry — RM0090 Table 5 is the detail most often got wrong
// ---------------------------------------------------------------------------

#[test]
fn the_f4_sector_map_has_four_small_sectors_one_medium_and_the_rest_large() {
    // RM0090 Table 5, "Flash module organization" for a 1 MiB part.
    let expect: [(u64, u64); 12] = [
        (0x0_0000, 16 * 1024),
        (0x0_4000, 16 * 1024),
        (0x0_8000, 16 * 1024),
        (0x0_c000, 16 * 1024),
        (0x1_0000, 64 * 1024),
        (0x2_0000, 128 * 1024),
        (0x4_0000, 128 * 1024),
        (0x6_0000, 128 * 1024),
        (0x8_0000, 128 * 1024),
        (0xa_0000, 128 * 1024),
        (0xc_0000, 128 * 1024),
        (0xe_0000, 128 * 1024),
    ];
    for (snb, want) in expect.iter().enumerate() {
        assert_eq!(
            f4_sector(F4_SIZE, snb as u32),
            Some(*want),
            "sector {snb} of a 1 MiB F4"
        );
    }
    // The sectors tile the part exactly and there is no sector 12 on 1 MiB.
    assert_eq!(f4_sector(F4_SIZE, 12), None);
    let total: u64 = expect.iter().map(|&(_, len)| len).sum();
    assert_eq!(total, F4_SIZE);

    // A 2 MiB part repeats the pattern from sector 12 (RM0090 Table 6).
    assert_eq!(f4_sector(2 * BANK, 12), Some((BANK, 16 * 1024)));
    assert_eq!(f4_sector(2 * BANK, 16), Some((BANK + 0x1_0000, 64 * 1024)));
    assert_eq!(f4_sector(2 * BANK, 23), Some((BANK + 0xe_0000, 128 * 1024)));
    assert_eq!(f4_sector(2 * BANK, 24), None);

    // And a half-megabyte part simply stops.
    assert_eq!(f4_sector(512 * 1024, 7), Some((0x6_0000, 128 * 1024)));
    assert_eq!(f4_sector(512 * 1024, 8), None);
}

#[test]
fn a_sector_lookup_finds_the_right_unequal_sector() {
    assert_eq!(f4_sector_of(F4_SIZE, 0), Some(0));
    assert_eq!(f4_sector_of(F4_SIZE, 16 * 1024 - 1), Some(0));
    assert_eq!(f4_sector_of(F4_SIZE, 16 * 1024), Some(1));
    assert_eq!(f4_sector_of(F4_SIZE, 0x1_0000), Some(4));
    assert_eq!(f4_sector_of(F4_SIZE, 0x1_ffff), Some(4));
    assert_eq!(f4_sector_of(F4_SIZE, 0x2_0000), Some(5));
    assert_eq!(f4_sector_of(F4_SIZE, F4_SIZE - 1), Some(11));
    assert_eq!(f4_sector_of(F4_SIZE, F4_SIZE), None);
}

// ---------------------------------------------------------------------------
// The mapping: reads come off the store, writes reach the controller
// ---------------------------------------------------------------------------

#[test]
fn the_array_region_reads_from_the_store_and_writes_through_the_controller() {
    let flash = f4();
    flash.load_image(0, &0xdead_beefu32.to_le_bytes()).unwrap();

    let space = AddressSpace::new("mem", 32).with_unassigned(UnassignedPolicy::FAULT);
    let region = Device::region(&flash, "array").expect("the array region");
    space.topology().map(region, 0x0800_0000).unwrap();

    // A read resolves to the store, not to this device.
    assert_eq!(
        space
            .read(0x0800_0000, Width::U32, MemAttrs::DEFAULT)
            .unwrap(),
        0xdead_beef
    );
    let view = space.view();
    let entry: &FlatEntry = view
        .locate(0x0800_0000)
        .and_then(|i| view.flat_view().entry(i))
        .expect("an entry");
    assert!(
        entry.write_to().is_some(),
        "the write side is a separate leaf — the directed split the design rests on"
    );
    assert!(
        !entry.is_direct_ram(),
        "which is what costs the RAM fast path"
    );
    drop(view);

    // A write with `LOCK` set is swallowed by the controller.
    space
        .write(0x0800_0000, Width::U32, 0, MemAttrs::DEFAULT)
        .expect("no bus fault");
    assert_eq!(
        space
            .read(0x0800_0000, Width::U32, MemAttrs::DEFAULT)
            .unwrap(),
        0xdead_beef,
        "the array is unchanged"
    );
    assert_eq!(sr(&flash) & SR_PGSERR, SR_PGSERR);
}

#[test]
fn a_read_only_mapping_of_the_array_refuses_writes_outright() {
    // What a board gets when it writes `perms = "r--"` on a flash window — the
    // `dmabus` line on `machines/stm32f407.machine`, where nothing should be
    // able to program flash. The permission narrows *both* children, so there
    // is no write winner left and the access is refused on terms rather than
    // reaching the controller to be dropped.
    let flash = f4();
    let space = AddressSpace::new("dmabus", 32).with_unassigned(UnassignedPolicy::FAULT);
    let region = Device::region(&flash, "array").expect("the array region");
    space
        .topology()
        .map_with_perms(region, 0x0800_0000, Perms::READ)
        .unwrap();
    assert_eq!(
        space.write(0x0800_0000, Width::U32, 0, MemAttrs::DEFAULT),
        Err(BusError::Protected)
    );
    assert_eq!(sr(&flash), 0, "the controller never saw it");
}

#[test]
fn the_array_is_fetchable() {
    let flash = f4();
    flash.load_image(0, &0x1234_5678u32.to_le_bytes()).unwrap();
    let space = AddressSpace::new("mem", 32).with_unassigned(UnassignedPolicy::FAULT);
    let region = Device::region(&flash, "array").expect("the array region");
    space.topology().map(region, 0x0800_0000).unwrap();
    let fetch = MemAttrs::DEFAULT.with_purpose(crate::core::space::AccessPurpose::FETCH);
    assert_eq!(
        space.read(0x0800_0000, Width::U32, fetch).unwrap(),
        0x1234_5678,
        "a Cortex-M fetches every instruction through this window"
    );
}

// ---------------------------------------------------------------------------
// Locking
// ---------------------------------------------------------------------------

#[test]
fn the_array_is_read_only_while_lock_is_set() {
    for flash in [f4(), l4()] {
        assert!(!flash.unlocked(), "the interface comes up locked");
        assert_eq!(word(&flash, 0x800), 0xffff_ffff);

        // A CPU `str` to 0x0800_0800 with `LOCK` set.
        flash
            .store(0x800, &0xdead_beefu32.to_le_bytes())
            .expect("the flash interface reports through SR, so a locked store is not a bus fault");
        assert_eq!(word(&flash, 0x800), 0xffff_ffff, "the value is unchanged");
        assert_eq!(
            sr(&flash) & SR_PGSERR,
            SR_PGSERR,
            "and PGSERR says the control register was not configured for it"
        );
        assert_eq!(sr(&flash) & SR_BSY, 0, "nothing was started");
    }
}

#[test]
fn the_unlock_sequence_clears_lock_and_a_wrong_key_locks_until_reset() {
    let flash = f4();
    assert!(!flash.unlocked());
    flash.poke(F4_KEYR, KEY1).unwrap();
    assert!(!flash.unlocked(), "half a sequence unlocks nothing");
    flash.poke(F4_KEYR, KEY2).unwrap();
    assert!(flash.unlocked());

    // Re-locking is a plain `CR.LOCK` write.
    flash.poke(F4_CR, CR_LOCK).unwrap();
    assert!(!flash.unlocked());

    // A wrong key locks the interface until the next reset.
    flash.poke(F4_KEYR, KEY1).unwrap();
    flash.poke(F4_KEYR, 0).unwrap();
    unlock(&flash);
    assert!(!flash.unlocked(), "the correct sequence no longer works");
    Device::reset(&flash, ResetKind::Warm);
    unlock(&flash);
    assert!(flash.unlocked(), "and a reset is what clears that");

    // An L4 reports the second attempt on the bus rather than ignoring it.
    let flash = l4();
    flash.poke(L4_KEYR, KEY1).unwrap();
    flash.poke(L4_KEYR, 0xbad).unwrap();
    assert_eq!(
        flash.poke(L4_KEYR, KEY1),
        Err(BusError::BadAccess),
        "RM0351 §3.3.5: a bus error is detected if KEYR is written again"
    );
    Device::reset(&flash, ResetKind::Warm);
    unlock(&flash);
    assert!(flash.unlocked());
}

// ---------------------------------------------------------------------------
// Programming
// ---------------------------------------------------------------------------

#[test]
fn a_double_word_program_takes_two_word_writes_and_ends_in_eop() {
    let flash = l4();
    unlock(&flash);
    flash.poke(L4_CR, CR_PG | CR_EOPIE).unwrap();
    let irq = probe(&flash, IRQ_PIN);
    assert!(!irq.is_high());

    flash.store(0x800, &0xdead_beefu32.to_le_bytes()).unwrap();
    assert_eq!(sr(&flash) & SR_BSY, 0, "the first word only latches");
    assert_eq!(word(&flash, 0x800), 0xffff_ffff);

    flash.store(0x804, &0xcafe_babeu32.to_le_bytes()).unwrap();
    assert_eq!(sr(&flash) & SR_BSY, SR_BSY, "the second word starts it");
    assert_eq!(sr(&flash) & SR_EOP, 0);
    assert_eq!(word(&flash, 0x800), 0xffff_ffff, "not yet committed");

    settle(&flash);
    assert_eq!(sr(&flash) & SR_BSY, 0);
    assert_eq!(sr(&flash) & SR_EOP, SR_EOP);
    assert_eq!(word(&flash, 0x800), 0xdead_beef);
    assert_eq!(word(&flash, 0x804), 0xcafe_babe);
    assert!(irq.is_high(), "EOPIE was set, so the line is asserted");

    clear_sr(&flash);
    assert!(!irq.is_high(), "and clearing EOP drops it again");
}

#[test]
fn programming_a_non_erased_double_word_sets_progerr_and_changes_nothing() {
    let flash = l4();
    unlock(&flash);
    flash.poke(L4_CR, CR_PG).unwrap();
    flash.store(0x800, &0xdead_beefu32.to_le_bytes()).unwrap();
    flash.store(0x804, &0xcafe_babeu32.to_le_bytes()).unwrap();
    settle(&flash);
    clear_sr(&flash);

    // The same double word again, over a location that is no longer erased.
    flash.poke(L4_CR, CR_PG).unwrap();
    flash.store(0x800, &0x0000_0000u32.to_le_bytes()).unwrap();
    flash.store(0x804, &0x0000_0000u32.to_le_bytes()).unwrap();
    assert_eq!(
        sr(&flash) & SR_PROGERR,
        SR_PROGERR,
        "RM0351 §3.7.6: PROGERR is set if the word to write is not previously erased"
    );
    assert_eq!(sr(&flash) & SR_BSY, 0, "and no operation was started");
    settle(&flash);
    assert_eq!(word(&flash, 0x800), 0xdead_beef, "nothing changed");
    assert_eq!(word(&flash, 0x804), 0xcafe_babe);
}

#[test]
fn an_l4_rejects_a_byte_write_and_a_misaligned_double_word() {
    let flash = l4();
    unlock(&flash);
    flash.poke(L4_CR, CR_PG).unwrap();

    flash.store(0x800, &[0x5a]).unwrap();
    assert_eq!(sr(&flash) & L4_SR_SIZERR, L4_SR_SIZERR);
    clear_sr(&flash);

    flash.store(0x804, &0u32.to_le_bytes()).unwrap();
    assert_eq!(
        sr(&flash) & SR_PGAERR,
        SR_PGAERR,
        "the first word must be double-word aligned"
    );
    clear_sr(&flash);

    // A correct first word followed by a word from a different double word.
    flash.store(0x800, &0u32.to_le_bytes()).unwrap();
    flash.store(0x80c, &0u32.to_le_bytes()).unwrap();
    assert_eq!(sr(&flash) & SR_PGAERR, SR_PGAERR);
    assert_eq!(word(&flash, 0x800), 0xffff_ffff);
}

#[test]
fn an_f4_programs_at_the_width_psize_selects_and_only_clears_bits() {
    let flash = f4();
    unlock(&flash);
    // `PSIZE` = 10, x32.
    flash.poke(F4_CR, CR_PG | (0b10 << 8)).unwrap();
    flash.store(0x100, &0xf0f0_f0f0u32.to_le_bytes()).unwrap();
    settle(&flash);
    assert_eq!(word(&flash, 0x100), 0xf0f0_f0f0);

    // A second program over the same word can only clear further bits
    // (RM0090 §3.6.2).
    clear_sr(&flash);
    flash.poke(F4_CR, CR_PG | (0b10 << 8)).unwrap();
    flash.store(0x100, &0xffff_00ffu32.to_le_bytes()).unwrap();
    settle(&flash);
    assert_eq!(word(&flash, 0x100), 0xf0f0_00f0);

    // A byte write while `PSIZE` says x32 is a parallelism error.
    clear_sr(&flash);
    flash.poke(F4_CR, CR_PG | (0b10 << 8)).unwrap();
    flash.store(0x200, &[0x5a]).unwrap();
    assert_eq!(sr(&flash) & F4_SR_PGPERR, F4_SR_PGPERR);
    assert_eq!(word(&flash, 0x200), 0xffff_ffff);

    // And a x32 write to an odd address is an alignment error.
    clear_sr(&flash);
    flash.poke(F4_CR, CR_PG | (0b10 << 8)).unwrap();
    flash.store(0x202, &0u32.to_le_bytes()).unwrap();
    assert_eq!(sr(&flash) & SR_PGAERR, SR_PGAERR);
}

// ---------------------------------------------------------------------------
// Erase
// ---------------------------------------------------------------------------

#[test]
fn a_page_erase_leaves_2048_bytes_of_ones_and_only_those() {
    let flash = l4();
    // Stamp the first three pages so the edges are visible.
    let stamp = alloc::vec![0x5au8; 3 * 2048];
    flash.load_image(0, &stamp).unwrap();

    unlock(&flash);
    // `PER` with `PNB` = 1: the second 2 KiB page of bank 1.
    flash.poke(L4_CR, L4_CR_PER | (1 << 3) | CR_STRT).unwrap();
    assert_eq!(sr(&flash) & SR_BSY, SR_BSY);
    settle(&flash);
    assert_eq!(sr(&flash) & SR_EOP, SR_EOP);

    let contents = flash.contents();
    assert!(
        contents[..2048].iter().all(|&b| b == 0x5a),
        "the page below is untouched"
    );
    assert!(
        contents[2048..4096].iter().all(|&b| b == 0xff),
        "exactly 2048 bytes of ones"
    );
    assert!(
        contents[4096..3 * 2048].iter().all(|&b| b == 0x5a),
        "and the page above is untouched"
    );
}

#[test]
fn a_sector_erase_costs_exactly_its_unequal_sector() {
    let flash = f4();
    flash
        .load_image(0, &alloc::vec![0x5au8; F4_SIZE as usize])
        .unwrap();
    unlock(&flash);
    // Sector 4 is the 64 KiB one at 0x0801_0000.
    flash.poke(F4_CR, F4_CR_SER | (4 << 3) | CR_STRT).unwrap();
    settle(&flash);

    let contents = flash.contents();
    assert!(contents[0..0x1_0000].iter().all(|&b| b == 0x5a));
    assert!(
        contents[0x1_0000..0x2_0000].iter().all(|&b| b == 0xff),
        "64 KiB, not 16 and not 128"
    );
    assert!(contents[0x2_0000..0x2_1000].iter().all(|&b| b == 0x5a));
}

#[test]
fn a_bank_erase_takes_the_bank_and_bker_picks_which() {
    let flash = l4();
    flash
        .load_image(0, &alloc::vec![0x5au8; L4_SIZE as usize])
        .unwrap();
    unlock(&flash);
    flash.poke(L4_CR, L4_CR_MER2 | CR_STRT).unwrap();
    settle(&flash);

    let contents = flash.contents();
    let half = (L4_SIZE / 2) as usize;
    assert!(contents[..half].iter().all(|&b| b == 0x5a), "bank 1 stands");
    assert!(
        contents[half..].iter().all(|&b| b == 0xff),
        "bank 2 is gone"
    );

    // A page in bank 2, reached through `BKER`.
    flash
        .load_image(0, &alloc::vec![0x5au8; L4_SIZE as usize])
        .unwrap();
    clear_sr(&flash);
    flash
        .poke(L4_CR, L4_CR_PER | L4_CR_BKER | (3 << 3) | CR_STRT)
        .unwrap();
    settle(&flash);
    let contents = flash.contents();
    let page = half + 3 * 2048;
    assert!(contents[page..page + 2048].iter().all(|&b| b == 0xff));
    assert!(contents[page - 1] == 0x5a && contents[page + 2048] == 0x5a);
}

#[test]
fn an_erase_with_no_operation_bit_is_a_sequence_error() {
    let flash = l4();
    unlock(&flash);
    flash.poke(L4_CR, CR_STRT).unwrap();
    assert_eq!(sr(&flash) & SR_PGSERR, SR_PGSERR);
    assert_eq!(sr(&flash) & SR_BSY, 0);
}

// ---------------------------------------------------------------------------
// Write protection
// ---------------------------------------------------------------------------

#[test]
fn a_write_protected_page_sets_wrperr() {
    let flash = l4();
    unlock(&flash);
    opt_unlock(&flash);
    // `WRP1AR`: pages 4 through 7 of bank 1, inclusive (RM0351 §3.7.12).
    flash.poke(L4_WRP1AR, 4 | (7 << 16)).unwrap();

    // An erase of page 5 is refused.
    flash.poke(L4_CR, L4_CR_PER | (5 << 3) | CR_STRT).unwrap();
    assert_eq!(sr(&flash) & SR_WRPERR, SR_WRPERR);
    assert_eq!(sr(&flash) & SR_BSY, 0);
    clear_sr(&flash);

    // And so is a program into it.
    flash.poke(L4_CR, CR_PG).unwrap();
    flash.store(5 * 2048, &0u32.to_le_bytes()).unwrap();
    assert_eq!(sr(&flash) & SR_WRPERR, SR_WRPERR);
    assert_eq!(word(&flash, 5 * 2048), 0xffff_ffff);
    clear_sr(&flash);

    // Page 8 is outside the range and programs normally.
    flash.poke(L4_CR, CR_PG).unwrap();
    flash.store(8 * 2048, &0u32.to_le_bytes()).unwrap();
    flash.store(8 * 2048 + 4, &0u32.to_le_bytes()).unwrap();
    settle(&flash);
    assert_eq!(sr(&flash) & SR_WRPERR, 0);
    assert_eq!(word(&flash, 8 * 2048), 0);
}

#[test]
fn an_f4_nwrp_zero_protects_its_sector() {
    let flash = f4();
    unlock(&flash);
    opt_unlock(&flash);
    // `nWRP` bit 16 + n is **zero** when sector n is protected
    // (RM0090 §3.9.8). Protect sector 2 and nothing else.
    let optcr = flash.peek(F4_OPTCR) & !(1 << (16 + 2));
    flash.poke(F4_OPTCR, optcr).unwrap();

    flash.poke(F4_CR, F4_CR_SER | (2 << 3) | CR_STRT).unwrap();
    assert_eq!(sr(&flash) & SR_WRPERR, SR_WRPERR);
    clear_sr(&flash);

    flash.poke(F4_CR, F4_CR_SER | (3 << 3) | CR_STRT).unwrap();
    assert_eq!(sr(&flash) & SR_WRPERR, 0, "sector 3 is not protected");
    settle(&flash);
    assert_eq!(sr(&flash) & SR_EOP, SR_EOP);
}

// ---------------------------------------------------------------------------
// Option bytes
// ---------------------------------------------------------------------------

#[test]
fn option_bytes_are_applied_by_obl_launch_through_a_reset() {
    let flash = l4();
    let reset = probe(&flash, RESET_PIN);
    /// `OPTR.nBOOT0` (RM0351 §3.4.1 bit 27).
    const NBOOT0: u32 = 1 << 27;
    let before = flash.peek(L4_OPTR);
    assert_eq!(before & NBOOT0, NBOOT0, "a factory part has nBOOT0 set");

    unlock(&flash);
    opt_unlock(&flash);
    flash.poke(L4_OPTR, before & !NBOOT0).unwrap();
    assert_eq!(flash.peek(L4_OPTR) & NBOOT0, 0, "the shadow moved");

    // `OPTSTRT` commits the shadow into the option bytes themselves.
    flash.poke(L4_CR, L4_CR_OPTSTRT).unwrap();
    assert_eq!(sr(&flash) & SR_BSY, SR_BSY);
    settle(&flash);
    assert_eq!(sr(&flash) & SR_EOP, SR_EOP);
    clear_sr(&flash);

    // `OBL_LAUNCH` reloads them and resets the device.
    assert_eq!(reset.rising_edges(), 0);
    let asked = flash.poke(L4_CR, L4_CR_OBL_LAUNCH).unwrap();
    assert!(asked, "OBL_LAUNCH generates a reset of the device");
    assert_eq!(reset.rising_edges(), 1, "and the reset line was pulsed");

    // The board wires that pulse to the core's reset input, which brings every
    // device back through `Device::reset` — including this one.
    Device::reset(&flash, ResetKind::Warm);
    assert_eq!(
        flash.peek(L4_OPTR) & NBOOT0,
        0,
        "OPTR reads the new value after the reset"
    );
    assert!(!flash.unlocked(), "and the interface is locked again");
}

/// `OBL_LAUNCH` through a real `st.rcc`: the flag the RCC has had a pin for
/// since it was written, and which nothing drove until this device existed.
///
/// A board writes `wire flash.reset -> rcc.oblrst` beside
/// `wire flash.reset -> cpu.reset`, exactly as `machines/stm32f407.machine`
/// already does for the two watchdogs — one pulse, two destinations: the
/// reboot and the reason for it.
#[cfg(feature = "dev-stm32-rcc")]
#[test]
fn obl_launch_latches_oblrstf_in_the_rcc() {
    use crate::dev::stm32::rcc::{self, Rcc};

    let flash = l4();
    let rcc = Rcc::with_config(rcc::Variant::L4, rcc::Frequencies::default(), 16);

    // The wire the machine file would build: this device drives it, the RCC
    // sinks it on the `oblrst` pin.
    let ids = WireIdAllocator::new();
    let id = ids.alloc();
    let pin = Device::sink(&rcc, "oblrst", &[id]).expect("rcc.oblrst");
    assert_eq!(pin.line, 25, "OBLRSTF is bit 25 of the L4's CSR");
    let wire = Wire::builder()
        .source(id)
        .sink(Arc::clone(&pin.sink), pin.line)
        .build_shared();
    Device::connect(&flash, RESET_PIN, WireSource::new(wire, id)).unwrap();

    // `RCC_CSR` is read the way a guest reads it, through a space.
    let space = AddressSpace::new("mem", 32).with_unassigned(UnassignedPolicy::FAULT);
    space
        .topology()
        .map(
            Device::region(&rcc, "regs").expect("the RCC registers"),
            0x4002_3800,
        )
        .unwrap();
    let csr = || {
        space
            .read(0x4002_3800 + 0x94, Width::U32, MemAttrs::DEFAULT)
            .expect("CSR")
    };
    assert_eq!(csr() & (1 << 25), 0, "nothing has reset yet");

    unlock(&flash);
    opt_unlock(&flash);
    flash.poke(L4_CR, L4_CR_OBL_LAUNCH).unwrap();
    assert_eq!(
        csr() & (1 << 25),
        1 << 25,
        "RCC_CSR.OBLRSTF says the part reset because the option bytes were reloaded"
    );
}

#[test]
fn an_uncommitted_option_write_does_not_survive_a_reset() {
    let flash = l4();
    unlock(&flash);
    opt_unlock(&flash);
    let before = flash.peek(L4_OPTR);
    flash.poke(L4_OPTR, before ^ (1 << 27)).unwrap();
    Device::reset(&flash, ResetKind::Warm);
    assert_eq!(
        flash.peek(L4_OPTR),
        before,
        "without OPTSTRT the shadow is all there ever was"
    );
}

#[test]
fn an_f4_comes_back_from_a_reset_with_its_option_register_locked() {
    // The F4 keeps `OPTLOCK` in the same register as its option bytes, and it
    // is not one of them: "Reset value: 0x0FFF AAED" has bit 0 set
    // (RM0090 §3.9.8), and the only way out is the key sequence. Committing
    // the options while unlocked used to store the *unlocked* register, so the
    // part came back from a reset able to reprogram its own protections.
    let flash = f4();
    unlock(&flash);
    opt_unlock(&flash);
    let value = flash.peek(F4_OPTCR) & !(1 << (16 + 5));
    flash.poke(F4_OPTCR, value | F4_OPTCR_OPTSTRT).unwrap();
    settle(&flash);

    Device::reset(&flash, ResetKind::Warm);
    assert_eq!(
        flash.peek(F4_OPTCR) & F4_OPTCR_OPTLOCK,
        F4_OPTCR_OPTLOCK,
        "the option register is locked out of reset"
    );
    assert_eq!(
        flash.peek(F4_OPTCR) & F4_OPTCR_OPTSTRT,
        0,
        "and OPTSTRT is a strobe, never stored"
    );
    assert_eq!(
        flash.peek(F4_OPTCR) & (1 << (16 + 5)),
        0,
        "while the option byte that was committed survived"
    );

    // And it is locked in earnest: a write is dropped until the key sequence.
    flash
        .poke(F4_OPTCR, flash.peek(F4_OPTCR) | (1 << (16 + 5)))
        .unwrap();
    assert_eq!(flash.peek(F4_OPTCR) & (1 << (16 + 5)), 0);
}

#[test]
fn the_option_registers_are_read_only_while_optlock_is_set() {
    let flash = l4();
    unlock(&flash);
    let before = flash.peek(L4_WRP1AR);
    flash.poke(L4_WRP1AR, 0x00ff_0000).unwrap();
    assert_eq!(flash.peek(L4_WRP1AR), before);
    opt_unlock(&flash);
    flash.poke(L4_WRP1AR, 0x00ff_0000).unwrap();
    assert_eq!(flash.peek(L4_WRP1AR), 0x00ff_0000);
}

// ---------------------------------------------------------------------------
// ACR and the debug seam
// ---------------------------------------------------------------------------

#[test]
fn acr_records_the_latency_and_reads_the_cache_bits_back() {
    let flash = f4();
    assert_eq!(flash.latency(), 0);
    // `LATENCY` = 5, `PRFTEN`, `ICEN`, `DCEN` — the F4 vendor startup write.
    flash
        .poke(ACR, 5 | (1 << 8) | (1 << 9) | (1 << 10))
        .unwrap();
    assert_eq!(flash.latency(), 5);
    assert_eq!(flash.peek(ACR), 5 | (1 << 8) | (1 << 9) | (1 << 10));

    // An L4 comes up with the caches already on (RM0351 §3.7.1).
    let flash = l4();
    assert_eq!(flash.peek(ACR), 0x0000_0600);
}

#[test]
fn a_debug_access_never_moves_the_controller() {
    let flash = l4();
    unlock(&flash);
    flash.poke(L4_CR, CR_PG).unwrap();
    flash.store(0x800, &0xdead_beefu32.to_le_bytes()).unwrap();
    let latched = flash.shared.state.lock().latch;
    assert!(latched.is_some());

    // A debug *write* to the array is the SWD loader's door: it pokes the
    // bytes and touches nothing else.
    let write = Program {
        shared: Arc::clone(&flash.shared),
    };
    write
        .write(0x1000, &0x1234_5678u32.to_le_bytes(), MemAttrs::DEBUG)
        .unwrap();
    assert_eq!(word(&flash, 0x1000), 0x1234_5678);
    assert_eq!(
        flash.shared.state.lock().latch,
        latched,
        "the double-word latch was not consumed"
    );
    assert_eq!(sr(&flash) & (SR_BSY | SR_EOP | L4_ERRORS), 0);

    // A debug *write* to a register is refused rather than guessed at.
    let regs = Registers {
        shared: Arc::clone(&flash.shared),
    };
    assert_eq!(
        regs.write(L4_KEYR, &KEY1.to_le_bytes(), MemAttrs::DEBUG),
        Err(BusError::BadAccess)
    );

    // A debug read is the same word an ordinary read gives.
    let mut buf = [0u8; 4];
    regs.read(cr(&flash), &mut buf, MemAttrs::DEBUG).unwrap();
    assert_eq!(u32::from_le_bytes(buf), flash.peek(L4_CR));
}

// ---------------------------------------------------------------------------
// Snapshots
// ---------------------------------------------------------------------------

/// Save `flash` into a fresh chunk and hand back the bytes.
fn save(flash: &Flash) -> alloc::vec::Vec<u8> {
    let mut shape = MachineShape::new();
    shape.add_device("flash", CLASS_NAME).unwrap();
    let mut w = StateWriter::new(shape);
    {
        let mut chunk = w.chunk("flash", CLASS_NAME, STATE_VERSION).unwrap();
        Device::save(flash, &mut chunk).unwrap();
    }
    w.to_vec().unwrap()
}

fn restore(flash: &Flash, bytes: &[u8]) {
    let reader = StateReader::new(bytes).unwrap();
    let chunk = reader
        .load("flash", CLASS_NAME, STATE_VERSION, &Migrations::new())
        .unwrap();
    Device::load(flash, &mut chunk.reader()).unwrap();
}

#[test]
fn flash_contents_survive_save_and_load() {
    let saved = l4();
    unlock(&saved);
    saved.poke(L4_CR, CR_PG | CR_EOPIE).unwrap();
    saved.store(0x800, &0xdead_beefu32.to_le_bytes()).unwrap();
    saved.store(0x804, &0xcafe_babeu32.to_le_bytes()).unwrap();
    settle(&saved);
    clear_sr(&saved);
    // Saved mid-double-word, with the first half latched and not committed:
    // the state a snapshot is most likely to drop.
    saved.poke(L4_CR, CR_PG).unwrap();
    saved.store(0x810, &0x1111_2222u32.to_le_bytes()).unwrap();

    let bytes = save(&saved);
    let restored = l4();
    restore(&restored, &bytes);

    // The identical-state assertion: a second save of the restored device is
    // byte-for-byte the first. Nothing survived that the encoding does not
    // carry, and nothing was invented on the way back in.
    assert_eq!(save(&restored), bytes, "identical state");

    assert_eq!(restored.contents(), saved.contents());
    assert_eq!(word(&restored, 0x800), 0xdead_beef);
    assert!(restored.unlocked(), "and the lock came across");

    // The latched half completes on the other side, which is the point of
    // carrying it.
    restored
        .store(0x814, &0x3333_4444u32.to_le_bytes())
        .unwrap();
    settle(&restored);
    assert_eq!(word(&restored, 0x810), 0x1111_2222);
    assert_eq!(word(&restored, 0x814), 0x3333_4444);
}

#[test]
fn a_snapshot_carries_an_operation_that_had_not_finished() {
    let saved = f4();
    unlock(&saved);
    saved.poke(F4_CR, F4_CR_SER | (1 << 3) | CR_STRT).unwrap();
    assert_eq!(sr(&saved) & SR_BSY, SR_BSY);

    let bytes = save(&saved);
    let restored = f4();
    restore(&restored, &bytes);
    assert_eq!(save(&restored), bytes, "identical state");
    assert_eq!(sr(&restored) & SR_BSY, SR_BSY);
    assert_eq!(
        Device::next_event_tick(&saved),
        Device::next_event_tick(&restored)
    );

    settle(&restored);
    assert_eq!(sr(&restored) & SR_EOP, SR_EOP);
    let contents = restored.contents();
    assert!(contents[0x4000..0x8000].iter().all(|&b| b == 0xff));
}

#[test]
fn a_snapshot_from_a_differently_sized_part_is_refused() {
    let saved = build(Variant::L4, 2048 * 4, None);
    let bytes = save(&saved);
    let other = build(Variant::L4, 2048 * 8, None);
    let reader = StateReader::new(&bytes).unwrap();
    let chunk = reader
        .load("flash", CLASS_NAME, STATE_VERSION, &Migrations::new())
        .unwrap();
    assert!(Device::load(&other, &mut chunk.reader()).is_err());
}

// ---------------------------------------------------------------------------
// Construction
// ---------------------------------------------------------------------------

#[test]
fn the_array_starts_erased_and_takes_an_image() {
    let flash = build(Variant::F4, 16 * 1024, None);
    assert!(flash.contents().iter().all(|&b| b == 0xff));

    let image: Arc<[u8]> = Arc::from(&[1u8, 2, 3, 4][..]);
    let props = Props::new()
        .with("variant", Value::Str(String::from("f4")))
        .with("size", Value::Uint(16 * 1024))
        .with(
            "image",
            Value::Media(crate::core::props::Media::new("fw", image)),
        );
    let flash = Flash::new(&props).unwrap();
    let contents = flash.contents();
    assert_eq!(&contents[..4], &[1, 2, 3, 4]);
    assert!(contents[4..].iter().all(|&b| b == 0xff));
}

#[test]
fn an_impossible_geometry_is_refused() {
    let props = Props::new()
        .with("variant", Value::Str(String::from("f4")))
        .with("size", Value::Uint(4 * BANK));
    let e = Flash::new(&props).unwrap_err().to_string();
    assert!(e.contains("2 MiB"), "{e}");

    let props = Props::new().with("size", Value::Uint(100));
    let e = Flash::new(&props).unwrap_err().to_string();
    assert!(e.contains("2 KiB"), "{e}");
}

#[test]
fn the_register_block_is_word_only() {
    let flash = f4();
    let regs = Registers {
        shared: Arc::clone(&flash.shared),
    };
    assert_eq!(
        regs.constraints(),
        AccessConstraints::word(Width::U32, Endian::Little)
    );
    assert_eq!(
        Device::region(&flash, "regs").map(|r| r.len()),
        Some(Variant::F4.register_bytes())
    );
    assert_eq!(
        Device::region(&flash, "array").map(|r| r.len()),
        Some(F4_SIZE)
    );
    assert!(Device::region(&flash, "nowhere").is_none());
}

// ---------------------------------------------------------------------------
// Protection: PCROP and RDP (RM0351 §3.5, RM0090 §3.6.3)
// ---------------------------------------------------------------------------

/// The factory `OPTR` these tests start from, dual-bank so `PCROP2` exists.
const L4_OPTR_FACTORY: u64 = 0xffef_f8aa | (1 << 21);

/// An L4 whose option bytes arm bank 1's `PCROP` over the **double words**
/// `[first, last]` — the granularity RM0351 §3.5.2 defines the area in.
fn l4_pcrop(first: u32, last: u32, pcrop_rdp: bool) -> Flash {
    let er = u64::from(last) | if pcrop_rdp { 1 << 31 } else { 0 };
    let props = Props::new()
        .with("variant", Value::Str(String::from("l4")))
        .with("size", Value::Uint(L4_SIZE))
        .with("optr", Value::Uint(L4_OPTR_FACTORY))
        .with("pcrop1sr", Value::Uint(u64::from(first)))
        .with("pcrop1er", Value::Uint(er));
    Flash::new(&props).expect("a PCROP part")
}

/// A part whose `RDP` option byte is `rdp` — `0xAA` level 0, `0xCC` level 2,
/// anything else level 1 (RM0351 Table 14).
fn with_rdp(variant: Variant, rdp: u32) -> Flash {
    let optr = if variant.is_f4() {
        u64::from((0x0fff_aaed_u32 & !(0xff << 8)) | (rdp << 8))
    } else {
        (L4_OPTR_FACTORY & !0xff) | u64::from(rdp)
    };
    let props = Props::new()
        .with("variant", Value::Str(String::from(variant.as_str())))
        .with(
            "size",
            Value::Uint(if variant.is_f4() { F4_SIZE } else { L4_SIZE }),
        )
        .with("optr", Value::Uint(optr));
    Flash::new(&props).expect("a read-protected part")
}

/// Map the array at `0x0800_0000`, the way every STM32 board does.
fn mapped(flash: &Flash) -> AddressSpace {
    let space = AddressSpace::new("mem", 32).with_unassigned(UnassignedPolicy::FAULT);
    let region = Device::region(flash, "array").expect("the array region");
    space.topology().map(region, 0x0800_0000).unwrap();
    space
}

/// The name of the child that answers reads — the whole of "does this part pay
/// for a guarded read side".
fn read_child(flash: &Flash) -> String {
    let region = Device::region(flash, "array").expect("the array region");
    let children = region.as_container().expect("a container").children();
    String::from(children[0].region.name())
}

/// What a Cortex-M's I-code fetch carries, and the D-bus does not.
fn fetch() -> MemAttrs {
    MemAttrs::DEFAULT.with_purpose(crate::core::space::AccessPurpose::FETCH)
}

#[test]
fn only_a_protected_part_pays_for_a_guarded_read_side() {
    // Nothing armed: the read side is the store, and a fetch never calls this
    // device at all.
    assert_eq!(read_child(&f4()), "flash.array");
    assert_eq!(read_child(&l4()), "flash.array");

    // Option bytes that arm either protection buy the guard …
    assert_eq!(read_child(&l4_pcrop(0x200, 0x21f, true)), "flash.guarded");
    assert_eq!(read_child(&with_rdp(Variant::L4, 0x55)), "flash.guarded");
    assert_eq!(read_child(&with_rdp(Variant::F4, 0x55)), "flash.guarded");

    // … and so does a board that says its firmware will arm them itself.
    let props = Props::new()
        .with("variant", Value::Str(String::from("l4")))
        .with("size", Value::Uint(L4_SIZE))
        .with("read-guard", Value::Bool(true));
    assert_eq!(read_child(&Flash::new(&props).unwrap()), "flash.guarded");
}

#[test]
fn an_f4_refuses_the_l4s_pcrop_properties() {
    let props = Props::new()
        .with("variant", Value::Str(String::from("f4")))
        .with("size", Value::Uint(F4_SIZE))
        .with("pcrop1sr", Value::Uint(0x200));
    let e = Flash::new(&props).unwrap_err().to_string();
    assert!(e.contains("SPRMOD"), "{e}");
}

#[test]
fn a_pcrop_area_may_be_fetched_and_not_read() {
    // Double words 0x200..=0x21f — bytes 0x1000..0x1100 of bank 1.
    let flash = l4_pcrop(0x200, 0x21f, true);
    flash
        .load_image(0x1000, &0x1234_5678u32.to_le_bytes())
        .unwrap();
    flash
        .load_image(0x1100, &0xdead_beefu32.to_le_bytes())
        .unwrap();
    let space = mapped(&flash);

    // An instruction fetch reaches the bytes: the area is execute-only, and
    // execute is the one thing it is.
    assert_eq!(
        space.read(0x0800_1000, Width::U32, fetch()).unwrap(),
        0x1234_5678
    );
    assert_eq!(sr(&flash) & L4_SR_RDERR, 0, "a fetch is not a read error");

    // A load of the same address is a D-bus read and the bytes are not
    // delivered: "Any read access performed through the D-bus to a PCROP
    // protected area will trigger RDERR flag error" (RM0351 §3.5.2).
    assert_eq!(
        space
            .read(0x0800_1000, Width::U32, MemAttrs::DEFAULT)
            .unwrap(),
        0
    );
    assert_eq!(sr(&flash) & L4_SR_RDERR, L4_SR_RDERR);
    assert_eq!(sr(&flash) & SR_OPERR, 0, "RDERR is not an operation error");
    clear_sr(&flash);

    // One double word past the end is ordinary flash.
    assert_eq!(
        space
            .read(0x0800_1100, Width::U32, MemAttrs::DEFAULT)
            .unwrap(),
        0xdead_beef
    );
    assert_eq!(sr(&flash) & L4_SR_RDERR, 0);
}

#[test]
fn a_literal_pool_load_is_refused_like_any_other_data_read() {
    // The nuance PCROP is famous for. `LDR r0, =const` inside the protected
    // range is a D-bus read *of* the range *by code in* it, and the part
    // refuses it exactly as it refuses a debugger's — the hardware keys on the
    // bus, not on the PC, which is why this model has no PC to consult and why
    // firmware for a PCROP'd part is built execute-only.
    let flash = l4_pcrop(0x200, 0x21f, true);
    flash
        .load_image(0x10f8, &0xcafe_babeu32.to_le_bytes())
        .unwrap();
    let space = mapped(&flash);

    // The code runs …
    assert_eq!(
        space.read(0x0800_1000, Width::U32, fetch()).unwrap(),
        0xffff_ffff
    );
    // … and cannot read its own constant pool at the end of the same range.
    assert_eq!(
        space
            .read(0x0800_10f8, Width::U32, MemAttrs::DEFAULT)
            .unwrap(),
        0
    );
    assert_eq!(sr(&flash) & L4_SR_RDERR, L4_SR_RDERR);
}

#[test]
fn a_pcrop_read_error_raises_the_interrupt_only_through_rderrie() {
    let flash = l4_pcrop(0x200, 0x21f, true);
    let irq = probe(&flash, IRQ_PIN);
    let space = mapped(&flash);

    // `ERRIE` alone does nothing for it: `RDERR` has its own enable
    // (RM0351 Table 16).
    unlock(&flash);
    flash.poke(L4_CR, CR_ERRIE).unwrap();
    space
        .read(0x0800_1000, Width::U32, MemAttrs::DEFAULT)
        .unwrap();
    assert!(!irq.is_high(), "ERRIE does not cover RDERR");
    clear_sr(&flash);

    flash.poke(L4_CR, L4_CR_RDERRIE).unwrap();
    space
        .read(0x0800_1000, Width::U32, MemAttrs::DEFAULT)
        .unwrap();
    assert!(irq.is_high(), "RDERRIE does");
    clear_sr(&flash);
    assert!(!irq.is_high());
}

#[test]
fn a_debug_read_of_a_pcrop_area_gets_nothing_and_moves_nothing() {
    let flash = l4_pcrop(0x200, 0x21f, true);
    flash
        .load_image(0x1000, &0x1234_5678u32.to_le_bytes())
        .unwrap();
    let space = mapped(&flash);

    // "all other accesses (DMA, debug and CPU data read, write and erase) are
    // strictly prohibited" — so a monitor sees zeros …
    assert_eq!(
        space
            .read(0x0800_1000, Width::U32, MemAttrs::DEBUG)
            .unwrap(),
        0
    );
    // … and no status bit moved, which is the debug invariant
    // (`ROADMAP.md` §15, invariant 5).
    assert_eq!(sr(&flash), 0, "a debug access never sets RDERR");
}

#[test]
fn a_pcrop_area_cannot_be_programmed_or_erased() {
    let flash = l4_pcrop(0x200, 0x21f, true);
    unlock(&flash);

    // "Any PCROP protected address is also write protected and any write
    // access to one of these addresses will trigger WRPERR" (§3.5.2).
    flash.poke(L4_CR, CR_PG).unwrap();
    flash.store(0x1000, &0u32.to_le_bytes()).unwrap();
    assert_eq!(sr(&flash) & SR_WRPERR, SR_WRPERR);
    assert_eq!(word(&flash, 0x1000), 0xffff_ffff);
    clear_sr(&flash);

    // "Any PCROP area is also erase protected" — and the protection is by
    // page, "including the page containing the start address and the end
    // address of this zone": bytes 0x1000..0x1100 are in page 2.
    flash.poke(L4_CR, L4_CR_PER | (2 << 3) | CR_STRT).unwrap();
    assert_eq!(sr(&flash) & SR_WRPERR, SR_WRPERR);
    clear_sr(&flash);

    // And "a software mass erase cannot be performed if one zone is PCROP
    // protected": the erase asks about the whole bank, which contains it.
    flash.poke(L4_CR, L4_CR_MER1 | CR_STRT).unwrap();
    assert_eq!(sr(&flash) & SR_WRPERR, SR_WRPERR);
    clear_sr(&flash);

    // Page 1 is outside the area and erases normally.
    flash.poke(L4_CR, L4_CR_PER | (1 << 3) | CR_STRT).unwrap();
    assert_eq!(sr(&flash) & SR_WRPERR, 0);
    settle(&flash);
    assert_eq!(sr(&flash) & SR_EOP, SR_EOP);
}

#[test]
fn a_pcrop_area_may_grow_but_never_shrink() {
    let flash = l4_pcrop(0x200, 0x21f, true);
    unlock(&flash);
    opt_unlock(&flash);

    // "If the user options modification tries to clear PCROP or to decrease
    // the PCROP area … PCROP area stays unchanged" (§3.5.2).
    flash.poke(L4_PCROP1SR, 0x208).unwrap();
    assert_eq!(flash.peek(L4_PCROP1SR), 0x200, "a later start is a shrink");
    flash.poke(L4_PCROP1ER, 0x210).unwrap();
    assert_eq!(
        flash.peek(L4_PCROP1ER) & 0xffff,
        0x21f,
        "an earlier end is a shrink"
    );
    // Clearing it outright — the factory "start above end" — is the same
    // request, and is refused the same way.
    flash.poke(L4_PCROP1SR, 0xffff).unwrap();
    assert_eq!(flash.peek(L4_PCROP1SR), 0x200);

    // "On the contrary, it is possible to increase the PCROP area."
    flash.poke(L4_PCROP1SR, 0x100).unwrap();
    assert_eq!(flash.peek(L4_PCROP1SR), 0x100);
    flash.poke(L4_PCROP1ER, 0x2ff).unwrap();
    assert_eq!(flash.peek(L4_PCROP1ER) & 0xffff, 0x2ff);
    assert_eq!(
        flash.peek(L4_PCROP1ER) & L4_PCROP_RDP,
        L4_PCROP_RDP,
        "PCROP_RDP is set-only, and a write that omitted it did not clear it"
    );
}

#[test]
fn rdp_level_1_shuts_the_debugger_out_of_the_array() {
    for variant in [Variant::F4, Variant::L4] {
        let flash = with_rdp(variant, 0x55);
        flash.load_image(0, &0x1234_5678u32.to_le_bytes()).unwrap();
        let space = mapped(&flash);

        // "In debug mode … a read or write access to the Flash generates a bus
        // error" (RM0351 §3.5.1; RM0090 §3.6.3).
        assert_eq!(
            space.read(0x0800_0000, Width::U32, MemAttrs::DEBUG),
            Err(BusError::Protected)
        );
        assert_eq!(
            space.write(0x0800_0000, Width::U32, 0, MemAttrs::DEBUG),
            Err(BusError::Protected),
            "the loader's door is exactly the door read protection closes"
        );

        // The guest's own code is unaffected: "Code executing in user mode
        // (Boot Flash) can access Flash main memory … with all operations."
        assert_eq!(
            space
                .read(0x0800_0000, Width::U32, MemAttrs::DEFAULT)
                .unwrap(),
            0x1234_5678
        );
        assert_eq!(
            space.read(0x0800_0000, Width::U32, fetch()).unwrap(),
            0x1234_5678
        );
    }

    // Level 0 is the part everybody ships: the debugger reads it.
    let flash = with_rdp(Variant::L4, 0xaa);
    flash.load_image(0, &0x1234_5678u32.to_le_bytes()).unwrap();
    let space = mapped(&flash);
    assert_eq!(
        space
            .read(0x0800_0000, Width::U32, MemAttrs::DEBUG)
            .unwrap(),
        0x1234_5678
    );
}

#[test]
fn rdp_level_1_refuses_to_program_while_a_debugger_has_the_core() {
    // The half of "debug mode" that no access attribute carries: an ordinary
    // guest store, made while something has the core stopped (RM0351 §3.5.3).
    let flash = with_rdp(Variant::L4, 0x55);
    unlock(&flash);

    Device::debug_halt(&flash, true);
    flash.poke(L4_CR, CR_PG).unwrap();
    flash.store(0x800, &0u32.to_le_bytes()).unwrap();
    assert_eq!(sr(&flash) & SR_WRPERR, SR_WRPERR);
    assert_eq!(word(&flash, 0x800), 0xffff_ffff);
    clear_sr(&flash);

    flash.poke(L4_CR, L4_CR_PER | (1 << 3) | CR_STRT).unwrap();
    assert_eq!(sr(&flash) & SR_WRPERR, SR_WRPERR, "nor an erase");
    clear_sr(&flash);

    // The debugger lets go, and the same sequence programs.
    Device::debug_halt(&flash, false);
    flash.poke(L4_CR, CR_PG).unwrap();
    flash.store(0x800, &0u32.to_le_bytes()).unwrap();
    flash.store(0x804, &0u32.to_le_bytes()).unwrap();
    settle(&flash);
    assert_eq!(sr(&flash) & SR_WRPERR, 0);
    assert_eq!(word(&flash, 0x800), 0);

    // At level 0 a debugger may sit there all day.
    let flash = l4();
    unlock(&flash);
    Device::debug_halt(&flash, true);
    flash.poke(L4_CR, CR_PG).unwrap();
    flash.store(0x800, &0u32.to_le_bytes()).unwrap();
    flash.store(0x804, &0u32.to_le_bytes()).unwrap();
    settle(&flash);
    assert_eq!(sr(&flash) & SR_WRPERR, 0);
    assert_eq!(word(&flash, 0x800), 0);
}

#[test]
fn dropping_rdp_from_level_1_to_level_0_mass_erases_the_array() {
    for variant in [Variant::F4, Variant::L4] {
        let flash = with_rdp(variant, 0x55);
        flash
            .load_image(0x2000, &0x1234_5678u32.to_le_bytes())
            .unwrap();
        let optr = if variant.is_f4() { F4_OPTCR } else { L4_OPTR };
        unlock(&flash);
        opt_unlock(&flash);

        // Another option byte, *committed*, to prove the rest survives the
        // erase: "The other option bytes including write protections remain
        // unchanged from before the mass-erase operation" (RM0090 §3.6.3), and
        // "the user options except PCROP protection are set to their previous
        // values" (RM0351 §3.5.1) — previous meaning as programmed, so an
        // uncommitted shadow write is not what survives.
        if variant.is_f4() {
            let value = flash.peek(F4_OPTCR) & !(1 << (16 + 3));
            flash.poke(F4_OPTCR, value | F4_OPTCR_OPTSTRT).unwrap();
        } else {
            flash.poke(L4_WRP1AR, 4 | (7 << 16)).unwrap();
            flash.poke(L4_CR, L4_CR_OPTSTRT).unwrap();
        }
        settle(&flash);
        clear_sr(&flash);

        // "programming the protection option byte (RDP) to Level 0 causes the
        // Flash memory … to be mass-erased" (RM0090 §3.6.3, RM0351 §3.5.1).
        let level0 = if variant.is_f4() {
            (flash.peek(F4_OPTCR) & !(0xff << 8)) | (0xaa << 8)
        } else {
            (flash.peek(L4_OPTR) & !0xff) | 0xaa
        };
        flash.poke(optr, level0).unwrap();
        if variant.is_f4() {
            flash.poke(F4_OPTCR, level0 | F4_OPTCR_OPTSTRT).unwrap();
        } else {
            flash.poke(L4_CR, L4_CR_OPTSTRT).unwrap();
        }
        assert_eq!(sr(&flash) & SR_BSY, SR_BSY, "the erase takes time");
        settle(&flash);
        assert_eq!(sr(&flash) & SR_EOP, SR_EOP);

        assert!(
            flash.contents().iter().all(|&b| b == 0xff),
            "the user code area is cleared before the protection is removed"
        );
        Device::reset(&flash, ResetKind::Warm);
        let (mask, want) = if variant.is_f4() {
            (0xff00, 0xaa00)
        } else {
            (0xff, 0xaa)
        };
        assert_eq!(
            flash.peek(optr) & mask,
            want,
            "and the part is at level 0 afterwards"
        );
        if variant.is_f4() {
            assert_eq!(
                flash.peek(F4_OPTCR) & (1 << (16 + 3)),
                0,
                "the other option bytes remain unchanged"
            );
        } else {
            assert_eq!(flash.peek(L4_WRP1AR), 4 | (7 << 16));
        }

        // A debugger may read it now.
        let space = mapped(&flash);
        assert_eq!(
            space
                .read(0x0800_0000, Width::U32, MemAttrs::DEBUG)
                .unwrap(),
            0xffff_ffff
        );
    }
}

#[test]
fn a_cleared_pcrop_rdp_turns_the_mass_erase_into_a_partial_one() {
    // "If the bit PCROP_RDP is cleared … the full mass erase is replaced by a
    // partial mass erase that is successive page erases … except for the pages
    // protected by PCROP" (RM0351 §3.5.1).
    let flash = l4_pcrop(0x200, 0x21f, false);
    unlock(&flash);
    opt_unlock(&flash);
    // Arm level 1 the way a part in the field is: through the option bytes.
    flash
        .poke(L4_OPTR, (flash.peek(L4_OPTR) & !0xff) | 0x55)
        .unwrap();
    flash.poke(L4_CR, L4_CR_OPTSTRT).unwrap();
    settle(&flash);
    clear_sr(&flash);

    flash
        .load_image(0x1000, &0xaaaa_aaaau32.to_le_bytes())
        .unwrap();
    flash
        .load_image(0x4000, &0xbbbb_bbbbu32.to_le_bytes())
        .unwrap();

    flash
        .poke(L4_OPTR, (flash.peek(L4_OPTR) & !0xff) | 0xaa)
        .unwrap();
    flash.poke(L4_CR, L4_CR_OPTSTRT).unwrap();
    settle(&flash);

    assert_eq!(
        word(&flash, 0x1000),
        0xaaaa_aaaa,
        "the PCROP page is kept, which is what PCROP_RDP is for"
    );
    assert_eq!(word(&flash, 0x4000), 0xffff_ffff, "everything else went");
    assert_eq!(
        flash.peek(L4_PCROP1SR),
        0x200,
        "and PCROP itself survives a partial erase"
    );
}

#[test]
fn a_full_regression_erase_disables_pcrop() {
    let flash = l4_pcrop(0x200, 0x21f, true);
    unlock(&flash);
    opt_unlock(&flash);
    flash
        .poke(L4_OPTR, (flash.peek(L4_OPTR) & !0xff) | 0x55)
        .unwrap();
    flash.poke(L4_CR, L4_CR_OPTSTRT).unwrap();
    settle(&flash);
    clear_sr(&flash);

    flash
        .poke(L4_OPTR, (flash.peek(L4_OPTR) & !0xff) | 0xaa)
        .unwrap();
    flash.poke(L4_CR, L4_CR_OPTSTRT).unwrap();
    settle(&flash);

    // "Deactivation of PCROP can only occur when the RDP is changing from
    // level 1 to level 0", and `PCROP_RDP` "is reset after a full mass erase"
    // (§3.5.2, §3.7.10).
    assert_eq!(flash.peek(L4_PCROP1SR), 0xffff);
    assert_eq!(flash.peek(L4_PCROP1ER), 0);
    let space = mapped(&flash);
    assert_eq!(
        space
            .read(0x0800_1000, Width::U32, MemAttrs::DEFAULT)
            .unwrap(),
        0xffff_ffff,
        "the area reads as ordinary erased flash again"
    );
}

#[test]
fn read_protection_level_2_is_permanent() {
    let flash = l4();
    unlock(&flash);
    opt_unlock(&flash);

    // 0 → 2 directly, which erases nothing: "When the protection level is
    // increased (0->1, 1->2, 0->2) there is no mass erase."
    flash
        .load_image(0x2000, &0x1234_5678u32.to_le_bytes())
        .unwrap();
    flash
        .poke(L4_OPTR, (flash.peek(L4_OPTR) & !0xff) | 0xcc)
        .unwrap();
    flash.poke(L4_CR, L4_CR_OPTSTRT).unwrap();
    settle(&flash);
    assert_eq!(word(&flash, 0x2000), 0x1234_5678);
    clear_sr(&flash);

    // "Thus, the level 2 cannot be removed at all: it is an irreversible
    // operation. When attempting to modify the options bytes, the protection
    // error flag WRPERR is set" (RM0351 §3.5.1).
    flash
        .poke(L4_OPTR, (flash.peek(L4_OPTR) & !0xff) | 0xaa)
        .unwrap();
    assert_eq!(sr(&flash) & SR_WRPERR, SR_WRPERR);
    assert_eq!(flash.peek(L4_OPTR) & 0xff, 0xcc, "the shadow did not move");
    clear_sr(&flash);

    flash.poke(L4_CR, L4_CR_OPTSTRT).unwrap();
    assert_eq!(sr(&flash) & SR_WRPERR, SR_WRPERR);
    assert_eq!(sr(&flash) & SR_BSY, 0, "and nothing was started");
    clear_sr(&flash);

    Device::reset(&flash, ResetKind::Cold);
    assert_eq!(flash.peek(L4_OPTR) & 0xff, 0xcc, "a reset does not help");

    // An F4 refuses the same write silently: RM0090 §3.6.3 says the option
    // bytes "can no longer be changed" and names no flag for the attempt.
    let flash = with_rdp(Variant::F4, 0xcc);
    unlock(&flash);
    opt_unlock(&flash);
    let before = flash.peek(F4_OPTCR);
    flash
        .poke(F4_OPTCR, (before & !(0xff << 8)) | (0xaa << 8))
        .unwrap();
    assert_eq!(flash.peek(F4_OPTCR), before);
    assert_eq!(sr(&flash), 0);
}

/// The geometry a loader is told about, which has to be the part's own: GDB
/// aligns every `vFlashErase` to the blocksize declared here, so a single
/// blocksize over unequal sectors would have it erase ranges the silicon
/// cannot.
#[test]
fn the_published_erase_geometry_is_the_part_s_own() {
    let layout = Program {
        shared: Arc::clone(&f4().shared),
    }
    .flash_layout()
    .expect("an F4 array is programmable by a loader");
    assert_eq!(layout.erased, 0xff, "RM0090 §3.6.3: an erased cell is ones");
    // RM0090 Table 5: four of 16 KiB, one of 64 KiB, seven of 128 KiB.
    assert_eq!(
        layout.blocks,
        alloc::vec![
            EraseBlocks {
                offset: 0,
                length: 4 * 16 * 1024,
                blocksize: 16 * 1024,
            },
            EraseBlocks {
                offset: 64 * 1024,
                length: 64 * 1024,
                blocksize: 64 * 1024,
            },
            EraseBlocks {
                offset: 128 * 1024,
                length: 7 * 128 * 1024,
                blocksize: 128 * 1024,
            },
        ]
    );
    // The whole megabyte is covered, with no gap and no overlap.
    let covered: u64 = layout.blocks.iter().map(|b| b.length).sum();
    assert_eq!(covered, F4_SIZE);

    // The L4's pages are uniform (RM0351 §3.2), both banks alike, so it is one
    // run whatever `DUALBANK` says.
    let l4 = l4();
    let page = l4.shared.page;
    let layout = Program {
        shared: Arc::clone(&l4.shared),
    }
    .flash_layout()
    .expect("an L4 array is programmable too");
    assert_eq!(
        layout.blocks,
        alloc::vec![EraseBlocks {
            offset: 0,
            length: L4_SIZE,
            blocksize: page,
        }]
    );
}

// ---------------------------------------------------------------------------
// The U5 (RM0456 §7; offsets and bits from ST CMSIS `stm32u585xx.h`)
// ---------------------------------------------------------------------------

/// A 2 MiB STM32U585: two banks of 1 MiB, 8 KiB pages.
const U5_SIZE: u64 = 2 * 1024 * 1024;

fn u5() -> Flash {
    build(Variant::U5, U5_SIZE, None)
}

/// Store one quad-word as HAL's `FLASH_Program_QuadWord` does: four word
/// writes, lowest address first.
fn u5_quad(flash: &Flash, offset: u64, words: [u32; 4]) {
    for (i, w) in words.iter().enumerate() {
        flash
            .store(offset + 4 * i as u64, &w.to_le_bytes())
            .unwrap();
    }
}

#[test]
fn a_u5_comes_out_of_reset_locked_with_tzen_clear_and_rdp_level_0() {
    let flash = u5();
    assert_eq!(flash.peek(U5_NSCR), 0xc000_0000, "LOCK and OPTLOCK");
    assert_eq!(flash.peek(ACR), 0);
    assert_eq!(flash.peek(U5_NSSR), 0);
    let optr = flash.peek(U5_OPTR);
    assert_eq!(optr & 0xff, 0xaa, "RDP level 0");
    assert_eq!(optr & U5_OPTR_TZEN, 0, "TrustZone off");
    assert_ne!(optr & (1 << 21), 0, "DUALBANK");
    assert_eq!(flash.peek(U5_NSBOOTADD0R) & U5_BOOTADD_MASK, 0x0800_0000);
    assert_eq!(
        Device::region(&flash, "regs").map(|r| r.len()),
        Some(0x100),
        "FLASH_TypeDef runs to PRIVBB2R4 at +0xFC"
    );
    // A board cannot ask for TrustZone, and a U5 has no PCROP.
    let props = Props::new()
        .with("variant", Value::Str(String::from("u5")))
        .with("size", Value::Uint(U5_SIZE))
        .with("optr", Value::Uint(0x9fef_f8aa));
    assert!(Flash::new(&props).is_err());
    let props = Props::new()
        .with("variant", Value::Str(String::from("u5")))
        .with("size", Value::Uint(U5_SIZE))
        .with("pcrop1sr", Value::Uint(0));
    assert!(Flash::new(&props).is_err());
    // 4 MiB would be 256 pages a bank, past `PNB[6:0]`.
    let props = Props::new()
        .with("variant", Value::Str(String::from("u5")))
        .with("size", Value::Uint(2 * U5_SIZE));
    assert!(Flash::new(&props).is_err());
}

#[test]
fn the_u5_key_sequences_unlock_and_a_wrong_key_is_a_bus_error_until_reset() {
    let flash = u5();
    unlock(&flash);
    assert!(flash.unlocked());
    assert_eq!(flash.peek(U5_NSKEYR), 0, "the key register is write-only");
    opt_unlock(&flash);
    assert_eq!(flash.peek(U5_NSCR) & L4_CR_OPTLOCK, 0);
    // Setting OPTLOCK leaves LOCK alone on a U5.
    flash.poke(U5_NSCR, L4_CR_OPTLOCK).unwrap();
    assert!(flash.unlocked());

    let flash = u5();
    flash.poke(U5_NSKEYR, 0x1234_5678).unwrap();
    assert_eq!(flash.poke(U5_NSKEYR, KEY1), Err(BusError::BadAccess));
    Device::reset(&flash, ResetKind::Warm);
    unlock(&flash);
    assert!(flash.unlocked(), "a reset clears the key error");
}

#[test]
fn a_u5_quad_word_is_four_words_and_wdw_says_the_buffer_is_waiting() {
    let flash = u5();
    let irq = probe(&flash, IRQ_PIN);
    unlock(&flash);
    flash.poke(U5_NSCR, CR_PG | CR_EOPIE).unwrap();
    let words: [u32; 4] = [0x1111_1111, 0x2222_2222, 0x3333_3333, 0x4444_4444];
    for (i, w) in words.iter().enumerate().take(3) {
        flash.store(0x100 + 4 * i as u64, &w.to_le_bytes()).unwrap();
        assert_eq!(
            sr(&flash),
            U5_SR_WDW,
            "waiting for the rest of the quad-word"
        );
    }
    assert_eq!(
        word(&flash, 0x100),
        0xffff_ffff,
        "nothing is programmed yet"
    );
    flash.store(0x10c, &words[3].to_le_bytes()).unwrap();
    assert_eq!(sr(&flash), SR_BSY, "the fourth word starts the program");
    settle(&flash);
    assert_eq!(sr(&flash), SR_EOP);
    assert!(irq.is_high(), "EOPIE");
    for (i, w) in words.iter().enumerate() {
        assert_eq!(word(&flash, 0x100 + 4 * i as u64), *w);
    }
    clear_sr(&flash);
    assert!(!irq.is_high());

    // The words may arrive in any order within the quad-word, and without
    // EOPIE no EOP is set at all.
    flash.poke(U5_NSCR, CR_PG).unwrap();
    for i in [3u64, 1, 0, 2] {
        flash
            .store(0x200 + 4 * i, &(i as u32).to_le_bytes())
            .unwrap();
    }
    settle(&flash);
    assert_eq!(sr(&flash), 0, "EOP needs EOPIE on a U5");
    assert_eq!(word(&flash, 0x20c), 3);
}

#[test]
fn a_u5_refuses_a_programmed_quad_word_but_allows_zeroing_it() {
    let flash = u5();
    unlock(&flash);
    flash.poke(U5_NSCR, CR_PG).unwrap();
    u5_quad(&flash, 0x40, [0x0f0f_0f0f; 4]);
    settle(&flash);
    u5_quad(&flash, 0x40, [0x0000_0f0f; 4]);
    assert_eq!(sr(&flash) & SR_PROGERR, SR_PROGERR);
    assert_eq!(sr(&flash) & SR_OPERR, 0, "OPERR only with ERRIE");
    assert_eq!(word(&flash, 0x40), 0x0f0f_0f0f, "nothing was programmed");

    // An uncleared program error makes the next program a sequence error.
    u5_quad(&flash, 0x80, [0; 4]);
    assert_eq!(sr(&flash) & SR_PGSERR, SR_PGSERR);
    assert_eq!(word(&flash, 0x80), 0xffff_ffff);
    clear_sr(&flash);

    // All zeroes is allowed over programmed cells.
    u5_quad(&flash, 0x40, [0; 4]);
    settle(&flash);
    assert_eq!(sr(&flash) & U5_ERRORS, 0);
    assert_eq!(word(&flash, 0x40), 0);
}

#[test]
fn a_u5_rejects_narrow_stores_stray_quad_words_and_a_clear_pg() {
    let flash = u5();
    let irq = probe(&flash, IRQ_PIN);
    unlock(&flash);
    flash.poke(U5_NSCR, CR_PG | CR_ERRIE).unwrap();
    flash.store(0x0, &[0u8; 2]).unwrap();
    assert_eq!(sr(&flash), L4_SR_SIZERR | SR_OPERR);
    assert!(irq.is_high(), "ERRIE");
    clear_sr(&flash);

    // A word into another quad-word while one is half-filled.
    flash.store(0x10, &0u32.to_le_bytes()).unwrap();
    flash.store(0x24, &0u32.to_le_bytes()).unwrap();
    assert_eq!(sr(&flash), SR_PGAERR | SR_OPERR, "and WDW is gone");
    clear_sr(&flash);
    assert_eq!(word(&flash, 0x10), 0xffff_ffff);

    flash.poke(U5_NSCR, 0).unwrap();
    flash.store(0x30, &0u32.to_le_bytes()).unwrap();
    assert_eq!(sr(&flash) & SR_PGSERR, SR_PGSERR, "PG clear");
}

#[test]
fn a_u5_burst_programs_eight_quad_words_at_once() {
    let flash = u5();
    unlock(&flash);
    flash.poke(U5_NSCR, CR_PG | U5_CR_BWR).unwrap();
    for i in 0..32u32 {
        flash
            .store(0x400 + 4 * u64::from(i), &i.to_le_bytes())
            .unwrap();
        if i < 31 {
            assert_eq!(sr(&flash), U5_SR_WDW);
        }
    }
    assert_eq!(sr(&flash), SR_BSY);
    settle(&flash);
    for i in 0..32u32 {
        assert_eq!(word(&flash, 0x400 + 4 * u64::from(i)), i);
    }
}

#[test]
fn a_u5_page_erase_takes_8_kib_of_the_bank_bker_names() {
    let flash = u5();
    flash
        .load_image(0, &alloc::vec![0u8; U5_SIZE as usize])
        .unwrap();
    unlock(&flash);
    flash
        .poke(U5_NSCR, L4_CR_PER | L4_CR_BKER | (3 << 3) | CR_STRT)
        .unwrap();
    assert_eq!(sr(&flash) & SR_BSY, SR_BSY);
    settle(&flash);
    let contents = flash.contents();
    let page = (U5_SIZE / 2 + 3 * U5_PAGE) as usize;
    assert!(contents[page..page + 8192].iter().all(|&b| b == 0xff));
    assert_eq!(contents[page - 1], 0);
    assert_eq!(contents[page + 8192], 0);
    assert!(contents[..U5_SIZE as usize / 2].iter().all(|&b| b == 0));

    // A 1 MiB U575 puts bank 2 at 512 KiB.
    let small = build(Variant::U5, U5_SIZE / 2, None);
    small
        .load_image(0, &alloc::vec![0u8; (U5_SIZE / 2) as usize])
        .unwrap();
    unlock(&small);
    small
        .poke(U5_NSCR, L4_CR_PER | L4_CR_BKER | CR_STRT)
        .unwrap();
    settle(&small);
    let contents = small.contents();
    assert!(
        contents[512 * 1024..512 * 1024 + 8192]
            .iter()
            .all(|&b| b == 0xff)
    );
    assert_eq!(contents[0], 0);
    // Page 64 is past a 512 KiB bank.
    small
        .poke(U5_NSCR, L4_CR_PER | (64 << 3) | CR_STRT)
        .unwrap();
    assert_eq!(sr(&small) & SR_PGSERR, SR_PGSERR);
}

#[test]
fn hal_s_back_to_back_bank_erases_erase_the_whole_u5() {
    let flash = u5();
    flash
        .load_image(0, &alloc::vec![0u8; U5_SIZE as usize])
        .unwrap();
    unlock(&flash);
    // `FLASH_MassErase(FLASH_BANK_BOTH)`: two read-modify-writes, no wait.
    flash.poke(U5_NSCR, L4_CR_MER1 | CR_STRT).unwrap();
    let cr = flash.peek(U5_NSCR);
    flash.poke(U5_NSCR, cr | L4_CR_MER2 | CR_STRT).unwrap();
    assert_eq!(sr(&flash) & U5_ERRORS, 0, "not a sequence error");
    settle(&flash);
    assert!(flash.contents().iter().all(|&b| b == 0xff));

    // One bank alone leaves the other.
    let flash = u5();
    flash
        .load_image(0, &alloc::vec![0u8; U5_SIZE as usize])
        .unwrap();
    unlock(&flash);
    flash.poke(U5_NSCR, L4_CR_MER2 | CR_STRT).unwrap();
    settle(&flash);
    let contents = flash.contents();
    assert!(contents[..U5_SIZE as usize / 2].iter().all(|&b| b == 0));
    assert!(contents[U5_SIZE as usize / 2..].iter().all(|&b| b == 0xff));
}

#[test]
fn a_u5_wrp_area_refuses_program_and_erase() {
    let flash = u5();
    unlock(&flash);
    opt_unlock(&flash);
    // Bank 2, pages 2..=3.
    flash.poke(U5_WRP2AR, 0x8003_0002).unwrap();
    assert_eq!(flash.peek(U5_WRP2AR), 0x8003_0002);
    flash.poke(U5_NSCR, CR_PG).unwrap();
    u5_quad(&flash, U5_SIZE / 2 + 2 * U5_PAGE, [0; 4]);
    assert_eq!(sr(&flash) & SR_WRPERR, SR_WRPERR);
    clear_sr(&flash);
    flash.poke(U5_NSCR, 0).unwrap();
    flash.poke(U5_NSCR, L4_CR_MER2 | CR_STRT).unwrap();
    assert_eq!(sr(&flash) & SR_WRPERR, SR_WRPERR);
    clear_sr(&flash);
    // The same page in bank 1 is not protected.
    flash.poke(U5_NSCR, CR_PG).unwrap();
    u5_quad(&flash, 2 * U5_PAGE, [0; 4]);
    settle(&flash);
    assert_eq!(sr(&flash) & U5_ERRORS, 0);
}

#[test]
fn u5_option_bytes_commit_with_optstrt_and_load_with_obl_launch() {
    /// `OPTR.nBOOT0` (`FLASH_OPTR_nBOOT0_Pos` = 27).
    const NBOOT0: u32 = 1 << 27;
    let flash = u5();
    let reset = probe(&flash, RESET_PIN);
    // Locked: the write does nothing.
    flash.poke(U5_OPTR, 0).unwrap();
    assert_eq!(flash.peek(U5_OPTR), 0x1fef_f8aa);

    unlock(&flash);
    opt_unlock(&flash);
    let before = flash.peek(U5_OPTR);
    // TZEN cannot be set; everything else defined can.
    flash
        .poke(U5_OPTR, (before & !NBOOT0) | U5_OPTR_TZEN)
        .unwrap();
    assert_eq!(flash.peek(U5_OPTR), before & !NBOOT0);
    flash.poke(U5_NSBOOTADD0R, 0x0810_0000).unwrap();
    assert_eq!(
        flash.peek(U5_NSBOOTADD0R),
        0x0810_007f,
        "reserved bits kept"
    );

    flash.poke(U5_NSCR, L4_CR_OPTSTRT | CR_EOPIE).unwrap();
    assert_eq!(sr(&flash) & SR_BSY, SR_BSY);
    settle(&flash);
    assert_eq!(sr(&flash) & SR_EOP, SR_EOP);

    assert!(flash.poke(U5_NSCR, L4_CR_OBL_LAUNCH).unwrap());
    assert_eq!(reset.rising_edges(), 1);
    Device::reset(&flash, ResetKind::Warm);
    assert_eq!(flash.peek(U5_OPTR) & NBOOT0, 0, "committed");
    assert_eq!(flash.peek(U5_NSBOOTADD0R), 0x0810_007f);
    assert_eq!(flash.peek(U5_NSCR), 0xc000_0000);
}

#[test]
fn the_u5_secure_registers_are_raz_wi_on_a_tzen_0_part() {
    let flash = u5();
    unlock(&flash);
    for offset in [U5_SECKEYR, 0x24, 0x2c, 0x80, 0xa0, 0xc0] {
        flash.poke(offset, 0xffff_ffff).unwrap();
        assert_eq!(flash.peek(offset), 0, "{offset:#x}");
    }
    flash.poke(U5_PRIVCFGR, 0xffff_ffff).unwrap();
    assert_eq!(flash.peek(U5_PRIVCFGR), 0b11);
    flash.poke(ACR, 0xffff_ffff).unwrap();
    assert_eq!(flash.peek(ACR), U5_ACR_MASK, "PDREQx are not kept");
    assert_eq!(flash.latency(), 15);
}

#[test]
fn a_u5_debug_access_never_moves_the_write_buffer() {
    let flash = u5();
    unlock(&flash);
    flash.poke(U5_NSCR, CR_PG).unwrap();
    flash.store(0x0, &0u32.to_le_bytes()).unwrap();
    let before = sr(&flash);
    // A debugger poking the array goes through the loader's door.
    Program {
        shared: Arc::clone(&flash.shared),
    }
    .write(0x1000, &0u32.to_le_bytes(), MemAttrs::DEBUG)
    .unwrap();
    assert_eq!(word(&flash, 0x1000), 0);
    let mut buf = [0u8; 4];
    Registers {
        shared: Arc::clone(&flash.shared),
    }
    .read(U5_NSSR, &mut buf, MemAttrs::DEBUG)
    .unwrap();
    assert_eq!(u32::from_le_bytes(buf), before);
    assert_eq!(sr(&flash), U5_SR_WDW, "the half-filled quad-word is intact");
    // And it still completes.
    for i in 1..4u64 {
        flash.store(4 * i, &0u32.to_le_bytes()).unwrap();
    }
    assert_eq!(sr(&flash), SR_BSY);
}

#[test]
fn a_u5_snapshot_round_trips_a_half_filled_quad_word() {
    let saved = u5();
    unlock(&saved);
    saved.poke(U5_NSCR, CR_PG | CR_EOPIE).unwrap();
    u5_quad(&saved, 0x2000, [1, 2, 3, 4]);
    settle(&saved);
    clear_sr(&saved);
    saved.store(0x3000, &0xaaaa_5555u32.to_le_bytes()).unwrap();
    saved.store(0x3008, &0x1234_5678u32.to_le_bytes()).unwrap();

    let bytes = save(&saved);
    let restored = u5();
    restore(&restored, &bytes);
    assert_eq!(save(&restored), bytes, "identical state");
    assert_eq!(word(&restored, 0x2004), 2);
    assert_eq!(sr(&restored), U5_SR_WDW);

    restored.store(0x3004, &7u32.to_le_bytes()).unwrap();
    restored.store(0x300c, &9u32.to_le_bytes()).unwrap();
    settle(&restored);
    assert_eq!(word(&restored, 0x3000), 0xaaaa_5555);
    assert_eq!(word(&restored, 0x3008), 0x1234_5678);
    assert_eq!(word(&restored, 0x300c), 9);

    // And a running burst survives too, all 128 bytes of it.
    let saved = u5();
    unlock(&saved);
    saved.poke(U5_NSCR, CR_PG | U5_CR_BWR).unwrap();
    for i in 0..32u32 {
        saved
            .store(0x4000 + 4 * u64::from(i), &(!i).to_le_bytes())
            .unwrap();
    }
    let bytes = save(&saved);
    let restored = u5();
    restore(&restored, &bytes);
    assert_eq!(save(&restored), bytes);
    settle(&restored);
    assert_eq!(word(&restored, 0x407c), !31u32);
}

// ---------------------------------------------------------------------------
// The WB (RM0434 §3; offsets and bits from ST CMSIS `stm32wb55xx.h`)
// ---------------------------------------------------------------------------

/// A 1 MiB STM32WB55xG: 256 pages of 4 KiB.
const WB_SIZE: u64 = 1024 * 1024;

/// A WB55 with the default option bytes: the full BLE stack's secure area
/// from page 0xD0.
fn wb() -> Flash {
    build(Variant::Wb, WB_SIZE, None)
}

/// A WB with no secure area at all (`SFR.FSD` set), for the tests that are
/// about the program engine rather than about CPU2.
fn wb_open() -> Flash {
    let props = Props::new()
        .with("variant", Value::Str(String::from("wb")))
        .with("size", Value::Uint(WB_SIZE))
        .with("sfr", Value::Uint(u64::from(WB_SFR_FSD)));
    Flash::new(&props).expect("a WB flash")
}

/// Map the array at its address and read a word through it.
fn wb_read(flash: &Flash, offset: u64, attrs: MemAttrs) -> MemResult<u64> {
    let space = AddressSpace::new("mem", 32).with_unassigned(UnassignedPolicy::FAULT);
    let region = Device::region(flash, "array").expect("the array region");
    space.topology().map(region, 0x0800_0000).unwrap();
    space.read(0x0800_0000 + offset, Width::U32, attrs)
}

/// Program one double word as HAL's `FLASH_Program_DoubleWord` does.
fn wb_double(flash: &Flash, offset: u64, low: u32, high: u32) -> MemResult {
    flash.store(offset, &low.to_le_bytes())?;
    flash.store(offset + 4, &high.to_le_bytes())
}

#[test]
fn a_wb_comes_out_of_reset_as_rm0434_and_the_header_say() {
    let flash = wb();
    assert_eq!(flash.variant(), Variant::Wb);
    assert_eq!(
        Device::region(&flash, "regs").map(|r| r.len()),
        Some(0x88),
        "FLASH_TypeDef runs to SRRVR at +0x84"
    );
    assert_eq!(flash.peek(L4_CR), 0xc000_0000, "LOCK and OPTLOCK");
    assert_eq!(
        flash.peek(ACR),
        0x0000_0600 | WB_ACR_EMPTY,
        "caches on, and an empty part says so"
    );
    assert_eq!(flash.peek(WB_C2ACR), 0x0000_0600);
    let optr = flash.peek(L4_OPTR);
    assert_eq!(optr & 0xff, 0xaa, "RDP level 0");
    assert_ne!(optr & WB_OPTR_ESE, 0, "security enabled: FSD is clear");
    // SFSA = 0xD0 for the full BLE stack on 1 MiB, DDS set; SBRV at SFSA.
    assert_eq!(flash.peek(WB_SFR), WB_SFR_DDS | 0xd0);
    assert_eq!(flash.peek(WB_SRRVR) & 0x3_ffff, 0xd_0000 / 4);
    assert_ne!(
        flash.peek(WB_SRRVR) & (1 << 31),
        0,
        "C2OPT: CPU2 boots from flash"
    );
    assert_eq!(flash.peek(L4_WRP1AR), 0x0000_00ff, "no WRP area");
    // The other sizes ST publishes an install address for.
    for (size, sfsa) in [(0x8_0000u64, 0x5c), (0x4_0000, 0x1c), (0xa_0000, 0x7c)] {
        let flash = build(Variant::Wb, size, None);
        assert_eq!(flash.peek(WB_SFR) & WB_SFR_SFSA, sfsa, "{size:#x}");
    }
    // A WB array is whole 4 KiB pages, at most 256 of them.
    for size in [WB_SIZE + 2048, 2 * WB_SIZE] {
        let props = Props::new()
            .with("variant", Value::Str(String::from("wb")))
            .with("size", Value::Uint(size));
        assert!(Flash::new(&props).is_err(), "{size}");
    }
    // `sfr` is the WB's; `pcrop2sr` is not.
    let props = Props::new()
        .with("variant", Value::Str(String::from("l4")))
        .with("size", Value::Uint(L4_SIZE))
        .with("sfr", Value::Uint(0x1100));
    assert!(Flash::new(&props).is_err());
    let props = Props::new()
        .with("variant", Value::Str(String::from("wb")))
        .with("size", Value::Uint(WB_SIZE))
        .with("pcrop2sr", Value::Uint(0));
    assert!(Flash::new(&props).is_err());
}

#[test]
fn a_wb_key_written_twice_or_wrongly_is_a_bus_error() {
    let flash = wb_open();
    unlock(&flash);
    assert!(flash.unlocked());
    assert_eq!(
        flash.poke(L4_KEYR, KEY1),
        Err(BusError::BadAccess),
        "after unlocking"
    );
    opt_unlock(&flash);
    assert_eq!(flash.poke(L4_OPTKEYR, OPTKEY1), Err(BusError::BadAccess));
    // Setting LOCK sets OPTLOCK with it.
    flash.poke(L4_CR, CR_LOCK).unwrap();
    assert_eq!(
        flash.peek(L4_CR) & (CR_LOCK | L4_CR_OPTLOCK),
        CR_LOCK | L4_CR_OPTLOCK
    );

    let flash = wb_open();
    assert_eq!(flash.poke(L4_KEYR, 0x1234_5678), Err(BusError::BadAccess));
    assert_eq!(
        flash.poke(L4_KEYR, KEY1),
        Err(BusError::BadAccess),
        "until reset"
    );
    Device::reset(&flash, ResetKind::Warm);
    unlock(&flash);
    assert!(flash.unlocked());
}

#[test]
fn a_wb_double_word_sets_cfgbsy_from_the_first_word_and_eop_only_with_eopie() {
    let flash = wb_open();
    unlock(&flash);
    flash.poke(L4_CR, CR_PG).unwrap();
    flash.store(0x1000, &0x1111_2222u32.to_le_bytes()).unwrap();
    assert_eq!(sr(&flash), WB_SR_CFGBSY, "the first word is in");
    assert_eq!(
        flash.poke(L4_CR, CR_PG | CR_EOPIE),
        Err(BusError::BadAccess),
        "CR may not change between the two words"
    );
    flash.store(0x1004, &0x3333_4444u32.to_le_bytes()).unwrap();
    assert_eq!(sr(&flash), SR_BSY | WB_SR_CFGBSY);
    settle(&flash);
    assert_eq!(sr(&flash), 0, "done, and no EOP without EOPIE");
    assert_eq!(word(&flash, 0x1000), 0x1111_2222);
    assert_eq!(word(&flash, 0x1004), 0x3333_4444);
    // With EOPIE, EOP; zeroing a programmed double word is allowed.
    flash.poke(L4_CR, CR_PG | CR_EOPIE).unwrap();
    wb_double(&flash, 0x1000, 0, 0).unwrap();
    settle(&flash);
    assert_eq!(sr(&flash), SR_EOP);
    assert_eq!(word(&flash, 0x1000), 0);
    clear_sr(&flash);
    // Anything else over a programmed double word is PROGERR, and OPERR only
    // with ERRIE.
    wb_double(&flash, 0x1000, 1, 1).unwrap();
    assert_eq!(sr(&flash), SR_PROGERR);
    // An uncleared program error makes the next program a sequence error.
    flash.store(0x2000, &0u32.to_le_bytes()).unwrap();
    assert_eq!(sr(&flash), SR_PROGERR | SR_PGSERR);
    clear_sr(&flash);
    flash.poke(L4_CR, CR_PG | CR_ERRIE).unwrap();
    flash.store(0x2002, &[0, 0]).unwrap();
    assert_eq!(sr(&flash), L4_SR_SIZERR | SR_OPERR);
}

#[test]
fn a_wb_page_erase_is_4_kib_and_a_mass_erase_by_cpu1_is_a_bus_error() {
    let flash = wb_open();
    flash.load_image(0x3000, &[0u8; 0x2000]).unwrap();
    unlock(&flash);
    flash.poke(L4_CR, L4_CR_PER | (4 << 3) | CR_STRT).unwrap();
    assert_eq!(sr(&flash), SR_BSY | WB_SR_CFGBSY);
    settle(&flash);
    assert_eq!(word(&flash, 0x3ffc), 0, "page 3 untouched");
    assert_eq!(word(&flash, 0x4000), 0xffff_ffff);
    assert_eq!(word(&flash, 0x4ffc), 0xffff_ffff);
    // HAL's FLASH_MassErase from CPU1: ignored, and the bus says so.
    flash.load_image(0, &[0u8; 8]).unwrap();
    assert_eq!(
        flash.poke(L4_CR, WB_CR_MER | CR_STRT),
        Err(BusError::Protected)
    );
    settle(&flash);
    assert_eq!(word(&flash, 0), 0, "nothing erased");
    // PER with PG beside it is a sequence error; so is a page past the end.
    flash.poke(L4_CR, L4_CR_PER | CR_PG | CR_STRT).unwrap();
    assert_ne!(sr(&flash) & SR_PGSERR, 0);
    clear_sr(&flash);
    let small = build(Variant::Wb, 0x8_0000, None);
    unlock(&small);
    small
        .poke(L4_CR, L4_CR_PER | (0xff << 3) | CR_STRT)
        .unwrap();
    assert_ne!(sr(&small) & SR_PGSERR, 0, "page 255 of a 512 KiB part");
}

#[test]
fn cpu1_is_shut_out_of_the_wb_secure_area() {
    let flash = wb();
    let secure = 0xd0 * WB_PAGE;
    flash
        .load_image(secure, &0x5157_ac4bu32.to_le_bytes())
        .unwrap();
    flash.load_image(0, &0x2000_8000u32.to_le_bytes()).unwrap();
    // Reads below SFSA are ordinary; reads above it are zeroes and RDERR.
    assert_eq!(wb_read(&flash, 0, MemAttrs::DEFAULT), Ok(0x2000_8000));
    assert_eq!(wb_read(&flash, secure, MemAttrs::DEFAULT), Ok(0));
    assert_ne!(sr(&flash) & L4_SR_RDERR, 0);
    clear_sr(&flash);
    let fetch = MemAttrs::DEFAULT.with_purpose(crate::core::space::AccessPurpose::FETCH);
    assert_eq!(wb_read(&flash, secure, fetch), Ok(0), "no execution either");
    clear_sr(&flash);
    // A debugger is refused outright, and moves no flag.
    assert_eq!(
        wb_read(&flash, secure, MemAttrs::DEBUG),
        Err(BusError::Protected)
    );
    assert_eq!(sr(&flash) & L4_SR_RDERR, 0);
    // A program there is a bus error and writes nothing.
    unlock(&flash);
    flash.poke(L4_CR, CR_PG).unwrap();
    assert_eq!(
        flash.store(secure + 8, &0u32.to_le_bytes()),
        Err(BusError::Protected)
    );
    flash.poke(L4_CR, 0).unwrap();
    // A page erase of a secure page too, with no WRPERR (RM0434 Table 5).
    assert_eq!(
        flash.poke(L4_CR, L4_CR_PER | (0xd0 << 3) | CR_STRT),
        Err(BusError::Protected)
    );
    settle(&flash);
    assert_eq!(sr(&flash) & SR_WRPERR, 0);
    assert_eq!(word(&flash, secure), 0x5157_ac4b, "the stack survives");
    // The page below SFSA is the application's.
    flash
        .poke(L4_CR, L4_CR_PER | (0xcf << 3) | CR_STRT)
        .unwrap();
    settle(&flash);
    // SFR and SRRVR are CPU2's to write.
    assert_eq!(flash.poke(WB_SFR, WB_SFR_FSD), Err(BusError::Protected));
    assert_eq!(flash.poke(WB_SRRVR, 0), Err(BusError::Protected));
    assert_eq!(flash.peek(WB_SFR) & WB_SFR_SFSA, 0xd0);
    // A debugger cannot load there either, though the device's own door can.
    assert_eq!(
        flash.shared.program(secure, &[0; 4], MemAttrs::DEBUG),
        Err(BusError::Protected)
    );
    assert!(flash.load_image(secure, &[1, 2, 3, 4]).is_ok());
}

#[test]
fn a_wb_pes_suspends_a_new_operation_until_it_clears() {
    let flash = wb_open();
    unlock(&flash);
    flash.poke(ACR, WB_ACR_RESET | WB_ACR_PES).unwrap();
    assert_ne!(sr(&flash) & WB_SR_PESD, 0);
    assert_ne!(flash.peek(WB_C2SR) & WB_SR_PESD, 0, "both CPUs see it");
    flash.poke(L4_CR, CR_PG).unwrap();
    wb_double(&flash, 0x100, 0xdead_beef, 0x0bad_f00d).unwrap();
    settle(&flash);
    assert_eq!(sr(&flash), SR_BSY | WB_SR_CFGBSY | WB_SR_PESD, "held back");
    assert_eq!(word(&flash, 0x100), 0xffff_ffff);
    assert_eq!(
        flash.poke(L4_CR, 0),
        Err(BusError::BadAccess),
        "a CR write while PESD holds an operation"
    );
    // CPU2's PES holds it too, and only both clear lets it go.
    flash.poke(WB_C2ACR, WB_ACR_RESET | WB_ACR_PES).unwrap();
    flash.poke(ACR, WB_ACR_RESET).unwrap();
    settle(&flash);
    assert_ne!(sr(&flash) & SR_BSY, 0);
    flash.poke(WB_C2ACR, WB_ACR_RESET).unwrap();
    settle(&flash);
    assert_eq!(sr(&flash), 0);
    assert_eq!(word(&flash, 0x100), 0xdead_beef);
}

#[test]
fn a_wb_wrp_and_pcrop_area_refuse_program_and_erase() {
    let flash = wb_open();
    unlock(&flash);
    opt_unlock(&flash);
    // WRP area A over pages 2..=3; PCROP area A over 2 KiB units 20..=23
    // (pages 10 and 11).
    flash.poke(L4_WRP1AR, (3 << 16) | 2).unwrap();
    flash.poke(WB_PCROP1ASR, 20).unwrap();
    flash.poke(WB_PCROP1AER, WB_PCROP_RDP | 23).unwrap();
    flash.poke(L4_CR, L4_CR_PER | (3 << 3) | CR_STRT).unwrap();
    assert_ne!(sr(&flash) & SR_WRPERR, 0);
    clear_sr(&flash);
    flash.poke(L4_CR, L4_CR_PER | (11 << 3) | CR_STRT).unwrap();
    assert_ne!(sr(&flash) & SR_WRPERR, 0, "PCROP is erase-protected");
    clear_sr(&flash);
    flash.poke(L4_CR, L4_CR_PER | (12 << 3) | CR_STRT).unwrap();
    settle(&flash);
    assert_eq!(sr(&flash) & SR_WRPERR, 0);
    // A PCROP area can grow but not shrink.
    flash.poke(WB_PCROP1AER, 21).unwrap();
    assert_eq!(flash.peek(WB_PCROP1AER), WB_PCROP_RDP | 23);
    flash.poke(WB_PCROP1AER, 30).unwrap();
    assert_eq!(flash.peek(WB_PCROP1AER), WB_PCROP_RDP | 30);
    // Start equal to end protects the whole flash (RM0434 Table 15).
    assert_eq!(wb_pcrop_area(5, 5, WB_SIZE), (0, WB_SIZE));
    assert_eq!(wb_pcrop_area(6, 5, WB_SIZE), (0, 0));
    assert_eq!(wb_pcrop_area(2, 3, WB_SIZE), (0x1000, 0x1000));
}

#[test]
fn wb_option_bytes_commit_with_optstrt_keep_ese_read_only_and_reload() {
    let flash = wb();
    unlock(&flash);
    opt_unlock(&flash);
    let optr = flash.peek(L4_OPTR);
    // ESE cannot be written; BOR_LEV can.
    flash
        .poke(L4_OPTR, (optr & !WB_OPTR_ESE) | (2 << 9))
        .unwrap();
    assert_ne!(flash.peek(L4_OPTR) & WB_OPTR_ESE, 0);
    flash.poke(WB_IPCCBR, 0xffff_ffff).unwrap();
    assert_eq!(flash.peek(WB_IPCCBR), 0x3fff);
    flash.poke(L4_CR, L4_CR_OPTSTRT).unwrap();
    settle(&flash);
    Device::reset(&flash, ResetKind::Warm);
    assert_eq!((flash.peek(L4_OPTR) >> 9) & 7, 2, "committed");
    assert_eq!(flash.peek(WB_IPCCBR), 0x3fff);
    assert_eq!(flash.peek(WB_SFR) & WB_SFR_SFSA, 0xd0, "SFR untouched");
}

#[test]
fn a_wb_rdp_regression_keeps_the_secure_pages() {
    // RDP level 1 in the option bytes, with the stack installed.
    let props = Props::new()
        .with("variant", Value::Str(String::from("wb")))
        .with("size", Value::Uint(WB_SIZE))
        .with("optr", Value::Uint(0x2d8f_70bb));
    let flash = Flash::new(&props).expect("a WB at RDP 1");
    let secure = 0xd0 * WB_PAGE;
    flash.load_image(0, &[0u8; 16]).unwrap();
    flash.load_image(secure, &[0u8; 16]).unwrap();
    unlock(&flash);
    opt_unlock(&flash);
    let optr = flash.peek(L4_OPTR);
    flash.poke(L4_OPTR, (optr & !0xff) | 0xaa).unwrap();
    flash.poke(L4_CR, L4_CR_OPTSTRT).unwrap();
    settle(&flash);
    assert_eq!(word(&flash, 0), 0xffff_ffff, "the application is erased");
    assert_eq!(word(&flash, secure), 0, "CPU2's stack is not");
    assert_eq!(flash.peek(L4_OPTR) & 0xff, 0xaa);
}

#[test]
fn a_wb_empty_check_follows_the_first_double_word_across_a_reset() {
    let flash = wb_open();
    assert_ne!(flash.peek(ACR) & WB_ACR_EMPTY, 0);
    flash.load_image(0, &0x2003_0000u32.to_le_bytes()).unwrap();
    Device::reset(&flash, ResetKind::Warm);
    assert_eq!(flash.peek(ACR) & WB_ACR_EMPTY, 0);
    // Firmware may overwrite it.
    flash.poke(ACR, WB_ACR_RESET | WB_ACR_EMPTY | 3).unwrap();
    assert_eq!(flash.peek(ACR), WB_ACR_RESET | WB_ACR_EMPTY | 3);
    assert_eq!(flash.latency(), 3);
}

#[test]
fn a_wb_snapshot_round_trips_a_suspended_operation_and_a_half_double_word() {
    let saved = wb();
    unlock(&saved);
    saved.poke(ACR, WB_ACR_RESET | WB_ACR_PES).unwrap();
    saved.poke(L4_CR, L4_CR_PER | (7 << 3) | CR_STRT).unwrap();
    let bytes = save(&saved);
    let restored = wb();
    restore(&restored, &bytes);
    assert_eq!(save(&restored), bytes, "identical state");
    assert_eq!(sr(&restored), SR_BSY | WB_SR_CFGBSY | WB_SR_PESD);
    restored.load_image(7 * WB_PAGE, &[0u8; 4]).unwrap();
    restored.poke(ACR, WB_ACR_RESET).unwrap();
    settle(&restored);
    assert_eq!(word(&restored, 7 * WB_PAGE), 0xffff_ffff, "the erase ran");

    let saved = wb_open();
    unlock(&saved);
    saved.poke(L4_CR, CR_PG).unwrap();
    saved.store(0x800, &0x0102_0304u32.to_le_bytes()).unwrap();
    let bytes = save(&saved);
    let restored = wb_open();
    restore(&restored, &bytes);
    assert_eq!(save(&restored), bytes);
    restored
        .store(0x804, &0x0506_0708u32.to_le_bytes())
        .unwrap();
    settle(&restored);
    assert_eq!(word(&restored, 0x800), 0x0102_0304);
    assert_eq!(word(&restored, 0x804), 0x0506_0708);
}
