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
    let keyr = if flash.variant().is_f4() {
        F4_KEYR
    } else {
        L4_KEYR
    };
    flash.poke(keyr, KEY1).expect("KEY1");
    flash.poke(keyr, KEY2).expect("KEY2");
}

/// The option unlock sequence.
fn opt_unlock(flash: &Flash) {
    let reg = if flash.variant().is_f4() {
        F4_OPTKEYR
    } else {
        L4_OPTKEYR
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
