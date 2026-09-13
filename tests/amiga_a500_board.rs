//! The `amiga-a500` board's memory map, end to end.
//!
//! The map is the deliverable, so this is what proves it: an MC68000 named in a
//! `.machine` file finds its reset vector **in a ROM that is not mapped at
//! address zero**, runs out of that overlay, reaches the custom-chip register
//! space at `$DFF000` with a big-endian word, and then — when `OVL` goes down
//! — finds chip RAM at the addresses the ROM was answering at a moment ago.
//!
//! Everything here uses a **synthetic ROM built in this file**. No Kickstart
//! image is in this repository and none ever will be; a real one would not get
//! far anyway, because the board has no chipset yet.
//!
//! # Standing in for the CIA
//!
//! `OVL` is bit 0 of CIA-A's port A and there is no CIA in this build, so the
//! test drives the pin itself — through [`Device::sink`], the same entry point
//! the wire graph uses, rather than through a back door on the device. When
//! `mos.8520` lands, the machine file gets `wire cia_a.pa0 -> gary.ovl` and
//! this stimulus becomes a guest store.
//!
//! Everything needs a machine, so the whole file is gated on
//! `machine-amiga-a500`.

#![cfg(feature = "machine-amiga-a500")]

use std::sync::Arc;

use rsemu::core::clock::GlobalTime;
use rsemu::core::device::ExportId;
use rsemu::core::space::MemAttrs;
use rsemu::core::value::Width;
use rsemu::core::wire::{Level, WireId};
use rsemu::dev::amiga::custom::CustomBus;
use rsemu::machine::{Machine, catalog};

/// Where the board's chip RAM answers once `OVL` is cleared — which is also
/// where the ROM answers before that.
const ZERO: u64 = 0x00_0000;

/// The Kickstart socket, with the file's default 512 KiB size.
const ROM: u64 = 0xF8_0000;

/// The custom-chip register base (Amiga Hardware Reference Manual, Appendix D).
const CUSTOM: u64 = 0xDF_F000;

/// `COLOR00`, at offset `$180` — a Denise register, write-only.
const COLOR00: u64 = CUSTOM + 0x180;

/// `DMACONR`, at offset `$002` — readable, and owned by Agnus and Paula, so on
/// a board with neither it is the clearest thing to point at when asking what a
/// read with nothing attached does.
const DMACONR: u64 = CUSTOM + 0x002;

/// The word the firmware writes to `COLOR00`.
const COLOUR: u16 = 0x0F00;

/// A Kickstart-shaped image, hand-assembled from the MC68000 user manual's
/// instruction formats (MC68000UM, *Instruction Set Details*).
///
/// ```text
///   000000: 0008 0000        dc.l  $00080000   ; the reset supervisor stack pointer
///   000004: 0000 000c        dc.l  $0000000c   ; the reset program counter
///   00000c: 33fc 0f00 00df f180   move.w #$0f00, ($00dff180).l
///   000014: 60fe             bra    *
/// ```
///
/// The reset program counter is `$00000C` — an address in the **overlay**, not
/// in the ROM's own window at `$F80000`. That is the point: with `OVL`
/// asserted the processor executes out of a ROM whose mapped home is eight
/// megabytes away, exactly as an Amiga does out of reset, and if the overlay
/// were not working the fetch would land in empty RAM and the test would say so.
///
/// `$0F00` rather than a round number because every nibble of it is distinct:
/// a byte-swapped board writes `$000F` and this test fails instead of passing
/// by coincidence.
fn kickstart() -> Vec<u8> {
    image(&[
        0x33fc, COLOUR, 0x00df, 0xf180, // move.w #$0f00, ($00dff180).l
        0x60fe, // bra *
    ])
}

/// A 512 KiB image with the two reset longwords and `code` at `$00000C`.
fn image(code: &[u16]) -> Vec<u8> {
    let mut image = vec![0u8; 512 * 1024];
    image[0..4].copy_from_slice(&0x0008_0000u32.to_be_bytes()); // SSP: top of chip RAM
    image[4..8].copy_from_slice(&0x0000_000Cu32.to_be_bytes()); // PC: into the overlay
    for (i, word) in code.iter().enumerate() {
        let at = 0x0c + 2 * i;
        image[at..at + 2].copy_from_slice(&word.to_be_bytes());
    }
    image
}

/// Build the board with that image in its `kickstart` slot.
fn boot() -> Machine {
    let entry = catalog::machine("amiga-a500").expect("this build ships amiga-a500");
    build(entry.source, kickstart())
}

/// Build `source` with `rom` in its `kickstart` slot.
fn build(source: &str, rom: Vec<u8>) -> Machine {
    let mut options = catalog::build_options().expect("the catalog agrees with itself");
    options.realize.media.insert("kickstart", rom);
    let registry = catalog::registry().expect("a registry");
    match rsemu::machine::build("amiga-a500", source, &registry, &options) {
        Ok(m) => m,
        Err(e) => panic!("the board does not realize: {e}"),
    }
}

/// The shipped board with the CIA decode the machine file's comment block
/// describes — and two sixteen-byte RAMs where the 8520s will go.
///
/// **The RAMs are placeholders**, and obviously so: no timers, no interrupt
/// control register, no port pins. `mos.8520` is being written elsewhere and is
/// not in this build. What they prove is the part this board owns: that
/// register *n* of each chip lands where the manual's `$BFEr01` and `$BFDr00`
/// say, on the right half of the data bus, and that the two chips are distinct.
fn with_cia_stand_ins() -> String {
    let entry = catalog::machine("amiga-a500").expect("this build ships amiga-a500");
    let body = entry
        .source
        .trim_end()
        .strip_suffix('}')
        .expect("the machine block closes the file");
    format!(
        "{body}
  object cia_a \"ram\" {{ size = 16 }}
  object cia_b \"ram\" {{ size = 16 }}
  object cia_a_decode \"amiga.cia-decode\" {{ chip = cia_a, lane = \"odd\"  }}
  object cia_b_decode \"amiga.cia-decode\" {{ chip = cia_b, lane = \"even\" }}
  map mem 0xBFE000 size 0x1000 = cia_a_decode {{ endian = \"big\" }}
  map mem 0xBFD000 size 0x1000 = cia_b_decode {{ endian = \"big\" }}
}}
"
    )
}

/// Read a big-endian longword out of the guest's memory space, a byte at a
/// time, so the assertion is about the *board's* byte order rather than about
/// whatever width the test happened to ask for.
fn peek_long(m: &Machine, addr: u64) -> u32 {
    let space = m.space("mem").expect("the memory space");
    let mut value = 0u32;
    for i in 0..4 {
        let byte = space
            .read(addr + i, Width::U8, MemAttrs::DEFAULT)
            .expect("a mapped byte") as u32;
        value = (value << 8) | byte;
    }
    value
}

/// Read one big-endian word.
fn peek_word(m: &Machine, addr: u64) -> u16 {
    let space = m.space("mem").expect("the memory space");
    space
        .read(addr, Width::U16, MemAttrs::DEFAULT)
        .expect("a mapped word") as u16
}

/// Write one big-endian word.
fn poke_word(m: &Machine, addr: u64, value: u16) {
    let space = m.space("mem").expect("the memory space");
    space
        .write(addr, Width::U16, u64::from(value), MemAttrs::DEFAULT)
        .expect("a mapped word");
}

/// The custom-chip bus, reached the way a chip model will reach it: through the
/// [`ExportId::CUSTOM_BUS`] handle `amiga.custom` publishes.
fn custom_bus(m: &Machine) -> Arc<CustomBus> {
    let custom = m.device("custom").expect("the board has a `custom`");
    let export = custom
        .device()
        .export(ExportId::CUSTOM_BUS)
        .expect("`amiga.custom` publishes its register bus");
    Arc::clone(export.opaque().expect("an opaque handle"))
        .downcast::<CustomBus>()
        .expect("the handle is a `CustomBus`")
}

/// Drive `OVL` the way CIA-A's port A will once there is a CIA.
///
/// Through [`Device::sink`] and a [`WireId`], which is what the wire graph
/// itself does — not through a method on `Gary` that only a test would call.
fn set_ovl(m: &Machine, level: Level) {
    let gary = m.device("gary").expect("the board has a `gary`");
    let src = WireId::new(1);
    let pin = gary
        .device()
        .sink("ovl", &[src])
        .expect("`amiga.gary` has an `ovl` input");
    pin.sink.set_level(src, pin.line, level);
}

#[test]
fn the_board_realizes_with_every_object_the_map_needs() {
    let m = boot();
    assert_eq!(m.name(), "amiga-a500");
    for path in ["cpu", "chipram", "kick", "custom", "gary"] {
        assert!(
            m.device(path).is_some(),
            "the machine has no instance called `{path}`"
        );
    }
}

#[test]
fn out_of_reset_the_rom_answers_at_zero_as_well_as_at_its_own_address() {
    let m = boot();
    // `OVL` comes up asserted, so the two longwords the processor fetches out
    // of reset are the ROM's first eight bytes — read at address zero, where
    // the ROM is not mapped.
    assert_eq!(peek_long(&m, ZERO), 0x0008_0000, "the reset stack pointer");
    assert_eq!(peek_long(&m, ZERO + 4), 0x0000_000C, "the reset PC");
    // And the same bytes at the ROM's own window, which is a different mapping
    // reaching the same store.
    assert_eq!(peek_long(&m, ROM), 0x0008_0000);
    assert_eq!(peek_long(&m, ROM + 4), 0x0000_000C);
}

#[test]
fn the_firmware_runs_out_of_the_overlay_and_reaches_the_custom_chip_space() {
    let mut m = boot();
    // A millisecond of virtual time at 7.09 MHz is about 7,000 cycles; the
    // program is under fifty, reset sequence included. A span rather than an
    // instruction count because the scheduler hands out budgets, not steps.
    m.run_for(GlobalTime::from_nanos(1_000_000))
        .expect("it runs");

    let bus = custom_bus(&m);

    // The word reached the register space in the right byte order. Nothing is
    // subscribed to Denise's half of the table, so the write was not claimed —
    // which is the placeholder state this board is in and is worth asserting
    // rather than glossing: when Denise lands, this count goes to zero and the
    // colour ends up in a palette entry instead.
    assert_eq!(
        bus.floating(),
        COLOUR,
        "either the MOVE.W never ran, or the board put a big-endian core on a \
         little-endian map and the word is swapped"
    );
    assert_eq!(
        bus.unclaimed(),
        1,
        "exactly one access, and nothing owns COLOR00 in this build"
    );
}

#[test]
fn the_custom_space_decodes_words_and_refuses_everything_else() {
    let m = boot();
    let space = m.space("mem").expect("the memory space");

    // A write-only register read back gives the floating chip data bus, which
    // with no chipset attached is the last word written. Placeholder behaviour,
    // documented as such in `src/dev/amiga/custom.rs`.
    poke_word(&m, COLOR00, 0x0123);
    assert_eq!(peek_word(&m, COLOR00), 0x0123);

    // A readable register whose owners are not in this build answers the same
    // way rather than inventing a value.
    assert_eq!(peek_word(&m, DMACONR), 0x0123);

    // The appendix's registers are words. A byte access has no documented
    // meaning, so it is refused rather than guessed at.
    assert!(
        space.read(COLOR00, Width::U8, MemAttrs::DEFAULT).is_err(),
        "a byte read of a custom register should be refused"
    );
    assert!(
        space
            .read(COLOR00 + 1, Width::U16, MemAttrs::DEFAULT)
            .is_err(),
        "an unaligned word read should be refused"
    );

    // The appendix's table stops at $1E4 and the board maps 512 bytes, so
    // $DFF200 is off the end of the window and the space's `fault` policy
    // applies.
    assert!(
        space
            .read(CUSTOM + 0x200, Width::U16, MemAttrs::DEFAULT)
            .is_err(),
        "the board maps only the 512 bytes the appendix documents"
    );
}

#[test]
fn clearing_ovl_puts_chip_ram_at_zero_and_the_processor_keeps_running() {
    let mut m = boot();
    m.run_for(GlobalTime::from_nanos(1_000_000))
        .expect("it runs");
    assert_eq!(peek_long(&m, ZERO), 0x0008_0000, "still the ROM");

    set_ovl(&m, Level::Low);

    // Chip RAM was zeroed by the cold reset and nothing has written it, so the
    // bytes that were the reset vector are now zero — and the ROM is still at
    // its own address, which is what makes this a decode rather than a move.
    assert_eq!(peek_long(&m, ZERO), 0, "chip RAM, not the ROM");
    assert_eq!(peek_long(&m, ROM), 0x0008_0000, "the ROM has not moved");

    // And it is writable, which the ROM behind the same window was not.
    poke_word(&m, ZERO, 0xDEAD);
    poke_word(&m, ZERO + 2, 0xBEEF);
    assert_eq!(peek_long(&m, ZERO), 0xDEAD_BEEF);

    // Putting `OVL` back hides it again without losing it — the overlay is
    // combinational, so nothing was copied anywhere.
    set_ovl(&m, Level::High);
    assert_eq!(peek_long(&m, ZERO), 0x0008_0000);
    set_ovl(&m, Level::Low);
    assert_eq!(peek_long(&m, ZERO), 0xDEAD_BEEF);

    // The processor is parked in `BRA *` in the ROM's own window, so it goes on
    // running with chip RAM underneath it.
    m.run_for(GlobalTime::from_nanos(1_000_000))
        .expect("it keeps running");
    assert_eq!(peek_long(&m, ZERO), 0xDEAD_BEEF);
}

#[test]
fn a_write_through_the_overlay_while_ovl_is_asserted_reaches_the_rom_and_is_dropped() {
    let m = boot();
    // The ROM is what answers, and a `rom` object drops writes rather than
    // faulting — which is what a ROM on a bus does. The bytes are unchanged and
    // chip RAM underneath is untouched, which the flip proves.
    poke_word(&m, ZERO, 0x1234);
    assert_eq!(peek_long(&m, ZERO), 0x0008_0000);
    set_ovl(&m, Level::Low);
    assert_eq!(peek_long(&m, ZERO), 0, "the write did not land in chip RAM");
}

#[test]
fn a_reset_puts_the_overlay_back_so_the_machine_can_start_again() {
    let mut m = boot();
    set_ovl(&m, Level::Low);
    assert_eq!(peek_long(&m, ZERO), 0);

    m.reset(rsemu::core::device::ResetKind::Cold);

    // Without this a warm boot would fetch its reset vector out of RAM that a
    // cold reset had just zeroed, and the processor would go nowhere.
    assert_eq!(peek_long(&m, ZERO), 0x0008_0000);
}

#[test]
fn the_board_snapshots_and_restores_to_an_identical_state_hash() {
    let mut m = boot();
    m.run_for(GlobalTime::from_nanos(1_000_000))
        .expect("it runs");
    // With the overlay *cleared*, because the level is the interesting piece of
    // state: a restore does not re-run the wire graph, so a decoder that failed
    // to carry it would come back pointed at the ROM over a running system's
    // vector table.
    set_ovl(&m, Level::Low);
    poke_word(&m, ZERO, 0xC0DE);

    let bytes = m.save().expect("the machine snapshots");
    let before = m.state_hash().expect("a hash");

    let mut other = boot();
    other.load(&bytes).expect("the snapshot loads");
    assert_eq!(
        other.state_hash().expect("a hash"),
        before,
        "a save/load round trip changed the machine's state hash"
    );
    assert_eq!(
        peek_word(&other, ZERO),
        0xC0DE,
        "the overlay came back down"
    );
}

#[test]
fn an_overlay_pointed_at_something_with_no_region_is_a_realize_error_naming_it() {
    // `BindCtx::region`'s refusal, reached the way a typo in a machine file
    // would reach it: the processor has no region to forward into.
    let entry = catalog::machine("amiga-a500").expect("this build ships amiga-a500");
    let source = entry.source.replace("ram   = chipram", "ram   = cpu");
    assert_ne!(source, entry.source, "the substitution found its line");
    let mut options = catalog::build_options().expect("the catalog agrees with itself");
    options.realize.media.insert("kickstart", kickstart());
    let registry = catalog::registry().expect("a registry");
    let err = match rsemu::machine::build("amiga-a500", &source, &registry, &options) {
        Ok(_) => panic!("an overlay onto a processor realized"),
        Err(e) => e.to_string(),
    };
    assert!(err.contains("`cpu`"), "{err}");
    assert!(err.contains("gary"), "{err}");
}

#[test]
fn the_cia_decode_puts_each_register_where_the_manual_says_on_the_right_lane() {
    // Hand-assembled as above (MC68000UM): `MOVE.B #imm,(xxx).L` is $13FC, and
    // its immediate byte travels in the low half of a word.
    let rom = image(&[
        0x13fc, 0x0003, 0x00bf, 0xe001, // move.b #$03, ($00bfe001).l   CIA-A register 0
        0x13fc, 0x005a, 0x00bf, 0xd100, // move.b #$5a, ($00bfd100).l   CIA-B register 1
        0x60fe, // bra *
    ]);
    let mut m = build(&with_cia_stand_ins(), rom);
    m.run_for(GlobalTime::from_nanos(1_000_000))
        .expect("it runs");
    let space = m.space("mem").expect("the memory space");
    let byte = |addr: u64| {
        space
            .read(addr, Width::U8, MemAttrs::DEFAULT)
            .expect("a decoded byte")
    };

    // Each store reached its own chip's register and nothing else.
    assert_eq!(byte(0xBFE001), 0x03, "CIA-A register 0 at $BFE001");
    assert_eq!(byte(0xBFD100), 0x5a, "CIA-B register 1 at $BFD100");
    assert_eq!(byte(0xBFE101), 0x00, "CIA-A register 1 is a different byte");
    assert_eq!(byte(0xBFD000), 0x00, "CIA-B register 0 is a different byte");

    // A word at a register reads that chip on its own lane and nothing on the
    // other: one access selects one CIA.
    let floating = MemAttrs::DEFAULT.with_bus(0xee);
    assert_eq!(
        space.read(0xBFE000, Width::U16, floating).expect("a word"),
        0xee03,
        "CIA-A is on the low byte, and the high byte is undriven"
    );
    assert_eq!(
        space.read(0xBFD100, Width::U16, floating).expect("a word"),
        0x5aee,
        "CIA-B is on the high byte, and the low byte is undriven"
    );
}
