//! The `amiga-a500` board's memory map, end to end.
//!
//! The map is the deliverable, so this is what proves it: an MC68000 named in a
//! `.machine` file finds its reset vector **in a ROM that is not mapped at
//! address zero**, runs out of that overlay, reaches the custom-chip register
//! space at `$DFF000` with a big-endian word, and then — when `OVL` goes down
//! — finds chip RAM at the addresses the ROM was answering at a moment ago.
//!
//! Everything here uses a **synthetic ROM built in this file**. No Kickstart
//! image is in this repository and none ever will be.
//!
//! # The overlay is driven by the real CIA
//!
//! `OVL` is bit 0 of CIA-A's port A (Appendix E: "PA0..OVL"), and the board
//! carries two real `mos.8520`s and `wire cia_a.pa0 -> gary.ovl`. So every
//! change to the overlay here is a **store into CIA-A** — `DDRA` to make the
//! pin an output, `PRA` to set its level — through the same `$BFEr01` decode a
//! guest uses, and never a poke at the decoder's pin.
//!
//! Everything needs a machine, so the whole file is gated on
//! `machine-amiga-a500`.

#![cfg(feature = "machine-amiga-a500")]

use std::sync::Arc;

use rsemu::core::clock::GlobalTime;
use rsemu::core::device::ExportId;
use rsemu::core::space::MemAttrs;
use rsemu::core::value::Width;
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

/// `JOY0DAT`, at offset `$00A` — readable, and owned by Denise alone.
const JOY0DAT: u64 = CUSTOM + 0x00A;

/// `DMACONR`, at offset `$002` — readable, owned by Agnus and Paula together.
const DMACONR: u64 = CUSTOM + 0x002;

/// `INTENA`, `INTENAR` and `INTREQR`, Paula's.
const INTENA: u64 = CUSTOM + 0x09A;
const INTENAR: u64 = CUSTOM + 0x01C;
const INTREQR: u64 = CUSTOM + 0x01E;

/// `PORTS`, the level 2 request bit that `INT2*` sets.
const PORTS: u16 = 0x0008;

/// The word the firmware writes to `COLOR00`.
const COLOUR: u16 = 0x0F00;

/// CIA-A's `PRA`, register 0: `$BFEr01` with `r` = 0 (Appendix F).
const CIAA_PRA: u64 = 0xBF_E001;

/// CIA-A's `DDRA`, register 2. A one makes the pin an output.
const CIAA_DDRA: u64 = 0xBF_E201;

/// CIA-B's `DDRB`, register 3: `$BFDr00` with `r` = 3.
const CIAB_DDRB: u64 = 0xBF_D300;

/// `PA0`, the `OVL` bit.
const OVL: u8 = 0x01;

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
    // DF0 names a media slot; an empty one is an empty drive (the PC floppy
    // precedent, `tests/pc_at_ide.rs`). The front ends bind it for a user.
    options.realize.media.insert("df0", Vec::new());
    let registry = catalog::registry().expect("a registry");
    match rsemu::machine::build("amiga-a500", source, &registry, &options) {
        Ok(m) => m,
        Err(e) => panic!("the board does not realize: {e}"),
    }
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

/// Read one byte.
fn peek_byte(m: &Machine, addr: u64) -> u8 {
    let space = m.space("mem").expect("the memory space");
    space
        .read(addr, Width::U8, MemAttrs::DEFAULT)
        .expect("a mapped byte") as u8
}

/// Write one byte.
fn poke_byte(m: &Machine, addr: u64, value: u8) {
    let space = m.space("mem").expect("the memory space");
    space
        .write(addr, Width::U8, u64::from(value), MemAttrs::DEFAULT)
        .expect("a mapped byte");
}

/// Set `OVL` the way software does: make `PA0` an output and write the level
/// into `PRA`, both through CIA-A's own decode.
///
/// The level goes in first, so the pin never glitches low on its way to high.
fn set_ovl(m: &Machine, asserted: bool) {
    poke_byte(m, CIAA_PRA, if asserted { OVL } else { 0 });
    poke_byte(m, CIAA_DDRA, OVL);
}

#[test]
fn the_board_realizes_with_every_object_the_map_needs() {
    let m = boot();
    assert_eq!(m.name(), "amiga-a500");
    for path in [
        "cpu",
        "chipram",
        "kick",
        "custom",
        "gary",
        "cia_a",
        "cia_b",
        "cia_a_decode",
        "cia_b_decode",
        "paula",
        "df0",
        "agnus",
        "denise",
    ] {
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

    // The word reached the register space in the right byte order, and with
    // Denise on the board it was claimed: nothing on this board falls through.
    assert_eq!(
        bus.floating(),
        COLOUR,
        "either the MOVE.W never ran, or the board put a big-endian core on a \
         little-endian map and the word is swapped"
    );
    assert_eq!(
        bus.unclaimed(),
        0,
        "COLOR00 is Denise's, and Denise is on the board"
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

    // An offset the appendix leaves empty answers the same way rather than
    // inventing a value.
    assert_eq!(peek_word(&m, CUSTOM + 0x068), 0x0123);

    // A readable register with its owner present answers from the chip:
    // Denise's `JOY0DAT` with no mouse moved, and `DMACONR`, which Agnus and
    // Paula share and whose every bit is Agnus's, with no channel enabled.
    assert_eq!(peek_word(&m, JOY0DAT), 0x0000);
    assert_eq!(peek_word(&m, DMACONR), 0x0000);

    // The appendix's registers are words, and a byte access is the word access
    // the chips see (`src/dev/amiga/custom.rs` has the sources): a read keeps
    // the half its strobe selects, and a write drives the byte onto both halves
    // (MC68000UM Table 3-1). The `DMACONR` read above left its own word on
    // the bus, so put a known one back first.
    poke_word(&m, COLOR00, 0x0123);
    assert_eq!(
        space.read(COLOR00, Width::U8, MemAttrs::DEFAULT),
        Ok(0x01),
        "UDS alone: the upper half of the floating word"
    );
    assert_eq!(
        space.read(COLOR00 + 1, Width::U8, MemAttrs::DEFAULT),
        Ok(0x23),
        "LDS alone: the lower half"
    );
    space
        .write(COLOR00 + 1, Width::U8, 0x45, MemAttrs::DEFAULT)
        .expect("a byte write reaches the register");
    assert_eq!(
        peek_word(&m, COLOR00),
        0x4545,
        "the byte in both halves, none kept"
    );
    assert!(
        space
            .read(COLOR00 + 1, Width::U16, MemAttrs::DEFAULT)
            .is_err(),
        "an unaligned word read should be refused"
    );

    // The appendix's table stops at $1FE and the board maps 512 bytes, so
    // $DFF200 is off the end of the window. On an A500 an empty address
    // completes and floats (Appendix K), so it is not an error either.
    assert!(
        space
            .read(CUSTOM + 0x200, Width::U16, MemAttrs::DEFAULT)
            .is_ok(),
        "past the 512 bytes the appendix documents, the bus floats"
    );
}

#[test]
fn clearing_ovl_puts_chip_ram_at_zero_and_the_processor_keeps_running() {
    let mut m = boot();
    m.run_for(GlobalTime::from_nanos(1_000_000))
        .expect("it runs");
    assert_eq!(peek_long(&m, ZERO), 0x0008_0000, "still the ROM");

    set_ovl(&m, false);

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
    set_ovl(&m, true);
    assert_eq!(peek_long(&m, ZERO), 0x0008_0000);
    set_ovl(&m, false);
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
    set_ovl(&m, false);
    assert_eq!(peek_long(&m, ZERO), 0, "the write did not land in chip RAM");
}

#[test]
fn a_reset_puts_the_overlay_back_so_the_machine_can_start_again() {
    let mut m = boot();
    set_ovl(&m, false);
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
    set_ovl(&m, false);
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
    // DF0 names a media slot; an empty one is an empty drive (the PC floppy
    // precedent, `tests/pc_at_ide.rs`). The front ends bind it for a user.
    options.realize.media.insert("df0", Vec::new());
    let registry = catalog::registry().expect("a registry");
    let err = match rsemu::machine::build("amiga-a500", &source, &registry, &options) {
        Ok(_) => panic!("an overlay onto a processor realized"),
        Err(e) => e.to_string(),
    };
    assert!(err.contains("`cpu`"), "{err}");
    assert!(err.contains("gary"), "{err}");
}

#[test]
fn out_of_reset_the_cia_pin_holds_the_overlay_up() {
    let mut m = boot();
    m.run_for(GlobalTime::from_nanos(1_000_000))
        .expect("it runs");
    // `DDRA` resets to zero, so `PA0` is an input showing the 8520's pull-up,
    // which is a high level on `OVL`: the ROM stays at zero without anybody
    // having written anything.
    assert_eq!(peek_byte(&m, CIAA_DDRA), 0);
    assert_eq!(peek_byte(&m, CIAA_PRA) & OVL, OVL, "the pull-up");
    assert_eq!(peek_long(&m, ZERO), 0x0008_0000, "still the ROM");
}

#[test]
fn a_guest_that_clears_ovl_through_cia_a_finds_chip_ram_at_zero() {
    // What a Kickstart does first: leave the overlay by jumping to the ROM's
    // own address, then clear `OVL` through the CIA, then write the vector
    // table in the RAM that has appeared underneath (MC68000UM for the
    // encodings: `JMP (xxx).L` is $4EF9, `MOVE.B #imm,(xxx).L` is $13FC,
    // `MOVE.L #imm,(xxx).L` is $23FC).
    let rom = image(&[
        0x4ef9, 0x00f8, 0x0012, // 00c: jmp ($00f80012).l
        0x13fc, 0x0000, 0x00bf, 0xe001, // 012: move.b #$00, ($00bfe001).l   PRA
        0x13fc, 0x0001, 0x00bf, 0xe201, // 01a: move.b #$01, ($00bfe201).l   DDRA
        0x23fc, 0xc0de, 0xcafe, 0x0000, 0x0000, // 022: move.l #$c0decafe, ($00000000).l
        0x60fe, // 02c: bra *
    ]);
    let mut m = build(catalog::machine("amiga-a500").unwrap().source, rom);
    m.run_for(GlobalTime::from_nanos(2_000_000))
        .expect("it runs");
    assert_eq!(peek_byte(&m, CIAA_DDRA), OVL, "PA0 is an output");
    assert_eq!(
        peek_long(&m, ZERO),
        0xC0DE_CAFE,
        "the store landed in chip RAM, so the CIA's pin reached the overlay"
    );
    assert_eq!(peek_long(&m, ROM), 0x0008_0000, "the ROM has not moved");
}

/// Where the interrupt test's handler counts.
const COUNTER: u64 = 0x00_0100;

/// A guest that takes CIA-A's timer as a level 2 autovectored interrupt.
///
/// Hand-assembled (MC68000UM for the encodings; Appendix F for the CIA
/// registers; chapter 7 for `INTENA` and `INTREQ`):
///
/// ```text
///   00c: jmp     ($00f80012).l            ; leave the overlay
///   012: move.b  #$00, ($00bfe001).l      ; PRA:  OVL low
///   01a: move.b  #$01, ($00bfe201).l      ; DDRA: PA0 an output -> chip RAM at 0
///   022: move.l  #$00f80060, ($00000068).l; vector 26, the level 2 autovector
///   02c: move.w  #$c008, ($00dff09a).l    ; INTENA: SET | INTEN | PORTS
///   034: move.b  #$00, ($00bfe401).l      ; TA LO
///   03c: move.b  #$10, ($00bfe501).l      ; TA HI: $1000 E clocks, 5.8 ms
///   044: move.b  #$81, ($00bfed01).l      ; ICR: SET | TA
///   04c: move.b  #$01, ($00bfee01).l      ; CRA: START, continuous
///   054: move.w  #$2000, sr               ; unmask
///   058: bra     *
///   05a: nop; nop; nop
///   060: addq.l  #1, ($00000100).l        ; the handler
///   066: tst.b   ($00bfed01).l            ; read ICR: clears it, lets INT2* go
///   06c: move.w  #$0008, ($00dff09c).l    ; INTREQ: clear PORTS
///   074: rte
/// ```
fn timer_interrupt_rom() -> Vec<u8> {
    image(&[
        0x4ef9, 0x00f8, 0x0012, //
        0x13fc, 0x0000, 0x00bf, 0xe001, //
        0x13fc, 0x0001, 0x00bf, 0xe201, //
        0x23fc, 0x00f8, 0x0060, 0x0000, 0x0068, //
        0x33fc, 0xc008, 0x00df, 0xf09a, //
        0x13fc, 0x0000, 0x00bf, 0xe401, //
        0x13fc, 0x0010, 0x00bf, 0xe501, //
        0x13fc, 0x0081, 0x00bf, 0xed01, //
        0x13fc, 0x0001, 0x00bf, 0xee01, //
        0x46fc, 0x2000, //
        0x60fe, //
        0x4e71, 0x4e71, 0x4e71, //
        0x52b9, 0x0000, 0x0100, //
        0x4a39, 0x00bf, 0xed01, //
        0x33fc, 0x0008, 0x00df, 0xf09c, //
        0x4e73,
    ])
}

#[test]
fn cia_a_interrupts_the_processor_at_level_2_through_paula() {
    let mut m = build(
        catalog::machine("amiga-a500").unwrap().source,
        timer_interrupt_rom(),
    );
    // Fifty milliseconds: eight or so periods of a 5.8 ms timer.
    m.run_for(GlobalTime::from_nanos(50_000_000))
        .expect("it runs");
    assert_eq!(peek_word(&m, INTENAR), 0x4008, "INTEN and PORTS");
    let taken = peek_long(&m, COUNTER);
    assert!(
        (6..=10).contains(&taken),
        "the handler ran {taken} times; a 5.8 ms timer over 50 ms is eight"
    );
    // Every one was acknowledged at the chip and at Paula, so nothing is left
    // pending once the handler has run.
    assert_eq!(peek_word(&m, INTREQR) & PORTS, 0);
}

#[test]
fn an_unmasked_cia_b_request_is_level_6() {
    let mut m = boot();
    // CIA-B's timer B, one-shot, with its interrupt enabled: poke it straight
    // through the board's decode and let the timer run out.
    poke_word(&m, INTENA, 0xc000 | 0x2000); // SET | INTEN | EXTER
    poke_byte(&m, 0xBF_D600, 0x02); // TB LO
    poke_byte(&m, 0xBF_D700, 0x00); // TB HI
    poke_byte(&m, 0xBF_DD00, 0x82); // ICR: SET | TB
    poke_byte(&m, 0xBF_DF00, 0x09); // CRB: START | ONESHOT
    m.run_for(GlobalTime::from_nanos(1_000_000))
        .expect("it runs");
    assert_eq!(peek_word(&m, INTREQR) & 0x2000, 0x2000, "EXTER");
}

#[test]
fn a_guest_serdat_reaches_the_host_serial_port() {
    // 00c: move.w #372, ($00dff032).l   SERPER: 9600 baud on PAL, (3546895/9600)-1
    // 014: move.w #$0141, ($00dff030).l SERDAT: 'A' and a stop bit
    // 01c: bra *
    let rom = image(&[
        0x33fc, 0x0173, 0x00df, 0xf032, //
        0x33fc, 0x0141, 0x00df, 0xf030, //
        0x60fe,
    ]);
    let mut options = catalog::build_options().expect("the catalog agrees with itself");
    options.realize.media.insert("kickstart", rom);
    // DF0 names a media slot; an empty one is an empty drive (the PC floppy
    // precedent, `tests/pc_at_ide.rs`). The front ends bind it for a user.
    options.realize.media.insert("df0", Vec::new());
    let registry = catalog::registry().expect("a registry");
    let source = catalog::machine("amiga-a500").unwrap().source;
    let mut m = rsemu::machine::build("amiga-a500", source, &registry, &options)
        .unwrap_or_else(|e| panic!("the board does not realize: {e}"));
    let host = rsemu::host::chardev::ports::open(&options.realize.hosts, "serial")
        .expect("Paula opened it");
    // Ten bits at 9600 baud is a little over a millisecond.
    m.run_for(GlobalTime::from_nanos(5_000_000))
        .expect("it runs");
    assert_eq!(host.drain(), b"A".to_vec());
}

/// A raw MFM disk for DF0 in which every track starts with its own number and
/// then the `$4489` sync mark, and is otherwise blank.
fn numbered_disk() -> Vec<u8> {
    const TRACK: usize = 12_500;
    let mut raw = vec![0u8; 160 * TRACK];
    for (t, track) in raw.chunks_mut(TRACK).enumerate() {
        track[..3].copy_from_slice(&[t as u8, 0x44, 0x89]);
    }
    raw
}

#[test]
fn df0_answers_the_cia_lines_the_way_table_8_5_says() {
    let mut options = catalog::build_options().expect("the catalog agrees with itself");
    options.realize.media.insert("kickstart", kickstart());
    options.realize.media.insert("df0", numbered_disk());
    let registry = catalog::registry().expect("a registry");
    // The shipped board names DF0's `df0` slot, so a disk goes straight in.
    let source = catalog::machine("amiga-a500").unwrap().source;
    let mut m = rsemu::machine::build("amiga-a500", source, &registry, &options)
        .unwrap_or_else(|e| panic!("the board does not realize: {e}"));

    const CIAB_PRB: u64 = 0xBF_D100;
    const CIAB_ICR: u64 = 0xBF_DD00;
    const RDY: u8 = 0x20;
    const TK0: u8 = 0x10;
    const WPRO: u8 = 0x08;
    const CHNG: u8 = 0x04;
    let lines = |m: &Machine| peek_byte(m, CIAA_PRA) & (RDY | TK0 | WPRO | CHNG);

    assert_eq!(
        lines(&m),
        RDY | TK0 | WPRO | CHNG,
        "deselected: all pulled up"
    );

    // Appendix F: "ddrb ... (set to 0xFF)". Then the motor, then the select —
    // "software that selects drives must set up the motor signal before
    // selecting any drives".
    poke_byte(&m, CIAB_DDRB, 0xff);
    poke_byte(&m, CIAB_PRB, 0xff);
    poke_byte(&m, CIAB_PRB, 0x7f); // MTR* low
    poke_byte(&m, CIAB_PRB, 0x77); // SEL0* low
    assert_eq!(
        lines(&m),
        WPRO,
        "ready, on track 0, not protected, and the change flop set at power-up"
    );

    // One step towards the centre: DIR low, then a pulse on STEP*.
    poke_byte(&m, CIAB_PRB, 0x75);
    poke_byte(&m, CIAB_PRB, 0x74);
    poke_byte(&m, CIAB_PRB, 0x75);
    assert_eq!(
        lines(&m),
        WPRO | TK0 | CHNG,
        "off track 0, and the flop reset"
    );

    // Let a revolution pass: the index pulse falls on CIA-B's /FLAG. Reading
    // the ICR clears it, so read it once, afterwards.
    let _ = peek_byte(&m, CIAB_ICR);
    m.run_for(GlobalTime::from_nanos(250_000_000))
        .expect("it runs");
    assert_eq!(peek_byte(&m, CIAB_ICR) & 0x10, 0x10, "ICR FLAG: the index");

    // And Paula reads the track under the head: cylinder 1, side 0, track 2,
    // whose sync mark sets DSKSYN.
    poke_word(&m, CUSTOM + 0x09E, 0x8100); // ADKCON: SET | FAST
    poke_word(&m, CUSTOM + 0x07E, 0x4489); // DSKSYNC
    poke_word(&m, CUSTOM + 0x09C, 0x7fff); // INTREQ: clear everything
    m.run_for(GlobalTime::from_nanos(250_000_000))
        .expect("it runs");
    assert_eq!(peek_word(&m, INTREQR) & 0x1000, 0x1000, "DSKSYN");
}

#[test]
fn the_cia_decode_puts_each_register_where_the_manual_says_on_the_right_lane() {
    let m = boot();
    // The direction registers read back exactly what was written, which makes
    // them the registers to aim at: a port register reads its pins.
    poke_byte(&m, CIAA_DDRA, 0x02);
    poke_byte(&m, CIAB_DDRB, 0x5a);

    // Each store reached its own chip's register and nothing else.
    assert_eq!(peek_byte(&m, CIAA_DDRA), 0x02, "CIA-A DDRA at $BFE201");
    assert_eq!(peek_byte(&m, CIAB_DDRB), 0x5a, "CIA-B DDRB at $BFD300");
    assert_eq!(
        peek_byte(&m, 0xBF_E301),
        0x00,
        "CIA-A DDRB is a different byte"
    );
    assert_eq!(
        peek_byte(&m, 0xBF_D200),
        0x00,
        "CIA-B DDRA is a different byte"
    );

    // A word at a register reads that chip on its own lane and nothing on the
    // other: one access selects one CIA.
    let space = m.space("mem").expect("the memory space");
    let floating = MemAttrs::DEFAULT.with_bus(0xee);
    assert_eq!(
        space.read(0xBF_E200, Width::U16, floating).expect("a word"),
        0xee02,
        "CIA-A is on the low byte, and the high byte is undriven"
    );
    assert_eq!(
        space.read(0xBF_D300, Width::U16, floating).expect("a word"),
        0x5aee,
        "CIA-B is on the high byte, and the low byte is undriven"
    );
}
