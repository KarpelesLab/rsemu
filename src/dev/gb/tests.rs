//! Tests for the Game Boy's devices.
//!
//! Every device here carries a save/load round trip, because CLAUDE.md asks for
//! one with the device rather than later, and the interesting behaviours — the
//! divider's side effects, the LCD's variable mode 3, MBC1's unreachable banks —
//! get a test each because none of them is visible in a screenshot.

use alloc::sync::Arc;
use alloc::vec;
use alloc::vec::Vec;

use crate::core::device::{Device, ResetKind};
use crate::core::props::Props;
use crate::core::space::{MemAttrs, MemOps, RegionKind, RegionRef};
use crate::core::state::{MachineShape, Migrations, StateReader, StateWriter};

use super::apu::GbApu;
use super::cart::{Cartridge, GbCart, Mapper, RTC_HZ, synthetic_image};
use super::joypad::{Button, GbJoypad};
use super::ppu::{self, GbPpu, Mode, lcdc, stat};
use super::serial::{GbSerial, TRANSFER_CLOCKS};
use super::timer::GbTimer;

/// The [`MemOps`] behind an I/O region.
///
/// A test reaches a device's registers the way the address space does, rather
/// than through a private method, so that what is tested is the surface a guest
/// actually sees.
fn io(region: &RegionRef) -> &Arc<dyn MemOps> {
    match region.kind() {
        RegionKind::Io(ops) => ops,
        other => panic!("expected an I/O region, found {other:?}"),
    }
}

/// Save a device and load it back into a fresh one, asserting the bytes match.
///
/// The round trip CLAUDE.md asks for, as one function: a device whose `save` and
/// `load` disagree produces a different chunk the second time round, and this
/// catches that without every test having to spell it out.
fn round_trip<D: Device>(saved: &D, restored: &D, path: &str) {
    let class = saved.class();
    let encode = |d: &D| {
        let mut writer = StateWriter::new(MachineShape::new());
        {
            let mut chunk = writer
                .chunk(path, class.name, class.version)
                .expect("a chunk");
            d.save(&mut chunk).expect("saves");
        }
        writer.to_vec().expect("serialises")
    };
    let bytes = encode(saved);
    let reader = StateReader::new(&bytes).expect("well formed");
    let chunk = reader
        .load(path, class.name, class.version, &Migrations::new())
        .expect("the chunk is there");
    restored.load(&mut chunk.reader()).expect("loads");
    assert_eq!(
        encode(restored),
        bytes,
        "`{}` does not round-trip",
        class.name
    );
}

// ---------------------------------------------------------------------------
// The cartridge header
// ---------------------------------------------------------------------------

#[test]
fn the_header_decides_the_mapper_the_ram_and_the_battery() {
    let rom = synthetic_image(2, 0x00, 0x00, &[0x00]);
    let cart = Cartridge::parse(rom).expect("a valid image");
    assert_eq!(cart.kind().mapper, Mapper::None);
    assert!(!cart.kind().ram);
    assert_eq!(cart.rom_banks(), 2);
    assert_eq!(cart.ram_len(), 0);
    assert!(cart.header_checksum_ok(), "the generator computes it");

    // $13 is MBC3 with RAM and a battery but no clock; $03 in $0149 is 32 KiB.
    let rom = synthetic_image(8, 0x13, 0x03, &[0x00]);
    let cart = Cartridge::parse(rom).expect("a valid image");
    assert_eq!(cart.kind().mapper, Mapper::Mbc3);
    assert!(cart.kind().ram && cart.kind().battery && !cart.kind().rtc);
    assert_eq!(cart.ram_len(), 0x8000);
    assert_eq!(cart.rom_banks(), 8);
}

#[test]
fn an_image_that_disagrees_with_its_own_header_is_refused() {
    // A header claiming eight banks in an image holding two: the mistake that
    // otherwise shows up as a game jumping into nothing.
    let mut rom = synthetic_image(2, 0x00, 0x00, &[0x00]);
    rom[0x0148] = 2;
    let e = Cartridge::parse(rom).expect_err("the size disagrees");
    assert!(alloc::format!("{e}").contains("declares"), "{e}");

    // A cartridge type that was never assigned.
    let mut rom = synthetic_image(2, 0x00, 0x00, &[0x00]);
    rom[0x0147] = 0x77;
    let e = Cartridge::parse(rom).expect_err("no such controller");
    assert!(
        alloc::format!("{e}").contains("0x77") || alloc::format!("{e}").contains("$77"),
        "{e}"
    );

    // Too short to hold a header at all.
    assert!(Cartridge::parse(vec![0u8; 16]).is_err());
}

#[test]
fn a_wrong_header_checksum_is_recorded_rather_than_refused() {
    // The boot ROM would refuse to start this cartridge; rsemu runs it anyway,
    // because a test ROM with a deliberately broken header is still worth
    // running. What matters is that we know.
    let mut rom = synthetic_image(2, 0x00, 0x00, &[0x00]);
    rom[0x014d] ^= 0xff;
    let cart = Cartridge::parse(rom).expect("still parses");
    assert!(!cart.header_checksum_ok());
}

// ---------------------------------------------------------------------------
// Bank switching
// ---------------------------------------------------------------------------

/// Build a cartridge device whose every bank is filled with its own number, so
/// a read says which bank answered.
fn banked_cart(banks: u32, kind: u8, ram: u8) -> GbCart {
    let mut rom = synthetic_image(banks, kind, ram, &[0x00]);
    for bank in 0..banks as usize {
        // Byte $2000 of each bank, chosen because it is past the header.
        rom[bank * 0x4000 + 0x2000] = bank as u8;
    }
    GbCart::new(Cartridge::parse(rom).expect("valid"))
}

/// Read one byte of the `$0000-$7FFF` window.
fn rom_read(cart: &GbCart, offset: u64) -> u8 {
    let region = Device::region(cart, super::cart::ROM_REGION).expect("the ROM window");
    let mut byte = [0u8; 1];
    io(&region)
        .read(offset, &mut byte, MemAttrs::DEFAULT)
        .expect("answers");
    byte[0]
}

/// Write one byte of the `$0000-$7FFF` window — a mapper register.
fn rom_write(cart: &GbCart, offset: u64, value: u8) {
    let region = Device::region(cart, super::cart::ROM_REGION).expect("the ROM window");
    io(&region)
        .write(offset, &[value], MemAttrs::DEFAULT)
        .expect("accepts");
}

#[test]
fn mbc1_cannot_select_bank_zero_in_the_switchable_window() {
    // The quirk every large MBC1 game works around: writing 0 to $2000 selects
    // bank 1, and so do $20, $40 and $60 in the upper groups.
    let cart = banked_cart(4, 0x01, 0x00);
    assert_eq!(cart.rom_bank(), 1, "bank 1 out of reset");
    rom_write(&cart, 0x2000, 0);
    assert_eq!(cart.rom_bank(), 1);
    assert_eq!(rom_read(&cart, 0x6000), 1);
    rom_write(&cart, 0x2000, 2);
    assert_eq!(cart.rom_bank(), 2);
    assert_eq!(rom_read(&cart, 0x6000), 2);
    // Bank 0 still answers at $0000 whatever is selected.
    assert_eq!(rom_read(&cart, 0x2000), 0);
}

#[test]
fn mbc1_advanced_mode_banks_the_low_window_too() {
    // 1 MiB, so the two high bits reach banks $20 and $40. In simple mode the
    // low window is always bank 0; in advanced mode it follows the high bits.
    let cart = banked_cart(64, 0x01, 0x00);
    rom_write(&cart, 0x4000, 1); // the high bits
    assert_eq!(cart.rom_bank_low(), 0, "simple mode pins the low window");
    assert_eq!(cart.rom_bank(), 0x21, "and $20 becomes $21");
    rom_write(&cart, 0x6000, 1); // advanced mode
    assert_eq!(cart.rom_bank_low(), 0x20);
    assert_eq!(rom_read(&cart, 0x2000), 0x20);
}

#[test]
fn mbc5_is_the_first_controller_where_bank_zero_means_bank_zero() {
    let cart = banked_cart(4, 0x19, 0x00);
    rom_write(&cart, 0x2000, 0);
    assert_eq!(cart.rom_bank(), 0);
    assert_eq!(rom_read(&cart, 0x6000), 0);
    rom_write(&cart, 0x2000, 3);
    assert_eq!(rom_read(&cart, 0x6000), 3);
}

#[test]
fn cartridge_ram_answers_only_while_it_is_enabled() {
    let cart = banked_cart(4, 0x03, 0x02); // MBC1, 8 KiB, battery
    let region = Device::region(&cart, super::cart::RAM_REGION).expect("the RAM window");
    let ops = io(&region);
    let mut byte = [0u8; 1];

    // Disabled out of reset: reads are the idle bus and writes go nowhere.
    ops.write(0, &[0x42], MemAttrs::DEFAULT).expect("accepts");
    ops.read(0, &mut byte, MemAttrs::DEFAULT).expect("answers");
    assert_eq!(byte[0], 0xff);

    // $0A in the low nibble is the magic value; nothing else enables it.
    rom_write(&cart, 0x0000, 0x0f);
    assert!(!cart.ram_enabled());
    rom_write(&cart, 0x0000, 0x0a);
    assert!(cart.ram_enabled());
    ops.write(0, &[0x42], MemAttrs::DEFAULT).expect("accepts");
    ops.read(0, &mut byte, MemAttrs::DEFAULT).expect("answers");
    assert_eq!(byte[0], 0x42);
}

#[test]
fn the_mbc3_clock_counts_its_own_crystals_ticks() {
    // $10 is MBC3 with RAM, a battery and the timer.
    let cart = banked_cart(4, 0x10, 0x02);
    rom_write(&cart, 0x0000, 0x0a); // enable RAM and the clock registers
    rom_write(&cart, 0x4000, 0x08); // select the seconds register

    // One second is exactly RTC_HZ ticks of the cartridge's own 32.768 kHz can.
    // No floating point anywhere: the residual is carried in whole ticks.
    Device::advance_to(&cart, RTC_HZ * 90 + RTC_HZ / 2);
    // The program has not latched, so it still reads the old copy.
    let region = Device::region(&cart, super::cart::RAM_REGION).expect("the RAM window");
    let ops = io(&region);
    let mut byte = [0u8; 1];
    ops.read(0, &mut byte, MemAttrs::DEFAULT).expect("answers");
    assert_eq!(byte[0], 0, "nothing latched yet");

    // The latch is a 0-then-1 edge, so a program reading five registers sees
    // one consistent instant.
    rom_write(&cart, 0x6000, 0);
    rom_write(&cart, 0x6000, 1);
    ops.read(0, &mut byte, MemAttrs::DEFAULT).expect("answers");
    assert_eq!(byte[0], 30, "90 seconds is one minute and thirty");
    rom_write(&cart, 0x4000, 0x09); // minutes
    ops.read(0, &mut byte, MemAttrs::DEFAULT).expect("answers");
    assert_eq!(byte[0], 1);

    let rtc = cart.rtc().expect("this cartridge has a clock");
    assert_eq!((rtc.minutes, rtc.seconds), (1, 30));
}

#[test]
fn a_halted_clock_does_not_run() {
    let cart = banked_cart(4, 0x10, 0x02);
    rom_write(&cart, 0x0000, 0x0a);
    rom_write(&cart, 0x4000, 0x0c); // the day-high register, which holds the halt bit
    let region = Device::region(&cart, super::cart::RAM_REGION).expect("RAM");
    let ops = io(&region);
    ops.write(0, &[0x40], MemAttrs::DEFAULT).expect("halts it");
    Device::advance_to(&cart, RTC_HZ * 10);
    assert_eq!(cart.rtc().expect("a clock").seconds, 0);
}

#[test]
fn a_cartridge_round_trips_its_banks_its_ram_and_its_clock() {
    let cart = banked_cart(4, 0x10, 0x02);
    rom_write(&cart, 0x0000, 0x0a);
    rom_write(&cart, 0x2000, 3);
    cart.poke_ram(0x10, 0x5a);
    Device::advance_to(&cart, RTC_HZ * 5);
    let restored = banked_cart(4, 0x10, 0x02);
    round_trip(&cart, &restored, "cart");
    assert_eq!(restored.rom_bank(), 3);
    assert_eq!(restored.peek_ram(0x10), Some(0x5a));
    assert_eq!(restored.rtc().expect("a clock").seconds, 5);
}

#[test]
fn the_cartridge_class_needs_its_media_slot() {
    let e = GbCart::from_props(&Props::new()).expect_err("no rom");
    assert!(alloc::format!("{e}").contains("rom"), "{e}");
}

// ---------------------------------------------------------------------------
// The divider and timer
// ---------------------------------------------------------------------------

#[test]
fn div_is_the_top_byte_of_a_counter_running_at_the_crystal_rate() {
    let timer = GbTimer::new();
    timer.advance_by(255);
    assert_eq!(timer.div(), 0);
    timer.advance_by(1);
    assert_eq!(timer.div(), 1);
    timer.advance_by(256 * 10);
    assert_eq!(timer.div(), 11);
}

#[test]
fn writing_div_resets_all_sixteen_bits() {
    let timer = GbTimer::new();
    timer.advance_by(1000);
    assert_ne!(timer.counter(), 0);
    timer.write_register(0, 0xff);
    assert_eq!(timer.counter(), 0, "the whole counter, not just DIV");
    assert_eq!(timer.div(), 0);
}

#[test]
fn tima_counts_at_the_rate_tac_selects() {
    // The four rates are four *bit positions*, and the order is not monotonic:
    // 00 is the slowest and 01 the fastest (Pan Docs, "Timer and Divider").
    for (tac, period) in [(0u8, 1024u64), (1, 16), (2, 64), (3, 256)] {
        let timer = GbTimer::new();
        timer.write_register(3, 0x04 | tac);
        timer.advance_by(period * 5);
        assert_eq!(timer.tima(), 5, "TAC={tac:#04x}");
    }
}

#[test]
fn a_div_write_can_clock_tima_through_the_falling_edge_detector() {
    // The famous one. With TAC = $05 the selected bit is bit 3, so a counter of
    // 8 has it set; zeroing the counter drops that bit, and a falling edge is a
    // TIMA increment however recently the last one was.
    let timer = GbTimer::new();
    timer.write_register(3, 0x05);
    timer.advance_by(8);
    assert_eq!(timer.tima(), 0, "no edge yet — the bit only just went up");
    timer.write_register(0, 0);
    assert_eq!(timer.tima(), 1, "the DIV write was the falling edge");

    // And with the bit already clear, a DIV write does nothing.
    let timer = GbTimer::new();
    timer.write_register(3, 0x05);
    timer.advance_by(4);
    timer.write_register(0, 0);
    assert_eq!(timer.tima(), 0);
}

#[test]
fn disabling_the_timer_while_the_selected_bit_is_high_also_clocks_it() {
    // Same detector, different input: clearing TAC's enable bit makes the
    // detector's input fall, and the increment happens for the same reason.
    let timer = GbTimer::new();
    timer.write_register(3, 0x05);
    timer.advance_by(8);
    timer.write_register(3, 0x01); // enable cleared, rate unchanged
    assert_eq!(timer.tima(), 1);
}

#[test]
fn an_overflow_reloads_from_tma_one_machine_cycle_late() {
    let timer = GbTimer::new();
    timer.write_register(2, 0x37); // TMA
    timer.write_register(1, 0xff); // TIMA, one from the top
    timer.write_register(3, 0x05); // enabled, bit 3
    timer.advance_by(16);
    // Inside the reload window TIMA genuinely reads zero: the counter has
    // wrapped and the reload has not happened yet.
    assert_eq!(timer.tima(), 0);
    timer.advance_by(4);
    assert_eq!(timer.tima(), 0x37, "TMA arrived");
}

#[test]
fn writing_tima_inside_the_reload_window_cancels_the_reload() {
    let timer = GbTimer::new();
    timer.write_register(2, 0x37);
    timer.write_register(1, 0xff);
    timer.write_register(3, 0x05);
    timer.advance_by(16);
    assert_eq!(timer.tima(), 0);
    timer.write_register(1, 0x11);
    timer.advance_by(8);
    assert_eq!(timer.tima(), 0x11, "the write won, not TMA");
}

/// The four clocks after `TIMA` wraps are not one window but two things: the
/// *delay*, during which a write to `TIMA` cancels the reload, and the single
/// clock on which the reload actually happens, on which `TIMA` is being driven
/// from `TMA` and a write to it loses (Pan Docs, *Timer Obscure Behaviour*).
#[test]
fn a_tima_write_on_the_reload_clock_itself_is_ignored() {
    let timer = GbTimer::new();
    timer.write_register(2, 0x37); // TMA
    timer.write_register(1, 0xff); // TIMA
    timer.write_register(3, 0x05); // enabled, bit 3
    timer.advance_by(20); // the overflow at 16, then the reload at 20
    assert_eq!(timer.tima(), 0x37, "the reload has just happened");
    timer.write_register(1, 0x11);
    assert_eq!(timer.tima(), 0x37, "and the write on that clock is ignored");
    // One clock later it is an ordinary write again.
    timer.advance_by(1);
    timer.write_register(1, 0x11);
    assert_eq!(timer.tima(), 0x11);
}

/// And a write to `TMA` on that clock lands in `TIMA` as well, because what the
/// reload copies is whatever `TMA` holds when it happens.
#[test]
fn a_tma_write_on_the_reload_clock_is_what_gets_loaded() {
    let timer = GbTimer::new();
    timer.write_register(2, 0x37);
    timer.write_register(1, 0xff);
    timer.write_register(3, 0x05);
    timer.advance_by(20);
    assert_eq!(timer.tima(), 0x37);
    timer.write_register(2, 0x42);
    assert_eq!(timer.tima(), 0x42, "TIMA followed TMA");
    // One clock later `TMA` is an ordinary register again.
    timer.advance_by(1);
    timer.write_register(2, 0x99);
    assert_eq!(timer.tima(), 0x42, "TIMA is left where it was");
}

#[test]
fn the_next_event_is_never_in_the_past_and_never_further_than_a_div_step() {
    // What makes a mid-quantum read correct: between two of this device's own
    // events nothing it publishes changes.
    let timer = GbTimer::new();
    for tac in [0u8, 1, 2, 3, 4, 5, 6, 7] {
        timer.write_register(3, tac);
        for _ in 0..40 {
            let now = Device::current_tick(&timer);
            let next = Device::next_event_tick(&timer).expect("always something");
            assert!(next > now, "TAC={tac:#04x}: {next} <= {now}");
            assert!(next - now <= 256, "TAC={tac:#04x}: {} clocks", next - now);
            timer.advance_by(3);
        }
    }
}

#[test]
fn a_reset_leaves_the_divider_where_a_boot_rom_would() {
    // `new` is the honest power-on state; `reset` is what a machine performs,
    // and on a console that means "after the boot ROM has run". Pan Docs gives
    // `DIV` as `$AB` at the handoff, and a game that seeds itself from `DIV`
    // gets the same number every run without this.
    let timer = GbTimer::new();
    assert_eq!(timer.counter(), 0, "power-on is zero");
    timer.reset(ResetKind::Cold);
    assert_eq!(timer.div(), 0xab);
    assert_eq!(timer.counter(), super::timer::POST_BOOT_COUNTER);
    // And the edge detectors start from the counter they are now looking at,
    // not from zero: the first tick after a reset must not look like an edge.
    timer.write_register(3, 0x04);
    timer.advance_by(1);
    assert_eq!(timer.tima(), 0);
}

#[test]
fn a_timer_round_trips() {
    let timer = GbTimer::new();
    timer.write_register(3, 0x05);
    timer.write_register(2, 0x42);
    timer.advance_by(1234);
    let restored = GbTimer::new();
    round_trip(&timer, &restored, "timer");
    assert_eq!(restored.counter(), timer.counter());
    assert_eq!(restored.tima(), timer.tima());
    assert_eq!(restored.tma(), 0x42);
}

// ---------------------------------------------------------------------------
// The LCD controller
// ---------------------------------------------------------------------------

/// A controller with the LCD on and the given `STAT` enables.
fn lcd(stat_bits: u8) -> GbPpu {
    let ppu = GbPpu::new();
    ppu.write_register(0x01, stat_bits);
    ppu
}

#[test]
fn a_line_walks_mode_two_then_three_then_zero() {
    let ppu = lcd(0);
    assert_eq!(ppu.mode(), Mode::OamScan);
    ppu.advance_by(ppu::OAM_SCAN_DOTS - 1);
    assert_eq!(ppu.mode(), Mode::OamScan, "80 dots of it");
    ppu.advance_by(1);
    assert_eq!(ppu.mode(), Mode::Drawing);
    ppu.advance_by(ppu::MODE3_MIN_DOTS);
    assert_eq!(ppu.mode(), Mode::HBlank);
    ppu.advance_by(ppu::DOTS_PER_LINE - ppu::OAM_SCAN_DOTS - ppu::MODE3_MIN_DOTS);
    assert_eq!(ppu.position(), (1, 0));
    assert_eq!(ppu.mode(), Mode::OamScan, "and the next line begins");
}

#[test]
fn vblank_starts_on_line_144_and_the_frame_ends_on_154() {
    let ppu = lcd(0);
    ppu.advance_by(ppu::DOTS_PER_LINE * 144);
    assert_eq!(ppu.position().0, 144);
    assert_eq!(ppu.mode(), Mode::VBlank);
    ppu.advance_by(ppu::DOTS_PER_LINE * 10);
    assert_eq!(ppu.position(), (0, 0));
    assert_eq!(ppu.frame(), 1);
    assert_eq!(ppu.dots(), ppu::DOTS_PER_FRAME);
}

#[test]
fn ly_reads_zero_for_most_of_line_153() {
    // The quirk every accuracy suite tests: `LY` reads 153 for four dots and
    // then 0, while the frame has not ended.
    let ppu = lcd(0);
    ppu.advance_by(ppu::DOTS_PER_LINE * 153);
    assert_eq!(ppu.read_register(0x04), 153);
    ppu.advance_by(4);
    assert_eq!(ppu.read_register(0x04), 0, "but the frame is still running");
    assert_eq!(ppu.frame(), 0);
    ppu.advance_by(ppu::DOTS_PER_LINE - 4);
    assert_eq!(ppu.frame(), 1);
}

#[test]
fn mode_three_is_extended_by_scroll_the_window_and_objects() {
    // A bare line is the minimum.
    let ppu = lcd(0);
    ppu.advance_by(ppu::OAM_SCAN_DOTS);
    assert_eq!(ppu.mode(), Mode::Drawing);
    ppu.advance_by(ppu::MODE3_MIN_DOTS - 1);
    assert_eq!(ppu.mode(), Mode::Drawing);
    ppu.advance_by(1);
    assert_eq!(ppu.mode(), Mode::HBlank);

    // `SCX & 7` discards that many pixels at the left edge, and the controller
    // pays for them.
    let ppu = lcd(0);
    ppu.write_register(0x03, 5); // SCX
    ppu.advance_by(ppu::OAM_SCAN_DOTS + ppu::MODE3_MIN_DOTS + 4);
    assert_eq!(ppu.mode(), Mode::Drawing, "five dots longer");
    ppu.advance_by(1);
    assert_eq!(ppu.mode(), Mode::HBlank);

    // One object on the line costs at least six more.
    let ppu = lcd(0);
    ppu.write_register(0x00, lcdc::LCD_ENABLE | lcdc::BG_ENABLE | lcdc::OBJ_ENABLE);
    ppu.poke_oam(0, 16); // y = 16 puts it on line 0
    ppu.poke_oam(1, 32); // x
    ppu.advance_by(ppu::OAM_SCAN_DOTS + ppu::MODE3_MIN_DOTS + 5);
    assert_eq!(ppu.mode(), Mode::Drawing, "an object extends mode 3");
}

#[test]
fn video_ram_reads_as_ff_during_mode_three_and_not_otherwise() {
    let ppu = lcd(0);
    ppu.poke_vram(0, 0x5a);
    let region = Device::region(&ppu, ppu::VRAM_REGION).expect("VRAM");
    let ops = io(&region);
    let mut byte = [0u8; 1];

    // Mode 2: readable.
    ops.read(0, &mut byte, MemAttrs::DEFAULT).expect("answers");
    assert_eq!(byte[0], 0x5a);

    // The gate follows the mode the CPU sees, which is `MODE_VISIBLE_LAG`
    // behind the one the controller is in — so mode 3 blocks video memory four
    // dots after the controller enters it.
    ppu.advance_by(ppu::OAM_SCAN_DOTS);
    assert_eq!(ppu.mode(), Mode::Drawing);
    ops.read(0, &mut byte, MemAttrs::DEFAULT).expect("answers");
    assert_eq!(byte[0], 0x5a, "not blocked for another machine cycle");
    ppu.advance_by(ppu::MODE_VISIBLE_LAG);
    ops.read(0, &mut byte, MemAttrs::DEFAULT).expect("answers");
    assert_eq!(byte[0], 0xff, "blocked");
    // A write is dropped rather than faulting: the write really does go nowhere.
    ops.write(0, &[0x11], MemAttrs::DEFAULT).expect("accepts");
    assert_eq!(ppu.peek_vram(0), 0x5a);

    // A debugger sees through the blocking, because a monitor showing the tile
    // map during mode 3 should show the tile map (invariant 5).
    ops.read(0, &mut byte, MemAttrs::DEBUG).expect("answers");
    assert_eq!(byte[0], 0x5a);
}

#[test]
fn object_memory_is_blocked_during_both_the_scan_and_the_drawing() {
    let ppu = lcd(0);
    ppu.poke_oam(0, 0x5a);
    let region = Device::region(&ppu, ppu::OAM_REGION).expect("OAM");
    let ops = io(&region);
    let mut byte = [0u8; 1];
    assert_eq!(ppu.mode(), Mode::OamScan);
    // One machine cycle behind the controller, so the gate closes four dots in.
    ppu.advance_by(ppu::MODE_VISIBLE_LAG);
    ops.read(0, &mut byte, MemAttrs::DEFAULT).expect("answers");
    assert_eq!(byte[0], 0xff);
    ppu.advance_by(ppu::OAM_SCAN_DOTS + ppu::MODE3_MIN_DOTS);
    assert_eq!(ppu.mode(), Mode::HBlank);
    ops.read(0, &mut byte, MemAttrs::DEFAULT).expect("answers");
    assert_eq!(byte[0], 0x5a);
}

/// The mode the CPU reads and the mode the controller is in are four dots
/// apart, and the gates follow the one the CPU reads.
///
/// Measured rather than assumed — `ppu::MODE_VISIBLE_LAG` carries the argument
/// and the Gekkio tests that pin each end of it.
#[test]
fn the_mode_a_program_reads_is_one_machine_cycle_behind_the_controllers() {
    let ppu = lcd(0);
    let oam = Device::region(&ppu, ppu::OAM_REGION).expect("OAM");
    let oam = io(&oam);
    let mut byte = [0u8; 1];
    let read_mode = |ppu: &GbPpu| ppu.read_register(0x01) & 3;

    // Dot 0 of line 0: the controller has entered the object scan, and what a
    // program reads is still the vertical blanking it just left.
    assert_eq!(ppu.mode(), Mode::OamScan);
    assert_eq!(read_mode(&ppu), Mode::VBlank.bits());
    oam.read(0, &mut byte, MemAttrs::DEFAULT).expect("answers");
    assert_eq!(byte[0], 0x00, "and object memory still answers");

    ppu.advance_by(ppu::MODE_VISIBLE_LAG);
    assert_eq!(read_mode(&ppu), Mode::OamScan.bits());
    oam.read(0, &mut byte, MemAttrs::DEFAULT).expect("answers");
    assert_eq!(byte[0], 0xff, "the gate follows the mode the program reads");

    // And the same four dots at the other two boundaries of the line.
    ppu.advance_by(ppu::OAM_SCAN_DOTS - ppu::MODE_VISIBLE_LAG);
    assert_eq!(ppu.mode(), Mode::Drawing);
    assert_eq!(read_mode(&ppu), Mode::OamScan.bits());
    ppu.advance_by(ppu::MODE_VISIBLE_LAG);
    assert_eq!(read_mode(&ppu), Mode::Drawing.bits());

    ppu.advance_by(ppu::MODE3_MIN_DOTS - ppu::MODE_VISIBLE_LAG);
    assert_eq!(ppu.mode(), Mode::HBlank);
    assert_eq!(read_mode(&ppu), Mode::Drawing.bits());
    ppu.advance_by(ppu::MODE_VISIBLE_LAG);
    assert_eq!(read_mode(&ppu), Mode::HBlank.bits());
}

#[test]
fn switching_the_lcd_off_parks_ly_and_unblocks_everything() {
    let ppu = lcd(0);
    ppu.advance_by(ppu::DOTS_PER_LINE * 3 + 100);
    assert_eq!(ppu.position().0, 3);
    ppu.write_register(0x00, 0);
    assert_eq!(ppu.read_register(0x04), 0, "LY parks at zero");
    assert_eq!(ppu.mode(), Mode::HBlank, "and the mode reads as 0");
    assert_eq!(
        Device::next_event_tick(&ppu),
        None,
        "nothing changes while it is off"
    );
    // Advancing does nothing but move the clock.
    let before = ppu.dots();
    ppu.advance_by(10_000);
    assert_eq!(ppu.dots(), before + 10_000);
    assert_eq!(ppu.position().0, 0);
}

#[test]
fn the_stat_line_is_the_or_of_whatever_is_enabled() {
    // With only the mode-0 interrupt enabled the line follows H-blank.
    let ppu = lcd(stat::HBLANK_INT);
    assert_eq!(ppu.mode(), Mode::OamScan);
    ppu.advance_by(ppu::MODE_VISIBLE_LAG);
    assert_eq!(ppu.read_register(0x01) & 3, 2);
    ppu.advance_by(ppu::OAM_SCAN_DOTS + ppu::MODE3_MIN_DOTS);
    assert_eq!(ppu.read_register(0x01) & 3, 0);

    // The coincidence flag is in `STAT` whether or not its interrupt is on.
    let ppu = lcd(0);
    ppu.write_register(0x05, 2); // LYC
    assert_eq!(ppu.read_register(0x01) & stat::LYC_EQUAL, 0);
    ppu.advance_by(ppu::DOTS_PER_LINE * 2);
    assert_eq!(ppu.read_register(0x01) & stat::LYC_EQUAL, stat::LYC_EQUAL);
    // Bit 7 is not implemented and reads as one.
    assert_eq!(ppu.read_register(0x01) & 0x80, 0x80);
}

#[test]
fn a_flat_background_renders_the_palette_shade_it_asks_for() {
    let ppu = lcd(0);
    // Tile 0, every pixel colour 3: both bitplanes all ones.
    for i in 0..16u64 {
        ppu.poke_vram(i, 0xff);
    }
    // The whole $9800 map is tile 0 already (VRAM is zeroed).
    // BGP: map colour 3 to shade 1 and leave the rest.
    ppu.write_register(0x07, 0b01_00_00_00);
    ppu.advance_by(ppu::DOTS_PER_LINE);
    assert_eq!(ppu.pixel(0, 0), Some(1));
    assert_eq!(ppu.pixel(159, 0), Some(1));
    assert_eq!(ppu.pixel(0, 143), Some(0), "not drawn yet");

    // With LCDC bit 0 clear the background is blank whatever the tiles say.
    let ppu = lcd(0);
    for i in 0..16u64 {
        ppu.poke_vram(i, 0xff);
    }
    ppu.write_register(0x07, 0b01_00_00_00);
    ppu.write_register(0x00, lcdc::LCD_ENABLE);
    ppu.advance_by(ppu::DOTS_PER_LINE);
    assert_eq!(ppu.pixel(0, 0), Some(0));
}

#[test]
fn an_object_draws_over_the_background_and_colour_zero_is_transparent() {
    let ppu = lcd(0);
    // Tile 1: the left four pixels of every row are colour 1, the rest colour 0.
    for row in 0..8u64 {
        ppu.poke_vram(16 + row * 2, 0xf0);
        ppu.poke_vram(16 + row * 2 + 1, 0x00);
    }
    ppu.write_register(
        0x00,
        lcdc::LCD_ENABLE | lcdc::BG_ENABLE | lcdc::TILE_DATA | lcdc::OBJ_ENABLE,
    );
    // Object 0 at the top left, using tile 1 and palette OBP0.
    ppu.poke_oam(0, 16);
    ppu.poke_oam(1, 8);
    ppu.poke_oam(2, 1);
    ppu.poke_oam(3, 0);
    // OBP0 maps colour 1 to shade 3.
    ppu.write_register(0x08, 0b00_00_11_00);
    ppu.advance_by(ppu::DOTS_PER_LINE);
    assert_eq!(ppu.pixel(0, 0), Some(3), "the object's colour 1");
    assert_eq!(ppu.pixel(4, 0), Some(0), "its colour 0 is transparent");
}

#[test]
fn an_oam_transfer_copies_a_page_over_160_machine_cycles() {
    use crate::core::space::{AddressSpace, RamStore, Region};

    let ppu = GbPpu::new();
    let space = Arc::new(AddressSpace::new("cpubus", 16));
    let ram = Arc::new(RamStore::new(0x10000));
    for i in 0..160u64 {
        ram.write_u8(0xc000 + i, i as u8).expect("in range");
    }
    space
        .topology()
        .map(Region::ram("ram", ram), 0)
        .expect("maps");
    ppu.attach_space(space);

    ppu.write_register(0x06, 0xc0);
    assert_eq!(ppu.read_register(0x06), 0xc0, "the register reads back");
    // Two machine cycles pass before the transfer starts — Gekkio's
    // `oam_dma_start`, and `ppu::DMA_START_DELAY`. So after one, nothing.
    ppu.advance_by(4);
    assert_eq!(
        ppu.peek_oam(0),
        0,
        "the cycle after the write moves nothing"
    );
    ppu.advance_by(4);
    assert_eq!(ppu.peek_oam(0), 0, "and the first byte moves on the second");
    // `peek_oam` looks past the blocking, so read the byte the transfer put
    // there rather than the `$FF` a guest would see.
    ppu.advance_by(4 * 5);
    assert_eq!(ppu.peek_oam(5), 5, "five more cycles, five more bytes");
    assert_eq!(ppu.peek_oam(6), 0, "and not one more");
    ppu.advance_by(4 * 154);
    assert_eq!(ppu.peek_oam(159), 159);
    // And it stops there rather than running off the end of OAM.
    ppu.advance_by(4 * 160);
    assert_eq!(ppu.peek_oam(159), 159);
}

/// The window in which object memory answers `$FF` is `[W+2, W+161]` for a
/// write on machine cycle `W` — the two-cycle start delay, then one cycle per
/// byte, with the cycle the last byte moves on still blocked.
///
/// Gekkio's `oam_dma_timing` is exactly this assertion: it aligns one read on
/// `W+161` and expects `$FF`, and another on `W+162` and expects the byte.
#[test]
fn object_memory_is_blocked_for_the_transfers_hundred_and_sixty_cycles() {
    use crate::core::space::{AddressSpace, RamStore, Region};

    let ppu = GbPpu::new();
    let space = Arc::new(AddressSpace::new("cpubus", 16));
    let ram = Arc::new(RamStore::new(0x10000));
    for i in 0..160u64 {
        ram.write_u8(0xc000 + i, 0x42).expect("in range");
    }
    space
        .topology()
        .map(Region::ram("ram", ram), 0)
        .expect("maps");
    ppu.attach_space(space);
    let oam = Device::region(&ppu, super::ppu::OAM_REGION).expect("the region");
    let oam = io(&oam);
    let read = || {
        let mut byte = [0u8];
        oam.read(0, &mut byte, MemAttrs::DEFAULT).expect("answers");
        byte[0]
    };

    // Switch the LCD off, so mode 0 is reported and only the transfer can
    // block. Gekkio's own ROM does the same thing for the same reason.
    ppu.write_register(0x00, 0);
    ppu.write_register(0x06, 0xc0);
    assert_eq!(read(), 0x00, "the write cycle itself is not blocked");
    ppu.advance_by(4);
    assert_eq!(read(), 0x00, "nor the one after it");
    ppu.advance_by(4);
    assert_eq!(read(), 0xff, "the transfer has the bus from W+2");
    // On to the cycle the last byte moves on, W+161.
    ppu.advance_by(4 * 159);
    assert_eq!(read(), 0xff, "still blocked on the last byte's cycle");
    ppu.advance_by(4);
    assert_eq!(read(), 0x42, "and readable on the next");
}

/// A write to `$FF46` while a transfer is running does not stop it: the old one
/// keeps the bus for the two cycles before the new one takes over, so object
/// memory never becomes readable in between (Gekkio, `oam_dma_restart`).
#[test]
fn restarting_a_transfer_leaves_no_readable_cycle_in_between() {
    use crate::core::space::{AddressSpace, RamStore, Region};

    let ppu = GbPpu::new();
    let space = Arc::new(AddressSpace::new("cpubus", 16));
    let ram = Arc::new(RamStore::new(0x10000));
    for i in 0..160u64 {
        ram.write_u8(0xc000 + i, 0x42).expect("in range");
    }
    space
        .topology()
        .map(Region::ram("ram", ram), 0)
        .expect("maps");
    ppu.attach_space(space);
    let oam = Device::region(&ppu, super::ppu::OAM_REGION).expect("the region");
    let oam = io(&oam);
    let read = || {
        let mut byte = [0u8];
        oam.read(0, &mut byte, MemAttrs::DEFAULT).expect("answers");
        byte[0]
    };

    ppu.write_register(0x00, 0);
    ppu.write_register(0x06, 0xc0);
    // Ten cycles in, so the first transfer is well under way.
    ppu.advance_by(4 * 10);
    assert_eq!(read(), 0xff);
    ppu.write_register(0x06, 0xc0);
    // The two cycles that belong to the transfer being displaced.
    assert_eq!(read(), 0xff, "the restart cycle itself");
    ppu.advance_by(4);
    assert_eq!(read(), 0xff, "and the one after it");
    ppu.advance_by(4);
    assert_eq!(read(), 0xff, "then the new transfer, for its full 160");
    // The new transfer's last byte lands 159 cycles after it started, which is
    // 161 after the restart — ten cycles later than the first one would have.
    ppu.advance_by(4 * 159);
    assert_eq!(read(), 0xff, "the last byte's cycle is still blocked");
    ppu.advance_by(4);
    assert_eq!(read(), 0x42);
}

#[test]
fn an_lcd_controller_round_trips() {
    let ppu = lcd(stat::LYC_INT | stat::HBLANK_INT);
    ppu.poke_vram(0x100, 0x5a);
    ppu.poke_oam(4, 0x42);
    ppu.write_register(0x03, 7);
    ppu.advance_by(ppu::DOTS_PER_LINE * 5 + 123);
    let restored = GbPpu::new();
    round_trip(&ppu, &restored, "ppu");
    assert_eq!(restored.position(), ppu.position());
    assert_eq!(restored.peek_vram(0x100), 0x5a);
    assert_eq!(restored.peek_oam(4), 0x42);
    assert_eq!(
        Device::next_event_tick(&restored),
        Device::next_event_tick(&ppu),
        "the derived next event follows the load"
    );
}

// ---------------------------------------------------------------------------
// The joypad
// ---------------------------------------------------------------------------

#[test]
fn the_joypad_is_a_matrix_and_zero_means_pressed() {
    let pad = GbJoypad::new();
    let region = Device::region(&pad, super::joypad::REGISTER_REGION).expect("the register");
    let ops = io(&region);

    // Nothing held: the low nibble is all ones however the rows are selected.
    assert_eq!(pad.read() & 0x0f, 0x0f);

    // Both select bits high means *neither* row is selected, and then nothing
    // can pull a column low.
    ops.write(0, &[0x30], MemAttrs::DEFAULT).expect("accepts");
    pad.pad().set_pressed(Button::Start, true);
    assert_eq!(pad.read() & 0x0f, 0x0f, "neither row is selected");

    // Bit 5 low selects the action buttons. Start is bit 3 of that row.
    ops.write(0, &[0x10], MemAttrs::DEFAULT).expect("accepts");
    assert_eq!(pad.read() & 0x0f, 0b0111);

    // Bit 4 low selects the directions instead, and Start is not in that row.
    ops.write(0, &[0x20], MemAttrs::DEFAULT).expect("accepts");
    assert_eq!(pad.read() & 0x0f, 0x0f);
    pad.pad().set_pressed(Button::Right, true);
    assert_eq!(pad.read() & 0x0f, 0b1110);

    // Both low at once: the two rows are wired together, and a button held in
    // either pulls its column down. This is the state the boot ROM leaves
    // behind — `$FF00` reads `$CF` on a DMG with nothing pressed.
    ops.write(0, &[0x00], MemAttrs::DEFAULT).expect("accepts");
    assert_eq!(pad.read() & 0x0f, 0b0110);
    // Bits 7 and 6 are not implemented.
    assert_eq!(pad.read() & 0xc0, 0xc0);
}

#[test]
fn a_joypad_round_trips_its_buttons_and_its_select_lines() {
    let pad = GbJoypad::new();
    pad.pad().set_buttons(0b1010_0101);
    let region = Device::region(&pad, super::joypad::REGISTER_REGION).expect("the register");
    io(&region)
        .write(0, &[0x10], MemAttrs::DEFAULT)
        .expect("accepts");
    let restored = GbJoypad::new();
    round_trip(&pad, &restored, "pad");
    assert_eq!(restored.buttons(), 0b1010_0101);
    assert_eq!(restored.read(), pad.read());
}

#[test]
fn buttons_are_named_both_ways() {
    for button in Button::ALL {
        assert_eq!(Button::from_name(button.name()), Some(button));
    }
    assert_eq!(Button::from_name("turbo"), None);
}

// ---------------------------------------------------------------------------
// The serial link
// ---------------------------------------------------------------------------

#[test]
fn a_transfer_takes_its_time_and_shifts_in_ones() {
    let link = GbSerial::new();
    let region = Device::region(&link, super::serial::REGISTER_REGION).expect("the registers");
    let ops = io(&region);
    ops.write(0, b"A", MemAttrs::DEFAULT).expect("SB");
    ops.write(1, &[0x81], MemAttrs::DEFAULT).expect("SC: start");
    // The transcript records the byte as it starts to go out.
    assert_eq!(link.transcript(), vec![b'A']);
    // But the transfer is still in flight.
    assert_eq!(link.control() & 0x80, 0x80);
    link.advance_to(TRANSFER_CLOCKS - 1);
    assert_eq!(link.control() & 0x80, 0x80);
    link.advance_to(TRANSFER_CLOCKS);
    assert_eq!(link.control() & 0x80, 0, "and now it is done");
    assert_eq!(link.data(), 0xff, "nothing on the other end of the cable");
}

#[test]
fn an_external_clock_transfer_never_completes_on_its_own() {
    // Which is what hardware does with no cable: the other console supplies the
    // clock, and there is no other console.
    let link = GbSerial::new();
    let region = Device::region(&link, super::serial::REGISTER_REGION).expect("the registers");
    let ops = io(&region);
    ops.write(1, &[0x80], MemAttrs::DEFAULT).expect("external");
    assert_eq!(Device::next_event_tick(&link), None);
    link.advance_to(TRANSFER_CLOCKS * 10);
    assert_eq!(link.control() & 0x80, 0x80, "still waiting");
}

#[test]
fn a_serial_link_round_trips_its_transcript() {
    let link = GbSerial::new();
    let region = Device::region(&link, super::serial::REGISTER_REGION).expect("the registers");
    let ops = io(&region);
    let mut sent = 0u64;
    for byte in b"hi" {
        ops.write(0, &[*byte], MemAttrs::DEFAULT).expect("SB");
        ops.write(1, &[0x81], MemAttrs::DEFAULT).expect("SC");
        sent += TRANSFER_CLOCKS;
        link.advance_to(sent);
    }
    assert_eq!(link.transcript_text(), "hi");
    let restored = GbSerial::new();
    round_trip(&link, &restored, "link");
    assert_eq!(restored.transcript_text(), "hi");
}

// ---------------------------------------------------------------------------
// The sound unit
// ---------------------------------------------------------------------------

#[test]
fn the_sound_unit_ignores_every_register_while_it_is_powered_down() {
    let apu = GbApu::new();
    assert!(!apu.powered());
    apu.write_register(0x02, 0xf0); // NR12
    assert_eq!(apu.read_register(0x02), 0x00, "the write went nowhere");
    apu.write_register(0x16, 0x80); // NR52: power on
    assert!(apu.powered());
    apu.write_register(0x02, 0xf0);
    assert_eq!(apu.read_register(0x02), 0xf0);
}

#[test]
fn powering_down_zeroes_everything_except_the_wave_ram() {
    let apu = GbApu::new();
    apu.write_register(0x16, 0x80);
    apu.write_register(0x14, 0x77); // NR50
    apu.write_register(0x20, 0xab); // wave RAM
    apu.write_register(0x16, 0x00);
    assert_eq!(apu.read_register(0x14), 0x00);
    assert_eq!(apu.read_register(0x20), 0xab, "the waveform survives");
}

#[test]
fn a_channel_reports_itself_in_nr52_until_its_length_runs_out() {
    let apu = GbApu::new();
    apu.write_register(0x16, 0x80); // power
    apu.write_register(0x07, 0xf0); // NR22: full volume, DAC on
    apu.write_register(0x06, 0x3f); // NR21: length 63, so one step to go
    apu.write_register(0x09, 0xc0); // NR24: trigger with length enabled
    assert_eq!(apu.status() & 0x02, 0x02, "channel 2 is on");
    // The length counter is stepped by the frame sequencer's even steps, and
    // the sequencer is clocked from the divider — not from anything of its own.
    apu.step_frame_sequencer();
    assert_eq!(apu.status() & 0x02, 0x00, "and now it is not");
}

#[test]
fn a_dac_that_is_switched_off_switches_its_channel_off_with_it() {
    let apu = GbApu::new();
    apu.write_register(0x16, 0x80);
    apu.write_register(0x07, 0xf0);
    apu.write_register(0x09, 0x80); // trigger
    assert_eq!(apu.status() & 0x02, 0x02);
    // The top five bits of NRx2 drive the DAC directly: all zero and it is off.
    apu.write_register(0x07, 0x07);
    assert_eq!(apu.status() & 0x02, 0x00);
}

#[test]
fn the_frame_sequencer_walks_its_eight_steps() {
    let apu = GbApu::new();
    apu.write_register(0x16, 0x80);
    for expected in 1..=8u8 {
        apu.step_frame_sequencer();
        assert_eq!(apu.frame_step(), expected % 8);
    }
}

#[test]
fn samples_come_out_at_a_power_of_two_divisor_of_the_crystal() {
    let apu = GbApu::new();
    apu.set_recording(true);
    apu.write_register(0x16, 0x80);
    apu.advance_to(super::apu::SAMPLE_DIVISOR * 100);
    assert_eq!(apu.queued_samples(), 100);
    assert_eq!(apu.take_samples().len(), 100);
    assert_eq!(apu.queued_samples(), 0);
    // 4194304 / 128 is exact — no rounding anywhere in the time path.
    assert_eq!(super::apu::SAMPLE_RATE, 32_768);
}

#[test]
fn a_sound_unit_round_trips() {
    let apu = GbApu::new();
    apu.write_register(0x16, 0x80);
    apu.write_register(0x07, 0xf0);
    apu.write_register(0x08, 0x55);
    apu.write_register(0x09, 0x87);
    apu.write_register(0x22, 0xab);
    apu.advance_to(5000);
    let restored = GbApu::new();
    round_trip(&apu, &restored, "apu");
    assert_eq!(restored.status(), apu.status());
    assert_eq!(restored.read_register(0x22), 0xab);
}

// ---------------------------------------------------------------------------
// The classes themselves
// ---------------------------------------------------------------------------

#[test]
fn every_class_registers_binds_and_has_a_schema() {
    let mut registry = crate::core::Registry::new();
    super::register(&mut registry).expect("no collisions");
    let mut bindings = crate::machine::Bindings::new();
    super::bind(&mut bindings).expect("no collisions");
    let schemas = super::schemas();
    let names: Vec<&str> = schemas.iter().map(|s| s.class.as_str()).collect();
    for class in [
        "gb.cart",
        "gb.ppu",
        "gb.timer",
        "gb.apu",
        "gb.joypad",
        "gb.serial",
    ] {
        assert!(registry.get(class).is_some(), "{class} is not registered");
        assert!(bindings.get(class).is_some(), "{class} is not bound");
        assert!(names.contains(&class), "{class} has no schema");
    }
}

#[test]
fn every_device_resets_to_a_documented_state() {
    let devices: Vec<alloc::boxed::Box<dyn Device>> = vec![
        alloc::boxed::Box::new(GbPpu::new()),
        alloc::boxed::Box::new(GbTimer::new()),
        alloc::boxed::Box::new(GbApu::new()),
        alloc::boxed::Box::new(GbJoypad::new()),
        alloc::boxed::Box::new(GbSerial::new()),
    ];
    for device in devices {
        device.reset(ResetKind::Cold);
        device.reset(ResetKind::Warm);
        device.reset(ResetKind::Bus);
    }
}

// ---------------------------------------------------------------------------
// Reset does not rewind a clock domain
// ---------------------------------------------------------------------------

/// A lazily-advanced device's tick is the **clock domain's** position, not
/// state of its own, and `Machine::reset` resets devices without rewinding
/// domains. A device that zeroes its tick therefore claims to be at the
/// beginning of time while the forest stands wherever it stands, and the very
/// next catch-up is asked to simulate every tick since power-on in one call —
/// a hang on a mid-run reset, and a wildly wrong frame if it survives.
///
/// The nasty part is that it is invisible to a test that resets a *fresh*
/// device, because there the domain really is at zero. So this one advances
/// first, and it covers every lazily-advanced device on the console rather
/// than the one the bug was found in.
#[test]
fn resetting_a_running_device_leaves_its_clock_where_the_domain_put_it() {
    fn check<D: Device>(device: &D, name: &str, advance: u64) {
        // `advance_to` is the catch-up entry point the scheduler uses.
        Device::advance_to(device, advance);
        let before = Device::current_tick(device);
        assert_eq!(before, advance, "{name}: did not advance");
        Device::reset(device, ResetKind::Cold);
        assert_eq!(
            Device::current_tick(device),
            before,
            "{name}: reset rewound the clock domain"
        );
        // And the device's own next event is still in the future, so catch-up
        // makes progress rather than stalling or replaying.
        if let Some(next) = Device::next_event_tick(device) {
            assert!(
                next > before,
                "{name}: the next event is at {next}, not after {before}"
            );
        }
    }

    check(&GbPpu::new(), "gb.ppu", 12_345);
    check(&GbTimer::new(), "gb.timer", 12_345);
    check(&GbSerial::new(), "gb.serial", 12_345);
    check(&GbApu::new(), "gb.apu", 12_345);
    // $10 is MBC3 with a real-time clock, which is what makes the cartridge
    // lazily advanced at all.
    let cart = GbCart::new(
        Cartridge::parse(synthetic_image(2, 0x10, 0x02, &[0x00])).expect("a valid image"),
    );
    check(&cart, "gb.cart", 12_345);
}

/// No boot ROM writes a mapper register, so a cartridge's entry point at
/// `$0100` runs with whatever the controller came up holding — and every image
/// expects bank 1 at `$4000` there. MBC1, MBC2 and MBC3 get it from their
/// "a written zero reads as one" rule; MBC5 has no such rule, so the power-on
/// value has to be right on its own.
#[test]
fn a_cartridge_powers_on_with_bank_one_at_4000_even_on_mbc5() {
    // $19 is MBC5 with no RAM; $02 in $0148 is four banks.
    let mut rom = synthetic_image(4, 0x19, 0x00, &[0x00]);
    // A byte per bank, at the same offset within each.
    for bank in 0..4usize {
        rom[bank * 0x4000 + 0x0200] = 0xb0 + bank as u8;
    }
    let cart = GbCart::new(Cartridge::parse(rom).expect("a valid image"));
    let region = Device::region(&cart, super::cart::ROM_REGION).expect("the ROM window");
    let ops = io(&region);
    let read = |addr: u64| {
        let mut byte = [0u8; 1];
        ops.read(addr, &mut byte, MemAttrs::DEFAULT)
            .expect("answers");
        byte[0]
    };

    assert_eq!(read(0x0200), 0xb0, "bank 0 is at $0000");
    assert_eq!(read(0x4200), 0xb1, "and bank 1 at $4000 before any write");

    // The bank number is split across two registers, and both halves live in
    // the ROM one: `bank_high` is MBC5's *RAM* bank and must not leak into it.
    ops.write(0x2000, &[0x03], MemAttrs::DEFAULT).expect("ok");
    assert_eq!(read(0x4200), 0xb3);
    ops.write(0x4000, &[0x0f], MemAttrs::DEFAULT).expect("ok");
    assert_eq!(
        read(0x4200),
        0xb3,
        "the RAM bank does not move the ROM bank"
    );
    // And zero really means zero on this controller.
    ops.write(0x2000, &[0x00], MemAttrs::DEFAULT).expect("ok");
    assert_eq!(read(0x4200), 0xb0);
}

/// Pan Docs documents an OAM transfer's source as `$XX00-$XX9F` with `XX` up to
/// `$DF` and stops there. A DMG answers higher pages out of work RAM, because
/// the transfer's address is decoded with fifteen bits — the echo, extended
/// over the whole quarter rather than stopping at `$FDFF` the way the CPU's own
/// decode does (Gekkio, `oam_dma/sources`).
#[test]
fn a_transfer_from_above_dfff_reads_work_ram() {
    use crate::core::space::{AddressSpace, RamStore, Region};

    let ppu = GbPpu::new();
    let space = Arc::new(AddressSpace::new("cpubus", 16));
    let ram = Arc::new(RamStore::new(0x2000));
    for i in 0..0xa0u64 {
        // $DE00 + i, which is offset $1E00 + i within the 8 KiB.
        ram.write_u8(0x1e00 + i, 0x40 + i as u8).expect("in range");
    }
    space
        .topology()
        .map(Region::ram("wram", ram), 0xc000)
        .expect("maps");
    ppu.attach_space(space);

    // Page $FE, which is $DE with the fifteenth bit decoded away.
    ppu.write_register(0x00, 0); // LCD off, so only the transfer blocks
    ppu.write_register(0x06, 0xfe);
    ppu.advance_by(4 * 162);
    assert_eq!(ppu.peek_oam(0), 0x40);
    assert_eq!(ppu.peek_oam(0x9f), 0x40 + 0x9f);
}

/// Switching the LCD on does not start line 0 from its beginning, and the scan
/// it lands in the middle of is reported as mode **0** rather than mode 2 —
/// "line 0 starts with mode 0 and goes straight to mode 3" (Gekkio,
/// `ppu/lcdon_timing`). Object memory is not shut out for it either.
#[test]
fn switching_the_lcd_on_lands_part_way_into_a_scan_that_reports_mode_zero() {
    let ppu = lcd(0);
    let oam = Device::region(&ppu, ppu::OAM_REGION).expect("OAM");
    let oam = io(&oam);
    let mut byte = [0u8; 1];
    let read_mode = |ppu: &GbPpu| ppu.read_register(0x01) & 3;

    ppu.write_register(0x00, 0);
    ppu.advance_by(1000);
    ppu.write_register(0x00, lcdc::LCD_ENABLE);

    assert_eq!(
        ppu.position(),
        (0, ppu::LCD_ON_SKIP),
        "already inside line 0"
    );
    assert_eq!(read_mode(&ppu), Mode::HBlank.bits(), "mode 0, not mode 2");
    oam.read(0, &mut byte, MemAttrs::DEFAULT).expect("answers");
    assert_eq!(byte[0], 0x00, "and object memory is not shut out");

    // Straight into mode 3: the reported mode never shows 2, so the
    // suppression holds right up to the machine cycle mode 3 appears on.
    ppu.advance_by(ppu::OAM_SCAN_DOTS - ppu::LCD_ON_SKIP);
    assert_eq!(read_mode(&ppu), Mode::HBlank.bits());
    ppu.advance_by(ppu::MODE_VISIBLE_LAG);
    assert_eq!(read_mode(&ppu), Mode::Drawing.bits());

    // And the line is four dots short, so `LY` moves one machine cycle early.
    ppu.advance_by(ppu::DOTS_PER_LINE - ppu::OAM_SCAN_DOTS - ppu::MODE_VISIBLE_LAG);
    assert_eq!(ppu.position().0, 1, "line 0 ran 452 dots, not 456");
    // Every line after it is normal again.
    ppu.advance_by(ppu::MODE_VISIBLE_LAG);
    assert_eq!(read_mode(&ppu), Mode::OamScan.bits());
}
