//! Tests for the Master System's devices.
//!
//! Every device here carries a save/load round trip, because `CLAUDE.md` asks
//! for one *with* the device rather than later. Beyond that the tests aim at the
//! behaviours that are invisible in a screenshot and expensive to find later:
//! the VDP's status-read side effects and the debug read that must not cause
//! them, the line-interrupt counter, the mapper's fixed first kilobyte, and the
//! PSG's noise register.

use alloc::sync::Arc;
use alloc::vec;
use alloc::vec::Vec;

use crate::core::device::{Device, ResetKind};
use crate::core::props::Props;
use crate::core::space::{MemAttrs, MemOps, RegionKind, RegionRef};
use crate::core::state::{MachineShape, Migrations, StateReader, StateWriter};

use super::io::{Button, Nationalisation, SmsIo};
use super::mapper::{BANK_SIZE, FIXED_LEN, SegaMapper};
use super::psg::{SAMPLE_DIVISOR, SmsPsg};
use super::sdsc::SdscConsole;
use super::vdp::{DOTS_PER_LINE, SCREEN_WIDTH, SmsVdp, TvRegion, VdpMode};

/// The [`MemOps`] behind an I/O region.
///
/// A test reaches a device's ports the way the address space does rather than
/// through a private method, so what is tested is the surface a guest sees.
fn io(region: &RegionRef) -> &Arc<dyn MemOps> {
    match region.kind() {
        RegionKind::Io(ops) => ops,
        other => panic!("expected an I/O region, found {other:?}"),
    }
}

fn read(ops: &Arc<dyn MemOps>, offset: u64, attrs: MemAttrs) -> u8 {
    let mut byte = [0u8];
    ops.read(offset, &mut byte, attrs).expect("a mapped byte");
    byte[0]
}

fn write(ops: &Arc<dyn MemOps>, offset: u64, value: u8) {
    ops.write(offset, &[value], MemAttrs::DEFAULT)
        .expect("an accepted write");
}

/// Save a device and load it back into a fresh one, asserting the bytes match.
///
/// The round trip `CLAUDE.md` asks for, as one function: a device whose `save`
/// and `load` disagree produces a different chunk the second time round.
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
// The VDP's command port
// ---------------------------------------------------------------------------

/// Set the address register and the command code the way a guest does.
fn command(vdp: &SmsVdp, code: u8, addr: u16) {
    vdp.write_port(1, (addr & 0xff) as u8);
    vdp.write_port(1, ((addr >> 8) as u8 & 0x3f) | (code << 6));
}

#[test]
fn a_control_write_is_two_bytes_and_the_second_carries_the_code() {
    let vdp = SmsVdp::new(TvRegion::Ntsc);
    // Code 1 is a VRAM write at $1234.
    command(&vdp, 1, 0x1234);
    vdp.write_port(0, 0xa5);
    assert_eq!(vdp.peek_vram(0x1234), 0xa5);
    // The address auto-increments.
    vdp.write_port(0, 0x5a);
    assert_eq!(vdp.peek_vram(0x1235), 0x5a);
}

#[test]
fn a_register_write_takes_its_value_from_the_first_byte() {
    let vdp = SmsVdp::new(TvRegion::Ntsc);
    // Code 2, register 1, value $60: display on, frame interrupt off.
    vdp.write_port(1, 0x60);
    vdp.write_port(1, 0x81);
    assert_eq!(vdp.register(1), 0x60);
}

#[test]
fn a_read_command_prefetches_so_the_first_read_is_the_addressed_byte() {
    let vdp = SmsVdp::new(TvRegion::Ntsc);
    vdp.poke_vram(0x0100, 0x11);
    vdp.poke_vram(0x0101, 0x22);
    command(&vdp, 0, 0x0100);
    assert_eq!(vdp.read_port(0), 0x11, "the prefetch, not the byte before");
    assert_eq!(vdp.read_port(0), 0x22);
}

#[test]
fn a_data_access_clears_the_half_finished_control_latch() {
    let vdp = SmsVdp::new(TvRegion::Ntsc);
    // Half a command, then a data write — which is what an interrupt handler
    // does to a main loop that was mid-command.
    vdp.write_port(1, 0x34);
    vdp.write_port(0, 0x00);
    // The next control byte is therefore a *first* byte again, so this pair
    // completes a register write rather than being taken as a second half.
    vdp.write_port(1, 0x7f);
    vdp.write_port(1, 0x87);
    assert_eq!(vdp.register(7), 0x7f);
}

// ---------------------------------------------------------------------------
// The status register, and the debug read that must not disturb it
// ---------------------------------------------------------------------------

/// Run the chip to the first dot of `line`'s horizontal blank.
fn run_to_hblank(vdp: &SmsVdp, line: u64) {
    vdp.advance_to(line * DOTS_PER_LINE + 256);
}

#[test]
fn reading_the_status_register_clears_the_frame_flag_and_drops_the_line() {
    let vdp = SmsVdp::new(TvRegion::Ntsc);
    // Display on, frame interrupt enabled.
    vdp.write_port(1, 0x60);
    vdp.write_port(1, 0x81);
    run_to_hblank(&vdp, 192);
    assert!(
        vdp.irq_line(),
        "the frame interrupt is asserted at line 192"
    );
    assert_eq!(vdp.read_port(1) & 0x80, 0x80, "and the flag is set");
    assert!(!vdp.irq_line(), "reading $BF is the acknowledge");
}

#[test]
fn a_debug_read_of_the_status_register_acknowledges_nothing() {
    let vdp = SmsVdp::new(TvRegion::Ntsc);
    vdp.write_port(1, 0x60);
    vdp.write_port(1, 0x81);
    run_to_hblank(&vdp, 192);

    let ports = vdp.region("ports").expect("the port aperture");
    let ops = io(&ports);
    let debug = MemAttrs::DEBUG;
    assert_eq!(read(ops, 1, debug) & 0x80, 0x80, "a monitor sees the flag");
    assert!(
        vdp.irq_line(),
        "and a monitor looking at it must not acknowledge the interrupt"
    );
    // The ordinary read still does.
    assert_eq!(read(ops, 1, MemAttrs::DEFAULT) & 0x80, 0x80);
    assert!(!vdp.irq_line());
}

#[test]
fn a_debug_read_of_the_data_port_does_not_advance_the_address() {
    let vdp = SmsVdp::new(TvRegion::Ntsc);
    vdp.poke_vram(0x0200, 0x77);
    vdp.poke_vram(0x0201, 0x88);
    command(&vdp, 0, 0x0200);

    let ports = vdp.region("ports").expect("the port aperture");
    let ops = io(&ports);
    let debug = MemAttrs::DEBUG;
    assert_eq!(read(ops, 0, debug), 0x77);
    assert_eq!(read(ops, 0, debug), 0x77, "twice, because nothing moved");
    assert_eq!(
        read(ops, 0, MemAttrs::DEFAULT),
        0x77,
        "and the guest still sees the same byte"
    );
    assert_eq!(read(ops, 0, MemAttrs::DEFAULT), 0x88, "then the next one");
}

#[test]
fn a_debug_read_does_not_reset_the_control_latch() {
    let vdp = SmsVdp::new(TvRegion::Ntsc);
    let ports = vdp.region("ports").expect("the port aperture");
    let ops = io(&ports);
    // Half a command.
    vdp.write_port(1, 0x7f);
    // A monitor looks at the status register.
    let _ = read(ops, 1, MemAttrs::DEBUG);
    // The other half still completes the register write it was going to.
    vdp.write_port(1, 0x87);
    assert_eq!(vdp.register(7), 0x7f);
}

// ---------------------------------------------------------------------------
// Interrupts
// ---------------------------------------------------------------------------

#[test]
fn the_line_counter_fires_every_reg10_plus_one_lines() {
    let vdp = SmsVdp::new(TvRegion::Ntsc);
    // R0 = $10: line interrupt enabled. R1 = $40: display on, no frame IRQ.
    vdp.write_port(1, 0x10);
    vdp.write_port(1, 0x80);
    vdp.write_port(1, 0x40);
    vdp.write_port(1, 0x81);
    // R10 = 1, so the counter underflows every second line.
    vdp.write_port(1, 0x01);
    vdp.write_port(1, 0x8a);

    // The counter starts at $FF from reset and is only reloaded on underflow,
    // so the first hit takes 256 lines. Get past that and then measure.
    vdp.advance_to(300 * DOTS_PER_LINE);
    let _ = vdp.read_port(1);

    let mut hits = 0;
    for line in 0..64u64 {
        run_to_hblank(&vdp, 300 + line);
        if vdp.irq_line() {
            hits += 1;
            let _ = vdp.read_port(1);
        }
    }
    assert!(hits > 0, "a line interrupt must fire at all");
}

#[test]
fn a_line_interrupt_stays_asserted_until_the_status_register_is_read() {
    let vdp = SmsVdp::new(TvRegion::Ntsc);
    vdp.write_port(1, 0x10);
    vdp.write_port(1, 0x80);
    vdp.write_port(1, 0x40);
    vdp.write_port(1, 0x81);
    vdp.write_port(1, 0x00);
    vdp.write_port(1, 0x8a);
    // R10 = 0 makes every line an interrupt, once the initial $FF has run out.
    vdp.advance_to(300 * DOTS_PER_LINE + 256);
    assert!(vdp.irq_line());
    // Running on does not clear it: the chip holds the line, and a handler that
    // returns without reading $BF is interrupted again. That is the behaviour a
    // *level* gives for free and a pulse would not.
    vdp.advance_by(DOTS_PER_LINE / 2);
    assert!(vdp.irq_line());
    let _ = vdp.read_port(1);
    assert!(!vdp.irq_line());
}

#[test]
fn the_frame_flag_is_raised_at_the_first_blanked_line_of_each_mode() {
    // The four mode bits are M1 = R1 bit 4, M2 = R0 bit 1, M3 = R1 bit 3 and
    // M4 = R0 bit 2, and the tall variants need *two* of them beyond M4:
    // 224 lines is M1 and M3, 240 lines is M2 and M3. Spelling them out is the
    // point of the test — a table read in bit order gets both wrong.
    for (r0, r1, height) in [
        (0x04u8, 0x60u8, 192u16),
        (0x04, 0x78, 224),
        (0x06, 0x68, 240),
    ] {
        let vdp = SmsVdp::new(TvRegion::Pal);
        vdp.write_port(1, r0);
        vdp.write_port(1, 0x80);
        // R1 also carries display-on and the frame interrupt enable.
        vdp.write_port(1, r1);
        vdp.write_port(1, 0x81);
        assert_eq!(vdp.mode(), VdpMode::Mode4 { height });

        run_to_hblank(&vdp, u64::from(height) - 1);
        assert!(!vdp.irq_line(), "not yet at line {}", height - 1);
        run_to_hblank(&vdp, u64::from(height));
        assert!(vdp.irq_line(), "at line {height}");
    }
}

// ---------------------------------------------------------------------------
// The counters
// ---------------------------------------------------------------------------

#[test]
fn the_v_counter_runs_are_the_length_of_the_frame() {
    for (region, lines) in [(TvRegion::Ntsc, 262u64), (TvRegion::Pal, 313)] {
        let vdp = SmsVdp::new(region);
        let mut seen = Vec::new();
        for line in 0..lines {
            vdp.advance_to(line * DOTS_PER_LINE + 1);
            seen.push(vdp.vcounter());
        }
        assert_eq!(seen.len() as u64, lines);
        assert_eq!(seen[0], 0, "the frame starts at zero");
        // The table's job is to fit more lines than a byte holds, so the counter
        // must repeat somewhere and must never be undefined.
        assert!(seen.contains(&0xff), "{region:?}: the counter reaches $FF");
    }
}

#[test]
fn the_h_counter_folds_342_pixels_into_171_values() {
    let vdp = SmsVdp::new(TvRegion::Ntsc);
    assert_eq!(vdp.hcounter(), 0x00);
    vdp.advance_by(0x93 * 2);
    assert_eq!(vdp.hcounter(), 0x93, "the last value before the jump");
    vdp.advance_by(2);
    assert_eq!(vdp.hcounter(), 0xe9, "and it jumps rather than wrapping");
}

// ---------------------------------------------------------------------------
// Rendering
// ---------------------------------------------------------------------------

/// Programme the chip for mode 4 with a name table at 0 and patterns at 0.
fn mode4(vdp: &SmsVdp) {
    vdp.write_port(1, 0x04); // R0: mode 4
    vdp.write_port(1, 0x80);
    vdp.write_port(1, 0x40); // R1: display on
    vdp.write_port(1, 0x81);
    vdp.write_port(1, 0xff); // R2: name table at $3800
    vdp.write_port(1, 0x82);
    vdp.write_port(1, 0x00); // R7: backdrop = colour 16
    vdp.write_port(1, 0x87);
}

#[test]
fn a_mode_4_tile_reaches_the_framebuffer_through_colour_ram() {
    let vdp = SmsVdp::new(TvRegion::Ntsc);
    mode4(&vdp);
    // Tile 1, row 0: bitplane 0 set for every pixel, so index 1 throughout.
    vdp.poke_vram(32, 0xff);
    // Name table entry (0,0) = tile 1.
    vdp.poke_vram(0x3800, 0x01);
    vdp.poke_vram(0x3801, 0x00);
    vdp.poke_cram(1, 0x2a);

    // Line 0 was drawn at reset with none of this in place, so step a frame.
    vdp.advance_to(DOTS_PER_LINE * 262);
    assert_eq!(vdp.pixel(0, 0), Some(0x2a));
    assert_eq!(vdp.pixel(7, 0), Some(0x2a));
}

#[test]
fn hiding_the_left_column_paints_it_with_the_backdrop() {
    let vdp = SmsVdp::new(TvRegion::Ntsc);
    mode4(&vdp);
    vdp.poke_vram(32, 0xff);
    // The first two tile columns, so there is something at x = 8 to survive.
    vdp.poke_vram(0x3800, 0x01);
    vdp.poke_vram(0x3802, 0x01);
    vdp.poke_cram(1, 0x2a);
    vdp.poke_cram(16, 0x15);
    // R0 bit 5 blanks the leftmost eight pixels.
    vdp.write_port(1, 0x24);
    vdp.write_port(1, 0x80);

    vdp.advance_to(DOTS_PER_LINE * 262);
    assert_eq!(vdp.pixel(0, 0), Some(0x15), "backdrop");
    assert_eq!(vdp.pixel(8, 0), Some(0x2a), "and the tile resumes at 8");
}

#[test]
fn a_blanked_display_is_the_backdrop_everywhere() {
    let vdp = SmsVdp::new(TvRegion::Ntsc);
    vdp.write_port(1, 0x04);
    vdp.write_port(1, 0x80);
    vdp.write_port(1, 0x00); // R1: display *off*
    vdp.write_port(1, 0x81);
    vdp.poke_cram(16 + 3, 0x3f);
    vdp.write_port(1, 0x03); // R7: backdrop = sprite colour 3
    vdp.write_port(1, 0x87);

    vdp.advance_to(DOTS_PER_LINE * 262);
    for x in [0usize, 100, SCREEN_WIDTH - 1] {
        assert_eq!(vdp.pixel(x, 100), Some(0x3f));
    }
}

#[test]
fn the_mode_bits_are_not_in_bit_order() {
    let vdp = SmsVdp::new(TvRegion::Ntsc);
    // Everything clear is Graphics I, the TMS9918A's default.
    assert_eq!(vdp.mode(), VdpMode::GraphicsI);
    // M1 alone (R1 bit 4) is text.
    vdp.write_port(1, 0x10);
    vdp.write_port(1, 0x81);
    assert_eq!(vdp.mode(), VdpMode::Text);
    // M2 alone (R0 bit 1) is Graphics II.
    vdp.write_port(1, 0x00);
    vdp.write_port(1, 0x81);
    vdp.write_port(1, 0x02);
    vdp.write_port(1, 0x80);
    assert_eq!(vdp.mode(), VdpMode::GraphicsII);
    // M3 alone (R1 bit 3) is multicolor.
    vdp.write_port(1, 0x00);
    vdp.write_port(1, 0x80);
    vdp.write_port(1, 0x08);
    vdp.write_port(1, 0x81);
    assert_eq!(vdp.mode(), VdpMode::Multicolor);
}

#[test]
fn a_vdp_round_trips() {
    let vdp = SmsVdp::new(TvRegion::Pal);
    mode4(&vdp);
    vdp.poke_vram(0x1000, 0x5a);
    vdp.poke_cram(7, 0x2a);
    vdp.advance_to(DOTS_PER_LINE * 40 + 17);
    let restored = SmsVdp::new(TvRegion::Pal);
    round_trip(&vdp, &restored, "vdp");
    assert_eq!(
        restored.dots(),
        vdp.dots(),
        "the derived tick follows the load"
    );
    assert_eq!(restored.position(), vdp.position());
}

#[test]
fn a_reset_returns_the_chip_to_its_power_on_registers() {
    let vdp = SmsVdp::new(TvRegion::Ntsc);
    mode4(&vdp);
    vdp.advance_by(1000);
    vdp.reset(ResetKind::Cold);
    assert_eq!(vdp.register(1), 0x00);
    assert_eq!(vdp.register(10), 0xff);
    assert_eq!(vdp.mode(), VdpMode::GraphicsI);
    // But **not** the device's own clock. The scheduler owns that, and
    // `Machine::reset` does not rewind the clock domains — a device that zeroed
    // its tick here would be told to advance to wherever the scheduler already
    // was and would replay every dot in between.
    assert_eq!(vdp.dots(), 1000, "a reset does not rewind the device's tick");
}

// ---------------------------------------------------------------------------
// The sound chip
// ---------------------------------------------------------------------------

#[test]
fn a_tone_register_takes_two_writes_and_a_volume_takes_one() {
    let psg = SmsPsg::new();
    psg.write(0x80 | 0x0e); // channel 0, tone, low nibble $E
    psg.write(0x3f); // high six bits $3F
    assert_eq!(psg.tone(0), 0x3fe);
    psg.write(0x90 | 0x03); // channel 0, volume 3
    assert_eq!(psg.volume(0), 3);
    // A bare data byte after a volume latch replaces it rather than extending.
    psg.write(0x0a);
    assert_eq!(psg.volume(0), 0x0a);
}

#[test]
fn writing_the_noise_control_resets_the_shift_register() {
    let psg = SmsPsg::new();
    psg.write(0xe7); // channel 3, tone: white noise, period from channel 2
    psg.write(0xf0); // channel 3, volume 0 — loudest
    psg.advance_by(SAMPLE_DIVISOR * 64);
    let shifted = psg.lfsr();
    assert_ne!(shifted, 0x8000, "the register has moved");
    psg.write(0xe4);
    assert_eq!(psg.lfsr(), 0x8000, "and a control write seeds it again");
}

#[test]
fn a_silent_chip_produces_a_flat_line_and_a_loud_one_does_not() {
    let psg = SmsPsg::new();
    psg.set_recording(true);
    // Everything attenuated to nothing at reset.
    psg.advance_by(SAMPLE_DIVISOR * 32);
    let quiet = psg.take_samples();
    assert_eq!(quiet.len(), 32);
    assert!(quiet.iter().all(|&(l, r)| l == 0 && r == 0));

    // Channel 0 at full volume with a short period must swing.
    psg.write(0x80 | 0x02);
    psg.write(0x00);
    psg.write(0x90);
    psg.advance_by(SAMPLE_DIVISOR * 256);
    let loud = psg.take_samples();
    assert!(
        loud.iter().any(|&(l, _)| l > 0) && loud.iter().any(|&(l, _)| l < 0),
        "a square wave crosses zero in both directions"
    );
}

#[test]
fn nothing_is_recorded_unless_a_sink_asked_for_it() {
    let psg = SmsPsg::new();
    psg.write(0x90);
    psg.advance_by(SAMPLE_DIVISOR * 100);
    assert_eq!(psg.queued_samples(), 0);
}

#[test]
fn a_psg_round_trips() {
    let psg = SmsPsg::new();
    psg.write(0x80 | 0x0e);
    psg.write(0x3f);
    psg.write(0x90 | 0x04);
    psg.write(0xe5);
    psg.advance_by(1234);
    let restored = SmsPsg::new();
    round_trip(&psg, &restored, "psg");
    assert_eq!(restored.ticks(), psg.ticks());
}

// ---------------------------------------------------------------------------
// The mapper
// ---------------------------------------------------------------------------

/// A four-bank image whose every byte is its own bank number.
fn image(banks: u8) -> Vec<u8> {
    let mut out = vec![0u8; BANK_SIZE as usize * banks as usize];
    for (i, byte) in out.iter_mut().enumerate() {
        *byte = (i as u64 / BANK_SIZE) as u8;
    }
    out
}

/// Read one byte through the cartridge's container region, as the bus would.
fn cart_read(mapper: &SegaMapper, addr: u64) -> u8 {
    let rom = mapper.region("rom").expect("the cartridge aperture");
    let mut byte = [0u8];
    // The container resolves the address to whichever window covers it, which
    // is the same walk `AddressSpace` does — a `RegionKind::Container` is not
    // `MemOps`, so the test reaches through the flat view the space builds.
    let space = crate::core::space::AddressSpace::new("test", 16);
    space.topology().map(rom, 0).expect("mapped");
    space
        .read_bytes(addr, &mut byte, MemAttrs::DEFAULT)
        .expect("a mapped byte");
    byte[0]
}

#[test]
fn the_first_kilobyte_never_moves() {
    let mapper = SegaMapper::new(&image(4), 0x8000).expect("a board");
    assert_eq!(mapper.banks(), 4);
    // Bank 0 everywhere below $0400, whatever $FFFD says — and it cannot be
    // checked through a rebase here, because a rebase needs the space the
    // windows are mapped in. What this asserts is the *shape*: the fixed alias
    // covers exactly the first kilobyte.
    assert_eq!(cart_read(&mapper, 0x0000), 0);
    assert_eq!(cart_read(&mapper, FIXED_LEN - 1), 0);
}

#[test]
fn the_three_slots_power_on_showing_the_first_three_banks() {
    let mapper = SegaMapper::new(&image(4), 0x8000).expect("a board");
    assert_eq!(mapper.bank(0), 0);
    assert_eq!(mapper.bank(1), 1);
    assert_eq!(mapper.bank(2), 2);
    assert_eq!(cart_read(&mapper, 0x0400), 0);
    assert_eq!(cart_read(&mapper, 0x4000), 1);
    assert_eq!(cart_read(&mapper, 0x8000), 2);
}

#[test]
fn slot_two_becomes_writable_when_the_control_register_says_so() {
    let mapper = SegaMapper::new(&image(4), 0x8000).expect("a board");
    let regs = mapper.region("regs").expect("the register aperture");
    let ops = io(&regs);

    // Writing ROM is swallowed, not faulted.
    let rom = mapper.region("rom").expect("the cartridge aperture");
    let space = crate::core::space::AddressSpace::new("test", 16);
    space.topology().map(rom, 0).expect("mapped");
    space
        .write_bytes(0x8000, &[0x5a], MemAttrs::DEFAULT)
        .expect("swallowed");
    let mut byte = [0u8];
    space
        .read_bytes(0x8000, &mut byte, MemAttrs::DEFAULT)
        .expect("read");
    assert_eq!(byte[0], 2, "still bank 2");

    // $FFFC bit 3 maps the cartridge's RAM there instead.
    write(ops, 0, 0x08);
    assert!(mapper.ram_enabled());
    space
        .write_bytes(0x8000, &[0x5a], MemAttrs::DEFAULT)
        .expect("stored");
    space
        .read_bytes(0x8000, &mut byte, MemAttrs::DEFAULT)
        .expect("read");
    assert_eq!(byte[0], 0x5a);
}

#[test]
fn a_bank_register_wraps_around_the_image() {
    let mapper = SegaMapper::new(&image(2), 0x8000).expect("a board");
    let regs = mapper.region("regs").expect("the register aperture");
    let ops = io(&regs);
    // Bank 5 of a two-bank image is bank 1: the address lines the board does
    // not have simply are not there.
    write(ops, 3, 5);
    assert_eq!(cart_read(&mapper, 0x8000), 1);
}

#[test]
fn a_debug_write_does_not_rebank_the_guest() {
    let mapper = SegaMapper::new(&image(4), 0x8000).expect("a board");
    let regs = mapper.region("regs").expect("the register aperture");
    let ops = io(&regs);
    ops.write(3, &[3], MemAttrs::DEBUG).expect("accepted");
    assert_eq!(
        mapper.bank(2),
        2,
        "a monitor must not move the guest's code"
    );
}

#[test]
fn a_mapper_round_trips_its_registers_and_its_cartridge_ram() {
    let mapper = SegaMapper::new(&image(4), 0x8000).expect("a board");
    let regs = mapper.region("regs").expect("the register aperture");
    let ops = io(&regs);
    write(ops, 0, 0x08);
    write(ops, 1, 3);
    write(ops, 2, 2);
    write(ops, 3, 1);
    mapper
        .ram()
        .write_u8(0x10, 0xc3)
        .expect("a byte of save RAM");

    let restored = SegaMapper::new(&image(4), 0x8000).expect("a board");
    round_trip(&mapper, &restored, "cart");
    assert_eq!(restored.bank(0), 3);
    assert_eq!(restored.ram().read_u8(0x10), Ok(0xc3));
}

// ---------------------------------------------------------------------------
// The I/O chip
// ---------------------------------------------------------------------------

#[test]
fn an_unpressed_pad_reads_as_all_ones() {
    let chip = SmsIo::new(Nationalisation::Export);
    assert_eq!(chip.read_dc(), 0xff);
    // $DD's unused bit reads high, Reset is not pressed, and both TH pins are
    // inputs pulled up.
    assert_eq!(chip.read_dd(), 0xff);
}

#[test]
fn a_pressed_button_pulls_its_line_low() {
    let chip = SmsIo::new(Nationalisation::Export);
    chip.set_pressed(0, Button::Up, true);
    assert_eq!(chip.read_dc(), 0xfe);
    chip.set_pressed(0, Button::Two, true);
    assert_eq!(chip.read_dc(), 0xde);
    // Port B's first two lines are the top of $DC and the rest are in $DD.
    chip.set_pressed(1, Button::Down, true);
    assert_eq!(chip.read_dc(), 0x5e);
    chip.set_pressed(1, Button::One, true);
    assert_eq!(chip.read_dd(), 0xfb);
}

#[test]
fn switching_the_io_chip_out_makes_both_ports_read_as_ones() {
    let chip = SmsIo::new(Nationalisation::Export);
    chip.set_pressed(0, Button::Up, true);
    assert_eq!(chip.read_dc(), 0xfe);
    // $3E bit 2 disables the I/O chip, which is the documented way to reach
    // $FC/$FD.
    chip.write_control(0, 0x04);
    assert_eq!(chip.read_dc(), 0xff);
    assert_eq!(chip.read_dd(), 0xff);
}

#[test]
fn the_th_pins_read_back_their_output_level_on_an_export_console() {
    let export = SmsIo::new(Nationalisation::Export);
    // Both TH pins outputs (bits 1 and 3 clear), both driving high.
    export.write_control(1, 0xf5);
    assert_eq!(export.read_dd() & 0xc0, 0xc0);
    // Both driving low.
    export.write_control(1, 0x05);
    assert_eq!(export.read_dd() & 0xc0, 0x00);

    let japan = SmsIo::new(Nationalisation::Japan);
    japan.write_control(1, 0xf5);
    assert_eq!(japan.read_dd() & 0xc0, 0x00, "and never on a Japanese one");
}

#[test]
fn the_reset_button_is_both_a_bit_and_a_pin() {
    let chip = SmsIo::new(Nationalisation::Export);
    assert_eq!(chip.read_dd() & 0x10, 0x10);
    chip.set_reset(true);
    assert_eq!(chip.read_dd() & 0x10, 0x00, "active low, like everything");
}

#[test]
fn an_io_chip_round_trips() {
    let chip = SmsIo::new(Nationalisation::Export);
    chip.set_buttons(0, 0x15);
    chip.set_pause(true);
    chip.write_control(0, 0x04);
    chip.write_control(1, 0xf5);
    let restored = SmsIo::new(Nationalisation::Export);
    round_trip(&chip, &restored, "io");
    assert_eq!(restored.buttons(0), 0x15);
    assert_eq!(restored.io_control(), 0xf5);
}

// ---------------------------------------------------------------------------
// The debug console
// ---------------------------------------------------------------------------

#[test]
fn the_debug_console_keeps_what_a_test_rom_prints() {
    let console = SdscConsole::new(1024);
    for byte in b"OK\n" {
        console.write_port(1, *byte);
    }
    assert_eq!(console.text(), "OK\n");
    // Control command 2 clears it.
    console.write_port(0, 2);
    assert!(console.text().is_empty());
}

#[test]
fn a_cursor_command_consumes_its_parameters_rather_than_printing_them() {
    let console = SdscConsole::new(1024);
    console.write_port(0, 4); // move cursor
    console.write_port(1, 5); // row
    console.write_port(1, 10); // column
    console.write_port(1, b'A');
    assert_eq!(
        console.text(),
        "A",
        "the row and column are parameters, not text"
    );
}

#[test]
fn a_suspend_request_is_recorded_and_not_obeyed() {
    let console = SdscConsole::new(1024);
    assert!(!console.suspend_requested());
    console.write_port(0, 1);
    assert!(console.suspend_requested());
}

#[test]
fn a_debug_console_round_trips() {
    let console = SdscConsole::new(1024);
    for byte in b"hello\n" {
        console.write_port(1, *byte);
    }
    console.write_port(0, 3);
    let restored = SdscConsole::new(1024);
    round_trip(&console, &restored, "sdsc");
    assert_eq!(restored.text(), "hello\n");
}

// ---------------------------------------------------------------------------
// Construction
// ---------------------------------------------------------------------------

#[test]
fn a_bad_region_name_is_a_configuration_error_rather_than_a_default() {
    let mut props = Props::new();
    props.insert("region", "secam");
    assert!(SmsVdp::from_props(&props).is_err());
}

#[test]
fn every_class_publishes_the_regions_its_schema_promises() {
    let vdp = SmsVdp::new(TvRegion::Ntsc);
    for name in ["ports", "counters"] {
        assert!(vdp.region(name).is_some(), "sms.vdp has no `{name}`");
    }
    assert!(vdp.region("nonsense").is_none());

    let chip = SmsIo::new(Nationalisation::Export);
    for name in ["ctrl", "pads"] {
        assert!(chip.region(name).is_some(), "sms.io has no `{name}`");
    }

    let mapper = SegaMapper::new(&image(2), 0x8000).expect("a board");
    for name in ["rom", "regs"] {
        assert!(mapper.region(name).is_some(), "sms.mapper has no `{name}`");
    }
}
