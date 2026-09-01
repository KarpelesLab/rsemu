//! What a serial NOR part must refuse, and what a snapshot must carry.
//!
//! Every test here talks to the device the way a controller does — whole
//! frames delimited by a chip select — because that is the only interface it
//! has. `contents()` is used to *check* the array, never to change it.

use super::*;
use crate::bus::spi::{SpiBus, exchange, pin as spi_pin};
use crate::core::props::Value;
use crate::core::state::{MachineShape, Migrations, StateReader, StateWriter};
use crate::core::wire::Level;

/// A 1 MiB part, which is small enough to erase in a test and still a power of
/// two with a plausible capacity byte.
const SIZE: u64 = 1024 * 1024;

fn new_part() -> SpiNor {
    part_with(Props::new().with("size", Value::Size(SIZE)))
}

fn part_with(props: Props) -> SpiNor {
    SpiNor::new(&props).expect("a plausible part")
}

/// The `-IM`/`-JM` ordering option: memory type `70h`, whose `QE` bit powers
/// up clear and is programmable. The default part is a `-IQ`/`-JQ` (`40h`)
/// with `QE` fixed set, so a test about `QE` needs this one.
fn programmable_quad_part() -> SpiNor {
    part_with(
        Props::new()
            .with("size", Value::Size(SIZE))
            .with("type", Value::Uint(0x70)),
    )
}

/// Clock one whole frame: assert CS, exchange every byte, release CS.
///
/// The return value is what came back on MISO during each byte, so index `n`
/// of the answer belongs to byte `n` of the question — which is the full-duplex
/// truth a request/response shape would hide.
fn frame(part: &SpiNor, bytes: &[u8]) -> Vec<u8> {
    let slave = part.slave();
    slave.select(true);
    let out = bytes
        .iter()
        .map(|b| exchange(&*slave, u32::from(*b)) as u8)
        .collect();
    slave.select(false);
    out
}

/// A frame that is cut short: CS rises without the instruction finishing.
fn truncated(part: &SpiNor, bytes: &[u8]) {
    let slave = part.slave();
    slave.select(true);
    for b in bytes {
        exchange(&*slave, u32::from(*b));
    }
    slave.select(false);
}

fn write_enable(part: &SpiNor) {
    frame(part, &[CMD_WRITE_ENABLE]);
}

/// `03h` at `addr` for `len` bytes.
fn read(part: &SpiNor, addr: u32, len: usize) -> Vec<u8> {
    let mut req = alloc::vec![CMD_READ, (addr >> 16) as u8, (addr >> 8) as u8, addr as u8,];
    req.extend(core::iter::repeat_n(0u8, len));
    frame(part, &req)[4..].to_vec()
}

fn program(part: &SpiNor, addr: u32, data: &[u8]) {
    write_enable(part);
    let mut req = alloc::vec![
        CMD_PAGE_PROGRAM,
        (addr >> 16) as u8,
        (addr >> 8) as u8,
        addr as u8,
    ];
    req.extend_from_slice(data);
    frame(part, &req);
}

fn erase(part: &SpiNor, cmd: u8, addr: u32) {
    write_enable(part);
    frame(
        part,
        &[cmd, (addr >> 16) as u8, (addr >> 8) as u8, addr as u8],
    );
}

fn status(part: &SpiNor, cmd: u8) -> u8 {
    frame(part, &[cmd, 0])[1]
}

// ---------------------------------------------------------------------------
// identification
// ---------------------------------------------------------------------------

#[test]
fn the_jedec_id_is_the_manufacturer_the_type_and_the_density() {
    let part = new_part();
    // Three answers, and the first byte is the part still driving nothing
    // while it receives the opcode — the full-duplex fact that a
    // request/response model gets wrong.
    let out = frame(&part, &[CMD_JEDEC_ID, 0, 0, 0]);
    assert_eq!(out[0], IDLE_BYTE);
    assert_eq!(&out[1..], &[WINBOND, TYPE_W25Q, 20], "1 MiB is 2^20");
    assert_eq!(part.jedec_id(), [WINBOND, TYPE_W25Q, 20]);
}

#[test]
fn the_jedec_id_repeats_for_as_long_as_the_master_clocks() {
    let part = new_part();
    let out = frame(&part, &[CMD_JEDEC_ID, 0, 0, 0, 0, 0, 0]);
    assert_eq!(&out[1..], &[WINBOND, TYPE_W25Q, 20, WINBOND, TYPE_W25Q, 20]);
}

#[test]
fn the_device_id_takes_a_three_byte_address_and_alternates() {
    let part = new_part();
    let out = frame(&part, &[CMD_DEVICE_ID, 0, 0, 0, 0, 0, 0]);
    assert_eq!(&out[4..], &[WINBOND, 19, WINBOND], "capacity less one");
}

#[test]
fn releasing_power_down_answers_the_device_id_after_three_dummies() {
    let part = new_part();
    frame(&part, &[CMD_POWER_DOWN]);
    // A part in deep power-down answers nothing else.
    assert_eq!(frame(&part, &[CMD_JEDEC_ID, 0, 0, 0])[1..], [IDLE_BYTE; 3]);
    let out = frame(&part, &[CMD_RELEASE_POWER_DOWN, 0, 0, 0, 0]);
    assert_eq!(out[4], 19);
    assert_eq!(frame(&part, &[CMD_JEDEC_ID, 0])[1], WINBOND, "awake again");
}

// ---------------------------------------------------------------------------
// the semantics that make flash flash
// ---------------------------------------------------------------------------

#[test]
fn a_fresh_part_is_erased_rather_than_zeroed() {
    let part = new_part();
    assert_eq!(read(&part, 0, 4), [0xff; 4]);
    assert!(part.contents().iter().all(|b| *b == 0xff));
}

#[test]
fn a_page_program_writes_and_a_read_reads_it_back() {
    let part = new_part();
    program(&part, 0x1234, b"hello");
    assert_eq!(read(&part, 0x1234, 5), b"hello");
    assert_eq!(read(&part, 0x1239, 1), [0xff], "and no further");
}

#[test]
fn a_program_can_only_clear_bits() {
    let part = new_part();
    program(&part, 0, &[0x0f]);
    assert_eq!(read(&part, 0, 1), [0x0f]);
    // The rule the device exists to enforce. `0xf0` over `0x0f` is `0x00`,
    // not `0xf0`: a write that could set a bit back to one would let every
    // fault-tolerant-write scheme ever built succeed at something the silicon
    // refuses.
    program(&part, 0, &[0xf0]);
    assert_eq!(read(&part, 0, 1), [0x00]);
}

#[test]
fn a_program_without_write_enable_does_nothing() {
    let part = new_part();
    // No `06h` first.
    frame(&part, &[CMD_PAGE_PROGRAM, 0, 0, 0, 0x55]);
    assert_eq!(read(&part, 0, 1), [0xff]);
}

#[test]
fn a_completed_program_clears_the_write_enable_latch() {
    let part = new_part();
    write_enable(&part);
    assert_eq!(status(&part, CMD_READ_STATUS1) & SR1_WEL, SR1_WEL);
    frame(&part, &[CMD_PAGE_PROGRAM, 0, 0, 0, 0x55]);
    assert_eq!(status(&part, CMD_READ_STATUS1) & SR1_WEL, 0);
    // Which is why the second program without a fresh `06h` does nothing.
    frame(&part, &[CMD_PAGE_PROGRAM, 0, 0, 1, 0x55]);
    assert_eq!(read(&part, 1, 1), [0xff]);
}

#[test]
fn write_disable_clears_the_latch() {
    let part = new_part();
    write_enable(&part);
    frame(&part, &[CMD_WRITE_DISABLE]);
    assert_eq!(status(&part, CMD_READ_STATUS1) & SR1_WEL, 0);
}

#[test]
fn a_page_program_wraps_within_its_page() {
    let part = new_part();
    write_enable(&part);
    let mut req = alloc::vec![CMD_PAGE_PROGRAM, 0, 0, 0xfe];
    // Four bytes from offset 0xfe: two land at the end of the page and two
    // wrap to its start, rather than spilling into the next page.
    req.extend_from_slice(&[1, 2, 3, 4]);
    frame(&part, &req);
    assert_eq!(read(&part, 0xfe, 2), [1, 2]);
    assert_eq!(read(&part, 0x00, 2), [3, 4]);
    assert_eq!(read(&part, 0x100, 1), [0xff], "the next page is untouched");
}

#[test]
fn an_erase_sets_a_whole_sector_and_only_that_sector() {
    let part = new_part();
    program(&part, 0x0ffe, &[0, 0]);
    program(&part, 0x1000, &[0]);
    program(&part, 0x2000, &[0]);
    erase(&part, CMD_SECTOR_ERASE, 0x1abc);
    assert_eq!(read(&part, 0x0ffe, 2), [0, 0], "below the sector");
    assert_eq!(read(&part, 0x1000, 1), [0xff], "inside it");
    assert_eq!(read(&part, 0x2000, 1), [0], "above it");
}

#[test]
fn the_three_erase_granules_are_the_sizes_the_opcodes_name() {
    for (cmd, span) in [
        (CMD_SECTOR_ERASE, SECTOR),
        (CMD_HALF_BLOCK_ERASE, HALF_BLOCK),
        (CMD_BLOCK_ERASE, BLOCK),
    ] {
        let part = new_part();
        for at in [0u32, span as u32 - 1, span as u32] {
            program(&part, at, &[0]);
        }
        // An address in the middle: the granule is aligned down, not taken
        // literally.
        erase(&part, cmd, span as u32 / 2 + 1);
        assert_eq!(read(&part, 0, 1), [0xff], "{cmd:#04x}");
        assert_eq!(read(&part, span as u32 - 1, 1), [0xff], "{cmd:#04x}");
        assert_eq!(read(&part, span as u32, 1), [0], "{cmd:#04x} stops here");
    }
}

#[test]
fn a_chip_erase_takes_the_whole_part() {
    let part = new_part();
    program(&part, 0, &[0]);
    program(&part, (SIZE - 1) as u32, &[0]);
    write_enable(&part);
    frame(&part, &[CMD_CHIP_ERASE]);
    assert!(part.contents().iter().all(|b| *b == 0xff));
}

// ---------------------------------------------------------------------------
// framing: an instruction cut short is not executed
// ---------------------------------------------------------------------------

#[test]
fn an_erase_whose_address_never_finished_is_not_executed() {
    let part = new_part();
    program(&part, 0, &[0x00]);
    write_enable(&part);
    // Two of the three address bytes, then CS rises. The datasheet is
    // explicit that the instruction is not executed — and the write-enable
    // latch survives, because nothing completed to clear it.
    truncated(&part, &[CMD_SECTOR_ERASE, 0, 0]);
    assert_eq!(read(&part, 0, 1), [0x00], "the sector was not erased");
    assert_eq!(status(&part, CMD_READ_STATUS1) & SR1_WEL, SR1_WEL);
}

#[test]
fn a_page_program_with_no_data_bytes_is_not_executed() {
    let part = new_part();
    write_enable(&part);
    truncated(&part, &[CMD_PAGE_PROGRAM, 0, 0, 0]);
    assert_eq!(read(&part, 0, 1), [0xff]);
    assert_eq!(
        status(&part, CMD_READ_STATUS1) & SR1_WEL,
        SR1_WEL,
        "nothing completed, so nothing cleared the latch"
    );
}

#[test]
fn an_unknown_opcode_is_ignored_for_the_rest_of_the_frame() {
    let part = new_part();
    write_enable(&part);
    // `0xaa` is not an instruction; the bytes after it must not be mistaken
    // for one.
    frame(&part, &[0xaa, CMD_CHIP_ERASE, 0, 0]);
    assert_eq!(status(&part, CMD_READ_STATUS1) & SR1_WEL, SR1_WEL);
}

// ---------------------------------------------------------------------------
// reads
// ---------------------------------------------------------------------------

#[test]
fn fast_read_costs_one_dummy_byte_and_plain_read_none() {
    let part = new_part();
    program(&part, 0x10, &[0xa5, 0x5a]);
    assert_eq!(read(&part, 0x10, 2), [0xa5, 0x5a]);
    let out = frame(&part, &[CMD_FAST_READ, 0, 0, 0x10, 0, 0, 0]);
    assert_eq!(&out[5..], &[0xa5, 0x5a], "one dummy byte, then the array");
}

#[test]
fn a_read_running_off_the_end_wraps_to_the_start() {
    let part = new_part();
    program(&part, 0, &[0x11]);
    let out = frame(
        &part,
        &[CMD_READ, ((SIZE - 1) >> 16) as u8, 0xff, 0xff, 0, 0],
    );
    assert_eq!(out[5], 0x11, "the byte after the last is the first");
}

#[test]
fn a_quad_read_needs_the_quad_enable_bit() {
    let part = programmable_quad_part();
    program(&part, 0, &[0x42]);
    // Refused: `QE` is clear out of reset on a `-IM` part, so the frame is
    // ignored entirely.
    let out = frame(&part, &[CMD_FAST_READ_QUAD_OUT, 0, 0, 0, 0, 0]);
    assert_eq!(out[5], IDLE_BYTE);
    // Set `QE` through `31h` and the same frame answers.
    write_enable(&part);
    frame(&part, &[CMD_WRITE_STATUS2, SR2_QE]);
    assert_eq!(status(&part, CMD_READ_STATUS2) & SR2_QE, SR2_QE);
    let out = frame(&part, &[CMD_FAST_READ_QUAD_OUT, 0, 0, 0, 0, 0]);
    assert_eq!(out[5], 0x42);
}

#[test]
fn the_memory_type_byte_decides_whether_quad_enable_is_fixed() {
    // `40h` is the ordering option whose `QE` is fixed set: it powers up set
    // and a status-register write cannot clear it (§11, §7.1.4).
    let fixed = new_part();
    assert_eq!(status(&fixed, CMD_READ_STATUS2) & SR2_QE, SR2_QE);
    write_enable(&fixed);
    frame(&fixed, &[CMD_WRITE_STATUS2, 0]);
    assert_eq!(status(&fixed, CMD_READ_STATUS2) & SR2_QE, SR2_QE);
    // `70h` powers up clear and takes the write.
    let programmable = programmable_quad_part();
    assert_eq!(status(&programmable, CMD_READ_STATUS2) & SR2_QE, 0);
}

#[test]
fn the_io_reads_consume_their_mode_byte_and_forget_it() {
    let part = new_part();
    program(&part, 0x20, &[0xc3, 0x3c]);
    // `BBh`: four mode clocks on two lines, which is one byte here, and no
    // further dummy — so the data starts at byte 5.
    let out = frame(&part, &[CMD_FAST_READ_DUAL_IO, 0, 0, 0x20, 0xff, 0, 0]);
    assert_eq!(&out[5..], &[0xc3, 0x3c]);
    // `EBh`: two mode clocks plus four dummy clocks on four lines, which is
    // three bytes, so the data starts at byte 7.
    let out = frame(
        &part,
        &[CMD_FAST_READ_QUAD_IO, 0, 0, 0x20, 0xff, 0, 0, 0, 0],
    );
    assert_eq!(&out[7..], &[0xc3, 0x3c]);
    // And the mode byte leaves nothing behind: the frame after it still needs
    // its opcode. This generation has no continuous-read latch, whatever the
    // `FV` family did.
    let out = frame(&part, &[0, 0, 0x20, 0xff, 0, 0]);
    assert_ne!(out[5], 0xc3, "no opcode, no read");
}

// ---------------------------------------------------------------------------
// the status registers
// ---------------------------------------------------------------------------

#[test]
fn busy_never_reads_set_because_this_model_takes_no_time() {
    let part = new_part();
    // The polling loop firmware writes. It terminates on the first read here,
    // deliberately: see the module docs for why a serial part gets no clock
    // domain.
    erase(&part, CMD_SECTOR_ERASE, 0);
    assert_eq!(status(&part, CMD_READ_STATUS1) & SR1_BUSY, 0);
}

#[test]
fn a_status_register_write_keeps_the_bits_that_are_the_parts_own() {
    let part = new_part();
    write_enable(&part);
    // Every bit set: `BUSY` and `WEL` must not take, because they are status
    // rather than settings.
    frame(&part, &[CMD_WRITE_STATUS1, 0xff]);
    let sr1 = status(&part, CMD_READ_STATUS1);
    assert_eq!(sr1 & (SR1_BUSY | SR1_WEL), 0);
    assert_eq!(sr1, SR1_WRITABLE);
}

#[test]
fn writing_status_register_one_can_carry_register_two_as_well() {
    let part = new_part();
    write_enable(&part);
    frame(&part, &[CMD_WRITE_STATUS1, SR1_SRP, SR2_QE]);
    assert_eq!(status(&part, CMD_READ_STATUS1), SR1_SRP);
    assert_eq!(status(&part, CMD_READ_STATUS2), SR2_QE);
}

#[test]
fn write_protect_with_srp_set_freezes_the_status_register() {
    let part = part_with(
        Props::new()
            .with("size", Value::Size(SIZE))
            .with("readonly", true),
    );
    write_enable(&part);
    frame(&part, &[CMD_WRITE_STATUS1, SR1_SRP]);
    assert_eq!(status(&part, CMD_READ_STATUS1), SR1_SRP, "the first takes");
    // `WP#` low plus `SRP` is hardware protection: from here the register
    // cannot be changed at all, not even to clear `SRP`.
    write_enable(&part);
    frame(&part, &[CMD_WRITE_STATUS1, 0]);
    assert_eq!(status(&part, CMD_READ_STATUS1), SR1_SRP);
}

// ---------------------------------------------------------------------------
// block protection
// ---------------------------------------------------------------------------

#[test]
fn block_protection_refuses_a_program_inside_the_protected_range() {
    let part = new_part();
    write_enable(&part);
    // BP = 1 with TB clear protects the top sixty-fourth of the array.
    frame(&part, &[CMD_WRITE_STATUS1, 1 << SR1_BP_SHIFT]);
    let protected = (SIZE - SIZE / 64) as u32;
    program(&part, protected, &[0x00]);
    assert_eq!(read(&part, protected, 1), [0xff], "refused");
    program(&part, protected - 1, &[0x00]);
    assert_eq!(read(&part, protected - 1, 1), [0x00], "just below, allowed");
}

#[test]
fn the_complement_bit_turns_the_protected_range_inside_out() {
    let part = new_part();
    write_enable(&part);
    frame(&part, &[CMD_WRITE_STATUS1, 1 << SR1_BP_SHIFT, SR2_CMP]);
    let boundary = (SIZE - SIZE / 64) as u32;
    program(&part, boundary, &[0x00]);
    assert_eq!(read(&part, boundary, 1), [0x00], "the top is now free");
    program(&part, 0, &[0x00]);
    assert_eq!(read(&part, 0, 1), [0xff], "and everything below is not");
}

#[test]
fn block_protection_refuses_an_erase_too() {
    let part = new_part();
    program(&part, 0, &[0x00]);
    write_enable(&part);
    // TB set puts the protected range at the bottom.
    frame(&part, &[CMD_WRITE_STATUS1, (1 << SR1_BP_SHIFT) | SR1_TB]);
    erase(&part, CMD_SECTOR_ERASE, 0);
    assert_eq!(read(&part, 0, 1), [0x00], "the sector survived");
}

// ---------------------------------------------------------------------------
// addressing and reset
// ---------------------------------------------------------------------------

#[test]
fn four_byte_address_mode_changes_how_many_bytes_a_frame_carries() {
    let part = new_part();
    program(&part, 0x30, &[0x77]);
    frame(&part, &[CMD_ENTER_4B]);
    assert_eq!(status(&part, CMD_READ_STATUS3) & SR3_ADS, SR3_ADS);
    let out = frame(&part, &[CMD_READ, 0, 0, 0, 0x30, 0]);
    assert_eq!(out[5], 0x77, "four address bytes now");
    frame(&part, &[CMD_EXIT_4B]);
    assert_eq!(read(&part, 0x30, 1), [0x77], "three again");
}

#[test]
fn the_status_register_lock_bit_holds_until_a_power_cycle() {
    let part = new_part();
    write_enable(&part);
    // `SRL` set is §7.1.1's power-supply lock-down.
    frame(&part, &[CMD_WRITE_STATUS1, SR1_SEC, SR2_SRL]);
    write_enable(&part);
    frame(&part, &[CMD_WRITE_STATUS1, 0]);
    assert_eq!(status(&part, CMD_READ_STATUS1) & SR1_SEC, SR1_SEC, "frozen");
    // Nothing but a power cycle lifts it, and a power cycle is a cold reset.
    part.reset(ResetKind::Cold);
    assert_eq!(status(&part, CMD_READ_STATUS2) & SR2_SRL, 0);
}

#[test]
fn the_software_reset_needs_both_halves_and_keeps_the_contents() {
    let part = new_part();
    program(&part, 0, &[0x5a]);
    frame(&part, &[CMD_ENTER_4B]);
    // `99h` on its own does nothing: only `66h` immediately before it arms it.
    frame(&part, &[CMD_RESET]);
    assert_eq!(status(&part, CMD_READ_STATUS3) & SR3_ADS, SR3_ADS);
    frame(&part, &[CMD_ENABLE_RESET]);
    frame(&part, &[CMD_RESET]);
    assert_eq!(status(&part, CMD_READ_STATUS3) & SR3_ADS, 0);
    assert_eq!(read(&part, 0, 1), [0x5a], "a reset is not an erase");
}

#[test]
fn a_device_reset_keeps_the_contents() {
    let part = new_part();
    program(&part, 0x40, &[0x13]);
    part.reset(ResetKind::Cold);
    assert_eq!(read(&part, 0x40, 1), [0x13], "flash is non-volatile");
    assert_eq!(status(&part, CMD_READ_STATUS1) & SR1_WEL, 0);
}

// ---------------------------------------------------------------------------
// the two link models
// ---------------------------------------------------------------------------

/// Clock a frame in bit by bit through [`SlavePins`], the way an SPI
/// controller in `link = "wired"` or a guest bit-banging GPIO would.
fn wired_frame(part: &SpiNor, bytes: &[u8]) -> Vec<u8> {
    let pins = part.pins();
    pins.drive(spi_pin::CS, Level::Low);
    let mut out = Vec::new();
    for byte in bytes {
        let mut got = 0u8;
        for bit in (0..8).rev() {
            // Mode 0: MOSI settles, SCK rises and samples, SCK falls.
            pins.drive(spi_pin::MOSI, Level::from_bool(byte >> bit & 1 != 0));
            got = (got << 1) | u8::from(pins.miso_level().is_high());
            pins.drive(spi_pin::SCK, Level::High);
            pins.drive(spi_pin::SCK, Level::Low);
        }
        out.push(got);
    }
    pins.drive(spi_pin::CS, Level::High);
    out
}

#[test]
fn a_bit_banged_frame_says_exactly_what_a_transactional_one_says() {
    // The claim `docs/buses/low-speed.md` asks for, at the device rather than
    // the fabric: one model, two links, the same answer. Programming through
    // the wires and reading back transactionally is the strong form of it.
    let wired = new_part();
    let transactional = new_part();

    let seq: &[&[u8]] = &[
        &[CMD_JEDEC_ID, 0, 0, 0],
        &[CMD_WRITE_ENABLE],
        &[CMD_PAGE_PROGRAM, 0, 0x02, 0x00, 0xde, 0xad, 0xbe, 0xef],
        &[CMD_READ_STATUS1, 0],
        &[CMD_READ, 0, 0x02, 0x00, 0, 0, 0, 0],
    ];
    for words in seq {
        assert_eq!(
            wired_frame(&wired, words),
            frame(&transactional, words),
            "frame {words:02x?}"
        );
    }
    assert_eq!(wired.contents(), transactional.contents());
}

#[test]
fn the_bus_routes_a_word_to_the_chip_select_the_part_answers_on() {
    let bus = SpiBus::new();
    let part = new_part();
    bus.attach(ChipSelect(3), part.slave())
        .expect("cs3 is free");
    // Nothing selected reads as the pull-up, which is what firmware probing an
    // empty bus sees.
    assert_eq!(bus.transfer(u32::from(CMD_JEDEC_ID)), 0xffff_ffff);
    bus.select(Some(ChipSelect(3)));
    assert_eq!(bus.transfer(u32::from(CMD_JEDEC_ID)) as u8, IDLE_BYTE);
    assert_eq!(bus.transfer(0) as u8, WINBOND);
    bus.select(None);
}

// ---------------------------------------------------------------------------
// snapshots
// ---------------------------------------------------------------------------

fn snapshot(part: &SpiNor) -> Vec<u8> {
    let mut shape = MachineShape::new();
    shape.add_device("nor", CLASS.name).expect("a fresh shape");
    let mut w = StateWriter::new(shape);
    {
        let mut chunk = w
            .chunk("nor", CLASS.name, CLASS.version)
            .expect("one chunk");
        part.save(&mut chunk).expect("the flash saves");
    }
    w.to_vec().expect("a snapshot")
}

fn restore(part: &SpiNor, bytes: &[u8]) {
    let reader = StateReader::new(bytes).expect("a snapshot");
    let chunk = reader
        .load("nor", CLASS.name, CLASS.version, &Migrations::new())
        .expect("the chunk is there");
    part.load(&mut chunk.reader()).expect("the flash loads");
}

#[test]
fn a_snapshot_round_trips_to_an_identical_chunk() {
    let part = new_part();
    program(&part, 0x800, b"state");
    write_enable(&part);
    let first = snapshot(&part);

    let other = new_part();
    restore(&other, &first);
    assert_eq!(snapshot(&other), first, "identical bytes, identical hash");
    assert_eq!(other.contents(), part.contents());
    assert_eq!(other.status(1) & SR1_WEL, SR1_WEL, "and the latch");
}

#[test]
fn a_snapshot_taken_mid_frame_carries_the_staged_page_program() {
    let part = new_part();
    let slave = part.slave();
    // A page program whose data has been latched and whose chip select has not
    // risen. The write has *not* happened yet, and a snapshot that restored
    // this as idle would swallow it.
    write_enable(&part);
    slave.select(true);
    for b in [CMD_PAGE_PROGRAM, 0, 0, 0x10, 0x0f, 0xf0] {
        exchange(&*slave, u32::from(b));
    }
    let bytes = snapshot(&part);
    assert_eq!(read(&part, 0x10, 2), [0xff, 0xff], "not yet committed");

    let other = new_part();
    restore(&other, &bytes);
    // Finish the frame on the *restored* part: the rising edge commits what
    // the snapshot carried.
    other.slave().select(false);
    assert_eq!(read(&other, 0x10, 2), [0x0f, 0xf0]);
}

#[test]
fn a_snapshot_taken_between_an_erase_and_its_chip_select_carries_it() {
    let part = new_part();
    program(&part, 0x2000, &[0x00]);
    write_enable(&part);
    let slave = part.slave();
    slave.select(true);
    for b in [CMD_SECTOR_ERASE, 0, 0x20, 0x00] {
        exchange(&*slave, u32::from(b));
    }
    let bytes = snapshot(&part);
    assert_eq!(read(&part, 0x2000, 1), [0x00], "the erase has not run");

    let other = new_part();
    restore(&other, &bytes);
    other.slave().select(false);
    assert_eq!(read(&other, 0x2000, 1), [0xff], "and now it has");
}

#[test]
fn a_snapshot_from_a_differently_sized_part_is_refused() {
    let big = part_with(Props::new().with("size", Value::Size(SIZE)));
    let bytes = snapshot(&big);
    let small = part_with(Props::new().with("size", Value::Size(SIZE / 2)));
    let reader = StateReader::new(&bytes).expect("a snapshot");
    let chunk = reader
        .load("nor", CLASS.name, CLASS.version, &Migrations::new())
        .expect("the chunk is there");
    let e = small
        .load(&mut chunk.reader())
        .expect_err("1 MiB is not 512 KiB")
        .to_string();
    assert!(e.contains("1048576") && e.contains("524288"), "{e}");
}

// ---------------------------------------------------------------------------
// construction
// ---------------------------------------------------------------------------

#[test]
fn a_size_that_is_not_a_power_of_two_is_refused() {
    let e = SpiNor::new(&Props::new().with("size", Value::Size(3 * BLOCK)))
        .expect_err("the capacity byte is a logarithm")
        .to_string();
    assert!(e.contains("logarithm"), "{e}");
}

#[test]
fn an_image_larger_than_the_part_is_refused() {
    use crate::core::props::Media;
    let image = Media::new("flash", alloc::vec![0u8; (SIZE + 1) as usize]);
    let e = SpiNor::new(
        &Props::new()
            .with("size", Value::Size(SIZE))
            .with("image", Value::Media(image)),
    )
    .expect_err("it does not fit")
    .to_string();
    assert!(e.contains("does not fit") || e.contains("flash is"), "{e}");
}

#[test]
fn a_bound_image_is_the_initial_contents_and_the_rest_stays_erased() {
    use crate::core::props::Media;
    let image = Media::new("flash", alloc::vec![0xa5u8; 8]);
    let part = part_with(
        Props::new()
            .with("size", Value::Size(SIZE))
            .with("image", Value::Media(image)),
    );
    assert_eq!(read(&part, 0, 8), [0xa5; 8]);
    assert_eq!(read(&part, 8, 1), [0xff]);
}
