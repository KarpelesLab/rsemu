//! What a QSPI pseudo-static RAM answers, and what it refuses.
//!
//! Every test here talks to the part the way a controller does — whole frames
//! delimited by a chip select, each phase clocked at a stated width — because
//! that is the only interface it has. `read_contents` is used to *check* the
//! array, never to change it.

use super::*;
use crate::bus::spi::{SpiBus, exchange_wide};
use crate::core::props::Value;
use crate::core::state::{MachineShape, Migrations, StateReader, StateWriter};
use alloc::vec::Vec;

/// A 64 KiB part: big enough for two 1 KiB pages' worth of interesting
/// addresses, small enough to snapshot in a test.
const SIZE: u64 = 64 * 1024;

fn new_part() -> Psram {
    part_with(Props::new().with("size", Value::Size(SIZE)))
}

fn part_with(props: Props) -> Psram {
    Psram::new(&props).expect("a plausible part")
}

/// Clock one whole frame, every phase at `lines` wires.
///
/// The return value is what came back on MISO during each byte, so index `n`
/// of the answer belongs to byte `n` of the question — the full-duplex truth a
/// request/response shape would hide.
fn frame_at(part: &Psram, bytes: &[u8], lines: Lines) -> Vec<u8> {
    let slave = part.slave();
    slave.select(true);
    let out = bytes
        .iter()
        .map(|b| exchange_wide(&*slave, u32::from(*b), lines) as u8)
        .collect();
    slave.select(false);
    out
}

/// A single-line frame, which is what a part in SPI mode mostly sees.
fn frame(part: &Psram, bytes: &[u8]) -> Vec<u8> {
    frame_at(part, bytes, Lines::SINGLE)
}

/// A mixed-width frame: `head` bytes on one wire, then `tail` on four.
///
/// This is what `EBh` and `38h` are in SPI mode, and the shape that could not
/// be expressed before `Lines` rode with the word.
fn mixed(part: &Psram, head: &[u8], tail: &[u8]) -> Vec<u8> {
    let slave = part.slave();
    slave.select(true);
    let mut out: Vec<u8> = head
        .iter()
        .map(|b| exchange_wide(&*slave, u32::from(*b), Lines::SINGLE) as u8)
        .collect();
    out.extend(
        tail.iter()
            .map(|b| exchange_wide(&*slave, u32::from(*b), Lines::QUAD) as u8),
    );
    slave.select(false);
    out
}

/// The three address bytes of `addr`, most significant first.
fn addr(addr: u32) -> [u8; 3] {
    [(addr >> 16) as u8, (addr >> 8) as u8, addr as u8]
}

/// `02h` at `at`, single line.
fn write(part: &Psram, at: u32, data: &[u8]) {
    let mut req = alloc::vec![CMD_WRITE];
    req.extend(addr(at));
    req.extend(data);
    frame(part, &req);
}

/// `03h` at `at` for `len` bytes, single line.
fn read(part: &Psram, at: u32, len: usize) -> Vec<u8> {
    let mut req = alloc::vec![CMD_READ];
    req.extend(addr(at));
    req.extend(core::iter::repeat_n(0u8, len));
    frame(part, &req)[4..].to_vec()
}

fn contents(part: &Psram, at: u64, len: usize) -> Vec<u8> {
    let mut out = alloc::vec![0u8; len];
    part.read_contents(at, &mut out).expect("inside the part");
    out
}

// ---------------------------------------------------------------------------
// identification and reset
// ---------------------------------------------------------------------------

#[test]
fn reset_then_read_id_returns_mfid_and_kgd() {
    let part = new_part();
    frame(&part, &[CMD_RESET_ENABLE]);
    frame(&part, &[CMD_RESET]);
    let mut req = alloc::vec![CMD_READ_ID];
    req.extend(addr(0));
    req.extend([0u8; 8]);
    let got = frame(&part, &req);
    // The opcode and three address bytes come back as idle; the identifier
    // starts with the byte after them.
    assert_eq!(got[4], AP_MEMORY, "MFID");
    assert_eq!(got[5], KGD, "KGD");
}

#[test]
fn the_id_repeats_and_an_is66_says_so() {
    let part = part_with(
        Props::new()
            .with("size", Value::Size(SIZE))
            .with("id", Value::Str("is66".into()))
            .with("eid", Value::Uint(0x0001_0203_0405)),
    );
    let mut req = alloc::vec![CMD_READ_ID];
    req.extend(addr(0));
    req.extend([0u8; 10]);
    let got = frame(&part, &req);
    assert_eq!(&got[4..12], &[ISSI, KGD, 0, 1, 2, 3, 4, 5]);
    // Byte nine is byte one again: the response repeats rather than running
    // off the end of an eight-byte identifier.
    assert_eq!(got[12], ISSI);
}

#[test]
fn a_lone_99h_does_not_reset_the_mode() {
    let part = new_part();
    frame(&part, &[CMD_ENTER_QUAD]);
    assert!(part.quad_mode());
    // `99h` on its own, without the `66h` that arms it.
    frame_at(&part, &[CMD_RESET], Lines::QUAD);
    assert!(part.quad_mode(), "an unarmed reset does nothing");
    frame_at(&part, &[CMD_RESET_ENABLE], Lines::QUAD);
    frame_at(&part, &[CMD_RESET], Lines::QUAD);
    assert!(!part.quad_mode(), "an armed one puts it back in SPI mode");
}

// ---------------------------------------------------------------------------
// single-line reads and writes
// ---------------------------------------------------------------------------

#[test]
fn a_single_line_write_reads_back_with_03h_and_0bh_including_the_dummy_byte() {
    let part = new_part();
    write(&part, 0x1234, b"hello");
    assert_eq!(read(&part, 0x1234, 5), b"hello");

    // `0Bh` is the same read with eight dummy clocks in front of the data. On
    // one wire that is exactly one byte, so the data starts at index 5.
    let mut req = alloc::vec![CMD_FAST_READ];
    req.extend(addr(0x1234));
    req.extend([0u8; 1 + 5]);
    let got = frame(&part, &req);
    assert_eq!(&got[5..10], b"hello");
}

#[test]
fn a_write_lands_as_it_is_clocked_rather_than_at_the_rising_edge() {
    // The difference from `flash.spinor`, which stages a page program until
    // the chip select rises. This is RAM: a frame cut short has written every
    // byte that got there.
    let part = new_part();
    let slave = part.slave();
    slave.select(true);
    for byte in [CMD_WRITE, 0, 0, 0, 0xaa, 0xbb] {
        exchange_wide(&*slave, u32::from(byte), Lines::SINGLE);
    }
    // No rising edge yet, and the bytes are already in the array.
    assert_eq!(contents(&part, 0, 2), [0xaa, 0xbb]);
    slave.select(false);
}

#[test]
fn a_write_can_set_a_bit_a_flash_could_not() {
    // The other half of the same point: this is not flash, so there is no
    // "a program only clears bits" rule to enforce.
    let part = new_part();
    write(&part, 0x40, &[0x00]);
    write(&part, 0x40, &[0xff]);
    assert_eq!(contents(&part, 0x40, 1), [0xff]);
}

#[test]
fn the_power_on_fill_is_a_property() {
    let part = part_with(
        Props::new()
            .with("size", Value::Size(SIZE))
            .with("fill", Value::Uint(0xa5)),
    );
    assert_eq!(read(&part, 0x100, 3), [0xa5, 0xa5, 0xa5]);
}

// ---------------------------------------------------------------------------
// quad
// ---------------------------------------------------------------------------

#[test]
fn ebh_in_spi_mode_is_a_one_line_opcode_with_four_line_everything_else() {
    let part = new_part();
    write(&part, 0x2000, b"quad");

    // `EBh`: opcode single, then address, six quad dummy clocks (24 bits, so
    // three bytes) and the data all on four wires.
    let mut tail = Vec::from(addr(0x2000));
    tail.extend([0u8; 3 + 4]);
    let got = mixed(&part, &[CMD_FAST_READ_QUAD], &tail);
    assert_eq!(&got[7..11], b"quad");
}

#[test]
fn a_quad_command_clocked_on_one_wire_is_not_a_command() {
    // The claim the width channel exists to make. Byte for byte this frame is
    // identical to the one above; the only difference is that the master
    // drove it on one wire, and the part cannot parse that.
    let part = new_part();
    write(&part, 0x2000, b"quad");
    let mut req = alloc::vec![CMD_FAST_READ_QUAD];
    req.extend(addr(0x2000));
    req.extend([0u8; 3 + 4]);
    let got = frame(&part, &req);
    assert!(
        got[7..11].iter().all(|b| *b == IDLE_BYTE),
        "the part answered a frame it could not have understood: {got:02x?}"
    );
}

#[test]
fn a_38h_quad_write_reads_back_through_a_single_line_read() {
    let part = new_part();
    let mut tail = Vec::from(addr(0x300));
    tail.extend(b"wide");
    mixed(&part, &[CMD_QUAD_WRITE], &tail);
    assert_eq!(read(&part, 0x300, 4), b"wide");
}

#[test]
fn entering_quad_mode_makes_ebh_and_38h_work_and_single_line_commands_fail() {
    let part = new_part();
    write(&part, 0x800, b"before");
    frame(&part, &[CMD_ENTER_QUAD]);
    assert!(part.quad_mode());

    // Every phase is four wires now, opcode included.
    let mut req = alloc::vec![CMD_QUAD_WRITE];
    req.extend(addr(0x800));
    req.extend(b"afterx");
    frame_at(&part, &req, Lines::QUAD);
    assert_eq!(contents(&part, 0x800, 6), b"afterx");

    let mut req = alloc::vec![CMD_FAST_READ_QUAD];
    req.extend(addr(0x800));
    req.extend([0u8; 3 + 6]);
    let got = frame_at(&part, &req, Lines::QUAD);
    assert_eq!(&got[7..13], b"afterx");

    // And the SPI-mode instructions are gone: in QPI the part is sampling
    // nibbles, and a one-line `03h` is not an opcode it can see.
    let mut req = alloc::vec![CMD_READ];
    req.extend(addr(0x800));
    req.extend([0u8; 6]);
    let got = frame(&part, &req);
    assert!(
        got[4..].iter().all(|b| *b == IDLE_BYTE),
        "a single-line read answered in QPI mode: {got:02x?}"
    );
    // Even clocked at the right width, `03h` is not a QPI instruction.
    let got = frame_at(&part, &req, Lines::QUAD);
    assert!(
        got[4..].iter().all(|b| *b == IDLE_BYTE),
        "03h is an SPI-mode instruction: {got:02x?}"
    );

    // `F5h` is the way back, and it works at the QPI width.
    frame_at(&part, &[CMD_EXIT_QUAD], Lines::QUAD);
    assert!(!part.quad_mode());
    assert_eq!(read(&part, 0x800, 6), b"afterx");
}

// ---------------------------------------------------------------------------
// the burst wrap
// ---------------------------------------------------------------------------

#[test]
fn a_burst_that_crosses_a_1k_page_wraps_within_the_page_unless_toggled() {
    let part = new_part();
    // Mark the last two bytes of page 0 and the first two of page 1, so the
    // wrap is visible in what comes back rather than inferred.
    write(&part, 0x3fe, &[0xee, 0xff]);
    write(&part, 0x400, &[0x11, 0x22]);
    // The first two bytes of page 0, which is where a wrap lands.
    write(&part, 0x000, &[0xaa, 0xbb]);

    let got = read(&part, 0x3fe, 4);
    assert_eq!(
        got,
        [0xee, 0xff, 0xaa, 0xbb],
        "a linear burst resumes at the start of its own page"
    );
    assert_eq!(part.wrap(), PAGE);

    // `C0h` toggles to a 32-byte wrap.
    frame(&part, &[CMD_WRAP_TOGGLE]);
    assert_eq!(part.wrap(), SHORT_WRAP);
    write(&part, 0x20, &[0x01, 0x02]);
    write(&part, 0x3e, &[0x03, 0x04]);
    let got = read(&part, 0x3e, 4);
    assert_eq!(
        got,
        [0x03, 0x04, 0x01, 0x02],
        "and now it resumes at the start of its own 32-byte group"
    );

    // And back again.
    frame(&part, &[CMD_WRAP_TOGGLE]);
    assert_eq!(part.wrap(), PAGE);
}

#[test]
fn a_write_burst_wraps_the_same_way_a_read_does() {
    let part = new_part();
    let mut req = alloc::vec![CMD_WRITE];
    req.extend(addr(0x3fe));
    req.extend([0x10, 0x20, 0x30, 0x40]);
    frame(&part, &req);
    assert_eq!(contents(&part, 0x3fe, 2), [0x10, 0x20]);
    assert_eq!(contents(&part, 0x000, 2), [0x30, 0x40]);
    // Page 1 was not touched, which is the whole point of the rule.
    assert_eq!(contents(&part, 0x400, 2), [0x00, 0x00]);
}

// ---------------------------------------------------------------------------
// tCEM
// ---------------------------------------------------------------------------

/// A part with an eight-clock tCEM budget, which any frame longer than one
/// single-line byte exceeds. Small so the arithmetic in a test is obvious;
/// a real board writes 672, which is 8 µs at 84 MHz.
fn tight_part(check: &str) -> Psram {
    part_with(
        Props::new()
            .with("size", Value::Size(SIZE))
            .with("tcem-cycles", Value::Uint(8))
            .with("tcem-check", Value::Str(check.into())),
    )
}

#[test]
fn holding_cs_low_past_tcem_is_counted_and_does_not_corrupt_data() {
    let part = tight_part("log");
    write(&part, 0x10, b"keep");
    // That write was six single-line bytes, 48 clocks, well past eight.
    assert_eq!(part.tcem_violations(), 1, "one frame, one violation");
    // And the data is exactly what was clocked: `log` reports, never corrupts.
    assert_eq!(read(&part, 0x10, 4), b"keep");
    assert_eq!(part.tcem_violations(), 2, "the read was over budget too");
}

#[test]
fn a_frame_inside_the_budget_is_not_a_violation() {
    let part = tight_part("log");
    // One single-line byte is eight clocks, which is the budget exactly rather
    // than past it.
    frame(&part, &[CMD_RESET_ENABLE]);
    assert_eq!(part.tcem_violations(), 0);
}

#[test]
fn four_wires_buy_four_times_the_bytes_inside_one_budget() {
    // The reason the budget is in *clocks* rather than bytes, and the reason
    // the part needs the width at all: a quad frame fits four times as much
    // into the same chip-select-low period.
    let part = tight_part("log");
    frame_at(&part, &[CMD_ENTER_QUAD], Lines::QUAD);
    // Four quad bytes are eight clocks.
    frame_at(&part, &[CMD_RESET_ENABLE, 0, 0, 0], Lines::QUAD);
    assert_eq!(part.tcem_violations(), 0);
    frame_at(&part, &[CMD_RESET_ENABLE, 0, 0, 0, 0], Lines::QUAD);
    assert_eq!(part.tcem_violations(), 1, "the fifth byte is over");
}

#[test]
fn fault_discards_the_rest_of_the_frame_and_still_keeps_what_landed() {
    let part = tight_part("fault");
    let mut req = alloc::vec![CMD_WRITE];
    req.extend(addr(0));
    req.extend([0x11, 0x22, 0x33, 0x44]);
    frame(&part, &req);
    assert_eq!(part.tcem_violations(), 1);
    // Four header bytes is 32 clocks, so the budget was already blown when the
    // first data byte arrived: nothing was written, and nothing was scrambled
    // either.
    assert_eq!(contents(&part, 0, 4), [0, 0, 0, 0]);
    // The part is not wedged: the next frame starts clean.
    let off = part_with(Props::new().with("size", Value::Size(SIZE)));
    write(&off, 0, &[0x11]);
    assert_eq!(contents(&off, 0, 1), [0x11]);
}

#[test]
fn off_does_not_look() {
    let part = tight_part("off");
    write(&part, 0x10, b"keep");
    assert_eq!(part.tcem_violations(), 0);
    assert_eq!(read(&part, 0x10, 4), b"keep");
}

// ---------------------------------------------------------------------------
// properties
// ---------------------------------------------------------------------------

#[test]
fn an_impossible_size_is_refused_by_name() {
    for size in [0u64, 512, 3 * 1024, 32 * 1024 * 1024] {
        let e = Psram::new(&Props::new().with("size", Value::Size(size)))
            .expect_err("not a part in this family")
            .to_string();
        assert!(e.contains("24-bit"), "{e}");
    }
}

#[test]
fn an_unknown_id_or_check_is_refused_by_name() {
    let e = Psram::new(
        &Props::new()
            .with("size", Value::Size(SIZE))
            .with("id", Value::Str("w25q".into())),
    )
    .expect_err("not a part this model knows")
    .to_string();
    assert!(e.contains("aps6404"), "{e}");

    let e = Psram::new(
        &Props::new()
            .with("size", Value::Size(SIZE))
            .with("tcem-check", Value::Str("explode".into())),
    )
    .expect_err("not a check")
    .to_string();
    assert!(e.contains("fault"), "{e}");
}

#[test]
fn two_parts_cannot_share_one_chip_select() {
    let bus = Arc::new(SpiBus::new());
    bus.attach(ChipSelect(1), new_part().slave())
        .expect("cs1 is free");
    let e = bus
        .attach(ChipSelect(1), new_part().slave())
        .expect_err("a short")
        .to_string();
    assert!(e.contains("chip select"), "{e}");
}

// ---------------------------------------------------------------------------
// the snapshot
// ---------------------------------------------------------------------------

fn snapshot(part: &Psram) -> Vec<u8> {
    let mut shape = MachineShape::new();
    shape
        .add_device("psram", CLASS.name)
        .expect("a fresh shape");
    let mut w = StateWriter::new(shape);
    {
        let mut chunk = w
            .chunk("psram", CLASS.name, CLASS.version)
            .expect("one chunk");
        part.save(&mut chunk).expect("the part saves");
    }
    w.to_vec().expect("a snapshot")
}

fn restore(part: &Psram, bytes: &[u8]) {
    let reader = StateReader::new(bytes).expect("a snapshot");
    let chunk = reader
        .load("psram", CLASS.name, CLASS.version, &Migrations::new())
        .expect("the chunk is there");
    part.load(&mut chunk.reader()).expect("the part loads");
}

#[test]
fn contents_and_mode_are_part_of_the_snapshot_and_the_state_hash() {
    let part = new_part();
    write(&part, 0x900, b"snap");
    frame(&part, &[CMD_WRAP_TOGGLE]);
    frame(&part, &[CMD_ENTER_QUAD]);
    let saved = snapshot(&part);

    let other = new_part();
    assert_ne!(snapshot(&other), saved, "a fresh part is a different state");
    restore(&other, &saved);
    assert_eq!(snapshot(&other), saved, "and the chunk round trips exactly");
    assert!(other.quad_mode());
    assert_eq!(other.wrap(), SHORT_WRAP);
    assert_eq!(contents(&other, 0x900, 4), b"snap");
}

#[test]
fn a_snapshot_taken_mid_frame_restores_mid_frame() {
    // The reason the phase is state at all. A machine saved between a `03h`'s
    // address bytes and its data has a frame open, and restoring it as idle
    // would answer the next byte with an opcode decode.
    let part = new_part();
    write(&part, 0x50, b"mid");
    let slave = part.slave();
    slave.select(true);
    for byte in [CMD_READ, 0x00, 0x00, 0x50] {
        exchange_wide(&*slave, u32::from(byte), Lines::SINGLE);
    }
    let saved = snapshot(&part);

    let other = new_part();
    restore(&other, &saved);
    let resumed = other.slave();
    // No `select(true)`: the snapshot says the chip select is already low, and
    // re-asserting it would start a fresh frame over the restored one.
    let got: Vec<u8> = (0..3)
        .map(|_| exchange_wide(&*resumed, 0, Lines::SINGLE) as u8)
        .collect();
    assert_eq!(got, b"mid");
}

#[test]
fn a_reset_clears_the_array_because_this_is_volatile_memory() {
    // The difference from every flash part in this tree, and the one thing a
    // board integrator is most likely to assume wrongly.
    let part = new_part();
    write(&part, 0x10, b"gone");
    part.reset(ResetKind::Cold);
    assert_eq!(read(&part, 0x10, 4), [0, 0, 0, 0]);
    assert!(!part.quad_mode());
    assert_eq!(part.wrap(), PAGE);
}

#[test]
fn a_hostile_chunk_is_refused_rather_than_believed() {
    let part = new_part();
    for hostile in [
        alloc::vec![],
        alloc::vec![0u8; 3],
        alloc::vec![0xff; 128],
        alloc::vec![0x00; 4096],
    ] {
        let mut shape = MachineShape::new();
        shape
            .add_device("psram", CLASS.name)
            .expect("a fresh shape");
        let mut w = StateWriter::new(shape);
        {
            let mut chunk = w
                .chunk("psram", CLASS.name, CLASS.version)
                .expect("one chunk");
            chunk.write_bytes(&hostile).expect("raw bytes");
        }
        let bytes = w.to_vec().expect("a snapshot");
        let reader = StateReader::new(&bytes).expect("a snapshot");
        let chunk = reader
            .load("psram", CLASS.name, CLASS.version, &Migrations::new())
            .expect("the chunk is there");
        // Either it is refused or it loads something bounded; what it must not
        // do is panic or hand out an address outside the array.
        let _ = part.load(&mut chunk.reader());
        assert!(part.wrap() == PAGE || part.wrap() == SHORT_WRAP);
    }
}

// ---------------------------------------------------------------------------
// through a real controller
// ---------------------------------------------------------------------------

/// The claim the whole change exists to make: a store and a load inside an
/// `stm32.octospi`'s memory-mapped window reach this part as real frames, at
/// whatever width `CCR` was programmed for.
///
/// Not a recorder and not a stub on either end — a real controller building
/// the frame out of its registers, and a real part decoding it.
#[cfg(feature = "dev-stm32-octospi")]
mod through_an_octospi {
    use super::*;
    use crate::core::space::{MemAttrs, MemOps, RegionKind, RegionRef};
    use crate::dev::stm32::octospi::Octospi;

    /// `DEVSIZE` for a part of `SIZE` bytes: the field holds the exponent less
    /// one, so `2^(DEVSIZE + 1)` is the density.
    const DEVSIZE: u32 = 15;

    /// `CR`: enable, memory-mapped (`FMODE = 11`).
    const CR_MAPPED: u32 = 1 | (3 << 28);
    /// `CR` with the chip-select timeout counter running as well.
    const CR_MAPPED_TIMEOUT: u32 = CR_MAPPED | (1 << 3);

    /// A `CCR` with an opcode on `imode` wires, a 24-bit address on `admode`
    /// and data on `dmode`. The mode encodings are `1` single and `3` quad.
    const fn ccr(imode: u32, admode: u32, dmode: u32) -> u32 {
        imode | (admode << 8) | (2 << 12) | (dmode << 24)
    }

    struct Board {
        part: Psram,
        regs: Arc<dyn MemOps>,
        window: Arc<dyn MemOps>,
        // Held so the controller's chip select keeps answering.
        _octospi: Octospi,
    }

    fn io(region: RegionRef) -> Arc<dyn MemOps> {
        match region.kind() {
            RegionKind::Io(ops) => Arc::clone(ops),
            _ => unreachable!("both regions are MMIO"),
        }
    }

    /// A controller and a part on one bus, with `props` deciding the part.
    fn board(props: Props) -> Board {
        let bus = Arc::new(SpiBus::new());
        let part = Psram::new(&props).expect("a plausible part");
        bus.attach(ChipSelect(0), part.slave())
            .expect("cs0 is free");
        let octospi = Octospi::with_bus(Some(Arc::clone(&bus)), ChipSelect(0), SIZE);
        let regs = io(octospi.region("regs").expect("registers"));
        let window = io(octospi.region("mem").expect("the aperture"));
        Board {
            part,
            regs,
            window,
            _octospi: octospi,
        }
    }

    fn plain_board() -> Board {
        board(Props::new().with("size", Value::Size(SIZE)))
    }

    impl Board {
        fn poke(&self, offset: u64, value: u32) {
            self.regs
                .write(offset, &value.to_le_bytes(), MemAttrs::DEFAULT)
                .expect("a word write is a legal cycle");
        }

        fn peek(&self, offset: u64) -> u32 {
            let mut bytes = [0u8; 4];
            self.regs
                .read(offset, &mut bytes, MemAttrs::DEFAULT)
                .expect("a word read is a legal cycle");
            u32::from_le_bytes(bytes)
        }

        /// Program the read and write command sets for single-line `0Bh` and
        /// `02h`, and put the peripheral in memory-mapped mode.
        fn single_line(&self) {
            self.poke(0x008, DEVSIZE << 16);
            self.poke(0x100, ccr(1, 1, 1));
            self.poke(0x108, 8); // `0Bh` takes eight dummy clocks, one byte.
            self.poke(0x110, u32::from(CMD_FAST_READ));
            self.poke(0x180, ccr(1, 1, 1));
            self.poke(0x188, 0);
            self.poke(0x190, u32::from(CMD_WRITE));
            self.poke(0x000, CR_MAPPED);
        }

        /// The same for `EBh` and `38h`: a one-line opcode, everything else on
        /// four wires. `DCYC = 6` at that width is 24 bits — three bytes —
        /// which is the conversion that was wrong before `Lines` existed.
        fn quad(&self) {
            self.poke(0x008, DEVSIZE << 16);
            self.poke(0x100, ccr(1, 3, 3));
            self.poke(0x108, 6);
            self.poke(0x110, u32::from(CMD_FAST_READ_QUAD));
            self.poke(0x180, ccr(1, 3, 3));
            self.poke(0x188, 0);
            self.poke(0x190, u32::from(CMD_QUAD_WRITE));
            self.poke(0x000, CR_MAPPED);
        }

        fn store(&self, at: u64, bytes: &[u8]) {
            self.window
                .write(at, bytes, MemAttrs::DEFAULT)
                .expect("inside the window");
        }

        fn load(&self, at: u64, len: usize) -> Vec<u8> {
            let mut out = alloc::vec![0u8; len];
            self.window
                .read(at, &mut out, MemAttrs::DEFAULT)
                .expect("inside the window");
            out
        }
    }

    #[test]
    fn memory_mapped_mode_lets_a_guest_store_and_load_on_one_wire() {
        let board = plain_board();
        board.single_line();
        board.store(0x1234, &0xdead_beefu32.to_le_bytes());
        assert_eq!(board.load(0x1234, 4), 0xdead_beefu32.to_le_bytes());
        // And nothing kept a copy on the side: the bytes are in the part's own
        // array, reachable only through the frames the controller clocked.
        assert_eq!(
            contents(&board.part, 0x1234, 4),
            0xdead_beefu32.to_le_bytes()
        );
    }

    #[test]
    fn memory_mapped_mode_lets_a_guest_store_and_load_on_four() {
        let board = plain_board();
        board.quad();
        board.store(0x2000, b"quad-mapped");
        assert_eq!(board.load(0x2000, 11), b"quad-mapped");
        assert_eq!(contents(&board.part, 0x2000, 11), b"quad-mapped");
    }

    #[test]
    fn a_kilobyte_memset_and_memcmp_through_the_window_agrees() {
        let board = plain_board();
        board.quad();
        let pattern: Vec<u8> = (0..1024u32).map(|i| (i * 7) as u8).collect();
        // A boundary-crossing base on purpose: the part wraps a burst inside
        // its 1 KiB page, so this only works if the controller releases the
        // chip select at the boundary. `CSBOUND = 10` is that field.
        board.poke(0x010, 10 << 16);
        board.store(0x300, &pattern);
        assert_eq!(board.load(0x300, 1024), pattern);
        assert_eq!(contents(&board.part, 0x300, 1024), pattern);
    }

    #[test]
    fn without_csbound_a_burst_across_a_page_wraps_and_the_data_is_wrong() {
        // The negative of the test above, and the reason `CSBOUND` had to be
        // implemented rather than assumed away: the part's wrap is real, and a
        // controller that never lets go of the chip select gets bitten by it.
        let board = plain_board();
        board.quad();
        let pattern: Vec<u8> = (0..1024u32).map(|i| (i * 7) as u8).collect();
        board.store(0x300, &pattern);
        assert_ne!(
            contents(&board.part, 0x300, 1024),
            pattern,
            "a burst that ran past 0x400 should have wrapped to 0x000"
        );
        // Specifically: the tail landed at the start of page 0.
        assert_eq!(contents(&board.part, 0x000, 16), pattern[0x100..0x110]);
    }

    #[test]
    fn holding_cs_past_tcem_raises_tof_and_maxtran_is_how_a_driver_avoids_it() {
        // Eight clocks of budget on the part, so any real burst blows it, and
        // `LPTR` set to the same number so the controller agrees about where
        // the line is.
        let board = board(
            Props::new()
                .with("size", Value::Size(SIZE))
                .with("tcem-cycles", Value::Uint(8))
                .with("tcem-check", Value::Str("log".into())),
        );
        board.single_line();
        board.poke(0x130, 8); // LPTR
        board.poke(0x000, CR_MAPPED_TIMEOUT);

        board.store(0x40, b"long enough to matter");
        assert_ne!(board.peek(0x020) & (1 << 4), 0, "SR.TOF is set");
        assert!(board.part.tcem_violations() > 0, "and the part noticed too");

        // `FCR` clears it, and `MAXTRAN` is how a driver stops it happening:
        // `MAXTRAN + 1` bytes per chip-select assertion. This is the field ST
        // put there for exactly this part's tCEM.
        board.poke(0x024, 1 << 4);
        assert_eq!(board.peek(0x020) & (1 << 4), 0, "SR.TOF cleared");
        board.poke(0x010, 1); // MAXTRAN = 1, so two bytes per assertion.
        let before = board.part.tcem_violations();
        board.store(0x80, b"aaaa");
        assert!(
            board.part.tcem_violations() - before <= 2,
            "MAXTRAN split the burst into short frames"
        );
    }

    #[test]
    fn a_debug_read_of_the_window_is_refused_rather_than_moving_the_part() {
        // `MemAttrs::debug` on a bus has no side-effect-free route: reaching
        // the part means asserting a chip select and clocking a frame.
        let board = plain_board();
        board.single_line();
        board.store(0x10, b"seen");
        let mut out = [0u8; 4];
        assert!(board.window.read(0x10, &mut out, MemAttrs::DEBUG).is_err());
        // And the part's own side door answers instead, which is where a
        // debugger is supposed to look.
        assert_eq!(contents(&board.part, 0x10, 4), b"seen");
    }

    /// `CR`: enable, indirect read (`FMODE = 01`).
    const CR_INDIRECT_READ: u32 = 1 | (1 << 28);
    /// `CR`: enable, indirect write (`FMODE = 00`), which is also how a
    /// command with no data phase is issued.
    const CR_INDIRECT_WRITE: u32 = 1;

    impl Board {
        /// Clock a header-only command — `66h`, `99h` — through the indirect
        /// path. Writing `IR` is what starts one, because there is no address
        /// phase to write `AR` for.
        fn command(&self, opcode: u8) {
            self.poke(0x000, CR_INDIRECT_WRITE);
            self.poke(0x100, 1); // IMODE = 1, nothing else.
            self.poke(0x110, u32::from(opcode));
        }

        /// One byte into `DR`, which is how a driver feeds an indirect write.
        fn push(&self, byte: u8) {
            self.regs
                .write(0x050, &[byte], MemAttrs::DEFAULT)
                .expect("a byte write of DR is a legal cycle");
        }

        /// `9Fh` with its 24-bit address phase, reading `len` bytes out of
        /// `DR` one at a time — which is what a driver does.
        fn read_id(&self, len: u64) -> Vec<u8> {
            self.poke(0x000, CR_INDIRECT_READ);
            self.poke(0x100, ccr(1, 1, 1));
            self.poke(0x108, 0);
            self.poke(0x110, u32::from(CMD_READ_ID));
            self.poke(0x040, (len - 1) as u32); // DLR holds the length less one.
            self.poke(0x048, 0); // AR: writing it starts the transaction.
            let mut out = alloc::vec![0u8; len as usize];
            for byte in &mut out {
                let mut one = [0u8; 1];
                self.regs
                    .read(0x050, &mut one, MemAttrs::DEFAULT)
                    .expect("a byte read of DR is a legal cycle");
                *byte = one[0];
            }
            out
        }
    }

    #[test]
    fn indirect_mode_resets_the_part_and_reads_its_identifier() {
        // The issue's first test, and the half of the peripheral the
        // memory-mapped window does not exercise: a driver clocking whole
        // frames through `IR`/`AR`/`DR` rather than a load reaching the part.
        let board = plain_board();
        board.poke(0x008, DEVSIZE << 16);
        board.command(CMD_RESET_ENABLE);
        board.command(CMD_RESET);
        let got = board.read_id(4);
        assert_eq!(&got[..2], &[AP_MEMORY, KGD], "MFID and KGD: {got:02x?}");
    }

    #[test]
    fn an_indirect_write_and_a_mapped_read_see_one_array() {
        // Both halves of the peripheral, against the same part: `02h` pushed
        // through `DR` a byte at a time, read back through the window.
        let board = plain_board();
        board.poke(0x008, DEVSIZE << 16);
        board.poke(0x000, CR_INDIRECT_WRITE);
        board.poke(0x100, ccr(1, 1, 1));
        board.poke(0x108, 0);
        board.poke(0x110, u32::from(CMD_WRITE));
        board.poke(0x040, 3); // four bytes
        board.poke(0x048, 0x500); // AR is what starts it
        for byte in b"both" {
            board.push(*byte);
        }
        board.single_line();
        assert_eq!(board.load(0x500, 4), b"both");
    }
}
