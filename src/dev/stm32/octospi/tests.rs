//! What a driver programs, and what comes back out of the window.
//!
//! The slave on the far end is a recorder rather than a flash — these tests
//! are about the *frame the peripheral builds*, and a recorder can assert on
//! every byte of it. The end-to-end claim, with a real `flash.spinor` on the
//! other end and a CPU fetching out of the window, is `tests/spi_flash.rs`.

use super::*;
use crate::bus::spi::{Format, SpiSlave};
use crate::core::props::Value;
use crate::core::space::RegionKind;
use crate::core::state::{MachineShape, Migrations, StateReader, StateWriter};
use alloc::vec::Vec;

// ---------------------------------------------------------------------------
// harness
// ---------------------------------------------------------------------------

/// A slave that writes down every byte of every frame and answers from a
/// canned reply stream.
#[derive(Debug)]
struct Recorder {
    state: Mutex<Recorded>,
}

#[derive(Debug, Default)]
struct Recorded {
    /// One vector per frame, in the order the chip select opened them.
    frames: Vec<Vec<u8>>,
    /// What to answer, byte by byte, across the whole run.
    replies: Vec<u8>,
    /// How far into that stream the master has clocked.
    at: usize,
    selected: bool,
}

impl Recorder {
    fn new(replies: &[u8]) -> Arc<Recorder> {
        Arc::new(Recorder {
            state: Mutex::with_rank(
                LockRank::DEVICE,
                Recorded {
                    replies: replies.to_vec(),
                    ..Recorded::default()
                },
            ),
        })
    }

    fn frames(&self) -> Vec<Vec<u8>> {
        self.state.lock().frames.clone()
    }
}

impl SpiSlave for Recorder {
    fn format(&self) -> Format {
        Format::DEFAULT
    }

    fn select(&self, selected: bool) {
        let mut state = self.state.lock();
        if selected {
            state.frames.push(Vec::new());
        }
        state.selected = selected;
    }

    fn transfer(&self, mosi: u32) -> u32 {
        let mut state = self.state.lock();
        if let Some(frame) = state.frames.last_mut() {
            frame.push(mosi as u8);
        }
        let at = state.at;
        state.at += 1;
        u32::from(state.replies.get(at).copied().unwrap_or(0xff))
    }

    fn peek(&self) -> u32 {
        let state = self.state.lock();
        u32::from(state.replies.get(state.at).copied().unwrap_or(0xff))
    }
}

struct Harness {
    octospi: Octospi,
    regs: Arc<dyn MemOps>,
    window: Arc<dyn MemOps>,
    bus: Arc<SpiBus>,
}

/// A peripheral whose window is 1 MiB, with `slave` on chip select 0.
fn harness(slave: Arc<dyn SpiSlave>) -> Harness {
    let bus = Arc::new(SpiBus::new());
    bus.attach(ChipSelect(0), slave).expect("cs0 is free");
    let octospi = Octospi::with_bus(Some(Arc::clone(&bus)), ChipSelect(0), 1024 * 1024);
    let regs = io(octospi.region("regs").expect("registers"));
    let window = io(octospi.region("mem").expect("the aperture"));
    Harness {
        octospi,
        regs,
        window,
        bus,
    }
}

fn io(region: RegionRef) -> Arc<dyn MemOps> {
    match region.kind() {
        RegionKind::Io(ops) => Arc::clone(ops),
        _ => unreachable!("both regions are MMIO"),
    }
}

/// `DEVSIZE` for a 1 MiB part: `2^(19 + 1)`.
const DEVSIZE_1M: u32 = 19 << DCR1_DEVSIZE_SHIFT;

/// A single-line command with a 24-bit address and a data phase — `IMODE = 1`,
/// `ADMODE = 1`, `ADSIZE = 2` (24-bit), `DMODE = 1`.
const CCR_SINGLE_24: u32 =
    1 | (1 << CCR_ADMODE_SHIFT) | (2 << CCR_ADSIZE_SHIFT) | (1 << CCR_DMODE_SHIFT);

/// The same with no address and no data: `06h`-shaped.
const CCR_OPCODE_ONLY: u32 = 1;

impl Harness {
    fn write(&self, offset: u64, value: u32) {
        self.regs
            .write(offset, &value.to_le_bytes(), MemAttrs::DEFAULT)
            .expect("a word write is a legal cycle");
    }

    fn read(&self, offset: u64) -> u32 {
        let mut bytes = [0u8; 4];
        self.regs
            .read(offset, &mut bytes, MemAttrs::DEFAULT)
            .expect("a word read is a legal cycle");
        u32::from_le_bytes(bytes)
    }

    fn byte(&self, offset: u64) -> u8 {
        let mut byte = [0u8; 1];
        self.regs
            .read(offset, &mut byte, MemAttrs::DEFAULT)
            .expect("a byte read is a legal cycle");
        byte[0]
    }

    fn enable(&self, fmode: u32) {
        self.write(0x008, DEVSIZE_1M);
        self.write(0x000, CR_EN | (fmode << CR_FMODE_SHIFT));
    }
}

// ---------------------------------------------------------------------------
// indirect mode
// ---------------------------------------------------------------------------

#[test]
fn a_command_with_no_address_and_no_data_is_one_opcode_byte() {
    let rec = Recorder::new(&[]);
    let h = harness(Arc::clone(&rec) as Arc<dyn SpiSlave>);
    h.enable(FMODE_WRITE);
    h.write(0x100, CCR_OPCODE_ONLY);
    // Writing `IR` is the trigger when there is no address phase — and
    // programming `CCR` before it deliberately is *not*, or the frame would
    // carry whatever instruction happened to be left over.
    assert!(rec.frames().is_empty(), "CCR alone starts nothing");
    h.write(0x110, 0x06);
    assert_eq!(rec.frames(), [[0x06]]);
    assert_eq!(h.read(0x020) & SR_TCF, SR_TCF);
    assert!(!h.octospi.busy(), "and the chip select rose again");
}

#[test]
fn an_indirect_write_sends_the_header_then_the_data_register_bytes() {
    let rec = Recorder::new(&[]);
    let h = harness(Arc::clone(&rec) as Arc<dyn SpiSlave>);
    h.enable(FMODE_WRITE);
    h.write(0x100, CCR_SINGLE_24);
    h.write(0x110, 0x02);
    h.write(0x040, 3 - 1); // DLR is the length less one
    h.write(0x048, 0x00_1234); // AR triggers
    assert!(h.octospi.busy(), "the chip select is down and waiting");
    for byte in [0xaau8, 0xbb, 0xcc] {
        h.regs
            .write(0x050, &[byte], MemAttrs::DEFAULT)
            .expect("a byte into DR");
    }
    assert!(!h.octospi.busy(), "the last byte closed the frame");
    assert_eq!(rec.frames(), [[0x02, 0x00, 0x12, 0x34, 0xaa, 0xbb, 0xcc]]);
    assert_eq!(h.read(0x020) & SR_TCF, SR_TCF);
    // §`FCR`: write one to clear.
    h.write(0x024, SR_TCF);
    assert_eq!(h.read(0x020) & SR_TCF, 0);
}

#[test]
fn an_indirect_read_clocks_dummy_bytes_and_pops_the_data_register() {
    let rec = Recorder::new(&[0xff, 0xff, 0xff, 0xff, 0xff, 0x11, 0x22, 0x33, 0x44]);
    let h = harness(Arc::clone(&rec) as Arc<dyn SpiSlave>);
    h.enable(FMODE_READ);
    h.write(0x100, CCR_SINGLE_24);
    h.write(0x108, 8); // TCR: eight dummy cycles, which is one byte
    h.write(0x110, 0x0b); // fast read
    h.write(0x040, 4 - 1);
    h.write(0x048, 0x10);
    // The header is opcode, three address bytes and one dummy: five bytes
    // before any data.
    assert_eq!(rec.frames()[0], [0x0b, 0x00, 0x00, 0x10, 0xff]);
    let got: Vec<u8> = (0..4).map(|_| h.byte(0x050)).collect();
    assert_eq!(got, [0x11, 0x22, 0x33, 0x44]);
    assert!(!h.octospi.busy());
    assert_eq!(h.read(0x020) & SR_TCF, SR_TCF);
}

#[test]
fn the_status_register_reports_what_is_left_of_a_read() {
    let rec = Recorder::new(&[]);
    let h = harness(Arc::clone(&rec) as Arc<dyn SpiSlave>);
    h.enable(FMODE_READ);
    h.write(0x100, CCR_SINGLE_24);
    h.write(0x110, 0x03);
    h.write(0x040, 8 - 1);
    h.write(0x048, 0);
    let sr = h.read(0x020);
    assert_eq!(sr & SR_BUSY, SR_BUSY);
    assert_eq!((sr >> SR_FLEVEL_SHIFT) & SR_FLEVEL_MASK, 8);
    h.byte(0x050);
    assert_eq!((h.read(0x020) >> SR_FLEVEL_SHIFT) & SR_FLEVEL_MASK, 7);
}

#[test]
fn an_abort_ends_the_frame_and_releases_the_chip_select() {
    let rec = Recorder::new(&[]);
    let h = harness(Arc::clone(&rec) as Arc<dyn SpiSlave>);
    h.enable(FMODE_READ);
    h.write(0x100, CCR_SINGLE_24);
    h.write(0x110, 0x03);
    h.write(0x040, 1000 - 1);
    h.write(0x048, 0);
    assert_eq!(h.bus.selected(), Some(ChipSelect(0)));
    h.write(0x000, h.read(0x000) | CR_ABORT);
    assert_eq!(h.bus.selected(), None, "the part is released");
    assert!(!h.octospi.busy());
    // `ABORT` is self-clearing.
    assert_eq!(h.read(0x000) & CR_ABORT, 0);
}

#[test]
fn automatic_status_polling_matches_against_the_mask() {
    // A part answering `02h` to a status read: bit 1 set, bit 0 clear.
    let rec = Recorder::new(&[0xff, 0x02, 0x02, 0x02, 0x02]);
    let h = harness(Arc::clone(&rec) as Arc<dyn SpiSlave>);
    h.enable(FMODE_POLL);
    h.write(0x080, 0x01); // PSMKR: watch bit 0, the flash's BUSY
    h.write(0x088, 0x00); // PSMAR: wait for it clear
    h.write(0x000, h.read(0x000) | CR_APMS);
    h.write(0x040, 1 - 1);
    h.write(0x100, CCR_OPCODE_ONLY | (1 << CCR_DMODE_SHIFT));
    h.write(0x110, 0x05); // read status register-1
    assert_eq!(rec.frames(), [[0x05, 0xff]], "opcode then one data byte");
    assert_eq!(h.read(0x020) & SR_SMF, SR_SMF, "0x02 & 0x01 == 0x00");
    assert_eq!(h.read(0x020) & SR_TCF, SR_TCF, "APMS stopped it");

    // The other way: a mask the answer fails.
    let rec = Recorder::new(&[0xff, 0x03]);
    let h = harness(Arc::clone(&rec) as Arc<dyn SpiSlave>);
    h.enable(FMODE_POLL);
    h.write(0x080, 0x01);
    h.write(0x088, 0x00);
    h.write(0x040, 0);
    h.write(0x100, CCR_OPCODE_ONLY | (1 << CCR_DMODE_SHIFT));
    h.write(0x110, 0x05);
    assert_eq!(h.read(0x020) & SR_SMF, 0, "0x03 & 0x01 is not 0x00");
}

// ---------------------------------------------------------------------------
// the memory-mapped window
// ---------------------------------------------------------------------------

fn window_read(h: &Harness, offset: u64, len: usize) -> MemResult<Vec<u8>> {
    let mut buf = alloc::vec![0u8; len];
    h.window.read(offset, &mut buf, MemAttrs::DEFAULT)?;
    Ok(buf)
}

#[test]
fn a_window_read_becomes_a_whole_flash_frame() {
    let rec = Recorder::new(&[0xff, 0xff, 0xff, 0xff, 0xff, 0xde, 0xad, 0xbe, 0xef]);
    let h = harness(Arc::clone(&rec) as Arc<dyn SpiSlave>);
    h.write(0x100, CCR_SINGLE_24);
    h.write(0x108, 8);
    h.write(0x110, 0x0b);
    h.enable(FMODE_MAPPED);

    let got = window_read(&h, 0x1234, 4).expect("the window answers");
    assert_eq!(got, [0xde, 0xad, 0xbe, 0xef]);
    // And this is the point of the peripheral: the load turned into a real
    // frame on the bus, built from the registers the driver programmed once.
    assert_eq!(
        rec.frames(),
        [[0x0b, 0x00, 0x12, 0x34, 0xff, 0xff, 0xff, 0xff, 0xff]]
    );
}

#[test]
fn the_window_does_not_decode_until_the_mode_selects_it() {
    let rec = Recorder::new(&[]);
    let h = harness(Arc::clone(&rec) as Arc<dyn SpiSlave>);
    h.write(0x100, CCR_SINGLE_24);
    h.write(0x110, 0x03);
    // Enabled, but in indirect mode: the aperture answers nothing, which is
    // an unassigned access rather than a fault the guest caused.
    h.enable(FMODE_READ);
    assert_eq!(window_read(&h, 0, 4).unwrap_err(), BusError::Unassigned);
    assert!(rec.frames().is_empty(), "and no frame was clocked");
}

#[test]
fn an_access_past_devsize_is_a_transfer_error() {
    let rec = Recorder::new(&[]);
    let h = harness(Arc::clone(&rec) as Arc<dyn SpiSlave>);
    h.write(0x100, CCR_SINGLE_24);
    h.write(0x110, 0x03);
    h.enable(FMODE_MAPPED);
    // The window decodes a megabyte; `DEVSIZE` says the part holds one too,
    // so the last byte is inside and one past it is not. ST's errata: an
    // access at or above `2^(DEVSIZE+1)` "should get an error response".
    assert!(window_read(&h, 1024 * 1024 - 1, 1).is_ok());
    let h2 = harness(Recorder::new(&[]) as Arc<dyn SpiSlave>);
    h2.write(0x008, 18 << DCR1_DEVSIZE_SHIFT); // half a megabyte
    h2.write(0x100, CCR_SINGLE_24);
    h2.write(0x110, 0x03);
    h2.write(0x000, CR_EN | (FMODE_MAPPED << CR_FMODE_SHIFT));
    assert_eq!(
        window_read(&h2, 512 * 1024, 1).unwrap_err(),
        BusError::BadAccess
    );
    assert_eq!(h2.read(0x020) & SR_TEF, SR_TEF);
}

#[test]
fn a_window_write_uses_the_write_register_set() {
    let rec = Recorder::new(&[]);
    let h = harness(Arc::clone(&rec) as Arc<dyn SpiSlave>);
    // The read set says `0Bh`; the write set says `02h`. A store must take
    // the second, which is the whole reason there are two.
    h.write(0x100, CCR_SINGLE_24);
    h.write(0x110, 0x0b);
    h.write(0x180, CCR_SINGLE_24); // WCCR
    h.write(0x190, 0x02); // WIR
    h.enable(FMODE_MAPPED);
    h.window
        .write(0x40, &[0x5a, 0xa5], MemAttrs::DEFAULT)
        .expect("the window takes a write");
    assert_eq!(rec.frames(), [[0x02, 0x00, 0x00, 0x40, 0x5a, 0xa5]]);
}

#[test]
fn a_debug_access_to_the_window_is_refused_rather_than_clocking_a_frame() {
    let rec = Recorder::new(&[]);
    let h = harness(Arc::clone(&rec) as Arc<dyn SpiSlave>);
    h.write(0x100, CCR_SINGLE_24);
    h.write(0x110, 0x03);
    h.enable(FMODE_MAPPED);
    let mut buf = [0u8; 4];
    // See the module docs: there is no side-effect-free route through a bus,
    // so the honest answer is to refuse rather than to move another device's
    // command state machine behind the guest's back.
    assert!(h.window.read(0, &mut buf, MemAttrs::DEBUG).is_err());
    assert!(h.window.write(0, &buf, MemAttrs::DEBUG).is_err());
    assert!(rec.frames().is_empty(), "nothing was clocked");
}

#[test]
fn the_data_register_reads_zero_in_memory_mapped_mode() {
    let rec = Recorder::new(&[0x11; 16]);
    let h = harness(Arc::clone(&rec) as Arc<dyn SpiSlave>);
    h.write(0x100, CCR_SINGLE_24);
    h.write(0x110, 0x03);
    h.enable(FMODE_MAPPED);
    // AN5050 §3.3.3: reading `DR` "has no meaning and returns 0".
    assert_eq!(h.read(0x050), 0);
    assert!(rec.frames().is_empty());
}

// ---------------------------------------------------------------------------
// interrupts
// ---------------------------------------------------------------------------

#[test]
fn transfer_complete_raises_the_interrupt_when_its_enable_is_set() {
    let rec = Recorder::new(&[]);
    let h = harness(Arc::clone(&rec) as Arc<dyn SpiSlave>);
    h.write(0x008, DEVSIZE_1M);
    h.write(0x000, CR_EN | CR_TCIE);
    assert!(!h.octospi.irq_asserted());
    h.write(0x100, CCR_OPCODE_ONLY);
    h.write(0x110, 0x06);
    assert!(h.octospi.irq_asserted(), "TCF with TCIE");
    h.write(0x024, SR_TCF);
    assert!(!h.octospi.irq_asserted());
}

// ---------------------------------------------------------------------------
// construction
// ---------------------------------------------------------------------------

#[test]
fn a_wired_link_is_refused_with_the_reason_written_out() {
    let e = Octospi::new(
        &Props::new()
            .with("link", Value::Str("wired".into()))
            .with("bus", Value::Str("q".into())),
    )
    .expect_err("a memory-mapped access cannot pace edges")
    .to_string();
    assert!(e.contains("inside a guest load"), "{e}");
}

#[test]
fn a_window_that_is_not_a_power_of_two_is_refused() {
    let e = Octospi::new(
        &Props::new()
            .with("link", Value::Str("transactional".into()))
            .with("bus", Value::Str("q".into()))
            .with("window", Value::Size(3 * 1024 * 1024)),
    )
    .expect_err("apertures are powers of two")
    .to_string();
    assert!(e.contains("power of two"), "{e}");
}

// ---------------------------------------------------------------------------
// snapshots
// ---------------------------------------------------------------------------

fn snapshot(octospi: &Octospi) -> Vec<u8> {
    let mut shape = MachineShape::new();
    shape.add_device("qspi", CLASS.name).expect("a fresh shape");
    let mut w = StateWriter::new(shape);
    {
        let mut chunk = w
            .chunk("qspi", CLASS.name, CLASS.version)
            .expect("one chunk");
        octospi.save(&mut chunk).expect("it saves");
    }
    w.to_vec().expect("a snapshot")
}

fn restore(octospi: &Octospi, bytes: &[u8]) {
    let reader = StateReader::new(bytes).expect("a snapshot");
    let chunk = reader
        .load("qspi", CLASS.name, CLASS.version, &Migrations::new())
        .expect("the chunk is there");
    octospi.load(&mut chunk.reader()).expect("it loads");
}

#[test]
fn a_snapshot_round_trips_to_an_identical_chunk() {
    let h = harness(Recorder::new(&[0x11; 32]) as Arc<dyn SpiSlave>);
    h.enable(FMODE_READ);
    h.write(0x100, CCR_SINGLE_24);
    h.write(0x110, 0x03);
    h.write(0x040, 4 - 1);
    h.write(0x048, 0x20);
    let first = snapshot(&h.octospi);

    let other = harness(Recorder::new(&[]) as Arc<dyn SpiSlave>);
    restore(&other.octospi, &first);
    assert_eq!(snapshot(&other.octospi), first, "identical bytes");
    assert!(other.octospi.busy(), "and the frame is still open");
    assert_eq!(other.read(0x020) & SR_BUSY, SR_BUSY);
}
