//! Paula's own tests: the manual's rules, one at a time.
//!
//! Every register access goes through a real `amiga.custom` bus with Paula
//! attached, so the ownership and direction checks of Appendix B are part of
//! what is exercised, and the offsets are the appendix's rather than this
//! file's private constants.

use super::*;
use crate::core::state::{MachineShape, Migrations, StateReader, StateWriter};
use crate::core::wire::{Drive, Wire, WireId};
use crate::dev::amiga::custom::Custom;
use crate::dev::amiga::regs;
use crate::host::chardev::CharPort;
use alloc::vec;

// -- offsets, from Appendix B -------------------------------------------------

const ADKCONR: u16 = 0x010;
const POTGOR: u16 = 0x016;
const SERDATR: u16 = 0x018;
const DSKBYTR: u16 = 0x01a;
const INTENAR: u16 = 0x01c;
const INTREQR: u16 = 0x01e;
const DMACONR: u16 = 0x002;
const DSKLEN: u16 = 0x024;
const SERDAT: u16 = 0x030;
const SERPER: u16 = 0x032;
const POTGO: u16 = 0x034;
const DSKSYNC: u16 = 0x07e;
const DMACON: u16 = 0x096;
const INTENA: u16 = 0x09a;
const INTREQ: u16 = 0x09c;
const ADKCON: u16 = 0x09e;
const AUD0LEN: u16 = 0x0a4;
const AUD0PER: u16 = 0x0a6;
const AUD0VOL: u16 = 0x0a8;
const AUD0DAT: u16 = 0x0aa;
const AUD1LEN: u16 = 0x0b4;
const AUD1PER: u16 = 0x0b6;
const AUD1VOL: u16 = 0x0b8;
const AUD3LEN: u16 = 0x0d4;
const AUD3PER: u16 = 0x0d6;
const AUD3VOL: u16 = 0x0d8;

/// `DMACON`: `SET/CLR`, `DMAEN`, `DSKEN`, `AUD0EN`.
const DMAF_SETCLR: u16 = 0x8000;
const DMAF_MASTER: u16 = 0x0200;
const DMAF_DISK: u16 = 0x0010;
const DMAF_AUD0: u16 = 0x0001;
const DMAF_AUD1: u16 = 0x0002;
const DMAF_AUD3: u16 = 0x0008;

/// `ADKCON`: `WORDSYNC`, `FAST`, `USE0V1`.
const ADKF_WORDSYNC: u16 = 0x0400;
const ADKF_FAST: u16 = 0x0100;
const ADKF_USE0V1: u16 = 0x0001;

// -- a chip on a bus ----------------------------------------------------------

struct Rig {
    paula: Paula,
    custom: Custom,
    host: Arc<CharPort>,
}

impl Rig {
    fn new() -> Rig {
        let host = Arc::new(CharPort::new());
        let paula = Paula::with_port(
            String::from("custom"),
            Arc::clone(&host) as Arc<dyn CharDevice>,
            String::from("serial"),
        );
        let custom = Custom::new(&Props::new()).expect("no properties");
        custom.bus().attach(paula.chip()).expect("Paula attaches");
        Rig {
            paula,
            custom,
            host,
        }
    }

    fn peek(&self, offset: u16) -> u16 {
        self.custom.bus().read(offset, Origin::cpu())
    }

    fn peek_debug(&self, offset: u16) -> u16 {
        self.custom
            .bus()
            .read(offset, Origin::cpu().for_debug(true))
    }

    fn poke(&self, offset: u16, value: u16) {
        assert!(
            self.custom.bus().write(offset, value, Origin::cpu()),
            "Paula should own the register at {offset:#05x}"
        );
    }

    fn run(&self, ticks: u64) {
        self.paula.advance_to(self.paula.ticks() + ticks);
    }

    /// Three wires on the level outputs, returned least significant first.
    fn ipl_wires(&self) -> [WireSource; 3] {
        core::array::from_fn(|bit| {
            let src = WireId::new(100 + bit as u64);
            let source = WireSource::new(Wire::builder().source(src).build_shared(), src);
            self.paula
                .connect(IPL_PINS[bit], source.clone())
                .expect("an ipl pin");
            source
        })
    }
}

/// The level three wires encode.
fn level_on(wires: &[WireSource; 3]) -> u8 {
    wires
        .iter()
        .enumerate()
        .map(|(bit, w)| u8::from(w.drive_state() == Drive::High) << bit)
        .sum()
}

// ---------------------------------------------------------------------------
// the table
// ---------------------------------------------------------------------------

#[test]
fn every_offset_the_chip_decodes_is_a_paula_register_in_appendix_b() {
    for (offset, name) in [
        (off::ADKCONR, "ADKCONR"),
        (off::POTGOR, "POTGOR"),
        (off::SERDATR, "SERDATR"),
        (off::DSKBYTR, "DSKBYTR"),
        (off::INTENAR, "INTENAR"),
        (off::INTREQR, "INTREQR"),
        (off::DSKLEN, "DSKLEN"),
        (off::DSKDAT, "DSKDAT"),
        (off::SERDAT, "SERDAT"),
        (off::SERPER, "SERPER"),
        (off::POTGO, "POTGO"),
        (off::DSKSYNC, "DSKSYNC"),
        (off::DMACON, "DMACON"),
        (off::INTENA, "INTENA"),
        (off::INTREQ, "INTREQ"),
        (off::ADKCON, "ADKCON"),
        (off::AUD0LEN, "AUD0LEN"),
        (off::AUD3DAT, "AUD3DAT"),
    ] {
        let reg = regs::lookup(offset).expect("declared");
        assert_eq!(reg.name, name);
        assert!(reg.chip.contains(ChipId::PAULA), "{name} is Paula's");
    }
    for ch in 0..4u16 {
        for (step, name) in [(4, "LEN"), (6, "PER"), (8, "VOL"), (0xa, "DAT")] {
            let reg = regs::lookup(0x0a0 + ch * 0x10 + step).expect("an audio register");
            assert_eq!(reg.name, format!("AUD{ch}{name}"));
            assert!(reg.chip.contains(ChipId::PAULA));
            assert_eq!(audio_channel(reg.offset), usize::from(ch));
        }
    }
}

// ---------------------------------------------------------------------------
// interrupts
// ---------------------------------------------------------------------------

#[test]
fn intena_and_intreq_take_the_set_clr_bit_and_read_back_without_it() {
    let rig = Rig::new();
    rig.poke(INTENA, 0x8000 | 0x4000 | int::VERTB | int::PORTS);
    assert_eq!(rig.peek(INTENAR), 0x4000 | int::VERTB | int::PORTS);
    rig.poke(INTENA, int::VERTB);
    assert_eq!(rig.peek(INTENAR), 0x4000 | int::PORTS, "cleared one bit");
    rig.poke(INTENA, 0x0000);
    assert_eq!(
        rig.peek(INTENAR),
        0x4000 | int::PORTS,
        "zeros select nothing"
    );

    rig.poke(INTREQ, 0x8000 | int::SOFT | int::COPER);
    assert_eq!(rig.peek(INTREQR), int::SOFT | int::COPER);
    rig.poke(INTREQ, 0x7fff);
    assert_eq!(rig.peek(INTREQR), 0);
}

#[test]
fn intreq_stores_no_master_bit_because_it_creates_no_request() {
    // "Warning: This bit is used for enable/disable only. It creates no
    // interrupt request."
    let rig = Rig::new();
    rig.poke(INTREQ, 0xc000);
    assert_eq!(rig.peek(INTREQR), 0);
}

#[test]
fn each_request_bit_raises_the_level_appendix_a_gives_it() {
    let expected: [(u16, u8); 14] = [
        (int::TBE, 1),
        (int::DSKBLK, 1),
        (int::SOFT, 1),
        (int::PORTS, 2),
        (int::COPER, 3),
        (int::VERTB, 3),
        (int::BLIT, 3),
        (int::AUD0, 4),
        (int::AUD1, 4),
        (int::AUD2, 4),
        (int::AUD3, 4),
        (int::RBF, 5),
        (int::DSKSYN, 5),
        (int::EXTER, 6),
    ];
    let rig = Rig::new();
    let wires = rig.ipl_wires();
    rig.poke(INTENA, 0xffff);
    for (bit, level) in expected {
        rig.poke(INTREQ, 0x7fff);
        rig.poke(INTREQ, 0x8000 | bit);
        assert_eq!(rig.paula.ipl(), level, "bit {bit:#06x}");
        assert_eq!(level_on(&wires), level, "and on the wires, bit {bit:#06x}");
    }
    // The highest pending level wins.
    rig.poke(INTREQ, 0x7fff);
    rig.poke(INTREQ, 0x8000 | int::TBE | int::VERTB | int::RBF);
    assert_eq!(rig.paula.ipl(), 5);
    rig.poke(INTREQ, int::RBF);
    assert_eq!(rig.paula.ipl(), 3);
    rig.poke(INTREQ, 0x7fff);
    assert_eq!(level_on(&wires), 0);
}

#[test]
fn a_request_reaches_the_processor_only_through_its_enable_and_the_master_bit() {
    let rig = Rig::new();
    let wires = rig.ipl_wires();
    rig.poke(INTREQ, 0x8000 | int::VERTB);
    assert_eq!(level_on(&wires), 0, "nothing enabled");
    rig.poke(INTENA, 0x8000 | int::VERTB);
    assert_eq!(level_on(&wires), 0, "enabled, but the master bit is off");
    rig.poke(INTENA, 0x8000 | int::INTEN);
    assert_eq!(level_on(&wires), 3);
    rig.poke(INTENA, int::INTEN);
    assert_eq!(level_on(&wires), 0, "the master bit masks everything");
    assert_eq!(
        rig.peek(INTREQR),
        int::VERTB,
        "and the request is still pending"
    );
}

#[test]
fn the_external_lines_hold_their_bits_up_while_asserted() {
    let rig = Rig::new();
    rig.poke(INTENA, 0x8000 | int::INTEN | int::PORTS | int::EXTER);
    let src = WireId::new(1);
    let int2 = rig.paula.sink(INT2_PIN, &[src]).expect("int2");
    let int6 = rig.paula.sink(INT6_PIN, &[src]).expect("int6");

    int2.sink.set_level(src, int2.line, Level::High);
    assert_eq!(rig.peek(INTREQR), int::PORTS);
    assert_eq!(rig.paula.ipl(), 2);

    // Software clears the bit while the CIA is still requesting: it comes
    // straight back, because the line is still low.
    rig.poke(INTREQ, int::PORTS);
    assert_eq!(rig.peek(INTREQR), int::PORTS);

    int6.sink.set_level(src, int6.line, Level::High);
    assert_eq!(rig.paula.ipl(), 6);

    // The CIA's ICR is read and its line lets go; now the clear sticks.
    int2.sink.set_level(src, int2.line, Level::Low);
    assert_eq!(rig.peek(INTREQR), int::PORTS | int::EXTER, "latched");
    rig.poke(INTREQ, int::PORTS);
    assert_eq!(rig.peek(INTREQR), int::EXTER);
    int6.sink.set_level(src, int6.line, Level::Low);
    rig.poke(INTREQ, int::EXTER);
    assert_eq!(rig.paula.ipl(), 0);
}

#[test]
fn agnus_raises_its_own_sources_through_the_port() {
    let rig = Rig::new();
    rig.poke(INTENA, 0x8000 | int::INTEN | int::VERTB);
    rig.paula.port().request(0, int::VERTB);
    assert_eq!(rig.paula.ipl(), 3);
    // And the copper's COPER comes through the bus as an ordinary write.
    assert!(
        rig.custom
            .bus()
            .write(INTREQ, 0x8000 | int::COPER, Origin::copper(false))
    );
    assert_eq!(rig.peek(INTREQR), int::VERTB | int::COPER);
}

/// A sink standing in for a processor's interrupt pins: it takes a
/// `DEVICE`-ranked lock, as any device's input does.
#[cfg(feature = "dev-mos8520")]
#[derive(Debug)]
struct LockingSink {
    seen: Mutex<u32>,
}

#[cfg(feature = "dev-mos8520")]
impl WireSink for LockingSink {
    fn set_level(&self, _src: WireId, _line: u32, _level: Level) {
        *self.seen.lock() += 1;
    }
}

#[cfg(feature = "dev-mos8520")]
#[test]
fn a_cia_interrupt_reaches_the_level_wires_without_nesting_device_locks() {
    use crate::core::space::{AddressSpace, MemAttrs};
    use crate::core::value::Width;
    use crate::dev::mos::Cia;

    // The rank checker is live in this build (`cfg(test)`), which an
    // integration test built without every feature is not: a chip that drove
    // a wire while holding its own state lock would panic here.
    let rig = Rig::new();
    let cpu = Arc::new(LockingSink {
        seen: Mutex::with_rank(LockRank::DEVICE, 0),
    });
    for (bit, name) in IPL_PINS.iter().enumerate() {
        let src = WireId::new(10 + bit as u64);
        let wire = Wire::builder()
            .source(src)
            .sink(Arc::clone(&cpu) as Arc<dyn WireSink>, bit as u32)
            .build_shared();
        rig.paula
            .connect(name, WireSource::new(wire, src))
            .expect("an ipl pin");
    }

    let cia = Cia::bare();
    let src = WireId::new(1);
    let int2 = rig.paula.sink(INT2_PIN, &[src]).expect("int2");
    let wire = Wire::builder()
        .source(src)
        .sink(int2.sink, int2.line)
        .build_shared();
    cia.connect_pin("irq", WireSource::new(wire, src))
        .expect("irq");

    let space = AddressSpace::new("cia", 16);
    space
        .topology()
        .map(cia.region("").expect("registers"), 0)
        .expect("maps");
    let poke = |reg: u64, v: u8| {
        space
            .write(reg, Width::U8, u64::from(v), MemAttrs::DEFAULT)
            .expect("a CIA register");
    };
    rig.poke(INTENA, 0x8000 | int::INTEN | int::PORTS);
    poke(0x4, 3); // TA LO
    poke(0x5, 0); // TA HI
    poke(0xd, 0x81); // ICR: SET, TA
    poke(0xe, 0x01); // CRA: START
    cia.advance_to(10);

    assert_eq!(rig.paula.ipl(), 2, "CIA-A's timer is a level 2 interrupt");
    assert!(*cpu.seen.lock() > 0, "and it reached the processor's pins");
}

// ---------------------------------------------------------------------------
// serial
// ---------------------------------------------------------------------------

#[test]
fn a_serdat_word_goes_out_at_the_serper_rate_and_the_host_gets_the_byte() {
    let rig = Rig::new();
    rig.poke(SERPER, 9); // ten colour clocks a bit
    rig.poke(SERDAT, 0x0155); // 'U' and one stop bit
    // Moved into the shifter straight away, so the buffer is empty again and
    // TBE is requested; the shifter is busy.
    assert_eq!(rig.peek(INTREQR) & int::TBE, int::TBE);
    let status = rig.peek(SERDATR);
    assert_eq!(status & 0x2000, 0x2000, "TBE: SERDAT can take another");
    assert_eq!(status & 0x1000, 0, "TSRE: still shifting");

    // A start bit and nine more up to the stop bit: a hundred ticks.
    rig.run(99);
    assert!(rig.host.drain().is_empty(), "not before the stop bit ends");
    rig.run(1);
    assert_eq!(rig.host.drain(), vec![0x55]);
    assert_eq!(rig.peek(SERDATR) & 0x1000, 0x1000, "TSRE");
}

#[test]
fn a_word_written_while_shifting_waits_in_serdat() {
    let rig = Rig::new();
    rig.poke(SERPER, 0);
    rig.poke(SERDAT, 0x0141);
    rig.poke(INTREQ, int::TBE);
    rig.poke(SERDAT, 0x0142);
    assert_eq!(rig.peek(SERDATR) & 0x2000, 0, "the buffer holds a word");
    assert_eq!(rig.peek(INTREQR) & int::TBE, 0);
    rig.run(10);
    assert_eq!(rig.peek(INTREQR) & int::TBE, int::TBE, "taken at the end");
    rig.run(10);
    assert_eq!(rig.host.drain(), b"AB".to_vec());
}

#[test]
fn a_zero_serdat_starts_nothing() {
    let rig = Rig::new();
    rig.poke(SERDAT, 0);
    assert_eq!(rig.peek(INTREQR) & int::TBE, 0);
    assert_eq!(rig.peek(SERDATR) & 0x1000, 0x1000);
}

#[test]
fn a_host_byte_arrives_as_a_frame_and_sets_rbf() {
    let rig = Rig::new();
    rig.poke(SERPER, 4); // five ticks a bit
    rig.poke(INTENA, 0x8000 | int::INTEN | int::RBF);
    rig.host.feed(b"hi");
    rig.paula.pump();
    rig.run(10);
    // Mid-frame the RXD pin shows the bits: the start bit is gone, and bit 1
    // of 'h' ($68) is zero.
    assert_eq!(rig.peek(SERDATR) & 0x0800, 0);
    rig.run(40);
    let status = rig.peek(SERDATR);
    assert_eq!(status & 0x4000, 0x4000, "RBF mirror");
    assert_eq!(status & 0x03ff, 0x0368, "the byte and its stop bits");
    assert_eq!(rig.paula.ipl(), 5);

    // The next byte waits for the guest.
    rig.paula.pump();
    rig.run(100);
    assert_eq!(rig.peek(SERDATR) & 0x00ff, 0x68);
    rig.poke(INTREQ, int::RBF);
    rig.paula.pump();
    rig.run(50);
    assert_eq!(rig.peek(SERDATR) & 0x00ff, 0x69);
    assert_eq!(rig.peek(SERDATR) & 0x8000, 0, "no overrun");
}

#[test]
fn a_frame_that_lands_on_a_full_buffer_is_an_overrun_until_rbf_is_cleared() {
    let rig = Rig::new();
    rig.paula.shared.with_state(|st| {
        st.intreq |= int::RBF;
        st.serial_receive(0x41);
    });
    rig.run(10);
    assert_eq!(rig.peek(SERDATR) & 0x8000, 0x8000, "OVRUN");
    rig.poke(INTREQ, int::RBF);
    assert_eq!(rig.peek(SERDATR) & 0x8000, 0, "cleared with RBF");
}

// ---------------------------------------------------------------------------
// disk
// ---------------------------------------------------------------------------

/// A drive with a loop of cells under the head and a record of what it was
/// asked to write. Its lock is ranked above `DEVICE`, as a real drive's must
/// be, so the rank checker proves that is enough.
#[derive(Debug)]
struct FakeDrive {
    cells: Vec<u8>,
    reading: bool,
    written: Mutex<Vec<u8>>,
}

impl FakeDrive {
    fn with_words(words: &[u16]) -> Arc<FakeDrive> {
        let mut cells = Vec::new();
        for w in words {
            for bit in (0..16).rev() {
                cells.push((w >> bit & 1) as u8);
            }
        }
        Arc::new(FakeDrive {
            cells,
            reading: true,
            written: Mutex::with_rank(LockRank::new(0x5400), Vec::new()),
        })
    }
}

impl DiskDrive for FakeDrive {
    fn reading(&self) -> bool {
        self.reading
    }

    fn read_cells(&self, start: u64, cell: u64, out: &mut [u8]) {
        let len = self.cells.len() as u64;
        for (i, o) in out.iter_mut().enumerate() {
            *o |= self.cells[((start / cell + i as u64) % len) as usize];
        }
    }

    fn write_cells(&self, _start: u64, _cell: u64, cells: &[u8]) {
        self.written.lock().extend_from_slice(cells);
    }
}

/// Two gap words, the sync mark, and three data words: 96 cells.
fn track() -> Arc<FakeDrive> {
    FakeDrive::with_words(&[0xaaaa, 0xaaaa, 0x4489, 0x2aaa, 0xaaa5, 0x5552])
}

#[test]
fn dsklen_starts_dma_only_on_the_second_write_with_dmaen() {
    let rig = Rig::new();
    rig.poke(DMACON, DMAF_SETCLR | DMAF_MASTER | DMAF_DISK);
    rig.poke(DSKLEN, 0x4000); // step 2: off
    rig.poke(DSKLEN, 0x8002); // step 3
    assert_eq!(rig.peek(DSKBYTR) & 0x4000, 0, "one write is not enough");
    rig.poke(DSKLEN, 0x8002); // step 4
    assert_eq!(rig.peek(DSKBYTR) & 0x4000, 0x4000, "DMAON");
    rig.poke(DSKLEN, 0x4000); // step 5
    let status = rig.peek(DSKBYTR);
    assert_eq!(status & 0x4000, 0);
    assert_eq!(status & 0x2000, 0x2000, "DISKWRITE mirrors DSKLEN's bit 14");

    // DMAON also needs DMACON.
    rig.poke(DSKLEN, 0x8002);
    rig.poke(DSKLEN, 0x8002);
    rig.poke(DMACON, DMAF_DISK);
    assert_eq!(rig.peek(DSKBYTR) & 0x4000, 0);
}

#[test]
fn a_wordsync_read_starts_after_the_sync_mark_and_ends_with_dskblk() {
    let rig = Rig::new();
    let drive = track();
    rig.paula.port().attach_drive(drive);
    let wires = rig.ipl_wires();
    rig.poke(ADKCON, 0x8000 | ADKF_FAST | ADKF_WORDSYNC);
    rig.poke(DSKSYNC, 0x4489);
    rig.poke(INTENA, 0x8000 | int::INTEN | int::DSKSYN | int::DSKBLK);
    rig.poke(DMACON, DMAF_SETCLR | DMAF_MASTER | DMAF_DISK);
    rig.poke(DSKLEN, 0x8002);
    rig.poke(DSKLEN, 0x8002);

    let port = rig.paula.port();
    let mut got = Vec::new();
    // One cell every seven ticks; poll like a DMA slot would, a few times a
    // word, for two turns of the loop.
    let mut at = 0;
    while at < 96 * 7 * 2 && got.len() < 2 {
        at += 20;
        while let Some(w) = port.disk_read_word(at) {
            got.push(w);
        }
    }
    assert_eq!(got, vec![0x2aaa, 0xaaa5], "the words after the sync mark");
    let req = rig.peek(INTREQR);
    assert_eq!(req & int::DSKSYN, int::DSKSYN);
    assert_eq!(req & int::DSKBLK, int::DSKBLK);
    assert_eq!(rig.peek(DSKBYTR) & 0x4000, 0, "the transfer is over");
    assert_eq!(level_on(&wires), 5);
    assert_eq!(port.disk_read_word(at + 1000), None);
}

#[test]
fn dskbytr_assembles_bytes_aligned_to_the_sync_mark_and_a_read_clears_dskbyt() {
    let rig = Rig::new();
    rig.paula.port().attach_drive(track());
    rig.poke(ADKCON, 0x8000 | ADKF_FAST);
    rig.poke(DSKSYNC, 0x4489);
    // The sync mark ends on cell 48; eight cells later the first byte after
    // it, $2A, is complete.
    rig.paula.advance_to(56 * 7);
    let status = rig.peek_debug(DSKBYTR);
    assert_eq!(status & 0x8000, 0x8000, "DSKBYT");
    assert_eq!(status & 0x00ff, 0x2a);
    assert_eq!(
        rig.peek_debug(DSKBYTR) & 0x8000,
        0x8000,
        "a debugger's read cleared nothing"
    );
    let _ = rig.peek(DSKBYTR);
    assert_eq!(rig.peek(DSKBYTR) & 0x8000, 0, "a guest's read did");
}

#[test]
fn wordequal_is_true_only_while_the_stream_matches() {
    let rig = Rig::new();
    rig.paula.port().attach_drive(track());
    rig.poke(ADKCON, 0x8000 | ADKF_FAST);
    rig.poke(DSKSYNC, 0x4489);
    rig.paula.advance_to(48 * 7 - 1);
    assert_eq!(rig.peek(DSKBYTR) & 0x1000, 0x1000, "during the last cell");
    rig.paula.advance_to(48 * 7 + 1);
    assert_eq!(rig.peek(DSKBYTR) & 0x1000, 0, "and not after it");
}

#[test]
fn a_write_dma_shifts_agnus_words_out_and_sets_dskblk_after_the_last() {
    let rig = Rig::new();
    let drive = FakeDrive::with_words(&[0]);
    rig.paula
        .port()
        .attach_drive(Arc::clone(&drive) as Arc<dyn DiskDrive>);
    rig.poke(ADKCON, 0x8000 | ADKF_FAST);
    rig.poke(DMACON, DMAF_SETCLR | DMAF_MASTER | DMAF_DISK);
    rig.poke(DSKLEN, 0xc002);
    rig.poke(DSKLEN, 0xc002);
    let port = rig.paula.port();
    let mut words = vec![0xf00f_u16, 0x8001].into_iter();
    let mut at = 0;
    while at < 2000 {
        at += 10;
        if port.disk_write_wanted(at)
            && let Some(w) = words.next()
        {
            port.disk_write_word(at, w);
        }
    }
    let written = drive.written.lock().clone();
    let mut expect = Vec::new();
    for w in [0xf00f_u16, 0x8001] {
        for bit in (0..16).rev() {
            expect.push((w >> bit & 1) as u8);
        }
    }
    assert_eq!(written, expect);
    assert_eq!(rig.peek(INTREQR) & int::DSKBLK, int::DSKBLK);
    assert!(!port.disk_write_wanted(at + 10));
}

#[test]
fn with_no_drive_the_line_is_idle_and_costs_nothing_to_count() {
    let rig = Rig::new();
    rig.poke(DSKSYNC, 0x4489);
    // A minute of colour clocks. A per-cell loop would take visibly long here
    // in a debug build; the idle path counts.
    rig.paula.advance_to(3_546_895 * 60);
    let status = rig.peek(DSKBYTR);
    assert_eq!(status & 0x80ff, 0x8000, "zero bytes, still clocked");
    assert_eq!(rig.peek(INTREQR) & int::DSKSYN, 0);
}

// ---------------------------------------------------------------------------
// audio
// ---------------------------------------------------------------------------

#[test]
fn starting_a_channel_copies_the_length_asks_for_a_restart_and_interrupts() {
    let rig = Rig::new();
    let port = rig.paula.port();
    rig.poke(AUD0LEN, 2);
    rig.poke(AUD0PER, 124);
    rig.poke(AUD0VOL, 64);
    rig.poke(DMACON, DMAF_SETCLR | DMAF_MASTER | DMAF_AUD0);
    assert_eq!(rig.peek(INTREQR) & int::AUD0, int::AUD0, "on starting");
    rig.poke(INTREQ, int::AUD0);

    let req = port.audio_request(0, 0);
    assert_eq!(
        req,
        AudioRequest {
            restart: true,
            fetch: true
        }
    );
    port.audio_word(0, 0, 0x7f80);
    assert_eq!(port.audio_request(0, 1), AudioRequest::default());

    // The first word boundary loads it: high byte, then low byte.
    rig.paula.advance_to(124);
    assert_eq!(rig.paula.audio_output(0), (0x7f, 64));
    assert_eq!(
        port.audio_request(0, 124),
        AudioRequest {
            restart: false,
            fetch: true
        }
    );
    port.audio_word(0, 124, 0x0102);
    rig.paula.advance_to(248);
    assert_eq!(rig.paula.audio_output(0), (-128, 64));
    assert_eq!(rig.peek(INTREQR) & int::AUD0, 0, "one word of two");

    // The second word is the last of the block: the back-up length is
    // reloaded, the pointer reset, and the interrupt comes again.
    rig.paula.advance_to(372);
    assert_eq!(rig.paula.audio_output(0), (0x01, 64));
    assert_eq!(rig.peek(INTREQR) & int::AUD0, int::AUD0);
    assert_eq!(
        port.audio_request(0, 372),
        AudioRequest {
            restart: true,
            fetch: true
        }
    );

    rig.poke(DMACON, DMAF_AUD0);
    assert_eq!(rig.paula.audio_output(0), (0, 0), "stopped");
}

#[test]
fn the_block_interrupt_is_on_the_calendar() {
    let rig = Rig::new();
    rig.poke(AUD0LEN, 3);
    rig.poke(AUD0PER, 200);
    rig.poke(DMACON, DMAF_SETCLR | DMAF_MASTER | DMAF_AUD0);
    // Boundaries at 200, 600, 1000 load words one to three; the third reloads.
    assert_eq!(rig.paula.next_event_tick(), Some(1000));
}

#[test]
fn a_processor_write_to_audxdat_plays_one_word_in_manual_mode() {
    let rig = Rig::new();
    rig.poke(AUD0PER, 100);
    rig.poke(AUD0VOL, 0x20);
    rig.poke(AUD0DAT, 0x4000);
    assert_eq!(
        rig.peek(INTREQR) & int::AUD0,
        int::AUD0,
        "the latch is ready for the next word"
    );
    assert_eq!(rig.paula.audio_output(0), (0x40, 0x20));
    rig.run(100);
    assert_eq!(rig.paula.audio_output(0), (0x00, 0x20));
    rig.run(100);
    assert_eq!(rig.paula.audio_output(0), (0, 0), "no next word: silence");
}

#[test]
fn an_attached_channel_writes_its_words_into_the_next_channels_volume() {
    let rig = Rig::new();
    rig.poke(ADKCON, 0x8000 | ADKF_USE0V1);
    rig.poke(AUD0PER, 50);
    rig.poke(AUD1VOL, 64);
    rig.poke(AUD0DAT, 0x0010);
    assert_eq!(rig.paula.shared.state.lock().aud[1].vol, 0x0010);
    assert_eq!(rig.paula.audio_output(0), (0, 0), "a modulator is silent");
}

// ---------------------------------------------------------------------------
// audio: the host stream
// ---------------------------------------------------------------------------

/// One channel at full volume: `sample × volume`, 127 × 64.
const FULL: i16 = 127 * 64;

/// Start channel `ch` on a square wave at `per` colour clocks a sample and
/// volume `vol`.
///
/// The word `$7f81` is +127 then −127 — each word is two samples, the high
/// byte first (Chapter 5) — fed once through the seam Agnus fetches
/// into. The block is one word long and nothing feeds another, so the output
/// buffer keeps replaying the word it has: a square wave whose period is four
/// times `per`, which is what the chapter's period formula says.
fn square(rig: &Rig, ch: u16, per: u16, vol: u16) {
    let base = 0x0a0 + ch * 0x10;
    rig.poke(base + 4, 1);
    rig.poke(base + 6, per);
    rig.poke(base + 8, vol);
    rig.poke(DMACON, DMAF_SETCLR | DMAF_MASTER | (1 << ch));
    rig.paula.port().audio_word(usize::from(ch), 0, 0x7f81);
}

#[test]
fn a_recorded_channel_produces_the_samples_the_period_formula_predicts() {
    let rig = Rig::new();
    rig.paula.set_recording(true);
    // 256 colour clocks a sample is eight frames a sample at the divisor's 32,
    // and 3 546 895 / 256 = 13 855 samples a second on a PAL machine.
    square(&rig, 0, 256, 64);
    rig.paula.advance_to(256 * 17);
    let frames = rig.paula.take_audio();

    assert_eq!(frames.len(), 256 * 17 / 32, "one frame every 32 colour clocks");
    for (i, frame) in frames.iter().enumerate() {
        // Nothing until the first word boundary at `per`: the channel is
        // playing but its output buffer has not been loaded yet.
        let want = if i < 8 {
            0
        } else if (i - 8) / 8 % 2 == 0 {
            FULL
        } else {
            -FULL
        };
        assert_eq!(*frame, (want, 0), "frame {i}");
    }
}

#[test]
fn a_boundary_inside_a_frame_is_weighted_rather_than_rounded() {
    // Chapter 5's minimum period, "124 color clocks", is deliberately *not* a
    // multiple of the 32 a frame covers. The
    // sample that starts at 124 begins seven eighths of the way through the
    // frame that runs 96..128, so that frame is 28 colour clocks of silence
    // and 4 of +8128: 8128 × 4 / 32 = 1016, exactly, with nothing rounded to
    // a frame edge and nothing left to the size of the step the scheduler
    // happened to take.
    let rig = Rig::new();
    rig.paula.set_recording(true);
    square(&rig, 0, 124, 64);
    rig.paula.advance_to(128);
    let frames = rig.paula.take_audio();
    assert_eq!(frames.len(), 4);
    assert_eq!(frames[0], (0, 0));
    assert_eq!(frames[2], (0, 0));
    assert_eq!(frames[3], (FULL * 4 / 32, 0));
}

#[test]
fn volume_scales_the_sample_and_bit_six_is_full_level() {
    // Chapter 5: six bits, 0 to 64, and bit 6 — `$40` — is the 65th level
    // however the rest of the register reads.
    for (written, level) in [(64u16, 64i16), (32, 32), (1, 1), (0x40, 64), (0x7f, 64), (0, 0)] {
        let rig = Rig::new();
        rig.paula.set_recording(true);
        square(&rig, 0, 256, written);
        rig.paula.advance_to(512);
        let frames = rig.paula.take_audio();
        assert_eq!(frames.len(), 16);
        assert_eq!(frames[15], (127 * level, 0), "AUD0VOL {written:#x}");
    }
}

#[test]
fn zero_and_three_are_the_left_output_and_one_and_two_the_right() {
    // The machine's wiring (Chapter 5), and the one thing a mono mixdown would
    // throw away.
    for ch in 0..4u16 {
        let rig = Rig::new();
        rig.paula.set_recording(true);
        square(&rig, ch, 256, 64);
        rig.paula.advance_to(512);
        let frame = *rig.paula.take_audio().last().expect("frames");
        let want = if ch == 0 || ch == 3 {
            (FULL, 0)
        } else {
            (0, FULL)
        };
        assert_eq!(frame, want, "channel {ch}");
    }

    // And a side is the *sum* of its pair.
    let rig = Rig::new();
    rig.paula.set_recording(true);
    square(&rig, 0, 256, 64);
    square(&rig, 3, 256, 64);
    rig.paula.advance_to(512);
    assert_eq!(
        *rig.paula.take_audio().last().expect("frames"),
        (2 * FULL, 0)
    );
}

#[test]
fn a_channel_that_stops_leaves_silence_rather_than_a_held_sample() {
    let rig = Rig::new();
    rig.paula.set_recording(true);
    square(&rig, 0, 256, 64);
    rig.paula.advance_to(512);
    assert_eq!(rig.paula.take_audio().len(), 16);

    rig.poke(DMACON, DMAF_AUD0);
    rig.paula.advance_to(1024);
    let frames = rig.paula.take_audio();
    assert_eq!(frames.len(), 16);
    assert!(
        frames.iter().all(|f| *f == (0, 0)),
        "a stopped channel is silent, not held: {frames:?}"
    );
}

#[test]
fn a_modulating_channel_is_heard_on_neither_side() {
    // Its words go to the next channel's volume (Table 5-4) rather than to a
    // speaker — including the mixer this file feeds.
    let rig = Rig::new();
    rig.paula.set_recording(true);
    rig.poke(ADKCON, 0x8000 | ADKF_USE0V1);
    square(&rig, 0, 256, 64);
    rig.paula.advance_to(2048);
    let frames = rig.paula.take_audio();
    assert_eq!(frames.len(), 64);
    assert!(frames.iter().all(|f| *f == (0, 0)), "{frames:?}");
    // And it did modulate: the word reached channel 1's volume register.
    assert_eq!(rig.paula.shared.state.lock().aud[1].vol, 0x7f81);
}

#[test]
fn the_frames_do_not_depend_on_how_the_run_was_cut_up() {
    // A period of 200 is not a multiple of a frame, so every other frame
    // straddles a boundary and the weighting above is in play throughout.
    fn whole() -> Vec<(i16, i16)> {
        let rig = Rig::new();
        rig.paula.set_recording(true);
        square(&rig, 0, 200, 48);
        rig.paula.advance_to(4000);
        rig.paula.take_audio()
    }

    let once = whole();
    assert_eq!(once.len(), 125);
    assert_eq!(once, whole(), "the same run twice");

    let sliced = {
        let rig = Rig::new();
        rig.paula.set_recording(true);
        square(&rig, 0, 200, 48);
        let mut frames = Vec::new();
        // Seven colour clocks at a time: coprime with the period, with the
        // divisor and with each other, so no two frames are filled the same
        // way.
        for at in (0..=4000).step_by(7) {
            rig.paula.advance_to(at);
            frames.extend(rig.paula.take_audio());
        }
        rig.paula.advance_to(4000);
        frames.extend(rig.paula.take_audio());
        frames
    };
    assert_eq!(once, sliced, "the integration is not a function of the step");
}

#[test]
fn listening_is_not_machine_state() {
    let heard = Rig::new();
    heard.paula.set_recording(true);
    let ignored = Rig::new();
    for rig in [&heard, &ignored] {
        square(rig, 0, 200, 64);
        rig.paula.advance_to(4000);
    }
    assert_eq!(
        snapshot(&heard.paula),
        snapshot(&ignored.paula),
        "the ring reached the snapshot"
    );
    assert_eq!(heard.paula.take_audio().len(), 125);
    assert!(
        ignored.paula.take_audio().is_empty(),
        "nobody asked for frames"
    );

    // And whether anybody is listening is not something a reset undoes: it is
    // the host's, not the guest's.
    Device::reset(&heard.paula, ResetKind::Cold);
    assert!(heard.paula.recording());
    assert!(!ignored.paula.recording());
}

// ---------------------------------------------------------------------------
// the rest of the register face
// ---------------------------------------------------------------------------

#[test]
fn adkcon_takes_set_clr_and_potgo_answers_through_potgor() {
    let rig = Rig::new();
    rig.poke(ADKCON, 0x8000 | ADKF_FAST | ADKF_WORDSYNC);
    rig.poke(ADKCON, ADKF_WORDSYNC);
    assert_eq!(rig.peek(ADKCONR), ADKF_FAST);

    // Every pin an input: each reads high.
    assert_eq!(rig.peek(POTGOR) & 0x5500, 0x5500);
    // Pin 10 (DATLY) an output driven low: the right mouse button line.
    rig.poke(POTGO, 0x0800);
    assert_eq!(rig.peek(POTGOR) & 0x0400, 0);
    assert_eq!(rig.peek(POTGOR) & 0x5100, 0x5100);
}

#[test]
fn a_button_to_ground_on_a_pot_pin_reads_zero_either_way_round() {
    let rig = Rig::new();
    let src = WireId::new(9);
    let pin = rig.paula.sink("potly", &[src]).expect("pin 9 of port 0");
    let press = |down: bool| {
        pin.sink.set_level(src, pin.line, Level::from_bool(!down));
    };
    // "set both OUT… and DAT… to 1. Reading POTINP will produce a 0 if the
    // button is pressed, a 1 if it is not."
    rig.poke(POTGO, 0x0c00);
    assert_eq!(rig.peek(POTGOR) & 0x0400, 0x0400);
    press(true);
    assert_eq!(rig.peek(POTGOR) & 0x0400, 0, "pressed");
    // As an input too, and the other three pins do not notice.
    rig.poke(POTGO, 0x0000);
    assert_eq!(rig.peek(POTGOR) & 0x5500, 0x5100);
    press(false);
    assert_eq!(rig.peek(POTGOR) & 0x5500, 0x5500);
    assert!(rig.paula.sink("potrx", &[src]).is_some());
    assert!(rig.paula.sink("pot9", &[src]).is_none());
}

#[test]
fn paula_drives_no_bit_of_dmaconr() {
    let rig = Rig::new();
    rig.poke(DMACON, DMAF_SETCLR | DMAF_MASTER | DMAF_DISK);
    assert_eq!(rig.peek(DMACONR), 0);
    assert_eq!(rig.custom.bus().unclaimed(), 0, "but it did answer");
}

#[test]
fn the_next_event_is_always_in_the_future() {
    let rig = Rig::new();
    assert_eq!(
        rig.paula.next_event_tick(),
        None,
        "an idle chip asks for nothing"
    );
    rig.poke(SERPER, 0);
    rig.poke(SERDAT, 0x0101);
    assert_eq!(rig.paula.next_event_tick(), Some(10));
    rig.paula.advance_to(10);
    assert_eq!(rig.paula.next_event_tick(), None);
}

#[test]
fn a_reset_clears_the_chip_and_keeps_the_lines_other_chips_drive() {
    let rig = Rig::new();
    rig.poke(INTENA, 0xc000 | int::PORTS);
    let src = WireId::new(1);
    let int2 = rig.paula.sink(INT2_PIN, &[src]).expect("int2");
    int2.sink.set_level(src, int2.line, Level::High);
    rig.poke(SERPER, 7);
    Device::reset(&rig.paula, ResetKind::Cold);
    assert_eq!(rig.peek(INTENAR), 0);
    assert_eq!(rig.peek(INTREQR), int::PORTS, "the CIA is still requesting");
}

fn snapshot(p: &Paula) -> Vec<u8> {
    let mut shape = MachineShape::new();
    shape.add_device("paula", CLASS_NAME).unwrap();
    let mut w = StateWriter::new(shape);
    {
        let mut chunk = w.chunk("paula", CLASS_NAME, STATE_VERSION).unwrap();
        Device::save(p, &mut chunk).unwrap();
    }
    w.to_vec().unwrap()
}

#[test]
fn a_snapshot_round_trips_to_identical_state() {
    let rig = Rig::new();
    rig.paula.port().attach_drive(track());
    rig.poke(INTENA, 0xc000 | int::RBF | int::AUD0);
    rig.poke(ADKCON, 0x8000 | ADKF_FAST | ADKF_WORDSYNC);
    rig.poke(DSKSYNC, 0x4489);
    rig.poke(DMACON, DMAF_SETCLR | DMAF_MASTER | DMAF_DISK | DMAF_AUD0);
    rig.poke(DSKLEN, 0x8004);
    rig.poke(DSKLEN, 0x8004);
    rig.poke(AUD0LEN, 5);
    rig.poke(AUD0PER, 130);
    rig.poke(SERPER, 3);
    rig.poke(SERDAT, 0x0133);
    rig.poke(SERDAT, 0x0134);
    rig.paula.advance_to(500);
    let bytes = snapshot(&rig.paula);

    let other = Rig::new();
    let reader = StateReader::new(&bytes).unwrap();
    let chunk = reader
        .load("paula", CLASS_NAME, STATE_VERSION, &Migrations::new())
        .unwrap();
    Device::load(&other.paula, &mut chunk.reader()).unwrap();
    assert_eq!(snapshot(&other.paula), bytes, "identical state");
    assert_eq!(other.paula.ticks(), 500);
    assert_eq!(other.paula.ipl(), rig.paula.ipl());
}

#[test]
fn the_class_needs_its_register_space_and_says_what_it_has() {
    use crate::core::props::{Link, Value};
    let props = Props::new().with("custom", Value::Link(Link::new("custom").unwrap()));
    assert!(Paula::new(&props).is_ok());
    assert!(Paula::new(&Props::new()).is_err(), "no `custom`");
    assert!(Paula::new(&props.clone().with("agnus", Value::from(1u64))).is_err());

    let p = Paula::new(&props).unwrap();
    assert_eq!(p.port_name(), "serial");
    let schema = schema();
    for pin in IPL_PINS.iter().chain([INT2_PIN, INT6_PIN].iter()) {
        assert!(schema.port_named(pin).is_some(), "{pin}");
    }
    assert!(p.sink(INT2_PIN, &[WireId::new(1)]).is_some());
    assert!(p.sink("ipl0", &[WireId::new(1)]).is_none(), "an output");
    let src = WireId::new(9);
    let wire = Wire::builder().source(src).build_shared();
    assert!(p.connect("int2", WireSource::new(wire, src)).is_err());
    assert!(p.export(ExportId::PAULA).is_some());
    assert!(p.export(ExportId::CUSTOM_BUS).is_none());
}
