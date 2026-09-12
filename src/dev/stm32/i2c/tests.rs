//! Tests for the STM32 I²C v1 peripheral.
//!
//! Two of these carry the weight.
//!
//! [`a_debug_dump_of_the_whole_block_clears_nothing`](tests::a_debug_dump_of_the_whole_block_clears_nothing)
//! is the [`MemAttrs::debug`] rule on the sharpest register file in the tree:
//! §25.6.6 clears `ADDR` when software reads `SR1` and then `SR2`, so a
//! debugger that dumped the block would clear it out from under the guest and
//! hang its driver — a bug that only appears when somebody attaches gdb.
//!
//! [`both_link_models_write_the_same_page_to_the_same_eeprom`](tests::both_link_models_write_the_same_page_to_the_same_eeprom)
//! is the `docs/buses/low-speed.md` claim carried all the way through a real
//! register block and a real device: the same firmware-shaped sequence of
//! register accesses leaves the same bytes in an EEPROM, and takes the same
//! number of ticks doing it.

use super::*;

use alloc::vec::Vec;

use crate::bus::i2c::wires::{ControllerWires, pin as line};
use crate::core::device::{Device, ResetKind};
use crate::core::props::{Props, Value};
use crate::core::space::{RegionKind, RegionRef};
use crate::core::state::{MachineShape, Migrations, StateReader, StateWriter};
use crate::core::wire::{Wire, WireId, WireSource};
use crate::dev::atmel::at24c::At24c;

// ---------------------------------------------------------------------------
// Register access
// ---------------------------------------------------------------------------

fn regs(ctrl: &Stm32I2c) -> RegionRef {
    ctrl.region("").expect("the peripheral maps its registers")
}

fn ops(region: &RegionRef) -> Arc<dyn MemOps> {
    match region.kind() {
        RegionKind::Io(ops) => Arc::clone(ops),
        other => panic!("expected an io region, got {other:?}"),
    }
}

fn poke(region: &RegionRef, offset: u64, value: u32) {
    ops(region)
        .write(offset, &value.to_le_bytes(), MemAttrs::DEFAULT)
        .expect("a 32-bit register write");
}

fn peek_with(region: &RegionRef, offset: u64, attrs: MemAttrs) -> u32 {
    let mut buf = [0u8; 4];
    ops(region)
        .read(offset, &mut buf, attrs)
        .expect("a 32-bit register read");
    u32::from_le_bytes(buf)
}

fn peek(region: &RegionRef, offset: u64) -> u32 {
    peek_with(region, offset, MemAttrs::DEFAULT)
}

const CR1: u64 = 0x00;
const CR2: u64 = 0x04;
const OAR1: u64 = 0x08;
const OAR2: u64 = 0x0c;
const DR: u64 = 0x10;
const SR1: u64 = 0x14;
const SR2: u64 = 0x18;
const CCR: u64 = 0x1c;
const TRISE: u64 = 0x20;

// ---------------------------------------------------------------------------
// A board: a controller, an EEPROM, and one shared clock
// ---------------------------------------------------------------------------

/// One `CCR`, and therefore `Tlow = Thigh = 4` peripheral clocks.
const CCR_VALUE_USED: u32 = 4;

/// A controller and an AT24C02 on the same virtual clock.
///
/// Both are lazily advanced devices with their own tick, so the test drives
/// them together; a machine file gives them a clock domain each and the
/// scheduler does it.
struct Board {
    ctrl: Stm32I2c,
    region: RegionRef,
    eeprom: At24c,
    now: u64,
}

impl Board {
    fn new(link: Link) -> Board {
        let bus = Arc::new(I2cBus::new());
        let mut p = Props::new();
        let eeprom = At24c::new(&p).expect("an EEPROM");
        let ctrl = match link {
            Link::Transactional => {
                bus.attach(eeprom.slave()).expect("room on the bus");
                Stm32I2c::with_bus(Link::Transactional, Some(Arc::clone(&bus)))
                    .expect("a controller")
            }
            Link::Wired => {
                let ctrl = Stm32I2c::with_bus(Link::Wired, None).expect("a wired controller");
                wire_up(&ctrl, &eeprom);
                ctrl
            }
        };
        p = Props::new();
        let _ = &mut p;
        let region = regs(&ctrl);
        Board {
            ctrl,
            region,
            eeprom,
            now: 0,
        }
    }

    /// Let `ticks` of the peripheral clock pass, for both devices.
    fn step(&mut self, ticks: u64) {
        self.now += ticks;
        self.ctrl.advance_to(self.now);
        self.eeprom.advance_to(self.now);
    }

    /// Run until `SR1` has every bit of `flags`, or give up.
    fn wait(&mut self, flags: u32) -> u32 {
        for _ in 0..4_000 {
            let sr1 = peek_with(&self.region, SR1, MemAttrs::DEBUG);
            if sr1 & flags == flags {
                return sr1;
            }
            if sr1 & SR1_AF != 0 {
                panic!("the transfer was not acknowledged while waiting for {flags:#x}");
            }
            self.step(1);
        }
        panic!(
            "gave up waiting for {flags:#x}; SR1 is {:#x}",
            peek_with(&self.region, SR1, MemAttrs::DEBUG)
        );
    }

    /// The initialisation §25.3.3 lists, in its order.
    fn init(&mut self) {
        poke(&self.region, CR2, 42);
        poke(&self.region, CCR, CCR_VALUE_USED);
        poke(&self.region, TRISE, 43);
        poke(&self.region, CR1, CR1_PE);
    }

    /// A page write, driven exactly as a driver would drive it.
    fn write_page(&mut self, address: u8, word: u8, data: &[u8]) {
        poke(&self.region, CR1, CR1_PE | CR1_ACK | CR1_START);
        self.wait(SR1_SB);
        // EV5: read SR1, then write DR with the address.
        peek(&self.region, SR1);
        poke(&self.region, DR, u32::from(address << 1));
        self.wait(SR1_ADDR);
        // EV6: read SR1, then read SR2.
        peek(&self.region, SR1);
        peek(&self.region, SR2);

        self.wait(SR1_TXE);
        poke(&self.region, DR, u32::from(word));
        for byte in data {
            self.wait(SR1_TXE);
            poke(&self.region, DR, u32::from(*byte));
        }
        // EV8_2: TxE and BTF are both set; program the STOP.
        self.wait(SR1_TXE | SR1_BTF);
        poke(&self.region, CR1, CR1_PE | CR1_ACK | CR1_STOP);
        for _ in 0..200 {
            if peek_with(&self.region, SR2, MemAttrs::DEBUG) & SR2_MSL == 0 {
                return;
            }
            self.step(1);
        }
        panic!("the STOP never completed");
    }

    /// A random read: a dummy write for the word address, a repeated START,
    /// then `count` bytes with `ACK` cleared before the last one.
    fn read_from(&mut self, address: u8, word: u8, count: usize) -> Vec<u8> {
        poke(&self.region, CR1, CR1_PE | CR1_ACK | CR1_START);
        self.wait(SR1_SB);
        peek(&self.region, SR1);
        poke(&self.region, DR, u32::from(address << 1));
        self.wait(SR1_ADDR);
        peek(&self.region, SR1);
        peek(&self.region, SR2);
        self.wait(SR1_TXE);
        poke(&self.region, DR, u32::from(word));
        self.wait(SR1_BTF);

        // The repeated START turns the transfer round.
        poke(&self.region, CR1, CR1_PE | CR1_ACK | CR1_START);
        self.wait(SR1_SB);
        peek(&self.region, SR1);
        poke(&self.region, DR, u32::from((address << 1) | 1));
        self.wait(SR1_ADDR);
        peek(&self.region, SR1);
        peek(&self.region, SR2);

        let mut out = Vec::new();
        for i in 0..count {
            if i + 1 == count {
                // §25.3.3: "the ACK bit must be cleared just after reading the
                // second last data byte", and the STOP set at the same time.
                poke(&self.region, CR1, CR1_PE | CR1_STOP);
            }
            self.wait(SR1_RXNE);
            out.push(peek(&self.region, DR) as u8);
        }
        for _ in 0..200 {
            if peek_with(&self.region, SR2, MemAttrs::DEBUG) & SR2_MSL == 0 {
                break;
            }
            self.step(1);
        }
        out
    }
}

/// Put the controller and the EEPROM on two open-drain nets, as a machine
/// file's four `wire` statements do.
fn wire_up(ctrl: &Stm32I2c, eeprom: &At24c) {
    let master: &Arc<ControllerWires> = ctrl.wires();
    let slave = Arc::clone(eeprom.wires());
    let scl_ids = [WireId::new(1), WireId::new(2)];
    let sda_ids = [WireId::new(3), WireId::new(4)];
    let scl = Wire::builder()
        .sources(&scl_ids)
        .sink(master.sink(line::SCL, &scl_ids), line::SCL)
        .sink(slave.sink(line::SCL, &scl_ids), line::SCL)
        .build_shared();
    let sda = Wire::builder()
        .sources(&sda_ids)
        .sink(master.sink(line::SDA, &sda_ids), line::SDA)
        .sink(slave.sink(line::SDA, &sda_ids), line::SDA)
        .build_shared();
    master.connect(line::SCL, WireSource::new(Arc::clone(&scl), scl_ids[0]));
    master.connect(line::SDA, WireSource::new(Arc::clone(&sda), sda_ids[0]));
    slave.connect(line::SCL, WireSource::new(Arc::clone(&scl), scl_ids[1]));
    slave.connect(line::SDA, WireSource::new(Arc::clone(&sda), sda_ids[1]));
    master.announce();
    slave.announce();
}

// ---------------------------------------------------------------------------
// Construction and the register file
// ---------------------------------------------------------------------------

#[test]
fn the_link_is_a_required_property_and_a_transactional_one_needs_a_bus() {
    let p = Props::new();
    assert!(
        Stm32I2c::new(&p).is_err(),
        "`link` has no default, by design"
    );

    let mut p = Props::new();
    p.insert("link", Value::Str("sideways".into()));
    let err = Stm32I2c::new(&p).expect_err("an unknown link");
    assert!(alloc::format!("{err}").contains("low-speed.md"));

    let mut p = Props::new();
    p.insert("link", Value::Str("transactional".into()));
    let err = Stm32I2c::new(&p).expect_err("no bus to reach");
    assert!(alloc::format!("{err}").contains("named bus"));

    let mut p = Props::new();
    p.insert("link", Value::Str("wired".into()));
    assert!(Stm32I2c::new(&p).is_ok(), "a wired link needs no bus");
}

#[test]
fn the_reset_values_are_the_ones_the_reference_manual_gives() {
    let ctrl = Stm32I2c::with_bus(Link::Wired, None).expect("a wired controller");
    let r = regs(&ctrl);
    // §25.6.1 to §25.6.10: everything resets to zero except TRISE, which is 2.
    assert_eq!(peek(&r, CR1), 0);
    assert_eq!(peek(&r, CR2), 0);
    assert_eq!(peek(&r, OAR1), 0);
    assert_eq!(peek(&r, DR), 0);
    assert_eq!(peek(&r, SR1), 0);
    assert_eq!(peek(&r, SR2), 0);
    assert_eq!(peek(&r, CCR), 0);
    assert_eq!(peek(&r, TRISE), 2, "§25.6.9's reset value");
}

#[test]
fn the_register_block_takes_half_words_and_words_and_nothing_else() {
    // §25.6: "The peripheral registers can be accessed by half-words (16 bits)
    // or words (32 bits)."
    let ctrl = Stm32I2c::with_bus(Link::Wired, None).expect("a wired controller");
    let r = regs(&ctrl);
    let block = ops(&r);
    let mut half = [0u8; 2];
    assert!(block.read(CR1, &mut half, MemAttrs::DEFAULT).is_ok());
    let mut byte = [0u8; 1];
    assert!(block.read(CR1, &mut byte, MemAttrs::DEFAULT).is_err());
    let mut word = [0u8; 4];
    assert!(block.read(0x02, &mut word, MemAttrs::DEFAULT).is_err());
    // A debug *write* is refused outright: it would start a transfer.
    assert!(block.write(CR1, &[1, 0], MemAttrs::DEBUG).is_err());
}

#[test]
fn a_software_reset_puts_everything_back() {
    // §25.6.1: "When set, the I2C is under reset state."
    let ctrl = Stm32I2c::with_bus(Link::Wired, None).expect("a wired controller");
    let r = regs(&ctrl);
    poke(&r, CCR, 0x1234);
    poke(&r, CR1, CR1_PE);
    poke(&r, CR1, CR1_SWRST);
    assert_eq!(peek(&r, CCR), 0);
    assert_eq!(peek(&r, CR1), CR1_SWRST);
    assert_eq!(peek(&r, TRISE), 2);
}

// ---------------------------------------------------------------------------
// The clearing sequences, and the debug rule
// ---------------------------------------------------------------------------

#[test]
fn addr_is_cleared_by_reading_sr1_and_then_sr2_and_by_nothing_else() {
    let mut board = Board::new(Link::Transactional);
    board.init();
    poke(&board.region, CR1, CR1_PE | CR1_ACK | CR1_START);
    board.wait(SR1_SB);
    peek(&board.region, SR1);
    poke(&board.region, DR, 0xa0);
    board.wait(SR1_ADDR);

    // SR2 on its own does nothing: the sequence is read SR1 *then* SR2.
    peek(&board.region, SR2);
    assert!(
        peek_with(&board.region, SR1, MemAttrs::DEBUG) & SR1_ADDR != 0,
        "ADDR must survive an SR2 read that was not preceded by an SR1 read"
    );
    peek(&board.region, SR1);
    peek(&board.region, SR2);
    assert_eq!(
        peek_with(&board.region, SR1, MemAttrs::DEBUG) & SR1_ADDR,
        0,
        "and the documented sequence clears it"
    );
}

#[test]
fn a_debug_dump_of_the_whole_block_clears_nothing() {
    // The invariant this peripheral is the sharpest test of in the tree
    // (`ROADMAP.md` §15, invariant 5). A debugger reads every register; the
    // guest must not be able to tell.
    let mut board = Board::new(Link::Transactional);
    board.init();
    poke(&board.region, CR1, CR1_PE | CR1_ACK | CR1_START);
    board.wait(SR1_SB);
    peek(&board.region, SR1);
    poke(&board.region, DR, 0xa0);
    board.wait(SR1_ADDR);

    // gdb dumps the register file. Twice, for good measure.
    for _ in 0..2 {
        for offset in [CR1, CR2, OAR1, 0x0c, DR, SR1, SR2, CCR, TRISE, 0x24] {
            let _ = peek_with(&board.region, offset, MemAttrs::DEBUG);
        }
    }
    assert!(
        peek_with(&board.region, SR1, MemAttrs::DEBUG) & SR1_ADDR != 0,
        "a debug SR1+SR2 read must not clear ADDR"
    );
    assert!(
        board.ctrl.stretching(),
        "and the peripheral is still stalled"
    );

    // And the guest's own sequence still works afterwards, which is the part
    // that would break if the debug read had merely failed to *clear* while
    // still arming the latch.
    peek(&board.region, SR1);
    peek(&board.region, SR2);
    assert_eq!(peek_with(&board.region, SR1, MemAttrs::DEBUG) & SR1_ADDR, 0);
}

#[test]
fn a_debug_read_of_the_data_register_pops_nothing() {
    let mut board = Board::new(Link::Transactional);
    board.init();
    board.write_page(0x50, 0x00, &[0x11, 0x22]);
    board.step(DEFAULT_EEPROM_WRITE);
    let got = board.read_from(0x50, 0x00, 1);
    assert_eq!(got, alloc::vec![0x11]);

    // Now with RxNE set, a debug read must leave it set.
    board.write_page(0x50, 0x00, &[0x33]);
    board.step(DEFAULT_EEPROM_WRITE);
    poke(&board.region, CR1, CR1_PE | CR1_ACK | CR1_START);
    board.wait(SR1_SB);
    peek(&board.region, SR1);
    poke(&board.region, DR, 0xa0);
    board.wait(SR1_ADDR);
    peek(&board.region, SR1);
    peek(&board.region, SR2);
    board.wait(SR1_TXE);
    poke(&board.region, DR, 0x00);
    board.wait(SR1_BTF);
    poke(&board.region, CR1, CR1_PE | CR1_ACK | CR1_START);
    board.wait(SR1_SB);
    peek(&board.region, SR1);
    poke(&board.region, DR, 0xa1);
    board.wait(SR1_ADDR);
    peek(&board.region, SR1);
    peek(&board.region, SR2);
    poke(&board.region, CR1, CR1_PE | CR1_STOP);
    board.wait(SR1_RXNE);

    let seen = peek_with(&board.region, DR, MemAttrs::DEBUG);
    assert_eq!(seen, 0x33);
    assert!(
        peek_with(&board.region, SR1, MemAttrs::DEBUG) & SR1_RXNE != 0,
        "a debug read of DR must not clear RxNE"
    );
    assert_eq!(peek(&board.region, DR), 0x33, "and the guest still gets it");
    assert_eq!(peek_with(&board.region, SR1, MemAttrs::DEBUG) & SR1_RXNE, 0);
}

/// tWR for the default EEPROM, in the ticks this test's shared clock counts.
const DEFAULT_EEPROM_WRITE: u64 = crate::dev::atmel::at24c::DEFAULT_WRITE_TICKS;

// ---------------------------------------------------------------------------
// Transfers
// ---------------------------------------------------------------------------

#[test]
fn a_guest_writes_a_page_to_an_eeprom_and_reads_it_back() {
    for link in [Link::Transactional, Link::Wired] {
        let mut board = Board::new(link);
        board.init();
        let page = [0xde_u8, 0xad, 0xbe, 0xef, 0x12, 0x34, 0x56, 0x78];
        board.write_page(0x50, 0x08, &page);
        board.step(DEFAULT_EEPROM_WRITE);
        assert_eq!(
            board.eeprom.byte(0x08),
            Some(0xde),
            "the page did not land under {link}"
        );
        assert_eq!(
            board.read_from(0x50, 0x08, page.len()),
            page.to_vec(),
            "and reading it back disagreed under {link}"
        );
    }
}

#[test]
fn both_link_models_write_the_same_page_to_the_same_eeprom() {
    // The `docs/buses/low-speed.md` claim, carried through a whole register
    // block: the same sequence of register accesses leaves the same array
    // behind and takes the same number of ticks doing it.
    let mut results = Vec::new();
    for link in [Link::Transactional, Link::Wired] {
        let mut board = Board::new(link);
        board.init();
        board.write_page(0x50, 0x10, &[1, 2, 3, 4, 5, 6, 7, 8]);
        board.step(DEFAULT_EEPROM_WRITE);
        results.push((board.eeprom.contents(), board.ctrl.ticks()));
    }
    assert_eq!(results[0].0, results[1].0, "the arrays differ");
    assert_eq!(
        results[0].1, results[1].1,
        "a transfer must cost the same virtual time either way"
    );
    assert_eq!(&results[0].0[0x10..0x18], &[1, 2, 3, 4, 5, 6, 7, 8]);
}

#[test]
fn an_address_nobody_answers_raises_acknowledge_failure() {
    // §25.6.6: `AF` is "set by hardware when no acknowledge is returned".
    for link in [Link::Transactional, Link::Wired] {
        let mut board = Board::new(link);
        board.init();
        poke(&board.region, CR1, CR1_PE | CR1_ACK | CR1_START);
        board.wait(SR1_SB);
        peek(&board.region, SR1);
        // 0x22 is not the EEPROM.
        poke(&board.region, DR, 0x44);
        let mut af = false;
        for _ in 0..200 {
            if peek_with(&board.region, SR1, MemAttrs::DEBUG) & SR1_AF != 0 {
                af = true;
                break;
            }
            board.step(1);
        }
        assert!(af, "no AF under {link}");
        assert_eq!(
            peek_with(&board.region, SR1, MemAttrs::DEBUG) & SR1_ADDR,
            0,
            "§25.6.6: ADDR is not set after a NACK reception"
        );
        // §25.6.6's `rc_w0`: writing a zero to the bit clears it.
        poke(&board.region, SR1, !SR1_AF);
        assert_eq!(peek_with(&board.region, SR1, MemAttrs::DEBUG) & SR1_AF, 0);
    }
}

#[test]
fn the_clock_control_register_is_what_a_transfer_costs() {
    // §25.6.8: standard mode has Thigh = Tlow = CCR * TPCLK1, and
    // `bus::i2c` fixes the half-period count of every bus event — so a START,
    // an address byte and a STOP cost exactly (4 + 18 + 2) / 2 bit periods.
    let mut board = Board::new(Link::Wired);
    board.init();
    let start = board.ctrl.ticks();
    board.write_page(0x50, 0x00, &[0xaa]);
    let spent = board.ctrl.ticks() - start;
    // START + address + word address + one data byte + STOP.
    let halves = u64::from(START_HALF_PERIODS + 3 * BYTE_HALF_PERIODS + STOP_HALF_PERIODS);
    let expected = halves / 2 * (2 * u64::from(CCR_VALUE_USED));
    assert_eq!(
        spent, expected,
        "a transfer costs the bit periods CCR asks for"
    );
}

#[test]
fn the_event_and_error_interrupts_follow_their_enable_bits() {
    // §25.6.2 lists exactly which flags raise which line.
    let mut board = Board::new(Link::Transactional);
    board.init();
    assert_eq!(board.ctrl.ev_level(), Level::Low);
    poke(&board.region, CR2, 42 | CR2_ITEVTEN | CR2_ITERREN);
    poke(&board.region, CR1, CR1_PE | CR1_ACK | CR1_START);
    board.wait(SR1_SB);
    assert_eq!(board.ctrl.ev_level(), Level::High, "SB raises EV");
    peek(&board.region, SR1);
    poke(&board.region, DR, 0x44);
    for _ in 0..200 {
        if peek_with(&board.region, SR1, MemAttrs::DEBUG) & SR1_AF != 0 {
            break;
        }
        board.step(1);
    }
    assert_eq!(board.ctrl.er_level(), Level::High, "AF raises ER");
    poke(&board.region, SR1, !SR1_AF);
    assert_eq!(board.ctrl.er_level(), Level::Low);
}

// ---------------------------------------------------------------------------
// Snapshots
// ---------------------------------------------------------------------------

#[test]
fn a_transfer_part_way_through_its_address_phase_round_trips() {
    let mut board = Board::new(Link::Wired);
    board.init();
    poke(&board.region, CR1, CR1_PE | CR1_ACK | CR1_START);
    board.wait(SR1_SB);
    // Stop *inside* the clearing sequence: SR1 has been read and DR has not
    // been written, which is the state the arming latch exists to remember.
    peek(&board.region, SR1);

    let mut shape = MachineShape::new();
    shape.add_device("i2c", ST_I2C_CLASS.name).unwrap();
    let mut w = StateWriter::new(shape.clone());
    {
        let mut chunk = w
            .chunk("i2c", ST_I2C_CLASS.name, ST_I2C_CLASS.version)
            .unwrap();
        board.ctrl.save(&mut chunk).unwrap();
    }
    let bytes = w.to_vec().unwrap();

    let other = Stm32I2c::with_bus(Link::Wired, None).expect("a wired controller");
    let reader = StateReader::new(&bytes).unwrap();
    let chunk = reader
        .load(
            "i2c",
            ST_I2C_CLASS.name,
            ST_I2C_CLASS.version,
            &Migrations::new(),
        )
        .unwrap();
    other.load(&mut chunk.reader()).unwrap();

    let mut w2 = StateWriter::new(shape);
    {
        let mut chunk = w2
            .chunk("i2c", ST_I2C_CLASS.name, ST_I2C_CLASS.version)
            .unwrap();
        other.save(&mut chunk).unwrap();
    }
    assert_eq!(w2.to_vec().unwrap(), bytes, "an identical state hash");

    // And the restored copy answers the same through its own registers.
    let other_region = regs(&other);
    for offset in [CR1, CR2, OAR1, DR, SR1, CCR, TRISE] {
        assert_eq!(
            peek_with(&other_region, offset, MemAttrs::DEBUG),
            peek_with(&board.region, offset, MemAttrs::DEBUG),
            "register {offset:#04x} differs after a round trip"
        );
    }
}

#[test]
fn a_reset_keeps_the_tick_and_drops_everything_else() {
    let mut board = Board::new(Link::Wired);
    board.init();
    board.write_page(0x50, 0x00, &[0x01]);
    let ticks = board.ctrl.ticks();
    assert!(ticks > 0);
    board.ctrl.reset(ResetKind::Cold);
    assert_eq!(
        board.ctrl.ticks(),
        ticks,
        "`Machine::reset` does not rewind clock domains"
    );
    assert_eq!(peek(&board.region, CR1), 0);
    assert_eq!(peek(&board.region, SR1), 0);
    assert_eq!(peek(&board.region, TRISE), 2);
}

// ---------------------------------------------------------------------------
// Slave mode (§25.3.2)
// ---------------------------------------------------------------------------

/// A controller with nothing else on its bus, for driving its slave face
/// directly.
fn slave_on_a_bus() -> (Stm32I2c, RegionRef, Arc<I2cBus>) {
    let bus = Arc::new(I2cBus::new());
    let ctrl =
        Stm32I2c::with_bus(Link::Transactional, Some(Arc::clone(&bus))).expect("a controller");
    let region = regs(&ctrl);
    poke(&region, CCR, CCR_VALUE_USED);
    poke(&region, CR1, CR1_PE | CR1_ACK);
    (ctrl, region, bus)
}

/// §25.3.2's `EV1`: `ADDR` is cleared by reading `SR1` then `SR2`.
fn clear_addr(region: &RegionRef) -> u32 {
    peek(region, SR1);
    peek(region, SR2)
}

#[test]
fn a_slave_answers_its_own_address_and_says_which_one_matched() {
    let (ctrl, r, bus) = slave_on_a_bus();
    poke(&r, OAR1, sadd7(0x42));

    assert_eq!(bus.start(Address::Seven(0x41), Direction::Write), Ack::Nack);
    assert_eq!(peek(&r, SR1) & SR1_ADDR, 0, "a near miss is a miss");

    assert_eq!(bus.start(Address::Seven(0x42), Direction::Write), Ack::Ack);
    assert_ne!(peek_with(&r, SR1, MemAttrs::DEBUG) & SR1_ADDR, 0, "EV1");
    assert!(
        ctrl.stretching(),
        "and ADDR holds SCL down until EV1 is done"
    );

    let sr2 = clear_addr(&r);
    assert_eq!(sr2 & SR2_MSL, 0, "a slave is not the master");
    assert_eq!(sr2 & SR2_TRA, 0, "and it is receiving");
    assert_eq!(sr2 & SR2_DUALF, 0);
    assert_eq!(sr2 & SR2_GENCALL, 0);
    assert_eq!(peek_with(&r, SR1, MemAttrs::DEBUG) & SR1_ADDR, 0);

    assert_eq!(bus.write(0xa5), Ack::Ack);
    assert_ne!(peek_with(&r, SR1, MemAttrs::DEBUG) & SR1_RXNE, 0, "EV2");
    assert_eq!(peek(&r, DR), 0xa5);

    bus.stop();
    assert_ne!(
        peek_with(&r, SR1, MemAttrs::DEBUG) & SR1_STOPF,
        0,
        "EV4: a STOP is a slave event"
    );
    // §25.6.6: `STOPF` is cleared by reading `SR1` then writing `CR1`.
    peek(&r, SR1);
    poke(&r, CR1, CR1_PE | CR1_ACK);
    assert_eq!(peek_with(&r, SR1, MemAttrs::DEBUG) & SR1_STOPF, 0);
}

#[test]
fn the_second_own_address_and_the_general_call_each_raise_their_own_sr2_bit() {
    // §25.6.4's `ENDUAL` and §25.6.1's `ENGC`, which are the two reasons `SR2`
    // has `DUALF` and `GENCALL` at all: `ADDR` alone does not say which of
    // three addresses matched.
    let (_ctrl, r, bus) = slave_on_a_bus();
    poke(&r, OAR1, sadd7(0x42));
    poke(&r, OAR2, sadd7(0x18) | OAR2_ENDUAL);
    poke(&r, CR1, CR1_PE | CR1_ACK | CR1_ENGC);

    assert_eq!(bus.start(Address::Seven(0x18), Direction::Write), Ack::Ack);
    let sr2 = clear_addr(&r);
    assert_ne!(sr2 & SR2_DUALF, 0, "OAR2 matched");
    assert_eq!(sr2 & SR2_GENCALL, 0);
    bus.stop();
    peek(&r, SR1);
    poke(&r, CR1, CR1_PE | CR1_ACK | CR1_ENGC);

    assert_eq!(bus.start(GENERAL_CALL, Direction::Write), Ack::Ack);
    let sr2 = clear_addr(&r);
    assert_ne!(sr2 & SR2_GENCALL, 0, "the general call matched");
    assert_eq!(sr2 & SR2_DUALF, 0);

    // With `ENDUAL` clear again `OAR2` answers nothing.
    bus.stop();
    poke(&r, OAR2, sadd7(0x18));
    assert_eq!(bus.start(Address::Seven(0x18), Direction::Write), Ack::Nack);
}

#[test]
fn a_slave_transmitter_serves_dr_and_stops_on_the_masters_nack() {
    let (_ctrl, r, bus) = slave_on_a_bus();
    poke(&r, OAR1, sadd7(0x42));

    assert_eq!(bus.start(Address::Seven(0x42), Direction::Read), Ack::Ack);
    let sr2 = clear_addr(&r);
    assert_ne!(sr2 & SR2_TRA, 0, "§25.6.7's TRA: the slave transmits");
    assert_ne!(
        peek_with(&r, SR1, MemAttrs::DEBUG) & SR1_TXE,
        0,
        "EV3_1: the data register is empty"
    );

    poke(&r, DR, 0x7e);
    assert_eq!(bus.read(Ack::Ack), 0x7e);
    assert_ne!(peek_with(&r, SR1, MemAttrs::DEBUG) & SR1_TXE, 0, "EV3");

    poke(&r, DR, 0x7f);
    assert_eq!(bus.read(Ack::Nack), 0x7f);
    assert_ne!(
        peek_with(&r, SR1, MemAttrs::DEBUG) & SR1_AF,
        0,
        "§25.3.2: a NACK from the master sets AF"
    );
}

#[test]
fn a_slave_with_ack_clear_answers_nothing_at_all() {
    // §25.6.1: "0: No acknowledge returned", and the address byte is a byte.
    let (_ctrl, r, bus) = slave_on_a_bus();
    poke(&r, OAR1, sadd7(0x42));
    poke(&r, CR1, CR1_PE);
    assert_eq!(bus.start(Address::Seven(0x42), Direction::Write), Ack::Nack);
    assert_eq!(peek_with(&r, SR1, MemAttrs::DEBUG) & SR1_ADDR, 0);
}

#[test]
fn a_controller_in_the_middle_of_its_own_transfer_is_deaf_to_its_own_address() {
    // A part cannot address itself. On a wired bus this is not a nicety: the
    // slave face is on the same two nets the master half is driving, so without
    // this every controller would answer its own address byte.
    let (_ctrl, r, bus) = slave_on_a_bus();
    poke(&r, OAR1, sadd7(0x42));
    poke(&r, CR1, CR1_PE | CR1_ACK | CR1_START);
    for _ in 0..4 {
        assert_eq!(bus.start(Address::Seven(0x42), Direction::Write), Ack::Nack);
    }
    assert_eq!(peek_with(&r, SR1, MemAttrs::DEBUG) & SR1_ADDR, 0);
}

#[test]
fn a_wired_controller_is_addressed_over_the_pins_it_would_drive() {
    // Issue #10 for the v1 block. Two `st.i2c` peripherals, one pair of
    // open-drain nets, no `I2cBus` anywhere.
    const PAYLOAD: [u8; 3] = [0xa0, 0xb1, 0xc2];
    /// The address the second block answers to.
    const TARGET: u8 = 0x42;

    let a = Stm32I2c::with_bus(Link::Wired, None).expect("a wired controller");
    let b = Stm32I2c::with_bus(Link::Wired, None).expect("a wired controller");
    let (ra, rb) = (regs(&a), regs(&b));
    wire_two(a.wires(), b.wires());
    for r in [&ra, &rb] {
        poke(r, CR2, 42);
        poke(r, CCR, CCR_VALUE_USED);
        poke(r, CR1, CR1_PE | CR1_ACK);
    }
    poke(&rb, OAR1, sadd7(TARGET));

    // A drives the transfer with §25.3.3's master sequence; B is served with
    // §25.3.2's slave one. Neither knows the other is an STM32.
    poke(&ra, CR1, CR1_PE | CR1_ACK | CR1_START);
    let mut now = 0u64;
    let mut sent = 0usize;
    let mut got = Vec::new();
    let mut b_addressed = false;
    let mut stalled = false;
    for _ in 0..40_000 {
        let sr1_b = peek_with(&rb, SR1, MemAttrs::DEBUG);
        if sr1_b & SR1_ADDR != 0 {
            if !b_addressed {
                stalled = b.stretching();
            }
            b_addressed = true;
            clear_addr(&rb);
            continue;
        }
        if sr1_b & SR1_RXNE != 0 {
            got.push(peek(&rb, DR) as u8);
            continue;
        }
        let sr1_a = peek_with(&ra, SR1, MemAttrs::DEBUG);
        assert_eq!(sr1_a & SR1_AF, 0, "the other block never answered");
        if sr1_a & SR1_SB != 0 {
            peek(&ra, SR1);
            poke(&ra, DR, u32::from(TARGET) << 1);
            continue;
        }
        if sr1_a & SR1_ADDR != 0 {
            peek(&ra, SR1);
            peek(&ra, SR2);
            continue;
        }
        if sr1_a & SR1_TXE != 0 && sent < PAYLOAD.len() {
            poke(&ra, DR, u32::from(PAYLOAD[sent]));
            sent += 1;
            continue;
        }
        if sent == PAYLOAD.len() && sr1_a & (SR1_TXE | SR1_BTF) == (SR1_TXE | SR1_BTF) {
            poke(&ra, CR1, CR1_PE | CR1_ACK | CR1_STOP);
            sent += 1;
            continue;
        }
        if sent > PAYLOAD.len() && peek_with(&rb, SR1, MemAttrs::DEBUG) & SR1_STOPF != 0 {
            break;
        }
        now += 1;
        a.advance_to(now);
        b.advance_to(now);
    }
    assert!(b_addressed, "the second block never saw its own address");
    assert!(stalled, "and it held SCL down until EV1 was done");
    assert_eq!(got, PAYLOAD.to_vec(), "the bytes that reached it");
    assert_ne!(
        peek_with(&rb, SR1, MemAttrs::DEBUG) & SR1_STOPF,
        0,
        "EV4, seen on the wire rather than through a fabric call"
    );
}

/// The `OAR1`/`OAR2` spelling of a seven-bit own address: `ADD[7:1]`.
const fn sadd7(address: u8) -> u32 {
    (address as u32) << 1
}

/// Put two combined engines on one SCL/SDA pair.
fn wire_two(a: &Arc<ControllerWires>, b: &Arc<ControllerWires>) -> (Arc<Wire>, Arc<Wire>) {
    let scl_ids = [WireId::new(1), WireId::new(2)];
    let sda_ids = [WireId::new(3), WireId::new(4)];
    let scl = Wire::builder()
        .sources(&scl_ids)
        .sink(a.sink(line::SCL, &scl_ids), line::SCL)
        .sink(b.sink(line::SCL, &scl_ids), line::SCL)
        .build_shared();
    let sda = Wire::builder()
        .sources(&sda_ids)
        .sink(a.sink(line::SDA, &sda_ids), line::SDA)
        .sink(b.sink(line::SDA, &sda_ids), line::SDA)
        .build_shared();
    a.connect(line::SCL, WireSource::new(Arc::clone(&scl), scl_ids[0]));
    a.connect(line::SDA, WireSource::new(Arc::clone(&sda), sda_ids[0]));
    b.connect(line::SCL, WireSource::new(Arc::clone(&scl), scl_ids[1]));
    b.connect(line::SDA, WireSource::new(Arc::clone(&sda), sda_ids[1]));
    a.announce();
    b.announce();
    (scl, sda)
}
