//! Tests for the STM32 I²C v2 peripheral.
//!
//! Three of these carry the weight.
//!
//! [`a_guest_writes_a_page_to_an_eeprom_and_reads_it_back`](tests::a_guest_writes_a_page_to_an_eeprom_and_reads_it_back)
//! is what makes this a controller rather than a register mock: an
//! `atmel.at24c` — a part written from its own datasheet, with no idea which
//! STM32 is talking to it — is driven through the `NBYTES`/`AUTOEND` machine in
//! both link models and gives its bytes back.
//!
//! [`the_v1_and_v2_blocks_leave_the_same_bytes_in_the_same_eeprom`](tests::the_v1_and_v2_blocks_leave_the_same_bytes_in_the_same_eeprom)
//! is the fork made honest. Two completely different register files, two
//! completely different driver shapes, one EEPROM, one array of bytes at the
//! end. If that ever diverges, one of the two models is wrong about I²C rather
//! than about ST.
//!
//! [`a_debug_dump_of_the_whole_block_pops_nothing`](tests::a_debug_dump_of_the_whole_block_pops_nothing)
//! is the [`MemAttrs::debug`] rule. v2 is gentler than v1 — `ISR` is read-only
//! and `ICR` clears — but `RXDR` still pops, so a debugger that dumped the
//! block would eat the byte the guest was waiting for.

use super::*;

use alloc::vec::Vec;

use crate::bus::i2c::wires::{MasterWires, pin as line};
use crate::core::device::{Device, ResetKind};
use crate::core::props::{Props, Value};
use crate::core::space::{RegionKind, RegionRef};
use crate::core::state::{MachineShape, Migrations, StateReader, StateWriter};
use crate::core::sync::Mutex;
use crate::core::wire::{Wire, WireId, WireSource};
use crate::dev::atmel::at24c::At24c;

// ---------------------------------------------------------------------------
// Register access
// ---------------------------------------------------------------------------

fn regs(ctrl: &Stm32I2cV2) -> RegionRef {
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
const TIMINGR: u64 = 0x10;
const TIMEOUTR: u64 = 0x14;
const ISR: u64 = 0x18;
const ICR: u64 = 0x1c;
const PECR: u64 = 0x20;
const RXDR: u64 = 0x24;
const TXDR: u64 = 0x28;

/// The whole register file, for a debugger to sweep.
const EVERY_REGISTER: [u64; 11] = [
    CR1, CR2, OAR1, OAR2, TIMINGR, TIMEOUTR, ISR, ICR, PECR, RXDR, TXDR,
];

/// `TIMINGR` for `t_SCLL = t_SCLH = 4` peripheral clocks: `PRESC = 1`,
/// `SCLL = SCLH = 1`, which §39.4.5 turns into `(1+1) × (1+1)` either way.
const TIMINGR_USED: u32 = (1 << 28) | (1 << 8) | 1;

/// The half period `TIMINGR_USED` asks for, in ticks.
const HALF_PERIOD: u64 = 4;

/// The `CR2.SADD` field for a seven-bit address.
const fn sadd(address: u8) -> u32 {
    (address as u32) << 1
}

/// The `CR2.NBYTES` field.
const fn nb(count: u8) -> u32 {
    (count as u32) << CR2_NBYTES_SHIFT
}

/// tWR for the default EEPROM, in the ticks this test's shared clock counts.
const DEFAULT_EEPROM_WRITE: u64 = crate::dev::atmel::at24c::DEFAULT_WRITE_TICKS;

// ---------------------------------------------------------------------------
// A board: a controller, an EEPROM, and one shared clock
// ---------------------------------------------------------------------------

/// A controller and an AT24C02 on the same virtual clock.
///
/// Both are lazily advanced devices with their own tick, so the test drives
/// them together; a machine file gives them a clock domain each and the
/// scheduler does it.
struct Board {
    ctrl: Stm32I2cV2,
    region: RegionRef,
    eeprom: At24c,
    now: u64,
}

impl Board {
    fn new(link: Link) -> Board {
        let bus = Arc::new(I2cBus::new());
        let p = Props::new();
        let eeprom = At24c::new(&p).expect("an EEPROM");
        let ctrl = match link {
            Link::Transactional => {
                bus.attach(eeprom.slave()).expect("room on the bus");
                Stm32I2cV2::with_bus(Link::Transactional, Some(Arc::clone(&bus)))
                    .expect("room for the controller's own slave face")
            }
            Link::Wired => {
                let ctrl = Stm32I2cV2::with_bus(Link::Wired, None).expect("a wired controller");
                wire_up(&ctrl, &eeprom);
                ctrl
            }
        };
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

    /// `ISR` without disturbing anything.
    fn isr(&self) -> u32 {
        peek_with(&self.region, ISR, MemAttrs::DEBUG)
    }

    /// Run until `ISR` has every bit of `flags`, or give up.
    fn wait(&mut self, flags: u32) -> u32 {
        for _ in 0..4_000 {
            let isr = self.isr();
            if isr & flags == flags {
                return isr;
            }
            if isr & ISR_NACKF != 0 && flags & ISR_NACKF == 0 {
                panic!("the transfer was not acknowledged while waiting for {flags:#x}");
            }
            self.step(1);
        }
        panic!("gave up waiting for {flags:#x}; ISR is {:#x}", self.isr());
    }

    /// Run until `flags` appear, reporting whether they did.
    fn wait_for(&mut self, flags: u32) -> bool {
        for _ in 0..4_000 {
            if self.isr() & flags == flags {
                return true;
            }
            self.step(1);
        }
        false
    }

    /// §39.4.5's configuration: the timing first, then `PE`.
    fn init(&mut self) {
        poke(&self.region, TIMINGR, TIMINGR_USED);
        poke(&self.region, CR1, CR1_PE);
    }

    /// A page write, driven the way a v2 driver drives one: one `CR2` write
    /// naming the address, the length and `AUTOEND`, then a byte per `TXIS`.
    fn write_page(&mut self, address: u8, word: u8, data: &[u8]) {
        let count = u8::try_from(data.len() + 1).expect("a page fits in NBYTES");
        poke(
            &self.region,
            CR2,
            sadd(address) | nb(count) | CR2_AUTOEND | CR2_START,
        );
        self.wait(ISR_TXIS);
        poke(&self.region, TXDR, u32::from(word));
        for byte in data {
            self.wait(ISR_TXIS);
            poke(&self.region, TXDR, u32::from(*byte));
        }
        self.wait(ISR_STOPF);
        // §39.7.8: `STOPCF`.
        poke(&self.region, ICR, ISR_STOPF);
    }

    /// A random read: a one-byte write that ends on `TC` rather than a STOP, a
    /// repeated START, then `count` bytes ending with `AUTOEND`.
    fn read_from(&mut self, address: u8, word: u8, count: usize) -> Vec<u8> {
        // No `AUTOEND`, no `RELOAD`: §39.4.7's `TC`, with SCL stretched until
        // software says what happens next.
        poke(&self.region, CR2, sadd(address) | nb(1) | CR2_START);
        self.wait(ISR_TXIS);
        poke(&self.region, TXDR, u32::from(word));
        self.wait(ISR_TC);

        let n = u8::try_from(count).expect("a read fits in NBYTES");
        poke(
            &self.region,
            CR2,
            sadd(address) | CR2_RD_WRN | nb(n) | CR2_AUTOEND | CR2_START,
        );
        let mut out = Vec::new();
        for _ in 0..count {
            self.wait(ISR_RXNE);
            out.push(peek(&self.region, RXDR) as u8);
        }
        self.wait(ISR_STOPF);
        poke(&self.region, ICR, ISR_STOPF);
        out
    }
}

/// Put the controller and the EEPROM on two open-drain nets, as a machine
/// file's four `wire` statements do.
fn wire_up(ctrl: &Stm32I2cV2, eeprom: &At24c) {
    let master: &Arc<MasterWires> = ctrl.wires();
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
        Stm32I2cV2::new(&p).is_err(),
        "`link` has no default, by design"
    );

    let mut p = Props::new();
    p.insert("link", Value::Str("sideways".into()));
    let err = Stm32I2cV2::new(&p).expect_err("an unknown link");
    assert!(alloc::format!("{err}").contains("low-speed.md"));

    let mut p = Props::new();
    p.insert("link", Value::Str("transactional".into()));
    let err = Stm32I2cV2::new(&p).expect_err("no bus to reach");
    assert!(alloc::format!("{err}").contains("named bus"));

    let mut p = Props::new();
    p.insert("link", Value::Str("wired".into()));
    assert!(Stm32I2cV2::new(&p).is_ok(), "a wired link needs no bus");
}

#[test]
fn the_reset_values_are_the_ones_the_reference_manual_gives() {
    let ctrl = Stm32I2cV2::with_bus(Link::Wired, None).unwrap();
    let r = regs(&ctrl);
    // §39.7.1 to §39.7.12: everything resets to zero except `ISR`, whose
    // `TXE` is 1 because an empty transmit register is empty.
    assert_eq!(peek(&r, CR1), 0);
    assert_eq!(peek(&r, CR2), 0);
    assert_eq!(peek(&r, OAR1), 0);
    assert_eq!(peek(&r, OAR2), 0);
    assert_eq!(peek(&r, TIMINGR), 0);
    assert_eq!(peek(&r, TIMEOUTR), 0);
    assert_eq!(peek(&r, ISR), ISR_TXE, "§39.7.7's reset value, 0x0000_0001");
    assert_eq!(peek(&r, ICR), 0, "§39.7.8: write-only, reads zero");
    assert_eq!(peek(&r, PECR), 0);
    assert_eq!(peek(&r, RXDR), 0);
    assert_eq!(peek(&r, TXDR), 0);
}

#[test]
fn the_register_block_takes_half_words_and_words_and_nothing_else() {
    let ctrl = Stm32I2cV2::with_bus(Link::Wired, None).unwrap();
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
fn clearing_pe_is_the_software_reset_and_keeps_the_configuration() {
    // §39.4.1: "PE=0 ... the I2C performs a software reset". The registers that
    // describe the *board* — the timing, the own addresses — are not part of
    // that; the flags and the transfer are.
    let ctrl = Stm32I2cV2::with_bus(Link::Wired, None).unwrap();
    let r = regs(&ctrl);
    poke(&r, TIMINGR, TIMINGR_USED);
    poke(&r, OAR1, OAR1_OA1EN | sadd(0x42));
    poke(&r, CR1, CR1_PE);
    poke(&r, CR2, sadd(0x50) | nb(2) | CR2_START);
    poke(&r, CR1, 0);
    assert_eq!(peek(&r, TIMINGR), TIMINGR_USED, "the timing survives");
    assert_eq!(peek(&r, OAR1), OAR1_OA1EN | sadd(0x42), "so does OAR1");
    assert_eq!(peek(&r, CR2), 0, "the transfer does not");
    assert_eq!(peek(&r, ISR), ISR_TXE, "and the flags are back to reset");
}

#[test]
fn an_enabled_own_address_only_takes_its_enable_bit() {
    // §39.7.3: "OA1[9:0] and OA1MODE should be written only when OA1EN = 0",
    // which is why every driver clears the enable before programming.
    let ctrl = Stm32I2cV2::with_bus(Link::Wired, None).unwrap();
    let r = regs(&ctrl);
    poke(&r, OAR1, OAR1_OA1EN | sadd(0x42));
    poke(&r, OAR1, OAR1_OA1EN | sadd(0x11));
    assert_eq!(
        peek(&r, OAR1),
        OAR1_OA1EN | sadd(0x42),
        "the address did not move while the enable was set"
    );
    poke(&r, OAR1, 0);
    poke(&r, OAR1, OAR1_OA1EN | sadd(0x11));
    assert_eq!(peek(&r, OAR1), OAR1_OA1EN | sadd(0x11));
}

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
fn a_two_byte_write_with_autoend_ends_in_stopf() {
    // §39.4.7: with `AUTOEND` the hardware sends the STOP when `NBYTES` are
    // gone. There is no `TC`, nothing for software to program, and `BUSY` drops
    // by itself.
    let mut board = Board::new(Link::Transactional);
    board.init();
    poke(
        &board.region,
        CR2,
        sadd(0x50) | nb(2) | CR2_AUTOEND | CR2_START,
    );
    board.wait(ISR_TXIS);
    poke(&board.region, TXDR, 0x20);
    board.wait(ISR_TXIS);
    poke(&board.region, TXDR, 0x5a);
    board.wait(ISR_STOPF);
    assert_eq!(
        board.isr() & (ISR_TC | ISR_TCR),
        0,
        "§39.4.7: AUTOEND replaces TC, it does not accompany it"
    );
    assert_eq!(board.isr() & ISR_BUSY, 0, "the bus is free again");
    poke(&board.region, ICR, ISR_STOPF);
    assert_eq!(board.isr() & ISR_STOPF, 0, "STOPCF clears it");
    board.step(DEFAULT_EEPROM_WRITE);
    assert_eq!(board.eeprom.byte(0x20), Some(0x5a));
}

#[test]
fn a_transfer_that_ends_without_autoend_sets_tc_and_holds_the_clock() {
    // §39.4.7's third case: neither `RELOAD` nor `AUTOEND`, so `TC` is set and
    // SCL stays low until software writes `START` or `STOP`.
    let mut board = Board::new(Link::Transactional);
    board.init();
    poke(&board.region, CR2, sadd(0x50) | nb(1) | CR2_START);
    board.wait(ISR_TXIS);
    poke(&board.region, TXDR, 0x00);
    board.wait(ISR_TC);
    assert!(board.ctrl.stretching(), "TC holds the clock");
    assert_ne!(board.isr() & ISR_BUSY, 0, "and the bus is still ours");

    // A STOP releases it, and `TC` goes with the `CR2` write that asked.
    poke(&board.region, CR2, sadd(0x50) | CR2_STOP);
    assert_eq!(
        board.isr() & ISR_TC,
        0,
        "§39.7.7: TC is cleared when START or STOP is set, and ICR cannot"
    );
    board.wait(ISR_STOPF);
}

#[test]
fn icr_cannot_clear_tc_and_tcr() {
    // The one edge of §39.7.8 a driver written against v1 gets wrong: `ICR` has
    // no `TCCF`. Writing every bit of it leaves `TC` exactly where it was.
    let mut board = Board::new(Link::Transactional);
    board.init();
    poke(&board.region, CR2, sadd(0x50) | nb(1) | CR2_START);
    board.wait(ISR_TXIS);
    poke(&board.region, TXDR, 0x00);
    board.wait(ISR_TC);
    poke(&board.region, ICR, u32::MAX);
    assert_ne!(board.isr() & ISR_TC, 0, "ICR has no TCCF");
    poke(&board.region, CR2, sadd(0x50) | CR2_STOP);
    board.wait(ISR_STOPF);
}

#[test]
fn a_reload_sets_tcr_and_a_new_nbytes_resumes_the_same_leg() {
    // The other half of the counter machine. `RELOAD` means "this is not the
    // end": `TCR`, a stretched clock, and then more bytes **without** a new
    // address phase.
    for link in [Link::Transactional, Link::Wired] {
        let mut board = Board::new(link);
        board.init();
        // Two bytes, then reload for two more: the word address plus three data
        // bytes, in two legs of two.
        poke(
            &board.region,
            CR2,
            sadd(0x50) | nb(2) | CR2_RELOAD | CR2_START,
        );
        board.wait(ISR_TXIS);
        poke(&board.region, TXDR, 0x30);
        board.wait(ISR_TXIS);
        poke(&board.region, TXDR, 0x01);
        board.wait(ISR_TCR);
        assert_eq!(board.isr() & ISR_TC, 0, "RELOAD replaces TC too");
        assert!(board.ctrl.stretching(), "TCR holds the clock");

        // Reload with `AUTOEND` for the tail. No START: the same transfer
        // continues, which is what makes `RELOAD` worth having.
        poke(&board.region, CR2, sadd(0x50) | nb(2) | CR2_AUTOEND);
        assert_eq!(board.isr() & ISR_TCR, 0, "a non-zero NBYTES clears TCR");
        board.wait(ISR_TXIS);
        poke(&board.region, TXDR, 0x02);
        board.wait(ISR_TXIS);
        poke(&board.region, TXDR, 0x03);
        board.wait(ISR_STOPF);
        poke(&board.region, ICR, ISR_STOPF);

        board.step(DEFAULT_EEPROM_WRITE);
        assert_eq!(
            &board.eeprom.contents()[0x30..0x33],
            &[0x01, 0x02, 0x03],
            "the reload did not continue the same transfer under {link}"
        );
    }
}

#[test]
fn a_master_receiver_acknowledges_through_a_reload_and_nacks_at_the_end() {
    // §39.4.7: the master NACKs the last byte of `NBYTES` — unless `RELOAD` is
    // set, in which case there is more to come and it must acknowledge. The
    // EEPROM is the witness: a NACK ends its sequential read (datasheet §8.3),
    // so a wrongly placed one loses every byte after it.
    let mut board = Board::new(Link::Transactional);
    board.init();
    let page = [1_u8, 2, 3, 4];
    board.write_page(0x50, 0x40, &page);
    board.step(DEFAULT_EEPROM_WRITE);

    poke(&board.region, CR2, sadd(0x50) | nb(1) | CR2_START);
    board.wait(ISR_TXIS);
    poke(&board.region, TXDR, 0x40);
    board.wait(ISR_TC);

    // Two bytes with `RELOAD`, then two more with `AUTOEND`.
    poke(
        &board.region,
        CR2,
        sadd(0x50) | CR2_RD_WRN | nb(2) | CR2_RELOAD | CR2_START,
    );
    let mut out = Vec::new();
    for _ in 0..2 {
        board.wait(ISR_RXNE);
        out.push(peek(&board.region, RXDR) as u8);
    }
    board.wait(ISR_TCR);
    poke(
        &board.region,
        CR2,
        sadd(0x50) | CR2_RD_WRN | nb(2) | CR2_AUTOEND,
    );
    for _ in 0..2 {
        board.wait(ISR_RXNE);
        out.push(peek(&board.region, RXDR) as u8);
    }
    board.wait(ISR_STOPF);
    assert_eq!(
        out,
        page.to_vec(),
        "the reload dropped the tail of the read"
    );
}

#[test]
fn an_absent_slave_sets_nackf_and_stops_by_itself() {
    // §39.4.8: "In master mode, the STOP condition is automatically generated
    // after a NACK reception." That is the difference from v1 that most changes
    // a driver: there is no STOP to program and no bus left hanging.
    for link in [Link::Transactional, Link::Wired] {
        let mut board = Board::new(link);
        board.init();
        // 0x58 is not the EEPROM.
        poke(
            &board.region,
            CR2,
            sadd(0x58) | nb(2) | CR2_AUTOEND | CR2_START,
        );
        assert!(board.wait_for(ISR_NACKF), "no NACKF under {link}");
        assert!(
            board.wait_for(ISR_STOPF),
            "the hardware did not stop by itself under {link}"
        );
        assert_eq!(board.isr() & ISR_BUSY, 0, "and it let the bus go");
        assert_eq!(
            board.isr() & ISR_TXIS,
            0,
            "§39.7.7: nothing more is asked for after a NACK"
        );
        poke(&board.region, ICR, ISR_NACKF | ISR_STOPF);
        assert_eq!(board.isr() & (ISR_NACKF | ISR_STOPF), 0);
    }
}

#[test]
fn a_write_polled_by_readdressing_sees_the_eeprom_write_cycle() {
    // Acknowledge polling, datasheet §5.3: while the AT24C is running its
    // internally self-timed write it answers nothing, so a re-addressing master
    // gets a NACK — and on v2 that NACK arrives as `NACKF` with a STOP already
    // sent, so the poll loop is one `CR2` write per attempt.
    let mut board = Board::new(Link::Transactional);
    board.init();
    board.write_page(0x50, 0x00, &[0x77]);

    let mut refusals = 0;
    let mut accepted = false;
    for _ in 0..200 {
        poke(
            &board.region,
            CR2,
            sadd(0x50) | nb(0) | CR2_AUTOEND | CR2_START,
        );
        assert!(board.wait_for(ISR_STOPF), "the probe never ended");
        if board.isr() & ISR_NACKF != 0 {
            refusals += 1;
            poke(&board.region, ICR, ISR_NACKF | ISR_STOPF);
        } else {
            poke(&board.region, ICR, ISR_STOPF);
            accepted = true;
            break;
        }
        board.step(DEFAULT_EEPROM_WRITE / 8);
    }
    assert!(refusals > 0, "the EEPROM answered during its write cycle");
    assert!(accepted, "and it never came back");
    assert_eq!(board.eeprom.byte(0x00), Some(0x77));
}

#[test]
fn the_wired_and_transactional_links_produce_the_same_bytes_and_timeline() {
    // The `docs/buses/low-speed.md` claim, carried through a whole register
    // block: the same sequence of register accesses leaves the same array
    // behind and takes the same number of ticks doing it.
    let mut results = Vec::new();
    for link in [Link::Transactional, Link::Wired] {
        let mut board = Board::new(link);
        board.init();
        board.write_page(0x50, 0x10, &[1, 2, 3, 4, 5, 6, 7, 8]);
        board.step(DEFAULT_EEPROM_WRITE);
        let read = board.read_from(0x50, 0x10, 8);
        results.push((board.eeprom.contents(), board.ctrl.ticks(), read));
    }
    assert_eq!(results[0].0, results[1].0, "the arrays differ");
    assert_eq!(results[0].2, results[1].2, "the bytes read back differ");
    assert_eq!(
        results[0].1, results[1].1,
        "a transfer must cost the same virtual time either way"
    );
    assert_eq!(&results[0].0[0x10..0x18], &[1, 2, 3, 4, 5, 6, 7, 8]);
}

#[cfg(feature = "dev-stm32-i2c")]
#[test]
fn the_v1_and_v2_blocks_leave_the_same_bytes_in_the_same_eeprom() {
    // The fork made honest. `st.i2c` and `st.i2c-v2` share no register, no flag
    // and no driver shape; what they share is I²C. Drive the same EEPROM
    // through each and the array has to come out identical — otherwise one of
    // the two is wrong about the *bus* rather than about ST.
    use crate::dev::stm32::i2c::Stm32I2c;

    const PAGE: [u8; 8] = [0xde, 0xad, 0xbe, 0xef, 0x12, 0x34, 0x56, 0x78];

    // v2, through the driver shape this module is for.
    let mut v2 = Board::new(Link::Transactional);
    v2.init();
    v2.write_page(0x50, 0x08, &PAGE);
    v2.step(DEFAULT_EEPROM_WRITE);
    let v2_read = v2.read_from(0x50, 0x08, PAGE.len());
    let v2_array = v2.eeprom.contents();

    // v1, through its own: `CCR`, `DR`, and the read-`SR1`-then-do sequences.
    // Written out here rather than shared, because there is nothing to share —
    // that is the entire point of the fork.
    let bus = Arc::new(I2cBus::new());
    let eeprom = At24c::new(&Props::new()).expect("an EEPROM");
    bus.attach(eeprom.slave()).expect("room on the bus");
    let ctrl = Stm32I2c::with_bus(Link::Transactional, Some(Arc::clone(&bus)));
    let r = regs_v1(&ctrl);
    let mut now = 0_u64;
    let mut step = |now: &mut u64, n: u64| {
        *now += n;
        ctrl.advance_to(*now);
        eeprom.advance_to(*now);
    };
    // `CCR` of 4 is v1's spelling of the same four-tick half period
    // `TIMINGR_USED` asks for (RM0090 §25.6.8 against RM0351 §39.4.5).
    poke(&r, 0x1c, 4);
    poke(&r, 0x00, 1);
    v1_write_page(&r, &mut now, &mut step, 0x50, 0x08, &PAGE);
    step(&mut now, DEFAULT_EEPROM_WRITE);
    let v1_read = v1_read_from(&r, &mut now, &mut step, 0x50, 0x08, PAGE.len());

    assert_eq!(v1_read, PAGE.to_vec(), "the v1 block read something else");
    assert_eq!(v2_read, PAGE.to_vec(), "the v2 block read something else");
    assert_eq!(
        eeprom.contents(),
        v2_array,
        "two STM32 I2C blocks left different arrays behind"
    );
}

/// v1's register block, for the cross-check above.
#[cfg(feature = "dev-stm32-i2c")]
fn regs_v1(ctrl: &crate::dev::stm32::i2c::Stm32I2c) -> RegionRef {
    ctrl.region("").expect("the peripheral maps its registers")
}

/// A page write as RM0090 §25.3.3 asks for one. v1 bit names, spelled out.
#[cfg(feature = "dev-stm32-i2c")]
fn v1_write_page(
    r: &RegionRef,
    now: &mut u64,
    step: &mut impl FnMut(&mut u64, u64),
    address: u8,
    word: u8,
    data: &[u8],
) {
    const PE: u32 = 1;
    const START: u32 = 1 << 8;
    const STOP: u32 = 1 << 9;
    const ACK: u32 = 1 << 10;
    const SB: u32 = 1;
    const ADDR: u32 = 1 << 1;
    const BTF: u32 = 1 << 2;
    const TXE: u32 = 1 << 7;
    let wait = |now: &mut u64, step: &mut dyn FnMut(&mut u64, u64), flags: u32| {
        for _ in 0..4_000 {
            if peek_with(r, 0x14, MemAttrs::DEBUG) & flags == flags {
                return;
            }
            step(now, 1);
        }
        panic!("v1 never reached {flags:#x}");
    };
    poke(r, 0x00, PE | ACK | START);
    wait(now, step, SB);
    peek(r, 0x14);
    poke(r, 0x10, u32::from(address << 1));
    wait(now, step, ADDR);
    peek(r, 0x14);
    peek(r, 0x18);
    wait(now, step, TXE);
    poke(r, 0x10, u32::from(word));
    for byte in data {
        wait(now, step, TXE);
        poke(r, 0x10, u32::from(*byte));
    }
    wait(now, step, TXE | BTF);
    poke(r, 0x00, PE | ACK | STOP);
    for _ in 0..200 {
        if peek_with(r, 0x18, MemAttrs::DEBUG) & 1 == 0 {
            return;
        }
        step(now, 1);
    }
    panic!("v1's STOP never completed");
}

/// A random read as RM0090 §25.3.3 asks for one.
#[cfg(feature = "dev-stm32-i2c")]
fn v1_read_from(
    r: &RegionRef,
    now: &mut u64,
    step: &mut impl FnMut(&mut u64, u64),
    address: u8,
    word: u8,
    count: usize,
) -> Vec<u8> {
    const PE: u32 = 1;
    const START: u32 = 1 << 8;
    const STOP: u32 = 1 << 9;
    const ACK: u32 = 1 << 10;
    const SB: u32 = 1;
    const ADDR: u32 = 1 << 1;
    const BTF: u32 = 1 << 2;
    const RXNE: u32 = 1 << 6;
    const TXE: u32 = 1 << 7;
    let wait = |now: &mut u64, step: &mut dyn FnMut(&mut u64, u64), flags: u32| {
        for _ in 0..4_000 {
            if peek_with(r, 0x14, MemAttrs::DEBUG) & flags == flags {
                return;
            }
            step(now, 1);
        }
        panic!("v1 never reached {flags:#x}");
    };
    poke(r, 0x00, PE | ACK | START);
    wait(now, step, SB);
    peek(r, 0x14);
    poke(r, 0x10, u32::from(address << 1));
    wait(now, step, ADDR);
    peek(r, 0x14);
    peek(r, 0x18);
    wait(now, step, TXE);
    poke(r, 0x10, u32::from(word));
    wait(now, step, BTF);

    poke(r, 0x00, PE | ACK | START);
    wait(now, step, SB);
    peek(r, 0x14);
    poke(r, 0x10, u32::from((address << 1) | 1));
    wait(now, step, ADDR);
    peek(r, 0x14);
    peek(r, 0x18);

    let mut out = Vec::new();
    for i in 0..count {
        if i + 1 == count {
            poke(r, 0x00, PE | STOP);
        }
        wait(now, step, RXNE);
        out.push(peek(r, 0x10) as u8);
    }
    for _ in 0..200 {
        if peek_with(r, 0x18, MemAttrs::DEBUG) & 1 == 0 {
            break;
        }
        step(now, 1);
    }
    out
}

// ---------------------------------------------------------------------------
// Timing
// ---------------------------------------------------------------------------

#[test]
fn timingr_is_what_a_transfer_costs() {
    // §39.4.5: `t_SCLL = (SCLL+1) × (PRESC+1) × t_I2CCLK`, and `bus::i2c` fixes
    // the half-period count of every bus event — so a START, three bytes and a
    // STOP cost exactly `(4 + 3×18 + 2) / 2` bit periods.
    let mut board = Board::new(Link::Wired);
    board.init();
    let start = board.ctrl.ticks();
    board.write_page(0x50, 0x00, &[0xaa]);
    let spent = board.ctrl.ticks() - start;
    let halves = u64::from(START_HALF_PERIODS + 3 * BYTE_HALF_PERIODS + STOP_HALF_PERIODS);
    assert_eq!(
        spent,
        halves * HALF_PERIOD,
        "a transfer costs the bit periods TIMINGR asks for"
    );
}

#[test]
fn the_prescaler_scales_both_half_periods() {
    // The same transfer, with `PRESC` one step higher, must cost exactly twice
    // as much — which is the claim that `PRESC` multiplies rather than offsets.
    let mut cost = Vec::new();
    for presc in [1_u32, 3] {
        let mut board = Board::new(Link::Wired);
        poke(&board.region, TIMINGR, (presc << 28) | (1 << 8) | 1);
        poke(&board.region, CR1, CR1_PE);
        let start = board.ctrl.ticks();
        board.write_page(0x50, 0x00, &[0xaa]);
        cost.push(board.ctrl.ticks() - start);
    }
    assert_eq!(cost[1], cost[0] * 2, "PRESC + 1 is a multiplier");
}

// ---------------------------------------------------------------------------
// The debug rule
// ---------------------------------------------------------------------------

#[test]
fn a_debug_dump_of_the_whole_block_pops_nothing() {
    // `ROADMAP.md` §15, invariant 5. v2 has no `SR1`-then-`SR2` trap, but
    // `RXDR` still pops and the stretch it releases still moves the transfer
    // on, so a debugger reading the file must be invisible to the guest.
    let mut board = Board::new(Link::Transactional);
    board.init();
    board.write_page(0x50, 0x00, &[0x33, 0x44]);
    board.step(DEFAULT_EEPROM_WRITE);

    poke(&board.region, CR2, sadd(0x50) | nb(1) | CR2_START);
    board.wait(ISR_TXIS);
    poke(&board.region, TXDR, 0x00);
    board.wait(ISR_TC);
    poke(
        &board.region,
        CR2,
        sadd(0x50) | CR2_RD_WRN | nb(2) | CR2_AUTOEND | CR2_START,
    );
    board.wait(ISR_RXNE);

    // gdb dumps the register file. Twice, for good measure.
    for _ in 0..2 {
        for offset in EVERY_REGISTER {
            let _ = peek_with(&board.region, offset, MemAttrs::DEBUG);
        }
    }
    assert_ne!(
        board.isr() & ISR_RXNE,
        0,
        "a debug read of RXDR must not clear RXNE"
    );
    assert!(board.ctrl.stretching(), "and the transfer is still stalled");

    // And the guest still gets both bytes, in order.
    let mut out = Vec::new();
    out.push(peek(&board.region, RXDR) as u8);
    board.wait(ISR_RXNE);
    out.push(peek(&board.region, RXDR) as u8);
    assert_eq!(out, alloc::vec![0x33, 0x44]);
    board.wait(ISR_STOPF);
}

// ---------------------------------------------------------------------------
// Ten-bit addressing
// ---------------------------------------------------------------------------

/// A slave that records what the bus did to it, so a ten-bit sequence can be
/// checked as *bytes on the wire* rather than as flags.
#[derive(Debug)]
struct Recorder {
    address: u16,
    log: Mutex<Vec<(Address, Direction)>>,
}

impl Recorder {
    fn new(address: u16) -> Arc<Recorder> {
        Arc::new(Recorder {
            address,
            log: Mutex::new(Vec::new()),
        })
    }
}

impl crate::bus::i2c::I2cSlave for Recorder {
    fn address(&self, address: Address, dir: Direction) -> Ack {
        if address == Address::Ten(self.address) {
            self.log.lock().push((address, dir));
            Ack::Ack
        } else {
            Ack::Nack
        }
    }

    fn ten_bit_header(&self, high: u8) -> bool {
        u16::from(high) == self.address >> 8
    }

    fn write(&self, _byte: u8) -> Ack {
        Ack::Ack
    }

    fn read(&self) -> u8 {
        0x5a
    }

    fn stop(&self) {}
}

#[test]
fn a_ten_bit_read_sends_the_header_twice_unless_head10r_says_otherwise() {
    // UM10204 §3.1.11 and §39.7.2's `HEAD10R`. With `HEAD10R` clear the master
    // sends the whole write address, a repeated START and the read header; with
    // it set it sends the read header alone, which only works because the part
    // was already addressed.
    for head10r in [false, true] {
        let bus = Arc::new(I2cBus::new());
        let slave = Recorder::new(0x123);
        bus.attach(Arc::clone(&slave) as Arc<dyn crate::bus::i2c::I2cSlave>)
            .unwrap();
        let ctrl = Stm32I2cV2::with_bus(Link::Transactional, Some(Arc::clone(&bus))).unwrap();
        let r = regs(&ctrl);
        poke(&r, TIMINGR, TIMINGR_USED);
        poke(&r, CR1, CR1_PE);
        let mut cr2 = 0x123 | CR2_ADD10 | CR2_RD_WRN | nb(1) | CR2_AUTOEND | CR2_START;
        if head10r {
            cr2 |= CR2_HEAD10R;
        }
        poke(&r, CR2, cr2);
        let mut now = 0;
        for _ in 0..4_000 {
            if peek_with(&r, ISR, MemAttrs::DEBUG) & ISR_STOPF != 0 {
                break;
            }
            now += 1;
            ctrl.advance_to(now);
        }
        assert_ne!(
            peek_with(&r, ISR, MemAttrs::DEBUG) & ISR_STOPF,
            0,
            "the ten-bit transfer never finished (HEAD10R = {head10r})"
        );
        assert_eq!(
            peek_with(&r, RXDR, MemAttrs::DEBUG),
            0x5a,
            "and it did not read the slave's byte"
        );
        // The addressing sequence is the whole claim, and it is checked at the
        // slave rather than at the flags. §39.7.2's `HEAD10R` = 1 sends "only
        // the 1st 7 bits of the 10-bit address, followed by Read direction", so
        // the part is addressed once, for a read; with it clear the master
        // sends the complete write address, then a repeated START and the read
        // header, and the slave is addressed twice.
        let addressed = slave.log.lock().clone();
        if head10r {
            assert_eq!(
                addressed,
                alloc::vec![(Address::Ten(0x123), Direction::Read)],
                "HEAD10R addresses the part once, for a read"
            );
        } else {
            assert_eq!(
                addressed,
                alloc::vec![
                    (Address::Ten(0x123), Direction::Write),
                    (Address::Ten(0x123), Direction::Read)
                ],
                "the full sequence is address, repeated START, read header"
            );
        }
    }
}

// ---------------------------------------------------------------------------
// Slave mode
// ---------------------------------------------------------------------------

/// A controller on a bus of its own, with nothing else on it: the slave tests
/// play the part of the master by calling [`I2cBus`] directly, which is exactly
/// what a transactional master does.
fn slave_on_a_bus() -> (Stm32I2cV2, RegionRef, Arc<I2cBus>) {
    let bus = Arc::new(I2cBus::new());
    let ctrl = Stm32I2cV2::with_bus(Link::Transactional, Some(Arc::clone(&bus))).unwrap();
    let region = regs(&ctrl);
    poke(&region, TIMINGR, TIMINGR_USED);
    poke(&region, CR1, CR1_PE);
    (ctrl, region, bus)
}

#[test]
fn slave_mode_matches_oar1_and_reports_dir_and_addcode() {
    let (ctrl, r, bus) = slave_on_a_bus();
    poke(&r, OAR1, OAR1_OA1EN | sadd(0x42));

    assert_eq!(bus.start(Address::Seven(0x41), Direction::Write), Ack::Nack);
    assert_eq!(peek(&r, ISR) & ISR_ADDR, 0, "a near miss is a miss");

    assert_eq!(bus.start(Address::Seven(0x42), Direction::Write), Ack::Ack);
    let isr = peek(&r, ISR);
    assert_ne!(isr & ISR_ADDR, 0, "§39.4.9: ADDR on an address match");
    assert_eq!(isr & ISR_DIR, 0, "DIR = 0 is a write, the slave receives");
    assert_eq!(
        (isr & ISR_ADDCODE_MASK) >> ISR_ADDCODE_SHIFT,
        0x42,
        "ADDCODE carries the address that matched"
    );
    assert!(ctrl.stretching(), "ADDR stretches until ADDRCF");

    poke(&r, ICR, ISR_ADDR);
    assert_eq!(peek(&r, ISR) & ISR_ADDR, 0, "ADDRCF clears it");
    assert_eq!(bus.write(0xa5), Ack::Ack);
    assert_ne!(peek(&r, ISR) & ISR_RXNE, 0);
    assert_eq!(peek(&r, RXDR), 0xa5);

    bus.stop();
    assert_ne!(
        peek(&r, ISR) & ISR_STOPF,
        0,
        "a STOP is seen as a slave too"
    );
}

#[test]
fn a_slave_read_reports_dir_and_hands_over_txdr() {
    let (_ctrl, r, bus) = slave_on_a_bus();
    poke(&r, OAR1, OAR1_OA1EN | sadd(0x42));
    assert_eq!(bus.start(Address::Seven(0x42), Direction::Read), Ack::Ack);
    let isr = peek(&r, ISR);
    assert_ne!(isr & ISR_DIR, 0, "DIR = 1 is a read, the slave transmits");
    assert_ne!(isr & ISR_TXIS, 0, "and the hardware asks for a byte");
    poke(&r, ICR, ISR_ADDR);
    poke(&r, TXDR, 0x99);
    assert_eq!(peek(&r, ISR) & ISR_TXE, 0, "TXDR is loaded");
    assert_eq!(bus.read(Ack::Ack), 0x99);
    assert_ne!(peek(&r, ISR) & ISR_TXIS, 0, "an ACK asks for another");
    bus.stop();
}

#[test]
fn the_second_own_address_answers_a_masked_range() {
    // §39.7.4's `OA2MSK`: the mask hides the *low* bits of `OA2[7:1]`, so a
    // mask of 3 answers eight consecutive addresses.
    let (_ctrl, r, bus) = slave_on_a_bus();
    poke(&r, OAR2, OAR2_OA2EN | sadd(0x40) | (3 << 8));
    for address in 0x40..=0x47_u8 {
        assert_eq!(
            bus.start(Address::Seven(address), Direction::Write),
            Ack::Ack,
            "{address:#04x} is inside the masked range"
        );
        bus.stop();
    }
    assert_eq!(bus.start(Address::Seven(0x48), Direction::Write), Ack::Nack);
    bus.stop();

    // And `OA2MSK = 111` answers everything except the reserved addresses.
    poke(&r, OAR2, 0);
    poke(&r, OAR2, OAR2_OA2EN | sadd(0x00) | (7 << 8));
    assert_eq!(bus.start(Address::Seven(0x33), Direction::Write), Ack::Ack);
    bus.stop();
    assert_eq!(
        bus.start(Address::Seven(0x01), Direction::Write),
        Ack::Nack,
        "§39.7.4: all addresses are acknowledged except those reserved"
    );
    bus.stop();
}

#[test]
fn the_general_call_needs_gcen() {
    let (_ctrl, r, bus) = slave_on_a_bus();
    poke(&r, OAR1, OAR1_OA1EN | sadd(0x42));
    assert_eq!(bus.start(GENERAL_CALL, Direction::Write), Ack::Nack);
    poke(&r, CR1, CR1_PE | CR1_GCEN);
    assert_eq!(bus.start(GENERAL_CALL, Direction::Write), Ack::Ack);
    assert_eq!(
        (peek(&r, ISR) & ISR_ADDCODE_MASK) >> ISR_ADDCODE_SHIFT,
        0,
        "the general call's address code is zero"
    );
    bus.stop();
}

#[test]
fn a_slave_nack_is_one_byte_and_clears_itself() {
    // §39.7.2: `NACK` is "cleared by hardware when the NACK is sent", so it
    // refuses exactly one byte.
    let (_ctrl, r, bus) = slave_on_a_bus();
    poke(&r, OAR1, OAR1_OA1EN | sadd(0x42));
    bus.start(Address::Seven(0x42), Direction::Write);
    poke(&r, ICR, ISR_ADDR);
    poke(&r, CR2, CR2_NACK);
    assert_eq!(bus.write(0x01), Ack::Nack);
    assert_eq!(peek(&r, CR2) & CR2_NACK, 0, "the NACK bit clears itself");
    poke(&r, RXDR, 0); // read-only; just proving the write is harmless
    let _ = peek(&r, RXDR);
    assert_eq!(bus.write(0x02), Ack::Ack, "and the next byte is taken");
    bus.stop();
}

#[test]
fn a_nostretch_slave_overruns_instead_of_holding_the_clock() {
    // §39.4.9: with `NOSTRETCH` the slave cannot make the master wait, so a
    // byte that arrives before the last one was read is lost and `OVR` says so.
    let (ctrl, r, bus) = slave_on_a_bus();
    poke(&r, CR1, CR1_PE | CR1_NOSTRETCH);
    poke(&r, OAR1, OAR1_OA1EN | sadd(0x42));
    bus.start(Address::Seven(0x42), Direction::Write);
    assert!(!ctrl.stretching(), "NOSTRETCH means never holding SCL");
    poke(&r, ICR, ISR_ADDR);
    assert_eq!(bus.write(0x01), Ack::Ack);
    assert_eq!(bus.write(0x02), Ack::Ack);
    assert_ne!(peek(&r, ISR) & ISR_OVR, 0, "the second byte overran");
    assert_eq!(peek(&r, RXDR), 0x01, "and the first one survived");
    poke(&r, ICR, ISR_OVR);
    bus.stop();
}

#[test]
fn a_master_never_answers_its_own_address() {
    // Real silicon cannot address itself, and this model's slave face is on the
    // same bus its master drives, so it has to be explicitly deaf while the
    // master half is working.
    let (_ctrl, r, bus) = slave_on_a_bus();
    poke(&r, OAR1, OAR1_OA1EN | sadd(0x42));
    poke(&r, CR2, sadd(0x42) | nb(1) | CR2_AUTOEND | CR2_START);
    // The START has been submitted, so the master half is no longer idle.
    assert_eq!(
        bus.start(Address::Seven(0x42), Direction::Write),
        Ack::Nack,
        "the slave face is deaf while this controller is the master"
    );
    assert_eq!(peek(&r, ISR) & ISR_ADDR, 0);
}

// ---------------------------------------------------------------------------
// SMBus PEC
// ---------------------------------------------------------------------------

#[test]
fn pecbyte_puts_a_real_crc_on_the_wire() {
    // §39.4.13: with `PECEN` and `CR2.PECBYTE`, the last of `NBYTES` is the
    // packet error check the hardware computed over everything from the address
    // byte onwards. The EEPROM is the witness — it stores whatever arrives, so
    // the byte on the wire is checkable rather than merely claimed.
    let mut board = Board::new(Link::Transactional);
    poke(&board.region, TIMINGR, TIMINGR_USED);
    poke(&board.region, CR1, CR1_PE | CR1_PECEN);
    // Word address 0x00, one data byte, then the PEC: three bytes.
    poke(
        &board.region,
        CR2,
        sadd(0x50) | nb(3) | CR2_PECBYTE | CR2_AUTOEND | CR2_START,
    );
    board.wait(ISR_TXIS);
    poke(&board.region, TXDR, 0x00);
    board.wait(ISR_TXIS);
    poke(&board.region, TXDR, 0xab);
    board.wait(ISR_STOPF);
    poke(&board.region, ICR, ISR_STOPF);
    board.step(DEFAULT_EEPROM_WRITE);

    let expected = crc8(crc8(crc8(0, 0xa0), 0x00), 0xab);
    assert_eq!(
        peek(&board.region, PECR) as u8,
        expected,
        "PECR holds the CRC-8 of the address byte and the data"
    );
    assert_eq!(
        board.eeprom.byte(0x01),
        Some(expected),
        "and the hardware, not TXDR, put it on the wire"
    );
    assert_eq!(board.eeprom.byte(0x00), Some(0xab));
}

#[test]
fn a_received_pec_that_disagrees_raises_pecerr() {
    // The receive half of §39.4.13. The EEPROM knows nothing about SMBus, so
    // the byte it hands over as "the PEC" is just data — which is exactly the
    // mismatch this flag exists to report.
    let mut board = Board::new(Link::Transactional);
    poke(&board.region, TIMINGR, TIMINGR_USED);
    poke(&board.region, CR1, CR1_PE);
    board.write_page(0x50, 0x60, &[0x11, 0x22]);
    board.step(DEFAULT_EEPROM_WRITE);

    poke(&board.region, CR1, CR1_PE | CR1_PECEN);
    poke(&board.region, CR2, sadd(0x50) | nb(1) | CR2_START);
    board.wait(ISR_TXIS);
    poke(&board.region, TXDR, 0x60);
    board.wait(ISR_TC);
    poke(
        &board.region,
        CR2,
        sadd(0x50) | CR2_RD_WRN | nb(2) | CR2_PECBYTE | CR2_AUTOEND | CR2_START,
    );
    board.wait(ISR_RXNE);
    assert_eq!(peek(&board.region, RXDR), 0x11, "one data byte");
    board.wait(ISR_STOPF);
    assert_ne!(
        board.isr() & ISR_PECERR,
        0,
        "0x22 is not the CRC-8 of what came before it"
    );
    assert_eq!(
        board.isr() & ISR_RXNE,
        0,
        "and the PEC byte is not delivered to RXDR"
    );
    poke(&board.region, ICR, ISR_PECERR | ISR_STOPF);
    assert_eq!(board.isr() & ISR_PECERR, 0, "PECCF clears it");
}

// ---------------------------------------------------------------------------
// The outputs
// ---------------------------------------------------------------------------

#[test]
fn the_interrupt_and_dma_lines_follow_their_enable_bits() {
    // §39.7.1 lists exactly which flag raises which line, and the DMA requests
    // are the same two flags gated by two other bits.
    let mut board = Board::new(Link::Transactional);
    board.init();
    assert_eq!(board.ctrl.ev_level(), Level::Low);
    poke(&board.region, CR1, CR1_PE | CR1_TXIE | CR1_NACKIE);
    poke(
        &board.region,
        CR2,
        sadd(0x50) | nb(2) | CR2_AUTOEND | CR2_START,
    );
    board.wait(ISR_TXIS);
    assert_eq!(board.ctrl.ev_level(), Level::High, "TXIS raises EV");
    assert_eq!(
        board.ctrl.tx_drq_level(),
        Level::Low,
        "but not the DMA request, which TXDMAEN gates separately"
    );
    poke(
        &board.region,
        CR1,
        CR1_PE | CR1_TXIE | CR1_NACKIE | CR1_TXDMAEN,
    );
    assert_eq!(board.ctrl.tx_drq_level(), Level::High);
    poke(&board.region, TXDR, 0x70);
    assert_eq!(
        board.ctrl.tx_drq_level(),
        Level::Low,
        "and it drops as soon as the register is served"
    );

    // Finish the transfer against an address nobody holds, for the error line.
    board.wait(ISR_TXIS);
    poke(&board.region, TXDR, 0x00);
    board.wait(ISR_STOPF);
    poke(&board.region, ICR, ISR_STOPF);

    poke(&board.region, CR1, CR1_PE | CR1_ERRIE);
    poke(
        &board.region,
        CR2,
        sadd(0x58) | nb(1) | CR2_AUTOEND | CR2_START,
    );
    assert!(board.wait_for(ISR_NACKF));
    assert_eq!(
        board.ctrl.er_level(),
        Level::Low,
        "§39.7.1: NACKF is an *event*, not an error — ERRIE does not carry it"
    );
}

// ---------------------------------------------------------------------------
// Snapshots and reset
// ---------------------------------------------------------------------------

#[test]
fn a_transfer_part_way_through_a_reload_round_trips() {
    // The byte counter is the field a naive snapshot loses: `CR2.NBYTES` says
    // what the *last* reload asked for, not how much of it is left.
    let mut board = Board::new(Link::Wired);
    board.init();
    poke(
        &board.region,
        CR2,
        sadd(0x50) | nb(4) | CR2_AUTOEND | CR2_START,
    );
    board.wait(ISR_TXIS);
    poke(&board.region, TXDR, 0x50);
    board.wait(ISR_TXIS);
    poke(&board.region, TXDR, 0x01);
    board.wait(ISR_TXIS);

    let mut shape = MachineShape::new();
    shape.add_device("i2c", ST_I2C_V2_CLASS.name).unwrap();
    let mut w = StateWriter::new(shape.clone());
    {
        let mut chunk = w
            .chunk("i2c", ST_I2C_V2_CLASS.name, ST_I2C_V2_CLASS.version)
            .unwrap();
        board.ctrl.save(&mut chunk).unwrap();
    }
    let bytes = w.to_vec().unwrap();

    let other = Stm32I2cV2::with_bus(Link::Wired, None).unwrap();
    let reader = StateReader::new(&bytes).unwrap();
    let chunk = reader
        .load(
            "i2c",
            ST_I2C_V2_CLASS.name,
            ST_I2C_V2_CLASS.version,
            &Migrations::new(),
        )
        .unwrap();
    other.load(&mut chunk.reader()).unwrap();

    let mut w2 = StateWriter::new(shape);
    {
        let mut chunk = w2
            .chunk("i2c", ST_I2C_V2_CLASS.name, ST_I2C_V2_CLASS.version)
            .unwrap();
        other.save(&mut chunk).unwrap();
    }
    assert_eq!(w2.to_vec().unwrap(), bytes, "an identical state hash");

    // And the restored copy answers the same through its own registers.
    let other_region = regs(&other);
    for offset in EVERY_REGISTER {
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
    assert_eq!(peek(&board.region, TIMINGR), 0);
    assert_eq!(peek(&board.region, ISR), ISR_TXE);
}
