//! Tests for the I²C fabric, its bit engines, and the two link models.
//!
//! The load-bearing one is
//! [`both_link_models_produce_identical_traffic`](tests::both_link_models_produce_identical_traffic):
//! the whole design claim is that a peripheral is written *once* and works
//! whether the bus hands it a byte or clocks it in one edge at a time, and that
//! is the test that would fail if it were not true.
//!
//! The other three that earn their place are
//! [`a_slave_that_stretches_the_clock_stalls_a_wired_master`](tests::a_slave_that_stretches_the_clock_stalls_a_wired_master),
//! [`two_masters_arbitrate_and_the_loser_lets_go`](tests::two_masters_arbitrate_and_the_loser_lets_go)
//! and
//! [`a_released_line_that_reads_low_is_somebody_else_pulling`](tests::a_released_line_that_reads_low_is_somebody_else_pulling):
//! clock stretching and arbitration are meaningless in a transactional model
//! and fall out of the open-drain nets in a wired one, which is the whole
//! reason [`super::wires`] exists.

use super::wires::{MasterEvent, MasterOp, MasterWires, SlaveWires, pin};
use super::*;

use alloc::vec;
use alloc::vec::Vec;

use crate::core::state::SliceSource;
use crate::core::sync::{AtomicBool, LockRank, Mutex};
use crate::core::wire::{Level, Wire, WireId, WireSource};

// ---------------------------------------------------------------------------
// Mocks
// ---------------------------------------------------------------------------

/// One call the fabric made into a device.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Call {
    Address(Address, Direction),
    Write(u8),
    Read(u8),
    ReadAck(Ack),
    Stop,
}

/// A slave that answers one address, logs every call, and serves bytes from a
/// counter.
///
/// Deliberately not a real device: what is under test is the bus, and a mock
/// that logs is what makes "the same traffic arrived" a checkable claim.
#[derive(Debug)]
struct Recorder {
    address: Address,
    /// Also answer the general call (§3.1.13).
    general: bool,
    log: Mutex<Vec<Call>>,
    /// Bytes handed back on a read, consumed front to back.
    replies: Mutex<Vec<u8>>,
    /// Whether we are currently holding SCL down (§3.1.9).
    stretch: AtomicBool,
    /// Refuse every data byte, to exercise the NACK paths.
    refuse: bool,
}

impl Recorder {
    fn new(address: Address, replies: &[u8]) -> Arc<Recorder> {
        Arc::new(Recorder {
            address,
            general: false,
            log: Mutex::with_rank(LockRank::LEAF, Vec::new()),
            replies: Mutex::with_rank(LockRank::LEAF, replies.to_vec()),
            stretch: AtomicBool::new(false),
            refuse: false,
        })
    }

    fn take_log(&self) -> Vec<Call> {
        core::mem::take(&mut *self.log.lock())
    }
}

impl I2cSlave for Recorder {
    fn address(&self, address: Address, dir: Direction) -> Ack {
        self.log.lock().push(Call::Address(address, dir));
        if address == GENERAL_CALL && self.general {
            return Ack::Ack;
        }
        if address == self.address {
            Ack::Ack
        } else {
            Ack::Nack
        }
    }

    fn ten_bit_header(&self, high: u8) -> bool {
        self.address.ten_bit_high() == Some(high)
    }

    fn write(&self, byte: u8) -> Ack {
        self.log.lock().push(Call::Write(byte));
        if self.refuse { Ack::Nack } else { Ack::Ack }
    }

    fn read(&self) -> u8 {
        let byte = self.replies.lock().first().copied().unwrap_or(0xff);
        self.log.lock().push(Call::Read(byte));
        byte
    }

    fn read_ack(&self, ack: Ack) {
        self.log.lock().push(Call::ReadAck(ack));
        if ack.is_ack() {
            let mut replies = self.replies.lock();
            if !replies.is_empty() {
                replies.remove(0);
            }
        }
    }

    fn stop(&self) {
        self.log.lock().push(Call::Stop);
    }

    fn stretching(&self) -> bool {
        self.stretch.load(crate::core::sync::Ordering::Relaxed)
    }

    fn peek(&self) -> u8 {
        self.replies.lock().first().copied().unwrap_or(0xff)
    }
}

// ---------------------------------------------------------------------------
// A wired harness
// ---------------------------------------------------------------------------

/// Two open-drain nets with one master and any number of slaves on each — what
/// a machine file's four `wire` statements build.
struct Harness {
    masters: Vec<Arc<MasterWires>>,
    slaves: Vec<Arc<SlaveWires>>,
    #[allow(dead_code)]
    scl: Arc<Wire>,
    #[allow(dead_code)]
    sda: Arc<Wire>,
}

impl Harness {
    fn new(masters: usize, slaves: &[Arc<dyn I2cSlave>]) -> Harness {
        let masters: Vec<Arc<MasterWires>> =
            (0..masters).map(|_| Arc::new(MasterWires::new())).collect();
        let slaves: Vec<Arc<SlaveWires>> = slaves
            .iter()
            .map(|s| Arc::new(SlaveWires::new(Arc::clone(s))))
            .collect();

        // Two ids per participant, allocated as the machine resolver would: in
        // a fixed order, so a run is reproducible.
        let total = masters.len() + slaves.len();
        let scl_ids: Vec<WireId> = (0..total).map(|i| WireId::new(1 + i as u64)).collect();
        let sda_ids: Vec<WireId> = (0..total)
            .map(|i| WireId::new(1 + (total + i) as u64))
            .collect();

        let mut scl = Wire::builder().sources(&scl_ids);
        let mut sda = Wire::builder().sources(&sda_ids);
        for m in &masters {
            scl = scl.sink(m.sink(pin::SCL, &scl_ids), pin::SCL);
            sda = sda.sink(m.sink(pin::SDA, &sda_ids), pin::SDA);
        }
        for s in &slaves {
            scl = scl.sink(s.sink(pin::SCL, &scl_ids), pin::SCL);
            sda = sda.sink(s.sink(pin::SDA, &sda_ids), pin::SDA);
        }
        let scl = scl.build_shared();
        let sda = sda.build_shared();

        for (i, m) in masters.iter().enumerate() {
            m.connect(pin::SCL, WireSource::new(Arc::clone(&scl), scl_ids[i]));
            m.connect(pin::SDA, WireSource::new(Arc::clone(&sda), sda_ids[i]));
        }
        for (i, s) in slaves.iter().enumerate() {
            let at = masters.len() + i;
            s.connect(pin::SCL, WireSource::new(Arc::clone(&scl), scl_ids[at]));
            s.connect(pin::SDA, WireSource::new(Arc::clone(&sda), sda_ids[at]));
        }
        // The realize sweep (§4.3): every driver announces, so the fan-ins agree
        // that both lines are released before anything moves.
        for m in &masters {
            m.announce();
        }
        for s in &slaves {
            s.announce();
        }
        Harness {
            masters,
            slaves,
            scl,
            sda,
        }
    }

    fn master(&self) -> &Arc<MasterWires> {
        &self.masters[0]
    }

    /// Run one bus event to completion, reporting how many half periods it
    /// spent stretched.
    fn run(&self, op: MasterOp) -> (MasterEvent, u32) {
        assert!(self.master().submit(op), "the engine was already busy");
        let mut stretched = 0;
        for _ in 0..256 {
            match self.master().tick() {
                MasterEvent::Working => {}
                MasterEvent::Stretched => stretched += 1,
                other => return (other, stretched),
            }
        }
        panic!("{op:?} never finished");
    }
}

/// Run a script against a fresh wired harness and hand back the slave's log.
fn wired_script(slave: Arc<Recorder>, script: &[MasterOp]) -> (Vec<Call>, Vec<u8>) {
    let harness = Harness::new(1, &[Arc::clone(&slave) as Arc<dyn I2cSlave>]);
    let mut got = Vec::new();
    for op in script {
        let (event, _) = harness.run(*op);
        if let MasterEvent::Read(byte) = event {
            got.push(byte);
        }
    }
    (slave.take_log(), got)
}

/// The same script through the transactional fabric.
fn transactional_script(slave: Arc<Recorder>, script: &[MasterOp]) -> (Vec<Call>, Vec<u8>) {
    let bus = I2cBus::new();
    bus.attach(Arc::clone(&slave) as Arc<dyn I2cSlave>)
        .expect("room on the bus");
    let mut got = Vec::new();
    // A transactional master has no START of its own: the START and the address
    // byte are one call, so the script's `Start` is remembered and applied to
    // the byte that follows it. That is the *only* place the two links differ,
    // and it differs in the master, not in what the slave sees.
    let mut pending_start = false;
    for op in script {
        match *op {
            MasterOp::Start => pending_start = true,
            MasterOp::Write(byte) if pending_start => {
                pending_start = false;
                if Address::is_ten_bit_header(byte) {
                    let high = (byte >> 1) & 0b11;
                    match Direction::from_bit(byte) {
                        Direction::Write => {
                            let _ = bus.ten_bit_header(high);
                            // The second byte completes the address; remembered
                            // here the way a real controller remembers it.
                            pending_start = true;
                        }
                        Direction::Read => {
                            // Only reachable with a ten-bit address already
                            // matched, which this helper's scripts do not use.
                            bus.write(byte);
                        }
                    }
                } else {
                    bus.start(Address::seven_from_byte(byte), Direction::from_bit(byte));
                }
            }
            MasterOp::Write(byte) => {
                bus.write(byte);
            }
            MasterOp::Read(ack) => got.push(bus.read(ack)),
            MasterOp::Stop => bus.stop(),
        }
    }
    (slave.take_log(), got)
}

// ---------------------------------------------------------------------------
// Framing
// ---------------------------------------------------------------------------

#[test]
fn an_address_byte_is_the_address_shifted_up_and_the_direction_bit() {
    // §3.1.10: seven bits, then R/W.
    assert_eq!(
        Address::Seven(0x50).first_byte(Direction::Write),
        0b1010_0000
    );
    assert_eq!(
        Address::Seven(0x50).first_byte(Direction::Read),
        0b1010_0001
    );
    assert_eq!(Address::seven_from_byte(0b1010_0001), Address::Seven(0x50));
    assert_eq!(Direction::from_bit(0b1010_0001), Direction::Read);
    assert_eq!(Address::Seven(0x50).second_byte(), None);
}

#[test]
fn a_ten_bit_address_is_a_reserved_header_and_a_whole_byte() {
    // §3.1.11: "1111 0XX of which the last two bits (XX) are the two
    // Most-Significant Bits (MSB) of the 10-bit address".
    let a = Address::Ten(0x2a5);
    assert_eq!(a.first_byte(Direction::Write), 0b1111_0100);
    assert_eq!(a.first_byte(Direction::Read), 0b1111_0101);
    assert_eq!(a.second_byte(), Some(0xa5));
    assert_eq!(a.ten_bit_high(), Some(0b10));
    assert!(Address::is_ten_bit_header(0b1111_0100));
    assert!(!Address::is_ten_bit_header(0b1111_1000), "1111 1XX is not");
    assert!(!Address::is_ten_bit_header(0b1010_0000));
}

#[test]
fn the_two_reserved_groups_are_the_ones_table_4_lists() {
    // §3.1.12: "Two groups of eight addresses (0000 XXX and 1111 XXX)".
    for a in 0..8u8 {
        assert!(Address::Seven(a).is_reserved(), "{a:#04x}");
        assert!(Address::Seven(0x78 | a).is_reserved(), "{:#04x}", 0x78 | a);
    }
    assert!(!Address::Seven(0x50).is_reserved());
    assert!(!Address::Seven(0x08).is_reserved());
    assert!(!Address::Seven(0x77).is_reserved());
    assert_eq!(GENERAL_CALL, Address::Seven(0));
    assert!(GENERAL_CALL.is_reserved());
}

#[test]
fn an_acknowledge_is_active_low_and_merges_the_way_the_wire_does() {
    assert_eq!(Ack::from_level(Level::Low), Ack::Ack);
    assert_eq!(Ack::from_level(Level::High), Ack::Nack);
    assert_eq!(Ack::Ack.level(), Level::Low);
    // §3.1.13: "if one or more targets acknowledge, the not-acknowledge will
    // not be seen by the controller."
    assert_eq!(Ack::Nack.merge(Ack::Ack), Ack::Ack);
    assert_eq!(Ack::Ack.merge(Ack::Nack), Ack::Ack);
    assert_eq!(Ack::Nack.merge(Ack::Nack), Ack::Nack);
}

#[test]
fn a_link_round_trips_through_the_name_a_machine_file_writes() {
    for name in Link::NAMES {
        let link = Link::from_name(name).expect("a known spelling");
        assert_eq!(link.name(), *name);
    }
    assert_eq!(Link::from_name("wired-ish"), None);
}

// ---------------------------------------------------------------------------
// The transactional fabric
// ---------------------------------------------------------------------------

#[test]
fn only_the_addressed_device_hears_the_bytes() {
    let a = Recorder::new(Address::Seven(0x50), &[]);
    let b = Recorder::new(Address::Seven(0x51), &[]);
    let bus = I2cBus::new();
    bus.attach(Arc::clone(&a) as Arc<dyn I2cSlave>).unwrap();
    bus.attach(Arc::clone(&b) as Arc<dyn I2cSlave>).unwrap();

    assert_eq!(bus.start(Address::Seven(0x50), Direction::Write), Ack::Ack);
    assert_eq!(bus.write(0x11), Ack::Ack);
    bus.stop();

    assert_eq!(
        a.take_log(),
        vec![
            Call::Address(Address::Seven(0x50), Direction::Write),
            Call::Write(0x11),
            Call::Stop,
        ]
    );
    // The other device saw the address — every device does, that is how I²C
    // selects — and nothing after it.
    assert_eq!(
        b.take_log(),
        vec![Call::Address(Address::Seven(0x50), Direction::Write)]
    );
}

#[test]
fn a_bus_with_nothing_on_the_address_nacks_and_reads_as_the_pull_up() {
    let bus = I2cBus::new();
    let a = Recorder::new(Address::Seven(0x50), &[]);
    bus.attach(a as Arc<dyn I2cSlave>).unwrap();

    // §3.1.6, reason 1: "No receiver is present on the bus with the transmitted
    // address so there is no device to respond with an acknowledge."
    assert_eq!(bus.start(Address::Seven(0x22), Direction::Write), Ack::Nack);
    assert_eq!(bus.state(), BusState::Unaddressed);
    assert!(bus.state().is_busy(), "the bus is busy from the START");
    assert_eq!(bus.write(0xaa), Ack::Nack);
    assert_eq!(bus.read(Ack::Nack), 0xff, "an undriven SDA is the pull-up");
    bus.stop();
    assert_eq!(bus.state(), BusState::Free);
}

#[test]
fn the_general_call_reaches_every_device_that_answers_it() {
    let mut listener = Recorder::new(Address::Seven(0x50), &[]);
    Arc::get_mut(&mut listener).unwrap().general = true;
    let deaf = Recorder::new(Address::Seven(0x51), &[]);
    let bus = I2cBus::new();
    bus.attach(Arc::clone(&listener) as Arc<dyn I2cSlave>)
        .unwrap();
    bus.attach(Arc::clone(&deaf) as Arc<dyn I2cSlave>).unwrap();

    // §3.1.13: "if a device does not need any of the data supplied within the
    // general call structure, it can ignore this address by not issuing an
    // acknowledgment."
    assert_eq!(bus.start(GENERAL_CALL, Direction::Write), Ack::Ack);
    assert_eq!(bus.write(0x06), Ack::Ack, "one acknowledge is enough");
    bus.stop();

    assert_eq!(
        listener.take_log(),
        vec![
            Call::Address(GENERAL_CALL, Direction::Write),
            Call::Write(0x06),
            Call::Stop,
        ]
    );
    assert_eq!(
        deaf.take_log(),
        vec![Call::Address(GENERAL_CALL, Direction::Write)],
        "a device that did not answer hears nothing after the address"
    );
    assert_eq!(
        bus.conflicts(),
        0,
        "two general-call answers is not a clash"
    );
}

#[test]
fn two_devices_on_one_address_are_counted_as_the_wiring_error_they_are() {
    let a = Recorder::new(Address::Seven(0x50), &[]);
    let b = Recorder::new(Address::Seven(0x50), &[]);
    let bus = I2cBus::new();
    bus.attach(a as Arc<dyn I2cSlave>).unwrap();
    bus.attach(b as Arc<dyn I2cSlave>).unwrap();
    assert_eq!(bus.start(Address::Seven(0x50), Direction::Write), Ack::Ack);
    // On real copper both would pull SDA low and the master would see an
    // ordinary ACK, which is exactly why nothing else can notice.
    assert_eq!(bus.conflicts(), 1);
}

#[test]
fn a_repeated_start_that_goes_elsewhere_ends_the_first_devices_transaction() {
    let a = Recorder::new(Address::Seven(0x50), &[]);
    let b = Recorder::new(Address::Seven(0x51), &[]);
    let bus = I2cBus::new();
    bus.attach(Arc::clone(&a) as Arc<dyn I2cSlave>).unwrap();
    bus.attach(Arc::clone(&b) as Arc<dyn I2cSlave>).unwrap();

    bus.start(Address::Seven(0x50), Direction::Write);
    bus.write(0x01);
    // §3.1.11: addressed "until it receives a STOP condition (P) or a repeated
    // START condition (Sr) followed by a different target address".
    bus.start(Address::Seven(0x51), Direction::Write);
    bus.write(0x02);
    bus.stop();

    assert_eq!(
        a.take_log(),
        vec![
            Call::Address(Address::Seven(0x50), Direction::Write),
            Call::Write(0x01),
            Call::Address(Address::Seven(0x51), Direction::Write),
            Call::Stop,
        ]
    );
    assert_eq!(
        b.take_log(),
        vec![
            Call::Address(Address::Seven(0x50), Direction::Write),
            Call::Address(Address::Seven(0x51), Direction::Write),
            Call::Write(0x02),
            Call::Stop,
        ]
    );
}

#[test]
fn a_debug_peek_moves_no_address_counter() {
    let slave = Recorder::new(Address::Seven(0x50), &[0x11, 0x22]);
    let bus = I2cBus::new();
    bus.attach(Arc::clone(&slave) as Arc<dyn I2cSlave>).unwrap();
    bus.start(Address::Seven(0x50), Direction::Read);
    slave.take_log();
    assert_eq!(bus.peek(), 0x11);
    assert_eq!(bus.peek(), 0x11, "and again, because nothing moved");
    assert!(
        slave.take_log().is_empty(),
        "peek is not a call into `read`"
    );
    assert_eq!(bus.read(Ack::Ack), 0x11);
    assert_eq!(bus.peek(), 0x22, "the acknowledge is what advanced it");
}

// ---------------------------------------------------------------------------
// The two links agree
// ---------------------------------------------------------------------------

#[test]
fn both_link_models_produce_identical_traffic() {
    // The claim `docs/buses/low-speed.md` asks for and the reason `I2cSlave` is
    // byte-level: a peripheral is written once, and a machine that switches
    // `link` gets the same calls in the same order. If this ever fails, the two
    // models have diverged and one of them is lying.
    let script = [
        MasterOp::Start,
        MasterOp::Write(Address::Seven(0x50).first_byte(Direction::Write)),
        MasterOp::Write(0x04),
        MasterOp::Write(0xde),
        MasterOp::Write(0xad),
        MasterOp::Stop,
    ];
    let (wired, _) = wired_script(Recorder::new(Address::Seven(0x50), &[]), &script);
    let (transactional, _) =
        transactional_script(Recorder::new(Address::Seven(0x50), &[]), &script);
    assert_eq!(wired, transactional);
    assert_eq!(
        wired,
        vec![
            Call::Address(Address::Seven(0x50), Direction::Write),
            Call::Write(0x04),
            Call::Write(0xde),
            Call::Write(0xad),
            Call::Stop,
        ],
        "and both are the traffic that was asked for"
    );
}

#[test]
fn both_link_models_read_the_same_bytes_back() {
    let script = [
        MasterOp::Start,
        MasterOp::Write(Address::Seven(0x50).first_byte(Direction::Read)),
        MasterOp::Read(Ack::Ack),
        MasterOp::Read(Ack::Ack),
        MasterOp::Read(Ack::Nack),
        MasterOp::Stop,
    ];
    let replies = [0x11u8, 0x22, 0x33];
    let (wired, wired_bytes) = wired_script(Recorder::new(Address::Seven(0x50), &replies), &script);
    let (transactional, transactional_bytes) =
        transactional_script(Recorder::new(Address::Seven(0x50), &replies), &script);
    assert_eq!(wired_bytes, vec![0x11, 0x22, 0x33]);
    assert_eq!(wired_bytes, transactional_bytes);
    assert_eq!(wired, transactional);
    // §3.1.6, reason 5: the master's NACK is what ends a sequential read, and
    // the slave must be told about it.
    assert!(wired.contains(&Call::ReadAck(Ack::Nack)));
}

#[test]
fn a_wired_master_hears_a_nack_from_an_address_nobody_answers() {
    let slave = Recorder::new(Address::Seven(0x50), &[]);
    let harness = Harness::new(1, &[Arc::clone(&slave) as Arc<dyn I2cSlave>]);
    assert_eq!(harness.run(MasterOp::Start).0, MasterEvent::Started);
    let byte = Address::Seven(0x22).first_byte(Direction::Write);
    assert_eq!(
        harness.run(MasterOp::Write(byte)).0,
        MasterEvent::Wrote(Ack::Nack)
    );
    assert_eq!(harness.run(MasterOp::Stop).0, MasterEvent::Stopped);
    assert_eq!(
        slave.take_log(),
        vec![Call::Address(Address::Seven(0x22), Direction::Write)],
        "it compared the address and said nothing more"
    );
}

#[test]
fn a_ten_bit_address_takes_two_bytes_and_a_repeated_start_to_read() {
    // §3.1.11's second worked example: header, second byte, repeated START,
    // header with R/W set.
    let addr = Address::Ten(0x2a5);
    let slave = Recorder::new(addr, &[0x5a]);
    let harness = Harness::new(1, &[Arc::clone(&slave) as Arc<dyn I2cSlave>]);

    assert_eq!(harness.run(MasterOp::Start).0, MasterEvent::Started);
    assert_eq!(
        harness
            .run(MasterOp::Write(addr.first_byte(Direction::Write)))
            .0,
        MasterEvent::Wrote(Ack::Ack),
        "the header alone is acknowledged (A1)"
    );
    assert_eq!(
        harness.run(MasterOp::Write(addr.second_byte().unwrap())).0,
        MasterEvent::Wrote(Ack::Ack),
        "and so is the second byte (A2)"
    );
    assert_eq!(harness.run(MasterOp::Start).0, MasterEvent::Started);
    assert_eq!(
        harness
            .run(MasterOp::Write(addr.first_byte(Direction::Read)))
            .0,
        MasterEvent::Wrote(Ack::Ack),
        "the device remembered that it was addressed before"
    );
    assert_eq!(
        harness.run(MasterOp::Read(Ack::Nack)).0,
        MasterEvent::Read(0x5a)
    );
    assert_eq!(harness.run(MasterOp::Stop).0, MasterEvent::Stopped);

    let log = slave.take_log();
    assert_eq!(log[0], Call::Address(addr, Direction::Write));
    assert_eq!(log[1], Call::Address(addr, Direction::Read));
    assert!(log.contains(&Call::Read(0x5a)));
}

// ---------------------------------------------------------------------------
// The things only the wired model can express
// ---------------------------------------------------------------------------

#[test]
fn a_slave_that_stretches_the_clock_stalls_a_wired_master() {
    // §3.1.9: "Clock stretching pauses a transaction by holding the SCL line
    // LOW. The transaction cannot continue until the line is released HIGH
    // again." This is the assertion the whole `wires` module exists for: a
    // transactional model cannot express it as a *level*, and the master here
    // makes no progress because it looks at the net rather than at a flag.
    let slave = Recorder::new(Address::Seven(0x50), &[]);
    let harness = Harness::new(1, &[Arc::clone(&slave) as Arc<dyn I2cSlave>]);
    assert_eq!(harness.run(MasterOp::Start).0, MasterEvent::Started);

    // The device decides it needs time before the next byte.
    slave
        .stretch
        .store(true, crate::core::sync::Ordering::Relaxed);
    let byte = Address::Seven(0x50).first_byte(Direction::Write);
    assert!(harness.master().submit(MasterOp::Write(byte)));

    // Clock the address byte in. On the falling edge that ends its acknowledge
    // slot the slave pulls SCL down, and from there the master goes nowhere.
    let mut spins = 0;
    let mut finished = None;
    for _ in 0..64 {
        match harness.master().tick() {
            MasterEvent::Working => {}
            MasterEvent::Stretched => spins += 1,
            other => {
                finished = Some(other);
                break;
            }
        }
    }
    assert_eq!(
        finished,
        Some(MasterEvent::Wrote(Ack::Ack)),
        "the byte itself still completed"
    );
    assert_eq!(spins, 0, "and nothing was stretched during it");
    assert_eq!(
        harness.slaves[0].scl().driving(),
        Level::Low,
        "the slave is holding the clock down"
    );

    // The next byte cannot start. Its first half period is the *low* one, which
    // the master spends putting the first bit on SDA and letting go of SCL —
    // that half really did elapse. Every one after it is stretched, because SCL
    // never gets up.
    assert!(harness.master().submit(MasterOp::Write(0x42)));
    assert_eq!(harness.master().tick(), MasterEvent::Working);
    for _ in 0..32 {
        assert_eq!(
            harness.master().tick(),
            MasterEvent::Stretched,
            "the master must make no progress while SCL is held"
        );
    }
    assert!(
        slave.take_log().iter().all(|c| *c != Call::Write(0x42)),
        "and the held byte never arrived"
    );

    // The device finishes whatever it was doing and lets go, which on a real
    // part happens on its own clock domain — here, explicitly.
    slave
        .stretch
        .store(false, crate::core::sync::Ordering::Relaxed);
    harness.slaves[0].refresh_stretch();
    let mut event = None;
    for _ in 0..64 {
        match harness.master().tick() {
            MasterEvent::Working | MasterEvent::Stretched => {}
            other => {
                event = Some(other);
                break;
            }
        }
    }
    assert_eq!(event, Some(MasterEvent::Wrote(Ack::Ack)));
    assert!(
        slave.take_log().contains(&Call::Write(0x42)),
        "and now the byte got through"
    );
}

#[test]
fn a_released_line_that_reads_low_is_somebody_else_pulling() {
    // The wired-AND, checked directly: two masters on one SDA net, one pulling.
    let harness = Harness::new(2, &[]);
    assert_eq!(harness.masters[0].sda().net(), Level::High, "the pull-up");
    harness.masters[1].sda().drive(Level::Low);
    assert_eq!(
        harness.masters[0].sda().net(),
        Level::Low,
        "one low driver takes the whole net down"
    );
    assert_eq!(
        harness.masters[0].sda().driving(),
        Level::High,
        "while we are still releasing it — which is what arbitration reads"
    );
    harness.masters[1].sda().drive(Level::High);
    assert_eq!(harness.masters[0].sda().net(), Level::High);
}

#[test]
fn two_masters_arbitrate_and_the_loser_lets_go() {
    // §3.1.8: "The first time a controller tries to send a HIGH, but detects
    // that the SDA level is LOW, the controller knows that it has lost the
    // arbitration and turns off its SDA output driver."
    let slave = Recorder::new(Address::Seven(0x40), &[]);
    let harness = Harness::new(2, &[Arc::clone(&slave) as Arc<dyn I2cSlave>]);
    let (a, b) = (&harness.masters[0], &harness.masters[1]);

    // Both start at the same instant — §3.1.8's "within the minimum hold time".
    assert!(a.submit(MasterOp::Start));
    assert!(b.submit(MasterOp::Start));
    for _ in 0..8 {
        a.tick();
        b.tick();
    }
    assert!(!a.is_working() && !b.is_working(), "both saw a valid START");

    // Now they address different devices. 0x40 is 100 0000; 0x50 is 101 0000,
    // so they agree for two bits and then A sends a zero where B sends a one.
    assert!(a.submit(MasterOp::Write(
        Address::Seven(0x40).first_byte(Direction::Write)
    )));
    assert!(b.submit(MasterOp::Write(
        Address::Seven(0x50).first_byte(Direction::Write)
    )));
    let mut b_lost = false;
    let mut a_result = None;
    for _ in 0..64 {
        if let MasterEvent::ArbitrationLost = b.tick() {
            b_lost = true;
        }
        match a.tick() {
            MasterEvent::Working | MasterEvent::Stretched => {}
            other => {
                a_result = Some(other);
                break;
            }
        }
    }
    assert!(b_lost, "the master sending the higher address must lose");
    assert_eq!(
        a_result,
        Some(MasterEvent::Wrote(Ack::Ack)),
        "and the winner's transfer is untouched — §3.1.8: no information is lost"
    );
    assert_eq!(
        b.sda().driving(),
        Level::High,
        "the loser turned off its SDA driver"
    );
    assert_eq!(
        slave.take_log(),
        vec![Call::Address(Address::Seven(0x40), Direction::Write)],
        "and the device the winner addressed is the one that answered"
    );
}

#[test]
fn a_wired_master_holds_the_clock_down_between_operations() {
    // The other half of §3.1.9, and the reason an STM32 waiting for its driver
    // to write a register really does stall the bus: nothing releases SCL until
    // the next operation starts.
    let harness = Harness::new(1, &[]);
    assert_eq!(harness.run(MasterOp::Start).0, MasterEvent::Started);
    assert_eq!(
        harness.master().scl().driving(),
        Level::Low,
        "a START ends with the clock held down"
    );
    assert_eq!(harness.master().scl().net(), Level::Low);
    // And a STOP is the one thing that lets both lines go.
    assert_eq!(harness.run(MasterOp::Stop).0, MasterEvent::Stopped);
    assert_eq!(harness.master().scl().driving(), Level::High);
    assert_eq!(harness.master().sda().driving(), Level::High);
    assert!(!harness.master().busy(), "the bus is free after a STOP");
}

#[test]
fn a_wired_transfer_costs_the_half_periods_the_fabric_charges_for_it() {
    // The other half of "a transfer costs the same virtual time either way":
    // the wired engine really does take exactly the number of half periods
    // `bus::i2c` charges a transactional controller for.
    let harness = Harness::new(1, &[]);
    let mut ticks = 0u32;
    assert!(harness.master().submit(MasterOp::Start));
    while harness.master().is_working() {
        harness.master().tick();
        ticks += 1;
    }
    assert_eq!(ticks, START_HALF_PERIODS);

    ticks = 0;
    assert!(harness.master().submit(MasterOp::Write(0xa5)));
    while harness.master().is_working() {
        harness.master().tick();
        ticks += 1;
    }
    assert_eq!(ticks, BYTE_HALF_PERIODS);

    ticks = 0;
    assert!(harness.master().submit(MasterOp::Stop));
    while harness.master().is_working() {
        harness.master().tick();
        ticks += 1;
    }
    assert_eq!(ticks, STOP_HALF_PERIODS);
}

#[test]
fn the_acknowledge_a_read_will_drive_can_change_until_the_eighth_bit() {
    // What a driver that clears `ACK` after the second-last byte relies on.
    let slave = Recorder::new(Address::Seven(0x50), &[0x77]);
    let harness = Harness::new(1, &[Arc::clone(&slave) as Arc<dyn I2cSlave>]);
    harness.run(MasterOp::Start);
    harness.run(MasterOp::Write(
        Address::Seven(0x50).first_byte(Direction::Read),
    ));
    slave.take_log();

    assert!(harness.master().submit(MasterOp::Read(Ack::Ack)));
    // Four half periods in — two bits — the decision is still ours to change.
    for _ in 0..4 {
        harness.master().tick();
    }
    assert!(harness.master().set_read_ack(Ack::Nack));
    let mut event = None;
    for _ in 0..64 {
        match harness.master().tick() {
            MasterEvent::Working | MasterEvent::Stretched => {}
            other => {
                event = Some(other);
                break;
            }
        }
    }
    assert_eq!(event, Some(MasterEvent::Read(0x77)));
    assert!(
        slave.take_log().contains(&Call::ReadAck(Ack::Nack)),
        "the slave saw the acknowledge software chose late"
    );
}

// ---------------------------------------------------------------------------
// Snapshots
// ---------------------------------------------------------------------------

#[test]
fn a_bit_engines_state_round_trips_through_its_own_codec() {
    let slave = Recorder::new(Address::Seven(0x50), &[]);
    let harness = Harness::new(1, &[Arc::clone(&slave) as Arc<dyn I2cSlave>]);
    harness.run(MasterOp::Start);
    // Stop half way through the address byte, which is the state a snapshot has
    // to be able to resume from.
    assert!(harness.master().submit(MasterOp::Write(0xa0)));
    for _ in 0..7 {
        harness.master().tick();
    }

    let m = harness.master().snapshot();
    let mut bytes = Vec::new();
    m.write(&mut bytes).expect("it encodes");
    let mut src = SliceSource::new(&bytes);
    let back = super::wires::MasterWiresState::read(&mut src).expect("it decodes");
    assert_eq!(m, back);

    let s = harness.slaves[0].snapshot();
    let mut bytes = Vec::new();
    s.write(&mut bytes).expect("it encodes");
    let mut src = SliceSource::new(&bytes);
    let back = super::wires::SlaveWiresState::read(&mut src).expect("it decodes");
    assert_eq!(s, back);
}
