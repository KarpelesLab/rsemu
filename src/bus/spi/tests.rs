//! Tests for the SPI bus, its shifter, and the generic controller.
//!
//! The load-bearing one is
//! [`both_link_models_produce_identical_traffic`](tests::both_link_models_produce_identical_traffic):
//! the whole design claim is that a peripheral is written *once* and works
//! whether the controller hands it a word or clocks it in one edge at a time,
//! and that is the test that would fail if it were not true.

use super::controller::pin as cpin;
use super::controller::{SPI_CONTROLLER_CLASS, SpiController};
use super::*;

use alloc::string::{String, ToString};
use alloc::vec;
use alloc::vec::Vec;

use crate::core::device::{Device, ResetKind};
use crate::core::props::{Props, Value};
use crate::core::space::{MemAttrs, MemOps, RegionKind, RegionRef};
use crate::core::state::{MachineShape, Migrations, StateReader, StateWriter};
use crate::core::sync::{LockRank, Mutex};
use crate::core::wire::{Wire, WireId};

// ---------------------------------------------------------------------------
// A slave to talk to
// ---------------------------------------------------------------------------

/// A slave that records every word and answers with a fixed reply queue.
///
/// Deliberately not a real device: what is under test is the bus, and a mock
/// that logs is what makes "the same traffic arrived" a checkable claim.
#[derive(Debug)]
struct Echo {
    format: Format,
    log: Mutex<Vec<Word>>,
    /// What to put on MISO for the next transfer, consumed front to back.
    replies: Mutex<Vec<u32>>,
    /// The word currently in the shift register.
    loaded: Mutex<u32>,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Word {
    Select(bool),
    Transfer(u32),
}

impl Echo {
    fn new(format: Format, replies: &[u32]) -> Arc<Echo> {
        Arc::new(Echo {
            format,
            log: Mutex::with_rank(LockRank::LEAF, Vec::new()),
            replies: Mutex::with_rank(LockRank::LEAF, replies.to_vec()),
            loaded: Mutex::with_rank(LockRank::LEAF, u32::MAX),
        })
    }

    fn take_log(&self) -> Vec<Word> {
        core::mem::take(&mut *self.log.lock())
    }

    fn next_reply(&self) -> u32 {
        let mut replies = self.replies.lock();
        if replies.is_empty() {
            u32::MAX
        } else {
            replies.remove(0)
        }
    }
}

impl SpiSlave for Echo {
    fn format(&self) -> Format {
        self.format
    }

    fn select(&self, selected: bool) {
        self.log.lock().push(Word::Select(selected));
        if selected {
            *self.loaded.lock() = self.next_reply();
        }
    }

    fn transfer(&self, mosi: u32) -> u32 {
        self.log.lock().push(Word::Transfer(mosi));
        // Full duplex: what goes back is what was *already* loaded. The next
        // reply becomes the shift register's contents for the word after this.
        let out = *self.loaded.lock();
        let next = self.next_reply();
        *self.loaded.lock() = next;
        out
    }

    fn peek(&self) -> u32 {
        *self.loaded.lock()
    }
}

// ---------------------------------------------------------------------------
// Framing
// ---------------------------------------------------------------------------

#[test]
fn the_four_modes_sample_on_the_edge_their_datasheet_says() {
    // Mode 0: CPOL 0, sample on the rising edge.
    assert!(Mode::Mode0.samples_on(Level::High));
    assert!(!Mode::Mode0.samples_on(Level::Low));
    // Mode 1: CPOL 0, sample on the falling edge.
    assert!(!Mode::Mode1.samples_on(Level::High));
    assert!(Mode::Mode1.samples_on(Level::Low));
    // Mode 2: CPOL 1, sample on the falling edge.
    assert!(!Mode::Mode2.samples_on(Level::High));
    assert!(Mode::Mode2.samples_on(Level::Low));
    // Mode 3: CPOL 1, sample on the rising edge.
    assert!(Mode::Mode3.samples_on(Level::High));
    assert!(!Mode::Mode3.samples_on(Level::Low));
}

#[test]
fn a_mode_round_trips_through_its_number_and_its_bits() {
    for n in 0..4u8 {
        let mode = Mode::from_number(n).unwrap();
        assert_eq!(mode.number(), n);
        assert_eq!(Mode::from_cpol_cpha(mode.cpol(), mode.cpha()), mode);
        assert_eq!(mode.idle_level().is_high(), mode.cpol());
    }
    assert_eq!(Mode::from_number(4), None);
}

#[test]
fn a_word_width_is_clamped_rather_than_refused() {
    // A guest's control register can hold nonsense; that is the guest's bug to
    // see in its own timing, not a bus fault.
    assert_eq!(Format::new(Mode::Mode0, 0, BitOrder::MsbFirst).bits, 1);
    assert_eq!(Format::new(Mode::Mode0, 99, BitOrder::MsbFirst).bits, 32);
    assert_eq!(
        Format::new(Mode::Mode0, 16, BitOrder::MsbFirst).mask(),
        0xffff
    );
    assert_eq!(
        Format::new(Mode::Mode0, 32, BitOrder::MsbFirst).mask(),
        u32::MAX
    );
    assert_eq!(
        Format::new(Mode::Mode0, 8, BitOrder::MsbFirst).truncate(0x1234),
        0x34
    );
}

#[test]
fn the_link_spelling_a_machine_file_writes_round_trips() {
    for name in Link::NAMES {
        let link = Link::from_name(name).expect("a listed name parses");
        assert_eq!(link.name(), *name);
    }
    assert_eq!(Link::from_name("magic"), None);
}

// ---------------------------------------------------------------------------
// The fabric
// ---------------------------------------------------------------------------

#[test]
fn two_devices_on_one_chip_select_is_a_short_not_a_style() {
    let bus = SpiBus::new();
    let a = Echo::new(Format::DEFAULT, &[]);
    let b = Echo::new(Format::DEFAULT, &[]);
    bus.attach(ChipSelect(0), a).unwrap();
    let err = bus.attach(ChipSelect(0), b).unwrap_err().to_string();
    assert!(err.contains("two devices"), "{err}");
}

#[test]
fn a_chip_select_beyond_the_bus_is_refused() {
    let bus = SpiBus::new();
    let err = bus
        .attach(
            ChipSelect(MAX_CHIP_SELECTS as u8),
            Echo::new(Format::DEFAULT, &[]),
        )
        .unwrap_err()
        .to_string();
    assert!(err.contains("chip selects"), "{err}");
}

#[test]
fn selecting_deasserts_the_previous_slave_before_asserting_the_next() {
    let bus = SpiBus::new();
    let a = Echo::new(Format::DEFAULT, &[]);
    let b = Echo::new(Format::DEFAULT, &[]);
    bus.attach(ChipSelect(0), Arc::clone(&a) as Arc<dyn SpiSlave>)
        .unwrap();
    bus.attach(ChipSelect(1), Arc::clone(&b) as Arc<dyn SpiSlave>)
        .unwrap();

    bus.select(Some(ChipSelect(0)));
    assert_eq!(a.take_log(), vec![Word::Select(true)]);
    bus.select(Some(ChipSelect(1)));
    // A part commits its command on deassertion, so the order matters.
    assert_eq!(a.take_log(), vec![Word::Select(false)]);
    assert_eq!(b.take_log(), vec![Word::Select(true)]);
    assert_eq!(bus.selected(), Some(ChipSelect(1)));
    bus.select(None);
    assert_eq!(b.take_log(), vec![Word::Select(false)]);
    assert_eq!(bus.selected(), None);
}

#[test]
fn clocking_a_bus_with_nothing_on_it_reads_the_pull_up() {
    // Probing an empty chip select is ordinary firmware behaviour, not an error.
    let bus = SpiBus::new();
    assert_eq!(bus.transfer(0x55), u32::MAX);
    bus.select(Some(ChipSelect(3)));
    assert_eq!(bus.transfer(0x55), u32::MAX);
    assert_eq!(bus.peek(), u32::MAX);
}

#[test]
fn full_duplex_returns_what_was_already_in_the_shift_register() {
    // The trap this whole seam is shaped to avoid: `transfer` is not a
    // request/response. The reply to word N comes back during word N, not N+1,
    // because it was loaded before word N started.
    let bus = SpiBus::new();
    let echo = Echo::new(Format::DEFAULT, &[0xa5, 0x5a]);
    bus.attach(ChipSelect(0), Arc::clone(&echo) as Arc<dyn SpiSlave>)
        .unwrap();
    bus.select(Some(ChipSelect(0)));
    // Selection preloads 0xa5; the first transfer hands *that* back — not a
    // reply to it — and loads 0x5a for the word after.
    assert_eq!(echo.peek(), 0xa5);
    assert_eq!(bus.transfer(0x11), 0xa5);
    assert_eq!(echo.peek(), 0x5a);
    assert_eq!(
        echo.take_log(),
        vec![Word::Select(true), Word::Transfer(0x11)]
    );
}

#[test]
fn check_format_names_the_chip_select_that_disagrees() {
    let bus = SpiBus::new();
    bus.attach(ChipSelect(0), Echo::new(Format::DEFAULT, &[]))
        .unwrap();
    bus.attach(
        ChipSelect(2),
        Echo::new(Format::new(Mode::Mode3, 16, BitOrder::LsbFirst), &[]),
    )
    .unwrap();
    assert_eq!(bus.check_format(Format::DEFAULT), Some(ChipSelect(2)));
    assert_eq!(bus.attached(), vec![ChipSelect(0), ChipSelect(2)]);
    assert!(bus.detach(ChipSelect(2)));
    assert_eq!(bus.check_format(Format::DEFAULT), None);
}

#[test]
fn the_named_table_is_a_rendezvous_not_a_registry() {
    let name = "test-spi-rendezvous";
    buses::close(name);
    let a = buses::open(name);
    let b = buses::open(name);
    assert!(Arc::ptr_eq(&a, &b));
    assert!(buses::names().iter().any(|n| n == name));
    assert!(buses::get(name).is_some());
    assert!(buses::close(name));
    assert!(buses::get(name).is_none());
}

// ---------------------------------------------------------------------------
// The shifter
// ---------------------------------------------------------------------------

/// Clock `word` into a shifter through its wires, returning what came out.
fn shift_through(format: Format, word: u32, reply: u32) -> (Option<u32>, u32) {
    let mut shifter = Shifter::new(format);
    shifter.set_select(true);
    shifter.preload(reply);
    let idle = format.mode.idle_level();
    let mut received = None;
    let mut miso = 0u32;
    let mut bit = 0u32;
    for k in 0..u32::from(format.bits) * 2 {
        let level = if k % 2 == 0 { idle.inverted() } else { idle };
        if format.mode.samples_on(level) {
            // Present the bit the master is driving, then sample MISO exactly
            // as a master does: before the edge that clocks it.
            let out = match format.order {
                BitOrder::MsbFirst => (word >> (u32::from(format.bits) - 1 - bit)) & 1,
                BitOrder::LsbFirst => (word >> bit) & 1,
            };
            shifter.set_mosi(Level::from_bool(out != 0));
            if shifter.miso().is_high() {
                match format.order {
                    BitOrder::MsbFirst => miso |= 1 << (u32::from(format.bits) - 1 - bit),
                    BitOrder::LsbFirst => miso |= 1 << bit,
                }
            }
            bit += 1;
        }
        if let Shifted::Word { mosi, .. } = shifter.set_sck(level, |_| 0) {
            received = Some(mosi);
        }
    }
    (received, miso)
}

#[test]
fn a_word_survives_the_wires_in_every_mode_and_both_orders() {
    for n in 0..4u8 {
        let mode = Mode::from_number(n).unwrap();
        for order in [BitOrder::MsbFirst, BitOrder::LsbFirst] {
            for bits in [1u8, 8, 16, 32] {
                let format = Format::new(mode, bits, order);
                let word = 0xdead_beefu32 & format.mask();
                let reply = 0x1234_5678u32 & format.mask();
                let (got, miso) = shift_through(format, word, reply);
                assert_eq!(got, Some(word), "mosi through {format}");
                assert_eq!(miso, reply, "miso through {format}");
            }
        }
    }
}

#[test]
fn an_edge_that_does_not_move_the_clock_does_nothing() {
    let mut shifter = Shifter::new(Format::DEFAULT);
    shifter.set_select(true);
    // SCK is already low in mode 0.
    assert_eq!(shifter.set_sck(Level::Low, |_| 0), Shifted::Idle);
    assert!(!shifter.in_word());
}

#[test]
fn a_short_frame_is_reported_so_a_datasheet_rule_can_act_on_it() {
    // ST7272A datasheet v0.5 §7.1(d): "If less than 16 bits of SCL are input
    // while CS is low, the transferred data is ignored."
    let format = Format::new(Mode::Mode0, 16, BitOrder::MsbFirst);
    let mut shifter = Shifter::new(format);
    shifter.set_select(true);
    shifter.preload(0);
    shifter.set_mosi(Level::High);
    for _ in 0..3 {
        shifter.set_sck(Level::High, |_| 0);
        shifter.set_sck(Level::Low, |_| 0);
    }
    assert_eq!(shifter.bit_count(), 3);
    let partial = shifter.set_select(false);
    assert_eq!(partial, Some(0b111));
    assert!(!shifter.in_word());
}

#[test]
fn rewriting_the_framing_abandons_the_word_in_flight() {
    let mut shifter = Shifter::new(Format::DEFAULT);
    shifter.set_select(true);
    shifter.preload(0);
    shifter.set_mosi(Level::High);
    shifter.set_sck(Level::High, |_| 0);
    assert!(shifter.in_word());
    shifter.set_format(Format::new(Mode::Mode3, 16, BitOrder::LsbFirst));
    assert!(!shifter.in_word());
    assert_eq!(shifter.format().bits, 16);
}

#[test]
fn a_shifter_round_trips_through_its_snapshot() {
    let format = Format::new(Mode::Mode1, 12, BitOrder::LsbFirst);
    let mut shifter = Shifter::new(format);
    shifter.set_select(true);
    shifter.preload(0x555);
    shifter.set_mosi(Level::High);
    shifter.set_sck(Level::High, |_| 0);
    shifter.set_sck(Level::Low, |_| 0);
    let saved = shifter.snapshot();

    let mut other = Shifter::new(format);
    other.restore(saved);
    assert_eq!(other.snapshot(), saved);
    assert_eq!(other.bit_count(), shifter.bit_count());
    assert_eq!(other.miso(), shifter.miso());
}

// ---------------------------------------------------------------------------
// A slave's pins
// ---------------------------------------------------------------------------

/// Clock `word` into `pins` by driving the wires, the way a GPIO controller
/// would. Returns the bits that came back on MISO.
fn bitbang(pins: &SlavePins, format: Format, word: u32) -> u32 {
    let idle = format.mode.idle_level();
    pins.drive(pin::SCK, idle);
    pins.drive(pin::CS, Level::Low); // asserted
    let mut miso = 0u32;
    let mut bit = 0u32;
    for k in 0..u32::from(format.bits) * 2 {
        let level = if k % 2 == 0 { idle.inverted() } else { idle };
        if format.mode.samples_on(level) {
            let out = match format.order {
                BitOrder::MsbFirst => (word >> (u32::from(format.bits) - 1 - bit)) & 1,
                BitOrder::LsbFirst => (word >> bit) & 1,
            };
            pins.drive(pin::MOSI, Level::from_bool(out != 0));
            if pins.miso_level().is_high() {
                match format.order {
                    BitOrder::MsbFirst => miso |= 1 << (u32::from(format.bits) - 1 - bit),
                    BitOrder::LsbFirst => miso |= 1 << bit,
                }
            }
            bit += 1;
        }
        pins.drive(pin::SCK, level);
    }
    pins.drive(pin::CS, Level::High); // deasserted
    miso
}

#[test]
fn a_peripheral_written_once_can_be_bit_banged_through_its_pins() {
    let format = Format::new(Mode::Mode0, 16, BitOrder::MsbFirst);
    let echo = Echo::new(format, &[0xbeef, 0xcafe]);
    let pins = SlavePins::new(Arc::clone(&echo) as Arc<dyn SpiSlave>);
    let miso = bitbang(&pins, format, 0x1042);
    assert_eq!(miso, 0xbeef, "the word preloaded at selection came back");
    assert_eq!(
        echo.take_log(),
        vec![
            Word::Select(true),
            Word::Transfer(0x1042),
            Word::Select(false),
        ]
    );
}

#[test]
fn an_unselected_slave_presents_the_pull_up() {
    let echo = Echo::new(Format::DEFAULT, &[0x00]);
    let pins = SlavePins::new(echo as Arc<dyn SpiSlave>);
    assert_eq!(pins.miso_level(), Level::High);
    // And driving the clock while unselected changes nothing.
    pins.drive(pin::SCK, Level::High);
    assert_eq!(pins.miso_level(), Level::High);
}

#[test]
fn a_wire_reaches_a_slaves_pin() {
    let format = Format::new(Mode::Mode0, 8, BitOrder::MsbFirst);
    let echo = Echo::new(format, &[0x00]);
    let pins = Arc::new(SlavePins::new(Arc::clone(&echo) as Arc<dyn SpiSlave>));
    let src = WireId::new(1);
    let cs = Arc::new(
        Wire::builder()
            .source(src)
            .sink(pins.sink(pin::CS), pin::CS)
            .build(),
    );
    // A fresh net idles low, which for an active-low chip select is asserted.
    cs.set(src, Level::High);
    cs.set(src, Level::Low);
    assert_eq!(echo.take_log(), vec![Word::Select(true)]);
}

// ---------------------------------------------------------------------------
// The controller
// ---------------------------------------------------------------------------

fn props(pairs: &[(&str, Value)]) -> Props {
    let mut p = Props::new();
    for (name, value) in pairs {
        p.insert(*name, value.clone());
    }
    p
}

/// A transactional controller on a bus nobody else can reach.
///
/// Deliberately **not** through [`buses`]: `cargo test` runs tests in parallel
/// threads of one process, and two tests sharing one named bus would interfere
/// on the *bus*, which no lock can fix. (The table itself is safe to reach from
/// several threads — it is a [`Global`](crate::core::sync::Global) — but that
/// makes concurrent access defined, not correct.)
fn transactional(bus: &Arc<SpiBus>) -> SpiController {
    SpiController::with_bus(Link::Transactional, Some(Arc::clone(bus)), 1)
}

fn wired() -> SpiController {
    SpiController::with_bus(Link::Wired, None, 1)
}

fn regs(ctrl: &SpiController) -> RegionRef {
    ctrl.region("").expect("the controller maps its registers")
}

/// The `MemOps` behind an MMIO region.
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

/// `CTRL` for `bits`-wide words in `mode`, enabled.
fn ctrl_word(mode: Mode, bits: u8, order: BitOrder) -> u32 {
    let mut v = 1u32; // EN
    if mode.cpol() {
        v |= 1 << 1;
    }
    if mode.cpha() {
        v |= 1 << 2;
    }
    if order == BitOrder::LsbFirst {
        v |= 1 << 3;
    }
    v | (u32::from(bits - 1) << 8)
}

#[test]
fn a_transactional_controller_needs_a_bus_to_reach() {
    let err = SpiController::new(&props(&[(
        "link",
        Value::Str(String::from("transactional")),
    )]))
    .unwrap_err()
    .to_string();
    assert!(err.contains("named bus"), "{err}");
}

#[test]
fn an_unknown_link_names_the_ones_that_exist() {
    let err = SpiController::new(&props(&[("link", Value::Str(String::from("telepathy")))]))
        .unwrap_err()
        .to_string();
    assert!(err.contains("transactional"), "{err}");
    assert!(err.contains("wired"), "{err}");
}

#[test]
fn a_transfer_costs_its_real_time() {
    let bus = Arc::new(SpiBus::new());
    bus.attach(ChipSelect(0), Echo::new(Format::DEFAULT, &[0x42]))
        .unwrap();

    let ctrl = transactional(&bus);
    let region = regs(&ctrl);
    // Eight-bit words, mode 0, CLKDIV 3 -> a half period of 4 ticks, so a word
    // is 8 x 2 x 4 = 64 ticks.
    poke(&region, 0x00, ctrl_word(Mode::Mode0, 8, BitOrder::MsbFirst));
    poke(&region, 0x04, 3);
    poke(&region, 0x08, 1);
    poke(&region, 0x10, 0x99);

    assert_eq!(peek(&region, 0x0c) & 1, 1, "busy the moment it starts");
    ctrl.advance_to(63);
    assert_eq!(peek(&region, 0x0c) & 1, 1, "still busy one tick short");
    ctrl.advance_to(64);
    assert_eq!(peek(&region, 0x0c) & 1, 0, "done on the tick it was due");
    assert_eq!(peek(&region, 0x0c) & 2, 2, "and the word is waiting");
    assert_eq!(peek(&region, 0x10), 0x42);
    assert_eq!(peek(&region, 0x0c) & 2, 0, "reading DATA pops it");
}

#[test]
fn the_tick_a_transfer_is_due_on_is_published_for_the_scheduler() {
    let bus = Arc::new(SpiBus::new());
    bus.attach(ChipSelect(0), Echo::new(Format::DEFAULT, &[0]))
        .unwrap();
    let ctrl = transactional(&bus);
    let region = regs(&ctrl);
    assert_eq!(ctrl.next_event_tick(), None, "idle schedules nothing");
    poke(&region, 0x00, ctrl_word(Mode::Mode0, 8, BitOrder::MsbFirst));
    poke(&region, 0x08, 1);
    poke(&region, 0x10, 0);
    // The scheduler requires this to be strictly greater than the current tick
    // or catch-up makes no progress and the device stalls where it stands.
    let due = ctrl.next_event_tick().expect("a transfer is pending");
    assert!(due > ctrl.current_tick(), "{due} must be past the present");
}

#[test]
fn a_debug_read_of_data_does_not_pop_the_fifo() {
    // The exact trap `MemAttrs::debug` exists for: a debugger that consumed the
    // guest's word would make the guest read a stale one, and nothing in the
    // core can undo that after the fact (`ROADMAP.md` §15, invariant 5).
    let bus = Arc::new(SpiBus::new());
    bus.attach(ChipSelect(0), Echo::new(Format::DEFAULT, &[0x7e]))
        .unwrap();
    let ctrl = transactional(&bus);
    let region = regs(&ctrl);
    poke(&region, 0x00, ctrl_word(Mode::Mode0, 8, BitOrder::MsbFirst));
    poke(&region, 0x08, 1);
    poke(&region, 0x10, 0);
    ctrl.advance_to(1000);

    assert_eq!(peek_with(&region, 0x10, MemAttrs::DEBUG), 0x7e);
    assert_eq!(
        peek(&region, 0x0c) & 2,
        2,
        "the debug read left RXVALID alone"
    );
    assert_eq!(
        peek(&region, 0x10),
        0x7e,
        "and the guest still gets its word"
    );
    assert_eq!(peek(&region, 0x0c) & 2, 0);
}

#[test]
fn a_debug_write_is_refused_rather_than_faked() {
    let ctrl = wired();
    let region = regs(&ctrl);
    let err = ops(&region)
        .write(0x10, &1u32.to_le_bytes(), MemAttrs::DEBUG)
        .unwrap_err();
    assert!(matches!(err, crate::core::error::BusError::BadAccess));
}

#[test]
fn a_disabled_controller_starts_nothing() {
    let bus = Arc::new(SpiBus::new());
    let echo = Echo::new(Format::DEFAULT, &[]);
    bus.attach(ChipSelect(0), Arc::clone(&echo) as Arc<dyn SpiSlave>)
        .unwrap();
    let ctrl = transactional(&bus);
    let region = regs(&ctrl);
    poke(&region, 0x08, 1);
    poke(&region, 0x10, 0x55);
    ctrl.advance_to(10_000);
    assert!(!ctrl.busy());
    assert_eq!(
        echo.take_log(),
        vec![Word::Select(true)],
        "the chip select still moved; no word was clocked"
    );
}

// ---------------------------------------------------------------------------
// The claim the whole design rests on
// ---------------------------------------------------------------------------

/// Run `words` through a transactional controller, returning what came back.
fn run_transactional(format: Format, words: &[u32], replies: &[u32]) -> (Vec<Word>, Vec<u32>) {
    let bus = Arc::new(SpiBus::new());
    let echo = Echo::new(format, replies);
    bus.attach(ChipSelect(0), Arc::clone(&echo) as Arc<dyn SpiSlave>)
        .unwrap();

    let ctrl = transactional(&bus);
    let region = regs(&ctrl);
    poke(
        &region,
        0x00,
        ctrl_word(format.mode, format.bits, format.order),
    );
    poke(&region, 0x04, 0);
    poke(&region, 0x08, 1);

    let mut now = 0u64;
    let mut got = Vec::new();
    for word in words {
        poke(&region, 0x10, *word);
        now += u64::from(format.bits) * 2;
        ctrl.advance_to(now);
        got.push(peek(&region, 0x10));
    }
    poke(&region, 0x08, 0);
    (echo.take_log(), got)
}

/// The same, through a controller that drives real wires into real pins.
fn run_wired(format: Format, words: &[u32], replies: &[u32]) -> (Vec<Word>, Vec<u32>) {
    let echo = Echo::new(format, replies);
    let pins = Arc::new(SlavePins::new(Arc::clone(&echo) as Arc<dyn SpiSlave>));

    let ctrl = wired();
    let region = regs(&ctrl);

    // Three nets from the controller's outputs to the slave's pins, and one
    // back for MISO — exactly what a machine file's `wire` statements build.
    let ids = [
        WireId::new(1),
        WireId::new(2),
        WireId::new(3),
        WireId::new(4),
    ];
    let sck = Wire::builder()
        .source(ids[0])
        .sink(pins.sink(pin::SCK), pin::SCK)
        .build_shared();
    let mosi = Wire::builder()
        .source(ids[1])
        .sink(pins.sink(pin::MOSI), pin::MOSI)
        .build_shared();
    let cs = Wire::builder()
        .source(ids[2])
        .sink(pins.sink(pin::CS), pin::CS)
        .build_shared();
    ctrl.connect(cpin::SCK, WireSource::new(sck, ids[0]))
        .unwrap();
    ctrl.connect(cpin::MOSI, WireSource::new(mosi, ids[1]))
        .unwrap();
    ctrl.connect("cs0", WireSource::new(cs, ids[2])).unwrap();

    // And MISO back, from the slave's pins into the controller's sink.
    let miso_sink = ctrl.sink(cpin::MISO, &[ids[3]]).expect("a miso input");
    let miso = Wire::builder()
        .source(ids[3])
        .sink(miso_sink.sink, miso_sink.line)
        .build_shared();
    pins.connect_miso(WireSource::new(miso, ids[3]));

    poke(
        &region,
        0x00,
        ctrl_word(format.mode, format.bits, format.order),
    );
    poke(&region, 0x04, 0);
    poke(&region, 0x08, 1);

    let mut now = 0u64;
    let mut got = Vec::new();
    for word in words {
        poke(&region, 0x10, *word);
        now += u64::from(format.bits) * 2;
        ctrl.advance_to(now);
        got.push(peek(&region, 0x10));
    }
    poke(&region, 0x08, 0);
    (echo.take_log(), got)
}

#[test]
fn both_link_models_produce_identical_traffic() {
    // The claim `docs/buses/low-speed.md` asks for and the reason `SpiSlave` is
    // word-level: a peripheral is written once, and a machine that switches
    // `link` gets the same bytes in the same order with the same timing. If
    // this ever fails, the two models have diverged and one of them is lying.
    let words = [0x1234u32, 0x00ff, 0xa55a];
    let replies = [0xbeef, 0xcafe, 0xf00d, 0x0bad];
    for n in 0..4u8 {
        let mode = Mode::from_number(n).unwrap();
        for order in [BitOrder::MsbFirst, BitOrder::LsbFirst] {
            let format = Format::new(mode, 16, order);
            let (log_t, rx_t) = run_transactional(format, &words, &replies);
            let (log_w, rx_w) = run_wired(format, &words, &replies);
            assert_eq!(log_t, log_w, "traffic differs in {format}");
            assert_eq!(rx_t, rx_w, "received words differ in {format}");
            assert_eq!(
                log_t,
                vec![
                    Word::Select(true),
                    Word::Transfer(0x1234),
                    Word::Transfer(0x00ff),
                    Word::Transfer(0xa55a),
                    Word::Select(false),
                ],
                "and both are the traffic that was asked for"
            );
        }
    }
}

#[test]
fn firmware_can_bit_bang_the_lines_register_and_the_slave_cannot_tell() {
    // The case `docs/buses/low-speed.md` names: guest firmware driving the pins
    // itself. Here it does it through the controller's own `LINES` register with
    // the controller disabled, which is a GPIO controller in all but name.
    let format = Format::new(Mode::Mode0, 8, BitOrder::MsbFirst);
    let echo = Echo::new(format, &[0x3c, 0x00]);
    let pins = Arc::new(SlavePins::new(Arc::clone(&echo) as Arc<dyn SpiSlave>));

    let ctrl = wired();
    let region = regs(&ctrl);
    let ids = [
        WireId::new(1),
        WireId::new(2),
        WireId::new(3),
        WireId::new(4),
    ];
    let sck = Wire::builder()
        .source(ids[0])
        .sink(pins.sink(pin::SCK), pin::SCK)
        .build_shared();
    let mosi = Wire::builder()
        .source(ids[1])
        .sink(pins.sink(pin::MOSI), pin::MOSI)
        .build_shared();
    let cs = Wire::builder()
        .source(ids[2])
        .sink(pins.sink(pin::CS), pin::CS)
        .build_shared();
    ctrl.connect(cpin::SCK, WireSource::new(sck, ids[0]))
        .unwrap();
    ctrl.connect(cpin::MOSI, WireSource::new(mosi, ids[1]))
        .unwrap();
    ctrl.connect("cs0", WireSource::new(cs, ids[2])).unwrap();
    let miso_sink = ctrl.sink(cpin::MISO, &[ids[3]]).expect("a miso input");
    let miso = Wire::builder()
        .source(ids[3])
        .sink(miso_sink.sink, miso_sink.line)
        .build_shared();
    pins.connect_miso(WireSource::new(miso, ids[3]));

    // LINES: bit 0 SCK, bit 1 MOSI, bit 2 CS-as-driven (1 = deasserted).
    const SCK: u32 = 1;
    const MOSI: u32 = 2;
    const CS_HIGH: u32 = 4;

    poke(&region, 0x14, CS_HIGH); // idle
    poke(&region, 0x14, 0); // assert CS, SCK low
    let word = 0x96u32;
    let mut got = 0u32;
    for bit in (0..8).rev() {
        let d = if word >> bit & 1 != 0 { MOSI } else { 0 };
        poke(&region, 0x14, d); // data settles, clock still low
        if peek(&region, 0x14) & (1 << 8) != 0 {
            got |= 1 << bit;
        }
        poke(&region, 0x14, d | SCK); // rising edge samples it
        poke(&region, 0x14, d); // and back down
    }
    poke(&region, 0x14, CS_HIGH); // deassert

    assert_eq!(
        echo.take_log(),
        vec![
            Word::Select(true),
            Word::Transfer(0x96),
            Word::Select(false)
        ],
        "the slave saw an ordinary transfer"
    );
    assert_eq!(
        got, 0x3c,
        "and full duplex worked in the other direction too"
    );
}

// ---------------------------------------------------------------------------
// Snapshots
// ---------------------------------------------------------------------------

#[test]
fn the_controller_round_trips_through_a_snapshot() {
    let bus = Arc::new(SpiBus::new());
    bus.attach(ChipSelect(0), Echo::new(Format::DEFAULT, &[0x5a]))
        .unwrap();

    let ctrl = transactional(&bus);
    let region = regs(&ctrl);
    poke(
        &region,
        0x00,
        ctrl_word(Mode::Mode3, 16, BitOrder::LsbFirst),
    );
    poke(&region, 0x04, 7);
    poke(&region, 0x08, 1);
    poke(&region, 0x10, 0xbeef);
    ctrl.advance_to(100);

    let mut shape = MachineShape::new();
    shape.add_device("spi", SPI_CONTROLLER_CLASS.name).unwrap();
    let mut w = StateWriter::new(shape.clone());
    {
        let mut chunk = w
            .chunk(
                "spi",
                SPI_CONTROLLER_CLASS.name,
                SPI_CONTROLLER_CLASS.version,
            )
            .unwrap();
        ctrl.save(&mut chunk).unwrap();
    }
    let bytes = w.to_vec().unwrap();

    let other = transactional(&bus);
    let reader = StateReader::new(&bytes).unwrap();
    let chunk = reader
        .load(
            "spi",
            SPI_CONTROLLER_CLASS.name,
            SPI_CONTROLLER_CLASS.version,
            &Migrations::new(),
        )
        .unwrap();
    other.load(&mut chunk.reader()).unwrap();

    assert_eq!(other.ticks(), ctrl.ticks());
    assert_eq!(other.format(), ctrl.format());
    assert_eq!(other.rx(), ctrl.rx());
    assert_eq!(other.busy(), ctrl.busy());

    // And the loaded copy reports the same state through its own registers,
    // which is the property a state hash compares.
    let other_region = regs(&other);
    for offset in [0x00u64, 0x04, 0x08, 0x0c, 0x14] {
        assert_eq!(
            peek_with(&other_region, offset, MemAttrs::DEBUG),
            peek_with(&region, offset, MemAttrs::DEBUG),
            "register {offset:#04x} differs after a round trip"
        );
    }
}

#[test]
fn a_cold_reset_returns_every_register_to_its_documented_value() {
    let ctrl = wired();
    let region = regs(&ctrl);
    poke(
        &region,
        0x00,
        ctrl_word(Mode::Mode3, 32, BitOrder::LsbFirst),
    );
    poke(&region, 0x04, 99);
    poke(&region, 0x14, 3);
    ctrl.reset(ResetKind::Cold);
    assert_eq!(peek(&region, 0x00), 7 << 8, "8-bit words, mode 0, disabled");
    assert_eq!(peek(&region, 0x04), 0);
    assert_eq!(peek(&region, 0x08), 0);
    assert_eq!(peek(&region, 0x0c), 0);
    assert_eq!(
        peek(&region, 0x14) & 7,
        4,
        "SCK low, MOSI low, CS deasserted"
    );
}

#[test]
fn the_register_block_is_word_wide_only() {
    let ctrl = wired();
    let region = regs(&ctrl);
    let block = ops(&region);
    let mut byte = [0u8; 1];
    assert!(block.read(0x00, &mut byte, MemAttrs::DEFAULT).is_err());
    let mut word = [0u8; 4];
    assert!(block.read(0x02, &mut word, MemAttrs::DEFAULT).is_err());
}
