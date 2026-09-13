//! The mouse's own tests: quadrature on the pins, buttons in order, and the
//! snapshot.

use super::*;
use alloc::vec::Vec;

use crate::core::state::{MachineShape, Migrations, StateReader, StateWriter};
use crate::core::wire::{Level, Pull, Wire, WireId, WireSink};

/// What the seven output pins are doing, as a wire would tell a sink.
#[derive(Debug, Default)]
struct Pins {
    low: Mutex<[bool; 7]>,
}

#[derive(Debug)]
struct PinSink {
    pins: Arc<Pins>,
    line: usize,
}

impl WireSink for PinSink {
    fn set_level(&self, _src: WireId, _line: u32, level: Level) {
        self.pins.low.lock()[self.line] = level.is_low();
    }
}

fn rig() -> (AmigaMouse, Arc<Pins>) {
    let device = AmigaMouse::with(Arc::new(Mouse::new()));
    let pins = Arc::new(Pins::default());
    for (line, name) in OUTPUT_PINS.iter().enumerate() {
        let id = WireId::new(line as u64 + 1);
        let wire = Wire::builder()
            .source(id)
            .resolved(Pull::Up)
            .sink(
                Arc::new(PinSink {
                    pins: Arc::clone(&pins),
                    line,
                }),
                0,
            )
            .build_shared();
        device
            .connect(name, WireSource::new(wire, id))
            .expect("an output");
    }
    (device, pins)
}

#[test]
fn counting_up_walks_the_gray_code_and_idle_is_every_pin_released() {
    assert_eq!(phase_pins(0), (false, false), "nothing pulled: idle");
    assert_eq!(phase_pins(1), (true, false));
    assert_eq!(phase_pins(2), (true, true));
    assert_eq!(phase_pins(3), (false, true));
    // One pin changes per step, either way round.
    for p in 0..4u8 {
        let (a, b) = phase_pins(p);
        let (c, d) = phase_pins((p + 1) & 3);
        assert_eq!(u8::from(a != c) + u8::from(b != d), 1);
    }
}

#[cfg(feature = "dev-amiga-denise")]
#[test]
fn denise_reads_the_phase_back_as_the_counter_low_bits() {
    use crate::dev::amiga::denise::quadrature_bits;
    for p in 0..4u8 {
        let (pin_low, q_low) = phase_pins(p);
        assert_eq!(quadrature_bits(!pin_low, !q_low), p);
    }
}

#[test]
fn motion_leaves_one_transition_per_step_on_each_axis() {
    let (device, pins) = rig();
    let mouse = device.mouse();
    mouse.report(3, -2, 0);
    assert_eq!(mouse.backlog(), (3, -2));
    mouse.advance_to(DEFAULT_STEP_TICKS - 1);
    assert_eq!(
        *pins.low.lock(),
        [false; 7],
        "nothing before the first step"
    );
    mouse.advance_to(DEFAULT_STEP_TICKS);
    // X up one (phase 1), Y down one (phase 3).
    assert_eq!(pins.low.lock()[..4], [true, false, false, true]);
    mouse.advance_to(3 * DEFAULT_STEP_TICKS);
    assert_eq!(mouse.backlog(), (0, 0));
    // X at phase 3, Y at phase 2 (-2 from 0).
    assert_eq!(pins.low.lock()[..4], [false, true, true, true]);
    assert_eq!(device.next_event_tick(), None, "idle once it has arrived");
}

#[test]
fn a_click_waits_for_the_motion_posted_before_it() {
    let (device, pins) = rig();
    let mouse = device.mouse();
    mouse.report(4, 0, 0);
    mouse.report(0, 0, buttons::LEFT | buttons::RIGHT);
    mouse.advance_to(3 * DEFAULT_STEP_TICKS);
    assert_eq!(mouse.buttons(), 0, "still on its way");
    mouse.advance_to(4 * DEFAULT_STEP_TICKS);
    assert_eq!(mouse.buttons(), buttons::LEFT | buttons::RIGHT);
    let low = *pins.low.lock();
    assert!(
        low[4] && low[5] && !low[6],
        "left and right pulled, middle not"
    );

    // A click on a mouse at rest is at once.
    mouse.report(0, 0, buttons::MIDDLE);
    assert_eq!(mouse.buttons(), buttons::MIDDLE);
}

fn snapshot(device: &AmigaMouse) -> Vec<u8> {
    let mut shape = MachineShape::new();
    shape.add_device("mouse", CLASS_NAME).unwrap();
    let mut w = StateWriter::new(shape);
    {
        let mut chunk = w.chunk("mouse", CLASS_NAME, STATE_VERSION).unwrap();
        Device::save(device, &mut chunk).unwrap();
    }
    w.to_vec().unwrap()
}

#[test]
fn a_snapshot_mid_motion_round_trips_and_carries_on_identically() {
    let (device, _) = rig();
    device.mouse().report(50, -7, 0);
    device.mouse().report(-3, 1, buttons::LEFT);
    device.mouse().advance_to(10 * DEFAULT_STEP_TICKS + 13);
    let bytes = snapshot(&device);

    let (other, pins) = rig();
    let reader = StateReader::new(&bytes).unwrap();
    let chunk = reader
        .load("mouse", CLASS_NAME, STATE_VERSION, &Migrations::new())
        .unwrap();
    Device::load(&other, &mut chunk.reader()).unwrap();
    assert_eq!(snapshot(&other), bytes);
    assert_eq!(other.mouse().backlog(), device.mouse().backlog());
    // The pins were driven to the restored phase.
    let (x, xq) = phase_pins(10 & 3);
    assert_eq!(pins.low.lock()[..2], [x, xq]);

    for d in [&device, &other] {
        d.mouse().advance_to(100 * DEFAULT_STEP_TICKS);
    }
    assert_eq!(snapshot(&other), snapshot(&device));
    assert_eq!(other.mouse().buttons(), buttons::LEFT);
}

#[test]
fn the_class_names_its_pins() {
    let debug = alloc::format!("{:?}", schema());
    for pin in OUTPUT_PINS {
        assert!(debug.contains(pin), "{pin}");
    }
    assert_eq!(mice::encode(-2, 300, 5), [0xfe, 0xff, 0x2c, 0x01, 5]);
}
