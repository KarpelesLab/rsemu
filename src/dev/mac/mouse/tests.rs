//! The mouse's own tests: quadrature on the pins, two phase steps to a count,
//! the button in order, and the snapshot.

use super::*;
use alloc::vec::Vec;

use crate::core::state::{MachineShape, Migrations, StateReader, StateWriter};
use crate::core::wire::{Pull, Wire, WireId, WireSink};

/// What the five output pins are doing, as a wire would tell a sink: `true`
/// when the line is high. The button's line is pulled up and the switch pulls
/// it low, so a *pressed* button reads `false` here — which is what the Guide
/// means by "0 = button down".
#[derive(Debug)]
struct Pins {
    high: Mutex<[bool; 5]>,
}

impl Default for Pins {
    fn default() -> Pins {
        // The button's line starts where its pull-up puts it. A released
        // switch drives `HiZ`, which is not a change on a fresh source and so
        // delivers nothing — the same fact `mac.video`'s `drive`-rather-than-
        // `set` comment is about, seen from the other end.
        Pins {
            high: Mutex::new([false, false, false, false, true]),
        }
    }
}

#[derive(Debug)]
struct PinSink {
    pins: Arc<Pins>,
    line: usize,
}

impl WireSink for PinSink {
    fn set_level(&self, _src: WireId, _line: u32, level: Level) {
        self.pins.high.lock()[self.line] = level.is_high();
    }
}

fn rig() -> (MacMouse, Arc<Pins>) {
    let device = MacMouse::with(Arc::new(Mouse::new()));
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
        Device::announce(&device, name);
    }
    (device, pins)
}

#[test]
fn counting_up_walks_the_gray_code() {
    assert_eq!(phase_pins(0), (false, false));
    assert_eq!(phase_pins(1), (true, false));
    assert_eq!(phase_pins(2), (true, true));
    assert_eq!(phase_pins(3), (false, true));
    // Exactly one pin of the pair changes per step, either way round — which
    // is the whole point of quadrature and what lets the second wire say the
    // direction of a transition on the first.
    for p in 0..4u8 {
        let (a, b) = phase_pins(p);
        let (c, d) = phase_pins((p + 1) & 3);
        assert_ne!((a, b), (c, d));
        assert!((a != c) ^ (b != d), "phase {p} moved both pins");
    }
}

/// One count is two phase transitions, of which exactly one moves `X1` — the
/// wire the SCC takes an interrupt on. That is the whole of why a Macintosh
/// gets half an encoder's resolution.
#[test]
fn a_count_is_two_transitions_and_one_of_them_moves_the_counted_wire() {
    let (device, pins) = rig();
    let mouse = device.mouse();
    mouse.report(3, 0, 0);
    assert_eq!(mouse.backlog(), (3, 0));

    let mut x1_edges = 0;
    let mut x2_edges = 0;
    let mut last = pins.high.lock()[0..2].to_vec();
    for t in 1..=3 * u64::from(STEPS_PER_COUNT as u32) * DEFAULT_STEP_TICKS {
        mouse.advance_to(t);
        let now = pins.high.lock()[0..2].to_vec();
        if now[0] != last[0] {
            x1_edges += 1;
        }
        if now[1] != last[1] {
            x2_edges += 1;
        }
        last = now;
    }
    assert_eq!(x1_edges, 3, "one X1 transition a count");
    assert_eq!(x2_edges, 3, "and one X2 transition a count, between them");
    assert_eq!(mouse.backlog(), (0, 0));
    // The other axis never moved.
    assert!(!pins.high.lock()[2] && !pins.high.lock()[3]);
}

/// Motion arrives one transition at a time, at the rate a hand can reach,
/// rather than as a jump: the guest counts edges and has to be given time to
/// take an interrupt for each.
#[test]
fn motion_is_spread_at_the_step_rate() {
    let (device, _pins) = rig();
    let mouse = device.mouse();
    mouse.report(10, 0, 0);
    assert_eq!(mouse.backlog(), (10, 0));
    mouse.advance_to(DEFAULT_STEP_TICKS - 1);
    assert_eq!(mouse.backlog(), (10, 0), "nothing has been clocked yet");
    mouse.advance_to(DEFAULT_STEP_TICKS * 10);
    assert_eq!(mouse.backlog(), (5, 0), "ten steps is five counts");
    mouse.advance_to(DEFAULT_STEP_TICKS * 100);
    assert_eq!(mouse.backlog(), (0, 0));
}

/// **Both axes at once step at half rate each**, which keeps the *distance*
/// per tick where a single axis put it — and distance is what the ROM's
/// scaling threshold is on. `State::interval` has the measurement.
#[test]
fn two_axes_at_once_step_at_half_rate_each() {
    let (device, _pins) = rig();
    let mouse = device.mouse();
    mouse.report(10, -10, 0);
    assert_eq!(mouse.backlog(), (10, -10));
    mouse.advance_to(DEFAULT_STEP_TICKS * 2 - 1);
    assert_eq!(mouse.backlog(), (10, -10), "nothing has been clocked yet");
    mouse.advance_to(DEFAULT_STEP_TICKS * 20);
    assert_eq!(
        mouse.backlog(),
        (5, -5),
        "twenty step-ticks is ten transitions and so five counts, on each axis"
    );
    // And the moment one axis runs out, the other goes back to full rate.
    mouse.report(40, 0, 0);
    mouse.advance_to(DEFAULT_STEP_TICKS * 40);
    let (x, y) = mouse.backlog();
    assert_eq!(y, 0, "the vertical axis finished");
    mouse.advance_to(DEFAULT_STEP_TICKS * 60);
    let (x2, _) = mouse.backlog();
    assert_eq!(
        x - x2,
        10,
        "twenty step-ticks on one axis alone is ten counts, not five"
    );
}

/// A click lands where the pointer was sent rather than part way there.
#[test]
fn a_click_waits_for_the_motion_posted_before_it() {
    let (device, pins) = rig();
    let mouse = device.mouse();
    mouse.report(20, 0, 0);
    mouse.report(0, 0, BUTTON);
    mouse.advance_to(DEFAULT_STEP_TICKS * 10);
    assert!(pins.high.lock()[4], "the switch is still open");
    assert_ne!(mouse.backlog(), (0, 0));
    mouse.advance_to(DEFAULT_STEP_TICKS * 100);
    assert_eq!(mouse.backlog(), (0, 0));
    assert!(
        !pins.high.lock()[4],
        "the switch closed, which pulls its line low"
    );
    assert_eq!(mouse.buttons(), BUTTON);
}

fn snapshot(device: &MacMouse) -> Vec<u8> {
    let mut shape = MachineShape::new();
    shape.add_device("mouse", CLASS_NAME).unwrap();
    let mut w = StateWriter::new(shape);
    {
        let mut chunk = w.chunk("mouse", CLASS_NAME, STATE_VERSION).unwrap();
        Device::save(device, &mut chunk).unwrap();
    }
    w.to_vec().unwrap()
}

/// The queue and the phase are state, so a snapshot taken mid-sweep carries on
/// to the same place.
#[test]
fn a_snapshot_mid_motion_round_trips_and_carries_on_identically() {
    let (saved, _pins) = rig();
    saved.mouse().report(40, -17, 0);
    saved.mouse().report(0, 0, BUTTON);
    saved.mouse().advance_to(DEFAULT_STEP_TICKS * 9);
    let first = snapshot(&saved);

    let (restored, _other) = rig();
    let reader = StateReader::new(&first).unwrap();
    let chunk = reader
        .load("mouse", CLASS_NAME, STATE_VERSION, &Migrations::new())
        .unwrap();
    Device::load(&restored, &mut chunk.reader()).unwrap();
    assert_eq!(snapshot(&restored), first);
    assert_eq!(restored.mouse().backlog(), saved.mouse().backlog());

    saved.mouse().advance_to(DEFAULT_STEP_TICKS * 200);
    restored.mouse().advance_to(DEFAULT_STEP_TICKS * 200);
    assert_eq!(snapshot(&restored), snapshot(&saved));
    assert_eq!(restored.mouse().buttons(), BUTTON);
}

#[test]
fn the_class_names_its_pins() {
    let mut registry = crate::core::Registry::new();
    register(&mut registry).expect("a fresh registry");
    assert_eq!(registry.get(CLASS_NAME).unwrap().version, STATE_VERSION);
    let (device, _pins) = rig();
    let schema = schema();
    for pin in OUTPUT_PINS {
        assert!(schema.port_named(pin).is_some(), "{pin}");
    }
    let err = Device::connect(&device, "middle", dummy_source())
        .expect_err("a Macintosh mouse has one button")
        .to_string();
    assert!(err.contains("`button`"), "{err}");
}

/// A wire source that drives nothing, for the pin-name check above.
fn dummy_source() -> WireSource {
    let wire = Wire::builder().source(WireId::new(7)).build_shared();
    WireSource::new(wire, WireId::new(7))
}

/// A recorded report is five bytes and comes back through the sink as the
/// motion that went in.
#[test]
fn a_recorded_report_replays_as_the_same_motion() {
    let mouse = Arc::new(Mouse::new());
    let sink = mice::sink(&mouse);
    let mut payload = Vec::new();
    payload.extend_from_slice(&mice::encode(12, -5, BUTTON));
    payload.extend_from_slice(&mice::encode(-12, 5, 0));
    payload.push(0xff); // a trailing partial report, which is discarded
    sink.deliver(&payload);
    assert_eq!(mouse.backlog(), (0, 0), "there and back again");
    mouse.advance_to(DEFAULT_STEP_TICKS * 1000);
    assert_eq!(mouse.buttons(), 0, "the second report let it go");
}
