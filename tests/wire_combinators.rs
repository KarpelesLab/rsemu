//! `wire.not`, `wire.and`, `wire.or` and `wire.split` from a machine file.
//!
//! `ROADMAP.md` §4.3 names the combinators and the unit tests beside them prove
//! each gate's truth table. What a unit test cannot show is that a *machine
//! description* can reach them: that the classes are registered, bound and
//! described to the validator, that a `wire` statement finds their pins, and
//! that the realize sweep announces an inverter's idle-high output rather than
//! leaving the net where an undriven one sits. That sweep is §4.3's own
//! example of why it exists, and it is the thing that goes wrong silently.
//!
//! The board under test is the smallest one that can observe a pin from the
//! guest's side of the bus: an AT's system control port at 0x61, whose bit 5 is
//! the `timer2` *input* read straight back out and whose bit 0 is the `gate2`
//! output. Wiring one to the other through a gate makes the whole path visible
//! to an `IN` instruction.

#![cfg(feature = "dev-pc")]

use rsemu::core::device::ResetKind;
use rsemu::core::space::MemAttrs;
use rsemu::core::value::Width;
use rsemu::machine::build;
use rsemu::machine::realize::Bindings;

/// Port 0x61's bit 0 (the timer 2 gate, an output) and bit 5 (that counter's
/// output, an input read back).
const GATE2: u8 = 0x01;
const TIMER2_IN: u8 = 0x20;

fn options() -> rsemu::machine::BuildOptions {
    let mut b = Bindings::new();
    rsemu::machine::builtin::bind(&mut b).expect("ram, rom and the gates");
    rsemu::dev::pc::bind(&mut b).expect("the chipset");
    rsemu::machine::BuildOptions::new()
        .with_classes(rsemu::machine::catalog::classes())
        .with_bindings(b)
}

fn assemble(source: &str) -> Result<rsemu::machine::Machine, String> {
    let registry = rsemu::machine::catalog::registry().expect("this build's registry");
    build("gates.machine", source, &registry, &options()).map_err(|e| e.to_string())
}

fn inb(m: &rsemu::machine::Machine, port: u64) -> u8 {
    m.space("port")
        .expect("the I/O space")
        .read(port, Width::U8, MemAttrs::DEFAULT)
        .expect("a decoded port") as u8
}

fn outb(m: &rsemu::machine::Machine, port: u64, value: u8) {
    m.space("port")
        .expect("the I/O space")
        .write(port, Width::U8, u64::from(value), MemAttrs::DEFAULT)
        .expect("a decoded port");
}

/// A board whose port 0x61 feeds its own bit 5 through `gate`.
///
/// `wiring` connects the gate; `objects` declares it. The loop runs through
/// `pc.sysctl`, which is sequential, so it is a legitimate handshake rather
/// than the combinational cycle §4.3 rejects.
fn loopback(objects: &str, wiring: &str) -> Result<rsemu::machine::Machine, String> {
    assemble(&format!(
        r#"machine "gates" {{
  space port {{ width = 16, unassigned = read-as-ones }}
  object sysctl "pc.sysctl" {{}}
{objects}
  map port 0x0061 size 0x0001 = sysctl.portb
{wiring}
}}"#
    ))
}

#[test]
fn an_inverter_comes_up_driving_high() {
    // §4.3: "An undriven wire sits low, which contradicts an inverter's
    // idle-high output, so a freshly realized *or freshly restored* machine is
    // inconsistent until every gate drives what its inputs imply." Nothing
    // drives this inverter's input, so its output must be high before the guest
    // has done anything at all.
    let mut m = loopback(
        "  object inv \"wire.not\" {}",
        "  wire inv.out -> sysctl.timer2",
    )
    .expect("the board realizes");
    m.reset(ResetKind::Cold);
    m.sweep();
    assert_eq!(
        inb(&m, 0x61) & TIMER2_IN,
        TIMER2_IN,
        "the realize sweep never announced the inverter's idle output"
    );
}

#[test]
fn a_level_travels_out_of_a_device_through_a_gate_and_back_in() {
    let mut m = loopback(
        "  object inv \"wire.not\" {}",
        "  wire sysctl.gate2 -> inv.in\n  wire inv.out -> sysctl.timer2",
    )
    .expect("the board realizes");
    m.reset(ResetKind::Cold);
    m.sweep();
    assert_eq!(
        inb(&m, 0x61) & TIMER2_IN,
        TIMER2_IN,
        "gate2 low, so out high"
    );

    outb(&m, 0x61, GATE2);
    assert_eq!(inb(&m, 0x61) & TIMER2_IN, 0, "gate2 high, so out low");
    outb(&m, 0x61, 0);
    assert_eq!(inb(&m, 0x61) & TIMER2_IN, TIMER2_IN, "and back");
}

#[test]
fn an_and_gate_needs_both_of_its_inputs_from_a_machine_file() {
    // One input from the guest, one from an inverter that idles high — so the
    // AND follows `gate2` exactly, and the assertion that it does is also an
    // assertion that `in0` and `in1` are separate pins rather than one net.
    let mut m = loopback(
        "  object inv \"wire.not\" {}\n  object gate \"wire.and\" { inputs = 2 }",
        "  wire sysctl.gate2 -> gate.in0\n  wire inv.out -> gate.in1\n  \
         wire gate.out -> sysctl.timer2",
    )
    .expect("the board realizes");
    m.reset(ResetKind::Cold);
    m.sweep();
    assert_eq!(inb(&m, 0x61) & TIMER2_IN, 0, "one input of two");
    outb(&m, 0x61, GATE2);
    assert_eq!(inb(&m, 0x61) & TIMER2_IN, TIMER2_IN, "and now both");
}

#[test]
fn a_split_carries_one_level_to_several_pins() {
    // Both halves of the split land on the same sink, which is a wired-OR of
    // one gate with itself — uninteresting electrically and exactly the point
    // here: two nets came out of one pin, and a device that only drove the
    // first would leave this reading zero.
    let mut m = loopback(
        "  object fan \"wire.split\" { outputs = 2 }",
        "  wire sysctl.gate2 -> fan.in\n  wire fan.out0 -> sysctl.timer2\n  \
         wire fan.out1 -> sysctl.refresh",
    )
    .expect("the board realizes");
    m.reset(ResetKind::Cold);
    m.sweep();
    assert_eq!(inb(&m, 0x61) & TIMER2_IN, 0);
    // Bit 4 is the refresh toggle, a divide-by-two on the `refresh` pin, so one
    // edge on `out1` flips it exactly once. Sampled before, because a second
    // edge would put it back and the test would pass on a device that never
    // moved it.
    let before = inb(&m, 0x61) & 0x10;
    outb(&m, 0x61, GATE2);
    assert_eq!(inb(&m, 0x61) & TIMER2_IN, TIMER2_IN, "out0 followed `in`");
    assert_ne!(inb(&m, 0x61) & 0x10, before, "out1 never moved");
}

#[test]
fn a_ring_of_gates_is_refused_and_a_loop_through_a_device_is_not() {
    // §4.3's rule, from the machine file's side: a cycle is an error only when
    // every device in it is combinational, because that is exactly the cycle
    // the realize sweep has no order for.
    let e = loopback(
        "  object a \"wire.not\" {}\n  object b \"wire.not\" {}",
        "  wire a.out -> b.in\n  wire b.out -> a.in",
    )
    .expect_err("a ring of inverters never settles");
    assert!(
        e.contains("cycle") || e.contains("combinational"),
        "the diagnostic should name the loop: {e}"
    );

    // The same shape with a `pc.sysctl` in it is a handshake, and it is the
    // test above — which passed.
}

#[test]
fn a_pin_a_gate_does_not_have_is_reported_by_name() {
    let e = loopback(
        "  object gate \"wire.and\" { inputs = 2 }",
        "  wire sysctl.gate2 -> gate.in7\n  wire gate.out -> sysctl.timer2",
    )
    .expect_err("a two-input gate has no `in7`");
    assert!(e.contains("in7"), "{e}");

    let e = loopback(
        "  object gate \"wire.and\" { inputs = 2 }",
        "  wire sysctl.gate2 -> gate.inx\n  wire gate.out -> sysctl.timer2",
    )
    .expect_err("`inx` is not a pin at all");
    assert!(e.contains("inx"), "{e}");

    let e = loopback(
        "  object gate \"wire.and\" { inpts = 2 }",
        "  wire gate.out -> sysctl.timer2",
    )
    .expect_err("a misspelt property");
    assert!(e.contains("inpts"), "{e}");
}
