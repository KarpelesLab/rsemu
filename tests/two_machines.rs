//! Two machines of the same kind, built in one process, share no device.
//!
//! This is the claim the host-object table exists to make. It used to be false
//! and looked true: every named rendezvous — character ports, pad ports, power
//! signals, SPI buses — and every constructor-interception table lived in a
//! `static`, so two NES machines both resolved `pads = "player1"` to one set of
//! buttons and two PC/ATs both resolved the 8042's default `keyboard` port to
//! one keyboard. The tests that would have caught it were serialised against
//! each other, for reasons that had nothing to do with this, so no two machines
//! were ever alive at once where it would have shown.
//!
//! Each build now carries its own [`HostObjects`](rsemu::core::HostObjects) —
//! `RealizeOptions` holds it, `Props` carries it into every constructor, and the
//! host reads it back afterwards. Two `BuildOptions` is two tables is two of
//! everything, by construction rather than by convention.
//!
//! Nothing here serialises anything, which is itself part of the claim.

// Two machines of *some* shipped kind is what this asserts, so a build with no
// machine in it has nothing to say rather than a file of dead helpers.
#![cfg(any(feature = "machine-apple1", feature = "machine-nes"))]

use rsemu::core::clock::GlobalTime;
use rsemu::machine::{BuildOptions, catalog};
use std::sync::Arc;

/// Options for a shipped machine, with `media` bound.
fn options(media: &[(&str, &[u8])]) -> BuildOptions {
    let mut options = catalog::build_options().expect("the catalog agrees with itself");
    for (slot, bytes) in media {
        options.realize.media.insert(*slot, *bytes);
    }
    options
}

/// The same, with the display rate turned off.
///
/// The Apple 1's PIA paces its output at the real display's sixty characters a
/// second, which is right for a person and far too slow for a test that only
/// wants to know whose keyboard it is typing on.
#[cfg(feature = "machine-apple1")]
fn unpaced(media: &[(&str, &[u8])]) -> BuildOptions {
    options(media).with_param("pace", "false")
}

/// Realize a shipped machine against `options`.
fn realize(name: &str, options: &BuildOptions) -> rsemu::machine::Machine {
    let entry = catalog::machine(name).unwrap_or_else(|| panic!("this build ships {name}"));
    let registry = catalog::registry().expect("a registry");
    match rsemu::machine::build(entry.name, entry.source, &registry, options) {
        Ok(m) => m,
        Err(e) => panic!("{name} does not realize: {e}"),
    }
}

// ---------------------------------------------------------------------------
// Named rendezvous: ports, pads
// ---------------------------------------------------------------------------

/// A machine's console is its own, even when both machines call it the same
/// thing.
///
/// The Apple 1's PIA opens whatever `console` names; both of these name the
/// default. Before the host-object table that was one `CharPort`, so a test
/// typing at one machine typed at the other — and the shipped `rsemu run`
/// would have handed a second guest the first guest's keyboard.
#[cfg(feature = "machine-apple1")]
#[test]
fn two_apple_1s_do_not_share_a_console() {
    use rsemu::host::chardev::ports;

    let rom: &[u8] = rsemu::dev::apple1::RSMON;
    let left_options = unpaced(&[("rom", rom)]);
    let right_options = unpaced(&[("rom", rom)]);

    let mut left = realize("apple1", &left_options);
    let mut right = realize("apple1", &right_options);

    // Both machines opened a port, and both called it the same thing.
    let names = ports::names(&left_options.realize.hosts);
    assert_eq!(
        names,
        ports::names(&right_options.realize.hosts),
        "the two machines name their console identically"
    );
    assert_eq!(names.len(), 1, "one console each: {names:?}");

    let left_port = ports::open(&left_options.realize.hosts, &names[0]).expect("the PIA opened it");
    let right_port =
        ports::open(&right_options.realize.hosts, &names[0]).expect("the PIA opened it");
    assert!(
        !Arc::ptr_eq(&left_port, &right_port),
        "one name in two builds must be two ports"
    );

    // And the guests agree: typing at one is not typing at the other. Let both
    // reach their prompt and discard the banners first, so what is left is only
    // what the keystroke caused.
    let span = GlobalTime::from_nanos(200_000_000);
    left.run_for(span).expect("the left machine runs");
    right.run_for(span).expect("the right machine runs");
    assert!(
        !left_port.drain().is_empty(),
        "the left monitor never greeted anybody, so this proves nothing"
    );
    let _ = right_port.drain();

    // The monitor echoes what it is given, so what comes back is the proof.
    left_port.feed(b"A\r");
    left.run_for(span).expect("the left machine runs");
    right.run_for(span).expect("the right machine runs");

    let heard = left_port.drain();
    assert!(
        heard.contains(&b'A'),
        "the left monitor never echoed what was typed at it: {heard:?}"
    );
    let overheard = right_port.drain();
    assert!(
        !overheard.contains(&b'A'),
        "the right machine heard the left machine's keyboard: {overheard:?}"
    );
}

/// Buttons pressed on one console are not pressed on the other.
///
/// `dev::nes::input::pads` was the second of the process-wide tables, and this
/// is what it cost: two NES machines in one process read one set of buttons.
#[cfg(all(feature = "machine-nes", feature = "dev-nes-io"))]
#[test]
fn two_nes_consoles_do_not_share_a_controller() {
    use rsemu::dev::nes::input::{buttons, pads};

    let cart = blank_nrom();
    let left_options = options(&[("cart", cart.as_slice())]);
    let right_options = options(&[("cart", cart.as_slice())]);
    let _left = realize("nes-ntsc", &left_options);
    let _right = realize("nes-ntsc", &right_options);

    let names = pads::names(&left_options.realize.hosts);
    assert_eq!(names.len(), 1, "one pad port each: {names:?}");
    assert_eq!(names, pads::names(&right_options.realize.hosts));

    let left_pad = pads::open(&left_options.realize.hosts, &names[0]).expect("opened");
    let right_pad = pads::open(&right_options.realize.hosts, &names[0]).expect("opened");
    assert!(!Arc::ptr_eq(&left_pad, &right_pad));

    left_pad.set(0, buttons::A);
    assert_eq!(left_pad.get(0), buttons::A);
    assert_eq!(
        right_pad.get(0),
        buttons::NONE,
        "the other console's controller moved"
    );
}

// ---------------------------------------------------------------------------
// Constructor interception
// ---------------------------------------------------------------------------

/// The capture seam hands each build back its own chip.
///
/// `install` used to write into a `static`, so the second build's PPU replaced
/// the first's and `take` handed both callers the same one — silently, because
/// a `NesScanout` over the wrong machine still renders a picture.
#[cfg(all(feature = "machine-nes", feature = "dev-nes-ppu"))]
#[test]
fn two_nes_consoles_capture_two_ppus() {
    use rsemu::host::display::{Scanout, nes::capture};

    let cart = blank_nrom();
    let mut left_options = options(&[("cart", cart.as_slice())]);
    let mut right_options = options(&[("cart", cart.as_slice())]);
    capture::install(&mut left_options).expect("the interception installs");
    capture::install(&mut right_options).expect("and again, into another table");

    let mut left = realize("nes-ntsc", &left_options);
    let right = realize("nes-ntsc", &right_options);

    let left_ppu = capture::take(&left_options.realize.hosts).expect("the left machine has a PPU");
    let right_ppu =
        capture::take(&right_options.realize.hosts).expect("the right machine has a PPU");
    assert!(
        !Arc::ptr_eq(left_ppu.ppu(), right_ppu.ppu()),
        "two builds must capture two chips"
    );

    // And they are the chips of the machines they came from: only the left one
    // is run, and only the left picture moves.
    assert_eq!(left_ppu.frame_counter(), right_ppu.frame_counter());
    left.run_for(GlobalTime::from_nanos(4 * left_ppu.frame_period_ns()))
        .expect("the left machine runs");
    assert!(
        left_ppu.frame_counter() > 0,
        "the left machine drew nothing"
    );
    assert_eq!(
        right_ppu.frame_counter(),
        0,
        "the right machine drew a frame without being run"
    );
    drop(right);
}

/// 16 KiB of PRG, 8 KiB of CHR, and `JMP $C000` at the reset vector.
///
/// Generated rather than a fixture: nothing here executes, and the point is a
/// machine that realizes and runs, not a program that does anything.
#[cfg(feature = "machine-nes")]
fn blank_nrom() -> Vec<u8> {
    let mut image = vec![0u8; 16 + 16 * 1024 + 8 * 1024];
    image[0..4].copy_from_slice(b"NES\x1a");
    image[4] = 1;
    image[5] = 1;
    image[16] = 0x4c;
    image[17] = 0x00;
    image[18] = 0xc0;
    image[16 + 0x3ffc] = 0x00;
    image[16 + 0x3ffd] = 0xc0;
    image
}

// ---------------------------------------------------------------------------
// The table itself
// ---------------------------------------------------------------------------

/// Sharing is still available to a caller who asks for it.
///
/// Isolation is the default, not the only option: a debug console attached to a
/// machine, or a link cable between two consoles, wants one table on purpose.
/// `RealizeOptions::with_hosts` is how that is said out loud.
#[cfg(feature = "machine-apple1")]
#[test]
fn two_machines_share_a_console_only_when_asked_to() {
    use rsemu::host::chardev::ports;

    let rom: &[u8] = rsemu::dev::apple1::RSMON;
    let left_options = options(&[("rom", rom)]);
    let mut right_options = options(&[("rom", rom)]);
    right_options.realize.hosts = Arc::clone(&left_options.realize.hosts);

    let _left = realize("apple1", &left_options);
    let _right = realize("apple1", &right_options);

    let names = ports::names(&left_options.realize.hosts);
    assert_eq!(names.len(), 1, "one shared console: {names:?}");
    let a = ports::open(&left_options.realize.hosts, &names[0]).expect("opened");
    let b = ports::open(&right_options.realize.hosts, &names[0]).expect("opened");
    assert!(Arc::ptr_eq(&a, &b), "an explicitly shared table is shared");
}
