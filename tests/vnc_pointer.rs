//! A client's pointer, all the way to the bytes a guest reads off the wire.
//!
//! `tests/vnc_input.rs` proves a *key* travels: a viewer's KeyEvent becomes an
//! `InputEvent`, the seam delivers it at a round boundary, and a guest polls
//! the scan code out of an 8042. The pointer had no such claim, and it had none
//! because it had nowhere to land: `KeyboardSink` and `PadSink` both open with
//! `let InputEvent::Key { .. } = event else { return }`, so **every pointer
//! event a client sent was decoded, encoded into the recording, delivered — and
//! silently dropped**.
//!
//! [`MouseSink`](rsemu::host::input::MouseSink) is where one lands now, and
//! this file is the join: real RFB PointerEvent bytes in at one end
//! ([`proto::parse_client`](rsemu::host::vnc::proto::parse_client), the same
//! parser the server runs), a HID boot-mouse report out of the interrupt
//! endpoint at the other — read through
//! [`UsbDevice::transfer_in`](rsemu::bus::usb::UsbDevice::transfer_in), which is
//! the call a host controller makes on behalf of a guest.
//!
//! # What this does not claim
//!
//! **That a *shipped* machine has a mouse.** `machines/xhci-pci-mini` does —
//! it was built for exactly this claim, and `tests/xhci_pci.rs` carries a
//! `PointerEvent` through it to a report a guest reads off an interrupt
//! endpoint — but the boards someone actually runs still do not. `usb-mini`,
//! `hub-mini` and `xhci-mini` have controllers and no display; `pc-at`, `q35`
//! and `q35-linux` have displays and, until someone adds the two objects
//! `machines/xhci-pci-mini.machine` spells out, no USB.
//!
//! What is no longer true is the reason this file used to give for that:
//! joining them *is* now a machine-file edit. `usb.xhci-pci` is the PCI
//! attachment that was missing, so a PC guest enumerates class code `0C0330h`
//! and finds a host controller.
//!
//! What `tests/usb_ehci.rs` already proves is the other half of the same wire:
//! a guest executing RV32 instructions builds a periodic schedule, and
//! `HidMouse::motion` lands in guest RAM through the controller's own DMA. The
//! two files meet at `motion`.

#![cfg(all(feature = "vnc", feature = "dev-usb-hid"))]

use std::sync::Arc;

use rsemu::bus::usb::Status;
use rsemu::dev::usb::hid::HidMouse;
use rsemu::host::input::{Feed, InputEvent, Keysym, MouseSink};
use rsemu::host::vnc::proto::{ClientMessage, Parsed, client_msg, parse_client};

/// The interrupt IN endpoint a boot mouse reports on.
const ENDPOINT: u8 = 1;

/// One RFB PointerEvent (RFC 6143 §7.5.5): type 5, the button mask, then x and
/// y as big-endian `u16`s.
fn pointer_event(buttons: u8, x: u16, y: u16) -> Vec<u8> {
    let mut out = vec![client_msg::POINTER_EVENT, buttons];
    out.extend_from_slice(&x.to_be_bytes());
    out.extend_from_slice(&y.to_be_bytes());
    out
}

/// Parse the wire bytes the way `host::vnc` does, and hand back the event it
/// would have posted.
///
/// The conversion in the middle is the server's, copied here because it is one
/// widening and because copying it is what makes this a test of the *protocol*
/// rather than of a helper.
fn decode(bytes: &[u8]) -> InputEvent {
    match parse_client(bytes) {
        Parsed::Message(ClientMessage::Pointer { x, y, buttons }, used) => {
            assert_eq!(used, bytes.len(), "a PointerEvent is six bytes");
            InputEvent::Pointer {
                x: u32::from(x),
                y: u32::from(y),
                buttons,
            }
        }
        other => panic!("not a pointer event: {other:?}"),
    }
}

/// A mouse with the sink that drives it, wired the way `vnc_session` wires them.
fn mouse() -> (Arc<HidMouse>, Feed) {
    let mouse = Arc::new(HidMouse::new_detached(0x1234, 0x5678));
    let feed = Feed::new();
    feed.attach(Arc::new(MouseSink::new(Arc::clone(&mouse))));
    (mouse, feed)
}

/// What a host controller would collect from the interrupt endpoint.
///
/// `None` for a NAK, which is what an interrupt endpoint with nothing new says
/// (USB 2.0 §8.5.4) and what the guest's queue head sees between reports.
fn poll(mouse: &HidMouse) -> Option<[u8; 3]> {
    let mut buf = [0u8; 8];
    let done = mouse.device().transfer_in(ENDPOINT, &mut buf);
    match done.status {
        Status::Ack => {
            assert_eq!(done.len, 3, "a boot report is three bytes");
            Some([buf[0], buf[1], buf[2]])
        }
        Status::Nak => None,
        other => panic!("the endpoint answered {other:?}"),
    }
}

/// The claim: a client moves its pointer and the guest's mouse moves by the
/// difference.
#[test]
fn a_clients_pointer_becomes_a_relative_report() {
    let (mouse, feed) = mouse();

    // The first event only says where the pointer is. A jump from the origin to
    // wherever the cursor entered the window would fling the guest's pointer
    // across the screen, so the delta is zero and the position is remembered.
    feed.deliver(decode(&pointer_event(0, 100, 100)));
    assert_eq!(poll(&mouse), Some([0, 0, 0]), "the first event is a datum");

    // The second is movement.
    feed.deliver(decode(&pointer_event(0, 110, 105)));
    assert_eq!(poll(&mouse), Some([0, 10, 5]));

    // And backwards, which is where the sign matters.
    feed.deliver(decode(&pointer_event(0, 90, 130)));
    let report = poll(&mouse).expect("a report");
    assert_eq!(report[1] as i8, -20);
    assert_eq!(report[2] as i8, 25);

    // Single-buffered, like the hardware: once collected there is nothing to
    // collect until it moves again.
    assert_eq!(poll(&mouse), None, "an idle endpoint NAKs");
}

/// RFB's buttons are in physical order and HID's are in usage order, so bits 1
/// and 2 swap.
///
/// RFC 6143 §7.5.5: bit 0 left, bit 1 middle, bit 2 right. HID 1.11 Appendix
/// B.2: bit 0 button 1 (primary, left), bit 1 button 2 (secondary, right), bit
/// 2 button 3 (tertiary, middle). Getting this backwards is invisible until
/// somebody right-clicks, which is exactly the kind of defect worth a test.
#[test]
fn the_button_mask_is_translated_rather_than_copied() {
    let (mouse, feed) = mouse();
    feed.deliver(decode(&pointer_event(0, 10, 10)));
    let _ = poll(&mouse);

    // Left is bit 0 in both.
    feed.deliver(decode(&pointer_event(0b001, 10, 10)));
    assert_eq!(poll(&mouse).expect("a report")[0], 0b001);

    // The middle button is RFB's bit 1 and HID's bit 2.
    feed.deliver(decode(&pointer_event(0b010, 10, 10)));
    assert_eq!(poll(&mouse).expect("a report")[0], 0b100);

    // The right button is RFB's bit 2 and HID's bit 1.
    feed.deliver(decode(&pointer_event(0b100, 10, 10)));
    assert_eq!(poll(&mouse).expect("a report")[0], 0b010);

    // All three at once, and the wheel — RFB bits 3 and 4, sent as button
    // presses — dropped, because this device's report has no wheel axis and a
    // guest would otherwise see a scroll as a click.
    feed.deliver(decode(&pointer_event(0b1_1111, 10, 10)));
    assert_eq!(poll(&mouse).expect("a report")[0], 0b111);
}

/// A jump bigger than a report can carry is clamped, and the remainder is still
/// owed.
///
/// The report's logical range is -127..127 (HID 1.11 Appendix E.10), so a
/// pointer that crosses a screen in one event cannot be expressed. What must
/// not happen is that the rest is thrown away: the sink advances its reference
/// position by what it *sent*, so the next event carries the remainder rather
/// than starting over from the client's new position.
#[test]
fn a_jump_is_clamped_and_the_remainder_is_carried() {
    let (mouse, feed) = mouse();
    feed.deliver(decode(&pointer_event(0, 0, 0)));
    let _ = poll(&mouse);

    feed.deliver(decode(&pointer_event(0, 300, 0)));
    let report = poll(&mouse).expect("a report");
    assert_eq!(report[1] as i8, 127, "as far as one report reaches");

    // The client has not moved; the sink still owes 173 pixels.
    feed.deliver(decode(&pointer_event(0, 300, 0)));
    assert_eq!(poll(&mouse).expect("a report")[1] as i8, 127);
    feed.deliver(decode(&pointer_event(0, 300, 0)));
    assert_eq!(poll(&mouse).expect("a report")[1] as i8, 46);

    // Arrived: nothing more is owed.
    feed.deliver(decode(&pointer_event(0, 300, 0)));
    assert_eq!(poll(&mouse).expect("a report")[1] as i8, 0);
}

/// A keystroke is not a mouse movement.
///
/// The mirror of `host::input`'s "a pointer event is not a keystroke": a feed
/// fans every event out to every sink, so each sink has to ignore what is not
/// its own. A `MouseSink` that moved on a key press would make typing scroll
/// the screen.
#[test]
fn a_key_reaches_no_mouse() {
    let (mouse, feed) = mouse();
    feed.deliver(InputEvent::Key {
        keysym: Keysym::from_ascii(b'a'),
        down: true,
    });
    assert!(!mouse.has_report(), "a key moved the pointer");
    assert_eq!(poll(&mouse), None);
}
