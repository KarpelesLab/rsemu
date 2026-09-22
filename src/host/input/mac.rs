//! A person's pointer, as a Macintosh's mouse receives it.
//!
//! Two [`InputSink`]s, the Macintosh's counterpart of [`MouseSink`]:
//! [`MacMouseSink`] turns absolute pointer positions into counts for a
//! `mac.mouse` — a Plus's quadrature mouse, two pulse trains on the VIA and the
//! SCC — and [`MacAdbSink`] does the same job for a Classic, where the pointer
//! is a **device on the Apple Desktop Bus**. Both deliver straight into the
//! device's host object, downstream of the frontend's own `input:` channel —
//! the event was recorded when the frontend posted it, so posting again on the
//! device's channel would record it twice. `input::amiga` set the shape.
//!
//! [`MouseSink`]: super::MouseSink
//!
//! There is no keyboard half here, on either seam. `mac.keyboard` takes the
//! *Guide*'s own transition codes and `mac.adb` takes ADB key codes, and
//! nothing in this tree turns a keysym into either: Figure 7-6 has the first
//! table and the scans of it in circulation are not clean enough to transcribe,
//! and inventing the second is precisely the failure
//! `docs/platforms/mac-plus.md` records under "The drive's register file was
//! invented". Both platform ledgers carry it.

#[cfg(feature = "dev-mac")]
use alloc::sync::Arc;

#[cfg(feature = "dev-mac")]
use super::{InputEvent, InputSink};
#[cfg(feature = "dev-mac")]
use crate::core::sync::Mutex;

/// Framebuffer pixels per mouse count.
///
/// **One**, and it is a measurement of Apple's ROM rather than a convention.
/// How far a guest moves its pointer per count is the guest's own business —
/// its mouse speed and its acceleration — so the only way to know is to ask it.
/// Posting counts and then finding the arrow in the rendered frame gives
/// exactly one 512 × 342 pixel per count: 39 counts to the right moved
/// `Mouse` at `$830` from 15 to 54 and moved the arrow's mark in the picture
/// from x = 15 to x = 54 (`tests/mac_plus.rs`).
///
/// Two caveats, both measured and both in
/// [`DEFAULT_STEP_TICKS`](crate::dev::mac::mouse::DEFAULT_STEP_TICKS):
///
/// * It holds **only below the ROM's own scaling threshold**. A delta of six or
///   more counts in one 60.15 Hz tick is doubled, and the device's default step
///   rate exists to keep it under that. A board run with `-p mousestep=` below
///   the default gets a faster pointer that no longer lands where it was sent,
///   which is the Amiga's old defect the other way round.
/// * The guest loses about **one count in three hundred**, and the loss is the
///   *ROM's* rather than the hardware's: every transition reaches the SCC,
///   latches, pulls `/INT` and is serviced — 400 of 400 at every step rate,
///   counted at all four places in `tests/mac_plus.rs` — but the cursor VBL
///   task reads `MTemp`, scales it and "also updates MTemp to reflect the new
///   value" (Apple Technical Note DV 520) at interrupt mask 0, so a count the
///   handler adds inside that window is overwritten. A real Macintosh loses it
///   too. So the pointer tracks rather than matching, and the error does not
///   cancel. A sweep into a screen edge pins the two ends back together,
///   because the ROM clamps the pointer to the screen and the host's cursor
///   stops at the same place; that is what a person does without thinking
///   about it, and `tests/mac_plus.rs` does it deliberately before each
///   placement.
#[cfg(feature = "dev-mac")]
pub const PIXELS_PER_COUNT: i64 = 1;

/// An [`InputSink`] that moves a `mac.mouse`.
///
/// Converts as [`MouseSink`](super::MouseSink) does — keeps the last position
/// and sends the difference, the first event only establishing where the
/// pointer is — and carries the part of a delta smaller than one count so slow
/// movement is not lost.
///
/// A Macintosh mouse has **one** button, so only bit 0 of a report's buttons
/// crosses; a right-click has nowhere to go and is dropped rather than
/// delivered as a left one.
#[cfg(feature = "dev-mac")]
#[derive(Debug)]
pub struct MacMouseSink {
    mouse: Arc<crate::dev::mac::mouse::Mouse>,
    /// Where the pointer was last reported, in framebuffer pixels, and the
    /// button then held.
    at: Mutex<Option<(i64, i64, u8)>>,
}

#[cfg(feature = "dev-mac")]
impl MacMouseSink {
    /// Move `mouse`.
    #[must_use]
    pub fn new(mouse: Arc<crate::dev::mac::mouse::Mouse>) -> MacMouseSink {
        MacMouseSink {
            mouse,
            at: Mutex::new(None),
        }
    }

    /// Move the first Macintosh mouse this build opened, if it has one.
    #[must_use]
    pub fn open(hosts: &crate::core::hosts::HostObjects) -> Option<MacMouseSink> {
        use crate::dev::mac::mouse::mice;
        let name = mice::names(hosts).into_iter().next()?;
        let mouse = mice::get(hosts, &name).ok().flatten()?;
        Some(MacMouseSink::new(mouse))
    }
}

#[cfg(feature = "dev-mac")]
impl InputSink for MacMouseSink {
    fn deliver(&self, event: InputEvent) {
        use crate::dev::mac::mouse::BUTTON;
        let InputEvent::Pointer { x, y, buttons } = event else {
            return;
        };
        let (x, y) = (i64::from(x), i64::from(y));
        let buttons = buttons & BUTTON;
        let (dx, dy, moved_buttons) = {
            let mut at = self.at.lock();
            let (px, py, pb) = at.unwrap_or((x, y, 0));
            let dx = (x - px) / PIXELS_PER_COUNT;
            let dy = (y - py) / PIXELS_PER_COUNT;
            // Advance by what was sent, so the remainder is still owed.
            *at = Some((
                px + dx * PIXELS_PER_COUNT,
                py + dy * PIXELS_PER_COUNT,
                buttons,
            ));
            (dx, dy, pb != buttons)
        };
        if dx == 0 && dy == 0 && !moved_buttons {
            return;
        }
        #[allow(clippy::cast_possible_truncation)]
        self.mouse.report(
            dx.clamp(-32_767, 32_767) as i32,
            dy.clamp(-32_767, 32_767) as i32,
            buttons,
        );
    }
}

/// How many counts one report may carry per axis.
///
/// **Four**, and the number is the ROM's rather than the bus's. An Apple
/// Desktop Bus mouse's register 0 packs a seven-bit signed delta an axis, so
/// the wire would take sixty-three; but the Macintosh's own cursor task
/// *doubles* a movement of six or more counts in one 60.15 Hz tick
/// ([`crate::dev::mac::mouse::DEFAULT_STEP_TICKS`] has the table, measured on a
/// Plus, and a Classic's ROM scales the same way), and a guest that accelerates
/// cannot be pointed at anything by a host whose cursor is absolute: the
/// pointer runs ahead, pins at an edge, and the two never agree again.
///
/// Four is under the threshold on both axes at once, which matters because the
/// threshold is on the two together. A far target therefore takes several
/// reports, and that is what the remainder in [`MacAdbSink`] is for — the
/// caller posts the same absolute position again and the rest of the distance
/// follows.
#[cfg(feature = "dev-mac")]
pub const MAX_COUNTS: i64 = 4;

/// An [`InputSink`] that moves the mouse on a `mac.adb` bus.
///
/// Converts as [`MacMouseSink`] does — it keeps where the host's cursor was and
/// sends the difference, the first event only establishing a position — and
/// hands out at most [`MAX_COUNTS`] an axis per report, carrying the rest.
/// Posting the same position again sends the next instalment, so a caller that
/// wants the pointer somewhere delivers until it arrives.
///
/// A Macintosh mouse has **one** button, so only bit 0 of a report's buttons
/// crosses; a right-click has nowhere to go.
#[cfg(feature = "dev-mac")]
#[derive(Debug)]
pub struct MacAdbSink {
    adb: Arc<crate::dev::mac::adb::Adb>,
    /// Where the pointer has been told to go, and the button then held.
    at: Mutex<Option<(i64, i64, u8)>>,
}

#[cfg(feature = "dev-mac")]
impl MacAdbSink {
    /// Move the mouse on `adb`.
    #[must_use]
    pub fn new(adb: Arc<crate::dev::mac::adb::Adb>) -> MacAdbSink {
        MacAdbSink {
            adb,
            at: Mutex::new(None),
        }
    }

    /// Move the mouse on the first Apple Desktop Bus this build opened, if it
    /// has one.
    #[must_use]
    pub fn open(hosts: &crate::core::hosts::HostObjects) -> Option<MacAdbSink> {
        use crate::dev::mac::adb::bus;
        let name = bus::names(hosts).into_iter().next()?;
        let adb = bus::get(hosts, &name).ok().flatten()?;
        Some(MacAdbSink::new(adb))
    }
}

#[cfg(feature = "dev-mac")]
impl InputSink for MacAdbSink {
    fn deliver(&self, event: InputEvent) {
        let InputEvent::Pointer { x, y, buttons } = event else {
            return;
        };
        let (x, y) = (i64::from(x), i64::from(y));
        let buttons = buttons & 1;
        let (dx, dy, moved_buttons) = {
            let mut at = self.at.lock();
            let (px, py, pb) = at.unwrap_or((x, y, 0));
            let dx = (x - px).clamp(-MAX_COUNTS, MAX_COUNTS);
            let dy = (y - py).clamp(-MAX_COUNTS, MAX_COUNTS);
            // Advance by what is being sent, so the rest is still owed.
            *at = Some((px + dx, py + dy, buttons));
            (dx, dy, pb != buttons)
        };
        if dx == 0 && dy == 0 && !moved_buttons {
            return;
        }
        #[allow(clippy::cast_possible_truncation)]
        self.adb.mouse(dx as i8, dy as i8, buttons != 0);
    }
}

#[cfg(all(test, feature = "dev-mac"))]
mod tests {
    use super::*;
    use crate::dev::mac::mouse::{BUTTON, Mouse};

    /// The first event only says where the host's cursor is; the second is the
    /// first motion the guest sees. Otherwise a session would begin by throwing
    /// the pointer at wherever the window happened to be.
    #[test]
    fn the_first_event_only_establishes_a_position() {
        let mouse = Arc::new(Mouse::new());
        let sink = MacMouseSink::new(Arc::clone(&mouse));
        sink.deliver(InputEvent::Pointer {
            x: 300,
            y: 200,
            buttons: 0,
        });
        assert_eq!(mouse.backlog(), (0, 0));
        sink.deliver(InputEvent::Pointer {
            x: 340,
            y: 180,
            buttons: 0,
        });
        assert_eq!(
            mouse.backlog(),
            (40, -20),
            "one count a pixel, right and up"
        );
    }

    /// A Macintosh mouse has one button, and the other two have nowhere to go.
    #[test]
    fn only_the_first_button_crosses() {
        let mouse = Arc::new(Mouse::new());
        let sink = MacMouseSink::new(Arc::clone(&mouse));
        let at = |buttons: u8| InputEvent::Pointer {
            x: 10,
            y: 10,
            buttons,
        };
        sink.deliver(at(0));
        sink.deliver(at(0b110));
        mouse.advance_to(1_000_000);
        assert_eq!(
            mouse.buttons(),
            0,
            "the right and middle buttons are dropped"
        );
        sink.deliver(at(0b111));
        mouse.advance_to(2_000_000);
        assert_eq!(mouse.buttons(), BUTTON);
    }
}
