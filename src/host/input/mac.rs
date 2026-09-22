//! A person's pointer, as a Macintosh's mouse receives it.
//!
//! One [`InputSink`], the Macintosh's counterpart of [`MouseSink`]:
//! [`MacMouseSink`] turns absolute pointer positions into counts for a
//! `mac.mouse`. It delivers straight into the device's host object, downstream
//! of the frontend's own `input:` channel — the event was recorded when the
//! frontend posted it, so posting again on the device's channel would record it
//! twice. `input::amiga` set the shape.
//!
//! [`MouseSink`]: super::MouseSink
//!
//! There is no keyboard half here. `mac.keyboard` takes the *Guide*'s own
//! transition codes and nothing in this tree turns a keysym into one; Figure
//! 7-6 has the table and the scans of it in circulation are not clean enough to
//! transcribe. `docs/platforms/mac-plus.md`'s ledger carries it.

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
/// * The guest loses about **one count in two hundred** — it counts interrupts,
///   and an edge arriving inside the level-2 handler with the VIA also waiting
///   is one nothing counts. So the pointer tracks rather than matching, and the
///   error does not cancel. A sweep into a screen edge pins the two ends back
///   together, because the ROM clamps the pointer to the screen and the host's
///   cursor stops at the same place; that is what a person does without
///   thinking about it, and `tests/mac_plus.rs` does it deliberately before
///   each placement.
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
