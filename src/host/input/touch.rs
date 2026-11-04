//! A touch screen driven by the host pointer: the left button is the finger.
//!
//! The navi's panel is a resistive touch controller (`rohm.bu21023`); a
//! frontend's absolute pointer maps straight onto it, so pressing the button
//! is touching the screen there, dragging is a move, and letting go lifts the
//! finger. The device scales pixels to its own range.

use alloc::sync::Arc;

use super::{InputEvent, InputSink};
use crate::dev::bu21023::Bu21023;

/// Delivers pointer events to a touch controller.
#[derive(Debug)]
pub struct TouchSink {
    panel: Arc<Bu21023>,
}

impl TouchSink {
    /// Drive `panel`.
    #[must_use]
    pub fn new(panel: Arc<Bu21023>) -> TouchSink {
        TouchSink { panel }
    }

    /// Drive the touch controller this build constructed, if it has one.
    #[must_use]
    pub fn open(hosts: &crate::core::hosts::HostObjects) -> Option<TouchSink> {
        capture::take(hosts).map(TouchSink::new)
    }
}

impl InputSink for TouchSink {
    fn deliver(&self, event: InputEvent) {
        let InputEvent::Pointer { x, y, buttons } = event else {
            return;
        };
        self.panel.touch(x, y, buttons & 1 != 0);
    }
}

/// Finding the touch controller a build constructed.
pub mod capture {
    use alloc::sync::Arc;

    use crate::core::error::Result;
    use crate::core::hosts::{Captured, HostKind, HostObjects};
    use crate::dev::bu21023::{Bu21023, CLASS};
    use crate::machine::BuildOptions;

    /// Replace `rohm.bu21023`'s constructor in `options` with one that keeps a
    /// handle.
    ///
    /// Installed unconditionally, like the mouse's: the interception builds
    /// the same device from the same properties, so whether a frontend is
    /// listening cannot change the machine.
    ///
    /// # Errors
    ///
    /// [`crate::Error::Config`] if something else has already claimed this
    /// build's capture table for this class.
    pub fn install(options: &mut BuildOptions) -> Result<()> {
        let seen: Arc<Captured<Bu21023>> =
            options
                .realize
                .hosts
                .open(HostKind::CAPTURE, CLASS.name, Captured::new)?;
        options.bindings.replace(CLASS.name, move |props| {
            let panel = Arc::new(Bu21023::new(props)?);
            seen.push(&panel);
            Ok(panel)
        });
        Ok(())
    }

    /// The touch controller this build constructed, the most recent one if
    /// there are several.
    #[must_use]
    pub fn take(hosts: &HostObjects) -> Option<Arc<Bu21023>> {
        hosts
            .get::<Captured<Bu21023>>(HostKind::CAPTURE, CLASS.name)
            .ok()
            .flatten()?
            .take()
    }
}
