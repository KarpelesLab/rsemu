//! The navi's buttons from the host keyboard.
//!
//! | Host key | Control |
//! | --- | --- |
//! | F1 | Setup (設定・編集) |
//! | F2 | Info (情報・TC) |
//! | F3 | Destination (目的地) |
//! | F4 | Current position (現在地) |
//! | F5 | TV |
//! | F6 | AUDIO |
//! | F7 | Traffic (交通情報) |
//! | F8 | Open (▲) |
//! | Page Up / Page Down | SEEK up / down |
//!
//! They reach the unit as its audio microcontroller sends them, over TAB-II
//! (`navi.tab`).

use alloc::sync::Arc;

use super::{InputEvent, InputSink, Keysym};
use crate::dev::navi::tab::{self, Tab};

/// Host keys and the controls they press.
const MAP: &[(Keysym, &str)] = &[
    (Keysym(0xffbe), "setup"),
    (Keysym(0xffbf), "info"),
    (Keysym(0xffc0), "destination"),
    (Keysym(0xffc1), "position"),
    (Keysym(0xffc2), "tv"),
    (Keysym(0xffc3), "audio"),
    (Keysym(0xffc4), "traffic"),
    (Keysym(0xffc5), "open"),
    (Keysym(0xff55), "seek-up"),
    (Keysym(0xff56), "seek-down"),
];

/// Presses the navi's buttons.
#[derive(Debug)]
pub struct NaviKeySink {
    tab: Arc<Tab>,
}

impl NaviKeySink {
    /// Drive `tab`.
    #[must_use]
    pub fn new(tab: Arc<Tab>) -> NaviKeySink {
        NaviKeySink { tab }
    }

    /// Drive the TAB-II peer this build constructed, if it has one.
    #[must_use]
    pub fn open(hosts: &crate::core::hosts::HostObjects) -> Option<NaviKeySink> {
        capture::take(hosts).map(NaviKeySink::new)
    }
}

impl InputSink for NaviKeySink {
    fn deliver(&self, event: InputEvent) {
        let InputEvent::Key { keysym, down } = event else {
            return;
        };
        if let Some(index) = MAP
            .iter()
            .find(|(k, _)| *k == keysym)
            .and_then(|(_, name)| tab::key(name))
        {
            self.tab.key(index, down);
        }
    }
}

/// Finding the TAB-II peer a build constructed.
pub mod capture {
    use alloc::sync::Arc;

    use crate::core::error::Result;
    use crate::core::hosts::{Captured, HostKind, HostObjects};
    use crate::dev::navi::tab::{CLASS, Tab};
    use crate::machine::BuildOptions;

    /// Replace `navi.tab`'s constructor in `options` with one that keeps a
    /// handle. Installed unconditionally: it builds the same device.
    ///
    /// # Errors
    ///
    /// [`crate::Error::Config`] if something else has already claimed this
    /// build's capture table for this class.
    pub fn install(options: &mut BuildOptions) -> Result<()> {
        let seen: Arc<Captured<Tab>> =
            options
                .realize
                .hosts
                .open(HostKind::CAPTURE, CLASS.name, Captured::new)?;
        options.bindings.replace(CLASS.name, move |props| {
            let tab = Arc::new(Tab::new(props)?);
            seen.push(&tab);
            Ok(tab)
        });
        Ok(())
    }

    /// The TAB-II peer this build constructed.
    #[must_use]
    pub fn take(hosts: &HostObjects) -> Option<Arc<Tab>> {
        hosts
            .get::<Captured<Tab>>(HostKind::CAPTURE, CLASS.name)
            .ok()
            .flatten()?
            .take()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn every_mapped_name_is_a_key() {
        for (_, name) in MAP {
            assert!(tab::key(name).is_some(), "{name}");
        }
    }
}
