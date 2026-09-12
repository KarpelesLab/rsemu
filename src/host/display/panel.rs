//! The host side of the **device-owned framebuffer** seam.
//!
//! [`PanelScanout`] holds an `Arc<dyn Panel>` — a smart panel that keeps its own
//! display RAM, an SSD1306 or an ST7789 — and presents it as a [`Scanout`] a
//! host can capture. [`crate::dev::lcd::panel`] argues why that is a different
//! kind of device from [`lcd`](super::lcd)'s RGB engine; this module is the
//! consequence.
//!
//! # One adapter for the whole family
//!
//! [`nes`](super::nes), [`sms`](super::sms), [`gb`](super::gb) and [`pc`](super::pc)
//! each need their own adapter because each converts a *different* encoding: a
//! palette index, a colour-table entry, a DAC index. A smart panel does not —
//! its datasheet defines the map from display RAM to glass completely, so the
//! device has already resolved it by the time anyone asks and hands over
//! finished intensities. One adapter therefore serves every such part, and a
//! new controller adds an interception here rather than a module.
//!
//! # There is no frame rate, and that is the honest answer
//!
//! [`Scanout::frame_period_ns`] is `0` — "no fixed rate". A real SSD1306
//! refreshes at `FOSC / (D × K × MUX)` and a real ST7789 at its own rate, but
//! **nothing in the machine can observe either**: no pin, no counter, no
//! interrupt. A host that advanced the machine by such a period to get "the
//! next frame" would be advancing for nothing, because the picture changes when
//! the guest sends bytes and at no other time. So the host is told there is no
//! rate, and [`Scanout::frame_counter`] carries [`Panel::generation`] instead —
//! a count of *content changes*, which is the question a host that redraws on
//! change is actually asking.
//!
//! # Getting hold of the panel
//!
//! Exactly as for [`lcd::capture`](super::lcd::capture), and for the same
//! reason: a machine built from a description hands back `Arc<dyn Device>` and
//! there is no route from that to a concrete type, because `Device` has no
//! `Any` in its supertrait chain on purpose. So the host takes its handle at
//! the one moment the concrete type exists — device construction:
//!
//! ```text
//! let mut options = catalog::build_options()?;
//! display::panel::capture::install(&mut options)?;   // intercept every panel class
//! let machine = machine::build(name, source, &registry, &options)?;
//! let picture = display::panel::capture::take(&options.realize.hosts);
//! ```
//!
//! [`capture::take`] needs no `Machine`, unlike the RGB engine's: that one has
//! to resolve a frame period out of the clock forest, and this one has no frame
//! period to resolve.
//!
//! **One table, many classes.** The interception is installed once per
//! compiled-in [`PanelClass`](crate::dev::lcd::panel::PanelClass) and every one
//! of them pushes into the same `Captured<dyn Panel>`, filed under
//! [`capture::CAPTURE_NAME`]. A board with an OLED and a board with a TFT are
//! the same call on the host side.
//!
//! This is a seam and it is marked as one. When `Device` grows a scanout hook,
//! every line of [`capture`] deletes.

use alloc::sync::Arc;
use alloc::vec;

use super::{PixelFormat, Scanout, Surface, SurfaceInfo};
use crate::dev::lcd::panel::Panel;

/// A [`Scanout`] over a display controller that owns its own framebuffer.
#[derive(Debug, Clone)]
pub struct PanelScanout {
    panel: Arc<dyn Panel>,
}

impl PanelScanout {
    /// Watch `panel`.
    #[must_use]
    pub fn new(panel: Arc<dyn Panel>) -> PanelScanout {
        PanelScanout { panel }
    }

    /// The panel being watched, for a host that wants to ask it something else.
    #[must_use]
    pub fn panel(&self) -> &Arc<dyn Panel> {
        &self.panel
    }
}

impl Scanout for PanelScanout {
    fn info(&self) -> SurfaceInfo {
        let (width, height) = self.panel.geometry();
        // RGB888, for the reason `lcd` gives: the device hands out RGB triples
        // and the seam has a format for exactly that, so nothing pads and
        // nothing converts.
        SurfaceInfo::new(width, height, PixelFormat::RGB888)
    }

    fn frame_counter(&self) -> u64 {
        self.panel.generation()
    }

    fn capture(&self, dst: &mut Surface) -> u64 {
        let info = self.info();
        dst.reshape(dst.format(), info.width, info.height);

        // The counter before the pixels: if the emulation thread is mid-write
        // the surface may hold the picture *after* this one, and a serial that
        // is never ahead of its pixels is the safe direction to err.
        let serial = self.panel.generation();
        let mut row = vec![[0u8; 3]; info.width as usize];
        for y in 0..info.height {
            self.panel.read_row(y, &mut row);
            for (x, pixel) in row.iter().enumerate() {
                dst.put(x as u32, y, *pixel);
            }
        }
        dst.set_serial(serial);
        serial
    }
}

/// The interception that gets a host an `Arc<dyn Panel>` out of a described
/// machine. See the module docs: a seam, not a design.
pub mod capture {
    use alloc::sync::Arc;

    use super::{Panel, PanelScanout};
    use crate::core::error::Result;
    use crate::core::hosts::{Captured, HostKind, HostObjects};
    use crate::dev::lcd::panel::PanelClass;
    use crate::machine::BuildOptions;

    /// The name every panel class captures under.
    ///
    /// One table rather than one per class: the host is asking "what is this
    /// board's screen", and which silicon answered is the machine file's
    /// business.
    pub const CAPTURE_NAME: &str = "display.panel";

    /// Replace every compiled-in panel class's constructor in `options` with
    /// one that keeps a handle, leaving every other class alone.
    ///
    /// Safe to call on a build that has no panel in it: it installs the
    /// interceptions and [`take`] answers `None`.
    ///
    /// # Errors
    ///
    /// [`crate::Error::Config`] if something else has already claimed this
    /// build's capture table under [`CAPTURE_NAME`].
    pub fn install(options: &mut BuildOptions) -> Result<()> {
        let seen: Arc<Captured<dyn Panel>> =
            options
                .realize
                .hosts
                .open(HostKind::CAPTURE, CAPTURE_NAME, Captured::new)?;
        for class in classes() {
            install_one(options, &seen, class);
        }
        Ok(())
    }

    /// The panel this build constructed.
    ///
    /// The most recently constructed one, for a board with several. `None` if
    /// this build has no panel in it — a machine with no picture, which a host
    /// must be able to render nothing for.
    #[must_use]
    pub fn take(hosts: &HostObjects) -> Option<PanelScanout> {
        let seen = hosts
            .get::<Captured<dyn Panel>>(HostKind::CAPTURE, CAPTURE_NAME)
            .ok()
            .flatten()?;
        Some(PanelScanout::new(seen.take()?))
    }

    /// Every device class in this build that owns a framebuffer.
    ///
    /// **The union site for the family.** A new smart panel adds one line here
    /// and needs no new host module.
    #[must_use]
    pub fn classes() -> alloc::vec::Vec<PanelClass> {
        alloc::vec![
            #[cfg(feature = "dev-ssd1306")]
            crate::dev::solomon::ssd1306::SSD1306_PANEL,
            #[cfg(feature = "dev-st77xx")]
            crate::dev::sitronix::st77xx::ST77XX_PANEL,
        ]
    }

    /// Point one class's constructor at the capture table.
    fn install_one(options: &mut BuildOptions, seen: &Arc<Captured<dyn Panel>>, class: PanelClass) {
        let seen = Arc::clone(seen);
        options.bindings.replace(class.name, move |props| {
            let built = (class.construct)(props)?;
            seen.push(&built.panel);
            Ok(built.instance)
        });
    }
}
