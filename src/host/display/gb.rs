//! The Game Boy side of the scanout seam: a DMG's LCD as host pixels.
//!
//! [`GbScanout`] holds an `Arc<GbPpu>` and does the one thing the controller
//! deliberately refuses to do — turn its output into a colour. The chip emits a
//! **two-bit shade**, already through `BGP`/`OBP0`/`OBP1`, and there its job
//! ends: a DMG has no colour hardware of any kind, no DAC and no palette RAM,
//! so what those four levels look like is a property of the panel in front of
//! them rather than of the silicon behind them.
//!
//! # Why the four levels are grey
//!
//! Shade `0` is the lightest and `3` the darkest (Pan Docs, "LCD Monochrome
//! Palettes"), and the four levels are evenly spaced, so `255`, `170`, `85`,
//! `0` — `(3 - shade) * 85`, exactly the arithmetic
//! [`sms_rgb`](super::sms::sms_rgb) does for the VDP's resistor ladder — is the
//! plain rendering of what the controller emits.
//!
//! An original DMG's reflective STN panel is famously *green*, a Pocket's is
//! not, and a Super Game Boy tints the whole picture from a table the cartridge
//! chooses. Those are three different pieces of glass in front of one unchanged
//! chip. Picking one of them here would be inventing hardware for every host at
//! once, so this adapter emits the greyscale and a front end that wants a tint
//! applies one — the same division of labour that keeps
//! [`palette`](super::palette) honest about there being no correct NES palette.
//!
//! # Getting hold of the controller
//!
//! A machine built from a description hands back `Arc<dyn Device>`, and there is
//! no route from a `dyn Device` to `Arc<GbPpu>` — `Device` has no `Any` in its
//! supertrait chain, on purpose (`machine::realize` explains why). So the host
//! takes its handle at the only moment the concrete type exists: device
//! construction. [`capture::install`] replaces `gb.ppu`'s constructor with one
//! that keeps a clone in **this build's** capture table before handing the
//! device on:
//!
//! ```text
//! let mut options = catalog::build_options()?;
//! display::gb::capture::install(&mut options)?;         // intercept gb.ppu
//! let machine = machine::build(name, source, &registry, &options)?;
//! let scanout = display::gb::capture::take(&options.realize.hosts);
//! ```
//!
//! **This is a seam, and it is marked as one**, exactly like
//! [`nes::capture`](super::nes::capture) and
//! [`sms::capture`](super::sms::capture). It exists because `Device` has no
//! scanout hook yet; when it grows one, every line of [`capture`] deletes and
//! nothing else here changes. The table belongs to the build rather than to the
//! process, so two consoles built in one process do not swap screens.
//!
//! The buttons are **not** here. They travel through the record/replay input
//! seam ([`pads`](crate::dev::gb::joypad::pads), channel `pad:gb-joypad`),
//! which a front end opens by name; this module is output only.

use alloc::sync::Arc;

use super::{PixelFormat, Scanout, Surface, SurfaceInfo};
use crate::dev::gb::ppu::{DOTS_PER_FRAME, GbPpu, SCREEN_HEIGHT, SCREEN_WIDTH};

/// One two-bit shade as host RGB: `0` white, `3` black, evenly spaced.
///
/// A value above `3` cannot come out of the controller — the framebuffer holds
/// palette output, which is two bits wide — and is clamped to black rather than
/// wrapping, for the same reason [`Surface::put`] ignores a pixel off the edge:
/// a host must not take the process down in the middle of a frame.
#[must_use]
pub const fn gb_rgb(shade: u8) -> [u8; 3] {
    let clamped = if shade > 3 { 3 } else { shade };
    let level = 255 - clamped * 85;
    [level, level, level]
}

/// A [`Scanout`] over a Game Boy LCD controller.
#[derive(Debug, Clone)]
pub struct GbScanout {
    ppu: Arc<GbPpu>,
}

impl GbScanout {
    /// Watch `ppu`.
    #[must_use]
    pub fn new(ppu: Arc<GbPpu>) -> GbScanout {
        GbScanout { ppu }
    }

    /// The chip being watched, for a host that wants its registers too.
    #[must_use]
    pub fn ppu(&self) -> &Arc<GbPpu> {
        &self.ppu
    }
}

impl Scanout for GbScanout {
    fn info(&self) -> SurfaceInfo {
        // Always 160x144. Unlike the Master System's VDP there is no mode that
        // changes the height: `LCDC.7` turns the panel off, and a blank screen
        // is the controller holding shade 0 rather than a different geometry.
        SurfaceInfo::new(
            SCREEN_WIDTH as u32,
            SCREEN_HEIGHT as u32,
            PixelFormat::RGBA8888,
        )
    }

    fn frame_counter(&self) -> u64 {
        self.ppu.frame()
    }

    fn frame_period_ns(&self) -> u64 {
        // 70224 dots x (1e9 / 4194304 Hz) = 16 742 706 ns, i.e. 59.727 Hz. One
        // dot is one period of the console's only crystal —
        // `machines/gameboy.machine` clocks `gb.ppu` at `master` for exactly
        // that reason — so this is exact integer arithmetic over the oscillator
        // forest and never a wall-clock measurement (`CLAUDE.md`,
        // determinism). It paces a host; it never drives guest time.
        DOTS_PER_FRAME.saturating_mul(1_000_000_000) / crate::dev::gb::MASTER_HZ
    }

    fn capture(&self, dst: &mut Surface) -> u64 {
        let info = self.info();
        dst.reshape(dst.format(), info.width, info.height);

        // The counter is read before the pixels: if the emulation thread is
        // mid-frame the surface may hold the frame *after* this one, and a
        // serial that is never ahead of its pixels is the safe direction to err
        // — a host redraws once more, rather than never.
        let serial = self.ppu.frame();
        self.ppu.with_framebuffer(|fb| {
            for y in 0..info.height {
                let row = (y as usize) * SCREEN_WIDTH;
                for x in 0..info.width {
                    // `put`, rather than a byte layout of our own: it is where
                    // format conversion lives, so a host asking for `BGRA8888`
                    // — or the wasm module, which pins `RGBA8888` whatever
                    // `info` prefers — gets what it asked for.
                    dst.put(x, y, gb_rgb(fb[row + x as usize]));
                }
            }
        });
        dst.set_serial(serial);
        serial
    }
}

/// The interception that gets a host an `Arc<GbPpu>` out of a described
/// machine. See the module docs: a seam, not a design.
pub mod capture {
    use super::{Arc, GbPpu, GbScanout};
    use crate::core::error::Result;
    use crate::core::hosts::{Captured, HostKind, HostObjects};
    use crate::dev::gb::ppu::CLASS;
    use crate::machine::BuildOptions;

    /// Replace `gb.ppu`'s constructor in `options` with one that keeps a
    /// handle, leaving every other class alone.
    ///
    /// The one call a host makes between [`catalog::build_options`] and
    /// [`machine::build`]. Binding the class in a machine that does not use it
    /// costs nothing, so there is no "was it already bound?" case to get wrong.
    ///
    /// [`catalog::build_options`]: crate::machine::catalog::build_options
    /// [`machine::build`]: crate::machine::build
    ///
    /// # Errors
    ///
    /// [`crate::Error::Config`] if something else has already claimed this
    /// build's capture table, which would be a name collision between two host
    /// modules rather than anything a machine file can cause.
    pub fn install(options: &mut BuildOptions) -> Result<()> {
        let seen: Arc<Captured<GbPpu>> =
            options
                .realize
                .hosts
                .open(HostKind::CAPTURE, CLASS.name, Captured::new)?;
        options.bindings.replace(CLASS.name, move |props| {
            let ppu = Arc::new(GbPpu::from_props(props)?);
            seen.push(&ppu);
            Ok(ppu)
        });
        Ok(())
    }

    /// The controller this build constructed, as a [`GbScanout`].
    ///
    /// The most recent one, for a build with more than one. `None` if this
    /// build has no LCD controller in it — a machine with no picture, which a
    /// host must be able to render nothing for.
    #[must_use]
    pub fn take(hosts: &HostObjects) -> Option<GbScanout> {
        let seen = hosts
            .get::<Captured<GbPpu>>(HostKind::CAPTURE, CLASS.name)
            .ok()
            .flatten()?;
        seen.take().map(GbScanout::new)
    }
}
