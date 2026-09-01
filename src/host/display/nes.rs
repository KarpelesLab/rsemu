//! The NES side of the scanout seam: a 2C02's framebuffer as host pixels.
//!
//! [`NesScanout`] holds an `Arc<NesPpu>` and does the one thing the PPU
//! deliberately refuses to do — turn [`Pixel`](crate::dev::ppu::Pixel), a
//! palette index plus emphasis bits, into RGB. The colour table and the reason
//! it looks the way it does are in [`palette`](super::palette).
//!
//! # Getting hold of the PPU
//!
//! A machine built from a description hands back `Arc<dyn Device>`, and there
//! is no route from a `dyn Device` to `Arc<NesPpu>` — `Device` has no `Any` in
//! its supertrait chain, on purpose (`machine::realize` explains why). So the
//! host takes its handle at the only moment the concrete type exists: device
//! construction. [`capture::install`] replaces `nes.ppu`'s constructor with one
//! that keeps a clone in **this build's** capture table before handing the
//! device on:
//!
//! ```text
//! let mut options = catalog::build_options()?;
//! display::nes::capture::install(&mut options)?;        // intercept nes.ppu
//! let machine = machine::build(name, source, &registry, &options)?;
//! let scanout = display::nes::capture::take(&options.realize.hosts);
//! ```
//!
//! **This is a seam, and it is marked as one.** It exists because `Device` has
//! no scanout hook yet; when it grows one — the obvious shape is a defaulted
//! `fn scanout(&self) -> Option<Arc<dyn Scanout>>` beside `Device::region` —
//! every line of [`capture`] deletes and nothing else here changes.
//!
//! It is not, however, process-wide any more. The table lives in the build's
//! [`HostObjects`](crate::core::hosts::HostObjects), so two machines built in
//! one process capture into two tables and neither can take the other's chip.

use alloc::sync::Arc;

use super::palette::nes_rgb;
use super::{PixelFormat, Scanout, Surface, SurfaceInfo};
use crate::dev::ppu::{NesPpu, SCREEN_HEIGHT, SCREEN_WIDTH};

/// A [`Scanout`] over a NES PPU.
#[derive(Debug, Clone)]
pub struct NesScanout {
    ppu: Arc<NesPpu>,
}

impl NesScanout {
    /// Watch `ppu`.
    #[must_use]
    pub fn new(ppu: Arc<NesPpu>) -> NesScanout {
        NesScanout { ppu }
    }

    /// The chip being watched, for a host that wants its registers too.
    #[must_use]
    pub fn ppu(&self) -> &Arc<NesPpu> {
        &self.ppu
    }
}

impl Scanout for NesScanout {
    fn info(&self) -> SurfaceInfo {
        // Always the full 256x240 the chip renders, on every region. PAL and
        // Dendy blank the top rendered line rather than dropping it — the
        // engine's own output stage paints it `BORDER_BLACK`, so the picture
        // keeps its shape and this side needs no special case.
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
        let region = self.ppu.tv_region();
        let (hz_num, hz_den) = region.master_clock();
        let dots = self.ppu.geometry().dots_per_frame;
        // dots × master-ticks-per-dot × (1e9 / master Hz), in exact integer
        // arithmetic — the frame period is a fact about the oscillator forest,
        // never a wall-clock measurement (`CLAUDE.md`, determinism). NTSC's
        // odd-frame skipped dot is not modelled here: it is 6 parts per
        // million, and this number only paces a host, it never drives time.
        dots.saturating_mul(region.dot_divider())
            .saturating_mul(hz_den)
            .saturating_mul(1_000_000_000)
            / hz_num
    }

    fn capture(&self, dst: &mut Surface) -> u64 {
        let info = self.info();
        dst.reshape(dst.format(), info.width, info.height);

        // The counter is read before the pixels: if the emulation thread is
        // mid-frame the surface may hold the frame *after* this one, and a
        // serial that is never ahead of its pixels is the safe direction to
        // err — a host redraws once more, rather than never.
        let serial = self.ppu.frame();
        self.ppu.with_framebuffer(|fb| {
            for y in 0..info.height {
                let row = (y as usize) * SCREEN_WIDTH;
                for x in 0..info.width {
                    let pixel = fb[row + x as usize];
                    dst.put(x, y, nes_rgb(pixel.0));
                }
            }
        });
        dst.set_serial(serial);
        serial
    }
}

/// The interception that gets a host an `Arc<NesPpu>` out of a described
/// machine. See the module docs: a seam, not a design.
pub mod capture {
    use super::{Arc, NesPpu, NesScanout};
    use crate::core::error::Result;
    use crate::core::hosts::{Captured, HostKind, HostObjects};
    use crate::dev::ppu::NES_PPU_CLASS;
    use crate::machine::BuildOptions;

    /// Replace `nes.ppu`'s constructor in `options` with one that keeps a
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
        let seen: Arc<Captured<NesPpu>> =
            options
                .realize
                .hosts
                .open(HostKind::CAPTURE, NES_PPU_CLASS.name, Captured::new)?;
        options.bindings.replace(NES_PPU_CLASS.name, move |props| {
            let ppu = Arc::new(NesPpu::new(props)?);
            seen.push(&ppu);
            Ok(ppu)
        });
        Ok(())
    }

    /// The PPU this build constructed, as a [`NesScanout`].
    ///
    /// The most recent one, for a machine with more than one. `None` if this
    /// build has no PPU in it — a machine with no picture, which a host must be
    /// able to render nothing for.
    #[must_use]
    pub fn take(hosts: &HostObjects) -> Option<NesScanout> {
        let seen = hosts
            .get::<Captured<NesPpu>>(HostKind::CAPTURE, NES_PPU_CLASS.name)
            .ok()
            .flatten()?;
        seen.take().map(NesScanout::new)
    }
}
