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
//! construction. [`capture::install`] wraps the machine's bindings so that
//! `nes.ppu` is built by a constructor here, which keeps a clone before handing
//! the device on:
//!
//! ```text
//! let mut options = catalog::build_options()?;
//! display::nes::capture::install(&mut options)?;   // intercept nes.ppu
//! let machine = machine::build(name, source, &registry, &options)?;
//! let scanout = display::nes::capture::take();     // the Arc it kept
//! ```
//!
//! **This is a seam, and it is marked as one**, exactly like the process-wide
//! table in [`chardev::ports`](crate::host::chardev::ports). It exists because
//! `Device` has no scanout hook yet; when it grows one — the obvious shape is a
//! defaulted `fn scanout(&self) -> Option<Arc<dyn Scanout>>` beside
//! `Device::region` — every line of [`capture`] deletes and nothing else here
//! changes. Until then the table is process-wide, so build one machine at a
//! time or [`capture::clear`] between them.

use alloc::sync::Arc;
use alloc::vec::Vec;

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
    use super::{Arc, NesPpu, NesScanout, Vec};
    use crate::core::error::Result;
    use crate::core::props::Props;
    use crate::core::sync::{Global, LockRank};
    use crate::dev::ppu::NES_PPU_CLASS;
    use crate::machine::realize::Instance;
    use crate::machine::{Bindings, BuildOptions};

    /// Every PPU constructed since the last [`take`] or [`clear`], oldest
    /// first. A `Vec` rather than a single slot because a machine with two
    /// PPUs is not this module's business to refuse.
    static CONSTRUCTED: Global<Vec<Arc<NesPpu>>> = Global::with_rank(LockRank::LEAF, Vec::new());

    /// Construct a PPU and keep a reference to it.
    ///
    /// An `InstanceCtor` is a bare `fn` that can capture nothing, which is why
    /// the table above is a static rather than a field.
    fn construct(props: &Props) -> Result<Arc<dyn Instance>> {
        let ppu = Arc::new(NesPpu::new(props)?);
        CONSTRUCTED.lock().push(Arc::clone(&ppu));
        Ok(ppu)
    }

    /// Replace `nes.ppu`'s constructor in `bindings` with one that keeps a
    /// handle, leaving every other class alone.
    ///
    /// # Errors
    ///
    /// [`crate::Error::Config`] if a class turns out to be bound twice, which
    /// would be a bug in the caller's binding table rather than here.
    pub fn intercept(bindings: &Bindings) -> Result<Bindings> {
        let mut out = Bindings::new();
        let mut replaced = false;
        let classes: Vec<&'static str> = bindings.classes().collect();
        for class in classes {
            if class == NES_PPU_CLASS.name {
                out.bind(class, construct)?;
                replaced = true;
            } else if let Some(ctor) = bindings.get(class) {
                out.bind(class, ctor)?;
            }
        }
        if !replaced {
            // The PPU's own `bind` was never called — a build with the device
            // feature but a machine that does not use it. Binding it here is
            // still correct: an unused binding costs nothing.
            out.bind(NES_PPU_CLASS.name, construct)?;
        }
        Ok(out)
    }

    /// Point `options` at intercepted bindings, in place.
    ///
    /// The one call a host makes between [`catalog::build_options`] and
    /// [`machine::build`].
    ///
    /// [`catalog::build_options`]: crate::machine::catalog::build_options
    /// [`machine::build`]: crate::machine::build
    ///
    /// # Errors
    ///
    /// As [`intercept`].
    pub fn install(options: &mut BuildOptions) -> Result<()> {
        options.bindings = intercept(&options.bindings)?;
        Ok(())
    }

    /// Take the most recently constructed PPU as a [`NesScanout`], forgetting
    /// every earlier one.
    ///
    /// `None` if no machine with a PPU has been built since the last call — a
    /// machine with no picture, which a host must be able to render nothing
    /// for.
    #[must_use]
    pub fn take() -> Option<NesScanout> {
        let mut table = CONSTRUCTED.lock();
        let last = table.pop();
        table.clear();
        last.map(NesScanout::new)
    }

    /// Forget every kept handle, so the next [`take`] cannot return a PPU from
    /// a machine that has already been dropped.
    pub fn clear() {
        CONSTRUCTED.lock().clear();
    }
}
