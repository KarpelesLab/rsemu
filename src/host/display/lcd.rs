//! The display-controller side of the scanout seam.
//!
//! [`LcdScanout`] holds an `Arc<Scanout>` — the generic RGB engine of
//! [`crate::dev::lcd::scanout`] — and presents it as a [`Scanout`] a host can
//! capture. There is no colour conversion to do here, unlike
//! [`nes`](super::nes): the engine already reads the guest's framebuffer as
//! RGB888, and [`PixelFormat::RGB888`] is a format the seam already has, so a
//! [`Surface`] is filled straight from it.
//!
//! # Getting hold of the engine
//!
//! Exactly as for the NES, and for the same reason: a machine built from a
//! description hands back `Arc<dyn Device>` and there is no route from that to
//! `Arc<Scanout>`, because `Device` has no `Any` in its supertrait chain on
//! purpose. So the host takes its handle at the only moment the concrete type
//! exists — device construction — through [`capture::install`]:
//!
//! ```text
//! let mut options = catalog::build_options()?;
//! display::lcd::capture::install(&mut options)?;   // intercept lcd.scanout
//! let machine = machine::build(name, source, &registry, &options)?;
//! let picture = display::lcd::capture::take(&options.realize.hosts, &machine);
//! ```
//!
//! [`capture::take`] takes the machine as well, because the frame *rate* is a
//! property of the clock forest rather than of the device: a device cannot
//! reach the forest from `&self`, so the host resolves the domain's exact
//! rational frequency once and hands it in. That is the same seam
//! `riscv-virt`'s `timebase` names in its machine file, closed here instead of
//! written twice.
//!
//! This is a seam and it is marked as one. When `Device` grows a scanout hook,
//! every line of [`capture`] deletes. The capture table belongs to the build
//! rather than to the process, so two panels built in one process do not swap
//! pictures.

use alloc::string::String;
use alloc::sync::Arc;
use alloc::vec;

use super::{PixelFormat, Scanout as HostScanout, Surface, SurfaceInfo};
use crate::dev::lcd::scanout::Scanout;

/// A [`HostScanout`] over a generic RGB scanout engine.
#[derive(Debug, Clone)]
pub struct LcdScanout {
    engine: Arc<Scanout>,
}

impl LcdScanout {
    /// Watch `engine`.
    #[must_use]
    pub fn new(engine: Arc<Scanout>) -> LcdScanout {
        LcdScanout { engine }
    }

    /// The engine being watched, for a host that wants its registers too.
    #[must_use]
    pub fn engine(&self) -> &Arc<Scanout> {
        &self.engine
    }
}

impl HostScanout for LcdScanout {
    fn info(&self) -> SurfaceInfo {
        let (width, height) = self.engine.geometry();
        // RGB888 rather than RGBA8888: the engine hands out RGB triples and the
        // seam has a format for exactly that, so nothing pads and nothing
        // converts. The NES adapter uses RGBA because a browser canvas wanted
        // it, not because the seam prefers it.
        SurfaceInfo::new(width, height, PixelFormat::RGB888)
    }

    fn frame_counter(&self) -> u64 {
        self.engine.frame()
    }

    fn frame_period_ns(&self) -> u64 {
        self.engine.frame_period_nanos()
    }

    fn capture(&self, dst: &mut Surface) -> u64 {
        let info = self.info();
        dst.reshape(dst.format(), info.width, info.height);

        // The counter before the pixels: if the emulation thread is mid-frame
        // the surface may hold the frame *after* this one, and a serial that is
        // never ahead of its pixels is the safe direction to err.
        let serial = self.engine.frame();
        let mut row = vec![[0u8; 3]; info.width as usize];
        for y in 0..info.height {
            self.engine.read_row(y, &mut row);
            for (x, pixel) in row.iter().enumerate() {
                dst.put(x as u32, y, *pixel);
            }
        }
        dst.set_serial(serial);
        serial
    }
}

/// The interception that gets a host an `Arc<Scanout>` out of a described
/// machine. See the module docs: a seam, not a design.
pub mod capture {
    use super::{Arc, LcdScanout, Scanout, String};
    use crate::core::error::Result;
    use crate::core::hosts::{Captured, HostKind, HostObjects};
    use crate::dev::lcd::scanout::{SCANOUT_CLASS, set_frame_rate};
    use crate::machine::{BuildOptions, Machine};

    /// Replace `lcd.scanout`'s constructor in `options` with one that keeps a
    /// handle, leaving every other class alone.
    ///
    /// # Errors
    ///
    /// [`crate::Error::Config`] if something else has already claimed this
    /// build's capture table.
    pub fn install(options: &mut BuildOptions) -> Result<()> {
        let seen: Arc<Captured<Scanout>> =
            options
                .realize
                .hosts
                .open(HostKind::CAPTURE, SCANOUT_CLASS.name, Captured::new)?;
        options.bindings.replace(SCANOUT_CLASS.name, move |props| {
            let engine = Arc::new(Scanout::new(props)?);
            seen.push(&engine);
            Ok(engine)
        });
        Ok(())
    }

    /// The engine this build constructed, with its frame period resolved from
    /// `machine`'s clock forest.
    ///
    /// The most recent one, for a machine with several. `None` if this build has
    /// no scanout engine in it.
    #[must_use]
    pub fn take(hosts: &HostObjects, machine: &Machine) -> Option<LcdScanout> {
        let seen = hosts
            .get::<Captured<Scanout>>(HostKind::CAPTURE, SCANOUT_CLASS.name)
            .ok()
            .flatten()?;
        let engine = seen.take()?;
        resolve_rate(machine, &engine);
        Some(LcdScanout::new(engine))
    }

    /// Find this engine's clock domain in the realized machine and hand it the
    /// domain's exact rational frequency.
    ///
    /// The rate is a fact about the oscillator forest, so it is read from the
    /// forest rather than written twice in a machine file
    /// (`CLAUDE.md`, determinism). A machine with several engines is matched by
    /// class, taking the last — which is the one [`take`] returned.
    fn resolve_rate(machine: &Machine, engine: &Arc<Scanout>) {
        let Some(entry) = machine
            .devices()
            .iter()
            .rev()
            .find(|d| d.class().name == SCANOUT_CLASS.name)
        else {
            return;
        };
        let Some(domain) = entry.domain() else {
            return;
        };
        if let Ok(freq) = machine.clocks().domain_frequency(domain) {
            set_frame_rate(engine, freq.num(), freq.den());
        }
    }

    /// The instance path convention this module documents, for a diagnostic.
    #[must_use]
    pub fn class_name() -> String {
        String::from(SCANOUT_CLASS.name)
    }
}
