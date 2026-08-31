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
//! let picture = display::lcd::capture::take(&machine);
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
//! every line of [`capture`] deletes.

use alloc::string::String;
use alloc::sync::Arc;
use alloc::vec;
use alloc::vec::Vec;

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
    use super::{Arc, LcdScanout, Scanout, String, Vec};
    use crate::core::error::Result;
    use crate::core::props::Props;
    use crate::core::sync::{LockRank, Mutex};
    use crate::dev::lcd::scanout::{SCANOUT_CLASS, set_frame_rate};
    use crate::machine::realize::Instance;
    use crate::machine::{Bindings, BuildOptions, Machine};

    /// Every engine constructed since the last [`take`] or [`clear`], oldest
    /// first, with the instance path it was built for.
    ///
    /// The path is kept because the frame rate is resolved from the machine's
    /// clock forest afterwards, and the forest is indexed by device.
    static CONSTRUCTED: Mutex<Vec<Arc<Scanout>>> = Mutex::with_rank(LockRank::LEAF, Vec::new());

    /// Construct an engine and keep a reference to it.
    fn construct(props: &Props) -> Result<Arc<dyn Instance>> {
        let engine = Arc::new(Scanout::new(props)?);
        CONSTRUCTED.lock().push(Arc::clone(&engine));
        Ok(engine)
    }

    /// Replace `lcd.scanout`'s constructor in `bindings` with one that keeps a
    /// handle, leaving every other class alone.
    ///
    /// # Errors
    ///
    /// [`crate::Error::Config`] if a class turns out to be bound twice.
    pub fn intercept(bindings: &Bindings) -> Result<Bindings> {
        let mut out = Bindings::new();
        let mut replaced = false;
        let classes: Vec<&'static str> = bindings.classes().collect();
        for class in classes {
            if class == SCANOUT_CLASS.name {
                out.bind(class, construct)?;
                replaced = true;
            } else if let Some(ctor) = bindings.get(class) {
                out.bind(class, ctor)?;
            }
        }
        if !replaced {
            out.bind(SCANOUT_CLASS.name, construct)?;
        }
        Ok(out)
    }

    /// Point `options` at intercepted bindings, in place.
    ///
    /// # Errors
    ///
    /// As [`intercept`].
    pub fn install(options: &mut BuildOptions) -> Result<()> {
        options.bindings = intercept(&options.bindings)?;
        Ok(())
    }

    /// Take the most recently constructed engine, forgetting every earlier one,
    /// and resolve its frame period from `machine`'s clock forest.
    ///
    /// `None` if no machine with a scanout engine has been built since the last
    /// call.
    #[must_use]
    pub fn take(machine: &Machine) -> Option<LcdScanout> {
        let engine = {
            let mut table = CONSTRUCTED.lock();
            let last = table.pop();
            table.clear();
            last?
        };
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

    /// Forget every kept handle.
    pub fn clear() {
        CONSTRUCTED.lock().clear();
    }

    /// The instance path convention this module documents, for a diagnostic.
    #[must_use]
    pub fn class_name() -> String {
        String::from(SCANOUT_CLASS.name)
    }
}
