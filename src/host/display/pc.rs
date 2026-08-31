//! The PC side of the scanout seam.
//!
//! [`crate::dev::pc::video::VideoScanout`] already does the work — the colour
//! chain from a text attribute through the attribute controller's palette and
//! the DAC is guest state, so it lives with the device. What is missing, and
//! what this module supplies, is the way a **host** gets hold of one.
//!
//! # Getting hold of the adapter
//!
//! A machine built from a description hands back `Arc<dyn Device>`, and there is
//! no route from a `dyn Device` to a concrete type — `Device` has no `Any` in
//! its supertrait chain, on purpose. So the host takes its handle at the only
//! moment the concrete type exists: device construction. [`capture::install`]
//! wraps the machine's bindings so that `pc.video` is built by a constructor
//! here, which keeps a clone before handing the device on.
//!
//! ```text
//! let mut options = catalog::build_options()?;
//! display::pc::capture::install(&mut options)?;   // intercept pc.video
//! let machine = machine::build(name, source, &registry, &options)?;
//! let scanout = display::pc::capture::take();     // the handle it kept
//! ```
//!
//! **This is a seam, and it is marked as one** — the same seam
//! [`nes`](super::nes) uses, with the same note attached: it exists because
//! `Device` has no scanout hook yet. When it grows one (the obvious shape is a
//! defaulted `fn scanout(&self) -> Option<Arc<dyn Scanout>>` beside
//! `Device::region`), both copies delete. Until then the table is process-wide,
//! so build one machine at a time or [`capture::clear`] between them.

/// The interception that gets a host a [`VideoScanout`] out of a described
/// machine.
///
/// [`VideoScanout`]: crate::dev::pc::video::VideoScanout
pub mod capture {
    use alloc::sync::Arc;
    use alloc::vec::Vec;

    use crate::core::error::Result;
    use crate::core::props::Props;
    use crate::core::sync::{Global, LockRank};
    use crate::dev::pc::video::{CLASS_NAME, Video, VideoScanout};
    use crate::machine::realize::Instance;
    use crate::machine::{Bindings, BuildOptions};

    /// Every adapter constructed since the last [`take`] or [`clear`], oldest
    /// first. A `Vec` rather than one slot because a machine with two display
    /// adapters — a PC really could have an MDA and a CGA at once — is not this
    /// module's business to refuse.
    static CONSTRUCTED: Global<Vec<Arc<Video>>> = Global::with_rank(LockRank::LEAF, Vec::new());

    /// Construct an adapter and keep a reference to it.
    ///
    /// An `InstanceCtor` is a bare `fn` that can capture nothing, which is why
    /// the table above is a static rather than a field.
    fn construct(props: &Props) -> Result<Arc<dyn Instance>> {
        let video = Arc::new(Video::new(props)?);
        CONSTRUCTED.lock().push(Arc::clone(&video));
        Ok(video)
    }

    /// Replace `pc.video`'s constructor in `bindings` with one that keeps a
    /// handle, leaving every other class alone.
    ///
    /// # Errors
    ///
    /// [`Error::Config`](crate::Error::Config) if a class turns out to be bound
    /// twice, which would be a bug in the caller's binding table rather than
    /// here.
    pub fn intercept(bindings: &Bindings) -> Result<Bindings> {
        let mut out = Bindings::new();
        let mut replaced = false;
        let classes: Vec<&'static str> = bindings.classes().collect();
        for class in classes {
            if class == CLASS_NAME {
                out.bind(class, construct)?;
                replaced = true;
            } else if let Some(ctor) = bindings.get(class) {
                out.bind(class, ctor)?;
            }
        }
        if !replaced {
            // A build with the feature but a machine that uses no display. An
            // unused binding costs nothing.
            out.bind(CLASS_NAME, construct)?;
        }
        Ok(out)
    }

    /// Point `options` at intercepted bindings, in place.
    ///
    /// The one call a host makes between `catalog::build_options` and
    /// `machine::build`.
    ///
    /// # Errors
    ///
    /// As [`intercept`].
    pub fn install(options: &mut BuildOptions) -> Result<()> {
        options.bindings = intercept(&options.bindings)?;
        Ok(())
    }

    /// Take the most recently constructed adapter as a [`VideoScanout`],
    /// forgetting every earlier one.
    ///
    /// `None` if no machine with a display has been built since the last call —
    /// a machine with no picture, which a host must be able to render nothing
    /// for.
    #[must_use]
    pub fn take() -> Option<VideoScanout> {
        let mut table = CONSTRUCTED.lock();
        let last = table.pop();
        table.clear();
        last.map(|video| video.scanout())
    }

    /// Forget every kept handle, so the next [`take`] cannot return an adapter
    /// from a machine that has already been dropped.
    pub fn clear() {
        CONSTRUCTED.lock().clear();
    }
}

#[cfg(test)]
mod tests {
    use super::capture;
    use crate::host::display::{Scanout, Surface};

    #[test]
    fn an_intercepted_binding_hands_the_host_a_picture() {
        capture::clear();
        assert!(capture::take().is_none(), "nothing has been built");

        let mut options = crate::machine::BuildOptions::new()
            .with_classes(crate::machine::catalog::classes())
            .with_bindings(crate::machine::catalog::bindings().expect("this build's bindings"));
        capture::install(&mut options).expect("the class is bound once");
        let registry = crate::machine::catalog::registry().expect("this build's registry");
        let machine = crate::machine::build(
            "one-adapter.machine",
            r#"
            machine "one-adapter" {
              osc dot = 28322000 Hz
              space port { width = 16, unassigned = open-bus }
              object vga "pc.video" { clock = dot / 9 }
              map port 0x03d4 size 0x0002 = vga.crtc-colour
            }
            "#,
            &registry,
            &options,
        )
        .expect("a machine with nothing but a display");
        assert_eq!(machine.name(), "one-adapter");

        let scanout = capture::take().expect("the constructor kept a handle");
        let mut surface = Surface::for_scanout(&scanout);
        scanout.capture(&mut surface);
        let info = scanout.info();
        assert_eq!(surface.width(), info.width);
        assert_eq!(surface.height(), info.height);
        // 80 columns of 9-pixel cells is the 720-wide text mode a VGA comes out
        // of reset in, which is the shape a `--screenshot` should produce.
        assert_eq!(info.width, 720, "80 columns of nine pixels");
        capture::clear();
    }
}
