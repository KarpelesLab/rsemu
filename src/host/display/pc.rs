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
//! replaces `pc.video`'s constructor with one that keeps a clone in this
//! build's capture table before handing the device on.
//!
//! ```text
//! let mut options = catalog::build_options()?;
//! display::pc::capture::install(&mut options)?;        // intercept pc.video
//! let machine = machine::build(name, source, &registry, &options)?;
//! let scanout = display::pc::capture::take(&options.realize.hosts);
//! ```
//!
//! **This is a seam, and it is marked as one** — the same seam
//! [`nes`](super::nes) uses, with the same note attached: it exists because
//! `Device` has no scanout hook yet. When it grows one (the obvious shape is a
//! defaulted `fn scanout(&self) -> Option<Arc<dyn Scanout>>` beside
//! `Device::region`), both copies delete. The table is the *build's* rather
//! than the process's, so two PCs built in one process do not swap screens.

/// The interception that gets a host a [`VideoScanout`] out of a described
/// machine.
///
/// [`VideoScanout`]: crate::dev::pc::video::VideoScanout
pub mod capture {
    use alloc::sync::Arc;

    use crate::core::error::Result;
    use crate::core::hosts::{Captured, HostKind, HostObjects};
    use crate::dev::pc::video::{CLASS_NAME, Video, VideoScanout};
    use crate::machine::BuildOptions;

    /// Replace `pc.video`'s constructor in `options` with one that keeps a
    /// handle, leaving every other class alone.
    ///
    /// # Errors
    ///
    /// [`Error::Config`](crate::Error::Config) if something else has already
    /// claimed this build's capture table.
    pub fn install(options: &mut BuildOptions) -> Result<()> {
        let seen: Arc<Captured<Video>> =
            options
                .realize
                .hosts
                .open(HostKind::CAPTURE, CLASS_NAME, Captured::new)?;
        options.bindings.replace(CLASS_NAME, move |props| {
            let video = Arc::new(Video::new(props)?);
            seen.push(&video);
            Ok(video)
        });
        Ok(())
    }

    /// The adapter this build constructed, as a [`VideoScanout`].
    ///
    /// The most recent one, for a PC that really does have an MDA and a CGA at
    /// once. `None` if this build has no display in it.
    #[must_use]
    pub fn take(hosts: &HostObjects) -> Option<VideoScanout> {
        let seen = hosts
            .get::<Captured<Video>>(HostKind::CAPTURE, CLASS_NAME)
            .ok()
            .flatten()?;
        seen.take().map(|video| video.scanout())
    }
}

#[cfg(test)]
mod tests {
    use super::capture;
    use crate::host::display::{Scanout, Surface};

    #[test]
    fn an_intercepted_binding_hands_the_host_a_picture() {
        let mut options = crate::machine::BuildOptions::new()
            .with_classes(crate::machine::catalog::classes())
            .with_bindings(crate::machine::catalog::bindings().expect("this build's bindings"));
        assert!(
            capture::take(&options.realize.hosts).is_none(),
            "nothing has been built"
        );
        capture::install(&mut options).expect("the capture table is this build's");
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

        let scanout = capture::take(&options.realize.hosts).expect("the constructor kept a handle");
        let mut surface = Surface::for_scanout(&scanout);
        scanout.capture(&mut surface);
        let info = scanout.info();
        assert_eq!(surface.width(), info.width);
        assert_eq!(surface.height(), info.height);
        // 80 columns of 9-pixel cells is the 720-wide text mode a VGA comes out
        // of reset in, which is the shape a `--screenshot` should produce.
        assert_eq!(info.width, 720, "80 columns of nine pixels");
    }
}
