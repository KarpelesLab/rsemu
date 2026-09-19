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
//! # Following the guest's mode
//!
//! A VGA's geometry is not a property of the board: the guest sets a mode and
//! the picture becomes 640 x 480, or 720 x 400, or whatever the registers now
//! say. So a host must not cache [`SurfaceInfo`](super::SurfaceInfo) — every
//! [`Scanout::capture`](super::Scanout::capture) reshapes the surface it is
//! given, and `rsemu run --screenshot`, the APNG recorder and the browser all
//! read the size back off the surface afterwards.
//!
//! The **frame period** follows the same change, and that is why
//! [`capture::take`] takes the realized machine: a frame is
//! `ticks_per_frame` ticks of the adapter's own clock domain, and the device
//! cannot reach the clock forest from `&self` (the seam
//! [`amiga`](super::amiga) and [`lcd`](super::lcd) already have). With the
//! domain's exact rational frequency the period is exact arithmetic from the
//! CRT controller's registers, and a mode set moves it the instant the guest
//! writes the last register: 70 Hz in the 720 x 400 text mode, 60 Hz in a
//! 640 x 480 one, with no wall clock anywhere near it (`CLAUDE.md`,
//! determinism).
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
    use crate::machine::{BuildOptions, Machine};

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
    ///
    /// The frame period this reports comes from the device's `dot-clock`
    /// property. A host that has the realized machine should call
    /// [`take_clocked`] instead, which resolves the character clock's exact
    /// rational frequency out of the clock forest — see the module docs.
    #[must_use]
    pub fn take(hosts: &HostObjects) -> Option<VideoScanout> {
        let seen = hosts
            .get::<Captured<Video>>(HostKind::CAPTURE, CLASS_NAME)
            .ok()
            .flatten()?;
        seen.take().map(|video| video.scanout())
    }

    /// The same, with the adapter's clock domain resolved out of `machine`.
    ///
    /// What `rsemu run` and the browser use: the period then follows every
    /// mode the guest sets, exactly.
    #[must_use]
    pub fn take_clocked(hosts: &HostObjects, machine: &Machine) -> Option<VideoScanout> {
        let scanout = take(hosts)?;
        let rate = machine
            .devices()
            .iter()
            .rev()
            .find(|d| d.class().name == CLASS_NAME)
            .and_then(|d| d.domain())
            .and_then(|domain| machine.clocks().domain_frequency(domain).ok())
            .map(|f| (f.num(), f.den()));
        Some(scanout.with_rate(rate))
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

    /// A VGA whose guest changes the mode: the host follows both the shape and
    /// the rate, and the rate is the clock forest's own answer.
    #[test]
    fn the_host_follows_a_mode_change_in_shape_and_in_period() {
        use crate::core::space::MemAttrs;
        use crate::core::value::Width;

        let mut options = crate::machine::BuildOptions::new()
            .with_classes(crate::machine::catalog::classes())
            .with_bindings(crate::machine::catalog::bindings().expect("this build's bindings"));
        capture::install(&mut options).expect("the capture table is this build's");
        let registry = crate::machine::catalog::registry().expect("this build's registry");
        let machine = crate::machine::build(
            "vga-adapter.machine",
            r#"
            machine "vga-adapter" {
              osc dot = 28322000 Hz
              space port { width = 16, unassigned = open-bus }
              space mem  { width = 32, unassigned = read-as-ones }
              object vga "pc.video" {
                clock = dot / 9, dot-clock = 28322000, model = "vga"
              }
              map port 0x03c0 size 0x0010 = vga.vga
              map port 0x03d4 size 0x0002 = vga.crtc-colour
              map mem 0x000a0000 size 0x20000 = vga.window
            }
            "#,
            &registry,
            &options,
        )
        .expect("a machine with nothing but a VGA");

        let scanout = capture::take_clocked(&options.realize.hosts, &machine)
            .expect("the constructor kept a handle");
        let info = scanout.info();
        assert_eq!((info.width, info.height), (720, 400), "mode 03h");
        // 100 character clocks by 449 lines at 28.322 MHz over nine.
        assert_eq!(
            scanout.frame_period_ns(),
            100 * 449 * 9_000_000_000 / 28_322_000
        );

        // What a mode set does: a wider line, more of them, eight-dot cells
        // and one scan line a row. Nothing here names a mode number.
        let port = machine.space("port").expect("the I/O space");
        let crtc = |index: u8, value: u8| {
            port.write(0x3d4, Width::U8, u64::from(index), MemAttrs::DEFAULT)
                .expect("the index register");
            port.write(0x3d5, Width::U8, u64::from(value), MemAttrs::DEFAULT)
                .expect("the data register");
        };
        crtc(0x11, 0x00); // unprotect 00h-07h
        crtc(0x00, 0x5f); // 100 character clocks
        crtc(0x01, 0x4f); // 80 displayed: 640 dots at eight a cell
        crtc(0x06, 0x0b); // vertical total 525
        crtc(0x07, 0x3e);
        crtc(0x09, 0x40); // one scan line a row
        crtc(0x12, 0xdf); // 480 displayed
        port.write(0x3c4, Width::U8, 0x01, MemAttrs::DEFAULT)
            .expect("the sequencer index");
        port.write(0x3c5, Width::U8, 0x01, MemAttrs::DEFAULT)
            .expect("eight dots a character clock");

        let info = scanout.info();
        assert_eq!((info.width, info.height), (640, 480), "the registers moved");
        assert_eq!(
            scanout.frame_period_ns(),
            100 * 525 * 9_000_000_000 / 28_322_000,
            "and so did the period: 525 lines at the same line rate"
        );
        let mut surface = Surface::new(crate::host::display::PixelFormat::RGBA8888, 1, 1);
        scanout.capture(&mut surface);
        assert_eq!(
            (surface.width(), surface.height()),
            (640, 480),
            "a capture reshapes the host's surface to the new mode"
        );
    }
}
