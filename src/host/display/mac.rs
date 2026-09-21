//! The Macintosh side of the scanout seam: one-bit pixels out of main memory
//! as host pixels.
//!
//! [`MacScanout`] holds an `Arc<Screen>` and does the two things the video
//! circuit deliberately does not: turn its packed bits into host bytes, and
//! turn a frame's length into a frame period.
//!
//! **A one bit is black.** A Macintosh's screen is white and the bits are ink,
//! which is the opposite of nearly every other framebuffer in this tree, and
//! getting it the wrong way round gives a picture that is recognisable and
//! entirely inverted. The leftmost pixel of a byte is its most significant
//! bit.
//!
//! # The frame period
//!
//! 370 lines of 704 dot clocks is 260,480 ticks of the video circuit's clock
//! domain, which is the dot clock itself. Converted to virtual nanoseconds
//! with the domain's exact rational frequency (`CLAUDE.md`, determinism) that
//! is 16,625,816 ns — 60.147 Hz — on a machine running its stock 15.6672 MHz
//! crystal. A board that gives the circuit no `clock` reports `0`.
//!
//! # Getting hold of the circuit
//!
//! As for every other adapter here, and for the same reason: `Device` has no
//! `Any` in its supertrait chain, so the host keeps its handle at
//! construction.
//!
//! ```text
//! let mut options = catalog::build_options()?;
//! display::mac::capture::install(&mut options)?;      // intercept mac.video
//! let machine = machine::build(name, source, &registry, &options)?;
//! let scanout = display::mac::capture::take(&options.realize.hosts, &machine);
//! ```
//!
//! This is a seam and it is marked as one. When `Device` grows a scanout hook,
//! every line of [`capture`] deletes.

use alloc::sync::Arc;
use alloc::vec::Vec;

use super::{PixelFormat, Scanout, Surface, SurfaceInfo};
use crate::dev::mac::video::{ROW_BYTES, Screen};

/// White, as the phosphor is with no ink on it.
const WHITE: [u8; 3] = [0xff, 0xff, 0xff];
/// Black, which is what a set bit paints.
const BLACK: [u8; 3] = [0x00, 0x00, 0x00];

/// A [`Scanout`] over a Macintosh video circuit.
#[derive(Debug, Clone)]
pub struct MacScanout {
    screen: Arc<Screen>,
    /// The frequency of the circuit's `clock` domain as `(numerator,
    /// denominator)` hertz, or `None` if the board gave it none.
    rate: Option<(u64, u64)>,
}

impl MacScanout {
    /// Watch `screen`, with the frequency of its dot clock if there is one.
    #[must_use]
    pub fn new(screen: Arc<Screen>, rate: Option<(u64, u64)>) -> MacScanout {
        MacScanout { screen, rate }
    }

    /// The circuit being watched.
    #[must_use]
    pub fn screen(&self) -> &Arc<Screen> {
        &self.screen
    }
}

impl Scanout for MacScanout {
    fn info(&self) -> SurfaceInfo {
        let (width, height) = self.screen.geometry();
        SurfaceInfo::new(width, height, PixelFormat::RGB888)
    }

    fn frame_counter(&self) -> u64 {
        self.screen.frames()
    }

    fn frame_period_ns(&self) -> u64 {
        let Some((num, den)) = self.rate else {
            return 0;
        };
        if num == 0 {
            return 0;
        }
        // u128 because a frame is 260,480 ticks and the denominator and 10⁹
        // multiply it further.
        let ticks = u128::from(self.screen.frame_ticks());
        let ns = ticks * u128::from(den) * 1_000_000_000 / u128::from(num);
        u64::try_from(ns).unwrap_or(u64::MAX)
    }

    fn capture(&self, dst: &mut Surface) -> u64 {
        // The bits, the size and the frame count in one call, for the reason
        // every adapter here gives: a serial taken after the pixels can belong
        // to a later frame than they do, and a serial never ahead of its
        // pixels errs toward one extra redraw rather than a missed one.
        let mut bits = Vec::new();
        let (width, height, serial) = self.screen.copy_frame(&mut bits);
        dst.reshape(dst.format(), width, height);
        for y in 0..height {
            let row = (u64::from(y) * ROW_BYTES) as usize;
            for x in 0..width {
                let byte = bits.get(row + (x / 8) as usize).copied().unwrap_or(0);
                let ink = byte & (0x80 >> (x % 8)) != 0;
                dst.put(x, y, if ink { BLACK } else { WHITE });
            }
        }
        dst.set_serial(serial);
        serial
    }
}

/// The interception that gets a host an `Arc<Screen>` out of a described
/// machine. See the module docs: a seam, not a design.
pub mod capture {
    use alloc::sync::Arc;

    use super::MacScanout;
    use crate::core::error::Result;
    use crate::core::hosts::{Captured, HostKind, HostObjects};
    use crate::dev::mac::video::{CLASS_NAME, Screen, Video};
    use crate::machine::{BuildOptions, Machine};

    /// Replace `mac.video`'s constructor in `options` with one that keeps a
    /// handle, leaving every other class alone.
    ///
    /// # Errors
    ///
    /// [`crate::Error::Config`] if something else has already claimed this
    /// build's capture table.
    pub fn install(options: &mut BuildOptions) -> Result<()> {
        let seen: Arc<Captured<Screen>> =
            options
                .realize
                .hosts
                .open(HostKind::CAPTURE, CLASS_NAME, Captured::new)?;
        options.bindings.replace(CLASS_NAME, move |props| {
            let video = Arc::new(Video::new(props)?);
            seen.push(&video.screen());
            Ok(video)
        });
        Ok(())
    }

    /// The video circuit this build constructed, with its dot-clock rate
    /// resolved from `machine`'s clock forest.
    ///
    /// The most recent one, for a machine with several. `None` if this build
    /// has no Macintosh video circuit in it.
    #[must_use]
    pub fn take(hosts: &HostObjects, machine: &Machine) -> Option<MacScanout> {
        let seen = hosts
            .get::<Captured<Screen>>(HostKind::CAPTURE, CLASS_NAME)
            .ok()
            .flatten()?;
        let screen = seen.take()?;
        let rate = machine
            .devices()
            .iter()
            .rev()
            .find(|d| d.class().name == CLASS_NAME)
            .and_then(|d| d.domain())
            .and_then(|domain| machine.clocks().domain_frequency(domain).ok())
            .map(|f| (f.num(), f.den()));
        Some(MacScanout::new(screen, rate))
    }
}
