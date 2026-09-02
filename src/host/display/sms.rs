//! The Master System side of the scanout seam: a 315-5124's framebuffer as host
//! pixels.
//!
//! [`SmsScanout`] holds an `Arc<SmsVdp>` and does the one thing the VDP
//! deliberately refuses to do — turn its six-bit `--BBGGRR` output into RGB.
//! The chip has one video DAC with two bits a gun, so the conversion is exact
//! and needs no table: `0`, `1`, `2`, `3` become `$00`, `$55`, `$AA`, `$FF`,
//! which are the four levels the resistor ladder actually produces.
//!
//! # The picture changes height
//!
//! Mode 4 has three: 192 lines, 224, and — on a PAL machine — 240. Which one is
//! selected is two bits of two VDP registers and a game may change it between
//! frames, so [`Scanout::info`] reports the *current* height rather than a
//! constant. The device's framebuffer is always the tallest of the three and
//! this crops it, which is why a mode change costs nothing and shows nothing
//! stale.
//!
//! # Getting hold of the chip
//!
//! A machine built from a description hands back `Arc<dyn Device>`, and there is
//! no route from a `dyn Device` to `Arc<SmsVdp>` — `Device` has no `Any` in its
//! supertrait chain, on purpose (`machine::realize` explains why). So the host
//! takes its handle at the only moment the concrete type exists: device
//! construction. [`capture::install`] wraps the machine's bindings so that
//! `sms.vdp` and `sms.sdsc` are built by constructors here, each of which keeps
//! a clone before handing the device on.
//!
//! **The I/O chip is not one of them any more.** It used to be captured so a
//! front end could press buttons on it, and that was the interim door
//! `dev::sms::io` documented; the buttons now live in a named host object
//! ([`pads`](crate::dev::sms::io::pads)) that a front end opens by name and a
//! recorder registers as a channel, so there is nothing left to intercept. What
//! remains here is *output* — a picture and a debug console — which is not
//! input and does not belong in the record/replay seam.
//!
//! ```text
//! let mut options = catalog::build_options()?;
//! display::sms::capture::install(&mut options)?;   // intercept the three
//! let machine = machine::build(name, source, &registry, &options)?;
//! let scanout = display::sms::capture::take_vdp(&options.realize.hosts);
//! ```
//!
//! **This is a seam, and it is marked as one**, exactly like
//! [`nes::capture`](super::nes::capture). It exists because `Device` has no
//! scanout hook and no input hook yet; when it grows them, every line of
//! [`capture`] deletes and nothing else here changes. The tables belong to the
//! build, so two consoles built in one process do not swap chips.
//!
//! The pads and the debug console are here rather than in a module of their own
//! for the same reason and by the same mechanism: a front end needs to press a
//! button, and a conformance run needs to read what a test ROM printed, and both
//! need a concrete handle the machine layer will not give them.

use alloc::sync::Arc;

use super::{PixelFormat, Scanout, Surface, SurfaceInfo};
use crate::dev::sms::vdp::{SCREEN_HEIGHT, SCREEN_WIDTH, SmsVdp, TvRegion};

/// One six-bit `--BBGGRR` value as host RGB.
///
/// Two bits a gun, and the four levels are evenly spaced: the VDP's output is a
/// two-bit resistor ladder per colour, so `n * 85` is the voltage it produces
/// rather than an approximation of it.
#[must_use]
pub const fn sms_rgb(colour: u8) -> [u8; 3] {
    let r = colour & 0x03;
    let g = (colour >> 2) & 0x03;
    let b = (colour >> 4) & 0x03;
    [r * 85, g * 85, b * 85]
}

/// A [`Scanout`] over a Master System VDP.
#[derive(Debug, Clone)]
pub struct SmsScanout {
    vdp: Arc<SmsVdp>,
}

impl SmsScanout {
    /// Watch `vdp`.
    #[must_use]
    pub fn new(vdp: Arc<SmsVdp>) -> SmsScanout {
        SmsScanout { vdp }
    }

    /// The chip being watched, for a host that wants its registers too.
    #[must_use]
    pub fn vdp(&self) -> &Arc<SmsVdp> {
        &self.vdp
    }
}

impl Scanout for SmsScanout {
    fn info(&self) -> SurfaceInfo {
        SurfaceInfo::new(
            SCREEN_WIDTH as u32,
            u32::from(self.vdp.active_height()),
            PixelFormat::RGBA8888,
        )
    }

    fn frame_counter(&self) -> u64 {
        self.vdp.frame()
    }

    fn frame_period_ns(&self) -> u64 {
        // Lines x pixels-per-line x master-ticks-per-pixel x (1e9 / master Hz),
        // in exact integer arithmetic. The frame period is a fact about the
        // oscillator forest, never a wall-clock measurement (`CLAUDE.md`), and
        // this number only paces a host — it never drives guest time.
        let region = self.vdp.tv_region();
        let (num, den) = match region {
            TvRegion::Ntsc => crate::dev::sms::NTSC_MASTER_HZ,
            TvRegion::Pal => crate::dev::sms::PAL_MASTER_HZ,
        };
        let dots = u64::from(region.lines_per_frame()) * crate::dev::sms::vdp::DOTS_PER_LINE;
        dots.saturating_mul(crate::dev::sms::DOT_DIVIDER)
            .saturating_mul(den)
            .saturating_mul(1_000_000_000)
            / num
    }

    fn capture(&self, dst: &mut Surface) -> u64 {
        let info = self.info();
        dst.reshape(dst.format(), info.width, info.height);

        // The counter is read before the pixels: if the emulation thread is
        // mid-frame the surface may hold the frame *after* this one, and a
        // serial that is never ahead of its pixels is the safe direction to err
        // — a host redraws once more, rather than never.
        let serial = self.vdp.frame();
        self.vdp.with_framebuffer(|fb| {
            for y in 0..info.height {
                let row = (y as usize) * SCREEN_WIDTH;
                for x in 0..info.width {
                    dst.put(x, y, sms_rgb(fb[row + x as usize]));
                }
            }
        });
        dst.set_serial(serial);
        serial
    }
}

/// Every pixel of the framebuffer, cropped to the active height, as one hash.
///
/// What a headless regression asserts: the picture a described machine produces
/// after N frames, in one number. FNV-1a, because it is four lines of code, has
/// no dependency and is being used to detect change rather than to resist an
/// adversary.
#[must_use]
pub fn frame_hash(vdp: &SmsVdp) -> u64 {
    let height = vdp.active_height() as usize;
    vdp.with_framebuffer(|fb| {
        let mut hash = 0xcbf2_9ce4_8422_2325u64;
        for byte in &fb[..height * SCREEN_WIDTH] {
            hash ^= u64::from(*byte);
            hash = hash.wrapping_mul(0x0000_0100_0000_01b3);
        }
        hash
    })
}

/// Ensure the framebuffer's declared size and the screen constants agree.
const _: () = assert!(SCREEN_HEIGHT == 240);

/// The interception that gets a host concrete handles out of a described
/// machine. See the module docs: a seam, not a design.
pub mod capture {
    use super::{Arc, SmsScanout, SmsVdp};
    use crate::core::error::Result;
    use crate::core::hosts::{Captured, HostKind, HostObjects};
    use crate::dev::sms::SdscConsole;
    use crate::machine::BuildOptions;

    /// Replace the two constructors in `options` with ones that keep a handle,
    /// leaving every other class alone.
    ///
    /// # Errors
    ///
    /// [`crate::Error::Config`] if something else has already claimed one of
    /// this build's capture tables.
    pub fn install(options: &mut BuildOptions) -> Result<()> {
        let vdps: Arc<Captured<SmsVdp>> = table(options)?;
        options
            .bindings
            .replace(crate::dev::sms::vdp::CLASS.name, move |props| {
                let vdp = Arc::new(SmsVdp::from_props(props)?);
                vdps.push(&vdp);
                Ok(vdp)
            });

        let consoles: Arc<Captured<SdscConsole>> = table(options)?;
        options
            .bindings
            .replace(crate::dev::sms::sdsc::CLASS.name, move |props| {
                let console = Arc::new(SdscConsole::from_props(props)?);
                consoles.push(&console);
                Ok(console)
            });
        Ok(())
    }

    /// This build's capture table for `T`, filed under the class it captures.
    fn table<T: Captures>(options: &BuildOptions) -> Result<Arc<Captured<T>>> {
        options
            .realize
            .hosts
            .open(HostKind::CAPTURE, T::CLASS_NAME, Captured::new)
    }

    /// A chip this module intercepts, and the class it is built for.
    ///
    /// A trait rather than three copies of the same two lines: the class name
    /// and the captured type have to agree, and this is the only way to say so
    /// once.
    trait Captures: Send + Sync + 'static {
        /// The class whose constructor is replaced to capture this type.
        const CLASS_NAME: &'static str;
    }

    impl Captures for SmsVdp {
        const CLASS_NAME: &'static str = crate::dev::sms::vdp::CLASS.name;
    }

    impl Captures for SdscConsole {
        const CLASS_NAME: &'static str = crate::dev::sms::sdsc::CLASS.name;
    }

    /// What this build captured of `T`, if the interception was installed.
    fn taken<T: Captures>(hosts: &HostObjects) -> Option<Arc<T>> {
        hosts
            .get::<Captured<T>>(HostKind::CAPTURE, T::CLASS_NAME)
            .ok()
            .flatten()?
            .take()
    }

    /// The VDP this build constructed, as an [`SmsScanout`].
    ///
    /// `None` if this build has no VDP in it — a machine with no picture, which
    /// a host must be able to render nothing for.
    #[must_use]
    pub fn take_vdp(hosts: &HostObjects) -> Option<SmsScanout> {
        taken::<SmsVdp>(hosts).map(SmsScanout::new)
    }

    /// The debug console this build constructed, which is how a headless
    /// conformance run reads a test ROM's output.
    #[must_use]
    pub fn take_console(hosts: &HostObjects) -> Option<Arc<SdscConsole>> {
        taken::<SdscConsole>(hosts)
    }
}
