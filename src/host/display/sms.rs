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
//! `sms.vdp`, `sms.io` and `sms.sdsc` are built by constructors here, each of
//! which keeps a clone before handing the device on.
//!
//! ```text
//! let mut options = catalog::build_options()?;
//! display::sms::capture::install(&mut options)?;   // intercept the three
//! let machine = machine::build(name, source, &registry, &options)?;
//! let scanout = display::sms::capture::take_vdp();
//! ```
//!
//! **This is a seam, and it is marked as one**, exactly like
//! [`nes::capture`](super::nes::capture). It exists because `Device` has no
//! scanout hook and no input hook yet; when it grows them, every line of
//! [`capture`] deletes and nothing else here changes. Until then the table is
//! process-wide, so build one machine at a time or [`capture::clear`] between
//! them.
//!
//! The pads and the debug console are here rather than in a module of their own
//! for the same reason and by the same mechanism: a front end needs to press a
//! button, and a conformance run needs to read what a test ROM printed, and both
//! need a concrete handle the machine layer will not give them.

use alloc::sync::Arc;
use alloc::vec::Vec;

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
    use super::{Arc, SmsScanout, SmsVdp, Vec};
    use crate::core::error::Result;
    use crate::core::props::Props;
    use crate::core::sync::{Global, LockRank};
    use crate::dev::sms::{SdscConsole, SmsIo};
    use crate::machine::realize::Instance;
    use crate::machine::{Bindings, BuildOptions};

    /// Every chip of each kind constructed since the last take or [`clear`],
    /// oldest first. A `Vec` rather than a single slot because a machine with
    /// two of something is not this module's business to refuse.
    static VDPS: Global<Vec<Arc<SmsVdp>>> = Global::with_rank(LockRank::LEAF, Vec::new());
    static IOS: Global<Vec<Arc<SmsIo>>> = Global::with_rank(LockRank::LEAF, Vec::new());
    static CONSOLES: Global<Vec<Arc<SdscConsole>>> = Global::with_rank(LockRank::LEAF, Vec::new());

    /// A class name and the constructor that keeps a handle to what it builds.
    type Interception = (&'static str, fn(&Props) -> Result<Arc<dyn Instance>>);

    fn construct_vdp(props: &Props) -> Result<Arc<dyn Instance>> {
        let vdp = Arc::new(SmsVdp::from_props(props)?);
        VDPS.lock().push(Arc::clone(&vdp));
        Ok(vdp)
    }

    fn construct_io(props: &Props) -> Result<Arc<dyn Instance>> {
        let io = Arc::new(SmsIo::from_props(props)?);
        IOS.lock().push(Arc::clone(&io));
        Ok(io)
    }

    fn construct_sdsc(props: &Props) -> Result<Arc<dyn Instance>> {
        let console = Arc::new(SdscConsole::from_props(props)?);
        CONSOLES.lock().push(Arc::clone(&console));
        Ok(console)
    }

    /// Replace the three constructors in `bindings`, leaving every other class
    /// alone.
    ///
    /// # Errors
    ///
    /// [`crate::Error::Config`] if a class turns out to be bound twice, which
    /// would be a bug in the caller's binding table rather than here.
    pub fn intercept(bindings: &Bindings) -> Result<Bindings> {
        let ours: [Interception; 3] = [
            (crate::dev::sms::vdp::CLASS.name, construct_vdp),
            (crate::dev::sms::io::CLASS.name, construct_io),
            (crate::dev::sms::sdsc::CLASS.name, construct_sdsc),
        ];
        let mut out = Bindings::new();
        let classes: Vec<&'static str> = bindings.classes().collect();
        for class in classes {
            match ours.iter().find(|(name, _)| *name == class) {
                Some((name, ctor)) => out.bind(name, *ctor)?,
                None => {
                    if let Some(ctor) = bindings.get(class) {
                        out.bind(class, ctor)?;
                    }
                }
            }
        }
        // A build with the device feature but a machine that does not use one:
        // binding it anyway is still correct, and an unused binding costs
        // nothing.
        for (name, ctor) in ours {
            if out.get(name).is_none() {
                out.bind(name, ctor)?;
            }
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

    /// Take the most recently constructed VDP as a [`SmsScanout`], forgetting
    /// every earlier one.
    ///
    /// `None` if no machine with one has been built since the last call — a
    /// machine with no picture, which a host must be able to render nothing for.
    #[must_use]
    pub fn take_vdp() -> Option<SmsScanout> {
        let mut table = VDPS.lock();
        let last = table.pop();
        table.clear();
        last.map(SmsScanout::new)
    }

    /// Take the most recently constructed I/O chip, for a front end that has
    /// buttons to press.
    #[must_use]
    pub fn take_io() -> Option<Arc<SmsIo>> {
        let mut table = IOS.lock();
        let last = table.pop();
        table.clear();
        last
    }

    /// Take the most recently constructed debug console, which is how a
    /// headless conformance run reads a test ROM's output.
    #[must_use]
    pub fn take_console() -> Option<Arc<SdscConsole>> {
        let mut table = CONSOLES.lock();
        let last = table.pop();
        table.clear();
        last
    }

    /// Forget every kept handle, so the next take cannot return a chip from a
    /// machine that has already been dropped.
    pub fn clear() {
        VDPS.lock().clear();
        IOS.lock().clear();
        CONSOLES.lock().clear();
    }
}
