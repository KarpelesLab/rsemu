//! The R-Car Display Unit's side of the scanout seam.
//!
//! [`RcarScanout`] holds an `Arc<Du>` — [`crate::dev::rcar::du`] — and presents
//! it as a [`Scanout`] a host can capture. The DU composes its planes into
//! RGB888 itself, exactly as `lcd.scanout` hands out RGB888 rows, so there is
//! no colour conversion here; the surface is simply reshaped to whatever
//! active window the guest's timing registers describe.
//!
//! # Getting hold of the unit
//!
//! The same seam as [`lcd`](super::lcd) and [`mac`](super::mac), for the same
//! reason: a built machine hands back `Arc<dyn Device>` with no way back to
//! `Arc<Du>`, so [`capture::install`] keeps a handle at construction, and
//! [`capture::take`] reads the dot clock's exact rational frequency out of the
//! machine's clock forest, since the frame period is `(HCR + 1) × (VCR + 1)`
//! dots of it and the guest may change the totals at any time.

use alloc::sync::Arc;

use super::{PixelFormat, Scanout, Surface, SurfaceInfo};
use crate::dev::rcar::du::Du;

/// A [`Scanout`] over an R-Car DU.
#[derive(Debug, Clone)]
pub struct RcarScanout {
    du: Arc<Du>,
    /// The dot clock in hertz as `(numerator, denominator)`, if known.
    rate: Option<(u64, u64)>,
}

impl RcarScanout {
    /// Watch `du`, whose dot clock runs at `rate` hertz (`num / den`).
    #[must_use]
    pub fn new(du: Arc<Du>, rate: Option<(u64, u64)>) -> RcarScanout {
        RcarScanout { du, rate }
    }

    /// The unit being watched.
    #[must_use]
    pub fn du(&self) -> &Arc<Du> {
        &self.du
    }
}

impl Scanout for RcarScanout {
    fn info(&self) -> SurfaceInfo {
        let (width, height) = self.du.geometry();
        SurfaceInfo::new(width, height, PixelFormat::RGB888)
    }

    fn frame_counter(&self) -> u64 {
        self.du.frames()
    }

    fn frame_period_ns(&self) -> u64 {
        let Some((num, den)) = self.rate else {
            return 0;
        };
        if num == 0 {
            return 0;
        }
        let ns =
            u128::from(self.du.frame_ticks()) * u128::from(den) * 1_000_000_000 / u128::from(num);
        u64::try_from(ns).unwrap_or(u64::MAX)
    }

    fn capture(&self, dst: &mut Surface) -> u64 {
        // The counter before the pixels, as `lcd` does: a serial is never
        // ahead of the picture it labels.
        let serial = self.du.frames();
        let (width, height, pixels) = self.du.read_frame();
        dst.reshape(dst.format(), width, height);
        if width > 0 {
            for (y, row) in pixels.chunks(width as usize).enumerate() {
                for (x, pixel) in row.iter().enumerate() {
                    dst.put(x as u32, y as u32, *pixel);
                }
            }
        }
        dst.set_serial(serial);
        serial
    }
}

/// The interception that gets a host an `Arc<Du>` out of a described machine.
pub mod capture {
    use alloc::sync::Arc;

    use super::RcarScanout;
    use crate::core::error::Result;
    use crate::core::hosts::{Captured, HostKind, HostObjects};
    use crate::dev::rcar::du::{CLASS_NAME, Du};
    use crate::machine::{BuildOptions, Machine};

    /// Replace `rcar.du`'s constructor in `options` with one that keeps a
    /// handle, leaving every other class alone.
    ///
    /// # Errors
    ///
    /// [`crate::Error::Config`] if something else has already claimed this
    /// build's capture table.
    pub fn install(options: &mut BuildOptions) -> Result<()> {
        let seen: Arc<Captured<Du>> =
            options
                .realize
                .hosts
                .open(HostKind::CAPTURE, CLASS_NAME, Captured::new)?;
        options.bindings.replace(CLASS_NAME, move |props| {
            let du = Arc::new(Du::new(props)?);
            seen.push(&du);
            Ok(du)
        });
        Ok(())
    }

    /// The DU this build constructed (the most recent, for a machine with
    /// several), with its dot clock's rate resolved from `machine`.
    #[must_use]
    pub fn take(hosts: &HostObjects, machine: &Machine) -> Option<RcarScanout> {
        let seen = hosts
            .get::<Captured<Du>>(HostKind::CAPTURE, CLASS_NAME)
            .ok()
            .flatten()?;
        let du = seen.take()?;
        let rate = machine
            .devices()
            .iter()
            .rev()
            .find(|d| d.class().name == CLASS_NAME)
            .and_then(|d| d.domain())
            .and_then(|domain| machine.clocks().domain_frequency(domain).ok())
            .map(|f| (f.num(), f.den()));
        Some(RcarScanout::new(du, rate))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::space::MemAttrs;
    use crate::core::value::Width;

    #[test]
    fn a_described_board_reaches_a_host_surface_at_sixty_hertz() {
        let mut options = crate::machine::BuildOptions::new();
        options.classes.insert(crate::dev::rcar::du::schema());
        for s in crate::machine::builtin::schemas() {
            options.classes.insert(s);
        }
        crate::machine::builtin::bind(&mut options.bindings).expect("ram");
        capture::install(&mut options).expect("the interception installs");
        let mut registry = crate::core::Registry::new();
        crate::dev::rcar::du::register(&mut registry).expect("rcar.du");
        crate::machine::builtin::register(&mut registry).expect("ram");
        let text = concat!(
            "machine \"navi\" {\n",
            "  osc dclk = 37044000 Hz\n",
            "  space mem { width = 32 }\n",
            "  object sdram \"ram\" { size = 4M }\n",
            "  object du \"rcar.du\" { clock = dclk\n space = mem }\n",
            "  map mem 0x9e000000 size 4M = sdram\n",
            "  map mem 0xfff80000 size 0x40000 = du\n",
            "}\n"
        );
        let machine = crate::machine::build("navi.machine", text, &registry, &options)
            .expect("the board builds");
        let scanout = capture::take(&options.realize.hosts, &machine).expect("the board has a DU");
        let mem = machine.space("mem").expect("mem");
        mem.write_bytes(0x9e00_0000, &[0x00, 0xf8], MemAttrs::DEFAULT)
            .unwrap();
        for (offset, value) in [
            (0x40u64, 0xc5u32),
            (0x44, 0x3e5),
            (0x48, 0x1f),
            (0x4c, 0x1ff),
            (0x50, 0x497),
            (0x58, 0x20c),
            (0x18, 0x8000_0000),
            (0x100, 0x4001),
            (0x104, 800),
            (0x110, 800),
            (0x114, 480),
            (0x120, 0x9e00_0000),
            (0x00, 0x100),
        ] {
            mem.write(
                0xfff8_0000 + offset,
                Width::U32,
                u64::from(value),
                MemAttrs::DEFAULT,
            )
            .unwrap();
        }
        let info = scanout.info();
        assert_eq!((info.width, info.height), (800, 480));
        assert_eq!(
            scanout.frame_period_ns(),
            16_666_666,
            "1176 x 525 at 37.044 MHz"
        );
        let mut surface = Surface::new(PixelFormat::RGB888, 1, 1);
        scanout.capture(&mut surface);
        assert_eq!((surface.width(), surface.height()), (800, 480));
        assert_eq!(surface.get(0, 0), Some([0xff, 0, 0]));
        assert_eq!(surface.get(1, 0), Some([0, 0, 0]));
    }
}
