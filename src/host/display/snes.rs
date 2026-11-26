//! The SNES side of the scanout seam: an S-PPU's framebuffer as host pixels.
//!
//! [`SnesScanout`] holds an `Arc<SnesPpu>` and turns its RGB555 output —
//! brightness already applied, which is what the PPU2's DAC is handed — into
//! eight bits a gun by bit replication, `c << 3 | c >> 2`, so 31 is `$FF` and
//! 0 is `$00`.
//!
//! # The picture changes size
//!
//! Four heights — 224 or 239 lines, doubled in interlace — and two widths, 256
//! or 512 once any line of the frame was hires. [`Scanout::info`] reports what
//! the last *completed* frame was. The device keeps one 512 × 478 buffer and
//! stores a low-resolution line doubled and a non-interlaced line on both rows
//! of its pair, so this adapter only has to sample it
//! ([`fb_index`]).
//!
//! # Getting hold of the chip
//!
//! The same seam [`sms`](super::sms) documents: [`capture::install`] replaces
//! the `snes.ppu` constructor with one that keeps a clone, and
//! [`capture::take`] hands it back as a scanout after the build.

use alloc::sync::Arc;

use super::{PixelFormat, Scanout, Surface, SurfaceInfo};
use crate::dev::snes::ppu::timing::MASTER_PER_LINE;
use crate::dev::snes::ppu::{SnesPpu, fb_index};

/// One RGB555 value as host RGB.
#[must_use]
pub const fn snes_rgb(c: u16) -> [u8; 3] {
    const fn ch(c: u16, shift: u16) -> u8 {
        let v = ((c >> shift) & 31) as u8;
        (v << 3) | (v >> 2)
    }
    [ch(c, 0), ch(c, 5), ch(c, 10)]
}

/// A [`Scanout`] over an S-PPU.
#[derive(Debug, Clone)]
pub struct SnesScanout {
    ppu: Arc<SnesPpu>,
}

impl SnesScanout {
    /// Watch `ppu`.
    #[must_use]
    pub fn new(ppu: Arc<SnesPpu>) -> SnesScanout {
        SnesScanout { ppu }
    }

    /// The chip being watched.
    #[must_use]
    pub fn ppu(&self) -> &Arc<SnesPpu> {
        &self.ppu
    }
}

impl Scanout for SnesScanout {
    fn info(&self) -> SurfaceInfo {
        let (w, h) = self.ppu.output_size();
        SurfaceInfo::new(u32::from(w), u32::from(h), PixelFormat::RGBA8888)
    }

    fn frame_counter(&self) -> u64 {
        self.ppu.frame()
    }

    /// A non-interlaced frame's nominal length — lines × 1364 master clocks —
    /// over the master clock, in integer nanoseconds. It only paces a host.
    fn frame_period_ns(&self) -> u64 {
        let region = self.ppu.tv_region();
        let (num, den) = region.master_clock();
        let clocks = u64::from(region.base_lines()) * u64::from(MASTER_PER_LINE);
        clocks.saturating_mul(den).saturating_mul(1_000_000_000) / num
    }

    fn capture(&self, dst: &mut Surface) -> u64 {
        let info = self.info();
        dst.reshape(dst.format(), info.width, info.height);
        let serial = self.ppu.frame();
        let (w, h) = (info.width as usize, info.height as usize);
        self.ppu.with_framebuffer(|fb| {
            for y in 0..h {
                for x in 0..w {
                    dst.put(x as u32, y as u32, snes_rgb(fb[fb_index(x, y, w, h)]));
                }
            }
        });
        dst.set_serial(serial);
        serial
    }
}

/// The picture of the last completed frame as one FNV-1a hash, over the
/// RGB555 values at output resolution.
#[must_use]
pub fn frame_hash(ppu: &SnesPpu) -> u64 {
    let (w, h) = ppu.output_size();
    let (w, h) = (usize::from(w), usize::from(h));
    ppu.with_framebuffer(|fb| {
        let mut hash = 0xcbf2_9ce4_8422_2325u64;
        for y in 0..h {
            for x in 0..w {
                for byte in fb[fb_index(x, y, w, h)].to_le_bytes() {
                    hash ^= u64::from(byte);
                    hash = hash.wrapping_mul(0x0000_0100_0000_01b3);
                }
            }
        }
        hash
    })
}

/// The interception that gets a host a concrete handle out of a described
/// machine. A seam, not a design — see [`sms::capture`](super::sms::capture).
pub mod capture {
    use super::{Arc, SnesPpu, SnesScanout};
    use crate::core::error::Result;
    use crate::core::hosts::{Captured, HostKind, HostObjects};
    use crate::dev::snes::ppu::CLASS;
    use crate::machine::BuildOptions;

    /// Replace the `snes.ppu` constructor in `options` with one that keeps a
    /// handle.
    ///
    /// # Errors
    ///
    /// [`crate::Error::Config`] if something else already claimed this
    /// build's capture table.
    pub fn install(options: &mut BuildOptions) -> Result<()> {
        let ppus: Arc<Captured<SnesPpu>> =
            options
                .realize
                .hosts
                .open(HostKind::CAPTURE, CLASS.name, Captured::new)?;
        options.bindings.replace(CLASS.name, move |props| {
            let ppu = Arc::new(SnesPpu::from_props(props)?);
            ppus.push(&ppu);
            Ok(ppu)
        });
        Ok(())
    }

    /// The S-PPU this build constructed, as a [`SnesScanout`]; `None` if the
    /// machine has none.
    #[must_use]
    pub fn take(hosts: &HostObjects) -> Option<SnesScanout> {
        hosts
            .get::<Captured<SnesPpu>>(HostKind::CAPTURE, CLASS.name)
            .ok()
            .flatten()?
            .take()
            .map(SnesScanout::new)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn rgb555_replicates_its_top_bits() {
        assert_eq!(snes_rgb(0x7FFF), [0xFF, 0xFF, 0xFF]);
        assert_eq!(snes_rgb(0), [0, 0, 0]);
        assert_eq!(snes_rgb(0x001F), [0xFF, 0, 0]);
        assert_eq!(snes_rgb(0x7C00), [0, 0, 0xFF]);
        assert_eq!(snes_rgb(16 << 5), [0, 0x84, 0]);
    }

    #[test]
    fn a_scanout_reports_the_completed_frame_and_its_period() {
        let ppu = Arc::new(SnesPpu::default());
        ppu.with_engine(|e| {
            e.cgram[0] = 0x001F;
            e.r.inidisp = 0x0F;
        });
        // Through the first frame's vblank.
        ppu.advance_by(230 * 1364);
        let s = SnesScanout::new(Arc::clone(&ppu));
        let info = s.info();
        assert_eq!((info.width, info.height), (256, 224));
        assert_eq!(s.frame_counter(), 1);
        let mut surface = Surface::new(PixelFormat::RGBA8888, 1, 1);
        assert_eq!(s.capture(&mut surface), 1);
        assert_eq!(surface.get(0, 0), Some([0xFF, 0, 0]));
        assert_eq!(surface.get(255, 223), Some([0xFF, 0, 0]));
        // 262 × 1364 / (236 250 000 / 11) s ≈ 16.64 ms.
        assert_eq!(s.frame_period_ns(), 16_639_356);
        assert_ne!(frame_hash(&ppu), 0);
    }
}
