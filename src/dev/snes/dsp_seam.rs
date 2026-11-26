//! The seam between the S-SMP and the S-DSP.
//!
//! The S-SMP owns audio RAM and the `$F2`/`$F3` register window; the S-DSP owns
//! the 128 registers behind that window and reads (and, for echo, writes) audio
//! RAM on its own timeline. Neither half names the other's type: the SMP holds
//! a `dyn SDsp`, and hands the DSP a `dyn Aram` on every clock.

use crate::core::error::Result;
use crate::core::state::{ChunkReader, ChunkWriter};

/// What the S-DSP needs of audio RAM: byte access by 16-bit address.
pub trait Aram {
    /// Read one byte.
    fn read(&self, addr: u16) -> u8;
    /// Write one byte.
    fn write(&mut self, addr: u16, value: u8);
}

/// The S-DSP as the SMP drives it.
pub trait SDsp: Send + Sync + core::fmt::Debug {
    /// $F3 read with DSPADDR's low 7 bits (the mirror rule for addr ≥ 0x80 is the SMP's).
    fn read(&self, reg: u8) -> u8;
    /// $F3 write.
    fn write(&mut self, reg: u8, value: u8);
    /// Advance by one SMP clock (1.024 MHz); returns a stereo sample every 32nd call (32 kHz).
    fn clock(&mut self, aram: &mut dyn Aram) -> Option<(i16, i16)>;
    /// Reset (power-on or the SNES reset line).
    fn reset(&mut self);

    /// Write the DSP's state into the SMP's snapshot chunk.
    ///
    /// The default writes nothing, so an implementation without state of its
    /// own (a stub, a test double) need not care. [`super::dsp::SDspChip`]
    /// overrides it.
    ///
    /// # Errors
    ///
    /// If the sink fails.
    fn save(&self, w: &mut ChunkWriter<'_>) -> Result<()> {
        let _ = w;
        Ok(())
    }

    /// Restore what [`SDsp::save`] wrote, in the same order.
    ///
    /// # Errors
    ///
    /// If the chunk is truncated or holds a state the chip cannot be in.
    fn load(&mut self, r: &mut ChunkReader<'_>) -> Result<()> {
        let _ = r;
        Ok(())
    }
}
