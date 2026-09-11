//! The deterministic stream a device draws "random" bytes from.
//!
//! # Why this is in `core/`
//!
//! `CLAUDE.md` says any non-deterministic input crossing into the machine goes
//! through the record/replay seam or it is a determinism bug, and
//! [`core::record`](crate::core::record)'s own table answers the entropy row
//! with *"already deterministic: `virtio-rng` is a seeded SplitMix64"*. That
//! is the right answer — a seed is not an input arriving at an instant, it is
//! part of the machine's **initial state**, so it is configuration rather than
//! something to log — but it had been written once, inside one device, and the
//! second device that needed it (`st.rng`) would have been the second copy of
//! the algorithm and the second set of determinism claims to check.
//!
//! So the generator lives here, unconditionally, next to the other framework
//! facts about reproducibility. It is four lines of arithmetic; what it is
//! really carrying is the *contract* below.
//!
//! # The contract
//!
//! 1. A stream is a pure function of its seed. The same machine file run twice
//!    produces the same bytes and the same state hash.
//! 2. A generator's **position** is architectural state. It is saved and
//!    restored, so a snapshot taken mid-stream resumes mid-stream rather than
//!    replaying numbers a guest has already consumed.
//! 3. A device does **not** use its seed property directly. It mixes it with
//!    its own instance path through [`derive_seed`], so two instances of one
//!    class on one board draw different streams without anybody having to
//!    write a second number into the machine file.
//! 4. None of this is a security primitive, and a guest that needs
//!    unpredictable bytes must not get them from an emulator whose whole
//!    purpose is that it does the same thing twice.
//!
//! # The generator
//!
//! SplitMix64, from Steele, Lea and Flood, *Fast Splittable Pseudorandom
//! Number Generators* (OOPSLA 2014), §4. A published algorithm, with good
//! enough statistical properties for this and no claim to be more.

use crate::core::error::Result;
use crate::core::state::{ChunkReader, ChunkWriter, Sink, Source};

/// SplitMix64's golden-ratio increment.
const GAMMA: u64 = 0x9e37_79b9_7f4a_7c15;

/// FNV-1a's 64-bit offset basis, for [`derive_seed`]'s path hash.
const FNV_OFFSET: u64 = 0xcbf2_9ce4_8422_2325;

/// FNV-1a's 64-bit prime.
const FNV_PRIME: u64 = 0x0000_0100_0000_01b3;

/// SplitMix64's finalizing mix, which is also what makes [`derive_seed`] spread
/// two neighbouring seeds apart.
#[inline]
const fn mix(z: u64) -> u64 {
    let z = (z ^ (z >> 30)).wrapping_mul(0xbf58_476d_1ce4_e5b9);
    let z = (z ^ (z >> 27)).wrapping_mul(0x94d0_49bb_1331_11eb);
    z ^ (z >> 31)
}

/// The seed one device instance draws from, given the board's seed.
///
/// A board declares **one** number — `param seed = 1`, handed to every device
/// that needs it — and each instance still gets a stream of its own, because
/// the instance path goes into the mix. That path is the snapshot chunk key
/// ([`RealizeCtx::path`](crate::core::device::RealizeCtx::path)), so it is
/// stable for the life of the machine and a restored snapshot derives the same
/// number.
///
/// Deriving rather than using `seed` directly is what stops two `st.rng`
/// objects on one board from handing the guest the same words twice — a bug
/// that looks like working hardware right up until firmware compares them.
#[must_use]
pub fn derive_seed(seed: u64, path: &str) -> u64 {
    let mut hash = FNV_OFFSET;
    for byte in path.as_bytes() {
        hash ^= u64::from(*byte);
        hash = hash.wrapping_mul(FNV_PRIME);
    }
    mix(seed ^ hash)
}

/// A seeded, snapshot-able stream of bytes.
///
/// The position is the whole state: `Stream::new(s)` and a `Stream` restored
/// from a snapshot of a `Stream::new(s)` that has produced nothing are the same
/// object.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Stream {
    /// What the stream was started from, so [`Stream::rewind`] can go back.
    seed: u64,
    /// The generator's position.
    state: u64,
}

impl Stream {
    /// A stream started from `seed`.
    #[must_use]
    pub const fn new(seed: u64) -> Stream {
        Stream { seed, state: seed }
    }

    /// The seed this stream was started from.
    #[must_use]
    pub const fn seed(&self) -> u64 {
        self.seed
    }

    /// Put the stream back to its seed, as a device reset does.
    ///
    /// A reset-and-rerun then produces the same numbers, which is what makes
    /// the reset path reproducible rather than merely deterministic-ish.
    pub const fn rewind(&mut self) {
        self.state = self.seed;
    }

    /// The next sixty-four bits (Steele, Lea and Flood, §4).
    #[inline]
    pub const fn next_u64(&mut self) -> u64 {
        self.state = self.state.wrapping_add(GAMMA);
        mix(self.state)
    }

    /// The next thirty-two bits — the low half of [`Stream::next_u64`].
    ///
    /// A 32-bit draw consumes a whole 64-bit step rather than handing out the
    /// two halves of one, so a device's word count and its stream position stay
    /// in step and a snapshot has one number to carry.
    #[inline]
    pub const fn next_u32(&mut self) -> u32 {
        self.next_u64() as u32
    }

    /// Fill `dst`, continuing across calls.
    pub fn fill(&mut self, dst: &mut [u8]) {
        for piece in dst.chunks_mut(8) {
            let word = self.next_u64().to_le_bytes();
            piece.copy_from_slice(&word[..piece.len()]);
        }
    }

    /// Write the position into a snapshot.
    ///
    /// The seed is **not** written: it comes from the machine description, so a
    /// snapshot that carried it could restore a stream the board no longer
    /// describes.
    ///
    /// # Errors
    ///
    /// Whatever the writer reports.
    pub fn save(&self, w: &mut ChunkWriter<'_>) -> Result<()> {
        w.write_u64(self.state)
    }

    /// Read the position back.
    ///
    /// # Errors
    ///
    /// Whatever the reader reports.
    pub fn load(&mut self, r: &mut ChunkReader<'_>) -> Result<()> {
        self.state = r.read_u64()?;
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use alloc::vec;
    use alloc::vec::Vec;

    fn take(seed: u64, n: usize) -> Vec<u64> {
        let mut s = Stream::new(seed);
        (0..n).map(|_| s.next_u64()).collect()
    }

    #[test]
    fn the_same_seed_gives_the_same_stream_and_a_different_seed_does_not() {
        assert_eq!(take(42, 16), take(42, 16));
        assert_ne!(take(42, 16), take(43, 16));
    }

    #[test]
    fn a_partial_word_is_filled_and_the_stream_continues_across_calls() {
        let mut a = Stream::new(1);
        let mut whole = [0u8; 16];
        a.fill(&mut whole);

        let mut b = Stream::new(1);
        let mut first = [0u8; 13];
        let mut rest = [0u8; 3];
        b.fill(&mut first);
        b.fill(&mut rest);

        // The thirteen-byte fill consumed two whole words, so the continuation
        // starts at word three rather than mid-word.
        assert_eq!(first[..], whole[..13]);
    }

    #[test]
    fn a_rewind_replays_the_stream() {
        let mut s = Stream::new(9);
        let first: Vec<u64> = (0..4).map(|_| s.next_u64()).collect();
        s.rewind();
        let again: Vec<u64> = (0..4).map(|_| s.next_u64()).collect();
        assert_eq!(first, again);
    }

    #[test]
    fn two_instance_paths_under_one_board_seed_do_not_share_a_stream() {
        // The property the whole derivation exists for.
        let a = derive_seed(1, "rng");
        let b = derive_seed(1, "rng2");
        assert_ne!(a, b);
        assert_ne!(take(a, 8), take(b, 8));
        // And the derivation is a function, so a restart reproduces it.
        assert_eq!(a, derive_seed(1, "rng"));
        // Neighbouring board seeds are not neighbouring device seeds.
        assert_ne!(derive_seed(1, "rng"), derive_seed(2, "rng"));
    }

    #[test]
    fn the_position_survives_a_round_trip_and_the_seed_is_not_in_it() {
        use crate::core::state::{MachineShape, Migrations, StateReader, StateWriter};

        let mut saved = Stream::new(5);
        for _ in 0..3 {
            saved.next_u64();
        }

        let mut shape = MachineShape::new();
        shape.add_device("rng", "st.rng").unwrap();
        let mut w = StateWriter::new(shape);
        {
            let mut chunk = w.chunk("rng", "st.rng", 1).unwrap();
            saved.save(&mut chunk).unwrap();
        }
        let bytes = w.to_vec().unwrap();

        let mut restored = Stream::new(5);
        let reader = StateReader::new(&bytes).unwrap();
        let chunk = reader.load("rng", "st.rng", 1, &Migrations::new()).unwrap();
        restored.load(&mut chunk.reader()).unwrap();

        assert_eq!(restored, saved);
        let next: Vec<u64> = vec![restored.next_u64()];
        let mut ahead = saved;
        assert_eq!(next, vec![ahead.next_u64()]);
    }
}
