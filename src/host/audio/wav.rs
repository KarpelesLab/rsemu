//! Headless capture: a stream as a RIFF/WAVE file.
//!
//! This is the audio counterpart of [`display::png`](super::super::display::png)
//! — how CI proves a machine makes a noise without a sound card, how a bug
//! report gets a recording rather than an adjective, and how a regression can
//! assert *what* a machine sounded like (`ROADMAP.md` §12).
//!
//! # No encoder, and no feature gate
//!
//! WAV is a header and the samples: eleven fields, all little-endian, and the
//! bytes we already have. So unlike `display-png` there is no dependency to
//! gate — this module is always compiled, including in the `no_std` build, and
//! its output is byte-for-byte reproducible.
//!
//! Written from Microsoft/IBM's *Multimedia Programming Interface and Data
//! Specifications 1.0* (the RIFF WAVE chunk layout) and IBM/Microsoft's
//! *Multiple Channel Audio Data and WAVE Files* (`WAVE_FORMAT_IEEE_FLOAT`).
//!
//! # Example
//!
//! ```
//! use rsemu::host::audio::{AudioBuffer, SampleFormat, StreamInfo, wav};
//!
//! let info = StreamInfo::new(44_100, 1, 1, SampleFormat::S16);
//! let mut queue = AudioBuffer::new(SampleFormat::S16, 1);
//! queue.push_frame(&[0]);
//! let file = wav::encode(info, &queue);
//! assert_eq!(&file[..4], b"RIFF");
//! assert_eq!(&file[8..12], b"WAVE");
//! ```

use alloc::vec::Vec;

use super::{AudioBuffer, SampleFormat, Sink, StreamInfo};

/// `WAVE_FORMAT_PCM`: integer samples.
const FORMAT_PCM: u16 = 1;
/// `WAVE_FORMAT_IEEE_FLOAT`: 32-bit floats in `[-1.0, 1.0]`.
const FORMAT_FLOAT: u16 = 3;

/// The 44 bytes before the first sample: `RIFF`, `fmt ` and the `data` header.
const HEADER_BYTES: usize = 44;

/// Which `wFormatTag` and how many bits per sample a [`SampleFormat`] is.
const fn tag_and_bits(format: SampleFormat) -> (u16, u16) {
    match format {
        SampleFormat::F32 => (FORMAT_FLOAT, 32),
        SampleFormat::U8 => (FORMAT_PCM, 8),
        _ => (FORMAT_PCM, 16),
    }
}

/// Encode a queue as a complete `.wav` file.
///
/// `info` supplies the rate and the channel count; the sample format and the
/// bytes come from `buffer`, because those are what is actually on disk.
///
/// A queue with no channels produces a valid, empty file rather than an error:
/// a machine that made no sound is a thing a host must be able to write down.
#[must_use]
pub fn encode(info: StreamInfo, buffer: &AudioBuffer) -> Vec<u8> {
    let mut out = Vec::with_capacity(HEADER_BYTES + buffer.bytes().len());
    write_header(
        &mut out,
        info.rate_hz(),
        buffer.channels(),
        buffer.format(),
        buffer.len(),
    );
    out.extend_from_slice(buffer.bytes());
    out
}

/// Lay down the 44-byte prologue for `data_bytes` of samples.
fn write_header(out: &mut Vec<u8>, rate: u32, channels: u16, format: SampleFormat, data: u64) {
    let (tag, bits) = tag_and_bits(format);
    let block_align = u32::from(channels) * u32::from(bits) / 8;
    let byte_rate = rate.saturating_mul(block_align);
    // RIFF's size field covers everything after it, which is the eight bytes of
    // `WAVE` + the `fmt ` header, the 16-byte `fmt ` body, the eight bytes of
    // the `data` header, and the samples: 36 + data. Saturating, because a
    // recording longer than 4 GiB is a broken file whatever we write here, and
    // panicking on the way out of a run would lose it entirely.
    let data = u32::try_from(data).unwrap_or(u32::MAX);

    out.extend_from_slice(b"RIFF");
    out.extend_from_slice(&data.saturating_add(36).to_le_bytes());
    out.extend_from_slice(b"WAVE");

    out.extend_from_slice(b"fmt ");
    out.extend_from_slice(&16u32.to_le_bytes());
    out.extend_from_slice(&tag.to_le_bytes());
    out.extend_from_slice(&channels.to_le_bytes());
    out.extend_from_slice(&rate.to_le_bytes());
    out.extend_from_slice(&byte_rate.to_le_bytes());
    // `nBlockAlign` is 16-bit; a stream with more than 8 192 channels is not a
    // thing, but truncating quietly is still worse than clamping.
    out.extend_from_slice(&(u16::try_from(block_align).unwrap_or(u16::MAX)).to_le_bytes());
    out.extend_from_slice(&bits.to_le_bytes());

    out.extend_from_slice(b"data");
    out.extend_from_slice(&data.to_le_bytes());
}

/// A [`Sink`] that accumulates a whole recording in memory and emits the file
/// when it is asked to.
///
/// Accumulating rather than streaming is deliberate: RIFF puts two lengths in
/// the header, so a streaming writer either seeks — which is not something a
/// `no_std` sink can do — or lies and patches later. A recording is bounded by
/// how long somebody ran the machine, and 44 100 stereo 16-bit frames is
/// 176 kB a second.
#[derive(Debug, Clone)]
pub struct Writer {
    info: StreamInfo,
    format: SampleFormat,
    channels: u16,
    samples: Vec<u8>,
}

impl Writer {
    /// A writer for a stream shaped like `info`, storing samples in `format`.
    #[must_use]
    pub fn new(info: StreamInfo, format: SampleFormat) -> Writer {
        Writer {
            info,
            format,
            channels: info.channels,
            samples: Vec::new(),
        }
    }

    /// How many frames have been written.
    #[must_use]
    pub fn frames(&self) -> u64 {
        let stride = self.format.bytes_per_sample() * u64::from(self.channels);
        // A zero-channel writer is a machine with no sound, not an error.
        (self.samples.len() as u64).checked_div(stride).unwrap_or(0)
    }

    /// How long the recording is, in whole milliseconds of *guest* time.
    ///
    /// Derived from the frame count and the stream's rate, never from a clock:
    /// this number is a property of what was recorded, so it is identical on
    /// every host (`CLAUDE.md`, determinism).
    #[must_use]
    pub fn duration_ms(&self) -> u64 {
        let rate = u64::from(self.info.rate_hz().max(1));
        self.frames().saturating_mul(1000) / rate
    }

    /// The complete `.wav` file.
    #[must_use]
    pub fn finish(&self) -> Vec<u8> {
        let mut out = Vec::with_capacity(HEADER_BYTES + self.samples.len());
        write_header(
            &mut out,
            self.info.rate_hz(),
            self.channels,
            self.format,
            self.samples.len() as u64,
        );
        out.extend_from_slice(&self.samples);
        out
    }
}

impl Sink for Writer {
    fn info(&self) -> StreamInfo {
        StreamInfo::new(
            self.info.rate_num,
            self.info.rate_den,
            self.channels,
            self.format,
        )
    }

    fn write(&mut self, buffer: &AudioBuffer) -> u64 {
        // A queue in another format would have to be converted, and silently
        // appending its bytes would produce noise. Refusing takes nothing,
        // which is the back pressure the trait already allows for.
        if buffer.format() != self.format || buffer.channels() != self.channels {
            return 0;
        }
        self.samples.extend_from_slice(buffer.bytes());
        buffer.frames()
    }
}
