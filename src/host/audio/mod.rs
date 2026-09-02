//! The audio seam: a guest's sound, converted for a host that can play it
//! (`ROADMAP.md` §8).
//!
//! An audio device produces whatever the silicon actually emits — an RP2A03
//! emits an unsigned level out of a non-linear DAC pair at half its CPU clock,
//! a Game Boy emits four 4-bit channels through two analogue mixers, an AC'97
//! emits 16-bit stereo already. **Turning that into something a sound card
//! wants — a signed sample at 44.1 or 48 kHz — is the host's job**, not the
//! device's, and this module is where it happens. Nothing under `dev/` may name
//! a host sample rate.
//!
//! | Type | Role |
//! | --- | --- |
//! | [`SampleFormat`] | how bytes are laid out in a host buffer |
//! | [`StreamInfo`] | rate, channels and analogue stage, without the samples |
//! | [`Pole`] | one corner of the device's own output filter, as a fact |
//! | [`AudioBuffer`] | the sample queue itself: interleaved frames a host reads |
//! | [`AudioSource`] | what an audio device offers: "here are my newest samples" |
//! | [`AudioStream`] | source + rate conversion + queue, which is the whole path |
//! | [`Sink`] | where the frames end up: a file, a card, a browser |
//!
//! # Shape
//!
//! ```text
//!   device (dev/)              seam (here)                         host
//!   ─────────────              ───────────                         ────
//!   Apu ──► u16 Q16 @ 894 886.36… Hz ─► NesAudio ─► Resampler ─► AudioBuffer
//!            (exact integer)          (AudioSource)   (f32)      ├► wav      → a file
//!                                                                └► wasm     → WebAudio
//! ```
//!
//! The device side is one small adapter per sound chip ([`nes::NesAudio`] is
//! the first); the host side never learns which machine it is listening to. A
//! Game Boy's APU, an SN76489 or an AC'97 each add an adapter and nothing else
//! changes.
//!
//! # Where the float line is
//!
//! `ROADMAP.md` §0 forbids floats in the time path and forbids wall-clock reads
//! below `host/`. Both rules are kept by putting the boundary exactly here:
//!
//! * **The device side is exact integer arithmetic.** The APU counts CPU
//!   cycles and pushes one sample per APU cycle; its rate is an exact rational
//!   derived from the board's crystal ([`StreamInfo::rate_num`] /
//!   [`StreamInfo::rate_den`] — 9 843 750 / 11 Hz on an NTSC NES, which is not
//!   an integer number of hertz and never gets rounded into one).
//! * **The rate conversion's *phase* is exact integer arithmetic too**
//!   ([`resample`]). Only sample *amplitudes* are `f32`, and an amplitude never
//!   feeds back into the machine.
//! * **Nothing here reads a clock.** How fast a host consumes the queue is the
//!   host's business; the guest cannot observe it.
//!
//! The consequence is the property that matters: **a machine's state hash does
//! not depend on whether anybody is listening.** Draining a device's sample
//! ring moves no architectural state, the ring is deliberately absent from the
//! snapshot, and the audio path never changes how far or in what steps the
//! machine is run. `tests` asserts it on a real NES.
//!
//! # The analogue stage is data, not code
//!
//! A console's output does not go straight from the DAC to the speaker: it goes
//! through an RC network on the board, and that network is as much a part of
//! how the machine sounds as the mixer is. That is a *hardware fact*, so a
//! device declares it as [`StreamInfo::output_stage`] — a list of first-order
//! [`Pole`]s with their corner frequencies — and the host implements the
//! filters ([`filter`]). A device that has no such stage, or whose stage is
//! already modelled internally, declares an empty list.
//!
//! # Where the samples go
//!
//! Two consumers exist today and neither of them is a sound card:
//!
//! * [`wav`] captures a stream as a RIFF/WAVE file — headless, dependency-free
//!   and byte-comparable, so a regression can assert what a machine *sounded*
//!   like the same way [`png`](super::display::png) asserts what it looked
//!   like.
//! * [`crate::wasm`] hands the queue's address to a page, which feeds it to
//!   WebAudio; `web/` is that page. **This is where a person actually hears
//!   it.**
//!
//! **There is deliberately no native sound card backend**, for exactly the
//! reason [`display`](super::display) has no native window. ALSA is not a file
//! you write to: it is an `ioctl`/`mmap` control protocol, and the dependency
//! policy rules out `libc` (`CLAUDE.md`), so reaching it means raw syscalls —
//! which means inline assembly, which means a **seventh** `unsafe` subsystem,
//! and `ROADMAP.md` §0 puts the ceiling at six. PulseAudio and PipeWire are
//! socket protocols with authentication handshakes; CoreAudio and WASAPI are C
//! and COM ABIs. Each is its own piece of work with its own design review, not
//! a corner of this one. Until then sound is a `.wav` or a browser tab, and
//! both are real.
//!
//! # Units
//!
//! A **frame** is one sample per channel; a **sample** is one channel's value.
//! Frame counts are `u64` (`CLAUDE.md`: sizes are never `usize`) and become
//! `usize` only where a buffer is actually indexed. Rates are exact rationals
//! in hertz, never floats.
//!
//! # Example
//!
//! ```
//! use rsemu::host::audio::{AudioBuffer, SampleFormat};
//!
//! let mut queue = AudioBuffer::new(SampleFormat::S16, 1);
//! queue.push_frame(&[i16::MAX]);
//! queue.push_frame(&[0]);
//! assert_eq!(queue.frames(), 2);
//! assert_eq!(queue.len(), 4);
//! assert_eq!(queue.sample(0, 0), Some(i16::MAX));
//! queue.consume(1);
//! assert_eq!(queue.sample(0, 0), Some(0));
//! ```

pub mod filter;
pub mod resample;
pub mod wav;

#[cfg(feature = "dev-gb")]
#[cfg_attr(docsrs, doc(cfg(feature = "dev-gb")))]
pub mod gb;

#[cfg(feature = "dev-nes-apu")]
#[cfg_attr(docsrs, doc(cfg(feature = "dev-nes-apu")))]
pub mod nes;

#[cfg(feature = "dev-sms")]
#[cfg_attr(docsrs, doc(cfg(feature = "dev-sms")))]
pub mod sms;

#[cfg(test)]
mod tests;

/// Greatest common divisor, so a device adapter reports its rate in lowest
/// terms.
///
/// Reducing is not cosmetic: [`resample`] multiplies the denominator by the
/// host rate, and an unreduced 236 250 000 / 264 — or a Game Boy's
/// 4 194 304 / 128 — would push that product further up the `u64` range for
/// nothing. Here rather than in one adapter because three of them need it —
/// and gated on those three, because a build with no sound chip in it has no
/// rate to reduce.
#[cfg(any(feature = "dev-nes-apu", feature = "dev-gb", feature = "dev-sms"))]
pub(crate) const fn gcd(mut a: u64, mut b: u64) -> u64 {
    while b != 0 {
        let t = a % b;
        a = b;
        b = t;
    }
    if a == 0 { 1 } else { a }
}

use alloc::boxed::Box;
use alloc::vec::Vec;
use core::fmt;

use resample::Resampler;

// ---------------------------------------------------------------------------
// Sample formats
// ---------------------------------------------------------------------------

/// How an [`AudioBuffer`]'s bytes are laid out.
///
/// An extensible enumeration in the `pktkit` style rather than a Rust `enum`
/// (`CLAUDE.md`): a backend that needs 24-bit packed or big-endian samples adds
/// a constant without breaking every `match` in the tree. The named constants
/// are **memory order** and are little-endian, because every consumer this
/// module has — RIFF/WAVE, a `Float32Array` over wasm memory, and every PC
/// sound card — is little-endian, and byte order is a property of the file or
/// the buffer rather than of the host.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
#[repr(transparent)]
pub struct SampleFormat(pub u16);

impl SampleFormat {
    /// Signed 16-bit, little-endian. What WAV, every sound card and every
    /// vintage DAC's dynamic range actually is; the default everywhere here.
    pub const S16: SampleFormat = SampleFormat(0);

    /// IEEE-754 32-bit float in `[-1.0, 1.0]`, little-endian. What WebAudio's
    /// `AudioBuffer` and every modern mixer want.
    pub const F32: SampleFormat = SampleFormat(1);

    /// Unsigned 8-bit, `0x80` centred — WAV's 8-bit form, and what a 1980s
    /// sample looks like on disk.
    pub const U8: SampleFormat = SampleFormat(2);

    /// How many bytes one sample of one channel occupies.
    #[inline]
    #[must_use]
    pub const fn bytes_per_sample(self) -> u64 {
        match self {
            SampleFormat::U8 => 1,
            SampleFormat::F32 => 4,
            // S16 and anything unknown: two bytes. An unknown constant gets a
            // wrong-sounding stream rather than a panic mid-frame, exactly as
            // an unknown `PixelFormat` gets a wrong-looking picture.
            _ => 2,
        }
    }

    /// A short name for diagnostics.
    #[must_use]
    pub const fn name(self) -> &'static str {
        match self {
            SampleFormat::S16 => "s16",
            SampleFormat::F32 => "f32",
            SampleFormat::U8 => "u8",
            _ => "unknown",
        }
    }

    /// Write one sample, given as a normalised `f32` in `[-1.0, 1.0]`, into
    /// `dst`, which must be at least [`bytes_per_sample`] long.
    ///
    /// The single place a float becomes bytes; every writer below goes through
    /// it, so adding a format is one arm here and one in
    /// [`bytes_per_sample`](SampleFormat::bytes_per_sample).
    ///
    /// [`bytes_per_sample`]: SampleFormat::bytes_per_sample
    #[inline]
    fn encode(self, value: f32, dst: &mut [u8]) {
        let clamped = clamp_unit(value);
        match self {
            SampleFormat::F32 => dst[..4].copy_from_slice(&clamped.to_le_bytes()),
            SampleFormat::U8 => {
                // 0x80 is silence, so the full-scale negative is 0 and the
                // full-scale positive is 0xff.
                let scaled = round_to_i32(clamped * 127.0) + 128;
                dst[0] = scaled.clamp(0, 255) as u8;
            }
            _ => {
                // 32767 rather than 32768, so +1.0 and -1.0 are symmetric and
                // neither wraps.
                let scaled = round_to_i32(clamped * 32767.0).clamp(-32768, 32767) as i16;
                dst[..2].copy_from_slice(&scaled.to_le_bytes());
            }
        }
    }

    /// Read one sample back as a normalised `f32`.
    #[inline]
    fn decode(self, src: &[u8]) -> f32 {
        match self {
            SampleFormat::F32 => f32::from_le_bytes([src[0], src[1], src[2], src[3]]),
            SampleFormat::U8 => (f32::from(src[0]) - 128.0) / 127.0,
            _ => f32::from(i16::from_le_bytes([src[0], src[1]])) / 32767.0,
        }
    }
}

impl fmt::Display for SampleFormat {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(self.name())
    }
}

/// `x` held inside `[-1.0, 1.0]`, with `NaN` treated as silence.
///
/// Written out rather than `f32::clamp` because `clamp` panics on a `NaN`
/// bound and, more to the point, a comparison chain is the one form that is
/// definitely available in a `no_std` build with no `libm`.
#[inline]
fn clamp_unit(x: f32) -> f32 {
    if x > 1.0 {
        1.0
    } else if x > -1.0 {
        x
    } else if x <= -1.0 {
        -1.0
    } else {
        // Only NaN reaches here, every comparison above having been false.
        0.0
    }
}

/// `x` rounded half-away-from-zero.
///
/// `f32::round` lives in `std`, and this module compiles in a `no_std` build.
#[inline]
fn round_to_i32(x: f32) -> i32 {
    if x >= 0.0 {
        (x + 0.5) as i32
    } else {
        (x - 0.5) as i32
    }
}

// ---------------------------------------------------------------------------
// The device's analogue output stage
// ---------------------------------------------------------------------------

/// Which way a [`Pole`] rolls off.
///
/// Extensible for the same reason [`SampleFormat`] is: a device with a
/// resonant or second-order stage adds a constant rather than an `enum` arm.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
#[repr(transparent)]
pub struct PoleKind(pub u16);

impl PoleKind {
    /// Passes everything above the corner; blocks DC. A coupling capacitor.
    pub const HIGH_PASS: PoleKind = PoleKind(0);

    /// Passes everything below the corner. An RC lag.
    pub const LOW_PASS: PoleKind = PoleKind(1);

    /// A short name for diagnostics.
    #[must_use]
    pub const fn name(self) -> &'static str {
        match self {
            PoleKind::HIGH_PASS => "high-pass",
            PoleKind::LOW_PASS => "low-pass",
            _ => "unknown",
        }
    }
}

impl fmt::Display for PoleKind {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(self.name())
    }
}

/// One first-order corner of a device's analogue output network.
///
/// A **fact about the board**, declared by the device's adapter and implemented
/// by the host — the same division of labour that keeps colour out of `dev/`.
/// The corner frequency comes from a schematic or a datasheet; the difference
/// equation that realises it is in [`filter`].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Pole {
    /// Which way it rolls off.
    pub kind: PoleKind,
    /// The −3 dB corner, in hertz.
    pub corner_hz: u32,
}

impl Pole {
    /// A high-pass corner at `hz`.
    #[must_use]
    pub const fn high_pass(hz: u32) -> Pole {
        Pole {
            kind: PoleKind::HIGH_PASS,
            corner_hz: hz,
        }
    }

    /// A low-pass corner at `hz`.
    #[must_use]
    pub const fn low_pass(hz: u32) -> Pole {
        Pole {
            kind: PoleKind::LOW_PASS,
            corner_hz: hz,
        }
    }
}

// ---------------------------------------------------------------------------
// Stream description
// ---------------------------------------------------------------------------

/// A stream's shape, without its samples.
///
/// What an [`AudioSource`] answers when asked what it produces, so a host can
/// build a rate converter before the first sample exists.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct StreamInfo {
    /// Numerator of the exact sample rate, in hertz.
    ///
    /// A rational rather than an integer because console clocks are rational:
    /// an NTSC NES produces samples at 9 843 750 / 11 Hz — 894 886.36… — and
    /// rounding that to 894 886 would put the recording 0.4 ppm off pitch
    /// *and* throw away the exactness `ROADMAP.md` §4.2 exists to preserve.
    pub rate_num: u64,
    /// Denominator of the exact sample rate. Never zero.
    pub rate_den: u64,
    /// How many channels one frame holds. `1` is mono.
    pub channels: u16,
    /// The format the device's adapter converts into most cheaply. A host may
    /// ask for a different one; every adapter here supports all of them.
    pub preferred_format: SampleFormat,
    /// The analogue network the chip's output actually passes through, as a
    /// list of first-order corners the host is to apply at the *device's* rate.
    /// Empty means "the samples are already what comes out of the box".
    pub output_stage: &'static [Pole],
}

impl StreamInfo {
    /// A stream at exactly `rate_num / rate_den` hertz, with no analogue stage.
    ///
    /// # Panics
    ///
    /// Never. A zero denominator is corrected to one rather than trapping: a
    /// silent stream is a recoverable configuration mistake and a panic in a
    /// device adapter is not.
    #[must_use]
    pub const fn new(
        rate_num: u64,
        rate_den: u64,
        channels: u16,
        preferred_format: SampleFormat,
    ) -> StreamInfo {
        StreamInfo {
            rate_num,
            rate_den: if rate_den == 0 { 1 } else { rate_den },
            channels,
            preferred_format,
            output_stage: &[],
        }
    }

    /// The same stream, declaring the board's analogue output network.
    #[must_use]
    pub const fn with_output_stage(mut self, stage: &'static [Pole]) -> StreamInfo {
        self.output_stage = stage;
        self
    }

    /// The rate rounded to whole hertz, for a diagnostic or a WAV header.
    ///
    /// Rounding happens **here and nowhere else**: everything that computes
    /// with the rate uses the exact pair.
    #[must_use]
    pub const fn rate_hz(self) -> u32 {
        let scaled = (self.rate_num * 2 + self.rate_den) / (self.rate_den * 2);
        if scaled > u32::MAX as u64 {
            u32::MAX
        } else {
            scaled as u32
        }
    }

    /// Bytes one frame occupies in [`preferred_format`](Self::preferred_format).
    #[must_use]
    pub const fn frame_bytes(self) -> u64 {
        self.preferred_format.bytes_per_sample() * self.channels as u64
    }
}

impl fmt::Display for StreamInfo {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(
            f,
            "{} Hz ({}/{}), {} ch, {}",
            self.rate_hz(),
            self.rate_num,
            self.rate_den,
            self.channels,
            self.preferred_format
        )
    }
}

// ---------------------------------------------------------------------------
// Buffers
// ---------------------------------------------------------------------------

/// A host-side sample queue: interleaved frames in one [`SampleFormat`].
///
/// The analogue of [`Surface`](super::display::Surface), with one deliberate
/// difference: a picture is a *snapshot* the producer overwrites, and sound is
/// a *stream* nobody may drop a hole in. So this is a FIFO — frames are pushed
/// at the back and [`consume`](Self::consume)d from the front — and its
/// address is only stable until it next has to grow, which is why an embedder
/// asks for the pointer immediately before reading rather than caching it.
#[derive(Debug, Clone, PartialEq)]
pub struct AudioBuffer {
    format: SampleFormat,
    channels: u16,
    bytes: Vec<u8>,
}

impl AudioBuffer {
    /// An empty queue in `format` with `channels` channels.
    ///
    /// A zero-channel queue is legal and holds nothing: a machine with no sound
    /// is something a host has to play silence for, not an error.
    #[must_use]
    pub fn new(format: SampleFormat, channels: u16) -> AudioBuffer {
        AudioBuffer {
            format,
            channels,
            bytes: Vec::new(),
        }
    }

    /// A queue with no samples and no channels at all.
    ///
    /// `const`, because a host that keeps its queue in a `static` — the wasm
    /// module does — needs one before any machine exists.
    #[must_use]
    pub const fn empty() -> AudioBuffer {
        AudioBuffer {
            format: SampleFormat::F32,
            channels: 0,
            bytes: Vec::new(),
        }
    }

    /// The format the bytes are in.
    #[inline]
    #[must_use]
    pub const fn format(&self) -> SampleFormat {
        self.format
    }

    /// How many channels one frame holds.
    #[inline]
    #[must_use]
    pub const fn channels(&self) -> u16 {
        self.channels
    }

    /// Bytes one frame occupies.
    #[inline]
    #[must_use]
    pub const fn frame_bytes(&self) -> u64 {
        self.format.bytes_per_sample() * self.channels as u64
    }

    /// How many whole frames are queued.
    #[inline]
    #[must_use]
    pub fn frames(&self) -> u64 {
        // `checked_div` rather than a guard: a zero-channel queue has a zero
        // stride, which is legal and holds no frames.
        (self.bytes.len() as u64)
            .checked_div(self.frame_bytes())
            .unwrap_or(0)
    }

    /// Total bytes queued.
    #[inline]
    #[must_use]
    pub fn len(&self) -> u64 {
        self.bytes.len() as u64
    }

    /// Whether nothing is queued.
    #[inline]
    #[must_use]
    pub fn is_empty(&self) -> bool {
        self.bytes.is_empty()
    }

    /// The raw bytes, oldest frame first.
    #[inline]
    #[must_use]
    pub fn bytes(&self) -> &[u8] {
        &self.bytes
    }

    /// The address of the first queued byte, for an embedder that reads the
    /// queue out of exported memory (`ROADMAP.md` §11.5).
    ///
    /// Valid until the queue next grows, so read it immediately before use.
    #[inline]
    #[must_use]
    pub fn as_ptr(&self) -> *const u8 {
        self.bytes.as_ptr()
    }

    /// Discard everything queued, keeping the allocation.
    pub fn clear(&mut self) {
        self.bytes.clear();
    }

    /// Reshape to `format`/`channels`, discarding anything queued.
    ///
    /// Anything already queued is in the *old* format, so it cannot be kept:
    /// reinterpreting it would be a click at best.
    pub fn reshape(&mut self, format: SampleFormat, channels: u16) {
        if self.format == format && self.channels == channels {
            return;
        }
        self.format = format;
        self.channels = channels;
        self.bytes.clear();
    }

    /// Append one frame, given as one normalised `f32` per channel.
    ///
    /// A short slice is padded with silence and a long one is truncated, so a
    /// producer that disagrees with the queue about channel count makes a
    /// wrong-sounding stream rather than a panic on the emulation thread.
    pub fn push_normalised(&mut self, frame: &[f32]) {
        let width = self.format.bytes_per_sample() as usize;
        for channel in 0..usize::from(self.channels) {
            let value = frame.get(channel).copied().unwrap_or(0.0);
            let at = self.bytes.len();
            self.bytes.resize(at + width, 0);
            self.format.encode(value, &mut self.bytes[at..]);
        }
    }

    /// Append one frame of signed 16-bit samples.
    pub fn push_frame(&mut self, frame: &[i16]) {
        let width = self.format.bytes_per_sample() as usize;
        for channel in 0..usize::from(self.channels) {
            let value = f32::from(frame.get(channel).copied().unwrap_or(0)) / 32767.0;
            let at = self.bytes.len();
            self.bytes.resize(at + width, 0);
            self.format.encode(value, &mut self.bytes[at..]);
        }
    }

    /// Read one channel of one queued frame back as signed 16-bit, or `None`
    /// past the end.
    #[must_use]
    pub fn sample(&self, frame: u64, channel: u16) -> Option<i16> {
        if channel >= self.channels || frame >= self.frames() {
            return None;
        }
        let width = self.format.bytes_per_sample();
        let at = (frame * self.frame_bytes() + u64::from(channel) * width) as usize;
        let value = self.format.decode(&self.bytes[at..]);
        Some(round_to_i32(clamp_unit(value) * 32767.0).clamp(-32768, 32767) as i16)
    }

    /// Drop the oldest `frames` frames, returning how many were actually
    /// dropped.
    pub fn consume(&mut self, frames: u64) -> u64 {
        let available = self.frames();
        let taken = frames.min(available);
        if taken == available {
            // The common case — a host that drained everything — and the one
            // that keeps the address stable.
            self.bytes.clear();
        } else if taken > 0 {
            self.bytes.drain(..(taken * self.frame_bytes()) as usize);
        }
        taken
    }

    /// FNV-1a over the queued bytes: the audio equivalent of a frame hash, for
    /// a regression that asserts *what a machine sounded like* rather than only
    /// that it made a noise (`ROADMAP.md` §12).
    ///
    /// The same function [`Surface::hash`](super::display::Surface::hash) uses,
    /// for the same reasons.
    #[must_use]
    pub fn hash(&self) -> u64 {
        let mut h: u64 = 0xcbf2_9ce4_8422_2325;
        for b in &self.bytes {
            h ^= u64::from(*b);
            h = h.wrapping_mul(0x0000_0100_0000_01b3);
        }
        h
    }
}

// ---------------------------------------------------------------------------
// The source seam
// ---------------------------------------------------------------------------

/// An audio device, as the host sees it.
///
/// `Send + Sync` like every device-facing trait (`ROADMAP.md` §0): the
/// emulation thread produces samples and an audio thread drains them.
///
/// Implementors live beside the host, not beside the device: they hold whatever
/// handle the device offers (an `Arc<Apu>`, later an `Arc<GbApu>`) and do the
/// normalisation the device deliberately does not.
///
/// # The contract
///
/// [`drain`](Self::drain) is a **pull**, and it is the only direction that
/// works: a device must never call into a host sink, because it would be doing
/// it from inside its own critical section on the emulation thread, which is
/// exactly what §4.7's re-entrancy contract forbids. So the device buffers a
/// bounded number of samples and the host takes them whenever it likes.
/// Whether the host keeps up is the host's problem and is invisible to the
/// guest — [`dropped`](Self::dropped) reports it as a diagnostic, never as
/// state.
pub trait AudioSource: Send + Sync + fmt::Debug {
    /// What this device produces.
    fn info(&self) -> StreamInfo;

    /// Move every sample the device has buffered into `out`, oldest first,
    /// interleaved by channel, and return how many **frames** were appended.
    ///
    /// Signed 16-bit is the seam's canonical unit: it is what a vintage DAC's
    /// dynamic range actually is, it converts to every [`SampleFormat`]
    /// exactly, and it keeps this trait free of floats so an adapter can live
    /// as close to the device as it likes.
    fn drain(&self, out: &mut Vec<i16>) -> u64;

    /// How many frames the device has overwritten because nobody drained it.
    ///
    /// Monotonic, and a **diagnostic**: it is not architectural state, it is
    /// not in any snapshot, and no guest can observe it.
    fn dropped(&self) -> u64 {
        0
    }
}

// ---------------------------------------------------------------------------
// The sink seam
// ---------------------------------------------------------------------------

/// Where frames end up: a file, a card, a browser.
///
/// `Send` but not `Sync` — a sink is owned by whoever is playing, and two
/// threads writing one card interleaved is not a thing to make representable.
///
/// [`wav::Writer`] is the only implementor today. A native backend would be
/// another, and would need nothing here to change.
pub trait Sink: Send + fmt::Debug {
    /// What this sink accepts. A stream in another shape must be converted
    /// before it is offered.
    fn info(&self) -> StreamInfo;

    /// Take up to every frame in `buffer`, returning how many were accepted.
    ///
    /// Fewer than offered means back pressure, which is the sink's own
    /// business: the caller keeps the remainder queued and tries again. It is
    /// never reported to the guest.
    fn write(&mut self, buffer: &AudioBuffer) -> u64;
}

// ---------------------------------------------------------------------------
// The whole path
// ---------------------------------------------------------------------------

/// How many frames a stream queues before it starts dropping the oldest.
///
/// Two seconds at 48 kHz. A host that has stopped consuming is either paused or
/// broken; growing without bound so that it can eventually play two minutes of
/// stale audio helps nobody, and in a browser tab it is a memory leak with a
/// soundtrack.
pub const DEFAULT_QUEUE_FRAMES: u64 = 96_000;

/// A device, a rate conversion and a queue: the entire host audio path.
///
/// This is what a host actually holds. [`pull`](Self::pull) takes whatever the
/// device has buffered, runs it through the board's analogue stage and the rate
/// converter, and appends the result to [`buffer`](Self::buffer) — from which
/// the host copies it to a file, a card or a page.
///
/// **Nothing here reads a clock or advances a machine.** When to call
/// [`pull`](Self::pull) is the host's decision, and it must be one the host
/// would have made anyway: a drain cadence that changed how far the machine was
/// run in one go would change the machine, and the whole point of this module
/// is that it cannot.
#[derive(Debug)]
pub struct AudioStream {
    source: Box<dyn AudioSource>,
    resampler: Resampler,
    buffer: AudioBuffer,
    scratch: Vec<i16>,
    out_rate: u32,
    limit: u64,
    overflowed: u64,
    produced: u64,
}

impl AudioStream {
    /// Watch `source`, converting to `out_rate` hertz in `format`.
    ///
    /// `out_rate` is the *host's* rate — 44 100 for a file, whatever
    /// `AudioContext.sampleRate` says in a browser — and is the only integer
    /// rate in the path. A rate of zero is corrected to one, for the same
    /// reason [`StreamInfo::new`] corrects a zero denominator.
    #[must_use]
    pub fn new(source: Box<dyn AudioSource>, out_rate: u32, format: SampleFormat) -> AudioStream {
        let info = source.info();
        let out_rate = out_rate.max(1);
        AudioStream {
            resampler: Resampler::new(info, out_rate),
            buffer: AudioBuffer::new(format, info.channels),
            scratch: Vec::new(),
            source,
            out_rate,
            limit: DEFAULT_QUEUE_FRAMES,
            overflowed: 0,
            produced: 0,
        }
    }

    /// What the device produces.
    #[must_use]
    pub fn source_info(&self) -> StreamInfo {
        self.source.info()
    }

    /// What this stream hands out: the host rate, the host format, the device's
    /// channel count.
    #[must_use]
    pub fn info(&self) -> StreamInfo {
        let source = self.source.info();
        StreamInfo::new(
            u64::from(self.out_rate),
            1,
            source.channels,
            self.buffer.format(),
        )
    }

    /// The host output rate in hertz.
    #[inline]
    #[must_use]
    pub const fn rate(&self) -> u32 {
        self.out_rate
    }

    /// Change the host output rate, discarding anything queued.
    ///
    /// A browser does not know its `AudioContext`'s rate until it has one, and
    /// on some machines that is 44 100 and on others 48 000, so this has to be
    /// settable after the stream exists. Everything queued is dropped rather
    /// than replayed at the wrong pitch.
    pub fn set_rate(&mut self, out_rate: u32) {
        let out_rate = out_rate.max(1);
        if out_rate == self.out_rate {
            return;
        }
        self.out_rate = out_rate;
        self.resampler = Resampler::new(self.source.info(), out_rate);
        self.buffer.clear();
    }

    /// How many frames may queue before the oldest are dropped.
    pub const fn set_limit_frames(&mut self, frames: u64) {
        self.limit = frames;
    }

    /// The queue, for a host that reads it directly.
    #[inline]
    #[must_use]
    pub const fn buffer(&self) -> &AudioBuffer {
        &self.buffer
    }

    /// Drop the oldest `frames` frames from the queue.
    pub fn consume(&mut self, frames: u64) -> u64 {
        self.buffer.consume(frames)
    }

    /// Total frames this stream has produced since it was built. Monotonic.
    #[inline]
    #[must_use]
    pub const fn produced(&self) -> u64 {
        self.produced
    }

    /// Frames lost, at either end: samples the device overwrote because nobody
    /// pulled, plus frames this queue dropped because nobody consumed.
    ///
    /// Purely diagnostic. Non-zero means a host that is not keeping up, never a
    /// machine that ran differently.
    #[must_use]
    pub fn dropped(&self) -> u64 {
        self.source.dropped().saturating_add(self.overflowed)
    }

    /// Take everything the device has buffered and append the converted frames
    /// to the queue. Returns how many frames were appended.
    pub fn pull(&mut self) -> u64 {
        self.scratch.clear();
        let taken = self.source.drain(&mut self.scratch);
        if taken == 0 {
            return 0;
        }
        let before = self.buffer.frames();
        // `scratch` and `buffer` are disjoint fields, so the borrow checker is
        // satisfied by naming them separately rather than by a temporary.
        self.resampler.process(&self.scratch, &mut self.buffer);
        let appended = self.buffer.frames().saturating_sub(before);
        self.produced = self.produced.saturating_add(appended);

        let queued = self.buffer.frames();
        if queued > self.limit {
            self.overflowed = self
                .overflowed
                .saturating_add(self.buffer.consume(queued - self.limit));
        }
        appended
    }

    /// Hand everything queued to `sink`, keeping whatever it would not take.
    /// Returns how many frames it accepted.
    pub fn drain_to(&mut self, sink: &mut dyn Sink) -> u64 {
        let accepted = sink.write(&self.buffer);
        self.buffer.consume(accepted);
        accepted
    }
}
