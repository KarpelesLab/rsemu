//! Tests for the audio seam.
//!
//! Three things are being asserted, in increasing order of what they are worth:
//! that the arithmetic is right, that a real APU's samples come through it, and
//! that **turning audio on does not change the machine**. The last one is the
//! reason the module is shaped the way it is.

use alloc::boxed::Box;
use alloc::vec::Vec;

use super::filter::{Chain, OnePole};
use super::resample::Resampler;
use super::{
    AudioBuffer, AudioSource, AudioStream, Pole, PoleKind, SampleFormat, Sink, StreamInfo, wav,
};

// ---------------------------------------------------------------------------
// formats and buffers
// ---------------------------------------------------------------------------

#[test]
fn every_format_round_trips_through_a_buffer() {
    for format in [SampleFormat::S16, SampleFormat::F32, SampleFormat::U8] {
        let mut buffer = AudioBuffer::new(format, 2);
        buffer.push_frame(&[0, 32767]);
        buffer.push_frame(&[-32767, 0]);
        assert_eq!(buffer.frames(), 2, "{format}");
        assert_eq!(buffer.len(), 4 * format.bytes_per_sample(), "{format}");
        assert_eq!(buffer.sample(0, 0), Some(0), "{format}");
        // Full scale survives every format; 8-bit quantises, so allow a step.
        let top = buffer.sample(0, 1).expect("a sample");
        assert!(top > 32000, "{format} lost full scale: {top}");
        let bottom = buffer.sample(1, 0).expect("a sample");
        assert!(bottom < -32000, "{format} lost full scale: {bottom}");
    }
}

#[test]
fn consuming_takes_from_the_front() {
    let mut buffer = AudioBuffer::new(SampleFormat::S16, 1);
    for i in 0..8 {
        buffer.push_frame(&[i * 1000]);
    }
    assert_eq!(buffer.consume(3), 3);
    assert_eq!(buffer.frames(), 5);
    assert_eq!(buffer.sample(0, 0), Some(3000));
    // Asking for more than is there takes what is there and says so.
    assert_eq!(buffer.consume(99), 5);
    assert!(buffer.is_empty());
}

#[test]
fn a_reshape_discards_rather_than_reinterprets() {
    let mut buffer = AudioBuffer::new(SampleFormat::S16, 1);
    buffer.push_frame(&[1234]);
    buffer.reshape(SampleFormat::F32, 2);
    assert!(buffer.is_empty());
    assert_eq!(buffer.channels(), 2);
    assert_eq!(buffer.frame_bytes(), 8);
}

#[test]
fn a_zero_channel_buffer_is_silence_rather_than_a_panic() {
    let mut buffer = AudioBuffer::new(SampleFormat::S16, 0);
    buffer.push_frame(&[1, 2, 3]);
    assert_eq!(buffer.frames(), 0);
    assert_eq!(buffer.sample(0, 0), None);
}

// ---------------------------------------------------------------------------
// stream description
// ---------------------------------------------------------------------------

#[test]
fn a_rational_rate_rounds_only_where_it_is_asked_to() {
    // An NTSC NES: 9 843 750 / 11 Hz = 894 886.36…
    let info = StreamInfo::new(9_843_750, 11, 1, SampleFormat::S16);
    assert_eq!(info.rate_hz(), 894_886);
    // And the exact pair is untouched, which is the whole point.
    assert_eq!((info.rate_num, info.rate_den), (9_843_750, 11));
}

#[test]
fn a_zero_denominator_becomes_silence_not_a_division_trap() {
    let info = StreamInfo::new(48_000, 0, 1, SampleFormat::S16);
    assert_eq!(info.rate_den, 1);
    assert_eq!(info.rate_hz(), 48_000);
}

// ---------------------------------------------------------------------------
// filters
// ---------------------------------------------------------------------------

/// A high-pass must eventually reject a constant: that is what removes the
/// 2A03's DC offset, and it is the difference between a thump and a note.
#[test]
fn a_high_pass_rejects_dc() {
    let mut pole = OnePole::new(Pole::high_pass(90), 48_000, 1);
    let mut last = 0.0;
    for _ in 0..48_000 {
        last = pole.step(0.5);
    }
    assert!(last.abs() < 0.01, "DC survived the high-pass: {last}");
}

/// And a low-pass must eventually pass one.
#[test]
fn a_low_pass_passes_dc() {
    let mut pole = OnePole::new(Pole::low_pass(1_000), 48_000, 1);
    let mut last = 0.0;
    for _ in 0..48_000 {
        last = pole.step(0.5);
    }
    assert!(
        (last - 0.5).abs() < 0.01,
        "DC did not reach the output: {last}"
    );
}

/// A corner at or above Nyquist cannot be realised by a one-pole section, so it
/// degrades to a pass-through rather than to an unstable filter.
#[test]
fn an_unrealisable_corner_becomes_a_pass_through() {
    let mut pole = OnePole::new(Pole::low_pass(30_000), 8_000, 1);
    assert_eq!(pole.coefficient(), 1.0);
    assert_eq!(pole.step(0.25), 0.25);

    let mut zero = OnePole::new(Pole::low_pass(0), 48_000, 1);
    assert_eq!(zero.step(0.25), 0.25);
}

#[test]
fn a_chain_applies_every_section_and_an_empty_one_is_the_identity() {
    static STAGE: &[Pole] = &[Pole::high_pass(90), Pole::low_pass(14_000)];
    let info = StreamInfo::new(48_000, 1, 1, SampleFormat::S16).with_output_stage(STAGE);
    let mut chain = Chain::for_stream(info);
    assert_eq!(chain.len(), 2);
    // The first sample through a high-pass passes almost unchanged; it is the
    // steady state that gets rejected.
    assert!(chain.step(1.0) > 0.0);

    let mut empty = Chain::passthrough();
    assert!(empty.is_empty());
    assert_eq!(empty.step(0.375), 0.375);
}

#[test]
fn pole_kinds_name_themselves() {
    assert_eq!(PoleKind::HIGH_PASS.name(), "high-pass");
    assert_eq!(PoleKind::LOW_PASS.name(), "low-pass");
    assert_eq!(PoleKind(99).name(), "unknown");
}

// ---------------------------------------------------------------------------
// resampling
// ---------------------------------------------------------------------------

/// The phase is exact integer arithmetic, so the frame count after a long run
/// is the rational's own answer and not a float's approximation to it.
#[test]
fn decimation_produces_exactly_the_rational_number_of_frames() {
    // 894 886.36… Hz in, 48 000 out: 9 843 750 / (48 000 × 11) = 18.643…
    let info = StreamInfo::new(9_843_750, 11, 1, SampleFormat::S16);
    let mut resampler = Resampler::new(info, 48_000);
    assert_eq!(resampler.ratio(), (9_843_750, 528_000));

    let input: Vec<i16> = (0..894_886).map(|i| ((i % 100) as i16) * 300).collect();
    let mut out = AudioBuffer::new(SampleFormat::S16, 1);
    resampler.process(&input, &mut out);

    // 894 886 input frames × 528 000 / 9 843 750 = 48 000.0 - a hair.
    let expected = 894_886u64 * 528_000 / 9_843_750;
    assert_eq!(out.frames(), expected);
    assert_eq!(out.frames(), 47_999);
}

/// Split the same input across two calls and the result is identical: the
/// accumulator carries across, so a host's pull cadence never changes the
/// audio.
#[test]
fn the_pull_cadence_does_not_change_the_samples() {
    let info = StreamInfo::new(9_843_750, 11, 1, SampleFormat::S16);
    let input: Vec<i16> = (0..50_000)
        .map(|i| ((i % 71) as i16) * 400 - 14_000)
        .collect();

    let mut whole = AudioBuffer::new(SampleFormat::S16, 1);
    Resampler::new(info, 48_000).process(&input, &mut whole);

    let mut piecemeal = AudioBuffer::new(SampleFormat::S16, 1);
    let mut resampler = Resampler::new(info, 48_000);
    for chunk in input.chunks(997) {
        resampler.process(chunk, &mut piecemeal);
    }

    assert_eq!(whole.frames(), piecemeal.frames());
    assert_eq!(whole.hash(), piecemeal.hash());
}

/// Upsampling holds the last value rather than emitting silence between frames.
#[test]
fn upsampling_holds_rather_than_gapping() {
    let info = StreamInfo::new(8_000, 1, 1, SampleFormat::S16);
    let mut resampler = Resampler::new(info, 48_000);
    let mut out = AudioBuffer::new(SampleFormat::S16, 1);
    resampler.process(&[20_000; 10], &mut out);
    assert_eq!(out.frames(), 60);
    for frame in 0..out.frames() {
        let value = out.sample(frame, 0).expect("a sample");
        assert!(value > 19_000, "frame {frame} gapped: {value}");
    }
}

#[test]
fn stereo_channels_stay_apart() {
    let info = StreamInfo::new(96_000, 1, 2, SampleFormat::S16);
    let mut resampler = Resampler::new(info, 48_000);
    assert_eq!(resampler.channels(), 2);
    let input: Vec<i16> = core::iter::repeat_n([10_000i16, -10_000], 64)
        .flatten()
        .collect();
    let mut out = AudioBuffer::new(SampleFormat::S16, 2);
    resampler.process(&input, &mut out);
    assert_eq!(out.frames(), 32);
    assert!(out.sample(10, 0).expect("left") > 9_000);
    assert!(out.sample(10, 1).expect("right") < -9_000);
}

// ---------------------------------------------------------------------------
// wav
// ---------------------------------------------------------------------------

#[test]
fn a_wav_header_says_what_the_samples_are() {
    let info = StreamInfo::new(44_100, 1, 2, SampleFormat::S16);
    let mut queue = AudioBuffer::new(SampleFormat::S16, 2);
    for i in 0..10 {
        queue.push_frame(&[i * 100, -i * 100]);
    }
    let file = wav::encode(info, &queue);

    assert_eq!(&file[0..4], b"RIFF");
    assert_eq!(&file[8..12], b"WAVE");
    assert_eq!(&file[12..16], b"fmt ");
    assert_eq!(u32::from_le_bytes(file[16..20].try_into().unwrap()), 16);
    assert_eq!(u16::from_le_bytes(file[20..22].try_into().unwrap()), 1); // PCM
    assert_eq!(u16::from_le_bytes(file[22..24].try_into().unwrap()), 2); // stereo
    assert_eq!(u32::from_le_bytes(file[24..28].try_into().unwrap()), 44_100);
    assert_eq!(
        u32::from_le_bytes(file[28..32].try_into().unwrap()),
        44_100 * 4
    );
    assert_eq!(u16::from_le_bytes(file[32..34].try_into().unwrap()), 4);
    assert_eq!(u16::from_le_bytes(file[34..36].try_into().unwrap()), 16);
    assert_eq!(&file[36..40], b"data");
    assert_eq!(u32::from_le_bytes(file[40..44].try_into().unwrap()), 40);
    assert_eq!(file.len(), 44 + 40);
    // And the RIFF size covers everything after its own field.
    assert_eq!(
        u32::from_le_bytes(file[4..8].try_into().unwrap()) as usize,
        file.len() - 8
    );
}

#[test]
fn a_float_wav_is_tagged_as_one() {
    let info = StreamInfo::new(48_000, 1, 1, SampleFormat::F32);
    let mut queue = AudioBuffer::new(SampleFormat::F32, 1);
    queue.push_normalised(&[0.5]);
    let file = wav::encode(info, &queue);
    assert_eq!(u16::from_le_bytes(file[20..22].try_into().unwrap()), 3);
    assert_eq!(u16::from_le_bytes(file[34..36].try_into().unwrap()), 32);
}

#[test]
fn a_writer_accumulates_and_refuses_a_stream_it_cannot_store() {
    let info = StreamInfo::new(44_100, 1, 1, SampleFormat::S16);
    let mut writer = wav::Writer::new(info, SampleFormat::S16);
    assert_eq!(writer.info().channels, 1);

    let mut queue = AudioBuffer::new(SampleFormat::S16, 1);
    for _ in 0..44_100 {
        queue.push_frame(&[1000]);
    }
    assert_eq!(writer.write(&queue), 44_100);
    assert_eq!(writer.frames(), 44_100);
    assert_eq!(writer.duration_ms(), 1000);

    // The wrong format takes nothing rather than appending noise.
    let mut wrong = AudioBuffer::new(SampleFormat::F32, 1);
    wrong.push_frame(&[1000]);
    assert_eq!(writer.write(&wrong), 0);
    assert_eq!(writer.frames(), 44_100);

    assert_eq!(writer.finish().len(), 44 + 44_100 * 2);
}

// ---------------------------------------------------------------------------
// the stream as a whole
// ---------------------------------------------------------------------------

/// A source that hands out a fixed sawtooth, so the plumbing can be tested
/// without a machine.
#[derive(Debug)]
struct Saw {
    info: StreamInfo,
}

impl AudioSource for Saw {
    fn info(&self) -> StreamInfo {
        self.info
    }

    fn drain(&self, out: &mut Vec<i16>) -> u64 {
        for i in 0..10_000i32 {
            out.push((i % 2000) as i16 * 16 - 16_000);
        }
        10_000
    }
}

#[test]
fn a_stream_converts_and_queues() {
    let info = StreamInfo::new(9_843_750, 11, 1, SampleFormat::S16);
    let mut stream = AudioStream::new(Box::new(Saw { info }), 48_000, SampleFormat::F32);
    assert_eq!(stream.rate(), 48_000);
    assert_eq!(stream.info().rate_hz(), 48_000);
    assert_eq!(stream.buffer().format(), SampleFormat::F32);

    let appended = stream.pull();
    assert!(appended > 500, "10 000 input frames gave {appended}");
    assert_eq!(stream.produced(), appended);
    assert_eq!(stream.buffer().frames(), appended);

    // And the queue drains where it is told to.
    assert_eq!(stream.consume(appended), appended);
    assert!(stream.buffer().is_empty());
}

#[test]
fn a_stream_bounds_its_queue_and_reports_what_it_lost() {
    let info = StreamInfo::new(48_000, 1, 1, SampleFormat::S16);
    let mut stream = AudioStream::new(Box::new(Saw { info }), 48_000, SampleFormat::S16);
    stream.set_limit_frames(1000);
    stream.pull();
    assert_eq!(stream.buffer().frames(), 1000);
    assert!(stream.dropped() >= 9000, "{}", stream.dropped());
}

#[test]
fn changing_the_rate_rebuilds_the_converter() {
    let info = StreamInfo::new(9_843_750, 11, 1, SampleFormat::S16);
    let mut stream = AudioStream::new(Box::new(Saw { info }), 48_000, SampleFormat::S16);
    stream.pull();
    assert!(!stream.buffer().is_empty());
    stream.set_rate(44_100);
    assert_eq!(stream.rate(), 44_100);
    assert!(
        stream.buffer().is_empty(),
        "a rate change keeps stale audio"
    );
    let at_44k = stream.pull();
    stream.consume(at_44k);
    stream.set_rate(48_000);
    let at_48k = stream.pull();
    assert!(
        at_48k > at_44k,
        "{at_48k} frames at 48k vs {at_44k} at 44.1k"
    );
}

#[test]
fn a_stream_hands_its_queue_to_a_sink() {
    let info = StreamInfo::new(48_000, 1, 1, SampleFormat::S16);
    let mut stream = AudioStream::new(Box::new(Saw { info }), 48_000, SampleFormat::S16);
    stream.pull();
    let queued = stream.buffer().frames();
    let mut writer = wav::Writer::new(stream.info(), SampleFormat::S16);
    assert_eq!(stream.drain_to(&mut writer), queued);
    assert!(stream.buffer().is_empty());
    assert_eq!(writer.frames(), queued);
}

// ---------------------------------------------------------------------------
// a real APU
// ---------------------------------------------------------------------------

#[cfg(feature = "dev-nes-apu")]
mod nes {
    use super::*;
    use crate::core::props::Props;
    use crate::dev::apu::Apu;
    use crate::host::audio::nes::NesAudio;
    use alloc::sync::Arc;

    /// An APU with both pulse channels playing a loud square wave.
    fn squealing(region: &str) -> Arc<Apu> {
        let apu = Arc::new(
            Apu::new(
                &Props::new()
                    .with("region", region)
                    .with("sample-buffer", 1u64 << 20),
            )
            .expect("a valid APU"),
        );
        apu.write(0x15, 0x0f); // enable the four waveform channels
        apu.write(0x00, 0x9f); // pulse 1: 50% duty, halt, constant volume 15
        apu.write(0x02, 0x40); // timer low
        apu.write(0x03, 0x08); // length load, timer high
        apu
    }

    #[test]
    fn the_rate_is_the_exact_rational_of_each_console() {
        for (region, num, den) in [
            ("ntsc", 9_843_750u64, 11u64),
            ("pal", 53_203_425, 64),
            ("dendy", 3_546_895, 4),
        ] {
            let source = NesAudio::new(squealing(region));
            let info = source.info();
            assert_eq!((info.rate_num, info.rate_den), (num, den), "{region}");
            assert_eq!(info.channels, 1);
            assert_eq!(info.output_stage.len(), 3, "the console's RC network");
        }
        // And the rounded figures are the ones the wiki quotes.
        assert_eq!(NesAudio::new(squealing("ntsc")).info().rate_hz(), 894_886);
        assert_eq!(NesAudio::new(squealing("pal")).info().rate_hz(), 831_304);
    }

    #[test]
    fn an_apu_that_is_playing_produces_audible_frames() {
        let apu = squealing("ntsc");
        apu.advance(200_000); // a little over a tenth of a second
        let source = NesAudio::new(apu);
        let mut stream = AudioStream::new(Box::new(source), 48_000, SampleFormat::S16);
        let frames = stream.pull();
        assert!(frames > 4_000, "a tenth of a second gave {frames} frames");

        // The DC offset is gone — the console's own coupling capacitors — and
        // what is left actually swings.
        let mut min = i16::MAX;
        let mut max = i16::MIN;
        let mut sum = 0i64;
        for frame in 0..stream.buffer().frames() {
            let value = stream.buffer().sample(frame, 0).expect("a sample");
            min = min.min(value);
            max = max.max(value);
            sum += i64::from(value);
        }
        let mean = sum / stream.buffer().frames() as i64;
        assert!(mean.abs() < 2_000, "the DC offset survived: mean {mean}");
        assert!(
            i32::from(max) - i32::from(min) > 4_000,
            "the signal does not swing: {min}..{max}"
        );
    }

    /// Nothing enabled: the mixer output is a constant, and a constant is
    /// exactly what the console's coupling capacitors reject.
    ///
    /// The first few milliseconds are *not* silent, and should not be: charging
    /// a capacitor from a cold start is a step, and a real console does it too.
    /// What must be true is that it settles, so the tail is asserted rather
    /// than the whole stream.
    #[test]
    fn a_silent_apu_settles_to_silence() {
        let apu = Arc::new(Apu::new(&Props::new()).expect("a valid APU"));
        apu.advance(100_000);
        let mut stream = AudioStream::new(Box::new(NesAudio::new(apu)), 44_100, SampleFormat::S16);
        let frames = stream.pull();
        assert!(frames > 2_000, "{frames} frames");
        for frame in frames / 2..frames {
            let value = stream.buffer().sample(frame, 0).expect("a sample");
            assert_eq!(value, 0, "frame {frame} of {frames} is not silent");
        }
    }

    /// A ring nobody drains overwrites its oldest samples and says so, and the
    /// count is a diagnostic rather than anything a guest could see.
    #[test]
    fn an_undrained_ring_reports_its_losses() {
        let apu =
            Arc::new(Apu::new(&Props::new().with("sample-buffer", 64u64)).expect("a valid APU"));
        apu.advance(10_000);
        let source = NesAudio::new(apu);
        assert!(source.dropped() > 4_000, "{}", source.dropped());
    }
}

/// The property the whole module exists to keep: **a machine sounds the same
/// whether or not anybody is listening, and hashes the same either way.**
///
/// Two identical NES machines are built and run for the same virtual time in
/// the same number of steps. One has its APU drained after every step and the
/// samples converted, filtered, resampled and queued; the other is ignored.
/// Their state hashes must be equal — if the audio path could ever move
/// architectural state, this is where it would show.
#[cfg(all(feature = "machine-nes", feature = "dev-nes-apu"))]
#[test]
fn listening_does_not_change_the_machine() {
    use crate::core::clock::GlobalTime;
    use crate::core::hosts::HostObjects;
    use crate::host::audio::nes::capture;
    use crate::machine::catalog;

    /// `JMP $C000` forever, with a handful of APU writes first so the chip is
    /// actually making a noise while the comparison runs.
    static ROM: &[u8] = &{
        let mut image = [0u8; 16 + 16384 + 8192];
        image[0] = b'N';
        image[1] = b'E';
        image[2] = b'S';
        image[3] = 0x1a;
        image[4] = 1;
        image[5] = 1;
        // reset vector -> $c000
        image[16 + 0x3ffc] = 0x00;
        image[16 + 0x3ffd] = 0xc0;
        // lda #$0f / sta $4015 / lda #$9f / sta $4000 / lda #$08 / sta $4003
        // / jmp $c00c
        let program: [u8; 15] = [
            0xa9, 0x0f, 0x8d, 0x15, 0x40, 0xa9, 0x9f, 0x8d, 0x00, 0x40, 0xa9, 0x08, 0x8d, 0x03,
            0x40,
        ];
        let mut i = 0;
        while i < program.len() {
            image[16 + i] = program[i];
            i += 1;
        }
        image[16 + 15] = 0x4c; // jmp
        image[16 + 16] = 0x0f;
        image[16 + 17] = 0xc0;
        image
    };

    /// A NES with an interception installed, and the host objects it captured
    /// into — one table per build, so the two machines below cannot see each
    /// other's APU.
    fn build(capacity: u64) -> (crate::machine::Machine, alloc::sync::Arc<HostObjects>) {
        let registry = catalog::registry().expect("a registry");
        let mut options = catalog::build_options().expect("build options");
        options.realize.media.insert("cart", ROM);
        capture::install(&mut options, capacity).expect("the interception");
        let entries = catalog::machines();
        let entry = entries
            .iter()
            .find(|e| e.name == "nes-ntsc")
            .expect("machine-nes is on");
        let machine = crate::machine::build(entry.name, entry.source, &registry, &options)
            .expect("a machine");
        (machine, options.realize.hosts)
    }

    // The step is one NTSC video frame, which is the cadence every front end
    // in this tree runs at. *The two machines are stepped identically* — that
    // is the point: audio must not change how the machine is driven, only what
    // is read out of it afterwards.
    let step = GlobalTime::from_nanos(16_639_356);
    const STEPS: u32 = 30;

    let (mut listened, listened_hosts) = build(1 << 20);
    let source = capture::take(&listened_hosts).expect("the machine has an APU");
    let mut stream = AudioStream::new(Box::new(source), 48_000, SampleFormat::S16);
    for _ in 0..STEPS {
        listened.run_for(step).expect("a frame");
        stream.pull();
    }

    // Built with the same interception and then simply not listened to: its
    // APU is captured into its own table, which nobody reads.
    let (mut ignored, _ignored_hosts) = build(1 << 20);
    for _ in 0..STEPS {
        ignored.run_for(step).expect("a frame");
    }

    assert!(
        stream.produced() > 20_000,
        "half a second gave {} frames",
        stream.produced()
    );
    assert_eq!(
        listened.state_hash().expect("a hash"),
        ignored.state_hash().expect("a hash"),
        "the state hash depends on whether audio was drained"
    );
    assert_eq!(listened.now().as_nanos(), ignored.now().as_nanos());
}
