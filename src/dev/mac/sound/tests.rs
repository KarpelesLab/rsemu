//! The sound circuit's own tests: the buffers' places, the duty-cycle scale,
//! the gate, the volume, and a known waveform in memory coming back as known
//! samples.

use super::*;
use crate::core::props::{Link, Value};
use crate::core::space::{RamStore, Region};
use crate::core::state::{MachineShape, Migrations, StateReader, StateWriter};
use crate::core::value::Endian;
use crate::dev::mac::video;

const RAM_LEN: u64 = 0x10_0000;

fn props() -> Props {
    Props::new().with("ram", Value::Link(Link::new("dram").unwrap()))
}

/// A sound circuit pointed at a megabyte of memory the test can write, already
/// recording and already enabled at full volume.
fn wired() -> (Sound, Arc<RamStore>) {
    let sound = Sound::new(&props()).expect("`ram` is enough");
    let store = Arc::new(RamStore::new(RAM_LEN));
    let ram: RegionRef = Arc::new(Region::ram("dram", Arc::clone(&store)).with_endian(Endian::Big));
    sound.attach(&ram, 24).expect("it maps");
    sound.set_recording(true);
    sound.set_sndenb(false);
    sound.set_volume(7);
    (sound, store)
}

/// One sample a scan line, 370 of them a frame, and the rate that falls out of
/// the board's crystal: 244 800 / 11 Hz. See the module docs.
#[test]
fn the_rate_is_the_line_rate_the_video_circuit_defines() {
    assert_eq!(SAMPLES_PER_FRAME, video::LINES_PER_FRAME);
    // 370 samples really is one frame of the raster.
    assert_eq!(video::FRAME_TICKS / video::DOTS_PER_LINE, SAMPLES_PER_FRAME);
    // 15 667 200 dot clocks a second, 704 of them a line: 22 254.5454… Hz,
    // which in lowest terms is what the host seam reports.
    assert_eq!(15_667_200 / video::DOTS_PER_LINE, 22_254);
    assert_ne!(15_667_200 % video::DOTS_PER_LINE, 0, "not a whole hertz");
    assert_eq!((15_667_200 / 64, video::DOTS_PER_LINE / 64), (244_800, 11));
}

/// The byte is a duty cycle, so the middle is silence and the ends are the
/// rails. `$80` is what the ROM leaves in the buffer when it has finished
/// chiming.
#[test]
fn a_byte_is_a_duty_cycle_centred_on_the_middle() {
    assert_eq!(sample(SILENCE, 7), 0);
    assert_eq!(sample(0x00, 7), i16::MIN);
    assert_eq!(sample(0xff, 7), 32_512);
    // Volume is linear in the setting and 0 is silence.
    assert_eq!(sample(0xff, 0), 0);
    assert_eq!(sample(0x00, 0), 0);
    assert_eq!(sample(0xff, 1), 32_512 / 7);
    assert!(sample(0xff, 3) < sample(0xff, 4));
}

/// The sound byte is the high-order byte of each word and the buffer hangs a
/// fixed distance below the top of memory; `SNDPG2` picks which of the two.
#[test]
fn the_sound_byte_is_the_high_half_of_each_word_of_the_buffer() {
    let (sound, ram) = wired();
    let main = RAM_LEN - MAIN_OFFSET;
    // Word 0 is $FF11 and word 1 is $0022: full scale then the other rail,
    // with disk-speed bytes beside them that must not be heard.
    ram.write_at(main, &[0xff, 0x11, 0x00, 0x22]).unwrap();
    for i in 2..SAMPLES_PER_FRAME {
        ram.write_at(main + i * 2, &[SILENCE, 0x33]).unwrap();
    }
    sound.advance_to(2);
    assert_eq!(sound.speaker().take_audio(), [32_512, i16::MIN]);

    // The alternate buffer is a different 740 bytes.
    let alt = RAM_LEN - ALT_OFFSET;
    ram.write_at(alt, &[0x00, 0, SILENCE, 0]).unwrap();
    sound.set_sndpg2(false);
    sound.advance_to(SAMPLES_PER_FRAME + 2);
    let out = sound.speaker().take_audio();
    assert_eq!(out.len(), SAMPLES_PER_FRAME as usize);
    assert_eq!(
        out[out.len() - 2],
        i16::MIN,
        "word 0 of the alternate buffer"
    );
    assert_eq!(out[out.len() - 1], 0);
}

/// The buffer is 370 samples and the index wraps with the frame, so a waveform
/// whose period divides 370 is seamless across a frame boundary — which is how
/// the ROM's chime is built.
#[test]
fn the_index_wraps_with_the_frame() {
    let (sound, ram) = wired();
    let main = RAM_LEN - MAIN_OFFSET;
    for i in 0..SAMPLES_PER_FRAME {
        let byte = if i == 0 { 0xff } else { SILENCE };
        ram.write_at(main + i * 2, &[byte, 0]).unwrap();
    }
    sound.advance_to(SAMPLES_PER_FRAME * 2 + 1);
    let out = sound.speaker().take_audio();
    let loud: Vec<usize> = out
        .iter()
        .enumerate()
        .filter(|&(_, &s)| s != 0)
        .map(|(i, _)| i)
        .collect();
    let n = SAMPLES_PER_FRAME as usize;
    assert_eq!(loud, [0, n, n * 2]);
}

/// `SNDENB` high is the idle pull-up and it mutes the speaker: the circuit
/// produces the rest level and does not read the buffer at all.
#[test]
fn the_via_gate_mutes_the_speaker() {
    let (sound, ram) = wired();
    let main = RAM_LEN - MAIN_OFFSET;
    for i in 0..SAMPLES_PER_FRAME {
        ram.write_at(main + i * 2, &[0xff, 0]).unwrap();
    }
    sound.set_sndenb(true);
    sound.advance_to(4);
    assert_eq!(sound.speaker().take_audio(), [0, 0, 0, 0]);
    sound.set_sndenb(false);
    sound.advance_to(8);
    assert_eq!(sound.speaker().take_audio(), [32_512; 4]);
    // The volume bits attenuate what it does produce.
    sound.set_volume(0);
    sound.advance_to(10);
    assert_eq!(sound.speaker().take_audio(), [0, 0]);
}

/// A machine nobody is recording produces nothing and, more to the point,
/// **names no event**: 22 254 scheduler rounds a second is the price of
/// reading each byte at the tick its line scanned it, and a board with no
/// listener should not pay it.
#[test]
fn nothing_is_scheduled_and_nothing_is_kept_unless_somebody_is_listening() {
    let (sound, _ram) = wired();
    sound.set_recording(false);
    assert_eq!(Device::next_event_tick(&sound), None);
    Device::advance_to(&sound, 100_000);
    assert_eq!(
        sound.speaker().ticks(),
        100_000,
        "it still keeps up with time"
    );
    assert!(sound.speaker().take_audio().is_empty());

    sound.set_recording(true);
    assert_eq!(Device::next_event_tick(&sound), Some(100_001));
    Device::advance_to(&sound, 100_003);
    assert_eq!(sound.speaker().take_audio().len(), 3);
}

/// The ring keeps the newest when nobody drains it, and says how many it threw
/// away.
#[test]
fn an_undrained_ring_keeps_the_newest_and_counts_what_it_dropped() {
    let (sound, ram) = wired();
    let main = RAM_LEN - MAIN_OFFSET;
    for i in 0..SAMPLES_PER_FRAME {
        ram.write_at(main + i * 2, &[SILENCE, 0]).unwrap();
    }
    sound.advance_to(RING_SAMPLES as u64 + 10);
    assert_eq!(sound.speaker().dropped(), 10);
    assert_eq!(sound.speaker().take_audio().len(), RING_SAMPLES);
}

/// The buffer is ordinary guest memory: the `ram` object saves it, not this
/// device. What this device has is where the sampler is and what the pins say
/// — and *not* the ring, which is output rather than state.
#[test]
fn a_snapshot_round_trips_to_an_identical_state_hash() {
    let (saved, _ram) = wired();
    saved.advance_to(SAMPLES_PER_FRAME * 7 + 123);
    saved.set_volume(5);
    saved.set_sndpg2(false);

    let image = |sound: &Sound| -> Vec<u8> {
        let mut shape = MachineShape::new();
        shape.add_device("snd", CLASS_NAME).unwrap();
        let mut w = StateWriter::new(shape);
        {
            let mut chunk = w.chunk("snd", CLASS_NAME, STATE_VERSION).unwrap();
            Device::save(sound, &mut chunk).unwrap();
        }
        w.to_vec().unwrap()
    };
    let first = image(&saved);

    let (restored, _other) = wired();
    let reader = StateReader::new(&first).unwrap();
    let chunk = reader
        .load("snd", CLASS_NAME, STATE_VERSION, &Migrations::new())
        .unwrap();
    Device::load(&restored, &mut chunk.reader()).unwrap();
    assert_eq!(image(&restored), first);
    assert_eq!(restored.speaker().ticks(), saved.speaker().ticks());
}

/// A memory too small to hold a sound buffer where a Macintosh puts one is
/// refused, rather than reading off the end of it every line.
#[test]
fn properties_are_checked() {
    assert!(Sound::new(&Props::new()).is_err(), "`ram` is required");
    let err = Sound::new(&props().with("size", Value::Size(1)))
        .expect_err("a property this class does not know")
        .to_string();
    assert!(err.contains("size"), "{err}");

    let sound = Sound::new(&props()).unwrap();
    let small: RegionRef = Arc::new(Region::ram("tiny", Arc::new(RamStore::new(0x1000))));
    let err = sound.attach(&small, 24).expect_err("too small").to_string();
    assert!(err.contains("alternate sound buffer"), "{err}");
}

/// The class registers, has no registers of its own, and names its five pins.
#[test]
fn the_class_is_registrable_and_its_schema_matches() {
    let mut registry = crate::core::Registry::new();
    register(&mut registry).expect("a fresh registry");
    assert_eq!(registry.get(CLASS_NAME).unwrap().version, STATE_VERSION);
    let (sound, _ram) = wired();
    assert!(
        sound.region("").is_none(),
        "the sound circuit has no registers at all"
    );
    let schema = schema();
    for pin in [
        SNDENB_PIN,
        SNDPG2_PIN,
        VOLUME_PINS[0],
        VOLUME_PINS[1],
        VOLUME_PINS[2],
    ] {
        assert!(Device::sink(&sound, pin, &[]).is_some(), "{pin}");
        assert!(schema.port_named(pin).is_some(), "{pin}");
    }
    assert!(Device::sink(&sound, "nosuch", &[]).is_none());
}
