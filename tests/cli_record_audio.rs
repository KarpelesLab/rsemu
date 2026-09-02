//! `rsemu run --record-audio` writes a WAV as long as the run, for every
//! machine that makes a noise.
//!
//! Until now that was the NES and nothing else: `host::audio` had one adapter,
//! `take_audio` had one arm, and a Game Boy or a Master System — both of which
//! have a complete, sample-generating sound chip in `dev/` — got
//! "this machine has no audio device".
//!
//! Two claims here, and the second is the one that costs something:
//!
//! 1. **The file is as long as the run.** A second of virtual time is a second
//!    of audio, within the rounding a rational device rate and an integer host
//!    rate imply. This is not free for a console: a `gb.apu`'s output ring holds
//!    8 192 frames — a quarter of a second — and `RING_FRAMES` is a `const` with
//!    no property behind it, so the whole-run ring the NES uses is not available
//!    and the run has to be drained as it goes.
//!
//! 2. **Draining as it goes does not change the machine.** The same run,
//!    recorded and not recorded, reaches the same state hash. That is the
//!    property `host::audio` exists to keep, asserted here against the binary's
//!    own driving loop rather than against a test harness's.
//!
//! The state hash is read off the binary's own summary line, which is what a
//! person comparing two runs would do.

#![cfg(all(feature = "cli", feature = "std"))]

use std::path::PathBuf;
use std::process::Command;

/// A scratch path nobody else in this run will pick.
#[allow(dead_code)]
fn scratch(name: &str) -> PathBuf {
    std::env::temp_dir().join(format!("rsemu-audio-{}-{name}", std::process::id()))
}

/// Run the binary and hand back success, stdout and stderr.
#[allow(dead_code)]
fn run(args: &[&str]) -> (bool, String, String) {
    let out = Command::new(env!("CARGO_BIN_EXE_rsemu"))
        .args(args)
        .output()
        .expect("the binary this test was built alongside");
    (
        out.status.success(),
        String::from_utf8_lossy(&out.stdout).into_owned(),
        String::from_utf8_lossy(&out.stderr).into_owned(),
    )
}

/// The `state hash 0x…` the run summary prints.
#[allow(dead_code)]
fn state_hash(stdout: &str) -> String {
    stdout
        .lines()
        .find_map(|l| l.strip_prefix("state hash "))
        .expect("the summary prints a state hash")
        .to_string()
}

/// A RIFF/WAVE file, decoded far enough to say how long it is.
///
/// `wav::encode` writes a fixed 44-byte prologue — RIFF, `WAVE`, a 16-byte
/// `fmt ` body and a `data` header — so the fields are at known offsets and
/// there is no chunk walk to write.
#[allow(dead_code)]
struct Wav {
    channels: u16,
    rate: u32,
    bits: u16,
    frames: u64,
}

#[allow(dead_code)]
fn parse_wav(bytes: &[u8]) -> Wav {
    assert!(bytes.len() >= 44, "a WAV is at least its own header");
    assert_eq!(&bytes[..4], b"RIFF");
    assert_eq!(&bytes[8..12], b"WAVE");
    assert_eq!(&bytes[12..16], b"fmt ");
    assert_eq!(&bytes[36..40], b"data");
    let channels = u16::from_le_bytes([bytes[22], bytes[23]]);
    let rate = u32::from_le_bytes([bytes[24], bytes[25], bytes[26], bytes[27]]);
    let bits = u16::from_le_bytes([bytes[34], bytes[35]]);
    let data = u32::from_le_bytes([bytes[40], bytes[41], bytes[42], bytes[43]]);
    let block = u64::from(channels) * u64::from(bits / 8);
    assert!(block > 0, "a frame has a size");
    assert_eq!(
        u64::from(data),
        bytes.len() as u64 - 44,
        "the data chunk's length is the rest of the file"
    );
    Wav {
        channels,
        rate,
        bits,
        frames: u64::from(data) / block,
    }
}

/// Assert that `frames` at `rate` is within one per cent of `seconds`.
///
/// A per-cent window rather than an exact count: the device rate is a rational
/// and the host rate is an integer, so the last output frame of a run lands
/// wherever the resampler's accumulator happens to be, and the first slice of
/// the run starts before the guest has written a single sound register.
#[allow(dead_code)]
fn assert_length(wav: &Wav, seconds: f64) {
    let want = f64::from(wav.rate) * seconds;
    let got = wav.frames as f64;
    assert!(
        (got - want).abs() < want / 100.0,
        "{} frames at {} Hz is {:.3} s, not {seconds} s",
        wav.frames,
        wav.rate,
        got / f64::from(wav.rate)
    );
}

// ---------------------------------------------------------------------------
// The Game Boy
// ---------------------------------------------------------------------------

/// Switch the chip on, open both sides, and trigger channel one.
///
/// Loaded at `$0150`, which is where `synthetic_image` puts a program and where
/// the header's `NOP; JP $0150` entry point goes. Register names and bit
/// layouts: Pan Docs, "Audio Registers".
///
/// ```text
///   3e 80  e0 26   ld a,$80 ; ldh ($26),a   NR52: sound on
///   3e ff  e0 25   ld a,$ff ; ldh ($25),a   NR51: every channel, both sides
///   3e 77  e0 24   ld a,$77 ; ldh ($24),a   NR50: full volume, both sides
///   3e 80  e0 11   ld a,$80 ; ldh ($11),a   NR11: 50% duty
///   3e f0  e0 12   ld a,$f0 ; ldh ($12),a   NR12: initial volume 15
///   3e 00  e0 13   ld a,$00 ; ldh ($13),a   NR13: frequency low
///   3e 87  e0 14   ld a,$87 ; ldh ($14),a   NR14: trigger, frequency high
///   18 fe          jr $                     and let it ring
/// ```
#[cfg(feature = "machine-gameboy")]
const GB_TONE: [u8; 30] = [
    0x3e, 0x80, 0xe0, 0x26, 0x3e, 0xff, 0xe0, 0x25, 0x3e, 0x77, 0xe0, 0x24, 0x3e, 0x80, 0xe0, 0x11,
    0x3e, 0xf0, 0xe0, 0x12, 0x3e, 0x00, 0xe0, 0x13, 0x3e, 0x87, 0xe0, 0x14, 0x18, 0xfe,
];

#[cfg(feature = "machine-gameboy")]
#[test]
fn a_game_boy_records_a_second_of_sound_for_a_second_of_run() {
    let cart = scratch("tone.gb");
    std::fs::write(
        &cart,
        rsemu::dev::gb::cart::synthetic_image(2, 0x00, 0x00, &GB_TONE),
    )
    .expect("the scratch directory is writable");
    let wav_path = scratch("gb.wav");
    let _ = std::fs::remove_file(&wav_path);

    let (ok, stdout, stderr) = run(&[
        "run",
        "gameboy",
        "--headless",
        "--media",
        &format!("cart={}", cart.display()),
        "--record-audio",
        wav_path.to_str().expect("a UTF-8 scratch path"),
        "--for",
        "1s",
    ]);
    assert!(ok, "rsemu run gameboy --record-audio failed: {stderr}");

    let bytes = std::fs::read(&wav_path).expect("--record-audio wrote a file");
    let wav = parse_wav(&bytes);
    assert_eq!(wav.channels, 2, "NR51 pans, so a DMG is stereo");
    assert_eq!(wav.rate, 44_100, "the default --audio-rate");
    assert_eq!(wav.bits, 16);
    assert_length(&wav, 1.0);

    // And it is a noise rather than a silence.
    let loud = bytes[44..]
        .as_chunks::<2>()
        .0
        .iter()
        .map(|s| i16::from_le_bytes(*s))
        .any(|s| s.abs() > 1_000);
    assert!(loud, "the guest triggered a square wave at full volume");

    // The second claim: the same run, not recorded, ends in the same place.
    let (ok, quiet_stdout, stderr) = run(&[
        "run",
        "gameboy",
        "--headless",
        "--media",
        &format!("cart={}", cart.display()),
        "--for",
        "1s",
    ]);
    assert!(ok, "the unrecorded run failed: {stderr}");
    assert_eq!(
        state_hash(&stdout),
        state_hash(&quiet_stdout),
        "draining the sound chip changed where the machine ended up"
    );

    let _ = std::fs::remove_file(&wav_path);
    let _ = std::fs::remove_file(&cart);
}

// ---------------------------------------------------------------------------
// The Master System
// ---------------------------------------------------------------------------

/// Latch a tone on channel 0 and open its attenuator, then stop.
///
/// The SN76489 takes one byte at a time on the console's port `$7F`: a latch
/// byte is `1 cc t dddd` — channel, type, and the low four data bits — and a
/// following data byte is `0 dddddd`, the high six (Texas Instruments SN76489
/// datasheet, and `dev::sms::psg`).
///
/// ```text
///   3e 8e  d3 7f   ld a,$8e ; out ($7f),a   channel 0 tone, low nibble $E
///   3e 01  d3 7f   ld a,$01 ; out ($7f),a   high six bits: period $01E
///   3e 90  d3 7f   ld a,$90 ; out ($7f),a   channel 0 attenuation 0: loudest
///   18 fe          jr $
/// ```
#[cfg(feature = "machine-sms")]
const SMS_TONE: [u8; 14] = [
    0x3e, 0x8e, 0xd3, 0x7f, 0x3e, 0x01, 0xd3, 0x7f, 0x3e, 0x90, 0xd3, 0x7f, 0x18, 0xfe,
];

#[cfg(feature = "machine-sms")]
#[test]
fn a_master_system_records_a_second_of_sound_for_a_second_of_run() {
    // A 32 KiB image, which is one Sega mapper page and the smallest thing the
    // board will take. The program sits at the reset vector.
    let mut image = vec![0u8; 0x8000];
    image[..SMS_TONE.len()].copy_from_slice(&SMS_TONE);
    let cart = scratch("tone.sms");
    std::fs::write(&cart, &image).expect("the scratch directory is writable");
    let wav_path = scratch("sms.wav");
    let _ = std::fs::remove_file(&wav_path);

    let (ok, stdout, stderr) = run(&[
        "run",
        "sms-ntsc",
        "--headless",
        "--media",
        &format!("cart={}", cart.display()),
        "--record-audio",
        wav_path.to_str().expect("a UTF-8 scratch path"),
        "--for",
        "1s",
    ]);
    assert!(ok, "rsemu run sms-ntsc --record-audio failed: {stderr}");

    let bytes = std::fs::read(&wav_path).expect("--record-audio wrote a file");
    let wav = parse_wav(&bytes);
    assert_eq!(wav.channels, 1, "an SN76489 has one output pin");
    assert_eq!(wav.rate, 44_100);
    assert_length(&wav, 1.0);

    let loud = bytes[44..]
        .as_chunks::<2>()
        .0
        .iter()
        .map(|s| i16::from_le_bytes(*s))
        .any(|s| s.abs() > 1_000);
    assert!(loud, "the guest opened channel 0's attenuator");

    let (ok, quiet_stdout, stderr) = run(&[
        "run",
        "sms-ntsc",
        "--headless",
        "--media",
        &format!("cart={}", cart.display()),
        "--for",
        "1s",
    ]);
    assert!(ok, "the unrecorded run failed: {stderr}");
    assert_eq!(
        state_hash(&stdout),
        state_hash(&quiet_stdout),
        "draining the sound chip changed where the machine ended up"
    );

    let _ = std::fs::remove_file(&wav_path);
    let _ = std::fs::remove_file(&cart);
}

// ---------------------------------------------------------------------------
// The machine that already worked
// ---------------------------------------------------------------------------

/// The NES's recording is no longer capped at the depth of its ring either,
/// because the same drain loop carries it.
#[cfg(feature = "machine-nes")]
#[test]
fn a_nes_recording_is_still_as_long_as_the_run() {
    // `JMP $C000` forever, with the APU's first square triggered on the way in.
    let mut image = vec![0u8; 16 + 0x4000 + 0x2000];
    image[..4].copy_from_slice(b"NES\x1a");
    image[4] = 1;
    image[5] = 1;
    let program: &[u8] = &[
        0xa9, 0x0f, 0x8d, 0x15, 0x40, // lda #$0f ; sta $4015  enable the squares
        0xa9, 0x9f, 0x8d, 0x00, 0x40, // lda #$9f ; sta $4000  duty, constant volume
        0xa9, 0x08, 0x8d, 0x02, 0x40, // lda #$08 ; sta $4002  period low
        0xa9, 0x00, 0x8d, 0x03, 0x40, // lda #$00 ; sta $4003  period high, trigger
        0x4c, 0x14, 0xc0, // jmp $c014
    ];
    image[16..16 + program.len()].copy_from_slice(program);
    image[16 + 0x3ffc] = 0x00;
    image[16 + 0x3ffd] = 0xc0;

    let cart = scratch("tone.nes");
    std::fs::write(&cart, &image).expect("the scratch directory is writable");
    let wav_path = scratch("nes.wav");
    let _ = std::fs::remove_file(&wav_path);

    let (ok, _stdout, stderr) = run(&[
        "run",
        "nes-ntsc",
        "--media",
        &format!("cart={}", cart.display()),
        "--record-audio",
        wav_path.to_str().expect("a UTF-8 scratch path"),
        "--for",
        "1s",
        "-q",
    ]);
    assert!(ok, "rsemu run nes-ntsc --record-audio failed: {stderr}");
    let bytes = std::fs::read(&wav_path).expect("--record-audio wrote a file");
    let wav = parse_wav(&bytes);
    assert_eq!(wav.channels, 1, "an RP2A03 mixes to one output");
    assert_length(&wav, 1.0);

    let _ = std::fs::remove_file(&wav_path);
    let _ = std::fs::remove_file(&cart);
}

// ---------------------------------------------------------------------------
// The machine that cannot be recorded
// ---------------------------------------------------------------------------

/// A machine with no sound chip refuses `--record-audio` rather than ignoring
/// it.
///
/// A PC is the case that was silently wrong: it opens a character port, so
/// `rsemu run` handed it to the console loop, and that loop reached neither the
/// drain nor the writer — `rsemu run pc-at --record-audio x.wav` exited zero
/// having written nothing at all. The refusal now comes from `check_outputs`,
/// before the run, and it is the same refusal a headless machine with no sound
/// chip gets.
#[cfg(feature = "machine-pc-at")]
#[test]
fn a_machine_with_no_sound_chip_says_so() {
    let wav_path = scratch("pc-at.wav");
    let _ = std::fs::remove_file(&wav_path);
    let (ok, _stdout, stderr) = run(&[
        "run",
        "pc-at",
        "--record-audio",
        wav_path.to_str().expect("a UTF-8 scratch path"),
        "--for",
        "50ms",
        "-q",
    ]);
    assert!(!ok, "a recording that cannot be made is a failing run");
    assert!(stderr.contains("no audio device"), "{stderr}");
    assert!(!wav_path.exists(), "and nothing was written");
}
