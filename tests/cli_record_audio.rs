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

// ---------------------------------------------------------------------------
// The Amiga
// ---------------------------------------------------------------------------

/// A Kickstart-shaped image whose "firmware" plays a square wave on channel 0.
///
/// **No byte of any real ROM**: the program is hand-assembled here from the
/// MC68000 user manual's instruction formats, exactly as `amiga_a500_board.rs`
/// builds its own, and the register offsets are the *Amiga Hardware Reference
/// Manual*'s Appendix B.
///
/// It runs out of the ROM's **own** window at `$F80000` rather than out of the
/// overlay, because the first thing it does is take the overlay away: chip RAM
/// has to answer at address zero before a waveform can be written into it. The
/// overlay is CIA-A's `PA0` (Appendix E), so that is a `DDRA` and a `PRA` write
/// through the real chip rather than a poke at a decoder.
///
/// ```text
///   f8000c: 13fc 0001 00bf e201       move.b #$01,$bfe201    CIA-A DDRA: PA0 an output
///           13fc 0000 00bf e001       move.b #$00,$bfe001    CIA-A PRA:  OVL low
///           23fc 7f7f 7f7f 0000 1000  move.l #$7f7f7f7f,$1000   +127, four times
///           23fc 8181 8181 0000 1004  move.l #$81818181,$1004   -127, four times
///           33fc 0000 00df f0a0       move.w #$0000,$dff0a0  AUD0LC, high half
///           33fc 1000 00df f0a2       move.w #$1000,$dff0a2  AUD0LC, low half
///           33fc 0004 00df f0a4       move.w #4,$dff0a4      AUD0LEN: four words
///           33fc 00c8 00df f0a6       move.w #200,$dff0a6    AUD0PER: 200 colour clocks
///           33fc 0040 00df f0a8       move.w #64,$dff0a8     AUD0VOL: full level
///           33fc 8201 00df f096       move.w #$8201,$dff096  DMACON: SET, DMAEN, AUD0EN
///           60fe                      bra    *               and let it play
/// ```
///
/// Eight samples of 200 colour clocks each is a 1 600 colour-clock cycle —
/// 3 546 895 / 1 600 = 2 217 Hz on this PAL board — at full volume and on the
/// **left** output, channels 0 and 3 being the left pair (Chapter 5).
///
/// The assertions below are about *level and side* rather than pitch, and
/// deliberately: Agnus and Paula are both lazily advanced and can be a
/// millisecond apart, so an audio DMA slot sometimes lands after Paula has
/// already crossed two word boundaries — `AUDxDR` is one flag, not a count, so
/// that word is fetched once and played twice. The stream reproduces what the
/// chipset fed the channel, which is the property this file is for; the
/// chipset's own handshake is `agnus`'s business.
#[cfg(feature = "machine-amiga-a500")]
fn amiga_tone_rom() -> Vec<u8> {
    const CODE: &[u16] = &[
        0x13fc, 0x0001, 0x00bf, 0xe201, // move.b #$01,$bfe201
        0x13fc, 0x0000, 0x00bf, 0xe001, // move.b #$00,$bfe001
        0x23fc, 0x7f7f, 0x7f7f, 0x0000, 0x1000, // move.l #$7f7f7f7f,$1000
        0x23fc, 0x8181, 0x8181, 0x0000, 0x1004, // move.l #$81818181,$1004
        0x33fc, 0x0000, 0x00df, 0xf0a0, // move.w #$0000,$dff0a0
        0x33fc, 0x1000, 0x00df, 0xf0a2, // move.w #$1000,$dff0a2
        0x33fc, 0x0004, 0x00df, 0xf0a4, // move.w #4,$dff0a4
        0x33fc, 0x00c8, 0x00df, 0xf0a6, // move.w #200,$dff0a6
        0x33fc, 0x0040, 0x00df, 0xf0a8, // move.w #64,$dff0a8
        0x33fc, 0x8201, 0x00df, 0xf096, // move.w #$8201,$dff096
        0x60fe, // bra *
    ];
    let mut image = vec![0u8; 512 * 1024];
    image[0..4].copy_from_slice(&0x0008_0000u32.to_be_bytes()); // SSP: top of chip RAM
    image[4..8].copy_from_slice(&0x00f8_000cu32.to_be_bytes()); // PC: the ROM's own window
    for (i, word) in CODE.iter().enumerate() {
        let at = 0x0c + 2 * i;
        image[at..at + 2].copy_from_slice(&word.to_be_bytes());
    }
    image
}

/// The A500 records a second of **stereo** for a second of run, twice over
/// identically, without moving the machine.
///
/// The stereo split is the claim worth making here: Paula's four channels are
/// two pairs, 0 and 3 to the left output and 1 and 2 to the right (Chapter 5),
/// so a model that mixed to mono would pass every length assertion and still
/// have thrown away the one thing an Amiga is famous for.
#[cfg(feature = "machine-amiga-a500")]
#[test]
fn an_amiga_records_a_second_of_stereo_for_a_second_of_run() {
    let rom = scratch("tone.rom");
    std::fs::write(&rom, amiga_tone_rom()).expect("the scratch directory is writable");
    let wav_path = scratch("amiga.wav");
    let again = scratch("amiga-again.wav");
    let _ = std::fs::remove_file(&wav_path);
    let _ = std::fs::remove_file(&again);

    let media = format!("kickstart={}", rom.display());
    let base: Vec<&str> = vec![
        "run",
        "amiga-a500",
        "--headless",
        "--media",
        &media,
        "--for",
        "1s",
    ];
    let recording = |to: &std::path::Path, args: &[&str]| {
        let mut args = args.to_vec();
        args.push("--record-audio");
        let path = to.to_str().expect("a UTF-8 scratch path");
        args.push(path);
        run(&args)
    };

    let (ok, stdout, stderr) = recording(&wav_path, &base);
    assert!(ok, "rsemu run amiga-a500 --record-audio failed: {stderr}");

    let bytes = std::fs::read(&wav_path).expect("--record-audio wrote a file");
    let wav = parse_wav(&bytes);
    assert_eq!(wav.channels, 2, "0 and 3 left, 1 and 2 right");
    assert_eq!(wav.rate, 44_100, "the default --audio-rate");
    assert_eq!(wav.bits, 16);
    // A wider window than the consoles get, and in one direction only: Paula is
    // lazily advanced (`ROADMAP.md` §4.2), so the last drain of the run sees it
    // wherever its most recent DMA slot left it rather than exactly at the end.
    assert!(
        wav.frames > u64::from(wav.rate) * 9 / 10 && wav.frames <= u64::from(wav.rate) + 100,
        "{} frames at {} Hz is not about a second",
        wav.frames,
        wav.rate
    );

    // The tone is on the left and the right is silent: a stereo split rather
    // than a mono sum wearing two channels.
    let samples: Vec<i16> = bytes[44..]
        .as_chunks::<2>()
        .0
        .iter()
        .map(|s| i16::from_le_bytes(*s))
        .collect();
    let (left, right): (Vec<i16>, Vec<i16>) =
        samples.chunks_exact(2).map(|f| (f[0], f[1])).unzip();
    assert!(
        left.iter().any(|s| s.abs() > 1_000),
        "the guest played a full-volume square on channel 0"
    );
    assert!(
        right.iter().all(|s| s.abs() < 100),
        "nothing was playing on channels 1 or 2"
    );

    // The same run again is the same file, byte for byte.
    let (ok, _, stderr) = recording(&again, &base);
    assert!(ok, "the second recording failed: {stderr}");
    assert_eq!(
        bytes,
        std::fs::read(&again).expect("a second file"),
        "two identical runs recorded different sound"
    );

    // And listening did not move the machine.
    let (ok, quiet_stdout, stderr) = run(&base);
    assert!(ok, "the unrecorded run failed: {stderr}");
    assert_eq!(
        state_hash(&stdout),
        state_hash(&quiet_stdout),
        "draining the sound chip changed where the machine ended up"
    );

    let _ = std::fs::remove_file(&wav_path);
    let _ = std::fs::remove_file(&again);
    let _ = std::fs::remove_file(&rom);
}
