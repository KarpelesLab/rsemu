//! The S-DSP: the SNES sound generator.
//!
//! Eight voices, each playing **BRR**-compressed samples out of the 64 KiB of
//! audio RAM the S-SMP shares with it, through a 4-point Gaussian interpolator,
//! an ADSR or GAIN envelope and a stereo volume pair, into a main mix and an
//! **echo** mix. The echo mix goes round a ring buffer that also lives in
//! audio RAM, through an 8-tap FIR filter, and comes back as feedback and as a
//! second stereo signal under its own volume. One stereo sample leaves every
//! 32 SMP clocks: 1 024 000 / 32 = **32 000 Hz**, exactly.
//!
//! ```text
//!   $x0 VOLL  $x1 VOLR  $x2 PITCHL  $x3 PITCHH  $x4 SRCN     voice x (0-7)
//!   $x5 ADSR1 $x6 ADSR2 $x7 GAIN    $x8 ENVX    $x9 OUTX
//!   $0C MVOLL $1C MVOLR $2C EVOLL   $3C EVOLR   $4C KON  $5C KOFF  $6C FLG  $7C ENDX
//!   $0D EFB             $2D PMON    $3D NON     $4D EON  $5D DIR   $6D ESA  $7D EDL
//!   $xF FIRx                                    (everything else: plain RAM)
//! ```
//!
//! # Time: the 32-slot timeline
//!
//! The chip does not compute a sample and then wait. It spends the 32 clocks
//! of each sample period doing one small job per voice per clock, staggered so
//! the eight voices share two RAM accesses and three register-array accesses a
//! clock. fullsnes (*SNES APU Low Level Timings*) charts which register and
//! which RAM byte is touched in each of the 32 slots, and [`SDsp::clock`]
//! follows that chart: each call performs exactly the work of one slot. A
//! register the SMP writes therefore takes effect at the slot that reads it,
//! not at some arbitrary point — which is what makes KON's poll, ESA's
//! one-sample lag and the "write ADSR2 before ADSR1" race come out the way the
//! hardware does them.
//!
//! Per voice *v* the stages are (voice 0's are displaced by the echo unit's
//! slots, as the chart shows):
//!
//! | Stage | Voice 1-7 | Voice 0 | Does |
//! | --- | --- | --- | --- |
//! | SRCN | T(3v-6), V1 T21 | T18 | latch SRCN |
//! | DIR | T(3v-2) | T22 | latch PITCHL, ADSR1; read a directory entry |
//! | PITCHH | T(3v-1) | T23 | latch PITCHH |
//! | BRR1 | T(3v-1) | T26 | read the BRR header and the first data byte |
//! | ENV | T(3v-1) | T31 | latch ADSR2 (or GAIN), apply FLG.7, step the envelope |
//! | BRR2 | T(3v) | T0 | read the second data byte, latch VOLL; interpolate, decode, advance the pitch counter, mix left |
//! | VOLR | T(3v+1) | T1 | latch VOLR, mix right, update ENDX |
//! | ENVX | T(3v+2) | T2 | write ENVX |
//! | OUTX | T(3v+3) | T3 | write OUTX |
//!
//! and the global ones: echo reads at T23/T24 with the FIR taps latched over
//! T23-T26, the left and right outputs at T27/T28 (latching EFB and PMON),
//! NON/EON/DIR at T29, the left echo write, ESA, EDL and the rate-counter tick
//! at T30, and the right echo write, noise and the KON/KOFF poll at T31.
//!
//! # Integer arithmetic, and the hardware's clamps
//!
//! There is no float anywhere. Every width and every overflow rule below is the
//! chip's, as fullsnes states it: BRR results clamped to 16 bits and then
//! wrapped to 15 ("lost sign"); the Gaussian sum wrapping on its first three
//! terms and saturating on the fourth; voice mixing saturating after every
//! addition; the FIR wrapping on seven taps and saturating on the eighth.
//!
//! # Interpretations — where the documents leave a choice
//!
//! Written down so they can be checked against hardware rather than mistaken
//! for it.
//!
//! * **The decode buffer.** The chip reads a BRR header and two data bytes —
//!   four nibbles — every sample, and the pitch counter carries into the next
//!   group of four every `$4000`. This model keeps three groups of four decoded
//!   samples per voice; one group is decoded each time the counter crosses
//!   `$4000`, and the three groups before playback starts are decoded during
//!   the last three of the five KON-delay samples. That puts interpolation
//!   "centred on the second sample" at key-on (SNESdev wiki, *BRR samples*) and
//!   keeps ENDX clear right after a key-on even for a one-block sample.
//! * **The pitch step is capped at `$3FFF`** after pitch modulation. fullsnes
//!   marks the cap's exact position as unknown ("XXX somewhere here").
//! * **Decay ends when envelope bits 10-8 equal the sustain level** (SNESdev
//!   wiki, *S-DSP registers*). fullsnes phrases it as `Level <= (SL+1)*$100`;
//!   the two differ only for an envelope landing exactly on the boundary.
//! * **Phase changes are checked after each sample's envelope step**, in GAIN
//!   mode too, with the boundary taken from whichever byte the ENV slot latched
//!   — GAIN bits 7-5 in GAIN mode, the "garbage boundary" fullsnes describes.
//! * **Master, echo and feedback products wrap at 23 bits**, which is how this
//!   reads fullsnes's "-8000h*-80h=-400000h". Only volume `-128` against a
//!   full-scale `-$8000` is affected.
//! * **The rate counter ticks at T30.** The SNESdev wiki gives its period
//!   (`$7800` samples, decrementing from 0 so the first tick wraps) and the
//!   per-column offsets; the slot is this model's choice.
//! * **The output is the pre-inversion sum.** fullsnes ends the mixer with
//!   `XOR FFFFh`, the post-amplifier's phase inversion; it is inaudible, and
//!   leaving it out keeps a muted chip at exactly zero.
//! * **Reset** puts FLG at `$E0` (fullsnes: "keyed off, muted, echo-write off,
//!   noise stopped"), clears ENDX and the internal KON latch, silences every
//!   voice and rewinds the echo index, rate counter and noise generator. The
//!   rest of the register array keeps its contents; a new chip starts with it
//!   zeroed, so that power-on is deterministic.
//!
//! # Not modelled
//!
//! * The "63 clocks after the poll" timing of the internal KON bit clearing
//!   (SNESdev wiki). Here the poll consumes the latch outright, as fullsnes
//!   describes it.
//! * The analogue output stage: DAC, filter, amplifier.
//! * fullsnes's parenthetical that a BRR result clamped to `+$7FFF` ends up
//!   at `+$3FFF`. The 15-bit rule printed beside it maps `$7FFF` to `-1`,
//!   and this model follows the rule.
//!
//! # Sources
//!
//! nocash **fullsnes** (Martin Korth): *SNES APU DSP BRR Samples*, *BRR Pitch*
//! (including the Gaussian table, transcribed mechanically and checked against
//! its own claim that each four-entry set sums to `$7FF`-`$801`), *ADSR/Gain
//! Envelope* (the rate table), *Volume Registers*, *Control Registers*, *Echo
//! Registers* and *Low Level Timings*. The **SNESdev wiki**: *S-DSP registers*,
//! *DSP envelopes* (the period offsets, after anomie's S-DSP document) and *BRR
//! samples*. No emulator source was consulted.

use alloc::string::String;

use super::dsp_seam::{Aram, SDsp};
use crate::core::error::{Error, Result};
use crate::core::state::{ChunkReader, ChunkWriter, Sink, Source};

/// SMP clocks per output sample.
pub const CLOCKS_PER_SAMPLE: u8 = 32;

/// The output sample rate: 1 024 000 / 32.
pub const SAMPLE_RATE: u32 = 32_000;

/// How many registers the DSP has.
pub const REGISTER_COUNT: usize = 128;

/// Register addresses (the `$F2` index).
pub mod reg {
    /// Voice `v`'s register `offset` (one of the `V_*` constants).
    #[must_use]
    pub const fn voice(v: u8, offset: u8) -> u8 {
        ((v & 7) << 4) | (offset & 0x0F)
    }
    /// Voice left volume, signed.
    pub const V_VOLL: u8 = 0x0;
    /// Voice right volume, signed.
    pub const V_VOLR: u8 = 0x1;
    /// Voice pitch, low eight bits.
    pub const V_PITCHL: u8 = 0x2;
    /// Voice pitch, high six bits.
    pub const V_PITCHH: u8 = 0x3;
    /// Voice source number: the directory entry.
    pub const V_SRCN: u8 = 0x4;
    /// ADSR enable, decay rate, attack rate.
    pub const V_ADSR1: u8 = 0x5;
    /// Sustain level and sustain rate.
    pub const V_ADSR2: u8 = 0x6;
    /// GAIN, when ADSR1 bit 7 is clear.
    pub const V_GAIN: u8 = 0x7;
    /// The envelope's top seven bits, written by the DSP.
    pub const V_ENVX: u8 = 0x8;
    /// The voice output's top eight bits, written by the DSP.
    pub const V_OUTX: u8 = 0x9;
    /// Main volume, left.
    pub const MVOLL: u8 = 0x0C;
    /// Main volume, right.
    pub const MVOLR: u8 = 0x1C;
    /// Echo volume, left.
    pub const EVOLL: u8 = 0x2C;
    /// Echo volume, right.
    pub const EVOLR: u8 = 0x3C;
    /// Key on.
    pub const KON: u8 = 0x4C;
    /// Key off.
    pub const KOFF: u8 = 0x5C;
    /// Soft reset, mute, echo-write disable, noise rate.
    pub const FLG: u8 = 0x6C;
    /// Voice end flags; any write clears all eight.
    pub const ENDX: u8 = 0x7C;
    /// Echo feedback volume.
    pub const EFB: u8 = 0x0D;
    /// Pitch modulation enable, voices 1-7.
    pub const PMON: u8 = 0x2D;
    /// Noise enable.
    pub const NON: u8 = 0x3D;
    /// Echo enable.
    pub const EON: u8 = 0x4D;
    /// Sample directory page.
    pub const DIR: u8 = 0x5D;
    /// Echo buffer page.
    pub const ESA: u8 = 0x6D;
    /// Echo delay: buffer length in 2 KiB steps.
    pub const EDL: u8 = 0x7D;
    /// FIR coefficient `n` (0 = applied to the oldest sample).
    #[must_use]
    pub const fn fir(n: u8) -> u8 {
        ((n & 7) << 4) | 0x0F
    }
}

/// FLG bit 7: key every voice off and zero its envelope.
const FLG_RESET: u8 = 0x80;
/// FLG bit 6: mute the amplifier.
const FLG_MUTE: u8 = 0x40;
/// FLG bit 5: disable echo-buffer writes.
const FLG_ECHO_OFF: u8 = 0x20;
/// FLG's value after reset.
const FLG_AT_RESET: u8 = 0xE0;

/// The rate counter's period in samples: a multiple of every entry in
/// [`RATE_PERIOD`] (SNESdev wiki, *DSP envelopes*).
const RATE_COUNTER_TOP: u16 = 0x77FF;

/// Samples between envelope (and noise) steps for each 5-bit rate; 0 is never
/// (fullsnes, *ADSR/Gain (and Noise) Rates*).
const RATE_PERIOD: [u16; 32] = [
    0, 2048, 1536, 1280, 1024, 768, 640, 512, 384, 320, 256, 192, 160, 128, 96, 80, 64, 48, 40, 32,
    24, 20, 16, 12, 10, 8, 6, 5, 4, 3, 2, 1,
];

/// The phase offset of each rate's column (SNESdev wiki, *DSP envelopes*,
/// "DSP Period Offset"): rates ≡ 0 mod 3 use 536, ≡ 1 use 0, ≡ 2 use 1040.
const fn rate_offset(rate: u8) -> u16 {
    match rate % 3 {
        0 => 536,
        1 => 0,
        _ => 1040,
    }
}

/// Whether `rate` steps on the sample whose rate counter is `counter`.
#[inline]
fn rate_fires(rate: u8, counter: u16) -> bool {
    let rate = rate & 0x1F;
    if rate == 0 {
        return false;
    }
    let period = RATE_PERIOD[usize::from(rate)];
    (counter + rate_offset(rate)).is_multiple_of(period)
}

/// The 4-point Gaussian interpolation table, 512 entries (fullsnes, *BRR
/// Pitch*, "4-Point Gaussian Interpolation"). Hardware fact, transcribed.
const GAUSS: [u16; 512] = [
    0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000,
    0x000, 0x000, 0x000, 0x001, 0x001, 0x001, 0x001, 0x001, 0x001, 0x001, 0x001, 0x001, 0x001,
    0x001, 0x002, 0x002, 0x002, 0x002, 0x002, 0x002, 0x002, 0x003, 0x003, 0x003, 0x003, 0x003,
    0x004, 0x004, 0x004, 0x004, 0x004, 0x005, 0x005, 0x005, 0x005, 0x006, 0x006, 0x006, 0x006,
    0x007, 0x007, 0x007, 0x008, 0x008, 0x008, 0x009, 0x009, 0x009, 0x00A, 0x00A, 0x00A, 0x00B,
    0x00B, 0x00B, 0x00C, 0x00C, 0x00D, 0x00D, 0x00E, 0x00E, 0x00F, 0x00F, 0x00F, 0x010, 0x010,
    0x011, 0x011, 0x012, 0x013, 0x013, 0x014, 0x014, 0x015, 0x015, 0x016, 0x017, 0x017, 0x018,
    0x018, 0x019, 0x01A, 0x01B, 0x01B, 0x01C, 0x01D, 0x01D, 0x01E, 0x01F, 0x020, 0x020, 0x021,
    0x022, 0x023, 0x024, 0x024, 0x025, 0x026, 0x027, 0x028, 0x029, 0x02A, 0x02B, 0x02C, 0x02D,
    0x02E, 0x02F, 0x030, 0x031, 0x032, 0x033, 0x034, 0x035, 0x036, 0x037, 0x038, 0x03A, 0x03B,
    0x03C, 0x03D, 0x03E, 0x040, 0x041, 0x042, 0x043, 0x045, 0x046, 0x047, 0x049, 0x04A, 0x04C,
    0x04D, 0x04E, 0x050, 0x051, 0x053, 0x054, 0x056, 0x057, 0x059, 0x05A, 0x05C, 0x05E, 0x05F,
    0x061, 0x063, 0x064, 0x066, 0x068, 0x06A, 0x06B, 0x06D, 0x06F, 0x071, 0x073, 0x075, 0x076,
    0x078, 0x07A, 0x07C, 0x07E, 0x080, 0x082, 0x084, 0x086, 0x089, 0x08B, 0x08D, 0x08F, 0x091,
    0x093, 0x096, 0x098, 0x09A, 0x09C, 0x09F, 0x0A1, 0x0A3, 0x0A6, 0x0A8, 0x0AB, 0x0AD, 0x0AF,
    0x0B2, 0x0B4, 0x0B7, 0x0BA, 0x0BC, 0x0BF, 0x0C1, 0x0C4, 0x0C7, 0x0C9, 0x0CC, 0x0CF, 0x0D2,
    0x0D4, 0x0D7, 0x0DA, 0x0DD, 0x0E0, 0x0E3, 0x0E6, 0x0E9, 0x0EC, 0x0EF, 0x0F2, 0x0F5, 0x0F8,
    0x0FB, 0x0FE, 0x101, 0x104, 0x107, 0x10B, 0x10E, 0x111, 0x114, 0x118, 0x11B, 0x11E, 0x122,
    0x125, 0x129, 0x12C, 0x130, 0x133, 0x137, 0x13A, 0x13E, 0x141, 0x145, 0x148, 0x14C, 0x150,
    0x153, 0x157, 0x15B, 0x15F, 0x162, 0x166, 0x16A, 0x16E, 0x172, 0x176, 0x17A, 0x17D, 0x181,
    0x185, 0x189, 0x18D, 0x191, 0x195, 0x19A, 0x19E, 0x1A2, 0x1A6, 0x1AA, 0x1AE, 0x1B2, 0x1B7,
    0x1BB, 0x1BF, 0x1C3, 0x1C8, 0x1CC, 0x1D0, 0x1D5, 0x1D9, 0x1DD, 0x1E2, 0x1E6, 0x1EB, 0x1EF,
    0x1F3, 0x1F8, 0x1FC, 0x201, 0x205, 0x20A, 0x20F, 0x213, 0x218, 0x21C, 0x221, 0x226, 0x22A,
    0x22F, 0x233, 0x238, 0x23D, 0x241, 0x246, 0x24B, 0x250, 0x254, 0x259, 0x25E, 0x263, 0x267,
    0x26C, 0x271, 0x276, 0x27B, 0x280, 0x284, 0x289, 0x28E, 0x293, 0x298, 0x29D, 0x2A2, 0x2A6,
    0x2AB, 0x2B0, 0x2B5, 0x2BA, 0x2BF, 0x2C4, 0x2C9, 0x2CE, 0x2D3, 0x2D8, 0x2DC, 0x2E1, 0x2E6,
    0x2EB, 0x2F0, 0x2F5, 0x2FA, 0x2FF, 0x304, 0x309, 0x30E, 0x313, 0x318, 0x31D, 0x322, 0x326,
    0x32B, 0x330, 0x335, 0x33A, 0x33F, 0x344, 0x349, 0x34E, 0x353, 0x357, 0x35C, 0x361, 0x366,
    0x36B, 0x370, 0x374, 0x379, 0x37E, 0x383, 0x388, 0x38C, 0x391, 0x396, 0x39B, 0x39F, 0x3A4,
    0x3A9, 0x3AD, 0x3B2, 0x3B7, 0x3BB, 0x3C0, 0x3C5, 0x3C9, 0x3CE, 0x3D2, 0x3D7, 0x3DC, 0x3E0,
    0x3E5, 0x3E9, 0x3ED, 0x3F2, 0x3F6, 0x3FB, 0x3FF, 0x403, 0x408, 0x40C, 0x410, 0x415, 0x419,
    0x41D, 0x421, 0x425, 0x42A, 0x42E, 0x432, 0x436, 0x43A, 0x43E, 0x442, 0x446, 0x44A, 0x44E,
    0x452, 0x455, 0x459, 0x45D, 0x461, 0x465, 0x468, 0x46C, 0x470, 0x473, 0x477, 0x47A, 0x47E,
    0x481, 0x485, 0x488, 0x48C, 0x48F, 0x492, 0x496, 0x499, 0x49C, 0x49F, 0x4A2, 0x4A6, 0x4A9,
    0x4AC, 0x4AF, 0x4B2, 0x4B5, 0x4B7, 0x4BA, 0x4BD, 0x4C0, 0x4C3, 0x4C5, 0x4C8, 0x4CB, 0x4CD,
    0x4D0, 0x4D2, 0x4D5, 0x4D7, 0x4D9, 0x4DC, 0x4DE, 0x4E0, 0x4E3, 0x4E5, 0x4E7, 0x4E9, 0x4EB,
    0x4ED, 0x4EF, 0x4F1, 0x4F3, 0x4F5, 0x4F6, 0x4F8, 0x4FA, 0x4FB, 0x4FD, 0x4FF, 0x500, 0x502,
    0x503, 0x504, 0x506, 0x507, 0x508, 0x50A, 0x50B, 0x50C, 0x50D, 0x50E, 0x50F, 0x510, 0x511,
    0x511, 0x512, 0x513, 0x514, 0x514, 0x515, 0x516, 0x516, 0x517, 0x517, 0x517, 0x518, 0x518,
    0x518, 0x518, 0x518, 0x519, 0x519,
];

/// Saturate to a signed 16-bit value.
#[inline]
fn clamp16(x: i32) -> i32 {
    x.clamp(i32::from(i16::MIN), i32::from(i16::MAX))
}

/// Keep the low 16 bits, sign-extended: the hardware's unsaturated adder.
#[inline]
fn wrap16(x: i32) -> i32 {
    i32::from(x as i16)
}

/// Keep the low 15 bits, sign-extended: fullsnes's "lost sign" clip.
#[inline]
fn wrap15(x: i32) -> i32 {
    i32::from((x as i16).wrapping_shl(1) >> 1)
}

/// A volume product as the 16×8 multiplier gives it: 23 bits, wrapping.
#[inline]
fn mul_vol(sample: i32, vol: u8) -> i32 {
    (sample * i32::from(vol as i8)).wrapping_shl(9) >> 9
}

/// The exponential decrease step: `-(((L-1) SAR 8)+1)` (fullsnes).
#[inline]
fn exp_decrease(env: i32) -> i32 {
    env - (((env - 1) >> 8) + 1)
}

/// One step of the noise generator, on its 15-bit register (fullsnes, *Control
/// Registers*, NON).
#[must_use]
pub fn noise_step(level: u16) -> u16 {
    let feedback = (level ^ (level >> 1)) & 1;
    ((level >> 1) & 0x3FFF) | (feedback << 14)
}

/// Decode one group of four BRR nibbles into `ring` at `pos`, using the two
/// samples before `pos` as the filter's history. Returns the position after.
///
/// fullsnes, *BRR Samples*: `sample = (nibble SHL shift) SAR 1`, shifts 13-15
/// act as 12 on `nibble SAR 3`; the four filters' exact formulas; the clamp to
/// 16 bits and the wrap to 15.
fn decode_group(header: u8, data: [u8; 2], ring: &mut [i16; 12], pos: usize) -> usize {
    let shift = header >> 4;
    let filter = (header >> 2) & 3;
    let nibbles = [data[0] >> 4, data[0] & 0xF, data[1] >> 4, data[1] & 0xF];
    let mut pos = pos;
    for nibble in nibbles {
        let n = i32::from(((nibble << 4) as i8) >> 4);
        let s = if shift <= 12 {
            (n << shift) >> 1
        } else {
            ((n >> 3) << 12) >> 1
        };
        let old = i32::from(ring[(pos + 11) % 12]);
        let older = i32::from(ring[(pos + 10) % 12]);
        let new = match filter {
            0 => s,
            1 => s + old + ((-old) >> 4),
            2 => s + old * 2 + ((-old * 3) >> 5) - older + (older >> 4),
            _ => s + old * 2 + ((-old * 13) >> 6) - older + ((older * 3) >> 4),
        };
        ring[pos] = wrap15(clamp16(new)) as i16;
        pos = (pos + 1) % 12;
    }
    pos
}

/// Gaussian interpolation of four samples at fraction index `i` (0-255).
///
/// The first three products are summed without overflow handling, the fourth
/// saturates, and the result drops to 15 bits (fullsnes).
fn gauss_interpolate(oldest: i32, older: i32, old: i32, new: i32, i: usize) -> i32 {
    let g = |k: usize| i32::from(GAUSS[k]);
    let mut out = (g(0xFF - i) * oldest) >> 10;
    out += (g(0x1FF - i) * older) >> 10;
    out += (g(0x100 + i) * old) >> 10;
    out = wrap16(out);
    out = clamp16(out + ((g(i) * new) >> 10));
    out >> 1
}

/// The pitch counter's step for one sample: VxPITCH, scaled by the previous
/// voice's output under PMON, capped at `$3FFF` (fullsnes, *Pitch Counter*).
#[must_use]
pub fn pitch_step(pitch: u16, modulator: Option<i16>) -> u16 {
    let pitch = pitch & 0x3FFF;
    match modulator {
        None => pitch,
        Some(out) => {
            let factor = (i32::from(out) >> 4) + 0x400;
            let step = (i32::from(pitch) * factor) >> 10;
            step.clamp(0, 0x3FFF) as u16
        }
    }
}

/// The FIR: seven taps wrapping, the eighth saturating (fullsnes, *Echo
/// Registers*). `pos` is the newest entry; coefficient 0 is the oldest.
fn fir(buf: &[i16; 8], pos: usize, coef: &[u8; 8]) -> i32 {
    let mut sum = 0i32;
    for (k, &c) in coef.iter().enumerate().take(7) {
        let s = i32::from(buf[(pos + 1 + k) & 7]);
        sum += (s * i32::from(c as i8)) >> 6;
    }
    let sum = wrap16(sum);
    clamp16(sum + ((i32::from(buf[pos & 7]) * i32::from(coef[7] as i8)) >> 6))
}

/// Envelope phase. A real enum: there are exactly four.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum EnvMode {
    Attack,
    Decay,
    Sustain,
    Release,
}

impl EnvMode {
    fn from_u8(v: u8) -> Option<Self> {
        Some(match v {
            0 => EnvMode::Attack,
            1 => EnvMode::Decay,
            2 => EnvMode::Sustain,
            3 => EnvMode::Release,
            _ => return None,
        })
    }

    fn as_u8(self) -> u8 {
        match self {
            EnvMode::Attack => 0,
            EnvMode::Decay => 1,
            EnvMode::Sustain => 2,
            EnvMode::Release => 3,
        }
    }
}

/// One voice's internal state: latches, decoder, envelope.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct Voice {
    srcn: u8,
    adsr1: u8,
    /// ADSR2 or GAIN, whichever the ENV slot read.
    adsr2: u8,
    pitch_lo: u8,
    pitch_hi: u8,
    loop_addr: u16,
    brr_addr: u16,
    /// Offset of the next data-byte pair in the 9-byte block: 1, 3, 5 or 7.
    brr_offset: u8,
    header: u8,
    data: [u8; 2],
    ring: [i16; 12],
    /// Where the next decoded group goes, and the oldest sample: 0, 4 or 8.
    ring_pos: u8,
    /// Pitch counter: bits 14-12 pick the sample, 11-4 the Gaussian index.
    counter: u16,
    env: u16,
    /// The last envelope result, clipped to 11 bits rather than clamped — what
    /// bent-increase GAIN compares against (fullsnes, "Misc Notes").
    hidden_env: u16,
    mode: EnvMode,
    /// Keyed on at the poll; the next DIR slot starts the sample.
    kon_pending: bool,
    /// Samples of KON delay left.
    kon_delay: u8,
    /// The latest 15-bit output, after the envelope.
    out: i16,
    /// An end block finished; ENDX is set at the VOLR slot.
    end_seen: bool,
    /// Keyed on; ENDX is cleared at the VOLR slot.
    endx_clear: bool,
}

impl Default for Voice {
    fn default() -> Self {
        Voice {
            srcn: 0,
            adsr1: 0,
            adsr2: 0,
            pitch_lo: 0,
            pitch_hi: 0,
            loop_addr: 0,
            brr_addr: 0,
            brr_offset: 1,
            header: 0,
            data: [0; 2],
            ring: [0; 12],
            ring_pos: 0,
            counter: 0,
            env: 0,
            hidden_env: 0,
            mode: EnvMode::Release,
            kon_pending: false,
            kon_delay: 0,
            out: 0,
            end_seen: false,
            endx_clear: false,
        }
    }
}

/// A per-voice stage of the timeline.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Stage {
    Srcn,
    Dir,
    PitchH,
    Brr1,
    Env,
    Brr2,
    VolR,
    Envx,
    Outx,
}

const STAGES: [Stage; 9] = [
    Stage::Srcn,
    Stage::Dir,
    Stage::PitchH,
    Stage::Brr1,
    Stage::Env,
    Stage::Brr2,
    Stage::VolR,
    Stage::Envx,
    Stage::Outx,
];

/// The slot (0-31) in which voice `v` performs `stage` (fullsnes, *Register
/// and RAM Access Timing Chart*).
const fn slot(v: usize, stage: Stage) -> usize {
    if v == 0 {
        return match stage {
            Stage::Srcn => 18,
            Stage::Dir => 22,
            Stage::PitchH => 23,
            Stage::Brr1 => 26,
            Stage::Env => 31,
            Stage::Brr2 => 0,
            Stage::VolR => 1,
            Stage::Envx => 2,
            Stage::Outx => 3,
        };
    }
    let b = 3 * v;
    match stage {
        Stage::Srcn => {
            if v == 1 {
                21
            } else {
                b - 6
            }
        }
        Stage::Dir => b - 2,
        Stage::PitchH | Stage::Brr1 | Stage::Env => b - 1,
        Stage::Brr2 => b,
        Stage::VolR => b + 1,
        Stage::Envx => b + 2,
        Stage::Outx => b + 3,
    }
}

/// Entries per slot in [`SCHEDULE`]; the busiest slot has four.
const SLOT_WIDTH: usize = 6;
/// An empty [`SCHEDULE`] entry.
const NONE: u8 = 0xFF;

/// For each slot, the `(voice << 4) | stage index` jobs it runs, in order.
const SCHEDULE: [[u8; SLOT_WIDTH]; 32] = build_schedule();

const fn build_schedule() -> [[u8; SLOT_WIDTH]; 32] {
    let mut table = [[NONE; SLOT_WIDTH]; 32];
    let mut fill = [0usize; 32];
    let mut v = 0;
    while v < 8 {
        let mut s = 0;
        while s < STAGES.len() {
            let t = slot(v, STAGES[s]);
            table[t][fill[t]] = ((v as u8) << 4) | s as u8;
            fill[t] += 1;
            s += 1;
        }
        v += 1;
    }
    table
}

/// Read a little-endian 16-bit value, wrapping at the top of audio RAM.
fn read16(aram: &dyn Aram, addr: u16) -> u16 {
    u16::from(aram.read(addr)) | (u16::from(aram.read(addr.wrapping_add(1))) << 8)
}

/// Write a little-endian 16-bit value, wrapping at the top of audio RAM.
fn write16(aram: &mut dyn Aram, addr: u16, value: u16) {
    aram.write(addr, value as u8);
    aram.write(addr.wrapping_add(1), (value >> 8) as u8);
}

/// The S-DSP.
///
/// A plain state machine with no device-graph presence of its own: the S-SMP
/// owns it, forwards `$F3` to [`SDsp::read`]/[`SDsp::write`], and clocks it
/// once per SMP clock with audio RAM in hand.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct SDspChip {
    regs: [u8; REGISTER_COUNT],
    endx: u8,
    /// The internal KON value: the last write, consumed by the poll.
    kon_latch: u8,
    /// Toggles each sample; KON/KOFF are polled when it turns true.
    kon_phase: bool,
    /// The slot the next [`SDsp::clock`] performs.
    cycle: u8,
    rate_counter: u16,
    /// 15-bit noise level.
    noise: u16,
    // Latched globals.
    dir: u8,
    pmon: u8,
    non: u8,
    eon: u8,
    esa: u8,
    efb: u8,
    // Echo.
    echo_addr: u16,
    echo_idx: u16,
    echo_len: u16,
    fir_pos: u8,
    fir_l: [i16; 8],
    fir_r: [i16; 8],
    fir_coef: [u8; 8],
    fir_out: [i16; 2],
    // Accumulators.
    main: [i16; 2],
    echo: [i16; 2],
    output: [i16; 2],
    voices: [Voice; 8],
}

impl Default for SDspChip {
    fn default() -> Self {
        Self::new()
    }
}

impl SDspChip {
    /// A powered-on chip: registers zeroed, then reset.
    #[must_use]
    pub fn new() -> Self {
        let mut chip = SDspChip {
            regs: [0; REGISTER_COUNT],
            endx: 0,
            kon_latch: 0,
            kon_phase: false,
            cycle: 0,
            rate_counter: 0,
            noise: 0x4000,
            dir: 0,
            pmon: 0,
            non: 0,
            eon: 0,
            esa: 0,
            efb: 0,
            echo_addr: 0,
            echo_idx: 0,
            echo_len: 0,
            fir_pos: 0,
            fir_l: [0; 8],
            fir_r: [0; 8],
            fir_coef: [0; 8],
            fir_out: [0; 2],
            main: [0; 2],
            echo: [0; 2],
            output: [0; 2],
            voices: [Voice::default(); 8],
        };
        chip.reset_state();
        chip
    }

    /// The slot the next clock performs (0-31).
    #[must_use]
    pub fn slot(&self) -> u8 {
        self.cycle
    }

    /// The most recent stereo output.
    #[must_use]
    pub fn output(&self) -> (i16, i16) {
        (self.output[0], self.output[1])
    }

    /// Voice `v`'s 11-bit envelope (`ENVX` shows only the top seven bits).
    #[must_use]
    pub fn envelope(&self, v: usize) -> u16 {
        self.voices[v & 7].env
    }

    /// Voice `v`'s current 15-bit output, after the envelope.
    #[must_use]
    pub fn voice_output(&self, v: usize) -> i16 {
        self.voices[v & 7].out
    }

    /// Run a whole sample period from wherever the timeline is, returning the
    /// sample produced at its end.
    pub fn run_sample(&mut self, aram: &mut dyn Aram) -> (i16, i16) {
        loop {
            if let Some(s) = self.clock(aram) {
                return s;
            }
        }
    }

    fn reset_state(&mut self) {
        self.regs[usize::from(reg::FLG)] = FLG_AT_RESET;
        self.endx = 0;
        self.kon_latch = 0;
        self.kon_phase = false;
        self.cycle = 0;
        self.rate_counter = 0;
        self.noise = 0x4000;
        self.echo_idx = 0;
        self.echo_len = 0;
        self.echo_addr = 0;
        self.fir_pos = 0;
        self.fir_l = [0; 8];
        self.fir_r = [0; 8];
        self.fir_out = [0; 2];
        self.main = [0; 2];
        self.echo = [0; 2];
        self.output = [0; 2];
        self.voices = [Voice::default(); 8];
    }

    #[inline]
    fn vreg(&self, v: usize, offset: u8) -> u8 {
        self.regs[usize::from(reg::voice(v as u8, offset))]
    }

    fn flg(&self) -> u8 {
        self.regs[usize::from(reg::FLG)]
    }

    fn voice_stage(&mut self, v: usize, stage: Stage, aram: &mut dyn Aram) {
        match stage {
            Stage::Srcn => self.voices[v].srcn = self.vreg(v, reg::V_SRCN),
            Stage::Dir => self.stage_dir(v, aram),
            Stage::PitchH => self.voices[v].pitch_hi = self.vreg(v, reg::V_PITCHH),
            Stage::Brr1 => {
                let voice = &mut self.voices[v];
                voice.header = aram.read(voice.brr_addr);
                voice.data[0] = aram.read(voice.brr_addr.wrapping_add(u16::from(voice.brr_offset)));
            }
            Stage::Env => self.stage_env(v),
            Stage::Brr2 => self.stage_brr2(v, aram),
            Stage::VolR => {
                let vol = self.vreg(v, reg::V_VOLR);
                let bit = 1u8 << v;
                let out = i32::from(self.voices[v].out);
                self.mix(1, out, vol, self.eon & bit != 0);
                let voice = &mut self.voices[v];
                if voice.endx_clear {
                    self.endx &= !bit;
                    voice.endx_clear = false;
                }
                if voice.end_seen {
                    self.endx |= bit;
                    voice.end_seen = false;
                }
            }
            Stage::Envx => {
                let envx = (self.voices[v].env >> 4) as u8;
                self.regs[usize::from(reg::voice(v as u8, reg::V_ENVX))] = envx;
            }
            Stage::Outx => {
                let outx = (self.voices[v].out >> 7) as u8;
                self.regs[usize::from(reg::voice(v as u8, reg::V_OUTX))] = outx;
            }
        }
    }

    fn stage_dir(&mut self, v: usize, aram: &mut dyn Aram) {
        let pitch_lo = self.vreg(v, reg::V_PITCHL);
        let adsr1 = self.vreg(v, reg::V_ADSR1);
        let dir = self.dir;
        let voice = &mut self.voices[v];
        voice.pitch_lo = pitch_lo;
        voice.adsr1 = adsr1;
        let entry = (u16::from(dir) << 8).wrapping_add(u16::from(voice.srcn) * 4);
        if voice.kon_pending {
            voice.brr_addr = read16(aram, entry);
            voice.brr_offset = 1;
            voice.ring_pos = 0;
            voice.counter = 0;
            voice.kon_delay = 5;
            voice.kon_pending = false;
            voice.endx_clear = true;
            voice.end_seen = false;
        } else {
            voice.loop_addr = read16(aram, entry.wrapping_add(2));
        }
    }

    fn stage_env(&mut self, v: usize) {
        let offset = if self.voices[v].adsr1 & 0x80 != 0 {
            reg::V_ADSR2
        } else {
            reg::V_GAIN
        };
        let latched = self.vreg(v, offset);
        let soft_reset = self.flg() & FLG_RESET != 0;
        let counter = self.rate_counter;
        let voice = &mut self.voices[v];
        voice.adsr2 = latched;
        if soft_reset {
            voice.mode = EnvMode::Release;
            voice.env = 0;
            voice.hidden_env = 0;
        }
        if voice.kon_pending || voice.kon_delay > 0 {
            return;
        }
        step_envelope(voice, counter);
    }

    fn stage_brr2(&mut self, v: usize, aram: &mut dyn Aram) {
        let vol = self.vreg(v, reg::V_VOLL);
        let bit = 1u8 << v;
        let modulator = (v > 0 && self.pmon & bit != 0).then(|| self.voices[v - 1].out);
        let noise = (self.non & bit != 0).then(|| wrap15(i32::from(self.noise)));
        let pitch = u16::from(self.voices[v].pitch_lo) | (u16::from(self.voices[v].pitch_hi) << 8);
        let voice = &mut self.voices[v];
        voice.data[1] = aram.read(
            voice
                .brr_addr
                .wrapping_add(u16::from(voice.brr_offset))
                .wrapping_add(1),
        );
        if voice.kon_delay > 0 {
            if voice.kon_delay <= 3 {
                decode_into(voice);
            }
            voice.kon_delay -= 1;
            voice.out = 0;
        } else {
            let sample = match noise {
                Some(n) => n,
                None => {
                    let i = usize::from((voice.counter >> 4) & 0xFF);
                    let base = usize::from(voice.ring_pos) + usize::from(voice.counter >> 12);
                    let s = |k: usize| i32::from(voice.ring[(base + k) % 12]);
                    gauss_interpolate(s(0), s(1), s(2), s(3), i)
                }
            };
            voice.out = ((sample * i32::from(voice.env)) >> 11) as i16;
            if voice.counter >= 0x4000 {
                decode_into(voice);
                voice.counter -= 0x4000;
            }
            voice.counter += pitch_step(pitch, modulator);
        }
        let out = i32::from(voice.out);
        self.mix(0, out, vol, self.eon & bit != 0);
    }

    /// Add a voice into channel `ch` of the main mix, and of the echo mix
    /// when EON selects it; saturating after each addition (fullsnes, *Output
    /// Mixer*).
    fn mix(&mut self, ch: usize, out: i32, vol: u8, echo: bool) {
        let term = (out * i32::from(vol as i8)) >> 6;
        self.main[ch] = clamp16(i32::from(self.main[ch]) + term) as i16;
        if echo {
            self.echo[ch] = clamp16(i32::from(self.echo[ch]) + term) as i16;
        }
    }

    fn global_stage(&mut self, t: u8, aram: &mut dyn Aram) {
        match t {
            23 => {
                self.echo_addr =
                    (u16::from(self.esa) << 8).wrapping_add(self.echo_idx.wrapping_mul(4));
                self.fir_pos = (self.fir_pos + 1) & 7;
                self.fir_l[usize::from(self.fir_pos)] = (read16(aram, self.echo_addr) as i16) >> 1;
                self.fir_coef[0] = self.regs[usize::from(reg::fir(0))];
            }
            24 => {
                self.fir_r[usize::from(self.fir_pos)] =
                    (read16(aram, self.echo_addr.wrapping_add(2)) as i16) >> 1;
                self.fir_coef[1] = self.regs[usize::from(reg::fir(1))];
                self.fir_coef[2] = self.regs[usize::from(reg::fir(2))];
            }
            25 => {
                for n in 3..6 {
                    self.fir_coef[usize::from(n)] = self.regs[usize::from(reg::fir(n))];
                }
            }
            26 => {
                self.fir_coef[6] = self.regs[usize::from(reg::fir(6))];
                self.fir_coef[7] = self.regs[usize::from(reg::fir(7))];
                let pos = usize::from(self.fir_pos);
                self.fir_out[0] = fir(&self.fir_l, pos, &self.fir_coef) as i16;
                self.fir_out[1] = fir(&self.fir_r, pos, &self.fir_coef) as i16;
            }
            27 => {
                self.output_channel(0, reg::MVOLL, reg::EVOLL);
                self.efb = self.regs[usize::from(reg::EFB)];
            }
            28 => {
                self.output_channel(1, reg::MVOLR, reg::EVOLR);
                self.pmon = self.regs[usize::from(reg::PMON)] & 0xFE;
            }
            29 => {
                self.non = self.regs[usize::from(reg::NON)];
                self.eon = self.regs[usize::from(reg::EON)];
                self.dir = self.regs[usize::from(reg::DIR)];
            }
            30 => {
                self.echo_write(0, self.echo_addr, aram);
                self.esa = self.regs[usize::from(reg::ESA)];
                if self.echo_idx == 0 {
                    self.echo_len = u16::from(self.regs[usize::from(reg::EDL)] & 0x0F) << 9;
                }
                self.echo_idx += 1;
                if self.echo_idx >= self.echo_len {
                    self.echo_idx = 0;
                }
                self.rate_counter = match self.rate_counter {
                    0 => RATE_COUNTER_TOP,
                    c => c - 1,
                };
            }
            31 => {
                self.echo_write(1, self.echo_addr.wrapping_add(2), aram);
                if rate_fires(self.flg() & 0x1F, self.rate_counter) {
                    self.noise = noise_step(self.noise);
                }
                self.poll_kon();
            }
            _ => {}
        }
    }

    /// Main volume over the voice mix, plus echo volume over the FIR output,
    /// saturated; zero when muted.
    fn output_channel(&mut self, ch: usize, mvol: u8, evol: u8) {
        let main = mul_vol(i32::from(self.main[ch]), self.regs[usize::from(mvol)]) >> 7;
        let echo = mul_vol(i32::from(self.fir_out[ch]), self.regs[usize::from(evol)]) >> 7;
        self.output[ch] = if self.flg() & FLG_MUTE != 0 {
            0
        } else {
            clamp16(main + echo) as i16
        };
        self.main[ch] = 0;
    }

    /// Echo mix plus feedback, bit 0 cleared, into the buffer unless FLG.5.
    fn echo_write(&mut self, ch: usize, addr: u16, aram: &mut dyn Aram) {
        let feedback = mul_vol(i32::from(self.fir_out[ch]), self.efb) >> 7;
        let value = clamp16(i32::from(self.echo[ch]) + feedback) & !1;
        if self.flg() & FLG_ECHO_OFF == 0 {
            write16(aram, addr, value as u16);
        }
        self.echo[ch] = 0;
    }

    /// The every-other-sample KON/KOFF poll (fullsnes, *KON/KOFF Notes*):
    /// KOFF releases, then the internal KON keys on, then the latch clears.
    fn poll_kon(&mut self) {
        self.kon_phase = !self.kon_phase;
        if !self.kon_phase {
            return;
        }
        let koff = self.regs[usize::from(reg::KOFF)];
        let kon = self.kon_latch;
        self.kon_latch = 0;
        for (v, voice) in self.voices.iter_mut().enumerate() {
            let bit = 1u8 << v;
            if koff & bit != 0 {
                voice.mode = EnvMode::Release;
            }
            if kon & bit != 0 {
                voice.kon_pending = true;
                voice.mode = EnvMode::Attack;
                voice.env = 0;
                voice.hidden_env = 0;
            }
        }
    }

    /// Write the chip's state, field by field.
    ///
    /// # Errors
    ///
    /// If the sink fails.
    pub fn save_state(&self, w: &mut ChunkWriter<'_>) -> Result<()> {
        w.write_bytes(&self.regs)?;
        for b in [
            self.endx,
            self.kon_latch,
            self.cycle,
            self.dir,
            self.pmon,
            self.non,
            self.eon,
            self.esa,
            self.efb,
            self.fir_pos,
        ] {
            w.write_u8(b)?;
        }
        w.write_bool(self.kon_phase)?;
        for h in [
            self.rate_counter,
            self.noise,
            self.echo_addr,
            self.echo_idx,
            self.echo_len,
        ] {
            w.write_u16(h)?;
        }
        for s in self
            .fir_l
            .iter()
            .chain(&self.fir_r)
            .chain(&self.fir_out)
            .chain(&self.main)
            .chain(&self.echo)
            .chain(&self.output)
        {
            w.write_i16(*s)?;
        }
        w.write_bytes(&self.fir_coef)?;
        for voice in &self.voices {
            for b in [
                voice.srcn,
                voice.adsr1,
                voice.adsr2,
                voice.pitch_lo,
                voice.pitch_hi,
                voice.brr_offset,
                voice.header,
                voice.data[0],
                voice.data[1],
                voice.ring_pos,
                voice.mode.as_u8(),
                voice.kon_delay,
            ] {
                w.write_u8(b)?;
            }
            for h in [
                voice.loop_addr,
                voice.brr_addr,
                voice.counter,
                voice.env,
                voice.hidden_env,
            ] {
                w.write_u16(h)?;
            }
            for s in voice.ring {
                w.write_i16(s)?;
            }
            w.write_i16(voice.out)?;
            w.write_bool(voice.kon_pending)?;
            w.write_bool(voice.end_seen)?;
            w.write_bool(voice.endx_clear)?;
        }
        Ok(())
    }

    /// Restore what [`SDspChip::save_state`] wrote. The chip is unchanged if
    /// this fails.
    ///
    /// # Errors
    ///
    /// If the chunk is truncated or describes a state the chip cannot be in.
    pub fn load_state(&mut self, r: &mut ChunkReader<'_>) -> Result<()> {
        let bad = |what: &str| Error::State(String::from("s-dsp snapshot: ") + what);
        let mut chip = SDspChip::new();
        let regs = r.read_bytes()?;
        if regs.len() != REGISTER_COUNT {
            return Err(bad("the register file is the wrong size"));
        }
        chip.regs.copy_from_slice(regs);
        chip.endx = r.read_u8()?;
        chip.kon_latch = r.read_u8()?;
        chip.cycle = r.read_u8()?;
        chip.dir = r.read_u8()?;
        chip.pmon = r.read_u8()?;
        chip.non = r.read_u8()?;
        chip.eon = r.read_u8()?;
        chip.esa = r.read_u8()?;
        chip.efb = r.read_u8()?;
        chip.fir_pos = r.read_u8()?;
        chip.kon_phase = r.read_bool()?;
        chip.rate_counter = r.read_u16()?;
        chip.noise = r.read_u16()?;
        chip.echo_addr = r.read_u16()?;
        chip.echo_idx = r.read_u16()?;
        chip.echo_len = r.read_u16()?;
        if chip.cycle >= CLOCKS_PER_SAMPLE
            || chip.fir_pos > 7
            || chip.rate_counter > RATE_COUNTER_TOP
            || chip.noise > 0x7FFF
        {
            return Err(bad("a global counter is out of range"));
        }
        for s in chip.fir_l.iter_mut().chain(chip.fir_r.iter_mut()) {
            *s = r.read_i16()?;
        }
        for s in chip
            .fir_out
            .iter_mut()
            .chain(chip.main.iter_mut())
            .chain(chip.echo.iter_mut())
            .chain(chip.output.iter_mut())
        {
            *s = r.read_i16()?;
        }
        let coef = r.read_bytes()?;
        if coef.len() != 8 {
            return Err(bad("the FIR coefficients are the wrong size"));
        }
        chip.fir_coef.copy_from_slice(coef);
        for voice in &mut chip.voices {
            voice.srcn = r.read_u8()?;
            voice.adsr1 = r.read_u8()?;
            voice.adsr2 = r.read_u8()?;
            voice.pitch_lo = r.read_u8()?;
            voice.pitch_hi = r.read_u8()?;
            voice.brr_offset = r.read_u8()?;
            voice.header = r.read_u8()?;
            voice.data = [r.read_u8()?, r.read_u8()?];
            voice.ring_pos = r.read_u8()?;
            voice.mode = EnvMode::from_u8(r.read_u8()?).ok_or_else(|| bad("an envelope mode"))?;
            voice.kon_delay = r.read_u8()?;
            voice.loop_addr = r.read_u16()?;
            voice.brr_addr = r.read_u16()?;
            voice.counter = r.read_u16()?;
            voice.env = r.read_u16()?;
            voice.hidden_env = r.read_u16()?;
            for s in &mut voice.ring {
                *s = r.read_i16()?;
            }
            voice.out = r.read_i16()?;
            voice.kon_pending = r.read_bool()?;
            voice.end_seen = r.read_bool()?;
            voice.endx_clear = r.read_bool()?;
            if !matches!(voice.brr_offset, 1 | 3 | 5 | 7)
                || !matches!(voice.ring_pos, 0 | 4 | 8)
                || voice.kon_delay > 5
                || voice.counter >= 0x8000
                || voice.env > 0x7FF
                || voice.hidden_env > 0x7FF
            {
                return Err(bad("a voice counter is out of range"));
            }
        }
        *self = chip;
        Ok(())
    }
}

/// Decode the voice's latched group, and move on through the block —
/// following the loop address and setting ENDX after an end block, and
/// releasing with a zero envelope after one that does not loop (fullsnes,
/// *BRR Samples*, the Loop/End codes).
fn decode_into(voice: &mut Voice) {
    let pos = decode_group(
        voice.header,
        voice.data,
        &mut voice.ring,
        usize::from(voice.ring_pos),
    );
    voice.ring_pos = pos as u8;
    voice.brr_offset += 2;
    if voice.brr_offset >= 9 {
        voice.brr_offset = 1;
        if voice.header & 1 != 0 {
            voice.brr_addr = voice.loop_addr;
            voice.end_seen = true;
            if voice.header & 2 == 0 {
                voice.mode = EnvMode::Release;
                voice.env = 0;
                voice.hidden_env = 0;
            }
        } else {
            voice.brr_addr = voice.brr_addr.wrapping_add(9);
        }
    }
}

/// One sample's envelope update (fullsnes, *ADSR/Gain Envelope*; SNESdev wiki,
/// *DSP envelopes*).
fn step_envelope(voice: &mut Voice, counter: u16) {
    let env = i32::from(voice.env);
    if voice.mode == EnvMode::Release {
        let e = (env - 8).max(0);
        voice.env = e as u16;
        voice.hidden_env = e as u16;
        return;
    }
    let (a1, a2) = (voice.adsr1, voice.adsr2);
    let (rate, new) = if a1 & 0x80 != 0 {
        match voice.mode {
            EnvMode::Attack => {
                let rate = (a1 & 0x0F) * 2 + 1;
                (rate, env + if rate == 31 { 1024 } else { 32 })
            }
            EnvMode::Decay => (((a1 >> 4) & 7) * 2 + 16, exp_decrease(env)),
            EnvMode::Sustain | EnvMode::Release => (a2 & 0x1F, exp_decrease(env)),
        }
    } else if a2 & 0x80 == 0 {
        // Direct gain: a fixed level, every sample.
        (31, i32::from(a2 & 0x7F) * 16)
    } else {
        let new = match (a2 >> 5) & 3 {
            0 => env - 32,
            1 => exp_decrease(env),
            2 => env + 32,
            _ => {
                if voice.hidden_env < 0x600 {
                    env + 32
                } else {
                    env + 8
                }
            }
        };
        (a2 & 0x1F, new)
    };
    if rate_fires(rate, counter) {
        voice.hidden_env = (new & 0x7FF) as u16;
        voice.env = new.clamp(0, 0x7FF) as u16;
    }
    voice.mode = match voice.mode {
        EnvMode::Attack if voice.env >= 0x7E0 => EnvMode::Decay,
        EnvMode::Decay if (voice.env >> 8) as u8 == a2 >> 5 => EnvMode::Sustain,
        mode => mode,
    };
}

impl SDsp for SDspChip {
    fn read(&self, reg: u8) -> u8 {
        let reg = reg & 0x7F;
        if reg == reg::ENDX {
            self.endx
        } else {
            self.regs[usize::from(reg)]
        }
    }

    fn write(&mut self, reg: u8, value: u8) {
        let reg = reg & 0x7F;
        match reg {
            reg::ENDX => self.endx = 0,
            reg::KON => {
                self.regs[usize::from(reg)] = value;
                self.kon_latch = value;
            }
            _ => self.regs[usize::from(reg)] = value,
        }
    }

    fn clock(&mut self, aram: &mut dyn Aram) -> Option<(i16, i16)> {
        let t = self.cycle;
        for &job in &SCHEDULE[usize::from(t)] {
            if job == NONE {
                break;
            }
            self.voice_stage(usize::from(job >> 4), STAGES[usize::from(job & 0x0F)], aram);
        }
        self.global_stage(t, aram);
        self.cycle = (t + 1) % CLOCKS_PER_SAMPLE;
        (t == CLOCKS_PER_SAMPLE - 1).then(|| self.output())
    }

    fn reset(&mut self) {
        self.reset_state();
    }

    fn save(&self, w: &mut ChunkWriter<'_>) -> Result<()> {
        self.save_state(w)
    }

    fn load(&mut self, r: &mut ChunkReader<'_>) -> Result<()> {
        self.load_state(r)
    }
}

#[cfg(test)]
mod tests;
