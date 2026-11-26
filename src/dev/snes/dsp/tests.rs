//! S-DSP tests. Expected values are worked by hand from fullsnes's formulas
//! and the SNESdev wiki's tables, not taken from any other implementation.

use alloc::collections::BTreeSet;
use alloc::vec;
use alloc::vec::Vec;

use super::*;
use crate::core::state::{MachineShape, Migrations, StateReader, StateWriter};

/// 64 KiB of audio RAM that remembers which addresses were written.
struct Ram {
    bytes: Vec<u8>,
    written: BTreeSet<u16>,
}

impl Ram {
    fn new(fill: u8) -> Self {
        Ram {
            bytes: vec![fill; 0x10000],
            written: BTreeSet::new(),
        }
    }

    fn load(&mut self, addr: u16, data: &[u8]) {
        for (i, b) in data.iter().enumerate() {
            self.bytes[usize::from(addr) + i] = *b;
        }
    }
}

impl Aram for Ram {
    fn read(&self, addr: u16) -> u8 {
        self.bytes[usize::from(addr)]
    }

    fn write(&mut self, addr: u16, value: u8) {
        self.bytes[usize::from(addr)] = value;
        self.written.insert(addr);
    }
}

fn run(dsp: &mut SDspChip, ram: &mut Ram, samples: usize) -> Vec<(i16, i16)> {
    (0..samples).map(|_| dsp.run_sample(ram)).collect()
}

/// A chip with voice 1 set up on directory page 2, entry 0 → `$1000`, pitch
/// `$1000`, the given envelope registers, and echo writes off.
fn voice1(adsr1: u8, gain_or_adsr2: u8) -> (SDspChip, Ram) {
    let mut ram = Ram::new(0);
    ram.load(0x0200, &[0x00, 0x10, 0x00, 0x10]);
    let mut dsp = SDspChip::new();
    dsp.write(reg::DIR, 0x02);
    dsp.write(reg::FLG, FLG_ECHO_OFF);
    dsp.write(reg::voice(1, reg::V_SRCN), 0);
    dsp.write(reg::voice(1, reg::V_PITCHL), 0x00);
    dsp.write(reg::voice(1, reg::V_PITCHH), 0x10);
    dsp.write(reg::voice(1, reg::V_ADSR1), adsr1);
    dsp.write(reg::voice(1, reg::V_ADSR2), gain_or_adsr2);
    dsp.write(reg::voice(1, reg::V_GAIN), gain_or_adsr2);
    // DIR is latched at T29, so let one sample pass before keying on.
    run(&mut dsp, &mut ram, 1);
    // Re-align so the KON write lands at the start of a polling sample: the
    // poll fires on the samples where `kon_phase` turns true.
    if dsp.kon_phase {
        run(&mut dsp, &mut ram, 1);
    }
    (dsp, ram)
}

// ---------------------------------------------------------------------------
// BRR
// ---------------------------------------------------------------------------

#[test]
fn brr_filter_0_shifts_and_the_reserved_shifts() {
    let mut ring = [0i16; 12];
    // Shift 12: nibble 7 → (7 << 12) >> 1 = $3800; -8 → -$4000; 1 → $800.
    decode_group(0xC0, [0x78, 0x1F], &mut ring, 0);
    assert_eq!(&ring[..4], &[0x3800, -0x4000, 0x0800, -0x0800]);
    // Shift 13-15 act as 12 on nibble SAR 3: 7 → 0, -8 → -$800, -1 → -$800.
    decode_group(0xD0, [0x78, 0x0F], &mut ring, 4);
    assert_eq!(&ring[4..8], &[0, -0x0800, 0, -0x0800]);
    // Shift 0 strips the low bit: 1 → 0, -1 → -1 (SAR floors).
    decode_group(0x00, [0x1F, 0x00], &mut ring, 8);
    assert_eq!(&ring[8..12], &[0, -1, 0, 0]);
}

#[test]
fn brr_filters_1_to_3_use_the_exact_formulas() {
    // Filter 1, shift 4 (sample = n * 8), old = $1000.
    let mut ring = [0i16; 12];
    ring[11] = 0x1000;
    decode_group(0x44, [0x10, 0x00], &mut ring, 0);
    // 8 + 4096 + (-4096 SAR 4 = -256) = 3848; then 3848 + (-3848 SAR 4 = -241).
    assert_eq!(&ring[..2], &[3848, 3607]);

    // Filter 2, zero nibble, old = $1000, older = $800:
    // 8192 + (-12288 SAR 5 = -384) - 2048 + 128 = 5888.
    let mut ring = [0i16; 12];
    ring[10] = 0x0800;
    ring[11] = 0x1000;
    decode_group(0x48, [0x00, 0x00], &mut ring, 0);
    assert_eq!(ring[0], 5888);

    // Filter 3: 8192 + (-53248 SAR 6 = -832) - 2048 + (6144 SAR 4 = 384) = 5696.
    let mut ring = [0i16; 12];
    ring[10] = 0x0800;
    ring[11] = 0x1000;
    decode_group(0x4C, [0x00, 0x00], &mut ring, 0);
    assert_eq!(ring[0], 5696);
}

#[test]
fn brr_clamps_to_16_bits_then_wraps_to_15() {
    // Filter 1: $3800 + $3000 - $300 = $6500, in $4000-$7FFF → $6500 - $8000.
    let mut ring = [0i16; 12];
    ring[11] = 0x3000;
    decode_group(0xC4, [0x70, 0x00], &mut ring, 0);
    assert_eq!(ring[0], -0x1B00);

    // Filter 1: -$4000 - $4000 + $400 = -$7C00 → +$8000 = $400.
    let mut ring = [0i16; 12];
    ring[11] = -0x4000;
    decode_group(0xC4, [0x80, 0x00], &mut ring, 0);
    assert_eq!(ring[0], 0x0400);

    // Filter 2 past +$7FFF: 14336 + 32766 - 1536 + 16384 - 1024 = 60926,
    // clamped to $7FFF, which wraps to -1.
    let mut ring = [0i16; 12];
    ring[10] = -0x4000;
    ring[11] = 0x3FFF;
    decode_group(0xC8, [0x70, 0x00], &mut ring, 0);
    assert_eq!(ring[0], -1);

    // Filter 2 past -$8000: -16384 - 32768 + 1536 - 16383 + 1023 = -62976,
    // clamped to -$8000, which wraps to zero.
    let mut ring = [0i16; 12];
    ring[10] = 0x3FFF;
    ring[11] = -0x4000;
    decode_group(0xC8, [0x80, 0x00], &mut ring, 0);
    assert_eq!(ring[0], 0);
}

// ---------------------------------------------------------------------------
// Gaussian interpolation
// ---------------------------------------------------------------------------

#[test]
fn gauss_table_spot_values_and_fullsnes_sum_claim() {
    assert_eq!(GAUSS[0x000], 0x000);
    assert_eq!(GAUSS[0x010], 0x001);
    assert_eq!(GAUSS[0x0FF], 0x172);
    assert_eq!(GAUSS[0x100], 0x176);
    assert_eq!(GAUSS[0x180], 0x3C9);
    assert_eq!(GAUSS[0x1FF], 0x519);
    // "each four values ... do sum up to 7FFh..801h".
    for i in 0..256 {
        let sum = GAUSS[i] + GAUSS[0xFF - i] + GAUSS[0x100 + i] + GAUSS[0x1FF - i];
        assert!((0x7FF..=0x801).contains(&sum), "i={i:#x}: {sum:#x}");
    }
}

#[test]
fn gauss_overflow_on_three_full_negative_samples() {
    // i = 0 weighs $172 + $519 + $176 = $801 on the three oldest: -16 × $801
    // = -$8010 wraps to +$7FF0, and SAR 1 gives fullsnes's +$3FF8.
    assert_eq!(
        gauss_interpolate(-0x4000, -0x4000, -0x4000, -0x4000, 0),
        0x3FF8
    );
    // A constant $3800 at i = 0: (5180 + 18270 + 5236) SAR 1.
    assert_eq!(gauss_interpolate(0x3800, 0x3800, 0x3800, 0x3800, 0), 14343);
    // The last term saturates: at i = $FF the weights on $3FFF give
    // 0 + 5983 + 20878 = 26861, plus 5919 = 32780, clamped to $7FFF.
    assert_eq!(
        gauss_interpolate(0x3FFF, 0x3FFF, 0x3FFF, 0x3FFF, 0xFF),
        0x3FFF
    );
}

// ---------------------------------------------------------------------------
// Pitch, noise, rates
// ---------------------------------------------------------------------------

#[test]
fn pitch_modulation_scales_by_the_previous_voice() {
    assert_eq!(pitch_step(0x1000, None), 0x1000);
    assert_eq!(pitch_step(0xFFFF, None), 0x3FFF, "bits 14-15 are ignored");
    assert_eq!(pitch_step(0x1000, Some(0)), 0x1000);
    // factor = ($3FFF SAR 4) + $400 = $7FF: $1000 × $7FF SAR 10 = $1FFC.
    assert_eq!(pitch_step(0x1000, Some(0x3FFF)), 0x1FFC);
    assert_eq!(pitch_step(0x1000, Some(-0x4000)), 0, "factor zero");
    assert_eq!(pitch_step(0x3FFF, Some(0x3FFF)), 0x3FFF, "capped");
}

#[test]
fn noise_lfsr_sequence() {
    let mut n = 0x4000;
    let mut seq = Vec::new();
    for _ in 0..16 {
        n = noise_step(n);
        seq.push(n);
    }
    // Shifts down until bits 1 and 0 differ, then feeds a one into bit 14.
    assert_eq!(&seq[..3], &[0x2000, 0x1000, 0x0800]);
    assert_eq!(seq[12], 0x0002);
    assert_eq!(seq[13], 0x4001, "0b10: feedback 1");
    assert_eq!(seq[14], 0x6000, "0b01: feedback 1");
    assert_eq!(seq[15], 0x3000);
    // Taps at bits 0 and 1 of a 15-bit register: maximal length.
    let mut n = 0x4000;
    let mut period = 0u32;
    loop {
        n = noise_step(n);
        period += 1;
        if n == 0x4000 {
            break;
        }
    }
    assert_eq!(period, 0x7FFF);
}

#[test]
fn rate_table_periods_and_offsets() {
    assert!((0..0x7800).all(|c| !rate_fires(0, c)), "rate 0 never");
    assert!(
        (0..0x7800).all(|c| rate_fires(31, c)),
        "rate 31 every sample"
    );
    // Rate 1: period 2048, offset 0.
    assert!(rate_fires(1, 0) && rate_fires(1, 2048) && !rate_fires(1, 1));
    // Rate 2: period 1536, offset 1040 → fires at 496.
    assert!(rate_fires(2, 496) && !rate_fires(2, 0));
    // Rate 3: period 1280, offset 536 → fires at 744.
    assert!(rate_fires(3, 744) && !rate_fires(3, 0));
    // Every period divides the counter's $7800, so each rate's step count
    // over a full counter cycle is exact.
    for rate in 1..32u8 {
        let n = (0..0x7800).filter(|&c| rate_fires(rate, c)).count();
        assert_eq!(n, 0x7800 / usize::from(RATE_PERIOD[usize::from(rate)]));
    }
}

// ---------------------------------------------------------------------------
// Envelopes and KON
// ---------------------------------------------------------------------------

#[test]
fn kon_waits_five_empty_samples() {
    let (mut dsp, mut ram) = voice1(0x00, 0x7F); // direct gain $7F → $7F0
    dsp.write(reg::KON, 0x02);
    let envx = reg::voice(1, reg::V_ENVX);
    for s in 0..6 {
        dsp.run_sample(&mut ram);
        assert_eq!(dsp.read(envx), 0, "sample {s}");
    }
    dsp.run_sample(&mut ram);
    assert_eq!(dsp.envelope(1), 0x7F0);
    assert_eq!(dsp.read(envx), 0x7F);
}

#[test]
fn linear_gain_steps_by_32_at_its_rate() {
    let (mut dsp, mut ram) = voice1(0x00, 0xDF); // linear increase, rate 31
    dsp.write(reg::KON, 0x02);
    run(&mut dsp, &mut ram, 7);
    assert_eq!(dsp.envelope(1), 32);
    run(&mut dsp, &mut ram, 1);
    assert_eq!(dsp.envelope(1), 64);

    // Rate 30: period 2 → one step every other sample.
    let (mut dsp, mut ram) = voice1(0x00, 0xDE);
    dsp.write(reg::KON, 0x02);
    run(&mut dsp, &mut ram, 6);
    let mut seen = Vec::new();
    for _ in 0..6 {
        dsp.run_sample(&mut ram);
        seen.push(dsp.envelope(1));
    }
    let steps = seen.windows(2).filter(|w| w[1] != w[0]).count();
    assert_eq!(seen.last().copied().unwrap() - seen[0], 32 * steps as u16);
    assert!((2..=3).contains(&steps), "{seen:?}");
}

#[test]
fn adsr_attack_15_then_exponential_decay() {
    // A = 15 (+1024 every sample), D = 7 (rate 30), SL = 0, SR = 0.
    let (mut dsp, mut ram) = voice1(0xFF, 0x00);
    dsp.write(reg::KON, 0x02);
    run(&mut dsp, &mut ram, 7);
    assert_eq!(dsp.envelope(1), 1024);
    run(&mut dsp, &mut ram, 1);
    assert_eq!(dsp.envelope(1), 0x7FF, "clamped, and now decaying");
    assert_eq!(dsp.voices[1].mode, EnvMode::Decay);
    // The first decay step: $7FF - ((($7FE) SAR 8) + 1) = $7F7.
    let mut env = dsp.envelope(1);
    while env == 0x7FF {
        dsp.run_sample(&mut ram);
        env = dsp.envelope(1);
    }
    assert_eq!(env, 0x7F7);
}

#[test]
fn koff_releases_by_8_per_sample() {
    let (mut dsp, mut ram) = voice1(0x00, 0x7F);
    dsp.write(reg::KON, 0x02);
    run(&mut dsp, &mut ram, 8);
    assert_eq!(dsp.envelope(1), 0x7F0);
    dsp.write(reg::KOFF, 0x02);
    // Wait for the poll to see it.
    let mut n = 0;
    while dsp.voices[1].mode != EnvMode::Release {
        dsp.run_sample(&mut ram);
        n += 1;
        assert!(n <= 2, "KOFF is polled every other sample");
    }
    let start = dsp.envelope(1);
    run(&mut dsp, &mut ram, 1);
    assert_eq!(dsp.envelope(1), start - 8);
    run(&mut dsp, &mut ram, 1);
    assert_eq!(dsp.envelope(1), start - 16);
}

#[test]
fn kon_under_koff_produces_nothing() {
    let (mut dsp, mut ram) = voice1(0x00, 0x7F);
    dsp.write(reg::KOFF, 0x02);
    dsp.write(reg::KON, 0x02);
    for _ in 0..12 {
        dsp.run_sample(&mut ram);
        assert_eq!(dsp.envelope(1), 0);
    }
}

#[test]
fn soft_reset_zeroes_envelopes() {
    let (mut dsp, mut ram) = voice1(0x00, 0x7F);
    dsp.write(reg::KON, 0x02);
    run(&mut dsp, &mut ram, 8);
    assert_eq!(dsp.envelope(1), 0x7F0);
    dsp.write(reg::FLG, FLG_RESET | FLG_ECHO_OFF);
    run(&mut dsp, &mut ram, 1);
    assert_eq!(dsp.envelope(1), 0);
    assert_eq!(dsp.voices[1].mode, EnvMode::Release);
}

// ---------------------------------------------------------------------------
// End flags, mixing, registers
// ---------------------------------------------------------------------------

#[test]
fn endx_sets_when_the_end_block_finishes_and_mutes() {
    let (mut dsp, mut ram) = voice1(0x00, 0x7F);
    // One block, code 1 (end, no loop), shift 0.
    ram.load(0x1000, &[0x01, 0, 0, 0, 0, 0, 0, 0, 0]);
    dsp.write(reg::KON, 0x02);
    // Three groups during the delay, then the fourth after $4000 of pitch at
    // $1000 a sample: decoded in sample 10.
    run(&mut dsp, &mut ram, 10);
    assert_eq!(dsp.read(reg::ENDX), 0, "clear right after key-on");
    assert_eq!(dsp.envelope(1), 0x7F0);
    run(&mut dsp, &mut ram, 1);
    assert_eq!(dsp.read(reg::ENDX), 0x02);
    assert_eq!(
        dsp.envelope(1),
        0,
        "end without loop: release, envelope zero"
    );
    assert_eq!(dsp.voices[1].mode, EnvMode::Release);
    dsp.write(reg::ENDX, 0x55);
    assert_eq!(dsp.read(reg::ENDX), 0, "any write clears every bit");
}

#[test]
fn a_constant_voice_through_the_whole_mixer() {
    let (mut dsp, mut ram) = voice1(0x00, 0x7F);
    let mut block = vec![0xC3]; // shift 12, filter 0, end + loop
    block.extend_from_slice(&[0x77; 8]);
    ram.load(0x1000, &block);
    for r in [
        reg::voice(1, reg::V_VOLL),
        reg::voice(1, reg::V_VOLR),
        reg::MVOLL,
        reg::MVOLR,
    ] {
        dsp.write(r, 0x7F);
    }
    dsp.write(reg::KON, 0x02);
    let out = run(&mut dsp, &mut ram, 7);
    // Interpolated $3800 → 14343; × $7F0 SAR 11 → 14230; × 127 SAR 6 → 28237;
    // × 127 SAR 7 → 28016.
    assert_eq!(dsp.voice_output(1), 14230);
    assert_eq!(
        dsp.read(reg::voice(1, reg::V_OUTX)),
        14230u16.wrapping_shr(7) as u8
    );
    assert_eq!(out[6], (28016, 28016));
    assert_eq!(out[5], (0, 0), "still in the KON delay");
    // Looping: it keeps going.
    let later = run(&mut dsp, &mut ram, 40);
    assert!(later.iter().all(|&s| s == (28016, 28016)));
    assert_eq!(dsp.read(reg::ENDX) & 0x02, 0x02, "the loop block sets ENDX");
    // Mute zeroes the output without stopping anything.
    dsp.write(reg::FLG, FLG_MUTE | FLG_ECHO_OFF);
    assert_eq!(dsp.run_sample(&mut ram), (0, 0));
    assert_eq!(dsp.voice_output(1), 14230);
}

#[test]
fn register_read_back() {
    let mut dsp = SDspChip::new();
    let mut ram = Ram::new(0);
    assert_eq!(dsp.read(reg::FLG), 0xE0, "reset value");
    // Unused registers are plain RAM.
    for r in [0x0A, 0x1B, 0x2E, 0x1D] {
        dsp.write(r, 0x5A);
        assert_eq!(dsp.read(r), 0x5A);
    }
    // ENVX and OUTX are writable but the DSP overwrites them each sample.
    dsp.write(reg::voice(3, reg::V_ENVX), 0x33);
    dsp.write(reg::voice(3, reg::V_OUTX), 0x44);
    assert_eq!(dsp.read(reg::voice(3, reg::V_ENVX)), 0x33);
    dsp.run_sample(&mut ram);
    assert_eq!(dsp.read(reg::voice(3, reg::V_ENVX)), 0);
    assert_eq!(dsp.read(reg::voice(3, reg::V_OUTX)), 0);
    // KON reads back what was written after the poll consumed it.
    dsp.write(reg::KON, 0x80);
    run(&mut dsp, &mut ram, 2);
    assert_eq!(dsp.read(reg::KON), 0x80);
    assert_eq!(dsp.kon_latch, 0);
    // The address wraps at 7 bits.
    dsp.write(0x80 | reg::EFB, 0x12);
    assert_eq!(dsp.read(reg::EFB), 0x12);
}

// ---------------------------------------------------------------------------
// Echo
// ---------------------------------------------------------------------------

#[test]
fn echo_buffer_with_edl_zero_is_four_bytes() {
    let mut dsp = SDspChip::new();
    let mut ram = Ram::new(0xAA);
    dsp.write(reg::ESA, 0x20);
    dsp.write(reg::EDL, 0x00);
    dsp.write(reg::FLG, 0x00);
    run(&mut dsp, &mut ram, 2); // ESA takes a sample to latch
    ram.written.clear();
    run(&mut dsp, &mut ram, 50);
    let want: BTreeSet<u16> = (0x2000..0x2004).collect();
    assert_eq!(ram.written, want);
    assert_eq!(ram.bytes[0x2004], 0xAA);
}

#[test]
fn echo_buffer_walks_edl_times_2k_and_wraps() {
    let mut dsp = SDspChip::new();
    let mut ram = Ram::new(0xAA);
    dsp.write(reg::ESA, 0x20);
    dsp.write(reg::EDL, 0x01);
    dsp.write(reg::FLG, 0x00);
    run(&mut dsp, &mut ram, 1);
    ram.written.clear();
    run(&mut dsp, &mut ram, 600);
    let want: BTreeSet<u16> = (0x2000..0x2800).collect();
    assert_eq!(ram.written, want, "512 entries of 4 bytes");
}

#[test]
fn echo_write_disable_stops_writes_not_reads() {
    let mut dsp = SDspChip::new();
    let mut ram = Ram::new(0);
    // A full-scale left sample sitting in the buffer at $2000.
    ram.load(0x2000, &[0x00, 0x40, 0, 0]);
    dsp.write(reg::ESA, 0x20);
    dsp.write(reg::FLG, FLG_ECHO_OFF);
    dsp.write(reg::fir(7), 0x40); // newest tap, gain 1
    dsp.write(reg::EVOLL, 0x7F);
    run(&mut dsp, &mut ram, 3);
    assert!(ram.written.is_empty());
    // $4000 SAR 1 = $2000 through FIR7 = $2000, × 127 SAR 7 = $1FC0.
    assert_eq!(dsp.output().0, 0x1FC0);
}

#[test]
fn fir_wraps_seven_taps_and_saturates_the_eighth() {
    let buf = [0x3FFF; 8];
    // Seven × (16383 × 127 SAR 6 = 32510) = 227570, wrapped to 30962.
    assert_eq!(
        fir(&buf, 0, &[0x7F, 0x7F, 0x7F, 0x7F, 0x7F, 0x7F, 0x7F, 0]),
        30962
    );
    // 32510 + 32510 saturates.
    assert_eq!(fir(&buf, 0, &[0, 0, 0, 0, 0, 0, 0x7F, 0x7F]), 0x7FFF);
    // FIR0 is the oldest: an impulse at the newest position is not seen by it.
    let mut buf = [0i16; 8];
    buf[3] = 0x1000;
    assert_eq!(fir(&buf, 3, &[0x40, 0, 0, 0, 0, 0, 0, 0]), 0);
    assert_eq!(fir(&buf, 3, &[0, 0, 0, 0, 0, 0, 0, 0x40]), 0x1000);
    assert_eq!(
        fir(&buf, 2, &[0, 0, 0, 0, 0, 0, 0x40, 0]),
        0,
        "pos 2: 3 is the oldest"
    );
    assert_eq!(fir(&buf, 2, &[0x40, 0, 0, 0, 0, 0, 0, 0]), 0x1000);
}

// ---------------------------------------------------------------------------
// Schedule, determinism, snapshots
// ---------------------------------------------------------------------------

#[test]
fn every_voice_runs_every_stage_once_per_sample() {
    let mut count = [[0u8; 9]; 8];
    for slot in &SCHEDULE {
        for &job in slot {
            if job != NONE {
                count[usize::from(job >> 4)][usize::from(job & 0xF)] += 1;
            }
        }
    }
    assert!(count.iter().flatten().all(|&n| n == 1));
    // Spot checks against the fullsnes chart.
    assert_eq!(slot(7, Stage::VolR), 22);
    assert_eq!(slot(7, Stage::Outx), 24);
    assert_eq!(slot(2, Stage::Srcn), 0);
    assert_eq!(slot(0, Stage::Brr1), 26);
}

/// A chip doing a bit of everything: two voices, pitch modulation, noise on a
/// third, echo with feedback.
fn exercised() -> (SDspChip, Ram) {
    let (mut dsp, mut ram) = voice1(0x8E, 0x34);
    let mut block = vec![0xB7]; // shift 11, filter 1, loop + end
    block.extend_from_slice(&[0x7F, 0x80, 0x19, 0xE2, 0x5A, 0xA5, 0x3C, 0xC3]);
    ram.load(0x1000, &block);
    for (r, v) in [
        (reg::voice(1, reg::V_VOLL), 0x60),
        (reg::voice(1, reg::V_VOLR), 0xA0),
        (reg::voice(2, reg::V_VOLL), 0x40),
        (reg::voice(2, reg::V_VOLR), 0x40),
        (reg::voice(2, reg::V_PITCHH), 0x08),
        (reg::voice(2, reg::V_GAIN), 0xC8),
        (reg::voice(3, reg::V_VOLL), 0x20),
        (reg::voice(3, reg::V_GAIN), 0x50),
        (reg::MVOLL, 0x7F),
        (reg::MVOLR, 0x70),
        (reg::EVOLL, 0x30),
        (reg::EVOLR, 0xD0),
        (reg::EFB, 0x50),
        (reg::fir(0), 0x7F),
        (reg::PMON, 0x04),
        (reg::NON, 0x08),
        (reg::EON, 0x06),
        (reg::ESA, 0x40),
        (reg::EDL, 0x02),
        (reg::FLG, 0x1C),
        (reg::KON, 0x0E),
    ] {
        dsp.write(r, v);
    }
    run(&mut dsp, &mut ram, 300);
    (dsp, ram)
}

fn hash(bytes: &[u8]) -> u64 {
    let mut h: u64 = 0xcbf2_9ce4_8422_2325;
    for b in bytes {
        h ^= u64::from(*b);
        h = h.wrapping_mul(0x0000_0100_0000_01b3);
    }
    h
}

fn snapshot(dsp: &SDspChip) -> Vec<u8> {
    let mut shape = MachineShape::new();
    shape.add_device("dsp", "snes.dsp").unwrap();
    let mut writer = StateWriter::new(shape);
    {
        let mut chunk = writer.chunk("dsp", "snes.dsp", 1).unwrap();
        SDsp::save(dsp, &mut chunk).unwrap();
    }
    writer.to_vec().unwrap()
}

#[test]
fn deterministic_state_hash() {
    let (a, ram_a) = exercised();
    let (b, ram_b) = exercised();
    assert_eq!(hash(&snapshot(&a)), hash(&snapshot(&b)));
    assert_eq!(hash(&ram_a.bytes), hash(&ram_b.bytes));
    assert!(
        a.read(reg::voice(1, reg::V_ENVX)) != 0,
        "the test is exercising something"
    );
}

#[test]
fn save_load_round_trip() {
    let (mut a, mut ram_a) = exercised();
    let bytes = snapshot(&a);
    let reader = StateReader::new(&bytes).unwrap();
    let chunk = reader
        .load("dsp", "snes.dsp", 1, &Migrations::new())
        .unwrap();
    let mut b = SDspChip::new();
    let mut r = chunk.reader();
    SDsp::load(&mut b, &mut r).unwrap();
    r.end().unwrap();
    assert_eq!(a, b);
    assert_eq!(hash(&snapshot(&b)), hash(&bytes));
    // And they stay together.
    let mut ram_b = Ram::new(0);
    ram_b.bytes.copy_from_slice(&ram_a.bytes);
    // Stop mid-sample too.
    for _ in 0..13 {
        a.clock(&mut ram_a);
        b.clock(&mut ram_b);
    }
    assert_eq!(run(&mut a, &mut ram_a, 200), run(&mut b, &mut ram_b, 200));
    assert_eq!(a, b);
}

#[test]
fn load_rejects_garbage() {
    let (a, _) = exercised();
    let bytes = snapshot(&a);
    let reader = StateReader::new(&bytes).unwrap();
    let chunk = reader
        .load("dsp", "snes.dsp", 1, &Migrations::new())
        .unwrap();
    let mut data = chunk.data().to_vec();
    // The slot byte sits right after the 8-byte length and 128 registers,
    // then ENDX and the KON latch.
    data[8 + 128 + 2] = 40;
    let mut b = SDspChip::new();
    assert!(b.load_state(&mut ChunkReader::new(&data)).is_err());
    assert_eq!(b, SDspChip::new(), "unchanged on failure");
    assert!(b.load_state(&mut ChunkReader::new(&data[..10])).is_err());
}

#[test]
fn reset_silences_and_rewinds() {
    let (mut a, _) = exercised();
    a.reset();
    assert_eq!(a.read(reg::FLG), 0xE0);
    assert_eq!(a.read(reg::ENDX), 0);
    assert_eq!(a.slot(), 0);
    assert!((0..8).all(|v| a.envelope(v) == 0));
    assert_eq!(a.read(reg::MVOLL), 0x7F, "the register array survives");
}
