//! Paula's audio DMA handshake with Agnus, on the whole A500 board.
//!
//! The hardware's invariant is simple: Agnus has one audio slot per channel
//! per scanline (*Amiga Hardware Reference Manual*, 3rd ed., chapter 6, the
//! DMA time-slot allocation), and a channel's minimum period is 124 colour
//! clocks (chapter 5), so a word — two samples, at least 248 colour clocks —
//! always lasts longer than a line. A channel therefore **never has two words
//! outstanding**, and the words played in a block are exactly the `AUDxLEN`
//! words in chip RAM, in order, once each.
//!
//! What broke it was not either chip but the scheduler between them: a round
//! that ended on Agnus's own next line could leave the colour-clock domain a
//! fraction of a tick short, and that line's event then went undelivered until
//! the next quantum boundary, a millisecond on. Paula, caught up first, ran as
//! far as fifteen lines past slots Agnus had not served yet. `AUDxDR` is one
//! flag rather than a count, so a fetch was lost and a word played twice; and
//! because the length counter counts word *boundaries*, the block restarted
//! early. About a fifth of the words of a steady tone were repeats.
//! `Scheduler::sync_lazy_devices` has the fix.
//!
//! The guest here is a hand-assembled 68000 program (MC68000 user's manual
//! instruction formats; register offsets from the manual's Appendix B) that
//! plays a table of distinct samples on channel 0 and counts its own audio
//! interrupts. **No byte of any ROM** is involved. What is observed is Paula's
//! host stream (`Paula::take_audio`): the exact integral of the left mixer over
//! each 32 colour clocks, in which a sample shows as a run of frames at
//! `sample × 64`.

#![cfg(all(feature = "machine-amiga-a500", feature = "std"))]

use rsemu::core::clock::GlobalTime;
use rsemu::core::space::MemAttrs;
use rsemu::core::value::Width;
use rsemu::dev::amiga::paula::AUDIO_SAMPLE_DIVISOR;
use rsemu::host::audio::amiga::capture;
use rsemu::machine::{Machine, catalog};

/// Words in the block, `AUDxLEN`.
const LEN: usize = 8;

/// Where the sample table lives in chip RAM.
const TABLE: u32 = 0x1000;

/// Where the interrupt handler counts.
const COUNTER: u32 = 0x2000;

/// The ROM's own window, where the program runs: it takes the overlay away
/// from address zero, so it cannot run out of the overlay.
const ROM_BASE: u32 = 0x00F8_0000;

/// Full volume, so a frame wholly inside a sample reads `sample × 64`.
const VOLUME: i16 = 64;

/// The block's samples, high byte of each word first. All distinct and none
/// zero, so a repeated word, a skipped one and an early restart are each
/// visible, and silence is not mistaken for any of them.
fn samples() -> [i8; 2 * LEN] {
    core::array::from_fn(|k| (7 * (k + 1)) as i8)
}

/// The block as the words `AUDxLEN` counts.
fn words() -> [u16; LEN] {
    let s = samples();
    core::array::from_fn(|i| u16::from_be_bytes([s[2 * i] as u8, s[2 * i + 1] as u8]))
}

/// `move.l #imm,(abs).l`.
fn move_l(code: &mut Vec<u16>, imm: u32, to: u32) {
    code.extend([
        0x23fc,
        (imm >> 16) as u16,
        imm as u16,
        (to >> 16) as u16,
        to as u16,
    ]);
}

/// `move.w #imm,(abs).l`.
fn move_w(code: &mut Vec<u16>, imm: u16, to: u32) {
    code.extend([0x33fc, imm, (to >> 16) as u16, to as u16]);
}

/// The program, at `$F8000C`:
///
/// ```text
///   move.b #$01,$bfe201        CIA-A DDRA: PA0 an output
///   move.b #$00,$bfe001        CIA-A PRA: OVL low, chip RAM at zero
///   move.l #handler,$70        the level 4 autovector (vector 28)
///   move.l #…,$1000 …          the sample table, LEN words
///   move.l #0,$2000            the interrupt counter
///   move.w #0,$dff0a0          AUD0LCH
///   move.w #$1000,$dff0a2      AUD0LCL
///   move.w #LEN,$dff0a4        AUD0LEN
///   move.w #per,$dff0a6        AUD0PER
///   move.w #64,$dff0a8         AUD0VOL
///   move.w #$7fff,$dff09c      INTREQ: clear everything
///   move.w #$c080,$dff09a      INTENA: SET, INTEN, AUD0
///   move.w #$2000,sr           supervisor, interrupt mask 0
///   move.w #$8201,$dff096      DMACON: SET, DMAEN, AUD0EN
///   bra    *
/// handler:
///   addq.l #1,$2000
///   move.w #$0080,$dff09c      INTREQ: clear AUD0
///   rte
/// ```
///
/// Audio is level 4 and `AUD0` is `INTREQ` bit 7 (chapter 7, *Interrupts*).
fn rom(per: u16) -> Vec<u8> {
    let mut code: Vec<u16> = vec![
        0x13fc, 0x0001, 0x00bf, 0xe201, // move.b #$01,$bfe201
        0x13fc, 0x0000, 0x00bf, 0xe001, // move.b #$00,$bfe001
    ];
    // The handler's address is patched in once the main program's length is
    // known.
    let vector_at = code.len() + 1;
    move_l(&mut code, 0, 0x70);
    let w = words();
    for (i, pair) in w.chunks(2).enumerate() {
        let long = (u32::from(pair[0]) << 16) | u32::from(pair[1]);
        move_l(&mut code, long, TABLE + 4 * i as u32);
    }
    move_l(&mut code, 0, COUNTER);
    move_w(&mut code, 0, 0xdff0a0);
    move_w(&mut code, TABLE as u16, 0xdff0a2);
    move_w(&mut code, LEN as u16, 0xdff0a4);
    move_w(&mut code, per, 0xdff0a6);
    move_w(&mut code, VOLUME as u16, 0xdff0a8);
    move_w(&mut code, 0x7fff, 0xdff09c);
    move_w(&mut code, 0xc080, 0xdff09a);
    code.extend([0x46fc, 0x2000]); // move.w #$2000,sr
    move_w(&mut code, 0x8201, 0xdff096);
    code.push(0x60fe); // bra *
    let handler = ROM_BASE + 0x0c + 2 * code.len() as u32;
    code[vector_at] = (handler >> 16) as u16;
    code[vector_at + 1] = handler as u16;
    code.extend([0x52b9, (COUNTER >> 16) as u16, COUNTER as u16]); // addq.l #1,$2000
    move_w(&mut code, 0x0080, 0xdff09c);
    code.push(0x4e73); // rte

    let mut image = vec![0u8; 512 * 1024];
    image[0..4].copy_from_slice(&0x0008_0000u32.to_be_bytes()); // SSP: top of chip RAM
    image[4..8].copy_from_slice(&(ROM_BASE + 0x0c).to_be_bytes()); // PC: the ROM's window
    for (i, word) in code.iter().enumerate() {
        let at = 0x0c + 2 * i;
        image[at..at + 2].copy_from_slice(&word.to_be_bytes());
    }
    image
}

/// Run the tone for `millis` of virtual time and return the left channel's
/// frames and the guest's interrupt count.
fn play(per: u16, millis: u64) -> (Vec<i16>, u32) {
    let entry = catalog::machine("amiga-a500").expect("this build ships amiga-a500");
    let mut options = catalog::build_options().expect("the catalog agrees with itself");
    options.realize.media.insert("kickstart", rom(per));
    options.realize.media.insert("df0", Vec::new());
    options.realize.media.insert("ext", Vec::new());
    capture::install(&mut options).expect("a capture table");
    let registry = catalog::registry().expect("a registry");
    let mut machine: Machine =
        rsemu::machine::build("amiga-a500", entry.source, &registry, &options)
            .unwrap_or_else(|e| panic!("the board does not realize: {e}"));
    let audio = capture::take(&options.realize.hosts, &machine).expect("a Paula");
    // Drained every 100 ms: the ring keeps a third of a second
    // (`AUDIO_RING_FRAMES`), and draining it changes nothing in the machine.
    let mut frames = Vec::new();
    for _ in 0..millis / 100 {
        machine
            .run_for(GlobalTime::from_nanos(100_000_000))
            .expect("it runs");
        frames.extend(audio.paula().take_audio());
    }
    assert_eq!(audio.paula().audio_dropped(), 0);
    assert!(
        frames.iter().all(|&(_, right)| right == 0),
        "only channel 0 plays, and it is on the left"
    );
    let space = machine.space("mem").expect("the memory space");
    let count = space
        .read(u64::from(COUNTER), Width::U32, MemAttrs::DEFAULT)
        .expect("chip RAM") as u32;
    (frames.into_iter().map(|(left, _)| left).collect(), count)
}

/// The samples the left output played, in order, read off the frames.
///
/// A frame wholly inside a sample is exactly `sample × 64`; a frame a
/// boundary falls inside is a weighted mix of two and belongs to neither. A
/// sample lasts at least 124 colour clocks, so it always has two whole frames
/// in a row, and a run of fewer is a boundary. Silence — before the first word
/// arrives — is dropped.
fn played(frames: &[i16]) -> Vec<i8> {
    let mut out: Vec<i8> = Vec::new();
    let mut i = 0;
    while i < frames.len() {
        let v = frames[i];
        let mut j = i;
        while j < frames.len() && frames[j] == v {
            j += 1;
        }
        if j - i >= 2 && v != 0 && v % VOLUME == 0 {
            out.push((v / VOLUME) as i8);
        }
        i = j;
    }
    out
}

/// Assert that `seq` is the block, over and over, from its first sample.
/// Returns how many times the block's last word began to play.
fn assert_blocks(seq: &[i8], per: u16) -> usize {
    let s = samples();
    let start = seq
        .iter()
        .position(|&v| v == s[0])
        .unwrap_or_else(|| panic!("period {per}: the block's first sample never played"));
    // The last sample may be cut off by the end of the run; everything before
    // it is whole.
    let body = &seq[start..seq.len() - 1];
    for (n, &v) in body.iter().enumerate() {
        assert_eq!(
            v,
            s[n % s.len()],
            "period {per}: sample {n} of the stream is {v}, the table's is {} — a word was \
             repeated, skipped, or the block restarted early",
            s[n % s.len()]
        );
    }
    body.len()
}

/// The words delivered are exactly the words in chip RAM, in order, once
/// each, block after block; and the guest takes one audio interrupt per block.
///
/// The period is long enough that the first slot always comes before the first
/// word boundary (a PAL line is 227 colour clocks), so the stream is exact
/// from the very first word.
#[test]
fn every_word_of_every_block_plays_once_in_order() {
    let per = 300;
    let (frames, interrupts) = play(per, 1000);
    // A 1 000 ms run at 3 546 895 colour clocks a second, 32 to a frame.
    assert!(frames.len() as u64 > 3_546_895 / AUDIO_SAMPLE_DIVISOR * 9 / 10);
    let seq = played(&frames);
    let n = assert_blocks(&seq, per);
    // 2 × LEN samples a block, each `per` colour clocks: about 740 blocks.
    let blocks = n / (2 * LEN);
    assert!(blocks > 700, "{blocks} blocks");
    // One interrupt on starting and one each time the last word of a block is
    // taken (chapter 5): so one per block begun, give or take the block the run
    // ends inside.
    let begun = 1 + (n + 1) / (2 * LEN);
    assert!(
        (begun.saturating_sub(1)..=begun + 1).contains(&(interrupts as usize)),
        "{interrupts} audio interrupts for {begun} blocks — the length counter is not counting \
         AUDxLEN words a block"
    );
}

/// The same at the manual's minimum period, 124 colour clocks, where a word
/// (248) outlasts a line (227) by only 21 and a slot served one line late is
/// already a second word boundary.
///
/// Checked from the first whole block on: what the channel does before its
/// very first word arrives is Figure 5-8's business, and at a period shorter
/// than a line that word can arrive after the first boundary. That is a
/// question about starting, not about the handshake, and it is not asserted
/// here (`docs/platforms/amiga.md`).
#[test]
fn at_the_minimum_period_no_word_is_repeated_either() {
    let per = 124;
    let (frames, _) = play(per, 500);
    let seq = played(&frames);
    let s = samples();
    let second = seq
        .windows(2)
        .position(|w| w[0] == s[2 * LEN - 1] && w[1] == s[0])
        .map(|i| i + 1)
        .expect("a block restarted");
    let n = assert_blocks(&seq[second..], per);
    assert!(n / (2 * LEN) > 600, "{} blocks", n / (2 * LEN));
}
