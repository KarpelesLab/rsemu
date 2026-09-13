//! Paula's DMA slots and the interrupt requests Agnus raises in Paula.
//!
//! # Direction
//!
//! The disk and audio pointers — `DSKPTH`/`DSKPTL`, `AUDxLCH`/`AUDxLCL` — are
//! Agnus registers (Appendix B), and only Agnus reaches chip RAM. Paula knows
//! *when* a channel needs a word; Agnus does the transfer in its DMA slot. So
//! on each line Agnus asks Paula, through its [`PaulaPort`], and moves the words:
//!
//! * **disk read** — while Paula has an assembled word, store it at `DSKPT` and
//!   add two;
//! * **disk write** — while Paula wants a word, fetch it from `DSKPT`, add two,
//!   and hand it over;
//! * **audio** — for each channel whose `AUDxEN` is on, ask what it wants: a
//!   restart reloads the channel's pointer from `AUDxLC` ("used to
//!   automatically restart pointers, such as ... the audio sample counter
//!   (whenever the audio length count is finished)", Appendix B's legend), and
//!   a fetch reads a word at the pointer, adds two, and hands it over.
//!
//! That is the contract `amiga.paula` defines.
//!
//! # When
//!
//! Chapter 6, *Blitter Operations and System DMA*: "Disk DMA, audio DMA,
//! display DMA, and sprite DMA all have the highest priority level. Each of
//! these four devices is allocated a group of time slots during each
//! horizontal scan", three cycles for disk and four for audio. Figure 6-9,
//! which places them on the line, is a drawing the text copy of the manual does
//! not carry, so **the slot is served on arrival at count 0 of every line** —
//! an inference, and the same count the sprite fetch is served on.
//!
//! Every call carries `at`, the count of this chip's colour-clock domain the
//! transfer happens on; Paula counts in the same domain. A slot is an event
//! the scheduler hears about whenever `DMAEN` and a disk or audio enable are
//! on, because what Paula does with a word — a `DSKBLK` interrupt on the last
//! one — is visible to the processor.
//!
//! # Interrupts
//!
//! Two of Paula's `INTREQ` bits are raised by things only Agnus sees (chapter 7,
//! *Interrupts*): `VERTB`, "at line 0 (start of vertical blank)", and `BLIT`,
//! "blitter finished". Agnus calls [`PaulaPort::request`] with each on the count
//! it happens.

use crate::dev::amiga::dma::{ChipDma, DmaChannel};
use crate::dev::amiga::paula::PaulaPort;

/// The `DMACON` bits whose channels have a slot served by [`serve`].
pub const SLOT_CHANNELS: u16 = DmaChannel::DISK.0
    | DmaChannel::AUD0.0
    | DmaChannel::AUD1.0
    | DmaChannel::AUD2.0
    | DmaChannel::AUD3.0;

/// A bound on how many words one slot moves in one direction, so that a sink
/// that always has another word cannot hold the chip in a loop. Chapter 6
/// gives the disk three cycles a line.
const WORDS_PER_SLOT: usize = 3;

/// Serve one line's disk and audio slots.
///
/// Chip RAM and the pointers are reached through [`ChipDma`], which takes no
/// blocking lock, so this runs with nothing of Agnus's held.
pub fn serve(paula: &PaulaPort, dma: &ChipDma, at: u64) {
    if dma.enabled(DmaChannel::DISK) {
        for _ in 0..WORDS_PER_SLOT {
            let Some(word) = paula.disk_read_word(at) else {
                break;
            };
            dma.disk_store(word);
        }
        for _ in 0..WORDS_PER_SLOT {
            if !paula.disk_write_wanted(at) {
                break;
            }
            let Some(word) = dma.disk_fetch() else {
                break;
            };
            paula.disk_write_word(at, word);
        }
    }
    for ch in 0..4 {
        if !dma.enabled(DmaChannel::audio(ch)) {
            continue;
        }
        let request = paula.audio_request(ch, at);
        if request.restart {
            dma.audio_restart(ch);
        }
        if request.fetch
            && let Some(word) = dma.audio_fetch(ch)
        {
            paula.audio_word(ch, at, word);
        }
    }
}
