#![no_main]
//! The AmigaDOS track decoder, on cells a guest wrote.
//!
//! `amiga.floppy` decodes a track back into sectors whenever a guest has
//! written it and the disk is a file (`--drive df0=disk.adf`), so
//! `dev::amiga::adf::scan_track` reads bytes nobody vetted: a sync word can be
//! anywhere, a header can claim any track and sector, and the track is a loop
//! whose sectors may straddle the index. The failure this looks for is the one
//! no unit test finds — an index computed from a header that runs off the end,
//! or a sector accepted that is not the sector that was written.
//!
//! What is asserted, beyond "it did not panic":
//!
//! * **An accepted sector is a real one.** `decode_track` only returns sectors
//!   whose header names this track and a sector below eleven.
//! * **One flipped cell never passes as different data.** A valid track,
//!   rotated so its write started anywhere, with one cell flipped: every
//!   sector that still decodes holds exactly the bytes that were encoded. An
//!   XOR checksum catches every single-bit error, and a flipped clock cell
//!   changes no data at all.
//!
//! # Input encoding
//!
//! ```text
//!   byte 0 = 0x00   the rest is a track's cells, verbatim (padded or cut to
//!                   one revolution)
//!   otherwise       byte 1 the track number, bytes 2-3 the rotation, bytes 4-6
//!                   which cell to flip, then sector data (repeated to fill)
//! ```

use libfuzzer_sys::fuzz_target;

use rsemu::dev::amiga::adf;
use rsemu::dev::amiga::floppy::{TRACK_BYTES, TRACK_CELLS};

fn byte(data: &[u8], at: usize) -> u8 {
    data.get(at).copied().unwrap_or(0)
}

fuzz_target!(|data: &[u8]| {
    let Some((&mode, rest)) = data.split_first() else {
        return;
    };
    if mode == 0 {
        let mut mfm = rest.to_vec();
        mfm.resize(TRACK_BYTES, 0);
        for track in [0u8, 1, 79, 159] {
            for (s, sector) in adf::decode_track(&mfm, track).iter().enumerate() {
                if let Some(sector) = sector {
                    assert_eq!(sector.len(), adf::SECTOR_BYTES, "sector {s}");
                }
            }
        }
        let _ = adf::scan_track(&mfm);
        return;
    }

    let track = byte(rest, 0) % 160;
    let rotation = usize::from(u16::from_le_bytes([byte(rest, 1), byte(rest, 2)])) % TRACK_BYTES;
    let flip = u32::from_le_bytes([byte(rest, 3), byte(rest, 4), byte(rest, 5), 0]) as u64
        % TRACK_CELLS;
    let seed = rest.get(6..).filter(|s| !s.is_empty()).unwrap_or(&[0x5a]);
    let sectors: Vec<u8> = seed.iter().copied().cycle().take(adf::TRACK_DATA).collect();

    let encoded = adf::encode_track(track, &sectors);
    let mut mfm: Vec<u8> = encoded[rotation..]
        .iter()
        .chain(&encoded[..rotation])
        .copied()
        .collect();
    mfm[(flip / 8) as usize] ^= 0x80 >> (flip % 8);

    let decoded = adf::decode_track(&mfm, track);
    let mut good = 0;
    for (s, sector) in decoded.iter().enumerate() {
        if let Some(sector) = sector {
            assert_eq!(
                &sector[..],
                &sectors[s * adf::SECTOR_BYTES..(s + 1) * adf::SECTOR_BYTES],
                "sector {s} decoded to bytes that were never written"
            );
            good += 1;
        }
    }
    assert!(good >= adf::SECTORS - 1, "one cell cost {} sectors", adf::SECTORS - good);
});
