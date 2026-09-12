//! The STM32 HASH processor.
//!
//! One class, `st.hash`: the block that turns a 512-bit message block into a
//! digest without the core touching a rotate. Firmware on a crypto-bearing
//! STM32 uses it for everything — a firmware-image check, an mbedTLS
//! accelerator shim, an HMAC over a provisioning blob — and a part without it
//! either faults on the register block or spins forever on `SR.BUSY`.
//!
//! | Offset | Register | What is in it |
//! | --- | --- | --- |
//! | `0x00` | `CR` | `INIT` 2, `DMAE` 3, `DATATYPE[5:4]`, `MODE` 6, `ALGO[0]` 7, `NBW[11:8]`, `DINNE` 12, `MDMAT` 13, `LKEY` 16, `ALGO[1]` 18 |
//! | `0x04` | `DIN` | the input FIFO, sixteen words deep |
//! | `0x08` | `STR` | `NBLW[4:0]`, `DCAL` 8 |
//! | `0x0c`–`0x1c` | `HR0`–`HR4` | the short digest window |
//! | `0x20` | `IMR` | `DINIE` 0, `DCIE` 1 |
//! | `0x24` | `SR` | `DINIS` 0, `DCIS` 1, `DMAS` 2, `BUSY` 3 |
//! | `0xf8`–`0x1cc` | `CSR0`–`CSR53` | the context swap |
//! | `0x310`–`0x32c` | `HR0`–`HR7` | the full digest window |
//!
//! # Where the compression functions come from
//!
//! From the standards, written in this file: **FIPS 180-4** §6.1.2 for SHA-1,
//! §6.2.2 for SHA-224/SHA-256, and **RFC 1321** §3.4 for MD5, with HMAC per
//! **RFC 2104** / FIPS 198-1. Not from `purecrypto`, which is a permitted
//! dependency and does have all four, and the reason is worth writing down
//! because it is a property of *this peripheral* rather than of that crate:
//!
//! * `CSR0..CSR53` is the running chaining state, exported and re-imported
//!   mid-message. A `Digest`-style `new`/`update`/`finalize` interface has no
//!   way to hand out its chaining variables or to be resumed from someone
//!   else's, so the context block would have to be fabricated — which is the
//!   one defect this peripheral exists not to have.
//! * `NBLW` counts **bits**, not bytes. FIPS 180-4's padding is defined over a
//!   bit string and firmware may legitimately write `NBLW = 5`; an
//!   `update(&[u8])` cannot express a five-bit message at all.
//! * HMAC here is three separate `DCAL` phases with a context swap allowed
//!   between any two of them, so the key schedule has to be drivable a block at
//!   a time from outside.
//!
//! So the hard half of what this block does is the half a hashing library does
//! not expose. The compression functions themselves are published arithmetic
//! with official test vectors, and those vectors — not self-consistency — are
//! what `tests.rs` asserts. `purecrypto` remains the right answer for a device
//! that wants an ordinary byte-oriented digest.
//!
//! # The four `DATATYPE` swaps are one rule
//!
//! RM0090 §25.3.3 draws four pictures; they are the same picture at four
//! widths. The word written to `DIN` is cut into units of 32, 16, 8 or 1 bits
//! and the units are **reversed**, after which the word is fed to the core
//! most-significant-bit first:
//!
//! | `DATATYPE` | unit | operation |
//! | --- | --- | --- |
//! | `00` | 32-bit | nothing — one unit reversed is itself |
//! | `01` | 16-bit | swap the halves (`rotate_left(16)`) |
//! | `10` | 8-bit | `swap_bytes` |
//! | `11` | 1 bit | `reverse_bits` |
//!
//! This is where driver bugs live, so it is worth checking against the case
//! everyone hits: a byte buffer `"abc"` read as a little-endian word is
//! `0x0063_6261`; in `DATATYPE = 10` the swap makes it `0x6162_6300`, whose top
//! twenty-four bits are `"abc"` — and `NBLW = 24` is exactly what ST's HAL
//! computes as `8 * (size % 4)`. Note that `01` swaps the *halves* and does not
//! byte-swap within them: the unit being reversed is the half-word, and a model
//! that byte-swapped inside each one would hash a 16-bit buffer backwards.
//!
//! # `NBLW` is measured from the top
//!
//! `NBLW = n` means the **most significant** `n` bits of the post-swap word are
//! message, and `NBLW = 0` means all thirty-two are. That falls out of the
//! `"abc"` case above and out of a driver writing a trailing partial group as a
//! whole over-read word: the junk is in the low bits, so the valid data has to
//! be in the high ones.
//!
//! # Why the FIFO is lazy, and what `DINIS`/`BUSY` mean here
//!
//! A block is *not* compressed when the sixteenth word lands. It is compressed
//! when a **seventeenth** word arrives, or when `DCAL` is written. RM0090
//! §25.3.2 describes it that way and the reason is `NBLW`: a sixty-three byte
//! message fills all sixteen words with the last one partial, and a core that
//! had already eaten the block could not be told afterwards to drop eight bits
//! of it. Drivers do hash sixty-three-byte buffers, so the hardware cannot be
//! eager, so neither is this.
//!
//! That fixes the two status bits. `DINIS` is set while `DIN` can take another
//! word (`NBW < 16`) and `BUSY` is set exactly when it cannot — the FIFO is
//! holding a complete block that the core has not taken yet, which is the only
//! state in this model that corresponds to "a block is being processed",
//! because the compression itself costs no virtual time. The pair is therefore
//! complementary, and neither can strand a guest: after `INIT` and after
//! `DCAL` the FIFO is empty, so a `while (SR & BUSY)` loop exits on its first
//! read.
//!
//! # The context block
//!
//! `CSR0..CSR53` is fifty-four words the reference manual declines to break
//! down — it says to save and restore them as a block and nothing else. So the
//! layout is this model's to define, and it is:
//!
//! | Word | Holds |
//! | --- | --- |
//! | `CSR0` | `ALGO`, `MODE`, `DATATYPE`, `LKEY`, the HMAC phase, `NBW`, the key length, `DCIS` |
//! | `CSR1`, `CSR2` | the message bit counter, low word first |
//! | `CSR3`–`CSR10` | the eight chaining variables |
//! | `CSR11`–`CSR26` | the sixteen FIFO words, already swapped |
//! | `CSR27`–`CSR42` | the sixty-four key bytes of an HMAC phase |
//! | `CSR43`–`CSR50` | the inner digest, between an HMAC's second and third `DCAL` |
//! | `CSR51`–`CSR53` | reserved, read as zero |
//!
//! Fifty-four words is enough because the compression buffer is **always
//! empty** at a point where a context can be taken: words reach the core only
//! sixteen at a time, so the bit counter is a multiple of 512 except inside a
//! `DCAL` that never returns to the guest mid-way. What is not in the block is
//! `CR`, `STR` and `IMR`, which a driver saves separately — that is what ST's
//! context-saving routine does, and it is why those three are not duplicated
//! here.
//!
//! Restoring is not advisory. Writing `CSRx` replaces the running state, so a
//! guest can suspend a digest half way through a message, run an unrelated one
//! on the same block, put the first one back and finish it — and get the
//! digest it would have got without the interruption. `tests.rs` asserts
//! precisely that, against the FIPS vector, and the snapshot round-trip
//! asserts it across `save`/`load` as well.
//!
//! # Which part
//!
//! `variant = "f4"` (the default) is RM0090 §25: **SHA-1 and MD5**, no
//! `ALGO[1]`, no `MDMAT`. `variant = "v2"` is RM0351 §30 / RM0432 §26 and adds
//! SHA-224 and SHA-256 — decoding `ALGO[1]` is the whole of the difference at
//! the register face, which is why it is a property here rather than a second
//! class.
//!
//! `no_std + alloc`, no `unsafe`, no dependencies.

use alloc::boxed::Box;
use alloc::format;
use alloc::string::String;
use alloc::sync::Arc;
use core::fmt;

use crate::core::device::{Device, DeviceClass, PropertySpec, RealizeCtx, ResetKind};
use crate::core::error::{BusError, Error, Result};
use crate::core::props::{Props, ValueKind};
use crate::core::space::{AccessConstraints, MemAttrs, MemOps, MemResult, Region, RegionRef};
use crate::core::state::{ChunkReader, ChunkWriter, Sink, Source};
use crate::core::sync::{LockRank, Mutex};
use crate::core::value::{Endian, Width};
use crate::core::wire::{Level, WireSource};
use crate::machine::realize::Instance;
use crate::machine::validate::{ClassSchema, PortDir, PropSchema};

/// The class name a `.machine` file asks for.
const CLASS_NAME: &str = "st.hash";

/// Bumped when the snapshot encoding below changes.
const STATE_VERSION: u32 = 1;

/// The decoded aperture: up to and including `HR7` at `0x32c`.
const REGISTER_BYTES: u64 = 0x330;

/// The pins this block drives.
pub mod pin {
    /// The interrupt request, shared with the RNG on an F4's vector 80.
    pub const IRQ: &str = "irq";
    /// The DMA request: high while `DMAE` is set and `DIN` has room.
    pub const DMA: &str = "dma";
}

// ---------------------------------------------------------------------------
// The register map (RM0090 §25.7, RM0432 §26.7)
// ---------------------------------------------------------------------------

const R_CR: u64 = 0x00;
const R_DIN: u64 = 0x04;
const R_STR: u64 = 0x08;
/// `HR0`..`HR4`, the window an SHA-1 or MD5 driver reads.
const R_HR_LOW: u64 = 0x0c;
const R_IMR: u64 = 0x20;
const R_SR: u64 = 0x24;
/// `CSR0`, the first of fifty-four context words.
const R_CSR: u64 = 0xf8;
/// `HR0`..`HR7`, the window a SHA-256 driver reads.
const R_HR_HIGH: u64 = 0x310;

/// How many context registers there are.
pub const CSR_COUNT: usize = 54;

const CR_INIT: u32 = 1 << 2;
const CR_DMAE: u32 = 1 << 3;
const CR_DATATYPE_SHIFT: u32 = 4;
const CR_DATATYPE_MASK: u32 = 0x3;
const CR_MODE: u32 = 1 << 6;
const CR_ALGO0: u32 = 1 << 7;
const CR_NBW_SHIFT: u32 = 8;
const CR_DINNE: u32 = 1 << 12;
const CR_MDMAT: u32 = 1 << 13;
const CR_LKEY: u32 = 1 << 16;
const CR_ALGO1: u32 = 1 << 18;

const STR_NBLW_MASK: u32 = 0x1f;
const STR_DCAL: u32 = 1 << 8;

const IMR_DINIE: u32 = 1 << 0;
const IMR_DCIE: u32 = 1 << 1;
const IMR_MASK: u32 = IMR_DINIE | IMR_DCIE;

const SR_DINIS: u32 = 1 << 0;
const SR_DCIS: u32 = 1 << 1;
const SR_DMAS: u32 = 1 << 2;
const SR_BUSY: u32 = 1 << 3;

/// The FIFO depth, in words — one 512-bit block.
const FIFO_WORDS: usize = 16;

/// The block size of every algorithm here, in bytes.
const BLOCK_BYTES: usize = 64;

// ---------------------------------------------------------------------------
// Algorithms
// ---------------------------------------------------------------------------

/// What `ALGO[1:0]` selects.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Algo {
    /// `00`: SHA-1, FIPS 180-4 §6.1.
    Sha1,
    /// `01`: MD5, RFC 1321.
    Md5,
    /// `10`: SHA-224, FIPS 180-4 §6.2. `variant = "v2"` only.
    Sha224,
    /// `11`: SHA-256, FIPS 180-4 §6.2. `variant = "v2"` only.
    Sha256,
}

impl Algo {
    /// Decode the two-bit `ALGO` field.
    #[must_use]
    pub const fn from_bits(bits: u32) -> Algo {
        match bits & 0x3 {
            0 => Algo::Sha1,
            1 => Algo::Md5,
            2 => Algo::Sha224,
            _ => Algo::Sha256,
        }
    }

    /// The two-bit `ALGO` field for this algorithm.
    #[must_use]
    pub const fn bits(self) -> u32 {
        match self {
            Algo::Sha1 => 0,
            Algo::Md5 => 1,
            Algo::Sha224 => 2,
            Algo::Sha256 => 3,
        }
    }

    /// Digest length, in bytes.
    #[must_use]
    pub const fn digest_len(self) -> usize {
        match self {
            Algo::Sha1 => 20,
            Algo::Md5 => 16,
            Algo::Sha224 => 28,
            Algo::Sha256 => 32,
        }
    }

    /// The initial chaining value.
    const fn iv(self) -> [u32; 8] {
        match self {
            Algo::Sha1 => [
                0x6745_2301,
                0xefcd_ab89,
                0x98ba_dcfe,
                0x1032_5476,
                0xc3d2_e1f0,
                0,
                0,
                0,
            ],
            Algo::Md5 => [
                0x6745_2301,
                0xefcd_ab89,
                0x98ba_dcfe,
                0x1032_5476,
                0,
                0,
                0,
                0,
            ],
            Algo::Sha224 => [
                0xc105_9ed8,
                0x367c_d507,
                0x3070_dd17,
                0xf70e_5939,
                0xffc0_0b31,
                0x6858_1511,
                0x64f9_8fa7,
                0xbefa_4fa4,
            ],
            Algo::Sha256 => [
                0x6a09_e667,
                0xbb67_ae85,
                0x3c6e_f372,
                0xa54f_f53a,
                0x510e_527f,
                0x9b05_688c,
                0x1f83_d9ab,
                0x5be0_cd19,
            ],
        }
    }
}

/// FIPS 180-4 §4.2.2: the sixty-four SHA-224/256 round constants.
#[rustfmt::skip]
const SHA256_K: [u32; 64] = [
    0x428a_2f98, 0x7137_4491, 0xb5c0_fbcf, 0xe9b5_dba5, 0x3956_c25b, 0x59f1_11f1, 0x923f_82a4, 0xab1c_5ed5,
    0xd807_aa98, 0x1283_5b01, 0x2431_85be, 0x550c_7dc3, 0x72be_5d74, 0x80de_b1fe, 0x9bdc_06a7, 0xc19b_f174,
    0xe49b_69c1, 0xefbe_4786, 0x0fc1_9dc6, 0x240c_a1cc, 0x2de9_2c6f, 0x4a74_84aa, 0x5cb0_a9dc, 0x76f9_88da,
    0x983e_5152, 0xa831_c66d, 0xb003_27c8, 0xbf59_7fc7, 0xc6e0_0bf3, 0xd5a7_9147, 0x06ca_6351, 0x1429_2967,
    0x27b7_0a85, 0x2e1b_2138, 0x4d2c_6dfc, 0x5338_0d13, 0x650a_7354, 0x766a_0abb, 0x81c2_c92e, 0x9272_2c85,
    0xa2bf_e8a1, 0xa81a_664b, 0xc24b_8b70, 0xc76c_51a3, 0xd192_e819, 0xd699_0624, 0xf40e_3585, 0x106a_a070,
    0x19a4_c116, 0x1e37_6c08, 0x2748_774c, 0x34b0_bcb5, 0x391c_0cb3, 0x4ed8_aa4a, 0x5b9c_ca4f, 0x682e_6ff3,
    0x748f_82ee, 0x78a5_636f, 0x84c8_7814, 0x8cc7_0208, 0x90be_fffa, 0xa450_6ceb, 0xbef9_a3f7, 0xc671_78f2,
];

/// RFC 1321 §3.4: `T[i] = floor(2^32 * abs(sin(i + 1)))`.
#[rustfmt::skip]
const MD5_T: [u32; 64] = [
    0xd76a_a478, 0xe8c7_b756, 0x2420_70db, 0xc1bd_ceee, 0xf57c_0faf, 0x4787_c62a, 0xa830_4613, 0xfd46_9501,
    0x6980_98d8, 0x8b44_f7af, 0xffff_5bb1, 0x895c_d7be, 0x6b90_1122, 0xfd98_7193, 0xa679_438e, 0x49b4_0821,
    0xf61e_2562, 0xc040_b340, 0x265e_5a51, 0xe9b6_c7aa, 0xd62f_105d, 0x0244_1453, 0xd8a1_e681, 0xe7d3_fbc8,
    0x21e1_cde6, 0xc337_07d6, 0xf4d5_0d87, 0x455a_14ed, 0xa9e3_e905, 0xfcef_a3f8, 0x676f_02d9, 0x8d2a_4c8a,
    0xfffa_3942, 0x8771_f681, 0x6d9d_6122, 0xfde5_380c, 0xa4be_ea44, 0x4bde_cfa9, 0xf6bb_4b60, 0xbebf_bc70,
    0x289b_7ec6, 0xeaa1_27fa, 0xd4ef_3085, 0x0488_1d05, 0xd9d4_d039, 0xe6db_99e5, 0x1fa2_7cf8, 0xc4ac_5665,
    0xf429_2244, 0x432a_ff97, 0xab94_23a7, 0xfc93_a039, 0x655b_59c3, 0x8f0c_cc92, 0xffef_f47d, 0x8584_5dd1,
    0x6fa8_7e4f, 0xfe2c_e6e0, 0xa301_4314, 0x4e08_11a1, 0xf753_7e82, 0xbd3a_f235, 0x2ad7_d2bb, 0xeb86_d391,
];

/// RFC 1321 §3.4: the per-round left-rotation amounts.
#[rustfmt::skip]
const MD5_S: [u32; 64] = [
    7, 12, 17, 22, 7, 12, 17, 22, 7, 12, 17, 22, 7, 12, 17, 22,
    5,  9, 14, 20, 5,  9, 14, 20, 5,  9, 14, 20, 5,  9, 14, 20,
    4, 11, 16, 23, 4, 11, 16, 23, 4, 11, 16, 23, 4, 11, 16, 23,
    6, 10, 15, 21, 6, 10, 15, 21, 6, 10, 15, 21, 6, 10, 15, 21,
];

// ---------------------------------------------------------------------------
// The engine
// ---------------------------------------------------------------------------

/// One Merkle-Damgård hash in progress: the chaining variables, the partial
/// block and the message bit counter.
///
/// Bit-granular on the way in, because `NBLW` is. `Copy` is deliberate: an
/// `Engine` is small, and taking a copy is how a `DCAL` finalizes without
/// destroying the state a context save would need.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct Engine {
    algo: Algo,
    h: [u32; 8],
    buf: [u8; BLOCK_BYTES],
    nbits: u64,
}

impl Engine {
    fn new(algo: Algo) -> Engine {
        Engine {
            algo,
            h: algo.iv(),
            buf: [0; BLOCK_BYTES],
            nbits: 0,
        }
    }

    /// Feed thirty-two bits, most significant first.
    ///
    /// The fast path writes four bytes; it is skipped when the bit counter is
    /// not byte-aligned or the write would straddle the block, both of which a
    /// hostile `CSR` restore can arrange.
    fn push_word(&mut self, w: u32) {
        let pos = ((self.nbits % 512) / 8) as usize;
        if self.nbits.is_multiple_of(8) && pos + 4 <= BLOCK_BYTES {
            self.buf[pos..pos + 4].copy_from_slice(&w.to_be_bytes());
            self.nbits = self.nbits.wrapping_add(32);
            if self.nbits.is_multiple_of(512) {
                self.compress();
            }
        } else {
            self.push_bits(w, 32);
        }
    }

    /// Feed the top `n` bits of `w`, most significant first.
    fn push_bits(&mut self, w: u32, n: u32) {
        for i in 0..n.min(32) {
            let pos = (self.nbits % 512) as usize;
            let mask = 0x80u8 >> (pos % 8);
            if (w >> (31 - i)) & 1 != 0 {
                self.buf[pos / 8] |= mask;
            } else {
                self.buf[pos / 8] &= !mask;
            }
            // Wrapping, not checked: a guest can put any bit counter it likes
            // into CSR1/CSR2, and the answer to one near 2^64 is a wrong digest
            // rather than a panic in a debug build.
            self.nbits = self.nbits.wrapping_add(1);
            if self.nbits.is_multiple_of(512) {
                self.compress();
            }
        }
    }

    fn push_bytes(&mut self, bytes: &[u8]) {
        let (words, rest) = bytes.as_chunks::<4>();
        for word in words {
            self.push_word(u32::from_be_bytes(*word));
        }
        for byte in rest {
            self.push_bits(u32::from(*byte) << 24, 8);
        }
    }

    /// Pad per the algorithm and return the digest, left-aligned in the array.
    ///
    /// Takes `self` by value so the caller's engine is untouched — an HMAC key
    /// phase finalizes a key stream and then keeps hashing.
    fn finalize(mut self) -> [u8; 32] {
        let len = self.nbits;
        self.push_bits(0x8000_0000, 1);
        while self.nbits % 512 != 448 {
            self.push_bits(0, 1);
        }
        // `nbits % 512 == 448` puts the cursor at byte 56 of the block, so the
        // length field lands whole. MD5 writes it little-endian (RFC 1321
        // §3.4); the SHA family writes it big-endian (FIPS 180-4 §5.1.1).
        let bytes = if self.algo == Algo::Md5 {
            len.to_le_bytes()
        } else {
            len.to_be_bytes()
        };
        self.buf[56..64].copy_from_slice(&bytes);
        self.nbits = self.nbits.wrapping_add(64);
        self.compress();
        self.digest_bytes()
    }

    fn digest_bytes(&self) -> [u8; 32] {
        let mut out = [0u8; 32];
        let (words, _) = out.as_chunks_mut::<4>();
        for (slot, h) in words.iter_mut().zip(self.h) {
            *slot = if self.algo == Algo::Md5 {
                h.to_le_bytes()
            } else {
                h.to_be_bytes()
            };
        }
        out
    }

    fn compress(&mut self) {
        match self.algo {
            Algo::Sha1 => compress_sha1(&mut self.h, &self.buf),
            Algo::Md5 => compress_md5(&mut self.h, &self.buf),
            Algo::Sha224 | Algo::Sha256 => compress_sha256(&mut self.h, &self.buf),
        }
    }
}

/// FIPS 180-4 §6.1.2.
fn compress_sha1(h: &mut [u32; 8], block: &[u8; BLOCK_BYTES]) {
    let mut w = [0u32; 80];
    for (i, slot) in w.iter_mut().take(16).enumerate() {
        let mut b = [0u8; 4];
        b.copy_from_slice(&block[4 * i..4 * i + 4]);
        *slot = u32::from_be_bytes(b);
    }
    for i in 16..80 {
        w[i] = (w[i - 3] ^ w[i - 8] ^ w[i - 14] ^ w[i - 16]).rotate_left(1);
    }
    let (mut a, mut b, mut c, mut d, mut e) = (h[0], h[1], h[2], h[3], h[4]);
    for (i, wi) in w.iter().enumerate() {
        let (f, k) = match i / 20 {
            0 => ((b & c) | ((!b) & d), 0x5a82_7999u32),
            1 => (b ^ c ^ d, 0x6ed9_eba1),
            2 => ((b & c) | (b & d) | (c & d), 0x8f1b_bcdc),
            _ => (b ^ c ^ d, 0xca62_c1d6),
        };
        let t = a
            .rotate_left(5)
            .wrapping_add(f)
            .wrapping_add(e)
            .wrapping_add(k)
            .wrapping_add(*wi);
        e = d;
        d = c;
        c = b.rotate_left(30);
        b = a;
        a = t;
    }
    h[0] = h[0].wrapping_add(a);
    h[1] = h[1].wrapping_add(b);
    h[2] = h[2].wrapping_add(c);
    h[3] = h[3].wrapping_add(d);
    h[4] = h[4].wrapping_add(e);
}

/// FIPS 180-4 §6.2.2 — the same round function for SHA-224 and SHA-256; only
/// the initial value and the truncation differ.
fn compress_sha256(h: &mut [u32; 8], block: &[u8; BLOCK_BYTES]) {
    let mut w = [0u32; 64];
    for (i, slot) in w.iter_mut().take(16).enumerate() {
        let mut b = [0u8; 4];
        b.copy_from_slice(&block[4 * i..4 * i + 4]);
        *slot = u32::from_be_bytes(b);
    }
    for i in 16..64 {
        let s0 = w[i - 15].rotate_right(7) ^ w[i - 15].rotate_right(18) ^ (w[i - 15] >> 3);
        let s1 = w[i - 2].rotate_right(17) ^ w[i - 2].rotate_right(19) ^ (w[i - 2] >> 10);
        w[i] = w[i - 16]
            .wrapping_add(s0)
            .wrapping_add(w[i - 7])
            .wrapping_add(s1);
    }
    let mut v = *h;
    for (i, wi) in w.iter().enumerate() {
        let s1 = v[4].rotate_right(6) ^ v[4].rotate_right(11) ^ v[4].rotate_right(25);
        let ch = (v[4] & v[5]) ^ ((!v[4]) & v[6]);
        let t1 = v[7]
            .wrapping_add(s1)
            .wrapping_add(ch)
            .wrapping_add(SHA256_K[i])
            .wrapping_add(*wi);
        let s0 = v[0].rotate_right(2) ^ v[0].rotate_right(13) ^ v[0].rotate_right(22);
        let maj = (v[0] & v[1]) ^ (v[0] & v[2]) ^ (v[1] & v[2]);
        let t2 = s0.wrapping_add(maj);
        v[7] = v[6];
        v[6] = v[5];
        v[5] = v[4];
        v[4] = v[3].wrapping_add(t1);
        v[3] = v[2];
        v[2] = v[1];
        v[1] = v[0];
        v[0] = t1.wrapping_add(t2);
    }
    for (slot, add) in h.iter_mut().zip(v) {
        *slot = slot.wrapping_add(add);
    }
}

/// RFC 1321 §3.4. The block words are **little-endian**, which is the whole of
/// what makes MD5 different from its SHA-1 cousin at this level.
fn compress_md5(h: &mut [u32; 8], block: &[u8; BLOCK_BYTES]) {
    let mut m = [0u32; 16];
    for (i, slot) in m.iter_mut().enumerate() {
        let mut b = [0u8; 4];
        b.copy_from_slice(&block[4 * i..4 * i + 4]);
        *slot = u32::from_le_bytes(b);
    }
    let (mut a, mut b, mut c, mut d) = (h[0], h[1], h[2], h[3]);
    for i in 0..64 {
        let (f, g) = match i / 16 {
            0 => ((b & c) | ((!b) & d), i),
            1 => ((d & b) | ((!d) & c), (5 * i + 1) % 16),
            2 => (b ^ c ^ d, (3 * i + 5) % 16),
            _ => (c ^ (b | (!d)), (7 * i) % 16),
        };
        let t = f.wrapping_add(a).wrapping_add(MD5_T[i]).wrapping_add(m[g]);
        a = d;
        d = c;
        c = b;
        b = b.wrapping_add(t.rotate_left(MD5_S[i]));
    }
    h[0] = h[0].wrapping_add(a);
    h[1] = h[1].wrapping_add(b);
    h[2] = h[2].wrapping_add(c);
    h[3] = h[3].wrapping_add(d);
}

// ---------------------------------------------------------------------------
// Variants
// ---------------------------------------------------------------------------

/// Which family's `HASH` this instance is.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Variant {
    /// RM0090 §25: SHA-1 and MD5. `ALGO[1]` and `MDMAT` are not decoded.
    F4,
    /// RM0351 §30 / RM0432 §26: adds SHA-224, SHA-256 and `MDMAT`.
    V2,
}

impl Variant {
    /// The spelling a `.machine` file uses.
    #[must_use]
    pub const fn name(self) -> &'static str {
        match self {
            Variant::F4 => "f4",
            Variant::V2 => "v2",
        }
    }

    /// The `CR` bits this variant actually implements.
    const fn cr_mask(self) -> u32 {
        let common =
            CR_DMAE | (CR_DATATYPE_MASK << CR_DATATYPE_SHIFT) | CR_MODE | CR_ALGO0 | CR_LKEY;
        match self {
            Variant::F4 => common,
            Variant::V2 => common | CR_MDMAT | CR_ALGO1,
        }
    }
}

// ---------------------------------------------------------------------------
// The block's state
// ---------------------------------------------------------------------------

/// Which of an HMAC's three `DCAL` phases is running.
///
/// A plain hash never leaves `Message`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Phase {
    /// The message of a plain hash, or the idle state after a digest.
    Message,
    /// HMAC: the first key write, which builds `K0 ^ ipad`.
    Key1,
    /// HMAC: the message, hashed under the inner pad.
    HmacMessage,
    /// HMAC: the second key write, which builds `K0 ^ opad`.
    Key2,
}

impl Phase {
    const fn bits(self) -> u32 {
        match self {
            Phase::Message => 0,
            Phase::Key1 => 1,
            Phase::HmacMessage => 2,
            Phase::Key2 => 3,
        }
    }

    const fn from_bits(bits: u32) -> Phase {
        match bits & 0x3 {
            0 => Phase::Message,
            1 => Phase::Key1,
            2 => Phase::HmacMessage,
            _ => Phase::Key2,
        }
    }

    const fn is_key(self) -> bool {
        matches!(self, Phase::Key1 | Phase::Key2)
    }
}

/// Everything the guest can change and everything a context swap carries.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct State {
    /// The writable `CR` bits, as last written.
    cr: u32,
    imr: u32,
    nblw: u32,
    /// What a read of `DIN` gives back: the last word written to it.
    din: u32,
    dcis: bool,
    algo: Algo,
    hmac: bool,
    datatype: u32,
    lkey: bool,
    phase: Phase,
    eng: Engine,
    fifo: [u32; FIFO_WORDS],
    nwords: u32,
    key: [u8; BLOCK_BYTES],
    key_len: u32,
    inner: [u8; 32],
    hr: [u32; 8],
}

impl State {
    fn new() -> State {
        State {
            cr: 0,
            imr: 0,
            nblw: 0,
            din: 0,
            dcis: false,
            algo: Algo::Sha1,
            hmac: false,
            datatype: 0,
            lkey: false,
            phase: Phase::Message,
            eng: Engine::new(Algo::Sha1),
            fifo: [0; FIFO_WORDS],
            nwords: 0,
            key: [0; BLOCK_BYTES],
            key_len: 0,
            inner: [0; 32],
            hr: [0; 8],
        }
    }

    /// `DIN` can take another word.
    const fn dinis(&self) -> bool {
        (self.nwords as usize) < FIFO_WORDS
    }

    /// The core is holding a complete block — see the module note.
    const fn busy(&self) -> bool {
        (self.nwords as usize) >= FIFO_WORDS
    }

    fn sr(&self) -> u32 {
        let mut sr = 0;
        if self.dinis() {
            sr |= SR_DINIS;
        }
        if self.dcis {
            sr |= SR_DCIS;
        }
        if self.cr & CR_DMAE != 0 {
            sr |= SR_DMAS;
        }
        if self.busy() {
            sr |= SR_BUSY;
        }
        sr
    }

    fn irq(&self) -> bool {
        (self.dinis() && self.imr & IMR_DINIE != 0) || (self.dcis && self.imr & IMR_DCIE != 0)
    }

    fn dma_request(&self) -> bool {
        self.cr & CR_DMAE != 0 && self.dinis()
    }

    /// RM0090 §25.3.3: reverse the word's units, then feed it MSB first.
    const fn swap(&self, w: u32) -> u32 {
        match self.datatype {
            1 => w.rotate_left(16),
            2 => w.swap_bytes(),
            3 => w.reverse_bits(),
            _ => w,
        }
    }

    /// `INIT`: latch the configuration from the same write and start over.
    fn init(&mut self, cr: u32, variant: Variant) {
        let algo_bits = ((cr >> 7) & 0x1)
            | if variant == Variant::V2 {
                (cr >> 17) & 0x2
            } else {
                0
            };
        self.algo = Algo::from_bits(algo_bits);
        self.hmac = cr & CR_MODE != 0;
        self.datatype = (cr >> CR_DATATYPE_SHIFT) & CR_DATATYPE_MASK;
        self.lkey = cr & CR_LKEY != 0;
        self.phase = if self.hmac {
            Phase::Key1
        } else {
            Phase::Message
        };
        self.eng = Engine::new(self.algo);
        self.fifo = [0; FIFO_WORDS];
        self.nwords = 0;
        self.key = [0; BLOCK_BYTES];
        self.key_len = 0;
        self.inner = [0; 32];
        self.hr = [0; 8];
        self.nblw = 0;
        self.dcis = false;
    }

    /// A write to `DIN`.
    ///
    /// The sixteen buffered words go to the core when a **seventeenth**
    /// arrives, never when the sixteenth does — see the module note on `NBLW`.
    fn write_din(&mut self, raw: u32) {
        self.din = raw;
        if (self.nwords as usize) >= FIFO_WORDS {
            for i in 0..FIFO_WORDS {
                self.absorb(self.fifo[i], 32);
            }
            self.nwords = 0;
        }
        let w = self.swap(raw);
        self.fifo[self.nwords as usize] = w;
        self.nwords += 1;
    }

    /// Push one (possibly partial) word into the running hash.
    fn absorb(&mut self, w: u32, nbits: u32) {
        if nbits >= 32 {
            self.eng.push_word(w);
        } else {
            self.eng.push_bits(w, nbits);
        }
        if self.phase.is_key() {
            // A key phase also records the bytes, because a short key is not
            // hashed — it is zero-extended to the block size (RFC 2104 §2).
            let bytes = w.to_be_bytes();
            for byte in bytes.iter().take((nbits / 8) as usize) {
                if (self.key_len as usize) < BLOCK_BYTES {
                    self.key[self.key_len as usize] = *byte;
                }
                self.key_len = self.key_len.saturating_add(1);
            }
        }
    }

    /// RFC 2104 §2 / FIPS 198-1 §4: the key, reduced to one block.
    fn k0(&self) -> [u8; BLOCK_BYTES] {
        let mut k0 = [0u8; BLOCK_BYTES];
        if self.lkey {
            // `LKEY` says the key is longer than a block, so it was streamed
            // through the engine and its digest stands in for it.
            let d = self.eng.finalize();
            let n = self.algo.digest_len();
            k0[..n].copy_from_slice(&d[..n]);
        } else {
            let n = (self.key_len as usize).min(BLOCK_BYTES);
            k0[..n].copy_from_slice(&self.key[..n]);
        }
        k0
    }

    /// Restart the engine on `K0 ^ pad`, the first block of an HMAC hash.
    fn start_padded(&mut self, pad: u8) {
        let k0 = self.k0();
        let mut block = [pad; BLOCK_BYTES];
        for (slot, key) in block.iter_mut().zip(k0) {
            *slot ^= key;
        }
        self.eng = Engine::new(self.algo);
        self.eng.push_bytes(&block);
        self.key_len = 0;
    }

    /// `HRx` holds the digest as a big-endian byte stream, which is what makes
    /// MD5's little-endian words come out byte-swapped relative to `h`.
    fn set_hr(&mut self, digest: &[u8]) {
        self.hr = [0; 8];
        for (slot, chunk) in self.hr.iter_mut().zip(digest.chunks(4)) {
            let mut w = [0u8; 4];
            w[..chunk.len()].copy_from_slice(chunk);
            *slot = u32::from_be_bytes(w);
        }
    }

    /// `DCAL`: flush the FIFO with the last word cut to `NBLW` bits, then end
    /// whichever phase is running.
    fn dcal(&mut self) {
        let n = (self.nwords as usize).min(FIFO_WORDS);
        let last = if self.nblw & STR_NBLW_MASK == 0 {
            32
        } else {
            self.nblw & STR_NBLW_MASK
        };
        for i in 0..n {
            let bits = if i + 1 == n { last } else { 32 };
            self.absorb(self.fifo[i], bits);
        }
        self.nwords = 0;
        let dlen = self.algo.digest_len();
        match self.phase {
            Phase::Message => {
                let digest = self.eng.finalize();
                self.set_hr(&digest[..dlen]);
            }
            Phase::Key1 => {
                self.start_padded(0x36);
                self.phase = Phase::HmacMessage;
            }
            Phase::HmacMessage => {
                self.inner = self.eng.finalize();
                // The third key write streams in from scratch exactly as the
                // first one did, and `LKEY` still selects how it is reduced.
                self.eng = Engine::new(self.algo);
                self.key = [0; BLOCK_BYTES];
                self.key_len = 0;
                self.phase = Phase::Key2;
            }
            Phase::Key2 => {
                let inner = self.inner;
                self.start_padded(0x5c);
                self.eng.push_bytes(&inner[..dlen]);
                let digest = self.eng.finalize();
                self.set_hr(&digest[..dlen]);
                self.phase = Phase::Message;
            }
        }
        self.dcis = true;
    }

    // -- the context block -------------------------------------------------

    fn csr(&self, i: usize) -> u32 {
        match i {
            0 => {
                self.algo.bits()
                    | u32::from(self.hmac) << 2
                    | (self.datatype & 0x3) << 3
                    | u32::from(self.lkey) << 5
                    | self.phase.bits() << 6
                    | (self.nwords & 0x1f) << 8
                    | (self.key_len.min(BLOCK_BYTES as u32) & 0x7f) << 13
                    | u32::from(self.dcis) << 20
            }
            1 => self.eng.nbits as u32,
            2 => (self.eng.nbits >> 32) as u32,
            3..=10 => self.eng.h[i - 3],
            11..=26 => self.fifo[i - 11],
            27..=42 => {
                let off = (i - 27) * 4;
                let mut w = [0u8; 4];
                w.copy_from_slice(&self.key[off..off + 4]);
                u32::from_le_bytes(w)
            }
            43..=50 => {
                let off = (i - 43) * 4;
                let mut w = [0u8; 4];
                w.copy_from_slice(&self.inner[off..off + 4]);
                u32::from_le_bytes(w)
            }
            _ => 0,
        }
    }

    fn set_csr(&mut self, i: usize, v: u32) {
        match i {
            0 => {
                self.algo = Algo::from_bits(v);
                self.hmac = v & (1 << 2) != 0;
                self.datatype = (v >> 3) & 0x3;
                self.lkey = v & (1 << 5) != 0;
                self.phase = Phase::from_bits(v >> 6);
                self.nwords = ((v >> 8) & 0x1f).min(FIFO_WORDS as u32);
                self.key_len = ((v >> 13) & 0x7f).min(BLOCK_BYTES as u32);
                self.dcis = v & (1 << 20) != 0;
                self.eng.algo = self.algo;
            }
            1 => self.eng.nbits = (self.eng.nbits & 0xffff_ffff_0000_0000) | u64::from(v),
            2 => self.eng.nbits = (self.eng.nbits & 0xffff_ffff) | (u64::from(v) << 32),
            3..=10 => self.eng.h[i - 3] = v,
            11..=26 => self.fifo[i - 11] = v,
            27..=42 => {
                let off = (i - 27) * 4;
                self.key[off..off + 4].copy_from_slice(&v.to_le_bytes());
            }
            43..=50 => {
                let off = (i - 43) * 4;
                self.inner[off..off + 4].copy_from_slice(&v.to_le_bytes());
            }
            _ => {}
        }
    }
}

// ---------------------------------------------------------------------------
// The register face
// ---------------------------------------------------------------------------

struct Registers {
    state: Mutex<State>,
    variant: Variant,
    irq: Mutex<Option<WireSource>>,
    drq: Mutex<Option<WireSource>>,
}

impl fmt::Debug for Registers {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let mut s = f.debug_struct("Registers");
        s.field("variant", &self.variant);
        match self.state.try_lock() {
            Some(state) => s.field("state", &*state),
            None => s.field("state", &"<locked>"),
        };
        s.finish()
    }
}

impl Registers {
    /// Drive both outputs from one register image, with no lock held — the
    /// re-entrancy contract in `CLAUDE.md`.
    fn refresh_outputs(&self) {
        let (irq, drq) = {
            let state = self.state.lock();
            (state.irq(), state.dma_request())
        };
        if let Some(wire) = self.irq.lock().as_ref() {
            wire.set(Level::from_bool(irq));
        }
        if let Some(wire) = self.drq.lock().as_ref() {
            wire.set(Level::from_bool(drq));
        }
    }

    fn read_register(&self, offset: u64) -> core::result::Result<u32, BusError> {
        let state = self.state.lock();
        match offset {
            R_CR => {
                let mut cr = state.cr & self.variant.cr_mask();
                cr |= (state.nwords & 0xf) << CR_NBW_SHIFT;
                if state.nwords > 0 {
                    cr |= CR_DINNE;
                }
                Ok(cr)
            }
            // Read-back of the last word written. Never a pop: the FIFO only
            // ever drains into the core, so `MemAttrs::debug` has nothing to
            // disturb here and every read below is likewise side-effect free.
            R_DIN => Ok(state.din),
            R_STR => Ok(state.nblw & STR_NBLW_MASK),
            R_IMR => Ok(state.imr & IMR_MASK),
            R_SR => Ok(state.sr()),
            o if (R_HR_LOW..R_HR_LOW + 5 * 4).contains(&o) && o % 4 == 0 => {
                Ok(state.hr[((o - R_HR_LOW) / 4) as usize])
            }
            o if (R_CSR..R_CSR + CSR_COUNT as u64 * 4).contains(&o) && o % 4 == 0 => {
                Ok(state.csr(((o - R_CSR) / 4) as usize))
            }
            o if (R_HR_HIGH..R_HR_HIGH + 8 * 4).contains(&o) && o % 4 == 0 => {
                Ok(state.hr[((o - R_HR_HIGH) / 4) as usize])
            }
            _ => Err(BusError::BadAccess),
        }
    }

    fn write_register(&self, offset: u64, value: u32) -> MemResult {
        let mut state = self.state.lock();
        match offset {
            R_CR => {
                state.cr = value & self.variant.cr_mask();
                if value & CR_INIT != 0 {
                    state.init(value, self.variant);
                }
            }
            R_DIN => state.write_din(value),
            R_STR => {
                // `NBLW` is an ordinary read/write field and `DCAL` a
                // write-one trigger, and every driver sets both in one store.
                // The field is taken first, because it is what the trigger
                // reads -- and it is taken even when it is zero, which is the
                // value that means a *full* last word rather than no value.
                state.nblw = value & STR_NBLW_MASK;
                if value & STR_DCAL != 0 {
                    state.dcal();
                }
            }
            R_IMR => state.imr = value & IMR_MASK,
            // `DCIS` is write-zero-to-clear. `DINIS` and `BUSY` follow the
            // FIFO, so a write that clears `DINIS` is accepted and the bit is
            // back before the guest can look.
            R_SR => {
                if value & SR_DCIS == 0 {
                    state.dcis = false;
                }
            }
            o if (R_CSR..R_CSR + CSR_COUNT as u64 * 4).contains(&o) && o % 4 == 0 => {
                state.set_csr(((o - R_CSR) / 4) as usize, value);
            }
            // `HR` is the result, not an input.
            o if (R_HR_LOW..R_HR_LOW + 5 * 4).contains(&o) && o % 4 == 0 => {}
            o if (R_HR_HIGH..R_HR_HIGH + 8 * 4).contains(&o) && o % 4 == 0 => {}
            _ => return Err(BusError::BadAccess),
        }
        Ok(())
    }
}

impl MemOps for Registers {
    fn read(&self, offset: u64, dst: &mut [u8], _attrs: MemAttrs) -> MemResult {
        if offset >= REGISTER_BYTES {
            return Err(BusError::BadAccess);
        }
        let value = self.read_register(offset)?;
        let bytes = value.to_le_bytes();
        for (slot, byte) in dst.iter_mut().zip(bytes) {
            *slot = byte;
        }
        Ok(())
    }

    fn write(&self, offset: u64, src: &[u8], attrs: MemAttrs) -> MemResult {
        if offset >= REGISTER_BYTES {
            return Err(BusError::BadAccess);
        }
        if attrs.debug {
            return Err(BusError::BadAccess);
        }
        let mut word = [0u8; 4];
        for (slot, byte) in word.iter_mut().zip(src) {
            *slot = *byte;
        }
        self.write_register(offset, u32::from_le_bytes(word))?;
        self.refresh_outputs();
        Ok(())
    }

    fn constraints(&self) -> AccessConstraints {
        AccessConstraints::word(Width::U32, Endian::Little)
    }
}

// ---------------------------------------------------------------------------
// The device
// ---------------------------------------------------------------------------

/// The STM32 HASH processor.
#[derive(Debug)]
pub struct Hash {
    regs: Arc<Registers>,
    region: RegionRef,
}

impl Hash {
    /// Build one from `.machine` properties.
    ///
    /// # Errors
    ///
    /// If `variant` is neither `f4` nor `v2`, or an unknown property is set.
    pub fn new(props: &Props) -> Result<Hash> {
        let mut r = props.reader();
        let variant = match r.or_str("variant", "f4")? {
            "f4" => Variant::F4,
            "v2" => Variant::V2,
            other => {
                return Err(Error::Property(format!(
                    "`variant` is `f4` or `v2`, not `{other}`"
                )));
            }
        };
        r.finish()?;
        Ok(Hash::build(variant))
    }

    /// Build one directly.
    #[must_use]
    pub fn build(variant: Variant) -> Hash {
        let regs = Arc::new(Registers {
            state: Mutex::with_rank(LockRank::DEVICE, State::new()),
            variant,
            irq: Mutex::with_rank(LockRank::WIRE, None),
            drq: Mutex::with_rank(LockRank::WIRE, None),
        });
        let region = Arc::new(Region::io(
            "hash",
            REGISTER_BYTES,
            Arc::clone(&regs) as Arc<dyn MemOps>,
        ));
        Hash { regs, region }
    }

    /// Which family's block this is.
    #[must_use]
    pub fn variant(&self) -> Variant {
        self.regs.variant
    }

    /// The interrupt line's current level.
    #[must_use]
    pub fn irq_level(&self) -> Level {
        Level::from_bool(self.regs.state.lock().irq())
    }

    /// Whether the DMA request is asserted.
    #[must_use]
    pub fn dma_requesting(&self) -> bool {
        self.regs.state.lock().dma_request()
    }

    /// The digest registers, `HR0`..`HR7`.
    #[must_use]
    pub fn digest(&self) -> [u32; 8] {
        self.regs.state.lock().hr
    }
}

impl Device for Hash {
    fn class(&self) -> &'static DeviceClass {
        &CLASS
    }

    fn realize(&self, _ctx: &mut RealizeCtx<'_>) -> Result<()> {
        Ok(())
    }

    fn reset(&self, _kind: ResetKind) {
        *self.regs.state.lock() = State::new();
        self.regs.refresh_outputs();
    }

    fn save(&self, w: &mut ChunkWriter<'_>) -> Result<()> {
        let state = *self.regs.state.lock();
        w.write_u8(match self.regs.variant {
            Variant::F4 => 0,
            Variant::V2 => 1,
        })?;
        w.write_u32(state.cr)?;
        w.write_u32(state.imr)?;
        w.write_u32(state.nblw)?;
        w.write_u32(state.din)?;
        w.write_bool(state.dcis)?;
        w.write_u8(state.algo.bits() as u8)?;
        w.write_bool(state.hmac)?;
        w.write_u32(state.datatype)?;
        w.write_bool(state.lkey)?;
        w.write_u8(state.phase.bits() as u8)?;
        w.write_u64(state.eng.nbits)?;
        for h in state.eng.h {
            w.write_u32(h)?;
        }
        w.write_all(&state.eng.buf)?;
        for word in state.fifo {
            w.write_u32(word)?;
        }
        w.write_u32(state.nwords)?;
        w.write_all(&state.key)?;
        w.write_u32(state.key_len)?;
        w.write_all(&state.inner)?;
        for hr in state.hr {
            w.write_u32(hr)?;
        }
        Ok(())
    }

    fn load(&self, r: &mut ChunkReader<'_>) -> Result<()> {
        let variant = match r.read_u8()? {
            0 => Variant::F4,
            1 => Variant::V2,
            other => {
                return Err(Error::State(format!(
                    "snapshot has HASH variant {other}, which this build does not know"
                )));
            }
        };
        if variant != self.regs.variant {
            return Err(Error::State(format!(
                "snapshot has a `{}` HASH block, this one is `{}`",
                variant.name(),
                self.regs.variant.name()
            )));
        }
        let mut state = State::new();
        state.cr = r.read_u32()?;
        state.imr = r.read_u32()?;
        state.nblw = r.read_u32()?;
        state.din = r.read_u32()?;
        state.dcis = r.read_bool()?;
        state.algo = Algo::from_bits(u32::from(r.read_u8()?));
        state.hmac = r.read_bool()?;
        state.datatype = r.read_u32()? & CR_DATATYPE_MASK;
        state.lkey = r.read_bool()?;
        state.phase = Phase::from_bits(u32::from(r.read_u8()?));
        state.eng = Engine::new(state.algo);
        state.eng.nbits = r.read_u64()?;
        for slot in &mut state.eng.h {
            *slot = r.read_u32()?;
        }
        state.eng.buf.copy_from_slice(r.take(BLOCK_BYTES)?);
        for slot in &mut state.fifo {
            *slot = r.read_u32()?;
        }
        state.nwords = r.read_u32()?;
        if state.nwords as usize > FIFO_WORDS {
            return Err(Error::State(format!(
                "snapshot has {} words in a sixteen-word HASH FIFO",
                state.nwords
            )));
        }
        state.key.copy_from_slice(r.take(BLOCK_BYTES)?);
        state.key_len = r.read_u32()?;
        state.inner.copy_from_slice(r.take(32)?);
        for slot in &mut state.hr {
            *slot = r.read_u32()?;
        }
        *self.regs.state.lock() = state;
        self.regs.refresh_outputs();
        Ok(())
    }

    fn region(&self, name: &str) -> Option<RegionRef> {
        matches!(name, "" | "regs").then(|| Arc::clone(&self.region))
    }

    fn connect(&self, port: &str, source: WireSource) -> Result<()> {
        match port {
            pin::IRQ => {
                *self.regs.irq.lock() = Some(source);
                self.regs.refresh_outputs();
                Ok(())
            }
            pin::DMA => {
                *self.regs.drq.lock() = Some(source);
                self.regs.refresh_outputs();
                Ok(())
            }
            other => Err(Error::Config {
                at: String::from(other),
                message: format!(
                    "a HASH processor drives `{}` and `{}` and nothing else",
                    pin::IRQ,
                    pin::DMA
                ),
            }),
        }
    }

    fn announce(&self, _port: &str) {
        self.regs.refresh_outputs();
    }
}

impl Instance for Hash {}

/// The `st.hash` device class.
pub static CLASS: DeviceClass = DeviceClass {
    name: CLASS_NAME,
    version: STATE_VERSION,
    summary: "STM32 HASH processor: SHA-1/MD5 (and SHA-224/256 on `v2`), HMAC, \
              and the CSR context swap",
    properties: &[PropertySpec {
        name: "variant",
        kind: ValueKind::Str,
        required: false,
        summary: "`f4` (default): SHA-1 and MD5, RM0090 §25. `v2`: adds SHA-224, \
                  SHA-256 and MDMAT",
    }],
    construct: |props| Ok(Box::new(Hash::new(props)?)),
};

/// Add this class to a registry.
///
/// # Errors
///
/// If something already claimed the name.
pub fn register(registry: &mut crate::core::Registry) -> Result<()> {
    registry.add(&CLASS)
}

/// Bind this class into the machine graph.
///
/// # Errors
///
/// If the class is already bound.
pub fn bind(bindings: &mut crate::machine::Bindings) -> Result<()> {
    bindings.bind(CLASS_NAME, |props| Ok(Arc::new(Hash::new(props)?)))
}

/// The validator's view of this class.
#[must_use]
pub fn schema() -> ClassSchema {
    ClassSchema::new(CLASS_NAME)
        .prop(PropSchema::new("variant", ValueKind::Str).values(&["f4", "v2"]))
        .port(pin::IRQ, PortDir::Out)
        .port(pin::DMA, PortDir::Out)
        .region("")
        .region("regs")
}

#[cfg(test)]
mod tests;
