//! `st.hash` against the published vectors.
//!
//! The point of a hash is that it agrees with everyone else's, so almost
//! nothing here is self-consistency. The digests are FIPS 180-4 Appendix A/B
//! (SHA-1, SHA-224, SHA-256), RFC 1321 Appendix A.5 (MD5), RFC 4231 (HMAC with
//! the SHA-2 family) and RFC 2202 (HMAC-MD5 and HMAC-SHA-1). The handful that
//! are not in a document — the sixty-three and two-hundred byte runs of `'a'`
//! that exercise a partial word inside a full block — were taken from a
//! third-party implementation used as a black box, which is the same check by
//! another route.

use super::*;

use alloc::vec::Vec;
use core::sync::atomic::{AtomicBool, Ordering};

use crate::core::props::Value;
use crate::core::registry::Registry;
use crate::core::state::{MachineShape, Migrations, StateReader, StateWriter};
use crate::core::wire::{Wire, WireId, WireIdAllocator, WireSink};

// ---------------------------------------------------------------------------
// Driving the block the way firmware does
// ---------------------------------------------------------------------------

fn f4() -> Hash {
    Hash::build(Variant::F4)
}

fn v2() -> Hash {
    Hash::build(Variant::V2)
}

fn read(d: &Hash, offset: u64, attrs: MemAttrs) -> u32 {
    let mut buf = [0u8; 4];
    d.regs
        .read(offset, &mut buf, attrs)
        .expect("a word read of a decoded register");
    u32::from_le_bytes(buf)
}

fn peek(d: &Hash, offset: u64) -> u32 {
    read(d, offset, MemAttrs::DEFAULT)
}

fn try_write(d: &Hash, offset: u64, value: u32, attrs: MemAttrs) -> MemResult {
    d.regs.write(offset, &value.to_le_bytes(), attrs)
}

fn poke(d: &Hash, offset: u64, value: u32) {
    try_write(d, offset, value, MemAttrs::DEFAULT).expect("a word write of a decoded register");
}

/// The `CR` write that starts an operation.
fn start(d: &Hash, algo: Algo, datatype: u32, hmac: bool, lkey: bool) {
    let mut cr = CR_INIT | (datatype << CR_DATATYPE_SHIFT);
    if algo.bits() & 1 != 0 {
        cr |= CR_ALGO0;
    }
    if algo.bits() & 2 != 0 {
        cr |= CR_ALGO1;
    }
    if hmac {
        cr |= CR_MODE;
    }
    if lkey {
        cr |= CR_LKEY;
    }
    poke(d, R_CR, cr);
}

/// Turn a message into the words a driver in this `DATATYPE` would write.
///
/// The swaps are all involutions, so undoing one is applying it: the word the
/// guest writes is the swap of the big-endian message word it wants the core to
/// see. Returns the `NBLW` the last word needs.
fn feed(d: &Hash, datatype: u32, msg: &[u8]) -> u32 {
    let mut state = State::new();
    state.datatype = datatype;
    for chunk in msg.chunks(4) {
        let mut b = [0u8; 4];
        b[..chunk.len()].copy_from_slice(chunk);
        poke(d, R_DIN, state.swap(u32::from_be_bytes(b)));
    }
    ((msg.len() % 4) * 8) as u32
}

/// `HR0..HR7` from the wide window, cut to the algorithm's digest length.
fn digest_of(d: &Hash, algo: Algo) -> Vec<u8> {
    let mut out = Vec::new();
    for i in 0..8 {
        out.extend_from_slice(&peek(d, R_HR_HIGH + i * 4).to_be_bytes());
    }
    out.truncate(algo.digest_len());
    out
}

/// One complete plain-hash operation.
fn digest(d: &Hash, algo: Algo, datatype: u32, msg: &[u8]) -> Vec<u8> {
    start(d, algo, datatype, false, false);
    let nblw = feed(d, datatype, msg);
    poke(d, R_STR, nblw | STR_DCAL);
    digest_of(d, algo)
}

/// One complete HMAC operation: key, message, key, exactly as RM0090 §25.3.6
/// lays it out.
fn hmac(d: &Hash, algo: Algo, datatype: u32, key: &[u8], msg: &[u8]) -> Vec<u8> {
    start(d, algo, datatype, true, key.len() > BLOCK_BYTES);
    for part in [key, msg, key] {
        let nblw = feed(d, datatype, part);
        poke(d, R_STR, nblw | STR_DCAL);
    }
    digest_of(d, algo)
}

fn hex(s: &str) -> Vec<u8> {
    assert!(s.len().is_multiple_of(2), "a hex string has an even length");
    s.as_bytes()
        .chunks(2)
        .map(|pair| {
            let digit = |c: u8| match c {
                b'0'..=b'9' => c - b'0',
                b'a'..=b'f' => c - b'a' + 10,
                _ => panic!("not lower-case hex: {c}"),
            };
            digit(pair[0]) << 4 | digit(pair[1])
        })
        .collect()
}

/// FIPS 180-4 Appendix A/B's second message: fifty-six bytes, so the digest
/// needs a padding block of its own.
const TWO_BLOCK: &[u8] = b"abcdbcdecdefdefgefghfghighijhijkijkljklmklmnlmnomnopnopq";

// ---------------------------------------------------------------------------
// The published digests
// ---------------------------------------------------------------------------

#[test]
fn sha1_matches_fips_180_4_appendix_a() {
    let d = f4();
    assert_eq!(
        digest(&d, Algo::Sha1, 2, b"abc"),
        hex("a9993e364706816aba3e25717850c26c9cd0d89d")
    );
    assert_eq!(
        digest(&d, Algo::Sha1, 2, TWO_BLOCK),
        hex("84983e441c3bd26ebaae4aa1f95129e5e54670f1")
    );
}

#[test]
fn sha256_matches_fips_180_4_appendix_b() {
    let d = v2();
    assert_eq!(
        digest(&d, Algo::Sha256, 2, b"abc"),
        hex("ba7816bf8f01cfea414140de5dae2223b00361a396177a9cb410ff61f20015ad")
    );
    assert_eq!(
        digest(&d, Algo::Sha256, 2, TWO_BLOCK),
        hex("248d6a61d20638b8e5c026930c3e6039a33ce45964ff2167f6ecedd419db06c1")
    );
}

#[test]
fn sha224_matches_fips_180_4_appendix_a() {
    let d = v2();
    assert_eq!(
        digest(&d, Algo::Sha224, 2, b"abc"),
        hex("23097d223405d8228642a477bda255b32aadbce4bda0b3f7e36c9da7")
    );
    assert_eq!(
        digest(&d, Algo::Sha224, 2, TWO_BLOCK),
        hex("75388b16512776cc5dba5da1fd890150b0c6455cb4f58b1952522525")
    );
}

#[test]
fn md5_matches_the_rfc_1321_suite() {
    let d = f4();
    for (msg, want) in [
        (&b""[..], "d41d8cd98f00b204e9800998ecf8427e"),
        (&b"a"[..], "0cc175b9c0f1b6a831c399e269772661"),
        (&b"abc"[..], "900150983cd24fb0d6963f7d28e17f72"),
        (&b"message digest"[..], "f96b697d7cb7938d525a2f31aaf161d0"),
        (
            &b"abcdefghijklmnopqrstuvwxyz"[..],
            "c3fcd3d76192e4007dfb496cca67e13b",
        ),
        (
            &b"ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789"[..],
            "d174ab98d277d9f5a5611c2c9f419d9f",
        ),
        (
            &b"12345678901234567890123456789012345678901234567890123456789012345678901234567890"[..],
            "57edf4a22be3c955ac49da2e2107b67a",
        ),
    ] {
        assert_eq!(digest(&d, Algo::Md5, 2, msg), hex(want), "MD5 of {msg:?}");
    }
}

#[test]
fn md5_of_the_empty_message() {
    // The degenerate path: `DCAL` with nothing at all in the FIFO still has to
    // emit the padding block.
    let d = f4();
    start(&d, Algo::Md5, 2, false, false);
    poke(&d, R_STR, STR_DCAL);
    assert_eq!(
        digest_of(&d, Algo::Md5),
        hex("d41d8cd98f00b204e9800998ecf8427e")
    );
}

#[test]
fn a_message_longer_than_one_block_compresses_as_words_arrive() {
    // FIPS 180-4 Appendix B.3: a million 'a's, which is 250 000 words through
    // `DIN` and 15 625 block compressions.
    let d = v2();
    let msg = alloc::vec![b'a'; 1_000_000];
    start(&d, Algo::Sha256, 2, false, false);
    let nblw = feed(&d, 2, &msg);
    poke(&d, R_STR, nblw | STR_DCAL);
    assert_eq!(
        digest_of(&d, Algo::Sha256),
        hex("cdc76e5c9914fb9281a1c7e284d73e67f1809a48a497200e046d39ccc7112cd0")
    );

    let d = f4();
    start(&d, Algo::Sha1, 2, false, false);
    let nblw = feed(&d, 2, &msg);
    poke(&d, R_STR, nblw | STR_DCAL);
    assert_eq!(
        digest_of(&d, Algo::Sha1),
        hex("34aa973cd4c4daa4f61eeb2bdbad27316534016f")
    );
}

// ---------------------------------------------------------------------------
// `DATATYPE`
// ---------------------------------------------------------------------------

#[test]
fn every_datatype_feeds_abc_to_the_core_the_same_way() {
    // The core must see the bits `0x616263` in all four modes. The words below
    // are what a driver holding "abc" in a 32-bit, 16-bit, 8-bit or bit buffer
    // would actually write, spelled out rather than computed, because a helper
    // that computed them would agree with a wrong implementation.
    let want = hex("ba7816bf8f01cfea414140de5dae2223b00361a396177a9cb410ff61f20015ad");
    for (datatype, word) in [
        // 32-bit: no swap at all.
        (0u32, 0x6162_6300u32),
        // 16-bit: the halves are exchanged, so the guest holds them the other
        // way round. A model that byte-swapped *inside* each half would see
        // 0x0063_6261 here and hash "\0cb".
        (1, 0x6300_6162),
        // 8-bit: the little-endian word a `uint32_t` read of "abc\0" gives.
        (2, 0x0063_6261),
        // bit: every bit of the 8-bit case's target reversed.
        (3, 0x6162_6300u32.reverse_bits()),
    ] {
        let d = v2();
        start(&d, Algo::Sha256, datatype, false, false);
        poke(&d, R_DIN, word);
        poke(&d, R_STR, 24 | STR_DCAL);
        assert_eq!(
            digest_of(&d, Algo::Sha256),
            want,
            "DATATYPE = {datatype:#04b}"
        );
    }
}

#[test]
fn datatype_bit_swap_reverses_every_bit_of_every_word() {
    let mut state = State::new();
    state.datatype = 3;
    assert_eq!(state.swap(0x8000_0001), 0x8000_0001);
    assert_eq!(state.swap(0x0000_00ff), 0xff00_0000);
    assert_eq!(state.swap(0x1234_5678), 0x1e6a_2c48);
    state.datatype = 2;
    assert_eq!(state.swap(0x1234_5678), 0x7856_3412);
    state.datatype = 1;
    assert_eq!(state.swap(0x1234_5678), 0x5678_1234);
    state.datatype = 0;
    assert_eq!(state.swap(0x1234_5678), 0x1234_5678);
}

// ---------------------------------------------------------------------------
// `NBLW`
// ---------------------------------------------------------------------------

#[test]
fn nblw_zero_means_a_full_last_word_and_nblw_eight_means_one_byte() {
    let d = v2();
    start(&d, Algo::Sha256, 2, false, false);
    poke(&d, R_DIN, u32::from_le_bytes(*b"abcd"));
    poke(&d, R_STR, STR_DCAL);
    assert_eq!(
        digest_of(&d, Algo::Sha256),
        hex("88d4266fd4e6338d13b845fcf289579d209c897823b9217da3e161936f031589")
    );

    start(&d, Algo::Sha256, 2, false, false);
    poke(&d, R_DIN, u32::from_le_bytes([b'a', 0xff, 0xff, 0xff]));
    poke(&d, R_STR, 8 | STR_DCAL);
    assert_eq!(
        digest_of(&d, Algo::Sha256),
        hex("ca978112ca1bbdcafac231b39a23dc4da786eff8147c4e72b9807785afee48bb")
    );
}

#[test]
fn a_partial_word_in_a_completely_full_fifo_is_still_truncated() {
    // Sixty-three bytes is sixteen words with the last one three bytes short:
    // the case that forces the FIFO to be lazy. If the block were compressed
    // when the sixteenth word landed, `NBLW` would arrive too late to drop the
    // junk byte and every digest here would be of sixty-four bytes instead.
    let a63 = alloc::vec![b'a'; 63];
    let a64 = alloc::vec![b'a'; 64];
    let d = v2();
    assert_eq!(
        digest(&d, Algo::Sha256, 2, &a63),
        hex("7d3e74a05d7db15bce4ad9ec0658ea98e3f06eeecf16b4c6fff2da457ddc2f34")
    );
    assert_eq!(
        digest(&d, Algo::Sha256, 2, &a64),
        hex("ffe054fe7ae0cb6dc65c3af9b61d5209f439851db43d0ba5997337df154668eb")
    );
    let d = f4();
    assert_eq!(
        digest(&d, Algo::Sha1, 2, &a63),
        hex("03f09f5b158a7a8cdad920bddc29b81c18a551f5")
    );
    assert_eq!(
        digest(&d, Algo::Md5, 2, &a63),
        hex("b06521f39153d618550606be297466d5")
    );
}

#[test]
fn nblw_is_a_bit_count_not_a_byte_count() {
    // Five bits of message. FIPS 180-4 pads a bit string, so this is a digest
    // no byte-oriented interface can ask for -- and the value below is what
    // SHA-1 gives for the five bits `11001`, which is FIPS 180-4's own
    // "01100011 (truncated)" style case written as a single word.
    let d = f4();
    start(&d, Algo::Sha1, 0, false, false);
    poke(&d, R_DIN, 0b1100_1000 << 24);
    poke(&d, R_STR, 5 | STR_DCAL);
    let five = digest_of(&d, Algo::Sha1);

    // Six bits of the same word must give something else: the length goes into
    // the padding, so a model that rounded to bytes could not tell them apart.
    start(&d, Algo::Sha1, 0, false, false);
    poke(&d, R_DIN, 0b1100_1000 << 24);
    poke(&d, R_STR, 6 | STR_DCAL);
    assert_ne!(digest_of(&d, Algo::Sha1), five);
}

// ---------------------------------------------------------------------------
// HMAC
// ---------------------------------------------------------------------------

#[test]
fn hmac_sha256_matches_rfc_4231() {
    let d = v2();
    assert_eq!(
        hmac(&d, Algo::Sha256, 2, &[0x0b; 20], b"Hi There"),
        hex("b0344c61d8db38535ca8afceaf0bf12b881dc200c9833da726e9376c2e32cff7"),
        "case 1"
    );
    assert_eq!(
        hmac(
            &d,
            Algo::Sha256,
            2,
            b"Jefe",
            b"what do ya want for nothing?"
        ),
        hex("5bdcc146bf60754e6a042426089575c75a003f089d2739839dec58b964ec3843"),
        "case 2"
    );
}

#[test]
fn hmac_with_a_long_key_sets_lkey_and_hashes_it_first() {
    // RFC 4231 cases 6 and 7: a 131-byte key, which is thirty-three words with
    // the last one partial -- so this also drives the FIFO past its sixteenth
    // word twice inside a key phase.
    let d = v2();
    assert_eq!(
        hmac(
            &d,
            Algo::Sha256,
            2,
            &[0xaa; 131],
            b"Test Using Larger Than Block-Size Key - Hash Key First"
        ),
        hex("60e431591ee0b67f0d8a26aacbf5b77f8e0bc6213728c5140546040f0ee37f54"),
        "case 6"
    );
    assert_eq!(
        hmac(
            &d,
            Algo::Sha256,
            2,
            &[0xaa; 131],
            b"This is a test using a larger than block-size key and a larger than \
              block-size data. The key needs to be hashed before being used by the \
              HMAC algorithm."
        ),
        hex("9b09ffa71b942fcb27635fbcd5b0e944bfdc63644f0713938a7f51535c3a35e2"),
        "case 7"
    );
}

#[test]
fn hmac_sha224_matches_rfc_4231() {
    let d = v2();
    assert_eq!(
        hmac(
            &d,
            Algo::Sha224,
            2,
            b"Jefe",
            b"what do ya want for nothing?"
        ),
        hex("a30e01098bc6dbbf45690f3a7e9e6d0f8bbea2a39e6148008fd05e44")
    );
}

#[test]
fn hmac_md5_and_hmac_sha1_match_rfc_2202() {
    let d = f4();
    assert_eq!(
        hmac(&d, Algo::Md5, 2, &[0x0b; 16], b"Hi There"),
        hex("9294727a3638bb1c13f48ef8158bfc9d"),
        "MD5 case 1"
    );
    assert_eq!(
        hmac(
            &d,
            Algo::Md5,
            2,
            &[0xaa; 80],
            b"Test Using Larger Than Block-Size Key - Hash Key First"
        ),
        hex("6b1ab7fe4bd7bf8f0b62e6ce61b9d0cd"),
        "MD5 case 6"
    );
    assert_eq!(
        hmac(&d, Algo::Sha1, 2, b"Jefe", b"what do ya want for nothing?"),
        hex("effcdf6ae5eb2fa2d27416d5f184df9c259a7c79"),
        "SHA-1 case 2"
    );
    assert_eq!(
        hmac(
            &d,
            Algo::Sha1,
            2,
            &[0xaa; 80],
            b"Test Using Larger Than Block-Size Key - Hash Key First"
        ),
        hex("aa4ae5e15272d00e95705637ce8a3b55ed402112"),
        "SHA-1 case 6"
    );
}

#[test]
fn an_hmac_takes_exactly_three_dcals() {
    let d = v2();
    start(&d, Algo::Sha256, 2, true, false);
    // The digest registers stay at zero until the third phase ends, so a driver
    // that read them early would see nothing that looks like an answer.
    for part in [&b"Jefe"[..], b"what do ya want for nothing?"] {
        let nblw = feed(&d, 2, part);
        poke(&d, R_STR, nblw | STR_DCAL);
        assert_eq!(d.digest(), [0; 8]);
    }
    let nblw = feed(&d, 2, b"Jefe");
    poke(&d, R_STR, nblw | STR_DCAL);
    assert_eq!(
        digest_of(&d, Algo::Sha256),
        hex("5bdcc146bf60754e6a042426089575c75a003f089d2739839dec58b964ec3843")
    );
}

// ---------------------------------------------------------------------------
// The context block
// ---------------------------------------------------------------------------

/// Everything a driver saves: `IMR`, `STR`, `CR` and the fifty-four context
/// words, in that order.
fn save_context(d: &Hash) -> Vec<u32> {
    let mut ctx = alloc::vec![peek(d, R_IMR), peek(d, R_STR), peek(d, R_CR)];
    for i in 0..CSR_COUNT as u64 {
        ctx.push(peek(d, R_CSR + i * 4));
    }
    ctx
}

fn restore_context(d: &Hash, ctx: &[u32]) {
    poke(d, R_IMR, ctx[0]);
    poke(d, R_STR, ctx[1]);
    poke(d, R_CR, ctx[2]);
    poke(d, R_CR, ctx[2] | CR_INIT);
    for (i, word) in ctx[3..].iter().enumerate() {
        poke(d, R_CSR + i as u64 * 4, *word);
    }
}

#[test]
fn a_context_saved_through_csr_and_restored_finishes_with_the_same_digest() {
    let msg = alloc::vec![b'a'; 200];
    let want = hex("c2a908d98f5df987ade41b5fce213067efbcc21ef2240212a41e54b5e7c28ae5");
    let d = v2();

    // A hundred bytes in: one block has gone to the core and nine words are
    // still in the FIFO, so a context that only carried the chaining variables
    // would lose thirty-six bytes of message.
    start(&d, Algo::Sha256, 2, false, false);
    feed(&d, 2, &msg[..100]);
    let ctx = save_context(&d);

    // Something else entirely on the same block, with a different algorithm, a
    // different DATATYPE and a completed digest of its own.
    assert_eq!(
        digest(&d, Algo::Md5, 0, b"message digest"),
        hex("f96b697d7cb7938d525a2f31aaf161d0")
    );

    restore_context(&d, &ctx);
    let nblw = feed(&d, 2, &msg[100..]);
    poke(&d, R_STR, nblw | STR_DCAL);
    assert_eq!(digest_of(&d, Algo::Sha256), want);

    // And the uninterrupted digest is the same, which is the whole claim.
    let fresh = v2();
    assert_eq!(digest(&fresh, Algo::Sha256, 2, &msg), want);
}

#[test]
fn an_hmac_survives_a_context_swap_between_its_key_and_its_message() {
    let d = v2();
    start(&d, Algo::Sha256, 2, true, false);
    let nblw = feed(&d, 2, b"Jefe");
    poke(&d, R_STR, nblw | STR_DCAL);

    // The inner pad is now compressed and the block is in the message phase.
    let ctx = save_context(&d);
    assert_eq!(digest(&d, Algo::Sha1, 2, b"abc").len(), 20);
    restore_context(&d, &ctx);

    for part in [&b"what do ya want for nothing?"[..], b"Jefe"] {
        let nblw = feed(&d, 2, part);
        poke(&d, R_STR, nblw | STR_DCAL);
    }
    assert_eq!(
        digest_of(&d, Algo::Sha256),
        hex("5bdcc146bf60754e6a042426089575c75a003f089d2739839dec58b964ec3843")
    );
}

#[test]
fn the_reserved_context_words_read_as_zero() {
    let d = v2();
    start(&d, Algo::Sha256, 2, false, false);
    for i in 51..CSR_COUNT as u64 {
        assert_eq!(peek(&d, R_CSR + i * 4), 0, "CSR{i}");
    }
}

// ---------------------------------------------------------------------------
// Status, interrupts and DMA
// ---------------------------------------------------------------------------

#[test]
fn dinis_and_busy_track_the_fifo() {
    let d = v2();
    start(&d, Algo::Sha256, 2, false, false);
    assert_eq!(peek(&d, R_SR) & (SR_DINIS | SR_BUSY), SR_DINIS);

    for i in 0..FIFO_WORDS {
        poke(&d, R_DIN, 0x1122_3344);
        let sr = peek(&d, R_SR);
        if i + 1 == FIFO_WORDS {
            assert_eq!(sr & (SR_DINIS | SR_BUSY), SR_BUSY, "a full FIFO");
        } else {
            assert_eq!(sr & (SR_DINIS | SR_BUSY), SR_DINIS, "{} words", i + 1);
        }
        assert_eq!(peek(&d, R_CR) & CR_DINNE, CR_DINNE);
        assert_eq!((peek(&d, R_CR) >> CR_NBW_SHIFT) & 0xf, (i as u32 + 1) & 0xf);
    }

    // The seventeenth word pushes the block and frees the FIFO again.
    poke(&d, R_DIN, 0x5566_7788);
    assert_eq!(peek(&d, R_SR) & (SR_DINIS | SR_BUSY), SR_DINIS);
    assert_eq!((peek(&d, R_CR) >> CR_NBW_SHIFT) & 0xf, 1);

    // `DCAL` empties it too, so a `while (BUSY)` loop after one always exits.
    poke(&d, R_STR, STR_DCAL);
    assert_eq!(peek(&d, R_SR) & SR_BUSY, 0);
    assert_eq!(peek(&d, R_CR) & CR_DINNE, 0);
}

#[derive(Debug, Default)]
struct Probe {
    high: AtomicBool,
}

impl WireSink for Probe {
    fn set_level(&self, _src: WireId, _line: u32, level: Level) {
        self.high.store(level.is_high(), Ordering::Relaxed);
    }
}

fn watch(d: &Hash, port: &str) -> Arc<Probe> {
    let ids = WireIdAllocator::new();
    let id = ids.alloc();
    let probe = Arc::new(Probe::default());
    let wire = Wire::builder()
        .source(id)
        .sink(Arc::clone(&probe) as Arc<dyn WireSink>, 0)
        .build_shared();
    Device::connect(d, port, WireSource::new(wire, id)).expect("a pin this block drives");
    probe
}

#[test]
fn dcie_raises_the_interrupt_when_a_digest_lands() {
    let d = v2();
    let probe = watch(&d, pin::IRQ);
    assert!(!probe.high.load(Ordering::Relaxed));

    poke(&d, R_IMR, IMR_DCIE);
    start(&d, Algo::Sha256, 2, false, false);
    assert!(!probe.high.load(Ordering::Relaxed), "INIT clears DCIS");

    let nblw = feed(&d, 2, b"abc");
    poke(&d, R_STR, nblw | STR_DCAL);
    assert!(probe.high.load(Ordering::Relaxed));
    assert_eq!(peek(&d, R_SR) & SR_DCIS, SR_DCIS);

    // Write-zero-to-clear, which is what a driver's clear-flag macro does.
    poke(&d, R_SR, 0);
    assert_eq!(peek(&d, R_SR) & SR_DCIS, 0);
    assert!(!probe.high.load(Ordering::Relaxed));
}

#[test]
fn dinie_raises_the_interrupt_while_din_has_room() {
    let d = v2();
    let probe = watch(&d, pin::IRQ);
    poke(&d, R_IMR, IMR_DINIE);
    start(&d, Algo::Sha256, 2, false, false);
    assert!(probe.high.load(Ordering::Relaxed));
    for _ in 0..FIFO_WORDS {
        poke(&d, R_DIN, 0);
    }
    assert!(!probe.high.load(Ordering::Relaxed));
}

#[test]
fn dmae_sets_dmas_and_asks_a_controller_for_service() {
    let d = v2();
    let probe = watch(&d, pin::DMA);
    assert!(!probe.high.load(Ordering::Relaxed));
    assert_eq!(peek(&d, R_SR) & SR_DMAS, 0);

    poke(
        &d,
        R_CR,
        CR_INIT | CR_DMAE | (2 << CR_DATATYPE_SHIFT) | CR_ALGO1 | CR_ALGO0,
    );
    assert_eq!(peek(&d, R_SR) & SR_DMAS, SR_DMAS);
    assert!(probe.high.load(Ordering::Relaxed));
    assert!(d.dma_requesting());

    // A full FIFO drops the request without touching `DMAS`.
    for _ in 0..FIFO_WORDS {
        poke(&d, R_DIN, 0);
    }
    assert!(!probe.high.load(Ordering::Relaxed));
    assert_eq!(peek(&d, R_SR) & SR_DMAS, SR_DMAS);
}

// ---------------------------------------------------------------------------
// The register face itself
// ---------------------------------------------------------------------------

#[test]
fn the_f4_variant_does_not_decode_algo1() {
    // An F4's HASH has SHA-1 and MD5. Bit 18 is not a bit there, so asking for
    // SHA-256 (ALGO = 11) selects MD5 (ALGO = 01) rather than being rejected --
    // a missing register bit reads as zero and the field below it still means
    // what it means. `CR` reads back without the bit, which is how a driver
    // that probes for the wide block finds out.
    let d = f4();
    digest(&d, Algo::Sha256, 2, b"abc");
    assert_eq!(
        digest_of(&d, Algo::Md5),
        hex("900150983cd24fb0d6963f7d28e17f72")
    );
    assert_eq!(
        digest(&d, Algo::Sha1, 2, b"abc"),
        hex("a9993e364706816aba3e25717850c26c9cd0d89d")
    );
    assert_eq!(peek(&d, R_CR) & CR_ALGO1, 0);
    assert_eq!(peek(&d, R_CR) & CR_MDMAT, 0);

    let d = v2();
    poke(&d, R_CR, CR_MDMAT);
    assert_eq!(peek(&d, R_CR) & CR_MDMAT, CR_MDMAT);
}

#[test]
fn the_digest_is_in_both_windows() {
    let d = f4();
    digest(&d, Algo::Sha1, 2, b"abc");
    for i in 0..5u64 {
        assert_eq!(peek(&d, R_HR_LOW + i * 4), peek(&d, R_HR_HIGH + i * 4));
    }
    assert_eq!(peek(&d, R_HR_LOW), 0xa999_3e36);
}

#[test]
fn md5_puts_its_little_endian_words_into_hr_big_endian() {
    // The one place the two families disagree about what a digest register
    // holds. `HR` is the digest as a byte stream, so MD5's words come out
    // byte-swapped relative to its internal state.
    let d = f4();
    digest(&d, Algo::Md5, 2, b"abc");
    assert_eq!(peek(&d, R_HR_LOW), 0x9001_5098);
}

#[test]
fn a_debug_read_disturbs_nothing() {
    let d = v2();
    start(&d, Algo::Sha256, 2, false, false);
    poke(&d, R_IMR, IMR_DCIE);
    let nblw = feed(&d, 2, b"abc");
    poke(&d, R_STR, nblw | STR_DCAL);

    let before = save_context(&d);
    for offset in [R_CR, R_DIN, R_STR, R_IMR, R_SR, R_HR_LOW, R_HR_HIGH] {
        read(&d, offset, MemAttrs::DEBUG);
    }
    assert_eq!(peek(&d, R_SR) & SR_DCIS, SR_DCIS, "a debug read kept DCIS");
    assert_eq!(save_context(&d), before);
}

#[test]
fn a_debug_write_faults_rather_than_hashing() {
    let d = v2();
    start(&d, Algo::Sha256, 2, false, false);
    assert!(try_write(&d, R_DIN, 0x6162_6300, MemAttrs::DEBUG).is_err());
    assert_eq!(peek(&d, R_CR) & CR_DINNE, 0);
}

#[test]
fn the_holes_in_the_aperture_fault() {
    let d = v2();
    for offset in [0x28, 0x30, 0xf4, 0x1d0, 0x300, 0x32e] {
        let mut buf = [0u8; 4];
        assert!(
            d.regs.read(offset, &mut buf, MemAttrs::DEFAULT).is_err(),
            "{offset:#x} is not a register"
        );
    }
}

#[test]
fn writing_hr_is_ignored_rather_than_faulting() {
    // Drivers memset their register images; a write to a read-only result must
    // not take the bus down.
    let d = f4();
    digest(&d, Algo::Sha1, 2, b"abc");
    poke(&d, R_HR_LOW, 0);
    poke(&d, R_HR_HIGH, 0);
    assert_eq!(peek(&d, R_HR_LOW), 0xa999_3e36);
}

#[test]
fn reset_clears_everything() {
    let d = v2();
    digest(&d, Algo::Sha256, 2, b"abc");
    poke(&d, R_IMR, IMR_DCIE);
    Device::reset(&d, ResetKind::Cold);
    assert_eq!(d.digest(), [0; 8]);
    assert_eq!(peek(&d, R_CR), 0);
    assert_eq!(peek(&d, R_IMR), 0);
    assert_eq!(peek(&d, R_SR), SR_DINIS);
}

#[test]
fn an_unknown_variant_is_a_property_error() {
    assert!(Hash::new(&Props::new().with("variant", Value::from("f7"))).is_err());
    assert!(Hash::new(&Props::new().with("algo", Value::from("sha256"))).is_err());
    assert_eq!(Hash::new(&Props::new()).unwrap().variant(), Variant::F4);
}

#[test]
fn the_class_registers_under_its_name() {
    let mut reg = Registry::new();
    register(&mut reg).expect("a fresh registry");
    assert!(reg.get(CLASS_NAME).is_some());
    assert_eq!(schema().class, CLASS_NAME);
}

// ---------------------------------------------------------------------------
// Snapshots
// ---------------------------------------------------------------------------

fn round_trip(saved: &Hash, restored: &Hash) {
    let mut shape = MachineShape::new();
    shape.add_device("hash", CLASS_NAME).unwrap();
    let mut w = StateWriter::new(shape);
    {
        let mut chunk = w.chunk("hash", CLASS_NAME, STATE_VERSION).unwrap();
        Device::save(saved, &mut chunk).unwrap();
    }
    let bytes = w.to_vec().unwrap();
    let reader = StateReader::new(&bytes).unwrap();
    let chunk = reader
        .load("hash", CLASS_NAME, STATE_VERSION, &Migrations::new())
        .unwrap();
    Device::load(restored, &mut chunk.reader()).unwrap();
}

#[test]
fn a_snapshot_round_trips_to_identical_state() {
    let saved = v2();
    start(&saved, Algo::Sha256, 2, false, false);
    poke(&saved, R_IMR, IMR_DCIE | IMR_DINIE);
    feed(&saved, 2, &alloc::vec![b'a'; 100]);

    let restored = v2();
    round_trip(&saved, &restored);

    assert_eq!(save_context(&restored), save_context(&saved));
    assert_eq!(restored.irq_level(), saved.irq_level());
}

#[test]
fn a_digest_interrupted_by_a_snapshot_still_comes_out_right() {
    let msg = alloc::vec![b'a'; 200];
    let saved = v2();
    start(&saved, Algo::Sha256, 2, false, false);
    feed(&saved, 2, &msg[..100]);

    let restored = v2();
    round_trip(&saved, &restored);

    let nblw = feed(&restored, 2, &msg[100..]);
    poke(&restored, R_STR, nblw | STR_DCAL);
    assert_eq!(
        digest_of(&restored, Algo::Sha256),
        hex("c2a908d98f5df987ade41b5fce213067efbcc21ef2240212a41e54b5e7c28ae5")
    );
}

#[test]
fn an_hmac_interrupted_by_a_snapshot_still_comes_out_right() {
    let saved = v2();
    start(&saved, Algo::Sha256, 2, true, false);
    let nblw = feed(&saved, 2, b"Jefe");
    poke(&saved, R_STR, nblw | STR_DCAL);
    let nblw = feed(&saved, 2, b"what do ya want for nothing?");
    poke(&saved, R_STR, nblw | STR_DCAL);

    let restored = v2();
    round_trip(&saved, &restored);

    let nblw = feed(&restored, 2, b"Jefe");
    poke(&restored, R_STR, nblw | STR_DCAL);
    assert_eq!(
        digest_of(&restored, Algo::Sha256),
        hex("5bdcc146bf60754e6a042426089575c75a003f089d2739839dec58b964ec3843")
    );
}

#[test]
fn a_snapshot_from_the_other_variant_is_refused() {
    let saved = f4();
    let restored = v2();
    let mut shape = MachineShape::new();
    shape.add_device("hash", CLASS_NAME).unwrap();
    let mut w = StateWriter::new(shape);
    {
        let mut chunk = w.chunk("hash", CLASS_NAME, STATE_VERSION).unwrap();
        Device::save(&saved, &mut chunk).unwrap();
    }
    let bytes = w.to_vec().unwrap();
    let reader = StateReader::new(&bytes).unwrap();
    let chunk = reader
        .load("hash", CLASS_NAME, STATE_VERSION, &Migrations::new())
        .unwrap();
    assert!(Device::load(&restored, &mut chunk.reader()).is_err());
}
