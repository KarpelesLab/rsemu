//! The wire format: `$payload#cs`, acknowledgements, escapes and run-length
//! encoding.
//!
//! Everything here parses bytes that arrived from a socket, so **nothing here
//! may panic**. Every length is checked, every slice access goes through
//! `get`, and a malformed packet becomes a negative acknowledgement rather than
//! an abort. A debugger that takes the emulator down with it is worse than no
//! debugger.
//!
//! # The format, in one paragraph
//!
//! A packet is `$`, a payload, `#`, and two hex digits holding the modulo-256
//! sum of the payload bytes *as they appear on the wire*. The receiver answers
//! `+` (understood) or `-` (resend), until `QStartNoAckMode` turns that off.
//! Inside the payload `#`, `$`, `}` and `*` are escaped as `}` followed by the
//! byte XOR `0x20`, and a run of one byte may be compressed as *byte*, `*`, and
//! a printable count character whose value is the number of **extra** copies
//! plus 29.
//!
//! # Sources
//!
//! The GDB manual's "Remote Protocol" appendix — Overview and Packet
//! Acknowledgment. That is a specification of a wire protocol, published so
//! that programs can speak it; see `docs/system/debug-protocols.md`.

/// The largest payload this stub will assemble from the wire.
///
/// A peer that sends more is not a debugger, and an unbounded buffer fed by a
/// socket is a denial of service. 64 KiB is far above the `PacketSize` the stub
/// advertises, so a well-behaved GDB never comes close.
pub const MAX_PACKET: usize = 64 * 1024;

/// The byte GDB sends to interrupt a running target: Ctrl-C, outside any
/// packet.
pub const INTERRUPT: u8 = 0x03;

/// Positive acknowledgement.
pub const ACK: u8 = b'+';

/// Negative acknowledgement: the packet was corrupt, send it again.
pub const NAK: u8 = b'-';

const HEX: &[u8; 16] = b"0123456789abcdef";

/// The modulo-256 sum a packet is framed with.
#[must_use]
pub fn checksum(data: &[u8]) -> u8 {
    data.iter().fold(0u8, |acc, b| acc.wrapping_add(*b))
}

/// Append `value` as two lowercase hex digits.
pub fn push_hex_u8(out: &mut Vec<u8>, value: u8) {
    out.push(HEX[usize::from(value >> 4)]);
    out.push(HEX[usize::from(value & 0x0f)]);
}

/// Append every byte of `data` as two lowercase hex digits, in order.
pub fn push_hex(out: &mut Vec<u8>, data: &[u8]) {
    for byte in data {
        push_hex_u8(out, *byte);
    }
}

/// Append `value` as hex with no leading zeroes, the way an address is spelled
/// in a packet. Zero is a single `0`.
pub fn push_hex_u64(out: &mut Vec<u8>, value: u64) {
    if value == 0 {
        out.push(b'0');
        return;
    }
    let mut started = false;
    for shift in (0..16u32).rev() {
        let nibble = (value >> (shift * 4)) & 0xf;
        if nibble != 0 {
            started = true;
        }
        if started {
            out.push(HEX[nibble as usize]);
        }
    }
}

/// One hex digit's value, upper or lower case.
#[must_use]
pub fn hex_digit(c: u8) -> Option<u8> {
    match c {
        b'0'..=b'9' => Some(c - b'0'),
        b'a'..=b'f' => Some(c - b'a' + 10),
        b'A'..=b'F' => Some(c - b'A' + 10),
        _ => None,
    }
}

/// Decode an even-length run of hex digits into bytes.
///
/// `None` for an odd length or any non-hex byte — never a partial result, so a
/// caller cannot act on half a register write.
#[must_use]
pub fn hex_decode(text: &[u8]) -> Option<Vec<u8>> {
    if !text.len().is_multiple_of(2) {
        return None;
    }
    let (pairs, rest) = text.as_chunks::<2>();
    if !rest.is_empty() {
        return None;
    }
    let mut out = Vec::with_capacity(pairs.len());
    for [hi, lo] in pairs {
        let hi = hex_digit(*hi)?;
        let lo = hex_digit(*lo)?;
        out.push((hi << 4) | lo);
    }
    Some(out)
}

/// Parse a hex integer with no `0x` prefix, as addresses and lengths appear in
/// packets.
///
/// `None` on an empty string, a non-hex digit, or a value that does not fit in
/// 64 bits. A guest address is a `u64` everywhere in rsemu (`CLAUDE.md`), and
/// silently truncating an over-long one would put a debugger read somewhere the
/// user did not ask for.
#[must_use]
pub fn parse_hex_u64(text: &[u8]) -> Option<u64> {
    if text.is_empty() {
        return None;
    }
    let mut value: u64 = 0;
    for byte in text {
        let digit = hex_digit(*byte)?;
        value = value.checked_mul(16)?.checked_add(u64::from(digit))?;
    }
    Some(value)
}

/// Parse a hex integer that must fit a `usize` — a length or a transfer offset.
#[must_use]
pub fn parse_hex_usize(text: &[u8]) -> Option<usize> {
    usize::try_from(parse_hex_u64(text)?).ok()
}

/// Whether a byte must be escaped inside a payload.
const fn must_escape(byte: u8) -> bool {
    matches!(byte, b'#' | b'$' | b'}' | b'*')
}

/// Escape and run-length-compress a payload into its on-the-wire body.
///
/// Compression is applied only to runs of a byte that needs no escaping, so the
/// escape marker can never end up as the base of a run — which is the one way
/// this transformation could produce something a receiver decodes differently
/// than it was meant.
#[must_use]
pub fn encode_body(payload: &[u8]) -> Vec<u8> {
    let mut out = Vec::with_capacity(payload.len());
    let mut i = 0usize;
    while let Some(&byte) = payload.get(i) {
        if must_escape(byte) {
            out.push(b'}');
            out.push(byte ^ 0x20);
            i += 1;
            continue;
        }
        // How many identical bytes follow this one.
        let mut run = 1usize;
        while payload.get(i + run) == Some(&byte) {
            run += 1;
        }
        out.push(byte);
        let mut extra = run - 1;
        // A run only pays once three copies can be folded away: `*` plus the
        // count character costs two bytes.
        while extra >= 3 {
            let mut take = extra.min(97);
            // The count character is `take + 29`, and `#` (35) and `$` (36)
            // are forbidden there — either would end the packet early. Backing
            // off to 5 is always legal and always makes progress.
            if take == 6 || take == 7 {
                take = 5;
            }
            out.push(b'*');
            // `take` is at most 97, so the sum is at most 126: printable.
            out.push((take as u8).wrapping_add(29));
            extra -= take;
        }
        for _ in 0..extra {
            out.push(byte);
        }
        i += run;
    }
    out
}

/// Undo [`encode_body`]: expand run-length groups and unescape.
///
/// `None` for a body that is malformed — a trailing `}`, a `*` with nothing to
/// repeat, a count character below the 29 bias, or an expansion that would run
/// past [`MAX_PACKET`]. Untrusted input, so every one of those is a rejection
/// rather than a panic.
#[must_use]
pub fn decode_body(body: &[u8]) -> Option<Vec<u8>> {
    let mut out: Vec<u8> = Vec::with_capacity(body.len());
    let mut i = 0usize;
    while let Some(&byte) = body.get(i) {
        match byte {
            b'}' => {
                let escaped = *body.get(i + 1)?;
                if out.len() >= MAX_PACKET {
                    return None;
                }
                out.push(escaped ^ 0x20);
                i += 2;
            }
            b'*' => {
                let count = *body.get(i + 1)?;
                let extra = usize::from(count.checked_sub(29)?);
                let last = *out.last()?;
                if out.len().checked_add(extra)? > MAX_PACKET {
                    return None;
                }
                out.extend(std::iter::repeat_n(last, extra));
                i += 2;
            }
            other => {
                if out.len() >= MAX_PACKET {
                    return None;
                }
                out.push(other);
                i += 1;
            }
        }
    }
    Some(out)
}

/// Frame `payload` as a complete packet and append it to `out`.
pub fn frame(payload: &[u8], out: &mut Vec<u8>) {
    let body = encode_body(payload);
    out.push(b'$');
    out.extend_from_slice(&body);
    out.push(b'#');
    push_hex_u8(out, checksum(&body));
}

/// Something the peer sent that a session has to act on.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum Event {
    /// A complete, checksum-verified packet payload.
    Packet(Vec<u8>),
    /// Ctrl-C outside a packet: stop the target.
    Interrupt,
    /// The peer acknowledged the last packet sent.
    Ack,
    /// The peer wants the last packet again.
    Nak,
    /// A packet arrived corrupt. The session answers [`NAK`].
    Corrupt,
}

/// Where the byte-at-a-time reader has got to.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Phase {
    /// Between packets.
    Idle,
    /// Inside `$…`, accumulating the body.
    Body,
    /// After `#`, reading the two checksum digits.
    Checksum,
}

/// Reassembles packets from a byte stream.
///
/// Byte at a time rather than line at a time, because a socket read can split a
/// packet anywhere — including between the two checksum digits — and a debugger
/// that only works when the network is kind is not a debugger.
#[derive(Debug)]
pub struct Framer {
    phase: Phase,
    body: Vec<u8>,
    digits: [u8; 2],
    have: usize,
    /// Set when the body outgrew [`MAX_PACKET`]; the rest of that packet is
    /// consumed and discarded rather than buffered.
    overflow: bool,
}

impl Default for Framer {
    fn default() -> Self {
        Framer::new()
    }
}

impl Framer {
    /// A framer waiting for the first `$`.
    #[must_use]
    pub fn new() -> Framer {
        Framer {
            phase: Phase::Idle,
            body: Vec::new(),
            digits: [0; 2],
            have: 0,
            overflow: false,
        }
    }

    /// Forget any half-received packet — used when a connection is replaced.
    pub fn reset(&mut self) {
        self.phase = Phase::Idle;
        self.body.clear();
        self.have = 0;
        self.overflow = false;
    }

    /// Feed one byte, and report what it completed.
    pub fn push(&mut self, byte: u8) -> Option<Event> {
        match self.phase {
            Phase::Idle => match byte {
                b'$' => {
                    self.body.clear();
                    self.overflow = false;
                    self.phase = Phase::Body;
                    None
                }
                INTERRUPT => Some(Event::Interrupt),
                ACK => Some(Event::Ack),
                NAK => Some(Event::Nak),
                // Line noise between packets. GDB ignores it, and so must we.
                _ => None,
            },
            Phase::Body => {
                if byte == b'#' {
                    self.phase = Phase::Checksum;
                    self.have = 0;
                    return None;
                }
                // A `$` inside a body means the previous one was lost; restart
                // rather than glue two packets together.
                if byte == b'$' {
                    self.body.clear();
                    self.overflow = false;
                    return None;
                }
                if self.body.len() >= MAX_PACKET {
                    self.overflow = true;
                } else {
                    self.body.push(byte);
                }
                None
            }
            Phase::Checksum => {
                if let Some(slot) = self.digits.get_mut(self.have) {
                    *slot = byte;
                }
                self.have += 1;
                if self.have < 2 {
                    return None;
                }
                self.phase = Phase::Idle;
                let sent = match (hex_digit(self.digits[0]), hex_digit(self.digits[1])) {
                    (Some(hi), Some(lo)) => (hi << 4) | lo,
                    _ => return Some(Event::Corrupt),
                };
                if self.overflow || sent != checksum(&self.body) {
                    return Some(Event::Corrupt);
                }
                match decode_body(&self.body) {
                    Some(payload) => Some(Event::Packet(payload)),
                    None => Some(Event::Corrupt),
                }
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Everything the framer would see for one packet, acknowledgement first.
    fn feed(framer: &mut Framer, bytes: &[u8]) -> Vec<Event> {
        bytes.iter().filter_map(|b| framer.push(*b)).collect()
    }

    #[test]
    fn a_packet_round_trips_through_the_framer() {
        let mut out = Vec::new();
        frame(b"OK", &mut out);
        assert_eq!(out, b"$OK#9a");
        let mut framer = Framer::new();
        assert_eq!(feed(&mut framer, &out), vec![Event::Packet(b"OK".to_vec())]);
    }

    #[test]
    fn the_four_special_bytes_are_escaped_and_come_back() {
        let payload = b"a#b$c}d*e";
        let mut wire = Vec::new();
        frame(payload, &mut wire);
        // None of the four may appear raw in the body.
        let body = &wire[1..wire.len() - 3];
        for special in *b"#$*" {
            assert!(!body.contains(&special), "raw {special:?} in {body:?}");
        }
        let mut framer = Framer::new();
        assert_eq!(
            feed(&mut framer, &wire),
            vec![Event::Packet(payload.to_vec())]
        );
    }

    #[test]
    fn long_runs_compress_and_expand_exactly() {
        // 200 identical bytes: more than one count character can hold, so the
        // encoder has to emit several groups.
        let payload = vec![0x41u8; 200];
        let body = encode_body(&payload);
        assert!(body.len() < 20, "200 bytes compressed to {}", body.len());
        assert_eq!(decode_body(&body).as_deref(), Some(&payload[..]));
    }

    #[test]
    fn every_run_length_round_trips() {
        // The count characters near `#` and `$` are the ones an encoder gets
        // wrong, so walk every length rather than spot-checking.
        for len in 1..=300usize {
            let payload = vec![0x30u8; len];
            let body = encode_body(&payload);
            assert!(
                !body.contains(&b'#') && !body.contains(&b'$'),
                "len {len} produced a packet terminator: {body:?}"
            );
            assert_eq!(
                decode_body(&body).as_deref(),
                Some(&payload[..]),
                "len {len}"
            );
        }
    }

    #[test]
    fn a_bad_checksum_is_reported_not_believed() {
        let mut framer = Framer::new();
        assert_eq!(feed(&mut framer, b"$OK#00"), vec![Event::Corrupt]);
        // And the framer is usable again straight away.
        assert_eq!(
            feed(&mut framer, b"$OK#9a"),
            vec![Event::Packet(b"OK".to_vec())]
        );
    }

    #[test]
    fn a_packet_split_across_reads_still_arrives() {
        let mut framer = Framer::new();
        let mut events = Vec::new();
        for chunk in [&b"$m1"[..], b"0,4", b"#", b"2", b"e"] {
            events.extend(feed(&mut framer, chunk));
        }
        assert_eq!(events, vec![Event::Packet(b"m10,4".to_vec())]);
    }

    #[test]
    fn control_c_between_packets_is_an_interrupt() {
        let mut framer = Framer::new();
        assert_eq!(feed(&mut framer, &[INTERRUPT]), vec![Event::Interrupt]);
    }

    #[test]
    fn acknowledgements_are_events_of_their_own() {
        let mut framer = Framer::new();
        assert_eq!(feed(&mut framer, b"+-"), vec![Event::Ack, Event::Nak]);
    }

    #[test]
    fn malformed_bodies_are_rejected_rather_than_panicking() {
        // A trailing escape, a repeat with nothing to repeat, a count below the
        // bias, and a repeat that would blow the size limit.
        assert_eq!(decode_body(b"abc}"), None);
        assert_eq!(decode_body(b"*!"), None);
        assert_eq!(decode_body(b"a* "), Some(b"aaaa".to_vec()));
        assert_eq!(decode_body(b"a*\x00"), None);
        assert_eq!(decode_body(b"a*"), None);
    }

    #[test]
    fn an_oversized_packet_is_refused_without_allocating_forever() {
        let mut framer = Framer::new();
        framer.push(b'$');
        for _ in 0..(MAX_PACKET + 16) {
            framer.push(b'z');
        }
        framer.push(b'#');
        framer.push(b'0');
        assert_eq!(framer.push(b'0'), Some(Event::Corrupt));
    }

    #[test]
    fn hex_helpers_refuse_what_they_cannot_represent() {
        assert_eq!(parse_hex_u64(b""), None);
        assert_eq!(parse_hex_u64(b"12g4"), None);
        assert_eq!(parse_hex_u64(b"ffffffffffffffff"), Some(u64::MAX));
        assert_eq!(parse_hex_u64(b"1ffffffffffffffff"), None);
        assert_eq!(hex_decode(b"abc"), None);
        assert_eq!(hex_decode(b"00ff"), Some(vec![0x00, 0xff]));
        let mut out = Vec::new();
        push_hex_u64(&mut out, 0);
        push_hex_u64(&mut out, 0xc0de);
        assert_eq!(out, b"0c0de");
    }
}
