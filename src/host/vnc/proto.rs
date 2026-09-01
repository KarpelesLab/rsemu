//! RFB messages: the bytes, and nothing else (RFC 6143).
//!
//! No sockets, no machine, no `Surface` — the same split
//! [`gdb::packet`](crate::host::gdb::packet) has, and for the same reason: the
//! protocol is then testable against a `Vec<u8>` and the session loop is
//! testable against a fake protocol.
//!
//! Every message here cites the section of RFC 6143 that defines it. The RFC is
//! an openly published Informational document describing a wire format; nothing
//! in this file came from an implementation of it (`ROADMAP.md` §1).
//!
//! # What is implemented
//!
//! | § | Message | Direction |
//! | --- | --- | --- |
//! | 7.1.1 | ProtocolVersion | both |
//! | 7.1.2 | Security | both |
//! | 7.1.3 | SecurityResult | server → client |
//! | 7.3.1 | ClientInit | client → server |
//! | 7.3.2 | ServerInit | server → client |
//! | 7.4 | PIXEL_FORMAT | inside both |
//! | 7.5.1 | SetPixelFormat | client → server |
//! | 7.5.2 | SetEncodings | client → server |
//! | 7.5.3 | FramebufferUpdateRequest | client → server |
//! | 7.5.4 | KeyEvent | client → server |
//! | 7.5.5 | PointerEvent | client → server |
//! | 7.5.6 | ClientCutText | client → server |
//! | 7.6.1 | FramebufferUpdate | server → client |
//! | 7.6.3 | Bell | server → client |
//! | 7.7.1 | Raw encoding | inside 7.6.1 |
//! | 7.8.2 | DesktopSize pseudo-encoding | inside 7.6.1 |
//!
//! # What is not, and why
//!
//! * **Every security type except `None` (7.2.1).** VNC Authentication (7.2.2)
//!   is a DES challenge on a password truncated to eight characters, which is
//!   not security and pretending otherwise would be worse than not offering it.
//!   The server binds the loopback interface unless told otherwise, exactly as
//!   the gdbstub does, and that is the actual protection.
//! * **Compressed encodings (7.7.2 onwards).** Zlib, Tight and ZRLE each need a
//!   zlib stream held open for the life of the connection with a sync flush at
//!   every rectangle boundary. `compcol` supplies exactly that — its
//!   `zlib::Encoder` sits on a `RawEncoder` whose `raw_flush` takes a `Flush`
//!   mode — so this is missing *work*, not a capability, and adding it is also
//!   a decision to put `compcol` in this feature's dependency tree. Raw is what
//!   §7.7.1 obliges every client to support, so Raw is what a first server owes
//!   them; on the loopback socket where a frontend of this kind is actually
//!   used, the difference is memory bandwidth rather than latency.
//! * **SetColourMapEntries (7.6.2).** Only a colour-map pixel format needs it,
//!   and this server refuses one — see [`PixelFormat::is_supported`].

use alloc::string::String;
use alloc::vec::Vec;

// ---------------------------------------------------------------------------
// version
// ---------------------------------------------------------------------------

/// The ProtocolVersion string this server offers (RFC 6143 §7.1.1).
///
/// Twelve bytes, always: `RFB xxx.yyy\n` with zero-padded three-digit fields.
pub const VERSION_3_8: &[u8; 12] = b"RFB 003.008\n";

/// How many bytes a ProtocolVersion message is.
pub const VERSION_LEN: usize = 12;

/// A protocol version the client asked for.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub struct Version {
    /// Major, always 3 in practice.
    pub major: u32,
    /// Minor: 3, 7 or 8 are the ones that exist.
    pub minor: u32,
}

impl Version {
    /// RFB 3.3, the original: no security-type negotiation.
    pub const V3_3: Version = Version { major: 3, minor: 3 };
    /// RFB 3.7: the client picks a security type; `None` skips SecurityResult.
    pub const V3_7: Version = Version { major: 3, minor: 7 };
    /// RFB 3.8: `None` still gets a SecurityResult, and failures carry a
    /// reason string.
    pub const V3_8: Version = Version { major: 3, minor: 8 };

    /// Parse the twelve bytes of §7.1.1.
    ///
    /// A version this server does not know is clamped *down* to the nearest one
    /// it does, which is what §7.1.1 tells both ends to do: "the server may
    /// assume that the client is using the highest version it supports".
    #[must_use]
    pub fn parse(bytes: &[u8]) -> Option<Version> {
        if bytes.len() < VERSION_LEN || &bytes[..4] != b"RFB " || bytes[7] != b'.' {
            return None;
        }
        let digits = |s: &[u8]| -> Option<u32> {
            let mut n = 0u32;
            for b in s {
                if !b.is_ascii_digit() {
                    return None;
                }
                n = n * 10 + u32::from(b - b'0');
            }
            Some(n)
        };
        let major = digits(&bytes[4..7])?;
        let minor = digits(&bytes[8..11])?;
        let clamped = if major > 3 || (major == 3 && minor >= 8) {
            Version::V3_8
        } else if major == 3 && minor >= 7 {
            Version::V3_7
        } else {
            Version::V3_3
        };
        Some(clamped)
    }
}

// ---------------------------------------------------------------------------
// security
// ---------------------------------------------------------------------------

/// Security type `None`: no authentication (RFC 6143 §7.2.1).
pub const SECURITY_NONE: u8 = 1;

/// Security type `Invalid`: the handshake failed (§7.1.2).
pub const SECURITY_INVALID: u8 = 0;

/// The security handshake a 3.7-or-later client gets: one type, `None`.
#[must_use]
pub fn security_types() -> Vec<u8> {
    alloc::vec![1u8, SECURITY_NONE]
}

/// The 3.3 form of the same thing: the server simply states the type as a
/// `u32` and the client has no say (§7.1.2).
#[must_use]
pub fn security_type_3_3() -> Vec<u8> {
    u32::from(SECURITY_NONE).to_be_bytes().to_vec()
}

/// SecurityResult: OK (§7.1.3).
#[must_use]
pub fn security_result_ok() -> Vec<u8> {
    0u32.to_be_bytes().to_vec()
}

/// SecurityResult: failed, with the 3.8 reason string (§7.1.3).
#[must_use]
pub fn security_result_failed(reason: &str) -> Vec<u8> {
    let mut out = 1u32.to_be_bytes().to_vec();
    #[allow(clippy::cast_possible_truncation)]
    out.extend_from_slice(&(reason.len() as u32).to_be_bytes());
    out.extend_from_slice(reason.as_bytes());
    out
}

// ---------------------------------------------------------------------------
// pixel format
// ---------------------------------------------------------------------------

/// How many bytes a PIXEL_FORMAT occupies on the wire (§7.4).
pub const PIXEL_FORMAT_LEN: usize = 16;

/// The RFB PIXEL_FORMAT structure (§7.4).
///
/// Distinct from [`display::PixelFormat`](crate::host::display::PixelFormat),
/// which names a *memory layout* out of a short list. This one is what the
/// protocol actually carries: an arbitrary true-colour packing the client is
/// allowed to invent, and which the server has to honour rather than choose.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct PixelFormat {
    /// Bits per pixel: 8, 16 or 32.
    pub bits_per_pixel: u8,
    /// How many of those bits are colour.
    pub depth: u8,
    /// Whether the pixel value is big-endian on the wire.
    pub big_endian: bool,
    /// Whether the value is RGB rather than an index into a colour map.
    pub true_colour: bool,
    /// The largest red value; also its mask, since it is always `2^n - 1`.
    pub red_max: u16,
    /// The largest green value.
    pub green_max: u16,
    /// The largest blue value.
    pub blue_max: u16,
    /// How far left to shift red.
    pub red_shift: u8,
    /// How far left to shift green.
    pub green_shift: u8,
    /// How far left to shift blue.
    pub blue_shift: u8,
}

impl PixelFormat {
    /// What the server offers in ServerInit: 32-bit true colour, little-endian,
    /// eight bits each with red highest.
    ///
    /// That is `0x00RRGGBB` in a little-endian word, which is the bytes `B G R
    /// x` in memory — the same order
    /// [`display::PixelFormat::BGRA8888`](crate::host::display::PixelFormat::BGRA8888)
    /// names, so the common case is a row copy rather than a per-pixel repack.
    /// Every client understands it, and most ask for exactly it back.
    pub const DEFAULT: PixelFormat = PixelFormat {
        bits_per_pixel: 32,
        depth: 24,
        big_endian: false,
        true_colour: true,
        red_max: 255,
        green_max: 255,
        blue_max: 255,
        red_shift: 16,
        green_shift: 8,
        blue_shift: 0,
    };

    /// Parse the sixteen bytes of §7.4.
    #[must_use]
    pub fn parse(bytes: &[u8]) -> Option<PixelFormat> {
        if bytes.len() < PIXEL_FORMAT_LEN {
            return None;
        }
        Some(PixelFormat {
            bits_per_pixel: bytes[0],
            depth: bytes[1],
            big_endian: bytes[2] != 0,
            true_colour: bytes[3] != 0,
            red_max: u16::from_be_bytes([bytes[4], bytes[5]]),
            green_max: u16::from_be_bytes([bytes[6], bytes[7]]),
            blue_max: u16::from_be_bytes([bytes[8], bytes[9]]),
            red_shift: bytes[10],
            green_shift: bytes[11],
            blue_shift: bytes[12],
            // Bytes 13..16 are padding.
        })
    }

    /// The sixteen bytes of §7.4.
    #[must_use]
    pub fn encode(self) -> [u8; PIXEL_FORMAT_LEN] {
        let r = self.red_max.to_be_bytes();
        let g = self.green_max.to_be_bytes();
        let b = self.blue_max.to_be_bytes();
        [
            self.bits_per_pixel,
            self.depth,
            u8::from(self.big_endian),
            u8::from(self.true_colour),
            r[0],
            r[1],
            g[0],
            g[1],
            b[0],
            b[1],
            self.red_shift,
            self.green_shift,
            self.blue_shift,
            0,
            0,
            0,
        ]
    }

    /// How many bytes one pixel occupies.
    #[inline]
    #[must_use]
    pub const fn bytes_per_pixel(self) -> usize {
        (self.bits_per_pixel as usize).div_ceil(8)
    }

    /// Whether this server can produce pixels in this format.
    ///
    /// True colour at 8, 16 or 32 bits, and nothing else. A colour-map format
    /// would need the server to choose a palette and send
    /// SetColourMapEntries — a real feature, for a client that asked for a
    /// depth this one never will.
    #[must_use]
    pub const fn is_supported(self) -> bool {
        self.true_colour
            && matches!(self.bits_per_pixel, 8 | 16 | 32)
            && self.red_max > 0
            && self.green_max > 0
            && self.blue_max > 0
    }

    /// Pack one 8-bit-per-channel colour into this format's pixel value.
    ///
    /// The channel is scaled by `max`, not shifted: a client asking for
    /// `red-max = 31` wants a 5-bit red, and `value * 31 / 255` is the rounding
    /// the RFC's own description implies. Integer arithmetic throughout — the
    /// determinism rule is about the *time* path, but a frame hash is compared
    /// too, and a float here would make it host-dependent.
    #[inline]
    #[must_use]
    pub fn pack(self, rgb: [u8; 3]) -> u32 {
        let scale = |v: u8, max: u16| -> u32 { (u32::from(v) * u32::from(max) + 127) / 255 };
        (scale(rgb[0], self.red_max) << self.red_shift)
            | (scale(rgb[1], self.green_max) << self.green_shift)
            | (scale(rgb[2], self.blue_max) << self.blue_shift)
    }

    /// Append one packed pixel to `out` in this format's byte order.
    #[inline]
    pub fn put(self, value: u32, out: &mut Vec<u8>) {
        let bytes = if self.big_endian {
            value.to_be_bytes()
        } else {
            value.to_le_bytes()
        };
        match self.bits_per_pixel {
            8 => out.push(if self.big_endian { bytes[3] } else { bytes[0] }),
            16 => {
                if self.big_endian {
                    out.extend_from_slice(&bytes[2..4]);
                } else {
                    out.extend_from_slice(&bytes[0..2]);
                }
            }
            _ => out.extend_from_slice(&bytes),
        }
    }
}

// ---------------------------------------------------------------------------
// initialisation
// ---------------------------------------------------------------------------

/// ServerInit (§7.3.2): geometry, the server's preferred pixel format, and the
/// desktop name.
#[must_use]
pub fn server_init(width: u16, height: u16, format: PixelFormat, name: &str) -> Vec<u8> {
    let mut out = Vec::with_capacity(24 + name.len());
    out.extend_from_slice(&width.to_be_bytes());
    out.extend_from_slice(&height.to_be_bytes());
    out.extend_from_slice(&format.encode());
    #[allow(clippy::cast_possible_truncation)]
    out.extend_from_slice(&(name.len() as u32).to_be_bytes());
    out.extend_from_slice(name.as_bytes());
    out
}

// ---------------------------------------------------------------------------
// client messages
// ---------------------------------------------------------------------------

/// A message type byte a client sends (§7.5).
pub mod client_msg {
    /// SetPixelFormat (§7.5.1).
    pub const SET_PIXEL_FORMAT: u8 = 0;
    /// SetEncodings (§7.5.2).
    pub const SET_ENCODINGS: u8 = 2;
    /// FramebufferUpdateRequest (§7.5.3).
    pub const FRAMEBUFFER_UPDATE_REQUEST: u8 = 3;
    /// KeyEvent (§7.5.4).
    pub const KEY_EVENT: u8 = 4;
    /// PointerEvent (§7.5.5).
    pub const POINTER_EVENT: u8 = 5;
    /// ClientCutText (§7.5.6).
    pub const CLIENT_CUT_TEXT: u8 = 6;
}

/// A message type byte the server sends (§7.6).
pub mod server_msg {
    /// FramebufferUpdate (§7.6.1).
    pub const FRAMEBUFFER_UPDATE: u8 = 0;
    /// SetColourMapEntries (§7.6.2). Never sent; here so a reader can see the
    /// number is spoken for.
    pub const SET_COLOUR_MAP_ENTRIES: u8 = 1;
    /// Bell (§7.6.3).
    pub const BELL: u8 = 2;
    /// ServerCutText (§7.6.4).
    pub const SERVER_CUT_TEXT: u8 = 3;
}

/// An encoding number (§7.7, §7.8).
pub mod encoding {
    /// Raw (§7.7.1). Every client must support it, so it is the one this
    /// server sends.
    pub const RAW: i32 = 0;
    /// CopyRect (§7.7.2).
    pub const COPY_RECT: i32 = 1;
    /// The DesktopSize pseudo-encoding (§7.8.2): a rectangle whose width and
    /// height are the framebuffer's new size and whose data is empty.
    pub const DESKTOP_SIZE: i32 = -223;
    /// The Cursor pseudo-encoding (§7.8.1).
    pub const CURSOR: i32 = -239;
}

/// One decoded client message.
#[derive(Debug, Clone, PartialEq, Eq)]
#[non_exhaustive]
pub enum ClientMessage {
    /// §7.5.1: from now on, send pixels like this.
    SetPixelFormat(PixelFormat),
    /// §7.5.2: the encodings the client understands, best first.
    SetEncodings(Vec<i32>),
    /// §7.5.3: send me this rectangle, changed-only if `incremental`.
    UpdateRequest {
        /// Whether only changes since the last update are wanted.
        incremental: bool,
        /// Left edge.
        x: u16,
        /// Top edge.
        y: u16,
        /// Width.
        width: u16,
        /// Height.
        height: u16,
    },
    /// §7.5.4: a key went down or came up. `key` is an X11 keysym.
    Key {
        /// The keysym.
        key: u32,
        /// Down, rather than up.
        down: bool,
    },
    /// §7.5.5: the pointer is here with these buttons held.
    Pointer {
        /// Pixels from the left.
        x: u16,
        /// Pixels from the top.
        y: u16,
        /// The RFB button mask.
        buttons: u8,
    },
    /// §7.5.6: the client's clipboard changed. Latin-1, per the RFC.
    CutText(String),
}

/// What [`parse_client`] concluded.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum Parsed {
    /// A whole message, and how many bytes it took.
    Message(ClientMessage, usize),
    /// Not enough bytes yet. Wait for more.
    Incomplete,
    /// A message type this server does not know.
    ///
    /// Fatal, and deliberately so: RFB has no length prefix on a client
    /// message, so an unknown type means the stream cannot be resynchronised —
    /// there is no way to know where the next message starts. Closing the
    /// connection is the only honest response.
    Unknown(u8),
}

/// Decode one client message from the front of `bytes` (§7.5).
#[must_use]
pub fn parse_client(bytes: &[u8]) -> Parsed {
    let Some(&kind) = bytes.first() else {
        return Parsed::Incomplete;
    };
    let need = |n: usize| bytes.len() >= n;
    match kind {
        client_msg::SET_PIXEL_FORMAT => {
            if !need(20) {
                return Parsed::Incomplete;
            }
            match PixelFormat::parse(&bytes[4..20]) {
                Some(format) => Parsed::Message(ClientMessage::SetPixelFormat(format), 20),
                None => Parsed::Incomplete,
            }
        }
        client_msg::SET_ENCODINGS => {
            if !need(4) {
                return Parsed::Incomplete;
            }
            let count = usize::from(u16::from_be_bytes([bytes[2], bytes[3]]));
            let total = 4 + count * 4;
            if !need(total) {
                return Parsed::Incomplete;
            }
            let mut list = Vec::with_capacity(count);
            for i in 0..count {
                let at = 4 + i * 4;
                list.push(i32::from_be_bytes([
                    bytes[at],
                    bytes[at + 1],
                    bytes[at + 2],
                    bytes[at + 3],
                ]));
            }
            Parsed::Message(ClientMessage::SetEncodings(list), total)
        }
        client_msg::FRAMEBUFFER_UPDATE_REQUEST => {
            if !need(10) {
                return Parsed::Incomplete;
            }
            Parsed::Message(
                ClientMessage::UpdateRequest {
                    incremental: bytes[1] != 0,
                    x: u16::from_be_bytes([bytes[2], bytes[3]]),
                    y: u16::from_be_bytes([bytes[4], bytes[5]]),
                    width: u16::from_be_bytes([bytes[6], bytes[7]]),
                    height: u16::from_be_bytes([bytes[8], bytes[9]]),
                },
                10,
            )
        }
        client_msg::KEY_EVENT => {
            if !need(8) {
                return Parsed::Incomplete;
            }
            Parsed::Message(
                ClientMessage::Key {
                    key: u32::from_be_bytes([bytes[4], bytes[5], bytes[6], bytes[7]]),
                    down: bytes[1] != 0,
                },
                8,
            )
        }
        client_msg::POINTER_EVENT => {
            if !need(6) {
                return Parsed::Incomplete;
            }
            Parsed::Message(
                ClientMessage::Pointer {
                    buttons: bytes[1],
                    x: u16::from_be_bytes([bytes[2], bytes[3]]),
                    y: u16::from_be_bytes([bytes[4], bytes[5]]),
                },
                6,
            )
        }
        client_msg::CLIENT_CUT_TEXT => {
            if !need(8) {
                return Parsed::Incomplete;
            }
            let len = u32::from_be_bytes([bytes[4], bytes[5], bytes[6], bytes[7]]) as usize;
            let total = 8 + len;
            if !need(total) {
                return Parsed::Incomplete;
            }
            // §7.5.6 says Latin-1. Every byte is a code point, so this cannot
            // fail — which is why it is a map rather than a `from_utf8`.
            let text: String = bytes[8..total].iter().map(|&b| char::from(b)).collect();
            Parsed::Message(ClientMessage::CutText(text), total)
        }
        other => Parsed::Unknown(other),
    }
}

// ---------------------------------------------------------------------------
// server messages
// ---------------------------------------------------------------------------

/// The header of a FramebufferUpdate carrying `rects` rectangles (§7.6.1).
#[must_use]
pub fn update_header(rects: u16) -> [u8; 4] {
    let n = rects.to_be_bytes();
    [server_msg::FRAMEBUFFER_UPDATE, 0, n[0], n[1]]
}

/// One rectangle header (§7.6.1): position, size, encoding.
#[must_use]
pub fn rect_header(x: u16, y: u16, width: u16, height: u16, encoding: i32) -> [u8; 12] {
    let x = x.to_be_bytes();
    let y = y.to_be_bytes();
    let w = width.to_be_bytes();
    let h = height.to_be_bytes();
    let e = encoding.to_be_bytes();
    [
        x[0], x[1], y[0], y[1], w[0], w[1], h[0], h[1], e[0], e[1], e[2], e[3],
    ]
}

/// Bell (§7.6.3): one byte.
#[must_use]
pub fn bell() -> [u8; 1] {
    [server_msg::BELL]
}

/// ServerCutText (§7.6.4). Latin-1, so anything outside it is dropped rather
/// than mangled into a different character.
#[must_use]
pub fn server_cut_text(text: &str) -> Vec<u8> {
    let latin1: Vec<u8> = text
        .chars()
        .filter_map(|c| u8::try_from(u32::from(c)).ok())
        .collect();
    let mut out = Vec::with_capacity(8 + latin1.len());
    out.push(server_msg::SERVER_CUT_TEXT);
    out.extend_from_slice(&[0, 0, 0]);
    #[allow(clippy::cast_possible_truncation)]
    out.extend_from_slice(&(latin1.len() as u32).to_be_bytes());
    out.extend_from_slice(&latin1);
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_version_string_is_clamped_to_one_we_speak() {
        assert_eq!(Version::parse(b"RFB 003.008\n"), Some(Version::V3_8));
        assert_eq!(Version::parse(b"RFB 003.007\n"), Some(Version::V3_7));
        assert_eq!(Version::parse(b"RFB 003.003\n"), Some(Version::V3_3));
        // §7.1.1: a client claiming more than we offer gets our best.
        assert_eq!(Version::parse(b"RFB 004.001\n"), Some(Version::V3_8));
        // 3.5 was never a thing; the RFC says treat it as 3.3.
        assert_eq!(Version::parse(b"RFB 003.005\n"), Some(Version::V3_3));
        assert_eq!(Version::parse(b"not a version"), None);
        assert_eq!(Version::parse(b"RFB 003.00"), None, "short");
        assert_eq!(Version::parse(b"RFB 0x3.008\n"), None, "not digits");
    }

    #[test]
    fn the_default_pixel_format_survives_its_own_encoding() {
        let bytes = PixelFormat::DEFAULT.encode();
        assert_eq!(bytes.len(), PIXEL_FORMAT_LEN);
        assert_eq!(PixelFormat::parse(&bytes), Some(PixelFormat::DEFAULT));
        assert!(PixelFormat::DEFAULT.is_supported());
        assert_eq!(PixelFormat::DEFAULT.bytes_per_pixel(), 4);
    }

    #[test]
    fn a_colour_map_format_is_refused() {
        let mut format = PixelFormat::DEFAULT;
        format.true_colour = false;
        assert!(!format.is_supported());
        let mut format = PixelFormat::DEFAULT;
        format.bits_per_pixel = 24;
        assert!(!format.is_supported(), "24bpp is not one of 8, 16, 32");
    }

    #[test]
    fn packing_honours_the_clients_masks_and_byte_order() {
        // The default: 0x00RRGGBB little-endian, so B G R x in memory.
        let mut out = Vec::new();
        let value = PixelFormat::DEFAULT.pack([0x12, 0x34, 0x56]);
        assert_eq!(value, 0x0012_3456);
        PixelFormat::DEFAULT.put(value, &mut out);
        assert_eq!(out, [0x56, 0x34, 0x12, 0x00]);

        // RGB565, big-endian: red 5 bits at 11, green 6 at 5, blue 5 at 0.
        let rgb565 = PixelFormat {
            bits_per_pixel: 16,
            depth: 16,
            big_endian: true,
            true_colour: true,
            red_max: 31,
            green_max: 63,
            blue_max: 31,
            red_shift: 11,
            green_shift: 5,
            blue_shift: 0,
        };
        assert!(rgb565.is_supported());
        assert_eq!(rgb565.bytes_per_pixel(), 2);
        let white = rgb565.pack([0xff, 0xff, 0xff]);
        assert_eq!(white, 0xffff);
        let black = rgb565.pack([0, 0, 0]);
        assert_eq!(black, 0);
        let mut out = Vec::new();
        rgb565.put(white, &mut out);
        assert_eq!(out, [0xff, 0xff]);
    }

    #[test]
    fn every_client_message_round_trips_from_its_own_bytes() {
        // SetPixelFormat
        let mut bytes = alloc::vec![client_msg::SET_PIXEL_FORMAT, 0, 0, 0];
        bytes.extend_from_slice(&PixelFormat::DEFAULT.encode());
        assert_eq!(
            parse_client(&bytes),
            Parsed::Message(ClientMessage::SetPixelFormat(PixelFormat::DEFAULT), 20)
        );

        // SetEncodings: two of them.
        let bytes = alloc::vec![
            client_msg::SET_ENCODINGS,
            0,
            0,
            2,
            0,
            0,
            0,
            0, // Raw
            0xff,
            0xff,
            0xff,
            0x21, // -223, DesktopSize
        ];
        assert_eq!(
            parse_client(&bytes),
            Parsed::Message(
                ClientMessage::SetEncodings(alloc::vec![encoding::RAW, encoding::DESKTOP_SIZE]),
                12
            )
        );

        // FramebufferUpdateRequest
        let bytes = alloc::vec![
            client_msg::FRAMEBUFFER_UPDATE_REQUEST,
            1,
            0,
            0,
            0,
            0,
            0x02,
            0xd0,
            0x01,
            0x90,
        ];
        assert_eq!(
            parse_client(&bytes),
            Parsed::Message(
                ClientMessage::UpdateRequest {
                    incremental: true,
                    x: 0,
                    y: 0,
                    width: 720,
                    height: 400,
                },
                10
            )
        );

        // KeyEvent: 'a' down.
        let bytes = alloc::vec![client_msg::KEY_EVENT, 1, 0, 0, 0, 0, 0, 0x61];
        assert_eq!(
            parse_client(&bytes),
            Parsed::Message(
                ClientMessage::Key {
                    key: 0x61,
                    down: true
                },
                8
            )
        );

        // PointerEvent
        let bytes = alloc::vec![client_msg::POINTER_EVENT, 0b001, 0x01, 0x00, 0x00, 0x40];
        assert_eq!(
            parse_client(&bytes),
            Parsed::Message(
                ClientMessage::Pointer {
                    x: 256,
                    y: 64,
                    buttons: 1
                },
                6
            )
        );

        // ClientCutText
        let mut bytes = alloc::vec![client_msg::CLIENT_CUT_TEXT, 0, 0, 0, 0, 0, 0, 2];
        bytes.extend_from_slice(b"hi");
        assert_eq!(
            parse_client(&bytes),
            Parsed::Message(ClientMessage::CutText(String::from("hi")), 10)
        );
    }

    #[test]
    fn a_half_arrived_message_waits_rather_than_guessing() {
        for prefix in 0..8 {
            let bytes = alloc::vec![client_msg::KEY_EVENT, 1, 0, 0, 0, 0, 0, 0x61];
            assert_eq!(
                parse_client(&bytes[..prefix]),
                Parsed::Incomplete,
                "{prefix} bytes of an 8-byte KeyEvent"
            );
        }
    }

    #[test]
    fn an_unknown_message_type_is_fatal_not_skipped() {
        assert_eq!(parse_client(&[250, 0, 0, 0]), Parsed::Unknown(250));
    }

    #[test]
    fn the_server_headers_are_the_shape_the_rfc_prints() {
        assert_eq!(update_header(1), [0, 0, 0, 1]);
        assert_eq!(
            rect_header(0, 0, 720, 400, encoding::RAW),
            [0, 0, 0, 0, 0x02, 0xd0, 0x01, 0x90, 0, 0, 0, 0]
        );
        assert_eq!(
            rect_header(0, 0, 640, 480, encoding::DESKTOP_SIZE),
            [0, 0, 0, 0, 0x02, 0x80, 0x01, 0xe0, 0xff, 0xff, 0xff, 0x21]
        );
        assert_eq!(bell(), [2]);
        assert_eq!(server_cut_text("ab"), [3, 0, 0, 0, 0, 0, 0, 2, b'a', b'b']);
        assert_eq!(security_types(), [1, SECURITY_NONE]);
        assert_eq!(security_type_3_3(), [0, 0, 0, 1]);
        assert_eq!(security_result_ok(), [0, 0, 0, 0]);
        assert_eq!(
            security_result_failed("no"),
            [0, 0, 0, 1, 0, 0, 0, 2, b'n', b'o']
        );
    }

    /// The fuzz target's properties, on a corpus this build generates — so the
    /// invariant is checked by `cargo test` and not only by a `cargo fuzz` run
    /// nobody remembers to start (`fuzz/fuzz_targets/vnc_proto.rs`).
    ///
    /// A 64-bit xorshift rather than a dependency: the sequence has to be the
    /// same on every host and in every build, or a failure is not reproducible
    /// (`CLAUDE.md`, determinism).
    #[test]
    fn arbitrary_bytes_parse_exactly_or_not_at_all() {
        let mut state: u64 = 0x2545_f491_4f6c_dd1d;
        let mut next = move || {
            state ^= state << 13;
            state ^= state >> 7;
            state ^= state << 17;
            state
        };
        for _ in 0..20_000 {
            let len = (next() % 40) as usize;
            let mut bytes = Vec::with_capacity(len);
            for _ in 0..len {
                // Biased towards the six message types, so most cases are a
                // real message with a hostile body rather than an unknown byte.
                let word = next();
                #[allow(clippy::cast_possible_truncation)]
                bytes.push(if word % 3 == 0 {
                    (word % 7) as u8
                } else {
                    word as u8
                });
            }
            match parse_client(&bytes) {
                Parsed::Incomplete | Parsed::Unknown(_) => {}
                Parsed::Message(message, used) => {
                    assert!(used > 0 && used <= bytes.len(), "{used} of {len}");
                    // Exactly its own bytes parse to exactly itself.
                    assert_eq!(
                        parse_client(&bytes[..used]),
                        Parsed::Message(message, used),
                        "framing is not exact for {bytes:02x?}"
                    );
                    // And no proper prefix is a whole message: a packet split
                    // at the wrong byte must not become a keystroke.
                    for cut in 1..used {
                        assert_eq!(
                            parse_client(&bytes[..cut]),
                            Parsed::Incomplete,
                            "a {cut}-byte prefix of {bytes:02x?} parsed whole"
                        );
                    }
                }
            }
        }
    }

    #[test]
    fn server_init_carries_the_geometry_and_the_name() {
        let bytes = server_init(720, 400, PixelFormat::DEFAULT, "rsemu");
        assert_eq!(&bytes[..4], [0x02, 0xd0, 0x01, 0x90]);
        assert_eq!(&bytes[4..20], PixelFormat::DEFAULT.encode());
        assert_eq!(&bytes[20..24], [0, 0, 0, 5]);
        assert_eq!(&bytes[24..], b"rsemu");
    }
}
