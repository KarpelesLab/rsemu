#![no_main]
//! The RFB client-message parser, which is a network surface on untrusted bytes.
//!
//! `CLAUDE.md` asks for a fuzz target on every MMIO surface, and the reason
//! generalises: this one decodes whatever a stranger on a TCP socket sends,
//! before anything has authenticated them — RFC 6143's security type `None` is
//! all the server offers, and the loopback default is a deployment decision
//! rather than a check in the code. Two of the six client messages carry a
//! length the peer chooses (`SetEncodings`'s `u16` count and `ClientCutText`'s
//! `u32`), and both of them index the same buffer the length came out of.
//!
//! The properties, none of which a unit test can assert across arbitrary input:
//!
//! > **Nothing panics**, whatever the bytes are.
//! >
//! > **Framing is exact.** A message the parser claims consumed `n` bytes must
//! > parse the same way from a buffer holding exactly those `n`, and must not
//! > claim more bytes than it was given. A parser that over-reports would
//! > desynchronise a real stream and start decoding key events out of the
//! > middle of a clipboard.
//! >
//! > **Incompleteness is monotone.** Every proper prefix of a whole message
//! > must parse as `Incomplete` rather than as some other, shorter message —
//! > otherwise a client whose packet was split at the wrong byte gets a
//! > keystroke it never sent.
//! >
//! > **A pixel format survives its own encoding**, for every one of the 2^128
//! > the wire can carry, and packing a colour into a supported one never
//! > panics on a shift the peer chose.
//!
//! The version handshake goes in too: `Version::parse` reads twelve bytes of
//! whatever a peer sends first, and its digit loop is arithmetic on them.

use libfuzzer_sys::fuzz_target;
use rsemu::host::vnc::proto::{self, Parsed, PixelFormat, Version};

fuzz_target!(|data: &[u8]| {
    // The first thing any peer sends (§7.1.1).
    let _ = Version::parse(data);

    // The pixel format a peer may ask for at any time (§7.4, §7.5.1).
    if let Some(format) = PixelFormat::parse(data) {
        let bytes = format.encode();
        assert_eq!(
            PixelFormat::parse(&bytes),
            Some(format),
            "a pixel format must survive its own encoding"
        );
        assert!(format.bytes_per_pixel() <= 4);
        if format.is_supported() {
            let mut out = Vec::new();
            for rgb in [[0, 0, 0], [0xff, 0xff, 0xff], [1, 2, 3]] {
                format.put(format.pack(rgb), &mut out);
            }
            assert_eq!(out.len(), 3 * format.bytes_per_pixel());
        }
    }

    // Client messages, parsed the way the server parses them: one at a time,
    // off the front of a buffer that keeps growing (§7.5).
    let mut rest = data;
    let mut budget = 256;
    while budget > 0 {
        budget -= 1;
        match proto::parse_client(rest) {
            Parsed::Incomplete | Parsed::Unknown(_) => break,
            Parsed::Message(message, used) => {
                assert!(used > 0, "a message must consume something");
                assert!(used <= rest.len(), "a message consumed bytes it was not given");

                // Exactly its own bytes parse to exactly itself.
                assert_eq!(
                    proto::parse_client(&rest[..used]),
                    Parsed::Message(message.clone(), used),
                    "framing is not exact"
                );

                // And every proper prefix of it is incomplete rather than
                // something else. Bounded, so a 4 GiB ClientCutText does not
                // turn this into a timeout: the interesting prefixes are the
                // ones near a field boundary.
                for cut in [1usize, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 19] {
                    if cut < used {
                        assert_eq!(
                            proto::parse_client(&rest[..cut]),
                            Parsed::Incomplete,
                            "a {cut}-byte prefix parsed as something whole"
                        );
                    }
                }

                rest = &rest[used..];
            }
        }
    }
});
