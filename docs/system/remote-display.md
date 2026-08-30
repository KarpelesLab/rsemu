# Remote display protocols

Consumed by: `host/vnc` (and later SPICE), phase 10. Remote display is the
highest-value frontend: no GUI dependencies, works over a network, and doubles
as the CI screenshot mechanism.

## RFB (VNC)

| Source | Covers |
| --- | --- |
| [RFC 6143 — The Remote Framebuffer Protocol](https://datatracker.ietf.org/doc/html/rfc6143) | The protocol as an IETF standard: handshake, security types, pixel formats, framebuffer updates, input events |
| [rfbproto](https://github.com/rfbproto/rfbproto) | The living community specification, documenting the encodings and pseudo-encodings that came after RFC 6143 |

RFC 6143 is enough to build an interoperable server. The extended encodings
(Tight, ZRLE) are where the compression wins are, and `compcol` already supplies
the zlib codec they need.

## SPICE

| Source | Covers |
| --- | --- |
| [spice-space.org protocol documentation](https://www.spice-space.org/spice-protocol.html) | Channel structure, display/input/cursor/playback channels |

Richer than RFB — multiple channels, audio, better cursor handling — and there
is existing in-house work in `../spice` and `../shells-spice` to draw on.

## Implementation notes

- Encoding runs on the task pool, off the emulation thread, driven by the
  region-level dirty tracking from §4.1. It must never stall the guest.
- TLS for either protocol comes from `purecrypto`.
- For CI, the same scanout path renders headlessly to a PNG and the frame hash
  is compared — the remote display server and the regression harness share
  everything below the encoder.
