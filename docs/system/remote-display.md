# Remote display protocols

Consumed by: `host/vnc` (and later SPICE). Remote display is the
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

## What is built (Phase 9)

`src/host/vnc` is an RFB server implemented from RFC 6143 alone, section by
section, with each message citing the section that defines it. It speaks RFB
3.3, 3.7 and 3.8; security type `None` and nothing else; Raw (§7.7.1) and the
DesktopSize pseudo-encoding (§7.8.2); and it takes `KeyEvent` (§7.5.4) and
`PointerEvent` (§7.5.5) back. `rsemu run <machine> --vnc :5900` is the frontend;
`tests/vnc_protocol.rs` is a client speaking the whole handshake to a real VGA
adapter's framebuffer, and it will also drive an installed viewer binary if
there is one.

Input lands in a **sink** chosen by what the machine opened rather than by a
flag: a character port named `keyboard` is what `pc.kbc` opens, a pad port is
what a console's controller ports open, and a `usb.mouse` is what the pointer
drives. That last one is the newest and the one with a caveat:

- **The pointer converts rather than translates.** RFB is absolute (§7.5.5) and
  a HID boot mouse is relative (HID 1.11 Appendix E.10), so
  [`MouseSink`](../../src/host/input.rs) keeps the last position and sends the
  difference, clamped to the report's `-127..127` with the remainder carried
  into the next event. The button masks are ordered differently as well — RFB
  is physical (left, middle, right) and HID is by usage (button 1 primary,
  button 2 secondary, button 3 tertiary, i.e. left, right, middle) — so bits 1
  and 2 swap, and the wheel, which RFB sends as presses on bits 3 and 4, is
  dropped because a three-byte boot report has no wheel axis. An absolute HID
  tablet would need none of this; there is no tablet model in `dev/`.
- **No shipped machine has a pointer yet.** The boards with a USB controller
  (`usb-mini`, `hub-mini`, `xhci-mini`) have no display, and every board with a
  display has no USB — and joining them is not a machine-file edit, because
  neither `usb.ehci` nor `usb.xhci` has a PCI attachment for the PC boards. So
  `tests/vnc_pointer.rs` proves the host end against a mouse built by hand:
  real `PointerEvent` bytes through the server's own parser, out as the report
  a host controller collects from the interrupt endpoint.
  `tests/usb_ehci.rs` already proves the far end, with an RV32 guest reading
  the same reports out of its own RAM.

Three of the implementation notes above turned out differently, and the reasons
are worth recording:

- **Encoding does not run on the task pool.** It cannot yet: the encoder reads
  the [`Surface`](../../src/host/display/mod.rs) the frontend loop owns and
  fills from the scanout, and shipping that to a worker means either a copy per
  frame per client or a lock the emulation loop then waits on. Raw encoding of a
  720x400 frame is a `memcpy` — the surface is allocated in `BGRA8888`, which is
  byte for byte what the default RFB pixel format asks for — so there is nothing
  to move off the thread. When a *compressed* encoding lands, that is the work
  that belongs on the pool, and the per-connection `FrameEncoder` is already the
  unit that would be submitted.
- **There is no region-level dirty tracking to drive it from.** §4.1's dirty
  log is about guest memory pages, not about scanout, and a display device's
  frame counter is the only change signal that exists. So damage is computed by
  comparing the previous frame's rows: a `memcmp` per row, coalesced into
  full-width bands. Cheaper than it sounds, and exact, which page-level
  tracking would not be for a text-mode adapter whose whole screen lives in
  4 kB.
- **The compression wins are available but not free.** Zlib, Tight and ZRLE
  each need a zlib stream held open for the life of the connection with a sync
  flush at every rectangle boundary, and `compcol` does supply that — its
  `zlib::Encoder` sits on a `RawEncoder` whose `raw_flush` takes a `Flush` mode,
  which is the primitive §7.7.2 is describing. So what is missing is the work
  and a decision to put `compcol` into the `vnc` feature's dependency tree, not
  a capability. Raw is what §7.7.1 obliges every client to support, so Raw is
  what a first server owes them.
