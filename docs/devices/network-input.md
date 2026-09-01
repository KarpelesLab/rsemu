# Network, serial, and input devices

Consumed by: `dev/net/*`, `dev/char/*`, `dev/input/*`.

## Network

### What exists

| Piece | Where | Feature |
| --- | --- | --- |
| The seam — frames out, frames in **against a virtual tick**, carrier, MAC | `src/dev/net/link.rs` | `dev-net` |
| `NetPort`, the deterministic in-memory backend, with loopback and a record/replay log | `src/dev/net/link.rs` | `dev-net` |
| An NE2000 card (DP8390 + the Novell card's data window and reset strap) | `src/dev/net/ne2000.rs` | `dev-ne2000` |
| The bridge to `pktkit` — hubs, TAP, slirp, tunnels | `src/dev/net/pktkit.rs` | `net-pktkit` |
| `ne2k-mini`, a Z80 board with the card on its port bus | `machines/ne2k-mini.machine` | `machine-ne2k-mini` |

### A NIC is not a `pktkit::L2Device` — it is behind one

`ROADMAP.md` §7.2 says every emulated NIC *is* a `pktkit::L2Device`. It cannot
be, and the correction matters more than it sounds.

`L2Device` delivers a received frame by **calling a handler** the moment the
frame exists, on whatever host thread produced it. An emulated machine has no
defined position in virtual time at that instant, so a NIC that accepted the
frame there would put it in the guest's receive ring at a different guest cycle
on every run. That is the non-deterministic input `CLAUDE.md` forbids, and it
would make every state hash built on a machine with a NIC worthless.

So the seam (`rsemu::dev::net::link::NetLink`) inverts it: an arriving frame is
queued **against a virtual tick** and the NIC *pulls* it out at a tick the
scheduler chose. A `pktkit::L2Device` is then one implementation of that seam
(`rsemu::dev::net::pktkit::PktkitLink`, which is itself an `L2Device` so it
plugs straight into an `L2Hub`), and hubs, NAT/slirp, TUN/TAP and tunnels do
still come free. What is *not* reinvented is packet parsing: nothing under
`dev/net/` knows what ARP or IP is.

Two consequences worth writing down:

- Only `dev/net/pktkit.rs` needs `std`. The seam and the NE2000 are
  `no_std + alloc` and dependency-free, so the exception `ROADMAP.md` §0 grants
  `dev/net/*` is one file wide rather than a whole subtree.
- `pktkit`'s `L2Hub` ages its MAC-learning table on `Instant::now`, and
  `L2Adapter`'s ARP and NDP caches expire the same way. Neither is reachable
  from inside the scheduler, but a topology that depends on an aged-out entry is
  a topology whose *recording* is the only reproducible artefact.

### Register-level documentation per NIC

| NIC | Source |
| --- | --- |
| Intel e1000 / e1000e | Intel *PCI/PCI-X Family of Gigabit Ethernet Controllers Software Developer's Manual* **[browser]** — thorough and free |
| Realtek RTL8139 | Realtek RTL8139 datasheet; [OSDev: RTL8139](https://wiki.osdev.org/RTL8139) |
| Intel i217 / i219 | [OSDev: Intel Ethernet i217](https://wiki.osdev.org/Intel_Ethernet_i217) |
| NE2000 | National Semiconductor DP8390 datasheet ([bitsavers](https://bitsavers.org/)) |
| virtio-net | [`../buses/virtio.md`](../buses/virtio.md) |

## Serial

| Device | Source |
| --- | --- |
| 16550 UART | National Semiconductor **PC16550D** datasheet — the specification for essentially every emulated serial port; [OSDev: Serial Ports](https://wiki.osdev.org/Serial_Ports) |
| ARM PL011 | Arm DDI 0183 **[browser]** |

## Input

| Device | Source |
| --- | --- |
| 8042 keyboard controller | IBM AT Technical Reference; [OSDev: PS/2 Keyboard](https://wiki.osdev.org/PS/2_Keyboard) |
| USB HID | [USB-IF HID class specification](https://www.usb.org/documents) (free) |
| Console controllers | The respective platform documentation |

## Implementation notes

- The 8042 is a **microcontroller with a command protocol**, not a register
  block, and guests use it for far more than keys (A20 gating, CPU reset).
  Implement it from the AT Technical Reference.
- Network receive is asynchronous and arrives from outside the machine — it is
  therefore a record/replay input and must cross into the machine through that
  seam, or determinism is lost (`ROADMAP.md` §4.5). That seam does not exist
  yet. `NetPort` carries the network half of it in the shape it has to have: a
  list of `(virtual tick, frame)`, recorded as the guest is handed each frame
  and replayed by queueing them back at the same ticks. When the general seam
  lands it should adopt that pair rather than invent a second one.
- **The FCS is not modelled.** No backend on this seam produces a frame check
  sequence and none checks one, so a NIC stores the frame as it arrived and
  `RSR.CRC`-style error bits are never set. Every driver that subtracts a
  header length from a byte count still gets exactly the bytes that were on the
  wire.
