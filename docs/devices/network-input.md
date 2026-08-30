# Network, serial, and input devices

Consumed by: `dev/net/*`, `dev/char/*`, `dev/input/*`.

## Network

All emulated NICs are `pktkit::L2Device` implementations, so hubs, NAT/slirp,
TUN/TAP and tunnels come free (`ROADMAP.md` §7.2). What each NIC model needs is
its own register-level documentation:

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
  seam, or determinism is lost (`ROADMAP.md` §4.5).
