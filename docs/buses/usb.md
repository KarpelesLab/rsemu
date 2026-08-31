# USB

Consumed by: `bus/usb` and the host controllers.

## Primary

| Source | Covers | Access |
| --- | --- | --- |
| [USB-IF document library](https://www.usb.org/documents) | USB 2.0 and USB 3.x base specifications, plus every device-class specification (HID, Mass Storage, CDC, Audio, Hub) | **Free download**, no membership |
| eXtensible Host Controller Interface (xHCI) specification | The modern host controller: rings, contexts, TRBs | intel.com **[browser]** |
| EHCI / UHCI / OHCI specifications | The legacy controllers | Intel / Compaq, freely available |

USB is unusual among the buses here: the specifications are genuinely free, so
there is no excuse for working from anything else.

## Working references

| Source | Covers |
| --- | --- |
| [OSDev: USB](https://wiki.osdev.org/Universal_Serial_Bus) | Overview and the controller landscape |
| [OSDev: xHCI](https://wiki.osdev.org/XHCI) / [EHCI](https://wiki.osdev.org/EHCI) / [UHCI](https://wiki.osdev.org/UHCI) | Practical implementation notes per controller |
| [OSDev: HID](https://wiki.osdev.org/USB_Human_Interface_Devices) | Keyboards, mice, gamepads |

## Implementation notes

- Model the **device side generically** — endpoints, transfer types, descriptors
  — so a device model works behind any controller. The controller then becomes a
  translator between its ring/queue format and generic transfers.
- **xHCI first** if only one is built: it is what modern guests prefer, and it
  is cleaner than EHCI's split-transaction handling.
- Transfers are asynchronous and take guest-visible time. Route completions
  through the event queue at a virtual time derived from the guest clock, never
  the host clock — that is what keeps determinism (`ROADMAP.md` §4.7).
- Bridging to *real* USB hardware is possible later via the existing
  [`usbmagic`](https://github.com/KarpelesLab/usbmagic) work.

## ChipIdea / ARC dual-role, as found on Conexant DigiColor

First-party reverse engineering of the CX92755-class firmware (§1: our own work
on our own hardware, so it is a clean source). Recorded here because no public
datasheet for this part is available to us, which makes these notes *the*
specification for that block.

The controller is a **ChipIdea/ARC dual-role** core in the standard layout:
operational registers sit at a **+0x140** offset from the block base, which is
what distinguishes it from a bare EHCI. On this SoC:

| Block | Address |
| --- | --- |
| Operational registers | `0xF00BC000` |
| Wrapper / PHY | `0xF0084000` |

| Register | Offset | Role |
| --- | --- | --- |
| `ID` | `+0x00` | detection — firmware checks `(ID & 0xFFFF) == 0xFA05` |
| `USBCMD` | `+0x140` | `RunStop` (bit 0), `HCReset` (bit 1) |
| `USBSTS` | `+0x144` | `HCHalted` (bit 12), interrupt status (**W1C**) |
| `USBINTR` | `+0x148` | interrupt enables |
| `USBMODE` | `+0x1A8` | host/device role select |

Both addresses are properties in the machine file, not constants: this is one
SoC's placement of a reusable core.

### The initialisation flow the firmware performs

`EHCI_Host_Reset` → `EHCI_Init`:

1. Poll `USBSTS.HCHalted` until the controller is halted.
2. Assert `USBCMD.HCReset` and poll until it **self-clears**.
3. Select host mode through `USBMODE`, then read it back.
4. Read `ID` to detect the controller.
5. Allocate qTD/QH/IOP/EP buffer pools in the `0x40000000` DRAM bank.
6. Build the async and periodic schedules — QH/qTD linked lists in RAM.
7. Enable interrupts. **Transfers are interrupt-driven, not polled.**

So the data path is textbook EHCI: the firmware builds queue-head and
transfer-descriptor linked lists in guest RAM, points the controller at them,
and the controller **DMA-walks them** and raises an interrupt on completion.
The controller is therefore a bus master and reads through the address space
like any other.

### Roles this firmware uses

Dual-role in the literal sense, and both directions are exercised:

- **Host** — USB mass storage (six drives, `b`–`g`) and PictBridge cameras
  (`dps_transport`).
- **Device** — presenting a printer to a PC (`libusb_printer`).

### What a stub already bought

Modelling only the reset handshake — `HCReset` self-clears, and
`HCHalted = !RunStop` — was enough to get the firmware past the `usbsts` spin
it was stuck on and into multithreaded operation, with the scheduler spawning
application tasks.

It logs `NO EHCI DETECTED`, because `ID` reads 0 rather than `0xFA05`, and
treats USB host as optional and continues. That is a useful property to keep in
mind while building: **this firmware degrades rather than failing**, so a
half-modelled controller can look like a working boot. Detection succeeding is
the first thing that will change its behaviour, and the first thing that can
regress it.
