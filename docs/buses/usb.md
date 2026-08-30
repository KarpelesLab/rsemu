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
