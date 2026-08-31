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

## USB, as built (`bus-usb`, `dev-usb-ehci`, `dev-usb-chipidea`, `dev-usb-hid`)

Three layers, in the order they matter — and the order matters because the
whole value of the arrangement is that the *next* controller reuses the first
two.

### 1. The fabric (`src/bus/usb/`, `bus-usb`)

Controller-agnostic, as the implementation note above asks. The seam is a
**transaction** rather than a transfer — `SETUP`, `IN`, `OUT`, and a handshake
back — because a transaction is the indivisible unit on the wire (USB 2.0 §8.4)
and *which sequence of them* a transfer is made of is the controller's business,
since the controller is what holds the schedule. All four transfer types are
therefore expressible without the transfer type ever appearing in a signature,
exactly as it never appears on the wire.

A device model is written **once**: `Endpoint0` implements the eleven standard
requests of §9.4 from a `Descriptors` table — including `SET_ADDRESS`'s
effect-after-the-status-stage, which is the classic thing to get wrong — and
`Peripheral` wraps it around a `Function`, which is the class-specific half. The
HID mouse is a report descriptor, three class requests and one endpoint.

Ports carry connect, enable, reset and speed. **Enable belongs to the
controller, not to the fabric**, which is what makes "two freshly attached
devices both answer address zero" a modelled situation rather than an
undetectable one.

### 2. A generic EHCI (`src/dev/usb/ehci.rs`, `dev-usb-ehci`)

The register file is the small half. The controller proper is the **schedule
walker**: the driver builds queue heads and queue element transfer descriptors
in guest RAM, hands over `ASYNCLISTADDR` and `PERIODICLISTBASE`, and the
controller DMA-walks them as a bus master with its own `MemAttrs::requester`.
Control and bulk on the asynchronous ring, interrupt on the periodic frame list
with its microframe S-mask; short packets and the alternate-next pointer, data
toggles, `IOC`, `STALL`-to-halt (and a halted queue head that *stays* halted),
and the async-advance doorbell.

**Every loop over a guest-built list is bounded** — a queue head can link to
itself, a descriptor can point at itself, a frame list can close a circle — and
`fuzz/fuzz_targets/usb_ehci.rs` drives arbitrary bytes through both the register
block and guest RAM to keep it that way.

Isochronous `iTD`/`siTD` nodes are followed and **not executed**; there are no
split transactions and no hub. EHCI is high-speed only, and this one does the
honest thing with anything else: it hands the port to a companion controller by
setting `PORTSC.Port Owner` (EHCI 1.0 §4.2.2), so a full-speed device attached to
a bare EHCI *vanishes* rather than silently enumerating on a bus that could not
have carried it.

A microframe is 125 µs, which §4.2 calls an awkward rate. It is only awkward if
you derive it from seconds: on the 60 MHz clock a USB 2.0 PHY actually runs at,
a microframe is exactly **7500** ticks and a frame exactly 60 000, with no
residue and no float. The controller is a lazily advanced device — it holds its
own tick, publishes the tick its next microframe falls on, and is caught up
before any register access.

### 3. The ChipIdea/ARC variant (`src/dev/usb/chipidea.rs`, `dev-usb-chipidea`)

**A register map, and nothing else.** It contains no schedule walker, no qTD
decoding and no DMA; those are the EHCI engine's, because that is what the block
*is*. What the vendor owns, and what is therefore in that file:

| | |
| --- | --- |
| `+0x000` | `ID` (the CX92755 reads `0xfa05`), `HWGENERAL`, `HWHOST`, `HWDEVICE`, `HWTXBUF`, `HWRXBUF` |
| `+0x100` | the EHCI capability registers, with `CAPLENGTH = 0x40` — so the operational registers are at **`+0x140`** |
| `+0x120` | `DCIVERSION`, `DCCPARAMS` |
| `+0x1a4` | `OTGSC`, `USBMODE` — the role select |

`0xfa05` is self-consistent with the published `ID` field format (`ID = 5` in
bits 5:0, `NID = 0x3a` in bits 13:8 — the six-bit complement of 5), which is a
useful check on a magic number arrived at independently. It is a **property**,
not a constant, and so are the block's addresses: `0xf00bc000` on the CX92755 is
one SoC's placement of a reusable core, and a `map` statement is where a board
says so. **The block base is where `ID` is** — `USBCMD` is `0x140` further on —
so a board maps its window at `0xf00bc000`, not at `0xf00bc140`.

The wrapper/PHY aperture at `0xf0084000` is **not modelled**. Nothing in the
recorded flow touches it, and a PHY nobody reads is three registers of
invention; `ULPI_VIEWPORT` inside the block is modelled as an access that
completes before the write returns, which is what keeps a "poll until `RUN`
clears" loop terminating.

#### Reconciling the two sources

The section above is first-party reverse engineering of one specific part; the
rest of the register map comes from the published ChipIdea/ARC core layout,
which is the general core. **Where they disagree the part wins.** They do not
currently disagree — every offset the reverse engineering fixes (`+0x140`,
`+0x144`, `+0x148`, `+0x1a8`, `ID` at `+0x00`) is exactly where the published
layout puts it, which is itself evidence that this is a stock core and that the
rest of that map is a reasonable thing to build on.

Two things are **inferred rather than confirmed**, flagged so that a future
contradiction is recognised instead of absorbed:

- **`CAPLENGTH = 0x40`, with the capability block at `+0x100`.** The reverse
  engineering fixes `+0x140` but not how the block gets there, and the
  firmware's flow never reads `CAPLENGTH` — it hard-codes the offset, as
  firmware for a known part does. `0x100 + 0x40` is the published arrangement
  and reproduces the one number that *is* confirmed.
- **`REVISION` in `ID` bits 23:16.** The firmware masks with `0xFFFF`, so the
  upper half is unconstrained. The `id` property is the whole 32-bit value and
  defaults to `0xfa05` exactly, revision reading zero.

Steps 1–4 of the recorded flow are asserted directly, in the documented order,
by `the_firmwares_reset_handshake_completes_in_the_documented_order`.

#### The one behavioural disagreement, and how it was settled

`USBMODE.CM` is write-once **after a reset**, which forces a decision about what
`HCReset` does to it. The recorded flow asserts `HCReset` at step 2 and *then*
selects the role and reads it back at step 3, and this firmware uses both roles
— host for mass storage and PictBridge, device for the printer it presents. So
the reset **re-arms** `CM` rather than preserving it: carrying the old role
across would make step 3's read-back return the previous mode and hang the
firmware on its own check. `a_reset_re_arms_the_role_select_so_a_switch_works`
is the test that keeps it that way, and `hchalted_is_the_complement_of_runstop`
pins the property the step-1 spin depends on — the one a stub already had.

#### Not in the recorded flow, and it matters

The flow covers reset, role selection, detection, allocation, schedule
construction and interrupt enable. It never mentions `CONFIGFLAG` or `PORTSC`.
EHCI 1.0 §4.2 leaves every root port owned by a companion controller until
`CONFIGFLAG` is written, and this model obeys that — so **if the firmware really
never writes it, nothing will enumerate**, and the cause will be an unclaimed
port rather than anything in the schedule walker. That is the first thing to
check when this block is wired to a real image. It is deliberately not papered
over by defaulting `CONFIGFLAG` to one.

The flow also says transfers are **interrupt-driven, not polled**. The
controller drives a level `irq` output (the AND of `USBSTS` with `USBINTR`) and
its assertion and acknowledgement are tested — but no test wires it to an
interrupt controller and no guest here has taken an interrupt from it: the
end-to-end test polls. That gap is real and belongs to whoever wires the board.

**Dual role is partial and says so.** Host mode is complete; selecting device
mode stops the host schedule so the controller does not quietly keep walking a
schedule that no longer exists. The device-side queue-head list is never walked
and `ENDPTSETUPSTAT`/`ENDPTPRIME`/`ENDPTFLUSH`/`ENDPTSTAT`/`ENDPTCOMPLETE`/
`ENDPTCTRLn` read zero — a whole second controller, facing the other way, is not
this piece of work. The firmware's `libusb_printer` role therefore does not
work, and would configure and then never see a transfer.

### 4. A device (`src/dev/usb/hid.rs`, `dev-usb-hid`)

A HID boot-protocol mouse: enumeration, a report descriptor, and three-byte
reports on an interrupt IN endpoint. **High speed, deliberately** — see above
for why a low-speed one would be a device that never enumerates. Movement enters
through `HidMouse::motion` and nowhere else, because a real pointer is a
non-deterministic input and those belong to the record/replay seam, which does
not exist yet.

`tests/usb_ehci.rs` is the end-to-end claim: an RV32 program, running on the
emulated hart, claims the root ports, resets one, builds queue heads and
descriptors in its own RAM, starts the controller, and finds the device
descriptor and a mouse report in the buffers it named — having learned that they
arrived by polling `USBSTS`.

## Host passthrough

Bridging to real hardware is a `UsbDevice` implementor under `host/` and nothing
more, which is why the seam is shaped the way it is. **`libusb` cannot be used**:
the dependency policy is absolute and OS interaction is by raw syscall. On Linux
the sanctioned route is `usbfs` — `/dev/bus/usb/BBB/DDD` with the `USBDEVFS_*`
ioctls, driven by raw `openat`/`ioctl`/`close` — behind its own feature, and it
is inherently non-portable and non-deterministic, so it wants the record/replay
seam first. Nothing of it is started.

## Lock order

Like every fabric here, USB cannot use `LockRank::BUS`, for the reason
`low-speed.md` records: a CPU core holds its execution state across a guest
access and that state *is* `LockRank::BUS`. The ladder is four ranks of its own
between `BUS` and `DEVICE`:

```text
  CPU session          (BUS 0x4000)
    → HCD registers    (0x4a00, bus::usb::HCD_RANK)
      → USB fabric     (0x4b00, bus::usb::FABRIC_RANK)
        → default pipe (0x4c00, bus::usb::EP0_RANK)
          → the device's own state (DEVICE 0x5000)
```

A controller never holds its register lock across the DMA walk or a call into
the fabric — that is the re-entrancy contract rather than a rank question, and
the ladder is what catches it if one does.
