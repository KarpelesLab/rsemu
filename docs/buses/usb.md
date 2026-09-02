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
  is cleaner than EHCI's split-transaction handling. Ordering went the other way
  here and the note is kept because the advice is still right for a fresh tree —
  what made EHCI the cheaper *first* controller was that the fabric under it did
  not exist yet, and a queue head is a smaller thing to be wrong about than a
  ring, a context and a Cycle bit. Both are built now (§2 and §7).
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

## USB, as built (`bus-usb`, `dev-usb-ehci`, `dev-usb-chipidea`, `dev-usb-dwc2`, `dev-usb-hid`, `dev-usb-msd`, `dev-usb-xhci`)

In the order they matter — and the order matters because the whole value of the
arrangement is that the *next* controller reuses the first one unchanged. That
is no longer a hope: §4 is the controller that tested it, §4.1 is the
*direction* that tested it, and §7 is the controller whose schedule format is
least like the fabric's and still needed nothing from it.

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
split transactions and no hub. Those two are less coupled than this file used to
say: `SPLIT` tokens, the `µFrame C-mask` and the `siTD` are what a high-speed hub
carrying a **full- or low-speed** device needs (USB 2.0 §11.14, EHCI 1.0 §4.12),
and a high-speed hub with high-speed devices behind it needs none of them. What a
hub of *any* kind actually needs first is routing in the fabric — `UsbBus::find`
searches a flat list of enabled ports — so the obstacle is smaller than it was
written to be and it is in a different place. EHCI is high-speed only, and this one does the
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
its assertion and acknowledgement are tested. For *this* block no test wires that
output to an interrupt controller — but the gap is no longer the shape it was:
`machines/usb-mini.machine` wires the **generic** EHCI's `irq` into a PLIC whose
`meip0` reaches the hart, and `tests/usb_msd.rs` has a guest take a real machine
external trap on `IOC` and acknowledge it in the order the level forces:
`USBSTS` write-one-to-clear *first*, then the PLIC claim complete. That order is
asserted by **counting traps** rather than by noticing one — eleven in the right
order, twenty-two in the wrong one, because completing a claim while the level is
still asserted makes the PLIC re-latch the source. Since `chipidea` is a register
map over the same engine, what is left here is a board, not a mechanism.

**Dual role is partial and says so.** Host mode is complete; selecting device
mode stops the host schedule so the controller does not quietly keep walking a
schedule that no longer exists. The device-side queue-head list is never walked
and `ENDPTSETUPSTAT`/`ENDPTPRIME`/`ENDPTFLUSH`/`ENDPTSTAT`/`ENDPTCOMPLETE`/
`ENDPTCTRLn` read zero. The firmware's `libusb_printer` role therefore does not
work, and would configure and then never see a transfer.

That is now a gap with a shape rather than an open question: §4.1 built device
mode for the dwc2 and the fabric carried it, so what is left here is this
block's own device engine — a **dQH/dTD schedule walker in guest memory**, which
is the ChipIdea device side's actual content and is nothing like dwc2's FIFO. It
would use `UsbDevice` for everything and the new host-side composer for nothing,
since it is a device. The honest obstacle is not the seam: it is that no public
datasheet for the CX92755's device half is available to us, and the first-party
reverse engineering above covers the host flow only.

### 4. A dwc2 (`src/dev/usb/dwc2.rs`, `dev-usb-dwc2`) — and what it proved

STM32's **OTG_FS** block is a Synopsys DesignWare USB 2.0 OTG core, and it is
not an EHCI in any respect. It was built here specifically to test the claim
§1 makes — that the fabric is controller-agnostic because its seam is a
*transaction* — against a controller shaped nothing like the one the fabric was
written alongside.

**The result: `src/bus/usb/` needed no change at all.** Not a signature, not a
type, not a new method. The one thing the controller borrowed from the fabric
beyond the transaction calls is `TransferType::from_attribute_bits`, which
decodes `HCCHAR.EPTYP` because those two bits mean what they mean in an endpoint
descriptor — the controller and the device agree on the encoding because neither
of them owns it.

How different the two controllers are is worth stating, because that is what
makes the result mean something:

| | EHCI | dwc2 (slave mode) |
| --- | --- | --- |
| Where the work is | linked lists in **guest RAM** | **registers**: `HCCHARn`, `HCTSIZn` |
| How data moves | the controller DMA-reads and writes it | the **CPU** pushes and pops a FIFO, a word at a time |
| Bus mastering | required; refused at bind without `space =` | **none**; the machine file gives it no space |
| Completion | a bit in a qTD the controller wrote back | `HCINTn` → `HAINT` → `GINTSTS` |
| Receiving | into the buffer the descriptor named | one shared FIFO, announced packet by packet through `GRXSTSP` |
| Ports | up to fifteen, with a companion to hand one to | exactly one, `HPRT`, and nobody to hand it to |
| Speed | high only | full and low only (with an FS PHY) |

`tests/usb_dwc2.rs` is the end-to-end claim, and it is the same claim
`tests/usb_ehci.rs` makes about the same fabric and the same device model: an
RV32 program running on the emulated hart powers the port, resets it, programs
host channels, pushes setup packets into a FIFO window and reads the replies
back out of `GRXSTSP` — and **enumerates a mouse and collects a report from its
interrupt endpoint**, with the device descriptor landing in the buffer the guest
named.

#### Speed, and why the honest answer is not EHCI's

An OTG_FS has an on-chip full-speed transceiver and no high-speed PHY, and
`HCFG.FSLSS` is the register that says so. It therefore drives full- **and
low-speed** devices, which an EHCI cannot do at all, and cannot drive a
high-speed one, which an EHCI can.

EHCI's refusal has a register for it — hand the port to a companion controller,
EHCI 1.0 §4.2.2 — and rsemu's EHCI does that. A dwc2 root port has no companion,
and there is no encoding for "that device signals faster than these pins". So the
port simply **does not enable**: `HPRT.PENA` stays clear and a driver's reset
times out, which is what happens on a bench. The `hid` mouse grew a `speed`
property for this, because "high speed, deliberately" was an argument about EHCI
and there is now a controller it does not apply to.

#### Time, and the one simplification

A frame is `HFIR.FRIVL` PHY clocks times the board's ticks-per-PHY-clock. At the
48 MHz an OTG_FS runs at, a driver writes 48 000 and a frame is exactly 48 000
ticks — integer, residue-free, the same property the EHCI's 7500-tick microframe
has. `HFIR`'s reset value of `0xea60` is *not* a millisecond at that clock, and
the model leaves it wrong until the driver fixes it, exactly as the silicon does.

Transactions are executed at frame boundaries, up to a per-frame byte budget
taken from the signalling rate (1500 bytes at full speed, 187 at low), each
charged its packet plus the 13 bytes of protocol overhead USB 2.0 §5.11.3 gives a
full-speed transaction. Throughput is therefore right; **latency is coarser than
silicon** — a real full-speed transaction takes about 50 µs, so a channel armed
just after a frame boundary waits longer here than it would on hardware. That is
the one timing simplification, and it is written down rather than hidden.

#### What is bounded, and why the fuzz target is different

The EHCI's hazard is that it *reads guest memory*: a list can close a circle.
This one reads no memory at all, so its hazards are the FIFOs the guest fills and
the frame that drains them. Each channel's transmit staging and the shared
receive FIFO are capped by the programmed FIFO sizes, themselves capped by the
`fifo` property — the RAM the part actually has, 1.25 KiB on an OTG_FS — so no
register write can make the device allocate. The frame is capped by the byte
budget and by a hard transaction count, and `HFIR.FRIVL` is clamped so a frame
interval a guest chose cannot make the host spin.
`fuzz/fuzz_targets/usb_dwc2.rs` drives arbitrary bytes at both surfaces and
asserts that a frame always ends.

#### `GRXSTSP` is the `MemAttrs::debug` trap

Reading it **pops the receive FIFO**, and reading a FIFO window consumes the
packet. Both answer a debug read without moving anything — `GRXSTSP` returns what
`GRXSTSR` would — and, as with the EHCI, a debug *write* is refused outright:
`HCINTn` is write-1-to-clear, `GRSTCTL` resets the core, `HCCHAR.CHENA` starts a
transaction and a FIFO write puts bytes on the wire.

#### The interrupt

`HCINTn` → `HAINT` → `GINTSTS`, gated by `HCINTMSKn`, `HAINTMSK`, `GINTMSK` and
finally `GAHBCFG.GINTMSK`, collapses to **one level output pin, `irq`**,
re-derived from the register file on `announce` rather than latched. The pin
carries no number: which interrupt an OTG_FS is on is a fact about the part — 67
on an STM32F407 — so a board writes `wire otgfs.irq -> cpu.irq67` and there is
nothing in between, a Cortex-M's NVIC being inside the core.

#### Generic, not STM32-specific

The file is `dev/usb/dwc2.rs` and the class is `usb.dwc2`, not `stm32.otg`,
because STM32 adds no registers to this core — unlike ChipIdea, which adds `ID`
and `USBMODE`. What ST supplies is a *configuration*: eight host channels,
1.25 KiB of FIFO RAM, a full-speed PHY, and a placement. All four are machine-file
properties, so an OTG_FS is `usb.dwc2` with `channels = 8, fifo = 320,
speed = "full"` mapped where the board maps it, and the same class serves the
several other SoCs that instantiate this core. ST's `GCCFG` and `CID` sit in the
core's vendor slots at `+0x38` and `+0x3c` and are modelled as what they are
there: a configuration register nothing gates on, and a user ID a board supplies.

#### Dual role, and the direction that tested the fabric

The block is dual-role and **so is this model**: `GUSBCFG.FDMOD` selects the
device side, `GINTSTS.CMOD` reports which role is running, and reaching for the
other role's registers raises `GINTSTS.MMIS` as the silicon does. Device mode —
where the guest is the peripheral and somebody else enumerates it — is §4.1
below, together with what it cost the fabric.

### 4.1 Device mode: the guest *is* the peripheral

Two controllers declined this in identical terms — *"a whole second controller
facing the other way"* — and both were right to at the time. It is now built,
for the dwc2, and the first thing to record is the answer to the question that
made it worth doing.

#### What drives a guest that is a device

Nothing new. **Anything that calls `UsbBus::setup`/`read`/`write` is a host**,
and there are now three kinds of caller, all of which were already possible:

| Driver | What it is | Where |
| --- | --- | --- |
| Another controller on the same bus | a `usb.dwc2` in host mode, root port on the same named bus | `src/dev/usb/dwc2/device/tests.rs`, `a_dwc2_host_enumerates_a_dwc2_device_over_one_bus` |
| A host-side transfer composer | `bus::usb::host::ControlTransfer` — no schedule, no clock | `tests/usb_dwc2_device.rs` |
| A bridge to real hardware | a caller of the same two, over `usbfs` | not started |

The loopback is the interesting one, because it is a *cable between two ports of
one machine*: a host core and a device core on one `UsbBus`, the host walking
`HCCHARn` and the device answering out of `DIEPCTLn`, and neither of them
containing a line about the other.

#### Did the fabric need changing? Almost not, and the "almost" is the finding

`UsbDevice` was already the right seam and needed no new method for the data
path. **`Function`/`Peripheral` is emphatically the wrong one**, and that is
worth stating because it is the tempting answer: a `Peripheral` is `Endpoint0`
wrapped around a `Function`, and `Endpoint0` answers the eleven standard
requests of USB 2.0 §9.4 *inside the emulator*, out of a descriptor table the
emulator holds. In device mode all of that is the guest firmware's — its
descriptors, its `SET_ADDRESS`, its bugs. `Dwc2Gadget` therefore implements
`UsbDevice` directly, which the trait's own documentation had anticipated:
*"implemented by `Peripheral` for anything built the ordinary way, and directly
by anything that genuinely is not, which so far is nothing."*

Three things did move, and all three are additive:

1. **`UsbDevice::start_of_frame`.** A `SOF` is the one thing on the wire that is
   neither a transaction nor a reset: a token with no data phase and no
   handshake, broadcast rather than addressed (USB 2.0 §8.4.3). A modelled
   peripheral never needed it; a guest does, because `DSTS.FNSOF` is a register
   its driver reads and `GINTSTS.SOF` is how a gadget paces an isochronous
   endpoint. It is exactly the argument `bus_reset` already made — the *other*
   non-transaction event — and it has a default that does nothing, so no
   existing device model changed. `UsbBus::start_of_frame` broadcasts it to
   every connected device, and the dwc2 host calls it at each frame boundary.
2. **`bus::usb::host`.** Every previous caller of the transaction methods was a
   controller with a schedule, so *which sequence of transactions a control
   transfer is* lived in the controller — correctly. A test harness, a loopback
   and a `usbfs` bridge have no schedule and would each have rewritten the three
   stages of §8.5.3. `ControlTransfer` is those three stages written once, and
   it deliberately **never waits**: one transaction per `step`, `Progress::Nak`
   handed straight back, because on this side of the seam the device may be a
   guest that has not run yet.
3. **One sentence of `UsbDevice::speed`'s documentation.** It said a device's
   speed is *"fixed for its lifetime: a device does not change speed, it is a
   different device"*. That is true of a mouse and false of a gadget: a
   dual-role core with a high-speed transceiver runs at full speed when its
   firmware writes `DCFG.DSPD` saying so. The *code* needed nothing — the fabric
   already asks afresh every time and caches nothing — so this is a comment fix,
   which is the smallest possible kind of finding and is recorded because
   "nothing changed" would have been the wrong summary.

No new lock rank. The ladder holds unchanged because the re-entrancy contract
was already being kept: a host controller releases its `HCD_RANK` lock before
calling the fabric, and `UsbBus` releases `FABRIC_RANK` before calling the
device, so the *device-side* controller takes `HCD_RANK` with no other
`HCD_RANK` lock held — even when both ends are the same class on one bus, which
is the case that would have caught it.

#### Why dwc2 and not ChipIdea

Three reasons, in order of weight. The register map is **documented**: RM0090
§34.15.3 is free and complete, whereas the CX92755's device half has no public
datasheet and the first-party reverse engineering above covers the host flow
only — building it would have meant inventing a queue-head format. The **data
path already exists**: a dwc2 gadget pushes and pops the same shared FIFO the
host half already models, so device mode is a second register file over an
existing engine, while ChipIdea's device side is a *second DMA schedule walker*
over dQH/dTD lists in guest memory. And the **far end exists**: the dwc2 host
half is in this tree, so a loopback proves the whole path with no new host.

#### What it is, in registers

`DCFG` (speed and the address the core answers to), `DCTL` (soft connect, global
NAK), `DSTS` (enumerated speed, and the frame number of the last `SOF`),
`DIEPCTLn`/`DOEPCTLn` with their `DIEPINTn`/`DOEPINTn` and
`DIEPTSIZn`/`DOEPTSIZn`, `DTXFSTSn`, `DIEPTXFn`, and `DAINT`/`DAINTMSK` feeding
`GINTSTS.IEPINT`/`OEPINT` the way `HAINT` feeds `GINTSTS.HCINT`. Setup packets
arrive through the same `GRXSTSP` the host half receives on, announced with the
device-mode `PKTSTS` codes (`0110b` setup data, `0100b` setup complete, `0010b`
`OUT` data, `0011b` `OUT` complete). Endpoint count is a property — four on an
OTG_FS — like the host's channel count.

Two behaviours are worth singling out because they are the ones a half-built
device controller gets wrong:

- **`DCTL.SDIS` is the pull-up on D+**, and it resets *set*. So a board with this
  class on it does not have a device on the bus until its firmware says so, and
  clearing that bit is literally what calls `UsbBus::attach`. Setting it again,
  or selecting host mode, or a core reset, is what calls `detach` — and a host
  core on the same bus sees the port disable, which is tested.
- **NAK is the whole synchronisation story.** A transaction arrives
  synchronously and the guest is not running at that instant; an endpoint that
  is not armed answers `NAK` and the host comes back. Nothing is queued, nothing
  is deferred, and no callback exists. That is not a simplification — it is what
  the `NAK` handshake is for (USB 2.0 §8.4.5).

#### What device mode does not do, said plainly

Back-to-back setup packets (`DOEPINT.B2BSTUP` is defined and never raised — one
packet arrives per transaction and nothing can be behind it); suspend and remote
wakeup (`DSTS.SUSPSTS` reads zero: a modelled host that stops issuing
transactions is indistinguishable from a busy one, so a suspend timer would be
an invented event); the `PKTSTS = 0001b` global-OUT-NAK FIFO entry (the global
NAK bits themselves *are* honoured — with either set, every endpoint NAKs); and
the frame-parity and data-toggle selectors `SD0PID`/`SODDFRM`, which are
accepted and dropped because this fabric carries no PID on an endpoint
transaction. A compile-time assertion keeps those two out of the writable set so
a later edit cannot quietly claim otherwise.

#### The end-to-end claim, and it is not "the registers read back"

`tests/usb_dwc2_device.rs`: an RV32 program on the emulated hart forces device
mode, partitions its FIFO RAM, releases soft disconnect, waits for
`GINTSTS.USBRST`, arms `DOEPCTL0`, and then — when a `GET_DESCRIPTOR` arrives —
reads the eight setup bytes out of `GRXSTSP` and its endpoint-zero FIFO window
into its own RAM, takes `wLength` **out of the request it just read**, programs
`DIEPTSIZ0` with it, arms `DIEPCTL0`, and pushes eighteen bytes of device
descriptor into the FIFO a word at a time. The host on the far side collects
exactly those eighteen bytes. A snapshot taken with the reply still sitting in
the transmit FIFO restores to the same state hash and the transfer then finishes.

The register-level version of the same claim, with a dwc2 *host core* on the far
end instead of a Rust composer, is
`a_dwc2_host_enumerates_a_dwc2_device_over_one_bus`.

### 5. A device (`src/dev/usb/hid.rs`, `dev-usb-hid`)

A HID boot-protocol mouse: enumeration, a report descriptor, and three-byte
reports on an interrupt IN endpoint. **High speed by default, deliberately** —
see §2 for why a low-speed one behind an EHCI would be a device that never
enumerates — and `speed = "full"` or `"low"` for the dwc2, which is the
controller that argument does not apply to. The descriptors follow the speed:
`bMaxPacketSize0`, `bInterval` in frames rather than microframes, and no device
qualifier for a device with only one speed to be (USB 2.0 §9.6.2). Movement enters
through `HidMouse::motion` and nowhere else, because a real pointer is a
non-deterministic input and those belong to the record/replay seam, which does
not exist yet.

`tests/usb_ehci.rs` is the end-to-end claim: an RV32 program, running on the
emulated hart, claims the root ports, resets one, builds queue heads and
descriptors in its own RAM, starts the controller, and finds the device
descriptor and a mouse report in the buffers it named — having learned that they
arrived by polling `USBSTS`.

### 6. A disk (`src/dev/usb/msd.rs`, `dev-usb-msd`)

The device that makes USB reach the storage stack. A **USB mass storage device**
speaking **Bulk-Only Transport** over a SCSI transparent command set, whose bytes
are a `dev::medium::Medium` — the same seam an ATA drive, an AHCI port and an NVMe
namespace read and write — so `rsemu run usb-mini --drive usb0=disk.qcow2` puts a
sparse qcow2 behind a USB stick through the media slot that already existed.

**It needed no new controller**, and that is the claim worth making about it.
Bulk-Only is two bulk endpoints and the default pipe (BOT §1.1: it does not even
require an interrupt endpoint), and §2's EHCI already walks bulk queue heads,
keeps data toggles, honours short packets and turns a `STALL` into a halted qTD.
So a guest driver reaches a device written months later through a controller that
never heard of it, which is the test of whether §1's transaction seam was the
right shape. It was; nothing in `src/bus/usb/` changed for the transfers.

One thing *was* added to the fabric, and it is argued where it lives:
`Function::halt_cleared`. This device halts a bulk pipe **itself**, as a protocol
signal, and BOT §3.1 says the class reset preserves endpoint stall conditions
while §5.3.4 makes Reset Recovery the class reset *followed by* a
`CLEAR_FEATURE(ENDPOINT_HALT)` on each pipe. Without the hook the class would
never hear about the host's `CLEAR_FEATURE` and could not model the sequence the
specification defines. Additive, defaulted, and no existing device model changed.

**All thirteen cases are implemented, not approximated.** BOT §6.7 tabulates
every combination of what the host said it wanted against what the device turned
out to intend, and says for each one what the residue is, whether the status is
Command Passed, Command Failed or Phase Error, and which pipe gets stalled. Six
of the thirteen are error paths a driver only reaches when it has miscomputed
something — exactly the ones an emulator is tempted to skip — and `msd/tests.rs`
walks every one by name.

**Nothing is ever sized from a number the guest chose.** This device masters no
bus and walks no guest structure, so the hazard the EHCI has is not its hazard;
what it has instead is a byte stream on a bulk endpoint, and the same discipline
applies to it. A CBW is one 31-byte packet or it is not a CBW (§5.1, §6.2.1), so
there is no reassembly buffer to grow. `dCBWDataTransferLength` is a `u32` the
guest chose and is a counter, never a length. A `READ (10)` for 65,535 blocks is
a **cursor into the medium**, so it costs one packet of memory rather than
32 MiB. Every allocation length is clamped before it reaches a `Vec`, and every
logical block address is checked in `u64` with `checked_add`, so `u64::MAX` is
`LOGICAL BLOCK ADDRESS OUT OF RANGE` rather than a wrap into block zero.
`fuzz/fuzz_targets/usb_msd.rs` is the target that keeps all of that true, and it
carries BOT §5.2's own invariant with it: the residue shall not exceed the
transfer length it answers.

`tests/usb_msd.rs` is the end-to-end claim, and it is the storage one:
`machines/usb-mini.machine` boots an RV32 program that enumerates the disk,
issues `GET_MAX_LUN`, and then runs `INQUIRY`, `READ CAPACITY (10)`,
`READ (10)` and `WRITE (10)` as CBW/data/CSW triples through the EHCI's bulk
path — and the sector that lands in guest RAM is asserted **against the medium**,
which the test holds a second handle to and the guest cannot reach, with the
neighbouring blocks asserted untouched. The completion interrupt is not polled:
it travels `ehci.irq → plic.irq1 → plic.meip0 → cpu.meip`, the guest's own trap
handler acknowledges it, and the test counts the traps so that acknowledging in
the wrong order is a failure rather than a slowdown nobody notices.

What is not modelled: **one LUN** (`Get Max LUN` answers zero), no unit attention
after a reset, no `MODE SENSE` page bodies beyond the parameter header that
carries the write-protect bit, and no `FORMAT UNIT`. A second LUN would be a
property and a `Vec` of media; the rest is a page whose contents would be an
invention.

### 7. An xHCI (`src/dev/usb/xhci.rs`, `dev-usb-xhci`)

The modern controller, and the one shaped like **NVMe** rather than like EHCI.
Where §2 hands the controller two linked lists, xHCI hands it *rings*: a command
ring, one transfer ring per endpoint, and an event ring the controller is the
**producer** of — plus a Device Context Base Address Array, device and endpoint
contexts, and a doorbell array. All of it lives in guest RAM.

**The Cycle bit is the ownership protocol** (xHCI 1.2 §4.9). A ring has no head
and tail register: each TRB carries a bit, the producer's cycle state says which
value means "mine", the consumer walks forward until the bit disagrees, and a
**Link TRB** closes the ring back on itself and toggles the state when its `TC`
flag is set. So a ring is a *cycle by construction*, which is the same hazard the
EHCI's circular queue-head list has with a different spelling, and it is bounded
the same way: link hops, TRBs per Transfer Descriptor, work items per doorbell,
packets per TRB. `fuzz/fuzz_targets/usb_xhci.rs` keeps all four true, and it maps
the register block **into the space the controller masters** so a TRB's data
buffer can be aimed at the doorbell array — four bytes of anything are a doorbell
write, so the engine is iterative rather than recursive, exactly as `dev-nvme`
is for a PRP entry aimed at `SQyTDBL`. It found a real defect on its first seeded
run: a Slot ID is eight bits of a guest-written TRB and the enabled-slot bitmap
is a `u32`, so 32 shifted past the end of it.

**Nothing in `src/bus/usb/` changed for it.** That is the second time the seam
has been tested by a controller with a completely different schedule format —
§4 was the first — and this time it was tested by the one whose format is least
like the fabric's. The device on the other end is §6's disk, unmodified.

**Acknowledging an interrupt is three writes in a fixed order**, and the order is
the specification's rather than a convention: `USBSTS.EINT` first (§5.4.2 bit 3
names the race), then `ERDP` with `EHB` (§5.5.2.3.3, which is also how software
says how far it has read), then `IMAN.IP` (§4.17.3: the pin stays asserted until
that write). `tests/usb_xhci.rs` counts the traps a guest takes and asserts
**fifteen**; completing the interrupt-controller claim before the third write
measures **thirty**.

**Traps and events are deliberately different numbers there** — nineteen events
in fifteen interrupts — because `ERDP.EHB` blocks a second interrupt until the
handler has drained the ring, which is §4.17.2's moderation scheme working. Both
numbers are asserted, so a change to either is a failure rather than a difference
nobody notices.

What is not modelled, said plainly: **SuperSpeed** (the fabric has no such
speed, so the one Supported Protocol capability declares `USB ` 2.0 and nothing
else), **streams** (a non-zero `MaxPStreams` is a *Parameter Error*, not a field
ignored), **isochronous transfers**, **scratchpad buffers**, **save/restore
state**, and **more than one interrupter**. Unlike an EHCI the root hub drives
full- and low-speed devices itself, so nothing is handed to a companion and
nothing vanishes.

`machines/xhci-mini.machine` is `usb-mini` with the controller swapped and
nothing else changed, which is what makes the comparison worth anything: the same
hart, the same PLIC, the same disk, and the sector read over rings checked
against the same `Medium::read_at` as the sector read over queue heads.

## Host passthrough

Bridging to real hardware is a `UsbDevice` implementor under `host/` and nothing
more, which is why the seam is shaped the way it is. The *other* direction — a
guest acting as a device, handed to a real host through `usbfs`'s gadget side —
is a caller of `bus::usb::host::ControlTransfer` and the bus's transaction
methods, and needs nothing new either. Neither is started. **`libusb` cannot be used**:
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
