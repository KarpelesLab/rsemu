# Low-speed buses

Consumed by: `bus/i2c`, `bus/spi`, and the sensor/EEPROM device models. Mostly
relevant to embedded and SoC machines rather than the PC or the consoles.

| Bus | Source | Notes |
| --- | --- | --- |
| I²C / SMBus | NXP **UM10204** *I2C-bus specification and user manual* — search nxp.com for "UM10204" | The definitive document. SMBus adds timeouts and a command layer on top; SBS Forum publishes the SMBus specification |
| SPI | No formal standard — Motorola's original application note plus each peripheral's datasheet | In practice the *device* datasheet is the specification: mode (CPOL/CPHA), word size, framing |
| SPI NOR flash | Winbond **W25Q** datasheets; see [`storage.md`](storage.md) | The one SPI peripheral almost every board has, and the one whose *semantics* matter more than its framing |
| QSPI pseudo-static RAM | AP Memory **APS6404L-3SQR** datasheet; ISSI IS66WVS2M8, Espressif ESP-PSRAM64H and Lyontek LY68L6400 share the command set | The instruction table, tCEM (8 µs of chip-select-low), the 1 KiB burst wrap and `C0h`'s toggle, and the `9Fh` identification bytes. ST **RM0432**/**RM0456**'s OCTOSPI chapter for `DCR3`'s `CSBOUND` and `MAXTRAN`, the two controller fields that exist to satisfy tCEM |
| 1-Wire | Analog Devices / Maxim device datasheets and application notes | Timing-defined protocol; the datasheet is authoritative |
| MDIO | IEEE 802.3 Clause 22 / 45 | For PHY management behind Ethernet MACs |

## Implementation notes

- These buses are **timing protocols on wires**, so they model naturally onto
  the wire abstraction (`ROADMAP.md` §4.3) plus clock domains — a good exercise
  for both. Most emulators cheat and model them transactionally; that is fine
  and much faster, but make it an explicit choice rather than an accident, since
  some guest firmware bit-bangs these lines directly and will notice.
- Device models (EEPROMs, RTCs, sensors, temperature monitors) are individually
  tiny and each has a free datasheet.

## SPI, as built (`bus-spi`)

`src/bus/spi` does **both**, and the choice is a machine-description property
rather than a default: `link = "transactional"` hands a whole word to the slave
in one call, `link = "wired"` drives SCK, MOSI and CS as real wires, one edge
per half bit period, paced by the scheduler.

A transfer **costs the same virtual time either way** — `bits × 2 × (CLKDIV+1)`
ticks of the controller's clock domain, charged through the lazy-device seam
(§4.2) — so firmware that polls a busy flag sees the same timing under both.
What differs is only whether the individual edges exist for something else to
watch.

**A peripheral is written once.** A device implements the word-level `SpiSlave`;
`Shifter` turns SCK edges into `transfer` calls and `SlavePins` wraps it in
`WireSink`s, so the same model answers a controller that hands it a word, a
controller that clocks it in bit by bit, and a GPIO pin a guest is toggling
itself. `both_link_models_produce_identical_traffic` asserts that over all four
CPOL/CPHA modes and both bit orders; `tests/spi_panel.rs` asserts it again at
machine level, with a whole firmware image.

### What the seam could not express, as of the serial-flash work

Three things, each recorded where a future change would want them:

- **A word format is fixed when `SlavePins` is built.** The only way to change
  it afterwards is `SlavePins::reset`, which also abandons the chip select.
- **A slave declares one mode, and some parts accept two.** A W25Q works in
  mode 0 and mode 3 — they differ only in where SCK rests between frames — so
  `flash.spinor` takes a `mode` property rather than the seam expressing "either".
- **The *wired* fabric has one data line.** A transactional word now carries
  its width — `Lines` on `SpiSlave::transfer_wide`, per *phase*, because an
  APS6404L's `EBh` is a one-line opcode with a four-line address, dummy and
  data — and `stm32.octospi` passes `CCR`'s `IMODE`/`ADMODE`/`ABMODE`/`DMODE`
  straight through, which is also what makes its `DCYC` convert to the right
  number of bytes on a quad command. What has not changed is the pin map:
  there is one `mosi` wire and one `miso` wire, so `SlavePins` announces
  `Lines::SINGLE` and `link = "wired"` cannot express a quad phase. A part
  whose command set stops being decodable in the wrong width — `psram.qspi`
  after `35h` — therefore needs a transactional master, and says so.

### How a slave gets time

It does not have a clock domain, and `SpiSlave::select` carries no timestamp.
Both were considered for `psram.qspi`, whose datasheet specifies **tCEM** — the
maximum time the chip select may stay low before the array goes unrefreshed —
and both were refused for the reason `flash.spinor` already gives about its own
busy window: `SCK` is the *master's* clock, the part's internal oscillator is
not a crystal any board wires, and a slave is reached from inside its
controller's own `advance_to`, where arming a scheduler event is not available.

What is left is the only clock actually on the link, and it turns out to be
enough: **the master's, counted**. A word arrives with the width it was clocked
at, a byte on `n` wires is exactly `8 / n` clocks, and so a slave can add up the
length of a chip-select assertion in integer arithmetic without reading a clock
of any kind. `psram.qspi` takes its budget as `tcem-cycles` rather than
nanoseconds because a slave has no rate to convert with — 8 µs is 672 clocks at
84 MHz and 480 at 60, and the *board* is what knows which. `stm32.octospi`
computes the same number on its side of the same bus, which is what lets
`CR.TCEN`/`LPTR` raise `SR.TOF` and `DCR3`'s `CSBOUND`/`MAXTRAN` split a burst
before the part complains.

`SpiSlave::turnaround` covers the parts that answer in the *second half of the
same word* rather than the next one — the ST7272A's read frame is `R A6..A0`
from the master and `D7..D0` from the panel — so a device that does that needs
no bit handling of its own.

A machine file names which style it uses and why; `machines/spi-panel.machine`
is the worked example.

## I²C, as built (`bus-i2c`)

The same two link styles as SPI, and the same rule: `link = "transactional"`
hands a bus event to `I2cBus` in one call, `link = "wired"` drives SCL and SDA
as real open-drain nets, one edge per half bit period, paced by the scheduler.
A machine file names which, and `st.i2c`/`st.i2c-v2` make the property
**required** so nothing inherits a default.

**A bus event costs the same virtual time either way**, and `bus::i2c` is where
that is fixed rather than in any controller: `START_HALF_PERIODS` (4),
`BYTE_HALF_PERIODS` (18 — nine bit slots, eight data plus the acknowledge) and
`STOP_HALF_PERIODS` (2). The controller decides only what a half period
*lasts* — v1 from `CCR`/`DUTY`/`F-S`, v2 from `TIMINGR`'s `SCLL`/`SCLH`. Anything
that changes those counts changes the guest-visible timeline of every board, so
`a_wired_transfer_costs_the_half_periods_the_fabric_charges_for_it` and each
block's `both_link_models_…` test assert them to the tick.

### One pin pair carries both roles

`MasterWires` and `SlaveWires` each model one role on a pin pair of its own. A
board has neither — an I²C peripheral drives the lines when it is the
controller and watches the *same two nets* when it is not — so `ControllerWires`
is the part a controller actually instantiates: one pin pair, both state
machines, their drive requests wired-AND together the way the pad does it.
That is what makes four things expressible, and the last three are the reason
the wired link exists at all:

- a controller **addressed by another controller**, acknowledging per byte;
- **clock stretching as a level on SCL**, so the other controller stalls on the
  net rather than by asking `I2cBus::stretching` a question;
- **multi-master arbitration** — two controllers starting together, the wired
  AND letting the lower address through, the loser reading a low where it drove
  a high and turning its driver off;
- and the loser then **answering the address that beat it**, because its target
  half has been following the same byte since the START. ST's "the peripheral
  automatically switches back to slave mode" needs no switching.

A controller's target face is the **same `I2cSlave` object** a transactional
`I2cBus` routes to. There is one implementation of a peripheral's slave mode
and no way for the two links to disagree about it, which is the only reason
building this was worth doing rather than special-casing the wired path.

A device whose face can stretch must call `ControllerWires::refresh_stretch`
from `advance_to` **and after every register access**: the engine puts the stall
on by itself at the end of a nine-bit slot, but nothing on the wire says when
software got round to serving it.

Two things the second controller made necessary, both recorded here because
they look like details and are not:

- **A controller waits for a free bus** (§3.1.8). The gap between two of
  somebody else's bits is not a START opportunity — SDA and SCL are both high in
  the middle of every `1` bit, and pulling SDA down there forges a START inside
  their byte. `MasterCore` stalls instead, and tells a *repeated* START apart
  from a fresh one by whether its own half is still holding SCL down. It has to
  be the half's own request rather than the pin: the target half pulls the same
  SDA low to acknowledge somebody else's byte.
- **An open-drain fan-in starts released.** `FanIn::new` starts every slot low,
  which is the neutral level for the wired-*OR* it was written for. On these
  nets low is *asserted*, so a fresh fan-in reads the net low until every driver
  has announced — and the realize sweep announces them one at a time. A `BUSY`
  latched from that is a transaction that never happened, and a controller that
  then refuses to start never sends anything at all. `OpenDrain::learn_sources`
  sets every slot high when it builds the fan.

## What a snapshot has to carry, and what it must not re-announce

A frame is not atomic with respect to a save. `Machine::save` can land between
two SCK edges, so **both endpoints have bit-level state** — the partly assembled
word in a `SlavePins` shifter, or in the controller's own — and both must be in
the chunk. `stm32.spi` and `flash.spinor` each write the seven fields
`Shifter::snapshot` returns; a part that saves only its byte-level decoder comes
back one byte out of step with the wire.

The chip select is the other half, and it is the one that is easy to get
backwards. It is a **level a master drives**, so it is not a chunk of its own:
each master saves whether it holds the line and puts it back on load. What the
bus must not do is treat that as an edge — a slave's `select` is where it
*begins a frame*, and a fresh bus has nothing selected, so a restoring master
calling `SpiBus::select` hands the part a falling edge that never happened and
throws away the mid-frame state the snapshot has just restored into it.
`SpiBus::restore_select` is the silent form, for `Device::load` and nowhere
else, and a master that holds no chip select claims nothing — the other master
on the bus may hold it. In `link = "wired"` the same rule falls out for free:
the level arrives through the wire graph, and `SlavePins::drive` already ignores
a level that did not move.

The `spi-flash` board is the worked example of getting it wrong. It was the
only entry in the catalog's resume ledger: `stm32.spi` re-drove `NSS` on load,
`flash.spinor` heard a fresh chip select, and a flash caught part way through an
OCTOSPI indirect write restarted its command decoder — visible in the very next
byte of its chunk, and in a guest that then diverged.

## Lock order

A fabric here cannot use `LockRank::BUS`, despite the name. A CPU core holds its
execution state across a guest access — the RISC-V hart's session mutex *is*
`LockRank::BUS` — so `BUS` is already held by the time an MMIO write reaches a
device. `bus::spi` takes two ranks of its own between `BUS` and `DEVICE`
(`FABRIC_RANK`, `SHIFTER_RANK`), which is what `LockRank::new` is for. Any other
low-speed fabric will need the same, and the debug ladder catches it on the
first test rather than at the first deadlock.
