# Low-speed buses

Consumed by: `bus/i2c`, `bus/spi`, and the sensor/EEPROM device models. Mostly
relevant to embedded and SoC machines rather than the PC or the consoles.

| Bus | Source | Notes |
| --- | --- | --- |
| I²C / SMBus | NXP **UM10204** *I2C-bus specification and user manual* — search nxp.com for "UM10204" | The definitive document. SMBus adds timeouts and a command layer on top; SBS Forum publishes the SMBus specification |
| SPI | No formal standard — Motorola's original application note plus each peripheral's datasheet | In practice the *device* datasheet is the specification: mode (CPOL/CPHA), word size, framing |
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

`SpiSlave::turnaround` covers the parts that answer in the *second half of the
same word* rather than the next one — the ST7272A's read frame is `R A6..A0`
from the master and `D7..D0` from the panel — so a device that does that needs
no bit handling of its own.

A machine file names which style it uses and why; `machines/spi-panel.machine`
is the worked example.

## Lock order

A fabric here cannot use `LockRank::BUS`, despite the name. A CPU core holds its
execution state across a guest access — the RISC-V hart's session mutex *is*
`LockRank::BUS` — so `BUS` is already held by the time an MMIO write reaches a
device. `bus::spi` takes two ranks of its own between `BUS` and `DEVICE`
(`FABRIC_RANK`, `SHIFTER_RANK`), which is what `LockRank::new` is for. Any other
low-speed fabric will need the same, and the debug ladder catches it on the
first test rather than at the first deadlock.
