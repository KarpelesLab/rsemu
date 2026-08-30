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
