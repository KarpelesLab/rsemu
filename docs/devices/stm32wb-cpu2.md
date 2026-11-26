# STM32WB CPU2: `st.wb-cpu2`

Consumed by: `src/dev/stm32/wb_cpu2.rs` (the transport) and
`src/dev/stm32/wb_cpu2/ble.rs` (the BLE stack). It works with `st.ipcc`
(`src/dev/stm32/ipcc.rs`) and is driven by `st.pwr`'s `C2BOOT`.

The WB55's Cortex-M0+ runs ST's wireless stack, an encrypted binary we cannot
run. This class is a bus master that behaves like that firmware. It serves
the SRAM2 mailbox, answers system and BLE commands in ST's formats, and keeps
real GAP/GATT state. It has no radio, so there are no peers and no
connections.

## Sources

| Document | Answers | Where |
| --- | --- | --- |
| ST **AN5289** rev 18, "How to build wireless applications with STM32WB MCUs" | §4.10 the wireless-stack device information table and the version-word format (Fig. 12); §5.2 CPU2 start-up; §8.6.5 `SHCI_C2_BLE_Init` parameters and the attribute-record accounting; §14.2 the mailbox interface, including **which events CPU1 returns** (all except BLE command responses) | st.com |
| ST **AN5185** rev 15, "ST firmware upgrade services for STM32WB" | §6.1 the FUS device information table (magic 0xA94656B9) and the system table; §6.2 IPCC channels 2 and 4; §6.3 packet indicators 0x10/0x11/0x12, the READY event 0x9200, FUS commands and responses (NumHCI 0xFF); §2.3 switching between FUS and the wireless stack | st.com |
| ST **AN5270** rev 35, "Introduction to STM32WB Bluetooth LE wireless interface" | every HCI/ACI opcode with its input and output parameters and "Events generated" (Command Complete or Command Status); status codes §4.1; Tx power §4.2; configuration data offsets and reset values §4.3/§2.3.1/§2.3.2 | st.com |
| ST **RM0434** rev 16 | §9.4 IPCC processor-2 registers (C2CR, C2MR reset 0xFFFFFFFF, C2SCR, C2TOC1SR); `PWR_CR4.C2BOOT` | st.com |
| Bluetooth **Core Specification** Vol 4 Part E | HCI packet and event formats, unknown-command handling (§4.5), LE_Encrypt byte order (§7.8.22), the cancelled-create LE Connection Complete (§7.8.13) | bluetooth.com |
| Apache **NuttX**, `arch/arm/src/stm32wb/stm32wb_mbox*.{c,h}`, `stm32wb_blehci.c` (Apache-2.0, ASF) | the field order of the reference, BLE, system and memory-manager tables; the list node layout; channel numbers; packet type values; SHCI opcodes; `BLE_Init` option bits | apache.googlesource.com/nuttx |
| **Pigweed**, `pw_bluetooth/public/pw_bluetooth/hci_events.emb` (Apache-2.0) | `Supported_Commands` and LE feature bit positions, transcribing Core Vol 4 Part E §6.27 and Vol 6 Part B §4.6 | pigweed.googlesource.com |

ST's `STM32_WPAN` middleware (`stm32-mw-wpan`) is under ST's SLA0044 per its
`LICENSE.md` SBOM and **was not read**. No emulator source was consulted
(`ROADMAP.md` §1).

## The command table

`ble::COMMANDS` was extracted mechanically from AN5270's tables (opcode
lists, output-parameter sizes, "Events generated") and then checked by hand.
Where AN5270 and the Core Specification disagree on a standard HCI command,
the Core Specification wins. Those commands are `LE_Read_Remote_Features`,
`LE_Extended_Create_Connection`, `LE_Generate_DHKey_v2` (Command Status) and
`LE_Transmitter_Test` (Command Complete). Two entries lose their "Events
generated" heading to a page break. A third is listed as
`ACI_HAL_ALLOW_REBOND` in the opcode table. The doc comment on `COMMANDS`
covers each of these.

## Inferences and unverified values

None of these could be confirmed from a permitted source. Each is marked
where it is defined.

* **BLE command responses do not come from the pool.** AN5289 §14.2.2 tells
  CPU1 not to return them, so they cannot come from the pool. This model
  writes a Command Complete over the command buffer and a Command Status into
  the command-status buffer, then links that buffer into the event queue.
  The command-status buffer is 16 bytes in NuttX, which is exactly a node
  plus a Command Status event.
* **Pool slots are 268 bytes.** That is a node plus the largest event,
  rounded to a word, and equals NuttX's `STM32WB_MBOX_CMDPKT_BUF_SIZE`.
  ST's own carving is unknown.
* **`FUS_GET_STATE` sent to the wireless stack** returns state 0xFE and
  error 0xFE. AN5185 names `FUS_STATE_NOT_RUNNING` but not the byte layout.
* **The FUS device information table** is written for the 36 bytes AN5185
  Table 9 lists, although the text says 42.
* `HCI_Version`/`LMP_Version` are 0x0D (Core 5.4). `HCI_Subversion` is
  derived from the stack version word.
* LE ACL buffers are 251 × 8. The filter accept and resolving lists both
  hold 8. The LE feature mask is 0x41FF, and the supported-states mask is
  every bit through 41.
* `ACI_GET_INFORMATION` puts the version word in its Version field and the
  `BLE_Init` options in Options.
* The DIT memory-size words are placeholders.
* The status for a full service range is 0x61, and for an exhausted
  attribute or service pool it is 0x64. A command before `ACI_GATT_INIT`
  gets 0x0C.
* Without a write at 0x2E, the static random address is derived from the
  seed. The real part derives it from its UID.

## Not modelled

* Radio activity of any kind: advertising reports, connections, pairing,
  GATT client responses, notifications.
* The Thread, 802.15.4, Zigbee, trace and LLD tables, and IPCC channels 3
  and 5.
* The local P-256 key and DH key (they answer Unsupported Feature).
* CPU2's use of HSEM, RNG and flash at boot. The system reset that
  `FUS_START_WS` causes on the part.
* `AttrValueArrSize` is recorded but not enforced.
