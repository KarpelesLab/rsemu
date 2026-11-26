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
| ST **AN5289** rev 18, "How to build wireless applications with STM32WB MCUs" | §4.10 the wireless-stack device information table and the version-word format (Fig. 12); §5.2 CPU2 start-up; §8.6.5 `SHCI_C2_BLE_Init` parameters, the `CFG_BLE_OPTIONS` bits and the attribute-record accounting; §14.2 the mailbox interface, including **which events CPU1 returns** (all except BLE command responses); §14.5 a second statement of the memory-size word | st.com |
| ST **AN5185** rev 15, "ST firmware upgrade services for STM32WB" | §6.1 the FUS device information table (magic 0xA94656B9) and the system table; §6.2 IPCC channels 2 and 4; §6.3 packet indicators 0x10/0x11/0x12, the READY event 0x9200, FUS commands and responses (NumHCI 0xFF); §2.3 switching between FUS and the wireless stack | st.com |
| ST **AN5270** rev 36 (24 November 2026), "Introduction to STM32WB Bluetooth LE wireless interface" | every HCI/ACI opcode with its input and output parameters and "Events generated" (Command Complete or Command Status); status codes §4.1 (Table 468); Tx power §4.2 (Table 469); configuration data offsets and reset values §4.3 (Table 470)/§2.3.1/§2.3.2; Table 472, the revision history, for what earlier revisions removed. The model was first written from rev 35. | st.com |
| ST **RM0434** rev 16 | §9.4 IPCC processor-2 registers (C2CR, C2MR reset 0xFFFFFFFF, C2SCR, C2TOC1SR); `PWR_CR4.C2BOOT` | st.com |
| Bluetooth **Core Specification** Vol 4 Part E | HCI packet and event formats, unknown-command handling (§4.5), LE_Encrypt byte order (§7.8.22), the cancelled-create LE Connection Complete (§7.8.13) | bluetooth.com |
| Apache **NuttX**, `arch/arm/src/stm32wb/stm32wb_mbox*.{c,h}`, `stm32wb_blehci.c` (Apache-2.0, ASF) | the field order of the reference, BLE, system and memory-manager tables; the list node layout; channel numbers; packet type values; SHCI opcodes; `BLE_Init` option bits | apache.googlesource.com/nuttx |
| **Pigweed**, `pw_bluetooth/public/pw_bluetooth/hci_events.emb` (Apache-2.0) | `Supported_Commands` and LE feature bit positions, transcribing Core Vol 4 Part E §6.27 and Vol 6 Part B §4.6 | pigweed.googlesource.com |

ST's `STM32_WPAN` middleware (`stm32-mw-wpan`) is under ST's SLA0044 per its
`LICENSE.md` SBOM and **was not read**. No emulator source was consulted
(`ROADMAP.md` §1).

## The command table

`ble::COMMANDS` was extracted mechanically from AN5270's tables (opcode
lists, input and output parameter sizes, "Events generated") and then checked
by hand. It was rechecked against rev 36 by running the same extraction over
rev 35 and rev 36 and diffing both the extractions and the full text.
Where AN5270 and the Core Specification disagree on a standard HCI command,
the Core Specification wins. Those commands are `LE_Read_Remote_Features`,
`LE_Extended_Create_Connection`, `LE_Generate_DHKey_v2` (Command Status) and
`LE_Transmitter_Test` (Command Complete). For an ACI command AN5270 is the
only authority, so `ACI_GATT_DISC_PRIMARY_SERVICE_BY_UUID` (§2.6.19) and
`ACI_GATT_READ_MULTIPLE_VAR_CHAR_VALUE` (§2.6.43) are Command Complete, as
both revisions say. They were Command Status here until the rev 36 check.
`ACI_GAP_SET_NON_CONNECTABLE` loses its "Events generated" heading to a page
break, and 0xFC95 is listed as `ACI_HAL_ALLOW_REBOND` in the opcode table.
The doc comment on `COMMANDS` covers each of these.

What rev 36 changed against rev 35:

* 0xFCD0 is renamed `ACI_GAP_START_SCAN`. Its `Procedure` is now 0x01
  (limited discovery), 0x02 (general discovery) or 0xB0
  (`GAP_GENERIC_SCAN_PROC`) (Table 273). `ACI_GAP_TERMINATE_GAP_PROC` with
  the same code ends it (§2.5.52). The model now runs it as a GAP procedure.
  Any other code gets 0x92, which is an inference.
  The four older scan procedures (§2.5.22, §2.5.25, §2.5.26, §2.5.32) are
  marked deprecated.
* `Service_Handle` is "ignored by the BLE stack" in `ACI_GATT_ADD_CHAR_DESC`,
  `UPDATE_CHAR_VALUE`, `DEL_CHAR`, `DEL_INCLUDE_SERVICE`,
  `SET_SECURITY_PERMISSION`, `SET_DESC_VALUE`, `UPDATE_CHAR_VALUE_EXT` and
  `SET_ACCESS_PERMISSION` (Tables 285–355). The model used to reject a
  mismatched service with 0x60. It now finds the characteristic by its handle
  alone.
* New: `ACI_GATT_NOTIFICATION_LL_ACK_EVENT` (0x0C1C, §3.4.27) and its
  `GATT_Evt_Mask` bit 0x0080_0000, `ACI_RESET` option 0x400 ("no NRPA
  timeout"), and `ACI_WARNING_EVENT` type 0x08. None can occur without a
  radio, an NRPA timer or an NVM.
* Many parameters are renamed to the Core Specification's spelling. No
  opcode, size or reply kind changed. The status-code and configuration-data
  tables are unchanged apart from their numbers. Table 1 now lists
  `HCI_LE_ENABLE_ENCRYPTION` in the BF variant too. Stack variants are not
  modelled. The document now targets Core 6.3 and STM32CubeWB 1.25.0.

## Inferences and unverified values

These could not be confirmed from a permitted source. Each is marked where it
is defined. Every item was searched for in AN5270 rev 36 and AN5289 rev 18.
The result is given with each.

* **BLE command responses do not come from the pool.** AN5289 §14.2.2 tells
  CPU1 not to return them, so they cannot come from the pool. This model
  writes a Command Complete over the command buffer and a Command Status into
  the command-status buffer, then links that buffer into the event queue.
  The command-status buffer is 16 bytes in NuttX, which is exactly a node
  plus a Command Status event. *Still unverified:* §14.2.2 says only "wait for
  the command response" and never says where it is written.
* **Pool slots are 268 bytes.** That is a node plus the largest event,
  rounded to a word, and equals NuttX's `STM32WB_MBOX_CMDPKT_BUF_SIZE`.
  ST's own carving is unknown. *Still unverified:* §14.2.2 names the pool
  (`p_AsynchEvtPool`) but not how it is divided.
* **`FUS_GET_STATE` sent to the wireless stack** returns state 0xFE and
  error 0xFE. AN5185 names `FUS_STATE_NOT_RUNNING` but not the byte layout.
  *Still unverified:* AN5289 lists the opcode (0xFC52, §14.6.1 Table 34)
  and refers to AN5185 for the rest.
* **The FUS device information table** is written for the 36 bytes AN5185
  Table 9 lists, although the text says 42. *Still unverified:* AN5289 §4.10
  covers only the wireless-stack layout and refers to AN5185 for FUS's.
* `HCI_Version`/`LMP_Version` are 0x0D (Core 5.4). *Still unverified:*
  AN5270 Table 16 says only "see Bluetooth assigned numbers". Rev 36 is
  written against Core 6.3 (rev 35: 6.2), which says nothing about what the
  controller reports.
* `HCI_Subversion` packs the stack version's major, minor and sub into 4 bits
  each. AN5270 Table 16 *confirms* it is "the BLE stack reduced version
  number", but not how it is reduced. Table 16 also says `LMP_Subversion`
  carries the HW version in its high byte and the FW version in its low byte.
  Neither value is documented, so the model repeats `HCI_Subversion` there.
  *Still unverified.*
* LE ACL buffers are 251 × 8. The filter accept and resolving lists both
  hold 8. The LE feature mask is 0x41FF, and the supported-states mask is
  every bit through 41. *Still unverified:* Tables 24, 44, 81 and the
  `LE_States` table give only ranges.
* `ACI_GET_INFORMATION`: the *layout* is verified. AN5270 Table 171 gives
  Status, Version (8), Options (4) and Debug_info (12), 25 bytes in all.
  Putting the `BLE_Init` options in Options is *verified in meaning*. Bits
  0–4 and 7 of Table 171 are bit for bit AN5289 §8.6.5's `CFG_BLE_OPTIONS`.
  Bits 8–9 have no `BLE_Init` source, so the model leaves them clear. *Still
  unverified:* what fills the eight Version octets. The model writes the
  version word and then zeros.
* The DIT memory-size words are placeholders. *Still unverified,* and the
  layout is now in question too. AN5289 Figure 12 puts SRAM2b in bits 31:24
  and SRAM2a in bits 23:16, which is what the model follows. §14.5
  (`LHCI_C1_Read_Device_Information`) puts SRAM2a in bits 31:24 and SRAM2b
  in bits 23:16. Both agree that flash, in 4 KB sectors, is bits 7:0.
* The status for a full service range is 0x61, and for an exhausted
  attribute or service pool it is 0x64. A command before `ACI_GATT_INIT`
  gets 0x0C. *Still unverified:* Table 468 defines the codes ("Out of
  handles", "Insufficient resources", "Command disallowed") but no command
  says when it returns them. §2.6.1 says only that "the GATT channel does
  not process any command" until `ACI_GATT_INIT`.
* Without a write at 0x2E, the static random address is derived from the
  seed. The real part derives it from its UID. *Still unverified:* Table 470
  gives the configuration value's reset as all zeros, and offset 0x80 reads
  back the address in use. Neither document says how that address is made.
* `ACI_GAP_START_SCAN` answers an unlisted `Procedure` with 0x92. This is an
  inference added with the rev 36 check, because Table 273 lists only three
  values.
* **Resolved:** `ACI_HAL_GET_FW_BUILD_NUMBER` (0xFC00) is an unknown command
  here. That is *verified*: AN5270 Table 472 records it as removed in rev 31
  (June 2025), with `ACI_HAL_GET_PM_DEBUG_INFO` and `ACI_HAL_STACK_RESET`.

## Not modelled

* Radio activity of any kind: advertising reports, connections, pairing,
  GATT client responses, notifications.
* Time inside the stack. A GAP procedure runs until it is terminated, even
  a discovery that the part ends after 10.24 s (AN5270 §2.5.52).
* `ACI_RESET`'s mode 0x01, which changes the stack options. The command
  resets the stack but leaves the options as `BLE_Init` set them.
* The Thread, 802.15.4, Zigbee, trace and LLD tables, and IPCC channels 3
  and 5.
* The local P-256 key and DH key (they answer Unsupported Feature).
* CPU2's use of HSEM, RNG and flash at boot. The system reset that
  `FUS_START_WS` causes on the part.
* `AttrValueArrSize` is recorded but not enforced.
