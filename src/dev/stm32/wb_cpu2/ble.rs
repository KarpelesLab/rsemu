//! The behavioural BLE stack: what ST's CPU2 firmware answers to an HCI or ACI
//! command, with no radio underneath.
//!
//! Everything here is a pure function of the command bytes and this module's
//! own state; the transport (`super`) moves bytes between it and SRAM2. The
//! split is what lets the command model be tested without a mailbox, and the
//! mailbox without a command model.
//!
//! # Sources
//!
//! * ST **AN5270** rev 36 (November 2026) — every opcode, every input and
//!   output parameter size, the "Events generated" list that decides Command
//!   Complete against Command Status, the status codes (§4.1), the Tx power
//!   table (§4.2) and the configuration-data offsets and reset values (§4.3,
//!   §2.3.1, §2.3.2). [`COMMANDS`] was extracted from that document's tables
//!   mechanically and then checked by hand; the handful of places where it and
//!   the Core Specification disagree are listed on the table. It was first
//!   written from rev 35; what rev 36 changed is listed there too.
//! * ST **AN5289** rev 18 §8.6.5 — the `SHCI_C2_BLE_Init` parameter block, and
//!   the attribute-record accounting ("increased by 9 … standard attribute
//!   profile and GAP service characteristics") that fixes the handles the GATT
//!   and GAP services occupy.
//! * Bluetooth **Core Specification** Vol 4 Part E (HCI): §5.4 packet
//!   formats, §7.7.14/§7.7.15 Command Complete / Command Status, §7.7.65 the LE
//!   Meta event, §6.27 the `Supported_Commands` bit table. The bit positions
//!   used by [`SUPPORTED`] were taken from Pigweed's `hci_events.emb`
//!   (Apache-2.0, © 2023 The Pigweed Authors), which transcribes that table.
//! * NuttX `arch/arm/src/stm32wb/stm32wb_mbox_shci.h` (Apache-2.0, ASF) for the
//!   `BLE_Init` option bits, cross-checked against AN5289.

use alloc::collections::BTreeMap;
use alloc::vec;
use alloc::vec::Vec;

use purecrypto::cipher::{Aes128, BlockCipher};

use crate::core::error::{Error, Result};
use crate::core::rand::Stream;
use crate::core::state::{ChunkReader, ChunkWriter, Sink, Source};

// ---------------------------------------------------------------------------
// Status codes (AN5270 §4.1, Table 468)
// ---------------------------------------------------------------------------

/// Success.
pub(crate) const OK: u8 = 0x00;
/// Unknown HCI command.
pub(crate) const UNKNOWN_COMMAND: u8 = 0x01;
/// Unknown connection identifier — what every command naming a connection
/// gets, because there never is one.
pub(crate) const UNKNOWN_CONNECTION: u8 = 0x02;
/// Command disallowed.
pub(crate) const DISALLOWED: u8 = 0x0C;
/// Unsupported feature or parameter value.
pub(crate) const UNSUPPORTED: u8 = 0x11;
/// Invalid HCI command parameters: a packet shorter than its own layout.
pub(crate) const INVALID_HCI_PARAMS: u8 = 0x12;
/// Invalid handle (an ACI code, AN5270 Table 468).
pub(crate) const INVALID_HANDLE: u8 = 0x60;
/// Out of handles: a service's reserved range is full.
pub(crate) const OUT_OF_HANDLES: u8 = 0x61;
/// Insufficient resources: the attribute or service pool `BLE_Init` sized is
/// exhausted.
pub(crate) const INSUFFICIENT_RESOURCES: u8 = 0x64;
/// Invalid parameters (an ACI code).
pub(crate) const INVALID_PARAMS: u8 = 0x92;

// ---------------------------------------------------------------------------
// Events (Core Vol 4 Part E §7.7; AN5270 §3)
// ---------------------------------------------------------------------------

/// `HCI_Command_Complete`.
pub(crate) const EVT_COMMAND_COMPLETE: u8 = 0x0E;
/// `HCI_Command_Status`.
pub(crate) const EVT_COMMAND_STATUS: u8 = 0x0F;
/// `HCI_LE_Meta`.
pub(crate) const EVT_LE_META: u8 = 0x3E;
/// The vendor-specific event every ACI event and every system event rides in.
pub(crate) const EVT_VENDOR: u8 = 0xFF;

/// `HCI_LE_Connection_Complete` (AN5270 Table 394).
const LE_CONNECTION_COMPLETE: u8 = 0x01;
/// `ACI_GAP_PROC_COMPLETE_EVENT` (AN5270 Table 410).
const ACI_GAP_PROC_COMPLETE: u16 = 0x0407;

/// `ACI_GAP_START_SCAN`'s procedures (AN5270 rev 36 Table 273). The first two
/// are Table 415's bits; `GAP_GENERIC_SCAN_PROC` is the general and selective
/// connection-establishment and observation bits together.
const GAP_LIMITED_DISCOVERY: u8 = 0x01;
const GAP_GENERAL_DISCOVERY: u8 = 0x02;
const GAP_GENERIC_SCAN: u8 = 0xB0;

/// `Num_HCI_Command_Packets` in every Command Complete and Command Status the
/// BLE channel carries: one, because the mailbox has exactly one command
/// buffer and CPU1 may not write another until this one is answered (AN5289
/// §14.2.2, `TL_BLE_SendCmd`).
const NUM_HCI_COMMANDS: u8 = 1;

// ---------------------------------------------------------------------------
// The command table
// ---------------------------------------------------------------------------

/// How a command is answered.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum Reply {
    /// `HCI_Command_Complete` carrying the return parameters.
    Complete,
    /// `HCI_Command_Status`; whatever the command starts finishes later.
    Status,
    /// Nothing at all — `HCI_Host_Number_Of_Completed_Packets` is the one.
    Silent,
}

/// One row of [`COMMANDS`].
#[derive(Debug, Clone, Copy)]
pub(crate) struct Command {
    /// The opcode, OGF in the top six bits.
    pub(crate) opcode: u16,
    /// The name AN5270 gives it — what a trace prints.
    pub(crate) name: &'static str,
    /// Command Complete or Command Status.
    pub(crate) reply: Reply,
    /// The fixed part of the return parameters, status included, for a
    /// Command Complete answered generically.
    pub(crate) ret: u8,
    /// Whether the first parameter is a `Connection_Handle`: such a command
    /// fails with [`UNKNOWN_CONNECTION`], because no connection ever exists.
    pub(crate) conn: bool,
}

const fn row(opcode: u16, name: &'static str, reply: Reply, ret: u8, conn: bool) -> Command {
    Command {
        opcode,
        name,
        reply,
        ret,
        conn,
    }
}

use Reply::{Complete as CC, Silent as NONE, Status as CS};

/// Every command AN5270 rev 36 lists, with how it is answered.
///
/// Extracted from AN5270's command tables (§2.1–§2.7: the opcode lists,
/// the "output parameters" tables and each section's "Events generated"),
/// then corrected by hand where the extraction or the document itself is
/// wrong:
///
/// * `ACI_GAP_ALLOW_REBOND` is listed as `ACI_HAL_ALLOW_REBOND` in Table 176
///   (still, in rev 36); 0xFC95 either way.
/// * `ACI_GAP_SET_NON_CONNECTABLE` loses its "Events generated" heading to a
///   page break; the bullet under it says Command Complete.
/// * `ACI_GATT_DISC_PRIMARY_SERVICE_BY_UUID` (§2.6.19) and
///   `ACI_GATT_READ_MULTIPLE_VAR_CHAR_VALUE` (§2.6.43) are Command Complete,
///   unlike every other GATT client procedure. Both revisions say so, and
///   for an ACI command AN5270 is the only authority. (This table answered
///   both with Command Status until it was rechecked against rev 36.)
/// * AN5270 says Command Complete for `HCI_LE_READ_REMOTE_FEATURES_PAGE_0`,
///   `HCI_LE_EXTENDED_CREATE_CONNECTION` and `HCI_LE_GENERATE_DHKEY_V2`, and
///   Command Status for `HCI_LE_TRANSMITTER_TEST`. The Core Specification
///   (Vol 4 Part E §7.8.21, §7.8.66, §7.8.94, §7.8.29) says the opposite in
///   all four, and a standard HCI command is answered as the standard says:
///   this table follows the Core Specification there.
/// * `ACI_GAP_CHECK_BONDED_DEVICE`'s `Id_Address` is six octets, not the one
///   the table prints; `ACI_GAP_GET_BONDED_DEVICES` and the variable-length
///   rows are answered by their own handlers, not from `ret`.
/// * `ACI_HAL_GET_FW_BUILD_NUMBER` (0xFC00) is an unknown command here: rev
///   31 removed it (Table 472, the revision history, "Removed former Section
///   2.3.1: ACI_HAL_GET_FW_BUILD_NUMBER"), along with
///   `ACI_HAL_GET_PM_DEBUG_INFO` and `ACI_HAL_STACK_RESET`.
///
/// What rev 36 changed against rev 35, besides renumbering every table from
/// 447 on by one:
///
/// * 0xFCD0 is renamed `ACI_GAP_START_SCAN` (was `ACI_GAP_EXT_START_SCAN`)
///   and its `Procedure` is now 0x01 limited discovery, 0x02 general
///   discovery or 0xB0 `GAP_GENERIC_SCAN_PROC` (Table 273), ended by
///   `ACI_GAP_TERMINATE_GAP_PROC` with that same code (§2.5.52). The four
///   older discovery and connection-establishment procedures are marked
///   deprecated in its favour (§2.5.22, §2.5.25, §2.5.26, §2.5.32).
/// * `Service_Handle`/`Serv_Handle` is "Deprecated; this parameter is ignored
///   by the BLE stack" in `ACI_GATT_ADD_CHAR_DESC`, `_UPDATE_CHAR_VALUE`,
///   `_DEL_CHAR`, `_DEL_INCLUDE_SERVICE`, `_SET_SECURITY_PERMISSION`,
///   `_SET_DESC_VALUE`, `_UPDATE_CHAR_VALUE_EXT` and `_SET_ACCESS_PERMISSION`
///   (Tables 285, 287, 289, 293, 345, 347, 353, 355): the characteristic
///   handle alone finds the characteristic. `ACI_GATT_ADD_CHAR` still uses it.
/// * A new event, `ACI_GATT_NOTIFICATION_LL_ACK_EVENT` (0x0C1C, §3.4.27,
///   `GATT_Evt_Mask` bit 0x0080_0000); `ACI_RESET`'s options gain 0x400 "no
///   NRPA timeout" (Table 169); `ACI_WARNING_EVENT` gains type 0x08 "GATT
///   database saving issue" (Table 467). None is reachable without a radio,
///   an NRPA timer or an NVM.
/// * Parameter renames to the Core Specification's spelling (`TX_Channel`,
///   `Test_Data_Length`, `Max_Latency`, `Handle` in `HCI_READ_RSSI`, …): no
///   size, opcode or reply kind changed.
#[rustfmt::skip]
pub(crate) static COMMANDS: &[Command] = &[
    // §2.1 HCI commands
    row(0x0406, "HCI_DISCONNECT", CS, 1, true),
    row(0x041D, "HCI_READ_REMOTE_VERSION_INFORMATION", CS, 1, true),
    row(0x0C01, "HCI_SET_EVENT_MASK", CC, 1, false),
    row(0x0C03, "HCI_RESET", CC, 1, false),
    row(0x0C2D, "HCI_READ_TRANSMIT_POWER_LEVEL", CC, 4, true),
    row(0x0C31, "HCI_SET_CONTROLLER_TO_HOST_FLOW_CONTROL", CC, 1, false),
    row(0x0C33, "HCI_HOST_BUFFER_SIZE", CC, 1, false),
    row(0x0C35, "HCI_HOST_NUMBER_OF_COMPLETED_PACKETS", NONE, 0, false),
    row(0x1001, "HCI_READ_LOCAL_VERSION_INFORMATION", CC, 9, false),
    row(0x1002, "HCI_READ_LOCAL_SUPPORTED_COMMANDS", CC, 65, false),
    row(0x1003, "HCI_READ_LOCAL_SUPPORTED_FEATURES", CC, 9, false),
    row(0x1009, "HCI_READ_BD_ADDR", CC, 7, false),
    row(0x1405, "HCI_READ_RSSI", CC, 4, true),
    row(0x2001, "HCI_LE_SET_EVENT_MASK", CC, 1, false),
    row(0x2002, "HCI_LE_READ_BUFFER_SIZE", CC, 4, false),
    row(0x2003, "HCI_LE_READ_LOCAL_SUPPORTED_FEATURES_PAGE_0", CC, 9, false),
    row(0x2005, "HCI_LE_SET_RANDOM_ADDRESS", CC, 1, false),
    row(0x2006, "HCI_LE_SET_ADVERTISING_PARAMETERS", CC, 1, false),
    row(0x2007, "HCI_LE_READ_ADVERTISING_PHYSICAL_CHANNEL_TX_POWER", CC, 2, false),
    row(0x2008, "HCI_LE_SET_ADVERTISING_DATA", CC, 1, false),
    row(0x2009, "HCI_LE_SET_SCAN_RESPONSE_DATA", CC, 1, false),
    row(0x200A, "HCI_LE_SET_ADVERTISING_ENABLE", CC, 1, false),
    row(0x200B, "HCI_LE_SET_SCAN_PARAMETERS", CC, 1, false),
    row(0x200C, "HCI_LE_SET_SCAN_ENABLE", CC, 1, false),
    row(0x200D, "HCI_LE_CREATE_CONNECTION", CS, 1, false),
    row(0x200E, "HCI_LE_CREATE_CONNECTION_CANCEL", CC, 1, false),
    row(0x200F, "HCI_LE_READ_FILTER_ACCEPT_LIST_SIZE", CC, 2, false),
    row(0x2010, "HCI_LE_CLEAR_FILTER_ACCEPT_LIST", CC, 1, false),
    row(0x2011, "HCI_LE_ADD_DEVICE_TO_FILTER_ACCEPT_LIST", CC, 1, false),
    row(0x2012, "HCI_LE_REMOVE_DEVICE_FROM_FILTER_ACCEPT_LIST", CC, 1, false),
    row(0x2013, "HCI_LE_CONNECTION_UPDATE", CS, 1, true),
    row(0x2014, "HCI_LE_SET_HOST_CHANNEL_CLASSIFICATION", CC, 1, false),
    row(0x2015, "HCI_LE_READ_CHANNEL_MAP", CC, 8, true),
    row(0x2016, "HCI_LE_READ_REMOTE_FEATURES_PAGE_0", CS, 1, true),
    row(0x2017, "HCI_LE_ENCRYPT", CC, 17, false),
    row(0x2018, "HCI_LE_RAND", CC, 9, false),
    row(0x2019, "HCI_LE_ENABLE_ENCRYPTION", CS, 1, true),
    row(0x201A, "HCI_LE_LONG_TERM_KEY_REQUEST_REPLY", CC, 3, true),
    row(0x201B, "HCI_LE_LONG_TERM_KEY_REQUEST_NEGATIVE_REPLY", CC, 3, true),
    row(0x201C, "HCI_LE_READ_SUPPORTED_STATES", CC, 9, false),
    row(0x2022, "HCI_LE_SET_DATA_LENGTH", CC, 1, true),
    row(0x2023, "HCI_LE_READ_SUGGESTED_DEFAULT_DATA_LENGTH", CC, 5, false),
    row(0x2024, "HCI_LE_WRITE_SUGGESTED_DEFAULT_DATA_LENGTH", CC, 1, false),
    row(0x2025, "HCI_LE_READ_LOCAL_P256_PUBLIC_KEY", CS, 1, false),
    row(0x2026, "HCI_LE_GENERATE_DHKEY", CS, 1, false),
    row(0x2027, "HCI_LE_ADD_DEVICE_TO_RESOLVING_LIST", CC, 1, false),
    row(0x2028, "HCI_LE_REMOVE_DEVICE_FROM_RESOLVING_LIST", CC, 1, false),
    row(0x2029, "HCI_LE_CLEAR_RESOLVING_LIST", CC, 1, false),
    row(0x202A, "HCI_LE_READ_RESOLVING_LIST_SIZE", CC, 2, false),
    row(0x202B, "HCI_LE_READ_PEER_RESOLVABLE_ADDRESS", CC, 7, false),
    row(0x202C, "HCI_LE_READ_LOCAL_RESOLVABLE_ADDRESS", CC, 7, false),
    row(0x202D, "HCI_LE_SET_ADDRESS_RESOLUTION_ENABLE", CC, 1, false),
    row(0x202E, "HCI_LE_SET_RESOLVABLE_PRIVATE_ADDRESS_TIMEOUT", CC, 1, false),
    row(0x202F, "HCI_LE_READ_MAXIMUM_DATA_LENGTH", CC, 9, false),
    row(0x2030, "HCI_LE_READ_PHY", CC, 5, true),
    row(0x2031, "HCI_LE_SET_DEFAULT_PHY", CC, 1, false),
    row(0x2032, "HCI_LE_SET_PHY", CS, 1, true),
    row(0x2035, "HCI_LE_SET_ADVERTISING_SET_RANDOM_ADDRESS", CC, 1, false),
    row(0x2036, "HCI_LE_SET_EXTENDED_ADVERTISING_PARAMETERS", CC, 2, false),
    row(0x2037, "HCI_LE_SET_EXTENDED_ADVERTISING_DATA", CC, 1, false),
    row(0x2038, "HCI_LE_SET_EXTENDED_SCAN_RESPONSE_DATA", CC, 1, false),
    row(0x2039, "HCI_LE_SET_EXTENDED_ADVERTISING_ENABLE", CC, 1, false),
    row(0x203A, "HCI_LE_READ_MAXIMUM_ADVERTISING_DATA_LENGTH", CC, 3, false),
    row(0x203B, "HCI_LE_READ_NUMBER_OF_SUPPORTED_ADVERTISING_SETS", CC, 2, false),
    row(0x203C, "HCI_LE_REMOVE_ADVERTISING_SET", CC, 1, false),
    row(0x203D, "HCI_LE_CLEAR_ADVERTISING_SETS", CC, 1, false),
    row(0x2041, "HCI_LE_SET_EXTENDED_SCAN_PARAMETERS", CC, 1, false),
    row(0x2042, "HCI_LE_SET_EXTENDED_SCAN_ENABLE", CC, 1, false),
    row(0x2043, "HCI_LE_EXTENDED_CREATE_CONNECTION", CS, 1, false),
    row(0x204B, "HCI_LE_READ_TRANSMIT_POWER", CC, 3, false),
    row(0x204C, "HCI_LE_READ_RF_PATH_COMPENSATION", CC, 5, false),
    row(0x204D, "HCI_LE_WRITE_RF_PATH_COMPENSATION", CC, 1, false),
    row(0x204E, "HCI_LE_SET_PRIVACY_MODE", CC, 1, false),
    row(0x205E, "HCI_LE_GENERATE_DHKEY_V2", CS, 1, false),
    row(0x209E, "HCI_LE_SET_RESOLVABLE_PRIVATE_ADDRESS_TIMEOUT_V2", CC, 1, false),
    // §2.2 HCI testing commands
    row(0x201D, "HCI_LE_RECEIVER_TEST", CC, 1, false),
    row(0x201E, "HCI_LE_TRANSMITTER_TEST", CC, 1, false),
    row(0x201F, "HCI_LE_TEST_END", CC, 3, false),
    row(0x2033, "HCI_LE_RECEIVER_TEST_V2", CC, 1, false),
    row(0x2034, "HCI_LE_TRANSMITTER_TEST_V2", CC, 1, false),
    // §2.3 HAL commands
    row(0xFC0C, "ACI_HAL_WRITE_CONFIG_DATA", CC, 1, false),
    row(0xFC0D, "ACI_HAL_READ_CONFIG_DATA", CC, 2, false),
    row(0xFC0F, "ACI_HAL_SET_TX_POWER_LEVEL", CC, 1, false),
    row(0xFC14, "ACI_HAL_LE_TX_TEST_PACKET_NUMBER", CC, 5, false),
    row(0xFC15, "ACI_HAL_TONE_START", CC, 1, false),
    row(0xFC16, "ACI_HAL_TONE_STOP", CC, 1, false),
    row(0xFC17, "ACI_HAL_GET_LINK_STATUS", CC, 25, false),
    row(0xFC18, "ACI_HAL_SET_RADIO_ACTIVITY_MASK", CC, 1, false),
    row(0xFC19, "ACI_HAL_GET_ANCHOR_PERIOD", CC, 9, false),
    row(0xFC1A, "ACI_HAL_SET_EVENT_MASK", CC, 1, false),
    row(0xFC20, "ACI_HAL_SET_PERIPHERAL_LATENCY", CC, 1, false),
    row(0xFC22, "ACI_HAL_READ_RSSI", CC, 2, false),
    row(0xFC2F, "ACI_HAL_EAD_ENCRYPT_DECRYPT", CC, 2, false),
    row(0xFC30, "ACI_HAL_READ_RADIO_REG", CC, 1, false),
    row(0xFC31, "ACI_HAL_WRITE_RADIO_REG", CC, 1, false),
    row(0xFC32, "ACI_HAL_READ_RAW_RSSI", CC, 4, false),
    row(0xFC33, "ACI_HAL_RX_START", CC, 1, false),
    row(0xFC34, "ACI_HAL_RX_STOP", CC, 1, false),
    row(0xFCB0, "ACI_HAL_ADDITIONAL_BEACON_START", CC, 1, false),
    row(0xFCB1, "ACI_HAL_ADDITIONAL_BEACON_STOP", CC, 1, false),
    row(0xFCB2, "ACI_HAL_ADDITIONAL_BEACON_SET_DATA", CC, 1, false),
    // §2.4 ACI general commands
    row(0xFF00, "ACI_RESET", CC, 1, false),
    row(0xFF01, "ACI_GET_INFORMATION", CC, 25, false),
    row(0xFF02, "ACI_WRITE_CONFIG_DATA", CC, 1, false),
    row(0xFF03, "ACI_READ_CONFIG_DATA", CC, 2, false),
    // §2.5 GAP commands
    row(0xFC81, "ACI_GAP_SET_NON_DISCOVERABLE", CC, 1, false),
    row(0xFC82, "ACI_GAP_SET_LIMITED_DISCOVERABLE", CC, 1, false),
    row(0xFC83, "ACI_GAP_SET_DISCOVERABLE", CC, 1, false),
    row(0xFC84, "ACI_GAP_SET_DIRECT_CONNECTABLE", CC, 1, false),
    row(0xFC85, "ACI_GAP_SET_IO_CAPABILITY", CC, 1, false),
    row(0xFC86, "ACI_GAP_SET_AUTHENTICATION_REQUIREMENT", CC, 1, false),
    row(0xFC87, "ACI_GAP_SET_AUTHORIZATION_REQUIREMENT", CC, 1, true),
    row(0xFC88, "ACI_GAP_PASS_KEY_RESP", CC, 1, true),
    row(0xFC89, "ACI_GAP_AUTHORIZATION_RESP", CC, 1, true),
    row(0xFC8A, "ACI_GAP_INIT", CC, 7, false),
    row(0xFC8B, "ACI_GAP_SET_NON_CONNECTABLE", CC, 1, false),
    row(0xFC8C, "ACI_GAP_SET_UNDIRECTED_CONNECTABLE", CC, 1, false),
    row(0xFC8D, "ACI_GAP_PERIPHERAL_SECURITY_REQ", CS, 1, true),
    row(0xFC8E, "ACI_GAP_UPDATE_ADV_DATA", CC, 1, false),
    row(0xFC8F, "ACI_GAP_DELETE_AD_TYPE", CC, 1, false),
    row(0xFC90, "ACI_GAP_GET_SECURITY_LEVEL", CC, 3, true),
    row(0xFC91, "ACI_GAP_SET_EVENT_MASK", CC, 1, false),
    row(0xFC92, "ACI_GAP_CONFIGURE_FILTER_ACCEPT_LIST", CC, 1, false),
    row(0xFC93, "ACI_GAP_TERMINATE", CS, 1, true),
    row(0xFC94, "ACI_GAP_CLEAR_SECURITY_DB", CC, 1, false),
    row(0xFC95, "ACI_GAP_ALLOW_REBOND", CC, 1, true),
    row(0xFC96, "ACI_GAP_START_LIMITED_DISCOVERY_PROC", CS, 1, false),
    row(0xFC97, "ACI_GAP_START_GENERAL_DISCOVERY_PROC", CS, 1, false),
    row(0xFC99, "ACI_GAP_START_AUTO_CONNECTION_ESTABLISH_PROC", CS, 1, false),
    row(0xFC9A, "ACI_GAP_START_GENERAL_CONNECTION_ESTABLISH_PROC", CS, 1, false),
    row(0xFC9B, "ACI_GAP_START_SELECTIVE_CONNECTION_ESTABLISH_PROC", CS, 1, false),
    row(0xFC9C, "ACI_GAP_CREATE_CONNECTION", CS, 1, false),
    row(0xFC9D, "ACI_GAP_TERMINATE_GAP_PROC", CC, 1, false),
    row(0xFC9E, "ACI_GAP_START_CONNECTION_UPDATE", CS, 1, true),
    row(0xFC9F, "ACI_GAP_SEND_PAIRING_REQ", CS, 1, true),
    row(0xFCA1, "ACI_GAP_SET_BROADCAST_MODE", CC, 1, false),
    row(0xFCA2, "ACI_GAP_START_OBSERVATION_PROC", CS, 1, false),
    row(0xFCA3, "ACI_GAP_GET_BONDED_DEVICES", CC, 2, false),
    row(0xFCA4, "ACI_GAP_CHECK_BONDED_DEVICE", CC, 8, false),
    row(0xFCA5, "ACI_GAP_NUMERIC_COMPARISON_VALUE_CONFIRM_YESNO", CC, 1, true),
    row(0xFCA6, "ACI_GAP_PASSKEY_INPUT", CC, 1, true),
    row(0xFCA7, "ACI_GAP_GET_OOB_DATA", CC, 26, false),
    row(0xFCA8, "ACI_GAP_SET_OOB_DATA", CC, 1, false),
    row(0xFCAA, "ACI_GAP_REMOVE_BONDED_DEVICE", CC, 1, false),
    row(0xFCAB, "ACI_GAP_ADD_DEVICES_TO_LIST", CC, 1, false),
    row(0xFCAD, "ACI_GAP_PAIRING_REQUEST_REPLY", CC, 1, true),
    row(0xFCC0, "ACI_GAP_ADV_SET_CONFIGURATION", CC, 1, false),
    row(0xFCC1, "ACI_GAP_ADV_SET_ENABLE", CC, 1, false),
    row(0xFCC2, "ACI_GAP_ADV_SET_ADV_DATA", CC, 1, false),
    row(0xFCC3, "ACI_GAP_ADV_SET_SCAN_RESP_DATA", CC, 1, false),
    row(0xFCC4, "ACI_GAP_ADV_REMOVE_SET", CC, 1, false),
    row(0xFCC5, "ACI_GAP_ADV_CLEAR_SETS", CC, 1, false),
    row(0xFCC6, "ACI_GAP_ADV_SET_RANDOM_ADDRESS", CC, 1, false),
    row(0xFCD0, "ACI_GAP_START_SCAN", CS, 1, false),
    row(0xFCD1, "ACI_GAP_EXT_CREATE_CONNECTION", CS, 1, false),
    // §2.6 GATT/ATT commands
    row(0xFD01, "ACI_GATT_INIT", CC, 1, false),
    row(0xFD02, "ACI_GATT_ADD_SERVICE", CC, 3, false),
    row(0xFD03, "ACI_GATT_INCLUDE_SERVICE", CC, 3, false),
    row(0xFD04, "ACI_GATT_ADD_CHAR", CC, 3, false),
    row(0xFD05, "ACI_GATT_ADD_CHAR_DESC", CC, 3, false),
    row(0xFD06, "ACI_GATT_UPDATE_CHAR_VALUE", CC, 1, false),
    row(0xFD07, "ACI_GATT_DEL_CHAR", CC, 1, false),
    row(0xFD08, "ACI_GATT_DEL_SERVICE", CC, 1, false),
    row(0xFD09, "ACI_GATT_DEL_INCLUDE_SERVICE", CC, 1, false),
    row(0xFD0A, "ACI_GATT_SET_EVENT_MASK", CC, 1, false),
    row(0xFD0B, "ACI_GATT_EXCHANGE_CONFIG", CS, 1, true),
    row(0xFD0C, "ACI_ATT_FIND_INFO_REQ", CS, 1, true),
    row(0xFD0D, "ACI_ATT_FIND_BY_TYPE_VALUE_REQ", CS, 1, true),
    row(0xFD0E, "ACI_ATT_READ_BY_TYPE_REQ", CS, 1, true),
    row(0xFD0F, "ACI_ATT_READ_BY_GROUP_TYPE_REQ", CS, 1, true),
    row(0xFD10, "ACI_ATT_PREPARE_WRITE_REQ", CS, 1, true),
    row(0xFD11, "ACI_ATT_EXECUTE_WRITE_REQ", CS, 1, true),
    row(0xFD12, "ACI_GATT_DISC_ALL_PRIMARY_SERVICES", CS, 1, true),
    row(0xFD13, "ACI_GATT_DISC_PRIMARY_SERVICE_BY_UUID", CC, 1, true),
    row(0xFD14, "ACI_GATT_FIND_INCLUDED_SERVICES", CS, 1, true),
    row(0xFD15, "ACI_GATT_DISC_ALL_CHAR_OF_SERVICE", CS, 1, true),
    row(0xFD16, "ACI_GATT_DISC_CHAR_BY_UUID", CS, 1, true),
    row(0xFD17, "ACI_GATT_DISC_ALL_CHAR_DESC", CS, 1, true),
    row(0xFD18, "ACI_GATT_READ_CHAR_VALUE", CS, 1, true),
    row(0xFD19, "ACI_GATT_READ_USING_CHAR_UUID", CS, 1, true),
    row(0xFD1A, "ACI_GATT_READ_LONG_CHAR_VALUE", CS, 1, true),
    row(0xFD1B, "ACI_GATT_READ_MULTIPLE_CHAR_VALUE", CS, 1, true),
    row(0xFD1C, "ACI_GATT_WRITE_CHAR_VALUE", CS, 1, true),
    row(0xFD1D, "ACI_GATT_WRITE_LONG_CHAR_VALUE", CS, 1, true),
    row(0xFD1E, "ACI_GATT_WRITE_CHAR_RELIABLE", CS, 1, true),
    row(0xFD23, "ACI_GATT_WRITE_WITHOUT_RESP", CC, 1, true),
    row(0xFD25, "ACI_GATT_CONFIRM_INDICATION", CC, 1, true),
    row(0xFD26, "ACI_GATT_PERMIT_WRITE", CC, 1, true),
    row(0xFD27, "ACI_GATT_PERMIT_READ", CC, 1, true),
    row(0xFD28, "ACI_GATT_SET_SECURITY_PERMISSION", CC, 1, false),
    row(0xFD29, "ACI_GATT_SET_DESC_VALUE", CC, 1, false),
    row(0xFD2A, "ACI_GATT_READ_HANDLE_VALUE", CC, 5, false),
    row(0xFD2B, "ACI_GATT_GET_ATTRIBUTE_VALUE", CC, 5, false),
    row(0xFD2C, "ACI_GATT_UPDATE_CHAR_VALUE_EXT", CC, 1, false),
    row(0xFD2E, "ACI_GATT_SET_ACCESS_PERMISSION", CC, 1, false),
    row(0xFD30, "ACI_GATT_STORE_DB", CC, 1, false),
    row(0xFD31, "ACI_GATT_SET_MULT_NOTIFICATION", CC, 1, true),
    row(0xFD32, "ACI_GATT_READ_MULTIPLE_VAR_CHAR_VALUE", CC, 1, true),
    row(0xFD40, "ACI_GATT_WRITE_WITHOUT_RESP_EXT", CC, 1, true),
    row(0xFD41, "ACI_GATT_WRITE_WITH_RESP_EXT", CS, 1, true),
    // §2.7 L2CAP commands
    row(0xFD81, "ACI_L2CAP_CONNECTION_PARAMETER_UPDATE_REQ", CS, 1, true),
    row(0xFD82, "ACI_L2CAP_CONNECTION_PARAMETER_UPDATE_RESP", CC, 1, true),
    row(0xFD88, "ACI_L2CAP_COC_CONNECT", CC, 1, true),
    row(0xFD89, "ACI_L2CAP_COC_CONNECT_CONFIRM", CC, 2, true),
    row(0xFD8A, "ACI_L2CAP_COC_RECONF", CC, 1, false),
    row(0xFD8B, "ACI_L2CAP_COC_RECONF_CONFIRM", CC, 1, false),
    row(0xFD8C, "ACI_L2CAP_COC_DISCONNECT", CC, 1, false),
    row(0xFD8D, "ACI_L2CAP_COC_FLOW_CONTROL", CC, 1, false),
    row(0xFD8E, "ACI_L2CAP_COC_TX_DATA", CC, 1, false),
];

/// The row for `opcode`, if AN5270 lists it.
pub(crate) fn lookup(opcode: u16) -> Option<&'static Command> {
    COMMANDS.iter().find(|c| c.opcode == opcode)
}

/// `(opcode, octet, bit)`: where each standard HCI command this model answers
/// sits in `Supported_Commands` (Core Vol 4 Part E §6.27, by way of Pigweed's
/// `hci_events.emb`, Apache-2.0). `HCI_Read_Local_Supported_Commands` itself
/// has no bit.
#[rustfmt::skip]
static SUPPORTED: &[(u16, u8, u8)] = &[
    (0x0406, 0, 5), (0x041D, 2, 7), (0x0C01, 5, 6), (0x0C03, 5, 7),
    (0x0C2D, 10, 2), (0x0C31, 10, 5), (0x0C33, 10, 6), (0x0C35, 10, 7),
    (0x1001, 14, 3), (0x1003, 14, 5), (0x1009, 15, 1), (0x1405, 15, 5),
    (0x2001, 25, 0), (0x2002, 25, 1), (0x2003, 25, 2), (0x2005, 25, 4),
    (0x2006, 25, 5), (0x2007, 25, 6), (0x2008, 25, 7), (0x2009, 26, 0),
    (0x200A, 26, 1), (0x200B, 26, 2), (0x200C, 26, 3), (0x200D, 26, 4),
    (0x200E, 26, 5), (0x200F, 26, 6), (0x2010, 26, 7), (0x2011, 27, 0),
    (0x2012, 27, 1), (0x2013, 27, 2), (0x2014, 27, 3), (0x2015, 27, 4),
    (0x2016, 27, 5), (0x2017, 27, 6), (0x2018, 27, 7), (0x2019, 28, 0),
    (0x201A, 28, 1), (0x201B, 28, 2), (0x201C, 28, 3), (0x201D, 28, 4),
    (0x201E, 28, 5), (0x201F, 28, 6), (0x2022, 33, 6), (0x2023, 33, 7),
    (0x2024, 34, 0), (0x2025, 34, 1), (0x2026, 34, 2), (0x2027, 34, 3),
    (0x2028, 34, 4), (0x2029, 34, 5), (0x202A, 34, 6), (0x202B, 34, 7),
    (0x202C, 35, 0), (0x202D, 35, 1), (0x202E, 35, 2), (0x202F, 35, 3),
    (0x2030, 35, 4), (0x2031, 35, 5), (0x2032, 35, 6), (0x2033, 35, 7),
    (0x2034, 36, 0), (0x2035, 36, 1), (0x2036, 36, 2), (0x2037, 36, 3),
    (0x2038, 36, 4), (0x2039, 36, 5), (0x203A, 36, 6), (0x203B, 36, 7),
    (0x203C, 37, 0), (0x203D, 37, 1), (0x2041, 37, 5), (0x2042, 37, 6),
    (0x2043, 37, 7), (0x204B, 38, 7), (0x204C, 39, 0), (0x204D, 39, 1),
    (0x204E, 39, 2), (0x205E, 41, 2), (0x209E, 48, 2),
];

/// `LE_Features` (Core Vol 6 Part B §4.6, bit order as Pigweed's
/// `LEFeatureSet`): encryption, connection parameter request, extended
/// reject, peripheral-initiated feature exchange, ping, data length
/// extension, LL privacy, extended scanner filter policies, 2M PHY (bits 0–8)
/// and channel selection algorithm #2 (bit 14). **A choice, not a
/// measurement**: what a WB55 BLE 5.x controller plausibly reports. Coded PHY
/// and extended advertising are left clear because they depend on the stack
/// variant and `BLE_Init` options.
const LE_FEATURES: u64 = 0x41FF;

/// The `HCI_Version`/`LMP_Version` assigned number reported: 0x0D, Core 5.4.
/// Unverified against a real part; AN5270 rev 36 is written against Core 6.3
/// (rev 35: 6.2) but Table 16 only says "see Bluetooth assigned numbers".
const HCI_VERSION: u8 = 0x0D;

/// `Company_Identifier`: 0x0030, STMicroelectronics (AN5270 Table 16).
const COMPANY_ST: u16 = 0x0030;

/// The configuration data structure (AN5270 §4.3 and §2.3.1).
mod config {
    /// `CONFIG_DATA_PUBLIC_ADDRESS_OFFSET`, six bytes.
    pub(super) const PUBLIC_ADDRESS: usize = 0x00;
    /// `CONFIG_DATA_ER_OFFSET`, sixteen bytes.
    pub(super) const ER: usize = 0x08;
    /// `CONFIG_DATA_IR_OFFSET`, sixteen bytes.
    pub(super) const IR: usize = 0x18;
    /// `CONFIG_DATA_RANDOM_ADDRESS_OFFSET` — where it is *written*.
    pub(super) const RANDOM_ADDRESS: usize = 0x2E;
    /// `CONFIG_DATA_GAP_ADD_REC_NBR_OFFSET`, one byte.
    pub(super) const GAP_ADD_REC_NBR: usize = 0x34;
    /// `CONFIG_DATA_LL_SCAN_CHAN_MAP_OFFSET`, one byte, reset value 7.
    pub(super) const LL_SCAN_CHAN_MAP: usize = 0xC0;
    /// `CONFIG_DATA_LL_MAX_DATA_EXT_OFFSET`, eight bytes, reset value
    /// `[251, 2120, 251, 2120]`.
    pub(super) const LL_MAX_DATA_EXT: usize = 0xD1;
    /// The read-only offset the static random address is *read* back from
    /// (`ACI_HAL_READ_CONFIG_DATA`, AN5270 Table 141) — not the one it is
    /// written at.
    pub(super) const READ_RANDOM_ADDRESS: u8 = 0x80;
    /// The size of the structure as modelled: every documented offset fits.
    pub(super) const SIZE: usize = 0x100;
}

// ---------------------------------------------------------------------------
// GATT database
// ---------------------------------------------------------------------------

/// `Char_Properties` bits (AN5270 Table 283).
const PROP_BROADCAST: u8 = 0x01;
const PROP_NOTIFY: u8 = 0x10;
const PROP_INDICATE: u8 = 0x20;
const PROP_EXT: u8 = 0x80;

/// What an attribute record is.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum AttrKind {
    /// A primary service declaration (0x2800).
    Primary = 0,
    /// A secondary service declaration (0x2801).
    Secondary = 1,
    /// An include declaration (0x2802).
    Include = 2,
    /// A characteristic declaration (0x2803).
    CharDecl = 3,
    /// A characteristic value.
    CharValue = 4,
    /// A characteristic descriptor.
    Descriptor = 5,
}

impl AttrKind {
    const fn from_u8(v: u8) -> Option<AttrKind> {
        Some(match v {
            0 => AttrKind::Primary,
            1 => AttrKind::Secondary,
            2 => AttrKind::Include,
            3 => AttrKind::CharDecl,
            4 => AttrKind::CharValue,
            5 => AttrKind::Descriptor,
            _ => return None,
        })
    }

    const fn is_service(self) -> bool {
        matches!(self, AttrKind::Primary | AttrKind::Secondary)
    }
}

/// One attribute record.
#[derive(Debug, Clone, PartialEq, Eq)]
pub(crate) struct Attr {
    /// What it is.
    pub(crate) kind: AttrKind,
    /// The handle of the service declaration it belongs to.
    pub(crate) service: u16,
    /// Its UUID, two or sixteen bytes little-endian as the command carried it.
    pub(crate) uuid: Vec<u8>,
    /// The longest the value may be.
    pub(crate) max_len: u16,
    /// Whether the value is variable-length.
    pub(crate) variable: bool,
    /// The value: for a declaration, the declaration's own value as a client
    /// would read it.
    pub(crate) value: Vec<u8>,
}

/// One service's reserved range.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct Range {
    /// The last handle reserved for it (`serviceHandle + max_attr_records - 1`,
    /// AN5270 Table 280).
    end: u16,
    /// The next handle free inside the range.
    next: u16,
}

/// The server's attribute table, with ST's allocation rules.
#[derive(Debug, Clone, Default, PartialEq, Eq)]
pub(crate) struct Gatt {
    /// Whether `ACI_GATT_INIT` has run.
    initialised: bool,
    /// The next handle a new service would get.
    next_handle: u32,
    /// Every record, by handle.
    attrs: BTreeMap<u16, Attr>,
    /// Every service's reserved range, by service handle.
    ranges: BTreeMap<u16, Range>,
}

impl Gatt {
    fn fresh() -> Gatt {
        Gatt {
            initialised: false,
            next_handle: 1,
            attrs: BTreeMap::new(),
            ranges: BTreeMap::new(),
        }
    }

    fn services(&self) -> usize {
        self.ranges.len()
    }

    /// Records counted against `NumAttrRecord`: everything that is not a
    /// service declaration (AN5289 §8.6.5, "excluding the services").
    fn records(&self) -> usize {
        self.attrs.values().filter(|a| !a.kind.is_service()).count()
    }

    fn add_service(
        &mut self,
        uuid: &[u8],
        primary: bool,
        max: u8,
    ) -> core::result::Result<u16, u8> {
        if max == 0 {
            return Err(INVALID_PARAMS);
        }
        let handle = self.next_handle;
        let end = handle + u32::from(max) - 1;
        if end > 0xFFFF {
            return Err(OUT_OF_HANDLES);
        }
        let h = handle as u16;
        self.attrs.insert(
            h,
            Attr {
                kind: if primary {
                    AttrKind::Primary
                } else {
                    AttrKind::Secondary
                },
                service: h,
                uuid: vec![if primary { 0x00 } else { 0x01 }, 0x28],
                max_len: uuid.len() as u16,
                variable: false,
                value: uuid.to_vec(),
            },
        );
        self.ranges.insert(
            h,
            Range {
                end: end as u16,
                next: h.wrapping_add(1),
            },
        );
        self.next_handle = end + 1;
        Ok(h)
    }

    /// Take `count` consecutive handles inside `service`'s range.
    fn reserve(&mut self, service: u16, count: u16) -> core::result::Result<u16, u8> {
        let range = self.ranges.get_mut(&service).ok_or(INVALID_HANDLE)?;
        let first = u32::from(range.next);
        let last = first + u32::from(count) - 1;
        if range.next == 0 || last > u32::from(range.end) {
            return Err(OUT_OF_HANDLES);
        }
        range.next = if last == 0xFFFF { 0 } else { (last + 1) as u16 };
        Ok(first as u16)
    }

    fn put(&mut self, handle: u16, attr: Attr) {
        self.attrs.insert(handle, attr);
    }

    /// The characteristic whose declaration is at `decl`, whatever service
    /// it is in: the commands that take one name the service too, but AN5270
    /// rev 36 says the stack ignores that (Tables 285, 287, 289, 347, 353).
    fn char_decl(&self, decl: u16) -> Option<&Attr> {
        self.attrs
            .get(&decl)
            .filter(|a| a.kind == AttrKind::CharDecl)
    }

    /// Add a characteristic: the declaration, its value at the next handle,
    /// then SCCD, CCCD and the extended properties descriptor in that order
    /// (AN5270 §2.6.4).
    #[allow(clippy::too_many_arguments)]
    fn add_char(
        &mut self,
        service: u16,
        uuid: &[u8],
        max_len: u16,
        props: u8,
        variable: bool,
        records_limit: u16,
    ) -> core::result::Result<u16, u8> {
        if !self.ranges.contains_key(&service) {
            return Err(INVALID_HANDLE);
        }
        let mut need: u16 = 2;
        need += u16::from(props & PROP_BROADCAST != 0);
        need += u16::from(props & (PROP_NOTIFY | PROP_INDICATE) != 0);
        need += u16::from(props & PROP_EXT != 0);
        if records_limit != 0 && self.records() + usize::from(need) > usize::from(records_limit) {
            return Err(INSUFFICIENT_RESOURCES);
        }
        let decl = self.reserve(service, need)?;
        let value = decl.wrapping_add(1);
        let mut decl_value = vec![props];
        decl_value.extend_from_slice(&value.to_le_bytes());
        decl_value.extend_from_slice(uuid);
        self.put(
            decl,
            Attr {
                kind: AttrKind::CharDecl,
                service,
                uuid: vec![0x03, 0x28],
                max_len: decl_value.len() as u16,
                variable: false,
                value: decl_value,
            },
        );
        self.put(
            value,
            Attr {
                kind: AttrKind::CharValue,
                service,
                uuid: uuid.to_vec(),
                max_len,
                variable,
                value: if variable {
                    Vec::new()
                } else {
                    vec![0; usize::from(max_len)]
                },
            },
        );
        let mut next = value.wrapping_add(1);
        let mut descriptor = |gatt: &mut Gatt, uuid: u16| {
            gatt.put(
                next,
                Attr {
                    kind: AttrKind::Descriptor,
                    service,
                    uuid: uuid.to_le_bytes().to_vec(),
                    max_len: 2,
                    variable: false,
                    value: vec![0, 0],
                },
            );
            next = next.wrapping_add(1);
        };
        if props & PROP_BROADCAST != 0 {
            descriptor(self, 0x2903);
        }
        if props & (PROP_NOTIFY | PROP_INDICATE) != 0 {
            descriptor(self, 0x2902);
        }
        if props & PROP_EXT != 0 {
            descriptor(self, 0x2900);
        }
        Ok(decl)
    }

    /// The handle holding `decl`'s value.
    fn value_of(&self, decl: u16) -> core::result::Result<u16, u8> {
        self.char_decl(decl).ok_or(INVALID_HANDLE)?;
        Ok(decl.wrapping_add(1))
    }

    /// Write `data` at `offset` into `handle`, by AN5270 §2.6.6's length rule:
    /// a write at offset zero of a variable-length value sets its length; any
    /// other write sees the value at its full length.
    fn write_value(&mut self, handle: u16, offset: usize, data: &[u8], total: Option<usize>) -> u8 {
        let Some(attr) = self.attrs.get_mut(&handle) else {
            return INVALID_HANDLE;
        };
        let max = usize::from(attr.max_len);
        let end = offset + data.len();
        if end > max || total.is_some_and(|t| t > max) {
            return INVALID_PARAMS;
        }
        if attr.variable && offset == 0 && total.is_none() {
            attr.value = data.to_vec();
            return OK;
        }
        let length = match total {
            Some(t) if attr.variable => t.max(end),
            _ => max,
        };
        attr.value.resize(length, 0);
        attr.value[offset..end].copy_from_slice(data);
        OK
    }

    /// Remove `decl` and everything up to the next declaration in its service.
    fn del_char(&mut self, decl: u16) -> u8 {
        let Some(service) = self.char_decl(decl).map(|a| a.service) else {
            return INVALID_HANDLE;
        };
        let doomed: Vec<u16> = self
            .attrs
            .range(decl..)
            .take_while(|(h, a)| {
                **h == decl
                    || (a.service == service
                        && matches!(a.kind, AttrKind::CharValue | AttrKind::Descriptor))
            })
            .map(|(h, _)| *h)
            .collect();
        for h in doomed {
            self.attrs.remove(&h);
        }
        OK
    }

    fn del_service(&mut self, service: u16) -> u8 {
        let Some(range) = self.ranges.remove(&service) else {
            return INVALID_HANDLE;
        };
        let doomed: Vec<u16> = self
            .attrs
            .range(service..=range.end)
            .map(|(h, _)| *h)
            .collect();
        for h in doomed {
            self.attrs.remove(&h);
        }
        OK
    }

    fn save(&self, w: &mut ChunkWriter<'_>) -> Result<()> {
        w.write_bool(self.initialised)?;
        w.write_u32(self.next_handle)?;
        w.write_seq_len(self.ranges.len() as u64)?;
        for (h, r) in &self.ranges {
            w.write_u16(*h)?;
            w.write_u16(r.end)?;
            w.write_u16(r.next)?;
        }
        w.write_seq_len(self.attrs.len() as u64)?;
        for (h, a) in &self.attrs {
            w.write_u16(*h)?;
            w.write_u8(a.kind as u8)?;
            w.write_u16(a.service)?;
            w.write_bytes(&a.uuid)?;
            w.write_u16(a.max_len)?;
            w.write_bool(a.variable)?;
            w.write_bytes(&a.value)?;
        }
        Ok(())
    }

    fn load(r: &mut ChunkReader<'_>) -> Result<Gatt> {
        let mut g = Gatt::fresh();
        g.initialised = r.read_bool()?;
        g.next_handle = r.read_u32()?;
        if g.next_handle == 0 || g.next_handle > 0x1_0000 {
            return Err(Error::State(alloc::format!(
                "snapshot has a GATT next handle of {:#x}",
                g.next_handle
            )));
        }
        let n = r.read_seq_len(6)?;
        for _ in 0..n {
            let h = r.read_u16()?;
            let end = r.read_u16()?;
            let next = r.read_u16()?;
            g.ranges.insert(h, Range { end, next });
        }
        let n = r.read_seq_len(9)?;
        for _ in 0..n {
            let h = r.read_u16()?;
            let kind = AttrKind::from_u8(r.read_u8()?).ok_or_else(|| {
                Error::State(alloc::string::String::from("bad GATT attribute kind"))
            })?;
            let service = r.read_u16()?;
            let uuid = r.read_bytes()?.to_vec();
            let max_len = r.read_u16()?;
            let variable = r.read_bool()?;
            let value = r.read_bytes()?.to_vec();
            g.attrs.insert(
                h,
                Attr {
                    kind,
                    service,
                    uuid,
                    max_len,
                    variable,
                    value,
                },
            );
        }
        Ok(g)
    }
}

// ---------------------------------------------------------------------------
// The stack
// ---------------------------------------------------------------------------

/// What `SHCI_C2_BLE_Init` configured (AN5289 §8.6.5; NuttX
/// `stm32wb_shci_ble_init_cfg_s`). Only the fields the model enforces are
/// kept; the rest are accepted and ignored.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub(crate) struct BleInit {
    /// `NumAttrRecord`: every non-service attribute record, the nine the GATT
    /// and GAP services bring included. Zero means "not enforced".
    pub(crate) attr_records: u16,
    /// `NumAttrServ`: every service, GATT and GAP included. Zero, not enforced.
    pub(crate) services: u16,
    /// `AttrValueArrSize`: recorded, not enforced (see the module notes).
    pub(crate) value_array: u16,
    /// `NumOfLinks`.
    pub(crate) links: u8,
    /// `Options`: bit 0 LL only, bit 1 *no* Service Changed characteristic,
    /// bit 2 device name read-only, bit 4 CSA #2, bit 7 power class 1.
    pub(crate) options: u8,
}

impl BleInit {
    /// Parse the parameter block, which starts at the first payload byte.
    ///
    /// Offsets from AN5289 §8.6.5's initializer, in order:
    /// `pBleBufferAddress` u32 @0, `BleBufferSize` u32 @4, `NumAttrRecord`
    /// u16 @8, `NumAttrServ` u16 @10, `AttrValueArrSize` u16 @12,
    /// `NumOfLinks` u8 @14, … `Options` u8 @31.
    pub(crate) fn parse(p: &[u8]) -> BleInit {
        let u16_at = |at: usize| {
            p.get(at..at + 2)
                .map_or(0, |b| u16::from_le_bytes([b[0], b[1]]))
        };
        BleInit {
            attr_records: u16_at(8),
            services: u16_at(10),
            value_array: u16_at(12),
            links: p.get(14).copied().unwrap_or(0),
            options: p.get(31).copied().unwrap_or(0),
        }
    }

    const fn service_changed(&self) -> bool {
        self.options & 0x02 == 0
    }
}

/// The GAP service's handles, once `ACI_GAP_INIT` has made it.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) struct GapHandles {
    /// The service declaration.
    pub(crate) service: u16,
    /// The device name characteristic declaration.
    pub(crate) device_name: u16,
    /// The appearance characteristic declaration.
    pub(crate) appearance: u16,
}

/// Who put the controller into advertising.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
#[repr(u8)]
pub enum Advertising {
    /// Not advertising.
    #[default]
    Off = 0,
    /// `ACI_GAP_SET_DISCOVERABLE` / `_LIMITED_DISCOVERABLE`.
    Discoverable = 1,
    /// One of the other `ACI_GAP_SET_*` modes.
    GapMode = 2,
    /// `HCI_LE_SET_ADVERTISING_ENABLE` or the extended-advertising enables.
    Hci = 3,
}

impl Advertising {
    const fn from_u8(v: u8) -> Advertising {
        match v {
            1 => Advertising::Discoverable,
            2 => Advertising::GapMode,
            3 => Advertising::Hci,
            _ => Advertising::Off,
        }
    }
}

/// What one command produced.
#[derive(Debug, Clone, PartialEq, Eq)]
pub(crate) struct Outcome {
    /// The Command Complete / Command Status event, as `[evtcode, plen,
    /// params…]`, or `None` for a command answered by silence.
    pub(crate) reply: Option<Vec<u8>>,
    /// Events that follow the reply, in order, each `[evtcode, plen, …]`.
    pub(crate) events: Vec<Vec<u8>>,
    /// Whether AN5270 lists the opcode at all.
    pub(crate) known: bool,
}

/// The modelled BLE host and controller.
#[derive(Debug, Clone)]
pub(crate) struct Ble {
    /// The public address a reset puts back (the `bd-addr` property),
    /// little-endian as HCI carries it. Configuration, not state.
    default_public: [u8; 6],
    /// The static random address a read of offset 0x80 reports while nothing
    /// was written at 0x2E. Derived from the seed; configuration, not state.
    default_random: [u8; 6],
    /// The stack version word (for `HCI_Read_Local_Version_Information`).
    stack_version: u32,
    /// Whether `SHCI_C2_BLE_Init` has started the stack.
    pub(crate) ready: bool,
    /// What it was started with.
    pub(crate) init: BleInit,
    /// The configuration data structure.
    config: Vec<u8>,
    /// `HCI_Set_Event_Mask`.
    event_mask: u64,
    /// `HCI_LE_Set_Event_Mask`.
    le_event_mask: u64,
    /// `HCI_LE_Set_Random_Address`.
    random_address: [u8; 6],
    /// Who is advertising.
    pub(crate) advertising: Advertising,
    /// The legacy advertising data, as the last command left it.
    pub(crate) adv_data: Vec<u8>,
    /// The legacy scan response data.
    pub(crate) scan_response: Vec<u8>,
    /// Whether an HCI scan is enabled.
    pub(crate) scanning: bool,
    /// Whether an `HCI_LE_Create_Connection` is pending.
    create_pending: bool,
    /// The GAP procedures running, as AN5270 Table 415's procedure codes.
    gap_procs: u8,
    /// The `Procedure` an `ACI_GAP_START_SCAN` is running (0x01, 0x02 or
    /// 0xB0), zero for none. Kept apart from `gap_procs` because 0xB0 is not
    /// one bit, and it ends only by that whole code (AN5270 §2.5.52).
    scan_proc: u8,
    /// `ACI_HAL_SET_TX_POWER_LEVEL`'s `PA_Level`.
    pa_level: u8,
    /// `HCI_LE_Write_Suggested_Default_Data_Length`.
    suggested: (u16, u16),
    /// The GATT server.
    pub(crate) gatt: Gatt,
    /// The GAP service, once made.
    pub(crate) gap: Option<GapHandles>,
    /// Where `HCI_LE_Rand` draws from.
    rand: Stream,
}

/// `HCI_Set_Event_Mask`'s value with every event on: AN5270 Table 385, "By
/// default, all HCI events are enabled".
const EVENT_MASK_ALL: u64 = u64::MAX;
/// `HCI_LE_Set_Event_Mask`'s default (Core Vol 4 Part E §7.8.1): the first
/// five LE events. AN5270 says only that "not all" are on by default.
const LE_EVENT_MASK_DEFAULT: u64 = 0x1F;
/// `Event_Mask` bit 61, the LE Meta event (Core Vol 4 Part E §7.3.1).
const EVENT_MASK_LE_META: u64 = 1 << 61;

impl Ble {
    /// A stack that has not been started.
    pub(crate) fn new(public: [u8; 6], random: [u8; 6], stack_version: u32, rand: Stream) -> Ble {
        let mut ble = Ble {
            default_public: public,
            default_random: random,
            stack_version,
            ready: false,
            init: BleInit::default(),
            config: Vec::new(),
            event_mask: EVENT_MASK_ALL,
            le_event_mask: LE_EVENT_MASK_DEFAULT,
            random_address: [0; 6],
            advertising: Advertising::Off,
            adv_data: Vec::new(),
            scan_response: Vec::new(),
            scanning: false,
            create_pending: false,
            gap_procs: 0,
            scan_proc: 0,
            pa_level: 0x1F,
            suggested: (0x001B, 0x0148),
            gatt: Gatt::fresh(),
            gap: None,
            rand,
        };
        ble.reset_config();
        ble
    }

    /// CPU2 restarted: everything back, the random stream to its seed.
    pub(crate) fn restart(&mut self) {
        let mut rand = self.rand;
        rand.rewind();
        *self = Ble::new(
            self.default_public,
            self.default_random,
            self.stack_version,
            rand,
        );
    }

    /// The configuration data structure's reset values (AN5270 Table 470).
    fn reset_config(&mut self) {
        self.config = vec![0; config::SIZE];
        self.config[config::PUBLIC_ADDRESS..config::PUBLIC_ADDRESS + 6]
            .copy_from_slice(&self.default_public);
        self.config[config::LL_SCAN_CHAN_MAP] = 7;
        let mut ext = Vec::with_capacity(8);
        for v in [251u16, 2120, 251, 2120] {
            ext.extend_from_slice(&v.to_le_bytes());
        }
        self.config[config::LL_MAX_DATA_EXT..config::LL_MAX_DATA_EXT + 8].copy_from_slice(&ext);
    }

    /// The controller half of a reset (`HCI_Reset`): link-layer state and the
    /// configuration data, which AN5270 §2.3.1's note says it resets. The GATT
    /// database is the host's and survives.
    fn reset_controller(&mut self) {
        self.reset_config();
        self.event_mask = EVENT_MASK_ALL;
        self.le_event_mask = LE_EVENT_MASK_DEFAULT;
        self.random_address = [0; 6];
        self.advertising = Advertising::Off;
        self.adv_data.clear();
        self.scan_response.clear();
        self.scanning = false;
        self.create_pending = false;
        self.gap_procs = 0;
        self.scan_proc = 0;
        self.suggested = (0x001B, 0x0148);
    }

    /// The public device address `HCI_Read_BD_ADDR` reports.
    pub(crate) fn public_address(&self) -> [u8; 6] {
        let mut a = [0; 6];
        a.copy_from_slice(&self.config[config::PUBLIC_ADDRESS..config::PUBLIC_ADDRESS + 6]);
        a
    }

    /// The value of the attribute at `handle`.
    pub(crate) fn attribute(&self, handle: u16) -> Option<&Attr> {
        self.gatt.attrs.get(&handle)
    }

    /// Every attribute, by handle.
    pub(crate) fn attributes(&self) -> impl Iterator<Item = (u16, &Attr)> {
        self.gatt.attrs.iter().map(|(h, a)| (*h, a))
    }

    // -----------------------------------------------------------------------
    // Event builders
    // -----------------------------------------------------------------------

    fn complete(opcode: u16, ret: &[u8]) -> Vec<u8> {
        let mut e = vec![EVT_COMMAND_COMPLETE, 0, NUM_HCI_COMMANDS];
        e.extend_from_slice(&opcode.to_le_bytes());
        e.extend_from_slice(ret);
        e[1] = (e.len() - 2) as u8;
        e
    }

    fn status(opcode: u16, status: u8) -> Vec<u8> {
        let mut e = vec![EVT_COMMAND_STATUS, 4, status, NUM_HCI_COMMANDS];
        e.extend_from_slice(&opcode.to_le_bytes());
        e
    }

    fn vendor(ecode: u16, params: &[u8]) -> Vec<u8> {
        let mut e = vec![EVT_VENDOR, (params.len() + 2) as u8];
        e.extend_from_slice(&ecode.to_le_bytes());
        e.extend_from_slice(params);
        e
    }

    fn le_meta(&self, sub: u8, params: &[u8]) -> Option<Vec<u8>> {
        let wanted = self.event_mask & EVENT_MASK_LE_META != 0
            && self.le_event_mask & (1u64 << (sub - 1)) != 0;
        wanted.then(|| {
            let mut e = vec![EVT_LE_META, (params.len() + 1) as u8, sub];
            e.extend_from_slice(params);
            e
        })
    }

    // -----------------------------------------------------------------------
    // Dispatch
    // -----------------------------------------------------------------------

    /// Answer one command.
    pub(crate) fn command(&mut self, opcode: u16, p: &[u8]) -> Outcome {
        let Some(row) = lookup(opcode) else {
            // Core Vol 4 Part E §4.5: an unknown opcode is answered with
            // Unknown HCI Command, here in a Command Complete with the status
            // as its only return parameter.
            return Outcome {
                reply: Some(Ble::complete(opcode, &[UNKNOWN_COMMAND])),
                events: Vec::new(),
                known: false,
            };
        };
        let mut events = Vec::new();
        let reply = match row.reply {
            Reply::Silent => None,
            Reply::Status => Some(Ble::status(opcode, self.started(row, p, &mut events))),
            Reply::Complete => {
                let ret = self.completed(row, p, &mut events);
                Some(Ble::complete(opcode, &ret))
            }
        };
        Outcome {
            reply,
            events,
            known: true,
        }
    }

    /// The status of a command answered by Command Status.
    fn started(&mut self, row: &Command, p: &[u8], events: &mut Vec<Vec<u8>>) -> u8 {
        let _ = events;
        if row.conn {
            return if p.len() < 2 {
                INVALID_HCI_PARAMS
            } else {
                UNKNOWN_CONNECTION
            };
        }
        match row.opcode {
            // HCI_LE_Create_Connection: pending until cancelled. There is no
            // peer to connect to, so it never completes on its own.
            0x200D | 0x2043 => {
                if self.create_pending {
                    DISALLOWED
                } else {
                    self.create_pending = true;
                    OK
                }
            }
            // The local P-256 key and DH key generation are not modelled.
            0x2025 | 0x2026 | 0x205E => UNSUPPORTED,
            // The GAP discovery / connection procedures (AN5270 Table 415).
            0xFC96 => self.start_proc(0x01),
            0xFC97 => self.start_proc(0x02),
            0xFC99 => self.start_proc(0x08),
            0xFC9A => self.start_proc(0x10),
            0xFC9B => self.start_proc(0x20),
            0xFC9C => self.start_proc(0x40),
            0xFCA2 => self.start_proc(0x80),
            0xFCD0 => self.start_scan(p),
            // Extended scanning and connection: accepted, nothing heard.
            _ => OK,
        }
    }

    fn start_proc(&mut self, code: u8) -> u8 {
        if (self.gap_procs | self.scan_proc) & code != 0 {
            return DISALLOWED;
        }
        self.gap_procs |= code;
        OK
    }

    /// `ACI_GAP_START_SCAN`: `Scan_Mode`(1), `Procedure`(1), … (AN5270 rev
    /// 36 Table 273). The procedure runs until `ACI_GAP_TERMINATE_GAP_PROC`
    /// names it. A discovery procedure's 10.24 s timeout (§2.5.52) is not
    /// modelled: nothing here keeps time. A `Procedure` outside the three the
    /// table lists gets Invalid Parameters — an inference, the table being
    /// silent on it.
    fn start_scan(&mut self, p: &[u8]) -> u8 {
        let Some(&code) = p.get(1) else {
            return INVALID_HCI_PARAMS;
        };
        if !matches!(
            code,
            GAP_LIMITED_DISCOVERY | GAP_GENERAL_DISCOVERY | GAP_GENERIC_SCAN
        ) {
            return INVALID_PARAMS;
        }
        if self.scan_proc != 0 || self.gap_procs & code != 0 {
            return DISALLOWED;
        }
        self.scan_proc = code;
        OK
    }

    /// The return parameters of a command answered by Command Complete.
    #[allow(clippy::too_many_lines)]
    fn completed(&mut self, row: &Command, p: &[u8], events: &mut Vec<Vec<u8>>) -> Vec<u8> {
        let mut ret = vec![0u8; usize::from(row.ret)];
        if row.conn {
            // Every return parameter after the status is meaningless; the
            // connection handle, where one is returned, echoes the command's.
            let Some(h) = p.get(..2) else {
                ret[0] = INVALID_HCI_PARAMS;
                return ret;
            };
            ret[0] = UNKNOWN_CONNECTION;
            if ret.len() >= 3 && returns_handle(row.opcode) {
                ret[1..3].copy_from_slice(h);
            }
            return ret;
        }
        let status = match row.opcode {
            0x0C01 => match le_u64(p, 0) {
                Some(m) => {
                    self.event_mask = m;
                    OK
                }
                None => INVALID_HCI_PARAMS,
            },
            0x0C03 => {
                self.reset_controller();
                OK
            }
            0x1001 => {
                // AN5270 Table 16: HCI_Subversion is "the BLE stack reduced
                // version number" — which reduction is unverified; this one
                // packs major.minor.sub into 4+4+4 bits. LMP_Subversion should
                // be the HW version in its high byte and the FW version in
                // its low byte; neither is documented, so it repeats
                // HCI_Subversion. Both unverified.
                let major = (self.stack_version >> 24) & 0xF;
                let minor = (self.stack_version >> 16) & 0xF;
                let sub = (self.stack_version >> 8) & 0xF;
                let subversion = ((major << 8) | (minor << 4) | sub) as u16;
                ret[1] = HCI_VERSION;
                ret[2..4].copy_from_slice(&subversion.to_le_bytes());
                ret[4] = HCI_VERSION;
                ret[5..7].copy_from_slice(&COMPANY_ST.to_le_bytes());
                ret[7..9].copy_from_slice(&subversion.to_le_bytes());
                OK
            }
            0x1002 => {
                for &(op, octet, bit) in SUPPORTED {
                    if lookup(op).is_some() {
                        ret[1 + usize::from(octet)] |= 1 << bit;
                    }
                }
                OK
            }
            0x1003 => {
                // LMP features page 0: bit 37 "BR/EDR Not Supported", bit 38
                // "LE Supported (Controller)" — octet 4, bits 5 and 6.
                ret[1 + 4] = 0x60;
                OK
            }
            0x1009 => {
                ret[1..7].copy_from_slice(&self.public_address());
                OK
            }
            0x2001 => match le_u64(p, 0) {
                Some(m) => {
                    self.le_event_mask = m;
                    OK
                }
                None => INVALID_HCI_PARAMS,
            },
            0x2002 => {
                // HC_LE_ACL_Data_Packet_Length and _Total_Num: 251 octets and
                // eight buffers. Unverified — a plausible WB55 answer.
                ret[1..3].copy_from_slice(&251u16.to_le_bytes());
                ret[3] = 8;
                OK
            }
            0x2003 => {
                ret[1..9].copy_from_slice(&LE_FEATURES.to_le_bytes());
                OK
            }
            0x2005 => match p.get(..6) {
                Some(a) => {
                    self.random_address.copy_from_slice(a);
                    OK
                }
                None => INVALID_HCI_PARAMS,
            },
            0x2007 => {
                ret[1] = 0;
                OK
            }
            0x2008 => match legacy_data(p) {
                Some(d) => {
                    self.adv_data = d;
                    OK
                }
                None => INVALID_HCI_PARAMS,
            },
            0x2009 => match legacy_data(p) {
                Some(d) => {
                    self.scan_response = d;
                    OK
                }
                None => INVALID_HCI_PARAMS,
            },
            0x200A | 0x2039 => match p.first() {
                Some(&on) => {
                    self.advertising = if on != 0 {
                        Advertising::Hci
                    } else {
                        Advertising::Off
                    };
                    OK
                }
                None => INVALID_HCI_PARAMS,
            },
            0x200C | 0x2042 => match p.first() {
                Some(&on) => {
                    self.scanning = on != 0;
                    OK
                }
                None => INVALID_HCI_PARAMS,
            },
            0x200E => {
                if self.create_pending {
                    self.create_pending = false;
                    // Core Vol 4 Part E §7.8.13: a cancelled create ends in an
                    // LE Connection Complete with Unknown Connection
                    // Identifier, after the Command Complete.
                    let mut params = vec![0u8; 18];
                    params[0] = UNKNOWN_CONNECTION;
                    events.extend(self.le_meta(LE_CONNECTION_COMPLETE, &params));
                    OK
                } else {
                    DISALLOWED
                }
            }
            0x200F | 0x202A => {
                // Filter accept list / resolving list size. Unverified.
                ret[1] = 8;
                OK
            }
            0x2017 => match (p.get(..16), p.get(16..32)) {
                (Some(key), Some(text)) => {
                    // Core Vol 4 Part E §7.8.22: the most significant octet of
                    // each is FIPS-197's [0], and HCI carries them least
                    // significant octet first.
                    let mut k = [0u8; 16];
                    let mut b = [0u8; 16];
                    for i in 0..16 {
                        k[i] = key[15 - i];
                        b[i] = text[15 - i];
                    }
                    Aes128::new(&k).encrypt_block(&mut b);
                    for i in 0..16 {
                        ret[1 + i] = b[15 - i];
                    }
                    OK
                }
                _ => INVALID_HCI_PARAMS,
            },
            0x2018 => {
                ret[1..9].copy_from_slice(&self.rand.next_u64().to_le_bytes());
                OK
            }
            0x201C => {
                // LE_States: every combination through bit 41. Unverified.
                ret[1..9].copy_from_slice(&0x0000_03FF_FFFF_FFFFu64.to_le_bytes());
                OK
            }
            0x2023 => {
                ret[1..3].copy_from_slice(&self.suggested.0.to_le_bytes());
                ret[3..5].copy_from_slice(&self.suggested.1.to_le_bytes());
                OK
            }
            0x2024 => match (le_u16(p, 0), le_u16(p, 2)) {
                (Some(o), Some(t)) => {
                    self.suggested = (o, t);
                    OK
                }
                _ => INVALID_HCI_PARAMS,
            },
            0x202F => {
                // supportedMax{Tx,Rx}{Octets,Time}, from LL_MAX_DATA_EXT
                // (AN5270 §4.3).
                ret[1..9].copy_from_slice(
                    &self.config[config::LL_MAX_DATA_EXT..config::LL_MAX_DATA_EXT + 8],
                );
                OK
            }
            0x2036 => {
                ret[1] = 0;
                OK
            }
            0x203A => {
                ret[1..3].copy_from_slice(&31u16.to_le_bytes());
                OK
            }
            0x203B => {
                ret[1] = 1;
                OK
            }
            0x204B => {
                // Min and max Tx power: AN5270 Table 469's ends, -40 and +6 dBm.
                ret[1] = (-40i8) as u8;
                ret[2] = 6;
                OK
            }
            0x201F => {
                ret[1..3].copy_from_slice(&0u16.to_le_bytes());
                OK
            }
            0xFC0C | 0xFF02 => self.write_config(p),
            0xFC0D | 0xFF03 => return self.read_config(p),
            0xFC0F => match p.get(1) {
                Some(&level) if level <= 0x23 => {
                    self.pa_level = level;
                    OK
                }
                Some(_) => INVALID_PARAMS,
                None => INVALID_HCI_PARAMS,
            },
            0xFC22 => {
                // ACI_HAL_READ_RSSI: 127, "not available" in HCI's convention.
                ret[1] = 127;
                OK
            }
            0xFF00 => {
                self.reset_controller();
                self.gatt = Gatt::fresh();
                self.gap = None;
                OK
            }
            0xFF01 => {
                // AN5270 Table 171: Status, Version (8), Options (4),
                // Debug_info (12). Options' bits 0–4 and 7 mean what
                // CFG_BLE_OPTIONS' do (AN5289 §8.6.5), so BLE_Init's options
                // byte goes there; bits 8–9 have no BLE_Init source and stay
                // clear. What fills the eight Version octets is not
                // documented: the stack version word, little-endian, then
                // zeros, is a guess. Debug_info is zeros.
                ret[1..5].copy_from_slice(&self.stack_version.to_le_bytes());
                ret[9..13].copy_from_slice(&u32::from(self.init.options).to_le_bytes());
                OK
            }
            0xFC81 => {
                if matches!(
                    self.advertising,
                    Advertising::Discoverable | Advertising::GapMode
                ) {
                    self.advertising = Advertising::Off;
                }
                OK
            }
            0xFC82 | 0xFC83 => self.set_discoverable(p),
            0xFC84 | 0xFC8B | 0xFC8C | 0xFCA1 => {
                self.advertising = Advertising::GapMode;
                OK
            }
            0xFC8A => return self.gap_init(p),
            0xFC8E => self.update_adv_data(p),
            0xFC8F => match p.first() {
                Some(&t) => {
                    self.adv_data = remove_ad(&self.adv_data, t);
                    OK
                }
                None => INVALID_HCI_PARAMS,
            },
            0xFC9D => match p.first() {
                Some(&code) if code != 0 && code == self.scan_proc => {
                    self.scan_proc = 0;
                    events.push(Ble::vendor(ACI_GAP_PROC_COMPLETE, &[code, OK, 0]));
                    OK
                }
                Some(&code) if self.gap_procs & code != 0 && code.count_ones() == 1 => {
                    self.gap_procs &= !code;
                    // AN5270 §3.3.6: the procedure reports that it ended.
                    events.push(Ble::vendor(ACI_GAP_PROC_COMPLETE, &[code, OK, 0]));
                    OK
                }
                Some(_) => DISALLOWED,
                None => INVALID_HCI_PARAMS,
            },
            0xFCA3 => {
                // No bonded devices: Num_of_Addresses = 0 and nothing after.
                ret.truncate(2);
                ret[1] = 0;
                OK
            }
            0xFCA4 => 0x5C, // "Device not found in DB" (Table 468)
            0xFCAA => 0x5C,
            0xFD01 => self.gatt_init(),
            0xFD02 => return self.add_service(p),
            0xFD03 => return self.include_service(p),
            0xFD04 => return self.add_char(p),
            0xFD05 => return self.add_char_desc(p),
            0xFD06 => self.update_char_value(p),
            // Serv_Handle(2) — ignored, AN5270 rev 36 Table 289 — Char_Handle(2).
            0xFD07 => match (le_u16(p, 0), le_u16(p, 2)) {
                (Some(_), Some(c)) => self.gatt.del_char(c),
                _ => INVALID_HCI_PARAMS,
            },
            0xFD08 => match le_u16(p, 0) {
                Some(s) => self.gatt.del_service(s),
                None => INVALID_HCI_PARAMS,
            },
            0xFD29 => self.set_desc_value(p),
            0xFD2A => return self.read_handle_value(p, 0),
            0xFD2B => return self.read_handle_value(p, 2),
            0xFD2C => self.update_char_value_ext(p),
            _ => OK,
        };
        ret[0] = status;
        if status != OK {
            // A failure carries zeros after the status, at the documented size.
            ret.clear();
            ret.resize(usize::from(row.ret).max(1), 0);
            ret[0] = status;
        }
        ret
    }

    fn write_config(&mut self, p: &[u8]) -> u8 {
        let (Some(&offset), Some(&len)) = (p.first(), p.get(1)) else {
            return INVALID_HCI_PARAMS;
        };
        let Some(value) = p.get(2..2 + usize::from(len)) else {
            return INVALID_HCI_PARAMS;
        };
        let at = usize::from(offset);
        if at + value.len() > config::SIZE {
            return INVALID_PARAMS;
        }
        self.config[at..at + value.len()].copy_from_slice(value);
        OK
    }

    fn read_config(&self, p: &[u8]) -> Vec<u8> {
        let Some(&offset) = p.first() else {
            return vec![INVALID_HCI_PARAMS, 0];
        };
        // AN5270 Table 141: the four readable offsets and their lengths.
        let data: Vec<u8> = match offset {
            0x00 => self.public_address().to_vec(),
            0x08 => self.config[config::ER..config::ER + 16].to_vec(),
            0x18 => self.config[config::IR..config::IR + 16].to_vec(),
            config::READ_RANDOM_ADDRESS => {
                let written = &self.config[config::RANDOM_ADDRESS..config::RANDOM_ADDRESS + 6];
                if written.iter().all(|&b| b == 0) {
                    self.default_random.to_vec()
                } else {
                    written.to_vec()
                }
            }
            _ => return vec![INVALID_PARAMS, 0],
        };
        let mut ret = vec![OK, data.len() as u8];
        ret.extend_from_slice(&data);
        ret
    }

    fn set_discoverable(&mut self, p: &[u8]) -> u8 {
        // AN5270 Table 178/180: Advertising_Type(1), Interval_Min(2),
        // Interval_Max(2), Own_Address_Type(1), Filter_Policy(1),
        // Local_Name_Length(1), Local_Name(n), Service_Uuid_length(1),
        // Service_Uuid_List(n), Conn_Interval_Min(2), Conn_Interval_Max(2).
        let mut at = 7;
        let Some(&name_len) = p.get(at) else {
            return INVALID_HCI_PARAMS;
        };
        at += 1;
        let Some(name) = p.get(at..at + usize::from(name_len)) else {
            return INVALID_HCI_PARAMS;
        };
        at += usize::from(name_len);
        let Some(&uuid_len) = p.get(at) else {
            return INVALID_HCI_PARAMS;
        };
        at += 1;
        let Some(uuids) = p.get(at..at + usize::from(uuid_len)) else {
            return INVALID_HCI_PARAMS;
        };
        // The Flags AD (LE General Discoverable, BR/EDR not supported), then
        // the name and the UUID list, each of which arrives with its AD type
        // as its first byte.
        let mut data = vec![0x02, 0x01, 0x06];
        for part in [name, uuids] {
            if !part.is_empty() {
                data.push(part.len() as u8);
                data.extend_from_slice(part);
            }
        }
        if data.len() > 31 {
            return INVALID_PARAMS;
        }
        self.adv_data = data;
        self.advertising = Advertising::Discoverable;
        OK
    }

    fn update_adv_data(&mut self, p: &[u8]) -> u8 {
        let Some(&len) = p.first() else {
            return INVALID_HCI_PARAMS;
        };
        let Some(new) = p.get(1..1 + usize::from(len)) else {
            return INVALID_HCI_PARAMS;
        };
        // Each AD structure in the update replaces the one of the same type,
        // or is appended.
        let mut data = self.adv_data.clone();
        for ad in ad_structures(new) {
            if let Some(&t) = ad.get(1) {
                data = remove_ad(&data, t);
            }
            data.extend_from_slice(ad);
        }
        if data.len() > 31 {
            return INVALID_PARAMS;
        }
        self.adv_data = data;
        OK
    }

    fn gatt_init(&mut self) -> u8 {
        if self.gatt.initialised {
            return OK;
        }
        self.gatt.initialised = true;
        // The Generic Attribute service, with Service Changed (indicate,
        // four-byte value) unless BLE_Init's option bit 1 removed it.
        let records = if self.init.service_changed() { 4 } else { 1 };
        if let Ok(svc) = self
            .gatt
            .add_service(&0x1801u16.to_le_bytes(), true, records)
            && self.init.service_changed()
        {
            let _ = self
                .gatt
                .add_char(svc, &0x2A05u16.to_le_bytes(), 4, PROP_INDICATE, false, 0);
        }
        OK
    }

    fn gap_init(&mut self, p: &[u8]) -> Vec<u8> {
        let (Some(&role), Some(&name_len)) = (p.first(), p.get(2)) else {
            return vec![INVALID_HCI_PARAMS, 0, 0, 0, 0, 0, 0];
        };
        if !self.gatt.initialised {
            return vec![DISALLOWED, 0, 0, 0, 0, 0, 0];
        }
        let gap = match self.gap {
            Some(g) => g,
            None => {
                let peripheral = role & 0x01 != 0;
                let extra = self.config[config::GAP_ADD_REC_NBR];
                let records = 1 + 2 + 2 + if peripheral { 2 } else { 0 };
                let total = (records + u16::from(extra)).min(255) as u8;
                let made = (|| {
                    let svc = self
                        .gatt
                        .add_service(&0x1800u16.to_le_bytes(), true, total)?;
                    let name = self.gatt.add_char(
                        svc,
                        &0x2A00u16.to_le_bytes(),
                        u16::from(name_len),
                        0x02 | if self.init.options & 0x04 == 0 {
                            0x08
                        } else {
                            0
                        },
                        true,
                        0,
                    )?;
                    let appearance =
                        self.gatt
                            .add_char(svc, &0x2A01u16.to_le_bytes(), 2, 0x02, false, 0)?;
                    if peripheral {
                        self.gatt
                            .add_char(svc, &0x2A04u16.to_le_bytes(), 8, 0x02, false, 0)?;
                    }
                    Ok::<_, u8>(GapHandles {
                        service: svc,
                        device_name: name,
                        appearance,
                    })
                })();
                match made {
                    Ok(g) => {
                        self.gap = Some(g);
                        g
                    }
                    Err(status) => return vec![status, 0, 0, 0, 0, 0, 0],
                }
            }
        };
        let mut ret = vec![OK];
        ret.extend_from_slice(&gap.service.to_le_bytes());
        ret.extend_from_slice(&gap.device_name.to_le_bytes());
        ret.extend_from_slice(&gap.appearance.to_le_bytes());
        ret
    }

    fn add_service(&mut self, p: &[u8]) -> Vec<u8> {
        let fail = |s: u8| vec![s, 0, 0];
        if !self.gatt.initialised {
            return fail(DISALLOWED);
        }
        let Some((uuid, rest)) = uuid_at(p, 0) else {
            return fail(INVALID_HCI_PARAMS);
        };
        let (Some(&kind), Some(&max)) = (p.get(rest), p.get(rest + 1)) else {
            return fail(INVALID_HCI_PARAMS);
        };
        if self.init.services != 0 && self.gatt.services() >= usize::from(self.init.services) {
            return fail(INSUFFICIENT_RESOURCES);
        }
        match self.gatt.add_service(uuid, kind != 0x02, max) {
            Ok(h) => {
                let mut ret = vec![OK];
                ret.extend_from_slice(&h.to_le_bytes());
                ret
            }
            Err(s) => fail(s),
        }
    }

    fn include_service(&mut self, p: &[u8]) -> Vec<u8> {
        let fail = |s: u8| vec![s, 0, 0];
        let (Some(svc), Some(start), Some(end)) = (le_u16(p, 0), le_u16(p, 2), le_u16(p, 4)) else {
            return fail(INVALID_HCI_PARAMS);
        };
        let Some((uuid, _)) = uuid_at(p, 6) else {
            return fail(INVALID_HCI_PARAMS);
        };
        if self.records_full(1) {
            return fail(INSUFFICIENT_RESOURCES);
        }
        match self.gatt.reserve(svc, 1) {
            Ok(h) => {
                let mut value = Vec::new();
                value.extend_from_slice(&start.to_le_bytes());
                value.extend_from_slice(&end.to_le_bytes());
                if uuid.len() == 2 {
                    value.extend_from_slice(uuid);
                }
                self.gatt.put(
                    h,
                    Attr {
                        kind: AttrKind::Include,
                        service: svc,
                        uuid: vec![0x02, 0x28],
                        max_len: value.len() as u16,
                        variable: false,
                        value,
                    },
                );
                let mut ret = vec![OK];
                ret.extend_from_slice(&h.to_le_bytes());
                ret
            }
            Err(s) => fail(s),
        }
    }

    fn records_full(&self, more: usize) -> bool {
        self.init.attr_records != 0
            && self.gatt.records() + more > usize::from(self.init.attr_records)
    }

    fn add_char(&mut self, p: &[u8]) -> Vec<u8> {
        // AN5270 Table 283: Service_Handle(2), Char_UUID_Type(1), UUID(2|16),
        // Char_Value_Length(2), Char_Properties(1), Security_Permissions(1),
        // GATT_Evt_Mask(1), Enc_Key_Size(1), Is_Variable(1).
        let fail = |s: u8| vec![s, 0, 0];
        let Some(svc) = le_u16(p, 0) else {
            return fail(INVALID_HCI_PARAMS);
        };
        let Some((uuid, at)) = uuid_at(p, 2) else {
            return fail(INVALID_HCI_PARAMS);
        };
        let (Some(max_len), Some(&props), Some(&variable)) =
            (le_u16(p, at), p.get(at + 2), p.get(at + 6))
        else {
            return fail(INVALID_HCI_PARAMS);
        };
        match self.gatt.add_char(
            svc,
            uuid,
            max_len,
            props,
            variable != 0,
            self.init.attr_records,
        ) {
            Ok(h) => {
                let mut ret = vec![OK];
                ret.extend_from_slice(&h.to_le_bytes());
                ret
            }
            Err(s) => fail(s),
        }
    }

    fn add_char_desc(&mut self, p: &[u8]) -> Vec<u8> {
        // AN5270 Table 285: Service_Handle(2), Char_Handle(2),
        // Char_Desc_Uuid_Type(1), UUID(2|16), Char_Desc_Value_Max_Length(1),
        // Char_Desc_Value_Length(1), Char_Desc_Value(n),
        // Security_Permissions(1), Access_Permissions(1), GATT_Evt_Mask(1),
        // Enc_Key_Size(1), Is_Variable(1). Rev 36 marks Service_Handle
        // ignored: the descriptor goes into the characteristic's service.
        let fail = |s: u8| vec![s, 0, 0];
        let (Some(_), Some(decl)) = (le_u16(p, 0), le_u16(p, 2)) else {
            return fail(INVALID_HCI_PARAMS);
        };
        let Some((uuid, at)) = uuid_at(p, 4) else {
            return fail(INVALID_HCI_PARAMS);
        };
        let (Some(&max), Some(&len)) = (p.get(at), p.get(at + 1)) else {
            return fail(INVALID_HCI_PARAMS);
        };
        let Some(value) = p.get(at + 2..at + 2 + usize::from(len)) else {
            return fail(INVALID_HCI_PARAMS);
        };
        let Some(&variable) = p.get(at + 2 + usize::from(len) + 4) else {
            return fail(INVALID_HCI_PARAMS);
        };
        if len > max {
            return fail(INVALID_PARAMS);
        }
        let Some(svc) = self.gatt.char_decl(decl).map(|a| a.service) else {
            return fail(INVALID_HANDLE);
        };
        if self.records_full(1) {
            return fail(INSUFFICIENT_RESOURCES);
        }
        match self.gatt.reserve(svc, 1) {
            Ok(h) => {
                self.gatt.put(
                    h,
                    Attr {
                        kind: AttrKind::Descriptor,
                        service: svc,
                        uuid: uuid.to_vec(),
                        max_len: u16::from(max),
                        variable: variable != 0,
                        value: value.to_vec(),
                    },
                );
                let mut ret = vec![OK];
                ret.extend_from_slice(&h.to_le_bytes());
                ret
            }
            Err(s) => fail(s),
        }
    }

    fn update_char_value(&mut self, p: &[u8]) -> u8 {
        // AN5270 Table 287: Service_Handle(2) (ignored since rev 36),
        // Char_Handle(2), Val_Offset(1), Char_Value_Length(1), Char_Value(n).
        let (Some(_), Some(decl), Some(&offset), Some(&len)) =
            (le_u16(p, 0), le_u16(p, 2), p.get(4), p.get(5))
        else {
            return INVALID_HCI_PARAMS;
        };
        let Some(value) = p.get(6..6 + usize::from(len)) else {
            return INVALID_HCI_PARAMS;
        };
        match self.gatt.value_of(decl) {
            Ok(h) => self.gatt.write_value(h, usize::from(offset), value, None),
            Err(s) => s,
        }
    }

    fn update_char_value_ext(&mut self, p: &[u8]) -> u8 {
        // AN5270 Table 353: Conn_Handle_To_Notify(2), Service_Handle(2)
        // (ignored since rev 36), Char_Handle(2), Update_Type(1),
        // Char_Length(2), Value_Offset(2), Value_Length(1), Value(n).
        let (Some(_), Some(decl), Some(total), Some(offset), Some(&len)) = (
            le_u16(p, 2),
            le_u16(p, 4),
            le_u16(p, 7),
            le_u16(p, 9),
            p.get(11),
        ) else {
            return INVALID_HCI_PARAMS;
        };
        let Some(value) = p.get(12..12 + usize::from(len)) else {
            return INVALID_HCI_PARAMS;
        };
        match self.gatt.value_of(decl) {
            Ok(h) => self
                .gatt
                .write_value(h, usize::from(offset), value, Some(usize::from(total))),
            Err(s) => s,
        }
    }

    fn set_desc_value(&mut self, p: &[u8]) -> u8 {
        // AN5270 Table 347: Serv_Handle(2) (ignored since rev 36),
        // Char_Handle(2), Char_Desc_Handle(2), Val_Offset(2),
        // Char_Desc_Value_Length(1), Char_Desc_Value(n).
        let (Some(_), Some(decl), Some(desc), Some(offset), Some(&len)) = (
            le_u16(p, 0),
            le_u16(p, 2),
            le_u16(p, 4),
            le_u16(p, 6),
            p.get(8),
        ) else {
            return INVALID_HCI_PARAMS;
        };
        let Some(value) = p.get(9..9 + usize::from(len)) else {
            return INVALID_HCI_PARAMS;
        };
        let Some(svc) = self.gatt.char_decl(decl).map(|a| a.service) else {
            return INVALID_HANDLE;
        };
        match self.gatt.attrs.get(&desc) {
            Some(a) if a.kind == AttrKind::Descriptor && a.service == svc && desc > decl => {}
            _ => return INVALID_HANDLE,
        }
        self.gatt
            .write_value(desc, usize::from(offset), value, None)
    }

    /// `ACI_GATT_READ_HANDLE_VALUE`, and `ACI_GATT_GET_ATTRIBUTE_VALUE` with
    /// its leading connection handle skipped (`skip` = 2): `Attr_Handle(2)`,
    /// `Offset(2)`, `Value_Length_Requested(2)` → `Length(2)`,
    /// `Value_Length(2)`, `Value`.
    fn read_handle_value(&self, p: &[u8], skip: usize) -> Vec<u8> {
        let (Some(h), Some(offset), Some(want)) =
            (le_u16(p, skip), le_u16(p, skip + 2), le_u16(p, skip + 4))
        else {
            return vec![INVALID_HCI_PARAMS, 0, 0, 0, 0];
        };
        let Some(attr) = self.gatt.attrs.get(&h) else {
            return vec![INVALID_HANDLE, 0, 0, 0, 0];
        };
        let total = attr.value.len();
        let from = usize::from(offset).min(total);
        // What fits in one Command Complete: 255 - 3 (header) - 5.
        let n = usize::from(want).min(total - from).min(247);
        let mut ret = vec![OK];
        ret.extend_from_slice(&(total as u16).to_le_bytes());
        ret.extend_from_slice(&(n as u16).to_le_bytes());
        ret.extend_from_slice(&attr.value[from..from + n]);
        ret
    }

    // -----------------------------------------------------------------------
    // Snapshot
    // -----------------------------------------------------------------------

    pub(crate) fn save(&self, w: &mut ChunkWriter<'_>) -> Result<()> {
        w.write_bool(self.ready)?;
        w.write_u16(self.init.attr_records)?;
        w.write_u16(self.init.services)?;
        w.write_u16(self.init.value_array)?;
        w.write_u8(self.init.links)?;
        w.write_u8(self.init.options)?;
        w.write_bytes(&self.config)?;
        w.write_u64(self.event_mask)?;
        w.write_u64(self.le_event_mask)?;
        w.write_bytes(&self.random_address)?;
        w.write_u8(self.advertising as u8)?;
        w.write_bytes(&self.adv_data)?;
        w.write_bytes(&self.scan_response)?;
        w.write_bool(self.scanning)?;
        w.write_bool(self.create_pending)?;
        w.write_u8(self.gap_procs)?;
        w.write_u8(self.scan_proc)?;
        w.write_u8(self.pa_level)?;
        w.write_u16(self.suggested.0)?;
        w.write_u16(self.suggested.1)?;
        self.gatt.save(w)?;
        match self.gap {
            None => w.write_bool(false)?,
            Some(g) => {
                w.write_bool(true)?;
                w.write_u16(g.service)?;
                w.write_u16(g.device_name)?;
                w.write_u16(g.appearance)?;
            }
        }
        self.rand.save(w)
    }

    pub(crate) fn load(&mut self, r: &mut ChunkReader<'_>) -> Result<()> {
        let bad = |what: &str| Error::State(alloc::format!("snapshot has a bad CPU2 {what}"));
        self.ready = r.read_bool()?;
        self.init = BleInit {
            attr_records: r.read_u16()?,
            services: r.read_u16()?,
            value_array: r.read_u16()?,
            links: r.read_u8()?,
            options: r.read_u8()?,
        };
        let config = r.read_bytes()?;
        if config.len() != config::SIZE {
            return Err(bad("configuration data block"));
        }
        self.config = config.to_vec();
        self.event_mask = r.read_u64()?;
        self.le_event_mask = r.read_u64()?;
        let ra = r.read_bytes()?;
        self.random_address = ra.try_into().map_err(|_| bad("random address"))?;
        self.advertising = Advertising::from_u8(r.read_u8()?);
        self.adv_data = r.read_bytes()?.to_vec();
        self.scan_response = r.read_bytes()?.to_vec();
        self.scanning = r.read_bool()?;
        self.create_pending = r.read_bool()?;
        self.gap_procs = r.read_u8()?;
        self.scan_proc = r.read_u8()?;
        self.pa_level = r.read_u8()?;
        self.suggested = (r.read_u16()?, r.read_u16()?);
        self.gatt = Gatt::load(r)?;
        self.gap = if r.read_bool()? {
            Some(GapHandles {
                service: r.read_u16()?,
                device_name: r.read_u16()?,
                appearance: r.read_u16()?,
            })
        } else {
            None
        };
        self.rand.load(r)
    }
}

/// The commands whose return parameters include the connection handle at
/// octets 1..3 (AN5270's output tables).
const fn returns_handle(opcode: u16) -> bool {
    matches!(opcode, 0x0C2D | 0x1405 | 0x2015 | 0x201A | 0x201B | 0x2030)
}

fn le_u16(p: &[u8], at: usize) -> Option<u16> {
    p.get(at..at + 2).map(|b| u16::from_le_bytes([b[0], b[1]]))
}

fn le_u64(p: &[u8], at: usize) -> Option<u64> {
    let b = p.get(at..at + 8)?;
    let mut a = [0u8; 8];
    a.copy_from_slice(b);
    Some(u64::from_le_bytes(a))
}

/// A UUID type byte at `at` and the two or sixteen bytes after it; returns the
/// UUID and the offset past it.
fn uuid_at(p: &[u8], at: usize) -> Option<(&[u8], usize)> {
    let len = match p.get(at)? {
        0x01 => 2,
        0x02 => 16,
        _ => return None,
    };
    Some((p.get(at + 1..at + 1 + len)?, at + 1 + len))
}

/// `HCI_LE_Set_Advertising_Data`/`_Scan_Response_Data`: a length byte, then
/// thirty-one octets of which that many are significant.
fn legacy_data(p: &[u8]) -> Option<Vec<u8>> {
    let len = usize::from(*p.first()?);
    if len > 31 {
        return None;
    }
    Some(p.get(1..1 + len)?.to_vec())
}

/// The AD structures in `data`, each `[len, type, …]` (Core Vol 3 Part C §11).
fn ad_structures(data: &[u8]) -> impl Iterator<Item = &[u8]> {
    let mut at = 0;
    core::iter::from_fn(move || {
        let len = usize::from(*data.get(at)?);
        if len == 0 {
            return None;
        }
        let ad = data.get(at..at + 1 + len)?;
        at += 1 + len;
        Some(ad)
    })
}

fn remove_ad(data: &[u8], ad_type: u8) -> Vec<u8> {
    let mut out = Vec::new();
    for ad in ad_structures(data) {
        if ad.get(1) != Some(&ad_type) {
            out.extend_from_slice(ad);
        }
    }
    out
}
