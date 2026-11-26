//! The STM32WB's second core, as the firmware on it behaves: a BLE stack that
//! answers every command and never transmits.
//!
//! One class, `st.wb-cpu2`. On a real STM32WB55 the radio is driven by a
//! Cortex-M0+ ("CPU2") running ST's wireless stack, an encrypted binary that
//! only ST's Firmware Upgrade Services will install and that nothing outside
//! ST can run. The application core ("CPU1", the Cortex-M4 `cpu.arm.v7m`
//! models) never touches the radio: it talks to CPU2 through tables in shared
//! SRAM2 and the doorbells of the IPCC (`st.ipcc`). This device sits on the
//! other end of that conversation. It is a **bus master** that reads and
//! writes SRAM2 and IPCC's processor-2 registers exactly as CPU2's firmware
//! would, and it answers in the formats ST documents — so firmware built on
//! ST's `STM32_WPAN` middleware, on NuttX's mailbox driver or on anything else
//! that follows AN5289 boots CPU2, gets its READY event, starts the BLE stack,
//! builds a GATT database and starts advertising. Nobody ever connects.
//!
//! ```text
//! object ipcc "st.ipcc" { }
//! map mem 0x58000C00 = ipcc
//! object cpu2 "st.wb-cpu2" { clock = hclk2, space = mem }
//! wire pwr.c2boot -> cpu2.boot
//! wire ipcc.c2-rx -> cpu2.ipcc-rx
//! wire ipcc.c2-tx -> cpu2.ipcc-tx
//! ```
//!
//! # The mailbox
//!
//! CPU1 places a reference table at the start of SRAM2a (0x2003_0000 unless
//! `FLASH_IPCCDBA` moves it, RM0434 §3.10 and AN5289 §4.10): ten
//! little-endian pointers, of which this model follows four (NuttX
//! `stm32wb_mbox.c`, `stm32wb_mbox_ref_table_s`, cross-checked against AN5185
//! Figure 4, which names offset 0 the device-information table and offset 3
//! the system table):
//!
//! | Word | Table | Fields this model reads (u32 each unless noted) |
//! | --- | --- | --- |
//! | 0 | device information | written by CPU2, see below |
//! | 1 | BLE | command buffer, command-status buffer, event queue, ACL buffer |
//! | 3 | system | command/response buffer, event queue (AN5185 Table 10) |
//! | 4 | memory manager | BLE spare buffer, system spare buffer, event pool, pool size, free-buffer queue, trace pool, trace pool size |
//!
//! Words 2 and 5–9 (Thread, traces, 802.15.4, Zigbee, LLD tests, BLE LLD) are
//! not followed.
//!
//! A **queue** is a circular doubly-linked list of eight-byte nodes
//! `{next, prev}`, its head a node in SRAM2, empty when `head.next == head`
//! (NuttX `stm32wb_mbox_list.h`). Every packet starts with such a node:
//!
//! | Packet | Bytes after the eight-byte node |
//! | --- | --- |
//! | command | type (BLE 0x01, system 0x10), opcode u16, length, parameters |
//! | ACL data | 0x02, handle u16, length u16, data |
//! | event | type (BLE 0x04, system event 0x12, system response 0x11), event code, length, parameters |
//!
//! The BLE and system types are NuttX's `STM32WB_MBOX_*`; the system response
//! type 0x11 and the command/response/event trio are AN5185 Table 11.
//!
//! ## The channels
//!
//! IPCC channel use is NuttX's `STM32WB_MBOX_*_CHANNEL`, matching AN5185
//! Figure 5 for the two FUS uses:
//!
//! | Ch | CPU1 → CPU2 | CPU2 → CPU1 |
//! | --- | --- | --- |
//! | 1 | BLE command in the command buffer | BLE events appended to the BLE event queue |
//! | 2 | system command in the system buffer | system events appended to the system event queue |
//! | 4 | event buffers handed back on the free queue | — |
//! | 6 | ACL data in the ACL buffer | — |
//!
//! **The system channel is half-duplex** (AN5185 §6.1.2): the response
//! overwrites the command in the same buffer, and clearing the channel's
//! CPU1→CPU2 flag is what tells CPU1 it is there. Both FUS and the wireless
//! stack answer with a Command Complete whose `Num_HCI_Command_Packets` is
//! 0xFF (AN5185 §6.3.4).
//!
//! **BLE command responses are not pool buffers.** AN5289 §14.2.2 tells CPU1
//! to hand *every* event back to the memory manager *except* a command
//! response, so the response cannot have come from the pool. This model
//! writes a Command Complete over the command it answers (the command buffer
//! is sized for the largest event anyway) and a Command Status into the
//! command-status buffer — which is exactly a node plus a four-parameter
//! event long (16 bytes in NuttX) — and links that buffer into the event
//! queue. That is an inference from the table layout, and is marked as such
//! below. A host that returns them anyway (NuttX does) is harmless: a buffer
//! handed back that is not from the pool is ignored.
//!
//! Every other event — the READY event, a procedure completing — is carved
//! from the CPU1-provided event pool in fixed 268-byte slots (a node plus the
//! largest event, rounded to a word, which is also NuttX's
//! `STM32WB_MBOX_CMDPKT_BUF_SIZE`), falling back to the spare buffer for its
//! channel when the pool is empty, as AN5289 §14.2.2 says the spares exist to
//! do. When neither is free the event **waits** for CPU1 to hand buffers back
//! on channel 4; memory is never overwritten.
//!
//! CPU2 appends to an event queue only while that channel's CPU2→CPU1 flag is
//! clear — CPU1 has drained the queue — so the two cores never edit one list
//! at once. Everything ready is appended in one batch, the flag set once.
//!
//! # Booting
//!
//! `boot` is `PWR_CR4.C2BOOT`. A rising edge starts CPU2 after `boot-latency`
//! ticks: it reads the reference table, writes the device information table,
//! enables its IPCC interrupts and unmasks the channels it serves, and posts
//! the system READY event — vendor event 0xFF, sub-event 0x9200, one byte:
//! 0x00 wireless stack running, 0x01 FUS running (AN5185 Table 12).
//!
//! RM0434 §6 says `C2BOOT` *permits* CPU2 to boot after a reset or a low-power
//! wake; clearing it does not stop a CPU2 that is running. So a falling edge
//! does nothing, and what stops CPU2 is a system reset ([`Device::reset`]),
//! after which it boots again only when `boot` rises — or at once, if it is
//! still high.
//!
//! The device information table has two layouts. While the wireless stack
//! runs it is AN5289 §4.10's seven words (safe boot version, FUS version, FUS
//! memory size, FUS info, wireless stack version, wireless stack memory size,
//! InfoStack); while FUS runs it is AN5185 Table 9's, which starts with the
//! magic 0xA94656B9. Version words are AN5289 Figure 12's
//! `major[31:24] minor[23:16] sub[15:8] branch[7:4] build[3:0]`.
//!
//! ## FUS
//!
//! Two consecutive `FUS_GET_STATE` commands to the wireless stack restart CPU2
//! in FUS (AN5185 §2.3): the first answers `FUS_STATE_NOT_RUNNING`, the second
//! answers the same and then CPU2 reboots, posting READY with 0x01. In FUS,
//! `FUS_GET_STATE` answers idle and `FUS_START_WS` reboots back into the
//! wireless stack — without the system reset the part performs, because a
//! device does not get to reset the board behind its back. Firmware upgrade
//! and delete fail: there is no image. BLE commands are not acknowledged while
//! FUS runs, which AN5185 §2 gives as one way to tell.
//!
//! # The stack
//!
//! The `ble` submodule answers HCI and ACI commands from a table extracted from AN5270
//! rev 35: Command Complete or Command Status exactly as that table (or, where
//! the two disagree, the Core Specification) says; generic return parameters
//! zero-filled at the documented size; and real behaviour where there is state
//! — the configuration data, the GATT database with ST's handle allocation,
//! advertising data, LE_Rand from the seeded stream, LE_Encrypt by AES-128.
//! Anything that names a connection fails with Unknown Connection Identifier,
//! because none can exist. An opcode AN5270 does not list gets a Command
//! Complete with Unknown HCI Command, and is reported (see *Observing it*).
//!
//! # Time
//!
//! Lazily advanced on its clock domain (CPU2's HCLK2). An IPCC interrupt edge
//! — or, with `poll`, every `poll` ticks — schedules a service pass
//! `latency` ticks later; the scheduler delivers it through
//! [`Device::next_event_tick`]. The input pins touch only atomics, so the
//! IPCC may call them from inside its own register write, and every bus
//! access happens in [`Device::advance_to`] with no lock but this device's
//! engine lock held (ranked [`ENGINE_RANK`], below `DEVICE`, so IPCC's lock
//! nests under it). That is CLAUDE.md's re-entrancy contract.
//!
//! # Observing it
//!
//! [`WbCpu2::notes`] keeps a deterministic ledger of what the model saw and
//! could not honour — an unknown opcode, a buffer handed back that it never
//! lent, a bus fault on a pointer CPU1 supplied — and [`WbCpu2::set_hook`]
//! calls back with each [`Note`] as it happens, outside the engine lock. Both
//! are diagnostics, not machine state: neither is in a snapshot.
//!
//! # What is not modelled, stated plainly
//!
//! * **No radio, no peers, no connections.** Advertising and scanning are
//!   state; no advertising report, connection, pairing, GATT client response
//!   or notification is ever produced. A GAP procedure runs until terminated.
//! * The Thread, 802.15.4, Zigbee, trace and LLD tables and channels 3 and 5.
//! * The local P-256 key and DH key commands answer Unsupported Feature.
//! * HSEM, RNG, flash and clock interaction: real CPU2 takes semaphores, fills
//!   an RNG pool and erases flash for its NVM at boot (AN5289 §5.2), and none
//!   of that happens here. `SHCI_C2_FLASH_EraseActivity`, `SetSystemClock`
//!   and the rest are acknowledged and otherwise ignored.
//! * `AttrValueArrSize` is recorded, not enforced.
//! * The system reset `FUS_START_WS` causes on the part.
//!
//! # Unverified values
//!
//! Each is marked at its definition. The ones a firmware author is most
//! likely to notice: the 268-byte pool slot and the placement of BLE command
//! responses (inferred, above); the wireless stack's answer to
//! `FUS_GET_STATE` (state and error both 0xFE); `HCI_Version` 0x0D; the
//! `HCI_Subversion` derivation; LE buffer size 251 × 8; list sizes of 8; the
//! LE feature and supported-state masks; `ACI_GET_INFORMATION`'s layout;
//! the memory-size words; and the status codes chosen for an exhausted
//! attribute pool (0x64) or a full service range (0x61).
//!
//! # Sources
//!
//! | Document | Answers |
//! | --- | --- |
//! | ST **AN5289** rev 18 | §4.10 the wireless device information table and version format, §5.2 CPU2 start-up, §8.6.5 `SHCI_C2_BLE_Init`, §14.2 the mailbox interface and which events are returned |
//! | ST **AN5185** rev 15 | §6.1 the FUS device information table and the system table, §6.2 channels, §6.3 packet indicators, READY, FUS commands and responses, §2.3 FUS/wireless switching |
//! | ST **AN5270** rev 35 | every HCI/ACI opcode, parameter layout, reply kind, status code and configuration offset |
//! | ST **RM0434** rev 16 | §9 IPCC (C2CR, C2MR reset 0xFFFFFFFF, C2SCR, C2TOC1SR), §6 `PWR_CR4.C2BOOT` |
//! | Bluetooth **Core Specification** Vol 4 Part E | HCI event formats, unknown-command handling, LE_Encrypt byte order |
//! | Apache **NuttX** `arch/arm/src/stm32wb/stm32wb_mbox*.{c,h}`, `stm32wb_blehci.c` (Apache-2.0, ASF) | the reference/BLE/system/memory-manager table field order, list node layout, channel numbers, packet type values, `SHCI` opcodes |
//! | **Pigweed** `pw_bluetooth/hci_events.emb` (Apache-2.0, © 2023 The Pigweed Authors) | the `Supported_Commands` and LE feature bit positions |
//!
//! ST's own `STM32_WPAN` middleware is under ST's SLA0044 and was not read. No
//! emulator source of any licence was consulted (`ROADMAP.md` §1).
//!
//! `no_std + alloc`, no `unsafe`.

mod ble;

use alloc::boxed::Box;
use alloc::collections::{BTreeMap, VecDeque};
use alloc::format;
use alloc::string::String;
use alloc::sync::{Arc, Weak};
use alloc::vec;
use alloc::vec::Vec;
use core::fmt;

use crate::core::device::{Device, DeviceClass, PropertySpec, RealizeCtx, ResetKind, SinkPin};
use crate::core::error::{Error, Result};
use crate::core::props::{Props, ValueKind};
use crate::core::rand::{Stream, derive_seed};
use crate::core::sched::LazyHandle;
use crate::core::space::{AddressSpace, MemAttrs, RequesterId};
use crate::core::state::{ChunkReader, ChunkWriter, Sink, Source};
use crate::core::sync::{AtomicBool, AtomicU64, LockRank, Mutex, Ordering};
use crate::core::value::Width;
use crate::core::wire::{FanIn, Resolve, WireId, WireSink};
use crate::machine::realize::{BindCtx, Instance};
use crate::machine::validate::{ClassSchema, PortDir, PropSchema};

pub use ble::{Advertising, AttrKind};

/// The class name a machine description writes.
pub const CLASS_NAME: &str = "st.wb-cpu2";

/// The snapshot chunk version. Bump it with the encoding, never on its own.
const STATE_VERSION: u32 = 1;

/// The engine lock's rank: above every bus fabric this device may cross and
/// below [`LockRank::DEVICE`], because an IPCC register write takes IPCC's
/// `DEVICE`-ranked lock while the engine is held.
pub const ENGINE_RANK: LockRank = LockRank::new(0x4c90);

/// IPCC's default base on a WB55 (RM0434 Table 2 / CMSIS `IPCC_BASE`).
pub const DEFAULT_IPCC: u64 = 0x5800_0C00;
/// SRAM2a's start, where the reference table is unless `IPCCDBA` moves it.
pub const DEFAULT_REF_TABLE: u64 = 0x2003_0000;
/// PWR's reset value for the public address: AN5270 Table 469, 0x0280E1000000.
pub const DEFAULT_BD_ADDR: u64 = 0x0280_E100_0000;

// IPCC, processor-2 view (RM0434 §9.4).
const C1TOC2SR: u32 = 0x0C;
const C2CR: u32 = 0x10;
const C2MR: u32 = 0x14;
const C2SCR: u32 = 0x18;
const C2TOC1SR: u32 = 0x1C;
const CR_RXOIE: u32 = 1 << 0;
const CR_TXFIE: u32 = 1 << 16;
/// `C2MR` with every channel's both interrupts masked.
const MR_ALL: u32 = 0x003F_003F;

/// The channels (1-based, as ST numbers them).
const CH_BLE: u32 = 1;
const CH_SYS: u32 = 2;
const CH_MM: u32 = 4;
const CH_ACL: u32 = 6;

const fn low(ch: u32) -> u32 {
    1 << (ch - 1)
}
const fn high(ch: u32) -> u32 {
    1 << (16 + ch - 1)
}

// Packet types.
const PKT_BLE_CMD: u8 = 0x01;
const PKT_BLE_EVT: u8 = 0x04;
const PKT_SYS_CMD: u8 = 0x10;
const PKT_SYS_RSP: u8 = 0x11;
const PKT_SYS_EVT: u8 = 0x12;

/// The list node every packet starts with.
const NODE: u32 = 8;
/// One pool slot: a node and the largest event (`8 + 3 + 255`), rounded to a
/// word. **Inferred**, and equal to NuttX's `STM32WB_MBOX_CMDPKT_BUF_SIZE`.
const SLOT: u32 = 268;
/// The most nodes a free-queue walk follows before deciding the list is
/// corrupt. Far more than any pool CPU1 could give.
const WALK_LIMIT: u32 = 4096;

/// The READY sub-event (AN5185 Table 12).
const SUBEVT_READY: u16 = 0x9200;
/// `DeviceInfoTableState`'s valid value (AN5185 Table 9).
const DIT_VALID: u32 = 0xA946_56B9;
/// AN5185 §6.3.4: `NumHCI` in every system response.
const SYS_NUM_HCI: u8 = 0xFF;
/// `FUS_STATE_NOT_RUNNING` (AN5185 Table 16).
const FUS_NOT_RUNNING: u8 = 0xFE;

// System commands (NuttX `stm32wb_mbox_shci.h`; AN5185 Table 13).
const SHCI_FUS_GET_STATE: u16 = 0xFC52;
const SHCI_FUS_FW_UPGRADE: u16 = 0xFC54;
const SHCI_FUS_FW_DELETE: u16 = 0xFC55;
const SHCI_FUS_START_WS: u16 = 0xFC5A;
const SHCI_BLE_INIT: u16 = 0xFC66;

/// The wireless-stack memory-size word: SRAM2b, SRAM2a 1 KB sectors and flash
/// 4 KB sectors (AN5289 Figure 12). A placeholder, not a real stack's
/// footprint.
const WS_MEMORY: u32 = 0x0A0E_0032;
/// The same for FUS. A placeholder.
const FUS_MEMORY: u32 = 0x0000_0008;

// ---------------------------------------------------------------------------
// Notes
// ---------------------------------------------------------------------------

/// Something the model saw and could not, or would not, honour.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub enum Note {
    /// A BLE opcode AN5270 does not list.
    UnknownBleCommand(u16),
    /// A system opcode this model has no answer for; it was acknowledged.
    UnknownSysCommand(u16),
    /// A BLE command arrived before `SHCI_C2_BLE_Init`; it waits.
    BleCommandBeforeInit(u16),
    /// A BLE command arrived while FUS runs; it is not acknowledged.
    BleCommandInFus(u16),
    /// A BLE command packet with a type other than 0x01.
    BadPacketType(u8),
    /// CPU1 handed back a buffer this CPU2 never lent it.
    ForeignBuffer(u32),
    /// A bus access to an address CPU1 supplied faulted.
    BusFault(u32),
    /// ACL data arrived for a connection that does not exist; dropped.
    AclDropped(u16),
    /// The free queue did not close within the walk limit.
    CorruptFreeQueue(u32),
}

/// A callback for each [`Note`], made outside the engine lock.
pub type Hook = Arc<dyn Fn(Note) + Send + Sync>;

// ---------------------------------------------------------------------------
// The port: what the engine needs from the bus
// ---------------------------------------------------------------------------

/// Why a pass stopped short.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Stop {
    /// A bus error at this address.
    Fault(u32),
    /// The topology was busy; try again.
    Retry,
}

type Step<T> = core::result::Result<T, Stop>;

/// Byte- and word-level access to the space CPU2 masters.
trait Port {
    fn read(&mut self, addr: u32, dst: &mut [u8]) -> Step<()>;
    fn write(&mut self, addr: u32, src: &[u8]) -> Step<()>;
    fn read32(&mut self, addr: u32) -> Step<u32>;
    fn write32(&mut self, addr: u32, value: u32) -> Step<()>;
}

/// [`Port`] over an [`AddressSpace`].
struct SpacePort<'a> {
    space: &'a AddressSpace,
    attrs: MemAttrs,
}

fn stop(e: crate::core::error::BusError, addr: u32) -> Stop {
    if matches!(e, crate::core::error::BusError::Retry) {
        Stop::Retry
    } else {
        Stop::Fault(addr)
    }
}

impl Port for SpacePort<'_> {
    fn read(&mut self, addr: u32, dst: &mut [u8]) -> Step<()> {
        match self.space.read_bytes(u64::from(addr), dst, self.attrs) {
            Ok(()) => Ok(()),
            Err(crate::core::error::BusError::Retry) => Err(Stop::Retry),
            Err(_) => {
                // A region that refuses bulk transfers still answers bytes.
                for (i, b) in dst.iter_mut().enumerate() {
                    let at = addr.wrapping_add(i as u32);
                    *b = self
                        .space
                        .read(u64::from(at), Width::U8, self.attrs)
                        .map_err(|e| stop(e, at))? as u8;
                }
                Ok(())
            }
        }
    }

    fn write(&mut self, addr: u32, src: &[u8]) -> Step<()> {
        match self.space.write_bytes(u64::from(addr), src, self.attrs) {
            Ok(()) => Ok(()),
            Err(crate::core::error::BusError::Retry) => Err(Stop::Retry),
            Err(_) => {
                for (i, b) in src.iter().enumerate() {
                    let at = addr.wrapping_add(i as u32);
                    self.space
                        .write(u64::from(at), Width::U8, u64::from(*b), self.attrs)
                        .map_err(|e| stop(e, at))?;
                }
                Ok(())
            }
        }
    }

    fn read32(&mut self, addr: u32) -> Step<u32> {
        self.space
            .read(u64::from(addr), Width::U32, self.attrs)
            .map(|v| v as u32)
            .map_err(|e| stop(e, addr))
    }

    fn write32(&mut self, addr: u32, value: u32) -> Step<()> {
        self.space
            .write(u64::from(addr), Width::U32, u64::from(value), self.attrs)
            .map_err(|e| stop(e, addr))
    }
}

// ---------------------------------------------------------------------------
// Engine state
// ---------------------------------------------------------------------------

/// Which firmware CPU2 is running.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum Firmware {
    /// The wireless (BLE) stack.
    Wireless = 0,
    /// Firmware Upgrade Services.
    Fus = 1,
}

/// Where CPU2 is in its life.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Run {
    /// Held in reset, or never booted.
    Halted,
    /// Booting; READY goes out at this tick.
    Booting(u64),
    /// Serving the mailbox.
    Running,
    /// Stopped on a bad reference table: real CPU2 would be in a fault loop.
    Faulted,
}

/// Where an outgoing event is written.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
enum Target {
    /// A pool slot, or the channel's spare.
    Pool = 0,
    /// Over the BLE command it answers.
    CmdBuf = 1,
    /// The BLE command-status buffer.
    CsBuf = 2,
}

/// One event waiting to be appended to a queue.
#[derive(Debug, Clone, PartialEq, Eq)]
struct Outgoing {
    target: Target,
    /// `[type, evtcode, plen, params…]`.
    body: Vec<u8>,
    /// Delivering it completes the BLE command in flight: clear channel 1.
    ack: bool,
}

/// The pointers CPU2 follows, read from CPU1's tables.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
struct Tables {
    dit: u32,
    ble_table: u32,
    sys_cmd: u32,
    sys_queue: u32,
    spare_ble: u32,
    spare_sys: u32,
    pool: u32,
    pool_size: u32,
    free_queue: u32,
    ble_cmd: u32,
    ble_cs: u32,
    ble_queue: u32,
    ble_acl: u32,
}

impl Tables {
    const WORDS: usize = 13;

    fn words(&self) -> [u32; Tables::WORDS] {
        [
            self.dit,
            self.ble_table,
            self.sys_cmd,
            self.sys_queue,
            self.spare_ble,
            self.spare_sys,
            self.pool,
            self.pool_size,
            self.free_queue,
            self.ble_cmd,
            self.ble_cs,
            self.ble_queue,
            self.ble_acl,
        ]
    }

    fn from_words(w: [u32; Tables::WORDS]) -> Tables {
        Tables {
            dit: w[0],
            ble_table: w[1],
            sys_cmd: w[2],
            sys_queue: w[3],
            spare_ble: w[4],
            spare_sys: w[5],
            pool: w[6],
            pool_size: w[7],
            free_queue: w[8],
            ble_cmd: w[9],
            ble_cs: w[10],
            ble_queue: w[11],
            ble_acl: w[12],
        }
    }
}

/// Configuration from the properties. Not state.
#[derive(Debug, Clone, Copy)]
struct Config {
    ipcc: u32,
    ref_table: u32,
    latency: u64,
    boot_latency: u64,
    poll: u64,
    stack_version: u32,
    fus_version: u32,
    safe_boot_version: u32,
    stack_type: u8,
}

/// Everything CPU2 knows, behind the engine lock.
#[derive(Debug, Clone)]
struct Engine {
    cfg: Config,
    run: Run,
    firmware: Firmware,
    tables: Tables,
    /// Pool slot occupancy, one entry per slot.
    slots: Vec<bool>,
    /// Whether the BLE / system spare is lent out.
    spare_busy: [bool; 2],
    /// Events waiting: `[0]` for channel 1, `[1]` for channel 2.
    queues: [VecDeque<Outgoing>; 2],
    /// A BLE command has been read and its reply not yet delivered.
    in_flight: bool,
    /// Consecutive `FUS_GET_STATE`s.
    fus_streak: u8,
    /// The stack.
    ble: ble::Ble,
    /// Diagnostics; not saved.
    notes: BTreeMap<Note, u64>,
    /// Notes since the last drain, for the hook; not saved.
    fresh: Vec<Note>,
}

impl Engine {
    fn note(&mut self, n: Note) {
        *self.notes.entry(n).or_insert(0) += 1;
        self.fresh.push(n);
    }

    /// Back to reset: nothing booted, nothing pending.
    fn halt(&mut self) {
        self.run = Run::Halted;
        self.firmware = Firmware::Wireless;
        self.reboot_state();
    }

    fn reboot_state(&mut self) {
        self.tables = Tables::default();
        self.slots.clear();
        self.spare_busy = [false; 2];
        self.queues = [VecDeque::new(), VecDeque::new()];
        self.in_flight = false;
        self.fus_streak = 0;
        self.ble.restart();
    }

    /// `boot` rose at `at`.
    fn boot_edge(&mut self, at: u64) {
        if matches!(self.run, Run::Halted | Run::Faulted) {
            self.run = Run::Booting(at.saturating_add(self.cfg.boot_latency));
        }
    }

    /// The tick this engine next wants to run at, from its own state.
    fn next_wake(&self, now: u64) -> Option<u64> {
        match self.run {
            Run::Booting(due) => Some(due),
            Run::Running if self.cfg.poll != 0 => Some(now.saturating_add(self.cfg.poll)),
            _ => None,
        }
    }

    fn ipcc(&self, reg: u32) -> u32 {
        self.cfg.ipcc.wrapping_add(reg)
    }

    // -----------------------------------------------------------------------
    // One pass
    // -----------------------------------------------------------------------

    fn service(&mut self, port: &mut dyn Port, now: u64) -> Step<()> {
        if let Run::Booting(due) = self.run {
            if now < due {
                return Ok(());
            }
            self.boot(port)?;
        }
        if self.run != Run::Running {
            return Ok(());
        }
        let pending = port.read32(self.ipcc(C1TOC2SR))?;
        if pending & low(CH_MM) != 0 {
            self.drain_free(port)?;
            port.write32(self.ipcc(C2SCR), low(CH_MM))?;
        }
        if pending & low(CH_SYS) != 0 {
            self.system_command(port, now)?;
            if self.run != Run::Running {
                return Ok(());
            }
        }
        if pending & low(CH_ACL) != 0 {
            self.acl(port)?;
            port.write32(self.ipcc(C2SCR), low(CH_ACL))?;
        }
        if pending & low(CH_BLE) != 0 && !self.in_flight {
            self.ble_command(port)?;
        }
        self.deliver(port, CH_SYS)?;
        self.deliver(port, CH_BLE)?;
        self.program_masks(port)
    }

    fn boot(&mut self, port: &mut dyn Port) -> Step<()> {
        self.reboot_state();
        let base = self.cfg.ref_table;
        let mut refs = [0u32; 10];
        for (i, w) in refs.iter_mut().enumerate() {
            *w = port.read32(base.wrapping_add(4 * i as u32))?;
        }
        let (dit, ble_table, sys_table, mm) = (refs[0], refs[1], refs[3], refs[4]);
        if dit == 0 || sys_table == 0 || mm == 0 {
            // Nothing to talk through. The real core would spin.
            self.run = Run::Faulted;
            self.note(Note::BusFault(base));
            return Ok(());
        }
        let mut t = Tables {
            dit,
            ble_table,
            sys_cmd: port.read32(sys_table)?,
            sys_queue: port.read32(sys_table.wrapping_add(4))?,
            ..Tables::default()
        };
        let mut mmw = [0u32; 5];
        for (i, w) in mmw.iter_mut().enumerate() {
            *w = port.read32(mm.wrapping_add(4 * i as u32))?;
        }
        t.spare_ble = mmw[0];
        t.spare_sys = mmw[1];
        t.pool = mmw[2];
        t.pool_size = mmw[3];
        t.free_queue = mmw[4];
        self.tables = t;
        self.slots = vec![false; (t.pool_size / SLOT) as usize];
        self.write_dit(port)?;
        port.write32(self.ipcc(C2CR), CR_RXOIE | CR_TXFIE)?;
        let state = match self.firmware {
            Firmware::Wireless => 0x00,
            Firmware::Fus => 0x01,
        };
        let mut ready = vec![PKT_SYS_EVT, ble::EVT_VENDOR, 3];
        ready.extend_from_slice(&SUBEVT_READY.to_le_bytes());
        ready.push(state);
        self.queues[1].push_back(Outgoing {
            target: Target::Pool,
            body: ready,
            ack: false,
        });
        self.run = Run::Running;
        Ok(())
    }

    fn write_dit(&mut self, port: &mut dyn Port) -> Step<()> {
        let c = self.cfg;
        let mut b = Vec::with_capacity(36);
        match self.firmware {
            Firmware::Wireless => {
                // AN5289 §4.10.
                for w in [
                    c.safe_boot_version,
                    c.fus_version,
                    FUS_MEMORY,
                    0,
                    c.stack_version,
                    WS_MEMORY,
                    u32::from(c.stack_type),
                ] {
                    b.extend_from_slice(&w.to_le_bytes());
                }
            }
            Firmware::Fus => {
                // AN5185 Table 9: state, reserved, last FUS state (0 idle),
                // last wireless stack state (0 not started), stack type
                // (1 BLE), then six version/size words.
                b.extend_from_slice(&DIT_VALID.to_le_bytes());
                b.extend_from_slice(&[0, 0x00, 0x00, 0x01]);
                for w in [
                    c.safe_boot_version,
                    c.fus_version,
                    FUS_MEMORY,
                    c.stack_version,
                    WS_MEMORY,
                    u32::from(c.stack_type),
                ] {
                    b.extend_from_slice(&w.to_le_bytes());
                }
            }
        }
        port.write(self.tables.dit, &b)
    }

    // -----------------------------------------------------------------------
    // Channel 4: buffers coming home
    // -----------------------------------------------------------------------

    fn drain_free(&mut self, port: &mut dyn Port) -> Step<()> {
        let head = self.tables.free_queue;
        if head == 0 {
            return Ok(());
        }
        if self.tables.ble_cmd == 0 {
            // So that a command response handed back is recognised as one.
            self.load_ble_table(port)?;
        }
        let mut node = port.read32(head)?;
        let mut walked = 0;
        while node != head {
            if walked == WALK_LIMIT {
                self.note(Note::CorruptFreeQueue(head));
                break;
            }
            walked += 1;
            let next = port.read32(node)?;
            self.release(node);
            node = next;
        }
        // Empty it: head.next = head.prev = head.
        port.write32(head, head)?;
        port.write32(head.wrapping_add(4), head)
    }

    fn release(&mut self, addr: u32) {
        let t = self.tables;
        if addr == t.spare_ble && t.spare_ble != 0 {
            self.spare_busy[0] = false;
            return;
        }
        if addr == t.spare_sys && t.spare_sys != 0 {
            self.spare_busy[1] = false;
            return;
        }
        let off = addr.wrapping_sub(t.pool);
        if addr >= t.pool && off.is_multiple_of(SLOT) && ((off / SLOT) as usize) < self.slots.len()
        {
            let i = (off / SLOT) as usize;
            if self.slots[i] {
                self.slots[i] = false;
                return;
            }
        } else if (addr == t.ble_cmd || addr == t.ble_cs) && addr != 0 {
            // A command response handed back: not ours to free, and harmless.
            return;
        }
        self.note(Note::ForeignBuffer(addr));
    }

    // -----------------------------------------------------------------------
    // Channel 2: system commands
    // -----------------------------------------------------------------------

    fn system_command(&mut self, port: &mut dyn Port, now: u64) -> Step<()> {
        let buf = self.tables.sys_cmd;
        let mut head = [0u8; 4];
        port.read(buf.wrapping_add(NODE), &mut head)?;
        let opcode = u16::from_le_bytes([head[1], head[2]]);
        // The payload area, whatever the length byte says: NuttX sends
        // `SHCI_C2_BLE_Init` with a length of 3 and the parameters after it.
        let mut payload = vec![0u8; 255];
        port.read(buf.wrapping_add(NODE + 4), &mut payload)?;
        if head[0] != PKT_SYS_CMD {
            self.note(Note::BadPacketType(head[0]));
        }
        let (status, ret, reboot) = self.system_answer(opcode, &payload);
        let mut rsp = vec![
            PKT_SYS_RSP,
            ble::EVT_COMMAND_COMPLETE,
            (4 + ret.len()) as u8,
            SYS_NUM_HCI,
        ];
        rsp.extend_from_slice(&opcode.to_le_bytes());
        rsp.push(status);
        rsp.extend_from_slice(&ret);
        port.write(buf.wrapping_add(NODE), &rsp)?;
        port.write32(self.ipcc(C2SCR), low(CH_SYS))?;
        if let Some(fw) = reboot {
            self.firmware = fw;
            self.reboot_state();
            self.run = Run::Booting(now.saturating_add(self.cfg.boot_latency));
            // While it reboots it serves nothing: mask everything.
            port.write32(self.ipcc(C2MR), MR_ALL)?;
        }
        Ok(())
    }

    fn system_answer(&mut self, opcode: u16, p: &[u8]) -> (u8, Vec<u8>, Option<Firmware>) {
        if opcode != SHCI_FUS_GET_STATE {
            self.fus_streak = 0;
        }
        match (self.firmware, opcode) {
            (Firmware::Wireless, SHCI_FUS_GET_STATE) => {
                // AN5185 §2.3: the first answers FUS_STATE_NOT_RUNNING, the
                // second starts FUS. Error byte 0xFE too — unverified.
                self.fus_streak += 1;
                let reboot = (self.fus_streak >= 2).then_some(Firmware::Fus);
                (FUS_NOT_RUNNING, vec![FUS_NOT_RUNNING], reboot)
            }
            (Firmware::Wireless, SHCI_BLE_INIT) => {
                self.ble.init = ble::BleInit::parse(p);
                self.ble.ready = true;
                (0, Vec::new(), None)
            }
            (Firmware::Wireless, 0xFC53..=0xFC5F) => (0x01, Vec::new(), None),
            (Firmware::Wireless, 0xFC60..=0xFC78) => (0, Vec::new(), None),
            (Firmware::Fus, SHCI_FUS_GET_STATE) => (0x00, vec![0x00], None),
            (Firmware::Fus, SHCI_FUS_START_WS) => (0x00, Vec::new(), Some(Firmware::Wireless)),
            (Firmware::Fus, SHCI_FUS_FW_UPGRADE | SHCI_FUS_FW_DELETE) => (0x01, Vec::new(), None),
            (Firmware::Fus, 0xFC58) => (0x00, vec![0x01], None),
            (Firmware::Fus, 0xFC56..=0xFC5F) => (0x00, Vec::new(), None),
            (Firmware::Fus, _) => {
                self.note(Note::UnknownSysCommand(opcode));
                (0x01, Vec::new(), None)
            }
            (Firmware::Wireless, _) => {
                self.note(Note::UnknownSysCommand(opcode));
                (0, Vec::new(), None)
            }
        }
    }

    // -----------------------------------------------------------------------
    // Channel 6: ACL data
    // -----------------------------------------------------------------------

    fn acl(&mut self, port: &mut dyn Port) -> Step<()> {
        if self.tables.ble_acl == 0 {
            self.load_ble_table(port)?;
        }
        let mut h = [0u8; 3];
        if self.tables.ble_acl != 0 {
            port.read(self.tables.ble_acl.wrapping_add(NODE), &mut h)?;
        }
        self.note(Note::AclDropped(u16::from_le_bytes([h[1], h[2]]) & 0x0FFF));
        Ok(())
    }

    // -----------------------------------------------------------------------
    // Channel 1: BLE commands
    // -----------------------------------------------------------------------

    /// The BLE table is filled by `TL_BLE_Init`, after READY (AN5289
    /// Figure 66), so it is read when first needed rather than at boot.
    fn load_ble_table(&mut self, port: &mut dyn Port) -> Step<()> {
        let t = self.tables.ble_table;
        if t == 0 {
            return Ok(());
        }
        self.tables.ble_cmd = port.read32(t)?;
        self.tables.ble_cs = port.read32(t.wrapping_add(4))?;
        self.tables.ble_queue = port.read32(t.wrapping_add(8))?;
        self.tables.ble_acl = port.read32(t.wrapping_add(12))?;
        Ok(())
    }

    fn ble_command(&mut self, port: &mut dyn Port) -> Step<()> {
        self.load_ble_table(port)?;
        if self.firmware == Firmware::Fus || !self.ble.ready {
            let mut op = [0u8; 2];
            if self.tables.ble_cmd != 0 {
                port.read(self.tables.ble_cmd.wrapping_add(NODE + 1), &mut op)?;
            }
            let op = u16::from_le_bytes(op);
            self.note(if self.firmware == Firmware::Fus {
                Note::BleCommandInFus(op)
            } else {
                Note::BleCommandBeforeInit(op)
            });
            return Ok(());
        }
        let buf = self.tables.ble_cmd;
        if buf == 0 || self.tables.ble_queue == 0 {
            self.note(Note::BusFault(self.tables.ble_table));
            return Ok(());
        }
        let mut head = [0u8; 4];
        port.read(buf.wrapping_add(NODE), &mut head)?;
        let opcode = u16::from_le_bytes([head[1], head[2]]);
        let mut params = vec![0u8; usize::from(head[3])];
        port.read(buf.wrapping_add(NODE + 4), &mut params)?;
        let outcome = if head[0] == PKT_BLE_CMD {
            self.ble.command(opcode, &params)
        } else {
            self.note(Note::BadPacketType(head[0]));
            ble::Ble::command_unknown(opcode)
        };
        if !outcome.known {
            self.note(Note::UnknownBleCommand(opcode));
        }
        match outcome.reply {
            Some(event) => {
                let target = if event[0] == ble::EVT_COMMAND_STATUS && self.tables.ble_cs != 0 {
                    Target::CsBuf
                } else {
                    Target::CmdBuf
                };
                let mut body = vec![PKT_BLE_EVT];
                body.extend_from_slice(&event);
                self.queues[0].push_back(Outgoing {
                    target,
                    body,
                    ack: true,
                });
                self.in_flight = true;
            }
            None => port.write32(self.ipcc(C2SCR), low(CH_BLE))?,
        }
        for event in outcome.events {
            let mut body = vec![PKT_BLE_EVT];
            body.extend_from_slice(&event);
            self.queues[0].push_back(Outgoing {
                target: Target::Pool,
                body,
                ack: false,
            });
        }
        Ok(())
    }

    // -----------------------------------------------------------------------
    // Delivery
    // -----------------------------------------------------------------------

    fn alloc(&mut self, ch: u32) -> Option<u32> {
        if let Some(i) = self.slots.iter().position(|used| !used) {
            self.slots[i] = true;
            return Some(self.tables.pool.wrapping_add(i as u32 * SLOT));
        }
        let (spare, which) = if ch == CH_BLE {
            (self.tables.spare_ble, 0)
        } else {
            (self.tables.spare_sys, 1)
        };
        if spare != 0 && !self.spare_busy[which] {
            self.spare_busy[which] = true;
            return Some(spare);
        }
        None
    }

    fn deliver(&mut self, port: &mut dyn Port, ch: u32) -> Step<()> {
        let q = (ch - 1) as usize;
        if self.queues[q].is_empty() {
            return Ok(());
        }
        if port.read32(self.ipcc(C2TOC1SR))? & low(ch) != 0 {
            // CPU1 has not drained the queue yet; TX-free will say when.
            return Ok(());
        }
        let head = if ch == CH_BLE {
            self.tables.ble_queue
        } else {
            self.tables.sys_queue
        };
        let mut sent = false;
        let mut ack = false;
        while let Some(front) = self.queues[q].front() {
            let (target, is_ack) = (front.target, front.ack);
            let addr = match target {
                Target::CmdBuf => self.tables.ble_cmd,
                Target::CsBuf => self.tables.ble_cs,
                Target::Pool => match self.alloc(ch) {
                    Some(a) => a,
                    // Out of buffers: wait for channel 4, in order.
                    None => break,
                },
            };
            let body = self.queues[q]
                .pop_front()
                .map(|o| o.body)
                .unwrap_or_default();
            port.write(addr.wrapping_add(NODE), &body)?;
            // list_add_tail(head, node).
            let tail = port.read32(head.wrapping_add(4))?;
            port.write32(addr, head)?;
            port.write32(addr.wrapping_add(4), tail)?;
            port.write32(tail, addr)?;
            port.write32(head.wrapping_add(4), addr)?;
            sent = true;
            ack |= is_ack;
        }
        if sent {
            port.write32(self.ipcc(C2SCR), high(ch))?;
        }
        if ack {
            // The reply is linked in: the command buffer is CPU1's again.
            port.write32(self.ipcc(C2SCR), low(CH_BLE))?;
            self.in_flight = false;
        }
        Ok(())
    }

    /// `C2MR`: listen on the channels this firmware serves right now, and ask
    /// for TX-free only where an event waits on an occupied channel.
    fn program_masks(&mut self, port: &mut dyn Port) -> Step<()> {
        let mut mr = MR_ALL;
        mr &= !low(CH_SYS);
        mr &= !low(CH_MM);
        if self.firmware == Firmware::Wireless {
            mr &= !low(CH_ACL);
            if self.ble.ready && !self.in_flight {
                mr &= !low(CH_BLE);
            }
        }
        let busy = port.read32(self.ipcc(C2TOC1SR))?;
        for ch in [CH_BLE, CH_SYS] {
            if !self.queues[(ch - 1) as usize].is_empty() && busy & low(ch) != 0 {
                mr &= !high(ch);
            }
        }
        port.write32(self.ipcc(C2MR), mr)
    }

    // -----------------------------------------------------------------------
    // Snapshot
    // -----------------------------------------------------------------------

    fn save(&self, w: &mut ChunkWriter<'_>) -> Result<()> {
        match self.run {
            Run::Halted => w.write_u8(0)?,
            Run::Booting(due) => {
                w.write_u8(1)?;
                w.write_u64(due)?;
            }
            Run::Running => w.write_u8(2)?,
            Run::Faulted => w.write_u8(3)?,
        }
        w.write_u8(self.firmware as u8)?;
        for word in self.tables.words() {
            w.write_u32(word)?;
        }
        w.write_seq_len(self.slots.len() as u64)?;
        for used in &self.slots {
            w.write_bool(*used)?;
        }
        w.write_bool(self.spare_busy[0])?;
        w.write_bool(self.spare_busy[1])?;
        for q in &self.queues {
            w.write_seq_len(q.len() as u64)?;
            for o in q {
                w.write_u8(o.target as u8)?;
                w.write_bytes(&o.body)?;
                w.write_bool(o.ack)?;
            }
        }
        w.write_bool(self.in_flight)?;
        w.write_u8(self.fus_streak)?;
        self.ble.save(w)
    }

    fn load(&mut self, r: &mut ChunkReader<'_>) -> Result<()> {
        let bad = |what: &str| Error::State(format!("snapshot has a bad CPU2 {what}"));
        self.run = match r.read_u8()? {
            0 => Run::Halted,
            1 => Run::Booting(r.read_u64()?),
            2 => Run::Running,
            3 => Run::Faulted,
            _ => return Err(bad("run state")),
        };
        self.firmware = match r.read_u8()? {
            0 => Firmware::Wireless,
            1 => Firmware::Fus,
            _ => return Err(bad("firmware")),
        };
        let mut words = [0u32; Tables::WORDS];
        for w in &mut words {
            *w = r.read_u32()?;
        }
        self.tables = Tables::from_words(words);
        let n = r.read_seq_len(1)?;
        if n != u64::from(self.tables.pool_size / SLOT) {
            return Err(bad("pool"));
        }
        self.slots = (0..n).map(|_| r.read_bool()).collect::<Result<_>>()?;
        self.spare_busy = [r.read_bool()?, r.read_bool()?];
        for slot in &mut self.queues {
            let n = r.read_seq_len(10)?;
            let mut queue = VecDeque::new();
            for _ in 0..n {
                let target = match r.read_u8()? {
                    0 => Target::Pool,
                    1 => Target::CmdBuf,
                    2 => Target::CsBuf,
                    _ => return Err(bad("event target")),
                };
                let body = r.read_bytes()?.to_vec();
                if body.len() < 3 || body.len() > 258 {
                    return Err(bad("event"));
                }
                queue.push_back(Outgoing {
                    target,
                    body,
                    ack: r.read_bool()?,
                });
            }
            *slot = queue;
        }
        self.in_flight = r.read_bool()?;
        self.fus_streak = r.read_u8()?;
        self.ble.load(r)
    }
}

impl ble::Ble {
    /// The reply to a packet that was not a command at all.
    fn command_unknown(opcode: u16) -> ble::Outcome {
        let mut e = vec![ble::EVT_COMMAND_COMPLETE, 4, 1];
        e.extend_from_slice(&opcode.to_le_bytes());
        e.push(ble::UNKNOWN_COMMAND);
        ble::Outcome {
            reply: Some(e),
            events: Vec::new(),
            known: true,
        }
    }
}

// ---------------------------------------------------------------------------
// The device
// ---------------------------------------------------------------------------

/// Everything the pins, the scheduler and the engine share.
struct Shared {
    engine: Mutex<Engine>,
    bus: Mutex<Option<(Weak<AddressSpace>, RequesterId)>>,
    lazy: Mutex<Option<LazyHandle>>,
    hook: Mutex<Option<Hook>>,
    /// The tick serviced up to.
    tick: AtomicU64,
    /// The absolute tick of the next service pass, [`u64::MAX`] for none.
    wake: AtomicU64,
    /// `boot`'s resolved level.
    boot_level: AtomicBool,
    /// The tick of a rising edge not yet seen by the engine, or `u64::MAX`.
    boot_rise: AtomicU64,
    latency: u64,
    /// The seed before the path is mixed in.
    board_seed: u64,
}

impl fmt::Debug for Shared {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("Shared")
            .field("tick", &self.tick.load(Ordering::Relaxed))
            .field("wake", &self.wake.load(Ordering::Relaxed))
            .field("boot", &self.boot_level.load(Ordering::Relaxed))
            .finish_non_exhaustive()
    }
}

impl Shared {
    /// Where "now" is for a pin that just moved: the scheduler's published
    /// tick or this device's own, whichever is later. Takes no lock across
    /// another: the handle is cloned out first.
    fn now(&self) -> u64 {
        let own = self.tick.load(Ordering::Relaxed);
        let handle = self.lazy.lock().clone();
        handle.map_or(own, |h| h.present_tick().max(own))
    }

    fn wake_at(&self, at: u64) {
        self.wake.fetch_min(at, Ordering::Relaxed);
    }

    fn set_boot(&self, high: bool) {
        let was = self.boot_level.swap(high, Ordering::Relaxed);
        if high && !was {
            let now = self.now();
            self.boot_rise.fetch_min(now, Ordering::Relaxed);
            self.wake_at(now);
        }
    }

    fn ring(&self, high: bool) {
        if high {
            self.wake_at(self.now().saturating_add(self.latency));
        }
    }

    fn advance_to(&self, target: u64) {
        loop {
            let due = self.wake.load(Ordering::Relaxed);
            if due == u64::MAX || due > target {
                break;
            }
            if self
                .wake
                .compare_exchange(due, u64::MAX, Ordering::Relaxed, Ordering::Relaxed)
                .is_err()
            {
                continue;
            }
            let now = due.max(self.tick.load(Ordering::Relaxed));
            self.tick.store(now, Ordering::Relaxed);
            self.service(now);
        }
        self.tick.fetch_max(target, Ordering::Relaxed);
    }

    fn service(&self, now: u64) {
        let bus = self
            .bus
            .lock()
            .clone()
            .and_then(|(space, id)| Some((space.upgrade()?, id)));
        let (notes, next, retry) = {
            let mut engine = self.engine.lock();
            let rise = self.boot_rise.swap(u64::MAX, Ordering::Relaxed);
            if rise != u64::MAX {
                engine.boot_edge(rise);
            }
            let mut retry = false;
            if let Some((space, id)) = &bus {
                let mut port = SpacePort {
                    space,
                    attrs: MemAttrs::DEFAULT.with_requester(*id),
                };
                match engine.service(&mut port, now) {
                    Ok(()) => {}
                    Err(Stop::Retry) => retry = true,
                    Err(Stop::Fault(a)) => engine.note(Note::BusFault(a)),
                }
            }
            (
                core::mem::take(&mut engine.fresh),
                engine.next_wake(now),
                retry,
            )
        };
        if let Some(at) = next {
            self.wake_at(at.max(now.saturating_add(1)));
        }
        if retry {
            self.wake_at(now.saturating_add(1));
        }
        if !notes.is_empty() {
            let hook = self.hook.lock().clone();
            if let Some(hook) = hook {
                for n in notes {
                    hook(n);
                }
            }
        }
    }
}

/// The STM32WB CPU2 wireless coprocessor, as its firmware behaves.
#[derive(Debug)]
pub struct WbCpu2 {
    shared: Arc<Shared>,
    pins: Mutex<Vec<Arc<Pin>>>,
}

/// The properties, parsed.
#[derive(Debug, Clone, Copy)]
struct Settings {
    cfg: Config,
    bd_addr: u64,
    seed: u64,
}

impl WbCpu2 {
    /// Validate `props` and build the model.
    ///
    /// # Errors
    ///
    /// [`Error::Property`] if a property is of the wrong kind or value, or
    /// one this class does not know was given.
    pub fn new(props: &Props) -> Result<WbCpu2> {
        let mut r = props.reader();
        r.touch("clock");
        let ipcc = r.or_addr("ipcc", DEFAULT_IPCC)?;
        let ref_table = r.or_addr("ref-table", DEFAULT_REF_TABLE)?;
        let seed = r.or("seed", 0u64)?;
        let bd_addr = r.or("bd-addr", DEFAULT_BD_ADDR)?;
        let latency = r.or("latency", 3200u64)?;
        let boot_latency = r.or("boot-latency", 32_000u64)?;
        let poll = r.or("poll", 0u64)?;
        let stack_version = r.or("stack-version", 0x0111_0301u64)?;
        let fus_version = r.or("fus-version", 0x0102_0001u64)?;
        let stack_type = r.or("stack-type", 0x01u64)?;
        r.finish()?;
        let word = |name: &str, v: u64| {
            u32::try_from(v).map_err(|_| Error::Property(format!("`{name}` is a 32-bit word")))
        };
        if ipcc > u64::from(u32::MAX) || ref_table > u64::from(u32::MAX) {
            return Err(Error::Property(String::from(
                "`ipcc` and `ref-table` are CPU2 bus addresses, which are 32 bits",
            )));
        }
        if bd_addr >> 48 != 0 {
            return Err(Error::Property(String::from("`bd-addr` is 48 bits")));
        }
        if latency == 0 {
            return Err(Error::Property(String::from(
                "`latency` must be at least one tick: a reply cannot precede its command",
            )));
        }
        Ok(WbCpu2::build(Settings {
            cfg: Config {
                ipcc: ipcc as u32,
                ref_table: ref_table as u32,
                latency,
                boot_latency,
                poll,
                stack_version: word("stack-version", stack_version)?,
                fus_version: word("fus-version", fus_version)?,
                safe_boot_version: 0x0101_0001,
                stack_type: u8::try_from(stack_type)
                    .map_err(|_| Error::Property(String::from("`stack-type` is one byte")))?,
            },
            bd_addr,
            seed,
        }))
    }

    /// Build one with every property at its default — the route a test takes.
    #[must_use]
    pub fn with_defaults() -> WbCpu2 {
        WbCpu2::new(&Props::new()).expect("the defaults are valid")
    }

    fn build(s: Settings) -> WbCpu2 {
        let public: [u8; 6] = s.bd_addr.to_le_bytes()[..6].try_into().expect("six bytes");
        let ble = ble::Ble::new(
            public,
            static_random(s.seed),
            s.cfg.stack_version,
            Stream::new(s.seed),
        );
        let engine = Engine {
            cfg: s.cfg,
            run: Run::Halted,
            firmware: Firmware::Wireless,
            tables: Tables::default(),
            slots: Vec::new(),
            spare_busy: [false; 2],
            queues: [VecDeque::new(), VecDeque::new()],
            in_flight: false,
            fus_streak: 0,
            ble,
            notes: BTreeMap::new(),
            fresh: Vec::new(),
        };
        WbCpu2 {
            shared: Arc::new(Shared {
                engine: Mutex::with_rank(ENGINE_RANK, engine),
                bus: Mutex::new(None),
                lazy: Mutex::new(None),
                hook: Mutex::new(None),
                tick: AtomicU64::new(0),
                wake: AtomicU64::new(u64::MAX),
                boot_level: AtomicBool::new(false),
                boot_rise: AtomicU64::new(u64::MAX),
                latency: s.cfg.latency,
                board_seed: s.seed,
            }),
            pins: Mutex::with_rank(LockRank::DEVICE, Vec::new()),
        }
    }

    /// Point the model at the space it masters. Normally done by
    /// [`Instance::bind`] from `space =`.
    pub fn attach_bus(&self, space: &Arc<AddressSpace>, requester: RequesterId) {
        *self.shared.bus.lock() = Some((Arc::downgrade(space), requester));
    }

    /// Re-seed `LE_Rand` from the board seed and this instance's path.
    pub fn seed_from_path(&self, path: &str) {
        let seed = derive_seed(self.shared.board_seed, path);
        let mut e = self.shared.engine.lock();
        let public = e.ble.public_address();
        e.ble = ble::Ble::new(
            public,
            static_random(seed),
            e.cfg.stack_version,
            Stream::new(seed),
        );
    }

    /// Drive `boot`, as `PWR_CR4.C2BOOT` does.
    pub fn set_boot(&self, high: bool) {
        self.shared.set_boot(high);
    }

    /// Drive the IPCC processor-2 RX-occupied interrupt input.
    pub fn set_ipcc_rx(&self, high: bool) {
        self.shared.ring(high);
    }

    /// Drive the IPCC processor-2 TX-free interrupt input.
    pub fn set_ipcc_tx(&self, high: bool) {
        self.shared.ring(high);
    }

    /// Ask for a service pass `latency` ticks from now, as an interrupt edge
    /// would. For a board that wires neither IPCC line and sets no `poll`.
    pub fn kick(&self) {
        self.shared.ring(true);
    }

    /// The tick serviced up to, in CPU2's clock domain.
    #[must_use]
    pub fn tick(&self) -> u64 {
        self.shared.tick.load(Ordering::Relaxed)
    }

    /// Advance to `tick`, doing whatever falls due. What
    /// [`Device::advance_to`] does; a test without a scheduler calls it.
    pub fn advance_to(&self, tick: u64) {
        self.shared.advance_to(tick);
    }

    /// Where CPU2 is in its life.
    #[must_use]
    pub fn run_state(&self) -> Run {
        self.shared.engine.lock().run
    }

    /// Which firmware is running (or would boot).
    #[must_use]
    pub fn firmware(&self) -> Firmware {
        self.shared.engine.lock().firmware
    }

    /// Whether `SHCI_C2_BLE_Init` has started the BLE stack.
    #[must_use]
    pub fn ble_ready(&self) -> bool {
        self.shared.engine.lock().ble.ready
    }

    /// Who is advertising.
    #[must_use]
    pub fn advertising(&self) -> Advertising {
        self.shared.engine.lock().ble.advertising
    }

    /// The legacy advertising data as the last command left it.
    #[must_use]
    pub fn adv_data(&self) -> Vec<u8> {
        self.shared.engine.lock().ble.adv_data.clone()
    }

    /// The public device address `HCI_Read_BD_ADDR` reports, little-endian.
    #[must_use]
    pub fn public_address(&self) -> [u8; 6] {
        self.shared.engine.lock().ble.public_address()
    }

    /// The GATT server's records: `(handle, kind, uuid, value)`, in handle
    /// order — for a host-side inspector.
    #[must_use]
    pub fn gatt(&self) -> Vec<(u16, AttrKind, Vec<u8>, Vec<u8>)> {
        self.shared
            .engine
            .lock()
            .ble
            .attributes()
            .map(|(h, a)| (h, a.kind, a.uuid.clone(), a.value.clone()))
            .collect()
    }

    /// The value of the attribute at `handle`.
    #[must_use]
    pub fn attribute_value(&self, handle: u16) -> Option<Vec<u8>> {
        self.shared
            .engine
            .lock()
            .ble
            .attribute(handle)
            .map(|a| a.value.clone())
    }

    /// Pool slots currently lent to CPU1.
    #[must_use]
    pub fn buffers_lent(&self) -> usize {
        self.shared
            .engine
            .lock()
            .slots
            .iter()
            .filter(|u| **u)
            .count()
    }

    /// Events waiting for a buffer or a free channel: `(BLE, system)`.
    #[must_use]
    pub fn events_waiting(&self) -> (usize, usize) {
        let e = self.shared.engine.lock();
        (e.queues[0].len(), e.queues[1].len())
    }

    /// Every [`Note`] so far, with how often.
    #[must_use]
    pub fn notes(&self) -> Vec<(Note, u64)> {
        self.shared
            .engine
            .lock()
            .notes
            .iter()
            .map(|(n, c)| (*n, *c))
            .collect()
    }

    /// Call `hook` with every new [`Note`], outside the engine lock. It must
    /// not wait on this device.
    pub fn set_hook(&self, hook: Option<Hook>) {
        *self.shared.hook.lock() = hook;
    }
}

/// The AN5270 name of a BLE opcode, for a trace of [`Note::UnknownBleCommand`]
/// neighbours or a host-side log. `None` for an opcode AN5270 does not list.
#[must_use]
pub fn command_name(opcode: u16) -> Option<&'static str> {
    ble::lookup(opcode).map(|c| c.name)
}

/// A static random address (Core Vol 6 Part B §1.3.2.1: top two bits set),
/// from the seed. What `ACI_HAL_READ_CONFIG_DATA` at 0x80 reports while none
/// was written; the real part derives one from its UID.
fn static_random(seed: u64) -> [u8; 6] {
    let mut s = Stream::new(seed ^ 0x5354_4154_4943_5241);
    let mut a = [0u8; 6];
    a.copy_from_slice(&s.next_u64().to_le_bytes()[..6]);
    a[5] |= 0xC0;
    a
}

/// An input pin, as something a wire can drive.
#[derive(Debug)]
struct Pin {
    shared: Arc<Shared>,
    which: Input,
    inputs: FanIn,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Input {
    Boot,
    Rx,
    Tx,
}

impl WireSink for Pin {
    fn set_level(&self, src: WireId, _line: u32, level: crate::core::wire::Level) {
        self.inputs.set(src, level);
        let high = self.inputs.resolve(Resolve::Or).is_high();
        // Atomics only: the IPCC may be inside its own register write.
        match self.which {
            Input::Boot => self.shared.set_boot(high),
            Input::Rx | Input::Tx => self.shared.ring(high),
        }
    }
}

impl Device for WbCpu2 {
    fn class(&self) -> &'static DeviceClass {
        &CLASS
    }

    fn realize(&self, ctx: &mut RealizeCtx<'_>) -> Result<()> {
        self.seed_from_path(ctx.path());
        Ok(())
    }

    fn reset(&self, _kind: ResetKind) {
        // A system reset resets CPU2. The tick survives: it is this device's
        // cursor in its domain, not architectural state.
        self.shared.engine.lock().halt();
        self.shared.boot_rise.store(u64::MAX, Ordering::Relaxed);
        self.shared.wake.store(u64::MAX, Ordering::Relaxed);
        if self.shared.boot_level.load(Ordering::Relaxed) {
            // RM0434 §6: CPU2 boots after reset once C2BOOT is set.
            let now = self.shared.tick.load(Ordering::Relaxed);
            self.shared.boot_rise.store(now, Ordering::Relaxed);
            self.shared.wake_at(now);
        }
    }

    fn save(&self, w: &mut ChunkWriter<'_>) -> Result<()> {
        let s = &self.shared;
        w.write_u64(s.tick.load(Ordering::Relaxed))?;
        w.write_u64(s.wake.load(Ordering::Relaxed))?;
        w.write_bool(s.boot_level.load(Ordering::Relaxed))?;
        w.write_u64(s.boot_rise.load(Ordering::Relaxed))?;
        s.engine.lock().save(w)
    }

    fn load(&self, r: &mut ChunkReader<'_>) -> Result<()> {
        let s = &self.shared;
        let tick = r.read_u64()?;
        let wake = r.read_u64()?;
        let boot = r.read_bool()?;
        let rise = r.read_u64()?;
        {
            let mut e = s.engine.lock();
            let mut fresh = e.clone();
            fresh.load(r)?;
            fresh.notes = core::mem::take(&mut e.notes);
            *e = fresh;
        }
        s.tick.store(tick, Ordering::Relaxed);
        s.wake.store(wake, Ordering::Relaxed);
        s.boot_level.store(boot, Ordering::Relaxed);
        s.boot_rise.store(rise, Ordering::Relaxed);
        Ok(())
    }

    fn sink(&self, port: &str, sources: &[WireId]) -> Option<SinkPin> {
        let which = match port {
            "boot" => Input::Boot,
            "ipcc-rx" => Input::Rx,
            "ipcc-tx" => Input::Tx,
            _ => return None,
        };
        let pin = Arc::new(Pin {
            shared: Arc::clone(&self.shared),
            which,
            inputs: FanIn::new(sources),
        });
        self.pins.lock().push(Arc::clone(&pin));
        Some(SinkPin { sink: pin, line: 0 })
    }

    fn is_lazy(&self) -> bool {
        true
    }

    fn current_tick(&self) -> u64 {
        self.shared.tick.load(Ordering::Relaxed)
    }

    fn advance_to(&self, tick: u64) {
        self.shared.advance_to(tick);
    }

    fn next_event_tick(&self) -> Option<u64> {
        let tick = self.shared.tick.load(Ordering::Relaxed);
        match self.shared.wake.load(Ordering::Relaxed) {
            u64::MAX => None,
            at => Some(at.max(tick + 1)),
        }
    }

    fn attach_lazy(&self, handle: LazyHandle) {
        *self.shared.lazy.lock() = Some(handle);
    }
}

/// The machine layer's half: CPU2 masters the bus it shares with CPU1.
impl Instance for WbCpu2 {
    fn bind(&self, ctx: &BindCtx<'_>) -> Result<()> {
        let space = ctx.space().ok_or_else(|| Error::Config {
            at: String::from(ctx.path()),
            message: String::from(
                "CPU2 reads and writes SRAM2 and the IPCC's processor-2 registers: add \
                 `space = mem` to the object that declares it",
            ),
        })?;
        self.attach_bus(space, ctx.requester());
        Ok(())
    }
}

/// The `st.wb-cpu2` device class.
pub static CLASS: DeviceClass = DeviceClass {
    name: CLASS_NAME,
    version: STATE_VERSION,
    summary: "STM32WB CPU2: the wireless coprocessor and its BLE stack, behaviourally, with no radio",
    properties: &[
        PropertySpec {
            name: "ipcc",
            kind: ValueKind::Addr,
            required: false,
            summary: "the IPCC's base on CPU2's bus (default 0x58000C00)",
        },
        PropertySpec {
            name: "ref-table",
            kind: ValueKind::Addr,
            required: false,
            summary: "where CPU1 puts the mailbox reference table (default 0x20030000, SRAM2a)",
        },
        PropertySpec {
            name: "seed",
            kind: ValueKind::Uint,
            required: false,
            summary: "the board seed LE_Rand and the default static random address draw from",
        },
        PropertySpec {
            name: "bd-addr",
            kind: ValueKind::Uint,
            required: false,
            summary: "the public address a reset restores (default 0x0280E1000000, AN5270 Table 469)",
        },
        PropertySpec {
            name: "latency",
            kind: ValueKind::Uint,
            required: false,
            summary: "ticks from an IPCC doorbell to CPU2's answer (default 3200)",
        },
        PropertySpec {
            name: "boot-latency",
            kind: ValueKind::Uint,
            required: false,
            summary: "ticks from C2BOOT to the READY event (default 32000)",
        },
        PropertySpec {
            name: "poll",
            kind: ValueKind::Uint,
            required: false,
            summary: "also poll the IPCC every this many ticks (default 0: interrupts only)",
        },
        PropertySpec {
            name: "stack-version",
            kind: ValueKind::Uint,
            required: false,
            summary: "the wireless stack version word in the device information table",
        },
        PropertySpec {
            name: "fus-version",
            kind: ValueKind::Uint,
            required: false,
            summary: "the FUS version word in the device information table",
        },
        PropertySpec {
            name: "stack-type",
            kind: ValueKind::Uint,
            required: false,
            summary: "InfoStack's low byte (default 0x01, INFO_STACK_TYPE_BLE_STANDARD, AN5289 §4.10)",
        },
    ],
    construct: |props| Ok(Box::new(WbCpu2::new(props)?)),
};

/// Add [`CLASS`] to a registry.
///
/// # Errors
///
/// If something already claimed the name.
pub fn register(registry: &mut crate::core::Registry) -> Result<()> {
    registry.add(&CLASS)
}

/// Bind [`CLASS`] into the machine graph.
///
/// # Errors
///
/// If the class is already bound.
pub fn bind(bindings: &mut crate::machine::Bindings) -> Result<()> {
    bindings.bind(CLASS_NAME, |props| Ok(Arc::new(WbCpu2::new(props)?)))
}

/// What the validator should know about `st.wb-cpu2`.
#[must_use]
pub fn schema() -> ClassSchema {
    ClassSchema::new(CLASS_NAME)
        .prop(PropSchema::new("ipcc", ValueKind::Addr))
        .prop(PropSchema::new("ref-table", ValueKind::Addr))
        .prop(PropSchema::new("seed", ValueKind::Uint))
        .prop(PropSchema::new("bd-addr", ValueKind::Uint))
        .prop(PropSchema::new("latency", ValueKind::Uint))
        .prop(PropSchema::new("boot-latency", ValueKind::Uint))
        .prop(PropSchema::new("poll", ValueKind::Uint))
        .prop(PropSchema::new("stack-version", ValueKind::Uint))
        .prop(PropSchema::new("fus-version", ValueKind::Uint))
        .prop(PropSchema::new("stack-type", ValueKind::Uint))
        .port("boot", PortDir::In)
        .port("ipcc-rx", PortDir::In)
        .port("ipcc-tx", PortDir::In)
}

#[cfg(test)]
mod tests;
