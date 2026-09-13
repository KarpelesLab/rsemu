//! The RF reader door: an ISO/IEC 15693 reader, through the record/replay seam
//! and nowhere else.
//!
//! # Why this is a door and not an API
//!
//! Everything else an ST25DV does is inside the machine. A reader is not: it is
//! a phone somebody waved at the board, at a moment nothing in the simulation
//! chose. `CLAUDE.md` is unambiguous about what that means — "any
//! non-deterministic input crossing into the machine goes through the
//! record/replay seam, or it is a determinism bug" — so the reader is modelled
//! exactly the way a joypad and a keypad are, and for exactly the same reason.
//!
//! ```text
//!   host                     seam (core::record)              device
//!   ────                     ───────────────────              ──────
//!   rf::Command::encode ─► Recorder::post
//!                                │  (a round boundary at t)
//!                                ▼
//!         (t, "st25dv-rf:reader", [0xAA, 0x03, …]) ─► Feed ─► the tag
//!                                │                              │
//!                           the recording              Reader::poll ◄─ response
//! ```
//!
//! **Commands in, responses out, and only the commands are recorded.** A
//! response is derived from machine state the same way a UART's transmitted
//! byte is: replaying the same commands into the same machine reproduces it
//! bit for bit, so logging it would be logging the machine's own output as if
//! it were input. [`Reader::poll`] hands it to whoever is playing the reader.
//!
//! # How it is sealed
//!
//! [`KIND`] is a [`HostKind::door`] carrying [`sink`], so:
//!
//! * A board built against a table that was **already sealed** onto a recorder
//!   with no `st25dv-rf:<name>` channel is refused at
//!   [`HostObjects::open`](crate::core::hosts::HostObjects::open) time, naming
//!   the channel — the tag never gets constructed.
//! * A board sealed **during realize**, which is what
//!   [`machine::realize`](mod@crate::machine::realize) does, has its reader
//!   wired to the recorder automatically, because the kind knows how to build
//!   its own sink. A caller who forgot to declare the channel gets a recorded
//!   reader rather than a refusal.
//!
//! Either way there is no third path: a reader that is not on a channel cannot
//! reach a recorded machine. That is the guarantee `HostObjects::seal` exists
//! for, and this module does nothing clever to get out of it.
//!
//! # What a recorded session is
//!
//! One [`InputEvent`](crate::core::record::InputEvent) payload is a sequence of
//! **RF commands, in the tag's own command codes** (DS10925 Table 101): `AAh`
//! is Write Message, `ACh` is Read Message, `23h` is Read Multiple Blocks. The
//! recording is therefore readable as what it is — a reader session — rather
//! than as an encoding somebody invented. [`Command`] builds one and
//! [`Command::encode`] writes it; several commands may travel in one payload,
//! and they are applied in order, because a host that batches what happened
//! between two round boundaries posts them together.
//!
//! The one op that is not an ISO command is [`Command::Field`], `00h` — a
//! value the ISO command space leaves RFU. It is not a command because a field
//! is not something a reader *sends*: it is the carrier appearing, and the tag
//! reports it on `FIELD_RISING`/`FIELD_FALLING` and a `GPO` pulse. **Every
//! other command needs the field first**; posted without one, nothing happens
//! and no response comes back, which is what a tag out of range does.
//!
//! # Replay
//!
//! `tests/st25dv_replay.rs` is the contract, in the three claims
//! `tests/keypad_replay.rs` makes: a reader session changes where the run ends
//! up, a recording of one replays to the same state hash with the same
//! responses, and a board whose reader has no channel refuses to build.

use alloc::collections::VecDeque;
use alloc::string::String;
use alloc::sync::Arc;
use alloc::vec::Vec;
use core::fmt;

use crate::core::error::Result;
use crate::core::hosts::{HostKind, HostObjects};
use crate::core::props::Props;
use crate::core::record::{Channel, FnSink, InputSink};
use crate::core::sync::{LockRank, Mutex};
use crate::core::wire::Drive;

use super::{Shared, State, eh_bit, gpo_bit, it_bit, mb_bit, rf_bit, sys};

/// The reader door a tag answers when its machine file does not say.
pub const DEFAULT_READER: &str = "reader";

/// The kind a reader door is filed under in a build's [`HostObjects`].
pub const KIND: HostKind = HostKind::door("st25dv-rf", make_sink);

/// The reader door `name` refers to in `hosts`, creating it on first mention.
///
/// The **host** side of the rendezvous: called before anybody taps anything, or
/// after the build to pick up what the tag opened.
///
/// # Errors
///
/// [`crate::Error::Config`] if another kind of host object already holds that
/// name, or if `hosts` is sealed onto a recorder that has no channel for it.
pub fn open(hosts: &HostObjects, name: &str) -> Result<Arc<Reader>> {
    hosts.open(KIND, name, Reader::new)
}

/// The reader door `name` refers to in the build these properties belong to.
///
/// The **device** side, called from `new(props)`: acquiring a host object is
/// allocation, not an outward action ([`core::hosts`](crate::core::hosts)
/// argues the case). A `Props` that belongs to no build gets a private reader,
/// so a tag a unit test built directly still works and simply meets nobody.
///
/// # Errors
///
/// As [`open`].
pub fn attach(props: &Props, name: &str) -> Result<Arc<Reader>> {
    props.host(KIND, name, Reader::new)
}

/// The reader door called `name`, if it has been opened.
///
/// # Errors
///
/// As [`open`].
pub fn get(hosts: &HostObjects, name: &str) -> Result<Option<Arc<Reader>>> {
    hosts.get(KIND, name)
}

/// Forget `name`, reporting whether there was one.
pub fn close(hosts: &HostObjects, name: &str) -> bool {
    hosts.close(KIND, name)
}

/// Every open name, in order.
#[must_use]
pub fn names(hosts: &HostObjects) -> Vec<String> {
    hosts.names(KIND)
}

/// The record/replay channel the reader called `name` taps through.
///
/// `st25dv-rf:reader`, which is the same `(kind, name)` pair the host-object
/// table files the reader under — so a board whose reader has no channel is
/// refused by [`HostObjects::seal`] naming this string.
#[must_use]
pub fn channel(name: &str) -> Channel {
    Channel::new(KIND, name)
}

/// The [`InputSink`] that applies a recorded payload to `reader`.
///
/// No rewind hook: a reader keeps no queue of undelivered input. Its *outbox*
/// holds responses, which are machine output rather than host input, and a
/// rewind replays the commands that produce them.
#[must_use]
pub fn sink(reader: &Arc<Reader>) -> Arc<dyn InputSink> {
    let reader = Arc::clone(reader);
    Arc::new(FnSink::new("st25dv-rf", move |payload: &[u8]| {
        reader.deliver(payload);
    }))
}

/// [`sink`], reached through the erased handle the host-object table holds.
///
/// What [`KIND`] carries so that [`HostObjects::seal`] can wire this reader to
/// a recorder without the caller having to name it. `None` means something
/// that is not a [`Reader`] is filed under `st25dv-rf`.
fn make_sink(object: &Arc<dyn core::any::Any + Send + Sync>) -> Option<Arc<dyn InputSink>> {
    Some(sink(&Arc::clone(object).downcast::<Reader>().ok()?))
}

// ---------------------------------------------------------------------------
// The command vocabulary
// ---------------------------------------------------------------------------

/// RF command codes (DS10925 Table 101), plus `00h` for the field.
pub mod code {
    /// Not an ISO command: the carrier appearing or going away.
    pub const FIELD: u8 = 0x00;
    /// Inventory: answer with DSFID and the UID.
    pub const INVENTORY: u8 = 0x01;
    /// Read Multiple Blocks.
    pub const READ_BLOCKS: u8 = 0x23;
    /// Write Multiple Blocks.
    pub const WRITE_BLOCKS: u8 = 0x24;
    /// Read Configuration.
    pub const READ_CONFIG: u8 = 0xa0;
    /// Write Configuration.
    pub const WRITE_CONFIG: u8 = 0xa1;
    /// Manage GPO.
    pub const MANAGE_GPO: u8 = 0xa9;
    /// Write Message.
    pub const WRITE_MESSAGE: u8 = 0xaa;
    /// Read Message Length.
    pub const READ_MESSAGE_LENGTH: u8 = 0xab;
    /// Read Message.
    pub const READ_MESSAGE: u8 = 0xac;
    /// Read Dynamic Configuration.
    pub const READ_DYN_CONFIG: u8 = 0xad;
    /// Write Dynamic Configuration.
    pub const WRITE_DYN_CONFIG: u8 = 0xae;
    /// Write Password.
    pub const WRITE_PASSWORD: u8 = 0xb1;
    /// Present Password.
    pub const PRESENT_PASSWORD: u8 = 0xb3;
}

/// ISO/IEC 15693 error codes, as DS10925 §7.6 lists them.
pub mod error {
    /// The command is not recognized.
    pub const NOT_RECOGNIZED: u8 = 0x02;
    /// The command option is not supported.
    pub const OPTION: u8 = 0x03;
    /// Error with no information given, and ST's catch-all.
    pub const NO_INFORMATION: u8 = 0x0f;
    /// The specified block is not available.
    pub const NOT_AVAILABLE: u8 = 0x10;
    /// The specified block is locked or protected and cannot be changed.
    pub const LOCKED: u8 = 0x12;
    /// The specified block was not successfully programmed.
    pub const NOT_PROGRAMMED: u8 = 0x13;
    /// The specified block is read-protected.
    pub const READ_PROTECTED: u8 = 0x15;
}

/// The response flag bit that says an error code follows (ISO/IEC 15693-3).
pub const ERROR_FLAG: u8 = 0x01;

/// The most blocks one Write Multiple Blocks may carry, per the feature list
/// ("Single and multiple blocks write (up to 4)").
pub const MAX_WRITE_BLOCKS: usize = 4;

/// One thing a reader does to the tag.
///
/// A builder for the byte string a payload carries, so a host never writes a
/// command code by hand. The wire form is that command code followed by its
/// parameters, exactly as [`code`] names them.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum Command<'a> {
    /// The carrier appeared (`true`) or went away.
    ///
    /// Not an ISO command — see the module documentation. Every other command
    /// needs a field, so a session starts with `Field(true)`.
    Field(bool),
    /// Inventory. The response is the DSFID and the eight UID bytes, LSB first
    /// as they travel.
    Inventory,
    /// Read `count` four-byte blocks from `block`.
    ReadBlocks {
        /// The first block.
        block: u16,
        /// How many, 1 to 256.
        count: u16,
    },
    /// Write four-byte blocks from `block`. `data` must be a whole number of
    /// blocks, at most [`MAX_WRITE_BLOCKS`] of them.
    WriteBlocks {
        /// The first block.
        block: u16,
        /// The bytes, four per block.
        data: &'a [u8],
    },
    /// Read one system configuration register by its RF pointer (Table 11).
    ReadConfiguration {
        /// The pointer, which is *not* the I²C address for every register:
        /// `I2CSS` and `LOCK_CCFILE` have no RF pointer at all.
        pointer: u8,
    },
    /// Write one system configuration register.
    WriteConfiguration {
        /// The pointer.
        pointer: u8,
        /// The value.
        value: u8,
    },
    /// Drive or pulse `GPO` (Table 194): bit 7 set asks for a pulse, otherwise
    /// bit 0 clear sets the pin and bit 0 set releases it.
    ManageGpo {
        /// `GPOVAL`.
        value: u8,
    },
    /// Put a message in the mailbox, 1 to 256 bytes.
    WriteMessage(&'a [u8]),
    /// Ask how long the message in the mailbox is.
    ReadMessageLength,
    /// Read `count` bytes of the mailbox message from `offset`.
    ReadMessage {
        /// Where in the message to start.
        offset: u8,
        /// How many bytes, 1 to 256.
        count: u16,
    },
    /// Read one dynamic register by its RF pointer: `00h`, `02h` or `0Dh`
    /// (Table 12).
    ReadDynamicConfiguration {
        /// The pointer.
        pointer: u8,
    },
    /// Write one dynamic register.
    WriteDynamicConfiguration {
        /// The pointer.
        pointer: u8,
        /// The value.
        value: u8,
    },
    /// Present an RF password: `0` is the configuration password, `1` to `3`
    /// are the user passwords (§7.6.36).
    PresentPassword {
        /// Which password.
        number: u8,
        /// Its eight bytes, MSB first.
        password: [u8; 8],
    },
    /// Replace an RF password, which needs its session already open.
    WritePassword {
        /// Which password.
        number: u8,
        /// Its eight bytes, MSB first.
        password: [u8; 8],
    },
}

impl Command<'_> {
    /// Append this command's wire form to `out`.
    ///
    /// A `count` outside 1..=256 is clamped and a `WriteBlocks` whose `data` is
    /// not a whole number of blocks is truncated to one: a malformed command is
    /// a caller bug, and the tag's own refusals are about *protocol* errors,
    /// which is a different thing worth keeping legible.
    pub fn encode_to(&self, out: &mut Vec<u8>) {
        #[allow(clippy::cast_possible_truncation)]
        fn len_field(count: u16) -> u8 {
            (count.clamp(1, 256) - 1) as u8
        }
        match *self {
            Command::Field(on) => out.extend_from_slice(&[code::FIELD, u8::from(on)]),
            Command::Inventory => out.push(code::INVENTORY),
            Command::ReadBlocks { block, count } => {
                out.push(code::READ_BLOCKS);
                out.extend_from_slice(&block.to_le_bytes());
                out.push(len_field(count));
            }
            Command::WriteBlocks { block, data } => {
                let blocks = (data.len() / 4).clamp(1, MAX_WRITE_BLOCKS);
                out.push(code::WRITE_BLOCKS);
                out.extend_from_slice(&block.to_le_bytes());
                #[allow(clippy::cast_possible_truncation)]
                out.push((blocks - 1) as u8);
                out.extend_from_slice(&data[..(blocks * 4).min(data.len())]);
            }
            Command::ReadConfiguration { pointer } => {
                out.extend_from_slice(&[code::READ_CONFIG, pointer]);
            }
            Command::WriteConfiguration { pointer, value } => {
                out.extend_from_slice(&[code::WRITE_CONFIG, pointer, value]);
            }
            Command::ManageGpo { value } => out.extend_from_slice(&[code::MANAGE_GPO, value]),
            Command::WriteMessage(data) => {
                let len = data.len().clamp(1, 256);
                out.push(code::WRITE_MESSAGE);
                #[allow(clippy::cast_possible_truncation)]
                out.push((len - 1) as u8);
                out.extend_from_slice(&data[..len.min(data.len())]);
            }
            Command::ReadMessageLength => out.push(code::READ_MESSAGE_LENGTH),
            Command::ReadMessage { offset, count } => {
                out.extend_from_slice(&[code::READ_MESSAGE, offset, len_field(count)]);
            }
            Command::ReadDynamicConfiguration { pointer } => {
                out.extend_from_slice(&[code::READ_DYN_CONFIG, pointer]);
            }
            Command::WriteDynamicConfiguration { pointer, value } => {
                out.extend_from_slice(&[code::WRITE_DYN_CONFIG, pointer, value]);
            }
            Command::PresentPassword { number, password } => {
                out.push(code::PRESENT_PASSWORD);
                out.push(number);
                out.extend_from_slice(&password);
            }
            Command::WritePassword { number, password } => {
                out.push(code::WRITE_PASSWORD);
                out.push(number);
                out.extend_from_slice(&password);
            }
        }
    }

    /// This command's wire form, on its own.
    #[must_use]
    pub fn encode(&self) -> Vec<u8> {
        let mut out = Vec::new();
        self.encode_to(&mut out);
        out
    }

    /// Several commands in one payload, which is what a host that batches a
    /// round's worth of reader activity posts.
    #[must_use]
    pub fn encode_all(commands: &[Command<'_>]) -> Vec<u8> {
        let mut out = Vec::new();
        for command in commands {
            command.encode_to(&mut out);
        }
        out
    }
}

// ---------------------------------------------------------------------------
// The reader
// ---------------------------------------------------------------------------

/// What a reader holds, under one lock.
#[derive(Debug, Default)]
struct ReaderState {
    /// The tag this door reaches, once a build has realized one.
    tag: Option<Arc<Shared>>,
    /// Responses the tag has produced and nobody has collected.
    outbox: VecDeque<Vec<u8>>,
    /// How many commands have been executed, for a host that wants to know its
    /// session went in.
    executed: u64,
}

/// One RF reader, at one end of the record/replay seam.
///
/// Shared by name, like a character port: whoever opens `"reader"` in a build's
/// [`HostObjects`] and whichever tag names it in its machine file hold the same
/// object. A reader with no tag bound discards what it is given, which is what
/// a reader waved at empty air does.
pub struct Reader {
    /// [`LockRank::LEAF`]: the tag's own state lock is taken *around* this one,
    /// never inside it — [`Reader::deliver`] clones the tag out, drops this
    /// lock, executes, and takes it again to post the response.
    state: Mutex<ReaderState>,
}

impl fmt::Debug for Reader {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let mut s = f.debug_struct("Reader");
        match self.state.try_lock() {
            Some(state) => s
                .field("bound", &state.tag.is_some())
                .field("pending", &state.outbox.len())
                .field("executed", &state.executed),
            None => s.field("state", &"<in use>"),
        };
        s.finish()
    }
}

impl Default for Reader {
    fn default() -> Reader {
        Reader::new()
    }
}

impl Reader {
    /// A reader with no tag and nothing queued.
    #[must_use]
    pub fn new() -> Reader {
        Reader {
            state: Mutex::with_rank(LockRank::LEAF, ReaderState::default()),
        }
    }

    /// Point this door at a tag. Called from
    /// [`Device::realize`](crate::core::device::Device::realize).
    pub(super) fn bind(&self, tag: &Arc<Shared>) {
        self.state.lock().tag = Some(Arc::clone(tag));
    }

    /// Whether a tag has been realized behind this door.
    #[must_use]
    pub fn is_bound(&self) -> bool {
        self.state.lock().tag.is_some()
    }

    /// The oldest response the tag has not handed back yet.
    ///
    /// A response is `[flags, …]` in ISO/IEC 15693's shape: flags `00h` and the
    /// data, or flags [`ERROR_FLAG`] and one error code. A command the tag was
    /// silent about — no field, `RF_SLEEP` — produces nothing at all, which is
    /// itself the answer.
    #[must_use]
    pub fn poll(&self) -> Option<Vec<u8>> {
        self.state.lock().outbox.pop_front()
    }

    /// Every response, oldest first, leaving the outbox empty.
    #[must_use]
    pub fn drain(&self) -> Vec<Vec<u8>> {
        let mut state = self.state.lock();
        state.outbox.drain(..).collect()
    }

    /// How many responses are waiting.
    #[must_use]
    pub fn pending(&self) -> usize {
        self.state.lock().outbox.len()
    }

    /// How many commands this door has executed against a tag.
    #[must_use]
    pub fn executed(&self) -> u64 {
        self.state.lock().executed
    }

    /// Throw away every queued response.
    pub fn clear(&self) {
        self.state.lock().outbox.clear();
    }

    /// Apply one recorded payload: a sequence of commands, in order.
    ///
    /// **Not public.** A host reaches this through
    /// [`Recorder::post`](crate::core::record::Recorder::post) on this reader's
    /// [`channel`], because that is the only path that gets the instant
    /// recorded — which is the whole point of the door. `sink` is the other
    /// end of that path and is the only caller.
    fn deliver(&self, payload: &[u8]) {
        // The tag is cloned out and the reader's lock released before anything
        // is executed: the tag takes its own lock and calls outward onto the
        // GPO net, and a leaf lock may not be held across either.
        let Some(tag) = self.state.lock().tag.clone() else {
            return;
        };
        let mut at = 0usize;
        let mut done = 0u64;
        let mut replies: Vec<Vec<u8>> = Vec::new();
        while at < payload.len() {
            let Some((used, reply)) = execute(&tag, &payload[at..]) else {
                // A truncated or unparseable tail: stop rather than guess. A
                // recording is replayed byte for byte, so this is the same
                // decision every time.
                break;
            };
            at += used;
            done += 1;
            if let Some(reply) = reply {
                replies.push(reply);
            }
        }
        let mut state = self.state.lock();
        state.executed += done;
        state.outbox.extend(replies);
    }
}

// ---------------------------------------------------------------------------
// Execution
// ---------------------------------------------------------------------------

/// A successful response: flags `00h` and `data`.
fn ok(data: &[u8]) -> Option<Vec<u8>> {
    let mut out = Vec::with_capacity(data.len() + 1);
    out.push(0x00);
    out.extend_from_slice(data);
    Some(out)
}

/// A failed response: the error flag and one code.
fn err(code: u8) -> Option<Vec<u8>> {
    Some(alloc::vec![ERROR_FLAG, code])
}

/// How many bytes one command occupies, and what the tag answered.
///
/// `None` means the command could not even be parsed, which ends the payload.
fn execute(tag: &Arc<Shared>, bytes: &[u8]) -> Option<(usize, Option<Vec<u8>>)> {
    let op = *bytes.first()?;
    let args = &bytes[1..];
    // How long this command is, decided before anything is locked so that a
    // truncated tail is a parse failure rather than a half-applied command.
    let used = match op {
        code::FIELD => 2,
        code::INVENTORY | code::READ_MESSAGE_LENGTH => 1,
        code::READ_BLOCKS => 4,
        code::WRITE_BLOCKS => 4 + 4 * (usize::from(*args.get(2)?) + 1),
        code::READ_CONFIG | code::READ_DYN_CONFIG | code::MANAGE_GPO => 2,
        code::WRITE_CONFIG | code::WRITE_DYN_CONFIG | code::READ_MESSAGE => 3,
        code::WRITE_MESSAGE => 2 + usize::from(*args.first()?) + 1,
        code::PRESENT_PASSWORD | code::WRITE_PASSWORD => 10,
        // An unknown code has no length, so nothing after it can be parsed.
        // The tag answers "command not recognized" and the payload ends.
        _ => return Some((bytes.len(), err(error::NOT_RECOGNIZED))),
    };
    if bytes.len() < used {
        return None;
    }
    let args = &bytes[1..used];

    let (reply, drive) = {
        let mut state = tag.state.lock();
        let reply = run(tag, &mut state, op, args);
        tag.publish(&state);
        (reply, tag.gpo_level(&state))
    };
    // Outward with the lock released, as the re-entrancy contract asks.
    tag.refresh_gpo(drive);
    Some((used, reply))
}

/// One command, with the tag's state locked.
#[allow(clippy::too_many_lines)]
fn run(tag: &Arc<Shared>, state: &mut State, op: u8, args: &[u8]) -> Option<Vec<u8>> {
    if op == code::FIELD {
        field_change(tag, state, args[0] != 0);
        return None;
    }
    // §5.4.2: RF_SLEEP means "ST25DV remains silent", RF_DISABLE means commands
    // are interpreted and answered `0Fh`. Both live in the *dynamic* register,
    // which is what the part acts on.
    if !state.field || state.rf_mngt_dyn & rf_bit::RF_SLEEP != 0 {
        return None;
    }
    if state.rf_mngt_dyn & rf_bit::RF_DISABLE != 0 {
        // "The Inventory command is not answered."
        if op == code::INVENTORY {
            return None;
        }
        return err(error::NO_INFORMATION);
    }
    // Every executed command is an RF access (Table 32). The pin is not driven
    // from it — see `Shared::gpo_level` for why — so the enable mask is zero.
    tag.raise(state, it_bit::RF_ACTIVITY, 0);

    match op {
        code::INVENTORY => {
            let mut out = Vec::with_capacity(9);
            out.push(state.ident[(sys::DSFID - sys::LOCK_DSFID) as usize]);
            out.extend_from_slice(&tag.uid);
            ok(&out)
        }
        code::READ_BLOCKS => read_blocks(tag, state, args),
        code::WRITE_BLOCKS => write_blocks(tag, state, args),
        code::READ_CONFIG => match rf_config_address(args[0]) {
            Some(reg) => ok(&[state.sys[reg as usize]]),
            None => err(error::NOT_AVAILABLE),
        },
        code::WRITE_CONFIG => write_config(tag, state, args[0], args[1]),
        code::MANAGE_GPO => manage_gpo(tag, state, args[0]),
        code::WRITE_MESSAGE => write_message(tag, state, &args[1..]),
        code::READ_MESSAGE_LENGTH => {
            if state.ftm() {
                ok(&[state.mb_len])
            } else {
                err(error::NO_INFORMATION)
            }
        }
        code::READ_MESSAGE => read_message(tag, state, args[0], usize::from(args[1]) + 1),
        code::READ_DYN_CONFIG => match args[0] {
            0x00 => ok(&[state.gpo_dyn]),
            0x02 => ok(&[tag.eh_ctrl(state)]),
            0x0d => ok(&[state.mb_ctrl]),
            // Table 12: everything else is "No access" from RF.
            _ => err(error::NOT_AVAILABLE),
        },
        code::WRITE_DYN_CONFIG => write_dyn_config(state, args[0], args[1]),
        code::PRESENT_PASSWORD => present_password(state, args[0], &args[1..]),
        code::WRITE_PASSWORD => write_password(tag, state, args[0], &args[1..]),
        _ => err(error::NOT_RECOGNIZED),
    }
}

/// The carrier appeared or went away (§5.2.2).
fn field_change(tag: &Arc<Shared>, state: &mut State, on: bool) {
    if state.field == on {
        return;
    }
    state.field = on;
    if !on {
        // The RF security sessions live on the field: no carrier, no session.
        state.rf_cfg_session = false;
        state.rf_user_session = 0;
    }
    if state.rf_mngt_dyn & rf_bit::RF_SLEEP != 0 {
        // Table 22: in RF sleep, "GPO remains High-Z … IT_STS_Dyn register is
        // not updated". The flag in EH_CTRL_Dyn still follows the field,
        // because that is a power-source fact rather than an interrupt.
        return;
    }
    if !on && !state.vcc_on() {
        // §5.2.2: "In case of RF field disappear, the pulse is emitted only if
        // VCC power supply is present" — and with neither source there is
        // nothing left running to report anything.
        return;
    }
    let status = if on {
        it_bit::FIELD_RISING
    } else {
        it_bit::FIELD_FALLING
    };
    tag.raise(state, status, gpo_bit::FIELD_CHANGE_EN);
}

/// Read Multiple Blocks (§7.6.5).
fn read_blocks(tag: &Arc<Shared>, state: &State, args: &[u8]) -> Option<Vec<u8>> {
    let first = u64::from(u16::from_le_bytes([args[0], args[1]]));
    let count = u64::from(args[2]) + 1;
    if first + count > tag.density.blocks() {
        return err(error::NOT_AVAILABLE);
    }
    let mut out = Vec::with_capacity((count * super::BLOCK_SIZE) as usize);
    for block in first..first + count {
        let at = block * super::BLOCK_SIZE;
        #[allow(clippy::cast_possible_truncation)]
        let area = state.area_of(at);
        if !state.rf_can_read(area) {
            return err(error::READ_PROTECTED);
        }
        out.extend_from_slice(&state.user[at as usize..(at + super::BLOCK_SIZE) as usize]);
    }
    ok(&out)
}

/// Write Multiple Blocks (§7.6.6).
fn write_blocks(tag: &Arc<Shared>, state: &mut State, args: &[u8]) -> Option<Vec<u8>> {
    let first = u64::from(u16::from_le_bytes([args[0], args[1]]));
    let count = usize::from(args[2]) + 1;
    let data = &args[3..];
    if count > MAX_WRITE_BLOCKS {
        // The feature list caps a multiple-block write at four.
        return err(error::OPTION);
    }
    if state.ftm() {
        // §5.1.2's caution: EEPROM writes transit the mailbox buffer, so fast
        // transfer mode must be off — "get an answer 0Fh for RF".
        return err(error::NO_INFORMATION);
    }
    if first + count as u64 > tag.density.blocks() {
        return err(error::NOT_AVAILABLE);
    }
    for index in 0..count {
        let block = first + index as u64;
        let at = block * super::BLOCK_SIZE;
        #[allow(clippy::cast_possible_truncation)]
        let area = state.area_of(at);
        if !state.rf_can_write(area) || locked_ccfile(state, block) {
            return err(error::LOCKED);
        }
    }
    for index in 0..count {
        let at = ((first + index as u64) * super::BLOCK_SIZE) as usize;
        let from = index * super::BLOCK_SIZE as usize;
        state.user[at..at + super::BLOCK_SIZE as usize]
            .copy_from_slice(&data[from..from + super::BLOCK_SIZE as usize]);
    }
    // One block is exactly one internal EEPROM page, so the write cycle is tW
    // per block (§6.4.2's page arithmetic, from the RF side).
    start_write(tag, state, count as u64);
    tag.raise(state, it_bit::RF_WRITE, gpo_bit::RF_WRITE_EN);
    ok(&[])
}

/// Whether `LOCK_CCFILE` protects `block` from an RF write (Table 54).
fn locked_ccfile(state: &State, block: u64) -> bool {
    let lock = state.sys[sys::LOCK_CCFILE as usize];
    (block == 0 && lock & 0b01 != 0) || (block == 1 && lock & 0b10 != 0)
}

/// Begin the internal write cycle for `pages` EEPROM pages.
fn start_write(tag: &Arc<Shared>, state: &mut State, pages: u64) {
    let now = tag.now(state);
    state.busy_until = now.saturating_add(tag.write_ticks.saturating_mul(pages.max(1)));
    state.busy = true;
}

/// The system configuration register an RF pointer names, if any.
///
/// Table 11's RF column: `I2CSS` (`0Bh`) and `LOCK_CCFILE` (`0Ch`) say "No
/// access", so the I²C host is the security master for both — exactly the
/// asymmetry the part exists to provide.
fn rf_config_address(pointer: u8) -> Option<u16> {
    match u16::from(pointer) {
        reg @ (sys::GPO..=sys::RFA4SS | sys::MB_MODE..=sys::LOCK_CFG) => Some(reg),
        _ => None,
    }
}

/// Write Configuration (§7.6.32).
fn write_config(tag: &Arc<Shared>, state: &mut State, pointer: u8, value: u8) -> Option<Vec<u8>> {
    let Some(reg) = rf_config_address(pointer) else {
        return err(error::NOT_AVAILABLE);
    };
    // §4.3: "Update is only possible when the access right was granted by
    // presenting the RF configuration password, and if the system configuration
    // was not previously locked by the I2C host (LOCK_CFG = 1)".
    if !state.rf_cfg_session || state.sys[sys::LOCK_CFG as usize] & 1 != 0 {
        return err(error::LOCKED);
    }
    if (reg == sys::ENDA1 || reg == sys::ENDA2 || reg == sys::ENDA3)
        && !tag.enda_ok(state, reg, value)
    {
        // §4.2: "an error 0Fh is returned in RF".
        return err(error::NO_INFORMATION);
    }
    state.sys[reg as usize] = value;
    tag.sys_written(state, reg);
    start_write(tag, state, 1);
    tag.raise(state, it_bit::RF_WRITE, gpo_bit::RF_WRITE_EN);
    ok(&[])
}

/// Manage GPO (§7.6.30, Table 194).
fn manage_gpo(tag: &Arc<Shared>, state: &mut State, value: u8) -> Option<Vec<u8>> {
    let user_enabled = state.gpo_dyn & gpo_bit::RF_USER_EN != 0;
    let int_enabled = state.gpo_dyn & gpo_bit::RF_INTERRUPT_EN != 0;
    if !user_enabled && !int_enabled {
        // "If neither RF_USER nor RF_INTERRUPT was enabled, the command is not
        // executed and ST25DVxxx responds an Error code 0F."
        return err(error::NO_INFORMATION);
    }
    if value & 0x80 != 0 {
        if !int_enabled {
            return err(error::NOT_PROGRAMMED);
        }
        tag.raise(state, it_bit::RF_INTERRUPT, gpo_bit::RF_INTERRUPT_EN);
        return ok(&[]);
    }
    if !user_enabled {
        return err(error::NOT_PROGRAMMED);
    }
    // Bit 0 clear is "set" and bit 0 set is "reset" — the polarity of a pin
    // that is asserted low on the open-drain parts.
    state.rf_user = value & 1 == 0;
    if state.rf_user {
        state.it_sts |= it_bit::RF_USER;
    } else {
        state.it_sts &= !it_bit::RF_USER;
    }
    ok(&[])
}

/// Write Message (§7.6.25).
fn write_message(tag: &Arc<Shared>, state: &mut State, data: &[u8]) -> Option<Vec<u8>> {
    if !state.ftm() {
        return err(error::NO_INFORMATION);
    }
    if state.mb_busy() {
        // §5.1.2: "Adding a message is only possible when … mailbox is free".
        return err(error::NO_INFORMATION);
    }
    state.mailbox[..data.len()].copy_from_slice(data);
    #[allow(clippy::cast_possible_truncation)]
    {
        state.mb_len = (data.len() - 1) as u8;
    }
    state.mb_ctrl |= mb_bit::RF_PUT_MSG | mb_bit::RF_CURRENT_MSG;
    state.mb_ctrl &= !mb_bit::HOST_CURRENT_MSG;
    tag.arm_watchdog(state);
    tag.raise(state, it_bit::RF_PUT_MSG, gpo_bit::RF_PUT_MSG_EN);
    ok(&[])
}

/// Read Message (§7.6.27).
fn read_message(tag: &Arc<Shared>, state: &mut State, offset: u8, count: usize) -> Option<Vec<u8>> {
    if !state.ftm() {
        return err(error::NO_INFORMATION);
    }
    let offset = usize::from(offset);
    let end = offset + count;
    if end > state.mb_message_len() {
        // "return an error 0Fh if trying to read after the last byte of the
        // message".
        return err(error::NO_INFORMATION);
    }
    let out = state.mailbox[offset..end].to_vec();
    if end == state.mb_message_len() {
        // §5.1.2: "HOST_PUT_MSG is cleared following a valid reading of the
        // last message byte, and mailbox is considered free (but message is not
        // cleared)". A read never clears the *reader's* own RF_PUT_MSG.
        if state.mb_ctrl & mb_bit::HOST_PUT_MSG != 0 {
            state.mb_ctrl &= !mb_bit::HOST_PUT_MSG;
            state.wdg_armed = false;
        }
        tag.raise(state, it_bit::RF_GET_MSG, gpo_bit::RF_GET_MSG_EN);
    }
    ok(&out)
}

/// Write Dynamic Configuration (§7.6.34).
fn write_dyn_config(state: &mut State, pointer: u8, value: u8) -> Option<Vec<u8>> {
    match pointer {
        // Table 29: `GPO_CTRL_Dyn` "is read only for RF user".
        0x00 => err(error::LOCKED),
        0x02 => {
            state.eh_en = value & eh_bit::EH_EN != 0;
            ok(&[])
        }
        0x0d => {
            let want = value & mb_bit::MB_EN != 0;
            if want && state.sys[sys::MB_MODE as usize] & 1 == 0 {
                return err(error::NOT_PROGRAMMED);
            }
            if want {
                state.mb_ctrl |= mb_bit::MB_EN;
            } else {
                state.mb_ctrl = 0;
                state.mb_len = 0;
                state.wdg_armed = false;
            }
            ok(&[])
        }
        _ => err(error::NOT_AVAILABLE),
    }
}

/// Present Password (§7.6.36).
fn present_password(state: &mut State, number: u8, password: &[u8]) -> Option<Vec<u8>> {
    if number > 3 {
        return err(error::NOT_AVAILABLE);
    }
    let index = number as usize;
    let matches = password == state.rf_pwd[index];
    if number == 0 {
        state.rf_cfg_session = matches;
    } else if matches {
        // "Close RF user security session: Present Password command, with a
        // different password number than the one currently open" — so a
        // successful presentation simply becomes the open session.
        state.rf_user_session = number;
    } else {
        state.rf_user_session = 0;
    }
    if matches {
        ok(&[])
    } else {
        err(error::NO_INFORMATION)
    }
}

/// Write Password (§7.6.37).
fn write_password(
    tag: &Arc<Shared>,
    state: &mut State,
    number: u8,
    password: &[u8],
) -> Option<Vec<u8>> {
    if number > 3 {
        return err(error::NOT_AVAILABLE);
    }
    let open = if number == 0 {
        state.rf_cfg_session
    } else {
        state.rf_user_session == number
    };
    if !open {
        // Table 11, footnote 9: "Write access only if corresponding RF security
        // session is open."
        return err(error::LOCKED);
    }
    state.rf_pwd[number as usize].copy_from_slice(password);
    start_write(tag, state, 2);
    ok(&[])
}

/// What the GPO pin would be doing — re-exported for a test that holds a
/// reader but not the device.
#[must_use]
pub fn gpo_of(reader: &Reader) -> Option<Drive> {
    let tag = reader.state.lock().tag.clone()?;
    let state = tag.state.lock();
    Some(tag.gpo_level(&state))
}
