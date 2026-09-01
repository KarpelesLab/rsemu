//! An Intel 8042 keyboard controller, and the keyboard on the far end of it.
//!
//! # Sources
//!
//! * *IBM Personal Computer AT Technical Reference* (1984), the keyboard and
//!   keyboard-controller sections: the status register, the controller command
//!   set, the input and output ports, and which line of the output port is the
//!   A20 gate and which is the system reset.
//! * The Intel *8042 Universal Peripheral Interface* data sheet, for the part
//!   itself — the two decoded addresses, and A2 as the bit that tells the
//!   firmware which of them the last write went to.
//! * The OSDev wiki's *"8042 PS/2 Controller"* and *"PS/2 Keyboard"* pages, for
//!   the same facts restated by people who have tested them, and for the
//!   set-2-to-set-1 translation table.
//!
//! **No emulator source was consulted** (`CLAUDE.md`, provenance).
//!
//! # Two ports, one byte each
//!
//! An AT decodes the controller at 0x60 and 0x64 and puts something else
//! entirely at 0x61, 0x62 and 0x63. So this device publishes **two one-byte
//! regions** rather than one four-byte block with holes in it:
//!
//! ```text
//!   region "data"  (also "")   port 0x60   the data port
//!   region "cmd"               port 0x64   status on read, command on write
//! ```
//!
//! The machine file maps each where the board puts it, which is the only place
//! those two addresses are written down.
//!
//! # What arrives on the character port is scan codes, not text
//!
//! The keyboard is fed through the character-device seam
//! ([`crate::host::chardev`]), and **every byte the host feeds is a raw AT
//! scan code in set 2** — `0x1C` is the `A` key going down and `0xF0 0x1C` is
//! it coming back up. It is not ASCII, and a byte that happens to be `b'A'`
//! means the `9` key rather than the letter.
//!
//! Turning a terminal's keystrokes — or a browser's `KeyboardEvent`s, or a
//! replay log — into scan codes belongs **above this device, in `host/`**. It
//! is a host concern: it depends on the keyboard layout the user is actually
//! typing on, and a device that guessed would be wrong for every non-US
//! layout and for every non-terminal front end. The chip's job is to carry
//! bytes and translate between the two documented encodings, not to invent
//! them.
//!
//! # Translation
//!
//! With bit 6 of the controller command byte set — which is how every PC comes
//! up, because the ROM BIOS's interrupt 9 handler only understands set 1 — the
//! controller rewrites the set-2 codes the keyboard sends into set 1 on their
//! way to the output buffer: a byte through [`TRANSLATE`], with the `0xF0`
//! break prefix folded into bit 7 of the code that follows it and the `0xE0`
//! extended prefix passed straight through.
//!
//! # A20, and why it matters more than anything else here
//!
//! Bit 1 of the controller's output port is the A20 gate. Firmware enables it
//! before it can address a byte above 1 MiB, and every protected-mode operating
//! system built for this machine does so through this chip. A controller that
//! accepted the write and did nothing would leave the guest addressing the
//! low megabyte forever, which does not look like a keyboard bug at all — so
//! the gate is a pin here, driven on every write of the output port.
//!
//! Bit 0 of the same port is the system reset line, **active low**, and command
//! `0xFE` pulses it. That is how a 286 leaves protected mode, and it is still
//! how `reboot` reaches the hardware on machines built decades later; it has to
//! actually drive the pin.

use alloc::boxed::Box;
use alloc::collections::VecDeque;
use alloc::string::{String, ToString};
use alloc::sync::Arc;
use core::fmt;

use crate::core::device::{Device, DeviceClass, PropertySpec, RealizeCtx, ResetKind};
use crate::core::error::{BusError, Error, Result};
use crate::core::props::{Props, ValueKind};
use crate::core::sched::{Budget, Consumed};
use crate::core::space::{AccessConstraints, MemAttrs, MemOps, MemResult, Region, RegionRef};
use crate::core::state::{ChunkReader, ChunkWriter, Sink, Source};
use crate::core::sync::{LockRank, Mutex};
use crate::core::value::{Endian, Width};
use crate::core::wire::{Level, WireSource};
use crate::host::chardev::{CharDevice, ports};
use crate::machine::realize::Instance;
use crate::machine::validate::ClassSchema;

/// The class name a machine description writes.
pub const CLASS_NAME: &str = "pc.kbc";

/// The snapshot chunk version. Bump with the encoding, never on its own.
const STATE_VERSION: u32 = 1;

/// How much address space each of the two register blocks answers.
///
/// One byte, twice. See the module docs for why this is not a single region.
pub const REGISTER_WINDOW_LEN: u64 = 1;

/// The character port a machine file gets if it names none.
const DEFAULT_PORT: &str = "keyboard";

/// How many bytes of controller RAM the 8042 has. Byte 0 is the command byte.
pub const RAM_LEN: usize = 32;

/// How many bytes the keyboard will hold before it stops accepting scan codes.
///
/// A real AT keyboard has a 16-byte transmit buffer and stops scanning when it
/// fills. Here it is also the back-pressure point: [`Kbc8042::pump`] stops
/// taking bytes off the character port when the buffer is full, so a host that
/// types faster than the guest reads waits rather than growing the heap.
pub const KEYBOARD_BUFFER: usize = 16;

// -- the status register, read at the command port --------------------------

/// Output buffer full: there is a byte here for the CPU.
const ST_OBF: u8 = 0x01;
/// System flag, which is bit 2 of the controller command byte.
const ST_SYS: u8 = 0x04;
/// A2: the last write went to the command port rather than the data port.
const ST_A2: u8 = 0x08;
/// Keyboard inhibited.
const ST_INH: u8 = 0x10;
/// The byte in the output buffer came from the auxiliary device.
const ST_AUX_OBF: u8 = 0x20;

// Bit 1 (IBF), bit 6 (receive timeout) and bit 7 (parity error) are never set.
// IBF is the input buffer the controller's own firmware would be draining, and
// this model executes a command the instant it is written, so there is no
// instant at which a guest could observe it full. Timeout and parity report a
// cable fault; the cable here is a `VecDeque`.

// -- the controller command byte, RAM byte 0 --------------------------------

/// Raise IRQ1 when the output buffer fills from the keyboard.
const CB_KBD_INT: u8 = 0x01;
/// Raise IRQ12 when the output buffer fills from the auxiliary device.
const CB_AUX_INT: u8 = 0x02;
/// The system flag, reflected into status bit 2.
const CB_SYS: u8 = 0x04;
/// Disable the keyboard clock: nothing reaches the output buffer.
const CB_KBD_CLOCK_OFF: u8 = 0x10;
/// Disable the auxiliary clock.
const CB_AUX_CLOCK_OFF: u8 = 0x20;
/// Translate scan code set 2 into set 1 on the way to the output buffer.
const CB_TRANSLATE: u8 = 0x40;

// -- the output port, command 0xD0 / 0xD1 -----------------------------------

/// The system reset line, **active low**: written clear, the machine resets.
const OP_RESET: u8 = 0x01;
/// The A20 gate: set enables address line 20.
const OP_A20: u8 = 0x02;
/// Keyboard output-buffer-full, reported on a read of the port.
const OP_KBD_OBF: u8 = 0x10;
/// Auxiliary output-buffer-full, reported on a read of the port.
const OP_AUX_OBF: u8 = 0x20;

/// What the output port holds out of reset.
///
/// Reset inactive (the line is active low, so the bit is *set*) and A20 shut,
/// which is the state a real AT powers up in — the gate exists precisely so
/// that the megabyte wrap of an 8086 survives, and firmware opens it when it
/// wants the memory above.
const OUTPUT_PORT_RESET: u8 = OP_RESET;

/// What a read of the input port (command 0xC0) answers.
///
/// Bit 7 keyboard not locked, bit 6 the display switch (0: a colour adapter is
/// the primary display), bit 5 the manufacturing jumper **not** installed —
/// firmware that finds it installed goes into a burn-in loop and never boots —
/// and bit 4 the memory jumper. The low nibble is the keyboard and auxiliary
/// data lines, idle.
const INPUT_PORT: u8 = 0b1011_0000;

/// What a read of the test inputs (command 0xE0) answers: T0 is the keyboard
/// clock and T1 its data line, both idle high with nothing being clocked.
const TEST_INPUTS: u8 = 0b0000_0011;

/// The self-test reply. `0x55` and nothing else; firmware that reads anything
/// different reports a controller failure and stops.
const SELF_TEST_OK: u8 = 0x55;

// -- what the keyboard says -------------------------------------------------

/// Acknowledge. Every command the keyboard understands is answered with it.
const KB_ACK: u8 = 0xfa;
/// Resend — also what the keyboard answers a command it does not understand.
const KB_RESEND: u8 = 0xfe;
/// Basic assurance test passed, sent after a reset.
const KB_BAT_OK: u8 = 0xaa;
/// The first byte of the two-byte identify reply.
const KB_ID_HIGH: u8 = 0xab;
/// The second. With translation on a guest reads it back as `0x41`, because
/// the controller translates replies as well as scan codes — see [`TRANSLATE`].
const KB_ID_LOW: u8 = 0x83;

/// The set-2 break prefix: the code after it is a key coming up.
const SET2_BREAK: u8 = 0xf0;
/// The set-2 extended prefix, which both sets spell the same way.
const SET2_EXTEND: u8 = 0xe0;

/// The typematic byte a reset or a "set defaults" leaves behind: 10.9 characters
/// per second after a 500 ms delay.
const DEFAULT_TYPEMATIC: u8 = 0x2b;

/// The scan code set a keyboard uses out of reset.
const DEFAULT_SCAN_SET: u8 = 2;

/// The AT controller's scan-code translation table: set 2 in, set 1 out.
///
/// This is the table the IBM PC/AT and PS/2 technical references print for the
/// controller's translate mode, as reproduced on the OSDev wiki. It is a
/// property of the hardware — the same 256 numbers appear in the manual, in the
/// wiki and in the ROM listings — and it was taken from documentation, never
/// from an emulator (`CLAUDE.md`, provenance).
///
/// Entries from `0x87` up are the identity, which is not padding: the keyboard
/// sends its *replies* through the same path, and `0xFA`, `0xAA`, `0xFE` and
/// `0xAB` have to survive translation or no driver could talk to the keyboard
/// at all. The one reply byte that does not survive is `0x83`, the low half of
/// the identify response, which comes back as `0x41` — the documented quirk
/// that makes a translated keyboard identify as `AB 41`.
pub const TRANSLATE: [u8; 256] = build_translation();

/// Build [`TRANSLATE`]: the documented head, then the identity.
const fn build_translation() -> [u8; 256] {
    /// Set-2 codes `0x00`-`0x86`, the range the table actually covers.
    const HEAD: [u8; 0x87] = [
        // 0x00
        0xff, 0x43, 0x41, 0x3f, 0x3d, 0x3b, 0x3c, 0x58, 0x64, 0x44, 0x42, 0x40, 0x3e, 0x0f, 0x29,
        0x59, // 0x10
        0x65, 0x38, 0x2a, 0x70, 0x1d, 0x10, 0x02, 0x5a, 0x66, 0x71, 0x2c, 0x1f, 0x1e, 0x11, 0x03,
        0x5b, // 0x20
        0x67, 0x2e, 0x2d, 0x20, 0x12, 0x05, 0x04, 0x5c, 0x68, 0x39, 0x2f, 0x21, 0x14, 0x13, 0x06,
        0x5d, // 0x30
        0x69, 0x31, 0x30, 0x23, 0x22, 0x15, 0x07, 0x5e, 0x6a, 0x72, 0x32, 0x24, 0x16, 0x08, 0x09,
        0x5f, // 0x40
        0x6b, 0x33, 0x25, 0x17, 0x18, 0x0b, 0x0a, 0x60, 0x6c, 0x34, 0x35, 0x26, 0x27, 0x19, 0x0c,
        0x61, // 0x50
        0x6d, 0x73, 0x28, 0x74, 0x1a, 0x0d, 0x62, 0x6e, 0x3a, 0x36, 0x1c, 0x1b, 0x75, 0x2b, 0x63,
        0x76, // 0x60
        0x55, 0x56, 0x77, 0x78, 0x79, 0x7a, 0x0e, 0x7b, 0x7c, 0x4f, 0x7d, 0x4b, 0x47, 0x7e, 0x7f,
        0x6f, // 0x70
        0x52, 0x53, 0x50, 0x4c, 0x4d, 0x48, 0x01, 0x45, 0x57, 0x4e, 0x51, 0x4a, 0x37, 0x49, 0x46,
        0x00, // 0x80
        0x00, 0x00, 0x00, 0x41, 0x54, 0x5b, 0x5f,
    ];

    let mut table = [0u8; 256];
    let mut i = 0;
    while i < 256 {
        table[i] = i as u8;
        i += 1;
    }
    let mut i = 0;
    while i < HEAD.len() {
        table[i] = HEAD[i];
        i += 1;
    }
    table
}

/// Which nibble of the input port a poll command is holding in the status
/// register, if either (commands `0xC1` and `0xC2`).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
enum Poll {
    /// Nothing: status bits 4-7 mean what they normally mean.
    #[default]
    None,
    /// The input port's low nibble, in status bits 4-7.
    Low,
    /// Its high nibble, in status bits 4-7.
    High,
}

/// What the next write to the data port is for.
///
/// Most controller commands take a parameter, and the 8042 has nowhere to put
/// it but the data port — so the command latches what the *next* data write
/// means. Getting this wrong is how a controller ends up typing its own A20
/// setting at the guest.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
enum Pending {
    /// The keyboard: an ordinary command byte on its way down the cable.
    #[default]
    Keyboard,
    /// Controller RAM byte N (command `0x60`-`0x7F`).
    RamWrite(u8),
    /// The output port (command `0xD1`).
    OutputPort,
    /// The output buffer, as if the keyboard had sent it (command `0xD2`).
    InjectKeyboard,
    /// The output buffer, as if the auxiliary device had (command `0xD3`).
    InjectAux,
    /// The auxiliary device (command `0xD4`).
    ToAux,
}

/// What the next write to the keyboard is for — the same problem one level
/// down the cable.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
enum KbPending {
    /// A command.
    #[default]
    Command,
    /// The LED bitmap (command `0xED`).
    Leds,
    /// The typematic rate and delay (command `0xF3`).
    Typematic,
    /// The scan code set, or `0` to ask which one is selected (command `0xF0`).
    ScanSet,
}

/// The keyboard on the other end of the cable.
#[derive(Debug)]
struct Keyboard {
    /// Bytes on their way to the controller: scan codes and command replies,
    /// in one queue because the cable is one wire and order is the whole point.
    queue: VecDeque<u8>,
    /// Whether the keyboard is scanning. A disabled keyboard still answers
    /// commands; it just stops reporting keys.
    enabled: bool,
    /// Which scan code set it is sending. Nothing here generates codes, so this
    /// is a number the guest sets and reads back — the *host* decides what the
    /// codes mean, and the module docs say they are set 2.
    scan_set: u8,
    /// The LED bitmap: scroll, num, caps.
    leds: u8,
    /// The typematic rate and delay byte.
    typematic: u8,
    /// What the next byte written to the keyboard is.
    pending: KbPending,
    /// The last byte handed to the controller, for a resend request.
    last_sent: u8,
}

impl Default for Keyboard {
    fn default() -> Keyboard {
        Keyboard {
            queue: VecDeque::new(),
            enabled: true,
            scan_set: DEFAULT_SCAN_SET,
            leds: 0,
            typematic: DEFAULT_TYPEMATIC,
            pending: KbPending::Command,
            // Nothing has been sent, so a resend request before the first byte
            // is answered with an acknowledge rather than with a byte that was
            // never there.
            last_sent: KB_ACK,
        }
    }
}

impl Keyboard {
    /// Queue one byte for the controller, dropping it if the buffer is full.
    fn send(&mut self, byte: u8) {
        if self.queue.len() < KEYBOARD_BUFFER {
            self.queue.push_back(byte);
            self.last_sent = byte;
        }
    }

    /// Whether a scan code from the host would be accepted right now.
    fn accepting(&self) -> bool {
        self.enabled && self.queue.len() < KEYBOARD_BUFFER
    }

    /// Restore what "set defaults" restores: the typematic rate and the LEDs.
    /// Deliberately not the scan code set, which only a reset returns to set 2.
    fn set_defaults(&mut self) {
        self.leds = 0;
        self.typematic = DEFAULT_TYPEMATIC;
        self.pending = KbPending::Command;
    }

    /// Handle one byte the guest wrote to the data port.
    fn write(&mut self, byte: u8) {
        match core::mem::take(&mut self.pending) {
            KbPending::Leds => {
                // Three LEDs; the top five bits are reserved and read back as
                // whatever the keyboard felt like, so they are dropped.
                self.leds = byte & 0x07;
                self.send(KB_ACK);
                return;
            }
            KbPending::Typematic => {
                self.typematic = byte;
                self.send(KB_ACK);
                return;
            }
            KbPending::ScanSet => {
                self.send(KB_ACK);
                if byte == 0 {
                    // Parameter zero is a question, not an assignment.
                    let set = self.scan_set;
                    self.send(set);
                } else {
                    self.scan_set = byte;
                }
                return;
            }
            KbPending::Command => {}
        }
        match byte {
            0xff => {
                // A reset abandons whatever was queued: the acknowledge and
                // then, after the basic assurance test, 0xAA.
                self.queue.clear();
                *self = Keyboard::default();
                self.send(KB_ACK);
                self.send(KB_BAT_OK);
            }
            0xfe => {
                // Nothing in this model can lose a byte between the keyboard
                // and the controller, so there is never anything genuinely to
                // resend; the keyboard repeats its last reply.
                let last = self.last_sent;
                self.send(last);
            }
            0xf6 => {
                self.set_defaults();
                self.send(KB_ACK);
            }
            0xf5 => {
                self.enabled = false;
                self.set_defaults();
                self.send(KB_ACK);
            }
            0xf4 => {
                self.enabled = true;
                self.send(KB_ACK);
            }
            0xf3 => {
                self.pending = KbPending::Typematic;
                self.send(KB_ACK);
            }
            0xf2 => {
                self.send(KB_ACK);
                self.send(KB_ID_HIGH);
                self.send(KB_ID_LOW);
            }
            0xf0 => {
                self.pending = KbPending::ScanSet;
                self.send(KB_ACK);
            }
            0xed => {
                self.pending = KbPending::Leds;
                self.send(KB_ACK);
            }
            // Echo (0xEE) and the per-key typematic commands of a PS/2
            // keyboard are not modelled; a keyboard answers what it does not
            // understand with a resend request, and so does this one.
            _ => self.send(KB_RESEND),
        }
    }
}

/// Everything the guest can see or change.
#[derive(Debug)]
struct State {
    /// Controller RAM. Byte 0 is the command byte, and the other 31 are the
    /// scratch the firmware uses for whatever it likes.
    ram: [u8; RAM_LEN],
    /// The byte waiting for the CPU at the data port.
    obuf: u8,
    /// Whether there is one.
    obf: bool,
    /// Whether it came from the auxiliary device, which is status bit 5 and
    /// decides whether IRQ1 or IRQ12 is the one that fires.
    from_aux: bool,
    /// A2: whether the last write went to the command port.
    a2: bool,
    /// Whether a poll command is holding an input-port nibble in the status
    /// register.
    poll: Poll,
    /// What the next data-port write is for.
    pending: Pending,
    /// A set-2 break prefix has been seen and applies to the next code.
    ///
    /// Controller state rather than keyboard state: it is the *translator* that
    /// folds `0xF0` into bit 7, and with translation off the prefix reaches the
    /// guest untouched.
    break_pending: bool,
    /// The output port latch: reset line, A20 gate, and the two buffer-full
    /// signals that are recomputed on a read.
    outport: u8,
    /// The keyboard.
    kbd: Keyboard,
}

impl Default for State {
    fn default() -> State {
        State {
            // Zero: no interrupts, both clocks running, no translation. Real
            // firmware writes the command byte during POST before it cares
            // about a keystroke, and picking a value here that a machine
            // without firmware happened to like would be a guess.
            ram: [0; RAM_LEN],
            obuf: 0,
            obf: false,
            from_aux: false,
            a2: false,
            poll: Poll::None,
            pending: Pending::Keyboard,
            break_pending: false,
            outport: OUTPUT_PORT_RESET,
            kbd: Keyboard::default(),
        }
    }
}

/// What has to happen once the state lock is released.
///
/// Driving a wire from inside the critical section is the re-entrancy bug the
/// contract exists to stop (`CLAUDE.md`): the reset pin reaches a CPU, and a
/// machine that reset itself from inside this device's write handler would
/// re-enter it holding its own lock.
#[derive(Debug, Default, Clone, Copy)]
struct Outward {
    /// Pulse the reset line.
    reset: bool,
}

/// The chip: state, pins, and the cable to the host.
struct Shared {
    state: Mutex<State>,
    /// The keyboard interrupt, at [`LockRank::LEAF`] so it can be driven with
    /// nothing else held.
    irq1: Mutex<Option<WireSource>>,
    /// The auxiliary-device interrupt.
    irq12: Mutex<Option<WireSource>>,
    /// The A20 gate.
    a20: Mutex<Option<WireSource>>,
    /// The system reset line.
    reset: Mutex<Option<WireSource>>,
    /// Where scan codes come from.
    port: Arc<dyn CharDevice>,
    /// The name the port was opened under, for `Debug` and diagnostics.
    port_name: String,
}

impl fmt::Debug for Shared {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let mut s = f.debug_struct("Shared");
        s.field("port", &self.port_name);
        match self.state.try_lock() {
            Some(state) => s.field("state", &*state).finish(),
            None => s.field("state", &"<in use>").finish(),
        }
    }
}

impl Shared {
    // -- the pins ----------------------------------------------------------

    /// Drive one output, with no lock held.
    fn drive(holder: &Mutex<Option<WireSource>>, level: Level) {
        let out = holder.lock().clone();
        if let Some(out) = out {
            out.set(level);
        }
    }

    /// Recompute every level output from the current state and drive it.
    ///
    /// Takes the state lock, reads three booleans out of it and drops it before
    /// touching a wire. The reset pin is not here: it is a pulse, not a level,
    /// and belongs to whatever asked for it.
    fn refresh(&self) {
        let (irq1, irq12, a20) = {
            let state = self.state.lock();
            let (a, b) = Self::interrupts(&state);
            (a, b, state.outport & OP_A20 != 0)
        };
        Self::drive(&self.irq1, Level::from_bool(irq1));
        Self::drive(&self.irq12, Level::from_bool(irq12));
        Self::drive(&self.a20, Level::from_bool(a20));
    }

    /// Whether each interrupt is being requested.
    ///
    /// The output buffer feeds exactly one of them, chosen by where the byte
    /// came from, and each is gated by its enable in the command byte.
    fn interrupts(state: &State) -> (bool, bool) {
        let kbd = state.obf && !state.from_aux && state.ram[0] & CB_KBD_INT != 0;
        let aux = state.obf && state.from_aux && state.ram[0] & CB_AUX_INT != 0;
        (kbd, aux)
    }

    /// Pulse the system reset line.
    ///
    /// The 8042's pin is active low; a net here idles low and carries the
    /// *logical* assertion, so a reset is a high pulse and a machine file that
    /// wants the electrical sense puts a `wire.not` in the way (`ROADMAP.md`
    /// §4.3, and `riscv::syscon`, which resets the same way).
    fn pulse_reset(&self) {
        let out = self.reset.lock().clone();
        if let Some(out) = out {
            out.pulse(Level::High);
        }
    }

    /// Do what an [`Outward`] asked for, then republish every level.
    fn settle(&self, out: Outward) {
        if out.reset {
            self.pulse_reset();
        }
        self.refresh();
    }

    // -- moving bytes ------------------------------------------------------

    /// Move what the keyboard has into the output buffer, translating on the
    /// way if the command byte says to.
    ///
    /// A disabled keyboard clock stops the transfer rather than discarding the
    /// bytes: they wait in the keyboard's buffer, which is what a real keyboard
    /// with its clock line held low does.
    fn transfer(state: &mut State) {
        if state.ram[0] & CB_KBD_CLOCK_OFF != 0 {
            return;
        }
        let translate = state.ram[0] & CB_TRANSLATE != 0;
        while !state.obf {
            let Some(raw) = state.kbd.queue.pop_front() else {
                break;
            };
            let byte = if !translate {
                raw
            } else if raw == SET2_BREAK {
                // The prefix is not a byte in set 1: it becomes bit 7 of the
                // code it applies to, so nothing is emitted yet.
                state.break_pending = true;
                continue;
            } else if raw == SET2_EXTEND {
                // Both sets spell the extended prefix the same way, and it must
                // not swallow a pending break — `E0 F0 xx` is one key coming up.
                raw
            } else {
                let mut b = TRANSLATE[raw as usize];
                if state.break_pending {
                    b |= 0x80;
                    state.break_pending = false;
                }
                b
            };
            state.obuf = byte;
            state.obf = true;
            state.from_aux = false;
        }
    }

    /// Put a byte in the output buffer as if the controller itself sent it.
    fn reply(state: &mut State, byte: u8) {
        state.obuf = byte;
        state.obf = true;
        state.from_aux = false;
    }

    /// Take scan codes off the character port and hand them to the keyboard.
    ///
    /// Nothing is read while the keyboard is not accepting, so a user typing
    /// before the guest has enabled scanning waits at the port rather than
    /// having the keystrokes thrown away — the seam's back-pressure rule, and
    /// the difference between a slow boot and a lost password.
    fn pump(&self) {
        {
            let mut state = self.state.lock();
            while state.kbd.accepting() {
                let Some(byte) = self.port.read_byte() else {
                    break;
                };
                state.kbd.send(byte);
            }
            Self::transfer(&mut state);
        }
        self.refresh();
    }

    // -- the data port, 0x60 -----------------------------------------------

    /// Read the data port. `debug` suppresses every side effect.
    fn read_data(&self, debug: bool) -> u8 {
        let mut state = self.state.lock();
        if debug {
            // A debugger looking at 0x60 must not eat the guest's keystroke,
            // clear OBF or drop IRQ1 (`CLAUDE.md`: a debug read pops nothing).
            return state.obuf;
        }
        let byte = state.obuf;
        state.obf = false;
        state.from_aux = false;
        Self::transfer(&mut state);
        byte
    }

    /// Write the data port: a parameter for the last command, or a byte for
    /// the keyboard.
    fn write_data(&self, value: u8) {
        let out = {
            let mut state = self.state.lock();
            let mut out = Outward::default();
            state.a2 = false;
            match core::mem::take(&mut state.pending) {
                Pending::Keyboard => state.kbd.write(value),
                Pending::RamWrite(index) => state.ram[index as usize] = value,
                Pending::OutputPort => {
                    state.outport = value;
                    // Bit 0 is active low, so a *clear* bit is the reset.
                    if value & OP_RESET == 0 {
                        out.reset = true;
                    }
                }
                Pending::InjectKeyboard => {
                    // 0xD2 puts the byte in the buffer verbatim: it never came
                    // off the cable, so there is nothing to translate.
                    Self::reply(&mut state, value);
                }
                Pending::InjectAux => {
                    Self::reply(&mut state, value);
                    state.from_aux = true;
                }
                // No auxiliary device is modelled, and nothing answers a byte
                // sent to one. When a mouse arrives it plugs in here.
                Pending::ToAux => {}
            }
            Self::transfer(&mut state);
            out
        };
        self.settle(out);
    }

    // -- the command port, 0x64 --------------------------------------------

    /// Read the status register.
    fn read_status(&self) -> u8 {
        let state = self.state.lock();
        let mut status = 0;
        if state.obf {
            status |= ST_OBF;
            if state.from_aux {
                status |= ST_AUX_OBF;
            }
        }
        if state.ram[0] & CB_SYS != 0 {
            status |= ST_SYS;
        }
        if state.a2 {
            status |= ST_A2;
        }
        if state.ram[0] & CB_KBD_CLOCK_OFF != 0 {
            status |= ST_INH;
        }
        // A poll command replaces the top nibble with half the input port for
        // as long as it is in force, which is the whole point of it: firmware
        // reads the jumpers without disturbing the output buffer.
        match state.poll {
            Poll::None => status,
            Poll::Low => (status & 0x0f) | ((INPUT_PORT & 0x0f) << 4),
            Poll::High => (status & 0x0f) | (INPUT_PORT & 0xf0),
        }
    }

    /// The output port as a read of it reports: the latch, with the two
    /// buffer-full signals recomputed rather than remembered.
    fn output_port_readback(state: &State) -> u8 {
        let mut value = state.outport & !(OP_KBD_OBF | OP_AUX_OBF);
        if state.obf {
            value |= if state.from_aux {
                OP_AUX_OBF
            } else {
                OP_KBD_OBF
            };
        }
        value
    }

    /// Write the command port.
    fn write_command(&self, value: u8) {
        let out = {
            let mut state = self.state.lock();
            let mut out = Outward::default();
            state.a2 = true;
            // A new command ends a poll and abandons any parameter the previous
            // one was still waiting for.
            state.poll = Poll::None;
            state.pending = Pending::Keyboard;
            match value {
                0x20..=0x3f => {
                    let byte = state.ram[(value & 0x1f) as usize];
                    Self::reply(&mut state, byte);
                }
                0x60..=0x7f => state.pending = Pending::RamWrite(value & 0x1f),
                0xa7 => state.ram[0] |= CB_AUX_CLOCK_OFF,
                0xa8 => state.ram[0] &= !CB_AUX_CLOCK_OFF,
                // The auxiliary interface test on a controller with nothing
                // plugged into it: no error.
                0xa9 => Self::reply(&mut state, 0x00),
                0xaa => {
                    // The self test sets the system flag, and the flag *is*
                    // command byte bit 2 — status bit 2 is a window onto it.
                    state.ram[0] |= CB_SYS;
                    Self::reply(&mut state, SELF_TEST_OK);
                }
                0xab => Self::reply(&mut state, 0x00),
                0xad => state.ram[0] |= CB_KBD_CLOCK_OFF,
                0xae => state.ram[0] &= !CB_KBD_CLOCK_OFF,
                0xc0 => Self::reply(&mut state, INPUT_PORT),
                0xc1 => state.poll = Poll::Low,
                0xc2 => state.poll = Poll::High,
                0xd0 => {
                    let byte = Self::output_port_readback(&state);
                    Self::reply(&mut state, byte);
                }
                0xd1 => state.pending = Pending::OutputPort,
                0xd2 => state.pending = Pending::InjectKeyboard,
                0xd3 => state.pending = Pending::InjectAux,
                0xd4 => state.pending = Pending::ToAux,
                0xe0 => Self::reply(&mut state, TEST_INPUTS),
                0xf0..=0xff => {
                    // The low nibble names the output lines to pulse for six
                    // microseconds, and a line is named by its bit being
                    // *clear*. 0xFE therefore pulses line 0 — the reset — which
                    // is how every PC built since 1984 leaves protected mode.
                    //
                    // Line 1 is the A20 gate, and no other line is wired here.
                    // Pulsing A20 is meaningless at this model's resolution —
                    // the gate would be back where it started before the next
                    // instruction — so the latch, and the pin, are left alone.
                    let pulsed = !value & 0x0f;
                    out.reset = pulsed & OP_RESET != 0;
                }
                // An 8042 ignores a command it does not implement, and answers
                // nothing; firmware that waits for a reply times out.
                _ => {}
            }
            Self::transfer(&mut state);
            out
        };
        self.settle(out);
    }
}

/// The data port, 0x60.
#[derive(Debug)]
struct DataPort(Arc<Shared>);

/// The status and command port, 0x64.
#[derive(Debug)]
struct CommandPort(Arc<Shared>);

impl MemOps for DataPort {
    fn read(&self, _offset: u64, dst: &mut [u8], attrs: MemAttrs) -> MemResult {
        let [byte] = dst else {
            return Err(BusError::BadAccess);
        };
        *byte = self.0.read_data(attrs.debug);
        if !attrs.debug {
            self.0.refresh();
        }
        Ok(())
    }

    fn write(&self, _offset: u64, src: &[u8], attrs: MemAttrs) -> MemResult {
        let [value] = src else {
            return Err(BusError::BadAccess);
        };
        if attrs.debug {
            // A debug write here would type at the guest, or set the A20 gate
            // out from under it. Neither can be made harmless.
            return Err(BusError::BadAccess);
        }
        self.0.write_data(*value);
        Ok(())
    }

    fn constraints(&self) -> AccessConstraints {
        // One byte at one address, on an 8-bit peripheral bus.
        AccessConstraints::word(Width::U8, Endian::Little)
    }
}

impl MemOps for CommandPort {
    fn read(&self, _offset: u64, dst: &mut [u8], _attrs: MemAttrs) -> MemResult {
        let [byte] = dst else {
            return Err(BusError::BadAccess);
        };
        // Reading the status register has no side effect at all, so a debug
        // read is the same read and is allowed to happen.
        *byte = self.0.read_status();
        Ok(())
    }

    fn write(&self, _offset: u64, src: &[u8], attrs: MemAttrs) -> MemResult {
        let [value] = src else {
            return Err(BusError::BadAccess);
        };
        if attrs.debug {
            return Err(BusError::BadAccess);
        }
        self.0.write_command(*value);
        Ok(())
    }

    fn constraints(&self) -> AccessConstraints {
        AccessConstraints::word(Width::U8, Endian::Little)
    }
}

/// An Intel 8042 keyboard controller, with a keyboard attached.
#[derive(Debug)]
pub struct Kbc8042 {
    shared: Arc<Shared>,
    data: RegionRef,
    cmd: RegionRef,
}

impl Kbc8042 {
    /// Validate `props` and build the device.
    ///
    /// # Errors
    ///
    /// [`Error::Property`] if a property is of
    /// the wrong kind, or if one this class does not know was given.
    pub fn new(props: &Props) -> Result<Kbc8042> {
        let mut r = props.reader();
        let port_name = r.or("port", String::from(DEFAULT_PORT))?;
        r.finish()?;
        Ok(Kbc8042::with_port(
            ports::attach(props, &port_name)?,
            port_name,
        ))
    }

    /// One on a private port, with no properties set.
    ///
    /// Private because there is no build to rendezvous in: a controller made
    /// this way meets nobody, which is what a unit test wants. Go through
    /// [`Kbc8042::new`] with a `Props` from a build, or
    /// [`Kbc8042::with_port`], to reach a port a host can also hold.
    #[must_use]
    pub fn default_device() -> Kbc8042 {
        Kbc8042::with_port(
            Arc::new(crate::host::chardev::CharPort::new()),
            String::from(DEFAULT_PORT),
        )
    }

    /// Build one against a character device the caller already has.
    #[must_use]
    pub fn with_port(port: Arc<dyn CharDevice>, port_name: String) -> Kbc8042 {
        let shared = Arc::new(Shared {
            state: Mutex::with_rank(LockRank::DEVICE, State::default()),
            irq1: Mutex::with_rank(LockRank::LEAF, None),
            irq12: Mutex::with_rank(LockRank::LEAF, None),
            a20: Mutex::with_rank(LockRank::LEAF, None),
            reset: Mutex::with_rank(LockRank::LEAF, None),
            port,
            port_name,
        });
        let data: RegionRef = Arc::new(Region::io(
            "pc.kbc.data",
            REGISTER_WINDOW_LEN,
            Arc::new(DataPort(Arc::clone(&shared))) as Arc<dyn MemOps>,
        ));
        let cmd: RegionRef = Arc::new(Region::io(
            "pc.kbc.cmd",
            REGISTER_WINDOW_LEN,
            Arc::new(CommandPort(Arc::clone(&shared))) as Arc<dyn MemOps>,
        ));
        Kbc8042 { shared, data, cmd }
    }

    /// The name of the character port the keyboard is attached to.
    #[must_use]
    pub fn port_name(&self) -> &str {
        &self.shared.port_name
    }

    /// Take whatever scan codes the host has fed and give them to the keyboard.
    ///
    /// This is what [`Device::run`] does; a test that is not running a
    /// scheduler calls it directly.
    pub fn pump(&self) {
        self.shared.pump();
    }

    /// Whether the A20 gate is open.
    #[must_use]
    pub fn a20_enabled(&self) -> bool {
        self.shared.state.lock().outport & OP_A20 != 0
    }

    /// Whether the keyboard interrupt is being requested.
    #[must_use]
    pub fn irq1_asserted(&self) -> bool {
        Shared::interrupts(&self.shared.state.lock()).0
    }

    /// Whether the auxiliary interrupt is being requested.
    #[must_use]
    pub fn irq12_asserted(&self) -> bool {
        Shared::interrupts(&self.shared.state.lock()).1
    }
}

/// The `pc.kbc` device class.
pub static CLASS: DeviceClass = DeviceClass {
    name: CLASS_NAME,
    version: STATE_VERSION,
    summary: "Intel 8042 keyboard controller, with the A20 gate",
    properties: &[PropertySpec {
        name: "port",
        kind: ValueKind::Str,
        required: false,
        summary: "the character port scan codes arrive on (default \"keyboard\")",
    }],
    construct: |props| Ok(Box::new(Kbc8042::new(props)?)),
};

impl Device for Kbc8042 {
    fn class(&self) -> &'static DeviceClass {
        &CLASS
    }

    fn realize(&self, _ctx: &mut RealizeCtx<'_>) -> Result<()> {
        Ok(())
    }

    fn reset(&self, _kind: ResetKind) {
        // Warm and cold are the same here: nothing in an 8042 is battery
        // backed, and a warm reset is usually this chip's own doing anyway.
        {
            let mut state = self.shared.state.lock();
            *state = State::default();
        }
        // A20 closes on reset, and something has to say so.
        self.shared.refresh();
    }

    fn region(&self, name: &str) -> Option<RegionRef> {
        match name {
            "" | "data" => Some(Arc::clone(&self.data)),
            "cmd" => Some(Arc::clone(&self.cmd)),
            _ => None,
        }
    }

    fn connect(&self, port: &str, source: WireSource) -> Result<()> {
        let holder = match port {
            "irq1" => &self.shared.irq1,
            "irq12" => &self.shared.irq12,
            "a20" => &self.shared.a20,
            "reset" => &self.shared.reset,
            _ => {
                return Err(Error::Config {
                    at: port.to_string(),
                    message: String::from("an 8042 drives `irq1`, `irq12`, `a20` and `reset`"),
                });
            }
        };
        *holder.lock() = Some(source);
        Ok(())
    }

    fn announce(&self, port: &str) {
        match port {
            // A20 in particular: the gate is shut out of reset and a machine
            // whose address decoder came up disagreeing would be wrong from
            // the first fetch.
            "irq1" | "irq12" | "a20" => self.shared.refresh(),
            // The reset line is a pulse, and idles where a fresh net already
            // is; announcing it would be indistinguishable from a reboot.
            _ => {}
        }
    }

    fn is_runnable(&self) -> bool {
        // Not because it executes anything, but because scan codes have to be
        // taken off the host port, and the scheduler is the only thing allowed
        // to decide when (`CLAUDE.md`: a device never reads the wall clock).
        true
    }

    fn run(&self, budget: Budget) -> Consumed {
        self.shared.pump();
        Consumed::new(budget.ticks)
    }

    fn save(&self, w: &mut ChunkWriter<'_>) -> Result<()> {
        let state = self.shared.state.lock();
        w.write_all(&state.ram)?;
        w.write_u8(state.obuf)?;
        w.write_bool(state.obf)?;
        w.write_bool(state.from_aux)?;
        w.write_bool(state.a2)?;
        w.write_u8(match state.poll {
            Poll::None => 0,
            Poll::Low => 1,
            Poll::High => 2,
        })?;
        let (tag, arg) = match state.pending {
            Pending::Keyboard => (0, 0),
            Pending::RamWrite(index) => (1, index),
            Pending::OutputPort => (2, 0),
            Pending::InjectKeyboard => (3, 0),
            Pending::InjectAux => (4, 0),
            Pending::ToAux => (5, 0),
        };
        w.write_u8(tag)?;
        w.write_u8(arg)?;
        w.write_bool(state.break_pending)?;
        w.write_u8(state.outport)?;

        w.write_seq_len(state.kbd.queue.len() as u64)?;
        for byte in &state.kbd.queue {
            w.write_u8(*byte)?;
        }
        w.write_bool(state.kbd.enabled)?;
        w.write_u8(state.kbd.scan_set)?;
        w.write_u8(state.kbd.leds)?;
        w.write_u8(state.kbd.typematic)?;
        w.write_u8(match state.kbd.pending {
            KbPending::Command => 0,
            KbPending::Leds => 1,
            KbPending::Typematic => 2,
            KbPending::ScanSet => 3,
        })?;
        w.write_u8(state.kbd.last_sent)
        // The character port's queues are the host's state, not the machine's,
        // and are deliberately absent (`ROADMAP.md` §4.5) — the same reason
        // `uart.ns16550` leaves them out.
    }

    fn load(&self, r: &mut ChunkReader<'_>) -> Result<()> {
        let mut state = State::default();
        for byte in &mut state.ram {
            *byte = r.read_u8()?;
        }
        state.obuf = r.read_u8()?;
        state.obf = r.read_bool()?;
        state.from_aux = r.read_bool()?;
        state.a2 = r.read_bool()?;
        state.poll = match r.read_u8()? {
            0 => Poll::None,
            1 => Poll::Low,
            2 => Poll::High,
            other => return Err(bad(alloc::format!("poll state {other}"))),
        };
        let tag = r.read_u8()?;
        let arg = r.read_u8()?;
        state.pending = match tag {
            0 => Pending::Keyboard,
            1 if (arg as usize) < RAM_LEN => Pending::RamWrite(arg),
            1 => return Err(bad(alloc::format!("controller RAM byte {arg}"))),
            2 => Pending::OutputPort,
            3 => Pending::InjectKeyboard,
            4 => Pending::InjectAux,
            5 => Pending::ToAux,
            other => return Err(bad(alloc::format!("pending-command tag {other}"))),
        };
        state.break_pending = r.read_bool()?;
        state.outport = r.read_u8()?;

        let count = r.read_seq_len(1)? as usize;
        if count > KEYBOARD_BUFFER {
            return Err(bad(alloc::format!(
                "{count} byte(s) in a {KEYBOARD_BUFFER}-byte keyboard buffer"
            )));
        }
        state.kbd.queue.clear();
        for _ in 0..count {
            state.kbd.queue.push_back(r.read_u8()?);
        }
        state.kbd.enabled = r.read_bool()?;
        state.kbd.scan_set = r.read_u8()?;
        state.kbd.leds = r.read_u8()?;
        state.kbd.typematic = r.read_u8()?;
        state.kbd.pending = match r.read_u8()? {
            0 => KbPending::Command,
            1 => KbPending::Leds,
            2 => KbPending::Typematic,
            3 => KbPending::ScanSet,
            other => return Err(bad(alloc::format!("keyboard command tag {other}"))),
        };
        state.kbd.last_sent = r.read_u8()?;

        *self.shared.state.lock() = state;
        // The restored state implies an A20 level and an interrupt level that
        // nothing has announced.
        self.shared.refresh();
        Ok(())
    }
}

/// A snapshot that does not describe a state this chip can be in.
fn bad(what: String) -> Error {
    Error::State(alloc::format!("8042 snapshot has an impossible {what}"))
}

impl Instance for Kbc8042 {}

/// Add [`CLASS`] to a registry.
///
/// # Errors
///
/// [`Error::Config`] if the name is claimed.
pub fn register(registry: &mut crate::core::Registry) -> Result<()> {
    registry.add(&CLASS)
}

/// Bind [`CLASS`] into the machine graph.
///
/// # Errors
///
/// [`Error::Config`] if the class is bound twice.
pub fn bind(bindings: &mut crate::machine::Bindings) -> Result<()> {
    bindings.bind(CLASS_NAME, |props| Ok(Arc::new(Kbc8042::new(props)?)))
}

/// What the validator should know about `pc.kbc`.
#[must_use]
pub fn schema() -> ClassSchema {
    use crate::machine::validate::{PortDir, PropSchema};
    ClassSchema::new(CLASS_NAME)
        .prop(PropSchema::new("port", ValueKind::Str))
        .region("")
        .region("data")
        .region("cmd")
        .port("irq1", PortDir::Out)
        .port("irq12", PortDir::Out)
        .port("a20", PortDir::Out)
        .port("reset", PortDir::Out)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::state::{MachineShape, Migrations, StateReader, StateWriter};
    use crate::core::sync::{AtomicU32, Ordering};
    use crate::core::wire::{Wire, WireId, WireIdAllocator, WireSink};
    use crate::host::chardev::CharPort;
    use alloc::vec::Vec;

    fn wired() -> (Kbc8042, Arc<CharPort>) {
        let port = Arc::new(CharPort::new());
        let kbc = Kbc8042::with_port(
            Arc::clone(&port) as Arc<dyn CharDevice>,
            String::from("test"),
        );
        (kbc, port)
    }

    /// Read the data port, 0x60.
    fn data(k: &Kbc8042) -> u8 {
        k.shared.read_data(false)
    }

    /// Write the data port.
    fn poke_data(k: &Kbc8042, value: u8) {
        k.shared.write_data(value);
    }

    /// Read the status register, 0x64.
    fn status(k: &Kbc8042) -> u8 {
        k.shared.read_status()
    }

    /// Write the command port.
    fn command(k: &Kbc8042, value: u8) {
        k.shared.write_command(value);
    }

    /// Set the controller command byte the way firmware does.
    fn set_command_byte(k: &Kbc8042, value: u8) {
        command(k, 0x60);
        poke_data(k, value);
    }

    /// A pin that remembers its level and counts every transition, so a pulse
    /// is distinguishable from a line that merely ended up back where it was.
    #[derive(Debug, Default)]
    struct Probe {
        level: AtomicU32,
        edges: AtomicU32,
    }

    impl WireSink for Probe {
        fn set_level(&self, _src: WireId, _line: u32, level: Level) {
            self.level
                .store(u32::from(level.is_high()), Ordering::Relaxed);
            self.edges.fetch_add(1, Ordering::Relaxed);
        }
    }

    impl Probe {
        fn high(&self) -> bool {
            self.level.load(Ordering::Relaxed) == 1
        }

        fn edges(&self) -> u32 {
            self.edges.load(Ordering::Relaxed)
        }
    }

    /// A device with all four outputs probed, in the order they are named.
    fn with_pins() -> (Kbc8042, Arc<CharPort>, Vec<Arc<Probe>>) {
        let (kbc, port) = wired();
        let ids = WireIdAllocator::new();
        let mut probes = Vec::new();
        for pin in ["irq1", "irq12", "a20", "reset"] {
            let id = ids.alloc();
            let probe = Arc::new(Probe::default());
            let wire = Wire::builder()
                .source(id)
                .sink(Arc::clone(&probe) as Arc<dyn WireSink>, 0)
                .build_shared();
            kbc.connect(pin, WireSource::new(wire, id))
                .expect("an 8042 drives all four");
            probes.push(probe);
        }
        (kbc, port, probes)
    }

    #[test]
    fn the_self_test_answers_0x55_and_sets_the_system_flag() {
        let (kbc, _port) = wired();
        assert_eq!(status(&kbc) & ST_SYS, 0, "not yet");
        command(&kbc, 0xaa);
        assert_eq!(status(&kbc) & ST_OBF, ST_OBF);
        assert_eq!(status(&kbc) & ST_SYS, ST_SYS);
        assert_eq!(status(&kbc) & ST_A2, ST_A2, "the last write was 0x64");
        assert_eq!(data(&kbc), SELF_TEST_OK);
        assert_eq!(status(&kbc) & ST_OBF, 0, "and the read emptied it");
    }

    #[test]
    fn the_command_byte_round_trips_through_controller_ram() {
        let (kbc, _port) = wired();
        set_command_byte(&kbc, CB_KBD_INT | CB_TRANSLATE);
        assert_eq!(status(&kbc) & ST_A2, 0, "the last write was 0x60");
        command(&kbc, 0x20);
        assert_eq!(data(&kbc), CB_KBD_INT | CB_TRANSLATE);

        // And so does a scratch byte, which is what the other 31 are for.
        command(&kbc, 0x60 | 0x17);
        poke_data(&kbc, 0x5a);
        command(&kbc, 0x20 | 0x17);
        assert_eq!(data(&kbc), 0x5a);
    }

    #[test]
    fn a_scan_code_from_the_host_fills_the_buffer_and_raises_irq1() {
        let (kbc, port, probes) = with_pins();
        set_command_byte(&kbc, CB_KBD_INT);
        port.feed(&[0x1c]);
        kbc.pump();

        assert_eq!(status(&kbc) & ST_OBF, ST_OBF);
        assert_eq!(status(&kbc) & ST_AUX_OBF, 0, "it came from the keyboard");
        assert!(probes[0].high(), "IRQ1");
        assert!(!probes[1].high(), "and not IRQ12");

        let mut byte = [0u8; 1];
        DataPort(Arc::clone(&kbc.shared))
            .read(0, &mut byte, MemAttrs::DEFAULT)
            .expect("a byte read is legal");
        assert_eq!(byte[0], 0x1c, "untranslated, since bit 6 is clear");
        assert_eq!(status(&kbc) & ST_OBF, 0);
        assert!(!probes[0].high(), "and the read dropped the interrupt");
    }

    #[test]
    fn a_scan_code_raises_nothing_while_the_interrupt_is_disabled() {
        let (kbc, port, probes) = with_pins();
        port.feed(&[0x1c]);
        kbc.pump();
        assert_eq!(status(&kbc) & ST_OBF, ST_OBF, "the byte is still there");
        assert!(!probes[0].high(), "but nothing asked for an interrupt");
    }

    #[test]
    fn a_debug_read_of_the_data_port_pops_nothing() {
        let (kbc, port, probes) = with_pins();
        set_command_byte(&kbc, CB_KBD_INT);
        port.feed(&[0x2a]);
        kbc.pump();

        let ops = DataPort(Arc::clone(&kbc.shared));
        let mut byte = [0u8; 1];
        ops.read(0, &mut byte, MemAttrs::DEBUG)
            .expect("a debug read is legal");
        assert_eq!(byte[0], 0x2a);
        assert_eq!(status(&kbc) & ST_OBF, ST_OBF, "still full");
        assert!(probes[0].high(), "and still interrupting");
        assert_eq!(data(&kbc), 0x2a, "the guest gets the same byte");

        // A debug *write* cannot be made harmless, so it is refused outright.
        assert!(ops.write(0, &[0xff], MemAttrs::DEBUG).is_err());
        let cmd = CommandPort(Arc::clone(&kbc.shared));
        assert!(cmd.write(0, &[0xaa], MemAttrs::DEBUG).is_err());
        // Reading the status register has no side effect, so it is allowed.
        assert!(cmd.read(0, &mut byte, MemAttrs::DEBUG).is_ok());
    }

    #[test]
    fn writing_the_output_port_drives_the_a20_gate() {
        let (kbc, _port, probes) = with_pins();
        let a20 = &probes[2];
        assert!(!a20.high(), "shut out of reset, as on a real AT");

        command(&kbc, 0xd1);
        poke_data(&kbc, OP_RESET | OP_A20);
        assert!(a20.high());
        assert!(kbc.a20_enabled());

        // And a read of the output port reports what was written, plus the
        // buffer-full signal the chip recomputes.
        command(&kbc, 0xd0);
        assert_eq!(data(&kbc) & OP_A20, OP_A20);

        command(&kbc, 0xd1);
        poke_data(&kbc, OP_RESET);
        assert!(!a20.high());
        assert!(!kbc.a20_enabled());
    }

    #[test]
    fn command_0xfe_pulses_the_reset_line() {
        let (kbc, _port, probes) = with_pins();
        let reset = &probes[3];
        let before = reset.edges();
        assert!(!reset.high());

        command(&kbc, 0xfe);
        assert!(reset.edges() > before, "the line moved");
        assert!(!reset.high(), "and came back: a pulse, not a level");

        // The other route to the same pin: the reset line is active low, so
        // writing the output port with bit 0 *clear* is a reset too.
        let before = reset.edges();
        command(&kbc, 0xd1);
        poke_data(&kbc, 0x00);
        assert!(reset.edges() > before);

        // A pulse command that does not name line 0 leaves it alone.
        let before = reset.edges();
        command(&kbc, 0xff);
        assert_eq!(reset.edges(), before);
    }

    #[test]
    fn the_keyboard_acknowledges_what_it_knows_and_refuses_what_it_does_not() {
        let (kbc, _port) = wired();
        poke_data(&kbc, 0xf4);
        assert_eq!(data(&kbc), KB_ACK, "enable scanning");

        poke_data(&kbc, 0x99);
        assert_eq!(data(&kbc), KB_RESEND, "and a command it never heard of");

        // A command with a parameter acknowledges both halves.
        poke_data(&kbc, 0xed);
        assert_eq!(data(&kbc), KB_ACK);
        poke_data(&kbc, 0x07);
        assert_eq!(data(&kbc), KB_ACK);

        // Identify is the two-byte reply firmware uses to tell a keyboard from
        // a mouse.
        poke_data(&kbc, 0xf2);
        assert_eq!(data(&kbc), KB_ACK);
        assert_eq!(data(&kbc), KB_ID_HIGH);
        assert_eq!(data(&kbc), KB_ID_LOW);
    }

    #[test]
    fn a_keyboard_reset_acknowledges_and_then_passes_its_self_test() {
        let (kbc, _port) = wired();
        poke_data(&kbc, 0xff);
        assert_eq!(data(&kbc), KB_ACK);
        assert_eq!(data(&kbc), KB_BAT_OK);
        assert_eq!(status(&kbc) & ST_OBF, 0, "and nothing follows");
    }

    #[test]
    fn translation_happens_only_when_the_command_byte_asks_for_it() {
        let (kbc, port) = wired();
        // Set 2's `A` is 0x1C; set 1's is 0x1E.
        port.feed(&[0x1c]);
        kbc.pump();
        assert_eq!(data(&kbc), 0x1c, "untranslated");

        set_command_byte(&kbc, CB_TRANSLATE);
        port.feed(&[0x1c]);
        kbc.pump();
        assert_eq!(data(&kbc), 0x1e, "translated");
    }

    #[test]
    fn a_set_2_break_sequence_becomes_one_set_1_byte() {
        let (kbc, port) = wired();
        set_command_byte(&kbc, CB_TRANSLATE);
        // 0xF0 0x1C: the `A` key coming up.
        port.feed(&[0xf0, 0x1c]);
        kbc.pump();
        assert_eq!(data(&kbc), 0x1e | 0x80, "one byte, with bit 7 set");
        assert_eq!(status(&kbc) & ST_OBF, 0, "the prefix was consumed");

        // The extended prefix passes through, and a break after it still folds
        // into the code rather than into the prefix.
        port.feed(&[0xe0, 0xf0, 0x14]);
        kbc.pump();
        assert_eq!(data(&kbc), 0xe0);
        assert_eq!(data(&kbc), 0x1d | 0x80, "right control, coming up");

        // Without translation the guest sees the set-2 stream verbatim.
        set_command_byte(&kbc, 0);
        port.feed(&[0xf0, 0x1c]);
        kbc.pump();
        assert_eq!(data(&kbc), 0xf0);
        assert_eq!(data(&kbc), 0x1c);
    }

    #[test]
    fn disabling_the_keyboard_clock_stops_scan_codes_at_the_keyboard() {
        let (kbc, port) = wired();
        command(&kbc, 0xad);
        assert_eq!(status(&kbc) & ST_INH, ST_INH, "and it says so");
        port.feed(&[0x1c]);
        kbc.pump();
        assert_eq!(status(&kbc) & ST_OBF, 0, "nothing reached the buffer");

        // Re-enabling releases it: the byte was waiting, not thrown away.
        command(&kbc, 0xae);
        kbc.pump();
        assert_eq!(status(&kbc) & ST_OBF, ST_OBF);
        assert_eq!(data(&kbc), 0x1c);
    }

    #[test]
    fn a_byte_can_be_injected_from_either_side_and_lands_on_its_own_interrupt() {
        let (kbc, _port, probes) = with_pins();
        set_command_byte(&kbc, CB_KBD_INT | CB_AUX_INT);

        command(&kbc, 0xd2);
        poke_data(&kbc, 0x5a);
        assert!(probes[0].high(), "0xD2 is the keyboard's side");
        assert_eq!(status(&kbc) & ST_AUX_OBF, 0);
        assert_eq!(data(&kbc), 0x5a);

        command(&kbc, 0xd3);
        poke_data(&kbc, 0x5b);
        assert_eq!(status(&kbc) & ST_AUX_OBF, ST_AUX_OBF, "0xD3 is the mouse's");
        assert!(probes[1].high(), "IRQ12");
        assert!(!probes[0].high());
        assert_eq!(data(&kbc), 0x5b);
    }

    #[test]
    fn the_interface_tests_and_the_input_port_answer_what_firmware_expects() {
        let (kbc, _port) = wired();
        command(&kbc, 0xab);
        assert_eq!(data(&kbc), 0x00, "keyboard interface: no error");
        command(&kbc, 0xa9);
        assert_eq!(data(&kbc), 0x00, "auxiliary interface: no error");
        command(&kbc, 0xc0);
        assert_eq!(data(&kbc), INPUT_PORT);
        command(&kbc, 0xe0);
        assert_eq!(data(&kbc), TEST_INPUTS);

        // The poll commands put half the input port in the status register
        // instead, and leave the output buffer alone.
        command(&kbc, 0xc2);
        assert_eq!(status(&kbc) & 0xf0, INPUT_PORT & 0xf0);
        command(&kbc, 0xc1);
        assert_eq!(status(&kbc) & 0xf0, (INPUT_PORT & 0x0f) << 4);
        command(&kbc, 0xaa);
        assert_eq!(status(&kbc) & 0xf0, 0, "and any command ends the poll");
    }

    #[test]
    fn an_access_that_is_not_a_single_byte_is_refused() {
        let (kbc, _port) = wired();
        let data_ops = DataPort(Arc::clone(&kbc.shared));
        let cmd_ops = CommandPort(Arc::clone(&kbc.shared));
        assert!(data_ops.read(0, &mut [0u8; 2], MemAttrs::DEFAULT).is_err());
        assert!(data_ops.write(0, &[0u8; 4], MemAttrs::DEFAULT).is_err());
        assert!(cmd_ops.read(0, &mut [0u8; 2], MemAttrs::DEFAULT).is_err());
        assert!(cmd_ops.write(0, &[0u8; 4], MemAttrs::DEFAULT).is_err());
    }

    #[test]
    fn the_two_ports_are_separate_regions_and_the_empty_name_is_the_data_port() {
        let (kbc, _port) = wired();
        assert!(kbc.region("").is_some());
        assert!(kbc.region("data").is_some());
        assert!(kbc.region("cmd").is_some());
        assert!(kbc.region("regs").is_none(), "there is no combined block");
        assert_eq!(kbc.region("").unwrap().len(), REGISTER_WINDOW_LEN);

        // The empty name and "data" are the same region, which is what makes
        // `map iobus 0x60 = kbc` mean the data port.
        let same = Arc::ptr_eq(&kbc.region("").unwrap(), &kbc.region("data").unwrap());
        assert!(same);
    }

    #[test]
    fn a_pin_this_chip_does_not_drive_is_a_configuration_error() {
        let (kbc, _port) = wired();
        let ids = WireIdAllocator::new();
        let id = ids.alloc();
        let wire = Wire::builder().source(id).build_shared();
        assert!(kbc.connect("irq", WireSource::new(wire, id)).is_err());
    }

    #[test]
    fn a_snapshot_round_trips_every_bit_of_architectural_state() {
        let (saved, port) = wired();
        // Mutate as much as there is: the command byte, a scratch RAM byte, the
        // output port, the keyboard's settings, and a half-finished command
        // with a scan code stuck behind a disabled clock.
        set_command_byte(&saved, CB_KBD_INT | CB_TRANSLATE);
        command(&saved, 0x60 | 0x05);
        poke_data(&saved, 0xc3);
        command(&saved, 0xd1);
        poke_data(&saved, OP_RESET | OP_A20);
        poke_data(&saved, 0xf3);
        poke_data(&saved, 0x20);
        poke_data(&saved, 0xed);
        command(&saved, 0xad);
        port.feed(&[0x1c, 0xf0]);
        saved.pump();
        command(&saved, 0xd4);

        let image = |dev: &Kbc8042| {
            let mut shape = MachineShape::new();
            shape.add_device("kbc", CLASS.name).unwrap();
            let mut w = StateWriter::new(shape);
            {
                let mut chunk = w.chunk("kbc", CLASS.name, CLASS.version).unwrap();
                dev.save(&mut chunk).unwrap();
            }
            w.to_vec().unwrap()
        };

        let first = image(&saved);
        let (restored, _other) = wired();
        let reader = StateReader::new(&first).unwrap();
        let chunk = reader
            .load("kbc", CLASS.name, CLASS.version, &Migrations::new())
            .unwrap();
        restored.load(&mut chunk.reader()).unwrap();

        assert_eq!(image(&restored), first, "the two images are identical");
        assert!(restored.a20_enabled(), "and A20 came back open");

        // And the restored chip behaves: the buffer still holds the reply the
        // keyboard had already handed over, the acknowledges behind it are
        // still queued, and releasing the clock delivers the scan code that was
        // stuck behind them, translated as the command byte says.
        assert!(
            restored.irq1_asserted(),
            "and the interrupt came back with it"
        );
        command(&restored, 0xae);
        restored.pump();
        for _ in 0..3 {
            assert_eq!(data(&restored), KB_ACK, "an acknowledge the keyboard owed");
        }
        assert_eq!(data(&restored), 0x1e);
        assert_eq!(
            status(&restored) & ST_OBF,
            0,
            "and the trailing break prefix is still waiting for its code"
        );
    }

    #[test]
    fn properties_are_checked_rather_than_ignored() {
        let kbc = Kbc8042::new(&Props::new()).expect("no properties is legal");
        assert_eq!(kbc.port_name(), DEFAULT_PORT);
        let named = Kbc8042::new(&Props::new().with("port", "test.kbc.props"))
            .expect("a port name is legal");
        assert_eq!(named.port_name(), "test.kbc.props");
        assert!(Kbc8042::new(&Props::new().with("prot", "x")).is_err());
    }

    #[test]
    fn the_translation_table_is_the_documented_one() {
        // A handful of well-known entries, as a guard against a transcription
        // slip in 135 hand-entered numbers.
        for (set2, set1) in [
            (0x76u8, 0x01u8), // escape
            (0x16, 0x02),     // 1
            (0x1c, 0x1e),     // A
            (0x5a, 0x1c),     // enter
            (0x29, 0x39),     // space
            (0x66, 0x0e),     // backspace
            (0x12, 0x2a),     // left shift
            (0x14, 0x1d),     // left control
            (0x11, 0x38),     // left alt
            (0x58, 0x3a),     // caps lock
            (0x05, 0x3b),     // F1
            (0x83, 0x41),     // F7, the one reply byte translation changes
        ] {
            assert_eq!(TRANSLATE[set2 as usize], set1, "set 2 {set2:#04x}");
        }
        // Everything above the table's range is the identity, which is what
        // lets a keyboard's replies survive a translating controller.
        for byte in [KB_ACK, KB_BAT_OK, KB_RESEND, KB_ID_HIGH, SET2_EXTEND] {
            assert_eq!(TRANSLATE[byte as usize], byte);
        }
    }
}
