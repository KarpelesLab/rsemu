//! The navi's audio microcontroller as the SoC meets it over TAB-II: a
//! clocked-synchronous serial link on which the panel keys and the steering
//! switches arrive as switch bitmaps.
//!
//! # The link
//!
//! SCIF5 runs clocked-synchronous, so every byte the SoC sends clocks one
//! back, and this peer answers on the [`UartLink`](crate::bus::uart::UartLink)
//! as a [`Responder`] — inside the SoC's transmit. Because the answer is
//! clocked out *with* the byte, it is chosen from the bytes before it:
//!
//! * after `0x1C` (the SoC asks to read), `0x8C` if a frame is waiting and
//!   `0xBC` if not; a waiting frame then streams out, one byte per clock;
//! * after anything else, `0xAC` (acknowledge).
//!
//! The SoC ends a read with `0x2C` (taken) or `0x3C` (send it again).
//! Frames are `[0x8C][L][payload][sum]`, `L` counting the whole frame and the
//! last byte making the byte sum zero. What the SoC writes is acknowledged
//! and not interpreted.
//!
//! # The lines
//!
//! * `ans` (GPIO 115) pulses high after each byte: the SoC's driver waits
//!   for it before the next one, and refuses an answer much later than 2 ms.
//!   The pulse is raised in the run slice after the byte, so the reply is
//!   already in the SoC's receive FIFO when the driver looks.
//! * `req` (GPIO 92) tells the SoC there is something to read. While a frame
//!   waits it rises, and falls [`REQ_HIGH_TICKS`] later; the falling edge is
//!   what the SoC's reader waits for. It repeats every [`REQ_PERIOD_TICKS`]
//!   until the frame is taken.
//!
//! # Keys
//!
//! A press or release queues the whole bitmap of its opcode: `0x1A` for the
//! panel keys, `0x1D` for SEEK and the steering switches. Each key is a bit
//! of one byte. The key manager numbers those bytes from the one before the
//! opcode, so its byte `n` is payload byte `n - 1` (`key-offset = 1`, the
//! default): in a live run, counting from the opcode opened nothing and this
//! opened the screens. Holding and repeating are the SoC's business.
//!
//! # Sources
//!
//! No documentation. The protocol, frame format, lines and key tables come
//! from the unit's proprietary TAB-II driver and key manager read as data, and
//! from the link's traffic in a run.

use alloc::boxed::Box;
use alloc::collections::VecDeque;
use alloc::string::{String, ToString};
use alloc::sync::Arc;
use alloc::vec::Vec;

use crate::bus::uart::{Responder, links};
use crate::core::device::{Device, DeviceClass, PropertySpec, RealizeCtx, ResetKind};
use crate::core::error::{Error, Result};
use crate::core::props::{Props, ValueKind};
use crate::core::sched::{Budget, Consumed};
use crate::core::state::{ChunkReader, ChunkWriter, Sink, Source};
use crate::core::sync::{LockRank, Mutex};
use crate::core::wire::{Level, WireSource};
use crate::machine::realize::Instance;

/// The class name a machine description writes.
pub const CLASS_NAME: &str = "navi.tab";

/// The snapshot chunk version. Bump with the encoding, never on its own.
const STATE_VERSION: u32 = 1;

/// The answer strobe.
pub const ANS_PIN: &str = "ans";
/// The data-ready line.
pub const REQ_PIN: &str = "req";

/// How long `req` stays high before its falling edge, in device-clock ticks
/// (milliseconds on the navi's 1 kHz clock).
pub const REQ_HIGH_TICKS: u64 = 10;
/// How often `req` repeats while a frame waits.
pub const REQ_PERIOD_TICKS: u64 = 100;

/// The SoC asks to read.
const READ: u8 = 0x1c;
/// The SoC took the frame.
const TAKEN: u8 = 0x2c;
/// The SoC wants it again.
const AGAIN: u8 = 0x3c;
/// A frame follows.
const FRAME: u8 = 0x8c;
/// Acknowledge.
const ACK: u8 = 0xac;
/// Nothing to send.
const NOTHING: u8 = 0xbc;

/// Panel-key bitmap opcode, and how many bytes from the opcode it spans.
const OP_PANEL: u8 = 0x1a;
const PANEL_LEN: usize = 19;
/// Steering-switch (and SEEK) bitmap opcode.
const OP_STEER: u8 = 0x1d;
const STEER_LEN: usize = 4;

/// Frames kept waiting at most; the oldest goes first.
const MAX_QUEUE: usize = 16;

/// One control: its name, the bitmap it lives in, the byte and the bit.
#[derive(Debug, Clone, Copy)]
pub struct Key {
    /// What the owner's manual calls it.
    pub name: &'static str,
    opcode: u8,
    byte: usize,
    bit: u8,
}

/// The controls this model knows: the ten panel keys of the key manager's
/// own list. `info`, `audio` and `open` are confirmed by the screen each one
/// opened. The others did nothing visible from the map-error screen they
/// were tried on, and carry the owner's manual's remaining names, unconfirmed.
#[rustfmt::skip]
pub const KEYS: &[Key] = &[
    Key { name: "setup", opcode: OP_PANEL, byte: 9, bit: 5 },
    Key { name: "info", opcode: OP_PANEL, byte: 9, bit: 4 },
    Key { name: "destination", opcode: OP_PANEL, byte: 6, bit: 4 },
    Key { name: "position", opcode: OP_PANEL, byte: 6, bit: 5 },
    Key { name: "tv", opcode: OP_PANEL, byte: 10, bit: 1 },
    Key { name: "audio", opcode: OP_PANEL, byte: 9, bit: 6 },
    Key { name: "traffic", opcode: OP_PANEL, byte: 6, bit: 6 },
    Key { name: "open", opcode: OP_PANEL, byte: 7, bit: 5 },
    Key { name: "seek-down", opcode: OP_STEER, byte: 2, bit: 2 },
    Key { name: "seek-up", opcode: OP_STEER, byte: 2, bit: 3 },
];

/// The key named `name`.
#[must_use]
pub fn key(name: &str) -> Option<usize> {
    KEYS.iter().position(|k| k.name == name)
}

#[derive(Debug, Clone, PartialEq, Eq, Default)]
struct State {
    /// The byte the next clock sends back.
    pending: u8,
    /// The rest of the frame being read out.
    out: VecDeque<u8>,
    /// The front frame has been streamed and waits for `0x2C` or `0x3C`.
    inflight: bool,
    queue: VecDeque<Vec<u8>>,
    /// Bytes clocked since the last `ans` pulse.
    clocked: bool,
    ans: bool,
    req: bool,
    /// Ticks since `req` last rose.
    req_age: u64,
    panel: [u8; PANEL_LEN],
    steer: [u8; STEER_LEN],
}

impl State {
    fn new() -> State {
        State {
            pending: ACK,
            req_age: REQ_PERIOD_TICKS,
            ..State::default()
        }
    }

    fn clock(&mut self, byte: u8) -> u8 {
        let reply = self.pending;
        self.clocked = true;
        if let Some(next) = self.out.pop_front() {
            self.pending = next;
            if self.out.is_empty() {
                self.inflight = true;
            }
            return reply;
        }
        self.pending = match byte {
            READ => match self.queue.front() {
                Some(f) if !self.inflight => {
                    self.out = f[1..].iter().copied().collect();
                    FRAME
                }
                _ => NOTHING,
            },
            _ => ACK,
        };
        match byte {
            TAKEN if self.inflight => {
                self.inflight = false;
                self.queue.pop_front();
            }
            AGAIN => self.inflight = false,
            _ => {}
        }
        reply
    }

    /// Queue the whole bitmap of `opcode`. Byte 0 of a bitmap is the
    /// opcode's own place.
    fn push(&mut self, opcode: u8) {
        let body: &[u8] = if opcode == OP_PANEL {
            &self.panel
        } else {
            &self.steer
        };
        let mut payload = alloc::vec![opcode];
        payload.extend_from_slice(&body[1..]);
        let mut f = alloc::vec![FRAME, (payload.len() + 3) as u8];
        f.extend_from_slice(&payload);
        let sum = f.iter().fold(0u8, |a, b| a.wrapping_add(*b));
        f.push(0u8.wrapping_sub(sum));
        if self.queue.len() >= MAX_QUEUE {
            // Never the one being read.
            let at = usize::from(self.inflight || !self.out.is_empty());
            if at < self.queue.len() {
                self.queue.remove(at);
            }
        }
        self.queue.push_back(f);
    }
}

#[derive(Debug)]
struct Shared {
    state: Mutex<State>,
}

impl Responder for Shared {
    fn clock(&self, byte: u8) -> u8 {
        self.state.lock().clock(byte)
    }
}

/// The audio microcontroller's TAB-II side.
#[derive(Debug)]
pub struct Tab {
    shared: Arc<Shared>,
    link: Arc<crate::bus::uart::UartLink>,
    ans: Mutex<Option<WireSource>>,
    req: Mutex<Option<WireSource>>,
    offset: usize,
}

impl Tab {
    /// Validate `props` and build the device.
    ///
    /// # Errors
    ///
    /// [`Error::Property`] on a missing, malformed or unknown property.
    pub fn new(props: &Props) -> Result<Tab> {
        let mut r = props.reader();
        let link = r.require_str("link")?.to_string();
        let offset = r.or_range("key-offset", 1u64, 0..=1)? as usize;
        r.finish()?;
        let link = links::attach(props, &link)?;
        Ok(Tab::on(link, offset))
    }

    fn on(link: Arc<crate::bus::uart::UartLink>, offset: usize) -> Tab {
        Tab {
            shared: Arc::new(Shared {
                // A leaf: it is taken inside the SCIF's transmit.
                state: Mutex::with_rank(LockRank::LEAF, State::new()),
            }),
            link,
            ans: Mutex::with_rank(LockRank::LEAF, None),
            req: Mutex::with_rank(LockRank::LEAF, None),
            offset,
        }
    }

    /// Press or release the control at `index` in [`KEYS`].
    pub fn key(&self, index: usize, down: bool) {
        let Some(k) = KEYS.get(index) else {
            return;
        };
        let mut s = self.shared.state.lock();
        let byte = k.byte - self.offset;
        let bits = if k.opcode == OP_PANEL {
            &mut s.panel[..]
        } else {
            &mut s.steer[..]
        };
        let Some(b) = bits.get_mut(byte) else {
            return;
        };
        let was = *b;
        if down {
            *b |= 1 << k.bit;
        } else {
            *b &= !(1 << k.bit);
        }
        if *b != was {
            s.push(k.opcode);
        }
    }

    fn line(slot: &Mutex<Option<WireSource>>, high: bool) {
        // Bind first: the guard must be gone before the wire reaches its sink.
        let out = slot.lock().clone();
        if let Some(out) = out {
            out.set(Level::from_bool(high));
        }
    }

    fn advance(&self, ticks: u64) {
        let (lower_ans, raise_ans, req) = {
            let mut s = self.shared.state.lock();
            let lower = s.ans;
            let raise = s.clocked;
            s.clocked = false;
            s.ans = raise;
            s.req_age = s.req_age.saturating_add(ticks);
            let waiting = !s.queue.is_empty() && !s.inflight && s.out.is_empty();
            if waiting && !s.req && s.req_age >= REQ_PERIOD_TICKS {
                s.req = true;
                s.req_age = 0;
            } else if s.req && s.req_age >= REQ_HIGH_TICKS {
                s.req = false;
            }
            (lower, raise, s.req)
        };
        if lower_ans {
            Tab::line(&self.ans, false);
        }
        if raise_ans {
            Tab::line(&self.ans, true);
        }
        Tab::line(&self.req, req);
    }
}

/// The device class.
pub static CLASS: DeviceClass = DeviceClass {
    name: CLASS_NAME,
    version: STATE_VERSION,
    summary: "the navi's audio microcontroller on TAB-II: key bitmaps over a clocked serial link",
    properties: &[
        PropertySpec {
            name: "link",
            kind: ValueKind::Str,
            required: true,
            summary: "the UART link to the SoC's SCIF; this device is its end `b`",
        },
        PropertySpec {
            name: "key-offset",
            kind: ValueKind::Uint,
            required: false,
            summary: "1 (default) if key byte numbers count from the byte before the opcode, 0 if from it",
        },
    ],
    construct: |props| Ok(Box::new(Tab::new(props)?)),
};

impl Device for Tab {
    fn class(&self) -> &'static DeviceClass {
        &CLASS
    }

    fn realize(&self, _ctx: &mut RealizeCtx<'_>) -> Result<()> {
        // This device is end `b`; the SCIF holds `a`.
        self.link
            .respond_on_b(Arc::clone(&self.shared) as Arc<dyn Responder>);
        Ok(())
    }

    fn reset(&self, _kind: ResetKind) {
        *self.shared.state.lock() = State::new();
        Tab::line(&self.ans, false);
        Tab::line(&self.req, false);
    }

    fn connect(&self, port: &str, source: WireSource) -> Result<()> {
        let slot = match port {
            ANS_PIN => &self.ans,
            REQ_PIN => &self.req,
            _ => {
                return Err(Error::Config {
                    at: port.to_string(),
                    message: String::from("the TAB-II peer drives `ans` and `req`"),
                });
            }
        };
        *slot.lock() = Some(source);
        Ok(())
    }

    fn announce(&self, port: &str) {
        let s = self.shared.state.lock().clone();
        match port {
            ANS_PIN => Tab::line(&self.ans, s.ans),
            REQ_PIN => Tab::line(&self.req, s.req),
            _ => {}
        }
    }

    fn is_runnable(&self) -> bool {
        // For the `ans` pulse and the `req` timing.
        true
    }

    fn run(&self, budget: Budget) -> Consumed {
        self.advance(budget.ticks);
        Consumed::new(budget.ticks)
    }

    fn save(&self, w: &mut ChunkWriter<'_>) -> Result<()> {
        let s = self.shared.state.lock().clone();
        w.write_u8(s.pending)?;
        w.write_bytes(&s.out.iter().copied().collect::<Vec<u8>>())?;
        w.write_bool(s.inflight)?;
        w.write_seq_len(s.queue.len() as u64)?;
        for f in &s.queue {
            w.write_bytes(f)?;
        }
        w.write_bool(s.clocked)?;
        w.write_bool(s.ans)?;
        w.write_bool(s.req)?;
        w.write_u64(s.req_age)?;
        w.write_bytes(&s.panel)?;
        w.write_bytes(&s.steer)
    }

    fn load(&self, r: &mut ChunkReader<'_>) -> Result<()> {
        let mut s = State::new();
        s.pending = r.read_u8()?;
        s.out = r.read_bytes()?.iter().copied().collect();
        s.inflight = r.read_bool()?;
        let n = r.read_seq_len(1)? as usize;
        if n > MAX_QUEUE {
            return Err(Error::State(alloc::format!(
                "a TAB snapshot with {n} queued frames"
            )));
        }
        for _ in 0..n {
            s.queue.push_back(r.read_bytes()?.to_vec());
        }
        s.clocked = r.read_bool()?;
        s.ans = r.read_bool()?;
        s.req = r.read_bool()?;
        s.req_age = r.read_u64()?;
        let (panel, steer) = (r.read_bytes()?, r.read_bytes()?);
        if panel.len() != PANEL_LEN || steer.len() != STEER_LEN {
            return Err(Error::State(String::from(
                "a TAB snapshot with other bitmap sizes",
            )));
        }
        s.panel.copy_from_slice(panel);
        s.steer.copy_from_slice(steer);
        let (ans, req) = (s.ans, s.req);
        *self.shared.state.lock() = s;
        Tab::line(&self.ans, ans);
        Tab::line(&self.req, req);
        Ok(())
    }
}

impl Instance for Tab {}

/// Add this class to a registry.
///
/// # Errors
///
/// If something already claimed the name.
pub fn register(registry: &mut crate::core::Registry) -> Result<()> {
    registry.add(&CLASS)
}

/// Bind this class into the machine graph.
///
/// # Errors
///
/// If the name is already bound.
pub fn bind(bindings: &mut crate::machine::Bindings) -> Result<()> {
    bindings.bind(CLASS_NAME, |props| Ok(Arc::new(Tab::new(props)?)))
}

/// The validator's view of this class.
#[must_use]
pub fn schema() -> crate::machine::validate::ClassSchema {
    use crate::machine::validate::{ClassSchema, PortDir, PropSchema};
    ClassSchema::new(CLASS_NAME)
        .prop(PropSchema::new("link", ValueKind::Str).required())
        .prop(PropSchema::new("key-offset", ValueKind::Uint).range(0, 1))
        .port(ANS_PIN, PortDir::Out)
        .port(REQ_PIN, PortDir::Out)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::bus::uart::{Side, UartLink};
    use crate::core::state::{MachineShape, Migrations, StateReader, StateWriter};
    use crate::host::chardev::CharDevice;

    fn tab() -> (Tab, Arc<crate::bus::uart::LinkEnd>) {
        let link = Arc::new(UartLink::new());
        let t = Tab::on(Arc::clone(&link), 1);
        link.respond_on_b(Arc::clone(&t.shared) as Arc<dyn Responder>);
        (t, link.end(Side::A))
    }

    /// Clock `bytes` out of the SoC's end and collect what came back.
    fn clock(soc: &crate::bus::uart::LinkEnd, bytes: &[u8]) -> Vec<u8> {
        assert_eq!(soc.write(bytes), bytes.len());
        let mut back = alloc::vec![0u8; bytes.len()];
        assert_eq!(soc.read(&mut back), bytes.len());
        back
    }

    #[test]
    fn a_write_is_acknowledged() {
        let (_t, soc) = tab();
        let frame = [0x0c, 0x04, 0x04, 0xec];
        let back = clock(&soc, &[&frame[..], &[0xff]].concat());
        assert_eq!(back[4], ACK, "the trailing clock reads the acknowledge");
    }

    #[test]
    fn with_nothing_to_send_a_read_says_so() {
        let (_t, soc) = tab();
        assert_eq!(clock(&soc, &[READ, 0xff])[1], NOTHING);
    }

    #[test]
    fn a_key_is_read_as_a_checksummed_bitmap_and_taken() {
        let (t, soc) = tab();
        t.key(key("audio").unwrap(), true);
        t.advance(REQ_PERIOD_TICKS);
        assert!(t.shared.state.lock().req, "data ready");
        let head = clock(&soc, &[READ, 0xff, 0xff]);
        assert_eq!(&head[1..], [FRAME, (PANEL_LEN + 3) as u8]);
        let rest = clock(&soc, &alloc::vec![0xff; PANEL_LEN + 1]);
        let mut frame = alloc::vec![FRAME, (PANEL_LEN + 3) as u8];
        frame.extend_from_slice(&rest);
        assert_eq!(frame[2], OP_PANEL);
        assert_eq!(
            frame[2 + 8],
            1 << 6,
            "AUDIO: the key manager's byte 9, bit 6"
        );
        assert_eq!(
            frame.iter().fold(0u8, |a, b| a.wrapping_add(*b)),
            0,
            "sums to zero"
        );
        clock(&soc, &[TAKEN]);
        assert!(t.shared.state.lock().queue.is_empty());
        assert_eq!(clock(&soc, &[READ, 0xff])[1], NOTHING);
    }

    #[test]
    fn a_nak_sends_the_frame_again() {
        let (t, soc) = tab();
        t.key(key("seek-up").unwrap(), true);
        let first = clock(&soc, &[READ, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff]);
        clock(&soc, &[AGAIN]);
        let again = clock(&soc, &[READ, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff]);
        assert_eq!(first, again);
        assert_eq!(first[3], OP_STEER);
        assert_eq!(
            first[4],
            1 << 3,
            "SEEK up: the key manager's byte 2, bit 3, of 0x1D"
        );
    }

    #[test]
    fn each_byte_gets_an_answer_pulse() {
        let (t, soc) = tab();
        clock(&soc, &[0x0c]);
        t.advance(1);
        assert!(t.shared.state.lock().ans, "raised the slice after the byte");
        t.advance(1);
        assert!(!t.shared.state.lock().ans, "and lowered the next");
    }

    #[test]
    fn a_snapshot_round_trips() {
        let (t, soc) = tab();
        t.key(key("setup").unwrap(), true);
        t.key(key("setup").unwrap(), false);
        clock(&soc, &[READ, 0xff, 0xff]);
        t.advance(3);
        let save = |t: &Tab| {
            let mut shape = MachineShape::new();
            shape.add_device("tab", CLASS.name).unwrap();
            let mut wr = StateWriter::new(shape);
            {
                let mut chunk = wr.chunk("tab", CLASS.name, CLASS.version).unwrap();
                t.save(&mut chunk).unwrap();
            }
            wr.to_vec().unwrap()
        };
        let bytes = save(&t);
        let (back, _) = tab();
        let reader = StateReader::new(&bytes).unwrap();
        let chunk = reader
            .load("tab", CLASS.name, CLASS.version, &Migrations::new())
            .unwrap();
        back.load(&mut chunk.reader()).unwrap();
        let restored = back.shared.state.lock().clone();
        assert_eq!(restored, *t.shared.state.lock());
        assert_eq!(save(&back), bytes);
    }
}
