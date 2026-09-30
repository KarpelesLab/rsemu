//! The base board's power-supply-control processor, as its serial peer sees
//! it.
//!
//! The Alphard navi's Aisin computer board plugs into a Panasonic base board
//! whose MN103-family microcontroller owns the power rails, the fans and a
//! bank of I/O the SoC cannot reach — the SD slot's supply among them. The
//! kernel talks to it over SCIF3 with an ACK/NAK packet protocol it calls
//! PSC. Nothing of the MCU itself is modelled; this is the protocol peer,
//! which is all the SoC can observe.
//!
//! # Frames
//!
//! ```text
//!   SOH  CMD  LEN  payload[LEN]  CS        CS = (CMD + LEN + Σpayload) & 0xff
//!   01   xx   nn   …             xx
//! ```
//!
//! with one exception, the sync frame `0f 00 01 00 01` the host opens with.
//!
//! | From the host | Answer |
//! | --- | --- |
//! | sync (`0x0f`) | ACK |
//! | `0x20` START | ACK, then STAT `0xa0`: ten bytes, the five port bytes last |
//! | `0x2a` VERG | ACK, then VERD `0xaa`: the version byte |
//! | `0x26` PORTR | ACK, then PORTD `0xa6`: the five port bytes |
//! | `0x3d` FANRPMGET | ACK, then FANRPMD `0xbd`: the fan speed in units of 50 rpm |
//! | `0x25` PORTW | ACK; the two five-byte vectors are applied to the output image and logged |
//! | anything else well-formed | ACK |
//!
//! ACK is `01 06 00 06`. A frame whose checksum fails is answered with NAK
//! (`01 15 01 00 16`), which the host retries.
//!
//! # The cyclic notice
//!
//! After START the MCU sends STAT unprompted, every `cycle` ticks. The host
//! counts on it: every 500 ms its driver checks that a STAT has arrived since
//! the last check and, if none has, concludes the base board is gone and
//! resets the unit (`psc_ltc_BreakCycleNti` → `psc_tif_ResetNavi` in the
//! kernel image). A sync frame or a reset stops the notices until the next
//! START.
//!
//! # PORTW
//!
//! The payload is two five-byte port vectors. Which is the value and which
//! the mask is not known from the host side alone; the model keeps both
//! vectors as the MCU's output image (`out = (out & !second) | (first &
//! second)`) and logs them verbatim, which is what a person tracing the
//! SD-slot power switch needs to see.
//!
//! # Sources
//!
//! The Alphard navi kernel's PSC driver, read as data from the flash image
//! (packet building in its `SndCmd*` functions, the dispatch in its receive
//! path), and the owner's own notes on it. No Panasonic documentation exists
//! publicly for this MCU's firmware.

use alloc::boxed::Box;
use alloc::format;
use alloc::string::{String, ToString};
use alloc::sync::Arc;
use alloc::vec::Vec;
use core::fmt;

use crate::bus::uart::{Side, links};
use crate::core::device::{Device, DeviceClass, PropertySpec, RealizeCtx, ResetKind};
use crate::core::error::{Error, Result};
use crate::core::props::{Props, ValueKind};
use crate::core::sched::{Budget, Consumed};
use crate::core::state::{ChunkReader, ChunkWriter, Sink, Source};
use crate::core::sync::{LockRank, Mutex};
use crate::host::chardev::{CharDevice, ports};
use crate::machine::realize::Instance;

/// The class name a machine description writes.
pub const CLASS_NAME: &str = "navi.psc";

/// The snapshot chunk version. Bump with the encoding, never on its own.
const STATE_VERSION: u32 = 2;

const SOH: u8 = 0x01;
const SYNC: u8 = 0x0f;

const ACK: u8 = 0x06;
const NAK: u8 = 0x15;
const STAT: u8 = 0xa0;
const PORTD: u8 = 0xa6;
const VERD: u8 = 0xaa;
const FANRPMD: u8 = 0xbd;

const START: u8 = 0x20;
const DSPSET: u8 = 0x24;
const PORTW: u8 = 0x25;
const PORTR: u8 = 0x26;
const VERG: u8 = 0x2a;
const FANRPMGET: u8 = 0x3d;

/// The default cyclic-notice period, in ticks of the device's clock: a
/// tenth of a second on a millisecond clock, well inside the host's 500 ms.
const DEFAULT_CYCLE: u64 = 100;

/// A frame cannot be longer than this; a length byte that says otherwise is
/// line noise and the byte is dropped.
const MAX_FRAME: usize = 4 + 64;

#[derive(Debug, Clone, PartialEq, Eq)]
struct State {
    /// Bytes received and not yet framed.
    rx: Vec<u8>,
    /// The five input port bytes STAT and PORTD report.
    inputs: [u8; 5],
    /// The output image PORTW writes.
    outputs: [u8; 5],
    /// Frames answered, for a monitor or a test.
    frames: u64,
    /// Whether START has turned the cyclic notice on.
    cyclic: bool,
    /// Ticks since the last cyclic notice.
    elapsed: u64,
}

/// The PSC peer.
pub struct Psc {
    line: Arc<dyn CharDevice>,
    log: Option<Arc<dyn CharDevice>>,
    version: u8,
    fan: u8,
    inputs: [u8; 5],
    cycle: u64,
    state: Mutex<State>,
}

impl fmt::Debug for Psc {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("Psc")
            .field("version", &self.version)
            .finish_non_exhaustive()
    }
}

fn checksum(bytes: &[u8]) -> u8 {
    bytes.iter().fold(0u8, |a, b| a.wrapping_add(*b))
}

/// A frame with its checksum.
fn frame(cmd: u8, payload: &[u8]) -> Vec<u8> {
    let mut f = Vec::with_capacity(payload.len() + 4);
    f.push(SOH);
    f.push(cmd);
    f.push(payload.len() as u8);
    f.extend_from_slice(payload);
    f.push(checksum(&f[1..]));
    f
}

/// STAT: ten bytes, the five port bytes last. What the first five carry is
/// not known; the host's STAT handler reads only the ports.
fn stat(inputs: &[u8; 5]) -> Vec<u8> {
    let mut p = [0u8; 10];
    p[5..].copy_from_slice(inputs);
    frame(STAT, &p)
}

fn command_name(cmd: u8) -> &'static str {
    match cmd {
        START => "START",
        0x21 => "PWMSET?",
        DSPSET => "DSPSET",
        PORTW => "PORTW",
        PORTR => "PORTR",
        VERG => "VERG",
        FANRPMGET => "FANRPMGET",
        _ => "?",
    }
}

impl Psc {
    /// Validate `props` and build the device.
    ///
    /// # Errors
    ///
    /// [`Error::Property`] on a malformed or unknown property.
    pub fn new(props: &Props) -> Result<Psc> {
        let mut r = props.reader();
        let link = r.require_str("link")?.to_string();
        let side = r.or_enum("side", "b", &["a", "b"])?;
        let version = r.or_range("version", 1u64, 0..=0xff)? as u8;
        let fan = r.or_range("fan-rpm", 2000u64, 0..=12_750)?;
        let inputs = r.or_range("inputs", 0u64, 0..=0xff_ffff_ffff)?;
        let log = r.optional_str("log")?.map(ToString::to_string);
        let cycle = r.or_range("cycle", DEFAULT_CYCLE, 1..=u64::MAX / 2)?;
        r.finish()?;
        let side = Side::parse(side).unwrap_or(Side::B);
        let line = links::attach(props, &link)?.end(side) as Arc<dyn CharDevice>;
        let log = match log {
            Some(name) => Some(ports::attach(props, &name)? as Arc<dyn CharDevice>),
            None => None,
        };
        let inputs = inputs.to_le_bytes();
        let inputs = [inputs[0], inputs[1], inputs[2], inputs[3], inputs[4]];
        let mut psc = Psc::with_line(line, log, version, (fan / 50) as u8, inputs);
        psc.cycle = cycle;
        Ok(psc)
    }

    /// Build one on a line the caller already has.
    #[must_use]
    pub fn with_line(
        line: Arc<dyn CharDevice>,
        log: Option<Arc<dyn CharDevice>>,
        version: u8,
        fan: u8,
        inputs: [u8; 5],
    ) -> Psc {
        Psc {
            line,
            log,
            version,
            fan,
            inputs,
            cycle: DEFAULT_CYCLE,
            state: Mutex::with_rank(
                LockRank::DEVICE,
                State {
                    rx: Vec::new(),
                    inputs,
                    outputs: [0; 5],
                    frames: 0,
                    cyclic: false,
                    elapsed: 0,
                },
            ),
        }
    }

    /// How many frames have been answered.
    #[must_use]
    pub fn frames(&self) -> u64 {
        self.state.lock().frames
    }

    /// The output image PORTW has written.
    #[must_use]
    pub fn outputs(&self) -> [u8; 5] {
        self.state.lock().outputs
    }

    /// Let `ticks` pass: send the cyclic notice as often as it falls due.
    pub fn advance(&self, ticks: u64) {
        let mut out = Vec::new();
        {
            let mut s = self.state.lock();
            if !s.cyclic {
                return;
            }
            s.elapsed = s.elapsed.saturating_add(ticks);
            // A long quantum owes several notices; one says the same thing.
            if s.elapsed >= self.cycle {
                s.elapsed %= self.cycle;
                out = stat(&s.inputs);
            }
        }
        if !out.is_empty() {
            self.line.write(&out);
        }
    }

    fn say(&self, text: &str) {
        if let Some(log) = &self.log {
            log.write(text.as_bytes());
        }
    }

    /// Read what the host sent and answer every complete frame.
    pub fn pump(&self) {
        let mut buf = [0u8; 256];
        let mut replies: Vec<u8> = Vec::new();
        let mut lines: Vec<String> = Vec::new();
        {
            let mut s = self.state.lock();
            loop {
                let n = self.line.read(&mut buf);
                if n == 0 {
                    break;
                }
                s.rx.extend_from_slice(&buf[..n]);
            }
            while let Some(&first) = s.rx.first() {
                if first == SYNC {
                    if s.rx.len() < 5 {
                        break;
                    }
                    s.rx.drain(..5);
                    s.frames += 1;
                    s.cyclic = false;
                    replies.extend(frame(ACK, &[]));
                    lines.push(String::from("psc: sync -> ACK\n"));
                    continue;
                }
                if first != SOH {
                    s.rx.remove(0);
                    continue;
                }
                if s.rx.len() < 3 {
                    break;
                }
                let total = usize::from(s.rx[2]) + 4;
                if total > MAX_FRAME {
                    s.rx.remove(0);
                    continue;
                }
                if s.rx.len() < total {
                    break;
                }
                let f: Vec<u8> = s.rx.drain(..total).collect();
                s.frames += 1;
                let cmd = f[1];
                let payload = &f[3..total - 1];
                if checksum(&f[1..total - 1]) != f[total - 1] {
                    replies.extend(frame(NAK, &[0]));
                    lines.push(format!("psc: bad checksum on {cmd:#04x} -> NAK\n"));
                    continue;
                }
                replies.extend(frame(ACK, &[]));
                match cmd {
                    VERG => {
                        replies.extend(frame(VERD, &[self.version]));
                        lines.push(format!("psc: VERG -> version {:#04x}\n", self.version));
                    }
                    START => {
                        replies.extend(stat(&s.inputs));
                        s.cyclic = true;
                        s.elapsed = 0;
                        lines.push(format!("psc: START -> STAT ports {:02x?}\n", s.inputs));
                    }
                    PORTR => {
                        replies.extend(frame(PORTD, &s.inputs));
                        lines.push(format!("psc: PORTR -> {:02x?}\n", s.inputs));
                    }
                    FANRPMGET => {
                        replies.extend(frame(FANRPMD, &[self.fan]));
                        lines.push(format!(
                            "psc: FANRPMGET -> {} rpm\n",
                            u32::from(self.fan) * 50
                        ));
                    }
                    PORTW if payload.len() >= 10 => {
                        for i in 0..5 {
                            s.outputs[i] =
                                (s.outputs[i] & !payload[5 + i]) | (payload[i] & payload[5 + i]);
                        }
                        lines.push(format!(
                            "psc: PORTW {:02x?} {:02x?}\n",
                            &payload[..5],
                            &payload[5..10]
                        ));
                    }
                    _ => {
                        lines.push(format!(
                            "psc: {:#04x} {} {:02x?} -> ACK\n",
                            cmd,
                            command_name(cmd),
                            payload
                        ));
                    }
                }
            }
        }
        if !replies.is_empty() {
            self.line.write(&replies);
        }
        for l in lines {
            self.say(&l);
        }
    }
}

/// The `navi.psc` device class.
pub static CLASS: DeviceClass = DeviceClass {
    name: CLASS_NAME,
    version: STATE_VERSION,
    summary: "the Alphard navi base board's PSC sub-processor, as its ACK/NAK serial peer",
    properties: &[
        PropertySpec {
            name: "link",
            kind: ValueKind::Str,
            required: true,
            summary: "the serial link to the SoC's UART, by name",
        },
        PropertySpec {
            name: "side",
            kind: ValueKind::Str,
            required: false,
            summary: "which end of the link this is (default \"b\")",
        },
        PropertySpec {
            name: "version",
            kind: ValueKind::Uint,
            required: false,
            summary: "the version byte VERD reports (default 1)",
        },
        PropertySpec {
            name: "fan-rpm",
            kind: ValueKind::Uint,
            required: false,
            summary: "the fan speed FANRPMD reports (default 2000)",
        },
        PropertySpec {
            name: "inputs",
            kind: ValueKind::Uint,
            required: false,
            summary: "the five input port bytes, low byte first (default 0)",
        },
        PropertySpec {
            name: "cycle",
            kind: ValueKind::Uint,
            required: false,
            summary: "ticks between the cyclic STAT notices after START (default 100)",
        },
        PropertySpec {
            name: "log",
            kind: ValueKind::Str,
            required: false,
            summary: "a character port to write one line per command to",
        },
    ],
    construct: |props| Ok(Box::new(Psc::new(props)?)),
};

impl Device for Psc {
    fn class(&self) -> &'static DeviceClass {
        &CLASS
    }

    fn realize(&self, _ctx: &mut RealizeCtx<'_>) -> Result<()> {
        Ok(())
    }

    fn reset(&self, kind: ResetKind) {
        let mut s = self.state.lock();
        // The SoC's reset is not the MCU's, but the MCU sees the host go
        // quiet and waits for the next START before it notifies again.
        s.cyclic = false;
        s.elapsed = 0;
        if kind == ResetKind::Cold {
            s.rx.clear();
            s.inputs = self.inputs;
            s.outputs = [0; 5];
        }
    }

    fn is_runnable(&self) -> bool {
        true
    }

    fn run(&self, budget: Budget) -> Consumed {
        self.pump();
        self.advance(budget.ticks);
        Consumed::new(budget.ticks)
    }

    fn save(&self, w: &mut ChunkWriter<'_>) -> Result<()> {
        let s = self.state.lock();
        w.write_bytes(&s.rx)?;
        w.write_bytes(&s.inputs)?;
        w.write_bytes(&s.outputs)?;
        w.write_u64(s.frames)?;
        w.write_bool(s.cyclic)?;
        w.write_u64(s.elapsed)
    }

    fn load(&self, r: &mut ChunkReader<'_>) -> Result<()> {
        let rx = r.read_bytes()?.to_vec();
        let five = |b: &[u8]| -> Result<[u8; 5]> {
            b.try_into()
                .map_err(|_| Error::State(String::from("a port vector is five bytes")))
        };
        let inputs = five(r.read_bytes()?)?;
        let outputs = five(r.read_bytes()?)?;
        let frames = r.read_u64()?;
        let cyclic = r.read_bool()?;
        let elapsed = r.read_u64()?;
        *self.state.lock() = State {
            rx,
            inputs,
            outputs,
            frames,
            cyclic,
            elapsed,
        };
        Ok(())
    }
}

impl Instance for Psc {}

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
    bindings.bind(CLASS_NAME, |props| Ok(Arc::new(Psc::new(props)?)))
}

/// The validator's view of this class.
#[must_use]
pub fn schema() -> crate::machine::validate::ClassSchema {
    use crate::machine::validate::{ClassSchema, PropSchema};
    ClassSchema::new(CLASS_NAME)
        .prop(PropSchema::new("link", ValueKind::Str).required())
        .prop(PropSchema::new("side", ValueKind::Str).values(&["a", "b"]))
        .prop(PropSchema::new("version", ValueKind::Uint).range(0, 0xff))
        .prop(PropSchema::new("fan-rpm", ValueKind::Uint).range(0, 12_750))
        .prop(PropSchema::new("inputs", ValueKind::Uint).range(0, 0xff_ffff_ffff))
        .prop(PropSchema::new("cycle", ValueKind::Uint).range(1, u64::MAX / 2))
        .prop(PropSchema::new("log", ValueKind::Str))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::bus::uart::UartLink;

    fn peer() -> (Psc, Arc<crate::bus::uart::LinkEnd>) {
        let link = Arc::new(UartLink::new());
        let host = link.end(Side::A);
        let psc = Psc::with_line(link.end(Side::B), None, 0x42, 40, [1, 2, 3, 4, 5]);
        (psc, host)
    }

    fn exchange(psc: &Psc, host: &crate::bus::uart::LinkEnd, bytes: &[u8]) -> Vec<u8> {
        host.write(bytes);
        psc.pump();
        let mut out = Vec::new();
        let mut buf = [0u8; 64];
        loop {
            let n = host.read(&mut buf);
            if n == 0 {
                break;
            }
            out.extend_from_slice(&buf[..n]);
        }
        out
    }

    #[test]
    fn the_sync_frame_is_acknowledged() {
        let (psc, host) = peer();
        assert_eq!(exchange(&psc, &host, &[0x0f, 0, 1, 0, 1]), [1, 6, 0, 6]);
    }

    #[test]
    fn verg_is_acknowledged_then_answered_with_the_version() {
        let (psc, host) = peer();
        let reply = exchange(&psc, &host, &frame(VERG, &[]));
        assert_eq!(reply, [1, 6, 0, 6, 1, 0xaa, 1, 0x42, 0xaa + 1 + 0x42]);
    }

    #[test]
    fn start_reports_the_ports_at_the_end_of_stat() {
        let (psc, host) = peer();
        let reply = exchange(&psc, &host, &frame(START, &[]));
        assert_eq!(&reply[4..7], &[1, STAT, 10]);
        assert_eq!(&reply[12..17], &[1, 2, 3, 4, 5]);
    }

    #[test]
    fn a_frame_split_across_reads_is_reassembled() {
        let (psc, host) = peer();
        let f = frame(PORTR, &[]);
        assert!(exchange(&psc, &host, &f[..2]).is_empty());
        let reply = exchange(&psc, &host, &f[2..]);
        assert_eq!(&reply[4..12], &[1, PORTD, 5, 1, 2, 3, 4, 5]);
    }

    #[test]
    fn a_bad_checksum_is_refused() {
        let (psc, host) = peer();
        let mut f = frame(PORTR, &[]);
        *f.last_mut().unwrap() ^= 0xff;
        assert_eq!(exchange(&psc, &host, &f), [1, NAK, 1, 0, NAK + 1]);
    }

    #[test]
    fn portw_writes_the_output_image() {
        let (psc, host) = peer();
        let reply = exchange(
            &psc,
            &host,
            &frame(PORTW, &[0xff, 0, 0, 0, 0, 0x0f, 0, 0, 0, 0]),
        );
        assert_eq!(reply, [1, 6, 0, 6]);
        assert_eq!(psc.outputs(), [0x0f, 0, 0, 0, 0]);
    }

    #[test]
    fn after_start_stat_arrives_every_cycle_until_the_next_sync() {
        let (psc, host) = peer();
        let drain = |host: &crate::bus::uart::LinkEnd| {
            let mut buf = [0u8; 64];
            let mut out = Vec::new();
            loop {
                let n = host.read(&mut buf);
                if n == 0 {
                    break out;
                }
                out.extend_from_slice(&buf[..n]);
            }
        };
        psc.advance(1000);
        assert!(drain(&host).is_empty(), "nothing before START");
        exchange(&psc, &host, &frame(START, &[]));
        psc.advance(DEFAULT_CYCLE - 1);
        assert!(drain(&host).is_empty());
        psc.advance(1);
        assert_eq!(drain(&host), stat(&[1, 2, 3, 4, 5]));
        exchange(&psc, &host, &[0x0f, 0, 1, 0, 1]);
        psc.advance(10 * DEFAULT_CYCLE);
        assert!(drain(&host).is_empty(), "a sync stops the notices");
    }

    #[test]
    fn a_snapshot_round_trips() {
        use crate::core::state::{StateReader, StateWriter};
        let (psc, host) = peer();
        exchange(&psc, &host, &frame(START, &[]));
        psc.advance(42);
        let save = |p: &Psc| {
            let mut shape = crate::core::state::MachineShape::new();
            shape.add_device("psc", CLASS.name).unwrap();
            let mut w = StateWriter::new(shape);
            {
                let mut chunk = w.chunk("psc", CLASS.name, CLASS.version).unwrap();
                p.save(&mut chunk).unwrap();
            }
            w.to_vec().unwrap()
        };
        let bytes = save(&psc);
        let (back, _host) = peer();
        let reader = StateReader::new(&bytes).unwrap();
        let chunk = reader
            .load(
                "psc",
                CLASS.name,
                CLASS.version,
                &crate::core::state::Migrations::new(),
            )
            .unwrap();
        back.load(&mut chunk.reader()).unwrap();
        let want = psc.state.lock().clone();
        assert_eq!(*back.state.lock(), want);
        assert_eq!(save(&back), bytes);
    }
}
