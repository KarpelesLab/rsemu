//! The navi's "SCAC" security coprocessor on the computer board's chip
//! select 3, modelled as a chip that holds no keys.
//!
//! The part is a crypto coprocessor (AES, SHA-256, RSA, random numbers, code
//! integrity) behind a three-register mailbox. The unit's start-up software
//! (`sammc`) reads its status and, finding it unprovisioned, sends one
//! Initialize, logs that the chip holds no information, and carries on. That
//! is the behaviour modelled here: a chip that is present, answers promptly,
//! and has nothing in it. It holds no keys and computes nothing, so every
//! command that would need them — and every verification — reports an error
//! rather than a result.
//!
//! Why it has to answer at all: the kernel driver reads `STATUS` in a busy
//! loop, without yielding, for up to three seconds until the value is neither
//! `0x0000` nor `0xFFFF`. Plain memory reads 0, so the thread spun on its CPU
//! and starved the one that forwards the power-supply controller's status
//! frames, and the PSC driver's 500 ms deadline reset the board.
//!
//! # Register map
//!
//! Three 16-bit registers, **big-endian** (the byte at the lower address is
//! the high byte). The rest of the 64 KiB window reads 0 and ignores writes.
//!
//! | Offset | Read | Write |
//! | --- | --- | --- |
//! | `+0` | `STATUS` | `COMMAND`: runs the command, consuming the `WDATA` words written since the last one |
//! | `+2` | `RDATALEN`: response length in words (bits 10:0) and a tag (15:11, 0 here) | `SCRSUM`: scramble checksum, or `0x5A5A` to acknowledge |
//! | `+4` | `RDATA`: pops the response FIFO; reads past the end give 0 | `WDATA`: pushes a payload word |
//!
//! `STATUS` bits: 0 ready, 1 busy, 2 response available, 3 result ready,
//! 7 acknowledge requested, 12:8 error code, 14 scramble mode, 15
//! provisioned. This chip idles at `0x0001` — ready, unprovisioned, not
//! scrambling — and finishes a command at once with bit 2 set and an error
//! code in 12:8. Reading `RDATALEN` and then every word it announces returns
//! it to idle; reading `STATUS` changes nothing, because the driver's
//! interrupt handler reads it too.
//!
//! Commands, by code: Initialize (`0x78AA`), the code-integrity data chunk
//! (`0x61C2`) and Ping (`0x4719`) succeed with an empty response. The
//! code-integrity finals (`0x0A60`, `0x45FB`) fail with code 10, which the
//! library reports as "verification failed". Everything else fails with
//! code 1, "invalid argument", the error with no retry behind it.
//!
//! The two interrupt lines (reset complete and command complete, GPIOs 76
//! and 77) are not driven: the driver pre-arms the one flag boot waits on.
//!
//! # Sources
//!
//! No datasheet. The register facts come from the navi's binaries read as
//! data — the GPL kernel driver for the register accesses, the proprietary
//! user libraries for the command codes and what callers accept — written up
//! as a facts-only description by a separate pass. No code of theirs is
//! reproduced.

use alloc::boxed::Box;
use alloc::collections::VecDeque;
use alloc::sync::Arc;
use alloc::vec::Vec;

use crate::core::device::{Device, DeviceClass, RealizeCtx, ResetKind};
use crate::core::error::{BusError, Error, Result};
use crate::core::props::Props;
use crate::core::space::{AccessConstraints, MemAttrs, MemOps, MemResult, Region, RegionRef};
use crate::core::state::{ChunkReader, ChunkWriter, Sink, Source};
use crate::core::sync::{LockRank, Mutex};
use crate::core::value::Width;
use crate::machine::realize::Instance;

/// The class name a machine description writes.
pub const CLASS_NAME: &str = "navi.scac";

/// The snapshot chunk version. Bump with the encoding, never on its own.
const STATE_VERSION: u32 = 1;

/// How much address space the chip select decodes.
pub const REGISTER_WINDOW_LEN: u64 = 0x1_0000;

/// `STATUS` / `COMMAND`.
pub const STATUS: u64 = 0;
/// `RDATALEN` / `SCRSUM`.
pub const RDATALEN: u64 = 2;
/// `RDATA` / `WDATA`.
pub const RDATA: u64 = 4;

/// Ready, unprovisioned, not scrambling.
const IDLE: u16 = 0x0001;
/// `STATUS` bit 2: a response is waiting.
const RESPONSE: u16 = 1 << 2;
/// `STATUS` bit 7: the chip wants `0x5A5A` written to `SCRSUM`.
const ACK_REQUESTED: u16 = 1 << 7;
/// The acknowledge word.
const ACK: u16 = 0x5a5a;

/// Initialize.
const CMD_INITIALIZE: u16 = 0x78aa;
/// One chunk of code-integrity data.
const CMD_INTEGRITY_DATA: u16 = 0x61c2;
/// Ping.
const CMD_PING: u16 = 0x4719;
/// Code-integrity final, by table ID.
const CMD_INTEGRITY_FINAL_ID: u16 = 0x0a60;
/// Code-integrity final, by hash.
const CMD_INTEGRITY_FINAL_HASH: u16 = 0x45fb;

/// "Invalid argument": the library returns it without retrying.
const ERR_INVALID: u16 = 1;
/// On a code-integrity final, "verification failed".
const ERR_VERIFY: u16 = 10;

/// The library sends at most 0x800 bytes per command.
const MAX_PAYLOAD_WORDS: usize = 0x400;

#[derive(Debug, Clone, PartialEq, Eq)]
struct State {
    status: u16,
    /// Payload words written since the last command.
    payload: Vec<u16>,
    /// The response still to be read.
    response: VecDeque<u16>,
    /// `RDATALEN` has been read for the pending response.
    announced: bool,
}

impl Default for State {
    fn default() -> State {
        State {
            status: IDLE,
            payload: Vec::new(),
            response: VecDeque::new(),
            announced: false,
        }
    }
}

impl State {
    /// Run `code` on the payload collected so far. Every command finishes
    /// at once.
    fn command(&mut self, code: u16) {
        self.payload.clear();
        self.response.clear();
        self.announced = false;
        let error = match code {
            CMD_INITIALIZE | CMD_INTEGRITY_DATA | CMD_PING => 0,
            CMD_INTEGRITY_FINAL_ID | CMD_INTEGRITY_FINAL_HASH => ERR_VERIFY,
            _ => ERR_INVALID,
        };
        self.status = IDLE | RESPONSE | (error << 8);
    }

    /// Back to idle once the whole response has been taken.
    fn settle(&mut self) {
        if self.announced && self.response.is_empty() {
            self.announced = false;
            self.status = IDLE;
        }
    }

    fn read(&mut self, offset: u64, debug: bool) -> u16 {
        match offset {
            STATUS => self.status,
            RDATALEN => {
                if self.status & RESPONSE == 0 {
                    return 0;
                }
                let words = self.response.len() as u16 & 0x7ff;
                if !debug {
                    self.announced = true;
                    self.settle();
                }
                words
            }
            RDATA => {
                if debug {
                    return self.response.front().copied().unwrap_or(0);
                }
                let word = self.response.pop_front().unwrap_or(0);
                self.settle();
                word
            }
            _ => 0,
        }
    }

    fn write(&mut self, offset: u64, value: u16) {
        match offset {
            STATUS => self.command(value),
            RDATALEN => {
                // A checksum only means something in scramble mode, which
                // this chip never enters.
                if value == ACK {
                    self.status &= !ACK_REQUESTED;
                }
            }
            RDATA if self.payload.len() < MAX_PAYLOAD_WORDS => self.payload.push(value),
            _ => {}
        }
    }
}

#[derive(Debug)]
struct Registers(Arc<Mutex<State>>);

impl MemOps for Registers {
    fn read(&self, offset: u64, dst: &mut [u8], attrs: MemAttrs) -> MemResult {
        if dst.len() != 2 {
            return Err(BusError::BadAccess);
        }
        let v = self.0.lock().read(offset, attrs.debug);
        dst.copy_from_slice(&v.to_be_bytes());
        Ok(())
    }

    fn write(&self, offset: u64, src: &[u8], attrs: MemAttrs) -> MemResult {
        if src.len() != 2 || attrs.debug {
            return Err(BusError::BadAccess);
        }
        self.0
            .lock()
            .write(offset, u16::from_be_bytes([src[0], src[1]]));
        Ok(())
    }

    fn constraints(&self) -> AccessConstraints {
        AccessConstraints::IO
            .with_widths(Width::U16, Width::U16)
            .with_natural_alignment(true)
    }
}

/// The SCAC coprocessor, holding no keys.
#[derive(Debug)]
pub struct Scac {
    state: Arc<Mutex<State>>,
    region: RegionRef,
}

impl Scac {
    /// Build it. It takes no properties.
    ///
    /// # Errors
    ///
    /// [`Error::Property`] if any property is given.
    pub fn new(props: &Props) -> Result<Scac> {
        props.reader().finish()?;
        Ok(Scac::default())
    }
}

impl Default for Scac {
    fn default() -> Scac {
        let state = Arc::new(Mutex::with_rank(LockRank::DEVICE, State::default()));
        let region: RegionRef = Arc::new(Region::io(
            CLASS_NAME,
            REGISTER_WINDOW_LEN,
            Arc::new(Registers(Arc::clone(&state))) as Arc<dyn MemOps>,
        ));
        Scac { state, region }
    }
}

/// The class descriptor.
pub static CLASS: DeviceClass = DeviceClass {
    name: CLASS_NAME,
    version: STATE_VERSION,
    summary: "the navi's SCAC security coprocessor, present and unprovisioned",
    properties: &[],
    construct: |props| Ok(Box::new(Scac::new(props)?)),
};

impl Device for Scac {
    fn class(&self) -> &'static DeviceClass {
        &CLASS
    }

    fn realize(&self, _ctx: &mut RealizeCtx<'_>) -> Result<()> {
        Ok(())
    }

    fn reset(&self, _kind: ResetKind) {
        *self.state.lock() = State::default();
    }

    fn region(&self, name: &str) -> Option<RegionRef> {
        matches!(name, "" | "regs").then(|| Arc::clone(&self.region))
    }

    fn save(&self, w: &mut ChunkWriter<'_>) -> Result<()> {
        let s = self.state.lock().clone();
        w.write_u16(s.status)?;
        w.write_seq_len(s.payload.len() as u64)?;
        for v in &s.payload {
            w.write_u16(*v)?;
        }
        w.write_seq_len(s.response.len() as u64)?;
        for v in &s.response {
            w.write_u16(*v)?;
        }
        w.write_bool(s.announced)
    }

    fn load(&self, r: &mut ChunkReader<'_>) -> Result<()> {
        let status = r.read_u16()?;
        let n = r.read_seq_len(2)? as usize;
        if n > MAX_PAYLOAD_WORDS {
            return Err(Error::State(alloc::format!(
                "snapshot has a {n}-word SCAC payload"
            )));
        }
        let mut payload = Vec::with_capacity(n);
        for _ in 0..n {
            payload.push(r.read_u16()?);
        }
        let n = r.read_seq_len(2)? as usize;
        let mut response = VecDeque::with_capacity(n);
        for _ in 0..n {
            response.push_back(r.read_u16()?);
        }
        let announced = r.read_bool()?;
        *self.state.lock() = State {
            status,
            payload,
            response,
            announced,
        };
        Ok(())
    }
}

impl Instance for Scac {}

/// Add the class to a registry.
///
/// # Errors
///
/// If something already claimed the name.
pub fn register(registry: &mut crate::core::Registry) -> Result<()> {
    registry.add(&CLASS)
}

/// Bind the class into the machine graph.
///
/// # Errors
///
/// If the name is already bound.
pub fn bind(bindings: &mut crate::machine::Bindings) -> Result<()> {
    bindings.bind(CLASS_NAME, |props| Ok(Arc::new(Scac::new(props)?)))
}

/// The validator schema.
#[must_use]
pub fn schema() -> crate::machine::validate::ClassSchema {
    crate::machine::validate::ClassSchema::new(CLASS_NAME)
        .region("")
        .region("regs")
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::state::{MachineShape, Migrations, StateReader, StateWriter};

    fn w(c: &Scac, off: u64, v: u16) {
        Registers(Arc::clone(&c.state))
            .write(off, &v.to_be_bytes(), MemAttrs::DEFAULT)
            .unwrap();
    }

    fn r(c: &Scac, off: u64) -> u16 {
        let mut b = [0u8; 2];
        Registers(Arc::clone(&c.state))
            .read(off, &mut b, MemAttrs::DEFAULT)
            .unwrap();
        u16::from_be_bytes(b)
    }

    #[test]
    fn it_is_present_from_power_on() {
        let c = Scac::default();
        let s = r(&c, STATUS);
        assert_eq!(s, 0x0001, "what the driver accepts as present");
        assert!(s != 0 && s != 0xffff, "never a value the driver spins on");
        assert_eq!(s & 0x009f, 0x0001, "the library's idle check");
        assert_eq!(s & 0x8000, 0, "unprovisioned");
    }

    #[test]
    fn the_registers_are_big_endian() {
        let c = Scac::default();
        let mut b = [0u8; 2];
        Registers(Arc::clone(&c.state))
            .read(STATUS, &mut b, MemAttrs::DEFAULT)
            .unwrap();
        assert_eq!(b, [0x00, 0x01], "high byte at the lower address");
    }

    #[test]
    fn initialize_completes_and_the_empty_response_returns_it_to_idle() {
        let c = Scac::default();
        w(&c, STATUS, CMD_INITIALIZE);
        let s = r(&c, STATUS);
        assert_eq!(s & 0x0006, 0x0004, "done, not busy");
        assert_eq!(s & 0x1f00, 0, "no error");
        assert_eq!(r(&c, STATUS), s, "reading STATUS changes nothing");
        assert_eq!(r(&c, RDATALEN) & 0x7ff, 0);
        assert_eq!(r(&c, STATUS), 0x0001, "idle again");
    }

    #[test]
    fn verification_fails_and_other_commands_are_refused() {
        let c = Scac::default();
        w(&c, RDATA, 0x0000);
        w(&c, RDATA, 0x0000);
        w(&c, STATUS, CMD_INTEGRITY_FINAL_ID);
        assert_eq!(r(&c, STATUS) >> 8 & 0x1f, 10, "verification failed");
        assert_eq!(r(&c, RDATALEN), 0);
        assert_eq!(r(&c, RDATA), 0, "a padded read past the end is harmless");
        assert_eq!(r(&c, STATUS), 0x0001);
        w(&c, STATUS, 0x5f66);
        assert_eq!(r(&c, STATUS) >> 8 & 0x1f, 1, "no keys, no random numbers");
    }

    #[test]
    fn a_debug_read_takes_nothing() {
        let c = Scac::default();
        w(&c, STATUS, CMD_PING);
        let mut b = [0u8; 2];
        Registers(Arc::clone(&c.state))
            .read(RDATALEN, &mut b, MemAttrs::DEBUG)
            .unwrap();
        assert_ne!(r(&c, STATUS) & RESPONSE, 0, "still waiting to be read");
    }

    #[test]
    fn a_snapshot_round_trips() {
        let c = Scac::default();
        w(&c, RDATA, 0x1234);
        w(&c, STATUS, CMD_PING);
        w(&c, RDATA, 0xbeef);
        let save = |c: &Scac| {
            let mut shape = MachineShape::new();
            shape.add_device("scac", CLASS.name).unwrap();
            let mut wr = StateWriter::new(shape);
            {
                let mut chunk = wr.chunk("scac", CLASS.name, CLASS.version).unwrap();
                c.save(&mut chunk).unwrap();
            }
            wr.to_vec().unwrap()
        };
        let bytes = save(&c);
        let back = Scac::default();
        let reader = StateReader::new(&bytes).unwrap();
        let chunk = reader
            .load("scac", CLASS.name, CLASS.version, &Migrations::new())
            .unwrap();
        back.load(&mut chunk.reader()).unwrap();
        let restored = back.state.lock().clone();
        assert_eq!(restored, *c.state.lock());
        assert_eq!(save(&back), bytes);
    }
}
