//! The R-Car system controller's power-domain block (SYSC): the switches that
//! power the GPU, the video and audio engines and the secondary cores up and
//! down, and the status a driver polls to know a switch has finished.
//!
//! # Register map
//!
//! All accesses are 32-bit.
//!
//! | Offset | Name | Here |
//! | --- | --- | --- |
//! | `0x00` | `SYSCSR` | ready to accept a request: reads `0b11` (bit 0 power-off, bit 1 power-on), always |
//! | `0x04` | `SYSCISR` | one completion bit per domain, set when its request finishes |
//! | `0x08` | `SYSCISCR` | write 1 to clear the matching `SYSCISR` bit |
//! | `0x0c`, `0x10` | `SYSCIER`, `SYSCIMR` | interrupt enable and mask: stored; the driver masks its domain and polls |
//! | `0x40 + 0x40·n` | `PWRSR` | domain *n*'s power status (+`0x00`): a request bit *b* reads at *b* once switched off, at *b* + 4 once switched on |
//! | | `PWROFFCR` | a write switches the written bits off (+`0x04`) |
//! | | `PWRONCR` | a write switches them on (+`0x0c`) |
//! | | `PWRER` | the request's error status (+`0x14`): reads 0, every request succeeds |
//!
//! Anything else in the window is plain storage, which is what it was before
//! this block was modelled.
//!
//! A request finishes the moment it is written: the status moves and the
//! completion bit is up before the driver's first poll. That is the shape the
//! driver waits for — it polls `SYSCSR`, writes the request, checks `PWRER`,
//! and then waits for `SYSCISR` — and a real switch's few microseconds are
//! inside its first `udelay`.
//!
//! The recovery kernel's framebuffer driver powers the SGX on and then waits
//! for its `PWRSR` to read exactly `0x10`: bit 0 written to `PWRONCR` comes
//! back as bit 4. The off side reading at the written bit is the mirror of
//! that, and an inference.
//!
//! # Which bit each domain completes on
//!
//! | Domain | Offset | `SYSCISR` |
//! | --- | --- | --- |
//! | 0 | `0x40` | bits 1–3: the written bit itself (the secondary cores: bit *n* for core *n*) |
//! | 1 | `0x80` | bit 16 |
//! | 2 | `0xc0` | bit 20 (the SGX) |
//! | 3 | `0x100` | bit 21 |
//! | 4 | `0x140` | bit 24 |
//!
//! # Sources
//!
//! No manual: the navi's SD-card kernel carries a table of its domains —
//! completion bit, `PWRONCR` address, `PWRER` address, one column each — that
//! its power-on routine indexes, and its SGX power routine performs the
//! handshake above inline. Both were read off the image, disassembled as data.
//! The table is where the domain-to-bit assignment comes from.

use alloc::boxed::Box;
use alloc::string::String;
use alloc::sync::Arc;
use alloc::vec::Vec;

use crate::core::device::{Device, DeviceClass, RealizeCtx, ResetKind};
use crate::core::error::{BusError, Error, Result};
use crate::core::props::Props;
use crate::core::space::{AccessConstraints, MemAttrs, MemOps, MemResult, Region, RegionRef};
use crate::core::state::{ChunkReader, ChunkWriter, Sink, Source};
use crate::core::sync::{LockRank, Mutex};
use crate::core::value::{Endian, Width};
use crate::machine::realize::Instance;

/// The class name a machine description writes.
pub const CLASS_NAME: &str = "rcar.sysc";

/// The snapshot chunk version. Bump with the encoding, never on its own.
const STATE_VERSION: u32 = 1;

/// How much address space the block answers.
pub const REGISTER_WINDOW_LEN: u64 = 0x1000;

const WORDS: usize = (REGISTER_WINDOW_LEN / 4) as usize;

const SYSCSR: u64 = 0x00;
const SYSCISR: u64 = 0x04;
const SYSCISCR: u64 = 0x08;

/// Both request kinds accepted.
const SYSCSR_READY: u32 = 0b11;

/// The first domain's registers, and the stride between domains.
const DOMAIN_BASE: u64 = 0x40;
const DOMAIN_STRIDE: u64 = 0x40;
const DOMAINS: u64 = 5;

const PWRSR: u64 = 0x00;
const PWROFFCR: u64 = 0x04;
const PWRONCR: u64 = 0x0c;
const PWRER: u64 = 0x14;

/// The `SYSCISR` bits a request on domain `n` for bits `v` completes on.
fn completion(n: u64, v: u32) -> u32 {
    match n {
        0 => v & 0b1110,
        1 => 1 << 16,
        2 => 1 << 20,
        3 => 1 << 21,
        _ => 1 << 24,
    }
}

/// The guest-visible state: every word of the window, the computed ones
/// included, so a snapshot is the window and nothing else.
#[derive(Debug, Clone, PartialEq, Eq)]
struct State {
    words: Vec<u32>,
}

impl Default for State {
    fn default() -> State {
        State {
            words: alloc::vec![0; WORDS],
        }
    }
}

impl State {
    fn word(&mut self, offset: u64) -> &mut u32 {
        &mut self.words[(offset / 4) as usize]
    }

    fn read(&mut self, offset: u64) -> u32 {
        if offset == SYSCSR {
            return SYSCSR_READY;
        }
        if let Some((_, PWRER)) = domain(offset) {
            return 0;
        }
        *self.word(offset)
    }

    fn write(&mut self, offset: u64, v: u32) {
        match (offset, domain(offset)) {
            (SYSCSR | SYSCISR, _) => {}
            (SYSCISCR, _) => *self.word(SYSCISR) &= !v,
            (_, Some((n, reg @ (PWRONCR | PWROFFCR)))) => {
                let status = DOMAIN_BASE + n * DOMAIN_STRIDE + PWRSR;
                let (on, off) = (v << 4, v & 0xf);
                if reg == PWRONCR {
                    *self.word(status) = (*self.word(status) & !off) | on;
                } else {
                    *self.word(status) = (*self.word(status) & !on) | off;
                }
                *self.word(SYSCISR) |= completion(n, v);
            }
            (_, Some((_, PWRSR | PWRER))) => {}
            _ => *self.word(offset) = v,
        }
    }
}

/// Which domain's register `offset` is, and which register.
fn domain(offset: u64) -> Option<(u64, u64)> {
    let rel = offset.checked_sub(DOMAIN_BASE)?;
    let n = rel / DOMAIN_STRIDE;
    (n < DOMAINS).then_some((n, rel % DOMAIN_STRIDE))
}

#[derive(Debug)]
struct Registers {
    state: Mutex<State>,
}

impl MemOps for Registers {
    fn read(&self, offset: u64, dst: &mut [u8], _attrs: MemAttrs) -> MemResult {
        if dst.len() != 4 {
            return Err(BusError::BadAccess);
        }
        // Reading has no side effect, so a debugger read is an ordinary one.
        let v = self.state.lock().read(offset & !3);
        dst.copy_from_slice(&v.to_le_bytes());
        Ok(())
    }

    fn write(&self, offset: u64, src: &[u8], attrs: MemAttrs) -> MemResult {
        if src.len() != 4 || attrs.debug {
            return Err(BusError::BadAccess);
        }
        let v = u32::from_le_bytes([src[0], src[1], src[2], src[3]]);
        self.state.lock().write(offset & !3, v);
        Ok(())
    }

    fn constraints(&self) -> AccessConstraints {
        AccessConstraints::word(Width::U32, Endian::Little)
    }
}

/// The SYSC power-domain block.
#[derive(Debug)]
pub struct Sysc {
    regs: Arc<Registers>,
    region: RegionRef,
}

impl Sysc {
    /// Build it. It takes no properties.
    ///
    /// # Errors
    ///
    /// [`Error::Property`] if any property is given.
    pub fn new(props: &Props) -> Result<Sysc> {
        props.reader().finish()?;
        Ok(Sysc::default())
    }

    /// Whether domain `n` reports request bit `bit` powered.
    #[must_use]
    pub fn powered(&self, n: u64, bit: u32) -> bool {
        let mut s = self.regs.state.lock();
        *s.word(DOMAIN_BASE + n * DOMAIN_STRIDE + PWRSR) & (1 << (bit + 4)) != 0
    }
}

impl Default for Sysc {
    fn default() -> Sysc {
        let regs = Arc::new(Registers {
            state: Mutex::with_rank(LockRank::DEVICE, State::default()),
        });
        let region: RegionRef = Arc::new(Region::io(
            "rcar.sysc",
            REGISTER_WINDOW_LEN,
            Arc::clone(&regs) as Arc<dyn MemOps>,
        ));
        Sysc { regs, region }
    }
}

/// The class descriptor.
pub static CLASS: DeviceClass = DeviceClass {
    name: CLASS_NAME,
    version: STATE_VERSION,
    summary: "the R-Car SYSC power domains: power switches that complete at once, with their status and completion bits",
    properties: &[],
    construct: |props| Ok(Box::new(Sysc::new(props)?)),
};

impl Device for Sysc {
    fn class(&self) -> &'static DeviceClass {
        &CLASS
    }

    fn realize(&self, _ctx: &mut RealizeCtx<'_>) -> Result<()> {
        Ok(())
    }

    fn reset(&self, _kind: ResetKind) {
        *self.regs.state.lock() = State::default();
    }

    fn region(&self, name: &str) -> Option<RegionRef> {
        matches!(name, "" | "regs").then(|| Arc::clone(&self.region))
    }

    fn save(&self, w: &mut ChunkWriter<'_>) -> Result<()> {
        let s = self.regs.state.lock().clone();
        for v in s.words {
            w.write_u32(v)?;
        }
        Ok(())
    }

    fn load(&self, r: &mut ChunkReader<'_>) -> Result<()> {
        let mut words = Vec::with_capacity(WORDS);
        for _ in 0..WORDS {
            words.push(r.read_u32()?);
        }
        if words[(SYSCSR / 4) as usize] != 0 {
            return Err(Error::State(String::from(
                "a SYSC snapshot stores no SYSCSR: it is computed",
            )));
        }
        *self.regs.state.lock() = State { words };
        Ok(())
    }
}

impl Instance for Sysc {}

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
    bindings.bind(CLASS_NAME, |props| Ok(Arc::new(Sysc::new(props)?)))
}

/// The validator schema.
#[must_use]
pub fn schema() -> crate::machine::validate::ClassSchema {
    use crate::machine::validate::ClassSchema;
    ClassSchema::new(CLASS_NAME).region("").region("regs")
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::state::{MachineShape, Migrations, StateReader, StateWriter};

    fn w(s: &Sysc, off: u64, v: u32) {
        s.regs
            .write(off, &v.to_le_bytes(), MemAttrs::DEFAULT)
            .unwrap();
    }

    fn r(s: &Sysc, off: u64) -> u32 {
        let mut b = [0u8; 4];
        s.regs.read(off, &mut b, MemAttrs::DEFAULT).unwrap();
        u32::from_le_bytes(b)
    }

    #[test]
    fn the_sgx_handshake_completes() {
        // The driver's sequence: mask its bit, wait for ready, request,
        // check the error register, wait for completion, clear it.
        let s = Sysc::default();
        w(&s, 0x0c, 1 << 20);
        w(&s, 0x10, r(&s, 0x10) | 1 << 20);
        assert_eq!(r(&s, SYSCSR) & 0b10, 0b10, "ready for a power-on");
        w(&s, 0xcc, 1);
        assert_eq!(r(&s, 0xd4) & 1, 0, "no error");
        assert_ne!(r(&s, SYSCISR) & 1 << 20, 0, "complete");
        assert!(s.powered(2, 0));
        assert_eq!(r(&s, 0xc0), 0x10, "what the framebuffer driver waits for");
        w(&s, SYSCISCR, 1 << 20);
        assert_eq!(r(&s, SYSCISR), 0, "cleared");
        assert_eq!(r(&s, 0x10), 1 << 20, "the mask is plain storage");
    }

    #[test]
    fn each_domain_completes_on_its_own_bit() {
        let s = Sysc::default();
        for (n, want) in [
            (0u64, 1u32 << 1),
            (1, 1 << 16),
            (2, 1 << 20),
            (3, 1 << 21),
            (4, 1 << 24),
        ] {
            w(&s, SYSCISCR, !0);
            w(
                &s,
                DOMAIN_BASE + n * DOMAIN_STRIDE + PWRONCR,
                if n == 0 { 2 } else { 1 },
            );
            assert_eq!(r(&s, SYSCISR), want, "domain {n}");
        }
    }

    #[test]
    fn power_off_clears_the_status() {
        let s = Sysc::default();
        w(&s, 0x10c, 1);
        assert!(s.powered(3, 0));
        w(&s, 0x104, 1);
        assert!(!s.powered(3, 0));
        assert_eq!(r(&s, 0x100), 0x01);
        assert_ne!(r(&s, SYSCISR) & 1 << 21, 0, "an off request completes too");
    }

    #[test]
    fn the_rest_of_the_window_is_storage() {
        let s = Sysc::default();
        w(&s, 0x800, 0x1234_5678);
        assert_eq!(r(&s, 0x800), 0x1234_5678);
        w(&s, 0xc0, 0xffff);
        assert_eq!(r(&s, 0xc0), 0, "status is not writable");
    }

    #[test]
    fn a_snapshot_round_trips() {
        let s = Sysc::default();
        w(&s, 0xcc, 1);
        w(&s, 0x0c, 0x55);
        let save = |s: &Sysc| {
            let mut shape = MachineShape::new();
            shape.add_device("sysc", CLASS.name).unwrap();
            let mut wr = StateWriter::new(shape);
            {
                let mut chunk = wr.chunk("sysc", CLASS.name, CLASS.version).unwrap();
                s.save(&mut chunk).unwrap();
            }
            wr.to_vec().unwrap()
        };
        let bytes = save(&s);
        let back = Sysc::default();
        let reader = StateReader::new(&bytes).unwrap();
        let chunk = reader
            .load("sysc", CLASS.name, CLASS.version, &Migrations::new())
            .unwrap();
        back.load(&mut chunk.reader()).unwrap();
        let want = s.regs.state.lock().clone();
        assert_eq!(*back.regs.state.lock(), want);
        assert_eq!(save(&back), bytes);
    }
}
