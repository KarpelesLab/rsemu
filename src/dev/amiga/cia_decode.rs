//! The Amiga's CIA address decode: sixteen registers, 256 bytes apart, on one
//! byte of the data bus.
//!
//! One class, `amiga.cia-decode`. It is the board's half of an 8520: the part
//! that turns a 68000 address into a register select and decides which half of
//! a word the chip is wired to. The chip itself (`mos.8520`) is decode-free and
//! publishes its sixteen registers back to back; this is what spreads them out.
//!
//! # What the manual says
//!
//! The *Amiga Hardware Reference Manual*'s 8520 appendix gives CIA-A's address
//! as `$BFEr01` and CIA-B's as `$BFDr00`, where `r` is the register number,
//! `0`…`F`. Two facts are packed into that notation:
//!
//! * **The register select is A8–A11.** Register *n* is 256 bytes after
//!   register *n − 1*, so each chip's sixteen registers fill a 4 KiB window.
//! * **Each chip is on one byte lane.** CIA-A's addresses are odd and CIA-B's
//!   are even. On a big-endian 68000 an even address is the high byte of a word
//!   (D8–D15) and an odd one the low byte (D0–D7), so CIA-B is wired to the
//!   upper half of the data bus and CIA-A to the lower.
//!
//! And the appendix says which address lines pick the chip: "CIAA is selected
//! when A12 is low, A13 high; CIAB is selected when A12 is high, A13 low."
//! That is `$BFE000` and `$BFD000` respectively, and it means **only one CIA is
//! ever selected by one access** — a word read at `$BFE000` gets CIA-A's
//! register 0 in its low byte and nothing at all in its high byte, not CIA-B.
//!
//! # The shape
//!
//! ```text
//!   object cia_a "mos.8520" { clock = cpu / 10 }
//!   object cia_b "mos.8520" { clock = cpu / 10 }
//!
//!   object cia_a_decode "amiga.cia-decode" { chip = cia_a, lane = "odd"  }
//!   object cia_b_decode "amiga.cia-decode" { chip = cia_b, lane = "even" }
//!
//!   map mem 0xBFE000 size 0x1000 = cia_a_decode { endian = "big" }
//!   map mem 0xBFD000 size 0x1000 = cia_b_decode { endian = "big" }
//! ```
//!
//! One decoder per chip, because one access selects one chip. Note the map
//! bases: `$BFE000`, not `$BFE001`. The window starts on the even address so
//! that the window-relative parity *is* the address parity, which is what the
//! lane is decided by.
//!
//! # Per byte, not per access
//!
//! Every byte of an access is decoded on its own:
//!
//! * a byte on this decoder's lane is forwarded to register
//!   `(address >> 8) & 0xF` of the chip, as a one-byte access;
//! * a byte on the other lane reaches no chip. A read of it answers with
//!   [`MemAttrs::bus`] — the byte the master last drove, which is the
//!   framework's open-bus model and what an undriven half of a 68000 data bus
//!   reads back as — and a write of it is dropped.
//!
//! So a byte access at a canonical address reaches its register exactly, and a
//! word access reaches it too, with the other half floating. Nothing is
//! invented for the word case; it falls out of decoding the two bytes.
//!
//! **A0–A7 other than the lane are not decoded.** `$BFE003` is register 0 as
//! much as `$BFE001` is: the appendix gives the register select as the one hex
//! digit, and the part has four register-select pins, so there is nothing that
//! could look at the low byte. That is the reading of the notation rather than a
//! sentence the manual prints, and it is the one decision here a later agent
//! with a schematic might revisit.
//!
//! # Retry and side effects
//!
//! A word access reaches the chip once. A longword reaches the same register
//! twice — a 68000 runs it as two word cycles, and the part is selected on each
//! — so a read-to-clear register sees two reads, which is also what the silicon
//! sees. A `Retry` part-way through, after the first forwarded byte had its side
//! effect, cannot arise: the target is in a private space this object built at
//! bind and nothing ever retopologises it.
//!
//! # Sources
//!
//! *Amiga Hardware Reference Manual*, Commodore-Amiga Inc., 3rd edition: the
//! 8520 appendix for the addresses, the register-select notation and the
//! A12/A13 chip selects; Appendix D ("System Memory Maps") for the windows. No
//! emulator source of any licence was consulted (`ROADMAP.md` §1).

use alloc::boxed::Box;
use alloc::format;
use alloc::string::{String, ToString};
use alloc::sync::Arc;

use crate::core::device::{Device, DeviceClass, PropertySpec, RealizeCtx, ResetKind};
use crate::core::error::{BusError, Error, Result};
use crate::core::props::{Props, ValueKind};
use crate::core::space::{
    AccessConstraints, AddressSpace, MemAttrs, MemOps, MemResult, Region, RegionRef,
};
use crate::core::sync::{LockRank, Mutex};
use crate::machine::realize::{BindCtx, Instance};
use crate::machine::validate::{ClassSchema, PropSchema};

/// The class name a machine file writes.
pub const CLASS_NAME: &str = "amiga.cia-decode";

/// How many registers an 8520 has: four register-select pins.
pub const REGISTERS: u64 = 16;

/// How far apart they are in the 68000's map: the select is A8–A11.
pub const STRIDE: u64 = 0x100;

/// The window one chip's registers fill.
pub const WINDOW: u64 = REGISTERS * STRIDE;

/// Which half of the 68000's data bus a chip is wired to.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Lane {
    /// D8–D15, the high byte of a word, reached at even addresses. CIA-B.
    Even,
    /// D0–D7, the low byte of a word, reached at odd addresses. CIA-A.
    Odd,
}

impl Lane {
    /// The machine-file spelling.
    #[must_use]
    pub const fn name(self) -> &'static str {
        match self {
            Lane::Even => "even",
            Lane::Odd => "odd",
        }
    }

    /// Whether the byte at window offset `offset` is on this lane.
    #[must_use]
    #[inline]
    pub const fn carries(self, offset: u64) -> bool {
        match self {
            Lane::Even => offset & 1 == 0,
            Lane::Odd => offset & 1 == 1,
        }
    }
}

/// The register a window offset selects.
#[must_use]
#[inline]
pub const fn select(offset: u64) -> u64 {
    (offset / STRIDE) % REGISTERS
}

// ---------------------------------------------------------------------------
// the decoder
// ---------------------------------------------------------------------------

#[derive(Debug)]
struct Decoder {
    lane: Lane,
    /// The chip's register block, alone in a space of this object's own, at
    /// zero. `None` until bind.
    ///
    /// Cloned out and the lock released before the forwarded access, which
    /// takes a `TOPOLOGY` guard; this one is a leaf.
    target: Mutex<Option<Arc<AddressSpace>>>,
}

impl Decoder {
    fn target(&self) -> Option<Arc<AddressSpace>> {
        self.target.lock().clone()
    }
}

impl MemOps for Decoder {
    fn read(&self, offset: u64, dst: &mut [u8], attrs: MemAttrs) -> MemResult {
        let target = self.target().ok_or(BusError::Unassigned)?;
        for (i, byte) in dst.iter_mut().enumerate() {
            let at = offset.wrapping_add(i as u64);
            if self.lane.carries(at) {
                // `attrs` unchanged, `debug` included: the chip decides what a
                // debugger's read of its interrupt control register does.
                target.read_bytes(select(at), core::slice::from_mut(byte), attrs)?;
            } else {
                // Nothing drives this half of the bus.
                *byte = attrs.bus;
            }
        }
        Ok(())
    }

    fn write(&self, offset: u64, src: &[u8], attrs: MemAttrs) -> MemResult {
        let target = self.target().ok_or(BusError::Unassigned)?;
        for (i, byte) in src.iter().enumerate() {
            let at = offset.wrapping_add(i as u64);
            if self.lane.carries(at) {
                target.write_bytes(select(at), core::slice::from_ref(byte), attrs)?;
            }
        }
        Ok(())
    }

    fn constraints(&self) -> AccessConstraints {
        // Any single width, no bursts. The byte order is the board's to declare
        // on the `map` statement, exactly as for the overlay: the lane is
        // decided by address, and the dispatcher assembles the word.
        AccessConstraints::IO
    }
}

// ---------------------------------------------------------------------------
// the device
// ---------------------------------------------------------------------------

/// One 8520's address decode.
#[derive(Debug)]
pub struct CiaDecode {
    decoder: Arc<Decoder>,
    region: RegionRef,
    chip: String,
}

impl CiaDecode {
    /// Build the decoder.
    ///
    /// # Errors
    ///
    /// [`Error::Property`] if `chip` or `lane` is missing, if `lane` is not
    /// `"odd"` or `"even"`, or if a property nothing here accepts was given.
    pub fn new(props: &Props) -> Result<CiaDecode> {
        let mut r = props.reader();
        let chip = r.require_link("chip")?.as_str().to_string();
        let lane = match r.require_enum("lane", &["odd", "even"])? {
            "odd" => Lane::Odd,
            _ => Lane::Even,
        };
        r.finish()?;
        let decoder = Arc::new(Decoder {
            lane,
            target: Mutex::with_rank(LockRank::LEAF, None),
        });
        let region = Arc::new(Region::io(
            "amiga.cia-decode",
            WINDOW,
            Arc::clone(&decoder) as Arc<dyn MemOps>,
        ));
        Ok(CiaDecode {
            decoder,
            region,
            chip,
        })
    }

    /// The lane this decoder answers on.
    #[must_use]
    pub fn lane(&self) -> Lane {
        self.decoder.lane
    }

    /// Give the decoder the chip's register block.
    ///
    /// Public so a test or an embedder that builds a machine by hand can do
    /// what `bind` does from a machine file.
    ///
    /// # Errors
    ///
    /// If the block is shorter than sixteen registers, which would leave a
    /// register select decoding nothing.
    pub fn attach(&self, registers: &RegionRef) -> Result<()> {
        if registers.len() < REGISTERS {
            return Err(Error::Config {
                at: String::from(CLASS_NAME),
                message: format!(
                    "an 8520 has {REGISTERS} registers and `{}` publishes only {}",
                    self.chip,
                    registers.len()
                ),
            });
        }
        let space = AddressSpace::new(format!("{CLASS_NAME}.{}", self.chip), 8);
        {
            let mut topo = space.topology();
            topo.map(Arc::clone(registers), 0)?;
        }
        *self.decoder.target.lock() = Some(Arc::new(space));
        Ok(())
    }
}

impl Device for CiaDecode {
    fn class(&self) -> &'static DeviceClass {
        &CLASS
    }

    fn realize(&self, _ctx: &mut RealizeCtx<'_>) -> Result<()> {
        Ok(())
    }

    fn reset(&self, _kind: ResetKind) {
        // Combinational decode: nothing to reset. The chip resets itself.
    }

    // No `save`/`load`: there is no state here. The lane is a property and the
    // target is wiring, re-established at bind.

    fn region(&self, name: &str) -> Option<RegionRef> {
        match name {
            "" | "window" => Some(Arc::clone(&self.region)),
            _ => None,
        }
    }
}

impl Instance for CiaDecode {
    fn bind(&self, ctx: &BindCtx<'_>) -> Result<()> {
        let registers = ctx.region(&self.chip, "")?;
        self.attach(&registers).map_err(|e| Error::Config {
            at: ctx.path().to_string(),
            message: e.to_string(),
        })
    }
}

/// The `amiga.cia-decode` device class.
pub static CLASS: DeviceClass = DeviceClass {
    name: CLASS_NAME,
    version: 1,
    summary: "the Amiga's decode for one 8520: register select on A8-A11, one data-bus lane",
    properties: &[
        PropertySpec {
            name: "chip",
            kind: ValueKind::Link,
            required: true,
            summary: "the 8520 whose sixteen back-to-back registers this spreads 256 bytes apart",
        },
        PropertySpec {
            name: "lane",
            kind: ValueKind::Str,
            required: true,
            summary: "`odd` for a chip on D0-D7 (CIA-A), `even` for one on D8-D15 (CIA-B)",
        },
    ],
    construct: |props| Ok(Box::new(CiaDecode::new(props)?)),
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
    bindings.bind(CLASS_NAME, |props| Ok(Arc::new(CiaDecode::new(props)?)))
}

/// What the validator should know about `amiga.cia-decode`.
#[must_use]
pub fn schema() -> ClassSchema {
    ClassSchema::new(CLASS_NAME)
        .prop(PropSchema::new("chip", ValueKind::Link))
        .prop(PropSchema::new("lane", ValueKind::Str).values(&["odd", "even"]))
        .region("")
        .region("window")
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::props::Value;
    use crate::core::space::RamStore;
    use crate::core::value::{Endian, Width};

    /// A sixteen-byte RAM standing in for an 8520's register block. **A
    /// placeholder**: it has no timers and no interrupt control register, and
    /// all it proves is where each byte lands, which is all this file decides.
    fn stand_in() -> (Arc<RamStore>, RegionRef) {
        let store = Arc::new(RamStore::new(REGISTERS));
        let region: RegionRef = Arc::new(Region::ram("cia", Arc::clone(&store)));
        (store, region)
    }

    fn props(lane: &str) -> Props {
        Props::new()
            .with(
                "chip",
                Value::Link(crate::core::props::Link::new("cia").unwrap()),
            )
            .with("lane", Value::from(lane))
    }

    /// A 24-bit big-endian space with one decoder mapped at `base`.
    fn board(lane: &str, base: u64) -> (AddressSpace, Arc<RamStore>) {
        let decode = CiaDecode::new(&props(lane)).unwrap();
        let (store, region) = stand_in();
        decode.attach(&region).unwrap();
        let space = AddressSpace::new("mem", 24).with_endian(Endian::Big);
        {
            let window = decode.region("").unwrap();
            let window: RegionRef = Arc::new(
                Region::alias("win", window, 0, WINDOW)
                    .unwrap()
                    .with_endian(Endian::Big),
            );
            let mut topo = space.topology();
            topo.map(window, base).unwrap();
        }
        (space, store)
    }

    fn byte(store: &RamStore, n: u64) -> u8 {
        let mut b = [0u8; 1];
        store.read_at(n, &mut b).unwrap();
        b[0]
    }

    #[test]
    fn cia_a_register_n_is_at_bfe001_plus_n_hundred() {
        let (space, store) = board("odd", 0xBFE000);
        for n in 0..REGISTERS {
            space
                .write(
                    0xBFE001 + n * STRIDE,
                    Width::U8,
                    0x40 + n,
                    MemAttrs::DEFAULT,
                )
                .unwrap();
        }
        for n in 0..REGISTERS {
            assert_eq!(byte(&store, n), 0x40 + n as u8, "register {n}");
            assert_eq!(
                space
                    .read(0xBFE001 + n * STRIDE, Width::U8, MemAttrs::DEFAULT)
                    .unwrap(),
                0x40 + n
            );
        }
    }

    #[test]
    fn cia_b_register_n_is_at_bfd000_plus_n_hundred() {
        let (space, store) = board("even", 0xBFD000);
        space
            .write(0xBFD100, Width::U8, 0x5a, MemAttrs::DEFAULT)
            .unwrap();
        assert_eq!(byte(&store, 1), 0x5a);
        // The odd byte beside it is the other lane: nothing lands.
        space
            .write(0xBFD101, Width::U8, 0xff, MemAttrs::DEFAULT)
            .unwrap();
        assert_eq!(byte(&store, 1), 0x5a);
        assert_eq!(byte(&store, 0), 0);
    }

    #[test]
    fn a_word_reaches_the_chip_on_its_own_half_and_floats_on_the_other() {
        let (space, store) = board("odd", 0xBFE000);
        store.write_at(0, &[0x81]).unwrap();
        let attrs = MemAttrs {
            bus: 0x3c,
            ..MemAttrs::DEFAULT
        };
        // Big-endian: $BFE000 is the high byte and on CIA-B's lane, which this
        // decoder does not carry; $BFE001 is the low byte and CIA-A's register 0.
        assert_eq!(space.read(0xBFE000, Width::U16, attrs).unwrap(), 0x3c81);
        // A word write stores only the low byte.
        space
            .write(0xBFE000, Width::U16, 0xAA55, MemAttrs::DEFAULT)
            .unwrap();
        assert_eq!(byte(&store, 0), 0x55);
    }

    #[test]
    fn only_a8_to_a11_and_the_lane_are_decoded() {
        let (space, store) = board("odd", 0xBFE000);
        store.write_at(2, &[0x77]).unwrap();
        for addr in [0xBFE201u64, 0xBFE203, 0xBFE2FF] {
            assert_eq!(
                space.read(addr, Width::U8, MemAttrs::DEFAULT).unwrap(),
                0x77,
                "{addr:#x}"
            );
        }
    }

    #[test]
    fn a_block_shorter_than_sixteen_registers_is_refused() {
        let decode = CiaDecode::new(&props("odd")).unwrap();
        let short: RegionRef = Arc::new(Region::ram("short", Arc::new(RamStore::new(15))));
        assert!(decode.attach(&short).is_err());
    }

    #[test]
    fn an_unbound_decoder_faults_rather_than_inventing_a_chip() {
        let decode = CiaDecode::new(&props("odd")).unwrap();
        let ops = Arc::clone(&decode.decoder);
        let mut b = [0u8; 1];
        assert!(ops.read(1, &mut b, MemAttrs::DEFAULT).is_err());
    }

    #[test]
    fn the_properties_are_checked() {
        assert_eq!(CiaDecode::new(&props("odd")).unwrap().lane(), Lane::Odd);
        assert_eq!(CiaDecode::new(&props("even")).unwrap().lane(), Lane::Even);
        assert!(CiaDecode::new(&props("both")).is_err());
        assert!(CiaDecode::new(&Props::new().with("lane", Value::from("odd"))).is_err());
        assert!(CiaDecode::new(&props("odd").with("stride", Value::from(1u64))).is_err());
    }

    #[test]
    fn the_register_select_and_the_lane_are_what_the_notation_says() {
        assert_eq!(select(0x001), 0);
        assert_eq!(select(0x0f01), 15);
        assert_eq!(select(0x0100), 1);
        assert!(Lane::Odd.carries(0xe01));
        assert!(!Lane::Odd.carries(0xe00));
        assert!(Lane::Even.carries(0xd00));
        assert_eq!(WINDOW, 0x1000);
    }
}
