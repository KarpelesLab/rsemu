//! The STM32 CRC calculation unit.
//!
//! One class, `st.crc`, in the two shapes the family has. Every STM32 has one
//! of these at `0x4002_3000` and firmware uses it constantly — image
//! checksums, settings-block integrity, seeding a hash — so a part without it
//! is a bus fault in the middle of a boot rather than a missing feature
//! somebody notices later.
//!
//! # The two blocks
//!
//! `variant = "f4"` is the original (F1/F2/F4, RM0090 §4.4): a **fixed**
//! CRC-32 with the polynomial `0x04C11DB7`, an initial value of `0xFFFFFFFF`,
//! no bit reversal anywhere, and a `DR` that takes 32-bit writes and nothing
//! else.
//!
//! | Offset | Register | `"f4"` | `"v2"` |
//! | --- | --- | --- | --- |
//! | `0x00` | `DR` | word writes only | 8-, 16- or 32-bit writes |
//! | `0x04` | `IDR` | an eight-bit scratch byte the unit never touches | same |
//! | `0x08` | `CR` | `RESET` at 0 | `RESET`, `POLYSIZE[4:3]`, `REV_IN[6:5]`, `REV_OUT` at 7 |
//! | `0x10` | `INIT` | not decoded | the value `RESET` reloads |
//! | `0x14` | `POL` | not decoded | the polynomial |
//!
//! On the fixed block the last two are not registers that happen to be
//! read-only: the parameters are wired into the gates, so the aperture is
//! twelve bytes and `0x10` is a hole. A driver that pokes them faults rather
//! than being told a comfortable lie about a polynomial it cannot change.
//!
//! `variant = "v2"` is everything from the F0/F3/F7/L0/L4 generation onwards
//! (RM0351 §14.5), and the programmability is the whole difference: a
//! seven-, eight-, sixteen- or thirty-two-bit polynomial, a programmable
//! initial value, and bit reversal on the way in and on the way out.
//!
//! # How the accumulator moves
//!
//! Bit at a time, most significant first, which is the shift register the
//! manual describes:
//!
//! ```text
//! for each bit b of the written unit, MSB first:
//!     top = accumulator's bit W-1
//!     accumulator <<= 1                    (W bits wide)
//!     if top ^ b:  accumulator ^= POL
//! ```
//!
//! Writing it that way rather than as an exclusive-or of the input into the
//! top of the register is what makes `POLYSIZE = 7` work: a seven-bit shift
//! register fed eight-bit units has nowhere to put the input otherwise.
//!
//! `REV_IN` reverses the bits of the written unit — within each byte, each
//! half-word, or the whole word — *before* it is fed. `REV_OUT` reverses the
//! accumulator on the way out of `DR`. A reversal wider than the unit being
//! written reverses that unit, since there is nothing else there to reverse.
//! `REV_OUT` reverses **`POLYSIZE` bits**, not always thirty-two: the manual
//! describes the reversal as applying to the CRC register, and the CRC
//! register is as wide as the polynomial. At `POLYSIZE = 32`, which is the
//! only width the two readings differ on, they agree.
//!
//! Writing `CR.RESET` reloads the accumulator from `INIT` (from `0xFFFFFFFF`
//! on the F4 block, where `INIT` is not a register). It is self-clearing, so
//! `CR` never reads that bit back. Programming `POL` or `POLYSIZE` does
//! **not** reload it; firmware is expected to write `RESET` afterwards and the
//! manual says so.
//!
//! # Where the test vectors come from
//!
//! From the polynomial, not from anybody's implementation. The shift register
//! above, run by hand over the manual's polynomial, gives every constant the
//! tests below assert — a single zero word under CRC-32 is `0xC704DD7B`,
//! `"123456789"` under `0x1021` from `0xFFFF` is `0x29B1`, and so on. That
//! they happen to match the catalogue names for those parameter sets is a
//! check on the arithmetic rather than its source.
//!
//! # Sources
//!
//! ST **RM0090** rev 21 §4.4 "CRC calculation unit" for the fixed block and
//! ST **RM0351** rev 9 §14 for the programmable one. No emulator source of
//! any licence was consulted (`ROADMAP.md` §1).

use alloc::boxed::Box;
use alloc::format;
use alloc::sync::Arc;
use core::fmt;

use crate::core::device::{Device, DeviceClass, PropertySpec, RealizeCtx, ResetKind};
use crate::core::error::{BusError, Error, Result};
use crate::core::props::{Props, ValueKind};
use crate::core::space::{AccessConstraints, MemAttrs, MemOps, MemResult, Region, RegionRef};
use crate::core::state::{ChunkReader, ChunkWriter, Sink, Source};
use crate::core::sync::{LockRank, Mutex};
use crate::core::value::{Endian, Width};
use crate::machine::Instance;
use crate::machine::validate::{ClassSchema, PropSchema};

/// The class name a machine description writes.
const CLASS_NAME: &str = "st.crc";

/// The snapshot chunk version. Bump it with the encoding, never on its own.
const STATE_VERSION: u32 = 1;

/// The F4 block's registers: `DR`, `IDR`, `CR`.
const F4_BYTES: u64 = 0x0c;

/// The v2 block's: the same three plus `INIT` and `POL`.
const V2_BYTES: u64 = 0x18;

/// The polynomial the fixed block is wired with, and the v2 block's reset
/// value: the CRC-32 one (RM0090 §4.3).
pub const DEFAULT_POLYNOMIAL: u32 = 0x04c1_1db7;

/// The initial value the fixed block is wired with, and the v2 block's reset
/// value.
pub const DEFAULT_INIT: u32 = 0xffff_ffff;

/// `CR.RESET`, bit 0. Self-clearing.
const CR_RESET: u32 = 1 << 0;

/// `CR.POLYSIZE[4:3]`.
const CR_POLYSIZE_SHIFT: u32 = 3;

/// `CR.REV_IN[6:5]`.
const CR_REV_IN_SHIFT: u32 = 5;

/// `CR.REV_OUT`, bit 7.
const CR_REV_OUT: u32 = 1 << 7;

/// One bit per bit of a `width`-bit register.
const fn width_mask(width: u32) -> u32 {
    if width >= 32 {
        u32::MAX
    } else {
        (1u32 << width) - 1
    }
}

/// The low `n` bits of `value`, bit-reversed.
fn reverse(value: u32, n: u32) -> u32 {
    if n == 0 {
        return 0;
    }
    (value & width_mask(n)).reverse_bits() >> (32 - n)
}

// ---------------------------------------------------------------------------
// Variants
// ---------------------------------------------------------------------------

/// Which block this is.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Variant {
    /// The F1/F2/F4 block: a fixed CRC-32, word writes only.
    F4,
    /// The F0/F3/F7/L0/L4/G0/G4/H7/WB block: programmable everything.
    V2,
}

impl Variant {
    /// The spelling a machine file writes.
    #[must_use]
    pub const fn name(self) -> &'static str {
        match self {
            Variant::F4 => "f4",
            Variant::V2 => "v2",
        }
    }

    /// How many bytes of registers the block decodes.
    #[must_use]
    pub const fn register_bytes(self) -> u64 {
        match self {
            Variant::F4 => F4_BYTES,
            Variant::V2 => V2_BYTES,
        }
    }
}

// ---------------------------------------------------------------------------
// State
// ---------------------------------------------------------------------------

/// Everything the guest can see or change.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct State {
    /// The CRC register itself, right-aligned in `polysize` bits.
    acc: u32,
    /// `IDR`: eight bits of scratch the unit never reads.
    idr: u8,
    /// `INIT`: what `CR.RESET` reloads `acc` from.
    init: u32,
    /// `POL`: the polynomial, right-aligned in `polysize` bits.
    pol: u32,
    /// `CR.POLYSIZE`, as the manual encodes it: 0 is 32 bits, 1 is 16, 2 is 8,
    /// 3 is 7.
    polysize: u32,
    /// `CR.REV_IN`: 0 none, 1 byte, 2 half-word, 3 word.
    rev_in: u32,
    /// `CR.REV_OUT`.
    rev_out: bool,
}

impl Default for State {
    fn default() -> State {
        State {
            acc: DEFAULT_INIT,
            idr: 0,
            init: DEFAULT_INIT,
            pol: DEFAULT_POLYNOMIAL,
            polysize: 0,
            rev_in: 0,
            rev_out: false,
        }
    }
}

impl State {
    /// How wide the accumulator and the polynomial are, in bits.
    fn width(&self) -> u32 {
        match self.polysize {
            0 => 32,
            1 => 16,
            2 => 8,
            _ => 7,
        }
    }

    /// Reload the accumulator, as `CR.RESET` does.
    fn reload(&mut self) {
        self.acc = self.init & width_mask(self.width());
    }

    /// `REV_IN` applied to a `bits`-wide written unit.
    fn reverse_in(&self, data: u32, bits: u32) -> u32 {
        let chunk = match self.rev_in {
            0 => return data,
            1 => 8,
            2 => 16,
            _ => 32,
        }
        // A half-word reversal of a byte write has only the byte to reverse.
        .min(bits);
        let mut out = 0;
        let mut offset = 0;
        while offset < bits {
            let piece = (data >> offset) & width_mask(chunk);
            out |= reverse(piece, chunk) << offset;
            offset += chunk;
        }
        out
    }

    /// Feed `bits` bits of `data` through the shift register, MSB first.
    fn feed(&mut self, data: u32, bits: u32) {
        let data = self.reverse_in(data, bits);
        let width = self.width();
        let mask = width_mask(width);
        let poly = self.pol & mask;
        self.acc &= mask;
        for i in (0..bits).rev() {
            let bit = (data >> i) & 1;
            let top = (self.acc >> (width - 1)) & 1;
            self.acc = (self.acc << 1) & mask;
            if top ^ bit != 0 {
                self.acc ^= poly;
            }
        }
    }

    /// What a read of `DR` answers: the accumulator, with `REV_OUT` applied.
    fn output(&self) -> u32 {
        let width = self.width();
        let value = self.acc & width_mask(width);
        if self.rev_out {
            reverse(value, width)
        } else {
            value
        }
    }
}

// ---------------------------------------------------------------------------
// The register block
// ---------------------------------------------------------------------------

/// The register block, as something an address space can dispatch to.
struct Registers {
    state: Mutex<State>,
    variant: Variant,
}

impl fmt::Debug for Registers {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let mut s = f.debug_struct("Registers");
        s.field("variant", &self.variant);
        match self.state.try_lock() {
            Some(state) => s.field("state", &*state),
            None => s.field("state", &"<locked>"),
        };
        s.finish()
    }
}

impl Registers {
    /// Read `len` bytes at `offset`.
    fn read_register(&self, offset: u64, len: usize) -> core::result::Result<u32, BusError> {
        let state = self.state.lock();
        match offset {
            // `DR` is a whole register on the fixed block; the programmable
            // one still answers a read of it at its full width, and a narrow
            // read takes the low bytes of the answer.
            0x00 => {
                if self.variant == Variant::F4 && len != 4 {
                    return Err(BusError::BadAccess);
                }
                Ok(state.output())
            }
            // "CRC_IDR: general-purpose 8-bit data register."
            0x04 if len == 1 || len == 4 => Ok(u32::from(state.idr)),
            0x08 if len == 4 => Ok(match self.variant {
                // The fixed block's `CR` is one self-clearing bit, so it
                // always reads zero.
                Variant::F4 => 0,
                Variant::V2 => {
                    (state.polysize << CR_POLYSIZE_SHIFT)
                        | (state.rev_in << CR_REV_IN_SHIFT)
                        | if state.rev_out { CR_REV_OUT } else { 0 }
                }
            }),
            // On the fixed block these two are not registers at all — the
            // parameters are wired into the gates — so the aperture stops
            // short of them and the addresses are a hole rather than a
            // plausible-looking answer.
            0x10 if len == 4 && self.variant == Variant::V2 => Ok(state.init),
            0x14 if len == 4 && self.variant == Variant::V2 => Ok(state.pol),
            _ => Err(BusError::BadAccess),
        }
    }

    /// Write `len` bytes of `value` at `offset`.
    fn write_register(&self, offset: u64, value: u32, len: usize) -> MemResult {
        let mut state = self.state.lock();
        match offset {
            0x00 => {
                let bits = match (self.variant, len) {
                    // "Only 32-bit data can be written" on the fixed block: a
                    // byte write there is not a one-byte CRC, it is a mistake,
                    // and answering it would silently give a wrong checksum.
                    (Variant::F4, 4) | (Variant::V2, 4) => 32,
                    (Variant::V2, 2) => 16,
                    (Variant::V2, 1) => 8,
                    _ => return Err(BusError::BadAccess),
                };
                state.feed(value, bits);
            }
            0x04 if len == 1 || len == 4 => state.idr = value as u8,
            0x08 if len == 4 => {
                if self.variant == Variant::V2 {
                    state.polysize = (value >> CR_POLYSIZE_SHIFT) & 0x3;
                    state.rev_in = (value >> CR_REV_IN_SHIFT) & 0x3;
                    state.rev_out = value & CR_REV_OUT != 0;
                }
                // Last, so a single write that programs the parameters *and*
                // asks for a reset reloads under the new ones.
                if value & CR_RESET != 0 {
                    state.reload();
                }
            }
            0x10 if len == 4 && self.variant == Variant::V2 => state.init = value,
            0x14 if len == 4 && self.variant == Variant::V2 => state.pol = value,
            _ => return Err(BusError::BadAccess),
        }
        Ok(())
    }
}

impl MemOps for Registers {
    fn read(&self, offset: u64, dst: &mut [u8], _attrs: MemAttrs) -> MemResult {
        if offset >= self.variant.register_bytes() {
            return Err(BusError::BadAccess);
        }
        // Nothing here changes on a read — `DR`'s accumulator moves on writes
        // alone — so a debug read is the same read (`ROADMAP.md` §15,
        // invariant 5).
        let value = self.read_register(offset, dst.len())?;
        let bytes = value.to_le_bytes();
        for (slot, byte) in dst.iter_mut().zip(bytes) {
            *slot = byte;
        }
        Ok(())
    }

    fn write(&self, offset: u64, src: &[u8], attrs: MemAttrs) -> MemResult {
        if offset >= self.variant.register_bytes() {
            return Err(BusError::BadAccess);
        }
        if attrs.debug {
            // A debug write to `DR` would fold bytes the guest never wrote
            // into a checksum it is about to compare. There is no harmless
            // version.
            return Err(BusError::BadAccess);
        }
        let mut word = [0u8; 4];
        for (slot, byte) in word.iter_mut().zip(src) {
            *slot = *byte;
        }
        self.write_register(offset, u32::from_le_bytes(word), src.len())
    }

    fn constraints(&self) -> AccessConstraints {
        // Byte, half-word and word, naturally aligned. The *register* decides
        // which of the three it accepts — the fixed block's `DR` takes only a
        // word — because "which widths does this address answer" is a
        // per-register fact here rather than a per-block one.
        AccessConstraints::word(Width::U32, Endian::Little).with_widths(Width::U8, Width::U32)
    }
}

// ---------------------------------------------------------------------------
// The device
// ---------------------------------------------------------------------------

/// An STM32 CRC calculation unit.
#[derive(Debug)]
pub struct Crc {
    regs: Arc<Registers>,
    region: RegionRef,
}

impl Crc {
    /// Validate `props` and build the unit.
    ///
    /// # Errors
    ///
    /// [`Error::Property`] if a property is of the wrong kind or value, or if
    /// one this class does not know was given.
    pub fn new(props: &Props) -> Result<Crc> {
        let mut r = props.reader();
        let variant = match r.or_str("variant", "f4")? {
            "f4" => Variant::F4,
            "v2" => Variant::V2,
            other => {
                return Err(Error::Property(format!(
                    "`variant` is `f4` or `v2`, not `{other}`"
                )));
            }
        };
        r.finish()?;
        Ok(Crc::build(variant))
    }

    /// Build one directly — the route a test takes.
    #[must_use]
    pub fn build(variant: Variant) -> Crc {
        let regs = Arc::new(Registers {
            state: Mutex::with_rank(LockRank::DEVICE, State::default()),
            variant,
        });
        let region = Arc::new(Region::io(
            "crc",
            variant.register_bytes(),
            Arc::clone(&regs) as Arc<dyn MemOps>,
        ));
        Crc { regs, region }
    }

    /// Which block this is.
    #[must_use]
    pub fn variant(&self) -> Variant {
        self.regs.variant
    }

    /// The accumulator, with `REV_OUT` applied — what a read of `DR` answers.
    #[must_use]
    pub fn value(&self) -> u32 {
        self.regs.state.lock().output()
    }

    /// How wide the polynomial and the accumulator are, in bits.
    #[must_use]
    pub fn width(&self) -> u32 {
        self.regs.state.lock().width()
    }
}

impl Device for Crc {
    fn class(&self) -> &'static DeviceClass {
        &CLASS
    }

    fn realize(&self, _ctx: &mut RealizeCtx<'_>) -> Result<()> {
        // Nothing outward: a `map` statement places the region, and this block
        // has no pins at all — it is arithmetic with an address.
        Ok(())
    }

    fn reset(&self, _kind: ResetKind) {
        // Both kinds. Everything here is APB register state with no battery
        // behind it, and an accumulator that survived a reset would give a
        // checksum that depended on what ran before the reboot.
        *self.regs.state.lock() = State::default();
    }

    fn save(&self, w: &mut ChunkWriter<'_>) -> Result<()> {
        let state = *self.regs.state.lock();
        w.write_u8(match self.regs.variant {
            Variant::F4 => 0,
            Variant::V2 => 1,
        })?;
        w.write_u32(state.acc)?;
        w.write_u8(state.idr)?;
        w.write_u32(state.init)?;
        w.write_u32(state.pol)?;
        w.write_u32(state.polysize)?;
        w.write_u32(state.rev_in)?;
        w.write_bool(state.rev_out)
    }

    fn load(&self, r: &mut ChunkReader<'_>) -> Result<()> {
        let variant = match r.read_u8()? {
            0 => Variant::F4,
            1 => Variant::V2,
            other => {
                return Err(Error::State(format!(
                    "snapshot has CRC variant {other}, which this build does not know"
                )));
            }
        };
        if variant != self.regs.variant {
            return Err(Error::State(format!(
                "snapshot has a `{}` CRC unit, this one is `{}`",
                variant.name(),
                self.regs.variant.name()
            )));
        }
        let state = State {
            acc: r.read_u32()?,
            idr: r.read_u8()?,
            init: r.read_u32()?,
            pol: r.read_u32()?,
            polysize: r.read_u32()?,
            rev_in: r.read_u32()?,
            rev_out: r.read_bool()?,
        };
        if state.polysize > 3 || state.rev_in > 3 {
            return Err(Error::State(format!(
                "snapshot has a CRC unit with POLYSIZE={} and REV_IN={}",
                state.polysize, state.rev_in
            )));
        }
        *self.regs.state.lock() = state;
        Ok(())
    }

    fn region(&self, name: &str) -> Option<RegionRef> {
        matches!(name, "" | "regs").then(|| Arc::clone(&self.region))
    }
}

impl Instance for Crc {}

/// The `st.crc` device class.
pub static CLASS: DeviceClass = DeviceClass {
    name: CLASS_NAME,
    version: STATE_VERSION,
    summary: "STM32 CRC calculation unit: the fixed CRC-32 block and the programmable one",
    properties: &[PropertySpec {
        name: "variant",
        kind: ValueKind::Str,
        required: false,
        summary: "`f4` (default): a fixed CRC-32. `v2`: programmable POL, INIT and reversal",
    }],
    construct: |props| Ok(Box::new(Crc::new(props)?)),
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
    bindings.bind(CLASS_NAME, |props| Ok(Arc::new(Crc::new(props)?)))
}

/// What the validator should know about `st.crc`.
#[must_use]
pub fn schema() -> ClassSchema {
    ClassSchema::new(CLASS_NAME)
        .prop(PropSchema::new("variant", ValueKind::Str).values(&["f4", "v2"]))
        .region("")
        .region("regs")
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::props::Value;
    use crate::core::registry::Registry;
    use crate::core::state::{MachineShape, Migrations, StateReader, StateWriter};
    use alloc::vec::Vec;

    /// The check string every CRC catalogue uses, and the one the arithmetic
    /// below is exercised with.
    const CHECK: &[u8] = b"123456789";

    fn f4() -> Crc {
        Crc::build(Variant::F4)
    }

    fn v2() -> Crc {
        Crc::build(Variant::V2)
    }

    fn read(d: &Crc, offset: u64, len: usize) -> core::result::Result<u32, BusError> {
        let mut buf = [0u8; 4];
        d.regs.read(offset, &mut buf[..len], MemAttrs::DEFAULT)?;
        Ok(u32::from_le_bytes(buf))
    }

    fn peek(d: &Crc, offset: u64) -> u32 {
        read(d, offset, 4).expect("a word read is legal")
    }

    fn write(d: &Crc, offset: u64, value: u32, len: usize) -> MemResult {
        d.regs
            .write(offset, &value.to_le_bytes()[..len], MemAttrs::DEFAULT)
    }

    fn poke(d: &Crc, offset: u64, value: u32) {
        write(d, offset, value, 4).expect("a word write is legal");
    }

    /// Write one byte to `DR`.
    fn poke_byte(d: &Crc, value: u8) {
        write(d, 0x00, u32::from(value), 1).expect("a byte write to DR");
    }

    /// Program `CR` and reset the accumulator in one write.
    fn configure(d: &Crc, polysize: u32, rev_in: u32, rev_out: bool) {
        let mut cr = CR_RESET | (polysize << CR_POLYSIZE_SHIFT) | (rev_in << CR_REV_IN_SHIFT);
        if rev_out {
            cr |= CR_REV_OUT;
        }
        poke(d, 0x08, cr);
    }

    #[test]
    fn a_single_zero_word_after_reset_gives_c704dd7b() {
        // The shift register of the module documentation, run over
        // `0x04C11DB7` from `0xFFFFFFFF` for thirty-two zero bits.
        let d = f4();
        poke(&d, 0x08, CR_RESET);
        poke(&d, 0x00, 0);
        assert_eq!(peek(&d, 0x00), 0xc704_dd7b);
        // And without the reset first, because that is the state it comes up
        // in anyway.
        let d = f4();
        poke(&d, 0x00, 0);
        assert_eq!(peek(&d, 0x00), 0xc704_dd7b);
    }

    #[test]
    fn the_fixed_block_has_no_pol_and_no_init_to_read() {
        let d = f4();
        assert_eq!(d.variant().register_bytes(), F4_BYTES);
        assert_eq!(read(&d, 0x10, 4), Err(BusError::BadAccess), "no INIT");
        assert_eq!(read(&d, 0x14, 4), Err(BusError::BadAccess), "no POL");
        assert_eq!(d.width(), 32);
        // `CR` is one self-clearing bit, so it never reads anything back.
        poke(&d, 0x08, CR_RESET);
        assert_eq!(peek(&d, 0x08), 0);
    }

    #[test]
    fn the_f4_variant_ignores_writes_to_pol_and_init_and_rejects_narrow_dr_writes() {
        let d = f4();
        // The addresses do not decode, so a write to them is a bus fault the
        // guest can see rather than a silent no-op that leaves firmware
        // believing it reprogrammed a polynomial it cannot.
        assert_eq!(write(&d, 0x10, 0x1234_5678, 4), Err(BusError::BadAccess));
        assert_eq!(write(&d, 0x14, 0x0000_1021, 4), Err(BusError::BadAccess));
        // The arithmetic is the one it is wired with.
        poke(&d, 0x08, CR_RESET);
        poke(&d, 0x00, 0);
        assert_eq!(peek(&d, 0x00), 0xc704_dd7b);

        // "Only 32-bit data can be written." A byte write is refused rather
        // than folded in as an eight-bit unit, because answering it would give
        // a checksum that is quietly wrong.
        assert_eq!(write(&d, 0x00, 0x31, 1), Err(BusError::BadAccess));
        assert_eq!(write(&d, 0x00, 0x3231, 2), Err(BusError::BadAccess));
        assert_eq!(read(&d, 0x00, 1), Err(BusError::BadAccess));
        // The whole v2 register window is outside this block's aperture.
        assert_eq!(d.variant().register_bytes(), F4_BYTES);
    }

    #[test]
    fn the_check_string_with_byte_reversal_in_and_out_matches_the_reflected_crc32() {
        // REV_IN = byte and REV_OUT together turn the manual's MSB-first
        // shift register into the reflected form; the final exclusive-or is
        // firmware's, not the hardware's.
        let d = v2();
        configure(&d, 0, 1, true);
        for byte in CHECK {
            poke_byte(&d, *byte);
        }
        assert_eq!(peek(&d, 0x00) ^ 0xffff_ffff, 0xcbf4_3926);
    }

    #[test]
    fn the_same_bytes_as_two_words_and_one_byte_give_the_same_result_as_nine_bytes() {
        // The unit is a bit stream: how the guest chose to chop it up cannot
        // change the answer, only how many writes it took.
        for (rev_in, rev_out) in [(0u32, false), (1, true)] {
            let bytes = v2();
            configure(&bytes, 0, rev_in, rev_out);
            for byte in CHECK {
                poke_byte(&bytes, *byte);
            }

            let mixed = v2();
            configure(&mixed, 0, rev_in, rev_out);
            // Big-endian packing, because the register is fed most significant
            // bit first and that is the order the bytes arrive in.
            poke(&mixed, 0x00, u32::from_be_bytes(*b"1234"));
            poke(&mixed, 0x00, u32::from_be_bytes(*b"5678"));
            poke_byte(&mixed, b'9');

            assert_eq!(
                peek(&bytes, 0x00),
                peek(&mixed, 0x00),
                "rev_in {rev_in}, rev_out {rev_out}"
            );
        }
        // And the unreversed answer is what the shift register gives.
        let d = v2();
        configure(&d, 0, 0, false);
        for byte in CHECK {
            poke_byte(&d, *byte);
        }
        assert_eq!(peek(&d, 0x00), 0x0376_e6e7);
    }

    #[test]
    fn a_halfword_write_feeds_two_bytes() {
        let d = v2();
        configure(&d, 0, 0, false);
        write(&d, 0x00, 0x1234, 2).expect("a half-word write to DR");
        write(&d, 0x00, 0x5678, 2).expect("a half-word write to DR");
        assert_eq!(peek(&d, 0x00), 0xdf8a_8a2b);
        // The same four bytes as one word are the same bit stream.
        let word = v2();
        configure(&word, 0, 0, false);
        poke(&word, 0x00, 0x1234_5678);
        assert_eq!(peek(&word, 0x00), peek(&d, 0x00));
    }

    #[test]
    fn polysize_16_with_pol_0x1021_and_init_0xffff_matches_ccitt() {
        let d = v2();
        poke(&d, 0x14, 0x1021);
        poke(&d, 0x10, 0xffff);
        configure(&d, 1, 0, false);
        assert_eq!(d.width(), 16);
        for byte in CHECK {
            poke_byte(&d, *byte);
        }
        assert_eq!(peek(&d, 0x00), 0x29b1);
        assert_eq!(peek(&d, 0x00) >> 16, 0, "the register is sixteen bits wide");
    }

    #[test]
    fn polysize_8_with_pol_0x07_from_zero_is_an_eight_bit_register() {
        let d = v2();
        poke(&d, 0x14, 0x07);
        poke(&d, 0x10, 0x00);
        configure(&d, 2, 0, false);
        assert_eq!(d.width(), 8);
        for byte in CHECK {
            poke_byte(&d, *byte);
        }
        assert_eq!(peek(&d, 0x00), 0xf4);
    }

    #[test]
    fn polysize_7_masks_the_accumulator_to_seven_bits() {
        // A seven-bit shift register fed eight-bit units, which is the case
        // the bit-at-a-time formulation exists for.
        let d = v2();
        poke(&d, 0x14, 0x09);
        poke(&d, 0x10, 0x00);
        configure(&d, 3, 0, false);
        assert_eq!(d.width(), 7);
        for byte in CHECK {
            poke_byte(&d, *byte);
            assert_eq!(peek(&d, 0x00) & !0x7f, 0, "never wider than seven bits");
        }
        assert_eq!(peek(&d, 0x00), 0x75);
    }

    #[test]
    fn rev_out_reverses_the_polynomial_s_width_and_not_always_thirty_two() {
        let plain = v2();
        poke(&plain, 0x14, 0x1021);
        poke(&plain, 0x10, 0xffff);
        configure(&plain, 1, 0, false);

        let reversed = v2();
        poke(&reversed, 0x14, 0x1021);
        poke(&reversed, 0x10, 0xffff);
        configure(&reversed, 1, 0, true);

        for byte in CHECK {
            poke_byte(&plain, *byte);
            poke_byte(&reversed, *byte);
        }
        assert_eq!(peek(&plain, 0x00), 0x29b1);
        assert_eq!(peek(&reversed, 0x00), 0x8d94, "0x29b1 reversed in sixteen");
        // Reversing thirty-two bits instead would have put it in the high
        // half, which is the reading this model does not take.
        assert_eq!(peek(&reversed, 0x00) >> 16, 0);
    }

    #[test]
    fn rev_in_reverses_within_the_unit_it_names() {
        // Each granularity is a different bit stream, so each is a different
        // answer, and a byte write has only one byte to reverse whichever of
        // the three is selected.
        let mut answers = Vec::new();
        for rev_in in [0u32, 1, 2, 3] {
            let d = v2();
            configure(&d, 0, rev_in, false);
            poke(&d, 0x00, 0x1234_5678);
            answers.push(peek(&d, 0x00));
        }
        assert_eq!(answers[3], 0xb41e_490a, "whole-word reversal");
        assert_eq!(answers[2], 0x2cfc_7ddb, "half-word reversal");
        for (i, a) in answers.iter().enumerate() {
            for b in &answers[i + 1..] {
                assert_ne!(a, b, "the four settings are four bit streams");
            }
        }

        // A word reversal of a byte write reverses the byte: there is nothing
        // else in the unit to reverse.
        let word = v2();
        configure(&word, 0, 3, false);
        poke_byte(&word, 0x31);
        let byte = v2();
        configure(&byte, 0, 1, false);
        poke_byte(&byte, 0x31);
        assert_eq!(peek(&word, 0x00), peek(&byte, 0x00));
    }

    #[test]
    fn reset_reloads_init_not_all_ones_when_init_is_programmed() {
        let d = v2();
        poke(&d, 0x10, 0x1234_5678);
        poke(&d, 0x08, CR_RESET);
        assert_eq!(peek(&d, 0x00), 0x1234_5678, "the accumulator is INIT");
        poke(&d, 0x00, 0);
        assert_ne!(peek(&d, 0x00), 0xc704_dd7b, "a different starting point");

        // Writing INIT does not itself reload — the manual makes RESET the
        // thing that does — so a firmware that forgets the reset keeps
        // accumulating.
        let d = v2();
        poke(&d, 0x00, 0);
        let after_one_word = peek(&d, 0x00);
        poke(&d, 0x10, 0x0000_0000);
        assert_eq!(peek(&d, 0x00), after_one_word);
        poke(&d, 0x08, CR_RESET);
        assert_eq!(peek(&d, 0x00), 0);
    }

    #[test]
    fn reset_reloads_under_the_polysize_the_same_write_asks_for() {
        let d = v2();
        poke(&d, 0x10, 0xffff_ffff);
        configure(&d, 1, 0, false);
        assert_eq!(peek(&d, 0x00), 0xffff, "INIT masked to sixteen bits");
        configure(&d, 3, 0, false);
        assert_eq!(peek(&d, 0x00), 0x7f, "and to seven");
    }

    #[test]
    fn idr_is_an_independent_scratch_byte() {
        for d in [f4(), v2()] {
            poke(&d, 0x08, CR_RESET);
            write(&d, 0x04, 0xa5, 1).expect("a byte write to IDR");
            assert_eq!(read(&d, 0x04, 1).unwrap() & 0xff, 0xa5);
            assert_eq!(peek(&d, 0x04), 0xa5, "eight bits, zero-extended");
            // The CRC does not touch it and it does not touch the CRC.
            poke(&d, 0x00, 0);
            assert_eq!(peek(&d, 0x04), 0xa5);
            assert_eq!(peek(&d, 0x00), 0xc704_dd7b);
            // A reset of the unit does not clear it either — it is scratch for
            // the *application*, not part of the calculation.
            poke(&d, 0x08, CR_RESET);
            assert_eq!(peek(&d, 0x04), 0xa5);
            // …but a reset of the chip does.
            Device::reset(&d, ResetKind::Cold);
            assert_eq!(peek(&d, 0x04), 0);
        }
    }

    #[test]
    fn a_debug_write_is_refused_and_a_debug_read_is_free() {
        let d = v2();
        assert_eq!(
            d.regs.write(0x00, &0u32.to_le_bytes(), MemAttrs::DEBUG),
            Err(BusError::BadAccess)
        );
        assert_eq!(d.value(), DEFAULT_INIT, "nothing was folded in");
        poke(&d, 0x00, 0);
        let mut word = [0u8; 4];
        d.regs
            .read(0x00, &mut word, MemAttrs::DEBUG)
            .expect("reading DR is free");
        assert_eq!(u32::from_le_bytes(word), 0xc704_dd7b);
        assert_eq!(d.value(), 0xc704_dd7b, "and it did not advance the unit");
    }

    #[test]
    fn an_offset_outside_the_block_is_a_fault() {
        let d = v2();
        assert_eq!(read(&d, 0x18, 4), Err(BusError::BadAccess));
        assert_eq!(read(&d, 0x0c, 4), Err(BusError::BadAccess), "a hole");
        let d = f4();
        assert_eq!(read(&d, 0x10, 4), Err(BusError::BadAccess), "no INIT here");
        assert_eq!(
            d.regs.constraints(),
            AccessConstraints::word(Width::U32, Endian::Little).with_widths(Width::U8, Width::U32)
        );
    }

    #[test]
    fn a_reset_returns_the_unit_to_the_fixed_parameters() {
        let d = v2();
        poke(&d, 0x14, 0x1021);
        poke(&d, 0x10, 0x0000);
        configure(&d, 1, 3, true);
        poke_byte(&d, 0x31);
        Device::reset(&d, ResetKind::Warm);
        assert_eq!(peek(&d, 0x14), DEFAULT_POLYNOMIAL);
        assert_eq!(peek(&d, 0x10), DEFAULT_INIT);
        assert_eq!(peek(&d, 0x08), 0);
        assert_eq!(peek(&d, 0x00), DEFAULT_INIT);
        assert_eq!(d.width(), 32);
    }

    #[test]
    fn a_snapshot_round_trips_to_identical_state() {
        let saved = v2();
        poke(&saved, 0x14, 0x1021);
        poke(&saved, 0x10, 0xabcd);
        configure(&saved, 1, 2, true);
        write(&saved, 0x04, 0x5a, 1).unwrap();
        for byte in CHECK {
            poke_byte(&saved, *byte);
        }

        let mut shape = MachineShape::new();
        shape.add_device("crc", CLASS_NAME).unwrap();
        let mut w = StateWriter::new(shape);
        {
            let mut chunk = w.chunk("crc", CLASS_NAME, STATE_VERSION).unwrap();
            Device::save(&saved, &mut chunk).unwrap();
        }
        let bytes = w.to_vec().unwrap();

        let restored = v2();
        let reader = StateReader::new(&bytes).unwrap();
        let chunk = reader
            .load("crc", CLASS_NAME, STATE_VERSION, &Migrations::new())
            .unwrap();
        Device::load(&restored, &mut chunk.reader()).unwrap();

        let offsets = [0x00, 0x04, 0x08, 0x10, 0x14];
        let before: Vec<u32> = offsets.iter().map(|o| peek(&saved, *o)).collect();
        let after: Vec<u32> = offsets.iter().map(|o| peek(&restored, *o)).collect();
        assert_eq!(before, after);

        // And the calculation carries on from the same place, which the
        // register values alone do not prove: `DR` reads the accumulator
        // through `REV_OUT`, so two different accumulators could read alike.
        poke_byte(&saved, b'x');
        poke_byte(&restored, b'x');
        assert_eq!(peek(&saved, 0x00), peek(&restored, 0x00));
    }

    #[test]
    fn a_snapshot_of_the_other_variant_is_refused() {
        let saved = v2();
        let mut shape = MachineShape::new();
        shape.add_device("crc", CLASS_NAME).unwrap();
        let mut w = StateWriter::new(shape);
        {
            let mut chunk = w.chunk("crc", CLASS_NAME, STATE_VERSION).unwrap();
            Device::save(&saved, &mut chunk).unwrap();
        }
        let bytes = w.to_vec().unwrap();
        let reader = StateReader::new(&bytes).unwrap();
        let chunk = reader
            .load("crc", CLASS_NAME, STATE_VERSION, &Migrations::new())
            .unwrap();
        assert!(Device::load(&f4(), &mut chunk.reader()).is_err());
    }

    #[test]
    fn a_property_this_class_does_not_know_is_a_typo() {
        let props = Props::new().with("variant", Value::from("v2"));
        assert_eq!(Crc::new(&props).unwrap().variant(), Variant::V2);
        assert_eq!(Crc::new(&Props::new()).unwrap().variant(), Variant::F4);
        assert!(Crc::new(&Props::new().with("variant", Value::from("l4"))).is_err());
        assert!(Crc::new(&Props::new().with("polynomial", Value::from(7u64))).is_err());
    }

    #[test]
    fn the_class_is_registrable_and_constructs_through_the_registry() {
        let mut reg = Registry::new();
        register(&mut reg).unwrap();
        assert!(register(&mut reg).is_err(), "twice is a collision");
        let device = reg.create(CLASS_NAME, &Props::new()).unwrap();
        assert_eq!(device.class().name, CLASS_NAME);
    }

    #[test]
    fn the_schema_and_the_device_agree_about_regions() {
        let d = v2();
        let schema = schema();
        assert_eq!(schema.ports.len(), 0, "this block has no pins");
        assert!(Device::region(&d, "").is_some());
        assert!(Device::region(&d, "regs").is_some());
        assert!(Device::region(&d, "dr").is_none());
    }
}
