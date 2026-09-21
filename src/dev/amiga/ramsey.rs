//! Ramsey: the A3000's memory controller, as software can see it.
//!
//! Two registers, and that is the whole of the chip a program can touch:
//!
//! | Register | Bit | Name | Function |
//! | --- | --- | --- | --- |
//! | `$00DE0003` | 7 | `TEST` | used for production test |
//! | | 6,5 | `REFRESH RATE` | timing for refresh |
//! | | 4 | `RAMWIDTH` | set up for ×4 or ×1 RAM |
//! | | 3 | `RAMSIZE` | 1 MB or 4 MB density |
//! | | 2 | `WRAP` | enable burst wrapping |
//! | | 1 | `BURST` | run 68030 burst cycles |
//! | | 0 | `PAGE DETECT` | use page-detect logic |
//! | `$00DE0043` | 7–0 | `VERSION` | Ramsey chip version |
//!
//! (*The A3000+ System Specification*, §2.2, Table 2-2. That document describes
//! the *enhanced* part and says so; it is the citation for the A3000's own
//! because it states which behaviour is which — "original A3000 versions of
//! RAMSEY (version code `$0D`) don't correctly support this mode", which is
//! also where the default [`VERSION_A3000`] comes from.)
//!
//! # Why a board needs this at all
//!
//! Everything the chip *does* — DRAM refresh, static-column page detection,
//! 68030 burst cycles, the row and column strobes — is invisible to an emulator
//! whose RAM answers in no time at all. What is not invisible is that
//! **Kickstart waits for this register to read back what it wrote**. Its
//! memory-sizing routine sets the bottom three bits and spins:
//!
//! ```text
//!     LEA     $00DE0003.l,A4
//!     LEA     $07F7FFF0.l,A3          ; the top of the Fast RAM window
//!     MOVEQ   #$7,D2
//!     CMPI.B  #$7F,$40(A4)            ; $00DE0043: no Ramsey?
//!     BEQ     done
//!     …
//!     MOVE.B  D0,(A4)                 ; set WRAP | BURST | PAGE DETECT
//!   wait:
//!     MOVE.B  (A4),D1
//!     AND.B   D2,D1
//!     CMP.B   D2,D1
//!     BNE     wait
//! ```
//!
//! With `$00DE0003` floating, that loop never ends and the machine never
//! reaches `exec`. So this file exists for one reason: a register that reads
//! back what was written, on a chip whose every other function is a timing
//! detail this emulator does not have. The spin loop is the specification's
//! own suggestion — "writes to `$00DE0003` don't actually take effect until the
//! next refresh cycle. A read loop on the changed bit can be used if it
//! necessary to wait for a change" (§2.2) — and a write here takes effect at
//! once, which that loop is written to tolerate.
//!
//! The version register is the other half of the same story. `$7F` is what the
//! ROM treats as "there is no Ramsey here", so a board that answers `$0D`
//! is telling the truth about what it has and a board that floats is not
//! telling the ROM anything reliable.
//!
//! # What is not modelled
//!
//! The refresh rate, the burst and page-detect modes, and the ×4/×1 and
//! 1 MB/4 MB straps: every bit is readable and writable and **none of them
//! changes anything**, because nothing downstream of this chip has a cycle
//! count. `RAMSIZE` is a jumper on a real board (J852) and is therefore a
//! construction property here rather than something the reset value invents.
//!
//! **The address counter.** "the ACR, physically contained in the RAMSEY chip,
//! will actually round any value written to it down to an even word" (§2.4.1) —
//! the DMA address register is *physically* Ramsey's and *architecturally* the
//! DMAC's, and it lives in [`crate::dev::amiga::sdmac`] with the rest of the
//! register block software reaches it through.
//!
//! **No emulator source, no AROS source and no Kickstart disassembly was
//! consulted** (`CLAUDE.md`, provenance). The instruction listing above is
//! rsemu's own disassembler printing what the guest was executing when it
//! stopped, which is black-box observation of a running machine.

#[cfg(test)]
mod tests;

use alloc::boxed::Box;
use alloc::format;
use alloc::sync::Arc;

use crate::core::device::{Device, DeviceClass, PropertySpec, RealizeCtx, ResetKind};
use crate::core::error::{BusError, Error, Result};
use crate::core::props::{Props, ValueKind};
use crate::core::space::{AccessConstraints, MemAttrs, MemOps, MemResult, Region, RegionRef};
use crate::core::state::{ChunkReader, ChunkWriter, Sink, Source};
use crate::core::sync::{LockRank, Mutex};
use crate::core::value::{Endian, Width};
use crate::machine::realize::Instance;
use crate::machine::validate::{ClassSchema, PropSchema};

/// The class name a machine file writes.
pub const CLASS_NAME: &str = "amiga.ramsey";

/// The snapshot chunk version. Bump with the encoding, never on its own.
const STATE_VERSION: u32 = 1;

/// The region a `map` statement places at `$00DE0000`.
pub const REGS_REGION: &str = "regs";

/// How much the window decodes: `$00DE0000`–`$00DE00FF`, which holds both
/// registers with room to spare.
pub const REGS_WINDOW_LEN: u64 = 0x100;

/// Where the control register is in the window.
pub const CONTROL: u64 = 0x03;

/// Where the version register is in the window.
pub const VERSION: u64 = 0x43;

/// `TEST`, used for production test.
pub const TEST: u8 = 0x80;
/// The refresh rate field, bits 6 and 5.
pub const REFRESH_RATE: u8 = 0x60;
/// `RAMWIDTH`: ×4 parts when set, ×1 when clear.
pub const RAMWIDTH: u8 = 0x10;
/// `RAMSIZE`: 1 MB×4 when set, 256 KB×4 when clear. A jumper on the board.
pub const RAMSIZE: u8 = 0x08;
/// `WRAP`: a 68030 burst wraps within its quadlongword.
pub const WRAP: u8 = 0x04;
/// `BURST`: run 68030 burst cycles.
pub const BURST: u8 = 0x02;
/// `PAGE DETECT`: hold `RAS` low and use the page comparators.
pub const PAGE_DETECT: u8 = 0x01;

/// The version an A3000's Ramsey reports: "original A3000 versions of RAMSEY
/// (version code `$0D`)" (§2.2.2).
pub const VERSION_A3000: u8 = 0x0d;

/// The version an A3000+ or an A4000's reports — the part that supports
/// page-detect mode properly (§2.2.2: "Any program … should check for a version
/// `$0E` or later RAMSEY before enabling this feature").
pub const VERSION_ENHANCED: u8 = 0x0f;

/// What the control register holds out of reset.
///
/// `RAMWIDTH` set, because "RAMWIDTH defaults to one" (§2.2.1); `RAMSIZE` set,
/// because the A3000's jumper reads high for the 1 MB×4 parts a machine with
/// more than 4 MiB of motherboard RAM is fitted with. Everything else clear:
/// the fastest refresh, no burst, no page detect, which is the "five clock
/// cycle to DRAM" the specification says the chip runs on power-up.
pub const CONTROL_RESET: u8 = RAMWIDTH | RAMSIZE;

/// The chip's two registers.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Regs {
    /// `$00DE0003`.
    pub control: u8,
    /// `$00DE0043`, read-only.
    pub version: u8,
}

// ---------------------------------------------------------------------------
// the window
// ---------------------------------------------------------------------------

/// The shared half of the device.
#[derive(Debug)]
struct Chip {
    regs: Mutex<Regs>,
    reset: Regs,
}

/// `$00DE0000`–`$00DE00FF`, one byte of which is a register.
#[derive(Debug)]
struct RegsWindow(Arc<Chip>);

impl MemOps for RegsWindow {
    fn read(&self, offset: u64, dst: &mut [u8], attrs: MemAttrs) -> MemResult {
        // Neither register has a read side effect, so a debugger's read is an
        // ordinary one and `MemAttrs::debug` needs nothing of its own.
        let regs = *self.0.regs.lock();
        for (i, byte) in dst.iter_mut().enumerate() {
            *byte = match offset + i as u64 {
                CONTROL => regs.control,
                VERSION => regs.version,
                // "each contains one significant bit, which will be D7 in the
                // access byte" is Gary's rule and not this chip's; every other
                // address in the window belongs to no Ramsey register at all,
                // so the bus floats.
                _ => attrs.bus,
            };
        }
        Ok(())
    }

    fn write(&self, offset: u64, src: &[u8], attrs: MemAttrs) -> MemResult {
        if attrs.debug {
            // Writing the control register changes how the guest's own memory
            // is refreshed. It is not observable here, but a debugger that did
            // it would still have changed guest-visible state.
            return Err(BusError::BadAccess);
        }
        let mut regs = self.0.regs.lock();
        for (i, byte) in src.iter().enumerate() {
            if offset + i as u64 == CONTROL {
                regs.control = *byte;
            }
            // The version register is read-only: §2.2's table marks it
            // `VERSION` with no writable field, and a chip that let software
            // rename itself would be a strange chip.
        }
        Ok(())
    }

    fn constraints(&self) -> AccessConstraints {
        AccessConstraints {
            min: Width::U8,
            natural_alignment: false,
            ..AccessConstraints::word(Width::U32, Endian::Big)
        }
    }
}

// ---------------------------------------------------------------------------
// the device
// ---------------------------------------------------------------------------

/// The A3000's memory controller.
#[derive(Debug)]
pub struct Ramsey {
    chip: Arc<Chip>,
    regs: RegionRef,
}

impl Ramsey {
    /// Validate `props` and build the chip.
    ///
    /// # Errors
    ///
    /// [`Error::Property`] if a property is of the wrong kind or unknown.
    pub fn new(props: &Props) -> Result<Ramsey> {
        let mut r = props.reader();
        let version = r.or_range("version", u64::from(VERSION_A3000), 0..=0xff)? as u8;
        let control = r.or_range("control", u64::from(CONTROL_RESET), 0..=0xff)? as u8;
        r.finish()?;
        Ok(Ramsey::with_regs(Regs { control, version }))
    }

    /// Build one whose reset values the caller chose.
    #[must_use]
    pub fn with_regs(reset: Regs) -> Ramsey {
        let chip = Arc::new(Chip {
            regs: Mutex::with_rank(LockRank::DEVICE, reset),
            reset,
        });
        Ramsey {
            regs: Arc::new(Region::io(
                format!("{CLASS_NAME}.{REGS_REGION}"),
                REGS_WINDOW_LEN,
                Arc::new(RegsWindow(Arc::clone(&chip))),
            )),
            chip,
        }
    }

    /// The registers as they stand.
    #[must_use]
    pub fn regs(&self) -> Regs {
        *self.chip.regs.lock()
    }
}

/// The `amiga.ramsey` device class.
pub static CLASS: DeviceClass = DeviceClass {
    name: CLASS_NAME,
    version: STATE_VERSION,
    summary: "the A3000's memory controller, as software can see it: the control register at \
              $DE0003 and the version register at $DE0043",
    properties: &[
        PropertySpec {
            name: "version",
            kind: ValueKind::Uint,
            required: false,
            summary: "what $DE0043 reports; `13` is an A3000's Ramsey, `15` an enhanced one",
        },
        PropertySpec {
            name: "control",
            kind: ValueKind::Uint,
            required: false,
            summary: "what $DE0003 holds out of reset: the RAMWIDTH and RAMSIZE straps, the \
                      refresh rate, and the three mode bits",
        },
    ],
    construct: |props| Ok(Box::new(Ramsey::new(props)?)),
};

impl Device for Ramsey {
    fn class(&self) -> &'static DeviceClass {
        &CLASS
    }

    fn realize(&self, _ctx: &mut RealizeCtx<'_>) -> Result<()> {
        // Nothing outward: a `map` statement places the window and the chip
        // has no pin a machine file can reach.
        Ok(())
    }

    fn reset(&self, _kind: ResetKind) {
        // Both kinds: the straps are straps and the mode bits come back clear,
        // which is the "five clock cycle to DRAM" power-up state of §2.2.2.
        *self.chip.regs.lock() = self.chip.reset;
    }

    fn region(&self, name: &str) -> Option<RegionRef> {
        match name {
            "" | REGS_REGION => Some(Arc::clone(&self.regs)),
            _ => None,
        }
    }

    fn save(&self, w: &mut ChunkWriter<'_>) -> Result<()> {
        let regs = *self.chip.regs.lock();
        w.write_u8(regs.control)?;
        w.write_u8(regs.version)?;
        Ok(())
    }

    fn load(&self, r: &mut ChunkReader<'_>) -> Result<()> {
        let control = r.read_u8()?;
        let version = r.read_u8()?;
        if version != self.chip.reset.version {
            return Err(Error::State(format!(
                "the snapshot's Ramsey is version {version:#04x} and this one is {:#04x}",
                self.chip.reset.version
            )));
        }
        *self.chip.regs.lock() = Regs { control, version };
        Ok(())
    }
}

impl Instance for Ramsey {}

/// Add [`CLASS`] to a registry.
///
/// # Errors
///
/// [`Error::Config`] if something already claimed the name.
pub fn register(registry: &mut crate::core::Registry) -> Result<()> {
    registry.add(&CLASS)
}

/// Bind [`CLASS`] into the machine graph.
///
/// # Errors
///
/// [`Error::Config`] if the class is already bound.
pub fn bind(bindings: &mut crate::machine::Bindings) -> Result<()> {
    bindings.bind(CLASS_NAME, |props| Ok(Arc::new(Ramsey::new(props)?)))
}

/// What the validator should know about `amiga.ramsey`.
#[must_use]
pub fn schema() -> ClassSchema {
    ClassSchema::new(CLASS_NAME)
        .prop(PropSchema::new("version", ValueKind::Uint))
        .prop(PropSchema::new("control", ValueKind::Uint))
        .region("")
        .region(REGS_REGION)
}
