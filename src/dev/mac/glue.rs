//! The Macintosh's address decoder: the ROM overlay at zero, and RAM in the
//! window the ROM sizes memory through.
//!
//! # Sources
//!
//! *Guide to the Macintosh Family Hardware*, 2nd edition, chapter 3 ("Memory
//! and Addressing"): the Macintosh Plus address map, and the overlay the
//! VIA's `ROMOVERLAY` bit controls. No emulator source was consulted
//! (`ROADMAP.md` §1).
//!
//! # What the decoder does
//!
//! A 128K Macintosh, a 512K, and a Plus all decode the low 4 MB two different
//! ways depending on one bit:
//!
//! ```text
//!   overlay asserted (the state at power-on)
//!     $00_0000 - $3F_FFFF   the ROM, repeating every 128 KiB
//!     $60_0000 - $7F_FFFF   main memory, repeating every `ram` bytes
//!
//!   overlay cleared (what software does once it has sized memory)
//!     $00_0000 - $3F_FFFF   main memory, repeating every `ram` bytes
//!     $60_0000 - $7F_FFFF   nothing at all
//! ```
//!
//! and `$40_0000 - $4F_FFFF` is the ROM either way, which is how the processor
//! finds anything at all: the reset vector at `$00_0004` is fetched out of the
//! overlaid ROM and points into `$40_xxxx`, where the same ROM answers after
//! the overlay is gone.
//!
//! # Both the ROM *and* main memory repeat, and the memory's fold is
//! load-bearing
//!
//! The ROM socket carries no address line above its own, so a 128 KiB part
//! answers all through the megabyte its select decodes. **Main memory does the
//! same thing** for the same reason: the DRAM is given only the address lines
//! its own depth needs, so a board with less than four megabytes on it leaves
//! `A20`/`A21` out of the decode and the memory answers again every `len`
//! bytes right up to `$3F_FFFF`.
//!
//! That fold is what makes the boot screen work, and it is a *measurement*
//! rather than a reading of the schematic. The Plus ROM draws the
//! insert-disk icon through a pointer of `$3F_CB5E` — a constant near the top
//! of the four-megabyte window, not a number derived from `ScrnBase`. Folded,
//! that address lands on `MemTop - $5900 + $245E` on **every** power-of-two
//! size: `$0F_CB5E` on a 1 MiB board, `$1F_CB5E` on a 2 MiB one, `$3F_CB5E` on
//! a 4 MiB one — the middle of the screen, every time. Unfolded it lands in
//! nothing on anything but a 4 MiB machine, the icon is written into the void,
//! and the ROM parks in its `$4006E8` idle loop having drawn a screen nobody
//! can see. That was this board's long-standing hang.
//!
//! The fold does *not* cost the ROM its memory sizing, which was the worry
//! that put the opposite claim here to begin with: the ROM still writes
//! `MemTop = $00100000` and `ScrnBase = $000FA700` on the stock board, because
//! its sizing pass writes markers at two addresses and compares, which is
//! exactly the test an alias fails. `tests/mac_plus.rs` asserts both numbers
//! on 1 MiB and on 4 MiB.
//!
//! # Why a decoder and not two mappings swapped
//!
//! The same reason `amiga.gary` gives: the write that clears the overlay
//! arrives *through* the address space, so calling `AddressSpace::topology`
//! from inside it would deadlock against the read guard the access already
//! holds, and deferring it would apply a whole quantum late — long after the
//! processor has fetched its next instruction from what is now the wrong
//! memory. One decoder in front of both, switched by an atomic, has neither
//! problem.

use alloc::boxed::Box;
use alloc::format;
use alloc::string::{String, ToString};
use alloc::sync::Arc;
use core::sync::atomic::{AtomicBool, Ordering};

use crate::core::device::{Device, DeviceClass, PropertySpec, RealizeCtx, ResetKind, SinkPin};
use crate::core::error::{BusError, Error, Result};
use crate::core::props::{Props, ValueKind};
use crate::core::space::{
    AccessConstraints, AddressSpace, MemAttrs, MemOps, MemResult, Region, RegionRef,
    UnassignedPolicy,
};
use crate::core::state::{ChunkReader, ChunkWriter, Sink, Source};
use crate::core::sync::{LockRank, Mutex};
use crate::core::wire::{FanIn, Level, WireId, WireSink};
use crate::machine::realize::{BindCtx, Instance};
use crate::machine::validate::{ClassSchema, PortDir, PropSchema};

/// The class name a machine description writes.
pub const CLASS_NAME: &str = "mac.glue";

/// The snapshot chunk version. Bump with the encoding, never on its own.
pub const STATE_VERSION: u32 = 1;

/// The input pin the VIA's `PA4` drives.
pub const OVERLAY_PIN: &str = "overlay";

/// The region a `map` statement places at `$00_0000`.
pub const LOW_REGION: &str = "low";

/// The region a `map` statement places at `$60_0000`.
pub const HIGH_REGION: &str = "high";

/// How much of the space the low window decodes: `$00_0000`-`$3F_FFFF`, the
/// four megabytes a Plus's `A23`/`A22` decode gives RAM.
pub const LOW_WINDOW: u64 = 0x40_0000;

/// How much the overlay's RAM window decodes: `$60_0000`-`$7F_FFFF`. The SCC
/// is at `$80_0000` and SCSI below at `$58_0000`, so this is the whole of the
/// gap the decoder can put RAM in.
pub const HIGH_WINDOW: u64 = 0x20_0000;

/// One of the two windows, and the two things it can be pointed at.
#[derive(Debug)]
struct Window {
    /// How many bytes it decodes.
    len: u64,
    /// `true` when this window answers out of the ROM while the overlay is up.
    /// The low window does; the high one answers out of RAM.
    rom_when_overlaid: bool,
    /// The overlay bit, shared with the other window. An atomic rather than a
    /// lock: this is read on every instruction fetch the processor makes.
    overlay: Arc<AtomicBool>,
    /// The ROM and the RAM, each in a private space of this object's own at
    /// base zero. `None` until bind.
    rom: Mutex<Option<Source_>>,
    ram: Mutex<Option<Source_>>,
}

/// A resolved source: a region in a private space of this decoder's own.
///
/// Private, so a later retopology of the board's map is not seen through the
/// decoder and no foreign space's read guard is held inside an MMIO handler.
/// The space's `unassigned` policy is `open-bus`, which is what makes an
/// address above the installed memory **float** rather than fault — see
/// [`Glue::attach`] for why that is the whole point.
type Source_ = Arc<AddressSpace>;

impl Window {
    /// The memory this window answers out of now, or `None` when nothing is
    /// decoded here and the address floats.
    ///
    /// Cloned out so the leaf lock is released before the forwarded access
    /// takes a topology guard.
    ///
    /// Four cases, written out rather than folded together because folding
    /// them is how the `$60_0000` window came to answer out of the ROM:
    ///
    /// ```text
    ///   window  overlay asserted   overlay cleared
    ///   low     the ROM            main memory
    ///   high    main memory        nothing
    /// ```
    fn resolve(&self) -> Option<Source_> {
        let overlaid = self.overlay.load(Ordering::Relaxed);
        match (self.rom_when_overlaid, overlaid) {
            (true, true) => self.rom.lock().clone(),
            (true, false) | (false, true) => self.ram.lock().clone(),
            (false, false) => None,
        }
    }
}

impl MemOps for Window {
    fn read(&self, offset: u64, dst: &mut [u8], attrs: MemAttrs) -> MemResult {
        let Some(src) = self.resolve() else {
            return Err(BusError::Unassigned);
        };
        // `attrs` travels unchanged, `MemAttrs::debug` included: a decoder has
        // no state to disturb and the memory behind it makes its own decision.
        src.read_bytes(offset, dst, attrs)
    }

    fn write(&self, offset: u64, bytes: &[u8], attrs: MemAttrs) -> MemResult {
        let Some(src) = self.resolve() else {
            return Err(BusError::Unassigned);
        };
        src.write_bytes(offset, bytes, attrs)
    }

    fn constraints(&self) -> AccessConstraints {
        // Whatever is on the far side decides. A decoder that imposed a width
        // of its own would refuse accesses the memory behind it accepts, and
        // the byte order is the board's to declare on the `map` statement.
        AccessConstraints::ANY
    }
}

/// The `ROMOVERLAY` input.
#[derive(Debug)]
struct OverlayPin {
    overlay: Arc<AtomicBool>,
    inputs: FanIn,
}

impl WireSink for OverlayPin {
    fn set_level(&self, src: WireId, _line: u32, level: Level) {
        self.inputs.set(src, level);
        // Active high: the Guide's port table has `ROMOVERLAY` set meaning the
        // ROM answers at zero, and the VIA's port A is all inputs out of reset
        // so the pull-up holds it there before any code has run.
        self.overlay
            .store(self.inputs.any_high(), Ordering::Relaxed);
    }
}

/// The Macintosh address decoder.
#[derive(Debug)]
pub struct Glue {
    overlay: Arc<AtomicBool>,
    low: Arc<Window>,
    high: Arc<Window>,
    low_region: RegionRef,
    high_region: RegionRef,
    /// The objects named by `rom` and `ram`, resolved at bind.
    rom_path: String,
    ram_path: String,
    /// The pin, kept alive here: a net holds only a `Weak` to its sinks
    /// (`ROADMAP.md` §4.3).
    pin: Mutex<Option<Arc<OverlayPin>>>,
}

impl Glue {
    /// Build the decoder.
    ///
    /// # Errors
    ///
    /// [`Error::Property`] if `rom` or `ram` is missing or of the wrong kind,
    /// or if a property this class does not know was given.
    pub fn new(props: &Props) -> Result<Glue> {
        let mut r = props.reader();
        let rom_path = r.require_link("rom")?.as_str().to_string();
        let ram_path = r.require_link("ram")?.as_str().to_string();
        r.finish()?;
        // Asserted out of reset, which is what lets the processor find a reset
        // vector in a machine whose RAM holds nothing.
        let overlay = Arc::new(AtomicBool::new(true));
        let window = |len, rom_when_overlaid| {
            Arc::new(Window {
                len,
                rom_when_overlaid,
                overlay: Arc::clone(&overlay),
                rom: Mutex::with_rank(LockRank::LEAF, None),
                ram: Mutex::with_rank(LockRank::LEAF, None),
            })
        };
        let low = window(LOW_WINDOW, true);
        let high = window(HIGH_WINDOW, false);
        let low_region = Arc::new(Region::io(
            "mac.glue.low",
            low.len,
            Arc::clone(&low) as Arc<dyn MemOps>,
        ));
        let high_region = Arc::new(Region::io(
            "mac.glue.high",
            high.len,
            Arc::clone(&high) as Arc<dyn MemOps>,
        ));
        Ok(Glue {
            overlay,
            low,
            high,
            low_region,
            high_region,
            rom_path,
            ram_path,
            pin: Mutex::with_rank(LockRank::LEAF, None),
        })
    }

    /// Whether the ROM is what answers at zero.
    #[must_use]
    pub fn overlaid(&self) -> bool {
        self.overlay.load(Ordering::Relaxed)
    }

    /// Point the decoder at the memory a machine file named, once the machine
    /// layer has resolved it.
    ///
    /// Public for the reason [`Glue::new`] is separate from realize: a test
    /// builds the decoder, hands it two regions and drives it, without a
    /// machine file.
    ///
    /// # Errors
    ///
    /// If the region cannot be mapped into the private space this decoder
    /// forwards through.
    pub fn attach(&self, rom: &RegionRef, ram: &RegionRef, bits: u32) -> Result<()> {
        for (which, region) in [("rom", rom), ("ram", ram)] {
            if region.is_empty() {
                return Err(Error::Config {
                    at: String::from(CLASS_NAME),
                    message: format!("`{which}` names a memory object with no bytes in it"),
                });
            }
            let space = AddressSpace::new(format!("{CLASS_NAME}.{which}"), bits)
                .with_unassigned(UnassignedPolicy::OPEN_BUS);
            if which == "rom" {
                // The ROM socket carries no address line above its own, so a
                // 128 KiB part answers all through the window its select
                // decodes. A mirror is how that is said here; a ROM as big as
                // the window needs none.
                if region.len() >= LOW_WINDOW {
                    space.topology().map(Arc::clone(region), 0)?;
                } else if LOW_WINDOW.is_multiple_of(region.len()) {
                    let mirror = Region::mirror(
                        format!("{CLASS_NAME}.rom-mirror"),
                        Arc::clone(region),
                        LOW_WINDOW,
                    )?;
                    space.topology().map(Arc::new(mirror), 0)?;
                } else {
                    return Err(Error::Config {
                        at: String::from(CLASS_NAME),
                        message: format!(
                            "a {:#x}-byte rom cannot repeat evenly through the {LOW_WINDOW:#x} \
                             bytes the overlay decodes",
                            region.len()
                        ),
                    });
                }
            } else if region.len() < LOW_WINDOW && LOW_WINDOW.is_multiple_of(region.len()) {
                // **Main memory repeats through its whole window**, and that
                // is load-bearing — see the module docs. The DRAM gets only
                // the address lines its own depth needs, so a board with less
                // than four megabytes in it leaves `A20`/`A21` undecoded and
                // the memory answers again every `len` bytes.
                let mirror = Region::mirror(
                    format!("{CLASS_NAME}.ram-mirror"),
                    Arc::clone(region),
                    LOW_WINDOW,
                )?;
                space.topology().map(Arc::new(mirror), 0)?;
            } else {
                // Four megabytes, or a size that does not divide the window:
                // it answers where it is and the rest floats.
                space.topology().map(Arc::clone(region), 0)?;
            }
            let source: Source_ = Arc::new(space);
            for window in [&self.low, &self.high] {
                let slot = if which == "rom" {
                    &window.rom
                } else {
                    &window.ram
                };
                *slot.lock() = Some(Arc::clone(&source));
            }
        }
        Ok(())
    }
}

impl Device for Glue {
    fn class(&self) -> &'static DeviceClass {
        &GLUE_CLASS
    }

    fn realize(&self, _ctx: &mut RealizeCtx<'_>) -> Result<()> {
        // Nothing outward: `map` statements place the regions and the wire
        // graph brings the pin.
        Ok(())
    }

    fn reset(&self, _kind: ResetKind) {
        // Both kinds. The processor is about to fetch its reset vector from
        // whatever is at zero, and a board whose overlay did not come back
        // would reset into empty RAM. The VIA re-announces its pin during the
        // post-reset sweep (§4.3) and may put it straight back down, which is
        // the right order.
        self.overlay.store(true, Ordering::Relaxed);
    }

    fn save(&self, w: &mut ChunkWriter<'_>) -> Result<()> {
        // The pin level, for the reason `amiga.gary` saves its: a restore does
        // not re-run the wire graph, so a decoder that forgot which way it was
        // pointing would come back with the ROM over a running system's vector
        // table.
        w.write_bool(self.overlaid())
    }

    fn load(&self, r: &mut ChunkReader<'_>) -> Result<()> {
        self.overlay.store(r.read_bool()?, Ordering::Relaxed);
        Ok(())
    }

    fn region(&self, name: &str) -> Option<RegionRef> {
        match name {
            "" | LOW_REGION => Some(Arc::clone(&self.low_region)),
            HIGH_REGION => Some(Arc::clone(&self.high_region)),
            _ => None,
        }
    }

    fn sink(&self, port: &str, sources: &[WireId]) -> Option<SinkPin> {
        if port != OVERLAY_PIN {
            return None;
        }
        let sink = Arc::new(OverlayPin {
            overlay: Arc::clone(&self.overlay),
            inputs: FanIn::new(sources),
        });
        *self.pin.lock() = Some(Arc::clone(&sink));
        Some(SinkPin { sink, line: 0 })
    }
}

/// The machine layer's half: the decoder has to be told what it decodes.
impl Instance for Glue {
    fn bind(&self, ctx: &BindCtx<'_>) -> Result<()> {
        let bits = ctx.space().map_or(32, |s| s.bits());
        let resolve = |which: &str, path: &str| {
            ctx.region(path, "").map_err(|e| Error::Config {
                at: ctx.path().to_string(),
                message: format!(
                    "`{which}` has to name a memory object this decoder can forward into: {e}"
                ),
            })
        };
        let rom = resolve("rom", &self.rom_path)?;
        let ram = resolve("ram", &self.ram_path)?;
        self.attach(&rom, &ram, bits)
    }
}

/// The `mac.glue` device class.
pub static GLUE_CLASS: DeviceClass = DeviceClass {
    name: CLASS_NAME,
    version: STATE_VERSION,
    summary: "the Macintosh address decode: the ROM overlay at zero and the RAM window above it",
    properties: &[
        PropertySpec {
            name: "rom",
            kind: ValueKind::Link,
            required: true,
            summary: "the memory object that answers at zero while the overlay is asserted",
        },
        PropertySpec {
            name: "ram",
            kind: ValueKind::Link,
            required: true,
            summary: "main memory, which answers at zero once the overlay is cleared",
        },
    ],
    construct: |props| Ok(Box::new(Glue::new(props)?)),
};

/// Add [`GLUE_CLASS`] to a registry.
///
/// # Errors
///
/// [`Error::Config`] if something already claimed the name.
pub fn register(registry: &mut crate::core::Registry) -> Result<()> {
    registry.add(&GLUE_CLASS)
}

/// Bind [`GLUE_CLASS`] into the machine graph.
///
/// # Errors
///
/// [`Error::Config`] if the class is already bound.
pub fn bind(bindings: &mut crate::machine::Bindings) -> Result<()> {
    bindings.bind(CLASS_NAME, |props| Ok(Arc::new(Glue::new(props)?)))
}

/// What the validator should know about `mac.glue`.
#[must_use]
pub fn schema() -> ClassSchema {
    ClassSchema::new(CLASS_NAME)
        .prop(PropSchema::new("rom", ValueKind::Link).required())
        .prop(PropSchema::new("ram", ValueKind::Link).required())
        .region("")
        .region(LOW_REGION)
        .region(HIGH_REGION)
        .port(OVERLAY_PIN, PortDir::In)
}

#[cfg(test)]
mod tests;
