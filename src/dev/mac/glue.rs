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
//! # The interrupt priority encoder, and how Apple's ROM proves it is there
//!
//! The other thing on this chip, and it was found rather than read. A
//! Macintosh has two interrupt sources — the VIA and the SCC — and a 68000
//! whose three `IPL` pins carry an *encoded level* rather than three separate
//! requests. The VIA is level 1 and the SCC is level 2, and both of those are
//! measurements: which autovector a real ROM takes, 25 for the VIA's
//! sixty-a-second tick chain and 26 for a carrier-detect change.
//!
//! What was wrong here was the sum. Wiring `/IRQ` straight to `IPL0` and
//! `/INT` straight to `IPL1` makes "both at once" **level 3**, and a level 3
//! that persists is a livelock on this machine:
//!
//! * The ROM's own vector table, which it builds in RAM, puts the level-3
//!   autovector (vector 27, at `$6C`) at `$401AB4`, and the word there is
//!   `$4E73` — `RTE`, per the MC68000 user's manual's instruction encodings.
//!   The whole handler is "return".
//! * The level-2 handler runs its entire length at `SR = $2200`, mask 2,
//!   measured by sampling `SR` through it. It never raises the mask.
//!
//! So: the SCC asks, the processor enters the level-2 handler at mask 2, the
//! VIA asks while it is in there, `IPL` becomes 3, the level-3 exception is
//! taken, `RTE` returns to mask 2 with level 3 still asserted, and it is taken
//! again — for ever, the stack frame pushed and popped in place. Measured, with
//! a mouse moving: `PC` pinned at `$401AB4`, `SR = $2300`, `A7` never moving,
//! the frame at `A7` reading `$2200 / $00401A88`, and the 60 Hz tick chain
//! stopped. An `RTE` at that vector is only a safe thing for Apple to have
//! shipped if **level 3 cannot be asserted**, so the board must encode:
//!
//! ```text
//!   SCC   VIA   IPL1  IPL0   level
//!    -     -     0     0       0
//!    -     x     0     1       1     the VIA
//!    x     -     1     0       2     the SCC
//!    x     x     1     0       2     the SCC, with the VIA still waiting
//! ```
//!
//! which is an ordinary priority encoder with the SCC on the higher input, and
//! is why `cpu.m68k`'s own documentation says "a board with a priority encoder
//! wires all three". The VIA's request is not lost: it is still asserted when
//! the level-2 handler clears the SCC, and the level falls to 1 rather than to
//! 0.
//!
//! It lives on this object because this object *is* the board's glue. Nothing
//! about the boot changes: with no mouse the SCC never asks, so `IPL0` is the
//! VIA and `IPL1` is zero, exactly what the two direct wires gave.
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
use alloc::vec::Vec;
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
use crate::core::wire::{Drive, FanIn, Level, WireId, WireSink, WireSource};
use crate::machine::realize::{BindCtx, Instance};
use crate::machine::validate::{ClassSchema, PortDir, PropSchema};

/// The class name a machine description writes.
pub const CLASS_NAME: &str = "mac.glue";

/// The snapshot chunk version. Bump with the encoding, never on its own.
///
/// 2 added the two interrupt inputs, which are pin levels and are saved for
/// the reason the overlay's is.
pub const STATE_VERSION: u32 = 2;

/// The input pin the VIA's `PA4` drives.
pub const OVERLAY_PIN: &str = "overlay";

/// The input pin the VIA's `/IRQ` drives: the level-1 source.
pub const VIA_IRQ_PIN: &str = "via_irq";

/// The input pin the SCC's `/INT` drives: the level-2 source.
pub const SCC_IRQ_PIN: &str = "scc_irq";

/// The output pins carrying the encoded level to the processor, bit 0 first.
pub const IPL_PINS: [&str; 2] = ["ipl0", "ipl1"];

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
    /// Whether the overlay, once cleared, stays cleared until a reset. See
    /// [`Mode`].
    latching: bool,
    inputs: FanIn,
}

impl WireSink for OverlayPin {
    fn set_level(&self, src: WireId, _line: u32, level: Level) {
        self.inputs.set(src, level);
        // Active high: the Guide's port table has `ROMOVERLAY` set meaning the
        // ROM answers at zero, and the VIA's port A is all inputs out of reset
        // so the pull-up holds it there before any code has run.
        let high = self.inputs.any_high();
        if self.latching && high && !self.overlay.load(Ordering::Relaxed) {
            // Cleared once is cleared for good — see [`Mode::Latching`].
            return;
        }
        self.overlay.store(high, Ordering::Relaxed);
    }
}

/// What a *rising* edge on the overlay pin does.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Mode {
    /// The pin is the decode: raise it and the ROM is back at zero. A
    /// Macintosh Plus, which never raises it again after startup.
    Level,
    /// The pin clears the overlay once and cannot put it back. A Macintosh
    /// Classic, **measured**: its ROM raises `PA4` again 5.4 virtual seconds
    /// into startup, while it is working through the disk controller, and on a
    /// board that took that as "the ROM is back at zero" the machine lost its
    /// own exception vector table mid-instruction — the next `RTS` popped a
    /// return address out of the ROM's header and the processor left the map.
    /// `docs/platforms/mac-classic.md` carries the trace and the argument.
    Latching,
}

impl Mode {
    /// The name a machine file writes.
    #[must_use]
    pub fn name(self) -> &'static str {
        match self {
            Mode::Level => "level",
            Mode::Latching => "latching",
        }
    }
}

/// The two interrupt inputs and the two `IPL` outputs.
///
/// See the module docs: the level is encoded rather than summed, because
/// Apple's ROM answers level 3 with a bare `RTE`.
#[derive(Debug, Default)]
struct Encoder {
    via: AtomicBool,
    scc: AtomicBool,
    out: Mutex<[Option<WireSource>; 2]>,
}

impl Encoder {
    /// The level the processor sees: the higher of the two sources.
    fn level(&self) -> u8 {
        if self.scc.load(Ordering::Relaxed) {
            2
        } else if self.via.load(Ordering::Relaxed) {
            1
        } else {
            0
        }
    }

    /// Drive both pins, holding no lock across the call outward.
    ///
    /// `drive` rather than `set`, for the reason `mac.video` gives about its
    /// blanking pins: a fresh source already rests at `Level::Low`, so
    /// `set(Low)` is not a change, never reaches the processor, and leaves the
    /// realize sweep with nothing to announce.
    fn refresh(&self) {
        let level = self.level();
        let out = self.out.lock().clone();
        for (bit, src) in out.iter().enumerate() {
            if let Some(src) = src {
                src.drive(Drive::strong(Level::from(level & (1 << bit) != 0)));
            }
        }
    }
}

/// One of the two interrupt inputs.
#[derive(Debug)]
struct IrqPin {
    encoder: Arc<Encoder>,
    /// Which source: `false` the VIA, `true` the SCC.
    scc: bool,
    inputs: FanIn,
}

impl WireSink for IrqPin {
    fn set_level(&self, src: WireId, _line: u32, level: Level) {
        self.inputs.set(src, level);
        let asserting = self.inputs.any_high();
        let held = if self.scc {
            &self.encoder.scc
        } else {
            &self.encoder.via
        };
        held.store(asserting, Ordering::Relaxed);
        // State first, then outward. Both halves are atomics, so there is no
        // critical section to leave (`CLAUDE.md`, re-entrancy).
        self.encoder.refresh();
    }
}

/// The Macintosh address decoder.
#[derive(Debug)]
pub struct Glue {
    overlay: Arc<AtomicBool>,
    /// What a rising edge on the overlay pin does.
    mode: Mode,
    low: Arc<Window>,
    high: Arc<Window>,
    low_region: RegionRef,
    high_region: RegionRef,
    /// The objects named by `rom` and `ram`, resolved at bind.
    rom_path: String,
    ram_path: String,
    /// The interrupt priority encoder.
    encoder: Arc<Encoder>,
    /// The pins, kept alive here: a net holds only a `Weak` to its sinks
    /// (`ROADMAP.md` §4.3).
    pin: Mutex<Option<Arc<OverlayPin>>>,
    irq_pins: Mutex<Vec<Arc<IrqPin>>>,
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
        let mode = match r.or_str("overlay", Mode::Level.name())? {
            "level" => Mode::Level,
            "latching" => Mode::Latching,
            other => {
                return Err(Error::Property(format!(
                    "property `overlay`: `level` (the pin is the decode) or `latching` (cleared \
                     once is cleared for good), not `{other}`"
                )));
            }
        };
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
            mode,
            low,
            high,
            low_region,
            high_region,
            rom_path,
            ram_path,
            encoder: Arc::new(Encoder {
                via: AtomicBool::new(false),
                scc: AtomicBool::new(false),
                out: Mutex::with_rank(LockRank::WIRE, [None, None]),
            }),
            pin: Mutex::with_rank(LockRank::LEAF, None),
            irq_pins: Mutex::with_rank(LockRank::LEAF, Vec::new()),
        })
    }

    /// The interrupt level the encoder is presenting to the processor, 0 to 2.
    ///
    /// For a test with no wire graph; see the module docs for why it is a
    /// level rather than a sum.
    #[must_use]
    pub fn interrupt_level(&self) -> u8 {
        self.encoder.level()
    }

    /// Assert or release one of the two interrupt inputs, for a test with no
    /// wire graph. `scc` picks which.
    pub fn set_interrupt(&self, scc: bool, asserting: bool) {
        let held = if scc {
            &self.encoder.scc
        } else {
            &self.encoder.via
        };
        held.store(asserting, Ordering::Relaxed);
        self.encoder.refresh();
    }

    /// What a rising edge on the overlay pin does.
    #[must_use]
    pub fn mode(&self) -> Mode {
        self.mode
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
        // The pin levels, for the reason `amiga.gary` saves its: a restore does
        // not re-run the wire graph, so a decoder that forgot which way it was
        // pointing would come back with the ROM over a running system's vector
        // table — and one that forgot which sources were asking would present
        // the wrong level until the next edge.
        w.write_bool(self.overlaid())?;
        w.write_bool(self.encoder.via.load(Ordering::Relaxed))?;
        w.write_bool(self.encoder.scc.load(Ordering::Relaxed))
    }

    fn load(&self, r: &mut ChunkReader<'_>) -> Result<()> {
        self.overlay.store(r.read_bool()?, Ordering::Relaxed);
        self.encoder.via.store(r.read_bool()?, Ordering::Relaxed);
        self.encoder.scc.store(r.read_bool()?, Ordering::Relaxed);
        self.encoder.refresh();
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
        if port == VIA_IRQ_PIN || port == SCC_IRQ_PIN {
            let pin = Arc::new(IrqPin {
                encoder: Arc::clone(&self.encoder),
                scc: port == SCC_IRQ_PIN,
                inputs: FanIn::new(sources),
            });
            self.irq_pins.lock().push(Arc::clone(&pin));
            return Some(SinkPin {
                sink: pin,
                line: u32::from(port == SCC_IRQ_PIN),
            });
        }
        if port != OVERLAY_PIN {
            return None;
        }
        let sink = Arc::new(OverlayPin {
            overlay: Arc::clone(&self.overlay),
            latching: self.mode == Mode::Latching,
            inputs: FanIn::new(sources),
        });
        *self.pin.lock() = Some(Arc::clone(&sink));
        Some(SinkPin { sink, line: 0 })
    }

    fn connect(&self, port: &str, source: WireSource) -> Result<()> {
        let Some(bit) = IPL_PINS.iter().position(|p| *p == port) else {
            return Err(Error::Config {
                at: port.to_string(),
                message: String::from("the Macintosh glue drives `ipl0` and `ipl1`"),
            });
        };
        self.encoder.out.lock()[bit] = Some(source);
        // No refresh here, and `mac.video` says why: a net has no sinks yet
        // when its source is handed out, so a level driven now goes nowhere,
        // and having driven it the realize sweep's `announce` would see no
        // change and deliver nothing either.
        Ok(())
    }

    fn announce(&self, _port: &str) {
        self.encoder.refresh();
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
        PropertySpec {
            name: "overlay",
            kind: ValueKind::Str,
            required: false,
            summary: "`level` (default): the pin is the decode. `latching`: cleared once is \
                      cleared until a reset",
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
        .prop(PropSchema::new("overlay", ValueKind::Str))
        .region("")
        .region(LOW_REGION)
        .region(HIGH_REGION)
        .port(OVERLAY_PIN, PortDir::In)
        .port(VIA_IRQ_PIN, PortDir::In)
        .port(SCC_IRQ_PIN, PortDir::In)
        .port(IPL_PINS[0], PortDir::Out)
        .port(IPL_PINS[1], PortDir::Out)
}

#[cfg(test)]
mod tests;
