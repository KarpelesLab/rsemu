//! The `OVL` overlay: what answers at address zero.
//!
//! One class, `amiga.gary`, named for the A500/A2000 gate array that does the
//! board's address decoding. It models exactly one of that part's jobs, and it
//! is the one a 68000 cannot start without.
//!
//! # The problem the overlay solves
//!
//! A 68000 comes out of reset by reading two longwords from `$000000` and
//! `$000004` — the supervisor stack pointer and the program counter (M68000
//! User's Manual, *Reset Operation*). An Amiga has RAM there, and RAM out of
//! power-on holds nothing. The Kickstart ROM lives at the top of the map.
//!
//! So at reset the board *also* decodes the ROM at zero. Software that has
//! finished with it clears `OVL` and chip RAM takes over the addresses, which
//! is where the exception vector table then lives for the rest of the run.
//!
//! `OVL` is bit 0 of CIA-A's port A: the hardware manual's 8520 appendix lists
//! `PA0` as "OVL memory overlay bit".
//!
//! ```text
//!   wire cia_a.pa0 -> gary.ovl
//! ```
//!
//! **The polarity is active high**, and the name is worth getting right because
//! it is usually written `OVL`. The same table stars every active-low line
//! beside it — `LED*`, `CHNG*`, `WPRO*`, `TK0*`, `RDY*` — and does not star
//! `OVL`. So a high level on `ovl` means the ROM is at zero, and clearing the
//! bit is what hands the addresses to chip RAM.
//!
//! This decoder comes up overlaid at every reset whatever is on the pin, because
//! the processor's first two fetches depend on it. Why the real pin is high at
//! that moment is the CIA's business — an 8520's ports come out of reset as
//! inputs, and what the board then pulls the line to is on the schematic, not in
//! the manual — and when the CIA re-announces its pin after the reset, its level
//! wins.
//!
//! # A decoder, not a remap
//!
//! This is an address **decoder**: one region at zero, whose accesses are
//! forwarded to the ROM or to chip RAM depending on the level on `ovl`. It is
//! not a pair of mappings that get swapped.
//!
//! The reason is the same one `st.syscfg`'s boot alias gives, and it is worth
//! repeating because the alternative looks obviously simpler. The write that
//! clears `OVL` is a store into a CIA register; that store arrives *through*
//! an address space, so the topology lock is already held for reading on this
//! very thread and
//! [`AddressSpace::topology`](crate::core::space::AddressSpace::topology) would
//! deadlock against it. The documented escape —
//! [`Deferred`](crate::core::device::Deferred) — runs after a scheduler event,
//! which is a whole quantum after the instruction that was supposed to change
//! what address zero means. A decoder has neither problem, and it is also what
//! the silicon is: Gary is combinational address decoding, not a second copy of
//! the ROM.
//!
//! The cost is real and worth writing down: every chip-RAM access on this board
//! goes through a [`MemOps`] call and a second space lookup instead of the
//! RAM host-pointer fast path, and an Amiga runs essentially all of its code out
//! of chip RAM. If that shows up in a profile, the fix is a **safe-point
//! retopology** the first time `OVL` goes low (`CLAUDE.md`, *Concurrency*) —
//! the flip happens once in a boot, so paying for a flatten is nothing, and the
//! only thing to argue about is whether deferring it to the next safe point can
//! be observed. It has not been done here because "correct and slow" is the
//! right order.
//!
//! # How it finds the two memories
//!
//! Two link-valued properties naming the objects:
//!
//! ```text
//!   object chipram "ram" { size = 512K }
//!   object kick    "rom" { size = 512K, image = "kickstart" }
//!   object gary "amiga.gary" { rom = kick, ram = chipram, size = 512K }
//!
//!   map mem 0x000000 size 512K = gary.overlay { endian = "big" }
//! ```
//!
//! Each is resolved at bind with
//! [`BindCtx::region`](crate::machine::BindCtx::region) — the object's own
//! region, the one a `map` statement naming it would place. It is *not* resolved
//! by address, the way `st.syscfg` resolves its `boot-sources`, and that is the
//! whole difficulty: chip RAM's only address on an Amiga is zero, and zero is
//! also this decoder's own address. There is nothing to look up.
//!
//! Each region is mapped into a private space of this object's own, so a later
//! retopology of the board's map is not seen through the overlay and no foreign
//! space's read guard is held inside an MMIO handler.
//!
//! # A ROM smaller than the window: a 256 KiB Kickstart
//!
//! Kickstart 1.x is 256 KiB; the window at zero is as big as chip RAM, 512 KiB
//! on a stock A500. The ROM **repeats** through it, every 256 KiB, and the
//! documents say so from three directions:
//!
//! * **The select decodes a 512 KiB window.** The A2000's PAL listing — the
//!   logic Gary replaced (*A500/A2000 Technical Reference Manual*, Commodore,
//!   §7.3, `/ROME`) — enables the ROM for a read at `$F8_0000`–`$FF_FFFF`, and
//!   at `$00_0000`–`$07_FFFF` while `OVL` is high: `A19`–`A23` and nothing
//!   below them.
//! * **The ROM socket carries `A1`–`A18`, and a 256 KiB part decodes 17 of
//!   them.** A500 schematic #312511-03 rev. 6A/7, sheet 3: `U6`, a "62402", has
//!   `A0`–`A16` on the processor's `A1`–`A17`, the processor's `A18` on pin 1
//!   (`A17`), `/CS` grounded and `/OE` on Gary's `_ROMEN`. Rev. 5 (#312511-02,
//!   sheet 3) fits an HN62402 "128K × 16 ROM" in the same place. A part with
//!   one address pin fewer ignores `A18`, so it answers in both halves of the
//!   window its select decodes.
//! * **It could not be otherwise and boot.** Kickstart 1.x is built to run at
//!   `$FC_0000` — Appendix D's "256K System ROM" — where `A18` is high; the
//!   processor's first two fetches are at `$00_0000` and `$00_0004`, where it
//!   is low. A ROM that decoded `A18` would be absent from one or the other.
//!
//! So the ROM side of the overlay is the ROM **mirrored across the first
//! 512 KiB** ([`ROM_WINDOW`]), and a 512 KiB ROM mirrored once is itself.
//! Past `ROM_WINDOW` — a 1 MiB chip-RAM board with `OVL` still up — the same
//! PAL selects neither the ROM nor RAM (`/RE` wants `OVL` low), so that part of
//! the window answers like any other empty address: the private space floats
//! (the HRM's own words for the reset state, p. 223: "On other models, no RAM
//! responds").
//!
//! The board's own ROM mapping has the same shape, which is the machine file's
//! business and not this decoder's: `map mem 0xF80000 size 512K = mirror(kick)`
//! puts a 256 KiB part at both `$F8_0000` and `$FC_0000`, exactly as `_ROMEN`
//! does.
//!
//! # What is *not* modelled
//!
//! Gary does a great deal more than this — bus arbitration between the
//! processor and the chipset, among other things — and none of it is here. This class is the
//! overlay and nothing else, and it is named `amiga.gary` because that is the
//! part the overlay lives in, not because the part is modelled.
//!
//! # Sources
//!
//! *Amiga Hardware Reference Manual*, Commodore-Amiga Inc., 3rd edition,
//! Appendix D ("System Memory Maps") for the map and p. 223 for the reset
//! state; the M68000 User's Manual for the reset sequence; the *A500/A2000
//! Technical Reference Manual* §7.3 and A500 schematics #312511-02 and
//! #312511-03 for how a 256 KiB ROM sits in the window. No emulator source of
//! any licence was consulted (`ROADMAP.md` §1).

use alloc::boxed::Box;
use alloc::format;
use alloc::string::{String, ToString};
use alloc::sync::Arc;

use crate::core::device::{Device, DeviceClass, PropertySpec, RealizeCtx, ResetKind, SinkPin};
use crate::core::error::{BusError, Error, Result};
use crate::core::props::{Props, ValueKind};
use crate::core::space::{
    AccessConstraints, AddressSpace, MemAttrs, MemOps, MemResult, Region, RegionRef,
    UnassignedPolicy,
};
use crate::core::state::{ChunkReader, ChunkWriter, Sink, Source};
use crate::core::sync::{AtomicBool, Ordering};
use crate::core::wire::{FanIn, Level, WireId, WireSink};
use crate::machine::realize::{BindCtx, Instance};
use crate::machine::validate::{ClassSchema, PortDir, PropSchema};

/// The class name a machine file writes.
pub const CLASS_NAME: &str = "amiga.gary";

/// Snapshot version for this class's chunk encoding.
const STATE_VERSION: u32 = 1;

/// The name of the input pin the CIA drives.
pub const OVL_PIN: &str = "ovl";

/// The name of the region a `map` statement places at address zero.
pub const OVERLAY_REGION: &str = "overlay";

/// How much of the window the ROM's select covers: `$00_0000`–`$07_FFFF`, the
/// `A19`–`A23` decode of the A2000 PAL's `/ROME` term. A smaller ROM repeats
/// through it; see the module documentation.
pub const ROM_WINDOW: u64 = 512 * 1024;

// ---------------------------------------------------------------------------
// the decoder
// ---------------------------------------------------------------------------

/// The window at address zero, and the two things it can be pointed at.
#[derive(Debug)]
struct Overlay {
    /// How many bytes it decodes.
    len: u64,
    /// `OVL` as the CIA drives it: true is asserted, which is the ROM.
    ///
    /// An atomic rather than a lock: this is read on every instruction fetch
    /// the processor makes before the overlay is cleared, and on every chip-RAM
    /// access afterwards.
    ovl: AtomicBool,
    /// The Kickstart ROM, in a space of this object's own. `None` until bind.
    rom: Mutexed,
    /// Chip RAM, likewise.
    ram: Mutexed,
}

/// A resolved source: one region, in a private space, at base zero.
///
/// Named rather than inlined so the two fields read the same way.
type Mutexed = crate::core::sync::Mutex<Option<Arc<AddressSpace>>>;

impl Overlay {
    fn new(len: u64) -> Overlay {
        use crate::core::sync::LockRank;
        Overlay {
            len,
            // Asserted out of reset: the CIA's port A is all inputs and the
            // line is pulled up, so a board that has not run any code yet has
            // the ROM at zero. This is what makes the processor able to start.
            ovl: AtomicBool::new(true),
            rom: Mutexed::with_rank(LockRank::LEAF, None),
            ram: Mutexed::with_rank(LockRank::LEAF, None),
        }
    }

    /// The space an access should be forwarded into, cloned out so the leaf
    /// lock is released before the forwarded access takes a `TOPOLOGY` guard.
    fn target(&self) -> Option<Arc<AddressSpace>> {
        if self.ovl.load(Ordering::Relaxed) {
            self.rom.lock().clone()
        } else {
            self.ram.lock().clone()
        }
    }
}

impl MemOps for Overlay {
    fn read(&self, offset: u64, dst: &mut [u8], attrs: MemAttrs) -> MemResult {
        // `attrs` travels unchanged, `MemAttrs::debug` included: a decoder has
        // no state to disturb and the memory behind it makes its own decision.
        let Some(space) = self.target() else {
            return Err(BusError::Unassigned);
        };
        space.read_bytes(offset, dst, attrs)
    }

    fn write(&self, offset: u64, src: &[u8], attrs: MemAttrs) -> MemResult {
        let Some(space) = self.target() else {
            return Err(BusError::Unassigned);
        };
        space.write_bytes(offset, src, attrs)
    }

    fn constraints(&self) -> AccessConstraints {
        // Whatever is on the far side decides. A decoder that imposed a width
        // of its own would refuse accesses the memory behind it accepts, and
        // the byte order is the board's to declare on the `map` statement.
        AccessConstraints::ANY
    }
}

// ---------------------------------------------------------------------------
// the pin
// ---------------------------------------------------------------------------

/// The `OVL` input.
#[derive(Debug)]
struct OvlPin {
    overlay: Arc<Overlay>,
    inputs: FanIn,
}

impl WireSink for OvlPin {
    fn set_level(&self, src: WireId, _line: u32, level: Level) {
        // Wired-or across whatever drives the net, which is what `FanIn` is
        // for; on a real board it is one CIA pin and nothing else.
        self.inputs.set(src, level);
        self.overlay
            .ovl
            .store(self.inputs.any_high(), Ordering::Relaxed);
    }
}

// ---------------------------------------------------------------------------
// the device
// ---------------------------------------------------------------------------

/// The `OVL` overlay decoder.
#[derive(Debug)]
pub struct Gary {
    overlay: Arc<Overlay>,
    region: RegionRef,
    /// The objects named by `rom` and `ram`, resolved at bind.
    rom_path: String,
    ram_path: String,
    /// The pin, kept alive here: a net holds only a `Weak` to its sinks
    /// (`ROADMAP.md` §4.3).
    pin: crate::core::sync::Mutex<Option<Arc<OvlPin>>>,
}

impl Gary {
    /// Build the decoder.
    ///
    /// # Errors
    ///
    /// [`Error::Property`] if `rom`, `ram` or `size` is missing or of the wrong
    /// kind, if `size` is zero, or if a property nothing here accepts was
    /// given.
    pub fn new(props: &Props) -> Result<Gary> {
        use crate::core::sync::LockRank;
        let mut r = props.reader();
        let rom_path = r.require_link("rom")?.as_str().to_string();
        let ram_path = r.require_link("ram")?.as_str().to_string();
        let size = r.require_size("size")?;
        r.finish()?;
        if size == 0 {
            return Err(Error::Property(String::from(
                "property `size`: an overlay that decodes no bytes cannot be mapped",
            )));
        }
        let overlay = Arc::new(Overlay::new(size));
        let region = Arc::new(Region::io(
            "amiga.gary.overlay",
            size,
            Arc::clone(&overlay) as Arc<dyn MemOps>,
        ));
        Ok(Gary {
            overlay,
            region,
            rom_path,
            ram_path,
            pin: crate::core::sync::Mutex::with_rank(LockRank::LEAF, None),
        })
    }

    /// Whether `OVL` is asserted — whether the ROM is what answers at zero.
    #[must_use]
    pub fn overlaid(&self) -> bool {
        self.overlay.ovl.load(Ordering::Relaxed)
    }

    /// How many bytes the window decodes.
    #[must_use]
    pub fn len(&self) -> u64 {
        self.overlay.len
    }

    /// Whether it decodes none — it never does; `new` refuses a zero size.
    #[must_use]
    pub fn is_empty(&self) -> bool {
        self.overlay.len == 0
    }

    /// Give the decoder a source region, in a private space of its own.
    ///
    /// Public so a test or an embedder that builds a machine by hand can do
    /// what `bind` does from a machine file.
    ///
    /// Chip RAM is mapped as it is. The ROM is repeated through the first
    /// [`ROM_WINDOW`] bytes of the window, because a part with fewer address
    /// pins than the socket ignores the top ones; anything of the window past
    /// that floats while `OVL` is up.
    ///
    /// # Errors
    ///
    /// If chip RAM is shorter than the window, which would leave part of
    /// address zero decoding nothing once `OVL` drops; or if the ROM is empty,
    /// larger than [`ROM_WINDOW`], or a size that does not repeat evenly
    /// through it — a part ignoring its top address lines repeats at a power of
    /// two.
    pub fn attach(&self, which: Side, region: &RegionRef, bits: u32) -> Result<()> {
        let refuse = |why: String| Error::Config {
            at: String::from(CLASS_NAME),
            message: why,
        };
        let space = AddressSpace::new(format!("{CLASS_NAME}.{}", which.name()), bits)
            .with_unassigned(UnassignedPolicy::OPEN_BUS);
        match which {
            Side::Ram => {
                if region.len() < self.overlay.len {
                    return Err(refuse(format!(
                        "the overlay decodes {:#x} bytes and its ram source holds only {:#x}",
                        self.overlay.len,
                        region.len()
                    )));
                }
                space.topology().map(Arc::clone(region), 0)?;
            }
            Side::Rom => {
                let len = region.len();
                if len == 0 || len > ROM_WINDOW || !ROM_WINDOW.is_multiple_of(len) {
                    return Err(refuse(format!(
                        "a {len:#x}-byte rom cannot repeat evenly through the {ROM_WINDOW:#x} \
                         bytes its select decodes"
                    )));
                }
                let span = self.overlay.len.min(ROM_WINDOW);
                if len >= span {
                    space.topology().map(Arc::clone(region), 0)?;
                } else {
                    let mirror = Region::mirror(
                        format!("{CLASS_NAME}.rom-mirror"),
                        Arc::clone(region),
                        span,
                    )?;
                    space.topology().map(Arc::new(mirror), 0)?;
                }
            }
        }
        let slot = match which {
            Side::Rom => &self.overlay.rom,
            Side::Ram => &self.overlay.ram,
        };
        *slot.lock() = Some(Arc::new(space));
        Ok(())
    }
}

/// Which of the overlay's two memories is being named.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Side {
    /// The Kickstart ROM — what answers while `OVL` is asserted.
    Rom,
    /// Chip RAM — what answers once the CIA has cleared `OVL`.
    Ram,
}

impl Side {
    /// The machine-file property this source is named by.
    #[must_use]
    pub const fn name(self) -> &'static str {
        match self {
            Side::Rom => "rom",
            Side::Ram => "ram",
        }
    }
}

impl Device for Gary {
    fn class(&self) -> &'static DeviceClass {
        &CLASS
    }

    fn realize(&self, _ctx: &mut RealizeCtx<'_>) -> Result<()> {
        // Nothing outward: a `map` statement places the region and the wire
        // graph brings the pin.
        Ok(())
    }

    fn reset(&self, _kind: ResetKind) {
        // Both kinds. The processor is about to fetch its reset vector from
        // whatever is at zero, and a board whose overlay did not come back would
        // reset into empty RAM.
        //
        // The CIA will re-announce its pin during the post-reset sweep
        // (`ROADMAP.md` §4.3) and may put it straight back down; that is the
        // right order, because the level it publishes then is the level its own
        // reset left it at.
        self.overlay.ovl.store(true, Ordering::Relaxed);
    }

    fn save(&self, w: &mut ChunkWriter<'_>) -> Result<()> {
        // The pin level, for the reason `st.syscfg` saves its input latches: a
        // restore does not re-run the wire graph, so a decoder that forgot
        // which way it was pointing would come back with the ROM over a
        // running system's vector table.
        w.write_bool(self.overlaid())?;
        Ok(())
    }

    fn load(&self, r: &mut ChunkReader<'_>) -> Result<()> {
        let ovl = r.read_bool()?;
        self.overlay.ovl.store(ovl, Ordering::Relaxed);
        Ok(())
    }

    fn region(&self, name: &str) -> Option<RegionRef> {
        match name {
            "" | OVERLAY_REGION => Some(Arc::clone(&self.region)),
            _ => None,
        }
    }

    fn sink(&self, port: &str, sources: &[WireId]) -> Option<SinkPin> {
        if port != OVL_PIN {
            return None;
        }
        let sink = Arc::new(OvlPin {
            overlay: Arc::clone(&self.overlay),
            inputs: FanIn::new(sources),
        });
        *self.pin.lock() = Some(Arc::clone(&sink));
        Some(SinkPin { sink, line: 0 })
    }
}

/// The machine layer's half: the decoder has to be told what it decodes.
impl Instance for Gary {
    fn bind(&self, ctx: &BindCtx<'_>) -> Result<()> {
        let bits = ctx.space().map_or(32, |s| s.bits());
        for (which, path) in [(Side::Rom, &self.rom_path), (Side::Ram, &self.ram_path)] {
            let region = ctx.region(path, "").map_err(|e| Error::Config {
                at: ctx.path().to_string(),
                message: format!(
                    "`{}` has to name a memory object this decoder can forward into: {e}",
                    which.name()
                ),
            })?;
            self.attach(which, &region, bits)?;
        }
        Ok(())
    }
}

/// The `amiga.gary` device class.
pub static CLASS: DeviceClass = DeviceClass {
    name: CLASS_NAME,
    version: STATE_VERSION,
    summary: "the Amiga `OVL` overlay: the Kickstart ROM or chip RAM at address zero",
    properties: &[
        PropertySpec {
            name: "rom",
            kind: ValueKind::Link,
            required: true,
            summary: "the memory object that answers while `OVL` is asserted",
        },
        PropertySpec {
            name: "ram",
            kind: ValueKind::Link,
            required: true,
            summary: "the memory object that answers once `OVL` is cleared",
        },
        PropertySpec {
            name: "size",
            kind: ValueKind::Size,
            required: true,
            summary: "how many bytes the window at address zero decodes",
        },
    ],
    construct: |props| Ok(Box::new(Gary::new(props)?)),
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
    bindings.bind(CLASS_NAME, |props| Ok(Arc::new(Gary::new(props)?)))
}

/// What the validator should know about `amiga.gary`.
#[must_use]
pub fn schema() -> ClassSchema {
    ClassSchema::new(CLASS_NAME)
        .prop(PropSchema::new("rom", ValueKind::Link))
        .prop(PropSchema::new("ram", ValueKind::Link))
        .prop(PropSchema::new("size", ValueKind::Size))
        .region("")
        .region(OVERLAY_REGION)
        .port(OVL_PIN, PortDir::In)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::props::{Link, Value};
    use crate::core::space::{RamStore, RomStore, RomWrite};
    use crate::core::state::{MachineShape, Migrations, StateReader, StateWriter};
    use alloc::vec;
    use alloc::vec::Vec;

    const SIZE: u64 = 0x1000;

    fn props() -> Props {
        Props::new()
            .with("rom", Value::Link(Link::new("kick").unwrap()))
            .with("ram", Value::Link(Link::new("chipram").unwrap()))
            .with("size", Value::Size(SIZE))
    }

    /// A decoder with a ROM whose first bytes are a recognisable pattern and a
    /// blank RAM behind it.
    fn gary() -> Gary {
        let g = Gary::new(&props()).unwrap();
        let mut image = vec![0u8; SIZE as usize];
        image[..4].copy_from_slice(&[0xde, 0xad, 0xbe, 0xef]);
        let rom: RegionRef = Arc::new(Region::rom(
            "kick",
            Arc::new(RomStore::new(image)),
            RomWrite::Ignore,
        ));
        let ram: RegionRef = Arc::new(Region::ram("chipram", Arc::new(RamStore::new(SIZE))));
        g.attach(Side::Rom, &rom, 24).unwrap();
        g.attach(Side::Ram, &ram, 24).unwrap();
        g
    }

    fn peek(g: &Gary, offset: u64) -> [u8; 4] {
        let mut bytes = [0u8; 4];
        g.overlay
            .read(offset, &mut bytes, MemAttrs::DEFAULT)
            .unwrap();
        bytes
    }

    fn drive(g: &Gary, level: Level) {
        let src = WireId::new(7);
        let pin = g.sink(OVL_PIN, &[src]).expect("an `ovl` input");
        pin.sink.set_level(src, pin.line, level);
    }

    #[test]
    fn out_of_reset_the_rom_answers() {
        let g = gary();
        assert!(g.overlaid());
        assert_eq!(peek(&g, 0), [0xde, 0xad, 0xbe, 0xef]);
    }

    #[test]
    fn a_low_ovl_puts_the_ram_there_and_a_high_one_takes_it_away() {
        let g = gary();
        drive(&g, Level::Low);
        assert!(!g.overlaid());
        assert_eq!(peek(&g, 0), [0; 4]);
        g.overlay
            .write(0, &[1, 2, 3, 4], MemAttrs::DEFAULT)
            .unwrap();
        assert_eq!(peek(&g, 0), [1, 2, 3, 4]);

        drive(&g, Level::High);
        assert_eq!(peek(&g, 0), [0xde, 0xad, 0xbe, 0xef]);
        drive(&g, Level::Low);
        assert_eq!(peek(&g, 0), [1, 2, 3, 4], "nothing was copied or lost");
    }

    #[test]
    fn a_write_while_overlaid_goes_to_the_rom_and_is_dropped() {
        let g = gary();
        g.overlay
            .write(0, &[9, 9, 9, 9], MemAttrs::DEFAULT)
            .unwrap();
        assert_eq!(peek(&g, 0), [0xde, 0xad, 0xbe, 0xef]);
        drive(&g, Level::Low);
        assert_eq!(peek(&g, 0), [0; 4], "and did not reach the RAM");
    }

    #[test]
    fn a_reset_brings_the_overlay_back() {
        let g = gary();
        drive(&g, Level::Low);
        Device::reset(&g, ResetKind::Warm);
        assert!(g.overlaid());
    }

    #[test]
    fn an_unbound_decoder_faults_rather_than_answering_with_nothing() {
        let g = Gary::new(&props()).unwrap();
        let mut b = [0u8; 1];
        assert!(g.overlay.read(0, &mut b, MemAttrs::DEFAULT).is_err());
    }

    #[test]
    fn a_ram_shorter_than_the_window_is_refused() {
        let g = Gary::new(&props()).unwrap();
        let short: RegionRef = Arc::new(Region::ram("short", Arc::new(RamStore::new(SIZE - 1))));
        assert!(g.attach(Side::Ram, &short, 24).is_err());
    }

    /// A ROM of `len` bytes whose every longword is its own offset, so a read
    /// says which ROM byte answered.
    fn numbered_rom(len: u64) -> RegionRef {
        let mut image = vec![0u8; len as usize];
        for (i, chunk) in image.chunks_mut(4).enumerate() {
            chunk.copy_from_slice(&((i * 4) as u32).to_be_bytes());
        }
        Arc::new(Region::rom(
            "kick",
            Arc::new(RomStore::new(image)),
            RomWrite::Ignore,
        ))
    }

    fn sized(size: u64) -> Gary {
        Gary::new(&props().with("size", Value::Size(size))).unwrap()
    }

    #[test]
    fn a_256k_rom_repeats_through_a_512k_window() {
        // Kickstart 1.x on a stock A500: the build that used to fail.
        const K: u64 = 1024;
        let g = sized(512 * K);
        g.attach(Side::Rom, &numbered_rom(256 * K), 24).unwrap();
        let ram: RegionRef = Arc::new(Region::ram("chipram", Arc::new(RamStore::new(512 * K))));
        g.attach(Side::Ram, &ram, 24).unwrap();

        assert_eq!(peek(&g, 0x00_0004), 4u32.to_be_bytes(), "the reset PC");
        assert_eq!(
            peek(&g, 0x04_0004),
            4u32.to_be_bytes(),
            "A18 is not decoded by a 256 KiB part: the same byte at $040004"
        );
        assert_eq!(
            peek(&g, 0x07_fffc),
            0x3_fffcu32.to_be_bytes(),
            "the last word"
        );
        // And a longword read straddling the two copies wraps, as the pins do.
        let mut pair = [0u8; 8];
        g.overlay
            .read(0x03_fffc, &mut pair, MemAttrs::DEFAULT)
            .unwrap();
        assert_eq!(pair, [0, 3, 0xff, 0xfc, 0, 0, 0, 0]);
    }

    #[test]
    fn a_512k_rom_fills_the_window_once() {
        const K: u64 = 1024;
        let g = sized(512 * K);
        g.attach(Side::Rom, &numbered_rom(512 * K), 24).unwrap();
        assert_eq!(peek(&g, 0x04_0004), 0x4_0004u32.to_be_bytes());
    }

    #[test]
    fn past_the_rom_select_a_1m_window_floats_while_overlaid() {
        // A 1 MiB chip-RAM board: the ROM's select stops at $07FFFF, and RAM is
        // not selected until `OVL` drops.
        const K: u64 = 1024;
        let g = sized(1024 * K);
        g.attach(Side::Rom, &numbered_rom(512 * K), 24).unwrap();
        let ram: RegionRef = Arc::new(Region::ram("chipram", Arc::new(RamStore::new(1024 * K))));
        g.attach(Side::Ram, &ram, 24).unwrap();
        assert_eq!(peek(&g, 0x07_fffc), 0x7_fffcu32.to_be_bytes());
        assert_eq!(peek(&g, 0x08_0000), [0; 4], "nothing drives the bus");
        g.overlay
            .write(0x08_0000, &[1, 2, 3, 4], MemAttrs::DEFAULT)
            .unwrap();
        drive(&g, Level::Low);
        assert_eq!(
            peek(&g, 0x08_0000),
            [0; 4],
            "and a store there reached no RAM"
        );
    }

    #[test]
    fn a_rom_that_cannot_repeat_evenly_is_refused() {
        const K: u64 = 1024;
        let g = sized(512 * K);
        assert!(g.attach(Side::Rom, &numbered_rom(300 * K), 24).is_err());
        assert!(g.attach(Side::Rom, &numbered_rom(1024 * K), 24).is_err());
    }

    #[test]
    fn the_properties_are_checked() {
        assert_eq!(Gary::new(&props()).unwrap().len(), SIZE);
        assert!(Gary::new(&props().with("size", Value::Size(0))).is_err());
        assert!(Gary::new(&props().with("cia", Value::from(1u64))).is_err());
        let no_ram = Props::new()
            .with("rom", Value::Link(Link::new("kick").unwrap()))
            .with("size", Value::Size(SIZE));
        assert!(Gary::new(&no_ram).is_err());
    }

    #[test]
    fn the_region_and_the_pin_are_what_the_schema_says() {
        let g = gary();
        let schema = schema();
        assert!(schema.port_named(OVL_PIN).is_some());
        assert!(g.sink(OVL_PIN, &[WireId::new(1)]).is_some());
        assert!(g.sink("led", &[WireId::new(1)]).is_none());
        assert!(g.region("").is_some());
        assert!(g.region(OVERLAY_REGION).is_some());
        assert!(g.region("cia").is_none());
    }

    fn snapshot(g: &Gary) -> Vec<u8> {
        let mut shape = MachineShape::new();
        shape.add_device("gary", CLASS_NAME).unwrap();
        let mut w = StateWriter::new(shape);
        {
            let mut chunk = w.chunk("gary", CLASS_NAME, STATE_VERSION).unwrap();
            Device::save(g, &mut chunk).unwrap();
        }
        w.to_vec().unwrap()
    }

    #[test]
    fn a_snapshot_round_trips_to_identical_state() {
        let saved = gary();
        drive(&saved, Level::Low);
        let bytes = snapshot(&saved);

        let restored = gary();
        assert!(restored.overlaid());
        let reader = StateReader::new(&bytes).unwrap();
        let chunk = reader
            .load("gary", CLASS_NAME, STATE_VERSION, &Migrations::new())
            .unwrap();
        Device::load(&restored, &mut chunk.reader()).unwrap();

        assert_eq!(
            snapshot(&restored),
            bytes,
            "identical state after a round trip"
        );
        assert!(!restored.overlaid(), "the overlay came back cleared");
    }
}
