//! The chips an IBM PC/AT-class machine is built from.
//!
//! Everything here is a part somebody could buy: two 8259A interrupt
//! controllers, an 8254 timer, an 8042 keyboard controller, an MC146818 RTC,
//! two 8237A DMA controllers, a 6845-derived CRTC with a character generator in
//! front of it, and a µPD765 floppy controller — plus one thing that is not a
//! part at all: [`ide`], which is address decode and a cable, because an IDE
//! drive's controller is on the drive. None of it is PC-specific in itself —
//! the PC is the *wiring*, and that lives in `machines/pc-at.machine`, not in
//! Rust.
//!
//! # Sources
//!
//! Every file cites its own, but the shared ones are:
//!
//! * *IBM Personal Computer AT Technical Reference* (1984) — the board: which
//!   chip answers which port, which IRQ each device lands on, and the system
//!   control ports that are not chips at all.
//! * The Intel component data sheets for the 8259A, 8253/8254, 8237A and 8042,
//!   and the Motorola MC146818 and MC6845 data sheets.
//! * Ralf Brown's Interrupt List, ports section, for the register-level
//!   behaviour the data sheets leave to the board.
//! * The OSDev wiki, for the same facts restated by people who have tested
//!   them.
//!
//! **No emulator source was consulted for any of it** (`CLAUDE.md`, provenance).
//!
//! # The board's I/O map
//!
//! The addresses are the AT's, and they are written once — in the machine file.
//! A device here knows only the offset within its own register block:
//!
//! ```text
//!   0x000-0x00f  DMA controller 1        (byte channels 0-3)
//!   0x020-0x021  interrupt controller 1  (master)
//!   0x040-0x043  8254 timer
//!   0x060,0x064  8042 keyboard controller
//!   0x061        system control port B   (speaker gate, timer 2 out, refresh)
//!   0x070-0x071  MC146818 RTC and CMOS   (plus the NMI mask, in bit 7 of 0x70)
//!   0x080-0x08f  DMA page registers
//!   0x092        system control port A   (A20 gate, fast reset)
//!   0x0a0-0x0a1  interrupt controller 2  (slave, cascaded onto IR2)
//!   0x0c0-0x0df  DMA controller 2        (word channels 4-7)
//!   0x170-0x177  IDE, secondary channel  (command block)
//!   0x1f0-0x1f7  IDE, primary channel    (command block)
//!   0x376        IDE, secondary channel  (control block)
//!   0x3b4-0x3b5  CRTC, monochrome        }  one chip; which pair answers is
//!   0x3d4-0x3d5  CRTC, colour            }  a board-level decode
//!   0x3f0-0x3f5  floppy controller
//!   0x3f6        IDE, primary channel    (control block)
//!   0x3f7        floppy digital input / configuration control
//! ```
//!
//! The floppy's window has a hole in it at `0x3f6`, and that is the board's
//! doing rather than an accident: the diskette adapter decodes `0x3f0-0x3f5`
//! and `0x3f7`, and `0x3f6` belongs to the fixed-disk adapter.
//!
//! # Conventions every chip here follows
//!
//! * State behind one [`Mutex`](crate::core::sync::Mutex) at
//!   [`LockRank::DEVICE`](crate::core::sync::LockRank::DEVICE); output pins at
//!   [`LEAF`](crate::core::sync::LockRank::LEAF), so a line can be driven with
//!   nothing else held. Never drive a wire with the state lock held — that is
//!   the re-entrancy contract, and an 8259A whose `INT` output re-enters its
//!   own port handler is exactly the case it exists for.
//! * `MemAttrs::debug` suppresses every side effect. A debugger that reads the
//!   8259A's IRR must not pop the 8042's output buffer or clear the RTC's
//!   interrupt flags.
//! * One [`DeviceClass`](crate::core::DeviceClass) per part, `save`/`load` for
//!   its architectural state, and a round-trip test beside it.

pub mod dma;
pub mod kbc;
pub mod pic;
pub mod pit;
pub mod rom;
pub mod rtc;
pub mod sysctl;

#[cfg(feature = "dev-pc-pci")]
#[cfg_attr(docsrs, doc(cfg(feature = "dev-pc-pci")))]
pub mod pmc;

#[cfg(feature = "dev-pc-pci")]
#[cfg_attr(docsrs, doc(cfg(feature = "dev-pc-pci")))]
pub mod vgapci;

#[cfg(feature = "dev-pc-video")]
#[cfg_attr(docsrs, doc(cfg(feature = "dev-pc-video")))]
pub mod video;

#[cfg(feature = "dev-pc-floppy")]
#[cfg_attr(docsrs, doc(cfg(feature = "dev-pc-floppy")))]
pub mod fdc;

#[cfg(feature = "dev-pc-ide")]
#[cfg_attr(docsrs, doc(cfg(feature = "dev-pc-ide")))]
pub mod ide;

use alloc::sync::Arc;
use alloc::vec::Vec;

use crate::core::error::Result;
use crate::core::registry::Registry;
use crate::core::space::MemOps;
use crate::machine::realize::Bindings;
use crate::machine::validate::ClassSchema;

/// A window of I/O space one chip decodes *inside* another chip's window.
///
/// The contract behind
/// [`ExportId::PORT_PASSTHROUGH`](crate::core::device::ExportId::PORT_PASSTHROUGH),
/// and it exists because a PC's `0xcf8`-`0xcfb` is claimed by the north bridge
/// for a Dword access and its `0xcf9` byte by the south bridge for a byte
/// access — two chips, one address, told apart by the byte enables. An
/// [`AddressSpace`](crate::core::space::AddressSpace) decodes by address alone,
/// so the chip that needs all four bytes holds them and passes the rest on.
///
/// The [`MemOps`] inside is addressed at the **same offsets as the outer
/// window**, so a chip decoding `0xcf9` answers at offset 1 of a window based
/// at `0xcf8`.
#[derive(Debug)]
pub struct PortPassthrough(Arc<dyn MemOps>);

impl PortPassthrough {
    /// Wrap `ops` as a pass-through window.
    #[must_use]
    pub fn new(ops: Arc<dyn MemOps>) -> PortPassthrough {
        PortPassthrough(ops)
    }

    /// What answers the cycles the outer chip does not claim.
    #[must_use]
    pub fn ops(&self) -> &Arc<dyn MemOps> {
        &self.0
    }
}

/// Add every class in this module to a registry.
///
/// # Errors
///
/// [`Error::Config`](crate::core::Error::Config) if a name is already claimed.
pub fn register(reg: &mut Registry) -> Result<()> {
    dma::register(reg)?;
    kbc::register(reg)?;
    pic::register(reg)?;
    pit::register(reg)?;
    rom::register(reg)?;
    rtc::register(reg)?;
    sysctl::register(reg)?;
    #[cfg(feature = "dev-pc-pci")]
    pmc::register(reg)?;
    #[cfg(feature = "dev-pc-pci")]
    vgapci::register(reg)?;
    #[cfg(feature = "dev-pc-video")]
    video::register(reg)?;
    #[cfg(feature = "dev-pc-floppy")]
    fdc::register(reg)?;
    #[cfg(feature = "dev-pc-ide")]
    ide::register(reg)?;
    Ok(())
}

/// Bind every class in this module into the machine graph.
///
/// # Errors
///
/// [`Error::Config`](crate::core::Error::Config) if a name is bound twice.
pub fn bind(b: &mut Bindings) -> Result<()> {
    dma::bind(b)?;
    kbc::bind(b)?;
    pic::bind(b)?;
    pit::bind(b)?;
    rom::bind(b)?;
    rtc::bind(b)?;
    sysctl::bind(b)?;
    #[cfg(feature = "dev-pc-pci")]
    pmc::bind(b)?;
    #[cfg(feature = "dev-pc-pci")]
    vgapci::bind(b)?;
    #[cfg(feature = "dev-pc-video")]
    video::bind(b)?;
    #[cfg(feature = "dev-pc-floppy")]
    fdc::bind(b)?;
    #[cfg(feature = "dev-pc-ide")]
    ide::bind(b)?;
    Ok(())
}

/// What the validator should know about every class in this module.
#[must_use]
pub fn schemas() -> Vec<ClassSchema> {
    #[allow(unused_mut)]
    let mut out = alloc::vec![
        dma::schema(),
        kbc::schema(),
        pic::schema(),
        pit::schema(),
        rom::schema(),
        rtc::schema(),
        sysctl::schema(),
    ];
    #[cfg(feature = "dev-pc-pci")]
    out.push(pmc::schema());
    #[cfg(feature = "dev-pc-pci")]
    out.push(vgapci::schema());
    #[cfg(feature = "dev-pc-video")]
    out.push(video::schema());
    #[cfg(feature = "dev-pc-floppy")]
    out.push(fdc::schema());
    #[cfg(feature = "dev-pc-ide")]
    out.push(ide::schema());
    out
}

/// The machine description this chipset was written for, compiled in so that a
/// build which can realize it always ships one that parses.
///
/// It is data, not code: a user copies `machines/pc-at.machine` and edits it.
pub const PC_AT: &str = include_str!("../../../machines/pc-at.machine");

#[cfg(test)]
mod tests {
    use super::*;
    #[cfg(all(
        feature = "dev-pc-video",
        feature = "dev-pc-floppy",
        feature = "dev-pc-ide"
    ))]
    use crate::machine::ClassTable;
    use crate::machine::ResolveOptions;
    use crate::machine::resolve_file;
    #[cfg(all(
        feature = "dev-pc-video",
        feature = "dev-pc-floppy",
        feature = "dev-pc-ide"
    ))]
    use crate::machine::validate::{ClassSchema, ValidateOptions, validate};
    // Only the fallback schema below writes these out by hand; with the core
    // compiled in it publishes its own.
    #[cfg(all(
        feature = "dev-pc-video",
        feature = "dev-pc-floppy",
        feature = "dev-pc-ide",
        not(feature = "cpu-x86")
    ))]
    use crate::machine::validate::{PortDir, PropSchema};
    use alloc::string::ToString;

    /// What the PC board needs from the x86 core.
    ///
    /// This used to be a specification written here because `cpu.i8086` was
    /// registered but not *bound* — no `Instance` impl, no `bind`, no input
    /// pins and no `schema` — so a machine file could not give it an address
    /// space or wire an interrupt to it, and the board could not go in the
    /// catalog. The core has that surface now, so this asks the core for its
    /// own schema instead: one description, and no second copy to drift.
    ///
    /// The board still validates against it here, because a chipset test that
    /// checks its own machine file is worth having whether or not a CPU feature
    /// is enabled.
    #[cfg(all(
        feature = "dev-pc-video",
        feature = "dev-pc-floppy",
        feature = "dev-pc-ide"
    ))]
    fn x86_schema() -> ClassSchema {
        #[cfg(feature = "cpu-x86")]
        {
            crate::cpu::x86::schemas()
                .into_iter()
                .find(|s| s.class == "cpu.x86")
                .expect("the core publishes both of its class names")
        }
        // Without the core compiled in there is nothing to ask, and the board
        // still has to validate: the memory map and the wire graph are the
        // chipset's, not the processor's.
        #[cfg(not(feature = "cpu-x86"))]
        {
            ClassSchema::new("cpu.x86")
                .prop(PropSchema::new("model", crate::core::props::ValueKind::Str))
                .prop(PropSchema::new(
                    "engine",
                    crate::core::props::ValueKind::Str,
                ))
                .prop(PropSchema::new(
                    "iospace",
                    crate::core::props::ValueKind::Str,
                ))
                .port("intr", PortDir::In)
                .port("nmi", PortDir::In)
                .port("reset", PortDir::In)
                .port("a20", PortDir::In)
        }
    }

    #[cfg(all(
        feature = "dev-pc-video",
        feature = "dev-pc-floppy",
        feature = "dev-pc-ide"
    ))]
    fn classes() -> ClassTable {
        let mut table = ClassTable::new();
        for schema in crate::machine::builtin::schemas() {
            table.insert(schema);
        }
        for schema in schemas() {
            table.insert(schema);
        }
        table.insert(x86_schema());
        table
    }

    #[test]
    fn the_board_parses_and_resolves() {
        let resolved = match resolve_file("pc-at.machine", PC_AT, &ResolveOptions::new()) {
            Ok(r) => r,
            Err(e) => panic!("{e}"),
        };
        assert_eq!(resolved.name, "pc-at");
        // Six crystals, because the board has six cans and they are six trees
        // rather than dividers off one (`ROADMAP.md` §4.2).
        assert_eq!(resolved.oscillators.len(), 6);
        // The 8254's input is not an integer number of hertz, which is the
        // whole reason the language takes rational frequency literals. Written
        // 105000000/88 because that is 14.31818 MHz over 12 and how the board
        // derives it; stored reduced, as 13125000/11.
        let pit = resolved
            .oscillators
            .iter()
            .find(|o| o.name == "pit")
            .expect("the timer's crystal");
        assert_eq!(pit.hz.denominator(), 11);
        assert_eq!(pit.hz.numerator(), 13125000);
        assert_eq!(resolved.spaces.len(), 2, "memory and I/O are separate");
    }

    // The board names a display and a floppy controller, so it can only be
    // checked against a build that has them. A `dev-pc`-only build still parses
    // it — the test above — which is the half that does not depend on features.
    #[cfg(all(
        feature = "dev-pc-video",
        feature = "dev-pc-floppy",
        feature = "dev-pc-ide"
    ))]
    #[test]
    fn the_board_validates_against_this_builds_classes() {
        // Everything the board names exists, every property is one its class
        // accepts, every `map` names a region the device publishes, and every
        // `wire` names a pin — with the x86 core's side of it stubbed above.
        let resolved =
            resolve_file("pc-at.machine", PC_AT, &ResolveOptions::new()).expect("it resolves");
        if let Err(d) = validate(&resolved, &classes(), &ValidateOptions::new()) {
            panic!("{}", d.message);
        }
    }

    #[test]
    fn the_board_names_exactly_the_media_slots_it_documents() {
        // Two firmware sockets and three removable-or-fitted media. None of
        // them is required: the disks default to an empty bay and the floppy to
        // an empty drive, so the only slot a user *has* to fill is `bios`.
        //
        // Deduplicated, because `vgabios` is named **twice** and on purpose:
        // the legacy socket at 0xc0000 and the PCI card's expansion ROM are two
        // ways into the same video BIOS, and which one a firmware takes depends
        // on whether it knows what a 440FX is. A slot is a slot however many
        // objects read it.
        let resolved =
            resolve_file("pc-at.machine", PC_AT, &ResolveOptions::new()).expect("it resolves");
        let mut slots: alloc::vec::Vec<alloc::string::String> = resolved
            .objects
            .iter()
            .filter_map(|o| o.props.get("image"))
            .filter_map(|v| v.as_str().map(ToString::to_string))
            .collect();
        slots.sort();
        slots.dedup();
        assert_eq!(slots, ["bios", "floppy", "hd0", "hd1", "vgabios"]);
    }
}
