//! What this build can emulate: its device classes and its shipped machines.
//!
//! A machine is a feature set (`ROADMAP.md` §3), so "which classes exist?" and
//! "which machines can this binary run?" are build-specific questions with
//! honest answers. This module is where they are answered, once, for the CLI,
//! the wasm shim, the tests and `rsemu describe` alike — three copies of a
//! registration list would drift apart on the first new device.
//!
//! # Registration is explicit
//!
//! One `#[cfg(feature = …)]` arm per component, calling that component's own
//! `register` / `bind` / `schema` (§4.4). No link-time magic, no inventory
//! crate: a class that is not named here is not in the build, and that is
//! visible by reading the file.
//!
//! # Three tables, not one
//!
//! * [`registry`] — construction, and the `rsemu devices` listing. The table
//!   of record.
//! * [`bindings`] — the classes that take part in the memory map and the wire
//!   graph. See [`Bindings`] for why this is still separate.
//! * [`classes`] — what the *validator* checks a machine file against. It
//!   cannot be derived from the registry: `DeviceClass` declares a class's
//!   properties but not its pins or its mappable regions, so a table built
//!   from the registry alone would reject `map cpubus 0x8000 = cart.prg`.
//!
//! [`registry`]: registry()
//! [`bindings`]: bindings()
//! [`classes`]: classes()
//!
//! # The machine catalog
//!
//! `machines/*.machine` ships as **data** — a user copies one and edits it —
//! and the files this build knows how to realize are also compiled in, so
//! `rsemu run nes-ntsc` works from any directory and a wasm build has them
//! without a filesystem. A path on the command line still wins: the catalog is
//! a default, not a jail.

use alloc::string::String;
use alloc::vec::Vec;

use crate::core::error::Result;
use crate::core::registry::Registry;
use crate::machine::builtin;
use crate::machine::realize::Bindings;
use crate::machine::validate::ClassTable;
use crate::machine::{BuildOptions, Machine};

/// One machine description this build ships.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct CatalogEntry {
    /// The name `rsemu run <name>` takes, and the file's stem under
    /// `machines/`.
    pub name: &'static str,
    /// One line for `rsemu machines`.
    pub summary: &'static str,
    /// Media slots the machine will not realize without, as
    /// `rsemu run … --<slot> <file>` spells them.
    pub media: &'static [&'static str],
    /// The description text itself.
    pub source: &'static str,
}

/// The NTSC NES, when this build has a 6502 and a cartridge to put in it.
#[cfg(feature = "machine-nes")]
#[cfg_attr(docsrs, doc(cfg(feature = "machine-nes")))]
pub static NES_NTSC: CatalogEntry = CatalogEntry {
    name: "nes-ntsc",
    summary: "Nintendo Entertainment System / Famicom, NTSC (RP2C02 at 60 Hz)",
    media: &["cart"],
    source: include_str!("../../machines/nes-ntsc.machine"),
};

/// The PAL NES: the same board with a different crystal and different
/// dividers.
///
/// Its own file rather than a parameter of the NTSC one. The region changes the
/// oscillator, both clock dividers and the PPU's frame geometry at once, and
/// §4.2 makes the oscillator topology part of the machine's identity — a
/// snapshot records it, and two machines that differ in it are not the same
/// machine wearing a flag.
#[cfg(feature = "machine-nes")]
#[cfg_attr(docsrs, doc(cfg(feature = "machine-nes")))]
pub static NES_PAL: CatalogEntry = CatalogEntry {
    name: "nes-pal",
    summary: "Nintendo Entertainment System, PAL (RP2C07 at 50 Hz, 312 scanlines)",
    media: &["cart"],
    source: include_str!("../../machines/nes-pal.machine"),
};

/// The Apple 1, when this build has a 6502 and the board's chips.
///
/// The `rom` slot has a default nothing else does: `rsemu run apple1` with no
/// `--rom` binds [`RSMON`](crate::dev::apple1::RSMON), rsemu's own monitor, so
/// the machine demonstrates itself without a ROM of unclear provenance.
#[cfg(feature = "machine-apple1")]
#[cfg_attr(docsrs, doc(cfg(feature = "machine-apple1")))]
pub static APPLE1: CatalogEntry = CatalogEntry {
    name: "apple1",
    summary: "Apple 1 (1976): 6502, 4K RAM, MC6821 keyboard and display",
    media: &["rom"],
    source: include_str!("../../machines/apple1.machine"),
};

/// The Game Boy, when this build has an SM83 and the console's chips.
///
/// Phase 4's genericity proof (`ROADMAP.md` §13): a machine that is not
/// NES-shaped, realized by the same core.
#[cfg(feature = "machine-gameboy")]
#[cfg_attr(docsrs, doc(cfg(feature = "machine-gameboy")))]
pub static GAMEBOY: CatalogEntry = CatalogEntry {
    name: "gameboy",
    summary: "Nintendo Game Boy (DMG, 1989): SM83 at 4.194304 MHz, 160x144 LCD",
    media: &["cart"],
    source: include_str!("../../machines/gameboy.machine"),
};

/// Every machine this build can realize, in catalog order.
// One `#[cfg]`-gated push per shipped machine, which is what the lint is
// complaining about: a `vec![]` literal cannot carry an attribute on one of its
// elements, so the push form is the only one that expresses "this entry exists
// only in some builds".
#[allow(unused_mut, clippy::vec_init_then_push)]
#[must_use]
pub fn machines() -> Vec<&'static CatalogEntry> {
    let mut out: Vec<&'static CatalogEntry> = Vec::new();
    #[cfg(feature = "machine-apple1")]
    out.push(&APPLE1);
    #[cfg(feature = "machine-gameboy")]
    out.push(&GAMEBOY);
    #[cfg(feature = "machine-nes")]
    out.push(&NES_NTSC);
    #[cfg(feature = "machine-nes")]
    out.push(&NES_PAL);
    out
}

/// One shipped machine by name, with or without its `.machine` suffix.
#[must_use]
pub fn machine(name: &str) -> Option<&'static CatalogEntry> {
    let stem = name.strip_suffix(".machine").unwrap_or(name);
    machines().into_iter().find(|m| m.name == stem)
}

/// Every device class this build can construct.
///
/// # Errors
///
/// [`Error::Config`](crate::core::Error::Config) if two features claimed one
/// class name, which is a bug in this file rather than in a machine
/// description.
pub fn registry() -> Result<Registry> {
    let mut reg = Registry::new();
    builtin::register(&mut reg)?;
    #[cfg(feature = "cpu-mos6502")]
    crate::cpu::mos6502::register(&mut reg)?;
    #[cfg(feature = "cpu-sm83")]
    crate::cpu::sm83::register(&mut reg)?;
    #[cfg(feature = "dev-gb")]
    crate::dev::gb::register(&mut reg)?;
    #[cfg(feature = "dev-nes-cart")]
    crate::dev::cart::nrom::register(&mut reg)?;
    #[cfg(feature = "dev-nes-ppu")]
    crate::dev::ppu::register(&mut reg)?;
    #[cfg(feature = "dev-nes-apu")]
    crate::dev::apu::register(&mut reg)?;
    #[cfg(feature = "dev-apple1")]
    crate::dev::apple1::register(&mut reg)?;
    Ok(reg)
}

/// Every class that takes part in the memory map and the wire graph.
///
/// # The PPU and the APU
///
/// Both are **lazily advanced** (`ROADMAP.md` §4.2): they hold their own tick
/// and are caught up by whoever accesses them, through
/// [`LazyHandle`](crate::core::sched::LazyHandle). They were kept out of this
/// table until a class could *declare* that, because a PPU that is mapped and
/// wired and then never advanced reports VBlank clear forever — a worse machine
/// than one with no PPU at all, where the open bus at least reads as ones.
///
/// [`Device::is_lazy`](crate::core::Device::is_lazy) is that declaration, and
/// [`realize`](mod@crate::machine::realize) registers such a device on its clock
/// domain and hands it back the handle its own `MemOps` syncs through. So they
/// are bound.
///
/// # Errors
///
/// As [`registry`].
pub fn bindings() -> Result<Bindings> {
    let mut b = Bindings::new();
    builtin::bind(&mut b)?;
    #[cfg(feature = "cpu-mos6502")]
    crate::cpu::mos6502::bind(&mut b)?;
    #[cfg(feature = "cpu-sm83")]
    crate::cpu::sm83::bind(&mut b)?;
    #[cfg(feature = "dev-gb")]
    crate::dev::gb::bind(&mut b)?;
    #[cfg(feature = "dev-nes-cart")]
    crate::dev::cart::nrom::bind(&mut b)?;
    #[cfg(feature = "dev-nes-ppu")]
    crate::dev::ppu::bind(&mut b)?;
    #[cfg(feature = "dev-nes-apu")]
    crate::dev::apu::bind(&mut b)?;
    #[cfg(feature = "dev-apple1")]
    crate::dev::apple1::bind(&mut b)?;
    Ok(b)
}

/// What the validator checks a machine file against.
#[must_use]
pub fn classes() -> ClassTable {
    let mut table = ClassTable::new();
    for schema in builtin::schemas() {
        table.insert(schema);
    }
    #[cfg(feature = "cpu-mos6502")]
    table.insert(crate::cpu::mos6502::schema());
    #[cfg(feature = "cpu-sm83")]
    table.insert(crate::cpu::sm83::schema());
    #[cfg(feature = "dev-gb")]
    for schema in crate::dev::gb::schemas() {
        table.insert(schema);
    }
    #[cfg(feature = "dev-nes-cart")]
    table.insert(crate::dev::cart::nrom::schema());
    #[cfg(feature = "dev-nes-ppu")]
    table.insert(crate::dev::ppu::schema());
    #[cfg(feature = "dev-nes-apu")]
    table.insert(crate::dev::apu::schema());
    #[cfg(feature = "dev-apple1")]
    for schema in crate::dev::apple1::schemas() {
        table.insert(schema);
    }
    table
}

/// [`BuildOptions`] wired to this build's classes and bindings.
///
/// The caller adds media and parameter overrides; everything else about "what
/// this binary knows" is already here.
///
/// # Errors
///
/// As [`registry`].
pub fn build_options() -> Result<BuildOptions> {
    Ok(BuildOptions::new()
        .with_classes(classes())
        .with_bindings(bindings()?))
}

/// Build a shipped machine by catalog name.
///
/// `media` binds the slots the description names: for the NES that is
/// `[("cart", &image)]`, which is what `--cart smb.nes` becomes.
///
/// # Errors
///
/// If the name is not in this build's catalog, or anything
/// [`build`](crate::machine::build) refuses — including a media slot the
/// caller did not bind.
pub fn build_catalog(name: &str, media: &[(&str, &[u8])]) -> Result<Machine> {
    let entry = machine(name).ok_or_else(|| unknown(name))?;
    let mut options = build_options()?;
    for (slot, bytes) in media {
        options.realize.media.insert(*slot, *bytes);
    }
    crate::machine::build(entry.name, entry.source, &registry()?, &options)
}

/// The error for a machine this build does not ship.
fn unknown(name: &str) -> crate::core::Error {
    let mut message = String::from("no machine named `");
    message.push_str(name);
    message.push_str("` in this build; it has ");
    let names: Vec<&str> = machines().into_iter().map(|m| m.name).collect();
    if names.is_empty() {
        message.push_str("none (enable a `machine-*` feature)");
    } else {
        for (i, n) in names.iter().enumerate() {
            if i != 0 {
                message.push_str(", ");
            }
            message.push('`');
            message.push_str(n);
            message.push('`');
        }
    }
    crate::core::Error::Config {
        at: String::from("catalog"),
        message,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use alloc::string::ToString;

    #[test]
    fn the_registry_and_the_bindings_agree() {
        let reg = registry().expect("no class name collides");
        let bound = bindings().expect("no binding collides");
        // A class with a binding but no registry entry is invisible to
        // `rsemu devices` and to the validator, which is exactly the drift
        // the registry-is-the-table-of-record rule exists to prevent.
        for class in bound.classes() {
            assert!(
                reg.get(class).is_some(),
                "`{class}` is bound but not registered"
            );
        }
        assert!(reg.get("ram").is_some(), "the language's own class");
    }

    #[test]
    fn every_shipped_machine_realizes() {
        // The catalog's whole claim. A machine file that no longer parses, or
        // names a class this build dropped, fails here rather than in front of
        // a user.
        for entry in machines() {
            let media: Vec<(&str, &[u8])> = entry
                .media
                .iter()
                .map(|slot| (*slot, fixture(entry.name, slot)))
                .collect();
            match build_catalog(entry.name, &media) {
                Ok(machine) => assert_eq!(machine.name(), entry.name),
                Err(e) => panic!("{}: {e}", entry.name),
            }
        }
    }

    #[test]
    fn an_unknown_machine_lists_what_there_is() {
        let e = build_catalog("megadrive", &[])
            .expect_err("no megadrive")
            .to_string();
        assert!(e.contains("megadrive"), "{e}");
    }

    /// The CPU's architectural state, read back out of a snapshot.
    ///
    /// There is no route from a `dyn Device` to a `Mos6502` — `core::device`
    /// keeps `Any` out of the supertrait chain deliberately — so the way to
    /// see a core's registers from outside is the surface §4.5 already
    /// promises: its snapshot chunk. Reading it here doubles as a check that
    /// the chunk really is the architectural state, and it pins the layout to
    /// the class version so a bump cannot silently change what this decodes.
    #[cfg(feature = "cpu-mos6502")]
    #[derive(Debug, Clone, Copy, PartialEq, Eq)]
    struct CpuState {
        a: u8,
        x: u8,
        y: u8,
        s: u8,
        p: u8,
        pc: u16,
        cycles: u64,
        halted: bool,
        reset_pending: bool,
        faults: u64,
        last_fault: u16,
    }

    #[cfg(feature = "cpu-mos6502")]
    fn cpu_state(machine: &Machine, path: &str) -> CpuState {
        use crate::core::state::{Migrations, Source, StateReader};
        let class = &crate::cpu::mos6502::CLASS;
        let bytes = machine.save().expect("a machine saves");
        let reader = StateReader::new(&bytes).expect("well formed");
        let chunk = reader
            .load(path, class.name, class.version, &Migrations::new())
            .expect("a chunk per device, keyed by instance path");
        let mut r = chunk.reader();
        let mut byte = || r.read_u8().expect("the chunk is not truncated");
        let (a, x, y, s, p) = (byte(), byte(), byte(), byte(), byte());
        let pc = r.read_u16().expect("pc");
        let cycles = r.read_u64().expect("cycles");
        let halted = r.read_bool().expect("halted");
        let reset_pending = r.read_bool().expect("reset_pending");
        let _pending_interrupt = r.read_u8().expect("pending");
        let _open_bus = r.read_u8().expect("open bus");
        let faults = r.read_u64().expect("faults");
        let last_fault = r.read_u16().expect("last fault");
        CpuState {
            a,
            x,
            y,
            s,
            p,
            pc,
            cycles,
            halted,
            reset_pending,
            faults,
            last_fault,
        }
    }

    /// One CPU-visible byte, read the way a debugger would.
    #[cfg(feature = "machine-nes")]
    fn peek(machine: &Machine, addr: u64) -> u8 {
        use crate::core::space::MemAttrs;
        use crate::core::value::Width;
        machine
            .space("cpubus")
            .expect("cpubus")
            .read(addr, Width::U8, MemAttrs::DEBUG)
            .expect("open bus answers everything") as u8
    }

    /// The reset vector is fetched from the cartridge and executed from.
    ///
    /// No corpus and no environment variable: the fixture is a generated NROM
    /// image whose vector points at `$C000` and whose only instruction is
    /// `JMP $C000`, so the CPU's program counter after any amount of running
    /// is exactly one known number. That makes this the test that fails when
    /// the memory map, the vector fetch or the scheduler wiring breaks —
    /// [`a_real_cartridge_boots_and_executes`] then says how far real software
    /// gets.
    ///
    /// [`a_real_cartridge_boots_and_executes`]: self::tests::a_real_cartridge_boots_and_executes
    #[cfg(feature = "machine-nes")]
    #[test]
    fn the_reset_vector_is_fetched_and_executed() {
        let mut machine =
            build_catalog("nes-ntsc", &[("cart", MINIMAL_NROM)]).expect("a minimal cart");

        // NROM-128: 16 KiB of PRG answers at $8000 *and* at $C000, because A14
        // is not connected. Both windows must show the same byte.
        assert_eq!(peek(&machine, 0xfffc), 0x00);
        assert_eq!(peek(&machine, 0xfffd), 0xc0);
        assert_eq!(peek(&machine, 0x8000), 0x4c, "JMP at the low window");
        assert_eq!(peek(&machine, 0xc000), 0x4c, "and at the high one");

        let before = cpu_state(&machine, "cpu");
        assert!(before.reset_pending);

        machine
            .run_for(crate::core::clock::GlobalTime::from_nanos(100_000))
            .expect("runs");

        let after = cpu_state(&machine, "cpu");
        assert!(!after.reset_pending, "the reset sequence ran");
        assert_eq!(
            after.pc, 0xc000,
            "the cpu is not executing the reset vector's target"
        );
        // The reset sequence pushes nothing but decrements S three times, from
        // the power-on 0 — so $FD, and it sets I.
        assert_eq!(after.s, 0xfd);
        assert_ne!(after.p & crate::cpu::mos6502::flags::I, 0);
        assert!(after.cycles >= 7 + 3, "reset plus at least one JMP");
        assert_eq!(after.faults, 0, "every access is answered");

        // 2 KiB of work RAM answers four times over $0000-$1FFF, which is the
        // one thing in this machine the cartridge does not provide.
        machine
            .space("cpubus")
            .expect("cpubus")
            .write(
                0x0003,
                crate::core::value::Width::U8,
                0xa5,
                crate::core::space::MemAttrs::DEFAULT,
            )
            .expect("wram is writable");
        for base in [0x0000u64, 0x0800, 0x1000, 0x1800] {
            assert_eq!(peek(&machine, base + 3), 0xa5, "mirror at {base:#06x}");
        }
    }

    /// A machine built from a `.machine` file snapshots and restores.
    #[cfg(feature = "machine-nes")]
    #[test]
    fn a_running_nes_round_trips_through_a_snapshot() {
        let mut machine =
            build_catalog("nes-ntsc", &[("cart", MINIMAL_NROM)]).expect("a minimal cart");
        machine
            .run_for(crate::core::clock::GlobalTime::from_nanos(100_000))
            .expect("runs");
        let saved = machine.save().expect("saves");
        let hash = machine.state_hash().expect("hashes");

        // Into a second machine built from the same description, which is the
        // case that matters: a save state is loaded by a fresh process.
        let mut restored =
            build_catalog("nes-ntsc", &[("cart", MINIMAL_NROM)]).expect("a minimal cart");
        assert_ne!(restored.state_hash().expect("hashes"), hash);
        restored.load(&saved).expect("loads");
        assert_eq!(restored.state_hash().expect("hashes"), hash);
        assert_eq!(cpu_state(&restored, "cpu"), cpu_state(&machine, "cpu"));

        // And it keeps running identically from there — the point of a
        // deterministic snapshot, and what the cycle debt is carried for.
        let span = crate::core::clock::GlobalTime::from_nanos(1_000_000);
        machine.run_for(span).expect("runs");
        restored.run_for(span).expect("runs");
        assert_eq!(
            restored.state_hash().expect("hashes"),
            machine.state_hash().expect("hashes")
        );
    }

    /// A whole NES boots from a real cartridge and retires instructions.
    ///
    /// The phase-3 milestone in one test: a `.machine` file, a real ROM bound
    /// to a media slot, a realized machine, and a 6502 fetching its reset
    /// vector out of PRG ROM and executing from it — through the scheduler,
    /// the address space and the region tree, with no hand-wiring anywhere.
    ///
    /// Gated on `RSEMU_NES_TEST_ROM`, like every other corpus (`CLAUDE.md`):
    /// point it at an iNES image. AccuracyCoin is the one this was written
    /// against — MIT, © 2025 Chris Siebert — but any NROM cartridge works.
    /// Without the variable the test passes trivially, so `cargo test` offline
    /// stays green.
    #[cfg(all(feature = "machine-nes", feature = "std"))]
    #[test]
    fn a_real_cartridge_boots_and_executes() {
        let Ok(path) = std::env::var("RSEMU_NES_TEST_ROM") else {
            println!("SKIP: set RSEMU_NES_TEST_ROM to an iNES image to run this");
            return;
        };
        let image = std::fs::read(&path).expect("RSEMU_NES_TEST_ROM is readable");
        let mut machine = match build_catalog("nes-ntsc", &[("cart", &image)]) {
            Ok(m) => m,
            Err(e) => panic!("{path}: {e}"),
        };

        // The machine came up cold, so the CPU owes a reset sequence and has
        // not fetched anything yet.
        let before = cpu_state(&machine, "cpu");
        assert!(before.reset_pending, "a cold machine owes a reset");
        assert_eq!(before.cycles, 0);

        // The reset vector, as the cartridge holds it. `$FFFC` is inside PRG
        // ROM, so this already proves the cart is mapped where the file says.
        let vector = u16::from(peek(&machine, 0xfffc)) | (u16::from(peek(&machine, 0xfffd)) << 8);
        assert!(
            vector >= 0x8000,
            "a reset vector of {vector:#06x} is not in cartridge space; is the ROM mapped?"
        );

        // A frame's worth of virtual time. Deterministic: the same span always
        // retires the same number of cycles, whatever the host is doing.
        let frame = crate::core::clock::GlobalTime::from_nanos(16_639_267);
        machine.run_for(frame).expect("the machine runs");

        let after = cpu_state(&machine, "cpu");
        let domain = machine
            .device("cpu")
            .and_then(crate::machine::machine::DeviceEntry::domain)
            .expect("the cpu has a clock domain");
        let ticks = machine.clocks().ticks(domain).expect("a tick count");

        println!(
            "nes-ntsc + {path}:\n  \
             reset vector ${vector:04x}\n  \
             {} cpu cycles in one frame ({ticks} domain ticks)\n  \
             {}\n  \
             {} refused access(es){}",
            after.cycles,
            regs_line(&after),
            after.faults,
            if after.faults == 0 {
                ""
            } else {
                " — the memory map has a hole the open-bus policy did not cover"
            },
        );

        assert!(!after.reset_pending, "the reset sequence must have run");
        assert!(
            after.cycles > 20_000,
            "only {} cycles in a frame; the cpu is not running",
            after.cycles
        );
        // The scheduler must not have been overrun or starved: one NTSC frame
        // is 29780.5 CPU cycles, and the debt mechanism keeps the core's own
        // count within one instruction of its domain's.
        assert!(
            after.cycles.abs_diff(ticks) <= 7,
            "cpu counted {} cycles but its domain advanced {ticks}",
            after.cycles
        );
        assert!(
            !after.halted,
            "a JAM opcode froze the core at ${:04x}",
            after.pc
        );
        // Every access is answered: RAM, the cartridge, or the open bus the
        // real console has. A refusal means the address space itself said no.
        assert_eq!(
            after.faults, 0,
            "bus fault at ${:04x} after {} cycles",
            after.last_fault, after.cycles
        );

        // -- and now the part the PPU makes possible -----------------------
        //
        // Two seconds of virtual time is far more than any NES ROM's init
        // needs: AccuracyCoin runs its whole first page of CPU tests and draws
        // its menu well inside one.
        for _ in 0..120 {
            machine.run_for(frame).expect("the machine runs");
        }
        let after = cpu_state(&machine, "cpu");
        let ppu_domain = machine
            .device("ppu")
            .and_then(crate::machine::machine::DeviceEntry::domain)
            .expect("the ppu has a clock domain");
        let dots = machine.clocks().ticks(ppu_domain).expect("a dot count");
        let ticks = machine.clocks().ticks(domain).expect("a tick count");
        println!(
            "  after 121 frames: {} PPU dots, {}\n  \
             {} tiles written to the first nametable",
            dots,
            regs_line(&after),
            nametable_tiles(&machine)
        );

        // Exactly three dots per CPU cycle, forever, by construction: both
        // counters descend from one crystal (`ROADMAP.md` §4.2). If this ever
        // drifts the split-screen effects in half the NES library break.
        assert_eq!(dots, ticks * 3, "the dot clock is not three times the CPU");
        assert_eq!(after.faults, 0, "bus fault at ${:04x}", after.last_fault);
        assert!(
            !after.halted,
            "a JAM opcode froze the core at ${:04x}",
            after.pc
        );

        // The ROM has finished its init, run its tests and drawn a screen.
        // Reaching this needs `$2002` to report vblank at the dot it is read
        // on, `$2006`/`$2007` to reach the nametables through the PPU's own
        // bus, and the NMI to arrive: an unadvanced PPU parks the wait loop at
        // the top of the reset path with a blank screen behind it.
        let tiles = nametable_tiles(&machine);
        assert!(
            tiles > 64,
            "only {tiles} non-blank tiles in the first nametable; the ROM never \
             drew anything"
        );
    }

    /// How many of the first nametable's 960 tiles are not the blank one.
    ///
    /// Read through the PPU's own address space with debug attributes, so
    /// counting them disturbs nothing.
    #[cfg(all(feature = "machine-nes", feature = "std"))]
    fn nametable_tiles(machine: &Machine) -> usize {
        use crate::core::space::MemAttrs;
        use crate::core::value::Width;
        let space = machine.space("ppubus").expect("ppubus");
        (0..960u64)
            .filter(|i| {
                let tile = space
                    .read(0x2000 + i, Width::U8, MemAttrs::DEBUG)
                    .unwrap_or(0);
                // `$24` is the blank tile in most ASCII-ish NES tile sets, and
                // `$00` is an unwritten nametable.
                tile != 0x24 && tile != 0x00
            })
            .count()
    }

    #[cfg(feature = "cpu-mos6502")]
    fn regs_line(s: &CpuState) -> alloc::string::String {
        alloc::format!(
            "A:{:02x} X:{:02x} Y:{:02x} P:{:02x} SP:{:02x} PC:{:04x}",
            s.a,
            s.x,
            s.y,
            s.p,
            s.s,
            s.pc
        )
    }

    /// The whole point of the exercise: vblank is reported and the NMI fires.
    ///
    /// No corpus and no environment variable — the cartridge is generated here,
    /// so this runs on every `cargo test` and is the regression gate for
    /// sync-on-access end to end (`ROADMAP.md` §4.2). It proves three separate
    /// things, and each of them was broken before the PPU could declare itself
    /// lazily advanced:
    ///
    /// 1. **`$2002` reports vblank.** The ROM does what every NES game's init
    ///    does — polls `$2002` until bit 7 goes high, twice. With an unadvanced
    ///    PPU that loop never ends; with an unmapped one it falls straight
    ///    through on open bus, which looks like success and is not.
    /// 2. **The chip advances with nobody looking at it.** After enabling the
    ///    NMI the ROM sits in `JMP *` and touches no PPU register ever again.
    ///    Sync-on-access alone would leave it there forever.
    /// 3. **The NMI lands once per frame.** Not twice, which is what a level
    ///    re-announced at every catch-up boundary would produce, and not never.
    #[cfg(feature = "machine-nes")]
    #[test]
    fn vblank_is_reported_and_the_nmi_fires_once_a_frame() {
        let image = nmi_rom();
        let mut machine = build_catalog("nes-ntsc", &[("cart", &image)]).expect("a cart");
        // Six frames of virtual time. The first is spent in the warm-up
        // lockout, so `$2000` is only accepted from the second.
        let frame = crate::core::clock::GlobalTime::from_nanos(16_639_267);
        for _ in 0..6 {
            machine.run_for(frame).expect("runs");
        }

        assert_eq!(
            peek(&machine, 0x0001),
            1,
            "the vblank wait loop never ended"
        );
        let nmis = peek(&machine, 0x0000);
        // Frame 0 is the lockout and frame 1 is where the wait loop ends, so
        // four are certain and six are the ceiling.
        assert!(
            (4..=6).contains(&nmis),
            "{nmis} NMIs in six frames; one per vblank is the answer"
        );

        // Another three frames: exactly three more.
        for _ in 0..3 {
            machine.run_for(frame).expect("runs");
        }
        assert_eq!(peek(&machine, 0x0000), nmis + 3);
    }

    /// A debug access advances nothing (`ROADMAP.md` §15, invariant 5).
    ///
    /// The catch-up hook sits at the top of the PPU's own `MemOps::read`, which
    /// is exactly where it would be easiest to move the chip's clock on a
    /// monitor read. It must not.
    #[cfg(feature = "machine-nes")]
    #[test]
    fn a_debug_read_of_2002_advances_no_clock() {
        use crate::core::space::MemAttrs;
        use crate::core::value::Width;

        let mut machine = build_catalog("nes-ntsc", &[("cart", MINIMAL_NROM)]).expect("a cart");
        machine
            .run_for(crate::core::clock::GlobalTime::from_nanos(1_000_000))
            .expect("runs");
        // Catch-up did happen: the chip is standing exactly on its domain's
        // tick. Without that this test would pass vacuously.
        let domain = machine
            .device("ppu")
            .and_then(crate::machine::machine::DeviceEntry::domain)
            .expect("the ppu has a clock domain");
        let before = ppu_dots(&machine);
        assert_eq!(before, machine.clocks().ticks(domain).expect("ticks"));
        assert!(before > 0);

        for _ in 0..64 {
            let _ = peek(&machine, 0x2002);
        }
        assert_eq!(
            ppu_dots(&machine),
            before,
            "a debug read moved the dot clock"
        );

        // A debug *write* to a port with side effects is refused outright,
        // which is the same invariant seen from the other side.
        assert!(
            machine
                .space("cpubus")
                .expect("cpubus")
                .write(0x2000, Width::U8, 0x80, MemAttrs::DEBUG)
                .is_err()
        );
        assert_eq!(ppu_dots(&machine), before);
    }

    /// The PPU's dot counter, out of its snapshot chunk.
    #[cfg(feature = "machine-nes")]
    fn ppu_dots(machine: &Machine) -> u64 {
        use crate::core::state::{Migrations, Source, StateReader};
        let class = &crate::dev::ppu::NES_PPU_CLASS;
        let bytes = machine.save().expect("a machine saves");
        let reader = StateReader::new(&bytes).expect("well formed");
        let chunk = reader
            .load("ppu", class.name, class.version, &Migrations::new())
            .expect("a chunk per device");
        chunk.reader().read_u64().expect("dots come first")
    }

    /// A cartridge whose init waits for two vblanks, enables the NMI and then
    /// does nothing at all — the shape of every NES game's reset path.
    ///
    /// `$00` counts NMIs and `$01` counts completions of the wait loop.
    #[cfg(feature = "machine-nes")]
    fn nmi_rom() -> alloc::vec::Vec<u8> {
        let mut image = alloc::vec![0u8; 16 + 16384 + 8192];
        image[..4].copy_from_slice(b"NES\x1a");
        image[4] = 1; // 16 KiB of PRG, which answers at $8000 and at $C000
        image[5] = 1; // 8 KiB of CHR
        let prg = &mut image[16..16 + 16384];
        let code: &[u8] = &[
            0x78, // $C000  SEI
            0xad, 0x02, 0x20, // $C001  LDA $2002   reset the address latch
            0xad, 0x02, 0x20, // $C004  LDA $2002
            0x10, 0xfb, //       $C007  BPL $C004   wait for vblank
            0xad, 0x02, 0x20, // $C009  LDA $2002
            0x10, 0xfb, //       $C00C  BPL $C009   and again
            0xee, 0x01, 0x00, // $C00E  INC $0001   the wait loop ended
            0xa9, 0x80, //       $C011  LDA #$80
            0x8d, 0x00, 0x20, // $C013  STA $2000   enable the NMI
            0x4c, 0x16, 0xc0, // $C016  JMP $C016   and never look again
        ];
        prg[..code.len()].copy_from_slice(code);
        // The handler, at $C020: count it and return. It deliberately does not
        // read $2002 — the request stays asserted for the whole of vblank, and
        // the CPU's edge latch is what must keep that from firing twice.
        prg[0x20..0x23].copy_from_slice(&[0xe6, 0x00, 0x40]); // INC $00 ; RTI
        // NMI $C020, RESET $C000, IRQ $C020.
        prg[0x3ffa..0x4000].copy_from_slice(&[0x20, 0xc0, 0x00, 0xc0, 0x20, 0xc0]);
        image
    }

    /// Something plausible to bind to a media slot, so the catalog can be
    /// realized without a corpus.
    fn fixture(machine: &str, slot: &str) -> &'static [u8] {
        match (machine, slot) {
            // Two machines take a slot called `cart` and they are not the same
            // kind of cartridge at all, which is why this is keyed by both.
            #[cfg(feature = "machine-gameboy")]
            ("gameboy", "cart") => minimal_gb(),
            (_, "cart") => MINIMAL_NROM,
            // The Apple 1's default: rsemu's own monitor, which is committed
            // precisely so that this needs no download and no licence
            // question.
            #[cfg(feature = "machine-apple1")]
            (_, "rom") => crate::dev::apple1::RSMON,
            (_, other) => panic!("no fixture for media slot `{other}`"),
        }
    }

    /// The smallest legal Game Boy image: two 16 KiB banks, a correct header
    /// checksum, and a one-instruction program (`JR -2`, the smallest program
    /// that neither ends nor wanders). Generated, never vendored.
    #[cfg(feature = "machine-gameboy")]
    fn minimal_gb() -> &'static [u8] {
        // A `OnceLock` rather than a `static` array because the header checksum
        // is computed over the image, and the generator is where that lives.
        use std::sync::OnceLock;
        static IMAGE: OnceLock<Vec<u8>> = OnceLock::new();
        IMAGE.get_or_init(|| crate::dev::gb::cart::synthetic_image(2, 0x00, 0x00, &[0x18, 0xfe]))
    }

    /// The smallest legal NROM image: an iNES header, 16 KiB of PRG, 8 KiB of
    /// CHR. Generated, never vendored.
    static MINIMAL_NROM: &[u8] = &{
        let mut image = [0u8; 16 + 16384 + 8192];
        image[0] = b'N';
        image[1] = b'E';
        image[2] = b'S';
        image[3] = 0x1a;
        image[4] = 1; // 16 KiB of PRG
        image[5] = 1; // 8 KiB of CHR
        // A reset vector at $C000 — the 16 KiB of PRG answers at both $8000
        // and $C000 — holding `JMP $C000`, so the program counter after any
        // amount of running is exactly one known number.
        image[16 + 0x3ffc] = 0x00;
        image[16 + 0x3ffd] = 0xc0;
        image[16] = 0x4c;
        image[17] = 0x00;
        image[18] = 0xc0;
        image
    };
}
