//! The interface a realized NES machine must expose for the AccuracyCoin runner.
//!
//! Separate from [`crate::cpu`] on purpose. AccuracyCoin is **not** a CPU test:
//! of its 67 documented sections roughly a third are PPU, a sixth APU, and the
//! most interesting ones — NMI suppression, sprite-zero timing, DMC DMA bus
//! conflicts, open bus — are about the *interaction* between the CPU, the PPU,
//! the APU, the DMA units and the cartridge, at exact clock alignments. Nothing
//! here can run against a bare core.
//!
//! So the interface is a whole machine, and it is deliberately tiny: advance
//! time, press a button, peek at RAM without disturbing it.

/// A realized NES, running headlessly.
///
/// No framebuffer, no audio sink, no host window. The runner never looks at a
/// pixel — everything it needs is in work RAM.
pub(crate) trait NesMachine: Send {
    /// Advance by whole frames.
    ///
    /// Frames rather than cycles because the ROM's menu is NMI-driven: a button
    /// has to be held across at least one vertical blank for the ROM's own
    /// edge detector to see it.
    fn run_frames(&mut self, frames: u32);

    /// Set the controller-1 button state.
    ///
    /// Bit order is the shift register's output order, which is what
    /// AccuracyCoin's own read routine assembles: bit 7 A, bit 6 B, bit 5
    /// Select, bit 4 Start, bit 3 Up, bit 2 Down, bit 1 Left, bit 0 Right. Use
    /// the [`buttons`] constants rather than literals.
    fn set_controller1(&mut self, buttons: u8);

    /// Where the CPU is, for a report about a machine that stopped making
    /// progress.
    ///
    /// A hang inside a test says nothing on its own: the ROM's result byte
    /// stays "in progress" and the tally stops. The program counter says which
    /// of the ROM's several *documented* infinite loops it is sitting in — the
    /// DMA sync routines "rely on open bus behavior, with the consequence of an
    /// infinite loop if not correctly emulated" — which turns a timeout into a
    /// diagnosis.
    fn cpu_state(&self) -> Option<String> {
        None
    }

    /// Read CPU-visible memory **without side effects**.
    ///
    /// This is a debugger read (`MemAttrs::debug`, `CLAUDE.md` Devices): it must
    /// not pop a FIFO, clear a status bit or advance a pointer. The runner only
    /// ever peeks at work RAM, so a machine may reasonably restrict this to
    /// `$0000-$1FFF`.
    fn peek(&self, addr: u16) -> u8;
}

/// Controller-1 button bits, in shift-register order.
pub(crate) mod buttons {
    /// The A button.
    pub(crate) const A: u8 = 0x80;
    /// The B button.
    pub(crate) const B: u8 = 0x40;
    /// Select.
    pub(crate) const SELECT: u8 = 0x20;
    /// Start.
    pub(crate) const START: u8 = 0x10;
    /// D-pad up.
    pub(crate) const UP: u8 = 0x08;
    /// D-pad down.
    pub(crate) const DOWN: u8 = 0x04;
    /// D-pad left.
    pub(crate) const LEFT: u8 = 0x02;
    /// D-pad right.
    pub(crate) const RIGHT: u8 = 0x01;
    /// Nothing held.
    pub(crate) const NONE: u8 = 0x00;
}

/// The Cargo features that let this harness run a NES.
///
/// `std` for the same reason [`crate::cpu::CPU_FEATURES`] needs it: this binary
/// creates threads, and without `std` the crate's `core::sync` selects a
/// backend that is single-threaded by construction.
pub(crate) const MACHINE_FEATURES: &str = "machine-nes,std";

/// Are those features on for this build?
pub(crate) fn machine_is_built() -> bool {
    cfg!(all(feature = "machine-nes", feature = "std"))
}

/// Build a machine from an iNES image, or say why not.
///
/// `nes-ntsc` out of the shipped catalog with `cart` bound to the image — the
/// same thing `rsemu run nes-ntsc --cart AccuracyCoin.nes` builds, with no
/// hand-wiring and no test-only topology.
#[cfg(all(feature = "machine-nes", feature = "std"))]
pub(crate) fn new_nes(rom: &[u8]) -> Result<Box<dyn NesMachine>, String> {
    match Nes::new(rom) {
        Ok(nes) => Ok(Box::new(nes)),
        Err(e) => Err(format!("nes-ntsc did not realize: {e}")),
    }
}

/// No NES this harness can drive in this build.
#[cfg(not(all(feature = "machine-nes", feature = "std")))]
pub(crate) fn new_nes(rom: &[u8]) -> Result<Box<dyn NesMachine>, String> {
    let _ = rom;
    Err(format!(
        "this build does not have all of {MACHINE_FEATURES}"
    ))
}

/// A machine, or the *reason* there is none — and only one reason is allowed.
///
/// The counterpart of [`crate::cpu::require_cpu`], for the same reason. "This
/// build has no NES" is a skip. "This build has a NES and it would not build"
/// is a defect, and it is asserted rather than printed — a whole-machine suite
/// that silently measures nothing is the failure mode this whole change exists
/// to remove. That assertion is not hypothetical: it is what caught
/// `machine-nes` not implying `dev-nes-ppu`, so `nes-ntsc` compiled and then
/// refused to realize for anyone who picked that feature alone.
///
/// # Panics
///
/// If a NES is compiled in and the machine will not realize.
pub(crate) fn require_nes(rom: &[u8]) -> Result<Box<dyn NesMachine>, crate::harness::Skip> {
    match new_nes(rom) {
        Ok(nes) => Ok(nes),
        Err(e) => {
            assert!(
                !machine_is_built(),
                "`{MACHINE_FEATURES}` are on but no NES could be built, so the \
                 whole-machine suite would skip and pass while measuring nothing: {e}"
            );
            Err(crate::harness::Skip::NotBuilt {
                component: "a NES machine",
                feature: MACHINE_FEATURES,
            })
        }
    }
}

/// A zeroed NROM-128 cartridge whose reset vector points at `$C000`.
///
/// Not a corpus and not a fixture on disk: 24 KiB generated here, so the seam
/// check in `main.rs` can prove `nes-ntsc` still realizes without the gate
/// being open or `AccuracyCoin.nes` having been fetched.
pub(crate) fn blank_nrom() -> Vec<u8> {
    let mut image = vec![0u8; 16 + 16 * 1024 + 8 * 1024];
    image[0..4].copy_from_slice(b"NES\x1a");
    image[4] = 1; // one 16 KiB PRG bank
    image[5] = 1; // one 8 KiB CHR bank
    // `JMP $C000` at the start of the bank, and every vector pointing at it.
    let prg = 16;
    image[prg] = 0x4c;
    image[prg + 1] = 0x00;
    image[prg + 2] = 0xc0;
    for slot in [0x3ffa, 0x3ffc, 0x3ffe] {
        image[prg + slot] = 0x00;
        image[prg + slot + 1] = 0xc0;
    }
    image
}

/// A realized `nes-ntsc`, driven by whole frames.
#[cfg(all(feature = "machine-nes", feature = "std"))]
struct Nes {
    machine: rsemu::machine::Machine,
    /// The host end of the controller seam. Buttons are a *level*: whatever is
    /// set here is what the console latches the next time the ROM strobes.
    pads: std::sync::Arc<rsemu::dev::nes::Pad>,
}

#[cfg(all(feature = "machine-nes", feature = "std"))]
impl Nes {
    /// One NTSC frame, in nanoseconds.
    ///
    /// 341 x 262 dots minus the odd-frame skip, at 236250000/11 / 4 Hz — which
    /// is 60.0988 Hz. The machine's own clocks are exact; this number only has
    /// to be long enough that every frame contains exactly one vertical blank,
    /// and the run loop stops on the PPU's own events regardless.
    const FRAME_NS: u64 = 16_639_267;

    fn new(rom: &[u8]) -> Result<Nes, rsemu::Error> {
        let machine = rsemu::machine::catalog::build_catalog("nes-ntsc", &[("cart", rom)])?;
        Ok(Nes {
            pads: rsemu::dev::nes::pads::open(rsemu::dev::nes::DEFAULT_PAD_PORT),
            machine,
        })
    }
}

#[cfg(all(feature = "machine-nes", feature = "std"))]
impl NesMachine for Nes {
    fn run_frames(&mut self, frames: u32) {
        let span = rsemu::core::clock::GlobalTime::from_nanos(Self::FRAME_NS);
        for _ in 0..frames {
            self.machine.run_for(span).expect("the machine runs");
        }
    }

    fn set_controller1(&mut self, buttons: u8) {
        self.pads.set(0, buttons);
    }

    fn cpu_state(&self) -> Option<String> {
        use rsemu::core::state::{MachineShape, Migrations, Source as _, StateReader, StateWriter};
        let cpu = self.machine.device("cpu")?;
        let class = cpu.device().class();
        // Through the snapshot surface, because the machine holds its devices
        // as `dyn Device` and the register file's encoding is published there.
        let mut shape = MachineShape::new();
        shape.add_device("cpu", class.name).ok()?;
        let mut writer = StateWriter::new(shape);
        let mut chunk = writer.chunk("cpu", class.name, class.version).ok()?;
        cpu.device().save(&mut chunk).ok()?;
        let bytes = writer.to_vec().ok()?;
        let reader = StateReader::new(&bytes).ok()?;
        let chunk = reader
            .load("cpu", class.name, class.version, &Migrations::new())
            .ok()?;
        let mut r = chunk.reader();
        let a = r.read_u8().ok()?;
        let x = r.read_u8().ok()?;
        let y = r.read_u8().ok()?;
        let s = r.read_u8().ok()?;
        let p = r.read_u8().ok()?;
        let pc = r.read_u16().ok()?;
        let cycles = r.read_u64().ok()?;
        Some(format!(
            "A:{a:02X} X:{x:02X} Y:{y:02X} S:{s:02X} P:{p:02X} PC:{pc:04X} cyc:{cycles}"
        ))
    }

    fn peek(&self, addr: u16) -> u8 {
        use rsemu::core::space::MemAttrs;
        use rsemu::core::value::Width;
        self.machine
            .space("cpubus")
            .expect("cpubus")
            .read(u64::from(addr), Width::U8, MemAttrs::DEBUG)
            .expect("the open-bus policy answers everything") as u8
    }
}
