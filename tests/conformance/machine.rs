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

/// Build a machine from an iNES image, or `None` if this build has no NES.
///
/// `nes-ntsc` out of the shipped catalog with `rom` bound to the image — the
/// same thing `rsemu run nes-ntsc --cart AccuracyCoin.nes` builds, with no
/// hand-wiring and no test-only topology. If it does not realize, the runner
/// wants to know why, so the error is printed rather than swallowed.
#[cfg(feature = "machine-nes")]
pub(crate) fn new_nes(rom: &[u8]) -> Option<Box<dyn NesMachine>> {
    match Nes::new(rom) {
        Ok(nes) => Some(Box::new(nes)),
        Err(e) => {
            println!("note: nes-ntsc did not realize: {e}");
            None
        }
    }
}

/// No NES in this build.
#[cfg(not(feature = "machine-nes"))]
pub(crate) fn new_nes(rom: &[u8]) -> Option<Box<dyn NesMachine>> {
    let _ = rom;
    None
}

/// A realized `nes-ntsc`, driven by whole frames.
#[cfg(feature = "machine-nes")]
struct Nes {
    machine: rsemu::machine::Machine,
    /// The host end of the controller seam. Buttons are a *level*: whatever is
    /// set here is what the console latches the next time the ROM strobes.
    pads: std::sync::Arc<rsemu::dev::nes::Pad>,
}

#[cfg(feature = "machine-nes")]
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

#[cfg(feature = "machine-nes")]
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
