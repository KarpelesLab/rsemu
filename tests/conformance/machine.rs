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
/// # Wiring up a real machine
///
/// When the NES board lands, this becomes a call into the `.machine` loader
/// with the cartridge substituted, plus a small adapter implementing the three
/// methods above. Everything else in the AccuracyCoin runner is already written
/// against them.
pub(crate) fn new_nes(rom: &[u8]) -> Option<Box<dyn NesMachine>> {
    let _ = rom;
    None
}
