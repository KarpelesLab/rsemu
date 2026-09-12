//! The **device-owned framebuffer** seam: a panel that holds its own picture.
//!
//! # Two kinds of display device, and why they need different seams
//!
//! [`scanout`](super::scanout) is the first kind. The pixels live in *guest
//! memory*, the device knows a base address, and the picture is whatever the
//! guest last stored there. Everything a host needs follows from that: the
//! guest's own stores are what change the picture, the dirty-page machinery
//! already tracks them, and a snapshot of the RAM device saves the pixels.
//!
//! The second kind is a **smart panel** — an SSD1306, an ST7789, a T6963, a
//! KS0108. The controller has its *own* display RAM on the far side of a serial
//! bus, and the guest changes it by sending **commands**: `RAMWR` and a stream
//! of pixels, or a control byte and a page of GDDRAM. Nothing about that is a
//! memory store. It follows that:
//!
//! * **Nothing invalidates the picture through the dirty-page machinery**,
//!   because no page was dirtied. A host cannot ask guest memory whether the
//!   screen moved; only the device knows. So the device publishes a counter and
//!   the host compares it — [`Panel::generation`], below.
//! * **The framebuffer is architectural state.** `scanout` serializes six
//!   registers and no pixels, because the pixels belong to the RAM device and
//!   are saved with it. A device-owned framebuffer has no such owner: if the
//!   panel does not put it in its own `save`/`load` chunk, it is gone. Every
//!   implementor of this trait therefore round-trips its display RAM, and has a
//!   test that asserts it.
//! * **Refresh is not observable and is therefore not modelled.** A real
//!   SSD1306 redraws the glass from GDDRAM at `FOSC / (D × K × MUX)`, and a real
//!   ST7789 at its own frame rate. No guest can see either: there is no
//!   readable frame counter, no VSYNC pin on the modules these parts ship on,
//!   and the tearing-effect line the ST7789 does have is a machine-file
//!   decision rather than something this seam needs. So the panel has **no
//!   clock domain and no scheduler event**, which is also why none of these
//!   devices is lazily advanced. See [`Panel::generation`] for what replaces
//!   the frame counter, and
//!   [`Scanout::frame_period_ns`](crate::host::display::Scanout::frame_period_ns)
//!   for what a host is told instead of a rate.
//!
//! # The seam itself
//!
//! Three methods, and deliberately the *same three* the RGB engine already
//! answers — geometry, a monotonic counter, and a row of RGB triples. That is
//! what lets one host adapter,
//! [`host::display::panel`](crate::host::display::panel), serve every part that
//! implements this, instead of one adapter per controller the way
//! [`host::display::nes`](crate::host::display::nes) and its siblings each need
//! one. They need one each because each converts a *different* encoding — a
//! 2C02's palette index, a VDP's colour table. A smart panel has already done
//! its own conversion by the time anyone asks: the datasheet defines the
//! mapping from display RAM to glass completely, so the device computes it and
//! hands over finished intensities.
//!
//! **`dev/` still names no colour** (`host::display`'s rule): a monochrome OLED
//! hands out the *intensity* its contrast register produces, and a 16-bit TFT
//! hands out the 8-bit channels its `COLMOD` expansion produces. Which phosphor
//! or filter that drives is the host's business, and neither device has an
//! opinion about it.
//!
//! # Registering one
//!
//! A host that wants the picture has the same problem `host::display::lcd`
//! documents: `machine::build` hands back `Arc<dyn Device>` and there is no
//! route from that to a concrete type. [`PanelClass`] is this family's answer —
//! a class name plus a constructor that yields *both* halves, the
//! [`Instance`] the machine keeps and the [`Panel`] the host keeps. The host
//! module installs one interception per compiled-in class and they all capture
//! into one table.

use alloc::sync::Arc;
use alloc::vec;
use alloc::vec::Vec;
use core::fmt;

use crate::core::error::Result;
use crate::core::props::Props;
use crate::machine::realize::Instance;

/// A display controller that holds its own framebuffer.
///
/// `Send + Sync` like every device-facing trait (`ROADMAP.md` §0): the
/// emulation thread changes the picture and the display thread captures it.
pub trait Panel: Send + Sync + fmt::Debug {
    /// The visible glass, in pixels: `(width, height)`.
    ///
    /// Fixed for the life of the device. It is what the *module* shows, which
    /// is not always what the controller stores — an SH1106 holds 132 columns
    /// and shows 128 of them, and a 240×240 ST7789 module holds 320 rows.
    fn geometry(&self) -> (u32, u32);

    /// How many times the visible picture has been able to change.
    ///
    /// Monotonic, and **not a frame count**. There is no frame here: the panel
    /// refreshes on an internal oscillator nothing can observe, so counting
    /// refreshes would publish a number with no meaning. What a host actually
    /// wants to know is "is this the same picture I already drew", and that is
    /// answered by counting *content changes* — every write that reaches
    /// display RAM, and every command that changes how display RAM is mapped to
    /// the glass. A command that changes neither must not bump it, or a host
    /// that draws only on change draws constantly.
    ///
    /// It is compared against [`Surface::serial`](crate::host::display::Surface::serial),
    /// which is exactly what `Scanout::frame_counter` is for, so the two kinds
    /// of display device stay interchangeable to a host.
    fn generation(&self) -> u64;

    /// Fill `dst` with row `y`, as RGB intensities.
    ///
    /// `dst` is `geometry().0` long; a shorter one is filled as far as it goes.
    /// A `y` past the bottom leaves it alone. **No side effects** — this is the
    /// [`MemAttrs::debug`](crate::core::space::MemAttrs::debug) rule applied to
    /// a picture, and a host may capture the same frame twice.
    fn read_row(&self, y: u32, dst: &mut [[u8; 3]]);
}

/// A device class whose instances own a framebuffer, and how to build one
/// while keeping hold of the picture.
///
/// The two `Arc`s are the same object; they differ only in which trait the
/// holder needs. `construct` is a plain `fn` pointer rather than a closure so a
/// class can declare one in a `static` beside its
/// [`DeviceClass`](crate::core::device::DeviceClass).
#[derive(Debug, Clone, Copy)]
pub struct PanelClass {
    /// The class name a machine description writes.
    pub name: &'static str,
    /// Build one, yielding both halves.
    ///
    /// # Errors
    ///
    /// Whatever the device's own constructor rejects.
    pub construct: fn(&Props) -> Result<Built>,
}

/// One panel, as both of the things its two holders need.
#[derive(Debug, Clone)]
pub struct Built {
    /// What the machine keeps: the device.
    pub instance: Arc<dyn Instance>,
    /// What the host keeps: the picture.
    pub panel: Arc<dyn Panel>,
}

/// Every visible pixel, row by row.
///
/// For a test that wants to assert a whole picture without a host surface.
#[must_use]
pub fn read_frame(panel: &dyn Panel) -> Vec<Vec<[u8; 3]>> {
    let (width, height) = panel.geometry();
    let mut rows = Vec::with_capacity(height as usize);
    for y in 0..height {
        let mut row = vec![[0u8; 3]; width as usize];
        panel.read_row(y, &mut row);
        rows.push(row);
    }
    rows
}
