//! The SDSC debug console — ports `$FC` and `$FD`.
//!
//! This is **not** a chip. It is a convention: a pair of write-only ports that
//! emulators agreed to watch so that a homebrew or test ROM can print text
//! without a screen. SMS Power! publishes the
//! [specification](https://www.smspower.org/Development/SDSCDebugConsoleSpecification),
//! and the ROMs that matter here — Maxim's port of Frank Cringle's Z80
//! exerciser above all — report their results through it.
//!
//! ```text
//!   $FC  control   1 suspend, 2 clear, 3 set attribute, 4 move cursor
//!   $FD  data      one character, or a parameter for the last control command
//! ```
//!
//! # Why it is in the shipped machine
//!
//! Because on real hardware **nothing answers a write to `$FC` or `$FD`**. Those
//! addresses are in the `$C0`-`$FF` block, where the I/O chip decodes reads and
//! the write side goes nowhere at all. So the board maps this device on the
//! *write* half of two addresses with `split()`, leaves the read half on the
//! control pads where the hardware puts it, and no guest behaviour changes. A
//! game that writes there — none do — writes into the same void it would on a
//! console, except that rsemu keeps the bytes.
//!
//! What that buys is the thing `ROADMAP.md` §12 asks for and this platform makes
//! hard: a **headless** conformance run. The Master System's test ROMs otherwise
//! report by drawing on the screen, which turns "did it pass" into "hash the
//! framebuffer and hope". With the console wired, a suite's own words come out
//! as text.
//!
//! # What is not modelled
//!
//! * **The `%`-prefixed formatting directives.** The specification defines a
//!   small language for printing numbers out of memory, video RAM, the CPU's
//!   registers and the VDP's — a debugger's `printf` living inside a port. A
//!   ROM that uses it gets its directives copied through literally. Nothing that
//!   this project runs uses them, and implementing a formatter that reaches into
//!   four other devices is a larger design question than it looks.
//! * **The 80x25 screen.** This keeps a linear character log, which is what a
//!   harness greps. `clear` empties it; a cursor move is accepted, its
//!   parameters consumed, and the log carries on where it was.
//! * **`suspend`.** Recorded in [`SdscConsole::suspend_requested`] rather than
//!   acted on: a device does not stop the scheduler (`CLAUDE.md`).

use alloc::boxed::Box;
use alloc::string::String;
use alloc::sync::Arc;
use core::fmt;

use crate::core::device::{Device, DeviceClass, PropertySpec, RealizeCtx, ResetKind};
use crate::core::error::{BusError, Error, Result};
use crate::core::props::{Props, ValueKind};
use crate::core::space::{
    AccessConstraints, MemAttrs, MemOps, MemResult, Region as MmioRegion, RegionRef,
};
use crate::core::state::{ChunkReader, ChunkWriter, Sink, Source};
use crate::core::sync::{AtomicBool, LockRank, Mutex, Ordering};
use crate::core::value::Width;

/// The name a `map` statement reaches the two ports by.
///
/// Two bytes: `$FC` at offset 0, `$FD` at offset 1.
pub const PORT_REGION: &str = "port";

/// How much text the log keeps before the oldest is dropped.
pub const DEFAULT_CAPACITY: u64 = 64 * 1024;

/// The snapshot chunk version. Bump with the encoding, never on its own.
const STATE_VERSION: u32 = 1;

/// What the next data byte means.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
enum Pending {
    /// A character to print.
    #[default]
    Text,
    /// The attribute byte for control command 3.
    Attribute,
    /// The row, then the column, for control command 4.
    CursorRow,
    /// The column.
    CursorColumn,
}

#[derive(Debug, Default)]
struct Console {
    log: String,
    capacity: usize,
    pending: Pending,
    attribute: u8,
    cursor: (u8, u8),
}

impl Console {
    fn control(&mut self, value: u8) {
        match value {
            2 => {
                self.log.clear();
                self.cursor = (0, 0);
                self.pending = Pending::Text;
            }
            3 => self.pending = Pending::Attribute,
            4 => self.pending = Pending::CursorRow,
            // 1 is suspend, handled by the caller; anything else is undefined
            // and the specification does not say what a console should do, so
            // the safe reading is "nothing".
            _ => {}
        }
    }

    fn data(&mut self, value: u8) {
        match self.pending {
            Pending::Attribute => {
                self.attribute = value;
                self.pending = Pending::Text;
            }
            Pending::CursorRow => {
                self.cursor.0 = value;
                self.pending = Pending::CursorColumn;
            }
            Pending::CursorColumn => {
                self.cursor.1 = value;
                self.pending = Pending::Text;
            }
            Pending::Text => {
                // Printable ASCII, plus the two line terminators the
                // specification names. Everything else is dropped rather than
                // written as a replacement character, so a log stays greppable.
                let keep = matches!(value, 0x0a | 0x0d | 0x20..=0x7e);
                if keep {
                    if self.log.len() >= self.capacity {
                        // Drop the oldest line rather than the oldest byte: a
                        // half-truncated line reads as a different result.
                        match self.log.find('\n') {
                            Some(index) => {
                                self.log.drain(..=index);
                            }
                            None => self.log.clear(),
                        }
                    }
                    self.log.push(value as char);
                }
            }
        }
    }
}

struct Shared {
    console: Mutex<Console>,
    suspend: AtomicBool,
}

impl fmt::Debug for Shared {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("Shared")
            .field("console", &self.console)
            .finish_non_exhaustive()
    }
}

/// The debug console as a device.
pub struct SdscConsole {
    shared: Arc<Shared>,
    port_region: RegionRef,
}

impl fmt::Debug for SdscConsole {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("SdscConsole")
            .field("shared", &self.shared)
            .finish_non_exhaustive()
    }
}

impl Default for SdscConsole {
    fn default() -> Self {
        SdscConsole::new(DEFAULT_CAPACITY)
    }
}

impl SdscConsole {
    /// A console keeping at most `capacity` bytes of text.
    #[must_use]
    pub fn new(capacity: u64) -> SdscConsole {
        let shared = Arc::new(Shared {
            console: Mutex::with_rank(
                LockRank::DEVICE,
                Console {
                    log: String::new(),
                    capacity: capacity.max(1) as usize,
                    ..Console::default()
                },
            ),
            suspend: AtomicBool::new(false),
        });
        let port_region = Arc::new(MmioRegion::io(
            "sms.sdsc.port",
            2,
            Arc::new(SdscPorts {
                shared: Arc::clone(&shared),
            }) as Arc<dyn MemOps>,
        ));
        SdscConsole {
            shared,
            port_region,
        }
    }

    /// Build one from machine-description properties.
    ///
    /// # Errors
    ///
    /// If an unknown property was given.
    pub fn from_props(props: &Props) -> Result<SdscConsole> {
        let mut r = props.reader();
        let capacity = r.or_size("capacity", DEFAULT_CAPACITY)?;
        r.finish()?;
        Ok(SdscConsole::new(capacity))
    }

    /// Everything the guest has printed since the last clear.
    #[must_use]
    pub fn text(&self) -> String {
        self.shared.console.lock().log.clone()
    }

    /// Empty the log.
    pub fn clear(&self) {
        self.shared.console.lock().log.clear();
    }

    /// Whether the guest has asked the emulator to suspend, which rsemu records
    /// rather than obeys.
    #[must_use]
    pub fn suspend_requested(&self) -> bool {
        self.shared.suspend.load(Ordering::Relaxed)
    }

    /// Write one of the two ports as the guest would — control at 0, data at 1.
    pub fn write_port(&self, offset: u64, value: u8) {
        if offset & 1 == 0 {
            if value == 1 {
                self.shared.suspend.store(true, Ordering::Relaxed);
            }
            self.shared.console.lock().control(value);
        } else {
            self.shared.console.lock().data(value);
        }
    }
}

/// `$FC` and `$FD`.
struct SdscPorts {
    shared: Arc<Shared>,
}

impl fmt::Debug for SdscPorts {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("SdscPorts").finish_non_exhaustive()
    }
}

impl MemOps for SdscPorts {
    fn read(&self, _offset: u64, dst: &mut [u8], _attrs: MemAttrs) -> MemResult {
        let [byte] = dst else {
            return Err(BusError::BadAccess);
        };
        // Write-only, and on hardware these addresses read as the control pads.
        // A board maps the read half there with `split()`; this is what answers
        // when it did not.
        *byte = 0xff;
        Ok(())
    }

    fn write(&self, offset: u64, src: &[u8], attrs: MemAttrs) -> MemResult {
        let [value] = src else {
            return Err(BusError::BadAccess);
        };
        if attrs.debug {
            return Ok(());
        }
        if offset & 1 == 0 {
            if *value == 1 {
                self.shared.suspend.store(true, Ordering::Relaxed);
            }
            self.shared.console.lock().control(*value);
        } else {
            self.shared.console.lock().data(*value);
        }
        Ok(())
    }

    fn constraints(&self) -> AccessConstraints {
        AccessConstraints::IO.with_widths(Width::U8, Width::U8)
    }
}

// ---------------------------------------------------------------------------
// Device
// ---------------------------------------------------------------------------

/// The `sms.sdsc` device class.
pub static CLASS: DeviceClass = DeviceClass {
    name: "sms.sdsc",
    version: 1,
    summary: "SDSC debug console at $FC/$FD: a test ROM's text output, headlessly",
    properties: &[PropertySpec {
        name: "capacity",
        kind: ValueKind::Size,
        required: false,
        summary: "how much text the log keeps before the oldest line is dropped",
    }],
    construct: |props| Ok(Box::new(SdscConsole::from_props(props)?) as Box<dyn Device>),
};

/// Add this class to a registry.
///
/// # Errors
///
/// If something already claimed the name.
pub fn register(reg: &mut crate::core::Registry) -> Result<()> {
    reg.add(&CLASS)
}

impl Device for SdscConsole {
    fn class(&self) -> &'static DeviceClass {
        &CLASS
    }

    fn realize(&self, _ctx: &mut RealizeCtx<'_>) -> Result<()> {
        Ok(())
    }

    fn region(&self, name: &str) -> Option<RegionRef> {
        (name.is_empty() || name == PORT_REGION).then(|| Arc::clone(&self.port_region))
    }

    fn reset(&self, _kind: ResetKind) {
        let mut console = self.shared.console.lock();
        console.log.clear();
        console.pending = Pending::Text;
        console.cursor = (0, 0);
        console.attribute = 0;
        self.shared.suspend.store(false, Ordering::Relaxed);
    }

    fn save(&self, w: &mut ChunkWriter<'_>) -> Result<()> {
        let console = self.shared.console.lock();
        w.write_u32(STATE_VERSION)?;
        w.write_str(&console.log)?;
        w.write_u8(console.pending as u8)?;
        w.write_u8(console.attribute)?;
        w.write_u8(console.cursor.0)?;
        w.write_u8(console.cursor.1)?;
        w.write_bool(self.shared.suspend.load(Ordering::Relaxed))?;
        Ok(())
    }

    fn load(&self, r: &mut ChunkReader<'_>) -> Result<()> {
        let version = r.read_u32()?;
        if version != STATE_VERSION {
            return Err(Error::State(alloc::format!(
                "the debug console's snapshot is version {version}, this build writes \
                 {STATE_VERSION}"
            )));
        }
        let log = r.read_string()?;
        let pending = match r.read_u8()? {
            0 => Pending::Text,
            1 => Pending::Attribute,
            2 => Pending::CursorRow,
            3 => Pending::CursorColumn,
            other => {
                return Err(Error::State(alloc::format!(
                    "the debug console's snapshot has an unknown pending state {other}"
                )));
            }
        };
        let mut console = self.shared.console.lock();
        console.log = log;
        console.pending = pending;
        console.attribute = r.read_u8()?;
        console.cursor.0 = r.read_u8()?;
        console.cursor.1 = r.read_u8()?;
        self.shared.suspend.store(r.read_bool()?, Ordering::Relaxed);
        Ok(())
    }
}

impl crate::machine::Instance for SdscConsole {}

/// Bind [`CLASS`] into the machine graph.
///
/// # Errors
///
/// If the class name is already bound.
pub fn bind(bindings: &mut crate::machine::Bindings) -> Result<()> {
    bindings.bind(CLASS.name, |props| {
        Ok(Arc::new(SdscConsole::from_props(props)?))
    })
}

/// What the validator should know about `sms.sdsc`.
#[must_use]
pub fn schema() -> crate::machine::validate::ClassSchema {
    use crate::machine::validate::{ClassSchema, PropSchema};
    ClassSchema::new(CLASS.name)
        .prop(PropSchema::new("capacity", ValueKind::Size))
        .region(PORT_REGION)
}
