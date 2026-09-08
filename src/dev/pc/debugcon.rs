//! The hypervisor debug console: one byte-wide I/O port a firmware writes its
//! whole `DEBUG()` log to, and reads a magic byte back from to find out whether
//! anybody is listening.
//!
//! # Sources
//!
//! The port is a virtual-machine convention rather than a part anyone soldered,
//! so there is no datasheet to work from and the *consumer* is the
//! specification. That consumer is EDK II, which is BSD-2-Clause-Patent and
//! therefore a permitted reference (`CLAUDE.md`, provenance):
//!
//! * `OvmfPkg/Library/PlatformDebugLibIoPort/DebugIoPortQemu.c` — the whole
//!   detect, in one expression:
//!
//!   ```c
//!   #define BOCHS_DEBUG_PORT_MAGIC  0xE9
//!
//!   BOOLEAN
//!   EFIAPI
//!   PlatformDebugLibIoPortDetect (
//!     VOID
//!     )
//!   {
//!     return IoRead8 (PcdGet16 (PcdDebugIoPort)) == BOCHS_DEBUG_PORT_MAGIC;
//!   }
//!   ```
//!
//! * `OvmfPkg/Library/PlatformDebugLibIoPort/DebugLib.c` — the output, in
//!   `DebugPrintMarker` and again in `DebugAssert`:
//!   `IoWriteFifo8 (PcdGet16 (PcdDebugIoPort), Length, Buffer)`, guarded by
//!   `PlatformDebugLibIoPortFound ()`. `IoWriteFifo8` is `rep outsb`: `Length`
//!   **byte** writes to one port, in order, and nothing is read back between
//!   them.
//! * `OvmfPkg/Library/PlatformDebugLibIoPort/DebugLibDetect.c` and
//!   `DebugLibDetectRom.c` — who asks, and how often. The DXE instance caches
//!   the answer in a static after the first read; the SEC/`ROM` instance cannot
//!   (it runs before writable globals exist) and so re-reads the port on
//!   **every** `DEBUG()`. Both are wired to the same `PlatformDebugLibIoPortDetect`.
//! * `OvmfPkg/OvmfPkg.dec` — `gUefiOvmfPkgTokenSpaceGuid.PcdDebugIoPort|0x402|UINT16|4`,
//!   which is where `0x402` comes from. It is a build-time token, so the port
//!   is the *firmware's* choice and belongs in the machine file rather than
//!   here.
//!
//! **The interface originated in QEMU (and in Bochs before it, which is where
//! the magic byte's name comes from), and both are copyleft and were not
//! opened** (`CLAUDE.md`, provenance). Everything above is EDK II's own account
//! of what it does to the port, which is the only account this device has to
//! satisfy.
//!
//! # The protocol, complete
//!
//! ```text
//!   read  (byte)   ->  0xe9, always, with no side effect
//!   write (byte)   ->  one character of the log
//! ```
//!
//! That is the whole of it, and each half is worth stating precisely:
//!
//! * **A read answers `0xe9` unconditionally.** It is not a status register and
//!   there is nothing to poll: the firmware compares the byte against a
//!   constant and either uses the port for the rest of the boot or never
//!   touches it again. A board whose I/O space is `read-as-ones` — which
//!   `machines/q35-uefi.machine`'s is — answers `0xff` at an unmapped address,
//!   the compare fails, and the entire log is dropped. That is exactly the
//!   state `docs/platforms/q35-uefi.md` recorded before this device existed.
//! * **A write is a character and never fails.** There is no busy bit, no
//!   handshake and no back pressure in the protocol, because `rep outsb` has
//!   nowhere to put one. So a byte the host will not take is *dropped* rather
//!   than retried: the alternative is a device that stalls a guest which has no
//!   way of learning it is stalled, on a diagnostic channel. [`dropped`] counts
//!   them so a truncated log is visible rather than silent.
//!
//!   [`dropped`]: DebugConsole::dropped
//!
//! * **Byte accesses only.** `IoRead8` and `IoWriteFifo8` are both byte-wide,
//!   and the port is one address; a wider access is a decode question for the
//!   address space, not something this device should invent an answer for.
//!
//! # `MemAttrs::debug`, and why a read has anything to suppress
//!
//! Reading the port has no guest-visible effect — the answer is a constant —
//! so there is nothing for a debugger to disturb in the guest. What it *can*
//! disturb is the record of the detect: [`probes`](DebugConsole::probes) counts
//! how many times the guest asked, and a monitor that peeked at `0x402` while a
//! run was stopped would otherwise make it look as though the firmware had
//! asked once more than it did. A debug read therefore answers the same byte
//! and counts nothing, which is the contract in its narrowest useful form.
//!
//! A debug *write* is refused outright: it would put a byte into the guest's
//! log that the guest did not write.
//!
//! # Where the bytes go
//!
//! To a [`CharDevice`] named by the `port` property, the same seam a 16550's
//! transmitter uses — so the log is one more character stream the host can
//! attach a terminal to, pipe to a file, or read out of a `CharPort` in a test.
//! It is deliberately **not** the console the board's serial port is on: the
//! firmware writes both at once, and interleaving a `DEBUG()` log into a UEFI
//! shell's line editor would corrupt both. `machines/q35-uefi.machine` gives it
//! its own name, and
//!
//! ```console
//! rsemu run q35-uefi --capture debug=boot.log --flash0 … --for 1400s
//! ```
//!
//! is how a run keeps it. `--capture` rather than `--console debug`, because a
//! terminal session is paced to wall clock and a capture is not: the same log
//! costs 1 400 seconds of somebody's afternoon through a terminal and under two
//! minutes drained to a file. Every other port the board opened is drained and
//! discarded by that same loop, which is what keeps a 16550 nobody is watching
//! from filling its 64 KiB and holding `THRE` clear.

use alloc::boxed::Box;
use alloc::string::String;
use alloc::sync::Arc;
use core::fmt;

use crate::core::device::{Device, DeviceClass, PropertySpec, RealizeCtx, ResetKind};
use crate::core::error::{BusError, Result};
use crate::core::props::{Props, ValueKind};
use crate::core::space::{AccessConstraints, MemAttrs, MemOps, MemResult, Region, RegionRef};
use crate::core::state::{ChunkReader, ChunkWriter, Sink, Source};
use crate::core::sync::{LockRank, Mutex};
use crate::core::value::{Endian, Width};
use crate::host::chardev::{CharDevice, ports};
use crate::machine::realize::Instance;
use crate::machine::validate::ClassSchema;

/// The class name a machine description writes.
pub const CLASS_NAME: &str = "pc.debugcon";

/// The snapshot chunk version. Bump with the encoding, never on its own.
const STATE_VERSION: u32 = 1;

/// How much I/O space the port decodes: one byte.
///
/// `PcdDebugIoPort` is a single `UINT16` address and both accessors are
/// byte-wide, so there is no second register to decode and no mirroring to
/// describe.
pub const REGISTER_WINDOW_LEN: u64 = 1;

/// What a read answers: EDK II's `BOCHS_DEBUG_PORT_MAGIC`.
///
/// `DebugIoPortQemu.c` compares the byte it reads against exactly this, and
/// anything else means "no debug port here". It is a constant rather than a
/// property because the firmware's constant is not configurable either: a board
/// that answered something else would be a board with no debug console, which
/// is what leaving the port unmapped already says.
pub const MAGIC: u8 = 0xe9;

/// The character port a machine file gets if it names none.
const DEFAULT_PORT: &str = "debug";

/// What a snapshot carries, and nothing else.
///
/// Both counters are functions of what the *guest* did — a read the firmware
/// issued, a byte the firmware wrote — so they are the same on every host and
/// under every engine, which is what lets them into `Machine::state_hash`.
/// Whether the host was draining the far end of the port is emphatically not,
/// and [`dropped`](DebugConsole::dropped) is therefore not in here.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
struct State {
    /// How many times the guest has read the port — that is, how many times
    /// something asked whether a debug console is fitted.
    probes: u64,
    /// How many bytes the guest has written to it.
    written: u64,
}

/// The port, its counters, and the far end of the character stream.
struct Registers {
    state: Mutex<State>,
    /// Bytes the host would not take, counted but not kept.
    ///
    /// Derived rather than architectural: it depends on when the host drained
    /// the port, which is not the guest's business and not deterministic. Never
    /// serialized (`CLAUDE.md`, Devices, invariant 3).
    dropped: Mutex<u64>,
    /// Where the log goes.
    out: Arc<dyn CharDevice>,
}

impl fmt::Debug for Registers {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let mut s = f.debug_struct("Registers");
        match self.state.try_lock() {
            Some(state) => s.field("state", &*state),
            None => s.field("state", &"<in use>"),
        };
        s.finish_non_exhaustive()
    }
}

impl Registers {
    fn new(out: Arc<dyn CharDevice>) -> Registers {
        Registers {
            state: Mutex::with_rank(LockRank::DEVICE, State::default()),
            dropped: Mutex::with_rank(LockRank::LEAF, 0),
            out,
        }
    }

    /// Answer the detect. `debug` suppresses the count and nothing else,
    /// because there is nothing else: the byte is a constant.
    fn read(&self, debug: bool) -> u8 {
        if !debug {
            self.state.lock().probes += 1;
        }
        MAGIC
    }

    /// One character of the log.
    ///
    /// The counter moves whether or not the host took the byte: it is what the
    /// *guest* wrote, and a snapshot that varied with the host's draining would
    /// not be a snapshot of the machine.
    fn write(&self, byte: u8) {
        self.state.lock().written += 1;
        // Outside the critical section — the re-entrancy contract — because a
        // `CharDevice` is a host object and this is a call out of the device.
        if !self.out.write_byte(byte) {
            *self.dropped.lock() += 1;
        }
    }
}

/// The one port.
#[derive(Debug)]
struct Portmap(Arc<Registers>);

impl MemOps for Portmap {
    fn read(&self, _offset: u64, dst: &mut [u8], attrs: MemAttrs) -> MemResult {
        let [byte] = dst else {
            return Err(BusError::BadAccess);
        };
        *byte = self.0.read(attrs.debug);
        Ok(())
    }

    fn write(&self, _offset: u64, src: &[u8], attrs: MemAttrs) -> MemResult {
        let [byte] = src else {
            return Err(BusError::BadAccess);
        };
        if attrs.debug {
            // A debug write would put a character into the guest's own log that
            // the guest never wrote, which is precisely what `MemAttrs::debug`
            // exists to prevent.
            return Err(BusError::BadAccess);
        }
        self.0.write(*byte);
        Ok(())
    }

    fn constraints(&self) -> AccessConstraints {
        AccessConstraints::word(Width::U8, Endian::Little)
    }
}

/// The hypervisor debug console.
#[derive(Debug)]
pub struct DebugConsole {
    regs: Arc<Registers>,
    port: RegionRef,
    /// The character port's name, kept for [`Debug`](fmt::Debug) and for the
    /// machine's own description of itself.
    name: String,
}

impl DebugConsole {
    /// Validate `props` and build the device.
    ///
    /// # Errors
    ///
    /// [`Error::Property`](crate::core::Error::Property) if a property this
    /// class does not know was given, or one has the wrong type.
    pub fn new(props: &Props) -> Result<DebugConsole> {
        let mut r = props.reader();
        let name = r.or("port", String::from(DEFAULT_PORT))?;
        r.finish()?;
        let out = ports::attach(props, &name)?;
        Ok(DebugConsole::with_device(out, name))
    }

    /// Build one against a character device the caller already has.
    ///
    /// The unit-test and embedder door, and the same one
    /// [`Uart16550`](crate::dev::uart::ns16550::Uart16550) opens.
    #[must_use]
    pub fn with_device(out: Arc<dyn CharDevice>, name: impl Into<String>) -> DebugConsole {
        let regs = Arc::new(Registers::new(out));
        let port: RegionRef = Arc::new(Region::io(
            "pc.debugcon.regs",
            REGISTER_WINDOW_LEN,
            Arc::new(Portmap(Arc::clone(&regs))) as Arc<dyn MemOps>,
        ));
        DebugConsole {
            regs,
            port,
            name: name.into(),
        }
    }

    /// The character port this console writes to, by name.
    #[must_use]
    pub fn port_name(&self) -> &str {
        &self.name
    }

    /// How many times the guest has read the port looking for the magic byte.
    ///
    /// Non-zero is the evidence that the firmware ran `PlatformDebugLibIoPortDetect`
    /// and got an answer; zero after a boot means the log never had a chance.
    #[must_use]
    pub fn probes(&self) -> u64 {
        self.regs.state.lock().probes
    }

    /// How many bytes of log the guest has written.
    #[must_use]
    pub fn written(&self) -> u64 {
        self.regs.state.lock().written
    }

    /// How many of those the host would not take.
    ///
    /// Non-zero means the log is truncated because nobody was draining the
    /// character port fast enough — the diagnostic that keeps a short log from
    /// looking like a short boot.
    #[must_use]
    pub fn dropped(&self) -> u64 {
        *self.regs.dropped.lock()
    }
}

/// The `pc.debugcon` device class.
pub static CLASS: DeviceClass = DeviceClass {
    name: CLASS_NAME,
    version: STATE_VERSION,
    summary: "the hypervisor debug console: a byte-wide port answering 0xe9, at 0x402",
    properties: &[PropertySpec {
        name: "port",
        kind: ValueKind::Str,
        required: false,
        summary: "the character port the log goes to, by name (default \"debug\")",
    }],
    construct: |props| Ok(Box::new(DebugConsole::new(props)?)),
};

impl Device for DebugConsole {
    fn class(&self) -> &'static DeviceClass {
        &CLASS
    }

    fn realize(&self, _ctx: &mut RealizeCtx<'_>) -> Result<()> {
        Ok(())
    }

    fn reset(&self, _kind: ResetKind) {
        // The counters are a record of the run rather than machine state a
        // guest can see, but a reset starts the machine again and a firmware
        // that reboots re-runs the detect — so they go back to zero along with
        // everything else, and `probes` after a warm reset counts that boot's
        // detects rather than every boot's.
        *self.regs.state.lock() = State::default();
        *self.regs.dropped.lock() = 0;
    }

    fn flush(&self) -> Result<()> {
        // Whatever the backend is still holding, on its way out. A run that
        // ends mid-line must not lose the line.
        self.regs.out.flush();
        Ok(())
    }

    fn region(&self, name: &str) -> Option<RegionRef> {
        match name {
            "" | "regs" => Some(Arc::clone(&self.port)),
            _ => None,
        }
    }

    fn save(&self, w: &mut ChunkWriter<'_>) -> Result<()> {
        let s = *self.regs.state.lock();
        w.write_u64(s.probes)?;
        w.write_u64(s.written)
    }

    fn load(&self, r: &mut ChunkReader<'_>) -> Result<()> {
        let state = State {
            probes: r.read_u64()?,
            written: r.read_u64()?,
        };
        *self.regs.state.lock() = state;
        // `dropped` is derived and is not in the chunk; a restored machine has
        // dropped nothing yet, which is true of it.
        *self.regs.dropped.lock() = 0;
        Ok(())
    }
}

impl Instance for DebugConsole {}

/// Add [`CLASS`] to a registry.
///
/// # Errors
///
/// [`Error::Config`](crate::core::Error::Config) if the name is claimed.
pub fn register(registry: &mut crate::core::Registry) -> Result<()> {
    registry.add(&CLASS)
}

/// Bind [`CLASS_NAME`] so a machine description can instantiate it.
///
/// # Errors
///
/// [`Error::Config`](crate::core::Error::Config) if the name is bound twice.
pub fn bind(bindings: &mut crate::machine::Bindings) -> Result<()> {
    bindings.bind(CLASS_NAME, |props| Ok(Arc::new(DebugConsole::new(props)?)))
}

/// What the validator should know about this class.
#[must_use]
pub fn schema() -> ClassSchema {
    ClassSchema::new(CLASS_NAME).region("").region("regs").prop(
        crate::machine::validate::PropSchema::new("port", ValueKind::Str),
    )
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::state::{MachineShape, Migrations, StateReader, StateWriter};
    use crate::host::chardev::CharPort;
    use alloc::vec::Vec;

    fn wired() -> (DebugConsole, Arc<CharPort>) {
        let port = Arc::new(CharPort::new());
        let dev = DebugConsole::with_device(Arc::clone(&port) as Arc<dyn CharDevice>, "test");
        (dev, port)
    }

    fn read(dev: &DebugConsole, attrs: MemAttrs) -> u8 {
        let mut byte = [0u8; 1];
        Portmap(Arc::clone(&dev.regs))
            .read(0, &mut byte, attrs)
            .expect("a byte read is legal");
        byte[0]
    }

    fn write(dev: &DebugConsole, byte: u8, attrs: MemAttrs) -> MemResult {
        Portmap(Arc::clone(&dev.regs)).write(0, &[byte], attrs)
    }

    /// The detect, exactly as `DebugIoPortQemu.c` runs it.
    #[test]
    fn a_read_answers_the_magic_byte() {
        let (dev, _port) = wired();
        assert_eq!(read(&dev, MemAttrs::DEFAULT), 0xe9);
        // And keeps answering it: the SEC instance of the library re-reads the
        // port before every single `DEBUG()`, because it has no writable
        // globals to cache the answer in.
        assert_eq!(read(&dev, MemAttrs::DEFAULT), 0xe9);
        assert_eq!(dev.probes(), 2);
    }

    /// A board whose I/O space is `read-as-ones` answers `0xff`, which is the
    /// state `docs/platforms/q35-uefi.md` recorded: the compare fails and the
    /// firmware writes nothing for the rest of the boot. Written down as an
    /// assertion rather than as prose, because it is the entire difference
    /// between having this device and not having it.
    #[test]
    fn ones_is_not_the_magic_byte() {
        assert_ne!(MAGIC, 0xff);
    }

    /// `IoWriteFifo8` is `rep outsb`: bytes, in order, to one address.
    #[test]
    fn every_byte_written_reaches_the_host_in_order() {
        let (dev, port) = wired();
        for byte in b"QemuFlashDetected => Yes\n" {
            write(&dev, *byte, MemAttrs::DEFAULT).expect("a byte write is legal");
        }
        assert_eq!(port.drain(), b"QemuFlashDetected => Yes\n");
        assert_eq!(dev.written(), 25);
        assert_eq!(dev.dropped(), 0);
    }

    /// The whole point of the device, driven as the firmware drives it: the
    /// detect first, and the log only because the detect succeeded.
    #[test]
    fn the_firmwares_own_sequence_gets_a_log_out() {
        let (dev, port) = wired();
        // PlatformDebugLibIoPortDetect()
        let found = read(&dev, MemAttrs::DEFAULT) == MAGIC;
        assert!(found, "the detect has to succeed or nothing else happens");
        // DebugPrintMarker(), guarded by PlatformDebugLibIoPortFound().
        if found {
            for byte in b"Loading DXE CORE\n" {
                write(&dev, *byte, MemAttrs::DEFAULT).unwrap();
            }
        }
        assert_eq!(port.drain(), b"Loading DXE CORE\n");
    }

    #[test]
    fn a_debug_read_answers_but_does_not_count_as_a_detect() {
        let (dev, _port) = wired();
        assert_eq!(read(&dev, MemAttrs::DEBUG), 0xe9, "a monitor still sees it");
        assert_eq!(dev.probes(), 0, "but the guest has not asked yet");
        assert_eq!(read(&dev, MemAttrs::DEFAULT), 0xe9);
        assert_eq!(dev.probes(), 1);
    }

    #[test]
    fn a_debug_write_is_refused_rather_than_logged() {
        let (dev, port) = wired();
        assert!(write(&dev, b'x', MemAttrs::DEBUG).is_err());
        assert_eq!(dev.written(), 0);
        assert!(port.drain().is_empty());
    }

    /// Back pressure is not in the protocol, so a byte the host will not take
    /// is dropped rather than stalling a `rep outsb` that cannot be stalled.
    #[test]
    fn a_full_host_port_drops_bytes_instead_of_blocking() {
        let (dev, _port) = wired();
        for _ in 0..crate::host::chardev::PORT_CAPACITY {
            write(&dev, b'.', MemAttrs::DEFAULT).unwrap();
        }
        assert_eq!(dev.dropped(), 0, "the port holds exactly this much");
        write(&dev, b'!', MemAttrs::DEFAULT).expect("and the write still succeeds");
        assert_eq!(dev.dropped(), 1);
        assert_eq!(
            dev.written(),
            crate::host::chardev::PORT_CAPACITY as u64 + 1,
            "what the guest wrote is what the guest wrote"
        );
    }

    #[test]
    fn only_byte_accesses_are_answered() {
        let (dev, _port) = wired();
        let ops = Portmap(Arc::clone(&dev.regs));
        let mut two = [0u8; 2];
        assert!(ops.read(0, &mut two, MemAttrs::DEFAULT).is_err());
        assert!(ops.write(0, &two, MemAttrs::DEFAULT).is_err());
        assert_eq!(
            ops.constraints(),
            AccessConstraints::word(Width::U8, Endian::Little)
        );
    }

    #[test]
    fn a_reset_starts_the_boot_over() {
        let (dev, _port) = wired();
        read(&dev, MemAttrs::DEFAULT);
        write(&dev, b'a', MemAttrs::DEFAULT).unwrap();
        Device::reset(&dev, ResetKind::Cold);
        assert_eq!(dev.probes(), 0);
        assert_eq!(dev.written(), 0);
    }

    #[test]
    fn properties_are_checked_rather_than_ignored() {
        assert!(DebugConsole::new(&Props::new()).is_ok());
        assert!(DebugConsole::new(&Props::new().with("port", "elsewhere")).is_ok());
        assert!(DebugConsole::new(&Props::new().with("magic", 0xe9u64)).is_err());
    }

    #[test]
    fn the_regions_it_publishes_are_the_ones_the_schema_names() {
        let (dev, _port) = wired();
        assert!(Device::region(&dev, "").is_some());
        assert!(Device::region(&dev, "regs").is_some());
        assert!(Device::region(&dev, "data").is_none());
        assert_eq!(dev.port_name(), "test");
    }

    /// The round trip `CLAUDE.md` asks of every stateful device, asserted as
    /// two identical images rather than field by field.
    #[test]
    fn a_snapshot_round_trips_every_bit_of_architectural_state() {
        let image = |dev: &DebugConsole| -> Vec<u8> {
            let mut shape = MachineShape::new();
            shape.add_device("debugcon", CLASS.name).unwrap();
            let mut out = StateWriter::new(shape);
            {
                let mut chunk = out.chunk("debugcon", CLASS.name, CLASS.version).unwrap();
                Device::save(dev, &mut chunk).unwrap();
            }
            out.to_vec().unwrap()
        };

        let (dev, port) = wired();
        read(&dev, MemAttrs::DEFAULT);
        read(&dev, MemAttrs::DEFAULT);
        for byte in b"ASSERT NvmExpressHci.c(778)\n" {
            write(&dev, *byte, MemAttrs::DEFAULT).unwrap();
        }
        let _ = port.drain();

        let first = image(&dev);
        let (restored, _other) = wired();
        let reader = StateReader::new(&first).unwrap();
        let chunk = reader
            .load("debugcon", CLASS.name, CLASS.version, &Migrations::new())
            .unwrap();
        Device::load(&restored, &mut chunk.reader()).unwrap();

        assert_eq!(image(&restored), first, "the two images are identical");
        assert_eq!(restored.probes(), 2, "the detect count came back");
        assert_eq!(restored.written(), 28, "and so did the byte count");
        assert_eq!(restored.dropped(), 0, "while the derived counter did not");
    }
}
