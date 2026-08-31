//! The firmware socket: where a PC's BIOS lives, and how a user's own binary
//! gets there.
//!
//! rsemu ships no BIOS. A PC's firmware is whatever the person running the
//! machine points at — their copy of SeaBIOS, a vendor image dumped off a real
//! board, a hand-written test stub — and it is bound to a **media slot**, the
//! same mechanism the RISC-V board loads its firmware through:
//!
//! ```console
//! $ rsemu run pc-at --bios /usr/share/qemu/bios.bin
//! $ rsemu run pc-at --media bios=/path/to/bios.bin --media vgabios=/path/to/vgabios.bin
//! ```
//!
//! Nothing is vendored, and nothing is downloaded: the bytes come from the
//! caller (`machine::realize`'s media table), the emulation core never opens a
//! file, and a slot nothing is bound to is an error naming the slot rather than
//! a machine that quietly comes up with no firmware.
//!
//! # Why this is a ROM socket and not a loader
//!
//! `riscv.loader` writes its image into DRAM at reset, because a RISC-V board
//! has no mask ROM and its firmware genuinely lives in RAM. A PC's does not: the
//! BIOS is in a ROM the chipset decodes at the top of the address space, the
//! guest can read it forever, and — this is the part that matters — **a PC's
//! firmware writes to its own address range on purpose**. Shadowing, the
//! chipset registers that make `0xe0000-0xfffff` writable so the BIOS can copy
//! itself into RAM, and the option-ROM checksum walk all touch it. A device that
//! had written bytes into RAM and gone away could model none of that.
//!
//! So this is a [`Region::rom`](crate::core::space::Region::rom): reads come
//! straight from the store, and a write is **ignored, not faulted**. A PC with
//! no chipset shadow control has ROM in those sockets and a write to ROM does
//! nothing at all — which is exactly what firmware that probes for shadow RAM
//! expects to find when there is none.
//!
//! # The image, the socket, and which end it sits at
//!
//! A socket has a fixed size, set by `size`, and `align` says which end a
//! shorter image lands at. The two answers are not a preference — they are two
//! different kinds of ROM:
//!
//! ```text
//!   align = "top"     size = 128K, image = 64K
//!       0x00000-0x0ffff erased, 0x10000-0x1ffff image
//!   align = "bottom"  size = 64K,  image = 38K
//!       0x00000-0x097ff image, 0x09800-0x0ffff erased
//! ```
//!
//! **The system BIOS is top-aligned**, because an x86 starts executing at the
//! *top* of its address space (`0xffff:0x0000` on an 8086, `0xfffffff0` on a
//! 386): whatever is in the last bytes of the socket is what runs first. A 64
//! KiB image bottom-aligned in a 128 KiB window would put erased bytes under
//! the reset vector, and the machine would execute `0xff` — `INC BYTE PTR
//! [BX+DI]` — forever, with nothing to say why.
//!
//! **An option ROM is bottom-aligned**, because firmware finds one by scanning
//! for the `0x55 0xaa` signature on a 2 KiB boundary and then trusting the
//! length byte that follows it. A video BIOS top-aligned in its window would
//! put that signature somewhere the scan never looks.
//!
//! `top` is the default, because getting the *system* BIOS wrong is the failure
//! with no diagnostic.
//!
//! The machine file then maps the socket wherever the board decodes it, and may
//! map it **twice**: a 386 fetches its first instruction from `0xfffffff0`,
//! sixteen bytes below 4 GiB, while every real-mode `far jmp` afterwards lands
//! at `0x000f0000-0x000fffff`. Both windows are the same chip, which is why a
//! `map` statement can name this region more than once.

use alloc::boxed::Box;
use alloc::format;
use alloc::sync::Arc;

use crate::core::device::{Device, DeviceClass, PropertySpec, RealizeCtx, ResetKind};
use crate::core::error::{Error, Result};
use crate::core::props::{Props, ValueKind};
use crate::core::space::{Region, RegionRef, RomStore, RomWrite};
use crate::machine::realize::Instance;
use crate::machine::validate::{ClassSchema, PropSchema};

/// The class name a machine description writes.
pub const CLASS_NAME: &str = "pc.rom";

/// What an unprogrammed cell reads as, and what a short image is padded with.
///
/// `0xff` because that is an erased EPROM byte, and because firmware that
/// checksums an option ROM header looks for `0x55 0xaa` — which erased flash
/// never accidentally is.
const ERASED: u8 = 0xff;

/// The socket size a machine file gets if it names none: 128 KiB, the AT's
/// `0xe0000-0xfffff` system ROM window.
const DEFAULT_SIZE: u64 = 128 * 1024;

/// The largest socket this class will allocate.
///
/// Not a hardware limit — it is a sanity bound, so that `size = 4G` in a
/// machine file is an error naming the property rather than an allocation that
/// takes the host down.
const MAX_SIZE: u64 = 16 * 1024 * 1024;

/// Which end of the socket a short image sits at.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Align {
    /// Against the top, under the reset vector. What a system BIOS needs.
    Top,
    /// Against the bottom, where an option-ROM scan looks for `0x55 0xaa`.
    Bottom,
}

/// A firmware ROM socket.
#[derive(Debug)]
pub struct FirmwareRom {
    store: Arc<RomStore>,
    region: RegionRef,
    /// How many bytes of the socket the image actually filled, for diagnostics
    /// and for `rsemu`'s summary line.
    image_len: u64,
}

impl FirmwareRom {
    /// Validate `props` and build the device.
    ///
    /// # Errors
    ///
    /// [`Error::Property`] if the `image` media slot is missing, if its image
    /// does not fit the socket, or if a property this class does not know was
    /// given.
    pub fn new(props: &Props) -> Result<FirmwareRom> {
        let mut r = props.reader();
        let image = r.require_media("image")?.to_bytes();
        let size = r.or_size("size", DEFAULT_SIZE)?;
        let align = r.or_enum("align", "top", &["top", "bottom"])?;
        let align = if align == "bottom" {
            Align::Bottom
        } else {
            Align::Top
        };
        r.finish()?;
        FirmwareRom::from_image(&image, size, align)
    }

    /// Build one from bytes directly.
    ///
    /// # Errors
    ///
    /// [`Error::Property`] if `size` is zero or implausibly large, or if the
    /// image does not fit.
    pub fn from_image(image: &[u8], size: u64, align: Align) -> Result<FirmwareRom> {
        if size == 0 || size > MAX_SIZE {
            return Err(Error::Property(format!(
                "property `size`: a firmware socket holds between 1 and {MAX_SIZE} bytes, not \
                 {size}"
            )));
        }
        let len = image.len() as u64;
        if len > size {
            return Err(Error::Property(format!(
                "property `image`: this socket is {size} bytes and the image is {len}; give the \
                 object a larger `size`, and map it over a larger window"
            )));
        }
        let mut bytes = alloc::vec![ERASED; size as usize];
        let at = match align {
            // An x86 fetches its first instruction from the top of the socket,
            // so a short system BIOS must end where the reset vector is.
            Align::Top => (size - len) as usize,
            // An option ROM is found by a scan for `0x55 0xaa` from the bottom
            // of its window upward, so it must start there.
            Align::Bottom => 0,
        };
        bytes[at..at + len as usize].copy_from_slice(image);
        let store = Arc::new(RomStore::new(bytes));
        let region: RegionRef = Arc::new(Region::rom(
            CLASS_NAME,
            Arc::clone(&store),
            // Ignored, not faulted: a write to a ROM socket that has no shadow
            // control behind it does nothing on real hardware, and firmware
            // probing for shadow RAM writes to itself to find out.
            RomWrite::Ignore,
        ));
        Ok(FirmwareRom {
            store,
            region,
            image_len: len,
        })
    }

    /// How many bytes the socket holds.
    #[must_use]
    pub fn len(&self) -> u64 {
        self.store.len()
    }

    /// Whether the socket holds no bytes — it never does; `new` refuses a zero
    /// size.
    #[must_use]
    pub fn is_empty(&self) -> bool {
        self.store.len() == 0
    }

    /// How many bytes of the socket the image filled.
    #[must_use]
    pub fn image_len(&self) -> u64 {
        self.image_len
    }

    /// The backing store, for a test or a debugger that wants the bytes.
    #[must_use]
    pub fn store(&self) -> &Arc<RomStore> {
        &self.store
    }
}

/// The `pc.rom` device class.
pub static CLASS: DeviceClass = DeviceClass {
    name: CLASS_NAME,
    version: 1,
    summary: "a firmware ROM socket: a user-supplied BIOS or option ROM image",
    properties: &[
        PropertySpec {
            name: "image",
            kind: ValueKind::Media,
            required: true,
            summary: "the media slot the image is bound to (`--bios`, `--media vgabios=…`)",
        },
        PropertySpec {
            name: "size",
            kind: ValueKind::Size,
            required: false,
            summary: "how many bytes the socket decodes (default 128K)",
        },
        PropertySpec {
            name: "align",
            kind: ValueKind::Str,
            required: false,
            summary: "\"top\" for a system BIOS under the reset vector, \"bottom\" for an option ROM",
        },
    ],
    construct: |props| Ok(Box::new(FirmwareRom::new(props)?)),
};

impl Device for FirmwareRom {
    fn class(&self) -> &'static DeviceClass {
        &CLASS
    }

    fn realize(&self, _ctx: &mut RealizeCtx<'_>) -> Result<()> {
        // Nothing outward: a `map` statement places the region, and the
        // realizer does that after every device has realized.
        Ok(())
    }

    fn reset(&self, _kind: ResetKind) {
        // A ROM is a ROM. There is no state here to return to — the image was
        // fixed at construction, and a cold reset does not re-read a socket.
    }

    fn region(&self, name: &str) -> Option<RegionRef> {
        matches!(name, "" | "rom").then(|| Arc::clone(&self.region))
    }

    // No `save`/`load`. The contents cannot change, and writing 128 KiB of
    // unchanging bytes into every snapshot would be a cost with no benefit —
    // `ROADMAP.md` §4.5 says devices serialize architectural state, and a mask
    // ROM's architectural state is the image the machine was built with.
}

impl Instance for FirmwareRom {}

/// Add [`CLASS`] to a registry.
///
/// # Errors
///
/// [`Error::Config`] if something already claimed the name.
pub fn register(registry: &mut crate::core::Registry) -> Result<()> {
    registry.add(&CLASS)
}

/// Bind [`CLASS`] into the machine graph.
///
/// # Errors
///
/// [`Error::Config`] if the class is already bound.
pub fn bind(bindings: &mut crate::machine::Bindings) -> Result<()> {
    bindings.bind(CLASS_NAME, |props| Ok(Arc::new(FirmwareRom::new(props)?)))
}

/// What the validator should know about `pc.rom`.
#[must_use]
pub fn schema() -> ClassSchema {
    ClassSchema::new(CLASS_NAME)
        .prop(PropSchema::new("image", ValueKind::Media).required())
        .prop(PropSchema::new("size", ValueKind::Size))
        .prop(PropSchema::new("align", ValueKind::Str).values(&["top", "bottom"]))
        .region("")
        .region("rom")
}

#[cfg(test)]
mod tests {
    use super::*;
    use alloc::vec::Vec;

    /// A socket's worth of bytes, for a test that wants to know what is where.
    fn socket_bytes(rom: &FirmwareRom) -> Vec<u8> {
        let mut out = alloc::vec![0u8; rom.len() as usize];
        rom.store.read_at(0, &mut out).expect("the whole socket");
        out
    }
    use crate::core::space::{AddressSpace, MemAttrs};
    use crate::core::value::Width;

    #[test]
    fn a_full_image_fills_the_socket() {
        let image: Vec<u8> = (0..256u32).map(|i| i as u8).collect();
        let rom = FirmwareRom::from_image(&image, 256, Align::Top).expect("it fits exactly");
        assert_eq!(socket_bytes(&rom), image);
        assert_eq!(rom.image_len(), 256);
    }

    #[test]
    fn a_short_image_lands_at_the_top_where_the_reset_vector_is() {
        // The property the whole class turns on: an x86 fetches from the top
        // of the socket, so a 16-byte image in a 64-byte socket must end at
        // offset 63 and not at offset 15.
        let image = [0xeau8; 16];
        let rom = FirmwareRom::from_image(&image, 64, Align::Top).expect("it fits");
        let bytes = socket_bytes(&rom);
        assert_eq!(&bytes[..48], &[ERASED; 48], "the unprogrammed half");
        assert_eq!(&bytes[48..], &image, "the image, ending at the top");
    }

    #[test]
    fn a_bottom_aligned_image_starts_where_an_option_rom_scan_looks() {
        // A video BIOS is found by a scan for `0x55 0xaa` from the bottom of
        // its window; top-aligning one would hide the signature.
        let image = [0x55u8, 0xaa, 0x4c];
        let rom = FirmwareRom::from_image(&image, 64, Align::Bottom).expect("it fits");
        let bytes = socket_bytes(&rom);
        assert_eq!(&bytes[..3], &image, "the signature is at the bottom");
        assert_eq!(&bytes[3..], &[ERASED; 61], "the rest is unprogrammed");
    }

    #[test]
    fn an_image_larger_than_the_socket_is_refused_by_name() {
        let e = FirmwareRom::from_image(&[0u8; 128], 64, Align::Top)
            .expect_err("128 bytes do not fit in 64")
            .to_string();
        assert!(e.contains("image"), "{e}");
        assert!(e.contains("64"), "{e}");
    }

    #[test]
    fn an_implausible_socket_is_refused_by_name() {
        let e = FirmwareRom::from_image(&[], 1 << 40, Align::Top)
            .expect_err("a terabyte of ROM")
            .to_string();
        assert!(e.contains("size"), "{e}");
    }

    #[test]
    fn a_write_to_rom_is_ignored_rather_than_faulted() {
        // Firmware probing for shadow RAM writes to its own address range and
        // reads it back. On a board with no shadow control the write does
        // nothing and the read still returns the ROM — a bus fault here would
        // send that probe down a path no real machine takes.
        let rom = FirmwareRom::from_image(&[0x55, 0xaa], 2, Align::Top).expect("it fits");
        let space = AddressSpace::new("mem", 20);
        space
            .topology()
            .map(rom.region("").expect("the socket's region"), 0)
            .expect("nothing else is mapped");
        space
            .write(0, Width::U8, 0x00, MemAttrs::DEFAULT)
            .expect("a write to ROM is swallowed, not refused");
        assert_eq!(
            space.read(0, Width::U8, MemAttrs::DEFAULT),
            Ok(0x55),
            "the ROM is unchanged"
        );
    }

    #[test]
    fn the_class_constructs_from_properties() {
        let mut props = Props::new();
        props.insert(
            "image",
            crate::core::props::Media::new("bios", alloc::vec![0x90u8; 4]),
        );
        props.insert("size", crate::core::props::Value::Size(16));
        let dev = (CLASS.construct)(&props).expect("a socket");
        assert_eq!(dev.class().name, CLASS_NAME);
    }
}
