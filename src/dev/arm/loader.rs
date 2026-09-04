//! Putting a kernel or a ramdisk into guest memory, and checking the one that
//! has a header.
//!
//! An AArch64 `virt` board has no mask ROM holding an operating system: what
//! runs is whatever the person running the machine points at, and it lives in
//! DRAM before the first instruction executes. Real hardware achieves that
//! with a boot medium and a first-stage loader; an emulator achieves it by
//! writing the bytes in, and pretending otherwise would add a SPI flash model
//! nothing would ever read twice.
//!
//! So this is a device with no registers. It publishes no region, drives no
//! wire and answers no access. What it does is **write its image into the
//! address space at reset**, which is the moment the machine says "everything
//! is back to how it starts". `dev::riscv::loader` makes the same argument at
//! the same length and this is the same device; what is new here is the header
//! check below.
//!
//! # The `Image` header
//!
//! An AArch64 kernel is a flat binary with a 64-byte header, and `format =
//! "arm64"` makes this loader read it:
//!
//! ```text
//!   0x00  code0        an instruction; its opcode bytes are also "MZ"
//!   0x04  code1        a branch to the entry point
//!   0x08  text_offset  where in RAM the image wants to be, from a 2 MiB base
//!   0x10  image_size   how much memory the loaded image occupies
//!   0x18  flags        bit 0 endianness, bits 2:1 page size, bit 3 placement
//!   0x20  res2, res3, res4
//!   0x38  magic        0x644d5241, the four bytes "ARM\x64"
//!   0x3c  res5
//! ```
//!
//! Every field is little-endian whatever the guest is. `image_size == 0` is an
//! old kernel that does not declare one, and the load offset is then 0x80000
//! by convention.
//!
//! **Provenance.** This layout is stated in the Linux kernel's own boot
//! documentation, which is GPL-2.0 and was **not** read. It was taken instead
//! from two permissive sources that implement it and agree field for field:
//! the ARM **boot-wrapper** (`scripts/AA64Image.pm`, BSD-3-Clause, "Copyright
//! (c) 2012, ARM Limited"), which unpacks exactly this structure and states
//! the 0x80000 default; and **TianoCore EDK II**
//! (`ArmVirtPkg/ArmVirtQemuKernel.fdf`, BSD-2-Clause-Patent), which emits the
//! header byte for byte with these field names. The `flags` bit 0 and bits 2:1
//! meanings were cross-checked against **crosvm** (BSD-3-Clause) and **Zephyr**
//! and **Apache NuttX** (Apache-2.0). What no permissive source states — and
//! what is therefore *not* acted on here — is the meaning of `flags` bit 3 and
//! the claim that `res5` is a PE/COFF header offset; both are read and
//! reported and neither changes what this loader does.
//!
//! # Why a check and not a computation
//!
//! The loader could compute the load address from `text_offset` and ignore
//! what the machine file said. It does not, because the *boot ROM* also has to
//! know where to jump, and a load address that appeared in one place and an
//! entry address that appeared in another would be two numbers that could
//! disagree silently. So the machine file says the address once, both objects
//! read it, and a header that wants a different one is a **build failure that
//! names both numbers** rather than a machine that boots into zeroes.

use alloc::boxed::Box;
use alloc::format;
use alloc::string::{String, ToString};
use alloc::sync::Arc;

use crate::core::device::{Device, DeviceClass, PropertySpec, RealizeCtx, ResetKind};
use crate::core::error::{Error, Result};
use crate::core::props::{Props, ValueKind};
use crate::core::space::{AddressSpace, MemAttrs, RequesterId};
use crate::core::sync::{LockRank, Mutex};
use crate::machine::realize::{BindCtx, Instance};

/// The class name a machine description writes.
pub const CLASS_NAME: &str = "arm.loader";

/// The address an image lands at when a machine file does not say: 2 MiB into
/// the conventional DRAM base, which is a 2 MiB-aligned address and therefore
/// the one a kernel with `text_offset = 0` asks for.
pub const DEFAULT_ADDR: u64 = 0x4020_0000;

/// The four bytes at offset 0x38 of an AArch64 `Image`: "ARM\x64".
pub const IMAGE_MAGIC: u32 = 0x644d_5241;

/// How long the header is.
pub const HEADER_LEN: usize = 64;

/// The load offset assumed when a kernel declares no `image_size`.
pub const DEFAULT_TEXT_OFFSET: u64 = 0x0008_0000;

/// The alignment the base address an image is placed from must have.
pub const BASE_ALIGN: u64 = 2 * 1024 * 1024;

/// What an `Image` header says about itself.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ImageHeader {
    /// Where the image wants to sit, measured from a 2 MiB-aligned base.
    pub text_offset: u64,
    /// How much memory the loaded image occupies, or zero if it does not say.
    pub image_size: u64,
    /// The flags word, kept whole: only bit 0 is acted on.
    pub flags: u64,
}

impl ImageHeader {
    /// Parse `bytes` as an AArch64 `Image` header.
    ///
    /// # Errors
    ///
    /// [`Error::Config`] if it is too short, if the magic is wrong, or if the
    /// kernel is big-endian — which this core is not and cannot pretend to be.
    pub fn parse(bytes: &[u8]) -> Result<ImageHeader> {
        let bad = |message: String| Error::Config {
            at: CLASS_NAME.to_string(),
            message,
        };
        if bytes.len() < HEADER_LEN {
            return Err(bad(format!(
                "an AArch64 `Image` starts with a {HEADER_LEN}-byte header and this image is \
                 {} byte(s) long",
                bytes.len()
            )));
        }
        let word = |at: usize| {
            u32::from_le_bytes([bytes[at], bytes[at + 1], bytes[at + 2], bytes[at + 3]])
        };
        let dword = |at: usize| u64::from(word(at)) | (u64::from(word(at + 4)) << 32);
        let magic = word(0x38);
        if magic != IMAGE_MAGIC {
            return Err(bad(format!(
                "this is not an AArch64 `Image`: the four bytes at offset 0x38 are {magic:#010x} \
                 and an `Image` has {IMAGE_MAGIC:#010x} (\"ARM\\x64\"). A `vmlinuz` or a `.gz` \
                 has to be decompressed first, and a bzImage is a different architecture's \
                 format entirely"
            )));
        }
        let flags = dword(0x18);
        if flags & 1 != 0 {
            return Err(bad(String::from(
                "this kernel is big-endian (`Image` header flags bit 0) and `cpu.arm.a64` \
                 executes little-endian only",
            )));
        }
        let image_size = dword(0x10);
        let text_offset = if image_size == 0 {
            // An old kernel that declares no size also declares no usable
            // offset; 0x80000 is the value every loader assumes for it.
            DEFAULT_TEXT_OFFSET
        } else {
            dword(0x08)
        };
        Ok(ImageHeader {
            text_offset,
            image_size,
            flags,
        })
    }

    /// Whether an image placed at `addr` lands where this header asked to be.
    ///
    /// The base is `addr` rounded *down* to 2 MiB, because that is the base a
    /// loader would have chosen for it: the kernel wants to be `text_offset`
    /// bytes past a 2 MiB-aligned address, so the only thing that has to match
    /// is where in a 2 MiB block it lands.
    #[must_use]
    pub fn wants(&self, addr: u64) -> bool {
        addr % BASE_ALIGN == self.text_offset % BASE_ALIGN
    }

    /// The lowest address at or above `floor` this image can be placed at.
    ///
    /// What the error message hands the reader, so a kernel that wants a
    /// different offset from the one a board defaults to is one `-p` away
    /// rather than an afternoon of arithmetic.
    #[must_use]
    pub fn placement(&self, floor: u64) -> u64 {
        let base = floor.next_multiple_of(BASE_ALIGN);
        let at = base + self.text_offset % BASE_ALIGN;
        if at >= floor { at } else { at + BASE_ALIGN }
    }
}

/// What the loader learned when the machine bound it.
#[derive(Debug, Default)]
struct Binding {
    space: Option<Arc<AddressSpace>>,
    requester: RequesterId,
    /// What the last load failed with, if it did. A `reset` cannot return an
    /// error, so the failure is kept here and surfaced by
    /// [`Loader::last_error`] — silence would give a machine that boots to
    /// zeroed memory with no explanation.
    error: Option<String>,
    /// How many times the image has been written in, for tests.
    loads: u64,
}

/// An image and the address it is written to.
#[derive(Debug)]
pub struct Loader {
    image: Arc<[u8]>,
    addr: u64,
    space_name: Option<String>,
    /// The header, if this image was declared to have one.
    header: Option<ImageHeader>,
    binding: Mutex<Binding>,
}

impl Loader {
    /// Validate `props` and build the device.
    ///
    /// # Errors
    ///
    /// [`Error::Property`] if the `image` media slot is missing or a property
    /// this class does not know was given, and [`Error::Config`] if `format =
    /// "arm64"` and the bytes are not an `Image` that wants to be where the
    /// machine file put it.
    pub fn new(props: &Props) -> Result<Loader> {
        let mut r = props.reader();
        let image = r.require_media("image")?.to_bytes();
        let addr = r.or_addr("addr", DEFAULT_ADDR)?;
        let space = r.optional_str("space")?.map(ToString::to_string);
        let format = r.or_enum("format", "raw", &["raw", "arm64"])?.to_string();
        r.finish()?;

        // An unbound slot is empty bytes, and an empty image is not an error
        // even when a format was declared: that is what lets one machine file
        // carry a kernel slot a bare-metal run simply does not fill.
        let header = if format == "arm64" && !image.is_empty() {
            let header = ImageHeader::parse(&image)?;
            if !header.wants(addr) {
                return Err(Error::Config {
                    at: CLASS_NAME.to_string(),
                    message: format!(
                        "this kernel's header asks to be loaded {:#x} byte(s) past a 2 MiB \
                         boundary and the machine file puts it at {addr:#x}, which is {:#x} \
                         past one. The boot ROM jumps to where the machine file says, so a \
                         kernel placed anywhere else would be entered at the wrong \
                         instruction — try `-p kernel-addr={:#x}`",
                        header.text_offset,
                        addr % BASE_ALIGN,
                        header.placement(addr)
                    ),
                });
            }
            Some(header)
        } else {
            None
        };
        Ok(Loader {
            image,
            addr,
            space_name: space,
            header,
            binding: Mutex::with_rank(LockRank::DEVICE, Binding::default()),
        })
    }

    /// Build one from bytes directly.
    #[must_use]
    pub fn from_image(image: impl Into<Arc<[u8]>>, addr: u64, space: Option<String>) -> Loader {
        Loader {
            image: image.into(),
            addr,
            space_name: space,
            header: None,
            binding: Mutex::with_rank(LockRank::DEVICE, Binding::default()),
        }
    }

    /// Where the image is written.
    #[must_use]
    pub fn addr(&self) -> u64 {
        self.addr
    }

    /// How many bytes it writes.
    #[must_use]
    pub fn len(&self) -> u64 {
        self.image.len() as u64
    }

    /// Whether there is nothing to write.
    #[must_use]
    pub fn is_empty(&self) -> bool {
        self.image.is_empty()
    }

    /// The `Image` header, if this loader was told to expect one.
    #[must_use]
    pub fn header(&self) -> Option<ImageHeader> {
        self.header
    }

    /// What the last load failed with, if it did.
    #[must_use]
    pub fn last_error(&self) -> Option<String> {
        self.binding.lock().error.clone()
    }

    /// How many times the image has been written into memory.
    #[must_use]
    pub fn loads(&self) -> u64 {
        self.binding.lock().loads
    }

    /// Write the image into `space` now.
    ///
    /// # Errors
    ///
    /// Whatever the address space refuses — which for a kernel almost always
    /// means the machine has less RAM than the image needs, or the image was
    /// aimed at an address nothing answers.
    pub fn load_into(&self, space: &AddressSpace, requester: RequesterId) -> Result<()> {
        if self.image.is_empty() {
            return Ok(());
        }
        let attrs = MemAttrs::DEFAULT
            .with_requester(requester)
            .with_privileged(true);
        // In page-sized pieces, so a failure names the address it happened at
        // rather than the start of a 30 MiB kernel.
        const CHUNK: usize = 4096;
        let mut at = self.addr;
        for piece in self.image.chunks(CHUNK) {
            space
                .write_bytes(at, piece, attrs)
                .map_err(|e| Error::Config {
                    at: CLASS_NAME.to_string(),
                    message: format!(
                        "cannot write the image at {at:#x} in space `{}`: {e} — the machine \
                         needs {} byte(s) of memory from {:#x}{}",
                        space.name(),
                        self.image.len(),
                        self.addr,
                        match self.header {
                            // A kernel needs room for its BSS as well as for
                            // its bytes, and `image_size` is the only thing
                            // that says how much.
                            Some(h) if h.image_size > self.image.len() as u64 => format!(
                                ", and {} byte(s) of it once it has decompressed and cleared \
                                 its BSS",
                                h.image_size
                            ),
                            _ => String::new(),
                        }
                    ),
                })?;
            at += piece.len() as u64;
        }
        Ok(())
    }
}

/// The `arm.loader` device class.
pub static CLASS: DeviceClass = DeviceClass {
    name: CLASS_NAME,
    version: 1,
    summary: "writes a media image into guest memory at reset: a kernel, a ramdisk, a blob",
    properties: &[
        PropertySpec {
            name: "image",
            kind: ValueKind::Media,
            required: true,
            summary: "the image, as the name of a media slot (`image = \"kernel\"`)",
        },
        PropertySpec {
            name: "addr",
            kind: ValueKind::Addr,
            required: false,
            summary: "where the first byte lands (default 0x40200000)",
        },
        PropertySpec {
            name: "space",
            kind: ValueKind::Str,
            required: false,
            summary: "which address space to write into, if not the one the object declares",
        },
        PropertySpec {
            name: "format",
            kind: ValueKind::Str,
            required: false,
            summary: "`raw`, or `arm64` to check the AArch64 `Image` header before loading",
        },
    ],
    construct: |props| Ok(Box::new(Loader::new(props)?)),
};

impl Device for Loader {
    fn class(&self) -> &'static DeviceClass {
        &CLASS
    }

    fn realize(&self, _ctx: &mut RealizeCtx<'_>) -> Result<()> {
        // Nothing outward: the image goes in at reset, once the memory it
        // lands in has been cleared. See the module docs.
        Ok(())
    }

    fn reset(&self, _kind: ResetKind) {
        let (space, requester) = {
            let binding = self.binding.lock();
            (binding.space.clone(), binding.requester)
        };
        let Some(space) = space else {
            return;
        };
        let result = self.load_into(&space, requester);
        let mut binding = self.binding.lock();
        binding.loads += 1;
        binding.error = result.err().map(|e| e.to_string());
    }

    // No `save`/`load`: the image is the media the caller bound, and a
    // snapshot that carried it would store the kernel in every save state
    // (`ROADMAP.md` §4.5).
}

impl Instance for Loader {
    fn bind(&self, ctx: &BindCtx<'_>) -> Result<()> {
        let space = match &self.space_name {
            Some(name) => ctx.space_named(name).ok_or_else(|| Error::Config {
                at: ctx.path().to_string(),
                message: format!("no address space named `{name}`"),
            })?,
            None => ctx.space().ok_or_else(|| Error::Config {
                at: ctx.path().to_string(),
                message: String::from(
                    "a loader needs an address space to write into (`space = mem`)",
                ),
            })?,
        };
        // Fail here rather than at reset, where nothing could report it: a
        // machine whose kernel does not fit should not build at all.
        self.load_into(space, ctx.requester())?;
        let mut binding = self.binding.lock();
        binding.space = Some(Arc::clone(space));
        binding.requester = ctx.requester();
        Ok(())
    }
}

/// Bind [`CLASS`] into the machine graph.
///
/// # Errors
///
/// [`Error::Config`] if the class is already bound.
pub fn bind(bindings: &mut crate::machine::Bindings) -> Result<()> {
    bindings.bind(CLASS_NAME, |props| Ok(Arc::new(Loader::new(props)?)))
}

/// Add [`CLASS`] to a registry.
///
/// # Errors
///
/// [`Error::Config`] if something already claimed the name.
pub fn register(registry: &mut crate::core::Registry) -> Result<()> {
    registry.add(&CLASS)
}

/// What the validator should know about `arm.loader`.
#[must_use]
pub fn schema() -> crate::machine::validate::ClassSchema {
    use crate::machine::validate::{ClassSchema, PropSchema};
    ClassSchema::new(CLASS_NAME)
        .prop(PropSchema::new("image", ValueKind::Media).required())
        .prop(PropSchema::new("addr", ValueKind::Addr))
        .prop(PropSchema::new("space", ValueKind::Str))
        .prop(PropSchema::new("format", ValueKind::Str).values(&["raw", "arm64"]))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::space::{RamStore, Region};
    use crate::core::value::Width;
    use alloc::vec::Vec;

    fn space_with_ram(len: u64) -> Arc<AddressSpace> {
        let space = AddressSpace::new("mem", 64);
        let ram = Arc::new(RamStore::new(len));
        space
            .topology()
            .map(Region::ram("ram", ram), 0x4000_0000)
            .expect("a fresh space");
        Arc::new(space)
    }

    fn peek(space: &AddressSpace, addr: u64) -> u8 {
        space
            .read(addr, Width::U8, MemAttrs::DEBUG)
            .expect("mapped") as u8
    }

    /// A minimal `Image`: the header, and nothing else.
    fn image(text_offset: u64, image_size: u64, flags: u64) -> Vec<u8> {
        let mut out = alloc::vec![0u8; HEADER_LEN];
        out[0x08..0x10].copy_from_slice(&text_offset.to_le_bytes());
        out[0x10..0x18].copy_from_slice(&image_size.to_le_bytes());
        out[0x18..0x20].copy_from_slice(&flags.to_le_bytes());
        out[0x38..0x3c].copy_from_slice(&IMAGE_MAGIC.to_le_bytes());
        out
    }

    #[test]
    fn the_image_lands_where_it_was_aimed() {
        let space = space_with_ram(0x1000);
        let loader = Loader::from_image(&b"\x01\x02\x03"[..], 0x4000_0000, None);
        loader
            .load_into(&space, RequesterId::ANONYMOUS)
            .expect("three bytes fit");
        assert_eq!(peek(&space, 0x4000_0000), 1);
        assert_eq!(peek(&space, 0x4000_0002), 3);
        assert_eq!(loader.len(), 3);
    }

    #[test]
    fn a_header_that_is_not_an_image_says_so_and_says_what_to_do() {
        let e = ImageHeader::parse(&[0u8; HEADER_LEN])
            .expect_err("no magic")
            .to_string();
        assert!(e.contains("ARM"), "{e}");
        assert!(e.contains("bzImage"), "the other format people try: {e}");
    }

    #[test]
    fn a_kernel_that_declares_no_size_gets_the_conventional_offset() {
        // `image_size == 0` is an old kernel, and its `text_offset` field
        // cannot be trusted: 0x80000 is what every loader assumes instead.
        let header = ImageHeader::parse(&image(0xdead_beef, 0, 0)).expect("an Image");
        assert_eq!(header.text_offset, DEFAULT_TEXT_OFFSET);
        assert_eq!(header.image_size, 0);
    }

    #[test]
    fn a_big_endian_kernel_is_refused_rather_than_run_backwards() {
        let e = ImageHeader::parse(&image(0x80000, 0x1000, 1))
            .expect_err("bit 0 of flags")
            .to_string();
        assert!(e.contains("big-endian"), "{e}");
    }

    #[test]
    fn a_header_that_wants_a_different_address_is_a_build_failure_naming_both() {
        // The check that stops a kernel being entered at the wrong
        // instruction: the boot ROM jumps where the machine file says.
        let header = ImageHeader::parse(&image(0x80000, 0x1000, 0)).expect("an Image");
        assert!(header.wants(0x4008_0000), "512 KiB past a 2 MiB boundary");
        assert!(header.wants(0x4028_0000), "and past the next one");
        assert!(!header.wants(0x4000_0000), "the base itself is not it");
    }

    #[test]
    fn an_image_that_does_not_fit_says_where_and_how_big() {
        let space = space_with_ram(0x100);
        let loader = Loader::from_image(alloc::vec![0xaau8; 0x400], 0x4000_0000, None);
        let e = loader
            .load_into(&space, RequesterId::ANONYMOUS)
            .expect_err("1 KiB into 256 bytes")
            .to_string();
        assert!(e.contains("1024"), "{e}");
        assert!(e.contains("40000000"), "{e}");
    }

    #[test]
    fn an_empty_image_loads_nothing_and_is_not_an_error() {
        let space = space_with_ram(0x100);
        let loader = Loader::from_image(&[][..], 0x4000_0000, None);
        assert!(loader.is_empty());
        loader
            .load_into(&space, RequesterId::ANONYMOUS)
            .expect("nothing to do");
        assert_eq!(peek(&space, 0x4000_0000), 0);
    }

    #[test]
    fn a_reset_writes_the_image_in_again() {
        // The whole reason the load happens at reset: a cold reset zeroes RAM,
        // and a machine that came back up with no kernel would be a puzzle.
        let space = space_with_ram(0x1000);
        let loader = Loader::from_image(&b"\xde\xad"[..], 0x4000_0000, None);
        loader.binding.lock().space = Some(Arc::clone(&space));
        space
            .write(0x4000_0000, Width::U8, 0, MemAttrs::DEFAULT)
            .unwrap();
        assert_eq!(peek(&space, 0x4000_0000), 0);
        loader.reset(ResetKind::Cold);
        assert_eq!(peek(&space, 0x4000_0000), 0xde);
        assert_eq!(loader.loads(), 1);
        assert_eq!(loader.last_error(), None);
    }

    #[test]
    fn a_media_slot_is_required() {
        let e = Loader::new(&Props::new())
            .expect_err("no image")
            .to_string();
        assert!(e.contains("image") && e.contains("media"), "{e}");
    }
}
