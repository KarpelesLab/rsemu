//! Entering a Linux/x86 kernel directly, with no firmware in between.
//!
//! A PC normally reaches an operating system through a chain: firmware, a boot
//! sector, a boot loader, and only then a kernel. Every link is software we
//! would be testing instead of the machine. This device is the short circuit
//! the same way `riscv.loader` is on the other board — it writes the kernel
//! into memory and hands the processor to it in the state the kernel's own
//! entry point documents.
//!
//! # What the interface is
//!
//! The **Linux/x86 boot protocol**, which is an ABI between a boot loader and
//! a kernel image rather than an implementation: a header at offset `0x1f1` of
//! the image, a `struct boot_params` ("the zero page") built from it, and a
//! documented register and descriptor state at the 32-bit entry point. Every
//! field this file names was read out of a real `bzImage` and cross-checked
//! against `file(1)`'s independent parse of the same bytes — the image is
//! *data* being loaded, which is the black-box use `ROADMAP.md` §1 allows, and
//! no kernel source was consulted.
//!
//! The header is self-describing in the one place it matters: the byte at
//! `0x201` gives the header's own length as `0x202 + that`, so this copies
//! exactly the header the image carries rather than a length compiled in here.
//!
//! # The two entry points, and why this one
//!
//! A `bzImage` can be entered in real mode at its setup code, or at the
//! **32-bit entry** — `code32_start`, `0x100000` for every image in
//! circulation — in flat protected mode with paging off. Only the second is
//! reachable without a BIOS: the real-mode setup calls `INT 10h`, `INT 15h`
//! and `INT 13h`, and there is no firmware on this board to answer them. So
//! this device switches the processor into protected mode itself, which is
//! eleven instructions, and jumps.
//!
//! A 64-bit kernel entered this way brings itself the rest of the way: its
//! decompressor builds identity page tables, sets `EFER.LME` and `CR4.PAE`,
//! enters long mode, decompresses, and jumps to the 64-bit kernel. That is
//! exactly the path `ROADMAP.md` phase 6b's core has to survive, which is why
//! this is worth having as a device rather than as a test helper.
//!
//! # Where things go
//!
//! ```text
//!   0x00001000  the stub: real mode in, protected mode out, and its GDT
//!   0x00007000  boot_params — "the zero page", built here
//!   0x00008000  the kernel command line
//!   0x00100000  the protected-mode kernel, at its own `code32_start`
//!   high        the initial ramdisk, at the top of memory it may occupy
//!   0xfffffff0  the `reset` region: `jmp far 0000:1000`
//! ```
//!
//! Everything below 1 MiB except the kernel is scratch the kernel is free to
//! overwrite once it has copied what it wants, and the e820 map this builds
//! says so by marking it ordinary memory.
//!
//! # Why a stub and not a poke at the register file
//!
//! A device may not reach into a processor and set `CR0`. It also should not
//! have to: the state the protocol asks for is reachable by executing eleven
//! instructions, and executing them tests the core rather than bypassing it.
//! `riscv.boot` makes the same choice for the same reason, and this is the
//! same shape — a reset vector that is code.
//!
//! # KASLR is off, deliberately
//!
//! `loadflags` bit 1 is how a boot loader tells the kernel that address-space
//! randomisation is available. This never sets it. Randomisation draws entropy
//! from `RDTSC` and `RDRAND`, and a machine whose boot lands somewhere new
//! each run is not one you can diff a trace of (`CLAUDE.md`, determinism).

use alloc::boxed::Box;
use alloc::format;
use alloc::string::{String, ToString};
use alloc::sync::Arc;
use alloc::vec;
use alloc::vec::Vec;

use crate::core::device::{Device, DeviceClass, PropertySpec, RealizeCtx, ResetKind};
use crate::core::error::{BusError, Error, Result};
use crate::core::props::{Props, ValueKind};
use crate::core::space::{
    AccessConstraints, AddressSpace, MemAttrs, MemOps, MemResult, Region, RegionRef, RequesterId,
};
use crate::core::sync::{LockRank, Mutex};
use crate::machine::realize::{BindCtx, Instance};

/// The class name a machine description writes.
pub const CLASS_NAME: &str = "x86.linuxboot";

// -- where this device puts things ------------------------------------------

/// Where the mode-switch stub is written, and where the reset vector jumps.
pub const STUB_ADDR: u64 = 0x1000;

/// Where `struct boot_params` is built.
pub const ZEROPAGE_ADDR: u64 = 0x7000;

/// Where the command line is written.
pub const CMDLINE_ADDR: u64 = 0x8000;

/// How much of the zero page there is. One page, and the structure's own size.
const ZEROPAGE_LEN: usize = 0x1000;

// -- the setup header, at offset 0x1f1 of a bzImage -------------------------
//
// Offsets into the image *and* into the zero page, because the header is
// copied to the same offset in the zero page it occupies in the image. That
// identity is the protocol's, not a convenience: `struct boot_params` has the
// setup header embedded at 0x1f1.

/// `setup_sects`: how many 512-byte sectors of real-mode setup precede the
/// protected-mode kernel. Zero means four, which is the pre-1.4 default.
const HDR_SETUP_SECTS: usize = 0x1f1;
/// The boot sector's `0xaa55` signature.
const HDR_BOOT_FLAG: usize = 0x1fe;
/// The second byte of the setup code's jump, which gives the header's length:
/// the header ends at `0x202 + this`.
const HDR_JUMP_LEN: usize = 0x201;
/// `"HdrS"`.
const HDR_MAGIC: usize = 0x202;
/// The boot protocol version, major in the high byte.
const HDR_VERSION: usize = 0x206;
/// `type_of_loader`.
const HDR_TYPE_OF_LOADER: usize = 0x210;
/// `loadflags`.
const HDR_LOADFLAGS: usize = 0x211;
/// `code32_start`: where the protected-mode kernel is loaded and entered.
const HDR_CODE32_START: usize = 0x214;
/// `ramdisk_image`.
const HDR_RAMDISK_IMAGE: usize = 0x218;
/// `ramdisk_size`.
const HDR_RAMDISK_SIZE: usize = 0x21c;
/// `cmd_line_ptr`.
const HDR_CMD_LINE_PTR: usize = 0x228;
/// `initrd_addr_max`: the highest address the ramdisk may end at.
const HDR_INITRD_ADDR_MAX: usize = 0x22c;
/// `cmdline_size`: the longest command line the kernel will read, less the NUL.
const HDR_CMDLINE_SIZE: usize = 0x238;
/// `init_size`: how much memory the kernel needs at its load address to
/// decompress and relocate itself into.
const HDR_INIT_SIZE: usize = 0x260;

/// `LOADED_HIGH`: the protected-mode kernel is at 1 MiB rather than at 0x10000.
const LOADFLAGS_LOADED_HIGH: u8 = 0x01;
/// `KASLR_FLAG`: the loader permits address-space randomisation. Never set.
const LOADFLAGS_KASLR: u8 = 0x02;

/// `type_of_loader` for a loader with no assigned identifier.
const LOADER_UNDEFINED: u8 = 0xff;

// -- boot_params fields outside the setup header ----------------------------

/// How many e820 entries `E820_TABLE` holds.
const BP_E820_ENTRIES: usize = 0x1e8;
/// The e820 table itself: twenty bytes per entry.
const BP_E820_TABLE: usize = 0x2d0;
/// How many entries fit before the structure ends.
const E820_MAX: usize = 128;
/// Ordinary memory.
const E820_RAM: u32 = 1;
/// Memory the operating system may not use.
const E820_RESERVED: u32 = 2;

/// The default low-memory size: the 640 KiB below the video hole.
const DEFAULT_BASEMEM: u64 = 640 * 1024;

/// Where the protected-mode kernel goes when the header says nothing sane.
const DEFAULT_LOAD_ADDR: u64 = 0x10_0000;

/// The lowest protocol version this can enter.
///
/// 2.02 is where `code32_start` became meaningful for a loader that does not
/// run the real-mode setup. Nothing older than 2001 is in circulation.
const MIN_VERSION: u16 = 0x0202;

/// What the image said about itself.
#[derive(Debug, Clone, Copy)]
pub struct Header {
    /// The protocol version, major in the high byte.
    pub version: u16,
    /// Where the protected-mode kernel is loaded and entered.
    pub code32_start: u64,
    /// How much memory the kernel needs from `code32_start` up.
    pub init_size: u64,
    /// The highest address a ramdisk may end at.
    pub initrd_addr_max: u64,
    /// The longest command line the kernel will read.
    pub cmdline_size: u32,
    /// Where the protected-mode kernel starts inside the image.
    pub payload_at: usize,
    /// How long the header is, from `0x1f1`.
    pub header_end: usize,
}

impl Header {
    /// Parse the header of a `bzImage`.
    ///
    /// # Errors
    ///
    /// [`Error::Property`] if this is not a bzImage, or is one this cannot
    /// enter.
    pub fn parse(image: &[u8]) -> Result<Header> {
        let refuse = |why: &str| Error::Property(format!("property `kernel`: {why}"));
        let u8_at = |at: usize| image.get(at).copied().unwrap_or(0);
        let u16_at = |at: usize| match image.get(at..at + 2) {
            Some(b) => u16::from_le_bytes([b[0], b[1]]),
            None => 0,
        };
        let u32_at = |at: usize| match image.get(at..at + 4) {
            Some(b) => u32::from_le_bytes([b[0], b[1], b[2], b[3]]),
            None => 0,
        };
        if image.len() < 1024 {
            return Err(refuse("too short to be a bzImage"));
        }
        if u16_at(HDR_BOOT_FLAG) != 0xaa55 {
            return Err(refuse("no 0xaa55 boot signature at offset 0x1fe"));
        }
        if u32_at(HDR_MAGIC) != u32::from_le_bytes(*b"HdrS") {
            return Err(refuse("no `HdrS` magic at offset 0x202 — not a bzImage"));
        }
        let version = u16_at(HDR_VERSION);
        if version < MIN_VERSION {
            return Err(refuse(&format!(
                "boot protocol {}.{:02} predates the 32-bit entry point; {}.{:02} is the oldest \
                 this can enter",
                version >> 8,
                version & 0xff,
                MIN_VERSION >> 8,
                MIN_VERSION & 0xff
            )));
        }
        if u8_at(HDR_LOADFLAGS) & LOADFLAGS_LOADED_HIGH == 0 {
            // A `zImage`: the protected-mode kernel goes at 0x10000 and is
            // entered through the real-mode setup, which needs a BIOS.
            return Err(refuse(
                "this is a zImage (LOADED_HIGH clear), which is entered through its real-mode \
                 setup and therefore needs firmware; only bzImage is supported",
            ));
        }
        // Zero means four: the field was one byte of a 1991 boot sector before
        // it was a header.
        let setup_sects = match u8_at(HDR_SETUP_SECTS) {
            0 => 4u64,
            n => u64::from(n),
        };
        let payload_at = usize::try_from((setup_sects + 1) * 512)
            .map_err(|_| refuse("the setup is larger than this host can address"))?;
        if payload_at >= image.len() {
            return Err(refuse("the setup claims to be longer than the whole image"));
        }
        let header_end = HDR_MAGIC + usize::from(u8_at(HDR_JUMP_LEN));
        let code32_start = match u64::from(u32_at(HDR_CODE32_START)) {
            0 => DEFAULT_LOAD_ADDR,
            at => at,
        };
        Ok(Header {
            version,
            code32_start,
            init_size: u64::from(u32_at(HDR_INIT_SIZE)),
            initrd_addr_max: match u64::from(u32_at(HDR_INITRD_ADDR_MAX)) {
                // Before 2.03 the field did not exist and the answer was
                // 0x37ffffff, which is what a kernel of that age assumed.
                0 => 0x37ff_ffff,
                max => max,
            },
            cmdline_size: match u32_at(HDR_CMDLINE_SIZE) {
                // Before 2.06: 255 bytes and a NUL.
                0 => 255,
                n => n,
            },
            payload_at,
            header_end,
        })
    }
}

/// The sixteen bytes at the top of the address space.
///
/// `jmp far 0000:1000`, which is the only instruction a processor coming out
/// of reset can execute that changes `CS`'s cached base from `0xffff0000` to
/// zero — and therefore the only way to reach anything this device wrote into
/// low memory.
#[derive(Debug)]
struct ResetVector;

impl MemOps for ResetVector {
    fn read(&self, offset: u64, dst: &mut [u8], _attrs: MemAttrs) -> MemResult {
        // EA <offset16> <segment16>: JMP ptr16:16, the real-mode far jump
        // (Intel SDM volume 2, `JMP`). Everything after it is `0x90`, so a
        // processor that somehow ran past the jump halts on the far side of
        // the region rather than executing the operand of something.
        let image = [
            0xeau8,
            STUB_ADDR as u8,
            (STUB_ADDR >> 8) as u8,
            0x00,
            0x00,
            0x90,
            0x90,
            0x90,
            0x90,
            0x90,
            0x90,
            0x90,
            0x90,
            0x90,
            0x90,
            0x90,
        ];
        for (i, byte) in dst.iter_mut().enumerate() {
            let at = offset.wrapping_add(i as u64) as usize;
            *byte = image.get(at).copied().unwrap_or(0x90);
        }
        Ok(())
    }

    fn write(&self, _offset: u64, _src: &[u8], _attrs: MemAttrs) -> MemResult {
        Err(BusError::BadAccess)
    }

    fn constraints(&self) -> AccessConstraints {
        AccessConstraints::ANY
    }
}

/// What the loader learned when the machine bound it.
#[derive(Debug, Default)]
struct Binding {
    space: Option<Arc<AddressSpace>>,
    requester: RequesterId,
    /// What the last load failed with. `reset` cannot return an error, so the
    /// failure is kept here and surfaced by [`LinuxBoot::last_error`].
    error: Option<String>,
    /// How many times everything has been written in, for tests.
    loads: u64,
}

/// A kernel, a ramdisk, a command line, and the stub that enters them.
#[derive(Debug)]
pub struct LinuxBoot {
    kernel: Arc<[u8]>,
    initrd: Arc<[u8]>,
    cmdline: String,
    /// `None` when no kernel was bound, which loads nothing and is not an
    /// error — the same rule `riscv.loader` has, so one machine file serves a
    /// run with a kernel and a run without.
    header: Option<Header>,
    basemem: u64,
    extmem: u64,
    initrd_addr: u64,
    reset: RegionRef,
    binding: Mutex<Binding>,
}

impl LinuxBoot {
    /// Validate `props` and build the device.
    ///
    /// # Errors
    ///
    /// [`Error::Property`] if the image is not a bzImage this can enter, if
    /// the ramdisk does not fit under the kernel's `initrd_addr_max`, or if a
    /// property this class does not know was given.
    pub fn new(props: &Props) -> Result<LinuxBoot> {
        let mut r = props.reader();
        let kernel = r
            .optional_media("kernel")?
            .map(crate::core::props::Media::to_bytes)
            .unwrap_or_else(|| Arc::from(&[][..]));
        let initrd = r
            .optional_media("initrd")?
            .map(crate::core::props::Media::to_bytes)
            .unwrap_or_else(|| Arc::from(&[][..]));
        let cmdline = r.or("cmdline", String::new())?;
        let basemem = r.or_size("basemem", DEFAULT_BASEMEM)?;
        let extmem = r.or_size("extmem", 0)?;
        let initrd_addr = r.or_addr("initrd-addr", 0)?;
        r.finish()?;

        if kernel.is_empty() {
            return Ok(LinuxBoot::assembled(
                kernel, initrd, cmdline, None, basemem, extmem, 0,
            ));
        }
        let header = Header::parse(&kernel)?;
        if cmdline.len() as u32 > header.cmdline_size {
            return Err(Error::Property(format!(
                "property `cmdline`: {} bytes, and this kernel reads at most {}",
                cmdline.len(),
                header.cmdline_size
            )));
        }
        // The kernel needs `init_size` from its load address, and the ramdisk
        // has to be clear of that as well as under the kernel's own ceiling.
        // Both are checked here rather than at reset, where nothing could
        // report them and the guest would simply misbehave.
        let top = 0x10_0000u64.saturating_add(extmem);
        let kernel_end = header.code32_start.saturating_add(header.init_size);
        if extmem != 0 && kernel_end > top {
            return Err(Error::Property(format!(
                "property `extmem`: this kernel needs {} MiB from {:#x} to decompress into and \
                 the machine has memory to {:#x}",
                header.init_size >> 20,
                header.code32_start,
                top
            )));
        }
        let initrd_addr = if initrd.is_empty() {
            0
        } else if initrd_addr != 0 {
            initrd_addr
        } else if extmem == 0 {
            return Err(Error::Property(String::from(
                "property `initrd`: a ramdisk was bound and there is nowhere to put it; give \
                 `extmem` so a top of memory exists, or `initrd-addr` outright",
            )));
        } else {
            // As high as it will go, which is what every loader does: it keeps
            // the ramdisk clear of wherever the kernel decompresses to without
            // having to know how far that reaches.
            let ceiling = top.min(header.initrd_addr_max.saturating_add(1));
            let at = ceiling.saturating_sub(initrd.len() as u64) & !0xfffu64;
            if at < kernel_end {
                return Err(Error::Property(format!(
                    "property `initrd`: {} byte(s) does not fit between the kernel's {:#x} and \
                     {ceiling:#x}",
                    initrd.len(),
                    kernel_end
                )));
            }
            at
        };
        Ok(LinuxBoot::assembled(
            kernel,
            initrd,
            cmdline,
            Some(header),
            basemem,
            extmem,
            initrd_addr,
        ))
    }

    /// The parts, once they have been checked against each other.
    fn assembled(
        kernel: Arc<[u8]>,
        initrd: Arc<[u8]>,
        cmdline: String,
        header: Option<Header>,
        basemem: u64,
        extmem: u64,
        initrd_addr: u64,
    ) -> LinuxBoot {
        LinuxBoot {
            kernel,
            initrd,
            cmdline,
            header,
            basemem,
            extmem,
            initrd_addr,
            reset: Arc::new(Region::io("x86.linuxboot", 16, Arc::new(ResetVector))),
            binding: Mutex::with_rank(LockRank::DEVICE, Binding::default()),
        }
    }

    /// What the image said about itself, or `None` if no kernel was bound.
    #[must_use]
    pub fn header(&self) -> Option<Header> {
        self.header
    }

    /// Where the ramdisk was staged, or zero if there is none.
    #[must_use]
    pub fn initrd_addr(&self) -> u64 {
        self.initrd_addr
    }

    /// What the last load failed with, if it did.
    #[must_use]
    pub fn last_error(&self) -> Option<String> {
        self.binding.lock().error.clone()
    }

    /// How many times the kernel has been written into memory.
    #[must_use]
    pub fn loads(&self) -> u64 {
        self.binding.lock().loads
    }

    /// The eleven instructions that turn a reset processor into the state the
    /// 32-bit entry point documents, and the descriptor table they need.
    ///
    /// Assembled here as bytes rather than by an assembler because there is no
    /// assembler in this crate's dependency list and because every byte wants
    /// a citation anyway. Encodings are *Intel SDM* volume 2.
    ///
    /// The blob is position-dependent — it contains its own GDT's linear
    /// address — which is why [`STUB_ADDR`] is a constant rather than a
    /// property: two numbers that must agree should be one number.
    #[must_use]
    pub fn stub(&self) -> Vec<u8> {
        /// Where the 32-bit half starts, inside the blob.
        const PM32: u64 = 0x40;
        /// Where the descriptor table starts, inside the blob.
        const GDT: u64 = 0x80;
        /// Where the pseudo-descriptor `LGDT` reads starts, inside the blob.
        const GDTR: u64 = 0xa0;
        /// The flat code segment's selector — `__BOOT_CS`, which is what a
        /// kernel that does not reload the segment registers expects.
        const SEL_CODE: u16 = 0x10;
        /// The flat data segment's selector — `__BOOT_DS`.
        const SEL_DATA: u16 = 0x18;

        let entry = self.header.map_or(DEFAULT_LOAD_ADDR, |h| h.code32_start) as u32;
        let mut out: Vec<u8> = Vec::with_capacity(0xb0);

        // -- 16-bit, at STUB_ADDR, reached by the reset vector's far jump ----
        out.extend_from_slice(&[0xfa]); // cli
        out.extend_from_slice(&[0xfc]); // cld — the protocol asks for DF clear
        out.extend_from_slice(&[0x31, 0xc0]); // xor ax, ax
        out.extend_from_slice(&[0x8e, 0xd8]); // mov ds, ax
        out.extend_from_slice(&[0x8e, 0xc0]); // mov es, ax
        out.extend_from_slice(&[0x8e, 0xd0]); // mov ss, ax
        // lgdt [GDTR]. The 16-bit form loads a 24-bit base, and the base here
        // is 0x000010a0, so the two forms agree — see the pseudo-descriptor
        // below, whose fourth base byte is zero for exactly this reason.
        let gdtr = (STUB_ADDR + GDTR) as u16;
        out.extend_from_slice(&[0x0f, 0x01, 0x16, gdtr as u8, (gdtr >> 8) as u8]);
        out.extend_from_slice(&[0x0f, 0x20, 0xc0]); // mov eax, cr0
        out.extend_from_slice(&[0x0c, 0x01]); // or al, 1  — CR0.PE
        out.extend_from_slice(&[0x0f, 0x22, 0xc0]); // mov cr0, eax
        // jmp far SEL_CODE:PM32, with the operand-size prefix that makes the
        // offset 32 bits. The far jump is what loads CS with a descriptor;
        // until it retires the processor is in protected mode with a real-mode
        // CS, which is why nothing may come between it and `mov cr0`.
        let pm32 = (STUB_ADDR + PM32) as u32;
        out.extend_from_slice(&[0x66, 0xea]);
        out.extend_from_slice(&pm32.to_le_bytes());
        out.extend_from_slice(&SEL_CODE.to_le_bytes());
        assert!(out.len() as u64 <= PM32, "the 16-bit half overran its half");
        out.resize(PM32 as usize, 0x90);

        // -- 32-bit, flat, paging off ---------------------------------------
        out.extend_from_slice(&[0xb8]); // mov eax, SEL_DATA
        out.extend_from_slice(&u32::from(SEL_DATA).to_le_bytes());
        out.extend_from_slice(&[0x8e, 0xd8]); // mov ds, ax
        out.extend_from_slice(&[0x8e, 0xc0]); // mov es, ax
        out.extend_from_slice(&[0x8e, 0xe0]); // mov fs, ax
        out.extend_from_slice(&[0x8e, 0xe8]); // mov gs, ax
        out.extend_from_slice(&[0x8e, 0xd0]); // mov ss, ax
        // The protocol's register state: ESI points at the zero page, and EBP,
        // EDI and EBX are zero.
        out.extend_from_slice(&[0xbe]); // mov esi, ZEROPAGE_ADDR
        out.extend_from_slice(&(ZEROPAGE_ADDR as u32).to_le_bytes());
        out.extend_from_slice(&[0x31, 0xed]); // xor ebp, ebp
        out.extend_from_slice(&[0x31, 0xff]); // xor edi, edi
        out.extend_from_slice(&[0x31, 0xdb]); // xor ebx, ebx
        // jmp entry, as a displacement from the end of this instruction. A
        // near jump rather than a far one because CS is already the flat code
        // segment and reloading it would only give the kernel a second chance
        // to disagree with us about the selector.
        let next = STUB_ADDR as u32 + out.len() as u32 + 5;
        out.extend_from_slice(&[0xe9]);
        out.extend_from_slice(&entry.wrapping_sub(next).to_le_bytes());
        assert!(out.len() as u64 <= GDT, "the 32-bit half overran its half");
        out.resize(GDT as usize, 0x90);

        // -- the descriptor table -------------------------------------------
        //
        // Two flat descriptors at the selectors above, with a null and one
        // unused entry ahead of them so the selectors land where a kernel
        // expects. Each is limit 0xfffff with G set, so the limit is 4 GiB.
        // Type 0x9a is present, ring 0, code, execute/read; 0x92 is the data
        // half. D/B and G are the `0xcf` byte (SDM volume 3 §3.4.5).
        out.extend_from_slice(&[0; 8]); // 0x00: the null descriptor
        out.extend_from_slice(&[0; 8]); // 0x08: unused
        out.extend_from_slice(&[0xff, 0xff, 0, 0, 0, 0x9a, 0xcf, 0]); // 0x10: code
        out.extend_from_slice(&[0xff, 0xff, 0, 0, 0, 0x92, 0xcf, 0]); // 0x18: data
        assert_eq!(out.len() as u64, GDTR, "the table is four descriptors");

        // -- the pseudo-descriptor LGDT reads -------------------------------
        let base = (STUB_ADDR + GDT) as u32;
        out.extend_from_slice(&(0x20u16 - 1).to_le_bytes()); // limit: four entries
        out.extend_from_slice(&base.to_le_bytes());
        out
    }

    /// The zero page, built from the image's own header.
    ///
    /// Infallible: everything that could be refused — a ramdisk that does not
    /// fit, a command line the kernel will not read — was refused at
    /// construction, which is where a machine description's mistake belongs.
    #[must_use]
    pub fn zero_page(&self) -> Vec<u8> {
        let mut page = vec![0u8; ZEROPAGE_LEN];
        let Some(header) = self.header else {
            return page;
        };
        // The setup header, copied from the image at the offset it already
        // occupies. Everything outside it stays zero, which is what tells the
        // kernel there is no EDD table, no APM, no EFI and — through
        // `screen_info` — no VGA console, so the serial line is the console.
        let end = header.header_end.min(self.kernel.len());
        if let Some(src) = self.kernel.get(HDR_SETUP_SECTS..end) {
            page[HDR_SETUP_SECTS..HDR_SETUP_SECTS + src.len()].copy_from_slice(src);
        }
        page[HDR_TYPE_OF_LOADER] = LOADER_UNDEFINED;
        // Keep LOADED_HIGH, and make sure randomisation stays off whatever the
        // image shipped in the field.
        page[HDR_LOADFLAGS] = (page[HDR_LOADFLAGS] | LOADFLAGS_LOADED_HIGH) & !LOADFLAGS_KASLR;
        let put32 = |page: &mut [u8], at: usize, value: u32| {
            page[at..at + 4].copy_from_slice(&value.to_le_bytes());
        };
        put32(&mut page, HDR_CMD_LINE_PTR, CMDLINE_ADDR as u32);
        put32(&mut page, HDR_RAMDISK_IMAGE, self.initrd_addr as u32);
        put32(&mut page, HDR_RAMDISK_SIZE, self.initrd.len() as u32);

        // The memory map. Three entries and no more: this board has low
        // memory, a hole where the video adapter and firmware would be, and
        // extended memory. A kernel that finds no e820 map at all falls back
        // to a BIOS call that cannot be made here, so this is not optional.
        let mut entries: Vec<(u64, u64, u32)> = Vec::new();
        entries.push((0, self.basemem, E820_RAM));
        entries.push((
            self.basemem,
            0x10_0000u64.saturating_sub(self.basemem),
            E820_RESERVED,
        ));
        if self.extmem != 0 {
            entries.push((0x10_0000, self.extmem, E820_RAM));
        }
        entries.truncate(E820_MAX);
        page[BP_E820_ENTRIES] = entries.len() as u8;
        for (i, (addr, len, kind)) in entries.iter().enumerate() {
            let at = BP_E820_TABLE + i * 20;
            page[at..at + 8].copy_from_slice(&addr.to_le_bytes());
            page[at + 8..at + 16].copy_from_slice(&len.to_le_bytes());
            page[at + 16..at + 20].copy_from_slice(&kind.to_le_bytes());
        }
        page
    }

    /// Write everything into `space`.
    ///
    /// # Errors
    ///
    /// Whatever the address space refuses, which for a kernel almost always
    /// means the machine has less memory than the image needs.
    pub fn load_into(&self, space: &AddressSpace, requester: RequesterId) -> Result<()> {
        let Some(header) = self.header else {
            return Ok(());
        };
        let attrs = MemAttrs::DEFAULT
            .with_requester(requester)
            .with_privileged(true);
        let put = |at: u64, bytes: &[u8], what: &str| -> Result<()> {
            // In page-sized pieces so a failure names the address it happened
            // at rather than the start of a 12 MiB kernel.
            let mut at = at;
            for piece in bytes.chunks(4096) {
                space
                    .write_bytes(at, piece, attrs)
                    .map_err(|e| Error::Config {
                        at: CLASS_NAME.to_string(),
                        message: format!(
                            "cannot write {what} at {at:#x} in space `{}`: {e}",
                            space.name()
                        ),
                    })?;
                at += piece.len() as u64;
            }
            Ok(())
        };
        put(STUB_ADDR, &self.stub(), "the mode-switch stub")?;
        put(ZEROPAGE_ADDR, &self.zero_page(), "the zero page")?;
        let mut cmdline = self.cmdline.clone().into_bytes();
        cmdline.push(0);
        put(CMDLINE_ADDR, &cmdline, "the command line")?;
        put(
            header.code32_start,
            &self.kernel[header.payload_at..],
            "the kernel",
        )?;
        if !self.initrd.is_empty() {
            put(self.initrd_addr, &self.initrd, "the ramdisk")?;
        }
        Ok(())
    }
}

/// The `x86.linuxboot` device class.
pub static CLASS: DeviceClass = DeviceClass {
    name: CLASS_NAME,
    version: 1,
    summary: "loads a Linux/x86 bzImage and enters it at its 32-bit entry point, with no firmware",
    properties: &[
        PropertySpec {
            name: "kernel",
            kind: ValueKind::Media,
            required: false,
            summary: "the bzImage, as the name of a media slot; an unbound slot loads nothing",
        },
        PropertySpec {
            name: "initrd",
            kind: ValueKind::Media,
            required: false,
            summary: "the initial ramdisk, as the name of a media slot",
        },
        PropertySpec {
            name: "cmdline",
            kind: ValueKind::Str,
            required: false,
            summary: "the kernel command line",
        },
        PropertySpec {
            name: "basemem",
            kind: ValueKind::Size,
            required: false,
            summary: "how much memory the e820 map calls usable below 1 MiB (default 640K)",
        },
        PropertySpec {
            name: "extmem",
            kind: ValueKind::Size,
            required: false,
            summary: "how much memory the e820 map calls usable from 1 MiB up",
        },
        PropertySpec {
            name: "initrd-addr",
            kind: ValueKind::Addr,
            required: false,
            summary: "where to stage the ramdisk; the default is as high as the kernel allows",
        },
    ],
    construct: |props| Ok(Box::new(LinuxBoot::new(props)?)),
};

impl Device for LinuxBoot {
    fn class(&self) -> &'static DeviceClass {
        &CLASS
    }

    fn realize(&self, _ctx: &mut RealizeCtx<'_>) -> Result<()> {
        // Nothing outward: everything goes in at reset, once the memory it
        // lands in has been cleared. `riscv.loader` documents the rule.
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

    fn region(&self, name: &str) -> Option<RegionRef> {
        matches!(name, "" | "reset").then(|| Arc::clone(&self.reset))
    }

    // No `save`/`load`: everything here is a pure function of the media the
    // caller bound, and a snapshot that carried it would store a kernel in
    // every save state (`ROADMAP.md` §4.5).
}

impl Instance for LinuxBoot {
    fn bind(&self, ctx: &BindCtx<'_>) -> Result<()> {
        let space = ctx.space().ok_or_else(|| Error::Config {
            at: ctx.path().to_string(),
            message: String::from("a kernel loader needs an address space (`space = mem`)"),
        })?;
        // Fail here rather than at reset, where nothing could report it: a
        // machine whose kernel does not fit should not build at all.
        self.load_into(space, ctx.requester())?;
        let mut binding = self.binding.lock();
        binding.space = Some(Arc::clone(space));
        binding.requester = ctx.requester();
        Ok(())
    }
}

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
    bindings.bind(CLASS_NAME, |props| Ok(Arc::new(LinuxBoot::new(props)?)))
}

/// What the validator should know about `x86.linuxboot`.
#[must_use]
pub fn schema() -> crate::machine::validate::ClassSchema {
    use crate::machine::validate::{ClassSchema, PropSchema};
    let mut schema = ClassSchema::new(CLASS_NAME);
    for spec in CLASS.properties {
        schema = schema.prop(PropSchema::new(spec.name, spec.kind));
    }
    schema.region("reset")
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::props::{Media, Value};
    use crate::core::space::RamStore;
    use crate::core::value::Width;

    /// The smallest thing that parses as a bzImage: a setup header claiming
    /// one sector of setup, and one sector of "kernel" after it.
    fn fake_bzimage(payload: &[u8]) -> Vec<u8> {
        let mut image = vec![0u8; 1024];
        image[HDR_SETUP_SECTS] = 1; // one sector of setup, so the payload is at 1024
        image[HDR_BOOT_FLAG] = 0x55;
        image[HDR_BOOT_FLAG + 1] = 0xaa;
        image[0x200] = 0xeb;
        image[HDR_JUMP_LEN] = 0x6a; // header ends at 0x26c, as a real image's does
        image[HDR_MAGIC..HDR_MAGIC + 4].copy_from_slice(b"HdrS");
        image[HDR_VERSION..HDR_VERSION + 2].copy_from_slice(&0x020fu16.to_le_bytes());
        image[HDR_LOADFLAGS] = LOADFLAGS_LOADED_HIGH | LOADFLAGS_KASLR;
        image[HDR_CODE32_START..HDR_CODE32_START + 4].copy_from_slice(&0x10_0000u32.to_le_bytes());
        image[HDR_INIT_SIZE..HDR_INIT_SIZE + 4].copy_from_slice(&0x10_0000u32.to_le_bytes());
        image[HDR_INITRD_ADDR_MAX..HDR_INITRD_ADDR_MAX + 4]
            .copy_from_slice(&0x7fff_ffffu32.to_le_bytes());
        image[HDR_CMDLINE_SIZE..HDR_CMDLINE_SIZE + 4].copy_from_slice(&2047u32.to_le_bytes());
        image.extend_from_slice(payload);
        image
    }

    fn boot(initrd: &[u8], cmdline: &str) -> LinuxBoot {
        let mut props = Props::new();
        props.insert(
            "kernel",
            Media::new("kernel", fake_bzimage(b"kernel bytes")),
        );
        if !initrd.is_empty() {
            props.insert("initrd", Media::new("initrd", initrd.to_vec()));
        }
        props.insert("cmdline", cmdline);
        props.insert("extmem", Value::Size(64 * 1024 * 1024));
        LinuxBoot::new(&props).expect("a well-formed image")
    }

    fn space_with_ram() -> Arc<AddressSpace> {
        let space = AddressSpace::new("mem", 32);
        let ram = Arc::new(RamStore::new(64 * 1024 * 1024 + 0x10_0000));
        space
            .topology()
            .map(Region::ram("ram", ram), 0)
            .expect("a fresh space");
        Arc::new(space)
    }

    fn peek(space: &AddressSpace, at: u64) -> u8 {
        space.read(at, Width::U8, MemAttrs::DEBUG).expect("mapped") as u8
    }

    #[test]
    fn a_header_that_is_not_a_bzimage_is_refused_at_construction() {
        let mut props = Props::new();
        props.insert("kernel", Media::new("kernel", vec![0u8; 4096]));
        let err = LinuxBoot::new(&props).expect_err("no boot signature");
        assert!(format!("{err}").contains("0xaa55"), "{err}");
    }

    #[test]
    fn a_zimage_is_refused_by_name_because_it_needs_a_bios() {
        let mut image = fake_bzimage(b"");
        image[HDR_LOADFLAGS] = 0;
        let mut props = Props::new();
        props.insert("kernel", Media::new("kernel", image));
        let err = LinuxBoot::new(&props).expect_err("LOADED_HIGH is clear");
        assert!(format!("{err}").contains("zImage"), "{err}");
    }

    #[test]
    fn an_unbound_kernel_loads_nothing_and_is_not_an_error() {
        let props = Props::new();
        let boot = LinuxBoot::new(&props).expect("an empty slot is a machine with no kernel");
        assert!(boot.header().is_none());
        let space = space_with_ram();
        boot.load_into(&space, RequesterId::ANONYMOUS)
            .expect("nothing to do");
        assert_eq!(peek(&space, STUB_ADDR), 0, "nothing was written");
    }

    #[test]
    fn the_zero_page_carries_the_images_own_header_and_our_memory_map() {
        let boot = boot(b"", "console=ttyS0");
        let page = boot.zero_page();
        assert_eq!(&page[HDR_MAGIC..HDR_MAGIC + 4], b"HdrS", "the header moved");
        assert_eq!(page[HDR_TYPE_OF_LOADER], LOADER_UNDEFINED);
        assert_eq!(
            page[HDR_LOADFLAGS] & LOADFLAGS_KASLR,
            0,
            "randomisation is cleared even when the image shipped with it set"
        );
        assert_eq!(
            u32::from_le_bytes(
                page[HDR_CMD_LINE_PTR..HDR_CMD_LINE_PTR + 4]
                    .try_into()
                    .unwrap()
            ),
            CMDLINE_ADDR as u32
        );
        // Three e820 entries: low memory, the hole, and extended memory.
        assert_eq!(page[BP_E820_ENTRIES], 3);
        let entry = |i: usize| {
            let at = BP_E820_TABLE + i * 20;
            (
                u64::from_le_bytes(page[at..at + 8].try_into().unwrap()),
                u64::from_le_bytes(page[at + 8..at + 16].try_into().unwrap()),
                u32::from_le_bytes(page[at + 16..at + 20].try_into().unwrap()),
            )
        };
        assert_eq!(entry(0), (0, DEFAULT_BASEMEM, E820_RAM));
        assert_eq!(entry(1).2, E820_RESERVED);
        assert_eq!(entry(2), (0x10_0000, 64 * 1024 * 1024, E820_RAM));
    }

    #[test]
    fn a_ramdisk_lands_as_high_as_the_kernel_allows_and_the_zero_page_says_where() {
        let boot = boot(&[0x5au8; 4096], "");
        let top = 0x10_0000 + 64 * 1024 * 1024;
        assert_eq!(
            boot.initrd_addr(),
            top - 4096,
            "as high as it goes, page aligned"
        );
        let page = boot.zero_page();
        assert_eq!(
            u32::from_le_bytes(
                page[HDR_RAMDISK_IMAGE..HDR_RAMDISK_IMAGE + 4]
                    .try_into()
                    .unwrap()
            ),
            boot.initrd_addr() as u32
        );
        assert_eq!(
            u32::from_le_bytes(
                page[HDR_RAMDISK_SIZE..HDR_RAMDISK_SIZE + 4]
                    .try_into()
                    .unwrap()
            ),
            4096
        );
    }

    #[test]
    fn the_reset_vector_is_a_far_jump_to_the_stub() {
        let boot = boot(b"", "");
        let region = boot.region("reset").expect("the class publishes one");
        let space = AddressSpace::new("mem", 32);
        space
            .topology()
            .map(region, 0xffff_fff0)
            .expect("a fresh space");
        assert_eq!(peek(&space, 0xffff_fff0), 0xea, "JMP ptr16:16");
        assert_eq!(peek(&space, 0xffff_fff1), STUB_ADDR as u8);
        assert_eq!(peek(&space, 0xffff_fff3), 0, "segment zero");
    }

    #[test]
    fn everything_lands_where_the_stub_and_the_zero_page_say_it_does() {
        let boot = boot(&[0x5au8; 4096], "quiet");
        let space = space_with_ram();
        boot.load_into(&space, RequesterId::ANONYMOUS)
            .expect("fits");
        assert_eq!(peek(&space, STUB_ADDR), 0xfa, "cli");
        assert_eq!(peek(&space, CMDLINE_ADDR), b'q');
        assert_eq!(peek(&space, CMDLINE_ADDR + 5), 0, "NUL terminated");
        assert_eq!(peek(&space, 0x10_0000), b'k', "the payload, not the setup");
        assert_eq!(peek(&space, boot.initrd_addr()), 0x5a);
        // And the zero page really is at the address the stub loads into ESI.
        assert_eq!(peek(&space, ZEROPAGE_ADDR + HDR_MAGIC as u64), b'H');
    }
}
