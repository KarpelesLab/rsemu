//! The boot ROM: five instructions and the device tree they hand over.
//!
//! A RISC-V hart comes out of reset in machine mode with nothing in its
//! registers, and the software it is about to run — OpenSBI, a bare-metal
//! program, a kernel entered directly — expects two things by convention:
//! `a0` holds the hart id, and `a1` points at a flattened device tree. Somebody
//! has to put them there. On real hardware it is a mask ROM; here it is this.
//!
//! ```text
//!   +0x00  auipc t0, 0        ; t0 = this ROM's own base, whatever it is
//!   +0x04  addi  a1, t0, 32   ; a1 = the device tree, just below
//!   +0x08  csrr  a0, mhartid  ; a0 = which hart am I
//!   +0x0c  ld    t0, 24(t0)   ; t0 = the entry address, stored at +0x18
//!   +0x10  jr    t0
//!   +0x18  .dword entry
//!   +0x20  the generated device tree
//! ```
//!
//! Position-independent on purpose: `auipc` means the stub works wherever a
//! machine file maps it, so the ROM's own address appears nowhere in the ROM.
//!
//! # The device tree is generated, not stored
//!
//! [`dt::generate`](super::dt::generate) walks the address space this device
//! was bound to and produces the tree from what is actually mapped there. The
//! image is rebuilt on every reset, which is the first moment the whole machine
//! graph exists — see [`dt`](super::dt) for why that is the only correct
//! moment, and for what the tree can and cannot derive.
//!
//! # Sources
//!
//! *The RISC-V Instruction Set Manual, Volume I: Unprivileged ISA* (CC-BY-4.0)
//! for the instruction formats encoded in [`asm`], and Volume II for `mhartid`
//! at CSR `0xF14`.

use alloc::boxed::Box;
use alloc::format;
use alloc::string::{String, ToString};
use alloc::sync::Arc;
use alloc::vec::Vec;

use crate::core::device::{Device, DeviceClass, PropertySpec, RealizeCtx, ResetKind};
use crate::core::error::{BusError, Error, Result};
use crate::core::props::{Media, Props, ValueKind};
use crate::core::space::{AccessConstraints, AddressSpace, MemAttrs, MemOps, MemResult};
use crate::core::space::{Region, RegionRef};
use crate::core::sync::{LockRank, Mutex};
use crate::machine::realize::{BindCtx, Instance};

use super::dt::{CpuSpec, TreeConfig};

/// The class name a machine description writes.
pub const CLASS_NAME: &str = "riscv.boot";

/// Where the generated tree starts inside the ROM.
///
/// Eight-byte aligned, which the device tree specification asks for and which
/// firmware that memcpy's the blob with 64-bit loads depends on.
pub const DTB_OFFSET: u64 = 0x20;

/// Where the entry address literal sits inside the ROM.
const ENTRY_OFFSET: u64 = 0x18;

/// How much address space the ROM answers by default.
pub const DEFAULT_SIZE: u64 = 0xf000;

/// Where the stub jumps when a machine file does not say.
pub const DEFAULT_ENTRY: u64 = 0x8000_0000;

/// Encoders for the handful of instructions the stub needs.
///
/// Written out rather than assembled from a string, and built from the formats
/// in Volume I §2.2 rather than from a table of magic numbers, so the stub can
/// be read against the manual.
pub mod asm {
    /// `auipc rd, imm` — U-type: the 20-bit immediate becomes the top of a
    /// PC-relative address (Volume I §2.4).
    #[must_use]
    pub const fn auipc(rd: u32, imm: u32) -> u32 {
        (imm << 12) | (rd << 7) | 0b0010111
    }

    /// `addi rd, rs1, imm` — I-type (Volume I §2.4).
    #[must_use]
    pub const fn addi(rd: u32, rs1: u32, imm: i32) -> u32 {
        i_type(0b0010011, 0b000, rd, rs1, imm)
    }

    /// `ld rd, imm(rs1)` — I-type with the 64-bit load width (Volume I §5.2).
    #[must_use]
    pub const fn ld(rd: u32, rs1: u32, imm: i32) -> u32 {
        i_type(0b0000011, 0b011, rd, rs1, imm)
    }

    /// `lw rd, imm(rs1)` — the RV32 stub's version of [`ld`].
    #[must_use]
    pub const fn lw(rd: u32, rs1: u32, imm: i32) -> u32 {
        i_type(0b0000011, 0b010, rd, rs1, imm)
    }

    /// `jalr rd, imm(rs1)` (Volume I §2.5). `jr rs1` is `jalr x0, 0(rs1)`.
    #[must_use]
    pub const fn jalr(rd: u32, rs1: u32, imm: i32) -> u32 {
        i_type(0b1100111, 0b000, rd, rs1, imm)
    }

    /// `csrr rd, csr` — `csrrs rd, csr, x0`, which reads without writing
    /// because `rs1` is `x0` (Volume II, Zicsr).
    #[must_use]
    pub const fn csrr(rd: u32, csr: u32) -> u32 {
        i_type(0b1110011, 0b010, rd, 0, csr as i32)
    }

    /// The I-type layout: `imm[11:0] | rs1 | funct3 | rd | opcode`.
    const fn i_type(opcode: u32, funct3: u32, rd: u32, rs1: u32, imm: i32) -> u32 {
        (((imm as u32) & 0xfff) << 20) | (rs1 << 15) | (funct3 << 12) | (rd << 7) | opcode
    }

    /// `x5`, the stub's scratch register.
    pub const T0: u32 = 5;
    /// `x10`, which carries the hart id.
    pub const A0: u32 = 10;
    /// `x11`, which carries the device tree pointer.
    pub const A1: u32 = 11;
    /// `x0`.
    pub const ZERO: u32 = 0;
    /// The `mhartid` CSR (Volume II).
    pub const CSR_MHARTID: u32 = 0xf14;
}

/// The image, and what it is built from.
#[derive(Debug)]
struct Contents {
    /// The bytes the ROM answers with. Empty until the first reset.
    image: Vec<u8>,
    /// What the last generation failed with, if it did.
    error: Option<String>,
    /// The size of the generated tree, for tests and diagnostics.
    dtb_len: usize,
    space: Option<Arc<AddressSpace>>,
}

/// The ROM, as something an address space can dispatch to.
#[derive(Debug)]
struct Rom {
    contents: Mutex<Contents>,
    len: u64,
}

impl MemOps for Rom {
    fn read(&self, offset: u64, dst: &mut [u8], _attrs: MemAttrs) -> MemResult {
        let contents = self.contents.lock();
        for (i, byte) in dst.iter_mut().enumerate() {
            let at = offset + i as u64;
            *byte = usize::try_from(at)
                .ok()
                .and_then(|at| contents.image.get(at))
                .copied()
                // Past the generated image but inside the window: an
                // unprogrammed cell, which reads as zero here rather than as
                // ones because a firmware that runs off the end should hit
                // `illegal instruction` rather than a valid `c.and`.
                .unwrap_or(0);
        }
        Ok(())
    }

    fn write(&self, _offset: u64, _src: &[u8], _attrs: MemAttrs) -> MemResult {
        // A mask ROM. A store here is not a bus error on real hardware — there
        // is no line to report one on — but this board's space does fault on
        // unassigned addresses, so a guest that writes to its own boot ROM has
        // a bug worth telling it about.
        Err(BusError::BadAccess)
    }

    fn constraints(&self) -> AccessConstraints {
        // Instruction fetch, a firmware `memcpy` of the tree, and a debugger
        // dump all read this, at every width and in bursts.
        AccessConstraints::ANY
    }
}

/// The reset vector and the generated device tree.
#[derive(Debug)]
pub struct BootRom {
    rom: Arc<Rom>,
    region: RegionRef,
    entry: u64,
    rv32: bool,
    config: TreeConfig,
    /// What this machine's devices published about themselves.
    ///
    /// A field rather than a `static`: the tree is regenerated at reset, when
    /// `&self` is all there is, and the table has to be *this* board's — see
    /// [`dt`](super::dt). Acquired in `new` from the build's host objects,
    /// which is where every other host object is acquired
    /// ([`core::hosts`](crate::core::hosts)); a `BootRom` built outside a build
    /// gets an empty one of its own and describes a machine with no
    /// peripherals, which is the honest answer for a device with no machine.
    dt: Arc<super::dt::Publications>,
}

impl BootRom {
    /// Validate `props` and build the device.
    ///
    /// # Errors
    ///
    /// [`Error::Property`] if a property is of the wrong kind or out of range,
    /// or if one this class does not know was given.
    pub fn new(props: &Props) -> Result<BootRom> {
        let dt = super::dt::table_for(props)?;
        let mut r = props.reader();
        let size = r.or_size("size", DEFAULT_SIZE)?;
        let entry = r.or_addr("entry", DEFAULT_ENTRY)?;
        let harts = r.or_range("harts", 1u64, 1..=4096)?;
        let boot_hart = r.or_range("boot-hart", 0u64, 0..=harts - 1)?;
        let isa = r.or("isa", String::from("rv64imafdc"))?;
        let mmu = r.or("mmu", String::from("sv39"))?;
        let bootargs = r.or("bootargs", String::new())?;
        // The ramdisk is *described* here and *placed* by a `riscv.loader`, so
        // the same media slot is named twice and its length is read out of the
        // bytes rather than written down a second time. Only the address is
        // repeated, which is the same duplication a `map` statement already
        // has, and a wrong one is visible in the generated tree.
        let initrd_len = r.optional_media("initrd")?.map_or(0, Media::len);
        let initrd_addr = r.or_addr("initrd-addr", 0)?;
        let model = r.or("model", String::from("rsemu riscv-virt"))?;
        let timebase = r.or_range(
            "timebase",
            u64::from(super::clint::DEFAULT_TIMEBASE_HZ),
            1..=u64::from(u32::MAX),
        )?;
        r.finish()?;

        // Computed here rather than at the tree, so a ramdisk that cannot be
        // described is a construction error rather than a wrong `/chosen`.
        let initrd = if initrd_len == 0 {
            None
        } else if initrd_addr == 0 {
            return Err(Error::Property(String::from(
                "property `initrd`: a ramdisk was bound but `initrd-addr` says where it is not; \
                 give the address the `riscv.loader` staging it writes to",
            )));
        } else {
            // Deliberately checked, not wrapping: this is a host-side
            // description of where an image was put, and an end that wrapped
            // past the top of the address space would describe nothing.
            let end = initrd_addr.checked_add(initrd_len).ok_or_else(|| {
                Error::Property(format!(
                    "property `initrd-addr`: {initrd_len} byte(s) at {initrd_addr:#x} runs off \
                     the end of a 64-bit address space"
                ))
            })?;
            Some((initrd_addr, end))
        };
        if size < DTB_OFFSET + 0x100 {
            return Err(Error::Property(format!(
                "property `size`: a boot ROM of {size} byte(s) has no room for a device tree; \
                 it needs at least {}",
                DTB_OFFSET + 0x100
            )));
        }
        let rv32 = isa.starts_with("rv32");
        let rom = Arc::new(Rom {
            contents: Mutex::with_rank(
                LockRank::DEVICE,
                Contents {
                    image: Vec::new(),
                    error: None,
                    dtb_len: 0,
                    space: None,
                },
            ),
            len: size,
        });
        let region: RegionRef = Arc::new(Region::io(
            "riscv.boot",
            size,
            Arc::clone(&rom) as Arc<dyn MemOps>,
        ));
        Ok(BootRom {
            rom,
            region,
            entry,
            rv32,
            dt,
            config: TreeConfig {
                model,
                bootargs,
                initrd,
                cpus: CpuSpec {
                    harts: harts as u32,
                    isa,
                    mmu: if mmu == "none" { String::new() } else { mmu },
                    boot_hart: boot_hart as u32,
                },
                default_timebase_hz: timebase as u32,
            },
        })
    }

    /// The address the stub jumps to.
    #[must_use]
    pub fn entry(&self) -> u64 {
        self.entry
    }

    /// What the last device tree generation failed with, if it did.
    #[must_use]
    pub fn last_error(&self) -> Option<String> {
        self.rom.contents.lock().error.clone()
    }

    /// The generated tree, or an empty vector if none has been built.
    #[must_use]
    pub fn device_tree(&self) -> Vec<u8> {
        let contents = self.rom.contents.lock();
        let at = DTB_OFFSET as usize;
        contents
            .image
            .get(at..at + contents.dtb_len)
            .map(<[u8]>::to_vec)
            .unwrap_or_default()
    }

    /// The five-instruction stub, followed by the entry address literal.
    #[must_use]
    pub fn stub(&self) -> Vec<u8> {
        use asm::{A0, A1, CSR_MHARTID, T0, ZERO};
        let load = if self.rv32 {
            asm::lw(T0, T0, ENTRY_OFFSET as i32)
        } else {
            asm::ld(T0, T0, ENTRY_OFFSET as i32)
        };
        let words = [
            asm::auipc(T0, 0),
            asm::addi(A1, T0, DTB_OFFSET as i32),
            asm::csrr(A0, CSR_MHARTID),
            load,
            asm::jalr(ZERO, T0, 0),
            0,
        ];
        let mut out = Vec::with_capacity(ENTRY_OFFSET as usize + 8);
        for word in words {
            out.extend_from_slice(&word.to_le_bytes());
        }
        debug_assert_eq!(out.len() as u64, ENTRY_OFFSET);
        out.extend_from_slice(&self.entry.to_le_bytes());
        out
    }

    /// Rebuild the image from the machine as it now stands.
    ///
    /// # Errors
    ///
    /// Whatever [`dt::generate`](super::dt::generate) refuses, or a tree too
    /// large for the ROM window.
    pub fn regenerate(&self) -> Result<()> {
        let space = self.rom.contents.lock().space.clone();
        let Some(space) = space else {
            return Err(Error::Config {
                at: CLASS_NAME.to_string(),
                message: String::from(
                    "a boot ROM needs an address space to describe (`space = mem`)",
                ),
            });
        };
        let dtb = super::dt::generate(&self.dt, &space, &self.config)?;
        let mut image = self.stub();
        image.resize(DTB_OFFSET as usize, 0);
        image.extend_from_slice(&dtb);
        if image.len() as u64 > self.rom.len {
            return Err(Error::Config {
                at: CLASS_NAME.to_string(),
                message: format!(
                    "the generated device tree needs {} byte(s) and the boot ROM is {}; \
                     give the object a larger `size`",
                    image.len(),
                    self.rom.len
                ),
            });
        }
        let mut contents = self.rom.contents.lock();
        contents.dtb_len = dtb.len();
        contents.image = image;
        contents.error = None;
        Ok(())
    }
}

/// The `riscv.boot` device class.
pub static CLASS: DeviceClass = DeviceClass {
    name: CLASS_NAME,
    version: 1,
    summary: "the reset vector, and the device tree generated from the realized machine",
    properties: &[
        PropertySpec {
            name: "size",
            kind: ValueKind::Size,
            required: false,
            summary: "how much address space the ROM answers (default 60K)",
        },
        PropertySpec {
            name: "entry",
            kind: ValueKind::Addr,
            required: false,
            summary: "the address the stub jumps to (default 0x80000000)",
        },
        PropertySpec {
            name: "harts",
            kind: ValueKind::Uint,
            required: false,
            summary: "how many harts the tree describes (default 1)",
        },
        PropertySpec {
            name: "boot-hart",
            kind: ValueKind::Uint,
            required: false,
            summary: "the hart the firmware is entered on (default 0)",
        },
        PropertySpec {
            name: "isa",
            kind: ValueKind::Str,
            required: false,
            summary: "the `riscv,isa` string the tree reports (default rv64imafdc)",
        },
        PropertySpec {
            name: "mmu",
            kind: ValueKind::Str,
            required: false,
            summary: "the `mmu-type` suffix, or `none` (default sv39)",
        },
        PropertySpec {
            name: "bootargs",
            kind: ValueKind::Str,
            required: false,
            summary: "the kernel command line, as `/chosen/bootargs`",
        },
        PropertySpec {
            name: "initrd",
            kind: ValueKind::Media,
            required: false,
            summary: "the ramdisk to describe, as a media slot — read for its length only, \
                      because a `riscv.loader` is what puts it in memory",
        },
        PropertySpec {
            name: "initrd-addr",
            kind: ValueKind::Addr,
            required: false,
            summary: "where that ramdisk was staged, as `/chosen/linux,initrd-start`",
        },
        PropertySpec {
            name: "model",
            kind: ValueKind::Str,
            required: false,
            summary: "the tree's `model` property",
        },
        PropertySpec {
            name: "timebase",
            kind: ValueKind::Uint,
            required: false,
            summary: "the timebase to report when no CLINT is mapped, in Hz",
        },
    ],
    construct: |props| Ok(Box::new(BootRom::new(props)?)),
};

impl Device for BootRom {
    fn class(&self) -> &'static DeviceClass {
        &CLASS
    }

    fn realize(&self, _ctx: &mut RealizeCtx<'_>) -> Result<()> {
        Ok(())
    }

    fn reset(&self, _kind: ResetKind) {
        // Every reset, because the tree describes the machine and the machine
        // may have been rebuilt around it — and because realize ends with a
        // cold reset, which is when the first tree is built at all.
        if let Err(e) = self.regenerate() {
            self.rom.contents.lock().error = Some(e.to_string());
        }
    }

    fn region(&self, name: &str) -> Option<RegionRef> {
        matches!(name, "" | "rom").then(|| Arc::clone(&self.region))
    }

    // No `save`/`load`: the image is a pure function of the machine's shape,
    // which the snapshot header already records (`ROADMAP.md` §4.5), and it is
    // rebuilt by the reset that follows a load.
}

impl Instance for BootRom {
    fn bind(&self, ctx: &BindCtx<'_>) -> Result<()> {
        let space = ctx.space().ok_or_else(|| Error::Config {
            at: ctx.path().to_string(),
            message: String::from("a boot ROM needs an address space to describe (`space = mem`)"),
        })?;
        self.rom.contents.lock().space = Some(Arc::clone(space));
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
    bindings.bind(CLASS_NAME, |props| Ok(Arc::new(BootRom::new(props)?)))
}

/// What the validator should know about `riscv.boot`.
#[must_use]
pub fn schema() -> crate::machine::validate::ClassSchema {
    use crate::machine::validate::{ClassSchema, PropSchema};
    ClassSchema::new(CLASS_NAME)
        .prop(PropSchema::new("size", ValueKind::Size))
        .prop(PropSchema::new("entry", ValueKind::Addr))
        .prop(PropSchema::new("harts", ValueKind::Uint).range(1, 4096))
        .prop(PropSchema::new("boot-hart", ValueKind::Uint))
        .prop(PropSchema::new("isa", ValueKind::Str))
        .prop(PropSchema::new("mmu", ValueKind::Str))
        .prop(PropSchema::new("bootargs", ValueKind::Str))
        .prop(PropSchema::new("initrd", ValueKind::Media))
        .prop(PropSchema::new("initrd-addr", ValueKind::Addr))
        .prop(PropSchema::new("model", ValueKind::Str))
        .prop(PropSchema::new("timebase", ValueKind::Uint))
        .region("")
        .region("rom")
}
