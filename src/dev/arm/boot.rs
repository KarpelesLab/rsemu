//! The boot ROM: six instructions and the device tree they hand over.
//!
//! An AArch64 core comes out of reset at `RVBAR_EL1` with nothing in its
//! registers, the MMU off, the caches off and every asynchronous exception
//! masked. The software it is about to run expects one thing by convention:
//! **`x0` holds the physical address of a flattened device tree**, and `x1`,
//! `x2` and `x3` hold zero. Somebody has to put them there. On real hardware
//! it is firmware; here it is this.
//!
//! ```text
//!   +0x00  adr x0, .+0x40   ; x0 = this ROM's own base + 0x40 — the tree
//!   +0x04  movz x1, #0
//!   +0x08  movz x2, #0
//!   +0x0c  movz x3, #0
//!   +0x10  ldr  x4, .+0x28  ; x4 = the entry address, stored at +0x38
//!   +0x14  br   x4
//!   +0x38  .dword entry
//!   +0x40  the generated device tree
//! ```
//!
//! Position-independent on purpose: `ADR` is PC-relative, so the stub works
//! wherever a machine file maps it and the ROM's own address appears nowhere
//! in the ROM. The tree lands 8-byte aligned, which the device tree
//! specification asks for and which a reader that copies the blob with 64-bit
//! loads depends on.
//!
//! # Which exception level the kernel is entered at
//!
//! **EL1**, because that is the highest level `cpu.arm.a64` implements —
//! `ID_AA64PFR0_EL1` reports EL0 and EL1 and nothing above them, and a guest
//! that reads it is told so. A kernel entered at EL1 simply has no hypervisor
//! of its own; the ARM boot-wrapper (BSD-3-Clause) has the same case for
//! Armv8-R, where its comment says plainly that Linux is booted at EL1 there.
//! What this board therefore cannot host is a guest hypervisor, which is not
//! something a single-core `virt` board was going to do.
//!
//! # The device tree is generated, not stored
//!
//! [`dt::generate`](super::dt::generate) walks the address space this device
//! was bound to and produces the tree from what is actually mapped there. The
//! image is rebuilt on every reset, which is the first moment the whole
//! machine graph exists — see [`dt`](super::dt) for why that is the only
//! correct moment, and for the three things the tree cannot derive.
//!
//! # Sources
//!
//! *Arm Architecture Reference Manual for A-profile*, DDI 0487, C4.1 for the
//! four instruction encodings in [`asm`]; the *Devicetree Specification* v0.4
//! for the blob's alignment; and, for the register hand-off, the ARM
//! boot-wrapper (BSD-3-Clause), whose `jump_kernel(addr, &dtb, 0, 0, 0)` is
//! the same convention written in C. The Linux boot documentation states it
//! too and is GPL-2.0; it was not read.

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

use super::dt::{Conduit, CpuSpec, TreeConfig};

/// The class name a machine description writes.
pub const CLASS_NAME: &str = "arm.boot";

/// Where the generated tree starts inside the ROM.
///
/// Past the sixteen exception vector slots, for the reason [`catch`] gives.
///
/// [`catch`]: BootRom::catch
pub const DTB_OFFSET: u64 = 0x800;

/// Where the entry address literal sits inside the ROM.
const ENTRY_OFFSET: u64 = 0x38;

/// How far apart AArch64's sixteen exception vector slots are (DDI 0487
/// D1.10.2): 128 bytes each, sixteen of them, 2 KiB in total.
const VECTOR_STRIDE: u64 = 0x80;

/// How many of them there are.
const VECTORS: u64 = 16;

/// How much address space the ROM answers by default.
///
/// 64 KiB. A generated tree for a board of this shape is a couple of
/// kilobytes; the headroom is what lets a board grow devices without the ROM
/// becoming the thing that stops it.
pub const DEFAULT_SIZE: u64 = 0x1_0000;

/// Where the stub jumps when a machine file does not say.
///
/// The same address `loader::DEFAULT_ADDR` puts a kernel at, and it has to be:
/// the ROM jumps to the kernel's first instruction.
pub const DEFAULT_ENTRY: u64 = 0x4020_0000;

/// The generic timer's four private interrupt numbers, in the order the
/// `arm,armv8-timer` binding lists them: secure physical, non-secure physical,
/// virtual, hypervisor.
///
/// These are the numbers every AArch64 board uses. This core has no secure
/// state and no EL2, so only the middle two exist as hardware; the other two
/// are described because the binding's list is positional and a kernel picks
/// the one that matches the exception level it was entered at.
pub const DEFAULT_TIMER_PPI: [u32; 4] = [13, 14, 11, 10];

/// `PSCI_SYSTEM_OFF`'s function identifier (DEN 0022 §5.1.9).
///
/// Written here rather than reached for through `cpu::arm::a64` because a
/// board's ROM is not allowed to depend on which core it is running on: this
/// is a number the *guest* interface defines, and the core that answers it is
/// the machine file's business.
const PSCI_SYSTEM_OFF: u32 = 0x8400_0008;

/// Encoders for the four instructions the stub needs.
///
/// Written out rather than assembled from a string, and built from the
/// encodings in DDI 0487 C4.1 rather than from a table of magic numbers, so
/// the stub can be read against the manual.
pub mod asm {
    /// `ADR Xd, label` — a PC-relative byte address, ±1 MiB (C6.2.10).
    ///
    /// The 21-bit immediate is split: its low two bits sit at 30:29 and the
    /// other nineteen at 23:5, which is the encoding's one genuine oddity.
    #[must_use]
    pub const fn adr(rd: u32, offset: i32) -> u32 {
        let imm = (offset as u32) & 0x001f_ffff;
        0x1000_0000 | ((imm & 3) << 29) | ((imm >> 2) << 5) | rd
    }

    /// `MOVZ Xd, #imm16` — a 64-bit move of a zero-extended halfword
    /// (C6.2.191). With `imm` zero this is `mov xd, #0`.
    #[must_use]
    pub const fn movz(rd: u32, imm: u32) -> u32 {
        0xd280_0000 | ((imm & 0xffff) << 5) | rd
    }

    /// `LDR Xt, label` — a 64-bit PC-relative literal load (C6.2.131). The
    /// 19-bit immediate counts *words*, so the label must be 4-byte aligned.
    #[must_use]
    pub const fn ldr_literal(rt: u32, offset: i32) -> u32 {
        let words = ((offset / 4) as u32) & 0x0007_ffff;
        0x5800_0000 | (words << 5) | rt
    }

    /// `BR Xn` — branch to a register, with no link (C6.2.36).
    #[must_use]
    pub const fn br(rn: u32) -> u32 {
        0xd61f_0000 | (rn << 5)
    }

    /// `MOVK Xd, #imm16, LSL #shift` (C6.2.190).
    #[must_use]
    pub const fn movk(rd: u32, imm: u32, shift: u32) -> u32 {
        0xf280_0000 | ((shift / 16) << 21) | ((imm & 0xffff) << 5) | rd
    }

    /// `SMC #imm16` (C6.2.259).
    #[must_use]
    pub const fn smc(imm: u32) -> u32 {
        0xd400_0003 | ((imm & 0xffff) << 5)
    }

    /// `B` to a target `words` instructions away (C6.2.26).
    #[must_use]
    pub const fn b(words: i32) -> u32 {
        0x1400_0000 | ((words as u32) & 0x03ff_ffff)
    }
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
                // ones because a core that runs off the end should hit
                // `UNDEFINED` rather than a valid instruction.
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
        // Instruction fetch, a `memcpy` of the tree, and a debugger dump all
        // read this, at every width and in bursts.
        AccessConstraints::ANY
    }
}

/// The reset vector and the generated device tree.
#[derive(Debug)]
pub struct BootRom {
    rom: Arc<Rom>,
    region: RegionRef,
    entry: u64,
    config: TreeConfig,
    /// What this machine's devices published about themselves.
    ///
    /// A field rather than a `static`: the tree is regenerated at reset, when
    /// `&self` is all there is, and the table has to be *this* board's. A
    /// `BootRom` built outside a build gets an empty one of its own and
    /// describes a machine with no peripherals, which is the honest answer for
    /// a device with no machine.
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
        let cpus = r.or_range("cpus", 1u64, 1..=256)?;
        let mpidr = r.or("mpidr", 0x8000_0000u64)?;
        let cpu_compatible = r.or("cpu-compatible", String::from("arm,armv8"))?;
        let psci = r
            .or_enum("psci", "smc", &["smc", "hvc", "none"])?
            .to_string();
        let bootargs = r.or("bootargs", String::new())?;
        // The ramdisk is *described* here and *placed* by an `arm.loader`, so
        // the same media slot is named twice and its length is read out of the
        // bytes rather than written down a second time. Only the address is
        // repeated, which is the same duplication a `map` statement already
        // has, and a wrong one is visible in the generated tree.
        let initrd_len = r.optional_media("initrd")?.map_or(0, Media::len);
        let initrd_addr = r.or_addr("initrd-addr", 0)?;
        let model = r.or("model", String::from("rsemu arm64-virt"))?;
        let apb_clock = r.or_range("apb-clock", 24_000_000u64, 1..=u64::from(u32::MAX))?;
        let timer = r.or("timer", true)?;
        r.finish()?;

        let initrd = if initrd_len == 0 {
            None
        } else if initrd_addr == 0 {
            return Err(Error::Property(String::from(
                "property `initrd`: a ramdisk was bound but `initrd-addr` says where it is not; \
                 give the address the `arm.loader` staging it writes to",
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

        // Affinity values for the processors the tree describes. A single-core
        // board declares one `mpidr`; an SMP board's cores differ in Aff0,
        // which is what a kernel matches its `reg` properties against.
        let affinities: Vec<u64> = (0..cpus).map(|i| mpidr.wrapping_add(i)).collect();

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
            CLASS_NAME,
            size,
            Arc::clone(&rom) as Arc<dyn MemOps>,
        ));
        Ok(BootRom {
            rom,
            region,
            entry,
            dt,
            config: TreeConfig {
                model,
                bootargs,
                initrd,
                cpus: CpuSpec {
                    mpidr: affinities,
                    compatible: cpu_compatible,
                    enable_method: if psci == "none" {
                        String::new()
                    } else {
                        String::from("psci")
                    },
                },
                psci: match psci.as_str() {
                    "smc" => Some(Conduit::Smc),
                    "hvc" => Some(Conduit::Hvc),
                    _ => None,
                },
                timer_ppi: if timer {
                    DEFAULT_TIMER_PPI.to_vec()
                } else {
                    Vec::new()
                },
                apb_clock_hz: apb_clock as u32,
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

    /// The six-instruction stub, followed by the entry address literal.
    #[must_use]
    pub fn stub(&self) -> Vec<u8> {
        let words = [
            asm::adr(0, DTB_OFFSET as i32),
            asm::movz(1, 0),
            asm::movz(2, 0),
            asm::movz(3, 0),
            asm::ldr_literal(4, ENTRY_OFFSET as i32 - 0x10),
            asm::br(4),
        ];
        let mut out = Vec::with_capacity(ENTRY_OFFSET as usize + 8);
        for word in words {
            out.extend_from_slice(&word.to_le_bytes());
        }
        out.resize(ENTRY_OFFSET as usize, 0);
        out.extend_from_slice(&self.entry.to_le_bytes());
        out
    }

    /// The four instructions that fill each unused exception vector slot.
    ///
    /// # Why a boot ROM has an exception vector table at all
    ///
    /// `VBAR_EL1` resets to **zero** on this board, because that is where the
    /// ROM is mapped — so until a guest writes its own `VBAR_EL1`, every
    /// exception it takes lands *here*, in the sixteen 128-byte slots this
    /// window happens to cover. A ROM that left them as unprogrammed zeros
    /// gets a guest that takes an `UNDEFINED` at the vector, takes another one
    /// at the same vector, and spins there forever with `ELR_EL1` and
    /// `ESR_EL1` overwritten by each trip round — which is *precisely* the
    /// state in which nobody can tell what went wrong.
    ///
    /// So the ROM fills them with a default handler, exactly as firmware does:
    ///
    /// ```text
    ///   movz x0, #0x0008          ; PSCI_SYSTEM_OFF, low half
    ///   movk x0, #0x8400, lsl #16 ; and high
    ///   smc  #0
    ///   b    .
    /// ```
    ///
    /// An unhandled early exception therefore **stops the machine**, with
    /// `ESR_EL1` still naming what happened and `ELR_EL1` still naming the
    /// instruction that did it. That is the difference between "the kernel
    /// hangs" and "the kernel executed `MRS x0, ID_AA64DFR0_EL1` and this core
    /// does not implement it", and this board exists to produce the second
    /// kind of answer.
    ///
    /// With `psci = "none"` there is nothing to call, so the handler is the
    /// self-branch alone — which still preserves the syndrome, and still beats
    /// re-entering the vector.
    #[must_use]
    pub fn catch(&self) -> Vec<u32> {
        match self.config.psci {
            Some(_) => alloc::vec![
                asm::movz(0, PSCI_SYSTEM_OFF & 0xffff),
                asm::movk(0, PSCI_SYSTEM_OFF >> 16, 16),
                asm::smc(0),
                asm::b(0),
            ],
            None => alloc::vec![asm::b(0)],
        }
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
        image.resize((VECTORS * VECTOR_STRIDE) as usize, 0);
        // Slot 0 is the reset vector and is already the stub; the other
        // fifteen get the default handler. See [`BootRom::catch`].
        let catch = self.catch();
        for slot in 1..VECTORS {
            let at = (slot * VECTOR_STRIDE) as usize;
            for (i, word) in catch.iter().enumerate() {
                image[at + i * 4..at + i * 4 + 4].copy_from_slice(&word.to_le_bytes());
            }
        }
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

/// The `arm.boot` device class.
pub static CLASS: DeviceClass = DeviceClass {
    name: CLASS_NAME,
    version: 1,
    summary: "the reset vector, and the device tree generated from the realized machine",
    properties: &[
        PropertySpec {
            name: "size",
            kind: ValueKind::Size,
            required: false,
            summary: "how much address space the ROM answers (default 64K)",
        },
        PropertySpec {
            name: "entry",
            kind: ValueKind::Addr,
            required: false,
            summary: "the address the stub jumps to (default 0x40200000)",
        },
        PropertySpec {
            name: "cpus",
            kind: ValueKind::Uint,
            required: false,
            summary: "how many processors the tree describes (default 1)",
        },
        PropertySpec {
            name: "mpidr",
            kind: ValueKind::Uint,
            required: false,
            summary: "the first processor's MPIDR_EL1; the rest count up in Aff0",
        },
        PropertySpec {
            name: "cpu-compatible",
            kind: ValueKind::Str,
            required: false,
            summary: "the `compatible` each cpu node carries (default arm,armv8)",
        },
        PropertySpec {
            name: "psci",
            kind: ValueKind::Str,
            required: false,
            summary: "the firmware conduit: `smc`, `hvc`, or `none` (default smc)",
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
                      because an `arm.loader` is what puts it in memory",
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
            name: "apb-clock",
            kind: ValueKind::Uint,
            required: false,
            summary: "what the generated `apb-pclk` fixed clock is rated at, in Hz",
        },
        PropertySpec {
            name: "timer",
            kind: ValueKind::Bool,
            required: false,
            summary: "whether to describe the generic timer at all (default true)",
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

/// What the validator should know about `arm.boot`.
#[must_use]
pub fn schema() -> crate::machine::validate::ClassSchema {
    use crate::machine::validate::{ClassSchema, PropSchema};
    ClassSchema::new(CLASS_NAME)
        .prop(PropSchema::new("size", ValueKind::Size))
        .prop(PropSchema::new("entry", ValueKind::Addr))
        .prop(PropSchema::new("cpus", ValueKind::Uint).range(1, 256))
        .prop(PropSchema::new("mpidr", ValueKind::Uint))
        .prop(PropSchema::new("cpu-compatible", ValueKind::Str))
        .prop(PropSchema::new("psci", ValueKind::Str).values(&["smc", "hvc", "none"]))
        .prop(PropSchema::new("bootargs", ValueKind::Str))
        .prop(PropSchema::new("initrd", ValueKind::Media))
        .prop(PropSchema::new("initrd-addr", ValueKind::Addr))
        .prop(PropSchema::new("model", ValueKind::Str))
        .prop(PropSchema::new("apb-clock", ValueKind::Uint))
        .prop(PropSchema::new("timer", ValueKind::Bool))
        .region("")
        .region("rom")
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The stub, as words, so a test can read it the way the core will.
    fn words(rom: &BootRom) -> Vec<u32> {
        rom.stub()
            .chunks(4)
            .map(|b| u32::from_le_bytes([b[0], b[1], b[2], b[3]]))
            .collect()
    }

    fn rom(entry: u64) -> BootRom {
        let props = Props::new().with("entry", crate::core::props::Value::Addr(entry));
        BootRom::new(&props).expect("a boot ROM with a default everything else")
    }

    #[test]
    fn the_stub_encodes_the_instructions_the_manual_says() {
        // Independently known encodings, so the assembler is checked against
        // DDI 0487 rather than against itself.
        assert_eq!(asm::movz(1, 0), 0xd280_0001, "mov x1, #0");
        assert_eq!(asm::movz(3, 0), 0xd280_0003, "mov x3, #0");
        assert_eq!(asm::br(4), 0xd61f_0080, "br x4");
        // `adr x0, .+4` sets imm21 = 4: the low two bits are zero and the
        // nineteen above them are 1, so bit 5 is the only one set.
        assert_eq!(asm::adr(0, 4), 0x1000_0020);
        // `ldr x4, .+8` counts words: imm19 = 2, at bit 5.
        assert_eq!(asm::ldr_literal(4, 8), 0x5800_0044);
    }

    #[test]
    fn the_stub_points_x0_at_the_tree_and_x4_at_the_entry() {
        let rom = rom(0x4020_0000);
        let w = words(&rom);
        assert_eq!(w[0], asm::adr(0, DTB_OFFSET as i32), "adr x0, .+0x40");
        assert_eq!(w[1], asm::movz(1, 0));
        assert_eq!(w[2], asm::movz(2, 0));
        assert_eq!(w[3], asm::movz(3, 0));
        // The literal load is relative to *its own* address, not to the base.
        assert_eq!(w[4], asm::ldr_literal(4, (ENTRY_OFFSET - 0x10) as i32));
        assert_eq!(w[5], asm::br(4));
        // And the literal is where the load points.
        let at = ENTRY_OFFSET as usize;
        let bytes = rom.stub();
        assert_eq!(
            u64::from_le_bytes(bytes[at..at + 8].try_into().unwrap()),
            0x4020_0000
        );
    }

    #[test]
    fn the_tree_lands_eight_byte_aligned() {
        // The specification asks for it, and a reader that copies the blob
        // with 64-bit loads needs it. Read through a binding so that this is
        // a check of the constants rather than of a literal the compiler has
        // already folded.
        let (dtb, entry) = (DTB_OFFSET, ENTRY_OFFSET);
        assert!(dtb.is_multiple_of(8));
        assert!(entry.is_multiple_of(8));
        assert!(entry + 8 <= dtb, "the literal fits before the tree");
    }

    #[test]
    fn the_unused_vector_slots_hold_a_handler_that_stops_the_machine() {
        // `VBAR_EL1` resets to zero and zero is this ROM, so the sixteen slots
        // are the guest's exception vectors until it installs its own. A ROM
        // that left them as zeros turns the first unhandled exception into a
        // spin with the syndrome overwritten -- see `BootRom::catch`.
        let rom = rom(0x4020_0000);
        let catch = rom.catch();
        assert_eq!(catch.len(), 4, "movz, movk, smc, b .");
        assert_eq!(catch[2], asm::smc(0));
        assert_eq!(catch[3], asm::b(0), "and then it stops rather than falls");
        // With no firmware to call there is nothing to do but stop moving.
        let props = Props::new()
            .with("psci", "none")
            .with("entry", crate::core::props::Value::Addr(0x4020_0000));
        let quiet = BootRom::new(&props).expect("a board with no PSCI");
        assert_eq!(quiet.catch(), alloc::vec![asm::b(0)]);
    }

    #[test]
    fn the_vector_table_is_sixteen_slots_and_the_tree_starts_after_it() {
        // DDI 0487 D1.10.2: sixteen vectors, 128 bytes apart, 2 KiB in all.
        assert_eq!(VECTORS * VECTOR_STRIDE, DTB_OFFSET);
        assert_eq!(VECTOR_STRIDE, 0x80);
    }

    #[test]
    fn a_rom_with_no_room_for_a_tree_is_refused_at_construction() {
        let props = Props::new().with("size", crate::core::props::Value::Size(0x40));
        let e = BootRom::new(&props).expect_err("too small").to_string();
        assert!(e.contains("device tree"), "{e}");
    }

    #[test]
    fn a_ramdisk_with_no_address_is_refused_rather_than_described_wrongly() {
        let bytes: alloc::sync::Arc<[u8]> = alloc::vec![0u8; 16].into();
        let props = Props::new().with("initrd", crate::core::props::Media::new("initrd", bytes));
        let e = BootRom::new(&props).expect_err("no address").to_string();
        assert!(e.contains("initrd-addr"), "{e}");
    }

    #[test]
    fn a_board_with_no_machine_describes_one_with_no_memory() {
        // A `BootRom` built outside a build has an empty publication table and
        // no space, and must say so rather than panicking.
        let rom = rom(0x4020_0000);
        let e = rom.regenerate().expect_err("no space").to_string();
        assert!(e.contains("address space"), "{e}");
        assert!(rom.device_tree().is_empty());
    }
}
