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

/// Where the secondary processors' parking loop sits, in the reset vector's
/// own 128-byte slot and just past the entry literal.
///
/// A board with one processor never emits it, so its ROM is byte-identical to
/// the one this file generated before secondaries existed.
const SECONDARY_OFFSET: u64 = 0x40;

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

    /// `MRS Xt, <system register>` (C6.2.194).
    ///
    /// The five fields that name a system register go in at 19:5, with `op0`
    /// contributing only its low bit — the encoding has room for `op0` of 2
    /// and 3 and nothing else.
    #[must_use]
    pub const fn mrs(rt: u32, op0: u32, op1: u32, crn: u32, crm: u32, op2: u32) -> u32 {
        0xd530_0000
            | ((op0 & 1) << 19)
            | ((op1 & 7) << 16)
            | ((crn & 0xf) << 12)
            | ((crm & 0xf) << 8)
            | ((op2 & 7) << 5)
            | rt
    }

    /// `MRS Xt, MPIDR_EL1` — op0 3, op1 0, CRn 0, CRm 0, op2 5 (D19.2.100).
    #[must_use]
    pub const fn mrs_mpidr(rt: u32) -> u32 {
        mrs(rt, 3, 0, 0, 0, 5)
    }

    /// `AND Xd, Xn, #imm` — the 64-bit logical-immediate form (C6.2.11).
    ///
    /// The immediate is the `N:immr:imms` triple the architecture encodes
    /// bitmasks with rather than a plain number; [`mask_low_bits`] builds the
    /// one shape this module needs.
    #[must_use]
    pub const fn and_imm(rd: u32, rn: u32, n: u32, immr: u32, imms: u32) -> u32 {
        0x9200_0000
            | ((n & 1) << 22)
            | ((immr & 0x3f) << 16)
            | ((imms & 0x3f) << 10)
            | (rn << 5)
            | rd
    }

    /// The `(N, immr, imms)` triple for a mask of the low `bits` bits, which
    /// is a run of ones ending at bit 0: `N = 1`, `immr = 0`, `imms = bits-1`
    /// (DDI 0487 J1, `DecodeBitMasks`).
    #[must_use]
    pub const fn mask_low_bits(bits: u32) -> (u32, u32, u32) {
        (1, 0, bits - 1)
    }

    /// `ADD Xd, Xn, Xm, LSL #shift` — the shifted-register form (C6.2.5).
    #[must_use]
    pub const fn add_lsl(rd: u32, rn: u32, rm: u32, shift: u32) -> u32 {
        0x8b00_0000 | (rm << 16) | ((shift & 0x3f) << 10) | (rn << 5) | rd
    }

    /// `LDR Xt, [Xn]` — the unsigned-offset form with an offset of zero
    /// (C6.2.132).
    #[must_use]
    pub const fn ldr_base(rt: u32, rn: u32) -> u32 {
        0xf940_0000 | (rn << 5) | rt
    }

    /// `CBZ Xt, label`, `words` instructions away (C6.2.40).
    #[must_use]
    pub const fn cbz(rt: u32, words: i32) -> u32 {
        0xb400_0000 | (((words as u32) & 0x0007_ffff) << 5) | rt
    }

    /// `CBNZ Xt, label`, `words` instructions away (C6.2.39).
    #[must_use]
    pub const fn cbnz(rt: u32, words: i32) -> u32 {
        0xb500_0000 | (((words as u32) & 0x0007_ffff) << 5) | rt
    }

    /// `STR Xt, [Xn]` — the unsigned-offset form with an offset of zero
    /// (C6.2.273). The counterpart of [`ldr_base`], and what writes a word of
    /// a release table.
    #[must_use]
    pub const fn str_base(rt: u32, rn: u32) -> u32 {
        0xf900_0000 | (rn << 5) | rt
    }

    /// `ADD Xd, Xn, #imm12` — the immediate form with no shift (C6.2.4).
    #[must_use]
    pub const fn add_imm(rd: u32, rn: u32, imm: u32) -> u32 {
        0x9100_0000 | ((imm & 0xfff) << 10) | (rn << 5) | rd
    }

    /// `MOVZ`/`MOVK` for a whole 64-bit constant, low halfword first.
    ///
    /// Four instructions, always: a shorter sequence would depend on the
    /// value, and a stub whose length moved with its operand is a stub whose
    /// branch offsets move with it too.
    #[must_use]
    pub fn load64(rd: u32, value: u64) -> [u32; 4] {
        [
            movz(rd, (value & 0xffff) as u32),
            movk(rd, ((value >> 16) & 0xffff) as u32, 16),
            movk(rd, ((value >> 32) & 0xffff) as u32, 32),
            movk(rd, ((value >> 48) & 0xffff) as u32, 48),
        ]
    }

    /// `WFE` — wait for an event (C6.2.320).
    #[must_use]
    pub const fn wfe() -> u32 {
        0xd503_205f
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
        let secondary = r
            .or_enum("secondary", "psci", &["psci", "spin-table"])?
            .to_string();
        let release_addr = r.or_addr("release-addr", 0)?;
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

        // Every processor comes out of reset at the same address, so a board
        // with more than one needs somewhere for the others to wait: one
        // 64-bit word each, in RAM, zero until something releases them. The
        // address cannot be guessed here — it has to be memory the board knows
        // is free and the tree can reserve — so a multiprocessor board that
        // does not give one is refused rather than parked on address zero.
        if cpus > 1 && release_addr == 0 {
            return Err(Error::Property(format!(
                "property `release-addr`: this board has {cpus} processors and they all come out \
                 of reset at the same address, so the ones that are not the boot processor need a \
                 release table to wait on; give `release-addr` an address in RAM below the kernel"
            )));
        }
        if !release_addr.is_multiple_of(8) {
            return Err(Error::Property(format!(
                "property `release-addr`: a release table is 64-bit words and the guest writes \
                 them as such, so it must be 8-byte aligned; {release_addr:#x} is not"
            )));
        }

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
                    // What the *tree* says, which is a different question from
                    // where a parked processor waits: the parking loop is the
                    // same either way and `spin-table` is the method that
                    // needs no firmware behind an instruction.
                    enable_method: match (secondary.as_str(), psci.as_str()) {
                        ("spin-table", _) => String::from("spin-table"),
                        (_, "none") => String::new(),
                        _ => String::from("psci"),
                    },
                    release_addr: (secondary == "spin-table" && release_addr != 0)
                        .then_some(release_addr),
                    // Only a board with somewhere to park anything: a
                    // one-processor board that names a table would reserve a
                    // page of the guest's RAM for nobody.
                    parked_at: (cpus > 1 && release_addr != 0).then_some(release_addr),
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

    /// The six-instruction stub, followed by the entry address literal — and,
    /// on a board with more than one processor, the three that pick a path and
    /// the parking loop the other processors take.
    ///
    /// # Why the reset vector branches at all
    ///
    /// Every processor on this board comes out of reset at the same address,
    /// because that is what a reset vector *is*. A stub that unconditionally
    /// jumped to the kernel would therefore enter the kernel once per
    /// processor, on all of them at once, which is not a boot. So the stub
    /// reads `MPIDR_EL1`, and everything but affinity 0 goes to the parking
    /// loop below and waits to be told where to go:
    ///
    /// ```text
    ///   mrs  x9, mpidr_el1
    ///   and  x9, x9, #0xff        ; Aff0, which is this board's cpu index
    ///   cbnz x9, secondary
    ///   ; … the primary path, unchanged …
    /// secondary:
    ///   movz x10, #<release-addr>  ; four halfwords
    ///   add  x10, x10, x9, lsl #3  ; &table[index]
    /// 1: wfe
    ///   ldr  x11, [x10]
    ///   cbz  x11, 1b
    ///   mov  x0, #0 … x3, #0
    ///   br   x11
    /// ```
    ///
    /// That table is the board's **warm-boot entry point**: one 64-bit word
    /// per processor, zero until somebody writes an address into it. Which is
    /// exactly the shape a spin table has (`cpu-release-addr`, Devicetree
    /// Specification v0.4 §3.8.1) *and* exactly what a PSCI `CPU_ON` would
    /// have to write, so the two boot methods differ only in what the device
    /// tree tells the guest.
    #[must_use]
    pub fn stub(&self) -> Vec<u8> {
        let mut words: Vec<u32> = Vec::new();
        if self.config.cpus.mpidr.len() > 1 {
            words.push(asm::mrs_mpidr(9));
            let (n, immr, imms) = asm::mask_low_bits(8);
            words.push(asm::and_imm(9, 9, n, immr, imms));
            // From here to the parking loop, in instructions.
            let here = words.len() as i32;
            words.push(asm::cbnz(9, (SECONDARY_OFFSET / 4) as i32 - here));
        }
        // Both PC-relative operands are measured from the instruction that
        // carries them, so neither can be a constant once something may sit in
        // front of them.
        let adr_at = words.len() as u64 * 4;
        words.push(asm::adr(0, (DTB_OFFSET - adr_at) as i32));
        words.push(asm::movz(1, 0));
        words.push(asm::movz(2, 0));
        words.push(asm::movz(3, 0));
        let ldr_at = words.len() as u64 * 4;
        words.push(asm::ldr_literal(4, (ENTRY_OFFSET - ldr_at) as i32));
        words.push(asm::br(4));

        let mut out = Vec::with_capacity(SECONDARY_OFFSET as usize);
        for word in &words {
            out.extend_from_slice(&word.to_le_bytes());
        }
        out.resize(ENTRY_OFFSET as usize, 0);
        out.extend_from_slice(&self.entry.to_le_bytes());
        if self.config.cpus.mpidr.len() > 1 {
            out.resize(SECONDARY_OFFSET as usize, 0);
            for word in self.parking_loop() {
                out.extend_from_slice(&word.to_le_bytes());
            }
        }
        out
    }

    /// The instructions a processor other than the first executes forever, or
    /// until its word of the release table stops being zero.
    ///
    /// `x9` holds the processor's index on entry, which the reset vector has
    /// already masked out of `MPIDR_EL1`.
    #[must_use]
    pub fn parking_loop(&self) -> Vec<u32> {
        // `parked_at` and not `release_addr`: the first is where processors
        // wait, the second is only whether the *tree* tells the guest about
        // it. A `psci` board parks its secondaries on the same table and would
        // otherwise be given address zero — which is the ROM, whose first word
        // is not zero, so every secondary would branch into an instruction
        // encoding the instant it looked.
        let release = self.config.cpus.parked_at.unwrap_or(0);
        let mut words: Vec<u32> = asm::load64(10, release).to_vec();
        // Eight bytes per entry, so the index shifts left by three.
        words.push(asm::add_lsl(10, 10, 9, 3));
        // `WFE` is a hint and this core treats it as one, so the loop is
        // correct whether or not anything ever signals an event — a released
        // processor is one that read a non-zero word, not one that woke up.
        let spin = words.len() as i32;
        words.push(asm::wfe());
        words.push(asm::ldr_base(11, 10));
        words.push(asm::cbz(11, spin - (words.len() as i32)));
        // The released processor enters with the same register state the
        // primary one did, less the device tree: it is not the boot processor
        // and has no argument.
        words.push(asm::movz(0, 0));
        words.push(asm::movz(1, 0));
        words.push(asm::movz(2, 0));
        words.push(asm::movz(3, 0));
        words.push(asm::br(11));
        words
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
            name: "secondary",
            kind: ValueKind::Str,
            required: false,
            summary: "how the tree says the other processors are started: `psci` or \
                      `spin-table` (default `psci`)",
        },
        PropertySpec {
            name: "release-addr",
            kind: ValueKind::Addr,
            required: false,
            summary: "where the release table of one 64-bit word per processor lives; \
                      required once `cpus` is more than one",
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
        .prop(PropSchema::new("secondary", ValueKind::Str).values(&["psci", "spin-table"]))
        .prop(PropSchema::new("release-addr", ValueKind::Addr))
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

    /// A two-processor ROM with its release table a page into DRAM.
    fn smp_rom() -> BootRom {
        use crate::core::props::Value;
        let props = Props::new()
            .with("entry", Value::Addr(0x4020_0000))
            .with("cpus", 2u64)
            .with("secondary", "spin-table")
            .with("release-addr", Value::Addr(0x4000_1000));
        BootRom::new(&props).expect("a two-processor boot ROM")
    }

    #[test]
    fn a_one_processor_rom_is_the_stub_it_always_was() {
        // The regression that matters: `machines/arm64-virt.machine` boots a
        // distribution kernel, and a second core must not change one byte of
        // the ROM it resets into.
        let rom = rom(0x4020_0000);
        assert_eq!(rom.stub().len(), ENTRY_OFFSET as usize + 8);
        assert_eq!(words(&rom).len(), (ENTRY_OFFSET / 4) as usize + 2);
        assert_eq!(words(&rom)[0], asm::adr(0, DTB_OFFSET as i32));
    }

    #[test]
    fn the_smp_encodings_are_the_ones_the_manual_gives() {
        // Independently known encodings again, so the four instructions the
        // parking loop needs are checked against DDI 0487 rather than against
        // the functions that produced them.
        assert_eq!(asm::mrs_mpidr(0), 0xd538_00a0, "mrs x0, mpidr_el1");
        let (n, immr, imms) = asm::mask_low_bits(8);
        assert_eq!(
            asm::and_imm(0, 0, n, immr, imms),
            0x9240_1c00,
            "and x0, x0, #0xff"
        );
        assert_eq!(
            asm::add_lsl(0, 0, 1, 3),
            0x8b01_0c00,
            "add x0, x0, x1, lsl #3"
        );
        assert_eq!(asm::ldr_base(0, 1), 0xf940_0020, "ldr x0, [x1]");
        assert_eq!(asm::cbnz(0, 2), 0xb500_0040, "cbnz x0, .+8");
        assert_eq!(asm::cbz(0, 2), 0xb400_0040, "cbz x0, .+8");
        assert_eq!(asm::movk(0, 0, 48), 0xf2e0_0000, "movk x0, #0, lsl #48");
        assert_eq!(asm::wfe(), 0xd503_205f);
    }

    #[test]
    fn a_two_processor_reset_vector_sends_everything_but_affinity_zero_to_the_loop() {
        let rom = smp_rom();
        let w = words(&rom);
        assert_eq!(w[0], asm::mrs_mpidr(9));
        let (n, immr, imms) = asm::mask_low_bits(8);
        assert_eq!(w[1], asm::and_imm(9, 9, n, immr, imms), "Aff0");
        // The branch is taken to `SECONDARY_OFFSET`, counted from the branch.
        assert_eq!(w[2], asm::cbnz(9, (SECONDARY_OFFSET / 4) as i32 - 2));
        // And the primary path is the same six instructions, three words
        // further on, with both PC-relative operands moved with them.
        assert_eq!(w[3], asm::adr(0, (DTB_OFFSET - 0x0c) as i32));
        assert_eq!(w[7], asm::ldr_literal(4, (ENTRY_OFFSET - 0x1c) as i32));
        assert_eq!(w[8], asm::br(4));
        let bytes = rom.stub();
        let at = ENTRY_OFFSET as usize;
        assert_eq!(
            u64::from_le_bytes(bytes[at..at + 8].try_into().unwrap()),
            0x4020_0000,
            "the entry literal is still where the load points"
        );
    }

    #[test]
    fn the_parking_loop_waits_on_this_processors_word_of_the_release_table() {
        let rom = smp_rom();
        let loop_words = rom.parking_loop();
        // The address is built a halfword at a time, low first.
        assert_eq!(loop_words[0], asm::movz(10, 0x1000));
        assert_eq!(loop_words[1], asm::movk(10, 0x4000, 16));
        assert_eq!(loop_words[2], asm::movk(10, 0, 32));
        assert_eq!(loop_words[3], asm::movk(10, 0, 48));
        // Eight bytes an entry: the index in `x9` shifts left by three.
        assert_eq!(loop_words[4], asm::add_lsl(10, 10, 9, 3));
        assert_eq!(loop_words[5], asm::wfe());
        assert_eq!(loop_words[6], asm::ldr_base(11, 10));
        // Back two instructions, to the `wfe`.
        assert_eq!(loop_words[7], asm::cbz(11, -2));
        assert_eq!(*loop_words.last().unwrap(), asm::br(11));
        // It fits in the reset vector's own slot, which is the only place it
        // can be: every other slot is an exception vector.
        assert!(SECONDARY_OFFSET + loop_words.len() as u64 * 4 <= VECTOR_STRIDE);
        // And it is where the reset vector branches to.
        let bytes = rom.stub();
        let at = SECONDARY_OFFSET as usize;
        assert_eq!(
            u32::from_le_bytes(bytes[at..at + 4].try_into().unwrap()),
            loop_words[0]
        );
    }

    #[test]
    fn a_psci_board_parks_its_secondaries_on_the_table_the_tree_does_not_mention() {
        // The two questions are separate and getting them confused is a live
        // defect: `secondary = "psci"` publishes no `cpu-release-addr`, but the
        // processors still have to wait *somewhere*, and a parking loop given
        // address zero would load the ROM's own first instruction word and
        // branch to it.
        use crate::core::props::Value;
        let props = Props::new()
            .with("cpus", 2u64)
            .with("release-addr", Value::Addr(0x4000_1000));
        let rom = BootRom::new(&props).expect("a two-processor PSCI board");
        assert_eq!(rom.config.cpus.enable_method, "psci");
        assert_eq!(rom.config.cpus.release_addr, None, "not in the tree");
        assert_eq!(rom.config.cpus.parked_at, Some(0x4000_1000), "but real");
        assert_eq!(rom.parking_loop()[0], asm::movz(10, 0x1000));
        // And a board with one processor reserves nothing, because there is
        // nothing waiting on it.
        let props = Props::new().with("release-addr", Value::Addr(0x4000_1000));
        let one = BootRom::new(&props).expect("one processor");
        assert_eq!(one.config.cpus.parked_at, None);
    }

    #[test]
    fn a_multiprocessor_rom_without_a_release_table_is_refused() {
        // Two processors resetting to the same address and nowhere to park one
        // of them is a board that enters the kernel twice.
        let props = Props::new().with("cpus", 2u64);
        let e = BootRom::new(&props)
            .expect_err("no release table")
            .to_string();
        assert!(e.contains("release-addr"), "{e}");
        // And a table a 64-bit store cannot land on squarely.
        let props = Props::new()
            .with("cpus", 2u64)
            .with("release-addr", crate::core::props::Value::Addr(0x4000_1004));
        assert!(BootRom::new(&props).is_err(), "misaligned");
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
