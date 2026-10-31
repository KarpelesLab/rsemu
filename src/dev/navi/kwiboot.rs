//! Booting a navi's product system straight from its `LOADING.KWI`, with no
//! flash image.
//!
//! On the unit, power-on runs a chain that ends with the product system in
//! RAM: U-Boot in NOR boots a recovery Linux, whose `osloader` copies the
//! map card's KWI members into DRAM, builds a shared area for the product
//! kernel, and resets; U-Boot's warm path then checks the staged kernel and
//! jumps into it (`docs/platforms/alphard-navi-kwi-boot.md`). Every link of
//! that chain lives in the NOR, which only a physical dump of the chip
//! obtains.
//!
//! This device is the short circuit, in the shape `x86.linuxboot` is for a PC:
//! it leaves memory exactly as U-Boot leaves it at the jump, and enters the
//! kernel in the state U-Boot enters it in. With it a machine needs only the
//! KWI, and the KWI is on the map card: any of the eleven models boots from
//! its own.
//!
//! # What is staged, and where
//!
//! Each address comes from the KWI's own boot descriptor (`pmb`), not from
//! here:
//!
//! | what | where (HC59) | from |
//! | --- | --- | --- |
//! | the `rootfs` member, `PCRD` block first | `0x68000000` | `pmb` `rootfs_load` |
//! | the execute-in-place kernel | `0x62f80000` | `pmb` `kernel_load` |
//! | `USRCONF` | `0x6010c000` | `pmb` +0x1ec |
//! | the ATAG list | `0x62e00100` | `pmb` `load_base` + 0x100 |
//! | the OSMEM records | `0x60000000` | see below |
//!
//! # The OSMEM records
//!
//! The product kernel's `bif_init` reads a shared area at `0x60000000` that
//! the loader chain fills, and resets the unit when it finds it empty. The
//! records below are the ones the real chain leaves there, as captured at the
//! instant U-Boot jumps — the HC59 NOR booting the HC59 card in this emulator,
//! stopped on the kernel's first instruction. The model-specific fields (the
//! version tag, the KWI path, the USRCONF record, the kernel descriptor) are
//! recomputed from the KWI; the rest are written as they were seen. The
//! checksums were identified from that capture: the `KWI ` and `CONF`
//! records' are 32-bit additive word sums, the descriptor's is a CRC-32, and
//! the descriptor's magic `0x0a375fe6` is a constant U-Boot carries.
//!
//! Left out on purpose: the recovery system's own process table at
//! `0x60002040`, and its error-log counters at `0x6024e000`. Those describe
//! the run that loaded the image, which did not happen here.
//!
//! # Entering the kernel
//!
//! The ARM Linux convention: SVC mode with interrupts masked and the MMU and
//! caches off, which is how `cpu.arm` comes out of reset, and `r0 = 0`,
//! `r1` = the machine type, `r2` = the ATAG list. U-Boot passes machine type
//! 3386 (`0xd3a`), read off the registers at the captured jump. A four-word
//! stub does that from the reset vector: this device maps it over the first
//! page of the address space, above the NOR, and only when a KWI was given —
//! with the slot empty it maps nothing and the board boots its flash as
//! before.
//!
//! The stub has to get out of the way once it has jumped. The kernel probes
//! the NOR by CFI, with commands and reads in exactly that first page, and a
//! stub that stayed there made the probe fail — no MTD partitions, and the
//! product system fell over later for want of them. So the stub disarms
//! itself when the processor fetches its last word (the entry address), and
//! from then on passes every access through to the object `flash` names; a
//! reset arms it again. A debugger's read does not disarm it.
//!
//! What U-Boot does that this does not: program the DDR controller, the
//! local bus, the clocks and the pins. Those are `ram` stand-ins on this
//! board, which nothing checks.

use alloc::boxed::Box;
use alloc::format;
use alloc::string::{String, ToString};
use alloc::sync::Arc;
use alloc::vec;
use alloc::vec::Vec;

use crate::core::device::{Device, DeviceClass, PropertySpec, RealizeCtx, ResetKind};
use crate::core::error::{Error, Result};
use crate::core::props::{Media, Props, ValueKind};
use core::sync::atomic::{AtomicBool, Ordering};

use crate::core::error::BusError;
use crate::core::space::{
    AccessConstraints, AddressSpace, MemAttrs, MemOps, MemResult, Region, RequesterId,
};
use crate::core::sync::{LockRank, Mutex};
use crate::machine::realize::{BindCtx, Instance};

use super::kwi::{Kwi, crc32, word_sum};

/// The class name a machine description writes.
pub const CLASS_NAME: &str = "navi.kwiboot";

/// The machine type U-Boot passes in `r1`.
pub const DEFAULT_MACHINE_TYPE: u32 = 0xd3a;

/// Where DRAM ends: 1 GiB from `0x60000000`. The ATAG memory record covers
/// from the kernel's `load_base` to here, as U-Boot's does.
pub const DEFAULT_DRAM_END: u64 = 0xa000_0000;

/// The OSMEM base.
pub const OSMEM: u32 = 0x6000_0000;

/// The KWI path osloader records, `%ID%` standing for the model code.
pub const DEFAULT_KWI_PATH: &str = "/HD14/EXE/%ID%";

/// How far past `load_base` the ATAG list goes.
const ATAG_OFFSET: u32 = 0x100;

/// The descriptor magic U-Boot looks for at `OSMEM + 0x1000`.
const DESCRIPTOR_MAGIC: u32 = 0x0a37_5fe6;

/// How big the reset stub's page is.
const STUB_LEN: usize = 0x1000;

/// Where in the stub the entry address is: the last word the processor
/// fetches from it, and so the signal that it has gone.
const STUB_ENTRY: u64 = 0x1c;

/// The reset stub, and the flash it stands in front of.
#[derive(Debug)]
struct Stub {
    code: Vec<u8>,
    /// Answering with `code`; cleared once the entry address is fetched.
    armed: AtomicBool,
    /// The flash's region in a space of this object's own, if `flash` named
    /// one. `None` before bind, and on a machine with no flash.
    below: Mutex<Option<Arc<AddressSpace>>>,
}

impl MemOps for Stub {
    fn read(&self, offset: u64, dst: &mut [u8], attrs: MemAttrs) -> MemResult {
        if self.armed.load(Ordering::Acquire) {
            let end = offset as usize + dst.len();
            match self.code.get(offset as usize..end) {
                Some(bytes) => dst.copy_from_slice(bytes),
                None => return Err(BusError::Unassigned),
            }
            if !attrs.debug && (offset..end as u64).contains(&STUB_ENTRY) {
                self.armed.store(false, Ordering::Release);
            }
            return Ok(());
        }
        let below = self.below.lock().clone();
        match below {
            Some(space) => space.read_bytes(offset, dst, attrs),
            None => Err(BusError::Unassigned),
        }
    }

    fn write(&self, offset: u64, src: &[u8], attrs: MemAttrs) -> MemResult {
        if self.armed.load(Ordering::Acquire) {
            // Nothing writes here before the kernel runs; a ROM drops it.
            return Ok(());
        }
        let below = self.below.lock().clone();
        match below {
            Some(space) => space.write_bytes(offset, src, attrs),
            None => Err(BusError::Unassigned),
        }
    }

    fn constraints(&self) -> AccessConstraints {
        // Whatever is behind it decides, as for any pass-through decoder.
        AccessConstraints::ANY
    }
}

/// One stretch of bytes to write at reset.
#[derive(Debug, Clone)]
struct Piece {
    addr: u64,
    what: &'static str,
    source: Source,
}

/// Where a piece's bytes come from.
#[derive(Debug, Clone)]
enum Source {
    /// A byte range of the KWI.
    Kwi { offset: u64, len: u64 },
    /// Bytes built here.
    Built(Vec<u8>),
}

/// What the device learned when the machine bound it.
#[derive(Debug, Default)]
struct Binding {
    space: Option<Arc<AddressSpace>>,
    requester: RequesterId,
    /// What the last staging failed with. A reset cannot return an error.
    error: Option<String>,
    loads: u64,
}

/// The `navi.kwiboot` device.
#[derive(Debug)]
pub struct KwiBoot {
    image: Arc<[u8]>,
    /// Everything that goes into memory, in the order it is written. Empty
    /// when no KWI was given.
    pieces: Vec<Piece>,
    stub: Arc<Stub>,
    /// The object whose region the stub stands in front of.
    flash: Option<String>,
    model: Option<String>,
    binding: Mutex<Binding>,
}

fn words(values: &[u32]) -> Vec<u8> {
    values.iter().flat_map(|v| v.to_le_bytes()).collect()
}

/// `value` then its complement, as the OSMEM records pair them.
fn pair(value: u32) -> [u32; 2] {
    [value, !value]
}

impl KwiBoot {
    /// Validate `props` and build the device.
    ///
    /// # Errors
    ///
    /// [`Error::Config`] if a KWI was given and it is not one this can boot
    /// (see [`Kwi::parse`]).
    pub fn new(props: &Props) -> Result<KwiBoot> {
        let mut r = props.reader();
        let image = r.optional_media("image")?.map(Media::to_bytes);
        let machine_type: u32 = r.or("machine-type", DEFAULT_MACHINE_TYPE)?;
        let dram_end = r.or_addr("dram-end", DEFAULT_DRAM_END)?;
        let path = r.or_str("kwi-path", DEFAULT_KWI_PATH)?.to_string();
        let flash = r.optional_str("flash")?.map(ToString::to_string);
        r.finish()?;
        let new_stub = |code: Vec<u8>| {
            Arc::new(Stub {
                code,
                armed: AtomicBool::new(true),
                below: Mutex::with_rank(LockRank::DEVICE, None),
            })
        };
        let image = image.unwrap_or_else(|| Arc::from(&[][..]));
        if image.is_empty() {
            return Ok(KwiBoot {
                image,
                pieces: Vec::new(),
                stub: new_stub(Vec::new()),
                flash,
                model: None,
                binding: Mutex::with_rank(LockRank::DEVICE, Binding::default()),
            });
        }
        let kwi = Kwi::parse(&image)?;
        let pieces = plan(&kwi, dram_end, &path)?;
        let atags = u64::from(kwi.pmb.load_base + ATAG_OFFSET);
        let stub = stub(machine_type, atags as u32, kwi.pmb.kernel_load);
        Ok(KwiBoot {
            image,
            pieces,
            stub: new_stub(stub),
            flash,
            model: Some(kwi.model),
            binding: Mutex::with_rank(LockRank::DEVICE, Binding::default()),
        })
    }

    /// The model code of the KWI being booted, if one was given.
    #[must_use]
    pub fn model(&self) -> Option<&str> {
        self.model.as_deref()
    }

    /// What the last staging failed with, if it did.
    #[must_use]
    pub fn last_error(&self) -> Option<String> {
        self.binding.lock().error.clone()
    }

    /// How many times the image has been staged.
    #[must_use]
    pub fn loads(&self) -> u64 {
        self.binding.lock().loads
    }

    /// Write every piece into `space`.
    ///
    /// # Errors
    ///
    /// Whatever the space refuses, naming the piece and the address.
    pub fn stage(&self, space: &AddressSpace, requester: RequesterId) -> Result<()> {
        let attrs = MemAttrs::DEFAULT
            .with_requester(requester)
            .with_privileged(true);
        for piece in &self.pieces {
            let bytes: &[u8] = match &piece.source {
                Source::Kwi { offset, len } => {
                    &self.image[*offset as usize..(*offset + *len) as usize]
                }
                Source::Built(bytes) => bytes,
            };
            let mut at = piece.addr;
            for chunk in bytes.chunks(64 * 1024) {
                space
                    .write_bytes(at, chunk, attrs)
                    .map_err(|e| Error::Config {
                        at: CLASS_NAME.to_string(),
                        message: format!(
                            "cannot stage {} at {at:#x} ({} bytes from {:#x}): {e}",
                            piece.what,
                            bytes.len(),
                            piece.addr
                        ),
                    })?;
                at += chunk.len() as u64;
            }
        }
        Ok(())
    }
}

/// The reset stub: the three registers, then a jump.
///
/// ```text
///   0x00  mov r0, #0
///   0x04  ldr r1, [pc, #8]    ; 0x14: the machine type
///   0x08  ldr r2, [pc, #8]    ; 0x18: the ATAG list
///   0x0c  ldr pc, [pc, #8]    ; 0x1c: the kernel
/// ```
///
/// A32 encodings from the ARM ARM (DDI 0406C) A8.8.102 `MOV` (immediate)
/// and A8.8.64 `LDR` (literal); the PC reads eight bytes ahead.
fn stub(machine_type: u32, atags: u32, entry: u32) -> Vec<u8> {
    let mut rom = vec![0u8; STUB_LEN];
    let code = words(&[
        0xe3a0_0000,
        0xe59f_1008,
        0xe59f_2008,
        0xe59f_f008,
        0,
        machine_type,
        atags,
        entry,
    ]);
    rom[..code.len()].copy_from_slice(&code);
    rom
}

/// Everything a boot writes, in order.
fn plan(kwi: &Kwi, dram_end: u64, path: &str) -> Result<Vec<Piece>> {
    let pmb = &kwi.pmb;
    let member = |name: &str| -> Result<Source> {
        let m = kwi.member(name)?;
        Ok(Source::Kwi {
            offset: m.offset,
            len: m.len,
        })
    };
    let at = |offset: u32| u64::from(OSMEM + offset);
    let mut pieces = vec![
        Piece {
            addr: u64::from(pmb.rootfs_load),
            what: "the root filesystem",
            source: member("rootfs")?,
        },
        Piece {
            addr: u64::from(pmb.kernel_load),
            what: "the kernel",
            source: member("xipImage")?,
        },
        Piece {
            addr: u64::from(pmb.usrconf_addr),
            what: "USRCONF",
            source: member("USRCONF")?,
        },
    ];

    // -- the OSMEM records, in address order --------------------------------
    // Records 1-5, each a word and its complement. Record 4 (+0x28) says
    // where USRCONF is: the unit's chain leaves 1, and the kernel then reads
    // it from the NOR at 0x20000; 0 sends it to the copy the `CONF` record
    // below describes, which is the one this staged. Seen live: with 1 and no
    // flash, CONFMEM_init finds no `USRCONF ` header in NOR and the kernel
    // requests a reset.
    let mut header = b"OS M0017".to_vec();
    header.resize(0x10, 0);
    for v in [0, 1, 1, 0, 0] {
        header.extend_from_slice(&words(&pair(v)));
    }
    pieces.push(Piece {
        addr: at(0),
        what: "the OSMEM header",
        source: Source::Built(header),
    });
    let mut flags = words(&[0, 0]);
    for v in [1, 0] {
        flags.extend_from_slice(&words(&pair(v)));
    }
    pieces.push(Piece {
        addr: at(0x130),
        what: "an OSMEM flag record",
        source: Source::Built(flags),
    });
    let mut tag = vec![0xff, 0, 0, 0, 0, 0xff, 0xff, 0xff];
    tag.extend_from_slice(&pmb.tag);
    tag.extend(pmb.tag.iter().map(|b| !b));
    tag.resize(0x128, 0xff);
    tag.extend_from_slice(b"PMB_DATA");
    pieces.push(Piece {
        addr: at(0x150),
        what: "the OSMEM version record",
        source: Source::Built(tag),
    });
    let mut pairs = Vec::new();
    for _ in 0..6 {
        pairs.extend_from_slice(&words(&pair(0)));
    }
    pieces.push(Piece {
        addr: at(0x400),
        what: "an OSMEM flag record",
        source: Source::Built(pairs),
    });
    let path = path.replace("%ID%", &kwi.model);
    if path.len() > 24 {
        return Err(Error::Config {
            at: CLASS_NAME.to_string(),
            message: format!("kwi-path `{path}` is longer than the record's 24 bytes"),
        });
    }
    let mut field = path.into_bytes();
    field.resize(24, 0);
    let mut record = b"KWI ".to_vec();
    record.extend_from_slice(&word_sum(&field).to_le_bytes());
    record.extend_from_slice(&field);
    pieces.push(Piece {
        addr: at(0x460),
        what: "the OSMEM KWI record",
        source: Source::Built(record),
    });
    let body = [pmb.usrconf_sum, pmb.usrconf_size, pmb.usrconf_addr];
    let mut conf = b"CONF".to_vec();
    conf.extend_from_slice(&word_sum(&words(&body)).to_le_bytes());
    conf.extend_from_slice(&words(&body));
    conf.resize(0x14 + 0x100, 0xff);
    pieces.push(Piece {
        addr: at(0x4b0),
        what: "the OSMEM USRCONF record",
        source: Source::Built(conf),
    });
    let mut cmdline = pmb.cmdline.clone().into_bytes();
    cmdline.push(0);
    cmdline.resize(cmdline.len().next_multiple_of(4), 0);
    let len = 0x1c + cmdline.len() as u32;
    let mut desc = words(&[
        DESCRIPTOR_MAGIC,
        0,
        len,
        pmb.load_base,
        pmb.kernel_load,
        pmb.kernel_size,
        pmb.kernel_crc,
    ]);
    desc.extend_from_slice(&cmdline);
    let crc = crc32(&desc[8..]);
    desc[4..8].copy_from_slice(&crc.to_le_bytes());
    pieces.push(Piece {
        addr: at(0x1000),
        what: "the OSMEM kernel descriptor",
        source: Source::Built(desc),
    });
    let mut log = b"OS L".to_vec();
    log.extend_from_slice(&words(&[0, 1, 0, 0x5555_5555]));
    pieces.push(Piece {
        addr: at(0x2010),
        what: "the OSMEM log record",
        source: Source::Built(log),
    });
    pieces.push(Piece {
        addr: at(0x4030),
        what: "an OSMEM record",
        source: Source::Built(words(&[0xffff_0000, 0, 0, 0x120])),
    });
    let mut table = vec![1u8];
    table.resize(0x80, 0xff);
    pieces.push(Piece {
        addr: at(0xb040),
        what: "an OSMEM table",
        source: Source::Built(table),
    });
    pieces.push(Piece {
        addr: at(0xb140),
        what: "an OSMEM flag",
        source: Source::Built(words(&[1])),
    });
    pieces.push(Piece {
        addr: at(0x24e800),
        what: "the error-log header",
        source: Source::Built(words(&[0xfffe_0001, 0, 0x1234_5678, 0xedcb_a987])),
    });

    // -- the ATAG list ------------------------------------------------------
    let start = pmb.load_base;
    let size =
        u32::try_from(dram_end.saturating_sub(u64::from(start))).map_err(|_| Error::Config {
            at: CLASS_NAME.to_string(),
            message: format!("dram-end {dram_end:#x} is out of reach of a 32-bit ATAG"),
        })?;
    let mut atags = words(&[5, 0x5441_0001, 0, 0, 0]);
    atags.extend_from_slice(&words(&[4, 0x5441_0002, size, start]));
    let mut line = pmb.cmdline.clone().into_bytes();
    line.push(0);
    line.resize(line.len().next_multiple_of(4), 0);
    atags.extend_from_slice(&words(&[2 + line.len() as u32 / 4, 0x5441_0009]));
    atags.extend_from_slice(&line);
    atags.extend_from_slice(&words(&[0, 0]));
    pieces.push(Piece {
        addr: u64::from(start + ATAG_OFFSET),
        what: "the ATAG list",
        source: Source::Built(atags),
    });
    Ok(pieces)
}

/// The `navi.kwiboot` device class.
pub static CLASS: DeviceClass = DeviceClass {
    name: CLASS_NAME,
    version: 1,
    summary: "boots a navi's product system from its LOADING.KWI, with no flash image",
    properties: &[
        PropertySpec {
            name: "image",
            kind: ValueKind::Media,
            required: false,
            summary: "the LOADING.KWI, as the name of a media slot; an unbound slot boots the flash",
        },
        PropertySpec {
            name: "machine-type",
            kind: ValueKind::Uint,
            required: false,
            summary: "the ARM Linux machine type passed in r1 (default 3386, U-Boot's)",
        },
        PropertySpec {
            name: "dram-end",
            kind: ValueKind::Addr,
            required: false,
            summary: "where DRAM ends, for the ATAG memory record (default 0xa0000000)",
        },
        PropertySpec {
            name: "flash",
            kind: ValueKind::Str,
            required: false,
            summary: "the flash object the reset stub stands in front of, and hands back to",
        },
        PropertySpec {
            name: "kwi-path",
            kind: ValueKind::Str,
            required: false,
            summary: "the KWI's directory as osloader records it (default `/HD14/EXE/%ID%`)",
        },
    ],
    construct: |props| Ok(Box::new(KwiBoot::new(props)?)),
};

impl Device for KwiBoot {
    fn class(&self) -> &'static DeviceClass {
        &CLASS
    }

    fn realize(&self, _ctx: &mut RealizeCtx<'_>) -> Result<()> {
        // Nothing outward: everything goes in at reset, after the memory it
        // lands in has been cleared.
        Ok(())
    }

    fn reset(&self, _kind: ResetKind) {
        // Every reset, warm ones included: on the unit any reset goes back
        // through the loader chain, which stages the image afresh.
        let (space, requester) = {
            let binding = self.binding.lock();
            (binding.space.clone(), binding.requester)
        };
        let Some(space) = space else {
            return;
        };
        self.stub.armed.store(true, Ordering::Release);
        let result = self.stage(&space, requester);
        let mut binding = self.binding.lock();
        binding.loads += 1;
        binding.error = result.err().map(|e| e.to_string());
    }

    // No `save`/`load`: everything here is a function of the bound KWI.
}

impl Instance for KwiBoot {
    fn bind(&self, ctx: &BindCtx<'_>) -> Result<()> {
        if self.pieces.is_empty() {
            return Ok(());
        }
        let space = ctx.space().ok_or_else(|| Error::Config {
            at: ctx.path().to_string(),
            message: String::from("a loader needs an address space (`space = mem`)"),
        })?;
        // The stub goes over the reset vector, above whatever is there, and
        // passes accesses through to the flash once it has done its job.
        if let Some(flash) = &self.flash {
            let region = ctx.region(flash, "")?;
            let below = AddressSpace::new(format!("{CLASS_NAME}.flash"), 32);
            below.topology().map(region, 0)?;
            *self.stub.below.lock() = Some(Arc::new(below));
        }
        let region = Region::io(
            "kwiboot-stub",
            STUB_LEN as u64,
            Arc::clone(&self.stub) as Arc<dyn MemOps>,
        );
        space
            .topology()
            .map_with_priority(region, 0, 1)
            .map_err(|e| Error::Config {
                at: ctx.path().to_string(),
                message: format!("cannot map the reset stub at 0: {e}"),
            })?;
        // Fail the build rather than a reset that cannot report it.
        self.stage(space, ctx.requester())?;
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
    bindings.bind(CLASS_NAME, |props| Ok(Arc::new(KwiBoot::new(props)?)))
}

/// What the validator should know about `navi.kwiboot`.
#[must_use]
pub fn schema() -> crate::machine::validate::ClassSchema {
    use crate::machine::validate::{ClassSchema, PropSchema};
    let mut schema = ClassSchema::new(CLASS_NAME);
    for spec in CLASS.properties {
        schema = schema.prop(PropSchema::new(spec.name, spec.kind));
    }
    schema
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::props::Value;
    use crate::core::space::RamStore;
    use crate::core::value::Width;
    use crate::dev::navi::kwi::fake;

    fn boot(model: &[u8; 4]) -> KwiBoot {
        let mut props = Props::new();
        props.insert("image", Value::Media(Media::new("kwi", fake::build(model))));
        KwiBoot::new(&props).expect("the fake KWI boots")
    }

    fn space() -> Arc<AddressSpace> {
        let space = AddressSpace::new("mem", 32);
        space
            .topology()
            .map(
                Region::ram("dram", Arc::new(RamStore::new(0x40_0000))),
                0x6000_0000,
            )
            .expect("a fresh space");
        Arc::new(space)
    }

    fn bytes(space: &AddressSpace, addr: u64, len: usize) -> Vec<u8> {
        let mut out = vec![0u8; len];
        space
            .read_bytes(addr, &mut out, MemAttrs::DEBUG)
            .expect("mapped");
        out
    }

    fn word(space: &AddressSpace, addr: u64) -> u32 {
        space
            .read(addr, Width::U32, MemAttrs::DEBUG)
            .expect("mapped") as u32
    }

    #[test]
    fn with_no_kwi_it_does_nothing() {
        let device = KwiBoot::new(&Props::new()).expect("an empty slot is fine");
        assert!(device.pieces.is_empty());
        assert_eq!(device.model(), None);
    }

    #[test]
    fn the_members_land_where_the_descriptor_says() {
        let device = boot(b"HC60");
        let space = space();
        device
            .stage(&space, RequesterId::ANONYMOUS)
            .expect("fits in 4 MiB");
        let (kernel, rootfs, usrconf) = fake::members();
        assert_eq!(
            bytes(&space, fake::KERNEL_LOAD.into(), kernel.len()),
            kernel
        );
        assert_eq!(
            bytes(&space, fake::ROOTFS_LOAD.into(), rootfs.len()),
            rootfs
        );
        assert_eq!(
            bytes(&space, fake::USRCONF_ADDR.into(), usrconf.len()),
            usrconf
        );
    }

    #[test]
    fn the_osmem_records_carry_the_images_own_values() {
        let device = boot(b"HC60");
        let space = space();
        device.stage(&space, RequesterId::ANONYMOUS).unwrap();
        let (_, _, usrconf) = fake::members();
        assert_eq!(bytes(&space, 0x6000_0000, 8), b"OS M0017");
        // Record 4: USRCONF is in RAM, not in a flash this machine lacks.
        assert_eq!(word(&space, 0x6000_0028), 0);
        assert_eq!(word(&space, 0x6000_002c), !0);
        assert_eq!(bytes(&space, 0x6000_0158, 8), b"VC0000KA");
        // The CONF record: magic, the sum of the next three words, then them.
        assert_eq!(bytes(&space, 0x6000_04b0, 4), b"CONF");
        let sum = word_sum(&usrconf);
        let len = usrconf.len() as u32;
        assert_eq!(word(&space, 0x6000_04b8), sum);
        assert_eq!(word(&space, 0x6000_04bc), len);
        assert_eq!(word(&space, 0x6000_04c0), fake::USRCONF_ADDR);
        assert_eq!(
            word(&space, 0x6000_04b4),
            sum.wrapping_add(len).wrapping_add(fake::USRCONF_ADDR)
        );
        // The KWI record names the model's directory.
        assert_eq!(bytes(&space, 0x6000_0468, 14), b"/HD14/EXE/HC60");
        // The kernel descriptor U-Boot reads, with its CRC over bytes 8 on.
        assert_eq!(word(&space, 0x6000_1000), DESCRIPTOR_MAGIC);
        let len = word(&space, 0x6000_1008) as usize;
        let desc = bytes(&space, 0x6000_1000, len);
        assert_eq!(word(&space, 0x6000_1004), crc32(&desc[8..]));
        assert_eq!(word(&space, 0x6000_1010), fake::KERNEL_LOAD);
    }

    #[test]
    fn the_atags_describe_memory_and_carry_the_command_line() {
        let device = boot(b"HC59");
        let space = space();
        device.stage(&space, RequesterId::ANONYMOUS).unwrap();
        let at = u64::from(fake::LOAD_BASE + ATAG_OFFSET);
        assert_eq!(word(&space, at + 4), 0x5441_0001, "ATAG_CORE first");
        assert_eq!(word(&space, at + 0x18), 0x5441_0002, "then ATAG_MEM");
        assert_eq!(
            u64::from(word(&space, at + 0x1c)),
            DEFAULT_DRAM_END - u64::from(fake::LOAD_BASE)
        );
        assert_eq!(word(&space, at + 0x20), fake::LOAD_BASE);
        assert_eq!(word(&space, at + 0x28), 0x5441_0009, "then ATAG_CMDLINE");
        assert_eq!(
            bytes(&space, at + 0x2c, fake::CMDLINE.len()),
            fake::CMDLINE.as_bytes()
        );
    }

    #[test]
    fn the_stub_enters_the_kernel_with_the_linux_registers() {
        let device = boot(b"HC59");
        let w =
            |i: usize| u32::from_le_bytes(device.stub.code[i * 4..i * 4 + 4].try_into().unwrap());
        assert_eq!(w(0), 0xe3a0_0000, "mov r0, #0");
        assert_eq!(w(5), DEFAULT_MACHINE_TYPE);
        assert_eq!(w(6), fake::LOAD_BASE + ATAG_OFFSET);
        assert_eq!(w(7), fake::KERNEL_LOAD);
    }

    #[test]
    fn a_reset_stages_it_again() {
        let device = boot(b"HC59");
        let space = space();
        device.binding.lock().space = Some(Arc::clone(&space));
        device.reset(ResetKind::Warm);
        assert_eq!(device.loads(), 1);
        assert_eq!(device.last_error(), None);
        assert_eq!(bytes(&space, 0x6000_0000, 8), b"OS M0017");
    }

    #[test]
    fn the_stub_steps_aside_once_the_entry_is_fetched() {
        let device = boot(b"HC59");
        let stub = &device.stub;
        let mut word = [0u8; 4];
        // A debugger looking at the entry word leaves it armed.
        stub.read(STUB_ENTRY, &mut word, MemAttrs::DEBUG).unwrap();
        assert!(stub.armed.load(Ordering::Relaxed));
        stub.read(0, &mut word, MemAttrs::DEFAULT).unwrap();
        assert_eq!(u32::from_le_bytes(word), 0xe3a0_0000);
        stub.read(STUB_ENTRY, &mut word, MemAttrs::DEFAULT).unwrap();
        assert_eq!(u32::from_le_bytes(word), fake::KERNEL_LOAD);
        assert!(!stub.armed.load(Ordering::Relaxed));
        // With no flash behind it, the page is now unassigned.
        assert!(stub.read(0, &mut word, MemAttrs::DEFAULT).is_err());
        // Something behind it answers once one is attached.
        let flash = AddressSpace::new("flash", 32);
        flash
            .topology()
            .map(Region::ram("nor", Arc::new(RamStore::new(0x1000))), 0)
            .unwrap();
        flash
            .write(0, Width::U32, 0x1234_5678, MemAttrs::DEFAULT)
            .unwrap();
        *stub.below.lock() = Some(Arc::new(flash));
        stub.read(0, &mut word, MemAttrs::DEFAULT).unwrap();
        assert_eq!(u32::from_le_bytes(word), 0x1234_5678);
    }
}
