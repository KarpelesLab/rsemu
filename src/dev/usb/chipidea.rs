//! The ChipIdea/ARC dual-role USB controller: an EHCI core, moved.
//!
//! # This file contains no schedule walker, and that is the claim
//!
//! Everything that makes a USB host controller a host controller — the queue
//! head and transfer descriptor formats, the DMA walk, the packet loop, the
//! interrupt logic, the port state machine — is [`crate::dev::usb::ehci::Hcd`],
//! because all of that is EHCI's and this block *is* an EHCI. What is the
//! vendor's, and therefore what is here, is:
//!
//! * a **register map**: the capability registers at `+0x100` with `CAPLENGTH`
//!   reading `0x40`, so the operational registers sit at **`+0x140`**;
//! * an **`ID` register** below them, with the core's own identification;
//! * a **`USBMODE`** role select, because the block is dual-role and can be a
//!   *device* rather than a host;
//! * the device-controller capability registers, and a handful of tuning
//!   registers (`BURSTSIZE`, `TXFILLTUNING`, `TTCTRL`, `ULPI_VIEWPORT`).
//!
//! That is the whole difference, and it is why the split is worth having: the
//! next SoC whose USB block is a licensed EHCI core — and there are a great
//! many — is another file this length, not another controller.
//!
//! ```text
//!   +0x000  ID, HWGENERAL, HWHOST, HWDEVICE, HWTXBUF, HWRXBUF   ← this file
//!   +0x100  CAPLENGTH = 0x40, HCIVERSION, HCSPARAMS, HCCPARAMS  ← ehci::Hcd
//!   +0x120  DCIVERSION, DCCPARAMS                               ← this file
//!   +0x140  USBCMD, USBSTS, USBINTR, FRINDEX, …                 ← ehci::Hcd
//!   +0x180  CONFIGFLAG, PORTSC1 …                               ← ehci::Hcd
//!   +0x1a4  OTGSC, USBMODE                                      ← ehci::Hcd state,
//!   +0x1ac  ENDPTSETUPSTAT and the device-mode block                this file's map
//! ```
//!
//! # Addresses are machine-file properties, never constants
//!
//! The immediate part is a Conexant DigiColor **CX92755**, whose USB block sits
//! at `0xf00bc000` with a wrapper and PHY aperture at `0xf0084000`. Neither
//! address appears in this file, and neither should: this is one SoC's
//! placement of a reusable core, and `machines/arm926.machine` — the
//! DigiColor-facing board — already parameterises its peripheral aperture for
//! exactly this reason. A `map` statement puts the register block where the
//! board puts it.
//!
//! # Dual role: what is modelled and what is left
//!
//! `USBMODE.CM` selects **idle**, **device** or **host**, and it is write-once
//! after a reset on the real part. Host mode is complete — it is
//! [`crate::dev::usb::ehci`]. Device mode means the *guest* is the peripheral,
//! and what is modelled of it is exactly this much:
//!
//! * the role select itself, so firmware that writes `USBMODE` before
//!   `USBCMD.HCRESET` — which is the order the block requires — behaves;
//! * the host schedule **stops** while the role is anything but host, so a
//!   controller put into device mode does not quietly keep walking a host
//!   schedule that no longer exists;
//! * the device-controller capability registers, so a driver can discover how
//!   many endpoints there would be;
//! * `PERIODICLISTBASE` and `ASYNCLISTADDR` doubling as `DEVICEADDR` and
//!   `ENDPOINTLISTADDR`, which is what those two registers *are* in device
//!   mode — the same storage, a different name.
//!
//! **What is left**: the device-side queue-head list is never walked, and
//! `ENDPTSETUPSTAT`, `ENDPTPRIME`, `ENDPTFLUSH`, `ENDPTSTAT`, `ENDPTCOMPLETE`
//! and `ENDPTCTRLn` read as zero and discard writes. A guest that configures
//! itself as a USB peripheral will therefore configure, and then never see a
//! transfer. That is a large piece of work — it is a whole second controller,
//! facing the other way — and pretending otherwise by half-implementing the
//! endpoint registers would produce a device that looks like it works.
//!
//! # The firmware's initialisation handshake
//!
//! `docs/buses/usb.md` records the `EHCI_Host_Reset` → `EHCI_Init` flow the
//! DigiColor firmware performs. Steps 1 to 4 are this file's; 5 to 7 are the
//! generic controller's, and are where they belong.
//!
//! | Step | What the firmware does | What answers it |
//! | --- | --- | --- |
//! | 1 | poll `USBSTS.HCHalted` until halted | `HCHalted` is set out of reset, and thereafter is exactly `!RunStop` |
//! | 2 | assert `USBCMD.HCReset`, poll until it **self-clears** | the write never reaches `USBCMD`; it runs a reset and the register reads back with the bit clear |
//! | 3 | select host mode through `USBMODE`, then **read it back** | `CM` is write-once *after a reset*, and step 2's reset is what re-armed it, so the write takes and the read-back answers |
//! | 4 | read `ID` and check `(ID & 0xFFFF) == 0xFA05` | the `id` property, defaulting to [`CX92755_ID`] |
//!
//! Step 3 is the one worth dwelling on. `CM` being write-once means a model
//! has to decide what a controller reset does to it, and the two readings are
//! not equivalent: **re-arming** is what makes the documented order work, and
//! it is also what makes a *role switch* work — this firmware is a host for
//! mass storage and PictBridge and a device for the printer it presents, so it
//! changes role. Carrying the old role across `HCReset` would make step 3's
//! read-back return the previous mode and hang the firmware on its own check.
//! [`Hcd::reset`](super::ehci::Hcd::reset) therefore re-arms it, and
//! `a_reset_re_arms_the_role_select_so_a_switch_works` is the test that keeps
//! it that way.
//!
//! Steps 1 and 2 are the ones a stub already satisfied — the note in
//! `docs/buses/usb.md` records that self-clearing `HCReset` and reporting
//! `HCHalted = !RunStop` was enough to get the real firmware past the spin it
//! was stuck on. **That is the trap**: this firmware treats USB host as
//! optional and degrades rather than failing, so those two bits alone look
//! like a clean boot. Step 4 succeeding is the first thing that changes its
//! behaviour, and the first thing that can regress it.
//!
//! # Sources, and which one wins
//!
//! Two sources, and they are not equal:
//!
//! * **First-party reverse engineering of the CX92755-class firmware**, in
//!   `docs/buses/usb.md`. No public datasheet for this block is available, so
//!   for this part those notes *are* the specification. They fix the `+0x140`
//!   operational offset, the `ID` magic and the check that reads it, the four
//!   registers the reset handshake touches (`USBCMD` `+0x140`, `USBSTS`
//!   `+0x144`, `USBINTR` `+0x148`, `USBMODE` `+0x1a8`), the seven-step flow,
//!   and both roles the firmware uses.
//! * **The published ChipIdea/ARC USB-HS core layout**, from the USB chapters
//!   of freely available SoC reference manuals for parts that license it. This
//!   is the general core, and it is where everything the reverse engineering
//!   did not need comes from: the capability block at `+0x100` with
//!   `CAPLENGTH` reading `0x40`, `DCIVERSION`/`DCCPARAMS` at `+0x120`/`+0x124`,
//!   `OTGSC` at `+0x1a4`, `BURSTSIZE`/`TXFILLTUNING`/`TTCTRL`/`ULPI_VIEWPORT`,
//!   the `HWxxx` block below `+0x100`, and the `ID`/`NID`/`REVISION` field
//!   format.
//!
//! **Where the two disagree, the part wins**, because the part is what this
//! board has. They do not currently disagree: every offset the reverse
//! engineering fixes is exactly where the published layout puts it, which is
//! itself worth recording — it is evidence that this really is a stock core
//! and that the rest of the published map is a reasonable thing to build on.
//!
//! Two things are inferred rather than confirmed, and are flagged here so that
//! a future contradiction is recognised as one rather than absorbed:
//!
//! * **`CAPLENGTH = 0x40` and the capability block at `+0x100`.** The reverse
//!   engineering fixes `+0x140` for the operational registers but does not say
//!   how the block gets there, and the firmware's flow never reads
//!   `CAPLENGTH` — it evidently hard-codes the offset, as firmware for a known
//!   part does. `0x100 + 0x40` is the published arrangement and reproduces the
//!   one number that *is* confirmed, so it is what this file implements.
//! * **`REVISION` in `ID` bits 23:16.** The firmware masks `ID` with `0xFFFF`,
//!   so the upper half is unconstrained; the `id` property is therefore the
//!   whole 32-bit register value and defaults to `0xfa05` exactly, with the
//!   revision reading zero. A board that knows its silicon revision puts it in
//!   the property.
//!
//! The `0xfa05` value is self-consistent with the published field format,
//! which is a useful cross-check on a magic number: it is `ID = 5` in bits 5:0
//! with `NID = 0x3a` in bits 13:8, and `0x3a` is the six-bit one's complement
//! of `5`. `the_identification_register_is_the_cx92755s` asserts that
//! relationship rather than only the constant.
//!
//! # What is *not* in the recorded flow, and matters
//!
//! The reverse-engineered flow covers reset, role selection, detection,
//! allocation, schedule construction and interrupt enable. It does not mention
//! `CONFIGFLAG` or `PORTSC`. EHCI 1.0 §4.2 leaves every root port owned by a
//! companion controller until `CONFIGFLAG` is written, and this model obeys
//! that — so if the firmware genuinely never writes it, nothing will enumerate
//! and the cause will be a port that was never claimed rather than anything in
//! the schedule walker. That is the first thing to check when this block is
//! wired to a real image, and it is not something to pre-emptively paper over
//! by defaulting `CONFIGFLAG` to one.

use alloc::boxed::Box;
use alloc::string::{String, ToString};
use alloc::sync::Arc;
use core::fmt;

use super::ehci::{Extra, Hcd, PORT_RESET, Params, narrow_read, word_write};
use crate::bus::usb::{MAX_PORTS, UsbBus, buses};
use crate::core::device::{Device, DeviceClass, PropertySpec, RealizeCtx, ResetKind};
use crate::core::error::{BusError, Error, Result};
use crate::core::props::{Props, ValueKind};
use crate::core::sched::LazyHandle;
use crate::core::space::{AccessConstraints, MemAttrs, MemOps, MemResult, Region, RegionRef};
use crate::core::state::{ChunkReader, ChunkWriter, Sink, Source};
use crate::core::sync::{LockRank, Mutex};
use crate::core::value::Width;
use crate::core::wire::WireSource;
use crate::machine::realize::{BindCtx, Instance};

/// The class name a machine description writes.
const CLASS_NAME: &str = "usb.chipidea";

/// The snapshot chunk version. Bump with the encoding, never on its own.
const STATE_VERSION: u32 = 1;

/// How much address space the register block occupies.
///
/// 512 bytes: the device-mode endpoint control registers run to `0x1ff` on a
/// part with eight endpoint pairs, and nothing is defined above that.
pub const REGISTER_BYTES: u64 = 0x200;

/// Where the EHCI capability registers start inside the block.
pub const CAPABILITY_BASE: u64 = 0x100;

/// `CAPLENGTH`, and therefore the offset from [`CAPABILITY_BASE`] to the
/// operational registers.
///
/// `0x100 + 0x40` is `0x140`, which is the number that distinguishes this block
/// from a bare EHCI more than any other.
pub const CAPLENGTH: u8 = 0x40;

/// Where the EHCI operational registers start inside the block: **`+0x140`**.
pub const OPERATIONAL_BASE: u64 = CAPABILITY_BASE + CAPLENGTH as u64;

/// The `ID` value a Conexant DigiColor CX92755 reports.
///
/// `ID = 5` in bits 5:0, `NID = 0x3a` in bits 13:8 — the six-bit one's
/// complement of `5`, which is what the field is defined to be — and the
/// reserved bits 15:14 reading one.
pub const CX92755_ID: u32 = 0xfa05;

/// `DCIVERSION`: the device-controller interface revision (a halfword at
/// `+0x120`).
const DCIVERSION: u32 = 0x0001;

/// How many device-mode endpoint pairs the block reports in `DCCPARAMS`.
const DEVICE_ENDPOINTS: u32 = 8;

// The offsets this file owns, relative to the block base.
/// Identification.
const REG_ID: u64 = 0x000;
/// General hardware parameters.
const REG_HWGENERAL: u64 = 0x004;
/// Host-mode hardware parameters.
const REG_HWHOST: u64 = 0x008;
/// Device-mode hardware parameters.
const REG_HWDEVICE: u64 = 0x00c;
/// Transmit buffer parameters.
const REG_HWTXBUF: u64 = 0x010;
/// Receive buffer parameters.
const REG_HWRXBUF: u64 = 0x014;
/// Device-controller interface version.
const REG_DCIVERSION: u64 = 0x120;
/// Device-controller capability parameters.
const REG_DCCPARAMS: u64 = 0x124;
/// Transaction-translator control. There is no TT here; see the module docs.
const REG_TTCTRL: u64 = 0x15c;
/// AHB burst size.
const REG_BURSTSIZE: u64 = 0x160;
/// Transmit FIFO fill tuning.
const REG_TXFILLTUNING: u64 = 0x164;
/// The ULPI PHY access window.
const REG_ULPI_VIEWPORT: u64 = 0x170;
/// On-the-go status and control.
const REG_OTGSC: u64 = 0x1a4;
/// The controller role select.
const REG_USBMODE: u64 = 0x1a8;
/// Where the device-mode endpoint registers start.
const REG_DEVICE_BLOCK: u64 = 0x1ac;

/// `ULPI_VIEWPORT` bit 30: a PHY access is in progress.
///
/// It always reads zero here: an access that completes before the write
/// returns is the honest degenerate model of a PHY that is not simulated, and
/// it is what makes firmware's "poll until `RUN` clears" loop terminate rather
/// than hang. A `RUN` bit that stuck at one would be worse than no register at
/// all.
const ULPI_RUN: u32 = 1 << 30;

/// The registers this file keeps that the EHCI engine has no notion of.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
struct VariantRegs {
    ttctrl: u32,
    ulpi: u32,
}

/// The ChipIdea/ARC USB controller as a machine object.
#[derive(Debug)]
pub struct ChipIdea {
    hcd: Arc<Hcd>,
    id: u32,
    regs: Arc<Mutex<VariantRegs>>,
    region: RegionRef,
}

impl ChipIdea {
    /// Validate `props` and build the controller.
    ///
    /// Properties:
    ///
    /// * `bus` — the named [`UsbBus`] this controller is the root of.
    ///   Required.
    /// * `ports` — how many root ports, 1 to 15. Defaults to 1, which is what
    ///   the CX92755's block has.
    /// * `microframe` — clock-domain ticks in one 125 µs microframe. Defaults
    ///   to 7500, exact at the 60 MHz a USB 2.0 PHY runs at.
    /// * `id` — what the `ID` register reads. Defaults to [`CX92755_ID`],
    ///   because that is the part this was written for; another SoC's core
    ///   reports its own, which is why this is a property.
    ///
    /// # Errors
    ///
    /// [`Error::Property`] for an unknown or missing property,
    /// [`Error::Config`] for a value outside its range.
    pub fn new(props: &Props) -> Result<ChipIdea> {
        let mut r = props.reader();
        let bus_name = r.require_str("bus")?.to_string();
        let ports = r.or_range("ports", 1u64, 1..=MAX_PORTS as u64)?;
        let microframe = r.or_range("microframe", 7500u64, 1..=u64::from(u32::MAX))?;
        let id = r.or_range("id", u64::from(CX92755_ID), 0..=u64::from(u32::MAX))?;
        r.finish()?;

        let bus = buses::attach(props, &bus_name, ports as u8)?;
        if bus.port_count() < ports as u8 {
            return Err(Error::Config {
                at: String::from(CLASS_NAME),
                message: alloc::format!(
                    "the USB bus `{bus_name}` already has {} ports and this controller asked for \
                     {ports}; the first object to name a bus fixes its size",
                    bus.port_count()
                ),
            });
        }
        Ok(ChipIdea::with_bus(
            bus,
            Params {
                ports: ports as u8,
                microframe_ticks: microframe,
                caplength: CAPLENGTH,
                dual_role: true,
            },
            id as u32,
        ))
    }

    /// A controller on a bus the caller already holds.
    #[must_use]
    pub fn with_bus(bus: Arc<UsbBus>, params: Params, id: u32) -> ChipIdea {
        let params = Params {
            caplength: CAPLENGTH,
            dual_role: true,
            ..params
        };
        let hcd = Arc::new(Hcd::new(bus, params));
        let regs = Arc::new(Mutex::with_rank(LockRank::DEVICE, VariantRegs::default()));
        let port = Arc::new(ChipIdeaPort {
            hcd: Arc::clone(&hcd),
            regs: Arc::clone(&regs),
            id,
        });
        let region = Arc::new(Region::io(
            "chipidea",
            REGISTER_BYTES,
            port as Arc<dyn MemOps>,
        ));
        ChipIdea {
            hcd,
            id,
            regs,
            region,
        }
    }

    /// The EHCI engine underneath. **The whole controller is in here** — see
    /// the module docs.
    #[must_use]
    pub fn hcd(&self) -> &Arc<Hcd> {
        &self.hcd
    }

    /// What the `ID` register reads.
    #[must_use]
    pub fn id(&self) -> u32 {
        self.id
    }
}

/// The pin names a machine description wires.
pub mod pin {
    /// The interrupt output. One line for both roles, as the block has.
    pub const IRQ: &str = "irq";
}

/// The ChipIdea register block, as something an address space dispatches to.
struct ChipIdeaPort {
    hcd: Arc<Hcd>,
    regs: Arc<Mutex<VariantRegs>>,
    id: u32,
}

impl fmt::Debug for ChipIdeaPort {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("ChipIdeaPort")
            .field("id", &self.id)
            .finish_non_exhaustive()
    }
}

impl ChipIdeaPort {
    /// The identification and hardware-parameter block below `+0x100`.
    ///
    /// Read-only, and none of it depends on any state, so it is answerable
    /// without catching the schedule up.
    fn read_hw(&self, offset: u64) -> u32 {
        let ports = u32::from(self.hcd.params().ports);
        match offset {
            REG_ID => self.id,
            // The PHY interface width, the clock source and the serial-engine
            // options live here on a real part. None of them changes anything
            // this model does, and inventing plausible values would be worse
            // than reading zero: a driver that keyed off one would be relying
            // on a number nobody checked.
            REG_HWGENERAL => 0,
            // Host capable, with `NPORT` in bits 3:1 as a count minus one.
            REG_HWHOST => 1 | ((ports - 1) << 1),
            // Device capable, with the endpoint count in bits 5:1.
            REG_HWDEVICE => 1 | (DEVICE_ENDPOINTS << 1),
            // FIFO geometry. Zero for the same reason `HWGENERAL` is.
            REG_HWTXBUF | REG_HWRXBUF => 0,
            _ => 0,
        }
    }

    /// The capability window at `+0x100`.
    fn read_cap(&self, offset: u64) -> u32 {
        match offset {
            REG_DCIVERSION => DCIVERSION,
            // `DEN` in bits 4:0, `DC` (device capable) bit 7, `HC` (host
            // capable) bit 8.
            REG_DCCPARAMS => DEVICE_ENDPOINTS | (1 << 7) | (1 << 8),
            // Everything else in this window is EHCI's, at its own offset.
            _ => self.hcd.read_cap(offset - CAPABILITY_BASE),
        }
    }

    /// The operational window at `+0x140`: EHCI's registers, plus the block's
    /// own.
    fn read_op(&self, offset: u64) -> u32 {
        match offset {
            REG_TTCTRL => self.regs.lock().ttctrl,
            REG_BURSTSIZE => self.hcd.read_extra(Extra::BurstSize),
            REG_TXFILLTUNING => self.hcd.read_extra(Extra::TxFillTuning),
            // The `RUN` bit always reads clear: see [`ULPI_RUN`].
            REG_ULPI_VIEWPORT => self.regs.lock().ulpi & !ULPI_RUN,
            REG_OTGSC => self.hcd.read_extra(Extra::Otgsc),
            REG_USBMODE => self.hcd.read_extra(Extra::UsbMode),
            // The device-mode endpoint registers. Zero, and the module docs
            // say why rather than the value pretending to be one.
            _ if offset >= REG_DEVICE_BLOCK => 0,
            _ => self.hcd.read_op(offset - OPERATIONAL_BASE),
        }
    }
}

impl MemOps for ChipIdeaPort {
    fn read(&self, offset: u64, dst: &mut [u8], attrs: MemAttrs) -> MemResult {
        self.hcd.sync_for(attrs);
        let aligned = offset & !0x3;
        let value = if aligned < CAPABILITY_BASE {
            self.read_hw(aligned)
        } else if aligned < OPERATIONAL_BASE {
            self.read_cap(aligned)
        } else {
            self.read_op(aligned)
        };
        narrow_read(offset, value, dst)
    }

    fn write(&self, offset: u64, src: &[u8], attrs: MemAttrs) -> MemResult {
        if attrs.debug {
            // `USBSTS` is write-1-to-clear and `USBCMD` starts the controller;
            // neither has a harmless version (`ROADMAP.md` §15, invariant 5).
            return Err(BusError::BadAccess);
        }
        let Some(value) = word_write(src) else {
            return Err(BusError::BadAccess);
        };
        // Below the operational window everything is read-only capability and
        // identification.
        if offset < OPERATIONAL_BASE {
            return Ok(());
        }
        self.hcd.sync_for(attrs);
        match offset {
            REG_TTCTRL => {
                // There is no transaction translator behind this register —
                // there is no hub to have one — so it stores and reads back,
                // which is what firmware writing a hub address expects to see.
                self.regs.lock().ttctrl = value & 0x7f00_0000;
                return Ok(());
            }
            REG_BURSTSIZE => {
                let after = self.hcd.write_extra(Extra::BurstSize, value);
                self.hcd.act(after);
                return Ok(());
            }
            REG_TXFILLTUNING => {
                let after = self.hcd.write_extra(Extra::TxFillTuning, value);
                self.hcd.act(after);
                return Ok(());
            }
            REG_ULPI_VIEWPORT => {
                // The access is complete by the time the write returns.
                self.regs.lock().ulpi = value & !ULPI_RUN;
                return Ok(());
            }
            REG_OTGSC => {
                let after = self.hcd.write_extra(Extra::Otgsc, value);
                self.hcd.act(after);
                return Ok(());
            }
            REG_USBMODE => {
                let after = self.hcd.write_extra(Extra::UsbMode, value);
                self.hcd.act(after);
                return Ok(());
            }
            _ if offset >= REG_DEVICE_BLOCK => return Ok(()),
            _ => {}
        }

        let op = offset - OPERATIONAL_BASE;
        // Software releasing `PORT_RESET` is what drives the reset, exactly as
        // in the generic controller — the offset is different, the semantics
        // are not.
        let resetting = Hcd::port_at(op)
            .map(|port| (port, self.hcd.portsc(port)))
            .filter(|(_, sc)| sc & PORT_RESET != 0);
        let after = self.hcd.write_op(op, value);
        self.hcd.act(after);
        if let Some((port, _)) = resetting
            && self.hcd.portsc(port) & PORT_RESET == 0
        {
            self.hcd.finish_reset(port);
            self.hcd.refresh_irq();
        }
        Ok(())
    }

    fn constraints(&self) -> AccessConstraints {
        AccessConstraints::IO
            .with_widths(Width::U8, Width::U32)
            .with_natural_alignment(true)
    }
}

impl Device for ChipIdea {
    fn class(&self) -> &'static DeviceClass {
        &CHIPIDEA_CLASS
    }

    fn realize(&self, _ctx: &mut RealizeCtx<'_>) -> Result<()> {
        Ok(())
    }

    fn reset(&self, kind: ResetKind) {
        *self.regs.lock() = VariantRegs::default();
        self.hcd.reset(kind);
    }

    fn save(&self, w: &mut ChunkWriter<'_>) -> Result<()> {
        self.hcd.save(w)?;
        let regs = *self.regs.lock();
        w.write_u32(regs.ttctrl)?;
        w.write_u32(regs.ulpi)
    }

    fn load(&self, r: &mut ChunkReader<'_>) -> Result<()> {
        self.hcd.load(r)?;
        let regs = VariantRegs {
            ttctrl: r.read_u32()?,
            ulpi: r.read_u32()?,
        };
        *self.regs.lock() = regs;
        Ok(())
    }

    fn region(&self, name: &str) -> Option<RegionRef> {
        matches!(name, "" | "regs").then(|| Arc::clone(&self.region))
    }

    fn connect(&self, port: &str, source: WireSource) -> Result<()> {
        if port != pin::IRQ {
            return Err(Error::Config {
                at: String::from(port),
                message: alloc::format!(
                    "a ChipIdea USB controller drives `{}` and nothing else",
                    pin::IRQ
                ),
            });
        }
        self.hcd.connect_irq(source);
        Ok(())
    }

    fn announce(&self, _port: &str) {
        self.hcd.refresh_irq();
    }

    fn is_lazy(&self) -> bool {
        true
    }

    fn current_tick(&self) -> u64 {
        self.hcd.ticks()
    }

    fn advance_to(&self, tick: u64) {
        self.hcd.advance_to(tick);
    }

    fn next_event_tick(&self) -> Option<u64> {
        self.hcd.next_event_tick()
    }

    fn attach_lazy(&self, handle: LazyHandle) {
        self.hcd.attach_lazy(handle);
    }
}

impl Instance for ChipIdea {
    fn bind(&self, ctx: &BindCtx<'_>) -> Result<()> {
        let space = ctx.space().ok_or_else(|| Error::Config {
            at: ctx.path().to_string(),
            message: String::from(
                "a ChipIdea USB controller masters the bus its queue heads live on: add \
                 `space = mem` to the object",
            ),
        })?;
        self.hcd.attach_space(space, ctx.requester());
        Ok(())
    }
}

/// The `usb.chipidea` device class.
pub static CHIPIDEA_CLASS: DeviceClass = DeviceClass {
    name: CLASS_NAME,
    version: STATE_VERSION,
    summary: "the ChipIdea/ARC dual-role USB controller: an EHCI core with its operational \
              registers at +0x140, an ID register and a USBMODE role select",
    properties: &[
        PropertySpec {
            name: "bus",
            kind: ValueKind::Str,
            required: true,
            summary: "the named USB bus this controller is the root of",
        },
        PropertySpec {
            name: "ports",
            kind: ValueKind::Uint,
            required: false,
            summary: "how many root ports, 1 to 15 (default 1, which is the CX92755's)",
        },
        PropertySpec {
            name: "microframe",
            kind: ValueKind::Uint,
            required: false,
            summary: "clock-domain ticks in one 125 us microframe (default 7500, exact at 60 MHz)",
        },
        PropertySpec {
            name: "id",
            kind: ValueKind::Uint,
            required: false,
            summary: "what the ID register reads (default 0xfa05, the CX92755's)",
        },
    ],
    construct: |props| Ok(Box::new(ChipIdea::new(props)?)),
};

/// Add [`CHIPIDEA_CLASS`] to a registry.
///
/// # Errors
///
/// [`Error::Config`] if something already claimed the name.
pub fn register(registry: &mut crate::core::Registry) -> Result<()> {
    registry.add(&CHIPIDEA_CLASS)
}

/// Bind [`CHIPIDEA_CLASS`] into the machine graph.
///
/// # Errors
///
/// [`Error::Config`] if the class is already bound.
pub fn bind(bindings: &mut crate::machine::Bindings) -> Result<()> {
    bindings.bind(CLASS_NAME, |props| Ok(Arc::new(ChipIdea::new(props)?)))
}

/// What the validator should know about `usb.chipidea`.
#[must_use]
pub fn schema() -> crate::machine::validate::ClassSchema {
    use crate::machine::validate::{ClassSchema, PortDir, PropSchema};
    ClassSchema::new(CLASS_NAME)
        .prop(PropSchema::new("bus", ValueKind::Str).required())
        .prop(PropSchema::new("ports", ValueKind::Uint).range(1, MAX_PORTS as u64))
        .prop(PropSchema::new("microframe", ValueKind::Uint).range(1, u64::from(u32::MAX)))
        .prop(PropSchema::new("id", ValueKind::Uint).range(0, u64::from(u32::MAX)))
        .port(pin::IRQ, PortDir::Out)
        .region("")
        .region("regs")
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::bus::usb::UsbBus;

    fn build() -> ChipIdea {
        ChipIdea::with_bus(
            Arc::new(UsbBus::new(1)),
            Params {
                ports: 1,
                microframe_ticks: 7500,
                caplength: CAPLENGTH,
                dual_role: true,
            },
            CX92755_ID,
        )
    }

    /// The `MemOps` behind the register block.
    fn ops(device: &ChipIdea) -> Arc<dyn MemOps> {
        let region = device.region("").expect("the register block");
        match region.kind() {
            crate::core::space::RegionKind::Io(ops) => Arc::clone(ops),
            other => panic!("expected an io region, got {other:?}"),
        }
    }

    fn read32(device: &ChipIdea, offset: u64) -> u32 {
        let mut bytes = [0u8; 4];
        ops(device)
            .read(offset, &mut bytes, MemAttrs::DEFAULT)
            .expect("a register read");
        u32::from_le_bytes(bytes)
    }

    fn write32(device: &ChipIdea, offset: u64, value: u32) {
        ops(device)
            .write(offset, &value.to_le_bytes(), MemAttrs::DEFAULT)
            .expect("a register write");
    }

    /// The one number that distinguishes this block from a bare EHCI.
    #[test]
    fn the_operational_registers_are_at_0x140() {
        let device = build();
        // `CAPLENGTH` is a byte at +0x100, and the operational registers are
        // that far past it.
        assert_eq!(
            read32(&device, CAPABILITY_BASE) & 0xff,
            u32::from(CAPLENGTH)
        );
        assert_eq!(CAPABILITY_BASE + u64::from(CAPLENGTH), 0x140);
        // And `USBCMD` really is there: the reset value has the interrupt
        // threshold in bits 23:16.
        assert_eq!(read32(&device, 0x140) >> 16, 8);
        // `CONFIGFLAG` at +0x40 and `PORTSC1` at +0x44 land on the two offsets
        // every ChipIdea register map lists.
        assert_eq!(read32(&device, 0x180), 0);
        assert_ne!(read32(&device, 0x184), 0, "PORTSC1 reports port power");
    }

    #[test]
    fn the_identification_register_is_the_cx92755s() {
        let device = build();
        let id = read32(&device, REG_ID);
        assert_eq!(id, 0xfa05);
        // Self-consistency with the published field format, which is the check
        // that would catch a transcription error in the magic number.
        let core_id = id & 0x3f;
        let nid = (id >> 8) & 0x3f;
        assert_eq!(core_id, 5);
        assert_eq!(nid, !core_id & 0x3f, "NID is the complement of ID");
    }

    #[test]
    fn a_board_may_report_a_different_core_id() {
        let device = ChipIdea::with_bus(Arc::new(UsbBus::new(1)), Params::default(), 0xfb04);
        assert_eq!(read32(&device, REG_ID), 0xfb04);
    }

    #[test]
    fn the_role_select_is_write_once_and_stops_the_host_schedule() {
        let device = build();
        assert_eq!(
            read32(&device, REG_USBMODE) & 0x3,
            super::super::ehci::MODE_IDLE
        );

        write32(&device, REG_USBMODE, super::super::ehci::MODE_DEVICE);
        assert_eq!(
            read32(&device, REG_USBMODE) & 0x3,
            super::super::ehci::MODE_DEVICE
        );
        // Write-once: a second write does not change the role.
        write32(&device, REG_USBMODE, super::super::ehci::MODE_HOST);
        assert_eq!(
            read32(&device, REG_USBMODE) & 0x3,
            super::super::ehci::MODE_DEVICE,
            "USBMODE.CM is write-once after a reset"
        );

        // In device mode the host schedule does not run, so the controller
        // schedules no microframe however hard the guest starts it.
        write32(&device, 0x140, 1 | (1 << 5));
        assert_eq!(
            device.hcd().next_event_tick(),
            None,
            "a controller in device mode must not walk a host schedule"
        );
    }

    #[test]
    fn host_mode_starts_the_schedule() {
        let device = build();
        write32(&device, REG_USBMODE, super::super::ehci::MODE_HOST);
        write32(&device, 0x140, 1);
        assert_eq!(device.hcd().next_event_tick(), Some(7500));
    }

    #[test]
    fn the_device_controller_capabilities_are_reported() {
        let device = build();
        let dcc = read32(&device, REG_DCCPARAMS);
        assert_eq!(dcc & 0x1f, DEVICE_ENDPOINTS, "endpoint count");
        assert_ne!(dcc & (1 << 7), 0, "device capable");
        assert_ne!(dcc & (1 << 8), 0, "host capable");
        assert_eq!(read32(&device, REG_DCIVERSION) & 0xffff, DCIVERSION);
        assert_ne!(read32(&device, REG_HWHOST) & 1, 0, "host capable");
        assert_ne!(read32(&device, REG_HWDEVICE) & 1, 0, "device capable");
    }

    #[test]
    fn the_ulpi_window_never_leaves_a_poll_spinning() {
        let device = build();
        write32(&device, REG_ULPI_VIEWPORT, ULPI_RUN | 0x1234_0000 | 0x5a);
        let value = read32(&device, REG_ULPI_VIEWPORT);
        assert_eq!(value & ULPI_RUN, 0, "the access completed");
        assert_eq!(value & 0xff, 0x5a, "the data byte reads back");
    }

    #[test]
    fn a_debug_write_is_refused() {
        let device = build();
        assert!(
            ops(&device)
                .write(0x144, &0xffu32.to_le_bytes(), MemAttrs::DEBUG)
                .is_err(),
            "USBSTS is write-1-to-clear; a debugger must not acknowledge an interrupt"
        );
    }

    // Offsets the firmware's reset handshake touches, spelled as
    // `docs/buses/usb.md` spells them rather than derived, so that a change to
    // `OPERATIONAL_BASE` shows up here as a failure rather than as agreement.
    const FW_USBCMD: u64 = 0x140;
    const FW_USBSTS: u64 = 0x144;
    const FW_USBINTR: u64 = 0x148;
    const FW_USBMODE: u64 = 0x1a8;

    /// `USBCMD` bit 0.
    const RUN_STOP: u32 = 1 << 0;
    /// `USBCMD` bit 1.
    const HC_RESET: u32 = 1 << 1;
    /// `USBSTS` bit 12.
    const HC_HALTED: u32 = 1 << 12;

    /// The `EHCI_Host_Reset` to `EHCI_Init` handshake of
    /// `docs/buses/usb.md`, steps 1 to 4, in the documented order.
    ///
    /// Steps 5 to 7 — allocating buffer pools, building the schedules and
    /// enabling interrupts — are the generic EHCI's, and are tested there.
    #[test]
    fn the_firmwares_reset_handshake_completes_in_the_documented_order() {
        let device = build();
        assert_eq!(FW_USBMODE, REG_USBMODE);
        assert_eq!(FW_USBCMD, OPERATIONAL_BASE);

        // Step 1: poll `USBSTS.HCHalted` until the controller is halted. It is
        // halted out of reset, so the spin exits on its first read — and that
        // spin is the one the real firmware was recorded stuck on.
        assert_ne!(
            read32(&device, FW_USBSTS) & HC_HALTED,
            0,
            "step 1: a controller out of reset is halted"
        );

        // Step 2: assert `USBCMD.HCReset` and poll until it self-clears.
        write32(&device, FW_USBCMD, HC_RESET);
        assert_eq!(
            read32(&device, FW_USBCMD) & HC_RESET,
            0,
            "step 2: HCReset self-clears"
        );
        assert_ne!(
            read32(&device, FW_USBSTS) & HC_HALTED,
            0,
            "step 2: and the reset leaves the controller halted"
        );

        // Step 3: select host mode through `USBMODE`, then read it back. The
        // read-back is what the firmware checks, so it has to answer — and it
        // has to answer *after* the reset of step 2, which is why a reset
        // re-arms the write-once field rather than preserving it.
        write32(&device, FW_USBMODE, super::super::ehci::MODE_HOST);
        assert_eq!(
            read32(&device, FW_USBMODE) & 0x3,
            super::super::ehci::MODE_HOST,
            "step 3: the role select reads back what was written"
        );

        // Step 4: read `ID` to detect the controller. This is the firmware's
        // test verbatim, and the first thing whose success changes what that
        // firmware does — it treats USB host as optional and degrades, so
        // detection failing looks like a clean boot.
        assert_eq!(
            read32(&device, REG_ID) & 0xffff,
            0xfa05,
            "step 4: (ID & 0xFFFF) == 0xFA05"
        );

        // Step 7's register is where the documented flow ends, and it is an
        // ordinary EHCI one at an ordinary offset.
        write32(&device, FW_USBINTR, 0x3f);
        assert_eq!(read32(&device, FW_USBINTR), 0x3f);
    }

    /// The property a stub already had, and which the firmware's step-1 spin
    /// depends on: `HCHalted` is the complement of `RunStop`.
    #[test]
    fn hchalted_is_the_complement_of_runstop() {
        let device = build();
        write32(&device, FW_USBMODE, super::super::ehci::MODE_HOST);
        assert_ne!(read32(&device, FW_USBSTS) & HC_HALTED, 0);
        write32(&device, FW_USBCMD, RUN_STOP);
        assert_eq!(read32(&device, FW_USBSTS) & HC_HALTED, 0);
        write32(&device, FW_USBCMD, 0);
        assert_ne!(read32(&device, FW_USBSTS) & HC_HALTED, 0);
    }

    /// This firmware uses **both** roles — host for mass storage and
    /// PictBridge, device for the printer it presents to a PC — so switching
    /// between them has to work.
    ///
    /// `USBMODE.CM` is write-once *after a reset*, and the reset is what
    /// re-arms it. A model that carried the old role across `HCReset` would
    /// refuse the new one and hang the read-back of step 3.
    #[test]
    fn a_reset_re_arms_the_role_select_so_a_switch_works() {
        let device = build();
        write32(&device, FW_USBMODE, super::super::ehci::MODE_HOST);
        assert_eq!(
            read32(&device, FW_USBMODE) & 0x3,
            super::super::ehci::MODE_HOST
        );

        write32(&device, FW_USBCMD, HC_RESET);
        assert_eq!(
            read32(&device, FW_USBMODE) & 0x3,
            super::super::ehci::MODE_IDLE,
            "HCReset re-arms the write-once role select"
        );

        write32(&device, FW_USBMODE, super::super::ehci::MODE_DEVICE);
        assert_eq!(
            read32(&device, FW_USBMODE) & 0x3,
            super::super::ehci::MODE_DEVICE,
            "so the read-back the firmware spins on returns the new role"
        );
    }

    /// The block base is where `ID` is, **not** where `USBCMD` is.
    ///
    /// Worth an assertion because `docs/buses/usb.md` heads its address table
    /// "operational registers" against the block base, and a board that read
    /// that as "map the aperture at the operational registers" would map its
    /// window 0x140 bytes too high and find nothing.
    #[test]
    fn the_block_base_is_the_id_register_not_the_operational_ones() {
        let device = build();
        assert_eq!(read32(&device, 0) & 0xffff, 0xfa05);
        assert_eq!(OPERATIONAL_BASE, 0x140);
    }

    #[test]
    fn the_variant_registers_round_trip() {
        let device = build();
        write32(&device, REG_USBMODE, super::super::ehci::MODE_HOST);
        write32(&device, REG_TTCTRL, 0x7f00_0000);
        write32(&device, REG_ULPI_VIEWPORT, 0x0012_0034);
        write32(&device, REG_BURSTSIZE, 0x1010);

        let mut first = alloc::vec::Vec::new();
        device.hcd.save(&mut first).expect("it saves");
        let regs = *device.regs.lock();
        first.extend_from_slice(&regs.ttctrl.to_le_bytes());
        first.extend_from_slice(&regs.ulpi.to_le_bytes());

        let fresh = build();
        {
            let mut reader = crate::core::state::ChunkReader::new(&first);
            fresh.hcd.load(&mut reader).expect("it loads");
            let restored = VariantRegs {
                ttctrl: reader.read_u32().expect("ttctrl"),
                ulpi: reader.read_u32().expect("ulpi"),
            };
            *fresh.regs.lock() = restored;
        }
        assert_eq!(read32(&fresh, REG_TTCTRL), read32(&device, REG_TTCTRL));
        assert_eq!(
            read32(&fresh, REG_ULPI_VIEWPORT),
            read32(&device, REG_ULPI_VIEWPORT)
        );
        assert_eq!(read32(&fresh, REG_USBMODE), read32(&device, REG_USBMODE));
        assert_eq!(read32(&fresh, 0x140), read32(&device, 0x140));
    }
}
