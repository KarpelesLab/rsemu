//! An xHCI controller **enumerated over PCI**, driven the way a driver drives
//! one, with a **mouse** behind it and a screen beside it.
//!
//! `tests/usb_xhci.rs` proves the controller; `tests/nvme_board.rs` proves a
//! PCI function. This is the join, and it is the one that was missing: every
//! USB controller in this tree was MMIO-attached only, so no board could have a
//! display and a USB port at once and `host::input::mouse::capture` had nowhere
//! to deliver a pointer. Nothing below reaches into the controller. Everything
//! it does, it does through `machines/xhci-pci-mini.machine`'s own address
//! spaces — configuration cycles at `0xcf8`/`0xcfc`, the register block at
//! wherever this driver decided to put the base address register, and the
//! 8259A at `0x20`.
//!
//! The order is weakest claim first:
//!
//! * the bus shows a class `0C0330h` function whose BAR sizes as 16 KiB;
//! * a function whose `COMMAND[2]` is clear fetches **nothing** — not a
//!   command TRB, not an Event Ring Segment Table entry;
//! * with it set, a driver resets the root port, issues Enable Slot and
//!   Address Device, runs two control transfers on the default pipe and a
//!   Configure Endpoint, and reads a **HID boot report** off the interrupt
//!   endpoint — a report a VNC client's PointerEvent put there, through
//!   [`MouseSink`], which the guest cannot reach any other way;
//! * the completion interrupt travels `INTA#` off the card edge into an 8259A,
//!   is visible in its interrupt request register, and drops on the third of
//!   the three writes xHCI 1.2 §4.17 fixes the order of — and not before;
//! * `COMMAND[10]` masks the pin while `STATUS[3]` still reports the
//!   condition;
//! * a debugger may read every register and may write none;
//! * the board snapshots and restores to an identical state hash.
//!
//! # Events and interrupts are different numbers, and both are asserted
//!
//! The driver below takes eight event TRBs in eight interrupts, because it asks
//! for no moderation (`IMOD` = 0) and drains after every doorbell. That the two
//! *can* differ is not incidental — it is what §4.17.2's `ERDP.EHB` handshake
//! exists to do, and what makes `tests/usb_xhci.rs` count nineteen events in
//! fifteen traps — so it is asserted on its own, in
//! `two_commands_in_one_doorbell_are_two_events_and_one_interrupt`, where one
//! doorbell retires two commands and the pin asserts once.
//!
//! # Sources
//!
//! The xHCI 1.2c specification (Intel, document 868295): §4.2 the
//! initialisation sequence, §4.6 the commands, §4.9 ring operation and the
//! Cycle bit, §4.11.2.2 control transfers, §4.17 interrupters, §5.2 the PCI
//! configuration registers, §5.3-§5.6 the register files, §6.2 the contexts,
//! §6.4 the TRBs, §6.5 the Event Ring Segment Table. The *PCI Local Bus
//! Specification* Rev 2.1 §6.1/§6.2 for the header and §6.2.5.1 for the sizing
//! read-back, Rev 3.0 §6.2.2/§6.2.3 for Interrupt Disable and Interrupt
//! Status. USB 2.0 §9.4 for `SET_CONFIGURATION` and `GET_DESCRIPTOR`, and HID
//! 1.11 Appendix B.2 for the boot mouse report. No emulator source and no
//! operating system's xHCI driver was opened (`ROADMAP.md` §1).

#![cfg(all(feature = "machine-xhci-pci-mini", feature = "std"))]

use std::cell::Cell;
use std::sync::Arc;

use rsemu::core::device::ResetKind;
use rsemu::core::space::MemAttrs;
use rsemu::core::value::Width;
use rsemu::dev::usb::hid::HidMouse;
use rsemu::host::input::{Feed, InputEvent, MouseSink};
use rsemu::machine::Machine;

// ---------------------------------------------------------------------------
// the board
// ---------------------------------------------------------------------------

/// The board, and the mouse the *host* got a handle to.
///
/// The handle comes through `host::input::mouse::capture`, which is the seam
/// `rsemu run` itself uses to find a pointer — so what this test holds is
/// exactly what a VNC session holds, and nothing more.
fn board() -> (Machine, Arc<HidMouse>) {
    let mut options = rsemu::machine::catalog::build_options().expect("this build's options");
    rsemu::host::input::mouse::capture::install(&mut options).expect("nothing else claimed it");

    let registry = rsemu::machine::catalog::registry().expect("this build's registry");
    let entry = &rsemu::machine::catalog::XHCI_PCI_MINI;
    let mut machine = match rsemu::machine::build(entry.name, entry.source, &registry, &options) {
        Ok(m) => m,
        Err(e) => panic!("the board does not realize: {e}"),
    };
    machine.reset(ResetKind::Cold);
    machine.sweep();

    let mouse = rsemu::host::input::mouse::capture::take(&options.realize.hosts)
        .expect("this board has a mouse, which is the whole point of it");
    (machine, mouse)
}

// ---------------------------------------------------------------------------
// the two spaces, as a driver reaches them
// ---------------------------------------------------------------------------

fn outb(m: &Machine, port: u64, value: u8) {
    m.space("port")
        .expect("the I/O space")
        .write(port, Width::U8, u64::from(value), MemAttrs::DEFAULT)
        .expect("a decoded port");
}

fn inb(m: &Machine, port: u64) -> u8 {
    m.space("port")
        .expect("the I/O space")
        .read(port, Width::U8, MemAttrs::DEFAULT)
        .expect("a decoded port") as u8
}

fn peek32(m: &Machine, addr: u64) -> u32 {
    m.space("mem")
        .expect("the memory space")
        .read(addr, Width::U32, MemAttrs::DEFAULT)
        .expect("a mapped dword") as u32
}

fn poke32(m: &Machine, addr: u64, value: u32) {
    m.space("mem")
        .expect("the memory space")
        .write(addr, Width::U32, u64::from(value), MemAttrs::DEFAULT)
        .expect("a mapped dword");
}

fn peek_bytes(m: &Machine, addr: u64, len: u64) -> Vec<u8> {
    let mut out = vec![0u8; len as usize];
    m.space("mem")
        .expect("the memory space")
        .read_bytes(addr, &mut out, MemAttrs::DEFAULT)
        .expect("mapped memory");
    out
}

// ---------------------------------------------------------------------------
// configuration space, mechanism #1
// ---------------------------------------------------------------------------

/// Where the controller sits: bus 0, device 5, function 0, as the machine file
/// says — and as [`find_controller`] confirms without being told.
const XHCI_DEVICE: u32 = 5;

fn config_read_at(m: &Machine, device: u32, register: u16) -> u32 {
    let addr = 0x8000_0000 | (device << 11) | u32::from(register & 0xfc);
    let port = m.space("port").expect("the I/O space");
    port.write(0xcf8, Width::U32, u64::from(addr), MemAttrs::DEFAULT)
        .expect("CONFADD takes a dword");
    port.read(0xcfc, Width::U32, MemAttrs::DEFAULT)
        .expect("CONFDATA answers") as u32
}

fn config_read(m: &Machine, register: u16) -> u32 {
    config_read_at(m, XHCI_DEVICE, register)
}

fn config_write(m: &Machine, register: u16, value: u32) {
    let addr = 0x8000_0000 | (XHCI_DEVICE << 11) | u32::from(register & 0xfc);
    let port = m.space("port").expect("the I/O space");
    port.write(0xcf8, Width::U32, u64::from(addr), MemAttrs::DEFAULT)
        .expect("CONFADD takes a dword");
    port.write(0xcfc, Width::U32, u64::from(value), MemAttrs::DEFAULT)
        .expect("CONFDATA takes a dword");
}

/// The Type 00h header offsets this driver names (Rev 2.1 §6.1).
const CFG_VENDOR: u16 = 0x00;
const CFG_COMMAND: u16 = 0x04;
const CFG_CLASS: u16 = 0x08;
const CFG_HEADER: u16 = 0x0e;
const CFG_BAR0: u16 = 0x10;
const CFG_BAR1: u16 = 0x14;
const CFG_INT_PIN: u16 = 0x3d;

/// `COMMAND[1]` Memory Space and `COMMAND[2]` Bus Master.
const COMMAND_ON: u32 = 0x0006;
/// `COMMAND[10]`, Interrupt Disable (Rev 3.0 §6.2.2).
const COMMAND_INTX_OFF: u32 = 0x0400;
/// `STATUS[3]`, Interrupt Status (Rev 3.0 §6.2.3), in the upper half of the
/// dword at `0x04`.
const STATUS_INTERRUPT: u32 = 0x08 << 16;

/// Class code `0C0330h`: a serial bus controller, USB, xHCI.
const CLASS_XHCI: u32 = 0x0c03_3000;

/// Scan bus 0 for the first function whose class code is `0C0330h`, the way an
/// operating system does — it is not told where the controller is.
fn find_controller(m: &Machine) -> u32 {
    for device in 0..32 {
        let id = config_read_at(m, device, CFG_VENDOR);
        if id == 0xffff_ffff || id == 0 {
            continue;
        }
        if config_read_at(m, device, CFG_CLASS) == CLASS_XHCI {
            return device;
        }
    }
    panic!("no xHCI on this bus");
}

// ---------------------------------------------------------------------------
// the register block, once the driver has placed it
// ---------------------------------------------------------------------------

/// Where this driver puts the controller's 16 KiB register window. Above the
/// board's 16 MiB of RAM, and aligned to the window's own size as §6.2.5.1
/// requires.
const BAR_BASE: u64 = 0xf000_0000;

/// What sizing BAR0 must report: a 16 KiB, 64-bit (type `10b`),
/// non-prefetchable memory window.
const BAR0_MASK: u32 = 0xffff_c004;

/// `CAPLENGTH` (§5.3.1) — where the operational registers start on this model.
const CAPLENGTH: u64 = 0x40;
const OP: u64 = CAPLENGTH;
/// `DBOFF` (§5.3.7) and `RTSOFF` (§5.3.8).
const DB: u64 = 0x1000;
const RT: u64 = 0x2000;
/// Interrupter 0's register set (§5.5, Table 5-35).
const IR0: u64 = RT + 0x20;

const USBCMD: u64 = OP;
const USBSTS: u64 = OP + 0x04;
const CRCR: u64 = OP + 0x18;
const DCBAAP: u64 = OP + 0x30;
const CONFIG: u64 = OP + 0x38;
const PORTSC1: u64 = OP + 0x400;
const IMAN: u64 = IR0;
const IMOD: u64 = IR0 + 0x04;
const ERSTSZ: u64 = IR0 + 0x08;
const ERSTBA: u64 = IR0 + 0x10;
const ERDP: u64 = IR0 + 0x18;

/// `USBCMD` bits (§5.4.1) and `USBSTS` bits (§5.4.2).
const CMD_RS: u32 = 1 << 0;
const CMD_INTE: u32 = 1 << 2;
const STS_EINT: u32 = 1 << 3;
/// `PORTSC` bits (§5.4.8).
const PORT_PED: u32 = 1 << 1;
const PORT_PR: u32 = 1 << 4;
const PORT_PP: u32 = 1 << 9;
const PORT_CSC: u32 = 1 << 17;
/// `IMAN` (§5.5.2.1) and `ERDP` (§5.5.2.3.3).
const IMAN_IP: u32 = 1 << 0;
const IMAN_IE: u32 = 1 << 1;
const ERDP_EHB: u32 = 1 << 3;

/// TRB layout (§6.4).
const TRB: u64 = 16;
const TRB_TYPE_SHIFT: u32 = 10;
const TRB_CYCLE: u32 = 1 << 0;
const TRB_ISP: u32 = 1 << 2;
const TRB_IOC: u32 = 1 << 5;
const TRB_IDT: u32 = 1 << 6;
const TRB_DIR: u32 = 1 << 16;

/// The TRB types this driver builds and reads (§6.4.6, Table 6-91).
mod trb {
    pub(crate) const NORMAL: u32 = 1;
    pub(crate) const SETUP_STAGE: u32 = 2;
    pub(crate) const DATA_STAGE: u32 = 3;
    pub(crate) const STATUS_STAGE: u32 = 4;
    pub(crate) const ENABLE_SLOT: u32 = 9;
    pub(crate) const ADDRESS_DEVICE: u32 = 11;
    pub(crate) const CONFIGURE_ENDPOINT: u32 = 12;
    pub(crate) const NO_OP_COMMAND: u32 = 23;
    pub(crate) const TRANSFER_EVENT: u32 = 32;
    pub(crate) const COMMAND_COMPLETION_EVENT: u32 = 33;
    pub(crate) const PORT_STATUS_CHANGE_EVENT: u32 = 34;
}

/// Completion code 1, Success (§6.4.5).
const CODE_SUCCESS: u32 = 1;

/// A context entry is 32 bytes, because `HCCPARAMS1.CSZ` is zero (§6.2).
const CTX: u64 = 32;
/// Slot Context fields (§6.2.2, Table 6-4).
const SLOT_ENTRIES_SHIFT: u32 = 27;
const SLOT_SPEED_SHIFT: u32 = 20;
const SLOT_PORT_SHIFT: u32 = 16;
/// Endpoint Context fields (§6.2.3, Table 6-8).
const EP_TYPE_SHIFT: u32 = 3;
const EP_MPS_SHIFT: u32 = 16;
const EP_DCS: u32 = 1 << 0;
/// Endpoint types: 4 control, 7 interrupt IN (§6.2.3, Table 6-9).
const EP_TYPE_CONTROL: u32 = 4;
const EP_TYPE_INTERRUPT_IN: u32 = 7;
/// Protocol Speed ID 1, full speed (§7.2.2.1.1, Table 7-13) — what the machine
/// file gives the mouse.
const PSI_FULL: u32 = 1;

/// The mouse's interrupt IN endpoint is endpoint 1, so its Device Context Index
/// is `1 * 2 + 1` (§4.5.1).
const DCI_EP0: u32 = 1;
const DCI_INT_IN: u32 = 3;
/// Three bytes: buttons, relative X, relative Y (HID 1.11 Appendix B.2).
const REPORT_BYTES: u32 = 3;

// ---------------------------------------------------------------------------
// where the driver builds everything, in the board's own RAM
// ---------------------------------------------------------------------------

const DCBAA: u64 = 0x0010_0000;
const ERST: u64 = 0x0010_0040;
const IN_CTX: u64 = 0x0010_0400;
const DEV_CTX: u64 = 0x0010_0800;
const EVT_RING: u64 = 0x0010_1000;
const EVT_TRBS: u32 = 16;
const CMD_RING: u64 = 0x0010_1400;
const EP0_RING: u64 = 0x0010_1800;
const EPINT_RING: u64 = 0x0010_1c00;
const DESC_BUF: u64 = 0x0010_2000;
const REPORT_BUF: u64 = 0x0010_2100;

// ---------------------------------------------------------------------------
// the driver
// ---------------------------------------------------------------------------

/// A driver's whole state: where it is up to on each ring, and the two counts
/// the module documentation claims.
struct Driver<'a> {
    m: &'a Machine,
    cmd_next: Cell<u64>,
    cmd_cycle: Cell<bool>,
    ep0_next: Cell<u64>,
    epint_next: Cell<u64>,
    evt_next: Cell<u64>,
    evt_cycle: Cell<bool>,
    /// Event TRBs consumed.
    events: Cell<usize>,
    /// Times the pin was found asserted when the handler ran.
    interrupts: Cell<usize>,
}

impl<'a> Driver<'a> {
    fn new(m: &'a Machine) -> Driver<'a> {
        Driver {
            m,
            cmd_next: Cell::new(CMD_RING),
            cmd_cycle: Cell::new(true),
            ep0_next: Cell::new(EP0_RING),
            epint_next: Cell::new(EPINT_RING),
            evt_next: Cell::new(EVT_RING),
            evt_cycle: Cell::new(true),
            events: Cell::new(0),
            interrupts: Cell::new(0),
        }
    }

    // -- the register block ------------------------------------------------

    fn reg(&self, offset: u64) -> u32 {
        peek32(self.m, BAR_BASE + offset)
    }

    fn set_reg(&self, offset: u64, value: u32) {
        poke32(self.m, BAR_BASE + offset, value);
    }

    fn put_trb(&self, addr: u64, trb: [u32; 4]) {
        for (i, word) in trb.iter().enumerate() {
            poke32(self.m, addr + (i * 4) as u64, *word);
        }
    }

    fn get_trb(&self, addr: u64) -> [u32; 4] {
        [
            peek32(self.m, addr),
            peek32(self.m, addr + 4),
            peek32(self.m, addr + 8),
            peek32(self.m, addr + 12),
        ]
    }

    // -- rings -------------------------------------------------------------

    /// Put `trb` on the command ring and ring doorbell 0 (§5.6).
    fn command(&self, mut trb: [u32; 4]) {
        let at = self.cmd_next.get();
        trb[3] = (trb[3] & !TRB_CYCLE) | u32::from(self.cmd_cycle.get());
        self.put_trb(at, trb);
        self.cmd_next.set(at + TRB);
        self.set_reg(DB, 0);
    }

    /// Take the next event off the ring, if the Cycle bit says there is one
    /// (§4.9.4).
    fn event(&self) -> Option<[u32; 4]> {
        let at = self.evt_next.get();
        let trb = self.get_trb(at);
        if (trb[3] & TRB_CYCLE != 0) != self.evt_cycle.get() {
            return None;
        }
        let mut next = at + TRB;
        if next >= EVT_RING + u64::from(EVT_TRBS) * TRB {
            next = EVT_RING;
            self.evt_cycle.set(!self.evt_cycle.get());
        }
        self.evt_next.set(next);
        self.events.set(self.events.get() + 1);
        Some(trb)
    }

    /// What an interrupt handler does: drain the ring, then acknowledge in the
    /// order §4.17 fixes — `USBSTS.EINT`, `ERDP` with `EHB`, `IMAN.IP`.
    fn drain(&self) -> Vec<[u32; 4]> {
        if irr(self.m) & IR5 != 0 {
            self.interrupts.set(self.interrupts.get() + 1);
        }
        let mut out = Vec::new();
        while let Some(trb) = self.event() {
            out.push(trb);
        }
        self.set_reg(USBSTS, STS_EINT);
        self.set_reg(ERDP, self.evt_next.get() as u32 | ERDP_EHB);
        self.set_reg(ERDP + 4, 0);
        self.set_reg(IMAN, IMAN_IP | IMAN_IE);
        out
    }

    /// One event, and it succeeded.
    fn one(&self, kind: u32) -> [u32; 4] {
        let events = self.drain();
        assert_eq!(events.len(), 1, "expected exactly one event: {events:?}");
        assert_eq!(
            events[0][3] >> TRB_TYPE_SHIFT & 0x3f,
            kind,
            "the wrong kind of event"
        );
        assert_eq!(
            events[0][2] >> 24,
            CODE_SUCCESS,
            "the event reports failure"
        );
        events[0]
    }

    // -- the initialisation sequence (§4.2) --------------------------------

    fn init(&self) {
        // The Device Context Base Address Array: entry 0 is the scratchpad
        // pointer (none), entry 1 is slot 1's Device Context (§6.1).
        for word in 0..4 {
            poke32(self.m, DCBAA + word * 4, 0);
        }
        poke32(self.m, DCBAA + 8, DEV_CTX as u32);
        for word in 0..(8 * CTX / 4) {
            poke32(self.m, DEV_CTX + word * 4, 0);
        }
        // One Event Ring Segment Table entry (§6.5).
        poke32(self.m, ERST, EVT_RING as u32);
        poke32(self.m, ERST + 4, 0);
        poke32(self.m, ERST + 8, EVT_TRBS);
        poke32(self.m, ERST + 12, 0);
        for word in 0..(u64::from(EVT_TRBS) * 4) {
            poke32(self.m, EVT_RING + word * 4, 0);
        }

        self.set_reg(CONFIG, 1);
        self.set_reg(DCBAAP, DCBAA as u32);
        self.set_reg(DCBAAP + 4, 0);
        // §5.4.5: the Ring Cycle State software starts the command ring with.
        self.set_reg(CRCR, CMD_RING as u32 | 1);
        self.set_reg(CRCR + 4, 0);
        self.set_reg(ERSTSZ, 1);
        self.set_reg(ERDP, EVT_RING as u32);
        self.set_reg(ERDP + 4, 0);
        self.set_reg(ERSTBA, ERST as u32);
        self.set_reg(ERSTBA + 4, 0);
        // §5.5.2.2: zero disables throttling, so an event interrupts at once.
        self.set_reg(IMOD, 0);
        self.set_reg(IMAN, IMAN_IE);
        self.set_reg(USBCMD, CMD_RS | CMD_INTE);
    }

    /// Clear the attach the port already reports, then reset it — which is what
    /// enables a USB2 port (§4.19.5, §5.4.8's `PR`).
    fn reset_port(&self) {
        self.set_reg(PORTSC1, PORT_PP | PORT_CSC);
        self.set_reg(PORTSC1, PORT_PP | PORT_PR);
    }

    /// Enable a slot and address the device on port 1 (§4.6.3, §4.6.5).
    fn address_device(&self) -> u8 {
        self.command([0, 0, 0, trb::ENABLE_SLOT << TRB_TYPE_SHIFT]);
        let event = self.one(trb::COMMAND_COMPLETION_EVENT);
        let slot = (event[3] >> 24) as u8;
        assert_eq!(slot, 1, "the first slot enabled is slot 1");

        // The Input Context: an Input Control Context, a Slot Context and an
        // Endpoint 0 Context (§6.2.5).
        for word in 0..(3 * CTX / 4) {
            poke32(self.m, IN_CTX + word * 4, 0);
        }
        // A0 | A1 (§4.6.5): add the slot and the default control endpoint.
        poke32(self.m, IN_CTX + 4, 0x3);
        // Context Entries 1, and the speed software believes the port is at.
        poke32(
            self.m,
            IN_CTX + CTX,
            (1 << SLOT_ENTRIES_SHIFT) | (PSI_FULL << SLOT_SPEED_SHIFT),
        );
        // Root Hub Port Number 1 — one-based here, zero-based on the fabric.
        poke32(self.m, IN_CTX + CTX + 4, 1 << SLOT_PORT_SHIFT);
        // Endpoint 0: control, CErr = 3, Max Packet Size 8, which is what a
        // full-speed device's default pipe is (USB 2.0 §5.5.3).
        poke32(
            self.m,
            IN_CTX + 2 * CTX + 4,
            (3 << 1) | (EP_TYPE_CONTROL << EP_TYPE_SHIFT) | (8 << EP_MPS_SHIFT),
        );
        poke32(self.m, IN_CTX + 2 * CTX + 8, EP0_RING as u32 | EP_DCS);

        self.command([
            IN_CTX as u32,
            0,
            0,
            (trb::ADDRESS_DEVICE << TRB_TYPE_SHIFT) | (u32::from(slot) << 24),
        ]);
        self.one(trb::COMMAND_COMPLETION_EVENT);
        slot
    }

    /// One control transfer on the default pipe (§4.11.2.2): a Setup Stage TD,
    /// an optional Data Stage TD, and a Status Stage TD which is the only one
    /// carrying `IOC` — so the transfer is one event however many TRBs it took.
    fn control(&self, setup: [u8; 8], data: Option<(u64, u32)>, slot: u8) {
        let mut at = self.ep0_next.get();
        // §6.4.1.2.1: the eight bytes carried immediately, and the Transfer
        // Type in bits 17:16 — 0 for no data stage, 2 for OUT, 3 for IN.
        let trt = match (data, setup[0] & 0x80 != 0) {
            (None, _) => 0,
            (Some(_), true) => 3,
            (Some(_), false) => 2,
        };
        self.put_trb(
            at,
            [
                u32::from_le_bytes([setup[0], setup[1], setup[2], setup[3]]),
                u32::from_le_bytes([setup[4], setup[5], setup[6], setup[7]]),
                8,
                (trb::SETUP_STAGE << TRB_TYPE_SHIFT) | TRB_IDT | TRB_CYCLE | (trt << 16),
            ],
        );
        at += TRB;
        if let Some((buffer, len)) = data {
            let dir = if setup[0] & 0x80 != 0 { TRB_DIR } else { 0 };
            self.put_trb(
                at,
                [
                    buffer as u32,
                    (buffer >> 32) as u32,
                    len,
                    (trb::DATA_STAGE << TRB_TYPE_SHIFT) | TRB_CYCLE | dir,
                ],
            );
            at += TRB;
        }
        // §4.11.2.2: the status stage runs the opposite way to the data stage,
        // and an IN when there was no data stage at all.
        let dir = if data.is_some() && setup[0] & 0x80 != 0 {
            0
        } else {
            TRB_DIR
        };
        self.put_trb(
            at,
            [
                0,
                0,
                0,
                (trb::STATUS_STAGE << TRB_TYPE_SHIFT) | TRB_IOC | TRB_CYCLE | dir,
            ],
        );
        self.ep0_next.set(at + TRB);
        self.set_reg(DB + 4 * u64::from(slot), DCI_EP0);
        self.one(trb::TRANSFER_EVENT);
    }

    /// Add the mouse's interrupt IN endpoint with a Configure Endpoint Command
    /// (§4.6.6).
    fn configure_endpoint(&self, slot: u8) {
        for word in 0..(6 * CTX / 4) {
            poke32(self.m, IN_CTX + word * 4, 0);
        }
        // A0, plus the endpoint's own Device Context Index.
        poke32(self.m, IN_CTX + 4, 1 | (1 << DCI_INT_IN));
        poke32(
            self.m,
            IN_CTX + CTX,
            (DCI_INT_IN << SLOT_ENTRIES_SHIFT) | (PSI_FULL << SLOT_SPEED_SHIFT),
        );
        poke32(self.m, IN_CTX + CTX + 4, 1 << SLOT_PORT_SHIFT);
        // The Input Context index of a Device Context Index is one higher
        // (§6.2.5.1): the Input Control Context sits in front.
        let ep = IN_CTX + u64::from(DCI_INT_IN + 1) * CTX;
        poke32(
            self.m,
            ep + 4,
            (3 << 1) | (EP_TYPE_INTERRUPT_IN << EP_TYPE_SHIFT) | (REPORT_BYTES << EP_MPS_SHIFT),
        );
        poke32(self.m, ep + 8, EPINT_RING as u32 | EP_DCS);

        self.command([
            IN_CTX as u32,
            0,
            0,
            (trb::CONFIGURE_ENDPOINT << TRB_TYPE_SHIFT) | (u32::from(slot) << 24),
        ]);
        self.one(trb::COMMAND_COMPLETION_EVENT);
    }

    /// Collect one report off the interrupt endpoint.
    fn poll_mouse(&self, slot: u8) -> Vec<u8> {
        for word in 0..2 {
            poke32(self.m, REPORT_BUF + word * 4, 0);
        }
        let at = self.epint_next.get();
        // `ISP` as well as `IOC` (§6.4.1.1 bit 2), so a short packet is
        // reported rather than looking like a full one.
        self.put_trb(
            at,
            [
                REPORT_BUF as u32,
                0,
                REPORT_BYTES,
                (trb::NORMAL << TRB_TYPE_SHIFT) | TRB_IOC | TRB_ISP | TRB_CYCLE,
            ],
        );
        self.epint_next.set(at + TRB);
        self.set_reg(DB + 4 * u64::from(slot), DCI_INT_IN);
        self.one(trb::TRANSFER_EVENT);
        peek_bytes(self.m, REPORT_BUF, u64::from(REPORT_BYTES))
    }
}

// ---------------------------------------------------------------------------
// the 8259A, and the pin
// ---------------------------------------------------------------------------

/// Which input `machines/xhci-pci-mini.machine` wires `INTA#` to.
const IR5: u8 = 1 << 5;

/// The 8259A, initialised the way a driver of a level-triggered PCI interrupt
/// has to initialise one.
fn init_pic(m: &Machine) {
    outb(m, 0x20, 0x11); // ICW1: cascade, ICW4 to follow
    outb(m, 0x21, 0x08); // ICW2: vectors from 0x08
    outb(m, 0x21, 0x04); // ICW3: a slave would be on IR2
    outb(m, 0x21, 0x01); // ICW4: 8086 mode
    outb(m, 0x21, 0xdf); // OCW1: everything masked but IR5
    // The edge/level control register. `INTx#` is level sensitive (Rev 2.1
    // §2.2.6) and so is xHCI's interrupt (§4.17.3): an edge-triggered input
    // would latch the first completion and miss every later one raised while
    // the line was already low.
    outb(m, 0x4d0, IR5);
}

/// The interrupt request register, which for a level-triggered line is the line
/// itself.
fn irr(m: &Machine) -> u8 {
    outb(m, 0x20, 0x0a); // OCW3: the next read of port 0 is the IRR
    inb(m, 0x20)
}

// ---------------------------------------------------------------------------
// enumeration
// ---------------------------------------------------------------------------

/// What firmware does before a driver exists: find the function, size its
/// window, place it, and switch it on.
fn enumerate(m: &Machine) {
    assert_eq!(find_controller(m), XHCI_DEVICE);
    // §6.2.5.1: write all ones, read the mask back, then put the base in.
    config_write(m, CFG_BAR0, 0xffff_ffff);
    assert_eq!(
        config_read(m, CFG_BAR0),
        BAR0_MASK,
        "a 16 KiB 64-bit non-prefetchable memory window"
    );
    config_write(m, CFG_BAR1, 0xffff_ffff);
    config_write(m, CFG_BAR0, BAR_BASE as u32);
    config_write(m, CFG_BAR1, (BAR_BASE >> 32) as u32);
    config_write(m, CFG_COMMAND, COMMAND_ON);
}

/// A board with the controller enumerated, the interrupt controller
/// initialised, and a driver ready to touch the register block.
fn ready() -> (Machine, Arc<HidMouse>) {
    let (m, mouse) = board();
    enumerate(&m);
    init_pic(&m);
    (m, mouse)
}

// ---------------------------------------------------------------------------
// the tests
// ---------------------------------------------------------------------------

#[test]
fn the_bus_shows_a_usb_controller_a_driver_would_bind_to() {
    let (m, _mouse) = board();
    let device = find_controller(&m);
    assert_eq!(device, XHCI_DEVICE);
    // §6.2.1: header type 00h, single function, and §6.2.4: `INTA#`.
    assert_eq!((config_read(&m, CFG_HEADER) >> 16) & 0xff, 0x00);
    assert_eq!((config_read(&m, CFG_INT_PIN) >> 8) & 0xff, 1);
    // §6.2.2: nothing decodes and nothing masters until firmware says so.
    assert_eq!(config_read(&m, CFG_COMMAND) & 0xffff, 0);
    assert!(
        m.space("mem")
            .expect("the memory space")
            .read(BAR_BASE, Width::U32, MemAttrs::DEFAULT)
            .is_ok_and(|v| v == u64::from(u32::MAX)),
        "an unclaimed cycle is a master abort, which reads as all ones"
    );
}

#[test]
fn the_capability_registers_say_where_everything_else_is() {
    let (m, _mouse) = ready();
    let d = Driver::new(&m);
    // §5.3.1, §5.3.2.
    assert_eq!(d.reg(0) & 0xff, CAPLENGTH as u32);
    assert_eq!(d.reg(0) >> 16, 0x0100);
    // §5.3.7, §5.3.8: a driver reads these rather than assuming them.
    assert_eq!(u64::from(d.reg(0x14)), DB);
    assert_eq!(u64::from(d.reg(0x18)), RT);
    // §5.3.3: MaxSlots in 7:0, MaxIntrs in 18:8, MaxPorts in 31:24 — the
    // machine file asked for four slots and two ports.
    let hcs1 = d.reg(0x04);
    assert_eq!(hcs1 & 0xff, 4);
    assert_eq!((hcs1 >> 8) & 0x7ff, 1);
    assert_eq!(hcs1 >> 24, 2);
}

#[test]
fn a_function_that_may_not_master_the_bus_fetches_nothing() {
    let (m, _mouse) = board();
    // Memory Space but no Bus Master: the driver can reach every register and
    // the controller can reach nothing (Rev 2.1 §6.2.2).
    assert_eq!(find_controller(&m), XHCI_DEVICE);
    config_write(&m, CFG_BAR0, BAR_BASE as u32);
    config_write(&m, CFG_BAR1, (BAR_BASE >> 32) as u32);
    config_write(&m, CFG_COMMAND, 0x0002);
    init_pic(&m);

    let d = Driver::new(&m);
    d.init();
    d.command([0, 0, 0, trb::NO_OP_COMMAND << TRB_TYPE_SHIFT]);
    assert_eq!(
        d.get_trb(EVT_RING),
        [0, 0, 0, 0],
        "an event ring the controller never wrote"
    );
    assert_eq!(irr(&m) & IR5, 0, "and no interrupt");

    // Now let it master. The Event Ring State Machine is started by a write to
    // `ERSTBA` (§5.5.2.3.2), and that write was a DMA read the function was not
    // allowed to make — so it is made again, which is what a driver that
    // enabled the function in the right order would never have to do.
    config_write(&m, CFG_COMMAND, COMMAND_ON);
    d.set_reg(ERSTBA, ERST as u32);
    d.set_reg(ERSTBA + 4, 0);
    d.set_reg(DB, 0);
    assert_eq!(d.get_trb(EVT_RING)[2] >> 24, CODE_SUCCESS);
    assert_eq!(irr(&m) & IR5, IR5, "and the pin follows");
}

/// The whole driver, once: enumerate, bring the controller up, address the
/// mouse, configure its endpoint, and collect a report a VNC client produced.
fn drive<'a>(m: &'a Machine, mouse: &Arc<HidMouse>) -> (Driver<'a>, Vec<u8>) {
    let d = Driver::new(m);
    d.init();
    d.reset_port();
    // §4.19.5: the port reset completes and the port is enabled.
    let events = d.drain();
    assert!(
        events
            .iter()
            .any(|e| e[3] >> TRB_TYPE_SHIFT & 0x3f == trb::PORT_STATUS_CHANGE_EVENT),
        "the reset produced no Port Status Change Event: {events:?}"
    );
    assert_eq!(d.reg(PORTSC1) & PORT_PED, PORT_PED, "the port is enabled");

    let slot = d.address_device();
    // §6.2.2: the Output Slot Context reports the speed the *port* latched,
    // whatever software guessed — full, which is what the machine file gives
    // the mouse.
    let out_slot = peek32(m, DEV_CTX);
    assert_eq!((out_slot >> SLOT_SPEED_SHIFT) & 0xf, PSI_FULL);

    // USB 2.0 §9.4.7, and §9.4.3 with a data stage of eighteen bytes.
    d.control([0x00, 9, 1, 0, 0, 0, 0, 0], None, slot);
    d.control([0x80, 6, 0, 1, 0, 0, 18, 0], Some((DESC_BUF, 18)), slot);
    let descriptor = peek_bytes(m, DESC_BUF, 18);
    assert_eq!(descriptor[0], 18, "bLength");
    assert_eq!(descriptor[1], 1, "bDescriptorType: DEVICE");
    assert_eq!(
        u16::from_le_bytes([descriptor[8], descriptor[9]]),
        0x1234,
        "idVendor, as the machine file gives it"
    );
    assert_eq!(
        u16::from_le_bytes([descriptor[10], descriptor[11]]),
        0x0002,
        "idProduct"
    );

    d.configure_endpoint(slot);

    // The pointer, arriving the way a VNC client's does: RFB coordinates into
    // the input seam, a relative HID report out. The first event is a datum —
    // a jump from the origin to wherever the cursor entered the window would
    // fling the guest's pointer across the screen — so it takes two.
    let feed = Feed::new();
    feed.attach(Arc::new(MouseSink::new(Arc::clone(mouse))));
    feed.deliver(InputEvent::Pointer {
        x: 100,
        y: 100,
        buttons: 0,
    });
    assert_eq!(d.poll_mouse(slot), vec![0, 0, 0], "the first is a datum");
    feed.deliver(InputEvent::Pointer {
        x: 110,
        y: 95,
        buttons: 1,
    });
    let report = d.poll_mouse(slot);
    (d, report)
}

#[test]
fn a_guest_enumerates_the_controller_and_reads_a_mouse_behind_it() {
    let (m, mouse) = ready();
    let (d, report) = drive(&m, &mouse);

    // HID 1.11 Appendix B.2: buttons, then relative X and Y as signed bytes.
    assert_eq!(report[0], 1, "the left button, which is bit 0 in both");
    assert_eq!(report[1] as i8, 10);
    assert_eq!(report[2] as i8, -5);

    // Both counts, separately, because they are separate facts: this driver
    // asked for no moderation (`IMOD` = 0) and drains after every doorbell, so
    // here they happen to be equal, and
    // `two_commands_in_one_doorbell_are_two_events_and_one_interrupt` is the
    // case where they are not.
    assert_eq!(d.events.get(), 8, "event TRBs");
    assert_eq!(d.interrupts.get(), 8, "interrupts taken");
}

/// Events and interrupts are different numbers, and the PCI layer does not
/// change that.
///
/// §4.17.2: `ERDP.EHB` blocks a second interrupt until the handler has written
/// the dequeue pointer, so a doorbell that retires two work items posts two
/// events and asserts the pin once. `tests/usb_xhci.rs` measures the same thing
/// at scale — nineteen events in fifteen traps — and this is the smallest form
/// of it, with an 8259A on the far end of `INTA#` instead of a PLIC.
#[test]
fn two_commands_in_one_doorbell_are_two_events_and_one_interrupt() {
    let (m, _mouse) = ready();
    let d = Driver::new(&m);
    d.init();

    // Two No Op Commands (§6.4.3.1) handed over before the doorbell rings.
    d.put_trb(
        CMD_RING,
        [0, 0, 0, (trb::NO_OP_COMMAND << TRB_TYPE_SHIFT) | TRB_CYCLE],
    );
    d.put_trb(
        CMD_RING + TRB,
        [0, 0, 0, (trb::NO_OP_COMMAND << TRB_TYPE_SHIFT) | TRB_CYCLE],
    );
    d.cmd_next.set(CMD_RING + 2 * TRB);
    d.set_reg(DB, 0);

    assert_eq!(irr(&m) & IR5, IR5, "one interrupt");
    let events = d.drain();
    assert_eq!(events.len(), 2, "two Command Completion Events");
    for event in &events {
        assert_eq!(event[2] >> 24, CODE_SUCCESS);
    }
    assert_eq!(irr(&m) & IR5, 0, "and it is acknowledged once");
    assert_eq!(d.events.get(), 2);
    assert_eq!(d.interrupts.get(), 1);
}

#[test]
fn the_interrupt_drops_on_the_third_write_and_not_before() {
    let (m, mouse) = ready();
    let (d, _) = drive(&m, &mouse);

    // Arm one more transfer and leave it unacknowledged.
    mouse.motion(3, 4, 0);
    let at = d.epint_next.get();
    d.put_trb(
        at,
        [
            REPORT_BUF as u32,
            0,
            REPORT_BYTES,
            (trb::NORMAL << TRB_TYPE_SHIFT) | TRB_IOC | TRB_CYCLE,
        ],
    );
    d.epint_next.set(at + TRB);
    d.set_reg(DB + 4, DCI_INT_IN);

    assert_eq!(irr(&m) & IR5, IR5, "the completion asserted the pin");
    assert_eq!(
        config_read(&m, CFG_COMMAND) & STATUS_INTERRUPT,
        STATUS_INTERRUPT,
        "and Rev 3.0 §6.2.3's Interrupt Status says so"
    );

    // xHCI 1.2 §5.4.2 bit 3, then §5.5.2.3.3, then §4.17.3. Only the third
    // drops the line; a handler that completed its interrupt-controller claim
    // before the third would take a second, spurious trap.
    let _ = d.event();
    d.set_reg(USBSTS, STS_EINT);
    assert_eq!(irr(&m) & IR5, IR5, "EINT is not the pin");
    d.set_reg(ERDP, d.evt_next.get() as u32 | ERDP_EHB);
    assert_eq!(irr(&m) & IR5, IR5, "ERDP.EHB is not the pin");
    d.set_reg(IMAN, IMAN_IP | IMAN_IE);
    assert_eq!(irr(&m) & IR5, 0, "IMAN.IP is");
    assert_eq!(config_read(&m, CFG_COMMAND) & STATUS_INTERRUPT, 0);
}

#[test]
fn interrupt_disable_gates_the_pin_and_not_the_status_bit() {
    let (m, mouse) = ready();
    let (d, _) = drive(&m, &mouse);

    mouse.motion(1, 1, 0);
    let at = d.epint_next.get();
    d.put_trb(
        at,
        [
            REPORT_BUF as u32,
            0,
            REPORT_BYTES,
            (trb::NORMAL << TRB_TYPE_SHIFT) | TRB_IOC | TRB_CYCLE,
        ],
    );
    d.epint_next.set(at + TRB);
    d.set_reg(DB + 4, DCI_INT_IN);
    assert_eq!(irr(&m) & IR5, IR5);

    // Rev 3.0 §6.2.2: the function drives no `INTx#` — and §6.2.3's Interrupt
    // Status still reports the condition, which is what lets a driver poll a
    // masked device at all.
    config_write(&m, CFG_COMMAND, COMMAND_ON | COMMAND_INTX_OFF);
    assert_eq!(irr(&m) & IR5, 0, "the pin is gated");
    assert_eq!(
        config_read(&m, CFG_COMMAND) & STATUS_INTERRUPT,
        STATUS_INTERRUPT,
        "STATUS[3] is the condition, not the output"
    );
    // Nothing about the controller changed, so clearing the bit puts the pin
    // back without the guest touching the interrupter.
    config_write(&m, CFG_COMMAND, COMMAND_ON);
    assert_eq!(irr(&m) & IR5, IR5);
}

#[test]
fn a_debugger_may_read_every_register_and_may_write_none() {
    let (m, mouse) = ready();
    let (d, _) = drive(&m, &mouse);
    let mem = m.space("mem").expect("the memory space");
    let port = m.space("port").expect("the I/O space");

    // Arm a transfer and leave the interrupt asserted, so there is something a
    // read could wrongly acknowledge.
    mouse.motion(2, 2, 0);
    let at = d.epint_next.get();
    d.put_trb(
        at,
        [
            REPORT_BUF as u32,
            0,
            REPORT_BYTES,
            (trb::NORMAL << TRB_TYPE_SHIFT) | TRB_IOC | TRB_CYCLE,
        ],
    );
    d.epint_next.set(at + TRB);
    d.set_reg(DB + 4, DCI_INT_IN);
    assert_eq!(irr(&m) & IR5, IR5);

    // Every configuration dword, twice, read with `MemAttrs::debug` set.
    //
    // The *address* is latched with an ordinary write, because `ConfigPorts`
    // refuses a debug write to `CONFADD` — latching it is itself a change to
    // guest-visible state, and a monitor that wanted a peek without one would
    // have to reach the fabric directly rather than through the ports.
    for _ in 0..2 {
        for register in (0u16..0x40).step_by(4) {
            let addr = 0x8000_0000 | (XHCI_DEVICE << 11) | u32::from(register);
            port.write(0xcf8, Width::U32, u64::from(addr), MemAttrs::DEFAULT)
                .expect("CONFADD");
            port.read(0xcfc, Width::U32, MemAttrs::DEBUG)
                .expect("CONFDATA");
        }
        // And the register block, which has its own rules: a debug read
        // advances nothing.
        for offset in [USBCMD, USBSTS, CRCR, PORTSC1, IMAN, ERDP] {
            mem.read(BAR_BASE + offset, Width::U32, MemAttrs::DEBUG)
                .expect("a mapped dword");
        }
    }
    assert_eq!(irr(&m) & IR5, IR5, "nothing was acknowledged");
    assert_eq!(
        config_read(&m, CFG_COMMAND) & STATUS_INTERRUPT,
        STATUS_INTERRUPT
    );

    // A debug write to configuration space is refused; so is one to the
    // register block, where there is no harmless version of a doorbell.
    let command = config_read(&m, CFG_COMMAND);
    let bar = config_read(&m, CFG_BAR0);
    let addr = 0x8000_0000 | (XHCI_DEVICE << 11) | u32::from(CFG_COMMAND);
    assert!(
        port.write(0xcf8, Width::U32, u64::from(addr), MemAttrs::DEBUG)
            .is_err(),
        "`ConfigPorts` refuses a debug write before one can reach a function"
    );
    port.write(0xcf8, Width::U32, u64::from(addr), MemAttrs::DEFAULT)
        .expect("CONFADD");
    assert!(
        port.write(0xcfc, Width::U32, 0, MemAttrs::DEBUG).is_err(),
        "and refuses the data cycle too"
    );
    assert_eq!(config_read(&m, CFG_COMMAND), command);
    assert_eq!(config_read(&m, CFG_BAR0), bar);
    assert!(
        mem.write(
            BAR_BASE + USBSTS,
            Width::U32,
            u64::from(STS_EINT),
            MemAttrs::DEBUG
        )
        .is_err(),
        "a write-1-to-clear register is not a debugger's to touch"
    );
    assert_eq!(irr(&m) & IR5, IR5);
}

#[test]
fn the_board_snapshots_and_restores_to_the_same_state_hash() {
    let (m, mouse) = ready();
    let (d, _) = drive(&m, &mouse);

    let before = m.state_hash().expect("a hash");
    let bytes = m.save().expect("it snapshots");

    // A second board, brought to the same point the long way round, then
    // loaded: the hash is over the whole machine, so this covers the
    // configuration header, where the window went, the register file, the
    // rings, and the board's RAM at once.
    let (mut other, other_mouse) = ready();
    let (epint_next, evt_next, evt_cycle, events) = {
        let other_d = drive(&other, &other_mouse).0;
        assert_eq!(
            other.state_hash().expect("a hash"),
            before,
            "two identical runs of the same driver must agree before a load says anything"
        );
        (
            other_d.epint_next.get(),
            other_d.evt_next.get(),
            other_d.evt_cycle.get(),
            other_d.events.get(),
        )
    };
    other.load(&bytes).expect("it restores");
    assert_eq!(other.state_hash().expect("a hash"), before);

    // And the restored controller is a working controller: the window is back
    // in the map, the function may master again, and a report still arrives —
    // all three re-derived from the Command register the chunk carried rather
    // than stored a second time.
    let other_d = Driver::new(&other);
    other_d.epint_next.set(epint_next);
    other_d.evt_next.set(evt_next);
    other_d.evt_cycle.set(evt_cycle);
    other_d.events.set(events);
    assert_eq!(other_d.reg(0) & 0xff, CAPLENGTH as u32);
    other_mouse.motion(7, -7, 0);
    assert_eq!(other_d.poll_mouse(1), vec![0, 7, 0xf9]);
    assert_eq!(other_d.events.get(), d.events.get() + 1);
}
