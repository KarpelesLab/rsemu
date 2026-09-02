//! A Serial ATA host bus adapter, driven the way a driver drives one.
//!
//! `tests/nvme_board.rs` and `tests/pc_at_ide.rs` are the standard this is
//! written to: everything here goes through the machine's own address spaces —
//! configuration cycles at `0xcf8`/`0xcfc`, the register block at wherever this
//! test's driver decided to put `ABAR`, and the interrupt controller at `0x20` —
//! and nothing reaches into the device except to check the **medium**, which is
//! the point. A model that never touched a medium would pass a test that
//! compared a read against the device's own buffer; it cannot pass this one.
//!
//! Every offset, bit and structure below is written out of the specification
//! rather than imported from the model, so that the two have to agree:
//!
//! * **Serial ATA AHCI Specification, Revision 1.3.1** (Intel) §2.1, §3.1,
//!   §3.3, §4.2 and §5.5;
//! * **Serial ATA, Revision 1.0** §8.5.2 and §8.5.3 for the two Register FIS
//!   layouts, and §8.5.8 for the PIO Setup FIS.
//!
//! The order is weakest claim first:
//!
//! * the bus shows a class `010601h` function whose `ABAR` sizes as 2 KiB;
//! * `CAP`, `PI` and `VS` describe the adapter the board was actually given, and
//!   the port reports the ATA signature of the drive in its bay;
//! * a **PIO** command — `IDENTIFY DEVICE` — travels a command list, a PRD and a
//!   PIO Setup FIS, and describes the drive the machine file declared;
//! * a **DMA** read arrives from the medium and a DMA write reaches it, both
//!   moved by the adapter into memory this test never handed it directly, with
//!   the neighbouring block asserted untouched;
//! * a transfer scattered over three descriptors lands in all three;
//! * the completion interrupt travels a **wire** into an 8259A, is visible in
//!   its interrupt request register, and drops only when the driver clears
//!   `PxIS` and then `IS` — in that order, which §5.5.3 requires;
//! * a command that fails stops the port the way §6.2.2 says, and the recovery
//!   sequence restarts it;
//! * the AHCI software reset sequence leaves the ATA signature;
//! * a debugger may read every register and may write none;
//! * the board snapshots and restores to an identical state hash.

#![cfg(feature = "machine-ahci-mini")]

use std::sync::Arc;

use rsemu::core::device::ResetKind;
use rsemu::core::space::{MemAttrs, RamStore};
use rsemu::core::value::Width;
use rsemu::dev::ata::Medium;
use rsemu::machine::Machine;

// ---------------------------------------------------------------------------
// the board
// ---------------------------------------------------------------------------

/// Bytes in a sector.
const SECTOR: u64 = 512;

/// How many sectors the drive holds. 4096 of them is 2 MiB, which is what
/// `machines/ahci-mini.machine`'s `disk` parameter defaults to.
const SECTORS: u64 = 4096;

/// What sector `lba` holds on a freshly stamped drive.
///
/// Every sector says which sector it is, so a transfer that lands one out — the
/// classic off-by-one in an LBA computation — fails rather than passing on
/// identical zeroes.
fn stamp(lba: u64) -> Vec<u8> {
    let mut out = vec![0u8; SECTOR as usize];
    for (i, byte) in out.iter_mut().enumerate() {
        *byte = (lba as u8) ^ (i as u8) ^ 0x3c;
    }
    out[0] = lba as u8;
    out[1] = (lba >> 8) as u8;
    out
}

/// The whole drive image.
fn image() -> Vec<u8> {
    let mut out = Vec::with_capacity((SECTORS * SECTOR) as usize);
    for lba in 0..SECTORS {
        out.extend_from_slice(&stamp(lba));
    }
    out
}

/// The board, and the medium the *host* installed under its media slot.
///
/// The medium is kept on this side of the seam deliberately: `--drive
/// sata0=disk.img` installs one exactly like this, and holding a second handle
/// to it is what lets every assertion below check the bytes that reached storage
/// rather than the bytes the adapter says it moved.
fn board() -> (Machine, Arc<RamStore>) {
    let store = Arc::new(RamStore::new(SECTORS * SECTOR));
    RamStore::write_at(&store, 0, &image()).expect("the image fits");

    let mut options = rsemu::machine::catalog::build_options().expect("this build's options");
    rsemu::dev::ata::medium::install(
        &options.realize.hosts,
        "sata0",
        Arc::clone(&store) as Arc<dyn Medium>,
    )
    .expect("nothing else claimed the name");
    // Bound to no bytes: the medium above wins, and this is only how the machine
    // file's `image = "sata0"` finds a slot at all.
    options.realize.media.insert("sata0", Vec::new());

    let registry = rsemu::machine::catalog::registry().expect("this build's registry");
    let entry = &rsemu::machine::catalog::AHCI_MINI;
    let mut machine = match rsemu::machine::build(entry.name, entry.source, &registry, &options) {
        Ok(m) => m,
        Err(e) => panic!("the board does not realize: {e}"),
    };
    machine.reset(ResetKind::Cold);
    machine.sweep();
    (machine, store)
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

fn poke_bytes(m: &Machine, addr: u64, bytes: &[u8]) {
    m.space("mem")
        .expect("the memory space")
        .write_bytes(addr, bytes, MemAttrs::DEFAULT)
        .expect("mapped memory");
}

fn peek_bytes(m: &Machine, addr: u64, len: u64) -> Vec<u8> {
    let mut out = vec![0u8; len as usize];
    m.space("mem")
        .expect("the memory space")
        .read_bytes(addr, &mut out, MemAttrs::DEFAULT)
        .expect("mapped memory");
    out
}

/// What the medium itself holds at `lba` — the only assertion that proves a
/// transfer happened rather than a buffer being echoed back.
fn on_medium(store: &RamStore, lba: u64) -> Vec<u8> {
    let mut got = vec![0u8; SECTOR as usize];
    Medium::read_at(store, lba * SECTOR, &mut got).expect("the medium reads");
    got
}

// ---------------------------------------------------------------------------
// configuration space, mechanism #1
// ---------------------------------------------------------------------------

/// Where the adapter sits: bus 0, device 5, function 0, as the machine file
/// says.
const AHCI_DEVICE: u32 = 5;

fn config_read(m: &Machine, register: u16) -> u32 {
    let addr = 0x8000_0000 | (AHCI_DEVICE << 11) | u32::from(register & 0xfc);
    m.space("port")
        .expect("the I/O space")
        .write(0xcf8, Width::U32, u64::from(addr), MemAttrs::DEFAULT)
        .expect("CONFADD takes a dword");
    m.space("port")
        .expect("the I/O space")
        .read(0xcfc, Width::U32, MemAttrs::DEFAULT)
        .expect("CONFDATA answers") as u32
}

fn config_write(m: &Machine, register: u16, value: u32) {
    let addr = 0x8000_0000 | (AHCI_DEVICE << 11) | u32::from(register & 0xfc);
    m.space("port")
        .expect("the I/O space")
        .write(0xcf8, Width::U32, u64::from(addr), MemAttrs::DEFAULT)
        .expect("CONFADD takes a dword");
    m.space("port")
        .expect("the I/O space")
        .write(0xcfc, Width::U32, u64::from(value), MemAttrs::DEFAULT)
        .expect("CONFDATA takes a dword");
}

const CFG_VENDOR: u16 = 0x00;
const CFG_COMMAND: u16 = 0x04;
const CFG_CLASS: u16 = 0x08;
/// AHCI §2.1.11: `ABAR` is at configuration offset `24h`, which is base address
/// register five.
const CFG_ABAR: u16 = 0x24;
const CFG_INT_PIN: u16 = 0x3c;

/// `COMMAND[1]` and `COMMAND[2]`: memory space and bus master.
const COMMAND_ON: u32 = 0x0006;

// ---------------------------------------------------------------------------
// the register block (§3.1, §3.3)
// ---------------------------------------------------------------------------

/// Where this test's driver puts the 2 KiB register window. Above the board's
/// 16 MiB of RAM, which is where firmware allocates from.
const ABAR_BASE: u64 = 0xf000_0000;

const REG_CAP: u64 = 0x00;
const REG_GHC: u64 = 0x04;
const REG_IS: u64 = 0x08;
const REG_PI: u64 = 0x0c;
const REG_VS: u64 = 0x10;

/// `GHC.IE`, the global interrupt enable.
const GHC_IE: u32 = 1 << 1;
/// `GHC.AE`, which `CAP.SAM` makes read-only one.
const GHC_AE: u32 = 1 << 31;

/// Port 0's registers: `100h + n * 80h` (§3.3).
fn port(offset: u64) -> u64 {
    0x100 + offset
}

const P_CLB: u64 = 0x00;
const P_CLBU: u64 = 0x04;
const P_FB: u64 = 0x08;
const P_FBU: u64 = 0x0c;
const P_IS: u64 = 0x10;
const P_IE: u64 = 0x14;
const P_CMD: u64 = 0x18;
const P_TFD: u64 = 0x20;
const P_SIG: u64 = 0x24;
const P_SSTS: u64 = 0x28;
const P_SCTL: u64 = 0x2c;
const P_SERR: u64 = 0x30;
const P_CI: u64 = 0x38;

const CMD_ST: u32 = 1 << 0;
const CMD_CLO: u32 = 1 << 3;
const CMD_FRE: u32 = 1 << 4;
const CMD_FR: u32 = 1 << 14;
const CMD_CR: u32 = 1 << 15;

const IS_DHRS: u32 = 1 << 0;
const IS_PSS: u32 = 1 << 1;
const IS_DPS: u32 = 1 << 5;
const IS_OFS: u32 = 1 << 24;
const IS_TFES: u32 = 1 << 30;

/// `PxTFD.STS.ERR` and `PxTFD.STS.BSY`.
const STS_ERR: u32 = 1 << 0;
const STS_BSY: u32 = 1 << 7;

// ---------------------------------------------------------------------------
// the structures a driver builds in its own memory (§4.2)
// ---------------------------------------------------------------------------

/// Where the command list goes. §3.3.1: 1 KiB aligned.
const CLB: u64 = 0x0010_0000;
/// Where the received FIS structure goes. §3.3.3: 256-byte aligned.
const FB: u64 = 0x0010_1000;
/// Where the command table goes. §4.2.2: 128-byte aligned.
const CTBA: u64 = 0x0010_2000;
/// Where data buffers go.
const DATA: u64 = 0x0020_0000;

/// The PRDT starts 128 bytes into a command table (§4.2.3).
const PRDT_AT: u64 = 0x80;

/// Where the PIO Setup FIS and the D2H Register FIS are posted (§4.2.1).
const PSFIS_AT: u64 = 0x20;
const RFIS_AT: u64 = 0x40;

/// One ATA command, as a driver assembles a Register - Host to Device FIS
/// (Serial ATA §8.5.2, Figure 58).
#[derive(Debug, Clone, Copy, Default)]
struct Cmd {
    command: u8,
    feature: u16,
    count: u16,
    lba: u64,
    device: u8,
}

impl Cmd {
    fn fis(&self) -> [u8; 20] {
        let mut f = [0u8; 20];
        f[0] = 0x27; // FIS Type: Register - Host to Device
        f[1] = 0x80; // C: this transfer is due to a write of the Command register
        f[2] = self.command;
        f[3] = self.feature as u8;
        f[4] = self.lba as u8; // Sector Number
        f[5] = (self.lba >> 8) as u8; // Cyl Low
        f[6] = (self.lba >> 16) as u8; // Cyl High
        f[7] = self.device; // Dev / Head
        f[8] = (self.lba >> 24) as u8; // Sector Num (exp)
        f[9] = (self.lba >> 32) as u8; // Cyl Low (exp)
        f[10] = (self.lba >> 40) as u8; // Cyl High (exp)
        f[11] = (self.feature >> 8) as u8; // Features (exp)
        f[12] = self.count as u8; // Sector Count
        f[13] = (self.count >> 8) as u8; // Sector Count (exp)
        f
    }
}

/// One Physical Region Descriptor (§4.2.3.3).
///
/// `DBC` is a **zero-based** byte count whose bit 0 must be one, which is the
/// same statement as "an even number of bytes".
fn prd(addr: u64, bytes: u64, interrupt: bool) -> [u8; 16] {
    assert!(
        bytes > 0 && bytes.is_multiple_of(2),
        "a PRD is an even length"
    );
    let mut d = [0u8; 16];
    d[0..4].copy_from_slice(&(addr as u32).to_le_bytes());
    d[4..8].copy_from_slice(&((addr >> 32) as u32).to_le_bytes());
    let dw3 = (bytes as u32 - 1) | if interrupt { 1 << 31 } else { 0 };
    d[12..16].copy_from_slice(&dw3.to_le_bytes());
    d
}

/// Build a command in slot 0 and issue it, without waiting: this adapter
/// completes inside the `PxCI` write, so there is nothing to wait for.
///
/// Returns the byte count the adapter wrote back into the command header's
/// `PRDBC` field (§5.4.1).
fn issue(m: &Machine, cmd: Cmd, write: bool, prds: &[[u8; 16]]) -> u32 {
    // §4.2.3: the command FIS goes at the front of the command table, the PRD
    // table 128 bytes in.
    poke_bytes(m, CTBA, &cmd.fis());
    for (i, entry) in prds.iter().enumerate() {
        poke_bytes(m, CTBA + PRDT_AT + i as u64 * 16, entry);
    }
    // §4.2.2, Figure 8: PRDTL in 31:16, W at bit 6, CFL — the command FIS length
    // in dwords — in 4:0. A Register FIS is five dwords.
    let dw0 = 5 | (u32::from(write) << 6) | ((prds.len() as u32) << 16);
    let mut header = [0u8; 32];
    header[0..4].copy_from_slice(&dw0.to_le_bytes());
    header[8..12].copy_from_slice(&(CTBA as u32).to_le_bytes());
    header[12..16].copy_from_slice(&((CTBA >> 32) as u32).to_le_bytes());
    poke_bytes(m, CLB, &header);

    // §3.3.14: write-1-to-set, and only while `ST` is set.
    poke32(m, ABAR_BASE + port(P_CI), 1);
    peek32(m, CLB + 4)
}

fn reg32(m: &Machine, offset: u64) -> u32 {
    peek32(m, ABAR_BASE + offset)
}

fn set_reg32(m: &Machine, offset: u64, value: u32) {
    poke32(m, ABAR_BASE + offset, value);
}

// ---------------------------------------------------------------------------
// bringing the adapter up, as a driver does
// ---------------------------------------------------------------------------

/// Enumerate, place `ABAR`, and switch the function on.
fn place_abar(m: &Machine) {
    // §6.2.5.1's sizing protocol: all ones in, and the window's size out.
    config_write(m, CFG_ABAR, 0xffff_ffff);
    config_write(m, CFG_ABAR, ABAR_BASE as u32);
    config_write(m, CFG_COMMAND, COMMAND_ON);
}

/// The 8259A, initialised the way a driver of a level-triggered PCI interrupt
/// has to initialise one.
fn init_pic(m: &Machine) {
    outb(m, 0x20, 0x11); // ICW1: cascade, ICW4 to follow
    outb(m, 0x21, 0x08); // ICW2: vectors from 0x08
    outb(m, 0x21, 0x04); // ICW3: a slave would be on IR2
    outb(m, 0x21, 0x01); // ICW4: 8086 mode
    outb(m, 0x21, 0xdf); // OCW1: everything masked but IR5
    // `INTx#` is a level, so IR5 has to be level triggered: an edge-triggered
    // input would latch the first completion and miss every later one raised
    // while the line was already low.
    outb(m, 0x4d0, 1 << 5);
}

/// The interrupt request register, which for a level-triggered line is the line
/// itself.
fn irr(m: &Machine) -> u8 {
    outb(m, 0x20, 0x0a); // OCW3: the next read of port 0 is the IRR
    inb(m, 0x20)
}

/// §5.5.3: clear the port's status bits first, then the global one. The other
/// order leaves the line asserted, because the adapter re-derives `IS` from the
/// ports — which is exactly what real hardware does and what this order exists
/// to avoid.
fn acknowledge(m: &Machine) {
    let pending = reg32(m, port(P_IS));
    set_reg32(m, port(P_IS), pending);
    set_reg32(m, REG_IS, 1);
}

/// Point the port at the structures this test built, turn the FIS receive
/// engine on, and start the command list engine (§10.1.2's order: `FRE` before
/// `ST`).
fn start_port(m: &Machine) {
    set_reg32(m, port(P_CLB), CLB as u32);
    set_reg32(m, port(P_CLBU), (CLB >> 32) as u32);
    set_reg32(m, port(P_FB), FB as u32);
    set_reg32(m, port(P_FBU), (FB >> 32) as u32);
    // Everything the adapter can raise, so a missing interrupt shows up as a
    // missing interrupt rather than as a disabled one.
    set_reg32(m, port(P_IE), 0xffff_ffff);
    set_reg32(m, port(P_CMD), CMD_FRE);
    assert_eq!(
        reg32(m, port(P_CMD)) & CMD_FR,
        CMD_FR,
        "the FIS receive engine did not start"
    );
    set_reg32(m, port(P_CMD), CMD_FRE | CMD_ST);
    assert_eq!(
        reg32(m, port(P_CMD)) & CMD_CR,
        CMD_CR,
        "the command list engine did not start"
    );
    set_reg32(m, REG_GHC, GHC_AE | GHC_IE);
}

/// A board with the adapter placed, its port started and its interrupt enabled
/// — everything a driver does before its first command.
fn ready() -> (Machine, Arc<RamStore>) {
    let (m, store) = board();
    init_pic(&m);
    place_abar(&m);
    start_port(&m);
    (m, store)
}

// ---------------------------------------------------------------------------
// the tests
// ---------------------------------------------------------------------------

#[test]
fn the_bus_shows_a_serial_ata_controller_whose_abar_sizes_as_two_kibibytes() {
    // The weakest claim, and the one everything else rests on: a driver finds
    // this device by class code and reaches it through a window it sizes.
    let (m, _store) = board();

    assert_eq!(
        config_read(&m, CFG_VENDOR) & 0xffff,
        0x1234,
        "the vendor the machine file declares"
    );
    // AHCI §2.1.5: base class 01h mass storage, sub class 06h Serial ATA,
    // programming interface 01h an AHCI HBA of major revision one.
    assert_eq!(
        config_read(&m, CFG_CLASS) >> 8,
        0x0001_0601,
        "the class code an AHCI driver enumerates for"
    );
    assert_eq!(
        config_read(&m, CFG_INT_PIN) >> 8 & 0xff,
        0x01,
        "the function drives INTA#"
    );

    // Out of reset the function decodes nothing, so the window is not there.
    assert_eq!(
        config_read(&m, CFG_COMMAND) & 0xffff,
        0,
        "every enable bit is clear until firmware sets one"
    );
    assert_eq!(
        m.space("mem")
            .expect("the memory space")
            .read(ABAR_BASE, Width::U32, MemAttrs::DEFAULT)
            .expect("read-as-ones"),
        0xffff_ffff,
        "nothing decodes at the base until ABAR is placed and enabled"
    );

    // §2.1.11 makes `ABAR` a 32-bit non-prefetchable memory window: bits 03:00
    // read back zero, so a 2 KiB window sizes as `fffff800h`.
    config_write(&m, CFG_ABAR, 0xffff_ffff);
    assert_eq!(config_read(&m, CFG_ABAR), 0xffff_f800);
    // And the first five base address registers are not implemented — this
    // adapter has no legacy task-file interface to decode one for.
    for register in [0x10u16, 0x14, 0x18, 0x1c, 0x20] {
        config_write(&m, register, 0xffff_ffff);
        assert_eq!(
            config_read(&m, register),
            0,
            "BAR at {register:#x} should not be implemented"
        );
    }

    place_abar(&m);
    assert_eq!(config_read(&m, CFG_ABAR), ABAR_BASE as u32);
    assert_eq!(reg32(&m, REG_VS), 0x0001_0301, "AHCI 1.3.1 (§3.1.5.6)");
}

#[test]
fn the_capabilities_describe_the_adapter_this_board_was_given() {
    let (m, _store) = board();
    place_abar(&m);

    let cap = reg32(&m, REG_CAP);
    // §3.1.1, and every one of these is zero-based or a flag with a
    // consequence a driver acts on.
    assert_eq!(cap & 0x1f, 0, "CAP.NP: one port, zero based");
    assert_eq!(
        (cap >> 8) & 0x1f,
        31,
        "CAP.NCS: 32 command slots, zero based"
    );
    assert_eq!(cap & (1 << 31), 1 << 31, "CAP.S64A: 64-bit data structures");
    assert_eq!(cap & (1 << 30), 0, "CAP.SNCQ: no native command queuing");
    assert_eq!(cap & (1 << 24), 1 << 24, "CAP.SCLO: command list override");
    assert_eq!(
        cap & (1 << 18),
        1 << 18,
        "CAP.SAM: AHCI only, no legacy mode"
    );
    assert_eq!(cap & (1 << 17), 0, "CAP.SPM: no port multiplier");
    assert_eq!(cap & (1 << 7), 0, "CAP.CCCS: no completion coalescing");
    assert_eq!(cap & (1 << 6), 0, "CAP.EMS: no enclosure management");

    assert_eq!(reg32(&m, REG_PI), 1, "PI: port 0 and no other");
    // §3.1.2: with `CAP.SAM` set, `GHC.AE` is read-only one.
    assert_eq!(reg32(&m, REG_GHC) & GHC_AE, GHC_AE);
    set_reg32(&m, REG_GHC, 0);
    assert_eq!(
        reg32(&m, REG_GHC) & GHC_AE,
        GHC_AE,
        "GHC.AE is read-only when CAP.SAM is set"
    );
}

#[test]
fn a_port_with_a_drive_reports_the_ata_signature() {
    let (m, _store) = board();
    place_abar(&m);

    // §3.3.10: `DET` 3h presence and communication, `SPD` 3h Gen 3, `IPM` 1h
    // active.
    assert_eq!(reg32(&m, port(P_SSTS)), 0x0000_0133);
    // §3.3.9: the signature packs LBA high, LBA mid, LBA low and sector count
    // into 31:24, 23:16, 15:8 and 7:0. An ATA device leaves 0/0/1/1 after a
    // reset, which is `00000101h`; a packet device would leave `eb140101h`, and
    // that is how a driver tells them apart.
    assert_eq!(reg32(&m, port(P_SIG)), 0x0000_0101);
    // §3.3.8: `PxTFD` holds the error register in 15:8 and the status in 7:0.
    // A drive that has just passed its power-on diagnostic answers 01h and
    // `DRDY | DSC`.
    assert_eq!(reg32(&m, port(P_TFD)), 0x0000_0150);
    // Nothing has started, so neither engine is running.
    assert_eq!(reg32(&m, port(P_CMD)) & (CMD_CR | CMD_FR), 0);
}

#[test]
fn identify_device_travels_a_pio_setup_fis_and_describes_this_drive() {
    // The PIO protocol end to end. `IDENTIFY DEVICE` is a PIO data-in command,
    // so §5.6.3 has the device send a PIO Setup FIS before the data and end the
    // command by latching that FIS's `E_Status` — which is why the interrupt a
    // driver waits on here is `PxIS.PSS` and not `PxIS.DHRS`.
    let (m, _store) = ready();

    let moved = issue(
        &m,
        Cmd {
            command: 0xec, // IDENTIFY DEVICE
            ..Cmd::default()
        },
        false,
        &[prd(DATA, 512, true)],
    );
    assert_eq!(moved, 512, "PRDBC: one 512-byte block moved (§5.4.1)");
    assert_eq!(reg32(&m, port(P_CI)), 0, "the slot completed");

    let is = reg32(&m, port(P_IS));
    assert_eq!(is & IS_PSS, IS_PSS, "a PIO command ends on PxIS.PSS");
    assert_eq!(is & IS_DHRS, 0, "and not on a D2H Register FIS");
    assert_eq!(is & IS_DPS, IS_DPS, "the PRD asked for an interrupt");
    assert_eq!(reg32(&m, port(P_TFD)) & 0xff, 0x50, "DRDY | DSC, not busy");

    // Serial ATA §8.5.8: the PIO Setup FIS is posted at offset 20h of the
    // received FIS structure. Its `Status` is what the host is to see while the
    // block moves — `DRQ` set — and its `E_Status` is what to latch when the
    // last byte has gone.
    let psfis = peek_bytes(&m, FB + PSFIS_AT, 20);
    assert_eq!(psfis[0], 0x5f, "FIS Type: PIO Setup - Device to Host");
    assert_eq!(
        psfis[1] & (1 << 5),
        1 << 5,
        "D: the device is writing memory"
    );
    assert_eq!(
        psfis[1] & (1 << 6),
        1 << 6,
        "I: the device's interrupt line"
    );
    assert_eq!(psfis[2], 0x58, "Status: DRDY | DSC | DRQ while data moves");
    assert_eq!(psfis[15], 0x50, "E_Status: DRQ gone when it has");
    assert_eq!(
        u16::from_le_bytes([psfis[16], psfis[17]]),
        512,
        "Transfer Count"
    );

    // And the 256 words themselves describe the drive the machine file
    // declared. T13 ATA/ATAPI-6: words 27-46 are the model string, with each
    // word's first character in its high byte.
    let id = peek_bytes(&m, DATA, 512);
    let model: String = id[54..94]
        .chunks(2)
        .flat_map(|w| [w[1] as char, w[0] as char])
        .collect();
    assert_eq!(model.trim_end(), "RSEMU HARDDISK");
    // Word 49 bit 8: DMA supported. The machine file says `dma = true`, and
    // that is what makes the next test's command legal.
    let word49 = u16::from_le_bytes([id[98], id[99]]);
    assert_eq!(word49 & (1 << 8), 1 << 8, "word 49: DMA supported");
    // Words 100-103: the 48-bit capacity, which has to be the drive the host
    // installed rather than the machine file's `size`.
    let capacity = u64::from(u16::from_le_bytes([id[200], id[201]]))
        | (u64::from(u16::from_le_bytes([id[202], id[203]])) << 16);
    assert_eq!(capacity, SECTORS);
}

#[test]
fn a_sector_arrives_from_the_medium_through_a_dma_command() {
    // The DMA protocol end to end, and the assertion that matters: the bytes
    // come from the **medium**, which this test holds a handle to and never
    // handed the adapter.
    let (m, store) = ready();
    const LBA: u64 = 1234;

    // Poison the destination first, so a model that moved nothing fails.
    poke_bytes(&m, DATA, &vec![0x5au8; SECTOR as usize]);
    let moved = issue(
        &m,
        Cmd {
            command: 0x25, // READ DMA EXT
            count: 1,
            lba: LBA,
            device: 0x40, // LBA addressing
            ..Cmd::default()
        },
        false,
        &[prd(DATA, SECTOR, false)],
    );
    assert_eq!(moved, SECTOR as u32, "PRDBC");
    assert_eq!(reg32(&m, port(P_CI)), 0, "the slot completed");

    let is = reg32(&m, port(P_IS));
    assert_eq!(is & IS_DHRS, IS_DHRS, "a DMA command ends on a D2H FIS");
    assert_eq!(is & IS_PSS, 0, "and posts no PIO Setup FIS");

    // Serial ATA §8.5.3: the D2H Register FIS is posted at offset 40h.
    let rfis = peek_bytes(&m, FB + RFIS_AT, 20);
    assert_eq!(rfis[0], 0x34, "FIS Type: Register - Device to Host");
    assert_eq!(rfis[1] & (1 << 6), 1 << 6, "I");
    assert_eq!(rfis[2], 0x50, "Status: DRDY | DSC");
    assert_eq!(rfis[3], 0x00, "Error: none");

    assert_eq!(
        peek_bytes(&m, DATA, SECTOR),
        on_medium(&store, LBA),
        "the sector in memory is the sector on the medium"
    );
    assert_eq!(peek_bytes(&m, DATA, SECTOR), stamp(LBA));
}

#[test]
fn a_sector_written_through_a_prdt_reaches_the_medium() {
    let (m, store) = ready();
    const LBA: u64 = 2000;
    let payload: Vec<u8> = (0..SECTOR as usize).map(|i| (i as u8) ^ 0xa5).collect();
    poke_bytes(&m, DATA, &payload);

    let moved = issue(
        &m,
        Cmd {
            command: 0x35, // WRITE DMA EXT
            count: 1,
            lba: LBA,
            device: 0x40,
            ..Cmd::default()
        },
        true,
        &[prd(DATA, SECTOR, false)],
    );
    assert_eq!(moved, SECTOR as u32, "PRDBC");
    assert_eq!(reg32(&m, port(P_IS)) & IS_TFES, 0, "no taskfile error");
    assert_eq!(reg32(&m, port(P_CI)), 0);

    // The medium, not the adapter's own buffer.
    assert_eq!(
        on_medium(&store, LBA),
        payload,
        "the write reached the disk"
    );
    // And the sector next door is exactly what it was, which is what catches an
    // off-by-one in the LBA arithmetic.
    assert_eq!(on_medium(&store, LBA + 1), stamp(LBA + 1));
    assert_eq!(on_medium(&store, LBA - 1), stamp(LBA - 1));
}

#[test]
fn a_transfer_scattered_over_three_descriptors_lands_in_all_three() {
    // The whole point of a PRD table: the destination is not contiguous, and the
    // descriptors do not align to sector boundaries either.
    let (m, store) = ready();
    const LBA: u64 = 64;
    const BLOCKS: u64 = 4;

    let a = DATA;
    let b = DATA + 0x1000;
    let c = DATA + 0x8000;
    // 600 + 1000 + 448 = 2048, and not one of them is a sector.
    let parts: [(u64, u64); 3] = [(a, 600), (b, 1000), (c, 448)];
    for (addr, len) in parts {
        poke_bytes(&m, addr, &vec![0xeeu8; len as usize]);
    }

    let moved = issue(
        &m,
        Cmd {
            command: 0x25,
            count: BLOCKS as u16,
            lba: LBA,
            device: 0x40,
            ..Cmd::default()
        },
        false,
        &[prd(a, 600, false), prd(b, 1000, false), prd(c, 448, true)],
    );
    assert_eq!(moved, (BLOCKS * SECTOR) as u32, "PRDBC over three PRDs");

    let mut got = Vec::new();
    for (addr, len) in parts {
        got.extend_from_slice(&peek_bytes(&m, addr, len));
    }
    let mut want = Vec::new();
    for lba in LBA..LBA + BLOCKS {
        want.extend_from_slice(&on_medium(&store, lba));
    }
    assert_eq!(
        got, want,
        "the four sectors, split across three descriptors"
    );
    assert_eq!(
        reg32(&m, port(P_IS)) & IS_DPS,
        IS_DPS,
        "the last descriptor asked for an interrupt"
    );
}

#[test]
fn a_prd_table_too_short_reports_overflow_without_stopping_the_port() {
    // §6.1.5 is an overflow and §6.2.2 is explicit that `OFS` is *not* fatal:
    // "the HBA continues to operate". So the command completes short, the bit
    // is raised, and the next command still runs.
    let (m, store) = ready();
    let moved = issue(
        &m,
        Cmd {
            command: 0x25,
            count: 4,
            lba: 10,
            device: 0x40,
            ..Cmd::default()
        },
        false,
        // Room for one sector of the four asked for.
        &[prd(DATA, SECTOR, false)],
    );
    assert_eq!(moved, SECTOR as u32, "PRDBC is what actually transferred");
    assert_eq!(reg32(&m, port(P_IS)) & IS_OFS, IS_OFS, "PxIS.OFS");
    assert_eq!(
        reg32(&m, port(P_CMD)) & CMD_CR,
        CMD_CR,
        "an overflow is not fatal, so the engine keeps running"
    );
    assert_eq!(reg32(&m, port(P_CI)), 0, "and the slot still completed");
    acknowledge(&m);

    // The drive is at rest, not half way through a read, and the next command
    // gets the sector it asked for.
    let moved = issue(
        &m,
        Cmd {
            command: 0x25,
            count: 1,
            lba: 77,
            device: 0x40,
            ..Cmd::default()
        },
        false,
        &[prd(DATA, SECTOR, false)],
    );
    assert_eq!(moved, SECTOR as u32);
    assert_eq!(peek_bytes(&m, DATA, SECTOR), on_medium(&store, 77));
}

#[test]
fn a_completion_interrupt_reaches_the_8259a_and_needs_both_registers_cleared() {
    let (m, _store) = ready();
    assert_eq!(irr(&m) & (1 << 5), 0, "nothing is pending yet");

    issue(
        &m,
        Cmd {
            command: 0x25,
            count: 1,
            lba: 3,
            device: 0x40,
            ..Cmd::default()
        },
        false,
        &[prd(DATA, SECTOR, false)],
    );
    assert_eq!(reg32(&m, REG_IS) & 1, 1, "IS.IPS[0]");
    assert_eq!(irr(&m) & (1 << 5), 1 << 5, "INTx# reached IR5");

    // §5.5.3's order matters, and this is the half that catches a model which
    // treats `IS` as ordinary storage: clearing the *global* register first
    // achieves nothing, because the port still has an enabled bit pending and
    // the adapter sets `IS` again.
    set_reg32(&m, REG_IS, 1);
    assert_eq!(
        irr(&m) & (1 << 5),
        1 << 5,
        "clearing IS before PxIS must not drop the line"
    );

    // The right order.
    let pending = reg32(&m, port(P_IS));
    assert_eq!(pending & IS_DHRS, IS_DHRS);
    set_reg32(&m, port(P_IS), pending);
    assert_eq!(
        irr(&m) & (1 << 5),
        1 << 5,
        "IS is write-1-to-clear and has not been written yet"
    );
    set_reg32(&m, REG_IS, 1);
    assert_eq!(reg32(&m, REG_IS), 0);
    assert_eq!(irr(&m) & (1 << 5), 0, "and now the line drops");

    // `GHC.IE` gates the pin and nothing else: the status bits stay where they
    // are, which is what lets a driver poll.
    issue(
        &m,
        Cmd {
            command: 0x25,
            count: 1,
            lba: 4,
            device: 0x40,
            ..Cmd::default()
        },
        false,
        &[prd(DATA, SECTOR, false)],
    );
    assert_eq!(irr(&m) & (1 << 5), 1 << 5);
    set_reg32(&m, REG_GHC, GHC_AE);
    assert_eq!(irr(&m) & (1 << 5), 0, "GHC.IE clear holds the pin off");
    assert_eq!(reg32(&m, REG_IS) & 1, 1, "the status is still there");
}

#[test]
fn a_command_that_fails_stops_the_port_and_the_recovery_sequence_restarts_it() {
    // §6.1.4 and §6.2.2, walked end to end.
    let (m, store) = ready();

    issue(
        &m,
        Cmd {
            command: 0x25,
            count: 1,
            // One past the last sector the drive holds.
            lba: SECTORS,
            device: 0x40,
            ..Cmd::default()
        },
        false,
        &[prd(DATA, SECTOR, false)],
    );

    assert_eq!(reg32(&m, port(P_IS)) & IS_TFES, IS_TFES, "PxIS.TFES");
    let tfd = reg32(&m, port(P_TFD));
    assert_eq!(tfd & STS_ERR, STS_ERR, "PxTFD.STS.ERR");
    assert_eq!(tfd & STS_BSY, 0, "and the device is not busy");
    // T13 ATA/ATAPI-6: the Error register's IDNF bit — the address the command
    // named is not one this drive has.
    assert_eq!((tfd >> 8) & 0xff, 0x10, "PxTFD.ERR: IDNF");
    // §6.2.2: a fatal error clears `PxCMD.CR` and leaves `PxCMD.ST` alone.
    assert_eq!(reg32(&m, port(P_CMD)) & CMD_CR, 0, "the engine stopped");
    assert_eq!(reg32(&m, port(P_CMD)) & CMD_ST, CMD_ST, "ST is software's");
    // And `PxCI` keeps the slot, so software can see which command failed.
    assert_eq!(reg32(&m, port(P_CI)), 1);
    assert_eq!(irr(&m) & (1 << 5), 1 << 5, "the error interrupted");

    // §3.3.14 gates `PxCI` on `PxCMD.ST`, which is software's, and not on
    // `PxCMD.CR`, which is the adapter's — so a slot issued into a port that
    // is in `ERR:Fatal` still *latches*. It simply never runs, and §6.2.2.1's
    // first step is to read `PxCI` and find exactly that.
    poke32(&m, ABAR_BASE + port(P_CI), 1 << 4);
    assert_eq!(reg32(&m, port(P_CI)), 0b1_0001, "the new slot latched");

    // §6.2.2.1's recovery, in its order.
    set_reg32(&m, port(P_CMD), CMD_FRE); // ST 1 -> 0 resets PxCI
    assert_eq!(reg32(&m, port(P_CMD)) & CMD_CR, 0, "CR is clear");
    assert_eq!(reg32(&m, port(P_CI)), 0, "PxCI was reset");
    set_reg32(&m, port(P_SERR), 0xffff_ffff);
    acknowledge(&m);
    assert_eq!(irr(&m) & (1 << 5), 0);
    set_reg32(&m, port(P_CMD), CMD_FRE | CMD_ST);
    assert_eq!(reg32(&m, port(P_CMD)) & CMD_CR, CMD_CR, "and it restarts");

    // A restarted port is a working port, and the stale slot did not re-run.
    let moved = issue(
        &m,
        Cmd {
            command: 0x25,
            count: 1,
            lba: 9,
            device: 0x40,
            ..Cmd::default()
        },
        false,
        &[prd(DATA, SECTOR, false)],
    );
    assert_eq!(moved, SECTOR as u32);
    assert_eq!(peek_bytes(&m, DATA, SECTOR), on_medium(&store, 9));
}

#[test]
fn the_software_reset_sequence_leaves_the_ata_signature() {
    // AHCI §10.4.1: two command FISes with `C` clear — Device Control writes
    // rather than commands. The first asserts SRST and the device goes silent,
    // so the command header's `C` bit is what clears `PxCI`; the second releases
    // it and the device answers with its signature.
    let (m, _store) = ready();

    // Move the drive off its reset state first, so "the signature came back"
    // means something: `INITIALIZE DEVICE PARAMETERS` changes the current
    // translation, and a software reset is specified to keep it.
    issue(
        &m,
        Cmd {
            command: 0x91,
            count: 32,
            device: 0x0f,
            ..Cmd::default()
        },
        false,
        &[],
    );
    acknowledge(&m);

    // §10.4.1 step one. `C` clear in the FIS, `C` set in the header.
    let mut fis = Cmd {
        command: 0,
        ..Cmd::default()
    }
    .fis();
    fis[1] = 0x00; // not a command: a Device Control transfer
    fis[15] = 0x04 | 0x02; // Control: SRST, nIEN
    poke_bytes(&m, CTBA, &fis);
    let mut header = [0u8; 32];
    // CFL 5, and bit 10 — `C`, clear busy upon R_OK — so the adapter clears the
    // slot for a device that will not answer while it is held in reset.
    header[0..4].copy_from_slice(&(5u32 | (1 << 10)).to_le_bytes());
    header[8..12].copy_from_slice(&(CTBA as u32).to_le_bytes());
    poke_bytes(&m, CLB, &header);
    poke32(&m, ABAR_BASE + port(P_CI), 1);
    assert_eq!(reg32(&m, port(P_CI)), 0, "the header's C bit cleared it");

    // §10.4.1 step two: SRST released, `C` clear in the header this time,
    // because the device does answer.
    fis[15] = 0x00;
    poke_bytes(&m, CTBA, &fis);
    header[0..4].copy_from_slice(&5u32.to_le_bytes());
    poke_bytes(&m, CLB, &header);
    poke32(&m, ABAR_BASE + port(P_CI), 1);
    assert_eq!(reg32(&m, port(P_CI)), 0, "the device's D2H FIS cleared it");

    // The signature, in the shadow taskfile and in the FIS that carried it.
    assert_eq!(reg32(&m, port(P_TFD)), 0x0000_0150);
    let rfis = peek_bytes(&m, FB + RFIS_AT, 20);
    assert_eq!(rfis[0], 0x34);
    assert_eq!(rfis[2], 0x50, "Status");
    assert_eq!(rfis[3], 0x01, "Error: diagnostic passed");
    assert_eq!(rfis[12], 0x01, "Sector Count");
    assert_eq!(rfis[4], 0x01, "Sector Number");
    assert_eq!(rfis[5], 0x00, "Cyl Low: an ATA device, not a packet one");
    assert_eq!(rfis[6], 0x00, "Cyl High");

    // And the port is still usable afterwards.
    let moved = issue(
        &m,
        Cmd {
            command: 0x25,
            count: 1,
            lba: 5,
            device: 0x40,
            ..Cmd::default()
        },
        false,
        &[prd(DATA, SECTOR, false)],
    );
    assert_eq!(moved, SECTOR as u32);
}

#[test]
fn a_comreset_resets_the_drive_and_reports_the_change() {
    let (m, _store) = ready();
    // §3.3.11: `DET` may only be modified while `PxCMD.ST` is zero.
    set_reg32(&m, port(P_CMD), CMD_FRE);
    set_reg32(&m, port(P_SCTL), 1);
    assert_eq!(
        reg32(&m, port(P_SSTS)) & 0xf,
        1,
        "presence, no communication"
    );
    set_reg32(&m, port(P_SCTL), 0);
    assert_eq!(reg32(&m, port(P_SSTS)), 0x0000_0133, "the link is back");
    assert_eq!(
        reg32(&m, port(P_SERR)) & (1 << 26),
        1 << 26,
        "PxSERR.DIAG.X: a change in device presence"
    );
    assert_eq!(
        reg32(&m, port(P_IS)) & (1 << 6),
        1 << 6,
        "PxIS.PCS reflects it"
    );
    assert_eq!(
        reg32(&m, port(P_TFD)),
        0x0000_0150,
        "the signature came back"
    );

    // §3.3.5: `PCS` is read-only and is only cleared by clearing `DIAG.X`.
    set_reg32(&m, port(P_IS), 1 << 6);
    assert_eq!(reg32(&m, port(P_IS)) & (1 << 6), 1 << 6);
    set_reg32(&m, port(P_SERR), 1 << 26);
    assert_eq!(reg32(&m, port(P_IS)) & (1 << 6), 0);
}

#[test]
fn command_list_override_clears_the_shadow_busy_bit() {
    // §3.3.7: `CLO` exists so that a software reset can be sent to a device
    // whose `BSY` or `DRQ` is still set, and hardware clears the bit once it
    // has done it. `CAP.SCLO` says this adapter has it.
    let (m, _store) = ready();
    set_reg32(&m, port(P_CMD), CMD_FRE | CMD_ST | CMD_CLO);
    assert_eq!(reg32(&m, port(P_CMD)) & CMD_CLO, 0, "hardware cleared CLO");
    assert_eq!(reg32(&m, port(P_TFD)) & (STS_BSY | (1 << 3)), 0);
}

#[test]
fn a_debugger_may_read_every_register_and_may_write_none() {
    let (m, _store) = ready();
    issue(
        &m,
        Cmd {
            command: 0x25,
            count: 1,
            lba: 11,
            device: 0x40,
            ..Cmd::default()
        },
        false,
        &[prd(DATA, SECTOR, false)],
    );
    assert_eq!(irr(&m) & (1 << 5), 1 << 5);
    let space = m.space("mem").expect("the memory space");

    // Every dword in the window, twice, live and debug: nothing here is
    // read-to-clear, so all four readings agree.
    for offset in (0..0x500u64).step_by(4) {
        let live = space
            .read(ABAR_BASE + offset, Width::U32, MemAttrs::DEFAULT)
            .expect("a mapped dword");
        let debug = space
            .read(ABAR_BASE + offset, Width::U32, MemAttrs::DEBUG)
            .expect("a mapped dword");
        let again = space
            .read(ABAR_BASE + offset, Width::U32, MemAttrs::DEBUG)
            .expect("a mapped dword");
        assert_eq!(
            live, debug,
            "a debug read of {offset:#x} answered differently"
        );
        assert_eq!(
            debug, again,
            "a debug read of {offset:#x} had a side effect"
        );
    }

    // And every write is refused: `PxCI` runs a command, `PxCMD.ST` stops the
    // engine, `GHC.HR` resets the adapter, and `PxIS` and `IS` are
    // write-1-to-clear — a debugger that touched any of them would have
    // acknowledged an interrupt the guest has not seen.
    for offset in [REG_GHC, REG_IS, port(P_CI), port(P_CMD), port(P_IS)] {
        assert!(
            space
                .write(ABAR_BASE + offset, Width::U32, 1, MemAttrs::DEBUG)
                .is_err(),
            "a debug write to {offset:#x} was accepted"
        );
    }

    // The completion is still outstanding and the line is still down.
    assert_eq!(reg32(&m, port(P_IS)) & IS_DHRS, IS_DHRS);
    assert_eq!(reg32(&m, REG_IS) & 1, 1);
    assert_eq!(irr(&m) & (1 << 5), 1 << 5);
}

#[test]
fn the_board_snapshots_and_restores_to_the_same_state_hash() {
    let (m, _store) = ready();
    const LBA: u64 = 300;
    let payload: Vec<u8> = (0..SECTOR as usize).map(|i| (i as u8) ^ 0x19).collect();
    poke_bytes(&m, DATA, &payload);
    issue(
        &m,
        Cmd {
            command: 0x35, // WRITE DMA EXT
            count: 1,
            lba: LBA,
            device: 0x40,
            ..Cmd::default()
        },
        true,
        &[prd(DATA, SECTOR, false)],
    );
    acknowledge(&m);

    let before = m.state_hash().expect("a hash");
    let bytes = m.save().expect("it snapshots");

    // A second board, loaded: the hash is over the whole machine, so this
    // covers the register file, the port state, the drive's command block, the
    // medium and the board's RAM at once.
    let (mut other, other_store) = board();
    other.load(&bytes).expect("it restores");
    assert_eq!(other.state_hash().expect("a hash"), before);
    assert_eq!(
        on_medium(&other_store, LBA),
        payload,
        "the snapshot did not carry the drive"
    );

    // The restored adapter is a working adapter: its port is still started, its
    // command list base came back, and a read lands.
    init_pic(&other);
    assert_eq!(reg32(&other, port(P_CMD)) & CMD_CR, CMD_CR);
    let moved = issue(
        &other,
        Cmd {
            command: 0x25,
            count: 1,
            lba: LBA,
            device: 0x40,
            ..Cmd::default()
        },
        false,
        &[prd(DATA + 0x4000, SECTOR, false)],
    );
    assert_eq!(moved, SECTOR as u32);
    assert_eq!(peek_bytes(&other, DATA + 0x4000, SECTOR), payload);
}

#[test]
fn a_stopped_port_runs_nothing_and_an_unmastered_function_fetches_nothing() {
    // Two guards a driver relies on without ever testing: `PxCI` is only
    // write-1-to-set while `ST` is set (§3.3.14), and a function without Bus
    // Master Enable generates no accesses of its own.
    let (m, _store) = ready();
    set_reg32(&m, port(P_CMD), CMD_FRE);
    poke_bytes(&m, DATA, &vec![0u8; SECTOR as usize]);
    let moved = issue(
        &m,
        Cmd {
            command: 0x25,
            count: 1,
            lba: 7,
            device: 0x40,
            ..Cmd::default()
        },
        false,
        &[prd(DATA, SECTOR, false)],
    );
    assert_eq!(moved, 0, "a stopped port issued nothing");
    assert_eq!(reg32(&m, port(P_CI)), 0, "and PxCI did not even latch");
    assert_eq!(peek_bytes(&m, DATA, 4), vec![0u8; 4]);

    // Bus mastering off, port started: `PxCI` latches and nothing happens.
    config_write(&m, CFG_COMMAND, 0x0002);
    set_reg32(&m, port(P_CMD), CMD_FRE | CMD_ST);
    let moved = issue(
        &m,
        Cmd {
            command: 0x25,
            count: 1,
            lba: 7,
            device: 0x40,
            ..Cmd::default()
        },
        false,
        &[prd(DATA, SECTOR, false)],
    );
    assert_eq!(
        moved, 0,
        "a function that may not master the bus fetches none"
    );
    assert_eq!(reg32(&m, port(P_CI)), 1, "the command is still outstanding");

    // And granting it picks the command up, which is what a driver that set the
    // bits in the wrong order would see happen.
    config_write(&m, CFG_COMMAND, COMMAND_ON);
    assert_eq!(reg32(&m, port(P_CI)), 1, "nothing has re-rung the doorbell");
    poke32(&m, ABAR_BASE + port(P_CI), 1);
    assert_eq!(reg32(&m, port(P_CI)), 0);
    assert_eq!(peek_bytes(&m, DATA, SECTOR), stamp(7));
}
