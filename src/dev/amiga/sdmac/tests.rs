//! The Super DMAC at `$00DD0000`, the way a 68030 reaches it.
//!
//! The controller's own behaviour is `src/dev/wd33c93/tests.rs`'s. What is
//! asserted here is the board's contribution: which longword is which
//! register, which *byte lane* the SCSI chip's two addresses are on, what
//! `CONTR` and `ISTR` mean bit by bit, and that a data phase reaches memory.

use super::*;
use crate::core::props::{Media, Props, Value};
use crate::core::space::{RamStore, RequesterId, UnassignedPolicy};
use crate::core::state::{MachineShape, Migrations, StateReader, StateWriter};
use crate::core::sync::{AtomicU32, Ordering};
use crate::core::value::Width;
use crate::core::wire::{Wire, WireId, WireIdAllocator, WireSink};
use crate::dev::scsi::{self, DiskDevice};
use crate::dev::wd33c93::{self, Wd33c93};
use alloc::vec;
use alloc::vec::Vec;

const BASE: u64 = 0x00DD_0000;
const BLOCK: usize = 512;
const BLOCKS: u64 = 32;

/// Where guest memory lives in the rig's space, and where a transfer goes.
const RAM_AT: u64 = 0x0100_0000;

/// What an undriven byte reads.
const FLOAT: u8 = 0xA5;

/// A wire sink that remembers the last level it was given.
#[derive(Debug, Default)]
struct Probe {
    level: AtomicU32,
}

impl Probe {
    fn high(&self) -> bool {
        self.level.load(Ordering::Relaxed) != 0
    }
}

impl WireSink for Probe {
    fn set_level(&self, _src: WireId, _line: u32, level: Level) {
        self.level
            .store(u32::from(level.is_high()), Ordering::Relaxed);
    }
}

struct Rig {
    sdmac: Sdmac,
    chip: Arc<Wd33c93>,
    _disk: DiskDevice,
    space: Arc<AddressSpace>,
    int: Arc<Probe>,
}

fn rig_with(disk: bool) -> Rig {
    let hosts = Arc::new(crate::core::hosts::HostObjects::new());
    let mut image = vec![0u8; BLOCKS as usize * BLOCK];
    for (n, block) in image.chunks_mut(BLOCK).enumerate() {
        block.fill(n as u8);
    }
    let bytes = if disk { image } else { Vec::new() };
    let disk = DiskDevice::new(
        &Props::new()
            .with("image", Value::Media(Media::new("hd0", bytes)))
            .with("bus", Value::Str(String::from("scsi0")))
            .with("id", Value::Uint(0))
            .with_hosts(Arc::clone(&hosts)),
    )
    .expect("a drive, or an empty address");
    let chip = Arc::new(
        Wd33c93::new(
            &Props::new()
                .with("bus", Value::Str(String::from("scsi0")))
                .with("id", Value::Uint(7))
                .with_hosts(Arc::clone(&hosts)),
        )
        .expect("a controller"),
    );

    let sdmac = Sdmac::with_link(None);
    sdmac.attach_scsi(chip.port());

    let space = Arc::new(AddressSpace::new("mem", 32).with_unassigned(UnassignedPolicy::OPEN_BUS));
    space
        .topology()
        .map(Device::region(&sdmac, REGS_REGION).expect("a region"), BASE)
        .expect("it maps");
    space
        .topology()
        .map(
            Arc::new(Region::ram("ram", Arc::new(RamStore::new(0x10000)))),
            RAM_AT,
        )
        .expect("it maps");
    sdmac.attach_space(&space, RequesterId(0));

    let int = Arc::new(Probe::default());
    let ids = WireIdAllocator::new();
    let id = ids.alloc();
    let wire = Wire::builder()
        .source(id)
        .sink(Arc::clone(&int) as Arc<dyn WireSink>, 0)
        .build_shared();
    Device::connect(&sdmac, INT_PIN, WireSource::new(wire, id)).expect("the pin");
    Device::announce(&sdmac, INT_PIN);

    Rig {
        sdmac,
        chip,
        _disk: disk,
        space,
        int,
    }
}

fn rig() -> Rig {
    rig_with(true)
}

impl Rig {
    fn rb(&self, at: u64) -> u8 {
        self.space
            .read(BASE + at, Width::U8, MemAttrs::DEFAULT.with_bus(FLOAT))
            .expect("mapped") as u8
    }

    fn wb(&self, at: u64, value: u8) {
        self.space
            .write(BASE + at, Width::U8, u64::from(value), MemAttrs::DEFAULT)
            .expect("mapped");
    }

    fn rl(&self, at: u64) -> u32 {
        self.space
            .read(BASE + at, Width::U32, MemAttrs::DEFAULT.with_bus(FLOAT))
            .expect("mapped") as u32
    }

    fn wl(&self, at: u64, value: u32) {
        self.space
            .write(BASE + at, Width::U32, u64::from(value), MemAttrs::DEFAULT)
            .expect("mapped");
    }

    /// The Address register of the SCSI chip, written at its byte lane.
    fn sasr(&self, value: u8) {
        self.wb(0x49, value);
    }

    /// The register the Address register names, at its byte lane.
    fn scmd(&self, value: u8) {
        self.wb(0x43, value);
    }

    fn scmd_read(&self) -> u8 {
        self.rb(0x43)
    }

    /// Auxiliary Status, at the `A0` low lane.
    fn aux(&self) -> u8 {
        self.rb(0x49)
    }

    /// Wait for an interrupt the way Kickstart does — by reading `ISTR` — and
    /// hand back the SCSI Status byte.
    fn interrupt(&self) -> Option<u8> {
        for _ in 0..8 {
            if self.rl(ISTR) & INT_S != 0 {
                self.sasr(wd33c93::SCSI_STATUS);
                return Some(self.scmd_read());
            }
        }
        None
    }
}

// ---------------------------------------------------------------------------
// the register map
// ---------------------------------------------------------------------------

#[test]
fn every_dmac_register_is_the_longword_table_2_5_puts_it_at() {
    let r = rig();
    // §2.4.1, Table 2-5. `WTC` is read/write on the A3000's part, which is how
    // software tells it from the enhanced one — "bit two of the register is
    // fixed at zero in the new part, but is a read/write bit in the old part".
    r.wl(WTC, 0x5555_5555);
    assert_eq!(r.rl(WTC), 0x5555_5555);
    assert_ne!(r.rl(WTC) & 4, 0, "the old part's bit two");
    r.wl(WTC, 0);
    assert_eq!(r.rl(WTC), 0);

    // `ACR` rounds down to an even word, because "the DMAC doesn't support
    // odd-byte aligned transfers".
    r.wl(ACR, 0x0012_3457);
    assert_eq!(r.rl(ACR), 0x0012_3456);

    // `CONTR`'s four bits, and `DMAENA` is the strobes' to set.
    r.wl(CONTR, INTENA | DMADIR);
    assert_eq!(r.rl(CONTR), INTENA | DMADIR);
    r.wl(CONTR, DMAENA | INTENA);
    assert_eq!(r.rl(CONTR) & DMAENA, 0, "a write cannot set DMAENA");
    r.wl(ST_DMA, 0);
    assert_eq!(r.rl(CONTR) & DMAENA, DMAENA, "the strobe can");
    r.wl(SP_DMA, 0);
    assert_eq!(r.rl(CONTR) & DMAENA, 0, "and the other one clears it");

    // `DAWR` takes a value and nothing reads it back.
    r.wl(DAWR, 3);
    assert_eq!(r.sdmac.regs().dawr, 3);
}

#[test]
fn a_narrow_access_still_behaves_as_a_whole_longword() {
    let r = rig();
    // §2.4: "it is impossible to independently access individual words or
    // bytes within these registers". Kickstart writes `CONTR` as the byte at
    // `$00DD000B`, which is the low byte of the longword at `$00DD0008`.
    r.wb(0x0b, INTENA as u8);
    assert_eq!(r.rl(CONTR), INTENA);
    assert_eq!(r.rb(0x0b), INTENA as u8);
    // And reads the low byte of `ISTR` at `$00DD001F`.
    assert_eq!(u32::from(r.rb(0x1f)), r.rl(ISTR) & 0xff);
    // A word write to the low half of `WTC` at `$00DD0006` lands in it.
    r.space
        .write(BASE + 0x06, Width::U16, 0xffff, MemAttrs::DEFAULT)
        .expect("mapped");
    assert_eq!(r.rl(WTC) & 0xffff, 0xffff);
}

#[test]
fn the_scsi_chips_two_registers_are_on_two_byte_lanes() {
    let r = rig();
    // The lane rule, and every row of Table 2-5 that falls out of it: lane 1
    // is `SASR`, lane 3 is `SCMD`.
    assert_eq!(scsi_lane(0x41), Some(false));
    assert_eq!(scsi_lane(0x43), Some(true));
    assert_eq!(scsi_lane(0x47), Some(true));
    assert_eq!(scsi_lane(0x49), Some(false));
    assert_eq!(scsi_lane(0x40), None);
    assert_eq!(scsi_lane(0x42), None);
    assert_eq!(scsi_lane(0x3f), None, "below the chip select");
    assert_eq!(scsi_lane(0x4c), None, "above it");

    // Through the window: set the Timeout register and read it back at the
    // *other* documented `SCMD` address.
    r.sasr(wd33c93::TIMEOUT);
    r.scmd(0x2c);
    r.sasr(wd33c93::TIMEOUT);
    assert_eq!(r.rb(0x47), 0x2c, "$00DD0047 is SCMD too");
    // A read at a `SASR` lane is Auxiliary Status (§6.2.2), not the address
    // register, and `$00DD0041` is the same lane as `$00DD0049`.
    assert_eq!(r.rb(0x41), r.aux());
    // The lanes the chip is not on float.
    for at in [0x40, 0x42, 0x44, 0x46, 0x48, 0x4a] {
        assert_eq!(r.rb(at), FLOAT, "offset {at:#04x}");
    }
}

#[test]
fn a_wide_access_over_both_lanes_takes_the_lower_one() {
    let r = rig();
    // Table 2-5's `$00DD0040 SASR_L Write` row: a longword write at `$…40`
    // covers lane 1 and lane 3, and the chip sees the `SASR` one.
    r.wl(0x40, 0x0002_0055);
    assert_eq!(r.chip.port().address(), 0x02, "the byte on lane 1");
}

// ---------------------------------------------------------------------------
// interrupts
// ---------------------------------------------------------------------------

#[test]
fn istr_is_a_live_view_of_the_controllers_line_gated_by_intena() {
    let r = rig();
    // Nothing pending: the FIFO is empty and nothing else is set.
    assert_eq!(r.rl(ISTR), FE);
    assert!(!r.int.high());

    // A `MR-` pulse leaves `INTRQ` asserted (§6.3.1).
    Device::reset(&r.sdmac, ResetKind::Cold);
    r.chip.port().master_reset();
    // §2.4.1: "Bits INT_F, INT_S, and E_INT each reflect the status of the
    // WD33C93 interrupt line"; "the INT_P bit is low if INTENA is cleared".
    // The `ISTR` read is also the poll that lets the controller deliver what
    // it decided on — see `Chip::poll_pending` there.
    assert_eq!(r.rl(ISTR), INT_F | INT_S | E_INT | FE);
    assert!(r.chip.irq_asserted());
    assert!(!r.int.high(), "and the pin follows INT_P");

    r.wl(CONTR, INTENA);
    assert_eq!(r.rl(ISTR), INT_F | INT_S | E_INT | INT_P | FE);
    assert!(r.int.high());

    // Reading the SCSI Status register is what clears the chip's line, and
    // `ISTR` follows it because it is a view rather than a latch.
    r.sasr(wd33c93::SCSI_STATUS);
    assert_eq!(r.scmd_read(), wd33c93::INT_RESET);
    assert_eq!(r.rl(ISTR), FE);
    assert!(!r.int.high());
}

#[test]
fn clr_int_negates_the_output_without_touching_the_controller() {
    let r = rig();
    Device::reset(&r.sdmac, ResetKind::Cold);
    r.chip.port().master_reset();
    r.wl(CONTR, INTENA);
    assert_eq!(r.rl(ISTR) & INT_P, INT_P);
    assert!(r.int.high());
    // §2.4.1: the strobe "clears all interrupts registered by ISTR, and
    // negates the DMAC's interrupt output line" — but only the SCSI chip can
    // clear its own, so what is left is the second half.
    r.wl(CLR_INT, 0);
    assert!(r.chip.irq_asserted(), "the controller still wants service");
    assert_eq!(r.rl(ISTR) & INT_S, INT_S);
}

// ---------------------------------------------------------------------------
// the data path
// ---------------------------------------------------------------------------

/// Run a `Select-With-ATN-And-Transfer` through the window, the way a driver
/// does: point `ACR` at memory, start DMA, load the command block, go.
fn read_blocks(r: &Rig, lba: u8, count: u16, at: u64) -> u8 {
    r.wl(SP_DMA, 0);
    r.wl(ACR, at as u32);
    // DMA from the SCSI bus into memory: `DMADIR` low (§2.4.1).
    r.wl(CONTR, INTENA);
    r.wl(ST_DMA, 0);

    // DMA Mode in the controller's Control register (§6.2.4's `DM` field).
    r.sasr(wd33c93::CONTROL);
    r.scmd(0x80);
    r.sasr(wd33c93::TARGET_LUN);
    r.scmd(0);
    r.sasr(wd33c93::DEST_ID);
    r.scmd(0);
    r.sasr(wd33c93::SOURCE_ID);
    r.scmd(wd33c93::SOURCE_ER);
    let bytes = u32::from(count) * BLOCK as u32;
    r.sasr(wd33c93::COUNT_MSB);
    r.scmd(bytes.to_be_bytes()[1]);
    r.scmd(bytes.to_be_bytes()[2]);
    r.scmd(bytes.to_be_bytes()[3]);
    r.sasr(wd33c93::CDB1);
    for byte in [0x28, 0, 0, 0, 0, lba, 0, (count >> 8) as u8, count as u8, 0] {
        r.scmd(byte);
    }
    r.sasr(wd33c93::COMMAND);
    r.scmd(wd33c93::CMD_SELECT_ATN_TRANSFER);
    r.interrupt().expect("an interrupt")
}

#[test]
fn a_data_phase_reaches_memory_at_the_address_control_register() {
    let r = rig();
    Device::reset(&r.sdmac, ResetKind::Cold);
    let _ = r.interrupt();
    assert_eq!(read_blocks(&r, 5, 2, RAM_AT), wd33c93::INT_SAT_DONE);

    let mut got = vec![0u8; 2 * BLOCK];
    r.space
        .read_bytes(RAM_AT, &mut got, MemAttrs::DEBUG)
        .expect("mapped");
    assert!(got[..BLOCK].iter().all(|&b| b == 5));
    assert!(got[BLOCK..].iter().all(|&b| b == 6));
    // §2.4.1: "the ACR … must be updated for every new DMA transfer" — it has
    // walked to the end of what it moved.
    assert_eq!(u64::from(r.rl(ACR)), RAM_AT + 2 * BLOCK as u64);
}

#[test]
fn nothing_moves_while_dma_is_stopped_or_pointed_the_other_way() {
    let r = rig();
    Device::reset(&r.sdmac, ResetKind::Cold);
    let _ = r.interrupt();

    // No `ST_DMA`: the port refuses, the transfer count does not reach zero,
    // and the command terminates rather than completing.
    r.wl(ACR, RAM_AT as u32);
    r.wl(CONTR, INTENA);
    r.sasr(wd33c93::CONTROL);
    r.scmd(0x80);
    r.sasr(wd33c93::DEST_ID);
    r.scmd(0);
    r.sasr(wd33c93::COUNT_MSB);
    r.scmd(0);
    r.scmd(2);
    r.scmd(0);
    r.sasr(wd33c93::CDB1);
    for byte in [0x28u8, 0, 0, 0, 0, 0, 0, 0, 1, 0] {
        r.scmd(byte);
    }
    r.sasr(wd33c93::COMMAND);
    r.scmd(wd33c93::CMD_SELECT_ATN_TRANSFER);
    let status = r.interrupt().expect("an interrupt");
    assert_eq!(status & 0xf0, wd33c93::INT_TERMINATED);

    let mut got = vec![0u8; 16];
    r.space
        .read_bytes(RAM_AT, &mut got, MemAttrs::DEBUG)
        .expect("mapped");
    assert!(got.iter().all(|&b| b == 0), "memory is untouched");
}

#[test]
fn an_empty_scsi_address_times_out_through_the_window() {
    let r = rig_with(false);
    Device::reset(&r.sdmac, ResetKind::Cold);
    let _ = r.interrupt();
    r.wl(CONTR, INTENA);
    r.sasr(wd33c93::DEST_ID);
    r.scmd(0);
    r.sasr(wd33c93::COMMAND);
    r.scmd(wd33c93::CMD_SELECT_ATN);
    assert_eq!(r.interrupt(), Some(wd33c93::INT_TIMEOUT));
}

#[test]
fn with_no_controller_linked_the_two_addresses_read_as_an_empty_socket() {
    let sdmac = Sdmac::with_link(None);
    let space = Arc::new(AddressSpace::new("mem", 32).with_unassigned(UnassignedPolicy::OPEN_BUS));
    space
        .topology()
        .map(Device::region(&sdmac, "").expect("a region"), BASE)
        .expect("it maps");
    for at in [0x41u64, 0x43, 0x47, 0x49] {
        assert_eq!(
            space
                .read(BASE + at, Width::U8, MemAttrs::DEFAULT.with_bus(FLOAT))
                .expect("mapped"),
            0
        );
    }
    // And the DMAC's own registers still work.
    space
        .write(
            BASE + CONTR,
            Width::U32,
            u64::from(INTENA),
            MemAttrs::DEFAULT,
        )
        .expect("mapped");
    assert_eq!(sdmac.regs().contr, INTENA);
}

// ---------------------------------------------------------------------------
// `debug`
// ---------------------------------------------------------------------------

#[test]
fn a_debug_write_is_refused_and_a_debug_read_strobes_nothing() {
    let r = rig();
    Device::reset(&r.sdmac, ResetKind::Cold);
    r.chip.port().master_reset();
    r.wl(CONTR, INTENA);

    // Four offsets in this window are strobes and two hand a byte to a chip
    // that will run a SCSI command with it.
    for at in [
        DAWR, WTC, CONTR, ACR, ST_DMA, FLUSH, CLR_INT, SP_DMA, 0x43, 0x49,
    ] {
        assert!(
            r.space
                .write(BASE + at, Width::U8, 0, MemAttrs::DEBUG)
                .is_err(),
            "offset {at:#04x}"
        );
    }

    // A `Select` raises its own interrupt at once — a chip asserts `INTRQ`
    // when it decides something, not when a host gets round to asking.
    r.sasr(wd33c93::SCSI_STATUS);
    assert_eq!(r.scmd_read(), wd33c93::INT_RESET);
    r.sasr(wd33c93::DEST_ID);
    r.scmd(0);
    r.sasr(wd33c93::COMMAND);
    r.scmd(wd33c93::CMD_SELECT_ATN);
    assert_eq!(
        r.space
            .read(BASE + ISTR, Width::U32, MemAttrs::DEBUG)
            .expect("mapped") as u32
            & INT_S,
        INT_S,
        "and a debugger may look at the line"
    );
    r.sasr(wd33c93::SCSI_STATUS);
    assert_eq!(r.scmd_read(), wd33c93::INT_SELECT_DONE);

    // The service-required interrupt queued behind it is the one the *guest's*
    // `ISTR` read lets out; a debugger's read asks nothing.
    let debug_istr = r
        .space
        .read(BASE + ISTR, Width::U32, MemAttrs::DEBUG)
        .expect("mapped") as u32;
    assert_eq!(debug_istr & INT_S, 0, "a debugger asks nothing");
    assert_eq!(r.rl(ISTR) & INT_S, INT_S, "the guest's read does");
    r.sasr(wd33c93::SCSI_STATUS);
    assert_eq!(
        r.scmd_read(),
        wd33c93::INT_SERVICE | 0b110,
        "§7.5.6's first REQ, naming the MESSAGE OUT phase"
    );
}

// ---------------------------------------------------------------------------
// snapshot
// ---------------------------------------------------------------------------

fn snapshot(s: &Sdmac) -> Vec<u8> {
    let mut shape = MachineShape::new();
    shape.add_device("sdmac", CLASS_NAME).unwrap();
    let mut w = StateWriter::new(shape);
    {
        let mut chunk = w.chunk("sdmac", CLASS_NAME, STATE_VERSION).unwrap();
        Device::save(s, &mut chunk).unwrap();
    }
    w.to_vec().unwrap()
}

#[test]
fn a_snapshot_round_trips_to_identical_state() {
    let saved = rig();
    saved.wl(DAWR, 3);
    saved.wl(WTC, 0x1234_5678);
    saved.wl(ACR, 0x0700_1000);
    saved.wl(CONTR, INTENA | DMADIR);
    saved.wl(ST_DMA, 0);
    let bytes = snapshot(&saved.sdmac);

    let restored = rig();
    assert_ne!(snapshot(&restored.sdmac), bytes);
    let reader = StateReader::new(&bytes).unwrap();
    let chunk = reader
        .load("sdmac", CLASS_NAME, STATE_VERSION, &Migrations::new())
        .unwrap();
    Device::load(&restored.sdmac, &mut chunk.reader()).unwrap();
    assert_eq!(snapshot(&restored.sdmac), bytes, "identical state");
    assert_eq!(restored.rl(ACR), 0x0700_1000);
    assert_eq!(restored.rl(CONTR), DMAENA | INTENA | DMADIR);
}

#[test]
fn a_reset_stops_dma_and_puts_every_register_back() {
    let r = rig();
    r.wl(WTC, 0xffff_ffff);
    r.wl(ACR, 0x0700_0000);
    r.wl(CONTR, INTENA | DMADIR);
    r.wl(ST_DMA, 0);
    assert_eq!(r.rl(CONTR) & DMAENA, DMAENA);
    Device::reset(&r.sdmac, ResetKind::Warm);
    assert_eq!(r.rl(CONTR), 0);
    assert_eq!(r.rl(ACR), 0);
    assert_eq!(r.rl(WTC), 0);
    assert!(!r.int.high());
}

/// The rendezvous the machine file relies on: both objects name `scsi0` and
/// meet there.
#[test]
fn a_named_bus_is_how_the_controller_and_the_target_meet() {
    let hosts = Arc::new(crate::core::hosts::HostObjects::new());
    let disk = DiskDevice::new(
        &Props::new()
            .with("image", Value::Media(Media::new("hd0", vec![0u8; 8 * 512])))
            .with("bus", Value::Str(String::from("scsi0")))
            .with("id", Value::Uint(4))
            .with_hosts(Arc::clone(&hosts)),
    )
    .expect("a drive");
    let chip = Wd33c93::new(
        &Props::new()
            .with("bus", Value::Str(String::from("scsi0")))
            .with_hosts(Arc::clone(&hosts)),
    )
    .expect("a controller");
    assert_eq!(chip.bus().occupied(), vec![4]);
    assert_eq!(scsi::names(&hosts), vec![String::from("scsi0")]);
    drop(disk);
}
