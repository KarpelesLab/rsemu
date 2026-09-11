//! What a driver does to this peripheral, and what it must get back.
//!
//! Everything is driven through the register block, because that is the only
//! interface a guest has. The peripheral is a lazily advanced device, so a
//! test that expects a frame to finish has to say how much virtual time passed
//! — which is the point: the timing is part of the model.

use super::*;
use crate::core::props::Value;
use crate::core::space::RegionKind;
use crate::core::state::{MachineShape, Migrations, StateReader, StateWriter};
use crate::core::wire::Wire;

// ---------------------------------------------------------------------------
// harness
// ---------------------------------------------------------------------------

/// A slave that answers with the complement of what it was handed one word
/// ago, so a test can tell a stale shift register from a fresh one.
#[derive(Debug)]
struct Echo {
    format: Format,
    state: Mutex<(u32, Vec<u32>)>,
}

impl Echo {
    fn new(format: Format) -> Arc<Echo> {
        Arc::new(Echo {
            format,
            state: Mutex::with_rank(LockRank::DEVICE, (0xffff_ffff, Vec::new())),
        })
    }

    fn seen(&self) -> Vec<u32> {
        self.state.lock().1.clone()
    }
}

impl SpiSlave for Echo {
    fn format(&self) -> Format {
        self.format
    }

    fn select(&self, _selected: bool) {}

    fn transfer(&self, mosi: u32) -> u32 {
        let mut state = self.state.lock();
        let out = state.0;
        state.0 = self.format.truncate(!mosi);
        state.1.push(mosi);
        out
    }

    fn peek(&self) -> u32 {
        self.state.lock().0
    }
}

fn ops(spi: &Stm32Spi) -> Arc<dyn MemOps> {
    match spi.region("regs").expect("the block is there").kind() {
        RegionKind::Io(ops) => Arc::clone(ops),
        _ => unreachable!("the register block is MMIO"),
    }
}

struct Harness {
    spi: Stm32Spi,
    regs: Arc<dyn MemOps>,
    bus: Arc<SpiBus>,
    now: core::cell::Cell<u64>,
}

fn harness(link: Link) -> Harness {
    harness_with(Variant::F4, link)
}

/// The same, of whichever generation the test is about.
fn harness_with(variant: Variant, link: Link) -> Harness {
    let bus = Arc::new(SpiBus::new());
    let spi = Stm32Spi::with_bus(variant, link, Some(Arc::clone(&bus)), ChipSelect(0));
    let regs = ops(&spi);
    Harness {
        spi,
        regs,
        bus,
        now: core::cell::Cell::new(0),
    }
}

/// A `"f7"` peripheral with an [`Echo`] framed to match `bits` on its bus.
///
/// The transactional link asks the *slave* how a word is framed
/// (`SpiBus::transfer` -> `bus::spi::exchange`), so a test about `DS` has to
/// frame the peer the same way or it is testing nothing.
fn fifo_with_echo(bits: u8) -> (Harness, Arc<Echo>) {
    let h = harness_with(Variant::Fifo, Link::Transactional);
    let echo = Echo::new(Format::new(Mode::Mode0, bits, BitOrder::MsbFirst));
    h.bus
        .attach(ChipSelect(0), Arc::clone(&echo) as Arc<dyn SpiSlave>)
        .expect("cs0 is free");
    h.bus.select(Some(ChipSelect(0)));
    (h, echo)
}

impl Harness {
    fn write(&self, offset: u64, value: u16) {
        self.regs
            .write(offset, &value.to_le_bytes(), MemAttrs::DEFAULT)
            .expect("a half-word write is a legal cycle");
    }

    fn read(&self, offset: u64) -> u16 {
        let mut bytes = [0u8; 2];
        self.regs
            .read(offset, &mut bytes, MemAttrs::DEFAULT)
            .expect("a half-word read is a legal cycle");
        u16::from_le_bytes(bytes)
    }

    fn read_debug(&self, offset: u64) -> u16 {
        let mut bytes = [0u8; 2];
        self.regs
            .read(offset, &mut bytes, MemAttrs::DEBUG)
            .expect("a debug read is a legal cycle");
        u16::from_le_bytes(bytes)
    }

    /// Let `ticks` of the peripheral's clock domain pass.
    fn run(&self, ticks: u64) {
        self.now.set(self.now.get() + ticks);
        self.spi.advance_to(self.now.get());
    }

    /// Write `bytes` to `offset` as one access of that width.
    fn write_wide(&self, offset: u64, bytes: &[u8]) {
        self.regs
            .write(offset, bytes, MemAttrs::DEFAULT)
            .expect("a legal cycle");
    }

    /// Read `N` bytes from `offset` as one access of that width.
    fn read_wide<const N: usize>(&self, offset: u64) -> [u8; N] {
        let mut out = [0u8; N];
        self.regs
            .read(offset, &mut out, MemAttrs::DEFAULT)
            .expect("a legal cycle");
        out
    }

    /// One byte of `DR`, which on an `"f7"` is one frame at `DS <= 8`.
    fn read_dr_byte(&self) -> u8 {
        self.read_wide::<1>(0x0c)[0]
    }

    /// Set `DS` to `bits`, keeping the rest of `CR2`.
    fn set_ds(&self, bits: u8) {
        let keep = self.read(0x04) & !(CR2_DS_MASK << CR2_DS_SHIFT);
        self.write(0x04, keep | ((u16::from(bits) - 1) << CR2_DS_SHIFT));
    }

    /// `SR.FRLVL`, as a number of quarters.
    fn frlvl(&self) -> u16 {
        (self.read(0x08) >> SR_FRLVL_SHIFT) & SR_LVL_MASK
    }

    /// `SR.FTLVL`, as a number of quarters.
    fn ftlvl(&self) -> u16 {
        (self.read(0x08) >> SR_FTLVL_SHIFT) & SR_LVL_MASK
    }

    /// Poll `BSY` the way a driver does, for at most `limit` ticks.
    fn wait(&self, limit: u64) {
        for _ in 0..limit {
            if self.read(0x08) & SR_BSY == 0 {
                return;
            }
            self.run(1);
        }
        panic!("the frame never finished");
    }

    fn enable_master(&self, cr1: u16) {
        // `SSM` and `SSI` set is the software slave management a driver uses
        // when it drives the chip select from a GPIO — and without `SSI` set,
        // a master takes an immediate mode fault (§28.3.10).
        self.write(0x00, cr1 | CR1_MSTR | CR1_SSM | CR1_SSI | CR1_SPE);
    }
}

// ---------------------------------------------------------------------------
// the register file
// ---------------------------------------------------------------------------

#[test]
fn the_reset_values_are_the_manuals() {
    let h = harness(Link::Transactional);
    assert_eq!(h.read(0x00), 0x0000, "CR1");
    assert_eq!(h.read(0x04), 0x0000, "CR2");
    assert_eq!(h.read(0x08), 0x0002, "SR: TXE is set out of reset");
    assert_eq!(h.read(0x10), 0x0007, "CRCPR");
    assert_eq!(h.read(0x14), 0x0000, "RXCRCR");
    assert_eq!(h.read(0x18), 0x0000, "TXCRCR");
    assert_eq!(h.read(0x20), 0x0002, "I2SPR");
}

#[test]
fn the_crc_registers_are_read_only() {
    let h = harness(Link::Transactional);
    h.write(0x14, 0x1234);
    h.write(0x18, 0x5678);
    assert_eq!(h.read(0x14), 0);
    assert_eq!(h.read(0x18), 0);
}

#[test]
fn a_reserved_cr2_bit_is_forced_to_zero() {
    let h = harness(Link::Transactional);
    // §28.5.2: bit 3 is "forced to 0 by hardware", not merely reserved.
    h.write(0x04, 0xffff);
    assert_eq!(h.read(0x04) & (1 << 3), 0);
    assert_eq!(h.read(0x04), CR2_MASK_F4);
}

#[test]
fn a_read_above_the_last_register_answers_zero_rather_than_faulting() {
    let h = harness(Link::Transactional);
    // The peripheral owns a kilobyte of the bus and decodes nine registers of
    // it; the rest is not a fault, it is silicon that does not answer.
    assert_eq!(h.read(0x100), 0);
    assert_eq!(h.read(0x3fc), 0);
}

#[test]
fn a_byte_access_reaches_its_own_lane() {
    let h = harness(Link::Transactional);
    h.write(0x10, 0xabcd);
    let mut byte = [0u8; 1];
    h.regs
        .read(0x10, &mut byte, MemAttrs::DEFAULT)
        .expect("a byte read");
    assert_eq!(byte[0], 0xcd);
    h.regs
        .read(0x11, &mut byte, MemAttrs::DEFAULT)
        .expect("a byte read");
    assert_eq!(byte[0], 0xab);
    // And a byte write leaves the other half alone.
    h.regs
        .write(0x10, &[0x11], MemAttrs::DEFAULT)
        .expect("a byte write");
    assert_eq!(h.read(0x10), 0xab11);
}

// ---------------------------------------------------------------------------
// a frame, as a master
// ---------------------------------------------------------------------------

#[test]
fn a_master_frame_moves_a_byte_each_way() {
    let h = harness(Link::Transactional);
    let echo = Echo::new(Format::DEFAULT);
    h.bus
        .attach(ChipSelect(0), Arc::clone(&echo) as Arc<dyn SpiSlave>)
        .expect("cs0 is free");
    h.bus.select(Some(ChipSelect(0)));
    h.enable_master(0);
    assert_eq!(h.read(0x08) & SR_TXE, SR_TXE, "the buffer starts empty");

    h.write(0x0c, 0x5a);
    assert_eq!(h.read(0x08) & SR_BSY, SR_BSY, "and now it is shifting");
    h.wait(64);
    assert_eq!(h.read(0x08) & SR_RXNE, SR_RXNE);
    // Full duplex: what comes back is what the slave had loaded *before* the
    // frame, not a reply to it.
    assert_eq!(h.read(0x0c), 0xff);
    assert_eq!(h.read(0x08) & SR_RXNE, 0, "reading DR pops the buffer");
    assert_eq!(echo.seen(), [0x5a]);

    // The second frame gets the answer to the first.
    h.write(0x0c, 0x00);
    h.wait(64);
    assert_eq!(h.read(0x0c), 0xa5, "the complement of 0x5a");
}

#[test]
fn a_frame_costs_the_baud_rate_the_prescaler_names() {
    // §28.5.1: the divisor is 2^(BR + 1) of PCLK, so an eight-bit frame at
    // BR = 3 is 8 x 16 = 128 ticks.
    for br in 0u16..8 {
        let h = harness(Link::Transactional);
        let echo = Echo::new(Format::DEFAULT);
        h.bus
            .attach(ChipSelect(0), echo as Arc<dyn SpiSlave>)
            .expect("cs0 is free");
        h.bus.select(Some(ChipSelect(0)));
        h.enable_master(br << CR1_BR_SHIFT);
        h.write(0x0c, 0x11);
        let want = 8 * (1u64 << (br + 1));
        h.run(want - 1);
        assert_eq!(h.read(0x08) & SR_BSY, SR_BSY, "BR={br}: not yet");
        h.run(1);
        assert_eq!(h.read(0x08) & SR_BSY, 0, "BR={br}: and now");
    }
}

#[test]
fn sixteen_bit_frames_carry_sixteen_bits() {
    let h = harness(Link::Transactional);
    let echo = Echo::new(Format::new(Mode::Mode0, 16, BitOrder::MsbFirst));
    h.bus
        .attach(ChipSelect(0), Arc::clone(&echo) as Arc<dyn SpiSlave>)
        .expect("cs0 is free");
    h.bus.select(Some(ChipSelect(0)));
    h.enable_master(CR1_DFF);
    h.write(0x0c, 0xbeef);
    h.wait(64);
    assert_eq!(echo.seen(), [0xbeef]);
    assert_eq!(h.read(0x0c), 0xffff);
    // And in 8-bit format §28.5.4 forces the top half of a read to zero.
    h.write(0x0c, 0x0000);
    h.wait(64);
    h.write(0x00, h.read(0x00) & !CR1_DFF);
    assert_eq!(h.read(0x0c) & 0xff00, 0);
}

#[test]
fn a_receive_only_master_clocks_itself_with_no_data_register_write() {
    let h = harness(Link::Transactional);
    let echo = Echo::new(Format::DEFAULT);
    h.bus
        .attach(ChipSelect(0), Arc::clone(&echo) as Arc<dyn SpiSlave>)
        .expect("cs0 is free");
    h.bus.select(Some(ChipSelect(0)));
    // §28.3.4: "the communication starts immediately and stops when the SPE
    // bit is cleared" — no `DR` write, and none expected.
    h.enable_master(CR1_RXONLY);
    h.run(16);
    assert_eq!(h.read(0x08) & SR_RXNE, SR_RXNE, "a word arrived unbidden");
    assert!(!echo.seen().is_empty());
    // Clearing `SPE` stops it.
    h.write(0x00, 0);
    let before = echo.seen().len();
    h.run(1000);
    assert_eq!(echo.seen().len(), before, "and it stayed stopped");
}

// ---------------------------------------------------------------------------
// the slave-select business
// ---------------------------------------------------------------------------

#[test]
fn a_master_with_ssi_low_takes_a_mode_fault_and_demotes_itself() {
    let h = harness(Link::Transactional);
    // Software slave management with `SSI` *clear*: the peripheral sees its
    // own NSS low, which §28.3.10 says is a master mode fault. This is the
    // classic driver bug — `SSM` set and `SSI` forgotten.
    h.write(0x00, CR1_MSTR | CR1_SSM | CR1_SPE);
    let sr = h.read(0x08);
    assert_eq!(sr & SR_MODF, SR_MODF, "MODF");
    let cr1 = h.read(0x00);
    assert_eq!(cr1 & CR1_SPE, 0, "SPE cleared itself");
    assert_eq!(cr1 & CR1_MSTR, 0, "and the master became a slave");
}

#[test]
fn while_mode_fault_stands_the_hardware_refuses_to_be_a_master_again() {
    let h = harness(Link::Transactional);
    h.write(0x00, CR1_MSTR | CR1_SSM | CR1_SPE);
    assert_eq!(h.read(0x08) & SR_MODF, SR_MODF);

    // This is the sentence that turns the bug into a peripheral that will not
    // start: "hardware does not allow the setting of the SPE and MSTR bits
    // while the MODF bit is set". The first write below also happens to be
    // the second half of the clearing sequence, so it clears MODF — but it
    // still does not take SPE or MSTR.
    h.read(0x08);
    h.write(0x00, CR1_MSTR | CR1_SSM | CR1_SSI | CR1_SPE);
    let cr1 = h.read(0x00);
    assert_eq!(cr1 & (CR1_SPE | CR1_MSTR), 0, "refused");
    assert_eq!(h.read(0x08) & SR_MODF, 0, "but MODF is gone now");

    // With the fault cleared, the same write takes.
    h.write(0x00, CR1_MSTR | CR1_SSM | CR1_SSI | CR1_SPE);
    let cr1 = h.read(0x00);
    assert_eq!(cr1 & (CR1_SPE | CR1_MSTR), CR1_SPE | CR1_MSTR);
}

#[test]
fn clearing_mode_fault_needs_the_status_register_access_first() {
    let h = harness(Link::Transactional);
    h.write(0x00, CR1_MSTR | CR1_SSM | CR1_SPE);
    // A `CR1` write with no `SR` access before it does *not* clear it.
    h.write(0x00, 0);
    assert_eq!(h.read(0x08) & SR_MODF, SR_MODF, "still set");
    // §28.3.10 accepts a read *or a write* of `SR` as the first step.
    h.write(0x08, 0xffff);
    h.write(0x00, 0);
    assert_eq!(h.read(0x08) & SR_MODF, 0);
}

#[test]
fn hardware_nss_output_follows_spe_and_moves_the_bus_chip_select() {
    let h = harness(Link::Transactional);
    let echo = Echo::new(Format::DEFAULT);
    h.bus
        .attach(ChipSelect(0), echo as Arc<dyn SpiSlave>)
        .expect("cs0 is free");
    assert_eq!(h.bus.selected(), None);
    // §28.3.1, hardware NSS with the output enabled: NSS "is driven low when
    // the master starts the communication and is kept low until the SPI is
    // disabled". So `SPE` is the chip select, which is how a single-slave
    // board needs no GPIO at all.
    h.write(0x04, CR2_SSOE);
    h.write(0x00, CR1_MSTR | CR1_SPE);
    assert_eq!(h.bus.selected(), Some(ChipSelect(0)));
    assert_eq!(
        h.read(0x08) & SR_MODF,
        0,
        "a master cannot fault on its own"
    );
    h.write(0x00, 0);
    assert_eq!(h.bus.selected(), None);
}

#[test]
fn a_master_with_hardware_nss_and_no_output_faults_on_the_pin() {
    let h = harness(Link::Transactional);
    // `SSM` clear and `SSOE` clear: NSS is a genuine input, and a board that
    // pulls it low takes the fault.
    h.write(0x00, CR1_MSTR | CR1_SPE);
    assert_eq!(h.read(0x08) & SR_MODF, 0, "an unwired pin idles high");
    let sink = h
        .spi
        .sink(pin::NSS_IN, &[])
        .expect("the peripheral has an NSS input");
    sink.sink.set_level(WireId(0), sink.line, Level::Low);
    assert_eq!(h.read(0x08) & SR_MODF, SR_MODF);
}

// ---------------------------------------------------------------------------
// overrun
// ---------------------------------------------------------------------------

#[test]
fn an_overrun_freezes_the_receive_buffer_and_needs_two_reads_to_clear() {
    let h = harness(Link::Transactional);
    let echo = Echo::new(Format::DEFAULT);
    h.bus
        .attach(ChipSelect(0), echo as Arc<dyn SpiSlave>)
        .expect("cs0 is free");
    h.bus.select(Some(ChipSelect(0)));
    h.enable_master(0);

    h.write(0x0c, 0x00);
    h.wait(64);
    // A second frame with the first still unread.
    h.write(0x0c, 0x11);
    h.wait(64);
    assert_eq!(h.read(0x08) & SR_OVR, SR_OVR);
    // §28.3.10: "the receiver buffer contents are not updated with the newly
    // received data" — the first word is what is there, not the second.
    assert_eq!(h.read(0x0c), 0xff, "frozen at the first");

    // The clearing sequence is a `DR` read then an `SR` read, in that order,
    // and nothing else will do.
    let h2 = harness(Link::Transactional);
    let echo = Echo::new(Format::DEFAULT);
    h2.bus
        .attach(ChipSelect(0), echo as Arc<dyn SpiSlave>)
        .expect("cs0 is free");
    h2.bus.select(Some(ChipSelect(0)));
    h2.enable_master(0);
    h2.write(0x0c, 0x00);
    h2.wait(64);
    h2.write(0x0c, 0x11);
    h2.wait(64);
    assert_eq!(h2.read(0x08) & SR_OVR, SR_OVR);
    // An `SR` read on its own does not do it.
    assert_eq!(h2.read(0x08) & SR_OVR, SR_OVR);
    h2.read(0x0c);
    // The `SR` read that completes the sequence still *reports* the flag —
    // hardware hands over the value it had and clears it behind the read — so
    // it takes one more read to see it gone.
    assert_eq!(
        h2.read(0x08) & SR_OVR,
        SR_OVR,
        "the clearing read still shows it"
    );
    assert_eq!(h2.read(0x08) & SR_OVR, 0, "DR then SR");
}

// ---------------------------------------------------------------------------
// the debug rule
// ---------------------------------------------------------------------------

#[test]
fn a_debug_read_consumes_none_of_the_guests_flag_sequences() {
    let h = harness(Link::Transactional);
    let echo = Echo::new(Format::DEFAULT);
    h.bus
        .attach(ChipSelect(0), echo as Arc<dyn SpiSlave>)
        .expect("cs0 is free");
    h.bus.select(Some(ChipSelect(0)));
    h.enable_master(0);
    h.write(0x0c, 0x77);
    h.wait(64);

    // Three traps in one register block, and this is all three.
    assert_eq!(h.read_debug(0x08) & SR_RXNE, SR_RXNE);
    assert_eq!(h.read_debug(0x0c), 0xff, "the word is visible");
    assert_eq!(
        h.read(0x08) & SR_RXNE,
        SR_RXNE,
        "and still there for the guest"
    );
    assert_eq!(h.read(0x0c), 0xff, "which reads it for real");
    assert_eq!(h.read(0x08) & SR_RXNE, 0);

    // A debug read of `SR` must not take a step of the mode-fault sequence
    // either.
    let h = harness(Link::Transactional);
    h.write(0x00, CR1_MSTR | CR1_SSM | CR1_SPE);
    h.read_debug(0x08);
    h.write(0x00, 0);
    assert_eq!(h.read(0x08) & SR_MODF, SR_MODF, "the debugger took no step");
}

#[test]
fn a_debug_write_is_refused_outright() {
    let h = harness(Link::Transactional);
    assert!(
        h.regs.write(0x0c, &[0u8, 0], MemAttrs::DEBUG).is_err(),
        "a debug write would start a frame"
    );
}

// ---------------------------------------------------------------------------
// interrupts
// ---------------------------------------------------------------------------

#[test]
fn the_interrupt_line_follows_the_enables_the_manual_lists() {
    let h = harness(Link::Transactional);
    let echo = Echo::new(Format::DEFAULT);
    h.bus
        .attach(ChipSelect(0), echo as Arc<dyn SpiSlave>)
        .expect("cs0 is free");
    h.bus.select(Some(ChipSelect(0)));
    assert!(!h.spi.irq_asserted());
    // `TXE` is set out of reset, so enabling its interrupt asserts at once —
    // which is exactly what a driver that enables `TXEIE` before writing `DR`
    // is relying on.
    h.write(0x04, CR2_TXEIE);
    assert!(h.spi.irq_asserted());
    h.write(0x04, 0);
    assert!(!h.spi.irq_asserted());
    // And `RXNEIE` after a frame.
    h.enable_master(0);
    h.write(0x0c, 0x22);
    h.wait(64);
    h.write(0x04, CR2_RXNEIE);
    assert!(h.spi.irq_asserted());
    h.read(0x0c);
    assert!(!h.spi.irq_asserted());
}

// ---------------------------------------------------------------------------
// CRC
// ---------------------------------------------------------------------------

#[test]
fn enabling_the_calculator_resets_both_accumulators() {
    let h = harness(Link::Transactional);
    let echo = Echo::new(Format::DEFAULT);
    h.bus
        .attach(ChipSelect(0), echo as Arc<dyn SpiSlave>)
        .expect("cs0 is free");
    h.bus.select(Some(ChipSelect(0)));
    h.enable_master(CR1_CRCEN);
    h.write(0x0c, 0x31);
    h.wait(64);
    assert_ne!(h.read(0x18), 0, "the transmit CRC moved");
    // §28.5.5: writing `CRCEN` resets both registers.
    h.write(0x00, h.read(0x00) & !CR1_CRCEN);
    h.write(0x00, h.read(0x00) | CR1_CRCEN);
    assert_eq!(h.read(0x18), 0);
    assert_eq!(h.read(0x14), 0);
}

#[test]
fn crc_next_sends_the_accumulator_and_checks_what_comes_back() {
    let h = harness(Link::Transactional);
    let echo = Echo::new(Format::DEFAULT);
    h.bus
        .attach(ChipSelect(0), Arc::clone(&echo) as Arc<dyn SpiSlave>)
        .expect("cs0 is free");
    h.bus.select(Some(ChipSelect(0)));
    h.enable_master(CR1_CRCEN);
    h.write(0x0c, 0x31);
    h.wait(64);
    h.read(0x0c);
    let txcrc = h.read(0x18);
    // §28.3.6: the frame after `CRCNEXT` carries the CRC rather than `DR`.
    h.write(0x00, h.read(0x00) | CR1_CRCNEXT);
    h.write(0x0c, 0x00);
    h.wait(64);
    assert_eq!(echo.seen().last().copied(), Some(u32::from(txcrc)));
    assert_eq!(h.read(0x00) & CR1_CRCNEXT, 0, "and it cleared itself");
    // The echo did not answer with the CRC we calculated, so the comparison
    // fails — which is the flag doing its job.
    assert_eq!(h.read(0x08) & SR_CRCERR, SR_CRCERR);
    // §28.5.3: `CRCERR` is cleared by writing zero to it.
    h.write(0x08, 0);
    assert_eq!(h.read(0x08) & SR_CRCERR, 0);
}

// ---------------------------------------------------------------------------
// the two link models
// ---------------------------------------------------------------------------

#[test]
fn both_link_models_move_the_same_bytes_in_the_same_time() {
    // The claim `docs/buses/low-speed.md` asks for, at this peripheral: a
    // frame costs `bits x 2^(BR+1)` ticks either way, and the slave sees the
    // same words. What differs is only whether the edges exist.
    for (mode, bits, order) in [
        (Mode::Mode0, 8, BitOrder::MsbFirst),
        (Mode::Mode1, 8, BitOrder::MsbFirst),
        (Mode::Mode2, 16, BitOrder::LsbFirst),
        (Mode::Mode3, 16, BitOrder::MsbFirst),
    ] {
        let format = Format::new(mode, bits, order);
        let cr1 = (if mode.cpol() { CR1_CPOL } else { 0 })
            | (if mode.cpha() { CR1_CPHA } else { 0 })
            | (if bits == 16 { CR1_DFF } else { 0 })
            | (if order == BitOrder::LsbFirst {
                CR1_LSBFIRST
            } else {
                0
            });

        let mut answers = Vec::new();
        let mut seen = Vec::new();
        let mut kept: Vec<Arc<Wire>> = Vec::new();
        for link in [Link::Transactional, Link::Wired] {
            let h = harness(link);
            let echo = Echo::new(format);
            let pins = Arc::new(SlavePins::new(Arc::clone(&echo) as Arc<dyn SpiSlave>));
            match link {
                Link::Transactional => {
                    h.bus
                        .attach(ChipSelect(0), Arc::clone(&echo) as Arc<dyn SpiSlave>)
                        .expect("cs0 is free");
                    h.bus.select(Some(ChipSelect(0)));
                }
                Link::Wired => {
                    // Real wires, exactly what a machine file's `wire`
                    // statements build: SCK, MOSI and NSS out to the slave's
                    // pins, and MISO back.
                    let ids = [
                        WireId::new(1),
                        WireId::new(2),
                        WireId::new(3),
                        WireId::new(4),
                    ];
                    let sck = Wire::builder()
                        .source(ids[0])
                        .sink(pins.sink(slave_pin::SCK), slave_pin::SCK)
                        .build_shared();
                    let mosi = Wire::builder()
                        .source(ids[1])
                        .sink(pins.sink(slave_pin::MOSI), slave_pin::MOSI)
                        .build_shared();
                    let nss = Wire::builder()
                        .source(ids[2])
                        .sink(pins.sink(slave_pin::CS), slave_pin::CS)
                        .build_shared();
                    h.spi
                        .connect(pin::SCK, WireSource::new(sck, ids[0]))
                        .expect("sck connects");
                    h.spi
                        .connect(pin::MOSI, WireSource::new(mosi, ids[1]))
                        .expect("mosi connects");
                    h.spi
                        .connect(pin::NSS, WireSource::new(Arc::clone(&nss), ids[2]))
                        .expect("nss connects");
                    let miso_sink = h.spi.sink(pin::MISO, &[ids[3]]).expect("a miso input");
                    let miso = Wire::builder()
                        .source(ids[3])
                        .sink(miso_sink.sink, miso_sink.line)
                        .build_shared();
                    pins.connect_miso(WireSource::new(miso, ids[3]));
                    // The chip select is the peripheral's own: `SSOE` makes
                    // `SPE` drive it low, which is what selects the slave.
                    h.write(0x04, CR2_SSOE);
                    kept.push(nss);
                }
            }
            match link {
                Link::Transactional => h.enable_master(cr1),
                // `SSOE` is already set above, and hardware NSS means `SSM`
                // must stay clear or the peripheral would not drive the pin.
                Link::Wired => h.write(0x00, cr1 | CR1_MSTR | CR1_SPE),
            }
            let mut got = Vec::new();
            for word in [0x35u16, 0x00, 0xc1] {
                h.write(0x0c, word);
                h.wait(4096);
                got.push(h.read(0x0c));
            }
            answers.push(got);
            seen.push(echo.seen());
        }
        assert_eq!(answers[0], answers[1], "{format}: what came back");
        assert_eq!(seen[0], seen[1], "{format}: what the slave saw");
    }
}

// ---------------------------------------------------------------------------
// slave mode
// ---------------------------------------------------------------------------

#[test]
fn with_mstr_clear_the_peripheral_answers_instead_of_asking() {
    let h = harness(Link::Wired);
    // A slave: `SPE` set, `MSTR` clear. It generates no clock and starts
    // nothing; another controller clocks it through the fabric's own pins.
    h.write(0x0c, 0xa1);
    h.write(0x00, CR1_SPE);
    let pins = h.spi.pins();
    pins.drive(slave_pin::CS, Level::Low);
    let mut got = 0u8;
    for bit in (0..8).rev() {
        pins.drive(slave_pin::MOSI, Level::from_bool(0x4c >> bit & 1 != 0));
        got = (got << 1) | u8::from(pins.miso_level().is_high());
        pins.drive(slave_pin::SCK, Level::High);
        pins.drive(slave_pin::SCK, Level::Low);
    }
    pins.drive(slave_pin::CS, Level::High);
    assert_eq!(got, 0xa1, "what the guest had put in DR went out");
    assert_eq!(h.read(0x08) & SR_RXNE, SR_RXNE);
    assert_eq!(h.read(0x0c), 0x4c, "and what arrived is readable");
}

// ---------------------------------------------------------------------------
// construction
// ---------------------------------------------------------------------------

#[test]
fn the_link_property_is_required_and_has_no_default() {
    let e = Stm32Spi::new(&Props::new())
        .expect_err("`link` is the one choice a machine file must make")
        .to_string();
    assert!(e.contains("link"), "{e}");
    let e = Stm32Spi::new(&Props::new().with("link", Value::Str("teleport".into())))
        .expect_err("and it must be one this module knows")
        .to_string();
    assert!(e.contains("low-speed"), "{e}");
}

#[test]
fn a_transactional_peripheral_needs_a_bus_to_reach_its_slaves() {
    let e = Stm32Spi::new(&Props::new().with("link", Value::Str("transactional".into())))
        .expect_err("no bus, no slaves")
        .to_string();
    assert!(e.contains("named bus"), "{e}");
    // A wired one does not: its slaves are on the other end of its pins.
    Stm32Spi::new(&Props::new().with("link", Value::Str("wired".into())))
        .expect("a wired peripheral needs no bus");
}

// ---------------------------------------------------------------------------
// snapshots
// ---------------------------------------------------------------------------

fn snapshot(spi: &Stm32Spi) -> Vec<u8> {
    let mut shape = MachineShape::new();
    shape.add_device("spi", CLASS.name).expect("a fresh shape");
    let mut w = StateWriter::new(shape);
    {
        let mut chunk = w
            .chunk("spi", CLASS.name, CLASS.version)
            .expect("one chunk");
        spi.save(&mut chunk).expect("it saves");
    }
    w.to_vec().expect("a snapshot")
}

fn restore(spi: &Stm32Spi, bytes: &[u8]) {
    let reader = StateReader::new(bytes).expect("a snapshot");
    let chunk = reader
        .load("spi", CLASS.name, CLASS.version, &Migrations::new())
        .expect("the chunk is there");
    spi.load(&mut chunk.reader()).expect("it loads");
}

#[test]
fn a_snapshot_round_trips_to_an_identical_chunk() {
    let h = harness(Link::Transactional);
    let echo = Echo::new(Format::DEFAULT);
    h.bus
        .attach(ChipSelect(0), echo as Arc<dyn SpiSlave>)
        .expect("cs0 is free");
    h.bus.select(Some(ChipSelect(0)));
    h.enable_master(CR1_CRCEN);
    h.write(0x0c, 0x5c);
    h.wait(64);
    let first = snapshot(&h.spi);

    let other = harness(Link::Transactional);
    restore(&other.spi, &first);
    assert_eq!(snapshot(&other.spi), first, "identical bytes");
    assert_eq!(other.read(0x08), h.read(0x08));
}

#[test]
fn a_snapshot_carries_a_half_consumed_overrun_sequence() {
    let h = harness(Link::Transactional);
    let echo = Echo::new(Format::DEFAULT);
    h.bus
        .attach(ChipSelect(0), echo as Arc<dyn SpiSlave>)
        .expect("cs0 is free");
    h.bus.select(Some(ChipSelect(0)));
    h.enable_master(0);
    h.write(0x0c, 0x00);
    h.wait(64);
    h.write(0x0c, 0x11);
    h.wait(64);
    assert_eq!(h.read(0x08) & SR_OVR, SR_OVR);
    // The driver has taken the first step and not the second.
    h.read(0x0c);
    let bytes = snapshot(&h.spi);

    let other = harness(Link::Transactional);
    restore(&other.spi, &bytes);
    // On the restored peripheral, the *second* step alone finishes it — which
    // it could not do if the snapshot had lost the first.
    assert_eq!(other.read(0x08) & SR_OVR, SR_OVR, "the clearing read");
    assert_eq!(other.read(0x08) & SR_OVR, 0);
}

// ---------------------------------------------------------------------------
// the `"f7"` block: the FIFO
// ---------------------------------------------------------------------------

#[test]
fn the_f7_reset_values_are_the_manuals() {
    let h = harness_with(Variant::Fifo, Link::Transactional);
    assert_eq!(h.read(0x00), 0x0000, "CR1");
    // RM0351 §42.6.2: `0x0700`, which is `DS` powering up at eight bits and
    // nothing else. Getting this wrong gives a peripheral whose frames are
    // one bit wide until a driver writes `CR2`.
    assert_eq!(h.read(0x04), 0x0700, "CR2");
    assert_eq!(h.read(0x08), 0x0002, "SR: TXE set, both levels empty");
    assert_eq!(h.frlvl(), 0);
    assert_eq!(h.ftlvl(), 0);
    assert_eq!(h.spi.format().bits, 8, "and so the frame is eight bits");
}

#[test]
fn a_byte_write_to_dr_shifts_exactly_one_frame_when_ds_is_eight() {
    let (h, echo) = fifo_with_echo(8);
    h.write(0x04, h.read(0x04) | CR2_FRXTH);
    h.enable_master(0);
    // One 8-bit store to `DR` — what `*(volatile uint8_t *)&SPI->DR = x` is.
    h.write_wide(0x0c, &[0x9f]);
    h.wait(256);
    assert_eq!(echo.seen(), [0x9f], "one frame, not two");
    // `FRXTH` set, so one byte is enough.
    assert_eq!(h.read(0x08) & SR_RXNE, SR_RXNE);
    assert_eq!(h.frlvl(), 1, "a quarter of the FIFO");
    assert_eq!(h.read_dr_byte(), 0xff);
    assert_eq!(h.read(0x08) & SR_RXNE, 0, "and that byte is gone");
    assert_eq!(h.frlvl(), 0);
}

#[test]
fn a_halfword_write_to_dr_with_ds_eight_sends_two_frames() {
    let (h, echo) = fifo_with_echo(8);
    h.enable_master(0);
    // §42.4.9's data packing: one 16-bit store, two frames, low byte first.
    h.write_wide(0x0c, &[0x34, 0x12]);
    assert_ne!(h.ftlvl(), 0, "the second byte is still queued");
    h.wait(256);
    assert_eq!(echo.seen(), [0x34, 0x12]);
    assert_eq!(h.ftlvl(), 0, "and now the transmit side is empty");
}

#[test]
fn rxne_waits_for_two_bytes_when_frxth_is_clear() {
    let (h, _echo) = fifo_with_echo(8);
    // `FRXTH` clear out of reset: `RXNE` is a *half-full* FIFO, two bytes.
    assert_eq!(h.read(0x04) & CR2_FRXTH, 0);
    h.enable_master(0);
    h.write_wide(0x0c, &[0xaa]);
    h.wait(256);
    assert_eq!(h.frlvl(), 1, "one byte arrived");
    assert_eq!(h.read(0x08) & SR_RXNE, 0, "and it is not enough");
    h.write_wide(0x0c, &[0xbb]);
    h.wait(256);
    assert_eq!(h.read(0x08) & SR_RXNE, SR_RXNE, "two is");
    // And one 16-bit read unpacks the pair, low byte first.
    assert_eq!(h.read_wide::<2>(0x0c), [0xff, 0x55]);
    assert_eq!(h.read(0x08) & SR_RXNE, 0);
}

#[test]
fn frlvl_and_ftlvl_report_quarter_half_full() {
    let h = harness_with(Variant::Fifo, Link::Transactional);
    let echo = Echo::new(Format::DEFAULT);
    h.bus
        .attach(ChipSelect(0), Arc::clone(&echo) as Arc<dyn SpiSlave>)
        .expect("cs0 is free");
    h.bus.select(Some(ChipSelect(0)));
    // A master that is not *enabled* starts nothing, so the transmit FIFO can
    // be filled and looked at rather than drained as fast as it is written.
    h.write(0x00, CR1_MSTR | CR1_SSM | CR1_SSI);
    for (i, want) in [(0u8, 1u16), (1, 2), (2, 3), (3, 3)] {
        h.write_wide(0x0c, &[0x10 + i]);
        assert_eq!(h.ftlvl(), want, "after {} byte(s)", i + 1);
    }
    assert_eq!(
        h.read(0x08) & SR_TXE,
        0,
        "three or four bytes is above half"
    );
    assert_eq!(h.read(0x08) & SR_BSY, 0, "but nothing is shifting");

    // Now let them go, one frame — eight bits at BR = 0, so sixteen ticks —
    // at a time, and watch the other level fill.
    h.write(0x00, h.read(0x00) | CR1_SPE);
    for (n, want) in [(1u32, 1u16), (2, 2), (3, 3), (4, 3)] {
        h.run(16);
        assert_eq!(h.frlvl(), want, "after {n} frame(s)");
    }
    // The manual has four codes for five occupancies, so three and four bytes
    // share one — which is why `00` is the only level a driver can trust.
    assert_eq!(h.ftlvl(), 0);
    assert_eq!(echo.seen(), [0x10, 0x11, 0x12, 0x13]);
}

#[test]
fn a_fifth_received_frame_sets_ovr_and_is_dropped() {
    let (h, _echo) = fifo_with_echo(8);
    h.write(0x04, h.read(0x04) | CR2_FRXTH);
    h.enable_master(0);
    // Four frames fill the receive FIFO exactly, and nothing reads it.
    h.write_wide(0x0c, &[0x01, 0x02]);
    h.wait(256);
    h.write_wide(0x0c, &[0x03, 0x04]);
    h.wait(256);
    assert_eq!(h.frlvl(), 3, "full");
    assert_eq!(h.read(0x08) & SR_OVR, 0, "and not yet an overrun");
    // The fifth completes with nowhere to go.
    h.write_wide(0x0c, &[0x05]);
    h.wait(256);
    assert_eq!(h.read(0x08) & SR_OVR, SR_OVR);
    // The four that did fit are the *first* four: §42.4.9 drops the new frame,
    // it does not push the oldest out.
    assert_eq!(
        [
            h.read_dr_byte(),
            h.read_dr_byte(),
            h.read_dr_byte(),
            h.read_dr_byte()
        ],
        [0xff, 0xfe, 0xfd, 0xfc]
    );
    // And the clearing sequence is still the F4's: a read of `DR`, then a read
    // of `SR` — and it is the read *after* that one which comes back clear,
    // because the clearing read still reports the flag it is clearing.
    assert_eq!(h.read(0x08) & SR_OVR, SR_OVR, "the clearing read");
    assert_eq!(h.read(0x08) & SR_OVR, 0);
}

// ---------------------------------------------------------------------------
// the `"f7"` block: `DS`
// ---------------------------------------------------------------------------

#[test]
fn ds_values_below_four_read_back_as_eight_bits() {
    let h = harness_with(Variant::Fifo, Link::Transactional);
    // §42.6.2: `0b0000`, `0b0001` and `0b0010` are "not used" and the hardware
    // forces `0b0111`. Forced on the way in, so the read-back does not claim a
    // frame size the peripheral is not using.
    for code in 0u16..3 {
        h.write(0x04, code << CR2_DS_SHIFT);
        assert_eq!((h.read(0x04) >> CR2_DS_SHIFT) & CR2_DS_MASK, 7, "DS={code}");
        assert_eq!(h.spi.format().bits, 8);
    }
    // And the first legal code really is four bits.
    h.write(0x04, DS_CODE_MIN << CR2_DS_SHIFT);
    assert_eq!(h.spi.format().bits, 4);
}

#[test]
fn a_five_bit_frame_clocks_five_bits_and_is_right_aligned() {
    let (h, echo) = fifo_with_echo(5);
    h.set_ds(5);
    h.write(0x04, h.read(0x04) | CR2_FRXTH);
    h.enable_master(0);
    // The top three bits of the byte are not on the wire at all.
    h.write_wide(0x0c, &[0xff]);
    // Five bit times at BR = 0 is ten ticks, not sixteen — which is the whole
    // claim that this is a frame size and not a rounded-up byte.
    h.run(9);
    assert_eq!(h.read(0x08) & SR_BSY, SR_BSY, "nine ticks is not a frame");
    h.run(1);
    assert_eq!(h.read(0x08) & SR_BSY, 0, "ten is");
    assert_eq!(echo.seen(), [0x1f], "right-aligned and masked to DS");
    assert_eq!(h.read_dr_byte(), 0x1f, "and so is what comes back");
}

#[test]
fn a_twelve_bit_frame_takes_two_fifo_bytes_each_way() {
    let (h, echo) = fifo_with_echo(12);
    h.set_ds(12);
    h.enable_master(0);
    // Above eight bits the FIFO is still a *byte* FIFO, so one byte is half a
    // frame and nothing goes out for it.
    h.write_wide(0x0c, &[0xbc]);
    h.run(64);
    assert!(echo.seen().is_empty(), "half a frame is not a frame");
    assert_eq!(h.ftlvl(), 1, "and the byte is waiting for its partner");
    h.write_wide(0x0c, &[0x0a]);
    h.wait(256);
    assert_eq!(echo.seen(), [0x0abc], "little-endian, right-aligned");
    // Two bytes back, so the receive level is a half rather than a quarter.
    assert_eq!(h.frlvl(), 2);
    assert_eq!(h.read_wide::<2>(0x0c), [0xff, 0x0f], "0xfff, right-aligned");
}

#[test]
fn sixteen_bit_frames_still_work_on_the_fifo_block() {
    let (h, echo) = fifo_with_echo(16);
    h.set_ds(16);
    h.enable_master(0);
    h.write_wide(0x0c, &[0xef, 0xbe]);
    h.wait(256);
    assert_eq!(echo.seen(), [0xbeef]);
    // And `CR1` bit 11 is *not* what chose it: on this block that bit is
    // `CRCL`, so setting it changes the CRC and not the frame.
    h.write(0x00, h.read(0x00) | CR1_CRCL);
    assert_eq!(h.spi.format().bits, 16, "DS decides, not DFF");
}

#[test]
fn the_variant_decides_what_cr1_bit_eleven_means() {
    let f4 = harness(Link::Transactional);
    f4.write(0x00, CR1_DFF);
    assert_eq!(f4.spi.format().bits, 16, "`f4`: bit 11 is DFF");

    let f7 = harness_with(Variant::Fifo, Link::Transactional);
    f7.write(0x00, CR1_CRCL);
    assert_eq!(f7.spi.format().bits, 8, "`f7`: bit 11 is CRCL, DS decides");
    // Which is the divergence a `variant` has to make real rather than
    // paper over: neither driver half-works as the other.
    assert_eq!(f4.read(0x04) & (CR2_DS_MASK << CR2_DS_SHIFT), 0);
    assert_ne!(f7.read(0x04) & (CR2_DS_MASK << CR2_DS_SHIFT), 0);
}

// ---------------------------------------------------------------------------
// the `"f7"` block: `CRCL`
// ---------------------------------------------------------------------------

#[test]
fn crcl_runs_a_sixteen_bit_crc_and_sends_it_as_two_eight_bit_frames() {
    let (h, echo) = fifo_with_echo(8);
    h.write(0x04, h.read(0x04) | CR2_FRXTH);
    h.enable_master(CR1_CRCEN | CR1_CRCL);
    // Three data frames, which is enough shifting for a sixteen-bit
    // accumulator to reach into its top half — an eight-bit CRC could not
    // hold the value the next assertion checks.
    for byte in [0x31u8, 0x41, 0x59] {
        h.write_wide(0x0c, &[byte]);
        h.wait(256);
        h.read_dr_byte();
    }
    let txcrc = h.read(0x18);
    assert!(txcrc > 0xff, "a sixteen-bit CRC over eight-bit frames");

    // §42.4.11: the CRC follows the data. It is wider than a frame here, so it
    // takes two of them, most significant first.
    h.write(0x00, h.read(0x00) | CR1_CRCNEXT);
    h.wait(256);
    let seen = echo.seen();
    assert_eq!(seen.len(), 5, "three data frames and two of CRC");
    assert_eq!(
        seen[seen.len() - 2..],
        [u32::from(txcrc >> 8), u32::from(txcrc & 0xff)]
    );
    assert_eq!(h.read(0x00) & CR1_CRCNEXT, 0, "and it cleared itself");
    // The echo answered with something else, so the comparison fails — which
    // is the flag doing its job over the reassembled sixteen bits.
    assert_eq!(h.read(0x08) & SR_CRCERR, SR_CRCERR);
}

#[test]
fn crcl_clear_is_an_eight_bit_crc_in_one_frame() {
    let (h, echo) = fifo_with_echo(8);
    h.write(0x04, h.read(0x04) | CR2_FRXTH);
    h.enable_master(CR1_CRCEN);
    h.write_wide(0x0c, &[0x31]);
    h.wait(256);
    h.read_dr_byte();
    let txcrc = h.read(0x18);
    assert!(txcrc <= 0xff, "eight bits wide");
    h.write(0x00, h.read(0x00) | CR1_CRCNEXT);
    h.wait(256);
    assert_eq!(echo.seen().last().copied(), Some(u32::from(txcrc)));
}

// ---------------------------------------------------------------------------
// the `"f7"` block: odd-length DMA
// ---------------------------------------------------------------------------

#[test]
fn ldma_tx_puts_five_frames_on_the_wire_from_three_halfword_writes() {
    let (h, echo) = fifo_with_echo(8);
    // What a driver programs before starting an odd-length packed DMA.
    h.write(0x04, h.read(0x04) | CR2_TXDMAEN | CR2_LDMA_TX);
    h.enable_master(0);
    // Five data as three 16-bit accesses: the sixth byte is padding.
    for pair in [[0u8, 1], [2, 3], [4, 0xff]] {
        h.write_wide(0x0c, &pair);
        h.run(64);
    }
    assert_eq!(
        echo.seen(),
        [0, 1, 2, 3, 4],
        "five frames, and the pad is not"
    );
    // The held byte is still counted — it occupies an entry — so the stream is
    // ended the way a driver ends one, by dropping the DMA enable.
    assert_ne!(h.ftlvl(), 0);
    h.write(0x04, h.read(0x04) & !CR2_TXDMAEN);
    assert_eq!(h.ftlvl(), 0);
    assert_eq!(h.read(0x08) & SR_BSY, 0);
    h.run(64);
    assert_eq!(echo.seen().len(), 5, "and it never went out");
}

#[test]
fn without_ldma_tx_the_same_three_writes_send_six_frames() {
    let (h, echo) = fifo_with_echo(8);
    h.enable_master(0);
    for pair in [[0u8, 1], [2, 3], [4, 0xff]] {
        h.write_wide(0x0c, &pair);
        h.run(64);
    }
    // Which is exactly the bug `LDMA_TX` exists to prevent: one frame too many.
    assert_eq!(echo.seen(), [0, 1, 2, 3, 4, 0xff]);
}

#[test]
fn ldma_rx_lets_the_odd_last_byte_raise_rxne() {
    let (h, _echo) = fifo_with_echo(8);
    // `FRXTH` clear, as a packed DMA read leaves it: `RXNE` at two bytes.
    h.enable_master(0);
    h.write_wide(0x0c, &[0x00]);
    h.wait(256);
    assert_eq!(h.frlvl(), 1);
    assert_eq!(h.read(0x08) & SR_RXNE, 0, "the DMA would stall here");

    h.write(0x04, h.read(0x04) | CR2_RXDMAEN | CR2_LDMA_RX);
    // Nothing in flight and nothing queued, so the lone byte can only be the
    // odd last one — and now it is reachable.
    assert_eq!(h.read(0x08) & SR_RXNE, SR_RXNE);
    // The 16-bit read the DMA makes pops the one byte and zeroes the rest.
    assert_eq!(h.read_wide::<2>(0x0c), [0xff, 0x00]);
    assert_eq!(h.read(0x08) & SR_RXNE, 0);
}

// ---------------------------------------------------------------------------
// the `"f7"` block: the debug rule, and a snapshot
// ---------------------------------------------------------------------------

#[test]
fn a_debug_read_of_dr_pops_nothing_out_of_the_receive_fifo() {
    let (h, _echo) = fifo_with_echo(8);
    h.write(0x04, h.read(0x04) | CR2_FRXTH);
    h.enable_master(0);
    h.write_wide(0x0c, &[0x01, 0x02]);
    h.wait(256);
    assert_eq!(h.frlvl(), 2);

    // A debugger dumping the block reads `DR` like anything else, and if that
    // popped the FIFO the guest's next read would return the *second* byte and
    // its driver would be one byte out for the rest of the transfer.
    for _ in 0..4 {
        assert_eq!(h.read_debug(0x0c), 0xfe_ff);
    }
    assert_eq!(h.frlvl(), 2, "nothing was consumed");
    assert_eq!(h.read(0x08) & SR_RXNE, SR_RXNE);
    assert_eq!(h.read_dr_byte(), 0xff, "and the guest still gets its own");
}

#[test]
fn a_snapshot_carries_both_fifos() {
    let (h, _echo) = fifo_with_echo(8);
    h.write(0x04, h.read(0x04) | CR2_FRXTH);
    h.enable_master(0);
    // Three frames fill the receive side...
    h.write_wide(0x0c, &[0x01, 0x02]);
    h.wait(256);
    h.write_wide(0x0c, &[0x03]);
    h.wait(256);
    assert_eq!(h.frlvl(), 3);
    // ...and a write with no time to drain leaves the transmit side loaded and
    // a frame in flight, which is the state a snapshot has to survive.
    h.write_wide(0x0c, &[0x04, 0x05]);
    assert_ne!(h.ftlvl(), 0);
    let bytes = snapshot(&h.spi);

    let other = harness_with(Variant::Fifo, Link::Transactional);
    restore(&other.spi, &bytes);
    assert_eq!(snapshot(&other.spi), bytes, "identical bytes");
    assert_eq!(other.read(0x08), h.read(0x08));
    // And the restored peripheral hands back the same three bytes, in order,
    // which is the only thing that proves the *contents* came across and not
    // just the level field.
    assert_eq!(
        [
            other.read_dr_byte(),
            other.read_dr_byte(),
            other.read_dr_byte()
        ],
        [0xff, 0xfe, 0xfd]
    );
}

#[test]
fn a_snapshot_carries_a_half_sent_sixteen_bit_crc() {
    let (h, _echo) = fifo_with_echo(8);
    h.write(0x04, h.read(0x04) | CR2_FRXTH);
    h.enable_master(CR1_CRCEN | CR1_CRCL);
    h.write_wide(0x0c, &[0x31]);
    h.wait(256);
    h.read_dr_byte();
    // Between the two halves of the CRC: `CRCNEXT` has fired and one frame of
    // two has gone. A snapshot that called this "not started" would send the
    // high half twice on resume.
    h.write(0x00, h.read(0x00) | CR1_CRCNEXT);
    h.run(16);
    let bytes = snapshot(&h.spi);

    let other = harness_with(Variant::Fifo, Link::Transactional);
    restore(&other.spi, &bytes);
    assert_eq!(snapshot(&other.spi), bytes, "identical bytes");
}

#[test]
fn the_variant_property_is_optional_and_checked() {
    let props = Props::new()
        .with("link", Value::from("transactional"))
        .with("bus", Value::from("spi-variant-test"))
        .with("variant", Value::from("f7"));
    assert_eq!(
        Stm32Spi::new(&props).expect("a legal peripheral").variant(),
        Variant::Fifo
    );
    let default = Props::new()
        .with("link", Value::from("transactional"))
        .with("bus", Value::from("spi-variant-default"));
    assert_eq!(
        Stm32Spi::new(&default)
            .expect("a legal peripheral")
            .variant(),
        Variant::F4,
        "the boards in the tree are F4s"
    );
    // The H7's is a third IP and not a third value; naming it has to fail
    // rather than quietly give an F4.
    let h7 = Props::new()
        .with("link", Value::from("transactional"))
        .with("bus", Value::from("spi-variant-h7"))
        .with("variant", Value::from("h7"));
    assert!(Stm32Spi::new(&h7).is_err());
}

// ---------------------------------------------------------------------------
// against a real part
// ---------------------------------------------------------------------------

/// Read a W25Q's `9Fh` identifier through the FIFO, popping `DR` however
/// `frxth` says to.
///
/// The proof that this is a controller and not a register model: every byte
/// below is a frame clocked down [`crate::bus::spi`] into
/// [`crate::dev::flash::spinor`]'s own command decoder, and the identifier can
/// only come back if the frames were right.
#[cfg(feature = "dev-flash-spinor")]
fn jedec_through_the_fifo(frxth: bool) -> [u8; 3] {
    use crate::dev::flash::SpiNor;

    let h = harness_with(Variant::Fifo, Link::Transactional);
    let part = SpiNor::new(&Props::new().with("size", Value::Size(1024 * 1024)))
        .expect("a plausible part");
    h.bus
        .attach(ChipSelect(0), part.slave())
        .expect("cs0 is free");

    // Hardware NSS rather than `SSM`, because the chip select is what delimits
    // a flash command: `SSOE` with `SPE` drops it, and clearing `SPE` raises
    // it again (§28.3.1), which is exactly what the board in
    // `machines/spi-flash.machine` does.
    let cr2 = h.read(0x04) | CR2_SSOE | if frxth { CR2_FRXTH } else { 0 };
    h.write(0x04, cr2);
    h.write(0x00, CR1_MSTR | CR1_SPE);
    assert_eq!(h.bus.selected(), Some(ChipSelect(0)), "NSS went low");

    // `9Fh` and three bytes of clocking, packed two to an access in the
    // FRXTH-clear case and one at a time otherwise — the same four frames
    // either way.
    if frxth {
        for byte in [0x9fu8, 0, 0, 0] {
            h.write_wide(0x0c, &[byte]);
            h.wait(512);
        }
    } else {
        h.write_wide(0x0c, &[0x9f, 0]);
        h.wait(512);
        h.write_wide(0x0c, &[0, 0]);
        h.wait(512);
    }
    assert_eq!(h.frlvl(), 3, "four bytes, which is the whole FIFO");
    assert_eq!(h.read(0x08) & SR_OVR, 0, "and not one too many");

    let got = if frxth {
        // The first byte is the part answering the opcode, which is idle.
        assert_eq!(h.read_dr_byte(), 0xff);
        [h.read_dr_byte(), h.read_dr_byte(), h.read_dr_byte()]
    } else {
        let first = h.read_wide::<2>(0x0c);
        let second = h.read_wide::<2>(0x0c);
        assert_eq!(first[0], 0xff);
        [first[1], second[0], second[1]]
    };
    // Raising the chip select is what ends the command on the part.
    h.write(0x00, h.read(0x00) & !CR1_SPE);
    assert_eq!(h.bus.selected(), None);
    got
}

#[cfg(feature = "dev-flash-spinor")]
#[test]
fn the_jedec_id_of_a_spinor_comes_back_through_the_fifo() {
    // `EFh` is Winbond (JEP106 bank 1), `40h` the W25Q ordering option, and
    // `14h` the capacity byte of a 1 MiB part — the logarithm of the density.
    assert_eq!(
        jedec_through_the_fifo(true),
        [0xef, 0x40, 0x14],
        "FRXTH set"
    );
    // And the same identifier, read two bytes to an access. The packing is
    // invisible on the wire, which is the claim being made.
    assert_eq!(
        jedec_through_the_fifo(false),
        [0xef, 0x40, 0x14],
        "FRXTH clear, packed"
    );
}
