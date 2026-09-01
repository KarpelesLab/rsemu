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
    let bus = Arc::new(SpiBus::new());
    let spi = Stm32Spi::with_bus(link, Some(Arc::clone(&bus)), ChipSelect(0));
    let regs = ops(&spi);
    Harness {
        spi,
        regs,
        bus,
        now: core::cell::Cell::new(0),
    }
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
    assert_eq!(h.read(0x04), CR2_MASK);
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
