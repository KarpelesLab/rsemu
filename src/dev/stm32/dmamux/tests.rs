//! Tests for the STM32 DMA request multiplexer.

use super::*;

use alloc::vec;

use crate::core::props::Value;
use crate::core::state::{MachineShape, Migrations, StateReader, StateWriter};
use crate::core::sync::Mutex as SyncMutex;
use crate::core::wire::{Wire, WireIdAllocator};

/// A channel control register's offset (RM0432 §14.5.2).
const fn ccr(c: u64) -> u64 {
    4 * c
}

/// A request generator's control register (RM0432 §14.5.5).
const fn rgcr(g: u64) -> u64 {
    OFF_RGCR + 4 * g
}

/// A wire sink that records every level it is handed.
///
/// Ranked at `DEVICE` deliberately: the multiplexer must drive its outputs
/// with its own `DEVICE` lock released, so a sink that takes one of the same
/// rank is what catches a regression in the re-entrancy contract.
#[derive(Debug)]
struct Probe {
    seen: SyncMutex<Vec<Level>>,
}

impl Probe {
    fn new() -> Arc<Probe> {
        Arc::new(Probe {
            seen: SyncMutex::with_rank(LockRank::DEVICE, Vec::new()),
        })
    }

    /// The level it is currently being driven at.
    fn level(&self) -> Level {
        self.seen.lock().last().copied().unwrap_or(Level::Low)
    }

    /// How many rising edges it has seen — one per forwarded request when the
    /// line is pulsed rather than held.
    fn rises(&self) -> usize {
        self.seen
            .lock()
            .iter()
            .filter(|l| **l == Level::High)
            .count()
    }

    fn clear(&self) {
        self.seen.lock().clear();
    }
}

impl WireSink for Probe {
    fn set_level(&self, _src: WireId, _line: u32, level: Level) {
        self.seen.lock().push(level);
    }
}

/// A multiplexer with a probe on every output that a test asks for.
struct Rig {
    mux: Dmamux,
    regs: Registers,
    ids: WireIdAllocator,
}

impl Rig {
    fn new(channels: usize) -> Rig {
        let mux = Dmamux::with_channels(channels);
        let regs = Registers {
            shared: Arc::clone(&mux.shared),
        };
        Rig {
            mux,
            regs,
            ids: WireIdAllocator::new(),
        }
    }

    /// A fresh net with `probe` on the far end, driven by `connect`.
    fn wire(&self, probe: &Arc<Probe>) -> WireSource {
        let id = self.ids.alloc();
        let wire = Arc::new(
            Wire::builder()
                .source(id)
                .sink(Arc::clone(probe) as Arc<dyn WireSink>, 0)
                .build(),
        );
        WireSource::new(wire, id)
    }

    /// Probe channel `c`'s request output.
    fn probe_channel(&self, c: usize) -> Arc<Probe> {
        let probe = Probe::new();
        let source = self.wire(&probe);
        Device::connect(&self.mux, &format!("{}{c}", pin::CHANNEL), source).expect("an output");
        probe.clear();
        probe
    }

    /// Probe channel `c`'s event output.
    fn probe_event(&self, c: usize) -> Arc<Probe> {
        let probe = Probe::new();
        let source = self.wire(&probe);
        Device::connect(&self.mux, &format!("{}{c}", pin::EVENT), source).expect("an output");
        probe.clear();
        probe
    }

    /// Probe the overrun interrupt.
    fn probe_irq(&self) -> Arc<Probe> {
        let probe = Probe::new();
        let source = self.wire(&probe);
        Device::connect(&self.mux, pin::IRQ, source).expect("an output");
        probe.clear();
        probe
    }

    fn poke(&self, offset: u64, value: u32) {
        self.regs
            .write(offset, &value.to_le_bytes(), MemAttrs::DEFAULT)
            .expect("a word write is legal");
    }

    fn peek(&self, offset: u64) -> u32 {
        let mut bytes = [0u8; 4];
        self.regs
            .read(offset, &mut bytes, MemAttrs::DEFAULT)
            .expect("a word read is legal");
        u32::from_le_bytes(bytes)
    }

    /// `DMAMUX_CSR`, the synchronization overrun flags.
    fn csr(&self) -> u32 {
        self.peek(OFF_CSR)
    }

    /// `DMAMUX_RGSR`, the trigger overrun flags.
    fn rgsr(&self) -> u32 {
        self.peek(OFF_RGSR)
    }
}

/// A `CxCR` value, spelled out field by field so a test reads like the manual.
#[derive(Default, Clone, Copy)]
struct Ccr {
    id: u32,
    soie: bool,
    ege: bool,
    se: bool,
    spol: u32,
    nbreq: u32,
    sync_id: u32,
}

impl Ccr {
    fn bits(self) -> u32 {
        (self.id & CCR_DMAREQ_ID)
            | if self.soie { CCR_SOIE } else { 0 }
            | if self.ege { CCR_EGE } else { 0 }
            | if self.se { CCR_SE } else { 0 }
            | (self.spol << CCR_SPOL_SHIFT)
            | (self.nbreq << CCR_NBREQ_SHIFT)
            | (self.sync_id << CCR_SYNC_ID_SHIFT)
    }
}

// ---------------------------------------------------------------------------
// routing, which is the whole point
// ---------------------------------------------------------------------------

#[test]
fn a_channel_forwards_the_request_line_its_id_names() {
    let rig = Rig::new(7);
    let ch0 = rig.probe_channel(0);
    let ch1 = rig.probe_channel(1);

    rig.poke(
        ccr(0),
        Ccr {
            id: 17,
            ..Ccr::default()
        }
        .bits(),
    );
    rig.poke(
        ccr(1),
        Ccr {
            id: 23,
            ..Ccr::default()
        }
        .bits(),
    );

    rig.mux.set_request(17, Level::High);
    assert_eq!(ch0.level(), Level::High, "channel 0 selected line 17");
    assert_eq!(ch1.level(), Level::Low, "channel 1 did not");

    rig.mux.set_request(17, Level::Low);
    assert_eq!(ch0.level(), Level::Low, "and it follows the line down");

    rig.mux.set_request(23, Level::High);
    assert_eq!(ch1.level(), Level::High);
    assert_eq!(ch0.level(), Level::Low);
}

#[test]
fn two_channels_may_share_one_request_line() {
    let rig = Rig::new(7);
    let ch0 = rig.probe_channel(0);
    let ch3 = rig.probe_channel(3);
    rig.poke(
        ccr(0),
        Ccr {
            id: 9,
            ..Ccr::default()
        }
        .bits(),
    );
    rig.poke(
        ccr(3),
        Ccr {
            id: 9,
            ..Ccr::default()
        }
        .bits(),
    );

    rig.mux.set_request(9, Level::High);
    assert_eq!(ch0.level(), Level::High);
    assert_eq!(ch3.level(), Level::High);
}

#[test]
fn request_id_zero_parks_the_channel() {
    let rig = Rig::new(7);
    let ch0 = rig.probe_channel(0);
    rig.poke(
        ccr(0),
        Ccr {
            id: 9,
            ..Ccr::default()
        }
        .bits(),
    );
    rig.mux.set_request(9, Level::High);
    assert_eq!(ch0.level(), Level::High);

    // `DMAREQ_ID = 0` is the reset value and means "no request": the channel
    // goes quiet even though the peripheral is still asking.
    rig.poke(ccr(0), 0);
    assert_eq!(ch0.level(), Level::Low);
    assert_eq!(rig.peek(ccr(0)), 0);
}

#[test]
fn re_pointing_a_channel_picks_up_a_line_already_held_high() {
    let rig = Rig::new(7);
    let ch0 = rig.probe_channel(0);
    // The peripheral has been asking all along; the guest only now routes it.
    rig.mux.set_request(11, Level::High);
    assert_eq!(ch0.level(), Level::Low);

    rig.poke(
        ccr(0),
        Ccr {
            id: 11,
            ..Ccr::default()
        }
        .bits(),
    );
    assert_eq!(
        ch0.level(),
        Level::High,
        "a routing change settles against the levels that already exist"
    );
}

#[test]
fn a_channel_beyond_the_instances_count_does_not_decode() {
    let rig = Rig::new(2);
    // `DMAMUX_C2CR` is outside a two-channel instance.
    rig.poke(ccr(2), 0x7f);
    assert_eq!(rig.peek(ccr(2)), 0, "not this part's register");
}

// ---------------------------------------------------------------------------
// synchronization (RM0432 §14.3.3)
// ---------------------------------------------------------------------------

#[test]
fn synchronization_holds_requests_back_until_the_sync_edge() {
    let rig = Rig::new(7);
    let ch0 = rig.probe_channel(0);
    // Two requests per sync event, rising edge of `sync3`.
    rig.poke(
        ccr(0),
        Ccr {
            id: 20,
            se: true,
            spol: 0b01,
            nbreq: 1,
            sync_id: 3,
            ..Ccr::default()
        }
        .bits(),
    );

    rig.mux.set_request(20, Level::High);
    assert_eq!(ch0.level(), Level::Low, "no sync event yet");
    rig.mux.set_request(20, Level::Low);

    rig.mux.set_sync(3, Level::High);
    // The sync edge grants credit for two; nothing is pending, so nothing
    // goes out yet.
    assert_eq!(ch0.level(), Level::Low);

    rig.mux.set_request(20, Level::High);
    assert_eq!(ch0.level(), Level::High, "the first of the two");
    rig.mux.set_request(20, Level::Low);
    rig.mux.set_request(20, Level::High);
    assert_eq!(ch0.level(), Level::High, "the second");
    rig.mux.set_request(20, Level::Low);

    rig.mux.set_request(20, Level::High);
    assert_eq!(
        ch0.level(),
        Level::Low,
        "the credit is spent: the third is swallowed"
    );
}

#[test]
fn a_held_request_is_forwarded_on_the_sync_edge_itself() {
    let rig = Rig::new(7);
    let ch0 = rig.probe_channel(0);
    rig.poke(
        ccr(0),
        Ccr {
            id: 20,
            se: true,
            spol: 0b01,
            sync_id: 0,
            ..Ccr::default()
        }
        .bits(),
    );
    // A FIFO-style peripheral holds its line high and never pulses it. If the
    // sync edge did not sample the level it would wait for an edge that is
    // never coming.
    rig.mux.set_request(20, Level::High);
    assert_eq!(ch0.level(), Level::Low);
    rig.mux.set_sync(0, Level::High);
    assert_eq!(ch0.level(), Level::High);
}

#[test]
fn sync_polarity_selects_which_edge_counts() {
    let rig = Rig::new(7);
    let ch0 = rig.probe_channel(0);
    // `SPOL = 10`: falling edge only.
    rig.poke(
        ccr(0),
        Ccr {
            id: 20,
            se: true,
            spol: 0b10,
            sync_id: 1,
            ..Ccr::default()
        }
        .bits(),
    );
    rig.mux.set_request(20, Level::High);

    rig.mux.set_sync(1, Level::High);
    assert_eq!(ch0.level(), Level::Low, "the rising edge is not selected");
    rig.mux.set_sync(1, Level::Low);
    assert_eq!(ch0.level(), Level::High, "the falling one is");
}

#[test]
fn spol_zero_is_no_event_at_all() {
    let rig = Rig::new(7);
    let ch0 = rig.probe_channel(0);
    rig.poke(
        ccr(0),
        Ccr {
            id: 20,
            se: true,
            spol: 0b00,
            sync_id: 1,
            ..Ccr::default()
        }
        .bits(),
    );
    rig.mux.set_request(20, Level::High);
    rig.mux.set_sync(1, Level::High);
    rig.mux.set_sync(1, Level::Low);
    assert_eq!(
        ch0.level(),
        Level::Low,
        "a synchronized channel with no edge"
    );
}

#[test]
fn the_event_output_pulses_on_the_last_request_of_a_burst() {
    let rig = Rig::new(7);
    let _ch0 = rig.probe_channel(0);
    let evt0 = rig.probe_event(0);
    // `EGE`, two requests per event.
    rig.poke(
        ccr(0),
        Ccr {
            id: 20,
            ege: true,
            se: true,
            spol: 0b01,
            nbreq: 1,
            sync_id: 0,
            ..Ccr::default()
        }
        .bits(),
    );
    rig.mux.set_sync(0, Level::High);

    rig.mux.set_request(20, Level::High);
    rig.mux.set_request(20, Level::Low);
    assert_eq!(evt0.rises(), 0, "not after the first of two");
    rig.mux.set_request(20, Level::High);
    assert_eq!(evt0.rises(), 1, "and exactly once after the second");
    assert_eq!(evt0.level(), Level::Low, "a pulse, not a level");
}

#[test]
fn a_sync_event_arriving_early_sets_sof_and_raises_the_interrupt() {
    let rig = Rig::new(7);
    let _ch2 = rig.probe_channel(2);
    let irq = rig.probe_irq();
    rig.poke(
        ccr(2),
        Ccr {
            id: 20,
            soie: true,
            se: true,
            spol: 0b01,
            nbreq: 3,
            sync_id: 5,
            ..Ccr::default()
        }
        .bits(),
    );

    rig.mux.set_sync(5, Level::High);
    assert_eq!(rig.csr(), 0, "the first event has nothing to overrun");
    assert_eq!(irq.level(), Level::Low);

    rig.mux.set_sync(5, Level::Low);
    rig.mux.set_sync(5, Level::High);
    assert_eq!(
        rig.csr(),
        1 << 2,
        "four requests were granted and none spent"
    );
    assert_eq!(irq.level(), Level::High, "SOIE is set");

    // `DMAMUX_CFR` is write-one-to-clear, and the interrupt follows it down.
    rig.poke(OFF_CFR, 1 << 2);
    assert_eq!(rig.csr(), 0);
    assert_eq!(irq.level(), Level::Low);
}

#[test]
fn an_overrun_without_its_enable_sets_the_flag_and_no_interrupt() {
    let rig = Rig::new(7);
    let irq = rig.probe_irq();
    rig.poke(
        ccr(0),
        Ccr {
            id: 20,
            se: true,
            spol: 0b01,
            sync_id: 0,
            ..Ccr::default()
        }
        .bits(),
    );
    rig.mux.set_sync(0, Level::High);
    rig.mux.set_sync(0, Level::Low);
    rig.mux.set_sync(0, Level::High);
    assert_eq!(rig.csr(), 1, "the flag is unconditional");
    assert_eq!(irq.level(), Level::Low, "SOIE is not");
}

// ---------------------------------------------------------------------------
// the request generators (RM0432 §14.3.5)
// ---------------------------------------------------------------------------

#[test]
fn a_request_generator_emits_gnbreq_plus_one_requests() {
    let rig = Rig::new(7);
    let ch4 = rig.probe_channel(4);
    // Generator 0 drives request line 1; channel 4 listens to it.
    rig.poke(
        ccr(4),
        Ccr {
            id: 1,
            ..Ccr::default()
        }
        .bits(),
    );
    // `GNBREQ = 2` is three requests, rising edge of `trg7`.
    rig.poke(
        rgcr(0),
        7 | RGCR_GE | (0b01 << RGCR_GPOL_SHIFT) | (2 << RGCR_GNBREQ_SHIFT),
    );

    rig.mux.set_trigger(7, Level::High);
    assert_eq!(ch4.rises(), 3, "GNBREQ + 1 pulses on the channel output");
    assert_eq!(ch4.level(), Level::Low, "and it ends low: they are pulses");
    assert_eq!(rig.rgsr(), 0, "all three were taken");
}

#[test]
fn a_generator_nobody_listens_to_overruns_on_the_second_trigger() {
    let rig = Rig::new(7);
    let irq = rig.probe_irq();
    // No channel selects line 2, which is generator 1's.
    rig.poke(rgcr(1), 3 | RGCR_GE | RGCR_OIE | (0b01 << RGCR_GPOL_SHIFT));

    rig.mux.set_trigger(3, Level::High);
    assert_eq!(rig.rgsr(), 0, "the first trigger merely goes unanswered");
    rig.mux.set_trigger(3, Level::Low);
    rig.mux.set_trigger(3, Level::High);
    assert_eq!(rig.rgsr(), 1 << 1, "the second finds the debt outstanding");
    assert_eq!(irq.level(), Level::High, "OIE is set");

    rig.poke(OFF_RGCFR, 1 << 1);
    assert_eq!(rig.rgsr(), 0);
    assert_eq!(irq.level(), Level::Low);
}

#[test]
fn disabling_a_generator_abandons_what_it_owed() {
    let rig = Rig::new(7);
    rig.poke(rgcr(2), 4 | RGCR_GE | (0b01 << RGCR_GPOL_SHIFT));
    rig.mux.set_trigger(4, Level::High);
    rig.mux.set_trigger(4, Level::Low);

    // `GE = 0`, then trigger twice more: a debt cleared with the generator
    // cannot report an overrun the guest could not have caused.
    rig.poke(rgcr(2), 4 | (0b01 << RGCR_GPOL_SHIFT));
    rig.poke(rgcr(2), 4 | RGCR_GE | (0b01 << RGCR_GPOL_SHIFT));
    rig.mux.set_trigger(4, Level::High);
    assert_eq!(rig.rgsr(), 0);
}

#[test]
fn a_synchronized_channel_gates_a_generator_too() {
    let rig = Rig::new(7);
    let ch0 = rig.probe_channel(0);
    // Channel 0 listens to generator 0 but is synchronized with credit for one.
    rig.poke(
        ccr(0),
        Ccr {
            id: 1,
            se: true,
            spol: 0b01,
            nbreq: 0,
            sync_id: 0,
            ..Ccr::default()
        }
        .bits(),
    );
    rig.poke(
        rgcr(0),
        RGCR_GE | (0b01 << RGCR_GPOL_SHIFT) | (2 << RGCR_GNBREQ_SHIFT),
    );

    rig.mux.set_trigger(0, Level::High);
    assert_eq!(ch0.rises(), 0, "no sync event, so no credit");

    rig.mux.set_sync(0, Level::High);
    rig.mux.set_trigger(0, Level::Low);
    rig.mux.set_trigger(0, Level::High);
    assert_eq!(ch0.rises(), 1, "one unit of credit takes one of the three");
}

// ---------------------------------------------------------------------------
// the register face
// ---------------------------------------------------------------------------

#[test]
fn the_control_registers_read_back_only_what_rm0432_makes_writable() {
    let rig = Rig::new(16);
    rig.poke(ccr(0), 0xffff_ffff);
    assert_eq!(
        rig.peek(ccr(0)),
        CCR_MASK,
        "CxCR bits 7, 10-15 and 27-31 are reserved"
    );
    rig.poke(rgcr(0), 0xffff_ffff);
    assert_eq!(
        rig.peek(rgcr(0)),
        RGCR_MASK,
        "RGxCR bits 5-7, 9-15 and 21-31 are reserved"
    );
    // `CSR` and `RGSR` are read-only; the clear registers read as zero.
    rig.poke(OFF_CSR, 0xffff_ffff);
    rig.poke(OFF_RGSR, 0xffff_ffff);
    assert_eq!(rig.csr(), 0);
    assert_eq!(rig.rgsr(), 0);
    assert_eq!(rig.peek(OFF_CFR), 0);
    assert_eq!(rig.peek(OFF_RGCFR), 0);
}

#[test]
fn a_debug_read_changes_nothing_and_a_debug_write_is_refused() {
    let rig = Rig::new(7);
    rig.poke(
        ccr(0),
        Ccr {
            id: 20,
            se: true,
            spol: 0b01,
            sync_id: 0,
            ..Ccr::default()
        }
        .bits(),
    );
    rig.mux.set_sync(0, Level::High);
    rig.mux.set_sync(0, Level::Low);
    rig.mux.set_sync(0, Level::High);
    assert_eq!(rig.csr(), 1);

    let debug = MemAttrs::DEFAULT.with_debug(true);
    let mut bytes = [0u8; 4];
    rig.regs.read(OFF_CSR, &mut bytes, debug).unwrap();
    assert_eq!(
        u32::from_le_bytes(bytes),
        1,
        "SOF reads, and does not clear"
    );
    assert_eq!(rig.csr(), 1);

    assert!(
        rig.regs.write(OFF_CFR, &1u32.to_le_bytes(), debug).is_err(),
        "a debug write to CFR would drop an overrun the guest has not seen"
    );
}

#[test]
fn only_word_accesses_are_allowed() {
    let rig = Rig::new(7);
    assert!(
        rig.regs.read(0, &mut [0u8; 2], MemAttrs::DEFAULT).is_err(),
        "RM0432 §14.5: words only"
    );
    assert!(
        rig.regs.write(0, &[0u8; 1], MemAttrs::DEFAULT).is_err(),
        "RM0432 §14.5: words only"
    );
}

// ---------------------------------------------------------------------------
// pins
// ---------------------------------------------------------------------------

#[test]
fn the_generators_request_lines_have_no_pins() {
    let rig = Rig::new(7);
    // 0 is the idle encoding and 1 to 4 are the four generators'.
    for n in 0..FIRST_EXTERNAL_LINE {
        assert!(
            rig.mux
                .input_index(&format!("{}{n}", pin::REQUEST))
                .is_none(),
            "req{n} is not a board's to wire"
        );
    }
    assert_eq!(
        rig.mux.input_index("req5"),
        Some((Bank::Request, 5)),
        "the peripherals start here"
    );
    assert_eq!(rig.mux.input_index("req127"), Some((Bank::Request, 127)));
    assert!(
        rig.mux.input_index("req128").is_none(),
        "DMAREQ_ID is 7 bits"
    );
    assert_eq!(rig.mux.input_index("sync7"), Some((Bank::Sync, 7)));
    assert!(rig.mux.input_index("sync8").is_none(), "SYNC_ID is 3 bits");
    assert_eq!(rig.mux.input_index("trg31"), Some((Bank::Trigger, 31)));
    assert!(rig.mux.input_index("trg32").is_none(), "SIG_ID is 5 bits");
}

#[test]
fn outputs_beyond_the_instances_channels_are_refused() {
    let rig = Rig::new(7);
    let probe = Probe::new();
    assert!(Device::connect(&rig.mux, "ch7", rig.wire(&probe)).is_err());
    assert!(Device::connect(&rig.mux, "evt7", rig.wire(&probe)).is_err());
    assert!(Device::connect(&rig.mux, "ch6", rig.wire(&probe)).is_ok());
}

#[test]
fn the_schema_declares_every_pin_the_device_answers_to() {
    let schema = schema();
    for port in [
        "req5", "req127", "sync0", "sync7", "trg0", "trg31", "ch0", "evt0", "irq",
    ] {
        assert!(
            schema.port_named(port).is_some(),
            "{port} should be declared"
        );
    }
    assert!(schema.port_named("req128").is_none());
    assert!(schema.port_named("ch16").is_none());
}

// ---------------------------------------------------------------------------
// properties, reset and snapshots
// ---------------------------------------------------------------------------

#[test]
fn channels_defaults_to_seven_and_is_range_checked() {
    let mux = Dmamux::new(&Props::new()).expect("defaults");
    assert_eq!(mux.channels(), 7);

    assert_eq!(
        Dmamux::new(&Props::new().with("channels", Value::from(16u64)))
            .expect("sixteen")
            .channels(),
        16
    );
    assert!(
        Dmamux::new(&Props::new().with("channels", Value::from(17u64))).is_err(),
        "a DMAMUX serves at most 16"
    );
    assert!(Dmamux::new(&Props::new().with("channels", Value::from(0u64))).is_err());
    assert!(
        Dmamux::new(&Props::new().with("invented", Value::from(1u64))).is_err(),
        "an unknown property is refused"
    );
}

#[test]
fn a_reset_clears_every_register_and_drops_every_output() {
    let rig = Rig::new(7);
    let ch0 = rig.probe_channel(0);
    rig.poke(
        ccr(0),
        Ccr {
            id: 9,
            ..Ccr::default()
        }
        .bits(),
    );
    rig.mux.set_request(9, Level::High);
    assert_eq!(ch0.level(), Level::High);

    Device::reset(&rig.mux, ResetKind::Cold);
    assert_eq!(rig.peek(ccr(0)), 0);
    assert_eq!(rig.csr(), 0);
    assert_eq!(rig.rgsr(), 0);
    assert_eq!(
        ch0.level(),
        Level::Low,
        "the wire is told, not merely forgotten"
    );
}

#[test]
fn dmamux_state_survives_save_and_load() {
    let saved = Rig::new(7);
    // A channel mid-burst, a generator mid-debt, and both overrun flags up.
    saved.poke(
        ccr(1),
        Ccr {
            id: 20,
            soie: true,
            ege: true,
            se: true,
            spol: 0b11,
            nbreq: 5,
            sync_id: 2,
        }
        .bits(),
    );
    saved.poke(
        ccr(3),
        Ccr {
            id: 31,
            ..Ccr::default()
        }
        .bits(),
    );
    saved.poke(rgcr(1), 6 | RGCR_GE | RGCR_OIE | (0b01 << RGCR_GPOL_SHIFT));
    saved.mux.set_request(31, Level::High);
    saved.mux.set_sync(2, Level::High);
    saved.mux.set_request(20, Level::High);
    saved.mux.set_trigger(6, Level::High);
    saved.mux.set_trigger(6, Level::Low);
    saved.mux.set_trigger(6, Level::High);
    assert_ne!(saved.rgsr(), 0, "an overrun to carry across");

    let mut shape = MachineShape::new();
    shape.add_device("dmamux", CLASS_NAME).unwrap();
    let mut w = StateWriter::new(shape);
    {
        let mut chunk = w.chunk("dmamux", CLASS_NAME, STATE_VERSION).unwrap();
        Device::save(&saved.mux, &mut chunk).unwrap();
    }
    let bytes = w.to_vec().unwrap();

    let restored = Rig::new(7);
    let reader = StateReader::new(&bytes).unwrap();
    let chunk = reader
        .load("dmamux", CLASS_NAME, STATE_VERSION, &Migrations::new())
        .unwrap();
    Device::load(&restored.mux, &mut chunk.reader()).unwrap();

    // Every register reads the same on both sides — the state hash of this
    // device is its register image plus the latches below.
    let image = |rig: &Rig| -> Vec<u32> { (0..WINDOW / 4).map(|i| rig.peek(i * 4)).collect() };
    assert_eq!(image(&saved), image(&restored));
    // One lock at a time: two `DEVICE` guards alive at once is the lock-order
    // violation this rig exists to catch elsewhere.
    let (credit, owed) = {
        let state = saved.mux.shared.state.lock();
        (state.credit, state.owed)
    };
    {
        let state = restored.mux.shared.state.lock();
        assert_eq!(credit, state.credit);
        assert_eq!(owed, state.owed);
    }

    // And it resumes where the original was: the request line it was
    // forwarding travelled with the state, so no new wire event is needed.
    assert!(
        restored.mux.forwarding(3),
        "line 31 was held high and channel 3 was passing it on"
    );
    // The edge detectors kept their inputs, so the *next* change is an edge
    // rather than a repeat: dropping sync 2 and raising it again overruns.
    restored.mux.set_sync(2, Level::Low);
    restored.mux.set_sync(2, Level::High);
    assert_eq!(restored.csr(), 1 << 1, "credit was still outstanding");
}

#[test]
fn a_snapshot_from_a_different_width_is_refused() {
    let seven = Rig::new(7);
    let mut shape = MachineShape::new();
    shape.add_device("dmamux", CLASS_NAME).unwrap();
    let mut w = StateWriter::new(shape);
    {
        let mut chunk = w.chunk("dmamux", CLASS_NAME, STATE_VERSION).unwrap();
        Device::save(&seven.mux, &mut chunk).unwrap();
    }
    let bytes = w.to_vec().unwrap();

    let five = Rig::new(5);
    let reader = StateReader::new(&bytes).unwrap();
    let chunk = reader
        .load("dmamux", CLASS_NAME, STATE_VERSION, &Migrations::new())
        .unwrap();
    assert!(Device::load(&five.mux, &mut chunk.reader()).is_err());
}

#[test]
fn a_snapshot_cannot_smuggle_a_request_onto_the_idle_line() {
    // Line 0 is `DMAREQ_ID = 0`, "no request". A channel parked on it must
    // stay parked whatever the encoding claims, so the loader forces it low.
    let rig = Rig::new(7);
    let mut state = State::reset();
    state.request[0] = true;
    state.request[9] = true;
    *rig.mux.shared.state.lock() = state;
    rig.mux.shared.refresh();
    assert!(!rig.mux.forwarding(0), "CxCR is still zero");

    let mut shape = MachineShape::new();
    shape.add_device("dmamux", CLASS_NAME).unwrap();
    let mut w = StateWriter::new(shape);
    {
        let mut chunk = w.chunk("dmamux", CLASS_NAME, STATE_VERSION).unwrap();
        Device::save(&rig.mux, &mut chunk).unwrap();
    }
    let bytes = w.to_vec().unwrap();

    let restored = Rig::new(7);
    let reader = StateReader::new(&bytes).unwrap();
    let chunk = reader
        .load("dmamux", CLASS_NAME, STATE_VERSION, &Migrations::new())
        .unwrap();
    Device::load(&restored.mux, &mut chunk.reader()).unwrap();
    assert!(!restored.mux.shared.state.lock().request[0]);
    assert!(
        restored.mux.shared.state.lock().request[9],
        "line 9 survived"
    );
}

#[test]
fn the_class_registers_and_binds_under_its_name() {
    let mut registry = crate::core::Registry::new();
    register(&mut registry).expect("a free name");
    assert!(registry.get(CLASS_NAME).is_some());
    assert_eq!(schema().class, CLASS_NAME);
}

// ---------------------------------------------------------------------------
// the re-entrancy contract
// ---------------------------------------------------------------------------

#[test]
fn an_output_is_driven_with_the_register_lock_released() {
    // The probe takes a `DEVICE`-ranked lock of its own, which the rank
    // checker refuses to nest under the multiplexer's. If `apply` ever drove a
    // wire with `state` held, this test would trip it.
    let rig = Rig::new(7);
    let ch0 = rig.probe_channel(0);
    rig.poke(
        ccr(0),
        Ccr {
            id: 40,
            ..Ccr::default()
        }
        .bits(),
    );
    rig.mux.set_request(40, Level::High);
    assert_eq!(ch0.level(), Level::High);
    assert_eq!(vec![Level::High], *ch0.seen.lock());
}
