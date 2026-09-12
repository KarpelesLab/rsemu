//! What the firewall does, judged through a real address space.
//!
//! Every test drives the `bus` region rather than the device's methods, because
//! the peripheral's whole behaviour is a function of the accesses that pass
//! through it — including the ones that land nowhere near a protected segment,
//! which is how it learns the processor has left the code.

use super::*;
use crate::core::props::Value;
use crate::core::registry::Registry;
use crate::core::space::{AccessPurpose, Mapping, Perms, RamStore, RequesterId};
use crate::core::state::{MachineShape, Migrations, StateReader, StateWriter};
use crate::core::sync::{AtomicU32, Ordering};
use crate::core::wire::{Wire, WireIdAllocator};

/// Where the rig puts its flash.
const FLASH: u64 = 0x0800_0000;
/// Where the rig puts its SRAM.
const SRAM: u64 = 0x2000_0000;
/// Where the rig puts the firewall's own registers.
const FW: u64 = 0x4001_1c00;
/// `FW_CSSA`, which is the block's first register.
const FW_CSSA: u64 = FW;

/// The code segment: 0x08001000, 0x400 bytes.
const CSSA: u32 = 0x0000_1000;
/// Its length.
const CSL: u32 = 0x0000_0400;
/// The non-volatile data segment: 0x08002000, 0x200 bytes.
const NVDSSA: u32 = 0x0000_2000;
/// Its length.
const NVDSL: u32 = 0x0000_0200;
/// The volatile data segment: 0x20001000, 0x80 bytes.
const VDSSA: u32 = 0x0000_1000;
/// Its length.
const VDSL: u32 = 0x0000_0080;

/// Counts rising edges on the reset line.
#[derive(Debug, Default)]
struct ResetProbe {
    pulses: AtomicU32,
}

impl WireSink for ResetProbe {
    fn set_level(&self, _src: WireId, _line: u32, level: Level) {
        if level.is_high() {
            self.pulses.fetch_add(1, Ordering::Relaxed);
        }
    }
}

/// A firewall in front of a flash and an SRAM, with its reset line watched.
struct Rig {
    fw: Arc<Firewall>,
    /// What the processor sees: nothing but the filter.
    cpu: Arc<AddressSpace>,
    resets: Arc<ResetProbe>,
}

impl Rig {
    fn new() -> Rig {
        let fw = Arc::new(Firewall::build(FLASH, SRAM, 1 << 32));
        let mem = Arc::new(AddressSpace::new("mem", 32));
        {
            let mut topo = mem.topology();
            topo.map(
                Arc::new(Region::ram("flash", Arc::new(RamStore::new(0x1_0000)))) as RegionRef,
                FLASH,
            )
            .expect("flash");
            topo.map(
                Arc::new(Region::ram("sram", Arc::new(RamStore::new(0x4000)))) as RegionRef,
                SRAM,
            )
            .expect("sram");
            topo.map(Device::region(fw.as_ref(), "regs").expect("regs"), FW)
                .expect("the registers");
        }
        fw.attach_bus(&mem);

        let cpu = Arc::new(AddressSpace::new("cpubus", 32));
        {
            let mut topo = cpu.topology();
            topo.map_with(
                Mapping::new(Device::region(fw.as_ref(), "bus").expect("bus"), 0)
                    .with_perms(Perms::RWX),
            )
            .expect("the filter covers everything the core can address");
        }

        let ids = WireIdAllocator::new();
        let id = ids.alloc();
        let resets = Arc::new(ResetProbe::default());
        let wire = Wire::builder()
            .source(id)
            .sink(Arc::clone(&resets) as Arc<dyn WireSink>, 0)
            .build_shared();
        Device::connect(fw.as_ref(), "reset", WireSource::new(wire, id)).expect("reset");

        Rig { fw, cpu, resets }
    }

    /// Program the three segments and switch the firewall on.
    fn arm(&self) {
        self.poke(FW_CSSA, CSSA);
        self.poke(FW + 0x04, CSL);
        self.poke(FW + 0x08, NVDSSA);
        self.poke(FW + 0x0c, NVDSL);
        self.poke(FW + 0x10, VDSSA);
        self.poke(FW + 0x14, VDSL);
        self.fw.set_fwdis(false);
    }

    /// An ordinary store, through the filter.
    fn poke(&self, addr: u64, value: u32) {
        self.cpu
            .write(addr, Width::U32, u64::from(value), MemAttrs::DEFAULT)
            .expect("a legal store");
    }

    /// An ordinary load, through the filter.
    fn read(&self, addr: u64) -> MemResult<u64> {
        self.cpu.read(addr, Width::U32, MemAttrs::DEFAULT)
    }

    /// A store, through the filter, whose verdict is the caller's business.
    fn write(&self, addr: u64, value: u32) -> MemResult {
        self.cpu
            .write(addr, Width::U32, u64::from(value), MemAttrs::DEFAULT)
    }

    /// An instruction fetch at `addr`.
    fn fetch(&self, addr: u64) -> MemResult<u64> {
        self.cpu.read(
            addr,
            Width::U32,
            MemAttrs {
                purpose: AccessPurpose::FETCH,
                ..MemAttrs::DEFAULT
            },
        )
    }

    /// How many times the reset line has been pulsed.
    fn resets(&self) -> u32 {
        self.resets.pulses.load(Ordering::Relaxed)
    }
}

/// The code segment's first word — the call gate.
const GATE: u64 = FLASH + CSSA as u64;
/// Somewhere in the middle of the code segment.
const INSIDE_CODE: u64 = GATE + 8;
/// Unprotected flash, where the caller lives.
const OUTSIDE: u64 = FLASH + 0x100;
/// The first word of the non-volatile data segment.
const NVDS: u64 = FLASH + NVDSSA as u64;
/// The first word of the volatile data segment.
const VDS: u64 = SRAM + VDSSA as u64;

#[test]
fn nothing_is_checked_until_fwdis_is_cleared() {
    // Reset value: `FWDIS` high, so the firewall is idle and the segments —
    // which are all zero-length anyway — fence nothing.
    let rig = Rig::new();
    assert!(rig.fw.disabled());
    rig.poke(FW_CSSA, CSSA);
    rig.poke(FW + 0x04, CSL);
    assert!(rig.fetch(INSIDE_CODE).is_ok());
    assert_eq!(rig.resets(), 0);
}

#[test]
fn a_jump_into_the_code_segment_that_is_not_the_call_gate_resets_the_machine() {
    // The failure the peripheral exists for: untrusted code branching past the
    // gate into the middle of the protected routine.
    let rig = Rig::new();
    rig.arm();
    assert_eq!(rig.fetch(INSIDE_CODE), Err(BusError::Protected));
    assert_eq!(rig.resets(), 1, "an illegal entry is a reset, not a fault");
    assert!(!rig.fw.is_open());
}

#[test]
fn entering_through_the_call_gate_opens_the_firewall() {
    // The gate is the *first word* of the code segment, and going through it
    // is what makes the protected data reachable.
    let rig = Rig::new();
    rig.poke(NVDS, 0xc0ff_ee00);
    rig.poke(VDS, 0x5eed_0001);
    rig.arm();

    assert_eq!(rig.read(NVDS), Err(BusError::Protected), "closed");
    assert_eq!(rig.resets(), 1);

    assert!(rig.fetch(GATE).is_ok(), "the call gate");
    assert!(rig.fw.is_open());
    assert_eq!(rig.read(NVDS).ok(), Some(0xc0ff_ee00));
    assert_eq!(rig.read(VDS).ok(), Some(0x5eed_0001));
    assert!(rig.fetch(INSIDE_CODE).is_ok(), "and the code runs");
    assert_eq!(rig.resets(), 1, "nothing else reset the machine");
}

#[test]
fn reading_the_nvds_from_outside_while_closed_resets() {
    let rig = Rig::new();
    rig.arm();
    assert_eq!(rig.read(NVDS), Err(BusError::Protected));
    assert_eq!(rig.resets(), 1);
    // A *write* is no better, and neither is a fetch: the segment is fenced in
    // every direction.
    assert_eq!(rig.write(NVDS, 1), Err(BusError::Protected));
    assert_eq!(rig.fetch(NVDS), Err(BusError::Protected));
    assert_eq!(rig.resets(), 3);
}

#[test]
fn an_illegal_read_does_not_hand_over_the_bytes() {
    // The reset is the headline, but the byte is the point: a model that
    // completed the access and *then* reset would have published the secret.
    let rig = Rig::new();
    rig.poke(NVDS, 0xdead_beef);
    rig.arm();
    assert!(rig.read(NVDS).is_err());
}

#[test]
fn leaving_the_code_segment_without_fpa_resets_and_with_it_closes() {
    // RM0351 §4.4.7, and ST's own HAL: "when FPA bit is set, any code executed
    // outside the protected segment will close the Firewall"; when it is
    // clear, that same fetch "will generate a system reset".
    let rig = Rig::new();
    rig.arm();
    assert!(rig.fetch(GATE).is_ok());
    assert!(rig.fw.is_open());
    assert_eq!(rig.fetch(OUTSIDE), Err(BusError::Protected), "no pre-arm");
    assert_eq!(rig.resets(), 1);

    // Again, this time through the documented exit.
    let rig = Rig::new();
    rig.arm();
    assert!(rig.fetch(GATE).is_ok());
    rig.poke(FW + 0x20, CR_FPA);
    assert!(rig.fetch(OUTSIDE).is_ok(), "the pre-armed exit");
    assert_eq!(rig.resets(), 0);
    assert!(!rig.fw.is_open(), "and it closed behind itself");
    assert_eq!(
        rig.read(FW + 0x20).ok(),
        Some(0),
        "the hardware clears FPA as it closes, so the next exit needs its own"
    );
    // Which it does: the protected data is out of reach again.
    assert_eq!(rig.read(NVDS), Err(BusError::Protected));
}

#[test]
fn returning_from_the_code_segment_closes_the_firewall_again() {
    // The whole call-and-return, twice, which is what a trusted-firmware
    // service looks like from the caller's side.
    let rig = Rig::new();
    rig.poke(NVDS, 0x1234_5678);
    rig.arm();
    for _ in 0..2 {
        assert!(rig.fetch(GATE).is_ok());
        assert_eq!(rig.read(NVDS).ok(), Some(0x1234_5678));
        rig.poke(FW + 0x20, CR_FPA);
        assert!(rig.fetch(OUTSIDE).is_ok());
        assert!(!rig.fw.is_open());
        assert_eq!(rig.read(NVDS), Err(BusError::Protected));
    }
    assert_eq!(rig.resets(), 2, "one per attempt from outside, and no more");
}

#[test]
fn a_data_access_outside_the_segments_does_not_close_an_open_firewall() {
    // Only a *fetch* moves the state machine. Protected code that loads from
    // an unprotected buffer is doing exactly what it is there for.
    let rig = Rig::new();
    rig.arm();
    assert!(rig.fetch(GATE).is_ok());
    assert!(rig.read(OUTSIDE).is_ok());
    assert!(rig.write(OUTSIDE, 7).is_ok());
    assert!(rig.fw.is_open(), "a load closed the firewall");
    assert_eq!(rig.resets(), 0);
}

#[test]
fn the_vds_is_readable_from_outside_when_vds_is_set_and_not_otherwise() {
    let rig = Rig::new();
    rig.poke(VDS, 0xabcd_0123);
    rig.arm();
    assert_eq!(rig.read(VDS), Err(BusError::Protected));
    assert_eq!(rig.resets(), 1);

    // `FW_CR.VDS`: the volatile data segment is shared with non-protected code.
    rig.poke(FW + 0x20, CR_VDS);
    assert_eq!(rig.read(VDS).ok(), Some(0xabcd_0123));
    assert!(rig.write(VDS, 4).is_ok());
    assert_eq!(rig.resets(), 1);

    // Shared is not executable: that is `VDE`, and it is a separate bit.
    assert_eq!(rig.fetch(VDS), Err(BusError::Protected));
    assert_eq!(rig.resets(), 2);
    rig.poke(FW + 0x20, CR_VDS | CR_VDE);
    assert!(rig.fetch(VDS).is_ok());
}

#[test]
fn vde_makes_the_volatile_segment_part_of_the_protected_code() {
    // With `VDE` set, a fetch in the volatile data segment has not left the
    // protected code, so the firewall stays open — which is the point of
    // running a routine out of RAM.
    let rig = Rig::new();
    rig.arm();
    rig.poke(FW + 0x20, CR_VDE);
    assert!(rig.fetch(GATE).is_ok());
    assert!(rig.fetch(VDS).is_ok());
    assert!(rig.fw.is_open());
    assert_eq!(rig.resets(), 0);
    // And leaving *that* still needs the pre-arm.
    assert_eq!(rig.fetch(OUTSIDE), Err(BusError::Protected));
}

#[test]
fn a_dma_write_into_a_closed_vds_resets_the_machine() {
    // A bus master is judged by address alone — it never fetches, so nothing
    // about the state machine is special-cased for it. The board reaches this
    // by mapping the same `bus` region into the master's space.
    let rig = Rig::new();
    rig.arm();
    let dma = Arc::new(AddressSpace::new("dmabus", 32));
    {
        let mut topo = dma.topology();
        topo.map(Device::region(rig.fw.as_ref(), "bus").expect("bus"), 0)
            .expect("the master sees the filter too");
    }
    let attrs = MemAttrs {
        requester: RequesterId::ANONYMOUS,
        ..MemAttrs::DEFAULT
    };
    assert_eq!(
        dma.write(VDS, Width::U32, 0x99, attrs),
        Err(BusError::Protected)
    );
    assert_eq!(rig.resets(), 1);

    // And opening the firewall does not open it for the master's sake either —
    // it is open for *everyone*, which is what the manual's "shared" bit is
    // for and why a design puts DMA buffers outside the segment.
    assert!(rig.fetch(GATE).is_ok());
    assert!(dma.write(VDS, Width::U32, 0x99, attrs).is_ok());
}

#[test]
fn segment_registers_are_read_only_once_fwdis_is_cleared() {
    // RM0351 §4.4: the segment registers are writable only while the firewall
    // is disabled. `FW_CR` is not one of them — `FPA` has to be settable from
    // inside the protected code.
    let rig = Rig::new();
    rig.arm();
    rig.poke(FW_CSSA, 0x0000_4000);
    assert_eq!(rig.read(FW_CSSA).ok(), Some(u64::from(CSSA)), "CSSA moved");
    rig.poke(FW + 0x14, 0x0000_ffc0);
    assert_eq!(rig.read(FW + 0x14).ok(), Some(u64::from(VDSL)));
    rig.poke(FW + 0x20, CR_FPA);
    assert_eq!(
        rig.read(FW + 0x20).ok(),
        Some(u64::from(CR_FPA)),
        "CR is not"
    );
}

#[test]
fn the_register_fields_are_granular_and_the_reserved_bits_read_zero() {
    // 256 bytes for the two flash segments, 64 for the volatile one, and the
    // bits above the field belong to nobody (RM0351 §4.4.1–§4.4.6).
    let rig = Rig::new();
    rig.poke(FW_CSSA, 0xffff_ffff);
    assert_eq!(rig.read(FW_CSSA).ok(), Some(u64::from(SSA_MASK)));
    rig.poke(FW + 0x04, 0xffff_ffff);
    assert_eq!(rig.read(FW + 0x04).ok(), Some(u64::from(SL_MASK)));
    rig.poke(FW + 0x10, 0xffff_ffff);
    assert_eq!(rig.read(FW + 0x10).ok(), Some(u64::from(VDS_MASK)));
    rig.poke(FW + 0x20, 0xffff_ffff);
    assert_eq!(rig.read(FW + 0x20).ok(), Some(u64::from(CR_MASK)));
}

#[test]
fn a_zero_length_segment_is_disabled() {
    // Which is why the reset state protects nothing: every length is zero.
    let rig = Rig::new();
    rig.poke(FW_CSSA, CSSA);
    rig.poke(FW + 0x08, NVDSSA);
    rig.fw.set_fwdis(false);
    assert!(rig.fetch(INSIDE_CODE).is_ok());
    assert!(rig.read(NVDS).is_ok());
    assert_eq!(rig.resets(), 0);
}

// The only test here that needs a *second* device. `syscfg` lives behind
// `dev-stm32-exti`, so without this gate a build of `dev-stm32-firewall` alone
// fails to compile its tests — which `cargo build` does not notice and the
// feature sweep's `cargo test` does.
#[cfg(feature = "dev-stm32-exti")]
#[test]
fn fwdis_from_a_real_syscfg_is_what_switches_the_firewall_on() {
    // The seam the issue is about: `SYSCFG_CFGR1.FWDIS` had no reader, so
    // clearing it enabled no protection. Two real devices and a real wire.
    use crate::dev::stm32::syscfg::{self, Syscfg};

    let rig = Rig::new();
    rig.poke(FW_CSSA, CSSA);
    rig.poke(FW + 0x04, CSL);

    let syscfg = Syscfg::build(syscfg::Variant::L4, 8, 0);
    let ids = WireIdAllocator::new();
    let id = ids.alloc();
    let pin = Device::sink(rig.fw.as_ref(), "fwdis", &[id]).expect("the firewall has one");
    let wire = Wire::builder()
        .source(id)
        .sink(Arc::clone(&pin.sink), pin.line)
        .build_shared();
    Device::connect(&syscfg, "fwdis", WireSource::new(wire, id)).expect("SYSCFG drives it");

    // `FWDIS` resets high, and connecting published that, so the firewall is
    // still idle.
    assert!(rig.fw.disabled());
    assert!(rig.fetch(INSIDE_CODE).is_ok());

    // Clearing it — the one write a trusted bootloader makes — arms the fence.
    let sys = Arc::new(AddressSpace::new("apb2", 32));
    sys.topology()
        .map(Device::region(&syscfg, "regs").expect("regs"), 0x4001_0000)
        .expect("SYSCFG maps");
    sys.write(0x4001_0004, Width::U32, 0, MemAttrs::DEFAULT)
        .expect("CFGR1");
    assert!(!rig.fw.disabled());
    assert_eq!(rig.fetch(INSIDE_CODE), Err(BusError::Protected));
    assert_eq!(rig.resets(), 1);

    // And only a system reset puts it back, which is `FWDIS`'s own latch.
    Device::reset(&syscfg, ResetKind::Cold);
    assert!(rig.fw.disabled());
}

#[test]
fn a_debug_access_neither_resets_nor_moves_the_state_machine() {
    // `ROADMAP.md` §15, invariant 5. A debugger that rebooted the machine by
    // looking at the protected flash would be unusable, and one whose read
    // counted as a call-gate entry would be worse.
    let rig = Rig::new();
    rig.poke(NVDS, 0x0bad_c0de);
    rig.arm();
    assert_eq!(
        rig.cpu.read(NVDS, Width::U32, MemAttrs::DEBUG).ok(),
        Some(0x0bad_c0de)
    );
    assert_eq!(rig.resets(), 0);
    assert!(!rig.fw.is_open());

    let peek = MemAttrs {
        purpose: AccessPurpose::FETCH,
        ..MemAttrs::DEBUG
    };
    assert!(rig.cpu.read(GATE, Width::U32, peek).is_ok());
    assert!(!rig.fw.is_open(), "a debugger's peek opened the firewall");

    // And a debug *write* to the registers is refused outright rather than
    // silently moving the fence.
    assert!(
        rig.cpu
            .write(FW + 0x20, Width::U32, 1, MemAttrs::DEBUG)
            .is_err()
    );
}

#[test]
fn an_unbound_filter_answers_rather_than_panicking() {
    // Constructed and mapped but never bound: the region has no memory behind
    // it and says so.
    let fw = Firewall::build(FLASH, SRAM, 1 << 32);
    let space = Arc::new(AddressSpace::new("cpubus", 32));
    space
        .topology()
        .map(Device::region(&fw, "bus").expect("bus"), 0)
        .expect("maps");
    assert_eq!(
        space.read(FLASH, Width::U32, MemAttrs::DEFAULT),
        Err(BusError::Unassigned)
    );
}

#[test]
fn only_a_full_word_is_a_legal_register_access() {
    let fw = Firewall::build(FLASH, SRAM, 1 << 32);
    let space = Arc::new(AddressSpace::new("apb2", 32));
    space
        .topology()
        .map(Device::region(&fw, "regs").expect("regs"), FW)
        .expect("maps");
    assert_eq!(
        space.read(FW, Width::U8, MemAttrs::DEFAULT),
        Err(BusError::BadAccess)
    );
    assert!(space.read(FW, Width::U32, MemAttrs::DEFAULT).is_ok());
}

#[test]
fn a_reset_takes_it_back_to_idle() {
    let rig = Rig::new();
    rig.arm();
    assert!(rig.fetch(GATE).is_ok());
    assert!(rig.fw.is_open());
    Device::reset(rig.fw.as_ref(), ResetKind::Warm);
    assert!(rig.fw.disabled(), "FWDIS is high again");
    assert!(!rig.fw.is_open());
    assert_eq!(rig.read(FW_CSSA).ok(), Some(0), "and the segments are gone");
}

#[test]
fn a_snapshot_round_trips_to_identical_state() {
    let saved = Rig::new();
    saved.arm();
    saved.poke(FW + 0x20, CR_VDS | CR_VDE);
    assert!(saved.fetch(GATE).is_ok());
    assert!(saved.fw.is_open());

    let mut shape = MachineShape::new();
    shape.add_device("fw", CLASS_NAME).unwrap();
    let mut w = StateWriter::new(shape);
    {
        let mut chunk = w.chunk("fw", CLASS_NAME, STATE_VERSION).unwrap();
        Device::save(saved.fw.as_ref(), &mut chunk).unwrap();
    }
    let bytes = w.to_vec().unwrap();

    let restored = Rig::new();
    let reader = StateReader::new(&bytes).unwrap();
    let chunk = reader
        .load("fw", CLASS_NAME, STATE_VERSION, &Migrations::new())
        .unwrap();
    Device::load(restored.fw.as_ref(), &mut chunk.reader()).unwrap();

    for offset in [0x00, 0x04, 0x08, 0x0c, 0x10, 0x14, 0x20] {
        assert_eq!(
            restored.read(FW + offset).ok(),
            saved.read(FW + offset).ok(),
            "register {offset:#x}"
        );
    }
    // The two bits with no register came across too: the firewall is still on
    // and still open, so the protected data is still reachable and the next
    // exit still needs a pre-arm.
    assert!(!restored.fw.disabled());
    assert!(restored.fw.is_open());
    assert!(restored.read(NVDS).is_ok());
    assert_eq!(restored.fetch(OUTSIDE), Err(BusError::Protected));
}

#[test]
fn a_snapshot_of_a_differently_placed_firewall_is_refused() {
    let saved = Firewall::build(FLASH, SRAM, 1 << 32);
    let mut shape = MachineShape::new();
    shape.add_device("fw", CLASS_NAME).unwrap();
    let mut w = StateWriter::new(shape);
    {
        let mut chunk = w.chunk("fw", CLASS_NAME, STATE_VERSION).unwrap();
        Device::save(&saved, &mut chunk).unwrap();
    }
    let bytes = w.to_vec().unwrap();
    let reader = StateReader::new(&bytes).unwrap();
    let chunk = reader
        .load("fw", CLASS_NAME, STATE_VERSION, &Migrations::new())
        .unwrap();
    let other = Firewall::build(0x0801_0000, SRAM, 1 << 32);
    assert!(Device::load(&other, &mut chunk.reader()).is_err());
}

#[test]
fn a_property_this_class_does_not_know_is_a_typo() {
    assert!(Firewall::new(&Props::new()).is_ok());
    assert!(
        Firewall::new(&Props::new().with("flash_base", Value::from(0x0800_0000u64))).is_err(),
        "an underscore is not a hyphen"
    );
    assert!(Firewall::new(&Props::new().with("bus-size", Value::from(0u64))).is_err());
}

#[test]
fn the_class_is_registrable_and_constructs_through_the_registry() {
    let mut reg = Registry::new();
    register(&mut reg).unwrap();
    assert!(register(&mut reg).is_err(), "twice is a collision");
    let device = reg.create(CLASS_NAME, &Props::new()).unwrap();
    assert_eq!(device.class().name, CLASS_NAME);
}

#[test]
fn the_schema_and_the_device_agree_about_pins_and_regions() {
    let fw = Firewall::build(FLASH, SRAM, 1 << 32);
    let schema = schema();
    assert!(schema.port_named("reset").is_some());
    assert!(schema.port_named("fwdis").is_some());
    assert!(schema.port_named("irq").is_none());
    assert!(Device::sink(&fw, "fwdis", &[WireId::new(1)]).is_some());
    assert!(Device::sink(&fw, "reset", &[WireId::new(1)]).is_none());
    for name in ["", "regs", "bus"] {
        assert!(Device::region(&fw, name).is_some(), "{name}");
    }
    assert!(Device::region(&fw, "code").is_none());
}
