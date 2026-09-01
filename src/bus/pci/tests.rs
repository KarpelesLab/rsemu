//! The fabric, the ports and the register file, checked against what the
//! specification and the 82441FX datasheet say they do.

use super::*;

use alloc::string::ToString;

use crate::core::space::{AddressSpace, Region, UnassignedPolicy};
use crate::core::value::Width;

/// A function that remembers every access it was asked for, so a test can
/// assert on the *shape* of a cycle rather than only on its answer.
#[derive(Debug)]
struct Recorder {
    space: Mutex<ConfigSpace>,
    log: Mutex<Vec<(bool, u16, usize, bool)>>,
}

impl Recorder {
    fn new() -> Arc<Recorder> {
        let mut space = ConfigSpace::new();
        space.hardwire(config::VENDOR_ID, 0x8086, 2);
        space.hardwire(config::DEVICE_ID, 0x1237, 2);
        space.hardwire(config::HEADER_TYPE, 0x00, 1);
        space.hardwire(0x40, 0xdead_beef, 4);
        space.allow(0x40, 4);
        Arc::new(Recorder {
            space: Mutex::with_rank(LockRank::DEVICE, space),
            log: Mutex::with_rank(LockRank::LEAF, Vec::new()),
        })
    }
}

impl PciFunction for Recorder {
    fn config_read(&self, offset: u16, dst: &mut [u8], attrs: MemAttrs) {
        self.log
            .lock()
            .push((false, offset, dst.len(), attrs.debug));
        self.space.lock().read(offset, dst);
    }

    fn config_write(&self, offset: u16, src: &[u8], attrs: MemAttrs) {
        self.log.lock().push((true, offset, src.len(), attrs.debug));
        self.space.lock().write(offset, src);
    }
}

/// A bus with one recorder at `00:00.0` and its port pair, mapped at 0xcf8 in
/// an I/O space that reads as ones where nothing is — which is what an ISA bus
/// with pull-ups does, and what the pc-at board declares.
fn rig() -> (Arc<AddressSpace>, Arc<PciBus>, Arc<Recorder>) {
    let bus = Arc::new(PciBus::new());
    let f = Recorder::new();
    bus.attach(Bdf::default(), Arc::clone(&f) as Arc<dyn PciFunction>)
        .expect("nothing is there yet");
    let ports = Arc::new(ConfigPorts::new(Arc::clone(&bus)));
    let space = Arc::new(AddressSpace::new("port", 16).with_unassigned(UnassignedPolicy::ONES));
    space
        .topology()
        .map(
            Region::io(
                "pci.config",
                CONFIG_PORT_WINDOW_LEN,
                ports as Arc<dyn MemOps>,
            ),
            0xcf8,
        )
        .expect("0xcf8 is free");
    (space, bus, f)
}

/// Point `CONFADD` at one register of one function, the way firmware does.
fn select(space: &AddressSpace, bdf: Bdf, register: u16) {
    let value = CONFIG_ENABLE
        | (u32::from(bdf.bus) << 16)
        | (u32::from(bdf.device) << 11)
        | (u32::from(bdf.function) << 8)
        | u32::from(register & 0xfc);
    space
        .write(0xcf8, Width::U32, u64::from(value), MemAttrs::DEFAULT)
        .expect("a Dword write to CONFADD");
}

#[test]
fn an_address_refuses_a_device_number_that_does_not_fit() {
    assert!(Bdf::new(0, 31, 7).is_ok());
    let e = Bdf::new(0, 32, 0).expect_err("five bits").to_string();
    assert!(e.contains("device numbers"), "{e}");
    let e = Bdf::new(0, 0, 8).expect_err("three bits").to_string();
    assert!(e.contains("function numbers"), "{e}");
}

#[test]
fn a_dword_read_of_confdata_reaches_the_function() {
    let (space, _bus, f) = rig();
    select(&space, Bdf::default(), config::VENDOR_ID);
    let v = space
        .read(0xcfc, Width::U32, MemAttrs::DEFAULT)
        .expect("a Dword read");
    assert_eq!(v, 0x1237_8086, "device and vendor in one Dword");
    let log = f.log.lock();
    assert_eq!(log.as_slice(), &[(false, 0x00, 4, false)]);
}

#[test]
fn a_byte_or_word_access_lands_on_the_right_byte_of_the_dword() {
    // The rule that a Dword-only model gets wrong: the low two bits of the I/O
    // address pick which bytes inside the addressed register are touched, and
    // firmware reads a header type as a byte at 0xcfe.
    let (space, _bus, _f) = rig();
    select(&space, Bdf::default(), config::VENDOR_ID);
    assert_eq!(
        space.read(0xcfe, Width::U16, MemAttrs::DEFAULT),
        Ok(0x1237),
        "the device ID as a word at 0xcfe"
    );
    assert_eq!(
        space.read(0xcfd, Width::U8, MemAttrs::DEFAULT),
        Ok(0x80),
        "the high byte of the vendor ID"
    );
    select(&space, Bdf::default(), config::CACHE_LINE_SIZE);
    assert_eq!(
        space.read(0xcfe, Width::U8, MemAttrs::DEFAULT),
        Ok(0x00),
        "header type 0, a byte at 0xcfe"
    );
}

#[test]
fn confadd_is_dword_only() {
    // 82441FX §3.1.1: a byte or word reference passes through to the PCI bus,
    // where nothing on this board claims it. So the latch does not move and the
    // read comes back as ones.
    let (space, _bus, _f) = rig();
    select(&space, Bdf::default(), config::VENDOR_ID);
    let before = space
        .read(0xcf8, Width::U32, MemAttrs::DEFAULT)
        .expect("a Dword read of the latch");
    space
        .write(0xcf9, Width::U8, 0x55, MemAttrs::DEFAULT)
        .expect("an unclaimed I/O write is not a fault");
    assert_eq!(
        space.read(0xcf8, Width::U32, MemAttrs::DEFAULT),
        Ok(before),
        "a narrow write did not touch the latch"
    );
    assert_eq!(
        space.read(0xcfa, Width::U8, MemAttrs::DEFAULT),
        Ok(0xff),
        "a narrow read of CONFADD is an unclaimed cycle"
    );
}

#[test]
fn a_cycle_with_the_enable_bit_clear_is_not_a_cycle() {
    let (space, _bus, f) = rig();
    space
        .write(0xcf8, Width::U32, 0x0000_0000, MemAttrs::DEFAULT)
        .expect("clearing CONFADD");
    assert_eq!(
        space.read(0xcfc, Width::U32, MemAttrs::DEFAULT),
        Ok(0xffff_ffff),
        "with CONE clear the ports are I/O space with nothing behind them"
    );
    assert!(f.log.lock().is_empty(), "the function saw nothing");
}

#[test]
fn an_empty_address_master_aborts_and_reads_as_ones() {
    // How firmware discovers an empty slot, so it is the interesting case
    // rather than an error path.
    let (space, _bus, _f) = rig();
    select(&space, Bdf::new(0, 3, 0).expect("a legal address"), 0);
    assert_eq!(
        space.read(0xcfc, Width::U32, MemAttrs::DEFAULT),
        Ok(0xffff_ffff)
    );
    // And a write there is dropped rather than faulted.
    space
        .write(0xcfc, Width::U32, 0x1234_5678, MemAttrs::DEFAULT)
        .expect("a write into a master abort is not a fault");
}

#[test]
fn the_address_decode_names_bus_device_function_and_register() {
    let bus = Arc::new(PciBus::new());
    let f = Recorder::new();
    let at = Bdf::new(2, 17, 5).expect("a legal address");
    bus.attach(at, Arc::clone(&f) as Arc<dyn PciFunction>)
        .expect("nothing is there");
    let ports = ConfigPorts::new(Arc::clone(&bus));
    ports.set_address(CONFIG_ENABLE | (2 << 16) | (17 << 11) | (5 << 8) | 0x40);
    let mut dst = [0u8; 4];
    ports
        .read(4, &mut dst, MemAttrs::DEFAULT)
        .expect("a Dword read of CONFDATA");
    assert_eq!(u32::from_le_bytes(dst), 0xdead_beef);
    assert_eq!(f.log.lock().as_slice(), &[(false, 0x40, 4, false)]);
}

#[test]
fn a_debug_write_anywhere_in_the_window_is_refused_and_a_debug_read_is_not() {
    // Moving the address latch under the guest's feet would send its next
    // CONFDATA access to a different device, and a config write is how a BAR
    // moves and how a shadow window is switched. Reading either changes
    // nothing.
    let (space, _bus, f) = rig();
    select(&space, Bdf::default(), config::VENDOR_ID);
    assert!(
        space.write(0xcf8, Width::U32, 0, MemAttrs::DEBUG).is_err(),
        "a debugger may not move the address latch"
    );
    assert!(
        space.write(0xcfc, Width::U32, 0, MemAttrs::DEBUG).is_err(),
        "a debugger may not write configuration space"
    );
    assert!(space.read(0xcf8, Width::U32, MemAttrs::DEBUG).is_ok());
    assert!(space.read(0xcfc, Width::U32, MemAttrs::DEBUG).is_ok());
    assert!(
        f.log.lock().iter().all(|entry| !entry.0),
        "no write reached the function"
    );
}

#[test]
fn debug_attributes_reach_the_function() {
    let (space, _bus, f) = rig();
    select(&space, Bdf::default(), config::VENDOR_ID);
    space
        .read(0xcfc, Width::U32, MemAttrs::DEBUG)
        .expect("a debug read");
    assert_eq!(
        f.log.lock().as_slice(),
        &[(false, 0x00, 4, true)],
        "the function was told this was a debugger"
    );
}

#[test]
fn two_functions_cannot_share_one_address() {
    let bus = PciBus::new();
    let a = Recorder::new();
    let b = Recorder::new();
    bus.attach(Bdf::default(), a as Arc<dyn PciFunction>)
        .expect("the first one");
    let e = bus
        .attach(Bdf::default(), b as Arc<dyn PciFunction>)
        .expect_err("the second one")
        .to_string();
    assert!(e.contains("cannot share"), "{e}");
}

#[test]
fn addresses_come_back_in_address_order() {
    // Enumeration order is guest-visible, so it is a BTreeMap and this asserts
    // it rather than trusting it.
    let bus = PciBus::new();
    for (b, d, f) in [(1u8, 0u8, 0u8), (0, 5, 1), (0, 0, 0), (0, 5, 0)] {
        bus.attach(
            Bdf::new(b, d, f).expect("legal"),
            Recorder::new() as Arc<dyn PciFunction>,
        )
        .expect("distinct");
    }
    let got: Vec<(u8, u8, u8)> = bus
        .addresses()
        .into_iter()
        .map(|a| (a.bus, a.device, a.function))
        .collect();
    assert_eq!(got, [(0, 0, 0), (0, 5, 0), (0, 5, 1), (1, 0, 0)]);
}

#[test]
fn detaching_leaves_a_master_abort_behind() {
    let bus = PciBus::new();
    let f = Recorder::new();
    bus.attach(Bdf::default(), f as Arc<dyn PciFunction>)
        .expect("attaches");
    assert!(bus.detach(Bdf::default()));
    assert!(!bus.detach(Bdf::default()), "only once");
    let mut dst = [0u8; 4];
    bus.config_read(Bdf::default(), 0, &mut dst, MemAttrs::DEFAULT);
    assert_eq!(dst, [0xff; 4]);
}

#[test]
fn the_register_file_honours_its_write_mask() {
    let mut cs = ConfigSpace::new();
    cs.hardwire(config::VENDOR_ID, 0x8086, 2);
    cs.hardwire(0x59, 0x00, 1);
    cs.allow(0x59, 1);

    assert!(!cs.write(config::VENDOR_ID, &[0, 0]), "read-only");
    let mut dst = [0u8; 2];
    cs.read(config::VENDOR_ID, &mut dst);
    assert_eq!(u16::from_le_bytes(dst), 0x8086);

    assert!(cs.write(0x59, &[0x30]), "writable, and it changed");
    assert!(
        !cs.write(0x59, &[0x30]),
        "the same value again changed nothing"
    );
    assert_eq!(cs.byte(0x59), 0x30);
}

#[test]
fn a_snapshot_restores_the_writable_bytes_and_not_the_hardwired_ones() {
    let mut cs = ConfigSpace::new();
    cs.hardwire(config::VENDOR_ID, 0x8086, 2);
    cs.allow(0x59, 1);
    cs.write(0x59, &[0x33]);
    let saved: Vec<u8> = cs.bytes().to_vec();

    // A different build of the same class, and a snapshot that claims a
    // different vendor ID: the vendor is this model's, the PAM byte is the
    // run's.
    let mut other = ConfigSpace::new();
    other.hardwire(config::VENDOR_ID, 0x8086, 2);
    other.allow(0x59, 1);
    let mut tampered = saved.clone();
    tampered[0] = 0x00;
    tampered[1] = 0x00;
    other.restore(&tampered);
    assert_eq!(other.byte(0x59), 0x33, "the writable byte came back");
    let mut dst = [0u8; 2];
    other.read(config::VENDOR_ID, &mut dst);
    assert_eq!(
        u16::from_le_bytes(dst),
        0x8086,
        "a snapshot cannot change a vendor ID"
    );
}

#[test]
fn an_access_straddling_the_end_of_confdata_is_refused() {
    let bus = Arc::new(PciBus::new());
    let ports = ConfigPorts::new(bus);
    ports.set_address(CONFIG_ENABLE);
    let mut dst = [0u8; 2];
    assert!(
        ports.read(7, &mut dst, MemAttrs::DEFAULT).is_err(),
        "0xcff plus one byte runs off the end of the window"
    );
}

#[test]
fn the_latch_survives_a_round_trip_and_a_reset_clears_it() {
    let ports = ConfigPorts::new(Arc::new(PciBus::new()));
    ports.set_address(0xffff_ffff);
    assert_eq!(
        ports.address(),
        CONFADD_MASK,
        "the reserved bits never latch, however a snapshot spells them"
    );
    ports.reset();
    assert_eq!(ports.address(), 0);
}
