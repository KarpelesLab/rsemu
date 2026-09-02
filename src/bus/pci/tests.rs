//! The fabric, the ports and the register file, checked against what the
//! specification and the 82441FX datasheet say they do.

use super::*;

use alloc::string::ToString;
use alloc::vec;

use crate::core::space::{AddressSpace, Perms, Region, UnassignedPolicy};
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
    let bus = Arc::new(PciBus::new());
    let ports = ConfigPorts::new(Arc::clone(&bus));
    ports.set_address(0xffff_ffff);
    assert_eq!(
        ports.address(),
        CONFADD_MASK,
        "the reserved bits never latch, however a snapshot spells them"
    );
    ports.reset();
    assert_eq!(ports.address(), 0);
}

// ---------------------------------------------------------------------------
// base address registers
// ---------------------------------------------------------------------------

/// Read one register as a Dword, the way firmware does.
fn bar_dword(bars: &Bars, offset: u16) -> u32 {
    let mut dst = [0u8; 4];
    bars.config_read(offset, &mut dst);
    u32::from_le_bytes(dst)
}

/// Write one register as a Dword, reporting whether a latch moved.
fn set_bar(bars: &Bars, offset: u16, value: u32) -> bool {
    bars.config_write(offset, &value.to_le_bytes())
}

/// A store to hang off a window, so a mapping has something behind it.
fn window_region(len: u64, byte: u8) -> crate::core::space::RegionRef {
    Arc::new(Region::rom(
        "window",
        Arc::new(crate::core::space::RomStore::new(alloc::vec![
            byte;
            len as usize
        ])),
        crate::core::space::RomWrite::Ignore,
    ))
}

#[test]
fn sizing_a_memory_bar_reads_back_the_size_mask() {
    // Rev 2.1 §6.2.5.1: write all ones, read back zeroes in the don't-care
    // address bits. A 64 KiB window therefore answers 0xffff0000, and the
    // format bits below bit 4 come back as this register's own.
    let bars = Bars::new()
        .with(0, Bar::memory(0x1_0000))
        .expect("BAR0 is free");
    set_bar(&bars, config::BAR0, 0xffff_ffff);
    assert_eq!(bar_dword(&bars, config::BAR0), 0xffff_0000);
    let size = !(bar_dword(&bars, config::BAR0) & 0xffff_fff0) + 1;
    assert_eq!(size, 0x1_0000, "which is how firmware computes the size");

    // A prefetchable window says so in bit 3, and a register that does not
    // exist reads as zero — Rev 2.1's way of saying "stop looking".
    let bars = Bars::new()
        .with(1, Bar::memory(0x100).prefetchable())
        .expect("BAR1 is free");
    set_bar(&bars, config::BAR0 + 4, 0xffff_ffff);
    assert_eq!(bar_dword(&bars, config::BAR0 + 4), 0xffff_ff08);
    assert_eq!(bar_dword(&bars, config::BAR0), 0);
}

#[test]
fn an_io_bar_marks_itself_and_keeps_its_low_two_bits_clear() {
    let bars = Bars::new().with(2, Bar::io(0x20)).expect("BAR2 is free");
    let at = config::BAR0 + 8;
    set_bar(&bars, at, 0xffff_ffff);
    // Bit 0 set marks I/O; bit 1 is reserved and reads zero; the window is 32
    // bytes, so bits 4:2 are don't-care.
    assert_eq!(bar_dword(&bars, at), 0xffff_ffe1);
    set_bar(&bars, at, 0xc0d5);
    assert_eq!(bar_dword(&bars, at), 0xc0c1, "aligned down to 32 bytes");
    assert_eq!(bars.window(2, config::COMMAND_IO), Some((0xc0c0, true)));
    assert_eq!(
        bars.window(2, 0),
        Some((0xc0c0, false)),
        "and it decodes nothing until COMMAND[0] says so"
    );
}

#[test]
fn a_64_bit_bar_is_two_registers_and_one_address() {
    // Rev 2.1 §6.2.5.1, type 10b: this register and the next one are one
    // address, and the next one has no format bits of its own.
    let bars = Bars::new()
        .with(0, Bar::memory(0x10_0000).wide().prefetchable())
        .expect("BAR0 and BAR1 are free");
    set_bar(&bars, config::BAR0, 0xffff_ffff);
    set_bar(&bars, config::BAR0 + 4, 0xffff_ffff);
    assert_eq!(bar_dword(&bars, config::BAR0), 0xfff0_000c);
    assert_eq!(bar_dword(&bars, config::BAR0 + 4), 0xffff_ffff);

    set_bar(&bars, config::BAR0, 0x8010_0000);
    set_bar(&bars, config::BAR0 + 4, 0x0000_0007);
    assert_eq!(
        bars.window(0, config::COMMAND_MEMORY),
        Some((0x7_8010_0000, true)),
        "the upper half is the top 32 bits of the address, not a second window"
    );
    assert!(
        bars.window(1, config::COMMAND_MEMORY).is_none(),
        "and the upper half is not a window in its own right"
    );
}

#[test]
fn the_expansion_rom_register_needs_both_enables() {
    // §6.2.5.2: the address field starts at bit 11, bit 0 is the enable, and
    // "the Memory Space bit in the Command register has precedence over the
    // Expansion ROM Enable bit".
    let bars = Bars::new()
        .with(Bars::ROM, Bar::rom(0x1_0000))
        .expect("the ROM register is free");
    set_bar(&bars, config::EXPANSION_ROM, 0xffff_ffff);
    assert_eq!(
        bar_dword(&bars, config::EXPANSION_ROM),
        0xffff_0001,
        "the size mask, plus the enable bit which is writable"
    );
    set_bar(&bars, config::EXPANSION_ROM, 0xfebf_07fe);
    assert_eq!(
        bar_dword(&bars, config::EXPANSION_ROM),
        0xfebf_0000,
        "bits 10:1 are reserved and never latch"
    );
    assert_eq!(
        bars.window(Bars::ROM, config::COMMAND_MEMORY),
        Some((0xfebf_0000, false)),
        "the memory space bit alone is not enough"
    );
    set_bar(&bars, config::EXPANSION_ROM, 0xfebf_0001);
    assert_eq!(
        bars.window(Bars::ROM, 0),
        Some((0xfebf_0000, false)),
        "and neither is the enable bit alone"
    );
    assert_eq!(
        bars.window(Bars::ROM, config::COMMAND_MEMORY),
        Some((0xfebf_0000, true)),
    );
}

#[test]
fn a_window_moves_when_its_register_does() {
    let space = Arc::new(AddressSpace::new("mem", 32).with_unassigned(UnassignedPolicy::ONES));
    let bars = Bars::new()
        .with(
            0,
            Bar::memory(0x1000).decoding(window_region(0x1000, 0x5a), Perms::RW),
        )
        .expect("BAR0 is free");
    bars.install(&space, 0).expect("nothing is there yet");
    // Out of reset the window decodes nothing at all, wherever it nominally
    // sits: COMMAND[1] is clear.
    assert_eq!(
        space.read(0x8000_0000, Width::U8, MemAttrs::DEFAULT),
        Ok(0xff)
    );

    set_bar(&bars, config::BAR0, 0x8000_0000);
    bars.sync(config::COMMAND_MEMORY, true);
    assert_eq!(
        space.read(0x8000_0000, Width::U8, MemAttrs::DEFAULT),
        Ok(0x5a),
        "enabled, and where the register says"
    );

    set_bar(&bars, config::BAR0, 0x9000_0000);
    bars.sync(config::COMMAND_MEMORY, true);
    assert_eq!(
        space.read(0x8000_0000, Width::U8, MemAttrs::DEFAULT),
        Ok(0xff),
        "it left"
    );
    assert_eq!(
        space.read(0x9000_0000, Width::U8, MemAttrs::DEFAULT),
        Ok(0x5a),
        "and arrived"
    );

    bars.sync(0, true);
    assert_eq!(
        space.read(0x9000_0000, Width::U8, MemAttrs::DEFAULT),
        Ok(0xff),
        "clearing COMMAND[1] takes it out of the decode without moving it"
    );
}

#[test]
fn a_retopology_that_cannot_happen_now_happens_later() {
    // The whole reason the try-lock exists: a configuration write may arrive
    // while something else holds the memory space's topology. That is not
    // swallowed — the flag says so and the next attempt puts it right.
    let space = Arc::new(AddressSpace::new("mem", 32).with_unassigned(UnassignedPolicy::ONES));
    let bars = Bars::new()
        .with(
            0,
            Bar::memory(0x1000).decoding(window_region(0x1000, 0x5a), Perms::RW),
        )
        .expect("BAR0 is free");
    bars.install(&space, 0).expect("nothing is there yet");
    set_bar(&bars, config::BAR0, 0x8000_0000);
    {
        let _held = space.topology();
        assert!(
            !bars.sync(config::COMMAND_MEMORY, false),
            "the try-lock fails while the guard above is alive"
        );
        assert!(bars.is_stale());
    }
    assert!(
        bars.sync(config::COMMAND_MEMORY, false),
        "and now it does not"
    );
    assert!(!bars.is_stale());
    assert_eq!(
        space.read(0x8000_0000, Width::U8, MemAttrs::DEFAULT),
        Ok(0x5a)
    );
}

#[test]
fn a_window_off_the_end_of_the_space_decodes_nothing() {
    // Firmware can write any base it likes. One that does not fit is a card
    // decoding an address the machine cannot drive, so it decodes nothing —
    // rather than the mapping being refused and the register silently
    // disagreeing with the map.
    let space = Arc::new(AddressSpace::new("mem", 20).with_unassigned(UnassignedPolicy::ONES));
    let bars = Bars::new()
        .with(
            0,
            Bar::memory(0x1000).decoding(window_region(0x1000, 0x5a), Perms::RW),
        )
        .expect("BAR0 is free");
    bars.install(&space, 0).expect("nothing is there yet");
    set_bar(&bars, config::BAR0, 0x000f_f000);
    assert!(bars.sync(config::COMMAND_MEMORY, true));
    assert_eq!(
        space.read(0xf_f000, Width::U8, MemAttrs::DEFAULT),
        Ok(0x5a),
        "the last page of a 1 MiB space fits exactly"
    );
    set_bar(&bars, config::BAR0, 0x0010_0000);
    assert!(bars.sync(config::COMMAND_MEMORY, true));
    assert_eq!(
        space.read(0xf_f000, Width::U8, MemAttrs::DEFAULT),
        Ok(0xff),
        "one page further is off the end, and nothing answers anywhere"
    );
}

#[test]
fn a_malformed_declaration_is_refused_by_name() {
    let e = Bars::new()
        .with(0, Bar::memory(0x1800))
        .expect_err("6 KiB is not a power of two")
        .to_string();
    assert!(e.contains("BAR0"), "{e}");
    assert!(e.contains("power of two"), "{e}");

    assert!(
        Bars::new().with(5, Bar::memory(0x1000).wide()).is_err(),
        "a 64-bit register cannot be the last one"
    );
    assert!(
        Bars::new()
            .with(0, Bar::memory(0x1000).wide())
            .expect("BAR0 and BAR1")
            .with(1, Bar::memory(0x1000))
            .is_err(),
        "BAR1 is the upper half of BAR0 and is not free"
    );
    assert!(
        Bars::new().with(Bars::ROM, Bar::memory(0x1000)).is_err(),
        "register 6 is the expansion ROM and holds nothing else"
    );
    assert!(
        Bars::new().with(0, Bar::rom(0x1000)).is_err(),
        "and the expansion ROM is not BAR0"
    );
    assert!(
        Bars::new().with(Bars::ROM, Bar::rom(1024)).is_err(),
        "a ROM window is at least 2 KiB — §6.2.5.2's address field starts at bit 11"
    );
}

#[test]
fn an_io_bar_that_wants_a_region_is_refused_with_the_reason() {
    // Not an oversight: a configuration cycle travels through the I/O space,
    // so the try-lock that saves every other case cannot help. Better to say
    // so at bind than to map a window that never moves again.
    let space = Arc::new(AddressSpace::new("port", 16).with_unassigned(UnassignedPolicy::ONES));
    let bars = Bars::new()
        .with(
            0,
            Bar::io(0x10).decoding(window_region(0x10, 0x5a), Perms::RW),
        )
        .expect("BAR0 is free");
    let e = bars
        .install(&space, 0)
        .expect_err("an I/O BAR cannot carry a region")
        .to_string();
    assert!(e.contains("I/O space"), "{e}");
}

#[test]
fn the_latches_round_trip_and_a_reset_clears_them() {
    let bars = Bars::new()
        .with(0, Bar::memory(0x1000))
        .expect("BAR0")
        .with(Bars::ROM, Bar::rom(0x1_0000))
        .expect("the ROM register");
    set_bar(&bars, config::BAR0, 0x8000_0000);
    set_bar(&bars, config::EXPANSION_ROM, 0xfebf_0001);
    let saved = bars.latches();

    let restored = Bars::new()
        .with(0, Bar::memory(0x1000))
        .expect("BAR0")
        .with(Bars::ROM, Bar::rom(0x1_0000))
        .expect("the ROM register");
    restored.set_latches(&saved);
    assert_eq!(restored.latches(), saved);
    assert_eq!(bar_dword(&restored, config::EXPANSION_ROM), 0xfebf_0001);

    // A snapshot cannot install bits the hardware could never hold, for the
    // same reason `ConfigSpace::restore` cannot change a vendor ID.
    restored.set_latches(&[0xffff_ffff; Bars::COUNT as usize]);
    assert_eq!(bar_dword(&restored, config::BAR0), 0xffff_f000);

    restored.reset();
    assert_eq!(bar_dword(&restored, config::BAR0), 0);
    assert_eq!(bar_dword(&restored, config::EXPANSION_ROM), 0);
}

// ---------------------------------------------------------------------------
// INTx#
// ---------------------------------------------------------------------------

/// A south bridge, reduced to the one thing this file can check: it remembers
/// what it was told, and it never asks the fabric anything.
#[derive(Debug)]
struct Router {
    levels: Mutex<[Level; INTX_LINES as usize]>,
    changes: Mutex<Vec<(u8, Level)>>,
}

impl Router {
    fn new() -> Arc<Router> {
        Arc::new(Router {
            levels: Mutex::with_rank(LockRank::DEVICE, [Level::Low; INTX_LINES as usize]),
            changes: Mutex::with_rank(LockRank::LEAF, Vec::new()),
        })
    }

    fn level(&self, line: u8) -> Level {
        self.levels.lock()[line as usize]
    }
}

impl IntxSink for Router {
    fn intx_changed(&self, line: u8, level: Level) {
        self.levels.lock()[line as usize] = level;
        self.changes.lock().push((line, level));
    }
}

/// A function at `device` driving `pin`, plugged into `bus`.
fn pinned(bus: &Arc<PciBus>, device: u8, pin: IntxPin) -> Intx {
    let at = Bdf::new(0, device, 0).expect("a legal device number");
    let intx = Intx::new(pin);
    intx.plug(bus, at);
    intx
}

#[test]
fn the_swizzle_rotates_by_device_number() {
    // PCI-to-PCI Bridge 1.1 Table 9-1, written out: four devices each driving
    // nothing but `INTA#` land on four different nets, and the rotation is by
    // device number.
    for device in 0..=MAX_DEVICE {
        let at = Bdf::new(0, device, 0).expect("a legal device number");
        assert_eq!(swizzle(at, IntxPin::A), Some(device % 4));
        assert_eq!(swizzle(at, IntxPin::B), Some((device + 1) % 4));
        assert_eq!(swizzle(at, IntxPin::C), Some((device + 2) % 4));
        assert_eq!(swizzle(at, IntxPin::D), Some((device + 3) % 4));
        // A function that has no pin reaches no net, whatever its number.
        assert_eq!(swizzle(at, IntxPin::NONE), None);
        assert_eq!(swizzle(at, IntxPin(0xff)), None);
    }
    // The function number is not part of it: two functions of one device with
    // the same pin are on the same net, which is what makes them share.
    let a = Bdf::new(0, 4, 0).expect("f0");
    let b = Bdf::new(0, 4, 7).expect("f7");
    assert_eq!(swizzle(a, IntxPin::A), swizzle(b, IntxPin::A));
}

#[test]
fn a_shared_net_stays_asserted_until_the_last_driver_lets_go() {
    // Rev 2.1 §2.2.6: the pins are open drain. Two functions on one net is the
    // whole difficulty, and it is why the fabric keeps the set of drivers
    // rather than a level.
    let bus = Arc::new(PciBus::new());
    let router = Router::new();
    bus.set_intx_sink(Arc::downgrade(&router) as Weak<dyn IntxSink>);

    // Device 4 pin A and device 8 pin A both swizzle onto net 0.
    let first = pinned(&bus, 4, IntxPin::A);
    let second = pinned(&bus, 8, IntxPin::A);
    assert_eq!(swizzle(Bdf::new(0, 8, 0).unwrap(), IntxPin::A), Some(0));

    first.set(Level::High);
    assert_eq!(router.level(0), Level::High);
    second.set(Level::High);
    assert_eq!(router.level(0), Level::High);
    assert_eq!(
        bus.intx_drivers(0),
        vec![
            Bdf::new(0, 4, 0).expect("f0"),
            Bdf::new(0, 8, 0).expect("f0")
        ],
        "and the fabric can say which two"
    );

    first.set(Level::Low);
    assert_eq!(
        router.level(0),
        Level::High,
        "one function releasing does not release the net"
    );
    second.set(Level::Low);
    assert_eq!(router.level(0), Level::Low);

    // And the sink was told once per *change*, not once per call: a level that
    // did not move is not an event, or a shared line would look like an edge.
    // The four that come first are the realize sweep, which announces every
    // net whether it moved or not.
    assert_eq!(
        *router.changes.lock(),
        vec![
            (0, Level::Low),
            (1, Level::Low),
            (2, Level::Low),
            (3, Level::Low),
            (0, Level::High),
            (0, Level::Low)
        ]
    );
}

#[test]
fn a_sink_installed_late_is_told_where_the_nets_already_are() {
    // `ROADMAP.md` §4.3: realize sweeps the graph, because an undriven wire
    // sits low and a machine that skipped the sweep comes up with interrupt
    // lines that are wrong on some paths only.
    let bus = Arc::new(PciBus::new());
    let intx = pinned(&bus, 1, IntxPin::A);
    intx.set(Level::High);

    let router = Router::new();
    bus.set_intx_sink(Arc::downgrade(&router) as Weak<dyn IntxSink>);
    assert_eq!(router.level(1), Level::High, "device 1 pin A is net 1");
    assert_eq!(router.level(0), Level::Low);
    assert_eq!(router.level(2), Level::Low);
    assert_eq!(router.level(3), Level::Low);
}

#[test]
fn a_pin_plugged_in_while_asserted_brings_its_level_with_it() {
    // A device may raise its interrupt before it is realized — a snapshot load
    // does exactly that — so plugging in is an announcement, not a reset.
    let bus = Arc::new(PciBus::new());
    let router = Router::new();
    bus.set_intx_sink(Arc::downgrade(&router) as Weak<dyn IntxSink>);

    let intx = Intx::new(IntxPin::B);
    intx.set(Level::High);
    assert_eq!(router.level(0), Level::Low, "it is not on the bus yet");
    intx.plug(&bus, Bdf::new(0, 3, 0).expect("device 3"));
    assert_eq!(router.level(0), Level::High, "device 3 pin B is net 0");

    // And unplugging releases the net rather than stranding it low for ever.
    intx.unplug();
    assert_eq!(router.level(0), Level::Low);
}

#[test]
fn a_pin_drives_the_board_wire_and_the_fabric_together() {
    // The same pin at two points on the board: the card edge, which a machine
    // with no router wires straight to an interrupt controller, and the net,
    // which a machine with one collects.
    use crate::core::wire::{FanIn, Resolve, Wire, WireId, WireIdAllocator, WireSink};

    #[derive(Debug)]
    struct Watch {
        inputs: FanIn,
        level: Mutex<Level>,
    }

    impl WireSink for Watch {
        fn set_level(&self, src: WireId, _line: u32, level: Level) {
            self.inputs.set(src, level);
            *self.level.lock() = self.inputs.resolve(Resolve::Or);
        }
    }

    let bus = Arc::new(PciBus::new());
    let router = Router::new();
    bus.set_intx_sink(Arc::downgrade(&router) as Weak<dyn IntxSink>);
    let intx = pinned(&bus, 0, IntxPin::A);

    let ids = WireIdAllocator::new();
    let id = ids.alloc();
    let watch = Arc::new(Watch {
        inputs: FanIn::new(&[id]),
        level: Mutex::with_rank(LockRank::LEAF, Level::Low),
    });
    let wire = Arc::new(
        Wire::builder()
            .source(id)
            .sink(Arc::clone(&watch) as Arc<dyn WireSink>, 0)
            .build(),
    );
    intx.connect(WireSource::new(wire, id));

    intx.set(Level::High);
    assert_eq!(*watch.level.lock(), Level::High, "the wire followed");
    assert_eq!(router.level(0), Level::High, "and so did the net");
    intx.set(Level::Low);
    assert_eq!(*watch.level.lock(), Level::Low);
    assert_eq!(router.level(0), Level::Low);
}

#[test]
fn detaching_a_function_releases_the_net_it_was_holding() {
    let bus = Arc::new(PciBus::new());
    let router = Router::new();
    bus.set_intx_sink(Arc::downgrade(&router) as Weak<dyn IntxSink>);
    let at = Bdf::new(0, 2, 0).expect("device 2");
    bus.attach(at, Recorder::new() as Arc<dyn PciFunction>)
        .expect("nothing is there yet");
    let intx = Intx::new(IntxPin::A);
    intx.plug(&bus, at);
    intx.set(Level::High);
    assert_eq!(router.level(2), Level::High);

    assert!(bus.detach(at));
    assert_eq!(
        router.level(2),
        Level::Low,
        "a card that is gone is not still pulling the line down"
    );
}

#[test]
fn ports_whose_fabric_is_gone_master_abort() {
    // `ConfigPorts` holds its fabric weakly so that a host bridge can keep the
    // ports in its own register file without leaking the machine. The price is
    // this case, and it has to be the answer a bus gives for nothing being
    // there.
    let ports = {
        let bus = Arc::new(PciBus::new());
        bus.attach(Bdf::default(), Recorder::new() as Arc<dyn PciFunction>)
            .expect("nothing is there yet");
        ConfigPorts::new(bus)
    };
    assert!(ports.bus().is_none());
    ports.set_address(CONFIG_ENABLE);
    let mut dst = [0u8; 4];
    ports
        .read(4, &mut dst, MemAttrs::DEFAULT)
        .expect("a decoded port");
    assert_eq!(dst, [0xff; 4], "a master abort reads as ones");
    ports
        .write(4, &[0x12, 0x34, 0x56, 0x78], MemAttrs::DEFAULT)
        .expect("an unclaimed write is not an error");
}
