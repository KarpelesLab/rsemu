//! The host bridge, checked against §3.2.18's four attribute encodings and the
//! shadow recipe the datasheet spells out.

use super::*;

use alloc::vec;

use crate::core::device::{Deferred, ResetKind};
use crate::core::error::BusError;
use crate::core::hosts::HostObjects;
use crate::core::space::{Region as CoreRegion, RequesterId, RomStore, RomWrite, UnassignedPolicy};
use crate::core::state::{MachineShape, Migrations, StateReader, StateWriter};
use crate::core::value::Width;

/// The byte a "ROM" under the shadow window answers with, so a test can tell
/// which of the two chips answered without looking at anything but the value.
const ROM_BYTE: u8 = 0xa5;

/// A bridge with a ROM under its whole window and its config ports mapped.
///
/// The board's shape in miniature: `mem` carries a 256 KiB ROM at 0xc0000 and
/// `port` carries the 0xcf8 pair, and nothing else — so every read below says
/// exactly which chip claimed the cycle.
struct Rig {
    mem: Arc<AddressSpace>,
    port: Arc<AddressSpace>,
    pmc: Pmc,
}

impl Rig {
    fn new() -> Rig {
        let bus = Arc::new(PciBus::new());
        let pmc = Pmc::with_bus(bus, Bdf::default(), 0x02).expect("a legal bridge");

        let mem = Arc::new(AddressSpace::new("mem", 32).with_unassigned(UnassignedPolicy::ONES));
        let rom = Arc::new(RomStore::new(vec![ROM_BYTE; SHADOW_LEN as usize]));
        mem.topology()
            .map(
                CoreRegion::rom("firmware", rom, RomWrite::Ignore),
                SHADOW_BASE,
            )
            .expect("nothing is there yet");

        let port = Arc::new(AddressSpace::new("port", 16).with_unassigned(UnassignedPolicy::ONES));
        port.topology()
            .map(pmc.region("").expect("the config ports"), 0xcf8)
            .expect("0xcf8 is free");

        let mut deferred = Deferred::new();
        let hosts = HostObjects::new();
        let mut ctx = RealizeCtx::new("pmc", RequesterId::ANONYMOUS, &mut deferred, &hosts);
        pmc.realize(&mut ctx).expect("it announces onto the fabric");
        deferred.drain();

        pmc.attach_space(&mem).expect("the windows go in");
        pmc.reset(ResetKind::Cold);

        Rig { mem, port, pmc }
    }

    /// Point `CONFADD` at one of the bridge's own registers.
    fn select(&self, register: u16) {
        let value = 0x8000_0000u32 | u32::from(register & 0xfc);
        self.port
            .write(0xcf8, Width::U32, u64::from(value), MemAttrs::DEFAULT)
            .expect("a Dword write to CONFADD");
    }

    /// Write one configuration byte, as firmware does.
    fn config_write_u8(&self, register: u16, value: u8) {
        self.select(register);
        self.port
            .write(
                0xcfc + u64::from(register & 3),
                Width::U8,
                u64::from(value),
                MemAttrs::DEFAULT,
            )
            .expect("a byte write to CONFDATA");
    }

    /// Read one configuration byte back.
    fn config_read_u8(&self, register: u16) -> u8 {
        self.select(register);
        self.port
            .read(
                0xcfc + u64::from(register & 3),
                Width::U8,
                MemAttrs::DEFAULT,
            )
            .expect("a byte read of CONFDATA") as u8
    }

    fn peek(&self, addr: u64) -> u8 {
        self.mem
            .read(addr, Width::U8, MemAttrs::DEFAULT)
            .expect("mapped") as u8
    }

    fn poke(&self, addr: u64, value: u8) -> core::result::Result<(), BusError> {
        self.mem
            .write(addr, Width::U8, u64::from(value), MemAttrs::DEFAULT)
    }

    /// One byte read from the base of each of the thirteen windows.
    fn every_window(&self) -> [u8; N] {
        let mut out = [0u8; N];
        for (slot, w) in out.iter_mut().zip(&WINDOWS) {
            *slot = self.peek(w.base);
        }
        out
    }
}

#[test]
fn the_header_is_the_one_the_datasheet_states() {
    let rig = Rig::new();
    // §3.2.2, §3.2.3: 8086h and 1237h. This is the register pair firmware reads
    // first and the one that decides whether it believes there is a bridge.
    rig.select(config::VENDOR_ID);
    assert_eq!(
        rig.port.read(0xcfc, Width::U32, MemAttrs::DEFAULT),
        Ok(0x1237_8086)
    );
    // §3.2.7: class code 060000h — bridge, host bridge, no programming
    // interface. Firmware looks for exactly this to decide it has found a host
    // bridge rather than some other device at 00:00.0.
    rig.select(config::CLASS_CODE);
    let dword = rig
        .port
        .read(0xcfc, Width::U32, MemAttrs::DEFAULT)
        .expect("a Dword read");
    assert_eq!(
        dword >> 8,
        0x0006_0000,
        "class code, above the revision byte"
    );
    assert_eq!(dword & 0xff, 0x02, "the revision this instance was given");
    // §3.2.9: header type 00h, and not multi-function.
    assert_eq!(rig.config_read_u8(config::HEADER_TYPE), 0x00);
    // §3.2.5: PCISTS default 0280h.
    rig.select(config::STATUS);
    assert_eq!(
        rig.port.read(0xcfe, Width::U16, MemAttrs::DEFAULT),
        Ok(0x0280)
    );
}

#[test]
fn pam_comes_out_of_reset_at_zero_and_the_rom_is_what_is_decoded() {
    let rig = Rig::new();
    for i in 0..PAM_COUNT {
        assert_eq!(rig.pmc.pam(i), Some(0), "PAM{i} defaults to 00h (§3.2.18)");
    }
    assert_eq!(rig.every_window(), [ROM_BYTE; N]);
}

#[test]
fn the_four_attribute_encodings_do_what_table_2_says() {
    let rig = Rig::new();
    // PAM0[7:4] governs 0F0000-0FFFFF, the system BIOS area.
    let at = 0xf_0000u64;

    // 00: disabled. Reads and writes both go to PCI — here, the ROM, which
    // answers reads and swallows writes.
    assert_eq!(rig.peek(at), ROM_BYTE);
    assert_eq!(rig.poke(at, 0x11), Ok(()), "a write to ROM is swallowed");
    assert_eq!(rig.peek(at), ROM_BYTE);

    // WE=1, RE=0: write only. This is the state the datasheet's shadow recipe
    // copies in. The read still comes from the ROM and the write lands in
    // DRAM, which is the whole trick.
    rig.config_write_u8(PAM0, WE << 4);
    assert_eq!(rig.peek(at), ROM_BYTE, "reads are still forwarded to PCI");
    rig.poke(at, 0x11).expect("the write is claimed");
    assert_eq!(rig.peek(at), ROM_BYTE, "and did not become visible yet");
    assert_eq!(
        rig.pmc.dram().read_u8(at - SHADOW_BASE),
        Ok(0x11),
        "it went to main memory"
    );

    // RE=1, WE=0: read only. The recipe's second half — reads now come from
    // DRAM and writes are forwarded to PCI, which write-protects the copy.
    rig.config_write_u8(PAM0, RE << 4);
    assert_eq!(rig.peek(at), 0x11, "the shadowed byte");
    assert_eq!(rig.poke(at, 0x22), Ok(()), "the write goes to the ROM");
    assert_eq!(rig.peek(at), 0x11, "and the shadow is unchanged");

    // 11: read/write, ordinary memory.
    rig.config_write_u8(PAM0, (RE | WE) << 4);
    rig.poke(at, 0x33).expect("claimed");
    assert_eq!(rig.peek(at), 0x33);

    // And back to 00: the ROM is decoded again and the DRAM keeps what it
    // holds — it is main memory, not a cache.
    rig.config_write_u8(PAM0, 0);
    assert_eq!(rig.peek(at), ROM_BYTE);
    assert_eq!(rig.pmc.dram().read_u8(at - SHADOW_BASE), Ok(0x33));
}

#[test]
fn every_window_is_governed_by_the_nibble_table_3_names() {
    // The test that catches a transposed nibble or an off-by-one window, which
    // is this table's failure mode.
    let rig = Rig::new();
    for (i, w) in WINDOWS.iter().enumerate() {
        rig.config_write_u8(w.reg, (RE | WE) << w.shift);
        rig.poke(w.base, i as u8).expect("claimed");
        rig.poke(w.base + w.len - 1, 0xf0 | i as u8)
            .expect("claimed at the far end too");
        let seen = rig.every_window();
        for (j, byte) in seen.iter().enumerate() {
            if j == i {
                assert_eq!(*byte, i as u8, "window {i} is the one that answered");
            } else {
                assert_eq!(*byte, ROM_BYTE, "window {j} moved when window {i} did");
            }
        }
        rig.config_write_u8(w.reg, 0);
    }
}

#[test]
fn pam0_low_nibble_is_reserved_and_governs_nothing() {
    // §3.2.18 Table 3's first row. Writing it must not open a window.
    let rig = Rig::new();
    rig.config_write_u8(PAM0, 0x0f);
    assert_eq!(rig.every_window(), [ROM_BYTE; N]);
}

#[test]
fn the_reserved_bits_of_a_nibble_change_nothing() {
    // Bits [7,6,3,2] are reserved (Table 2). They latch — firmware that reads
    // back what it wrote is entitled to see it — but they open no window.
    let rig = Rig::new();
    rig.config_write_u8(PAM0 + 1, 0xcc);
    assert_eq!(rig.config_read_u8(PAM0 + 1), 0xcc, "it latches");
    assert_eq!(rig.peek(WINDOWS[0].base), ROM_BYTE);
    assert_eq!(rig.peek(WINDOWS[1].base), ROM_BYTE);
}

#[test]
fn the_datasheet_shadow_recipe_works_end_to_end() {
    // §3.2.18: set write only, read the address (which reaches the ROM), write
    // the same address (which reaches DRAM), then set read only. This is what a
    // real firmware does, and it is the whole reason this device exists.
    let rig = Rig::new();
    rig.config_write_u8(PAM0, WE << 4);
    for off in (0xf_0000u64..0x10_0000).step_by(0x1000) {
        let byte = rig.peek(off);
        rig.poke(off, byte).expect("claimed");
    }
    rig.config_write_u8(PAM0, RE << 4);
    for off in (0xf_0000u64..0x10_0000).step_by(0x1000) {
        assert_eq!(rig.peek(off), ROM_BYTE, "the copy reads back at {off:#x}");
    }
    // And it is now write-protected, exactly as the datasheet says.
    rig.poke(0xf_0000, 0x00).expect("forwarded to PCI");
    assert_eq!(rig.peek(0xf_0000), ROM_BYTE);
}

#[test]
fn a_reset_puts_the_rom_back() {
    let rig = Rig::new();
    rig.config_write_u8(PAM0, (RE | WE) << 4);
    rig.poke(0xf_0000, 0x5a).expect("claimed");
    assert_eq!(rig.peek(0xf_0000), 0x5a);

    // A warm reset is PCIRST#: the registers go back to their defaults and the
    // ROM is decoded again, which is what firmware has to find at its reset
    // vector. The DRAM keeps its contents.
    rig.pmc.reset(ResetKind::Warm);
    assert_eq!(rig.peek(0xf_0000), ROM_BYTE);
    assert_eq!(rig.pmc.pam(0), Some(0));
    assert_eq!(rig.pmc.dram().read_u8(0x3_0000), Ok(0x5a));

    // A cold reset is power: the DRAM goes too.
    rig.pmc.reset(ResetKind::Cold);
    assert_eq!(rig.pmc.dram().read_u8(0x3_0000), Ok(0x00));
}

#[test]
fn a_debug_access_cannot_move_a_pam_window() {
    let rig = Rig::new();
    assert!(
        rig.port
            .write(0xcf8, Width::U32, 0x8000_0059, MemAttrs::DEBUG)
            .is_err()
    );
    rig.select(PAM0);
    assert!(
        rig.port
            .write(0xcfd, Width::U8, 0x33, MemAttrs::DEBUG)
            .is_err(),
        "a debugger may not write PAM"
    );
    assert_eq!(rig.pmc.pam(0), Some(0), "and did not");
    // A debug *read* is fine and changes nothing.
    assert!(rig.port.read(0xcfd, Width::U8, MemAttrs::DEBUG).is_ok());
    assert_eq!(rig.pmc.pam(0), Some(0));
}

#[test]
fn a_read_only_register_is_read_only_however_it_is_written() {
    let rig = Rig::new();
    rig.config_write_u8(config::VENDOR_ID, 0x00);
    rig.select(config::VENDOR_ID);
    assert_eq!(
        rig.port.read(0xcfc, Width::U16, MemAttrs::DEFAULT),
        Ok(0x8086),
        "§3.2.2: writes to VID have no effect"
    );
}

#[test]
fn a_stale_retopology_is_re_applied_rather_than_lost() {
    // The one path the machine never takes: a PAM write while something else
    // holds the memory space's topology guard. The write latches, the mapping
    // does not move, the device knows it is stale, and the next configuration
    // access puts it right.
    let rig = Rig::new();
    {
        let _held = rig.mem.topology();
        rig.config_write_u8(PAM0, (RE | WE) << 4);
        assert_eq!(rig.pmc.pam(0), Some(0x30), "the register still latched");
        assert!(*rig.pmc.regs.stale.lock(), "and it noticed");
    }
    // Any later configuration access re-applies. A read is enough.
    let _ = rig.config_read_u8(config::VENDOR_ID);
    assert!(!*rig.pmc.regs.stale.lock(), "and put it right");
    assert_eq!(rig.peek(0xf_0000), 0x00, "the DRAM is decoded now");
}

/// One snapshot chunk holding this bridge's state.
fn image(pmc: &Pmc) -> alloc::vec::Vec<u8> {
    let mut shape = MachineShape::new();
    shape.add_device("pmc", CLASS_NAME).expect("unique path");
    let mut w = StateWriter::new(shape);
    {
        let mut chunk = w
            .chunk("pmc", CLASS_NAME, STATE_VERSION)
            .expect("one chunk");
        pmc.save(&mut chunk).expect("saves");
    }
    w.to_vec().expect("encodes")
}

#[test]
fn the_state_round_trips_byte_for_byte() {
    let a = Rig::new();
    a.config_write_u8(PAM0, (RE | WE) << 4);
    a.config_write_u8(PAM0 + 3, RE | WE);
    a.config_write_u8(config::LATENCY_TIMER, 0x40);
    a.poke(0xf_1234, 0x5a).expect("claimed");
    a.select(0x40);
    let saved = image(&a.pmc);

    let b = Rig::new();
    let reader = StateReader::new(&saved).expect("it parses");
    let chunk = reader
        .load("pmc", CLASS_NAME, STATE_VERSION, &Migrations::new())
        .expect("the chunk is there");
    b.pmc.load(&mut chunk.reader()).expect("it loads");

    // Every guest-visible thing agrees: the registers, the address latch, the
    // DRAM, and — the one that is not saved and has to be rebuilt — the memory
    // map the PAM registers imply.
    for i in 0..PAM_COUNT {
        assert_eq!(a.pmc.pam(i), b.pmc.pam(i), "PAM{i}");
    }
    assert_eq!(
        b.peek(0xf_1234),
        0x5a,
        "the shadow came back, and is decoded"
    );
    assert_eq!(b.peek(WINDOWS[4].base), 0x00, "PAM3[3:0]'s window too");
    // Before any further configuration access, because `CONFADD` is part of
    // the saved state and selecting a register would move it.
    assert_eq!(
        image(&b.pmc),
        saved,
        "a reload saves byte-identically, which is what a state hash is"
    );
    assert_eq!(
        b.config_read_u8(config::LATENCY_TIMER),
        0x40,
        "and the writable header bytes came back too"
    );
}

#[test]
fn a_snapshot_of_the_wrong_size_is_refused_by_name() {
    let rig = Rig::new();
    let mut shape = MachineShape::new();
    shape.add_device("pmc", CLASS_NAME).expect("unique path");
    let mut w = StateWriter::new(shape);
    {
        let mut chunk = w
            .chunk("pmc", CLASS_NAME, STATE_VERSION)
            .expect("one chunk");
        chunk.write_bytes(&[0u8; 4]).expect("config");
        chunk.write_u32(0).expect("the latch");
        chunk.write_bytes(&[0u8; 16]).expect("far too little DRAM");
    }
    let bytes = w.to_vec().expect("encodes");
    let reader = StateReader::new(&bytes).expect("it parses");
    let chunk = reader
        .load("pmc", CLASS_NAME, STATE_VERSION, &Migrations::new())
        .expect("the chunk is there");
    let e = rig
        .pmc
        .load(&mut chunk.reader())
        .expect_err("16 bytes of shadow")
        .to_string();
    assert!(e.contains("shadow DRAM"), "{e}");
}

#[test]
fn the_config_ports_are_the_only_region_and_it_answers_to_two_names() {
    let pmc = Pmc::with_bus(Arc::new(PciBus::new()), Bdf::default(), 0).expect("a bridge");
    assert!(pmc.region("").is_some());
    assert!(pmc.region("config").is_some());
    assert!(pmc.region("pam").is_none());
}

#[test]
fn two_bridges_cannot_claim_one_address() {
    let bus = Arc::new(PciBus::new());
    let a = Pmc::with_bus(Arc::clone(&bus), Bdf::default(), 0).expect("a bridge");
    let b = Pmc::with_bus(Arc::clone(&bus), Bdf::default(), 0).expect("another");
    let hosts = HostObjects::new();
    let mut deferred = Deferred::new();
    {
        let mut ctx = RealizeCtx::new("a", RequesterId::ANONYMOUS, &mut deferred, &hosts);
        a.realize(&mut ctx).expect("the first one");
    }
    let mut ctx = RealizeCtx::new("b", RequesterId::ANONYMOUS, &mut deferred, &hosts);
    let e = b.realize(&mut ctx).expect_err("the second one").to_string();
    assert!(e.contains("cannot share"), "{e}");
}

#[test]
fn the_class_constructs_from_properties() {
    let mut props = Props::new();
    props.insert("device", crate::core::props::Value::Uint(0));
    let dev = (CLASS.construct)(&props).expect("a bridge");
    assert_eq!(dev.class().name, CLASS_NAME);

    let mut bad = Props::new();
    bad.insert("device", crate::core::props::Value::Uint(32));
    let e = (CLASS.construct)(&bad).expect_err("five bits").to_string();
    assert!(e.contains("device"), "{e}");
}

/// A stand-in for the south bridge's four bytes at `0xcf8`: the reset control
/// register at offset 1, and nothing at the other three.
#[derive(Debug, Default)]
struct Cf9(Mutex<u8>);

impl crate::core::space::MemOps for Cf9 {
    fn read(&self, offset: u64, dst: &mut [u8], _: MemAttrs) -> crate::core::space::MemResult {
        for (i, slot) in dst.iter_mut().enumerate() {
            *slot = if offset + i as u64 == 1 {
                *self.0.lock()
            } else {
                0xff
            };
        }
        Ok(())
    }

    fn write(&self, offset: u64, src: &[u8], _: MemAttrs) -> crate::core::space::MemResult {
        for (i, byte) in src.iter().enumerate() {
            if offset + i as u64 == 1 {
                *self.0.lock() = *byte;
            }
        }
        Ok(())
    }

    fn constraints(&self) -> crate::core::space::AccessConstraints {
        crate::core::space::AccessConstraints::IO
    }
}

#[test]
fn a_narrow_access_inside_confadd_passes_through_and_a_dword_does_not() {
    // The reason this seam exists: 0xcf9 is inside CONFADD's four bytes and
    // belongs to a different chip, and an address space decodes by address
    // alone. Mapping both would split every Dword write to 0xcf8 into three
    // pieces around 0xcf9 — measured, and the reason the board maps only one.
    let rig = Rig::new();
    let cf9 = Arc::new(Cf9::default());
    rig.pmc
        .regs
        .ports
        .set_passthrough(Arc::clone(&cf9) as Arc<dyn crate::core::space::MemOps>);

    // A byte write at 0xcf9 reaches the other chip and leaves the latch alone.
    rig.select(PAM0);
    let latched = rig.pmc.regs.ports.address();
    rig.port
        .write(0xcf9, Width::U8, 0x06, MemAttrs::DEFAULT)
        .expect("an ordinary byte port");
    assert_eq!(*cf9.0.lock(), 0x06, "the south bridge saw it");
    assert_eq!(
        rig.pmc.regs.ports.address(),
        latched,
        "and CONFADD did not move"
    );
    assert_eq!(rig.port.read(0xcf9, Width::U8, MemAttrs::DEFAULT), Ok(0x06));
    assert_eq!(
        rig.port.read(0xcfa, Width::U8, MemAttrs::DEFAULT),
        Ok(0xff),
        "the other three bytes are I/O space with nothing behind them"
    );

    // And a Dword access at 0xcf8 is the bridge's own, whole and undivided.
    rig.port
        .write(0xcf8, Width::U32, 0x8000_5900, MemAttrs::DEFAULT)
        .expect("a Dword write to CONFADD");
    assert_eq!(rig.pmc.regs.ports.address(), 0x8000_5900);
    assert_eq!(
        rig.port.read(0xcf8, Width::U32, MemAttrs::DEFAULT),
        Ok(0x8000_5900)
    );
    assert_eq!(*cf9.0.lock(), 0x06, "the pass-through saw none of it");
}

#[test]
fn the_board_wires_the_pass_through_to_the_reset_control_register() {
    // The end-to-end version of the test above, through `pc.sysctl`'s own
    // export rather than a stand-in: a byte write of 0x02 then 0x06 at 0xcf9
    // pulses the reset line, which is the third way to reboot a PC.
    use crate::core::sync::{AtomicU32, Ordering};
    use crate::core::wire::{Level, Wire, WireId, WireIdAllocator, WireSink, WireSource};

    /// Counts rising edges on the reset net.
    #[derive(Debug, Default)]
    struct Probe(AtomicU32);

    impl WireSink for Probe {
        fn set_level(&self, _src: WireId, _line: u32, level: Level) {
            if level.is_high() {
                self.0.fetch_add(1, Ordering::Relaxed);
            }
        }
    }

    let sysctl = super::super::sysctl::SysCtl::default_device();
    let handle = sysctl
        .export(ExportId::PORT_PASSTHROUGH)
        .expect("sysctl publishes its 0xcf8 window");
    let ops = handle
        .opaque()
        .and_then(|h| {
            Arc::clone(h)
                .downcast::<super::super::PortPassthrough>()
                .ok()
        })
        .expect("as a port pass-through");

    let rig = Rig::new();
    rig.pmc.regs.ports.set_passthrough(Arc::clone(ops.ops()));

    let ids = WireIdAllocator::new();
    let id = ids.alloc();
    let probe = Arc::new(Probe::default());
    let wire = Wire::builder()
        .source(id)
        .sink(Arc::clone(&probe) as Arc<dyn WireSink>, 0)
        .build_shared();
    sysctl
        .connect("reset", WireSource::new(wire, id))
        .expect("the system control ports drive reset");

    rig.port
        .write(0xcf9, Width::U8, 0x02, MemAttrs::DEFAULT)
        .expect("SYS_RST, no trigger yet");
    assert_eq!(
        probe.0.load(Ordering::Relaxed),
        0,
        "bit 2 is the trigger and it is clear"
    );
    rig.port
        .write(0xcf9, Width::U8, 0x06, MemAttrs::DEFAULT)
        .expect("RST_CPU's rising edge");
    assert_eq!(
        probe.0.load(Ordering::Relaxed),
        1,
        "the machine was reset through the bridge's pass-through"
    );
}
