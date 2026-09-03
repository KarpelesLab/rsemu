//! Unit tests for the xHCI PCI function.
//!
//! These are about **the transport**, not the controller: `xhci/tests.rs`
//! already drives rings and contexts through the register block, and repeating
//! that here would test the engine twice and the attachment once. What is new
//! when an xHCI becomes a PCI function is the header a driver enumerates, the
//! base address register it sizes, the two Command bits that gate mastering and
//! the pin, and the shared open-drain net the pin lands on — so those are what
//! this file asserts, each against the specification section that fixes it.
//!
//! Everything below reaches the function the way a guest does: through
//! [`PciBus::config_read`] and [`PciBus::config_write`], never through a Rust
//! handle to the function's own registers.

use super::*;

use alloc::vec::Vec;

use crate::bus::pci::{INTX_LINES, IntxSink, swizzle};
use crate::core::space::{RamStore, Region, RegionRef, RequesterId};
use crate::core::state::{MachineShape, Migrations, StateReader, StateWriter};
use crate::core::sync::LockRank;
use crate::core::value::Width;
use crate::dev::usb::xhci::offset;

// ---------------------------------------------------------------------------
// the map both sides agree on
// ---------------------------------------------------------------------------

/// Where "guest" RAM starts. Not zero, so a null pointer in a register is a bus
/// fault the controller has to survive rather than a plausible read.
const RAM: u64 = 0x1_0000;
/// How much of it there is.
const RAM_SIZE: u64 = 0x8000;
/// Where this test's "firmware" puts the register window. Above the RAM, and
/// aligned to the window's own size, which is what §6.2.5.1 requires of a base.
const BAR_BASE: u64 = 0x10_0000;

/// The event ring segment table, and the ring it names.
const ERST: u64 = RAM;
const EVT_RING: u64 = RAM + 0x1000;
/// Sixteen TRBs, the smallest segment §6.5 allows.
const EVT_TRBS: u32 = 16;
/// The command ring.
const CMD_RING: u64 = RAM + 0x1400;

/// Register offsets inside the window, derived rather than repeated.
const USBCMD: u64 = offset::OPERATIONAL;
const CRCR: u64 = offset::OPERATIONAL + 0x18;
const CONFIG: u64 = offset::OPERATIONAL + 0x38;
const IR0: u64 = offset::RUNTIME + offset::INTERRUPTER0;
const IMAN: u64 = IR0;
const ERSTSZ: u64 = IR0 + 0x08;
const ERSTBA: u64 = IR0 + 0x10;
const ERDP: u64 = IR0 + 0x18;
const DB0: u64 = offset::DOORBELL;

/// Type 00h header offsets a driver names (Rev 2.1 §6.1).
const CFG_VENDOR: u16 = 0x00;
const CFG_COMMAND: u16 = 0x04;
const CFG_STATUS: u16 = 0x06;
const CFG_CLASS: u16 = 0x08;
const CFG_HEADER: u16 = 0x0e;
const CFG_BAR0: u16 = 0x10;
const CFG_BAR1: u16 = 0x14;
const CFG_BAR2: u16 = 0x18;
const CFG_INT_PIN: u16 = 0x3d;

/// `COMMAND[1]` memory space, `COMMAND[2]` bus master.
const CMD_MEM: u32 = 0x0002;
const CMD_BM: u32 = 0x0004;
/// `COMMAND[10]`, Interrupt Disable.
const CMD_INTX_OFF: u32 = 0x0400;

/// Where this function sits: device 5, which is the class default.
const DEVICE_NO: u8 = 5;

// ---------------------------------------------------------------------------
// a router to collect the nets
// ---------------------------------------------------------------------------

/// What a south bridge would be: the four bus interrupt nets, and a log of
/// every change so a test can count transitions rather than sample a level.
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
        self.levels.lock()[usize::from(line)]
    }

    fn transitions(&self) -> usize {
        self.changes.lock().len()
    }
}

impl IntxSink for Router {
    fn intx_changed(&self, line: u8, level: Level) {
        self.levels.lock()[usize::from(line)] = level;
        self.changes.lock().push((line, level));
    }
}

// ---------------------------------------------------------------------------
// the fixture
// ---------------------------------------------------------------------------

struct Fixture {
    function: XhciPci,
    bus: Arc<PciBus>,
    space: Arc<AddressSpace>,
    router: Arc<Router>,
    at: Bdf,
}

fn build() -> Fixture {
    let space = AddressSpace::new("mem", 32);
    {
        let mut topo = space.topology();
        let ram: RegionRef = Arc::new(Region::ram("ram", Arc::new(RamStore::new(RAM_SIZE))));
        topo.map(ram, RAM).expect("the map fits");
    }
    let space = Arc::new(space);

    let bus = Arc::new(PciBus::new());
    let router = Router::new();
    bus.set_intx_sink(Arc::downgrade(&router) as Weak<dyn IntxSink>);

    let usb = Arc::new(UsbBus::new(1));
    let at = Bdf::new(0, DEVICE_NO, 0).expect("a legal device number");
    let function = XhciPci::with_buses(
        Arc::clone(&bus),
        at,
        usb,
        Params {
            ports: 1,
            slots: 4,
            // Short, so a test that lets a microframe pass is cheap.
            microframe_ticks: 8,
        },
        0x1234,
        0x1e31,
        0,
    )
    .expect("the BAR table takes the window");

    // What `realize` does, and what `bind` does, in that order.
    bus.attach(at, Arc::clone(&function.regs) as Arc<dyn PciFunction>)
        .expect("an empty address");
    function.regs.intx.plug(&bus, at);
    function
        .attach_space(&space, RequesterId(9))
        .expect("the window fits");

    Fixture {
        function,
        bus,
        space,
        router,
        at,
    }
}

impl Fixture {
    // -- configuration space, as a guest reaches it -------------------------

    fn cfg_read(&self, offset: u16) -> u32 {
        let mut buf = [0u8; 4];
        self.bus
            .config_read(self.at, offset & !3, &mut buf, MemAttrs::DEFAULT);
        u32::from_le_bytes(buf)
    }

    fn cfg_write(&self, offset: u16, value: u32) {
        self.bus.config_write(
            self.at,
            offset & !3,
            &value.to_le_bytes(),
            MemAttrs::DEFAULT,
        );
    }

    // -- the register block, through the window the BAR placed --------------

    fn reg(&self, offset: u64) -> u32 {
        self.space
            .read(BAR_BASE + offset, Width::U32, MemAttrs::DEFAULT)
            .expect("a decoded dword") as u32
    }

    fn set_reg(&self, offset: u64, value: u32) {
        self.space
            .write(
                BAR_BASE + offset,
                Width::U32,
                u64::from(value),
                MemAttrs::DEFAULT,
            )
            .expect("a decoded dword");
    }

    fn mem(&self, addr: u64) -> u32 {
        self.space
            .read(addr, Width::U32, MemAttrs::DEBUG)
            .expect("mapped RAM") as u32
    }

    fn set_mem(&self, addr: u64, value: u32) {
        self.space
            .write(addr, Width::U32, u64::from(value), MemAttrs::DEBUG)
            .expect("mapped RAM");
    }

    /// Size BAR0 the way §6.2.5.1 says: write all ones, read the mask back,
    /// then put the base in.
    ///
    /// Returns the mask the function reported.
    fn place_bar(&self) -> u32 {
        let saved = self.cfg_read(CFG_BAR0);
        self.cfg_write(CFG_BAR0, 0xffff_ffff);
        let mask = self.cfg_read(CFG_BAR0);
        self.cfg_write(CFG_BAR0, saved);
        self.cfg_write(CFG_BAR0, BAR_BASE as u32);
        self.cfg_write(CFG_BAR1, (BAR_BASE >> 32) as u32);
        mask
    }

    /// Everything before a doorbell: an event ring, a command ring, one slot,
    /// and the interrupter armed.
    fn bring_up(&self) {
        // §6.5: one segment, sixteen TRBs, at EVT_RING.
        self.set_mem(ERST, EVT_RING as u32);
        self.set_mem(ERST + 4, (EVT_RING >> 32) as u32);
        self.set_mem(ERST + 8, EVT_TRBS);
        self.set_mem(ERST + 12, 0);
        for i in 0..EVT_TRBS * 4 {
            self.set_mem(EVT_RING + u64::from(i) * 4, 0);
        }
        self.set_reg(CONFIG, 1);
        self.set_reg(CRCR, CMD_RING as u32 | 1);
        self.set_reg(CRCR + 4, (CMD_RING >> 32) as u32);
        self.set_reg(ERSTSZ, 1);
        self.set_reg(ERDP, EVT_RING as u32);
        self.set_reg(ERDP + 4, (EVT_RING >> 32) as u32);
        self.set_reg(ERSTBA, ERST as u32);
        self.set_reg(ERSTBA + 4, (ERST >> 32) as u32);
        // §5.5.2.1: Interrupter Enable, and §5.4.1: Run/Stop with the master
        // Interrupter Enable.
        self.set_reg(IMAN, 0x2);
        self.set_reg(USBCMD, 0x5);
    }

    /// Put a No Op Command (§6.4.3.1, TRB type 23) on the command ring at
    /// `slot`, owned by the controller.
    fn queue_no_op(&self, slot: u32) {
        let at = CMD_RING + u64::from(slot) * 16;
        self.set_mem(at, 0);
        self.set_mem(at + 4, 0);
        self.set_mem(at + 8, 0);
        // Cycle bit set, so it belongs to the controller.
        self.set_mem(at + 12, (23 << 10) | 1);
    }

    /// The Completion Code of the event TRB at ring slot `slot`, or zero for a
    /// slot the controller has not written.
    fn event_code(&self, slot: u32) -> u32 {
        self.mem(EVT_RING + u64::from(slot) * 16 + 8) >> 24
    }
}

// ---------------------------------------------------------------------------
// the header
// ---------------------------------------------------------------------------

#[test]
fn the_header_is_what_an_operating_system_enumerates_for() {
    let f = build();
    assert_eq!(f.cfg_read(CFG_VENDOR), 0x1e31_1234, "device:vendor");
    // Rev 2.1 §6.2.1: class code in 09h-0Bh, revision in 08h. `0C0330h` is a
    // serial bus controller, USB, xHCI.
    assert_eq!(f.cfg_read(CFG_CLASS), 0x0c03_3000);
    // §6.2.1: header type 00h, and not multi-function.
    assert_eq!((f.cfg_read(CFG_HEADER) >> 16) & 0xff, 0x00);
    // §6.2.4: `INTA#`.
    assert_eq!(
        (f.cfg_read(CFG_INT_PIN) >> 8) & 0xff,
        u32::from(IntxPin::A.0)
    );
    assert_eq!(f.function.intx().pin(), IntxPin::A);
    // §6.2.2: nothing is enabled out of reset.
    assert_eq!(f.cfg_read(CFG_COMMAND) & 0xffff, 0);
}

#[test]
fn the_base_address_register_sizes_to_the_window_it_carries() {
    let f = build();
    // §6.2.5.1: write all ones and read back; the bits that stayed zero are the
    // ones inside the window, and the low four are the format field. Memory,
    // 64-bit (type 10b), not prefetchable.
    let mask = f.place_bar();
    assert_eq!(
        mask,
        (!(BAR_BYTES as u32 - 1)) | 0b0100,
        "a {BAR_BYTES}-byte 64-bit non-prefetchable memory window"
    );
    // The upper half is a plain register with no format bits.
    f.cfg_write(CFG_BAR1, 0xffff_ffff);
    assert_eq!(f.cfg_read(CFG_BAR1), 0xffff_ffff);
    f.cfg_write(CFG_BAR1, 0);
    // A 64-bit register is two of them, so BAR2 is the next one a driver sees
    // and this function declares nothing there.
    assert_eq!(f.cfg_read(CFG_BAR2), 0);
    // The window is 16 KiB, which is the register block rounded up.
    assert_eq!(BAR_BYTES, 0x4000);
    assert_eq!(BAR_BYTES, REGISTER_BYTES.next_power_of_two());
}

#[test]
fn the_window_decodes_nothing_until_command_says_so() {
    let f = build();
    f.place_bar();
    // §6.2.2: Memory Space clear, so the window is not in the map. The space
    // is `read-as-ones`-free here, so an unassigned read is a bus fault.
    assert!(
        f.space
            .read(BAR_BASE, Width::U32, MemAttrs::DEFAULT)
            .is_err(),
        "a function that decodes nothing answers nothing"
    );
    f.cfg_write(CFG_COMMAND, CMD_MEM);
    // `CAPLENGTH` in the low byte and `HCIVERSION` in the top half (§5.3.1,
    // §5.3.2) — the first dword any xHCI driver reads.
    let cap = f.reg(0);
    assert_eq!(cap & 0xff, u32::from(super::super::CAPLENGTH));
    assert_eq!(cap >> 16, u32::from(super::super::HCIVERSION));
    // And it goes away again.
    f.cfg_write(CFG_COMMAND, 0);
    assert!(
        f.space
            .read(BAR_BASE, Width::U32, MemAttrs::DEFAULT)
            .is_err()
    );
}

#[test]
fn an_unimplemented_command_bit_reads_back_zero() {
    let f = build();
    // §6.2.2: a function may hardwire to zero every bit it does not implement.
    // This one decodes no I/O space and has no VGA snoop, no parity response
    // and no SERR#.
    f.cfg_write(CFG_COMMAND, 0xffff);
    assert_eq!(
        f.cfg_read(CFG_COMMAND) & 0xffff,
        u32::from(COMMAND_IMPLEMENTED)
    );
}

// ---------------------------------------------------------------------------
// bus mastering
// ---------------------------------------------------------------------------

#[test]
fn a_function_that_may_not_master_the_bus_fetches_nothing() {
    let f = build();
    f.place_bar();
    // Memory space only: the guest can reach the registers, and the controller
    // can reach nothing.
    f.cfg_write(CFG_COMMAND, CMD_MEM);
    assert!(!f.function.xhci().is_master());

    f.bring_up();
    f.queue_no_op(0);
    f.set_reg(DB0, 0);

    // Not one byte of the command ring was read and not one event was posted:
    // §6.2.2's Bus Master Enable is the whole gate, and it is checked at the
    // one place every walk in `xhci.rs` asks for its address space.
    assert_eq!(f.event_code(0), 0, "an event ring the xHC never wrote");
    assert_eq!(
        f.router.level(swizzle(f.at, IntxPin::A).unwrap()),
        Level::Low
    );

    // Now let it master, and re-arm the event ring: the Event Ring State
    // Machine is started by a write to `ERSTBA` (§5.5.2.3.2), and that write
    // was a DMA read the function was not allowed to make.
    f.cfg_write(CFG_COMMAND, CMD_MEM | CMD_BM);
    assert!(f.function.xhci().is_master());
    f.set_reg(ERSTBA, ERST as u32);
    f.set_reg(ERSTBA + 4, (ERST >> 32) as u32);
    f.set_reg(DB0, 0);

    // §6.4.5: completion code 1 is Success, and a No Op Command answers with a
    // Command Completion Event.
    assert_eq!(f.event_code(0), 1, "the No Op Command completed");
}

// ---------------------------------------------------------------------------
// the pin
// ---------------------------------------------------------------------------

/// Bring the controller up with mastering on, run one command, and leave the
/// interrupt asserted.
fn raised() -> Fixture {
    let f = build();
    f.place_bar();
    f.cfg_write(CFG_COMMAND, CMD_MEM | CMD_BM);
    f.bring_up();
    f.queue_no_op(0);
    f.set_reg(DB0, 0);
    assert_eq!(f.event_code(0), 1, "the command completed");
    f
}

#[test]
fn the_interrupt_reaches_the_router_on_the_swizzled_net() {
    let f = raised();
    // PCI-to-PCI Bridge 1.1 Table 9-1: device 5's `INTA#` is net 1.
    let line = swizzle(f.at, IntxPin::A).expect("this function drives a pin");
    assert_eq!(line, DEVICE_NO % 4);
    assert_eq!(f.router.level(line), Level::High);
    assert_eq!(f.bus.intx_drivers(line), alloc::vec![f.at]);
    // Rev 3.0 §6.2.3: `STATUS[3]` reports the condition.
    assert_eq!((f.cfg_read(CFG_STATUS) >> 16) & 0xffff & 0x08, 0x08);
    let before = f.router.transitions();

    // xHCI 1.2 §5.4.2, §5.5.2.3.3 and §4.17.3, in the order the specification
    // fixes: `USBSTS.EINT`, `ERDP` with `EHB`, then `IMAN.IP`. Only the last of
    // the three drops the pin.
    f.set_reg(offset::OPERATIONAL + 0x04, 1 << 3);
    assert_eq!(f.router.level(line), Level::High, "EINT is not the pin");
    f.set_reg(ERDP, (EVT_RING as u32 + 16) | (1 << 3));
    assert_eq!(f.router.level(line), Level::High, "ERDP.EHB is not the pin");
    f.set_reg(IMAN, 0x3);
    assert_eq!(f.router.level(line), Level::Low, "IMAN.IP is");

    // Exactly one transition, and no bouncing on the way: a pin that pulsed
    // would work until two functions shared the net (Rev 2.1 §2.2.6).
    assert_eq!(f.router.transitions(), before + 1);
    assert_eq!((f.cfg_read(CFG_STATUS) >> 16) & 0x08, 0x00);
}

#[test]
fn interrupt_disable_gates_the_pin_and_not_the_status_bit() {
    let f = raised();
    let line = swizzle(f.at, IntxPin::A).expect("a pin");
    assert_eq!(f.router.level(line), Level::High);

    // Rev 3.0 §6.2.2: with Interrupt Disable set the function drives no
    // `INTx#` — and §6.2.3's Interrupt Status still reports the condition,
    // which is what makes a driver able to poll a masked device at all.
    f.cfg_write(CFG_COMMAND, CMD_MEM | CMD_BM | CMD_INTX_OFF);
    assert_eq!(f.router.level(line), Level::Low, "the pin is gated");
    assert!(f.bus.intx_drivers(line).is_empty(), "and the net released");
    assert_eq!(
        (f.cfg_read(CFG_STATUS) >> 16) & 0x08,
        0x08,
        "STATUS[3] is the condition, not the output"
    );
    // The engine never saw any of it: nothing acknowledged the interrupter.
    assert_eq!(f.function.xhci().irq_level(), Level::High);

    // Clearing it puts the pin back where the condition already was, without
    // the guest having to touch the controller.
    f.cfg_write(CFG_COMMAND, CMD_MEM | CMD_BM);
    assert_eq!(f.router.level(line), Level::High);
}

#[test]
fn the_pin_also_comes_out_on_the_card_edge() {
    let f = raised();
    // A board with no interrupt router wires the pin straight to a controller,
    // which is what `machines/xhci-pci-mini.machine` does. Both destinations
    // are the same pin at two points on the board.
    let landing = Arc::new(Landing::new());
    let wire = Wire::builder()
        .source(WireId::new(7))
        .sink(Arc::clone(&landing) as Arc<dyn WireSink>, 0)
        .build_shared();
    f.function
        .connect(pin::IRQ, WireSource::new(Arc::clone(&wire), WireId::new(7)))
        .expect("the one pin this function has");
    // `Intx::connect` drives the current level immediately, so a wire connected
    // after the condition arose still sees it.
    assert_eq!(landing.level(), Level::High);

    f.set_reg(offset::OPERATIONAL + 0x04, 1 << 3);
    f.set_reg(ERDP, (EVT_RING as u32 + 16) | (1 << 3));
    f.set_reg(IMAN, 0x3);
    assert_eq!(landing.level(), Level::Low);

    assert!(
        f.function
            .connect("nope", WireSource::new(wire, WireId::new(7)))
            .is_err(),
        "an unknown pin name is an error rather than a silent no-op"
    );
}

/// Somewhere for the card-edge wire to land.
#[derive(Debug)]
struct Landing(AtomicBool);

impl Landing {
    fn new() -> Landing {
        Landing(AtomicBool::new(false))
    }

    fn level(&self) -> Level {
        Level::from_bool(self.0.load(Ordering::Relaxed))
    }
}

impl WireSink for Landing {
    fn set_level(&self, _src: WireId, _line: u32, level: Level) {
        self.0.store(level.is_high(), Ordering::Relaxed);
    }
}

// ---------------------------------------------------------------------------
// the debugger
// ---------------------------------------------------------------------------

#[test]
fn a_debugger_may_read_every_register_and_may_write_none() {
    let f = raised();
    let line = swizzle(f.at, IntxPin::A).expect("a pin");

    // A debug read of the whole header, twice, and nothing moves — including
    // `STATUS[3]`, which is the bit a read-to-acknowledge bug would show up in.
    for _ in 0..2 {
        for offset in (0u16..0x40).step_by(4) {
            let mut buf = [0u8; 4];
            f.bus.config_read(f.at, offset, &mut buf, MemAttrs::DEBUG);
        }
    }
    assert_eq!(f.router.level(line), Level::High);
    assert_eq!((f.cfg_read(CFG_STATUS) >> 16) & 0x08, 0x08);

    // A debug write is refused: it would move the window under a running
    // driver, or clear the bus mastering an in-flight transfer depends on.
    let command = f.cfg_read(CFG_COMMAND);
    let bar = f.cfg_read(CFG_BAR0);
    f.bus
        .config_write(f.at, CFG_COMMAND, &0u32.to_le_bytes(), MemAttrs::DEBUG);
    f.bus
        .config_write(f.at, CFG_BAR0, &0u32.to_le_bytes(), MemAttrs::DEBUG);
    assert_eq!(f.cfg_read(CFG_COMMAND), command);
    assert_eq!(f.cfg_read(CFG_BAR0), bar);
}

// ---------------------------------------------------------------------------
// the snapshot
// ---------------------------------------------------------------------------

/// The whole function as a state chunk.
fn snapshot(f: &Fixture) -> Vec<u8> {
    let mut shape = MachineShape::new();
    shape.add_device("xhci", CLASS_NAME).expect("a shape");
    let mut w = StateWriter::new(shape);
    {
        let mut chunk = w.chunk("xhci", CLASS_NAME, STATE_VERSION).expect("a chunk");
        f.function.save(&mut chunk).expect("it saves");
    }
    w.to_vec().expect("bytes")
}

#[test]
fn a_snapshot_round_trips_to_an_identical_state() {
    let a = raised();
    let b = build();

    let first = snapshot(&a);
    let reader = StateReader::new(&first).expect("we just wrote it");
    let chunk = reader
        .load("xhci", CLASS_NAME, STATE_VERSION, &Migrations::new())
        .expect("it is in there");
    b.function
        .load(&mut chunk.reader())
        .expect("our own snapshot loads");
    assert_eq!(snapshot(&b), first, "the state hash must be identical");

    // Everything a guest can see came back: the Command register, where the
    // window went, and the pin.
    assert_eq!(b.cfg_read(CFG_COMMAND), a.cfg_read(CFG_COMMAND));
    assert_eq!(b.cfg_read(CFG_BAR0), a.cfg_read(CFG_BAR0));
    assert_eq!(b.cfg_read(CFG_BAR1), a.cfg_read(CFG_BAR1));
    assert_eq!(b.cfg_read(CFG_STATUS), a.cfg_read(CFG_STATUS));
    let line = swizzle(a.at, IntxPin::A).expect("a pin");
    assert_eq!(b.router.level(line), Level::High);
    // And the derived state that is deliberately *not* in the chunk: the
    // window is back in the map and the engine may master again, both
    // re-derived from the Command register.
    assert!(b.function.xhci().is_master());
    assert_eq!(b.reg(0) & 0xff, u32::from(super::super::CAPLENGTH));
}

#[test]
fn a_reset_puts_the_function_back_where_a_cold_machine_has_it() {
    let f = raised();
    let line = swizzle(f.at, IntxPin::A).expect("a pin");
    assert_eq!(f.router.level(line), Level::High);

    f.function.reset(ResetKind::Cold);

    assert_eq!(f.cfg_read(CFG_COMMAND) & 0xffff, 0, "every enable clear");
    assert_eq!(f.cfg_read(CFG_BAR0) & !0xf, 0, "the base forgotten");
    assert!(!f.function.xhci().is_master(), "and mastering off with it");
    assert_eq!(f.router.level(line), Level::Low);
    assert!(f.bus.intx_drivers(line).is_empty());
    assert!(
        f.space
            .read(BAR_BASE, Width::U32, MemAttrs::DEFAULT)
            .is_err(),
        "the window is out of the map"
    );
}
