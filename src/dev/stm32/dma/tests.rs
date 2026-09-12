//! Tests for the STM32 DMA controllers.

use super::*;

use alloc::vec;
use alloc::vec::Vec;

use crate::core::props::Value;
use crate::core::space::{RamStore, Region as MemRegion};
use crate::core::state::{MachineShape, Migrations, StateReader, StateWriter};
use crate::core::wire::{Wire, WireIdAllocator};

/// Where the RAM under every rig starts, and how much there is.
const RAM_BASE: u64 = 0x2000_0000;
const RAM_LEN: u64 = 0x1000;

/// A stream's register offsets (RM0090 §10.5).
const fn s_cr(s: u64) -> u64 {
    0x10 + 0x18 * s
}
const fn s_ndtr(s: u64) -> u64 {
    s_cr(s) + 4
}
const fn s_par(s: u64) -> u64 {
    s_cr(s) + 8
}
const fn s_m0ar(s: u64) -> u64 {
    s_cr(s) + 0x0c
}
const fn s_m1ar(s: u64) -> u64 {
    s_cr(s) + 0x10
}

/// A channel's register offsets, by the manual's 1-based number (RM0351 §11.6).
const fn c_cr(x: u64) -> u64 {
    0x08 + 0x14 * (x - 1)
}
const fn c_ndtr(x: u64) -> u64 {
    c_cr(x) + 4
}
const fn c_par(x: u64) -> u64 {
    c_cr(x) + 8
}
const fn c_mar(x: u64) -> u64 {
    c_cr(x) + 0x0c
}

/// A controller with a page of RAM under it.
struct Rig {
    dma: Dma,
    space: Arc<AddressSpace>,
    regs: Registers,
}

impl Rig {
    fn new(variant: Variant) -> Rig {
        let space = Arc::new(AddressSpace::new("mem", 32));
        {
            let mut topo = space.topology();
            topo.map(
                Arc::new(MemRegion::ram("ram", Arc::new(RamStore::new(RAM_LEN)))),
                RAM_BASE,
            )
            .expect("maps");
        }
        let dma = Dma::with_variant(variant);
        dma.attach_bus(&space, RequesterId::ANONYMOUS);
        let regs = Registers {
            shared: Arc::clone(&dma.shared),
        };
        Rig { dma, space, regs }
    }

    fn stream() -> Rig {
        Rig::new(Variant::Stream)
    }

    fn channel() -> Rig {
        Rig::new(Variant::Channel)
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

    /// `LISR` / `ISR`.
    fn isr(&self) -> u32 {
        self.peek(0x00)
    }

    fn put(&self, addr: u64, width: Width, value: u64) {
        self.space
            .write(addr, width, value, MemAttrs::DEFAULT)
            .expect("mapped");
    }

    fn get(&self, addr: u64, width: Width) -> u64 {
        self.space
            .read(addr, width, MemAttrs::DEFAULT)
            .expect("mapped")
    }

    fn byte(&self, addr: u64) -> u8 {
        self.get(addr, Width::U8) as u8
    }
}

// ---------------------------------------------------------------------------
// the channel face
// ---------------------------------------------------------------------------

#[test]
fn mem2mem_copies_with_size_conversion_and_sets_tcif() {
    let rig = Rig::channel();
    let src = RAM_BASE;
    let dst = RAM_BASE + 0x100;
    for i in 0..4u64 {
        rig.put(src + i * 4, Width::U32, 0x1122_3300 | i);
    }

    rig.poke(c_par(1), src as u32);
    rig.poke(c_mar(1), dst as u32);
    rig.poke(c_ndtr(1), 4);
    // MEM2MEM, PSIZE = 32, MSIZE = 8, PINC | MINC, TCIE, EN.
    rig.poke(
        c_cr(1),
        C_CR_MEM2MEM | (2 << 8) | C_CR_PINC | C_CR_MINC | C_CR_TCIE | CR_EN,
    );

    // No request line is wired and none is needed.
    assert_eq!(rig.dma.pump(16), 4, "four beats and then nothing to do");

    for i in 0..4u64 {
        assert_eq!(
            rig.byte(dst + i),
            i as u8,
            "RM0351 Table 41: a narrowing conversion keeps the low byte"
        );
    }
    assert_eq!(rig.dma.remaining(0), 0);
    assert!(!rig.dma.is_running(0), "a non-circular channel disables");
    assert_eq!(rig.peek(c_cr(1)) & CR_EN, 0, "EN is cleared in hardware");
    // GIF1, TCIF1 and HTIF1 — a four-item transfer passed its midpoint too.
    assert_eq!(
        rig.isr() & 0xf,
        (1 << C_GIF) | (1 << C_TCIF) | (1 << C_HTIF)
    );

    // Write-1-to-clear through `IFCR`, one flag at a time…
    rig.poke(0x04, 1 << C_TCIF);
    assert_eq!(
        rig.isr() & 0xf,
        (1 << C_GIF) | (1 << C_HTIF),
        "GIF is the OR of the other three, so HTIF holds it up"
    );
    // …or the whole nibble through `CGIF1` (RM0351 §11.6.2).
    rig.poke(0x04, 1 << C_GIF);
    assert_eq!(rig.isr() & 0xf, 0);
}

#[test]
fn a_widening_conversion_zero_extends() {
    let rig = Rig::channel();
    let src = RAM_BASE;
    let dst = RAM_BASE + 0x100;
    rig.put(src, Width::U8, 0xab);
    rig.put(dst, Width::U32, 0xffff_ffff);

    rig.poke(c_par(1), src as u32);
    rig.poke(c_mar(1), dst as u32);
    rig.poke(c_ndtr(1), 1);
    // PSIZE = 8, MSIZE = 32.
    rig.poke(c_cr(1), C_CR_MEM2MEM | (2 << 10) | CR_EN);
    assert_eq!(rig.dma.pump(4), 1);
    assert_eq!(rig.get(dst, Width::U32), 0x0000_00ab);
}

#[test]
fn the_channel_face_decodes_seven_channels_and_cselr() {
    let rig = Rig::channel();
    assert_eq!(Variant::Channel.window(), 0xac);
    rig.poke(0xa8, 0x0123_4567);
    assert_eq!(rig.peek(0xa8), 0x0123_4567, "CSELR round-trips");
    // Channel 7's `CMAR` is the last register before `CSELR`.
    rig.poke(c_mar(7), 0xdead_beef);
    assert_eq!(rig.peek(c_mar(7)), 0xdead_beef);
    // Bits 31..15 of `CCRx` are reserved and read as zero.
    rig.poke(c_cr(3), !CR_EN);
    assert_eq!(rig.peek(c_cr(3)), C_CR_MASK & !CR_EN);
}

// ---------------------------------------------------------------------------
// the stream face, and the request line
// ---------------------------------------------------------------------------

/// Program stream 0 for a peripheral-to-memory byte transfer of `n` items.
fn arm_p2m(rig: &Rig, periph: u64, mem: u64, n: u32, extra: u32) {
    rig.poke(s_par(0), periph as u32);
    rig.poke(s_m0ar(0), mem as u32);
    rig.poke(s_ndtr(0), n);
    // DIR = 00, PSIZE = MSIZE = 8, MINC, every interrupt enabled.
    rig.poke(
        s_cr(0),
        S_CR_MINC | S_CR_TCIE | S_CR_HTIE | S_CR_TEIE | extra | CR_EN,
    );
}

#[test]
fn a_peripheral_request_moves_one_unit_per_request() {
    let rig = Rig::stream();
    let periph = RAM_BASE + 0x20;
    let mem = RAM_BASE + 0x100;
    rig.put(periph, Width::U8, 0x5a);
    arm_p2m(&rig, periph, mem, 3, 0);

    assert_eq!(rig.dma.pump(16), 0, "nothing is asking yet");
    assert_eq!(rig.dma.remaining(0), 3);

    // A peripheral that pulses its line once per item: one beat per pulse.
    let pulse = || {
        rig.dma.set_request(0, Level::High);
        rig.dma.set_request(0, Level::Low);
    };

    pulse();
    assert_eq!(rig.dma.pump(16), 1, "one pulse buys exactly one beat");
    assert_eq!(rig.dma.remaining(0), 2);
    assert_eq!(rig.isr() & (1 << S_HTIF), 0, "not half way yet");

    pulse();
    assert_eq!(rig.dma.pump(16), 1);
    assert_eq!(rig.dma.remaining(0), 1);
    assert_ne!(rig.isr() & (1 << S_HTIF), 0, "HTIF at the midpoint");
    assert_eq!(rig.isr() & (1 << S_TCIF), 0);

    pulse();
    assert_eq!(rig.dma.pump(16), 1);
    assert_eq!(rig.dma.remaining(0), 0);
    assert_ne!(rig.isr() & (1 << S_TCIF), 0, "TCIF at zero");
    assert!(!rig.dma.is_running(0));

    for i in 0..3 {
        assert_eq!(rig.byte(mem + i), 0x5a);
    }
    assert_eq!(rig.byte(mem + 3), 0, "and not one byte further");
}

#[test]
fn a_held_request_line_gives_continuous_service() {
    let rig = Rig::stream();
    let periph = RAM_BASE + 0x20;
    let mem = RAM_BASE + 0x100;
    rig.put(periph, Width::U8, 0x77);
    arm_p2m(&rig, periph, mem, 8, 0);

    // A FIFO-style peripheral: the line stays high while it has data.
    rig.dma.set_request(0, Level::High);
    assert_eq!(rig.dma.pump(3), 3, "the budget, not the line, is the limit");
    assert_eq!(rig.dma.remaining(0), 5);
    rig.dma.set_request(0, Level::Low);
    assert_eq!(rig.dma.pump(16), 0, "the line went away and so did service");
    assert_eq!(rig.dma.remaining(0), 5);
}

#[test]
fn circular_mode_reloads_ndtr_and_keeps_going() {
    let rig = Rig::stream();
    let periph = RAM_BASE + 0x20;
    let mem = RAM_BASE + 0x100;
    rig.put(periph, Width::U8, 0x11);
    arm_p2m(&rig, periph, mem, 4, S_CR_CIRC);
    rig.dma.set_request(0, Level::High);

    assert_eq!(rig.dma.pump(4), 4);
    assert_eq!(rig.dma.remaining(0), 4, "reloaded, not stopped at zero");
    assert!(rig.dma.is_running(0));
    assert_ne!(rig.isr() & (1 << S_TCIF), 0);
    assert_ne!(rig.isr() & (1 << S_HTIF), 0);

    // The memory pointer went back to `M0AR`, so the second pass overwrites
    // the first — which is what a circular buffer is.
    rig.put(periph, Width::U8, 0x22);
    rig.poke(0x08, (1 << S_TCIF) | (1 << S_HTIF)); // LIFCR
    assert_eq!(rig.isr(), 0);
    assert_eq!(rig.dma.pump(4), 4);
    for i in 0..4 {
        assert_eq!(rig.byte(mem + i), 0x22, "the second lap");
    }
    assert_ne!(rig.isr() & (1 << S_TCIF), 0, "and TCIF again");
}

#[test]
fn double_buffer_alternates_between_m0ar_and_m1ar() {
    let rig = Rig::stream();
    let periph = RAM_BASE + 0x20;
    let a = RAM_BASE + 0x100;
    let b = RAM_BASE + 0x200;
    rig.put(periph, Width::U8, 0xa0);
    rig.poke(s_m1ar(0), b as u32);
    arm_p2m(&rig, periph, a, 2, S_CR_DBM);
    rig.dma.set_request(0, Level::High);

    assert_eq!(rig.dma.pump(2), 2);
    assert_ne!(rig.peek(s_cr(0)) & S_CR_CT, 0, "CT flipped to buffer 1");
    rig.put(periph, Width::U8, 0xb0);
    assert_eq!(rig.dma.pump(2), 2);
    assert_eq!(rig.peek(s_cr(0)) & S_CR_CT, 0, "and back");

    assert_eq!([rig.byte(a), rig.byte(a + 1)], [0xa0, 0xa0]);
    assert_eq!([rig.byte(b), rig.byte(b + 1)], [0xb0, 0xb0]);
}

#[test]
fn a_transfer_into_an_unmapped_address_sets_teif_and_disables_the_stream() {
    let rig = Rig::stream();
    let periph = RAM_BASE + 0x20;
    rig.put(periph, Width::U8, 0x5a);
    // Nothing is mapped at 0x9000_0000 and the space faults an unassigned
    // access by default.
    arm_p2m(&rig, periph, 0x9000_0000, 4, 0);
    rig.dma.set_request(0, Level::High);

    assert_eq!(
        rig.dma.pump(16),
        1,
        "one beat tried, then the stream is off"
    );
    assert_ne!(rig.isr() & (1 << S_TEIF), 0, "TEIF");
    assert_eq!(rig.isr() & (1 << S_TCIF), 0, "and no completion");
    assert!(!rig.dma.is_running(0));
    assert_eq!(rig.peek(s_cr(0)) & CR_EN, 0);
    assert_eq!(rig.dma.remaining(0), 4, "the faulted item is not counted");
}

#[test]
fn a_reserved_size_code_is_a_configuration_error() {
    let rig = Rig::stream();
    // PSIZE = 11b is reserved (RM0090 §10.5.5).
    rig.poke(s_ndtr(0), 4);
    rig.poke(s_cr(0), (3 << 11) | S_CR_TEIE | CR_EN);
    assert_eq!(rig.peek(s_cr(0)) & CR_EN, 0, "the stream refuses to arm");
    assert_ne!(rig.isr() & (1 << S_TEIF), 0);
    assert_eq!(rig.dma.pump(16), 0);
}

#[test]
fn ndtr_par_and_m0ar_are_write_protected_while_the_stream_runs() {
    let rig = Rig::stream();
    let periph = RAM_BASE + 0x20;
    let mem = RAM_BASE + 0x100;
    arm_p2m(&rig, periph, mem, 4, 0);
    assert!(rig.dma.is_running(0));

    rig.poke(s_ndtr(0), 99);
    rig.poke(s_par(0), 0x1234);
    rig.poke(s_m0ar(0), 0x5678);
    assert_eq!(rig.dma.remaining(0), 4);
    assert_eq!(rig.peek(s_par(0)), periph as u32);
    assert_eq!(rig.peek(s_m0ar(0)), mem as u32);
    // `M1AR` is not protected: swapping the idle half is what DBM is for.
    rig.poke(s_m1ar(0), 0x9abc);
    assert_eq!(rig.peek(s_m1ar(0)), 0x9abc);

    // `DIR` is protected too, but `EN` and the interrupt enables are not.
    rig.poke(s_cr(0), (1 << 6) | CR_EN);
    assert_eq!(rig.peek(s_cr(0)) & (3 << 6), 0, "DIR did not change");
    rig.poke(s_cr(0), 0);
    assert!(!rig.dma.is_running(0), "clearing EN always works");
}

#[test]
fn software_priority_is_served_before_a_lower_stream_number() {
    let rig = Rig::stream();
    let periph = RAM_BASE + 0x20;
    rig.put(periph, Width::U8, 1);

    // Stream 0 at PL = 0, stream 1 at PL = 3 (very high).
    rig.poke(s_par(0), periph as u32);
    rig.poke(s_m0ar(0), (RAM_BASE + 0x100) as u32);
    rig.poke(s_ndtr(0), 4);
    rig.poke(s_cr(0), S_CR_MINC | CR_EN);

    rig.poke(s_par(1), periph as u32);
    rig.poke(s_m0ar(1), (RAM_BASE + 0x200) as u32);
    rig.poke(s_ndtr(1), 4);
    rig.poke(s_cr(1), S_CR_MINC | (3 << 16) | CR_EN);

    rig.dma.set_request(0, Level::High);
    rig.dma.set_request(1, Level::High);
    assert_eq!(rig.dma.pump(4), 4);
    assert_eq!(rig.dma.remaining(1), 0, "the very-high stream finished");
    assert_eq!(rig.dma.remaining(0), 4, "the low one has not started");
}

// ---------------------------------------------------------------------------
// re-entrancy: the reason this device has its own file
// ---------------------------------------------------------------------------

/// A peripheral whose data register, when the DMA writes it, takes the
/// peripheral's *own* `DEVICE`-ranked lock and raises its request line from
/// inside that critical section.
///
/// This is the shape `CLAUDE.md`'s re-entrancy contract exists for. Two things
/// are being asserted, and both are asserted by the test not panicking in a
/// debug build, where `core::sync`'s rank ladder turns a lock-order violation
/// into a panic naming both ranks:
///
/// * the controller does **not** hold its own `DEVICE`-ranked state lock
///   across the bus access, or taking this one would violate the ladder;
/// * raising a request takes no lock at all, so the controller can never be
///   re-entered into a lock it is already holding.
#[derive(Debug)]
struct Peripheral {
    /// The bytes the controller has written, behind a lock of the same rank
    /// the controller uses for its own registers.
    fifo: Mutex<Vec<u8>>,
    dma: Mutex<Option<Arc<Shared>>>,
    /// How much room is left, so the line drops when the peripheral is full.
    room: Mutex<u32>,
}

impl MemOps for Peripheral {
    fn read(&self, _offset: u64, dst: &mut [u8], _attrs: MemAttrs) -> MemResult {
        dst.fill(0);
        Ok(())
    }

    fn write(&self, _offset: u64, src: &[u8], _attrs: MemAttrs) -> MemResult {
        // The critical section the contract is about: our own state, mutated
        // under our own lock, while the controller is mid-transfer.
        let level = {
            let mut fifo = self.fifo.lock();
            fifo.push(src[0]);
            let mut room = self.room.lock();
            *room = room.saturating_sub(1);
            Level::from_bool(*room > 0)
        };
        // And the outward call, after the section: a peripheral asking for the
        // next beat. Nothing of the controller's may be held for this to be
        // safe, and nothing of ours is held either.
        let dma = self.dma.lock().clone();
        if let Some(dma) = dma {
            dma.set_request(0, 0, level);
        }
        Ok(())
    }

    fn constraints(&self) -> AccessConstraints {
        AccessConstraints::word(Width::U8, Endian::Little)
    }
}

/// A wire sink that takes a `DEVICE`-ranked lock, the way an interrupt
/// controller's own register state would.
#[derive(Debug)]
struct IrqProbe {
    seen: Mutex<Vec<Level>>,
}

impl WireSink for IrqProbe {
    fn set_level(&self, _src: WireId, _line: u32, level: Level) {
        self.seen.lock().push(level);
    }
}

#[test]
fn a_peripheral_may_raise_its_request_from_inside_its_own_register_write() {
    let rig = Rig::stream();
    let peripheral = Arc::new(Peripheral {
        fifo: Mutex::with_rank(LockRank::DEVICE, Vec::new()),
        dma: Mutex::with_rank(LockRank::LEAF, Some(Arc::clone(&rig.dma.shared))),
        room: Mutex::with_rank(LockRank::WIRE, 6),
    });
    let port = RAM_BASE + 0x800;
    {
        let mut topo = rig.space.topology();
        topo.map(
            Arc::new(MemRegion::io(
                "peripheral",
                4,
                Arc::clone(&peripheral) as Arc<dyn MemOps>,
            )),
            port,
        )
        .expect("maps");
    }

    // An interrupt sink that locks, too: `drive_irq` must run with nothing of
    // the controller's held.
    let probe = Arc::new(IrqProbe {
        seen: Mutex::with_rank(LockRank::DEVICE, Vec::new()),
    });
    let ids = WireIdAllocator::new();
    let id = ids.alloc();
    let wire = Arc::new(
        Wire::builder()
            .source(id)
            .sink(Arc::clone(&probe) as Arc<dyn WireSink>, 0)
            .build(),
    );
    rig.dma.connect_irq(0, WireSource::new(wire, id));

    // Memory to peripheral: six bytes out of RAM into the FIFO.
    let mem = RAM_BASE + 0x100;
    for i in 0..6u64 {
        rig.put(mem + i, Width::U8, 0x40 + i);
    }
    rig.poke(s_par(0), port as u32);
    rig.poke(s_m0ar(0), mem as u32);
    rig.poke(s_ndtr(0), 6);
    // DIR = 01 (memory to peripheral), MINC, TCIE.
    rig.poke(s_cr(0), (1 << 6) | S_CR_MINC | S_CR_TCIE | CR_EN);

    rig.dma.set_request(0, Level::High);
    assert_eq!(rig.dma.pump(16), 6);
    assert_eq!(
        *peripheral.fifo.lock(),
        vec![0x40, 0x41, 0x42, 0x43, 0x44, 0x45]
    );
    assert_eq!(rig.dma.remaining(0), 0);
    assert_eq!(
        probe.seen.lock().last().copied(),
        Some(Level::High),
        "TCIE was set, so the line ends up asserted"
    );
}

/// The other half of the seam: a peripheral that offers a [`DmaPeripheral`]
/// instead of driving a level.
#[derive(Debug)]
struct ReadyPeer {
    ready: AtomicBool,
}

impl DmaPeripheral for ReadyPeer {
    fn dma_read(&self, _terminal: bool) -> u8 {
        unreachable!("the STM32 controller reads the bus at CPAR, not the peer")
    }
    fn dma_write(&self, _byte: u8, _terminal: bool) {
        unreachable!("the STM32 controller writes the bus at CPAR, not the peer")
    }
    fn dma_ready(&self) -> bool {
        self.ready.load(Ordering::SeqCst)
    }
}

#[test]
fn a_dma_peripheral_peer_is_polled_for_readiness() {
    let rig = Rig::stream();
    let peer = Arc::new(ReadyPeer {
        ready: AtomicBool::new(false),
    });
    rig.dma
        .attach_dma_peripheral("req0", Arc::downgrade(&peer) as Weak<dyn DmaPeripheral>);

    let periph = RAM_BASE + 0x20;
    rig.put(periph, Width::U8, 0x33);
    arm_p2m(&rig, periph, RAM_BASE + 0x100, 2, 0);

    assert_eq!(rig.dma.pump(8), 0, "the peer says it is not ready");
    peer.ready.store(true, Ordering::SeqCst);
    assert_eq!(rig.dma.pump(8), 2, "and now it is");
}

// ---------------------------------------------------------------------------
// snapshots, debug accesses, properties
// ---------------------------------------------------------------------------

#[test]
fn dma_state_survives_save_and_load_mid_transfer() {
    let saved = Rig::stream();
    let periph = RAM_BASE + 0x20;
    let mem = RAM_BASE + 0x100;
    saved.put(periph, Width::U8, 0x5a);
    arm_p2m(&saved, periph, mem, 8, S_CR_CIRC);
    saved.dma.set_request(0, Level::High);
    assert_eq!(saved.dma.pump(5), 5, "stopped half way through on purpose");
    assert_eq!(saved.dma.remaining(0), 3);

    let mut shape = MachineShape::new();
    shape.add_device("dma", CLASS_NAME).unwrap();
    let mut w = StateWriter::new(shape);
    {
        let mut chunk = w.chunk("dma", CLASS_NAME, STATE_VERSION).unwrap();
        Device::save(&saved.dma, &mut chunk).unwrap();
    }
    let bytes = w.to_vec().unwrap();

    let restored = Rig::stream();
    restored.put(periph, Width::U8, 0x5a);
    let reader = StateReader::new(&bytes).unwrap();
    let chunk = reader
        .load("dma", CLASS_NAME, STATE_VERSION, &Migrations::new())
        .unwrap();
    Device::load(&restored.dma, &mut chunk.reader()).unwrap();

    // Every register reads the same on both sides.
    let image = |rig: &Rig| -> Vec<u32> {
        (0..Variant::Stream.window() / 4)
            .map(|i| rig.peek(i * 4))
            .collect()
    };
    assert_eq!(image(&saved), image(&restored));
    assert_eq!(restored.dma.remaining(0), 3);
    assert!(restored.dma.is_running(0));

    // And it finishes where the original would have: the held request line
    // travelled with the state, so no new wire event is needed.
    assert_eq!(restored.dma.pump(3), 3);
    assert_eq!(restored.dma.remaining(0), 8, "reloaded, circular");
    // The restored controller has its own RAM, so only the three beats it ran
    // itself are there — and they are the three the original had left, at the
    // memory pointer the snapshot carried.
    for i in 5..8 {
        assert_eq!(restored.byte(mem + i), 0x5a, "it resumed at item {i}");
    }
    assert_eq!(restored.byte(mem), 0, "and did not start over from M0AR");
}

#[test]
fn a_reset_clears_every_stream_and_every_request() {
    let rig = Rig::stream();
    let periph = RAM_BASE + 0x20;
    arm_p2m(&rig, periph, RAM_BASE + 0x100, 4, 0);
    rig.dma.set_request(0, Level::High);
    assert!(rig.dma.is_running(0));

    Device::reset(&rig.dma, ResetKind::Cold);
    assert!(!rig.dma.is_running(0));
    assert_eq!(rig.peek(s_cr(0)), 0);
    assert_eq!(rig.isr(), 0);
    // `SxFCR`'s reset value is 0x21 — RM0090 §10.5.10.
    assert_eq!(rig.peek(s_cr(0) + 0x14), S_FCR_RESET);
    assert_eq!(rig.dma.pump(8), 0, "the latched request went with it");
}

#[test]
fn a_debug_read_changes_nothing_and_a_debug_write_is_refused() {
    let rig = Rig::stream();
    let periph = RAM_BASE + 0x20;
    arm_p2m(&rig, periph, RAM_BASE + 0x100, 4, 0);
    rig.dma.set_request(0, Level::High);
    assert_eq!(rig.dma.pump(2), 2);

    let debug = MemAttrs::DEFAULT.with_debug(true);
    let mut bytes = [0u8; 4];
    rig.regs.read(s_ndtr(0), &mut bytes, debug).unwrap();
    assert_eq!(
        u32::from_le_bytes(bytes),
        2,
        "NDTR reads, and does not move"
    );
    assert_eq!(rig.dma.remaining(0), 2);

    assert!(
        rig.regs.write(0x08, &1u32.to_le_bytes(), debug).is_err(),
        "a debug write to LIFCR would drop a flag the guest has not seen"
    );
}

#[test]
fn the_stream_face_packs_flags_where_rm0090_says() {
    let rig = Rig::stream();
    {
        let mut state = rig.dma.shared.state.lock();
        for (i, unit) in state.unit.iter_mut().enumerate() {
            unit.flags = if i % 2 == 0 { F_TC } else { F_HT };
        }
    }
    // Streams 0..3 in LISR at shifts 0, 6, 16, 22; 4..7 in HISR the same.
    assert_eq!(
        rig.peek(0x00),
        (1 << S_TCIF) | (1 << (6 + S_HTIF)) | (1 << (16 + S_TCIF)) | (1 << (22 + S_HTIF))
    );
    assert_eq!(
        rig.peek(0x04),
        (1 << S_TCIF) | (1 << (6 + S_HTIF)) | (1 << (16 + S_TCIF)) | (1 << (22 + S_HTIF))
    );
    // `HIFCR` clears the high half only.
    rig.poke(0x0c, 0xffff_ffff);
    assert_ne!(rig.peek(0x00), 0);
    assert_eq!(rig.peek(0x04), 0);
}

#[test]
fn pins_follow_the_manuals_numbering() {
    let stream = Dma::with_variant(Variant::Stream);
    assert_eq!(stream.pin_index("irq0", "irq"), Some(0));
    assert_eq!(stream.req_pin("req7"), Some((7, 0)));
    assert_eq!(stream.req_pin("req8"), None);

    let channel = Dma::with_variant(Variant::Channel);
    assert_eq!(channel.pin_index("irq1", "irq"), Some(0));
    assert_eq!(channel.req_pin("req7"), Some((6, 0)));
    assert_eq!(channel.req_pin("req0"), None, "channels count from 1");
    assert!(Device::connect(&channel, "irq0", never_driven()).is_err());
}

#[test]
fn a_selector_qualified_pin_names_a_cell_of_the_request_matrix() {
    let stream = Dma::with_variant(Variant::Stream);
    // RM0090 Table 43's SDIO cell: DMA2, stream 3, channel 4. Slot 5, because
    // slot 0 is the unnumbered pin.
    assert_eq!(stream.req_pin("req3c4"), Some((3, 5)));
    assert_eq!(stream.req_pin("req0c0"), Some((0, 1)));
    assert_eq!(
        stream.req_pin("req0c15"),
        Some((0, 16)),
        "the channel face's four-bit CSELR sets the bound, not CHSEL's three"
    );
    assert_eq!(stream.req_pin("req0c16"), None);
    assert_eq!(stream.req_pin("req8c0"), None);
    assert_eq!(stream.req_pin("reqc0"), None);
    assert_eq!(stream.req_pin("req0c"), None);

    // The channel face renumbers the unit and keeps the selector.
    let channel = Dma::with_variant(Variant::Channel);
    assert_eq!(channel.req_pin("req1c9"), Some((0, 10)));
    assert_eq!(channel.req_pin("req0c1"), None);
}

fn never_driven() -> WireSource {
    let ids = WireIdAllocator::new();
    let id = ids.alloc();
    WireSource::new(Arc::new(Wire::builder().source(id).build()), id)
}

#[test]
fn the_dmamux_is_named_rather_than_faked() {
    let props = Props::new().with("mux", Value::from(true));
    let err = Dma::new(&props).expect_err("DMAMUX is not written");
    assert!(
        err.to_string().contains("st.dmamux"),
        "the error names the missing class: {err}"
    );
    assert_eq!(
        Dma::new(&Props::new()).unwrap().variant(),
        Variant::Stream,
        "the F4 is the part in this tree, so it is the default"
    );
    assert_eq!(
        Dma::new(&Props::new().with("variant", Value::from("channel")))
            .unwrap()
            .variant(),
        Variant::Channel
    );
}

#[test]
fn a_controller_without_a_space_is_a_machine_file_bug() {
    // `bind` refuses rather than silently transferring nothing; the message is
    // what a board author reads.
    let dma = Dma::with_variant(Variant::Stream);
    assert!(dma.shared.bus().is_none());
    assert_eq!(dma.pump(8), 0);
}

#[test]
fn a_run_consumes_its_budget_when_there_is_nothing_to_move() {
    let rig = Rig::stream();
    let budget = Budget {
        until: crate::core::clock::GlobalTime::ZERO,
        ticks: 10_000,
    };
    assert_eq!(Device::run(&rig.dma, budget).ticks, 10_000);

    // With work to do, a run is capped so one quantum cannot spin forever on a
    // circular memory-to-memory transfer.
    let src = RAM_BASE;
    rig.poke(s_par(0), src as u32);
    rig.poke(s_m0ar(0), (RAM_BASE + 0x100) as u32);
    rig.poke(s_ndtr(0), 8);
    // DIR = 10 (memory to memory), CIRC.
    rig.poke(s_cr(0), (2 << 6) | S_CR_CIRC | CR_EN);
    assert_eq!(Device::run(&rig.dma, budget).ticks, MAX_BEATS_PER_RUN);
}

// ---------------------------------------------------------------------------
// the board file's half
// ---------------------------------------------------------------------------

/// The wiring a board writes, built and run for real.
///
/// Two things are being checked that a hand-wired rig cannot: that `bind`
/// finds the space from `space = mem` and refuses without it, and that the
/// scheduler paces the transfer at one beat per tick of the domain the board
/// picked rather than finishing it inside the register write.
const BOARD: &str = r#"
machine "dma-mini" {
  osc ahb = 8000000 Hz

  space mem { width = 32 }

  object ram "ram" { size = 4K }

  object dma2 "st.dma" {
    clock   = ahb / 4
    space   = mem
    variant = "stream"
  }

  map mem 0x20000000 size 4K   = ram
  map mem 0x40026400 size 0xd0 = dma2
}
"#;

#[test]
fn a_board_gets_a_bus_master_paced_by_its_own_clock_domain() {
    let options = crate::machine::BuildOptions::new()
        .with_bindings(crate::machine::catalog::bindings().expect("bindings"))
        .with_classes(crate::machine::catalog::classes());
    let mut machine = match crate::machine::build(
        "dma-mini.machine",
        BOARD,
        &crate::machine::catalog::registry().expect("registry"),
        &options,
    ) {
        Ok(m) => m,
        Err(e) => panic!("{e}"),
    };

    let space = Arc::clone(machine.space("mem").expect("mem"));
    let regs = 0x4002_6400u64;
    let src = 0x2000_0000u64;
    let dst = 0x2000_0100u64;
    let poke = |addr: u64, value: u64| {
        space
            .write(addr, Width::U32, value, MemAttrs::DEFAULT)
            .expect("mapped");
    };
    space
        .write(src, Width::U8, 0xc3, MemAttrs::DEFAULT)
        .expect("mapped");

    // Three thousand bytes out of one fixed address, which is longer than the
    // 2 MHz domain can move inside one 1 ms quantum. That is the point: the
    // transfer has to survive being cut in half by the scheduler.
    poke(regs + s_par(0), src);
    poke(regs + s_m0ar(0), dst);
    poke(regs + s_ndtr(0), 3000);
    // DIR = 10 (memory to memory), PSIZE = MSIZE = 8, MINC, EN.
    poke(regs + s_cr(0), u64::from((2 << 6) | S_CR_MINC | CR_EN));

    let ndtr = || {
        space
            .read(regs + s_ndtr(0), Width::U32, MemAttrs::DEFAULT)
            .expect("mapped")
    };
    assert_eq!(ndtr(), 3000, "the write that set EN moved nothing");

    // One quantum of a 2 MHz domain is 2000 ticks, and a tick is a beat.
    machine
        .run_until(crate::core::clock::GlobalTime::from_nanos(1_000_000))
        .expect("runs");
    assert_eq!(
        ndtr(),
        1000,
        "exactly one beat per tick, and then it stopped"
    );
    assert_eq!(
        space
            .read(dst + 1999, Width::U8, MemAttrs::DEFAULT)
            .expect("mapped"),
        0xc3
    );
    assert_eq!(
        space
            .read(dst + 2000, Width::U8, MemAttrs::DEFAULT)
            .expect("mapped"),
        0,
        "and not one byte past where the quantum ended"
    );

    machine
        .run_until(crate::core::clock::GlobalTime::from_nanos(2_000_000))
        .expect("runs");
    assert_eq!(ndtr(), 0, "the next quantum finishes it");
    // TCIF0 and HTIF0 in `LISR`.
    let lisr = space
        .read(regs, Width::U32, MemAttrs::DEFAULT)
        .expect("mapped");
    assert_ne!(lisr & (1 << S_TCIF), 0);
}

#[test]
fn a_board_that_forgets_the_space_is_told_so() {
    let without = BOARD.replace("    space   = mem\n", "");
    let options = crate::machine::BuildOptions::new()
        .with_bindings(crate::machine::catalog::bindings().expect("bindings"))
        .with_classes(crate::machine::catalog::classes());
    let err = crate::machine::build(
        "dma-mini.machine",
        &without,
        &crate::machine::catalog::registry().expect("registry"),
        &options,
    )
    .expect_err("a DMA controller with no space masters nothing");
    assert!(
        err.to_string().contains("masters the bus"),
        "the message names the fix: {err}"
    );
}

// ---------------------------------------------------------------------------
// `CHSEL`, the other half of RM0090 Table 43
// ---------------------------------------------------------------------------

/// `CHSEL` in `SxCR`, RM0090 §10.5.5 bits 27:25.
const fn chsel(n: u32) -> u32 {
    n << 25
}

#[test]
fn a_request_on_the_wrong_channel_is_not_served() {
    let rig = Rig::stream();
    let periph = RAM_BASE + 0x20;
    rig.put(periph, Width::U8, 0x5a);
    // Stream 0 listening to channel 4, which is where Table 43 puts SDIO.
    arm_p2m(&rig, periph, RAM_BASE + 0x100, 4, chsel(4));

    // Something else on the same stream, on channel 2.
    rig.dma.set_selected_request(0, 2, Level::High);
    assert_eq!(
        rig.dma.pump(8),
        0,
        "CHSEL says 4, so channel 2 is not heard"
    );
    assert_eq!(rig.dma.remaining(0), 4);

    rig.dma.set_selected_request(0, 4, Level::High);
    assert_eq!(rig.dma.pump(8), 4, "and channel 4 is");
    assert_eq!(rig.dma.remaining(0), 0);
}

#[test]
fn the_unnumbered_pin_is_served_whatever_chsel_reads() {
    let rig = Rig::stream();
    let periph = RAM_BASE + 0x20;
    rig.put(periph, Width::U8, 0x11);
    arm_p2m(&rig, periph, RAM_BASE + 0x100, 2, chsel(7));

    // A board that wired `req0` said it was not modelling the selector.
    rig.dma.set_request(0, Level::High);
    assert_eq!(rig.dma.pump(8), 2);
}

#[test]
fn chsel_is_write_protected_while_the_stream_runs() {
    let rig = Rig::stream();
    let periph = RAM_BASE + 0x20;
    rig.put(periph, Width::U8, 0x22);
    arm_p2m(&rig, periph, RAM_BASE + 0x100, 4, chsel(3));
    rig.dma.set_selected_request(0, 3, Level::High);
    assert_eq!(rig.dma.pump(2), 2);

    // `CHSEL` is write-protected while `EN` is set (RM0090 §10.5.5), so this
    // write is dropped and the stream keeps hearing channel 3.
    rig.poke(s_cr(0), S_CR_MINC | chsel(5) | CR_EN);
    assert_eq!(rig.peek(s_cr(0)) & chsel(7), chsel(3));
    assert_eq!(rig.dma.pump(2), 2, "still channel 3");
}

#[test]
fn the_channel_face_gates_on_its_cselr_nibble() {
    let rig = Rig::channel();
    let periph = RAM_BASE + 0x20;
    let mem = RAM_BASE + 0x100;
    rig.put(periph, Width::U8, 0x77);
    // Channel 3 — unit 2 — selecting request 9 out of `CSELR`.
    rig.poke(0xa8, 9 << (4 * 2));
    rig.poke(c_par(3), periph as u32);
    rig.poke(c_mar(3), mem as u32);
    rig.poke(c_ndtr(3), 2);
    rig.poke(c_cr(3), C_CR_MINC | CR_EN);

    rig.dma.set_selected_request(2, 4, Level::High);
    assert_eq!(rig.dma.pump(8), 0, "CSELR selects 9, not 4");
    rig.dma.set_selected_request(2, 9, Level::High);
    assert_eq!(rig.dma.pump(8), 2);
}

#[test]
fn two_requests_in_one_table_cell_are_wired_or() {
    // RM0090 Table 43 puts TIM2_CH2 *and* TIM2_CH4 on DMA1 stream 6 channel 3:
    // one line into the stream, driven by two sources. A board writes two
    // `wire` statements into `req6c3`, and the second dropping must not cancel
    // the first holding.
    let dma = Dma::with_variant(Variant::Stream);
    let ids = WireIdAllocator::new();
    let (a, b) = (ids.alloc(), ids.alloc());
    let pin = Device::sink(&dma, "req6c3", &[a, b]).expect("a cell of Table 43");

    pin.sink.set_level(a, pin.line, Level::High);
    pin.sink.set_level(b, pin.line, Level::High);
    pin.sink.set_level(b, pin.line, Level::Low);
    assert!(
        dma.shared.held[6][4].load(Ordering::SeqCst),
        "the other driver is still holding the line"
    );
    pin.sink.set_level(a, pin.line, Level::Low);
    assert!(!dma.shared.held[6][4].load(Ordering::SeqCst));
}

// ---------------------------------------------------------------------------
// `PFCTRL`, peripheral flow control
// ---------------------------------------------------------------------------

/// A peripheral that is the flow controller: it has a fixed number of items to
/// give and says so on the last one, the way an SDIO block does once the card
/// has decided how long the transfer is.
#[derive(Debug)]
struct FlowController {
    left: Mutex<u32>,
}

impl DmaPeripheral for FlowController {
    fn dma_read(&self, _terminal: bool) -> u8 {
        unreachable!("the STM32 controller reads the bus at CPAR, not the peer")
    }
    fn dma_write(&self, _byte: u8, _terminal: bool) {
        unreachable!("the STM32 controller writes the bus at CPAR, not the peer")
    }
    fn dma_ready(&self) -> bool {
        *self.left.lock() > 0
    }
    fn dma_last(&self) -> bool {
        // Sampled once per beat, before the item moves.
        let mut left = self.left.lock();
        *left = left.saturating_sub(1);
        *left == 0
    }
}

/// Arm stream 0 peripheral-to-memory with `PFCTRL` and a flow-controlling peer.
fn arm_pfctrl(rig: &Rig, periph: u64, mem: u64, ndtr: u32, items: u32) -> Arc<FlowController> {
    let peer = Arc::new(FlowController {
        left: Mutex::with_rank(LockRank::DEVICE, items),
    });
    rig.dma
        .attach_dma_peripheral("req0c4", Arc::downgrade(&peer) as Weak<dyn DmaPeripheral>);
    arm_p2m(rig, periph, mem, ndtr, S_CR_PFCTRL);
    peer
}

#[test]
fn the_peripheral_ends_a_flow_controlled_transfer_before_ndtr_runs_out() {
    let rig = Rig::stream();
    let periph = RAM_BASE + 0x20;
    let mem = RAM_BASE + 0x100;
    rig.put(periph, Width::U8, 0x5a);
    // `NDTR` is a maximum (RM0090 §10.3.2), and the peripheral has three items.
    let peer = arm_pfctrl(&rig, periph, mem, 64, 3);

    assert_eq!(rig.dma.pump(64), 3, "the peer stopped asking after three");
    assert_eq!(*peer.left.lock(), 0);
    assert!(!rig.dma.is_running(0), "the peripheral ended the transfer");
    assert_eq!(rig.peek(s_cr(0)) & CR_EN, 0, "and EN came down with it");
    assert_ne!(
        rig.isr() & (1 << S_TCIF),
        0,
        "TCIF, at the peripheral's word"
    );
    assert_eq!(
        rig.dma.remaining(0),
        61,
        "NDTR keeps whatever the maximum had left"
    );
    for i in 0..3 {
        assert_eq!(rig.byte(mem + i), 0x5a);
    }
    assert_eq!(rig.byte(mem + 3), 0, "and nothing past the last item");
}

#[test]
fn a_flow_controlled_stream_still_stops_when_the_maximum_is_too_small() {
    let rig = Rig::stream();
    let periph = RAM_BASE + 0x20;
    let mem = RAM_BASE + 0x100;
    rig.put(periph, Width::U8, 0x5a);
    // The peripheral has eight items and firmware allowed two: RM0090 §10.5.6
    // says a stream whose `NDTR` is zero serves no transaction.
    let peer = arm_pfctrl(&rig, periph, mem, 2, 8);

    assert_eq!(rig.dma.pump(64), 2);
    assert!(!rig.dma.is_running(0));
    assert_eq!(rig.dma.remaining(0), 0, "and it did not reload");
    assert_eq!(*peer.left.lock(), 6, "the peripheral still had six to give");
}

#[test]
fn a_flow_controlled_stream_does_not_reload_in_circular_mode() {
    let rig = Rig::stream();
    let periph = RAM_BASE + 0x20;
    let mem = RAM_BASE + 0x100;
    rig.put(periph, Width::U8, 0x5a);
    // `CIRC` alongside `PFCTRL` is a combination RM0090 forbids; the flow
    // controller wins rather than the buffer wrapping for ever.
    let peer = Arc::new(FlowController {
        left: Mutex::with_rank(LockRank::DEVICE, 2),
    });
    rig.dma
        .attach_dma_peripheral("req0", Arc::downgrade(&peer) as Weak<dyn DmaPeripheral>);
    arm_p2m(&rig, periph, mem, 4, S_CR_PFCTRL | S_CR_CIRC);

    assert_eq!(rig.dma.pump(64), 2);
    assert!(!rig.dma.is_running(0));
    assert_eq!(rig.dma.remaining(0), 2, "stopped where the peripheral said");
}

#[test]
fn pfctrl_is_ignored_in_memory_to_memory_mode() {
    // RM0090 §10.5.5: with `DIR = 10` there is no peripheral on either port,
    // so hardware forces `PFCTRL` to zero and the count ends the transfer.
    let rig = Rig::stream();
    let src = RAM_BASE;
    let dst = RAM_BASE + 0x100;
    for i in 0..4u64 {
        rig.put(src + i, Width::U8, 0xa0 + i);
    }
    rig.poke(s_par(0), src as u32);
    rig.poke(s_m0ar(0), dst as u32);
    rig.poke(s_ndtr(0), 4);
    rig.poke(
        s_cr(0),
        (2 << 6) | S_CR_PINC | S_CR_MINC | S_CR_PFCTRL | CR_EN,
    );
    assert_eq!(rig.dma.pump(16), 4);
    assert_eq!(rig.dma.remaining(0), 0);
    assert_ne!(rig.isr() & (1 << S_TCIF), 0);
}

#[test]
fn a_selected_request_survives_save_and_load() {
    let saved = Rig::stream();
    let periph = RAM_BASE + 0x20;
    saved.put(periph, Width::U8, 0x5a);
    arm_p2m(&saved, periph, RAM_BASE + 0x100, 8, chsel(6));
    saved.dma.set_selected_request(0, 6, Level::High);
    assert_eq!(saved.dma.pump(3), 3);

    let mut shape = MachineShape::new();
    shape.add_device("dma", CLASS_NAME).unwrap();
    let mut w = StateWriter::new(shape);
    {
        let mut chunk = w.chunk("dma", CLASS_NAME, STATE_VERSION).unwrap();
        Device::save(&saved.dma, &mut chunk).unwrap();
    }
    let bytes = w.to_vec().unwrap();

    let restored = Rig::stream();
    restored.put(periph, Width::U8, 0x5a);
    let reader = StateReader::new(&bytes).unwrap();
    let chunk = reader
        .load("dma", CLASS_NAME, STATE_VERSION, &Migrations::new())
        .unwrap();
    Device::load(&restored.dma, &mut chunk.reader()).unwrap();

    // The latch travelled *in its slot*: the restored stream is still hearing
    // channel 6 and nothing else, so it finishes without a new wire event.
    assert_eq!(restored.dma.pump(5), 5);
    assert_eq!(restored.dma.remaining(0), 0);
    assert!(
        restored.dma.shared.held[0][7].load(Ordering::SeqCst),
        "slot 7 is channel 6"
    );
    assert!(!restored.dma.shared.held[0][0].load(Ordering::SeqCst));
}
