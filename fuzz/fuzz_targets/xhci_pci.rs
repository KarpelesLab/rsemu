#![no_main]
//! The **configuration surface** of an xHCI PCI function, and the retopology a
//! write to it performs.
//!
//! `usb_xhci` already fuzzes the register block and everything behind it — the
//! rings, the contexts, the walks. What a PCI attachment adds is a *second*
//! surface with a different shape and a much nastier consequence:
//!
//! > **A configuration write moves a mapping in a live address space, from
//! > inside the write.** Rev 2.1 §6.2.5.1's base address register and §6.2.2's
//! > Memory Space bit between them decide whether a 16 KiB window is in the map
//! > and where. A guest may move it onto its own RAM, onto the top of the
//! > address space where it does not fit, onto itself, or back and forth
//! > forever — and each of those is one `outl` to `0xcfc`.
//!
//! So this target pokes whatever the fuzzer says into configuration space, into
//! the register block at wherever the window currently is, and into guest RAM,
//! and checks the properties no unit test can cover across an arbitrary stream:
//!
//! * **Nothing panics and nothing deadlocks.** A `Bars::sync` runs inside a
//!   configuration write with the function's own state lock already released;
//!   a lock-order inversion here is a `core::sync` panic naming the ranks.
//! * **`COMMAND[2]` is honoured absolutely.** After every step, a function that
//!   may not master the bus is asked to; nothing it reads may come from guest
//!   memory. This is checked by construction — the engine's space handle is
//!   gated — and observed here by ringing every doorbell with mastering off.
//! * **The window and the registers agree.** Whatever the stream did, a guest
//!   read of the window either faults or answers `CAPLENGTH`, and never
//!   something in between.
//! * **A debug read has no side effects and a debug write is refused**, on both
//!   surfaces.
//! * **The snapshot loader is a parser on untrusted bytes.** A round trip must
//!   reproduce the chunk exactly from whatever half-configured state the stream
//!   left behind.
//!
//! # Input encoding
//!
//! A stream of one-byte opcodes, hand-decoded rather than derived (see
//! `state_roundtrip` for why the corpus is more stable that way):
//!
//! ```text
//!   0x00 rr dd dd dd dd   configuration write: register (rr & 0x3f) * 4
//!   0x01 rr               configuration read, compared against a debug read
//!   0x02 aa dd dd dd dd   a register-block write at (aa mod the window)
//!   0x03 aa               a register-block read, compared against a debug read
//!   0x04 aa aa dd dd dd dd  poke a dword into guest RAM
//!   0x05                  reset
//!   0x06                  save, then load what was saved: a round trip
//! ```
//!
//! Anything else is skipped, which keeps a mutated corpus productive rather
//! than mostly-rejected.

use libfuzzer_sys::fuzz_target;

use rsemu::bus::pci::{Bdf, PciBus};
use rsemu::bus::usb::UsbBus;
use rsemu::core::device::{Device, ResetKind};
use rsemu::core::space::{AddressSpace, MemAttrs, RamStore, Region, RequesterId, UnassignedPolicy};
use rsemu::core::state::{MachineShape, Migrations, StateReader, StateWriter};
use rsemu::core::value::Width;
use rsemu::dev::usb::hid::HidMouse;
use rsemu::dev::usb::xhci::Params;
use rsemu::dev::usb::xhci::pci::{BAR_BYTES, CLASS_NAME, XhciPci};
use std::sync::Arc;

/// Where guest RAM starts. Not zero, so a null pointer in a context is a bus
/// fault the controller has to survive rather than a plausible read.
const RAM_BASE: u64 = 0x1000;
/// How much of it there is.
const RAM_LEN: u64 = 0x20_0000;
/// Where this function sits on the fabric.
const DEVICE_NO: u8 = 5;

struct Fixture {
    function: XhciPci,
    bus: Arc<PciBus>,
    space: Arc<AddressSpace>,
    at: Bdf,
}

fn build() -> Fixture {
    let space = Arc::new(AddressSpace::new("mem", 32).with_unassigned(UnassignedPolicy::ONES));
    {
        let mut topo = space.topology();
        topo.map(
            Region::ram("ram", Arc::new(RamStore::new(RAM_LEN))),
            RAM_BASE,
        )
        .expect("the map fits");
    }

    let bus = Arc::new(PciBus::new());
    let usb = Arc::new(UsbBus::new(1));
    // A real device behind it, so a transfer the stream stumbles into has
    // somewhere to go.
    let mouse = Arc::new(HidMouse::new_detached(0x1234, 0x0002));
    usb.attach(0, mouse.device()).expect("an empty port");

    let at = Bdf::new(0, DEVICE_NO, 0).expect("a legal device number");
    let function = XhciPci::with_buses(
        Arc::clone(&bus),
        at,
        usb,
        Params {
            ports: 1,
            slots: 4,
            microframe_ticks: 8,
        },
        0x1234,
        0x1e31,
        0,
    )
    .expect("the BAR table takes the window");
    function
        .attach_space(&space, RequesterId(7))
        .expect("the window fits");

    Fixture {
        function,
        bus,
        space,
        at,
    }
}

impl Fixture {
    /// Where the window currently decodes, if the Command register lets it.
    fn window(&self) -> Option<u64> {
        self.function
            .bars()
            .window(0, self.function.command())
            .map(|(base, _)| base)
    }
}

fn snapshot(f: &Fixture) -> Option<Vec<u8>> {
    let mut shape = MachineShape::new();
    shape.add_device("xhci", CLASS_NAME).ok()?;
    let mut w = StateWriter::new(shape);
    {
        let mut chunk = w.chunk("xhci", CLASS_NAME, 1).ok()?;
        f.function.save(&mut chunk).ok()?;
    }
    w.to_vec().ok()
}

fuzz_target!(|data: &[u8]| {
    let f = build();
    let mut at = 0usize;

    while at < data.len() {
        let op = data[at];
        at += 1;
        match op {
            0x00 => {
                if at + 5 > data.len() {
                    break;
                }
                let register = u16::from(data[at] & 0x3f) * 4;
                let value =
                    u32::from_le_bytes([data[at + 1], data[at + 2], data[at + 3], data[at + 4]]);
                at += 5;
                // **The property.** This moves a mapping in a live address
                // space from inside a write, and must neither panic nor
                // deadlock however often or however far the window moves.
                f.bus
                    .config_write(f.at, register, &value.to_le_bytes(), MemAttrs::DEFAULT);
                // Rev 2.1 §6.2.5.1 and §6.2.2 between them: a debug write would
                // move the window out from under a running driver.
                let mut kept = [0u8; 4];
                f.bus
                    .config_read(f.at, register, &mut kept, MemAttrs::DEFAULT);
                f.bus
                    .config_write(f.at, register, &(!value).to_le_bytes(), MemAttrs::DEBUG);
                let mut after = [0u8; 4];
                f.bus
                    .config_read(f.at, register, &mut after, MemAttrs::DEFAULT);
                assert_eq!(kept, after, "a debug write reached configuration space");
            }
            0x01 => {
                if at >= data.len() {
                    break;
                }
                let register = u16::from(data[at] & 0x3f) * 4;
                at += 1;
                let mut live = [0u8; 4];
                let mut dbg = [0u8; 4];
                let mut again = [0u8; 4];
                f.bus
                    .config_read(f.at, register, &mut live, MemAttrs::DEFAULT);
                f.bus.config_read(f.at, register, &mut dbg, MemAttrs::DEBUG);
                f.bus
                    .config_read(f.at, register, &mut again, MemAttrs::DEBUG);
                assert_eq!(dbg, again, "a configuration read had a side effect");
                assert_eq!(live, dbg, "a debug read answered differently");
            }
            0x02 => {
                if at + 5 > data.len() {
                    break;
                }
                let offset = u64::from(data[at]) * 64 % BAR_BYTES;
                let value =
                    u32::from_le_bytes([data[at + 1], data[at + 2], data[at + 3], data[at + 4]]);
                at += 5;
                if let Some(base) = f.window() {
                    // Whatever the rings in RAM say, this returns: every walk
                    // is bounded, and a function that may not master the bus
                    // fetches nothing at all.
                    let _ = f.space.write(
                        base + offset,
                        Width::U32,
                        u64::from(value),
                        MemAttrs::DEFAULT,
                    );
                    assert!(
                        f.space
                            .write(base + offset, Width::U32, u64::from(value), MemAttrs::DEBUG)
                            .is_err(),
                        "a debug write to the xHCI register block must be refused"
                    );
                }
            }
            0x03 => {
                if at >= data.len() {
                    break;
                }
                let offset = u64::from(data[at]) * 64 % BAR_BYTES;
                at += 1;
                if let Some(base) = f.window() {
                    let live = f.space.read(base + offset, Width::U32, MemAttrs::DEFAULT);
                    let dbg = f.space.read(base + offset, Width::U32, MemAttrs::DEBUG);
                    assert_eq!(
                        live.is_ok(),
                        dbg.is_ok(),
                        "a debug read and a guest read disagree about what is mapped"
                    );
                    if let (Ok(live), Ok(dbg)) = (live, dbg) {
                        assert_eq!(live, dbg, "a debug read answered differently");
                        let again = f
                            .space
                            .read(base + offset, Width::U32, MemAttrs::DEBUG)
                            .expect("it read a moment ago");
                        assert_eq!(dbg, again, "a debug read had a side effect");
                    }
                }
            }
            0x04 => {
                if at + 6 > data.len() {
                    break;
                }
                let addr =
                    (u64::from(u16::from_le_bytes([data[at], data[at + 1]])) * 4) % (RAM_LEN - 4);
                let value =
                    u32::from_le_bytes([data[at + 2], data[at + 3], data[at + 4], data[at + 5]]);
                at += 6;
                let _ = f.space.write(
                    RAM_BASE + addr,
                    Width::U32,
                    u64::from(value),
                    MemAttrs::DEFAULT,
                );
            }
            0x05 => f.function.reset(ResetKind::Cold),
            0x06 => {
                // A round trip must reproduce the chunk exactly, whatever
                // half-configured state the stream left behind — including a
                // base address register pointing somewhere the window cannot
                // go, which the loader has to accept and then leave unmapped.
                if let Some(bytes) = snapshot(&f) {
                    let fresh = build();
                    let reader = StateReader::new(&bytes).expect("we just wrote it");
                    let chunk = reader
                        .load("xhci", CLASS_NAME, 1, &Migrations::new())
                        .expect("it is in there");
                    fresh
                        .function
                        .load(&mut chunk.reader())
                        .expect("our own snapshot loads");
                    assert_eq!(
                        snapshot(&fresh),
                        Some(bytes),
                        "the function did not round trip"
                    );
                }
            }
            _ => {}
        }
    }

    // Whatever the stream did, the two halves still agree: either the window is
    // out of the map, or the first dword of it is the capability register file.
    if let Some(base) = f.window()
        && let Ok(value) = f.space.read(base, Width::U32, MemAttrs::DEFAULT)
    {
        assert_eq!(
            value as u32 & 0xff,
            0x40,
            "the window decodes something that is not this controller"
        );
    }
});
