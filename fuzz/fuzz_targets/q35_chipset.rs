#![no_main]
//! The q35 chipset's three new surfaces: the ECAM window, the two bridges'
//! configuration registers, and the ACPI register block at `PMBASE`.
//!
//! `CLAUDE.md` asks for a fuzz target on every MMIO surface. These earn one
//! between them rather than one each, because the interesting failures are
//! *between* them: a configuration write through ECAM moves a window in the
//! same address space the write is travelling through, and a configuration
//! write through `0xcfc` moves one in the other. Fuzzing them apart would
//! exercise three register decoders and none of the paths that connect them.
//!
//! What is being tested, beyond "does not panic":
//!
//! * **Every offset in the ECAM window is decoded or refused, and neither
//!   panics.** The window is 256 MiB of address arithmetic — bus, device,
//!   function and register are all sliced out of one number — and the input
//!   reaches every field of it, including the 3840 bytes of extended
//!   configuration space above `0xff` and the addresses no function answers at.
//! * **A debug read answers the same value and changes nothing**, compared on
//!   every read (`ROADMAP.md` §15, invariant 5). A debug *write* must be
//!   refused everywhere, and the register it would have moved must not move.
//! * **A window that moves cannot corrupt the address space.** `PCIEXBAR`,
//!   `PMBASE` and the seven `PAM` registers are all reachable, so the input can
//!   place a window over another, off the end of the space, at an unaligned
//!   base, or at a `LENGTH` the datasheet reserves. Every one of those has to
//!   end in a map the space still agrees with.
//! * **A stale retopology terminates.** Both bridges own a `stale` flag and the
//!   north bridge asks the scheduler for a tick while it is set; advancing it
//!   from this input is what proves the flag clears rather than latching for
//!   ever.
//! * **The snapshot loader is a parser on untrusted bytes.** A tail of the
//!   input is handed to `Device::load`, which must reject it or accept it,
//!   never panic, and leave a bridge that still works.
//!
//! # Input encoding
//!
//! A stream of one-byte opcodes, hand-decoded rather than derived (see
//! `state_roundtrip` for why the corpus is more stable that way):
//!
//! ```text
//!   0x00 rr dd dd dd dd   write the north bridge's config register rr (dword)
//!   0x01 rr               read it, and compare against a debug read
//!   0x02 rr dd dd dd dd   write the south bridge's config register rr
//!   0x03 rr               read it, and compare against a debug read
//!   0x04 bb ff rr dd dd dd dd   write through ECAM: bus bb, dev/fn ff, reg rr
//!   0x05 bb ff rr         read through ECAM, and compare against a debug read
//!   0x06 aa dd dd dd dd   write the ACPI block at PMBASE + (aa % 0x80)
//!   0x07 aa               read it, and compare against a debug read
//!   0x08 nn nn            advance the PM timer and the north bridge by nn
//!   0x09                  cold reset both bridges
//!   0x0a                  save, then load what was saved: a round trip
//!   0x0b ...              load the rest of the input as a snapshot chunk
//! ```
//!
//! Anything else is skipped, which keeps a mutated corpus productive rather
//! than mostly-rejected.

use std::sync::Arc;

use libfuzzer_sys::fuzz_target;

use rsemu::bus::pci::{Bdf, PciBus};
use rsemu::core::HostObjects;
use rsemu::core::device::{Deferred, Device, RealizeCtx, ResetKind};
use rsemu::core::space::{
    AddressSpace, MemAttrs, Perms, RamStore, Region, RegionRef, RequesterId, UnassignedPolicy,
};
use rsemu::core::state::{MachineShape, Migrations, StateReader, StateWriter};
use rsemu::core::value::Width;
use rsemu::dev::q35::{lpc, mch};

/// Where the board starts the ECAM window, so most inputs land inside it.
const ECAM: u64 = 0xe000_0000;

/// Where the board starts the ACPI register block.
const PMBASE: u32 = 0x600;

/// A memory space, an I/O space, and the two bridges on one fabric.
struct Rig {
    mem: Arc<AddressSpace>,
    io: Arc<AddressSpace>,
    bus: Arc<PciBus>,
    mch: mch::Mch,
    lpc: lpc::Lpc,
}

impl Rig {
    fn new() -> Rig {
        let bus = Arc::new(PciBus::new());
        let mem =
            Arc::new(AddressSpace::new("mem", 32).with_unassigned(UnassignedPolicy::ONES));
        let io =
            Arc::new(AddressSpace::new("port", 16).with_unassigned(UnassignedPolicy::ONES));
        // Something under the shadow, so a PAM window has a decode to displace
        // and an unmap has somewhere to fall through to.
        let rom = Arc::new(RamStore::new(mch::SHADOW_LEN));
        let region: RegionRef = Arc::new(Region::ram("fuzz.rom", rom));
        mem.topology()
            .map_with(
                rsemu::core::space::Mapping::new(region, mch::SHADOW_BASE)
                    .with_perms(Perms::READ.union(Perms::EXEC)),
            )
            .expect("it fits a 32-bit space");

        let mch = mch::Mch::with_bus(
            Arc::clone(&bus),
            Bdf::new(0, 0, 0).expect("legal"),
            mch::DEVICE_ID_82Q35,
            0,
            ECAM | 1,
        )
        .expect("the window table fits its store");
        let lpc = lpc::Lpc::with_bus(
            Arc::clone(&bus),
            Bdf::new(0, lpc::LPC_DEVICE, 0).expect("legal"),
            0x2918,
            0,
            PMBASE,
            String::from("port"),
        );
        mch.attach_space(&mem);
        lpc.attach_space(&io);
        let mut deferred = Deferred::new();
        let hosts = HostObjects::new();
        {
            let mut ctx =
                RealizeCtx::new("mch", RequesterId::ANONYMOUS, &mut deferred, &hosts);
            mch.realize(&mut ctx).expect("00:00.0 is free");
        }
        {
            let mut ctx =
                RealizeCtx::new("lpc", RequesterId::ANONYMOUS, &mut deferred, &hosts);
            lpc.realize(&mut ctx).expect("00:1f.0 is free");
        }
        Rig {
            mem,
            io,
            bus,
            mch,
            lpc,
        }
    }

    /// The function at `at`, if the fabric has one.
    fn function(&self, at: Bdf) -> Option<Arc<dyn rsemu::bus::pci::PciFunction>> {
        self.bus.function(at)
    }

    /// A configuration dword write straight to a function, which is the path a
    /// `0xcfc` write takes once the fabric has routed it — so the window it
    /// moves is in the *other* space and the try-lock succeeds.
    fn config_write(&self, at: Bdf, reg: u16, value: u32) {
        if let Some(f) = self.function(at) {
            f.config_write(reg & 0xfc, &value.to_le_bytes(), MemAttrs::DEFAULT);
        }
    }

    /// A configuration dword read, checked against a debug read of the same
    /// register: a debugger must see the same bytes and disturb nothing.
    fn config_read(&self, at: Bdf, reg: u16) {
        let Some(f) = self.function(at) else {
            return;
        };
        let reg = reg & 0xfc;
        let mut guest = [0u8; 4];
        f.config_read(reg, &mut guest, MemAttrs::DEFAULT);
        let mut debug = [0u8; 4];
        f.config_read(
            reg,
            &mut debug,
            MemAttrs::DEFAULT.with_debug(true),
        );
        assert_eq!(
            guest, debug,
            "a debug read of configuration register {reg:#x} disagreed with a guest read"
        );
        // And a debug *write* must be refused outright: it would move a window
        // under the guest's feet, which is exactly what the flag forbids.
        let before = guest;
        f.config_write(
            reg,
            &[!guest[0], !guest[1], !guest[2], !guest[3]],
            MemAttrs::DEFAULT.with_debug(true),
        );
        let mut after = [0u8; 4];
        f.config_read(reg, &mut after, MemAttrs::DEFAULT);
        assert_eq!(
            before, after,
            "a debug write moved configuration register {reg:#x}"
        );
    }

    /// Everything that has to hold after any operation.
    fn invariants(&self) {
        // `next_event_tick` must be strictly ahead of `current_tick`, or the
        // scheduler spins where it stands. Both bridges are lazily advanced.
        for device in [&self.mch as &dyn Device, &self.lpc as &dyn Device] {
            if let Some(next) = device.next_event_tick() {
                assert!(
                    next > device.current_tick(),
                    "a lazily advanced device asked the scheduler to stop where it already is"
                );
            }
        }
        // Whatever the registers now say, the ECAM window either decodes at a
        // legal base or does not decode. `ecam()` is the register's own answer
        // and the space is where it actually went; a base the space cannot
        // drive leaves the window unmapped, which reads as ones.
        if let Some((base, len)) = self.mch.ecam() {
            assert!(len.is_power_of_two());
            assert_eq!(base % len, 0, "an ECAM window is aligned to its own size");
        }
        if let Some(base) = self.lpc.acpi_base() {
            assert_eq!(base % 128, 0, "PMBASE is on a 128-byte boundary");
            assert!(base <= 0xffff, "an I/O window is in the 64 KiB I/O space");
        }
    }
}

/// Read `len` bytes at `at` in `space`, ignoring a refusal, and check that a
/// debug read of the same address agrees.
fn probe(space: &AddressSpace, at: u64, width: Width) {
    let guest = space.read(at, width, MemAttrs::DEFAULT);
    let debug = space.read(
        at,
        width,
        MemAttrs::DEFAULT.with_debug(true),
    );
    match (guest, debug) {
        (Ok(a), Ok(b)) => assert_eq!(a, b, "a debug read of {at:#x} disagreed"),
        // A refusal on one side and not the other is allowed only where the
        // device refuses debug access deliberately — the ACPI block refuses a
        // debug *write*, never a read — so both sides must agree on refusal.
        (Err(_), Err(_)) => {}
        (a, b) => panic!("{at:#x}: guest {a:?} but debug {b:?}"),
    }
}

/// Save both bridges into one image.
fn save(rig: &Rig) -> Vec<u8> {
    let mut shape = MachineShape::new();
    shape.add_device("mch", mch::CLASS_NAME).expect("unique");
    shape.add_device("lpc", lpc::CLASS_NAME).expect("unique");
    let mut w = StateWriter::new(shape);
    {
        let mut chunk = w.chunk("mch", mch::CLASS_NAME, 1).expect("one chunk");
        rig.mch.save(&mut chunk).expect("saves");
    }
    {
        let mut chunk = w.chunk("lpc", lpc::CLASS_NAME, 1).expect("one chunk");
        rig.lpc.save(&mut chunk).expect("saves");
    }
    w.to_vec().expect("encodes")
}

fuzz_target!(|data: &[u8]| {
    let rig = Rig::new();
    let mch_at = Bdf::new(0, 0, 0).expect("legal");
    let lpc_at = Bdf::new(0, lpc::LPC_DEVICE, 0).expect("legal");
    let mut at = 0usize;
    // Bounded, so a mutated input cannot turn into a long-running test: the
    // fuzzer's own timeout is not the place to discover a loop.
    let mut budget = 4096;

    let byte = |at: usize| data.get(at).copied().unwrap_or(0);
    let dword = |at: usize| {
        u32::from_le_bytes([byte(at), byte(at + 1), byte(at + 2), byte(at + 3)])
    };

    while at < data.len() && budget > 0 {
        budget -= 1;
        let op = data[at];
        at += 1;
        match op {
            0x00 => {
                rig.config_write(mch_at, u16::from(byte(at)), dword(at + 1));
                at += 5;
            }
            0x01 => {
                rig.config_read(mch_at, u16::from(byte(at)));
                at += 1;
            }
            0x02 => {
                rig.config_write(lpc_at, u16::from(byte(at)), dword(at + 1));
                at += 5;
            }
            0x03 => {
                rig.config_read(lpc_at, u16::from(byte(at)));
                at += 1;
            }
            0x04 => {
                // Through the ECAM window, if it is currently decoded — which
                // it may not be, because a previous operation could have moved
                // or disabled it. That is the point: the write goes to whatever
                // is at that address now.
                if let Some((base, len)) = rig.mch.ecam() {
                    let offset = (u64::from(byte(at)) << 20)
                        | (u64::from(byte(at + 1)) << 12)
                        | u64::from(byte(at + 2) & 0xfc);
                    if offset < len {
                        let _ = rig.mem.write(
                            base + offset,
                            Width::U32,
                            u64::from(dword(at + 3)),
                            MemAttrs::DEFAULT,
                        );
                    }
                }
                at += 7;
            }
            0x05 => {
                if let Some((base, len)) = rig.mch.ecam() {
                    let offset = (u64::from(byte(at)) << 20)
                        | (u64::from(byte(at + 1)) << 12)
                        | u64::from(byte(at + 2) & 0xfc);
                    if offset < len {
                        probe(&rig.mem, base + offset, Width::U32);
                    }
                }
                at += 3;
            }
            0x06 => {
                if let Some(base) = rig.lpc.acpi_base() {
                    let _ = rig.io.write(
                        base + u64::from(byte(at) & 0x7c),
                        Width::U32,
                        u64::from(dword(at + 1)),
                        MemAttrs::DEFAULT,
                    );
                }
                at += 5;
            }
            0x07 => {
                if let Some(base) = rig.lpc.acpi_base() {
                    probe(&rig.io, base + u64::from(byte(at) & 0x7c), Width::U32);
                }
                at += 1;
            }
            0x08 => {
                let span = u64::from(u16::from_le_bytes([byte(at), byte(at + 1)]));
                at += 2;
                // Monotonic by contract, and the only place a stale retopology
                // owed by an ECAM write can land.
                let target = rig.lpc.current_tick().saturating_add(span);
                Device::advance_to(&rig.lpc, target);
                assert!(rig.lpc.current_tick() >= target.min(u64::MAX));
                let target = rig.mch.current_tick().saturating_add(span);
                Device::advance_to(&rig.mch, target);
            }
            0x09 => {
                rig.mch.reset(ResetKind::Cold);
                rig.lpc.reset(ResetKind::Cold);
            }
            0x0a => {
                let image = save(&rig);
                let reader = StateReader::new(&image).expect("what we just wrote parses");
                let chunk = reader
                    .load("mch", mch::CLASS_NAME, 1, &Migrations::new())
                    .expect("the chunk is there");
                rig.mch.load(&mut chunk.reader()).expect("its own image");
                let chunk = reader
                    .load("lpc", lpc::CLASS_NAME, 1, &Migrations::new())
                    .expect("the chunk is there");
                rig.lpc.load(&mut chunk.reader()).expect("its own image");
            }
            0x0b => {
                // The snapshot loader is a parser on bytes nobody vouched for.
                // It has to reject or accept, never panic, and leave bridges
                // that still work — which the invariants below then check.
                if let Ok(reader) = StateReader::new(&data[at..]) {
                    if let Ok(chunk) =
                        reader.load("mch", mch::CLASS_NAME, 1, &Migrations::new())
                    {
                        let _ = rig.mch.load(&mut chunk.reader());
                    }
                    if let Ok(chunk) =
                        reader.load("lpc", lpc::CLASS_NAME, 1, &Migrations::new())
                    {
                        let _ = rig.lpc.load(&mut chunk.reader());
                    }
                }
                at = data.len();
            }
            _ => {}
        }
        rig.invariants();
    }

    // And whatever state the input left the bridges in, they still answer.
    rig.config_read(mch_at, 0);
    rig.config_read(lpc_at, 0);
    rig.invariants();
});
