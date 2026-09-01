//! A smoke test that runs a **real PC firmware image** on this core.
//!
//! Hand-written tests check the instructions this core's author thought to
//! check. A firmware image checks the ones the firmware's authors thought to
//! use, which is a different and much larger set: it is the first thing that
//! exercises reset from `ffff:fff0`, sixteen-bit real mode at volume, the
//! descriptor-table bring-up, the switch to protected mode, and then hundreds
//! of millions of 32-bit instructions in a row.
//!
//! # Running it
//!
//! Gated on an environment variable naming an image, exactly as the corpus
//! runner is, so `cargo test` stays hermetic and needs nothing installed:
//!
//! ```text
//! RSEMU_BIOS=/usr/share/qemu/bios.bin cargo test --release --all-features x86::firmware -- --nocapture
//! ```
//!
//! Any 128 KiB legacy PC BIOS works. **Nothing is vendored**: the image is
//! read from wherever the variable points and never enters this repository.
//! Running a program — including a copyleft one — is ordinary use and creates
//! no derivative work (`ROADMAP.md` §1); reading its source would be a
//! different matter, and was not done.
//!
//! # What it can and cannot show
//!
//! There is **no chipset here**: no interrupt controller, no timer, no
//! real-time clock, no DMA, no PCI. Memory is flat RAM and every I/O port
//! reads as ones. So this cannot show a firmware image *booting* — it will
//! spin the moment it waits for a timer tick, which is the correct thing for
//! it to do on a machine with no timer. What it does show is that the
//! processor runs the firmware's code correctly for as long as the firmware
//! does not need a device, and it prints the ports the image touched, which is
//! the list of chips a machine has to supply before there is anything more to
//! see.

use alloc::sync::Arc;
use alloc::vec::Vec;

use crate::core::space::{
    AccessConstraints, AddressSpace, MemAttrs, MemOps, MemResult, RamStore, Region,
};
use crate::core::sync::{self, LockRank};

use super::{Config, Variant, X86};

/// How many instructions to run before giving up, unless `RSEMU_BIOS_STEPS`
/// says otherwise.
const DEFAULT_STEPS: usize = 20_000_000;

/// An I/O space that reads as ones and records every port anything touched.
///
/// An unterminated ISA bus reads as ones, so this is what a machine with no
/// chips in it actually does — and the log is the useful output: it says
/// which devices the image reached for, in the order it reached for them.
#[derive(Debug)]
struct PortLog {
    seen: sync::Mutex<Vec<u16>>,
}

impl Default for PortLog {
    fn default() -> PortLog {
        PortLog {
            seen: sync::Mutex::with_rank(LockRank::DEVICE, Vec::new()),
        }
    }
}

impl MemOps for PortLog {
    fn read(&self, offset: u64, dst: &mut [u8], _: MemAttrs) -> MemResult {
        self.seen.lock().push(offset as u16);
        dst.fill(0xff);
        Ok(())
    }

    fn write(&self, offset: u64, _: &[u8], _: MemAttrs) -> MemResult {
        self.seen.lock().push(offset as u16);
        Ok(())
    }

    fn constraints(&self) -> AccessConstraints {
        AccessConstraints::ANY
    }
}

#[test]
fn a_pc_firmware_image_reaches_protected_mode_and_keeps_running() {
    run_firmware(Variant::I80486);
}

/// The same image on an x86-64 part.
///
/// Long mode changes what `CPUID` reports, adds `CR4` and the model-specific
/// registers, and widens every register in the file — so a firmware image that
/// still runs unchanged is evidence that the widening did not disturb the
/// 16- and 32-bit paths it spends all its time in. SeaBIOS probes `CPUID`
/// early and takes different branches on what it finds, so this is not the
/// same run with a different label on it.
#[test]
fn the_same_image_runs_on_a_sixty_four_bit_part() {
    run_firmware(Variant::X86_64);
}

fn run_firmware(variant: Variant) {
    let Ok(path) = std::env::var("RSEMU_BIOS") else {
        println!(
            "firmware: set RSEMU_BIOS to a legacy PC BIOS image to run it on this \
             core; see the module docs"
        );
        return;
    };
    let image = std::fs::read(&path).unwrap_or_else(|e| panic!("{path}: {e}"));
    let size = image.len() as u64;
    assert!(
        size.is_power_of_two() && (0x1_0000..=0x10_0000).contains(&size),
        "{path}: {size} bytes is not a plausible BIOS image"
    );

    // 16 MiB of RAM at zero, the image at the very top of the 32-bit address
    // space — where the reset vector points — and its last 128 KiB shadowed
    // into `0xe0000`, which is where a PC's chipset aliases it so that real
    // mode can reach it at all.
    let ram = Arc::new(RamStore::new(0x100_0000));
    let rom = Arc::new(RamStore::new(size));
    for (i, byte) in image.iter().enumerate() {
        let i = i as u64;
        rom.write_u8(i, *byte).unwrap();
        if i >= size - 0x2_0000 {
            ram.write_u8(0xe_0000 + (i - (size - 0x2_0000)), *byte)
                .unwrap();
        }
    }
    let mem = AddressSpace::new("mem", 32);
    mem.topology()
        .map(Region::ram("ram", ram), 0)
        .expect("16 MiB at zero");
    mem.topology()
        .map(Region::ram("rom", rom), 0x1_0000_0000u64 - size)
        .expect("the image at the top of the space");

    let ports = Arc::new(PortLog::default());
    let io = AddressSpace::new("io", 16);
    io.topology()
        .map(Region::io("ports", 0x1_0000, ports.clone()), 0)
        .expect("64 KiB fits in 16 bits");

    let cpu = X86::new(Config::default().with_variant(variant));
    cpu.attach_space(Arc::new(mem));
    cpu.attach_io_space(Arc::new(io));

    let limit: usize = std::env::var("RSEMU_BIOS_STEPS")
        .ok()
        .and_then(|v| v.parse().ok())
        .unwrap_or(DEFAULT_STEPS);
    let mut protected_after = None;
    let mut executed = 0usize;
    // Whether a descriptor table was ever loaded, not whether one is loaded at
    // the end. A PC firmware goes in and out of protected mode many times and
    // leaves real mode behind it, so the final `GDTR` is routinely the reset
    // value — asserting on it was asserting that the run stopped in the middle
    // of a protected-mode stretch, which is not a property of the processor.
    let mut gdt_loaded = false;
    for step in 0..limit {
        if cpu.step() == 0 {
            break;
        }
        executed += 1;
        if protected_after.is_none() && cpu.sys().protected() {
            protected_after = Some(step);
        }
        gdt_loaded |= cpu.sys().gdtr.limit > 0;
    }

    let regs = cpu.regs();
    let sys = cpu.sys();
    println!(
        "firmware[{variant}]: {executed} instructions; protected mode after {:?}; \
         stopped at {:04x}:{:08x}",
        protected_after, regs.cs, regs.rip
    );
    println!("firmware: {regs}");
    println!(
        "firmware: cr0={:08x} gdtr={:08x}+{:x} idtr={:08x}+{:x}",
        sys.cr0, sys.gdtr.base, sys.gdtr.limit, sys.idtr.base, sys.idtr.limit
    );
    let (faults, last) = cpu.bus_faults();
    println!("firmware: {faults} unanswered bus access(es), last at {last:08x}");
    let mut seen = ports.seen.lock().clone();
    seen.sort_unstable();
    seen.dedup();
    println!("firmware: I/O ports touched: {seen:04x?}");

    // The gate. Reaching protected mode means reset, the far jump out of the
    // top of the address space, real-mode execution, `LGDT` and `MOV CR0` all
    // worked; still running means nothing since then has faulted into a
    // handler that does not exist, and the processor has not shut down.
    assert!(
        protected_after.is_some(),
        "the image never set CR0.PE — see the trace above"
    );
    assert!(gdt_loaded, "no descriptor table was ever loaded");
    assert_eq!(
        executed, limit,
        "the core stopped early: halted or shut down"
    );
    assert!(
        seen.contains(&0x0070) && seen.contains(&0x0043),
        "a PC firmware image should have reached for the RTC and the timer"
    );
}
