//! The `mac-plus` board itself, with no ROM but the two reset longwords the
//! catalog supplies: that it assembles, that every chip answers where the
//! Guide puts it, and that the overlay really is the VIA's `PA4`.
//!
//! This is the board test that needs nothing of anybody's: `tests/mac_plus.rs`
//! is where a real Apple ROM runs, and it skips when the user has not said
//! where theirs is. Nothing here reads a file.

#![cfg(feature = "machine-mac-plus")]

use std::sync::Arc;

use rsemu::core::Captured;
use rsemu::core::clock::GlobalTime;
use rsemu::core::space::{AddressSpace, MemAttrs};
use rsemu::core::value::Width;
use rsemu::cpu::m68k::M68k;
use rsemu::machine::{Machine, catalog};

/// The VIA's published base. Its sixteen registers are 512 bytes apart.
const VIA: u64 = 0xef_e1fe;
/// The IWM's, which is odd because it sits on the other byte lane.
const IWM: u64 = 0xdf_e1ff;
/// Channel B's control address in the SCC's read window.
const SCC_READ: u64 = 0x9f_fff8;

/// The whole of the ROM this file uses: the two longwords a 68000 fetches out
/// of reset — a stack pointer at the top of the default megabyte and a program
/// counter at `$000008` — and `BRA .`, the two-byte branch to itself.
///
/// rsemu's own bytes, not anybody's ROM. It is the same image the catalog
/// binds when nothing else is given.
const STUB_ROM: [u8; 10] = [
    0x00, 0x10, 0x00, 0x00, // SSP = $00100000
    0x00, 0x00, 0x00, 0x08, // PC  = $00000008
    0x60, 0xfe, // BRA .
];

fn build(params: &[(&str, &str)]) -> (Machine, Arc<M68k>) {
    let cores: Arc<Captured<M68k>> = Arc::new(Captured::new());
    let kept = Arc::clone(&cores);
    let mut options = catalog::build_options().expect("the catalog agrees with itself");
    options.bindings.replace("cpu.m68k", move |props| {
        let cpu = Arc::new(M68k::from_props(props)?);
        kept.push(&cpu);
        Ok(cpu)
    });
    for &(name, value) in params {
        options
            .resolve
            .params
            .push((name.to_string(), value.to_string()));
    }
    options.realize.media.insert("macrom", STUB_ROM.to_vec());
    // The drive's bay, empty: `machine::realize` refuses a slot that is named
    // and unbound, so a machine with nothing in the drive binds zero bytes for
    // it — which is what `rsemu run` does when nobody says `--floppy`.
    options.realize.media.insert("floppy", Vec::new());
    // And the SCSI cable at address 0, empty for the same reason: no bytes is
    // an address nobody answers at, which is a Plus with nothing plugged into
    // the port on the back.
    options.realize.media.insert("hd0", Vec::new());
    let registry = catalog::registry().expect("a registry");
    let source = catalog::machine("mac-plus")
        .expect("this build ships mac-plus")
        .source;
    let machine = rsemu::machine::build("mac-plus", source, &registry, &options)
        .unwrap_or_else(|e| panic!("the board does not realize: {e}"));
    let cpu = cores.last().expect("the binding captured the processor");
    (machine, cpu)
}

fn space(machine: &Machine) -> &Arc<AddressSpace> {
    machine.space("mem").expect("the board has `mem`")
}

fn read8(machine: &Machine, addr: u64) -> u8 {
    space(machine)
        .read(addr, Width::U8, MemAttrs::DEFAULT)
        .unwrap_or_else(|e| panic!("{addr:#08x}: {e:?}")) as u8
}

fn write8(machine: &Machine, addr: u64, value: u64) {
    space(machine)
        .write(addr, Width::U8, value, MemAttrs::DEFAULT)
        .unwrap_or_else(|e| panic!("{addr:#08x}: {e:?}"));
}

/// The board assembles and the processor fetches out of the overlaid ROM: the
/// catalog's default image is the two reset longwords and a branch to itself,
/// so a running board sits at `$000008` with no fault.
#[test]
fn the_board_realizes_and_the_processor_fetches_through_the_overlay() {
    let (mut machine, cpu) = build(&[]);
    machine
        .run_for(GlobalTime::from_nanos(1_000_000))
        .expect("it runs");
    let regs = cpu.regs();
    assert_eq!(regs.pc, 0x0000_0008, "the branch to itself");
    assert_eq!(regs.a[7], 0x0010_0000, "the stack pointer the image gives");
    assert_eq!(cpu.bus_faults().0, 0);
    assert!(!cpu.is_halted());
}

/// The ROM answers at zero *and* at `$400000`, which is the only way a
/// Macintosh starts: the reset vector points into `$40xxxx`, and the same part
/// has to answer there once the overlay is gone.
#[test]
fn the_rom_answers_at_zero_and_at_its_own_address() {
    let (machine, _cpu) = build(&[]);
    for base in [0x00_0000u64, 0x40_0000] {
        assert_eq!(read8(&machine, base), 0x00);
        assert_eq!(read8(&machine, base + 1), 0x10, "SSP = $00100000");
        assert_eq!(read8(&machine, base + 7), 0x08, "PC = $00000008");
    }
    // And a 128 KiB part repeats through the megabyte its select decodes.
    assert_eq!(read8(&machine, 0x42_0001), 0x10);
}

/// The VIA's `PA4` is the overlay, and it is asserted out of reset because
/// port A is all inputs and the pin is pulled up. Making it an output and
/// writing a zero swaps memory in underneath — which is what every Macintosh
/// ROM does once it has sized memory.
#[test]
fn writing_the_vias_port_a_clears_the_overlay() {
    let (machine, _cpu) = build(&[]);
    assert_eq!(read8(&machine, 0), 0x00, "the ROM's first byte");
    assert_eq!(read8(&machine, 1), 0x10);

    // DDRA is register 3 and `vBufA` — port A with no handshake — is 15.
    write8(&machine, VIA + 3 * 0x200, 0x7f);
    write8(&machine, VIA + 15 * 0x200, 0x00);
    // Memory is at zero now, and it is blank.
    assert_eq!(read8(&machine, 1), 0x00, "memory, not the ROM");
    // And the `$600000` window, which was memory while the overlay was up, is
    // not decoded at all any more.
    write8(&machine, VIA + 15 * 0x200, 0x10);
    assert_eq!(read8(&machine, 1), 0x10, "the overlay is back");
}

/// The VIA is decoded on A9-A12, so its registers are 512 bytes apart and the
/// block repeats every 8 KiB through `$E80000`-`$EFFFFF`.
#[test]
fn the_via_answers_where_the_guide_puts_it() {
    let (machine, _cpu) = build(&[]);
    // A value with no read-only bits in it, through DDRB.
    write8(&machine, VIA + 2 * 0x200, 0xa5);
    assert_eq!(read8(&machine, VIA + 2 * 0x200), 0xa5);
    // The same chip at the bottom of the window and at the top.
    assert_eq!(read8(&machine, 0xe8_0000 + 2 * 0x200), 0xa5);
    assert_eq!(read8(&machine, 0xef_e000 + 2 * 0x200), 0xa5);
    // A different register is a different byte.
    assert_ne!(read8(&machine, VIA + 3 * 0x200), 0xa5);
}

/// The IWM's sixteen soft switches are on the same decode, and **a read moves
/// one**: reading switch 9 turns the drive enable on, and the status register
/// says so.
#[test]
fn the_iwm_answers_where_the_guide_puts_it_and_every_read_is_a_switch() {
    let (machine, _cpu) = build(&[]);
    let _ = read8(&machine, IWM + 13 * 0x200); // Q6 on
    let _ = read8(&machine, IWM + 14 * 0x200); // Q7 off: the status register
    let before = read8(&machine, IWM + 14 * 0x200);
    assert_eq!(before & 0x20, 0, "the drive enable is off");
    let _ = read8(&machine, IWM + 9 * 0x200); // the motor
    let after = read8(&machine, IWM + 14 * 0x200);
    assert_eq!(after & 0x20, 0x20, "and a read turned it on");
    // The window repeats every 8 KiB through $C00000-$DFFFFF.
    assert_eq!(read8(&machine, 0xc0_01ff + 14 * 0x200), after);
}

/// The SCC's two windows: `A1` picks the channel, `A2` picks data over
/// control, and the register pointer is shared between them.
#[test]
fn the_scc_answers_in_both_its_windows() {
    let (machine, _cpu) = build(&[]);
    // Point at WR4 through the write window, load it, then read RR4 back
    // through the read window.
    write8(&machine, 0xbf_fff9, 4);
    write8(&machine, 0xbf_fff9, 0x44);
    write8(&machine, 0xbf_fff9, 4);
    assert_eq!(read8(&machine, SCC_READ), 0x44);
    // RR0 with nothing plugged in: the transmitter is empty and nothing has
    // arrived.
    let rr0 = read8(&machine, SCC_READ);
    assert_eq!(rr0 & 0x04, 0x04, "Tx Buffer Empty");
    assert_eq!(rr0 & 0x01, 0x00, "no character available");
}

/// Everything the board does *not* claim floats rather than faulting, which is
/// what a Macintosh Plus does: there is no bus-error timeout on these ranges,
/// and a ROM's first probe of a chip that is not fitted must not be an
/// exception the startup code has nowhere to take.
#[test]
fn an_address_nothing_claims_floats() {
    let (machine, cpu) = build(&[]);
    for addr in [0x50_0000u64, 0x58_0000, 0xf0_0000, 0xff_fffe] {
        assert!(
            space(&machine)
                .read(addr, Width::U16, MemAttrs::DEFAULT)
                .is_ok(),
            "{addr:#08x} must not fault"
        );
    }
    assert_eq!(cpu.bus_faults().0, 0);
}

/// `-p ram=4M` is a board with four megabytes, and on anything smaller memory
/// **repeats** through the four-megabyte window: the DRAM is given only the
/// address lines its own depth needs, so the top of the window is the top of
/// memory on every board. The ROM's boot icon depends on it —
/// `docs/platforms/mac-plus.md`.
#[test]
fn memory_repeats_through_the_four_megabyte_window() {
    for (param, len) in [("1M", 0x10_0000u64), ("2M", 0x20_0000), ("4M", 0x40_0000)] {
        let (machine, _cpu) = build(&[("ram", param)]);
        // Drop the overlay so the low window is memory.
        write8(&machine, VIA + 3 * 0x200, 0x7f);
        write8(&machine, VIA + 15 * 0x200, 0x00);

        write8(&machine, 0, 0x5a);
        assert_eq!(read8(&machine, 0), 0x5a);
        write8(&machine, len - 1, 0xa5);
        assert_eq!(read8(&machine, len - 1), 0xa5, "the last installed byte");
        // Every copy of the first byte, and the last byte of the window, which
        // is where the ROM draws its screen through.
        let mut at = len;
        while at < 0x40_0000 {
            assert_eq!(
                read8(&machine, at),
                0x5a,
                "{param}: the copy at {at:#08x} must alias the first byte"
            );
            at += len;
        }
        assert_eq!(
            read8(&machine, 0x40_0000 - 1),
            0xa5,
            "{param}: the top of the window is the top of memory"
        );
    }
}

/// A snapshot of the whole board round-trips to the same state hash, and the
/// two machines stay identical when they are run on.
#[test]
fn the_whole_board_snapshots_and_restores() {
    let (mut saved, _cpu) = build(&[]);
    saved
        .run_for(GlobalTime::from_nanos(5_000_000))
        .expect("it runs");
    // Put the board somewhere interesting first: overlay off, a timer running.
    write8(&saved, VIA + 3 * 0x200, 0x7f);
    write8(&saved, VIA + 15 * 0x200, 0x00);
    write8(&saved, VIA + 4 * 0x200, 0x34);
    write8(&saved, VIA + 5 * 0x200, 0x12);
    let image = saved.save().expect("a snapshot");

    let (mut restored, _other) = build(&[]);
    restored.load(&image).expect("it loads");
    assert_eq!(
        restored.state_hash().expect("a hash"),
        saved.state_hash().expect("a hash"),
        "the same board, bit for bit"
    );

    for _ in 0..4 {
        saved
            .run_for(GlobalTime::from_nanos(1_000_000))
            .expect("it runs");
        restored
            .run_for(GlobalTime::from_nanos(1_000_000))
            .expect("it runs");
    }
    assert_eq!(
        restored.state_hash().expect("a hash"),
        saved.state_hash().expect("a hash"),
        "and they stay identical"
    );
}
