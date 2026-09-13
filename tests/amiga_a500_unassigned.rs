//! What an Amiga 500 does with an address nothing answers at.
//!
//! # Why this file exists
//!
//! A real Kickstart 2.04 on `machines/amiga-a500.machine` halted before it
//! touched a single chip register. It sums its own ROM, and its very next data
//! access is a **word read at `$F0_0000`** — made while `OVL` is still up, so
//! the exception vectors are still whatever the ROM has at those offsets. The
//! board left its space at the default `unassigned = fault`, the read became a
//! bus error, vector 2 came out of the ROM overlay as a longword that is not a
//! handler, and the processor double-faulted. AROS's ROM stops at the same
//! access. Neither ROM guards the read, which is the first piece of evidence
//! that the machine they were written for does not raise `/BERR` there.
//!
//! # What the manuals say
//!
//! * *Amiga Hardware Reference Manual*, 3rd ed., Appendix D, "A1000, A500 and
//!   A2000 Memory Map" (p. 314): `$F0_0000`–`$FB_FFFF` is "Reserved. Do not
//!   use." — as are `$10_0000`–`$1F_FFFF`, `$A0_0000`–`$BE_FFFF`, the
//!   `$C0_0000`–`$DF_EFFF` block around slow RAM and the clock, and
//!   `$E0_0000`–`$E7_FFFF`. The A3000 map on p. 315 names the same
//!   `$F0_0000` range "Diagnostic ROM (Reserved)". Reserved says what software
//!   may rely on; it does not say anything drives the bus.
//! * *MC68000 User's Manual* (M68000UM/AD rev. 8), §5.4: `BERR` is an input
//!   that "external circuitry can be provided to assert" after a timeout. A
//!   68000 board raises a bus error only if it was built with that circuit.
//! * HRM Appendix K, "Zorro Expansion Bus" (p. 397, `/DTACK`): "If a Zorro II
//!   slave does nothing, this /DTACK will be driven by the bus controller with
//!   no wait states". And (p. 393–394, `/BERR`) the controller drives `/BERR`
//!   "in the event of a detected bus collision or DMA error" — never for an
//!   address with nothing at it. Appendix E's 86-pin expansion connector table
//!   gives the A500 the same `/DTACK`, `/OVR` and `RDY` pins as the A2000.
//!
//! So an empty address on an A500 **completes its cycle and floats**: the
//! bus controller acknowledges it, nobody drives the data lines, and a write
//! goes nowhere. None of the manuals gives a value for the floating bus, and
//! no region of the A500 map is documented as raising `/BERR`.
//!
//! `unassigned = open-bus` is the policy that says exactly that. The 68000
//! core keeps no data-bus latch, so today it reads zero; that is also what a
//! Kickstart's own probes need to conclude "nothing fitted" — no diagnostic
//! cartridge at `$F0_0000`, no slow RAM at `$C0_0000`, no board at `$E8_0000`,
//! no second half of chip RAM at `$08_0000`.
//!
//! # How it is tested
//!
//! The board is built from the shipped machine source with the space line
//! rewritten, so the same file proves the fix before and after it lands in
//! `amiga-a500.machine`. The firmware is synthetic and hand-assembled from the
//! MC68000 User's Manual's instruction formats; **no byte of any Kickstart is
//! in this file**. One test runs the user's own ROMs *in place*, behind
//! `RSEMU_AMIGA_ROM_DIR`, and asserts nothing about their contents — only
//! where the processor faulted and whether it reached CIA-A.
//!
//! No Amiga emulator source and no AROS source was consulted (`ROADMAP.md` §1).

#![cfg(feature = "machine-amiga-a500")]

use std::sync::Arc;

use rsemu::core::Captured;
use rsemu::core::clock::GlobalTime;
use rsemu::core::space::MemAttrs;
use rsemu::core::value::Width;
use rsemu::cpu::m68k::M68k;
use rsemu::machine::{Machine, catalog};

/// The board's space line as the shipped file had it: no policy, so `fault`.
const SPACE_DEFAULT: &str = r#"space mem { width = 24, endian = "big" }"#;

/// The line the board needs.
const SPACE_OPEN_BUS: &str = r#"space mem { width = 24, endian = "big", unassigned = open-bus }"#;

/// The same space with the policy the board used to have, spelled out.
const SPACE_FAULT: &str = r#"space mem { width = 24, endian = "big", unassigned = fault }"#;

/// CIA-A's `DDRA`, `$BFEr01` with `r` = 2 (Appendix F).
const CIAA_DDRA: u64 = 0xBF_E201;

/// Where the synthetic firmware leaves its marker once every probe is done.
const MARKER_AT: u32 = 0x00_0100;
const MARKER: u32 = 0x1234_ABCD;

/// Every range on a stock A500 that Appendix D leaves without a device, as
/// this board maps it (512 KiB of chip RAM, a 512 KiB ROM at `$F8_0000`).
///
/// Inclusive bounds, and each with the row it comes from.
const HOLES: &[(u64, u64, &str)] = &[
    (0x08_0000, 0x0F_FFFF, "Extended chip RAM, not fitted"),
    (0x10_0000, 0x1F_FFFF, "Reserved. Do not use."),
    (0x20_0000, 0x9F_FFFF, "Primary 8 MB Auto-config space"),
    (0xA0_0000, 0xBE_FFFF, "Reserved. Do not use."),
    (0xBF_0000, 0xBF_CFFF, "below the 8520-B window, in no row"),
    (0xBF_F000, 0xBF_FFFF, "above the 8520-A window, in no row"),
    (0xC0_0000, 0xD7_FFFF, "Internal expansion (slow) memory"),
    (0xD8_0000, 0xDB_FFFF, "Reserved. Do not use."),
    (0xDC_0000, 0xDC_FFFF, "Real time clock, no socket"),
    (0xDD_0000, 0xDF_EFFF, "rest of C0 0000 - DF EFFF"),
    (0xDF_F200, 0xDF_FFFF, "chip window past the table"),
    (0xE0_0000, 0xE7_FFFF, "Reserved. Do not use."),
    (0xE8_0000, 0xE8_FFFF, "Auto-config space, no board"),
    (0xE9_0000, 0xEF_FFFF, "Secondary auto-config space"),
    (0xF0_0000, 0xF7_FFFF, "Reserved (A3000: diagnostic ROM)"),
];

/// The shipped board's source with its space line set to `line`.
///
/// Tolerant of the fix having landed: whichever of the two known spellings the
/// file carries is the one replaced, and a file with neither fails loudly
/// rather than building an unmodified board.
fn source_with(line: &str) -> String {
    let shipped = catalog::machine("amiga-a500")
        .expect("this build ships amiga-a500")
        .source;
    let current = [SPACE_DEFAULT, SPACE_OPEN_BUS]
        .into_iter()
        .find(|known| shipped.contains(known))
        .expect("amiga-a500.machine's `space mem` line is one this test knows");
    shipped.replace(current, line)
}

/// Build `source` with `rom` in the `kickstart` slot, keeping the processor.
fn build(source: &str, rom: Vec<u8>) -> (Machine, Arc<M68k>) {
    let cores: Arc<Captured<M68k>> = Arc::new(Captured::new());
    let kept = Arc::clone(&cores);
    let mut options = catalog::build_options().expect("the catalog agrees with itself");
    options.bindings.replace("cpu.m68k", move |props| {
        let cpu = Arc::new(M68k::from_props(props)?);
        kept.push(&cpu);
        Ok(cpu)
    });
    options.realize.media.insert("kickstart", rom);
    let registry = catalog::registry().expect("a registry");
    let machine = match rsemu::machine::build("amiga-a500", source, &registry, &options) {
        Ok(m) => m,
        Err(e) => panic!("the board does not realize: {e}"),
    };
    let cpu = cores.last().expect("the binding captured the processor");
    (machine, cpu)
}

fn read(m: &Machine, addr: u64, width: Width) -> rsemu::core::space::MemResult<u64> {
    m.space("mem")
        .expect("the memory space")
        .read(addr, width, MemAttrs::DEFAULT)
}

fn write(m: &Machine, addr: u64, width: Width, value: u64) -> rsemu::core::space::MemResult {
    m.space("mem")
        .expect("the memory space")
        .write(addr, width, value, MemAttrs::DEFAULT)
}

// ---------------------------------------------------------------------------
// A hand assembler, just big enough
// ---------------------------------------------------------------------------

/// 68000 code assembled at ROM offset `$0C`, the reset PC the image carries.
///
/// Encodings from MC68000UM, *Instruction Set Details*: `TST <ea>` is
/// `0100 1010 ss mmmrrr`, `MOVE` is `00ss dddddd ssssss`, absolute long is mode
/// 7 register 1, immediate is mode 7 register 4.
struct Asm(Vec<u16>);

impl Asm {
    /// Where the next word lands, as a ROM offset.
    fn here(&self) -> u32 {
        0x0C + 2 * self.0.len() as u32
    }
    fn long(&mut self, value: u32) {
        self.0.push((value >> 16) as u16);
        self.0.push(value as u16);
    }
    /// `TST.B (xxx).L`
    fn tst_b(&mut self, addr: u32) {
        self.0.push(0x4A39);
        self.long(addr);
    }
    /// `TST.W (xxx).L`
    fn tst_w(&mut self, addr: u32) {
        self.0.push(0x4A79);
        self.long(addr);
    }
    /// `MOVE.B #imm,(xxx).L`
    fn move_b(&mut self, imm: u8, addr: u32) {
        self.0.push(0x13FC);
        self.0.push(u16::from(imm));
        self.long(addr);
    }
    /// `MOVE.W #imm,(xxx).L`
    fn move_w(&mut self, imm: u16, addr: u32) {
        self.0.push(0x33FC);
        self.0.push(imm);
        self.long(addr);
    }
    /// `MOVE.L #imm,(xxx).L`
    fn move_l(&mut self, imm: u32, addr: u32) {
        self.0.push(0x23FC);
        self.long(imm);
        self.long(addr);
    }
    /// `JMP (xxx).L` to the instruction after itself, in the ROM's own window.
    fn leave_overlay(&mut self) {
        let next = 0x00F8_0000 + self.here() + 6;
        self.0.push(0x4EF9);
        self.long(next);
    }
    /// `BRA *`
    fn park(&mut self) {
        self.0.push(0x60FE);
    }
}

/// A 512 KiB image: SSP at the top of chip RAM, PC at `$0C`, then `code`.
fn image(code: &[u16]) -> Vec<u8> {
    let mut image = vec![0u8; 512 * 1024];
    image[0..4].copy_from_slice(&0x0008_0000u32.to_be_bytes());
    image[4..8].copy_from_slice(&0x0000_000Cu32.to_be_bytes());
    for (i, word) in code.iter().enumerate() {
        let at = 0x0C + 2 * i;
        image[at..at + 2].copy_from_slice(&word.to_be_bytes());
    }
    image
}

/// Firmware that does what a Kickstart does first, then walks every hole.
///
/// The `$F0_0000` read comes *before* the overlay is left, as it does on the
/// real ROM, so a board that faults it takes the bus error with its vectors
/// still in ROM.
fn prober() -> Vec<u8> {
    let mut a = Asm(Vec::new());
    a.tst_w(0x00F0_0000);
    a.leave_overlay();
    a.move_b(0x00, 0x00BF_E001); // PRA:  OVL low
    a.move_b(0x01, 0x00BF_E201); // DDRA: PA0 an output -> chip RAM at 0
    for &(start, end, _) in HOLES {
        for addr in [start, end & !1] {
            let addr = addr as u32;
            a.tst_b(addr);
            a.tst_w(addr);
            a.move_w(0xA55A, addr);
            a.tst_w(addr);
        }
    }
    a.move_l(MARKER, MARKER_AT);
    a.park();
    image(&a.0)
}

fn marker(m: &Machine) -> u32 {
    read(m, u64::from(MARKER_AT), Width::U32).expect("chip RAM") as u32
}

// ---------------------------------------------------------------------------
// The map
// ---------------------------------------------------------------------------

#[test]
fn every_hole_in_appendix_d_completes_a_read_and_drops_a_write() {
    let (m, _) = build(&source_with(SPACE_OPEN_BUS), prober());
    for &(start, end, row) in HOLES {
        for (addr, width) in [
            (start, Width::U8),
            (start, Width::U16),
            (end, Width::U8),
            (end - 1, Width::U16),
        ] {
            let before = read(&m, addr, width)
                .unwrap_or_else(|e| panic!("{addr:#08x} ({row}): a read faulted: {e:?}"));
            write(&m, addr, width, 0xA55A & width.mask())
                .unwrap_or_else(|e| panic!("{addr:#08x} ({row}): a write faulted: {e:?}"));
            assert_eq!(
                read(&m, addr, width),
                Ok(before),
                "{addr:#08x} ({row}): a store stuck to an address with nothing at it"
            );
        }
    }
}

#[test]
fn the_holes_do_not_swallow_what_is_mapped() {
    // A permissive policy must not paper over a decode. The ROM still answers
    // with its own bytes rather than a float, and a CIA register keeps a store
    // that a floating address would not. (`DDRA` bit 1 is the LED pin, which
    // leaves the overlay alone.)
    let (m, _) = build(&source_with(SPACE_OPEN_BUS), prober());
    assert_eq!(read(&m, 0xF8_0004, Width::U32), Ok(0x0000_000C), "the ROM");
    write(&m, CIAA_DDRA, Width::U8, 0x02).expect("CIA-A");
    assert_eq!(read(&m, CIAA_DDRA, Width::U8), Ok(0x02), "CIA-A's DDRA");
}

// ---------------------------------------------------------------------------
// The processor
// ---------------------------------------------------------------------------

#[test]
fn firmware_that_probes_every_hole_runs_through_without_a_bus_error() {
    let (mut m, cpu) = build(&source_with(SPACE_OPEN_BUS), prober());
    m.run_for(GlobalTime::from_nanos(50_000_000))
        .expect("it runs");
    assert_eq!(cpu.bus_faults().0, 0, "no access was refused");
    assert!(!cpu.is_halted());
    assert_eq!(marker(&m), MARKER, "the firmware reached its last store");
    assert_eq!(
        read(&m, CIAA_DDRA, Width::U8),
        Ok(0x01),
        "and left the overlay through CIA-A on the way"
    );
}

#[test]
fn the_same_firmware_on_a_faulting_board_takes_a_bus_error_at_f00000() {
    // The defect, kept reproducible: with `fault` the very first data access
    // is refused, and it is the one Kickstart makes. A millisecond is a few
    // thousand clocks, long enough for the probe and too short for the stack a
    // repeating bus error eats to reach anything else.
    let (mut m, cpu) = build(&source_with(SPACE_FAULT), prober());
    m.run_for(GlobalTime::from_nanos(1_000_000))
        .expect("it runs");
    let (faults, last) = cpu.bus_faults();
    assert!(faults > 0, "the probe was refused");
    assert_eq!(last, 0x00F0_0000, "and it was the probe at $F00000");
    assert_ne!(marker(&m), MARKER, "it never got past it");
}

// ---------------------------------------------------------------------------
// The user's own ROMs, run in place
// ---------------------------------------------------------------------------

/// Run a real ROM on the board built from `source`, in place, for `span`.
///
/// Returns `None` (after saying so) when the variable or the file is absent.
/// Nothing is copied: the decoded bytes live in this process and go nowhere
/// but the media slot.
#[cfg(feature = "media-kickstart")]
fn real_rom(file: &str, source: &str, span: GlobalTime) -> Option<(Machine, Arc<M68k>)> {
    let Ok(dir) = std::env::var("RSEMU_AMIGA_ROM_DIR") else {
        println!(
            "amiga-a500: set RSEMU_AMIGA_ROM_DIR to an Amiga Forever `Shared/rom` directory \
             to run real ROMs against the board's unassigned policy."
        );
        return None;
    };
    let path = std::path::Path::new(&dir).join(file);
    if !path.exists() {
        println!("amiga-a500: {} is not there; skipped", path.display());
        return None;
    }
    let image = rsemu::host::media::kickstart::open(&path.to_string_lossy())
        .unwrap_or_else(|e| panic!("{}: {e}", path.display()));
    let (mut m, cpu) = build(source, image.bytes);
    m.run_for(span).expect("it runs");
    Some((m, cpu))
}

/// Kickstart 2.04 and AROS: halted at `$F00000` on a faulting board, through to
/// CIA-A with no bus error on an open-bus one.
///
/// Two virtual seconds, because both ROMs sum all 512 KiB of themselves before
/// their first data access, which takes a little over one. The assertions name
/// addresses on the board and nothing inside the images.
#[cfg(feature = "media-kickstart")]
#[test]
fn real_roms_reach_cia_a_once_reserved_space_floats() {
    let span = GlobalTime::from_nanos(2_000_000_000);
    for rom in ["amiga-os-204.rom", "aros-20250422.rom"] {
        let Some((_, cpu)) = real_rom(rom, &source_with(SPACE_FAULT), span) else {
            continue;
        };
        assert!(cpu.is_halted(), "{rom}: a faulting board halts it");
        assert_eq!(
            cpu.bus_faults().1,
            0x00F0_0000,
            "{rom}: at the $F00000 probe"
        );

        let Some((m, cpu)) = real_rom(rom, &source_with(SPACE_OPEN_BUS), span) else {
            continue;
        };
        assert!(
            !cpu.is_halted(),
            "{rom}: an open-bus board does not halt it"
        );
        assert_eq!(cpu.bus_faults().0, 0, "{rom}: no access was refused");
        assert_eq!(
            read(&m, CIAA_DDRA, Width::U8).expect("CIA-A") & 0x01,
            0x01,
            "{rom}: PA0 was made an output, so the ROM reached CIA-A"
        );
    }
}
