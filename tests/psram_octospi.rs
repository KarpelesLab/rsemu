//! A guest storing into a QSPI PSRAM and loading it back, at both widths.
//!
//! A unit test can say "the controller built the right frame". This says
//! something stronger: an RV32 program, running on the emulated hart,
//!
//! 1. programs an `stm32.octospi` for single-line `02h`/`0Bh` and stores a word
//!    into its memory-mapped window,
//! 2. loads it back and leaves it in RAM,
//! 3. reprograms the same peripheral for **quad** `38h`/`EBh` — a one-line
//!    opcode with a four-line address, six quad dummy clocks and four-line data
//!    — and reads the *same address* again,
//! 4. writes a second word through the quad path and reads it back through the
//!    single-line one,
//! 5. and finally turns on `CR.TCEN` with a tiny `LPTR` and checks that a store
//!    long enough to hold the chip select past it sets `SR.TOF`.
//!
//! Every one of those accesses is a real frame clocked down `bus::spi` to a
//! real `psram.qspi`. Nothing copies the bytes anywhere: after step 1 the only
//! place they exist is the part's own array, and step 4 proves the two widths
//! are reading and writing the same array rather than two shadows of it.
//!
//! The board is built from the description below rather than from the catalog,
//! because it models no product — it exists so this claim has somewhere to run.

#![cfg(all(
    feature = "cpu-riscv",
    feature = "dev-stm32-octospi",
    feature = "dev-psram-qspi"
))]

use rsemu::core::clock::GlobalTime;
use rsemu::core::space::MemAttrs;
use rsemu::machine::{Machine, catalog};

// ---------------------------------------------------------------------------
// the board
// ---------------------------------------------------------------------------

/// Where the OCTOSPI's register block lands.
const REGS: u32 = 0xf000_1000;
/// Where its memory-mapped window lands — an STM32's OCTOSPI1 aperture.
const WINDOW: u32 = 0x9000_0000;
/// Where the RAM the firmware leaves its findings in lands.
const RAM: u32 = 0x2000_0000;
/// How big the part is, and therefore the window.
const SIZE: u32 = 64 * 1024;

const BOARD: &str = r#"
machine "psram-demo" {
  osc sysclk = 100000000 Hz
  space mem { width = 32 }

  object cpu "cpu.riscv" {
    clock  = sysclk
    space  = mem
    engine = "interp"
    xlen   = "rv32"
    isa    = "ima"
    hartid = 0
    reset  = 0x00000000
  }

  object fw "rom" { size = 64K, image = "firmware" }
  object dram "ram" { size = 64K }

  # An APS6404L-class part, with a tCEM budget it can actually exceed: 672
  # clocks is 8 us at 84 MHz, which is what a real board would write.
  object sram "psram.qspi" {
    size        = 64K
    bus         = "psram-bus"
    cs          = 0
    tcem-cycles = 672
    tcem-check  = "log"
  }

  object qspi "stm32.octospi" {
    link   = "transactional"
    bus    = "psram-bus"
    cs     = 0
    window = 64K
  }

  map mem 0x00000000 size 64K = fw
  map mem 0x20000000 size 64K = dram
  map mem 0x90000000 size 64K = qspi.mem
  map mem 0xf0001000 size 1K  = qspi
}
"#;

// ---------------------------------------------------------------------------
// just enough RV32I to write the firmware down
// ---------------------------------------------------------------------------

const T0: u32 = 5;
const T1: u32 = 6;
const T2: u32 = 7;
const A0: u32 = 10;

const fn lui(rd: u32, imm: u32) -> u32 {
    (imm << 12) | (rd << 7) | 0x37
}

const fn addi(rd: u32, rs1: u32, imm: i32) -> u32 {
    (((imm as u32) & 0xfff) << 20) | (rs1 << 15) | (rd << 7) | 0x13
}

const fn sw(rs2: u32, rs1: u32, imm: i32) -> u32 {
    let imm = imm as u32;
    ((imm >> 5) & 0x7f) << 25 | (rs2 << 20) | (rs1 << 15) | (2 << 12) | ((imm & 0x1f) << 7) | 0x23
}

const fn lw(rd: u32, rs1: u32, imm: i32) -> u32 {
    (((imm as u32) & 0xfff) << 20) | (rs1 << 15) | (2 << 12) | (rd << 7) | 0x03
}

/// `jal x0, 0`: branch to self.
const fn spin() -> u32 {
    0x6f
}

/// `li rd, imm`, as the assembler expands it: `lui` of the top twenty bits
/// with the sign of the low twelve folded in, then `addi`.
fn li(rd: u32, imm: u32) -> [u32; 2] {
    let hi = (imm.wrapping_add(0x800) >> 12) & 0xf_ffff;
    let lo = ((imm & 0xfff) as i32) << 20 >> 20;
    [lui(rd, hi), addi(rd, rd, lo)]
}

/// A `CCR` with an opcode on `imode` wires, a 24-bit address on `admode` and
/// data on `dmode`; `1` is one line and `3` is four.
const fn ccr(imode: u32, admode: u32, dmode: u32) -> u32 {
    imode | (admode << 8) | (2 << 12) | (dmode << 24)
}

/// Program `CCR`/`TCR`/`IR` and `WCCR`/`WTCR`/`WIR` from `t0`, using `a0`.
fn program(read: (u32, u32, u32), write: (u32, u32, u32)) -> Vec<u32> {
    let mut out = Vec::new();
    for (offset, value) in [
        (0x100, read.0),
        (0x108, read.1),
        (0x110, read.2),
        (0x180, write.0),
        (0x188, write.1),
        (0x190, write.2),
    ] {
        out.extend(li(A0, value));
        out.push(sw(A0, T0, offset));
    }
    out
}

/// `sw a0, off(t0)` with `a0` loaded from `value`.
fn poke(offset: i32, value: u32) -> Vec<u32> {
    let mut out = Vec::from(li(A0, value));
    out.push(sw(A0, T0, offset));
    out
}

/// The single-line command pair: `0Bh` fast read with eight dummy clocks, and
/// `02h` write.
const SINGLE: ((u32, u32, u32), (u32, u32, u32)) =
    ((ccr(1, 1, 1), 8, 0x0b), (ccr(1, 1, 1), 0, 0x02));

/// The quad pair: `EBh` with six dummy clocks — 24 bits at four lines, which
/// is three bytes — and `38h` quad write. The opcode stays on one wire, which
/// is what the part expects outside QPI mode.
const QUAD: ((u32, u32, u32), (u32, u32, u32)) = ((ccr(1, 3, 3), 6, 0xeb), (ccr(1, 3, 3), 0, 0x38));

/// `CR`: enable, memory-mapped (`FMODE = 11`).
const CR_MAPPED: u32 = 1 | (3 << 28);

/// The two words the firmware moves, and the sentinel that says it finished.
const FIRST: u32 = 0x1122_3344;
const SECOND: u32 = 0x5566_7788;
const SENTINEL: u32 = 0x600d_600d;

/// Where in the window each word goes. Both inside one 1 KiB page, so the
/// part's burst wrap is not what this test is about.
const AT_FIRST: i32 = 0x100;
const AT_SECOND: i32 = 0x200;

fn firmware() -> Vec<u8> {
    let mut p: Vec<u32> = Vec::new();
    p.extend(li(T0, REGS));
    p.extend(li(T1, WINDOW));
    p.extend(li(T2, RAM));

    // `DEVSIZE` for 64 KiB is 15: the field holds the exponent less one.
    p.extend(poke(0x008, 15 << 16));
    // `CSBOUND = 10` so the controller releases the chip select at every 1 KiB
    // boundary, which is where this part's linear burst wraps.
    p.extend(poke(0x010, 10 << 16));

    // -- single line ---------------------------------------------------------
    p.extend(program(SINGLE.0, SINGLE.1));
    p.extend(poke(0x000, CR_MAPPED));
    p.extend(li(A0, FIRST));
    p.push(sw(A0, T1, AT_FIRST));
    p.push(lw(A0, T1, AT_FIRST));
    p.push(sw(A0, T2, 0));

    // -- the same address, read back on four wires ---------------------------
    p.extend(program(QUAD.0, QUAD.1));
    p.push(lw(A0, T1, AT_FIRST));
    p.push(sw(A0, T2, 4));

    // -- a quad write, read back on one wire ---------------------------------
    p.extend(li(A0, SECOND));
    p.push(sw(A0, T1, AT_SECOND));
    p.extend(program(SINGLE.0, SINGLE.1));
    p.push(lw(A0, T1, AT_SECOND));
    p.push(sw(A0, T2, 8));

    // -- the chip-select timeout ---------------------------------------------
    // `LPTR` is eight clocks, which one single-line byte already spends, so any
    // real frame holds the chip select past it and `TOF` sets.
    p.extend(poke(0x130, 8));
    p.extend(poke(0x000, CR_MAPPED | (1 << 3)));
    p.extend(li(A0, FIRST));
    p.push(sw(A0, T1, AT_FIRST));
    p.push(lw(A0, T0, 0x020)); // SR
    p.push(sw(A0, T2, 12));

    // -- done ----------------------------------------------------------------
    p.extend(li(A0, SENTINEL));
    p.push(sw(A0, T2, 16));
    p.push(spin());

    p.iter().flat_map(|w| w.to_le_bytes()).collect()
}

// ---------------------------------------------------------------------------
// the run
// ---------------------------------------------------------------------------

fn boot() -> Machine {
    let mut options = catalog::build_options().expect("the catalog agrees with itself");
    options.realize.media.insert("firmware", firmware());
    let registry = catalog::registry().expect("a registry");
    rsemu::machine::build("psram-demo", BOARD, &registry, &options).expect("it realizes")
}

/// Read a word of the machine's address space, as a debugger would.
fn peek(machine: &Machine, at: u32) -> u32 {
    let space = machine.space("mem").expect("the board has one space");
    let mut out = [0u8; 4];
    space
        .read_bytes(u64::from(at), &mut out, MemAttrs::DEBUG)
        .expect("inside the map");
    u32::from_le_bytes(out)
}

/// Run until the firmware has left its sentinel, or give up.
fn run_until_done(machine: &mut Machine) {
    let mut elapsed = 0u64;
    while elapsed < 2_000_000_000 {
        if peek(machine, RAM + 16) == SENTINEL {
            return;
        }
        machine
            .run_for(GlobalTime::from_nanos(5_000_000))
            .expect("it runs");
        elapsed += 5_000_000;
    }
    panic!("the firmware never finished within {elapsed} ns of virtual time");
}

#[test]
fn a_guest_stores_and_loads_through_the_window_on_one_wire() {
    let mut machine = boot();
    run_until_done(&mut machine);
    assert_eq!(
        peek(&machine, RAM),
        FIRST,
        "a single-line 02h store and 0Bh load through the mapped window"
    );
}

#[test]
fn the_same_bytes_come_back_on_four_wires() {
    // The claim the width channel exists for. `EBh`'s address, dummy and data
    // phases are four wires wide, its opcode is one, and `DCYC = 6` is three
    // bytes rather than one — none of which the fabric could express before.
    let mut machine = boot();
    run_until_done(&mut machine);
    assert_eq!(
        peek(&machine, RAM + 4),
        FIRST,
        "a quad EBh read of what a single-line 02h wrote"
    );
}

#[test]
fn a_quad_write_is_visible_to_a_single_line_read() {
    // And the other direction, which is what proves there is one array behind
    // both paths rather than a shadow per width.
    let mut machine = boot();
    run_until_done(&mut machine);
    assert_eq!(peek(&machine, RAM + 8), SECOND);
}

#[test]
fn holding_the_chip_select_past_the_timeout_sets_tof() {
    let mut machine = boot();
    run_until_done(&mut machine);
    let sr = peek(&machine, RAM + 12);
    assert_ne!(
        sr & (1 << 4),
        0,
        "SR.TOF, in a register the guest read: {sr:#x}"
    );
}

#[test]
fn a_debugger_is_refused_the_window_rather_than_quietly_clocking_a_frame() {
    // The other half of `MemAttrs::debug`, at board level: reaching the part
    // means asserting a chip select and moving its command decoder, and there
    // is no side-effect-free route through a bus. A debugger that wants the
    // contents reads the part's own snapshot chunk.
    let machine = boot();
    let space = machine.space("mem").expect("the board has one space");
    let mut out = [0u8; 4];
    assert!(
        space
            .read_bytes(u64::from(WINDOW + SIZE - 4), &mut out, MemAttrs::DEBUG)
            .is_err()
    );
}
