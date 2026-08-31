//! The `z80-mini` board, end to end.
//!
//! A unit test can say "the interpreter executed `OUT`". This says something
//! stronger: a Z80 **named in a `.machine` file** is handed *two* address
//! spaces by the machine layer — memory, and the separate 64 KiB of ports that
//! `IN` and `OUT` reach and nothing else does — resets to `0x0000`, runs a
//! program out of ROM, and the byte it wrote lands in the port space rather
//! than in memory.
//!
//! That second space is the thing worth proving. `space =` is structural and
//! there is one of it, so the I/O space is named by the `iospace` string
//! property and resolved with `BindCtx::space_named`; a board that got it wrong
//! would still realize, and every `OUT` would quietly vanish.
//!
//! Everything here needs a machine, so the whole file is gated on
//! `machine-z80-mini`.

#![cfg(feature = "machine-z80-mini")]

use rsemu::core::clock::GlobalTime;
use rsemu::core::space::MemAttrs;
use rsemu::core::value::Width;
use rsemu::machine::{Machine, catalog};

/// Where the board's RAM starts, with the file's default `ram-base`.
const RAM: u64 = 0x4000;

/// A firmware image, hand-assembled from the Zilog user manual's opcode table
/// (UM0080, appendix "Z80 CPU Instruction Set").
///
/// ```text
///   0000: 31 00 60     ld   sp, $6000     ; a stack in RAM
///   0003: 3e 5a        ld   a, $5a
///   0005: 32 00 40     ld   ($4000), a    ; -> memory
///   0008: 3c           inc  a             ; $5b
///   0009: d3 42        out  ($42), a      ; -> the *port* space
///   000b: 0e 07        ld   c, $07
///   000d: ed 78        in   a, (c)        ; and back out of it again
///   000f: 32 01 40     ld   ($4001), a
///   0012: 18 fe        jr   $             ; park
/// ```
///
/// The `IN` reads port `$07`, which the board mirrors onto the same 256-byte
/// scratch as everything else — so what comes back is whatever was written to
/// offset 7, and the assertion below is that it is *not* the `$5b` written to
/// `$42`. That is what catches a board that mapped one region over the whole
/// port space without mirroring, or a core that ignored the high address byte
/// differently from the map.
const FIRMWARE: &[u8] = &[
    0x31, 0x00, 0x60, // ld sp, $6000
    0x3e, 0x5a, // ld a, $5a
    0x32, 0x00, 0x40, // ld ($4000), a
    0x3c, // inc a
    0xd3, 0x42, // out ($42), a
    0x0e, 0x07, // ld c, $07
    0xed, 0x78, // in a, (c)
    0x32, 0x01, 0x40, // ld ($4001), a
    0x18, 0xfe, // jr $
];

/// Build the board out of the catalog with the firmware in its `firmware` slot.
fn boot() -> Machine {
    let entry = catalog::machine("z80-mini").expect("this build ships z80-mini");
    let mut options = catalog::build_options().expect("the catalog agrees with itself");
    options.realize.media.insert("firmware", FIRMWARE);
    let registry = catalog::registry().expect("a registry");
    match rsemu::machine::build(entry.name, entry.source, &registry, &options) {
        Ok(m) => m,
        Err(e) => panic!("the board does not realize: {e}"),
    }
}

/// Read one byte of a named space.
fn peek(m: &Machine, space: &str, addr: u64) -> u64 {
    m.space(space)
        .unwrap_or_else(|| panic!("the machine has no space called `{space}`"))
        .read(addr, Width::U8, MemAttrs::DEFAULT)
        .expect("a mapped byte")
}

#[test]
fn the_board_realizes_with_the_core_bound_to_both_of_its_spaces() {
    let m = boot();
    assert_eq!(m.name(), "z80-mini");
    for path in ["cpu", "boot", "dram", "io"] {
        assert!(
            m.device(path).is_some(),
            "the machine has no instance called `{path}`"
        );
    }
    // Two spaces, which is the whole point of this board.
    assert!(m.space("mem").is_some());
    assert!(m.space("port").is_some());
    assert_eq!(peek(&m, "mem", 0), u64::from(FIRMWARE[0]));
}

#[test]
fn the_firmware_runs_and_its_out_lands_in_the_port_space() {
    let mut m = boot();
    // A millisecond of virtual time at 3.5 MHz is 3,500 T-states, and the
    // program is under a hundred. A span rather than an instruction count
    // because the scheduler hands out budgets, not steps.
    m.run_for(GlobalTime::from_nanos(1_000_000))
        .expect("it runs");

    assert_eq!(
        peek(&m, "mem", RAM),
        0x5a,
        "the `LD ($4000),A` did not reach RAM"
    );
    assert_eq!(
        peek(&m, "port", 0x42),
        0x5b,
        "the `OUT ($42),A` did not reach the space `iospace` names"
    );
    assert_eq!(
        peek(&m, "mem", 0x42),
        0x00,
        "and it must not have gone into memory instead — that is the bug this \
         board exists to catch"
    );
    assert_eq!(
        peek(&m, "mem", RAM + 1),
        0x00,
        "the `IN A,(C)` read port $07, which nothing has written"
    );
}

#[test]
fn the_board_snapshots_and_restores_to_an_identical_state_hash() {
    let mut m = boot();
    m.run_for(GlobalTime::from_nanos(1_000_000))
        .expect("it runs");

    let bytes = m.save().expect("the machine snapshots");
    let before = m.state_hash().expect("a hash");

    let mut other = boot();
    other.load(&bytes).expect("the snapshot loads");
    assert_eq!(
        other.state_hash().expect("a hash"),
        before,
        "a save/load round trip changed the machine's state hash"
    );
    assert_eq!(peek(&other, "port", 0x42), 0x5b);
}
