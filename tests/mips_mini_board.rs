//! The `mips-mini` board, end to end.
//!
//! A unit test can say "the interpreter executed `SW`". This says something
//! stronger: a MIPS processor **named in a `.machine` file** is handed an
//! address space by the machine layer, fetches its first instruction from the
//! `kseg1` reset vector at `0xBFC0_0000`, runs a program out of a boot ROM,
//! and the bytes it wrote are in RAM afterwards.
//!
//! The segment map is the half a unit test with a hand-built space does not
//! exercise. The board maps **physical** addresses — RAM at 0 and the ROM at
//! `0x1FC0_0000` — and it is the *processor* that turns `0xBFC0_0000` into the
//! ROM and `0x8000_0000` into the RAM. A board that had written those out as
//! virtual mappings would realize, and would fetch nothing.
//!
//! Everything here needs a machine, so the whole file is gated on
//! `machine-mips-mini`.

#![cfg(feature = "machine-mips-mini")]

use rsemu::core::clock::GlobalTime;
use rsemu::core::space::MemAttrs;
use rsemu::core::value::Width;
use rsemu::machine::{Machine, catalog};

/// Where the board's boot ROM is in physical memory — `0xBFC0_0000` seen
/// through `kseg1`.
const ROM_PHYS: u64 = 0x1fc0_0000;

/// Where the program leaves its answer, in physical memory.
const RESULT: u64 = 0x0000_0400;

// ---------------------------------------------------------------------------
// A firmware image, assembled from the MIPS I instruction formats
// ---------------------------------------------------------------------------

const fn itype(op: u32, rs: u32, rt: u32, imm: u32) -> u32 {
    (op << 26) | (rs << 21) | (rt << 16) | (imm & 0xffff)
}
const fn special(funct: u32, rd: u32, rs: u32, rt: u32) -> u32 {
    (rs << 21) | (rt << 16) | (rd << 11) | funct
}
const fn lui(rt: u32, imm: u32) -> u32 {
    itype(0x0f, 0, rt, imm)
}
const fn addiu(rt: u32, rs: u32, imm: i32) -> u32 {
    itype(0x09, rs, rt, imm as u32)
}
const fn addu(rd: u32, rs: u32, rt: u32) -> u32 {
    special(0x21, rd, rs, rt)
}
const fn sw(rt: u32, rs: u32, imm: i32) -> u32 {
    itype(0x2b, rs, rt, imm as u32)
}
const fn lw(rt: u32, rs: u32, imm: i32) -> u32 {
    itype(0x23, rs, rt, imm as u32)
}
/// A `BNE` whose displacement is worked out from word indices. It counts from
/// the **delay slot**, so it is `to - (from + 1)`.
const fn bne(rs: u32, rt: u32, from: u32, to: u32) -> u32 {
    itype(0x05, rs, rt, (to as i32 - from as i32 - 1) as u32)
}
/// A jump to an absolute address. The top four bits of the target come from
/// the delay slot's program counter, which on this board never changes.
const fn j(target: u32) -> u32 {
    (0x02 << 26) | ((target >> 2) & 0x03ff_ffff)
}

const T0: u32 = 8;
const T1: u32 = 9;
const T2: u32 = 10;
const T3: u32 = 11;

/// The firmware: sum 1 to 10 in a loop, store the answer through `kseg0`, read
/// it back through `kseg1`, store *that* too, and spin.
///
/// Three things are being proved at once and each needs the others to be real:
///
/// * the loop has a **backwards branch with a delay slot**, so the answer is
///   wrong by one iteration if delay slots are mishandled;
/// * the store goes out through `kseg0` (`0x8000_0400`) and the load comes
///   back through `kseg1` (`0xA000_0400`), so the answer is wrong if the two
///   segments are not the same physical memory;
/// * both land at physical `0x400`, where the test reads them without going
///   through the processor at all.
fn firmware() -> Vec<u8> {
    const LOOP: u32 = 2;
    const BNE: u32 = 4;
    let code: &[u32] = &[
        // t1 = 0, t2 = 10
        addiu(T1, 0, 0),
        addiu(T2, 0, 10),
        // loop: t1 += t2; t2 -= 1; bne t2, zero, loop; nop
        addu(T1, T1, T2),
        addiu(T2, T2, -1),
        bne(T2, 0, BNE, LOOP),
        0, // the delay slot
        // t0 = 0x80000000 (kseg0), store the sum at physical 0x400
        lui(T0, 0x8000),
        sw(T1, T0, RESULT as i32),
        // t0 = 0xa0000000 (kseg1), read the same physical word back
        lui(T0, 0xa000),
        lw(T3, T0, RESULT as i32),
        0, // the load delay slot
        // Double it and store it back, so a stale read is visible as 55 and a
        // correct one as 110.
        addu(T3, T3, T3),
        sw(T3, T0, RESULT as i32),
        // Spin, with a delay slot.
        j(0xbfc0_0034),
        0,
    ];
    let mut image = Vec::with_capacity(code.len() * 4);
    for word in code {
        image.extend_from_slice(&word.to_le_bytes());
    }
    image
}

/// Build the board out of the catalog with the firmware in its `firmware`
/// slot.
fn boot() -> Machine {
    let entry = catalog::machine("mips-mini").expect("this build ships mips-mini");
    let mut options = catalog::build_options().expect("the catalog agrees with itself");
    options.realize.media.insert("firmware", firmware());
    let registry = catalog::registry().expect("a registry");
    match rsemu::machine::build(entry.name, entry.source, &registry, &options) {
        Ok(m) => m,
        Err(e) => panic!("the board does not realize: {e}"),
    }
}

/// Read a little-endian word out of the guest's **physical** memory.
fn peek(m: &Machine, addr: u64) -> u32 {
    let space = m.space("mem").expect("the memory space");
    space
        .read(addr, Width::U32, MemAttrs::DEFAULT)
        .expect("a mapped word") as u32
}

#[test]
fn the_board_realizes_with_the_core_bound_to_its_space() {
    let m = boot();
    assert_eq!(m.name(), "mips-mini");
    for path in ["cpu", "boot", "dram"] {
        assert!(
            m.device(path).is_some(),
            "the machine has no instance called `{path}`"
        );
    }
    // The first instruction the processor fetches is at physical 0x1fc00000,
    // which is where 0xbfc00000 lands after kseg1 strips the top three bits.
    assert_eq!(peek(&m, ROM_PHYS), addiu(T1, 0, 0));
}

#[test]
fn the_firmware_runs_a_loop_through_two_segments_and_leaves_the_right_answer() {
    let mut m = boot();
    // A millisecond of virtual time at 25 MHz is 25,000 accesses and the
    // program is under fifty. A span rather than an instruction count because
    // the scheduler hands out budgets, not steps.
    m.run_for(GlobalTime::from_nanos(1_000_000))
        .expect("it runs");

    assert_eq!(
        peek(&m, RESULT),
        110,
        "either the loop's delay slot was mishandled (55 doubled is 110, and \
         one iteration too few or too many is 90 or 132), or kseg0 and kseg1 \
         are not the same physical memory"
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
    assert_eq!(peek(&other, RESULT), 110);
}

#[test]
fn the_restored_board_keeps_running_the_same_way() {
    // The stronger claim: not only does the state hash match, the two machines
    // stay in step afterwards — which is what catches a snapshot that dropped
    // the delay-slot flag or a load in flight.
    let mut m = boot();
    m.run_for(GlobalTime::from_nanos(200)).expect("it runs");
    let bytes = m.save().expect("the machine snapshots");

    let mut other = boot();
    other.load(&bytes).expect("the snapshot loads");

    for _ in 0..20 {
        m.run_for(GlobalTime::from_nanos(200)).expect("it runs");
        other.run_for(GlobalTime::from_nanos(200)).expect("it runs");
        assert_eq!(
            m.state_hash().expect("a hash"),
            other.state_hash().expect("a hash"),
            "the two machines diverged after a restore"
        );
    }
}

#[test]
fn a_machine_file_can_ask_for_the_part_with_no_tlb() {
    // The `ROADMAP.md` §6.1.1 claim, made by a `.machine` file rather than by
    // a Rust test: which part this is comes from a construction property, so
    // one binary runs an R3000A and an LR33300.
    let entry = catalog::machine("mips-mini").expect("this build ships mips-mini");
    let mut options = catalog::build_options().expect("the catalog agrees with itself");
    options.realize.media.insert("firmware", firmware());
    options
        .resolve
        .params
        .push(("arch".into(), "lr33300".into()));
    let registry = catalog::registry().expect("a registry");
    let mut m = rsemu::machine::build(entry.name, entry.source, &registry, &options)
        .expect("the board realizes with the LSI part");
    m.run_for(GlobalTime::from_nanos(1_000_000))
        .expect("it runs");
    assert_eq!(
        peek(&m, RESULT),
        110,
        "the same firmware runs on the part with no TLB"
    );
}
