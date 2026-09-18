//! The A500's 68000 executes the cycles its crystal owes it.
//!
//! `clk / 4` is 7 093 790 Hz on a PAL board, and a 68000 on an A500 retires
//! that many clocks a second whatever else is on the crystal: Agnus, Denise and
//! Paula are clocked by the same oscillator *at the same time*, they do not take
//! turns with the processor. The board used to run it at half that — Paula was
//! a runnable on the processor's crystal so it could poll the host serial port,
//! and a scheduler round divided a crystal's span between its runnables, so the
//! poll took half of every round with nothing executing
//! (`docs/platforms/amiga.md`).
//!
//! What is measured is what the processor **retired**, not what the scheduler
//! believes it handed out: [`M68k::cycles`] is the core's own count of bus and
//! internal cycles, and a counter in a register says how many times the loop
//! went round. Both have to agree with the crystal.
//!
//! A synthetic ROM built in this file, like every other ROM-free A500 test: no
//! Kickstart image is in this repository and none ever will be.

#![cfg(feature = "machine-amiga-a500")]

use std::sync::Arc;

use rsemu::core::Captured;
use rsemu::core::clock::GlobalTime;
use rsemu::cpu::m68k::{M68k, Reg};
use rsemu::machine::{Machine, catalog};

/// MC68000UM, *Instruction Execution Times*: `ADDQ.L #<data>,Dn` is 8(1/0)
/// and a taken `BRA` with a byte displacement is 10(2/0).
const CYCLES_PER_ITERATION: u64 = 8 + 10;

/// ```text
///   000000: 0008 0000   dc.l  $00080000   ; reset SSP
///   000004: 0000 000c   dc.l  $0000000c   ; reset PC, in the overlay
///   00000c: 7000        moveq  #0,d0
///   00000e: 5280        addq.l #1,d0
///   000010: 60fc        bra.s  $00000e
/// ```
///
/// The loop runs out of the ROM, which nothing else on the board contends for,
/// and touches no memory, so no DMA slot can stretch it.
fn rom() -> Vec<u8> {
    let mut image = vec![0u8; 512 * 1024];
    image[0..4].copy_from_slice(&0x0008_0000u32.to_be_bytes());
    image[4..8].copy_from_slice(&0x0000_000Cu32.to_be_bytes());
    for (i, word) in [0x7000u16, 0x5280, 0x60fc].iter().enumerate() {
        let at = 0x0c + 2 * i;
        image[at..at + 2].copy_from_slice(&word.to_be_bytes());
    }
    image
}

fn board() -> (Machine, Arc<M68k>) {
    let cores: Arc<Captured<M68k>> = Arc::new(Captured::new());
    let kept = Arc::clone(&cores);
    let mut options = catalog::build_options().expect("the catalog agrees with itself");
    options.bindings.replace("cpu.m68k", move |props| {
        let cpu = Arc::new(M68k::from_props(props)?);
        kept.push(&cpu);
        Ok(cpu)
    });
    options.realize.media.insert("kickstart", rom());
    options.realize.media.insert("df0", Vec::new());
    let registry = catalog::registry().expect("a registry");
    let source = catalog::machine("amiga-a500")
        .expect("this build ships amiga-a500")
        .source;
    let machine = match rsemu::machine::build("amiga-a500", source, &registry, &options) {
        Ok(m) => m,
        Err(e) => panic!("the board does not realize: {e}"),
    };
    let cpu = cores.last().expect("the binding captured the processor");
    (machine, cpu)
}

/// What the processor's own clock owes `nanos` of virtual time, exactly.
fn owed(m: &Machine, nanos: u64) -> u64 {
    let domain = m
        .device("cpu")
        .and_then(|d| d.domain())
        .expect("the processor has a clock domain");
    let f = m.clocks().domain_frequency(domain).expect("a rated domain");
    u64::try_from(u128::from(f.num()) * u128::from(nanos) / (u128::from(f.den()) * 1_000_000_000))
        .expect("a span this short fits")
}

#[test]
fn the_68000_retires_every_cycle_its_crystal_owes() {
    let (mut m, cpu) = board();
    // Past the reset sequence and into the loop, then measure a span that
    // starts and ends on the quantum grid, so no round is deferred at either
    // end.
    let warm = GlobalTime::from_nanos(10_000_000);
    m.run_until(warm).expect("the board runs");
    let (c0, d0) = (cpu.cycles(), cpu.reg(Reg::D(0)));

    let span_ns = 100_000_000;
    m.run_until(GlobalTime::from_nanos(10_000_000 + span_ns))
        .expect("the board runs");
    let (c1, d1) = (cpu.cycles(), cpu.reg(Reg::D(0)));

    let want = owed(&m, span_ns);
    let retired = c1 - c0;
    let percent = retired as f64 * 100.0 / want as f64;
    eprintln!("{retired} cycles retired of {want} owed ({percent:.2}%)");
    // One instruction either way: a core may overrun a budget by the tail of
    // the instruction it was in and carry that as debt into the next.
    assert!(
        retired.abs_diff(want) <= CYCLES_PER_ITERATION,
        "the 68000 retired {retired} cycles in {span_ns} ns where `clk / 4` owes {want} \
         ({percent:.2}%)"
    );

    // And the loop agrees with the core's own count, so the cycles were spent
    // executing rather than merely charged.
    let iterations = u64::from(d1.wrapping_sub(d0));
    assert!(
        (iterations * CYCLES_PER_ITERATION).abs_diff(retired) <= CYCLES_PER_ITERATION,
        "{iterations} iterations of {CYCLES_PER_ITERATION} cycles do not account for {retired}"
    );
}
