//! **How much of a real program the m68k IR frontend actually lifts**, measured
//! on the boards in this tree that run a 68000.
//!
//! # Why this file exists separately from the differential harness
//!
//! `tests/m68k_lift_differential.rs` answers *are the two engines the same*.
//! It says nothing at all about *how much of a run the translated one carried*,
//! and those are different questions with different failure modes: a frontend
//! that lifted nothing at all would pass every sweep in that file, because a
//! fallback agrees with the interpreter by construction. `docs/cpu/m68k.md`
//! had a paragraph of reasoning about `JSR`/`RTS` pairs being common in 68000
//! code and no number beside it. This is the number.
//!
//! # What is measured, and on what
//!
//! Four columns, out of `M68k::ir_stats` and `M68k::ir_declines`:
//!
//! 1. the fraction of **executed guest instructions** that retired inside a
//!    lifted block, against the ones the interpreter took;
//! 2. the fraction of **block executions** that ended at an encoding the
//!    frontend declined rather than at a transfer of control;
//! 3. the **mean guest instructions per block execution**;
//! 4. a **histogram of every fallback**, by the five categories of
//!    [`Decline`](rsemu::cpu::m68k::lift::Decline) and, within each, by
//!    mnemonic.
//!
//! The histogram sums to `IrStats::interpreted` — there is no "other" bucket —
//! and `executed` is partitioned by the five `ended_*` rows plus `spent` plus
//! `faults`. Both closures are asserted here rather than hoped for, because a
//! histogram that does not account for every fallback reads as a lift rate
//! better than it is.
//!
//! # The boards
//!
//! | Board | Code | Media |
//! | --- | --- | --- |
//! | `m68k-mini` | a hand-written program, so **synthetic** | none |
//! | `mac-plus` | Apple's own 128 KiB ROM | `RSEMU_MAC_ROM_DIR/Mac-Plus.ROM` |
//! | `mac-classic` | Apple's own 512 KiB ROM | `RSEMU_MAC_ROM_DIR/Classic.ROM` |
//!
//! The Amiga boards would be the fourth source and are not measured here: they
//! need a Kickstart, which is `RSEMU_AMIGA_ROM_DIR`, and nothing in this file
//! ships one. `docs/cpu/m68k.md` records which boards the published numbers
//! came from.
//!
//! **No byte of any ROM is in this repository or in this file.** Each ROM is
//! read in place out of the user's own directory and the test skips, saying
//! why, when it is not there — the pattern `tests/mac_plus.rs` established.
//! Nothing here disassembles a ROM or prints an instruction from one:
//! aggregate counters and a per-mnemonic tally are facts about *this engine*,
//! and that is all that is produced (`CLAUDE.md`, *Provenance*).
//!
//! # And the run is checked against the oracle
//!
//! Each ROM measurement runs the **same board on both engines** and compares
//! the whole machine's state hash at every virtual second. That is the
//! strongest differential case this core has — millions of instructions of
//! somebody else's real code rather than a generated program — and it is what
//! makes the rate beside it worth quoting: a rate measured on a translated
//! engine that had drifted would be a rate for a different guest.

#![cfg(feature = "cpu-m68k-lift")]

// The instrument's own arithmetic, behind the gate that says there is a board
// to point it at: a build with `cpu-m68k-lift` and no machine at all — which
// is the feature-alone gate CI runs — has nothing to measure and must not
// carry an unused reporter.
#[cfg(any(
    feature = "machine-m68k-mini",
    feature = "machine-mac-plus",
    feature = "machine-mac-classic"
))]
mod instrument {
    pub(crate) use rsemu::cpu::m68k::lift::Decline;
    use rsemu::cpu::m68k::{IrDeclineRow, IrStats};

    /// Integer per-mille of `part` in `whole`, or `None` for an empty whole.
    ///
    /// Per-mille and not a percentage with a decimal point: no float is allowed to
    /// reach anything a run produces (`CLAUDE.md`, *Determinism*), and the raw
    /// counts are printed beside every ratio anyway.
    fn permille(part: u64, whole: u64) -> Option<u64> {
        part.saturating_mul(1_000).checked_div(whole)
    }

    /// Print the measurement, and assert the two closure properties it rests on.
    ///
    /// Returns the lift rate in per-mille, so a caller can assert a floor without
    /// re-deriving it.
    pub(crate) fn report(label: &str, stats: IrStats, declines: &[IrDeclineRow]) -> u64 {
        // Every guest instruction the core executed, by who executed it. A fault
        // hands exactly one instruction back, so it is a third column and not part
        // of either of the other two.
        let executed_insns = stats.retired + stats.interpreted + stats.faults;
        let rate = permille(stats.retired, executed_insns).unwrap_or(0);

        let ended = stats.ended_unsupported
            + stats.ended_transfer
            + stats.ended_window
            + stats.ended_limit
            + stats.ended_unreadable;

        println!("\n=== {label} ===");
        println!(
            "instructions   {executed_insns:>12}  = {:>12} in a block + {:>10} interpreted \
             + {} restarted after a fault",
            stats.retired, stats.interpreted, stats.faults
        );
        println!(
            "lift rate      {rate:>12} per mille of executed guest instructions ran inside \
             a lifted block"
        );
        println!(
            "blocks         {:>12}  from {} translations; {} left on the budget, {} faulted",
            stats.executed, stats.lifted, stats.spent, stats.faults
        );
        // Hundredths, for the same reason the ratios are per-mille: two decimal
        // places of "instructions per block" without a float in sight.
        println!(
            "per block      {:>12} hundredths of a guest instruction per block execution",
            stats
                .retired
                .saturating_mul(100)
                .checked_div(stats.executed)
                .unwrap_or(0)
        );
        println!("blocks ended at a terminator, by why lifting stopped there:");
        for (name, count) in [
            ("declined encoding", stats.ended_unsupported),
            ("transfer of control", stats.ended_transfer),
            ("window boundary", stats.ended_window),
            ("instruction limit", stats.ended_limit),
            ("unreadable words", stats.ended_unreadable),
        ] {
            println!(
                "  {name:<22}{count:>12}  {:>4} per mille of the {ended} that did",
                permille(count, ended).unwrap_or(0)
            );
        }
        println!(
            "fallbacks, by why no block could run ({} in all):",
            stats.interpreted
        );
        for &reason in Decline::ALL {
            let total: u64 = declines
                .iter()
                .filter(|r| r.reason == reason)
                .map(|r| r.count)
                .sum();
            if total == 0 {
                continue;
            }
            println!(
                "  {:<8}{total:>14}  {:>4} per mille of fallbacks — {}",
                reason.name(),
                permille(total, stats.interpreted).unwrap_or(0),
                reason.summary()
            );
            for row in declines
                .iter()
                .filter(|r| r.reason == reason && r.count > 0)
            {
                println!(
                    "      {:<18}{:>12}  {:>4} per mille",
                    row.what,
                    row.count,
                    permille(row.count, stats.interpreted).unwrap_or(0)
                );
            }
        }

        // The two closures. Neither is a property of the guest: both are
        // arithmetic this instrument does, and an instrument whose own arithmetic
        // is unchecked is an instrument that will be quoted and be wrong.
        let attributed: u64 = declines.iter().map(|r| r.count).sum();
        assert_eq!(
            attributed, stats.interpreted,
            "{label}: the histogram accounts for {attributed} of {} fallbacks",
            stats.interpreted
        );
        assert_eq!(
            stats.executed,
            ended + stats.spent + stats.faults,
            "{label}: {} block executions, {ended} at a terminator + {} on the budget + {} faulted",
            stats.executed,
            stats.spent,
            stats.faults
        );
        rate
    }
}

#[cfg(any(
    feature = "machine-m68k-mini",
    feature = "machine-mac-plus",
    feature = "machine-mac-classic"
))]
use instrument::{Decline, report};

// ---------------------------------------------------------------------------
// m68k-mini: synthetic, and the one that runs everywhere
// ---------------------------------------------------------------------------

#[cfg(feature = "machine-m68k-mini")]
mod mini {
    use std::sync::Arc;

    use rsemu::core::Captured;
    use rsemu::core::clock::GlobalTime;
    use rsemu::cpu::m68k::M68k;
    use rsemu::machine::{Machine, catalog};

    /// A program with the shape of compiled 68000 code rather than a loop:
    /// a caller that sets up arguments and calls a subroutine, and a
    /// subroutine that builds a frame, works in it and returns.
    ///
    /// Hand-assembled from the instruction formats of M68000PM/AD §8. It is
    /// **synthetic** and is labelled so wherever its number is quoted: what
    /// mix a program has is exactly the thing being measured, so a mix this
    /// file chose is evidence about this file. It is here because it needs no
    /// media, so the instrument is exercised on every machine that runs the
    /// suite — and because a `JSR`/`RTS` pair per iteration is the case
    /// `docs/cpu/m68k.md` speculated about.
    ///
    /// ```text
    ///   000400: 2e7c 0020 0000   move.l  #$00200000,a7   ; the stack, at the top of RAM
    ///   000406: 227c 0010 1000   move.l  #$00101000,a1   ; the array, well inside it
    ///   00040c: 303c 7fff        move.w  #$7fff,d0      ; the counter
    ///   000410: 4eb9 0000 0420   jsr     $00000420       ; -- the call, once per iteration
    ///   000416: 51c8 fff8        dbf     d0,$00000410
    ///   00041a: 60fe             bra     *
    ///
    ///   000420: 4e56 fffc        link    a6,#-4          ; a frame
    ///   000424: 2f02             move.l  d2,-(a7)        ; a callee-saved register
    ///   000426: 2411             move.l  (a1),d2
    ///   000428: d481             add.l   d1,d2
    ///   00042a: e58a             lsl.l   #2,d2
    ///   00042c: 2482             move.l  d2,(a1)
    ///   00042e: 3221             move.w  -(a1),d1
    ///   000430: d269 0002        add.w   2(a1),d1
    ///   000434: 0c41 1234        cmpi.w  #$1234,d1
    ///   000438: 6702             beq     $0000043c
    ///   00043a: 5341             subq.w  #1,d1
    ///   00043c: 241f             move.l  (a7)+,d2
    ///   00043e: 4e5e             unlk    a6
    ///   000440: 4e75             rts
    /// ```
    ///
    /// Two things about the numbers rather than the encoding:
    ///
    /// * `A1` walks *down* two bytes an iteration, which is why the array
    ///   starts four kilobytes into RAM rather than at its base — it has to
    ///   stay mapped for as long as the run goes. It did not, the first time,
    ///   and every block faulted into a zeroed vector table. That is why the
    ///   test asserts the fault count rather than only printing it.
    /// * the counter is $7FFF so the run **never reaches the `bra *`**. A
    ///   one-instruction block executed for the rest of the quantum would be
    ///   most of the sample, and the mean instructions per block would then be
    ///   a measurement of the idle loop.
    fn firmware() -> Vec<u8> {
        let mut image = vec![0u8; 0x0500];
        image[0..4].copy_from_slice(&0x0020_0000u32.to_be_bytes());
        image[4..8].copy_from_slice(&0x0000_0400u32.to_be_bytes());
        let caller: &[u16] = &[
            0x2e7c, 0x0020, 0x0000, // move.l #$00200000,a7
            0x227c, 0x0010, 0x1000, // move.l #$00101000,a1
            0x303c, 0x7fff, // move.w #$7fff,d0
            0x4eb9, 0x0000, 0x0420, // jsr $00000420
            0x51c8, 0xfff8, // dbf d0,*-6
            0x60fe, // bra *
        ];
        let callee: &[u16] = &[
            0x4e56, 0xfffc, // link a6,#-4
            0x2f02, // move.l d2,-(a7)
            0x2411, // move.l (a1),d2
            0xd481, // add.l d1,d2
            0xe58a, // lsl.l #2,d2
            0x2482, // move.l d2,(a1)
            0x3221, // move.w -(a1),d1
            0xd269, 0x0002, // add.w 2(a1),d1
            0x0c41, 0x1234, // cmpi.w #$1234,d1
            0x6702, // beq *+4
            0x5341, // subq.w #1,d1
            0x241f, // move.l (a7)+,d2
            0x4e5e, // unlk a6
            0x4e75, // rts
        ];
        for (base, code) in [(0x0400usize, caller), (0x0420, callee)] {
            for (i, word) in code.iter().enumerate() {
                let at = base + 2 * i;
                image[at..at + 2].copy_from_slice(&word.to_be_bytes());
            }
        }
        image
    }

    /// The board with `engine` as its core's execution engine, and a handle on
    /// the core.
    ///
    /// The handle comes from replacing the class's constructor, which is the
    /// only route from a built machine to a concrete core: `Device` has no
    /// `Any` in its supertrait chain (`host::trace`, *How it gets hold of a
    /// processor*).
    pub(crate) fn boot(engine: &str) -> (Machine, Arc<M68k>) {
        let entry = catalog::machine("m68k-mini").expect("this build ships m68k-mini");
        let cores: Arc<Captured<M68k>> = Arc::new(Captured::new());
        let kept = Arc::clone(&cores);
        let mut options = catalog::build_options().expect("the catalog agrees with itself");
        options.bindings.replace("cpu.m68k", move |props| {
            let cpu = Arc::new(M68k::from_props(props)?);
            kept.push(&cpu);
            Ok(cpu)
        });
        options.realize.media.insert("firmware", firmware());
        options
            .resolve
            .params
            .push((String::from("engine"), String::from(engine)));
        let registry = catalog::registry().expect("a registry");
        let machine = rsemu::machine::build(entry.name, entry.source, &registry, &options)
            .unwrap_or_else(|e| panic!("the board does not realize with engine={engine}: {e}"));
        let cpu = cores.last().expect("the binding captured the processor");
        (machine, cpu)
    }

    /// Run `m` for `ms` milliseconds of virtual time.
    pub(crate) fn advance(m: &mut Machine, ms: u64) {
        m.run_for(GlobalTime::from_nanos(ms * 1_000_000))
            .expect("it runs");
    }
}

/// The instrument on the board that needs no media, with the interpreter's
/// answer beside it.
///
/// Synthetic code, and said so: the assertion is about the *instrument* —
/// that it counts, that its two closures hold, and that a `JSR` shows up
/// under `stores` rather than under `gap`. The rate itself is quoted in
/// `docs/cpu/m68k.md` only for the two ROMs.
#[test]
#[cfg(feature = "machine-m68k-mini")]
fn the_mini_board_reports_a_rate_and_the_instrument_closes() {
    let (mut ir, cpu) = mini::boot("ir");
    let (mut interp, _) = mini::boot("interp");
    for ms in 1..=8 {
        mini::advance(&mut ir, 1);
        mini::advance(&mut interp, 1);
        let want = interp.state_hash().expect("a deterministic machine hashes");
        let got = ir.state_hash().expect("a deterministic machine hashes");
        assert_eq!(want, got, "ms {ms}: the two engines parted company");
    }

    let stats = cpu.ir_stats().expect("a translated core keeps statistics");
    let declines = cpu.ir_declines().expect("and a histogram");
    let rate = report("m68k-mini (synthetic)", stats, &declines);

    assert!(stats.executed > 0, "blocks ran: {stats:?}");
    assert!(rate > 0, "something was lifted: {stats:?}");
    // The program has to *work*, or the mix being measured is the mix of a
    // zeroed vector table. This is the assertion that would have caught the
    // first version of this firmware, whose `-(A1)` walked off the bottom of
    // RAM and whose every block therefore faulted.
    assert!(!cpu.is_halted(), "the processor double-faulted: {stats:?}");
    assert_eq!(cpu.bus_faults().0, 0, "an access faulted: {stats:?}");
    assert_eq!(stats.faults, 0, "and no block took one: {stats:?}");
    // The `JSR` in the loop is declined, and it is declined for
    // restartability rather than for want of a lowering. That is the claim
    // `docs/cpu/m68k.md` makes and the one a later change to `classify` could
    // silently move.
    assert!(
        declines
            .iter()
            .any(|r| r.reason == Decline::STORES && r.what == "JSR" && r.count > 0),
        "the `JSR` in the loop is counted under `stores`: {declines:#?}"
    );
}

// ---------------------------------------------------------------------------
// The Macintosh boards: Apple's own ROM, read in place
// ---------------------------------------------------------------------------

#[cfg(any(feature = "machine-mac-plus", feature = "machine-mac-classic"))]
mod mac {
    use std::sync::Arc;

    use rsemu::core::Captured;
    use rsemu::core::clock::GlobalTime;
    use rsemu::cpu::m68k::M68k;
    use rsemu::machine::{Machine, catalog};

    /// Read `file` out of `RSEMU_MAC_ROM_DIR`, trimmed to `len`; `None`
    /// (having said why) when the variable or the file is not there.
    ///
    /// No checksum here: `tests/mac_plus.rs` and `tests/mac_classic.rs` are
    /// where a ROM image is validated, and this file's job is a rate rather
    /// than a second opinion about somebody's file.
    pub(crate) fn rom(board: &str, file: &str, len: usize) -> Option<Vec<u8>> {
        let Ok(dir) = std::env::var("RSEMU_MAC_ROM_DIR") else {
            println!(
                "{board}: set RSEMU_MAC_ROM_DIR to a directory holding {file} to measure the \
                 lift rate on a real Macintosh ROM; skipped."
            );
            return None;
        };
        let path = std::path::Path::new(&dir).join(file);
        let Ok(bytes) = std::fs::read(&path) else {
            println!("{board}: {} is not there; skipped", path.display());
            return None;
        };
        if bytes.len() < len {
            println!(
                "{board}: {} is {} bytes and the socket takes {len}; skipped",
                path.display(),
                bytes.len()
            );
            return None;
        }
        println!("{board}: measuring on {}", path.display());
        Some(bytes[..len].to_vec())
    }

    /// One board, and the handle on its processor.
    pub(crate) struct Board {
        pub(crate) machine: Machine,
        pub(crate) cpu: Arc<M68k>,
    }

    /// Build `board` around `image` with `engine` as the core's engine.
    ///
    /// The engine is set on the **core**, not in the machine file: `mac-plus`
    /// and `mac-classic` say `engine = "interp"` and belong to somebody else.
    /// `M68k::with_engine` is the same seam `engine = "ir"` reaches, so this
    /// measures the board as shipped with one property moved.
    pub(crate) fn board(
        board: &'static str,
        image: Vec<u8>,
        engine: rsemu::cpu::m68k::Engine,
    ) -> Board {
        let cores: Arc<Captured<M68k>> = Arc::new(Captured::new());
        let kept = Arc::clone(&cores);
        let mut options = catalog::build_options().expect("the catalog agrees with itself");
        options.bindings.replace("cpu.m68k", move |props| {
            let cpu = Arc::new(M68k::from_props(props)?.with_engine(engine));
            kept.push(&cpu);
            Ok(cpu)
        });
        rsemu::host::display::mac::capture::install(&mut options).expect("a capture table");
        options.realize.media.insert("macrom", image);
        // An empty drive, which is what `rsemu run` binds when nobody says
        // `--floppy`. `machine::realize` refuses a slot that is named and
        // unbound, so every bay this board has needs zero bytes bound for it
        // — including the Plus's SCSI port, where no bytes is an address
        // nobody answers at. `tests/mac_plus.rs` binds them the same way.
        options.realize.media.insert("floppy", Vec::new());
        if board == "mac-plus" {
            options.realize.media.insert("hd0", Vec::new());
        }
        let registry = catalog::registry().expect("a registry");
        let source = catalog::machine(board)
            .unwrap_or_else(|| panic!("this build ships {board}"))
            .source;
        let machine = rsemu::machine::build(board, source, &registry, &options)
            .unwrap_or_else(|e| panic!("{board} does not realize: {e}"));
        let cpu = cores.last().expect("the binding captured the processor");
        Board { machine, cpu }
    }

    /// Run both boards a virtual second at a time, comparing the whole
    /// machine's state hash after each, and return how many seconds agreed.
    ///
    /// The oracle runs beside the subject rather than after it so a
    /// divergence names the second it happened in.
    pub(crate) fn lockstep(board: &str, subject: &mut Board, oracle: &mut Board, seconds: u64) {
        for second in 1..=seconds {
            for b in [&mut *subject, &mut *oracle] {
                b.machine
                    .run_for(GlobalTime::from_nanos(1_000_000_000))
                    .expect("it runs");
            }
            let want = oracle
                .machine
                .state_hash()
                .expect("a deterministic machine hashes");
            let got = subject
                .machine
                .state_hash()
                .expect("a deterministic machine hashes");
            assert_eq!(
                want, got,
                "{board}: at {second} virtual second(s) the interpreter hashes to {want:#018x} \
                 and the translated engine to {got:#018x}"
            );
        }
        println!("{board}: {seconds} virtual seconds, {seconds} state hashes, 0 disagreements");
        assert!(
            !subject.cpu.is_halted(),
            "{board}: the processor double-faulted"
        );
    }
}

/// How long each ROM is run for.
///
/// Twelve virtual seconds is where `tests/mac_plus.rs` and
/// `tests/mac_classic.rs` take their goldens: past reset, past the memory
/// test, past the chime, through the device probes and into the insert-disk
/// loop. It is a whole boot rather than a slice of one, which is what makes
/// the instruction mix the ROM's own.
#[cfg(any(feature = "machine-mac-plus", feature = "machine-mac-classic"))]
const SECONDS: u64 = 12;

/// **The Macintosh Plus ROM**: Apple's own code, and how much of it is lifted.
#[test]
#[cfg(feature = "machine-mac-plus")]
fn the_macintosh_plus_rom_reports_a_rate() {
    use rsemu::cpu::m68k::Engine;

    let Some(image) = mac::rom("mac-plus", "Mac-Plus.ROM", 128 * 1024) else {
        return;
    };
    let mut ir = mac::board("mac-plus", image.clone(), Engine::Ir);
    let mut interp = mac::board("mac-plus", image, Engine::Interp);
    mac::lockstep("mac-plus", &mut ir, &mut interp, SECONDS);

    let stats = ir
        .cpu
        .ir_stats()
        .expect("a translated core keeps statistics");
    let declines = ir.cpu.ir_declines().expect("and a histogram");
    let rate = report(
        "mac-plus, a real 128 KiB Macintosh Plus ROM",
        stats,
        &declines,
    );
    assert!(
        stats.retired > 1_000_000,
        "a twelve-second boot retires millions of instructions in blocks: {stats:?}"
    );
    // A floor rather than a golden: this is a measurement, and the number it
    // produces belongs in `docs/cpu/m68k.md` where it can be explained. What
    // is asserted is that the translated engine is carrying the run at all —
    // the failure this would catch is a change that quietly sent most of a
    // real ROM down the fallback.
    assert!(
        rate >= 500,
        "the frontend carried {rate} per mille of a real ROM's instructions: {stats:?}"
    );
}

/// **The Macintosh Classic ROM**: the same, four years of ROM later.
#[test]
#[cfg(feature = "machine-mac-classic")]
fn the_macintosh_classic_rom_reports_a_rate() {
    use rsemu::cpu::m68k::Engine;

    let Some(image) = mac::rom("mac-classic", "Classic.ROM", 512 * 1024) else {
        return;
    };
    let mut ir = mac::board("mac-classic", image.clone(), Engine::Ir);
    let mut interp = mac::board("mac-classic", image, Engine::Interp);
    mac::lockstep("mac-classic", &mut ir, &mut interp, SECONDS);

    let stats = ir
        .cpu
        .ir_stats()
        .expect("a translated core keeps statistics");
    let declines = ir.cpu.ir_declines().expect("and a histogram");
    let rate = report(
        "mac-classic, a real 512 KiB Macintosh Classic ROM",
        stats,
        &declines,
    );
    assert!(
        stats.retired > 1_000_000,
        "a twelve-second boot retires millions of instructions in blocks: {stats:?}"
    );
    assert!(
        rate >= 500,
        "the frontend carried {rate} per mille of a real ROM's instructions: {stats:?}"
    );
}
